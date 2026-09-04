//! Bridge configuration.
//!
//! Until phase 010 `config/bridge_config.yaml` was never read: `serde` and `serde_yaml` were
//! declared and unused, `blueprint_map` did nothing, and every real setting came from an
//! environment variable. This module makes the file load, and makes every key in it do
//! something.
//!
//! # Precedence
//!
//! **Environment overrides file, file overrides default.** The environment is the more
//! specific source: the launch files set `CARLA_HOST` / `CARLA_PORT` / `SSV2_PORT` per run,
//! and a checked-in config must not silently win over what a launch explicitly asked for.

use eyre::{bail, Result, WrapErr};
use serde::Deserialize;
use std::collections::HashMap;
use std::path::Path;

fn default_carla_host() -> String {
    "localhost".to_string()
}
fn default_carla_port() -> u16 {
    2000
}
fn default_ssv2_port() -> u16 {
    5555
}
fn default_ego_role_name() -> String {
    "hero".to_string()
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct CarlaConfig {
    pub host: String,
    pub port: u16,
}

impl Default for CarlaConfig {
    fn default() -> Self {
        Self {
            host: default_carla_host(),
            port: default_carla_port(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct Ssv2Config {
    pub port: u16,
}

impl Default for Ssv2Config {
    fn default() -> Self {
        Self {
            port: default_ssv2_port(),
        }
    }
}

/// Where to announce upcoming despawns, and how long to wait for sensor bridges to react.
///
/// A vehicle's sensors belong to whichever bridge attached them, and destroying one that
/// is still being listened to leaves that client hammering a dead stream -- see
/// `sensor_release`. Announcing first lets it stop cleanly.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct SensorReleaseConfig {
    /// Set false to skip the announcement entirely (and pay the error storm).
    pub enabled: bool,
    /// Our PUB socket; sensor bridges SUB to it.
    pub notify_endpoint: String,
    /// Our PULL socket; sensor bridges PUSH acknowledgements to it.
    pub ack_endpoint: String,
    /// How long to hold a freshly spawned ego before letting the scenario start, waiting
    /// for its bridge to report localization seated on it. Zero disables the wait.
    ///
    /// Autoware routes within a couple of seconds of the spawn, from whatever pose it has,
    /// and on every run after the first that pose is the previous run's -- about 95 m away.
    /// Moving the estimate onto the new vehicle takes about 7 s of Autoware's own pipeline
    /// and cannot be shortened from either bridge, so the only way not to route from a
    /// stale pose is not to start yet. Generous by default because a cold stack, where
    /// Autoware initializes from GNSS rather than from a seed, is slower than a warm one.
    #[serde(default = "default_localization_warmup_s")]
    pub localization_warmup_s: f64,
    /// How long to wait for an acknowledgement before despawning anyway.
    ///
    /// Paid once per despawned entity that nobody acknowledges, so it trades scenario
    /// latency against a burst of server log. 300 ms is far longer than a local round
    /// trip and short enough not to matter to a scenario.
    pub timeout_ms: u64,
}

impl Default for SensorReleaseConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            notify_endpoint: "tcp://127.0.0.1:5556".to_string(),
            ack_endpoint: "tcp://127.0.0.1:5557".to_string(),
            timeout_ms: 300,
            localization_warmup_s: default_localization_warmup_s(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct EgoConfig {
    /// `role_name` given to the scenario ego.
    ///
    /// This is how `acb_bridge` finds the vehicle it serves, so it must match the
    /// `vehicle_name` parameter of the bridge in the ego's ROS domain.
    pub role_name: String,
}

impl Default for EgoConfig {
    fn default() -> Self {
        Self {
            role_name: default_ego_role_name(),
        }
    }
}

/// A pose in the ROS frame, as scenarios and Autoware use it.
#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct PoseConfig {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Heading in degrees.
    pub yaw: f64,
}

/// An Autoware instance driving its own vehicle, outside SSv2's model.
///
/// Background AVs exist because SSv2 supports exactly one ego -- it throws
/// `"Multiple egos in the simulation are unsupported yet."` -- and its concealer forks
/// Autoware into SSv2's own `ROS_DOMAIN_ID`. So additional Autoware stacks cannot be
/// scenario entities; they are ordinary CARLA vehicles that a separate Autoware drives.
///
/// **They are invisible to SSv2.** Its collision detection and conditions never see them.
/// See `docs/design/multi-instance-architecture.md`.
#[derive(Debug, Clone, Deserialize)]
pub struct BackgroundAv {
    /// CARLA `role_name`. Must match the `vehicle_name` of the `acb_bridge` serving it.
    pub role_name: String,
    /// ROS domain its Autoware runs in.
    ///
    /// Not used by this bridge -- launch owns the actual domain -- but recorded here so one
    /// file describes the whole run and the per-domain launch can read the same source.
    #[serde(default)]
    pub ros_domain_id: Option<u32>,
    /// CARLA blueprint to spawn.
    #[serde(default)]
    pub blueprint: Option<String>,
    /// Where to spawn, in the ROS frame.
    pub spawn_pose: PoseConfig,
    /// Where its pilot should route it. Consumed by that domain's pilot, not by this bridge.
    #[serde(default)]
    pub goal_pose: Option<PoseConfig>,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct BridgeConfig {
    pub carla: CarlaConfig,
    pub ssv2: Ssv2Config,
    pub ego: EgoConfig,
    /// SSv2 asset key to CARLA blueprint, for keys that are not blueprint names.
    pub blueprint_map: HashMap<String, String>,
    /// Map directory name to CARLA town, for maps not named after their town.
    pub map_alias: HashMap<String, String>,
    /// Additional Autoware instances. Empty means a plain single-ego run.
    pub background_avs: Vec<BackgroundAv>,
    /// Channel for telling sensor bridges to release their sensors before a despawn.
    pub sensor_release: SensorReleaseConfig,
    /// How many CARLA ticks to run per SSv2 frame. See `default_substeps`.
    #[serde(default = "default_substeps")]
    pub substeps: u32,
}

/// Off, because under a managed ego it cannot work -- see `warm_up_localization`.
///
/// Waiting here holds SSv2 at the spawn, and under a managed ego SSv2 is what publishes
/// `/clock`. Autoware runs on simulation time, so a stopped clock stops NDT and the EKF
/// with it, and the estimate this waits for can never arrive. Measured exactly that way:
/// the bridge reported the estimate seated **six seconds after** a 30 s wait gave up, on
/// the clock SSv2 resumed once it was answered. A 90 s wait then timed out in full.
///
/// It is left in place, and defaults to off rather than being deleted, because it is
/// correct for an *unmanaged* ego, where acb_bridge owns `/clock` and the world keeps
/// running while this waits. Set a positive number there.
fn default_localization_warmup_s() -> f64 {
    0.0
}

/// One CARLA tick per SSv2 frame -- the historical behaviour.
///
/// SSv2 asks for a step time (0.1 s in practice) and the bridge ticks CARLA once per frame
/// at that delta. A control command therefore waits up to a full 100 ms for the next tick
/// before it can take effect, because Autoware publishes at 20 Hz on its own clock and
/// nothing synchronises the two. Raising this divides the delta and ticks that many times
/// per frame, so SSv2's timeline is untouched while control is picked up more often: at 2,
/// the worst-case wait halves to 50 ms.
///
/// It is not free. Every tick fires the sensors that publish per frame rather than on a
/// timer, and the ego LiDAR is configured that way (`sensor_tick: 0.0`), so its point
/// clouds arrive twice as often. That happens to suit it -- `rotation_frequency` is 20 Hz
/// against a 10 Hz frame, so today each scan sweeps two rotations and at 2 substeps it
/// sweeps exactly one, which is what the config comment asks for -- but it doubles the
/// point cloud rate into Autoware. Physics also runs at the finer delta, which is more
/// accurate and more expensive.
///
/// Default 1 so nothing changes unless asked. See acb `docs/issues/016`.
fn default_substeps() -> u32 {
    1
}

impl BridgeConfig {
    /// Load configuration, or use defaults when the file is absent.
    ///
    /// A missing file is fine -- every key has a default. A *malformed* file is fatal:
    /// silently falling back to defaults would run the scenario with settings the operator
    /// believes they changed.
    pub fn load_or_default(path: &Path) -> Result<Self> {
        let config = if path.exists() {
            let text = std::fs::read_to_string(path)
                .wrap_err_with(|| format!("read {}", path.display()))?;
            let config: BridgeConfig = serde_yaml::from_str(&text)
                .wrap_err_with(|| format!("parse {}", path.display()))?;
            tracing::info!("Loaded configuration from {}", path.display());
            config
        } else {
            // A warning, not an info line: the defaults have no background AVs and no
            // blueprint or map aliases, so a run started from the wrong directory looks
            // healthy while quietly being a different run than the one configured.
            tracing::warn!(
                "No configuration at {}; using defaults (no background AVs, no blueprint \
                 or map aliases). Set CSB_CONFIG_DIR if this is not what you meant.",
                path.display()
            );
            BridgeConfig::default()
        };

        config.validate()?;
        Ok(config)
    }

    /// Apply environment overrides. See the module docs for why the environment wins.
    pub fn apply_env_overrides(&mut self) {
        if let Ok(host) = std::env::var("CARLA_HOST") {
            if !host.is_empty() {
                self.carla.host = host;
            }
        }
        if let Some(port) = env_port("CARLA_PORT") {
            self.carla.port = port;
        }
        if let Some(port) = env_port("SSV2_PORT") {
            self.ssv2.port = port;
        }
    }

    /// Reject configurations that would misbehave in ways that are hard to diagnose later.
    fn validate(&self) -> Result<()> {
        // Role names identify vehicles to acb_bridge. A duplicate means two bridges race for
        // one vehicle, or one bridge attaches sensors to a vehicle meant for another --
        // discovered as confusing runtime behaviour rather than as a config error.
        let mut seen: HashMap<&str, &str> = HashMap::new();
        seen.insert(self.ego.role_name.as_str(), "ego");

        for av in &self.background_avs {
            if av.role_name.trim().is_empty() {
                bail!("a background AV has an empty role_name");
            }
            if let Some(previous) = seen.insert(av.role_name.as_str(), "background AV") {
                bail!(
                    "role_name '{}' is used by more than one vehicle (already used by the \
                     {previous}); acb_bridge finds vehicles by role_name, so they must be unique",
                    av.role_name
                );
            }
        }

        Ok(())
    }

    /// CARLA blueprint for an SSv2 asset key, if the config maps it.
    pub fn blueprint_for(&self, asset_key: &str) -> Option<&str> {
        self.blueprint_map.get(asset_key).map(String::as_str)
    }
}

fn env_port(name: &str) -> Option<u16> {
    let raw = std::env::var(name).ok()?;
    match raw.parse() {
        Ok(port) => Some(port),
        Err(e) => {
            tracing::warn!("Ignoring {name}='{raw}': not a valid port ({e})");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_config_is_all_defaults() {
        let config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert_eq!(config.carla.host, "localhost");
        assert_eq!(config.carla.port, 2000);
        assert_eq!(config.ssv2.port, 5555);
        assert_eq!(config.ego.role_name, "hero");
        assert!(config.background_avs.is_empty());
        assert!(config.blueprint_map.is_empty());
    }

    /// A partial file must not blank out the keys it does not mention.
    #[test]
    fn unspecified_sections_keep_their_defaults() {
        let config: BridgeConfig = serde_yaml::from_str("carla:\n  port: 3000\n").expect("parses");
        assert_eq!(config.carla.port, 3000);
        assert_eq!(config.carla.host, "localhost", "host must keep its default");
        assert_eq!(config.ssv2.port, 5555);
    }

    #[test]
    fn blueprint_map_resolves_asset_keys() {
        let config: BridgeConfig =
            serde_yaml::from_str("blueprint_map:\n  sample_vehicle: vehicle.tesla.model3\n")
                .expect("parses");
        assert_eq!(
            config.blueprint_for("sample_vehicle"),
            Some("vehicle.tesla.model3")
        );
        assert_eq!(config.blueprint_for("unknown"), None);
    }

    #[test]
    fn background_avs_parse_with_their_poses() {
        let yaml = r#"
background_avs:
  - role_name: bg_av_1
    ros_domain_id: 1
    blueprint: vehicle.audi.tt
    spawn_pose: {x: 10.0, y: -20.0, z: 0.5, yaw: 90.0}
    goal_pose: {x: 100.0, y: -20.0, z: 0.0, yaw: 0.0}
"#;
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        assert_eq!(config.background_avs.len(), 1);

        let av = &config.background_avs[0];
        assert_eq!(av.role_name, "bg_av_1");
        assert_eq!(av.ros_domain_id, Some(1));
        assert_eq!(av.blueprint.as_deref(), Some("vehicle.audi.tt"));
        assert_eq!(av.spawn_pose.x, 10.0);
        assert_eq!(av.spawn_pose.y, -20.0);
        assert_eq!(av.spawn_pose.yaw, 90.0);
        assert_eq!(av.goal_pose.map(|p| p.x), Some(100.0));
    }

    /// Only role_name and spawn_pose are required; the rest have sensible defaults.
    #[test]
    fn a_minimal_background_av_parses() {
        let yaml = "background_avs:\n  - role_name: bg\n    spawn_pose: {x: 1.0}\n";
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        let av = &config.background_avs[0];
        assert_eq!(av.blueprint, None);
        assert_eq!(av.ros_domain_id, None);
        assert_eq!(av.spawn_pose.y, 0.0);
    }

    /// acb_bridge finds its vehicle by role_name, so a duplicate makes two bridges fight
    /// over one vehicle. Catch it at startup, not as puzzling runtime behaviour.
    #[test]
    fn duplicate_role_names_are_rejected() {
        let yaml = r#"
background_avs:
  - role_name: dup
    spawn_pose: {x: 0.0}
  - role_name: dup
    spawn_pose: {x: 1.0}
"#;
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        let err = config.validate().expect_err("duplicate must be rejected");
        assert!(err.to_string().contains("dup"), "{err}");
    }

    /// The ego's role name shares the namespace with the background AVs.
    #[test]
    fn a_background_av_may_not_take_the_egos_role_name() {
        let yaml = r#"
ego:
  role_name: hero
background_avs:
  - role_name: hero
    spawn_pose: {x: 0.0}
"#;
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        let err = config
            .validate()
            .expect_err("clash with ego must be rejected");
        assert!(err.to_string().contains("ego"), "{err}");
    }

    #[test]
    fn an_empty_role_name_is_rejected() {
        let yaml = "background_avs:\n  - role_name: '  '\n    spawn_pose: {x: 0.0}\n";
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        assert!(config.validate().is_err());
    }

    #[test]
    fn a_valid_config_passes_validation() {
        let yaml = r#"
ego:
  role_name: hero
background_avs:
  - role_name: bg_av_1
    spawn_pose: {x: 0.0}
  - role_name: bg_av_2
    spawn_pose: {x: 1.0}
"#;
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        assert!(config.validate().is_ok());
    }

    #[test]
    fn a_custom_ego_role_name_is_honoured() {
        let config: BridgeConfig =
            serde_yaml::from_str("ego:\n  role_name: scenario_ego\n").expect("parses");
        assert_eq!(config.ego.role_name, "scenario_ego");
    }

    #[test]
    fn map_aliases_parse() {
        let config: BridgeConfig =
            serde_yaml::from_str("map_alias:\n  kashiwanoha: Town01\n").expect("parses");
        assert_eq!(
            config.map_alias.get("kashiwanoha").map(String::as_str),
            Some("Town01")
        );
    }

    /// A malformed file must not silently fall back to defaults -- the operator believes
    /// their settings are in effect.
    #[test]
    fn malformed_yaml_is_an_error() {
        assert!(serde_yaml::from_str::<BridgeConfig>("carla: [not, a, mapping]").is_err());
    }
}
