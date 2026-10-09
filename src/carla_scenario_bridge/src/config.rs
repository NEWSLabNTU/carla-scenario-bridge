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
    /// How long `Initialize` keeps trying to reach a CARLA that is gone -- restarting,
    /// typically, which takes ~3 min before it serves RPC -- before it fails the scenario.
    pub reconnect_wait_seconds: u64,
}

impl Default for CarlaConfig {
    fn default() -> Self {
        Self {
            host: default_carla_host(),
            port: default_carla_port(),
            reconnect_wait_seconds: 240,
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

/// CARLA's Traffic Manager, which drives the entities whose controller is
/// `simulator_autopilot` (see `autopilot`).
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct TrafficManagerConfig {
    /// TM port. csb runs the TM in its own process (LibCarla starts it on first use); a TM
    /// another client already serves on this port is used instead.
    pub port: u16,
    /// Seed for TM's random choices (lane changes, roaming turns), set at every
    /// Initialize, so a scenario's simulator-driven traffic repeats run to run.
    pub seed: u64,
}

impl Default for TrafficManagerConfig {
    fn default() -> Self {
        Self {
            port: 8000,
            seed: 2017,
        }
    }
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
    /// Removed (roadmap 017): background AVs are scenario entities whose controller is
    /// `agent`. Kept only so a file that still declares them is refused with that message
    /// instead of being silently ignored -- see [`BridgeConfig::validate`].
    #[serde(rename = "background_avs")]
    pub removed_background_avs: Option<serde_yaml::Value>,
    /// Channel for telling sensor bridges to release their sensors before a despawn.
    pub sensor_release: SensorReleaseConfig,
    /// How many CARLA ticks to run per SSv2 frame. See `default_substeps`.
    #[serde(default = "default_substeps")]
    pub substeps: u32,
    /// Target CARLA tick length in seconds. When set, the ticks per SSv2 frame follow the
    /// frame's step time -- `round(step_time / carla_tick_seconds)`, at least 1 -- and
    /// `substeps` is ignored. See [`ticks_per_frame`].
    pub carla_tick_seconds: Option<f64>,
    /// Attach a `sensor.other.collision` to the ego and log what CARLA's physics thinks
    /// it hit (roadmap 014, gap 3). Diagnostic only: never changes SSv2's verdict. Unset
    /// means on -- read it through [`BridgeConfig::collision_monitor_enabled`].
    pub collision_monitor: Option<bool>,
    /// Traffic Manager for simulator-driven entities.
    pub traffic_manager: TrafficManagerConfig,
    /// The agent relay that entities with controller `agent` are driven through
    /// (`tcp://host:port`; empty = the default). csb connects to it as a commander (see
    /// `agent_link`). Read it through [`BridgeConfig::agent_relay`].
    pub agent_relay: String,
    /// CARLA weather preset (`ClearNoon`, ...) applied on connect and after every map load;
    /// empty leaves CARLA's (roadmap 018). `/carla/set_weather` replaces it at run time.
    pub weather: String,
}

const DEFAULT_AGENT_RELAY: &str = "tcp://localhost:5560";

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

/// CARLA ticks per SSv2 frame of `step_time`.
///
/// The CARLA tick is what the sensors are tuned to, not the SSv2 frame: the ego LiDAR fires
/// once per tick (`sensor_tick: 0.0`) with `rotation_frequency` 20 Hz, so it sweeps exactly
/// one rotation only at a 0.05 s tick, and the IMU's spacing against gyro_odometer's 0.2 s
/// tolerance is the tick too. A fixed `substeps` ties that to SSv2's frame rate: 2 is right
/// at 10 Hz and gives a 0.025 s tick -- half-rotation scans at 40 Hz -- at 20 Hz. With
/// `carla_tick_seconds` the tick stays put and the frame rate only decides how often SSv2
/// (and the /clock it publishes) advances.
pub fn ticks_per_frame(step_time: f64, substeps: u32, carla_tick_seconds: Option<f64>) -> u32 {
    match carla_tick_seconds {
        Some(tick) if tick > 0.0 && step_time > 0.0 => (step_time / tick).round().max(1.0) as u32,
        _ => substeps.max(1),
    }
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
            // A warning, not an info line: the defaults have no blueprint or map aliases,
            // so a run started from the wrong directory looks healthy while quietly being a
            // different run than the one configured.
            tracing::warn!(
                "No configuration at {}; using defaults (no blueprint or map aliases). Pass config_file (ROS parameter) or set CSB_CONFIG_DIR if this is not what you meant.",
                path.display()
            );
            BridgeConfig::default()
        };

        config.validate()?;
        Ok(config)
    }

    /// Apply ROS parameters (`carla_host`, `carla_port`, `ssv2_port`, `agent_relay`,
    /// `reconnect_wait_seconds`, `weather`) over the file and the environment: a launch file's explicit
    /// value is the most specific statement of intent. A malformed number is an error, not
    /// a silent default.
    pub fn apply_ros_params(&mut self, params: &crate::ros_args::RosParams) -> Result<()> {
        if let Some(host) = params.get("carla_host").filter(|h| !h.is_empty()) {
            self.carla.host = host.to_string();
        }
        let num = |name: &str| -> Result<Option<u64>> {
            params
                .get(name)
                .filter(|v| !v.is_empty())
                .map(|v| {
                    v.parse::<u64>()
                        .map_err(|e| eyre::eyre!("parameter {name}:={v}: {e}"))
                })
                .transpose()
        };
        if let Some(port) = num("carla_port")? {
            self.carla.port = u16::try_from(port).map_err(|e| eyre::eyre!("carla_port: {e}"))?;
        }
        if let Some(port) = num("ssv2_port")? {
            self.ssv2.port = u16::try_from(port).map_err(|e| eyre::eyre!("ssv2_port: {e}"))?;
        }
        if let Some(wait) = num("reconnect_wait_seconds")? {
            self.carla.reconnect_wait_seconds = wait;
        }
        if let Some(relay) = params.get("agent_relay").filter(|v| !v.is_empty()) {
            self.agent_relay = relay.to_string();
        }
        if let Some(weather) = params.get("weather").filter(|v| !v.trim().is_empty()) {
            crate::weather::preset_or_err(weather.trim())
                .map_err(|e| eyre::eyre!("parameter weather:={weather}: {e}"))?;
            self.weather = weather.trim().to_string();
        }
        if params.get("background_avs").is_some_and(|v| !v.is_empty()) {
            bail!("{}", REMOVED_BACKGROUND_AVS);
        }
        Ok(())
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
        if std::env::var("CSB_BACKGROUND_AVS").is_ok_and(|v| !v.trim().is_empty()) {
            tracing::warn!("CSB_BACKGROUND_AVS is ignored: {REMOVED_BACKGROUND_AVS}");
        }
    }

    /// Reject configurations that would misbehave in ways that are hard to diagnose later.
    fn validate(&self) -> Result<()> {
        if self.removed_background_avs.is_some() {
            bail!("{}", REMOVED_BACKGROUND_AVS);
        }
        if self.ego.role_name.trim().is_empty() {
            bail!("ego.role_name is empty; acb_bridge finds the ego by it");
        }
        crate::agent_link::parse_relay_address(self.agent_relay()).wrap_err("agent_relay")?;
        if !self.weather.trim().is_empty() {
            crate::weather::preset_or_err(self.weather.trim())
                .map_err(|e| eyre::eyre!("weather: {e}"))?;
        }
        Ok(())
    }

    /// The agent relay address, defaulted.
    pub fn agent_relay(&self) -> &str {
        if self.agent_relay.trim().is_empty() {
            DEFAULT_AGENT_RELAY
        } else {
            self.agent_relay.trim()
        }
    }

    /// Whether to run the ego collision monitor. Defaults to on: it costs one sensor that
    /// only fires on contact, and the disagreement it exposes is otherwise invisible.
    pub fn collision_monitor_enabled(&self) -> bool {
        self.collision_monitor.unwrap_or(true)
    }

    /// CARLA blueprint for an SSv2 asset key, if the config maps it.
    pub fn blueprint_for(&self, asset_key: &str) -> Option<&str> {
        self.blueprint_map.get(asset_key).map(String::as_str)
    }
}

/// Why `background_avs` (file key, ROS parameter, `CSB_BACKGROUND_AVS`) is refused.
pub const REMOVED_BACKGROUND_AVS: &str = "background_avs was removed (roadmap 017): a \
    background AV is now a scenario entity whose controller is `agent` -- the bridge spawns \
    it with role_name = the entity name and drives it through the agent that registers with \
    the agent relay under that name (see docs/design/scenario-authoring.md). Delete the \
    background_avs section from bridge_config.yaml.";

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
        assert_eq!(config.agent_relay(), "tcp://localhost:5560");
        assert!(config.validate().is_ok());
        assert!(BridgeConfig::default().validate().is_ok());
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

    /// Background AVs moved into the scenario (controller `agent`); a file that still
    /// declares them is refused, not silently ignored.
    #[test]
    fn a_background_avs_section_is_refused_with_the_way_forward() {
        let yaml = "background_avs:\n  - role_name: bg_av_1\n    spawn_pose: {x: 1.0}\n";
        let config: BridgeConfig = serde_yaml::from_str(yaml).expect("parses");
        let err = config.validate().expect_err("removed key must be refused");
        assert!(err.to_string().contains("controller is `agent`"), "{err}");
    }

    #[test]
    fn the_agent_relay_is_configurable_and_checked() {
        let config: BridgeConfig =
            serde_yaml::from_str("agent_relay: tcp://sim:6000\n").expect("parses");
        assert_eq!(config.agent_relay(), "tcp://sim:6000");
        assert!(config.validate().is_ok());
        let config: BridgeConfig =
            serde_yaml::from_str("agent_relay: http://sim\n").expect("parses");
        assert!(config.validate().is_err());

        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        let params = crate::ros_args::RosParams::from_args(
            ["--ros-args", "-p", "agent_relay:=tcp://h:1"]
                .iter()
                .map(|s| s.to_string()),
        )
        .expect("parses");
        config.apply_ros_params(&params).expect("applies");
        assert_eq!(config.agent_relay(), "tcp://h:1");
    }

    #[test]
    fn weather_comes_from_file_or_parameter_and_must_be_a_preset() {
        let params = |v: &str| {
            crate::ros_args::RosParams::from_args([
                "--ros-args".to_string(),
                "-p".to_string(),
                format!("weather:={v}"),
            ])
            .expect("parses")
        };
        // Unset: leave CARLA's.
        let config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert_eq!(config.weather, "");
        assert!(config.validate().is_ok());

        let mut config: BridgeConfig =
            serde_yaml::from_str("weather: CloudyNoon\n").expect("parses");
        assert!(config.validate().is_ok());
        // An empty parameter (the launch default) keeps the file's.
        config.apply_ros_params(&params("")).expect("applies");
        assert_eq!(config.weather, "CloudyNoon");
        config
            .apply_ros_params(&params("ClearNoon"))
            .expect("applies");
        assert_eq!(config.weather, "ClearNoon");

        let err = config.apply_ros_params(&params("Sunny")).unwrap_err();
        assert!(
            format!("{err}").contains("unknown weather preset 'Sunny'"),
            "{err}"
        );
        let config: BridgeConfig = serde_yaml::from_str("weather: Sunny\n").expect("parses");
        assert!(config.validate().is_err());
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

    #[test]
    fn the_collision_monitor_defaults_on_and_can_be_turned_off() {
        let config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert!(config.collision_monitor_enabled());
        assert!(BridgeConfig::default().collision_monitor_enabled());
        let config: BridgeConfig =
            serde_yaml::from_str("collision_monitor: false\n").expect("parses");
        assert!(!config.collision_monitor_enabled());
    }

    /// A malformed file must not silently fall back to defaults -- the operator believes
    /// their settings are in effect.
    #[test]
    fn malformed_yaml_is_an_error() {
        assert!(serde_yaml::from_str::<BridgeConfig>("carla: [not, a, mapping]").is_err());
    }
}
