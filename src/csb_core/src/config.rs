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
            reconnect_wait_seconds: 120,
        }
    }
}

/// The world server the bridge drives (roadmaps 037, 046): whichever simulator answers there.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct WorldConfig {
    /// The world server's REP socket (the Isaac one binds 127.0.0.1:5590 by default, the
    /// CARLA one 127.0.0.1:5591; `WORLD_PORT`).
    pub endpoint: String,
    /// How long a request may take before the server counts as hung, s: the bridge then
    /// exits for its supervisor to restart (034).
    pub request_timeout_seconds: u64,
    /// The same for `LoadWorld`, which may open a large USD stage, and for an ego's `Spawn`,
    /// which waits for its sensors' first frames (038).
    pub load_timeout_seconds: u64,
    /// Ask the server to render once per frame (Isaac: off by default, a render costs 3-5 ms
    /// of the frame, 037; on to watch the world on a display). A server that renders anyway
    /// ignores it.
    pub render: bool,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            endpoint: "tcp://127.0.0.1:5590".to_string(),
            request_timeout_seconds: 10,
            load_timeout_seconds: 120,
            render: false,
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
    /// Serve the TM for other clients too: start it with synchronous mode and keep it for
    /// the bridge's lifetime, even with no `simulator_autopilot` entity. Another process
    /// that drives vehicles through the TM (a harness following csb's frames) connects to
    /// `port` as a TM-Client; csb's `world.tick()` steps the TM once per tick, which a TM
    /// hosted by a process that never ticks cannot have (`CSB_TM_SERVE=1` overrides).
    pub serve: bool,
}

impl Default for TrafficManagerConfig {
    fn default() -> Self {
        Self {
            port: 8000,
            seed: 2017,
            serve: false,
        }
    }
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct BridgeConfig {
    /// Removed (roadmap 046): the bridge drives whichever world server answers at
    /// `world.endpoint`. Kept so a file that still names it is refused with that message.
    #[serde(rename = "backend")]
    pub removed_backend: Option<serde_yaml::Value>,
    /// Renamed `world` (roadmap 046); refused with that message.
    #[serde(rename = "isaac")]
    pub removed_isaac: Option<serde_yaml::Value>,
    pub carla: CarlaConfig,
    /// The world server the bridge drives (roadmap 046).
    pub world: WorldConfig,
    pub ssv2: Ssv2Config,
    pub ego: EgoConfig,
    /// SSv2 asset key to CARLA blueprint, for keys that are not blueprint names.
    pub blueprint_map: HashMap<String, String>,
    /// CARLA `color` attribute ("R,G,B", 0-255) per entity, keyed by the role_name it
    /// spawns with (the ego's `hero`, an agent's entity name). An entity not listed, or a
    /// blueprint without the attribute, spawns in the blueprint's default colour.
    pub actor_colors: HashMap<String, String>,
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
    /// Where each ego run's CARLA contacts are written for `physical_verdict` (roadmap 021):
    /// unset = `$ROS_HOME/csb/contacts`, `off` = no reports, else that directory. Read it
    /// through [`BridgeConfig::contact_report_dir`].
    pub contact_report_dir: Option<String>,
    /// Give every tracked NPC vehicle a collision sensor of its own, so its contacts with
    /// the static world and other NPCs reach the gap trace and the oracle (roadmap 032 step
    /// 5). Diagnostic only, like the ego's. Unset means on -- read it through
    /// [`BridgeConfig::npc_contact_sensors_enabled`]. Needs `collision_monitor`.
    pub npc_contact_sensors: Option<bool>,
    /// Traffic Manager for simulator-driven entities.
    pub traffic_manager: TrafficManagerConfig,
    /// The agent relay that entities with controller `agent` are driven through
    /// (`tcp://host:port`; empty = the default). csb connects to it as a commander (see
    /// `agent_link`). Read it through [`BridgeConfig::agent_relay`].
    pub agent_relay: String,
    /// CARLA weather preset (`ClearNoon`, ...) applied on connect and after every map load;
    /// empty leaves CARLA's (roadmap 018). `/carla/set_weather` replaces it at run time.
    pub weather: String,
    /// How SSv2-driven NPC vehicles and pedestrians move in CARLA (roadmap 023).
    pub npc_motion: NpcMotion,
    /// Where each episode's gap trace is written for the oracle (roadmap 024); empty = no
    /// trace (the default). Read it through [`BridgeConfig::gap_trace_dir`].
    pub gap_trace_dir: String,
    /// The tick gate's UDP address (`host:port`, [`crate::tick_gate`]): a process that
    /// follows csb's frames and joins the gate is waited for before every tick. Empty (the
    /// default) = no gate. `CSB_TICK_GATE` overrides.
    pub tick_gate: String,
    /// How long one tick waits for the gate's follower at most (ms); unset = 1000.
    pub tick_gate_timeout_ms: Option<u64>,
}

/// How SSv2-driven NPC vehicles and pedestrians move in CARLA (roadmap 023).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NpcMotion {
    /// Physics off, teleported to SSv2's pose every frame: exact, but CARLA reports them at
    /// 0 m/s, they cannot be pushed, and walkers do not animate. The opt-in fallback.
    Kinematic,
    /// Physics on, driven toward SSv2's pose with a velocity each frame and snapped back
    /// past 0.5 m / 5 deg (`npc_tracking`). The default since roadmap 023 step 6.
    #[default]
    Tracked,
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

/// Whose configuration file a [`BridgeConfig`] is read from (roadmap 046): the bridge's and
/// the CARLA world server's share the struct, and each refuses the other's keys.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigOwner {
    Bridge,
    CarlaWorld,
}

/// The keys of the bridge's `bridge_config.yaml`.
pub const BRIDGE_KEYS: &[&str] = &[
    "world",
    "ssv2",
    "ego",
    "blueprint_map",
    "agent_relay",
    "weather",
    "npc_motion",
    "gap_trace_dir",
    "contact_report_dir",
];

/// The keys of the CARLA world server's `carla_world.yaml`.
pub const CARLA_WORLD_KEYS: &[&str] = &[
    "carla",
    "ssv2",
    "ego",
    "blueprint_map",
    "actor_colors",
    "map_alias",
    "sensor_release",
    "collision_monitor",
    "npc_contact_sensors",
    "substeps",
    "carla_tick_seconds",
    "traffic_manager",
    "tick_gate",
    "tick_gate_timeout_ms",
];

impl ConfigOwner {
    fn keys(self) -> &'static [&'static str] {
        match self {
            Self::Bridge => BRIDGE_KEYS,
            Self::CarlaWorld => CARLA_WORLD_KEYS,
        }
    }

    fn file(self) -> &'static str {
        match self {
            Self::Bridge => "the bridge's bridge_config.yaml",
            Self::CarlaWorld => "the CARLA world server's carla_world.yaml",
        }
    }

    fn other(self) -> Self {
        match self {
            Self::Bridge => Self::CarlaWorld,
            Self::CarlaWorld => Self::Bridge,
        }
    }
}

/// A key of `text` that belongs in the other owner's file, if any (roadmap 046 split them).
pub fn misplaced_key(text: &str, owner: ConfigOwner) -> Option<String> {
    let value: serde_yaml::Value = serde_yaml::from_str(text).ok()?;
    let map = value.as_mapping()?;
    map.keys().filter_map(|k| k.as_str()).find_map(|k| {
        (!owner.keys().contains(&k) && owner.other().keys().contains(&k)).then(|| {
            format!(
                "{k} belongs in {}, not {} (roadmap 046: the bridge and the CARLA world \
                 server read their own)",
                owner.other().file(),
                owner.file()
            )
        })
    })
}

impl BridgeConfig {
    /// [`BridgeConfig::load_or_default`] for `owner`'s file: a key that belongs in the other
    /// file is refused, naming where it went.
    pub fn load_for(path: &Path, owner: ConfigOwner) -> Result<Self> {
        if path.exists() {
            let text = std::fs::read_to_string(path)
                .wrap_err_with(|| format!("read {}", path.display()))?;
            if let Some(why) = misplaced_key(&text, owner) {
                bail!("{}: {why}", path.display());
            }
        }
        Self::load_or_default(path)
    }

    /// The Traffic Manager's port follows CARLA's (CARLA_PORT + 6000) unless the file names
    /// another than the default: a second CARLA on a host needs its own TM too.
    fn follow_carla_port(&mut self) {
        if self.traffic_manager.port == TrafficManagerConfig::default().port
            && self.carla.port != default_carla_port()
        {
            self.traffic_manager.port = self.carla.port.saturating_add(6000);
        }
    }

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
                "No configuration at {}; using defaults (no blueprint or map aliases). Pass the ROS parameter config_file if this is not what you meant.",
                path.display()
            );
            BridgeConfig::default()
        };

        config.validate()?;
        Ok(config)
    }

    /// Apply ROS parameters (`world_endpoint`, `world_render`,
    /// `world_request_timeout_seconds`, `world_load_timeout_seconds`, `carla_host`, `carla_port`, `ssv2_port`, `agent_relay`,
    /// `reconnect_wait_seconds`, `weather`, `npc_motion`, `gap_trace_dir`, `contact_report_dir`,
    /// `npc_contact_sensors`) over the file and the environment: a launch file's explicit
    /// value is the most specific statement of intent. A malformed number is an error, not
    /// a silent default.
    pub fn apply_ros_params(&mut self, params: &crate::ros_args::RosParams) -> Result<()> {
        if params.get("backend").is_some_and(|v| !v.trim().is_empty()) {
            bail!("{REMOVED_BACKEND}");
        }
        for old in [
            "isaac_endpoint",
            "isaac_render",
            "isaac_request_timeout_seconds",
            "isaac_load_timeout_seconds",
        ] {
            if params.get(old).is_some_and(|v| !v.trim().is_empty()) {
                bail!(
                    "parameter {old} was renamed {} (roadmap 046: one world interface)",
                    old.replacen("isaac_", "world_", 1)
                );
            }
        }
        if let Some(endpoint) = params
            .get("world_endpoint")
            .filter(|v| !v.trim().is_empty())
        {
            self.world.endpoint = endpoint.trim().to_string();
        }
        if let Some(v) = params.get("world_render").filter(|v| !v.trim().is_empty()) {
            self.world.render = match v.trim().to_ascii_lowercase().as_str() {
                "true" | "on" | "1" => true,
                "false" | "off" | "0" => false,
                other => bail!("parameter world_render:={other}: expected true or false"),
            };
        }
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
        if let Some(s) = num("world_request_timeout_seconds")? {
            self.world.request_timeout_seconds = s;
        }
        if let Some(s) = num("world_load_timeout_seconds")? {
            self.world.load_timeout_seconds = s;
        }
        if let Some(port) = num("carla_port")? {
            self.carla.port = u16::try_from(port).map_err(|e| eyre::eyre!("carla_port: {e}"))?;
        }
        if let Some(port) = num("ssv2_port")? {
            let port = u16::try_from(port).map_err(|e| eyre::eyre!("ssv2_port: {e}"))?;
            self.ssv2.port = port;
            // The sensor-release channel sits beside the SSv2 port (+1, +2) unless the file
            // names its own endpoints, so a second bridge on a host needs one setting
            // (SSV2_PORT) and acb's launch derives the same two from it.
            let defaults = SensorReleaseConfig::default();
            if self.sensor_release.notify_endpoint == defaults.notify_endpoint {
                self.sensor_release.notify_endpoint = format!("tcp://127.0.0.1:{}", port + 1);
            }
            if self.sensor_release.ack_endpoint == defaults.ack_endpoint {
                self.sensor_release.ack_endpoint = format!("tcp://127.0.0.1:{}", port + 2);
            }
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
        if let Some(motion) = params.get("npc_motion").filter(|v| !v.trim().is_empty()) {
            self.npc_motion = match motion.trim() {
                "kinematic" => NpcMotion::Kinematic,
                "tracked" => NpcMotion::Tracked,
                other => {
                    eyre::bail!("parameter npc_motion:={other}: expected `kinematic` or `tracked`")
                }
            };
        }
        if let Some(dir) = params.get("gap_trace_dir").filter(|v| !v.trim().is_empty()) {
            self.gap_trace_dir = dir.trim().to_string();
        }
        if let Some(dir) = params
            .get("contact_report_dir")
            .filter(|v| !v.trim().is_empty())
        {
            self.contact_report_dir = Some(dir.trim().to_string());
        }
        if let Some(v) = params
            .get("npc_contact_sensors")
            .filter(|v| !v.trim().is_empty())
        {
            self.npc_contact_sensors = Some(match v.trim().to_ascii_lowercase().as_str() {
                "true" | "on" | "1" => true,
                "false" | "off" | "0" => false,
                other => {
                    eyre::bail!("parameter npc_contact_sensors:={other}: expected true or false")
                }
            });
        }
        if params.get("background_avs").is_some_and(|v| !v.is_empty()) {
            bail!("{}", REMOVED_BACKGROUND_AVS);
        }
        self.follow_carla_port();
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
        if let Ok(v) = std::env::var("CSB_TICK_GATE") {
            self.tick_gate = v.trim().to_string();
        }
        if let Ok(v) = std::env::var("CSB_TM_SERVE") {
            match v.trim() {
                "1" | "true" | "on" => self.traffic_manager.serve = true,
                "0" | "false" | "off" | "" => self.traffic_manager.serve = false,
                other => tracing::warn!("Ignoring CSB_TM_SERVE='{other}': not 0/1"),
            }
        }
        if std::env::var("CSB_BACKGROUND_AVS").is_ok_and(|v| !v.trim().is_empty()) {
            tracing::warn!("CSB_BACKGROUND_AVS is ignored: {REMOVED_BACKGROUND_AVS}");
        }
        self.follow_carla_port();
    }

    /// Reject configurations that would misbehave in ways that are hard to diagnose later.
    fn validate(&self) -> Result<()> {
        if self.removed_background_avs.is_some() {
            bail!("{}", REMOVED_BACKGROUND_AVS);
        }
        if self.removed_backend.is_some() {
            bail!("{REMOVED_BACKEND}");
        }
        if self.removed_isaac.is_some() {
            bail!(
                "the isaac section was renamed world (roadmap 046): the bridge drives whichever \
                 world server answers at world.endpoint"
            );
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

    /// Whether tracked NPC vehicles get collision sensors (roadmap 032 step 5). Defaults to
    /// on: one sensor per NPC that fires only on contact, measured within the frame budget
    /// on the 20-NPC bench.
    pub fn npc_contact_sensors_enabled(&self) -> bool {
        self.npc_contact_sensors.unwrap_or(true)
    }

    /// The contact-report directory, `None` when reports are off.
    pub fn contact_report_dir(&self) -> Option<std::path::PathBuf> {
        crate::contact_report::resolve_dir(self.contact_report_dir.as_deref())
    }

    /// The gap-trace directory, `None` when the trace is off (roadmap 024).
    pub fn gap_trace_dir(&self) -> Option<std::path::PathBuf> {
        Some(self.gap_trace_dir.trim())
            .filter(|d| !d.is_empty())
            .map(std::path::PathBuf::from)
    }

    /// CARLA blueprint for an SSv2 asset key, if the config maps it.
    pub fn blueprint_for(&self, asset_key: &str) -> Option<&str> {
        self.blueprint_map.get(asset_key).map(String::as_str)
    }

    /// The configured body colour of the entity with this role_name, if any.
    pub fn color_for(&self, role_name: &str) -> Option<&str> {
        self.actor_colors.get(role_name).map(String::as_str)
    }
}

/// Why `backend` (file key, ROS parameter) is refused.
pub const REMOVED_BACKEND: &str = "backend was removed (roadmap 046): the bridge drives \
    whichever world server answers at world.endpoint (the world_endpoint parameter); start \
    the simulator's world server (CARLA's runs inside its simulation launch, Isaac's is `just \
    isaac-world`)";

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

    /// The served TM (a harness drives vehicles through it as a TM-Client) is off unless
    /// asked for, and keeps the port and seed it is given.
    #[test]
    fn the_traffic_manager_is_served_only_when_asked() {
        let config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert!(!config.traffic_manager.serve);
        let config: BridgeConfig =
            serde_yaml::from_str("traffic_manager:\n  port: 8100\n  serve: true\n")
                .expect("parses");
        assert!(config.traffic_manager.serve);
        assert_eq!(config.traffic_manager.port, 8100);
        assert_eq!(config.traffic_manager.seed, 2017);
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
    fn actor_colors_resolve_by_role_name() {
        let config: BridgeConfig =
            serde_yaml::from_str("actor_colors:\n  hero: \"255,110,0\"\n").expect("parses");
        assert_eq!(config.color_for("hero"), Some("255,110,0"));
        assert_eq!(config.color_for("bg_av_1"), None);
        let empty = BridgeConfig::default();
        assert_eq!(empty.color_for("hero"), None);
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
    fn release_endpoints_follow_the_ssv2_port_unless_the_file_names_them() {
        let params = |pairs: &[&str]| {
            let mut args = vec!["--ros-args".to_string()];
            for p in pairs {
                args.push("-p".to_string());
                args.push(p.to_string());
            }
            crate::ros_args::RosParams::from_args(args).expect("parses")
        };
        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        config
            .apply_ros_params(&params(&["ssv2_port:=5565"]))
            .expect("applies");
        assert_eq!(config.ssv2.port, 5565);
        assert_eq!(
            config.sensor_release.notify_endpoint,
            "tcp://127.0.0.1:5566"
        );
        assert_eq!(config.sensor_release.ack_endpoint, "tcp://127.0.0.1:5567");

        let mut config: BridgeConfig =
            serde_yaml::from_str("sensor_release:\n  notify_endpoint: tcp://127.0.0.1:7001\n")
                .expect("parses");
        config
            .apply_ros_params(&params(&["ssv2_port:=5565"]))
            .expect("applies");
        assert_eq!(
            config.sensor_release.notify_endpoint,
            "tcp://127.0.0.1:7001"
        );
        assert_eq!(config.sensor_release.ack_endpoint, "tcp://127.0.0.1:5567");

        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        config
            .apply_ros_params(&params(&["ssv2_port:="]))
            .expect("applies");
        assert_eq!(
            config.sensor_release.notify_endpoint,
            "tcp://127.0.0.1:5556"
        );
    }

    #[test]
    fn world_settings_take_launch_parameters_over_the_file() {
        let params = |pairs: &[&str]| {
            let mut args = vec!["--ros-args".to_string()];
            for p in pairs {
                args.push("-p".to_string());
                args.push(p.to_string());
            }
            crate::ros_args::RosParams::from_args(args).expect("parses")
        };
        let mut config: BridgeConfig = serde_yaml::from_str(
            "world:\n  request_timeout_seconds: 10\n  load_timeout_seconds: 120\n",
        )
        .expect("parses");
        // Empty values (the launch defaults) keep the file's.
        config
            .apply_ros_params(&params(&[
                "world_request_timeout_seconds:=",
                "world_render:=",
                "world_endpoint:=",
            ]))
            .expect("applies");
        assert_eq!(config.world.request_timeout_seconds, 10);
        assert!(!config.world.render);
        config
            .apply_ros_params(&params(&[
                "world_request_timeout_seconds:=60",
                "world_load_timeout_seconds:=300",
                "world_render:=true",
                "world_endpoint:=tcp://127.0.0.1:5591",
            ]))
            .expect("applies");
        assert_eq!(config.world.request_timeout_seconds, 60);
        assert_eq!(config.world.load_timeout_seconds, 300);
        assert!(config.world.render);
        assert_eq!(config.world.endpoint, "tcp://127.0.0.1:5591");
        assert!(config
            .apply_ros_params(&params(&["world_request_timeout_seconds:=soon"]))
            .is_err());
        // The old names are refused with the new.
        let err = config
            .apply_ros_params(&params(&["isaac_endpoint:=tcp://127.0.0.1:5590"]))
            .unwrap_err();
        assert!(
            format!("{err:#}").contains("renamed world_endpoint"),
            "{err:#}"
        );
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
    fn gap_trace_dir_defaults_to_off_and_takes_file_or_parameter() {
        let params = |v: &str| {
            crate::ros_args::RosParams::from_args([
                "--ros-args".to_string(),
                "-p".to_string(),
                format!("gap_trace_dir:={v}"),
            ])
            .expect("parses")
        };
        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert_eq!(config.gap_trace_dir(), None);
        config.apply_ros_params(&params("")).expect("applies");
        assert_eq!(config.gap_trace_dir(), None);
        config
            .apply_ros_params(&params(" /tmp/traces "))
            .expect("applies");
        assert_eq!(
            config.gap_trace_dir(),
            Some(std::path::PathBuf::from("/tmp/traces"))
        );
        let mut config: BridgeConfig =
            serde_yaml::from_str("gap_trace_dir: /a/b\n").expect("parses");
        assert_eq!(
            config.gap_trace_dir(),
            Some(std::path::PathBuf::from("/a/b"))
        );
        // An empty parameter keeps the file's, as every other parameter does.
        config.apply_ros_params(&params("")).expect("applies");
        assert_eq!(
            config.gap_trace_dir(),
            Some(std::path::PathBuf::from("/a/b"))
        );
        config.apply_ros_params(&params("/c")).expect("applies");
        assert_eq!(config.gap_trace_dir(), Some(std::path::PathBuf::from("/c")));
    }

    #[test]
    fn npc_contact_sensors_default_on_and_take_file_or_parameter() {
        let params = |v: &str| {
            crate::ros_args::RosParams::from_args([
                "--ros-args".to_string(),
                "-p".to_string(),
                format!("npc_contact_sensors:={v}"),
            ])
            .expect("parses")
        };
        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert!(config.npc_contact_sensors_enabled());
        config.apply_ros_params(&params("")).expect("applies");
        assert!(config.npc_contact_sensors_enabled());
        for off in ["false", "off", "0", " False "] {
            config.apply_ros_params(&params(off)).expect("applies");
            assert!(!config.npc_contact_sensors_enabled(), "{off}");
        }
        for on in ["true", "on", "1"] {
            config.apply_ros_params(&params(on)).expect("applies");
            assert!(config.npc_contact_sensors_enabled(), "{on}");
        }
        assert!(config.apply_ros_params(&params("maybe")).is_err());
        let mut config: BridgeConfig =
            serde_yaml::from_str("npc_contact_sensors: false\n").expect("parses");
        assert!(!config.npc_contact_sensors_enabled());
        // An empty parameter keeps the file's.
        config.apply_ros_params(&params("")).expect("applies");
        assert!(!config.npc_contact_sensors_enabled());
    }

    #[test]
    fn the_backend_switch_and_the_isaac_section_are_refused_with_the_way_forward() {
        let params = crate::ros_args::RosParams::from_args([
            "--ros-args".to_string(),
            "-p".to_string(),
            "backend:=isaac".to_string(),
        ])
        .expect("parses");
        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        let err = config.apply_ros_params(&params).unwrap_err();
        assert!(format!("{err:#}").contains("world.endpoint"), "{err:#}");
        for file in ["backend: isaac\n", "isaac:\n  render: true\n"] {
            let config: BridgeConfig = serde_yaml::from_str(file).expect("parses");
            let err = config.validate().unwrap_err();
            assert!(format!("{err:#}").contains("world"), "{file}: {err:#}");
        }
    }

    #[test]
    fn npc_motion_defaults_to_tracked_and_takes_file_or_parameter() {
        let params = |v: &str| {
            crate::ros_args::RosParams::from_args([
                "--ros-args".to_string(),
                "-p".to_string(),
                format!("npc_motion:={v}"),
            ])
            .expect("parses")
        };
        let config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        assert_eq!(config.npc_motion, NpcMotion::Tracked);
        let mut config: BridgeConfig =
            serde_yaml::from_str("npc_motion: kinematic\n").expect("parses");
        assert_eq!(config.npc_motion, NpcMotion::Kinematic);
        config.apply_ros_params(&params("")).expect("applies");
        assert_eq!(config.npc_motion, NpcMotion::Kinematic);
        config
            .apply_ros_params(&params("tracked"))
            .expect("applies");
        assert_eq!(config.npc_motion, NpcMotion::Tracked);
        config
            .apply_ros_params(&params("kinematic"))
            .expect("applies");
        assert_eq!(config.npc_motion, NpcMotion::Kinematic);
        assert!(config.apply_ros_params(&params("physics")).is_err());
        assert!(serde_yaml::from_str::<BridgeConfig>("npc_motion: fast\n").is_err());
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
    fn contact_reports_default_on_and_turn_off_or_move() {
        let dir = BridgeConfig::default()
            .contact_report_dir()
            .expect("default is on");
        assert!(dir.ends_with("csb/contacts"), "{}", dir.display());
        let off: BridgeConfig = serde_yaml::from_str("contact_report_dir: \"off\"\n").unwrap();
        assert_eq!(off.contact_report_dir(), None);

        let mut config = BridgeConfig::default();
        let params = crate::ros_args::RosParams::from_args(
            ["--ros-args", "-p", "contact_report_dir:=/tmp/x"].map(String::from),
        )
        .unwrap();
        config.apply_ros_params(&params).unwrap();
        assert_eq!(
            config.contact_report_dir(),
            Some(std::path::PathBuf::from("/tmp/x"))
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

    #[test]
    fn each_file_refuses_the_others_keys() {
        assert_eq!(
            misplaced_key("world: {}\nweather: ClearNoon\n", ConfigOwner::Bridge),
            None
        );
        let why = misplaced_key("substeps: 2\n", ConfigOwner::Bridge).unwrap();
        assert!(why.contains("carla_world.yaml"), "{why}");
        let why = misplaced_key("gap_trace_dir: /tmp\n", ConfigOwner::CarlaWorld).unwrap();
        assert!(why.contains("bridge_config.yaml"), "{why}");
        // Shared keys and unknown keys are not misplaced.
        assert_eq!(
            misplaced_key("ego:\n  role_name: hero\n", ConfigOwner::CarlaWorld),
            None
        );
        assert_eq!(misplaced_key("sky: blue\n", ConfigOwner::Bridge), None);
    }

    #[test]
    fn the_traffic_manager_port_follows_a_moved_carla() {
        let params = |p: &str| {
            crate::ros_args::RosParams::from_args([
                "--ros-args".to_string(),
                "-p".to_string(),
                p.to_string(),
            ])
            .expect("parses")
        };
        let mut config: BridgeConfig = serde_yaml::from_str("{}").expect("parses");
        config
            .apply_ros_params(&params("carla_port:=2010"))
            .unwrap();
        assert_eq!(config.traffic_manager.port, 8010);
        let mut config: BridgeConfig =
            serde_yaml::from_str("traffic_manager:\n  port: 9000\n").expect("parses");
        config
            .apply_ros_params(&params("carla_port:=2010"))
            .unwrap();
        assert_eq!(
            config.traffic_manager.port, 9000,
            "a port the file names is kept"
        );
    }
}
