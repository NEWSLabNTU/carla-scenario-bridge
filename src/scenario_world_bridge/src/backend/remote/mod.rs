//! The bridge's simulator: [`RemoteBackend`], a world server over the world protocol
//! (roadmaps 037, 046; `proto/swb_world.proto`, `docs/design/simulator-interface-split.md`).
//!
//! Every simulator is a world server: a process that owns the simulator and serves the world
//! protocol on a ZMQ REP socket (the Isaac Sim one, `src/isaac_world`; the CARLA one,
//! `src/carla_world`). This backend is the bridge's half, REQ to that REP, two round trips
//! per SSv2 frame as SSv2 sends them: `UpdateEntityStatus` becomes `Apply` (every entity's
//! drive for the coming step; the server answers with what it moves itself), `UpdateFrame`
//! becomes `Step` (the world advances; the server answers with every entity's state after it
//! and the contacts it had). Nothing advances between requests.
//!
//! What the backend does depends on what the server declares in its `Hello`
//! (`Capability`), never on which simulator it is:
//!
//! - Scenario vehicles under `npc_motion: tracked` (the default) are physics bodies following
//!   SSv2. A server that tracks them itself (`TRACKS_TARGETS`: CARLA, with steer and walker
//!   control) gets SSv2's pose and twist and reports each frame's tracking; one that is
//!   driven by velocity (`TRACKED_MOTION`: Isaac) gets the velocity this backend's tracking
//!   law computes (`tracking.rs`, roadmap 044). The rest are kinematic, posed by SSv2.
//! - The ego, agent-driven vehicles (`AGENT_VEHICLES`) and simulator-driven ones
//!   (`AUTOPILOT`) are moved by the simulator and read back: by the server, with its own
//!   acceleration and jerk (`READS_BACK`), or here, differenced from its poses.
//! - Signals (`SIGNALS`), goals (`AUTOPILOT`), weather (`WEATHER`): a server without the
//!   capability is never asked, and SSv2's request is an honest failure naming it (006).
//!
//! The policy every simulator shares lives here: the entity table, the agent link (controller
//! `agent`, through the relay), the gap trace (024), contact reports (021), tracking
//! statistics, the map origin (`<map dir>/<simulator>/world.yaml`) and the signal table.
//!
//! Ownership (roadmap 046): the bridge's `Hello` makes it the server's one owner, and its idle
//! probe keeps the lease. A server that restarted (its `instance` changed) has lost every
//! entity: the backend forgets them too. A request the server does not answer in time, or a
//! reply marked fatal, wedges the bridge: SSv2 gets the error, the bridge drains and exits 75,
//! and its supervisor starts a fresh one (034).

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use serde::Deserialize;

use crate::agent_link::{self, AgentLink, AgentPose, Command as AgentCommand};
use crate::backend::{
    BackendError, Driver, EntityKind, EntityStatus, Environment, Episode, Goal, Sensor, SimBackend,
    SpawnRequest, StatusBatch, StatusOutcome,
};
use crate::body_clearance::{self, BodyBox};
use crate::budgets;
use crate::config::{BridgeConfig, NpcMotion};
use crate::contact_report::{self, ContactReport, RawCollision, ReportContact};
use crate::coordinate_conversion::quaternion_to_euler;
use crate::entity_manager::EntityType;
use crate::environment::{PrecipitationType, ScenarioWeather};
use crate::gap_trace::{self, GapTrace};
use crate::map_resolver::{load_map_origin, map_dir_of};
use crate::npc_tracking::{self, TrackingStats};
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::simulation_api_schema::TrafficSignal;
use crate::proto::swb_world as wire;
use crate::proto::traffic_simulator_msgs::{ActionStatus, BoundingBox};
use crate::weather::{wire_form, WeatherReply, WeatherRequest};
use swb_core::world_client::{self, CallError, Client};

mod signals;
mod tracking;
use signals::SignalTable;
use tracking::{Drive, Tracked};

pub use world_client::PROTOCOL_VERSION;

/// How long the bridge waits, at its end, for the world server to undo what it did. CARLA's
/// teardown of a scenario's actors takes well under a second.
const RELEASE_TIMEOUT: Duration = Duration::from_secs(3);

/// `<map dir>/<simulator>/world.yaml`: the simulator's world for a Lanelet2 map. Absent: the
/// server's default world (a flat ground plane; CARLA: the town the directory names) with
/// the map frame as the world frame.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct WorldFile {
    /// The world, as the server names it: a USD stage (relative to the map directory or
    /// absolute), a CARLA town. Empty: the server's default. `usd` is its older name.
    #[serde(alias = "usd")]
    pub world: String,
    /// Prims a body rests on (terrain, road surface): contacts with them are not reported.
    pub ground_paths: Vec<String>,
    /// Where the world's origin sits in the map frame, m.
    pub map_origin: [f64; 3],
}

impl WorldFile {
    /// Where a map's world file is for the simulator `name`, under its map directory.
    pub fn subpath(name: &str) -> PathBuf {
        Path::new(name).join("world.yaml")
    }

    /// The world for the map at `lanelet2_map_path` on simulator `name`, and where it was
    /// read from (`None`: the default). A file that does not parse is an error, not a silent
    /// default. CARLA's maps may instead carry only `carla/map_origin.yaml` (the older file).
    pub fn for_map(name: &str, lanelet2_map_path: &str) -> eyre::Result<(Self, Option<PathBuf>)> {
        let map_dir = map_dir_of(Path::new(lanelet2_map_path));
        let path = map_dir.join(Self::subpath(name));
        if !path.exists() {
            let mut world = Self::default();
            let origin = map_dir.join(name).join("map_origin.yaml");
            if origin.exists() {
                let o = load_map_origin(&map_dir)?;
                world.map_origin = [o.x, o.y, o.z];
                return Ok((world, Some(origin)));
            }
            return Ok((world, None));
        }
        let text = std::fs::read_to_string(&path)
            .map_err(|e| eyre::eyre!("read {}: {e}", path.display()))?;
        let mut world: Self = serde_yaml::from_str(&text)
            .map_err(|e| eyre::eyre!("parse {}: {e}", path.display()))?;
        // A world named by a path beside the map is resolved against the map directory.
        if !world.world.is_empty() && Path::new(&world.world).is_relative() {
            let beside = map_dir.join(&world.world);
            if beside.exists() || world.world.contains('/') {
                world.world = beside.to_string_lossy().into_owned();
            }
        }
        Ok((world, Some(path)))
    }
}

/// Physics substeps and their length for an SSv2 step: the fewest steps of at most
/// `max_dt` that make up the step exactly; one step of the whole when `max_dt` is 0 (the
/// server splits it itself).
pub fn physics_steps(step_time: f64, max_dt: f64) -> (u32, f64) {
    if max_dt <= 0.0 {
        return (1, step_time);
    }
    let n = ((step_time / max_dt) - 1e-9).ceil().max(1.0) as u32;
    (n, step_time / n as f64)
}

/// The box an entity is spawned with when SSv2 declares none, by kind: centre and size, m.
fn default_box(kind: wire::Kind) -> ([f64; 3], [f64; 3]) {
    match kind {
        wire::Kind::Ego | wire::Kind::Vehicle => ([1.5, 0.0, 0.9], [4.5, 2.1, 1.8]),
        wire::Kind::Pedestrian => ([0.0, 0.0, 0.9], [0.5, 0.5, 1.8]),
        _ => ([0.0, 0.0, 0.5], [1.0, 1.0, 1.0]),
    }
}

/// The declared box, or the kind's default when absent or degenerate; and whether the
/// declared one was used.
fn entity_box(declared: Option<&BoundingBox>, kind: wire::Kind) -> ([f64; 3], [f64; 3], bool) {
    let fallback = default_box(kind);
    let Some(b) = declared else {
        return (fallback.0, fallback.1, false);
    };
    let dims = b.dimensions.map(|d| [d.x, d.y, d.z]);
    match dims {
        Some(d) if d.iter().all(|v| v.is_finite() && *v > 0.0) => {
            let c = b.center.map_or(fallback.0, |c| [c.x, c.y, c.z]);
            (c, d, true)
        }
        _ => (fallback.0, fallback.1, false),
    }
}

fn v3(a: [f64; 3]) -> Option<wire::Vec3> {
    Some(wire::Vec3 {
        x: a[0],
        y: a[1],
        z: a[2],
    })
}

/// A map-frame SSv2 pose in the world frame.
fn to_world(pose: &Pose, origin: [f64; 3]) -> wire::Pose {
    let p = pose.position.unwrap_or_default();
    let q = pose.orientation.unwrap_or(geometry_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    });
    wire::Pose {
        position: Some(wire::Vec3 {
            x: p.x - origin[0],
            y: p.y - origin[1],
            z: p.z - origin[2],
        }),
        orientation: Some(wire::Quat {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        }),
        rpy: None,
    }
}

/// A world-frame pose as an SSv2 map-frame pose.
fn to_map(pose: &wire::Pose, origin: [f64; 3]) -> Pose {
    let p = pose.position.unwrap_or_default();
    let q = pose.orientation.unwrap_or(wire::Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    });
    Pose {
        position: Some(geometry_msgs::Point {
            x: p.x + origin[0],
            y: p.y + origin[1],
            z: p.z + origin[2],
        }),
        orientation: Some(geometry_msgs::Quaternion {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        }),
    }
}

/// `[x, y, z, roll, pitch, yaw]` of an SSv2 pose, for the gap trace.
fn pose6(pose: &Pose) -> gap_trace::Pose6 {
    let p = pose.position.unwrap_or_default();
    let (roll, pitch, yaw) = pose
        .orientation
        .map_or((0.0, 0.0, 0.0), |q| quaternion_to_euler(q.x, q.y, q.z, q.w));
    [p.x, p.y, p.z, roll, pitch, yaw]
}

/// A server's world-frame pose as `[x, y, z, roll, pitch, yaw]` in the map frame: from its
/// Euler angles when it holds them (`rpy`), else from the quaternion.
fn state_pose6(pose: &wire::Pose, origin: [f64; 3]) -> gap_trace::Pose6 {
    match pose.rpy {
        Some(rpy) => {
            let p = pose.position.unwrap_or_default();
            [
                p.x + origin[0],
                p.y + origin[1],
                p.z + origin[2],
                rpy.x,
                rpy.y,
                rpy.z,
            ]
        }
        None => pose6(&to_map(pose, origin)),
    }
}

fn wrap_angle(a: f64) -> f64 {
    let a = (a + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI);
    a - std::f64::consts::PI
}

/// One entity's motion, differenced from the server's poses at successive steps.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Motion {
    /// World-frame velocity, m/s.
    velocity: [f64; 3],
    /// Body-frame (x forward) velocity, m/s, and yaw rate, rad/s.
    body: [f64; 2],
    yaw_rate: f64,
    /// Body-frame acceleration, m/s2.
    accel: [f64; 2],
}

/// The motion from `prev` (pose6, motion) to `now` over `dt`.
fn motion(prev: (&gap_trace::Pose6, &Motion), now: &gap_trace::Pose6, dt: f64) -> Motion {
    if dt <= 0.0 {
        return *prev.1;
    }
    let (p, m) = prev;
    let velocity = [
        (now[0] - p[0]) / dt,
        (now[1] - p[1]) / dt,
        (now[2] - p[2]) / dt,
    ];
    let (sin, cos) = now[5].sin_cos();
    let body = [
        velocity[0] * cos + velocity[1] * sin,
        -velocity[0] * sin + velocity[1] * cos,
    ];
    Motion {
        velocity,
        body,
        yaw_rate: wrap_angle(now[5] - p[5]) / dt,
        accel: [(body[0] - m.body[0]) / dt, (body[1] - m.body[1]) / dt],
    }
}

/// The motion from a body's own world-frame `twist` at pose `now`, after `prev` over `dt`.
fn motion_from_twist(
    prev: &Motion,
    now: &gap_trace::Pose6,
    twist: &wire::Twist,
    dt: f64,
) -> Motion {
    let v = twist.linear.unwrap_or_default();
    let velocity = [v.x, v.y, v.z];
    let (sin, cos) = now[5].sin_cos();
    let body = [
        velocity[0] * cos + velocity[1] * sin,
        -velocity[0] * sin + velocity[1] * cos,
    ];
    let accel = if dt > 0.0 {
        [(body[0] - prev.body[0]) / dt, (body[1] - prev.body[1]) / dt]
    } else {
        prev.accel
    };
    Motion {
        velocity,
        body,
        yaw_rate: twist.angular.map_or(0.0, |w| w.z),
        accel,
    }
}

fn action_status(m: &Motion) -> ActionStatus {
    let v = |x, y, z| Some(geometry_msgs::Vector3 { x, y, z });
    ActionStatus {
        current_action: String::new(),
        twist: Some(geometry_msgs::Twist {
            linear: v(m.body[0], m.body[1], 0.0),
            angular: v(0.0, 0.0, m.yaw_rate),
        }),
        accel: Some(geometry_msgs::Accel {
            linear: v(m.accel[0], m.accel[1], 0.0),
            angular: v(0.0, 0.0, 0.0),
        }),
        linear_jerk: 0.0,
    }
}

/// A server's read-back of a body it moves, as SSv2 wants it.
fn read_back_status(name: &str, rb: &wire::ReadBack, origin: [f64; 3]) -> EntityStatus {
    let vec = |v: Option<wire::Vec3>| {
        let v = v.unwrap_or_default();
        Some(geometry_msgs::Vector3 {
            x: v.x,
            y: v.y,
            z: v.z,
        })
    };
    let twist = rb.twist.unwrap_or_default();
    EntityStatus {
        name: name.to_string(),
        pose: rb.pose.as_ref().map(|p| to_map(p, origin)),
        action_status: Some(ActionStatus {
            current_action: String::new(),
            twist: Some(geometry_msgs::Twist {
                linear: vec(twist.linear),
                angular: vec(twist.angular),
            }),
            accel: Some(geometry_msgs::Accel {
                linear: vec(rb.accel),
                angular: vec(None),
            }),
            linear_jerk: rb.linear_jerk,
        }),
    }
}

/// SSv2's twist for an entity, its own frame, as the protocol carries it.
fn ssv2_twist(status: &ActionStatus) -> Option<wire::Twist> {
    let t = status.twist.as_ref()?;
    let l = t.linear.unwrap_or_default();
    let a = t.angular.unwrap_or_default();
    Some(wire::Twist {
        linear: v3([l.x, l.y, l.z]),
        angular: v3([a.x, a.y, a.z]),
    })
}

/// SSv2's pose as an agent's pose message.
fn agent_pose(pose: &Pose) -> AgentPose {
    let p = pose.position.unwrap_or_default();
    let q = pose.orientation.unwrap_or(geometry_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    });
    AgentPose {
        x: p.x,
        y: p.y,
        z: p.z,
        qx: q.x,
        qy: q.y,
        qz: q.z,
        qw: q.w,
    }
}

/// Who moves an entity, as this backend treats it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mover {
    /// SSv2's pose, every frame.
    Kinematic,
    /// A physics body following SSv2: by this backend's law (`TRACKED_MOTION`).
    BridgeTracked,
    /// The same, tracked by the server (`TRACKS_TARGETS`).
    ServerTracked,
    /// The simulator moves it; read back (the ego, agent and autopilot vehicles).
    Simulator,
}

#[derive(Debug, Clone)]
struct Entity {
    handle: u32,
    entity_type: EntityType,
    mover: Mover,
    /// Controller `agent`: its agent drives it through the relay.
    agent: bool,
    /// Controller `simulator_autopilot`: the simulator's traffic drives it.
    autopilot: bool,
    /// The server's last pose of it, map frame, and its motion.
    pose: Pose,
    pose6: gap_trace::Pose6,
    motion: Motion,
    /// The declared box (centre from the origin, dimensions), for clearances.
    center: [f64; 3],
    dims: [f64; 3],
    /// The simulator's id for it, for the gap trace and contacts (the handle when none).
    sim_id: u32,
    /// A bridge-tracked body's law state (roadmap 044).
    tracked: Option<Tracked>,
    /// A server-tracked body's statistics, from its `TrackingSample`s.
    server_stats: Option<TrackingStats>,
    /// The last [`ACCEL_SMOOTHING_FRAMES`] raw body-frame accelerations.
    accel_history: std::collections::VecDeque<[f64; 2]>,
}

/// Frames the reported acceleration is the median of, as the CARLA world server smooths its
/// own: a one-frame contact jolt -- the ego shoved by a tracked NPC, 35 m/s2 in
/// `town01_rear_contact` (044 step 2) -- is discarded, a sustained profile passes.
const ACCEL_SMOOTHING_FRAMES: usize = 3;

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    match v.len() {
        0 => 0.0,
        n if n % 2 == 1 => v[n / 2],
        n => (v[n / 2 - 1] + v[n / 2]) / 2.0,
    }
}

/// The median of each axis over `history`.
fn smoothed(history: &std::collections::VecDeque<[f64; 2]>) -> [f64; 2] {
    [
        median(history.iter().map(|a| a[0]).collect()),
        median(history.iter().map(|a| a[1]).collect()),
    ]
}

/// The ego's contacts over its run, for the contact report (021).
#[derive(Debug)]
struct EgoRun {
    name: String,
    episode: u64,
    start_unix: f64,
    handle: u32,
    /// By other entity handle (0: the environment, then by path), in order of first contact.
    contacts: Vec<(u32, String, ReportContact)>,
    /// Tracked NPCs' own contacts (032 step 5's `npc_contacts`), by NPC and other, in order.
    npc_contacts: Vec<(String, u32, String, ReportContact)>,
}

/// What the server said about itself in its `Hello`.
#[derive(Debug, Clone, Default)]
struct ServerInfo {
    name: String,
    version: String,
    capabilities: BTreeSet<i32>,
    instance: String,
    max_physics_dt: f64,
}

impl ServerInfo {
    fn of(h: &wire::HelloReply) -> Self {
        Self {
            name: h.simulator_name.clone(),
            version: h.simulator_version.clone(),
            capabilities: world_client::capabilities(h),
            instance: h.instance.clone(),
            max_physics_dt: h.max_physics_dt,
        }
    }

    fn offers(&self, c: wire::Capability) -> bool {
        self.capabilities.contains(&(c as i32))
    }

    fn capability_list(&self) -> String {
        let names: Vec<&str> = self
            .capabilities
            .iter()
            .filter_map(|c| wire::Capability::try_from(*c).ok())
            .map(world_client::capability_name)
            .collect();
        if names.is_empty() {
            "core only".into()
        } else {
            names.join(", ")
        }
    }
}

/// Connect to the configured world server, retrying every 5 s until it answers `Hello` in
/// this build's protocol version, takes this bridge as its owner and is ready to load a
/// world; then build the backend on it. `None` when `shutdown` was set first. A world
/// server takes from seconds (CARLA's) to minutes (Isaac's preload) to start, so the bridge
/// may well come up first; until it is connected it does not serve SSv2.
pub fn connect(config: BridgeConfig, shutdown: &AtomicBool) -> eyre::Result<Option<RemoteBackend>> {
    let endpoint = config.world.endpoint.clone();
    let mut client = Client::connect(&endpoint)?;
    let mut waited = Instant::now();
    let mut first = true;
    loop {
        if shutdown.load(Ordering::SeqCst) {
            return Ok(None);
        }
        let why = match client.hello(false, Duration::from_secs(2)) {
            Ok(h) if h.ready => {
                let info = ServerInfo::of(&h);
                tracing::info!(
                    "World server at {endpoint}: {} ({}), protocol {PROTOCOL_VERSION}, \
                     capabilities {}",
                    info.name,
                    info.version,
                    info.capability_list()
                );
                return Ok(Some(RemoteBackend::new(client, config, info)));
            }
            Ok(h) => format!("not ready: {}", h.not_ready_reason),
            Err(e) => {
                client.reset();
                format!("{e}")
            }
        };
        if first || waited.elapsed() >= Duration::from_secs(60) {
            tracing::warn!("World server at {endpoint} not ready ({why}); waiting");
            waited = Instant::now();
            first = false;
        }
        std::thread::sleep(Duration::from_secs(5));
    }
}

pub struct RemoteBackend {
    client: Client,
    config: BridgeConfig,
    server: ServerInfo,
    request_timeout: Duration,
    load_timeout: Duration,
    /// By SSv2 name.
    entities: BTreeMap<String, Entity>,
    /// Handle to SSv2 name, for contacts.
    names: BTreeMap<u32, String>,
    /// What the next `Apply` does with each body, by handle (built by `update_entity_status`,
    /// sent at its end).
    pending: BTreeMap<u32, wire::EntityPose>,
    /// Tracking stats of the episode's NPCs already despawned, by name (roadmap 044).
    tracking_done: BTreeMap<String, TrackingStats>,
    /// The map's signal table (roadmap 044 B), and the colour last sent per OpenDRIVE id.
    signals: SignalTable,
    signal_colours: BTreeMap<String, wire::Colour>,
    /// Way ids SSv2 commanded that the table does not have, warned once each.
    unmapped_signals: BTreeSet<i32>,
    map_origin: [f64; 3],
    step_time: f64,
    substeps: u32,
    dt: f64,
    /// The server's sim time after the last step, s, and its frame.
    sim_time: f64,
    server_frame: u64,
    /// `UpdateFrame`s since `Initialize`.
    frame: u64,
    episode: u64,
    tick_time: Duration,
    wedged: Option<String>,
    server_lost: Option<String>,
    idle_probe_failures: u32,
    /// When the idle check first went unanswered, in the current run of failures.
    unanswered_since: Option<Instant>,
    gap_trace: GapTrace,
    contact_dir: Option<PathBuf>,
    ego_run: Option<EgoRun>,
    /// The agent relay connection for `agent` entities, started at the first such spawn.
    agent_link: Option<AgentLink>,
    /// The drives the last `Apply` carried, by handle, for the tests.
    #[cfg(test)]
    sent: BTreeMap<u32, wire::EntityPose>,
}

impl RemoteBackend {
    fn new(client: Client, config: BridgeConfig, server: ServerInfo) -> Self {
        let (substeps, dt) = physics_steps(0.05, server.max_physics_dt);
        Self {
            client,
            request_timeout: Duration::from_secs(config.world.request_timeout_seconds.max(1)),
            load_timeout: Duration::from_secs(config.world.load_timeout_seconds.max(1)),
            gap_trace: GapTrace::new(config.gap_trace_dir()),
            contact_dir: config.contact_report_dir(),
            config,
            server,
            entities: BTreeMap::new(),
            names: BTreeMap::new(),
            pending: BTreeMap::new(),
            tracking_done: BTreeMap::new(),
            signals: SignalTable::default(),
            signal_colours: BTreeMap::new(),
            unmapped_signals: Default::default(),
            map_origin: [0.0; 3],
            step_time: 0.05,
            substeps,
            dt,
            sim_time: 0.0,
            server_frame: 0,
            frame: 0,
            episode: 0,
            tick_time: Duration::ZERO,
            wedged: None,
            server_lost: None,
            idle_probe_failures: 0,
            unanswered_since: None,
            ego_run: None,
            agent_link: None,
            #[cfg(test)]
            sent: BTreeMap::new(),
        }
    }

    fn offers(&self, c: wire::Capability) -> bool {
        self.server.offers(c)
    }

    /// The server's name, for messages.
    fn sim(&self) -> &str {
        if self.server.name.is_empty() {
            "the world server"
        } else {
            &self.server.name
        }
    }

    /// One request; a timeout or a fatal reply wedges the bridge (see the module docs).
    fn call(
        &mut self,
        what: &str,
        request: wire::request::Request,
        timeout: Duration,
    ) -> Result<wire::Reply, BackendError> {
        if let Some(reason) = &self.wedged {
            return Err(BackendError::Failed(reason.clone()));
        }
        match self.client.call(request, timeout) {
            Ok(reply) => Ok(reply),
            Err(CallError::Refused { error, fatal }) => {
                if fatal {
                    let reason = format!(
                        "{what}: the {} world server cannot go on: {error}; the bridge ends so \
                         its supervisor starts a fresh one",
                        self.sim()
                    );
                    tracing::error!("{reason}");
                    self.wedged = Some(reason.clone());
                    return Err(BackendError::Failed(reason));
                }
                Err(BackendError::Failed(format!("{what}: {error}")))
            }
            Err(CallError::Io(e)) => Err(BackendError::Disconnected(format!(
                "{what}: the world server at {}: {e}",
                self.client.endpoint()
            ))),
            Err(CallError::Timeout(t)) => {
                let reason = format!(
                    "the {} world server did not answer {what} within {} s; the bridge ends so \
                     its supervisor starts a fresh one",
                    self.sim(),
                    t.as_secs()
                );
                tracing::error!("{reason}");
                self.wedged = Some(reason.clone());
                Err(BackendError::Failed(reason))
            }
        }
    }

    fn set_step_time(&mut self, step_time: f64) {
        self.step_time = step_time;
        (self.substeps, self.dt) = physics_steps(step_time, self.server.max_physics_dt);
    }

    /// The agent link, started on first use.
    fn agent_link(&mut self) -> &mut AgentLink {
        let relay = self.config.agent_relay().to_string();
        self.agent_link
            .get_or_insert_with(|| AgentLink::start(&relay))
    }

    /// End the ego's run: write its contact report (021), if reports are on.
    fn finish_ego_run(&mut self, complete: bool) {
        let Some(run) = self.ego_run.take() else {
            return;
        };
        let tracking = self.tracking_rows();
        let npc_contacts = run
            .npc_contacts
            .into_iter()
            .map(|(npc, _, _, contact)| contact_report::ReportNpcContact { npc, contact })
            .collect();
        let contacts: Vec<ReportContact> = run.contacts.into_iter().map(|(_, _, c)| c).collect();
        tracing::info!(
            "Contacts of ego '{}' on {}: {} other(s), {} contact frame(s)",
            run.name,
            self.sim(),
            contacts.len(),
            contacts.iter().map(|c| c.count).sum::<u64>()
        );
        let Some(dir) = self.contact_dir.clone() else {
            return;
        };
        contact_report::write_in_background(
            dir,
            ContactReport {
                schema: contact_report::SCHEMA,
                episode: run.episode,
                ego: run.name,
                start_unix: run.start_unix,
                end_unix: contact_report::now_unix(),
                complete,
                contacts,
                geometry: Vec::new(),
                tracking,
                npc_contacts,
            },
        );
    }

    /// Every tracked NPC of the episode: those despawned, then those still here.
    fn tracking_rows(&self) -> Vec<contact_report::ReportTracking> {
        let live = self.entities.iter().filter_map(|(n, e)| {
            e.tracked
                .as_ref()
                .map(|t| &t.stats)
                .or(e.server_stats.as_ref())
                .map(|s| (n, s))
        });
        self.tracking_done
            .iter()
            .chain(live)
            .map(|(name, stats)| {
                let s = stats.summary();
                contact_report::ReportTracking {
                    name: name.clone(),
                    frames: s.frames,
                    error_p95: s.error_p95,
                    error_max: s.error_max,
                    yaw_p95: s.yaw_p95,
                    yaw_max: s.yaw_max,
                    snaps: s.snaps,
                    missed: s.missed,
                    fallback: s.fallback,
                }
            })
            .collect()
    }

    /// One log line per tracked NPC of the episode.
    fn log_tracking(&self) {
        for row in self.tracking_rows() {
            tracing::info!(
                "Tracked '{}' on {}: {} frames, error p95 {:.3} / max {:.3} m, yaw p95 {:.2} / \
                 max {:.2} deg, {} snap(s){}{}",
                row.name,
                self.sim(),
                row.frames,
                row.error_p95,
                row.error_max,
                row.yaw_p95.unwrap_or_default(),
                row.yaw_max.unwrap_or_default(),
                row.snaps,
                if row.missed > 0 {
                    format!(", {} frame(s) missed", row.missed)
                } else {
                    String::new()
                },
                if row.fallback > 0 {
                    format!(
                        ", {} frame(s) kinematic after a blocked fallback",
                        row.fallback
                    )
                } else {
                    String::new()
                }
            );
        }
    }

    /// The contacts of one step: the ego's into its run, tracked NPCs' into theirs (and a
    /// bridge-tracked NPC's contact hold), all into the gap trace.
    fn record_contacts(&mut self, contacts: &[wire::Contact]) {
        let mut traced = Vec::new();
        let ego = self.ego_run.as_ref().map(|r| r.handle);
        let frame = self.frame;
        for c in contacts {
            let name = (c.other != 0)
                .then(|| self.names.get(&c.other).cloned())
                .flatten();
            let other_type = if !c.other_type.is_empty() {
                c.other_type.clone()
            } else {
                match &name {
                    Some(n) => self
                        .entities
                        .get(n)
                        .map_or("entity", |e| kind_name(e.entity_type))
                        .to_string(),
                    None => c.other_path.clone(),
                }
            };
            let other_id = if c.other_sim_id != 0 {
                Some(c.other_sim_id)
            } else {
                (c.other != 0).then_some(c.other)
            };
            let subject = (Some(c.handle) != ego)
                .then(|| self.names.get(&c.handle).cloned())
                .flatten();
            // The contact hold of a bridge-tracked NPC touching the ego, seen from either side.
            for (npc, other) in [(c.handle, c.other), (c.other, c.handle)] {
                if Some(other) != ego || npc == 0 {
                    continue;
                }
                if let Some(t) = self
                    .names
                    .get(&npc)
                    .and_then(|n| self.entities.get_mut(n))
                    .and_then(|e| e.tracked.as_mut())
                {
                    t.last_ego_contact = Some(frame);
                }
            }
            let key_path = if c.other == 0 {
                format!("{}{}", c.other_path, c.other_sim_id)
            } else {
                String::new()
            };
            let sim_time = if c.sim_time > 0.0 {
                c.sim_time
            } else {
                self.sim_time
            };
            let fresh = |count: u64| ReportContact {
                other_id,
                name: name.clone(),
                other_type: other_type.clone(),
                is_static: c.other == 0,
                first_ssv2_frame: frame,
                first_sim_time: sim_time,
                count,
                max_impulse: c.impulse,
            };
            let sim = self.server.name.clone();
            if let Some(run) = self.ego_run.as_mut() {
                if Some(c.handle) == ego {
                    match run
                        .contacts
                        .iter_mut()
                        .find(|(h, p, _)| *h == c.other && *p == key_path)
                    {
                        Some((_, _, rc)) => {
                            rc.count += 1;
                            rc.max_impulse = rc.max_impulse.max(c.impulse);
                        }
                        None => {
                            tracing::info!(
                                "{sim} contact: ego '{}' touches {} at frame {frame}",
                                run.name,
                                name.as_deref().unwrap_or(&other_type),
                            );
                            run.contacts.push((c.other, key_path.clone(), fresh(1)));
                        }
                    }
                } else if let Some(npc) = subject.as_ref() {
                    match run
                        .npc_contacts
                        .iter_mut()
                        .find(|(n, h, p, _)| n == npc && *h == c.other && *p == key_path)
                    {
                        Some((_, _, _, rc)) => {
                            rc.count += 1;
                            rc.max_impulse = rc.max_impulse.max(c.impulse);
                        }
                        None => {
                            tracing::info!(
                                "{sim} contact: NPC '{npc}' touches {} at frame {frame}",
                                name.as_deref().unwrap_or(&other_type),
                            );
                            run.npc_contacts.push((
                                npc.clone(),
                                c.other,
                                key_path.clone(),
                                fresh(1),
                            ));
                        }
                    }
                }
            }
            traced.push((
                RawCollision {
                    ssv2_frame: frame,
                    carla_frame: if c.frame != 0 {
                        c.frame
                    } else {
                        self.server_frame
                    },
                    sim_time,
                    other_id,
                    other_type,
                    impulse: c.impulse,
                    subject,
                },
                name,
            ));
        }
        if self.gap_trace.recording() {
            self.gap_trace.contacts(traced);
        }
    }

    /// The smallest box-to-box clearance from `name` to any other entity, m.
    fn clearance(&self, name: &str) -> Option<f64> {
        let place = |e: &Entity| {
            BodyBox {
                offset_x: e.center[0],
                offset_y: e.center[1],
                half_length: e.dims[0] / 2.0,
                half_width: e.dims[1] / 2.0,
            }
            .at(e.pose6[0], e.pose6[1], e.pose6[5].to_degrees())
        };
        let subject = place(self.entities.get(name)?);
        body_clearance::min_clearance(
            &subject,
            self.entities
                .iter()
                .filter(|(n, _)| n.as_str() != name)
                .map(|(_, e)| place(e)),
        )
    }

    /// One frame of the bridge-tracked NPC `name` (roadmap 044): SSv2's map-frame `pose` and
    /// its twist (`None` while held) into the next step's drive. Returns the gap trace's
    /// `(contact_hold, kinematic)` flags.
    fn track(&mut self, name: &str, pose: &Pose, twist: Option<(f64, f64, f64)>) -> (bool, bool) {
        let clearance = self.clearance(name);
        let origin = self.map_origin;
        let frame = self.frame + 1;
        let dt = self.step_time;
        let Some(entity) = self.entities.get_mut(name) else {
            return (false, false);
        };
        let world = to_world(pose, origin);
        let target6 = pose6(&to_map(&world, [0.0; 3]));
        let frame_in = tracking::Frame {
            target: tracking::TargetPose {
                x: target6[0],
                y: target6[1],
                z: target6[2],
                roll_deg: target6[3].to_degrees(),
                pitch_deg: target6[4].to_degrees(),
                yaw_deg: target6[5].to_degrees(),
            },
            twist,
            body: tracking::BodyPose {
                x: entity.pose6[0] - origin[0],
                y: entity.pose6[1] - origin[1],
                yaw_deg: entity.pose6[5].to_degrees(),
            },
            clearance,
            frame,
            dt,
        };
        let handle = entity.handle;
        let Some(t) = entity.tracked.as_mut() else {
            return (false, false);
        };
        let o = tracking::track(t, &frame_in);
        let twist_of = |linear: [f64; 3], yaw_rate: f64| {
            Some(wire::Twist {
                linear: v3(linear),
                angular: v3([0.0, 0.0, yaw_rate]),
            })
        };
        let entity_pose = match o.drive {
            Drive::Twist { linear, yaw_rate } => wire::EntityPose {
                handle,
                twist: twist_of(linear, yaw_rate),
                ..Default::default()
            },
            Drive::Teleport {
                pose: p,
                linear,
                yaw_rate,
            } => {
                tracing::debug!(
                    "Snap '{name}': error {:.2} m, yaw error {:.1} deg",
                    o.error,
                    o.yaw_error
                );
                let q = crate::coordinate_conversion::euler_to_quaternion(
                    p.roll_deg.to_radians(),
                    p.pitch_deg.to_radians(),
                    p.yaw_deg.to_radians(),
                );
                wire::EntityPose {
                    handle,
                    pose: Some(wire::Pose {
                        position: v3([p.x, p.y, p.z]),
                        orientation: Some(wire::Quat {
                            x: q.0,
                            y: q.1,
                            z: q.2,
                            w: q.3,
                        }),
                        rpy: None,
                    }),
                    twist: twist_of(linear, yaw_rate),
                    teleport: true,
                    kinematic: false,
                    ssv2_twist: None,
                }
            }
            Drive::Kinematic => {
                tracing::warn!(
                    "Tracking '{name}': blocked -- {:.2} m off -- so not snapped into what holds \
                     it: kinematic for the rest of the episode (SSv2's pose every frame)",
                    o.error
                );
                wire::EntityPose {
                    handle,
                    pose: Some(world),
                    kinematic: true,
                    ..Default::default()
                }
            }
        };
        let kinematic = t.kinematic;
        self.pending.insert(handle, entity_pose);
        (o.held, kinematic)
    }

    fn handle_of(&self, name: &str) -> Option<u32> {
        self.entities.get(name).map(|e| e.handle)
    }

    /// The server restarted (its `Hello` names another instance): every entity it had is
    /// gone, and so is what this backend knew of them.
    fn forget_server_state(&mut self, why: &str) {
        tracing::warn!(
            "The {} world server restarted ({why}): forgetting its {} entit{}",
            self.sim(),
            self.entities.len(),
            if self.entities.len() == 1 { "y" } else { "ies" }
        );
        self.finish_ego_run(false);
        if self.gap_trace.recording() {
            self.gap_trace.mark_incomplete();
            self.gap_trace.end_episode();
        }
        self.entities.clear();
        self.names.clear();
        self.pending.clear();
        self.signal_colours.clear();
    }

    /// `UpdateEntityGoal` for an `agent` entity: its goal, route or target speed goes to its
    /// agent (queued; sent once SSv2's NPC logic runs and the agent can take it). Fails at
    /// once, naming the entity, when no agent is registered for it -- SSv2 then ends the
    /// scenario.
    fn update_agent_goal(&mut self, name: &str, req: &Goal) -> eyre::Result<()> {
        let target_speed = match req.target_speed {
            Some(v) if !(v.is_finite() && v >= 0.0) => {
                eyre::bail!("target speed {v} m/s is not a speed");
            }
            v => v,
        };
        let waypoints: Vec<AgentPose> = req.waypoints.iter().map(agent_pose).collect();
        let commands = AgentCommand::from_goal_request(&waypoints, req.clear, target_speed);
        if commands.is_empty() {
            return Ok(());
        }
        self.agent_link()
            .check_agent(name)
            .map_err(|e| eyre::eyre!("{e:#}"))?;
        for command in &commands {
            match command {
                AgentCommand::SetGoal { goal, waypoints } => tracing::info!(
                    "'{name}': goal ({:.1}, {:.1}) via {} waypoint(s) for its agent",
                    goal.x,
                    goal.y,
                    waypoints.len()
                ),
                other => tracing::info!("'{name}': {} for its agent", other.name()),
            }
        }
        self.agent_link().push(name, commands);
        Ok(())
    }
}

/// CARLA's default MiscObject blueprint, as the CARLA world server spawns it.
const DEFAULT_MISC_OBJECT: &str = "static.prop.streetbarrier";

/// The body a MiscObject wears: its asset key when that names a CARLA vehicle or prop, else
/// the default MiscObject.
fn misc_object_model(key: &str) -> &str {
    if key.starts_with("vehicle.") || key.starts_with("static.prop.") {
        key
    } else {
        DEFAULT_MISC_OBJECT
    }
}

/// The walker a pedestrian wears: its asset key when that names a CARLA walker, else none
/// (the world server picks one by the entity's name, 044 C).
fn pedestrian_model(key: &str) -> &str {
    if key.starts_with("walker.") {
        key
    } else {
        ""
    }
}

fn kind_name(t: EntityType) -> &'static str {
    match t {
        EntityType::Ego => "ego",
        EntityType::Vehicle => "vehicle",
        EntityType::Pedestrian => "pedestrian",
        EntityType::MiscObject => "misc_object",
    }
}

fn trace_kind(t: EntityType) -> gap_trace::Kind {
    match t {
        EntityType::Ego => gap_trace::Kind::Ego,
        EntityType::Vehicle => gap_trace::Kind::Vehicle,
        EntityType::Pedestrian => gap_trace::Kind::Pedestrian,
        EntityType::MiscObject => gap_trace::Kind::MiscObject,
    }
}

/// The scenario's weather as the world protocol carries it.
fn wire_weather(w: &ScenarioWeather) -> wire::ScenarioWeather {
    use wire::scenario_weather as sw;
    wire::ScenarioWeather {
        cloud_cover: w.cloud_cover_oktas.map(|oktas| sw::CloudCover {
            oktas: u32::from(oktas),
        }),
        sun: w.sun.map(|s| sw::Sun {
            azimuth: s.azimuth,
            elevation: s.elevation,
            illuminance: s.illuminance,
        }),
        fog: w
            .fog_visual_range
            .map(|visual_range| sw::Fog { visual_range }),
        precipitation: w.precipitation.map(|p| sw::Precipitation {
            r#type: match p.kind {
                PrecipitationType::Dry => sw::PrecipitationType::Dry,
                PrecipitationType::Rain => sw::PrecipitationType::Rain,
                PrecipitationType::Snow => sw::PrecipitationType::Snow,
            } as i32,
            intensity: p.intensity,
        }),
        wind: w.wind.map(|w| sw::Wind {
            direction: w.direction,
            speed: w.speed,
        }),
    }
}

fn unsupported(what: &str, needs: wire::Capability, sim: &str) -> BackendError {
    BackendError::Unsupported(format!(
        "{what}: the {sim} world server does not offer {}",
        world_client::capability_name(needs)
    ))
}

impl SimBackend for RemoteBackend {
    fn before_serving(&mut self) {
        // The configured weather, now that there is a world to apply it to; the server keeps
        // it across map loads.
        let name = self.config.weather.trim().to_string();
        if name.is_empty() || !self.offers(wire::Capability::Weather) {
            if !name.is_empty() {
                tracing::warn!(
                    "Weather '{name}' not applied: the {} world server has no weather",
                    self.sim()
                );
            }
            return;
        }
        match crate::weather::decode_request(&name, false, [0.0; 9]) {
            Ok(request) => {
                let reply = self.set_weather(request);
                tracing::info!("Weather '{name}': {}", reply.message);
            }
            Err(e) => tracing::error!("Weather '{name}': {e}"),
        }
    }

    fn initialize(&mut self, episode: &Episode) -> Result<i64, BackendError> {
        tracing::info!(
            "Initialize on {}: step_time={}, map {}",
            self.sim(),
            episode.step_time,
            episode.lanelet2_map_path
        );
        self.finish_ego_run(true);
        if !self.entities.is_empty() {
            self.log_tracking();
        }
        self.tracking_done.clear();
        if episode.lanelet2_map_path.is_empty() {
            return Err(BackendError::Failed(
                "Initialize supplied no lanelet2_map_path, so the world cannot be chosen".into(),
            ));
        }
        let name = self.server.name.clone();
        let (world, file) = WorldFile::for_map(&name, &episode.lanelet2_map_path)
            .map_err(|e| BackendError::Failed(format!("Cannot prepare the map: {e:#}")))?;
        match &file {
            Some(f) => tracing::info!(
                "{name} world from {}: {}, map origin {:?}",
                f.display(),
                if world.world.is_empty() {
                    "the server's default"
                } else {
                    &world.world
                },
                world.map_origin
            ),
            None => tracing::info!(
                "No {} beside the map: the server's default world, map frame = world frame",
                WorldFile::subpath(&name).display()
            ),
        }
        let map_dir = map_dir_of(Path::new(&episode.lanelet2_map_path));
        let signals = if self.offers(wire::Capability::Signals) {
            let signals = SignalTable::for_map_dir(&map_dir).map_err(|e| {
                BackendError::Failed(format!("Cannot read the signal table: {e:#}"))
            })?;
            match &signals.path {
                Some(p) if signals.is_empty() => {
                    tracing::warn!("{} lists no traffic signals", p.display())
                }
                Some(p) => {
                    tracing::info!("{} traffic signal(s) from {}", signals.len(), p.display())
                }
                None => tracing::info!(
                    "No {} beside the map: no traffic signals",
                    signals::TABLE_SUBPATH
                ),
            }
            signals
        } else {
            SignalTable::default()
        };
        let reply = self.call(
            "LoadWorld",
            wire::request::Request::LoadWorld(wire::LoadWorld {
                world: world.world.clone(),
                ground_paths: world.ground_paths.clone(),
                signals: signals.wire(world.map_origin),
                map_dir: map_dir.to_string_lossy().into_owned(),
                lanelet2_map_path: episode.lanelet2_map_path.clone(),
                map_origin: v3(world.map_origin),
                step_time: episode.step_time,
                realtime_factor: episode.realtime_factor,
            }),
            self.load_timeout,
        )?;
        let start_ns = match reply.reply {
            Some(wire::reply::Reply::LoadWorld(l)) => Some(l.sim_time_ns),
            _ => None,
        };
        // The world server shows every signal green until SSv2 commands it.
        self.signal_colours = signals
            .entries
            .iter()
            .map(|e| (e.opendrive_id.clone(), wire::Colour::Green))
            .collect();
        self.signals = signals;
        self.unmapped_signals.clear();
        self.map_origin = world.map_origin;
        self.entities.clear();
        self.names.clear();
        self.pending.clear();
        self.set_step_time(episode.step_time);
        self.frame = 0;
        self.episode += 1;
        let motion = match self.config.npc_motion {
            NpcMotion::Tracked => "tracked",
            NpcMotion::Kinematic => "kinematic",
        };
        self.gap_trace
            .begin_episode(self.episode, motion, contact_report::now_unix());
        tracing::info!(
            "Initialized on {} (step {} s = {} physics step(s) of {:.6} s)",
            self.sim(),
            self.step_time,
            self.substeps,
            self.dt
        );
        Ok(start_ns.unwrap_or_else(|| (self.sim_time * 1e9).round() as i64))
    }

    fn update_frame(&mut self, current_simulation_time: f64) -> Result<i64, BackendError> {
        self.frame += 1;
        if self.gap_trace.recording() {
            self.gap_trace
                .frame_time(self.frame, current_simulation_time);
        }
        let started = Instant::now();
        let reply = self.call(
            "Step",
            wire::request::Request::Step(wire::Step {
                poses: Vec::new(),
                substeps: self.substeps,
                dt: self.dt,
                render: self.config.world.render,
                current_simulation_time,
                want_states: self.gap_trace.recording()
                    || !self.offers(wire::Capability::ReadsBack),
            }),
            self.request_timeout,
        );
        let round_trip = started.elapsed();
        let step = match reply?.reply {
            Some(wire::reply::Reply::Step(s)) => s,
            other => {
                self.tick_time += round_trip;
                return Err(BackendError::Failed(format!(
                    "Step answered with {other:?}"
                )));
            }
        };
        // The frame budget excludes the simulator's own stepping, as it excluded CARLA's
        // tick: the server says how long that took; a server that does not say is charged
        // the whole round trip.
        self.tick_time += if step.tick_seconds > 0.0 {
            Duration::from_secs_f64(step.tick_seconds).min(round_trip)
        } else {
            round_trip
        };
        let dt = step.sim_time - self.sim_time;
        self.sim_time = step.sim_time;
        self.server_frame = step.frame;
        let origin = self.map_origin;
        let mut trace_states: BTreeMap<u32, (gap_trace::Pose6, Option<[f64; 3]>)> = BTreeMap::new();
        for state in &step.states {
            let Some(pose) = state.pose.as_ref() else {
                continue;
            };
            let now = state_pose6(pose, origin);
            let velocity = state
                .twist
                .as_ref()
                .and_then(|t| t.linear)
                .map(|v| [v.x, v.y, v.z]);
            trace_states.insert(state.handle, (now, velocity));
            let Some(name) = self.names.get(&state.handle) else {
                continue;
            };
            let Some(entity) = self.entities.get_mut(name) else {
                continue;
            };
            entity.motion = match state.twist.as_ref() {
                // The body's own velocity, where the server has it (tracked bodies, the ego
                // vehicle): a velocity step shows in one frame, not smeared over two by
                // differencing poses, so a one-frame jolt is one frame for the median.
                Some(t) => motion_from_twist(&entity.motion, &now, t, dt),
                None => motion((&entity.pose6, &entity.motion), &now, dt),
            };
            entity.accel_history.push_back(entity.motion.accel);
            if entity.accel_history.len() > ACCEL_SMOOTHING_FRAMES {
                entity.accel_history.pop_front();
            }
            entity.motion.accel = smoothed(&entity.accel_history);
            entity.pose = to_map(pose, origin);
            entity.pose6 = now;
            if let (Some(t), Some(v)) = (entity.tracked.as_mut(), velocity) {
                t.velocity = v;
            }
        }
        self.record_contacts(&step.contacts);
        if self.gap_trace.recording() {
            let by_id: BTreeMap<u32, (gap_trace::Pose6, [f64; 3])> = self
                .entities
                .values()
                .filter_map(|e| {
                    let (p, v) = trace_states.get(&e.handle)?;
                    Some((e.sim_id, (*p, v.unwrap_or(e.motion.velocity))))
                })
                .collect();
            self.gap_trace
                .emit_staged_at(self.frame, Some(step.frame), |id| by_id.get(&id).copied());
        }
        Ok(if step.sim_time_ns != 0 {
            step.sim_time_ns
        } else {
            (self.sim_time * 1e9).round() as i64
        })
    }

    fn update_step_time(&mut self, step_time: f64) -> Result<(), BackendError> {
        self.call(
            "SetStepTime",
            wire::request::Request::SetStepTime(wire::SetStepTime { step_time }),
            self.request_timeout,
        )?;
        self.set_step_time(step_time);
        tracing::info!(
            "step_time={step_time} on {}: {} physics step(s) of {:.6} s",
            self.sim(),
            self.substeps,
            self.dt
        );
        Ok(())
    }

    fn spawn(&mut self, req: &SpawnRequest) -> Result<(), BackendError> {
        use wire::Capability as C;
        let name = req.name.clone();
        let sim = self.sim().to_string();
        let tracked_npcs = self.config.npc_motion == NpcMotion::Tracked;
        let server_tracks = self.offers(C::TracksTargets);
        let (kind, entity_type, subtype, mover, role_name, agent, autopilot) = match &req.kind {
            EntityKind::Vehicle {
                driver, subtype, ..
            } => {
                let subtype = format!("{subtype:?}").to_lowercase();
                match driver {
                    Driver::Ego => (
                        wire::Kind::Ego,
                        EntityType::Ego,
                        subtype,
                        Mover::Simulator,
                        self.config.ego.role_name.clone(),
                        false,
                        false,
                    ),
                    Driver::Scenario => {
                        let mover = match (tracked_npcs, server_tracks) {
                            (true, true) => Mover::ServerTracked,
                            (true, false) if self.offers(C::TrackedMotion) => Mover::BridgeTracked,
                            _ => Mover::Kinematic,
                        };
                        (
                            wire::Kind::Vehicle,
                            EntityType::Vehicle,
                            subtype,
                            mover,
                            String::new(),
                            false,
                            false,
                        )
                    }
                    Driver::SimulatorAutopilot => {
                        if !self.offers(C::Autopilot) {
                            return Err(unsupported(
                                &format!("Cannot spawn '{name}' ({})", crate::autopilot::BEHAVIOR),
                                C::Autopilot,
                                &sim,
                            ));
                        }
                        (
                            wire::Kind::Vehicle,
                            EntityType::Vehicle,
                            subtype,
                            Mover::Simulator,
                            String::new(),
                            false,
                            true,
                        )
                    }
                    Driver::Agent => {
                        if !self.offers(C::AgentVehicles) {
                            return Err(unsupported(
                                &format!("Cannot spawn '{name}' ({})", agent_link::BEHAVIOR),
                                C::AgentVehicles,
                                &sim,
                            ));
                        }
                        (
                            wire::Kind::Vehicle,
                            EntityType::Vehicle,
                            subtype,
                            Mover::Simulator,
                            name.clone(),
                            true,
                            false,
                        )
                    }
                }
            }
            EntityKind::Pedestrian => (
                wire::Kind::Pedestrian,
                EntityType::Pedestrian,
                "pedestrian".to_string(),
                // A server that tracks walks its pedestrians (CARLA's WalkerControl); this
                // backend's law is for vehicles.
                if tracked_npcs && server_tracks {
                    Mover::ServerTracked
                } else {
                    Mover::Kinematic
                },
                String::new(),
                false,
                false,
            ),
            EntityKind::MiscObject => (
                wire::Kind::MiscObject,
                EntityType::MiscObject,
                String::new(),
                Mover::Kinematic,
                String::new(),
                false,
                false,
            ),
        };
        if self.entities.contains_key(&name) {
            return Err(BackendError::Failed(format!(
                "Cannot spawn '{name}': an entity of that name exists"
            )));
        }
        if entity_type == EntityType::Ego {
            if let Some(ego) = self
                .entities
                .iter()
                .find(|(_, e)| e.entity_type == EntityType::Ego)
            {
                return Err(BackendError::Failed(format!(
                    "Cannot spawn '{name}' as ego: ego '{}' already exists, and this bridge \
                     supports one ego per scenario",
                    ego.0
                )));
            }
        }
        let (center, dims, declared) = entity_box(req.bounding_box.as_ref(), kind);
        let (axles, performance) = match &req.kind {
            EntityKind::Vehicle {
                axles, performance, ..
            } => (*axles, *performance),
            _ => (None, None),
        };
        let tracked = (mover == Mover::BridgeTracked).then(|| {
            Tracked::new(
                performance,
                axles.and_then(|(front, rear)| npc_tracking::steering(front, rear, 70.0)),
            )
        });
        // The body's look: the CARLA blueprint the asset key maps to (`blueprint_map` first,
        // else the key itself). A MiscObject names a vehicle or a prop (`model3d`), else it
        // is CARLA's default MiscObject, the street barrier (044 D). A pedestrian names its
        // walker when the key is one (044 C). A server that resolves keys itself (CARLA) has
        // the raw key too.
        let key = self
            .config
            .blueprint_for(&req.asset_key)
            .unwrap_or(req.asset_key.as_str());
        let model = match entity_type {
            EntityType::Ego | EntityType::Vehicle => key.to_string(),
            EntityType::MiscObject => misc_object_model(key).to_string(),
            EntityType::Pedestrian => pedestrian_model(key).to_string(),
        };
        let pose = req.pose.unwrap_or(Pose {
            position: Some(geometry_msgs::Point::default()),
            orientation: Some(geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        });
        let motion = match mover {
            Mover::Kinematic => wire::Motion::Kinematic,
            Mover::BridgeTracked | Mover::ServerTracked => wire::Motion::Tracked,
            Mover::Simulator if autopilot => wire::Motion::Autopilot,
            Mover::Simulator => wire::Motion::External,
        };
        let reply = self.call(
            &format!("Spawn '{name}'"),
            wire::request::Request::Spawn(wire::Spawn {
                name: name.clone(),
                kind: kind as i32,
                subtype,
                box_center: v3(center),
                box_dimensions: v3(dims),
                pose: Some(to_world(&pose, self.map_origin)),
                model,
                motion: motion as i32,
                role_name,
                asset_key: req.asset_key.clone(),
                axles: axles.map(|(front, rear)| wire::Axles { front, rear }),
                performance: performance.map(|(a, d)| wire::Performance {
                    max_acceleration: a,
                    max_deceleration: d,
                }),
                box_declared: declared,
            }),
            // An ego brings its vehicle and sensors: the world server warms the sensors up
            // before it answers, seconds on a loaded host.
            if entity_type == EntityType::Ego {
                self.load_timeout
            } else {
                self.request_timeout
            },
        )?;
        let (handle, body) = match reply.reply {
            Some(wire::reply::Reply::Spawn(s)) if s.handle != 0 => (s.handle, s.body),
            other => {
                return Err(BackendError::Failed(format!(
                    "Spawn '{name}' answered with {other:?}"
                )))
            }
        };
        let body = body.unwrap_or_default();
        let sim_id = if body.sim_id != 0 {
            body.sim_id
        } else {
            handle
        };
        let p6 = pose6(&pose);
        self.entities.insert(
            name.clone(),
            Entity {
                handle,
                entity_type,
                mover,
                agent,
                autopilot,
                pose,
                pose6: p6,
                motion: Motion::default(),
                center,
                dims,
                sim_id,
                tracked,
                server_stats: (mover == Mover::ServerTracked).then(TrackingStats::default),
                accel_history: Default::default(),
            },
        );
        self.names.insert(handle, name.clone());
        if self.gap_trace.recording() {
            let vec = |v: Option<wire::Vec3>| {
                let v = v.unwrap_or_default();
                [v.x, v.y, v.z]
            };
            let offset = vec(body.origin_offset);
            self.gap_trace.entity(gap_trace::EntityHeader {
                frame: self.frame,
                name: name.clone(),
                kind: trace_kind(entity_type),
                declared: gap_trace::DeclaredBox {
                    center: vec(body.declared_center),
                    dimensions: vec(body.declared_dimensions),
                },
                origin_offset: [offset[0], offset[1], body.origin_yaw],
                actor_id: sim_id,
                blueprint: body.model.clone(),
                blueprint_box: body.has_box.then(|| gap_trace::BlueprintBox {
                    location: vec(body.box_location),
                    extent: vec(body.box_extent),
                }),
            });
        }
        if entity_type == EntityType::Ego {
            self.finish_ego_run(true);
            self.ego_run = Some(EgoRun {
                name: name.clone(),
                episode: self.episode,
                start_unix: contact_report::now_unix(),
                handle,
                contacts: Vec::new(),
                npc_contacts: Vec::new(),
            });
        }
        if agent {
            // Whether an agent is registered yet does not matter here: the vehicle side may
            // come up later. The agent is told where its vehicle is as soon as it is there; a
            // goal for an entity with no agent is what fails (UpdateEntityGoal).
            let at = agent_pose(&pose);
            self.agent_link().spawned(&name, at);
        }
        tracing::info!(
            "Spawned {} '{name}' on {} (handle {handle}, {}, box {:.2} x {:.2} x {:.2} m{})",
            kind_name(entity_type),
            self.sim(),
            body.model,
            dims[0],
            dims[1],
            dims[2],
            match mover {
                Mover::BridgeTracked => ", tracked",
                Mover::ServerTracked => ", tracked by the server",
                Mover::Simulator if autopilot => ", the simulator's autopilot",
                Mover::Simulator if agent => ", its agent",
                _ => "",
            }
        );
        Ok(())
    }

    fn despawn(&mut self, name: &str) -> Result<(), BackendError> {
        let Some(handle) = self.handle_of(name) else {
            return Err(BackendError::Failed(format!("Entity '{name}' not found")));
        };
        self.call(
            &format!("Despawn '{name}'"),
            wire::request::Request::Despawn(wire::Despawn { handle }),
            self.request_timeout,
        )?;
        let entity = self.entities.remove(name).expect("looked up above");
        self.names.remove(&handle);
        self.pending.remove(&handle);
        if let Some(stats) = entity.tracked.map(|t| t.stats).or(entity.server_stats) {
            self.tracking_done.insert(name.to_string(), stats);
        }
        if entity.entity_type == EntityType::Ego {
            self.finish_ego_run(true);
        }
        tracing::info!("Despawned '{name}' on {} (handle {handle})", self.sim());
        if self.entities.is_empty() {
            self.log_tracking();
            if self.gap_trace.recording() {
                // SSv2 despawns every entity when a scenario ends.
                self.gap_trace.end_episode();
            }
        }
        Ok(())
    }

    fn update_entity_status(&mut self, batch: StatusBatch) -> StatusOutcome {
        if let Some(link) = self.agent_link.as_ref() {
            link.set_npc_logic_started(batch.npc_logic_started);
        }
        let reads_back = self.offers(wire::Capability::ReadsBack);
        let mut unknown = Vec::new();
        let mut agent_failures = Vec::new();
        let origin = self.map_origin;
        // Every entity SSv2 sent, in its order, with what this backend makes of it.
        struct Row {
            status: EntityStatus,
            handle: u32,
            read_back: bool,
            flags: (bool, bool),
        }
        let mut rows = Vec::with_capacity(batch.entities.len());
        self.pending.clear();
        for status in batch.entities {
            let Some(entity) = self.entities.get(&status.name) else {
                unknown.push(format!("'{}'", status.name));
                continue;
            };
            let handle = entity.handle;
            let mover = entity.mover;
            let agent = entity.agent;
            let read_back = match entity.entity_type {
                EntityType::Ego => !batch.overwrite_ego_status,
                _ => mover == Mover::Simulator,
            };
            let mut flags = (false, entity.tracked.as_ref().is_some_and(|t| t.kinematic));
            if agent {
                if let Some(error) = self.agent_link.as_ref().and_then(|l| l.error(&status.name)) {
                    agent_failures.push(format!("{}: {error}", status.name));
                }
            }
            if read_back {
                // The simulator moves it: listed, so the server knows SSv2 still has it.
                self.pending.insert(
                    handle,
                    wire::EntityPose {
                        handle,
                        ..Default::default()
                    },
                );
            } else if mover == Mover::ServerTracked {
                // The server tracks it from SSv2's pose and twist; SSv2 holds its NPCs still
                // until their logic starts, while the twist already carries their Init
                // speed: no twist until then.
                self.pending.insert(
                    handle,
                    wire::EntityPose {
                        handle,
                        pose: status.pose.as_ref().map(|p| to_world(p, origin)),
                        ssv2_twist: status
                            .action_status
                            .as_ref()
                            .filter(|_| batch.npc_logic_started)
                            .and_then(ssv2_twist),
                        ..Default::default()
                    },
                );
            } else if entity.tracked.as_ref().is_some_and(|t| !t.kinematic) {
                // A bridge-tracked body (044): the law turns SSv2's pose into a velocity (or
                // a snap, or the fallback).
                if let Some(pose) = status.pose.as_ref() {
                    let twist = status
                        .action_status
                        .as_ref()
                        .filter(|_| batch.npc_logic_started)
                        .and_then(|a| a.twist.as_ref())
                        .map(|t| {
                            let l = t.linear.unwrap_or_default();
                            (l.x, l.y, t.angular.map_or(0.0, |a| a.z))
                        });
                    flags = self.track(&status.name, pose, twist);
                }
            } else if let Some(pose) = status.pose.as_ref() {
                // SSv2 moves it: the pose goes to the next step.
                if let Some(t) = self
                    .entities
                    .get_mut(&status.name)
                    .and_then(|e| e.tracked.as_mut())
                    .filter(|t| t.kinematic)
                {
                    t.stats.record_fallback();
                }
                self.pending.insert(
                    handle,
                    wire::EntityPose {
                        handle,
                        pose: Some(to_world(pose, origin)),
                        ..Default::default()
                    },
                );
            }
            rows.push(Row {
                status,
                handle,
                read_back,
                flags,
            });
        }

        // The drives go out now, as SSv2's UpdateEntityStatus; the server answers with what it
        // moves itself.
        let poses: Vec<wire::EntityPose> =
            std::mem::take(&mut self.pending).into_values().collect();
        #[cfg(test)]
        {
            self.sent = poses.iter().map(|p| (p.handle, *p)).collect();
        }
        let mut server_failure = None;
        let mut read_backs: BTreeMap<u32, wire::ReadBack> = BTreeMap::new();
        let mut samples: BTreeMap<u32, wire::TrackingSample> = BTreeMap::new();
        if self.wedged.is_none() {
            match self.call(
                "Apply",
                wire::request::Request::Apply(wire::Apply {
                    poses,
                    npc_logic_started: batch.npc_logic_started,
                    overwrite_ego_status: batch.overwrite_ego_status,
                }),
                self.request_timeout,
            ) {
                Ok(reply) => {
                    if let Some(wire::reply::Reply::Apply(a)) = reply.reply {
                        // Contacts of the last step the server learned of late: the last
                        // frame's, as the CARLA backend labelled them.
                        if !a.contacts.is_empty() {
                            self.record_contacts(&a.contacts);
                        }
                        read_backs = a.read_back.into_iter().map(|r| (r.handle, r)).collect();
                        samples = a.tracking.into_iter().map(|s| (s.handle, s)).collect();
                        if !a.failure.is_empty() {
                            server_failure = Some(a.failure);
                        }
                    }
                }
                Err(e) => server_failure = Some(e.to_string()),
            }
        } else {
            server_failure = self.wedged.clone();
        }

        let mut out = Vec::with_capacity(rows.len());
        let mut staged = Vec::new();
        for row in rows {
            let name = row.status.name.clone();
            let mut flags = row.flags;
            if let Some(s) = samples.get(&row.handle) {
                flags = (s.held, s.kinematic);
                if let Some(stats) = self
                    .entities
                    .get_mut(&name)
                    .and_then(|e| e.server_stats.as_mut())
                {
                    if s.measured {
                        stats.record(s.error, s.has_yaw_error.then_some(s.yaw_error), s.snapped);
                    }
                    if s.missed {
                        stats.record_missed();
                    }
                    if s.fallback {
                        stats.record_fallback();
                    }
                }
            }
            let reported = if row.read_back {
                match (reads_back, read_backs.get(&row.handle)) {
                    (true, Some(rb)) => read_back_status(&name, rb, origin),
                    // The server could not read it: echo what SSv2 sent, as CARLA always did.
                    (true, None) => row.status.clone(),
                    (false, _) => {
                        // Read back here, from the server's last step.
                        let entity = &self.entities[&name];
                        EntityStatus {
                            name: name.clone(),
                            action_status: Some(action_status(&entity.motion)),
                            pose: Some(entity.pose),
                        }
                    }
                }
            } else {
                row.status.clone()
            };
            if let (Some(pose), Some(entity)) = (reported.pose.as_ref(), self.entities.get(&name)) {
                staged.push(gap_trace::StagedEntity {
                    name: name.clone(),
                    actor_id: entity.sim_id,
                    ssv2: pose6(pose),
                    contact_hold: flags.0,
                    kinematic: flags.1,
                });
            }
            out.push(reported);
        }
        if self.gap_trace.recording() {
            self.gap_trace.stage_frame(self.frame + 1, staged);
        }
        let mut failure = (!unknown.is_empty()).then(|| {
            format!(
                "UpdateEntityStatus: unknown entit{} {} (not spawned through this bridge, or \
                 already despawned)",
                if unknown.len() == 1 { "y" } else { "ies" },
                unknown.join(", ")
            )
        });
        for (what, extra) in [
            (
                "the world server",
                server_failure.map(|f| vec![f]).unwrap_or_default(),
            ),
            ("an agent-driven vehicle cannot be driven", agent_failures),
        ] {
            if extra.is_empty() {
                continue;
            }
            let earlier = match &failure {
                None => String::from("UpdateEntityStatus: "),
                Some(earlier) => format!("{earlier}; "),
            };
            failure = Some(format!("{earlier}{what}: {}", extra.join("; ")));
        }
        StatusOutcome {
            entities: out,
            failure,
        }
    }

    fn update_entity_goal(&mut self, name: &str, goal: &Goal) -> Result<(), BackendError> {
        let Some(entity) = self.entities.get(name) else {
            return Err(BackendError::Failed(format!("Entity '{name}' not found")));
        };
        if entity.agent {
            return self.update_agent_goal(name, goal).map_err(|e| {
                BackendError::Failed(format!("UpdateEntityGoal for '{name}': {e:#}"))
            });
        }
        if !entity.autopilot {
            return Err(BackendError::Failed(format!(
                "'{name}' is not driven by the simulator (its controller is neither {} nor {})",
                crate::autopilot::BEHAVIOR,
                agent_link::BEHAVIOR
            )));
        }
        let handle = entity.handle;
        if let Some(v) = goal.target_speed {
            if !(v.is_finite() && v >= 0.0) {
                return Err(BackendError::Failed(format!(
                    "UpdateEntityGoal for '{name}': target speed {v} m/s is not a speed"
                )));
            }
        }
        let origin = self.map_origin;
        self.call(
            &format!("SetGoal '{name}'"),
            wire::request::Request::SetGoal(wire::SetGoal {
                handle,
                waypoints: goal.waypoints.iter().map(|p| to_world(p, origin)).collect(),
                clear: goal.clear,
                has_target_speed: goal.target_speed.is_some(),
                target_speed: goal.target_speed.unwrap_or_default(),
            }),
            self.request_timeout,
        )
        .map_err(|e| BackendError::Failed(format!("UpdateEntityGoal for '{name}': {e}")))?;
        Ok(())
    }

    fn update_environment(&mut self, environment: &Environment) -> Result<(), BackendError> {
        // What the server lacks, before anything is sent: a refusal changes nothing.
        let road = environment
            .road_condition
            .as_ref()
            // The default road changes nothing, whatever the simulator.
            .filter(|rc| self.offers(wire::Capability::RoadCondition) || !rc.is_default());
        if road.is_some() && !self.offers(wire::Capability::RoadCondition) {
            return Err(unsupported(
                "RoadCondition",
                wire::Capability::RoadCondition,
                self.sim(),
            ));
        }
        let weather = environment.weather.as_ref().filter(|w| !w.is_empty());
        let sky = weather.is_some() || environment.time_of_day.is_some();
        if sky && !self.offers(wire::Capability::Environment) {
            return Err(unsupported(
                "Weather and TimeOfDay",
                wire::Capability::Environment,
                self.sim(),
            ));
        }
        if let Some(rc) = road {
            let properties = rc
                .swb_properties()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect();
            tracing::info!(
                "RoadCondition: friction scale {}, {}, {:?}",
                rc.friction_scale_factor,
                rc.wetness,
                properties
            );
            self.call(
                "SetRoadCondition",
                wire::request::Request::SetRoadCondition(wire::SetRoadCondition {
                    road_condition: Some(wire::RoadCondition {
                        friction_scale_factor: rc.friction_scale_factor,
                        wetness: rc.wetness.clone(),
                        properties,
                    }),
                }),
                self.request_timeout,
            )?;
        }
        if sky {
            tracing::info!(
                "Environment: weather {:?}, time of day {:?}",
                weather,
                environment.time_of_day
            );
            self.call(
                "SetEnvironment",
                wire::request::Request::SetEnvironment(wire::SetEnvironment {
                    weather: weather.map(wire_weather),
                    time_of_day: environment.time_of_day.as_ref().map(|t| wire::TimeOfDay {
                        hours: t.hours,
                        day_of_year: t.day_of_year,
                        animation: t.animation,
                        date_time: t.date_time.clone(),
                    }),
                }),
                self.request_timeout,
            )?;
        }
        Ok(())
    }

    fn update_traffic_lights(&mut self, signals: &[TrafficSignal]) -> Result<(), BackendError> {
        if !self.offers(wire::Capability::Signals) {
            if signals.is_empty() {
                return Ok(());
            }
            return Err(unsupported(
                "UpdateTrafficLights",
                wire::Capability::Signals,
                self.sim(),
            ));
        }
        // SSv2 sends every signal's state every frame; only changes go to the world server.
        let mut changed = Vec::new();
        let mut unmapped = Vec::new();
        for signal in signals {
            let colour = signals::colour_for_signal(signal);
            let Some(opendrive_id) = u64::try_from(signal.id)
                .ok()
                .and_then(|id| self.signals.opendrive_id(id))
            else {
                if self.unmapped_signals.insert(signal.id) {
                    unmapped.push(signal.id);
                }
                continue;
            };
            if self.signal_colours.get(opendrive_id) != Some(&colour) {
                self.signal_colours.insert(opendrive_id.to_string(), colour);
                changed.push(wire::SignalState {
                    opendrive_id: opendrive_id.to_string(),
                    colour: colour as i32,
                });
            }
        }
        if !unmapped.is_empty() {
            // A configuration gap, not a request failure.
            tracing::warn!(
                "No lamp mapped for lanelet signal(s) {unmapped:?} (not in {}): these signals \
                 are not shown",
                self.signals
                    .path
                    .as_ref()
                    .map_or(signals::TABLE_SUBPATH.into(), |p| p.display().to_string())
            );
        }
        if changed.is_empty() {
            return Ok(());
        }
        tracing::debug!("{} signal colour(s) changed", changed.len());
        self.call(
            "SetSignals",
            wire::request::Request::SetSignals(wire::SetSignals { states: changed }),
            self.request_timeout,
        )?;
        Ok(())
    }

    fn attach_sensor(&mut self, _sensor: Sensor) -> Result<(), BackendError> {
        Err(BackendError::Unsupported(
            "sensors are published by the vehicle's own bridge (acb, aib), not attached by \
             the scenario"
                .into(),
        ))
    }

    fn get_weather(&mut self) -> WeatherReply {
        if !self.offers(wire::Capability::Weather) {
            return WeatherReply::failed(
                format!("the {} world server has no weather", self.sim()),
                None,
            );
        }
        match self.call(
            "GetWeather",
            wire::request::Request::GetWeather(wire::GetWeather {}),
            self.request_timeout,
        ) {
            Ok(wire::Reply {
                reply: Some(wire::reply::Reply::Weather(w)),
                ..
            }) => wire_form::reply_of(&w),
            Ok(_) => WeatherReply::failed("GetWeather answered with no weather", None),
            Err(e) => WeatherReply::failed(e.to_string(), None),
        }
    }

    fn set_weather(&mut self, request: WeatherRequest) -> WeatherReply {
        if !self.offers(wire::Capability::Weather) {
            return WeatherReply::failed(
                format!(
                    "the {} world server has no weather; nothing changed",
                    self.sim()
                ),
                None,
            );
        }
        match self.call(
            "SetWeather",
            wire::request::Request::SetWeather(wire::SetWeather {
                weather: Some(wire_form::request(&request)),
            }),
            self.load_timeout,
        ) {
            Ok(wire::Reply {
                reply: Some(wire::reply::Reply::Weather(w)),
                ..
            }) => wire_form::reply_of(&w),
            Ok(_) => WeatherReply::failed("SetWeather answered with no weather", None),
            Err(e) => WeatherReply::failed(e.to_string(), None),
        }
    }

    fn entity_count(&self) -> usize {
        self.entities.len()
    }

    fn take_tick_time(&mut self) -> Duration {
        std::mem::take(&mut self.tick_time)
    }

    fn probe_connection_idle(&mut self) {
        if self.server_lost.is_some() || self.wedged.is_some() {
            return;
        }
        match self.client.hello(false, budgets::IDLE_PROBE_TIMEOUT) {
            Ok(h) => {
                self.idle_probe_failures = 0;
                self.unanswered_since = None;
                if h.instance != self.server.instance {
                    let why = format!("instance {} after {}", h.instance, self.server.instance);
                    if !self.entities.is_empty() {
                        self.forget_server_state(&why);
                    } else {
                        tracing::info!("The {} world server restarted ({why})", self.sim());
                    }
                    self.server = ServerInfo::of(&h);
                }
            }
            Err(e) => {
                self.client.reset();
                self.idle_probe_failures += 1;
                let since = *self.unanswered_since.get_or_insert_with(Instant::now);
                tracing::warn!(
                    "Idle check: the {} world server did not answer hello ({e}); unanswered \
                     for {:.0} s of {} s",
                    self.sim(),
                    since.elapsed().as_secs_f64(),
                    budgets::WORLD_SERVER_LOST_AFTER.as_secs()
                );
                if self.idle_probe_failures >= budgets::IDLE_PROBE_FAILURES
                    && since.elapsed() >= budgets::WORLD_SERVER_LOST_AFTER
                {
                    let reason = format!(
                        "the {} world server at {} is gone",
                        self.sim(),
                        self.client.endpoint()
                    );
                    tracing::error!("{reason}");
                    self.server_lost = Some(reason);
                }
            }
        }
    }

    fn wedged(&self) -> Option<&str> {
        self.wedged.as_deref()
    }

    fn server_lost(&self) -> Option<&str> {
        self.server_lost.as_deref()
    }

    fn abandon_after_hang(&mut self) {
        self.finish_ego_run(false);
        self.gap_trace.mark_incomplete();
        self.gap_trace.end_episode();
        self.gap_trace.shutdown(Duration::from_secs(5));
    }

    fn shutdown(&mut self) {
        self.finish_ego_run(true);
        self.gap_trace.shutdown(Duration::from_secs(5));
        if self.wedged.is_some() {
            return;
        }
        // The world server outlives the bridge: it undoes what it did for this one and has
        // no owner after. Briefly: a bridge ending with its supervisor (killed, its world
        // server going too) must not wait on a server that is gone; the owner lease covers a
        // release that never arrived.
        match self.client.call(
            wire::request::Request::Release(wire::Release {}),
            RELEASE_TIMEOUT,
        ) {
            Ok(_) => tracing::info!("Released the {} world server", self.sim()),
            Err(e) => tracing::warn!("Could not release the {} world server: {e}", self.sim()),
        }
    }
}

#[cfg(test)]
mod tests;
