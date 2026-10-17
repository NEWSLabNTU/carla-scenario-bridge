//! Isaac Sim as csb's simulator: [`IsaacBackend`], through csb's world server (roadmap 037,
//! `docs/design/simulator-backends.md`).
//!
//! The world server (`src/csb_isaac`, run with `just isaac-world`) owns the Isaac stage and
//! the loop; this backend drives it over `proto/csb_world.proto`, ZMQ REQ to its REP, one
//! round trip per SSv2 frame: `UpdateEntityStatus` keeps the poses SSv2 sends and answers from
//! the states of the last step, `UpdateFrame` sends them in one `Step` and keeps the states
//! and contacts it returns. Nothing advances between requests.
//!
//! What this phase has (no Autoware yet, 038 brings the PhysX vehicle): every entity is a
//! kinematic body with a box collider at the declared box, posed by SSv2 -- the ego too, when
//! SSv2 drives it along a trajectory (`overwrite_ego_status`), else it stands where it
//! spawned. Simulator- and agent-driven vehicles, traffic lights, weather and SSv2's sensors
//! are `Unsupported`: honest failures (roadmap 006).
//!
//! Frames: Isaac's world frame is right-handed and Z-up like ROS, so a map-frame pose only
//! loses the map origin (`<map dir>/isaac/world.yaml`) on its way to the server. Poses are at
//! the SSv2 entity origin on both sides; the server puts the box around it. Sim time is the
//! server's, monotonic over its lifetime, so no episode clock is needed (cf. CARLA's, 015).
//!
//! A request the server does not answer in time wedges the bridge, as a CARLA tick that
//! never returned does: SSv2 gets the error, the bridge drains and exits 75, and its
//! supervisor starts a fresh one (034).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use prost::Message;
use serde::Deserialize;

use crate::backend::{
    BackendError, Driver, EntityKind, EntityStatus, Episode, Goal, Sensor, SimBackend,
    SpawnRequest, StatusBatch, StatusOutcome,
};
use crate::budgets;
use crate::config::BridgeConfig;
use crate::contact_report::{self, ContactReport, RawCollision, ReportContact};
use crate::coordinate_conversion::quaternion_to_euler;
use crate::entity_manager::EntityType;
use crate::gap_trace::{self, GapTrace};
use crate::map_resolver::map_dir_of;
use crate::proto::csb_world as wire;
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::simulation_api_schema::TrafficSignal;
use crate::proto::traffic_simulator_msgs::{ActionStatus, BoundingBox};
use crate::weather::WeatherRequest;
use crate::world_services::WeatherReply;

/// csb's world protocol version; the server's is `csb_isaac/protocol.py`'s.
pub const PROTOCOL_VERSION: u32 = 1;

/// Physics step the frame is split into, at most: Isaac's default 60 Hz.
const MAX_PHYSICS_DT: f64 = 1.0 / 60.0;

/// Where a map's Isaac world is described, under its map directory.
pub const WORLD_SUBPATH: &str = "isaac/world.yaml";

/// `<map dir>/isaac/world.yaml`: the Isaac world for a Lanelet2 map. Absent: a flat ground
/// plane at z = 0 with the map frame as the world frame.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct WorldFile {
    /// The USD stage, relative to the map directory or absolute; empty: the ground plane.
    pub usd: String,
    /// Prims a body rests on (terrain, road surface): contacts with them are not reported.
    pub ground_paths: Vec<String>,
    /// Where the world's origin sits in the map frame, m.
    pub map_origin: [f64; 3],
}

impl WorldFile {
    /// The world for the map at `lanelet2_map_path`, and where it was read from (`None`: the
    /// default). A file that does not parse is an error, not a silent default.
    pub fn for_map(lanelet2_map_path: &str) -> eyre::Result<(Self, Option<PathBuf>)> {
        let map_dir = map_dir_of(Path::new(lanelet2_map_path));
        let path = map_dir.join(WORLD_SUBPATH);
        if !path.exists() {
            return Ok((Self::default(), None));
        }
        let text = std::fs::read_to_string(&path)
            .map_err(|e| eyre::eyre!("read {}: {e}", path.display()))?;
        let mut world: Self = serde_yaml::from_str(&text)
            .map_err(|e| eyre::eyre!("parse {}: {e}", path.display()))?;
        if !world.usd.is_empty() && Path::new(&world.usd).is_relative() {
            world.usd = map_dir.join(&world.usd).to_string_lossy().into_owned();
        }
        Ok((world, Some(path)))
    }
}

/// Physics substeps and their length for an SSv2 step: the fewest steps of at most
/// [`MAX_PHYSICS_DT`] that make up the step exactly.
pub fn physics_steps(step_time: f64) -> (u32, f64) {
    let n = ((step_time / MAX_PHYSICS_DT) - 1e-9).ceil().max(1.0) as u32;
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

/// The declared box, or the kind's default when absent or degenerate.
fn entity_box(declared: Option<&BoundingBox>, kind: wire::Kind) -> ([f64; 3], [f64; 3]) {
    let fallback = default_box(kind);
    let Some(b) = declared else {
        return fallback;
    };
    let dims = b.dimensions.map(|d| [d.x, d.y, d.z]);
    match dims {
        Some(d) if d.iter().all(|v| v.is_finite() && *v > 0.0) => {
            let c = b.center.map_or(fallback.0, |c| [c.x, c.y, c.z]);
            (c, d)
        }
        _ => fallback,
    }
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

#[derive(Debug, Clone)]
struct Entity {
    handle: u32,
    entity_type: EntityType,
    /// The server's last pose of it, map frame, and its motion.
    pose: Pose,
    pose6: gap_trace::Pose6,
    motion: Motion,
}

/// The ego's contacts over its run, for the contact report (021).
#[derive(Debug)]
struct EgoRun {
    name: String,
    episode: u64,
    start_unix: f64,
    /// By other entity handle (0: the environment, then by path), in order of first contact.
    contacts: Vec<(u32, String, ReportContact)>,
}

/// Why a round trip to the world server failed.
#[derive(Debug)]
enum CallError {
    /// No reply within the bound: the server is hung or gone.
    Timeout(Duration),
    /// The server answered with an error.
    Refused(String),
    /// ZMQ or decoding failed.
    Io(String),
}

/// The REQ end of the world protocol. Relaxed and correlated, so a request after a timeout
/// is possible and a late reply to the old one is dropped.
struct Client {
    ctx: zmq::Context,
    endpoint: String,
    socket: zmq::Socket,
}

impl Client {
    fn connect(endpoint: &str) -> eyre::Result<Self> {
        let ctx = zmq::Context::new();
        let socket = Self::socket(&ctx, endpoint)?;
        Ok(Self {
            ctx,
            endpoint: endpoint.to_string(),
            socket,
        })
    }

    fn socket(ctx: &zmq::Context, endpoint: &str) -> eyre::Result<zmq::Socket> {
        let socket = ctx.socket(zmq::REQ)?;
        socket.set_req_relaxed(true)?;
        socket.set_req_correlate(true)?;
        socket.set_linger(0)?;
        socket.connect(endpoint)?;
        Ok(socket)
    }

    fn call(
        &mut self,
        request: wire::request::Request,
        timeout: Duration,
    ) -> Result<wire::Reply, CallError> {
        let bytes = wire::Request {
            request: Some(request),
        }
        .encode_to_vec();
        self.socket
            .set_rcvtimeo(timeout.as_millis().min(i32::MAX as u128) as i32)
            .map_err(|e| CallError::Io(e.to_string()))?;
        self.socket
            .send(bytes, 0)
            .map_err(|e| CallError::Io(format!("send: {e}")))?;
        let reply = match self.socket.recv_bytes(0) {
            Ok(b) => b,
            Err(zmq::Error::EAGAIN) => return Err(CallError::Timeout(timeout)),
            Err(e) => return Err(CallError::Io(format!("recv: {e}"))),
        };
        let reply = wire::Reply::decode(reply.as_slice())
            .map_err(|e| CallError::Io(format!("undecodable reply: {e}")))?;
        if reply.ok {
            Ok(reply)
        } else {
            Err(CallError::Refused(reply.error))
        }
    }

    /// A fresh socket, after a timeout left the old one waiting.
    fn reset(&mut self) {
        match Self::socket(&self.ctx, &self.endpoint) {
            Ok(s) => self.socket = s,
            Err(e) => tracing::warn!("World server socket not rebuilt: {e}"),
        }
    }

    /// `Hello`: the server's Isaac version, or why not.
    fn hello(&mut self, timeout: Duration) -> Result<String, CallError> {
        let reply = self.call(
            wire::request::Request::Hello(wire::Hello {
                protocol_version: PROTOCOL_VERSION,
            }),
            timeout,
        )?;
        match reply.reply {
            Some(wire::reply::Reply::Hello(h)) if h.protocol_version == PROTOCOL_VERSION => {
                Ok(h.isaac_version)
            }
            Some(wire::reply::Reply::Hello(h)) => Err(CallError::Refused(format!(
                "the world server speaks protocol {}, csb {PROTOCOL_VERSION}",
                h.protocol_version
            ))),
            other => Err(CallError::Io(format!("hello answered with {other:?}"))),
        }
    }
}

/// Connect to the configured world server, retrying every 5 s until it answers `Hello` in
/// this build's protocol version, and build the backend on it. `None` when `shutdown` was
/// set first. Isaac Sim takes 30 s or more to start, so csb may well come up first.
pub fn connect(config: BridgeConfig, shutdown: &AtomicBool) -> eyre::Result<Option<IsaacBackend>> {
    let endpoint = config.isaac.endpoint.clone();
    let mut client = Client::connect(&endpoint)?;
    let mut waited = Instant::now();
    loop {
        if shutdown.load(Ordering::SeqCst) {
            return Ok(None);
        }
        match client.hello(Duration::from_secs(2)) {
            Ok(isaac) => {
                tracing::info!(
                    "Isaac world server at {endpoint}: Isaac Sim {isaac}, protocol {PROTOCOL_VERSION}"
                );
                return Ok(Some(IsaacBackend::new(client, config)));
            }
            Err(e) => {
                if waited.elapsed() >= Duration::from_secs(60) || matches!(e, CallError::Refused(_))
                {
                    tracing::warn!("Isaac world server at {endpoint} not ready ({e:?}); waiting");
                    waited = Instant::now();
                }
                client.reset();
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    }
}

pub struct IsaacBackend {
    client: Client,
    config: BridgeConfig,
    request_timeout: Duration,
    load_timeout: Duration,
    /// By SSv2 name.
    entities: BTreeMap<String, Entity>,
    /// Handle to SSv2 name, for contacts.
    names: BTreeMap<u32, String>,
    /// World-frame poses for the next `Step`, by handle.
    pending: BTreeMap<u32, wire::Pose>,
    map_origin: [f64; 3],
    /// The world the last `Initialize` loaded, for the shutdown's clearing load.
    world: WorldFile,
    step_time: f64,
    substeps: u32,
    dt: f64,
    /// The server's sim time after the last step, s, and its step count.
    sim_time: f64,
    server_frame: u64,
    /// `UpdateFrame`s since `Initialize`.
    frame: u64,
    episode: u64,
    tick_time: Duration,
    wedged: Option<String>,
    server_lost: Option<String>,
    idle_probe_failures: u32,
    gap_trace: GapTrace,
    contact_dir: Option<PathBuf>,
    ego_run: Option<EgoRun>,
}

impl IsaacBackend {
    fn new(client: Client, config: BridgeConfig) -> Self {
        let (substeps, dt) = physics_steps(0.05);
        Self {
            client,
            request_timeout: Duration::from_secs(config.isaac.request_timeout_seconds.max(1)),
            load_timeout: Duration::from_secs(config.isaac.load_timeout_seconds.max(1)),
            gap_trace: GapTrace::new(config.gap_trace_dir()),
            contact_dir: config.contact_report_dir(),
            config,
            entities: BTreeMap::new(),
            names: BTreeMap::new(),
            pending: BTreeMap::new(),
            map_origin: [0.0; 3],
            world: WorldFile::default(),
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
            ego_run: None,
        }
    }

    /// One request; a timeout wedges the bridge (see the module docs).
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
            Err(CallError::Refused(e)) => Err(BackendError::Failed(format!("{what}: {e}"))),
            Err(CallError::Io(e)) => Err(BackendError::Disconnected(format!(
                "{what}: the world server at {}: {e}",
                self.config.isaac.endpoint
            ))),
            Err(CallError::Timeout(t)) => {
                let reason = format!(
                    "the Isaac world server did not answer {what} within {} s; csb ends this \
                     bridge so its supervisor starts a fresh one",
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
        (self.substeps, self.dt) = physics_steps(step_time);
    }

    /// End the ego's run: write its contact report (021), if reports are on.
    fn finish_ego_run(&mut self, complete: bool) {
        let Some(run) = self.ego_run.take() else {
            return;
        };
        let contacts: Vec<ReportContact> = run.contacts.into_iter().map(|(_, _, c)| c).collect();
        tracing::info!(
            "Contacts of ego '{}' on Isaac: {} other(s), {} contact frame(s)",
            run.name,
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
                tracking: Vec::new(),
                npc_contacts: Vec::new(),
            },
        );
    }

    /// The contacts of one step: into the ego's run and the gap trace.
    fn record_contacts(&mut self, contacts: &[wire::Contact]) {
        let mut traced = Vec::new();
        for c in contacts {
            let name = (c.other != 0)
                .then(|| self.names.get(&c.other).cloned())
                .flatten();
            let other_type = match &name {
                Some(n) => self
                    .entities
                    .get(n)
                    .map_or("entity", |e| kind_name(e.entity_type))
                    .to_string(),
                None => c.other_path.clone(),
            };
            if let Some(run) = self.ego_run.as_mut() {
                let key_path = if c.other == 0 {
                    c.other_path.clone()
                } else {
                    String::new()
                };
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
                            "Isaac contact: ego '{}' touches {} at frame {}",
                            run.name,
                            name.as_deref().unwrap_or(&c.other_path),
                            self.frame
                        );
                        run.contacts.push((
                            c.other,
                            key_path,
                            ReportContact {
                                other_id: (c.other != 0).then_some(c.other),
                                name: name.clone(),
                                other_type: other_type.clone(),
                                is_static: c.other == 0,
                                first_ssv2_frame: self.frame,
                                first_sim_time: self.sim_time,
                                count: 1,
                                max_impulse: c.impulse,
                            },
                        ));
                    }
                }
            }
            traced.push((
                RawCollision {
                    ssv2_frame: self.frame,
                    carla_frame: self.server_frame,
                    sim_time: self.sim_time,
                    other_id: (c.other != 0).then_some(c.other),
                    other_type,
                    impulse: c.impulse,
                    subject: None,
                },
                name,
            ));
        }
        if self.gap_trace.recording() {
            self.gap_trace.contacts(traced);
        }
    }

    fn handle_of(&self, name: &str) -> Option<u32> {
        self.entities.get(name).map(|e| e.handle)
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

impl SimBackend for IsaacBackend {
    fn before_serving(&mut self) {}

    fn initialize(&mut self, episode: &Episode) -> Result<i64, BackendError> {
        tracing::info!(
            "Initialize on Isaac: step_time={}, map {}",
            episode.step_time,
            episode.lanelet2_map_path
        );
        self.finish_ego_run(true);
        if episode.lanelet2_map_path.is_empty() {
            return Err(BackendError::Failed(
                "Initialize supplied no lanelet2_map_path, so the Isaac world cannot be chosen"
                    .into(),
            ));
        }
        let (world, file) = WorldFile::for_map(&episode.lanelet2_map_path)
            .map_err(|e| BackendError::Failed(format!("Cannot prepare the map: {e:#}")))?;
        match &file {
            Some(f) => tracing::info!(
                "Isaac world from {}: {}, map origin {:?}",
                f.display(),
                if world.usd.is_empty() {
                    "flat ground plane"
                } else {
                    &world.usd
                },
                world.map_origin
            ),
            None => tracing::info!(
                "No {WORLD_SUBPATH} beside the map: a flat ground plane, map frame = world frame"
            ),
        }
        self.call(
            "LoadWorld",
            wire::request::Request::LoadWorld(wire::LoadWorld {
                usd: world.usd.clone(),
                ground_paths: world.ground_paths.clone(),
            }),
            self.load_timeout,
        )?;
        self.map_origin = world.map_origin;
        self.world = world;
        self.entities.clear();
        self.names.clear();
        self.pending.clear();
        self.set_step_time(episode.step_time);
        self.frame = 0;
        self.episode += 1;
        self.gap_trace
            .begin_episode(self.episode, "kinematic", contact_report::now_unix());
        tracing::info!(
            "Initialized on Isaac (step {} s = {} physics step(s) of {:.6} s)",
            self.step_time,
            self.substeps,
            self.dt
        );
        Ok((self.sim_time * 1e9).round() as i64)
    }

    fn update_frame(&mut self, current_simulation_time: f64) -> Result<i64, BackendError> {
        self.frame += 1;
        if self.gap_trace.recording() {
            self.gap_trace
                .frame_time(self.frame, current_simulation_time);
        }
        let poses = std::mem::take(&mut self.pending)
            .into_iter()
            .map(|(handle, pose)| wire::EntityPose {
                handle,
                pose: Some(pose),
            })
            .collect();
        let started = Instant::now();
        let reply = self.call(
            "Step",
            wire::request::Request::Step(wire::Step {
                poses,
                substeps: self.substeps,
                dt: self.dt,
                render: self.config.isaac.render,
            }),
            self.request_timeout,
        );
        self.tick_time += started.elapsed();
        let step = match reply?.reply {
            Some(wire::reply::Reply::Step(s)) => s,
            other => {
                return Err(BackendError::Failed(format!(
                    "Step answered with {other:?}"
                )))
            }
        };
        let dt = step.sim_time - self.sim_time;
        self.sim_time = step.sim_time;
        self.server_frame = step.frame;
        let origin = self.map_origin;
        for state in &step.states {
            let Some(name) = self.names.get(&state.handle) else {
                continue;
            };
            let Some(entity) = self.entities.get_mut(name) else {
                continue;
            };
            let Some(pose) = state.pose.as_ref() else {
                continue;
            };
            let pose = to_map(pose, origin);
            let now = pose6(&pose);
            entity.motion = motion((&entity.pose6, &entity.motion), &now, dt);
            entity.pose = pose;
            entity.pose6 = now;
        }
        self.record_contacts(&step.contacts);
        if self.gap_trace.recording() {
            let by_handle: BTreeMap<u32, (gap_trace::Pose6, [f64; 3])> = self
                .entities
                .values()
                .map(|e| (e.handle, (e.pose6, e.motion.velocity)))
                .collect();
            self.gap_trace
                .emit_staged_at(self.frame, Some(step.frame), |h| by_handle.get(&h).copied());
        }
        Ok((self.sim_time * 1e9).round() as i64)
    }

    fn update_step_time(&mut self, step_time: f64) -> Result<(), BackendError> {
        self.set_step_time(step_time);
        tracing::info!(
            "step_time={step_time} on Isaac: {} physics step(s) of {:.6} s",
            self.substeps,
            self.dt
        );
        Ok(())
    }

    fn spawn(&mut self, req: &SpawnRequest) -> Result<(), BackendError> {
        let name = req.name.clone();
        let (kind, entity_type, subtype) = match &req.kind {
            EntityKind::Vehicle {
                driver, subtype, ..
            } => {
                let subtype = format!("{subtype:?}").to_lowercase();
                match driver {
                    Driver::Ego => (wire::Kind::Ego, EntityType::Ego, subtype),
                    Driver::Scenario => (wire::Kind::Vehicle, EntityType::Vehicle, subtype),
                    Driver::SimulatorAutopilot => {
                        return Err(BackendError::Unsupported(format!(
                            "Cannot spawn '{name}' ({}): Isaac Sim has no traffic of its own \
                             to drive it",
                            crate::autopilot::BEHAVIOR
                        )))
                    }
                    Driver::Agent => {
                        return Err(BackendError::Unsupported(format!(
                            "Cannot spawn '{name}' ({}): agent-driven vehicles on Isaac arrive \
                             with its vehicle bridge (roadmap 038, 039)",
                            crate::agent_link::BEHAVIOR
                        )))
                    }
                }
            }
            EntityKind::Pedestrian => (
                wire::Kind::Pedestrian,
                EntityType::Pedestrian,
                "pedestrian".to_string(),
            ),
            EntityKind::MiscObject => (
                wire::Kind::MiscObject,
                EntityType::MiscObject,
                String::new(),
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
                    "Cannot spawn '{name}' as ego: ego '{}' already exists, and this backend \
                     supports one ego per scenario",
                    ego.0
                )));
            }
        }
        let (center, dims) = entity_box(req.bounding_box.as_ref(), kind);
        let pose = req.pose.unwrap_or(Pose {
            position: Some(geometry_msgs::Point::default()),
            orientation: Some(geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        });
        let v3 = |a: [f64; 3]| {
            Some(wire::Vec3 {
                x: a[0],
                y: a[1],
                z: a[2],
            })
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
            }),
            self.request_timeout,
        )?;
        let handle = match reply.reply {
            Some(wire::reply::Reply::Spawn(s)) if s.handle != 0 => s.handle,
            other => {
                return Err(BackendError::Failed(format!(
                    "Spawn '{name}' answered with {other:?}"
                )))
            }
        };
        let p6 = pose6(&pose);
        self.entities.insert(
            name.clone(),
            Entity {
                handle,
                entity_type,
                pose,
                pose6: p6,
                motion: Motion::default(),
            },
        );
        self.names.insert(handle, name.clone());
        if self.gap_trace.recording() {
            self.gap_trace.entity(gap_trace::EntityHeader {
                frame: self.frame,
                name: name.clone(),
                kind: trace_kind(entity_type),
                declared: gap_trace::DeclaredBox {
                    center,
                    dimensions: dims,
                },
                // Isaac's body is posed at the entity origin: no offset to undo.
                origin_offset: [0.0; 3],
                actor_id: handle,
                blueprint: "isaac:box".to_string(),
                blueprint_box: Some(gap_trace::BlueprintBox {
                    location: center,
                    extent: [dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0],
                }),
            });
        }
        if entity_type == EntityType::Ego {
            self.finish_ego_run(true);
            self.ego_run = Some(EgoRun {
                name: name.clone(),
                episode: self.episode,
                start_unix: contact_report::now_unix(),
                contacts: Vec::new(),
            });
        }
        tracing::info!(
            "Spawned {} '{name}' on Isaac (handle {handle}, box {:.2} x {:.2} x {:.2} m)",
            kind_name(entity_type),
            dims[0],
            dims[1],
            dims[2]
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
        if entity.entity_type == EntityType::Ego {
            self.finish_ego_run(true);
        }
        tracing::info!("Despawned '{name}' on Isaac (handle {handle})");
        if self.entities.is_empty() && self.gap_trace.recording() {
            // SSv2 despawns every entity when a scenario ends (as on CARLA).
            self.gap_trace.end_episode();
        }
        Ok(())
    }

    fn update_entity_status(&mut self, batch: StatusBatch) -> StatusOutcome {
        let mut out = Vec::with_capacity(batch.entities.len());
        let mut unknown = Vec::new();
        let mut staged = Vec::new();
        let origin = self.map_origin;
        for status in batch.entities {
            let Some(entity) = self.entities.get(&status.name) else {
                unknown.push(format!("'{}'", status.name));
                continue;
            };
            let read_back = entity.entity_type == EntityType::Ego && !batch.overwrite_ego_status;
            let reported = if read_back {
                // Isaac moves it (here: it stands where it is); SSv2 adopts the pose.
                EntityStatus {
                    name: status.name.clone(),
                    action_status: Some(action_status(&entity.motion)),
                    pose: Some(entity.pose),
                }
            } else {
                // SSv2 moves it: the pose goes to the next step, SSv2's status is echoed.
                if let Some(pose) = status.pose.as_ref() {
                    self.pending.insert(entity.handle, to_world(pose, origin));
                }
                status.clone()
            };
            if let Some(pose) = reported.pose.as_ref() {
                staged.push(gap_trace::StagedEntity {
                    name: status.name.clone(),
                    actor_id: entity.handle,
                    ssv2: pose6(pose),
                    contact_hold: false,
                    kinematic: false,
                });
            }
            out.push(reported);
        }
        if self.gap_trace.recording() {
            self.gap_trace.stage_frame(self.frame + 1, staged);
        }
        StatusOutcome {
            entities: out,
            failure: (!unknown.is_empty()).then(|| {
                format!(
                    "UpdateEntityStatus: unknown entit{} {} (not spawned through this bridge, or \
                     already despawned)",
                    if unknown.len() == 1 { "y" } else { "ies" },
                    unknown.join(", ")
                )
            }),
        }
    }

    fn update_entity_goal(&mut self, name: &str, _goal: &Goal) -> Result<(), BackendError> {
        if !self.entities.contains_key(name) {
            return Err(BackendError::Failed(format!("Entity '{name}' not found")));
        }
        Err(BackendError::Unsupported(format!(
            "'{name}' is not driven by the simulator: Isaac has no simulator- or agent-driven \
             vehicles yet (roadmap 038, 039)"
        )))
    }

    fn update_traffic_lights(&mut self, signals: &[TrafficSignal]) -> Result<(), BackendError> {
        // SSv2 sends the states every frame, an empty list when the scenario has no lights.
        if signals.is_empty() {
            return Ok(());
        }
        Err(BackendError::Unsupported(format!(
            "{} traffic light state(s) commanded, but the Isaac world has no traffic lights",
            signals.len()
        )))
    }

    fn attach_sensor(&mut self, _sensor: Sensor) -> Result<(), BackendError> {
        Err(BackendError::Unsupported(
            "Isaac sensors are published by csb's vehicle bridge (roadmap 038)".into(),
        ))
    }

    fn get_weather(&self) -> WeatherReply {
        WeatherReply::failed("the Isaac backend has no weather", None)
    }

    fn set_weather(&mut self, _request: WeatherRequest) -> WeatherReply {
        WeatherReply::failed("the Isaac backend has no weather; nothing changed", None)
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
        match self.client.hello(budgets::IDLE_PROBE_TIMEOUT) {
            Ok(_) => self.idle_probe_failures = 0,
            Err(e) => {
                self.client.reset();
                self.idle_probe_failures += 1;
                tracing::warn!(
                    "Idle check: the Isaac world server did not answer hello ({e:?}); {} of {}",
                    self.idle_probe_failures,
                    budgets::IDLE_PROBE_FAILURES
                );
                if self.idle_probe_failures >= budgets::IDLE_PROBE_FAILURES {
                    let reason = format!(
                        "the Isaac world server at {} is gone",
                        self.config.isaac.endpoint
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
        if self.wedged.is_some() || self.entities.is_empty() {
            return;
        }
        // The world server outlives csb: leave it empty, as teardown leaves CARLA.
        let empty = wire::request::Request::LoadWorld(wire::LoadWorld {
            usd: self.world.usd.clone(),
            ground_paths: self.world.ground_paths.clone(),
        });
        if let Err(e) = self.call("LoadWorld (shutdown)", empty, Duration::from_secs(5)) {
            tracing::warn!("Could not clear the Isaac world at shutdown: {e}");
        }
    }
}

#[cfg(test)]
mod tests;
