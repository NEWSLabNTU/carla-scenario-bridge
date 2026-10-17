//! [`CarlaBackend`]: SSv2's requests as CARLA API calls (formerly the coordinator, roadmap
//! 035). Each public handler is a [`SimBackend`] method's body; the trait impl near the end
//! of the file delegates to them.

use carla::client::{ActorBase, Client, World};
use carla::geom::{Location, Rotation, Transform};
use eyre::{Result, WrapErr};
use std::collections::{HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use super::gap_trace::{self, GapTrace};
use super::traffic_light_mapper::{carla_state_for_signal, signal_uses_arrows, SignalMap};
use crate::agent_link::{self, AgentLink, AgentPose, Command as AgentCommand};
use crate::autopilot;
use crate::blueprint_boxes::{self, BoxTable, Dimensions, SizeClass};
use crate::body_clearance::{self, BodyBox};
use crate::budgets::{self, Deadline, HUNG_STATE_READ_WAIT};
use crate::config::{BridgeConfig, NpcMotion, SensorReleaseConfig};
use crate::map_resolver::{lanelet_map_file, load_map_origin, town_from_map_path};
use crate::tick_watchdog::{HangHook, WatchError, Watchdog};

/// Minimum spawn height when the ground cannot be probed.
const FALLBACK_SPAWN_Z: f32 = 0.5;

/// Clearance above the ground to spawn at, so the vehicle settles onto its suspension
/// rather than starting interpenetrated with the road.
const SPAWN_CLEARANCE: f32 = 0.3;

/// Frames after spawn during which a physics actor's reported acceleration is zeroed.
/// The drop from SPAWN_CLEARANCE onto the suspension shows up as a spike (observed
/// -11 m/s² longitudinal at 30 Hz), which SSv2's entity sanity bounds ([-5, 3] m/s²)
/// treat as scenario-fatal. Half a second covers the bounce at any supported step time.
const SETTLE_FRAMES: u32 = 15;

/// Frames of raw CARLA acceleration combined before reporting. PhysX delivers
/// single-frame contact jolts (suspension contact, curb touches, brake grab) that no real
/// accelerometer-and-consumer chain would see unfiltered. SSv2 validates reported
/// acceleration against the scenario's declared performance bounds, so one such frame is
/// scenario-fatal.
///
/// The window is combined with a **median, not a mean**. A mean passes a fraction of every
/// outlier through: averaging three frames leaves a third of a one-frame jolt in the
/// reported value, which is how a 52 m/s² jolt was still reported as 17.5 and aborted a
/// scenario against SSv2's ±15 bound. A median of three discards a single outlier
/// outright, because two ordinary neighbours outvote it, while a genuine sustained profile
/// -- two or more consecutive frames agreeing -- passes through unchanged. Same window,
/// same latency, the filter that actually matches the noise.
const ACCEL_SMOOTHING_FRAMES: usize = 3;

/// Frames after a hand-over to Traffic Manager during which the vehicle's acceleration is
/// reported as zero (see `engage_autopilot`): one second at 20 Hz.
const ENGAGE_SETTLE_FRAMES: u32 = 20;

/// Extra height added on each spawn retry after a collision.
const SPAWN_RETRY_STEP: f32 = 0.5;

/// How many times to retry a colliding spawn before giving up.
const SPAWN_RETRIES: usize = 4;

/// Consecutive CARLA failures before the bridge concludes the connection is gone.
const MAX_CONSECUTIVE_CARLA_FAILURES: u32 = 3;

/// What is being spawned, and how it must behave once placed.
///
/// The distinction that matters is pose authority (invariant 5): the ego is driven by CARLA
/// PhysX under Autoware's control, everything else is teleported by SSv2. An actor must
/// never have both, which is what `physics_driven` decides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SpawnKind {
    /// The scenario ego. CARLA PhysX moves it; Autoware steers it.
    Ego,
    /// A scenario NPC vehicle. SSv2 computes its path and teleports it each frame.
    Npc,
    /// A scenario NPC vehicle whose controller is `simulator_autopilot`: CARLA PhysX moves
    /// it, Traffic Manager steers it, and SSv2 adopts the pose csb reads back (`autopilot`).
    AutopilotNpc,
    /// A scenario pedestrian. Also teleported -- SSv2's behaviour plugins own the walk.
    Pedestrian,
    /// A static obstacle. Placed once and left alone unless SSv2 moves it.
    MiscObject,
    /// A scenario vehicle whose controller is `agent`: CARLA PhysX moves it under the
    /// control of the autopilot behind the agent registered as its entity name (typically
    /// a full Autoware); SSv2 adopts the pose csb reads back (`agent_link`).
    AgentVehicle,
}

/// Which blueprint a spawn should use once CARLA has been asked about them.
///
/// Separated from [`CarlaBackend::resolve_blueprint_key`] so the policy can be tested
/// without a running server: `actor_builder` takes `&mut World` and needs a live CARLA,
/// which would otherwise make the fallback and unknown-key paths untestable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BlueprintChoice {
    /// The requested key is a real CARLA blueprint; use it unchanged.
    Requested,
    /// The requested key is unknown but the kind's default works; warn and substitute.
    Fallback,
    /// Neither is usable, so the spawn cannot proceed.
    Neither,
}

/// `fallback_exists` is only consulted when the requested key is missing, so callers that
/// have not probed it may pass `None`.
fn choose_blueprint(requested_exists: bool, fallback_exists: Option<bool>) -> BlueprintChoice {
    if requested_exists {
        return BlueprintChoice::Requested;
    }
    match fallback_exists {
        Some(true) => BlueprintChoice::Fallback,
        _ => BlueprintChoice::Neither,
    }
}

impl SpawnKind {
    /// Blueprint used when the scenario's asset key does not name a CARLA blueprint.
    fn default_blueprint(self) -> &'static str {
        match self {
            SpawnKind::Ego | SpawnKind::Npc | SpawnKind::AutopilotNpc | SpawnKind::AgentVehicle => {
                "vehicle.tesla.model3"
            }
            SpawnKind::Pedestrian => "walker.pedestrian.0001",
            SpawnKind::MiscObject => "static.prop.streetbarrier",
        }
    }

    /// The blueprints this kind may be size-matched among when the scenario names no
    /// blueprint (roadmap 022), or `None` when its body is not the scenario's to choose:
    /// the ego's and an agent vehicle's blueprint decide acb's sensor mounts and vehicle
    /// parameters, so the ego stack's configuration picks it; CARLA's walkers are all
    /// within centimetres of each other, so a pedestrian keeps its default and the
    /// spawn-time comparison reports the difference.
    fn size_class(self, subtype: EntitySubtype) -> Option<SizeClass> {
        match self {
            SpawnKind::Ego | SpawnKind::AgentVehicle | SpawnKind::Pedestrian => None,
            SpawnKind::Npc | SpawnKind::AutopilotNpc => SizeClass::of_vehicle(subtype),
            SpawnKind::MiscObject => Some(SizeClass::MiscObject),
        }
    }

    /// The SSv2 entity type this actor is registered as, if any. Every kind is a scenario
    /// entity since background AVs became one (controller `agent`, roadmap 017); the pose
    /// authority question is settled by `physics_driven` and the readback, not by hiding
    /// the vehicle from SSv2.
    fn entity_type(self) -> Option<EntityType> {
        match self {
            SpawnKind::Ego => Some(EntityType::Ego),
            SpawnKind::Npc | SpawnKind::AutopilotNpc | SpawnKind::AgentVehicle => {
                Some(EntityType::Vehicle)
            }
            SpawnKind::Pedestrian => Some(EntityType::Pedestrian),
            SpawnKind::MiscObject => Some(EntityType::MiscObject),
        }
    }

    /// Moved by physics toward SSv2's pose under `npc_motion: tracked` (the default, roadmap
    /// 023): SSv2-driven vehicles and pedestrians. Misc objects stay put by teleport.
    fn trackable(self) -> bool {
        matches!(self, SpawnKind::Npc | SpawnKind::Pedestrian)
    }

    /// Whether CARLA physics drives this actor on its own.
    ///
    /// False means SSv2 owns the pose. Such an actor is either tracked (physics on, but
    /// driven toward SSv2's pose by a closed loop, `trackable`) or kinematic (physics off at
    /// spawn: misc objects, and every SSv2-driven actor under `npc_motion: kinematic`).
    /// Physics on with plain per-frame `set_transform` would give the actor two authorities:
    /// `set_transform` places it, then PhysX pulls it down and shoves it out of collisions
    /// before the next frame.
    fn physics_driven(self) -> bool {
        // Driven through CARLA physics -- by an Autoware, or by Traffic Manager -- so none of
        // them may be teleported.
        matches!(
            self,
            SpawnKind::Ego | SpawnKind::AgentVehicle | SpawnKind::AutopilotNpc
        )
    }

    fn label(self) -> &'static str {
        match self {
            SpawnKind::Ego => "ego",
            SpawnKind::Npc => "NPC vehicle",
            SpawnKind::AutopilotNpc => "simulator-driven vehicle",
            SpawnKind::Pedestrian => "pedestrian",
            SpawnKind::MiscObject => "misc object",
            SpawnKind::AgentVehicle => "agent-driven vehicle",
        }
    }
}

/// Outcome of a teardown sweep, so the log can state what actually happened.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct TeardownReport {
    pub attempted: usize,
    pub destroyed: usize,
    pub failed: usize,
}

/// Report what a destroy-with-children sweep left behind.
///
/// Kept separate from the sweep itself because carla-rust does not log: it hands back a
/// [`carla::client::DestroyOutcome`] and lets the caller decide what is worth saying.
fn log_destroy_outcome(actor_id: u32, outcome: &carla::client::DestroyOutcome) {
    if let Some(e) = &outcome.list_error {
        tracing::warn!(
            "Could not list actors to detach children from {actor_id}: {e}. Any attached \
             sensors are now orphaned, and the server may crash when it next ticks them."
        );
    }
    for (sensor_id, e) in outcome.orphans() {
        tracing::warn!(
            "Could not destroy actor {sensor_id} attached to {actor_id}: {e}. \
             The server may crash when it next ticks it."
        );
    }
    if outcome.children_destroyed > 0 {
        tracing::debug!(
            "Destroyed {} actor(s) attached to {actor_id}",
            outcome.children_destroyed
        );
    }
    if !outcome.destroyed {
        tracing::debug!("Actor {actor_id} was already destroyed");
    }
}

/// Destroy one actor by ID in a given world. A missing actor counts as destroyed: the
/// requested end state holds either way.
fn destroy_actor_in(world: &carla::client::World, actor_id: u32) -> Result<()> {
    let actor = world
        .actor(actor_id)
        .wrap_err_with(|| format!("look up actor {actor_id}"))?;

    match actor {
        Some(actor) => {
            // Children first, or the server ticks them against a dead owner: CARLA leaves
            // a destroyed vehicle's sensors alive, and an orphaned IMU segfaults the
            // server on its next tick. carla-rust's helper reads attachment from the
            // server's actor list, which is what makes it work across clients -- acb
            // attached these sensors, not us.
            let outcome = actor
                .destroy_with_children()
                .wrap_err_with(|| format!("destroy actor {actor_id}"))?;
            log_destroy_outcome(actor_id, &outcome);
        }
        None => tracing::debug!("Actor {actor_id} is no longer in the world"),
    }
    Ok(())
}

/// Whether this bridge froze CARLA's traffic lights, and therefore owes an unfreeze.
///
/// A separate type so the "only undo what we did" rule is testable on its own. CARLA is
/// shared: unfreezing lights this bridge never froze would stomp whatever else set them,
/// and leaving our own freeze behind strands the next scenario on the last one's states.
#[derive(Debug, Default)]
pub struct FreezeGuard {
    frozen: bool,
}

impl FreezeGuard {
    fn mark_frozen(&mut self) {
        self.frozen = true;
    }

    fn mark_restored(&mut self) {
        self.frozen = false;
    }

    fn needs_restore(&self) -> bool {
        self.frozen
    }
}

/// The set of actors this bridge created and has not yet destroyed.
///
/// Deliberately separate from `EntityManager`, which is a name lookup that gets emptied on
/// despawn and on re-initialise: tying teardown to it is exactly what leaked every actor
/// whose name mapping was dropped first (phase 007). The ledger only forgets an actor once
/// something has confirmed it gone.
///
/// It owns the teardown loop rather than just the set, so the bookkeeping can be tested
/// without a CARLA world -- the caller supplies the destroyer.
#[derive(Debug, Default)]
pub struct SpawnLedger {
    ids: HashSet<u32>,
}

impl SpawnLedger {
    fn record(&mut self, actor_id: u32) {
        self.ids.insert(actor_id);
    }

    fn forget(&mut self, actor_id: u32) {
        self.ids.remove(&actor_id);
    }

    fn clear(&mut self) {
        self.ids.clear();
    }

    fn len(&self) -> usize {
        self.ids.len()
    }

    fn contains(&self, actor_id: u32) -> bool {
        self.ids.contains(&actor_id)
    }

    fn max_id(&self) -> Option<u32> {
        self.ids.iter().copied().max()
    }

    /// Destroy everything tracked, in ascending id order so a failure is reproducible.
    ///
    /// An actor is forgotten only when `destroy` reports success; one that fails stays on
    /// the books for the next attempt. Returns the report and the ids that were destroyed,
    /// so the caller can drop whatever else it keys by actor id.
    fn teardown<F>(&mut self, mut destroy: F) -> (TeardownReport, Vec<u32>)
    where
        F: FnMut(u32) -> Result<()>,
    {
        let mut report = TeardownReport {
            attempted: self.ids.len(),
            ..Default::default()
        };
        let mut destroyed = Vec::new();
        if report.attempted == 0 {
            return (report, destroyed);
        }

        let mut ids: Vec<u32> = self.ids.iter().copied().collect();
        ids.sort_unstable();
        for actor_id in ids {
            match destroy(actor_id) {
                Ok(()) => {
                    report.destroyed += 1;
                    destroyed.push(actor_id);
                    self.ids.remove(&actor_id);
                }
                Err(e) => {
                    report.failed += 1;
                    tracing::warn!("Teardown: failed to destroy actor {actor_id}: {e}");
                }
            }
        }
        (report, destroyed)
    }
}

/// Heights to try when a spawn collides, in order.
///
/// Only the height varies. The commanded x/y is SSv2's to decide (invariant 5), and quietly
/// relocating a vehicle to a different point would put CARLA and SSv2 into exactly the kind
/// of silent disagreement phase 006 removed. Lifting slightly is the standard CARLA
/// workaround for "Spawn failed because of collision at spawn position" and self-corrects as
/// the vehicle settles.
fn spawn_retry_heights(base_z: f32) -> impl Iterator<Item = f32> {
    (0..=SPAWN_RETRIES).map(move |attempt| base_z + attempt as f32 * SPAWN_RETRY_STEP)
}

/// Spawn height for a known ground height.
fn spawn_height_above_ground(ground_z: f32) -> f32 {
    ground_z + SPAWN_CLEARANCE
}

/// Acceleration along the entity's heading, in m/s².
///
/// Median of up to [`ACCEL_SMOOTHING_FRAMES`] samples, used to reject PhysX's one-frame
/// contact jolts (see that constant for why a mean will not do).
///
/// With an even count there is no middle sample, so the two straddling it are averaged --
/// the usual definition. That only arises on the first frame after a spawn, before the
/// window has filled; from then on the count is odd and one outlier is always outvoted.
fn median_of(values: impl Iterator<Item = f64>) -> f64 {
    let mut v: Vec<f64> = values.collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = v.len() / 2;
    if v.len() % 2 == 1 {
        v[mid]
    } else {
        0.5 * (v[mid - 1] + v[mid])
    }
}

/// SSv2's `linear_jerk` is a signed scalar, so the acceleration vector has to be reduced to
/// one axis first. Projecting onto the heading keeps the sign that matters -- positive when
/// speeding up, negative when braking -- which the vector magnitude would throw away.
///
/// `yaw_rad` is the ROS-frame heading, so `ax`/`ay` must be ROS-frame too.
fn longitudinal_acceleration(ax: f64, ay: f64, yaw_rad: f64) -> f64 {
    ax * yaw_rad.cos() + ay * yaw_rad.sin()
}

/// A world-frame (ROS) planar vector in the frame of a body heading `yaw_rad` (ROS):
/// x forward, y left.
fn world_to_body(x: f64, y: f64, yaw_rad: f64) -> (f64, f64) {
    let (sin, cos) = yaw_rad.sin_cos();
    (x * cos + y * sin, -x * sin + y * cos)
}

/// Jerk from two consecutive longitudinal accelerations.
///
/// Returns 0.0 for a non-positive `dt`, which keeps a bad step time from producing an
/// infinite or NaN jerk that SSv2 would then evaluate conditions against.
fn jerk_from_acceleration(previous: f64, current: f64, dt: f64) -> f64 {
    if dt <= 0.0 {
        return 0.0;
    }
    (current - previous) / dt
}

/// An SSv2 pose (map frame) as an agent-protocol pose.
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

/// Spawn height when the ground could not be probed.
///
/// Keeps the commanded height if it is already plausible, rather than forcing everything to
/// one flat value -- the old `z < 0.5 => 0.5` clamp buried vehicles on elevated roads and
/// floated them in dips.
fn fallback_spawn_height(commanded_z: f32) -> f32 {
    commanded_z.max(FALLBACK_SPAWN_Z)
}

use super::collision_monitor::CollisionMonitor;
use crate::actor_census;
use crate::backend::{
    BackendError, Driver, EntityKind, EntityStatus, Episode, Goal, Sensor, SimBackend,
    SpawnRequest, StatusBatch, StatusOutcome,
};
use crate::contact_report::ReportGeometry;
use crate::coordinate_conversion::{self, MapOrigin, OriginOffset};
use crate::entity_manager::{EntityManager, EntityType};
use crate::episode_clock::{
    fmt_ns, old_episode_last_elapsed, reload_as_pause, tick_then_read, EpisodeClock, EpisodeReload,
};
use crate::npc_tracking::{self, Steering};
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::simulation_api_schema::TrafficSignal;
use crate::proto::traffic_simulator_msgs::entity_subtype::Enum as EntitySubtype;
use crate::proto::traffic_simulator_msgs::{self, BoundingBox};
use crate::sensor_release::SensorReleaseNotifier;
use crate::spawn_ladder;
use crate::weather::{Weather, WeatherRequest};
use crate::world_services::WeatherReply;

/// `(elapsed_seconds, delta_seconds)` of `world`'s current snapshot.
/// Why a watched tick failed (roadmap 032 N5).
#[derive(Debug)]
enum TickFailure {
    /// CARLA (LibCarla) reported an error: the connection signal `update_frame` acts on.
    Carla(carla::CarlaError),
    /// The tick never returned; the bridge is wedged. The request's error message.
    Hung(String),
}

impl std::fmt::Display for TickFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TickFailure::Carla(e) => write!(f, "{e}"),
            TickFailure::Hung(reason) => f.write_str(reason),
        }
    }
}

/// How long csb waits at connect for a frame of a world CARLA ticks by itself, to write
/// the epoch marker (`write_epoch_marker_at_connect`). One tick of a loaded server is well
/// under a second.
const CONNECT_FRAME_WAIT: Duration = Duration::from_secs(2);

/// CARLA's frame and synchronous-mode settings for the log after a hung tick, read on a
/// thread of its own so that a CARLA that no longer answers cannot hang the reply too.
fn carla_state_for_log(world: &World) -> String {
    let world = world.clone();
    let (tx, rx) = std::sync::mpsc::channel();
    let spawned = std::thread::Builder::new()
        .name("csb-hung-state".into())
        .spawn(move || {
            let frame = world
                .snapshot()
                .map(|s| s.frame().to_string())
                .unwrap_or_else(|e| format!("unknown ({e})"));
            let settings = match world.settings() {
                Ok(s) => format!(
                    "synchronous_mode {}, fixed_delta_seconds {:?}, substepping {}, \
                     max_substep_delta_time {}, max_substeps {}, no_rendering_mode {}",
                    s.synchronous_mode,
                    s.fixed_delta_seconds,
                    s.substepping,
                    s.max_substep_delta_time,
                    s.max_substeps,
                    s.no_rendering_mode
                ),
                Err(e) => format!("unreadable ({e})"),
            };
            let _ = tx.send(format!(
                "CARLA snapshot frame {frame}, settings: {settings}"
            ));
        });
    if spawned.is_err() {
        return "CARLA's state not read (no thread)".to_string();
    }
    rx.recv_timeout(HUNG_STATE_READ_WAIT).unwrap_or_else(|_| {
        format!(
            "CARLA did not report its frame and settings within {} s",
            HUNG_STATE_READ_WAIT.as_secs()
        )
    })
}

fn read_frame_of(world: &World) -> Result<(f64, f64)> {
    let snapshot = world.snapshot().wrap_err("world snapshot")?;
    let ts = snapshot.timestamp();
    Ok((ts.elapsed_seconds, ts.delta_seconds))
}

/// The backend's CARLA calls for one episode change, in the shape
/// [`reload_as_pause`] orders them.
struct CarlaReload<'a> {
    backend: &'a mut CarlaBackend,
    town: &'a str,
    current_town: &'a str,
    /// Frame number of the pre-reload snapshot, for the frame count after the load.
    f0: Option<u64>,
}

impl EpisodeReload for CarlaReload<'_> {
    type World = carla::client::World;
    type Error = eyre::Report;

    fn enter_sync(&mut self) -> Result<()> {
        self.backend.enable_sync_mode()
    }

    fn read_frame(&mut self) -> Result<(f64, f64)> {
        let snapshot = self.backend.world.snapshot().wrap_err("world snapshot")?;
        let ts = snapshot.timestamp();
        self.f0 = Some(snapshot.frame() as u64);
        Ok((ts.elapsed_seconds, ts.delta_seconds))
    }

    fn reload(&mut self) -> Result<carla::client::World> {
        self.backend.prepare_map(self.town, self.current_town)
    }
}

/// A census of CARLA's actors (034 step 5): what a short-lived client found by id.
struct Census {
    found: Vec<actor_census::Found>,
}

/// The test-only spawn timeout hook's variable (034 step 4).
const SPAWN_TIMEOUT_ENV: &str = "CSB_TEST_SPAWN_TIMEOUT_MS";

fn spawn_timeout_hook_from_env() -> Option<Duration> {
    parse_spawn_timeout_hook(std::env::var(SPAWN_TIMEOUT_ENV).ok().as_deref())
}

fn parse_spawn_timeout_hook(value: Option<&str>) -> Option<Duration> {
    let ms: u64 = value?.trim().parse().ok()?;
    (ms > 0).then(|| Duration::from_millis(ms))
}

/// How far above the id watermark a timed-out spawn's actor is looked for: actor ids are
/// handed out one per actor server-wide, so this covers every actor anybody (acb's sensors,
/// other clients) created since the last id csb saw.
const LATE_ACTOR_ID_MARGIN: u32 = 4096;

/// CARLA's error for a spawn blocked at its point (`ActorSpawnResult.cpp`).
fn is_collision_error(message: &str) -> bool {
    message.contains("collision")
}

/// The CARLA calls of one entity's spawn ([`spawn_ladder::SpawnBackend`]).
struct CarlaSpawn<'a> {
    backend: &'a mut CarlaBackend,
    blueprint: carla::client::ActorBlueprint,
    blueprint_id: String,
    /// The `role_name` set on it; `None` for a blueprint without the attribute.
    role: Option<String>,
    transform: Transform,
}

impl spawn_ladder::SpawnBackend for CarlaSpawn<'_> {
    fn spawn(&mut self, z: f32, timeout: Duration) -> spawn_ladder::Attempt {
        use spawn_ladder::Attempt;
        let mut transform = self.transform.clone();
        transform.location.z = z;
        let timeout = self.backend.spawn_timeout_hook.unwrap_or(timeout);
        self.backend.set_client_timeout(timeout);
        let command = carla::rpc::Command::spawn_actor(self.blueprint.clone(), transform, None);
        let result = match std::sync::Arc::get_mut(&mut self.backend.client) {
            Some(client) => client
                .apply_batch_sync(vec![command], false)
                .map_err(|e| e.to_string()),
            None => Err("the CARLA client is held by a call that never returned".to_string()),
        };
        self.backend.set_client_timeout(budgets::CLIENT_RPC_TIMEOUT);
        match result {
            Err(e) => Attempt::CallFailed(e),
            Ok(responses) => match responses.first() {
                None => Attempt::CallFailed("CARLA returned no result for the spawn".into()),
                Some(r) => match (r.error(), r.actor_id()) {
                    (None, Some(id)) => Attempt::Spawned(id),
                    (None, None) => Attempt::Refused("CARLA returned actor id 0".into()),
                    (Some(e), _) if is_collision_error(e) => Attempt::Collision(e.to_string()),
                    (Some(e), _) => Attempt::Refused(e.to_string()),
                },
            },
        }
    }

    fn find_late(
        &mut self,
        watermark: u32,
        timeout: Duration,
    ) -> std::result::Result<Vec<u32>, String> {
        let Some(role) = self.role.clone() else {
            return Err(format!(
                "'{}' takes no role_name, so its actor cannot be told from anybody else's",
                self.blueprint_id
            ));
        };
        let ids: Vec<u32> = (watermark.saturating_add(1)
            ..=watermark.saturating_add(LATE_ACTOR_ID_MARGIN))
            .collect();
        // Through a client of its own: csb's answers from its actor cache first, which still
        // holds actors destroyed since -- the previous scenario's `hero` was adopted that way
        // in the first live run (034 step 4 results).
        let probe = self
            .backend
            .probe_world(timeout)
            .map_err(|e| format!("{e:#}"))?;
        let found = probe
            .1
            .actors_by_ids(&ids)
            .map_err(|e| format!("look up actors above {watermark}: {e}"))?;
        let mut matches: Vec<u32> = found
            .iter()
            .filter(|a| a.type_id() == self.blueprint_id)
            .filter(|a| role_name_of(a).as_deref() == Some(role.as_str()))
            .map(|a| a.id())
            .collect();
        matches.sort_unstable();
        Ok(matches)
    }
}

/// An actor's `role_name` attribute, if it has one.
fn role_name_of(actor: &carla::client::Actor) -> Option<String> {
    actor
        .attributes()
        .ok()?
        .iter()
        .find(|a| a.id() == "role_name")
        .map(|a| a.value_string())
}

/// What an `UpdateFrame` should do, given how far startup has progressed.
///
/// CARLA synchronous mode cannot be enabled at `Initialize`. `acb_bridge` discovers its
/// vehicle by polling `world.actors()`, which needs CARLA advancing on its own; but in
/// sync mode nothing advances until someone ticks, and this bridge does not tick until
/// SSv2 sends a frame, and SSv2 sends no frame until the ego exists. Enabling sync mode
/// early deadlocks all three. So we stay async until the ego is spawned. Since roadmap
/// 015 the switch happens just *before* the ego spawn (`sync_before_spawn`) and the
/// spawn's own tick puts the ego into the episode; `EnableSyncThenTick` remains the
/// fallback if that switch failed.
///
/// See `docs/design/multi-instance-architecture.md` (gap 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameAction {
    /// No ego yet. Leave CARLA free-running so `acb_bridge` can poll for its vehicle.
    WaitForEgo,
    /// Ego exists and this is the first frame since. Enable sync mode, then tick.
    EnableSyncThenTick,
    /// Steady state.
    Tick,
}

/// A CARLA call that fails after this long failed by timing out, not by being refused: the
/// server is gone or hung, and the next call will wait just as long.
const SLOW_FAILURE: Duration = Duration::from_secs(5);

/// The `role_name` prefix csb puts on every actor it spawns for an SSv2 entity that has no
/// configured role, so a later bridge can tell them from everybody else's actors.
const ENTITY_ROLE_PREFIX: &str = "csb_entity:";

fn entity_role_name(entity: &str) -> String {
    format!("{ENTITY_ROLE_PREFIX}{entity}")
}

fn is_entity_role_name(role: &str) -> bool {
    role.starts_with(ENTITY_ROLE_PREFIX)
}

/// Whether to switch CARLA to synchronous mode before this spawn: for the ego, if not
/// already. See the comment at its call in `spawn_entity`.
fn sync_before_spawn(kind: SpawnKind, sync_mode_enabled: bool) -> bool {
    kind == SpawnKind::Ego && !sync_mode_enabled
}

impl SpawnKind {
    /// A vehicle's kind from who drives it: `simulator_autopilot` hands it to CARLA's
    /// Traffic Manager, `agent` to the agent registered under its name.
    fn of_driver(driver: Driver) -> Self {
        match driver {
            Driver::Ego => SpawnKind::Ego,
            Driver::Scenario => SpawnKind::Npc,
            Driver::SimulatorAutopilot => SpawnKind::AutopilotNpc,
            Driver::Agent => SpawnKind::AgentVehicle,
        }
    }
}

/// Why an `agent` entity cannot have the CARLA role_name it needs (= its entity name).
fn agent_role_conflict(entity: &str, ego_role: &str) -> Option<String> {
    if entity.trim().is_empty() {
        return Some("an agent-driven vehicle needs a name (it is its CARLA role_name)".into());
    }
    if entity == ego_role {
        return Some(format!(
            "'{entity}' is the ego's CARLA role_name; an agent-driven vehicle's role_name is \
             its entity name, and two vehicles answering to one role_name confuse acb_bridge"
        ));
    }
    if is_entity_role_name(entity) {
        return Some(format!("'{entity}' collides with csb's own entity marks"));
    }
    None
}

/// Spacing of the points sampled along each lane for the lane graph [m].
const LANE_SAMPLE_M: f64 = 2.0;

/// Full brake and hand brake: how a vehicle waits for its driver (or its next goal).
const PARKED_CONTROL: carla::rpc::VehicleControl = carla::rpc::VehicleControl {
    throttle: 0.0,
    steer: 0.0,
    brake: 1.0,
    hand_brake: true,
    reverse: false,
    manual_gear_shift: false,
    gear: 0,
};

fn lane_key(waypoint: &carla::client::Waypoint) -> autopilot::LaneKey {
    autopilot::LaneKey {
        road: waypoint.road_id(),
        section: waypoint.section_id(),
        lane: waypoint.lane_id(),
    }
}

/// A point of a Traffic Manager path. carla-rust's `set_custom_path` takes `AsRef<Location>`,
/// which `Location` itself does not implement.
struct PathPoint(Location);

impl AsRef<Location> for PathPoint {
    fn as_ref(&self) -> &Location {
        &self.0
    }
}

fn location_of(p: &autopilot::Point) -> Location {
    Location {
        x: p[0] as f32,
        y: p[1] as f32,
        z: p[2] as f32,
    }
}

fn point_of(l: &Location) -> autopilot::Point {
    [l.x as f64, l.y as f64, l.z as f64]
}

/// A `simulator_autopilot` entity: a CARLA vehicle Traffic Manager drives (see `autopilot`).
struct AutopilotVehicle {
    vehicle: carla::client::Vehicle,
    /// Registered with Traffic Manager. Not before SSv2 starts its NPC logic: until then
    /// SSv2's own NPCs stand still, and so does this one (parked by the spawn).
    engaged: bool,
    /// Stopped at its goal: released by Traffic Manager and braked, until a new goal.
    parked: bool,
    /// The scenario's absolute target speed [m/s]; `None` leaves TM's (the speed limit).
    target_speed: Option<f64>,
    /// The route to the current goal and the path TM follows for it.
    route: Option<(autopilot::RouteProgress, Vec<autopilot::Point>)>,
    /// The route's path has not been given to TM yet.
    path_pending: bool,
    /// The speed the approach to the goal started from.
    approach_cruise: Option<f64>,
}

fn decide_frame_action(sync_mode_enabled: bool, has_ego: bool) -> FrameAction {
    match (sync_mode_enabled, has_ego) {
        (true, _) => FrameAction::Tick,
        (false, true) => FrameAction::EnableSyncThenTick,
        (false, false) => FrameAction::WaitForEgo,
    }
}

/// Green, yellow and red phase length set on every light while frozen: a day, so no light
/// completes a phase during any scenario. See `hold_light_phases`.
const HOLD_PHASE_SECONDS: f32 = 99_999.0;

/// Longest PhysX substep the bridge lets CARLA take, in seconds.
///
/// CARLA's default is 0.01 s. On 0.9.16 that shows up as phantom yaw rate -- up to 0.9 °/s
/// in the IMU and in `get_angular_velocity` on a vehicle standing still; 0.002 s brings it
/// to about 0.003 °/s (tier4, autoware_universe PR #13406, measured on 0.9.16). The ego's
/// IMU feeds Autoware's EKF, so the phantom rate is a heading drift the ego never had.
const MAX_PHYSICS_SUBSTEP_DELTA: f64 = 0.002;

/// CARLA's ceiling on `max_substeps` (the UE physics setting is clamped to 16).
const CARLA_MAX_PHYSICS_SUBSTEPS: u64 = 16;

/// CARLA's own physics-substep defaults, put back when the bridge hands the world back.
const CARLA_DEFAULT_MAX_SUBSTEP_DELTA: f64 = 0.01;
const CARLA_DEFAULT_MAX_SUBSTEPS: u64 = 10;

/// The world-settings timing for synchronous mode: one CARLA tick per bridge substep, and
/// the physics substepping inside that tick.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SyncTiming {
    fixed_delta_seconds: f64,
    max_substep_delta_time: f64,
    max_substeps: u64,
}

/// Timing for an SSv2 frame of `step_time` delivered in `substeps` CARLA ticks.
///
/// `max_substeps = ceil(delta / 0.002)`. When that would exceed CARLA's 16 (a CARLA tick
/// longer than 32 ms, e.g. a 0.05 s step with one substep), the count is clamped and the
/// substep length is stretched to `delta / 16` instead: CARLA requires
/// `max_substep_delta_time * max_substeps >= fixed_delta_seconds`, and the tick length is
/// the one thing that must not change.
fn sync_timing(step_time: f64, substeps: u32) -> SyncTiming {
    let delta = step_time / substeps.max(1) as f64;
    // The small epsilon keeps 0.02 / 0.002 = 10.000000000000002 from rounding up to 11.
    let wanted = ((delta / MAX_PHYSICS_SUBSTEP_DELTA) - 1e-9).ceil().max(1.0) as u64;
    if wanted <= CARLA_MAX_PHYSICS_SUBSTEPS {
        SyncTiming {
            fixed_delta_seconds: delta,
            max_substep_delta_time: MAX_PHYSICS_SUBSTEP_DELTA,
            max_substeps: wanted,
        }
    } else {
        SyncTiming {
            fixed_delta_seconds: delta,
            max_substep_delta_time: delta / CARLA_MAX_PHYSICS_SUBSTEPS as f64,
            max_substeps: CARLA_MAX_PHYSICS_SUBSTEPS,
        }
    }
}

/// Write `timing` into CARLA settings for synchronous mode.
fn apply_sync_timing(settings: &mut carla::rpc::EpisodeSettings, timing: SyncTiming) {
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(timing.fixed_delta_seconds);
    settings.substepping = true;
    settings.max_substep_delta_time = timing.max_substep_delta_time;
    settings.max_substeps = timing.max_substeps;
}

/// Height of a walker's CARLA origin above its feet, from its bounding box (centre height
/// relative to the origin, and half-height). The box bottom is at
/// `origin + center_z - extent_z`; putting that on the ground puts the origin at
/// `ground + extent_z - center_z`. For CARLA walkers the box is centred on the origin, so
/// this is the capsule half-height (~0.9 m).
fn walker_lift(bbox_center_z: f32, bbox_extent_z: f32) -> f32 {
    (bbox_extent_z - bbox_center_z).max(0.0)
}

/// How far a walker may move from where its ground height was last looked up before it is
/// looked up again. A waypoint lookup is one RPC; per walker per frame at 20 Hz that is too
/// many for a height that changes by centimetres over a few metres of road.
const WALKER_GROUND_REFRESH_M: f32 = 2.0;

/// The world as tracked NPCs are commanded in one `UpdateEntityStatus`: one snapshot to
/// read every body from, one actor list to command them through (roadmap 023).
struct TrackedView {
    snapshot: carla::client::WorldSnapshot,
    actors: carla::client::ActorList,
}

/// An NPC under `npc_motion: tracked` (roadmap 023): physics on, driven toward SSv2's pose
/// every frame by `npc_tracking::command`.
#[derive(Debug, Clone, Copy)]
struct TrackedNpc {
    walker: bool,
    /// Steer feed-forward geometry; `None` for walkers and vehicles without declared axles.
    steering: Option<Steering>,
    /// Declared `(maxAcceleration, maxDeceleration)`, m/s2: the force bound
    /// (`npc_tracking::max_change`). `None` when undeclared.
    performance: Option<(f64, f64)>,
    /// SSv2's heading for it last frame, CARLA degrees: the yaw rate comes from the poses
    /// (`npc_tracking::pose_yaw_rate`).
    last_yaw_deg: Option<f64>,
    /// The planar velocity commanded last frame, CARLA m/s: what the blocked rule
    /// (`npc_tracking::blocked`) holds the body's velocity after the tick against.
    last_command: Option<(f64, f64)>,
    /// Blocked last frame (`npc_tracking::blocked`, which holds it until the body moves on).
    blocked: bool,
    /// Held last frame (`npc_tracking::held`): an ego contact, or a body within
    /// `PROXIMITY_HOLD_M`. The gap trace's `contact_hold` flag.
    held: bool,
    /// Not tracked yet: its first command is not force-bounded when SSv2 already moves it
    /// (`npc_tracking::first_frame_change`, roadmap 032 gap D).
    first: bool,
    /// Origin above the bottom of the vehicle's CARLA box, m: a snap puts the box bottom on
    /// the ground (`npc_tracking::snap_attitude`). Unused for walkers (`walker_teleport_z`).
    lift: f32,
}

/// A walker's height bookkeeping: its fixed origin-above-feet lift, and the last ground
/// lookup (`x`, `y`, and the road z there -- `None` when CARLA had no lane nearby).
#[derive(Debug, Clone, Copy, Default)]
struct WalkerHeight {
    lift: f32,
    ground: Option<(f32, f32, Option<f32>)>,
}

/// Whether a walker commanded to (`x`, `y`) needs a fresh ground lookup, given where the
/// last one was made. Always on the first teleport; afterwards only once it has moved
/// more than [`WALKER_GROUND_REFRESH_M`] from that point.
fn walker_ground_is_stale(last: Option<(f32, f32)>, x: f32, y: f32) -> bool {
    match last {
        None => true,
        Some((lx, ly)) => (x - lx).hypot(y - ly) > WALKER_GROUND_REFRESH_M,
    }
}

/// CARLA origin z for a walker. SSv2's own z for a pedestrian is the scenario author's
/// number (0.3 in town01_pedestrian.xosc), not the road: adding the lift to it floated the
/// walker's feet 0.30 m above the road. So the ground under the walker wins, and the
/// commanded z is used only when there is no ground to be had.
fn walker_origin_z(ground_z: Option<f32>, commanded_z: f32, lift: f32) -> f32 {
    ground_z.unwrap_or(commanded_z) + lift
}

/// Why a vehicle spawn must be refused because an ego already exists, if it must.
fn second_ego_rejection(is_ego: bool, existing_ego: Option<&str>, name: &str) -> Option<String> {
    match (is_ego, existing_ego) {
        (true, Some(existing)) => Some(format!(
            "Cannot spawn '{name}' as ego: ego '{existing}' already exists, and this backend \
             supports one ego per scenario"
        )),
        _ => None,
    }
}

/// The overall result of an `UpdateEntityStatus`.
///
/// An entity the bridge does not know fails the request, naming it. It used to be echoed
/// back, which told SSv2 the entity was exactly where it asked -- the same silent success
/// that once let a pedestrian scenario pass with no pedestrian. The stock backend
/// (simple_sensor_simulator) throws on an unknown name and SSv2 throws on `success=false`,
/// so this is the stock semantics.
fn entity_status_result(unknown: &[String], teleport_failures: &[String]) -> Option<String> {
    let mut problems = Vec::new();
    if !unknown.is_empty() {
        problems.push(format!(
            "unknown entit{} {} (not spawned through this bridge, or already despawned)",
            if unknown.len() == 1 { "y" } else { "ies" },
            unknown
                .iter()
                .map(|n| format!("'{n}'"))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    if !teleport_failures.is_empty() {
        problems.push(format!(
            "failed to apply the commanded pose to {} entit{} ({})",
            teleport_failures.len(),
            if teleport_failures.len() == 1 {
                "y"
            } else {
                "ies"
            },
            teleport_failures.join("; ")
        ));
    }
    (!problems.is_empty()).then(|| format!("UpdateEntityStatus: {}", problems.join("; ")))
}

pub struct CarlaBackend {
    /// The server's map list, fetched once (see `prepare_map`).
    available_maps: Option<Vec<String>>,
    /// Kept so the bridge can rebuild `world` after a CARLA outage.
    /// Shared with the watchdog's helper thread for the map load; `set_timeout` needs it
    /// unshared ([`CarlaBackend::set_client_timeout`]).
    client: std::sync::Arc<Client>,
    /// The deadline of the request in progress (`Initialize`, a spawn), which caps every
    /// watched call's bound ([`CarlaBackend::bound`]); `None` between them and for frames.
    request_deadline: Option<Deadline>,
    carla_host: String,
    carla_port: u16,
    world: World,
    entities: EntityManager,
    step_time: f64,
    /// CARLA ticks per SSv2 frame, for the current `step_time`. Recomputed whenever the step
    /// changes; see [`crate::config::ticks_per_frame`].
    substeps: u32,
    /// Whether this bridge has switched CARLA into synchronous mode. Also gates the
    /// cleanup path: we must not restore async mode we never left.
    sync_mode_enabled: bool,
    /// Whether an ego vehicle has been spawned. See [`FrameAction`].
    has_ego: bool,
    /// Whether `update_traffic_lights` has already logged its rejection. SSv2 sends
    /// states every frame, so it must warn once.
    warned_traffic_lights: bool,
    /// Announces despawns to sensor bridges so they can stop listening first.
    ///
    /// `None` when disabled in config, or when the sockets could not be bound -- this is
    /// an optimisation, and a scenario that cannot announce still runs (noisily). See
    /// `sensor_release`.
    sensor_release: Option<SensorReleaseNotifier>,
    /// Seconds to hold a freshly spawned ego while its bridge settles localization onto it.
    /// Zero disables the wait. See `warm_up_localization`.
    localization_warmup: f64,
    /// Every CARLA actor this bridge created and has not confirmed destroyed.
    ///
    /// Deliberately separate from `EntityManager`, which is a name lookup that gets
    /// emptied on despawn and on re-initialise. Tying teardown to it leaked every actor
    /// whose name mapping was dropped first.
    spawned_actors: SpawnLedger,
    /// Whether this bridge froze CARLA's traffic lights, so shutdown only unfreezes what
    /// it actually froze. Set by phase 009 when freezing lands (invariant 3).
    froze_traffic_lights: FreezeGuard,
    /// Each light's own phase durations (actor id, green, yellow, red), saved before
    /// [`HOLD_PHASE_SECONDS`] overwrote them, so unfreezing gives the server its cycle back.
    held_light_timings: Vec<(u32, f32, f32, f32)>,
    /// Consecutive CARLA operation failures, used to decide the connection is gone.
    consecutive_carla_failures: u32,
    /// Last longitudinal acceleration seen per actor, so jerk can be differenced across
    /// frames. CARLA reports acceleration but not its derivative.
    previous_longitudinal_accel: HashMap<u32, f64>,
    /// Frames left to report zero acceleration for a freshly spawned physics actor.
    /// Vehicles spawn slightly above the road and settle onto their suspension; CARLA
    /// reports the impact as a spike (observed: -11 m/s² longitudinal), which SSv2's
    /// entity sanity bounds ([-5, 3] m/s²) treat as a scenario-fatal error. The spike
    /// is a spawn artifact, not vehicle behaviour.
    settle_frames: HashMap<u32, u32>,
    /// Recent raw acceleration samples per actor, for the reporting-side smoothing
    /// (see ACCEL_SMOOTHING_FRAMES).
    accel_history: HashMap<u32, VecDeque<(f64, f64, f64)>>,
    /// Per walker actor: metres from the ground to the CARLA actor origin (SSv2 poses a
    /// pedestrian at its feet; a CARLA walker's origin is its capsule centre, see
    /// [`walker_lift`]), and the cached ground height under it (see [`WalkerHeight`]).
    walker_height: HashMap<u32, WalkerHeight>,
    /// NPCs moved by physics rather than teleported (roadmap 023), by actor.
    tracked_npcs: HashMap<u32, TrackedNpc>,
    /// Tracked NPCs handed to kinematic after a blocked snap (roadmap 032, gap A), by
    /// actor: physics off, teleported to SSv2's pose every frame for the rest of the
    /// episode, flagged `kinematic` in the gap trace.
    kinematic_fallback: HashSet<u32>,
    /// The box of every physics body csb spawned (ego, tracked NPCs, simulator- and
    /// agent-driven vehicles), by actor: what a tracked NPC's proximity hold measures
    /// against (roadmap 032 N2).
    body_boxes: HashMap<u32, BodyBox>,
    /// Every `world.tick()` runs under it (roadmap 032 N5).
    tick_watchdog: Watchdog,
    /// The highest id of an actor this process spawned (034 step 4's watermark keeps rising
    /// after teardown empties the ledger).
    highest_actor_id: Option<u32>,
    /// The test-only fault hook (034 step 2, [`HangHook`]); `None` in every product run.
    hang_hook: Option<HangHook>,
    /// Test-only: every spawn RPC's timeout (`CSB_TEST_SPAWN_TIMEOUT_MS`, 034 step 4), to
    /// force the timed-out spawn that is then adopted. `None` in every product run.
    spawn_timeout_hook: Option<Duration>,
    /// Set when a tick never returned: the reason, for the error every later request gets
    /// and for the log as the bridge exits (see [`CarlaBackend::wedged`]).
    wedged: Option<String>,
    /// Set when the idle check found the CARLA connection dead: the reason the bridge exits
    /// (see [`CarlaBackend::probe_connection_idle`]).
    server_lost: Option<String>,
    /// Consecutive failed idle checks.
    idle_probe_failures: u32,
    /// Directory holding per-map config, e.g. the traffic light signal mapping.
    config_dir: PathBuf,
    /// Map directory name to CARLA town, for maps whose directory is not the town name.
    map_aliases: HashMap<String, String>,
    /// CARLA time plus the episode epoch, reported to SSv2 (roadmap 015).
    episode_clock: EpisodeClock,
    /// Why the world's epoch marker is still to be checked against `episode_clock`
    /// (`epoch_marker`), if it is: set at start-up and on every episode change, cleared once
    /// csb could see the world's actors.
    epoch_marker_due: Option<&'static str>,
    /// Lanelet2 signal ID to OpenDRIVE sign ID for the loaded town.
    signal_map: SignalMap,
    /// Lanelet2 signal IDs already reported as unmapped. SSv2 sends states every frame, so
    /// an unmapped signal must warn once rather than per frame.
    warned_unmapped_signals: HashSet<i32>,
    /// Whether the arrow-shape limitation has been reported for this run.
    warned_arrow_shapes: bool,
    /// Bridge configuration: ego role name, blueprint map, background AVs.
    config: BridgeConfig,
    /// CARLA's view of the ego's collisions, logged beside SSv2's verdict and never fed
    /// into it (roadmap 014, gap 3). See `collision_monitor`.
    collision_monitor: CollisionMonitor,
    /// CARLA's episode id for `world`, when it could be read. A reconnect that finds a
    /// different one is talking to a restarted server (or a world someone else loaded): the
    /// old actor ids mean nothing there and the old episode's clock has ended.
    world_id: Option<u64>,
    /// Where the episode clock is recorded for the next csb process (clock_store).
    clock_path: PathBuf,
    /// Generator mode: write `<map dir>/carla/traffic_lights.yaml` instead of checking it.
    generate_signal_table: bool,
    /// Map-frame position of CARLA's world origin for the scenario's map, read at every
    /// Initialize from `<map dir>/carla/map_origin.yaml` (zero without one).
    map_origin: MapOrigin,
    /// Set after the first failed write, so a broken runtime dir warns once.
    warned_clock_store: bool,
    /// Set when a CARLA call failed by timing out (`SLOW_FAILURE`): the server is gone or
    /// hung. Until a reconnect succeeds, teleports, despawns and teardown skip their RPCs
    /// rather than wait out the same timeout once per actor (roadmap 016, gap 10).
    carla_unreachable: bool,
    /// Time spent inside `world.tick()` since [`CarlaBackend::take_tick_time`] last read
    /// it, for the frame budget's processing time (which excludes it).
    tick_time: Duration,
    /// The frame number the last `world.tick()` returned in this world (`None` until one,
    /// and again after a world change). A world snapshot older than this is the client's
    /// stale cache: [`CarlaBackend::fresh_snapshot`] waits for it to catch up (roadmap 023).
    last_tick_frame: Option<u64>,
    /// Snapshots read stale this episode (caught up or not), to keep their log lines few.
    stale_snapshots: u32,
    /// Tracked NPCs missing from the snapshot: warned once, failed only when all are, for
    /// more than `npc_tracking::MISS_LIMIT_FRAMES` frames (roadmap 023).
    miss_watch: npc_tracking::MissWatch,
    /// CARLA's Traffic Manager, created on the first `simulator_autopilot` spawn. LibCarla
    /// ticks it inside `world.tick()` when it is synchronous (verified on 0.9.16: a
    /// registered vehicle drives with only world ticks), so csb stays the only ticker.
    traffic_manager: Option<carla::traffic_manager::TrafficManager>,
    /// The loaded town's map and lane graph, for routing simulator-driven vehicles. Built
    /// on first use, dropped when the world changes.
    road_graph: Option<(carla::client::Map, autopilot::RoadGraph)>,
    /// The entities Traffic Manager drives, by SSv2 name.
    autopilot: HashMap<String, AutopilotVehicle>,
    /// The state the scenario last commanded per light (OpenDRIVE id), this run. Kept so
    /// the lights can be put back after Traffic Manager's start resets them.
    commanded_lights: HashMap<String, carla::rpc::TrafficLightState>,
    /// The agent relay connection for `agent` entities, started at the first such spawn.
    agent_link: Option<AgentLink>,
    /// The entities agents drive (controller `agent`), by SSv2 name = CARLA role_name.
    agent_vehicles: HashSet<String>,
    /// Where their role names are recorded for a later csb's reaper (`agent_link`).
    agent_ledger: PathBuf,
    /// The weather the user asked for -- the `weather` parameter, then the last
    /// `/carla/set_weather` -- with a label for logs. Re-applied after every map load and
    /// reconnect, because CARLA resets it with the world. `None`: leave CARLA's.
    requested_weather: Option<(String, Weather)>,
    /// Measured blueprint boxes for this CARLA version (roadmap 022); `None` spawns as
    /// before: the kind's default, no size check.
    blueprint_boxes: Option<BoxTable>,
    /// Blueprints already warned about as differing from the table, so a stale table
    /// warns once per blueprint rather than once per spawn.
    warned_stale_boxes: HashSet<String>,
    /// The per-frame record of both worlds for the oracle (roadmap 024); records nothing
    /// unless `gap_trace_dir` is set. See `gap_trace`.
    gap_trace: GapTrace,
}

impl CarlaBackend {
    pub fn new(
        client: Client,
        world: World,
        carla_host: String,
        carla_port: u16,
        config_dir: PathBuf,
        config: BridgeConfig,
    ) -> Self {
        let config_for_release = config.sensor_release.clone();
        let substeps =
            crate::config::ticks_per_frame(0.05, config.substeps, config.carla_tick_seconds);
        let collision_monitor_enabled = config.collision_monitor_enabled();
        let contact_report_dir = config.contact_report_dir();
        let world_id = world.id().ok();
        let clock_path = crate::clock_store::path_for(&carla_host, carla_port);
        let agent_ledger = agent_link::ledger_path(&carla_host, carla_port);
        let episode_clock = Self::resume_clock(&clock_path, world_id);
        // Validated by BridgeConfig::validate, so an unknown name cannot get this far.
        let requested_weather =
            crate::weather::preset(config.weather.trim()).map(|(name, w)| (name.to_string(), w));
        let blueprint_boxes = BoxTable::load_for(&config_dir, super::version::BUILT_FOR);
        let gap_trace = GapTrace::new(config.gap_trace_dir());
        let mut collision_monitor =
            CollisionMonitor::new(collision_monitor_enabled, contact_report_dir);
        if gap_trace.is_on() {
            collision_monitor.keep_for_trace();
        }
        collision_monitor.set_npc_sensors(config.npc_contact_sensors_enabled());
        Self {
            map_aliases: config.map_alias.clone(),
            config,
            client: std::sync::Arc::new(client),
            request_deadline: None,
            carla_host,
            carla_port,
            config_dir,
            signal_map: SignalMap::new(),
            episode_clock,
            epoch_marker_due: Some("start-up"),
            warned_unmapped_signals: HashSet::new(),
            warned_arrow_shapes: false,
            world,
            entities: EntityManager::new(),
            step_time: 0.05,
            substeps,
            sync_mode_enabled: false,
            has_ego: false,
            warned_traffic_lights: false,
            sensor_release: Self::bind_sensor_release(&config_for_release),
            localization_warmup: config_for_release.localization_warmup_s,
            spawned_actors: SpawnLedger::default(),
            froze_traffic_lights: FreezeGuard::default(),
            available_maps: None,
            held_light_timings: Vec::new(),
            consecutive_carla_failures: 0,
            previous_longitudinal_accel: HashMap::new(),
            settle_frames: HashMap::new(),
            accel_history: HashMap::new(),
            walker_height: HashMap::new(),
            tracked_npcs: HashMap::new(),
            kinematic_fallback: HashSet::new(),
            body_boxes: HashMap::new(),
            tick_watchdog: Watchdog::default(),
            hang_hook: {
                let hook = HangHook::from_env();
                if let Some(h) = &hook {
                    tracing::warn!(
                        "TEST FAULT HOOK {} is set: {} (roadmap 034; never in a product run)",
                        HangHook::FRAME_ENV,
                        h.describe()
                    );
                }
                hook
            },
            highest_actor_id: None,
            spawn_timeout_hook: {
                let hook = spawn_timeout_hook_from_env();
                if let Some(t) = hook {
                    tracing::warn!(
                        "TEST FAULT HOOK {SPAWN_TIMEOUT_ENV} is set: every spawn RPC times out \
                         after {} ms (roadmap 034; never in a product run)",
                        t.as_millis()
                    );
                }
                hook
            },
            wedged: None,
            server_lost: None,
            idle_probe_failures: 0,
            collision_monitor,
            world_id,
            clock_path,
            warned_clock_store: false,
            generate_signal_table: false,
            map_origin: MapOrigin::default(),
            carla_unreachable: false,
            tick_time: Duration::ZERO,
            last_tick_frame: None,
            stale_snapshots: 0,
            miss_watch: npc_tracking::MissWatch::default(),
            traffic_manager: None,
            road_graph: None,
            autopilot: HashMap::new(),
            commanded_lights: HashMap::new(),
            agent_link: None,
            agent_vehicles: HashSet::new(),
            agent_ledger,
            requested_weather,
            blueprint_boxes,
            warned_stale_boxes: HashSet::new(),
            gap_trace,
        }
    }

    // --- World configuration (roadmap 018) ---------------------------------------------

    /// Apply the requested weather to the current world, if there is one. `why` names the
    /// occasion for the log (connect, map load, reconnect).
    pub fn reapply_weather(&mut self, why: &str) {
        let Some((label, w)) = self.requested_weather.clone() else {
            return;
        };
        match self.world.set_weather(&w.to_carla()) {
            Ok(()) => tracing::info!("Weather {label} applied ({why})"),
            Err(e) => tracing::warn!("Could not apply weather {label} ({why}): {e}"),
        }
    }

    /// CARLA's current weather.
    pub fn weather(&self) -> Result<Weather> {
        self.world
            .weather()
            .map(|w| Weather::from(&w))
            .map_err(|e| eyre::eyre!("get weather: {e}"))
    }

    /// `/carla/set_weather`: remember the request, apply it, and report what CARLA holds
    /// afterwards. Remembered even when applying fails (CARLA unreachable), so the next
    /// reconnect applies it.
    pub fn set_weather(&mut self, request: WeatherRequest) -> WeatherReply {
        let (label, w) = match request {
            WeatherRequest::Preset(name, w) => (name.to_string(), w),
            WeatherRequest::Parameters(fields) => match self.weather() {
                Ok(current) => ("custom".to_string(), current.with_user_fields(fields)),
                Err(e) => return WeatherReply::failed(format!("{e}; nothing changed"), None),
            },
        };
        self.requested_weather = Some((label.clone(), w));
        if let Err(e) = self.world.set_weather(&w.to_carla()) {
            return WeatherReply::failed(
                format!("set weather {label}: {e}; kept, and applied on the next connect"),
                None,
            );
        }
        tracing::info!("Weather {label} applied (set_weather)");
        match self.weather() {
            Ok(now) => WeatherReply::ok(
                now,
                format!("weather {label} applied; kept across map loads"),
            ),
            Err(e) => WeatherReply::failed(format!("applied {label}, but {e}"), None),
        }
    }

    /// `/carla/get_weather`.
    pub fn get_weather(&self) -> WeatherReply {
        match self.weather() {
            Ok(w) => {
                let kept = match &self.requested_weather {
                    Some((label, _)) => format!("bridge keeps {label} across map loads"),
                    None => "bridge leaves the weather to CARLA".to_string(),
                };
                WeatherReply::ok(w, kept)
            }
            Err(e) => WeatherReply::failed(e.to_string(), None),
        }
    }

    /// Jerk for this actor, from the change in longitudinal acceleration since last frame.
    ///
    /// The first sample for an actor has nothing to difference against and reports 0.0.
    fn differentiate_acceleration(&mut self, actor_id: u32, longitudinal: f64) -> f64 {
        let jerk = match self.previous_longitudinal_accel.get(&actor_id) {
            Some(&previous) => jerk_from_acceleration(previous, longitudinal, self.step_time),
            None => 0.0,
        };
        self.previous_longitudinal_accel
            .insert(actor_id, longitudinal);
        jerk
    }

    // --- Actor ownership -------------------------------------------------------------

    /// Record an actor this bridge created, so teardown can find it later.
    fn record_spawned(&mut self, actor_id: u32) {
        self.spawned_actors.record(actor_id);
        // Kept after the actor goes: ids are never reused, so the watermark only rises.
        self.highest_actor_id = Some(self.highest_actor_id.map_or(actor_id, |h| h.max(actor_id)));
    }

    /// Drop an actor from the teardown set once it is confirmed gone.
    fn forget_spawned(&mut self, actor_id: u32) {
        self.spawned_actors.forget(actor_id);
        self.forget_actor_samples(actor_id);
    }

    /// Drop the per-actor motion history. Actor IDs are reused by CARLA, so a stale jerk
    /// sample would otherwise be differenced against a completely different entity.
    fn forget_actor_samples(&mut self, actor_id: u32) {
        self.previous_longitudinal_accel.remove(&actor_id);
        self.settle_frames.remove(&actor_id);
        self.accel_history.remove(&actor_id);
        self.walker_height.remove(&actor_id);
        self.body_boxes.remove(&actor_id);
        self.tracked_npcs.remove(&actor_id);
        self.kinematic_fallback.remove(&actor_id);
    }

    /// Bind the release channel, or explain why we are not using one.
    fn bind_sensor_release(config: &SensorReleaseConfig) -> Option<SensorReleaseNotifier> {
        if !config.enabled {
            tracing::info!(
                "Sensor release channel disabled by config; sensor bridges will not be \
                 warned before a despawn"
            );
            return None;
        }
        match SensorReleaseNotifier::bind(
            &config.notify_endpoint,
            &config.ack_endpoint,
            Duration::from_millis(config.timeout_ms),
        ) {
            Ok(notifier) => Some(notifier),
            Err(e) => {
                tracing::warn!(
                    "Sensor release channel unavailable ({e:#}); despawns will destroy \
                     sensors without warning their bridges, which costs a burst of CARLA \
                     server log per teardown"
                );
                None
            }
        }
    }

    /// Destroy vehicles left behind by a *previous* bridge process.
    ///
    /// `destroy_all_spawned` only knows what this process created. A bridge that was killed
    /// -- or that segfaulted with the server -- leaves its vehicles in the world, and the
    /// next bridge has no ledger entry for them. It then spawns its own `bg_av_1` next to
    /// the old one, and the config's uniqueness rule (`role_name` identifies a vehicle,
    /// because that is how acb_bridge finds it) is broken by two live actors answering to
    /// one name.
    ///
    /// The spawn retry ladder made this worse rather than louder: the leftover occupies the
    /// spawn point, CARLA refuses the spawn as a collision, and the ladder lifts the new
    /// vehicle half a metre at a time until it fits -- dropping a car on top of a car and
    /// reporting success. Reaping first is what makes the ladder mean "the ground is not
    /// where we thought" again, rather than "something is already parked here".
    ///
    /// Entities -- NPCs, walkers, props -- are found by the mark every bridge puts on them
    /// (`entity_role_name`), whatever their type; a bridge that died mid-scenario leaves
    /// them standing where the next scenario drives (roadmap 016).
    ///
    /// Agent-driven vehicles carry their entity name as role_name (the vehicle side finds
    /// them by it), not the mark, so every csb records those names in a ledger on tmpfs
    /// (`agent_link::ledger_path`) and the reaper reads it.
    ///
    /// Only the ego's role name, the ledger's names and marked entities are touched.
    /// Anything else in the world belongs to somebody else -- a manually spawned vehicle,
    /// another tool's traffic -- and is left alone.
    fn reap_orphaned_actors(&mut self) {
        let mut owned: Vec<String> = vec![self.config.ego.role_name.clone()];
        owned.extend(agent_link::load_ledger(&self.agent_ledger));
        self.store_agent_ledger();

        // By id, through a client of its own (034 step 5, `actor_census`): the snapshot list
        // misses what was created since the last tick and keeps what was destroyed since.
        let census = match self.census() {
            Ok(c) => c,
            Err(e) => {
                tracing::warn!(
                    "Could not take a census of CARLA's actors to look for orphans: {e:#}"
                );
                return;
            }
        };
        let spawned = &self.spawned_actors;
        let orphans = actor_census::orphans(&census.found, ENTITY_ROLE_PREFIX, &owned, |id| {
            spawned.contains(id)
        });
        let mut destroyed = HashSet::new();

        if !orphans.is_empty() {
            tracing::warn!(
                "Found {} actor(s) from a previous bridge process still in the world (census of \
                 {} actors by id); destroying them so this run's spawn points are clear",
                orphans.len(),
                census.found.len()
            );
        }
        for (actor_id, role) in orphans {
            // Their acb_bridge may still be listening to sensors attached to them, exactly
            // as for our own actors -- see the release protocol in issue 015.
            self.announce_release(actor_id, &role);
            match self.destroy_census_actor(&census, actor_id) {
                Ok(children) => {
                    destroyed.insert(actor_id);
                    destroyed.extend(children);
                    tracing::warn!("Destroyed orphaned '{role}' (actor {actor_id})");
                }
                Err(e) => tracing::error!(
                    "Could not destroy orphaned '{role}' (actor {actor_id}): {e:#}. \
                     Its spawn point is still occupied, and this run may stack a vehicle on it."
                ),
            }
        }

        // Sensors whose vehicle is gone -- before this sweep or by it. CARLA keeps ticking a
        // sensor after its parent dies, so an orphaned LiDAR ray-casts every frame forever;
        // twenty-one were once found on one server from bridges killed outright. Only
        // parentless ones: a sensor on a live vehicle belongs to whoever attached it
        // (issue 015's release protocol).
        let remaining: Vec<actor_census::Found> = census
            .found
            .iter()
            .filter(|f| !destroyed.contains(&f.id))
            .cloned()
            .collect();
        let sensors = actor_census::parentless_sensors(&remaining);
        if sensors.is_empty() {
            return;
        }
        tracing::warn!(
            "Found {} sensor(s) whose vehicle is gone; destroying them so they stop being \
             ticked every frame",
            sensors.len()
        );
        for actor_id in sensors {
            if let Err(e) = self.destroy_by_ids(&[actor_id]) {
                tracing::warn!("Could not destroy orphaned sensor {actor_id}: {e:#}");
            }
        }
    }

    /// Destroy an actor found by a census and, first, everything attached to it as the
    /// census saw them: CARLA leaves a destroyed vehicle's sensors alive, and an orphaned IMU
    /// segfaults the server on its next tick. Returns the children destroyed.
    fn destroy_census_actor(&mut self, census: &Census, id: u32) -> Result<Vec<u32>> {
        let children = actor_census::children_of(&census.found, id);
        if !children.is_empty() {
            if let Err(e) = self.destroy_by_ids(&children) {
                tracing::warn!(
                    "Could not destroy every actor attached to {id} ({children:?}): {e:#}. The \
                     server may crash when it next ticks one."
                );
            }
        }
        self.destroy_by_ids(&[id])?;
        Ok(children)
    }

    /// Destroy actors by id on the server, in one `apply_batch_sync`, in order.
    ///
    /// Not `Actor::destroy`: LibCarla refuses to destroy an actor its client's last episode
    /// state does not show alive ("attempting to destroy an actor that is already dead",
    /// `Actor.cpp:149-160`) -- every actor created since the last tick, and every actor a
    /// census client (which never ticks) found. The first live run of 034 step 5 left the
    /// dead bridge's props and walker in the world that way. A batch `DestroyActor` is
    /// decided by the server. An actor already gone counts as destroyed.
    fn destroy_by_ids(&mut self, ids: &[u32]) -> Result<()> {
        let commands = ids
            .iter()
            .map(|&id| carla::rpc::Command::destroy_actor(id))
            .collect();
        let client = std::sync::Arc::get_mut(&mut self.client)
            .ok_or_else(|| eyre::eyre!("the CARLA client is held by a call that never returned"))?;
        let responses = client
            .apply_batch_sync(commands, false)
            .map_err(|e| eyre::eyre!("destroy {ids:?}: {e}"))?;
        let failed: Vec<String> = ids
            .iter()
            .zip(&responses)
            .filter_map(|(id, r)| {
                r.error()
                    .filter(|e| !e.contains("not found"))
                    .map(|e| format!("{id}: {e}"))
            })
            .collect();
        if failed.is_empty() {
            Ok(())
        } else {
            Err(eyre::eyre!("{}", failed.join("; ")))
        }
    }

    /// A short-lived client and its world, for lookups by id that must not be answered from
    /// csb's own actor cache (`actor_census`). Keep the client alive while using the world.
    fn probe_world(&self, timeout: Duration) -> Result<(Client, World)> {
        let mut client = Client::connect(&self.carla_host, self.carla_port, None)
            .map_err(|e| eyre::eyre!("connect a lookup client: {e}"))?;
        client
            .set_timeout(timeout)
            .map_err(|e| eyre::eyre!("lookup client timeout: {e}"))?;
        let world = client
            .world()
            .map_err(|e| eyre::eyre!("lookup client world: {e}"))?;
        Ok((client, world))
    }

    /// Every actor CARLA holds now, by id, through a short-lived client of its own whose
    /// actor cache is empty (`actor_census`). Its RPCs are bounded by what is left of the
    /// request.
    fn census(&self) -> Result<Census> {
        let watermark = self.id_watermark().unwrap_or(0);
        let (client, world) = self.probe_world(self.bound(budgets::CLIENT_RPC_TIMEOUT))?;
        let found = actor_census::scan(watermark, |ids| {
            let list = world.actors_by_ids(ids).map_err(|e| e.to_string())?;
            Ok(list
                .iter()
                .map(|a| actor_census::Found {
                    id: a.id(),
                    type_id: a.type_id(),
                    role: role_name_of(&a),
                    parent: Some(a.parent_id()).filter(|&p| p != 0),
                })
                .collect())
        })
        .map_err(|e| eyre::eyre!("actors by id: {e}"))?;
        drop(client);
        Ok(Census { found })
    }

    /// Record the agent-driven vehicles this process has in the world, for the reaper of
    /// a later csb should this one die without tearing them down.
    fn store_agent_ledger(&self) {
        if let Err(e) = agent_link::store_ledger(&self.agent_ledger, &self.agent_vehicles) {
            tracing::warn!("Could not record agent-driven vehicles: {e:#}");
        }
    }

    /// The vehicle sitting on a spawn point, if any, described for the log; `exclude` is the
    /// actor just spawned. Vehicles come from a census by id (034 step 5), so one created
    /// since the last tick is named too, as a candidate whose pose is not known yet.
    ///
    /// Only used to explain a spawn that had to be lifted. Compares in the horizontal plane:
    /// a vehicle blocking a spawn does so by standing on the same patch of road, whatever
    /// its own z is.
    fn vehicle_occupying(&self, x: f32, y: f32, exclude: u32) -> Option<String> {
        /// Half a car length. Close enough to be the thing that refused the spawn.
        const OCCUPIED_RADIUS_M: f32 = 2.5;

        // Which vehicles exist now, by id (034 step 5); their pose from the last frame.
        let census = match self.census() {
            Ok(c) => c,
            Err(e) => {
                tracing::warn!("No census of CARLA's actors to explain the lift: {e:#}");
                return None;
            }
        };
        let snapshot = self.world.snapshot().ok()?;
        let mut unknown = Vec::new();
        for f in census
            .found
            .iter()
            .filter(|f| f.type_id.starts_with("vehicle."))
        {
            if f.id == exclude {
                continue;
            }
            let role = f.role.clone().unwrap_or_else(|| "?".to_string());
            match snapshot.find(f.id) {
                Some(state) => {
                    let t = state.transform();
                    if (t.location.x - x).hypot(t.location.y - y) <= OCCUPIED_RADIUS_M {
                        return Some(format!("actor {} ('{role}')", f.id));
                    }
                }
                None => unknown.push(format!("{} ('{role}')", f.id)),
            }
        }
        (!unknown.is_empty()).then(|| {
            format!(
                "possibly actor(s) {} (created since the last tick, pose not known yet)",
                unknown.join(", ")
            )
        })
    }

    /// Give whoever attached sensors to `actor_id` a chance to stop them first.
    ///
    /// Must be called immediately before destroying the actor: the point is that the
    /// sensors are still alive when their owner stops listening.
    fn announce_release(&self, actor_id: u32, role_name: &str) {
        if let Some(notifier) = &self.sensor_release {
            notifier.release(actor_id, role_name);
        }
    }

    /// Log the collisions CARLA reported since the last drain, named by SSv2 entity.
    fn drain_collisions(&mut self) {
        let entities = &self.entities;
        self.collision_monitor
            .drain(|id| entities.name_of_actor_this_episode(id).map(str::to_owned));
    }

    /// Stop and destroy the ego's collision sensor and log the run's summary. A no-op when
    /// none is attached. Must run before the ego is destroyed -- see
    /// `CollisionMonitor::finish_run`.
    fn finish_collision_monitor(&mut self) {
        let entities = &self.entities;
        self.collision_monitor
            .finish_run(|id| entities.name_of_actor_this_episode(id).map(str::to_owned));
        self.trace_contacts();
    }

    /// The collision sensor's server is gone (CARLA restarted): drop it, and mark the gap
    /// trace's episode incomplete.
    fn abandon_collision_monitor(&mut self) {
        self.collision_monitor.abandon();
        self.trace_contacts();
        self.gap_trace.mark_incomplete();
    }

    // --- Gap trace (roadmap 024) -----------------------------------------------------

    /// Hand the contacts drained since the last call to the gap trace.
    fn trace_contacts(&mut self) {
        let events = self.collision_monitor.take_traced();
        self.gap_trace.contacts(events);
    }

    /// Write the staged trace frame (if it has been ticked) from `snapshot`, or from one
    /// taken here when the caller has none -- never a per-actor RPC.
    fn trace_emit_staged(&mut self, snapshot: Option<&carla::client::WorldSnapshot>) {
        let now = self.collision_monitor.frame();
        if !self.gap_trace.staged_due(now) {
            return;
        }
        let own;
        let snapshot = match snapshot {
            Some(s) => s,
            None => match self.fresh_snapshot() {
                Ok(s) => {
                    own = s;
                    &own
                }
                Err(e) => {
                    tracing::debug!("Gap trace: no world snapshot ({e}); CARLA's half is null");
                    self.gap_trace.emit_staged(now, |_| None);
                    return;
                }
            },
        };
        let map_origin = self.map_origin;
        let carla_frame = Some(snapshot.frame() as u64);
        self.gap_trace.emit_staged_at(now, carla_frame, |id| {
            snapshot
                .find(id)
                .map(|actor| carla_trace_state(&actor, map_origin))
        });
    }

    /// One `UpdateEntityStatus` for the gap trace: write the previous frame from this
    /// request's snapshot (the world after that frame's tick), then stage this frame's SSv2
    /// poses -- what SSv2 sent for an NPC, what the bridge returned for the ego.
    fn trace_entity_status(
        &mut self,
        updated: &[EntityStatus],
        snapshot: Option<&carla::client::WorldSnapshot>,
    ) {
        self.trace_emit_staged(snapshot);
        let staged = updated
            .iter()
            .filter_map(|u| {
                let entity = self.entities.get(&u.name)?;
                let pose = u.pose.as_ref()?;
                let actor_id = entity.carla_actor_id;
                Some(gap_trace::StagedEntity {
                    name: u.name.clone(),
                    actor_id,
                    ssv2: ssv2_trace_pose(pose),
                    // Held this frame: an ego contact, or another body's box within
                    // PROXIMITY_HOLD_M when it would have snapped (roadmap 032 N2).
                    contact_hold: self.tracked_npcs.get(&actor_id).is_some_and(|n| {
                        n.held
                            || self
                                .collision_monitor
                                .touched_within(actor_id, npc_tracking::CONTACT_HOLD_FRAMES)
                    }),
                    kinematic: self.kinematic_fallback.contains(&actor_id),
                })
            })
            .collect();
        self.gap_trace
            .stage_frame(self.collision_monitor.frame() + 1, staged);
    }

    /// Destroy every actor this bridge created that is still alive.
    ///
    /// Best effort throughout: CARLA may already be gone, and an actor that has vanished on
    /// its own is a success -- the requested end state holds either way. Never panics, so it
    /// is safe on the shutdown path.
    pub fn destroy_all_spawned(&mut self) -> TeardownReport {
        // Traffic Manager lets go of its vehicles before they are destroyed; agents stop.
        self.release_autopilot_vehicles();
        if let Some(link) = self.agent_link.as_ref() {
            link.despawn_all();
        }
        let had_agent_vehicles = !self.agent_vehicles.is_empty();
        self.agent_vehicles.clear();
        if self.carla_unreachable {
            // One timeout per actor and sensor otherwise (gap 10). The ledger is kept for
            // the next teardown after a reconnect.
            self.abandon_collision_monitor();
            tracing::warn!(
                "Teardown skipped: CARLA unreachable; {} actor(s) stay in the ledger for the \
                 next teardown, or the reaper",
                self.spawned_actors.len()
            );
            return TeardownReport::default();
        }

        // Our own collision sensors go before the vehicles they are attached to -- the NPCs'
        // (032 step 5), then the ego's -- and the run's summary is logged while the entity
        // names can still be resolved.
        self.collision_monitor.detach_all_npcs();
        self.finish_collision_monitor();

        // The ledger drives the loop; the world only supplies the destroyer. Split this way
        // so the bookkeeping half is reachable from a unit test, which a CarlaBackend (and
        // therefore a live CARLA world) is not.
        // Disjoint field borrows: the ledger is taken mutably, the world and the release
        // channel immutably.
        let world = &self.world;
        let notifier = self.sensor_release.as_ref();
        // The first destroy that fails by timing out ends the RPCs for the rest (gap 10).
        let mut timed_out = false;
        let (report, destroyed) = self.spawned_actors.teardown(|actor_id| {
            if timed_out {
                return Err(eyre::eyre!("skipped: CARLA unreachable"));
            }
            // The ledger knows the actor id but not the role name; the notice carries "-"
            // and the bridges match on the id, which is the key both sides always have.
            if let Some(notifier) = notifier {
                notifier.release(actor_id, "-");
            }
            let started = Instant::now();
            destroy_actor_in(world, actor_id)
                .inspect_err(|_| timed_out = started.elapsed() > SLOW_FAILURE)
        });
        if timed_out && !self.carla_unreachable {
            self.carla_unreachable = true;
            tracing::warn!("Teardown: a destroy timed out; CARLA treated as unreachable");
        }
        for actor_id in destroyed {
            self.forget_actor_samples(actor_id);
        }
        // Their names stay in the ledger while a destroy may have failed: the reaper of the
        // next Initialize (or csb) finishes the job.
        if had_agent_vehicles && report.failed == 0 {
            self.store_agent_ledger();
        }

        if report.attempted == 0 {
            return report;
        }

        tracing::info!(
            "Teardown: {} spawned, {} destroyed, {} failed",
            report.attempted,
            report.destroyed,
            report.failed
        );
        report
    }

    /// Undo the CARLA-wide changes this bridge made: destroy its actors, unfreeze traffic
    /// lights it froze, and hand synchronous mode back.
    ///
    /// Safe to call more than once, and safe when CARLA has already gone away.
    pub fn shutdown(&mut self) {
        if self.gap_trace.recording() {
            self.trace_emit_staged(None);
        }
        self.destroy_all_spawned();
        self.gap_trace.shutdown(Duration::from_secs(5));
        if let Some(mut link) = self.agent_link.take() {
            link.shutdown();
        }
        self.restore_traffic_lights();
        self.restore_async_mode();
        self.shut_down_traffic_manager("bridge shutdown");
    }

    // --- Map and traffic lights ------------------------------------------------------

    /// Load the town the scenario was authored against, and its signal mapping.
    ///
    /// Reloading the world destroys every actor in it, so this must happen before anything
    /// is spawned -- and the teardown bookkeeping from phase 007 is cleared to match, since
    /// those actor IDs no longer refer to anything.
    fn load_scenario_map(&mut self, lanelet2_map_path: &str) -> Result<()> {
        if lanelet2_map_path.is_empty() {
            eyre::bail!(
                "Initialize supplied no lanelet2_map_path, so the town cannot be verified. \
                 Running against whatever town CARLA currently holds would interpret every \
                 scenario pose against the wrong map."
            );
        }

        let town = town_from_map_path(lanelet2_map_path, &self.map_aliases).ok_or_else(|| {
            eyre::eyre!(
                "cannot resolve a CARLA town from lanelet2_map_path '{lanelet2_map_path}'; \
                 add an entry to the map alias table if this map's directory is not named \
                 after its town"
            )
        })?;

        // Every scenario pose is a map-frame pose; read where CARLA's origin sits in that
        // frame before any of them is converted. A broken file fails Initialize.
        let map_dir = super::resolved_signals::map_dir_of(std::path::Path::new(lanelet2_map_path));
        self.map_origin = load_map_origin(&map_dir)?;
        if self.map_origin != MapOrigin::default() {
            tracing::info!(
                "Map origin: CARLA's world origin is at map ({:.3}, {:.3}, {:.3}) ({})",
                self.map_origin.x,
                self.map_origin.y,
                self.map_origin.z,
                map_dir
                    .join(crate::map_resolver::MAP_ORIGIN_SUBPATH)
                    .display()
            );
        }

        // CARLA reports map names like "Carla/Maps/Town01"; compare on the final component.
        let current = self.world.map().map(|m| m.name()).unwrap_or_default();
        let current_town = current.rsplit('/').next().unwrap_or(&current).to_string();

        if current_town == town {
            tracing::info!("Map '{town}' is already loaded; not reloading");
        } else {
            // Both the map listing and the load take longer than the normal 30 s RPC
            // timeout when the server is rendering, catching up after a restart, or
            // enumerating maps mid-tick (observed: 66 s first load, >30 s listing on a
            // just-restarted server). Widen the timeout for the whole prepare path and
            // restore it afterwards even on failure.
            let load_timeout = self.bound(budgets::LOAD_WORLD_RPC_TIMEOUT);
            self.set_client_timeout(load_timeout);
            let prepared = self.reload_as_pause(&town, &current_town);
            self.set_client_timeout(budgets::CLIENT_RPC_TIMEOUT);
            // CARLA stays synchronous across the load (reset_settings = false), and csb stays
            // its only ticker. Re-apply the settings to the new world anyway: the step and
            // physics substeps are this bridge's, and a failed load left the old world.
            self.sync_mode_enabled = false;
            let world = prepared?;
            self.world_id = world.id().ok();
            self.world = world;
            self.last_tick_frame = None;
            self.forget_road_graph();
            // The new episode's epoch, recorded before anything can kill this process: a
            // restart would otherwise resume the old episode's (gap 11).
            self.record_clock();
            // And in the new world, for a vehicle side that starts after this reload.
            self.epoch_marker_due = Some("map load");
            self.write_epoch_marker();
            if let Err(e) = self.enable_sync_mode() {
                tracing::warn!("Could not re-apply synchronous mode to the new world: {e}");
            }

            // The old world and everything in it is gone. Anything still recorded for
            // teardown refers to actors that no longer exist.
            self.spawned_actors.clear();
            self.previous_longitudinal_accel.clear();
            self.settle_frames.clear();
            self.accel_history.clear();
            self.walker_height.clear();
            self.body_boxes.clear();
            self.tracked_npcs.clear();
            self.kinematic_fallback.clear();
            self.kinematic_fallback.clear();
            self.miss_watch.reset();
            self.stale_snapshots = 0;
            // `sync_mode_enabled` is left as `enable_sync_mode` above set it. It used to be
            // cleared here (a line from before the load re-applied synchronous mode), which
            // left CARLA synchronous with csb believing it async: frames waited for an ego
            // without ticking, so after a map change a scenario with no ego never advanced
            // and its NPCs never reached a snapshot (roadmap 023, "a stale snapshot fails
            // the frame"), and spawns skipped their settling tick.
            // The reloaded world's lights are CARLA's own again, so we owe no unfreeze.
            self.froze_traffic_lights.mark_restored();
            self.held_light_timings.clear();
            tracing::info!("Map '{town}' loaded");
            // The new world came with its town's default weather.
            self.reapply_weather("map load");
        }

        self.load_signal_map(&town, lanelet2_map_path)
    }

    /// Reload the world as a pause in simulation time (roadmap 015).
    ///
    /// Sync mode goes on *before* the last snapshot is read, and nothing ticks between that
    /// read and the reload. acb applies the same epoch rule from the last `on_tick` it
    /// received; with CARLA synchronous, the frame read here is that frame, so both bridges
    /// land on the same epoch with no channel between them. Read in async mode instead and
    /// CARLA could tick once more before the load, putting acb one step ahead of csb.
    fn reload_as_pause(&mut self, town: &str, current_town: &str) -> Result<carla::client::World> {
        let mut backend = CarlaReload {
            backend: self,
            town,
            current_town,
            f0: None,
        };
        let out = reload_as_pause(&mut backend);
        let f0 = backend.f0;
        if let Some(e) = &out.sync_error {
            tracing::warn!(
                "Could not switch CARLA to synchronous mode before reloading ({e}); acb may \
                 see a later last frame and its epoch may differ from csb's"
            );
        }
        if let Some(e) = &out.read_error {
            tracing::warn!("Could not read CARLA's last frame before reloading: {e}");
        }
        let world = out.world?;

        // The snapshot csb read is not the old episode's last frame: CARLA's LoadEpisode
        // ticked it again while waiting. Count the extra frames from the new world's first
        // snapshot (see `old_episode_last_elapsed`), so the epoch lands where acb's does.
        let counted = match (out.last_frame, f0, world.snapshot()) {
            (Some((e0, d0)), Some(f0), Ok(after)) => {
                let e1 = after.timestamp().elapsed_seconds;
                let f1 = after.frame() as u64;
                match old_episode_last_elapsed(e0, d0, f0, f1, e1) {
                    Some(e_last) => {
                        tracing::debug!(
                            "Old episode's last frame counted: F0 {f0} E0 {e0:.9}, F1 {f1} E1 \
                             {e1:.9} -> E_last {e_last:.9}"
                        );
                        Some((e_last, d0))
                    }
                    None => {
                        tracing::warn!(
                            "Could not count the old episode's frames (F0 {f0} E0 {e0:.9}, F1 {f1} \
                             E1 {e1:.9}); the epoch uses the pre-reload snapshot and may trail \
                             acb's by the frames CARLA ticked during the load"
                        );
                        Some((e0, d0))
                    }
                }
            }
            (last, _, _) => last,
        };
        let last = counted.or(self.episode_clock.last_frame());
        let (e_last, d_last) = last.unwrap_or_else(|| {
            tracing::warn!(
                "No last frame known for the old episode; the new one starts at the current \
                 epoch and simulation time may go back"
            );
            (0.0, 0.0)
        });
        let change = self.episode_clock.begin_episode(e_last, d_last);
        tracing::info!(
            "Episode change: epoch {} -> {}, sim_time continues at {} (town {current_town} -> {town})",
            fmt_ns(change.old_epoch_ns),
            fmt_ns(change.new_epoch_ns),
            fmt_ns(change.continues_at_ns)
        );
        Ok(world)
    }

    /// `(elapsed_seconds, delta_seconds)` of CARLA's current snapshot.
    fn read_frame(&self) -> Result<(f64, f64)> {
        read_frame_of(&self.world)
    }

    /// The simulation time to report to SSv2, in integer nanoseconds: CARLA's current frame
    /// plus the epoch, or 0 (the protocol's "unknown") if the snapshot cannot be read.
    fn current_simulation_time_ns(&mut self) -> i64 {
        match self.read_frame() {
            Ok((elapsed, delta)) => self.episode_clock.observe(elapsed, delta),
            Err(e) => {
                tracing::warn!("Could not read CARLA's time; reporting 0: {e}");
                0
            }
        }
    }

    /// Validate the town against the server's map list and load it.
    ///
    /// Split out so the caller can hold a widened RPC timeout across both calls.
    fn prepare_map(&mut self, town: &str, current_town: &str) -> Result<carla::client::World> {
        // load_world on a town the server does not have segfaults inside carla-rust
        // (the C++ exception never crosses the FFI as an Err), so the name must be
        // validated against the server's own list first.
        // Listed once per bridge and cached: the server's maps cannot change while it runs,
        // and the listing took 20-35 s on every reload (roadmap 015 step 7).
        if self.available_maps.is_none() {
            self.available_maps = Some(
                self.client
                    .avaiable_maps()
                    .map_err(|e| eyre::eyre!("list available maps: {e}"))?,
            );
        }
        let available = self.available_maps.clone().unwrap_or_default();
        let known = available.iter().any(|m| m.rsplit('/').next() == Some(town));
        if !known {
            eyre::bail!(
                "Map '{town}' is not available on this CARLA server. Available: {}",
                available
                    .iter()
                    .map(|m| m.rsplit('/').next().unwrap_or(m))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }

        tracing::info!("Loading map '{town}' (CARLA currently holds '{current_town}')");
        // reset_settings = false: the new world stays synchronous, so it advances only when
        // csb ticks it -- csb is the only ticker (docs/design/time-and-ticking.md) -- and
        // the frame count after the load is exact. The default reverts it to async, where it
        // free-runs until someone reads it (and loads took 44 s to over 120 s).
        let client = self.client.clone();
        let name = town.to_string();
        let bound = self.bound(budgets::LOAD_WORLD_WATCHDOG);
        match self.watched(
            "load_world",
            &format!("the load of '{town}'"),
            bound,
            move || client.load_world_opt(&name, false),
        ) {
            Ok(loaded) => loaded.map_err(|e| eyre::eyre!("load world '{town}': {e}")),
            Err(reason) => Err(eyre::eyre!(reason)),
        }
    }

    /// The bound a watched call gets: `step`, capped by the deadline of the request in
    /// progress (034 step 3), never under [`budgets::LIBCARLA_TICK_TIMEOUT`].
    fn bound(&self, step: Duration) -> Duration {
        match &self.request_deadline {
            Some(d) => d.cap(step, budgets::LIBCARLA_TICK_TIMEOUT),
            None => step,
        }
    }

    /// Set the CARLA client's RPC timeout. Impossible only while the helper thread still
    /// holds the client, i.e. after a map load that never returned, when nothing is sent to
    /// CARLA again anyway.
    fn set_client_timeout(&mut self, timeout: Duration) {
        match std::sync::Arc::get_mut(&mut self.client) {
            Some(client) => {
                if let Err(e) = client.set_timeout(timeout) {
                    tracing::warn!("Could not set the CARLA client timeout: {e}");
                }
            }
            None => tracing::warn!(
                "CARLA client timeout left as is: a call that never returned still holds it"
            ),
        }
    }

    /// `apply_settings` under the watchdog (034 step 3): it runs up to 30 ticks, each one
    /// through the Traffic Manager handshake.
    fn apply_settings_watched(
        &mut self,
        step: &str,
        settings: carla::rpc::EpisodeSettings,
    ) -> Result<u64> {
        let mut world = self.world.clone();
        let bound = self.bound(budgets::SETTINGS_WATCHDOG);
        match self.watched("apply_settings", step, bound, move || {
            world.apply_settings(&settings, budgets::LIBCARLA_TICK_TIMEOUT)
        }) {
            Ok(applied) => applied.map_err(|e| eyre::eyre!("apply settings: {e}")),
            Err(reason) => Err(eyre::eyre!(reason)),
        }
    }

    /// `TrafficManager::SetSynchronousMode` under the watchdog, if there is a TM: it signals
    /// the worker through the handshake that can lose a wakeup (034).
    fn set_traffic_manager_synchronous(&mut self, on: bool) -> Result<()> {
        let Some(mut tm) = self.traffic_manager.take() else {
            return Ok(());
        };
        match self.watched(
            "TrafficManager::SetSynchronousMode",
            if on {
                "synchronous mode"
            } else {
                "asynchronous mode"
            },
            budgets::TM_CALL_WATCHDOG,
            move || {
                let r = tm.set_synchronous_mode(on);
                (tm, r)
            },
        ) {
            Ok((tm, r)) => {
                self.traffic_manager = Some(tm);
                r.map_err(|e| eyre::eyre!("Traffic Manager synchronous mode: {e}"))
            }
            Err(reason) => Err(eyre::eyre!(reason)),
        }
    }

    /// Load the explicit signal mapping, fill the rest by position matching, and write the
    /// resolved table acb publishes signals from (roadmap 015).
    fn load_signal_map(&mut self, town: &str, lanelet2_map_path: &str) -> Result<()> {
        // Explicit per-map mapping first: it exists to correct what position matching gets
        // wrong, so it must take precedence over anything derived below.
        let mapping_path = self.config_dir.join(format!("traffic_lights_{town}.yaml"));
        self.signal_map = SignalMap::load_or_empty(&mapping_path)?;
        self.warned_unmapped_signals.clear();
        self.warned_arrow_shapes = false;

        let (lanelet_lights, carla_signals) = self.match_signals_by_position(lanelet2_map_path);

        if self.signal_map.is_empty() {
            tracing::warn!(
                "No traffic light mapping for '{town}', automatic or explicit. Signals the \
                 scenario commands cannot be applied to CARLA. Add {} if this map has lights.",
                mapping_path.display()
            );
        }

        self.check_resolved_signals(town, lanelet2_map_path, &lanelet_lights, &carla_signals)
    }

    /// Compare the resolved table with `<map dir>/carla/traffic_lights.yaml`, the map
    /// artifact acb publishes signals from (roadmap 017: the map dir is read-only at run
    /// time). In generator mode (`generate_signal_table`) write it instead.
    ///
    /// A missing table is a warning -- csb still applies the scenario's signals to CARLA,
    /// only acb has nothing to publish them from -- but a different one fails Initialize:
    /// acb would report the state of the wrong lights to Autoware.
    fn check_resolved_signals(
        &self,
        town: &str,
        lanelet2_map_path: &str,
        lanelet_lights: &[crate::lanelet_map::TrafficLightElement],
        carla_signals: &[super::traffic_light_mapper::CarlaSignal],
    ) -> Result<()> {
        use super::resolved_signals as rs;
        let map_file = lanelet_map_file(lanelet2_map_path);
        let table = rs::ResolvedTable::build(
            town,
            &map_file,
            &self.signal_map,
            lanelet_lights,
            carla_signals,
        );
        let summary = format!(
            "{} mapped way(s) in {} regulatory element(s); unmapped: {} lanelet, {} CARLA",
            table.signals.len(),
            table.regulatory_element_count(),
            table.unmapped.lanelet_way_ids.len(),
            table.unmapped.carla_opendrive_ids.len()
        );
        let map_dir = rs::map_dir_of(std::path::Path::new(lanelet2_map_path));
        let path = rs::table_path(&map_dir);

        if self.generate_signal_table {
            let written = rs::write_table(&map_dir, &table.to_yaml()?)?;
            tracing::info!(
                "Traffic light table ({summary}) written to {}",
                written.display()
            );
            return Ok(());
        }

        match rs::check_table(&path, &table)? {
            rs::Check::Matches => {
                tracing::info!(
                    "Traffic light table {} matches CARLA ({summary})",
                    path.display()
                );
                Ok(())
            }
            rs::Check::Missing => {
                tracing::warn!(
                    "No traffic light table at {} ({summary} resolved): acb_bridge will \
                     publish no signals. Generate it once per map: \
                     ros2 run carla_scenario_bridge carla_scenario_bridge \
                     --generate-signal-table {}",
                    path.display(),
                    map_dir.display()
                );
                Ok(())
            }
            rs::Check::Differs(why) => eyre::bail!(
                "The traffic light table {} does not match CARLA's lights ({why}; resolved: \
                 {summary}). acb_bridge would publish the wrong lights. Regenerate it: \
                 ros2 run carla_scenario_bridge carla_scenario_bridge \
                 --generate-signal-table {}",
                path.display(),
                map_dir.display()
            ),
        }
    }

    /// Fill in the signal mapping by pairing Lanelet2 traffic lights with CARLA's by position.
    ///
    /// Best-effort and never fatal: a map that cannot be read, or lights that do not pair up,
    /// leave the explicit YAML mapping as the only source. Every shortfall is reported with
    /// counts so the gap is visible before the scenario runs rather than discovered when a
    /// light fails to change.
    ///
    /// Returns what it read -- the Lanelet2 lights and CARLA's -- for the resolved table;
    /// either is empty when it could not be read.
    fn match_signals_by_position(
        &mut self,
        lanelet2_map_path: &str,
    ) -> (
        Vec<crate::lanelet_map::TrafficLightElement>,
        Vec<super::traffic_light_mapper::CarlaSignal>,
    ) {
        let map_file = lanelet_map_file(lanelet2_map_path);

        let lanelet_lights = match crate::lanelet_map::load_traffic_lights(&map_file) {
            Ok(lights) => lights,
            Err(e) => {
                tracing::warn!(
                    "Could not read traffic lights from {} ({e}); relying on the explicit \
                     mapping alone",
                    map_file.display()
                );
                return (
                    Vec::new(),
                    self.enumerate_carla_signals().unwrap_or_default(),
                );
            }
        };

        if lanelet_lights.is_empty() {
            tracing::info!("Lanelet2 map declares no traffic lights");
            return (lanelet_lights, Vec::new());
        }

        let carla_signals = match self.enumerate_carla_signals() {
            Ok(signals) => signals,
            Err(e) => {
                tracing::warn!("Could not enumerate CARLA traffic lights ({e})");
                return (lanelet_lights, Vec::new());
            }
        };

        let report = super::traffic_light_mapper::match_signals(
            &lanelet_lights,
            &carla_signals,
            self.map_origin,
        );
        let added = self.signal_map.merge_matched(report.matched);

        tracing::info!(
            "Traffic light matching: {} lanelet element(s), {} CARLA light(s), {added} newly \
             mapped by position (tolerance {}m)",
            lanelet_lights.len(),
            carla_signals.len(),
            super::traffic_light_mapper::SIGNAL_MATCH_TOLERANCE_M
        );

        if !report.unmatched_lanelet.is_empty() {
            tracing::warn!(
                "{} lanelet traffic light(s) had no CARLA light within tolerance: {:?}. These \
                 signals cannot be driven; add them to config/traffic_lights_<town>.yaml.",
                report.unmatched_lanelet.len(),
                report.unmatched_lanelet
            );
        }
        if !report.unmatched_carla.is_empty() {
            tracing::info!(
                "{} CARLA traffic light(s) are not referenced by the Lanelet2 map; the \
                 scenario cannot address them.",
                report.unmatched_carla.len()
            );
        }
        (lanelet_lights, carla_signals)
    }

    /// CARLA's traffic lights, reduced to an OpenDRIVE ID and a position.
    fn enumerate_carla_signals(&self) -> Result<Vec<super::traffic_light_mapper::CarlaSignal>> {
        use super::traffic_light_mapper::CarlaSignal;

        let actors = self.world.actors().wrap_err("list actors")?;
        let lights = actors
            .filter("traffic.traffic_light*")
            .wrap_err("filter traffic lights")?;

        let mut signals = Vec::new();
        for actor in lights.iter() {
            let location = match actor.transform() {
                Ok(t) => t.location,
                Err(e) => {
                    tracing::warn!("Could not read a traffic light's transform: {e}");
                    continue;
                }
            };

            let light = match actor.into_kinds() {
                carla::client::ActorKind::TrafficLight(light) => light,
                _ => continue,
            };

            match light.opendrive_id() {
                Ok(id) => signals.push(CarlaSignal {
                    opendrive_id: id.to_string(),
                    x: location.x as f64,
                    y: location.y as f64,
                    z: location.z as f64,
                }),
                Err(e) => tracing::warn!("Could not read a traffic light's OpenDRIVE id: {e}"),
            }
        }

        Ok(signals)
    }

    /// Freeze CARLA's signal cycling so SSv2 is the only writer (invariant 3).
    ///
    /// Freezing alone leaves each light in whatever state it happened to be in. Since acb
    /// publishes what CARLA's lights show (roadmap 015), every mapped light is first set
    /// GREEN, so a light the scenario never addresses is GREEN for the whole run rather than
    /// an arbitrary phase; unmapped lights keep their frozen state (Autoware cannot see
    /// them). A cycling light would make the run non-deterministic, which is what invariant
    /// 3 exists to prevent. A scenario that cares about a signal must command it.
    fn freeze_traffic_lights(&mut self) {
        match self.world.freeze_all_traffic_lights(true) {
            Ok(()) => {
                self.froze_traffic_lights.mark_frozen();
                tracing::info!(
                    "CARLA traffic light cycling frozen; SSv2 is now the only writer. \
                     Mapped signals the scenario does not command stay GREEN."
                );
                self.set_mapped_lights_green();
                self.hold_light_phases();
            }
            Err(e) => {
                // Not fatal: the scenario may not use traffic lights at all. But if it
                // does, CARLA will be cycling underneath it, so say so plainly.
                tracing::warn!(
                    "Could not freeze CARLA traffic lights ({e}). CARLA's own cycling is \
                     still running and will fight any scenario-commanded state."
                );
            }
        }
    }

    /// Set every mapped light GREEN, before the scenario commands any (roadmap 015).
    ///
    /// With acb publishing what CARLA's lights show, a light frozen at whatever phase the
    /// last run (or CARLA's own cycle) left it in would stop the ego at random
    /// intersections. GREEN is deterministic and permissive: a scenario commands the lights
    /// it cares about, and SSv2's first `UpdateTrafficLights` overrides this for those.
    /// Unmapped CARLA lights are left alone -- Autoware cannot see them. See
    /// `docs/design/scenario-authoring.md`.
    fn set_mapped_lights_green(&mut self) {
        let ids: Vec<String> = self
            .signal_map
            .entries()
            .into_iter()
            .map(|(_, od)| od.to_string())
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect();
        let mut failed = Vec::new();
        for od in &ids {
            if let Err(e) = self.set_signal_state(od, carla::rpc::TrafficLightState::Green) {
                failed.push(format!("{od}: {e:#}"));
            }
        }
        if failed.is_empty() {
            tracing::info!(
                "Set {} mapped traffic light(s) GREEN; the scenario's commands override them",
                ids.len()
            );
        } else {
            tracing::warn!(
                "Set {} of {} mapped traffic light(s) GREEN; failed: {}",
                ids.len() - failed.len(),
                ids.len(),
                failed.join("; ")
            );
        }
    }

    /// Back up the freeze: stretch every light's phases to [`HOLD_PHASE_SECONDS`].
    ///
    /// tier4/carla-autoware-native found `freeze_all_traffic_lights` skips dynamically
    /// placed light groups; those kept cycling and overrode `set_state` within ~22 s. A
    /// light whose phase lasts a day does not advance during a scenario, frozen or not.
    /// Applied to every light in the world, not only the mapped ones: an unmapped light
    /// that cycles breaks invariant 3 just the same.
    fn hold_light_phases(&mut self) {
        let lights = match self
            .world
            .actors()
            .and_then(|a| a.filter("traffic.traffic_light*"))
        {
            Ok(l) => l,
            Err(e) => {
                tracing::warn!("Could not list traffic lights to hold their phases: {e}");
                return;
            }
        };

        let mut held = 0usize;
        let mut failed = 0usize;
        for actor in lights.iter() {
            let id = actor.id();
            let carla::client::ActorKind::TrafficLight(light) = actor.into_kinds() else {
                continue;
            };
            // Save first; a light whose timings cannot be read is still held, and simply
            // not restored.
            let saved = (|| {
                Ok::<_, carla::CarlaError>((
                    light.green_time()?,
                    light.yellow_time()?,
                    light.red_time()?,
                ))
            })();
            let outcome = light
                .set_green_time(HOLD_PHASE_SECONDS)
                .and_then(|()| light.set_yellow_time(HOLD_PHASE_SECONDS))
                .and_then(|()| light.set_red_time(HOLD_PHASE_SECONDS));
            match outcome {
                Ok(()) => {
                    held += 1;
                    if let Ok((g, y, r)) = saved {
                        self.held_light_timings.push((id, g, y, r));
                    }
                }
                Err(e) => {
                    failed += 1;
                    tracing::debug!("Could not hold phases of traffic light {id}: {e}");
                }
            }
        }
        if failed > 0 {
            tracing::warn!(
                "Held phases on {held} traffic light(s); {failed} refused. Those rely on the \
                 freeze alone."
            );
        } else {
            tracing::info!("Held phases on {held} traffic light(s) ({HOLD_PHASE_SECONDS} s each)");
        }
    }

    /// Put back the phase durations [`hold_light_phases`](Self::hold_light_phases) saved.
    fn release_light_phases(&mut self) {
        if self.held_light_timings.is_empty() {
            return;
        }
        let actors = match self.world.actors() {
            Ok(a) => a,
            Err(e) => {
                tracing::warn!("Could not list actors to restore traffic light phases: {e}");
                return;
            }
        };
        let mut restored = 0usize;
        for (id, g, y, r) in std::mem::take(&mut self.held_light_timings) {
            let Ok(Some(actor)) = actors.find(id) else {
                continue;
            };
            let carla::client::ActorKind::TrafficLight(light) = actor.into_kinds() else {
                continue;
            };
            let outcome = light
                .set_green_time(g)
                .and_then(|()| light.set_yellow_time(y))
                .and_then(|()| light.set_red_time(r));
            match outcome {
                Ok(()) => restored += 1,
                Err(e) => tracing::debug!("Could not restore phases of traffic light {id}: {e}"),
            }
        }
        tracing::info!("Restored CARLA phase durations on {restored} traffic light(s)");
    }

    /// Unfreeze CARLA's traffic lights, but only if this bridge froze them.
    ///
    /// Freezing arrives with phase 009. The guard is here now so that landing the freeze
    /// cannot leave a shared CARLA server stuck on whatever states the last scenario left.
    pub fn restore_traffic_lights(&mut self) {
        if !self.froze_traffic_lights.needs_restore() {
            return;
        }

        self.release_light_phases();
        match self.world.freeze_all_traffic_lights(false) {
            Ok(()) => {
                self.froze_traffic_lights.mark_restored();
                tracing::info!("Traffic lights unfrozen; CARLA cycling restored");
            }
            Err(e) => tracing::warn!("Failed to unfreeze traffic lights: {e}"),
        }
    }

    // --- CARLA connection ------------------------------------------------------------

    /// Note a successful CARLA operation.
    fn note_carla_ok(&mut self) {
        self.consecutive_carla_failures = 0;
        self.carla_unreachable = false;
    }

    /// Note a failed CARLA operation; returns true once the connection looks lost.
    fn note_carla_failure(&mut self) -> bool {
        self.consecutive_carla_failures += 1;
        self.consecutive_carla_failures >= MAX_CONSECUTIVE_CARLA_FAILURES
    }

    /// Rebuild the CARLA client and world after an outage.
    ///
    /// What survives is deliberately explicit:
    ///
    /// - **Synchronous mode does not.** A restarted server defaults to async, and even a
    ///   surviving one is no longer known to hold our settings, so `sync_mode_enabled` is
    ///   cleared and the next frame re-applies it (invariant 1).
    /// - **Entity mappings do**, but may be stale. Actor IDs from a restarted server are
    ///   meaningless; teardown treats a missing actor as already destroyed, so stale IDs
    ///   cost a warning rather than a failure.
    /// - **SSv2 sees failures** for every request made during the outage. It decides
    ///   whether that ends the scenario; this bridge does not pretend the frames succeeded.
    fn reconnect_carla(&mut self) -> Result<()> {
        tracing::warn!(
            "Reconnecting to CARLA at {}:{} after {} consecutive failures",
            self.carla_host,
            self.carla_port,
            self.consecutive_carla_failures
        );

        let mut client = Client::connect(&self.carla_host, self.carla_port, None)
            .map_err(|e| eyre::eyre!("connect: {e}"))?;
        client
            .set_timeout(Duration::from_secs(30))
            .map_err(|e| eyre::eyre!("set timeout: {e}"))?;
        // A restarted server may be another CARLA release; refuse it like a failed connect.
        let server_version = super::version::verify(&client).map_err(|e| eyre::eyre!(e))?;
        tracing::info!(
            "CARLA server version {server_version} matches this build ({})",
            super::version::BUILT_FOR
        );
        let world = client.world().map_err(|e| eyre::eyre!("get world: {e}"))?;
        // A starting CARLA serves RPC before its first map is loaded; the map name is empty
        // until then. Settings or a load_world sent in that window wedged the server for
        // good (three restarts in a row on 2026-10-10, each fed apply_settings or
        // load_world while it held ''), so a server without a map is not reachable yet.
        let map_name = world.map().map(|m| m.name()).unwrap_or_default();
        if map_name.is_empty() {
            eyre::bail!("CARLA is still loading its first map");
        }

        let world_id = world.id().ok();
        let restarted = world_id.is_none() || world_id != self.world_id;
        self.client = std::sync::Arc::new(client);
        self.world = world;
        self.world_id = world_id;
        self.last_tick_frame = None;
        self.consecutive_carla_failures = 0;
        self.carla_unreachable = false;
        self.forget_road_graph();
        if restarted {
            self.forget_previous_server();
            // Its Traffic Manager served the old server; LibCarla keeps one per port, so it
            // must be shut down for the next use to start a fresh one.
            self.shut_down_traffic_manager("CARLA restarted: the old server's");
        }

        // Whatever CARLA we are now talking to, it is not holding our settings.
        self.sync_mode_enabled = false;
        // A restarted server froze nothing for us; a surviving one is no longer known to
        // hold our freeze either. Either way we are not the one who owes an unfreeze.
        self.froze_traffic_lights.mark_restored();
        self.held_light_timings.clear();

        tracing::info!("Reconnected to CARLA; synchronous mode will be re-applied next frame");
        // A restarted server came back with its default weather.
        self.reapply_weather("reconnect");
        Ok(())
    }

    /// `reconnect_carla`, retried every 5 s until `carla.reconnect_wait_seconds` have passed.
    ///
    /// A scenario started while CARLA is still coming back waits for it instead of failing
    /// (docs/design/failure-and-frame-budget.md). Logs once a minute, not every attempt.
    fn reconnect_carla_waiting(&mut self) -> Result<()> {
        let wait = Duration::from_secs(self.config.carla.reconnect_wait_seconds);
        let start = std::time::Instant::now();
        let mut next_log = Duration::ZERO;
        loop {
            match self.reconnect_carla() {
                Ok(()) => {
                    if start.elapsed() >= Duration::from_secs(5) {
                        tracing::info!(
                            "CARLA reachable again after {:.0} s",
                            start.elapsed().as_secs_f64()
                        );
                    }
                    return Ok(());
                }
                Err(e) if start.elapsed() + Duration::from_secs(5) < wait => {
                    if start.elapsed() >= next_log {
                        tracing::warn!(
                            "CARLA still unreachable after {:.0} s ({e}); waiting up to {} s",
                            start.elapsed().as_secs_f64(),
                            wait.as_secs()
                        );
                        next_log += Duration::from_secs(60);
                    }
                    std::thread::sleep(Duration::from_secs(5));
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// `--generate-signal-table <map dir>`: load the map's town in CARLA, resolve its signals
    /// and write `<map dir>/carla/traffic_lights.yaml`. The one place csb writes a map dir.
    pub fn generate_signal_table(&mut self, map_dir: &std::path::Path) -> Result<()> {
        self.generate_signal_table = true;
        let result = self.load_scenario_map(&map_dir.to_string_lossy());
        self.generate_signal_table = false;
        result
    }

    /// The clock a new csb starts with: the previous process's, if it left one.
    fn resume_clock(path: &std::path::Path, world_id: Option<u64>) -> EpisodeClock {
        use crate::clock_store::{load, resume, Resume};
        let Some(stored) = load(path) else {
            tracing::info!(
                "Episode clock: none recorded at {}; epoch 0",
                path.display()
            );
            return EpisodeClock::default();
        };
        match resume(stored, world_id) {
            Resume::SameEpisode(clock) => {
                tracing::info!(
                    "Episode clock: resumed epoch {} from the previous csb (same CARLA episode)",
                    fmt_ns(clock.epoch())
                );
                clock
            }
            Resume::NewEpisode(clock) => {
                tracing::warn!(
                    "Episode clock: CARLA is in another episode than the previous csb recorded \
                     ({:?} -> {world_id:?}); epoch {} -> {}, sim_time continues at {}",
                    stored.world_id,
                    fmt_ns(stored.epoch_ns),
                    fmt_ns(clock.epoch()),
                    fmt_ns(clock.sim_ns(0.0))
                );
                clock
            }
        }
    }

    /// Record the episode clock for the next csb process. Cheap (tmpfs); called after every
    /// change to it.
    fn record_clock(&mut self) {
        let Some(world_id) = self.world_id else {
            return;
        };
        let stored = crate::clock_store::StoredClock {
            world_id,
            epoch_ns: self.episode_clock.epoch(),
            last_frame: self.episode_clock.last_frame(),
        };
        if let Err(e) = crate::clock_store::save(&self.clock_path, &stored) {
            if !std::mem::replace(&mut self.warned_clock_store, true) {
                tracing::warn!(
                    "Could not record the episode clock at {} ({e}); a restarted csb would \
                     start from epoch 0",
                    self.clock_path.display()
                );
            }
        }
    }

    /// Note how a failed CARLA call failed: one that took longer than `SLOW_FAILURE` timed
    /// out, and marks CARLA unreachable until the next reconnect.
    fn note_failure_after(&mut self, started: Instant) {
        if started.elapsed() > SLOW_FAILURE && !self.carla_unreachable {
            self.carla_unreachable = true;
            tracing::warn!(
                "A CARLA call timed out after {:.1} s; treating CARLA as unreachable until it \
                 reconnects (teardown skips its RPCs; the reaper catches what is left)",
                started.elapsed().as_secs_f64()
            );
        }
    }

    /// After a reconnect to a different episode -- CARLA restarted -- drop what belonged to
    /// the old server and end its episode on the clock.
    ///
    /// Actor ids from the old server name nothing, or worse something else, on the new one,
    /// so the ledger is forgotten, not torn down: destroying by stale id could take a new
    /// server's traffic light or another client's car. Time continues by the episode rule
    /// applied to the last frame csb observed, anchored at the new episode's elapsed 0 --
    /// the rule acb applies to the last frame it received (docs/design/time-and-ticking.md).
    fn forget_previous_server(&mut self) {
        let forgotten = self.spawned_actors.len();
        self.spawned_actors.clear();
        self.entities.clear();
        self.autopilot.clear();
        // A new server holds none of their vehicles; their agents are told to stop.
        if let Some(link) = self.agent_link.as_ref() {
            link.despawn_all();
        }
        self.agent_vehicles.clear();
        self.abandon_collision_monitor();
        self.previous_longitudinal_accel.clear();
        self.settle_frames.clear();
        self.accel_history.clear();
        self.walker_height.clear();
        self.body_boxes.clear();
        self.tracked_npcs.clear();
        self.kinematic_fallback.clear();
        self.miss_watch.reset();
        self.stale_snapshots = 0;

        match self.episode_clock.last_frame() {
            Some((e_last, d_last)) => {
                let change = self.episode_clock.begin_episode(e_last, d_last);
                tracing::warn!(
                    "CARLA was restarted (new episode {:?}): forgot {forgotten} actor(s) of the \
                     old server; epoch {} -> {}, sim_time continues at {}",
                    self.world_id,
                    fmt_ns(change.old_epoch_ns),
                    fmt_ns(change.new_epoch_ns),
                    fmt_ns(change.continues_at_ns)
                );
            }
            None => tracing::warn!(
                "CARLA was restarted (new episode {:?}): forgot {forgotten} actor(s) of the old \
                 server; no frame of the old one was ever read, so the epoch stays",
                self.world_id
            ),
        }
        self.record_clock();
        // The new server's world carries no marker; a vehicle side started from here on
        // reads it.
        self.epoch_marker_due = Some("CARLA restart");
        self.write_epoch_marker();
    }

    /// Leave exactly one epoch marker in the world, recording csb's epoch (`epoch_marker`),
    /// if one is due. Stays due while csb cannot see the world's actors: the client's actor
    /// list is only as new as the last frame it received, and a csb that has not ticked yet
    /// (start-up, CARLA paused) has none -- it would spawn a duplicate, or miss a stale one.
    fn write_epoch_marker(&mut self) {
        let Some(why) = self.epoch_marker_due else {
            return;
        };
        use crate::epoch_marker as em;
        let epoch = self.episode_clock.epoch();
        let seen_a_frame = self.world.snapshot().is_ok_and(|s| s.frame() != 0);
        if !seen_a_frame {
            return;
        }
        let markers: Vec<(u32, i64)> =
            match self.world.actors().and_then(|a| a.filter(em::BLUEPRINT)) {
                Ok(actors) => actors
                    .iter()
                    .filter_map(|actor| {
                        let role = actor
                            .attributes()
                            .ok()?
                            .iter()
                            .find(|a| a.id() == "role_name")
                            .map(|a| a.value_string())?;
                        Some((actor.id(), em::parse(&role)?))
                    })
                    .collect(),
                Err(e) => {
                    tracing::warn!("Epoch marker ({why}): could not list the world's actors: {e}");
                    return;
                }
            };
        self.epoch_marker_due = None;
        if epoch == 0 {
            if let Some(&(id, recorded)) = markers.iter().max_by_key(|&&(id, _)| id) {
                tracing::warn!(
                    "Epoch marker ({why}): csb's epoch is 0 but the world records {} (actor \
                     {id}); a vehicle side started now adopts that. csb probably lost its \
                     clock record ({}); left in place",
                    fmt_ns(recorded),
                    self.clock_path.display()
                );
            }
            return;
        }
        let plan = em::plan(epoch, &markers);
        for &(id, recorded) in &plan.stale {
            if recorded != epoch {
                tracing::warn!(
                    "Epoch marker ({why}): the world records {} (actor {id}), csb's epoch is \
                     {}; replacing it",
                    fmt_ns(recorded),
                    fmt_ns(epoch)
                );
            }
            let destroyed = self
                .world
                .actors()
                .ok()
                .and_then(|a| a.find(id).ok().flatten())
                .map(|actor| actor.destroy());
            if let Some(Err(e)) = destroyed {
                tracing::warn!("Epoch marker ({why}): could not destroy actor {id}: {e}");
            }
        }
        if !plan.spawn {
            tracing::debug!(
                "Epoch marker ({why}): the world already records {}",
                fmt_ns(epoch)
            );
            return;
        }
        self.spawn_epoch_marker(why, epoch);
    }

    /// Spawn one marker recording `epoch` (nonzero), without looking at what the world holds.
    fn spawn_epoch_marker(&mut self, why: &str, epoch: i64) {
        use crate::epoch_marker as em;
        let transform = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: em::LOCATION_Z,
            },
            rotation: Rotation {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
        };
        let spawned = self
            .world
            .actor_builder(em::BLUEPRINT)
            .and_then(|b| b.set_attribute("role_name", &em::role_name(epoch)))
            .and_then(|b| b.spawn(transform));
        match spawned {
            // Not in `spawned_actors`: it belongs to the episode, not to a scenario, and
            // teardown must leave it.
            Ok(actor) => tracing::info!(
                "Epoch marker ({why}): epoch {} recorded in the world (actor {})",
                fmt_ns(epoch),
                actor.id()
            ),
            Err(e) => tracing::warn!(
                "Epoch marker ({why}): could not spawn {} ({e}); a vehicle side started in \
                 this episode starts at epoch 0, {} behind",
                em::BLUEPRINT,
                fmt_ns(epoch)
            ),
        }
    }

    /// The start-up marker, written when csb connects rather than at the first scenario
    /// tick, so that a vehicle side started before any scenario reads the right epoch.
    ///
    /// A world CARLA ticks by itself (asynchronous: the state csb leaves on shutdown) sends
    /// a frame within one of its ticks; csb waits for it and writes the marker as after any
    /// change. A paused synchronous world (a csb that did not shut down cleanly left it so)
    /// sends none until csb ticks, and csb does not tick outside a scenario. There csb
    /// spawns its marker without looking: acb reads the newest marker, so the new one
    /// wins over a stale one, and the marker stays due, so the first tick removes the
    /// stale ones and collapses duplicates.
    pub fn write_epoch_marker_at_connect(&mut self) {
        let Some(why) = self.epoch_marker_due else {
            return;
        };
        let seen_a_frame = |w: &World| w.snapshot().is_ok_and(|s| s.frame() != 0);
        if !seen_a_frame(&self.world) {
            let asynchronous = self.world.settings().is_ok_and(|s| !s.synchronous_mode);
            if asynchronous {
                let _ = self.world.wait_for_tick_or_timeout(CONNECT_FRAME_WAIT);
            }
        }
        if seen_a_frame(&self.world) {
            self.write_epoch_marker();
            return;
        }
        let epoch = self.episode_clock.epoch();
        if epoch != 0 {
            self.spawn_epoch_marker(&format!("{why}, before a frame"), epoch);
        }
    }

    /// `Initialize`, under its deadline ([`budgets::INITIALIZE_BUDGET`], 034 step 3): every
    /// watched call is capped by what is left of it, and a request past it fails naming the
    /// step instead of outliving SSv2's 300 s.
    pub fn initialize(&mut self, episode: &Episode) -> Result<i64, BackendError> {
        self.request_deadline = Some(Deadline::new(budgets::INITIALIZE_BUDGET));
        let response = self.initialize_within(episode);
        self.request_deadline = None;
        response
    }

    /// Why `Initialize` cannot go on after `step`: a call never returned, or the deadline
    /// has passed.
    fn initialize_stopped(&self, step: &str) -> Option<String> {
        if let Some(reason) = &self.wedged {
            return Some(reason.clone());
        }
        let deadline = self.request_deadline?;
        deadline.expired().then(|| {
            format!(
                "Initialize ran past its {} s budget at {step} (SSv2 waits {} s); CARLA is too \
                 slow to prepare a scenario now",
                deadline.budget().as_secs(),
                budgets::SSV2_INITIALIZE_TIMEOUT.as_secs()
            )
        })
    }

    fn initialize_within(&mut self, req: &Episode) -> Result<i64, BackendError> {
        let stopped =
            |reason: String| -> Result<i64, BackendError> { Err(BackendError::Failed(reason)) };
        tracing::info!(
            "Initialize: step_time={}, realtime_factor={}",
            req.step_time,
            req.realtime_factor
        );

        self.step_time = req.step_time;
        self.update_ticks_per_frame();

        // Forget what we believe about CARLA's mode; synchronous mode is (re-)applied below,
        // after the connection probe. If that fails, `sync_mode_enabled` stays false and
        // the first frame after the ego spawn retries it (FrameAction::EnableSyncThenTick);
        // until then frames are acknowledged without ticking.
        self.sync_mode_enabled = false;
        self.has_ego = false;

        // The reconnect logic is driven by consecutive tick failures, but a CARLA that
        // died between scenarios never gets ticked -- the stale client then poisons every
        // CARLA call this Initialize makes, and SSv2 sees failures until the bridge is
        // restarted by hand. Probe the connection first and rebuild it if it is gone;
        // the server may have been restarted and be perfectly healthy.
        if self.world.settings().is_err() {
            tracing::warn!("CARLA connection looks dead at Initialize; reconnecting");
            if let Err(e) = self.reconnect_carla_waiting() {
                return Err(BackendError::Disconnected(format!(
                    "CARLA is unreachable and reconnecting failed for {} s: {e}",
                    self.config.carla.reconnect_wait_seconds
                )));
            }
        }

        // Synchronous from here on, with csb the only ticker: nothing advances the world
        // between scenarios or before the ego spawns, so idle time is a pause and a server
        // stall cannot reach /clock as a leap (docs/design/time-and-ticking.md). acb needs no
        // free-running world to find its vehicle; it follows on_tick, and the spawn's tick
        // delivers the first frame.
        if let Err(e) = self.enable_sync_mode() {
            tracing::warn!("Could not put CARLA in synchronous mode at Initialize: {e}");
        }
        if let Some(reason) = self.initialize_stopped("synchronous mode") {
            return stopped(reason);
        }

        // Fresh run, fresh warnings -- otherwise a second scenario in one process would
        // stay quiet about problems it also has.
        self.warned_traffic_lights = false;

        // Jerk is differenced across frames; carrying last run's samples into this one
        // would report a spurious spike on the first frame.
        self.previous_longitudinal_accel.clear();
        self.settle_frames.clear();
        self.accel_history.clear();
        self.walker_height.clear();
        self.body_boxes.clear();
        self.tracked_npcs.clear();
        self.kinematic_fallback.clear();
        self.miss_watch.reset();
        self.stale_snapshots = 0;
        if self.gap_trace.recording() {
            // The last frame of the previous episode, while its actors still exist.
            self.trace_emit_staged(None);
            self.trace_contacts();
        }
        self.collision_monitor.begin_episode();
        self.gap_trace.begin_episode(
            self.collision_monitor.episode(),
            match self.config.npc_motion {
                NpcMotion::Tracked => "tracked",
                NpcMotion::Kinematic => "kinematic",
            },
            crate::contact_report::now_unix(),
        );

        // Destroy the previous run's actors BEFORE dropping the name mappings. Clearing
        // EntityManager first is what used to orphan them: the map was the only record of
        // what to clean up, so emptying it leaked every actor it referenced, and those
        // actors then blocked the spawn points this run needs.
        let report = self.destroy_all_spawned();
        // Contacts the previous episode's leftovers had are not this episode's.
        let _ = self.collision_monitor.take_traced();
        if report.attempted > 0 {
            tracing::info!(
                "Initialize: cleaned up {} actor(s) from the previous run",
                report.destroyed
            );
        }
        self.entities.clear();
        self.commanded_lights.clear();
        // The previous run's simulator-driven vehicles are gone, so is the Traffic Manager:
        // a scenario without one ticks with no TM in this process (034 step 1). The next
        // `simulator_autopilot` spawn starts a new one, seeded as configured, so
        // simulator-driven traffic repeats.
        self.retire_traffic_manager("Initialize");
        if let Some(reason) = self.initialize_stopped("the previous run's teardown") {
            return stopped(reason);
        }

        // Load the town the scenario was authored against. A mismatch is not a visible
        // failure -- every pose would be interpreted against whatever town CARLA happened
        // to hold -- so an unresolvable path fails Initialize rather than guessing.
        if let Err(e) = self.load_scenario_map(&req.lanelet2_map_path) {
            return stopped(format!("Cannot prepare the map: {e}"));
        }
        if let Some(reason) = self.initialize_stopped("the map") {
            return stopped(reason);
        }

        // Take authority over the signals (invariant 3).
        self.freeze_traffic_lights();

        // A previous bridge process may have died without tearing its vehicles down, and
        // they sit on the spawn points this run needs. Reap them after the map is settled
        // (a reload would have cleared them anyway) and before anything spawns.
        // Sensors whose vehicle is gone go in the same sweep, from the same census.
        self.reap_orphaned_actors();
        if let Some(reason) = self.initialize_stopped("the reaper") {
            return stopped(reason);
        }

        tracing::info!(
            "Initialized (step_time={}). CARLA synchronous; csb is its only ticker.",
            req.step_time
        );
        // CARLA is paused here: this is the frame SSv2's first UpdateFrame steps from.
        let simulation_time_ns = self.current_simulation_time_ns();
        self.record_clock();
        Ok(simulation_time_ns)
    }

    /// Hold a freshly spawned ego still until its bridge says localization is on it.
    ///
    /// Autoware routes from whatever pose it has, within a couple of seconds of the spawn.
    /// On every run after the first that pose belongs to the previous run -- measured at
    /// 92 to 99 m away -- and a route planned from it can survive the correction and leave
    /// the ego holding position for the rest of the run. Moving the estimate onto the new
    /// vehicle costs about 7 s of Autoware's own pipeline, which cannot be shortened from
    /// either bridge: a confident covariance on the seed measured 9.4 s against the loose
    /// one's 7.8 s, so the delay is not a search width. The only remaining option is not to
    /// start the scenario yet.
    ///
    /// Ticking here is safe -- SSv2's client does a plain blocking `recv` with no timeout
    /// set, so a slow reply costs patience rather than a failed run -- but under a managed
    /// ego it is **not sufficient**, which is why this defaults to off.
    ///
    /// CARLA frames are not the only clock in play. Autoware runs on simulation time, and
    /// under a managed ego it is SSv2 that publishes `/clock` -- the very process this is
    /// holding. So while this waits, CARLA advances and Autoware does not: NDT and the EKF
    /// never run, and the estimate never moves. Measured: with a 30 s wait, the bridge
    /// reported the estimate seated six seconds *after* the wait gave up, on the clock that
    /// SSv2 resumed the moment it was answered. A 90 s wait timed out in full.
    ///
    /// It is kept because it is correct for an unmanaged ego, where acb_bridge publishes
    /// `/clock` itself and the world genuinely keeps running while this waits.
    fn warm_up_localization(&mut self, actor_id: u32) {
        let Some(config_seconds) = self
            .sensor_release
            .as_ref()
            .map(|_| self.localization_warmup)
            .filter(|s| *s > 0.0)
        else {
            return;
        };

        // Ticking needs synchronous mode, which is otherwise switched on by the first
        // UpdateFrame after the ego appears. Doing it here is the same transition, earlier.
        if !self.sync_mode_enabled {
            if let Err(e) = self.enable_sync_mode() {
                tracing::warn!("Localization warm-up skipped: could not enter sync mode ({e:#})");
                return;
            }
        }

        let timeout = Duration::from_secs_f64(config_seconds);
        let started = Instant::now();
        let frame = Duration::from_secs_f64(self.step_time.max(0.01));
        loop {
            if let Some(notifier) = &self.sensor_release {
                if notifier.seated(actor_id) {
                    tracing::info!(
                        "Localization seated on ego actor {actor_id} after {:.1}s;                          starting the scenario",
                        started.elapsed().as_secs_f64()
                    );
                    return;
                }
            }
            if started.elapsed() >= timeout {
                tracing::warn!(
                    "No bridge reported localization seated on ego actor {actor_id} within                      {:.0}s; starting anyway. If a bridge is attached, the run may route                      from the previous run's pose -- see acb docs/issues/022.",
                    timeout.as_secs_f64()
                );
                return;
            }
            if let Err(e) = self.tick_frame() {
                tracing::warn!("Localization warm-up stopped: tick failed ({e})");
                return;
            }
            // Paced at about real time: Autoware converges on the wall clock, not on
            // simulation time, so ticking as fast as the server allows would race the
            // sensor pipeline rather than feed it.
            std::thread::sleep(frame);
        }
    }

    /// Switch CARLA into synchronous mode at `self.step_time` divided by `self.substeps`.
    ///
    /// SSv2's frame still advances by `step_time`; it is delivered in `substeps` ticks, so
    /// the scenario's timeline is unchanged while control commands are picked up more
    /// often. See `config::default_substeps` for what that costs.
    fn enable_sync_mode(&mut self) -> Result<()> {
        let timing = self.sync_timing();
        let delta = timing.fixed_delta_seconds;
        let mut settings = self.world.settings().wrap_err("get settings")?;
        apply_sync_timing(&mut settings, timing);
        self.apply_settings_watched("synchronous mode", settings)?;
        self.sync_mode_enabled = true;
        self.set_traffic_manager_synchronous(true)?;
        tracing::info!(
            "CARLA sync mode enabled, fixed_delta_seconds={delta} ({} substep(s) per {}s \
             SSv2 frame), physics max_substep_delta_time={} x max_substeps={}",
            self.substeps,
            self.step_time,
            timing.max_substep_delta_time,
            timing.max_substeps
        );
        Ok(())
    }

    /// Re-derive the ticks per frame from the current step time and log a change.
    fn update_ticks_per_frame(&mut self) {
        let ticks = crate::config::ticks_per_frame(
            self.step_time,
            self.config.substeps,
            self.config.carla_tick_seconds,
        );
        if ticks != self.substeps {
            tracing::info!(
                "CARLA ticks per SSv2 frame: {} -> {ticks} (step_time={}s, CARLA tick {}s)",
                self.substeps,
                self.step_time,
                self.step_time / ticks as f64
            );
            self.substeps = ticks;
        }
    }

    /// The CARLA timing for the current step: one SSv2 frame split into `substeps` ticks,
    /// each with short PhysX substeps. See [`sync_timing`].
    fn sync_timing(&self) -> SyncTiming {
        sync_timing(self.step_time, self.substeps)
    }

    /// Advance CARLA by one SSv2 frame, in `substeps` ticks.
    ///
    /// Returns the first error seen. Later substeps are skipped once one fails: the frame
    /// is already wrong, and the caller treats a tick failure as a connection signal.
    ///
    /// On success, returns the simulation time (ns) after the last tick (0 if the snapshot
    /// could not be read, the protocol's "unknown").
    fn tick_frame(&mut self) -> std::result::Result<i64, TickFailure> {
        let substeps = self.substeps.max(1);
        let ssv2_frame = self.collision_monitor.frame();
        let mut substep = 0;
        let frame = tick_then_read(
            self,
            substeps,
            |c| {
                substep += 1;
                let started = Instant::now();
                let step = format!("UpdateFrame {ssv2_frame}, tick {substep} of {substeps}");
                let hang =
                    substep == 1 && c.hang_hook.as_ref().is_some_and(|h| h.fires_at(ssv2_frame));
                let result = if hang {
                    c.watched_hang(&step)
                } else {
                    c.watched_tick(&step)
                };
                c.tick_time += started.elapsed();
                result.map(|_| ())
            },
            |c| read_frame_of(&c.world),
        );
        let frame = frame?;
        // A marker still due (start-up, or a paused world after a change) can be written
        // now: csb sees the world's actors from its first tick on.
        self.write_epoch_marker();
        Ok(match frame {
            Ok((elapsed, delta)) => self.episode_clock.observe(elapsed, delta),
            Err(e) => {
                tracing::warn!("Could not read CARLA's time after the frame; reporting 0: {e}");
                0
            }
        })
    }

    /// One `world.tick()` under the watchdog (roadmap 032 N5): the CARLA frame it returned,
    /// noted for the collision labels (032 N2). `step` names the caller in the error.
    ///
    /// A tick that does not return within [`budgets::TICK_WATCHDOG`] wedges the bridge
    /// ([`CarlaBackend::wedged`]): CARLA's frame and settings are logged, and this and every
    /// later tick fail without touching CARLA.
    fn watched_tick(&mut self, step: &str) -> std::result::Result<u64, TickFailure> {
        let mut world = self.world.clone();
        let bound = self.bound(budgets::TICK_WATCHDOG);
        match self.watched("world.tick()", step, bound, move || {
            world.tick_or_timeout(budgets::LIBCARLA_TICK_TIMEOUT)
        }) {
            Ok(Ok(frame)) => {
                self.last_tick_frame = Some(frame);
                self.collision_monitor.note_tick(frame);
                Ok(frame)
            }
            Ok(Err(e)) => Err(TickFailure::Carla(e)),
            Err(reason) => Err(TickFailure::Hung(reason)),
        }
    }

    /// The test fault (034 step 2): a "tick" that blocks the helper thread forever, watched
    /// exactly like a real one.
    fn watched_hang(&mut self, step: &str) -> std::result::Result<u64, TickFailure> {
        tracing::warn!("TEST FAULT HOOK: the tick of {step} is made to hang");
        match self.watched(
            "world.tick()",
            step,
            budgets::TICK_WATCHDOG,
            || -> std::result::Result<u64, carla::CarlaError> {
                loop {
                    std::thread::park();
                }
            },
        ) {
            Ok(Ok(frame)) => Ok(frame),
            Ok(Err(e)) => Err(TickFailure::Carla(e)),
            Err(reason) => Err(TickFailure::Hung(reason)),
        }
    }

    /// Run one CARLA call that can block inside LibCarla -- a tick, a settings change, a map
    /// load, a Traffic Manager call -- on the watchdog's helper thread, waiting at most
    /// `bound`. `Err` is the request's error once the call did not return: the bridge is
    /// wedged ([`CarlaBackend::wedged`]) and nothing more is sent to CARLA from this process.
    fn watched<T: Send + 'static>(
        &mut self,
        call: &str,
        step: &str,
        bound: Duration,
        job: impl FnOnce() -> T + Send + 'static,
    ) -> std::result::Result<T, String> {
        if let Some(reason) = &self.wedged {
            return Err(reason.clone());
        }
        match self.tick_watchdog.run(bound, job) {
            Ok(value) => Ok(value),
            Err(e) => Err(self.call_hung(call, step, bound, e)),
        }
    }

    /// A watched call that never returned: log what CARLA says now, wedge the bridge, and
    /// return the request's error.
    fn call_hung(&mut self, call: &str, step: &str, bound: Duration, error: WatchError) -> String {
        let waited = match error {
            WatchError::Expired { waited } => format!("{:.1} s", waited.as_secs_f64()),
            WatchError::Wedged => "an earlier call".to_string(),
            WatchError::WorkerGone => "no watchdog thread".to_string(),
        };
        let state = carla_state_for_log(&self.world);
        let reason = format!(
            "CARLA's {call} in {step} did not return (waited {waited}, bound {} s); {state}; \
             last tick returned CARLA frame {:?}, Traffic Manager {}. csb ends this bridge so \
             its supervisor starts a fresh one",
            bound.as_secs(),
            self.last_tick_frame,
            if self.traffic_manager.is_some() {
                "running"
            } else {
                "not running"
            }
        );
        tracing::error!("{reason}");
        self.wedged = Some(reason.clone());
        reason
    }

    /// Why the bridge cannot go on, once a tick never returned (roadmap 032 N5): the
    /// server replies to the request in flight and then ends the process.
    pub fn wedged(&self) -> Option<&str> {
        self.wedged.as_deref()
    }

    /// Check, between scenarios, that the CARLA connection is still alive (the port-2000
    /// hang, 2026-10-16).
    ///
    /// A CARLA client outlives its server badly. Its RPC connection never reconnects (every
    /// call then times out), but LibCarla's streaming client does: the world-tick
    /// subscription reattaches to whatever server comes up next on the same port, because
    /// the broadcast stream's token is the same on every server. On each frame flagged with
    /// a light update (any map load) the client's LightManager queries the lights from
    /// inside that stream callback, over the dead RPC connection, and blocks for the full
    /// RPC timeout (30 s) without reading the stream. A synchronous CARLA 0.9.16 server
    /// busy-waits on its game thread in `ServerSession::Write` until each subscriber took the
    /// previous frame, so the whole server stops answering until the timeout passes. An
    /// idle bridge sends nothing to CARLA and so never noticed its server was replaced: it
    /// stalled every fresh server on its port at the first map load in synchronous mode.
    ///
    /// The version RPC is answered by the server's RPC threads, so a live server answers
    /// even while its world waits for a tick. After [`budgets::IDLE_PROBE_FAILURES`]
    /// failures the connection is lost; the bridge then exits, as after a hung tick, so the
    /// supervisor starts a fresh process whose client has no stale subscription. (Dropping
    /// the client in-process is not enough: every actor and world handle keeps LibCarla's
    /// episode, and its subscription, alive.)
    pub fn probe_connection_idle(&mut self) {
        if self.server_lost.is_some() || self.wedged.is_some() {
            return;
        }
        if std::sync::Arc::get_mut(&mut self.client).is_none() {
            return; // a watched call still holds the client; not idle
        }
        self.set_client_timeout(budgets::IDLE_PROBE_TIMEOUT);
        let probe = self.client.server_version();
        self.set_client_timeout(budgets::CLIENT_RPC_TIMEOUT);
        match probe {
            Ok(_) => self.idle_probe_failures = 0,
            Err(e) => {
                self.idle_probe_failures += 1;
                tracing::warn!(
                    "Idle check: CARLA did not answer the version RPC ({e}); {} of {}",
                    self.idle_probe_failures,
                    budgets::IDLE_PROBE_FAILURES
                );
                if self.idle_probe_failures >= budgets::IDLE_PROBE_FAILURES {
                    let reason = format!(
                        "the CARLA connection to {}:{} is dead (server gone or replaced); a \
                         client that outlives its server stalls the next one",
                        self.carla_host, self.carla_port
                    );
                    tracing::error!("{reason}");
                    self.server_lost = Some(reason);
                }
            }
        }
    }

    /// Why the bridge exits after the idle check found its connection dead.
    pub fn server_lost(&self) -> Option<&str> {
        self.server_lost.as_deref()
    }

    /// The bridge ends after a hung tick: keep what the run recorded without a word to
    /// CARLA (the tick thread may still be inside LibCarla) -- close the gap trace, marked
    /// incomplete, and leave the actors to the next bridge's orphan reaper, which destroys
    /// each with its children.
    pub fn abandon_after_hang(&mut self) {
        self.abandon_collision_monitor();
        self.gap_trace.end_episode();
        self.gap_trace.shutdown(Duration::from_secs(5));
    }

    /// Time spent inside `world.tick()` since the last call, and reset it.
    pub fn take_tick_time(&mut self) -> Duration {
        std::mem::take(&mut self.tick_time)
    }

    /// Entities SSv2 currently has spawned through this bridge.
    pub fn entity_count(&self) -> usize {
        self.entities.count()
    }

    pub fn update_frame(&mut self, current_simulation_time: f64) -> Result<i64, BackendError> {
        // Stamp this frame's collisions with this frame's number, and log last frame's.
        let frame = self.collision_monitor.next_frame();
        self.drain_collisions();
        if self.gap_trace.recording() {
            self.trace_contacts();
            self.gap_trace.frame_time(frame, current_simulation_time);
        }

        match decide_frame_action(self.sync_mode_enabled, self.has_ego) {
            FrameAction::WaitForEgo => {
                // CARLA is still free-running so acb_bridge can find its vehicle. Ticking
                // here would be meaningless in async mode, so just acknowledge the frame.
                tracing::debug!("UpdateFrame before ego spawn: staying async, not ticking");
                // Still report a time, so SSv2 has one from its first frame.
                return Ok(self.current_simulation_time_ns());
            }
            FrameAction::EnableSyncThenTick => {
                if let Err(e) = self.enable_sync_mode() {
                    return Err(BackendError::Failed(format!(
                        "Failed to enable sync mode: {e}"
                    )));
                }
            }
            FrameAction::Tick => {}
        }

        let tick_started = Instant::now();
        let simulation_time_ns = match self.tick_frame() {
            Ok(t) => t,
            Err(TickFailure::Hung(reason)) => return Err(BackendError::Failed(reason)),
            Err(TickFailure::Carla(e)) => {
                tracing::error!("world.tick() failed: {e}");
                self.note_failure_after(tick_started);

                // A tick failure is the bridge's most reliable connection signal: SSv2 drives
                // frames continuously, so repeated failures here mean CARLA is gone rather
                // than merely idle.
                if self.note_carla_failure() {
                    if let Err(re) = self.reconnect_carla() {
                        tracing::error!("CARLA reconnection failed: {re}");
                    }
                }

                return Err(BackendError::Failed(format!("tick failed: {e}")));
            }
        };

        self.note_carla_ok();
        self.record_clock();
        Ok(simulation_time_ns)
    }

    pub fn update_step_time(&mut self, step_time: f64) -> Result<(), BackendError> {
        self.step_time = step_time;
        self.update_ticks_per_frame();

        // Before sync mode is on, just remember the value. Applying fixed_delta_seconds
        // while still async would pin CARLA to a fixed timestep without a ticker, which
        // is not what "async until the ego spawns" means. enable_sync_mode() picks up
        // self.step_time when it runs.
        if !self.sync_mode_enabled {
            tracing::info!("Recorded step_time={step_time} (applies when sync mode is enabled)");
            return Ok(());
        }

        let mut settings = match self.world.settings() {
            Ok(s) => s,
            Err(e) => {
                return Err(BackendError::Failed(format!("Failed to get settings: {e}")));
            }
        };

        // The CARLA tick is the SSv2 step split into substeps, exactly as enable_sync_mode
        // sets it. This used to write the full step here, so a mid-run UpdateStepTime with
        // `substeps: 2` ran CARLA at twice the scenario's speed.
        let timing = self.sync_timing();
        apply_sync_timing(&mut settings, timing);
        tracing::info!(
            "step_time={} applied: fixed_delta_seconds={} ({} substep(s))",
            self.step_time,
            timing.fixed_delta_seconds,
            self.substeps
        );
        if let Err(e) = self.apply_settings_watched("UpdateStepTime", settings) {
            return Err(BackendError::Failed(format!(
                "Failed to apply settings: {e}"
            )));
        }
        Ok(())
    }

    /// Spawn one scenario entity into CARLA.
    ///
    /// Shared by all four entity kinds: the differences (blueprint fallback, `role_name`,
    /// whether CARLA physics drives it) live in [`SpawnKind`] rather than in four copies of
    /// this logic.
    fn spawn_entity(
        &mut self,
        name: &str,
        asset_key: &str,
        pose: Option<&Pose>,
        shape: EntityShape,
        kind: SpawnKind,
        role_name: Option<&str>,
    ) -> Result<(), BackendError> {
        // One deadline for the whole request (034 step 4): every attempt, the adoption of
        // a late actor, the settling tick and the RPCs after it.
        self.request_deadline = Some(Deadline::new(budgets::SPAWN_REQUEST_BUDGET));
        let result = self.spawn_entity_within(name, asset_key, pose, shape, kind, role_name);
        self.request_deadline = None;
        self.set_client_timeout(budgets::CLIENT_RPC_TIMEOUT);
        result
    }

    /// The highest actor id csb knows of: CARLA hands ids out in increasing order and never
    /// reuses them, so an actor created after this has a higher one.
    fn id_watermark(&self) -> Option<u32> {
        let seen = self
            .world
            .actors()
            .ok()
            .and_then(|actors| actors.iter().map(|a| a.id()).max());
        seen.max(self.spawned_actors.max_id())
            .max(self.highest_actor_id)
    }

    fn spawn_entity_within(
        &mut self,
        name: &str,
        asset_key: &str,
        pose: Option<&Pose>,
        mut shape: EntityShape,
        kind: SpawnKind,
        role_name: Option<&str>,
    ) -> Result<(), BackendError> {
        // Resolve the blueprint once up front so the retry loop does not re-probe CARLA.
        // The configured blueprint_map takes first refusal: SSv2 asset keys are not CARLA
        // blueprint names in general, and this is where an operator states the translation.
        // Below both explicit words, the body closest to the declared box (roadmap 022),
        // else the kind's default.
        let mapped = self.config.blueprint_for(asset_key).map(str::to_owned);
        let size_choice = self.size_matched_blueprint(name, kind, &shape);
        let fallback = size_choice
            .as_ref()
            .map(|c| c.blueprint.clone())
            .unwrap_or_else(|| kind.default_blueprint().to_string());
        let requested_key = match (mapped.as_deref(), asset_key.is_empty()) {
            (Some(mapped), _) => {
                tracing::debug!("Asset key '{asset_key}' mapped to blueprint '{mapped}'");
                mapped
            }
            (None, true) => fallback.as_str(),
            (None, false) => asset_key,
        };
        let blueprint_key = match self.resolve_blueprint_key(requested_key, &fallback, kind) {
            Ok(k) => k,
            Err(e) => return failed(format!("Cannot spawn '{name}': {e}")),
        };
        // A prop matched turned is spawned turned, here and on every pose update after.
        if size_choice.is_some_and(|c| c.turned && c.blueprint == blueprint_key) {
            shape.origin_offset.yaw = std::f64::consts::FRAC_PI_2;
            tracing::info!("'{name}' spawns '{blueprint_key}' turned 90° to fit its box");
        }

        let origin_offset = shape.origin_offset;
        tracing::info!(
            "Spawn {}: name={name}, asset_key={asset_key}, origin offset \
             (bounding_box.center)=({:.3}, {:.3}) m",
            kind.label(),
            origin_offset.x,
            origin_offset.y
        );

        // Convert pose from ROS to CARLA frame, moving it from the SSv2 entity origin to
        // the CARLA actor origin (see OriginOffset).
        let mut carla_transform = match pose {
            Some(pose) => ros_pose_to_carla_transform(pose, origin_offset, self.map_origin),
            None => Transform {
                location: Location {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: Rotation {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                },
            },
        };

        // Place just above the actual ground under the commanded x/y. The old flat
        // `z < 0.5 => 0.5` clamp assumed a level map: on an elevated road it buried the
        // actor, and in a dip it dropped one in from height.
        carla_transform.location.z = self.resolve_spawn_height(&carla_transform.location);

        // Freeze the world before the ego goes in (roadmap 015, the init-window fix). In
        // async mode a spawn that stalls the server -- the first vehicle of a fresh CARLA,
        // a walker -- comes back as one frame whose elapsed_seconds leaps by the stall
        // (measured 1.5 s and 6.6 s), and acb publishes that leap as /clock: every
        // heartbeat and topic timeout in Autoware fires at once and mrm_handler blips
        // EMERGENCY_STOP. In sync mode a stall is only a pause. The tick after the spawn
        // (below) puts the ego into the episode, so acb finds it as before.
        if sync_before_spawn(kind, self.sync_mode_enabled) {
            match self.enable_sync_mode() {
                Ok(()) => tracing::info!(
                    "CARLA synchronous from the ego spawn on: spawns stall the server, and an \
                     async stall would reach /clock as a leap"
                ),
                Err(e) => tracing::warn!(
                    "Could not enter sync mode before the ego spawn ({e:#}); a slow spawn may \
                     leap /clock. The first frame will switch it on as before."
                ),
            }
        }

        // Retry a colliding spawn at increasing height, same x/y. A leftover actor on the
        // point is the usual cause; teardown and the reaper prevent most of those, and lifting
        // clears the rest. A spawn call that failed (a timeout) is never lifted over: its
        // actor may exist, and is looked for and adopted (034 step 4, `spawn_ladder`).
        let base_z = carla_transform.location.z;
        let base_x = carla_transform.location.x;
        let base_y = carla_transform.location.y;

        // An entity without a configured role still carries one: the mark that lets the
        // next bridge reap it if this one dies mid-scenario (reap_orphaned_actors).
        let entity_role = entity_role_name(name);
        let (blueprint, role) = {
            let builder = match self.build_actor(&blueprint_key, role_name, &entity_role) {
                Ok(b) => b,
                Err(e) => return failed(format!("Cannot build '{name}': {e}")),
            };
            let blueprint = builder.into_blueprint();
            // What `build_actor` set: the configured role, else the entity mark, where the
            // blueprint has the attribute.
            let role = blueprint
                .contains_attribute("role_name")
                .then(|| role_name.unwrap_or(&entity_role).to_string());
            (blueprint, role)
        };
        let watermark = self.id_watermark();
        let deadline = self
            .request_deadline
            .unwrap_or_else(|| Deadline::new(budgets::SPAWN_REQUEST_BUDGET));
        let outcome = {
            let mut backend = CarlaSpawn {
                backend: self,
                blueprint,
                blueprint_id: blueprint_key.clone(),
                role,
                transform: carla_transform.clone(),
            };
            match watermark {
                Some(w) => spawn_ladder::run(
                    &mut backend,
                    spawn_retry_heights(base_z),
                    w,
                    &deadline,
                    budgets::CLIENT_RPC_TIMEOUT,
                    Instant::now,
                ),
                None => spawn_ladder::Outcome::Failed(
                    "no actor id watermark (CARLA's actor list unreadable), so a timed-out \
                     spawn could not be told apart; not attempted"
                        .to_string(),
                ),
            }
        };
        // What is left of the request bounds every RPC after the spawn.
        let rpc_timeout = self.bound(budgets::CLIENT_RPC_TIMEOUT);
        self.set_client_timeout(rpc_timeout);

        let (actor_id, spawn_z) = match outcome {
            spawn_ladder::Outcome::Spawned { actor, attempt, z } => {
                for failed in 0..attempt {
                    tracing::warn!(
                        "Spawn of '{name}' at z={:.2} collided (attempt {}/{})",
                        base_z + failed as f32 * SPAWN_RETRY_STEP,
                        failed + 1,
                        SPAWN_RETRIES + 1
                    );
                }
                if attempt > 0 {
                    // Lifting is meant for ground that sits higher than the pose says.
                    // If something is parked on the commanded point instead, the lift
                    // has put this vehicle on top of it, and the run needs to know which
                    // actor rather than just that the first attempts failed.
                    match self.vehicle_occupying(base_x, base_y, actor) {
                        Some(other) => tracing::warn!(
                            "Spawned '{name}' on attempt {} at z={z:.2} (commanded \
                             z={base_z:.2}) because {other} is already on that point -- \
                             '{name}' is now stacked above it",
                            attempt + 1
                        ),
                        None => tracing::info!(
                            "Spawned '{name}' on attempt {} at z={z:.2} (commanded z={base_z:.2})",
                            attempt + 1
                        ),
                    }
                }
                (actor, z)
            }
            spawn_ladder::Outcome::Adopted {
                actor,
                attempt,
                z,
                error,
                others,
            } => {
                tracing::warn!(
                    "Spawn of '{name}' at z={z:.2} failed as a call (attempt {}: {error}); CARLA \
                     created it anyway as actor {actor} (above id watermark {}), adopted, not \
                     spawned again",
                    attempt + 1,
                    watermark.unwrap_or_default()
                );
                if !others.is_empty() {
                    tracing::warn!(
                        "More actors of '{blueprint_key}' with its role above the watermark: \
                         {others:?}; left to the reaper"
                    );
                }
                (actor, z)
            }
            spawn_ladder::Outcome::Failed(why) => {
                return failed(format!(
                    "Failed to spawn {} '{name}' (blueprint '{blueprint_key}') at \
                     CARLA({base_x:.1}, {base_y:.1}, {base_z:.1}): {why}",
                    kind.label()
                ));
            }
        };
        // Record for teardown before anything else can fail.
        self.record_spawned(actor_id);
        let actor = match self.world.actor(actor_id) {
            Ok(Some(a)) => a,
            Ok(None) => {
                return failed(format!(
                    "Spawned '{name}' as actor {actor_id}, but CARLA no longer has it"
                ))
            }
            Err(e) => {
                return failed(format!(
                    "Spawned '{name}' as actor {actor_id}, but cannot read it back: {e}"
                ))
            }
        };
        let mut spawn_loc = carla_transform.location;
        spawn_loc.z = spawn_z;

        let bb = actor.bounding_box();
        let read = Dimensions {
            length: 2.0 * f64::from(bb.extent.x),
            width: 2.0 * f64::from(bb.extent.y),
            height: 2.0 * f64::from(bb.extent.z),
        };
        let actual = if kind == SpawnKind::MiscObject {
            prop_box_in_actor_frame(read, carla_transform.rotation.yaw)
        } else {
            Some(read)
        };
        // The gap trace's CARLA box: the same box the geometry row reports, centred where
        // CARLA says, both in the actor's own frame, ROS-handed.
        let trace_box = actual.map(|d| {
            let l = &bb.transform.location;
            let (x, y) = if kind == SpawnKind::MiscObject {
                prop_offset_in_actor_frame(
                    f64::from(l.x),
                    f64::from(l.y),
                    carla_transform.rotation.yaw,
                )
            } else {
                (f64::from(l.x), f64::from(l.y))
            };
            gap_trace::BlueprintBox {
                location: [x, -y, f64::from(l.z)],
                extent: [d.length / 2.0, d.width / 2.0, d.height / 2.0],
            }
        });
        self.compare_geometry(name, &blueprint_key, &shape, actual);

        if kind == SpawnKind::Pedestrian {
            // SSv2 sends a pedestrian's pose at its feet every frame; CARLA's walker origin
            // is its capsule centre. CARLA seats a walker on the ground at spawn (measured:
            // origin 0.951 m up on Town01), but every teleport after that took SSv2's z
            // unchanged and sank the walker to its waist (origin z = 0.000). Remember the
            // lift here; update_entity_status puts the origin that far above the ground
            // under each teleport (see walker_teleport_z).
            let bb = actor.bounding_box();
            let lift = walker_lift(bb.transform.location.z, bb.extent.z);
            self.walker_height
                .insert(actor_id, WalkerHeight { lift, ground: None });
            tracing::debug!("Walker '{name}' origin sits {lift:.3} m above its feet");
        }

        // Hand pose authority to whoever owns it (invariant 5). For everything SSv2
        // teleports, CARLA physics must be off, or PhysX fights set_transform every frame:
        // gravity pulls the actor down and collision response shoves it out of position
        // between ticks, while the pose reported back to SSv2 is the commanded one -- so
        // the divergence is invisible to the scenario.
        let tracked = kind.trackable() && self.config.npc_motion == NpcMotion::Tracked;
        if tracked {
            // Physics stays on: npc_tracking drives it toward SSv2's pose every frame
            // (roadmap 023). No settle masking -- SSv2 reports its NPCs' state itself -- and
            // no hand brake, which would fight the commanded velocity.
            let steering = self.npc_steering(&actor, &shape);
            self.tracked_npcs.insert(
                actor_id,
                TrackedNpc {
                    walker: kind == SpawnKind::Pedestrian,
                    steering,
                    performance: shape.performance,
                    last_yaw_deg: None,
                    last_command: None,
                    blocked: false,
                    held: false,
                    first: true,
                    lift: {
                        let bb = actor.bounding_box();
                        walker_lift(bb.transform.location.z, bb.extent.z)
                    },
                },
            );
            tracing::info!(
                "'{name}' is tracked (physics on){}",
                match (kind, steering) {
                    (SpawnKind::Npc, None) => ", no steer feed-forward (no axles declared)",
                    _ => "",
                }
            );
        } else if !kind.physics_driven() {
            if let Err(e) = actor.set_simulate_physics(false) {
                // Not fatal, but the actor will not track its commanded pose properly.
                tracing::warn!(
                    "Could not disable physics on {} '{name}' (actor {actor_id}): {e}. \
                     It may drift from its commanded pose.",
                    kind.label()
                );
            }
        } else {
            // A physics actor spawns above the road (SPAWN_Z_CLEARANCE) and slams onto
            // its suspension in the first ticks; report zero acceleration while it
            // settles so the impact spike is not read as vehicle behaviour.
            self.settle_frames.insert(actor_id, SETTLE_FRAMES);

            // Park it until its driver takes over. A fresh physics vehicle free-rolls
            // (observed ~1 m/s while Autoware initializes), and the first stop command
            // acb_bridge then forwards slams the brake at speed -- a one-frame
            // deceleration spike past SSv2's [-5, 3] m/s² sanity bounds, which is
            // scenario-fatal. Any later apply_control from the driver replaces this
            // whole control struct, hand_brake included.
            if let carla::client::ActorKind::Vehicle(vehicle) = actor.clone().into_kinds() {
                let park = carla::rpc::VehicleControl {
                    throttle: 0.0,
                    steer: 0.0,
                    brake: 1.0,
                    hand_brake: true,
                    reverse: false,
                    manual_gear_shift: false,
                    gear: 0,
                };
                if let Err(e) = vehicle.apply_control(&park) {
                    tracing::warn!(
                        "Could not park '{name}' (actor {actor_id}) after spawn: {e}. \
                         It may roll until its driver's first command."
                    );
                }
            }
        }

        // Every physics body's box, for the proximity hold of tracked NPCs (roadmap 032 N2).
        if tracked || kind.physics_driven() {
            self.body_boxes.insert(
                actor_id,
                BodyBox {
                    offset_x: f64::from(bb.transform.location.x),
                    offset_y: f64::from(bb.transform.location.y),
                    half_length: f64::from(bb.extent.x),
                    half_width: f64::from(bb.extent.y),
                },
            );
        }

        // Wait one tick for the actor to be fully initialized. Only meaningful once we
        // own the tick -- before sync mode is on, CARLA is advancing by itself.
        //
        // SAFETY: a failure here is not fatal to the spawn. The actor exists; it is merely
        // not yet settled. A genuinely broken connection is caught by update_frame, which
        // owns reconnection.
        if self.sync_mode_enabled {
            match self.watched_tick(&format!("the settling tick after spawning '{name}'")) {
                Ok(_) => {}
                Err(TickFailure::Carla(e)) => {
                    tracing::warn!("Tick after spawning '{name}' failed: {e}")
                }
                // Not registered: the bridge ends after this reply.
                Err(TickFailure::Hung(reason)) => return failed(reason),
            }
        }

        // Everything SSv2 spawns is registered (SpawnKind::entity_type).
        if let Some(entity_type) = kind.entity_type() {
            self.entities
                .insert(name.to_string(), entity_type, actor_id, origin_offset);
            if self.gap_trace.recording() {
                self.gap_trace.entity(gap_trace::EntityHeader {
                    frame: self.collision_monitor.frame(),
                    name: name.to_string(),
                    kind: match entity_type {
                        EntityType::Ego => gap_trace::Kind::Ego,
                        EntityType::Vehicle => gap_trace::Kind::Vehicle,
                        EntityType::Pedestrian => gap_trace::Kind::Pedestrian,
                        EntityType::MiscObject => gap_trace::Kind::MiscObject,
                    },
                    declared: gap_trace::DeclaredBox {
                        center: shape.center,
                        dimensions: shape
                            .declared
                            .map_or([0.0; 3], |d| [d.length, d.width, d.height]),
                    },
                    origin_offset: [origin_offset.x, origin_offset.y, origin_offset.yaw],
                    actor_id,
                    blueprint: blueprint_key.clone(),
                    blueprint_box: trace_box,
                });
            }
        }

        if kind == SpawnKind::Ego {
            // Releases the sync-mode gate: from the next UpdateFrame onward this bridge
            // owns the tick. See FrameAction.
            self.has_ego = true;
            self.collision_monitor.attach(&mut self.world, &actor, name);
            self.warm_up_localization(actor_id);
            // A warm-up tick that never returned (roadmap 032 N5): fail the spawn, so SSv2
            // has the error before the bridge ends.
            if let Some(reason) = &self.wedged {
                return failed(reason.clone());
            }
        } else if tracked && kind == SpawnKind::Npc {
            // Its own contacts -- the static world, other NPCs -- which the ego's sensor
            // cannot see (roadmap 032 step 5). Detached before every destroy of the NPC.
            self.collision_monitor
                .attach_npc(&mut self.world, &actor, name);
        }

        tracing::info!(
            "Spawned {} '{name}' (actor_id={actor_id}, blueprint='{blueprint_key}', \
             physics={}) at CARLA({:.1}, {:.1}, {:.1})",
            kind.label(),
            kind.physics_driven() || self.tracked_npcs.contains_key(&actor_id),
            spawn_loc.x,
            spawn_loc.y,
            spawn_loc.z
        );

        Ok(())
    }

    /// `Spawn*Entity`: one scenario entity into CARLA.
    pub fn spawn(&mut self, req: &SpawnRequest) -> Result<(), BackendError> {
        let name = req.name.clone();
        match &req.kind {
            EntityKind::Vehicle {
                driver,
                subtype,
                axles,
                performance,
            } => self.spawn_vehicle(req, *driver, *subtype, *axles, *performance),
            // No AI walker controller. SSv2's behaviour plugins compute the walk and send a
            // pose every frame, so a controller would be a second authority over the same
            // actor (invariant 5). The walker is handled exactly like an NPC vehicle: tracked
            // with `WalkerControl` under `npc_motion: tracked` (the default), else spawned
            // kinematic and teleported.
            EntityKind::Pedestrian => self.spawn_entity(
                &name,
                &req.asset_key,
                req.pose.as_ref(),
                EntityShape::of(req.bounding_box.as_ref(), EntitySubtype::Pedestrian),
                SpawnKind::Pedestrian,
                None,
            ),
            EntityKind::MiscObject => self.spawn_entity(
                &name,
                &req.asset_key,
                req.pose.as_ref(),
                EntityShape::of(req.bounding_box.as_ref(), EntitySubtype::Unknown),
                SpawnKind::MiscObject,
                None,
            ),
        }
    }

    fn spawn_vehicle(
        &mut self,
        req: &SpawnRequest,
        driver: Driver,
        subtype: EntitySubtype,
        axles: Option<(f64, f64)>,
        performance: Option<(f64, f64)>,
    ) -> Result<(), BackendError> {
        let name = req.name.clone();
        let kind = SpawnKind::of_driver(driver);

        // One ego per session. A second would get the same role_name, and acb_bridge would
        // pick whichever it found first; the stock backend rejects it too.
        if let Some(reason) =
            second_ego_rejection(driver == Driver::Ego, self.entities.ego_name(), &name)
        {
            return failed(reason);
        }

        // Only the vehicles an autopilot drives carry a role_name: acb_bridge finds its
        // vehicle by it, and an NPC tagged the same would be picked up as if it were an
        // Autoware vehicle. The ego's is configured; an agent-driven vehicle's is its entity
        // name, which the vehicle side is started with (vehicle_name:=<entity>).
        let role_name = match kind {
            SpawnKind::Ego => Some(self.config.ego.role_name.clone()),
            SpawnKind::AgentVehicle => Some(name.clone()),
            _ => None,
        };
        if kind == SpawnKind::AgentVehicle {
            if let Err(reason) = self.check_agent_role(&name) {
                return failed(format!(
                    "Cannot spawn '{name}' (controller {}): {reason}",
                    agent_link::BEHAVIOR
                ));
            }
        }
        let mut shape = EntityShape::of(req.bounding_box.as_ref(), subtype);
        shape.axles = axles;
        shape.performance = performance;
        // The Traffic Manager first: a simulator-driven vehicle nothing can drive must not
        // be spawned and left parked.
        if kind == SpawnKind::AutopilotNpc {
            if let Err(e) = self.traffic_manager() {
                return failed(format!(
                    "Cannot spawn '{name}' ({}): {e:#}",
                    autopilot::BEHAVIOR
                ));
            }
        }
        self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
            shape,
            kind,
            role_name.as_deref(),
        )?;
        if kind == SpawnKind::AutopilotNpc {
            if let Err(e) = self.track_autopilot_vehicle(&name) {
                return failed(format!(
                    "Spawned '{name}' but cannot drive it ({}): {e:#}",
                    autopilot::BEHAVIOR
                ));
            }
        }
        if kind == SpawnKind::AgentVehicle {
            // Whether an agent is registered yet does not matter here: the vehicle side
            // may come up later. The agent is told where its vehicle is as soon as it is
            // there; a goal for an entity with no agent is what fails (UpdateEntityGoal).
            self.agent_vehicles.insert(name.clone());
            self.store_agent_ledger();
            let pose = req.pose.as_ref().map(agent_pose);
            let link = self.agent_link();
            if let Some(pose) = pose {
                link.spawned(&name, pose);
            }
        }
        Ok(())
    }

    /// The agent link, started on first use.
    fn agent_link(&mut self) -> &mut AgentLink {
        let relay = self.config.agent_relay().to_string();
        self.agent_link
            .get_or_insert_with(|| AgentLink::start(&relay))
    }

    /// An `agent` entity's role_name is its name; refuse one another vehicle already has,
    /// since acb_bridge would attach to whichever it finds first.
    fn check_agent_role(&self, name: &str) -> std::result::Result<(), String> {
        if let Some(reason) = agent_role_conflict(name, &self.config.ego.role_name) {
            return Err(reason);
        }
        let actors = self
            .world
            .actors()
            .map_err(|e| format!("cannot list CARLA's actors: {e}"))?;
        let taken = actors.iter().find(|actor| {
            actor.type_id().starts_with("vehicle.")
                && actor.attributes().ok().is_some_and(|attrs| {
                    attrs
                        .iter()
                        .any(|a| a.id() == "role_name" && a.value_string() == name)
                })
        });
        match taken {
            Some(actor) => Err(format!(
                "CARLA already has a vehicle with role_name '{name}' (actor {}, not spawned \
                 by this bridge); its vehicle side could attach to either",
                actor.id()
            )),
            None => Ok(()),
        }
    }

    /// Start tracking a freshly spawned `simulator_autopilot` vehicle. It stays parked
    /// until SSv2's NPC logic starts (see `drive_autopilot`).
    fn track_autopilot_vehicle(&mut self, name: &str) -> Result<()> {
        let actor_id = self
            .entities
            .get(name)
            .map(|e| e.carla_actor_id)
            .ok_or_else(|| eyre::eyre!("not registered after its spawn"))?;
        let actor = self
            .world
            .actors()
            .wrap_err("get actors")?
            .find(actor_id)
            .wrap_err("find actor")?
            .ok_or_else(|| eyre::eyre!("actor {actor_id} not found"))?;
        let carla::client::ActorKind::Vehicle(vehicle) = actor.into_kinds() else {
            eyre::bail!("actor {actor_id} is not a vehicle");
        };
        self.autopilot.insert(
            name.to_string(),
            AutopilotVehicle {
                vehicle,
                engaged: false,
                parked: false,
                target_speed: None,
                route: None,
                path_pending: false,
                approach_cruise: None,
            },
        );
        Ok(())
    }

    pub fn despawn(&mut self, name: &str) -> Result<(), BackendError> {
        let was_autopilot = self.autopilot.contains_key(name);
        let response = self.despawn_entity_traced(name);
        if was_autopilot {
            // The last simulator-driven vehicle gone: so is the Traffic Manager (034 step 1).
            self.retire_traffic_manager(&format!("despawn of '{name}'"));
            if let Some(reason) = self.wedged.clone() {
                return failed(reason);
            }
        }
        response
    }

    fn despawn_entity_traced(&mut self, name: &str) -> Result<(), BackendError> {
        if !self.gap_trace.recording() {
            return self.despawn_entity_inner(name);
        }
        // The staged frame still holds this entity: write it while the actor exists.
        self.trace_emit_staged(None);
        let response = self.despawn_entity_inner(name);
        self.trace_contacts();
        if self.entities.count() == 0 {
            // SSv2 despawns every entity when a scenario ends, and sends nothing more until
            // the next Initialize -- which may never come for a suite's last scenario.
            self.gap_trace.end_episode();
        }
        response
    }

    fn despawn_entity_inner(&mut self, name: &str) -> Result<(), BackendError> {
        // Whether this despawn removes the ego, read before the entry goes away. `has_ego`
        // gates synchronous mode and the idle watchdog's threshold, and it used to be
        // cleared only at Initialize -- so after a scenario despawned its ego the bridge
        // still believed one existed, and the watchdog waited out the long "a scenario may
        // merely be paused" timeout when the short one applied.
        let was_ego = matches!(
            self.entities.get(name).map(|e| e.entity_type),
            Some(EntityType::Ego)
        );

        // Its agent stops driving it.
        if self.agent_vehicles.remove(name) {
            if let Some(link) = self.agent_link.as_ref() {
                link.despawned(name);
            }
            self.store_agent_ledger();
        }

        // Traffic Manager lets go of a simulator-driven vehicle before it is destroyed.
        if let Some(ap) = self.autopilot.remove(name) {
            if ap.engaged && !ap.parked && !self.carla_unreachable {
                if let Some(tm) = self.traffic_manager.as_mut() {
                    if let Err(e) = tm.unregister_vehicles(std::slice::from_ref(&ap.vehicle)) {
                        tracing::warn!("Could not release '{name}' from Traffic Manager: {e}");
                    }
                }
            }
        }

        if self.carla_unreachable {
            // Every RPC below would wait out the client timeout (gap 10). The actor stays in
            // the ledger: torn down after a reconnect to the same server, forgotten after a
            // restart, and reaped by its mark either way.
            if was_ego {
                self.abandon_collision_monitor();
                self.has_ego = false;
            }
            return match self.entities.remove(name) {
                Some(actor_id) => {
                    self.collision_monitor.forget_npc(actor_id);
                    tracing::warn!(
                        "Despawn '{name}' (actor {actor_id}): CARLA unreachable; left to the \
                         next teardown or the reaper"
                    );
                    Ok(())
                }
                None => failed(format!("Entity '{name}' not found")),
            };
        }

        if was_ego {
            // Our collision sensor before the ego it rides on; logs the run's summary.
            self.finish_collision_monitor();
        }

        match self.entities.remove(name) {
            Some(actor_id) => {
                // An NPC's own collision sensor (032 step 5): stopped and destroyed by this
                // client, its listener, before the vehicle goes.
                self.collision_monitor.detach_npc(actor_id);
                let started = Instant::now();
                // Report failure when the actor could not be destroyed. This used to warn
                // and return success regardless, so SSv2 believed an entity was gone while
                // it was still in the world -- blocking spawn points and appearing in
                // Autoware's sensors.
                //
                // "Already destroyed" and "not found" are both successes: the requested end
                // state holds either way.
                let outcome: Result<()> = (|| {
                    let actors = self.world.actors().wrap_err("get actors")?;
                    match actors.find(actor_id).wrap_err("find actor")? {
                        Some(actor) => {
                            // Warn whoever attached sensors to this vehicle, while those
                            // sensors are still alive and they can still stop listening.
                            // Destroying a sensor out from under its listener leaves that
                            // client retrying a dead stream at ~48k errors/s. See
                            // sensor_release.
                            self.announce_release(actor_id, name);

                            // Sensors first: a despawn is where a vehicle most often leaves
                            // acb's sensors behind, and the server segfaults on the next
                            // tick of an IMU whose owner is gone.
                            let outcome =
                                actor.destroy_with_children().wrap_err("destroy actor")?;
                            log_destroy_outcome(actor_id, &outcome);
                            if outcome.destroyed {
                                tracing::info!("Despawned '{name}' (actor_id={actor_id})");
                            } else {
                                tracing::warn!(
                                    "Despawn '{name}' (actor_id={actor_id}) returned false; \
                                     treating as already destroyed"
                                );
                            }
                        }
                        None => {
                            tracing::warn!(
                                "Despawn '{name}': actor {actor_id} not in the world; \
                                 treating as already destroyed"
                            );
                        }
                    }
                    if was_ego {
                        self.has_ego = false;
                    }
                    Ok(())
                })();

                match outcome {
                    Ok(()) => {
                        // Confirmed gone, so teardown no longer needs to chase it.
                        self.forget_spawned(actor_id);
                        Ok(())
                    }
                    Err(e) => {
                        self.note_failure_after(started);
                        failed(format!("Failed to despawn '{name}': {e}"))
                    }
                }
            }
            None => failed(format!("Entity '{name}' not found")),
        }
    }

    pub fn update_entity_status(&mut self, req: StatusBatch) -> StatusOutcome {
        let mut updated = Vec::new();
        let mut teleport_failures: Vec<String> = Vec::new();
        let mut unknown: Vec<String> = Vec::new();
        // Set when a teleport fails slowly: the client timed out, so CARLA is gone or hung,
        // and every further call would wait out the same timeout (30 s each -- a dead
        // server times out rather than refusing). The rest of the batch fails at once and
        // SSv2 ends the scenario on the failed frame (roadmap 016, gap 9).
        let mut carla_unreachable = self.carla_unreachable;
        let mut autopilot_failures: Vec<String> = Vec::new();
        let mut agent_failures: Vec<String> = Vec::new();
        if let Some(link) = self.agent_link.as_ref() {
            link.set_npc_logic_started(req.npc_logic_started);
        }
        // The contacts of the last frame's ticks, before the tracked NPCs are held by them
        // (roadmap 032 N2): otherwise they wait for the next UpdateFrame's drain, a frame
        // after the tracking that needs them.
        self.drain_collisions();
        // Tracked NPCs (roadmap 023) are read from one world snapshot and commanded through
        // one actor list per frame, fetched on the first tracked entity.
        let mut tracked_view: Option<TrackedView> = None;
        // Tracked NPCs looked up in that snapshot this frame, and those it did not have.
        let mut tracked_looked_up = 0_usize;
        let mut tracked_missing: Vec<String> = Vec::new();

        for entity_status in &req.entities {
            let name = &entity_status.name;

            let entity_info = self
                .entities
                .get(name)
                .map(|e| (e.carla_actor_id, e.entity_type, e.origin_offset));

            let Some((actor_id, entity_type, origin_offset)) = entity_info else {
                // Refused, not echoed: see entity_status_result. The rest of the batch is
                // still applied so one stale name does not freeze every other entity for
                // the frame SSv2 spends deciding to throw.
                unknown.push(name.clone());
                continue;
            };
            let is_ego = entity_type == EntityType::Ego;
            let agent_driven = self.agent_vehicles.contains(name);
            let simulator_driven = self.autopilot.contains_key(name) || agent_driven;
            if agent_driven {
                if let Some(error) = self.agent_link.as_ref().and_then(|l| l.error(name)) {
                    agent_failures.push(format!("{name}: {error}"));
                }
            }

            if simulator_driven && !agent_driven {
                // CARLA drives it: no teleport, the pose goes the other way (like the ego's).
                if carla_unreachable {
                    autopilot_failures.push(format!("{name}: skipped, CARLA unreachable"));
                } else if let Err(e) = self.drive_autopilot(name, req.npc_logic_started) {
                    tracing::warn!("Simulator autopilot for '{name}': {e:#}");
                    autopilot_failures.push(format!("{name}: {e:#}"));
                }
            }

            if (is_ego && !req.overwrite_ego_status) || simulator_driven {
                // Read ego pose from CARLA physics
                match self.read_actor_state(actor_id, origin_offset) {
                    Some((pose, action_status)) => {
                        updated.push(EntityStatus {
                            name: name.clone(),
                            action_status: Some(action_status),
                            pose: Some(pose),
                        });
                    }
                    None => {
                        // Fallback: echo what SSv2 sent
                        updated.push(EntityStatus {
                            name: name.clone(),
                            action_status: entity_status.action_status.clone(),
                            pose: entity_status.pose,
                        });
                    }
                }
            } else if let Some(npc) = self.tracked_npcs.get(&actor_id).copied() {
                // A tracked NPC (roadmap 023): moved toward SSv2's pose with a velocity,
                // snapped back past the thresholds. SSv2 is told what it sent, as for a
                // teleported NPC: it owns its NPCs' state, CARLA's residual is reported.
                if let Some(pose) = entity_status.pose.as_ref() {
                    if carla_unreachable {
                        teleport_failures.push(format!("{name}: skipped, CARLA unreachable"));
                    } else {
                        let started = std::time::Instant::now();
                        match self.track_npc(
                            name,
                            actor_id,
                            npc,
                            pose,
                            // SSv2 holds its NPCs still until their logic starts, while the
                            // twist already carries their Init speed: no motion until then.
                            entity_status
                                .action_status
                                .as_ref()
                                .filter(|_| req.npc_logic_started),
                            origin_offset,
                            &mut tracked_view,
                        ) {
                            Ok(found) => {
                                tracked_looked_up += 1;
                                if !found {
                                    tracked_missing.push(name.clone());
                                }
                            }
                            Err(e) => {
                                carla_unreachable = started.elapsed() > SLOW_FAILURE;
                                tracing::warn!("Tracking '{name}': {e:#}");
                                teleport_failures.push(format!("{name}: {e:#}"));
                            }
                        }
                    }
                }
                updated.push(EntityStatus {
                    name: name.clone(),
                    action_status: entity_status.action_status.clone(),
                    pose: entity_status.pose,
                });
            } else {
                if self.kinematic_fallback.contains(&actor_id) {
                    self.collision_monitor.record_tracking_fallback(name);
                }
                // NPC or ego overwrite: set transform from SSv2 pose
                if let Some(pose) = entity_status.pose.as_ref() {
                    let mut transform =
                        ros_pose_to_carla_transform(pose, origin_offset, self.map_origin);
                    if let Some(z) = self.walker_teleport_z(actor_id, &transform.location) {
                        transform.location.z = z;
                    }
                    // Pose only. SSv2's twist (action_status) is not passed on: CARLA accepts
                    // set_target_velocity and WalkerControl here, but a physics-off actor
                    // reports 0 m/s under both (measured on 0.9.16: 0.000 m/s kinematic vs
                    // 1.504 m/s for the same WalkerControl with physics on). Per-frame
                    // velocity and the walk animation need walker physics on; tracked as
                    // roadmap 014 gap 5.
                    if carla_unreachable {
                        teleport_failures.push(format!("{name}: skipped, CARLA unreachable"));
                    } else if let Err(e) = {
                        let started = std::time::Instant::now();
                        self.set_actor_transform(actor_id, &transform)
                            .inspect_err(|_| {
                                carla_unreachable = started.elapsed() > SLOW_FAILURE;
                            })
                    } {
                        // A failed teleport used to warn and still report success, so SSv2
                        // went on believing the NPC had moved -- the same silent divergence
                        // phase 006 exists to remove.
                        tracing::warn!("set_transform for '{name}': {e}");
                        teleport_failures.push(format!("{name}: {e}"));
                    }
                }

                // Echo back the same pose. It is SSv2's own entity-origin pose, never read
                // back from CARLA, so there is no origin offset to undo here.
                updated.push(EntityStatus {
                    name: name.clone(),
                    action_status: entity_status.action_status.clone(),
                    pose: entity_status.pose,
                });
            }
        }

        // A missing NPC was skipped above; every one missing for too long is a real loss.
        if self
            .miss_watch
            .end_frame(tracked_looked_up, tracked_missing.len())
        {
            tracing::error!(
                "Every tracked NPC has been missing from the world snapshot for more than {} \
                 frames: failing the frame",
                npc_tracking::MISS_LIMIT_FRAMES
            );
            let frame = tracked_view.as_ref().map(|v| v.snapshot.frame());
            for name in &tracked_missing {
                teleport_failures.push(format!(
                    "{name}: not in the world snapshot (frame {frame:?}) for more than {} \
                     frames",
                    npc_tracking::MISS_LIMIT_FRAMES
                ));
            }
        }

        if carla_unreachable && !self.carla_unreachable {
            self.carla_unreachable = true;
            tracing::warn!("A teleport timed out; CARLA treated as unreachable");
        }
        if self.gap_trace.recording() {
            // The tracked path's snapshot when it took one (roadmap 023), else our own.
            self.trace_entity_status(&updated, tracked_view.as_ref().map(|v| &v.snapshot));
        }
        let mut failure = entity_status_result(&unknown, &teleport_failures);
        if !autopilot_failures.is_empty() {
            let earlier = match &failure {
                None => String::from("UpdateEntityStatus: "),
                Some(earlier) => format!("{earlier}; "),
            };
            failure = Some(format!(
                "{earlier}the simulator autopilot failed for {}",
                autopilot_failures.join("; ")
            ));
        }
        if !agent_failures.is_empty() {
            let earlier = match &failure {
                None => String::from("UpdateEntityStatus: "),
                Some(earlier) => format!("{earlier}; "),
            };
            failure = Some(format!(
                "{earlier}an agent-driven vehicle cannot be driven: {}",
                agent_failures.join("; ")
            ));
        }

        StatusOutcome {
            entities: updated,
            failure,
        }
    }

    // --- Simulator-driven vehicles (simulator_autopilot) ------------------------------

    /// The Traffic Manager, created on first use: synchronous whenever the world is, with
    /// the configured seed, hybrid physics off (every vehicle simulated in full).
    fn traffic_manager(&mut self) -> Result<&mut carla::traffic_manager::TrafficManager> {
        if self.traffic_manager.is_none() {
            let port = self.config.traffic_manager.port;
            let seed = self.config.traffic_manager.seed;
            let mut tm = self
                .client
                .instance_tm(port)
                .wrap_err_with(|| format!("start Traffic Manager on port {port}"))?;
            tm.set_synchronous_mode(self.sync_mode_enabled)
                .wrap_err("Traffic Manager synchronous mode")?;
            tm.set_hybrid_physics_mode(false)
                .wrap_err("Traffic Manager hybrid physics")?;
            tm.set_random_device_seed(seed)
                .wrap_err("Traffic Manager seed")?;
            tracing::info!(
                "Traffic Manager on port {port}: synchronous={}, seed={seed}",
                self.sync_mode_enabled
            );
            self.traffic_manager = Some(tm);
            // Starting a Traffic Manager resets every light group to its own cycle (one
            // green per junction, measured on 0.9.16: 24 of Town01's 36 lights RED after
            // the first simulator_autopilot spawn). Put back what this run holds: GREEN for
            // every mapped light, then whatever the scenario commanded.
            self.set_mapped_lights_green();
            let commanded: Vec<_> = self
                .commanded_lights
                .iter()
                .map(|(od, state)| (od.clone(), state.clone()))
                .collect();
            for (od, state) in commanded {
                if let Err(e) = self.set_signal_state(&od, state) {
                    tracing::warn!("Could not re-apply the commanded state of light {od}: {e:#}");
                }
            }
        }
        Ok(self.traffic_manager.as_mut().expect("set above"))
    }

    /// Shut the Traffic Manager down once no `simulator_autopilot` entity is left (roadmap
    /// 034 step 1, [`autopilot::traffic_manager_action`]); the next such spawn starts a new
    /// one. `why` names the occasion for the log. Under the watchdog: `Stop()` joins the TM's
    /// worker through the same unsynchronized handshake as its tick.
    fn retire_traffic_manager(&mut self, why: &str) {
        let action =
            autopilot::traffic_manager_action(self.traffic_manager.is_some(), self.autopilot.len());
        if action == autopilot::TmAction::ShutDown {
            self.shut_down_traffic_manager(&format!(
                "{why}: no {} entity left",
                autopilot::BEHAVIOR
            ));
        }
    }

    /// Shut the Traffic Manager down, if there is one, under the watchdog.
    fn shut_down_traffic_manager(&mut self, why: &str) {
        let Some(tm) = self.traffic_manager.take() else {
            return;
        };
        let port = self.config.traffic_manager.port;
        match self.watched(
            "TrafficManager::ShutDown",
            why,
            budgets::TM_CALL_WATCHDOG,
            move || tm.shutdown(),
        ) {
            Ok(Ok(())) => tracing::info!("Traffic Manager on port {port} shut down ({why})"),
            Ok(Err(e)) => tracing::warn!("Traffic Manager shutdown ({why}): {e}"),
            // Logged and wedged by `watched`.
            Err(_) => {}
        }
    }

    /// Build the loaded town's lane graph if it is not built yet.
    fn ensure_road_graph(&mut self) -> Result<()> {
        if self.road_graph.is_some() {
            return Ok(());
        }
        let started = Instant::now();
        let map = self.world.map().wrap_err("get map")?;
        let mut segments = Vec::new();
        for (entry, exit) in map.topology().wrap_err("map topology")? {
            let mut points = vec![point_of(&entry.transform().location)];
            match entry.next_until_lane_end(LANE_SAMPLE_M) {
                Ok(list) => points.extend(list.iter().map(|wp| point_of(&wp.transform().location))),
                Err(e) => {
                    tracing::debug!("Lane graph: could not sample a lane ({e}); using its ends")
                }
            }
            let end = point_of(&exit.transform().location);
            if points
                .last()
                .is_none_or(|p| (p[0] - end[0]).hypot(p[1] - end[1]) > 0.1)
            {
                points.push(end);
            }
            segments.push(autopilot::Segment {
                key: lane_key(&entry),
                is_junction: entry.is_junction(),
                points,
            });
        }
        let graph = autopilot::RoadGraph::new(segments);
        if graph.is_empty() {
            eyre::bail!("the map has no lane topology");
        }
        tracing::info!(
            "Lane graph for {}: {} segments, built in {} ms",
            map.name(),
            graph.len(),
            started.elapsed().as_millis()
        );
        self.road_graph = Some((map, graph));
        Ok(())
    }

    /// Where a CARLA location lies on the lane graph (its driving lane, as `waypoint_at`
    /// projects it).
    fn locate_on_road(&self, location: &Location) -> Option<autopilot::OnRoad> {
        let (map, graph) = self.road_graph.as_ref()?;
        let waypoint = map.waypoint_at(location).ok()??;
        graph.locate(lane_key(&waypoint), &point_of(location))
    }

    /// Plan the lane route from `from` through `waypoints` (the last is the goal), and the
    /// path Traffic Manager is to follow for it.
    fn plan_autopilot_route(
        &mut self,
        from: &Location,
        waypoints: &[Location],
    ) -> Result<(autopilot::RouteProgress, Vec<autopilot::Point>)> {
        self.ensure_road_graph()?;
        let mut at = self.locate_on_road(from).ok_or_else(|| {
            eyre::eyre!(
                "the vehicle at CARLA({:.1}, {:.1}) is not on a driving lane",
                from.x,
                from.y
            )
        })?;
        let mut route: Vec<usize> = Vec::new();
        let mut path = Vec::new();
        let mut goal_s = 0.0;
        for (i, waypoint) in waypoints.iter().enumerate() {
            let to = self.locate_on_road(waypoint).ok_or_else(|| {
                eyre::eyre!(
                    "waypoint {i} at CARLA({:.1}, {:.1}) is not on a driving lane",
                    waypoint.x,
                    waypoint.y
                )
            })?;
            let graph = &self.road_graph.as_ref().expect("built above").1;
            let leg = graph.plan(at, to).ok_or_else(|| {
                eyre::eyre!(
                    "no lane route reaches waypoint {i} at CARLA({:.1}, {:.1}): the lanes do \
                     not connect (a goal on another lane of the same road would need a lane \
                     change, which is not planned)",
                    waypoint.x,
                    waypoint.y
                )
            })?;
            path.extend(graph.tm_path(&leg, point_of(waypoint)));
            let skip = usize::from(route.last() == leg.first());
            route.extend_from_slice(&leg[skip..]);
            at = to;
            goal_s = to.s;
        }
        Ok((autopilot::RouteProgress::new(route, goal_s), path))
    }

    /// Hand a vehicle to Traffic Manager, with its target speed and route.
    fn engage_autopilot(&mut self, name: &str, ap: &mut AutopilotVehicle) -> Result<()> {
        self.traffic_manager()?
            .register_vehicles(std::slice::from_ref(&ap.vehicle))
            .wrap_err("register with Traffic Manager")?;
        ap.engaged = true;
        ap.parked = false;
        // Traffic Manager launches from the hand brake with a jolt CARLA reports as up to
        // 15.5 m/s^2 for a frame, decaying to TM's steady ~5.2 m/s^2 within a second
        // (measured, Town01, tesla.model3). Like the spawn's settle, it is not driving
        // behaviour; without this SSv2's performance check ends the scenario on it.
        self.settle_frames
            .insert(ap.vehicle.id(), ENGAGE_SETTLE_FRAMES);
        self.apply_autopilot_settings(ap)?;
        tracing::info!(
            "'{name}' handed to Traffic Manager (target speed {}, {})",
            ap.target_speed
                .map_or("TM's".to_string(), |v| format!("{v:.2} m/s")),
            match &ap.route {
                Some((progress, path)) => format!(
                    "route of {} lane segment(s), {} path point(s)",
                    progress.route.len(),
                    path.len()
                ),
                None => "no goal: roaming".to_string(),
            }
        );
        Ok(())
    }

    /// Give Traffic Manager a registered vehicle's speed and, if new, its path.
    fn apply_autopilot_settings(&mut self, ap: &mut AutopilotVehicle) -> Result<()> {
        let tm = self.traffic_manager()?;
        match ap.target_speed {
            Some(v) => tm
                .set_desired_speed(&ap.vehicle, autopilot::mps_to_kmh(v))
                .wrap_err("set desired speed")?,
            // Back to TM's own choice, the speed limit (this clears a desired speed).
            None => tm
                .set_percentage_speed_difference(&ap.vehicle, 0.0)
                .wrap_err("reset speed")?,
        }
        if ap.path_pending {
            match &ap.route {
                Some((_, path)) => {
                    let points: Vec<PathPoint> =
                        path.iter().map(|p| PathPoint(location_of(p))).collect();
                    // A lane change would leave the planned lanes.
                    tm.set_auto_lane_change(&ap.vehicle, false)
                        .wrap_err("disable lane changes")?;
                    tm.set_custom_path(&ap.vehicle, &points, true)
                        .wrap_err("set path")?;
                }
                None => {
                    tm.remove_upload_path(ap.vehicle.id(), true)
                        .wrap_err("remove path")?;
                    tm.set_auto_lane_change(&ap.vehicle, true)
                        .wrap_err("enable lane changes")?;
                }
            }
            ap.path_pending = false;
        }
        ap.approach_cruise = None;
        Ok(())
    }

    /// Stop a vehicle at its goal: Traffic Manager lets go (it would roam on past the end
    /// of its path), and the brakes hold it until a new goal.
    fn park_autopilot(&mut self, ap: &mut AutopilotVehicle) -> Result<()> {
        if let Some(tm) = self.traffic_manager.as_mut() {
            tm.unregister_vehicles(std::slice::from_ref(&ap.vehicle))
                .wrap_err("release from Traffic Manager")?;
        }
        ap.vehicle
            .apply_control(&PARKED_CONTROL)
            .wrap_err("brake")?;
        ap.parked = true;
        Ok(())
    }

    /// One frame of a simulator-driven vehicle: hand it to Traffic Manager when SSv2's NPC
    /// logic starts, slow it into its goal, park it there.
    fn drive_autopilot(&mut self, name: &str, npc_logic_started: bool) -> Result<()> {
        let Some(mut ap) = self.autopilot.remove(name) else {
            return Ok(());
        };
        let outcome = self.drive_autopilot_vehicle(name, &mut ap, npc_logic_started);
        self.autopilot.insert(name.to_string(), ap);
        outcome
    }

    fn drive_autopilot_vehicle(
        &mut self,
        name: &str,
        ap: &mut AutopilotVehicle,
        npc_logic_started: bool,
    ) -> Result<()> {
        if !ap.engaged {
            if npc_logic_started {
                self.engage_autopilot(name, ap)?;
            }
            return Ok(());
        }
        if ap.parked {
            return Ok(());
        }
        let location = ap.vehicle.location().wrap_err("read location")?;
        let at = self.locate_on_road(&location);
        let remaining = ap
            .route
            .as_mut()
            .and_then(|(progress, _)| progress.observe(at));
        tracing::debug!(
            "'{name}' at CARLA({:.1}, {:.1}) on {at:?}, route {:?}, remaining {remaining:?}",
            location.x,
            location.y,
            ap.route
                .as_ref()
                .map(|(p, _)| (&p.route, p.reached, p.goal_s))
        );
        let Some(remaining) = remaining else {
            return Ok(());
        };
        let speed = ap
            .vehicle
            .velocity()
            .map(|v| ((v.x * v.x + v.y * v.y) as f64).sqrt())
            .unwrap_or(0.0);
        if remaining <= autopilot::ARRIVE_DISTANCE {
            // Let Traffic Manager bring it to rest first: setting the hand brake at the
            // approach's 1 m/s stops it in one frame (measured -8 m/s^2 reported), more
            // than a scenario's declared deceleration may allow.
            if speed > autopilot::PARK_SPEED {
                self.traffic_manager()?
                    .set_desired_speed(&ap.vehicle, 0.0)
                    .wrap_err("stop at the goal")?;
                return Ok(());
            }
            self.park_autopilot(ap)?;
            tracing::info!(
                "'{name}' reached its goal ({:.2} m {} it along the lane); parked until a new goal",
                remaining.abs(),
                if remaining >= 0.0 { "short of" } else { "past" }
            );
            return Ok(());
        }
        let reference = ap
            .approach_cruise
            .or(ap.target_speed)
            .unwrap_or(speed)
            .max(1.0);
        if remaining < autopilot::approach_window(reference) {
            let cruise = *ap.approach_cruise.get_or_insert(reference);
            let v = autopilot::approach_speed(remaining, cruise);
            self.traffic_manager()?
                .set_desired_speed(&ap.vehicle, autopilot::mps_to_kmh(v))
                .wrap_err("set approach speed")?;
        }
        Ok(())
    }

    /// Release every simulator-driven vehicle from Traffic Manager (before their actors are
    /// destroyed) and forget them.
    fn release_autopilot_vehicles(&mut self) {
        let vehicles: Vec<AutopilotVehicle> = self.autopilot.drain().map(|(_, ap)| ap).collect();
        if self.carla_unreachable {
            return;
        }
        let registered: Vec<carla::client::Vehicle> = vehicles
            .into_iter()
            .filter(|ap| ap.engaged && !ap.parked)
            .map(|ap| ap.vehicle)
            .collect();
        if registered.is_empty() {
            return;
        }
        if let Some(tm) = self.traffic_manager.as_mut() {
            if let Err(e) = tm.unregister_vehicles(&registered) {
                tracing::warn!(
                    "Could not release {} vehicle(s) from Traffic Manager: {e}",
                    registered.len()
                );
            }
        }
    }

    /// The world changed (another town, or a new connection): the lane graph is stale.
    fn forget_road_graph(&mut self) {
        self.road_graph = None;
    }

    /// Point a simulator-driven entity at a new goal, route or target speed
    /// (`UpdateEntityGoal`, sent by the SSv2 fork for `simulator_autopilot` entities).
    pub fn update_entity_goal(&mut self, name: &str, req: &Goal) -> Result<(), BackendError> {
        let name = name.to_string();
        if self.agent_vehicles.contains(&name) {
            return self
                .update_agent_goal(&name, req)
                .or_else(|e| failed(format!("UpdateEntityGoal for '{name}': {e:#}")));
        }
        let Some(mut ap) = self.autopilot.remove(&name) else {
            let why = if self.entities.get(&name).is_some() {
                format!(
                    "'{name}' is not driven by the simulator (its controller is neither {} \
                     nor {})",
                    autopilot::BEHAVIOR,
                    agent_link::BEHAVIOR
                )
            } else {
                format!("Entity '{name}' not found")
            };
            return failed(why);
        };
        let outcome = self.update_autopilot_goal(&name, &mut ap, req);
        self.autopilot.insert(name.clone(), ap);
        outcome.or_else(|e| failed(format!("UpdateEntityGoal for '{name}': {e:#}")))
    }

    /// An `agent` entity's goal, route or target speed goes to its agent (queued; sent once
    /// SSv2's NPC logic runs and the agent can take it). Fails at once, naming the entity,
    /// when no agent is registered for it -- SSv2 then ends the scenario.
    fn update_agent_goal(&mut self, name: &str, req: &Goal) -> Result<()> {
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

    fn update_autopilot_goal(
        &mut self,
        name: &str,
        ap: &mut AutopilotVehicle,
        req: &Goal,
    ) -> Result<()> {
        if self.carla_unreachable {
            eyre::bail!("CARLA unreachable");
        }
        let origin_offset = self
            .entities
            .get(name)
            .map(|e| e.origin_offset)
            .unwrap_or_default();
        let mut changed = false;
        if let Some(target_speed) = req.target_speed {
            if !(target_speed.is_finite() && target_speed >= 0.0) {
                eyre::bail!("target speed {target_speed} m/s is not a speed");
            }
            ap.target_speed = Some(target_speed);
            changed = true;
            tracing::info!("'{name}': target speed {target_speed:.2} m/s");
        }
        if !req.waypoints.is_empty() {
            // Where the CARLA actor's origin is when the entity stands on each waypoint.
            let waypoints: Vec<Location> = req
                .waypoints
                .iter()
                .map(|pose| {
                    ros_pose_to_carla_transform(pose, origin_offset, self.map_origin).location
                })
                .collect();
            let from = ap.vehicle.location().wrap_err("read location")?;
            let (progress, path) = self.plan_autopilot_route(&from, &waypoints)?;
            let goal = waypoints.last().expect("non-empty");
            tracing::info!(
                "'{name}': goal CARLA({:.1}, {:.1}) via {} waypoint(s): {} lane segment(s), TM \
                 path of {} point(s)",
                goal.x,
                goal.y,
                waypoints.len() - 1,
                progress.route.len(),
                path.len()
            );
            ap.route = Some((progress, path));
            ap.path_pending = true;
            changed = true;
        } else if req.clear {
            ap.route = None;
            ap.path_pending = true;
            changed = true;
            tracing::info!("'{name}': route cleared; roaming");
        }
        // Not yet handed over: kept for when SSv2's NPC logic starts.
        if changed && ap.engaged {
            if ap.parked {
                self.engage_autopilot(name, ap)?;
            } else {
                self.apply_autopilot_settings(ap)?;
            }
        }
        Ok(())
    }

    /// `Attach*Sensor`: refused. CARLA's sensors are acb's, published in the ego's domain.
    pub fn attach_sensor(&mut self, _sensor: Sensor) -> Result<(), BackendError> {
        Err(BackendError::Unsupported(SENSORS_ELSEWHERE.into()))
    }

    pub fn update_traffic_lights(&mut self, states: &[TrafficSignal]) -> Result<(), BackendError> {
        let mut applied = 0usize;
        let mut newly_unmapped = Vec::new();
        let mut failures = Vec::new();

        for signal in states {
            // Report the arrow limitation once: CARLA has no arrow states, so a protected
            // turn cannot be represented and the ego sees a plain colour.
            if !self.warned_arrow_shapes && signal_uses_arrows(signal) {
                self.warned_arrow_shapes = true;
                tracing::warn!(
                    "Signal {} uses arrow shapes, which CARLA cannot represent. Arrows are \
                     flattened to their colour, so protected-turn phases will not be faithful.",
                    signal.id
                );
            }

            let state = carla_state_for_signal(signal);

            let Some(opendrive_id) = self.signal_map.opendrive_id(signal.id).map(str::to_owned)
            else {
                // Warn once per signal: SSv2 sends states every frame.
                if self.warned_unmapped_signals.insert(signal.id) {
                    newly_unmapped.push(signal.id);
                }
                continue;
            };

            match self.set_signal_state(&opendrive_id, state.clone()) {
                Ok(()) => {
                    applied += 1;
                    self.commanded_lights.insert(opendrive_id, state);
                }
                Err(e) => failures.push(format!("signal {} ({opendrive_id}): {e}", signal.id)),
            }
        }

        if !newly_unmapped.is_empty() {
            tracing::warn!(
                "No CARLA traffic light mapped for lanelet signal(s) {newly_unmapped:?}. Add \
                 them to config/traffic_lights_<town>.yaml; these signals are not being driven."
            );
        }

        // An unmapped signal is a configuration gap, not a request failure -- failing here
        // would abort every scenario on a map that has no mapping file yet. A signal that
        // *is* mapped but could not be written is a real failure.
        if failures.is_empty() {
            tracing::debug!("Applied {applied} traffic light state(s)");
            Ok(())
        } else {
            failed(format!(
                "Failed to apply {} of {} traffic light state(s): {}",
                failures.len(),
                states.len(),
                failures.join("; ")
            ))
        }
    }

    /// Write one signal state to the CARLA light with the given OpenDRIVE sign ID.
    fn set_signal_state(
        &mut self,
        opendrive_id: &str,
        state: carla::rpc::TrafficLightState,
    ) -> Result<()> {
        let actor = self
            .world
            .traffic_light_from_open_drive(opendrive_id)
            .map_err(|e| eyre::eyre!("look up OpenDRIVE signal '{opendrive_id}': {e}"))?
            .ok_or_else(|| {
                eyre::eyre!("no CARLA traffic light with OpenDRIVE id '{opendrive_id}'")
            })?;

        let light = match actor.into_kinds() {
            carla::client::ActorKind::TrafficLight(light) => light,
            _ => {
                eyre::bail!("CARLA actor for OpenDRIVE id '{opendrive_id}' is not a traffic light")
            }
        };

        light
            .set_state(state)
            .map_err(|e| eyre::eyre!("set state on '{opendrive_id}': {e}"))
    }

    /// Restore async mode so CARLA isn't stuck waiting for ticks on exit.
    ///
    /// No-op if we never enabled sync mode -- CARLA may be shared with other clients and
    /// we should not rewrite settings we did not set.
    pub fn restore_async_mode(&mut self) {
        if !self.sync_mode_enabled {
            tracing::info!("Sync mode was never enabled; leaving CARLA settings untouched");
            return;
        }
        if self.wedged.is_some() {
            // A call never returned; nothing more goes to CARLA (034 step 2).
            return;
        }

        match self.world.settings() {
            Ok(mut settings) => {
                settings.synchronous_mode = false;
                settings.fixed_delta_seconds = None;
                // Hand back CARLA's own physics substepping too; the short substeps are
                // this bridge's choice for its scenario, not the shared server's.
                settings.max_substep_delta_time = CARLA_DEFAULT_MAX_SUBSTEP_DELTA;
                settings.max_substeps = CARLA_DEFAULT_MAX_SUBSTEPS;
                if let Err(e) = self.apply_settings_watched("the restore of async mode", settings) {
                    tracing::warn!("Failed to restore async mode: {e}");
                } else {
                    self.sync_mode_enabled = false;
                    if let Err(e) = self.set_traffic_manager_synchronous(false) {
                        tracing::warn!("Traffic Manager asynchronous mode: {e}");
                    }
                    tracing::info!("Restored CARLA to async mode");
                }
            }
            Err(e) => tracing::warn!("Failed to get settings for cleanup: {e}"),
        }
    }

    // --- Private helpers ---

    /// Origin z for a walker teleport to `commanded`, or `None` if `actor_id` is no walker.
    ///
    /// Ignores the commanded z (see [`walker_origin_z`]) and uses the road under the
    /// commanded x/y, looked up again only once the walker has moved
    /// [`WALKER_GROUND_REFRESH_M`] from the last lookup.
    fn walker_teleport_z(&mut self, actor_id: u32, commanded: &Location) -> Option<f32> {
        let height = *self.walker_height.get(&actor_id)?;
        let last = height.ground.map(|(x, y, _)| (x, y));
        let ground_z = if walker_ground_is_stale(last, commanded.x, commanded.y) {
            let z = self.ground_height_at(commanded);
            if let Some(entry) = self.walker_height.get_mut(&actor_id) {
                entry.ground = Some((commanded.x, commanded.y, z));
            }
            z
        } else {
            height.ground.and_then(|(_, _, z)| z)
        };
        Some(walker_origin_z(ground_z, commanded.z, height.lift))
    }

    /// Road height under `location`, from the nearest driving-lane waypoint; `None` when
    /// there is no lane nearby or CARLA will not answer.
    fn ground_height_at(&self, location: &Location) -> Option<f32> {
        let map = self.world.map().ok()?;
        match map.waypoint_at(location) {
            Ok(Some(waypoint)) => Some(waypoint.transform().location.z),
            _ => None,
        }
    }

    /// Height to spawn at for a commanded location.
    ///
    /// Takes the road height from the nearest driving-lane waypoint. Only the height is
    /// used -- the commanded x/y is SSv2's to decide, and snapping the whole pose to lane
    /// centre would silently move the vehicle off the scenario's mark.
    ///
    /// Falls back to the commanded height when there is no lane nearby, or when CARLA will
    /// not answer.
    fn resolve_spawn_height(&self, commanded: &Location) -> f32 {
        let map = match self.world.map() {
            Ok(m) => m,
            Err(e) => {
                let z = fallback_spawn_height(commanded.z);
                tracing::warn!(
                    "Could not read the map to resolve ground height ({e}); using z={z:.2}"
                );
                return z;
            }
        };

        match map.waypoint_at(commanded) {
            Ok(Some(waypoint)) => {
                let road_z = waypoint.transform().location.z;
                let z = spawn_height_above_ground(road_z);
                tracing::debug!(
                    "Road under ({:.1}, {:.1}) is z={road_z:.2}; spawning at z={z:.2}",
                    commanded.x,
                    commanded.y
                );
                z
            }
            Ok(None) => {
                let z = fallback_spawn_height(commanded.z);
                tracing::warn!(
                    "No driving lane near ({:.1}, {:.1}); spawning at z={z:.2}",
                    commanded.x,
                    commanded.y
                );
                z
            }
            Err(e) => {
                let z = fallback_spawn_height(commanded.z);
                tracing::warn!("Waypoint lookup failed ({e}); spawning at z={z:.2}");
                z
            }
        }
    }

    /// A tracked vehicle's steering geometry: the wheelbase from its declared axles and the
    /// lock of CARLA's front wheel (`max_steer_angle`, the same field on every CARLA
    /// version, unlike the wheel's position). `None` for walkers or when either is missing.
    fn npc_steering(&self, actor: &carla::client::Actor, shape: &EntityShape) -> Option<Steering> {
        let (front, rear) = shape.axles?;
        let carla::client::ActorKind::Vehicle(vehicle) = actor.clone().into_kinds() else {
            return None;
        };
        let lock = vehicle
            .physics_control()
            .ok()?
            .wheels
            .first()
            .map(|w| f64::from(w.max_steer_angle))?;
        npc_tracking::steering(front, rear, lock)
    }

    /// The blueprint closest to the entity's declared box among its class (roadmap 022), or
    /// `None` when the kind is not size-matched, nothing was declared, or there is no table.
    fn size_matched_blueprint(
        &self,
        name: &str,
        kind: SpawnKind,
        shape: &EntityShape,
    ) -> Option<blueprint_boxes::SizeChoice> {
        let class = kind.size_class(shape.subtype)?;
        let declared = shape.declared?;
        let table = self.blueprint_boxes.as_ref()?;
        let default = kind.default_blueprint();
        let chosen = blueprint_boxes::choose_by_size(class, &declared, table, default)?;
        if chosen.blueprint != default {
            tracing::info!(
                "'{name}' ({:?}, declared {}) gets '{}' by size; '{default}' does not fit",
                shape.subtype,
                format_dims(&declared),
                chosen.blueprint
            );
        }
        Some(chosen)
    }

    /// The spawned body against the table and against the declared box: a stale table is
    /// warned about once per blueprint; the residual is logged (WARN outside tolerance)
    /// and goes into the episode's contact reports.
    fn compare_geometry(
        &mut self,
        name: &str,
        blueprint: &str,
        shape: &EntityShape,
        actual: Option<Dimensions>,
    ) {
        let table = self
            .blueprint_boxes
            .as_ref()
            .and_then(|t| t.get(blueprint))
            .map(|entry| entry.dimensions());
        // A prop at a yaw its box cannot be read back at: the table's box stands in.
        let Some(actual) = actual.or(table) else {
            tracing::debug!("'{name}': the box of '{blueprint}' cannot be read at this yaw");
            return;
        };
        let actual = &actual;
        if let Some(table) = table {
            if table.differs_from(actual) && self.warned_stale_boxes.insert(blueprint.to_string()) {
                tracing::warn!(
                    "Stale blueprint box table: '{blueprint}' is {} in CARLA, {} in the table; \
                     rerun {}",
                    format_dims(actual),
                    format_dims(&table),
                    blueprint_boxes::MEASURE_SCRIPT
                );
            }
        }
        let Some(declared) = shape.declared else {
            tracing::debug!(
                "'{name}' declares no box; its '{blueprint}' is {}",
                format_dims(actual)
            );
            return;
        };
        // In the entity's frame: a turned prop's length lies along the declared width.
        let turned = shape.origin_offset.yaw != 0.0;
        let actual = &if turned { actual.turned() } else { *actual };
        let r = declared.residual(actual);
        let fits = declared.fits(actual);
        let line = format!(
            "'{name}': declared {}, '{blueprint}'{} is {} (difference {:+.2} / {:+.2} / {:+.2} m)",
            format_dims(&declared),
            if turned { " turned 90°" } else { "" },
            format_dims(actual),
            r.length,
            r.width,
            r.height
        );
        if fits {
            tracing::info!("Body {line}");
        } else {
            tracing::warn!(
                "Body outside tolerance ({} m in length and width, {} m in height): {line}",
                blueprint_boxes::PLANE_TOLERANCE,
                blueprint_boxes::HEIGHT_TOLERANCE
            );
        }
        self.collision_monitor.record_geometry(ReportGeometry {
            name: name.to_string(),
            blueprint: blueprint.to_string(),
            declared: [declared.length, declared.width, declared.height],
            actual: [actual.length, actual.width, actual.height],
            fits,
        });
    }

    /// Pick a blueprint that CARLA will actually accept, falling back when the requested
    /// one is unknown. Resolved once per spawn, before any retry.
    ///
    /// The decision itself is [`choose_blueprint`]; this only performs the probes, because
    /// `actor_builder` needs a live server and the policy is worth testing without one.
    fn resolve_blueprint_key(
        &mut self,
        requested: &str,
        fallback: &str,
        kind: SpawnKind,
    ) -> Result<String> {
        // The probe builder is dropped at the end of this statement, releasing the borrow
        // on `world` before the next one is taken.
        let requested_exists = self.world.actor_builder(requested).is_ok();

        // Probing the fallback is a server round trip, so it only happens when the answer
        // can matter -- hence `None` rather than a fabricated `false`.
        let (fallback_exists, fallback_error) = if requested_exists {
            (None, None)
        } else {
            match self.world.actor_builder(fallback) {
                Ok(_) => (Some(true), None),
                Err(e) => (Some(false), Some(e.to_string())),
            }
        };

        match choose_blueprint(requested_exists, fallback_exists) {
            BlueprintChoice::Requested => Ok(requested.to_string()),
            BlueprintChoice::Fallback => {
                // SSv2 asset keys are not CARLA blueprint names in general. Where they
                // happen to coincide the branch above passes them straight through; where
                // they do not, the fallback (the size-matched body, else the kind's
                // default) keeps the scenario running with a warning naming both. The
                // operator's translation is `blueprint_map`.
                tracing::warn!(
                    "Blueprint '{requested}' is not a CARLA blueprint; falling back to \
                     '{fallback}' for this {}",
                    kind.label()
                );
                Ok(fallback.to_string())
            }
            BlueprintChoice::Neither => {
                let detail = fallback_error.unwrap_or_else(|| "unknown blueprint".to_string());
                eyre::bail!("neither '{requested}' nor '{fallback}' is a valid blueprint: {detail}")
            }
        }
    }

    /// Build a configured actor builder for an already-resolved blueprint.
    ///
    /// `ActorBuilder::spawn` consumes the builder, so a retry needs a fresh one each time.
    fn build_actor(
        &mut self,
        blueprint_key: &str,
        role_name: Option<&str>,
        entity_role: &str,
    ) -> Result<carla::client::ActorBuilder<'_>> {
        // role_name is how acb_bridge finds the vehicle it is meant to serve. Everything else
        // gets the entity mark where the blueprint has the attribute; a blueprint without
        // it spawns unmarked -- it cannot be reaped after a crash, which is the state every
        // entity was in before, not a reason to refuse the spawn.
        let (role, required) = match role_name {
            Some(role) => (role, true),
            None => (entity_role, false),
        };
        let takes_role = required
            || self
                .world
                .actor_builder(blueprint_key)
                .map_err(|e| eyre::eyre!("build '{blueprint_key}': {e}"))?
                .set_attribute("role_name", role)
                .is_ok();
        let builder = self
            .world
            .actor_builder(blueprint_key)
            .map_err(|e| eyre::eyre!("build '{blueprint_key}': {e}"))?;
        if !takes_role {
            tracing::debug!("'{blueprint_key}' takes no role_name; spawning unmarked");
            return Ok(builder);
        }
        builder
            .set_attribute("role_name", role)
            .map_err(|e| eyre::eyre!("set role_name: {e}"))
    }

    fn read_actor_state(
        &mut self,
        actor_id: u32,
        origin_offset: OriginOffset,
    ) -> Option<(Pose, traffic_simulator_msgs::ActionStatus)> {
        let actors = self.world.actors().ok()?;
        let actor = actors.find(actor_id).ok()??;

        let t = actor.transform().ok()?;
        let v = actor.velocity().ok()?;
        let av = actor.angular_velocity().ok()?;
        let acc = actor.acceleration().ok()?;

        // The entity's heading: the actor's, less any spawn-time turn (OriginOffset::yaw).
        let entity_yaw = t.rotation.yaw + origin_offset.yaw.to_degrees() as f32;
        let mut pose = coordinate_conversion::carla_to_ros_pose(
            t.location.x,
            t.location.y,
            t.location.z,
            t.rotation.roll,
            t.rotation.pitch,
            entity_yaw,
        );
        // Into the map frame SSv2 works in.
        if let Some(position) = pose.position.as_mut() {
            (position.x, position.y, position.z) = self.map_origin.carla_to_map(
                t.location.x as f64,
                t.location.y as f64,
                t.location.z as f64,
            );
        }

        // CARLA reports the actor origin (body centre); SSv2 expects the entity origin its
        // scenario declared (the rear axle for vehicles). Undo the spawn-time shift.
        //
        // Twist and accel are left as CARLA reports them at the body centre. Longitudinal
        // speed is the same at every point of a rigid body, but lateral velocity at the rear
        // axle differs by yaw_rate × offset.x; that is deliberately not corrected yet.
        if let Some(position) = pose.position.as_mut() {
            let ros_yaw = -(entity_yaw as f64).to_radians();
            let (x, y) = coordinate_conversion::actor_origin_to_entity_origin(
                position.x,
                position.y,
                ros_yaw,
                origin_offset,
            );
            position.x = x;
            position.y = y;
        }

        let (vx, vy, vz) = coordinate_conversion::carla_to_ros_velocity(v.x, v.y, v.z);
        let (wx, wy, wz) = coordinate_conversion::carla_to_ros_angular_velocity(av.x, av.y, av.z);
        let (ax, ay, az) = coordinate_conversion::carla_to_ros_acceleration(acc.x, acc.y, acc.z);

        // Smooth the reported acceleration (see ACCEL_SMOOTHING_FRAMES).
        let hist = self.accel_history.entry(actor_id).or_default();
        hist.push_back((ax, ay, az));
        if hist.len() > ACCEL_SMOOTHING_FRAMES {
            hist.pop_front();
        }
        let (ax, ay, az) = (
            median_of(hist.iter().map(|(x, _, _)| *x)),
            median_of(hist.iter().map(|(_, y, _)| *y)),
            median_of(hist.iter().map(|(_, _, z)| *z)),
        );

        // Longitudinal acceleration, then its rate of change. CARLA reports an acceleration
        // vector but no jerk, so it is differenced across frames -- see
        // `longitudinal_acceleration` for why the heading projection is used.
        // `ax`/`ay` are ROS-frame here, so the heading must be too: CARLA's yaw negated.
        // (CARLA's yaw was passed before, which flipped the sign on any road not aligned
        // with x -- a vehicle speeding up northbound read as braking at -15 m/s^2.)
        let yaw_rad = -(t.rotation.yaw as f64).to_radians();
        let longitudinal = longitudinal_acceleration(ax, ay, yaw_rad);
        let linear_jerk = self.differentiate_acceleration(actor_id, longitudinal);

        // Suppress the suspension-settle impact after spawn (see SETTLE_FRAMES).
        let settling = matches!(self.settle_frames.get_mut(&actor_id), Some(n) if *n > 0);
        if !settling && !(-5.0..=3.0).contains(&longitudinal) {
            tracing::warn!(
                "Actor {actor_id} acceleration outside SSv2 bounds: longitudinal={longitudinal:.2} \
                 carla=({:.2},{:.2},{:.2}) ros=({ax:.2},{ay:.2},{az:.2}) speed={:.2} z={:.2}",
                acc.x, acc.y, acc.z,
                (v.x * v.x + v.y * v.y).sqrt(),
                t.location.z,
            );
        }
        let (ax, ay, az, linear_jerk) = match self.settle_frames.get_mut(&actor_id) {
            Some(n) if *n > 0 => {
                *n -= 1;
                (0.0, 0.0, 0.0, 0.0)
            }
            _ => (ax, ay, az, linear_jerk),
        };

        // SSv2's twist and accel are in the entity's own frame (linear.x = forward), as its
        // behaviors write them and SpeedCondition reads them; CARLA's are world-frame.
        // Reporting world-frame values made a vehicle driving along -x read as reversing,
        // and one driving along y as standing still (measured: SpeedCondition < 0.05 m/s
        // held for a simulator_autopilot vehicle moving at 2.8 m/s northbound).
        let (vx, vy) = world_to_body(vx, vy, yaw_rad);
        let (wx, wy) = world_to_body(wx, wy, yaw_rad);
        let (ax, ay) = world_to_body(ax, ay, yaw_rad);

        let action_status = traffic_simulator_msgs::ActionStatus {
            // Left empty deliberately. `current_action` names the behaviour-plugin action
            // driving an entity, and SSv2 owns that for the NPCs it puppeteers -- their
            // status is echoed back untouched. The ego has no SSv2 behaviour plugin: it is
            // driven by Autoware, whose action has no equivalent in this field.
            current_action: String::new(),
            twist: Some(geometry_msgs::Twist {
                linear: Some(geometry_msgs::Vector3 {
                    x: vx,
                    y: vy,
                    z: vz,
                }),
                angular: Some(geometry_msgs::Vector3 {
                    x: wx,
                    y: wy,
                    z: wz,
                }),
            }),
            accel: Some(geometry_msgs::Accel {
                linear: Some(geometry_msgs::Vector3 {
                    x: ax,
                    y: ay,
                    z: az,
                }),
                // Angular acceleration stays zero: CARLA exposes angular velocity but not
                // its derivative, and differencing it frame to frame is too noisy at a 20 Hz
                // step to be worth reporting as truth. Known limitation.
                angular: Some(geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }),
            }),
            linear_jerk,
        };

        Some((pose, action_status))
    }

    /// The world snapshot, at least as new as the last tick when it gets there within
    /// `npc_tracking::SNAPSHOT_FRESH_WAIT` (roadmap 023, "a stale snapshot fails the
    /// frame"). `world.snapshot()` is the client's cached copy of the last tick CARLA
    /// streamed, not a request: right after a spawn, and most after a `load_world` while the
    /// client's stream re-subscribes, it can predate the frame `world.tick()` returned.
    /// Past the bound the stale copy is returned with a warning; a missing actor in it is
    /// the caller's to absorb.
    fn fresh_snapshot(&mut self) -> Result<carla::client::WorldSnapshot> {
        let started = Instant::now();
        let world = &self.world;
        let read = npc_tracking::read_fresh(
            self.last_tick_frame,
            npc_tracking::SNAPSHOT_FRESH_WAIT,
            npc_tracking::SNAPSHOT_REREAD_PAUSE,
            || world.snapshot(),
            |s| s.frame() as u64,
        )
        .wrap_err("world snapshot")?;
        if read.reads > 1 || !read.fresh {
            self.stale_snapshots += 1;
            // Every one while few, then one in a hundred: a lagging stream is an event,
            // a persistently lagging one should not flood the log.
            let n = self.stale_snapshots;
            if n <= 10 || n.is_multiple_of(100) {
                let tick = self.last_tick_frame.unwrap_or_default();
                let waited = started.elapsed().as_secs_f64() * 1e3;
                if read.fresh {
                    tracing::info!(
                        "Stale world snapshot (frame {} < tick {tick}): fresh after \
                         {waited:.1} ms, {} reads ({n} this episode)",
                        read.frame,
                        read.reads
                    );
                } else {
                    tracing::warn!(
                        "Stale world snapshot (frame {} < tick {tick}) still stale after \
                         {waited:.1} ms, {} reads: used as is ({n} this episode)",
                        read.frame,
                        read.reads
                    );
                }
            }
        }
        Ok(read.snapshot)
    }

    /// The smallest box-to-box clearance in the plane between `actor_id`'s body and every
    /// other physics body csb spawned, from `snapshot` (roadmap 032 N2). `None` when its own
    /// box or pose is unknown, or no other body is in the snapshot.
    fn clearance_to_bodies(
        &self,
        actor_id: u32,
        snapshot: &carla::client::WorldSnapshot,
    ) -> Option<f64> {
        let place = |id: u32, b: &BodyBox| {
            let t = snapshot.find(id)?.transform();
            Some(b.at(
                f64::from(t.location.x),
                f64::from(t.location.y),
                f64::from(t.rotation.yaw),
            ))
        };
        let subject = place(actor_id, self.body_boxes.get(&actor_id)?)?;
        body_clearance::min_clearance(
            &subject,
            self.body_boxes
                .iter()
                .filter(|(id, _)| **id != actor_id)
                .filter_map(|(id, b)| place(*id, b)),
        )
    }

    /// One frame of a tracked NPC (roadmap 023): measure it against SSv2's pose, record the
    /// error, and command its velocity (or snap it back). `status` is `None` while SSv2
    /// holds the NPC still: zero velocity and yaw rate.
    ///
    /// Returns `false` when the NPC was not in the world snapshot: skipped for this frame,
    /// no command sent (CARLA keeps the last one), counted as missed; the caller decides
    /// with `npc_tracking::MissWatch` whether the frame fails.
    #[allow(clippy::too_many_arguments)]
    fn track_npc(
        &mut self,
        name: &str,
        actor_id: u32,
        npc: TrackedNpc,
        pose: &Pose,
        status: Option<&traffic_simulator_msgs::ActionStatus>,
        origin_offset: OriginOffset,
        view: &mut Option<TrackedView>,
    ) -> Result<bool> {
        if view.is_none() {
            let ids: Vec<u32> = self.tracked_npcs.keys().copied().collect();
            let snapshot = self.fresh_snapshot()?;
            *view = Some(TrackedView {
                snapshot,
                actors: self.world.actors_by_ids(&ids).wrap_err("tracked actors")?,
            });
        }
        let view = view.as_ref().expect("filled above");

        // SSv2's pose and twist as the CARLA body's target (npc_tracking::world_twist).
        let target_tf = ros_pose_to_carla_transform(pose, origin_offset, self.map_origin);
        let Some(body) = view.snapshot.find(actor_id) else {
            // Not in the snapshot: skip this NPC for the frame, not the frame. Its heading
            // is still taken, so next frame's pose yaw rate spans one frame, not two.
            if let Some(entry) = self.tracked_npcs.get_mut(&actor_id) {
                entry.last_yaw_deg = Some(f64::from(target_tf.rotation.yaw));
            }
            self.collision_monitor.record_tracking_missed(name);
            if self.miss_watch.missed(name) {
                tracing::warn!(
                    "Tracking '{name}': actor {actor_id} is not in the world snapshot \
                     (frame {}); skipped for the frame, CARLA keeps its last command \
                     (warned once per episode, counted as missed)",
                    view.snapshot.frame()
                );
            }
            return Ok(false);
        };
        let now = body.transform();
        let ros_yaw = pose.orientation.as_ref().map_or(0.0, |q| {
            coordinate_conversion::quaternion_to_euler(q.x, q.y, q.z, q.w).2
        });
        let twist = status.and_then(|a| a.twist.as_ref());
        let linear = twist.and_then(|t| t.linear.as_ref());
        let reported_yaw_rate = twist.and_then(|t| t.angular.as_ref()).map_or(0.0, |a| a.z);
        // SSv2's NPC twist carries no angular velocity (0 through a corner): the yaw rate
        // comes from its poses, in CARLA's sense (degrees, clockwise), and from there back
        // to ROS's for the body's ω × r.
        let target_yaw = f64::from(target_tf.rotation.yaw);
        let yaw_rate_deg = if status.is_some() {
            npc_tracking::pose_yaw_rate(
                npc.last_yaw_deg,
                target_yaw,
                self.step_time,
                -reported_yaw_rate.to_degrees(),
            )
        } else {
            0.0
        };
        if let Some(entry) = self.tracked_npcs.get_mut(&actor_id) {
            entry.last_yaw_deg = Some(target_yaw);
        }
        let (vx, vy, yaw_rate_deg) = npc_tracking::world_twist(
            ros_yaw,
            linear.map_or(0.0, |l| l.x),
            linear.map_or(0.0, |l| l.y),
            -yaw_rate_deg.to_radians(),
            origin_offset.x,
            origin_offset.y,
        );
        let target = npc_tracking::Target {
            x: f64::from(target_tf.location.x),
            y: f64::from(target_tf.location.y),
            yaw_deg: target_yaw,
            vx,
            vy,
            yaw_rate_deg,
        };
        let current = npc_tracking::Current {
            x: f64::from(now.location.x),
            y: f64::from(now.location.y),
            yaw_deg: f64::from(now.rotation.yaw),
        };
        let dt = self.step_time;
        let gains = if npc.walker {
            npc_tracking::WALKER_GAINS
        } else {
            npc_tracking::VEHICLE_GAINS
        };
        // SSv2's pose for a blind NPC may run through the ego: no snapping while touching it.
        let in_contact = self
            .collision_monitor
            .touched_within(actor_id, npc_tracking::CONTACT_HOLD_FRAMES);
        let mut hold = in_contact;
        let mut command = npc_tracking::command(&target, &current, dt, gains, npc.steering, !hold);
        // Nor next to the ego or another physics body, contact seen or not (roadmap 032 N2):
        // a snap -- or the kinematic hand-over a blocked snap becomes -- would put it into
        // that body. Measured only when it would snap: the hold changes nothing else.
        if !hold && matches!(command, npc_tracking::Command::Snap { .. }) {
            let clearance = self.clearance_to_bodies(actor_id, &view.snapshot);
            if npc_tracking::held(false, clearance) {
                hold = true;
                command = npc_tracking::command(&target, &current, dt, gains, npc.steering, false);
                tracing::debug!(
                    "Tracking '{name}': {:.3} m from another body, held (no snap)",
                    clearance.unwrap_or_default()
                );
            }
        }
        // Held by something other than the ego (roadmap 032, gap A): a snap would teleport
        // it into that, and PhysX would throw it out (bench npc_05, 80 m up).
        let body_v = body.velocity();
        let body_planar = (f64::from(body_v.x), f64::from(body_v.y));
        let is_blocked = npc_tracking::blocked(
            npc.blocked,
            npc.last_command,
            (target.vx, target.vy),
            body_planar,
            hold,
        );
        if let Some(entry) = self.tracked_npcs.get_mut(&actor_id) {
            entry.blocked = is_blocked;
            entry.held = hold;
        }
        let action = npc_tracking::act(command, is_blocked);

        // The error the law acts on: against the start-of-step pose, before this step.
        let error = (target.x - vx * dt - current.x).hypot(target.y - vy * dt - current.y);
        let yaw_error = (!npc.walker)
            .then(|| npc_tracking::wrap_deg(target.yaw_deg - yaw_rate_deg * dt - current.yaw_deg));
        let snapped = matches!(
            action,
            npc_tracking::Action::Command(npc_tracking::Command::Snap { .. })
        );
        self.collision_monitor
            .record_tracking(name, error, yaw_error, snapped);

        let actor = view
            .actors
            .find(actor_id)
            .wrap_err("find tracked actor")?
            .ok_or_else(|| eyre::eyre!("actor {actor_id} not found"))?;
        let command = match action {
            npc_tracking::Action::Command(c) => c,
            npc_tracking::Action::Fallback => {
                let speed = target.vx.hypot(target.vy);
                let along = if speed > 0.0 {
                    (body_planar.0 * target.vx + body_planar.1 * target.vy) / speed
                } else {
                    0.0
                };
                // The span's `frame` is the gap trace's first `kinematic` frame.
                tracing::warn!(
                    "Tracking '{name}': blocked -- moving {along:.2} m/s along SSv2's \
                     {speed:.2} m/s, {error:.2} m off -- so not snapped into what holds it: \
                     kinematic for the rest of the episode (physics off, SSv2's pose every \
                     frame)"
                );
                actor
                    .set_simulate_physics(false)
                    .wrap_err("set_simulate_physics(false) (fallback)")?;
                // As the kinematic path places it from now on (update_entity_status).
                let mut place = target_tf;
                if let Some(z) = self.walker_teleport_z(actor_id, &place.location) {
                    place.location.z = z;
                }
                actor
                    .set_transform(&place)
                    .wrap_err("set_transform (fallback)")?;
                self.tracked_npcs.remove(&actor_id);
                self.kinematic_fallback.insert(actor_id);
                self.collision_monitor.record_tracking_fallback(name);
                return Ok(true);
            }
        };
        let (vx, vy, yaw_rate_deg, steer) = match command {
            npc_tracking::Command::Track {
                vx,
                vy,
                yaw_rate_deg,
                steer,
            } if npc.walker => (vx, vy, yaw_rate_deg, steer),
            npc_tracking::Command::Track {
                vx,
                vy,
                yaw_rate_deg,
                steer,
            } => {
                // A finite force: the change from the body's velocity is capped
                // (npc_tracking::limit_change), so a contact holds it back like a car --
                // except on the first frame of an NPC SSv2 adds moving, which starts at
                // speed (npc_tracking::first_frame_change). The turn SSv2's target makes
                // this frame is not charged to the cap (npc_tracking::bounded_command,
                // roadmap 032 gap B): it used all of it in the bench's corners.
                let (vx, vy) = npc_tracking::bounded_command(
                    (vx, vy),
                    body_planar,
                    target.yaw_rate_deg,
                    dt,
                    npc_tracking::first_frame_change(
                        npc.first,
                        status.is_some(),
                        npc_tracking::max_change(npc.performance, dt),
                    ),
                );
                (vx, vy, yaw_rate_deg, steer)
            }
            npc_tracking::Command::Snap {
                x,
                y,
                yaw_deg,
                vx,
                vy,
                yaw_rate_deg,
            } => {
                let mut snap_to = now.clone();
                snap_to.location.x = x as f32;
                snap_to.location.y = y as f32;
                snap_to.rotation.yaw = yaw_deg as f32;
                if let Some(z) = self.walker_teleport_z(actor_id, &snap_to.location) {
                    snap_to.location.z = z;
                } else {
                    // Upright on the ground under the target (roadmap 032, gap A), not the
                    // body's roll, pitch and z: a snap never keeps a tumble.
                    let ground = self.ground_height_at(&snap_to.location).map(f64::from);
                    let attitude = npc_tracking::snap_attitude(
                        f64::from(target_tf.rotation.roll),
                        f64::from(target_tf.rotation.pitch),
                        ground,
                        f64::from(target_tf.location.z),
                        f64::from(npc.lift),
                    );
                    snap_to.rotation.roll = attitude.roll_deg as f32;
                    snap_to.rotation.pitch = attitude.pitch_deg as f32;
                    snap_to.location.z = attitude.z as f32;
                }
                tracing::debug!(
                    "Snap '{name}': error {error:.2} m, yaw error {yaw_error:?} deg, target \
                     {:.2} m/s turning {yaw_rate_deg:.1} deg/s",
                    vx.hypot(vy)
                );
                actor
                    .set_transform(&snap_to)
                    .wrap_err("set_transform (snap)")?;
                (vx, vy, yaw_rate_deg, None)
            }
        };
        if let Some(entry) = self.tracked_npcs.get_mut(&actor_id) {
            entry.last_command = Some((vx, vy));
            entry.first = false;
        }
        if npc.walker {
            let walker = carla::client::Walker::try_from(actor.clone())
                .map_err(|_| eyre::eyre!("actor {actor_id} is not a walker"))?;
            // Standing, it faces SSv2's heading at speed 0: a noise-sized command normalised
            // into a direction spun it 18 deg per frame (roadmap 032, gap C).
            let (dx, dy, speed) = npc_tracking::walker_control(vx, vy, target.yaw_deg);
            let mut control = walker.control().wrap_err("read walker control")?;
            // The control's direction is CARLA's own C++ Vector3D (a POD), not carla::geom's.
            control.direction.x = dx as f32;
            control.direction.y = dy as f32;
            control.direction.z = 0.0;
            control.speed = speed as f32;
            control.jump = false;
            walker.apply_control(&control).wrap_err("walker control")?;
        } else {
            // Gravity and the suspension keep z; only the plane is commanded. A snap sets the
            // whole velocity to the command's: no vertical speed carried into the new pose.
            actor
                .set_target_velocity(&carla::geom::Vector3D {
                    x: vx as f32,
                    y: vy as f32,
                    z: if snapped { 0.0 } else { body_v.z },
                })
                .wrap_err("set_target_velocity")?;
            actor
                .set_target_angular_velocity(&carla::geom::Vector3D {
                    x: 0.0,
                    y: 0.0,
                    z: yaw_rate_deg as f32,
                })
                .wrap_err("set_target_angular_velocity")?;
            if let (Some(steer), carla::client::ActorKind::Vehicle(vehicle)) =
                (steer, actor.clone().into_kinds())
            {
                let control = carla::rpc::VehicleControl {
                    throttle: 0.0,
                    steer: steer as f32,
                    brake: 0.0,
                    hand_brake: false,
                    reverse: false,
                    manual_gear_shift: false,
                    gear: 0,
                };
                vehicle.apply_control(&control).wrap_err("steer")?;
            }
        }
        Ok(true)
    }

    fn set_actor_transform(&self, actor_id: u32, transform: &Transform) -> Result<()> {
        let actors = self.world.actors().wrap_err("get actors")?;
        let actor = actors
            .find(actor_id)
            .wrap_err("find actor")?
            .ok_or_else(|| eyre::eyre!("actor {actor_id} not found"))?;
        actor.set_transform(transform).wrap_err("set_transform")?;
        Ok(())
    }
}

impl SimBackend for CarlaBackend {
    fn before_serving(&mut self) {
        // The configured weather, now that there is a world to apply it to.
        self.reapply_weather("connect");
        // The episode's epoch, for a vehicle side started before the first scenario.
        self.write_epoch_marker_at_connect();
    }

    fn initialize(&mut self, episode: &Episode) -> Result<i64, BackendError> {
        CarlaBackend::initialize(self, episode)
    }

    fn update_frame(&mut self, current_simulation_time: f64) -> Result<i64, BackendError> {
        CarlaBackend::update_frame(self, current_simulation_time)
    }

    fn update_step_time(&mut self, step_time: f64) -> Result<(), BackendError> {
        CarlaBackend::update_step_time(self, step_time)
    }

    fn spawn(&mut self, request: &SpawnRequest) -> Result<(), BackendError> {
        CarlaBackend::spawn(self, request)
    }

    fn despawn(&mut self, name: &str) -> Result<(), BackendError> {
        CarlaBackend::despawn(self, name)
    }

    fn update_entity_status(&mut self, batch: StatusBatch) -> StatusOutcome {
        CarlaBackend::update_entity_status(self, batch)
    }

    fn update_entity_goal(&mut self, name: &str, goal: &Goal) -> Result<(), BackendError> {
        CarlaBackend::update_entity_goal(self, name, goal)
    }

    fn update_traffic_lights(&mut self, signals: &[TrafficSignal]) -> Result<(), BackendError> {
        CarlaBackend::update_traffic_lights(self, signals)
    }

    fn attach_sensor(&mut self, sensor: Sensor) -> Result<(), BackendError> {
        CarlaBackend::attach_sensor(self, sensor)
    }

    fn get_weather(&self) -> WeatherReply {
        CarlaBackend::get_weather(self)
    }

    fn set_weather(&mut self, request: WeatherRequest) -> WeatherReply {
        CarlaBackend::set_weather(self, request)
    }

    fn entity_count(&self) -> usize {
        CarlaBackend::entity_count(self)
    }

    fn take_tick_time(&mut self) -> Duration {
        CarlaBackend::take_tick_time(self)
    }

    fn probe_connection_idle(&mut self) {
        CarlaBackend::probe_connection_idle(self)
    }

    fn wedged(&self) -> Option<&str> {
        CarlaBackend::wedged(self)
    }

    fn server_lost(&self) -> Option<&str> {
        CarlaBackend::server_lost(self)
    }

    fn abandon_after_hang(&mut self) {
        CarlaBackend::abandon_after_hang(self)
    }

    fn shutdown(&mut self) {
        CarlaBackend::shutdown(self)
    }
}

/// What a spawn request says about the entity's body: where its origin sits in its box,
/// how big the box is, and what kind of vehicle it is.
#[derive(Debug, Clone, Copy)]
struct EntityShape {
    origin_offset: OriginOffset,
    /// The declared `bounding_box.center` (x, y, z), zero when absent (the gap trace).
    center: [f64; 3],
    /// The declared box; `None` when absent or degenerate.
    declared: Option<Dimensions>,
    subtype: EntitySubtype,
    /// Declared front and rear axle `positionX`, m from the entity origin (vehicles).
    axles: Option<(f64, f64)>,
    /// Declared `(maxAcceleration, maxDeceleration)`, m/s2 (vehicles).
    performance: Option<(f64, f64)>,
}

impl EntityShape {
    fn of(bounding_box: Option<&BoundingBox>, subtype: EntitySubtype) -> Self {
        let declared = bounding_box
            .and_then(|b| b.dimensions.as_ref())
            .map(|d| Dimensions {
                length: d.x,
                width: d.y,
                height: d.z,
            })
            .filter(Dimensions::is_valid);
        EntityShape {
            origin_offset: origin_offset_of(bounding_box),
            center: bounding_box
                .and_then(|b| b.center.as_ref())
                .map_or([0.0; 3], |c| [c.x, c.y, c.z]),
            declared,
            subtype,
            axles: None,
            performance: None,
        }
    }
}

/// `4.50 x 2.10 x 1.80 m`.
fn format_dims(d: &Dimensions) -> String {
    format!("{:.2} x {:.2} x {:.2} m", d.length, d.width, d.height)
}

/// A prop's box as CARLA reads it back, in the prop's own frame, or `None` when it cannot
/// be recovered. CARLA 0.9.16 does not report a prop's `bounding_box()` in the actor's
/// frame: the street barrier reads 1.21 x 0.37 m at yaw 0, 0.37 x 1.21 at 90 and
/// 0.60 x 1.12 at 45. At a multiple of 90° it is the box, swapped at 90/270; at any other
/// yaw it is neither the box nor its axis-aligned bound.
fn prop_box_in_actor_frame(read: Dimensions, yaw_deg: f32) -> Option<Dimensions> {
    const TOLERANCE_DEG: f32 = 1.0;
    let quarter = yaw_deg.rem_euclid(180.0);
    if quarter < TOLERANCE_DEG || quarter > 180.0 - TOLERANCE_DEG {
        Some(read)
    } else if (quarter - 90.0).abs() < TOLERANCE_DEG {
        Some(read.turned())
    } else {
        None
    }
}

/// A prop's box-centre offset as CARLA reads it, into the prop's own frame (CARLA axes),
/// by the same reading as [`prop_box_in_actor_frame`]: at a multiple of 90° CARLA reports
/// the box in world axes, so the offset is turned back by the yaw. Only called when that
/// function recovered the box.
fn prop_offset_in_actor_frame(x: f64, y: f64, yaw_deg: f32) -> (f64, f64) {
    let quarters = (f64::from(yaw_deg) / 90.0).round();
    let (sin, cos) = (-quarters * std::f64::consts::FRAC_PI_2).sin_cos();
    let (x, y) = (x * cos - y * sin, x * sin + y * cos);
    // Exact quarter turns: no 1e-17 residue or -0.0 in the trace.
    let clean = |v: f64| (v * 1e9).round() / 1e9 + 0.0;
    (clean(x), clean(y))
}

/// An SSv2 pose as the gap trace writes it: `[x, y, z, roll, pitch, yaw]`, map frame.
fn ssv2_trace_pose(pose: &Pose) -> gap_trace::Pose6 {
    let p = pose.position.unwrap_or_default();
    let (roll, pitch, yaw) = pose.orientation.as_ref().map_or((0.0, 0.0, 0.0), |q| {
        coordinate_conversion::quaternion_to_euler(q.x, q.y, q.z, q.w)
    });
    [p.x, p.y, p.z, roll, pitch, yaw]
}

/// An actor's state from a world snapshot as the gap trace writes it: its transform and
/// velocity in the map frame, raw (no `OriginOffset` undone).
fn carla_trace_state(
    actor: &carla::client::ActorSnapshot,
    map_origin: MapOrigin,
) -> gap_trace::CarlaState {
    let t = actor.transform();
    let v = actor.velocity();
    let (x, y, z) = map_origin.carla_to_map(
        f64::from(t.location.x),
        f64::from(t.location.y),
        f64::from(t.location.z),
    );
    let (roll, pitch, yaw) = coordinate_conversion::carla_to_ros_rotation(
        f64::from(t.rotation.roll),
        f64::from(t.rotation.pitch),
        f64::from(t.rotation.yaw),
    );
    let (vx, vy, vz) = coordinate_conversion::carla_to_ros_velocity(v.x, v.y, v.z);
    ([x, y, z, roll, pitch, yaw], [vx, vy, vz])
}

/// The origin offset an SSv2 entity declares: its `bounding_box.center` x and y. Absent or
/// zero (pedestrians and misc objects usually) means the SSv2 and CARLA origins coincide.
///
/// The declaration is trusted over CARLA's geometry. Vehicles declare 1.5 m, while
/// `vehicle.tesla.model3`'s rear axle is 1.386 m behind its origin; the 0.11 m residual is a
/// blueprint-fidelity matter (roadmap 014, gap 9), not something this shift should absorb.
fn origin_offset_of(bounding_box: Option<&BoundingBox>) -> OriginOffset {
    bounding_box
        .and_then(|b| b.center.as_ref())
        .map(|c| OriginOffset {
            x: c.x,
            y: c.y,
            yaw: 0.0,
        })
        .unwrap_or_default()
}

/// Convert an SSv2 pose to the CARLA transform of its actor: shifted by `origin_offset`
/// from the entity origin to the actor origin in the ROS frame, moved from the map frame to
/// CARLA's world origin by `map_origin`, then Y-flipped, and turned by its `yaw` (a turned
/// prop, roadmap 022).
fn ros_pose_to_carla_transform(
    pose: &Pose,
    origin_offset: OriginOffset,
    map_origin: MapOrigin,
) -> Transform {
    let mut shifted = *pose;
    if let (Some(p), Some(q)) = (shifted.position.as_mut(), pose.orientation.as_ref()) {
        let (_, _, yaw) = coordinate_conversion::quaternion_to_euler(q.x, q.y, q.z, q.w);
        (p.x, p.y) =
            coordinate_conversion::entity_origin_to_actor_origin(p.x, p.y, yaw, origin_offset);
        p.x -= map_origin.x;
        p.y -= map_origin.y;
        p.z -= map_origin.z;
    }
    let (cx, cy, cz, cr, cp, cyaw) = coordinate_conversion::ros_pose_to_carla(&shifted);
    Transform {
        location: Location {
            x: cx as f32,
            y: cy as f32,
            z: cz as f32,
        },
        rotation: Rotation {
            roll: cr as f32,
            pitch: cp as f32,
            // CARLA's yaw is the ROS yaw negated (Y-flip).
            yaw: (cyaw - origin_offset.yaw.to_degrees()) as f32,
        },
    }
}

/// A request that failed, and why: the reply SSv2 gets (logged by the coordinator).
fn failed<T>(why: impl Into<String>) -> Result<T, BackendError> {
    Err(BackendError::Failed(why.into()))
}

/// Why SSv2's sensors are refused.
const SENSORS_ELSEWHERE: &str = "CARLA sensors provided by autoware_carla_bridge";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_spawn_timeout_hook_is_off_unless_set_to_milliseconds() {
        assert_eq!(parse_spawn_timeout_hook(None), None);
        assert_eq!(parse_spawn_timeout_hook(Some("")), None);
        assert_eq!(parse_spawn_timeout_hook(Some("0")), None);
        assert_eq!(parse_spawn_timeout_hook(Some("fast")), None);
        assert_eq!(
            parse_spawn_timeout_hook(Some(" 5 ")),
            Some(Duration::from_millis(5))
        );
    }

    #[test]
    fn only_carlas_collision_error_climbs_the_ladder() {
        // ActorSpawnResult.cpp's wording for a blocked point, and the others.
        assert!(is_collision_error(
            "Spawn failed because of collision at spawn position"
        ));
        assert!(!is_collision_error(
            "Spawn failed because of invalid actor description"
        ));
        assert!(!is_collision_error(
            "Unknown error while trying to spawn actor"
        ));
    }

    #[test]
    fn entity_marks_are_recognised_and_configured_roles_are_not() {
        assert_eq!(entity_role_name("npc_3"), "csb_entity:npc_3");
        assert!(is_entity_role_name(&entity_role_name("walker")));
        for role in ["hero", "bg_av_1", "autopilot", "", "csb_entity"] {
            assert!(!is_entity_role_name(role), "{role}");
        }
    }

    /// Regression guard for gap 1. Enabling sync mode at Initialize deadlocks startup:
    /// The scenario's timeline must not move when substeps change: SSv2 asks for a frame
    /// of `step_time` and gets exactly that, however many ticks it is delivered in.
    #[test]
    fn substeps_divide_the_step_without_changing_the_frame() {
        for (step, subs) in [(0.1_f64, 1_u32), (0.1, 2), (0.1, 4), (0.05, 2)] {
            let delta = step / subs.max(1) as f64;
            assert!(
                (delta * subs as f64 - step).abs() < 1e-12,
                "substeps {subs} at step {step} must still advance one frame"
            );
        }
    }

    /// A 0.05 s CARLA tick at every frame rate SSv2 is run at, and `substeps` only as the
    /// fallback when no tick is configured.
    #[test]
    fn ticks_per_frame_keep_the_carla_tick_fixed() {
        use crate::config::ticks_per_frame;
        let tick = Some(0.05);
        assert_eq!(ticks_per_frame(0.1, 2, tick), 2);
        assert_eq!(ticks_per_frame(0.05, 2, tick), 1);
        assert_eq!(ticks_per_frame(0.2, 1, tick), 4);
        assert_eq!(ticks_per_frame(1.0 / 30.0, 2, tick), 1);
        assert_eq!(ticks_per_frame(0.025, 2, tick), 1);
        assert_eq!(ticks_per_frame(0.1, 2, None), 2);
        assert_eq!(ticks_per_frame(0.1, 0, None), 1);
        assert_eq!(ticks_per_frame(0.1, 3, Some(0.0)), 3);
    }

    /// Zero would divide by zero and stop time; the config clamps it.
    #[test]
    fn zero_substeps_is_treated_as_one() {
        let step = 0.1_f64;
        // Through a binding, so this exercises the clamp rather than const-folding to
        // `step / 1.0` before it ever runs -- which is what the literal form did.
        let substeps: u32 = 0;
        assert_eq!(step / substeps.max(1) as f64, step);
    }

    /// acb_bridge polls for its vehicle and needs CARLA free-running, but in sync mode
    /// nothing advances until this bridge ticks, and this bridge does not tick until
    /// SSv2 sends a frame, which SSv2 will not do until the ego exists.
    #[test]
    fn frames_before_the_ego_spawns_do_not_tick() {
        assert_eq!(
            decide_frame_action(false, false),
            FrameAction::WaitForEgo,
            "CARLA must stay async until the ego exists, or acb_bridge can never find it"
        );
    }

    #[test]
    fn the_first_frame_after_the_ego_spawns_enables_sync_mode() {
        assert_eq!(
            decide_frame_action(false, true),
            FrameAction::EnableSyncThenTick
        );
    }

    #[test]
    fn later_frames_only_tick() {
        assert_eq!(decide_frame_action(true, true), FrameAction::Tick);
    }

    /// Sync mode is enabled exactly once. Should the ego flag ever be cleared without
    /// leaving sync mode, keep ticking rather than re-applying world settings mid-run.
    #[test]
    fn only_the_ego_spawn_switches_to_sync_and_only_once() {
        assert!(sync_before_spawn(SpawnKind::Ego, false));
        assert!(!sync_before_spawn(SpawnKind::Ego, true));
        for kind in [
            SpawnKind::Npc,
            SpawnKind::Pedestrian,
            SpawnKind::MiscObject,
            SpawnKind::AgentVehicle,
        ] {
            assert!(!sync_before_spawn(kind, false), "{kind:?}");
        }
    }

    #[test]
    fn sync_mode_is_not_re_enabled() {
        assert_eq!(decide_frame_action(true, false), FrameAction::Tick);
    }

    // Phase 006's traffic-light rejection test is gone: phase 009 implements the handler,
    // and the real conversion is covered in `traffic_light_mapper`. Sensor rejection is
    // still asserted by `rejections_always_explain_themselves` below.

    // --- Phase 007: repeatable runs ---

    /// Regression guard for B3/C2. The old code forced every spawn to z >= 0.5 regardless
    /// of terrain, which buried vehicles on elevated roads.
    #[test]
    fn spawn_height_sits_above_the_road() {
        assert!((spawn_height_above_ground(0.0) - SPAWN_CLEARANCE).abs() < 1e-6);
        // An elevated road: the old flat clamp would have put the vehicle 10m underground.
        assert!(spawn_height_above_ground(10.0) > 10.0);
        // A road below the origin, which the old clamp raised into the air.
        assert!(spawn_height_above_ground(-3.0) < 0.0);
    }

    #[test]
    fn fallback_height_keeps_a_plausible_commanded_value() {
        // Too low to be real: lift to the floor.
        assert_eq!(fallback_spawn_height(0.0), FALLBACK_SPAWN_Z);
        assert_eq!(fallback_spawn_height(-5.0), FALLBACK_SPAWN_Z);
        // Already plausible: leave it alone rather than flattening it.
        assert_eq!(fallback_spawn_height(12.0), 12.0);
    }

    #[test]
    fn spawn_retries_only_raise_the_height() {
        let heights: Vec<f32> = spawn_retry_heights(2.0).collect();

        assert_eq!(
            heights.len(),
            SPAWN_RETRIES + 1,
            "first attempt plus retries"
        );
        assert_eq!(
            heights[0], 2.0,
            "the first attempt uses the commanded height"
        );
        for pair in heights.windows(2) {
            assert!(pair[1] > pair[0], "each retry must go higher: {heights:?}");
        }
    }

    // --- Phase 008: entity fidelity ---

    /// Regression guard for C1, the whole point of the phase. Anything SSv2 teleports must
    /// have CARLA physics off, or PhysX and `set_transform` both drive the same actor.
    #[test]
    fn only_ssv2_driven_vehicles_and_pedestrians_are_trackable() {
        // npc_motion: tracked (roadmap 023) moves these with physics; props stay put, and
        // everything already physics-driven keeps its own driver.
        assert!(SpawnKind::Npc.trackable());
        assert!(SpawnKind::Pedestrian.trackable());
        for kind in [
            SpawnKind::MiscObject,
            SpawnKind::Ego,
            SpawnKind::AgentVehicle,
            SpawnKind::AutopilotNpc,
        ] {
            assert!(!kind.trackable(), "{kind:?}");
        }
    }

    #[test]
    fn only_the_ego_is_physics_driven() {
        assert!(SpawnKind::Ego.physics_driven());
        assert!(
            SpawnKind::AgentVehicle.physics_driven(),
            "an agent-driven vehicle is driven by its own Autoware through CARLA physics"
        );
        assert!(!SpawnKind::Npc.physics_driven());
        assert!(
            SpawnKind::AutopilotNpc.physics_driven(),
            "a simulator_autopilot vehicle is driven by Traffic Manager through CARLA physics"
        );
        assert!(!SpawnKind::Pedestrian.physics_driven());
        assert!(!SpawnKind::MiscObject.physics_driven());
    }

    /// `acb_bridge` finds its vehicle by role_name. Only the ego should carry one -- an NPC
    /// tagged `hero` would be picked up by a bridge as if it were an Autoware vehicle.
    /// An agent-driven vehicle's role_name is its entity name, so it may not take the ego's
    /// or look like one of csb's marks.
    #[test]
    fn an_agent_vehicle_needs_a_role_name_of_its_own() {
        assert_eq!(agent_role_conflict("bg_av_1", "hero"), None);
        assert!(agent_role_conflict("hero", "hero").unwrap().contains("ego"));
        assert!(agent_role_conflict("", "hero").is_some());
        assert!(agent_role_conflict("csb_entity:x", "hero").is_some());
    }

    #[test]
    fn ssv2_poses_become_agent_poses() {
        let pose = Pose {
            position: Some(geometry_msgs::Point {
                x: 230.0,
                y: -129.8,
                z: 0.3,
            }),
            orientation: Some(geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 1.0,
                w: 0.0,
            }),
        };
        let a = agent_pose(&pose);
        assert_eq!((a.x, a.y, a.z, a.qz, a.qw), (230.0, -129.8, 0.3, 1.0, 0.0));
        let a = agent_pose(&Pose::default());
        assert_eq!(a.qw, 1.0, "a missing orientation is the identity");
    }

    #[test]
    fn the_behavior_decides_who_drives_a_vehicle() {
        let kind = |is_ego, behavior| SpawnKind::of_driver(Driver::of_vehicle(is_ego, behavior));
        assert_eq!(kind(true, "Autoware"), SpawnKind::Ego);
        assert_eq!(kind(false, ""), SpawnKind::Npc);
        assert_eq!(
            kind(false, "behavior_tree_plugin/VehicleBehaviorTree"),
            SpawnKind::Npc
        );
        assert_eq!(kind(false, "simulator_autopilot"), SpawnKind::AutopilotNpc);
        assert_eq!(kind(false, "agent"), SpawnKind::AgentVehicle);
        assert_eq!(kind(true, "agent"), SpawnKind::Ego);
    }

    #[test]
    fn each_kind_maps_to_its_entity_type() {
        assert_eq!(SpawnKind::Ego.entity_type(), Some(EntityType::Ego));
        assert_eq!(SpawnKind::Npc.entity_type(), Some(EntityType::Vehicle));
        assert_eq!(
            SpawnKind::AutopilotNpc.entity_type(),
            Some(EntityType::Vehicle)
        );
        assert_eq!(
            SpawnKind::AgentVehicle.entity_type(),
            Some(EntityType::Vehicle)
        );
        assert_eq!(
            SpawnKind::Pedestrian.entity_type(),
            Some(EntityType::Pedestrian)
        );
        assert_eq!(
            SpawnKind::MiscObject.entity_type(),
            Some(EntityType::MiscObject)
        );
    }

    /// An SSv2 asset key that happens to name a real CARLA blueprint passes through
    /// unchanged. This is the only path that preserves what the scenario actually asked
    /// for, so a regression here silently substitutes a different vehicle.
    #[test]
    fn a_known_asset_key_is_used_as_is() {
        assert_eq!(choose_blueprint(true, None), BlueprintChoice::Requested);
        // Whatever the fallback looks like, a usable request wins.
        assert_eq!(
            choose_blueprint(true, Some(true)),
            BlueprintChoice::Requested
        );
        assert_eq!(
            choose_blueprint(true, Some(false)),
            BlueprintChoice::Requested
        );
    }

    /// SSv2 asset keys are not CARLA blueprint names in general, so the common case is a
    /// key CARLA has never heard of. That must not abort the scenario while the kind's
    /// default is available.
    #[test]
    fn an_unknown_asset_key_falls_back_to_the_kind_default() {
        assert_eq!(
            choose_blueprint(false, Some(true)),
            BlueprintChoice::Fallback
        );
    }

    /// With neither usable there is nothing to spawn, and the caller must fail rather than
    /// invent a substitute -- a scenario that silently spawns the wrong kind of actor is
    /// worse than one that stops.
    #[test]
    fn an_unknown_key_with_an_unusable_fallback_is_an_error() {
        assert_eq!(
            choose_blueprint(false, Some(false)),
            BlueprintChoice::Neither
        );
    }

    /// `None` means the fallback was never probed. That only happens when the request
    /// succeeded, so reaching the fallback branch with `None` is a caller bug; treating it
    /// as "no blueprint" fails loudly instead of substituting one that was never checked.
    #[test]
    fn an_unprobed_fallback_is_not_assumed_to_work() {
        assert_eq!(choose_blueprint(false, None), BlueprintChoice::Neither);
    }

    /// The whole point of the median: one PhysX contact jolt must not reach SSv2. A mean
    /// of three leaves a third of it in the reported value, which is how a 52 m/s2 jolt
    /// was reported as 17.5 and aborted a scenario against SSv2's +-15 bound.
    #[test]
    fn a_single_frame_jolt_is_rejected() {
        // Two ordinary frames either side of one contact jolt.
        assert_eq!(median_of([-1.2, -52.6, -1.4].into_iter()), -1.4);
        assert_eq!(median_of([0.5, 17.5, 0.6].into_iter()), 0.6);
        // A mean would have passed a third of each through; check the gap is real.
        let mean = (-1.2 + -52.6 + -1.4) / 3.0;
        assert!(
            mean < -18.0,
            "mean of the same window is {mean}, well past SSv2's bound"
        );
    }

    /// A real braking profile is not an outlier: once two frames agree, the median has to
    /// follow them. Rejecting sustained deceleration would hide exactly what SSv2 is
    /// validating.
    #[test]
    fn a_sustained_profile_passes_through() {
        assert_eq!(median_of([-3.0, -3.2, -3.1].into_iter()), -3.1);
        // Onset: two frames of braking outvote the one remaining quiet frame.
        assert_eq!(median_of([0.0, -4.0, -4.2].into_iter()), -4.0);
    }

    /// Before the window fills there is no third sample to break the tie, so the even case
    /// averages the two straddling the middle rather than inventing a preference.
    #[test]
    fn a_partial_window_still_reports_something_sane() {
        assert_eq!(median_of([2.0].into_iter()), 2.0);
        assert_eq!(median_of([2.0, 4.0].into_iter()), 3.0);
        assert_eq!(median_of(std::iter::empty()), 0.0);
    }

    /// A pedestrian must not fall back to a car, nor a prop to a walker.
    #[test]
    fn fallback_blueprints_match_their_kind() {
        assert!(SpawnKind::Ego.default_blueprint().starts_with("vehicle."));
        assert!(SpawnKind::Npc.default_blueprint().starts_with("vehicle."));
        assert!(SpawnKind::Pedestrian
            .default_blueprint()
            .starts_with("walker."));
        assert!(SpawnKind::MiscObject
            .default_blueprint()
            .starts_with("static."));
    }

    /// The ego's body is the ego stack's to choose and a walker's is never size-matched;
    /// an NPC follows its subtype, a prop is matched among props.
    #[test]
    fn only_npcs_and_props_are_size_matched() {
        for kind in [
            SpawnKind::Ego,
            SpawnKind::AgentVehicle,
            SpawnKind::Pedestrian,
        ] {
            for subtype in [
                EntitySubtype::Car,
                EntitySubtype::Truck,
                EntitySubtype::Unknown,
            ] {
                assert_eq!(kind.size_class(subtype), None, "{kind:?} {subtype:?}");
            }
        }
        assert_eq!(
            SpawnKind::Npc.size_class(EntitySubtype::Bus),
            Some(SizeClass::Bus)
        );
        assert_eq!(
            SpawnKind::AutopilotNpc.size_class(EntitySubtype::Unknown),
            Some(SizeClass::FourWheeled)
        );
        assert_eq!(
            SpawnKind::MiscObject.size_class(EntitySubtype::Unknown),
            Some(SizeClass::MiscObject)
        );
    }

    /// Jerk needs a signed scalar, so acceleration is projected onto the heading. The
    /// magnitude would report braking and accelerating identically.
    #[test]
    fn world_vectors_are_reported_in_the_body_frame() {
        use std::f64::consts::{FRAC_PI_2, PI};
        let close =
            |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9;
        // Heading +x: unchanged.
        assert!(close(world_to_body(3.0, 1.0, 0.0), (3.0, 1.0)));
        // Heading -x at 3 m/s: forward, not reversing.
        assert!(close(world_to_body(-3.0, 0.0, PI), (3.0, 0.0)));
        // Heading +y at 2.8 m/s: forward, not standing still.
        assert!(close(world_to_body(0.0, 2.8, FRAC_PI_2), (2.8, 0.0)));
        // A world +x component seen heading +y is to the right (negative y).
        assert!(close(world_to_body(1.0, 0.0, FRAC_PI_2), (0.0, -1.0)));
    }

    #[test]
    fn longitudinal_acceleration_keeps_its_sign() {
        // Heading +x: accelerating forward is positive, braking negative.
        assert!((longitudinal_acceleration(2.0, 0.0, 0.0) - 2.0).abs() < 1e-9);
        assert!((longitudinal_acceleration(-2.0, 0.0, 0.0) + 2.0).abs() < 1e-9);

        // Heading +y (90 deg): the y component is now the longitudinal one.
        let yaw = std::f64::consts::FRAC_PI_2;
        assert!((longitudinal_acceleration(0.0, 3.0, yaw) - 3.0).abs() < 1e-9);

        // Pure lateral acceleration is not longitudinal at all.
        assert!(longitudinal_acceleration(0.0, 5.0, 0.0).abs() < 1e-9);
    }

    #[test]
    fn jerk_is_the_rate_of_change_of_acceleration() {
        // 0 -> 1 m/s^2 over 0.05 s is 20 m/s^3.
        assert!((jerk_from_acceleration(0.0, 1.0, 0.05) - 20.0).abs() < 1e-9);
        // Steady acceleration means no jerk.
        assert!(jerk_from_acceleration(2.0, 2.0, 0.05).abs() < 1e-9);
        // Easing off produces negative jerk.
        assert!(jerk_from_acceleration(2.0, 1.0, 0.05) < 0.0);
    }

    /// A zero or negative step time must not produce an infinity or NaN that SSv2 would
    /// then evaluate conditions against.
    #[test]
    fn jerk_is_finite_for_a_bad_step_time() {
        assert_eq!(jerk_from_acceleration(0.0, 1.0, 0.0), 0.0);
        assert_eq!(jerk_from_acceleration(0.0, 1.0, -0.05), 0.0);
    }

    /// A teardown sweep with nothing recorded must not report phantom work.
    #[test]
    fn empty_teardown_reports_nothing() {
        let mut ledger = SpawnLedger::default();
        let mut calls = 0;
        let (report, destroyed) = ledger.teardown(|_| {
            calls += 1;
            Ok(())
        });
        assert_eq!(calls, 0, "nothing recorded, so nothing to destroy");
        assert!(destroyed.is_empty());
        assert_eq!(report.attempted, 0);
        assert_eq!(report.destroyed, 0);
        assert_eq!(report.failed, 0);
    }

    /// Teardown must chase every actor this bridge created, including the ones
    /// `EntityManager` has already forgotten.
    ///
    /// This is the leak phase 007 exists to close: `EntityManager` is a name lookup that
    /// gets emptied on despawn and on re-initialise, so an actor whose name mapping went
    /// first used to become untouchable. The ledger is deliberately not keyed by name and
    /// never learns that a name was dropped -- which is exactly what this asserts.
    #[test]
    fn teardown_destroys_entities_dropped_from_entity_manager() {
        let mut entities = EntityManager::new();
        let mut ledger = SpawnLedger::default();

        for (name, kind, actor_id) in [
            ("ego", EntityType::Ego, 10u32),
            ("npc_1", EntityType::Vehicle, 11),
            ("bg_av_1", EntityType::Vehicle, 12),
        ] {
            entities.insert(name.to_string(), kind, actor_id, OriginOffset::default());
            ledger.record(actor_id);
        }

        // The scenario despawns one entity and re-initialises, which historically left the
        // ledger as the only remaining trace of actors 10 and 11.
        entities.remove("ego");
        entities.clear();
        assert!(entities.get("ego").is_none());

        let mut destroyed_ids = Vec::new();
        let (report, destroyed) = ledger.teardown(|actor_id| {
            destroyed_ids.push(actor_id);
            Ok(())
        });

        assert_eq!(
            destroyed_ids,
            vec![10, 11, 12],
            "every spawned actor is chased"
        );
        assert_eq!(destroyed, vec![10, 11, 12]);
        assert_eq!(report.attempted, 3);
        assert_eq!(report.destroyed, 3);
        assert_eq!(report.failed, 0);
        assert_eq!(ledger.len(), 0, "destroyed actors are forgotten");
    }

    /// An actor that could not be destroyed stays on the books for the next sweep; only
    /// confirmed destruction forgets it.
    #[test]
    fn teardown_keeps_actors_it_failed_to_destroy() {
        let mut ledger = SpawnLedger::default();
        ledger.record(10);
        ledger.record(11);

        let (report, destroyed) = ledger.teardown(|actor_id| {
            if actor_id == 11 {
                eyre::bail!("CARLA is gone")
            }
            Ok(())
        });

        assert_eq!(report.attempted, 2);
        assert_eq!(report.destroyed, 1);
        assert_eq!(report.failed, 1);
        assert_eq!(destroyed, vec![10]);
        assert!(!ledger.contains(10));
        assert!(ledger.contains(11), "a failure is retried, not lost");
    }

    /// Unfreezing must be skipped entirely when this bridge never froze anything: a shared
    /// CARLA server may have had its lights frozen by someone else, and stomping that is
    /// the same class of bug as leaving our own freeze behind.
    #[test]
    fn unfreeze_is_skipped_when_nothing_was_frozen() {
        let mut guard = FreezeGuard::default();
        assert!(!guard.needs_restore(), "nothing frozen, nothing to undo");

        guard.mark_frozen();
        assert!(guard.needs_restore());

        guard.mark_restored();
        assert!(
            !guard.needs_restore(),
            "a second shutdown pass must not unfreeze again"
        );
    }

    /// Whatever the message says, it must never be empty -- SSv2 surfaces this text and a
    /// bare failure with no reason is what phase 006 exists to eliminate.
    /// The probe's pose: Town01 (320, -129.8) heading 180 deg with a rear-axle origin. The
    /// CARLA actor must land 1.5 m further along -x, and the offset is applied before the
    /// Y-flip, so CARLA y is just the negated ROS y.
    #[test]
    fn an_ssv2_pose_lands_the_actor_ahead_of_its_rear_axle() {
        let pose = Pose {
            position: Some(geometry_msgs::Point {
                x: 320.0,
                y: -129.8,
                z: 0.0,
            }),
            orientation: Some(geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 1.0,
                w: 0.0,
            }),
        };
        let bbox = BoundingBox {
            center: Some(geometry_msgs::Point {
                x: 1.5,
                y: 0.0,
                z: 0.9,
            }),
            dimensions: None,
        };
        let t =
            ros_pose_to_carla_transform(&pose, origin_offset_of(Some(&bbox)), MapOrigin::default());
        assert!((t.location.x - 318.5).abs() < 1e-3, "{:?}", t.location);
        assert!((t.location.y - 129.8).abs() < 1e-3, "{:?}", t.location);
        assert_eq!(t.location.z, 0.0);

        let untouched =
            ros_pose_to_carla_transform(&pose, origin_offset_of(None), MapOrigin::default());
        assert!((untouched.location.x - 320.0).abs() < 1e-3);
    }

    #[test]
    fn a_prop_box_is_read_back_only_at_right_angles() {
        let barrier = Dimensions {
            length: 1.21,
            width: 0.37,
            height: 1.07,
        };
        assert_eq!(prop_box_in_actor_frame(barrier, 0.0), Some(barrier));
        assert_eq!(prop_box_in_actor_frame(barrier, -180.0), Some(barrier));
        assert_eq!(prop_box_in_actor_frame(barrier, 179.9), Some(barrier));
        // At 90 CARLA reads 0.37 x 1.21: turned back, it is the barrier again.
        let at_90 = barrier.turned();
        assert_eq!(prop_box_in_actor_frame(at_90, 90.0), Some(barrier));
        assert_eq!(prop_box_in_actor_frame(at_90, -270.0), Some(barrier));
        assert_eq!(prop_box_in_actor_frame(barrier, 45.0), None);
    }

    /// The gap trace's prop box offset is turned back with its extent (roadmap 024): read in
    /// world axes at a quarter turn, written in the prop's own.
    #[test]
    fn a_turned_props_box_offset_is_turned_back_into_its_frame() {
        assert_eq!(prop_offset_in_actor_frame(0.2, 0.1, 0.0), (0.2, 0.1));
        assert_eq!(prop_offset_in_actor_frame(0.2, 0.1, 90.0), (0.1, -0.2));
        assert_eq!(prop_offset_in_actor_frame(0.2, 0.1, -90.0), (-0.1, 0.2));
        assert_eq!(prop_offset_in_actor_frame(0.2, 0.1, 180.0), (-0.2, -0.1));
        assert_eq!(prop_offset_in_actor_frame(0.0, 0.0, 270.0), (0.0, 0.0));
    }

    /// A prop matched turned (roadmap 022) keeps its position and is turned a quarter
    /// against the entity's heading: ROS yaw +90° is CARLA yaw -90°.
    #[test]
    fn a_turned_offset_turns_the_actor_only() {
        let pose = Pose {
            position: Some(geometry_msgs::Point {
                x: 233.0,
                y: -129.5,
                z: 0.0,
            }),
            orientation: Some(geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        };
        let turned = OriginOffset {
            yaw: std::f64::consts::FRAC_PI_2,
            ..OriginOffset::default()
        };
        let plain =
            ros_pose_to_carla_transform(&pose, OriginOffset::default(), MapOrigin::default());
        let t = ros_pose_to_carla_transform(&pose, turned, MapOrigin::default());
        assert_eq!(
            (t.location.x, t.location.y),
            (plain.location.x, plain.location.y)
        );
        assert!(
            (t.rotation.yaw - (plain.rotation.yaw - 90.0)).abs() < 1e-3,
            "{:?}",
            t.rotation
        );
    }

    #[test]
    fn rejections_always_explain_themselves() {
        assert!(!SENSORS_ELSEWHERE.is_empty());
    }

    // --- Roadmap 014 hardening batch ---------------------------------------------------

    /// UpdateStepTime and enable_sync_mode share one timing, and it is the SSv2 step split
    /// into substeps -- never the full step.
    #[test]
    fn the_carla_tick_is_the_step_over_substeps() {
        let t = sync_timing(0.05, 2);
        assert!((t.fixed_delta_seconds - 0.025).abs() < 1e-12);
        assert!((sync_timing(0.05, 1).fixed_delta_seconds - 0.05).abs() < 1e-12);
        assert!((sync_timing(0.05, 0).fixed_delta_seconds - 0.05).abs() < 1e-12);
    }

    /// autoware_universe #13406: 2 ms physics substeps, as many as the tick needs.
    #[test]
    fn physics_substeps_are_two_milliseconds() {
        let t = sync_timing(0.05, 2);
        assert_eq!(t.max_substep_delta_time, 0.002);
        assert_eq!(t.max_substeps, 13); // ceil(0.025 / 0.002)
        assert_eq!(sync_timing(0.02, 1).max_substeps, 10); // not 11 from rounding
        assert_eq!(sync_timing(0.001, 1).max_substeps, 1);
    }

    /// Past CARLA's 16 substeps the count is clamped and the substep stretched, so the
    /// substeps still cover the whole tick.
    #[test]
    fn long_ticks_clamp_to_sixteen_substeps_and_still_cover_the_tick() {
        let t = sync_timing(0.05, 1);
        assert_eq!(t.max_substeps, 16);
        assert!((t.max_substep_delta_time - 0.05 / 16.0).abs() < 1e-12);
        for (step, subs) in [
            (0.01, 1),
            (0.025, 1),
            (0.05, 2),
            (0.05, 1),
            (0.1, 1),
            (0.1, 3),
        ] {
            let t = sync_timing(step, subs);
            assert!(t.max_substeps >= 1 && t.max_substeps <= 16);
            assert!(
                t.max_substep_delta_time * t.max_substeps as f64 >= t.fixed_delta_seconds - 1e-12,
                "{step}/{subs}: substeps must cover the tick"
            );
            assert!(t.max_substep_delta_time <= 0.002 + 1e-12 || t.max_substeps == 16);
        }
    }

    #[test]
    fn a_second_ego_is_refused_naming_the_first() {
        let reason = second_ego_rejection(true, Some("Ego"), "Ego2").expect("must refuse");
        assert!(
            reason.contains("'Ego'") && reason.contains("'Ego2'"),
            "{reason}"
        );
        assert!(second_ego_rejection(true, None, "Ego").is_none());
        assert!(second_ego_rejection(false, Some("Ego"), "npc").is_none());
    }

    #[test]
    fn an_unknown_entity_fails_the_status_update_by_name() {
        let r = entity_status_result(&["ghost".to_string()], &[]).expect("a failure");
        assert!(r.contains("'ghost'"), "{r}");
        let r = entity_status_result(&[], &["npc: gone".to_string()]).expect("a failure");
        assert!(r.contains("npc: gone"));
        assert!(entity_status_result(&[], &[]).is_none());
    }

    /// A CARLA walker's box is centred on its origin, so its feet are one half-height down.
    #[test]
    fn a_walker_is_lifted_by_its_half_height() {
        assert!((walker_lift(0.0, 0.93) - 0.93).abs() < 1e-6);
        // A box whose centre is already above the origin needs less.
        assert!((walker_lift(0.9, 0.93) - 0.03).abs() < 1e-6);
        assert_eq!(walker_lift(1.0, 0.5), 0.0);
    }

    /// One lookup on the first teleport, then only after moving more than 2 m from it.
    #[test]
    fn walker_ground_is_looked_up_again_only_after_moving() {
        assert!(walker_ground_is_stale(None, 300.0, -129.8));
        let last = Some((300.0, -129.8));
        assert!(!walker_ground_is_stale(last, 300.0, -129.8));
        assert!(!walker_ground_is_stale(last, 299.5, -129.8));
        assert!(!walker_ground_is_stale(last, 298.5, -128.5)); // 1.9 m diagonally
        assert!(walker_ground_is_stale(last, 297.9, -129.8));
        assert!(walker_ground_is_stale(last, 290.0, -129.8));
    }

    /// The ground wins over SSv2's z; the commanded z is only the no-ground fallback.
    #[test]
    fn walker_origin_is_ground_plus_lift_whatever_ssv2_sends() {
        // town01_pedestrian.xosc: z = 0.3 commanded, road at 0.0, lift 0.93.
        assert!((walker_origin_z(Some(0.0), 0.3, 0.93) - 0.93).abs() < 1e-6);
        assert!((walker_origin_z(Some(0.12), 5.0, 0.93) - 1.05).abs() < 1e-6);
        // No lane nearby: fall back to the commanded feet height.
        assert!((walker_origin_z(None, 0.3, 0.93) - 1.23).abs() < 1e-6);
    }
}
