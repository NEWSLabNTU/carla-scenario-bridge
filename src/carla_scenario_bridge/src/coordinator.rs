use carla::client::{ActorBase, Client, World};
use carla::geom::{Location, Rotation, Transform};
use eyre::{Result, WrapErr};
use std::collections::{HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crate::config::{BackgroundAv, BridgeConfig, SensorReleaseConfig};
use crate::map_resolver::{lanelet_map_file, town_from_map_path};
use crate::traffic_light_mapper::{carla_state_for_signal, signal_uses_arrows, SignalMap};

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
    /// A scenario pedestrian. Also teleported -- SSv2's behaviour plugins own the walk.
    Pedestrian,
    /// A static obstacle. Placed once and left alone unless SSv2 moves it.
    MiscObject,
    /// An extra Autoware instance's vehicle. Driven by CARLA PhysX under that Autoware's
    /// control, and deliberately unknown to SSv2 -- see [`SpawnKind::entity_type`].
    BackgroundAv,
}

/// Which blueprint a spawn should use once CARLA has been asked about them.
///
/// Separated from [`Coordinator::resolve_blueprint_key`] so the policy can be tested
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
            SpawnKind::Ego | SpawnKind::Npc | SpawnKind::BackgroundAv => "vehicle.tesla.model3",
            SpawnKind::Pedestrian => "walker.pedestrian.0001",
            SpawnKind::MiscObject => "static.prop.streetbarrier",
        }
    }

    /// The SSv2 entity type this actor is registered as, if any.
    ///
    /// `None` for a background AV, and that is the whole point: it is a CARLA vehicle driven
    /// by its own Autoware, not a scenario entity. Registering it would put it into
    /// `UpdateEntityStatus`, and SSv2 would start teleporting a vehicle that Autoware is
    /// already driving -- two pose authorities on one actor (invariant 5).
    fn entity_type(self) -> Option<EntityType> {
        match self {
            SpawnKind::Ego => Some(EntityType::Ego),
            SpawnKind::Npc => Some(EntityType::Vehicle),
            SpawnKind::Pedestrian => Some(EntityType::Pedestrian),
            SpawnKind::MiscObject => Some(EntityType::MiscObject),
            SpawnKind::BackgroundAv => None,
        }
    }

    /// Whether CARLA physics drives this actor.
    ///
    /// False means SSv2 owns the pose, so CARLA physics is switched off at spawn. Leaving it
    /// on gives the actor two authorities: `set_transform` places it, then PhysX pulls it
    /// down and shoves it out of collisions before the next frame. The AWSIM pattern this
    /// design follows makes puppeteered actors kinematic for exactly this reason.
    fn physics_driven(self) -> bool {
        // Both are driven by an Autoware through CARLA physics, so neither may be teleported.
        matches!(self, SpawnKind::Ego | SpawnKind::BackgroundAv)
    }

    fn label(self) -> &'static str {
        match self {
            SpawnKind::Ego => "ego",
            SpawnKind::Npc => "NPC vehicle",
            SpawnKind::Pedestrian => "pedestrian",
            SpawnKind::MiscObject => "misc object",
            SpawnKind::BackgroundAv => "background AV",
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

    #[cfg(test)]
    fn len(&self) -> usize {
        self.ids.len()
    }

    fn contains(&self, actor_id: u32) -> bool {
        self.ids.contains(&actor_id)
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

/// Convert a configured background-AV pose into the protobuf pose the spawn path expects.
///
/// Config states yaw in degrees, which is what an operator writing a YAML file will reach
/// for; the wire format is a quaternion.
fn background_av_pose(av: &BackgroundAv) -> Pose {
    let half = av.spawn_pose.yaw.to_radians() / 2.0;
    Pose {
        position: Some(geometry_msgs::Point {
            x: av.spawn_pose.x,
            y: av.spawn_pose.y,
            z: av.spawn_pose.z,
        }),
        orientation: Some(geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: half.sin(),
            w: half.cos(),
        }),
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

use crate::collision_monitor::CollisionMonitor;
use crate::coordinate_conversion::{self, OriginOffset};
use crate::entity_manager::{EntityManager, EntityType};
use crate::episode_clock::{fmt_ns, reload_as_pause, tick_then_read, EpisodeClock, EpisodeReload};
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::simulation_api_schema::{self as api, Result as ProtoResult};
use crate::proto::traffic_simulator_msgs::{self, BoundingBox};
use crate::sensor_release::SensorReleaseNotifier;

/// `(elapsed_seconds, delta_seconds)` of `world`'s current snapshot.
fn read_frame_of(world: &World) -> Result<(f64, f64)> {
    let snapshot = world.snapshot().wrap_err("world snapshot")?;
    let ts = snapshot.timestamp();
    Ok((ts.elapsed_seconds, ts.delta_seconds))
}

/// The coordinator's CARLA calls for one episode change, in the shape
/// [`reload_as_pause`] orders them.
struct CoordinatorReload<'a> {
    coordinator: &'a mut Coordinator,
    town: &'a str,
    current_town: &'a str,
}

impl EpisodeReload for CoordinatorReload<'_> {
    type World = carla::client::World;
    type Error = eyre::Report;

    fn enter_sync(&mut self) -> Result<()> {
        self.coordinator.enable_sync_mode()
    }

    fn read_frame(&mut self) -> Result<(f64, f64)> {
        self.coordinator.read_frame()
    }

    fn reload(&mut self) -> Result<carla::client::World> {
        self.coordinator.prepare_map(self.town, self.current_town)
    }
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

/// Whether to switch CARLA to synchronous mode before this spawn: for the ego, if not
/// already. See the comment at its call in `spawn_entity`.
fn sync_before_spawn(kind: SpawnKind, sync_mode_enabled: bool) -> bool {
    kind == SpawnKind::Ego && !sync_mode_enabled
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
fn entity_status_result(unknown: &[String], teleport_failures: &[String]) -> ProtoResult {
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
    if problems.is_empty() {
        proto_ok()
    } else {
        proto_err(format!("UpdateEntityStatus: {}", problems.join("; ")))
    }
}

pub struct Coordinator {
    /// Kept so the bridge can rebuild `world` after a CARLA outage.
    client: Client,
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
    /// Directory holding per-map config, e.g. the traffic light signal mapping.
    config_dir: PathBuf,
    /// Map directory name to CARLA town, for maps whose directory is not the town name.
    map_aliases: HashMap<String, String>,
    /// CARLA time plus the episode epoch, reported to SSv2 (roadmap 015).
    episode_clock: EpisodeClock,
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
}

impl Coordinator {
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
        Self {
            map_aliases: config.map_alias.clone(),
            config,
            client,
            carla_host,
            carla_port,
            config_dir,
            signal_map: SignalMap::new(),
            episode_clock: EpisodeClock::default(),
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
            held_light_timings: Vec::new(),
            consecutive_carla_failures: 0,
            previous_longitudinal_accel: HashMap::new(),
            settle_frames: HashMap::new(),
            accel_history: HashMap::new(),
            walker_height: HashMap::new(),
            collision_monitor: CollisionMonitor::new(collision_monitor_enabled),
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
    /// Only role names this bridge is configured to own are touched. Anything else in the
    /// world belongs to somebody else -- a manually spawned vehicle, another tool's traffic
    /// -- and is left alone.
    ///
    /// That includes background AVs the config declares but this run did not enable: a
    /// `bg_av_1` left by a previous bridge started with `CSB_BACKGROUND_AVS=all` is exactly
    /// the undriven car in the ego's lane that the opt-in default exists to prevent.
    fn reap_orphaned_vehicles(&mut self) {
        let mut owned: Vec<String> = vec![self.config.ego.role_name.clone()];
        owned.extend(
            self.config
                .background_avs
                .iter()
                .map(|av| av.role_name.clone()),
        );
        owned.extend(self.config.disabled_background_avs.iter().cloned());

        let actors = match self.world.actors() {
            Ok(actors) => actors,
            Err(e) => {
                tracing::warn!("Could not list actors to look for orphans: {e}");
                return;
            }
        };

        let orphans: Vec<(u32, String)> = actors
            .iter()
            .filter(|actor| actor.type_id().starts_with("vehicle."))
            .filter(|actor| !self.spawned_actors.contains(actor.id()))
            .filter_map(|actor| {
                let attrs = actor.attributes().ok()?;
                let role = attrs
                    .iter()
                    .find(|a| a.id() == "role_name")
                    .map(|a| a.value_string())?;
                owned.contains(&role).then(|| (actor.id(), role))
            })
            .collect();

        if orphans.is_empty() {
            return;
        }

        tracing::warn!(
            "Found {} vehicle(s) from a previous bridge process still in the world; \
             destroying them so this run's spawn points are clear",
            orphans.len()
        );

        for (actor_id, role) in orphans {
            // Their acb_bridge may still be listening to sensors attached to them, exactly
            // as for our own actors -- see the release protocol in issue 015.
            self.announce_release(actor_id, &role);
            match destroy_actor_in(&self.world, actor_id) {
                Ok(()) => tracing::warn!("Destroyed orphaned '{role}' (actor {actor_id})"),
                Err(e) => tracing::error!(
                    "Could not destroy orphaned '{role}' (actor {actor_id}): {e}. \
                     Its spawn point is still occupied, and this run may stack a vehicle on it."
                ),
            }
        }
    }

    /// Destroy sensors whose vehicle is gone.
    ///
    /// CARLA keeps ticking a sensor after its parent dies, so an orphaned LiDAR ray-casts on
    /// every frame forever. They accumulate: twenty-one were found on one server, from bridges
    /// that had been killed outright rather than shut down, since a process that is SIGKILLed
    /// never gets to release anything. Clearing them took a stack that had been failing three
    /// runs in a row back to passing, though with three runs either side that is an
    /// observation rather than a proven cause.
    ///
    /// Only parentless ones are touched. A sensor attached to a live vehicle belongs to
    /// whichever bridge created it, and the release protocol in issue 015 exists precisely so
    /// that this process does not destroy those behind its owner's back.
    fn reap_parentless_sensors(&mut self) {
        let actors = match self.world.actors() {
            Ok(actors) => actors,
            Err(e) => {
                tracing::warn!("Could not list actors to look for orphaned sensors: {e}");
                return;
            }
        };

        let orphans: Vec<u32> = actors
            .iter()
            .filter(|actor| actor.type_id().starts_with("sensor."))
            // `parent()` returns a Result whose Ok is the Option. A failure to read it is
            // not evidence of an orphan, so only a definite None counts.
            .filter(|actor| matches!(actor.parent(), Ok(None)))
            .map(|actor| actor.id())
            .collect();

        if orphans.is_empty() {
            return;
        }

        tracing::warn!(
            "Found {} sensor(s) whose vehicle is gone; destroying them so they stop being \
             ticked every frame",
            orphans.len()
        );
        for actor_id in orphans {
            if let Err(e) = destroy_actor_in(&self.world, actor_id) {
                tracing::warn!("Could not destroy orphaned sensor {actor_id}: {e}");
            }
        }
    }

    /// The vehicle sitting on a spawn point, if any, as (actor id, role name).
    ///
    /// Only used to explain a spawn that had to be lifted. Compares in the horizontal plane:
    /// a vehicle blocking a spawn does so by standing on the same patch of road, whatever
    /// its own z is.
    fn vehicle_occupying(&self, x: f32, y: f32, _z: f32) -> Option<(u32, String)> {
        /// Half a car length. Close enough to be the thing that refused the spawn.
        const OCCUPIED_RADIUS_M: f32 = 2.5;

        let actors = self.world.actors().ok()?;
        // Bind the result before returning: the iterator borrows `actors`, so building it
        // in tail position would drop the list while the temporary still holds the borrow.
        let found = actors
            .iter()
            .filter(|actor| actor.type_id().starts_with("vehicle."))
            .find_map(|actor| {
                let t = actor.transform().ok()?;
                let dx = t.location.x - x;
                let dy = t.location.y - y;
                if dx.hypot(dy) > OCCUPIED_RADIUS_M {
                    return None;
                }
                let role = actor
                    .attributes()
                    .ok()
                    .and_then(|attrs| {
                        attrs
                            .iter()
                            .find(|a| a.id() == "role_name")
                            .map(|a| a.value_string())
                    })
                    .unwrap_or_else(|| "?".to_string());
                Some((actor.id(), role))
            });
        found
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
            .drain(|id| entities.name_of_actor(id).map(str::to_owned));
    }

    /// Stop and destroy the ego's collision sensor and log the run's summary. A no-op when
    /// none is attached. Must run before the ego is destroyed -- see
    /// `CollisionMonitor::finish_run`.
    fn finish_collision_monitor(&mut self) {
        let entities = &self.entities;
        self.collision_monitor
            .finish_run(|id| entities.name_of_actor(id).map(str::to_owned));
    }

    /// Destroy every actor this bridge created that is still alive.
    ///
    /// Best effort throughout: CARLA may already be gone, and an actor that has vanished on
    /// its own is a success -- the requested end state holds either way. Never panics, so it
    /// is safe on the shutdown path.
    pub fn destroy_all_spawned(&mut self) -> TeardownReport {
        // Our own collision sensor goes before the ego it is attached to, and its run's
        // summary is logged while the entity names can still be resolved.
        self.finish_collision_monitor();

        // The ledger drives the loop; the world only supplies the destroyer. Split this way
        // so the bookkeeping half is reachable from a unit test, which a Coordinator (and
        // therefore a live CARLA world) is not.
        // Disjoint field borrows: the ledger is taken mutably, the world and the release
        // channel immutably.
        let world = &self.world;
        let notifier = self.sensor_release.as_ref();
        let (report, destroyed) = self.spawned_actors.teardown(|actor_id| {
            // The ledger knows the actor id but not the role name; the notice carries "-"
            // and the bridges match on the id, which is the key both sides always have.
            if let Some(notifier) = notifier {
                notifier.release(actor_id, "-");
            }
            destroy_actor_in(world, actor_id)
        });
        for actor_id in destroyed {
            self.forget_actor_samples(actor_id);
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
        self.destroy_all_spawned();
        self.restore_traffic_lights();
        self.restore_async_mode();
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
            let _ = self.client.set_timeout(Duration::from_secs(120));
            let prepared = self.reload_as_pause(&town, &current_town);
            let _ = self.client.set_timeout(Duration::from_secs(30));
            // The pause left CARLA synchronous: the old world if the load failed (its
            // settings are untouched), and the new one only if the server kept settings
            // across the load. Either way Initialize wants async until the ego exists.
            self.sync_mode_enabled = false;
            if prepared.is_err() {
                self.force_async_mode();
            }
            self.world = prepared?;
            self.force_async_mode();

            // The old world and everything in it is gone. Anything still recorded for
            // teardown refers to actors that no longer exist.
            self.spawned_actors.clear();
            self.previous_longitudinal_accel.clear();
            self.settle_frames.clear();
            self.accel_history.clear();
            self.walker_height.clear();
            self.sync_mode_enabled = false;
            // The reloaded world's lights are CARLA's own again, so we owe no unfreeze.
            self.froze_traffic_lights.mark_restored();
            self.held_light_timings.clear();
            tracing::info!("Map '{town}' loaded");
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
        let out = reload_as_pause(&mut CoordinatorReload {
            coordinator: self,
            town,
            current_town,
        });
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

        let last = out.last_frame.or(self.episode_clock.last_frame());
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
        let available = self
            .client
            .avaiable_maps()
            .map_err(|e| eyre::eyre!("list available maps: {e}"))?;
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
        self.client
            .load_world(town)
            .map_err(|e| eyre::eyre!("load world '{town}': {e}"))
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

        self.write_resolved_signals(town, lanelet2_map_path, &lanelet_lights, &carla_signals);
        Ok(())
    }

    /// Write `traffic_lights.resolved.yaml` beside the Lanelet2 map, or to the fallback
    /// directory if the map directory is read-only. Never fatal: without the file acb
    /// publishes no signals, which the log says, and the scenario still runs.
    fn write_resolved_signals(
        &self,
        town: &str,
        lanelet2_map_path: &str,
        lanelet_lights: &[crate::lanelet_map::TrafficLightElement],
        carla_signals: &[crate::traffic_light_mapper::CarlaSignal],
    ) {
        use crate::resolved_signals as rs;
        let map_file = lanelet_map_file(lanelet2_map_path);
        let table = rs::ResolvedTable::build(
            town,
            &map_file,
            &self.signal_map,
            lanelet_lights,
            carla_signals,
        );
        let yaml = match table.to_yaml() {
            Ok(y) => y,
            Err(e) => {
                tracing::warn!("Could not serialize the resolved traffic light table: {e:#}");
                return;
            }
        };
        let summary = format!(
            "{} mapped way(s) in {} regulatory element(s); unmapped: {} lanelet, {} CARLA",
            table.signals.len(),
            table.regulatory_element_count(),
            table.unmapped.lanelet_way_ids.len(),
            table.unmapped.carla_opendrive_ids.len()
        );
        match rs::write_with_fallback(&rs::map_dir(&map_file), &rs::fallback_dir(town), &yaml) {
            Ok(rs::Written::MapDir(path)) => tracing::info!(
                "Resolved traffic light table ({summary}) written to {}",
                path.display()
            ),
            Ok(rs::Written::Fallback(path, why)) => tracing::warn!(
                "Map directory not writable ({why}); resolved traffic light table ({summary}) \
                 written to the fallback {} instead. acb_bridge's traffic_light_map_path \
                 must name this path, or it publishes no signals.",
                path.display()
            ),
            Err(e) => tracing::warn!(
                "Could not write the resolved traffic light table anywhere ({e:#}); acb_bridge \
                 will publish no traffic signals from CARLA"
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
        Vec<crate::traffic_light_mapper::CarlaSignal>,
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

        let report = crate::traffic_light_mapper::match_signals(&lanelet_lights, &carla_signals);
        let added = self.signal_map.merge_matched(report.matched);

        tracing::info!(
            "Traffic light matching: {} lanelet element(s), {} CARLA light(s), {added} newly \
             mapped by position (tolerance {}m)",
            lanelet_lights.len(),
            carla_signals.len(),
            crate::traffic_light_mapper::SIGNAL_MATCH_TOLERANCE_M
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
    fn enumerate_carla_signals(&self) -> Result<Vec<crate::traffic_light_mapper::CarlaSignal>> {
        use crate::traffic_light_mapper::CarlaSignal;

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

    /// Spawn the configured background AVs.
    ///
    /// These are ordinary CARLA vehicles that a separate Autoware drives. They are tracked
    /// for teardown like anything else this bridge creates, but are **not** registered as
    /// SSv2 entities, so SSv2 neither controls them nor sees them in its conditions.
    ///
    /// A failure to spawn one is not fatal: the scenario ego can still run, and aborting the
    /// whole scenario because a secondary vehicle would not fit is the wrong trade. It is
    /// reported loudly.
    fn spawn_background_avs(&mut self) {
        if self.config.background_avs.is_empty() {
            // Opt-in since the default flipped to none. Say so once per scenario when the
            // config declares some, so a two-AV run started against a bridge that was not
            // asked for them is visible in the bridge log rather than just missing a car.
            if !self.config.disabled_background_avs.is_empty() {
                tracing::info!(
                    "No background AVs spawned; declared but not enabled: {} (start the \
                     bridge with {}=all to spawn them)",
                    self.config.disabled_background_avs.join(", "),
                    crate::config::BACKGROUND_AVS_ENV
                );
            }
            return;
        }

        let avs: Vec<BackgroundAv> = self.config.background_avs.clone();
        tracing::info!("Spawning {} background AV(s)", avs.len());

        let mut spawned = 0;
        for av in &avs {
            let pose = background_av_pose(av);
            let asset_key = av.blueprint.clone().unwrap_or_default();

            let result = self.spawn_entity(
                &av.role_name,
                &asset_key,
                Some(&pose),
                // Not an SSv2 entity: its pose comes from bridge_config and already names
                // the CARLA actor origin, so nothing is shifted.
                OriginOffset::default(),
                SpawnKind::BackgroundAv,
                Some(&av.role_name),
            );

            if result.success {
                spawned += 1;
                // goal_pose is for that domain's pilot, not for this bridge -- logged so the
                // operator can see what the run expects without opening the config.
                tracing::info!(
                    "Background AV '{}' spawned; its acb_bridge should be launched with \
                     vehicle_name:={} in ROS domain {:?}, and its pilot routed to {:?}",
                    av.role_name,
                    av.role_name,
                    av.ros_domain_id,
                    av.goal_pose.map(|p| (p.x, p.y))
                );
                // Nothing in this bridge drives it. Without its own stack running it parks
                // at the spawn pose, and a scenario ego in that lane stops behind it --
                // which reads as a planner stall (roadmap 014, town01_pedestrian).
                tracing::warn!(
                    "Background AV '{}' spawned at ({:.1}, {:.1}, yaw {:.0} deg): an undriven \
                     background AV blocks the lane it is spawned in. If no Autoware stack is \
                     running in ROS domain {:?} for it, restart the bridge without {} \
                     (background AVs are opt-in).",
                    av.role_name,
                    av.spawn_pose.x,
                    av.spawn_pose.y,
                    av.spawn_pose.yaw,
                    av.ros_domain_id,
                    crate::config::BACKGROUND_AVS_ENV
                );
            } else {
                tracing::error!(
                    "Background AV '{}' failed to spawn: {}. The scenario continues without it.",
                    av.role_name,
                    result.description
                );
            }
        }

        tracing::info!("{spawned}/{} background AV(s) spawned", avs.len());
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
        let world = client.world().map_err(|e| eyre::eyre!("get world: {e}"))?;

        self.client = client;
        self.world = world;
        self.consecutive_carla_failures = 0;

        // Whatever CARLA we are now talking to, it is not holding our settings.
        self.sync_mode_enabled = false;
        // A restarted server froze nothing for us; a surviving one is no longer known to
        // hold our freeze either. Either way we are not the one who owes an unfreeze.
        self.froze_traffic_lights.mark_restored();
        self.held_light_timings.clear();

        tracing::info!("Reconnected to CARLA; synchronous mode will be re-applied next frame");
        Ok(())
    }

    pub fn initialize(&mut self, req: api::InitializeRequest) -> api::InitializeResponse {
        tracing::info!(
            "Initialize: step_time={}, realtime_factor={}",
            req.step_time,
            req.realtime_factor
        );

        self.step_time = req.step_time;
        self.update_ticks_per_frame();

        // Deliberately NOT enabling synchronous mode here -- see FrameAction. CARLA must
        // keep free-running until the ego exists, or acb_bridge can never discover it.
        //
        // Clearing the flag is not enough: it only records what *we* believe. CARLA can
        // already be in synchronous mode -- left there by a previous scenario in this
        // process, or by a run that died before restoring it -- and then the bridge thinks
        // it is async while CARLA is frozen, waiting for a tick that will not come until an
        // ego exists. That is the gap 1 deadlock returning through the back door, and it is
        // exactly what a second scenario run hits. So assert the state rather than assume it.
        self.sync_mode_enabled = false;
        self.has_ego = false;

        // The reconnect logic is driven by consecutive tick failures, but a CARLA that
        // died between scenarios never gets ticked -- the stale client then poisons every
        // CARLA call this Initialize makes, and SSv2 sees failures until the bridge is
        // restarted by hand. Probe the connection first and rebuild it if it is gone;
        // the server may have been restarted and be perfectly healthy.
        if self.world.settings().is_err() {
            tracing::warn!("CARLA connection looks dead at Initialize; reconnecting");
            if let Err(e) = self.reconnect_carla() {
                return api::InitializeResponse {
                    result: Some(proto_err(format!(
                        "CARLA is unreachable and reconnecting failed: {e}"
                    ))),
                    simulation_time_ns: 0,
                };
            }
        }

        self.force_async_mode();

        // Fresh run, fresh warnings -- otherwise a second scenario in one process would
        // stay quiet about problems it also has.
        self.warned_traffic_lights = false;

        // Jerk is differenced across frames; carrying last run's samples into this one
        // would report a spurious spike on the first frame.
        self.previous_longitudinal_accel.clear();
        self.settle_frames.clear();
        self.accel_history.clear();
        self.walker_height.clear();
        self.collision_monitor.reset_frames();

        // Destroy the previous run's actors BEFORE dropping the name mappings. Clearing
        // EntityManager first is what used to orphan them: the map was the only record of
        // what to clean up, so emptying it leaked every actor it referenced, and those
        // actors then blocked the spawn points this run needs.
        let report = self.destroy_all_spawned();
        if report.attempted > 0 {
            tracing::info!(
                "Initialize: cleaned up {} actor(s) from the previous run",
                report.destroyed
            );
        }
        self.entities.clear();

        // Load the town the scenario was authored against. A mismatch is not a visible
        // failure -- every pose would be interpreted against whatever town CARLA happened
        // to hold -- so an unresolvable path fails Initialize rather than guessing.
        if let Err(e) = self.load_scenario_map(&req.lanelet2_map_path) {
            return api::InitializeResponse {
                result: Some(proto_err(format!("Cannot prepare the map: {e}"))),
                simulation_time_ns: 0,
            };
        }

        // Take authority over the signals (invariant 3).
        self.freeze_traffic_lights();

        // A previous bridge process may have died without tearing its vehicles down, and
        // they sit on the spawn points this run needs. Reap them after the map is settled
        // (a reload would have cleared them anyway) and before anything spawns.
        self.reap_orphaned_vehicles();
        // Separately, and not from the tail of the call above: that function returns early
        // when it finds no orphaned vehicles, which is the common case, so anything appended
        // to it never runs.
        self.reap_parentless_sensors();

        // Background AVs go in after the map (a reload would destroy them) and before the
        // ego, so their Autoware instances can be finding their vehicles while SSv2 is
        // still setting the scenario up.
        self.spawn_background_avs();

        tracing::info!(
            "Initialized (step_time={}). CARLA left in async mode until the ego is spawned.",
            req.step_time
        );
        // CARLA is free-running here, so this is the time at the reply, not a frame SSv2
        // will step from; the first UpdateFrame reports the next one.
        api::InitializeResponse {
            result: Some(proto_ok()),
            simulation_time_ns: self.current_simulation_time_ns(),
        }
    }

    /// Force CARLA into asynchronous mode, whatever it was in before.
    ///
    /// Unlike [`restore_async_mode`](Self::restore_async_mode), this does not check whether
    /// *this* bridge enabled sync mode. At `Initialize` the bridge is taking the world for a
    /// scenario, and the state CARLA happens to be in — possibly left synchronous by a run
    /// that died — is not something to inherit.
    fn force_async_mode(&mut self) {
        let settings = match self.world.settings() {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!("Could not read CARLA settings to force async mode: {e}");
                return;
            }
        };

        if !settings.synchronous_mode {
            return;
        }

        tracing::info!("CARLA was left in synchronous mode; forcing async for startup");
        let mut settings = settings;
        settings.synchronous_mode = false;
        settings.fixed_delta_seconds = None;
        if let Err(e) = self
            .world
            .apply_settings(&settings, Duration::from_secs(10))
        {
            // Not fatal here, but say so plainly: acb_bridge polls for its vehicle and needs
            // CARLA advancing, so a stuck synchronous world will stall startup.
            tracing::error!(
                "Failed to force CARLA back to async mode ({e}). Startup may deadlock: \
                 acb_bridge cannot discover its vehicle while CARLA is frozen."
            );
        }
    }

    /// Whether an ego exists in this session. The idle watchdog uses it to tell a
    /// finished scenario from one that is merely paused: SSv2 pauses routinely, for
    /// `initialize_duration` alone, and a pause with a live ego must keep sync mode.
    pub fn has_ego(&self) -> bool {
        self.has_ego
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

    /// Whether this bridge currently holds CARLA in synchronous mode.
    ///
    /// The server loop uses this to notice a session that has gone away while sync mode is
    /// still on, which leaves CARLA frozen with nobody ticking it.
    pub fn sync_mode_enabled(&self) -> bool {
        self.sync_mode_enabled
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
        self.world
            .apply_settings(&settings, Duration::from_secs(10))
            .wrap_err("apply settings")?;
        self.sync_mode_enabled = true;
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
    fn tick_frame(&mut self) -> std::result::Result<i64, carla::CarlaError> {
        let frame = tick_then_read(
            &mut self.world,
            self.substeps,
            |w| w.tick().map(|_| ()),
            read_frame_of,
        )?;
        Ok(match frame {
            Ok((elapsed, delta)) => self.episode_clock.observe(elapsed, delta),
            Err(e) => {
                tracing::warn!("Could not read CARLA's time after the frame; reporting 0: {e}");
                0
            }
        })
    }

    pub fn update_frame(&mut self, _req: api::UpdateFrameRequest) -> api::UpdateFrameResponse {
        // Stamp this frame's collisions with this frame's number, and log last frame's.
        self.collision_monitor.next_frame();
        self.drain_collisions();

        match decide_frame_action(self.sync_mode_enabled, self.has_ego) {
            FrameAction::WaitForEgo => {
                // CARLA is still free-running so acb_bridge can find its vehicle. Ticking
                // here would be meaningless in async mode, so just acknowledge the frame.
                tracing::debug!("UpdateFrame before ego spawn: staying async, not ticking");
                // Still report a time, so SSv2 has one from its first frame.
                return api::UpdateFrameResponse {
                    result: Some(proto_ok()),
                    simulation_time_ns: self.current_simulation_time_ns(),
                };
            }
            FrameAction::EnableSyncThenTick => {
                if let Err(e) = self.enable_sync_mode() {
                    return api::UpdateFrameResponse {
                        result: Some(proto_err(format!("Failed to enable sync mode: {e}"))),
                        simulation_time_ns: 0,
                    };
                }
            }
            FrameAction::Tick => {}
        }

        let simulation_time_ns = match self.tick_frame() {
            Ok(t) => t,
            Err(e) => {
                tracing::error!("world.tick() failed: {e}");

                // A tick failure is the bridge's most reliable connection signal: SSv2 drives
                // frames continuously, so repeated failures here mean CARLA is gone rather
                // than merely idle.
                if self.note_carla_failure() {
                    if let Err(re) = self.reconnect_carla() {
                        tracing::error!("CARLA reconnection failed: {re}");
                    }
                }

                return api::UpdateFrameResponse {
                    result: Some(proto_err(format!("tick failed: {e}"))),
                    simulation_time_ns: 0,
                };
            }
        };

        self.note_carla_ok();
        api::UpdateFrameResponse {
            result: Some(proto_ok()),
            simulation_time_ns,
        }
    }

    pub fn update_step_time(
        &mut self,
        req: api::UpdateStepTimeRequest,
    ) -> api::UpdateStepTimeResponse {
        self.step_time = req.simulation_step_time;
        self.update_ticks_per_frame();

        // Before sync mode is on, just remember the value. Applying fixed_delta_seconds
        // while still async would pin CARLA to a fixed timestep without a ticker, which
        // is not what "async until the ego spawns" means. enable_sync_mode() picks up
        // self.step_time when it runs.
        if !self.sync_mode_enabled {
            tracing::info!(
                "Recorded step_time={} (applies when sync mode is enabled)",
                req.simulation_step_time
            );
            return api::UpdateStepTimeResponse {
                result: Some(proto_ok()),
            };
        }

        let mut settings = match self.world.settings() {
            Ok(s) => s,
            Err(e) => {
                return api::UpdateStepTimeResponse {
                    result: Some(proto_err(format!("Failed to get settings: {e}"))),
                };
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
        if let Err(e) = self
            .world
            .apply_settings(&settings, Duration::from_secs(10))
        {
            return api::UpdateStepTimeResponse {
                result: Some(proto_err(format!("Failed to apply settings: {e}"))),
            };
        }

        api::UpdateStepTimeResponse {
            result: Some(proto_ok()),
        }
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
        origin_offset: OriginOffset,
        kind: SpawnKind,
        role_name: Option<&str>,
    ) -> ProtoResult {
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
            Some(pose) => ros_pose_to_carla_transform(pose, origin_offset),
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

        // Resolve the blueprint once up front so the retry loop does not re-probe CARLA.
        // The configured blueprint_map takes first refusal: SSv2 asset keys are not CARLA
        // blueprint names in general, and this is where an operator states the translation.
        let mapped = self.config.blueprint_for(asset_key).map(str::to_owned);
        let requested_key = match (mapped.as_deref(), asset_key.is_empty()) {
            (Some(mapped), _) => {
                tracing::debug!("Asset key '{asset_key}' mapped to blueprint '{mapped}'");
                mapped
            }
            (None, true) => kind.default_blueprint(),
            (None, false) => asset_key,
        };
        let blueprint_key = match self.resolve_blueprint_key(requested_key, kind) {
            Ok(k) => k,
            Err(e) => return proto_err(format!("Cannot spawn '{name}': {e}")),
        };

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
        // point is the usual cause; teardown now prevents most of those, and lifting clears
        // the rest.
        //
        // ActorBuilder::spawn consumes the builder, so each attempt builds a fresh one.
        let base_z = carla_transform.location.z;
        let base_x = carla_transform.location.x;
        let base_y = carla_transform.location.y;
        let mut spawn_loc = carla_transform.location;
        let mut last_error: Option<String> = None;
        let mut actor = None;

        for (attempt, z) in spawn_retry_heights(base_z).enumerate() {
            let builder = match self.build_actor(&blueprint_key, role_name) {
                Ok(b) => b,
                Err(e) => return proto_err(format!("Cannot build '{name}': {e}")),
            };

            let mut candidate = carla_transform.clone();
            candidate.location.z = z;
            let candidate_loc = candidate.location;

            match builder.spawn(candidate) {
                Ok(a) => {
                    if attempt > 0 {
                        // Lifting is meant for ground that sits higher than the pose says.
                        // If something is parked on the commanded point instead, the lift
                        // has put this vehicle on top of it, and the run needs to know which
                        // actor rather than just that the first attempts failed.
                        match self.vehicle_occupying(base_x, base_y, base_z) {
                            Some((other_id, other_role)) => tracing::warn!(
                                "Spawned '{name}' on attempt {} at z={z:.2} (commanded \
                                 z={base_z:.2}) because actor {other_id} ('{other_role}') is \
                                 already on that point -- '{name}' is now stacked above it",
                                attempt + 1
                            ),
                            None => tracing::info!(
                                "Spawned '{name}' on attempt {} at z={z:.2} (commanded z={base_z:.2})",
                                attempt + 1
                            ),
                        }
                    }
                    spawn_loc = candidate_loc;
                    actor = Some(a);
                    break;
                }
                Err(e) => {
                    tracing::warn!(
                        "Spawn of '{name}' at z={z:.2} failed (attempt {}/{}): {e}",
                        attempt + 1,
                        SPAWN_RETRIES + 1
                    );
                    last_error = Some(e.to_string());
                }
            }
        }

        let actor = match actor {
            Some(a) => a,
            None => {
                let detail = last_error.unwrap_or_else(|| "no attempts were made".to_string());
                return proto_err(format!(
                    "Failed to spawn {} '{name}' (blueprint '{blueprint_key}') at \
                     CARLA({base_x:.1}, {base_y:.1}, {base_z:.1}) after {} attempts at \
                     increasing height; last error: {detail}. A leftover actor may be \
                     occupying the spawn point.",
                    kind.label(),
                    SPAWN_RETRIES + 1
                ));
            }
        };

        let actor_id = actor.id();
        // Record for teardown before anything else can fail. This is the only durable
        // record of what to clean up; EntityManager is emptied on despawn and re-init.
        self.record_spawned(actor_id);

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
        if !kind.physics_driven() {
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

        // Wait one tick for the actor to be fully initialized. Only meaningful once we
        // own the tick -- before sync mode is on, CARLA is advancing by itself.
        //
        // SAFETY: a failure here is not fatal to the spawn. The actor exists; it is merely
        // not yet settled. A genuinely broken connection is caught by update_frame, which
        // owns reconnection.
        if self.sync_mode_enabled {
            if let Err(e) = self.world.tick() {
                tracing::warn!("Tick after spawning '{name}' failed: {e}");
            }
        }

        // Background AVs are deliberately absent from EntityManager -- see
        // SpawnKind::entity_type. Everything else SSv2 knows about is registered.
        if let Some(entity_type) = kind.entity_type() {
            self.entities
                .insert(name.to_string(), entity_type, actor_id, origin_offset);
        }

        if kind == SpawnKind::Ego {
            // Releases the sync-mode gate: from the next UpdateFrame onward this bridge
            // owns the tick. See FrameAction.
            self.has_ego = true;
            self.collision_monitor.attach(&mut self.world, &actor, name);
            self.warm_up_localization(actor_id);
        }

        tracing::info!(
            "Spawned {} '{name}' (actor_id={actor_id}, blueprint='{blueprint_key}', \
             physics={}) at CARLA({:.1}, {:.1}, {:.1})",
            kind.label(),
            kind.physics_driven(),
            spawn_loc.x,
            spawn_loc.y,
            spawn_loc.z
        );

        proto_ok()
    }

    pub fn spawn_vehicle_entity(
        &mut self,
        req: api::SpawnVehicleEntityRequest,
    ) -> api::SpawnVehicleEntityResponse {
        let name = req
            .parameters
            .as_ref()
            .map(|p| p.name.clone())
            .unwrap_or_default();
        let kind = if req.is_ego {
            SpawnKind::Ego
        } else {
            SpawnKind::Npc
        };

        // One ego per session. A second would get the same role_name, and acb_bridge would
        // pick whichever it found first; the stock backend rejects it too.
        if let Some(reason) = second_ego_rejection(req.is_ego, self.entities.ego_name(), &name) {
            return api::SpawnVehicleEntityResponse {
                result: Some(proto_err(reason)),
            };
        }

        // Only the ego carries a role_name: acb_bridge finds its vehicle by it, and an NPC
        // tagged the same would be picked up as if it were an Autoware vehicle.
        let role_name = (kind == SpawnKind::Ego).then(|| self.config.ego.role_name.clone());
        let origin_offset = origin_offset_of(
            req.parameters
                .as_ref()
                .and_then(|p| p.bounding_box.as_ref()),
        );
        let result = self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
            origin_offset,
            kind,
            role_name.as_deref(),
        );
        api::SpawnVehicleEntityResponse {
            result: Some(result),
        }
    }

    pub fn spawn_pedestrian_entity(
        &mut self,
        req: api::SpawnPedestrianEntityRequest,
    ) -> api::SpawnPedestrianEntityResponse {
        let name = req
            .parameters
            .as_ref()
            .map(|p| p.name.clone())
            .unwrap_or_default();

        // No AI walker controller. SSv2's behaviour plugins compute the walk and send a
        // pose every frame, so a controller would be a second authority over the same
        // actor (invariant 5). The walker is spawned kinematic and teleported, exactly
        // like an NPC vehicle.
        let origin_offset = origin_offset_of(
            req.parameters
                .as_ref()
                .and_then(|p| p.bounding_box.as_ref()),
        );
        let result = self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
            origin_offset,
            SpawnKind::Pedestrian,
            None,
        );
        api::SpawnPedestrianEntityResponse {
            result: Some(result),
        }
    }

    pub fn spawn_misc_object_entity(
        &mut self,
        req: api::SpawnMiscObjectEntityRequest,
    ) -> api::SpawnMiscObjectEntityResponse {
        let name = req
            .parameters
            .as_ref()
            .map(|p| p.name.clone())
            .unwrap_or_default();

        let origin_offset = origin_offset_of(
            req.parameters
                .as_ref()
                .and_then(|p| p.bounding_box.as_ref()),
        );
        let result = self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
            origin_offset,
            SpawnKind::MiscObject,
            None,
        );
        api::SpawnMiscObjectEntityResponse {
            result: Some(result),
        }
    }

    pub fn despawn_entity(&mut self, req: api::DespawnEntityRequest) -> api::DespawnEntityResponse {
        let name = &req.name;

        // Whether this despawn removes the ego, read before the entry goes away. `has_ego`
        // gates synchronous mode and the idle watchdog's threshold, and it used to be
        // cleared only at Initialize -- so after a scenario despawned its ego the bridge
        // still believed one existed, and the watchdog waited out the long "a scenario may
        // merely be paused" timeout when the short one applied.
        let was_ego = matches!(
            self.entities.get(name).map(|e| e.entity_type),
            Some(EntityType::Ego)
        );

        if was_ego {
            // Our collision sensor before the ego it rides on; logs the run's summary.
            self.finish_collision_monitor();
        }

        match self.entities.remove(name) {
            Some(actor_id) => {
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
                        api::DespawnEntityResponse {
                            result: Some(proto_ok()),
                        }
                    }
                    Err(e) => api::DespawnEntityResponse {
                        result: Some(proto_err(format!("Failed to despawn '{name}': {e}"))),
                    },
                }
            }
            None => api::DespawnEntityResponse {
                result: Some(proto_err(format!("Entity '{name}' not found"))),
            },
        }
    }

    pub fn update_entity_status(
        &mut self,
        req: api::UpdateEntityStatusRequest,
    ) -> api::UpdateEntityStatusResponse {
        let mut updated = Vec::new();
        let mut teleport_failures: Vec<String> = Vec::new();
        let mut unknown: Vec<String> = Vec::new();

        for entity_status in &req.status {
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

            if is_ego && !req.overwrite_ego_status {
                // Read ego pose from CARLA physics
                match self.read_actor_state(actor_id, origin_offset) {
                    Some((pose, action_status)) => {
                        updated.push(api::UpdatedEntityStatus {
                            name: name.clone(),
                            action_status: Some(action_status),
                            pose: Some(pose),
                        });
                    }
                    None => {
                        // Fallback: echo what SSv2 sent
                        updated.push(api::UpdatedEntityStatus {
                            name: name.clone(),
                            action_status: entity_status.action_status.clone(),
                            pose: entity_status.pose,
                        });
                    }
                }
            } else {
                // NPC or ego overwrite: set transform from SSv2 pose
                if let Some(pose) = entity_status.pose.as_ref() {
                    let mut transform = ros_pose_to_carla_transform(pose, origin_offset);
                    if let Some(z) = self.walker_teleport_z(actor_id, &transform.location) {
                        transform.location.z = z;
                    }
                    // Pose only. SSv2's twist (action_status) is not passed on: CARLA accepts
                    // set_target_velocity and WalkerControl here, but a physics-off actor
                    // reports 0 m/s under both (measured on 0.9.16: 0.000 m/s kinematic vs
                    // 1.504 m/s for the same WalkerControl with physics on). Per-frame
                    // velocity and the walk animation need walker physics on; tracked as
                    // roadmap 014 gap 5.
                    if let Err(e) = self.set_actor_transform(actor_id, &transform) {
                        // A failed teleport used to warn and still report success, so SSv2
                        // went on believing the NPC had moved -- the same silent divergence
                        // phase 006 exists to remove.
                        tracing::warn!("set_transform for '{name}': {e}");
                        teleport_failures.push(format!("{name}: {e}"));
                    }
                }

                // Echo back the same pose. It is SSv2's own entity-origin pose, never read
                // back from CARLA, so there is no origin offset to undo here.
                updated.push(api::UpdatedEntityStatus {
                    name: name.clone(),
                    action_status: entity_status.action_status.clone(),
                    pose: entity_status.pose,
                });
            }
        }

        let result = entity_status_result(&unknown, &teleport_failures);

        api::UpdateEntityStatusResponse {
            result: Some(result),
            status: updated,
        }
    }

    pub fn attach_lidar_sensor(
        &self,
        _req: api::AttachLidarSensorRequest,
    ) -> api::AttachLidarSensorResponse {
        api::AttachLidarSensorResponse {
            result: Some(sensor_not_supported()),
        }
    }

    pub fn attach_detection_sensor(
        &self,
        _req: api::AttachDetectionSensorRequest,
    ) -> api::AttachDetectionSensorResponse {
        api::AttachDetectionSensorResponse {
            result: Some(sensor_not_supported()),
        }
    }

    pub fn attach_occupancy_grid_sensor(
        &self,
        _req: api::AttachOccupancyGridSensorRequest,
    ) -> api::AttachOccupancyGridSensorResponse {
        api::AttachOccupancyGridSensorResponse {
            result: Some(sensor_not_supported()),
        }
    }

    pub fn attach_imu_sensor(
        &self,
        _req: api::AttachImuSensorRequest,
    ) -> api::AttachImuSensorResponse {
        api::AttachImuSensorResponse {
            result: Some(sensor_not_supported()),
        }
    }

    pub fn attach_pseudo_traffic_light_detector(
        &self,
        _req: api::AttachPseudoTrafficLightDetectorRequest,
    ) -> api::AttachPseudoTrafficLightDetectorResponse {
        api::AttachPseudoTrafficLightDetectorResponse {
            result: Some(sensor_not_supported()),
        }
    }

    pub fn update_traffic_lights(
        &mut self,
        req: api::UpdateTrafficLightsRequest,
    ) -> api::UpdateTrafficLightsResponse {
        let mut applied = 0usize;
        let mut newly_unmapped = Vec::new();
        let mut failures = Vec::new();

        for signal in &req.states {
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

            match self.set_signal_state(&opendrive_id, state) {
                Ok(()) => applied += 1,
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
            api::UpdateTrafficLightsResponse {
                result: Some(proto_ok()),
            }
        } else {
            api::UpdateTrafficLightsResponse {
                result: Some(proto_err(format!(
                    "Failed to apply {} of {} traffic light state(s): {}",
                    failures.len(),
                    req.states.len(),
                    failures.join("; ")
                ))),
            }
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

        match self.world.settings() {
            Ok(mut settings) => {
                settings.synchronous_mode = false;
                settings.fixed_delta_seconds = None;
                // Hand back CARLA's own physics substepping too; the short substeps are
                // this bridge's choice for its scenario, not the shared server's.
                settings.max_substep_delta_time = CARLA_DEFAULT_MAX_SUBSTEP_DELTA;
                settings.max_substeps = CARLA_DEFAULT_MAX_SUBSTEPS;
                if let Err(e) = self
                    .world
                    .apply_settings(&settings, Duration::from_secs(10))
                {
                    tracing::warn!("Failed to restore async mode: {e}");
                } else {
                    self.sync_mode_enabled = false;
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

    /// Pick a blueprint that CARLA will actually accept, falling back when the requested
    /// one is unknown. Resolved once per spawn, before any retry.
    ///
    /// The decision itself is [`choose_blueprint`]; this only performs the probes, because
    /// `actor_builder` needs a live server and the policy is worth testing without one.
    fn resolve_blueprint_key(&mut self, requested: &str, kind: SpawnKind) -> Result<String> {
        let fallback = kind.default_blueprint();

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
                // they do not, falling back to the kind's default keeps the scenario
                // running with a warning naming both. A real asset-key mapping table is
                // config-driven and lands with phase 010.
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
    ) -> Result<carla::client::ActorBuilder<'_>> {
        let mut builder = self
            .world
            .actor_builder(blueprint_key)
            .map_err(|e| eyre::eyre!("build '{blueprint_key}': {e}"))?;

        // role_name is how acb_bridge finds the vehicle it is meant to serve.
        if let Some(role) = role_name {
            builder = builder
                .set_attribute("role_name", role)
                .map_err(|e| eyre::eyre!("set role_name: {e}"))?;
        }

        Ok(builder)
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

        let mut pose = coordinate_conversion::carla_to_ros_pose(
            t.location.x,
            t.location.y,
            t.location.z,
            t.rotation.roll,
            t.rotation.pitch,
            t.rotation.yaw,
        );

        // CARLA reports the actor origin (body centre); SSv2 expects the entity origin its
        // scenario declared (the rear axle for vehicles). Undo the spawn-time shift.
        //
        // Twist and accel are left as CARLA reports them at the body centre. Longitudinal
        // speed is the same at every point of a rigid body, but lateral velocity at the rear
        // axle differs by yaw_rate × offset.x; that is deliberately not corrected yet.
        if let Some(position) = pose.position.as_mut() {
            let ros_yaw = -(t.rotation.yaw as f64).to_radians();
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
        let yaw_rad = (t.rotation.yaw as f64).to_radians();
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

/// The origin offset an SSv2 entity declares: its `bounding_box.center` x and y. Absent or
/// zero (pedestrians and misc objects usually) means the SSv2 and CARLA origins coincide.
///
/// The declaration is trusted over CARLA's geometry. Vehicles declare 1.5 m, while
/// `vehicle.tesla.model3`'s rear axle is 1.386 m behind its origin; the 0.11 m residual is a
/// blueprint-fidelity matter (roadmap 014, gap 9), not something this shift should absorb.
fn origin_offset_of(bounding_box: Option<&BoundingBox>) -> OriginOffset {
    bounding_box
        .and_then(|b| b.center.as_ref())
        .map(|c| OriginOffset { x: c.x, y: c.y })
        .unwrap_or_default()
}

/// Convert an SSv2 pose to the CARLA transform of its actor: shifted by `origin_offset`
/// from the entity origin to the actor origin in the ROS frame, then Y-flipped.
fn ros_pose_to_carla_transform(pose: &Pose, origin_offset: OriginOffset) -> Transform {
    let mut shifted = *pose;
    if let (Some(p), Some(q)) = (shifted.position.as_mut(), pose.orientation.as_ref()) {
        let (_, _, yaw) = coordinate_conversion::quaternion_to_euler(q.x, q.y, q.z, q.w);
        (p.x, p.y) =
            coordinate_conversion::entity_origin_to_actor_origin(p.x, p.y, yaw, origin_offset);
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
            yaw: cyaw as f32,
        },
    }
}

fn proto_ok() -> ProtoResult {
    ProtoResult {
        success: true,
        description: String::new(),
    }
}

fn proto_err(description: String) -> ProtoResult {
    tracing::warn!("Returning error: {description}");
    ProtoResult {
        success: false,
        description,
    }
}

fn sensor_not_supported() -> ProtoResult {
    ProtoResult {
        success: false,
        description: "CARLA sensors provided by autoware_carla_bridge".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            SpawnKind::BackgroundAv,
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
    fn only_the_ego_is_physics_driven() {
        assert!(SpawnKind::Ego.physics_driven());
        assert!(
            SpawnKind::BackgroundAv.physics_driven(),
            "a background AV is driven by its own Autoware through CARLA physics"
        );
        assert!(!SpawnKind::Npc.physics_driven());
        assert!(!SpawnKind::Pedestrian.physics_driven());
        assert!(!SpawnKind::MiscObject.physics_driven());
    }

    /// `acb_bridge` finds its vehicle by role_name. Only the ego should carry one -- an NPC
    /// tagged `hero` would be picked up by a bridge as if it were an Autoware vehicle.
    /// A background AV is a CARLA vehicle driven by its own Autoware, not a scenario
    /// entity. Registering it would put it into UpdateEntityStatus and SSv2 would start
    /// teleporting a vehicle Autoware is already driving.
    #[test]
    fn a_background_av_is_not_an_ssv2_entity() {
        assert_eq!(SpawnKind::BackgroundAv.entity_type(), None);
        assert!(SpawnKind::Ego.entity_type().is_some());
        assert!(SpawnKind::Npc.entity_type().is_some());
    }

    #[test]
    fn each_kind_maps_to_its_entity_type() {
        assert_eq!(SpawnKind::Ego.entity_type(), Some(EntityType::Ego));
        assert_eq!(SpawnKind::Npc.entity_type(), Some(EntityType::Vehicle));
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

    /// Jerk needs a signed scalar, so acceleration is projected onto the heading. The
    /// magnitude would report braking and accelerating identically.
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
        let t = ros_pose_to_carla_transform(&pose, origin_offset_of(Some(&bbox)));
        assert!((t.location.x - 318.5).abs() < 1e-3, "{:?}", t.location);
        assert!((t.location.y - 129.8).abs() < 1e-3, "{:?}", t.location);
        assert_eq!(t.location.z, 0.0);

        let untouched = ros_pose_to_carla_transform(&pose, origin_offset_of(None));
        assert!((untouched.location.x - 320.0).abs() < 1e-3);
    }

    #[test]
    fn rejections_always_explain_themselves() {
        let result = sensor_not_supported();
        assert!(!result.success);
        assert!(!result.description.is_empty());
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
        let r = entity_status_result(&["ghost".to_string()], &[]);
        assert!(!r.success);
        assert!(r.description.contains("'ghost'"), "{}", r.description);
        let r = entity_status_result(&[], &["npc: gone".to_string()]);
        assert!(!r.success && r.description.contains("npc: gone"));
        assert!(entity_status_result(&[], &[]).success);
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
