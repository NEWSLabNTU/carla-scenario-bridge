use carla::client::{ActorBase, Client, World};
use carla::geom::{Location, Rotation, Transform};
use eyre::{Result, WrapErr};
use std::collections::{HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use std::time::Duration;

use crate::config::{BackgroundAv, BridgeConfig};
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

/// Frames of raw CARLA acceleration averaged before reporting. PhysX delivers
/// single-frame contact jolts (±10-17 m/s² for one 100 ms step: suspension contact,
/// curb touches, brake grab) that no real accelerometer-and-consumer chain would see
/// unfiltered. SSv2 validates reported acceleration against the scenario's declared
/// performance bounds, so one such frame is scenario-fatal. A short moving average
/// damps one-frame jolts while a genuine sustained braking profile passes through.
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

use crate::coordinate_conversion;
use crate::entity_manager::{EntityManager, EntityType};
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::simulation_api_schema::{self as api, Result as ProtoResult};
use crate::proto::traffic_simulator_msgs;

/// What an `UpdateFrame` should do, given how far startup has progressed.
///
/// CARLA synchronous mode cannot be enabled at `Initialize`. `acb_bridge` discovers its
/// vehicle by polling `world.actors()`, which needs CARLA advancing on its own; but in
/// sync mode nothing advances until someone ticks, and this bridge does not tick until
/// SSv2 sends a frame, and SSv2 sends no frame until the ego exists. Enabling sync mode
/// early deadlocks all three. So we stay async until the ego has been spawned, then
/// switch on the first frame after it.
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

fn decide_frame_action(sync_mode_enabled: bool, has_ego: bool) -> FrameAction {
    match (sync_mode_enabled, has_ego) {
        (true, _) => FrameAction::Tick,
        (false, true) => FrameAction::EnableSyncThenTick,
        (false, false) => FrameAction::WaitForEgo,
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
    /// Whether this bridge has switched CARLA into synchronous mode. Also gates the
    /// cleanup path: we must not restore async mode we never left.
    sync_mode_enabled: bool,
    /// Whether an ego vehicle has been spawned. See [`FrameAction`].
    has_ego: bool,
    /// Entity names already reported as unknown by `update_entity_status`. SSv2 sends
    /// status every frame, so without this a desync would emit one warning per frame.
    warned_unknown_entities: HashSet<String>,
    /// Whether `update_traffic_lights` has already logged its rejection. Same reason.
    warned_traffic_lights: bool,
    /// Every CARLA actor this bridge created and has not confirmed destroyed.
    ///
    /// Deliberately separate from `EntityManager`, which is a name lookup that gets
    /// emptied on despawn and on re-initialise. Tying teardown to it leaked every actor
    /// whose name mapping was dropped first.
    spawned_actors: HashSet<u32>,
    /// Whether this bridge froze CARLA's traffic lights, so shutdown only unfreezes what
    /// it actually froze. Set by phase 009 when freezing lands (invariant 3).
    froze_traffic_lights: bool,
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
    /// Directory holding per-map config, e.g. the traffic light signal mapping.
    config_dir: PathBuf,
    /// Map directory name to CARLA town, for maps whose directory is not the town name.
    map_aliases: HashMap<String, String>,
    /// Lanelet2 signal ID to OpenDRIVE sign ID for the loaded town.
    signal_map: SignalMap,
    /// Lanelet2 signal IDs already reported as unmapped. SSv2 sends states every frame, so
    /// an unmapped signal must warn once rather than per frame.
    warned_unmapped_signals: HashSet<i32>,
    /// Whether the arrow-shape limitation has been reported for this run.
    warned_arrow_shapes: bool,
    /// Bridge configuration: ego role name, blueprint map, background AVs.
    config: BridgeConfig,
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
        Self {
            map_aliases: config.map_alias.clone(),
            config,
            client,
            carla_host,
            carla_port,
            config_dir,
            signal_map: SignalMap::new(),
            warned_unmapped_signals: HashSet::new(),
            warned_arrow_shapes: false,
            world,
            entities: EntityManager::new(),
            step_time: 0.05,
            sync_mode_enabled: false,
            has_ego: false,
            warned_unknown_entities: HashSet::new(),
            warned_traffic_lights: false,
            spawned_actors: HashSet::new(),
            froze_traffic_lights: false,
            consecutive_carla_failures: 0,
            previous_longitudinal_accel: HashMap::new(),
            settle_frames: HashMap::new(),
            accel_history: HashMap::new(),
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
        self.spawned_actors.insert(actor_id);
    }

    /// Drop an actor from the teardown set once it is confirmed gone.
    fn forget_spawned(&mut self, actor_id: u32) {
        self.spawned_actors.remove(&actor_id);
        // Actor IDs are reused by CARLA, so a stale jerk sample would otherwise be
        // differenced against a completely different entity.
        self.previous_longitudinal_accel.remove(&actor_id);
        self.settle_frames.remove(&actor_id);
        self.accel_history.remove(&actor_id);
    }

    /// Destroy every actor this bridge created that is still alive.
    ///
    /// Best effort throughout: CARLA may already be gone, and an actor that has vanished on
    /// its own is a success -- the requested end state holds either way. Never panics, so it
    /// is safe on the shutdown path.
    pub fn destroy_all_spawned(&mut self) -> TeardownReport {
        let mut report = TeardownReport {
            attempted: self.spawned_actors.len(),
            ..Default::default()
        };

        if report.attempted == 0 {
            return report;
        }

        let ids: Vec<u32> = self.spawned_actors.iter().copied().collect();
        for actor_id in ids {
            match self.destroy_actor(actor_id) {
                Ok(()) => {
                    report.destroyed += 1;
                    self.forget_spawned(actor_id);
                }
                Err(e) => {
                    report.failed += 1;
                    tracing::warn!("Teardown: failed to destroy actor {actor_id}: {e}");
                }
            }
        }

        tracing::info!(
            "Teardown: {} spawned, {} destroyed, {} failed",
            report.attempted,
            report.destroyed,
            report.failed
        );
        report
    }

    /// Destroy one actor by ID. A missing actor counts as destroyed.
    fn destroy_actor(&self, actor_id: u32) -> Result<()> {
        let actor = self
            .world
            .actor(actor_id)
            .wrap_err_with(|| format!("look up actor {actor_id}"))?;

        match actor {
            Some(actor) => {
                let destroyed = actor
                    .destroy()
                    .wrap_err_with(|| format!("destroy actor {actor_id}"))?;
                if !destroyed {
                    tracing::debug!("Actor {actor_id} was already destroyed");
                }
            }
            None => tracing::debug!("Actor {actor_id} is no longer in the world"),
        }
        Ok(())
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
            let prepared = self.prepare_map(&town, &current_town);
            let _ = self.client.set_timeout(Duration::from_secs(30));
            self.world = prepared?;

            // The old world and everything in it is gone. Anything still recorded for
            // teardown refers to actors that no longer exist.
            self.spawned_actors.clear();
            self.previous_longitudinal_accel.clear();
            self.settle_frames.clear();
            self.accel_history.clear();
            self.sync_mode_enabled = false;
            self.froze_traffic_lights = false;
            tracing::info!("Map '{town}' loaded");
        }

        self.load_signal_map(&town, lanelet2_map_path)
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

    /// Load the explicit signal mapping and fill the rest by position matching.
    fn load_signal_map(&mut self, town: &str, lanelet2_map_path: &str) -> Result<()> {
        // Explicit per-map mapping first: it exists to correct what position matching gets
        // wrong, so it must take precedence over anything derived below.
        let mapping_path = self.config_dir.join(format!("traffic_lights_{town}.yaml"));
        self.signal_map = SignalMap::load_or_empty(&mapping_path)?;
        self.warned_unmapped_signals.clear();
        self.warned_arrow_shapes = false;

        self.match_signals_by_position(lanelet2_map_path);

        if self.signal_map.is_empty() {
            tracing::warn!(
                "No traffic light mapping for '{town}', automatic or explicit. Signals the \
                 scenario commands cannot be applied to CARLA. Add {} if this map has lights.",
                mapping_path.display()
            );
        }

        Ok(())
    }

    /// Fill in the signal mapping by pairing Lanelet2 traffic lights with CARLA's by position.
    ///
    /// Best-effort and never fatal: a map that cannot be read, or lights that do not pair up,
    /// leave the explicit YAML mapping as the only source. Every shortfall is reported with
    /// counts so the gap is visible before the scenario runs rather than discovered when a
    /// light fails to change.
    fn match_signals_by_position(&mut self, lanelet2_map_path: &str) {
        let map_file = lanelet_map_file(lanelet2_map_path);

        let lanelet_lights = match crate::lanelet_map::load_traffic_lights(&map_file) {
            Ok(lights) => lights,
            Err(e) => {
                tracing::warn!(
                    "Could not read traffic lights from {} ({e}); relying on the explicit \
                     mapping alone",
                    map_file.display()
                );
                return;
            }
        };

        if lanelet_lights.is_empty() {
            tracing::info!("Lanelet2 map declares no traffic lights");
            return;
        }

        let carla_signals = match self.enumerate_carla_signals() {
            Ok(signals) => signals,
            Err(e) => {
                tracing::warn!("Could not enumerate CARLA traffic lights ({e})");
                return;
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
    /// Freezing leaves each light in whatever state it happened to be in, which for a light
    /// the scenario never addresses means an arbitrary but *fixed* state for the whole run.
    /// That is the intended trade: a cycling light near the ego's route would make the run
    /// non-deterministic, which is what invariant 3 exists to prevent. A scenario that cares
    /// about a signal must command it.
    fn freeze_traffic_lights(&mut self) {
        match self.world.freeze_all_traffic_lights(true) {
            Ok(()) => {
                self.froze_traffic_lights = true;
                tracing::info!(
                    "CARLA traffic light cycling frozen; SSv2 is now the only writer. \
                     Signals the scenario does not command hold their current state."
                );
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

    /// Unfreeze CARLA's traffic lights, but only if this bridge froze them.
    ///
    /// Freezing arrives with phase 009. The guard is here now so that landing the freeze
    /// cannot leave a shared CARLA server stuck on whatever states the last scenario left.
    pub fn restore_traffic_lights(&mut self) {
        if !self.froze_traffic_lights {
            return;
        }

        match self.world.freeze_all_traffic_lights(false) {
            Ok(()) => {
                self.froze_traffic_lights = false;
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
        self.froze_traffic_lights = false;

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
                };
            }
        }

        self.force_async_mode();

        // Fresh run, fresh warnings -- otherwise a second scenario in one process would
        // stay quiet about problems it also has.
        self.warned_unknown_entities.clear();
        self.warned_traffic_lights = false;

        // Jerk is differenced across frames; carrying last run's samples into this one
        // would report a spurious spike on the first frame.
        self.previous_longitudinal_accel.clear();
        self.settle_frames.clear();
        self.accel_history.clear();

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
            };
        }

        // Take authority over the signals (invariant 3).
        self.freeze_traffic_lights();

        // Background AVs go in after the map (a reload would destroy them) and before the
        // ego, so their Autoware instances can be finding their vehicles while SSv2 is
        // still setting the scenario up.
        self.spawn_background_avs();

        tracing::info!(
            "Initialized (step_time={}). CARLA left in async mode until the ego is spawned.",
            req.step_time
        );
        api::InitializeResponse {
            result: Some(proto_ok()),
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

    /// Switch CARLA into synchronous mode at `self.step_time`.
    fn enable_sync_mode(&mut self) -> Result<()> {
        let mut settings = self.world.settings().wrap_err("get settings")?;
        settings.synchronous_mode = true;
        settings.fixed_delta_seconds = Some(self.step_time);
        self.world
            .apply_settings(&settings, Duration::from_secs(10))
            .wrap_err("apply settings")?;
        self.sync_mode_enabled = true;
        tracing::info!(
            "CARLA sync mode enabled, fixed_delta_seconds={}",
            self.step_time
        );
        Ok(())
    }

    pub fn update_frame(&mut self, _req: api::UpdateFrameRequest) -> api::UpdateFrameResponse {
        match decide_frame_action(self.sync_mode_enabled, self.has_ego) {
            FrameAction::WaitForEgo => {
                // CARLA is still free-running so acb_bridge can find its vehicle. Ticking
                // here would be meaningless in async mode, so just acknowledge the frame.
                tracing::debug!("UpdateFrame before ego spawn: staying async, not ticking");
                return api::UpdateFrameResponse {
                    result: Some(proto_ok()),
                };
            }
            FrameAction::EnableSyncThenTick => {
                if let Err(e) = self.enable_sync_mode() {
                    return api::UpdateFrameResponse {
                        result: Some(proto_err(format!("Failed to enable sync mode: {e}"))),
                    };
                }
            }
            FrameAction::Tick => {}
        }

        if let Err(e) = self.world.tick() {
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
            };
        }

        self.note_carla_ok();
        api::UpdateFrameResponse {
            result: Some(proto_ok()),
        }
    }

    pub fn update_step_time(
        &mut self,
        req: api::UpdateStepTimeRequest,
    ) -> api::UpdateStepTimeResponse {
        self.step_time = req.simulation_step_time;

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

        settings.fixed_delta_seconds = Some(req.simulation_step_time);
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
        kind: SpawnKind,
        role_name: Option<&str>,
    ) -> ProtoResult {
        tracing::info!("Spawn {}: name={name}, asset_key={asset_key}", kind.label());

        // Convert pose from ROS to CARLA frame
        let mut carla_transform = match pose {
            Some(pose) => ros_pose_to_carla_transform(pose),
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
                        tracing::info!(
                            "Spawned '{name}' on attempt {} at z={z:.2} (commanded z={base_z:.2})",
                            attempt + 1
                        );
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
                .insert(name.to_string(), entity_type, actor_id);
        }

        if kind == SpawnKind::Ego {
            // Releases the sync-mode gate: from the next UpdateFrame onward this bridge
            // owns the tick. See FrameAction.
            self.has_ego = true;
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

        // Only the ego carries a role_name: acb_bridge finds its vehicle by it, and an NPC
        // tagged the same would be picked up as if it were an Autoware vehicle.
        let role_name = (kind == SpawnKind::Ego).then(|| self.config.ego.role_name.clone());
        let result = self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
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
        let result = self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
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

        let result = self.spawn_entity(
            &name,
            &req.asset_key,
            req.pose.as_ref(),
            SpawnKind::MiscObject,
            None,
        );
        api::SpawnMiscObjectEntityResponse {
            result: Some(result),
        }
    }

    pub fn despawn_entity(&mut self, req: api::DespawnEntityRequest) -> api::DespawnEntityResponse {
        let name = &req.name;

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
                            if actor.destroy().wrap_err("destroy actor")? {
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

        for entity_status in &req.status {
            let name = &entity_status.name;

            // Copy out what we need so the immutable borrow of `entities` ends here --
            // the unknown-entity path below needs `&mut self` for the warn-once set.
            let entity_info = self
                .entities
                .get(name)
                .map(|e| (e.carla_actor_id, e.entity_type == EntityType::Ego));

            let (actor_id, is_ego) = match entity_info {
                Some(v) => v,
                None => {
                    // Echo the requested pose back unchanged.
                    //
                    // This is deliberately non-fatal but no longer silent. An unknown
                    // entity means SSv2 and this bridge disagree about what exists, and
                    // echoing hides that: SSv2 receives exactly the pose it asked for and
                    // concludes the entity is tracking its scripted path. That is how the
                    // pedestrian stub used to produce passing scenarios with no pedestrian.
                    //
                    // Failing the whole request instead was considered and rejected: a
                    // status arriving for an entity mid-despawn would abort an otherwise
                    // healthy scenario. With spawn now rejecting loudly (phase 006), SSv2
                    // aborts at spawn time, so reaching here is already exceptional --
                    // worth a warning, not a scenario failure.
                    //
                    // Warns once per entity name; SSv2 sends status every frame.
                    if self.warned_unknown_entities.insert(name.clone()) {
                        tracing::warn!(
                            "UpdateEntityStatus for unknown entity '{name}': echoing the \
                             requested pose back. SSv2 and the bridge disagree about which \
                             entities exist; this entity is not present in CARLA."
                        );
                    }
                    updated.push(api::UpdatedEntityStatus {
                        name: name.clone(),
                        action_status: entity_status.action_status.clone(),
                        pose: entity_status.pose,
                    });
                    continue;
                }
            };

            if is_ego && !req.overwrite_ego_status {
                // Read ego pose from CARLA physics
                match self.read_actor_state(actor_id) {
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
                    let transform = ros_pose_to_carla_transform(pose);
                    if let Err(e) = self.set_actor_transform(actor_id, &transform) {
                        // A failed teleport used to warn and still report success, so SSv2
                        // went on believing the NPC had moved -- the same silent divergence
                        // phase 006 exists to remove.
                        tracing::warn!("set_transform for '{name}': {e}");
                        teleport_failures.push(format!("{name}: {e}"));
                    }
                }

                // Echo back the same pose
                updated.push(api::UpdatedEntityStatus {
                    name: name.clone(),
                    action_status: entity_status.action_status.clone(),
                    pose: entity_status.pose,
                });
            }
        }

        let result = if teleport_failures.is_empty() {
            proto_ok()
        } else {
            proto_err(format!(
                "Failed to apply the commanded pose to {} entit{} ({})",
                teleport_failures.len(),
                if teleport_failures.len() == 1 {
                    "y"
                } else {
                    "ies"
                },
                teleport_failures.join("; ")
            ))
        };

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
    fn resolve_blueprint_key(&mut self, requested: &str, kind: SpawnKind) -> Result<String> {
        let fallback = kind.default_blueprint();

        // The probe builder is dropped at the end of this statement, releasing the borrow
        // on `world` before the next one is taken.
        if self.world.actor_builder(requested).is_ok() {
            return Ok(requested.to_string());
        }

        // SSv2 asset keys are not CARLA blueprint names in general. Where they happen to
        // coincide this passes straight through; where they do not, falling back to the
        // kind's default keeps the scenario running with a warning naming both. A real
        // asset-key mapping table is config-driven and lands with phase 010.
        tracing::warn!(
            "Blueprint '{requested}' is not a CARLA blueprint; falling back to '{fallback}' \
             for this {}",
            kind.label()
        );

        self.world.actor_builder(fallback).map_err(|e| {
            eyre::eyre!("neither '{requested}' nor '{fallback}' is a valid blueprint: {e}")
        })?;

        Ok(fallback.to_string())
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
    ) -> Option<(Pose, traffic_simulator_msgs::ActionStatus)> {
        let actors = self.world.actors().ok()?;
        let actor = actors.find(actor_id).ok()??;

        let t = actor.transform().ok()?;
        let v = actor.velocity().ok()?;
        let av = actor.angular_velocity().ok()?;
        let acc = actor.acceleration().ok()?;

        let pose = coordinate_conversion::carla_to_ros_pose(
            t.location.x,
            t.location.y,
            t.location.z,
            t.rotation.roll,
            t.rotation.pitch,
            t.rotation.yaw,
        );

        let (vx, vy, vz) = coordinate_conversion::carla_to_ros_velocity(v.x, v.y, v.z);
        let (wx, wy, wz) = coordinate_conversion::carla_to_ros_angular_velocity(av.x, av.y, av.z);
        let (ax, ay, az) = coordinate_conversion::carla_to_ros_acceleration(acc.x, acc.y, acc.z);

        // Smooth the reported acceleration (see ACCEL_SMOOTHING_FRAMES).
        let hist = self.accel_history.entry(actor_id).or_default();
        hist.push_back((ax, ay, az));
        if hist.len() > ACCEL_SMOOTHING_FRAMES {
            hist.pop_front();
        }
        let n = hist.len() as f64;
        let (ax, ay, az) = hist
            .iter()
            .fold((0.0, 0.0, 0.0), |(sx, sy, sz), (x, y, z)| {
                (sx + x, sy + y, sz + z)
            });
        let (ax, ay, az) = (ax / n, ay / n, az / n);

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

fn ros_pose_to_carla_transform(pose: &Pose) -> Transform {
    let (cx, cy, cz, cr, cp, cyaw) = coordinate_conversion::ros_pose_to_carla(pose);
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
        let report = TeardownReport::default();
        assert_eq!(report.attempted, 0);
        assert_eq!(report.destroyed, 0);
        assert_eq!(report.failed, 0);
    }

    /// Whatever the message says, it must never be empty -- SSv2 surfaces this text and a
    /// bare failure with no reason is what phase 006 exists to eliminate.
    #[test]
    fn rejections_always_explain_themselves() {
        let result = sensor_not_supported();
        assert!(!result.success);
        assert!(!result.description.is_empty());
    }
}
