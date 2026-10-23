//! The simulator behind the bridge (roadmaps 035, 046, `docs/design/simulator-backends.md`).
//!
//! [`SimBackend`] is everything the bridge asks of a simulator, in SSv2's terms: entities by
//! their scenario names, poses in the map frame (ROS: right-handed, metres, radians), SSv2's
//! own message types for poses, twists and signals. Nothing in it names a simulator: actor
//! ids, frame conventions, blueprints and town names stay inside the implementation. The
//! bridge's coordinator turns SSv2's protobuf requests into these calls and their results
//! back into replies.
//!
//! Since roadmap 046 the bridge has one implementation, `RemoteBackend`, which speaks the
//! world protocol (`proto/swb_world.proto`) to a simulator's world server; a world server may
//! use these types inside (the CARLA one does). [`crate::fake`] is the in-memory one the
//! coordinator's tests run on.
//!
//! One call per SSv2 request, as SSv2 sends them: `UpdateEntityStatus` carries every
//! entity's pose for the frame in one batch and gets every state back, `UpdateFrame`
//! advances the world by one step. A backend that steps over a socket therefore makes one
//! round trip per request, never one per entity.

use std::collections::BTreeMap;
use std::time::Duration;

use crate::proto::geometry_msgs::Pose;
use crate::proto::simulation_api_schema::TrafficSignal;
use crate::proto::traffic_simulator_msgs::entity_subtype::Enum as EntitySubtype;
use crate::proto::traffic_simulator_msgs::{ActionStatus, BoundingBox};
use crate::weather::WeatherReply;
use crate::weather::WeatherRequest;

/// Why a backend could not do what was asked. Every variant becomes a failed reply to SSv2
/// (roadmap 006: never a success the simulator did not deliver); the text is the reply's
/// description, as is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BackendError {
    /// The simulator has no such feature (traffic lights in an off-road world, a sensor
    /// SSv2 would attach). The text says what, and where to get it if anywhere.
    Unsupported(String),
    /// The simulator is gone; the request was not attempted.
    Disconnected(String),
    /// The simulator was asked and refused, or failed.
    Failed(String),
}

impl std::fmt::Display for BackendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unsupported(s) | Self::Disconnected(s) | Self::Failed(s) => f.write_str(s),
        }
    }
}

impl std::error::Error for BackendError {}

/// `Initialize`: a new episode on the scenario's map.
#[derive(Debug, Clone, PartialEq)]
pub struct Episode {
    pub step_time: f64,
    pub realtime_factor: f64,
    /// SSv2's map path: the backend finds its world for it (a CARLA town, a USD stage) from
    /// the map directory, as it does the map origin.
    pub lanelet2_map_path: String,
}

/// Who moves a vehicle: SSv2's rule, from `is_ego` and the entity's controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Driver {
    /// The scenario ego, driven by its Autoware through the simulator's physics.
    Ego,
    /// SSv2 computes the path and sends a pose every frame.
    Scenario,
    /// Controller `simulator_autopilot`: the simulator's own traffic drives it.
    SimulatorAutopilot,
    /// Controller `agent`: the autopilot registered under the entity's name drives it.
    Agent,
}

impl Driver {
    /// The driver of a vehicle SSv2 spawns. `behavior` is the entity's OpenSCENARIO
    /// controller name (or SSv2's plugin name).
    pub fn of_vehicle(is_ego: bool, behavior: &str) -> Self {
        if is_ego {
            Self::Ego
        } else if behavior == crate::autopilot::BEHAVIOR {
            Self::SimulatorAutopilot
        } else if behavior == crate::agent_link::BEHAVIOR {
            Self::Agent
        } else {
            Self::Scenario
        }
    }
}

/// What an entity is, with what only vehicles declare.
#[derive(Debug, Clone, PartialEq)]
pub enum EntityKind {
    Vehicle {
        driver: Driver,
        subtype: EntitySubtype,
        /// Declared front and rear axle `positionX`, m from the entity origin.
        axles: Option<(f64, f64)>,
        /// Declared `(maxAcceleration, maxDeceleration)`, m/s2.
        performance: Option<(f64, f64)>,
    },
    Pedestrian,
    MiscObject,
}

/// One entity to spawn, as SSv2 declares it.
#[derive(Debug, Clone, PartialEq)]
pub struct SpawnRequest {
    pub name: String,
    /// SSv2's asset key; the backend maps it to a body (and may ignore it).
    pub asset_key: String,
    /// Map frame, at the entity origin; `None` when SSv2 sent none.
    pub pose: Option<Pose>,
    /// The declared box, entity origin to box centre included.
    pub bounding_box: Option<BoundingBox>,
    pub kind: EntityKind,
}

/// One entity's pose and motion: what SSv2 sends for it, what the backend reports back.
#[derive(Debug, Clone, PartialEq)]
pub struct EntityStatus {
    pub name: String,
    pub action_status: Option<ActionStatus>,
    pub pose: Option<Pose>,
}

/// `UpdateEntityStatus`: every entity's pose for the coming frame.
#[derive(Debug, Clone, PartialEq)]
pub struct StatusBatch {
    pub entities: Vec<EntityStatus>,
    pub npc_logic_started: bool,
    pub overwrite_ego_status: bool,
}

/// What a backend made of a [`StatusBatch`]: each entity's state for SSv2 -- read back where
/// the simulator moves it, echoed where SSv2 does -- and, when something failed, why. SSv2
/// gets the states either way.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StatusOutcome {
    pub entities: Vec<EntityStatus>,
    pub failure: Option<String>,
}

/// `UpdateEntityGoal`, sent by the SSv2 fork for entities the simulator or an agent drives.
#[derive(Debug, Clone, PartialEq)]
pub struct Goal {
    /// Map frame, entity origin; the last is the goal.
    pub waypoints: Vec<Pose>,
    /// Drop the current route (and roam), when no waypoints come with it.
    pub clear: bool,
    /// m/s, absolute; `None` leaves the target speed as it is.
    pub target_speed: Option<f64>,
}

/// OpenSCENARIO's `RoadCondition` (roadmap 047), from SSv2's `UpdateEnvironment`: the whole
/// road state from now on, replacing the last one. `Initialize` goes back to the default.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadCondition {
    /// Scales every surface's friction, world-wide.
    pub friction_scale_factor: f64,
    /// `dry`, `moist`, `wetWithPuddles`, `lowFlooded` or `highFlooded`.
    pub wetness: String,
    /// The `Properties`, by name: `swb.*` are ours (`swb.zone.<id>.<field>`), the rest
    /// belong to other tools and are ignored.
    pub properties: BTreeMap<String, String>,
}

impl Default for RoadCondition {
    fn default() -> Self {
        Self {
            friction_scale_factor: 1.0,
            wetness: "dry".into(),
            properties: BTreeMap::new(),
        }
    }
}

impl RoadCondition {
    /// Our properties (`swb.*`): what a simulator must apply or refuse.
    pub fn swb_properties(&self) -> impl Iterator<Item = (&String, &String)> {
        self.properties
            .iter()
            .filter(|(name, _)| name.starts_with(SWB_PROPERTY_PREFIX))
    }

    /// Whether it changes nothing from the default: any simulator can take it.
    pub fn is_default(&self) -> bool {
        self.friction_scale_factor == 1.0
            && (self.wetness.is_empty() || self.wetness == "dry")
            && self.swb_properties().next().is_none()
    }
}

/// The prefix of the `RoadCondition` properties swb interprets.
pub const SWB_PROPERTY_PREFIX: &str = "swb.";

/// SSv2's `UpdateEnvironment` (OpenSCENARIO `EnvironmentAction`): `None` is unchanged.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Environment {
    pub road_condition: Option<RoadCondition>,
}

/// The sensors SSv2 can ask a simulator to attach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sensor {
    Lidar,
    Detection,
    OccupancyGrid,
    Imu,
    PseudoTrafficLightDetector,
}

/// A simulator swb drives SSv2 scenarios on. One per swb process, used from the thread that
/// serves SSv2 (`zmq_server`).
pub trait SimBackend {
    /// Once, after connecting and before the first request: apply the world state the
    /// configuration asks for (weather, the episode's clock marker, ...).
    fn before_serving(&mut self);

    /// `Initialize`: tear down the previous run, load the scenario's world, and pause on the
    /// frame the first `UpdateFrame` steps from. The simulation time, ns.
    fn initialize(&mut self, episode: &Episode) -> Result<i64, BackendError>;

    /// `UpdateFrame`: advance one step. The simulation time after it, ns (0: unknown).
    /// `current_simulation_time` is SSv2's own clock, s, for the record.
    fn update_frame(&mut self, current_simulation_time: f64) -> Result<i64, BackendError>;

    /// `UpdateStepTime`.
    fn update_step_time(&mut self, step_time: f64) -> Result<(), BackendError>;

    /// `Spawn*Entity`.
    fn spawn(&mut self, request: &SpawnRequest) -> Result<(), BackendError>;

    /// `DespawnEntity`.
    fn despawn(&mut self, name: &str) -> Result<(), BackendError>;

    /// `UpdateEntityStatus`.
    fn update_entity_status(&mut self, batch: StatusBatch) -> StatusOutcome;

    /// `UpdateEntityGoal`.
    fn update_entity_goal(&mut self, name: &str, goal: &Goal) -> Result<(), BackendError>;

    /// `UpdateEnvironment`.
    fn update_environment(&mut self, environment: &Environment) -> Result<(), BackendError>;

    /// `UpdateTrafficLights`: every signal's state, by Lanelet2 id.
    fn update_traffic_lights(&mut self, signals: &[TrafficSignal]) -> Result<(), BackendError>;

    /// `Attach*Sensor`.
    fn attach_sensor(&mut self, sensor: Sensor) -> Result<(), BackendError>;

    /// `/scenario_world_bridge/get_weather` (the world services).
    fn get_weather(&mut self) -> WeatherReply;

    /// `/scenario_world_bridge/set_weather`.
    fn set_weather(&mut self, request: WeatherRequest) -> WeatherReply;

    /// Entities SSv2 currently has spawned.
    fn entity_count(&self) -> usize;

    /// Time spent stepping the simulator since the last call, and reset it: the frame
    /// budget's processing time excludes it (`frame_stats`).
    fn take_tick_time(&mut self) -> Duration;

    /// Between scenarios, every few seconds: check the simulator is still there.
    fn probe_connection_idle(&mut self);

    /// Why the bridge cannot go on, once a simulator call never returned: the server replies
    /// to the request in flight and to the drain, then ends the process.
    fn wedged(&self) -> Option<&str>;

    /// Why the bridge exits after the idle check found the simulator gone.
    fn server_lost(&self) -> Option<&str>;

    /// The bridge ends after a call that never returned: keep what the run recorded without
    /// a word to the simulator.
    fn abandon_after_hang(&mut self);

    /// Undo what this bridge changed in the simulator. Safe to call more than once.
    fn shutdown(&mut self);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_controller_decides_who_drives_a_vehicle() {
        assert_eq!(Driver::of_vehicle(true, ""), Driver::Ego);
        // The ego is the ego whatever its controller says.
        assert_eq!(
            Driver::of_vehicle(true, crate::autopilot::BEHAVIOR),
            Driver::Ego
        );
        assert_eq!(
            Driver::of_vehicle(false, crate::autopilot::BEHAVIOR),
            Driver::SimulatorAutopilot
        );
        assert_eq!(
            Driver::of_vehicle(false, crate::agent_link::BEHAVIOR),
            Driver::Agent
        );
        assert_eq!(Driver::of_vehicle(false, ""), Driver::Scenario);
        assert_eq!(
            Driver::of_vehicle(false, "behavior_tree_plugin/VehicleBehaviorTree"),
            Driver::Scenario
        );
    }

    #[test]
    fn an_error_reads_as_its_text() {
        assert_eq!(
            BackendError::Failed("tick failed".into()).to_string(),
            "tick failed"
        );
        assert_eq!(
            BackendError::Unsupported("no lights".into()).to_string(),
            "no lights"
        );
    }
}
