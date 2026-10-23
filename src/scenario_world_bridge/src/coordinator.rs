//! SSv2's `simulation_interface` on a [`SimBackend`] (roadmap 035).
//!
//! The coordinator is the protocol side of swb and knows no simulator: it decodes each
//! protobuf request into the backend's terms ([`crate::backend`]), and turns what the
//! backend did into SSv2's reply. Every failure the backend reports -- refused, unsupported,
//! disconnected -- is a failed reply with the backend's reason, never a success (roadmap
//! 006).

use std::time::Duration;

use crate::backend::{
    BackendError, Driver, EntityKind, EntityStatus, Environment, Episode, Goal, RoadCondition,
    Sensor, SimBackend, SpawnRequest, StatusBatch,
};
use crate::proto::simulation_api_schema::{self as api, Result as ProtoResult};
use crate::proto::traffic_simulator_msgs::entity_subtype::Enum as EntitySubtype;
use crate::weather::WeatherRequest;
use crate::world_services::WeatherReply;

pub struct Coordinator {
    backend: Box<dyn SimBackend>,
}

impl Coordinator {
    pub fn new(backend: Box<dyn SimBackend>) -> Self {
        Self { backend }
    }

    pub fn initialize(&mut self, req: api::InitializeRequest) -> api::InitializeResponse {
        let episode = Episode {
            step_time: req.step_time,
            realtime_factor: req.realtime_factor,
            lanelet2_map_path: req.lanelet2_map_path,
        };
        match self.backend.initialize(&episode) {
            Ok(simulation_time_ns) => api::InitializeResponse {
                result: Some(ok()),
                simulation_time_ns,
            },
            Err(e) => api::InitializeResponse {
                result: Some(failure(e)),
                simulation_time_ns: 0,
            },
        }
    }

    pub fn update_frame(&mut self, req: api::UpdateFrameRequest) -> api::UpdateFrameResponse {
        match self.backend.update_frame(req.current_simulation_time) {
            Ok(simulation_time_ns) => api::UpdateFrameResponse {
                result: Some(ok()),
                simulation_time_ns,
            },
            Err(e) => api::UpdateFrameResponse {
                result: Some(failure(e)),
                simulation_time_ns: 0,
            },
        }
    }

    pub fn update_step_time(
        &mut self,
        req: api::UpdateStepTimeRequest,
    ) -> api::UpdateStepTimeResponse {
        api::UpdateStepTimeResponse {
            result: Some(reply(
                self.backend.update_step_time(req.simulation_step_time),
            )),
        }
    }

    pub fn spawn_vehicle_entity(
        &mut self,
        req: api::SpawnVehicleEntityRequest,
    ) -> api::SpawnVehicleEntityResponse {
        let parameters = req.parameters.as_ref();
        let request = SpawnRequest {
            name: parameters.map(|p| p.name.clone()).unwrap_or_default(),
            asset_key: req.asset_key.clone(),
            pose: req.pose,
            bounding_box: parameters.and_then(|p| p.bounding_box),
            kind: EntityKind::Vehicle {
                driver: Driver::of_vehicle(req.is_ego, &req.behavior),
                subtype: parameters
                    .and_then(|p| p.subtype.as_ref())
                    .map_or(EntitySubtype::Unknown, |s| s.value()),
                axles: parameters.and_then(|p| p.axles.as_ref()).and_then(|a| {
                    Some((
                        a.front_axle.as_ref()?.position_x,
                        a.rear_axle.as_ref()?.position_x,
                    ))
                }),
                performance: parameters
                    .and_then(|p| p.performance.as_ref())
                    .map(|p| (p.max_acceleration, p.max_deceleration)),
            },
        };
        api::SpawnVehicleEntityResponse {
            result: Some(reply(self.backend.spawn(&request))),
        }
    }

    pub fn spawn_pedestrian_entity(
        &mut self,
        req: api::SpawnPedestrianEntityRequest,
    ) -> api::SpawnPedestrianEntityResponse {
        let parameters = req.parameters.as_ref();
        let request = SpawnRequest {
            name: parameters.map(|p| p.name.clone()).unwrap_or_default(),
            asset_key: req.asset_key.clone(),
            pose: req.pose,
            bounding_box: parameters.and_then(|p| p.bounding_box),
            kind: EntityKind::Pedestrian,
        };
        api::SpawnPedestrianEntityResponse {
            result: Some(reply(self.backend.spawn(&request))),
        }
    }

    pub fn spawn_misc_object_entity(
        &mut self,
        req: api::SpawnMiscObjectEntityRequest,
    ) -> api::SpawnMiscObjectEntityResponse {
        let parameters = req.parameters.as_ref();
        let request = SpawnRequest {
            name: parameters.map(|p| p.name.clone()).unwrap_or_default(),
            asset_key: req.asset_key.clone(),
            pose: req.pose,
            bounding_box: parameters.and_then(|p| p.bounding_box),
            kind: EntityKind::MiscObject,
        };
        api::SpawnMiscObjectEntityResponse {
            result: Some(reply(self.backend.spawn(&request))),
        }
    }

    pub fn despawn_entity(&mut self, req: api::DespawnEntityRequest) -> api::DespawnEntityResponse {
        api::DespawnEntityResponse {
            result: Some(reply(self.backend.despawn(&req.name))),
        }
    }

    pub fn update_entity_status(
        &mut self,
        req: api::UpdateEntityStatusRequest,
    ) -> api::UpdateEntityStatusResponse {
        let batch = StatusBatch {
            entities: req
                .status
                .into_iter()
                .map(|s| EntityStatus {
                    name: s.name,
                    action_status: s.action_status,
                    pose: s.pose,
                })
                .collect(),
            npc_logic_started: req.npc_logic_started,
            overwrite_ego_status: req.overwrite_ego_status,
        };
        let outcome = self.backend.update_entity_status(batch);
        api::UpdateEntityStatusResponse {
            result: Some(match outcome.failure {
                None => ok(),
                Some(why) => failure(BackendError::Failed(why)),
            }),
            status: outcome
                .entities
                .into_iter()
                .map(|s| api::UpdatedEntityStatus {
                    name: s.name,
                    action_status: s.action_status,
                    pose: s.pose,
                })
                .collect(),
        }
    }

    /// `UpdateEntityGoal`, sent by the SSv2 fork for `simulator_autopilot` and `agent`
    /// entities.
    pub fn update_entity_goal(
        &mut self,
        req: api::UpdateEntityGoalRequest,
    ) -> api::UpdateEntityGoalResponse {
        let goal = Goal {
            waypoints: req.waypoints,
            clear: req.clear,
            target_speed: req.has_target_speed.then_some(req.target_speed),
        };
        api::UpdateEntityGoalResponse {
            result: Some(reply(self.backend.update_entity_goal(&req.name, &goal))),
        }
    }

    /// `UpdateEnvironment`, sent by the SSv2 fork for OpenSCENARIO's `EnvironmentAction`
    /// (roadmap 047): today its road condition.
    pub fn update_environment(
        &mut self,
        req: api::UpdateEnvironmentRequest,
    ) -> api::UpdateEnvironmentResponse {
        let outcome = environment_of(req).and_then(|env| self.backend.update_environment(&env));
        api::UpdateEnvironmentResponse {
            result: Some(reply(outcome)),
        }
    }

    pub fn update_traffic_lights(
        &mut self,
        req: api::UpdateTrafficLightsRequest,
    ) -> api::UpdateTrafficLightsResponse {
        api::UpdateTrafficLightsResponse {
            result: Some(reply(self.backend.update_traffic_lights(&req.states))),
        }
    }

    pub fn attach_lidar_sensor(
        &mut self,
        _req: api::AttachLidarSensorRequest,
    ) -> api::AttachLidarSensorResponse {
        api::AttachLidarSensorResponse {
            result: Some(self.attach(Sensor::Lidar)),
        }
    }

    pub fn attach_detection_sensor(
        &mut self,
        _req: api::AttachDetectionSensorRequest,
    ) -> api::AttachDetectionSensorResponse {
        api::AttachDetectionSensorResponse {
            result: Some(self.attach(Sensor::Detection)),
        }
    }

    pub fn attach_occupancy_grid_sensor(
        &mut self,
        _req: api::AttachOccupancyGridSensorRequest,
    ) -> api::AttachOccupancyGridSensorResponse {
        api::AttachOccupancyGridSensorResponse {
            result: Some(self.attach(Sensor::OccupancyGrid)),
        }
    }

    pub fn attach_imu_sensor(
        &mut self,
        _req: api::AttachImuSensorRequest,
    ) -> api::AttachImuSensorResponse {
        api::AttachImuSensorResponse {
            result: Some(self.attach(Sensor::Imu)),
        }
    }

    pub fn attach_pseudo_traffic_light_detector(
        &mut self,
        _req: api::AttachPseudoTrafficLightDetectorRequest,
    ) -> api::AttachPseudoTrafficLightDetectorResponse {
        api::AttachPseudoTrafficLightDetectorResponse {
            result: Some(self.attach(Sensor::PseudoTrafficLightDetector)),
        }
    }

    /// A sensor SSv2 would attach. Not logged as an error: SSv2 asks for every sensor its
    /// scenario declares, and a backend that serves sensors elsewhere refuses each.
    fn attach(&mut self, sensor: Sensor) -> ProtoResult {
        match self.backend.attach_sensor(sensor) {
            Ok(()) => ok(),
            Err(e) => ProtoResult {
                success: false,
                description: e.to_string(),
            },
        }
    }

    pub fn set_weather(&mut self, request: WeatherRequest) -> WeatherReply {
        self.backend.set_weather(request)
    }

    pub fn get_weather(&mut self) -> WeatherReply {
        self.backend.get_weather()
    }

    pub fn entity_count(&self) -> usize {
        self.backend.entity_count()
    }

    pub fn take_tick_time(&mut self) -> Duration {
        self.backend.take_tick_time()
    }

    pub fn probe_connection_idle(&mut self) {
        self.backend.probe_connection_idle();
    }

    pub fn wedged(&self) -> Option<&str> {
        self.backend.wedged()
    }

    pub fn server_lost(&self) -> Option<&str> {
        self.backend.server_lost()
    }

    pub fn abandon_after_hang(&mut self) {
        self.backend.abandon_after_hang();
    }

    pub fn shutdown(&mut self) {
        self.backend.shutdown();
    }
}

/// OpenSCENARIO's wetness levels, driest first.
pub const WETNESS: [&str; 5] = [
    "dry",
    "moist",
    "wetWithPuddles",
    "lowFlooded",
    "highFlooded",
];

fn environment_of(req: api::UpdateEnvironmentRequest) -> Result<Environment, BackendError> {
    let Some(rc) = req.road_condition else {
        return Ok(Environment::default());
    };
    let wetness = if rc.wetness.is_empty() {
        "dry".to_string()
    } else {
        rc.wetness
    };
    if !WETNESS.contains(&wetness.as_str()) {
        return Err(BackendError::Failed(format!(
            "RoadCondition: wetness {wetness:?} is not one of {}",
            WETNESS.join(", ")
        )));
    }
    let f = rc.friction_scale_factor;
    if !(f.is_finite() && f >= 0.0) {
        return Err(BackendError::Failed(format!(
            "RoadCondition: frictionScaleFactor {f} is not in [0, inf)"
        )));
    }
    Ok(Environment {
        road_condition: Some(RoadCondition {
            friction_scale_factor: f,
            wetness,
            properties: rc.properties.into_iter().collect(),
        }),
    })
}

fn ok() -> ProtoResult {
    ProtoResult {
        success: true,
        description: String::new(),
    }
}

/// A failed reply, logged: SSv2 usually ends the scenario on it.
fn failure(e: BackendError) -> ProtoResult {
    let description = e.to_string();
    tracing::warn!("Returning error: {description}");
    ProtoResult {
        success: false,
        description,
    }
}

fn reply(outcome: Result<(), BackendError>) -> ProtoResult {
    match outcome {
        Ok(()) => ok(),
        Err(e) => failure(e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::fake::{Call, FakeBackend};
    use crate::proto::geometry_msgs::{Point, Pose};
    use crate::proto::traffic_simulator_msgs::{
        self as ts, Axle, Axles, BoundingBox, EntitySubtype as Subtype, Performance,
        VehicleParameters,
    };

    fn coordinator(fake: &FakeBackend) -> Coordinator {
        Coordinator::new(Box::new(fake.clone()))
    }

    fn pose(x: f64) -> Pose {
        Pose {
            position: Some(Point { x, y: 0.0, z: 0.0 }),
            orientation: None,
        }
    }

    fn vehicle(name: &str, is_ego: bool, behavior: &str) -> api::SpawnVehicleEntityRequest {
        api::SpawnVehicleEntityRequest {
            parameters: Some(VehicleParameters {
                name: name.into(),
                subtype: Some(Subtype {
                    value: ts::entity_subtype::Enum::Truck as i32,
                }),
                bounding_box: Some(BoundingBox::default()),
                axles: Some(Axles {
                    front_axle: Some(Axle {
                        position_x: 2.8,
                        ..Default::default()
                    }),
                    rear_axle: Some(Axle {
                        position_x: 0.0,
                        ..Default::default()
                    }),
                }),
                performance: Some(Performance {
                    max_acceleration: 3.0,
                    max_deceleration: 5.0,
                    ..Default::default()
                }),
                ..Default::default()
            }),
            is_ego,
            asset_key: "vehicle.tesla.model3".into(),
            pose: Some(pose(1.0)),
            behavior: behavior.into(),
            ..Default::default()
        }
    }

    #[test]
    fn a_vehicle_spawn_carries_its_declaration_and_its_driver() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        let r = c.spawn_vehicle_entity(vehicle("npc", false, "simulator_autopilot"));
        assert!(r.result.unwrap().success);
        let Call::Spawn(request) = fake.last_call() else {
            panic!("no spawn: {:?}", fake.calls());
        };
        assert_eq!(request.name, "npc");
        assert_eq!(request.asset_key, "vehicle.tesla.model3");
        assert_eq!(request.pose, Some(pose(1.0)));
        assert_eq!(
            request.kind,
            EntityKind::Vehicle {
                driver: Driver::SimulatorAutopilot,
                subtype: ts::entity_subtype::Enum::Truck,
                axles: Some((2.8, 0.0)),
                performance: Some((3.0, 5.0)),
            }
        );
        c.spawn_vehicle_entity(vehicle("ego", true, "agent"));
        let Call::Spawn(request) = fake.last_call() else {
            panic!("no spawn");
        };
        assert!(matches!(
            request.kind,
            EntityKind::Vehicle {
                driver: Driver::Ego,
                ..
            }
        ));
    }

    #[test]
    fn walkers_and_props_spawn_as_their_kind() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        c.spawn_pedestrian_entity(api::SpawnPedestrianEntityRequest {
            parameters: Some(ts::PedestrianParameters {
                name: "walker".into(),
                ..Default::default()
            }),
            ..Default::default()
        });
        let Call::Spawn(request) = fake.last_call() else {
            panic!("no spawn");
        };
        assert_eq!(
            (request.name.as_str(), &request.kind),
            ("walker", &EntityKind::Pedestrian)
        );
        c.spawn_misc_object_entity(api::SpawnMiscObjectEntityRequest {
            parameters: Some(ts::MiscObjectParameters {
                name: "cone".into(),
                ..Default::default()
            }),
            ..Default::default()
        });
        let Call::Spawn(request) = fake.last_call() else {
            panic!("no spawn");
        };
        assert_eq!(request.kind, EntityKind::MiscObject);
    }

    /// Every way a backend can fail is a failed reply carrying its reason (roadmap 006).
    #[test]
    fn a_backend_failure_is_a_failed_reply_with_its_reason() {
        for error in [
            BackendError::Failed("refused".into()),
            BackendError::Unsupported("no traffic lights in this world".into()),
            BackendError::Disconnected("simulator gone".into()),
        ] {
            let fake = FakeBackend::failing(error.clone());
            let mut c = coordinator(&fake);
            let r = c.despawn_entity(api::DespawnEntityRequest { name: "x".into() });
            let result = r.result.unwrap();
            assert!(!result.success, "{error:?}");
            assert_eq!(result.description, error.to_string());
            let r = c.update_traffic_lights(api::UpdateTrafficLightsRequest::default());
            assert!(!r.result.unwrap().success);
            let r = c.initialize(api::InitializeRequest::default());
            assert_eq!(r.simulation_time_ns, 0);
            assert!(!r.result.unwrap().success);
            let r = c.update_frame(api::UpdateFrameRequest::default());
            assert_eq!(r.simulation_time_ns, 0);
            assert!(!r.result.unwrap().success);
        }
    }

    #[test]
    fn frames_report_the_backends_time() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        let r = c.initialize(api::InitializeRequest {
            step_time: 0.05,
            lanelet2_map_path: "/maps/Town01/lanelet2_map.osm".into(),
            ..Default::default()
        });
        assert_eq!(r.simulation_time_ns, 0);
        assert!(r.result.unwrap().success);
        assert_eq!(
            fake.calls()[0],
            Call::Initialize(Episode {
                step_time: 0.05,
                realtime_factor: 0.0,
                lanelet2_map_path: "/maps/Town01/lanelet2_map.osm".into(),
            })
        );
        let r = c.update_frame(api::UpdateFrameRequest {
            current_simulation_time: 0.05,
            ..Default::default()
        });
        assert_eq!(r.simulation_time_ns, 50_000_000);
        assert_eq!(fake.last_call(), Call::UpdateFrame(0.05));
    }

    #[test]
    fn a_sensor_refusal_is_the_backends_answer() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        let r = c.attach_imu_sensor(api::AttachImuSensorRequest::default());
        let result = r.result.unwrap();
        assert!(!result.success);
        assert_eq!(result.description, crate::backend::fake::NO_SENSORS);
        assert_eq!(fake.last_call(), Call::AttachSensor(Sensor::Imu));
    }

    #[test]
    fn a_status_update_returns_every_state_even_when_it_fails() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        c.spawn_vehicle_entity(vehicle("npc", false, ""));
        let status = |name: &str| api::EntityStatus {
            name: name.into(),
            pose: Some(pose(2.0)),
            ..Default::default()
        };
        let r = c.update_entity_status(api::UpdateEntityStatusRequest {
            status: vec![status("npc"), status("ghost")],
            npc_logic_started: true,
            overwrite_ego_status: false,
        });
        // The fake knows no 'ghost': the reply fails naming it, and still carries 'npc'.
        let result = r.result.unwrap();
        assert!(!result.success);
        assert!(
            result.description.contains("'ghost'"),
            "{}",
            result.description
        );
        assert_eq!(r.status.len(), 1);
        assert_eq!(r.status[0].name, "npc");
        assert_eq!(r.status[0].pose, Some(pose(2.0)));
        let Call::UpdateEntityStatus(batch) = fake.last_call() else {
            panic!("no status update");
        };
        assert!(batch.npc_logic_started && !batch.overwrite_ego_status);
        assert_eq!(batch.entities.len(), 2);
    }

    #[test]
    fn a_goal_without_a_target_speed_leaves_the_speed() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        c.update_entity_goal(api::UpdateEntityGoalRequest {
            name: "npc".into(),
            waypoints: vec![pose(10.0)],
            target_speed: 7.0,
            has_target_speed: false,
            clear: false,
        });
        assert_eq!(
            fake.last_call(),
            Call::UpdateEntityGoal(
                "npc".into(),
                Goal {
                    waypoints: vec![pose(10.0)],
                    clear: false,
                    target_speed: None,
                }
            )
        );
        c.update_entity_goal(api::UpdateEntityGoalRequest {
            name: "npc".into(),
            target_speed: 7.0,
            has_target_speed: true,
            ..Default::default()
        });
        let Call::UpdateEntityGoal(_, goal) = fake.last_call() else {
            panic!("no goal");
        };
        assert_eq!(goal.target_speed, Some(7.0));
    }

    fn road(f: f64, wetness: &str, props: &[(&str, &str)]) -> api::UpdateEnvironmentRequest {
        api::UpdateEnvironmentRequest {
            road_condition: Some(api::RoadCondition {
                friction_scale_factor: f,
                wetness: wetness.into(),
                properties: props
                    .iter()
                    .map(|(k, v)| (k.to_string(), v.to_string()))
                    .collect(),
            }),
        }
    }

    #[test]
    fn the_road_condition_reaches_the_backend() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        let r = c.update_environment(road(
            0.7,
            "wetWithPuddles",
            &[("swb.zone.ford.water_depth", "0.1"), ("other.tool", "x")],
        ));
        assert!(r.result.unwrap().success);
        let Call::UpdateEnvironment(env) = fake.last_call() else {
            panic!("no environment");
        };
        let rc = env.road_condition.unwrap();
        assert_eq!(rc.friction_scale_factor, 0.7);
        assert_eq!(rc.wetness, "wetWithPuddles");
        assert_eq!(rc.properties.len(), 2);
        assert_eq!(rc.swb_properties().count(), 1);
        assert!(!rc.is_default());
        // No wetness is dry; an environment without a road condition changes none.
        c.update_environment(road(1.0, "", &[("other.tool", "x")]));
        let Call::UpdateEnvironment(env) = fake.last_call() else {
            panic!("no environment");
        };
        assert!(env.road_condition.unwrap().is_default());
        c.update_environment(api::UpdateEnvironmentRequest::default());
        assert_eq!(
            fake.last_call(),
            Call::UpdateEnvironment(Environment::default())
        );
    }

    #[test]
    fn a_road_condition_ssv2_could_not_send_is_refused() {
        let fake = FakeBackend::default();
        let mut c = coordinator(&fake);
        for (req, needle) in [
            (road(1.0, "soaked", &[]), "wetness \"soaked\""),
            (road(-0.5, "dry", &[]), "frictionScaleFactor -0.5"),
            (road(f64::NAN, "dry", &[]), "frictionScaleFactor NaN"),
        ] {
            let result = c.update_environment(req).result.unwrap();
            assert!(
                !result.success && result.description.contains(needle),
                "{}",
                result.description
            );
        }
        assert!(fake.calls().is_empty());
    }
}
