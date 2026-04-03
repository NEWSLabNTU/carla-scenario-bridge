use carla::client::{ActorBase, World};
use carla::geom::{Location, Rotation, Transform};
use eyre::{Result, WrapErr};
use std::time::Duration;

use crate::coordinate_conversion;
use crate::entity_manager::{EntityManager, EntityType};
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::simulation_api_schema::{self as api, Result as ProtoResult};
use crate::proto::traffic_simulator_msgs;

pub struct Coordinator {
    world: World,
    entities: EntityManager,
    step_time: f64,
}

impl Coordinator {
    pub fn new(world: World) -> Self {
        Self {
            world,
            entities: EntityManager::new(),
            step_time: 0.05,
        }
    }

    pub fn initialize(&mut self, req: api::InitializeRequest) -> api::InitializeResponse {
        tracing::info!(
            "Initialize: step_time={}, realtime_factor={}",
            req.step_time,
            req.realtime_factor
        );

        self.step_time = req.step_time;

        let mut settings = match self.world.settings() {
            Ok(s) => s,
            Err(e) => {
                return api::InitializeResponse {
                    result: Some(proto_err(format!("Failed to get settings: {e}"))),
                };
            }
        };

        settings.synchronous_mode = true;
        settings.fixed_delta_seconds = Some(req.step_time);

        if let Err(e) = self.world.apply_settings(&settings, Duration::from_secs(10)) {
            return api::InitializeResponse {
                result: Some(proto_err(format!("Failed to apply settings: {e}"))),
            };
        }

        // Clear any entities from previous runs
        self.entities.clear();

        tracing::info!("CARLA sync mode enabled, fixed_delta_seconds={}", req.step_time);
        api::InitializeResponse {
            result: Some(proto_ok()),
        }
    }

    pub fn update_frame(&mut self, _req: api::UpdateFrameRequest) -> api::UpdateFrameResponse {
        if let Err(e) = self.world.tick() {
            tracing::error!("world.tick() failed: {e}");
            return api::UpdateFrameResponse {
                result: Some(proto_err(format!("tick failed: {e}"))),
            };
        }

        api::UpdateFrameResponse {
            result: Some(proto_ok()),
        }
    }

    pub fn update_step_time(
        &mut self,
        req: api::UpdateStepTimeRequest,
    ) -> api::UpdateStepTimeResponse {
        self.step_time = req.simulation_step_time;

        let mut settings = match self.world.settings() {
            Ok(s) => s,
            Err(e) => {
                return api::UpdateStepTimeResponse {
                    result: Some(proto_err(format!("Failed to get settings: {e}"))),
                };
            }
        };

        settings.fixed_delta_seconds = Some(req.simulation_step_time);
        if let Err(e) = self.world.apply_settings(&settings, Duration::from_secs(10)) {
            return api::UpdateStepTimeResponse {
                result: Some(proto_err(format!("Failed to apply settings: {e}"))),
            };
        }

        api::UpdateStepTimeResponse {
            result: Some(proto_ok()),
        }
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
        let is_ego = req.is_ego;
        let asset_key = &req.asset_key;

        tracing::info!(
            "SpawnVehicle: name={name}, is_ego={is_ego}, asset_key={asset_key}"
        );

        // Convert pose from ROS to CARLA frame
        let carla_transform = match req.pose.as_ref() {
            Some(pose) => ros_pose_to_carla_transform(pose),
            None => Transform {
                location: Location { x: 0.0, y: 0.0, z: 0.0 },
                rotation: Rotation { roll: 0.0, pitch: 0.0, yaw: 0.0 },
            },
        };

        // Determine blueprint name
        let blueprint_key = if asset_key.is_empty() {
            "vehicle.tesla.model3"
        } else {
            asset_key.as_str()
        };

        // Spawn using actor_builder
        let mut builder = match self.world.actor_builder(blueprint_key) {
            Ok(b) => b,
            Err(e) => {
                // Try a fallback blueprint
                tracing::warn!("Blueprint '{blueprint_key}' not found: {e}, trying vehicle.tesla.model3");
                match self.world.actor_builder("vehicle.tesla.model3") {
                    Ok(b) => b,
                    Err(e2) => {
                        return api::SpawnVehicleEntityResponse {
                            result: Some(proto_err(format!(
                                "No valid blueprint: {e2}"
                            ))),
                        };
                    }
                }
            }
        };

        // Set role_name for ego so autoware_carla_bridge can find it
        if is_ego {
            match builder.set_attribute("role_name", "hero") {
                Ok(b) => builder = b,
                Err(e) => {
                    return api::SpawnVehicleEntityResponse {
                        result: Some(proto_err(format!("Failed to set role_name: {e}"))),
                    };
                }
            }
        }

        let spawn_loc = carla_transform.location;
        let actor = match builder.spawn(carla_transform) {
            Ok(a) => a,
            Err(e) => {
                return api::SpawnVehicleEntityResponse {
                    result: Some(proto_err(format!("spawn failed: {e}"))),
                };
            }
        };

        // Wait one tick for actor to be fully initialized in sync mode
        let _ = self.world.tick();

        let actor_id = actor.id();
        let entity_type = if is_ego {
            EntityType::Ego
        } else {
            EntityType::Vehicle
        };
        self.entities.insert(name.clone(), entity_type, actor_id);

        tracing::info!(
            "Spawned vehicle '{name}' (actor_id={actor_id}, is_ego={is_ego}) at CARLA({:.1}, {:.1}, {:.1})",
            spawn_loc.x,
            spawn_loc.y,
            spawn_loc.z
        );

        api::SpawnVehicleEntityResponse {
            result: Some(proto_ok()),
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

        tracing::info!("SpawnPedestrian: name={name} (stub - will implement in Phase 3)");

        // Phase 3: spawn walker + AI controller
        api::SpawnPedestrianEntityResponse {
            result: Some(proto_ok()),
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

        tracing::info!("SpawnMiscObject: name={name} (stub - will implement in Phase 3)");

        api::SpawnMiscObjectEntityResponse {
            result: Some(proto_ok()),
        }
    }

    pub fn despawn_entity(
        &mut self,
        req: api::DespawnEntityRequest,
    ) -> api::DespawnEntityResponse {
        let name = &req.name;

        match self.entities.remove(name) {
            Some(actor_id) => {
                // Find and destroy the CARLA actor
                match self.world.actors() {
                    Ok(actors) => {
                        if let Ok(Some(actor)) = actors.find(actor_id) {
                            match actor.destroy() {
                                Ok(true) => {
                                    tracing::info!("Despawned '{name}' (actor_id={actor_id})");
                                }
                                Ok(false) => {
                                    tracing::warn!("Despawn '{name}' returned false (already destroyed?)");
                                }
                                Err(e) => {
                                    tracing::warn!("Despawn '{name}' error: {e}");
                                }
                            }
                        }
                    }
                    Err(e) => {
                        tracing::warn!("Failed to get actors for despawn: {e}");
                    }
                }
                api::DespawnEntityResponse {
                    result: Some(proto_ok()),
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

        for entity_status in &req.status {
            let name = &entity_status.name;

            let entity = match self.entities.get(name) {
                Some(e) => e,
                None => {
                    // Echo back unknown entities unchanged
                    updated.push(api::UpdatedEntityStatus {
                        name: name.clone(),
                        action_status: entity_status.action_status.clone(),
                        pose: entity_status.pose.clone(),
                    });
                    continue;
                }
            };

            let actor_id = entity.carla_actor_id;
            let is_ego = entity.entity_type == EntityType::Ego;

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
                            pose: entity_status.pose.clone(),
                        });
                    }
                }
            } else {
                // NPC or ego overwrite: set transform from SSv2 pose
                if let Some(pose) = entity_status.pose.as_ref() {
                    let transform = ros_pose_to_carla_transform(pose);
                    if let Err(e) = self.set_actor_transform(actor_id, &transform) {
                        tracing::warn!("set_transform for '{name}': {e}");
                    }
                }

                // Echo back the same pose
                updated.push(api::UpdatedEntityStatus {
                    name: name.clone(),
                    action_status: entity_status.action_status.clone(),
                    pose: entity_status.pose.clone(),
                });
            }
        }

        api::UpdateEntityStatusResponse {
            result: Some(proto_ok()),
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
        &self,
        _req: api::UpdateTrafficLightsRequest,
    ) -> api::UpdateTrafficLightsResponse {
        // Phase 4: freeze + set_state per signal
        api::UpdateTrafficLightsResponse {
            result: Some(proto_ok()),
        }
    }

    /// Restore async mode so CARLA isn't stuck waiting for ticks on exit.
    pub fn restore_async_mode(&mut self) {
        match self.world.settings() {
            Ok(mut settings) => {
                settings.synchronous_mode = false;
                settings.fixed_delta_seconds = None;
                if let Err(e) = self.world.apply_settings(&settings, Duration::from_secs(10)) {
                    tracing::warn!("Failed to restore async mode: {e}");
                } else {
                    tracing::info!("Restored CARLA to async mode");
                }
            }
            Err(e) => tracing::warn!("Failed to get settings for cleanup: {e}"),
        }
    }

    // --- Private helpers ---

    fn read_actor_state(
        &self,
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

        let (vx, vy, vz) =
            coordinate_conversion::carla_to_ros_velocity(v.x, v.y, v.z);
        let (wx, wy, wz) =
            coordinate_conversion::carla_to_ros_angular_velocity(av.x, av.y, av.z);
        let (ax, ay, az) =
            coordinate_conversion::carla_to_ros_acceleration(acc.x, acc.y, acc.z);

        let action_status = traffic_simulator_msgs::ActionStatus {
            current_action: String::new(),
            twist: Some(geometry_msgs::Twist {
                linear: Some(geometry_msgs::Vector3 { x: vx, y: vy, z: vz }),
                angular: Some(geometry_msgs::Vector3 { x: wx, y: wy, z: wz }),
            }),
            accel: Some(geometry_msgs::Accel {
                linear: Some(geometry_msgs::Vector3 { x: ax, y: ay, z: az }),
                angular: Some(geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 }),
            }),
            linear_jerk: 0.0,
        };

        Some((pose, action_status))
    }

    fn set_actor_transform(
        &self,
        actor_id: u32,
        transform: &Transform,
    ) -> Result<()> {
        let actors = self.world.actors().wrap_err("get actors")?;
        let actor = actors
            .find(actor_id)
            .wrap_err("find actor")?
            .ok_or_else(|| eyre::eyre!("actor {actor_id} not found"))?;
        actor
            .set_transform(transform)
            .wrap_err("set_transform")?;
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
