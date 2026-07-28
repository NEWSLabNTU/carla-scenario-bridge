use carla::client::{ActorBase, World};
use carla::geom::{Location, Rotation, Transform};
use eyre::{Result, WrapErr};
use std::time::Duration;

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
    world: World,
    entities: EntityManager,
    step_time: f64,
    /// Whether this bridge has switched CARLA into synchronous mode. Also gates the
    /// cleanup path: we must not restore async mode we never left.
    sync_mode_enabled: bool,
    /// Whether an ego vehicle has been spawned. See [`FrameAction`].
    has_ego: bool,
}

impl Coordinator {
    pub fn new(world: World) -> Self {
        Self {
            world,
            entities: EntityManager::new(),
            step_time: 0.05,
            sync_mode_enabled: false,
            has_ego: false,
        }
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
        self.sync_mode_enabled = false;
        self.has_ego = false;

        // Clear any entities from previous runs
        self.entities.clear();

        tracing::info!(
            "Initialized (step_time={}). CARLA left in async mode until the ego is spawned.",
            req.step_time
        );
        api::InitializeResponse {
            result: Some(proto_ok()),
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
        let mut carla_transform = match req.pose.as_ref() {
            Some(pose) => ros_pose_to_carla_transform(pose),
            None => Transform {
                location: Location { x: 0.0, y: 0.0, z: 0.0 },
                rotation: Rotation { roll: 0.0, pitch: 0.0, yaw: 0.0 },
            },
        };

        // Ensure minimum spawn height to avoid ground collision
        if carla_transform.location.z < 0.5 {
            tracing::info!(
                "Raising spawn Z from {:.1} to 0.5 to avoid ground collision",
                carla_transform.location.z
            );
            carla_transform.location.z = 0.5;
        }

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

        // Wait one tick for the actor to be fully initialized. Only meaningful once we
        // own the tick -- before sync mode is on, CARLA is advancing by itself.
        if self.sync_mode_enabled {
            let _ = self.world.tick();
        }

        let actor_id = actor.id();
        let entity_type = if is_ego {
            EntityType::Ego
        } else {
            EntityType::Vehicle
        };
        self.entities.insert(name.clone(), entity_type, actor_id);

        if is_ego {
            // Releases the sync-mode gate: from the next UpdateFrame onward this bridge
            // owns the tick. See FrameAction.
            self.has_ego = true;
        }

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
                if let Err(e) = self.world.apply_settings(&settings, Duration::from_secs(10)) {
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
}
