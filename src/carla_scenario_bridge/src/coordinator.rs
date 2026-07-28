use carla::client::{ActorBase, World};
use carla::geom::{Location, Rotation, Transform};
use eyre::{Result, WrapErr};
use std::collections::HashSet;
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
    /// Entity names already reported as unknown by `update_entity_status`. SSv2 sends
    /// status every frame, so without this a desync would emit one warning per frame.
    warned_unknown_entities: HashSet<String>,
    /// Whether `update_traffic_lights` has already logged its rejection. Same reason.
    warned_traffic_lights: bool,
}

impl Coordinator {
    pub fn new(world: World) -> Self {
        Self {
            world,
            entities: EntityManager::new(),
            step_time: 0.05,
            sync_mode_enabled: false,
            has_ego: false,
            warned_unknown_entities: HashSet::new(),
            warned_traffic_lights: false,
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

        // Fresh run, fresh warnings -- otherwise a second scenario in one process would
        // stay quiet about problems it also has.
        self.warned_unknown_entities.clear();
        self.warned_traffic_lights = false;

        // Clear any entities from previous runs
        self.entities.clear();

        // The map named by the scenario is currently ignored -- whichever town CARLA
        // already holds is the town the scenario runs on, and every pose is interpreted
        // against it. Loading is roadmap phase 009. Until then this cannot silently pass:
        // a scenario written for one town running against another does not fail, it
        // produces meaningless results.
        if req.lanelet2_map_path.is_empty() {
            tracing::warn!("Initialize: no lanelet2_map_path supplied; cannot verify the map");
        } else {
            tracing::warn!(
                "Initialize: scenario requests map '{}', but map loading is not implemented \
                 (roadmap phase 009). Verify CARLA already has the matching town loaded -- \
                 a mismatch will not fail, it will produce meaningless poses.",
                req.lanelet2_map_path
            );
        }

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

        tracing::info!("SpawnVehicle: name={name}, is_ego={is_ego}, asset_key={asset_key}");

        // Convert pose from ROS to CARLA frame
        let mut carla_transform = match req.pose.as_ref() {
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
                tracing::warn!(
                    "Blueprint '{blueprint_key}' not found: {e}, trying vehicle.tesla.model3"
                );
                match self.world.actor_builder("vehicle.tesla.model3") {
                    Ok(b) => b,
                    Err(e2) => {
                        return api::SpawnVehicleEntityResponse {
                            result: Some(proto_err(format!("No valid blueprint: {e2}"))),
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

        // Rejected rather than stubbed. Returning success here would leave SSv2 believing
        // the pedestrian exists: nothing lands in EntityManager, so update_entity_status
        // takes its unknown-entity branch and echoes the requested pose straight back.
        // SSv2 would then score conditions against a pedestrian that CARLA never had and
        // Autoware's sensors cannot see. Implemented in roadmap phase 008.
        tracing::warn!("SpawnPedestrian '{name}' rejected: not implemented (roadmap phase 008)");

        api::SpawnPedestrianEntityResponse {
            result: Some(pedestrian_not_supported(&name)),
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

        // Rejected rather than stubbed -- see spawn_pedestrian_entity for why.
        tracing::warn!("SpawnMiscObject '{name}' rejected: not implemented (roadmap phase 008)");

        api::SpawnMiscObjectEntityResponse {
            result: Some(misc_object_not_supported(&name)),
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
                    Ok(()) => api::DespawnEntityResponse {
                        result: Some(proto_ok()),
                    },
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
        _req: api::UpdateTrafficLightsRequest,
    ) -> api::UpdateTrafficLightsResponse {
        // Rejected rather than stubbed. CARLA's built-in cycling is never frozen, so
        // reporting success would let a scenario script a red light while CARLA runs its
        // own schedule underneath. Implemented in roadmap phase 009.
        //
        // SSv2 sends this every frame, so the warning fires once.
        if !self.warned_traffic_lights {
            self.warned_traffic_lights = true;
            tracing::warn!(
                "UpdateTrafficLights rejected: not implemented (roadmap phase 009). \
                 CARLA's built-in cycling is still running."
            );
        }

        api::UpdateTrafficLightsResponse {
            result: Some(traffic_lights_not_supported()),
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

        let (vx, vy, vz) = coordinate_conversion::carla_to_ros_velocity(v.x, v.y, v.z);
        let (wx, wy, wz) = coordinate_conversion::carla_to_ros_angular_velocity(av.x, av.y, av.z);
        let (ax, ay, az) = coordinate_conversion::carla_to_ros_acceleration(acc.x, acc.y, acc.z);

        let action_status = traffic_simulator_msgs::ActionStatus {
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
                angular: Some(geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }),
            }),
            linear_jerk: 0.0,
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

/// Rejection for a feature this bridge does not implement yet.
///
/// These must never report success. A handler that returns `success = true` without acting
/// leaves SSv2 believing the entity exists, and SSv2 will then score scenario conditions
/// against something CARLA never had. See `docs/roadmap/006-honest-failures.md`.
fn not_implemented(what: &str, roadmap_doc: &str) -> ProtoResult {
    ProtoResult {
        success: false,
        description: format!(
            "{what} is not implemented in carla-scenario-bridge; see docs/roadmap/{roadmap_doc}"
        ),
    }
}

fn pedestrian_not_supported(name: &str) -> ProtoResult {
    not_implemented(
        &format!("Pedestrian entities (entity '{name}')"),
        "008-entity-fidelity.md",
    )
}

fn misc_object_not_supported(name: &str) -> ProtoResult {
    not_implemented(
        &format!("Misc object entities (entity '{name}')"),
        "008-entity-fidelity.md",
    )
}

fn traffic_lights_not_supported() -> ProtoResult {
    not_implemented("Traffic light control", "009-map-and-traffic-lights.md")
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

    // --- Phase 006: unimplemented features must reject, never report success ---

    #[test]
    fn pedestrian_spawn_is_rejected() {
        let result = pedestrian_not_supported("ped_1");
        assert!(!result.success);
        assert!(
            result.description.contains("ped_1"),
            "description should name the entity: {}",
            result.description
        );
        assert!(
            result.description.contains("008-entity-fidelity.md"),
            "description should point at the roadmap: {}",
            result.description
        );
    }

    #[test]
    fn misc_object_spawn_is_rejected() {
        let result = misc_object_not_supported("barrier_1");
        assert!(!result.success);
        assert!(result.description.contains("barrier_1"));
        assert!(result.description.contains("008-entity-fidelity.md"));
    }

    #[test]
    fn traffic_light_update_is_rejected() {
        let result = traffic_lights_not_supported();
        assert!(!result.success);
        assert!(
            result.description.contains("009-map-and-traffic-lights.md"),
            "description should point at the roadmap: {}",
            result.description
        );
    }

    /// Whatever the message says, it must never be empty -- SSv2 surfaces this text and a
    /// bare failure with no reason is what phase 006 exists to eliminate.
    #[test]
    fn rejections_always_explain_themselves() {
        for result in [
            pedestrian_not_supported("x"),
            misc_object_not_supported("x"),
            traffic_lights_not_supported(),
            sensor_not_supported(),
        ] {
            assert!(!result.success);
            assert!(!result.description.is_empty());
        }
    }
}
