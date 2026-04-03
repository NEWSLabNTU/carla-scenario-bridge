/// Coordinator: translates SSv2 protobuf requests into CARLA API calls.
///
/// Owns the CARLA client connection, entity manager, and traffic light mapper.
/// Each method corresponds to one of the 14 SSv2 message types.
///
/// Design follows the AWSIM pattern:
/// - NPCs are puppeteered (set_transform from SSv2 poses)
/// - Ego vehicle uses CARLA PhysX (pose read back to SSv2)
/// - Sensors return Success=false (CARLA sensors via autoware_carla_bridge)
/// - Traffic lights are frozen and set per SSv2 commands

// TODO(Phase 1): CARLA client connection with retry loop
// TODO(Phase 1): Initialize handler (sync mode, fixed_delta_seconds)
// TODO(Phase 1): UpdateFrame handler (world.tick())
// TODO(Phase 1): SpawnVehicleEntity handler
// TODO(Phase 1): DespawnEntity handler
// TODO(Phase 1): UpdateEntityStatus handler (NPC set_transform, ego readback)
// TODO(Phase 1): All AttachSensor handlers (return Success=false)
// TODO(Phase 2): Ego sensor attachment (trigger autoware_carla_bridge)
// TODO(Phase 4): UpdateTrafficLights handler
