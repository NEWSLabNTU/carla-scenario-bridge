# SSv2 Simulation Interface Protocol

Reference for the ZMQ+Protobuf protocol defined by `scenario_simulator_v2`.

## Transport

- **Socket**: ZMQ REQ/REP
- **Binding**: Server binds `tcp://*:PORT`, client connects `tcp://host:PORT`
- **Default port**: 5555 (configurable via `port` launch parameter)
- **Serialization**: Protobuf binary

## Message Wrapper

All messages use a `oneof` union:

```protobuf
message SimulationRequest {
  oneof request {
    InitializeRequest initialize = 1;
    UpdateFrameRequest update_frame = 2;
    SpawnVehicleEntityRequest spawn_vehicle_entity = 3;
    SpawnPedestrianEntityRequest spawn_pedestrian_entity = 4;
    SpawnMiscObjectEntityRequest spawn_misc_object_entity = 5;
    DespawnEntityRequest despawn_entity = 6;
    UpdateEntityStatusRequest update_entity_status = 7;
    AttachLidarSensorRequest attach_lidar_sensor = 8;
    AttachDetectionSensorRequest attach_detection_sensor = 9;
    AttachOccupancyGridSensorRequest attach_occupancy_grid_sensor = 10;
    UpdateTrafficLightsRequest update_traffic_lights = 11;
    AttachPseudoTrafficLightDetectorRequest attach_pseudo_traffic_light_detector = 13;
    UpdateStepTimeRequest update_step_time = 14;
    AttachImuSensorRequest attach_imu_sensor = 15;
  }
}
```

## Message Sequence

```
Client (SSv2)                          Server (this adapter)
    |                                       |
    |--- Initialize --->                    |  Connect to CARLA, set sync mode
    |<-- InitializeResponse ---             |
    |                                       |
    |--- SpawnVehicleEntity (ego) --->      |  world.spawn_actor()
    |<-- Response ---                       |
    |                                       |
    |--- SpawnVehicleEntity (npc) --->      |  world.spawn_actor()
    |<-- Response ---                       |
    |                                       |
    |--- AttachLidarSensor --->             |  Success=false (CARLA sensors)
    |<-- Response ---                       |
    |                                       |
    |  [simulation loop]                    |
    |--- UpdateEntityStatus --->            |  NPCs: set_transform()
    |<-- UpdatedEntityStatus (ego pose) --- |  Ego: read from CARLA
    |                                       |
    |--- UpdateFrame --->                   |  world.tick()
    |<-- Response ---                       |
    |                                       |
    |--- UpdateTrafficLights --->           |  freeze() + set_state()
    |<-- Response ---                       |
    |  [/simulation loop]                   |
    |                                       |
    |--- DespawnEntity --->                 |  actor.destroy()
    |<-- Response ---                       |
```

## Key Request Details

### Initialize
- `realtime_factor`: Simulation speed (1.0 = real-time)
- `step_time`: Per-frame duration (seconds), typically 0.05 (20 Hz)
- `lanelet2_map_path`: Path to Lanelet2 map (used by SSv2 internally, not by this adapter)

### SpawnVehicleEntity
- `is_ego`: True for Autoware vehicle, false for NPC
- `asset_key`: Vehicle model identifier (map to CARLA blueprint)
- `pose`: Initial position + orientation (ROS frame)
- `parameters`: Name, bounding box, performance limits, axle config

### UpdateEntityStatus
- `status[]`: List of entity updates (pose, velocity, acceleration per entity)
- `npc_logic_started`: Whether NPC behaviors are active
- `overwrite_ego_status`: Force ego pose override (teleport/manual mode)
- **Response** includes `UpdatedEntityStatus[]` with server-computed poses (ego from CARLA physics)

### UpdateTrafficLights
- `states[]`: List of `TrafficSignal` with lanelet `id` and bulb states (color, shape, status)

## Proto File Location

Source: `scenario_simulator_v2/simulation/simulation_interface/proto/simulation_api_schema.proto`
Local copy: `proto/simulation_api_schema.proto`

## References

- [SSv2 Protobuf Documentation](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/)
- [SSv2 ZMQ Server Reference](https://github.com/tier4/scenario_simulator_v2/blob/main/simulation/simulation_interface/include/simulation_interface/zmq_multi_server.hpp)
- [AWSIM Connector Reference](https://github.com/autowarefoundation/AWSIM-Labs/blob/main/Assets/ScenarioSimulatorConnector/Script/ScenarioSimulatorConnector.cs)
