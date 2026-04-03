# Phase 1: Core ZMQ Adapter

Minimal viable adapter that SSv2 can connect to, spawn an ego vehicle in CARLA, and step the simulation.

## Design

### ZMQ Server

A single-threaded ZMQ REP socket bound to `tcp://*:{SSV2_PORT}`. The server polls for incoming messages, deserializes the protobuf `SimulationRequest`, dispatches to the matching handler, and sends back the serialized `SimulationResponse`.

**Reference implementation**: SSv2's `zmq_multi_server.hpp` polls with 1ms timeout in a background thread. Our Rust implementation uses `zmq::poll()` in the main loop with a shutdown flag check.

**Message flow per tick**:
```
recv SimulationRequest
  -> match request.oneof_case
  -> call handler (e.g., coordinator.update_frame())
  -> build SimulationResponse
send SimulationResponse
```

### Protobuf Code Generation

Use `prost-build` in `build.rs` to compile all `.proto` files in `proto/` into Rust types. The proto files are copied from SSv2's `simulation/simulation_interface/proto/`. They include nested imports (`geometry_msgs.proto`, `traffic_simulator_msgs.proto`, etc.) that must all be available.

### CARLA Connection

Connect to CARLA using `carla-rust` with an infinite retry loop (same pattern as `autoware_carla_bridge`). On `Initialize`:
1. Connect to CARLA at `{CARLA_HOST}:{CARLA_PORT}`
2. Set synchronous mode with `fixed_delta_seconds` from `InitializeRequest.step_time`
3. Store `realtime_factor` for potential frame pacing

### Entity Lifecycle

`SpawnVehicleEntity`:
- Map `asset_key` to a CARLA vehicle blueprint (direct match or lookup in `bridge_config.yaml`)
- Convert `pose` from ROS right-handed to CARLA left-handed frame
- Call `world.spawn_actor(blueprint, transform)`
- If `is_ego == true`, set `role_name` attribute to `"hero"` so `autoware_carla_bridge` can find it
- Register in `EntityManager` as `(name, actor_id, EntityType::Ego | Vehicle)`

`DespawnEntity`:
- Look up CARLA actor ID from `EntityManager`
- Call `actor.destroy()`
- Remove from `EntityManager`

### Entity Status Updates

`UpdateEntityStatus` processes a list of entity poses per frame:
- **NPC vehicles** (`is_ego == false`): Convert pose to CARLA frame, call `actor.set_transform(transform)`. Echo the same pose back in the response.
- **Ego vehicle** (`is_ego == true`):
  - If `overwrite_ego_status == true`: teleport ego via `set_transform()` (manual/init mode)
  - If `overwrite_ego_status == false`: read ego's current pose from CARLA PhysX via `actor.transform()`, convert to ROS frame, return in response. CARLA physics drives the ego - Autoware control commands reach CARLA via `autoware_carla_bridge`.

### Frame Synchronization

`UpdateFrame` calls `world.tick()` to advance CARLA by one step. The adapter is the **sole ticker** - `autoware_carla_bridge` must be in passive `wait_for_tick_or_timeout()` mode.

`UpdateStepTime` updates CARLA's `fixed_delta_seconds` via `world.settings()`.

### Coordinate Conversion

Identical to `autoware_carla_bridge`'s `coordinate_conversion.rs`:
- Position: `carla_y = -ros_y` (Y-axis flip)
- Rotation: Corresponding quaternion adjustment (roll/yaw sign flips)

### Sensor Attach Handlers

All five `AttachSensor` requests return `Success=false` with a description like `"CARLA sensors provided by autoware_carla_bridge"`. This follows the AWSIM pattern where the external simulator provides its own sensors.

## Tasks

### Protobuf + ZMQ Infrastructure
- [ ] Proto compilation: `prost-build` in `build.rs` compiles all `.proto` files
- [ ] Proto module: `proto.rs` re-exports generated types with doc comments
- [ ] ZMQ server: bind REP socket, poll loop with shutdown flag
- [ ] Request dispatch: deserialize `SimulationRequest`, match `oneof`, call handler
- [ ] Response serialization: build `SimulationResponse`, serialize, send

### CARLA Connection
- [ ] Connect to CARLA with infinite retry loop (catch panics, log progress every 2s)
- [ ] `Initialize` handler: set sync mode, `fixed_delta_seconds`, store `realtime_factor`
- [ ] `UpdateFrame` handler: `world.tick()`
- [ ] `UpdateStepTime` handler: update `fixed_delta_seconds`
- [ ] Graceful shutdown: restore async mode on exit so CARLA isn't stuck

### Entity Management
- [ ] `EntityManager`: insert/remove/lookup by name, find ego
- [ ] `SpawnVehicleEntity` handler: blueprint lookup, coordinate conversion, `spawn_actor()`, register
- [ ] `SpawnVehicleEntity`: set `role_name="hero"` on ego vehicle
- [ ] `DespawnEntity` handler: `actor.destroy()`, deregister
- [ ] `UpdateEntityStatus` handler: NPC `set_transform()`, ego pose readback
- [ ] `UpdateEntityStatus`: respect `overwrite_ego_status` flag for teleport

### Coordinate Conversion
- [ ] Position conversion: ROS right-handed <-> CARLA left-handed (Y-flip)
- [ ] Orientation conversion: quaternion adjustment for handedness
- [ ] Unit test: round-trip conversion preserves values

### Sensor Stubs
- [ ] `AttachLidarSensor`: return `Success=false`
- [ ] `AttachDetectionSensor`: return `Success=false`
- [ ] `AttachOccupancyGridSensor`: return `Success=false`
- [ ] `AttachImuSensor`: return `Success=false`
- [ ] `AttachPseudoTrafficLightDetector`: return `Success=false`

## Acceptance Criteria

- [ ] SSv2's `scenario_test_runner` connects to the adapter via ZMQ without errors
- [ ] `Initialize` succeeds: CARLA enters sync mode
- [ ] `SpawnVehicleEntity` with `is_ego=true` spawns a vehicle in CARLA visible in the spectator view
- [ ] `SpawnVehicleEntity` with `is_ego=false` spawns an NPC vehicle at the correct position
- [ ] `UpdateEntityStatus` moves NPC vehicles to new positions each frame
- [ ] `UpdateEntityStatus` returns the ego vehicle's actual CARLA pose (not the SSv2-sent pose)
- [ ] `UpdateFrame` advances CARLA simulation by one tick
- [ ] `DespawnEntity` removes vehicles from the CARLA world
- [ ] All `AttachSensor` requests return `Success=false` without crashing
- [ ] Coordinate conversion: a vehicle spawned at ROS pose `(10, 5, 0)` appears at CARLA position `(10, -5, 0)`
- [ ] Ctrl-C gracefully shuts down (restores CARLA async mode, exits within 1s)
- [ ] Adapter handles CARLA not running on startup (retries until connected)
