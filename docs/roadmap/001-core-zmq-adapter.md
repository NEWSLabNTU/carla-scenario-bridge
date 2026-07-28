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
- [x] Proto compilation: `prost-build` in `build.rs` compiles all 8 `.proto` files
- [x] Proto module: `proto.rs` re-exports all generated packages under a common parent (fixes cross-package `super::` references)
- [x] ZMQ server: bind REP socket, poll loop with 100ms timeout + shutdown flag
- [x] Request dispatch: deserialize `SimulationRequest`, match all 14 `oneof` variants, call coordinator
- [x] Response serialization: build `SimulationResponse`, `encode_to_vec()`, send

### CARLA Connection
- [x] Connect to CARLA with infinite retry loop (5s between retries, checks shutdown flag)
- [x] `Initialize` handler: set sync mode, `fixed_delta_seconds` from `step_time`, clear entity state
- [x] `UpdateFrame` handler: `world.tick()`
- [x] `UpdateStepTime` handler: update `fixed_delta_seconds` via `apply_settings()`
- [x] Graceful shutdown: restore async mode on exit so CARLA isn't stuck

### Entity Management
- [x] `EntityManager`: insert/remove/lookup by name, find ego, clear
- [x] `SpawnVehicleEntity` handler: `actor_builder()` with blueprint lookup + fallback to `vehicle.tesla.model3`, coordinate conversion, spawn, register
- [x] `SpawnVehicleEntity`: set `role_name="hero"` on ego vehicle via `set_attribute()`
- [x] `DespawnEntity` handler: lookup actor ID, `actors.find()`, `actor.destroy()`, deregister
- [x] `UpdateEntityStatus` handler: NPC `set_transform()`, ego pose/velocity readback from CARLA
- [x] `UpdateEntityStatus`: respect `overwrite_ego_status` flag (teleport via `set_transform` when true)

### Coordinate Conversion
- [x] Position conversion: ROS right-handed <-> CARLA left-handed (Y-flip)
- [x] Orientation conversion: quaternion<->euler with degree/radian + roll/yaw sign flips
- [x] Unit tests: 3 round-trip tests (position, rotation, quaternion-euler) all passing

### Sensor Stubs
- [x] `AttachLidarSensor`: return `Success=false` ("CARLA sensors provided by autoware_carla_bridge")
- [x] `AttachDetectionSensor`: return `Success=false`
- [x] `AttachOccupancyGridSensor`: return `Success=false`
- [x] `AttachImuSensor`: return `Success=false`
- [x] `AttachPseudoTrafficLightDetector`: return `Success=false`

### Implementation Notes
- `SpawnPedestrianEntity` and `SpawnMiscObjectEntity` are stub-ok (return success, no CARLA spawn) - full implementation in Phase 3
- `UpdateTrafficLights` is stub-ok (return success, no CARLA action) - full implementation in Phase 4
- `cargo build` succeeds, `cargo test` passes 3/3

## Acceptance Criteria

All criteria verified on 2026-04-04 using `scripts/test_phase1.py` (26/26 passed) + manual retry test.

- [x] SSv2's `scenario_test_runner` connects to the adapter via ZMQ without errors
- [x] `Initialize` succeeds: CARLA enters sync mode
- [x] `SpawnVehicleEntity` with `is_ego=true` spawns a vehicle in CARLA (actor_id=174, role_name=hero)
- [x] `SpawnVehicleEntity` with `is_ego=false` spawns an NPC vehicle at the correct position
- [x] `UpdateEntityStatus` moves NPC vehicles to new positions each frame (pose echoed correctly)
- [x] `UpdateEntityStatus` returns the ego vehicle's actual CARLA pose (non-zero, from PhysX)
- [x] `UpdateFrame` advances CARLA simulation by one tick
- [x] `DespawnEntity` removes vehicles from the CARLA world (confirmed via actor_id lookup)
- [x] All `AttachSensor` requests return `Success=false` without crashing
- [x] Coordinate conversion: vehicle spawned at ROS pose `(10, 5, 0.5)` appears at CARLA `(10.0, -5.0, 0.5)` (confirmed in adapter logs)
- [x] Ctrl-C gracefully shuts down (restores CARLA async mode, exits within 1s)
- [x] Adapter handles CARLA not running on startup (retries every 5s until connected)

### Test Artifacts
- Test script: `scripts/test_phase1.py` (acts as SSv2 ZMQ client, sends all 14 message types)
- Run: `python3 scripts/test_phase1.py [--port 5555] [--no-carla]`
- Requires: `pip install pyzmq protobuf` + proto stubs generated via `protoc --python_out` (in `tmp/proto_py/`)

### Known Issue
- `CARLA_VERSION=0.9.16` must be set during build and run. The justfile handles this for `just build` and `just run`. Without it, carla-rust defaults to 0.10.0 API which crashes against CARLA 0.9.16.
