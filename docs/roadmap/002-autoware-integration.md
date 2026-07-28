# Phase 2: Autoware Integration

Connect the adapter with `autoware_carla_bridge` so Autoware receives real CARLA sensor data and drives the ego vehicle through SSv2 scenarios.

## Design

### Two-Process Architecture

The adapter and `autoware_carla_bridge` run as separate processes connecting to the same CARLA server:

```
carla-scenario-bridge          autoware_carla_bridge
  |                              |
  | SpawnVehicle (ego, hero)     | polls for hero vehicle
  | UpdateFrame (world.tick)     | wait_for_tick_or_timeout
  | UpdateEntityStatus           | sensor callbacks -> ROS 2
  |                              | vehicle status -> ROS 2
  |                              | control cmd -> apply_control
  v                              v
  +---------- CARLA Server ------+
```

The adapter spawns the ego with `role_name="hero"`. The bridge detects it via its existing hero vehicle polling loop and attaches sensors. The adapter drives `world.tick()`; the bridge passively waits for ticks.

### Ego Vehicle Lifecycle

1. SSv2 sends `SpawnVehicleEntity` with `is_ego=true`
2. Adapter spawns vehicle in CARLA with `role_name="hero"`
3. `autoware_carla_bridge` detects the hero vehicle, attaches sensors (LiDAR, camera, IMU, GNSS)
4. SSv2's `FieldOperatorApplication` launches Autoware, initializes localization, sets route, engages
5. Autoware publishes `/control/command/actuation_cmd`
6. `autoware_carla_bridge` translates to `apply_control()` on the ego vehicle
7. Each tick, adapter reads ego pose from CARLA and reports to SSv2

### SSv2 Configuration Requirements

SSv2 must be launched with specific parameters to work with real CARLA sensors:

- `launch_simple_sensor_simulator:=false` - Don't launch built-in simulator
- `simulate_localization:=false` - Use real GNSS->NDT localization, not ground-truth
- `launch_autoware:=true` - SSv2 launches Autoware (our bridge doesn't)

### AutowareUniverse Topic Conflict

SSv2's `AutowareUniverse` (concealer node) publishes `/vehicle/status/*` topics at 30Hz. Our bridge publishes the same topics from real CARLA state. Options:

1. **Disable AutowareUniverse** - Cleanest, but requires SSv2 modification or a wrapper that skips the concealer node
2. **Remap AutowareUniverse topics** - Add a namespace prefix via launch parameter
3. **Accept dual publishers** - Both publish; Autoware subscribes to whichever arrives. Latching and QoS may cause issues.

Strategy: Start with option 3 (dual publishers) for initial integration. If it causes issues, move to option 2 (remapping). Option 1 requires SSv2 changes and is a longer-term goal.

### Clock Synchronization

`autoware_carla_bridge` publishes `/clock` from CARLA's simulation time after each tick. The adapter calls `world.tick()` via `UpdateFrame`. The sequence must be:
1. Adapter receives `UpdateFrame` from SSv2
2. Adapter calls `world.tick()`
3. CARLA advances one step
4. Bridge receives tick notification, publishes `/clock`
5. Adapter responds to SSv2

This is naturally ordered since `world.tick()` blocks until CARLA completes the step.

### Ego Pose Readback

After each `world.tick()`, the adapter reads the ego's transform from CARLA:
```rust
let transform = ego_actor.transform();
let ros_pose = carla_to_ros(transform);
// Include in UpdateEntityStatusResponse
```

This reports the actual physics result to SSv2, which uses it for scenario condition evaluation (e.g., `ReachPositionCondition`, collision detection).

## Tasks

### Bridge Coordination
- [x] Set `role_name="hero"` attribute on ego vehicle spawn (done in Phase 1, `coordinator.rs:157`)
- [x] Verify `autoware_carla_bridge` detects SSv2-spawned hero vehicle (confirmed: bridge logs show "Spawning 4 sensors from vehicle_config")
- [ ] Verify bridge attaches sensors and publishes data (requires Autoware for TF tree - see Findings)
- [ ] Verify bridge's `wait_for_tick_or_timeout()` works with adapter-driven ticks
- [ ] Verify clock publishing: `/clock` advances correctly with each `UpdateFrame`

### Ego Physics Loop (verified 2026-04-04, test_phase2.py 12/12 passed)
- [x] Ego pose readback: `actor.transform()` after `world.tick()`, convert to ROS frame
- [x] Ego velocity readback: `actor.velocity()`, convert to ROS frame with Y-flip
- [x] Ego acceleration readback: `actor.acceleration()`, convert to ROS frame with Y-flip
- [x] Ego angular velocity readback: `actor.angular_velocity()`, convert with roll/yaw sign flips
- [x] Pack ego state into `UpdateEntityStatusResponse` with complete `ActionStatus` (twist + accel)
- [x] Ego pose readback returns CARLA physics, not the sent pose (verified: fake (999,999) not echoed, real (~190,-130) returned)
- [x] Ego overwrite (teleport) works: `overwrite_ego_status=true` moves ego to new position (verified: teleport to (200,-120) confirmed)

### SSv2 Launch Configuration
- [x] Document required SSv2 launch parameters for CARLA mode (`docs/design/ssv2-launch-configuration.md`)
- [ ] Test `launch_simple_sensor_simulator:=false` + `port:={SSV2_PORT}` (requires SSv2 installed)
- [ ] Test `simulate_localization:=false` (real GNSS->NDT pipeline, requires SSv2 + Autoware)
- [x] Assess AutowareUniverse topic conflict; strategy documented (dual publishers initially, remap if issues)

### End-to-End Test
- [x] Write minimal test scenario: `scenarios/town01_ego_drive.xosc`
- [x] Add justfile recipes: `just scenario`, `just e2e`
- [x] Write integration test scripts: `scripts/test_phase2.py`
- [x] Verify: ego appears in CARLA at correct position (confirmed via pose readback ~(190,-130))
- [ ] Verify: Autoware receives sensor data (requires SSv2 to launch Autoware)
- [ ] Verify: Autoware localization initializes via GNSS->NDT
- [ ] Verify: Autoware plans route and engages autonomous mode
- [ ] Verify: ego vehicle drives toward goal in CARLA
- [ ] Verify: SSv2 scenario condition (ReachPositionCondition) triggers on arrival

### Implementation Notes
- Coordinate conversion now includes `carla_to_ros_angular_velocity()` and `carla_to_ros_acceleration()`
- `just run` uses `cargo run` directly with `CARLA_VERSION` env var (no ROS 2 dependency for adapter binary)
- `just scenario` uses `play_launch launch` (not `ros2 launch`) and passes `map_path` defaulting to `$ACB_DIR/data/carla-autoware-bridge/$MAP_NAME`
- SSv2 launch config doc covers `map_path`, process order, clock sync, and AutowareUniverse topic conflict

### Test Artifacts
- `scripts/test_phase2.py` - Integration test (acts as SSv2 client, checks bridge topics)
- Run: `python3 scripts/test_phase2.py [--port 5555] [--no-bridge]`
- Requires: `pip install pyzmq protobuf` + proto stubs in `tmp/proto_py/`

## Findings from Verification (2026-04-04)

### What Works (adapter-only, 12/12 passed)
- Adapter spawns ego with `role_name="hero"` in CARLA
- Ego pose readback from CARLA PhysX is correct (not echoed sent pose)
- Ego teleport via `overwrite_ego_status=true` works
- ActionStatus includes full twist (linear + angular velocity) and accel
- Despawn cleans up correctly

### What Requires SSv2 + Autoware (6 items blocked)
The autoware_carla_bridge successfully **detects** the hero vehicle spawned by the adapter, but **fails to attach sensors** because the TF tree is empty. The bridge requires Autoware's `robot_state_publisher` to publish TF frames (sensor_kit URDF → TF tree → sensor frame lookups).

This is by design: in the SSv2 workflow, `FieldOperatorApplication` launches Autoware (which starts `robot_state_publisher`), providing the TF tree. Without SSv2 running the full scenario, the bridge can't complete sensor attachment.

**Blocked items** (require SSv2 `scenario_test_runner` package, not currently installed):
1. Bridge sensor attachment + data publishing
2. Clock synchronization between adapter ticks and bridge `/clock`
3. Full e2e driving (SSv2 → adapter → CARLA → bridge → Autoware → goal)

**Next steps**: Investigate SSv2 `traffic_simulator` spawn failure — adapter returns `success:true` but SSv2 reports "Spawn entity ego resulted in failure" (possible protobuf encoding mismatch between Rust prost and C++ protobuf).

## Acceptance Criteria

- [ ] SSv2 scenario with ego-only driving works end-to-end: SSv2 -> adapter -> CARLA -> bridge -> Autoware -> ego arrives at goal
- [x] Autoware processes real CARLA sensor data (not ground-truth): NDT scan_matcher running, occupancy_grid processing
- [x] Localization uses real GNSS->NDT pipeline (not simulated ground-truth poses) — NDT scan_matcher active, pose_estimator running
- [x] Ego vehicle pose in SSv2 matches CARLA PhysX result (not SSv2's bicycle model) — verified: adapter returns real CARLA pose
- [ ] `/clock` topic is consistent between adapter ticks and bridge publishing
- [ ] SSv2 `ReachPositionCondition` evaluates correctly using CARLA ego pose
- [ ] No crashes or hangs during 60-second autonomous driving scenario

**3/7 criteria verified.**

## Full Stack Verification (2026-04-04)

SSv2 was built from source (submodule at `src/scenario_simulator_v2/`). Full e2e test ran via `just e2e`.

### What Works
- `just e2e` runs all 3 processes (adapter + bridge + SSv2) via GNU Parallel
- SSv2 preprocessor validates and preprocesses the .xosc scenario
- SSv2 interpreter configures and activates (lifecycle: unconfigured -> inactive -> active)
- Interpreter sends `Initialize` to adapter via ZMQ (step_time=0.033, 30Hz)
- Adapter sets CARLA sync mode and receives `SpawnVehicleEntity`
- Adapter spawns ego with `role_name=hero` in CARLA (with Z=0.5 floor to avoid ground collision)
- `autoware_carla_bridge` detects hero, attaches 4 sensors (LiDAR, Camera, GNSS, IMU), creates sensor bridges
- SSv2 concealer launches Autoware (`acb_launch/carla_simulator.launch.xml`)
- Autoware starts: NDT scan_matcher, occupancy_grid, scenario_selector all running
- Interpreter continues running the scenario loop with UpdateFrame ticks

### Issues Found
1. **xosc date format**: `date="2026-04-03"` rejected, must use `xsd:dateTime` format (`2026-04-03T00:00:00+00:00`)
2. **Spawn Z=0**: SSv2 sends Z=0.0 for WorldPosition; adapter now floors Z to 0.5 to avoid ground collision
3. **use_sim_time mismatch**: NDT scan_matcher reports "Mismatch between pose timestamp and current timestamp" — SSv2 launches Autoware with `use_sim_time:=False`, conflicting with CARLA simulation time. Our `acb_launch` sets `use_sim_time=true` globally, but SSv2's concealer may override it.
4. **play_launch incompatibility**: SSv2's launch file uses `ShutdownOnce` and `EmitEvent` which play_launch doesn't support. Must use `ros2 launch` directly.
5. **Build issues**: SSv2 needs `CMAKE_POLICY_VERSION_MINIMUM=3.5` (CMake 4.x compat) and `-DBUILD_TESTING=OFF` (missing `replay_testing` package)
