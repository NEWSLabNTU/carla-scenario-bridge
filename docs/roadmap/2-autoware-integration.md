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
- [ ] Set `role_name="hero"` attribute on ego vehicle spawn (Phase 1 should have this)
- [ ] Verify `autoware_carla_bridge` detects and attaches sensors to SSv2-spawned ego
- [ ] Verify bridge's `wait_for_tick_or_timeout()` works with adapter-driven ticks
- [ ] Verify clock publishing: `/clock` advances correctly with each `UpdateFrame`

### Ego Physics Loop
- [ ] Ego pose readback: `actor.transform()` after `world.tick()`, convert to ROS frame
- [ ] Ego velocity readback: `actor.velocity()`, convert to ROS frame
- [ ] Ego acceleration readback: `actor.acceleration()`, convert to ROS frame
- [ ] Pack ego state into `UpdateEntityStatusResponse` with correct `ActionStatus`

### SSv2 Launch Configuration
- [ ] Document required SSv2 launch parameters for CARLA mode
- [ ] Test `launch_simple_sensor_simulator:=false` + `port:={SSV2_PORT}`
- [ ] Test `simulate_localization:=false` (real GNSS->NDT pipeline)
- [ ] Assess AutowareUniverse topic conflict; choose mitigation strategy

### End-to-End Test
- [ ] Write minimal test scenario: ego spawns at pose A, route to pose B
- [ ] Verify: ego appears in CARLA at correct position
- [ ] Verify: Autoware receives sensor data (LiDAR pointcloud, IMU, GNSS)
- [ ] Verify: Autoware localization initializes via GNSS->NDT
- [ ] Verify: Autoware plans route and engages autonomous mode
- [ ] Verify: ego vehicle drives toward goal in CARLA
- [ ] Verify: SSv2 scenario condition (ReachPositionCondition) triggers on arrival

## Acceptance Criteria

- [ ] SSv2 scenario with ego-only driving works end-to-end: SSv2 -> adapter -> CARLA -> bridge -> Autoware -> ego arrives at goal
- [ ] Autoware processes real CARLA sensor data (not ground-truth): LiDAR pointcloud visible in RViz
- [ ] Localization uses real GNSS->NDT pipeline (not simulated ground-truth poses)
- [ ] Ego vehicle pose in SSv2 matches CARLA PhysX result (not SSv2's bicycle model)
- [ ] `/clock` topic is consistent between adapter ticks and bridge publishing
- [ ] SSv2 `ReachPositionCondition` evaluates correctly using CARLA ego pose
- [ ] No crashes or hangs during 60-second autonomous driving scenario
