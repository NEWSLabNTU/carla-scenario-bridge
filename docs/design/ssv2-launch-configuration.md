# SSv2 Launch Configuration for CARLA

How to launch scenario_simulator_v2 to use this adapter as its backend instead of `simple_sensor_simulator`.

## Required Launch Parameters

```bash
just scenario /path/to/scenario.xosc
```

Or manually:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/repos/autoware_carla_bridge/install/setup.bash
source install/setup.bash

play_launch launch --web-addr 0.0.0.0:8081 \
    scenario_test_runner scenario_test_runner.launch.py \
    scenario:=/path/to/scenario.xosc \
    map_path:=$ACB_DIR/data/carla-autoware-bridge/Town01 \
    architecture_type:=awf/universe/20250130 \
    autoware_launch_package:=acb_launch \
    autoware_launch_file:=carla_simulator.launch.xml \
    launch_simple_sensor_simulator:=false \
    simulate_localization:=false \
    launch_autoware:=true \
    launch_rviz:=false \
    record:=false \
    port:=5555 \
    sensor_model:=acb_sensor_kit \
    vehicle_model:=acb_vehicle
```

### Parameter Reference

| Parameter | Value | Why |
|---|---|---|
| `autoware_launch_package` | `acb_launch` | Use our CARLA-optimized Autoware launch (not `autoware_launch`). Provides `use_sim_time`, MRM timeout config, etc. |
| `autoware_launch_file` | `carla_simulator.launch.xml` | Our launch file with CARLA-specific settings (from autoware_carla_bridge). |
| `launch_simple_sensor_simulator` | `false` | Don't launch SSv2's built-in simulator. This adapter replaces it. |
| `simulate_localization` | `false` | Use real GNSS->NDT localization from CARLA sensors, not ground-truth poses. |
| `launch_autoware` | `true` | SSv2's concealer launches Autoware (init localization, set route, engage). |
| `port` | `5555` (or `$SSV2_PORT`) | ZMQ port the adapter listens on. Must match adapter's `SSV2_PORT`. |
| `map_path` | Path to map directory | Directory with `lanelet2_map.osm` and `pointcloud_map.pcd`. Default: `$ACB_DIR/data/carla-autoware-bridge/$MAP_NAME` |
| `architecture_type` | `awf/universe/20250130` | Must match Autoware version for topic naming and concealer behavior. |
| `sensor_model` | `acb_sensor_kit` | Our CARLA sensor kit (from autoware_carla_bridge). |
| `vehicle_model` | `acb_vehicle` | Our CARLA vehicle model (from autoware_carla_bridge). |
| `launch_rviz` | `false` | SSv2's RViz not needed; use Autoware's own if visualization required. |
| `record` | `false` | Disable rosbag recording for initial testing. |

### Critical: Autoware Launch Package

SSv2's `FieldOperatorApplication` (concealer) launches Autoware as a child process. By default it uses `autoware_launch/planning_simulator.launch.xml`, which doesn't have our CARLA-specific settings:
- `use_sim_time=true` globally
- `system_run_mode=logging_simulation` (disables pose_initializer stop check)
- Relaxed MRM handler timeouts
- CARLA-optimized NDT parameters
- Traffic light recognition disabled

Setting `autoware_launch_package:=acb_launch` and `autoware_launch_file:=carla_simulator.launch.xml` makes SSv2 launch our CARLA-optimized Autoware configuration. This requires `autoware_carla_bridge` to be built and its `install/` sourced.

### Source Order Matters

The `just scenario` recipe sources three environments:
1. `/opt/autoware/1.5.0/setup.bash` — base Autoware packages
2. `$ACB_DIR/install/setup.bash` — autoware_carla_bridge packages (acb_launch, sensor_kit, vehicle)
3. `$PROJECT/install/setup.bash` — SSv2 packages (scenario_test_runner, etc.)

All three must be sourced for the full launch to resolve all package dependencies.

## Process Startup Order

Four processes across three terminals (CARLA is a background service):

```
Terminal 1 (background):  just carla-start           → CARLA server on :2000
Terminal 2:               just run                    → adapter (CARLA + ZMQ :5555)
Terminal 3:               cd $ACB_DIR && just bridge  → autoware_carla_bridge (sensors + vehicle)
Terminal 4:               just scenario <file>        → SSv2 + Autoware
```

**Order matters:**
1. **CARLA** — `just carla-start` (wait ~30s for initialization)
2. **Adapter** — `just run` (connects to CARLA, binds ZMQ port, waits for SSv2 requests)
3. **autoware_carla_bridge** — `cd $ACB_DIR && just bridge` (connects to CARLA, waits for hero vehicle)
4. **SSv2** — `just scenario <file>` (connects to adapter, spawns ego, launches Autoware, runs scenario)

**What happens in sequence:**
1. SSv2 sends `Initialize` → adapter sets CARLA sync mode
2. SSv2 sends `SpawnVehicleEntity(ego, is_ego=true)` → adapter spawns vehicle with `role_name="hero"`
3. `autoware_carla_bridge` detects hero vehicle → attaches LiDAR, camera, IMU, GNSS sensors
4. SSv2's `FieldOperatorApplication` launches Autoware (`acb_launch/carla_simulator.launch.xml`)
5. Autoware's `robot_state_publisher` publishes TF tree → bridge resolves sensor frames
6. Bridge starts publishing sensor data + vehicle status to ROS 2 topics
7. SSv2 concealer calls `/api/localization/initialize` → GNSS→gnss_poser→NDT→localized
8. SSv2 concealer calls `/api/routing/set_route_points` → Autoware plans route
9. SSv2 concealer calls `/api/external/set/engage` → Autoware drives
10. Each `UpdateFrame`: adapter calls `world.tick()` → CARLA steps → bridge publishes `/clock`
11. Each `UpdateEntityStatus`: adapter reads ego pose from CARLA → reports to SSv2
12. SSv2 evaluates scenario conditions (ReachPositionCondition, timeout) → exitSuccess or exitFailure

## Clock Synchronization

The tick timing chain is:

```
SSv2 sends UpdateFrame
  -> adapter calls world.tick()
  -> CARLA advances one physics/render step
  -> autoware_carla_bridge receives tick notification
  -> bridge publishes /clock (simulation time)
  -> adapter returns UpdateFrameResponse to SSv2
```

`world.tick()` blocks until CARLA completes the step, so clock publishing is naturally ordered. The bridge must be in passive `wait_for_tick_or_timeout()` mode (not calling `world.tick()` itself). This is the default when `demo_scenario.py` is not running.

## AutowareUniverse Topic Conflict

SSv2's `AutowareUniverse` (concealer node, part of `EgoEntity`) publishes vehicle status topics at 30Hz from its internal vehicle model. Our bridge publishes the same topics from real CARLA physics:

| Topic                             | AutowareUniverse    | autoware_carla_bridge |
|-----------------------------------|---------------------|-----------------------|
| `/vehicle/status/velocity_status` | From bicycle model  | From CARLA PhysX      |
| `/vehicle/status/steering_status` | Echoes Autoware cmd | From CARLA state      |
| `/vehicle/status/gear_status`     | Echoes Autoware cmd | From CARLA state      |
| `/vehicle/status/control_mode`    | Internal            | Internal              |

### Mitigation Strategy

**Phase 2 approach: Accept dual publishers.** Both publish to the same topics; Autoware subscribes and receives whichever message arrives. Since both are publishing at similar rates with similar data (the bridge reflects actual CARLA state, concealer reflects its model), this is unlikely to cause issues in practice.

**If issues arise**: Remap AutowareUniverse topics by modifying SSv2's launch to add a namespace prefix to the concealer node.

**Long term**: Disable AutowareUniverse entirely. This requires either:
- A fork of SSv2 with a `launch_concealer:=false` option
- A wrapper launch file that skips the concealer node
- Contributing upstream to SSv2

### Localization Topics

With `simulate_localization:=false`:
- AutowareUniverse publishes localization to `/simulation/debug/...` (harmless, not subscribed by Autoware)
- Real localization comes from CARLA GNSS -> gnss_poser -> NDT align (via autoware_carla_bridge)
- TF `map->base_link` comes from Autoware's EKF (not from concealer)

This is the correct configuration for full-pipeline testing.
