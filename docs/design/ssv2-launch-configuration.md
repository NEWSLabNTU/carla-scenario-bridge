# SSv2 Launch Configuration for CARLA

How to launch scenario_simulator_v2 to use this adapter as its backend instead of `simple_sensor_simulator`.

## Required Launch Parameters

```bash
just ego-av                            # ego's Autoware + acb_bridge, once, long-lived
just scenario /path/to/scenario.xosc   # then, per scenario run
```

Or manually:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/repos/autoware_carla_bridge/install/setup.bash
source install/setup.bash

# First: the ego stack, in the same ROS domain SSv2 will run in. Wait for Autoware's
# ADAPI services to come up before starting a scenario.
play_launch launch --web-addr 0.0.0.0:8082 \
    csb_launch ego_av.launch.xml \
    map_path:=$PROJECT/data/carla-autoware-bridge/Town01

# Then, per scenario run:
play_launch launch --web-addr 0.0.0.0:8081 \
    scenario_test_runner scenario_test_runner.launch.py \
    scenario:=/path/to/scenario.xosc \
    architecture_type:=awf/universe/20250130 \
    launch_simple_sensor_simulator:=false \
    simulate_localization:=false \
    launch_autoware:=false \
    launch_rviz:=false \
    record:=false \
    port:=5555 \
    sensor_model:=acb_sensor_kit \
    vehicle_model:=acb_vehicle
```

### Parameter Reference

| Parameter | Value | Why |
|---|---|---|
| `launch_simple_sensor_simulator` | `false` | Don't launch SSv2's built-in simulator. This adapter replaces it. |
| `simulate_localization` | `false` | Use real GNSS->NDT localization from CARLA sensors, not ground-truth poses. |
| `launch_autoware` | `false` | Don't fork Autoware; connect the concealer to the Autoware already in this ROS domain (`ego_av.launch.xml`). SSv2 still drives init/route/engage through it. |
| `port` | `5555` (or `$SSV2_PORT`) | ZMQ port the adapter listens on. Must match adapter's `SSV2_PORT`. |
| `architecture_type` | `awf/universe/20250130` | Must match Autoware version for topic naming and concealer behavior. |
| `sensor_model` | `acb_sensor_kit` | Still required with `launch_autoware:=false`: `EgoEntity` reads it before it reaches the `launch_autoware` branch (`ego_entity.cpp:64-65`). |
| `vehicle_model` | `acb_vehicle` | Same as `sensor_model` — read unconditionally (`ego_entity.cpp:64-65`). |
| `launch_rviz` | `false` | SSv2's RViz not needed; use Autoware's own if visualization required. |
| `record` | `false` | Disable rosbag recording for initial testing. |

`autoware_launch_package` / `autoware_launch_file` are gone: they configured the fork that
no longer happens. `map_path` moved to the ego stack launch; the scenario runner takes the
map from the `.xosc`'s `LogicFile`.

### The Ego's Autoware Stack

SSv2's `FieldOperatorApplication` (concealer) no longer launches Autoware as a child
process. `ego_av.launch.xml` brings up the same launch file the concealer used to fork —
`acb_launch/carla_simulator.launch.xml` — together with the ego's `acb_bridge`
(`publish_clock:=false`), mirroring `background_av.launch.xml`. That launch file carries
the CARLA-specific settings a stock `autoware_launch/planning_simulator.launch.xml` lacks:
- `use_sim_time=true` globally
- `system_run_mode=logging_simulation` (disables pose_initializer stop check)
- Relaxed MRM handler timeouts
- CARLA-optimized NDT parameters
- Traffic light recognition disabled

Two constraints replace the old fork parameters:
- **Domain**: the ego stack must run in SSv2's `ROS_DOMAIN_ID` — the concealer reaches it
  over plain ROS topics and services. `ego_av.launch.xml` sets no domain; run it with
  SSv2's domain ambient.
- **Environment**: `autoware_carla_bridge` must be built and its `install/` sourced where
  `ego_av.launch.xml` runs (it resolves `acb_launch` and `acb_bridge`).

### Source Order Matters

The `just ego-av` and `just scenario` recipes source three environments:
1. `/opt/autoware/1.5.0/setup.bash` — base Autoware packages
2. `$ACB_DIR/install/setup.bash` — autoware_carla_bridge packages (acb_launch, sensor_kit, vehicle)
3. `$PROJECT/install/setup.bash` — SSv2 packages (scenario_test_runner, etc.)

All three must be sourced for the full launch to resolve all package dependencies.

## Process Startup Order

Four processes across three terminals (CARLA is a background service):

```
Terminal 1 (background):  just carla-start      → CARLA server on :2000
Terminal 2:               just run              → adapter (CARLA + ZMQ :5555)
Terminal 3:               just ego-av           → ego's Autoware + acb_bridge (long-lived)
Terminal 4:               just scenario <file>  → SSv2 (per run)
```

**Order matters:**
1. **CARLA** — `just carla-start` (wait ~30s for initialization)
2. **Adapter** — `just run` (connects to CARLA, binds ZMQ port, waits for SSv2 requests)
3. **Ego stack** — `just ego-av` (Autoware + `acb_bridge`; bridge connects to CARLA and waits for the hero vehicle)
4. **SSv2** — `just scenario <file>` (connects to adapter, spawns ego, connects the concealer to the running Autoware, runs scenario)

The ego stack must be up — and Autoware's ADAPI services available — **before** SSv2
starts. With `launch_autoware:=false` nothing enforces this by construction anymore: the
concealer's constructor queues a `ChangeToStop` ADAPI call with a 180 s service timeout,
so a scenario started against a missing or late ego stack does not fail fast — it dies
~180 s in with an `AutowareError` service timeout. Check that
`/api/routing/set_route_points` (or any `/api/...` service) is listed by `ros2 service
list` before starting a scenario.

**Reuse across runs:** SSv2 forked nothing, so it kills nothing on scenario end — the ego
stack persists and the next `just scenario` reuses it. This is the concealer's intended
`launch_autoware:=False` flow: it returns Autoware to a safe STOP state between scenarios
(`field_operator_application.cpp:171-180`) so the next run can initialize, route, and
engage again. One long-lived ego stack, many scenario runs; restart it only when Autoware
itself is wedged.

**What happens in sequence:**
1. `just ego-av` brings up Autoware (`acb_launch/carla_simulator.launch.xml`) and the ego's `acb_bridge`; the bridge waits for a hero vehicle
2. SSv2 sends `Initialize` → adapter sets CARLA sync mode
3. SSv2 sends `SpawnVehicleEntity(ego, is_ego=true)` → adapter spawns vehicle with `role_name="hero"`
4. `acb_bridge` detects the hero vehicle → attaches LiDAR, camera, IMU, GNSS sensors; resolves sensor frames from Autoware's TF tree
5. Bridge starts publishing sensor data + vehicle status to ROS 2 topics
6. SSv2 concealer (already connected to the running Autoware) calls `/api/localization/initialize` → GNSS→gnss_poser→NDT→localized
7. SSv2 concealer calls `/api/routing/set_route_points` → Autoware plans route
8. SSv2 concealer calls `/api/external/set/engage` → Autoware drives
9. Each `UpdateFrame`: adapter calls `world.tick()` → CARLA steps → bridge publishes `/clock`
10. Each `UpdateEntityStatus`: adapter reads ego pose from CARLA → reports to SSv2
11. SSv2 evaluates scenario conditions (ReachPositionCondition, timeout) → exitSuccess or exitFailure
12. On scenario end SSv2 requests Autoware's stop state and exits; the ego stack stays up for the next run

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

> **`/clock` ownership.** SSv2's `traffic_simulator` publishes `/clock` in its own domain. So
> in the ego's domain `acb_bridge` must be launched with `publish_clock:=false` — two
> publishers cause "Detected jump back in time. Clearing TF buffer" in NDT and EKF. In
> background-AV domains there is no SSv2, so `acb_bridge` publishes `/clock` itself. See
> [multi-instance-architecture.md](multi-instance-architecture.md#clock-ownership).
>
> Related: sensor messages must be stamped from the node's ROS clock, never from
> `data.timestamp()` — the latter is CARLA *server uptime* (tens of thousands of seconds)
> while SSv2's `/clock` starts at 0.

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

> **Caveat.** The two publishers agree only while CARLA physics and the concealer's bicycle
> model agree — and their divergence is exactly what a scenario is meant to detect. Accepting
> this is a Phase 2 expedient, not a resting state. Tracked in
> [multi-instance-architecture.md](multi-instance-architecture.md#known-conflict-autowareuniverse-vehicle-status).

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
