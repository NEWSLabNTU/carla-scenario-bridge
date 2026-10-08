# User Guide

Run OpenSCENARIO scenarios in CARLA against your own Autoware, with ROS tools only.
Background: [design/user-workflow.md](design/user-workflow.md).

Two sides run independently and meet only in CARLA and the map directory:

- **Simulation side**: CARLA, `carla_scenario_bridge` (CARLA's adapter for
  scenario_simulator_v2), `scenario_agent_relay`, and scenario_simulator_v2 (SSv2), which runs
  your `.xosc` files and judges them.
- **Vehicle side**, one per autonomous vehicle: your Autoware with this project's sensor kit
  and vehicle interface, the CARLA vehicle bridge (`acb_bridge`) and the vehicle agent
  (`acb_pilot agent`), which takes goals from the scenario through the relay and drives
  Autoware's API locally.

They need not share a ROS domain, a host, or a start order.

Use `play_launch launch <pkg> <file>` wherever you would type `ros2 launch`; it is a drop-in
replacement with a web UI, per-node logs and crash recovery.

## 1. Install

- CARLA 0.9.16 (vendor package), ROS 2 Humble, Autoware 1.5.0 (or your own build).
- This repository with submodules, built once:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/carla-scenario-bridge.git
cd carla-scenario-bridge && just install-deps && just build
source install/setup.bash        # in every terminal below, after Autoware's setup.bash
```

## 2. Prepare a map directory

One directory per CARLA town, in Autoware's map layout plus one CARLA file:

```
<map dir>/  lanelet2_map.osm  pointcloud_map.pcd  pointcloud_map_metadata.yaml
            map_config.yaml   map_projector_info.yaml
            carla/traffic_lights.yaml
```

`carla/traffic_lights.yaml` (which Lanelet2 signal is which CARLA light) is generated once
per map, with CARLA running:

```bash
ros2 run carla_scenario_bridge carla_scenario_bridge --generate-signal-table <map dir>
```

The bridge checks it at every scenario start and refuses to run with a table that no longer
matches CARLA. The map directory is read-only at run time.

## 3. Prepare a scenario directory

```
<scenario dir>/  my_scenario.xosc
                 bridge.yaml          # optional: copy of carla_scenario_bridge's config
```

In the `.xosc`, name the map directory by path:

```xml
<RoadNetwork>
  <LogicFile filepath="/data/maps/Town01"/>          <!-- or "$(env CARLA_MAPS)/Town01" -->
</RoadNetwork>
```

The ego is the entity whose controller has `isEgo=true`; give it a goal with
`AcquirePositionAction` or `AssignRouteAction` as usual. Other vehicles are SSv2-driven by
default (exact choreography). Name a vehicle's controller `simulator_autopilot` and CARLA's
Traffic Manager drives it with physics instead: give it a goal with `AcquirePositionAction`
and a speed with an absolute `SpeedAction`, and it routes there and stops
([design/scenario-authoring.md](design/scenario-authoring.md#vehicles-the-simulator-drives-simulator_autopilot);
example `scenarios/town01_simulator_autopilot.xosc`). Scenario files are never modified.

## 4. Run

```bash
# Terminal 1 -- CARLA (headless; pin Vulkan to the NVIDIA card on hybrid-GPU hosts)
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000

# Terminal 2 -- simulation side, long-lived: the bridge (restarted if it crashes) and the
# agent relay (TCP 5560), in the ROS domain scenarios will run in
ROS_DOMAIN_ID=9 play_launch launch csb_launch simulation.launch.xml \
    [config_file:=<scenario dir>/bridge.yaml]

# Terminal 3 -- vehicle side, any ROS domain, any host, any time
ROS_DOMAIN_ID=1 play_launch launch acb_launch carla_simulator.launch.xml \
    map_path:=<map dir> vehicle_name:=hero relay:=tcp://<simulation host>:5560 \
    [autoware_launch:=<your autoware.launch.xml>]

# Terminal 4 -- one scenario, in the simulation side's domain; exits when it ends
ROS_DOMAIN_ID=9 play_launch launch csb_launch scenario.launch.xml \
    scenario:=<scenario dir>/my_scenario.xosc
```

The verdict is in `/tmp/scenario_test_runner/result.junit.xml`.

### Your own Autoware

`carla_simulator.launch.xml` is a CARLA profile around a stock `autoware.launch.xml` (yours,
via `autoware_launch:=`). It selects `vehicle_model:=acb_vehicle` and
`sensor_model:=acb_sensor_kit`, starts the vehicle interface (`acb_bridge`), and launches
three components with CARLA settings. Turn any of those off to keep your own:

| Argument | CARLA setting it applies |
|---|---|
| `carla_localization:=false` | CARLA-tuned NDT/EKF/pose-initializer parameters |
| `carla_perception:=false` | selectable `lidar_detection_model`, fusion-only traffic lights (signals arrive from CARLA over V2X) |
| `carla_system:=false` | component-state topics, MRM parameters and diagnostic graph matching CARLA's sensors |

## 5. When things go wrong

- **CARLA dies**: the running scenario fails within ~30 s; restart CARLA; the next scenario
  waits for it (up to 240 s) and runs.
- **The bridge dies**: the launch restarts it; the next scenario cleans up after it.
- **A scenario waits for the ego**: the vehicle side is not up, or its agent has not
  registered with the relay (`relay:=` wrong or unreachable). The relay logs every agent
  that registers; the scenario proceeds as soon as one does (SSv2 waits up to its
  `initialize_duration`).
- **Without a scenario**: `relay:=` empty and `goal_poses_file:=<yaml>` makes the vehicle
  side drive to a local goal on its own -- a quick check that Autoware and CARLA work.
