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

Use `play_launch launch <pkg> <file>` wherever you would type `ros2 launch`; it runs the same
launch files with a web UI and per-node logs (`play_log/<time>/node/<name>/{out,err}`), and
restarts the nodes a launch file declares `respawn="true"` (the bridge, here). Other nodes
that crash stay down, except composable nodes on the vehicle side, which
`--composable-respawn on-crash` reloads. Every command below carries the flags it needs:

| Flag | Where | Why |
|---|---|---|
| `--container-mode isolated --composable-respawn on-crash` | vehicle side | reloads a composable node whose process crashed. Autoware 1.5.0's `behavior_path_planner` aborts now and then when a new route arrives (`failed to add guard condition to wait set`, autoware_universe#12460; once in 27 UC-ACC variants here); without a reload the vehicle side cannot plan again and every later scenario fails in PLANNING. Autoware's containers declare no respawn, so only this brings it back, and a composable can be reloaded alone only when it is a process of its own (`isolated`; play_launch's default, `observable`, keeps one process per container as `ros2 launch` does) |
| `--web-addr 127.0.0.1:<port>` | every launch | each launch serves a web UI, 8080 by default; two on one port collide. This guide uses 8084 (simulation side), 8082 (vehicle side), 8083 (second vehicle side), 8081 (scenario) |

Everything else is play_launch's default, which since 0.15.1 matches `ros2 launch`: no
interception layer unless a contract asks for one, packages found the way `ament_index` finds
them, one process per container. Any `ros2 launch` command here works as well, without the
web UI and without the vehicle side's crash recovery.

## 1. Install

- CARLA 0.9.16 (vendor package), ROS 2 Humble, Autoware 1.5.0 (or your own build).
- Once per machine, the two things rosdep cannot install: Rust (rustup) and colcon's Rust
  plugin.

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
pip install colcon-cargo-ros2
```

- play_launch, the launcher every command below uses (PyPI; 0.15.1 or newer -- 0.15.1 is the first to end a scenario stack when its runner exits under the default parser):

```bash
sudo apt install libz3-dev ros-humble-rclcpp-components ros-humble-class-loader
pip install 'play_launch>=0.15.1'      # installs ~/.local/bin/play_launch
play_launch --version
```

  Optional, once: `play_launch setcap` (sudo) lets it record per-process I/O. Without it each
  launch warns `play_launch_io_helper lacks cap_sys_ptrace`; that is harmless.

- This repository with its submodules, its system dependencies, and one build:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/carla-scenario-bridge.git
cd carla-scenario-bridge
source /opt/ros/humble/setup.bash && source <Autoware>/setup.bash   # e.g. /opt/autoware/1.5.0
rosdep install --from-paths src --ignore-src -y   # prints only a summary when all is present
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
source install/setup.bash        # in every terminal below, after Autoware's setup.bash
```

The build targets CARLA 0.9.16: the version is pinned by the `carla-0916` feature in the
workspace `Cargo.toml`, and the bridges refuse to talk to a server of another version (both
versions are in the error). A clone made without `--recurse-submodules` is completed with
`git submodule update --init --recursive`.

Fresh-machine notes:

- Autoware's `data_path` must be writable (TensorRT writes each engine next to its ONNX file);
  the CARLA profile defaults to `~/autoware_data`. Populate it once:
  `ros2 run acb_launch setup_autoware_data` (mirrors `/opt/autoware/1.5.0/data`; pass
  `SRC DST` for other locations). `carla_simulator.launch.xml` refuses to start, naming that
  command, while `data_path` is missing or read-only.
- If `traffic_simulator` fails to find a dependency's headers after that dependency built,
  delete `build/traffic_simulator`: its CMake cache remembers the failed configure.
- CARLA's default Town01 weather is overcast and raining; start the simulation side with
  `weather:=ClearNoon` for a clear sky ([5. Configure the CARLA world](#5-configure-the-carla-world)).

## 2. Prepare a map directory

> **You need converted maps before anything runs.** This project uses the TUM
> carla-autoware-bridge map pack (LGPL-3.0), and that pack's public download link is dead at
> the moment; obtaining or generating maps is a separate effort (see "Where the maps come
> from" below). If someone has given you the pack, point `CARLA_MAPS` at the directory that
> holds `Town01/`, `Town02/`, ...

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

**Where the maps come from.** This project runs on the CARLA towns converted for Autoware by
TUM's [carla-autoware-bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge) (map pack
copyright 2024 TUM, LGPL-3.0; its `LICENSE.md` ships in each town directory). The pack's
download link (`scripts/download_maps.sh`) is dead at the time of writing, so there is no
public download yet; providing the maps is a separate effort. With a copy of the pack,
`CSB_MAP_SOURCE=<pack dir> scripts/download_maps.sh` installs it under
`data/carla-autoware-bridge/<Town>/` with the projector fix SSv2 needs; then generate each
town's `carla/traffic_lights.yaml` as above. The script also repairs two defects of the
pack, idempotently (`scripts/repair_lanelet_traffic_lights.py`, `scripts/repair_lanelet_subtypes.py`;
run them on a copy you installed another way): traffic lights whose type tags are empty
(invisible to Autoware and SSv2), and lanelets with no `subtype` tag -- the kerb strips beside
every road -- which make Autoware's `map_based_prediction` abort as soon as a pedestrian is
near one (`lanelet::NoSuchAttributeError: Could not find 1`). Keep the towns side by side in one directory --
the scenarios below find them as `$(env CARLA_MAPS)/<Town>`:

```bash
export CARLA_MAPS=<repo>/data/carla-autoware-bridge     # holds Town01/, Town02/, ...
```

## 3. Prepare a scenario directory

**Starter scenarios** are installed with the workspace, no scenario directory needed
(package `csb_examples`; each scenario, its town and what it checks are listed in its
[README](../src/csb_examples/README.md)):

```
$(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/
    basic/      this project's Town01/Town02 scenarios (ego drive, traffic light, pedestrian,
                rear contact, engage state, simulator autopilot, Town02 episode change)
    multi_av/   town01_two_av.xosc: needs a second vehicle side, entity bg_av_1 (Terminal 3b)
    awf/        AWF ODD use cases UC-ACC-001-0001 and UC-AEB-001-0001 (Apache-2.0), YAML with
                ScenarioModifiers: one run per parameter combination (27 and 9)
```

They name their map as `$(env CARLA_MAPS)/<Town>`: export `CARLA_MAPS` (section 2) in the
terminal that runs them.

**Which town the vehicle side loads.** The vehicle side's `map_path` is fixed when it starts,
and it must be the town the scenario drives in: every starter scenario with an ego, in
`basic/`, `multi_av/` and `awf/`, is Town01, so start the vehicle side with
`map_path:=$CARLA_MAPS/Town01`. `town02_episode_change` has no ego; it only makes CARLA load
Town02, and the next Town01 scenario loads Town01 back. A scenario whose ego drives another
town needs a vehicle side started with that town's map: with the wrong one, localization
never converges and the scenario fails waiting for the ego (section 6).

For your own:

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
example `src/csb_examples/scenarios/basic/town01_simulator_autopilot.xosc`). Name it `agent` and an autopilot of its
own drives it -- a second Autoware, say: start one more vehicle side for it with
`entity:=<its name>` ([design/scenario-authoring.md](design/scenario-authoring.md#vehicles-an-autopilot-drives-agent);
example `src/csb_examples/scenarios/multi_av/town01_two_av.xosc`). Scenario files are never modified.

## 4. Run

```bash
# Terminal 1 -- CARLA (headless; pin Vulkan to the NVIDIA card on hybrid-GPU hosts)
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  <CARLA_0.9.16>/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000

# Terminal 2 -- simulation side, long-lived: the bridge (restarted if it crashes) and the
# agent relay (TCP 5560), in the ROS domain scenarios will run in
ROS_DOMAIN_ID=9 play_launch launch --web-addr 127.0.0.1:8084 \
    csb_launch simulation.launch.xml \
    [config_file:=<scenario dir>/bridge.yaml] [weather:=ClearNoon]

# Terminal 3 -- vehicle side, any ROS domain, any host, any time
ROS_DOMAIN_ID=1 play_launch launch \
    --container-mode isolated --composable-respawn on-crash --web-addr 127.0.0.1:8082 \
    acb_launch carla_simulator.launch.xml \
    map_path:=$CARLA_MAPS/Town01 vehicle_name:=hero relay:=tcp://<simulation host>:5560 \
    [rviz:=false] [autoware_launch:=<your autoware.launch.xml>]

# Terminal 3b -- one more vehicle side per entity whose controller is `agent`, e.g. bg_av_1
ROS_DOMAIN_ID=2 play_launch launch \
    --container-mode isolated --composable-respawn on-crash --web-addr 127.0.0.1:8083 \
    acb_launch carla_simulator.launch.xml \
    map_path:=$CARLA_MAPS/Town01 vehicle_name:=bg_av_1 entity:=bg_av_1 \
    relay:=tcp://<simulation host>:5560 rviz:=false

# Terminal 4 -- one scenario, in the simulation side's domain; exits when it ends
ROS_DOMAIN_ID=9 play_launch launch \
    --web-addr 127.0.0.1:8081 \
    csb_launch scenario.launch.xml \
    scenario:=<scenario dir>/my_scenario.xosc [output_directory:=<dir>]
```

**When each part is ready.** Start them in this order and wait for each:

- CARLA serves its RPC port: `ss -lnt | grep ':2000 '` lists it. Its own log prints nothing
  useful; a cold headless start takes from under a minute (NVIDIA Vulkan) to about three.
- The simulation side's bridge logs `ZMQ server ready` and its relay `Relaying entity 'ego'`.
  Under play_launch these are in the per-node logs, not on the launch's console:
  `play_log/latest/node/carla_scenario_bridge/out` and
  `play_log/latest/node/scenario_agent_relay/err`. It waits for CARLA by itself if started
  first.
- The vehicle side logs `Startup complete: all nodes ready (nodes 63/63, containers 18/18,
  composable 127/127)` from play_launch (the counts depend on your Autoware; check that each
  is whole) -- and the relay then logs
  `entity 'ego': agent acb_agent registered`. About a minute on a warm host; the first
  start builds TensorRT engines and takes several.

**RViz.** The vehicle side opens Autoware's RViz on `$DISPLAY` by default. Pass `rviz:=false`
on a headless host or for a second vehicle side. It shows the ego, its route and the objects
it perceives while a scenario runs; between scenarios the ego is parked and its readouts
(speed, distance to goal) keep their last values -- that is not a frozen RViz.

The verdict is in `<output_directory>/scenario_test_runner/result.junit.xml`
(`output_directory` defaults to `/tmp`), beside SSv2's preprocessed copy of the scenario and
its logs. SSv2 empties that directory's subdirectories when a run starts and the next run
overwrites the verdict, so give every run you want to keep its own `output_directory`.

### Many scenarios: `run_suite`

```bash
# Terminal 4, instead of one scenario
export CARLA_MAPS=<map root>
ros2 run csb_launch run_suite \
    $(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/basic \
    --output ~/csb_results/basic \
    [--domain 9] [--relay tcp://<simulation host>:5560] [--timeout-per-scenario S] \
    [-- global_timeout:=900 ...]
```

Arguments are scenario files (`.xosc`, `.yaml`) or directories (every scenario file in them,
sorted, recursively). Each scenario runs to its end, one after another, through
`scenario.launch.xml` (with `play_launch`, or `ros2 launch` without it), in its own process
group and with its own `output_directory:=<output>/<scenario name>/`; arguments after `--`
go to every `scenario.launch.xml`. `--domain` (default `$ROS_DOMAIN_ID`, else 9) must be the
simulation side's domain.

Before each scenario, a pre-flight asks the agent relay (`--relay`, default
`tcp://localhost:5560`) whether the vehicle agent of every entity the scenario needs is
registered: the ego, and each entity whose controller is `agent`. If one is not, that
scenario fails at once (`Preflight: no vehicle agent registered ... as 'ego'`) instead of
waiting out the scenario's timeouts. `--no-preflight` skips the check.

`--timeout-per-scenario` caps one scenario file's wall-clock time, all its variants
together; a scenario that hits it is killed and its line says `TIMEOUT after N s`. There is
no cap by default (0): SSv2 already ends every variant at `global_timeout`, and one YAML
file can expand to many variants -- `awf/` UC-ACC is 27, which took over 2000 s. In CI set a
cap above the suite's longest file (`awf/`: UC-ACC took 2258 s, UC-AEB 541 s).

Each scenario prints one line (`PASS`/`FAIL`, testcases passed, time, the first failure),
and the results directory holds:

```
<output>/suite.junit.xml                      every testcase of every scenario (classname =
                                              scenario name; one per variant of a YAML
                                              scenario); a scenario with no verdict is an
                                              error saying why
<output>/<scenario>/scenario_test_runner/     its own result.junit.xml, preprocessed copy, logs
<output>/<scenario>/launch.log, play_log/     the launch's output
```

The exit status is 0 only if every testcase passed. Like a single scenario, a suite needs the
simulation side and the vehicle side(s) already up; scenarios that load another town
(`town02_episode_change`) make CARLA switch towns, and the next scenario switches back.

### Your own Autoware

`carla_simulator.launch.xml` is a CARLA profile around a stock `autoware.launch.xml` (yours,
via `autoware_launch:=`). It selects `vehicle_model:=acb_vehicle` and
`sensor_model:=acb_sensor_kit`, starts the vehicle interface (`acb_bridge`), and launches
three components with CARLA settings. Turn any of those off to keep your own:

| Argument | CARLA setting it applies |
|---|---|
| `carla_localization:=false` | CARLA-tuned NDT/EKF/pose-initializer parameters |
| `carla_perception:=false` | selectable `lidar_detection_model` (default `clustering`, CPU; `centerpoint` is TensorRT), fusion-only traffic lights (signals arrive from CARLA over V2X) |
| `carla_system:=false` | component-state topics, MRM parameters and diagnostic graph matching CARLA's sensors |
| `launch_deprecated_api:=false` | Autoware's deprecated API, which serves the velocity limit a scenario's absolute `SpeedAction` sets (the `awf/` use cases fail without it) |

The lidar detector defaults to `clustering` because it is the one that passes on CARLA: the
`awf/` standing-pedestrian use case went 9/9 with it and 2/9 with `centerpoint`
(2026-10-10, same host and build). `lidar_detection_model:=centerpoint` selects the other.

### Timeouts

A broken setup should fail in a minute or two, naming the cause, and a healthy run should
never sit in a fixed wait. These are the waits on the scenario path and what bounds them;
tune them where named.

| Wait | Where to tune | Old | New | Why |
|---|---|---|---|---|
| Vehicle agent registered? | `run_suite --relay`, `--no-preflight` | none (the scenario waited) | checked before each scenario, fails at once | an unregistered agent can only time out |
| Init through engage | `scenario.launch.xml initialize_duration:=` | 480 s | 120 s | scenario start to engage measured 52 s at worst (median ~20 s, 33 starts, basic/ and awf/); 480 dates from roadmap 012, when Autoware started with the scenario |
| First call to the ego's Autoware (`change_to_stop`) | follows `initialize_duration` | max(180 s, budget) | the budget | the relay offers it exactly while an agent is registered; the 180 s floor outlived its reason |
| Any other ADAPI service to appear | SSv2 fork | 180 s | 30 s | the relay's services are discovered in a second or two |
| Velocity limit retries | SSv2 fork | 30 x 3 s (90-180 s) | 5 (15-30 s) | a refusal means the vehicle side lacks the API (section 6), not a transient |
| Route retries | SSv2 fork | 30 x (10 + 10 s) | 5 | Autoware is waiting for a route when it is sent; a refused route stays refused |
| Clear route / engage / enable control / RTC retries | SSv2 fork | 30 x 3 s | 10 | rides out a transient refusal, fails a persistent one in about a minute |
| One simulator response (frame, spawn, despawn) | env `SIMULATOR_RESPONSE_TIMEOUT` | 420 s | 90 s | the bridge's own CARLA RPC timeout is 30 s (+10 s settings); a dead bridge is reported in 1.5 min |
| Simulator `Initialize` | env `SIMULATOR_INITIALIZE_TIMEOUT` | 420 s | 300 s | covers the CARLA wait below plus a town load (bridge's 120 s map-load RPC timeout) |
| CARLA gone at `Initialize` | `bridge.yaml carla.reconnect_wait_seconds` | 240 s | 120 s | a CARLA restart serves its map in 25-51 s when the host is not saturated (three restarts, 2026-10-10); raise it where CARLA restarts unattended and slowly |
| One scenario (variant) | `scenario.launch.xml global_timeout:=` | 600 s | 400 s | longest variant measured 190 s (town01_traffic_light, end to end) |
| One scenario file, all variants | `run_suite --timeout-per-scenario` | 1800 s | none (0) | SSv2 bounds each variant; a fixed cap killed UC-ACC (27 variants) in proof run 1 |

Unchanged, already short: the agent's own waits (10 s for fresh scans, 5 localization
initialize attempts 2 s apart, 30 s to converge on the teleported pose, 2 s for a command to
take effect), the relay's 10 s command timeout and the bridge's 15 s read timeout on it, and
the bridge's 120 s RPC timeout for a CARLA town load (measured 44-71 s).

## 5. Configure the CARLA world

The bridge owns the simulation side's CARLA connection, so world settings are ROS services
on it, in the simulation side's domain (`csb_interfaces`):

```bash
ROS_DOMAIN_ID=9 ros2 service call /carla/set_weather csb_interfaces/srv/SetWeather "{preset: ClearNoon}"
ROS_DOMAIN_ID=9 ros2 service call /carla/get_weather csb_interfaces/srv/GetWeather

# explicit values instead of a preset (fields not given are 0; use_parameters guards an
# empty request): cloudiness, precipitation, precipitation_deposits, wind_intensity,
# fog_density, fog_distance, wetness, sun_azimuth_angle, sun_altitude_angle
ROS_DOMAIN_ID=9 ros2 service call /carla/set_weather csb_interfaces/srv/SetWeather \
    "{use_parameters: true, weather: {cloudiness: 30.0, sun_altitude_angle: 60.0, fog_distance: 0.75}}"
```

Presets are CARLA's (`carla.WeatherParameters`): `ClearNoon`, `CloudyNoon`, `WetNoon`,
`WetCloudyNoon`, `SoftRainNoon`, `MidRainyNoon`, `HardRainNoon`, the same for `Sunset`
(`MidRainSunset`) and `Night` (`MidRainyNight`), and `DustStorm`; case is ignored. Both
services answer with the resulting values and the preset they match.

Every map load resets CARLA's weather to the town's default. The bridge keeps the weather
you asked for -- `weather:=` on `simulation.launch.xml` (or `weather:` in `bridge.yaml`),
then the last `set_weather` -- and applies it again after every map load and CARLA restart,
so it holds across scenarios on different towns. Empty (the default) leaves CARLA's own.

## 6. When things go wrong

- **CARLA dies**: the running scenario fails within ~30 s; restart CARLA; the next scenario
  waits for it (up to `carla.reconnect_wait_seconds`, 120 s) and runs.
- **CARLA just started**: it opens its RPC port before its first map is loaded; the bridge
  waits until CARLA reports a map ("CARLA is still loading its first map" in its log) before
  sending it anything, since settings or a map load in that window can hang CARLA for good.
- **The bridge dies**: the launch restarts it (`respawn="true"`); the running scenario fails
  within 90 s (`SIMULATOR_RESPONSE_TIMEOUT`), and the next one cleans up after it.
- **A scenario waits for the ego, or fails with "is not ready" / "agent relay"**: the vehicle
  side is not up, or its agent has not registered with the relay (`relay:=` wrong or
  unreachable). The relay logs every agent that registers; a single scenario proceeds as soon
  as one does and fails after `initialize_duration` (120 s) naming the relay; `run_suite`'s
  pre-flight fails the scenario at once.
- **A scenario fails in PLANNING or WAITING_FOR_ENGAGE** ("Simulator waited for the Autoware
  state to transition to WAITING_FOR_ENGAGE, but time is up"): the vehicle side is up but
  cannot plan or engage. Most often a node in it has exited: play_launch logs `Exited` and
  `Check play_log/.../node/<name>` for it, its web UI (port 8082 above) shows it stopped,
  and `play_log/<time>/node/<name>/err` says why. Restart the vehicle side; nothing on the
  simulation side needs restarting. `map_based_prediction` aborting with
  `Could not find 1` is a map without lanelet subtypes (section 2); `behavior_path_planner`
aborting is an Autoware bug that `--composable-respawn on-crash` recovers from (the scenario
it hits fails, the next one runs). Otherwise check that its
  `map_path` is the scenario's town (section 3).
- **"Requested the service /api/autoware/set/velocity_limit 5 times"**: the vehicle side
  lacks Autoware's deprecated API, which serves the velocity limit scenarios set with an
  absolute `SpeedAction` (every `awf/` scenario). `carla_simulator.launch.xml` starts it
  (`launch_deprecated_api:=true`, the default); a launch passing `false` loses it.
- **`pose_instability_detector` exits** (`cannot store a negative time point in
  rclcpp::Time`, in its `err`): an Autoware diagnostic node that does not survive the ego
  being teleported to a new scenario's start. It only feeds a localization diagnostic;
  driving and verdicts are unaffected.
- **Without a scenario**: leave `relay` unset (launch rejects an empty `relay:=`) and pass
  `goal_poses_file:=<yaml>` (`goal_pose: {x, y, qz, qw}`): the vehicle side drives to that
  goal on its own -- a quick check that Autoware and CARLA work. Nothing else may hold CARLA
  then (stop the simulation side: it restores CARLA to free-running when it exits), and the
  vehicle must exist, e.g. `ros2 launch acb_scenario single_vehicle_scenario.launch.xml`.
