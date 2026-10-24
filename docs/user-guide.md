# User Guide

Run OpenSCENARIO scenarios in CARLA against your own Autoware, with ROS tools only.
Background: [design/user-workflow.md](design/user-workflow.md).

Two sides run independently and meet only in CARLA and the map directory:

- **Simulation side**: CARLA, `scenario_world_bridge` (CARLA's adapter for
  scenario_simulator_v2), `scenario_agent_relay`, and scenario_simulator_v2 (SSv2), which runs
  your `.xosc` files and judges them.
- **Vehicle side**, one per autonomous vehicle: your Autoware with this project's sensor kit
  and vehicle interface, the CARLA vehicle bridge (`acb_bridge`) and the vehicle agent
  (`acb_pilot agent`), which takes goals from the scenario through the relay and drives
  Autoware's API locally.

They need not share a ROS domain, a host, or a start order.

Every command here is a plain `ros2 launch` or `ros2 run`. **Recommended:**
[play_launch](https://pypi.org/project/play-launch/), a drop-in for `ros2 launch` --
`play_launch launch <pkg> <file> <args...>`, same arguments -- which adds a web UI and
per-node logs (`play_log/<time>/node/<name>/{out,err}`), and restarts the nodes a launch file
declares `respawn="true"` (the bridge, here). Other nodes that crash stay down, except
composable nodes on the vehicle side, which `--composable-respawn on-crash` reloads. The
flags worth giving it:

| Flag | Where | Why |
|---|---|---|
| `--container-mode isolated --composable-respawn on-crash` | vehicle side | reloads a composable node whose process crashed. Autoware 1.5.0's `behavior_path_planner` aborts now and then when a new route arrives (`failed to add guard condition to wait set`, autoware_universe#12460; once in 27 UC-ACC variants here); without a reload the vehicle side cannot plan again and every later scenario fails in PLANNING. Autoware's containers declare no respawn, so only this brings it back, and a composable can be reloaded alone only when it is a process of its own (`isolated`; play_launch's default, `observable`, keeps one process per container as `ros2 launch` does) |
| `--container-mode observable` | vehicle sides, three or more on one host | the shared mode: one process per container. `isolated` costs a process per composable node; three isolated vehicle sides took about 47 of 48 cores on one host, enough to stall a pointcloud map load and to make CARLA's switch to Town05 outrun the bridge's 120 s timeout. Shared, the three cost about 9 cores and came up in about 15 s each. You lose the per-node reload: a crashed composable takes its container's other nodes with it until the side is restarted (section 6, "Several vehicle sides on one host") |
| `--web-addr 127.0.0.1:<port>` | every launch | each launch serves a web UI, 8080 by default; two on one port collide. This guide uses 8084 (simulation side), 8082 (vehicle side), 8083 (second vehicle side), 8081 (scenario) |

Everything else is play_launch's default, which since 0.15.1 matches `ros2 launch`: no
interception layer unless a contract asks for one, packages found the way `ament_index` finds
them, one process per container. Any `ros2 launch` command here works as well, without the
web UI and without the vehicle side's crash recovery -- which is what this guide shows.

## 1. Install

- CARLA 0.9.16 (vendor package), ROS 2 Humble, Autoware 1.5.0 (or your own build).
- Once per machine, the two things rosdep cannot install: Rust (rustup) and colcon's Rust
  plugin.

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
pip install colcon-cargo-ros2
```

- Optional, recommended: play_launch, the drop-in for `ros2 launch` (PyPI; 0.15.1 or newer -- 0.15.1 is the first to end a scenario stack when its runner exits under the default parser):

```bash
sudo apt install libz3-dev ros-humble-rclcpp-components ros-humble-class-loader
pip install 'play_launch>=0.15.1'      # installs ~/.local/bin/play_launch
play_launch --version
```

  Optional, once: `play_launch setcap` (sudo) lets it record per-process I/O. Without it each
  launch warns `play_launch_io_helper lacks cap_sys_ptrace`; that is harmless.

- This repository with its submodules, its system dependencies, and one build:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/scenario-world-bridge.git
cd scenario-world-bridge
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
  `weather:=ClearNoon` for a clear sky ([5. Configure the world's weather](#5-configure-the-worlds-weather)).

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
            carla/map_origin.yaml       (only for a map not in CARLA's frame)
```

`carla/traffic_lights.yaml` (which Lanelet2 signal is which CARLA light) is generated once
per map, with CARLA running:

```bash
ros2 run scenario_world_bridge scenario_world_bridge --generate-signal-table <map dir>
```

The bridge checks it at every scenario start and refuses to run with a table that no longer
matches CARLA. The map directory is read-only at run time.

**A map not drawn in CARLA's frame** needs one more file. The TUM towns' Lanelet2 maps use
CARLA's world coordinates, so map and CARLA share an origin. A surveyed map does not: its
Lanelet2 coordinates are grid metres tens of kilometres from zero, and its OpenDRIVE is
converted with an offset taken off so CARLA's world stays near its origin. Record that offset,
the map-frame position of CARLA's world origin, in `carla/map_origin.yaml`:

```yaml
# <map dir>/carla/map_origin.yaml -- metres, map frame; no rotation
x: 52600.0
y: 67800.0
z: 0.0       # optional
```

Both bridges read it (scenario_world_bridge at every scenario start, acb_bridge through
`map_origin_path`, which `carla_simulator.launch.xml` points at `<map_path>/carla/`). No file
means an origin of zero; a file that does not parse stops the run rather than putting every
pose hundreds of metres off.

**Where the maps come from.** This project runs on the CARLA towns converted for Autoware by
TUM's [carla-autoware-bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge) (map pack
copyright 2024 TUM, LGPL-3.0; its `LICENSE.md` ships in each town directory). The pack's
download link (`scripts/download_maps.sh`) is dead at the time of writing, so there is no
public download yet; providing the maps is a separate effort. With a copy of the pack,
`SWB_MAP_SOURCE=<pack dir> scripts/download_maps.sh` installs it under
`data/carla-autoware-bridge/<Town>/` with the projector fix SSv2 needs; then generate each
town's `carla/traffic_lights.yaml` as above. The script also repairs six defects of the
pack, idempotently (`scripts/repair_lanelet_traffic_lights.py`, `scripts/repair_lanelet_subtypes.py`,
`scripts/repair_lanelet_local_coords.py`, `scripts/repair_lanelet_turn_direction.py`,
`scripts/repair_lanelet_joins.py`, `scripts/repair_lanelet_right_of_way.py`; run them on a copy you
installed another way):
traffic lights whose type tags are empty (invisible to Autoware and SSv2); lanelets with no
`subtype` tag -- the kerb strips beside every road -- which make Autoware's
`map_based_prediction` abort as soon as a pedestrian is near one
(`lanelet::NoSuchAttributeError: Could not find 1`); and stop lines and signs whose nodes
carry only lat/lon, which Autoware projects up to 2.4 m away from where the pack meant them;
and junction lanelets with no `turn_direction` tag, without which Autoware's `intersection`
and `blind_spot` modules never launch (each lanelet that leaves a fork or joins a merge is
tagged `straight`, `left` or `right` from its heading change); and road lanelets that start
at fresh nodes exactly where their predecessor ends, which Lanelet2 does not treat as a
successor, so routing stops there (one join in Town03, between the roundabout and its south
exit); and `right_of_way` elements that name no lanelet with right of way (one each in Town05
and Town10), over which scenario_simulator_v2 refuses to load the map. One more repair needs CARLA holding the town and something ticking it (swb during a
scenario): `scripts/repair_lanelet_signal_links.py <Town>` links every signalised approach
lanelet to the regulatory element of the light CARLA controls it by (from each light's stop
waypoints and the signal table) and gives that element a stop line at the stop waypoints. On
the pack's Town05 most approach lanelets referenced no light or a neighbouring approach's, so
Autoware crossed signalised junctions as unsignalised; on its Town01, 28 of the 36 approaches
had no link to their light in their own direction, and the ego drove through a red at 4 m/s
(roadmap 051). Run it (at `fe33862` or later) for every town you drive, and restart the vehicle
sides after it: a stack started before keeps the old map. In the same session, for a town
aib will drive on its own (no SSv2 to command the lights), `scripts/generate_signal_groups.py
<Town> --port <CARLA port>` writes CARLA's light groups to `<map dir>/carla/signal_groups.yaml`,
which aib's `just isaac-ego signals:=<map dir>` cycles as CARLA does.
Keep the towns side by side in one directory --
the scenarios below find them as `$(env CARLA_MAPS)/<Town>`:

```bash
export CARLA_MAPS=<repo>/data/carla-autoware-bridge     # holds Town01/, Town02/, ...
```

## 3. Prepare a scenario directory

**Starter scenarios** are installed with the workspace, no scenario directory needed
(package `swb_examples`; each scenario, its town and what it checks are listed in its
[README](../src/swb_examples/README.md)):

```
$(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/
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
`map_path:=$CARLA_MAPS/Town01`. A scenario whose ego drives another town needs a vehicle
side started with that town's map: with the wrong one, localization never converges and the
scenario fails waiting for the ego (section 6). `just scenario` does this for you: it
replaces a running ego stack whose map is not the scenario's.

**A town change ends the vehicle side** (roadmap 051). Loading a world starts a new episode
of the simulator, whose time starts again, and Autoware cannot follow time back. So on CARLA
every vehicle side of the old episode ends at the load: `acb_bridge` exits (status 3,
naming the change) and its launch takes the Autoware stack with it; a restarted CARLA does
the same. The stack comes back when something restarts it: `just scenario` and `just e2e`
start the ego stack under a supervisor (`swb-agent supervise`) that does, and the next
scenario waits for its agent. `town02_episode_change` has no ego; it only makes CARLA load
Town02, ending the ego stack, and the next Town01 scenario loads Town01 back, ending it once
more. Each town change costs a stack start (about a minute here); keep a suite's scenarios on
one town where you can.

For your own:

```
<scenario dir>/  my_scenario.xosc
                 bridge.yaml          # optional: copy of scenario_world_bridge's config
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
example `src/swb_examples/scenarios/basic/town01_simulator_autopilot.xosc`). Name it `agent` and an autopilot of its
own drives it -- a second Autoware, say: start one more vehicle side for it with
`entity:=<its name>` ([design/scenario-authoring.md](design/scenario-authoring.md#vehicles-an-autopilot-drives-agent);
example `src/swb_examples/scenarios/multi_av/town01_two_av.xosc`). Scenario files are never modified.

## 4. Run

```bash
# Terminal 1 -- CARLA (headless; pin Vulkan to the NVIDIA card on hybrid-GPU hosts)
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  <CARLA_0.9.16>/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000

# Terminal 2 -- simulation side, long-lived: the CARLA world server, the scenario bridge
# driving it and the agent relay (TCP 5560), each restarted if it dies, in the ROS domain
# scenarios will run in
ROS_DOMAIN_ID=9 ros2 launch swb_launch carla_simulation.launch.xml \
    [config_file:=<scenario dir>/bridge.yaml] [weather:=ClearNoon] [npc_motion:=kinematic]

# Terminal 3 -- vehicle side, any ROS domain, any host, any time
ROS_DOMAIN_ID=1 ros2 launch acb_launch carla_simulator.launch.xml \
    map_path:=$CARLA_MAPS/Town01 vehicle_name:=hero relay:=tcp://<simulation host>:5560 \
    [rviz:=false] [autoware_launch:=<your autoware.launch.xml>]

# Terminal 3b -- one more vehicle side per entity whose controller is `agent`, e.g. bg_av_1
ROS_DOMAIN_ID=2 ros2 launch swb_launch background_av.launch.xml \
    entity:=bg_av_1 map_path:=$CARLA_MAPS/Town01 relay:=tcp://<simulation host>:5560

# Terminal 4 -- one scenario, in the simulation side's domain; exits when it ends
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=<scenario dir>/my_scenario.xosc [output_directory:=<dir>]
```

With play_launch instead, put `play_launch launch` and the flags from the table at the top
in place of `ros2 launch`.

**Two processes, two configurations** (roadmap 046). The simulation side runs the scenario
bridge, which speaks SSv2's protocol and knows no simulator, and a world server per
simulator, which drives the simulator over the world protocol (`swb_world.proto`, in
`swb_world_protocol`). On CARLA the world server (`swb_carla_world`) runs inside the same
launch as the bridge; on Isaac Sim it is its own long-lived launch (`just isaac-world`, then
`isaac_simulation.launch.xml` for the simulation side). Each has its file, and each refuses the other's keys, naming
where they went:

- The bridge's `bridge_config.yaml` (the world server's endpoint and timeouts, the asset map,
  the agent relay, `npc_motion`, gap traces and contact reports), chosen by the
  launch argument `config_file:=` (`just run config_file:=<dir>/bridge.yaml`), default the
  installed `share/scenario_world_bridge/config/bridge_config.yaml`.
- The CARLA world server's `swb_carla_world.yaml` (CARLA's host and port, blueprints and map
  aliases, ticking, the Traffic Manager, the collision sensors, the sensor-release channel,
  the weather preset), chosen by `swb_carla_world_config:=`, default
  `share/swb_carla_world/config/swb_carla_world.yaml`; `blueprint_boxes_<version>.yaml` is
  read beside it. Its ROS parameter `vehicle_physics` names the ego's physics (default the
  installed `sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml`; `none` keeps the
  blueprint's).

Both log the file they loaded. `ros2 param get /scenario_world_bridge config_file` names the
bridge's. The older `CSB_CONFIG_DIR` variable is refused.

**Is a world server up?** `just world-health [sim:=carla|isaac]` sends it one probe (which
changes nothing) and prints its simulator, what it offers, its owner and whether it is ready.
One bridge drives a world server at a time: a second bridge is refused while the first lives.
`just world-check` runs the conformance check between scenarios (it takes the server over).

**When each part is ready.** Start them in this order and wait for each:

- CARLA serves its RPC port: `ss -lnt | grep ':2000 '` lists it. Its own log prints nothing
  useful; a cold headless start takes from under a minute (NVIDIA Vulkan) to about three.
- The simulation side's bridge logs `ZMQ server ready` and its relay `Relaying entity 'ego'`.
  Under play_launch these are in the per-node logs, not on the launch's console:
  `play_log/latest/node/scenario_world_bridge/out` and
  `play_log/latest/node/scenario_agent_relay/err`. The world server waits for CARLA by itself
  if started first (answering `just world-health` with "not ready" meanwhile), and the bridge
  binds the SSv2 port only once its world server is ready.
- The vehicle side is ready when the relay (Terminal 2) logs
  `entity 'ego': agent acb_agent registered` and then `ego: agent up`. Under play_launch the
  vehicle side also logs `Startup complete: all nodes ready (nodes 63/63, containers 18/18,
  composable 127/127)` (the counts depend on your Autoware; check that each is whole). About
  a minute on a warm host; the first start builds TensorRT engines and takes several (every
  start does when your TensorRT is not 10.8.0; see below).

**RViz.** The vehicle side opens Autoware's RViz on `$DISPLAY` by default. Pass `rviz:=false`
on a headless host (no `DISPLAY`, or one you do not own) and for every vehicle side after the
first. Without it a vehicle side on a host with no display of its own did not come up
(2026-10-12, a headless shared server), and each extra RViz costs CPU and GPU memory for a
window nobody watches. It shows the ego, its route and the objects
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
ros2 run swb_launch run_suite \
    $(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/basic \
    --output ~/swb_results/basic \
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
(`town02_episode_change`) make CARLA switch towns, and the next scenario switches back. Each
switch ends the vehicle sides (above): run them under `swb-agent supervise`, and run_suite
waits for a supervised agent whose stack ended to register again before the next scenario.

### CARLA's contacts in the verdict: `--physical-verdict`

SSv2 decides collisions in its own geometry: declared bounding boxes, in 2D, and no static
world. When the ego hits a curb, a pole, or an NPC whose CARLA body is bigger than its
declared box, the run can still pass. The bridge writes what CARLA's collision sensor saw
for every ego run to `~/.ros/swb/contacts/` (`contact_report_dir:=` on `carla_simulation.launch.xml`
moves it, `off` disables it). With `--physical-verdict`, `run_suite` adds a `carla_contact`
testcase to each scenario, next to SSv2's own:

| `carla_contact` | When |
|---|---|
| pass | no contact, or only the contacts the scenario declares |
| failure | CARLA saw the ego touch something else; the message names it, with the SSv2 frame and the impulse |
| error | no report for the run (wrong bridge, reports off), or CARLA restarted mid-run |

A scenario that is meant to make contact declares it, and SSv2 ignores the parameter:

```xml
<ParameterDeclaration name="csbExpectedContacts" parameterType="string" value="npc_rear" />
```

The value is a comma-separated list of entity names. `static` allows contact with the static world.
`--min-impulse N` ignores contacts below N N·s. For one scenario, run the join after it ends:

```bash
ros2 run swb_launch physical_verdict <output>/scenario_test_runner/result.junit.xml \
    --scenario <scenario file>       # judges the newest report; --since/--until for a window
```

SSv2's own testcases are never changed. A run SSv2 passed with a contact therefore shows up
as one pass and one failure in the same result.

The bridge gives each NPC and prop the CARLA body closest in size to its declared box, within
its `vehicleCategory` (a truck gets a truck), and keeps the Model 3 for any car box it fits.
A prop declared across its length (a street barrier set across the road) is spawned turned
90° to match; vehicles are never turned.
Where CARLA has nothing close (a 12 m truck: its longest is 8.5 m), the testcase's
`system-out` lists the body with its difference from the declared box; that never fails it.
To force a body, set the entity's `model3d` to a CARLA blueprint name, or map it in
`blueprint_map` (`bridge_config.yaml`).

### How NPCs move: `npc_motion`

SSv2 computes every NPC's pose; the bridge decides how CARLA's body follows it.
`npc_motion:=` on `carla_simulation.launch.xml` (or `npc_motion:` in `bridge.yaml`) selects:

| `npc_motion` | What CARLA has | Choose it for |
|---|---|---|
| `tracked` (default) | NPC vehicles and pedestrians are physics bodies driven toward SSv2's pose each frame: within 0.2 m p95 of it, snapped back past 0.5 m or 5°. CARLA reports their real velocity (radar, anything that reads it), walkers animate, a hit NPC moves, and an NPC that rams the ego pushes with finite force -- its declared acceleration plus a margin -- not as a wall. | realistic contacts and sensors; the normal case |
| `kinematic` | physics off, teleported to SSv2's pose each frame: exact, but CARLA reads 0 m/s, walkers glide, a hit NPC never moves and two NPCs pass through each other | exact poses, or the cheapest CARLA tick (20 tracked NPCs cost ~7 ms more per tick at p95, of a 50 ms frame) |

Props (`MiscObject`) and vehicles driven by `simulator_autopilot` or an `agent` are not
affected. Under `tracked`, the bridge logs each NPC's tracking error and snaps at the end of
the ego's run (`Tracked '<name>': ...`) and adds them to the contact report as `tracking`.
An NPC that drives into the ego keeps pushing for as long as SSv2 commands it on: make it
brake once the collision is seen, as `basic/town01_rear_contact.xosc` does.

### Your own Autoware

Both vehicle sides launch the same **Autoware profile**, `autoware_sim_profile`'s
`autoware.launch.xml` (in `autoware_sim_vehicle`), around a stock `autoware.launch.xml`
(yours, via `autoware_launch:=`): acb's `carla_simulator.launch.xml` is the profile plus
`acb_bridge`, aib's `isaac_simulator.launch.xml` the profile alone. It selects the car's
generated packages, `vehicle_model:=tesla_model3` and `sensor_model:=tesla_model3_sensor_kit`,
and launches components with simulation settings. Turn any of those off to keep your own:

| Argument | Simulation setting it applies |
|---|---|
| `sim_localization:=false` | simulation-tuned NDT/EKF/pose-initializer parameters |
| `sim_perception:=false` | selectable `lidar_detection_model` (default `clustering`, CPU; `centerpoint` is TensorRT), fusion-only traffic lights (signals arrive from the simulator over V2X) |
| `sim_system:=false` | component-state topics, MRM parameters and diagnostic graph matching the simulated sensors |
| `sim_control:=false` | upstream's MPC parameters (the profile turns off steering-offset learning) |
| `sim_planning:=false` | `max_velocity:=<m/s>`, Autoware's `max_vel` (default 4.17, upstream's 15 km/h, at which planning is exactly upstream's); under `just ego-av`, `just ego-av max_velocity:=11.11` for 40 km/h |
| `sim_clock_guard:=false` | the clock guard, which ends the stack when `/clock` goes back (a world load, a restarted simulator) |
| `launch_deprecated_api:=false` | Autoware's deprecated API, which serves the velocity limit a scenario's absolute `SpeedAction` sets (the `awf/` use cases fail without it) |

The arguments these replaced (`carla_localization`, `carla_control`, ...) and the old car
(`vehicle_model:=acb_vehicle`) are refused by name by every `just` wrapper (the profile's
`check_launch_args`); a plain `ros2 launch` would ignore them silently.

The lidar detector defaults to `clustering` because it is the one that passes on CARLA: the
`awf/` standing-pedestrian use case went 9/9 with it and 2/9 with `centerpoint`
(2026-10-10, same host and build). `lidar_detection_model:=centerpoint` selects the other.

Autoware 1.5.0's `blind_spot` module never stops for a cyclist on the inside of a **right**
turn (autoware_universe#13509: its ego passage interval comes out reversed). The fix is one
package, built from source over your Autoware: the developer workflow does it with `just
build-autoware-overlay` (roadmap 042); by hand, build
`autoware_behavior_velocity_blind_spot_module` from branch `csb/blind-spot-right-turn-0.48.0`
of github.com/NEWSLabNTU/autoware_universe in a workspace of its own and source that
workspace's `install/local_setup.bash` right after Autoware's setup.

Autoware 1.5.0 was compiled against TensorRT **10.8.0**, and it rebuilds any cached engine
that another TensorRT version made. If the first `libnvinfer.so.10` your loader finds is
another version, for example CUDA 12.8's 10.9 under `/usr/local/cuda`, CenterPoint rebuilds
its engines at every start. That takes minutes, and its log says `Plan was created with a
different version of TensorRT`. To avoid it, give the vehicle side a 10.8.0 runtime. Fetch
NVIDIA's `tensorrt-cu12-libs==10.8.0.43` wheel from `https://pypi.nvidia.com` and unzip its
`tensorrt_libs/` directory anywhere. Then put that directory first on `LD_LIBRARY_PATH` in
the vehicle side's terminal, after sourcing the setups. The developer workflow does this with
`just setup-tensorrt` and the `AUTOWARE_TENSORRT_LIB` host setting.

### Another autopilot

The vehicle side does not have to be Autoware. Anything that registers with the relay as
the entity's agent drives it: a Python class (`swb_agent.Agent`) in your own project, or a
ROS 2 autopilot described in a YAML file (`ros2 run swb_agent ros_adapter --config
agent.yaml`). Start one with `ros2 run swb_agent swb-agent new <name>` and test it with
`swb-agent check` before running scenarios: [Writing an agent](agent-interface.md).

### Another simulator

Every simulator is a world server behind the world protocol (`swb_world.proto` in
`swb_world_protocol`;
roadmap 046, [design](design/simulator-interface-split.md)); the bridge needs nothing new for
one. A port is one process that owns the simulator and implements the protocol:

- **Python**: subclass `swb_world_server.world.World` (`load`, `spawn`, `despawn`, `step`;
  `set_signals` and the rest with a capability), set `simulator_name`, `capabilities` and
  `max_physics_dt`, and serve it with `swb_world_server.server.bind` and `run`. The Isaac
  world server (`swb_isaac_world/swb_isaac_world/world.py`) is the worked example.
- **Rust**: implement `swb_core::world_server::World` and serve it with `world_server::bind`
  and `serve`, as `swb_carla_world` does for CARLA.
- **Anything else**: answer the `.proto`'s `Request` with its `Reply` on a ZMQ REP socket.

Declare only what the simulator does (`Capability`): the bridge never sends a request or a
drive the server did not declare, and answers SSv2 with an honest failure naming the missing
capability. A map tells the server which world it means in `<map dir>/<simulator name>/world.yaml`
(`world`, `ground_paths`, `map_origin`). Check the server with
`swb_world_check conformance tcp://127.0.0.1:<port> --map <map dir>` before a scenario, then
run the starter scenarios against it: start the simulation side with
`ros2 launch swb_launch simulation.launch.xml world_endpoint:=tcp://127.0.0.1:<port>`, then
scenarios as above. The vehicle under test needs a vehicle bridge of its own
(sensors, `/vehicle/status/*`, `/control/command/*`, `/clock`), as acb and aib are.

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

## 5. Configure the world's weather

The weather is a world server's, set through ROS services on the bridge, in the simulation
side's domain (`swb_interfaces`): a preset, or parameters by name, both in the world
server's own names. On CARLA these are CARLA's presets and `WeatherParameters` fields
(`swb_carla_world`'s README lists them); Isaac's world server offers no weather service (a
scenario's own `Weather` reaches it through SSv2).

```bash
ROS_DOMAIN_ID=9 ros2 service call /scenario_world_bridge/set_weather swb_interfaces/srv/SetWeather "{preset: ClearNoon}"
ROS_DOMAIN_ID=9 ros2 service call /scenario_world_bridge/get_weather swb_interfaces/srv/GetWeather

# parameters instead of a preset; those not named keep their current values
ROS_DOMAIN_ID=9 ros2 service call /scenario_world_bridge/set_weather swb_interfaces/srv/SetWeather \
    "{names: [cloudiness, sun_altitude_angle], values: [30.0, 60.0]}"
```

CARLA's presets: `ClearNoon`, `CloudyNoon`, `WetNoon`, `WetCloudyNoon`, `SoftRainNoon`,
`MidRainyNoon`, `HardRainNoon`, the same for `Sunset` (`MidRainSunset`) and `Night`
(`MidRainyNight`), and `DustStorm`; case is ignored. Both services answer with every
parameter and the preset they match. An unknown preset or parameter is refused with the
valid names, and an empty request is refused.

Every map load resets CARLA's weather to the town's default. The world server keeps the
weather you asked for -- `weather:` in `swb_carla_world.yaml` (checked at start), or the
bridge's `weather:=` on `carla_simulation.launch.xml`, then the last `set_weather` -- and
applies it again after every map load and CARLA restart, so it holds across scenarios on
different towns. Empty (the default) leaves CARLA's own.

## 6. When things go wrong

- **Nothing ever connects** -- the relay never sees the vehicle's agent, `ros2 topic list`
  is empty, a scenario waits for a simulator that is running: DDS discovery is failing. On
  hosts where multicast is disabled on the loopback interface (common on shared servers),
  use the unicast-on-loopback profile that ships with the workspace, in **every** terminal on
  both sides:
  `export CYCLONEDDS_URI=file://<workspace>/config/cyclonedds-localhost.xml`.
  It keeps all participants on one host; across hosts, use your network's DDS setup.
- **The vehicle side never localizes; the pointcloud map never arrives**: the side runs on
  another RMW than CycloneDDS, or on CycloneDDS without the profile above. Under Humble's
  default RMW (Fast DDS) the pointcloud map, one large transient-local message, never
  reached its subscribers on a shared host (2026-10-12), so NDT never initialised and the
  ego never came up. Run every process of every side with
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the same `CYCLONEDDS_URI`; an Autoware
  install that exports another RMW in its `setup.bash` overrides an earlier export, so set
  both after sourcing it. Large messages also need kernel socket buffers bigger than the
  stock 208 kB: install `config/99-cyclonedds-max.conf` (the values of Autoware's DDS
  settings page) as root,
  `sudo install -m 644 config/99-cyclonedds-max.conf /etc/sysctl.d/ && sudo sysctl -p
  /etc/sysctl.d/99-cyclonedds-max.conf`. Without it a point cloud fragments into more
  datagrams than a socket holds, which shows as `RcvbufErrors` in `netstat -su` and
  publishers that stall.
- **Several vehicle sides on one host**: start them one at a time, each after the previous
  one logs `Startup complete`, in play_launch's shared container mode
  (`--container-mode observable`) and with `rviz:=false` on all but one. Three sides
  starting together, in `isolated` mode, saturated a 48-core host: the pointcloud map load
  stalled and the first switch of CARLA to Town05 outran the bridge's 120 s timeout. A map
  load that times out can leave CARLA holding the town without its traffic-light actors;
  load it again with a long client timeout and regenerate the town's signal table
  (section 2). Under `just ego-av`, `container_mode:=observable` selects the shared mode.
  On a host with several GPUs, `gpu:=<index>` on a vehicle side's launch pins its CUDA
  nodes to one card (`CUDA_VISIBLE_DEVICES`; empty, the default, inherits the
  environment's); under `just ego-av`, `just ego-av gpu:=1`. `extra_launch:=<file>`
  starts a launch file of your own beside a vehicle side, in its domain (a V2X node, a
  recorder); it receives `vehicle_name`, `entity`, `map_path` and `use_sim_time`.
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
