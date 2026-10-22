# Tutorial 1: First run

Bring up CARLA, the simulation side and an Autoware vehicle side, run one of the starter
scenarios, and read its verdict. About 20 minutes, most of it waiting for Autoware to start.

**Before you start**: the [README](../../README.md)'s prerequisites and setup are done (the
workspace is built), and you have converted maps (a directory holding `Town01/`, `Town02/`,
...; [User Guide §2](../user-guide.md#2-prepare-a-map-directory)).

## 0. Every terminal

Each terminal below starts with the same three lines. Put them in a file and `source` it:

```bash
source /opt/autoware/1.5.0/setup.bash                  # your Autoware
source <your clone>/install/setup.bash                 # the workspace built in the README's Setup
export CARLA_MAPS=~/maps/carla                         # holds Town01/, Town02/, ...
# only if DDS discovery fails on this host (see "When it does not work"):
# export CYCLONEDDS_URI=file://<your clone>/config/cyclonedds-localhost.xml
```

Check the map directory has what the scenario and Autoware read:

```bash
ls $CARLA_MAPS/Town01
# carla  lanelet2_map.osm  map_config.yaml  map_projector_info.yaml
# pointcloud_map.pcd  pointcloud_map_metadata.yaml
ls $CARLA_MAPS/Town01/carla
# traffic_lights.yaml
```

If `carla/traffic_lights.yaml` is missing, generate it once with CARLA running (step 1):
`ros2 run scenario_world_bridge scenario_world_bridge --generate-signal-table $CARLA_MAPS/Town01`.

## 1. CARLA

```bash
# Terminal 1
~/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
```

It prints little. It is ready when it serves its port, from under a minute to about three:

```bash
ss -lnt | grep ':2000 '
# LISTEN 0  ...  0.0.0.0:2000  ...
```

On a laptop or a host with two GPUs, put
`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json` in front so CARLA renders on
the NVIDIA card.

## 2. The simulation side

The bridge (CARLA's adapter for SSv2) and the agent relay (which passes the scenario's
goals to the vehicle). Start it once and leave it running; scenarios come and go.

```bash
# Terminal 2
ROS_DOMAIN_ID=9 ros2 launch swb_launch simulation.launch.xml
```

Ready when both of these lines have appeared:

```
[scenario_world_bridge-1] ... INFO scenario_world_bridge::zmq_server: ZMQ server ready, waiting for SSv2 requests...
[relay-2] [INFO] ... [scenario_agent_relay]: Relaying entity 'ego': agents register on tcp://0.0.0.0:5560; ...
```

`ROS_DOMAIN_ID=9` is the domain scenarios run in; the scenario in step 4 uses the same.

## 3. The vehicle side

Autoware with this project's CARLA sensor kit and vehicle interface, for the CARLA vehicle
named `hero`, registering with the relay. It runs in its own ROS domain -- the two sides
talk only through CARLA and the relay.

```bash
# Terminal 3
ROS_DOMAIN_ID=1 ros2 launch acb_launch carla_simulator.launch.xml \
    map_path:=$CARLA_MAPS/Town01 vehicle_name:=hero relay:=tcp://localhost:5560
```

This starts ~90 processes and opens RViz (add `rviz:=false` to skip it). Ready when Terminal
2's relay reports the vehicle's agent:

```
[relay-2] [INFO] ... entity 'ego': agent acb_agent registered from 127.0.0.1:...
[relay-2] [INFO] ... ego: agent up -- ADAPI services offered
```

The first start on a machine builds Autoware's TensorRT engines and can take several
minutes. Later starts take about one, as long as the TensorRT that loads is the 10.8.0
Autoware was built with. With any other version every start rebuilds. The fix is in the
[user guide](../user-guide.md) (section 4, TensorRT 10.8.0) or `just setup-tensorrt`.

`map_path` must be the town the scenario drives in. Every starter scenario with an ego is
Town01.

## 4. A scenario

```bash
# Terminal 4
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=$(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/basic/town01_ego_drive.xosc \
    output_directory:=/tmp/first_run
```

`town01_ego_drive` gives the ego a goal 70 m down the road. What you see:

- RViz (Terminal 3's window): the route appears, the ego engages and drives to the goal.
- Terminal 2: the bridge spawns the ego (`Spawned ego 'ego' ... blueprint='vehicle.tesla.model3'`).
- Terminal 4: the scenario's own processes; the launch **ends by itself** when the scenario
  does, after about a minute and a half.

## 5. The verdict

SSv2 writes a JUnit file:

```bash
cat /tmp/first_run/scenario_test_runner/result.junit.xml
```

```xml
<testsuites name="/tmp/first_run/scenario_test_runner" failures="0" errors="0" tests="1">
  <testsuite name="town01_ego_drive" failures="0" errors="0" tests="1">
    <testcase name="town01_ego_drive" />
```

`failures="0" errors="0"`: passed. A failed run has a `<failure message="...">` with the
reason -- a timeout, a collision, or the ego never engaging.

Run another starter scenario the same way: Terminals 1-3 stay up. The list, with what each
checks, is in [swb_examples' README](../../src/swb_examples/README.md).

## When it does not work

| Symptom | Look at |
|---|---|
| Terminal 4 waits and fails with `... waited for the Autoware state ...` | the vehicle side never registered or never localized: Terminal 2 must show `ego: agent up`; Terminal 3's `map_path` must be the scenario's town |
| The bridge prints `CARLA version mismatch` and retries | CARLA and the build disagree on the version (README, *Other CARLA versions*) |
| Nothing ever connects | DDS discovery fails on this host (multicast off on loopback): uncomment the `CYCLONEDDS_URI` line of step 0 in every terminal; [User Guide §6](../user-guide.md#6-when-things-go-wrong) |

## Stopping

Ctrl-C in each terminal: the vehicle side(s) first, then the simulation side; CARLA last,
if at all (it serves every run, and restarting it takes minutes). An Autoware vehicle side can take up to a minute to wind down
its ~90 processes; if a launch is still running after that, press Ctrl-C again. The
simulation side's bridge is declared `respawn="true"` and may come back once while the launch
shuts down; a second Ctrl-C ends it.

## Use play_launch instead

[play_launch](https://pypi.org/project/play-launch/) runs the same launch files with the
same arguments, and gives each node its own log file and the stack a web UI -- worth it for
an Autoware stack of ~90 processes:

```bash
pip install 'play_launch>=0.15.1'
ROS_DOMAIN_ID=1 play_launch launch --web-addr 127.0.0.1:8082 \
    acb_launch carla_simulator.launch.xml map_path:=$CARLA_MAPS/Town01 vehicle_name:=hero \
    relay:=tcp://localhost:5560
# then open http://127.0.0.1:8082 ; logs in ./play_log/
```

Next: [Tutorial 2, prepare a scenario](02-prepare-a-scenario.md).
