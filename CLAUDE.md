# carla-scenario-bridge

## Project Overview

ZMQ+Protobuf adapter that makes CARLA a backend for tier4/scenario_simulator_v2 (SSv2). Implements SSv2's `simulation_interface` protocol, translating protobuf requests into CARLA API calls via carla-rust.

**Design doc**: `docs/design/architecture.md`
**Roadmap**: `docs/roadmap/`
**Parent project**: [autoware_carla_bridge](https://github.com/NEWSLabNTU/ros_zenoh_bridge) (sensor/vehicle interface layer)
**SSv2 protocol reference**: See `proto/` directory

## Build & Run

```bash
just build    # Build with colcon + cargo
just clean    # Clean artifacts
just run      # Run the bridge (connects to CARLA, listens for SSv2)
just check    # Format + clippy
just test     # Run tests
```

## Key Design Decisions

- **AWSIM pattern**: SSv2 puppeteers NPCs via `set_transform()`, CARLA owns ego physics
- **Sensors**: Reject SSv2 `AttachSensor` requests (`Success=false`); CARLA sensors publish via autoware_carla_bridge
- **Traffic lights**: Freeze CARLA's built-in cycling, set states per SSv2 commands
- **Coordinate conversion**: SSv2 uses ROS right-handed frame; CARLA uses left-handed (Y-flip)
- **Vehicle interface**: autoware_carla_bridge handles `/vehicle/status/*` and `/control/command/*`; SSv2's `AutowareUniverse` (concealer) should be disabled

## Repository Structure

```
.
├── src/carla_scenario_bridge/  # Rust crate (ament_cargo package)
│   ├── src/
│   │   ├── main.rs             # Entry point, CARLA retry loop, shutdown
│   │   ├── zmq_server.rs       # ZMQ REP socket, protobuf dispatch
│   │   ├── coordinator.rs      # 14 SSv2 handlers → CARLA API calls
│   │   ├── entity_manager.rs   # SSv2 name ↔ CARLA actor ID mapping
│   │   ├── coordinate_conversion.rs  # ROS ↔ CARLA frame conversion
│   │   ├── traffic_light_mapper.rs   # Lanelet signal ID ↔ CARLA actor (Phase 4)
│   │   └── proto.rs            # Generated protobuf type re-exports
│   ├── build.rs                # prost-build proto compilation
│   ├── config/bridge_config.yaml
│   ├── Cargo.toml
│   └── package.xml
├── proto/                  # SSv2 protobuf definitions (8 .proto files)
├── scenarios/              # Example OpenSCENARIO test files
├── docs/
│   ├── design/             # Architecture, protocol, launch config docs
│   └── roadmap/            # Phase 1-5 roadmap with task checklists
├── Cargo.toml              # Workspace root
├── justfile                # Build and run commands
└── CLAUDE.md               # This file
```

## Coding Practices

### Use play_launch instead of ros2 launch
All launch commands use `play_launch launch` (with `--web-addr` for the web UI), not `ros2 launch` directly.

### Use just build
Always use `just build` instead of `colcon build` directly (ensures `--symlink-install`).

### Never build or check with bare cargo

carla-rust exposes a **different API per CARLA version**, selected by the `CARLA_VERSION`
environment variable that the justfile sets to 0.9.16. Under 0.9.x a wheel's position is
`WheelPhysicsControl::position`; under 0.10 the same field is `offset`. A bare
`cargo build` or `cargo check` does not set the variable, so cargo silently selects the
0.10 API and reports correct 0.9.16 code as "no field `position`".

That error is convincing and wrong, and acting on it has already changed a correct field
access into a broken one and back again. If a build error names a missing field on a
carla-rust type, check `CARLA_VERSION` before believing it:

```bash
just build          # sets CARLA_VERSION=0.9.16
just check          # same, so clippy sees the API the build uses
CARLA_VERSION=0.9.16 cargo check    # if you must call cargo directly
```

### Keep the system setuptools
`--symlink-install` makes colcon run `setup.py develop --editable`, which setuptools removed
in v80. A pip-installed setuptools in `~/.local` shadows the apt one (`python3-setuptools`,
59.6.0) and every Python package in the workspace then fails with:

```
error: option --editable not recognized
```

colcon aborts the remaining packages after the first failure, so a Rust-only change looks
broken when it never compiled. `just build` checks for this up front and refuses to start;
`just install-deps` warns if a `pip install` pulled setuptools in. Fix with:

```bash
pip uninstall -y setuptools   # falls back to the apt package
```

### Push submodule commits before bumping the superproject pin
Never update a submodule pin in this repo to a commit that only exists locally.
Push the commit to the submodule's remote first, then stage and commit the pin
here. A pin referencing an unpushed commit breaks `git submodule update` for
everyone else cloning the repo.

```bash
cd src/<submodule> && git push origin <branch>   # first
cd - && git add src/<submodule> && git commit    # then
```

Verify with `git submodule status` and confirm the SHA is reachable on the
remote (e.g. `git ls-remote origin | grep <sha>`) before pushing the superproject.

### Account for every process a run starts, and reap what it leaves

A run starts a lot: one CARLA server, an ego stack of ~90 `component_node` processes, a
scenario stack, and the bridge. Killing the `play_launch` that owns a stack does not
always take its children with it, so repeated start/stop cycles silently accumulate
orphans that hold ports and memory.

Before a run, know what is already up:

```bash
# stacks, by web port -- 8081 is the scenario stack, 8082 the ego stack
for p in $(pgrep -x play_launch); do tr '\0' ' ' < /proc/$p/cmdline | grep -oE 'web-addr [0-9.:]+'; done
ps -eo pid,ppid,etimes,args --no-headers | grep -E "CarlaUE4-Linux|carla_scenario_bridge" | grep -v grep
ss -lpn | grep 5555        # exactly one bridge should hold the SSv2 port
```

**The bridge is not part of any launch file.** `carla_scenario.launch.xml` includes only
SSv2's `scenario_test_runner`; the bridge is a separate long-lived process started by
`just run`. Nothing restarts it between scenarios, so it routinely outlives the session
that started it -- one was found still serving scenarios 44 hours later, running code from
two rebuilds earlier. Killing it does not get a fresh one: it leaves no bridge at all, and
every scenario then spawns no ego.

So treat the bridge as a tracked singleton. Check its age against the current build before
trusting a measurement, and after a rebuild restart it deliberately:

```bash
ps -eo pid,etimes,args --no-headers | grep carla_scenario_bridge | grep -v grep
just run &      # the only thing that starts one
```

After stopping a stack, reap what outlived it. Orphans show up as `ppid == 1`:

```bash
ps -eo pid,ppid,etimes,comm --no-headers | awk '$2==1 && ($4=="component_node" ||
    $4=="add_two_ints_se" || $4=="zenohd" || $4=="carla_scenario_")'
```

Kill those by pid. Leave the CARLA server alone unless it is actually wedged -- it costs
minutes to restart and one instance serves every run.

### Never match your own command line with pkill

`pkill -f ego_av` also matches the script that runs it, so a cleanup step kills its own
harness mid-run. This has cost several runs. Match on the executable name and read the
full command line from `/proc`, which cannot match the shell doing the matching:

```bash
for p in $(pgrep -x play_launch); do
    tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q "8082" && kill "$p"
done
```

For a process tree you started yourself, give it its own process group with `setsid` and
signal the group, which needs no pattern at all:

```bash
setsid just scenario "$SCENARIO" > run.log 2>&1 &
SPID=$!
kill -TERM -$SPID        # the whole tree, and only that tree
```
