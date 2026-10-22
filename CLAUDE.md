# carla-scenario-bridge

## Project Overview

ZMQ+Protobuf adapter that makes CARLA a backend for tier4/scenario_simulator_v2 (SSv2). Implements SSv2's `simulation_interface` protocol, translating protobuf requests into CARLA API calls via carla-rust.

**Design doc**: `docs/design/architecture.md`
**Roadmap**: `docs/roadmap/`
**Parent project**: [autoware_carla_bridge](https://github.com/NEWSLabNTU/ros_zenoh_bridge) (sensor/vehicle interface layer)
**SSv2 protocol reference**: See `proto/` directory

## Build & Run

```bash
just setup    # One-time: the user install (rustup, colcon plugin, submodules, rosdep) + dev tools
just build    # The user's colcon build, plus --symlink-install and the dev-release profile
just clean    # Clean artifacts
just run      # Simulation side: bridge + agent relay (simulation.launch.xml)
just check    # Format + clippy
just test     # Run tests
```

## Key Design Decisions

- **AWSIM pattern**: SSv2 puppeteers NPCs via `set_transform()`, CARLA owns ego physics
- **Sensors**: Reject SSv2 `AttachSensor` requests (`Success=false`); CARLA sensors publish via autoware_carla_bridge
- **Traffic lights**: Freeze CARLA's built-in cycling, set states per SSv2 commands
- **World config**: ROS services on the bridge (`/carla/set_weather`, `/carla/get_weather`,
  `csb_interfaces`), applied on the ZMQ thread between SSv2 requests; the requested weather
  (param `weather`, then the last set) is re-applied after every map load and reconnect
- **Coordinate conversion**: SSv2 uses ROS right-handed frame; CARLA uses left-handed (Y-flip)
- **Vehicle interface**: autoware_carla_bridge handles `/vehicle/status/*` and `/control/command/*`; SSv2's `AutowareUniverse` (concealer) should be disabled

## Repository Structure

```
.
├── src/carla_scenario_bridge/  # Rust crate (ament_cargo package)
│   ├── src/
│   │   ├── main.rs             # Entry point: config, backend choice, shutdown
│   │   ├── zmq_server.rs       # ZMQ REP socket, protobuf dispatch
│   │   ├── coordinator.rs      # SSv2 requests → SimBackend calls (no simulator)
│   │   ├── backend/            # The simulator seam (035)
│   │   │   ├── mod.rs          # SimBackend trait and its types
│   │   │   ├── fake.rs         # In-memory backend for coordinator tests
│   │   │   ├── carla/          # CarlaBackend: the only code that uses carla-rust
│   │   │   │   ├── backend.rs  # SSv2 handlers → CARLA API calls
│   │   │   │   ├── collision_monitor.rs, traffic_light_mapper.rs,
│   │   │   │   └── resolved_signals.rs, version.rs, weather.rs
│   │   │   └── isaac/          # IsaacBackend: csb_world.proto to the world server (037)
│   │   ├── gap_trace.rs        # Both worlds per frame, for the gap oracle (024)
│   │   ├── entity_manager.rs   # SSv2 name ↔ CARLA actor ID mapping
│   │   ├── coordinate_conversion.rs  # ROS ↔ CARLA frame conversion
│   │   ├── weather.rs          # Weather presets, kept across map loads (018)
│   │   ├── world_services.rs   # rclrs node: /carla/set_weather, /carla/get_weather
│   │   └── proto.rs            # Generated protobuf type re-exports
│   ├── build.rs                # prost-build proto compilation
│   ├── config/bridge_config.yaml
│   ├── Cargo.toml
│   └── package.xml
├── src/csb_interfaces/     # rosidl: Weather.msg, SetWeather.srv, GetWeather.srv
├── src/csb_agent/          # Agent SDK + agent protocol v1/v2 (030): csb_agent.Agent, Client, main
├── src/scenario_agent_relay/  # the relay: concealer ADAPI <-> agents (imports csb_agent.protocol)
├── src/csb_isaac/          # Isaac Sim world server (Python, Isaac's venv; `just isaac-world`):
│                           #   lockstep world, NPCs, contacts, the CARLA vehicle catalogue,
│                           #   the surface layer (047: <map>/surfaces.yaml, per-wheel friction)
├── src/autoware_isaac_bridge/  # aib submodule (045): the Isaac ego the world server imports
│                           #   (PhysX vehicle, sensors, vehicle interface; vehicle spec ->
│                           #   Autoware vehicle/sensor kit + rig), aib_launch for `just ego-isaac`
├── proto/                  # SSv2 protobuf definitions, and csb_world.proto (Isaac, 037)
├── src/csb_examples/       # Starter scenarios (basic/, multi_av/, awf/), installed
├── scenarios/              # Benchmark scenarios (bench/), goal-pose files
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

### Project settings are arguments, host settings are environment

Recipes take project settings -- map, rviz, vehicle and launch choices, scenario output,
`keep_stacks`, `record` -- as `name:=value` arguments, as a launch file does (`just ego-av
map_path:=<dir> rviz:=true`, `just scenario <xosc> output_directory:=<dir>`); `ego-av`,
`ego-isaac` and `bg-av` hand any argument they do not know to the launch file. Host
settings come from the environment, made permanent in `.env` (just's `dotenv-load`) or
`.envrc`: ports (`CARLA_PORT`, `SSV2_PORT`, `AGENT_PORT`, `ISAAC_PORT`; the play_launch web
UIs from `CSB_WEB_PORT`, default 8080: +0 demo, +1 scenario, +2 ego, +3 background AV, +4
the simulation side, +5 the Isaac world server), displays
(`DISPLAY`, `ISAAC_DISPLAY`, `RECORD_DISPLAY`), install and data paths (`AUTOWARE_SETUP`,
`ISAAC_ROOT`, `CARLA_SRC`, `CARLA_USD`, `CSB_VEHICLE_USD`, `CARLA_DIR`, `DATA_DIR`,
`CARLA_MAPS`, `AUTOWARE_TENSORRT_LIB`), the ROS domains
(`EGO_ROS_DOMAIN_ID`, `SCENARIO_ROS_DOMAIN_ID`, `BG_ROS_DOMAIN_ID` for the first background
AV; the Isaac ego's `isaac_vehicle` node follows
`EGO_ROS_DOMAIN_ID` too, over aib's `ego.yaml`) and the ego's RMW (`EGO_RMW_IMPLEMENTATION`),
the log root and archive (`CSB_LOG_ROOT`, `CSB_LOG_ARCHIVE`), and the Isaac bring-up waits
(`ISAAC_READY_TIMEOUT`, `EGO_READY_TIMEOUT`). The variables that used to carry project
settings (`MAP_NAME`, `LAUNCH_RVIZ`, `EGO_LAUNCH_ARGS`, `OUTPUT_DIR`, `KEEP_STACKS`, ...;
the list is `CSB_RETIRED_ENV` in `scripts/just_lib.bash`) make a recipe refuse to start,
naming the argument to pass instead. Do not add new ones.

Two checkouts share a host by moving one checkout's whole set in its `.env`: `CSB_WEB_PORT`,
`SSV2_PORT` (the bridge's sensor-release sockets follow it at +1 and +2, on both csb's and
acb's side), `AGENT_PORT`, `EGO_ROS_DOMAIN_ID`, `SCENARIO_ROS_DOMAIN_ID`, `BG_ROS_DOMAIN_ID`. `ISAAC_PORT` and
`CARLA_PORT` keep the simulators' own defaults (5590, 2000) unless those are taken too; a
second CARLA also needs its Traffic Manager port moved (`traffic_manager.port`, default 8000).

### Use just build
Always use `just build` instead of `colcon build` directly (ensures `--symlink-install`).

**And rebuild both workspaces after an acb change, in this order: acb's, then this one.**
`just ego-av` sources three overlays -- Autoware, `src/autoware_carla_bridge/install/`, then
this repo's `install/` -- and the last one wins. Because `just build` here runs colcon with
`--base-paths src`, it compiles the acb packages too, so the `acb_bridge` the stack actually
executes is `install/acb_bridge/lib/acb_bridge/acb_bridge` **in this repo** (verified from
`/proc/<pid>/cmdline`, 2026-09-25). Rebuilding only the acb workspace changes nothing the
stack runs; rebuilding only this repo works until someone rebuilds acb and wonders why it
had no effect. A pull that moves the acb submodule leaves the old binaries in place either
way, and the stack keeps running them, silently.

Stop the ego stack before `just build` here: with it running, colcon fails at the install
step with `Failed to copy binary: acb_bridge: Text file busy` after compiling everything.

This is not cosmetic. An `acb_bridge` three days stale measured longitudinal tracking at
**0.618 m/s^2** against a 0.35 limit; rebuilding acb and changing nothing else took the
same scenario, on the same machine, to **0.051**. Twelve times better, and every run in
between had been failing for a reason that looked like a control regression and was a build
artefact. Worse, the numbers still *look* plausible -- the ego drives, the scenario runs,
only a threshold notices -- so nothing announces it.

```bash
git submodule update --init --recursive
(cd src/autoware_carla_bridge && just build)   # acb's own workspace
just build                                      # this repo: the overlay the stack runs
```

Check the binary against the source when a measurement surprises you -- both copies, and
the one the live process actually loaded:

```bash
ls -l install/acb_bridge/lib/acb_bridge/acb_bridge \
      src/autoware_carla_bridge/install/acb_bridge/lib/acb_bridge/acb_bridge
(cd src/autoware_carla_bridge && git log -1 --format='%h %ad' --date=short)
tr '\0' '\n' < /proc/$(pgrep -f 'acb_bridge/acb_bridge' | head -1)/cmdline | head -1
```

### Autoware fixes ride in an overlay

Autoware 1.5.0 is a binary install we do not rebuild. Fixes it lacks are carried per package
(roadmap 042): `just build-autoware-overlay` fetches the NEWSLabNTU `autoware_universe` fork
at the commit pinned in `scripts/autoware_overlay.bash` (sparse: only the listed package
dirs), builds them against `/opt/autoware/1.5.0` into `third_party/autoware_overlay/`
(git-ignored), and `just ego-av` / `bg-av` / the scenario precheck source it right after
Autoware. On by default when built; `autoware_overlay:=false` runs stock. Today it holds one
package, `blind_spot`, with the right-turn ego passage fix (autoware_universe#13509, branch
`csb/blind-spot-right-turn-0.48.0` = tag 0.48.0 + the fix). To carry another fix: commit on
a `csb/*` branch of the fork from the release tag, push, move the pin, add the dir, rebuild.
Check what a stack loaded: `grep blind_spot /proc/<behavior_velocity_planner pid>/maps`.

### Autoware's TensorRT is 10.8.0, not the host's

Autoware 1.5.0's `libautoware_tensorrt_common` was compiled against TensorRT **10.8.0** and
accepts a cached engine only when the plan was built by exactly that version. The loader takes
`libnvinfer.so.10` from whatever comes first: on this host `/usr/local/cuda` (CUDA 12.8, so
TensorRT **10.9.0**) is ahead of the system's 10.15 in `ld.so.conf`. So 10.9 built every
engine, the next start rejected it, and CenterPoint rebuilt both of its engines at every
ego-stack start: 3-11 minutes, depending on load. The `lidar_centerpoint` err log said so
each time:

```
[W] [TRT] Plan was created with a different version of TensorRT! Current version: 10.8.0
[W] [TRT] Validation failed for the existing engine file. Rebuilding
```

`just ego-av`, `ego-isaac`, `bg-av` and `e2e` now put a 10.8.0 runtime first on the stack's
`LD_LIBRARY_PATH`, after every setup is sourced (`scripts/autoware_tensorrt.bash`). The
directory is a host setting, `AUTOWARE_TENSORRT_LIB` (default `third_party/tensorrt/lib`;
empty: the host's own). `just setup-tensorrt` fills it from NVIDIA's `tensorrt-cu12-libs`
10.8.0.43 wheel: libnvinfer, its builder resource, the plugin library and the ONNX parser,
2.6 GB. Nothing goes into the system or a Python environment, and `ld.so.conf` is left
alone. On this host it is `/mnt/wd/aeon/tensorrt-10.8.0/lib`, set in `.env`. A missing
directory warns and the stack starts on the host's TensorRT, rebuilding as before.

The first start on 10.8 rebuilds once, because the old engine came from 10.9. After that
the engine loads. Measured 2026-10-21 with `lidar_detection_model:=centerpoint`:
CenterPoint was ready at **189 s** on the first start and **19 s** on the second, the whole
stack in 20 s, and the engine mtimes did not change. Demo incident 2 passed with the
pedestrian 5.4 m ahead, as before. On the default stack (`clustering`) only
`shape_estimation` and `detection_by_tracker` load libnvinfer. Neither builds an engine,
and both start as before. To check what a stack loaded:
`grep nvinfer /proc/<lidar_centerpoint container pid>/maps`.
When Autoware moves to a release built against another TensorRT, bump the version, wheel
and checksum in `scripts/autoware_tensorrt.bash` together.

### The CARLA version is a Cargo feature

carla-rust exposes a **different API per CARLA version**. Under 0.9.x a wheel's position is
`WheelPhysicsControl::position`; under 0.10 the same field is `offset`. The version is now
pinned in both workspaces' `Cargo.toml` by `features = ["carla-0916"]` on the `carla`
dependency (carla 0.15.1 from crates.io in both, no git patch; roadmap 018), so a bare
`cargo build`/`cargo check` and a plain `colcon build` compile the 0.9.16 API with no
environment at all. Bump both workspaces to the same release together.

- `CARLA_VERSION`, when set, still **overrides** the feature (the justfile sets 0.9.16).
  A stray `CARLA_VERSION=0.10.0` in your shell therefore still builds the wrong API and
  reports correct code as "no field `position`" -- check `echo $CARLA_VERSION` before
  believing an error about a missing field on a carla-rust type.
- No feature and no `CARLA_VERSION` is a **build error** (carla-rust no longer falls back to
  0.10 silently); two version features in one dependency graph is a build error naming both.
- At run time the bridges compare the version they were built for (`carla::CARLA_VERSION`)
  with the server's on every connect and refuse a mismatch with both versions in the message
  (csb retries as for an unreachable server; acb exits).

### Keep the system setuptools
`--symlink-install` makes colcon run `setup.py develop --editable`, which setuptools removed
in v80. A pip-installed setuptools in `~/.local` shadows the apt one (`python3-setuptools`,
59.6.0) and every Python package in the workspace then fails with:

```
error: option --editable not recognized
```

colcon aborts the remaining packages after the first failure, so a Rust-only change looks
broken when it never compiled. `just build` checks for this up front and refuses to start.
A plain `colcon build` (no `--symlink-install`, the user path) is not affected. Fix with:

```bash
pip uninstall -y setuptools   # falls back to the apt package
```

### Linear history

Every repo and branch, this one and each submodule: rebase onto the remote, never push
a merge commit. Pull with `git pull --rebase`; land a branch on `main` by fast-forward
(`git rebase origin/main`, then `git push origin <branch>:main`).

### Submodules move in lockstep with their remote

What a pin may point at:

- **Third-party code** (not ours, e.g. `color_names`): a commit on a stable release
  (a tag), or one specific commit chosen on purpose and named in the commit message
  that sets it. Never an arbitrary upstream head.
- **Our own repos and forks** (NEWSLabNTU: `autoware_carla_bridge`,
  `autoware_isaac_bridge`, the `scenario_simulator_v2` fork): a commit on a branch of
  our repo that the pin tracks. Put that branch in `.gitmodules` (`branch = ...`, as
  `scenario_simulator_v2` has `managed-ego-unforked`); without one, the branch is
  `main`. A third-party project we need to change gets a fork under NEWSLabNTU, and
  the pin tracks a branch there. Do not pin a commit that lives only in someone
  else's repo.

Order: **push the submodule commit to its remote first, then bump the pin here.**
Never update a pin to a commit that only exists locally. A pin referencing an
unpushed commit breaks `git submodule update` for everyone else cloning the repo.

```bash
cd src/<submodule> && git push origin <branch>   # first
cd - && git add src/<submodule> && git commit    # then
```

Verify with `git submodule status` and confirm the SHA is on the tracked branch of the
remote before pushing the superproject (`git ls-remote` lists only branch tips, so an
older commit does not show there):

```bash
cd src/<submodule> && git fetch origin && git branch -r --contains <sha>   # must list origin/<branch>
```

### Account for every process a run starts, and reap what it leaves

A run starts a lot: one CARLA server, an ego stack of ~90 `component_node` processes, a
scenario stack, and the bridge. Killing the `play_launch` that owns a stack does not
always take its children with it, so repeated start/stop cycles silently accumulate
orphans that hold ports and memory.

**The scenario stack now ends itself.** SSv2 declares `scenario_test_runner` with
`on_exit=ShutdownOnce()` -- "this process is required; when it exits, take the launch with
it" -- and play_launch used to discard that handler, so every finished run left its
interpreter, preprocessor and visualization alive. That is what had scenario stacks being
found hours later (one interpreter was still spinning 40 hours after its scenario passed),
and it is why the interpreter's own watchdog kept reporting `main ... unresponsive`: an
idle deactivated node blocks in `spin_once()` with nothing left to do. Fixed in play_launch
`779d68e`; a scenario stack started with a play_launch at or after that commit disappears
on its own when the run ends, and its log says so:

```
[node:/simulation/scenario_test_runner] exited (code 0) and was declared on_exit=Shutdown:
    shutting down the launch
```

**The ego stack does not** -- nothing in it is declared required -- so `just ego-av` still
has to be stopped deliberately, and everything below still applies to it.

Before a run, know what is already up:

```bash
# stacks, by web port -- with CSB_WEB_PORT=8080: 8081 is the scenario stack, 8082 the ego stack
for p in $(pgrep -x play_launch); do tr '\0' ' ' < /proc/$p/cmdline | grep -oE 'web-addr [0-9.:]+'; done
ps -eo pid,ppid,etimes,args --no-headers | grep -E "CarlaUE4-Linux|carla_scenario_bridge" | grep -v grep
ss -lpn | grep 5555        # exactly one bridge should hold the SSv2 port
```

**The simulation side is its own long-lived launch.** `just run` starts
`csb_launch simulation.launch.xml` under play_launch (web 8084, scenario domain 9): the bridge
(`respawn="true"`, so a crash restarts it) and the agent relay (TCP 5560) the ego's vehicle
agent registers with. Scenario launches never include it, so it outlives sessions -- one
bridge was once found serving scenarios 44 hours later, running code from two rebuilds
earlier. Treat it as a tracked singleton: check its age against the build before trusting a
measurement, and after `just build` restart it deliberately (stop the play_launch on 8084 by
pid, then `setsid just run > log 2>&1 &`). It runs the *installed* binary, so `just build`
must run first. Its config is the bridge's ROS parameter `config_file` (`just run
config_file:=<file>`, default the installed `bridge_config.yaml`; `CSB_CONFIG_DIR` is
refused): `ros2 param get /carla_scenario_bridge config_file` in domain 9 names the file a
running bridge loaded.

```bash
ps -eo pid,etimes,args --no-headers | grep -E "[c]arla_scenario_bridge|[s]cenario_agent_relay"
ss -lpn | grep -E ":5555 |:5560 "   # bridge, relay
```

**On Isaac (roadmap 043) there is a fourth stack, the world server**: `just isaac-world`
(csb_launch `isaac_world.launch.xml`, web 8085, `play_log/isaac/`, respawned on death).
`just scenario|acceptance|e2e <xosc> backend:=isaac` starts the world server, the simulation
side on backend isaac and `just ego-isaac` when they are absent (`scripts/isaac_up.sh`) and
leaves them up (`e2e` stops what it started); it refuses a CARLA bridge on 8084 or a
`just ego-av` on 8082 rather than replace them. `just isaac-health` is the Hello probe. The
world server holds ~17.5 GB of RAM with Town01's SimReady world: on a host whose memory other
work has taken, the kernel's OOM killer picks it first (seen once, as the ego stack started),
and the respawned one preloads again for ~7 minutes.

**The ego is reached through the relay, not ROS (roadmap 017).** SSv2 runs in the scenario
domain (9), the ego stack in its own (1); a scenario whose ego never engages usually means
the agent never registered: check the relay's log (`play_log/bridge/latest/node/
scenario_agent_relay/err`) and the agent's (`play_log/ego/latest/node/acb_agent/err`).

After stopping a stack, reap what outlived it. This is now mostly the ego stack's
business -- a scenario stack that ended on its own leaves nothing -- but a stack killed by
hand mid-run still can. Orphans show up as `ppid == 1`:

```bash
ps -eo pid,ppid,etimes,comm --no-headers | awk '$2==1 && ($4=="component_node" ||
    $4=="add_two_ints_se" || $4=="zenohd" || $4=="carla_scenario_" ||
    $4=="carla_manual_co" || $4=="auto_drive" || $4=="acb_bridge" || $4=="agent" || $4=="relay")'
```

A name-based list only finds what is on it. A `carla_manual_control` left over from the
cockpit work survived the kill of its `ros2 run` wrapper and spent **50 hours burning nine
cores**, and this check walked straight past it because its `comm` was not listed. Load
average was 68 on 32 cores, the interpreter logged "your machine is not powerful enough"
continuously, and an ego stack took nine minutes to come up instead of four. So read the
top of `ps` as well, not just the list:

```bash
ps -eo pcpu,pid,etimes,comm --no-headers --sort=-pcpu | head
uptime          # load should be well under the core count between runs
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

The justfile itself has one of these: `_clear-stale-scenario` runs `pgrep -f
scenario_test_runner` and SIGKILLs every match before a scenario starts. A shell that is
waiting on `/tmp/scenario_test_runner/result.junit.xml` has that string on its command line
and dies with the stale processes. Keep the string out of any long-lived command line (read
the path from a variable set in a different process, or wait on the play_launch log instead).

### A CARLA client must not outlive its server

A LibCarla client whose server died keeps a dead RPC connection (it never reconnects) and a
streaming client that does reconnect, to the next server on the same port. Its light-update
callback then blocks inside the stream for the client's whole timeout, and a synchronous
server waits for every subscriber before the next frame: the whole server hangs (the "port
2000 hang", `docs/design/failure-and-frame-budget.md`). csb exits for a fresh process when
its idle probe finds the connection dead; acb drops and rebuilds such clients. Any other
long-lived client on the testbed's port needs the same, or must be restarted with CARLA.

### Never destroy a vehicle while its sensors are attached

Destroy an actor's children first, in every client and language: in Rust
`ActorBase::destroy_with_children()`; in Python sweep `sensor.*` actors whose `parent.id` is
the vehicle before `vehicle.destroy()` (acb's `demo_scenario.py::destroy_attached_sensors`).
CARLA leaves a destroyed vehicle's sensors alive and ticking, and an orphaned IMU segfaults
the server (`AInertialMeasurementUnit::ComputeGyroscope`, its owner check compiled out of the
Shipping build): three of eight runs once died that way, 24-40 s in.

### Talking to the stack from outside a launch

Every ROS tool that must see the ego stack -- `scripts/ego_stack_health.py`, `ros2 topic`,
probes -- needs the same DDS transport the stack uses, or discovery silently finds nothing
and the health check reports "nothing publishes /api/operation_mode/state" on a healthy
stack:

```bash
source /opt/autoware/1.5.0/setup.bash
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds-localhost.xml" ROS_DOMAIN_ID=1
scripts/ego_stack_health.py
```

`pgrep -x` matches `comm`, which the kernel truncates to 15 characters:
`pgrep -x CarlaUE4-Linux-Shipping` finds nothing while CARLA runs. Use `pgrep -f` with a
pattern the calling shell does not contain, or `ps -eo comm` and match the prefix.

### play_launch interception fills the disk

play_launch before 0.15 defaulted to `--enforce-rules warn`, which enabled LD_PRELOAD
interception of every DDS take in every node and logged each event at DEBUG to
`play_launch.log` plus `interception/events.jsonl`. One 50-minute ego stack wrote **169 GB**
of it on 2026-09-27 and took the volume to 0 bytes free, which then failed a colcon install
and a git checkout mid-way. 0.15.0 (2026-10-11) intercepts only when a contract resolves or
`--enforce-rules` is given, and logs events at trace; this workspace has no contracts, so
nothing intercepts. Require play_launch >= 0.18.3 (`play_launch --version`): 0.15.1 is the first to honour SSv2's on_exit=ShutdownOnce under the default Rust parser, and 0.18.3 (issue #0075) the first whose respawn keeps the dead process's log as `node/carla_scenario_bridge/out.1`/`err.1` -- older versions truncate it, losing the log that explains a hung tick; and if `df` moves
faster than a build explains, look at `play_log/*/` first:

```bash
du -sh play_log/ego/*/play_launch.log play_log/ego/*/interception 2>/dev/null | sort -rh | head
```

### The log disk can stall a node

A node whose stdout goes to a busy spinning disk blocks in the ext4 journal when other work
saturates it: acb's main loop stalled 0.5–3 s several times per scenario and 25 s once,
skipping CARLA frames and starving Autoware into emergency stops (roadmap 014, when `/home`
was a spinning disk; acb `0d091de` moved acb's logging to a writer thread). Any other node
that logs at tens of lines per second from a real-time loop is exposed the same way, and
play_launch's own `system_stats.csv` shows the holes.

On the host roadmaps 036-045 were built on (the scripts' `/mnt/disk1` defaults), `/home` is
`/dev/nvme1n1p1` (NVMe, shared, 94 % full) and `/mnt/disk1` is `/dev/sda1`,
a 16 TB Seagate **spinning** disk (`lsblk -d -o NAME,ROTA,MODEL`: ROTA 1) that holds Isaac
Sim, its USD worlds, the NTU tiles and other users' data, and is often 80–95 % busy with
reads. Measured 2026-10-19 (`append + fsync every 20th line` at 200 lines/s for 60 s, both at
once, sda at 93 % util from other readers):

| | fsync p50 | p99 | max | writes the loop managed |
|---|---|---|---|---|
| `/home` (NVMe) | 0.9 ms | 3.4 ms | 6.8 ms | 12000 of 12000 |
| `/mnt/disk1` (HDD) | 142 ms | 339 ms | 1500 ms | 7500 of 12000 |

So live logs stay on NVMe: the log root, `CSB_LOG_ROOT` (a host setting; default the repo's
`play_log/`), is where every recipe's play_launch writes, `<root>/<kind>/<time>/` (`ego`,
`bg-<entity>`, `bridge`, `isaac`, `scenario`, `demo`, `e2e`, `acceptance`, `two-av`). Point
it at another *fast* disk if `/home` fills; when it is not `play_log/`, the recipes link
`play_log/<kind>` to it, so `play_log/<kind>/latest` keeps working. Never point it at
`/mnt/disk1`.

newslab-server139 is laid out differently: `/` is the only fast disk (`/dev/nvme0n1p2`, NVMe,
1.8 TB, ~1 TB free); `/home` (`/dev/sda1`, 87 % full) and `/mnt/wd` (`/dev/sdb1`, 4.2 TB
free) are both 5.5 TB WD Purple **spinning** disks, idle (1-2 % util) when measured on
2026-10-21. Large things live under `/mnt/wd/aeon/`, set in its `.env`: `ISAAC_ROOT`
(`isaac-sim`), `CARLA_SRC` (`carla-src`, the UE 4.26 + CARLA editor build), `CARLA_USD` and
`CSB_VEHICLE_USD` (`carla-usd`), `AUTOWARE_TENSORRT_LIB`; CARLA 0.9.16 itself is
`/mnt/wd/aeon/carla-simulator/CARLA_0.9.16`. The 045 reproduction ran with
`CSB_LOG_ROOT=/mnt/wd/aeon/csb-045/play_log` (per command, not in `.env`).

The space problem is solved by moving old runs, not by logging elsewhere: `just prune-logs
[days:=7] [dest:=<dir>] [dry_run:=true]` moves runs untouched for that long to
`$CSB_LOG_ARCHIVE` (default `/mnt/disk1/csb-play_log-archive`, where a spinning disk is
fine), kind by kind. It never deletes or overwrites, and keeps each kind's `latest` and any
run a live process holds open (a running play_launch). `src/autoware_carla_bridge/play_log`
is acb's own and not touched. Check before blaming code:

```bash
iostat -dx sda nvme1n1 5 2       # %util per device; /proc/pressure/io is host-wide, not per disk
df -h /home
for t in /proc/$(pgrep -x acb_bridge)/task/*; do grep -H State "$t/status"; done | grep -c "D (disk"
```

### CARLA on a host that is not the checkpoint's

`third_party/carla/run.sh` assumes `~/Downloads/CARLA_0.9.16` and a `DISPLAY` you own; the
`carla-run-<port>` unit fails itself if RPC is not up in 180 s. A first launch on a new host
took 120 s to serve RPC on this hardware, and with restart overhead the unit never came up.
Pass `CARLA_DIR=... DISPLAY=:N just carla-start`, and if the unit keeps restarting, launch
`./CarlaUE4.sh -carla-rpc-port=2000 -nosound -RenderOffScreen` directly
under `setsid` and wait. Any X display works for offscreen Vulkan rendering, GLX or not; a
private one is `setsid /opt/TurboVNC/bin/Xvnc :4 -SecurityTypes None`.
No display at all also works: `env -u DISPLAY setsid ./CarlaUE4.sh ... -RenderOffScreen`
(RPC up in 182 s at Epic quality, 2026-10-05; the 180 s unit limit is too tight for it).

**Pin Vulkan to the NVIDIA card** when starting CARLA by hand:
`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`. Left to choose on this host,
a restarted CARLA came up on the AMD iGPU or the software renderer (18 MiB on the NVIDIA
card), took 4–8 min to serve RPC and froze in `apply_settings(synchronous)`; pinned, RPC was
up in 50 s (roadmap 016, gap 13). acb's `third_party/carla/run.sh` sets it when unset.

**CARLA at `-quality-level=Low` must never change towns.** At Low, CARLA 0.9.16 segfaults
on `load_world` to another town and on `reload_world()` (same town too): UE 4.26's landscape
render task reads a texture freed by the level swap (`FGetSectionLODBiasesTask` →
`UTexture2D::GetNumResidentMips` in UE's own crash reports under
`~/.config/Epic/CarlaUE4/Saved/Crashes/`, which is where to look first when CARLA dies -- its
stdout has no stack; carla-simulator/carla#4940). Re-measured 2026-10-19: 5 of 5 Low
attempts died that way (3 x load_world Town02, 1 x reload_world, 1 x town02_episode_change),
the client seeing only a 180 s RPC time-out; at Epic 5/5 cycles of Town02 → Town01 →
reload_world and the episode change passed. A run that stays on the town CARLA booted on is
fine at Low (town01_ego_drive passed 2/2; csb skips `load_world` when the town matches), but
Low saves little: CARLA held 2424 MB RSS / 1730 MiB VRAM at Low against 2463 MB / 2005 MiB at
Epic on Town01, idle or running the scenario (`docs/design/carla-server-tuning.md`). So the
default is **Epic**: `just carla-start quality:=low` only for single-town work, never for
`town02_episode_change`, a suite that mixes towns, or a scenario on another town than
`map:=` (default Town01). Quality, map and render mode are arguments of `just carla-start`
(`map:=`, `quality:=epic|low`, `render:=offscreen|window`, `extra_args:=`); the old
`CARLA_QUALITY`/`CARLA_MAP`/`CARLA_RENDER`/`CARLA_EXTRA_ARGS` are refused. (Until acb's
`carla_start.sh` took arguments it still defaulted to Low after `run.sh` moved to Epic, which
is how two episode-change runs crashed CARLA on 2026-10-17.) Kill CARLA by pid, and if it
ignores SIGTERM use SIGKILL before starting another: a new instance on a held port crashes.