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
just run      # Simulation side: bridge + agent relay (simulation.launch.xml)
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
# stacks, by web port -- 8081 is the scenario stack, 8082 the ego stack
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
must run first.

```bash
ps -eo pid,etimes,args --no-headers | grep -E "[c]arla_scenario_bridge|[s]cenario_agent_relay"
ss -lpn | grep -E ":5555 |:5560 "   # bridge, relay
```

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

play_launch 0.12 defaults to `--enforce-rules warn`, which enables LD_PRELOAD interception
of every DDS take in every node and logs each event at DEBUG to `play_launch.log` plus
`interception/events.jsonl`. One 50-minute ego stack wrote **169 GB** of it on 2026-09-27
and took the volume to 0 bytes free, which then failed a colcon install and a git checkout
mid-way. There are no contract files in this workspace, so nothing is enforced either way.
Every `play_launch launch` in the justfile now passes `--enforce-rules off`; keep it on any
new invocation, and if `df` moves faster than a build explains, look at `play_log/*/` first:

```bash
du -sh play_log/ego/*/play_launch.log play_log/ego/*/interception 2>/dev/null | sort -rh | head
```

### The log disk can stall a node

`/home` (`/dev/sda1`) is a spinning ext4 disk at 99 % that other users saturate at will.
A node whose stdout goes to `play_log/` on it blocks in the ext4 journal when they do: acb's
main loop stalled 0.5–3 s several times per scenario and 25 s once, skipping CARLA frames
and starving Autoware into emergency stops (roadmap 014, acb `0d091de` moved acb's logging
to a writer thread). Any other node that logs at tens of lines per second from a real-time
loop is exposed the same way, and play_launch's own `system_stats.csv` shows the holes.
Check before blaming code:

```bash
cat /proc/pressure/io            # "some avg10" over ~30 means the disk is the bottleneck
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

**Never run CARLA at `-quality-level=Low`.** At Low, CARLA 0.9.16 segfaults on `load_world`
to another town and hangs on `reload_world()`: UE 4.26's landscape render task reads a
texture freed by the level swap (`FGetSectionLODBiasesTask` → `UTexture2D::GetNumResidentMips`
in UE's own crash reports under `~/.config/Epic/CarlaUE4/Saved/Crashes/`, which is where to
look first when CARLA dies -- its stdout has no stack). At the default (Epic) quality the
same server loads Town02 in 44 s and back in 4 s (carla-simulator/carla#4940; roadmap 015
step 6). acb's `third_party/carla/run.sh` now defaults to Epic. Kill CARLA by pid, and if it
ignores SIGTERM use SIGKILL before starting another: a new instance on a held port crashes.