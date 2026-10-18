# Developer Guide

For working on carla-scenario-bridge itself: building, testing, running against a live
stack, and finding your way around the code and the design documents. Users start at the
[README](../README.md) and the [tutorials](tutorials/).

## Set up

On top of the user prerequisites (ROS 2 Humble, Autoware, CARLA 0.9.16, maps):

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/carla-scenario-bridge.git
cd carla-scenario-bridge
just setup      # rustup, colcon-cargo-ros2, submodules, rosdep, and the dev tools
just build      # the user's colcon build plus --symlink-install and a dev-release profile
```

[`just`](https://github.com/casey/just) wraps the developer workflow; `just` alone lists the
recipes. The ones you will use most:

| Recipe | What it does |
|---|---|
| `just build` | colcon build of everything under `src/` (both workspaces' packages, see below) |
| `just check` | `cargo +nightly fmt --check` and clippy with `-D warnings` |
| `just test` | Rust tests (cargo-nextest) and `run_suite`'s Python tests |
| `just ci` | build + check + test |
| `just format` | format the Rust code |
| `just carla-start` / `carla-status` / `carla-stop` / `carla-health` | CARLA as a background service, and whether it is fit to run a scenario |
| `just run` | the simulation side (bridge + agent relay), play_launch, web UI on 8084 |
| `just ego-av` | the vehicle side for the ego, web UI on 8082 (`just ego-av rviz:=true` for RViz; any acb_launch argument as `name:=value`) |
| `just build-autoware-overlay` | Autoware fixes 1.5.0 lacks, built from a pinned fork commit into `third_party/autoware_overlay/`; `ego-av` / `bg-av` source it by default (`autoware_overlay:=false`: off; roadmap 042) |
| `just scenario <file>` | one scenario (web UI on 8081); verdict in `<output_directory>/scenario_test_runner/` (`output_directory:=`, default `/tmp`) |
| `just two-av` | two Autoware stacks, one scenario; `record:=true` records a screencast |
| `just prune-logs [days:=7]` | move runs older than that out of the log root into the archive on `/mnt/disk1` (never deletes) |
| `just acceptance <file> <runs>` | repeat a scenario and summarise |

The `just` recipes use [play_launch](https://pypi.org/project/play-launch/) (per-node logs
under `play_log/<kind>/<time>/`, a web UI per stack); everything they start is also startable
with plain `ros2 launch`, as the README shows.

Project settings (map, rviz, vehicle and launch choices, scenario output) are recipe
arguments, `name:=value` as for a launch file; host settings (ports, displays, install and
data paths such as `CARLA_MAPS`, the log root `CSB_LOG_ROOT`) come from the environment, made
permanent in `.env` or `.envrc`. The recipes refuse to run while one of the environment
variables that used to carry a project setting is set, naming the argument that replaced it
(`scripts/just_lib.bash`).

### Rules that cost hours when broken

- **Always `just build`**, never a bare `colcon build`: it adds `--symlink-install` and
  checks for the setuptools that breaks editable Python installs.
- **Stop every stack before building.** A running bridge or `acb_bridge` makes the install
  step fail with `Text file busy` -- after compiling everything.
- **This repo's `install/` is the one that runs.** `just build` here compiles the
  autoware_carla_bridge packages too (colcon's `--base-paths src`), and `just ego-av`
  sources this workspace last, so the `acb_bridge` a stack executes is
  `install/acb_bridge/...` in *this* repo. After an acb change, rebuild acb's workspace and
  then this one; after `git pull` moves a submodule, rebuild both.
- **Build the Autoware overlay once** (`just build-autoware-overlay`, again when its pin in
  `scripts/autoware_overlay.bash` moves). Without it `just ego-av` warns and runs stock
  Autoware, whose `blind_spot` never yields on a right turn (autoware_universe#13509).
- **Restart the simulation side after a build.** It is long-lived and keeps running the old
  binary.
- **The CARLA version is a Cargo feature** (`carla-0916` in both workspaces' `Cargo.toml`);
  `CARLA_VERSION` in the environment overrides it. A stray `CARLA_VERSION=0.10.0` in your
  shell builds the wrong API and reports correct code as a missing field.
- **Never destroy a vehicle with sensors still attached** (destroy children first): an
  orphaned IMU segfaults CARLA.
- **Kill processes by pid**, never `pkill -f <pattern>` -- the pattern matches your own
  shell -- and reap what a stopped stack leaves (`ps` for `ppid == 1`).

[`CLAUDE.md`](../CLAUDE.md) keeps the long form of these, with the incident behind each.

## Repository layout

```
src/
  carla_scenario_bridge/   Rust: the SSv2 backend (ZMQ server, coordinator, CARLA adapter)
    src/main.rs              entry point, CARLA connection loop, shutdown
    src/zmq_server.rs        ZMQ REP socket, protobuf dispatch
    src/coordinator.rs       the SSv2 handlers -> CARLA API calls
    src/entity_manager.rs    SSv2 entity name <-> CARLA actor
    src/coordinate_conversion.rs  ROS (right-handed) <-> CARLA (left-handed) frames
    src/traffic_light_mapper.rs   Lanelet2 signal <-> CARLA traffic light
    src/blueprint_boxes.rs   CARLA bodies chosen by declared size (roadmap 022)
    src/collision_monitor.rs, contact_report.rs  CARLA's contacts for the verdict (021)
    src/npc_tracking.rs      tracked NPC motion law (023)
    config/                  bridge_config.yaml, blueprint box tables
  csb_launch/              launch files (simulation, scenario, background AV); run_suite,
                           physical_verdict
  csb_examples/            starter scenarios: basic/, multi_av/, awf/ (installed)
  csb_interfaces/          ROS messages and services (weather)
  scenario_agent_relay/    the scenario's side of the vehicle agent protocol
  autoware_carla_bridge/   submodule: the vehicle side (sensors, vehicle interface, agent)
  scenario_simulator_v2/   submodule: our SSv2 fork
proto/                     SSv2 protobuf definitions (kept equal to the fork's)
scenarios/                 benchmark and gap reproducer scenarios
scripts/                   measurement and media tools
docs/                      user docs, design docs, roadmap
```

## How a scenario runs

SSv2's interpreter steps the scenario and, every frame, sends the simulator requests over
ZMQ + Protobuf (`simulation_interface`): `Initialize`, spawns, `UpdateFrame`,
`UpdateEntityStatus`, traffic light updates, despawns. `zmq_server.rs` decodes each and the
`Coordinator` turns it into CARLA calls -- NPCs are moved to SSv2's poses, the ego is driven
by CARLA's physics under Autoware's control, and CARLA advances one fixed step per frame in
synchronous mode. The ego's goals reach Autoware through the agent relay, not ROS, so the two
sides need no shared domain. Details: [architecture](design/architecture.md),
[protocol](design/ssv2-protocol.md), [time and ticking](design/time-and-ticking.md).

## Testing

- **Unit tests** (`just test`): the coordinator's policies, coordinate conversion, the
  blueprint chooser, the contact report, `run_suite` and `physical_verdict`. Pure logic is
  kept apart from the CARLA calls so it can be tested without a server.
- **Live runs**: `just run`, `just ego-av`, then `just scenario <file>` or `ros2 run
  csb_launch run_suite <dir> --physical-verdict`. The starter scenarios are the regression
  set; `scenarios/gap/` holds reproducers for fixed SSv2-vs-CARLA disagreements.
- Before trusting a surprising measurement, check the running binaries against the source
  (`ls -l install/.../acb_bridge`, `/proc/<pid>/cmdline`) and the host's load (`uptime`):
  stale builds and other users' jobs have both looked like regressions here.

## Design documents

| Document | About |
|---|---|
| [Architecture](design/architecture.md) | design principles, the responsibility split, the main flows |
| [SSv2 protocol](design/ssv2-protocol.md) | the ZMQ messages the bridge implements |
| [SSv2 launch configuration](design/ssv2-launch-configuration.md) | how SSv2 is launched against CARLA |
| [Time and ticking](design/time-and-ticking.md) | who ticks CARLA, `/clock`, synchronous mode |
| [Failure handling and the frame budget](design/failure-and-frame-budget.md) | what fails a frame, what retries |
| [User workflow and the vehicle agent](design/user-workflow.md) | the two sides, the agent protocol |
| [Writing scenarios](design/scenario-authoring.md) | what scenarios may use, and how |
| [SSv2 feature completeness](design/ssv2-feature-completeness.md) | which SSv2 features work on CARLA |
| [Multi-instance architecture](design/multi-instance-architecture.md) | several stacks on one host |
| [CARLA server tuning](design/carla-server-tuning.md) | quality, GPU, startup |
| [Fork policy](design/fork-policy.md) | what lives in our SSv2 fork and why |
| [Gap campaign](design/gap-campaign.md) | finding SSv2-vs-CARLA verdict disagreements |
| [Two Autoware installs on one host](design/duplicate-autoware-installs.md) | keeping a source build and the binary packages apart |
| [Survey: Isaac Sim and digital twins](design/survey-isaac-sim-and-digital-twin.md) | other simulators, real-site twins (roadmap 019) |

The [roadmap](roadmap/README.md) has one document per phase: the problem, the design, the
steps and the measured results.

## Conventions

- **Docs before code**: a phase's roadmap document states the design before it is built,
  and records the results after.
- **Submodules**: push a submodule commit to its remote *before* committing the new pin
  here; a pin to an unpushed commit breaks every clone. SSv2 and Autoware patches live on
  our forks (NEWSLabNTU), not upstream.
- **Commits** are small and topical (`feat(022): ...`, `docs(023): ...`), each building and
  passing `just check`.
