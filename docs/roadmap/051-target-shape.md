# Phase 051: The Target Shape, at Once

Refactor the Autoware vehicle side and the simulation interface straight to the shape the
designs end in, in one landing, with no intermediate releases and no compatibility shims.
This replaces the stepwise plan of 048 (the vehicle layer and the profile), 049 (the
vehicle spec), and the design's step 8 (the repository split), which this phase absorbs.
Designs: [design/vehicle-side.md](../design/vehicle-side.md),
[design/simulator-interface-split.md](../design/simulator-interface-split.md).

**Status**: in progress, 2026-10-23. K0 done (below, "Progress").

**Source**: user direction 2026-10-23: "now that we'll go through Autoware package and
simulation interface refactor in phase 50 and so on, I'd like to refactor them to the
target shape at once". Also carried over from the same day:

- share packages across simulators only if future simulators are guaranteed to work with
  them;
- one source for vehicle dynamics, physics and sensors across Autoware, the simulators
  and SSv2;
- CARLA gets the spec's physics;
- Autoware connects to CARLA and to Isaac Sim without SSv2.

**Depends on**: [050](050-model3-box.md), the Model 3's box fixed and measured first.
Doing it first keeps its before and after free of this phase's changes.
[046](046-unified-world-interface.md), parts A and B.
**Feeds**: [039](039-isaac-multi-autoware.md) (several egos, each naming a spec);
[047](047-off-road-surfaces.md) (road friction on the spec's tires); any third simulator.

## The target

Eight repositories:

| repository | holds | depends on |
|---|---|---|
| `sim_vehicle_spec` (new; the car, for any autopilot) | vehicle specs (`vehicles/<car>.yaml`), the spec format and the generator library; the outputs no autopilot owns, generated: `carla_physics.yaml`, `carla_sensors.yaml`, Isaac's rig, SSv2's catalog (`catalogs/vehicles.xosc`); `spec_hash` | nothing |
| `autoware_sim_vehicle` (new; the car, for Autoware) | per-car Autoware packages (`<car>_description`, `<car>_launch`, `<car>_sensor_kit_description`, `<car>_sensor_kit_launch`, `individual_params`), generated from the spec; the pedal maps (measured); the Autoware profile (`autoware_sim_profile`); the I5 contract; `vehicle_check` (contract and dynamics) | `sim_vehicle_spec`, Autoware |
| `swb_world_protocol` (new) | `swb_world.proto`; the Rust shell (`WorldServer` trait, loop, lease) and the world client; the Python shell (`World` base class, `main`); the conformance check | nothing simulator-side, nothing SSv2-side |
| `swb_carla_world` (new, from `src/carla_world`) | the CARLA world server; its config and blueprint tables; `carla-start` and the CARLA tooling; `up.sh`; CARLA's weather message and service | `swb_world_protocol`, `sim_vehicle_spec` (spawn physics) |
| `swb_isaac_world` (new, from `src/isaac_world`) | the Isaac world server, the USD and vehicle tooling, `up.sh` | `swb_world_protocol`, aib |
| `autoware_carla_bridge` (acb) | the CARLA vehicle bridge, CARLA's standalone bring-up and demo (`demo_scenario.py`, `auto_drive`), the map tools | `sim_vehicle_spec`, `autoware_sim_vehicle` |
| `autoware_isaac_bridge` (aib) | the Isaac vehicle library and `aib.runner` | `sim_vehicle_spec`, `autoware_sim_vehicle` |
| `scenario-world-bridge` (this one; the adapter and the workspace) | `swb_sim_interface` (SSv2's protocol as a package, L0); the bridge; the relay; `swb_agent` with the vehicle agent; `swb_launch` (scenario, simulation, ego); the run tooling (`sim:=<name>` finds `swb_<name>_world`'s `up.sh`); `swb_examples`, whose scenarios reach the catalog in `sim_vehicle_spec` through `$(find-pkg-share …)`; the gap oracle | `swb_world_protocol`; pins every repository above, flat |

What depends on what:

- **Submodules only, no cycles, no duplicate packages** (decision 4): every repository
  above is a direct submodule of scenario-world-bridge. A repository that needs another
  to build on its own carries it as a submodule under `deps/` (acb and aib:
  `deps/sim_vehicle_spec` and `deps/autoware_sim_vehicle`; `autoware_sim_vehicle`:
  `deps/sim_vehicle_spec`; a world server: `deps/swb_world_protocol` and
  `deps/sim_vehicle_spec`, or aib). `deps/` holds a committed `COLCON_IGNORE`, so a colcon run over the
  superproject's `src/` never sees a second copy; the component's own `just build` names
  `deps/*` as base paths directly, which the marker does not block. No vcstool, no
  `.repos` file.
  - A check in this repository's `just test` (the pin check) fails when a component's
    `deps/<repo>` pin is not an ancestor of, or equal to, this repository's pin of the
    same repository: the workspace never builds a component against an older dependency
    than the one it was tested with standalone.
  - The superproject's `just setup` initialises its direct submodules and leaves the
    components' `deps/` uninitialised (it has its own copies); a component's `just setup`
    initialises its `deps/`.
- **No copies**: every output lives in exactly one repository. What describes the car to
  a simulator or to SSv2 lives in `sim_vehicle_spec`; what describes it to Autoware lives
  in `autoware_sim_vehicle`. Consumers read the installed files. Another autopilot adds
  its own `<autopilot>_sim_vehicle` next to Autoware's, generated from the same spec with
  `sim_vehicle_spec`'s generator library, and takes goals through the relay as any agent
  does ([agent-interface](../agent-interface.md)). If it cannot read the I5 contract's
  Autoware messages, it needs its own vehicle bridge.
- **Without SSv2**: acb or aib, plus the two vehicle repositories, is a complete Autoware-on-a-simulator
  system. Neither needs swb, `swb_world_protocol` or the other bridge. On CARLA this
  includes the manual scene: no scenario runner, the scene set up by hand in CARLA, acb
  attaching to whatever `hero` appears ([vehicle-side](../design/vehicle-side.md)).
- **Without Autoware**: a world server plus the adapter runs SSv2's NPC scenarios (the
  benchmarks do today).

## What changes, by area

### Vehicle side (was 048, 049)

- **The profile**: acb's `carla_simulator.launch.xml`, without the vehicle interface and
  the agent, becomes `autoware_sim_profile/launch/autoware.launch.xml`, with the
  arguments renamed `sim_*`.
  - acb's `carla_simulator.launch.xml` becomes the profile plus `acb_bridge`.
  - aib's `isaac_simulator.launch.xml` becomes the profile with nothing added.
- **The agent**: `swb_agent`'s `autoware_agent` takes ROS parameters and the local mode.
  swb's ego launches start it as `vehicle_agent`. `acb_pilot agent` is deleted.
- **The spec, version 2**: aib's format, `aib.spec` and `aib.gen` move to
  `sim_vehicle_spec`, with a `carla:` section and outputs for Isaac's rig, CARLA's physics
  and sensors, and the SSv2 catalog. Autoware's generator is in `autoware_sim_vehicle`, on
  the same library. `check-generated` runs in both repositories.
- **`spec_hash`**: a SHA-256 of the spec in canonical form (parsed, keys sorted, numbers
  in one format; comments and whitespace do not count). Every generated output carries
  the hash of the spec it came from. At start each side reports the hash it loaded:
  the world server in `SpawnReply` (the physics or rig it applied), the vehicle bridge
  on attach, the vehicle agent when it registers with the relay (the autopilot's vehicle
  files; so it works for any autopilot), and the catalog. The bridge refuses to start the
  ego when they differ, naming who loaded which hash. Without SSv2, acb compares its hash
  with the Autoware packages' and warns. It catches what the build checks cannot see: a
  process running an old install. The pedal maps keep their own physics hash, over the
  physics sections only, so moving a sensor does not invalidate them.
- **Per-car Autoware packages** replace `acb_vehicle`, `aib_vehicle` and their kits. The
  sensing pipeline is generated from the spec's sensor list.
- **CARLA on the spec's physics**:
  - One writer per mode: with SSv2, `swb_carla_world` applies `carla_physics.yaml` after
    the spawn and acb_bridge verifies it (`carla_physics:=verify`); without a world
    server, acb_bridge applies it on attach (`carla_physics:=apply`, its launches'
    default), so a hero spawned by any client becomes the spec's car. `demo_scenario.py`
    only spawns.
  - acb_bridge refuses a hero whose blueprint is not the spec's, and reads the physics
    back on attach and refuses a mismatch, in both modes.
  - Road friction scales the spec's tire friction.
  - The brake and motor lags CARLA's model lacks go into acb_bridge's command path.
  - The pedal maps are re-measured and tagged with the spec's physics hash.
- **SSv2's catalog**: the generated `catalogs/vehicles.xosc`. Every scenario refers to it,
  and 050's lint becomes a catalog lint.
- **Spawn check**: `SpawnReply.body` carries the wheel layout, and the bridge refuses an ego
  more than 2 cm from the catalog.
- **`vehicle_check`**: the contract (topics, types, rates, frames, stamps,
  `/robot_description`) and the dynamics run (acceleration steps, braking, a steering
  sweep), with no SSv2, on both simulators.

### Simulation interface (was design step 8, and what 046 left)

- **`swb_world_protocol`** leaves this repository: `proto/swb_world.proto`, the
  `swb_core` world client and server modules, `src/swb_world_server`, and
  `swb_world_check conformance`.
- **`swb_sim_interface`**: SSv2's `simulation_interface` protos (today copied into
  `proto/`) as a package, with a test that they are byte-identical to the SSv2 fork's.
- **`swb_carla_world` and `swb_isaac_world`** leave for their own repositories with history
  (`git filter-repo`), each with `swb_<sim>_world.launch.xml`, `up.sh` (from `carla_up.sh` and
  `isaac_up.sh`) and its config. The bridge's tree no longer contains a simulator.
- **Weather**: `swb_interfaces/msg/Weather.msg` is CARLA's fourteen floats. It moves to
  `swb_carla_world` with CARLA's service. The bridge's `set_weather` and `get_weather` take the
  protocol's generic form (a preset and named parameters).
- **Time** (decision 5): an episode is a vehicle stack's lifetime. The design is in
  [time-and-ticking](../design/time-and-ticking.md), "Episodes and vehicle stacks".
  - `sim_ns = nanos(elapsed_seconds) + BASE_NS`. `BASE_NS` is one constant
    (1000 s) in `swb_world_protocol`, beside `nanos()`. It keeps ROS time far from 0,
    which ROS reads as "no clock yet", and from the small values that make
    `now() - duration` negative. Any process computes the time from the frame it holds;
    nothing is shared, stored or sent.
  - An episode change (a world load, a CARLA restart) ends every vehicle stack.
    acb exits when the CARLA episode it attached to is gone (a new world id, or a
    reconnect), before it publishes a frame of the new episode, so Autoware never sees
    time go back. `acb_bridge` is declared `on_exit="shutdown"` in its launch and never
    respawned, so its exit ends the whole stack. aib does the same on the Isaac world
    server's episode id.
  - aib's `clock_offset: wall` goes. Today `aib.ros` stamps sim time plus the wall clock
    at its creation, so on Isaac Autoware's time and SSv2's differ by that offset, and
    its one `EgoRos` per process keeps `/clock` running across episodes. It stamps
    `sim + BASE_NS` like acb, and ends the stack on an episode change.
  - The stack comes back through what roadmap 030 built: `swb-agent supervise` restarts
    a group whose agent exited, and `run_suite`'s restart ladder counts an episode change
    as a reason to restart. A restarted stack takes the new episode's map: the ego launch
    resolves `map_path` from the town (`$CARLA_MAPS/<town>`), and `up.sh` restarts a
    running stack whose map is not the scenario's instead of only noting it.
  - The protocol: `LoadWorldReply` and `StepReply` carry `episode_id`. `sim_time_ns` is
    monotonic within an episode; across episodes it may start again. A server may keep
    it running (Isaac's does) but nothing may depend on that.
  - Deleted: `episode_clock`, `clock_store`, `epoch_marker` on both sides, acb's epoch
    rule, its reconnect continuation and mid-session join path, the frame-count
    arithmetic around `load_world`, acb's 60 s uptime gate, and roadmap 016's gap 12
    (the two epochs differing after a double CARLA restart).
  - Kept: a scenario never spans an episode change (the load happens at `Initialize`),
    the pause between scenarios inside an episode, and `reset_settings = false` (the new
    world stays synchronous).
- **No shims**: every renamed package, launch argument, node and entry point is gone, not
  aliased. Old launch arguments that a wrapper used to accept are refused with a message
  naming the new one, as `SWB_RETIRED_ENV` does for environment variables.

## Decisions (for the user)

1. **Repository names**: `sim_vehicle_spec` (added 2026-10-23 with the split; the name is
   open to change), `autoware_sim_vehicle`, `swb_world_protocol`, `swb_carla_world`,
   `swb_isaac_world`, all under NEWSLabNTU. [decided 2026-10-23; the world servers'
   packages, binaries and launch files take the repository's name]
2. **Per-car package names**: `tesla_model3_description` and the rest, as in the table.
   [decided 2026-10-23: as written]
3. **The profile package**: `autoware_sim_profile`. [decided 2026-10-23]
4. **Workspace**: git submodules only. [decided 2026-10-23: every repository a direct
   submodule here; a component's own dependencies are submodules under its `deps/`,
   ignored by colcon in the superproject; no `.repos` files]
5. **The clock**: in this phase, as K4. [decided 2026-10-23: no epoch; an episode
   change ends every vehicle stack, which restarts on the new episode]
6. **`aib.ros`** (the rclpy vehicle interface and sensor publishers): it stays in aib.
   It moves when a second Python world server needs it. [decided 2026-10-23]

## How it lands

"At once" means one landing: nothing is merged to any `main` until the whole target
passes. Inside, the work is ordered so that each change of shape is checked before the
next one changes behaviour. That way a failure can be traced to the change that caused
it.

- **Branches**: `target-shape` in every repository involved, the new ones included. This
  repository's branch pins the others' `target-shape` heads. At the end each repository is
  rebased onto its `main` and fast-forwarded, submodules first, then the pins here (linear
  history; submodule commits pushed before the pins move).
- **Checkpoints**: each one has its runs written into this doc as it passes.
  - **K0. Baseline**, after 050 lands:
    - `play_launch dump` of the resolved launches: `just ego-av`, `bg-av`, `ego-isaac`,
      acb's `just sim`, aib's `just autoware`;
    - conformance against both world servers;
    - `just test`, `just e2e sim:=carla`, the CARLA suite, the gap set, the 20-NPC bench;
    - CARLA's step responses on today's physics, through a throwaway script that does what
      `vehicle_check`'s dynamics run will do.
  - **K1. Repositories and workspace, no change in behaviour.**
    - The new repositories are split out with history and the profile has moved.
    - `swb_sim_interface`, each component's `deps/` submodules and the pin check are in place.
    - The launch dumps are identical to K0, except for install paths and renamed names.
      A script lists any other difference.
    - Conformance, `just test` and `just e2e sim:=carla` pass.
  - **K2. One source of the car, CARLA's physics unchanged.**
    - The spec v2 and its generators are in place in `sim_vehicle_spec` and
      `autoware_sim_vehicle`; the per-car packages replace acb's and aib's; the catalog,
      the spawn check and the `spec_hash` start check are in.
    - The generated Autoware files are identical in value to K1's.
    - The suite's numbers are unchanged from K0, except where the catalog's `Performance`
      differs from what a scenario had.
  - **K3. CARLA on the spec's physics.**
    - `carla_physics.yaml` is applied and read back; the pedal maps are re-measured; the
      lags are in acb.
    - `vehicle_check` runs on CARLA and on Isaac. Their step responses go into a table with
      K0's.
    - Every change in the suite is explained by the physics.
  - **K4. The clock** (decision 5).
    - The epoch code is gone on both sides; time is `nanos(elapsed) + BASE_NS`
      everywhere.
    - Checks:
      - `basic/` with the ego stack up through `town02_episode_change`: acb exits at the
        load, the stack ends, the supervisor restarts it on Town01's map, and the next
        Town01 scenario passes;
      - a scenario on Town02 started against a Town01 ego stack: `up.sh` restarts it on
        Town02's map;
      - CARLA restarted under a running stack: the stack ends, never publishing a frame
        of the new server;
      - `swb_carla_world` and the bridge restarted under a running stack: nothing changes
        for the stack, and SSv2's time agrees with acb's stamps, bit for bit;
      - acb standalone (`just sim`, `demo_scenario.py`): the town is loaded before acb
        attaches, and a later load ends the stack with the message.
  - **K5. No shims, the docs.**
    - Old names are refused. CLAUDE.md, the user guide, the READMEs and the design docs
      are updated.

## Acceptance

- [x] **Shape**: the bridge's repository has no simulator package and no simulator crate
  in its graph. acb and aib depend on the two vehicle repositories only, and the world
  servers on `sim_vehicle_spec`, not on Autoware. The pin check passes. Each
  repository builds and tests on its own from a fresh clone with its `deps/` submodules.
- [x] **One source**: changing one spec value without regenerating fails `check-generated`
  in every repository holding an output, and fails the catalog lint.
- [x] **Spawn**: an ego more than 2 cm from the catalog is refused with both boxes in the
  message. On CARLA, acb's physics read-back matches.
- [x] **Start**: a vehicle side built from a different spec commit than the world server
  is refused before the scenario starts, with both hashes and who loaded them.
- [x] **The same car**: `vehicle_check`'s dynamics run passes on CARLA and on Isaac without
  SSv2, within the spec's limits and acceptance.py's 0.35 m/s². The CARLA and Isaac table
  is in the results, with CARLA's remaining gap named (ABS, regen).
- [ ] **Without SSv2**: from acb's own workspace, `just sim` and `auto_drive` drive Town01
  to arrival. From aib's, `just isaac-ego` and `just autoware` do the same.
  (K5: acb's half is done; aib's is open for newslab-server139, see K5.)
- [x] **The manual scene** (run synchronous, see K5): CARLA asynchronous with no swb process; a Model 3 spawned as
  `hero` from a plain CARLA Python client; traffic from CARLA's `generate_traffic.py`;
  acb's `just autoware map:=Town01`. acb applies and verifies the spec's physics, Autoware
  localizes, takes a goal from RViz and arrives, stopping at CARLA's own cycling lights.
  A hero of another blueprint is refused with both names.
- [ ] **With SSv2**: `just e2e sim:=carla` and `just e2e sim:=isaac` PASS. The CARLA suite
  and the gap set have every difference from K0 explained by K3's physics or K4's clock.
  (K5: CARLA's half is done; `e2e sim:=isaac` is open for newslab-server139, see K5.)
- [x] **A future simulator**: a fake vehicle side passes `vehicle_check`. A fake world
  server (Python, on the shell) passes conformance. Both are built from the repositories'
  docs alone, as the check that the contracts are enough.
- [x] `just test` and `just check` pass in every repository.

Isaac's runs need a host that renders Town01: newslab-server139, with its `.env` moved
from `CSB_*` to `SWB_*` first.

Not in this phase: upstreaming the SSv2 patches; a second car; NPC and background-AV
physics from specs; `aib.ros` as a shared library.

## Progress

### K0. Baseline (2026-10-23, `69b7a99`, newslab-118-server)

Logs in `play_log/051/k0/`.

- **Launch dumps**: `scripts/dump_launches.sh play_log/051/k0/launch` resolves nine stacks
  without starting them (a `play_launch` shim turns each recipe's `play_launch launch` into
  `play_launch dump`): `ego-av`, `bg-av`, `ego-isaac`, `run` (CARLA), `run sim:=isaac`,
  `isaac-world`, acb's `sim` and `autoware`, aib's `autoware` (aib's workspace built here for
  it; it needs `ACB_SETUP` and a `map_path`). All nine resolve.
- **Conformance**: the CARLA world server alone on CARLA 2010 with Town01, 24 passed, 0
  failed (autopilot skipped: needs a road). The Isaac world server headless with no ego and
  its default world, **on this host** (driver 615; no Town01 render), 23 passed, 0 failed.
- **`just test`** passes (418 Rust, the Python suites, the scenario lint over 35 files after
  047's two new scenarios were given the measured box, `69b7a99`). **`just e2e`** on CARLA:
  PASS.
- **The suite, the gap set, the 20-NPC bench**: 050's after-run (`play_log/050/after/`,
  report `play_log/050/report.md`) is the baseline; the only commit between it and K0 (047
  S4-S6) touches Isaac and two new surface scenarios, nothing on the CARLA path.
- **CARLA's step responses** (`scripts/carla_step_response.py`, direct CARLA controls on the
  blueprint's own physics, synchronous 0.05 s x 16 substeps; two runs identical to 0.001):

  | throttle | v at 1 / 2 / 4 / 8 s, m/s | peak accel, m/s2 |
  |---|---|---|
  | 0.3 | 0.0 / 2.79 / 4.42 / 5.74 | 8.46 (a lurch after a 1 s dead start) |
  | 0.6 | 3.06 / 6.18 / 10.79 / 15.91 | 4.02 |
  | 1.0 | 5.40 / 12.04 / 24.52 / 44.24 | 6.94 |

  | brake from 10.4 m/s | stop time, s | distance, m | mean decel, m/s2 |
  |---|---|---|---|
  | 0.3 | 4.0 | 18.83 | 2.60 |
  | 0.6 | 2.9 | 14.90 | 3.59 |
  | 1.0 | 2.2 | 11.94 | 4.73 |

  | steer at 5 m/s | yaw rate, deg/s | curvature, 1/m |
  |---|---|---|
  | 0.05 | 5.21 | 0.0182 |
  | 0.10 | 10.08 | 0.0353 |
  | 0.15 | 14.88 | 0.0527 |


### K1. Repositories and workspace (2026-10-23, `7b5822a` on `target-shape`)

New repositories, public under NEWSLabNTU, each on a `target-shape` branch, split with
`git filter-repo` from the paths' history (pre-046 names included):

| repository | from | holds |
|---|---|---|
| swb_world_protocol | `proto/`, `src/swb_core`, `src/swb_world_server`, `swb_world_check` | the protocol, the Rust shell and world client and shared world policy (`swb_core`), the Python shell, `swb_world_check` (a package of its own) |
| swb_carla_world | `src/carla_world`, `scripts/measure_blueprint_boxes.py` | the CARLA world server, renamed: package, binary, node, `swb_carla_world.launch.xml`, `swb_carla_world.yaml` |
| swb_isaac_world | `src/isaac_world`, swb_launch's Isaac launch file and wrapper | the Isaac world server as a ROS package: module `swb_isaac_world`, `swb_isaac_world.launch.xml`, `swb_isaac_world_server` |
| autoware_sim_vehicle | acb's `acb_launch`, `acb_pilot`, component sync scripts | `autoware_sim_profile` (`launch/autoware.launch.xml`; switches `sim_*`), `autoware_sim_pilot` (`auto_drive`, `capture_poses`), the profile's tests (from acb's `launch_profile.rs`) |

acb (`target-shape`) is the profile plus `acb_bridge`; aib (`target-shape`) includes the
profile and no longer needs acb. The vehicle agent is `swb_agent`'s `vehicle_agent`, started
by `swb_launch vehicle_agent.launch.xml`, which the ego launches and recipes hand the vehicle
bridge as `extra_launch` (a user's own comes in as `user_launch`).

Checks, all passing:

- **Launch dumps**: `scripts/compare_launch_dumps.py play_log/051/k0/launch
  play_log/051/k1/launch`: the nine stacks resolve to the same nodes (207, 208, 206, ...),
  parameters, remappings and respawns as K0, but for install paths and the renames above
  (`acb_agent` -> `vehicle_agent` included). A planted relay change is reported.
- **Conformance**: CARLA world server 24 passed, Isaac world server 23 passed (headless,
  default world, this host).
- **`just test`**: the bridge's crates (70), the protocol's (187, in its own workspace, with
  the gap trace read back by this repository's reader), the CARLA world server's (160, with
  Isaac's surface table for the wet-road comparison), the Python suites, the profile's 17,
  the SSv2 proto byte check, the scenario lint, and the pin check (5 component dependencies,
  0 mismatches). **`just e2e`**: PASS.

Deviations from the plan above, and why:

- **SSv2's message protos live in swb_world_protocol**, beside `swb_world.proto`: `swb_core`'s
  `SimBackend` seam and the shared world policy (which the CARLA world server uses whole) are
  typed in them. No `swb_sim_interface` package; `scripts/test_ssv2_protos.py` here checks the
  copy against the SSv2 fork byte for byte. Making the protocol repository free of SSv2 needs
  the seam moved to the protocol's own types: a phase of its own.
- **Components' `deps/` are initialised in this workspace too** (`git submodule update
  --init --recursive`), COLCON_IGNOREd, and a component always builds against its own
  `deps/` (Rust path dependencies, Python paths): it builds the same here as alone. So the
  pin check requires a `deps/` pin to *equal* this repository's pin of the same repository,
  not merely be an ancestor.
- **`up.sh` stays here** as `scripts/carla_up.sh` and `scripts/isaac_up.sh`: they bring up
  this repository's stacks (the simulation side, the ego), so they are the adapter's tooling.
- **`autoware_sim_pilot`** (acb's `auto_drive` and `capture_poses`) moved too: Autoware's API
  only, so both bridges' standalone runs drive to a goal with it.
- Not yet done: CARLA's `Weather.msg` move (with K5's interface clean-up), the docs (K5).

### K2. One source of the car (2026-10-23, `c1d59af` on `target-shape`)

One spec per car, in **sim_vehicle_spec** (new, public): `vehicles/tesla_model3.yaml`
(version 2), the library (`spec`, `emit`), and what it generates: CARLA's sensor file
(`generated/<car>/carla_sensors.yaml`), the SSv2 vehicle catalog (`catalogs/vehicles.xosc`).
Every output names the spec's `spec_hash`. From the same spec, **autoware_sim_vehicle**
generates Autoware's packages per car (`tesla_model3_description`, `_launch`,
`_sensor_kit_description`, `_sensor_kit_launch`, `individual_params`), and **aib** generates
its Isaac rig. acb's `acb_vehicle*`, `acb_sensor_kit*`, `individual_params` and
`vehicle_config.yaml`, and aib's spec, `aib.spec` and `aib_*` packages are gone; everything
selects `vehicle_model:=tesla_model3`.

- **The catalog (K2d)**: every scenario Model 3 (134, in 33 files) is a `CatalogReference`
  to `tesla_model3` through `$(find-pkg-share sim_vehicle_spec)/catalogs`, assigning the
  four values it chooses (maxSpeed, maxAcceleration, maxDeceleration, maxSteering) from what
  it had inline (`scripts/catalog_scenarios.py`; the generators emit references). The catalog
  declares **no defaults**: SSv2 evaluates an entry's ParameterDeclarations in the entry's own
  scope, where they shadow the reference's assignments (found by a probe run that failed on
  SSv2's acceleration range, 13 against the catalog's 9.8). The 050 lint is a catalog lint;
  truck_pass's 12 m truck on the Model 3 body is exempt.
- **The spawn check (K2e)**: `Body.wheels` (the protocol); the CARLA world server reports the
  measured axles moved into the entity frame, Isaac's the rig's; the bridge refuses an ego
  whose box or wheels are more than 2 cm from SSv2's declaration (the catalog's), each field
  named with both values.
- **The start check (K2f)**: `Body.spec_hash`, the vehicle agent's registration (from
  `<vehicle_model>_description/config/spec_hash.yaml`, through the relay's query), and the
  installed catalog's entry for the ego's body (read by the bridge: SSv2's `Property` message
  is empty, so a catalog property does not reach it in the spawn). The bridge refuses an ego
  whose parts name different specs; a part that names none is left out (CARLA's world server
  until K3). acb, standalone, warns when its sensor file and Autoware's package disagree.
  Seen in the e2e: `the world server (carla) none, the vehicle agent d5d4b266c7fd, the catalog
  d5d4b266c7fd`.

Checks:

- **Generated values**: Autoware's files equal K1's but for the overhangs (front 0.923 ->
  0.807, rear 0.864 -> 0.980: measured against the body instead of a centred wheel base), and
  the Isaac rig's box centre (1.532 -> 1.416).
- **Launch dumps** (`play_log/051/k2/launch`, against K1's): the same nodes in all nine
  stacks; the differences are the overhangs and what derives from them (the footprint
  offsets, the RViz body: CARLA's ego now has aib's generated box instead of acb's Lexus mesh,
  and no camera0 links, which nothing spawned), the renamed packages and paths, and two new
  parameters (`vehicle_model` on the agent, `autoware_spec_hash_file` on acb_bridge).
- **Conformance**: CARLA 24 passed, Isaac 23 passed. **`just test`** passes, the pin check 8
  dependencies, 0 mismatches. **`just e2e`**: PASS.
- **The suite** (`scripts/run_050_set.sh 051_k2`, report `play_log/051/k2_report.md`, against
  050's run, K0's baseline): the same verdicts (bench "missing" in both: it has no
  verdict). The numbers are within run-to-run noise but for one systematic shift, explained:
  the gaps to stopped leads shrink by 0.11-0.14 m (i1 lead 5.43 -> 5.29, i4 broken 5.44 ->
  5.33): Autoware's outline is the body now (front overhang 11.6 cm shorter), so it stops
  that much closer. No catalog `Performance` differs from what a scenario had (each assigns
  its own). truck_pass, four runs: SSv2's boxes meet (-2.2 m), CARLA's bodies stay ~1.0 m
  apart, as in 050's four.

### K3. CARLA on the spec's physics (2026-10-24, `03e065e` on `target-shape`)

CARLA's ego now runs the car the spec describes, not the blueprint's. sim_vehicle_spec
generates `generated/<car>/carla_physics.yaml` from the spec plus its `carla.physics`
block (the few values CARLA's model needs that the spec does not state: tire friction scale,
drag area, body CoM, rear axle, steer limit, engine inertia and damping). The file holds the
mass, the CoM nudge, the motor's torque curve on one gear, and per wheel the friction,
stiffnesses and brakes. It also holds `command_lags`, the actuator delays CARLA lacks (brake
dead time 0.08 s, brake time constant 0.06 s, motor 0.04 s), and names its `spec_hash` and a
`physics_hash`. Python (`sim_vehicle_spec.carla_physics`) and Rust (`crates/carla_physics_spec`)
both read it.

- **Who applies it**: the CARLA world server, for a scenario's ego (ROS parameter
  `vehicle_physics`, default the installed file; `none` turns it off). It applies the file
  before the spawn's settling tick, reads it back after that tick (CARLA applies physics at
  its next tick, so a read straight after the apply returns the old values) and refuses an
  actor that differs. `SpawnBody.spec_hash` now names the spec, so the start check has all
  three parts. acb has `carla_physics_mode`: `apply` (standalone, acb owns the hero) or
  `verify` (under SSv2, the world server wrote it; `just ego-av` and the background AV pass
  `verify`). A wrong blueprint is refused by name.
- **The lags** are in acb's command path (`pedal_lag.rs`): first-order on each pedal plus the
  brake's dead time. The filter snaps to its target within 1e-3. Without the snap, f32 decay
  left a brake of ~1e-6 for 5.2 s after release, and CARLA held the car with it.
- **The pedal maps** were re-measured on this physics (acb's probes, `--physics`).
  `pedal_maps.yaml` records `measured_on: spec` and the `physics_hash`, and
  autoware_sim_vehicle's packages were regenerated from them.
- **vehicle_check** (autoware_sim_vehicle `autoware_sim_check`, `vehicle_check.launch.xml`):
  it checks the vehicle-interface contract (topics, types, rates from stamps, frames, /clock,
  /robot_description), then steps acceleration and brake and judges them against 0.35 m/s^2,
  then sweeps the steering against the bicycle model. A step that makes no progress for 120 s
  of wall time fails rather than hangs.

Step responses (`scripts/carla_step_response.py`; Isaac through aib's runner on the same
spec):

| | CARLA blueprint (K0) | CARLA, spec physics (K3) | Isaac, spec |
|---|---|---|---|
| 0-100 km/h | 4.6 s | 6.35 s | 6.45 s |
| 100-0 km/h | 49.5 m | 43.09 m | 42.9-44.1 m |
| coast at 25 m/s | 5.58 m/s^2 | 0.20 m/s^2 | 0.276 m/s^2 |

The blueprint coasted like a car in a hard brake. The spec's car coasts on drag and rolling
resistance, as a Model 3 with regen off does. The two simulators now agree within 0.1 s and
about 1 m.

vehicle_check, CARLA (`play_log/051/k3/vc_carla4`): the contract is clean. Acceleration and
brake steps of ±1 to +3/-4 m/s^2 track within 0.01 m/s^2, and the yaw rate is within 0.4 % of
the bicycle model across ±0.05-0.15 steer. Isaac (`vc_isaac6`): the contract is clean,
acceleration tracks within 0.004, and the brakes measure -0.98/-2.36/-3.70 for -1/-2.5/-4. The
last is 0.30 short, inside the 0.35 limit: the motor's regen fades at low speed and the
friction brakes lag. The yaw rate is within 1.1 %.

Checks:

- `just test` passes. `just e2e`: PASS, with the three parts naming one spec
  (`862c2168f77e`).
- **The suite** (`scripts/run_050_set.sh 051_k3b`, report `play_log/051/k3_report.md`,
  against K2): the same verdicts in every scenario (truck_pass fail and the bench "missing",
  as in K2 and 050). The numbers that move do so because of the physics:
  - Gaps to stopped leads grow 0.23 m (i1 lead 5.29 -> 5.52, i4 broken 5.33 -> 5.56; in the
    tour 4.74 -> 5.58 and 5.29 -> 5.58). The pedestrian darts in i2 grow 0.68-0.76 m
    (4.80 -> 5.48). The ego now brakes through the re-measured maps on a car without the
    blueprint's 5.6 m/s^2 of hidden coast braking, so it ends nearer where the planner puts
    it.
  - The short gap reproducers drive less in their fixed time (entity_bodies 13.3 -> 10.7 m,
    motorbike_body 21.2 -> 14.6 m), because 0-100 km/h is 6.35 s now, not 4.6. In
    motorbike_body the motorbike closes on a slower ego (min gap 0.68 -> 0.36 m), and SSv2 and
    CARLA agree within 2 cm (they were 8 cm apart in K2).
  - Everything else is within run-to-run noise.
- **Found on the way**: the ego stack executes acb's own install,
  `src/autoware_carla_bridge/install`, not this repo's. `ros2 pkg prefix acb_bridge` resolves
  there because acb's overlay is sourced after this repo's underlays. CLAUDE.md says the
  opposite, so K5 corrects it. A stale acb install made the first K3 suite (`051_k3`) fail
  motorbike_body and stall starts. After acb's workspace was rebuilt, the same scenario
  passed, and the whole set was run again as `051_k3b`.

### K4. The clock (2026-10-24, `target-shape`)

Time is `nanos(elapsed) + BASE_NS` (1000 s) everywhere: swb_core's `sim_time.rs`, the Python
world-server shell, acb's `clock.rs`, aib's `sim_time.py`. Each rounds half away from zero.
`scripts/test_sim_time.py` (in `just test`) checks that BASE_NS and a table of rounding cases
agree across all four. The epoch code is gone: `episode_clock`, `clock_store`, the epoch
marker, the reload-as-pause in the CARLA world server, acb's epoch rule, and aib's
`clock_offset`, which aib now refuses by name.

- **The protocol** names the episode: `LoadWorldReply.episode_id` and
  `StepReply.episode_id`. CARLA's is its world id. The Python shell counts world loads. The
  conformance check now has two more checks: the time is past BASE_NS, and the same world
  loaded again keeps its episode with time going on. The bridge warns when the episode
  changes.
- **An episode change ends every vehicle stack.** acb attaches to the episode it first sees.
  On a frame or a hero of another episode, it logs the change naming both ids, flushes its
  log writer and exits with status 3, before publishing anything of the new episode. Its
  launch node is `on_exit="shutdown"` and is never respawned. The Autoware profile's
  `clock_guard` (required, `sim_clock_guard:=true`) is the backstop for any simulator: it
  ends the stack when `/clock` goes back.
- **The stack comes back**: `carla_up.sh` runs the ego stack under `swb-agent supervise`,
  which starts it again in the new episode. A stack on another map than the scenario's is
  stopped through its supervisor and replaced. That replacement ends once more at the
  scenario's load of its town, so a town change costs two stack starts (about 1 min each
  here). `run_suite` waits for a supervised agent whose stack ended to register again.
  `isaac_up.sh` restarts a stack on another map too.
- **acb standalone**: `just sim` loads its town first (`scripts/carla_load_town.py`), so
  the demo scenario finds it loaded and acb attaches to the episode that stays. Before
  this, a town other than CARLA's would have ended the stack at once.
- **Isaac, a deviation**: the Isaac world server's time is monotonic over its whole
  lifetime. aib's `EgoRos` lives in that process and keeps `/clock` running across world
  loads, so an Isaac stack is never ended by a load. A stack on another map is restarted by
  `isaac_up.sh`. `clock_guard` covers a restarted world server, whose time starts again.

Checks (`play_log/051/k4/`, CARLA 0.9.16 on newslab-118-server):

1. **Through an episode change** (`check1c`, run_suite with a supervised ego): Town01
   ego_drive, traffic_light, town02_episode_change, Town01 ego_drive, pedestrian: **5/5
   pass**. At the Town02 load acb exited with status 3 ("CARLA's episode changed (… -> …)"),
   the launch ended, the supervisor restarted the stack 9 s later, and run_suite waited for
   it. The next Town01 scenario's load ended that stack too, and the scenario passed on the
   third stack (77 s, against 51 s without the restart). No Autoware node logged a jump back
   in time.
2. **A Town02 scenario against a Town01 stack** (`check2b`, `just scenario`): Town01,
   Town02, Town01: **3/3 pass**, each town change as described above. The first try
   (`check2`), before the stack was supervised, failed both town changes. The replacement
   stack attached to the old episode and died at the load, which is why `carla_up.sh` now
   supervises it.
3. **CARLA restarted under a running stack** (`check3`): acb saw the connection die and a
   new episode, and exited before publishing anything of it. The CARLA world server exited
   75 and was respawned, the supervisor restarted the stack, and town01_ego_drive passed.
4. **The world server and the bridge restarted under a running stack** (`check4`,
   `check4b`): both were killed and respawned in 6 s, and the stack did not notice.
   town01_ego_drive passed twice. SSv2's time agrees with acb's: all 235 distinct stamps
   of SSv2's V2I signal message (scenario domain) equal a `/clock` value acb published (ego
   domain), bit for bit, and `/clock` never stepped back.
5. **acb standalone** (`check5b`, acb's `just sim map:=Town02`, CARLA on Town01):
   `carla_load_town.py` loaded Town02, the demo scenario found it loaded, and acb attached
   to its episode and found the hero. After 120 s, a load from outside ended the stack: acb
   exited with status 3 and logged the message, and the launch shut down.

6. **`just e2e`**: PASS. CARLA was still on Town02 from check 5, so the supervised Town01
   stack ended once at the scenario's load and came back (two starts). `just test` passes
   too, and the pin check finds 9 dependencies, 0 mismatches.

Also seen, not K4's: `pose_instability_detector` dies with "cannot store a negative time
point in rclcpp::Time" in some stack starts (ego logs since 2026-10-22, K0's time base
included). It is a diagnostic node, and the stack runs on without it. acb's `just sim`
never reports "Startup complete" here: rviz2 crash-loops without a display, and play_launch
waits on 8 perception composables. The bridge and the drive are unaffected.

### K5. No shims, the docs, acceptance (2026-10-24, `target-shape`)

- **No shims.** Renamed arguments are refused by name, never aliased. The Autoware
  profile's `check_launch_args` (autoware_sim_vehicle) refuses the `carla_*` switches, naming
  each one's `sim_*` replacement, and the old `acb_vehicle` / `aib_vehicle` / sensor-kit
  values, naming the car's packages. Every entry point runs it: swb's `ego-av`, `ego-isaac`
  and `bg-av` (`swb_reject_args`), acb's `just sim` / `just autoware` (which accept only
  `map:=`, `auto_drive:=` and `spawn_index:=`), and aib's `just autoware`. Removed outright:
  acb's `urdf_parser.rs`, the legacy `CarlaConfig` / `SensorTypeDefaults` / `SensorParams`,
  its urdf-rs dependency, `config/bridge.yaml` and the second copy of `demo_scenario.py`
  (now a symlink), and swb's `scripts/carla_step_response.py` (vehicle_check replaces it).
  A `map_origin` key in acb's sensor config is refused.
- **Weather is the world server's, by name** (a deviation: there is no typed CARLA weather
  service). `swb_core::weather` carries a preset or named parameters, in the server's own
  names. CARLA's presets and 14 fields live in `swb_carla_world/src/backend/carla/weather.rs`
  and its README, and the Python shell has `World.set_weather` / `get_weather`. The bridge's
  `SetWeather` / `GetWeather` (`swb_interfaces`) take `{preset, names[], values[]}`, and
  `Weather.msg` is gone. The conformance check's weather test is simulator-neutral: it
  changes the first parameter and reads it back.
- **The docs.**
  - CLAUDE.md now lists the eight repositories, supervised ego stacks, and which
    `acb_bridge` an overlay runs (corrected, see K3).
  - The README, `docs/user-guide.md` (the `sim_*` table, the refused arguments, generic
    weather, a town change ending the vehicle side, the signal-link repair for every town)
    and `docs/development.md` are updated.
  - Design docs: `time-and-ticking.md` is rewritten, `failure-and-frame-budget.md` is
    updated, `simulator-interface-split.md` and `vehicle-side.md` carry status notes, and
    the paths are renamed elsewhere.
  - Each component has its own README. The protocol has `docs/world-protocol.md` and
    `docs/world-server.md`, autoware_sim_vehicle has `docs/vehicle-contract.md`, and acb's
    CLAUDE.md is rewritten.
- **Every repository** has `just test` and `just check`, and all 16 pass
  (sim_vehicle_spec gained `check`, autoware_sim_vehicle `test`, swb_isaac_world `check`).
  The catalog lint, `check_scenario_vehicles.py`, now also fails a catalog entry whose
  `swb.spec_hash` is not its spec's.

Acceptance (`play_log/051/k5/`, CARLA 0.9.16 on newslab-118-server, port 2010):

- **Shape**: no simulator package or crate is in this repository's graph, and the pin check
  finds 9 dependencies and 0 mismatches. Fresh clones of all seven components, with their
  `deps/`, build and test on their own.
- **One source** (`one_source.txt`): one spec value changed without regenerating fails
  `check-generated` in sim_vehicle_spec (carla_sensors, carla_physics, the catalog),
  autoware_sim_vehicle (8 files, and the pedal maps' `physics_hash`) and aib, and it fails
  the catalog lint.
- **Spawn** (`spawn_refusal`): an ego box off the catalog is refused with every field and
  both values: "box centre x declared 1.450, body 1.479; length declared 4.860, body 4.792;
  front axle x declared 3.005, body 3.068; rear axle x declared 0.000, body 0.064". acb's
  read-back matches on every K3-K5 run ("The hero runs the vehicle spec's physics").
- **Start** (`start_refusal`): a vehicle side with another `spec_hash` is refused before
  the scenario starts, naming all three parts: "the world server (carla) 862c2168f77e, the
  vehicle agent 41, the catalog 862c2168f77e". The test's hash was all digits
  (`0…051`), and YAML read it as the octal integer 41. A real hash is hex, quoted or not, so
  this is cosmetic.
- **The same car**: vehicle_check passes on CARLA (`vc_carla2`) and on Isaac (`vc_isaac2`).
  The table and CARLA's remaining gap are in K3: the regen fade and ABS are not modelled, and
  Isaac's -4 m/s^2 brake is 0.30 short, inside 0.35.
- **Without SSv2**: from acb's workspace, `just sim spawn_index:=74 auto_drive:=true`
  arrives at the traffic-light goal (84.6 s). Spawn point 0 is held at its first corner by
  a street lamp (acb issue 029), so `spawn_index:=` was added. Bugs found and fixed on the
  way:
  - the demo ticked 2.7x real time (now paced);
  - the perception composables never loaded (isolated containers);
  - every light was held at 99999 s by swb_carla_world saving an already-held light's
    timings (now a held light gets CARLA's default phases);
  - `just autoware` connected to CARLA on 2000, not `CARLA_PORT`;
  - a second SIGTERM cut the demo's cleanup short, leaving CARLA synchronous with heroes in
    it.

  From aib's workspace (`just isaac-ego` and `just autoware` to arrival on Town01): **not
  run**. It needs Isaac to render Town01, and this host cannot (see "With SSv2" below). It
  stays open for newslab-server139, with the Isaac e2e. On a flat world here, aib's
  `just vehicle-check` passes.
- **The manual scene** (`manual_scene*`): CARLA is **synchronous** at 0.05 s, ticked in real
  time by a plain client, with no swb process. This deviates from the line above, which
  said asynchronous: the project runs CARLA synchronous for reproducibility, and an
  asynchronous scene's lidar arrives in slices that Autoware's pipeline does not gather.
  A Model 3 is spawned as `hero` at spawn point 74 from a plain client, with 30 vehicles and
  20 walkers from `generate_traffic.py`, acb's `just autoware map:=Town01`, and
  `auto_drive` to `scenarios/ego_poses.yaml` (in place of RViz's goal tool, same service).
  An observer logs the light state and each crossing of CARLA's stop waypoints.
  - Runs 1-2: arrived in 85.6 s and 96.5 s, but the ego crossed light 644's stop line
    **on red at 4.17 m/s**. Its approach lanelet referenced no light: the TUM map pack links
    most Town01 lights to a stop line on the next lane or at another junction (acb issue
    030). This is roadmap 031's defect 8, which had only been repaired on Town05; 28 of 36
    approaches had no link in their own direction. `scripts/repair_lanelet_signal_links.py
    Town01` relinks them.
  - The repair had two bugs of its own (`fe33862`):
    - its heading test was unsigned, so on a waypoint lying on the line two opposite lanes
      share it linked light 373 to the oncoming lane;
    - its stop line only spanned CARLA's lane, which missed Autoware's path by 0.2 m where
      the map's lanes sit 2.2 m off.

    With the first repair, the ego drove through SSv2's red in `town01_i3_red_light`; the
    K5 suite (`051_k5`) passed anyway, because i3 forces green after a while. After the
    fix, all 36 approaches are linked in their direction, and each stop line crosses its
    lane's centre. The map lives in `data/`, which git ignores, so the user guide and acb's
    README now say to run the repair for every town and restart the vehicle stack (a stack
    started before the repair kept the old map once).
  - On the final map (`manual_scene`): arrived in 99.7 s. The ego stopped 12 s for light
    644's red with its front at CARLA's line, and crossed both lines on green. Earlier runs
    on the first repair: 77.2 s (both lights turned yellow 2.7 m before the line and were
    passed, as Autoware should) and 94.3 s (an 11.5 s stop at 644's red). acb applied and
    verified the spec's physics each time.
  - A hero of another blueprint (`refuse_blueprint`, a Lincoln MKZ) is refused with both
    names: "the hero is a vehicle.lincoln.mkz_2020, but the vehicle spec's car is a
    vehicle.tesla.model3 (carla_physics 862c2168f77e): refusing it". No sensor is
    spawned.
  - The scene's own driver had a bug: a background job of a script starts with SIGINT
    ignored, so `generate_traffic.py` never ran its cleanup and left 28 vehicles behind. It
    now gets its KeyboardInterrupt back, and the world is left empty.
- **A future simulator**: a fake world server built from `world-server.md` and
  `world-protocol.md` alone passes conformance 25/25. A fake vehicle side built from
  `vehicle-contract.md` alone passes vehicle_check. CARLA's world server passes conformance
  25/25 (`conf_carla`), with the weather services live.
- **With SSv2**: `just e2e sim:=carla` PASS. `just e2e sim:=isaac` **could not run here**. On
  newslab-118-server the Isaac world server dies on loading Town01: an `omni.rtx` GPU page
  fault while it renders the landscape, about 34 s in, exit 139. That happened on all 36
  respawns in the 1800 s `isaac_up.sh` waits (`play_log/051/k5_e2e_isaac.log`, NVIDIA
  driver 615.71.09). This is the host's known Town01 render crash, not 051's code: Isaac's
  vehicle_check on a flat world passes here (K3), and its time and ego paths are unchanged
  since K4. The Isaac e2e stays open for newslab-server139, the host that renders Town01.
  - The CARLA suite on the final map (`run_050_set.sh 051_k5b`, report
    `play_log/051/k5b_report.md`, against K3's `051_k3b`) has the same verdicts in every
    scenario: 15 pass, truck_pass fails and the bench is "missing", as in K2, K3 and 050.
  - The numbers agree with K3 within run-to-run noise. `town01_i3_red_light` takes 86.8 s
    (K3: 86.9 s). The ego stops for SSv2's red 2.8 m before CARLA's line, centre of the car,
    where K3 stopped about 6 m past it on the pack's stop line. `town01_traffic_light` stops
    2.6 m before.
  - The two fixed-time gap reproducers drove 11.9 m and 18.0 m (K3: 10.7 m and 14.6 m; the
    first K5 suite 10.3 m and 14.8 m). That is engagement timing inside a fixed time. SSv2
    and CARLA still agree within 5 cm.
  - The first K5 suite (`051_k5`, on the first repair) had one more difference. A Town01
    plastic chair (a 0.5 kg dynamic prop) that earlier runs' traffic had knocked about
    touched the ego in the tour, with an impulse of 49 N·s. A world that is never reloaded
    keeps such props where they were knocked. Its i3 ran in 66 s because its ego never
    stopped (above).
  - On the default clustering stack (not the suite's tour stack, which runs the incidents
    on CenterPoint), i3 is held after its light at Town01's south-east corner by the street
    lamp of acb issue 029, 2 of 2 times.
  - `run_050_set.sh` left the bench's supervised ego running after each suite (K4 made
    `carla_up.sh` supervise it, and `stop_stack` only killed the play_launch). That
    stack would take the next hero on the port. It now stops the supervisor (`59521ef`).
