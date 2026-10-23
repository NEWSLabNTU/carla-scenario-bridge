# Phase 051: The Target Shape, at Once

Refactor the Autoware vehicle side and the simulation interface straight to the shape the
designs end in, in one landing, with no intermediate releases and no compatibility shims.
This replaces the stepwise plan of 048 (the vehicle layer and the profile), 049 (the
vehicle spec), and the design's step 8 (the repository split), which this phase absorbs.
Designs: [design/vehicle-side.md](../design/vehicle-side.md),
[design/simulator-interface-split.md](../design/simulator-interface-split.md).

**Status**: planned, 2026-10-23. Decisions 1-6 are taken; starts after 050.

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

Seven repositories:

| repository | holds | depends on |
|---|---|---|
| `autoware_sim_vehicle` (new; the vehicle layer) | vehicle specs (`vehicles/<car>.yaml`) and their generators; per-car Autoware packages (`<car>_description`, `<car>_launch`, `<car>_sensor_kit_description`, `<car>_sensor_kit_launch`, `individual_params`), generated; the Autoware profile (`autoware_sim_profile`); the I5 contract; `vehicle_check` (contract and dynamics) | Autoware only |
| `swb_world_protocol` (new) | `swb_world.proto`; the Rust shell (`WorldServer` trait, loop, lease) and the world client; the Python shell (`World` base class, `main`); the conformance check | nothing simulator-side, nothing SSv2-side |
| `swb_carla_world` (new, from `src/carla_world`) | the CARLA world server; its config and blueprint tables; `carla-start` and the CARLA tooling; `up.sh`; CARLA's weather message and service | `swb_world_protocol`, the vehicle layer (spawn physics) |
| `swb_isaac_world` (new, from `src/isaac_world`) | the Isaac world server, the USD and vehicle tooling, `up.sh` | `swb_world_protocol`, aib |
| `autoware_carla_bridge` (acb) | the CARLA vehicle bridge, CARLA's standalone bring-up and demo (`demo_scenario.py`, `auto_drive`), the map tools | the vehicle layer |
| `autoware_isaac_bridge` (aib) | the Isaac vehicle library and `aib.runner` | the vehicle layer |
| `scenario-world-bridge` (this one; the adapter and the workspace) | `swb_sim_interface` (SSv2's protocol as a package, L0); the bridge; the relay; `swb_agent` with the vehicle agent; `swb_launch` (scenario, simulation, ego); the run tooling (`sim:=<name>` finds `swb_<name>_world`'s `up.sh`); `swb_examples` with the generated vehicle catalog; the gap oracle | `swb_world_protocol`; pins every repository above, flat |

What depends on what:

- **Submodules only, no cycles, no duplicate packages** (decision 4): every repository
  above is a direct submodule of scenario-world-bridge. A repository that needs another
  to build on its own carries it as a submodule under `deps/` (acb and aib:
  `deps/autoware_sim_vehicle`; a world server: `deps/swb_world_protocol` and the vehicle
  layer or aib). `deps/` holds a committed `COLCON_IGNORE`, so a colcon run over the
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
- **Without SSv2**: acb or aib, plus the vehicle layer, is a complete Autoware-on-a-simulator
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
- **The spec, version 2**: aib's format, `aib.spec` and `aib.gen` move to the vehicle layer,
  with a `carla:` section and outputs for Autoware, Isaac's rig, CARLA's physics and
  sensors, and the SSv2 catalog. `check-generated` runs in every repository holding output.
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

1. **Repository names**: `autoware_sim_vehicle`, `swb_world_protocol`, `swb_carla_world`,
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
    - The spec v2 and its generators are in place; the per-car packages replace acb's and
      aib's; the catalog and the spawn check are in.
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

- [ ] **Shape**: the bridge's repository has no simulator package and no simulator crate
  in its graph. acb and aib depend on the vehicle layer only. The pin check passes. Each
  repository builds and tests on its own from a fresh clone with its `deps/` submodules.
- [ ] **One source**: changing one spec value without regenerating fails `check-generated`
  in every repository holding an output, and fails the catalog lint.
- [ ] **Spawn**: an ego more than 2 cm from the catalog is refused with both boxes in the
  message. On CARLA, acb's physics read-back matches.
- [ ] **The same car**: `vehicle_check`'s dynamics run passes on CARLA and on Isaac without
  SSv2, within the spec's limits and acceptance.py's 0.35 m/s². The CARLA and Isaac table
  is in the results, with CARLA's remaining gap named (ABS, regen).
- [ ] **Without SSv2**: from acb's own workspace, `just sim` and `auto_drive` drive Town01
  to arrival. From aib's, `just isaac-ego` and `just autoware` do the same.
- [ ] **The manual scene**: CARLA asynchronous with no swb process; a Model 3 spawned as
  `hero` from a plain CARLA Python client; traffic from CARLA's `generate_traffic.py`;
  acb's `just autoware map:=Town01`. acb applies and verifies the spec's physics, Autoware
  localizes, takes a goal from RViz and arrives, stopping at CARLA's own cycling lights.
  A hero of another blueprint is refused with both names.
- [ ] **With SSv2**: `just e2e sim:=carla` and `just e2e sim:=isaac` PASS. The CARLA suite
  and the gap set have every difference from K0 explained by K3's physics or K4's clock.
- [ ] **A future simulator**: a fake vehicle side passes `vehicle_check`. A fake world
  server (Python, on the shell) passes conformance. Both are built from the repositories'
  docs alone, as the check that the contracts are enough.
- [ ] `just test` and `just check` pass in every repository.

Isaac's runs need a host that renders Town01: newslab-server139, with its `.env` moved
from `CSB_*` to `SWB_*` first.

Not in this phase: upstreaming the SSv2 patches; a second car; NPC and background-AV
physics from specs; `aib.ros` as a shared library.
