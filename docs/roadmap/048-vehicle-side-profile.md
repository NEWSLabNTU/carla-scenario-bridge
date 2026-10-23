# Phase 048: The Vehicle Layer and the Autoware Profile

Make the vehicle layer, the part with no SSv2 and no simulator in it, and move into it the
launch profile that runs Autoware on a simulated car. After this phase, acb and aib include
the profile with their own vehicle interface. Neither depends on the other or on swb.
Autoware still drives on CARLA and on Isaac Sim without SSv2, checked. The contract a
vehicle bridge keeps (I5) is written down, with a first `vehicle_check`. Design:
[design/vehicle-side.md](../design/vehicle-side.md). The spec as the single source of the
car is [049](049-vehicle-spec.md).

**Status**: draft, 2026-10-23, revised the same day. Waiting on the decisions below and on
the design's V1.

**Source**: user direction 2026-10-23:

- "Draft phase 48";
- share packages across simulators only if future simulators are guaranteed to work with
  them, otherwise one package per simulator;
- "Autoware can connect to CARLA and Isaac Sim without SSv2".

This is design step 7 of [simulator-interface-split.md](../design/simulator-interface-split.md),
with its target moved from swb to the vehicle layer.
**Depends on**: [046](046-unified-world-interface.md), parts A and B. Its Isaac runs that
are still owed do not block this phase. They share this phase's Isaac exit run.
**Feeds**: [049](049-vehicle-spec.md); the repository split (design step 8).

## Where it starts (2026-10-23)

| | today |
|---|---|
| the profile | `acb_launch/launch/carla_simulator.launch.xml`, 494 lines: Autoware 1.5.0 with acb's localization, perception, planning, control and system settings (`config/`, 152 KB), three component copies made by `scripts/sync_*_component.sh`, `planning_common_param`, `setup_autoware_data`, `check_data_path.launch.py`. It starts `acb_bridge` (`launch_vehicle_interface`) and the agent (`relay` or `goal_poses_file`) itself |
| CARLA, with SSv2 | `just ego-av` and `bg-av` (`ego_av.launch.xml`, `background_av.launch.xml`) include it; `carla_up.sh` finds the ego stack by `acb_launch carla_simulator.launch.xml` in its command line |
| CARLA, without SSv2 | acb `just sim` / `scenario` (`demo_scenario.py` spawns the hero), the same launch, `auto_drive` |
| Isaac, with SSv2 | `just ego-isaac` runs aib's `isaac_simulator.launch.xml`, which includes acb's with `launch_vehicle_interface:=false`; `aib_launch` has `exec_depend` on `acb_launch` and `acb_pilot` |
| Isaac, without SSv2 | aib `just isaac-ego` (`aib.runner`) and `just autoware` |
| the agent | `acb_pilot agent` (node `acb_agent`): ROS parameters around `swb_agent`'s `AutowareAgent`, plus a local mode (drive to a goal file, exit 0 on arrival). `swb_agent` already has `autoware_agent`, the same agent behind command-line flags |
| CARLA-named arguments | `carla_localization`, `carla_perception`, `carla_system`, `carla_control`, `carla_planning`, `carla_*_path`; `simulator_type`, read by nothing (its include is commented out); diagnostic graph `autoware-carla.yaml` |
| acb's tests on the profile | `acb_bridge/tests/launch_profile.rs` reads `../acb_launch` |

## Design (as proposed)

- **`autoware_sim_profile`** (ament_cmake) in the vehicle layer's repository (design V1).
  It holds `launch/autoware.launch.xml`: `carla_simulator.launch.xml` without `acb_bridge`,
  without the arguments only `acb_bridge` reads, and without the agent. With it go
  `config/`, `launch/components/`, the sync scripts with `--check`,
  `planning_common_param`, `setup_autoware_data` and `check_data_path.launch.py`.
  - It starts no vehicle interface (`launch_vehicle_interface:=false` to Autoware).
  - It needs no relay.
  - `vehicle_model` and `sensor_model` have no defaults.
  - An `extra_launch` hook stays.
- **Argument names** say what they select: `sim_localization`, `sim_perception`,
  `sim_system`, `sim_control`, `sim_planning`, and the `*_path` arguments likewise.
  `simulator_type` goes. The diagnostic graph becomes `autoware-simulation.yaml`.
- **The agent leaves the profile.** It needs swb's relay protocol, so it belongs to the
  SSv2 side.
  - `swb_agent`'s `autoware_agent` learns ROS parameters and local mode from
    `acb_pilot/agent.py`. swb's ego launches start it as node `vehicle_agent`.
  - acb's standalone path keeps `auto_drive`.
  - `acb_pilot agent` stays one release as a shim.
- **acb**: `carla_simulator.launch.xml` becomes the profile plus `acb_bridge`. It keeps the
  same name, arguments (the old `carla_*` names mapped to the new ones) and defaults, so
  `carla_up.sh`, acb's demo and the documented command lines still work.
  - `relay` and `goal_poses_file` stay accepted for one release: they start the shim
    agent, with a warning that names swb's launch.
  - `launch_profile.rs` shrinks to the wrapper. The profile's checks move to a test in
    `autoware_sim_profile`.
- **aib**: `isaac_simulator.launch.xml` includes the profile. `aib_launch` depends on
  `autoware_sim_profile`, not on `acb_*`. aib's justfile fetches the vehicle layer instead
  of sourcing acb.
- **swb**:
  - `ego_av.launch.xml` and `background_av.launch.xml` start the agent beside acb's
    wrapper.
  - A new `ego_isaac.launch.xml` does the same around aib's, and `just ego-isaac` runs it.
    `isaac_up.sh` matches it.
- **The I5 contract**, written into the vehicle layer's docs (design, "The I5 contract").
- **`vehicle_check`**, first version: topics, types, rates, frames, stamps and
  `/robot_description`, against a running vehicle side, with no SSv2. Its dynamics run
  comes in 049, when there is a spec to compare against.

## Decisions (for the user)

1. **The repository and package names**: design V1, and `autoware_sim_profile` for the
   profile. [proposed: as written]
2. **acb on its own** fetches the vehicle layer at a pinned commit in its `just setup`, and
   its release tarball includes the layer's packages. [proposed: as written]
3. **The CARLA clock**: moving acb onto the world server's time, so `carla_world` can drop
   the epoch marker, changes timing that 015 and 016 settled. [proposed: the contract
   names it as the target and the exception; its own phase]
4. **`aib.ros` and aib's generator** move with 049's spec, since the spec's generator is
   what moves. `aib.ros` waits for the first Python world server that asks for it.
   [proposed: as written]

## Steps

- [ ] **S1. Baseline.** Save `play_launch dump` of the resolved `SystemModel` for:
  - `just ego-av`, `bg-av` and `ego-isaac`, with the arguments the recipes pass;
  - acb's `just sim`, standalone;
  - aib's `just autoware`, standalone.

  Record `just test` and `just e2e sim:=carla`.
- [ ] **S2. The repository.** Create the vehicle layer's repository with the pin and fetch
  rules (design, "Layers"), and add it to swb as a submodule.
- [ ] **S3. The profile.** Move it into `autoware_sim_profile` from acb with history
  (`git filter-repo` of the paths, then a merge into the new repository's first commit).
  Rename the arguments.
- [ ] **S4. The agent.** `autoware_agent` gets ROS parameters and local mode, with tests on
  the fake Autoware. `acb_pilot agent` becomes a shim.
- [ ] **S5. acb.** Make the wrapper, delete acb's copies, add the fetch to `just setup` and
  `package`. Push on acb `main` first, then move the pin here.
- [ ] **S6. aib.** Include the profile, change the dependencies and the fetch. Push on aib
  `main` first, then move the pin.
- [ ] **S7. swb.** Agent nodes in the ego launches, `ego_isaac.launch.xml`, `isaac_up.sh`,
  `run_suite`'s message, CLAUDE.md (the agent's log is now `node/vehicle_agent/err`), the
  user guide, `docs/design/ssv2-launch-configuration.md`. Build in order: the vehicle
  layer, acb, then this repository.
- [ ] **S8. The contract and `vehicle_check`** (first version).
- [ ] **S9. Exit runs** (below).

## Acceptance

No behaviour change. These are checked exactly where that is possible.

- [ ] **Resolved launches unchanged**: each of S1's five dumps, before against after. The
  same node set, executables, remappings and parameter values. The only allowed
  differences are install paths, the renamed arguments' own names, and the agent's package
  and node name. A script lists every other difference. This also covers Isaac's launches
  on this host, where nothing has to render.
- [ ] `just test` and `just check` pass, and so do acb's, aib's and the vehicle layer's own
  tests in their own workspaces.
- [ ] **Without SSv2, CARLA**: from acb's own workspace, with no swb install sourced, `just
  sim` and `auto_drive` drive Town01's demo route to arrival, and `vehicle_check` passes.
- [ ] **Without SSv2, Isaac**: the same from aib's workspace, with `just isaac-ego` and
  `just autoware`. Needs a host that renders Town01 (server139).
- [ ] **With SSv2**: `just e2e sim:=carla` PASS, `just ego-av carla_control:=false`
  resolves as before, and `just e2e sim:=isaac` PASS on server139, which settles 046's
  owed run too.
- [ ] aib builds and resolves with no acb package in its environment.

Not in this phase: the spec as the single source and CARLA on the spec's physics (049),
acb's clock (decision 3), the repository split of the world servers (design step 8).
