# Phase 048: The Vehicle-Side Profile Leaves acb

Move the simulator-free half of the ego's Autoware side out of acb into this repository:
Autoware's launch profile, tuned for a simulated car, and the vehicle agent. Then acb and
aib each include the profile with their own vehicle bridge, vehicle model and sensor kit,
and aib no longer depends on acb. Write down the contract a vehicle bridge keeps with
Autoware and with the bridge (I5). Design:
[design/simulator-interface-split.md](../design/simulator-interface-split.md), "The vehicle
side" and "How to get there", step 7.

**Status**: draft, 2026-10-23. Waiting on the decisions below.

**Source**: user direction 2026-10-23 ("Draft phase 48"). This is the next step of the
design after 046 ("Not in this phase: ... moving the Autoware launch profile out of acb
(`scenario_vehicle_launch`) ... each is its own phase").
**Depends on**: [046](046-unified-world-interface.md), parts A and B: the world servers, the
`swb_*` names and `RemoteBackend` as the only backend. Its Isaac runs that are still owed do
not block this phase. They do share its Isaac exit run (below).
**Feeds**: the repository split (design step 8), where the profile and the agent go into
the adapter repository with the rest of the `swb_*` packages. Also the next simulator port,
whose vehicle bridge then includes one package instead of acb.

## Where it starts (2026-10-23)

| | today |
|---|---|
| the profile | `acb_launch/launch/carla_simulator.launch.xml`, 494 lines: Autoware 1.5.0 with acb's localization, perception, planning, control and system settings (`config/`, 152 KB), three component copies made by `scripts/sync_*_component.sh`, `planning_common_param`, `setup_autoware_data`, `check_data_path.launch.py`; it starts `acb_bridge` itself when `launch_vehicle_interface` is true |
| CARLA's use | `just ego-av` and `bg-av` (`ego_av.launch.xml`, `background_av.launch.xml`) include it; `carla_up.sh` finds the ego stack by `acb_launch carla_simulator.launch.xml` in its command line |
| Isaac's use | aib's `isaac_simulator.launch.xml` includes it with `simulator_type:=isaac` and `launch_vehicle_interface:=false`; `aib_launch` has `exec_depend` on `acb_launch` and `acb_pilot`; aib's justfile sources an acb install for them |
| the agent | `acb_pilot agent` (node `acb_agent`, 145 lines): ROS parameters (`relay`, `entity`, `goal_poses_file`, `timeout`, `agent_config`) wrapped around `swb_agent.adapters.autoware.AutowareAgent`, plus a local mode (no relay: drive to a goal file's pose, exit 0 on arrival). `swb_agent` already has `autoware_agent` (`autoware_ros.py`), the same agent behind command-line flags with no local mode |
| CARLA-named arguments in the profile | `carla_localization`, `carla_perception`, `carla_system`, `carla_control`, `carla_planning`, `carla_*_path`, `simulator_type`; the diagnostic graph is `autoware-carla.yaml` |
| acb's own tests on the profile | `acb_bridge/tests/launch_profile.rs` reads `../acb_launch` |
| `/clock` | acb: CARLA's `elapsed_seconds` plus an episode epoch, which `carla_world` keeps (`episode_clock`, `clock_store`) and leaves in CARLA for a late-starting acb (`epoch_marker`); the Isaac world server: its own time, monotonic over its lifetime |

## Design (as proposed)

### Packages

- **`swb_vehicle_launch`** (ament_cmake, in `src/`). It holds the profile as
  `launch/autoware.launch.xml`: `carla_simulator.launch.xml` without `acb_bridge` and the
  arguments only `acb_bridge` reads (`carla_host`, `carla_port`, `vehicle_name`,
  `publish_clock`, the V2X signal table, `map_origin_path`, pedal maps, steering shaping,
  ground truth, control trace, release endpoints, `allow_manual`, `honor_emergency_cmd`,
  `control_cmd_timeout_s`). With it go `config/`, `launch/components/`, the sync scripts with
  their `--check`, `planning_common_param`, `setup_autoware_data` and
  `check_data_path.launch.py`. The profile has no vehicle interface. `vehicle_model`
  and `sensor_model` have no defaults: the including launch names them.
- **The agent** lives in `swb_agent`. `autoware_agent` learns ROS parameters (when it is run
  as a node from a launch) and local mode (`goal_poses_file`, `timeout`) from
  `acb_pilot/agent.py`. The profile starts it as node `vehicle_agent`. `acb_pilot` keeps
  `auto_drive` and `capture_poses`, acb's standalone demo tools. Its `agent` entry stays one
  release as a shim that runs `swb_agent`'s and warns.
- **acb**: `acb_launch/launch/carla_simulator.launch.xml` becomes the profile plus
  `acb_bridge`. It has the same name, the same arguments (the old `carla_*` names are
  accepted and passed on to the new ones) and the same defaults. So `just ego-av`,
  `carla_up.sh`, acb's demo and every command line in the docs are unchanged. `acb_launch`
  keeps only that file and the `acb_bridge` wiring. The profile checks in
  `launch_profile.rs` move to a test in `swb_vehicle_launch`. acb keeps a test that its
  wrapper resolves.
- **aib**: `isaac_simulator.launch.xml` includes `swb_vehicle_launch` directly, with aib's
  models. `aib_launch` depends on `swb_vehicle_launch` and `swb_agent`, not on `acb_*`.
  aib's justfile sources a swb install instead of acb's.

### Argument names

The profile's arguments say what they select, not which simulator asked for it:
`sim_localization`, `sim_perception`, `sim_system`, `sim_control`, `sim_planning`, and the
`*_path` arguments likewise. `simulator_type` goes away (nothing reads it since the CARLA
interface include was commented out). The diagnostic graph becomes
`autoware-simulation.yaml`. acb's wrapper maps the old names, so a `carla_control:=false`
on a command line still works.

### The I5 contract

`docs/design/vehicle-side-contract.md`, written from what acb and aib do today:

- **Topics and messages** the vehicle bridge publishes: `/sensing/*` per sensor kit,
  `/vehicle/status/*`, `/clock`, the signal V2X table (`traffic_lights.yaml`). The topics it
  takes: `/control/command/*`.
- **Frames**: `map`, `base_link`, the sensor kit's frames from the vehicle's URDF.
- **Timing**: every stamp in one domain comes from the one `/clock`. Sensor samples are
  stamped with the frame they were taken in.
- **The agent**: it registers with the relay as its entity and is started by the profile.
- **The clock rule as the target**: one `/clock` per domain, carrying the world server's
  time, monotonic by construction. **Today's CARLA shape**: an epoch kept by `carla_world`
  and acb, and the marker actor. It is written down as the CARLA exception, and changing
  it is not in this phase (decision 3).

## Decisions (for the user)

1. **The package name.** `swb_vehicle_launch` matches 046's `swb_*` prefix and `swb_launch`
   beside it. The design said `scenario_vehicle_launch` before the rename picked `swb`.
   [proposed: `swb_vehicle_launch`]
2. **acb standalone.** After the move, acb on its own (its demo, its release tarball, its
   own workspace) needs `swb_vehicle_launch` and `swb_agent`. It gets them the way aib gets
   acb's packages today: by sourcing a swb install, named in acb's justfile and README.
   Alternatives: acb's `just package` bundles the two packages from the superproject; or
   wait for step 8's adapter repository and pin it as a submodule then. [proposed: source a
   swb install now; acb's tarball bundles the two packages built from the pinned swb]
3. **The CARLA clock.** Moving acb onto the world server's time, so that `carla_world` drops
   the epoch marker, changes acb's `clock.rs` and how a CARLA time reaches acb. It touches
   timing that 015 and 016 took weeks to settle. [proposed: write it in the contract as the
   target, do it in its own phase with its own exit runs; not here]
4. **`aib.ros` and aib's spec generator** (`aib.gen`, `aib.spec`): move them with this phase,
   or wait for the first Python world server that asks? [proposed: wait, as the design
   recommends; listed in the contract as candidates]

## Steps

- [ ] **S1. Baseline.** Before anything moves, save `play_launch dump` of the resolved
  `SystemModel` for four launches: `just ego-av`'s, `bg-av`'s, aib's `isaac_simulator` and
  acb's demo, with the arguments the recipes pass. Also record a `just e2e sim:=carla` and
  `just test` result.
- [ ] **S2. `swb_vehicle_launch`.** Create the package. Move the profile, `config/`,
  components, scripts and their tests from acb with `git mv`, so blame follows. This is a
  copy at first: acb still has its own, so S2 lands without changing a running stack.
- [ ] **S3. The agent.** Give `swb_agent`'s `autoware_agent` ROS parameters and local mode,
  with tests on the fake Autoware (`test_autoware_agent.py`). Then make `acb_pilot agent` a
  shim.
- [ ] **S4. acb.** `carla_simulator.launch.xml` becomes the wrapper; delete acb's copies;
  `launch_profile.rs` shrinks to the wrapper. Do this on acb `main`, push, and only then
  move the pin here.
- [ ] **S5. aib.** `isaac_simulator.launch.xml` includes the profile; package and justfile
  dependencies follow. Do this on aib `main`, push, and only then move the pin here.
- [ ] **S6. This repo.** `swb_launch`'s dependencies; `ego_av.launch.xml`, `background_av`,
  `run_suite`'s message; CLAUDE.md (the agent's log is now `node/vehicle_agent/err`), the
  user guide, and `docs/design/ssv2-launch-configuration.md`. Then build both workspaces in order (acb's first,
  then this one).
- [ ] **S7. The contract.** `docs/design/vehicle-side-contract.md`, and links to it from the
  architecture doc, acb's and aib's READMEs.
- [ ] **S8. Exit runs** (below).

## Acceptance

No behaviour change, checked where it can be exact:

- [ ] **Resolved launches unchanged.** For each of S1's four launches, `play_launch dump`
  after the move has the same node set, executables, remappings and parameter values as
  before. The only differences allowed are install paths (`acb_launch` to
  `swb_vehicle_launch`) and the agent's package and node name. A script diffs the two
  models and lists every other difference. This is the main check: it covers Isaac's
  launch on this host too, where nothing needs to render.
- [ ] `just test` and `just check` pass; acb's and aib's own tests pass in their workspaces.
- [ ] `just e2e sim:=carla` PASS on the new build. The old command lines still work:
  `just ego-av carla_control:=false` resolves as before.
- [ ] aib builds and resolves with no acb package in the environment.
- [ ] Isaac: `just e2e sim:=isaac` PASS, on a host that renders Town01 (server139). It is
  shared with 046's runs that are still owed: one run settles both.

Not in this phase: acb's clock (decision 3), `aib.ros` and the generator (decision 4), the
repository split (step 8).
