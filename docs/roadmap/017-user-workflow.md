# Phase 017: User Workflow and the Vehicle Agent

Give users a ROS-native workflow -- a scenario dir, a map dir, and `play_launch launch` /
`ros2 run` -- in place of this repo's justfile, and separate the simulation side from the
vehicle side so that neither needs the other's ROS domain or start order, and so that CARLA
and Autoware can each be replaced by another implementation.

**Source**: UX campaign, user direction 2026-10-08: "make components self contained and the
user mostly use ROS commands"; "the user prepare a scenario dir"; Autoware gets acb's sensor
kit and vehicle interface; "SSv2 and Autoware should not be tightly coupled"; "the vehicle may
not be backed by Autoware"; relay method chosen; background vehicles offloaded to the
simulator per scenario.
**Design**: [user-workflow](../design/user-workflow.md) (interfaces I1–I5, agent protocol,
swap audit).
**Depends on**: [015](015-carla-as-time-and-signal-source.md) (CARLA owns time and signals),
[016](016-robustness-and-performance.md) (failures end scenarios; csb supervised).
**Relates to**: [012](012-ssv2-unmanaged-autoware.md), [013](013-forked-unmanaged-ego.md)
(the managed/unmanaged split this phase retires), [005](005-hardening-awf-contribution.md)
(setup guide and contribution prerequisites).

## Problem

- **Users drive the justfile.** Every run is `just carla-start`, `just run`, `just ego-av`,
  `just scenario`; reproducing a recipe by hand means three overlays, a DDS file, a domain,
  and a dozen play_launch flags (survey 2026-10-08).
- **csb is not a ROS program.** It reads env vars and `./config` relative to its working
  directory; its supervisor is a bash loop in the justfile.
- **Scenarios reference this repo.** `LogicFile` is `$(find-pkg-share csb_launch)/data/...`;
  the map is named twice (the `.xosc` and Autoware's `map_path`) and must agree.
- **SSv2 rewrites the user's scenario file** (preprocessor), so the justfile copies it first.
- **csb writes into Autoware's map dir** (`traffic_lights.resolved.yaml` at every
  `Initialize`).
- **The Autoware wrapper** (`acb_launch/carla_simulator.launch.xml`) always launches
  localization, perception and system from stock `autoware_launch`, discarding a user's own;
  acb's vehicle interface is disabled and `acb_bridge` launched separately by csb's files.
- **SSv2 and Autoware share a ROS domain and a start order**, because the concealer calls
  Autoware's ADAPI directly; the unmanaged mode that avoids it loses every Autoware-state
  condition.
- **Background vehicles** are either SSv2-puppeteered or a full Autoware stack each; nothing
  in between, and they are declared in `bridge_config.yaml`, not in the scenario.

## Steps

### 0. Docs
- [x] Design doc `user-workflow.md`: interfaces, agent protocol, swap audit
- [x] This phase doc

### 1. csb as a ROS node
- [ ] csb reads ROS params `carla_host`, `carla_port`, `ssv2_port`, `config_file` (and the
      background-AV selection), via the Rust ROS client acb uses; env vars stay as overrides
- [ ] Default config is the installed `share/carla_scenario_bridge/config/bridge_config.yaml`,
      not `./config` relative to the working directory
- [ ] `bridge.launch.xml` with `respawn="true"` (play_launch supervises); `just run` becomes
      a thin call to it and the bash supervisor loop is removed
- [ ] `ros2 run carla_scenario_bridge carla_scenario_bridge --ros-args -p ...` works from any
      directory

### 2. Scenario launch and scenario files
- [ ] `csb_launch/scenario.launch.xml`: `scenario:=` plus `port`, `global_timeout`; SSv2's
      fixed args inside (renamed from `carla_scenario.launch.xml`, old name kept as an
      include for one phase)
- [ ] SSv2 fork: the preprocessor writes its re-serialized copy to the output dir and never
      touches the input file; the justfile's copy step is removed
- [ ] Scenarios reference the map dir by plain path; the repo's examples use a path the
      justfile passes in (no `find-pkg-share csb_launch` in any `.xosc`)

### 3. Simulator artifacts out of the shared map dir
- [ ] Offline generator for `<map dir>/carla/traffic_lights.yaml` (csb's resolver as a tool)
- [ ] csb checks the table against CARLA at `Initialize` and fails on a mismatch, instead of
      writing it; acb reads it from `carla/`

### 4. Vehicle-side packages for Autoware users
- [ ] `carla_simulator.launch.xml` starts `acb_bridge` itself (`vehicle_name`, CARLA host and
      port, clock); no `launch_vehicle_interface:=false` workaround
- [ ] CARLA overrides switchable: `carla_localization`, `carla_perception`, `carla_system`
      (default true); with one off, the user's `autoware.launch.xml` launches that component
- [ ] csb's `ego_av.launch.xml` and `background_av.launch.xml` reduce to includes of it
- [ ] An Autoware started this way attaches to a CARLA vehicle and drives a route set by hand
      (RViz), with no scenario running

### 5. Agent protocol, relay, Autoware agent
- [ ] Protobuf schema with a version field: commands `set_goal`, `clear_goal`,
      `set_speed_limit`, `stop`, `teleported`, optional `cooperate`/`cooperate_auto`; state
      `phase`, `fault`, `turn_indicators`, `capabilities`, `detail`; registration by entity
      name; heartbeat
- [ ] `scenario_agent_relay` package (sim-neutral): serves the concealer's ADAPI subset in the
      scenario domain, forwards to the registered agent, maps state back (design: relay
      mapping); unknown or silent agent → `UNAVAILABLE`
- [ ] `acb_agent` from `acb_pilot/auto_drive.py`: connects to the relay, drives Autoware
      through its local ADAPI, engages on its own once ready, honours `teleported`; local
      goal file when no relay is configured
- [ ] Unit tests: relay state mapping, schema round trip, agent state machine against a
      fake ADAPI
- [ ] `managed_ego:=false`, the unmanaged domain split and `EGO_MANAGED` retired

### 6. Background vehicles driven by the simulator
- [ ] Check carla-rust's Traffic Manager bindings (fall back to a server-side alternative if
      absent)
- [ ] `simulator_autopilot` controller: csb turns physics on, hands the actor to Traffic
      Manager, returns CARLA's pose in `UpdateEntityStatus`; SSv2 behavior `do_nothing`
      (verify the property name)
- [ ] Goal-driven: `AcquirePositionAction` → lanelet route planned in csb → TM path;
      `SpeedAction` → `set_desired_speed`; deterministic with a fixed TM seed in sync mode
- [ ] Background AVs move from `bridge_config.yaml` to scenario entities; the `agent`
      controller addresses a registered agent by entity name

### 7. Documentation
- [ ] User guide: install, prepare a map dir and a scenario dir, the four commands, both
      sides on separate hosts or domains
- [ ] README rewritten to the user guide; `ssv2-launch-configuration.md` updated or retired
- [ ] Justfile recipes reduced to the commands in the guide plus developer conveniences

### 8. Live verification
- [ ] A scenario dir and map dir outside the repo; the four commands from the guide, no
      justfile; `town01_ego_drive` and `town01_engage_state` pass
- [ ] Vehicle side in a **different** ROS domain from SSv2, started **after** the scenario
      launch: the scenario waits (`UNAVAILABLE`), then engages and passes
- [ ] `ego.currentState` conditions (DRIVING, ARRIVED_GOAL) pass through the relay
- [ ] A background vehicle on `simulator_autopilot` reaches its goal; one on the default
      controller follows its SSv2 route; both in one scenario with the ego
- [ ] RTC scenario on Autoware passes; on an agent without RTC it fails with `UNSUPPORTED`

## Acceptance

- A user runs a scenario with only `play_launch launch` / `ros2 run` and the vendor CARLA
  binary, from a scenario dir and a map dir outside this repo
- SSv2 and Autoware run in different ROS domains, started in either order
- Every scenario feature in the design's interface table works through the relay, with
  Autoware-specific ones degrading explicitly on another autopilot
- No component writes into another's files: the user's `.xosc` and the map dir are read-only
  at run time
- Swap audit holds in the code: csb and acb_bridge are the only CARLA-specific components on
  the command path; the relay and acb_agent have no CARLA dependency
- Existing scenarios (014–016) still pass
