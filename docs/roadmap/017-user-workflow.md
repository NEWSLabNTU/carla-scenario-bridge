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
- [x] csb reads ROS params `carla_host`, `carla_port`, `ssv2_port`, `config_file`,
      `background_avs` (now `agent_relay`, step 6), `reconnect_wait_seconds`; env vars stay as overrides (file < env <
      params). **No ROS client library**: csb has no topics, so it parses `--ros-args`
      (`-p`, `--params-file`) itself (`ros_args.rs`, 4 tests)
- [x] Default config is the installed `share/carla_scenario_bridge/config/bridge_config.yaml`,
      not `./config` relative to the working directory
- [x] `bridge.launch.xml` with `respawn="true"` (play_launch supervises); `just run` becomes
      a thin call (to `csb_launch simulation.launch.xml`: bridge + relay) and the bash
      supervisor loop is removed
- [x] `ros2 run carla_scenario_bridge carla_scenario_bridge --ros-args -p ...` works from any
      directory (the signal tables were generated with it from /tmp)

### 2. Scenario launch and scenario files
- [x] `csb_launch/scenario.launch.xml`: `scenario:=` plus `port`, `global_timeout`; SSv2's
      fixed args inside (renamed from `carla_scenario.launch.xml`, old name kept as an
      include for one phase)
- [x] SSv2 fork (`ac4c9e983`): the runner hands the preprocessor a copy in its output dir,
      so the input file is never touched; the justfile's copy step is removed (and the
      tracked `scenarios/raw/` debris it had produced)
- [x] Scenarios reference the map dir by plain path; the repo's examples use
      `$(env CARLA_MAPS)/<Town>` (the fork implements the `$(env NAME [default])`
      substitution, an upstream TODO); `just scenario` exports `CARLA_MAPS`

### 3. Simulator artifacts out of the shared map dir
- [x] Offline generator: `carla_scenario_bridge --generate-signal-table <map dir>` writes
      `<map dir>/carla/traffic_lights.yaml` (Town01: 36 signals, Town02: 24, 2026-10-08)
- [x] csb checks the table against CARLA at `Initialize` and fails on a mismatch (missing:
      warning), instead of writing it; acb reads it from `carla/` (tests: check matches,
      ignores the map path spelling, detects a change)

### 4. Vehicle-side packages for Autoware users
- [x] `carla_simulator.launch.xml` starts `acb_bridge` itself (`vehicle_name`, CARLA host and
      port, clock); no `launch_vehicle_interface:=false` workaround. Found live: the
      unscoped `autoware.launch.xml` include leaked its own `launch_vehicle_interface`
      (false) and kept the bridge from starting; captured beforehand (acb `c7e0eb5`)
- [x] CARLA overrides switchable: `carla_localization`, `carla_perception`, `carla_system`
      (default true); with one off, the user's `autoware.launch.xml` launches that component
- [x] csb's `ego_av.launch.xml` and `background_av.launch.xml` reduce to includes of it
- [ ] An Autoware started this way attaches to a CARLA vehicle and drives a route set by hand
      (RViz), with no scenario running

### 5. Agent protocol, relay, Autoware agent
- [x] Schema with a version field -- **newline-delimited JSON over plain TCP**, `"v": 1`
      (decision 2026-10-08: stdlib only, no pyzmq/protobuf for agent authors): commands
      `set_goal`, `clear_goal`, `set_speed_limit`, `stop`, `teleported`, optional
      `cooperate`/`cooperate_auto`; state `phase`, `fault`, `turn_indicators`,
      `capabilities`, `detail` (+ `pose`, `rtc_status`); registration by entity name;
      heartbeat 1 s / timeout 3 s. Spec: design doc "The agent protocol (I3)"; code:
      `scenario_agent_relay/protocol.py` (agent copy: acb `acb_pilot/agent_protocol.py`)
- [x] `scenario_agent_relay` package (sim-neutral, no CARLA import): serves the concealer's
      ADAPI subset in the scenario domain only while the agent is up, forwards to the
      registered agent, maps state back; no or silent agent → UNKNOWN states and services
      withdrawn (`INITIALIZING` to the concealer). The concealer's scan / kinematic-state
      waits are met in the relay's own time base, no fork change.
      `launch/relay.launch.xml`
- [x] `acb_agent` = `ros2 run acb_pilot agent` (acb `8805efb`): connects to the relay,
      drives Autoware through its local ADAPI, engages on its own once ready, honours
      `teleported` (fresh scans → initialize → wait for agreement), reconnects when the
      relay restarts (seen live); local goal-file mode when no relay is configured
- [x] Unit tests: relay state mapping (incl. a transcription of the concealer's legacy
      state, and constants checked against the ROS messages), schema round trip, TCP
      server (register / reply / heartbeat timeout / replace / version), agent state
      machine against a fake ADAPI, relay client reconnect -- 62 + 30 pass
- [x] Live (2026-10-08): `town01_engage_state` **passed with SSv2 + relay in domain 9 and
      the ego Autoware in domain 1** (agent there), managed_ego:=true, the ego stack
      untouched: teleported → localized in 3.7 s, concealer "Localization reached the
      initial pose ... after 0 ms", READY → engaged → DRIVING in 0.3 s, ARRIVED 28 s
      later, ARRIVED_GOAL condition met; passed again on an immediate rerun against the
      same stacks. The first attempt failed ("initialize ... state
      is PLANNING"): the relay published on a timer, so the concealer read the state from
      before `clear_route`; the relay now publishes when a state arrives (design doc,
      ordering rule)
- [x] `managed_ego:=false`, the unmanaged domain split and `EGO_MANAGED` retired:
      `scenario.launch.xml` always runs the concealer against the relay; `ego_av` passes
      `relay`/`entity`; the unmanaged scenarios are removed; the justfile has a scenario
      domain (9) and an ego domain (1)
- [x] Fresh-stack startup (found 2026-10-08): Autoware's ADAPI state is latched but only
      published once `/clock` runs, i.e. once a scenario ticks CARLA, so an agent that
      waited for it before reporting ready deadlocked the first scenario. The agent is now
      ready on Autoware's services and reports INITIALIZING until the states arrive;
      commands do not wait for unpublished state. `town01_engage_state` then **passed on a
      freshly started ego stack** (concealer "Localization reached the initial pose ...
      after 0 ms", success)

### 6. Background vehicles driven by the simulator
- [x] Check carla-rust's Traffic Manager bindings (fall back to a server-side alternative if
      absent): present (`Client::instance_tm`, register/unregister, `set_custom_path`,
      `set_desired_speed`, sync mode, seed). Two catches, worked around in csb rather than
      patched: `set_custom_path` takes `AsRef<Location>`, which `Location` does not
      implement (a newtype), and `set_desired_speed` is km/h, not the m/s its doc says.
      **LibCarla ticks a synchronous Traffic Manager inside `world.tick()`** (verified live,
      0.9.16: a registered vehicle drives on world ticks alone), so csb stays the only ticker
- [x] `simulator_autopilot` controller: csb turns physics on, hands the actor to Traffic
      Manager, returns CARLA's pose in `UpdateEntityStatus`; SSv2 behavior `do_nothing`
      (verify the property name). SSv2 sent no controller to the simulator, so the fork adds
      `SpawnVehicleEntityRequest.behavior` and a `SimulatorDrivenVehicleEntity` (do_nothing
      plugin, no behavior update); csb hands over at `npc_logic_started`, reads back pose,
      twist (now entity-frame, for the ego too) and acceleration. Found on the way: starting
      a TM resets all light groups (csb re-applies GREEN + commanded states), and TM's launch
      jolt (15.5 m/s² for a frame) gets a 1 s settle
- [x] Goal-driven: `AcquirePositionAction` → lanelet route planned in csb → TM path;
      `SpeedAction` → `set_desired_speed`; deterministic with a fixed TM seed in sync mode.
      As built: new `UpdateEntityGoal` request (fork); the route is planned on CARLA's lane
      topology, not Lanelet2 (TM drives CARLA's lanes), and TM gets a point past each
      junction plus the goal -- TM alone took a wrong branch on 1 of 4 sparse paths and on a
      dense one; csb stops the vehicle at the goal (TM roams past a path's end). Lane changes,
      relative speeds, trajectories and mid-run controller switches are scenario errors.
      Live (2026-10-08): `town01_simulator_autopilot` (ego + SSv2 NPC + TM NPC) passed both
      runs on the final build, after the fixes above; the TM NPC drove 5 lane segments / 2 junctions to its goal and parked
      0.33 m short of it 617 frames after the hand-over in **both** of the last two runs
      (seed 2017); `town01_engage_state` and `bench/town01_npc_10` still pass (npc_10:
      processing p50 0.60 ms, tick p50 3.75 ms, 10 entities)
- [x] Background AVs move from `bridge_config.yaml` to scenario entities; the `agent`
      controller addresses a registered agent by entity name. As built (design doc,
      "`agent` as built"): the fork treats `agent` like `simulator_autopilot`
      (SimulatorDrivenVehicleEntity, `behavior: "agent"`, goals as `UpdateEntityGoal`); the
      relay accepts **commanders** (`register` with `role: "commander"`, `command_for`,
      `query`; replies carry `registered` and the agent's state; agents unchanged); csb spawns
      the vehicle physics-on with `role_name` = entity name (reaped via a tmpfs ledger, not
      the `csb_entity:` mark), sends `teleported` with the spawn pose, forwards goals from a
      worker thread once NPC logic runs and the agent can take one, `stop` on despawn, and
      reads the pose back. `background_avs` (file key, ROS param, `CSB_BACKGROUND_AVS`) is
      removed and refused with that message; `background_av.launch.xml` / `just bg-av` run
      the agent with `entity:=`. Tests: csb 182 (agent_link: queue gating, retries, errors,
      relay wire format against a fake relay, ledger; config; spawn kinds), relay 84
      (commander query/command/timeout/no agent, silent commander kept, register+query in one
      packet), acb_pilot 30 (protocol copy in sync). Live (2026-10-08): `just two-av`
      (`town01_two_av.xosc`: ego in domain 1, `bg_av_1` driven by its own Autoware in
      domain 2, scenario in 9) **passed twice**; bg_av_1 localized at its spawn pose ~10 s
      after `teleported`, took its goal, arrived and was judged by SSv2 (ReachPosition +
      stopped). First attempt failed on bg_av_1's declared 10 m/s² bound (Autoware's launch
      off the parked hand brake read 13.3 m/s²); it now declares the ego's 15.
      `town01_simulator_autopilot` and `town01_engage_state` still pass. With the
      background stack stopped the scenario fails at once: "no agent is registered with the
      agent relay (tcp://localhost:5560) as 'bg_av_1' (controller agent); start its vehicle
      side with entity:=bg_av_1"

### 7. Documentation
- [x] User guide (`docs/user-guide.md`): install, prepare a map dir and a scenario dir, the
      four commands, both sides on separate hosts or domains, your own Autoware
- [x] README rewritten to the user guide; `ssv2-launch-configuration.md` marked superseded
      for the run workflow (kept as SSv2 parameter background)
- [x] Justfile recipes reduced to the commands in the guide plus developer conveniences:
      `run` = `simulation.launch.xml`, `ego-av`/`bg-av` = the acb profile, `scenario` =
      `scenario.launch.xml` (no copy, no mode switch); health gates and `two-av` remain as
      developer conveniences

### 8. Live verification
- [x] A scenario dir and map dir outside the repo; the four commands from the guide, no
      justfile; `town01_ego_drive` and `town01_engage_state` pass (2026-10-08: map dir and
      scenarios under the session scratchpad, `LogicFile` an absolute path there; plain
      `play_launch launch` of `simulation.launch.xml`, `carla_simulator.launch.xml`,
      `scenario.launch.xml`; both passed)
- [x] Vehicle side in a **different** ROS domain from SSv2, started **after** the scenario
      launch: the scenario waits (`UNAVAILABLE`), then engages and passes. Scenario launched
      00:56:21, vehicle side (domain 1) 40 s later; agent registered, ADAPI offered,
      teleported → IDLE → set_goal → DRIVING → ARRIVED; verdict 00:58:06, success. Needed
      a fork change: the concealer's initial change_to_stop now waits for the service
      within `initialize_duration` instead of a fixed 180 s (SSv2 `efe7f87f8`)
- [x] `ego.currentState` conditions (DRIVING, ARRIVED_GOAL) pass through the relay
      (`town01_engage_state`, every run above)
- [x] A background vehicle on `simulator_autopilot` reaches its goal; one on the default
      controller follows its SSv2 route; both in one scenario with the ego
      (`scenarios/town01_simulator_autopilot.xosc`, step 6)
- [x] RTC through the relay, verified at the interface (2026-10-08): the relay's
      `/api/external/set/rtc_commands` (scenario domain) reached Autoware through the agent
      and returned Autoware's verdict (`cooperate -> FAILED` for an unknown uuid); against a
      fake agent without RTC the same call returned `success=False` with
      `cooperate -> UNSUPPORTED: this autopilot has no RTC`.
      **Not run as a scenario**: stock Autoware 1.5 sets `enable_rtc: false` for every
      behavior module (autoware_launch planning config), so cooperation requests are never
      raised and an RTC scenario has nothing to approve. Running one needs a vehicle side
      whose Autoware enables RTC for the module the scenario commands -- the user's planning
      configuration, outside this project

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
