# User Workflow and the Vehicle Agent

How a user sets up and runs a scenario, how the scenario reaches the vehicles in it, and
which interfaces let the simulator and the autopilot be swapped.
Status: **design, decisions taken 2026-10-08** (UX campaign). Roadmap: 017.

## Goals

1. **Users run ROS tools.** `ros2 run`, and `play_launch launch` in place of `ros2 launch`.
   The justfile stays as the developers' shortcut layer and calls exactly the commands below.
2. **The user brings a scenario dir and a map dir**, passed by path. Nothing in a scenario
   references this repo's tree.
3. **Two self-contained sides** that share the simulator and the map dir and nothing else --
   no ROS domain, no start order:
   - *Simulation side*: simulator, its SSv2 adapter (csb for CARLA), SSv2. Builds the world,
     runs NPCs and signals, owns time, judges the scenario.
   - *Vehicle side*, one per autonomous vehicle: an autopilot (the user's Autoware, or
     anything else), its agent, and the simulator's vehicle bridge (acb for CARLA).
4. **The scenario observes and commands a vehicle without knowing what drives it.** A
   command is a goal (plus a speed limit); the autopilot decides how to get there.
5. **Swappable**: CARLA can be replaced by another simulator, and Autoware by another
   autopilot, each by replacing only its own adapters (see "Swap audit").

## How SSv2 talks to Autoware today

All of it goes through the concealer (`external/concealer/src/field_operator_application.cpp`),
built into the ego entity, speaking Autoware's ADAPI **over ROS in SSv2's own domain** -- the
only reason SSv2 and Autoware must share a domain. Our fork runs it only with
`managed_ego:=true`; unmanaged, every call is inert (`requireManaged`).

| Scenario feature | Concealer call | Autoware interface |
|---|---|---|
| Ego initial pose (`TeleportAction` at init) | `initialize()` | `/api/localization/initialize`, waits on `/api/localization/initialization_state` |
| Ego goal (`AcquirePositionAction`, `AssignRouteAction`) | `plan()`, then `enableAutowareControl()` + `engage()` | `/api/routing/set_route_points` (`set_route`), `/api/routing/clear_route`, `/api/operation_mode/enable_autoware_control`, `/api/external/set/engage` |
| Ego speed limit (controller `maxSpeed`, `SpeedAction` on ego) | `setVelocityLimit()` | `/api/autoware/set/velocity_limit` |
| Stop between scenarios | `change_to_stop` | `/api/operation_mode/change_to_stop` |
| Engage gating | `engageable()`, `engaged()` | derived state (below) |
| `ego.currentState` | `getLegacyAutowareState()` | `/api/routing/state`, `/api/operation_mode/state`, `/api/localization/initialization_state` |
| `ego.currentMinimumRiskManeuverState.*`, `currentEmergencyState` | MRM state | `/api/fail_safe/mrm_state`, `/api/external/get/emergency` |
| `ego.currentTurnIndicatorsState` | last command | `/control/command/turn_indicators_cmd` |
| RTC (`CustomCommandAction RequestToCooperateCommandAction`) | `sendCooperateCommand()`, `requestAutoModeForCooperation()` | `/api/external/set/rtc_commands`, `/api/external/set/rtc_auto_mode`, `/api/external/get/rtc_status` |
| Observed only | | `/localization/kinematic_state`, `/control/command/control_cmd`, `.../path_with_lane_id`, `/localization/util/downsample/pointcloud` |

The ego's pose each frame and its dimensions do not involve Autoware: the simulator adapter
reports the pose, SSv2 reads dimensions from the vehicle description package. SSv2 refuses
lane changes and relative speed changes on an ego by design ("Ego cars make autonomous
decisions about everything but their destination"). The scenario already treats the ego as
**goal-driven**; only the transport has to change.

**RTC** (Request To Cooperate) is Autoware's operator-approval mechanism: behavior modules
(lane change, avoidance, intersection, crosswalk, blind spot) can ask an operator before
acting, and the scenario plays the operator. Autoware-specific; carried as an optional agent
command that other autopilots reject.

## Architecture

```
 scenario domain                                        vehicle domain (per AV)
┌────────────────────────────────────────┐            ┌───────────────────────────────┐
│ SSv2 interpreter + concealer           │            │ autopilot (Autoware, or other)│
│     │ ADAPI subset (ROS)               │            │     ▲ its native interface    │
│     ▼                                  │   agent    │     │                         │
│ agent relay  ◀─────────────────────────┼─protocol───┼─ agent (Autoware: acb_agent)  │
│ (sim-neutral)                          │ (TCP+JSON) │                               │
│                                        │            │ vehicle bridge (CARLA: acb)   │
│ SSv2 ──ZMQ :5555 (SSv2 sim protocol)──▶│ simulator  │  sensors, vehicle interface,  │
│                                        │ adapter    │  /clock, traffic signals      │
│                                        │ (CARLA:csb)│                               │
└────────────────────────────────────┼───┘            └──────────────┼────────────────┘
                                     ▼                               ▼
                               simulator (CARLA) ◀── world, time, actors, control
```

Five interfaces, each with one owner:

| # | Interface | Between | Defined by | Simulator-specific? | Autopilot-specific? |
|---|---|---|---|---|---|
| I1 | SSv2 simulation protocol (ZMQ + protobuf) | SSv2 ↔ simulator adapter | SSv2 (upstream) | no -- AWSIM implements it too | no |
| I2 | ADAPI subset (ROS, scenario domain) | concealer ↔ agent relay | Autoware ADAPI, as SSv2 uses it | no | Autoware-shaped, but only inside the scenario domain |
| I3 | **Agent protocol** (TCP, newline-delimited JSON) | agent relay ↔ agent | this design | no | no |
| I4 | Autopilot native interface | agent ↔ autopilot | the autopilot | no | yes (Autoware: ADAPI, local domain) |
| I5 | Vehicle I/O (sensors, vehicle interface, `/clock`, signals) | vehicle bridge ↔ autopilot | the autopilot's message set | yes (bridge talks to the simulator) | yes (publishes the autopilot's messages) |

Private, not part of any contract: csb ↔ CARLA, acb ↔ CARLA, and the csb ↔ acb sensor-release
channel (CARLA actor ids).

## The agent protocol (I3)

Implemented in `scenario_agent_relay/protocol.py` (relay) and its copy
`acb_pilot/agent_protocol.py` (agent); a test fails if the copies drift.

- **Transport**: plain TCP, **newline-delimited JSON** (one UTF-8 object per line), standard
  library only on both ends -- no ZMQ or protobuf dependency for an agent author (decision
  2026-10-08; supersedes the ZMQ + protobuf draft). Every message carries `"v": 1` and a
  `"type"`; a receiver answers any other version with `error` and closes. JSON has no
  NaN/infinity: a non-finite number travels as `null`.
- The relay listens on one port (`agent_port`, default 5560); agents connect and
  **register by entity name** -- the SSv2 entity name the scenario uses (`ego`), never a
  simulator id. The relay needs no address list, and an agent can come up before or after
  the scenario. A newer registration for the same entity replaces the older connection (a
  restarted agent does not wait out its predecessor's timeout).

**Messages**

| type | direction | fields |
|---|---|---|
| `register` | agent → relay, first line | `entity`, `agent` (name, informational), `capabilities` (command names) |
| `registered` | relay → agent | `entity` |
| `error` | either; sender then closes | `reason` |
| `command` | relay → agent | `id` (int, unique per connection), `command`, `args` (object) |
| `reply` | agent → relay | `id` (of the command), `status` ∈ {`OK`, `FAILED`, `UNSUPPORTED`}, `message` |
| `state` | agent → relay | below |
| `heartbeat` | either | -- |

**Commands** (`args`; poses are `{x, y, z, qx, qy, qz, qw}` in the map frame):

| command | args | meaning |
|---|---|---|
| `set_goal` | `goal`, `waypoints[]`, `allow_goal_modification`, optional `segments[]` (`{id, type, alternatives[]}`: lanelet ids, when the scenario routes by lane) | drive there: route, enable control, engage on its own once ready |
| `clear_goal` | -- | drop the goal (idempotent) |
| `set_speed_limit` | `mps` (`null` = no limit) | |
| `stop` | -- | stop autonomous driving; hold until the next `set_goal` |
| `teleported` | `pose` | an event, not an instruction: the vehicle was placed at `pose`; an agent whose autopilot localizes uses it to re-initialize |
| `cooperate` (optional) | `module` (name, e.g. `INTERSECTION`), `command` ∈ {`ACTIVATE`, `DEACTIVATE`}, `uuid` (hex, from `rtc_status`) | RTC |
| `cooperate_auto` (optional) | `module`, `enable` | RTC auto mode |

An agent answers a command it does not offer with `UNSUPPORTED`; the relay turns that (and
`FAILED`, and no reply in time) into the concealer's service failure, so a scenario that
needs RTC fails clearly on an autopilot without it. **Ordering rule**: before replying, the
agent pushes a `state` that shows whatever the command changed (`teleported` →
`INITIALIZING`, `clear_goal` → `IDLE`, `set_goal` → `PLANNING`), and the relay publishes a
state the moment it arrives. The concealer's next queued task reads the topics as soon as
the service returns; with the relay publishing on a timer instead, `initialize()` read
`PLANNING` from before a `clear_route` and aborted (seen live, 2026-10-08). A command may be
repeated (the concealer retries a slow answer); the agent treats a repeat of an in-progress
`teleported` or `set_goal` with the same pose as already done.

**State** (pushed on change and at least every `HEARTBEAT_PERIOD`):
`phase ∈ {UNAVAILABLE, INITIALIZING, IDLE, PLANNING, READY, DRIVING, ARRIVED, STOPPED}`;
`fault ∈ {NONE, MINIMAL_RISK_MANEUVER, EMERGENCY}` with optional `fault_behavior`
(`EMERGENCY_STOP`, `COMFORTABLE_STOP`, `PULL_OVER`, ...) and `fault_progress ∈ {OPERATING,
SUCCEEDED, FAILED}`; optional `turn_indicators ∈ {NONE, LEFT, RIGHT, HAZARD}`;
`capabilities`; free-form `detail`; optional `pose` (the autopilot's own estimate of where
the vehicle is); optional `rtc_status[]` (`{module, uuid, safe, requested, command_status,
state, auto_mode, start_distance, finish_distance}`, only with the RTC capability).

**Liveness**: each side sends something at least every `HEARTBEAT_PERIOD` = 1 s (`state` or
`heartbeat`) and treats a peer silent for `PEER_TIMEOUT` = 3 s as gone. The relay then
reports the entity `UNAVAILABLE`; the agent reconnects (backoff 1 → 5 s) and registers again.

**Commanders** (added for the `agent` controller, roadmap 017 step 6). Besides agents, the
relay accepts *commanders*: a client -- in practice a simulator adapter -- that commands the
agent registered under an entity's name and reads its state. That is how a scenario reaches
an agent-driven vehicle other than the ego, without the relay or the adapter learning
anything about the other's side. Same transport, same `"v": 1`; an agent's messages are
unchanged, and a `register` without `role` is an agent's, so existing agents need nothing.

| type | direction | fields |
|---|---|---|
| `register` | commander → relay, first line | `role: "commander"`, `agent` (name, informational); no `entity` |
| `registered` | relay → commander | `entity: ""` |
| `command_for` | commander → relay | `id` (the commander's), `entity`, `command`, `args` (as in `command`), optional `timeout` (s, default 9, capped at 30) |
| `query` | commander → relay | `id`, `entity` |
| `reply` | relay → commander | `id`, `status`, `message`, `registered` (bool: an agent is registered as `entity`), `state` (that agent's latest, when it has one) |

The relay forwards a `command_for` to the entity's agent as an ordinary `command` with an id
of its own, and answers the commander with the agent's `reply` -- `status`, `message`, and the
state that arrived with it (the ordering rule makes that the state the command produced). No
agent: `FAILED`, `registered: false`. No answer within `timeout`: `FAILED`. A `query` is
answered from memory at once: `OK` and the state, or `FAILED` and `registered: false`.
Commanders exchange no heartbeats and are never timed out for silence (they are
request/response clients; a closed socket ends them), own no entity, and cannot send agent
messages (`state`, `reply`: `error`, closed). Requests on one connection are served
concurrently; replies carry the commander's `id`. The relay keeps serving the concealer's ADAPI for its own
`entity` (the ego) exactly as before; agents registered under other names are reachable only
through commanders.

**Relay mapping to the concealer** (`scenario_agent_relay/mapping.py`):

| phase | localization | route | operation mode | concealer reads |
|---|---|---|---|---|
| no agent / silent / `UNAVAILABLE` | UNKNOWN | UNKNOWN | UNKNOWN, ADAPI services withdrawn | `INITIALIZING` (Autoware not up: every call waits for its service, 180 s) |
| `INITIALIZING` | INITIALIZING | UNSET | STOP | `INITIALIZING` |
| `IDLE` | INITIALIZED | UNSET | STOP | `WAITING_FOR_ROUTE` |
| `PLANNING` | INITIALIZED | SET | STOP, autonomous unavailable | `PLANNING` |
| `READY` | INITIALIZED | SET | STOP, autonomous available | `WAITING_FOR_ENGAGE` |
| `DRIVING` | INITIALIZED | SET | AUTONOMOUS, control enabled | `DRIVING` |
| `ARRIVED` | INITIALIZED | ARRIVED, stamped when first seen | AUTONOMOUS | `ARRIVED_GOAL` for 2 s, then `WAITING_FOR_ROUTE` |
| `STOPPED` | INITIALIZED | SET | STOP, autonomous unavailable | `PLANNING` (not engageable until a new goal) |

Services: `initialize` → `teleported(pose)`; `set_route_points` / `set_route` → `set_goal`;
`enable_autoware_control` and `engage(true)` → answered by the relay (the agent engages on
its own); `engage(false)` and `change_to_stop` → `stop`; `clear_route` → `clear_goal`;
`velocity_limit` → `set_speed_limit`; `rtc_commands` / `rtc_auto_mode` → `cooperate` /
`cooperate_auto`. `fault` → `/api/fail_safe/mrm_state` (NONE → NORMAL; MRM → OPERATING /
SUCCEEDED / FAILED with the behavior) and `EMERGENCY` → `/api/external/get/emergency`
true; `turn_indicators` → `/control/command/turn_indicators_cmd` (absent → NO_COMMAND);
`rtc_status` → `/api/external/get/rtc_status`; also `/autoware/state` (legacy). Each relay
reply deadline is under the concealer's per-attempt wait (2.5 s; 9 s for routing and
initialize), so a slow agent costs a retry, not a lost answer.

**What the concealer waits for, and who satisfies it now.** The concealer was written to
watch Autoware's own pipeline, not just ADAPI states:

- `initialize()` takes the newest `/localization/kinematic_state` stamp, waits for two
  `/localization/util/downsample/pointcloud` scans newer than it, sends the pose stamped
  with the newest scan, waits for `WAITING_FOR_ROUTE`, then requires a kinematic state
  newer than the first stamp within 1 m / 0.2 rad of the pose; ARRIVED_GOAL is judged
  against the newest scan stamp.
- The relay satisfies these **in its own time base**: it publishes an empty
  `PointCloud2` stamped with its clock at 10 Hz and republishes the agent's `pose` as
  `kinematic_state` stamped when that pose arrived. Every stamp the concealer compares
  then comes from one clock, and no fork change is needed.
- The real waits move to the vehicle side, against the autopilot's clock and sensors:
  `acb_agent` on `teleported` waits for two fresh scans of its own (10 s cap), calls
  `/api/localization/initialize`, and reports `INITIALIZING` until its estimate is newer
  than before and within 1 m / 0.2 rad of the pose (30 s cap, then `IDLE` with a `detail`
  saying it did not converge -- the concealer's own pose check then fails the scenario
  with both poses named). So `WAITING_FOR_ROUTE` is only reported once the pose agrees.

## Vehicle side

- **Self-contained autopilot.** The agent, at start: waits for its vehicle (bridge), lets
  the autopilot localize (seeded by the bridge, or from `teleported`), reports `IDLE`; on
  `set_goal` it routes, enables control, engages, and reports up to `ARRIVED`. Parameters
  and presets are the vehicle side's launch config, applied when it starts, never pushed by
  the scenario. With no scenario attached it can take a goal from a local file (today's
  `goal_poses_file`), so a vehicle side is testable alone.
- **acb_agent** (from `acb_pilot/auto_drive.py`, already CARLA-free: ADAPI only) is the
  Autoware agent. It is **simulator-neutral** -- it would serve an Autoware on AWSIM
  unchanged. It does not live in `acb_bridge`. Implemented as the `agent` entry point of
  `acb_pilot` (`ros2 run acb_pilot agent --ros-args -p relay:=tcp://host:5560 -p
  entity:=ego`), beside `auto_drive`: same package, same ADAPI-only dependencies, and the
  logic (`agent_core.py`) is ROS-free behind an `AutopilotPort` so it is tested against a
  fake Autoware. Without `relay` it drives once to `goal_poses_file` (local mode).
- **acb_bridge** is the CARLA vehicle bridge (I5): sensors, vehicle interface, `/clock`,
  traffic signals for one CARLA vehicle, found by `role_name`.
- Packages for Autoware users: `acb_launch` (`carla_simulator.launch.xml`, the CARLA profile
  around the user's `autoware.launch.xml`; starts `acb_bridge` and `acb_agent`; CARLA
  overrides switchable: `carla_localization`, `carla_perception`, `carla_system`),
  `acb_sensor_kit_launch` / `_description`, `acb_vehicle_launch` / `_description`.

## Simulation side

- **csb** (CARLA's SSv2 adapter, I1) becomes a ROS node: params `carla_host`, `carla_port`,
  `ssv2_port`, `config_file`, `weather`; `bridge.launch.xml` with `respawn="true"`. It serves
  the world-configuration services `/carla/set_weather` and `/carla/get_weather`
  (`csb_interfaces`, roadmap 018) -- simulator-specific, outside the five interfaces above.
- **agent relay** is its **own** package and node (`scenario_agent_relay`), not part of csb:
  it speaks I2 and I3 only, so it survives a simulator swap. One relay per scenario domain.
- `csb_launch/scenario.launch.xml`: `scenario:=` plus a few knobs; runs SSv2 with
  `managed_ego:=true` against the relay. `managed_ego:=false` and the unmanaged domain
  split are retired once the relay works.

## Background vehicles

Spawned by the scenario. Who drives each one is its OpenSCENARIO controller, settable per
entity and changeable mid-run (`AssignControllerAction`):

| Controller | Backend | Use |
|---|---|---|
| *(default)* | **SSv2 behavior tree**; the adapter places the actor kinematically each frame | scripted traffic, exact choreography; every SSv2 action; ~0.065 ms per NPC |
| `simulator_autopilot` | **the simulator's own driver**: for CARLA, Traffic Manager with physics on. Goal-driven (`AcquirePositionAction` → a route csb plans on lanelet2, converted to a TM path) or roaming; speed via `set_desired_speed`. The adapter returns the simulator's pose and SSv2 adopts it (`api.cpp:113`: non-ego status is applied as returned); the entity's SSv2 behavior is `do_nothing` | ambient traffic with real dynamics |
| `agent` | **an agent registered under the entity's name** (I3) -- e.g. a full Autoware on its own vehicle side. The adapter spawns the vehicle physics-on with the entity name as the simulator's vehicle name, forwards goals to the agent through the relay (as a commander), and returns the simulator's pose; SSv2's behavior is `do_nothing`, as for `simulator_autopilot` | AV-vs-AV studies, at a full stack's cost |

The controller names are **simulator-neutral**: `simulator_autopilot` means "whatever the
simulator drives with"; another adapter maps it to its own driver or rejects it.

### `simulator_autopilot` as built (roadmap 017 step 6)

SSv2 never told the simulator who drives an entity, so the fork adds the minimum to the
protocol (`simulation_api_schema.proto`) and keeps the scenario standard:

- **`SpawnVehicleEntityRequest.behavior`** (field 6): the entity's controller name. The
  interpreter already passes a non-ego controller's name to `traffic_simulator` as the
  behavior-plugin name; for `simulator_autopilot` it now spawns a
  `SimulatorDrivenVehicleEntity` (a `VehicleEntity` on the `do_nothing` plugin) instead of
  loading a plugin of that name. That entity runs no behavior and does not even update the
  do-nothing plugin (which would zero the twist the simulator reported); its status is
  what `UpdateEntityStatus` returned, adopted as for every non-ego entity.
- **`UpdateEntityGoal{name, waypoints[], clear, target_speed, has_target_speed}`** (oneof 16):
  `requestAcquirePosition`, `requestAssignRoute` and absolute `requestSpeedChange` on such an
  entity are queued; `API::updateFrame` sends the queue before the frame's
  `UpdateEntityStatus`, and a refused goal throws (the scenario fails with the simulator's
  reason). `cancelRequest` sends `clear`. Lane changes, relative speeds and trajectories
  throw `SemanticError`; so does switching to or from the controller after the spawn.
- **Other simulators**: `MultiServer` acknowledges `UpdateEntityGoal` itself, so a server
  that drives nothing needs no change; `simple_sensor_simulator` refuses a
  `simulator_autopilot` spawn rather than leave a vehicle that never moves.

csb (`autopilot.rs`, `coordinator.rs`):

- Spawn: physics on, parked, `csb_entity:<name>` role like every entity (reaped the same
  way); one Traffic Manager per csb process (port and seed in `bridge.yaml`
  `traffic_manager`), created at the first such spawn, synchronous whenever the world is.
  **LibCarla ticks a synchronous Traffic Manager inside `world.tick()`** (verified on
  0.9.16: a registered vehicle drives with world ticks alone, no `synchronous_tick`), so csb
  remains the only ticker.
- Hand-over when `UpdateEntityStatus.npc_logic_started` first arrives, so the vehicle waits
  with SSv2's own NPCs; goals received earlier are kept for then.
- Goals: Traffic Manager does not route between distant points -- given a path it takes, at
  each junction, the branch nearest the next point. Measured on Town01 with Traffic Manager
  alone: the goal as the only point missed 1 route in 4, points every 10 m along the route
  missed one that the goal alone reached. csb therefore plans the lane route itself (Dijkstra
  over `Map::topology()` segments sampled every 2 m, from the actor's lane position to the
  goal's) and gives Traffic Manager one point 5 m past each junction on it, then the goal;
  every probed route arrived, run to run identical with a fixed seed. Traffic Manager roams
  on past the end of a path, so csb slows the vehicle into the goal (1.5 m/s²) and, within
  0.5 m along the lane, unregisters it and brakes.
- Status: `UpdateEntityStatus` returns CARLA's pose, twist and acceleration for these
  entities (the ego's readback: origin offset undone, settle frames and median filter), and
  never teleports them. Failures name the entity and fail the frame. Twist and acceleration
  are now reported in the **entity's frame** (x forward), as SSv2 reads them; they used to
  go out world-frame, which made a vehicle heading along y read as standing still (a
  `SpeedCondition < 0.05` held at 2.8 m/s) and one heading along -x as reversing -- for the
  ego too. The hand-over gets a 1 s settle like the spawn: Traffic Manager's launch from
  the hand brake reads as 15.5 m/s² for a frame, then a steady ~5.2 m/s².
- Signals: **starting a Traffic Manager resets every light group to its own cycle** (24 of
  Town01's 36 lights RED after the first such spawn; the vehicle then waited at a red light
  until the timeout). csb re-applies its signal state right after creating it: every mapped
  light GREEN, then what the scenario has commanded this run.
- Arrival: within 0.5 m of the goal csb asks Traffic Manager for 0 m/s and parks the vehicle
  (unregistered, hand brake) once below 0.1 m/s; parking at the approach's 1 m/s read as
  -8 m/s².
- Teardown: despawn, Initialize and shutdown release vehicles from Traffic Manager before
  destroying them; a CARLA restart shuts the old Traffic Manager down.

The lane graph is CARLA's OpenDRIVE topology, not the Lanelet2 map: Traffic Manager follows
CARLA's lanes, and routing on the same graph it drives keeps the two from disagreeing.

### `agent` as built (roadmap 017 step 6)

Background AVs used to be listed in `bridge_config.yaml` (`background_avs`), spawned by csb
at every `Initialize` and invisible to SSv2. They are now scenario entities:

```xml
<ScenarioObject name="bg_av_1"> ... <Controller name="agent"><Properties/></Controller> ...
```

- **SSv2 fork**: `agent` is handled exactly like `simulator_autopilot` -- a
  `SimulatorDrivenVehicleEntity` (it records which of the two names it stands for), spawned
  with `behavior: "agent"`; `AcquirePositionAction`, `AssignRouteAction` and an absolute
  `SpeedAction` go out as `UpdateEntityGoal`; lane changes, relative speeds, trajectories and
  controller switches are scenario errors. `simple_sensor_simulator` refuses the spawn.
- **csb** (`agent_link.rs`, `coordinator.rs`):
  - Spawn: physics on, parked (hand brake) like the ego, **`role_name` = the entity name**,
    so the vehicle side started with `vehicle_name:=<entity>` attaches to it; not given to
    Traffic Manager. Refused if the name is the ego's role name, a `csb_entity:` mark, or a
    role another CARLA vehicle already has. Spawning succeeds whether or not an agent is
    registered (a vehicle side may come up later).
  - Reaping: these vehicles carry no `csb_entity:` mark (acb finds them by name), so csb
    records their role names in a ledger on tmpfs (`$XDG_RUNTIME_DIR/carla-scenario-bridge/
    agent-vehicles-<host>-<port>`, beside the episode clock) and the reaper at `Initialize`
    destroys vehicles with those names, as it does the ego's `hero`.
  - Commands: csb is a relay **commander** (`agent_relay` in `bridge.yaml`, default
    `tcp://localhost:5560`; ROS parameter `agent_relay`; `simulation.launch.xml` points it at
    its own relay). After the spawn it sends `teleported` with the spawn pose (retried until
    an agent is registered); `UpdateEntityGoal` becomes `set_speed_limit` (absolute target
    speed) and `set_goal` (waypoints, the last is the goal) or `clear_goal`; despawn,
    `Initialize` and shutdown send `stop`. All of it from a worker thread, in order per
    entity, never inside a frame: a goal is held until SSv2's NPC logic has started (as for
    `simulator_autopilot`) **and** the agent reports a phase that takes goals (`IDLE` and
    later) -- acb's agent refuses one while re-localizing after `teleported`.
  - Failures are honest: `UpdateEntityGoal` for an entity with no registered agent fails at
    once, naming the entity and the `entity:=` to start its vehicle side with (one
    synchronous relay `query`, answered from memory); a goal the agent refuses (`FAILED`,
    `UNSUPPORTED`) or a relay that is gone becomes the entity's error, and the next
    `UpdateEntityStatus` fails the frame with it, so SSv2 ends the scenario.
  - Status: `UpdateEntityStatus` returns CARLA's pose, entity-frame twist and acceleration
    for it -- the ego's and `simulator_autopilot`'s readback -- and never teleports it.
- **Vehicle side**: `csb_launch background_av.launch.xml entity:=<name> relay:=...` is the
  acb CARLA profile with the agent registered as `<name>` and `vehicle_name` = `<name>`;
  `just bg-av` runs it in domain 2.
- **Speed**: an absolute `SpeedAction` is the agent's speed *limit*, not a target: the
  autopilot drives its own profile under it.

## Swap audit

**Replace CARLA with simulator X.** Replace: the simulator adapter (csb → X's I1
implementation; AWSIM already has one) and the vehicle bridge (acb_bridge → X's bridge for
Autoware). Keep: SSv2, the relay, the agent protocol, acb_agent, Autoware and its launch
profile minus the CARLA overrides, scenario files, the map dir minus `carla/`.

**Replace Autoware with autopilot Y.** Replace: the agent (acb_agent → Y's agent, I3 ↔ I4)
and the vehicle bridge's message set (I5: acb publishes Autoware's messages; Y needs its own
or an adapter). Keep: SSv2, csb, the relay, CARLA, scenario files. Scenario features that
are Autoware-specific degrade explicitly: RTC answers `UNSUPPORTED`; `fault` is `NONE` if Y
has no notion of it.

Findings, each folded into the design above:

1. **The relay must not live in csb.** First draft had csb serve the ADAPI subset; that
   ties every vehicle's command path to the CARLA adapter. → separate `scenario_agent_relay`.
2. **Agents are keyed by entity name, not `role_name`.** `role_name` is CARLA's; the
   scenario knows entity names. The vehicle bridge maps entity → simulator vehicle on its
   side (acb: `vehicle_name`).
3. **The Autoware agent must not live in the CARLA bridge.** First draft put it in acb; it
   belongs beside it, simulator-neutral (it already is: `auto_drive.py` speaks ADAPI only).
4. **"Initialize localization" is an Autoware action, not a scenario event.** →
   `teleported(pose)`, which each agent interprets.
5. **Autoware states in the protocol** (MRM, legacy states) → a neutral `phase` and `fault`;
   the relay alone maps them back to Autoware's names for the concealer.
6. **`carla_autopilot` in a scenario file would pin scenarios to CARLA.** →
   `simulator_autopilot`.
7. **CARLA artifacts in the shared map dir.** `traffic_lights.yaml` (lanelet signal ↔ CARLA
   light) is CARLA's → `<map dir>/carla/traffic_lights.yaml`, generated offline with the
   map; csb checks it at `Initialize` instead of writing it at run time; acb reads it.
8. **The vehicle description package is a shared dependency, and that is acceptable**:
   SSv2 needs dimensions, the autopilot needs them, both read `vehicle_model`'s
   `*_description` (`vehicle_info.param.yaml`). It is the one artifact both sides must agree
   on; a non-Autoware autopilot provides a package of the same shape.

Remaining couplings, accepted:

- **Time** is the simulator's: the vehicle bridge publishes `/clock` from it, the adapter
  reports it to SSv2 (time-and-ticking.md). Any simulator pair must do the same.
- **Signals**: the vehicle bridge publishes the simulator's light states as the autopilot's
  V2X input. Per simulator, per autopilot -- part of I5.
- **I2 is Autoware-shaped** because SSv2's concealer is. The relay confines it to the
  scenario domain; no vehicle side sees it.

## The user's files

```
<map dir>/      lanelet2_map.osm  pointcloud_map.pcd  pointcloud_map_metadata.yaml
                map_config.yaml   map_projector_info.yaml
                carla/traffic_lights.yaml        # simulator-specific, generated with the map
<scenario dir>/ bridge.yaml        # optional: CARLA host/port, blueprint mapping
                <name>.xosc        # LogicFile filepath = <map dir>; entities and controllers
```

Background vehicles are entities in the `.xosc`, not entries in `bridge.yaml`
(`background_avs` is refused since roadmap 017 step 6).

## A session

```bash
# CARLA (vendor binary; documented flags: -RenderOffScreen, Epic, NVIDIA Vulkan ICD)
./CarlaUE4.sh -RenderOffScreen

# Simulation side, long-lived (scenario domain)
play_launch launch carla_scenario_bridge bridge.launch.xml config_file:=<scenario dir>/bridge.yaml
play_launch launch scenario_agent_relay relay.launch.xml

# Vehicle side, any domain, any order
ROS_DOMAIN_ID=7 play_launch launch acb_launch carla_simulator.launch.xml \
    autoware_launch:=<their autoware.launch.xml> map_path:=<map dir> \
    vehicle_name:=hero entity:=ego relay:=tcp://<sim host>:5560
# ... and one per background AV (an entity with controller `agent`)
ROS_DOMAIN_ID=8 play_launch launch acb_launch carla_simulator.launch.xml \
    autoware_launch:=<their autoware.launch.xml> map_path:=<map dir> \
    vehicle_name:=bg_av_1 entity:=bg_av_1 relay:=tcp://<sim host>:5560

# A scenario; exits on its own; repeat
play_launch launch csb_launch scenario.launch.xml scenario:=<scenario dir>/<name>.xosc
```

## Migration (roadmap 017)

1. csb as a ROS node: params, installed config as default, launch file with respawn.
2. `scenario.launch.xml`; SSv2 fork's preprocessor writes its copy to the output dir, never
   over the input.
3. Offline `<map dir>/carla/traffic_lights.yaml`; csb verifies instead of writing.
4. `carla_simulator.launch.xml` starts `acb_bridge` (and later `acb_agent`); switchable
   overrides; csb's `ego_av`/`background_av` launch files shrink to it.
5. Agent protocol schema; `scenario_agent_relay`; `acb_agent` from `auto_drive.py`. Then
   retire `managed_ego:=false` and the domain coupling.
6. `simulator_autopilot` controller in csb (Traffic Manager; lanelet route → TM path).
7. User guide; README; `ssv2-launch-configuration.md` rewritten to this.
