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
  `ssv2_port`, `config_file`; `bridge.launch.xml` with `respawn="true"`.
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
| `agent` | an agent registered under the entity's name (I3) -- e.g. a full Autoware | AV-vs-AV studies, at a full stack's cost |

The controller names are **simulator-neutral**: `simulator_autopilot` means "whatever the
simulator drives with"; another adapter maps it to its own driver or rejects it.

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

Background vehicles are entities in the `.xosc`, not entries in `bridge.yaml`.

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
