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
│ (sim-neutral)                          │  (ZMQ+pb)  │                               │
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
| I3 | **Agent protocol** (ZMQ + protobuf) | agent relay ↔ agent | this design | no | no |
| I4 | Autopilot native interface | agent ↔ autopilot | the autopilot | no | yes (Autoware: ADAPI, local domain) |
| I5 | Vehicle I/O (sensors, vehicle interface, `/clock`, signals) | vehicle bridge ↔ autopilot | the autopilot's message set | yes (bridge talks to the simulator) | yes (publishes the autopilot's messages) |

Private, not part of any contract: csb ↔ CARLA, acb ↔ CARLA, and the csb ↔ acb sensor-release
channel (CARLA actor ids).

## The agent protocol (I3)

- **Transport**: ZMQ, protobuf schema with a version field. The relay binds one endpoint
  (`agent_endpoint`, default `tcp://*:5560`); agents connect and **register by entity
  name** -- the SSv2 entity name the scenario uses (`ego`), never a simulator id. The relay
  needs no address list, and an agent can come up before or after the scenario.
- **Commands** (relay → agent):
  `set_goal(pose, waypoints[], allow_goal_modification)`, `clear_goal()`,
  `set_speed_limit(mps)`, `stop()`, `teleported(pose)` -- an event, not an instruction:
  the vehicle was placed at `pose`; an agent whose autopilot localizes uses it to
  re-initialize -- and optional `cooperate(module, command)` / `cooperate_auto(module, on)`
  (RTC). Unsupported optional commands are answered `UNSUPPORTED`, which the relay turns
  into the concealer's failure, so a scenario that needs RTC fails clearly on an autopilot
  without it.
- **State** (agent → relay, pushed on change, with a heartbeat):
  `phase ∈ {UNAVAILABLE, INITIALIZING, IDLE, PLANNING, READY, DRIVING, ARRIVED, STOPPED}`,
  `fault ∈ {NONE, MINIMAL_RISK_MANEUVER, EMERGENCY}` with an optional behavior string,
  `turn_indicators ∈ {NONE, LEFT, RIGHT, HAZARD}` (optional), `capabilities` (which optional
  commands it accepts), and a free-form `detail`.
- **Relay mapping to the concealer**: `phase` → legacy Autoware state
  (`INITIALIZING`, `WAITING_FOR_ROUTE`, `PLANNING`, `WAITING_FOR_ENGAGE`, `DRIVING`,
  `ARRIVED_GOAL`), the ADAPI state topics the concealer reads, and engage gating;
  `set_route_points` + `enable_autoware_control` + `engage` → one `set_goal` (the agent
  engages on its own once ready); `fault` → `mrm_state`; a missing or silent agent →
  `UNAVAILABLE`, which the concealer sees as Autoware not up.

## Vehicle side

- **Self-contained autopilot.** The agent, at start: waits for its vehicle (bridge), lets
  the autopilot localize (seeded by the bridge, or from `teleported`), reports `IDLE`; on
  `set_goal` it routes, enables control, engages, and reports up to `ARRIVED`. Parameters
  and presets are the vehicle side's launch config, applied when it starts, never pushed by
  the scenario. With no scenario attached it can take a goal from a local file (today's
  `goal_poses_file`), so a vehicle side is testable alone.
- **acb_agent** (from `acb_pilot/auto_drive.py`, already CARLA-free: ADAPI only) is the
  Autoware agent. It is **simulator-neutral** -- it would serve an Autoware on AWSIM
  unchanged. It does not live in `acb_bridge`.
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
