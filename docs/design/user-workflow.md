# User Workflow and the Vehicle Agent

How a user sets up and runs a scenario, and how the scenario reaches the vehicles in it.
Status: **proposal** (UX campaign, 2026-10-08). Roadmap: 017 (to be written).

## Goals

1. **Users run ROS tools.** `ros2 run`, and `play_launch launch` in place of `ros2 launch`.
   The justfile stays as the developers' shortcut layer and calls exactly the commands below.
2. **The user brings a scenario dir and a map dir.** Nothing in this repo's tree is
   referenced from a scenario; maps are passed by path.
3. **Two self-contained sides.**
   - *Simulation side* -- CARLA, csb, SSv2: builds the world, runs NPCs and signals, owns
     time, judges the scenario.
   - *Vehicle side* -- one per vehicle: an autopilot (the user's Autoware, or anything
     else) plus acb, attached to one CARLA vehicle by `role_name`.
   They share CARLA and the map dir, and nothing else: no ROS domain, no start order.
4. **The scenario observes and commands a vehicle without knowing what drives it.** A
   command is a *goal* (plus a speed limit); the autopilot decides how to get there.

## How SSv2 talks to Autoware today

All of it goes through the concealer (`external/concealer/src/field_operator_application.cpp`),
built into the ego entity, speaking Autoware's ADAPI **over ROS in SSv2's own domain**. That
is the only reason SSv2 and Autoware must share a domain. In our fork it is active only with
`managed_ego:=true`; unmanaged, every call below is inert (`requireManaged`).

| Scenario feature | Concealer call | Autoware interface |
|---|---|---|
| Ego initial pose (`TeleportAction` at init) | `initialize()` | `/api/localization/initialize`, waits on `/api/localization/initialization_state` |
| Ego goal (`AcquirePositionAction`, `AssignRouteAction`) | `plan()`, then `enableAutowareControl()` + `engage()` | `/api/routing/set_route_points` (`set_route`), `/api/routing/clear_route`, `/api/operation_mode/enable_autoware_control`, `/api/external/set/engage` |
| Ego speed limit (controller `maxSpeed`, `SpeedAction` on ego) | `setVelocityLimit()` | `/api/autoware/set/velocity_limit` |
| Stop between scenarios | `change_to_stop` at start/end | `/api/operation_mode/change_to_stop` |
| Engage gating in the interpreter | `engageable()`, `engaged()` | derived state (below) |
| `ego.currentState` | `getLegacyAutowareState()` | `/api/routing/state`, `/api/operation_mode/state`, `/api/localization/initialization_state` (legacy `/autoware/state` as fallback) |
| `ego.currentMinimumRiskManeuverState.*`, `currentEmergencyState` | MRM state | `/api/fail_safe/mrm_state`, `/api/external/get/emergency` |
| `ego.currentTurnIndicatorsState` | last command | `/control/command/turn_indicators_cmd` |
| RTC (`CustomCommandAction` `RequestToCooperateCommandAction`) | `sendCooperateCommand()`, `requestAutoModeForCooperation()` | `/api/external/set/rtc_commands`, `/api/external/set/rtc_auto_mode`, `/api/external/get/rtc_status` |
| Observed only | | `/localization/kinematic_state`, `/control/command/control_cmd`, `.../path_with_lane_id`, `/localization/util/downsample/pointcloud` (our fork: scan stamps for the time base) |

Everything else about the ego -- its pose each frame, its dimensions -- does not involve
Autoware: csb reads the pose back from CARLA, and SSv2 reads the dimensions from
`acb_vehicle_description`. SSv2 also refuses lane changes and relative speed changes on an
ego by design ("Ego cars make autonomous decisions about everything but their destination").

So the scenario already treats the ego as **goal-driven**: it sets where to go, how fast at
most, and occasionally answers an RTC request; it observes a coarse state. That is the
contract to keep, minus the transport.

## Target: the vehicle agent

```
  scenario domain                            vehicle domain (one per vehicle)
 ┌──────────────────────────────┐           ┌───────────────────────────────────┐
 │ SSv2 interpreter + concealer │           │ autopilot (Autoware, or other)    │
 │        │ ADAPI subset (ROS)  │           │        ▲ its own interface        │
 │        ▼                     │  agent    │        │ (Autoware: local ADAPI)  │
 │ csb (ROS node)  ─────────────┼─protocol──┼──▶ acb agent ── acb_bridge ───┐   │
 │   │ ZMQ :5555 (SSv2 protocol)│  (ZMQ)    │                              │   │
 └───┼──────────────────────────┘           └──────────────────────────────┼───┘
     ▼                                                                     ▼
   CARLA ◀──────────────────── world, time, sensors, control ──────────────┘
```

- **Agent protocol** (new, small, transport-neutral; ZMQ like the existing csb↔acb sensor
  release channel, so it crosses domains and hosts): per vehicle, by `role_name`.
  - Commands: `set_goal(pose, waypoints[], allow_goal_modification)`, `clear_goal()`,
    `set_speed_limit(mps)`, `stop()`; optionally `cooperate(module, command)` for RTC.
  - State (pushed on change): `IDLE | LOCALIZING | ROUTING | READY | DRIVING | ARRIVED |
    STOPPED`, MRM `{state, behavior}`, turn indicators, and a free-form detail string.
- **Vehicle side: acb serves the agent.** For Autoware, acb's agent (today's `acb_pilot`
  grown up) calls Autoware's ADAPI **locally, in the vehicle's own domain**. Autoware is
  self-contained: on attach, acb seeds localization at the vehicle's pose (exists:
  `seed_localization_on_attach`); it waits for a goal; on a goal it sets the route,
  enables control and engages; on `ARRIVED` it reports and holds. Vehicle-specific
  settings (params, presets) are the vehicle side's launch config, applied when it starts,
  not pushed by the scenario. A goal can also come from a local file when no scenario is
  running (today's `goal_poses_file`), so a vehicle side is testable on its own.
- **Other autopilots.** Anything that implements the agent drives a vehicle: a different
  stack behind its own acb-style adapter, or a built-in csb agent using CARLA's
  BasicAgent/Traffic Manager for a goal-driven vehicle with no ROS at all. The scenario
  cannot tell them apart.
- **Simulation side: csb relays.** csb becomes a ROS node (below) and serves, in the
  scenario's domain, the ADAPI subset the concealer calls, for the ego's `role_name`, and
  forwards each call to that vehicle's agent; it publishes the agent's state back as the
  ADAPI state topics the concealer reads. **SSv2 is unchanged**: it runs `managed_ego:=true`
  and believes it is talking to Autoware. (Alternative: replace the concealer's transport
  in the fork. The relay keeps the fork small and works with upstream SSv2.)
- **Initial pose.** `TeleportAction` places the vehicle (csb, in CARLA) and the concealer's
  `initialize()` arrives at the relay, which tells the agent to re-seed localization at the
  new pose -- the same thing acb already does on an ego respawn.

What this removes: the shared-domain requirement, the start-order requirement (the agent
waits for its vehicle, the relay reports "not ready" until an agent attaches), and the
managed/unmanaged split -- there is one mode. What it keeps: every scenario feature in the
table, with `currentState` mapped from the agent state.

Not carried in v1, decided per feature: the observed-only topics (`control_cmd`,
`path_with_lane_id`), which are Autoware internals no other autopilot has.

## Packages users get

**Vehicle side (for Autoware users)**, all in acb:
- `acb_launch` -- `carla_simulator.launch.xml`, the CARLA profile around the user's own
  `autoware.launch.xml` (`autoware_launch:=`). It starts `acb_bridge` and the agent itself,
  and each CARLA override is switchable (`carla_localization`, `carla_perception`,
  `carla_system`, default on) so a user's own component wins when they turn ours off. What
  it overrides and why: `use_sim_time` everywhere; the planning preset (modules that need
  lanelet attributes CARLA maps lack); CARLA-tuned localization params; perception with a
  selectable `lidar_detection_model` and fusion-only traffic lights (signals come from acb);
  system with our component-state topics, MRM params and diagnostic graph.
- `acb_sensor_kit_launch`, `acb_sensor_kit_description` -- `sensor_model:=acb_sensor_kit`.
- `acb_vehicle_launch`, `acb_vehicle_description` -- `vehicle_model:=acb_vehicle`.

**Simulation side**, in this repo:
- `carla_scenario_bridge` -- csb as a ROS node: params `carla_host`, `carla_port`,
  `ssv2_port`, `config_file`; launch file `bridge.launch.xml` with `respawn="true"`
  (play_launch supervises; the justfile loop goes).
- `csb_launch` -- `scenario.launch.xml`: `scenario:=` plus a few knobs; SSv2's fixed args
  inside.

## The user's files

```
<map dir>/      lanelet2_map.osm  pointcloud_map.pcd  pointcloud_map_metadata.yaml
                map_config.yaml   map_projector_info.yaml  traffic_lights.yaml
<scenario dir>/ bridge.yaml        # optional: CARLA host/port, vehicles to spawn
                <name>.xosc        # LogicFile filepath = <map dir>
```

- The map dir is generated by a future tool. `traffic_lights.yaml` (lanelet signal ↔ CARLA
  light) is generated with it, offline; csb **checks** it against CARLA at `Initialize`
  instead of writing it into the map dir at run time, as it does today.
- Vehicles other than the ego (background AVs) are declared in `bridge.yaml` (role_name,
  blueprint, spawn pose); csb spawns them, and a vehicle side attaches by `role_name`. Their
  goals come from the scenario through the same agent protocol once SSv2 can address a
  non-ego entity's agent -- until then, from the vehicle side's goal file.

## A session

```bash
# CARLA (vendor binary; flags documented: -RenderOffScreen, Epic, NVIDIA Vulkan ICD)
./CarlaUE4.sh -RenderOffScreen

# Simulation side, long-lived
play_launch launch carla_scenario_bridge bridge.launch.xml config_file:=<scenario dir>/bridge.yaml

# Vehicle side, any domain, any order
ROS_DOMAIN_ID=7 play_launch launch acb_launch carla_simulator.launch.xml \
    autoware_launch:=<their autoware.launch.xml> map_path:=<map dir> vehicle_name:=hero

# A scenario; exits on its own; repeat
play_launch launch csb_launch scenario.launch.xml scenario:=<scenario dir>/<name>.xosc
```

## Migration

1. csb as a ROS node: params, installed config as default, launch file with respawn.
2. `scenario.launch.xml`; SSv2 fork's preprocessor writes its copy to the output dir, never
   over the input (the justfile copies the input today for that reason).
3. Offline `traffic_lights.yaml`; csb verifies instead of writing.
4. `carla_simulator.launch.xml` starts `acb_bridge`; switchable overrides; csb's
   `ego_av`/`background_av` launch files shrink to it.
5. Agent protocol; acb agent for Autoware (from `acb_pilot`); csb relay for the concealer's
   ADAPI subset. Then retire `managed_ego` and the domain coupling.
6. User guide; README; `ssv2-launch-configuration.md` rewritten to this.
