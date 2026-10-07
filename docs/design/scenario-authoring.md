# Writing Scenarios

Scenarios are standard OpenSCENARIO 1.x files run by scenario_simulator_v2 (SSv2). This page
covers what is specific to running them in CARLA through carla-scenario-bridge. Workflow:
[../user-guide.md](../user-guide.md); architecture: [user-workflow.md](user-workflow.md).

## The map

`RoadNetwork/LogicFile` names the map directory -- the same directory the vehicle side's
Autoware gets as `map_path` -- by path:

```xml
<LogicFile filepath="/data/maps/Town01"/>
<LogicFile filepath="$(env CARLA_MAPS)/Town01"/>   <!-- the repo's examples -->
```

The directory's name is the CARLA town (or an alias in `bridge_config.yaml`'s `map_alias`);
the bridge loads that town if CARLA holds another. The scenario file itself is never
modified.

## The ego

The entity whose `ObjectController` has the property `isEgo=true`. Its Autoware runs on the
vehicle side, possibly in another ROS domain or on another host; the scenario reaches it
through the agent relay, so everything SSv2 offers for an ego works as usual:

- `TeleportAction` at init places the vehicle in CARLA and tells the vehicle's agent, which
  re-initializes localization there.
- `AcquirePositionAction` / `AssignRouteAction` set its goal; the agent routes and engages.
- `maxSpeed` (controller property) and `SpeedAction` set its speed limit.
- `ego.currentState` (`WAITING_FOR_ROUTE`, `DRIVING`, `ARRIVED_GOAL`, ...), MRM and turn
  indicator conditions read the agent's reported state.
- `RequestToCooperateCommandAction` (RTC) works on Autoware; an agent without RTC answers
  `UNSUPPORTED` and the action fails (slowly: SSv2 retries it for ~90 s).

The ego cannot be told to change lanes or follow a relative speed: SSv2 refuses both for an
ego, by design.

## Other vehicles, pedestrians and objects

Driven by SSv2 by default: its behavior tree computes each pose and the bridge places the
CARLA actor there every frame, physics off -- exact and repeatable, but never pushed by a
collision (see "Collisions" below). Every SSv2 action works on them.

## Traffic lights: uncommanded means GREEN

Applies to every scenario, managed ego or not (roadmap 015, "Signals from CARLA").

Autoware learns signal state from CARLA's light actors: acb_bridge reads them every frame and
publishes them on the traffic light arbiter's V2X input
(`/perception/traffic_light_recognition/external/traffic_signals`). So every mapped light is
visible to the ego, not only the ones the scenario names.

At every `Initialize` the bridge therefore sets **every mapped light GREEN**, then freezes
CARLA's cycling and holds each light's phase. A light the scenario never commands stays GREEN
for the whole run. A light the scenario does command (`TrafficSignalStateAction`) shows what
it commands from its first `UpdateTrafficLights` on, and keeps the last commanded state until
the next command or the next `Initialize`.

- To stop the ego at an intersection, command that signal. Do not rely on the state a previous
  run or CARLA's own cycle left a light in; it is reset to GREEN.
- "Mapped" means listed in the resolved table the bridge writes beside the Lanelet2 map at
  `Initialize` (`traffic_lights.resolved.yaml`, logged with its path): Lanelet2 traffic light
  ways matched to a CARLA light by position, plus `config/traffic_lights_<town>.yaml`. CARLA
  lights the Lanelet2 map does not reference keep their frozen state; Autoware cannot see them.
- SSv2's own V2X publisher (`publish_conventional_traffic_signals`) is off in
  `carla_scenario.launch.xml`; turning it on as well would put two sources on the arbiter's
  input. SSv2 itself still treats an uncommanded signal as having no state, so conditions on
  a signal's state only see what the scenario commanded.

## Collisions: do not end on the collision frame

SSv2's `CollisionCondition` (bounding boxes) is the verdict for every collision, NPC-NPC
included. csb also attaches a CARLA collision sensor to the ego and logs what CARLA saw, as
a cross-check. That cross-check only sees contacts that last more than one frame: csb applies
SSv2's NPC pose one frame after SSv2 computes it, so a scenario that exits on the first frame
of overlap ends before CARLA ticks with the bodies touching, and the bridge logs "no
collisions" for a run SSv2 scored as a collision. Delay the exit (e.g. `delay="3"` on the
`CollisionCondition`'s `Condition`) when the CARLA side matters. Also remember that NPCs are
kinematic: one never moves when hit, and two NPCs pass through each other in CARLA (see
[multi-instance-architecture.md](multi-instance-architecture.md#consequence-puppeteered-actors-are-kinematic)).
Example: `scenarios/town01_rear_contact.xosc`.
