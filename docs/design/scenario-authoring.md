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

## Vehicles the simulator drives: `simulator_autopilot`

Name a vehicle's controller `simulator_autopilot` and CARLA drives it instead of SSv2:
Traffic Manager steers it with physics on, it brakes for vehicles ahead and for signals, and
it can be pushed in a collision. The file stays standard OpenSCENARIO -- the controller name
is the only marker:

```xml
<ScenarioObject name="npc_autopilot">
  <Vehicle name="vehicle.tesla.model3" vehicleCategory="car"> ... </Vehicle>
  <ObjectController>
    <Controller name="simulator_autopilot"><Properties/></Controller>
  </ObjectController>
</ScenarioObject>
...
<Private entityRef="npc_autopilot">
  <PrivateAction><TeleportAction> ...start... </TeleportAction></PrivateAction>
  <PrivateAction><LongitudinalAction><SpeedAction>
    <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
    <SpeedActionTarget><AbsoluteTargetSpeed value="8.0"/></SpeedActionTarget>
  </SpeedAction></LongitudinalAction></PrivateAction>
  <PrivateAction><RoutingAction><AcquirePositionAction> ...goal... </AcquirePositionAction></RoutingAction></PrivateAction>
</Private>
```

Example: `scenarios/town01_simulator_autopilot.xosc` (ego, an SSv2-driven NPC and a
CARLA-driven NPC in one run).

What happens:

- **Start.** The vehicle is spawned with physics on and parked (hand brake) until SSv2
  starts its NPC logic, like SSv2's own NPCs; then it is handed to Traffic Manager.
- **Goal.** `AcquirePositionAction` (or `AssignRouteAction`: waypoints in order, the last is
  the goal) goes to the bridge, which plans the lane route on CARLA's road topology and
  gives Traffic Manager a point past every junction on it, then the goal. The vehicle stops
  at the goal (≤ 0.5 m along its lane, decelerating at 1.5 m/s²) and stays parked until the
  next goal. Without a goal it roams the town at Traffic Manager's choice.
- **Speed.** An absolute `SpeedAction` target becomes Traffic Manager's desired speed. The
  vehicle accelerates and brakes as its driver does, not with the action's dynamics; a
  `step` SpeedAction completes at once as usual, others when the speed is reached.
- **Pose.** SSv2 adopts the pose, velocity and acceleration CARLA reports every frame, so
  `ReachPositionCondition`, `SpeedCondition`, distances and `CollisionCondition` see where
  the vehicle really is. Declare a `Performance` that fits the driver: Traffic Manager
  accelerates at about 5.2 m/s² from rest (the example declares 10).
- **Repeatability.** Traffic Manager runs synchronously, ticked with the world, and is
  reseeded at every scenario start (`traffic_manager.seed` in `bridge.yaml`): the same
  scenario on the same CARLA drives the same way.

Not supported -- each is an explicit scenario error, not a silent no-op:

- `LaneChangeAction`, a relative `SpeedAction` (`RelativeTargetSpeed`), and
  `FollowTrajectoryAction`: the driver decides lanes and speed profile.
- Switching to or from `simulator_autopilot` with `AssignControllerAction` mid-run: the
  simulator learns who drives an entity only at its spawn.
- A goal the lane topology does not reach -- on another lane of the same road (that needs a
  lane change, which the route does not plan) or behind a one-way end: the action fails with
  the reason.
- Pedestrians and misc objects: vehicles only.

Limits worth knowing: a `TeleportAction` after the start is ignored (CARLA owns the pose);
the route's lanes are followed with lane changes off, so a stopped vehicle ahead on a
single-lane road holds it up; and with SSv2's own simulator (`simple_sensor_simulator`)
the spawn is refused, since it has no driver of its own.

## Vehicles an autopilot drives: `agent`

Name a vehicle's controller `agent` and an autopilot of its own drives it -- typically a
full Autoware on its own vehicle side, the same stack as the ego's. This is how a scenario
has more than one autonomous vehicle (SSv2 allows one ego), and the extra ones are ordinary
scenario entities: conditions, distances and `CollisionCondition` see them.

```xml
<ScenarioObject name="bg_av_1">
  <Vehicle name="vehicle.tesla.model3" vehicleCategory="car"> ... </Vehicle>
  <ObjectController>
    <Controller name="agent"><Properties/></Controller>
  </ObjectController>
</ScenarioObject>
...
<Private entityRef="bg_av_1">
  <PrivateAction><TeleportAction> ...start... </TeleportAction></PrivateAction>
  <PrivateAction><RoutingAction><AcquirePositionAction> ...goal... </AcquirePositionAction></RoutingAction></PrivateAction>
</Private>
```

Example: `scenarios/town01_two_av.xosc` (the ego and `bg_av_1` in one lane; `just two-av`).

The entity's name is the link to its vehicle side: start one per such entity, with
`entity:=<name>` (and `vehicle_name:=<name>`, the default in `csb_launch
background_av.launch.xml`) and the simulation side's relay, in a ROS domain of its own:

```bash
ROS_DOMAIN_ID=2 play_launch launch csb_launch background_av.launch.xml \
    entity:=bg_av_1 map_path:=<map dir> relay:=tcp://<simulation host>:5560
```

What happens:

- **Start.** The bridge spawns the vehicle with physics on, parked, and with CARLA
  `role_name` = the entity name, which is how that vehicle side's `acb_bridge` finds it. It
  tells the agent where the vehicle is (`teleported`); Autoware re-localizes there.
- **Goal.** `AcquirePositionAction` (or `AssignRouteAction`: waypoints in order, the last is
  the goal) becomes the agent's goal once SSv2's NPC logic has started and the agent has
  localized; the autopilot plans its own route, engages and drives.
- **Speed.** An absolute `SpeedAction` target becomes the autopilot's speed *limit*.
- **Pose.** SSv2 adopts the pose, velocity and acceleration CARLA reports every frame, as for
  `simulator_autopilot`.
- **End.** Despawn (and the next scenario's start) tells the agent to stop. The vehicle side
  is long-lived and serves the next scenario.

Fails, explicitly:

- A goal for an entity no agent is registered for: the action fails at once and names the
  `entity:=` to start. The spawn itself succeeds -- a vehicle side may start late; it is
  told where its vehicle is as soon as it registers.
- A goal the autopilot refuses (e.g. Autoware's "The planned route is empty" for a goal
  behind the vehicle or on no lane): the next frame fails with the autopilot's reason.
- `LaneChangeAction`, a relative `SpeedAction`, `FollowTrajectoryAction`, and switching to or
  from `agent` mid-run, as for `simulator_autopilot`; a name equal to the ego's CARLA role
  (`hero`), or one another CARLA vehicle already has.

Background AVs used to be declared in `bridge.yaml` (`background_avs`), outside the
scenario and invisible to SSv2; that key is now refused.

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
