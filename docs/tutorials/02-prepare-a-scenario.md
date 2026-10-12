# Tutorial 2: Prepare a scenario

Write a scenario of your own from nothing but the map: the ego drives 80 m down a Town01
road to a goal, behind a slower car it must not touch. You will find the poses with a helper,
write the `.xosc`, and run it on the stack from [Tutorial 1](01-first-run.md).

The finished file ships with the workspace, so you can compare against it:
`$(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/tutorial/town01_follow_to_goal.xosc`.

## 1. A scenario directory

```bash
mkdir -p ~/my_scenarios
```

Scenario files are never modified by a run; results go wherever you point `output_directory`.

## 2. Find poses on the map

Scenarios place entities in the ROS frame of the Autoware map. For the CARLA towns that is
CARLA's world with y flipped and headings negated, which is easy to get wrong by hand.
`carla_poses` reads the map CARLA has loaded (it only reads; it is safe beside a running
scenario) and prints positions already converted, as OpenSCENARIO elements:

```bash
ros2 run csb_launch carla_poses spawn-points
```

```
# CARLA 0.9.16 at localhost:2000, map Town01
   0  <WorldPosition x="335.49" y="-273.74" z="0.30" h="-1.57080" p="0" r="0" />  <!-- road 19 lane -1 -->
   1  <WorldPosition x="299.40" y="-133.24" z="0.30" h="0.00000" p="0" r="0" />  <!-- road 4 lane -1 -->
   2  <WorldPosition x="299.40" y="-129.75" z="0.30" h="-3.14159" p="0" r="0" />  <!-- road 4 lane 1 -->
   ...
```

(It needs CARLA's Python API, `pip install carla==0.9.16`; see the README's prerequisites.)

CARLA has to have the town loaded: run any Town01 scenario first (Tutorial 1), or check the
first line. Spawn point 2 starts on a long straight road heading west. Walk along its lane
for the lead car's start (20 m) and the goal (80 m):

```bash
ros2 run csb_launch carla_poses ahead 2 --distance 20
ros2 run csb_launch carla_poses ahead 2 --distance 80
```

```
start  <WorldPosition x="299.40" y="-129.50" z="0.00" h="3.14115" p="0" r="0" />  <!-- spawn point 2, road 4 lane 1 -->
end    <WorldPosition x="279.40" y="-129.49" z="0.00" h="3.14115" p="0" r="0" />  <!-- 20 m on, road 4 lane 1 -->
...
end    <WorldPosition x="219.40" y="-129.47" z="0.00" h="3.14115" p="0" r="0" />  <!-- 80 m on, road 4 lane 1 -->
```

`ahead` follows the lane and, at a junction, keeps the straightest branch (`--turn` takes the
sharpest, `--every` prints the way). `at X Y` snaps a point you picked elsewhere, for example
in RViz, onto the nearest driving lane. The `start` line is the lane centre at spawn point 2:
use it for the ego.

## 3. Write the scenario

Create `~/my_scenarios/follow_to_goal.xosc`. **Start from a copy of the shipped file** and
keep its frame -- `FileHeader`, `ParameterDeclarations`, `CatalogLocations`, and the
`Storyboard`'s `Story` / `Act` / `ManeuverGroup` / `Maneuver` nesting with the Act's own
`StartTrigger` (`SimulationTimeCondition` > 0) and the empty `StopTrigger` -- which
OpenSCENARIO requires and every scenario repeats. What you write yourself are four parts:

**The map**, by path. `$(env CARLA_MAPS)` lets the same file work on any machine:

```xml
<RoadNetwork>
  <LogicFile filepath="$(env CARLA_MAPS)/Town01" />
</RoadNetwork>
```

The directory's name is the CARLA town; the bridge loads Town01 into CARLA if another town is
loaded.

**The entities.** The ego is the vehicle whose controller is named `Autoware` (or has the
property `isEgo=true`); copy its block from the shipped file. The lead car has an empty
controller name, which means SSv2 drives it:

```xml
<ScenarioObject name="lead">
  <Vehicle name="lead" vehicleCategory="car">
    <BoundingBox>
      <Center x="1.5" y="0.0" z="0.9" />
      <Dimensions width="2.1" height="1.8" length="4.5" />
    </BoundingBox>
    <Performance maxSpeed="20" maxAcceleration="3.0" maxDeceleration="8.0" />
    <Axles> ... </Axles>
    <Properties />
  </Vehicle>
  <ObjectController><Controller name=""><Properties /></Controller></ObjectController>
</ScenarioObject>
```

The bridge spawns the CARLA vehicle closest in size to the declared box within its
`vehicleCategory`; a 4.5 x 2.1 x 1.8 m car keeps the Tesla Model 3. A `truck` gets a truck, a
`bicycle` a bicycle. To force a body, set `model3d="vehicle.audi.tt"` on the `Vehicle`.

**Init**: teleport both, give the ego its goal, start the lead car at a parameterized speed.
Paste the poses from step 2; the ego's goal is the `AcquirePositionAction`:

```xml
<Private entityRef="ego">
  <PrivateAction><TeleportAction><Position>
    <WorldPosition x="299.40" y="-129.50" z="0.30" h="3.14115" p="0" r="0" />
  </Position></TeleportAction></PrivateAction>
  <PrivateAction><RoutingAction><AcquirePositionAction><Position>
    <WorldPosition x="219.40" y="-129.47" z="0.00" h="3.14115" p="0" r="0" />
  </Position></AcquirePositionAction></RoutingAction></PrivateAction>
</Private>
<Private entityRef="lead">
  <PrivateAction><TeleportAction><Position>
    <WorldPosition x="279.40" y="-129.49" z="0.30" h="3.14115" p="0" r="0" />
  </Position></TeleportAction></PrivateAction>
  <PrivateAction><LongitudinalAction><SpeedAction>
    <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time" />
    <SpeedActionTarget><AbsoluteTargetSpeed value="$leadSpeed" /></SpeedActionTarget>
  </SpeedAction></LongitudinalAction></PrivateAction>
</Private>
```

with `<ParameterDeclaration name="leadSpeed" parameterType="double" value="4.0" />` at the
top. The `z` you give is not critical: the bridge places every actor on the ground under its
x/y.

**The verdict**: three events in one maneuver of the story, each ending the run.

| Event | Condition | Action |
|---|---|---|
| reached the goal | `ReachPositionCondition` on the ego, the goal pose, `tolerance="3.0"` | `exitSuccess` |
| touched the lead car | `CollisionCondition` between the ego and `lead` | `exitFailure` |
| too long | `SimulationTimeCondition` > 120 | `exitFailure` |

Each is a `UserDefinedAction` with a `CustomCommandAction type="exitSuccess"` (or
`exitFailure`) and a `StartTrigger` holding the condition; the shipped file has all three
written out. Simulation time starts when the ego is engaged, so the timeout does not count
Autoware's start-up.

## 4. Run it

The stack from Tutorial 1 (Terminals 1-3) stays up:

```bash
ROS_DOMAIN_ID=9 ros2 launch csb_launch scenario.launch.xml \
    scenario:=$HOME/my_scenarios/follow_to_goal.xosc output_directory:=/tmp/follow
grep -o 'failures="[0-9]*" errors="[0-9]*"' /tmp/follow/scenario_test_runner/result.junit.xml | head -1
# failures="0" errors="0"
```

In RViz the lead car appears ahead as a detected object, the ego slows to follow it at a
distance, and the run ends when the ego reaches the goal (about 35 s of simulation time).

Things to try: a different `leadSpeed`; a lead car that stops halfway (a second
`SpeedAction` to 0, triggered by a `SimulationTimeCondition`); the goal moved with
`carla_poses ahead 2 --distance ...`. Each is one edit and one more run.

## 5. Rules worth knowing

From [Writing scenarios](../design/scenario-authoring.md), the ones a first scenario meets:

- **SSv2-driven NPCs never get pushed.** They are placed exactly where SSv2 says every frame;
  a collision does not move them. For vehicles that drive with physics, name the controller
  `simulator_autopilot` (CARLA's Traffic Manager) or `agent` (another Autoware, [Tutorial
  4](04-demo-run.md)).
- **Uncommanded traffic lights are green.** CARLA's own light cycling is frozen; command
  the lights your scenario depends on.
- **Do not end a run on the collision frame.** Give a collision condition a `delay` of a few
  seconds if you want CARLA to have registered the contact too ([Tutorial 3](03-run-and-judge.md)).
- **The ego takes goals and speed limits, not lane changes.** SSv2 refuses relative-speed
  and lane-change actions for an ego by design.

Next: [Tutorial 3, run and judge](03-run-and-judge.md).
