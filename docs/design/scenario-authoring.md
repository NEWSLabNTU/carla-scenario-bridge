# Writing Scenarios for an Unmanaged Ego

A scenario that runs against `managed_ego:=true` (the default, and what phase 012 shipped)
is an ordinary SSv2 scenario and nothing here applies to it. This document is about
`managed_ego:=false`, where SSv2 does not drive the ego's autonomy at all — see
[architecture.md](architecture.md#5-ssv2-drives-the-scenario-autoware-is-external) for what
that means and why the option exists.

The short version: **the ego is a vehicle the scenario observes, not one it commands.**

## The rules

### 1. No ego autonomy actions

The concealer is inert, so there is nothing to carry these out. They are not ignored —
they are hard errors, on purpose, at the moment the storyboard reaches them:

```
clearRoute cannot be requested because the ego vehicle is not managed by
scenario_simulator_v2 (the parameter managed_ego:=false was given).
```

That covers `AcquirePositionAction` and `AssignRouteAction` on the ego (both route through
`clearRoute`), engage, `enableAutowareControl`, `setVelocityLimit` and the cooperate
commands. Failing fast is deliberate: the alternative is a scenario that appears to run
while the ego quietly does something else.

`setVelocityLimit` is the one exception. It is called unconditionally by
`applyAssignControllerAction` — every scenario with a controller hits it, whether or not it
asks for a speed limit — so it is inert rather than fatal, and an unmanaged run simply has
no velocity limit.

### 2. No ego-state conditions

`UserDefinedValueCondition`s that read ego state keep their stock behavior, which for an
inert concealer means pinned values (`"INITIALIZING"`, `""`). Engage, MRM and emergency
conditions therefore never fire. There is no error for this — the condition just never
becomes true — so do not use them.

Everything the simulator feeds is unaffected and is what these scenarios should be written
against: pose, distance, speed, collision, time, and traffic-light conditions.

### 3. The goal lives in the pilot's poses file

With no `AcquirePositionAction`, the ego's destination comes from
`EGO_GOAL_POSES_FILE` — `acb_pilot` format, `goal_pose` in the map frame:

```yaml
goal_pose:
  x: 88.4
  y: -100.0
  z: 0.0
  qx: 0.0
  qy: 0.0
  qz: -0.7071
  qw: 0.7071
```

The pilot reads **only** `goal_pose`. An `initial_pose` key is accepted by other tools in
that format but ignored here; the ego's start pose comes from the scenario's
`TeleportAction`, and localization finds it from GNSS.

This means the goal is stated twice whenever the scenario also wants to detect arrival —
once in the poses file, once in the `ReachPositionCondition` that ends the run. Keep them
in sync; `scenarios/town01_unmanaged.xosc` and `scenarios/ego_poses.yaml` are the worked
example, both naming (88.4, -100.0).

### 4. Budget scenario time for the pilot

A managed ego is engaged by the concealer before the storyboard starts moving. An unmanaged
one is engaged by the pilot, which localizes, waits out a stabilization period, routes and
engages **inside** scenario time — about 95 s before the ego moves at all. A timeout copied
from a managed scenario will fire before the ego has driven anywhere.
`town01_unmanaged.xosc` allows 600 s where its managed sibling allows 300 s.

## Running one

```bash
EGO_MANAGED=false EGO_GOAL_POSES_FILE=$PWD/scenarios/ego_poses.yaml just ego-av
# wait for 'Startup complete'
EGO_MANAGED=false just scenario $PWD/scenarios/town01_unmanaged.xosc
```

`EGO_MANAGED` is an environment variable rather than a `just` parameter because just's
parameters are positional: `just ego-av managed=false` would silently assign
`"managed=false"` to `map_path`. It must be set for **both** commands — the ego stack uses
it to pick its domain, and the scenario uses it to pass `managed_ego` to the interpreter and
to check the right domain for a healthy stack.

Starting the scenario first is refused rather than left to time out: `just scenario` runs
`scripts/ego_stack_health.py`, which for an unmanaged run also requires the pilot to be
running, since nothing else would ever route the ego.

## What a good unmanaged scenario looks like

`scenarios/town01_unmanaged.xosc`: teleport the ego to a start pose, set a traffic light,
end on `exitSuccess` when the ego reaches a position, and `exitFailure` on a generous
timeout. No routing action, no engage, no ego-state condition — the ego drives because a
pilot is driving it, and the scenario only observes and scores.

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
