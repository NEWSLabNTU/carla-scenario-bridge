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
