# Phase 052: Isaac Physics, Feature-Complete

Close what the Isaac ego's physics still lacks before any of it is calibrated against
measured ground: every surface class reachable from a scenario, every body on the soil
feeling it, nothing a finished scenario leaves behind in the next one's sensors, and the
ego's own dynamics measured the way CARLA's already are. Calibration against published
measurements is [053](053-terramechanics-calibration.md); this phase changes no soil
parameter.

**Status**: planned, 2026-10-24.

**Source**: user direction 2026-10-24: "Focus on feature completeness first. Suggest the
roadmap", then "do both phase 052 and dataset survey".

**Depends on**: [047](047-off-road-surfaces.md) (the surface and soil layers; issues
[001](../issues/001-soil-tile-rebuild-memory.md)-[003](../issues/003-world-reload-memory.md)
fixed). **Feeds**: [051](051-target-shape.md) (its "same car" table gets Isaac's column
before `vehicle_check` exists), [039](039-isaac-multi-autoware.md) (egos on soil),
[040](040-ntu-offroad-field.md), [053](053-terramechanics-calibration.md).

Not in this phase: anything 051 owns (`vehicle_check`, the spawn box check, the spec's
physics on CARLA); the NTU field's import (040); soil parameters (053); deformable ground on
CARLA.

## Where it starts

| | today |
|---|---|
| ego teardown | `IsaacWorld.despawn` deletes an ego's prims where it stands; NPCs and props are buried first (044 step 7: a prim deleted in place stays in the RTX lidar's scene) |
| ego dynamics | CARLA's step responses measured (051 K0, `scripts/carla_step_response.py`); Isaac's not |
| snow | `snow_packed` (friction only) used by `town01_zones.xosc`; `snow_loose` has Wong's US snow soil row but no zone, scenario or check uses it |
| NPCs on soil | an NPC vehicle lays ruts (`SurfaceLayer.set_kinematic`, 047 S4) but stands on the zone's flat box: it does not sink and does not follow ruts (issue 001) |
| several egos on soil | one ego per run measured; the soil grid's cost per extra ego unmeasured |

## Steps

- [ ] **1. Ghost egos.** Check whether a finished scenario's ego stays in the next one's
  lidar: two `town01_soil.xosc` runs on one world server, the second run's
  `/sensing/lidar/concatenated/pointcloud` searched for returns at the first ego's last
  pose. If it is there, bury the ego as `despawn` buries an NPC (kinematic, under the world,
  deleted after one step), keeping the slot path and the kept lidar (issue 003), and write it
  up as an issue.
- [ ] **2. Isaac's step responses.** `scripts/isaac_step_response.py`: the K0 protocol
  (throttle 0.3 / 0.6 / 1.0 from rest, brake 0.3 / 0.6 / 1.0 from 10.4 m/s, steering 0.05 /
  0.1 / 0.15 at 5 m/s) on aib's vehicle on flat asphalt, controlled directly, no Autoware.
  The CARLA and Isaac table goes into 051's progress as the baseline its "same car" check is
  held to. Repeat on sand and dirt (the soil's resistance on the step responses).
- [ ] **3. Snow from end to end.** A soil zone of `snow_loose` on Town01
  (`scenarios/surfaces/town01_snow.xosc` and its surface map), the ego driving through with
  Autoware. Compare the pad numbers (sinkage, resistance, pull-away) with Wong's for that
  row; mark the class `unvalidated:` like mud until 053 has a reference.
- [ ] **4. NPCs on the soil.** A tracked or kinematic NPC vehicle on soil stands at the
  grid's sinkage under its wheels and follows existing ruts (its pose's z and pitch/roll
  from the ruts under its four wheels), without plates of its own. Check with
  `town01_soil.xosc`'s lead car seen from the ego's camera and lidar.
- [ ] **5. Several egos on soil.** Two and four egos (039's multi-ego setup) through one soil
  zone: frame time against one ego, ruts each ego leaves felt by the others, the world
  server's memory over a batch.

## Acceptance

- [ ] No return from an earlier scenario's ego in a later scenario's lidar.
- [ ] The CARLA and Isaac step-response table in 051, Isaac's two runs identical within its
  noise.
- [ ] `town01_snow.xosc`, `town01_soil.xosc` and `town01_mud.xosc` PASS on Isaac with
  Autoware, 0 MRM, 4 runs each.
- [ ] A lead NPC on soil visibly sinks and rides in the ego's ruts.
- [ ] Frame time with four egos on soil recorded; the world server's memory flat over a
  batch, as issue 003 left it.

## Progress
