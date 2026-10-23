# Phase 049: One Vehicle Spec -- Autoware, CARLA, Isaac and SSv2 on the Same Car

Make one spec per vehicle the single source of the car: geometry, physics and sensors. Then
generate from it everything that Autoware, CARLA, Isaac Sim and SSv2's scenarios read about
the car, and check at build time, at spawn and in behaviour that they agree. CARLA gets the
spec's physics, so both simulators drive the same Model 3. Design:
[design/vehicle-side.md](../design/vehicle-side.md), "The spec as the single source" and
"Consistency, end to end".

**Status**: draft, 2026-10-23. Waiting on the design's decisions V2 and V3.

**Source**: user direction 2026-10-23:

- "think about the SSoT of vehicle dynamics, physics, sensor setup for Autoware, simulator
  and SSv2 and see how to ensure consistency";
- "CARLA get the spec's physics";
- Autoware without SSv2 on both simulators, which this phase keeps.

**Depends on**: [048](048-vehicle-side-profile.md) (the vehicle layer and its repository).
Step S1 does not depend on it.
**Feeds**: [047](047-off-road-surfaces.md), whose road friction then scales the spec's tire
friction on both simulators; [039](039-isaac-multi-autoware.md), whose several egos each
name a spec; any later vehicle.

## Where it starts (2026-10-23)

The design's "What exists" table, in short:

- aib's `vehicles/tesla_model3.yaml` is the single source for Isaac and for Autoware there.
- acb's Autoware files and sensors agree with it only by hand.
- CARLA drives the blueprint's own physics.
- 130 of 132 inline ego definitions in the scenarios carry a box 29 cm short and 31 cm too
  tall, in four different `Performance` sets, and no scenario uses a catalog.
- The sensing launch exists twice, the same apart from comments.

## Steps

- [ ] **S1. Correct the scenarios' ego box** (V3). Do it now, before the catalog: set the
  box to the car's 4.792 x 2.163 x 1.488 m, centre (1.532, 0, 0.744), and the axles to
  3.005 m, track 1.667 m, wheel diameter 0.74 m, in all 132 definitions. Run the CARLA
  suite before and after and record what moved (distances, TTC, contact times).
- [ ] **S2. Spec version 2** in the vehicle layer.
  - Move the format, `aib.spec` and `aib.gen` from aib with history.
  - Add the `carla:` section and the SSv2 catalog output.
  - Version 1 files are read with a warning, for one release.
- [ ] **S3. Per-vehicle Autoware packages** (V2), generated:
  - `tesla_model3_description` and `tesla_model3_launch`, with an empty vehicle interface;
  - `tesla_model3_sensor_kit_description` and `tesla_model3_sensor_kit_launch`, whose
    sensing pipeline comes from the spec's sensor list;
  - the `individual_params` entry.

  acb and aib stop shipping `acb_vehicle`/`aib_vehicle` and their kits. Their wrappers
  pass `vehicle_model:=tesla_model3` (old names: one release, a warning).
- [ ] **S4. CARLA on the spec's physics.**
  - Generate `carla_physics.yaml` and `carla_sensors.yaml`.
  - `carla_world` applies the physics file at the ego's spawn, and `demo_scenario.py` does
    standalone. Both only copy fields.
  - acb_bridge reads the physics back on attach and refuses a mismatch, naming the fields.
  - carla_world's road friction scales the spec's friction, not a cached first read.
  - The lags CARLA cannot express go into acb_bridge's command path.
- [ ] **S5. CARLA's pedal maps, measured.** A calibration run on CARLA with the spec's
  physics writes `accel_map.csv` and `brake_map.csv` with the spec's physics hash.
  `check-generated` fails on a stale hash.
- [ ] **S6. The SSv2 catalog.** Generate `catalogs/vehicles.xosc` and replace every inline
  ego with a `CatalogReference`. Add a lint that rejects an inline ego.
  - The world servers resolve the entry name to the spec: carla_world to the blueprint and
    physics, isaac_world to the rig.
  - The bridge maps the catalog name for SSv2's `model`.
- [ ] **S7. The spawn check.**
  - `SpawnReply.body` gains the wheel layout.
  - For the ego, the bridge compares the box and axles with the declared ones and refuses
    past 2 cm, naming both.
- [ ] **S8. `vehicle_check`'s dynamics run**: acceleration steps, braking to a stop and a
  steering sweep, against the spec's limits and acceptance.py's 0.35 m/s². It runs
  without SSv2.
- [ ] **S9. Exit runs** (below).

## Acceptance

- [ ] **Build time**: `check-generated` passes in the vehicle layer, acb and aib, and the
  `.xosc` lint passes. Changing one spec value and not regenerating makes both fail.
- [ ] **Spawn time**: on both simulators, the ego's measured box and axles are within 2 cm
  of the catalog's. A scenario with a wrong ego box is refused with both boxes in the
  message. On CARLA, acb_bridge's read-back matches `carla_physics.yaml`.
- [ ] **The same car**: `vehicle_check`'s dynamics run on CARLA and on Isaac, without SSv2.
  Both stay within the spec's limits and the tracking limit. The table of the two
  simulators' step responses goes in the results, and so does the gap that CARLA's model
  leaves (ABS, regen).
- [ ] **Without SSv2**: acb's and aib's standalone runs drive to arrival, as in 048.
- [ ] **With SSv2**: `just e2e sim:=carla` PASS, and the CARLA suite against S1's numbers,
  with every change explained by the corrected box or the new physics. `just e2e
  sim:=isaac` PASS on server139.

Not in this phase: a second vehicle; the background AVs' and NPCs' physics (they keep
their blueprints); `aib.ros` as a shared publisher library.
