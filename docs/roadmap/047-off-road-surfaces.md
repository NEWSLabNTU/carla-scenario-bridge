# Phase 047: Off-road Surfaces -- Road Conditions and Soft Soil in Zones

Rally-grade ground for the Isaac ego on CARLA-scale maps: wet asphalt, puddles and fords,
gravel, and deformable mud and sand where a map says so, with the road condition named in
the scenario file. Whole-world deformation is out of reach (the earlier Chrono study); this
phase gives every ground region a surface and confines soil physics to authored zones.
Design: [design/off-road-surfaces.md](../design/off-road-surfaces.md); survey:
[design/survey-localized-deformable-terrain.md](../design/survey-localized-deformable-terrain.md).

**Source**: user direction 2026-10-22 ("we focus on simulation physics ... deformation effect
on ground, such as sand and muddy road"; "do the muddy, sand, wet road simulation on parts in
the world ... enable highly fidelity off-road rally conditions but still support large CARLA
scale map"; "find the proper solution for Isaac Sim and see if we can specify the road
conditions in scenario files").
**Depends on**: [045](045-autoware-isaac-bridge.md) (the ego is aib's PhysX vehicle, its `ev`
powertrain with ABS and TC); [046](046-unified-world-interface.md) for the scenario path (the
road condition is a field of its world protocol v4, not of Isaac's private one).
**Feeds**: [040](040-ntu-offroad-field.md) (its "terrain materials" are this phase's surface
layer, its zones this phase's soil layer), [041](041-competition-director.md).

Not in this phase: NPCs and signals on Isaac (044, another agent), deformable ground on
CARLA (CARLA has no channel for it), particle or continuum soil at run time (offline
calibration only), weather rendering (046 carries `SetWeather`).

## Where it starts (2026-10-22)

| | today |
|---|---|
| ground | every surface one friction: aib's single `/World/vehicleTireFriction` table, no ground materials, `defaultFrictionValue = tire_friction` |
| tire | PhysX vehicle tire, one 3-point friction-vs-slip curve (grip falling to 0.8 at lock, 045) for every surface |
| soft ground | none; 040 plans friction zones only; [design/simulator-backends.md](../design/simulator-backends.md): "deformable terrain is experimental" |
| scenario | SSv2 parses `EnvironmentAction` / `RoadCondition` and does nothing (`start()`/`run()` are `LINE()`); `simulation_api_schema.proto` has no slot ([design/ssv2-feature-completeness.md](../design/ssv2-feature-completeness.md)) |
| weather | CARLA's ROS services only; Isaac `Unsupported` (046 adds `SetWeather`) |

## Survey (done 2026-10-22)

Four parallel literature tracks and one synthesis
([survey-localized-deformable-terrain.md](../design/survey-localized-deformable-terrain.md)):
terramechanics models; localizing deformation in engines (Chrono, Isaac/PhysX/Newton,
AGX, Vortex); off-road games (Spintires/MudRunner, BeamNG, rally titles, AAA snow); wet and
loose surfaces (friction models, hydroplaning, simulators' friction maps). Findings that set
the design:

- Every system that scales confines soil to authored regions, computes it only in boxes
  around the wheels, and stores it as a sparse height delta over immutable terrain; the rest
  of the world is a per-material grip model. Cost follows the driven corridor (~10 MB per km)
  and the wheels in contact, not the map.
- Real-time soil fidelity tops out at Bekker-Wong-Janosi (Chrono SCM): drawbar pull within
  ±20-30 %, sinkage worse. Chrono CRM runs 20-70x slower than real time for one wheel even
  with active domains; DEM and MPM worse. Those calibrate SCM offline ("virtual bevameter").
- No published model handles mud; ours will be engineered and documented as unvalidated.
- On roads, surface condition is a friction multiplier for wet asphalt but a different curve
  shape for gravel, snow and ice; water depth, not a wet flag, drives friction loss and
  hydroplaning.

## Isaac spike (done 2026-10-22)

`src/csb_isaac/spikes/terrain_hooks.py` on Isaac Sim 6.1 (omni.physx.vehicle 110.3.2), aib's
Model 3: results in [design/off-road-surfaces.md](../design/off-road-surfaces.md), "What
Isaac Sim 6.1 gives us, measured". Works: per-wheel material lookup, friction-table edits
while running (next frame), a table per wheel (per-wheel friction), `apply_force_at_pos` at
the contact (soil resistance, within 3 % of F/m), ruts by re-cooking a collision tile (7-25 ms
per 4 x 4 m tile). Does not: sinkage by the suspension frame, ruts by editing mesh points
without a re-cook, a custom tire or road-geometry component (not exposed), a heightfield (no
schema). So the "script path": the soil model runs in the world server and returns through
friction, force and re-cooked tiles.

## Decisions (for the user)

Each needs an answer before the step it gates; the design's proposal is in brackets.

- [ ] **D1. Where the surface map lives** (gates S1): per-map `<map>/isaac/world.yaml`
  `surfaces:` section, classes from the CARLA material of each exported mesh, zones as
  polygons [proposed]; or a georeferenced raster (0.5-1 m cells) for the off-road field.
- [ ] **D2. Soil model implementation** (gates S3): our own SCM-style node loop in Python/warp
  inside the world server [proposed: Chrono SCM cannot drive a PhysX raycast wheel]; or Chrono
  as a library with its SCM terrain and a per-wheel rigid "tire".
- [ ] **D3. Scenario format for zones** (gates S2): standard `RoadCondition` for world-wide
  `frictionScaleFactor` and `wetness`, per-zone state as `Properties` named
  `csb.zone.<id>.<field>` [proposed]; or a `CustomCommandAction`.
- [ ] **D4. Physics rate in zones** (gates S3): raise the world's physics rate while a wheel is
  in a zone, or keep 60 Hz and integrate along the contact patch; decided by S3's measurement.
- [ ] **D5. Rut persistence across runs** (gates S4): fresh per scenario by default, with
  `csb.ruts` naming a saved set [proposed]; or always fresh.
- [ ] **D6. Mud** (gates S4): ship an engineered mud model marked unvalidated [proposed], or
  limit zones to sand and dry soil until a validation source exists.

## Steps

- [x] Survey (above).
- [x] Isaac spike: which hooks Isaac 6.1 honours (above).
- [ ] **S1. Surface layer in aib and the world server.** A friction table per wheel in
      `aib.isaac.vehicle`; USD physics materials per surface class on the exported town
      meshes (D1); per-class friction and slip-curve shape (gravel flat or rising, asphalt
      falling); `mu_eff = mu_class(wetness) * f_hydroplane(v, depth)` written per wheel per
      step, blended over a contact patch at class edges. Measured: per-wheel write cost; the
      ABS/TC scenario set below on one town.
- [ ] **S2. Road condition from the scenario** (after 046's protocol v4, D3): SSv2 fork
      (`managed-ego-unforked`) sends `EnvironmentAction`'s environment through a new
      `simulation_api_schema` request (pushed to the fork before the pin moves); the bridge
      forwards it as a world-protocol request with a `ROAD_CONDITION` capability; the Isaac
      server applies it to S1; the CARLA server maps `wetness` to weather wetness and puddles
      (camera only in 0.9.16) and `frictionScaleFactor` to tire friction, and refuses zones in
      its reply. Fake backend and conformance check cover it.
- [ ] **S3. Soil layer in one zone** (D2, D4): SCM-style per-wheel model on a sparse tiled grid,
      allocated on first contact; traction cap through the wheel's table, resistance through
      `apply_force_at_pos`, ruts through re-cooked 2 x 2 m tiles (budgeted per frame); zone-edge
      blend over one to two wheel diameters. Measured: Python cost per wheel per step at the
      chosen rate, re-cook cost with RTX sensors on, whether RTX lidar sees the ruts.
- [ ] **S4. Persistence and the second pass** (D5, D6): node blocks saved per tile, loaded per
      scenario; every vehicle reads and writes the same grid in a fixed order (determinism);
      mud model if D6 says so.
- [ ] **S5. Calibration**: Chrono CRM offline as a virtual bevameter per soil class (plate
      sinkage, shear, single-wheel drawbar pull vs slip at our wheel); cone index from the field
      when 040 can measure it.
- [ ] **S6. Scenarios and exit runs** (below).

## Acceptance

- [ ] Surface layer: on one town, a dry-to-wet step, a one-side puddle (mu-split: the ego yaws,
  ABS holds it), gravel braking (ABS no shorter than locked), an ice patch, and speed rising
  toward hydroplaning each behave in the expected direction, with numbers recorded; 0 MRM on
  the dry baseline (`town01_ego_drive` unchanged within 045's spread).
- [ ] Scenario path: a `RoadCondition` in the Init and one in the Story change the ego's
  measured friction on Isaac, and CARLA's weather wetness; a zone property on CARLA is refused
  in the reply, not ignored.
- [ ] Soil layer: in a sand zone, sinkage grows with slip and load, a second pass sinks less,
  lateral grip falls, motion resistance outgrows thrust as sinkage deepens (the survey's
  trends, within ±30 % of the calibrated model); zone entry at speed shows no force step
  larger than the blend allows; the frame budget holds with one ego in the zone.
- [ ] Map size: a full CARLA town with one zone runs at the same frame time as without it
  when the ego is outside the zone.
- [ ] Lidar sees the ruts the wheels feel (rut depth in the point cloud within one grid cell).
