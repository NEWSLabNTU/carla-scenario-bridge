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

`src/isaac_world/spikes/terrain_hooks.py` on Isaac Sim 6.1 (omni.physx.vehicle 110.3.2), aib's
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

- [x] **D1. Where the surface map lives** (gates S1). **Decided 2026-10-22 (user): a
  simulator-neutral `<map>/surfaces.yaml` as the source of truth, with raster tiles allowed
  per zone for gradients** (options B + C of five compared: USD-authored materials, the
  YAML, raster tiles, Lanelet2 tags, OpenDRIVE `<material>`). The file holds semantic-label
  defaults for the whole town (Road -> asphalt, Terrain -> grass, ...), zones as map-frame
  polygons with stable ids and a class (`soil: true` for S3), and per zone an optional raster
  (water depth, soil variant) with origin and resolution; per-class parameters sit in one
  shared class table. Why: a CARLA town gets a surface map from its SimReady labels with no
  painting; zones are reviewable text with the ids D3's scenario properties name; it survives
  a town re-export; both world servers read it (CARLA: zones as friction trigger boxes).
  Rejected: USD-authored materials (lost on re-export, unreviewable, Isaac-only), Lanelet2
  tags (puts Autoware's map at risk, lanes only), OpenDRIVE materials (CARLA-only, lanes
  only, and CARLA's towns carry none). USD materials become the world server's generated
  output. Needs a plot of the zones over the map's top view (`just surfaces-plot <map>`).
- [ ] **D2. Soil model implementation** (gates S3): our own SCM-style node loop in Python/warp
  inside the world server [proposed: Chrono SCM cannot drive a PhysX raycast wheel]; or Chrono
  as a library with its SCM terrain and a per-wheel rigid "tire".
- [x] **D3. Scenario format for zones** (gates S2). **Decided 2026-10-23 (user): the
  standard `RoadCondition`, `frictionScaleFactor` and `wetness` world-wide, per-zone state as
  its `Properties` named `swb.zone.<id>.<field>`** (option A of four compared: the
  properties, a `CustomCommandAction` built-in, one property naming a preset in
  `surfaces.yaml`, a launch argument outside the scenario). Why: the SSv2 fork only forwards
  OpenSCENARIO's `Environment` as name/value pairs and knows nothing of swb, so the change
  could go upstream (it fills the interpreter's empty `EnvironmentAction`); the file stays
  schema-valid and other tools ignore the properties; parameters (`$depth`) work in property
  values, so a distribution can sweep a ford's depth; a story event changes the state
  mid-run. A `RoadCondition` is the whole road state: it replaces the last one, and zones it
  does not name are as the map has them. The world server checks each `swb.` property
  against the loaded map and refuses an unknown zone, field or value with every problem
  listed, and SSv2 fails the scenario with that text. Rejected: a `CustomCommandAction`
  (swb-specific code in the fork, a free-form string, and a mistyped command runs as a shell
  command), presets alone (no sweeps; can be added later as one more property over the same
  path), a launch argument (no mid-run change, and the scenario no longer reproduces the run
  on its own).
- [ ] **D4. Physics rate in zones** (gates S3): raise the world's physics rate while a wheel is
  in a zone, or keep 60 Hz and integrate along the contact patch; decided by S3's measurement.
- [ ] **D5. Rut persistence across runs** (gates S4): fresh per scenario by default, with
  `swb.ruts` naming a saved set [proposed]; or always fresh.
- [ ] **D6. Mud** (gates S4): ship an engineered mud model marked unvalidated [proposed], or
  limit zones to sand and dry soil until a validation source exists.

## Steps

- [x] Survey (above).
- [x] Isaac spike: which hooks Isaac 6.1 honours (above).
- [x] **S1. Surface layer in aib and the world server.** A friction table per wheel in
      `aib.isaac.vehicle`; `<map>/surfaces.yaml` (D1) loaded by the world server: label
      defaults, zones, per-zone rasters, a lint and `just surfaces-plot`; per-class friction and slip-curve shape (gravel flat or rising, asphalt
      falling); `mu_eff = mu_class(wetness) * f_hydroplane(v, depth)` written per wheel per
      step, blended over a contact patch at class edges. Measured: per-wheel write cost; the
      ABS/TC scenario set below on one town.
  - [x] A table per wheel in aib (`/World/vehicleTireFriction/<vehicle>/<wheel>`, no ground materials;
        `Vehicle.set_wheel_friction`, `Vehicle.contacts`: each wheel's hit point and collider).
        Checked first in Isaac (2026-10-23): a table's `defaultFrictionValue` edit takes the
        next step, 0.05 ms a write; a tire's `frictionVsSlipGraph` edit is refused once the
        simulation runs, so a class's slip curve goes through the friction value instead
        (`class(slip) / vehicle(slip)`, one step late).
  - [x] `isaac_world.surfaces` (pure, unit-tested: `test_surfaces.py`, 23 tests): the
        surfaces.yaml loader that lists every problem, label defaults, zones with fields
        that fall through (later zones on top), PNG/.npy rasters, the class table
        (`src/isaac_world/config/surface_classes.yaml`: ten classes, literature values),
        wetness and water depth, hydroplaning (Horne, engineered), the distance filter, the
        slip-curve emulation. `just surfaces-check` / `just surfaces-plot [map_path:=]`.
  - [x] `isaac_world.surface_layer` in the world server: SimReady label of the hit collider,
        zone at the hit point (map origin applied), written per wheel after every substep;
        reset on teleport. The map is `surfaces.yaml` beside a `--world` world file, checked
        before Isaac starts, looked for and read again at every `LoadWorld`; no map, no writes.
  - [x] On a pad through the world server's code (`spikes/surface_drive.py`, design doc
        "Surface layer, measured"): every case moves the expected way (wet, gravel, ice,
        dry-to-wet step, mu-split yaws toward the dry side, ABS shorter than locked on asphalt
        and longer on gravel, hydroplaning at 25 m/s and not at 15, TC holds on ice); 21 us a
        step for four wheels, no frame-time change.
  - [x] On Town01 (2026-10-23, newslab-server-241, swb `fa9123f` + this step, aib
        `df94015`, acb `8f72488`, all rebuilt; design doc "Surface layer on Town01"):
        - scripted (`spikes/surface_town.py`): the surface cases at speed on the town's own
          road (lanelet 13397); its colliders resolve to `Road`, and every number matches the
          pad's within 0.01 m. No surface layer and a dry map give the same stop, 24.73 m.
        - Autoware, `just acceptance town01_ego_drive.xosc 3 backend:=isaac worlds:=Town01`,
          three sets on one world server: A no map, B a dry map (label defaults, a zone off
          the route), C the route crossing wet asphalt, a puddle under the left wheels,
          gravel, and its goal on ice. 9/9 PASS, 0 MRM; longitudinal A 0.013-0.018, B
          0.007-0.018, C 0.010-0.016 m/s^2 (045: 0.008-0.021), cross-track 0.03-0.04 m.
          The world server's log shows each wheel taking each zone at its edge, the puddle
          on the left wheels only. The route peaks at 4.3 m/s and brakes gently, inside even
          ice's grip, so C shows the plumbing and that nothing regresses, not surface
          effects on Autoware.
        Surface maps of the runs: `scenarios/surfaces/town01_{dry,route}.yaml`, copied to
        `<map>/surfaces.yaml` for a run (maps stay in git-ignored `data/`). A surfaces.yaml
        added or changed while the world server runs counts from the next `LoadWorld`.
- [x] **S2. Road condition from the scenario** (D3; design doc "The map says where, the
      scenario says what state"). Done 2026-10-23:
  - [x] SSv2 fork (`managed-ego-unforked`): `EnvironmentAction` sends its `RoadCondition` as
        `UpdateEnvironment` (oneof 17), throws on a refusal; a missing `wetness` is dry, and an
        `Environment` without `TimeOfDay` or `Weather` is no longer refused (neither type
        could be default-constructed: `Environment requires class TimeOfDay as element`).
        The runner now logs the preprocessor's refusal, which it had dropped: the first runs
        ended in a second with no word of why.
  - [x] Bridge: `UpdateEnvironment` to `SetRoadCondition` (world protocol request 14,
        `CAPABILITY_ROAD_CONDITION`, no version bump), `swb.` properties only; the default
        condition passes on a server without the capability. Fake backend, remote-backend
        tests, `swb_world_check` (three checks).
  - [x] Isaac: `surfaces.road_condition` (every problem listed), `SurfaceLayer.set_condition`,
        back to the map at `LoadWorld`, a new owner and `Release`; a world with no surface
        map takes the world-wide state.
  - [x] CARLA: tire friction x `frictionScaleFactor` x asphalt's wet factor on every vehicle
        (own value kept, reapplied at spawn), the weather's `wetness`/`precipitation_deposits`
        for the level and the old weather back at `Initialize`; `swb.` properties refused.
  - [x] Measured 2026-10-23 (`scenarios/surfaces/town01_{wet,zones}.xosc`: town01_ego_drive
        with an Init condition and a Story one at 20 s):
        - conformance: Isaac 22/22, CARLA 23/23, the road checks included.
        - Isaac, over `town01_route.yaml`: zones PASS; the Init property took the puddle from
          the map's 6 mm to 3 mm (left wheels 0.715, right 0.837), and at 20 s `ice_at_goal`
          became `snow_packed` under the wheels. Wet PASS; asphalt 0.810 while moist at scale
          0.9 (0.9 x 0.9), lower after `wetWithPuddles`, and the puddle back at the map's 6 mm
          (a condition is the whole road state).
        - CARLA: wet PASS (second run; the first ended with Autoware still PLANNING on a
          freshly started ego stack), tire friction x0.810 then x0.675, wetness 40 then 80
          with deposits 40. Zones refused, SSv2's verdict carrying the world server's text:
          "the CARLA world server has no surface map: zones and other swb properties are the
          Isaac world server's (refused: swb.zone.puddle_left.water_depth)".
  - [x] S1 fix found on the way (aib `e91e8bc`, aib `docs/issues/001`): the per-wheel tables
        sat under the vehicle, and PhysX refuses to remove a friction table while playing, so
        every ego despawn logged one error per wheel and leaked the tables. They now live at
        `/World/vehicleTireFriction/<vehicle>/<wheel>` and stay: two conformance runs and a
        zones run afterwards without the error, the per-wheel friction as before.
- [x] **S2b. Weather and time of day from the scenario** (asked 2026-10-23, after the Isaac
      probe; design doc "Weather and time of day from the scenario"). Done 2026-10-23:
  - [x] SSv2 fork: `UpdateEnvironment` carries `Weather` (each part optional) and `TimeOfDay`.
        Upstream refused any Environment missing an optional part whose type cannot be
        default-constructed (`TimeOfDay`, `Weather` and every one of its attributes and
        elements, a Fog's `BoundingBox`, `RoadCondition`); all optional now.
  - [x] Bridge: every part checked, `dateTime` to clock time and day, `SetEnvironment`
        (request 15, `CAPABILITY_ENVIRONMENT`), capabilities checked before any part is sent;
        conformance checks a clear noon.
  - [x] Isaac (`isaac_world.sky`): the scenario's sun takes over the world's (time of day over
        the map's latitude, or placed), domes dimmed at night, clouds as dimming, RTX fog,
        lidar rain, snow refused. CARLA (`scenario_weather.rs`): cloudiness, precipitation,
        fog, wind, the sun; snow refused.
  - [x] Measured 2026-10-23: `spikes/sky_world.py` (a world with its own sun and dome):
        the town's sun off and ours on at 3000 at noon, 1425 at 07:00, 0 at 21:30 (dome 800 to
        24, camera 208 to 20), 600 under 8 oktas, snow refused, the town's lights back after a
        reset. Conformance: Isaac 23/23, CARLA 24/24. `scenarios/surfaces/town01_weather.xosc`
        (07:00, rain 5 mm/h, fog 300 m, 2 oktas; at 20 s 21:30, dry, clear) PASS on Isaac (sun
        east 14 deg up at x0.38, then off, dome x0.03; the rain warning logged) and on CARLA
        (cloudiness 25, precipitation 20, fog density 70, then 0 and 0); town01_zones on Isaac
        and town01_wet on CARLA PASS on the same builds. After the runs CARLA held the wet run's
        look over the weather from before: the weather run's sky was undone at Initialize.
        Not checked: CARLA's sun azimuth reference.
      Found on the way: on a loaded host the simulation side's play_launch wrapper ran three
      minutes before the real one, `isaac_up.sh` timed out at 120 s and the next run started a
      second simulation side whose bridge fought the first for the world server. Both bring-up
      scripts now count the wrapper as the stack, and Isaac's bridge wait is 300 s.
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

- [x] Surface layer: on one town, a dry-to-wet step, a one-side puddle (mu-split: the ego yaws,
  ABS holds it), gravel braking (ABS no shorter than locked), an ice patch, and speed rising
  toward hydroplaning each behave in the expected direction, with numbers recorded; 0 MRM on
  the dry baseline (`town01_ego_drive` unchanged within 045's spread).
- [x] Scenario path: a `RoadCondition` in the Init and one in the Story change the ego's
  measured friction on Isaac, and CARLA's weather wetness; a zone property on CARLA is refused
  in the reply, not ignored.
- [ ] Soil layer: in a sand zone, sinkage grows with slip and load, a second pass sinks less,
  lateral grip falls, motion resistance outgrows thrust as sinkage deepens (the survey's
  trends, within ±30 % of the calibrated model); zone entry at speed shows no force step
  larger than the blend allows; the frame budget holds with one ego in the zone.
- [ ] Map size: a full CARLA town with one zone runs at the same frame time as without it
  when the ego is outside the zone.
- [ ] Lidar sees the ruts the wheels feel (rut depth in the point cloud within one grid cell).
