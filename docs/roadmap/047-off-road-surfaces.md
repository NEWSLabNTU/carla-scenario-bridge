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
- [x] **D2. Soil model implementation** (gates S3). **Decided 2026-10-23 (user): our own
  SCM-style grid in the world server (option A), with a closed-form Wong-Reece wheel beside it
  (C)** (of four compared: A; PyChrono's `SCMTerrain` driven by a kinematic stand-in wheel per
  PhysX wheel (B); C alone; Isaac/Newton MPM (D, rejected: experimental, no PhysX vehicle
  coupling, ~4 s per frame)). Why: one physics engine in-process, the coupling a substep late
  through the channels the spike proved; no new dependency in Isaac's Python; the grid is ours,
  so the collision tiles and the lidar see exactly what the wheels felt, and every vehicle
  writes it in a fixed order. B's risks were a contact nobody has published (a rigid Chrono
  wheel standing in for a PhysX raycast wheel, two loads that disagree), packaging PyChrono for
  Isaac's Python, and following Chrono's API. C is the first-pass reference A is checked
  against and a zone's `soil: closed_form` (no memory). Chrono stays offline, for S5.
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
- [x] **D4. Physics rate in zones** (gates S3): raise the world's physics rate while a wheel is
  in a zone, or keep 60 Hz and integrate along the contact patch; decided by S3's measurement.
  **Answered 2026-10-23 by S3: keep 60 Hz and sweep** each wheel along its path one node
  spacing at a time inside the step: 60 Hz then reads as 600 Hz within 0.4 % at 25 m/s
  (`test_soil.py`), and a raised rate would multiply PhysX's cost for nothing.
- [x] **D5. Rut persistence across runs** (gates S4). **Decided 2026-10-23 (user): fresh per
  scenario by default; the scenario names a saved set with the `swb.ruts` property** (option
  B of four compared: always fresh; B; ruts carried over between scenarios until the server
  restarts or another map loads; a per-zone default set in `surfaces.yaml`). Why: every run
  starts the same (reproducible, and fair for 041's entrants), a worn stage is still one
  property in the scenario (and a parameter for sweeps), and the grid already saved and
  loaded its blocks. Carrying over was rejected (a run's start depends on the runs before it
  and on restarts); a map default can come later on the same file format. A set goes in only
  before any wheel has driven on the soil; a different one later is refused.
- [x] **D6. Mud** (gates S4), decided 2026-10-23: **B**, mud as a clay row in the soil table,
  marked unvalidated. Options were A (no mud soil zones), B (Wong's wettest clay row plus an
  `unvalidated:` flag and a map opt-in; no new physics), C (B plus rate softening, damping, a
  water-content friction cap and a clogging state per tire: engineered knobs with nothing to
  check them against) and D (a BeamNG-style resistive volume: no ruts, a second model).
  C's terms come one at a time only when a scenario needs them (clogging first).

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
- [x] **S3. Soil layer in one zone** (D2, D4): SCM-style per-wheel model on a sparse tiled grid,
      allocated on first contact; traction cap through the wheel's table, resistance through
      `apply_force_at_pos`, ruts through re-cooked 2 x 2 m tiles (budgeted per frame); zone-edge
      blend over one to two wheel diameters. Measured: Python cost per wheel per step at the
      chosen rate, re-cook cost with RTX sensors on, whether RTX lidar sees the ruts.
  - [x] `isaac_world.soil` (pure numpy, `test_soil.py`, 21 tests): `SoilGrid` (A: 16 x 16
        node blocks at 5 cm, float32, allocated on first contact; Bekker pressure with elastic
        reload, so a second pass rides in the rut; Janosi shear; Lyasko slip sinkage) and
        `wong_reece` (C). Soil parameters per class in `surface_classes.yaml` (sand, dirt =
        sandy loam, snow_loose; mud since D6, unvalidated); a zone's `soil: grid | closed_form`, refused on
        a class without them.
  - [x] D4 answered offline: the grid sweeps each wheel from its last position to its
        current one, one node spacing at a time, so 60 Hz reads as 600 Hz (sinkage within
        0.3 %, resistance within 0.4 % at 25 m/s); coarser sweeps lose 12-28 % of the
        resistance. The physics rate stays the world's.
  - [x] Surface layer: soil-limited friction into the wheel's table, resistance as
        `apply_force_at_pos` at the contact for the next substep, faded in over 1.5 wheel
        diameters from the zone edge; ruts fresh at every reset. aib's `Contact` carries the
        wheel's load and rolling direction (aib `deaef80`).
  - [x] On a pad (`spikes/soil_drive.py`, 2026-10-23; design doc "Soil layer on a pad").
  - [x] Cost: 3.7 ms per vehicle per substep in a zone with numpy (p50; p99 12.5 ms, host at
        load 68). The sweep is now one warp kernel on the CPU (`soil_warp`, the world
        server's; numpy stays for hosts without warp, `test_soil.py` checks they agree within
        0.2 mm): 141 us per wheel per step at 25 m/s, 1.29 ms per vehicle per substep in a zone
        on the pad (p50; p99 6.3 ms), ~4 ms of a 50 ms frame.
  - [x] Ruts in the collision and the render mesh: a zone with `ground_z` is the server's own
        1.6 m tiles (`soil_tiles`; the map has no collider in the zone's box), a rutted tile
        rebuilt as a new prim once no wheel stands on it, 4 a frame. Pad: the rear wheels ride
        in the front wheels' ruts (ground -7.1 cm), the second pass in its own (-9.9 cm, 37 %
        of the resistance); rebuilds 2.0 ms a frame (p50, max 6.4). Rebuilding under a wheel
        shook the car to a stop at 22 cm: hence the wait.
  - [x] RTX lidar sees the ruts (`spikes/soil_lidar.py`): 9.36 cm deep against the grid's
        9.43; four tiles rebuilt with the lidar rendering: 4 ms to edit, no frame-time rise.
  - [x] Memory (docs/issues/001): PhysX keeps every distinct mesh it cooks (~465 KB a 5 cm
        tile; no API frees it or re-cooks in place). Collision tiles at 10 cm (64 KB), render
        tiles at 5 cm edited in place, one rebuild per tile per pass: ~3 MB a pass on the pad
        against 23-36, ~100 MB per km of tracks. Mitigated, not gone.
  - [x] On a CARLA town, no asset step: at load a zone with `ground_z` cuts its box out of the
        town's meshes (`ground_cut`, `soil_tiles.cut_stage`; faces split at the edge, UVs and
        subsets carried), collision from new labelled copies; another load undoes it.
        Town01 (`spikes/soil_town.py`): 14 meshes in 0.8 s, rays meet the tiles, the second
        pass rides in the rut, frame time with the car outside the zone 2.91 ms against 3.07
        without the zone. Only faces in the ground band are cut (not a bridge's sides or a
        guardrail's uprights), an emptied mesh is hidden, the cut settles before an ego.
  - [x] Autoware on Town01 with the zone (design doc, "A zone on a CARLA town"; done by S6 on
        a quieter host, 2026-10-23): the soil
        worked (in C the ego slowed 4.1 -> 2.4 m/s in the dirt, Autoware pulled it back at
        its 1.0 m/s^2 limit), but every set failed on MRM flapping, the no-map baseline
        too, on a host at load 30-130 from other jobs. Rerun on a quiet host.
- [x] **S4. Persistence and the second pass** (D5, D6): node blocks saved per tile, loaded per
      scenario; every vehicle reads and writes the same grid in a fixed order (determinism);
      mud (D6: B).
  - [x] Rut sets (D5): `swb.ruts=<name>|fresh` in a scenario's RoadCondition loads
        `<map>/ruts/<name>.npz` (every grid zone's blocks and spacing) and shows it in the
        zones' tiles at once; refused with every problem listed for a map without a grid zone,
        a name that is not a set (the sets listed), a file that is not one, a zone the map
        does not have, another spacing, or a different set once the soil has been driven on.
        A RoadCondition without it leaves the ground as it is. When a scenario ends (another
        load, a new owner, a release) the world server saves its ruts, if any, to
        `<map>/ruts/runs/<time>.npz`; copied to `<map>/ruts/<name>.npz` it is a set. CARLA
        refuses the property as it refuses every `swb.*` one. Tests: `RutsTest` (4).
  - [x] On the pad (`soil_drive.py`, `SOIL_SPIKE_ONLY=rut`): two passes saved, a pass on fresh
        ground (as the first: 2354 N, ground 0), then the set loaded (0.17 s, its tiles
        rebuilt) and the pass again: the car rides in the saved ruts at -10.6 cm with 591 N,
        a third pass.
  - [x] Every vehicle on the same grid in a fixed order (2026-10-23): NPC vehicles roll on the
        zones' grids without slip on a quarter of their mass each (CARLA's wheel base, tread
        and radius for the body), kinematic ones from SSv2's pose, tracked ones from PhysX's,
        a frame's motion split over its substeps, a jump over 5 m a teleport; each substep
        the egos in handle order, then the NPCs in path order. NPCs mark the soil, they do
        not feel it. `KinematicTest` (4); on Town01 a lead car's ruts (7.5 cm in sand) are
        the ego's ground (S6).
  - [x] Mud (D6: B): `mud_wet` has Wong's clayey soil at 55 % moisture and `unvalidated:` (the
        reason); a soil zone on it is refused unless surfaces.yaml lists the class in
        `allow_unvalidated:`, and the world server and `just surfaces-check` warn at every
        load. Tests: `UnvalidatedTest` (3), a mud trend test in `test_soil.py`. On the pad:
        friction 0.17, 3.4 cm, 1.2 kN at 10 m/s, the second pass 351 N; the Model 3 cannot
        pull away from rest (dug in to 5.7 cm); ABS stops from 15 m/s in 30.8 m.
- [x] **S5. Calibration** (2026-10-23; design doc "Calibration against Chrono"):
      `spikes/crm_calibrate.py`, our grid against Chrono's single-wheel rig on our wheel at
      4.5 kN, slip 0-0.5, on Chrono SCM (same parameters: the implementation check) and CRM
      (the class's density, friction angle, cohesion: the virtual bevameter; pychrono 10 with
      CUDA). Static sinkage matches SCM within 1-16 %; Lyasko's slip sinkage was too strong
      (x1.7 at 50 % slip where SCM-treaded and CRM agree on x1.26-1.41), so a class carries
      `slip_sinkage` (sand 0.5, dirt 0.65), and Janosi K is fitted to SCM-treaded pull
      (x1.5): pull within -26..+16 % of SCM at 10-50 % slip. CRM's pull stays near zero for
      any wheel tried (a rigid tire's interface, not resolution: 1 cm agrees with 2 cm), so it
      is recorded, not fitted. Cone index and a real tire's drawbar test wait for the field
      (040); mud and snow have no reference.
- [x] **S6. Scenarios and exit runs** (2026-10-23; design doc "Scenarios and exit runs on
      Town01"): `scenarios/surfaces/town01_soil.xosc` (sand then dirt, a lead car laying ruts
      first, parameter `ruts`) and `town01_mud.xosc`, each with its surface map. Two runs a
      set on Isaac with Autoware, one world server: baseline 2/2 (0 MRM, 0.02 m/s^2), soil
      2/2 with the lead beside the ego's line (0.21-0.22), 1/2 with it on the line (one
      0.1 s MRM where the ego slid into its ruts at the sand's edge; 0.20-0.30), from a
      saved rut set 2/2 (0.12-0.14), mud 2/2 (0.08). After S5's fit the edges drew four short
      MRMs in two runs, where 9-10 cm ruts met the rigid road in a wall; the tiles now
      feather the ruts out over a zone's last metre (`soil_tiles.FEATHER`), and the next two
      runs had none at an edge, one 0.4 s MRM mid-sand in one of them, cause unknown (the ego
      pulled 0.2 m sideways into the lead's fresh ruts). This also closes S3's open
      Autoware run.

## Acceptance

- [x] Surface layer: on one town, a dry-to-wet step, a one-side puddle (mu-split: the ego yaws,
  ABS holds it), gravel braking (ABS no shorter than locked), an ice patch, and speed rising
  toward hydroplaning each behave in the expected direction, with numbers recorded; 0 MRM on
  the dry baseline (`town01_ego_drive` unchanged within 045's spread).
- [x] Scenario path: a `RoadCondition` in the Init and one in the Story change the ego's
  measured friction on Isaac, and CARLA's weather wetness; a zone property on CARLA is refused
  in the reply, not ignored.
- [x] Soil layer: in a sand zone, sinkage grows with slip and load, a second pass sinks less,
  lateral grip falls, motion resistance outgrows thrust as sinkage deepens (the survey's
  trends, within ±30 % of the calibrated model); zone entry at speed shows no force step
  larger than the blend allows; the frame budget holds with one ego in the zone.
  *2026-10-23: the trends hold (`test_soil.py`, the pad); entry at 20 m/s: largest step
  537 N against the blend's ~340 N for two wheels plus the steady ripple of 198 N; the
  frame budget holds on the pad (~4 ms soil, ~2 ms rebuilds of 50). S5: against the
  calibrated reference (Chrono SCM with a treaded wheel, slip sinkage where SCM and CRM
  agree) the fitted grid's sinkage is within 1 % (sand) and 16 % (dirt) at zero slip and its
  slip-sinkage ratio within 3 %; its drawbar pull within -26..+16 % at 10-50 % slip.*
- [x] Map size: a full CARLA town with one zone runs at the same frame time as without it
  when the ego is outside the zone. *2026-10-23, Town01: 2.88 / 2.91 ms p50 with the zone
  against 2.74 / 3.07 without (`spikes/soil_town.py`, physics only); with the whole stack
  and Autoware, 43.9-44.4 ms against 43.2-54.8 (set B against A; a loaded host).*
- [x] Lidar sees the ruts the wheels feel (rut depth in the point cloud within one grid cell).
  *2026-10-23 (`spikes/soil_lidar.py`): 9.36 cm in the point cloud, 9.43 cm in the grid.*

## Left after 047 (2026-10-23)

Every step and acceptance item above is done. What stays open is tracked elsewhere:

- [Issue 002](../issues/002-mid-sand-mrm.md): emergency stops on `town01_soil`. The trigger
  is known: one NDT scan at its iteration limit is a WARN, and in Autoware 1.5's graph that
  makes autonomous unavailable. Two came from the first plates leaving a wheel without ground
  at a zone's exit (fixed); what makes a lone scan hit the limit elsewhere is open.
- [Issue 001](../issues/001-soil-tile-rebuild-memory.md), fixed: the cooked collision tiles are
  gone (a flat box per zone and a plate under each ego wheel, nothing cooked after load).
- [Issue 003](../issues/003-world-reload-memory.md): the world server grows ~130-150 MB with
  every scenario's world load, with or without the plates.
- Calibration against the ground itself: cone index and a drawbar test of a real road tire on
  sand and on dirt, which is what settles CRM's and Bekker's disagreement on traction (design
  doc, "Still to measure"). Waits for the field (040).
- Mud runs on Wong's clay row unchecked against the book (`unvalidated:`, D6), opted into per
  map; snow has no soil row and no reference at all.

