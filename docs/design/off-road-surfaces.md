# Off-road surfaces: wet roads, mud and sand in parts of a large world

How swb's Isaac backend should give an ego rally-grade surfaces (wet asphalt, puddles,
gravel, mud, sand) without simulating deformable ground over a whole CARLA town, and how a
scenario file names the road condition. The literature and games survey behind it is
[survey-localized-deformable-terrain.md](survey-localized-deformable-terrain.md) (2026-10-22);
this document adds what Isaac Sim 6.1 itself allows, measured, and the scenario side.

**Status**: design and spike only (2026-10-22), roadmap [047](../roadmap/047-off-road-surfaces.md). Nothing here is built. Isaac Sim 6.1.0 (Kit
110.3.0, omni.physx / omni.physx.vehicle 110.3.2), aib `defb5d0`, newslab-server-241.

## The shape of the answer

Every system that scales does the same thing (survey, first section): soft soil lives only
in authored zones, is computed only in small boxes around the wheels, and is stored as a
sparse height delta over terrain that never changes; everywhere else a cheap per-material
grip model carries wet, gravel and wear. So two layers, one state:

| layer | where | what it is | cost follows |
|---|---|---|---|
| **surface layer** | the whole world | a surface class per ground region (asphalt, gravel, dirt, grass, sand, mud, snow, ice) plus a wetness / water-depth state; per wheel it gives a friction value and a friction-vs-slip shape | nothing: a table lookup per wheel |
| **soil layer** | authored zones only (a mud pit, a sand stretch, a ford) | a Bekker-Wong-Janosi (Chrono SCM-style) wheel model on a sparse grid allocated where wheels have been: sinkage, compaction, multipass hardening, motion resistance, ruts | the driven corridor (~10 MB per km, survey) and the wheels in contact, not the map |

Map size stops mattering because neither layer stores anything per square metre of town.

## What Isaac Sim 6.1 gives us, measured

`src/isaac_world/spikes/terrain_hooks.py` drives aib's ego (`aib.isaac.vehicle.Vehicle`, the
Model 3, `ev`) over a flat asphalt strip into a mud strip, each a USD `PhysicsMaterial`, and
pokes each hook while it runs (0.05 s frames of 3 physics steps, as aib's host steps it).

| question | answer | evidence |
|---|---|---|
| Per-wheel material lookup | **yes, free** | at the strip edge the front wheels read `mud` / mu 0.35 while the rears still read `asphalt` / 1.0 (`VEHICLE_WHEEL_STATE_GROUND_MATERIAL`, `_TIRE_FRICTION`) |
| Change friction while running | **yes, next step** | `PhysxVehicleTireFrictionTable.frictionValues` 1.0 -> 0.6 mid-run: the wheels on that table read 0.6 one frame later |
| Per-wheel friction override | **yes**, by giving each wheel its own table | left wheels on a second table set to 0.3 read 0.30 / 0.296 while the right wheels read 0.6 |
| A wheel's friction with no ground materials at all | **yes, next step, 0.05 ms a write** | a table per wheel with an empty `groundMaterials`: `defaultFrictionValue` 1.0 -> 0.3 / 0.6 read back one frame later; four writes 0.18 ms, a frame with four writes every substep no slower than one without (2.2 vs 2.9 ms, noise). Halving it mid-slide halved the deceleration (9.83 -> 5.09 m/s^2) |
| Change a tire's friction-vs-slip graph while running | **no** | `physxVehicleTire:frictionVsSlipGraph`: "modification not allowed once the simulation has been started. Changes will be ignored." Set before the start it works: a locked slide decelerates 9.83 m/s^2 with the graph ending at 0.8, 6.49 at 0.4 |
| Soil resistance as a force at the contact | **yes** | `apply_force_at_pos` of 1500 N rearward at each of the four contact points for 1 s, no drive correction: 7.76 -> 4.43 m/s, -3.33 m/s against -3.41 m/s from F/m alone |
| Sinkage by moving the suspension frame | **no** | `suspensionFramePosition` +5 cm on one wheel, held 3 s: jounce 0.097 -> 0.084, body height unchanged |
| Ruts by editing a collision mesh's points | **not picked up** | 5 cm ruts written into a 41 x 41-vertex tile (4 x 4 m, 3200 triangles): the wheels' ground hit stays at z 0.0 |
| Ruts by re-cooking that tile | **yes, 7-25 ms** | re-applying `CollisionAPI` or replacing the prim: ground hit -0.050 under both rear wheels, body 5 cm lower. Edit plus next step, two runs: re-apply 25 ms / 7 ms, replace 16 ms / 22 ms |
| A custom tire or road-geometry component (PhysX vehicle2's seams) | **not reachable** | `omni.physx.vehicle`'s bindings expose controller and steering helpers only; `omni.physx` exposes `get_wheel_state`, `get_vehicle_drive_state`, `set_wheel_rotation_*`, `set_vehicle_to_rest_state`. No callback into the tire or road query |
| A deformable heightfield | **no** | the PhysX USD schema (110.3.2) has no heightfield prim; Newton's MPM is experimental, couples only through MJWarp, never the PhysX vehicle |

So the survey's "script path" is the one Isaac 6.1 allows, and it is enough: everything the
soil model must return to the wheel has a working channel.

## The Isaac design

### Surface layer

- **The surface map is `<map>/surfaces.yaml`** (047 D1, decided 2026-10-22), read by the
  world server at load: semantic-label defaults for the whole town (SimReady labels every
  ground mesh Road, SideWalk, RoadLine, Terrain or Ground), zones as map-frame polygons with
  ids and a class, and per zone an optional raster for gradients (a ford's water depth):

  ```yaml
  classes: config/surface_classes.yaml        # friction, slip shape, soil parameters per class
  defaults: {Road: asphalt, RoadLine: asphalt, SideWalk: concrete, Terrain: grass, Ground: dirt}
  zones:
    - id: mud_pit
      class: mud_wet
      soil: true                              # the soil layer, not only friction
      polygon: [[120.0, -40.0], [160.0, -40.0], [160.0, -10.0], [120.0, -10.0]]
    - id: ford_1
      class: asphalt
      water_depth: raster/ford_1_depth.png
      origin: [300.0, 55.0]
      resolution: 0.25
  ```

  A zone polygon need not follow mesh boundaries: each wheel has its own friction table, so
  the server resolves the class per wheel from its ground hit (the hit shape's label, then
  point-in-zone, then the zone's raster) and writes the table, rather than relying on a
  material bound per mesh. USD physics materials, where used, are generated from the file,
  never authored.

  Every zone field but `id` is optional and falls through to the zone below (zones listed
  later lie on top), the label's default, then the world's road condition: a puddle zone
  that sets only `water_depth` keeps the gravel or asphalt under it. A zone with a raster
  and no polygon covers the raster's extent. A raster is a one-channel PNG (`scale`: metres
  per count) or a `.npy` of metres, row 0 at the top (+y) edge, `origin` its lower-left
  corner in the map frame. `just surfaces-check` refuses what the server would (unknown
  classes or labels, bad ids, self-crossing polygons, rasters without placement) and warns
  about zones that overlap while setting the same field, or lie off the Lanelet2 map;
  `just surfaces-plot` draws the zones over the lanes (`isaac_world.surfaces`).
- **The friction table holds the class's peak friction** (the survey's table: dry asphalt
  ~0.85, wet 0.5-0.7, gravel ~0.6, dry earth ~0.68, packed snow 0.2, ice 0.1; calibrate,
  do not copy). PhysX already does the per-wheel lookup, so mu-split (one side in a puddle)
  costs nothing.
- **Each wheel has its own table** (aib, `<vehicle>/tireFriction/<wheel>`, no ground
  materials: its `defaultFrictionValue` is the wheel's friction on anything), and the world
  server writes it after every physics step for the next:
  `mu = tire_friction * friction_scale * mu_class * wet_class(wetness) * f_hydroplane(v, depth) / mu_reference`.
  Relative to the reference class (asphalt), so on dry asphalt a wheel keeps the rig's own
  `tire_friction` and a map of asphalt drives exactly as no map. It follows a change of
  surface through a first-order filter in distance travelled, `blend_length` (0.2 m, about a
  contact patch), so a class edge is no step for ABS/TC to chatter on, yet stays per wheel.
- **The slip curve's shape** differs by class: gravel and loose snow flat or rising after the
  peak (a locked wheel is not worse than ABS there), asphalt falling. PhysX fixes a tire's
  `frictionVsSlipGraph` once the simulation runs (measured above), so the class's curve goes
  through the same friction value: `* class_graph(slip) / vehicle_graph(slip)` on the last
  step's longitudinal slip. PhysX applies the vehicle's graph on top, so the product is the
  class's curve, one physics step (17 ms) late.
- **Water**: a zone's `water_depth` (a number, or a raster) or the wetness's implied film
  (`wetness_depth` in the class table) picks the wetness level it reaches, and on paved
  classes (`hydroplaning: true`) a hydroplaning factor: Horne's onset speed for the tire
  pressure (6.35 sqrt(p kPa) km/h, 100 km/h at 250 kPa), grip falling from half that speed to
  `residual` (0.1) at it, scaled by how much of the full film (2.5 mm) stands. Engineered, not
  validated; loose and soft classes soak the water up (their wet factors) instead.
- **Where the server finds the map**: `surfaces.yaml` in the map directory of a world file
  the world server was started with (`--world`, `just isaac-world world:=...`), checked
  before Isaac starts and read again at every `LoadWorld` (an edit needs no restart). swb's
  `LoadWorld` sends only the USD, which the server maps back; 046's world protocol v4 should
  carry the map's surfaces itself. A world with no surface map writes nothing.

### Surface layer, measured on a pad (047 S1, 2026-10-23)

`src/isaac_world/spikes/surface_drive.py`: aib's Model 3 (`ev`, ABS and traction control on)
through `isaac_world.surface_layer` -- the world server's own code -- on one flat mesh labelled
Road, one lane per case, zones from x = 150; full braking demand from 20 m/s inside the zone.
The class values are the shipped table's literature numbers, so these check directions and
plumbing, not magnitudes.

| case | stop | decel (from v0 - 1 to 3 m/s) | written mu | |
|---|---|---|---|---|
| dry asphalt | 24.7 m | 8.81 m/s^2 | 1.0 (the rig's own) | baseline |
| dry, ABS off | 25.1 m | 8.63 | 1.0 | ABS shorter on asphalt |
| wet (`wetWithPuddles`) | 28.4 m | 7.56 | 0.75 | |
| gravel | 29.8 m | 7.17 | 0.65-0.66 (curve: grip builds with slip) | |
| gravel, ABS off | 27.9 m | 7.69 | 0.82 front (locked) | locking shorter on gravel |
| ice | 144.6 m | 1.38 | 0.11-0.12 | |
| dry braking into wet 10 m on | 27.5 m | ~8.9, then ~7.75 | 1.0 -> 0.75 | |
| 6 mm water under the left wheels | 27.3 m | 8.04 | left 0.54-0.65, right 1.0 | yaw -5.4 deg, 0.77 m toward the dry side, steering held straight |
| 10 mm water, from 15 m/s | 20.3 m | 5.90 | 0.55 (`highFlooded`, below onset) | |
| 10 mm water, from 25 m/s | 65.5 m | 5.45 | 0.24 at 25 m/s, 0.55 slower | hydroplaning |
| full throttle 3 s on ice | 1.4 m/s reached | | | driven-wheel slip <= 0.18: TC holds |

Cost: the layer's update, four wheels, 21 us p50 / 0.41 ms p99 per physics step (10 k
steps); a frame of three steps 2.05 ms with it, 2.26 ms without (noise).

### Soil layer (zones)

Per wheel in contact with a zone, every physics step, in the world server (Python first,
numpy/warp if the cost asks):

1. read the wheel's state from PhysX: ground hit, load (suspension force), spin, slips;
2. run an SCM-style node loop over the sparse grid cells under the contact patch (Bekker
   elastic/plastic pressure, Janosi shear on accumulated shear displacement, the per-node
   yield pressure that gives multipass hardening);
3. return through the channels measured above:
   - **traction cap**: the soil-limited friction into the wheel's own table;
   - **motion resistance** (compaction and bulldozing): `apply_force_at_pos` at the contact;
   - **sinkage and ruts**: the zone's collision is a set of small tiles (2 x 2 m at 5 cm, 1600
     triangles); a tile whose nodes moved more than a threshold is re-cooked, at most a few
     per frame and staggered, so the wheel and every other vehicle meet the rut.
4. at the zone edge, blend surface-layer and soil outputs over one to two wheel diameters.

Ruts persist as the sparse node blocks (`GetModifiedNodes`-style), keyed by tile, saved per
run, so a scenario can start fresh or on yesterday's ruts. Every vehicle, ego or NPC, reads
and writes the same grid in a fixed order inside the lockstep tick (determinism, 039).

The render mesh of a dirty tile is the same geometry, so RTX lidar and cameras see the ruts
the wheels feel. Mud is an engineered model (low friction angle, cohesion for undrained
strength, slip-rate softening, a clogging state); no published real-time model exists, so it
is documented as unvalidated.

### Still to measure

- **Physics rate in a zone.** At 25 m/s a wheel crosses a 5 cm cell in 2 ms; the survey asks
  ~500 Hz in zones. aib steps 60 Hz with PhysX's own vehicle substeps inside. Measure the
  per-step Python cost at 300-600 Hz, or integrate along the patch analytically.
- **Re-cook budget.** 7-25 ms per 4 x 4 m tile, two runs on a loaded host; 2 x 2 m
  tiles and a per-frame cap need measuring against the 50 ms frame with RTX sensors on.
- **RTX cost of a changed mesh** and whether RTX lidar sees displacement maps or only
  geometry.
- **Calibration**: Chrono CRM offline as a virtual bevameter per soil class, cone-index from the
  field; judge on trends (survey, last recommendations).

## Road condition in scenario files

### What the standard gives

OpenSCENARIO 1.x has a road condition, but for the whole world:

```xml
<GlobalAction>
  <EnvironmentAction>
    <Environment name="wet">
      <TimeOfDay animation="false" dateTime="2026-10-22T14:00:00"/>
      <Weather .../>
      <RoadCondition frictionScaleFactor="0.7" wetness="wetWithPuddles">
        <Properties>
          <Property name="..." value="..."/>
        </Properties>
      </RoadCondition>
    </Environment>
  </EnvironmentAction>
</GlobalAction>
```

`wetness` is `dry | moist | wetWithPuddles | lowFlooded | highFlooded` (1.2+; highFlooded is
"water depth > 5 cm"). There are no zones; `Properties` is free-form name/value.

### What reaches the simulator today: nothing

SSv2 parses `Environment`, `Weather` and `RoadCondition`
(`openscenario_interpreter/syntax/road_condition.hpp`: `friction_scale_factor`, `wetness`,
`properties`), but `EnvironmentAction::start()` and `run()` are `LINE()` no-ops, and
`simulation_api_schema.proto` has no message for it
([ssv2-feature-completeness.md](ssv2-feature-completeness.md), section 2: "parse-only no-op",
"no slot"). swb's weather comes only from its ROS services (`/scenario_world_bridge/set_weather`).

### Proposal: the map says where, the scenario says what state

- **Where** (static, per map): `<map>/surfaces.yaml` (D1, above); both world servers read
  it, the CARLA one turning zones into friction trigger boxes.
- **What state** (per scenario, can change mid-run): the standard `RoadCondition`.
  `frictionScaleFactor` and `wetness` apply world-wide (wetness maps to a water depth and a
  friction factor per class); per-zone state rides in `Properties`, named by the map's zone
  ids, so the file stays schema-valid and any other tool ignores what it does not know:

  ```xml
  <RoadCondition frictionScaleFactor="1.0" wetness="moist">
    <Properties>
      <Property name="swb.zone.ford_1.water_depth_m" value="0.12"/>
      <Property name="swb.zone.mud_pit.soil" value="mud_wet"/>
      <Property name="swb.ruts" value="fresh"/>   <!-- or: keep, or a saved rut set -->
    </Properties>
  </RoadCondition>
  ```

  An `EnvironmentAction` in the Init sets the start state; one in the Story (on a time or
  distance trigger) changes it, which is how a scenario makes rain start or a ford rise.

What that needs, in order (roadmap [047](../roadmap/047-off-road-surfaces.md), step S2, after
[046](../roadmap/046-unified-world-interface.md)'s world protocol v4, which every simulator
speaks):

1. **SSv2 fork** (`managed-ego-unforked`, ours): `EnvironmentAction::start()` sends the
   environment to the simulator through a new `simulation_api_schema` request (an
   `UpdateEnvironmentRequest` with the road condition and its properties; weather and time of
   day can ride along later). Pushed to the fork before the pin moves (CLAUDE.md).
2. **Bridge**: the proto copy, the coordinator handling it, and a world-protocol request
   for it with a `ROAD_CONDITION` capability; the fake backend and the conformance check
   cover it.
3. **Isaac world server**: applies it to the surface layer (friction tables) and the zones.
4. **CARLA world server**: what CARLA can do: `wetness` to the weather's wetness and puddles
   (camera only in CARLA 0.9.16), `frictionScaleFactor` to the vehicles' tire friction or
   friction trigger boxes; zones refused in the reply.

Autoware is not told: a real car does not know the friction either. Its ABS/TC (in aib's
powertrain) and its planner meet the surface as a driver would.

## Next steps

Roadmap [047](../roadmap/047-off-road-surfaces.md): surface layer, road condition from the
scenario, soil layer in one zone, persistence and the second pass, calibration, the ABS/TC
scenario set; its decisions D1-D6 are the open choices above.
