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

`src/swb_isaac_world/spikes/terrain_hooks.py` drives aib's ego (`aib.isaac.vehicle.Vehicle`, the
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
  `just surfaces-plot` draws the zones over the lanes (`swb_isaac_world.surfaces`).
- **The friction table holds the class's peak friction** (the survey's table: dry asphalt
  ~0.85, wet 0.5-0.7, gravel ~0.6, dry earth ~0.68, packed snow 0.2, ice 0.1; calibrate,
  do not copy). PhysX already does the per-wheel lookup, so mu-split (one side in a puddle)
  costs nothing.
- **Each wheel has its own table** (aib, `/World/vehicleTireFriction/<vehicle>/<wheel>`, no ground
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
  before Isaac starts, and looked for and read again at every `LoadWorld` (adding or editing
  one needs no restart). swb's
  `LoadWorld` sends only the USD, which the server maps back; 046's world protocol v4 should
  carry the map's surfaces itself. A world with no surface map writes nothing.

### Surface layer, measured on a pad (047 S1, 2026-10-23)

`src/swb_isaac_world/spikes/surface_drive.py`: aib's Model 3 (`ev`, ABS and traction control on)
through `swb_isaac_world.surface_layer` -- the world server's own code -- on one flat mesh labelled
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

### Surface layer on Town01 (047 S1, 2026-10-23)

`src/swb_isaac_world/spikes/surface_town.py` drives the same cases on Town01's SimReady world
(`Town01_physics.usda`, loaded in 534 s on a loaded host) along lanelet 13397, the east road
heading +y, zones on the lane from y = -160: every wheel's collider is a `Road` mesh
(`.../Road_Road_Town01_455/St_Road_TileRoad_RoadR`) and resolves to `Road` -> asphalt. Every
case matches the pad within 0.01 m (dry 24.73 m, wet 28.43, gravel 29.85, ice from 12 m/s
53.26 at 1.36 m/s^2, step 27.54, mu-split 27.35 with yaw -5.4 deg toward the dry side, flood
20.31 from 15 m/s and 65.37 from 25), so the town's triangles change nothing; Road labelled
gravel by `defaults` alone (no zone) gives the gravel zone's numbers. No surface layer and a
dry map stop at the same 24.73 m. Update 19 us p50 / 0.39 ms p99.

With Autoware (`town01_ego_drive`, 3 runs each, one world server): no map, a dry map, and
the route over wet asphalt, a left-wheel puddle, gravel and ice at the goal all pass with 0
MRM and tracking inside 045's spread (047 S1). The route's 4.3 m/s and gentle stops stay
inside every class's grip; a scenario that stresses Autoware on low grip is S6's.

The world server logs each wheel's change of surface (`<vehicle> wheel <i> at (x, y): <class>
[wetness] [water] (zone <id>), mu <value>`, the value on the first step there, still
blending), which is how a run shows its zones took.

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

**Rut sets (047 D5, built).** Every scenario starts on fresh ground unless its
RoadCondition names a saved set:

```xml
<RoadCondition frictionScaleFactor="1.0" wetness="dry">
  <Properties>
    <Property name="swb.ruts" value="worn"/>   <!-- or "fresh"; a parameter works: "$ruts" -->
  </Properties>
</RoadCondition>
```

The world server loads `<map>/ruts/worn.npz` (every grid zone's blocks of plastic sinkage and
its spacing; `soil.save_ruts` / `load_ruts`) and rebuilds the zones' tiles at once, so the
first wheel meets the ruts. It is an instruction, not road state: a later RoadCondition
without it leaves the ground as the wheels made it, the same set again is fine, a different
one (or `fresh`) after any wheel has driven on the soil is refused, since ruts swapped under a
moving car are not physical; put it in the Init. When a scenario ends (another load, a new
owner, a release) the server saves the ruts it made to `<map>/ruts/runs/<time>.npz` and logs
the path; copying that file to `<map>/ruts/<name>.npz` makes it a set. On the pad, a set saved
after two passes and loaded on fresh ground put the next pass in the ruts at -10.6 cm with a
third pass's 591 N, in 0.17 s.

The render mesh of a dirty tile is the same geometry, so RTX lidar and cameras see the ruts
the wheels feel.

### Mud (047 D6, decided 2026-10-23: B)

No published real-time mud model exists (survey): Bekker-Wong has no pore water, no rate
dependence and no clogging, and nothing measured on mud is available to fit one. So mud is a
stand-in on the same grid as sand, marked as one. `mud_wet` carries Wong's wettest clay row
(clayey soil, Thailand, 55 % moisture: n 0.7, k_c 16.03 kN/m^1.7, k_phi 1262.5 kN/m^2.7,
c 2.07 kPa, phi 10 deg; Janosi K 2.5 cm, the soft end) and a class key `unvalidated:` giving
the reason. The row's numbers are as the literature reproduces Wong's Table 2.3; they were not
checked against the book here. A soil zone of an unvalidated class is refused unless the
map opts in by name:

```yaml
allow_unvalidated: [mud_wet]
zones:
  - {id: pit, class: mud_wet, soil: grid, polygon: [...]}
```

and the world server (at every load) and `just surfaces-check` warn about each such zone with
the reason. Listing classes rather than a yes/no keeps a stand-in added to the table later
from passing silently. A `mud_wet` zone without `soil` is friction only, as before, and needs
nothing. Everything the grid does for sand it does for mud: ruts in the collision and the
lidar, rut sets, slip sinkage (the only dig-in: no rate softening or clogging, which stay
open until a scenario needs them; see the D6 options in the roadmap).

Offline (`test_soil.py`, aib's wheel at 4.5 kN): mud's soil-limited friction is 0.20 against
loam's 0.58 and sand's 0.55, and it sinks like loam (3.4 cm at 10 % slip, 5.0 at 50 %).
On the pad (`soil_drive.py`, `SOIL_SPIKE_ONLY=mud`, warp kernel):

| case | asphalt | mud (grid) |
|---|---|---|
| coast in at 15 m/s, 3 s: deceleration | 0.00 m/s^2 | 0.67 (3.4 cm, 1195 N, friction 0.17) |
| hold 10 m/s for 80 m: speed, acceleration demand | 10.0, 0.00 | 9.53, 0.70 (1265 N) |
| the same line again (ruts) | | 9.87, 0.19 (4.3 cm, 351 N) |
| full throttle from rest, 3 s | 10.84 m/s | 0.02 m/s: stuck, dug in to 5.7 cm, 1941 N |
| full braking from 15 m/s (ABS) | 14.3 m | 30.8 m |

The rear-driven Model 3 cannot pull away from rest in mud: the spinning rear wheels dig in
(slip sinkage) until the resistance, 1.9 kN, exceeds what friction 0.19 on the rear axle
gives. That is plausible for a road car on road tyres, but it comes from a stand-in, which is
why the map has to ask for it.

### Soil layer on a pad (047 S3, 2026-10-23)

`spikes/soil_drive.py`: aib's Model 3 on a flat pad labelled Road, three soil zones from
x = 100 to 400 m (sand on the grid, dirt = sandy loam on the grid, sand closed-form), through
`SurfaceLayer` exactly as the world server runs it (3 substeps of 1/60 s per 50 ms frame). No
ruts in the collision yet: the soil reaches PhysX as friction and as the resistance force.
Host newslab-server-241 at load average 68 from other users' jobs.

| case | asphalt | sand (grid) | dirt (grid) | sand (closed form) |
|---|---|---|---|---|
| coast in at 15 m/s, no throttle, 3 s: deceleration | 0.00 m/s^2 | 1.23 | 0.56 | 3.11 |
| ... sinkage, resistance (4 wheels), friction written | | 8.4 cm, 2305 N, 0.39 | 2.9 cm, 975 N, 0.54 | 10.9 cm, 5635 N, 0.33 |
| hold 10 m/s for 80 m: speed, acceleration demand | 10.0, 0.00 | 9.13, 1.30 | 9.60, 0.59 | 0.07, 3.0 (stuck, 25 cm) |
| the same line again (ruts) | | 9.70, 0.46; 834 N | | |
| full throttle from rest, 3 s: speed, mean rear slip | 10.84, 0.11 | 1.15, 0.18 | 7.36, 0.19 | |
| full braking from 15 m/s (ABS): stop | 14.3 m | 13.5 m | 14.7 m | |

- The force channel behaves: 2305 N decelerates the car at 1.23 m/s^2, F/m within 2 %.
- Multipass: the second pass on the same line meets 36 % of the first one's resistance and
  sinks 2.2 cm more, not 8.6 (the grid's elastic reload: the wheel rides in its rut).
- The closed form pushes fresh soil at every step, so its resistance is 2.4 times the grid's
  at the same speed and the car bogs down at 10 m/s; offline, Wong-Reece's stress field puts
  no load at the bottom of the wheel and sinks it deeper (`test_soil.py`). S5's calibration
  against Chrono decides between them; until then the grid is the default and the closed
  form is a check.
- Front wheels in a zone carry the class's slip curve at zero slip (sand 0.6 of peak), which
  also takes their lateral grip: the S1 shape applied through the one friction value.
- Entry at 20 m/s: the resistance ramps over 12 substeps (front, then rear axle, 2.9 m apart,
  plus the 1.11 m blend); the largest change between two substeps was 535 N of 2388 N.
- Cost: the surface layer's update per vehicle per substep was 3.7 ms in a zone with the
  numpy sweep (p50, p99 12.5 ms; 23 us outside one), about 11 ms of a 50 ms frame. The sweep
  is now one warp kernel on the CPU (`soil_warp`; a single thread per wheel, the nodes in
  compiled loops, the wheel bottom by bisection): 141 us per wheel per step at 25 m/s offline,
  1.29 ms per vehicle per substep in a zone on the pad (p99 6.3 ms), ~4 ms of a frame. The
  numbers above are the same with either kernel. Grid memory: 1.1 MB for two 80 m passes of
  four wheels in sand.
- Entry at 20 m/s, again with steady state: the largest change between substeps was 537 N on
  entry and 198 N in steady sand 10 m on (mean 2210 N). The blend lets a wheel's ~550 N in
  over 3.3 substeps, ~170 N a substep, both front wheels at once ~340 N, plus the ripple.

### Ruts in the collision and the lidar (047 S3, 2026-10-23)

A soil zone that sets `ground_z` gets the server's own ground (`soil_tiles.SoilTiles`): 1.6 m
tiles (two grid blocks, 32 cells at 5 cm) over the zone's bounding box at that height, flat
two-triangle quads at load. The map must have no ground collider inside the box: a wheel's
suspension ray takes the highest hit, so the ruts must be the top surface. Once a frame
(`SurfaceLayer.flush`) the tiles over blocks whose ruts moved more than 5 mm are rebuilt at
full resolution from the grid's nodes (edges shared with the neighbours), at most 4 a frame,
oldest first. PhysX does not see a cooked mesh's points change, so a rebuild is a new prim,
defined before the old one goes.

A tile is rebuilt only once no wheel on soil stands within 0.5 m of it. The first version
rebuilt tiles under the wheels: every frame the ground under a slow wheel dropped by the
sinkage the grid had just added, the car shook to a crawl, the slipping wheels dug (Lyasko),
and it sat on its belly 22 cm down after 10 m of sand, 676 rebuilds later. With the wait, a
wheel's own pass is the grid's forces' to carry, and its rut is felt from the next pass on,
by it, by the axle behind it and by every other vehicle.

Pad, `soil_drive.py` `sand_rut` lane (60 m of tiles in a hole in the pad), 8 m/s, one line:

| | speed | rear wheels' ground | body | resistance | tiles rebuilt | rebuild time a frame |
|---|---|---|---|---|---|---|
| first pass | 7.13 m/s | -7.1 cm (in the front wheels' ruts) | -6.8 cm | 2360 N | 88 | 2.0 ms p50, 4.6 max |
| second pass | 7.58 m/s | -9.9 cm (in its own) | -9.9 cm | 879 N | 88 | 2.1 ms p50, 4.9 max |

Rut depth in the grid at x = 130 m: 9.2 cm after the first pass, 11.2 cm after the second. The
largest speed drop in a frame on the second pass was 0.25 m/s, dropping into the ruts at the
zone's start. Memory: ~0.3 MB stays per rebuild (docs/issues/001).

RTX lidar (`spikes/soil_lidar.py`: two tracks driven into the grid, the tiles rebuilt, aib's
OS1 2 m up): the ground between the tracks read -3.63 cm (the same as flat ground before,
a constant offset), the tracks -12.98 cm, so 9.36 cm deep against the grid's 9.43. Four tiles
rebuilt while the lidar rendered: 3.7-4.9 ms to edit, frames 22-23 ms against 28.5 ms
without a rebuild (no cost the noise shows).

Memory per rebuild (docs/issues/001, `spikes/tile_memory.py`): PhysX keeps the cooked data
of every distinct mesh it has cooked, released or not, ~465 KB for a 5 cm tile; nothing
reachable from Python frees it or re-cooks a mesh in place. So the collision tile is 10 cm
(64 KB a rebuild) and the render tile, the grid's 5 cm, is one prim edited in place (no
cooking); and a tile waits for every wheel to be 3 m away, one rebuild a pass. Two passes on
the pad: 44 rebuilds and ~3 MB a pass, against 88 and 23-36 MB; the second pass rides in its
ruts at -9.3 cm, the lidar still reads them 9.36 cm deep.

**Since issue 001's second fix (2026-10-23) no collision is cooked after load.** The egos are
the only bodies that feel the ruts, and each meets the ground through one suspension ray per
wheel, so the zone's collision is a flat box at `ground_z` that those rays do not hit (a
collision group filtering aib's `groundQuery`; tracked NPCs stand on it), and under each ego
wheel a thin box, a plate, placed before every substep where the wheel will be after it, its
top the least-squares plane of the shown ruts over the tire's footprint (`SoilTiles.place`,
`surface_layer.wheel_points`). A tile rebuild is now its render mesh and the heights the
plates read. The footprint matters because the grid's rut is a box with vertical walls: the
10 cm collision mesh made each wall a slope a wheel slid into from beside the rut, and a plate
at the single ray point let a tire overlapping a rut by 9 cm ignore it; over the footprint a
tire half over a rut sits half in it, tilted toward it. Pad, same passes as above: the second
pass rides at -7.8 cm (rear wheels) / -8.1 (body), 7.67 m/s, largest drop in a frame 0.02 m/s;
ten more passes on shifting lines, 331 rebuilds, grew the process by 0.9 MB in all, against
2.5-6.7 MB a pass with the cooked tiles. The fit costs 26 us a wheel. A plate stays up until
its wheel is half a plate clear of the tiles' box (flat past the tiles): parked any sooner,
a ray landing a few cm from the prediction found the box's hole, a substep in the air per
axle at a zone's exit, which on Town01 was two emergency stops (issue 002). Town01 with
Autoware, the lead on the ego's line: 4/4 PASS, 0 MRM, 0.19-0.28 m/s^2.

### A zone on a CARLA town: the town's ground cut out (047 S3, 2026-10-23)

At load (after the world is referenced), each soil zone with `ground_z` cuts its tiles' box
out of every mesh under `/World/env` that reaches into it between 0.8 m below and 0.05 m above
the zone's ground (`soil_tiles.cut_stage`, geometry in `ground_cut`): faces inside go, faces
across the edge are split into the convex pieces outside, UVs and other primvars interpolated,
material subsets remapped. The cut geometry is written over the prim in the in-memory stage,
so cameras and the lidar see the town with a hole the soil tiles fill. PhysX does not re-cook
an edited mesh, so a colliding prim's collision is switched off and its cut collision is a new
invisible mesh under the zone, carrying the prim's SimReady label (the surface layer's
classes still resolve). Loading another world, or the same one again, clears every opinion
the cut wrote: the town is whole again. A zone is then only lines in `surfaces.yaml`; no
asset step.

Town01 (`spikes/soil_town.py`), dirt zone x 145.6-169.6, y -134.4..-128.0 on
town01_ego_drive's road: 14 meshes cut in 0.8 s -- eight road tiles, four lane markings, the
crosswalk and the 101 866-face landscape under all of them; street lights over the zone left
alone.

| | result |
|---|---|
| rays inside the zone | soil tiles at z 0 (lane, crosswalk, edge) |
| rays just outside | the cut road's collision copies at z 0 |
| frame time, car outside, no zone / with the zone | 3.07 / 2.91 ms p50 (3 substeps, no rendering) |
| frame time in the zone | 6.7 / 6.2 ms p50, p95 14.6 / 17.4 ms (two passes, 8 m/s) |
| pass 1 / pass 2 | 7.65 / 7.90 m/s; resistance 1054 / 279 N; rear wheels on the ground at 0 / -2.6 cm (in the rut) |
| rut in the grid | 2.6 cm, then 2.9 cm (sandy loam) |
| after the zone is removed | rays hit Town01's own road again |

Only faces lying wholly between 0.8 m below and 0.05 m above the zone's ground are cut: the
first version cut whole meshes whose box reached the band, and a zone on Town01's east road
took 87 faces off a bridge and 304 off a guardrail's uprights. A mesh the cut empties (two
lane markings) is hidden with its collision off rather than left faceless ("does not have
indices", a PhysX error). And the cut's stage edits get ten physics steps of their own
before anything else: handed to PhysX in the same step as preload's warm-up ego, the ego
was not in the physics scene and the server respawned 33 times.

With Autoware (`just acceptance town01_ego_drive.xosc 2 sim:=isaac worlds:=Town01`,
2026-10-23, host load 30-130 on 24 cores from other users' jobs; three sets on one world
server: A no surface map, B the zone off the route on the east road, C the zone across the
route, `scenarios/surfaces/town01_soil_{off,route}.yaml`):

| set | SSv2 verdicts | MRM entries | longitudinal | step p50 / p95 |
|---|---|---|---|---|
| A (two batches) | 4/4 pass | 83, (none: never drove); 74, 67 | 0.084-0.118 | 43-55 / 82-392 ms |
| B | 2/2 pass | 70, 75 | 0.101-0.112 | 43.9-44.4 / 82-91 ms |
| C | 1 pass, 1 never reported in 900 s | 42 | 0.348 | 59.6 / 176 ms |

Every set fails acceptance, the baseline too: the MRM state flaps (emergency stop on and
off every second or two) from the start of each run, before any zone, and one run in A and
one in C never drove: Autoware starved of CPU (the simulation ran at about 0.2 x real time).
S1's runs on the same scenario had 0 MRM and 0.007-0.018 m/s^2. So these runs show only the
plumbing and what the soil does: in C the ego entered the dirt at 4.1 m/s, the soil took it
to 2.4, Autoware answered at its 1.0 m/s^2 limit and was back at 4.1 by the zone's exit
(hence 0.348 m/s^2 of tracking error); with the zone off the route, B's step time is A's.
A clean Autoware comparison waits for a quiet host.

### Every vehicle on the grid (047 S4, 2026-10-23)

A zone's grid is shared. The egos run on it as before; an NPC vehicle rolls on it too, so the
ruts the egos meet include the traffic's. An NPC does not feel the soil (SSv2 drives it), it
only marks it: each of its four wheels rolls without slip on a quarter of its mass, its
wheel base, tread and radius CARLA's for its body (`surface_layer.kinematic_wheels`, from the
catalogue's physics; shares of its box where CARLA gives none). A kinematic NPC's pose comes
from SSv2's drive; a tracked one's (044, a dynamic body driven by velocity) from PhysX, a
frame behind, since its drive carries no pose (the first Town01 run found that: the lead
stamped its footprint once at spawn and never again). Each frame's motion is split over the
substeps, so the grid sweeps its wheels as continuously as an ego's; a jump of more than 5 m
in a frame is a teleport and leaves no rut. Every substep updates the egos in handle order,
then the NPCs in path order, so a rerun cuts the same ruts. Tests: `KinematicTest` (4).

### Scenarios and exit runs on Town01 (047 S6, 2026-10-23)

Two scenarios on town01_ego_drive's route (x 190.8 to 120 along y -130), each with its
surface map in `scenarios/surfaces/` (copied to the map directory for a run):

- `town01_soil.xosc` over `town01_soil.yaml`: dry sand (x 171.2-180.8), then dirt (sandy
  loam, x 145.6-169.6), both soil zones with their own ground; a lead car SSv2 moves at
  5 m/s on the ego's line lays its ruts first; parameter `ruts` (default `fresh`) is the
  rut set the soil starts from.
- `town01_mud.xosc` over `town01_mud.yaml`: 12.8 m of wet mud (x 145.6-158.4), opted in.

`just acceptance <xosc> 2 sim:=isaac worlds:=Town01`, one world server, newslab-server-241,
host load 25-175 on 24 cores from other users' jobs (a 30-hour multiprocessing job):

| set | runs | MRM | longitudinal (m/s^2) | peak |
|---|---|---|---|---|
| A: town01_ego_drive, no surface map | 2/2 PASS | 0, 0 | 0.022, 0.019 | 4.26 m/s |
| S: soil, lead 0.7 m beside the ego's line | 2/2 PASS | 0, 0 | 0.224, 0.214 | 4.48 |
| S: soil, lead on the ego's line | 1/2 | 0, 1 | 0.300, 0.203 | 4.48, 4.45 |
| R: soil from rut set `worn` (that run's ruts) | 2/2 PASS | 0, 0 | 0.115, 0.140 | 4.28, 4.32 |
| M: mud | 2/2 PASS | 0, 0 | 0.086, 0.081 | 4.38 |
| S after S5's fit, lead on the line | 0/2 | 1, 3 | 0.186, 0.190 | 4.44 |
| S after the fit, ruts feathered at the edges | 1/2 | 1, 0 | 0.284, 0.206 | 4.47, 4.41 |

The rows above S5's fit ran Wong's rows as published; the last two the fitted sand and dirt
(S5, below). Every SSv2 verdict was a pass; the FAILs are acceptance's MRM count.

(Before these, a first S batch lost run 1 to a 14 s step while the world server compiled the
lead's body's materials, its first use on the host; and the lead left no ruts, the fault
above.) What the runs show:

- The soil costs Autoware tracking, not the route: its speed falls in the sand and it pulls
  back at its limit (0.2-0.3 m/s^2 of median error against 0.02 without a map, under the
  0.35 limit). Mud costs less than sand (0.08): the zone is shorter and sinks 3-5 cm.
- Ruts make it easier, as on the pad: from the saved set (sand 13.6 cm deepest, dirt 3.8)
  the error halves to 0.12-0.14.
- The NPC's ruts: 7.5 cm in the sand and 1.9 cm in the dirt (1130 kg, no slip). Beside the
  ego's line, both pairs stay separate; on it, the ego enters the sand 0.3 m beside them and
  is in them within 4 m, and from there on the grid holds one pair of tracks. The one MRM
  (0.1 s, MRM_OPERATING then back to normal) came at x 181.4, the sand's edge, at 4.0 m/s,
  where the ego's wheels slid sideways into the lead's 6-7 cm ruts; the run passed. With the
  fitted soil, two runs had four: at the sand's entry (x 181.3), its exit onto the road
  (170.0) and the dirt's exit (146.2, 146.6), each 0.1-0.2 s, emergency stop on and off.
  There the ruts, 9-10 cm deep with the lead's and the ego's passes in them, ended in a
  vertical wall against the rigid road: a step loose soil cannot hold, met at 4 m/s. So the
  tiles now feather the ruts out over the zone's last metre (`soil_tiles.FEATHER`; the
  grid keeps them whole, and the soil's forces already fade in over ~1.1 m there): the next
  two runs had no MRM at any edge. One remains, in one run of two: 0.4 s at x 176.5, in the
  sand, where the ego was pulled 0.2 m sideways into the lead's fresh ruts (y -129.58
  against its line's -129.37). Its cause is not known: Autoware's logs say only "EMERGENCY_STOP
  is operated", and a /diagnostics listener on the ego's domain received nothing. Runs on
  packed ruts (R, 2/2) or beside them (2/2) had none. Seven reruns of the same set later
  that evening had none either; the acceptance bag now records the diagnostic graph for the
  next one. It was NDT, not the soil: its solver cycles between two poses to its iteration
  limit, more often where the ego pitches in the ruts, and that WARN is an emergency stop in
  Autoware 1.5's graph; fixed in the Autoware overlay ([issue 002](../issues/002-mid-sand-mrm.md)).
- The world server's step time in the zones, whole stack: p50 39-85 ms, p95 95-408 ms, the
  high ones with host load at 100-175 (every part of the step slowed, commands too).

### Calibration against Chrono (047 S5, 2026-10-23)

`spikes/crm_calibrate.py` runs our grid and Chrono's single-wheel test rig (pychrono 10.0
from the projectchrono conda channel, CUDA 12.8, in its own environment on the data disk)
on the same wheel: rigid, 0.37 m radius, 0.235 m wide, 4.5 kN, the carrier at 1 m/s, slip
pinned at 0-0.5; the steady last 40 % of a 4 s test. Two references:

- **Chrono SCM** with the class's own Bekker-Wong-Janosi row: the same model family, so it
  checks our implementation. With a smooth cylinder its sinkage does not grow with slip
  at all; with a tread (24 bars 2 cm deep in the contact mesh) it does, through the bars.
- **Chrono CRM** (SPH continuum, mu(I) rheology) with the class's density, friction angle
  and cohesion, E = 2 MPa as Chrono's CRM wheel validations use: a different model of the
  same soil, the survey's virtual bevameter. 85 s a case at 2 cm spacing on one RTX 5090,
  280 s at 1 cm; the two spacings agree on sinkage within 4 %.

| sinkage, cm, slip 0 / 0.5 | sand | dirt (sandy loam) |
|---|---|---|
| grid, Wong's row as published | 7.9 / 13.5 | 2.7 / 4.4 |
| grid, fitted (below) | 7.9 / 10.4 | 2.7 / 3.7 |
| Chrono SCM, smooth / treaded | 8.3 / 7.9; 7.8 / 9.8 | 3.4 / 3.3; 3.2 / 4.5 |
| Chrono CRM, smooth / treaded (1 cm: 6.6 / 9.1 treaded sand) | 5.6 / 7.8; 6.8 / 8.9 | 3.6 / 5.4; 4.9 / 6.7 |

| drawbar pull, N, slip 0 / 0.1 / 0.2 / 0.3 / 0.5 | sand | dirt |
|---|---|---|
| grid, as published | -768 / 1185 / 1479 / 1475 / 1391 | -380 / 565 / 1301 / 1640 / 1906 |
| grid, fitted | -768 / 950 / 1400 / 1508 / 1535 | -380 / 322 / 863 / 1415 / 1851 |
| Chrono SCM, treaded | -953 / 876 / 1312 / 1654 / 2061 | -165 / 407 / 901 / 1221 / 1597 |
| Chrono CRM, treaded | -529 / -318 / -188 / -96 / 89 | -339 / -57 / 111 / 224 / 385 |

What it says:

- **The implementation agrees with Chrono's SCM** on what both model the same way: static
  sinkage within 1 % (sand) and 16 % (dirt), and the resistance at zero slip within 19 %
  (sand).
- **Slip sinkage was too strong.** Lyasko's factor as published sank our wheel 1.63-1.71
  times deeper at 50 % slip than at none; treaded SCM and CRM, two different models, agree
  on 1.26-1.41. So a class now carries `slip_sinkage`, the factor's power, fitted to the
  mean of the two: sand 0.5, dirt 0.65 (sinkage ratio within 0.02-0.04 of the references
  at every slip). Mud and snow keep 1: nothing to fit them to.
- **Janosi K** (the shear build-up, the row's least-sourced number, "typical ranges") fitted
  to treaded SCM's drawbar pull: x1.5, sand 0.015 m, dirt 0.0375 m. The grid's pull is now
  within -26 % to +16 % of SCM's from 10 % to 50 % slip in both soils (was +35 % to +44 %
  in dirt).
- **CRM's pull is not a target.** It stays near zero to 50 % slip for a smooth, a 2 cm
  treaded and a 1 cm-resolved treaded wheel alike: a rigid tire's markers in CRM grip the
  soil far less than Bekker-Janosi's assumed full soil shear, and a road tire's tread is
  below what CRM resolves. The truth for a pneumatic road tire lies between; the field
  (040's cone index) has to say where. CRM's sinkage is used, its pull is recorded only.
- Static sinkage against CRM: sand within 20 % (7.9 against 6.6), dirt 25-45 % shallower
  (2.7 against 3.6-4.9). Dirt's k_phi is the next number to question when field data comes.

On the pad, the fitted soil changes little at steady speed (coasting into sand 1.22 m/s^2
against 1.23, holding 10 m/s 9.15 m/s against 9.13, the second pass 875 N against 834) and
much where the wheels spin: a full-throttle launch in sand reaches 3.29 m/s in 3 s against
1.15, the rear wheels digging in less; dirt 7.66 against 7.36. Entry at 20 m/s: the same
537 N largest step.

### Still to measure

- **Physics rate in a zone.** Answered (047 D4, S3): the grid sweeps each wheel along its
  path one node spacing at a time inside a step, so 60 Hz reads as 600 Hz within 0.4 %
  (`test_soil.py`); the rate stays the world's.
- **Re-cook budget.** Measured (S3): 1.6 m tiles, 4 a frame, 2 ms a frame on the pad; four
  with an RTX lidar rendering, 4 ms to edit and no frame-time rise.
- **RTX cost of a changed mesh**: none visible; the lidar sees the rebuilt geometry within a
  millimetre (S3). Displacement maps were not needed.
- **Calibration**: done against Chrono SCM and CRM (S5, above); what is left is the field:
  cone index and a drawbar test of a real road tire on sand and on dirt (040), which is
  what settles CRM's and Bekker's disagreement on traction, and mud.

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

### Until 047 S2, nothing reached the simulator

SSv2 parsed `Environment`, `Weather` and `RoadCondition`
(`openscenario_interpreter/syntax/road_condition.hpp`: `friction_scale_factor`, `wetness`,
`properties`), but `EnvironmentAction::start()` and `run()` were `LINE()` no-ops, and
`simulation_api_schema.proto` had no message for it
([ssv2-feature-completeness.md](ssv2-feature-completeness.md), section 2). A `RoadCondition`
without `wetness` was even refused as a missing required attribute, although the schema makes
it optional.

### The map says where, the scenario says what state (D3, decided 2026-10-23)

- **Where** (static, per map): `<map>/surfaces.yaml` (D1, above).
- **What state** (per scenario, can change mid-run): the standard `RoadCondition`.
  `frictionScaleFactor` and `wetness` apply world-wide; per-zone state rides in its
  `Properties`, named by the map's zone ids, so the file stays schema-valid and any other
  tool ignores what it does not know:

  ```xml
  <GlobalAction>
    <EnvironmentAction>
      <Environment name="ford_rises">
        <RoadCondition frictionScaleFactor="1.0" wetness="moist">
          <Properties>
            <Property name="swb.zone.ford_1.water_depth" value="$ford_depth"/>
            <Property name="swb.zone.track.class" value="gravel"/>
          </Properties>
        </RoadCondition>
      </Environment>
    </EnvironmentAction>
  </GlobalAction>
  ```

  `swb.zone.<id>.<field>` sets one field of a zone, as `surfaces.yaml` names it: `class` (a
  class of the table; not on a `soil` zone), `wetness` (an OpenSCENARIO level) or
  `water_depth` (metres; replaces a raster with one depth). Values may be parameters, so a
  distribution can sweep them. An `EnvironmentAction` in the Init sets the start state; one in
  the Story (on a time or distance trigger) changes it. A `RoadCondition` is the whole road
  state: it replaces the last one, and a zone it does not name is as the map has it. Each
  `Initialize` (the world server's `LoadWorld`), a new owner and a release go back to the
  default: friction scale 1, dry, the map's zones.

How it gets there (roadmap [047](../roadmap/047-off-road-surfaces.md), S2):

1. **SSv2 fork** (`managed-ego-unforked`): `EnvironmentAction` ends immediately and its
   `start()` sends the `RoadCondition`, when the `Environment` has one, as
   `UpdateEnvironmentRequest` (oneof 17: friction scale, wetness, every property as a
   string); `API::updateEnvironment` throws on a refusal, so the scenario fails with the
   simulator's text. A missing `wetness` reads as dry. The fork knows nothing of `swb.`.
   `MultiServer` (simple_sensor_simulator) acknowledges it. Weather and time of day are not
   sent yet.
2. **Bridge**: checks the wetness and the scale, drops properties outside `swb.` and sends
   the rest as `SetRoadCondition` (world protocol, request 14) to a server declaring
   `CAPABILITY_ROAD_CONDITION`; to one without, it refuses any condition but the default.
   No protocol bump: only a server declaring the capability is sent it.
3. **Isaac world server**: `surfaces.road_condition` checks every `swb.` property against the
   loaded map and the class table, lists every problem and then changes nothing; otherwise
   the surface layer takes the condition and the overridden zones from the next step. A
   world with no `surfaces.yaml` gets a map of label defaults for a condition that changes
   something, so a zone property is refused there ("zones: none").
4. **CARLA world server**: one friction per wheel and no road model, so every vehicle's tire
   friction (`WheelPhysicsControl::tire_friction`, its own value read at the first change)
   becomes its own times `frictionScaleFactor` times asphalt's wet factor (the class table's
   numbers, checked by a test); the weather's `wetness` and `precipitation_deposits` show the
   level (rendering only on 0.9.16), and the weather from before is put back at the next
   `Initialize`. `swb.` properties are refused: zones are the Isaac surface layer's.

### Weather and time of day on Isaac (probe, 2026-10-23)

The fork sends only the `RoadCondition` so far. Before sending `Weather` and `TimeOfDay`, what
Isaac Sim 6.1 can do with them was measured on a test pad (`spikes/weather_probe.py`: 2 m
boxes at 10-80 m, aib's OS1 RTX lidar and an RGB camera 2 m up; four runs):

| OpenSCENARIO | Isaac 6.1 | Measured |
|---|---|---|
| `TimeOfDay` (`dateTime`, `animation`) | the dynamic sky (`Skies/Dynamic/*.usd` on NVIDIA's content server, as the Environment menu adds it), moved by `omni.kit.environment.core`'s sun-study player from time, day, latitude and longitude; `animation` is its play mode | sun elevation 6.0 deg at 06:00, 32.9 at 09:00, 46.6 at 12:00, 6.5 at 18:00, -25.1 at 22:00; camera mean 155, 202, 209, 156, 0.8 |
| `Weather` cloud state, `fractionalCloudCover` | the dynamic sky's `CloudCoverage`, `CumulusEnabled`, `haze` (or another sky: Cumulus, Overcast) | not measured |
| `Weather.sun` | the sky's distant light, set directly | -- |
| `Weather.fog` (`visualRange`) | RTX fog, `/rtx/fog/*` carb settings (distance and height based): **cameras only** | camera mean 72 to 142, contrast 83 to 63; lidar unchanged (36.6k points per scan either way) |
| `Weather.precipitation` rain (`precipitationIntensity`, mm/h) | the RTX lidar's Mie-scattering rain model, `/app/sensors/nv/atmospherics/rainRate`: **lidar only**, rain only | points per scan 51.3k dry, 51.3k at 2.5 mm/h, 43.0k at 25, 1.2k at 100; no phantom returns near the sensor (they need the sensor material framework, off here) |
| precipitation snow, wind | none (fog, haze and snow are "planned" in the lidar's docs) | -- |
| rain or snow seen by a camera, a wet road's look | none built in (would be particles and material edits of our own) | -- |
| radar | no weather model | -- |

Two findings matter for using it:
- **Rain does not stop**: with `rainRate` back at 0 after the sweep the lidar kept 36.6k points per
  scan, 29 % under the 51.3k before any rain, in both runs that checked it. A scenario that
  stops the rain mid-run would keep a degraded lidar; until that is understood, rain on Isaac is
  set once per world server.
- The dynamic sky is fetched from NVIDIA's content server at run time (`omni.client`, ~17 kB of
  USD plus its MDL and textures): a host without internet needs a local mirror. The sun-study
  player has no instance when headless (`get_sunstudy_player()` is None): the world server would
  make its own, and `SimulationApp.close()` then never returned in the probe.
- One run of four got no lidar data at all (the probe's annotator, not the weather).

So Isaac covers time of day well, fog for cameras, rain for the lidar, and nothing else;
CARLA's `WeatherParameters` cover all of it visually (none of it physically, as with wetness).

### Weather and time of day from the scenario (047, built 2026-10-23)

```xml
<Environment name="morning_rain">
  <TimeOfDay animation="false" dateTime="2026-06-21T07:00:00"/>
  <Weather fractionalCloudCover="twoOktas">
    <Fog visualRange="300"/>
    <Precipitation precipitationType="rain" precipitationIntensity="$rain_mm_h"/>
  </Weather>
</Environment>
```

Within a `Weather` every attribute and element is optional, and one not given keeps its last
value (a story event that only stops the rain leaves the fog); a `TimeOfDay` replaces the last.
`Initialize`, a new owner and a release go back to the world's own sky.

1. **SSv2 fork**: `UpdateEnvironment` carries `Weather` (cloud cover or the deprecated
   `cloudState`, sun, fog's visual range, precipitation type and mm/h -- the deprecated [0..1]
   `intensity` as a fraction of 25 mm/h -- wind, temperature, pressure) and `TimeOfDay`
   (`dateTime` as written, `animation`). Upstream read `temperature`, `atmosphericPressure`
   and `fractionalCloudCover` as required and could not take a `Weather` without each of its
   elements; all of it is optional now. `DomeImage` is not sent.
2. **Bridge**: checks every part (all problems in one refusal), turns `dateTime` into the
   clock time written and the day of the year (the zone is not applied: the time as written is
   the local solar time), drops temperature and pressure (nothing renders them), and sends
   `SetEnvironment` (world protocol request 15, `CAPABILITY_ENVIRONMENT`). It checks the
   capabilities of the whole environment before sending any part.
3. **Isaac** (`swb_isaac_world.sky`, `IsaacWorld.set_environment`): a time of day or a placed sun
   (a placed one wins) takes over the world's sun: the town's own distant lights go off and
   ours, with their brightness, turns to where the sun stands over the map's latitude
   (`map_projector_info.yaml`; CARLA's towns: 0), full from 30 deg up, off below the horizon;
   the domes dim with it to 3 % at night. Cloud cover dims the sun (to 20 % at 8 oktas, off at
   9): a lighting approximation, no clouds drawn. Fog: RTX fog to the visual range, none from
   1000 m. Rain: the lidar's `rainRate` (a warning when it stops: the lidar does not fully
   recover). Snow refused; wind taken and ignored. An animated time moves the sun each
   simulated second. `spikes/sky_world.py` measures it on a world with its own sun and dome.
4. **CARLA** (`scenario_weather.rs`): `cloudiness` (8 oktas = 100), `precipitation` (25 mm/h
   = 100), `fog_density` (none from 1000 m, 100 at 0 m), `wind_intensity` (20 m/s = 100), the
   sun's altitude and azimuth (from the time of day with the same sun model as Isaac's, or
   placed); snow refused. The azimuth's reference (CARLA's 0 taken as east, clockwise) is
   assumed, not checked against CARLA's renderer. All of it lies over the weather from before
   the scenario together with the road's wet look, and that weather comes back at
   `Initialize`.

Autoware is not told: a real car does not know the friction either. Its ABS/TC (in aib's
powertrain) and its planner meet the surface as a driver would.

## Next steps

Roadmap [047](../roadmap/047-off-road-surfaces.md): surface layer, road condition from the
scenario, soil layer in one zone, persistence and the second pass, calibration, the ABS/TC
scenario set; its decisions D1-D6 are the open choices above.
