# Phase 022: Entity Geometry Parity

Second fix of the [gap campaign](../design/gap-campaign.md): the body CARLA spawns for an
entity is the closest CARLA has to the box the scenario declares, and whatever difference
remains is measured at spawn and reported, not discovered in a collision.

**Source**: user direction 2026-10-11 ("fix scenario spec and Carla gap first").
**Depends on**: [008](008-entity-fidelity.md) (blueprint resolution, misc objects),
[014](014-feature-completeness.md) (`OriginOffset`), [021](021-physical-verdict.md)
(the contact report this phase adds a geometry section to).
**Feeds**: [024](024-gap-trace-and-oracle.md) (blueprint boxes in the trace header).

## Problem

- **One body for every vehicle.** `spawn_entity` (`coordinator.rs:2636`) takes the
  scenario's `model3d` when it names a CARLA blueprint, else `blueprint_map` (one entry,
  `sample_vehicle`), else `vehicle.tesla.model3`. A scenario's 12 m truck, 2 m-wide bus or
  1.8 m motorcycle is a 4.7 m Model 3 in CARLA; SSv2 judges the declared box. SSv2 then
  fails runs on overlaps CARLA never had, and passes runs in which CARLA's body was hit
  outside the declared one -- and the ego's own perception sees the Model 3.
- **The subtype never reaches the simulator.** SSv2 knows each vehicle's `EntitySubtype`
  (CAR, TRUCK, BUS, TRAILER, MOTORCYCLE, BICYCLE; from `vehicleCategory`, `vehicle.cpp:43`)
  and keeps it in its ROS `VehicleParameters`, but the protobuf `VehicleParameters` has no
  such field, so `simulation_interface`'s `toProto` drops it: a spawn request carries the
  declared box and nothing that says truck. (Found at step 3; the subtype appears only in
  each frame's `EntityStatus`, after the spawn.) The declared box is not used either.
- **The mismatch is silent.** Nothing compares the declared box with the spawned actor's
  `bounding_box()`, so even a deliberate `blueprint_map` choice gives no sign of how far off
  it is.

## Design

### Blueprint box catalogue

CARLA's API gives a blueprint no box; only a spawned actor has one. So the boxes come from a
table, generated once per CARLA version and shipped in the package:

- `scripts/measure_blueprint_boxes.py`: spawns every `vehicle.*`, `walker.pedestrian.*` and
  `static.prop.*` once, 500 m above the map, reads `bounding_box` (extent, location) and
  the blueprint's `base_type` / `number_of_wheels`, destroys it, writes
  `src/carla_scenario_bridge/config/blueprint_boxes_0.9.16.yaml` (installed to the
  package's share with the rest of `config/`). It never ticks, so it does not fight the
  bridge for the synchronous world; sensors none, so nothing is left behind.
- The bridge loads `blueprint_boxes_<carla::CARLA_VERSION>.yaml` at start, beside its config
  file first (like `traffic_lights_<town>.yaml`), then from each installed share. Missing
  or unreadable table: the bridge logs once and keeps today's behaviour.
- **Every spawn checks the table against the actor CARLA returned** (one `bounding_box()`
  read, at spawn only). A difference above 5 cm is a stale table, logged at WARN naming
  the script to rerun.

### Subtype on the wire (SSv2 fork)

`EntitySubtype subtype = 6` on the protobuf `VehicleParameters`, filled by `toProto` and read
back by `toMsg` -- the same kind of fork extension as `SpawnVehicleEntityRequest.behavior`
(017). Wire-compatible both ways: an SSv2 without it sends nothing, which protobuf reads as
`UNKNOWN`, and the bridge then matches among all four-wheelers; a bridge without it ignores
the field. `proto/` in this repo gets the same line.

### Choice, in order

1. `blueprint_map[model3d]` -- the operator's explicit word, as now.
2. `model3d` that is a CARLA blueprint -- the scenario's explicit word, as now.
3. **Size match**: candidates are the table entries of the entity's class -- vehicle
   subtype to `base_type` (CAR → car, van; TRUCK and TRAILER → truck; BUS → bus;
   MOTORCYCLE → motorcycle; BICYCLE → bicycle; UNKNOWN → all four-wheeled), misc object to
   `static.prop.*` -- and the winner minimises the summed relative error of length, width,
   height against the declared dimensions, length weighted 2 (it is what longitudinal
   scenarios measure). **The default wins whenever it is in the class and fits** the
   declared box (tolerances below): the starter scenarios' 4.5 x 2.1 x 1.8 m car scores
   closer to a Mini Cooper S 2021 (4.55 x 2.10 x 1.77) than to a Model 3 (4.79 x 2.16 x
   1.49), but the Model 3 fits, and swapping every NPC for a Mini would change behaviour
   where no gap is being closed. A plain "ties go to the default" would never fire: exact
   ties do not happen. A truck never keeps a car default, fit or not.
4. The kind's default, as now (no table, no candidates).

The **ego is excluded** from 3: its blueprint decides acb's sensor mounts and vehicle
parameters, so it is chosen by the ego stack's configuration, not by the scenario's box.
It still gets the spawn-time comparison below. **Pedestrians** are excluded too: CARLA's
walkers are within a few centimetres of each other and none matches SSv2's default
pedestrian box; the comparison reports the difference.

### Residual mismatch, reported

At spawn, for every entity: declared dimensions, chosen blueprint, actual extent, and per
axis the difference. Logged at INFO; outside tolerance at WARN. Tolerances: **0.3 m in
length and width** -- SSv2 judges the 2D footprint, so these decide whether SSv2 and CARLA
can disagree about a contact -- and **0.5 m in height**, which no verdict reads (a single
0.3 m on every axis would warn on every starter NPC: the Model 3 is 0.31 m lower than the
declared 1.8 m). The same tolerances define "fits" above. The same per-entity records go into 021's contact report as a `geometry` list, and
`physical_verdict` copies entries over tolerance into the `carla_contact` testcase's
`system-out` -- information, not a failure: a declared 16 m articulated bus that CARLA has
no body for is a fact of the scenario, and the contact testcase already fails if it
matters.

`OriginOffset` is unchanged: it moves the pose reference by the declared center, and the
chosen body is positioned the same way.

## Steps

1. [x] **Docs.** This phase doc.
2. [x] **Catalogue script and table.** Run on CARLA 0.9.16 (Town01); commit the YAML; check a
   few entries against a live `bounding_box()` by hand.
3. [x] **Choice, test-driven.** A pure `choose_by_size(class, declared, table)` with tests:
   12 m truck → the longest truck; 4.5 m car → Model 3 (the default fits); motorcycle; unknown
   subtype; empty table; misc-object prop by size; ego and pedestrian never size-matched.
4. [x] **Bridge wiring.** The subtype field in the SSv2 fork and `proto/` (rebuild SSv2);
   table loading by CARLA version; the chain above in `spawn_entity`;
   the spawn-time comparison and stale-table warning; `geometry` in the contact report and
   `physical_verdict`'s `system-out`.
5. [ ] **Live.** A crafted scenario per subtype (car, truck, bus, motorcycle, bicycle, misc
   object) on Town01: each spawns its matched body, and the logged residual matches the
   table. The nine starter scenarios: same blueprints as before for every entity whose box
   fits a Model 3 (no behaviour change where none is due), all pass.
6. [ ] **Disagreement closed.** A truck scenario that, before this phase, gives SSv2 PASS
   with a CARLA contact (or the reverse) under `--physical-verdict`; after it, both agree.
   The before-run is kept as the regression reproducer in `scenarios/gap/`.

### Results

**Step 2 (2026-10-11).** 191 blueprints measured on CARLA 0.9.16, Town01: 41 vehicles, 52
walkers, 98 props. `static.prop.mesh` has no finite box (an empty mesh: extent `-inf`) and
is left out. The Model 3 reads 4.792 x 2.163 x 1.488 m, CARLA's published extent (2.396,
1.082, 0.744) doubled, so the probe's reads are the live actor's. What the table holds:

- CARLA's `base_type` is inconsistent: `Bus` capitalised (the script lower-cases it), and
  empty on `bmw.grandtourer` and `mini.cooper_s` (a missing one on four wheels counts as
  a car).
- One bus (`mitsubishi.fusorosa`, 10.27 m), four trucks (`carlacola` 5.20, `cybertruck`
  6.27, `european_hgv` 7.94, `firetruck` 8.47 m), no trailer: a TRAILER gets the closest
  truck, and a 12 m truck gets an 8.5 m body, reported as a 3.5 m residual.
- Walkers come in three boxes only: 0.375 x 0.375 x 1.86, 0.375 x 0.375 x 1.30 (children)
  and 0.5 x 0.5 x 1.10 m.

**Step 3.** `blueprint_boxes.rs`: `BoxTable` (the YAML), `SizeClass::of_vehicle(subtype)`,
`choose_by_size(class, declared, table, default)`; `SpawnKind::size_class` excludes the
ego, agent vehicles and pedestrians. Twelve tests against the shipped table, plus one in
the coordinator: the starter car box keeps its Model 3; a 3.6 m city car gets a
`nissan.micra`; a 12 m truck the `firetruck`; a 12 m bus the `fusorosa`; a 2.2 m
motorcycle the `yamaha.yzf`; a bicycle the `gazelle.omafiets`; an unknown-subtype
bicycle-sized box a four-wheeler; a cone-sized prop `trafficcone02`; invalid or empty
input, nothing.

**Step 4.** SSv2 fork: `subtype = 6` on `VehicleParameters`, `toProto`/`toMsg`, and the
conversion test checks it round-trips (TRUCK). Bridge: `EntityShape` (origin offset,
declared box, subtype) replaces the bare origin offset in `spawn_entity`; the fallback
under `blueprint_map` and the scenario's blueprint is now the size-matched body, else the
default; after the spawn, `compare_geometry` reads the actor's box once, warns once per
blueprint on a stale table, logs the residual (WARN outside tolerance) and records it for
the episode's contact reports (`geometry`, optional within schema 1, so 021's reports and
readers are unaffected). `physical_verdict` and `run_suite --physical-verdict` put the
out-of-tolerance bodies in the `carla_contact` testcase's `system-out`; the verdict does
not change.

## Acceptance

- A declared truck, bus, motorcycle and bicycle each spawn as the closest CARLA body of
  their class; a car-sized box still spawns a Model 3.
- Every spawn logs its residual against the declared box, and over-tolerance entries reach
  the run's JUnit.
- The step-6 reproducer disagrees before and agrees after.
