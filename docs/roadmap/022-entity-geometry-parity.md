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
   **Props may turn.** A `static.prop.*` has no driving direction, and scenarios declare
   one across the road as often as along it (`town01_pedestrian`'s barriers: 0.40 x 1.20 m,
   the 1.22 x 0.37 m street barrier turned). For the misc-object class each candidate is
   also tried with length and width swapped, the better orientation counts (for "fits" and
   for the score; the unturned one on a tie), and a turned winner is spawned turned 90°: the
   turn is part of the entity's stored origin offset (`OriginOffset.yaw`), so every
   later pose update keeps it and the CARLA footprint lies where SSv2's does. The residual
   is then reported in the entity's frame (the body's length against the declared width).
   CARLA quirk: a prop's `bounding_box()` is not in the actor's frame -- the street barrier
   reads 1.21 x 0.37 at yaw 0, 0.37 x 1.21 at yaw 90 and 0.60 x 1.12 at yaw 45 (a vehicle
   reads 4.79 x 2.16 at every yaw). So a prop's spawned box is read back into its own frame
   only at a yaw that is a multiple of 90° (swapped at 90/270); at any other yaw the
   table's box stands in for it and the stale-table check is skipped.
   Vehicles never turn: their length is their direction of travel.
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
5. [x] **Live.** A crafted scenario per subtype (car, truck, bus, motorcycle, bicycle, misc
   object) on Town01: each spawns its matched body, and the logged residual matches the
   table. One scenario carries all six, `scenarios/gap/town01_entity_bodies.xosc`: parked
   in the ego's lane behind it (x 205-280 on road 4, straight to x 320), each declared box
   picked to match one blueprint that is not its class default (car 3.70 m -> Audi A2, so
   the Model 3 default visibly loses); the ego drives off and the run exits at 10 s. The nine starter scenarios: same blueprints as before for every entity whose box
   fits a Model 3 (no behaviour change where none is due), all pass.
6. [x] **Disagreement closed.** A scenario that, before this phase, gives SSv2 PASS with a
   CARLA contact (or the reverse) under `--physical-verdict`; after it, both agree.
   The before-run is kept as the regression reproducer in `scenarios/gap/`.
   *Revised (2026-10-12): a motorbike, not a truck.* A truck's old body (the Model 3) is
   shorter than its box, so CARLA misses a contact SSv2 sees; 021 passes a missing expected
   contact by design (`csbExpectedContacts` permits, it does not require), so the only
   visible disagreement would be SSv2 FAIL / CARLA PASS, and the "after" state would be a
   scenario that fails in both worlds -- not something a suite can keep green. A motorbike's
   old body is 2.8 m *longer* than its box: CARLA touches what SSv2 clears. So
   `scenarios/gap/town01_motorbike_body.xosc`: the ego drives west as in
   `town01_rear_contact` (a parked ego cannot be had: the interpreter holds the storyboard
   until the managed ego is engaged, and an ego with no route never leaves
   `WAITING_FOR_ROUTE`). A blind motorbike (2.0 x 0.8 x 1.5 m, centre 0.6 m ahead of its
   origin) catches up at 8 m/s and, once SSv2's freespace gap to the ego is under 0.9 m,
   holds the ego's speed (relative target speed 0, continuous) for 5 s; SSv2 fails on a
   collision, else passes. CARLA's gap is SSv2's less ~0.17 m (the ego's Model 3 overhangs
   its declared rear by 0.15, a Ninja its declared front by 0.02). Before (`MotorbikeBody`
   = `vehicle.tesla.model3`, the pre-022 body, 1.4 m past the declared front): ~0.7 m into
   the ego, SSv2 PASS + CARLA contact. After (`kawasaki.ninja`): ~0.6 m clear, both PASS.

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

**Step 5 (2026-10-11/12).** Fork `a337a606`, csb `b5e0371` plus the prop turn, CARLA 0.9.16.

- `scenarios/gap/town01_entity_bodies.xosc`: PASS (2/2, 40 s). Every entity got the body its
  box was picked for, and the logged residual is the table's: car 3.70 m -> `audi.a2`,
  motorbike -> `kawasaki.ninja`, bicycle -> `diamondback.century` (-0.04 / -0.02 / +0.02),
  truck -> `european_hgv` (-0.06 / -0.01 / -0.04), bus -> `fusorosa` (+0.27 / +0.04 / +0.05),
  barrel -> `static.prop.barrel` (0.00). The contact report holds seven `geometry` records
  (the ego's included), all `fits`, no contacts.
- Starters, `run_suite --physical-verdict`, `basic/` + `awf/`: 9/9 PASS (ACC 28/28, AEB
  10/10). Every entity whose box fits a Model 3 kept it (the ego x42, `npc_rear`,
  `npc_route`); pedestrians kept `walker.pedestrian.0001`.
- UC-ACC `Npc1` declares 4.00 x 1.80 x **2.50** m: no car fits (2.50 m is a van's height),
  the closest is `jeep.wrangler_rubicon` (-0.13 / +0.11 / -0.62), noted in `system-out`.
  A change that is due; ACC still 28/28.
- **Regression:** `town01_pedestrian`'s `barrier_0`/`barrier_1` declare 0.40 x 1.20 x 1.00 m,
  the street barrier turned across the road, and now spawn as `static.prop.gnome`
  (0.37 x 0.38 x 0.88, -0.82 m in width) where they used to get the default
  `streetbarrier` (1.22 x 0.37 x 1.07). Two causes: the plane match is orientation-bound,
  so the barrier itself misses by 0.8 m on both axes; and when nothing fits, the closest
  score wins even if it is further off than the default. Fixed by letting props turn
  (design, item 3): the barriers keep the street barrier, spawned turned 90°. The first
  rerun still reported them 0.8 m out and warned the table stale (CARLA 0.37 x 1.21): a
  probe showed a prop's `bounding_box()` follows its yaw (design, the CARLA quirk), so the
  readback is now taken back into the prop's frame. Rerun: `town01_pedestrian` PASS, the
  barriers `streetbarrier` turned 90°, -0.03 / +0.01 / +0.07 m, no stale warning, no
  `system-out` notes; `town01_entity_bodies` PASS unchanged. The other seven starters
  spawn no prop, so the turn cannot reach them; their 9/9 run above stands.

**Step 6 (2026-10-12).** Both workspaces rebuilt on `carla` 0.15.0 (acb `5ae5334`), stacks
restarted. The first draft parked the ego with no route and failed in both variants with
`AutowareError ... WAITING_FOR_ROUTE` (the design note above); the driving version, run
twice with each body under `run_suite --physical-verdict`:

| Motorbike body | SSv2 | `carla_contact` |
|---|---|---|
| `vehicle.tesla.model3` (pre-022, `MotorbikeBody` forced) | PASS, PASS | FAIL, FAIL: "ego hit 'motorbike' (vehicle.tesla.model3)", 2 contacts, max impulse 432.5 N*s; `system-out` lists the body +2.79 / +1.36 / -0.01 m |
| `vehicle.kawasaki.ninja` (size match) | PASS, PASS | PASS, PASS |

The disagreement is reproduced and closed; `scenarios/gap/town01_motorbike_body.xosc`
stays as its regression reproducer (set `MotorbikeBody` to `vehicle.tesla.model3` to see
the old failure).

**022 complete (2026-10-12).** All three acceptance items hold: steps 5 and 6.

## Acceptance

- A declared truck, bus, motorcycle and bicycle each spawn as the closest CARLA body of
  their class; a car-sized box still spawns a Model 3.
- Every spawn logs its residual against the declared box, and over-tolerance entries reach
  the run's JUnit.
- The step-6 reproducer disagrees before and agrees after.
