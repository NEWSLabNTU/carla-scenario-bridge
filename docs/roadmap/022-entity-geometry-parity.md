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
- **The subtype is ignored.** Every spawn carries `EntitySubtype` (CAR, TRUCK, BUS,
  TRAILER, MOTORCYCLE, BICYCLE, PEDESTRIAN) and the declared bounding box; neither feeds
  the choice.
- **The mismatch is silent.** Nothing compares the declared box with the spawned actor's
  `bounding_box()`, so even a deliberate `blueprint_map` choice gives no sign of how far off
  it is.

## Design

### Blueprint box catalogue

CARLA's API gives a blueprint no box; only a spawned actor has one. So the boxes come from a
table, generated once per CARLA version and shipped in the package:

- `scripts/measure_blueprint_boxes.py`: spawns every `vehicle.*`, a sample of `walker.*`, and
  every `static.prop.*` once, off the road, reads `bounding_box()` (extent, location) and
  the blueprint's `base_type` / `number_of_wheels`, destroys it, writes
  `config/blueprint_boxes_0.9.16.yaml`. Sensors none, so nothing is left behind.
- The bridge loads the table for its build's `carla::CARLA_VERSION`. Missing table: the
  bridge logs once and keeps today's behaviour.
- **Every spawn checks the table against the actor CARLA returned** (one `bounding_box()`
  read, at spawn only). A difference above 5 cm is a stale table, logged at WARN naming
  the script to rerun.

### Choice, in order

1. `blueprint_map[model3d]` -- the operator's explicit word, as now.
2. `model3d` that is a CARLA blueprint -- the scenario's explicit word, as now.
3. **Size match**: candidates are the table entries of the entity's class -- vehicle
   subtype to `base_type` (CAR → car, van; TRUCK and TRAILER → truck; BUS → bus;
   MOTORCYCLE → motorcycle; BICYCLE → bicycle; UNKNOWN → all four-wheeled), misc object to
   `static.prop.*` -- and the winner minimises the summed relative error of length, width,
   height against the declared dimensions, length weighted 2 (it is what longitudinal
   scenarios measure). Ties go to the current default, so a declared box that already fits
   a Model 3 keeps a Model 3.
4. The kind's default, as now (no table, no candidates).

The **ego is excluded** from 3: its blueprint decides acb's sensor mounts and vehicle
parameters, so it is chosen by the ego stack's configuration, not by the scenario's box.
It still gets the spawn-time comparison below. **Pedestrians** are excluded too: CARLA's
walkers are within a few centimetres of each other and none matches SSv2's default
pedestrian box; the comparison reports the difference.

### Residual mismatch, reported

At spawn, for every entity: declared dimensions, chosen blueprint, actual extent, and per
axis the difference. Logged at INFO; above a tolerance (default 0.3 m on any axis) at
WARN. The same per-entity records go into 021's contact report as a `geometry` list, and
`physical_verdict` copies entries over tolerance into the `carla_contact` testcase's
`system-out` -- information, not a failure: a declared 16 m articulated bus that CARLA has
no body for is a fact of the scenario, and the contact testcase already fails if it
matters.

`OriginOffset` is unchanged: it moves the pose reference by the declared center, and the
chosen body is positioned the same way.

## Steps

1. [ ] **Docs.** This phase doc.
2. [ ] **Catalogue script and table.** Run on CARLA 0.9.16 (Town01); commit the YAML; check a
   few entries against a live `bounding_box()` by hand.
3. [ ] **Choice, test-driven.** A pure `choose_by_size(class, declared, table)` with tests:
   12 m truck → the longest truck; 4.5 m car → Model 3 (tie rule); motorcycle; unknown
   subtype; empty table; misc-object prop by size; ego and pedestrian never size-matched.
4. [ ] **Bridge wiring.** Table loading by CARLA version; the chain above in `spawn_entity`;
   the spawn-time comparison and stale-table warning; `geometry` in the contact report and
   `physical_verdict`'s `system-out`.
5. [ ] **Live.** A crafted scenario per subtype (car, truck, bus, motorcycle, bicycle, misc
   object) on Town01: each spawns its matched body, and the logged residual matches the
   table. The nine starter scenarios: same blueprints as before for every entity whose box
   fits a Model 3 (no behaviour change where none is due), all pass.
6. [ ] **Disagreement closed.** A truck scenario that, before this phase, gives SSv2 PASS
   with a CARLA contact (or the reverse) under `--physical-verdict`; after it, both agree.
   The before-run is kept as the regression reproducer in `scenarios/gap/`.

## Acceptance

- A declared truck, bus, motorcycle and bicycle each spawn as the closest CARLA body of
  their class; a car-sized box still spawns a Model 3.
- Every spawn logs its residual against the declared box, and over-tolerance entries reach
  the run's JUnit.
- The step-6 reproducer disagrees before and agrees after.
