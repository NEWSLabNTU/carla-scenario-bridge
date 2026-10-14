# Phase 031: Maps for Scenarios -- deferred

**Status: deferred** (user, 2026-10-13: "defer all stage 1-3 for now, keep them noted").
Nothing here is scheduled. This doc keeps the analysis so the phase can start from it.

The user need behind it: building an Autoware map of your own is hard, and a scenario is
often designed first, with no map that fits it at hand. Three users, three stages:

| User | Starts with | Pain today | Stage |
|---|---|---|---|
| Wants a CARLA town we do not cover | a town name | no tool; our maps are TUM's converted pack (Town01/02/03/05/10, LGPL-3.0, download link dead), and converting a town means Lanelet2, point cloud, projector and signal table by hand | 1. Build |
| Has a scenario idea | a road requirement | must know the towns, find a spot by hand (`carla_poses`), and hope it fits | 2. Find |
| Needs a road no town has | a road shape | no path short of authoring a CARLA map | 3. Generate |

Decisions taken in the discussion (user, 2026-10-13): no map fingerprint in scenarios for
now; stages 2 and 3 are deterministic tools, with an LLM at most an optional front end.

## Stage 1: build Autoware maps from any CARLA town

`map_build --town <Town> --out <map dir>`: OpenDRIVE from CARLA
(`world.get_map().to_opendrive()`), converted to Lanelet2; a point cloud from a CARLA lidar
carried along every driving lane (teleported every few metres, voxel-downsampled); the
projector (CARLA frame, origin zero); `--generate-signal-table`; then the gates. Users build
maps locally, which also sidesteps the dead TUM link and the map-publishing decision.

**Open-source status (2026-10-13):**

- OpenDRIVE -> Lanelet2: CommonRoad Scenario Designer (`crdesigner`, TUM; its predecessor
  `opendrive2lanelet` is deprecated) works "generally" but produced ways with far too many
  nodes, enough to stall Autoware (Autoware ODD WG discussion #6019, 2025-04); a
  post-processing proof of concept exists (`TranHuuNhatHuy/opendrive-lanelet-conversion`,
  tested on CARLA's OpenDRIVE test files, last pushed 2026-07); an OpenDRIVE-lane to
  lanelet mapping file is implemented on a branch (odd-usecase-scenario #118, PR pending).
  Vector Map Builder (TIER IV, web) is the manual fallback.
- Point cloud from a CARLA lidar: standard practice, simple.

**Defects to expect** -- the TUM pack, made by such a conversion, had all of these, found
and repaired by hand here:

1. Traffic lights untyped (`subtype=""` / `type=""`) in 4 of 5 towns, 152 lights: SSv2
   rejects the ids, Autoware sees no signals (009; `repair_lanelet_traffic_lights.py`).
2. Lanelets without `subtype` (65 of Town01's 300, kerb strips): `map_based_prediction`
   aborts, Autoware does not route over them (010, 020; `repair_lanelet_subtypes.py`).
3. Node bloat per way (#6019).
4. Lanelet ids unrelated to CARLA lane ids (the UC-ACC remapping, 020).
5. Autoware tags converters tend to drop: `turn_direction` on junction lanelets, speed
   limits, stop lines tied to lights, crosswalks (separate objects in OpenDRIVE).

**Gates**: Autoware's Lanelet2 map validator (`autoware_lanelet2_map_validator`, not yet run
on our maps), 028's map check, a live smoke test (localize, route, drive), and a lanelet-by-
lanelet comparison against the TUM pack for the five towns it covers.

**Estimates (not measured):** per town once built, minutes to tens of minutes unattended
(point cloud ~10-30 min for a Town01-sized town; smoke test ~10 min). Building the pipeline,
~2-4 weeks: a 2-3 day spike (crdesigner + post-processing on Town01 and Town05, scored
against the defects, the validator and the TUM pack -> go / no-go with a measured repair
list), repair passes ~1 week, recorder and `map_build` ~3-5 days, all 0.9.16 towns
validated and documented ~1 week. If the converter scores poorly, the fallback is our own
OpenDRIVE -> Lanelet2 converter restricted to what CARLA towns use (the larger half).

**Built by hand once (Town04, 2026-10-14, for the crdt-tensor testbed's ramp merge).** The
steps stage 1 would automate, with the two tools that came out of it now in `scripts/`:

1. Lanelet2 from CARLA's own `Town04.xodr`, offline (`carla_map_gen --xodr`, CommonRoad
   Scenario Designer 0.8.5 in a uv environment), then `repair_lanelet_subtypes.py` (52
   lanelets) and `repair_lanelet_local_coords.py`.
2. `repair_lanelet_far_traffic_lights.py`: the converter linked 69 of 128 traffic-light
   elements to lanelets whose stop line is more than 60 m from every light the element
   refers to (one at the ramp's foot pointed at a light 400 m away), and Autoware stopped at
   the merge for a signal that never comes. The lanelets stop referencing them (defect 6
   below). A dry run on the TUM pack's Town05 finds 2 of 54 such; not applied.
3. `densify_pcd_along_lanes.py`: `carla_pcd_gen` samples at spawn points only, which left
   the ramp bare (no point within 20 m of its middle) and NDT lost the vehicle there. Eight
   lidars carried along every driving lane every 6 m, merged at 0.2 m: 3.0 M -> 21.0 M
   points. This is stage 1's point-cloud step.
4. `--generate-signal-table` with CARLA freshly reloaded: after the collector CARLA reported
   no lights until `reload_world`.

Defect 6, for the list above: traffic-light elements linked to a stop line far from their
lights (step 2).

Defect 7 (found 2026-10-14 on the TUM pack's Town03 by the crdt-tensor testbed): road
lanelets that start at fresh nodes exactly where their predecessor ends. Lanelet2's routing
graph follows a lanelet only to one starting at the same point ids, so no route led from
the roundabout's north entry to its south exit. `scripts/repair_lanelet_joins.py` merges
the nodes of such meeting road lanelets (2 nodes in Town03, none in the other five towns);
stage 1 needs the same check after any conversion.

## Stage 2: find a place for a designed scenario

`find_place --spec needs.yaml`: a per-lanelet feature index of every local map, built once
(neighbours and their direction, length, curvature, junction type, signals, crosswalks,
speed limit; line of sight from CARLA ray casts against the town's meshes, for "view
blocked" use cases), queried by a structured spec, ranked by rules (length margin, distance
to the next junction, nothing nearby the scenario did not ask for), each candidate with a
top-view preview; the result feeds the scenario-writing skill (028 step 2). 028 step 8
(location sweeps) asks the same index the other way round.

```yaml
ego_road: {lanes_same_direction: 2, shape: straight, length_min: 120, junction: none}
neighbours: {left: same_direction}
extras: {signal_ahead: false}
```

## Stage 3: generate a road no town has

A road description (segments, lanes, junction templates, grade) -> OpenDRIVE -> CARLA
`generate_opendrive_world` (procedural road meshes, no buildings) -> stage 1's pipeline.
Risk: a featureless world gives NDT little to match; acb's direct localization is the
fallback on such maps, to be measured. Camera perception sees nothing town-like.

```yaml
segments:
  - {type: straight, length: 200, lanes: [2, 1]}
  - {type: junction, kind: T, branch: right, signals: false}
  - {type: straight, length: 150, lanes: [1, 1], grade: 0.06}
```

## LLMs and models

None in the tools. An LLM may fill the YAML from a use case's prose (through the
scenario-writing skill) and never emits lanelet ids or coordinates; the spec is saved next
to the scenario so a run reproduces without it; ambiguity becomes a question to the user.
Learned models only later, if ever: realistic surroundings for generated roads, ranking
from accepted places.

## Sources

- Autoware ODD WG, "OpenDRIVE to Lanelet2 conversion", discussion #6019.
- CommonRoad Scenario Designer; `opendrive2lanelet` (deprecated).
- "A Workflow for Map Creation in Autonomous Vehicle Simulations", arXiv 2508.16856 (CARLA
  lidar PCD + OpenDRIVE -> Lanelet2 for Autoware).
- This repo: 009 (traffic-light defects), 010 / 020 (subtype defects), 020 (UC-ACC lane
  remap).
