# Phase 024: Gap Trace and Oracle

Measurement part of the [gap campaign](../design/gap-campaign.md): make every run produce a
machine-readable record of SSv2's world and CARLA's world side by side, and an offline
oracle that finds the frames where they disagree. Comes after the known gaps are fixed
(021-023), so its baseline measures what those fixes left; baselined on the nine starter
scenarios before any fuzzing.

**Source**: user direction 2026-10-11 ("fuzzy scenario campaign to exploit scenario runner
to Carla gap"); design agreed in discussion the same day.
**Depends on**: [014](014-feature-completeness.md) (`collision_monitor`, `OriginOffset`),
[020](020-starter-scenarios-and-suites.md) (starter scenarios, `run_suite`),
[021](021-physical-verdict.md) (the per-episode contact report; the trace's contact lines
extend it), [022](022-entity-geometry-parity.md), [023](023-npc-motion-parity.md).
**Feeds**: 025 (adaptive generator and exploit pipeline), 026 (parallel pipelines).

## Problem

- **The comparison is by hand.** `collision_monitor.rs` logs CARLA's contacts with the ego
  and an end-of-run table; a human reads it against the JUnit verdict. Nothing scales to
  hundreds of runs a night.
- **Only contacts are recorded.** The commonest disagreement leaves no contact at all: SSv2
  overlaps a declared 12 m truck box with the ego while CARLA's 4.7 m Model 3 passes clear,
  or the reverse. Seeing it needs both poses and both boxes every frame.
- **SSv2's half is not observable from outside.** `CollisionCondition` is evaluated inside
  the interpreter and only surfaces as a pass or a failure message; scenarios without the
  condition report nothing. The bridge, though, holds every input SSv2's check uses.

## Design

### Gap trace (bridge)

Recorded in the coordinator, which already has both halves:

- **Header, per entity, at spawn**: SSv2 name, kind (ego / vehicle / pedestrian /
  misc object), declared bounding box (center, dimensions) and `OriginOffset`; CARLA actor
  id, blueprint id, blueprint box from `actor.bounding_box()` (location offset, extent).
  Read once: boxes do not change.
- **Per frame**: sim time, SSv2 frame; per entity, the SSv2 pose (the intended pose for an
  NPC; for the ego, the pose the bridge reported back) and the CARLA transform and velocity,
  both in the ROS map frame (CARLA through the existing Y-flip conversion), raw -- the
  oracle applies offsets, so a wrong offset is visible in the trace instead of baked in.
- **Per contact**: every `collision_monitor` event, including static-world hits (other
  actor id null, kind `static`, as in the contact report), with SSv2 frame, other actor, and
  impulse.

Cost rules: CARLA state comes from **one world snapshot per tick**, never a per-actor RPC
(7-10 ms each, [ssv2-feature-completeness.md](../design/ssv2-feature-completeness.md) §4);
lines go to a **writer thread** over a bounded channel, never written on the ZMQ thread
(`/home` is the disk that stalled acb up to 25 s, CLAUDE.md "The log disk can stall a
node"). If the channel is full the frame is dropped and counted; the count is in the
episode's last line.

Format: JSON Lines, one file per Initialize: `<gap_trace_dir>/<UTC time>_<episode>.jsonl`.
The bridge never learns the scenario's name; the runner collects the files whose time falls
in its run window. Control: ROS param `gap_trace_dir`, empty = off (default), exposed as a
`simulation.launch.xml` argument. Diagnostic only: nothing in a reply to SSv2 changes.

### Oracle (`ros2 run csb_launch gap_oracle`)

`gap_oracle <trace.jsonl> [--junit result.junit.xml] [--out findings.json]`, Python, no
CARLA or ROS needed at run time. It checks the campaign's differential contracts C1-C4
([design](../design/gap-campaign.md#contracts)) as G1-G4. Each frame is judged in two geometries:

- **SSv2's**: 2D oriented boxes at the SSv2 poses with the declared boxes, the check
  `CollisionCondition` makes (`api.cpp:345`).
- **CARLA's**: 2D oriented boxes at the CARLA transforms with the blueprint boxes; plus the
  recorded contacts.

| Check | Reported when |
|---|---|
| **G1** overlap disagreement | ego vs an entity overlaps in one geometry and not the other. Reported per interval: frames, max penetration in one, min gap in the other, and the signed margin difference `d_ssv2 - d_carla` (the generator's feedback in part 2) |
| **G2** contact SSv2 cannot see | a CARLA contact with no SSv2 overlap within ±2 frames; every static-world hit |
| **G3** pose divergence | SSv2 intended vs CARLA actual (after offsets) beyond 0.2 m / 2° in plane or 0.5 m in z, sustained 3+ frames (spawn snapping, rejected teleports, walker lift) |
| **G4** verdict class | G1-G3 joined with the JUnit result: `PASS+contact`, `PASS+overlap-only-in-CARLA`, `FAIL(collision)+no-contact`, `FAIL(collision)+overlap-only-in-SSv2`, `agree` |

Three rules the table does not spell out, settled building it (step 6) on the step-4
`town01_rear_contact` trace, where a contact both worlds saw still produced 1-2-frame G1s at
its edges: (1) CARLA's side of G1 counts a recorded contact of the pair as overlap at its
frame and the `--contact-window` frames after it (the collision sensor reports the hit, not
every frame the bodies then rest a few mm apart: 1 mm at the onset frame there); (2) a
disagreement frame whose `|d_ssv2 - d_carla|` is within the ego's one-tick lag (its SSv2
pose is read back before the tick, measured per frame as the in-plane distance between that
pose, offset applied, and CARLA's) plus `--touch` (0.05 m) is timing, not geometry, and is
reported as info `lag_band` (the separation edge there: 0.27 m against a 0.44 m lag);
(3) G3 compares z box bottom to box bottom, since a CARLA walker's origin is its body centre
(0.65 m above SSv2's origin in the step-4 pedestrian trace, at the feet).
Two more from the step-7 baseline, both G3: (4) an entity SSv2 reads back from CARLA like
the ego -- a `simulator_autopilot` NPC, whose SSv2 pose is CARLA's of the frame before -- is
not judged on pose (info `read_back`, detected from the trace: while it moves, SSv2's pose
sits on CARLA's previous one), and its one-tick lag widens G1's band as the ego's does;
(5) a divergence run adjacent to a `contact_hold` run whose error only shrinks is the
tracker closing a sub-snap error after the hold (info `hold_recovery`).
NPC-NPC pairs are judged too and reported at low priority (they rarely change an ego
verdict). Output: `findings.json` per run -- including the run's **gap signature** (contracts
broken × entity class × geometry) and the min margin difference, which part 2 consumes as
feedback -- and one summary line per run for aggregate tables. Tolerances are flags with these defaults; the baseline tunes them.

### What 021-023 changed (2026-10-13)

Written before the known gaps were fixed; four corrections from what those phases found:

- **NPCs are physics bodies by default** (023, `npc_motion: tracked`). C3's 0.2 m / 2°
  budget is the tracking budget 023 measured against (p95 ≤ 0.007 m on UC-ACC's lead car),
  so G3 now catches tracking faults as well as spawn and walker-lift ones. Two windows are
  expected divergence, not findings, and are reported as *info*: the spawn frame (one snap
  by design), and frames within 20 of an ego contact, where a tracked NPC is held back
  unsnapped by design (`CONTACT_HOLD_FRAMES`; `npc_rear` reached 1.6 m error there). The
  trace marks both, so the oracle needs no copy of the bridge's rules. Under `kinematic`
  G3 is unchanged.
- **One snapshot per frame already exists** under `tracked`: `track_npc` reads one world
  snapshot and one `actors_by_ids` list per frame (023 step 4). The recorder reuses that
  view and takes its own only under `kinematic` or with no tracked NPC.
- **A prop's CARLA box follows its yaw** (022: a barrier read 0.37 x 1.21 m at 90°, 0.60 x
  1.12 m at 45°); vehicle and walker boxes do not. The header records the box in the
  actor's own frame through the same conversion 022's report uses
  (`prop_box_in_actor_frame`), so CARLA's geometry in the oracle is right for turned props.
- **The contact report already carries per-entity geometry and tracking rows** (021 schema
  1, extended in 022 and 023). The trace does not replace it: the report stays the
  per-episode summary `run_suite` joins into the verdict; the trace is the per-frame record
  the oracle needs. The header's box values come from the same code as the report's
  `geometry` rows, so the two cannot disagree.

## Steps

1. [x] **Docs.** This phase doc, [design/gap-campaign.md](../design/gap-campaign.md),
   README entry.
2. [x] **Oracle geometry first, test-driven.** Oriented-box overlap and signed distance;
   synthetic traces for touching, separated, rotated, declared-vs-blueprint-size, offset
   applied twice / not at all. No CARLA.
3. [x] **Trace format.** A JSON schema in `docs/design/gap-campaign.md` (appendix) and a
   reader the oracle and tests share.
4. [x] **Bridge recorder.** Header at spawn, per-frame line from one snapshot, contacts from
   `collision_monitor`'s queue; writer thread; `gap_trace_dir` param and launch argument.
   Unit test: recorder output parses with the oracle's reader.
5. [x] **No change when off; bounded cost when on.** Starter scenarios unchanged with the
   param unset; bridge processing p95 with it on at most +0.5 ms over off on the 20-NPC bench
   (roadmap 016), no frame dropped. Restated 2026-10-13 (user) from "within noise of off":
   the measured cost is small and constant (+0.05-0.15 ms p95 steady state, under 0.5 % of
   the 50 ms frame) but larger than the off runs' spread, so "within noise" could not hold.
6. [x] **Positive control.** A crafted scenario: a declared 12 m truck that `blueprint_map`
   pins to `vehicle.tesla.model3` (the override 022 keeps, so the mismatch is deliberate)
   passes the ego 1 m to the side. G1 must fire
   (overlap in SSv2's geometry only). `town01_rear_contact`: both geometries and the contact
   must agree (no G1, no G2).
7. [x] **Baseline.** All nine starter scenarios (`basic/` 7, one UC-ACC and one UC-AEB
   variant) plus the reproducers of 021-023, with the trace on; every finding triaged here
   as real gap, oracle bug, or tolerance to tune. That table is the hand-off to 025.
8. [ ] **Spike: SSv2's `random_test_runner` on CARLA** (added 2026-10-13, user; carried to
   [025](025-adaptive-generator-and-exploit-pipeline.md) step 2 on 2026-10-24). SSv2 ships
   a random tester (`test_runner/random_test_runner`: random ego goals, NPC count / distance
   / speed; stuck, timeout and collision reported to JUnit). It drives `traffic_simulator`
   directly, not OpenSCENARIO, over the same simulation interface the bridge serves, so
   CARLA can in principle be its backend -- a fuzzer for this campaign at no authoring cost,
   with the gap trace on and the oracle judging every run. Find out: (a) how it brings up
   Autoware (it launches it through the concealer; ours is reached through the relay), (b)
   which requests it sends that the bridge does not handle, (c) whether a short random
   session on Town01 runs end to end. Output: a go / no-go and the list of changes, as a
   finding in the baseline table; building it out belongs to 025 or 030 (once the agent API
   decouples the autopilot). Its known limits: no traffic lights, NPCs do not yield.

## Acceptance

- `gap_oracle` flags the positive control (G1) and stays silent on `town01_rear_contact`.
- With `gap_trace_dir` unset the bridge behaves exactly as before; with it set, bridge
  processing p95 rises by at most 0.5 ms and no frame is dropped (restated 2026-10-13 from
  "unchanged within noise").
- The baseline table exists, with every starter-scenario finding classified.

### Results

**2026-10-13, step 7 -- baseline: every finding classified; two oracle bugs fixed, no
tolerance changed.** Installed binaries at `d2746ca` (newer than every source, acb's
included; no rebuild). Simulation side restarted with `GAP_TRACE_DIR` (log
`play_log/bridge/2026-10-13_12-34-12`), one ego stack via the relay, Town01 (Town02 for the
episode change), `npc_motion: tracked`, `CARLA_MAPS=data/carla-autoware-bridge`, `run_suite
--physical-verdict`, one `run_suite` per scenario file so each trace maps to its run by time
window; every trace judged with `gap_oracle --junit`. AWF: one variant each, from scratch
copies with every `ScenarioModifier` list cut to one value (SSv2 then expands a single
variant): UC-ACC `vxi_ego` 8.333, `vxi_npc0` 5.556, `vx_target_npc0` 1.944; UC-AEB `vxi_ego`
8.333, `dxi_ego_pedestrian0` 30. `town02_episode_change` hit the known Town02 load timeout
(`load world 'Town02': time-out of 120000ms`) on the first try and passed on the rerun.
Counts below are after the oracle fixes (before -> after where they differ):

| scenario | verdict | frames | G1 | G2 | G3 | info | G4 |
|---|---|---|---|---|---|---|---|
| `town01_ego_drive` | PASS 2/2 | 732 | 0 | 0 | 0 | 0 | agree |
| `town01_engage_state` | PASS 2/2 | 673 | 0 | 0 | 0 | 0 | agree |
| `town01_pedestrian` | PASS 2/2 | 2270 | 0 | 0 | 2 | 0 | agree |
| `town01_rear_contact` | PASS 2/2 (23 contacts) | 303 | 0 | 0 | 1 -> 0 | 4 -> 5 | agree |
| `town01_simulator_autopilot` | PASS 2/2 | 730 | 0 | 0 | 4 -> 3 | 0 -> 1 | agree |
| `town01_traffic_light` | PASS 2/2 | 3436 | 0 | 0 | 0 | 0 | agree |
| `town02_episode_change` | FAIL (Town02 load timeout), rerun PASS 2/2 | 0 / 101 | 0 | 0 | 0 | 0 | unknown / agree |
| UC-ACC-001-0001, 1 variant | PASS 2/2 | 1353 | 0 | 0 | 1 | 0 | agree |
| UC-AEB-001-0001, 1 variant | PASS 2/2 | 1041 | 0 | 0 | 0 | 0 | agree |
| `gap/town01_truck_pass` (positive control) | FAIL 1/2 (`ssv2_collision_held`, by design) | 244 | 1 | 0 | 1 | 0 | FAIL(collision)+overlap-only-in-SSv2 |
| `gap/town01_entity_bodies` | PASS 2/2 | 331 | 0 | 0 | 0 | 0 | agree |
| `gap/town01_motorbike_body` | PASS 2/2 | 304 | 0 | 0 | 1 | 0 | agree |
| `bench/town01_npc_20`, run 1 | PASS 2/2 | 1202 | 0 | 0 | 74 | 0 | agree |
| `bench/town01_npc_20`, run 2 | PASS 2/2 | 1202 | 0 | 0 | 74 | 0 | agree |

No G2 anywhere: every ego contact (23, all in `town01_rear_contact`) had an SSv2 overlap
within ±2 frames, and no static hit by the ego. The only G1 and the only non-`agree` G4 are
the positive control's. Every G3 below repeated exactly where a scenario was run twice (bench
today ×2 and step 5 ×2: 74 each, identical intervals; the pedestrian's two yaw runs as in
step 4). Info, not triaged: `rear_contact` 2 `lag_band` (frame 243: difference 0.12 m, band
0.16; frame 273: -0.25, band 0.52), 2 `contact_hold` (245-272 max 1.61 m, 276-290 0.57 m),
1 `hold_recovery` (291-293, new); `simulator_autopilot` 1 `read_back` (`npc_autopilot`,
frames 124-726, every moving frame on CARLA's previous pose). Also seen, not a G finding: an
episode with no entities (`town02_episode_change`) never gets an `end` line until the next
`Initialize` (the recorder closes an episode on the last despawn), so the oracle marks it
INCOMPLETE; harmless.

Triage (the hand-off to 025):

| # | finding | class | evidence | proposed action / owner |
|---|---|---|---|---|
| 1 | G1 `truck_pass` ego-truck, frames 185-244 | **real gap** (modelling: declared box vs blueprint), by construction | SSv2 penetration 2.29 m, CARLA gap ≥ 0.40 m, `d_ssv2 - d_carla` -3.55 m; declared 12 m truck spawned as a 4.79 m Model 3 | none: it is the positive control. The class (`C1:vehicle:ssv2`) is 025's first target |
| 2 | G3 bench `npc_05`, frames 25-1202 (1178 frames, z 80.3 m) | **real gap** (bridge: tracked NPC vs the static world, snap into a collider), triggered by a **scenario artefact** (bench lane is a shoulder) | see "npc_05" below: on CARLA's 0.3 m shoulder lane, wheels on the curb (roll 3.6° from spawn), front hits the `CityLamp81` pole at x 360.94 at frame 25, a snap at 0.9 m error teleports it into the pole keeping roll/pitch/z, PhysX ejects it at vz 39.7 m/s; 4/4 runs | new tracking fix phase before 025: (a) never snap a blocked body into penetration -- hand it to kinematic (`set_simulate_physics(false)`) and follow SSv2, as before 023, flagged in the trace; (b) a snap resets roll/pitch and takes z from the ground; (c) NPC-vs-static contacts reach `collision_monitor` and the trace (an NPC G2); (d) `gen_npc_benchmark.py` skips lanelets narrower than ~2.5 m |
| 3 | 69 G3 on the bench (×2) + `simulator_autopilot` `npc_route` 450-462, 706-730 | **real gap** (bridge: tracker in corners) | all on 24-55° turns at 8 m/s; error is cross-track (0.24-0.58 m) with along-track ≤ 0.08 m, and always on the **outside** of the turn (69/69); yaw 1.7-3.6°. 023's budget (p95 ≤ 0.007 m) was measured on a straight | same fix phase: a curvature (centripetal) feed-forward in `npc_tracking::command`, gains re-measured on the bench's corners; C3's 0.2 m / 2° stays the target |
| 4 | 4 G3 bench on straights (`npc_11` 306-367, `npc_14` 970-994: yaw 2.4/3.3°, xy ≤ 0.16; `npc_18` 245-248, `npc_11` 880-882: 7-8° turns) | **real gap**, same as 3 | each follows a corner: the yaw error left at the exit decays at the yaw gain (2/s) | with 3 |
| 5 | G3 `pedestrian_0` yaw, frames 135-163 (174°) and 2206-2270 (108°) | **real gap** (bridge: walker control at rest), verdict-neutral | position ≤ 0.03 m throughout; standing still, the walker spins 18°/frame: the bridge normalises a noise-sized commanded velocity (> 1e-6 m/s) into `control.direction` and CARLA turns the walker toward it; at the end it turns away and stays at -161.6°. Both boxes are square (0.6 / 0.375 m), so no footprint changes | same fix phase: below ~0.05 m/s send speed 0 and SSv2's heading as direction |
| 6 | G3 `UC-ACC` `Npc1` 539-541 (0.57 m) | **real gap** (bridge: spawn at speed) | SSv2 adds `Npc1` at frame 537 already moving ~5.5 m/s; the body spawns at rest (0.5 m/s after one tick, +0.49 m/s per frame, the force bound) and is snapped at 542 | same fix phase: seed the body's velocity with SSv2's twist at spawn |
| 7 | G3 `truck_pass` truck 193-195 (0.71 m) | **scenario artefact** (speed step) | the hold steps 8 -> 1.6 m/s in one frame; the tracked body brakes at 0.49 m/s per frame and runs past until the snap at 196 | none in the bridge; `linear` dynamics in the scenario if it should be silent |
| 8 | G3 `motorbike_body` 155-159 (0.51 m) | **scenario artefact** (speed step) | SSv2 steps 4.4 -> 8 m/s at frame 153; the body accelerates at the bound; snap at 160 | none |
| 9 | G3 `simulator_autopilot` `npc_route` 114-116 (0.66 m) | **scenario artefact** (speed step) | 0 -> 6 m/s in one frame at 113; snap at 117 | none |
| 10 | G3 `simulator_autopilot` `npc_autopilot` 131-681 (551 frames, 0.40 m) | **oracle bug** | a `simulator_autopilot` NPC is read back like the ego: its SSv2 pose equals CARLA's previous one exactly (0.000 m p95), so the 0.39 m "error" is one tick at 8 m/s | fixed: rule 4 (`read_back`); G3 4 -> 3 |
| 11 | G3 `rear_contact` `npc_rear` 291-293 (0.35 -> 0.20 m) | **oracle bug** | the frames right after the `contact_hold` window; the error (0.33 m, under the 0.5 m snap distance) is closed by the tracker in 3 frames, by design | fixed: rule 5 (`hold_recovery`); G3 1 -> 0 |

No **tolerance to tune**: the G3 budget held its meaning in every case above (every
excursion it flagged had a cause), so the defaults stay. Oracle changes
(`src/csb_launch/scripts/gap_oracle`, test-first: 5 tests in `test_gap_oracle.py` seen
failing, then green; csb_launch pytest 149 passed): rules 4 and 5. On every trace of
steps 4-7 the counts are unchanged except those two (step-6 positive control still G1 1,
step-6 rear contact still silent, with one more info line, a 1-frame `hold_recovery`).

**npc_05** (finding 2). The bench puts it on lanelet 12083, which the converted Town01 tags
`subtype=road` but which is CARLA's road 7 lane 2, a 0.3 m **shoulder**, against the
sidewalk (centre y 324.46 in CARLA; the sidewalk's edge at 324.3). A 2.16 m Model 3 centred
there has its right wheels on the curb: roll 3.6° from the spawn frame. SSv2 drives it at
8 m/s westward; at frame 24 -> 25 its front (x ~ 361.2) meets the pole of street lamp
`CityLamp81` (`get_environment_objects`, x 360.94): velocity -8.0 -> +2.17 m/s, no other NPC
within 15 m, and no contact line (NPCs carry no collision sensor). The error grows to 0.9 m,
past `SNAP_DISTANCE` (0.5), and `track_npc` snaps it -- `snap_to = now.clone()` with x, y and
yaw replaced, so into the pole, keeping z, roll and pitch. PhysX resolves the penetration
upward: frame 27 vz 39.7 m/s, roll -75°, pitch -30°. Every later frame snaps x, y and yaw
again (in plane it tracks to 0.00 m) while z is left to physics and the tumbling attitude is
kept: z peaks at 80.3 m near frame 100, and the car lands on its side (roll -87°, z 1.5 m)
where it stays to the end. Same frames and numbers in all four bench runs. Cause: a
scenario artefact (a shoulder lanelet in the bench) exposing a real bridge gap (SSv2 has no
static world, the tracked body does, and the snap is a teleport into it). Fix as row 2; not
fixed here (findings are inputs to fixes).

**Step 8 (random_test_runner spike): not done.**

**2026-10-13, step 6 -- acceptance holds: G1 fires on the truck, rear contact silent.**
`gap_oracle` (`src/csb_launch/scripts/gap_oracle`, stdlib, imports `gap_geometry` /
`gap_trace` from its own directory, `install(PROGRAMS)`), test-first: `test_gap_oracle` 36
tests on synthetic traces written through `gap_trace.dumps` (seen failing on the missing
script, then green): G1 SSv2-only (declared 12 m truck on a Model 3: penetration 2.25,
CARLA gap 1.3, `d_ssv2 - d_carla` -3.55 m), CARLA-only (2 m bike on a Model 3), both clear,
both overlapping, two intervals, NPC pair at low priority, height gate, missing CARLA state,
contact as CARLA touch, lag band and its limit; G2 contact with/without SSv2 overlap within
+-2, static hit, non-entity actor, window flag; G3 sustained/brief/within tolerance, yaw, z,
walker origin, flags, spawn/contact_hold as info, ego excluded, missed as info; G4 every
class incl. `unknown` and `agree`, `carla_contact` testcase ignored; CLI exit 0 with
findings, non-zero on an unreadable or missing trace, a cut trace judged and marked. The
three rules above the table came from running it on the step-4 `town01_rear_contact` trace
before any live run: as first written it reported two SSv2-only G1s there (frame 421:
SSv2 -0.29 m, CARLA +0.001 m, the frame after the hit; frames 450-451: SSv2 -0.25/-0.03,
CARLA +0.012/+0.114, the ego's SSv2 pose 0.44 m behind CARLA's at 9 m/s), both edges of a
contact both worlds saw. Scenario `scenarios/gap/town01_truck_pass.xosc`: the truck declared
12 x 2.5 x 3.5 m (centre x 4.0) with `model3d="vehicle.tesla.model3"` -- 022's rule 2, the
scenario's own word, so no operator `blueprint_map` entry is needed; spawned as the Model 3
(4.79 x 2.16 m) -- closes from behind at 8 m/s and holds the ego's speed at 8.5 m reference
distance; SSv2's `CollisionCondition` held 3 s ends it with `exitFailure`, by design (the
"SSv2 fails on a collision CARLA never had" class). Behind, not beside: the boxes differ by
7.2 m in length but 0.2 m in width, so a 1 m lateral pass is clear in both geometries.
**Live** (bridge + relay restarted with `GAP_TRACE_DIR`, ego via relay, Town01, `npc_motion:
tracked`, `run_suite --physical-verdict`): `town01_truck_pass` FAIL 1/2 (SSv2:
`ssv2_collision_held` "[truck] colliding with ... ego"; `carla_contact` pass), 45 s;
`town01_rear_contact` PASS 2/2, 26 s. Oracle, with each JUnit:

```
20261013T020756.462Z_1.jsonl: G1 1 (1 ego, 0 npc) G2 0 G3 1 info 0 | G4 FAIL(collision)+overlap-only-in-SSv2 (fail_collision) | min d_ssv2-d_carla -3.57 m | 353 frames, 0 contacts | signature C1:vehicle:ssv2,C3:vehicle:pose,C4:FAIL(collision)+overlap-only-in-SSv2
20261013T020850.798Z_2.jsonl: G1 0 (0 ego, 0 npc) G2 0 G3 0 info 3 | G4 agree (pass) | min d_ssv2-d_carla -1.58 m | 262 frames, 23 contacts | signature -
```

Truck: G1 ego-truck frames 294-353 (60, 3.0 s), SSv2 penetration max 2.31 m, CARLA gap min
0.53 m (held 0.88-1.15), `d_ssv2 - d_carla` -3.57 m; the G3 is real, not the oracle's:
frames 303-305, 0.63 m in plane -- the hold's step from 8 to 2 m/s is beyond the tracked
body's braking (0.49 m/s per frame, ~10 m/s^2), so CARLA's Model 3 ran 0.2/0.44/0.63 m past
its intended pose until the snap at 306 (0.004 m); exactly `--sustain` frames, so a
`linear` hold would remove it. Rear contact: 23 contacts (frames 203-229, max 7942 N·s),
both geometries overlap through them; info = one `lag_band` frame (233: SSv2 -0.09, CARLA
+0.086, band 0.50) and two `contact_hold` windows (205-231 max 1.51 m, 235-250 0.51 m).
With the band off (`--touch -10`) the same trace reads G1 1 (frame 233), so the band is what
keeps it silent here; the contact rule was not needed this run (the onset frames agreed).
Also on old traces (not judged here, step 7's): the step-4 pedestrian trace has G3 on the
walker's yaw (174° for 29 frames, 108° for 68); the first step-5 20-NPC bench trace has
74 G3s: 73 tracking excursions of 3-70 frames (most 8-17), max 0.07-0.58 m in plane, and
`npc_05` diverging from frame 25 to the end, 80 m off in z (left the road). The oracle
takes 0.05 s on the 479-frame trace, 0.45 s on the 1202-frame 20-NPC one. `just test` 270
Rust passed / 1 skipped, 144 pytest; `just check` clean. Bridge log
`play_log/bridge/2026-10-13_10-06-58`.

**2026-10-13, step 5 -- not ticked: off unchanged, on costs ~0.05-0.15 ms, above the
off-run spread.** Build: acb submodule 3ed0a8a -> 8a5af50 (the `ad14899` pin), both
workspaces rebuilt (acb's, then this one), installed binaries newer than the sources; Town01,
`npc_motion: tracked` (default), ego via relay, `run_suite --physical-verdict`, one
simulation-side restart per mode. **Off**: `basic/` 7/7 PASS (every scenario 2/2,
`town02_episode_change` included, first try); the bridge's params show `gap_trace_dir: ""`,
no `Gap trace on` line, no `.jsonl` written anywhere on the volume. **On/off bench**
(`scenarios/bench/town01_npc_20.xosc`, 20 tracked NPCs, 1201 frames each, all PASS; "whole
run" `Frame budget` lines, ms):

| run | trace | processing p50/p95 | UpdateEntityStatus p50/p95 | tick p50/p95 |
|-----|-------|--------------------|----------------------------|--------------|
| off 1 | off | 0.35 / 0.45 | 0.24 / 0.34 | 9.17 / 10.34 |
| off 2 | off | 0.35 / 0.46 | 0.25 / 0.34 | 9.19 / 10.43 |
| on 1  | on  | 0.50 / 0.90 | 0.38 / 0.74 | 6.21 / 10.12 |
| on 2  | on  | 0.50 / 0.69 | 0.38 / 0.55 | 6.10 / 7.08 |
| off 3 (after the on runs) | off | 0.44 / 0.59 | 0.32 / 0.47 | 6.17 / 7.54 |

Every run has one processing max of 266-340 ms, frame 2 (20 spawns), on and off alike; no
`tick > 100 ms`. Noise as the step defines it (off 1 vs off 2) is 0.01 ms at p95, and both
on runs exceed it (+0.23 / +0.44 ms); even taking off 3 into the spread (0.45-0.59, CARLA's
tick having dropped from 9.2 to 6.2 ms p50 between sessions, so the host moved), on 2 is
0.10 ms above it and on 1 0.31 (its first two 200-frame windows read p95 1.67 / 1.06 with
tick p95 13.8 / 12.2, a disturbed host; its later windows 0.63-0.71). Steady-state
200-frame windows put the recorder's cost at +0.05-0.07 ms p50 and +0.05-0.15 ms p95 against
the adjacent off 3, under 0.5 % of the 50 ms step -- a real, small, constant cost (the
snapshot plus building a 20-entity frame on the ZMQ thread), not noise. So the criterion as
written fails; it passes if restated as an absolute bound (e.g. p95 +0.5 ms), which is the
user's call. **Traces** (on 1 / on 2): 7.26 MB each (~6 KB/frame), 1202 frames, 20 entities,
0 contacts, 0 frames with an entity missing from CARLA's snapshot, `dropped_frames: 0`,
`complete: true`; both load with `gap_trace.load_trace`. **Odd, not 024's**: the first bench
run (off, straight after `basic/` left CARLA on Town02) failed -- Initialize loaded Town01,
all 20 NPCs spawned at frame 2, and the next `UpdateEntityStatus` found every one "not in
the world snapshot" (`track_npc`, `self.world.snapshot()`), a failed frame; SSv2 ended the
scenario. The rerun with Town01 already loaded passed, as did every later run. Seen once;
suspect a stale snapshot after `load_world` with tracked NPCs. Bridge logs:
`play_log/bridge/2026-10-13_06-54-30` (off 1-2), `_07-38-56` (on), `_07-48-10` (off 3).

**2026-10-13, step 4.** Recorder `src/carla_scenario_bridge/src/gap_trace.rs`, wired into the
coordinator; param `gap_trace_dir` (config key, ROS param, `bridge.launch.xml` and
`simulation.launch.xml` args, `GAP_TRACE_DIR=... just run`), empty = off: no thread, no
snapshot, no line. **Pairing**: frame `k`'s SSv2 half is what `UpdateEntityStatus` `k`
carried (NPC) and returned (ego); its CARLA half is the world after tick `k`, i.e. the
snapshot the *next* `UpdateEntityStatus` reads -- under `tracked` the one `track_npc` already
takes (`TrackedView`), else one `world.snapshot()` of our own (client-side state, no RPC).
So a frame line is written one request late from a staged frame; the last frame is written
from a snapshot taken at the despawn (or Initialize / shutdown). The ego's SSv2 pose
therefore trails CARLA's by one tick by protocol (read back before the tick): 0.43 m p95 at
~8.5 m/s, 0.000 standing -- G3 must not judge the ego. Header from the same `actual` box as
022's `geometry` row (`prop_box_in_actor_frame`, offset turned back by
`prop_offset_in_actor_frame`), y flipped. `spawn` = first frame line naming the entity
(not inherited if that frame is dropped); `contact_hold` = tracked NPC with
`touched_within(CONTACT_HOLD_FRAMES)` when staged. Contacts: `collision_monitor` keeps
drained events (`keep_for_trace`/`take_traced`), so contacts the ego's `finish_run` drains
are kept too; kind from the headers, `static` for null id, else `unknown`. Episode ends
when the last entity despawns (SSv2 sends nothing until the next Initialize, which a suite's
last scenario never gets), else at Initialize / shutdown; CARLA restart marks it
incomplete. Writer: one thread, `sync_channel(512)`; full = frame dropped and counted,
other lines wait in order in a local backlog; flushed when the channel runs dry. Tests: 8
recorder tests (each record type, pairing/flags/contact kinds, full channel, off) plus one
that writes a trace through the real writer thread and parses it with `gap_trace.load_trace`
via `python3 -c` (one cross-language test instead of a golden file: a rename on either side
fails it; skips with a message without python3); config and prop-offset tests. `just test`
259 Rust passed / 1 skipped (pre-existing), 108 pytest; `just check` clean. **Live** (Town01,
`npc_motion: tracked`, ego via relay, `run_suite --physical-verdict`): `town01_rear_contact`
PASS 2/2, `town01_pedestrian` PASS 2/2; both traces parse. rear_contact: 479 frames,
0 dropped, 2 entities, 24 contacts (`npc_rear`, `vehicle`, frames 420-446, max 7786 N·s),
`contact_hold` on `npc_rear` frames 422-467 (46), `spawn` on each entity's first line,
`npc_rear` offset-applied error p50 0.000 / p95 0.015 m outside hold. pedestrian: 2292
frames, 0 dropped, 4 entities (2 turned barriers: origin_offset yaw π/2, CARLA box
1.21 x 0.37 m in actor frame against declared 0.4 x 1.2), 0 contacts, barrier error 0.000,
walker max 0.093 m. 0.3 MB and 2.1 MB. Step 5 (cost on/off) not measured yet.

**2026-10-13, steps 2-3.** Geometry and reader, test-first (tests seen failing on the
missing modules, then green), stdlib only: `src/csb_launch/scripts/gap_geometry.py` and
`gap_trace.py`, library modules installed beside the scripts (`install(FILES)`, not
executables) so `gap_oracle` imports them from its own directory, as `run_suite` and
`physical_verdict` already share code by sibling path; tests put `scripts/` on `sys.path`.
`signed_distance`: separating axes over the four edge normals; disjoint gives the exact
polygon distance (vertex-to-edge, so a corner-to-corner approach reads 1.41 m, not SAT's
1.0), overlap the minimum separating translation (containment counted right). Touching
counts as overlap, matching SSv2's `checkCollision2D` (boost `intersects`); its height gate
is `z_separated`. `actor_pose_from_entity` is the bridge's `entity_origin_to_actor_origin`
plus the spawn turn. `test_gap_geometry` 28 tests: separated, touching, penetration,
containment, corner-to-corner, 45/90° rotations, rigid-motion invariance, the declared 12 m
truck as a Model 3 (`d_ssv2 = -0.95`, `d_carla = +2.70`: C1 broken), `OriginOffset` applied
once (0 error), twice and not at all (1.5 m each, past G3's 0.2 m; not applied flips an NPC
1.0 m clear into a 0.5 m overlap), a turned prop. Trace schema 1 is the appendix of
[gap-campaign.md](../design/gap-campaign.md#appendix-gap-trace-format): five record types
(`episode`, `entity`, `frame`, `contact`, `end`), everything in the ROS map frame, raw;
contact fields named as the 021 contact report's (`other_id` null for the static world,
`other_kind` adds `static` and `unknown`); `spawn` / `contact_hold` are per-entity flags on
the frame line. The reader rejects unknown types, kinds, flags and schemas, non-finite
numbers, undeclared entity names, non-increasing frames, records after `end` and a missing
`end` (opt-out for cut-short traces), each as `TraceError` naming file, line and dotted
field. `test_gap_trace` 33 tests (7 valid incl. round trip through `to_json`/`dumps`, 26
malformed). All csb_launch tests: 108 passed. Not yet installed: `just build` picks up the
`CMakeLists.txt` change.
