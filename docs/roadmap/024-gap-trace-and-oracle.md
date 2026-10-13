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
7. [ ] **Baseline.** All nine starter scenarios (`basic/` 7, one UC-ACC and one UC-AEB
   variant) plus the reproducers of 021-023, with the trace on; every finding triaged here
   as real gap, oracle bug, or tolerance to tune. That table is the hand-off to 025.
8. [ ] **Spike: SSv2's `random_test_runner` on CARLA** (added 2026-10-13, user). SSv2 ships
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
