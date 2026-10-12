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
5. [ ] **No change when off; no cost when on.** Starter scenarios unchanged with the param
   unset; `frame_stats` p95 with it on within noise of off (20-NPC bench, roadmap 016).
6. [ ] **Positive control.** A crafted scenario: a declared 12 m truck that `blueprint_map`
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
- With `gap_trace_dir` unset the bridge behaves exactly as before; with it set, frame-time
  p95 is unchanged within noise and no frame is dropped on the starter scenarios.
- The baseline table exists, with every starter-scenario finding classified.

### Results

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
