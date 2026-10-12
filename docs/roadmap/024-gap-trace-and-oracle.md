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
  actor id 0 / `static.*`), with SSv2 frame, other actor, and impulse.

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

## Steps

1. [ ] **Docs.** This phase doc, [design/gap-campaign.md](../design/gap-campaign.md),
   README entry.
2. [ ] **Oracle geometry first, test-driven.** Oriented-box overlap and signed distance;
   synthetic traces for touching, separated, rotated, declared-vs-blueprint-size, offset
   applied twice / not at all. No CARLA.
3. [ ] **Trace format.** A JSON schema in `docs/design/gap-campaign.md` (appendix) and a
   reader the oracle and tests share.
4. [ ] **Bridge recorder.** Header at spawn, per-frame line from one snapshot, contacts from
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
