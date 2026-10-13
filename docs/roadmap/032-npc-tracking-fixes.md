# Phase 032: NPC Tracking Fixes

Fix the five real gaps 024's baseline found in tracked NPC motion (023's default since
2026-10-13), worst first: a tracked NPC that meets the static world can be snapped into it
and thrown 80 m into the air. Every fix is judged by the instrument that found it: the gap
trace and `gap_oracle` on the same scenarios, before and after.

**Source**: user direction 2026-10-13 ("do 032"), from the triage table of
[024](024-gap-trace-and-oracle.md) step 7 (rows 2-6).
**Depends on**: [023](023-npc-motion-parity.md) (`npc_tracking`, `track_npc`),
[024](024-gap-trace-and-oracle.md) (gap trace, `gap_oracle`, the baseline it is measured
against).
**Before**: 025 (the generator would otherwise spend its runs rediscovering these).

## The gaps (024 baseline, 2026-10-13)

| # | Gap | Evidence | Seen in |
|---|---|---|---|
| A | **A snap into the static world.** A blocked tracked body is teleported into what blocks it, keeping its z, roll and pitch; PhysX resolves the penetration upward | bench `npc_05`: lamp pole at frame 25, vz 39.7 m/s, z 80.3 m, on its side to the end; 4/4 runs | bench (×4) |
| B | **Outside of the turn in corners.** Cross-track error 0.24-0.58 m, along-track ≤ 0.08 m, yaw 1.7-3.6°, always on the outside; the yaw error left at a corner exit decays at the yaw gain (2/s) | 69 + 4 G3 on the bench, `npc_route` 450-462 / 706-730 | bench, `simulator_autopilot` |
| C | **A standing walker spins.** A noise-sized velocity is normalised into `WalkerControl.direction`; 18°/frame while position stays within 0.03 m | `pedestrian_0` yaw 174° and 108° | `town01_pedestrian` |
| D | **An NPC added while moving spawns at rest.** SSv2 adds it at ~5.5 m/s; the body reaches 0.5 m/s after a tick, +0.49 m/s per frame (the force bound), and is snapped | UC-ACC `Npc1` 539-541, 0.57 m | UC-ACC |
| E | **NPC contacts with the world are invisible.** NPCs carry no collision sensor, so A left no contact line, and an NPC-vs-static G2 cannot exist | `npc_05` | bench |

Also from the baseline, not the bridge's: the bench's generator put `npc_05` on lanelet
12083, tagged `road` in the converted Town01 but CARLA's 0.3 m shoulder against the
sidewalk (the scenario artefact that exposed A).

## Design

**A -- never snap into an obstacle; a snap restores attitude and ground.**

- A snap sets roll and pitch to SSv2's (zero for a vehicle on Town01's flat roads) and z
  from the ground under the target (the existing ground-projection path spawns use), not the
  body's current values; the body's velocity is set to the command's.
- **Blocked**: a body is blocked when, for a frame, its velocity along the commanded
  direction is below a fraction of the command (or reversed) while the command is above a
  minimum speed and it is not in contact with the ego (which 023 already handles by holding
  without a snap). A blocked body that would be snapped is not snapped into place: it is
  handed to **kinematic** for the rest of the episode -- physics off, teleported to SSv2's
  pose every frame, exactly 023's `kinematic` mode for that one actor -- so SSv2's pose is
  kept and nothing is ever resolved by PhysX. Logged once, counted in the NPC's tracking
  statistics (`fallback` frame), and flagged in the trace (frame flag `kinematic`; trace
  schema 1 gains the flag: reader, appendix and oracle together -- G3 treats a kinematic
  NPC as exact).
- The threshold values are named constants, chosen from the bench's `npc_05` frames and
  checked against the bench's other 19 NPCs (no fallback for an unblocked NPC).

**B -- curvature feed-forward.** `npc_tracking::command` adds the centripetal term the
corner needs: the target's curvature (from SSv2's yaw rate over speed, already computed for
the yaw-rate fix) feeds the steering feed-forward and the lateral component of the
commanded velocity, so the body is not left to correct a lateral error the corner keeps
creating. Gains re-measured on the bench's corners (24-55° at 8 m/s) and the 023 spike's
7 m-radius corner. C3's 0.2 m / 2° stays the target.

**C -- a standing walker stays still.** Below a speed floor (~0.05 m/s, a named constant)
the walker gets speed 0 and SSv2's heading as its direction instead of the normalised noise.

**D -- spawn at speed.** A tracked NPC is spawned with SSv2's twist at spawn applied as its
initial velocity (`set_target_velocity` right after spawn, before the settling tick), so an
entity SSv2 adds moving starts moving.

**E -- NPC contacts.** Each tracked NPC vehicle gets a collision sensor like the ego's; its
events reach `collision_monitor` (kind of the other actor, impulse) and the gap trace's
contact lines, so the oracle can report an NPC-vs-static contact (G2 at low priority, as
NPC-NPC pairs are). Sensors follow the never-destroy-a-vehicle-with-sensors rule
(`destroy_with_children`). Its cost -- one sensor per NPC, 20 on the bench -- is measured;
if it costs more than the frame budget allows, E stays opt-in behind a parameter.

**Bench generator.** `gen_npc_benchmark.py` skips lanelets narrower than ~2.5 m (by CARLA
lane width at the spawn, not the Lanelet2 tag), and the bench is regenerated. The old bench
stays as `scenarios/gap/` reproducer of A (the shoulder start is what makes it one).

## Steps

1. [x] **Docs.** This phase doc; README entry.
2. [x] **A, test-driven.** The blocked rule and the snap's attitude/ground as pure
   functions in `npc_tracking` with tests (a blocked body never snapped into place, an
   unblocked one snapped upright on the ground); the kinematic hand-over in `track_npc`; the
   trace flag (schema, reader, oracle, tests). Live: the old bench ×2 -- `npc_05` falls back
   to kinematic and stays on the road (z within 0.5 m of the ground for the whole run), no
   other NPC falls back.
3. [ ] **C and D.** Walker speed floor; spawn at speed. Live: `town01_pedestrian` (the yaw
   G3 gone), the UC-ACC variant of 024's baseline (the `Npc1` G3 gone).
4. [ ] **B.** Curvature feed-forward with tests (a constant-radius target tracked within
   budget at 8 m/s in the pure model); gains on the bench. Live: bench ×2 and
   `town01_simulator_autopilot`.
5. [ ] **E.** NPC collision sensors, the contact lines, the oracle's NPC-vs-static G2; cost
   on the bench with and without.
6. [ ] **Bench.** Generator skips narrow lanes; regenerated bench; the old one kept as A's
   reproducer.
7. [ ] **Re-baseline.** 024's baseline set rerun, traced and judged; the before/after table
   in this doc.

## Acceptance

- On 024's baseline set, rows 2-6 of its triage table are gone: no G3 on the bench beyond
  the speed-step artefacts' kind, `npc_05`'s reproducer stays on the road, no walker yaw
  G3, no `Npc1` spawn G3; every starter scenario still passes, and the truck-pass control
  still fires G1.
- No NPC falls back to kinematic in any starter scenario or on the regenerated bench.
- Frame budget: bridge processing p95 on the bench within +0.5 ms of 024's tracked figures
  (E's sensors included, or E opt-in).

### Results

**2026-10-13, step 2 -- A done: `npc_05` falls back at frame 27 and stays on the road; bench
G3 74 -> 73.** `npc_tracking`: `blocked` (rule), `act` (a snap of a blocked body becomes
`Action::Fallback`), `snap_attitude` (SSv2's roll/pitch, z = ground under the target + the
origin's lift over the box bottom, SSv2's z without ground), `TrackingStats.fallback`; tests
first (6 new, 276 Rust passed / 1 skipped, 151 pytest, `just check` clean). `track_npc`: a
snap now sets roll/pitch/z as above (walkers keep `walker_teleport_z`) and zero vz; a
fallback turns physics off, teleports to SSv2's pose, moves the actor from `tracked_npcs` to
`kinematic_fallback` (the kinematic path then places it every frame and counts `fallback`),
logs once. Contact report row `fallback` (`serde(default)`), trace flag `kinematic` under
schema 1 (reader, appendix, oracle: G3 `kinematic` is info), old traces read unchanged.
**Two departures from the design text, both from the data.** (1) The rule is measured
against the velocity the bridge commanded *last frame* (force-bounded), not SSv2's: against
SSv2's 8 m/s every bench NPC reads 0.12 of it at frames 3-4 (starting at the force bound) and
would have fallen back. (2) The state latches: blocked stays set until the body moves along
SSv2's velocity at the same fraction, because once held the bounded command follows the body
-- the first build, unlatched, saw `npc_05` blocked at 25, clear at 26 (commanded +1.67 m/s,
moving +1.67), snapped into the pole at 26 (z 2.05 m, yaw 39° at 27) and only then fell back.
Constants: `BLOCKED_FRACTION` 0.5 (024's bench traces replayed through the command law: the
19 free NPCs' worst frame keeps 0.94 of the command, `npc_05` at 25 -0.27, -8.0 -> +2.17
m/s), `BLOCKED_MIN_SPEED` 1.0 m/s. Replaying all fifteen 024 step-7 traces trips only
`npc_05` (25-27) and `npc_rear` in `rear_contact` (243, 288: ego contacts, excluded by the
contact term). **Live** (simulation side traced, one ego stack via the relay, Town01,
`run_suite --physical-verdict`, bridge log `play_log/bridge/2026-10-13_14-37-22`): bench ×2
PASS 2/2 each, 1202 frames, `npc_05` blocked and handed over at frame 27 in both (warning in
the `frame=27` span; `kinematic` from trace frame 27 to the end), frames 25-26 on the curb
(z +0.08 m), from 27 at SSv2's pose; `npc_05` max |z_carla - z_ssv2| 0.082 m over the run
(024: 80.3 m), every NPC ≤ 0.249 m; no other NPC flagged `kinematic`. `gap_oracle --junit`:
G1 0 G2 0 **G3 73** (74 before: `npc_05`'s 25-1202 gone, its 2 frames under `--sustain`)
info 0, G4 agree, both runs identical. `town01_rear_contact` PASS 2/2, 378 frames, 23
contacts, report `npc_rear` snaps 0, fallback 0; oracle G1 0 G2 0 G3 0, info 4 (2
`lag_band`, 2 `contact_hold` 321-347 1.45 m / 352-363 0.39 m): contact hold unchanged. A run
on the unlatched first build had a 1-frame ego G1 at a contact edge (margin difference
0.23, band 0.11) not seen on the final build. The first bench attempt failed in Initialize
(`load world 'Town01'` timeout with CARLA on Town02, 024's known one) and was rerun.
