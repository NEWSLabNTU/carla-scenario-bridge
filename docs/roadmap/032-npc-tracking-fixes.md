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
| B | **Outside of the turn in corners.** Cross-track error 0.24-0.58 m, along-track ≤ 0.08 m, yaw 1.7-3.6°, always on the outside; a yaw error is left at a corner exit (024 read it as decaying at the yaw gain, 2/s; step 4 found it does not decay at all -- the tyres cancel the commanded turn) | 69 + 4 G3 on the bench, `npc_route` 450-462 / 706-730 | bench, `simulator_autopilot` |
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

**B -- curvature feed-forward.** As built (step 4), from the bench's traces: the outside
error was the **force bound** charging the turn itself -- SSv2's 6-7 m corners at 8 m/s need
9-11 m/s2 of centripetal change (12-18 at its entries) against the 10 m/s2 bound, so the bound
spent itself turning the velocity and the position correction got nothing -- and the **yaw
error was never steered**: the tyres undo a set angular velocity within the tick, so a yaw
correction carried only by `set_target_angular_velocity` did nothing. So
`npc_tracking::bounded_command` measures the bound from the body's velocity carried through
the target's turn (`turned`: rotated by SSv2's yaw rate x dt, the centripetal term fed
forward, never adding speed), and `command` steers from the commanded yaw rate (SSv2's +
`k_yaw` x error) instead of SSv2's alone; `VEHICLE_GAINS.yaw` 2 -> 4/s. The commanded velocity
already carried SSv2's lateral component (`world_twist`). C3's 0.2 m / 2° stays the target.
(First written as: the target's curvature, from SSv2's yaw rate over speed, feeds the
steering feed-forward and the lateral component of the commanded velocity, so the body is not
left to correct a lateral error the corner keeps creating; gains re-measured on the bench's
corners and the 023 spike's 7 m-radius corner.)

**C -- a standing walker stays still.** Below a speed floor (~0.05 m/s, a named constant)
the walker gets speed 0 and SSv2's heading as its direction instead of the normalised noise.

**D -- spawn at speed.** An entity SSv2 adds moving starts moving. As built (step 3): the
spawn request carries no speed -- SSv2 hard-codes `set_initial_speed(0.0)`
(`traffic_simulator/api/api.hpp:210`, an `@todo`) and a pedestrian spawn has no speed field
-- so nothing can be applied at spawn. SSv2's first status for the new entity is the spawn
pose with a twist already at speed; the body's first tracked frame applies that frame's
command without the force bound, when SSv2 passes the twist through. An NPC SSv2 still
holds before `npc_logic_started` starts at rest, bounded, as before. Walkers need nothing:
`WalkerControl` sets their speed directly. (First written as a velocity set right after
spawn, before the settling tick.)

**E -- NPC contacts.** Each tracked NPC vehicle gets a collision sensor like the ego's; its
events reach `collision_monitor` (kind of the other actor, impulse) and the gap trace's
contact lines, so the oracle can report an NPC-vs-static contact (G2 at low priority, as
NPC-NPC pairs are). Sensors follow the never-destroy-a-vehicle-with-sensors rule
(`destroy_with_children`). Its cost -- one sensor per NPC, 20 on the bench -- is measured;
if it costs more than the frame budget allows, E stays opt-in behind a parameter. As built
(step 5): parameter `npc_contact_sensors` (config key, ROS param, launch arg, `just run`'s
`NPC_CONTACT_SENSORS`), default **on** -- within budget. Walkers get none. Each NPC sensor is
stopped and destroyed by csb (its listener) before its vehicle on every destroy path
(despawn, teardown at Initialize/shutdown; dropped unspoken when CARLA is unreachable or
restarted, as the ego's is), and `destroy_with_children` still backs it. An NPC sensor's
events are kept out of everything that judges the ego: not in the ego's log or the contact
report's `contacts` (so `physical_verdict`'s `carla_contact` is unchanged) but in a separate
`npc_contacts` list (`serde(default)`, schema 1), and not in the ego-contact window
(`touched_within`) that holds a tracked NPC. Trace: the contact line gains optional
`subject` (the NPC's name; absent = the ego's sensor, schema 1). Oracle: an NPC-side contact
is G2 priority `low` (signature `C2:npc_static` / `C2:npc_pair`), both NPCs' reports of one
pair are one finding, an NPC's contact with the ego is left to the ego's sensor (no second
count, not used in G1), and only the ego's G2 enters G4. Found live: CARLA 0.9.16 reports
the static world as an actor with id 0 (`static.pole`), not as no actor, so every world hit
-- the ego's too -- read as `unknown` actor 0; csb now maps id 0 to the static world.

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
3. [x] **C and D.** Walker speed floor; spawn at speed. Live: `town01_pedestrian` (the yaw
   G3 gone), the UC-ACC variant of 024's baseline (the `Npc1` G3 gone).
4. [x] **B.** Curvature feed-forward with tests (a constant-radius target tracked within
   budget at 8 m/s in the pure model); gains on the bench. Live: bench ×2 and
   `town01_simulator_autopilot`.
5. [x] **E.** NPC collision sensors, the contact lines, the oracle's NPC-vs-static G2; cost
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

**2026-10-14, step 5 -- E done: `npc_05`'s lamp pole is now a contact line and a low-priority
G2; NPC sensors on by default (bench processing p95 0.96-1.08 ms with, 0.92 without).**
`collision_monitor`: `RawCollision.subject`, `attach_npc` / `detach_npc` /
`detach_all_npcs` / `forget_npc`, NPC events routed to per-NPC logs (INFO), never the ego's
log or `touched_within`; `actor_or_world` (id 0 = static world). `contact_report`:
`npc_contacts` (`ReportNpcContact`, flattened, `serde(default)`). `gap_trace`: contact
`subject` (skipped when `None`). Coordinator: attach after a tracked `Npc` spawn, detach
before the despawn's `destroy_with_children` and before teardown; forget when CARLA is
unreachable. `gap_trace.py` (`Contact.subject`, `ego_side`), `gap_oracle` (G2 low for NPC
side, pair dedupe, NPC-ego skipped in G2 and G1, G4 and `no-contact` on ego contacts only,
counts `G2_ego` / `G2_npc`, summary `G2 n (a ego, b npc)`), appendix. Tests first: 6 Rust
(report separation, routing/window, NPC warn line, actor 0, trace `subject`, config) + the
Python reader in the cross-language test; 9 pytest (reader `subject`, G2 NPC-static / NPC-ego
no double count / pair dedupe / pair seen by SSv2 / pair contact as CARLA overlap in G1 / G4 /
summary); 288 Rust passed / 1 skipped, 160 pytest, `just check` clean. **Live** (simulation
side traced, one ego stack via the relay, Town01, `run_suite --physical-verdict`, bridge log
`play_log/bridge/2026-10-14_01-37-19`): bench x3 on the final build PASS 2/2 each, 1202
frames; each has exactly one contact, `{"subject": "npc_05", "other_id": null, "other_type":
"static.pole", "other_kind": "static", "frame": 25, "impulse": 19167}` -> oracle G2 **low**
`[npc_05, static]` frame 25, signature `C2:npc_static:carla`; G1 0, G3 2 (step 4's `npc_07` /
`npc_09` corners) info 20, G4 agree; `npc_05` still hands over to kinematic at 27. A first
bench on the build before the actor-0 fix saw the same hit as `unknown:0` (hence the fix).
Mid-run, CARLA held 20 `sensor.other.collision` on the 20 NPCs. `town01_rear_contact` PASS 2/2,
418 frames, `carla_contact` pass; 46 contact lines = 23 ego-side (`npc_rear`) + 23 NPC-side
(`npc_rear` -> ego); oracle G1 0 G2 0 G3 0 info 2 -- the NPC side not counted again; contact
report `contacts` one row (`npc_rear`, 23 events, as before), `npc_contacts` one row
(`npc_rear` -> `ego`, 23). `town01_pedestrian` PASS 2/2, 2287 frames, oracle all 0; mid-run
one collision sensor (the ego's): walkers get none. After every scenario CARLA held 0
collision sensors and 0 vehicles (read-only client, `get_actors(ids)`). **Cost** (bench,
traced, whole run, ms): sensors off processing p95 0.92 / tick p95 6.91; on 1.01 / 7.20, 0.98
/ 6.94, 0.96 / 6.84, 1.08 / 6.96 -- +0.04-0.16 ms, inside step 4's 0.89-1.10 and well within
its +0.5 ms; CARLA's tick unchanged within noise. Spawn frame max 370-405 ms with sensors
(296 without; step 4 312-641). Egoless scenarios (the bench) write no contact report, so
their NPC contacts are in the trace only.

**2026-10-13, step 4 -- B done: bench G3 73 -> 2 per run (both SSv2 corners beyond
tyre grip), `npc_route`'s corner G3 gone, no new fallback.** **Diagnosis**, from step 2's bench
traces replaying the command law on SSv2's recorded targets (`s032_4/diag.py`, `summ.py`,
`replay.py`): of the four candidates, (d) **the force bound** decides the cross-track error --
in 71 of the 73 windows the bound binds on 55-100 % of the frames, the corners need 5.4-18.8
m/s2 of centripetal change (`npc_09` 76-84: R 7.0 m, 9.4 m/s2) against the 10 m/s2 bound
(declared 5/8 + 2), the body follows the bounded command to 0.03 m/s, and so drifts outside
by 0.036 m a frame to the 0.5 m snap; a closed-loop replay of the position law (body = the
bounded command, as measured) reproduces it (corner frames p95 0.325 m, max 0.56, 21 snaps)
and with the turn fed forward drops it to p95 0.020 / max 0.112, 0 snaps. (b) **the steer**
decides the yaw: a set angular velocity is undone by the tyres within the tick -- `npc_11`
306-367 on a straight held 2.37° for 60 frames, commanded -4.7 deg/s and turning 0.0 (so 024's
"decays at the yaw gain" was not a decay at all), `npc_14` 970-994 commanded +5 and turned -1
-- and the steer fed forward only SSv2's rate, at which the body turns at 0.98 of the kinematic
rate (`npc_09`, -64.5 against -65.6 deg/s), so the yaw fell behind for good. (a) is not it:
SSv2's twist already carries the lateral component and the body holds the commanded velocity
to 0.03 m/s; (c) is 0.01 m: `s* = p* - v*dt` sits off the arc by (v dt)^2/2R. **Fix**
(departs from design B's text, updated above): `turned` / `bounded_command` (the bound
measured from the body's velocity rotated by SSv2's yaw rate x dt; speed never added, so a
held body still pushes with at most the bound), `command` steers from the commanded yaw rate,
`VEHICLE_GAINS.yaw` 2 -> 4/s (pure model at 0.95 understeer, 8 m/s on 6/7/10/20 m: max yaw
1.12/0.94/0.63/0.31° at 4/s; at 6 m 2.01° at 2/s, 1.46 at 3, 0.90 at 5, 0.57 at 8; xy 0.039
m at 6 m whatever the yaw gain). Tests first (3 new: a constant-radius corner tracked within 0.2 m / 2° at 8 m/s
on 6-20 m in the pure model -- as built 0.22 m and 4.6-4.8°; the turn not charged to the
bound, straight unchanged; a yaw error steered out), 282 Rust passed / 1 skipped, 151 pytest,
`just check` clean. **Live** (simulation side traced, one ego stack via the relay, Town01,
`run_suite --physical-verdict`, bridge log `play_log/bridge/2026-10-13_23-31-58`): bench ×2
PASS 2/2 each, 1202 frames, oracle G1 0 G2 0 **G3 2** (73 at step 2) info 20, G4 agree, both
runs identical; the two are `npc_07` 254-256 (0.233 m, 1.77°) and `npc_09` 961-965 (0.253 m,
1.89°), each at SSv2's tightest point -- R 3.9-4.1 m at 8.5 m/s, 17.6 / 18.8 m/s2, 1.8-1.9 g --
where the body's velocity turns ~60 % of the commanded rotation per tick (the tyres' grip,
not the bound: the command is unbounded there) and runs 0.15-0.25 m wide; a real car would
too. Bench-wide, every tracked NPC frame from 10 on: xy p95/p99 0.221/0.428 -> **0.059/0.103
m**, yaw p95/p99 1.55/2.24 -> **0.66/1.11°**, frames over 0.2 m 1262 -> 12, over 2° 392 -> 0.
The 20 info are `spawn` at frame 2 (0.29 m, one frame each): step 3's D -- the first tracked
frame now starts the body at speed while SSv2's first pose is still the spawn pose -- not B.
`npc_05` hands over to kinematic at frame 27 in both runs as in step 2; no other fallback.
Bridge processing p95 whole run 1.10 / 0.89 ms (step 2: 0.88 / 0.83); run 1's 1.10 is two
200-frame windows at 1.60/1.48 with CARLA's tick p95 at 12-13 ms (host load), its other
windows 0.86-0.97. `town01_simulator_autopilot` PASS, 846 frames, G3 1 info 1 (`read_back`):
`npc_route`'s corners (024: 450-462 0.28 m, 706-730 0.38 m / 2.1°) are gone -- its worst
frame after the start 0.086 m, 0.90° -- and the one G3 is 024's row-9 speed step (0 -> 6 m/s
in a frame, 230-232, 0.655 m, the same number to the micron, 116 frames later than 024's run).
`town01_rear_contact` PASS, 282 frames, 24 contacts, G1 **1** (ego-`npc_rear` frame 222, the
contact onset, CARLA geometry, penetration 0.07 m, margin difference 0.1545 against band
0.1533), G2 0 G3 0, info 3 (1 `lag_band`, 2 `contact_hold` 224-250 1.44 m / 255-265 0.34 m);
`npc_rear` 0 snaps, 0 fallback, yaw p95 0.45°. The same 1-frame onset edge steps 2 and 3 saw,
1 mm over the band, on a straight where B changes nothing but the steered yaw; for step 7.

**2026-10-13, step 3 -- C and D done: the walker yaw G3 (174° / 108°) and `Npc1`'s spawn
G3 are gone; pedestrian G3 2 -> 0, UC-ACC G3 1 -> 0.** `npc_tracking`: `walker_control`
(below `WALKER_SPEED_FLOOR` speed 0, direction SSv2's heading) and `first_frame_change`
(a body's first tracked frame is not force-bounded when SSv2 already moves it), tests first
(3 new, 279 Rust passed / 1 skipped, 151 pytest, `just check` clean). `WALKER_SPEED_FLOOR`
0.05 m/s from 024's step-7 pedestrian trace: standing, SSv2's poses step by its position
precision (0.0038 / 0.0075 m/s alternating, frames 135-156), a command of at most 0.007 m/s
over 203 standing frames; the floor is 7x that, and leaves at most 0.05/6 = 0.008 m
uncorrected. **D departs from the design text**: SSv2 sends no speed at spawn
(`traffic_simulator` `API::spawn` sets `initial_speed` 0.0, "@todo"; a pedestrian request has
no field), and its first status of an added entity is the spawn pose with the twist already
at speed (UC-ACC `Npc1`: pose = spawn pose at 5.5 m/s), so nothing is known to seed before
the settling tick. The seed is the first tracked frame instead: its command (SSv2's twist +
the position term) is applied unbounded, and only when the twist is passed -- an NPC still
held before `npc_logic_started` starts at rest under the force bound as before. Walkers need
nothing for D: `WalkerControl` sets speed directly. **Live** (simulation side traced, one ego
stack via the relay, Town01, `run_suite --physical-verdict`, bridge log
`play_log/bridge/2026-10-13_23-09-18`): `town01_pedestrian` PASS 2/2, 2383 frames, oracle G1 0
G2 0 **G3 0** info 0, G4 agree; CARLA walker yaw 90.0002° on every frame (024: spins to 174° /
108°), max |yaw| difference 0.00°, xy max 0.094 m, 0 snaps. UC-ACC variant (`vxi_ego` 8.333,
`vxi_npc0` 5.556, `vx_target_npc0` 1.944) PASS 2/2, 1322 frames, G1 0 G2 0 **G3 0**, info 1
(`spawn`, `Npc1` frame 507 only, xy 0.208 m), G4 agree; `Npc1` added at frame 507 leaves the
first tick at 4.17 m/s (024: 0.5), error 0.208 / 0.155 / 0.112 / 0.082 m over 507-510 (024:
0.03, 0.20, 0.40, 0.57, then a snap), report 0 snaps, max 0.278 m (024: 1 snap, 0.572 m). The
0.21 m first frame is SSv2's: its first pose is the spawn pose while its twist says 5.5 m/s,
so the start-of-step pose sits 0.275 m behind the body. `town01_rear_contact` ×2 PASS 2/2:
run 1 259 frames 22 contacts, G1 **1** (ego-`npc_rear` frame 199 only, CARLA geometry,
penetration 0.03 m, margin difference 0.196 vs band 0.147, at the contact onset) G2 0 G3 0
info 3; run 2 261 frames 23 contacts, G1 0 G2 0 G3 0 info 4 (its onset frame 202, 0.128 vs
band 0.169, is a `lag_band`). Not from C or D -- `npc_rear` is held at spawn (0 m/s on its
first frames), so neither path touches it -- the same 1-frame onset edge step 2 saw once; a
lag-band tolerance question for step 7, not a regression. `npc_rear` 0 snaps, 0 fallback.

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
