# Phase 023: NPC Motion Parity

Third fix of the [gap campaign](../design/gap-campaign.md): an NPC moves through CARLA as a
body with a velocity, not as a sequence of teleports at 0 m/s, while staying on the poses
SSv2 commands within a stated budget. Gated by a spike: if the budget cannot be held, the
phase ends with that measurement and NPCs stay kinematic.

**Source**: user direction 2026-10-11 ("fix scenario spec and Carla gap first").
**Depends on**: [014](014-feature-completeness.md) gap 5 (the measurement that opened this),
[022](022-entity-geometry-parity.md) (the right body before its motion).
**Feeds**: [024](024-gap-trace-and-oracle.md) (G3 pose divergence measures this phase's
residual).

## Problem

- **CARLA says every NPC is standing still.** NPCs are physics-off actors moved by
  `set_transform` each frame (`coordinator.rs:3279`). CARLA ignores `set_target_velocity`
  and `WalkerControl` on a physics-off actor: `get_velocity()` reads 0.000 m/s (measured
  2026-09-27, 014 gap 5).
- **What reads that zero.** acb's ground-truth objects already estimate each NPC's twist
  from its pose history (acb `028d8a8`), so Autoware's object list is not the problem any
  more. What remains:
  - **contact**: a kinematic body has infinite mass and no velocity, so the impulse it gives
    the ego is not the impulse of a moving car; with a 021 failure now carrying the
    impulse, it should mean something;
  - **walkers**: no walk cycle without physics, so a camera sees a gliding statue (009's
    classification work needs the animation);
  - **sensors that measure velocity** (radar Doppler) report 0.
- **Height.** Spawn snaps to the ground (`resolve_spawn_height`), walkers are lifted per
  teleport (`walker_teleport_z`), vehicles are not: on a grade, a teleported vehicle's z is
  SSv2's, which can float or sink. A body under physics follows the road by itself.

## Design

Spike first; the design below is what the spike tests.

**Tracked physics.** NPC vehicles and walkers spawn with physics on. Each frame, from SSv2's
commanded pose `p*` (after `OriginOffset`) and twist `v*`:

- **aim at the start of the step**: the body must be at `p*` *after* the tick, and a
  velocity set before the tick acts during it, so the law's reference is
  `s* = p* - v*·dt` (yaw likewise, `yaw* - ω*·dt`). Aiming at `p*` itself, as first written,
  leaves the body exactly one frame ahead (0.40 m at 8 m/s, measured, step 2);
- vehicles: `set_target_velocity(v* + k_p (s* - p))` in the plane, angular velocity
  `ω* + k_yaw (yaw_s* - yaw)`; z left to physics; plus **steer feed-forward**
  `steer = atan(wheelbase · ω*/|v*|) / max_steer_angle` through `VehicleControl`, or the
  unsteered front tyres fight the imposed yaw rate in corners;
- walkers: `WalkerControl` with direction and speed from the same law;
- **snap rule**: if the in-plane error exceeds 0.5 m or the yaw error 5°, teleport to `s*`
  with velocity `v*` (spawn, SSv2 teleport actions, a body knocked off course);
- gains from step 2: `k_p` 5 /s, `k_yaw` 2 /s for vehicles; `k_p` 5-8 /s for walkers;
- the ego's own contacts with a tracked NPC are physical both ways: the NPC can be pushed,
  which the snap rule bounds.

**Budget.** Tracking error after `OriginOffset`, in-plane: p95 ≤ 0.2 m, max ≤ 0.5 m (the
snap); yaw p95 ≤ 2° -- the same tolerances 024's G3 uses, so this phase's success is the
oracle's silence. Measured per frame from one world snapshot, never per-actor RPCs.

**Switch.** Config key `npc_motion: kinematic | tracked`, default stays `kinematic` until
the live step passes; then the default flips and `kinematic` remains for comparison and
fallback.

**Wiring (step 4).**

- *Who is tracked*: SSv2-driven NPC vehicles and pedestrians under `npc_motion: tracked`.
  Misc objects (static props), simulator-autopilot and agent vehicles and the ego are
  unchanged. A tracked actor spawns with physics on, is not parked (no hand brake), and
  does not get the physics actors' settle-frame acceleration masking -- SSv2 reports its
  own NPC state, nothing is read back from CARLA.
- *Target*: SSv2's pose through the same `ros_pose_to_carla_transform` (origin offset, map
  origin) as the teleport, and SSv2's twist (`action_status.twist`, body frame, at the
  entity origin) turned into CARLA's world frame: rotate by the entity yaw, flip y, and
  negate the yaw rate. The body's origin is not the entity's, so its velocity gets
  `ω × r` added, `r` the origin offset in the world frame: 1 m/s at 0.7 rad/s and the
  1.5 m rear-axle offset, not negligible in a turn.
- *Steering geometry*: the wheelbase from SSv2's declared axles (front `positionX` - rear
  `positionX`) -- version-free, unlike CARLA's wheel `position`/`offset`, and the geometry
  SSv2's own model turns with -- and `max_steer_angle` of CARLA's front wheel. No axles
  declared: no steer feed-forward.
- *Per frame*: one world snapshot and one actor list for all tracked NPCs; `command()`
  from `npc_tracking`; `Track` -> `set_target_velocity` (vehicle: keep CARLA's own z
  velocity), `set_target_angular_velocity`, `apply_control(steer)`; walkers ->
  `WalkerControl(direction, speed)`; `Snap` -> the existing teleport path (its failure
  handling unchanged; walkers keep their ground height) plus the velocity.
- *What SSv2 is told*: the pose and twist it sent, as for a kinematic NPC (SSv2 owns NPC
  state); the tracking error is CARLA's business, measured and reported.
- *Statistics*: per tracked entity and episode -- frames, in-plane error p95/max, yaw error
  p95/max (vehicles), snaps -- measured before each step against the start-of-step pose;
  logged at the end of the ego's run and written to 021's contact report as `tracking`
  (optional, schema 1, like 022's `geometry`).

## Steps

1. [x] **Docs.** This phase doc.
2. [x] **Spike (gate).** A scratch Python client on Town01: one vehicle NPC tracking a
   recorded SSv2 trajectory (straight, curve, stop-and-go) and one walker crossing, under
   synchronous mode at the bridge's fixed delta. Report error p95/max, snap count,
   `get_velocity()` against `v*`, frame-time cost. **If the budget is missed with
   reasonable gains, stop here**: record the numbers in this doc and 014 gap 5, keep NPCs
   kinematic, close the phase.
3. [x] **Control law, test-driven.** Pure function `(p*, v*, p, yaw*, yaw) → command | snap`
   with tests: on track, lagging, overshoot, snap thresholds, stopped target.
4. [x] **Bridge wiring.** Physics on at spawn under `tracked`; the per-frame command from the
   snapshot; snap via the existing teleport path (its failure handling unchanged); walker
   control. Error statistics per episode in the log and in 021's report.
5. [ ] **Live.** The nine starter scenarios under `tracked`: all pass, error within budget,
   `frame_stats` p95 within noise of `kinematic` (20-NPC bench, 016).
   `town01_pedestrian`: the walker animates in the screencast view. `town01_rear_contact`:
   `max_impulse` now scales with closing speed (run at two NPC speeds).
6. [ ] **Default flip** to `tracked`; user guide and bridge config comment.

### Results

**Step 4 (2026-10-12).** `npc_motion: kinematic | tracked` in `bridge_config.yaml`, and as
the `npc_motion:=` argument of `simulation.launch.xml` / `bridge.launch.xml` (default
`kinematic`). Under `tracked`, SSv2-driven NPC vehicles and pedestrians spawn with physics on
and `UpdateEntityStatus` drives each through `track_npc`: one world snapshot and one
`actors_by_ids` list per frame, SSv2's twist through `world_twist` (with `ω × r`), the
wheelbase from the declared axles and `max_steer_angle` from CARLA, `set_target_velocity` /
`set_target_angular_velocity` / steer for vehicles, `WalkerControl` for walkers, snaps
through `set_transform` (walkers keep their ground height). Per-episode statistics are logged
at the end of the ego's run (`Tracked '<name>': N frames, error p95 / max, yaw p95 / max,
snaps`) and written to the contact report as `tracking`. 245 unit tests (new: twist
conversion and `ω × r`, steering geometry, statistics, the report rows, the monitor's
per-episode reset, the config key and its parameter, which kinds are trackable); clippy
clean.

Smoke run, `npc_motion:=tracked`, `ros2 launch`, three starter scenarios with
`--physical-verdict`:

| Scenario | Tracked entity | Frames | Error p95 / max | Yaw p95 / max | Snaps | Verdict |
|---|---|---|---|---|---|---|
| `town01_pedestrian` | `pedestrian_0` (walker) | 3374 | 0.000 / 0.092 m | -- | 0 | PASS 2/2 |
| `town01_rear_contact` | `npc_rear` | 287 | 0.022 / 0.156 m | 0.00 / 0.36 deg | 0 | **FAIL**: SSv2 `Entity: ego - current acceleration (which is 34.5074) is out of range (which is [-15, 15])` |
| `town01_simulator_autopilot` | `npc_route` | 828 | 0.525 / 0.525 m | 3.61 / 7.27 deg | **133** | PASS 2/2 |

The wiring holds; two findings for step 5:

- **A tracked NPC that rams the ego shoves it far harder.** `npc_rear` tracks its pose
  closely right up to the contact, but it is a full-mass body still commanded at 8 m/s into
  the ego: 34.5 m/s2 on the ego, past SSv2's sanity bound, so the scenario fails where the
  kinematic NPC passed (once at 15.6, the same bound, in an earlier kinematic run). The
  velocity command does not stop at a contact; the snap rule only bounds position.
- **`npc_route` snaps often**: 133 of 828 frames, yaw error to 7.3 deg, the in-plane error
  pinned at the snap distance -- the spike's corner finding (a turn SSv2 commands faster
  than the tyres hold), now on the starter route. Its p95 is over the 0.2 m budget.

**Step 3 (2026-10-12).** `npc_tracking.rs`: `command(target, current, dt, gains, steering)
-> Track | Snap`, in CARLA's world frame, with the start-of-step reference, steer
feed-forward, the 0.5 m / 5° snap (no yaw snap for walkers), and the spike's gains as
`VEHICLE_GAINS` (5, 2) and `WALKER_GAINS` (6). Ten tests: on track, aiming at `p*` would
be a frame ahead, lagging, overshoot, yaw error across the ±180° wrap, both snap thresholds
and their snap pose, walker never yaw-snaps, stopped target (no division by zero), steer
sign/magnitude/saturation, wrap range. Not wired yet (step 4).

**Step 2, spike (2026-10-12): the gate passes.** Scratch client (`scratchpad/spike023/
track.py`, not kept) on the shared CARLA 0.9.16, Town01, synchronous at the bridge's 0.05 s,
world settings untouched. Trajectories were synthesized from Town01 waypoints with SSv2-like
speed profiles (not recorded from SSv2): **cruise** to 8 m/s, **turn** at 5 m/s, **stop-and-go**
(0→10 m/s at 2 m/s², hold, 10→0 at 3 m/s², hold 2 s, twice), and a **walker** crossing the
lane at 1.4 m/s. All three vehicle routes pass a ~7 m-radius corner at s ≈ 46-60 m (spawn
point 0 sits just before it). Error is measured after each tick against the pose commanded
for that frame, from one world snapshot; "settled" leaves out the first second (spawn drop).

| Case (Model 3 unless noted) | Kinematic `get_velocity()` | Tracked, k_p 5, k_yaw 2, steer FF: settled in-plane p95 / max | yaw p95 / max | snaps | speed error p95 |
|---|---|---|---|---|---|
| cruise | 0 m/s (100 % off) | 0.022 / 0.037 m | 0.85° / 3.7° | 1 (the corner, 8 m/s) | 0.2 % |
| turn | 0 | 0.038 / 0.058 m | 1.02° / 4.1° | 2 (the corner, 5 m/s) | 0.9 % |
| stop-and-go | 0 | 0.011 / 0.035 m | 0.49° / 4.6° | 3 (the corner, ≤10 m/s) | 0.2 % |
| walker, k_p 5 / 8 | 0 | 0.004 / 0.020, 0.000 / 0.002 m | -- | 0 | 1.7 %, 0.1 % |

- **Budget met** (p95 ≤ 0.2 m, max ≤ 0.5 m, yaw p95 ≤ 2°), CARLA's velocity within 1 % of
  the command, against 100 % off for kinematic NPCs.
- **Two corrections to the law** were needed, both now in the design above. The first run
  aimed at `p*`: steady 0.40 m error, one frame ahead, at every gain (0.80 m against the
  previous frame's pose). Without steer feed-forward the corner cost 5-6 snaps per pass and
  yaw p95 up to 2.9°; with it, 1-3 snaps and ≤ 1.0°.
- **Snaps only in the tight corner at speed**: 7 m radius at 5-10 m/s is 3.6-14 m/s² of
  lateral acceleration, which SSv2's kinematic NPCs will command and no tyre holds. The snap
  rule bounds it; step 4 counts snaps per episode so a scenario that relies on them shows.
- **Walker**: the first second lags 0.17-0.29 m while the walker accelerates from rest
  (under the snap); settled it is exact. The walk animation was not looked at (headless);
  step 5's screencast checks it.
- **Cost**, 20 Model 3s, 100 frames: tick p50 / p95 5.2 / 7.7 ms kinematic, 7.0 / 12.2 ms
  tracked (physics on); the per-frame commands are fire-and-forget in the client (0.06 ms
  for 20 `set_transform`, 0.16 ms for 3 x 20 tracked calls), and `apply_batch` brought
  nothing. Well inside the 50 ms frame; step 5's 20-NPC bench decides.
- **Not covered**: grades (Town01 is flat; z is left to physics by design), a trajectory
  recorded from SSv2 itself, the Rust client's call cost (step 4).

## Acceptance

- Either: the spike's numbers miss the budget and are recorded, NPCs stay kinematic, phase
  closed with that finding.
- Or: under `tracked`, CARLA reports NPC velocities within 10 % of SSv2's commanded speed,
  tracking error within budget on all starter scenarios, walkers animate, no frame-time
  regression, and `tracked` is the default.
