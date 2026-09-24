# Phase 014: Feature Completeness

Close the gaps the 2026-09-25 completeness audit found between what SSv2 can ask of a backend
and what this bridge plus acb deliver, and take the cheap lessons from TIER IV's
`carla-autoware-native` fork.

**Source**: [design/ssv2-feature-completeness.md](../design/ssv2-feature-completeness.md),
ranked gap table.
**Depends on**: nothing new. A live single-ego stack is enough for every measurement below.
**Relates to**: [009](009-map-and-traffic-lights.md) owns the traffic-light publisher
(its 2026-08-28 decision); this phase does not duplicate it. [008](008-entity-fidelity.md)
owns blueprint choice; the bbox work here feeds it.

## Problem

Three things the scenario verdict rests on are computed twice, by SSv2 and by CARLA, with
nothing comparing them:

- **Where an entity is.** Scenario vehicles declare `BoundingBox/Center x="1.5"`: SSv2's
  entity origin is the rear axle. CARLA's actor origin is near the bbox centre. The bridge
  reads `bounding_box` nowhere and applies no offset in either direction, so every NPC mesh
  should sit ~1.5 m behind its SSv2 pose and the ego pose reported back should be ~1.5 m off
  the other way. Every distance condition — cut-in gap, TimeHeadway, following distance — is
  suspect until this is measured.
- **Whether two entities collided.** SSv2 runs its own 2D bbox check
  (`api.cpp:345`). CARLA resolves the PhysX ego against kinematic NPC colliders, which stay
  active under `set_simulate_physics(false)`. A teleported NPC can shove the ego and SSv2 sees
  nothing; SSv2 can flag a collision CARLA never rendered.
- **How fast an NPC is moving.** NPCs are teleported with zero CARLA velocity. Anything
  CARLA derives from actor velocity — radar Doppler, wheel spin, walker animation, and acb's
  ground-truth object velocity if it reads CARLA — is wrong.

Underneath those sit a handful of small, verified defects: `update_step_time` sets
`fixed_delta_seconds = step_time` while sync mode uses `step_time / substeps`
(`coordinator.rs:1504` vs `:1417`, latent because SSv2 sends it once before sync mode);
`max_substep_delta_time` is never set, which TIER IV measured as up to 0.9 °/s phantom yaw
rate on 0.9.16; acb follows ticks with `wait_for_tick_or_timeout`, which drops frames that
complete while it is busy — a candidate cause of the ego LiDAR falling to 10 Hz with two
stacks.

## Decisions (2026-09-25)

- **Ego origin is the rear axle**, Autoware's `base_link` convention. Today acb publishes
  `base_link` at the CARLA actor origin (`autoware.rs:608-613`, no offset), so the offset is
  not only bridge↔SSv2 but acb↔Autoware: planner stop distances, pure-pursuit lookahead and
  sensor TFs all assume rear axle. acb derives the offset from CARLA's rear wheel positions
  (`WheelPhysicsControl::position`, 0.9.16); the bridge applies SSv2's `bounding_box.center`
  for ego readback and NPC teleports. Both must agree; the measurement below checks that.
- **CARLA collisions are diagnostic only.** The collision sensor on the ego logs every event
  and the run ends with a mismatch summary against SSv2's CollisionCondition. SSv2 stays
  the verdict; there is no protocol slot to hand it a CARLA collision anyway. NPC colliders
  stay on: disabling them blinds the ray-cast LiDAR.
- **Detection-sensor knobs are unsupported**, and documented as such. Perception-fault
  scenarios (`isClairvoyant`, noise, delay) are out of scope; CARLA's value here is real
  Autoware perception.
- **Order**: pose measurement, then 009's traffic-light publisher, then the hardening batch.

## Work Items

Measurement first for the three big ones. No fix lands before its number is known.

### Pose reference point (gap 2)

- [ ] Measure: spawn one NPC at a known lanelet pose with `Center x="1.5"`, read the CARLA
      actor transform and `Vehicle::bounding_box`, report the along-axis delta
- [ ] Measure the ego side: compare the pose the bridge returns in `UpdateEntityStatus` with
      the rear-axle pose SSv2 expects, on a straight and in a turn
- [ ] Measure acb's side: `base_link` in `/carla/ground_truth/odom` against the CARLA rear
      wheel positions; expected delta ≈ half the wheelbase
- [ ] acb publishes `base_link` at the rear axle (pose, `/tf`, ground truth) and shifts
      sensor TFs accordingly; offset read from CARLA wheel physics, not hard-coded
- [ ] Apply `bounding_box.center` in `ros_pose_to_carla_transform` and its inverse; walkers
      and misc objects get their own offsets (walker capsule origin ~0.9 m)
- [ ] Re-run `town01_two_av.xosc` and `town01_pedestrian.xosc`; the follow distance and stop
      distance in the logs move by the measured delta and nothing else changes

### Collision truth (gap 3)

- [ ] Attach `sensor.other.collision` to the ego; log every event with the other actor's
      SSv2 name and the SSv2 frame
- [ ] Run a scenario with a deliberate NPC cut-in that clips the ego; record whether SSv2's
      CollisionCondition and the CARLA sensor agree
- [ ] End-of-run summary: CARLA collision events vs SSv2 CollisionCondition outcomes, with
      the SSv2 frame of each; mismatches logged at WARN. Policy is diagnostic only (above)

### NPC motion (gap 5)

- [ ] `set_target_velocity` from `action_status.twist` on every teleported vehicle
- [ ] `WalkerControl` speed from the same on every walker, so the walk cycle plays
- [ ] Check acb's ground-truth object velocity source; if it reads CARLA, it now agrees with
      SSv2

### Hardening batch (gaps 4, 6, 10, 12, 13, 16)

- [ ] `update_step_time` uses `substep_delta()` in the sync-mode branch
- [ ] `max_substep_delta_time = 0.002` and `max_substeps = ceil(substep_delta / 0.002)` in
      the world settings; before/after IMU yaw-rate at standstill recorded
- [ ] Pedestrian z after the first `UpdateEntityStatus`: verified, offset applied if wrong
- [ ] `/control/control_mode_request` service in acb (AUTONOMOUS→ok, MANUAL→fail, matching
      stock `autoware_universe.cpp:52`)
- [ ] Second `is_ego` rejected; unknown entity in `UpdateEntityStatus` rejected, not echoed;
      `overwrite_ego_status` also honored when ControlModeReport is MANUAL
- [ ] `set_{green,yellow,red}_time(99999)` on every mapped light as backup to
      `freeze_all_traffic_lights`

### acb tick following (gap 7)

- [ ] Measure: ego LiDAR `ros2 topic hz` with one stack and with two, before any change
- [ ] Replace `wait_for_tick_or_timeout` with an `on_tick` callback feeding a queue; drain to
      newest, warn on skipped frames
- [ ] Re-measure; the two-stack number is the acceptance

### Steering (gap 8)

- [ ] Measure: commanded tire angle vs `get_wheel_steer_angle` across the range at 0, 20,
      40 km/h
- [ ] Flatten `VehiclePhysicsControl.steering_curve` to 1.0 at spawn if the measurement
      shows speed dependence
- [ ] Steer rate limit and first-order lag in acb config, default off, values from SSv2
      #1849 as the documented starting point (20 °/s, τ=0.2 s)

### Deferred, with reasons

- **Traffic-light state to Autoware (gap 1)**: 009's decision, tracked there.
- **Blueprint by subtype / bbox (gap 9)**: 008's territory; the offset work above is its
  prerequisite.
- **Detection-sensor knobs (gap 11)**: unsupported by decision (above). Add the statement
  to `docs/design/scenario-authoring.md` so scenario authors do not expect them to work.
- **Arrows, flashing (gap 14)**: CARLA 0.9.16 has no arrow bulbs; only representable on the
  V2X path from gap 1.
- **Weather / TimeOfDay (gap 15)**: no protocol slot and SSv2's own interpreter no-ops it.
  Belongs in the [005](005-hardening-awf-contribution.md) upstreaming discussion.

## Acceptance

- The pose delta is measured, applied, and the follow distance in `town01_two_av.xosc`
  changes by that amount and nothing else.
- A cut-in that clips the ego is seen by both SSv2 and the CARLA collision sensor, or the
  chosen policy makes one of them authoritative and the doc says which.
- Ego LiDAR rate with two stacks is measured before and after the tick-follower change.
- Every hardening item has a before/after number or a test.

## Not in scope

Anything requiring an engine patch. Anything that publishes on a topic SSv2 already owns.
