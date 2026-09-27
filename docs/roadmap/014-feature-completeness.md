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

- [x] Measure: spawn one NPC at a known lanelet pose with `Center x="1.5"`, read the CARLA
      actor transform and `Vehicle::bounding_box`, report the along-axis delta —
      **2026-09-25, `scripts/pose_offset_probe.py`, Town01 (320, −129.8) heading 180°:**
      the bridge puts the CARLA actor origin exactly on the SSv2 pose (Δ along heading
      0.000 m at spawn and again after an `UpdateEntityStatus` teleport). On
      `vehicle.tesla.model3` the rear axle is **1.386 m behind** that origin (front axle
      +1.618 m, wheelbase 3.005 m, bbox centre +0.029 m, length 4.79 m). SSv2 models the same
      entity as a 4.5 m body from −0.75 m to +3.75 m of its origin; CARLA renders it from
      −2.37 m to +2.42 m. Net: the mesh sits 1.39 m behind SSv2's model, and for two
      same-lane vehicles the real bumper gap is ≈2.9 m larger than the gap SSv2 computes
      (1.33 m front-overhang error on the follower + 1.62 m rear-overhang error on the leader)
- [x] Measure the ego side: the `UpdateEntityStatus` reply returns the actor origin
      unchanged (readback == actor origin, Δ 0.000 m), so SSv2 takes the vehicle centre as
      the rear axle: it believes the ego is 1.386 m further along than its rear axle is.
      Turn case not measured separately; the offset is a rigid body-frame translation, so a
      straight suffices
- [x] Measure acb's side: acb reads the same actor transform for `base_link`, so its
      `base_link` is 1.386 m ahead of the rear axle on this blueprint (from the rear wheel
      positions in `get_physics_control()`; not re-measured over ROS)
- [x] acb publishes `base_link` at the rear axle (pose, `/tf`, ground truth) and shifts
      sensor TFs accordingly; offset read from CARLA wheel physics, not hard-coded —
      done 2026-09-25 (acb, uncommitted pin until pushed). acb logs
      `base_link (rear-axle centre) at (-1.386, 0.000, 0.000) m`; sensors declared 0.900 m
      ahead of base_link attach at −0.486 m from the actor origin. Stationary EKF error
      against the rear axle 0.046–0.085 m (1.3 m against the origin before); moving 0.32–0.41 m
      ahead of truth, which is estimator latency at ~4 m/s, not a reference-point error
- [x] Apply `bounding_box.center` in `ros_pose_to_carla_transform` and its inverse —
      done 2026-09-25: `OriginOffset` stored per entity at spawn, applied on spawn, teleport
      and ego-pose overwrite, undone in the ego readback. Probe after: NPC and ego actor
      origins +1.500 m along heading from the SSv2 pose, ego readback − commanded = 0.000 m.
      x/y only; the walker z question stays in the hardening batch below
- [ ] Re-run `town01_two_av.xosc` and `town01_pedestrian.xosc`; the follow distance and stop
      distance in the logs move by the measured delta and nothing else changes —
      **partly done 2026-09-25 on newslab-server139** (two_av needs a second Autoware stack,
      not yet run here). `town01_ego_drive.xosc` passes before and after (34.7 → 31.7 s sim).
      `town01_pedestrian.xosc`: the walker stop moved from 5.84 m to **4.66 m** front-bumper
      gap (Δ 1.18 m, against 1.386 expected; the hold is 58 s both times), so the geometry
      moved as predicted. But the scenario **fails by timeout in both runs**: after the hold
      the ego pulls away while the walker is still ~1.7 m from its centreline, stops again
      level with the walker (bumper 0.97 m / 2.58 m past it) and never moves for the
      remaining ~200 s. 008 recorded this scenario passing on the previous host. Not a
      reference-point problem; tracked as its own item below

### Pedestrian scenario stalls after the hold (found 2026-09-25)

`town01_pedestrian.xosc` times out on newslab-server139 with and without the acb fix: the
ego holds ~58 s for the crossing walker, resumes while the walker is still ~1.7 m from its
centreline, stops again beside it and stays stopped ~200 s. Ground-truth CSVs:
scratchpad `baseline/pedestrian_gt.csv` and `postfix/pedestrian_gt.csv` (this session).
Perception counted 0–6 objects during the run. Host load was ~32 on 32 cores.

- [ ] Reproduce on a quiet host; if it passes there, this is capacity (roadmap README
      priority 2), not planning
- [ ] If it reproduces: read the crosswalk / obstacle-stop module decisions around the
      resume (planning debug topics), and whether the walker is still a tracked object when
      the ego stops the second time

### Collision truth (gap 3)

- [x] Attach `sensor.other.collision` to the ego; log every event with the other actor's
      SSv2 name and the SSv2 frame — `078ea9f`; live 2026-09-27 (`scratchpad/measure/collision_probe.py`): sensor attached at
      ego spawn, one WARN at first contact (SSv2 frame 11), one INFO summary at the ego
      despawn, no `sensor.other.collision` left in the world. CARLA prints
      `attempting to unsubscribe from stream but sensor wasn't listening` on the stop before
      destroy; harmless, not chased
- [ ] Run a scenario with a deliberate NPC cut-in that clips the ego; record whether SSv2's
      CollisionCondition and the CARLA sensor agree
- [x] End-of-run summary: CARLA collision events vs SSv2 CollisionCondition outcomes, with
      the SSv2 frame of each; mismatches logged at WARN. Policy is diagnostic only (above)

### NPC motion (gap 5)

- [ ] ~~`set_target_velocity` from `action_status.twist` on every teleported vehicle~~ —
      **measured 2026-09-27: CARLA ignores it on a physics-off actor** (`get_velocity()` stays
      0.000; the same control with physics on reads 1.504 m/s). Per-frame velocity needs the
      actor's physics on, which conflicts with teleport-driven placement; needs a design, not
      a call. NPCs stay kinematic
- [ ] ~~`WalkerControl` speed from the same on every walker, so the walk cycle plays~~ —
      same finding: accepted by CARLA, no motion while physics is off
- [ ] Check acb's ground-truth object velocity source; if it reads CARLA, it now agrees with
      SSv2

### Hardening batch (gaps 4, 6, 10, 12, 13, 16)

- [x] `update_step_time` uses `substep_delta()` in the sync-mode branch — `ddf64d7`;
      probe after UpdateStepTime(0.05): fixed_delta 0.025 (was 0.05)
- [x] `max_substep_delta_time = 0.002` and `max_substeps = ceil(substep_delta / 0.002)` in
      the world settings; before/after IMU yaw-rate at standstill recorded
- [x] Pedestrian z after the first `UpdateEntityStatus`: verified wrong (origin at 0.000
      after teleport, 0.951 at spawn: half the walker under the road); `walker_lift` stored at
      spawn and added per teleport, now 0.930 — `ddf64d7`
- [x] `/control/control_mode_request` service in acb (AUTONOMOUS→ok, MANUAL→fail, matching
      stock `autoware_universe.cpp:52`)
- [x] Second `is_ego` rejected; unknown entity in `UpdateEntityStatus` rejected, not echoed;
      (both verified by probe, `ddf64d7`); the MANUAL-overwrite case needs a ControlModeReport
      subscription the bridge has no ROS node for — dropped;
      `overwrite_ego_status` also honored when ControlModeReport is MANUAL
- [x] `set_{green,yellow,red}_time(99999)` on every light (mapped or not) as backup to
      `freeze_all_traffic_lights`

### acb tick following (gap 7)

- [ ] Measure: ego LiDAR `ros2 topic hz` with one stack and with two, before any change
- [x] Replace `wait_for_tick_or_timeout` with an `on_tick` callback feeding a queue; drain to
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
