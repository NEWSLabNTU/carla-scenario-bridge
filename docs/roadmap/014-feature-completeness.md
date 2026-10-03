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

### Pedestrian scenario stalls after the hold (found 2026-09-25, root-caused 2026-09-27)

`town01_pedestrian.xosc` timed out three times on newslab-server139. The second stop was
never about the walker: `/api/planning/velocity_factors` said `route-obstacle`, and
`obstacle_stop` named an object at (231.8, −129.8) — **`bg_av_1`**, the background AV the
bridge spawns from `bridge_config.yaml` at (230, −129.8) in the ego's lane at every
Initialize. With no domain-2 stack to drive it, it is a parked car 8 m ahead. The earlier
host always had the background stack up, so it drove away before the ego arrived.

- [x] Reproduce on a quiet host — reproduced at load ~15; not capacity
- [x] Read the planner's decision — `route-obstacle`, object = `bg_av_1`
- [x] Bridge: `CSB_BACKGROUND_AVS=all|none|<list>` override and a WARN per spawned
      background AV naming the lane it blocks — `fc8e10f`
- [x] Re-run `town01_pedestrian.xosc` with `CSB_BACKGROUND_AVS=none` — **PASS**
      2026-09-27 (`failures="0"`, 310 s wall, collision summary: no collisions)
- [x] Default flipped: background AVs are **opt-in**. Unset (or blank) `CSB_BACKGROUND_AVS`
      now means `none`, so a plain `just run` no longer parks `bg_av_1` in the ego's lane on
      a host without the domain-2 stack. `just two-av` starts its bridge with `all` and
      refuses a running bridge that does not spawn `bg_av_1`; `just bg-av` warns in that case.
      One startup INFO line names the enabled set and how to enable the rest; each
      `Initialize` logs declared-but-disabled AVs. Unit tests in `config.rs`

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
- [x] acb's ground-truth objects read `actor.velocity()`, so every teleported NPC was published
      as stationary. acb `028d8a8` estimates each actor's twist from its snapshot pose on
      simulation time (arc-corrected; reset on first sight, id reuse, despawn, gaps > 1 s and
      implausible jumps) and keeps CARLA's own twist whenever it is non-zero. Live
      2026-09-28 (`town01_pedestrian.xosc` passing, with a passive second acb in domain 7
      publishing ground-truth objects; `scratchpad/npcvel`): the walker reads 1.39 m/s
      while moving (was 0); over 1 s windows published 0.695 m/s against displacement 0.692
      (n = 249); published direction equals travel direction to 0.0°. The walker faces
      along the lane (yaw ≈ 0°) while crossing it (heading 90°), so its velocity is lateral
      in its own frame -- that is the scenario's orientation, not the estimator

### Hardening batch (gaps 4, 6, 10, 12, 13, 16)

- [x] `update_step_time` uses `substep_delta()` in the sync-mode branch — `ddf64d7`;
      probe after UpdateStepTime(0.05): fixed_delta 0.025 (was 0.05)
- [x] `max_substep_delta_time = 0.002` and `max_substeps = ceil(substep_delta / 0.002)` in
      the world settings; before/after IMU yaw-rate at standstill recorded
- [x] Pedestrian z after the first `UpdateEntityStatus`: verified wrong (origin at 0.000
      after teleport, 0.951 at spawn: half the walker under the road); `walker_lift` stored at
      spawn and added per teleport, now 0.930 — `ddf64d7`. **Regression found 2026-09-27**: the
      lift is added to SSv2's commanded z, and the scenario sends z=0.3, so the walker's feet
      sit 0.30 m above the road. Fixed in `fc8e10f`: ground under x/y (map lookup, cached until the walker
      moves 2 m) + lift; feet at 0.000 m after every teleport
- [x] `/control/control_mode_request` service in acb (AUTONOMOUS→ok, MANUAL→fail, matching
      stock; live 2026-09-27: `{mode: 1}` → true, `{mode: 4}` → false). Caveat: acb does not
      spin its executor while waiting for the hero, so calls made before the ego spawns sit
      unanswered until discovery — fixed in acb `7040b2d` (the waits pump the executor);
      live 2026-09-27: answered in 1 s with no hero spawned;
      stock `autoware_universe.cpp:52`)
- [x] Second `is_ego` rejected; unknown entity in `UpdateEntityStatus` rejected, not echoed;
      (both verified by probe, `ddf64d7`); the MANUAL-overwrite case needs a ControlModeReport
      subscription the bridge has no ROS node for — dropped;
      `overwrite_ego_status` also honored when ControlModeReport is MANUAL
- [x] `set_{green,yellow,red}_time(99999)` on every light (mapped or not) as backup to
      `freeze_all_traffic_lights`

### acb tick following (gap 7)

- [x] Measure: ego LiDAR `ros2 topic hz` with one stack — after the change, one stack:
      `/sensing/lidar/top/pointcloud_before_sync` 20.00 Hz (19.96–20.10, n=31), 22 of
      6385 frames skipped over a 300 s run; moving pose error vs rear axle fell
      0.324 → **0.102 m** (less publish latency). Before-change one-stack number not taken;
      the two-stack baseline is 10 Hz (roadmap README)
- [x] Replace `wait_for_tick_or_timeout` with an `on_tick` callback feeding a queue; drain to
      newest, warn on skipped frames
- [ ] Re-measure with two stacks; that number is the acceptance (needs the background
      stack on this host)

### Steering (gap 8)

Measured 2026-09-27 on `vehicle.tesla.model3` (`scripts/steering_probe.py`, raw data in
`docs/measurements/steering_tesla_model3_0916.csv`): the 0.9.16 command is **linear** —
inner wheel = 70° × cmd with 0.00° residual, outer wheel by Ackermann (47.4° at full lock;
that is the "45.5°" autoware_universe #13276 saw, not a squared input). TIER IV's sqrt fix is
a Chaos/0.10 matter and does not apply here. What does apply is `steering_curve`
(0→1.0, 20→0.9, 60→0.8, 120→0.7 km/h): the achieved angle equals the standstill angle times
the curve interpolated at the actual speed, to within 0.003, and acb's model
(`vehicle_control.rs:52-95`, exact at standstill) ignores it. The ego therefore under-steers
by 10 % at 20 km/h, 15–17 % at 40 and 25–27 % at 80. Wheel slew is ~150 °/s (full lock in
0.47 s); acb does not model it and Autoware's command rate makes that moot.

- [x] Measure: commanded tire angle vs `get_wheel_steer_angle` across the range at 0, 20,
      40 km/h — done (table above)
- [x] Compensate in acb: read `steering_curve` from `physics_control` at adoption and
      divide the steer command by `curve(v_kmh)`, clamped to ±1, so the achieved angle equals
      the commanded one at every speed. Chosen over flattening the curve with
      `apply_physics_control`, which rebuilds the vehicle's physics at spawn; the residual is
      only the clamp at large angles above ~60 km/h, irrelevant for Autoware's commands
- [x] Verify: achieved/commanded through acb — done 2026-09-27 (acb `88224c9`,
      `scratchpad/steercheck`): median **0.999** [IQR 0.998–1.000] over 30 samples at
      10.8–14.7 km/h, against 1.06–1.08 for achieved/(commanded × curve(v)), i.e. the
      compensation is what closed it. Only pull-away manoeuvres ≤16 km/h were available
      (see the traffic-light finding below); a check at 20–40 km/h through a real turn is
      closed 2026-09-28 below
- [x] Verify at 30 km/h through a steady turn (acb standalone in domain 7, δ = 0.06 rad,
      `scratchpad/steer30`): compensation on **1.0000** [IQR 1.0000–1.0000] over 123 samples at
      28.4–30.4 km/h; off **0.8763** against curve(30.2) = 0.8745; A/B 1.141 against 1/curve
      1.1435
- [ ] Steer rate limit and first-order lag in acb config, default off, values from SSv2
      #1849 as the documented starting point (20 °/s, τ=0.2 s) — unchanged, still optional

### Traffic-light scenario stalls between lights (found 2026-09-27, fixed 2026-09-28)

`town01_traffic_light.xosc` timed out stop-and-go and parked at signal 43763's stop line.
The held light phases were not the cause. **Autoware never received any signal state.**
Every `traffic_light_recognition` topic was empty for the whole run, so the traffic_light
module treated the signal as UNKNOWN and held its stop, while CARLA did turn green at
t = 150 s. Stock SSv2 delivers conventional signals only through simple_sensor_simulator's
pseudo detector, which this setup does not run, and 009's V2X decision (2026-08-28) was
never implemented.

- [x] SSv2 fork `2cddfac4d`, `publish_conventional_traffic_signals`: commanded conventional
      state merged with V2I into one `TrafficLightGroupArray` per cycle on
      `external/traffic_signals` (regulatory-element ids, 43856 for way 43763), on in
      `carla_scenario.launch.xml`. Live: `43856:RED`, then GREEN at sim 150, and planning
      follows it
- [x] **The time base was wrong too.** SSv2's `/clock` carried wall time; when its frame loop
      fell behind (0.26–0.5× under load) Autoware integrated simulated speeds over wall
      seconds, the EKF ran +53 m ahead and rejected NDT. Fork `2cddfac4d`,
      `clock_follows_simulation_time`: `/clock` = start + frames × step_time, start =
      the last `/clock` this domain saw plus one step (fork `a65f563f5`; the first version
      used max(wall now, last), and the forward leap by the idle gap made every stamp
      Autoware held stale at once -- about twenty ERRORs and an EMERGENCY_STOP per scenario
      before the ego existed). The gap between scenarios is a pause, as in the simulated
      world.
      Live: /clock/CARLA 0.999 at factor 1, 0.493 at factor 0.5 with 0 NDT time-validation
      errors
- [x] **Stale route, older and intermittent.** The concealer requested the route 41–92 ms
      after the EKF re-activated, and the EKF publishes only on a /clock tick. When no tick
      fell in that gap, mission_planner planned from the previous scenario's pose (seen:
      (88.9, −131.1) with the ego at (190.7, −130.1)); the ego weaved and once hit a wall.
      The same stale routes appear in logs from before the clock change. Fork `7908d82d0`:
      `initialize()` holds the task queue until `/localization/kinematic_state` is newer than
      the pre-init estimate and within 1.0 m / 0.2 rad of the initial pose, 10 s timeout with
      a named error. Live 2026-09-28: traffic_light → ego_drive × 4, **8/8 pass**; the wait
      triggered in 6 runs (100 ms, then the correct pose)
- [x] **No brake at low speed.** `build_longitudinal_maps.py` drops brake samples below 5 m/s
      and fills 0–4 m/s by copying, so the brake map's no-pedal row claims −2.55 m/s² where
      coasting is −0.28…−1.51. acb used that row as the throttle/brake boundary, so a −2.5
      stop from 2 m/s got pedal 0 and decelerated at −0.7…−0.9, and control_validator's
      acceleration check tripped the MRM. acb `87f8346`: the boundary is the measured coasting
      line and the brake covers the excess. Live: acceleration-error onsets while driving 11 → 1
      over 8 runs
- [x] Brake map re-measured below 5 m/s — acb `731b044`, 2026-09-28
      (`scripts/probe_brake_lowspeed.py`: 11 pedals × 1–6 m/s × 3 repeats, dv/dt fitted over
      target ± 1 m/s, first 0.4 s after brake-on and the stop ticks excluded; spread ≤ 0.009).
      The old floor was hiding a **physics-timing artefact**, not the stop: both maps had been
      measured under CARLA's default 10 ms substeps, where a braked car reads about −27 m/s²
      for several ticks below 5–6 m/s. The bridge runs 3.125 ms substeps since `ddf64d7`,
      so both maps were re-measured under the bridge's timing, and the builder now refuses
      probe files from a different timing. Pedal 0 at 0/2/4 m/s: −2.55 → −0.42/−0.84/−1.69;
      pedal 1.0: −5.17 → −3.71/−4.15/−4.96. The no-pedal rows of the two maps now agree
      within 0.002 m/s² at 2 and 4 m/s, so `command_for`'s increment logic reduces to a plain
      lookup (unit-tested on the shipped CSVs); −2.5 at 2 m/s → brake 0.503. 184 acb tests,
      ego_drive and pedestrian pass, no driving MRM onsets
- [x] MRM churn while driving: **fixed 2026-09-28, 69 → 0 driving MRM onsets over 7 runs.**
      The two control_validator checks that showed up next to it were never the trigger:
      neither is wired into the diagnostic graph. `latency` (`now − control_cmd.stamp`
      against 0.01 s) only fired 0–0.3 s *after* an MRM began, on the emergency operator's
      stale stamp; `max_distance_deviation` is the MPC's 5 s open-loop prediction drifting
      1.0–1.25 m in 90° turns at ~3 m/s while the real lateral error was 0.08–0.20 m -- an
      Autoware artifact, harmless. 26 of 27 real triggers were NDT `scan_matching_status`
      WARN "Couldn't interpolate pose": acb truncated CARLA's f64 time to ns, so scans sat
      1 ns after the EKF pose on the same /clock tick; NDT drops poses stamped before the scan
      it just matched, emptied its buffer, and failed every other scan (32 %). Any WARN makes
      autonomous mode unavailable, so mrm_handler went to EMERGENCY_STOP. The per-tick offset
      re-learn also flipped by a frame (74 duplicate scan stamps per run). acb `d28fe01`:
      CARLA time rounded to the microsecond, offset = mode of the last 100 readings (a jump
      over 0.5 s is a new scenario). After: NDT failures 0.2 % (run edges only), duplicate
      stamps ≤ 5, 7/7 pass, no validator threshold changed
- [x] SSv2 stepped at 20 Hz with the CARLA tick held at 0.05 s (`carla_tick_seconds` in
      bridge_config.yaml: ticks per frame follow the step, 2 at 10 Hz, 1 at 20 Hz) --
      `97f0295`. /clock in 0.05 s steps; 9/9 pass. It did not change the MRM counts (the
      cause was the stamps above), but it halves the fresh-pose wait (100 → 60 ms)
- [x] Unmanaged ego (its own domain) gets neither signals nor the sim clock -- closed by
      [015](015-carla-as-time-and-signal-source.md) step 5: acb publishes both from CARLA in
      every domain (2026-10-03)

### Emergency stop at every scenario start (fixed 2026-09-29)

Each scenario opened with ~28 diagnostic ERROR onsets and an EMERGENCY_STOP before the ego
existed, and engage took 6.5–18.9 s after the first tick. Timeline (`scratchpad/initmrm`):
the previous scenario left Autoware AUTONOMOUS on its route with the EKF still at 3.4 m/s;
the first /clock tick leapt over the idle gap; acb seeded /initialpose and the concealer
initialized again on top of it; NDT was asked to align before the new ego's first scans.

- [x] /clock continues from the last value (fork `a65f563f5`, above)
- [x] Concealer puts a reused Autoware back to STOP when the ego despawns, and waits for two
      scans from the new ego before initializing -- fork `7bfe6a159`; every init now within
      2 cm (was up to 1.6 m, once a reversed yaw)
- [x] acb publishes eight zero IMU + VelocityReport samples when the ego is lost, so the EKF
      does not carry its speed -- acb `ad80cd8` (EKF at rest 0.40–0.47 m/s, was 3.4)
- [x] `just ego-av` turns acb's /initialpose seed off under a managed ego; the concealer
      does that init -- `497c1c3`
- Result over 7 runs: init-window ERROR onsets 28/33/29/28/24/30/17 → 6/6/7/6/3/6/6,
  EMERGENCY_STOP before engage 7/7 → 0/7, engage 3.9–7.2 s, 7/7 pass. What remains is
  inherent to re-seeding a running Autoware: one pose_instability at the jump, one EKF
  initial-covariance tick, ADAPI state transitions, ~0.5 s of tracker/gyro timeouts before
  the first sensor frames
- [x] **acb loop stalls** -- fixed 2026-09-29, acb `0d091de`. Not CARLA RPC, locks or the
      follower: acb's main thread blocked on **writes to its own log file**
      (`play_log/.../acb_bridge/out` on `/home`, an ext4 spinning disk at 99 %), in D state in
      `wait_transaction_locked` for 0.5–3 s several times per scenario and 25 s when another
      user's `rm -rf` saturated it; `tf_bridge` logged "TF Buffer updated" for every
      `/tf_static` republish, ~40 lines/s. Every "Skipped N CARLA frames" and every gap in
      `/vehicle/status/velocity_status` lined up with a wait, and 79 of 95 driving
      EMERGENCY_STOPs in a 7-run baseline began inside one. Fix: tracing writes through a
      bounded queue (4096 lines) drained by a `log-writer` thread, dropping and counting
      lines when full; tf_bridge logs only when the transform count changes. After: 0 skip
      warnings (was 138), longest velocity gap 3.1 s and only during `/clock` pauses (was
      34.3 s), 0 of 13 driving MRMs tied to an acb gap (was 79 of 95), 7/7 pass (was 3/7)
- [x] **The 13 driving EMERGENCY_STOPs that remained** -- fixed 2026-09-29, acb `3405aaa` +
      `2634caf` (local, not yet pushed; pin not bumped). Not the iteration limit: every one
      (and the 3 pre-drive ones) was an NDT "Couldn't interpolate pose" WARN starting
      0.1-0.23 s before the MRM, on a scan stamped *after* the /clock of its own frame, so
      `SmartPoseBuffer::pop_old` dropped that frame's EKF pose. Three shapes: +1 us for ~2.5 s
      (8 of 13) -- CARLA steps `elapsed_seconds` by the f32 0.05 (0.050000000745 s), gaining
      a whole microsecond on SSv2's exact 50 ms grid every ~1340 ticks, and the 100-reading
      mode kept the old offset for half a window; +1 ns (3) -- SSv2's /clock wobbles by 1 ns;
      one frame ahead (2) -- the microsecond split halved the majority and the one-frame-low
      minority won. The graph needs `/autoware/localization` at OK: Autoware's converter
      publishes a mode available only when its unit is OK (`command_mode_mapping.cpp`,
      `unit->level() == DiagnosticStatus::OK`), mrm_handler then operates an MRM, and
      comfortable_stop needs localization too, so it is EMERGENCY_STOP. Upstream's
      `localization.yaml` has the same `and` over `scan_matching_status`: intended semantics,
      graph left alone. Fix: stamps on the grid of the newest /clock reading, 1 us early; the
      offset is the highest cluster of readings with >= 3 members (only the correct pairing
      reads high; a majority vote flipped every few ticks on an even split and mis-stamped
      23-52 scans a run)
- [x] "Off-route ERRORs at the end of the traffic_light route" -- a mislabel. Nothing in the
      logs reports off-route; the 12 onsets are `control_validation_max_distance_deviation`
      again, 55–60° into the last left turn (lanelet 39114, radius 11 m) at ~3 m/s, EKF at
      (90.3, −60.5), while the ego's real lateral offset is ≤ 0.18 m (runs with and without
      the ERROR drove within 5 cm of each other). MPC's 5 s open-loop prediction sits right
      at the 1.0 m threshold there and toggles every 50 ms. Not in the diagnostic graph, no
      effect on any verdict. No fix
- [x] **Emergency stop mid-turn** at (93, −58) -- fixed 2026-09-29, acb `3405aaa`
      (`acb_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml`,
      `step_size` 0.1 → 0.05, `max_iterations` 30 → 60). Not EKF lag (initial→result ≤ 6 cm,
      ≤ 5 mrad at 0.28 rad/s), not scan distortion or map density: at some poses the Newton
      step on CARLA's noise-free scan and map exceeds `step_size`, is clamped to exactly 0.1 m
      and flips ±(74, 2, 65) mm per iteration to the limit (oscillation count 29). Once the
      ego stopped at (88.9, −62.9) the cycle held for 2611 consecutive scans and the run timed
      out. Standalone ndt_scan_matcher replaying CARLA scans at ten poses: 0.1 → 75/520 at the
      limit, 0.05 → 1/520, same mean iterations; 60 iterations keep the 3 m reach. After:
      0 iteration-limit WARNs in 3 traffic_light runs (max 3 iterations in the turn)
- [x] behavior_path_planner aborted once (`terminate called ... failed to add guard
      condition to wait set: guard condition implementation is invalid`, 1 of ~64 route
      resets across 7 sessions): every reset recreates GoalPlanner's and StartPlanner's
      callback groups while the executor still holds the old guard conditions
      (autoware_universe#12460, rclcpp#2163, both open, no fix in 0.48.0/1.5.0). play_launch
      0.12 runs each composable in its own process and does not respawn one that crashes, so
      the container, the ADAPI and `/api/operation_mode/state` stayed up and
      `ego_stack_health.py` said "ok" for two scenarios that then sat at the spawn until
      their timeouts. Contained 2026-09-29: the health check also requires a publisher on
      each planning output and asks play_launch's ledger for crashed composables; the
      scenario gate runs it with `--reload-failed`, which POSTs `/api/nodes/<name>/load` and
      waits for the publishers to return. Longer term: composable respawn in play_launch, or
      an overlay of goal/start_planner with #12460's static callback groups

### Ego twist at base_link (fixed 2026-09-28)

- [x] acb `028d8a8`: odometry, ground truth and VelocityReport now carry the rear axle's
      velocity (v + ω × r in the body frame), not the actor origin's. Live, 30 km/h turn:
      `lateral_velocity` −0.0311 m/s against CARLA's rear axle −0.0310 (origin +0.201);
      `heading_rate` 0.1674 = CARLA yaw rate

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
