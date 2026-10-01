# Phase 015: CARLA as the Single Source of Simulation Time and Signal State

Make CARLA's clock the one time base every process stamps with, and CARLA's traffic-light
actors the one place the ego learns signal state from. Today three clocks exist and two of
them are reconciled by a learned offset; every localization failure chased in phase 014
(1 ns, 1 µs drift, a one-frame flip, a leap over the idle gap) lived in that reconciliation.
Signal state reaches Autoware only from SSv2, only in SSv2's ROS domain, so an ego stack in
any other domain is blind to it.

**Source**: 014's clock and stamp fixes (SSv2 fork `2cddfac4d`, `a65f563f5`; acb `d28fe01`,
`3405aaa`, `2634caf`) and the user's direction on 2026-09-29: "take the CARLA clock as the
simulation clock SSoT; acb takes the clock from the simulator; csb can control the CARLA
clock in the case of replay or reload".
**Depends on**: [014](014-feature-completeness.md) shipped (its fixes are the fallbacks this
phase replaces one by one). The NEWSLabNTU fork carries the SSv2 side.
**Relates to**: [009](009-map-and-traffic-lights.md) (signal mapping, the V2X decision),
[013](013-forked-unmanaged-ego.md) (the unmanaged ego, which this phase makes whole),
[010](010-multi-instance.md) (one clock per domain).

## Problem

**Time.** Three time bases run at once:

| Who | Produces | From | Consumers |
|---|---|---|---|
| SSv2 fork | `/clock` (managed mode), signal stamps | start + frames × step; start = last persisted value + one step (`simulation_clock.cpp:56-95`) | the ego's Autoware (`use_sim_time=true`), the arbiter |
| acb | sensor/IMU stamps | CARLA `elapsed_seconds` + an offset learned against the node clock, snapped to the newest `/clock` tick minus 1 µs (`utils.rs:168-262`) | NDT, EKF, gyro_odometer |
| acb | odometry, VelocityReport, `/clock` in unmanaged mode | the node clock (`main.rs:1501,1538`); `/clock` = CARLA uptime minus the first value seen, only once a hero is attached (`main.rs:1415-1455`) | EKF, control |
| SSv2 | entity status stamps | wall time (`entity_manager.cpp:145,181,202`, `use_sim_time=false`) | SSv2's own nodes |

CARLA's step is an f32 (0.05 = 0.050000000745 s), so CARLA time and SSv2's exact grid drift
1 µs apart every ~1340 ticks; SSv2's own `/clock` wobbles by 1 ns; the offset window
flipped by a frame on a 50:50 split. Each of those made a scan stamp land after its frame's
EKF pose, NDT dropped the pose, warned, and the WARN took autonomous mode away. The fixes in
014 hold, but they are patches on a reconciliation that should not exist: **the simulator
already has the time**, and csb ticks it exactly once per SSv2 frame.

No SSv2 response carries time -- `InitializeResponse` and `UpdateFrameResponse` hold only
`result` (`proto/simulation_api_schema.proto:115,131`) -- so SSv2 cannot know CARLA's time
today, and csb never reads it (no `snapshot`/`elapsed_seconds` in csb's sources).

**Signals.** The commanded state's owner is SSv2's `TrafficLights`; csb writes it into CARLA's
light actors each frame (`coordinator.rs:2428-2485`), so CARLA holds the rendered truth.
Autoware learns it from two inputs to `traffic_light_arbiter`: vision (camera on CARLA's
render; 009 measured it too poor to rely on) and V2X (`external/traffic_signals`), which the
fork now publishes from SSv2's ground truth (014). That publisher lives in SSv2's ROS domain,
so it reaches a managed ego and nothing else. Nobody publishes what CARLA's lights actually
show; and lights the scenario never commands are frozen at whatever phase they were in
(`hold_light_phases`, `coordinator.rs:1323`), which the vision path would see and the V2X
path reports as nothing.

## Decisions (2026-09-29)

- **Simulation time is CARLA's `WorldSnapshot.timestamp.elapsed_seconds` plus an episode
  epoch, and the epoch follows one rule that every reader applies on its own.**
  `sim_time = elapsed_seconds + epoch`, epoch 0 at first, **in integer nanoseconds**:
  `sim_ns = ns(elapsed) + epoch_ns`, `ns(s)` = round half away from zero of `s * 1e9`
  (Rust `f64::round`), applied to each term on its own. Never an f64 sum rounded once: that
  disagrees with the per-term rule by 1 ns on a share of frames. CARLA restarts `elapsed_seconds`
  at 0 for every episode (`load_world`), and ROS has no general answer to a clock that goes
  backwards: the time design allows jumps and offers jump callbacks
  (design.ros2.org "Clock and Time"), but only rcl timers and tf2 ("Detected jump back in
  time ... Clearing TF buffer", geometry2#43) use them; Autoware's EKF, NDT and planners keep
  their state. Simulators therefore keep the clock running across a world reset -- Gazebo's
  `reset_world` resets poses and not time, and `reset_simulation`, which resets time, is the
  one that breaks nodes (gazebo_ros_pkgs#863); CARLA's own ros-bridge publishes raw
  `elapsed_seconds` and its users meet the tf2 warning after every `load_world`. So the
  episode change becomes a pause, not a rewind: whoever sees it sets
  `epoch_ns += ns(E_last) + ns(Δ_last)` (the old episode's last frame time and step), and the new
  episode's first frame lands one step after the old one's last. No learned offset, no
  persisted file, no channel between processes: csb and acb both watch the same frame
  stream and apply the same rule, so they agree bit-exactly provided they see the same
  last frame -- which csb guarantees by switching CARLA to synchronous mode *before* it
  reads the snapshot and reloads (nothing ticks in sync mode unless csb ticks). The one
  residual risk, a lost final `on_tick` callback putting acb one step off, is caught by the
  acceptance test (signal stamp − `/clock` == 0).
  *Risk accepted*: ROS time near zero right after a fresh CARLA start can make
  `now() - duration` negative in nodes that assume an epoch (tf2 cache, some Autoware
  monitors). The ego stack takes ~10 min to come up and acb attaches later still, so CARLA
  uptime is in the hundreds of seconds by then; acb refuses to publish `/clock` below 60 s
  of uptime and says why.
- **acb depends on CARLA and nothing else.** It must publish a correct, monotonic `/clock`
  with no csb running (someone driving CARLA directly, another scenario tool). Everything
  above is therefore computed by acb from CARLA alone; csb's job is only to make its own
  reading of the same rule agree, and to hand the result to SSv2.
- **acb publishes `/clock` in every mode, from CARLA connect, not from hero attach.** A world
  `on_tick` subscription exists without a vehicle. Between scenarios CARLA free-runs
  (async), so `/clock` advances wall-paced; during a scenario csb ticks it, so `/clock`
  advances one step per SSv2 frame. Continuous, monotonic, never leaps, never pauses.
  Publish rate capped at 100 Hz (CARLA async can exceed that). `publish_clock` stops being
  a mode switch; it stays as an emergency off.
- **acb stamps every message with the CARLA frame time it was computed from.** Sensors: the
  sensor data's own `timestamp` (CARLA sets it to the tick). Odometry, ground truth,
  VelocityReport, IMU: the snapshot `elapsed_seconds` of the frame being published, not the
  node clock (the node clock is acb's own `/clock` echoed back a frame late).
  `SimClockOffset` is deleted, with its 1 µs margin and its tests; the snap-to-grid property
  it protected becomes a *measured invariant* (Acceptance).
- **SSv2 takes time from the simulator.** `InitializeResponse` and `UpdateFrameResponse` gain
  `int64 simulation_time_ns` (integer nanoseconds, CARLA `elapsed_seconds` after the frame's
  last tick plus the epoch, by the rule above -- csb computes the same integer acb publishes;
  amended 2026-10-02 from a `double` in seconds, which SSv2 would have had to round again).
  The fork's `SimulationClock` gets a third mode, `clock_source: simulator`: ROS time =
  `rclcpp::Time(simulation_time_ns, RCL_ROS_TIME)`, the last value received, no conversion; `/clock` is **not** published (acb owns the topic in every
  domain); the persisted-last-value file goes away. Signal stamps and entity-status stamps
  both use it (the entity stamps are wall time today, a third base for no reason). SSv2's
  own nodes keep `use_sim_time=false` (the interpreter deadlocks otherwise -- 014).
  `clock_follows_simulation_time` from 014 stays as the fallback for a backend that cannot
  report time; the two are mutually exclusive.
- **Signal state reaches Autoware from CARLA's light actors, published by acb.** Same shape as
  `ground_truth_objects.rs`: each frame, read the state of every mapped light within range,
  publish one `TrafficLightGroupArray` on `external/traffic_signals`, ids = lanelet
  regulatory-element ids, stamp = the frame's CARLA time. Works in every domain, so the
  unmanaged ego gets signals for free. The fork's `publish_conventional_traffic_signals`
  goes back to off in `carla_scenario.launch.xml` (two publishers on the topic would
  interleave); it stays available for a non-CARLA backend.
- **The lanelet ↔ CARLA light mapping is produced by csb and consumed by acb through a file.**
  csb already resolves it (`traffic_light_mapper.rs`: explicit `config/traffic_lights_<town>.yaml`
  merged with a 5 m planar match against the lanelet map SSv2 names in `InitializeRequest`).
  At Initialize it writes the resolved table -- regulatory element id, way ids, OpenDRIVE
  sign id, CARLA actor position -- to `<map dir>/traffic_lights.resolved.yaml`; acb reads it
  from a `traffic_light_map_path` parameter and re-reads when the file changes. Chosen over
  teaching acb to parse lanelet2 (csb's `lanelet_map.rs` would have to move to a shared
  crate; do that later if a third consumer appears) and over an RPC between the two bridges
  (they share nothing but CARLA today, and that is a feature).
- **Uncommanded lights are set GREEN at Initialize, then held.** With CARLA as the source
  every light's state becomes visible to Autoware, so "frozen at whatever phase" would stop
  the ego at random intersections. GREEN is deterministic and permissive: a scenario commands
  the lights it cares about and nothing else changes behaviour. Documented in
  `docs/design/scenario-authoring.md`. (SSv2's own semantics -- uncommanded = no state --
  are kept for the V2X fallback path.)
- **An episode change is a logged pause, not a reset.** csb is the only process that reloads
  the world (`load_world` when the town differs, `coordinator.rs:1025-1080`). It switches to
  synchronous mode, reads the last snapshot, reloads, bumps its epoch by the rule, logs
  `Episode change: epoch <old> -> <new>, sim_time continues at <t>`, and reports the new time
  on the next response. acb notices the new episode (world id changed, `elapsed_seconds`
  restarted), bumps its epoch from the last tick it received, logs the same line, and
  publishes the next `/clock` one step after the last. Autoware sees a short pause, as it
  does between scenarios. A CARLA *restart* is different: acb reconnects to a server whose
  episode is new and whose previous last frame acb may not have seen; it then restarts its
  own epoch from the last `/clock` it published plus one step, which keeps Autoware's clock
  monotonic, and logs that csb's epoch (if a csb is running) will not match until the ego
  stack is restarted.
- **Order of work**: measure first (what the three clocks disagree by today), then acb's
  clock and stamps (self-contained, testable against the current fork), then the protocol
  field and csb, then the fork's `clock_source: simulator`, then the signal publisher and
  the GREEN default, then the unmanaged-mode run that proves the domain dependency is gone.
  Each step leaves the system passing the 014 suite on its own.

## Work Items

Measurement first. No step lands before the number it changes is known.

### Baseline (step 0)

- [x] Record for one traffic_light run, per frame: SSv2's `/clock`, acb's sensor stamp, acb's
      odometry stamp, CARLA `elapsed_seconds` (from a passive client), the fork's signal
      stamp. Report the pairwise differences and their drift over 300 s. Expected from 014:
      `/clock − CARLA` constant to within 1 µs per ~1340 ticks, sensor stamp = `/clock` − 1 µs,
      odometry stamp = `/clock` (a frame late)
      **Measured 2026-10-01** (traffic_light then ego_drive, both pass, acb 2634caf, managed,
      load 33-43 on 32 cores; recorder `scratchpad/p015/rec015.py`, analysis `ana015.py`;
      CSVs `scratchpad/p015/base.frames.csv` (per `/clock`: CARLA elapsed, frame, difference)
      and `base.stamps.csv` (per stamp: nearest `/clock`, difference)). The traffic_light run
      is 3432 frames (171.6 s sim, 215 s wall), not 300 s; numbers scale linearly.
      - `/clock − CARLA elapsed`: one CARLA frame per `/clock` (3430 of 3432; one 0, one 2).
        Drifts **−0.745 ns/frame**: −2557 ns over the run (−416 ns over ego_drive's 560
        frames); −4.47 µs at 300 s. `/clock` steps are 50 ms exactly but for six ±1 ns.
      - LiDAR / IMU / NDT-input stamps: `/clock − 1000 ns` for 3957 of 3970 (99.7%); 6 at
        exactly `/clock`, 7 at ±1 ns off the margin. At each run's start they lag: the first
        ~12 traffic_light samples all carry the frozen pre-start `/clock` (up to 550 ms behind
        the newest), the first 3 of ego_drive are 250 ms behind, until the offset settles.
      - Odometry (`/carla/ground_truth/odom`), VelocityReport, SteeringReport: the node clock
        through an f64, so **never exact**: `/clock` + {22, 70, −26, −121, −73, 21, ...} ns
        (f64 at 1.79e9 s). Relative to the newest `/clock` the recorder had seen: same frame
        73%, one frame newer 21%, one frame older 6%.
      - EKF `kinematic_state` and biased pose: exactly `/clock` (3891/3891, 3892/3892).
      - Fork signal stamp (`external/traffic_signals`): exactly `/clock`, 1972/1972.
      - NDT scan − EKF pose stamp: −1000 ns 3854 of 3870, 0 ns ×6, ±1 ns ×7, 50-150 ms ×6
        (run start). 0 "Couldn't interpolate pose", no MRM onset.
- [x] Record what happens across a scenario boundary today: `/clock` continues (+1 step), no
      messages for the idle gap, first tick of the next scenario. This is the behaviour the
      "continuous /clock" decision replaces with a wall-paced idle
      **Measured** (same recording): last traffic_light `/clock` 1790611998.740707376, first
      ego_drive `/clock` 1790611998.840707376: **+0.100 s (two steps, not one)** over 101.6 s
      of wall time, while CARLA free-ran 90.04 s / 4204 frames asynchronously (41 Hz, delta
      0.021 s). In the gap: no `/clock`, no sensor or status messages, except acb's 8
      despawn standstill IMU+VelocityReport pairs 12 s after the last frame, stamped
      1790611998.690707445 -- one frame *behind* the last `/clock`, and 445 ns off its grid.
      The next scenario's first `/clock` pairs with a CARLA frame 16 after the previous
      pairing (the catch-up into sync mode). Within a scenario the same pattern holds at the
      initialize pause: one `/clock`, 32.9 s of silence, then +0.05 s.

### acb: one clock, one stamp (step 1)

- [x] `/clock` from the world `on_tick` subscription established at CARLA connect, value =
      `elapsed_seconds`, rate-capped at 100 Hz, refused below 60 s of CARLA uptime with an
      INFO line; hero attach and loss no longer start or stop it
- [x] Sensor bridges stamp with `data.timestamp()` directly; `SimClockOffset` and its tests
      removed; `stamp_nanos` grid snap and 1 µs margin removed
- [x] Odometry, ground-truth odom/TF, VelocityReport, SteeringReport, ground-truth objects
      stamped with the frame's `elapsed_seconds` passed in from the tick, not
      `ros_time_now_secs`. The despawn zero-twist samples (014) stamped with the last frame
      time seen
- [x] Episode epoch: track world id and `elapsed_seconds`; on a new episode set
      `epoch += E_last + Δ_last` from the last tick received and log the change; on a CARLA
      reconnect with an unknown previous frame, continue from the last published `/clock`
      plus one step and log that csb's epoch may differ
- [x] Unit tests: stamp == `/clock` for the same frame (bit-exact), monotonic `/clock` under
      skipped frames, rate cap, uptime gate, the epoch rule on an episode change (new
      first frame = old last + Δ), the reconnect fallback

      Done in acb `a038be9` (main, unpushed; superproject pin not bumped). `clock.rs`:
      `EpisodeClock` holds the rules (pure, 6 new tests), `SimClock::stamp` is the one
      conversion every stamp and `/clock` uses, `spawn_clock_source` runs `/clock` on a CARLA
      client of its own from node start (the session client is dropped after every despawn),
      re-subscribing if the world is replaced and reconnecting if CARLA stops answering.
      `utils.rs` is down to 13 lines; `ClockEpoch` and the `ACB_CLOCK_DECIMATE` hook went
      too. Arithmetic is integer nanoseconds: `nanos(s) = round(s * 1e9)`, **half away from
      zero**, applied to each term; the epoch rule is `epoch_ns += nanos(E_last) +
      nanos(Δ_last)`. csb's `episode_clock.rs` keeps the epoch as an f64 and reports
      `elapsed + epoch` as a double, which SSv2 turns into a ROS time; unless that conversion
      rounds the same way, signal stamps will differ from acb's `/clock` by 1 ns on a share
      of frames (Python's half-to-even `round` already disagrees on 96 of 6683 frames
      measured below; a truncating `from_seconds` would disagree on about half). Step 3's
      acceptance (signal stamp − `/clock` == 0) is where that shows.
      124 unit tests pass; clippy has the 5 pre-existing warnings and no new ones.
- [ ] Live, against the *current* fork (SSv2 still publishing `/clock`): two publishers on
      `/clock` for one run is expected to fight -- so run this step with
      `clock_follows_simulation_time:=false` and `publish_clock` on the fork off if it has a
      switch, else stop after the unit tests and go to step 2. Record scan stamp − EKF pose
      stamp over 20 runs: expect exactly 0, zero "Couldn't interpolate pose" WARNs

      **Not run live.** The installed fork publishes `/clock` unconditionally
      (`api.cpp:147`); `clock_follows_simulation_time:=false` only switches its value to wall
      time. The `clock_source: simulator` switch exists in the fork source since step 3 but
      also needs csb's step-2 `simulation_time`, and neither is built or running. The new
      acb is also **not deployable** in a managed stack until then: its stamps are CARLA time,
      SSv2's `/clock` is not. Do not rebuild this repo's `install/` from acb `a038be9` before
      step 3 lands.
      **Standalone instead** (2026-10-02, acb `a038be9` dev build alone in domain 7,
      vehicle_name `p015hero` so the live ego's acb ignores it, CARLA asynchronous at ~45 Hz,
      LiDAR + IMU attached; `scratchpad/p015/sa/run.sh`, recording `sa/run1/rec`, analysis
      `sa/ana_sa.py`): `/clock` ran from node start, 1340 messages in the 30 s before any
      hero and 1804 in the 40 s after it was destroyed; 0 non-increasing steps in 6712;
      every `/clock` equal to the passive client's `elapsed_seconds` in ns (6683/6683, epoch
      0); every LiDAR (3522), IMU (3531), ground-truth odom (1485), VelocityReport (1493) and
      SteeringReport (1485) stamp equal to its CARLA frame time, and all but 2 LiDAR/IMU
      stamps equal to a published `/clock` -- those two belong to a frame 3.4 ms after the
      previous one, which the 100 Hz cap dropped, as designed. The 8 despawn standstill
      samples carried the last frame's stamp. Not exercised live: the 60 s gate (CARLA uptime
      was 237 000 s) and an episode change (a `load_world` would have taken the live stack's
      world); both are covered by unit tests only, and step 6 is their live test.
      Superseded by step 3's live run below, which is this check with the fork's `/clock`
      gone: 0 interpolate WARNs, scan stamp == EKF pose stamp source (both a `/clock` value).

### Protocol and csb: report the time (step 2)

- [x] `proto/simulation_api_schema.proto` and the fork's copy
      (`simulation/simulation_interface/proto/`): `int64 simulation_time_ns = 2` (was
      `double simulation_time`, changed in the integration below) on
      `InitializeResponse` and `UpdateFrameResponse`. Same file, same field numbers, checked
      by a test that diffs the two copies
- [x] csb fills it from `world.snapshot().timestamp.elapsed_seconds` after the frame's last
      tick (and after the async no-op before the ego exists, so SSv2 has a time from the first
      frame)
- [x] csb keeps the same epoch rule: `load_world` first switches CARLA to synchronous mode,
      reads the last snapshot, reloads, sets `epoch += E_last + Δ_last`, logs the episode
      change naming the town, and reports `elapsed + epoch` from then on
- [x] Unit tests: the response time equals the snapshot after `substeps` ticks plus the
      epoch; the epoch rule on a reload; sync mode is on before the pre-reload snapshot

      Done offline (csb `0ec3364`, fork `357b3be15`, unpushed; superproject pin not yet
      bumped): `episode_clock.rs` holds the rule and its tests, `proto::tests` diffs all eight
      proto files against the fork's copies. Not yet run against a live CARLA.
      **Integer ns (2026-10-02)**, csb `a50f430`, fork `f0e480452`, acb `a0f8b8e`: the field is
      `int64 simulation_time_ns`; `episode_clock.rs` keeps `epoch_ns` and reports
      `nanos(elapsed) + epoch_ns` with `nanos` = acb's (`f64::round`, half away from zero, per
      term); the fork takes `rclcpp::Time(ns, RCL_ROS_TIME)` with no conversion. csb and acb
      carry the same table of 15 `(seconds, ns)` cases plus 7 f32-step accumulations (exact
      halves, 3432 steps of 0.050000000745 = 171 600 002 557 ns, ...). Live below.

### SSv2 fork: take time from the simulator (step 3)

- [x] `SimulationClock` mode `clock_source: simulator` (parameter; default off, upstream
      behaviour unchanged): `getCurrentRosTime()` returns the last received
      `simulation_time`; `update()` no longer advances a counter; no `/clock` publisher; the
      `$TMP` persisted-time file removed
- [x] `API::initialize`/`updateFrame` store the response's `simulation_time` before anything
      stamps; `traffic_lights` time source and `entity_manager` stamps both read it
- [x] `carla_scenario.launch.xml`: `clock_source:=simulator`, `clock_follows_simulation_time`
      off, comment rewritten to point here

      Edited, not yet built or verified (fork `bd6197f42` on `managed-ego-unforked`,
      unpushed; superproject pin not yet bumped). `clock_source` is `frames` (stock),
      `follows_simulation_time` (014; `clock_follows_simulation_time:=true` still maps to
      it) or `simulator`; `simulator` with `clock_follows_simulation_time:=true` is an
      error. One deviation from the item above: `update()` still advances the frame
      counter, because that counter is the *scenario* time (`SimulationTimeCondition`,
      NPC behaviour step), not ROS time; only `getCurrentRosTime()` follows the
      simulator. The zmq client already returned whole responses, so no client change;
      `API::init`/`updateTimeInSim` read `simulation_time` from them. gtests for the
      three modes in `traffic_simulator/test/src/simulation_clock`.
- [x] Live: managed traffic_light → ego_drive × 3 + pedestrian. Signal stamp − `/clock` = 0 at
      the frame it was published for; no jump at scenario boundaries; the 014 suite passes;
      init-window ERROR onsets do not regress against 014's 3–7 per run
      **Run 2026-10-02** (acb `a0f8b8e`, csb `a50f430`, fork `f0e480452`, fresh ego stack with
      acb `publish_clock` on in managed mode -- `just ego-av` no longer derives it; recorder
      `scratchpad/p015int/suite.rec`, analysis `ana_int.py`, epoch 0, load 12-21 on 32 cores).
      5/5 pass. Over 772 s wall / 18 237 `/clock` messages / 7 481 scenario frames:
      - every `/clock` == a CARLA frame's `nanos(elapsed)`: 0 mismatches; 0 non-increasing
        steps; exactly one publisher (`acb_bridge`) before, between and after the runs
      - every stamp == a CARLA frame time **and** a published `/clock`, bit-exact, 0
        mismatches: LiDAR 7417, IMU 7458, NDT input 7417, ground-truth odom 7411,
        VelocityReport 7451, SteeringReport 7411, fork signals 3624, EKF kinematic 10 880,
        EKF biased pose 10 881. Each of the 3300 distinct signal stamps is also a LiDAR stamp
      - 4 scenario frames have no `/clock`: each scenario's last sync frame, 1.5 ms before the
        first async one, dropped by the 100 Hz cap as designed
      - idle gaps (36-50 s): `/clock` advances wall-paced, 1713-2310 messages per gap. Each
        scenario end holds it ~10 s (CARLA left in sync, unticked, until csb returns it to
        async) -- the concealer's STOP handover runs inside that hold
      - 0 NDT "Couldn't interpolate pose"; 0 MRM onsets while driving; two 0.1-0.2 s
        EMERGENCY_STOP blips at init (tl, pedestrian), with the EKF/ellipse init ERROR, in STOP
      - **Init-window ERROR onsets regress: 2 / 12 / 10 / 12 / 14** (014: 3-7). The new ones
        are the four `topic_state_monitor`s (steering, velocity, obstacle pointcloud, traffic
        signal; + trajectory twice) going ERROR 1-4 s after a scenario starts: the previous
        ego lives on through the idle gap and is destroyed at the next Initialize, and for
        the ~8 s until the new ego's first frame `/clock` now keeps running (CARLA async,
        `WaitForEgo`) where SSv2's frame clock used to stand still. No effect on any verdict
        or on driving; Autoware is in STOP then. Open item below
      - STOP handover: "Left Autoware in STOP for the next scenario" 5/5; no fork change
        needed (the destructor runs while `/clock` is held, as its comment assumes)
      - The concealer's initial-pose stamp is still the interpreter's wall time
        (~1.79e9 s) against a ROS time of ~2.4e5 s. Nothing logged about it: pose_initializer
        and the EKF take the pose, not the stamp; localization reached the pose after 0 ms
        in 5/5. A fourth time base nonetheless; stamping it with the simulation clock is a
        one-line fork change left for step 4's fork work
- [x] Init-window regression above. **Fixed 2026-10-02** (csb, `sync_before_spawn`), but the
      cause was not the one guessed here. Measured on the integration run
      (`scratchpad/p015s4/init_onsets.py`, `mrm_classify.py`): csb's log shows SSv2 despawning
      every ego at its own scenario end, so no ego survives the gap. The new onsets were two
      things:
      - **The four `topic_state_monitor`s are idle-gap timeouts, not init.** Their timeout is
        30 s of ROS time; `/clock` now runs through the gap, so 30.0 s after csb returns CARLA
        to async they go ERROR -- every one *before* the next `Initialize` (4-10 s before it).
        The harness starts `just scenario` 30 s after the previous run, so they landed in the
        launch-anchored window. Anchored at `Initialize` the integration run had 2/7/6/8/10.
        No vehicle exists then; the ERROR is true, and clears at the next ego's first frame.
        What acb can say truthfully it now says: its signal publisher runs from node start,
        so the traffic-signal monitor no longer times out (step 4). Steering, velocity and
        pointcloud still do (3 per gap); faking a vehicle that does not exist was rejected.
      - **The two EMERGENCY_STOP blips were `/clock` leaps.** CARLA async advances
        `elapsed_seconds` by wall time; a spawn that stalls the server comes back as one frame
        that leaps by the stall: +1.54 s at traffic_light's ego spawn (first vehicle after a
        stack start), +6.63 s at pedestrian's walker spawn (ego already spawned, first frame not
        yet sent). acb publishes it faithfully, every heartbeat times out at once, and
        mrm_handler blipped EMERGENCY_STOP 0.13 s later both times.
      Fix: csb switches CARLA to synchronous *before* spawning the ego (the spawn's own tick
      puts it in the episode, so acb finds it as before), so a stall is a pause.
      `EnableSyncThenTick` stays as the fallback. Live (acb `9e93260`, two suites tl/ed/ed/ped,
      4/4 pass each): `/clock` steps > 0.3 s during runs **0** (was 2), EMERGENCY_STOP blips
      **0** (was 2), init-window onsets anchored at `Initialize` **0/7/7/5/6** and **0/7/6/4**
      (014: 3-7); anchored at launch 0/10/10/8 and 0/10/9/7, the difference being the three
      idle-gap monitors above

### Signals from CARLA (step 4)

- [x] csb writes `<map dir>/traffic_lights.resolved.yaml` at Initialize: for each mapped
      lanelet way id, its regulatory element id(s), OpenDRIVE sign id, CARLA position; and
      the list of unmapped signals. Logs the path
      (`resolved_signals.rs`; map dir = the map file's directory *symlinks resolved*, so the
      scenario's `csb_launch` share path and the ego's `map_path` reach the same file; a
      read-only map dir falls back to `$XDG_RUNTIME_DIR/carla_scenario_bridge/<town>/`,
      logged as a WARN naming the path; written atomically)
- [x] csb sets every mapped light the scenario has not commanded to GREEN at Initialize
      before `hold_light_phases`; `docs/design/scenario-authoring.md` says so
- [x] acb `traffic_light_publisher.rs`: parameter `traffic_light_map_path` (empty = off),
      re-read on mtime change; each frame, `World::traffic_light_from_open_drive` for each
      mapped id (cache the actors, refresh on map change), read `state`, publish
      `TrafficLightGroupArray` on `/perception/traffic_light_recognition/external/traffic_signals`
      with one group per regulatory element id, CARLA colour → `TrafficLightElement`
      (red/amber/green, shape CIRCLE, status SOLID_ON, confidence 1.0), stamp = frame time,
      range filter reusing `ground_truth_range_m`
      (acb `9e93260`. Own CARLA client and `on_tick`, from node start like the clock, so it
      publishes between scenarios too (no hero: every mapped light). Disagreeing lights of
      one regulatory element report the most restrictive colour; Off/Unknown → UNKNOWN at
      confidence 0. Throttled on *simulation* time (40 ms), not wall time: SSv2's frames come
      in pairs ~17 ms apart and a 20 ms wall cap dropped every other scenario frame. Wired
      through `acb_bridge.launch.xml`, `ego_av.launch.xml` (default `$(var map_path)/
      traffic_lights.resolved.yaml`) and `just ego-av` (`TRAFFIC_LIGHT_MAP_PATH`, empty = off))
- [x] `carla_scenario.launch.xml`: `publish_conventional_traffic_signals` back to false,
      comment explains the two sources and why only one may be on. **Not sufficient on its
      own**: SSv2's V2I channel publishes on the same topic every frame regardless (upstream
      behaviour; an empty array when the scenario has no V2I light). Measured: two publishers
      during each run, every other message empty. A `set_remap` around the include moves the
      interpreter's copy to `/simulation/v2i/external/traffic_signals`
- [x] Live: traffic_light passes with the fork's publisher off; `judged/traffic_signals`
      carries `43856: RED` then `GREEN` at t=150 from acb's message; the arbiter's "not latest"
      warning is gone (one stamp base)
      **Run 2026-10-02** (acb `9e93260`, csb `69603ef`, fork `f0e480452` unchanged; suite tl → ed
      → ed → ped, `scratchpad/p015s4/rec4.py`, `ana4.py`; 4/4 pass):
      - exactly one publisher on `external/traffic_signals`, `acb_bridge`, in 42 samples taken
        every 15 s through the suite (scenario stacks up and down)
      - 43856 in acb's message: RED from the first frame the ego is within 100 m (41.75 s after
        the ego spawn) until GREEN at 159.8 s (SSv2's t > 150 command); `judged` switches on
        the same frames. Every scenario frame has a signal message (6994/6994, 0 without a
        group); 4129 more between scenarios
      - signal stamp == `/clock`, bit-exact: every message stamped with a CARLA frame time;
        all but 6 of 11 119 equal a published `/clock` -- 3 at recorder start, 3 each a
        scenario's last frame, the one the `/clock` 100 Hz cap drops by design
      - 0 MRM state changes (0 EMERGENCY_STOP at init, 0 driving), 0 NDT interpolate WARNs;
        4 driving onsets, all `control_validator max_distance_deviation` (014: not wired to MRM)
      - **"not latest" is not gone**: 69 in this stack's log (throttled, one per ~6 s for the
        whole of every run, ed and ped included), against 34 over the integration suite (tl
        only). The arbiter warns when an input is older than what it last published; acb's
        V2X message is always the newest frame, the camera classification path lags it. One
        stamp base does not remove that. Harmless to the verdicts; left open
      - first suite of the day (before the remap and the sim-time throttle) also passed 4/4;
        it is the measurement that found the second publisher

### Unmanaged ego made whole (step 5)

- [ ] `EGO_MANAGED=false just ego-av` + `town01_unmanaged.xosc`, then a copy of
      `town01_traffic_light.xosc` with the ego route moved to the pilot's goal file: the ego
      stops at the red and proceeds at the green in domain 3, with `/clock` from acb and
      signals from acb. 013's two open gaps close with this run
- [x] `ego_av.launch.xml`: `publish_clock` no longer derived from `managed`; comment updated
      (done with step 3, csb `a50f430`: a managed ego has no other `/clock` once the fork runs
      `clock_source: simulator`; `ACB_PUBLISH_CLOCK=false` is the emergency off)

### Episode change (step 6)

Not run live (2026-10-02): only Town01 scenarios exist, and a `load_world` would take the
shared CARLA. Covered offline: csb `a_reload_continues_one_step_after_the_last_frame`,
`epochs_accumulate_over_several_reloads`, `sync_mode_is_on_before_the_pre_reload_snapshot`;
acb `episode_change_continues_one_step_after_the_last_frame`,
`reconnect_continues_from_the_last_published_clock`; and on both sides the same `nanos` table
plus `sim_ns_rounds_each_term_not_the_sum` / `epoch_sums_rounded_terms_like_csb`, which pin
the post-reload time to `nanos(elapsed) + nanos(E_last) + nanos(Δ_last)` in both crates.

- [ ] Live: change the town between two scenarios with the ego stack up. Both episode-change
      lines fire with the same epoch; `/clock` never decreases; the next scenario's signal
      stamps equal `/clock` (bit-exact); no "jump back in time" from tf2; the ego drives
- [ ] Live: restart CARLA with the ego stack up. acb's reconnect fallback keeps `/clock`
      monotonic; the health check names the restart the stack now needs for csb and acb to
      agree again

## Acceptance

- One time base: for any frame, `sensor stamp == odometry stamp == /clock == SSv2 signal
  stamp == CARLA elapsed_seconds`, bit-exact, measured over a 300 s run. No offset, margin
  or grid snap left in acb or the fork.
- `/clock` is continuous and monotonic across 10 consecutive scenarios with idle gaps, and
  advances during the gaps.
- 20 scenario runs (managed) with zero NDT "Couldn't interpolate pose" WARNs and zero
  driving-window MRM onsets, at the same load as 014's verification.
- `town01_traffic_light` passes managed and unmanaged, with the fork's signal publisher off.
- A map reload between scenarios is a pause: `/clock` never decreases, csb's and acb's
  epochs agree, and the next scenario passes without restarting the ego stack.
- acb publishes the same `/clock` with csb stopped (CARLA driven by hand) as with it.

## Not in scope

- Surviving a *CARLA server restart* without restarting the ego stack: acb keeps the clock
  monotonic, but csb's epoch cannot be made to agree without a channel, and the sensors
  and the hero are gone anyway.
- Motion-compensating the LiDAR sweep or otherwise changing what CARLA sensors report.
- A shared lanelet crate between csb and acb; the resolved-table file is enough for one
  consumer.
- Camera-based signal recognition (009, still open as its own gap).
- Replay of recorded scenarios: csb owning CARLA's clock makes it possible; the recorder and
  player are a phase of their own.
