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

- **Simulation time is CARLA's `WorldSnapshot.timestamp.elapsed_seconds`, unmodified.** ROS
  time in every domain = that number. No epoch constant, no persisted last value, no learned
  offset. Two codebases (csb → SSv2, acb → Autoware) reading the same field need no
  agreement beyond the field. A CARLA restart or a map reload resets it; that is a real
  reset of the simulated world and is handled as one (below), not hidden behind an offset.
  *Risk accepted*: ROS time near zero right after a fresh CARLA start can make
  `now() - duration` negative in nodes that assume an epoch (tf2 cache, some Autoware
  monitors). The ego stack takes ~10 min to come up and acb attaches later still, so CARLA
  uptime is in the hundreds of seconds by then; acb refuses to publish `/clock` below 60 s
  of uptime and says why.
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
  `double simulation_time` (seconds, CARLA `elapsed_seconds` after the frame's last tick).
  The fork's `SimulationClock` gets a third mode, `clock_source: simulator`: ROS time = the
  last `simulation_time` received; `/clock` is **not** published (acb owns the topic in every
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
- **Reset is a first-class event, not smoothed over.** csb is the only process that changes
  CARLA's world (`load_world` when the town differs, `coordinator.rs:1025-1080`), so it is
  the one that knows time went backwards. It logs `Simulation time reset: <old> -> <new>` and
  reports the new time on the next response; acb sees `elapsed_seconds` decrease, logs an
  ERROR naming the reset and the fact that a long-lived Autoware must be restarted (TF
  buffers and every stamp it holds are now in the future), and keeps publishing the true
  time. Making the ego stack survive a reset is out of scope (below).
- **Order of work**: measure first (what the three clocks disagree by today), then acb's
  clock and stamps (self-contained, testable against the current fork), then the protocol
  field and csb, then the fork's `clock_source: simulator`, then the signal publisher and
  the GREEN default, then the unmanaged-mode run that proves the domain dependency is gone.
  Each step leaves the system passing the 014 suite on its own.

## Work Items

Measurement first. No step lands before the number it changes is known.

### Baseline (step 0)

- [ ] Record for one traffic_light run, per frame: SSv2's `/clock`, acb's sensor stamp, acb's
      odometry stamp, CARLA `elapsed_seconds` (from a passive client), the fork's signal
      stamp. Report the pairwise differences and their drift over 300 s. Expected from 014:
      `/clock − CARLA` constant to within 1 µs per ~1340 ticks, sensor stamp = `/clock` − 1 µs,
      odometry stamp = `/clock` (a frame late)
- [ ] Record what happens across a scenario boundary today: `/clock` continues (+1 step), no
      messages for the idle gap, first tick of the next scenario. This is the behaviour the
      "continuous /clock" decision replaces with a wall-paced idle

### acb: one clock, one stamp (step 1)

- [ ] `/clock` from the world `on_tick` subscription established at CARLA connect, value =
      `elapsed_seconds`, rate-capped at 100 Hz, refused below 60 s of CARLA uptime with an
      INFO line; hero attach and loss no longer start or stop it
- [ ] Sensor bridges stamp with `data.timestamp()` directly; `SimClockOffset` and its tests
      removed; `stamp_nanos` grid snap and 1 µs margin removed
- [ ] Odometry, ground-truth odom/TF, VelocityReport, SteeringReport, ground-truth objects
      stamped with the frame's `elapsed_seconds` passed in from the tick, not
      `ros_time_now_secs`. The despawn zero-twist samples (014) stamped with the last frame
      time seen
- [ ] Detect `elapsed_seconds` decreasing: ERROR once, naming the reset and the restart it
      needs; keep publishing
- [ ] Unit tests: stamp == `/clock` for the same frame (bit-exact), monotonic `/clock` under
      skipped frames, rate cap, uptime gate, reset detection
- [ ] Live, against the *current* fork (SSv2 still publishing `/clock`): two publishers on
      `/clock` for one run is expected to fight -- so run this step with
      `clock_follows_simulation_time:=false` and `publish_clock` on the fork off if it has a
      switch, else stop after the unit tests and go to step 2. Record scan stamp − EKF pose
      stamp over 20 runs: expect exactly 0, zero "Couldn't interpolate pose" WARNs

### Protocol and csb: report the time (step 2)

- [ ] `proto/simulation_api_schema.proto` and the fork's copy
      (`simulation/simulation_interface/proto/`): `double simulation_time = 2` on
      `InitializeResponse` and `UpdateFrameResponse`. Same file, same field numbers, checked
      by a test that diffs the two copies
- [ ] csb fills it from `world.snapshot().timestamp.elapsed_seconds` after the frame's last
      tick (and after the async no-op before the ego exists, so SSv2 has a time from the first
      frame)
- [ ] csb logs `Simulation time reset` when the value decreases (map reload); `load_world`
      is the only path that can cause it, so the log names the town change
- [ ] Unit test: the response time equals the snapshot after `substeps` ticks; the reset log
      fires on a decrease and not on a normal frame

### SSv2 fork: take time from the simulator (step 3)

- [ ] `SimulationClock` mode `clock_source: simulator` (parameter; default off, upstream
      behaviour unchanged): `getCurrentRosTime()` returns the last received
      `simulation_time`; `update()` no longer advances a counter; no `/clock` publisher; the
      `$TMP` persisted-time file removed
- [ ] `API::initialize`/`updateFrame` store the response's `simulation_time` before anything
      stamps; `traffic_lights` time source and `entity_manager` stamps both read it
- [ ] `carla_scenario.launch.xml`: `clock_source:=simulator`, `clock_follows_simulation_time`
      off, comment rewritten to point here
- [ ] Live: managed traffic_light → ego_drive × 3 + pedestrian. Signal stamp − `/clock` = 0 at
      the frame it was published for; no jump at scenario boundaries; the 014 suite passes;
      init-window ERROR onsets do not regress against 014's 3–7 per run

### Signals from CARLA (step 4)

- [ ] csb writes `<map dir>/traffic_lights.resolved.yaml` at Initialize: for each mapped
      lanelet way id, its regulatory element id(s), OpenDRIVE sign id, CARLA position; and
      the list of unmapped signals. Logs the path
- [ ] csb sets every mapped light the scenario has not commanded to GREEN at Initialize
      before `hold_light_phases`; `docs/design/scenario-authoring.md` says so
- [ ] acb `traffic_light_publisher.rs`: parameter `traffic_light_map_path` (empty = off),
      re-read on mtime change; each frame, `World::traffic_light_from_open_drive` for each
      mapped id (cache the actors, refresh on map change), read `state`, publish
      `TrafficLightGroupArray` on `/perception/traffic_light_recognition/external/traffic_signals`
      with one group per regulatory element id, CARLA colour → `TrafficLightElement`
      (red/amber/green, shape CIRCLE, status SOLID_ON, confidence 1.0), stamp = frame time,
      range filter reusing `ground_truth_range_m`
- [ ] `carla_scenario.launch.xml`: `publish_conventional_traffic_signals` back to false,
      comment explains the two sources and why only one may be on
- [ ] Live: traffic_light passes with the fork's publisher off; `judged/traffic_signals`
      carries `43856: RED` then `GREEN` at t=150 from acb's message; the arbiter's "not latest"
      warning is gone (one stamp base)

### Unmanaged ego made whole (step 5)

- [ ] `EGO_MANAGED=false just ego-av` + `town01_unmanaged.xosc`, then a copy of
      `town01_traffic_light.xosc` with the ego route moved to the pilot's goal file: the ego
      stops at the red and proceeds at the green in domain 3, with `/clock` from acb and
      signals from acb. 013's two open gaps close with this run
- [ ] `ego_av.launch.xml`: `publish_clock` no longer derived from `managed`; comment updated

### Reset (step 6, small)

- [ ] Live: change the town between two scenarios (or restart CARLA) with the ego stack up;
      confirm csb's and acb's reset lines fire, the stack's failure is named in the health
      check (`scripts/ego_stack_health.py`: a `/clock` that went backwards since the stack
      started is "restart required"), and a restarted stack recovers

## Acceptance

- One time base: for any frame, `sensor stamp == odometry stamp == /clock == SSv2 signal
  stamp == CARLA elapsed_seconds`, bit-exact, measured over a 300 s run. No offset, margin
  or grid snap left in acb or the fork.
- `/clock` is continuous and monotonic across 10 consecutive scenarios with idle gaps, and
  advances during the gaps.
- 20 scenario runs (managed) with zero NDT "Couldn't interpolate pose" WARNs and zero
  driving-window MRM onsets, at the same load as 014's verification.
- `town01_traffic_light` passes managed and unmanaged, with the fork's signal publisher off.
- A map reload is reported by csb and acb within one frame, and the health check refuses a
  run until the stack is restarted.

## Not in scope

- Surviving a time reset without restarting the ego stack (would need every Autoware node
  to handle a backward jump; TF alone clears its buffer and re-fills, the EKF does not).
- Motion-compensating the LiDAR sweep or otherwise changing what CARLA sensors report.
- A shared lanelet crate between csb and acb; the resolved-table file is enough for one
  consumer.
- Camera-based signal recognition (009, still open as its own gap).
- Replay of recorded scenarios: csb owning CARLA's clock makes it possible; the recorder and
  player are a phase of their own.
