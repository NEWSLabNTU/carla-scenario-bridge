# Time and Ticking

How simulation time advances and is observed across CARLA, the scenario bridge (csb), the
Autoware bridge (acb) and SSv2. Authoritative for the clock and the tick; other design
docs point here. History and measurements: roadmap
[015](../roadmap/015-carla-as-time-and-signal-source.md).

## One source of time

Simulation time is CARLA's `WorldSnapshot.timestamp.elapsed_seconds` plus an **episode
epoch**, in integer nanoseconds:

```
sim_time_ns = nanos(elapsed_seconds) + epoch_ns
nanos(s)    = round-half-away-from-zero(s * 1e9)
```

- **acb** publishes `/clock` from it in every ROS domain, and stamps every message it
  publishes (sensors, odometry, vehicle status, ground-truth objects, traffic signals) with
  the time of the CARLA frame the message was computed from. acb depends on CARLA and
  nothing else: no csb, no SSv2.
- **csb** reports the same number to SSv2 as `simulation_time_ns` on every
  `InitializeResponse` and `UpdateFrameResponse`.
- **SSv2** (fork, `clock_source:=simulator`) takes its ROS time from that field and
  publishes no `/clock`.

Nothing learns an offset; every stamp is a CARLA frame time. The epoch exists only so that
time never runs backwards when CARLA starts a new episode (below).

## One ticker

CARLA runs **synchronous from csb's first `Initialize` on, and csb is the only process that
ticks it.** Nothing else ever advances the world:

| Phase | Ticks |
|---|---|
| Scenario running | csb, `N = round(step_time / carla_tick_seconds)` ticks per SSv2 frame |
| Initialize → ego spawn | the spawn's own tick (csb), nothing else |
| Between scenarios | none: the world, and `/clock`, are **paused** |
| Map load | CARLA's `LoadEpisode` (see Episode change) |

Idle is a pause, not free-running. A long-lived Autoware sees the gap between two scenarios
as stopped time, as it would in any simulator that pauses. This is what removes, by
construction, the problems free-running caused: `/clock` leaps when the server stalled
while async (2.99 s before a spawn), topic monitors timing out between scenarios as ROS
time ran on with no vehicle, and async frames arriving closer together than acb's 100 Hz
`/clock` cap.

acb needs no free-running world to find its vehicle: it follows `on_tick`, and the ego's
spawn tick delivers the first frame with the hero in it. (Before 015 acb polled
`world.actors()`, which is why sync mode used to be deferred until after the spawn.)

**No idle watchdog.** csb used to hand CARLA back to async after 10 s without a request
(300 s with an ego) so a dead scenario would not leave the world frozen. That free-running
is what this design removes, and a ZMQ REP socket cannot tell a dead SSv2 from a long pause
anyway. A graceful csb shutdown still restores async; after a csb crash CARLA stays paused
until the next csb takes it (which runs it synchronous anyway). Any other CARLA client that
waits for ticks will wait while no scenario runs.

## Tick granularity

```
SSv2 frame   step_time = realtime_factor / global_frame_rate   (0.05 s at 20 Hz)
 └─ CARLA tick   fixed_delta_seconds = carla_tick_seconds        (0.05 s)
     └─ PhysX substep  max_substep_delta_time × max_substeps     (0.003125 s × 16)
```

- **The CARLA tick is pinned** (`carla_tick_seconds` in `bridge_config.yaml`): sensors are
  tuned to it, not to the SSv2 frame. The ego LiDAR fires once per tick at a 20 Hz rotation,
  so one tick is one full sweep only at 0.05 s; the IMU's spacing against gyro_odometer's
  0.2 s tolerance is the tick too. The SSv2 frame rate only decides how often SSv2 (and
  ROS time) advances: 2 ticks per frame at 10 Hz, 1 at 20 Hz.
- **Physics substeps** are internal to a tick and invisible above CARLA. CARLA requires
  `max_substep_delta_time × max_substeps ≥ fixed_delta_seconds`; csb wants ≤ 2 ms substeps
  and clamps to CARLA's 16, so a 0.05 s tick runs 16 × 3.125 ms.
- **Control** from Autoware is applied by acb when it arrives and takes effect at the next
  tick, so a command waits at most one CARLA tick (50 ms).

## Episode change

CARLA restarts `elapsed_seconds` at 0 for every episode (`load_world`, `reload_world`). ROS
has no general answer to a clock that goes backwards -- only rcl timers and tf2 handle the
jump, and Autoware's EKF, NDT and planners keep their state -- so an episode change is a
**pause**: the new episode's first frame lands one tick after the old episode's last.

Rule, applied independently by every reader of the frame stream:

```
epoch += E_last + Δ_last     # E_last, Δ_last: the old episode's last frame
```

**acb** takes `E_last` from the last `on_tick` frame it received from the old episode.

**csb**, the only process that loads maps, must arrive at the same frame without a channel
to acb. It cannot simply read a snapshot before loading: CARLA's `LoadEpisode` sends its own
tick cue while it waits, so the old episode advances after the snapshot (measured: exactly
one frame per load). csb instead counts:

```
before:  sync mode on; snapshot → (F0, E0, Δ)
load:    load_world(town, reset_settings = false)      # new world stays synchronous
after:   snapshot → (F1, E1)
n_new  = round(E1 / Δ)          # frames the new episode has had (it only advances by Δ)
k_old  = (F1 − n_new) − F0      # old-episode frames after csb's snapshot
E_last = E0, then + Δ, k_old times   # accumulated as CARLA accumulates, not multiplied
```

This needs two CARLA properties, both verified live on 0.9.16 (2026-10-05): the frame
counter continues across episodes, and with `reset_settings = false` the new episode stays
synchronous and starts at `elapsed = Δ` after one tick. Measured: `k_old = 1`, `n_new = 1`,
`E1 = Δ`, both directions, loads of 2.3 s. With `reset_settings = true` (the default) the new
world reverts to async and free-runs for several frames before anyone reads it, which both
breaks the count and made loads slow (44 s, and one > 120 s).

A **CARLA restart** is treated as an episode change whose last old frame is the last frame
each bridge observed: acb continues from the last `/clock` it published plus one tick, and
csb (roadmap 016) from the last frame it ticked plus one tick. Both saw the frame csb last
ticked, so the epochs agree unless acb missed that frame. See
[failure-and-frame-budget](failure-and-frame-budget.md).

**Known limitation (accepted, roadmap 016 gap 12).** A restarted CARLA free-runs before
csb's first `Initialize`. If that server dies again before csb has read a frame from it,
acb has seen its frames and applies the episode rule to the last one, while csb has seen
none and keeps its epoch: from then on the two differ by a constant (265.86 s measured).
Both stay monotonic and scenarios pass; restarting the ego stack (acb) clears it. A csb
restart is not affected: csb records its clock on tmpfs and resumes it.

## Requirements on the CARLA server

- **Never `-quality-level=Low`.** At Low, CARLA 0.9.16 segfaults on `load_world` to another
  town and hangs on `reload_world` (UE 4.26 landscape render task reads a texture the level
  swap freed; carla-simulator/carla#4940). acb's `third_party/carla/run.sh` defaults to Epic.
- **Read snapshots without waiting for a tick.** In synchronous mode nothing ticks unless csb
  does; a snapshot call that waits for the next tick blocks until its timeout.
