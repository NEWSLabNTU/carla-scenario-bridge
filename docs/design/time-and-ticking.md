# Time and Ticking

How simulation time advances and is observed across CARLA, the scenario bridge (swb), the
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
  nothing else: no swb, no SSv2.
- **swb** reports the same number to SSv2 as `simulation_time_ns` on every
  `InitializeResponse` and `UpdateFrameResponse`.
- **SSv2** (fork, `clock_source:=simulator`) takes its ROS time from that field and
  publishes no `/clock`.

Nothing learns an offset; every stamp is a CARLA frame time. The epoch exists only so that
time never runs backwards when CARLA starts a new episode (below).

## One ticker

CARLA runs **synchronous from swb's first `Initialize` on, and swb is the only process that
ticks it.** Nothing else ever advances the world:

| Phase | Ticks |
|---|---|
| Scenario running | swb, `N = round(step_time / carla_tick_seconds)` ticks per SSv2 frame |
| Initialize → ego spawn | the spawn's own tick (swb), nothing else |
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

**No idle watchdog.** swb used to hand CARLA back to async after 10 s without a request
(300 s with an ego) so a dead scenario would not leave the world frozen. That free-running
is what this design removes, and a ZMQ REP socket cannot tell a dead SSv2 from a long pause
anyway. A graceful swb shutdown still restores async; after a swb crash CARLA stays paused
until the next swb takes it (which runs it synchronous anyway). Any other CARLA client that
waits for ticks will wait while no scenario runs.

## Followers: the served Traffic Manager and the tick gate

A process that drives vehicles without ticking (2026-vehicle-consensus' junction harness,
which follows swb's frames) needs two things from the ticker.

**The Traffic Manager has to live here.** LibCarla steps a synchronous TM only from the
tick of the process that hosts it: `Simulator::Tick` → `SynchronizeFrame` →
`TrafficManager::Tick`, which calls `SynchronousTick` on the TMs in that process's
`_tm_map`; a TM-Client's `SynchronousTick` (`TrafficManagerRemote`) returns at once. A TM
hosted by a follower is never stepped by swb's tick and has to run asynchronously, on
whatever frames its own thread catches (the harness's cars swerved off their lanes after
a junction that way). With `traffic_manager.serve` (or `SWB_TM_SERVE=1`) swb starts its TM
with the synchronous world, seeded and hybrid physics off, and keeps it for its lifetime;
the follower connects to `traffic_manager.port` as a TM-Client and leaves the mode alone
(only the TM-Server is set synchronous). swb sets it asynchronous and shuts it down when it
shuts down. A TM-Client exists only for a port CARLA has registered: a client that comes
too early gets a bind error from LibCarla trying to start a server of its own, so it retries.

**The tick gate** (`tick_gate`, or `SWB_TICK_GATE=host:port`, see `tick_gate.rs`). A
follower whose round takes longer than a frame skips frames. A follower that joins the gate
(`HELLO`, then `DONE <frame>` after each frame, `BYE`) is waited for before every tick, at
most `tick_gate_timeout_ms` (1 s), after which the tick goes ahead and the miss is counted.
A follower silent for 5 s has left. The wait is in swb's request path before
`world.tick()`, never inside a callback, and is bounded well inside SSv2's reply timeout.

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
  `max_substep_delta_time × max_substeps ≥ fixed_delta_seconds`; swb wants ≤ 2 ms substeps
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

**swb**, the only process that loads maps, must arrive at the same frame without a channel
to acb. It cannot simply read a snapshot before loading: CARLA's `LoadEpisode` sends its own
tick cue while it waits, so the old episode advances after the snapshot (measured: exactly
one frame per load). swb instead counts:

```
before:  sync mode on; snapshot → (F0, E0, Δ)
load:    load_world(town, reset_settings = false)      # new world stays synchronous
after:   snapshot → (F1, E1)
n_new  = round(E1 / Δ)          # frames the new episode has had (it only advances by Δ)
k_old  = (F1 − n_new) − F0      # old-episode frames after swb's snapshot
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
swb (roadmap 016) from the last frame it ticked plus one tick. Both saw the frame swb last
ticked, so the epochs agree unless acb missed that frame. See
[failure-and-frame-budget](failure-and-frame-budget.md).

**A bridge that starts mid-session** has watched none of the earlier episode changes, and the
rule alone would start it at epoch 0: a vehicle side restarted after a reload published a
`/clock` 150 ms behind its peers for a whole session (2026-10-13). So the epoch is also kept
in the world it belongs to. After every episode change (map load, CARLA restart) and at
start-up, swb leaves one **epoch marker** in the world -- a `util.actor.empty` (no mesh,
collision or physics) with `role_name = epoch_ns:<epoch>` -- once it can see the world's
actors (it has ticked). `load_world` destroys it with the old world, so each episode carries
its own. acb, on its first frame of an episode it did not watch begin, withholds `/clock`
until it has read that episode's marker and adopts it; no marker means epoch 0, the
behaviour without swb. A bridge that watched the change keeps its own epoch and only warns
if the marker disagrees. Still CARLA and nothing else: no file, no channel between bridges.
swb writes no marker for epoch 0. See roadmap 015, "Joining mid-session (step 8)".

**Known limitation (accepted, roadmap 016 gap 12).** A restarted CARLA free-runs before
swb's first `Initialize`. If that server dies again before swb has read a frame from it,
acb has seen its frames and applies the episode rule to the last one, while swb has seen
none and keeps its epoch: from then on the two differ by a constant (265.86 s measured).
Both stay monotonic and scenarios pass; restarting the ego stack (acb) clears it. A swb
restart is not affected: swb records its clock on tmpfs and resumes it.

## Requirements on the CARLA server

- **Never `-quality-level=Low`.** At Low, CARLA 0.9.16 segfaults on `load_world` to another
  town and hangs on `reload_world` (UE 4.26 landscape render task reads a texture the level
  swap freed; carla-simulator/carla#4940). acb's `third_party/carla/run.sh` defaults to Epic.
- **Read snapshots without waiting for a tick.** In synchronous mode nothing ticks unless swb
  does; a snapshot call that waits for the next tick blocks until its timeout.
