# Time and Ticking

How simulation time advances and is observed across CARLA, the scenario bridge (swb), the
Autoware bridge (acb) and SSv2. Authoritative for the clock and the tick; other design
docs point here. History and measurements: roadmap
[015](../roadmap/015-carla-as-time-and-signal-source.md).

## One source of time

Simulation time is CARLA's `WorldSnapshot.timestamp.elapsed_seconds` plus a constant, in
integer nanoseconds (roadmap 051):

```
sim_time_ns = nanos(elapsed_seconds) + BASE_NS     # BASE_NS = 1000 s
nanos(s)    = round-half-away-from-zero(s * 1e9)
```

`BASE_NS` and `nanos` are one definition per language -- `swb_core::sim_time` (Rust),
`swb_world_server.protocol` (Python), acb's `clock.rs`, aib's `sim_time.py` -- and
`scripts/test_sim_time.py` (in `just test`) checks that all four agree on the constant and on
a table of rounding cases. `BASE_NS` keeps ROS time away from 0 (ROS's "no clock yet") and
from the small values for which `now() - duration` goes negative in tf2 and Autoware's
monitors.

- **acb** publishes `/clock` from it in every ROS domain, and stamps every message it
  publishes (sensors, odometry, vehicle status, ground-truth objects, traffic signals) with
  the time of the CARLA frame the message was computed from. acb depends on CARLA and
  nothing else: no swb, no SSv2.
- **The world server** reports it in `LoadWorldReply.sim_time_ns` and `StepReply.sim_time`,
  and **swb** passes it to SSv2 as `simulation_time_ns` on every `InitializeResponse` and
  `UpdateFrameResponse`.
- **SSv2** (fork, `clock_source:=simulator`) takes its ROS time from that field and
  publishes no `/clock`.

Nothing learns an offset, and nothing is shared, stored or sent: every process computes the
same integer from the frame it holds, so a restarted swb, world server or vehicle bridge needs
no state to agree. Measured (roadmap 051, K4): with the world server and the bridge just
restarted under a running stack, all 235 distinct stamps of SSv2's V2I signal message equal a
`/clock` value acb published, bit for bit.

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

## Episodes and vehicle stacks

An **episode** is one simulator world: on CARLA, one world id, from its load until the next
load or the server's death. CARLA restarts `elapsed_seconds` at 0 in every episode, so time
is monotonic within an episode and only there. ROS has no general answer to a clock that goes
back -- tf2 clears its buffers, Autoware's EKF, NDT and planners keep their state -- so
**an episode change ends every vehicle stack**, and the stack comes back in the new episode.

- **The vehicle bridge ends its stack before the stack can see time go back.** acb attaches
  to the episode it first sees. When a frame or the hero belongs to another episode (a world
  load, or a reconnect that finds a restarted CARLA), it logs the change naming both world
  ids, flushes its log and exits with status 3, before publishing anything stamped in the
  new episode. `acb_bridge` is declared `on_exit="shutdown"` and never respawned, so its exit
  takes the whole Autoware stack with it. Any future simulator's bridge must do the same
  (part of the I5 contract, [vehicle-side](vehicle-side.md)).
- **The backstop is the profile's `clock_guard`**, a required node in every Autoware stack
  (`sim_clock_guard:=true`): it ends the stack the moment `/clock` steps back, whichever
  simulator published it.
- **The stack comes back** through roadmap 030's restart path. `scripts/carla_up.sh` starts the
  ego stack under `swb-agent supervise`, which starts it again (same map) when it ends; a stack
  on another map than the scenario's is stopped through its supervisor and replaced by one on
  the scenario's map. That one attaches to the old episode and ends once more at the
  scenario's load, so a town change costs two stack starts. `run_suite` waits for a
  supervised agent whose stack ended to register again before the next scenario.
- **The protocol** carries `episode_id` in `LoadWorldReply` and `StepReply`; the bridge logs
  when it changes. Loading the same world again keeps the episode (swb skips `load_world`
  when the town matches), and the conformance check holds a server to that.
- **A scenario never spans an episode change**: the load happens at `Initialize`, before
  the scenario's first frame. Between scenarios inside one episode nothing ticks (the pause).
- **acb stand-alone** (`just sim`) loads its town before it starts, so the demo scenario
  finds it loaded and acb attaches to the episode that stays.
- **Isaac, a deviation**: the Isaac world server's time is monotonic over its whole lifetime
  and aib's `/clock` lives in that process, so an Isaac stack is not ended by a world load;
  `isaac_up.sh` restarts a stack on another map, and `clock_guard` covers a restarted world
  server, whose time starts again. Nothing may depend on a server keeping time across
  episodes.

`load_world` keeps `reset_settings = false`: the new world stays synchronous, and loads stay
fast (2.3 s measured against 44 s and more with the default, which reverted the world to
asynchronous and let it free-run).

Before roadmap 051 a stack lived through a world load: an episode epoch carried time across
it, kept by two copies of one rule (swb's and acb's) that had to agree to the nanosecond, a
frame count around `load_world`, a clock file for swb restarts and a marker actor for bridges
joining late, with an accepted case where the two still drifted apart (roadmap 016, gap 12).
All of it is gone; [roadmap 015](../roadmap/015-carla-as-time-and-signal-source.md) and
[051](../roadmap/051-target-shape.md) have the history and the measurements. The cost is a
stack restart per town change and per CARLA restart.

## Requirements on the CARLA server

- **Never `-quality-level=Low`.** At Low, CARLA 0.9.16 segfaults on `load_world` to another
  town and hangs on `reload_world` (UE 4.26 landscape render task reads a texture the level
  swap freed; carla-simulator/carla#4940). acb's `third_party/carla/run.sh` defaults to Epic.
- **Read snapshots without waiting for a tick.** In synchronous mode nothing ticks unless swb
  does; a snapshot call that waits for the next tick blocks until its timeout.
