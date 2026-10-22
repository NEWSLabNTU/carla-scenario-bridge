# Failure Handling and the Frame Budget

What swb does when CARLA, SSv2 or swb itself fails, and how much of a frame swb may spend.
Authoritative for both; history and measurements in roadmap
[016](../roadmap/016-robustness-and-performance.md). Time across failures follows
[time-and-ticking](time-and-ticking.md).

## Principles

1. **A failed scenario fails; it never hangs.** Every failure ends the running scenario with
   a message that names the cause, in bounded time. SSv2 decides what a failure means for the
   scenario; swb never reports a frame it did not simulate as a success.
2. **The next scenario runs without a hand on the keyboard.** After any single failure --
   CARLA crash, CARLA hang, swb crash, SSv2 crash -- the next `Initialize` succeeds once CARLA
   serves RPC again, with no process restarted by hand.
3. **Time never runs backwards**, across any of the above.

## Failure model

| Failure | Detected by | swb does | The running scenario | The next one |
|---|---|---|---|---|
| CARLA crashes or hangs | RPC **timeout** (client timeout 30 s; a killed server times out, it does not refuse -- measured 016) | the request fails with its cause. Teleports never wait (LibCarla's `SetActorTransform` is fire-and-forget), so the first frame's `tick` is what times out; the first call that fails slowly (> 5 s) marks CARLA unreachable and later RPCs are skipped | the SSv2 fork ends it on the first failed frame (~30–60 s) | `Initialize` waits for CARLA (bounded), reconnects, reaps, runs |
| A CARLA call never returns (the 0.9.16 Traffic Manager handshake, 034) | the watchdog: every tick, settings change, map load and TM `ShutDown`/`SetSynchronousMode` runs on one helper thread with a bound (below) | replies the request's error naming the call and step, then **drains**: every later request gets the same error at once, nothing goes to CARLA; idle ~2 s (10 s at most) → socket closed with a linger, `_exit(75)` | an error verdict within ~55 s; its deactivate despawn answered, no 90 s wait | `respawn` starts a fresh swb; its `Initialize` enables sync, reaps the dead one's actors by census |
| swb panics in a handler | `catch_unwind` around dispatch | replies failure with the panic message; refuses everything but `Initialize` until the next one | fails on that request | `Initialize` resets the session |
| swb dies (segfault in LibCarla, abort, OOM, kill) | process exit | the supervisor (`just run`) restarts it with backoff | a request in flight: SSv2's receive times out → error; a request queued after the death: the new swb refuses it (unknown entities) → failed frame → error | new swb serves it; reaps what the dead one left |
| SSv2 dies mid-scenario | nothing (REP socket just idles) | nothing: CARLA stays paused with the scenario's actors | -- | `Initialize` destroys them (session reset + reap) |
| Malformed request | protobuf decode error | replies failure for the peeked variant (phase 006) | fails | unaffected |

### CARLA crash and restart

The reconnect path exists (`note_carla_failure` / `reconnect_carla`); this phase makes the
whole cycle hold:

- **Initialize waits for CARLA.** A restarted server needs ~3 min before it serves RPC (182 s
  headless at Epic). `Initialize` probes the connection; if it is dead it retries the
  reconnect every 5 s up to `carla.reconnect_wait_seconds` (default 120) and only then
  fails. A scenario started while CARLA is still coming up waits instead of failing.
- **Time continues.** A restarted CARLA starts `elapsed_seconds` at 0. swb treats the restart
  as an episode change whose last old frame is the last frame swb observed:
  `epoch += E_last + Δ_last` (`EpisodeClock::begin_episode`), so the first new frame is
  reported one tick after the last old one. acb applies the same rule to the last frame it
  received; both saw the frame swb last ticked, so their epochs agree unless acb missed it.
- **A restarted server holds none of swb's state**: not sync mode, not frozen lights, not the
  map. The next `Initialize` re-applies sync and loads the scenario's town as usual; entity
  mappings from before the crash are dropped (their actor IDs mean nothing to the new server).

- **One timeout, not one per call.** A call that fails after more than 5 s failed by timing
  out, so CARLA is gone or hung. swb then marks CARLA unreachable until the next successful
  reconnect: teleports, despawns and teardown skip their RPCs (the collision sensor is
  abandoned, actors stay in the ledger), so ending a scenario against a dead server costs
  one timeout instead of 30 s per actor and sensor (measured 186 s for an ego with 5
  sensors). Nothing leaks: the ledger is torn down after the reconnect if the server
  survived, forgotten if it restarted, and the reaper catches marked actors either way.

- **A CARLA restart between scenarios ends the bridge (2026-10-16).** A LibCarla client must
  not outlive its server. Its RPC connection never reconnects -- every call then times out --
  but its streaming client does, to the next server on the same port, and the world-tick
  stream's token is the same on every server, so the old `on_tick` subscription resumes on
  the new server. On a frame flagged with a light update (any map load) the client's
  LightManager queries the lights from inside that stream callback over the dead RPC
  connection and blocks 30 s without reading the stream, and a synchronous 0.9.16 server
  busy-waits on its game thread in `ServerSession::Write` until every subscriber took the
  previous frame (its session deadline timer is cancelled by each write and never re-armed,
  so a stuck subscriber is never dropped). An idle swb sent nothing to CARLA, never noticed
  its server was replaced, and stalled every fresh server on port 2000 at its first
  synchronous map load (round 40's "port 2000 hang", reproduced at tick 94-498). Between
  scenarios swb now probes its connection: after 5 s without a request, every 2 s, the
  server version with a 2 s timeout (CARLA's RPC threads answer it without the game thread,
  so a live but paused server answers in milliseconds). Two failures in a row and the bridge
  exits with status 75, like a hung tick, so `respawn` starts a fresh process whose client
  has no stale subscription; dropping the client in-process would not do, since every actor
  and world handle keeps LibCarla's episode and its subscription alive. acb probes its
  clients the same way (acb `conn_probe.rs`). Measured: before, every CARLA restart with the
  bridges attached hung the next synchronous run within 500 ticks; after, the bridge exits
  4 s after the server dies and three restart cycles ran 6,000-10,000 ticks each.

### swb crash and restart

- **Supervisor.** swb is a long-lived singleton outside any launch file, so nothing restarted
  it. `just run` now loops: a clean exit (status 0, or SIGINT/SIGTERM forwarded by the
  operator) ends the loop; any other exit restarts swb after a backoff
  (`2 s × 2^(n−1)`, capped at 60 s), and more than 5 crashes in 300 s ends the loop with an
  error -- the same policy play_launch uses for composables.
- **SSv2 must not retry a failed frame.** Upstream `SimulatorCore::update()` discards
  `updateFrame()`'s result, so a failed frame was simply sent again: time stops advancing,
  so not even a `SimulationTimeCondition` timeout fires, and a scenario whose simulator lost
  its world ran forever at one frame per 30–60 s (measured: 615 s, then stopped by hand). The
  fork throws a `SimulationError` on the first failed frame. This is also phase 006's
  promise kept end to end: swb has reported honest failures since then, and SSv2 was
  dropping them.
- **SSv2 must not wait forever.** SSv2's ZMQ `REQ` client blocks in `recv` with no timeout,
  and a request sent to a swb that died is never answered by the next one, so the scenario
  hangs until killed by hand. The fork sets a receive timeout
  (`simulation_interface` `ZMQ_RCVTIMEO`, `SIMULATOR_RESPONSE_TIMEOUT`, default 420 s at first, 90 s since 2026-10-10 with `Initialize` under its own `SIMULATOR_INITIALIZE_TIMEOUT` 300 s; swb's watchdog (032 N5, 034: 45 s per tick or settings change, 55 s with the state read) answers inside it; see "Timeouts" below. Originally chosen above the worst legitimate request:
  `Initialize` with a map load and the CARLA wait) and throws on expiry, which the
  interpreter reports as a simulation error. The socket is recreated after a timeout, since a
  `REQ` socket that missed its reply cannot send again.
- **Keep the epoch.** The episode epoch lived only in swb's memory, so a restarted swb
  reported CARLA's raw elapsed time: SSv2's time went backwards (16825 s → 283 s, measured)
  and stayed an epoch apart from acb's `/clock`. swb now records `(CARLA episode id, epoch,
  last frame)` after every frame in `$XDG_RUNTIME_DIR/scenario-world-bridge/` (tmpfs: the
  log disk can stall a writer, see CLAUDE.md), keyed by CARLA host and port. A new swb
  restores it when the episode id matches; when it does not (CARLA also restarted while
  swb was down), it applies the episode rule to the recorded last frame, as acb did.
- **Reap what the dead swb left.** `destroy_all_spawned` only knows this process's ledger.
  Vehicles with a configured `role_name` are already reaped at `Initialize`
  (`reap_orphaned_vehicles`). NPCs, walkers and props carried no mark, so a swb that died
  mid-scenario left them in the world. Every actor swb spawns for an SSv2 entity now carries
  `role_name = swb_entity:<name>` (where the blueprint has the attribute), and `Initialize`
  destroys any `swb_entity:` actor it does not own. Actors without the mark are somebody
  else's and are left alone.

### Panics

A Rust panic in a handler used to unwind out of `main` and end the process, which with no
supervisor meant no bridge. Dispatch runs under `catch_unwind`: the request gets a failure
response (`"swb internal error: <message>"`), the panic is logged at ERROR with its location,
and the session is marked poisoned -- every request but `Initialize` is refused until the next
`Initialize` resets it, because a handler that died half-way may have left the entity map and
CARLA disagreeing. A segfault inside LibCarla cannot be caught; the supervisor covers it.

### Timeouts

Every bound swb puts on CARLA, nested inside SSv2's (`budgets.rs`, whose tests check the
arithmetic; roadmap 034 step 3):

| Bound | Value | Why |
|---|---|---|
| SSv2 response (`SIMULATOR_RESPONSE_TIMEOUT`) | 90 s | every request but `Initialize` |
| SSv2 `Initialize` (`SIMULATOR_INITIALIZE_TIMEOUT`) | 300 s | |
| CARLA client RPC | 30 s (120 s around the map listing and `load_world`) | |
| LibCarla frame wait in `Tick` (`LIBCARLA_TICK_TIMEOUT`) | 10 s | 5x the worst tick measured (1.90 s); also each tick inside `apply_settings` |
| Tick watchdog (`TICK_WATCHDOG`) | 45 s | above `tick_cue` 30 + frame 10, so LibCarla's own timeouts stay ordinary errors |
| `apply_settings` watchdog | 45 s | `set_episode_settings` 30 + 1 s + its ticks |
| `load_world` watchdog | 150 s | above the 120 s RPC |
| TM `ShutDown` / `SetSynchronousMode` watchdog | 45 s | the same handshake as the tick |
| State read after a hung call | 10 s | 45 + 10 = 55 s < 90 s |
| One spawn request | deadline 75 s | attempts get min(30 s, left − 30 s); the settling tick is capped by what is left |
| One `Initialize` | deadline 280 s | the watched steps are capped by what is left; a step past it fails `Initialize` naming it |
| Drain after a hang | idle 2 s, at most 10 s | then the socket closes with a 1 s linger |

A test-only fault hook makes the hang reproducible on a live stack:
`SWB_TEST_HANG_TICK_AT=<SSv2 frame>` blocks that frame's first tick forever, and with
`SWB_TEST_HANG_TICK_TOKEN=<file>` only while the file exists (removed when it fires, so the
respawned bridge runs normally). `SWB_TEST_SPAWN_TIMEOUT_MS=<ms>` gives every spawn RPC that
timeout, forcing the timed-out spawn that is then adopted. Both are off unless set, and the
bridge logs a warning at start-up when one is.

## Frame budget

At 20 Hz a frame has 50 ms of wall time if the run is to keep real time. swb's share is what
it spends outside CARLA's tick:

```
frame processing = Σ handler time for one SSv2 step  −  time inside world.tick()
```

One SSv2 step is every request from one `UpdateFrame` response to the next
(`UpdateEntityStatus`, `UpdateTrafficLights`, spawns, ..., `UpdateFrame`).

- **Target**: < 10 ms with 20 NPCs (acceptance); warn above 25 ms (half the budget).
- **Tick time** is CARLA's and reported separately; warn above 100 ms.

### Instrumentation

- **Spans.** Each of the 14 handlers runs in a `tracing` span named after the request
  (`update_entity_status`, `update_frame`, ...) carrying the SSv2 frame number, so every log
  line a handler emits says which request and frame produced it. Spans are not logged on
  enter/exit (that would be 100+ lines a second to the shared disk, see CLAUDE.md "The log
  disk can stall a node").
- **Per-frame timing.** swb measures every step's processing time, its tick time, the
  `UpdateEntityStatus` time and the entity count. It logs **one summary line every 200
  frames** (10 s at 20 Hz) at INFO -- p50, p95 and max of each, and how many frames exceeded
  25 ms -- and one at scenario end for the whole run. Per-frame values go to DEBUG only.
- **Warnings** are rate-limited: the first frame over 25 ms (or tick over 100 ms) in a
  summary window logs a WARN with its breakdown; the rest are counted in the summary.

### Benchmarks

NPC-only scenarios (no ego, so no Autoware needed) generated for N = 10, 20, 50 vehicles
driving puppeteered on Town01, 60 s each, at 20 Hz. The run's end summary is the measurement.
They exercise the per-entity path -- `set_transform` per NPC, state readback -- which is where
the cost grows with N.
