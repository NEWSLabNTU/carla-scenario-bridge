# Failure Handling and the Frame Budget

What csb does when CARLA, SSv2 or csb itself fails, and how much of a frame csb may spend.
Authoritative for both; history and measurements in roadmap
[016](../roadmap/016-robustness-and-performance.md). Time across failures follows
[time-and-ticking](time-and-ticking.md).

## Principles

1. **A failed scenario fails; it never hangs.** Every failure ends the running scenario with
   a message that names the cause, in bounded time. SSv2 decides what a failure means for the
   scenario; csb never reports a frame it did not simulate as a success.
2. **The next scenario runs without a hand on the keyboard.** After any single failure --
   CARLA crash, CARLA hang, csb crash, SSv2 crash -- the next `Initialize` succeeds once CARLA
   serves RPC again, with no process restarted by hand.
3. **Time never runs backwards**, across any of the above.

## Failure model

| Failure | Detected by | csb does | The running scenario | The next one |
|---|---|---|---|---|
| CARLA crashes | RPC error (connection refused/reset, fast) | 3 consecutive failures → one reconnect attempt per request; every request fails with its cause meanwhile | fails on its next frame | `Initialize` waits for CARLA (bounded), reconnects, reaps, runs |
| CARLA hangs | RPC timeout (client timeout 30 s) | as above, but each failure costs the timeout | fails within ~3 timeouts | as above |
| csb panics in a handler | `catch_unwind` around dispatch | replies failure with the panic message; refuses everything but `Initialize` until the next one | fails on that request | `Initialize` resets the session |
| csb dies (segfault in LibCarla, abort, OOM, kill) | process exit | the supervisor (`just run`) restarts it with backoff | SSv2's receive times out → scenario errors | new csb serves it; reaps what the dead one left |
| SSv2 dies mid-scenario | nothing (REP socket just idles) | nothing: CARLA stays paused with the scenario's actors | -- | `Initialize` destroys them (session reset + reap) |
| Malformed request | protobuf decode error | replies failure for the peeked variant (phase 006) | fails | unaffected |

### CARLA crash and restart

The reconnect path exists (`note_carla_failure` / `reconnect_carla`); this phase makes the
whole cycle hold:

- **Initialize waits for CARLA.** A restarted server needs ~3 min before it serves RPC (182 s
  headless at Epic). `Initialize` probes the connection; if it is dead it retries the
  reconnect every 5 s up to `carla.reconnect_wait_seconds` (default 240) and only then
  fails. A scenario started while CARLA is still coming up waits instead of failing.
- **Time continues.** A restarted CARLA starts `elapsed_seconds` at 0. csb treats the restart
  as an episode change whose last old frame is the last frame csb observed:
  `epoch += E_last + Δ_last` (`EpisodeClock::begin_episode`), so the first new frame is
  reported one tick after the last old one. acb applies the same rule to the last frame it
  received; both saw the frame csb last ticked, so their epochs agree unless acb missed it.
- **A restarted server holds none of csb's state**: not sync mode, not frozen lights, not the
  map. The next `Initialize` re-applies sync and loads the scenario's town as usual; entity
  mappings from before the crash are dropped (their actor IDs mean nothing to the new server).

### csb crash and restart

- **Supervisor.** csb is a long-lived singleton outside any launch file, so nothing restarted
  it. `just run` now loops: a clean exit (status 0, or SIGINT/SIGTERM forwarded by the
  operator) ends the loop; any other exit restarts csb after a backoff
  (`2 s × 2^(n−1)`, capped at 60 s), and more than 5 crashes in 300 s ends the loop with an
  error -- the same policy play_launch uses for composables.
- **SSv2 must not wait forever.** SSv2's ZMQ `REQ` client blocks in `recv` with no timeout,
  and a request sent to a csb that died is never answered by the next one, so the scenario
  hangs until killed by hand. The fork sets a receive timeout
  (`simulation_interface` `ZMQ_RCVTIMEO`, default 300 s, above the worst legitimate request:
  `Initialize` with a map load and the CARLA wait) and throws on expiry, which the
  interpreter reports as a simulation error. The socket is recreated after a timeout, since a
  `REQ` socket that missed its reply cannot send again.
- **Reap what the dead csb left.** `destroy_all_spawned` only knows this process's ledger.
  Vehicles with a configured `role_name` are already reaped at `Initialize`
  (`reap_orphaned_vehicles`). NPCs, walkers and props carried no mark, so a csb that died
  mid-scenario left them in the world. Every actor csb spawns for an SSv2 entity now carries
  `role_name = csb_entity:<name>` (where the blueprint has the attribute), and `Initialize`
  destroys any `csb_entity:` actor it does not own. Actors without the mark are somebody
  else's and are left alone.

### Panics

A Rust panic in a handler used to unwind out of `main` and end the process, which with no
supervisor meant no bridge. Dispatch runs under `catch_unwind`: the request gets a failure
response (`"csb internal error: <message>"`), the panic is logged at ERROR with its location,
and the session is marked poisoned -- every request but `Initialize` is refused until the next
`Initialize` resets it, because a handler that died half-way may have left the entity map and
CARLA disagreeing. A segfault inside LibCarla cannot be caught; the supervisor covers it.

## Frame budget

At 20 Hz a frame has 50 ms of wall time if the run is to keep real time. csb's share is what
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
- **Per-frame timing.** csb measures every step's processing time, its tick time, the
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
