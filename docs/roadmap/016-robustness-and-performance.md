# Phase 016: Robustness and Performance

Close 005's robustness and performance items: every single failure ends the running scenario
in bounded time and leaves the next one runnable without a manual restart, and csb's share of
a frame is measured and bounded.

**Source**: 005 audit (2026-10-06): "Adapter recovers from CARLA crash" was never verified
live -- every CARLA crash that week was followed by a bridge restarted by hand -- and no
performance item had been started. User direction 2026-10-07: "work on robustness and
performance".
**Design**: [failure-and-frame-budget](../design/failure-and-frame-budget.md).
**Depends on**: [015](015-carla-as-time-and-signal-source.md) (always-sync, episode epoch).
**Closes**: 005 "Graceful degradation", "Adapter recovers from CARLA crash", the five
Performance tasks and "20 NPC entities at 20Hz".

## Gaps found while designing

1. **SSv2 waits forever.** `MultiClient::call` blocks in `recv` with no timeout; a csb that
   dies mid-request leaves the scenario hung until killed.
2. **Nothing restarts csb.** It runs outside every launch file; a crash leaves no bridge.
3. **A panic ends csb.** No `catch_unwind` around dispatch.
4. **NPCs outlive a dead csb.** Only vehicles with a configured `role_name` are reaped.
5. **Initialize fails while CARLA restarts.** One reconnect attempt, then failure.
6. **Time after a CARLA restart** -- csb keeps its epoch and reports `elapsed` from 0 again:
   time runs backwards for SSv2 (time-and-ticking.md said so, as a limitation).
7. **No timing at all**: no spans, no frame timing, no benchmark.
8. **SSv2 ignores a failed frame** (found live, step 5). `SimulatorCore::update()`
   (`simulator_core.hpp:70`) discards `API::updateFrame()`'s `false`, and `updateFrame`
   returns before `clock_.update()`. A frame csb answers with a failure is retried forever
   and SSv2's time stands still, so not even the scenario's `SimulationTimeCondition`
   timeout fires. The receive timeout covers only a request nobody answers.
9. **A killed CARLA times out, it does not refuse** (found live, step 5). LibCarla's RPC to
   a dead server returns "time-out of 30000ms", for `tick` and for a reconnect's
   `get world` alike, so each failed request costs 30 s (60 s with a reconnect attempt), not
   the fast failure the design's failure table assumed.

## Steps

### 1. Docs
- [x] Design doc `failure-and-frame-budget.md`; this phase doc; 005 boxes point here

### 2. Robustness (csb)
- [x] Dispatch under `catch_unwind`; failure response with the panic message; session
      poisoned until the next `Initialize`; unit test with a panicking handler
      (`guarded_dispatch`, 4 tests)
- [x] `Initialize` waits for CARLA: reconnect every 5 s up to `carla.reconnect_wait_seconds`
      (default 240)
- [x] CARLA restart is an episode change: `epoch += E_last + Δ_last` from the last frame csb
      observed; entity mappings dropped on reconnect. Detected by a changed episode id
      (`world.id()`); the ledger is forgotten, not torn down, since old ids may name a new
      server's actors. acb's reconnect rule changed to match: it used to anchor at the first
      frame it saw after reconnecting, which no other process can know
- [x] Entity actors carry `role_name = csb_entity:<name>`; `Initialize` reaps unowned ones
      (`reap_orphaned_actors`, formerly `reap_orphaned_vehicles`)
- [x] Remove the stale "Deliberately NOT enabling synchronous mode" comment. `has_ego` is
      not unused after all: it is the fallback that re-applies sync on the first frame when
      `Initialize` (or a reconnect) could not, so it stays

### 3. Robustness (around csb)
- [x] `just run` supervises csb: restart on abnormal exit, backoff, give up after 5 in 300 s
      (`CSB_SUPERVISE=0` for a single run)
- [x] SSv2 fork: `ZMQ_RCVTIMEO` on the `simulation_interface` client (default 420 s), socket
      recreated after a timeout, error names csb. Live 2026-10-07 with
      `SIMULATOR_RESPONSE_TIMEOUT=60`: junit `SimulationError ... No response from the
      simulator at tcp://localhost:5555 within 60 s` (step 5)
- [ ] SSv2 fork: end the scenario when `updateFrame()` fails (gap 8) -- throw, or count
      consecutive failures and throw; without it steps 5a/5b below cannot pass

### 4. Performance (csb)
- [x] `tracing` spans on all 14 handlers, carrying the frame number (csb's own count: SSv2's
      `UpdateFrame` carries none)
- [x] Per-step timing: processing, tick, `UpdateEntityStatus`, entity count; summary every
      200 frames and at scenario end; per-frame at DEBUG (`frame_stats.rs`, 7 tests)
- [x] Rate-limited WARN: processing > 25 ms, tick > 100 ms
- [x] Benchmark scenario generator; NPC-only Town01 scenarios for 10, 20, 50 vehicles
      (`scripts/gen_npc_benchmark.py`, `scenarios/bench/`). With `--carla-port` it keeps only
      starts on a CARLA driving-lane centre with 2.5 m clearance: about two thirds of the
      Town01 lanelet candidates were 2–7 m off CARLA's lanes, some on planters, and CARLA
      refused a spawn there

### 5. Live verification
- [ ] (5a) Kill CARLA mid-scenario: the scenario fails within bounded time with a CARLA message;
      restart CARLA; the next scenario passes with the same csb and ego stack; `/clock` and
      csb's reported time never decrease.
      2026-10-07, `town01_ego_drive`, CARLA SIGKILLed ~3 s into the drive (frame 464).
      **Bound: fails.** The scenario never ended; stopped by hand after 615 s. csb answered
      every `UpdateFrame` with `tick failed: ... time-out of 30000ms` (30 s each, 60 s once
      reconnects began after 3), SSv2 ignored each one (gap 8) at frame 464–475, its
      watchdog logging `main ... unresponsive`; the 420 s receive timeout never trips because
      every reply arrives within 60 s (gap 9).
      **Recovery: passes.** CARLA restarted headless at Epic (RPC up in ~90 s); the next
      `town01_ego_drive`, started while CARLA was still down (gate bypassed), with the same
      csb and ego stack: `Initialize` found the connection dead and waited ("CARLA reachable
      again after 35 s"), csb `CARLA was restarted ... epoch 0 -> 16686.302864088, sim_time
      continues at 16686.302864088`, acb `Episode change: epoch 0 -> 16686.302864088` --
      **identical epochs**, one 0.05 s step after the last `/clock` before the kill
      (16686.252864). acb logged the plain episode change ("the world was replaced"), not the
      "after a reconnect" variant. Scenario passed. `/clock` sampled in domain 1 from before
      the kill to the end of the session: 0 decreases (16686.252864 -> 16686.302964 across
      the restart, 17004.34 at the end)
- [ ] (5b) Kill csb (SIGKILL) mid-scenario: the scenario errors within the receive timeout; the
      supervisor restarts csb; no NPC from the dead run remains; the next scenario passes.
      2026-10-07, `bench/town01_npc_10`, `SIMULATOR_RESPONSE_TIMEOUT=60`, csb child killed
      15 s after `Initialize`. Supervisor: `exited with status 137 (crash 1 in 300 s);
      restarting in 2 s`, new csb listening 2 s later, every time.
      **Request in flight** (csb SIGSTOPped, then SIGKILLed): passes -- junit `SimulationError
      ... No response from the simulator ... within 60 s`, scenario ended 64.8 s after the
      kill; the next run's `Initialize`: `Found 10 actor(s) from a previous bridge process`,
      destroyed, scenario passed, 0 `csb_entity:` actors left (CARLA queried by id).
      **Between requests** (plain SIGKILL, twice): **fails.** SSv2's next request was queued
      in its REQ socket and delivered to the new csb on reconnect, so no timeout; the new csb
      answered `UpdateEntityStatus: unknown entities 'npc_00' ...` and SSv2 retried it at
      20 Hz (gap 8) until stopped by hand after 10 min (11,500 WARN lines). Reaping still
      worked: the next `Initialize` destroyed the dead run's 10 NPCs both times.
      A last `town01_ego_drive` on the twice-restarted csb, same ego stack, passed
- [x] Benchmarks 10 / 20 / 50: record p50/p95/max processing and tick; 20 NPCs < 10 ms.
      2026-10-07, dev-release build, 1201 frames each, all pass; ms, p50 / p95 / max:

      | NPCs | processing | tick | UpdateEntityStatus |
      |---|---|---|---|
      | 10 | 0.70 / 0.90 / 252.9 | 1.63 / 1.89 / 4.6 | 0.68 / 0.87 / 1.5 |
      | 20 | 1.36 / **1.79** / 609.5 | 2.50 / 2.99 / 31.7 | 1.33 / 1.76 / 6.7 |
      | 50 | 3.24 / 4.19 / 1670.3 | 4.22 / 5.06 / 15.7 | 3.21 / 4.16 / 9.4 |

      ~0.065 ms per NPC, nearly all in `UpdateEntityStatus`; linear to 50, so no scaling limit
      reached there. Each max is the single spawn step (~33 ms per NPC spawned; the first
      spawn on a fresh bridge once took 8.8 s), the only frame over 25 ms per run. Wall time
      was 2–10 min per 60 s scenario at load 20–25: SSv2, not csb, set the pace. The
      production bridge was a debug build; `just run` now builds and runs dev-release

## Acceptance

- After a CARLA crash and a csb crash, each, the next scenario passes with no process
  restarted by hand (CARLA itself excepted) -- **met** 2026-10-07 for csb and ego stack;
  the failed scenario itself had to be stopped by hand (next item)
- No failure leaves a scenario running past its bound -- **not met**: a CARLA kill, and a
  csb kill between requests, leave the scenario retrying forever (gap 8, SSv2 fork task in
  step 3). A csb kill with a request in flight ends in 60 s
- Reported simulation time and `/clock` never decrease across a CARLA restart -- **met**:
  csb and acb epochs identical, `/clock` 0 decreases
- 20 NPCs at 20 Hz: csb processing p95 < 10 ms, excluding the tick
- Scaling limit at 50 NPCs recorded
