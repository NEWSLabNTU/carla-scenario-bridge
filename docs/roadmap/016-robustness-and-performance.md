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

## Steps

### 1. Docs
- [x] Design doc `failure-and-frame-budget.md`; this phase doc; 005 boxes point here

### 2. Robustness (csb)
- [ ] Dispatch under `catch_unwind`; failure response with the panic message; session
      poisoned until the next `Initialize`; unit test with a panicking handler
- [ ] `Initialize` waits for CARLA: reconnect every 5 s up to `carla.reconnect_wait_seconds`
      (default 240)
- [ ] CARLA restart is an episode change: `epoch += E_last + Δ_last` from the last frame csb
      observed; entity mappings dropped on reconnect
- [ ] Entity actors carry `role_name = csb_entity:<name>`; `Initialize` reaps unowned ones
- [ ] Remove the stale "Deliberately NOT enabling synchronous mode" comment and the unused
      `has_ego` path left from 015 step 7

### 3. Robustness (around csb)
- [ ] `just run` supervises csb: restart on abnormal exit, backoff, give up after 5 in 300 s
- [ ] SSv2 fork: `ZMQ_RCVTIMEO` on the `simulation_interface` client (default 300 s), socket
      recreated after a timeout, error names csb

### 4. Performance (csb)
- [ ] `tracing` spans on all 14 handlers, carrying the frame number
- [ ] Per-step timing: processing, tick, `UpdateEntityStatus`, entity count; summary every
      200 frames and at scenario end; per-frame at DEBUG
- [ ] Rate-limited WARN: processing > 25 ms, tick > 100 ms
- [ ] Benchmark scenario generator; NPC-only Town01 scenarios for 10, 20, 50 vehicles

### 5. Live verification
- [ ] Kill CARLA mid-scenario: the scenario fails within bounded time with a CARLA message;
      restart CARLA; the next scenario passes with the same csb and ego stack; `/clock` and
      csb's reported time never decrease
- [ ] Kill csb (SIGKILL) mid-scenario: the scenario errors within the receive timeout; the
      supervisor restarts csb; no NPC from the dead run remains; the next scenario passes
- [ ] Benchmarks 10 / 20 / 50: record p50/p95/max processing and tick; 20 NPCs < 10 ms

## Acceptance

- After a CARLA crash and a csb crash, each, the next scenario passes with no process
  restarted by hand (CARLA itself excepted)
- No failure leaves a scenario running past its bound
- Reported simulation time and `/clock` never decrease across a CARLA restart
- 20 NPCs at 20 Hz: csb processing p95 < 10 ms, excluding the tick
- Scaling limit at 50 NPCs recorded
