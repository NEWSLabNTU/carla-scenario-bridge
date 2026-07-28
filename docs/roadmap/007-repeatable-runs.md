# Phase 007: Repeatable Runs

Make the second scenario run behave like the first.

**Source**: workflow audit 2026-07-28, findings B1-B5 and C2.
**Blocks**: every phase that needs more than one test run — which is all of them.

## Problem

Nothing destroys the actors this bridge spawns. `main.rs` cleanup restores async mode and
stops:

```rust
// Cleanup: restore CARLA async mode
server.cleanup();
```

`Coordinator::restore_async_mode()` is the whole of it. Ego, NPCs and any future background
AV stay in the CARLA world after the bridge exits.

`EntityManager::clear()` on `Initialize` makes it worse: it empties the name→actor map
*without destroying the actors*, so a second scenario in the same process orphans the first
run's actors and simultaneously loses the handles needed to clean them up.

These compound with the third problem — spawn has no retry and no occupied-point handling:

```rust
let actor = match builder.spawn(carla_transform) {
    Ok(a) => a,
    Err(e) => { return ... proto_err(format!("spawn failed: {e}")) }
};
```

A leaked ego sitting on the spawn point fails the next run's spawn, which aborts the scenario.
**Run once, then every subsequent run fails until CARLA is restarted.** That is the current
state of the workflow.

## Work Items

### Actor ownership and teardown (B1, B2)

- [ ] `Coordinator` tracks every actor it spawns, including ones whose `EntityManager` entry
      is later removed or cleared
- [ ] Add `Coordinator::destroy_all_spawned()`; call it from the `main.rs` shutdown path
      alongside `restore_async_mode`
- [ ] `initialize()` destroys the previous run's actors before clearing `EntityManager`
- [ ] Teardown is best-effort and never panics — CARLA may already be gone
- [ ] Teardown logs a count: spawned N, destroyed N, failed N
- [ ] Confirm `acb_bridge` still destroys only its own sensors and never a vehicle
      (invariant 2)

### Spawn robustness (B3, C2)

- [ ] Spawn failure retries against alternative points before failing the request
- [ ] Spawn failure describes *why* — occupied point, unknown blueprint, out of bounds
- [ ] Replace the flat `z < 0.5 → 0.5` clamp with a ground-height query, so sloped maps do
      not bury or float the vehicle
- [ ] Log the resolved spawn pose at `info` for post-mortem

### Traffic light restoration (B4)

- [ ] Shutdown unfreezes any traffic light this bridge froze
- [ ] No-op when nothing was frozen, mirroring `restore_async_mode`'s guard

### CARLA reconnection (B5)

- [ ] csb reconnects to CARLA on connection loss, as `acb_bridge` already does
- [ ] State after reconnect is explicit: which of sync mode, entities and frozen lights
      survive, and which are re-established
- [ ] Document what SSv2 sees during a reconnect — it will keep sending frames

### Tests

- [ ] Unit test: teardown bookkeeping destroys entities dropped from `EntityManager`
- [ ] Unit test: unfreeze is skipped when nothing was frozen
- [ ] Integration test (CARLA, no Autoware): spawn, shut down, verify zero leftover actors
- [ ] Integration test: two consecutive `Initialize` cycles in one process leave no leak

## Acceptance Criteria

- [ ] Running the same scenario twice in a row succeeds both times, no CARLA restart
- [ ] After a clean shutdown, `world.actors()` contains nothing this bridge spawned
- [ ] After a Ctrl-C mid-scenario, the same holds
- [ ] A second `Initialize` in one process does not leak the first run's actors
- [ ] CARLA restarted mid-run: csb recovers rather than erroring until killed
- [ ] `just test` passes
