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

- [x] `Coordinator` tracks every actor it spawns, including ones whose `EntityManager` entry
      is later removed or cleared
- [x] Add `Coordinator::destroy_all_spawned()`; call it from the `main.rs` shutdown path
      alongside `restore_async_mode`
- [x] `initialize()` destroys the previous run's actors before clearing `EntityManager`
- [x] Teardown is best-effort and never panics — CARLA may already be gone
- [x] Teardown logs a count: spawned N, destroyed N, failed N
- [x] Confirm `acb_bridge` still destroys only its own sensors and never a vehicle
      (invariant 2)

Tracking lives in a `spawned_actors: HashSet<u32>` deliberately separate from
`EntityManager`. `EntityManager` is a name lookup that gets emptied on despawn and on
re-initialise, and tying teardown to it is precisely what leaked every actor whose name
mapping was dropped first. `Coordinator::shutdown()` bundles teardown, traffic-light
restoration and async-mode restoration behind one call from `ZmqServer::cleanup`.

**Invariant 2 check found a latent violation.** `carla_vehicle.rs::cleanup` is correct — it
destroys sensors only. But `VehicleBridge::drop` in the dead `vehicle_bridge` module calls
`self.actor.destroy()` on the **vehicle**, and being in `Drop` it would fire implicitly on
scope exit. The module is `#![allow(dead_code)]` and unreferenced, so it is inert today.
Tracked in [011](011-robustness.md) with the other `vehicle_bridge` hazards.

### Spawn robustness (B3, C2)

- [x] ~~Spawn failure retries against alternative points~~ retries at increasing **height**,
      same x/y, before failing the request
- [x] Spawn failure describes *why* — blueprint, commanded pose, attempt count, last CARLA
      error, and the likely cause
- [x] Replace the flat `z < 0.5 → 0.5` clamp with a ground-height query, so sloped maps do
      not bury or float the vehicle
- [x] Log the resolved spawn pose at `info` for post-mortem

**Deviation from the written item, deliberately.** Retrying at *alternative spawn points*
would place a vehicle somewhere SSv2 did not ask for, which is the silent-divergence class
[006](006-honest-failures.md) exists to remove — the commanded x/y is SSv2's to decide
(invariant 5). Only the height varies, which is the standard CARLA workaround for "Spawn
failed because of collision at spawn position" and self-corrects as the vehicle settles. If
every height fails, the request fails with the reason.

Ground height comes from the nearest driving-lane waypoint rather than a raw ground
projection: `Map::waypoint_at` gives road height without the FFI-type juggling, and only its
`z` is used so the commanded x/y is untouched.

### Traffic light restoration (B4)

- [x] Shutdown unfreezes any traffic light this bridge froze
- [x] No-op when nothing was frozen, mirroring `restore_async_mode`'s guard

Nothing freezes yet — that is [009](009-map-and-traffic-lights.md). The `froze_traffic_lights`
flag and guarded `restore_traffic_lights()` are in place now so that landing the freeze cannot
leave a shared CARLA server stuck on the last scenario's states.

### CARLA reconnection (B5)

- [x] csb reconnects to CARLA on connection loss, as `acb_bridge` already does
- [x] State after reconnect is explicit
- [x] Document what SSv2 sees during a reconnect

`Coordinator` now holds the `Client` so it can rebuild the `World`. Repeated `world.tick()`
failures are the trigger: SSv2 drives frames continuously, so consecutive failures there mean
CARLA is gone rather than merely idle. After a reconnect:

- **Synchronous mode does not survive.** A restarted server defaults to async and a surviving
  one is no longer known to hold our settings, so the flag is cleared and the next frame
  re-applies it (invariant 1).
- **Entity mappings survive but may be stale.** Actor IDs from a restarted server are
  meaningless; teardown treats a missing actor as already destroyed, so a stale ID costs a
  warning, not a failure.
- **SSv2 sees failures** for every request made during the outage. It decides whether that
  ends the scenario; the bridge does not pretend the frames succeeded.

### Tests

- [x] Unit test: spawn retries only ever raise the height
- [x] Unit test: ground-relative and fallback spawn heights
- [x] Unit test: an empty teardown reports no work
- [ ] Unit test: teardown bookkeeping destroys entities dropped from `EntityManager`
- [ ] Unit test: unfreeze is skipped when nothing was frozen
- [x] Integration test (CARLA, no Autoware): spawn, shut down, verify zero leftover actors
      — probe, re-verified 2026-08-08
- [x] Integration test: two consecutive `Initialize` cycles in one process leave no leak
      — probe, re-verified 2026-08-08

The two unchecked unit tests need a `Coordinator`, which cannot be built without a live CARLA
`World` — the pure helpers were extracted and tested instead. Making `Coordinator` testable
without CARLA means introducing a world trait; worth doing, but it is a refactor in its own
right rather than something to smuggle into this phase.

## Acceptance Criteria

- [ ] Running the same scenario twice in a row succeeds both times, no CARLA restart
- [x] After a clean shutdown, `world.actors()` contains nothing this bridge spawned
- [x] After a Ctrl-C mid-scenario, the same holds
- [ ] CARLA restarted mid-run: csb recovers rather than erroring until killed — **blocked
      upstream**, see below
- [x] A second `Initialize` in one process does not leak the first run's actors
- [x] `just test` passes

## Verified 2026-08-08: Ctrl-C teardown; CARLA-loss recovery blocked upstream

**Ctrl-C mid-scenario is clean.** With an ego and an NPC spawned and sync mode on, SIGINT
produced `Teardown: 2 spawned, 2 destroyed, 0 failed`, unfroze all 36 Town01 lights,
restored async mode, and a fresh CARLA client then saw zero vehicles and walkers.

**CARLA restarted mid-run: the reconnect logic works, the process does not always survive
to use it.** Killing CARLA under a running scenario produced honest per-frame failures, and
in one run the bridge logged `Reconnected to CARLA; synchronous mode will be re-applied
next frame` and resumed ticking — the designed behaviour, observed live. But in two other
runs the process died first: carla-rust (rev `575f6aa`) let a
`carla::client::TimeoutException` escape uncaught — `terminate called after throwing` —
from the client's worker thread while the connection was down. Nothing on the Rust side of
the FFI can catch an exception on a thread the binding library owns; this is an upstream
carla-rust bug (jerry73204/carla-rust — in-house). Related: `load_world` on an unknown town
segfaulted the same way; the bridge now validates the town against `avaiable_maps()` first
and widens the RPC timeout to 120 s for the load itself (a first Town01 load took 66 s
under rendering, past the default 30 s).

Until carla-rust contains worker-thread exceptions, this criterion stays open and a CARLA
restart mid-run may kill the bridge outright rather than being ridden out.


## Verified against live CARLA (2026-07-30)

`scripts/integration/ssv2_probe.py` drives the bridge over the real protocol and asserts
against the CARLA world. See that script's README.

`Teardown: 3 spawned, 3 destroyed, 0 failed`, and a fresh world handle then reports 0
vehicles, 0 walkers, 0 props. A second `Initialize` cleans up the previous run.

Running the probe twice without restarting CARLA also exposed a real bug: `initialize()`
recorded async mode without applying it, so a second run inherited a synchronous CARLA and
resurrected the gap 1 deadlock. Fixed by `force_async_mode`.
