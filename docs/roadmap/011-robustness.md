# Phase 011: Robustness

Failure handling, latent hazards, and dead code that will bite later.

**Source**: gaps 8 and 11 from
[multi-instance-architecture.md](../design/multi-instance-architecture.md), plus audit
findings E2 and E3.
**Relates to**: [005-hardening-awf-contribution.md](005-hardening-awf-contribution.md), which
covers upstream contribution and remains valid.

## Problem

`acb_bridge` treats a tick timeout as evidence that CARLA is gone:

```rust
let sec = match carla_tick(&world, loop_duration) {
    Err(e) => {
        consecutive_failures += 1;
        if consecutive_failures >= MAX_CONSECUTIVE_FAILURES { break true; }
```

`loop_duration` is 50 ms and `MAX_CONSECUTIVE_FAILURES` is 3, so 150 ms without a tick is
enough to declare disconnection. But SSv2 pauses between frames routinely — Autoware
initialisation alone runs to `initialize_duration`, 120 s by default. A bare timeout means
"no frame yet", which is normal; only a transport error means the connection is gone. This
has not caused an observed failure yet because the current single-ego workflow ticks steadily,
but [009](009-map-and-traffic-lights.md) adds per-frame work and makes it likelier.

Two latent hazards sit in `acb_bridge`. `autoware.rs` creates publishers for
`vehicle/status/{steering,gear,control_mode,turn_indicators,hazard_lights}` and
`actuation_status` that are never published to — dead today, but they target exactly the
topics `vehicle_control.rs` owns. Wiring them up would put two publishers on one topic inside
a single process, the same class of bug as the `/clock` regression. Separately,
`publish_direct_localization=true` publishes `/tf` and `/localization/kinematic_state`,
competing with Autoware's EKF. It defaults off, so it is a footgun rather than a bug.

## Work Items

### Tick timeout is not a disconnect (gap 8)

- [x] Distinguish "no tick within the timeout" from a transport `CarlaError`
- [x] Only transport errors count toward the reconnect counter
- [x] Waiting logs at a decreasing rate so a 120 s Autoware startup is not a log flood
- [x] The timeout's default is justified in a comment; **configurability was declined** —
      recorded below as a scope change rather than silently ticked off

carla-rust turned out to model this precisely: `ConnectionError::Timeout` and
`ConnectionError::Disconnected` are separate variants, so `classify_tick_error` maps the
first to `TickOutcome::Idle` and everything else to `ConnectionLost`. Only the latter
increments the failure counter. Idle periods log after ~2 s and then roughly every 30 s.

The timeout is still the hardcoded 50 ms loop period, and it now carries its reasoning in
`main.rs` rather than only here (acb `66f6fd6`). A timeout used to decide whether CARLA was
gone, which made it worth tuning, and 150 ms was far too short against SSv2's 120 s
`initialize_duration`. Now that a timeout classifies as `TickOutcome::Idle` it decides
nothing: it only sets how promptly a new frame is noticed — half a poll of latency, 25 ms
against a 100 ms CARLA step — and how often the idle log ticks over.

**The configurability half of this item was dropped on purpose.** A knob here would let
someone tune a number that no longer affects robustness, which is how a parameter becomes a
trap.

### Ego respawn (gap 11)

- [x] Replace the `TODO` and "please restart the bridge" path in `acb_bridge/src/main.rs`
- [x] Re-discover the vehicle, re-attach sensors, rebuild bridges after Autoware reconnects
- [x] Sensors from the previous incarnation are destroyed first — no leak, no duplicates
- [x] Decide whether csb needs a matching path when SSv2 despawns and respawns the ego

The main loop's exit is now a `SessionExit` with three cases rather than a bool: `Shutdown`,
`CarlaLost`, and `AutowareRestarted`. The last re-enters the setup path without dropping the
CARLA connection — reconnecting would reset the clock epoch and rewind `/clock` under a
simulation that never paused. Sensor teardown already happened on the Autoware-lost branch,
so the rebuild starts clean.

**csb needs no matching path.** It has no per-ego state to rebuild: `has_ego` gates sync
mode, `EntityManager` is keyed by name, and a respawned ego arrives as an ordinary
`SpawnVehicleEntity`. Teardown bookkeeping already tolerates an actor that vanished.

### Duplicate-publisher hazards (E2, E3)

- [x] Remove the unused vehicle-status publishers from `autoware.rs`
- [x] **`VehicleBridge::drop` destroys the vehicle actor** — removed
- [x] Document `publish_direct_localization` as mutually exclusive with Autoware's own
      localization, and warn at startup when it is enabled
- [x] Audit for any other topic with two potential publishers in one domain (invariant 4)

The six publishers targeted `vehicle/status/{actuation,steering,gear,control_mode,
turn_indicators,hazard_lights}` and were never published to — but they still advertised those
topics, and they are exactly the topics `vehicle_control.rs` owns. Wiring them up would have
put two publishers on one topic inside a single process. Removed rather than left as a trap.

`VehicleBridge::drop` called `self.actor.destroy()` on the **vehicle**. From a `Drop` that
fires implicitly on any scope exit, so activating the module would have destroyed a vehicle
the scenario still believed it owned, mid-run. The call is gone; the impl now only logs.

Audit result: after these removals, no topic has two potential publishers within one domain.
`/clock` is parameterised (phase 006), vehicle status is `vehicle_control.rs` alone, and
`/tf` + `/localization/kinematic_state` are only touched under `publish_direct_localization`,
which now warns loudly at startup that it competes with Autoware's EKF.

### Error paths

- [x] Review every `let _ =` and swallowed `Result` in both bridges against the acb coding
      practice on silencing errors
- [x] Failures that abort a scenario carry a description naming the cause

Two silenced results remained, both genuine tick waits after a spawn. Neither is a
correctness requirement — the actor exists either way, and the wait is an optimisation — so
both keep ignoring the failure but now say why, per the acb practice of documenting rather
than hiding. Scenario-aborting failures gained their descriptions in phase 006.

### Tests

- [x] Unit: timeout classification — timeout does not increment the disconnect counter, a
      transport error does
- [x] Integration: a 120 s idle period with no frames does not trigger a reconnect —
      live, 2026-08-20: CARLA forced into synchronous mode with nobody ticking it, so the
      bridge genuinely received no frames for 150 s. Zero reconnect, disconnect or
      connection-lost lines. The only output was five INFO lines, at 2 s, 30 s, 60 s, 90 s
      and 120 s — "No CARLA frame for 120s; the simulation is paused" — which is the
      decreasing-rate idle log doing exactly what it was built for.
- [x] Integration: ego destroyed and respawned mid-run recovers without a bridge restart
      (live, 2026-08-10: despawn → sensor release → re-attach in under a second, repeatedly)

## Acceptance Criteria

- [x] A scenario with a long Autoware startup never logs a spurious CARLA reconnect —
      live, 2026-08-20, same run: `acb_bridge` reached its main loop while the ego stack was
      still bringing Autoware up, and stayed there through a startup running well past ten
      minutes without a single reconnect line.
- [x] Genuine CARLA loss is still detected and recovered from (acb side verified live
      through ten server OOM-kills on 2026-08-10; the csb bridge process can still wedge
      after a crash — fresh `Client::world()` times out in-process — restart the bridge,
      it is stateless)
- [x] Ego respawn works without restarting `acb_bridge` — acb `aeb2031`: per-tick
      `is_alive` despawn detection (`SessionExit::VehicleLost`), world re-fetch per wait
      poll (map reload = new episode), 60 s fruitless-wait reconnect (silently replaced
      server). Verified across CARLA server crashes and map reloads.
- [x] No topic has two publishers in one ROS domain, dead code included
- [x] Enabling `publish_direct_localization` warns about the conflict it creates
- [x] `just test` passes

All acceptance criteria are now met. The idle classification was unit-tested first and has
since been exercised live: 150 s of genuine no-frame idle produced no reconnect, and a long
Autoware startup produced none either.

One caveat worth keeping, because it changes how this is retested. Reaching a genuine
no-frame state now takes deliberate setup: csb hands CARLA back to asynchronous mode when a
session goes idle (the sync-mode watchdog, csb `d8a92d7`), and with CARLA free-running the
bridge is never starved of frames. The test therefore forces synchronous mode by hand and
ticks nothing. That is still the state the bridge meets during a real SSv2 pause — it is
just no longer the state a crashed scenario leaves behind.

Also fixed on the way through: `acb_bridge` did not compile at all, having read a
`WheelPhysicsControl::position` field that the pinned carla-rust calls `offset` (acb
`66f6fd6`). Nothing in this phase could have been verified live until that was repaired.
