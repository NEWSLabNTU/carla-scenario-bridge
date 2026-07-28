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

- [ ] Distinguish "no tick within the timeout" from a transport `CarlaError`
- [ ] Only transport errors count toward the reconnect counter
- [ ] Waiting logs at a decreasing rate so a 120 s Autoware startup is not a log flood
- [ ] The timeout is configurable and its default justified in a comment

### Ego respawn (gap 11)

- [ ] Replace the `TODO` and "please restart the bridge" path in `acb_bridge/src/main.rs`
- [ ] Re-discover the vehicle, re-attach sensors, rebuild bridges after Autoware reconnects
- [ ] Sensors from the previous incarnation are destroyed first — no leak, no duplicates
- [ ] Decide whether csb needs a matching path when SSv2 despawns and respawns the ego

### Duplicate-publisher hazards (E2, E3)

- [ ] Remove the unused vehicle-status publishers from `autoware.rs`, or document precisely
      why they exist and how they avoid colliding with `vehicle_control.rs`
- [ ] **`VehicleBridge::drop` destroys the vehicle actor** (`vehicle_bridge.rs`), which
      violates invariant 2 — only `csb_bridge` may destroy vehicles. Being in `Drop`, it
      would fire implicitly on scope exit. Inert today because the module is
      `#![allow(dead_code)]` and unreferenced, but it is a trap for whoever activates it.
      Found by the invariant-2 check in [007](007-repeatable-runs.md).
- [ ] Document `publish_direct_localization` as mutually exclusive with Autoware's own
      localization, and warn at startup when it is enabled
- [ ] Audit for any other topic with two potential publishers in one domain (invariant 4)

### Error paths

- [ ] Review every `let _ =` and swallowed `Result` in both bridges against the acb coding
      practice on silencing errors
- [ ] Failures that abort a scenario carry a description naming the cause

### Tests

- [ ] Unit: timeout classification — timeout does not increment the disconnect counter, a
      transport error does
- [ ] Integration: a 120 s idle period with no frames does not trigger a reconnect
- [ ] Integration: ego destroyed and respawned mid-run recovers without a bridge restart

## Acceptance Criteria

- [ ] A scenario with a long Autoware startup never logs a spurious CARLA reconnect
- [ ] Genuine CARLA loss is still detected and recovered from
- [ ] Ego respawn works without restarting `acb_bridge`
- [ ] No topic has two publishers in one ROS domain, dead code included
- [ ] Enabling `publish_direct_localization` warns about the conflict it creates
- [ ] `just test` passes
