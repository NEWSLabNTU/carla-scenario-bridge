# Phase 023: NPC Motion Parity

Third fix of the [gap campaign](../design/gap-campaign.md): an NPC moves through CARLA as a
body with a velocity, not as a sequence of teleports at 0 m/s, while staying on the poses
SSv2 commands within a stated budget. Gated by a spike: if the budget cannot be held, the
phase ends with that measurement and NPCs stay kinematic.

**Source**: user direction 2026-10-11 ("fix scenario spec and Carla gap first").
**Depends on**: [014](014-feature-completeness.md) gap 5 (the measurement that opened this),
[022](022-entity-geometry-parity.md) (the right body before its motion).
**Feeds**: [024](024-gap-trace-and-oracle.md) (G3 pose divergence measures this phase's
residual).

## Problem

- **CARLA says every NPC is standing still.** NPCs are physics-off actors moved by
  `set_transform` each frame (`coordinator.rs:3279`). CARLA ignores `set_target_velocity`
  and `WalkerControl` on a physics-off actor: `get_velocity()` reads 0.000 m/s (measured
  2026-09-27, 014 gap 5).
- **What reads that zero.** acb's ground-truth objects already estimate each NPC's twist
  from its pose history (acb `028d8a8`), so Autoware's object list is not the problem any
  more. What remains:
  - **contact**: a kinematic body has infinite mass and no velocity, so the impulse it gives
    the ego is not the impulse of a moving car; with a 021 failure now carrying the
    impulse, it should mean something;
  - **walkers**: no walk cycle without physics, so a camera sees a gliding statue (009's
    classification work needs the animation);
  - **sensors that measure velocity** (radar Doppler) report 0.
- **Height.** Spawn snaps to the ground (`resolve_spawn_height`), walkers are lifted per
  teleport (`walker_teleport_z`), vehicles are not: on a grade, a teleported vehicle's z is
  SSv2's, which can float or sink. A body under physics follows the road by itself.

## Design

Spike first; the design below is what the spike tests.

**Tracked physics.** NPC vehicles and walkers spawn with physics on. Each frame, from SSv2's
commanded pose `p*` (after `OriginOffset`) and twist `v*`:

- vehicles: `set_target_velocity(v* + k_p (p* - p))` in the plane, angular velocity likewise
  toward the commanded yaw; z left to physics;
- walkers: `WalkerControl` with direction and speed from the same law;
- **snap rule**: if the in-plane error exceeds 0.5 m or the yaw error 5°, teleport to `p*`
  and zero the error (spawn, SSv2 teleport actions, a body knocked off course);
- the ego's own contacts with a tracked NPC are physical both ways: the NPC can be pushed,
  which the snap rule bounds.

**Budget.** Tracking error after `OriginOffset`, in-plane: p95 ≤ 0.2 m, max ≤ 0.5 m (the
snap); yaw p95 ≤ 2° -- the same tolerances 024's G3 uses, so this phase's success is the
oracle's silence. Measured per frame from one world snapshot, never per-actor RPCs.

**Switch.** Config key `npc_motion: kinematic | tracked`, default stays `kinematic` until
the live step passes; then the default flips and `kinematic` remains for comparison and
fallback.

## Steps

1. [ ] **Docs.** This phase doc.
2. [ ] **Spike (gate).** A scratch Python client on Town01: one vehicle NPC tracking a
   recorded SSv2 trajectory (straight, curve, stop-and-go) and one walker crossing, under
   synchronous mode at the bridge's fixed delta. Report error p95/max, snap count,
   `get_velocity()` against `v*`, frame-time cost. **If the budget is missed with
   reasonable gains, stop here**: record the numbers in this doc and 014 gap 5, keep NPCs
   kinematic, close the phase.
3. [ ] **Control law, test-driven.** Pure function `(p*, v*, p, yaw*, yaw) → command | snap`
   with tests: on track, lagging, overshoot, snap thresholds, stopped target.
4. [ ] **Bridge wiring.** Physics on at spawn under `tracked`; the per-frame command from the
   snapshot; snap via the existing teleport path (its failure handling unchanged); walker
   control. Error statistics per episode in the log and in 021's report.
5. [ ] **Live.** The nine starter scenarios under `tracked`: all pass, error within budget,
   `frame_stats` p95 within noise of `kinematic` (20-NPC bench, 016).
   `town01_pedestrian`: the walker animates in the screencast view. `town01_rear_contact`:
   `max_impulse` now scales with closing speed (run at two NPC speeds).
6. [ ] **Default flip** to `tracked`; user guide and bridge config comment.

## Acceptance

- Either: the spike's numbers miss the budget and are recorded, NPCs stay kinematic, phase
  closed with that finding.
- Or: under `tracked`, CARLA reports NPC velocities within 10 % of SSv2's commanded speed,
  tracking error within budget on all starter scenarios, walkers animate, no frame-time
  regression, and `tracked` is the default.
