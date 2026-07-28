# Phase 006: Honest Failures

Stop the bridge reporting success for work it did not do.

**Source**: workflow audit 2026-07-28, findings A1-A3 and E1.
**Blocks**: everything. A scenario suite that scores fiction cannot validate any later phase.

## Problem

Three handlers return `Success=true` while doing nothing at all. Because SSv2 trusts the
result, the scenario keeps running and reports a verdict based on entities that do not exist.

`spawn_pedestrian_entity` is the clearest case. It returns success without spawning, so
nothing is recorded in `EntityManager`. SSv2 then sends `UpdateEntityStatus` for that
pedestrian every frame, and `update_entity_status` takes its unknown-entity branch:

```rust
None => {
    // Echo back unknown entities unchanged
    updated.push(api::UpdatedEntityStatus {
        name: name.clone(),
        action_status: entity_status.action_status.clone(),
        pose: entity_status.pose.clone(),
    });
    continue;
}
```

SSv2 receives exactly the pose it asked for, concludes the pedestrian is walking its scripted
path, and evaluates conditions against it. CARLA never had the actor. Autoware's sensors see
empty road. **A pedestrian-crossing scenario passes without a pedestrian.**

The `AttachSensor` handlers already show the correct shape — they return `Success=false` with
a description, and SSv2 handles that cleanly. These three should do the same until they are
genuinely implemented.

This phase does not implement the missing features. It makes their absence *loud*. The
implementations land in [008](008-entity-fidelity.md) and [009](009-map-and-traffic-lights.md).

## Work Items

### Honest rejection

- [ ] `spawn_pedestrian_entity` returns `Success=false` with a description naming phase 008
- [ ] `spawn_misc_object_entity` returns `Success=false`, same
- [ ] `update_traffic_lights` returns `Success=false` with a description naming phase 009
- [ ] Each logs at `warn` on first call, not per frame, so the log stays readable

### Unknown-entity handling

- [ ] `update_entity_status` logs a warning the first time it sees an entity absent from
      `EntityManager`, naming the entity — silent echo is what let A1 hide
- [ ] Decide and document whether echoing is the right fallback at all, or whether an
      unknown entity should fail the request

### Protocol correctness (E1)

- [ ] `encode_error_response` returns a response matching the request's oneof variant rather
      than always `InitializeResponse`
- [ ] Decode failures, where the variant is genuinely unknown, use a documented fallback

### Tests

- [ ] Unit test: the three handlers return `success == false`
- [ ] Unit test: `encode_error_response` round-trips to the requested variant

## Acceptance Criteria

- [ ] A scenario using a pedestrian fails loudly instead of passing
- [ ] A scenario using a misc object fails loudly instead of passing
- [ ] A scenario scripting traffic lights fails loudly instead of passing
- [ ] No handler in `coordinator.rs` returns `proto_ok()` without performing its action —
      verified by reading every handler, not just the three above
- [ ] `just test` passes
