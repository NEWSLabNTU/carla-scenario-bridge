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

- [x] `spawn_pedestrian_entity` returns `Success=false` with a description naming phase 008
- [x] `spawn_misc_object_entity` returns `Success=false`, same
- [x] `update_traffic_lights` returns `Success=false` with a description naming phase 009
- [x] Each logs at `warn` on first call, not per frame, so the log stays readable

### Unknown-entity handling

- [x] `update_entity_status` logs a warning the first time it sees an entity absent from
      `EntityManager`, naming the entity — silent echo is what let A1 hide
- [x] Decide and document whether echoing is the right fallback at all, or whether an
      unknown entity should fail the request

**Decision: keep echoing, warn once.** Failing the request would abort an otherwise healthy
scenario when a status arrives for an entity mid-despawn. With spawn now rejecting loudly,
SSv2 aborts at spawn time, so reaching this path is already exceptional — it earns a warning,
not a scenario failure. Recorded in the code at the branch itself.

### Protocol correctness (E1)

- [x] `encode_error_response` returns a response matching the request's oneof variant rather
      than always `InitializeResponse`
- [x] Decode failures, where the variant is genuinely unknown, use a documented fallback

The variant is recovered by `peek_request_variant`, which reads the leading protobuf tag.
`SimulationRequest` is a bare `oneof`, so the first tag identifies the intended request even
when the rest of the payload is malformed. Request and response assign the same field number
to each variant, so the mapping is direct. `None` — an empty oneof, or a message too corrupt
to yield a tag — falls back to `Initialize`.

Worth recording why this matters at all: SSv2 calls `client.call(request).update_frame()`, and
protobuf returns a **default-constructed** message when the response holds a different
variant. So a mismatched variant still reads as `success == false` — the failure was never
lost. What was lost is the `description`, leaving the operator with a bare failure and no
reason.

### Additional handlers found by the acceptance audit

Reading every handler rather than only the three known ones turned up three more instances of
the same bug:

- [x] `despawn_entity` returned success even when `actor.destroy()` errored, so SSv2 believed
      an entity was gone while it was still in the world — blocking spawn points and visible
      to Autoware's sensors. Now reports failure. "Already destroyed" and "not found" remain
      successes: the requested end state holds either way.
- [x] `update_entity_status` returned success even when `set_transform` failed, so SSv2 went
      on believing an NPC had moved. Now collects failures and reports them.
- [x] `initialize` silently ignored `lanelet2_map_path`. Loading is phase 009, so this cannot
      be fixed here — but it now warns prominently that a map mismatch will not fail, it will
      produce meaningless poses.

### Tests

- [x] Unit test: the three handlers return `success == false`
- [x] Unit test: `encode_error_response` round-trips to the requested variant
- [x] Unit test: every known variant round-trips as a failure carrying its description
- [x] Unit test: `peek_request_variant` recovers the variant from real encoded requests
- [x] Unit test: rejections always carry a non-empty description

## Acceptance Criteria

- [ ] A scenario using a pedestrian fails loudly instead of passing
- [ ] A scenario using a misc object fails loudly instead of passing
- [ ] A scenario scripting traffic lights fails loudly instead of passing
- [x] No handler in `coordinator.rs` returns `proto_ok()` without performing its action —
      verified by reading every handler, not just the three above
- [x] `just test` passes

The first three require a live CARLA and a full stack run. The code path is unit-tested and
the rejection is unconditional, but **no scenario has actually been run** — these stay
unchecked until someone does.

## Known Issue Left Open

`cargo clippy -- -D warnings`, and therefore `just check` and `just ci`, fails with 18
dead-code errors. All are pre-existing and none were introduced here: generated protobuf
types, `traffic_light_mapper`, `entity_manager::ego`, and the `EntityType::Pedestrian` /
`MiscObject` variants. Phase 008 constructs the entity variants and phase 009 uses the
mapper, so most resolve naturally; the generated protobuf types need an `allow` attribute.
