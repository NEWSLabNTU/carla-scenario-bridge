# Phase 010: Multi-Instance and Background AVs

Several Autoware stacks driving real vehicles in one CARLA world.

**Source**: gaps 9-10 from
[multi-instance-architecture.md](../design/multi-instance-architecture.md), plus audit
findings D1 and D2.
**Depends on**: [007](007-repeatable-runs.md) for lifecycle, [009](009-map-and-traffic-lights.md)
for map loading.

## Problem

Two prerequisites block the feature before any fan-out work starts.

`config/bridge_config.yaml` is **never read**. There is no config loading anywhere in the
crate: `Coordinator::new(world)` takes no config, `serde` and `serde_yaml` are declared
dependencies and unused, and all real configuration arrives through environment variables in
`main.rs`. The documented `blueprint_map` mechanism does not exist — `spawn_vehicle_entity`
uses the raw `asset_key` with a hardcoded `vehicle.tesla.model3` fallback.

The ego's `role_name` is hardcoded:

```rust
if is_ego {
    match builder.set_attribute("role_name", "hero") { ... }
}
```

`acb_bridge` finds its vehicle by `role_name`, so with one fixed name there is exactly one
bridge, one Autoware, one domain. Background AVs need distinct names before anything else can
work.

The scale is bounded by upstream: SSv2 throws `"Multiple egos in the simulation are
unsupported yet."` and its concealer forks Autoware into SSv2's own `ROS_DOMAIN_ID`. So one
SSv2 drives one scenario ego, and additional Autoware instances are *background AVs* outside
SSv2's model — invisible to its conditions. See the design doc for why.

## Work Items

### Config loading (D1)

- [ ] Load `bridge_config.yaml` at startup; make `serde`/`serde_yaml` earn their place
- [ ] Precedence between file and environment is defined and documented
- [ ] Wire `blueprint_map` into spawn so the documented mechanism works
- [ ] Missing or malformed config fails at startup with a clear message, not at first use
- [ ] Delete any config key that nothing reads

### Role names (D2)

- [ ] Ego `role_name` is configurable, defaulting to `hero` for compatibility
- [ ] Background AVs get distinct role names from config
- [ ] Names are unique; a collision is rejected at startup, not discovered at spawn

### Background AV spawning (gap 9)

- [ ] `background_avs` config section: role name, blueprint, spawn pose, goal pose, domain
- [ ] Spawn them at `Initialize`, after map load and before the ego
- [ ] They are tracked for teardown like every other spawned actor (invariant 2, phase 007)
- [ ] They are **not** registered with SSv2 — no `EntityManager` entry that would reach
      `UpdateEntityStatus`
- [ ] Their pose authority is CARLA PhysX, never teleport (invariant 5)
- [ ] Empty list behaves exactly like today's single-ego run

### Per-domain launch

- [ ] Launch file bringing up Autoware + `acb_bridge` for one background AV in a given domain
- [ ] `publish_clock:=true` in background domains, `false` in the ego's — already supported
- [ ] A pilot per background domain to set route and engage, since no concealer is present
- [ ] Document the startup order across domains

### Concealer web-addr (gap 10)

- [ ] Parameterise the hardcoded `--web-addr 0.0.0.0:8082` in our SSv2 `launch.hpp` patch
- [ ] Confirm no port collision when several play_launch-managed stacks run on one host
- [ ] Keep the patch minimal — it is rebased onto upstream regularly

### Documentation

- [ ] Record that background AVs are invisible to SSv2 conditions, with the misc-object
      registration escape hatch described in the design doc
- [ ] Worked two-domain example

### Tests

- [ ] Unit: config parsing, including background AV lists and duplicate-name rejection
- [ ] Unit: `blueprint_map` resolution
- [ ] Integration: configured background AVs appear with correct role names
- [ ] Integration: background AVs are absent from SSv2 entity status
- [ ] Integration: all background AVs are destroyed at teardown

## Acceptance Criteria

- [ ] `bridge_config.yaml` is read, and every key in it does something
- [ ] Ego role name is configurable and defaults to `hero`
- [ ] A two-domain run works: scenario ego plus one background AV, both localize and drive
- [ ] Each domain has exactly one `/clock` publisher
- [ ] The background AV is perceived by the ego's Autoware through its sensors
- [ ] SSv2 neither sees nor controls the background AV
- [ ] An empty `background_avs` list is byte-for-byte the current behaviour
- [ ] Teardown leaves no background AV behind
- [ ] `just test` passes
