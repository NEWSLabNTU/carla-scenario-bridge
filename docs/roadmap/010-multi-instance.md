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

- [x] Load `bridge_config.yaml` at startup; make `serde`/`serde_yaml` earn their place
- [x] Precedence between file and environment is defined and documented
- [x] Wire `blueprint_map` into spawn so the documented mechanism works
- [x] Missing or malformed config fails at startup with a clear message, not at first use
- [x] Delete any config key that nothing reads

**Precedence: environment > file > default.** The environment is the more specific source —
launch files set `CARLA_HOST` / `CARLA_PORT` / `SSV2_PORT` per run, and a checked-in config
must not silently win over what a launch explicitly asked for.

A *missing* file is fine, since every key has a default. A *malformed* one is fatal: falling
back to defaults would run the scenario with settings the operator believes they changed.
`timeout_ms` was dropped — nothing read it.

### Role names (D2)

- [x] Ego `role_name` is configurable, defaulting to `hero` for compatibility
- [x] Background AVs get distinct role names from config
- [x] Names are unique; a collision is rejected at startup, not discovered at spawn

`role_name` is no longer a property of the spawn kind — it is passed in, so the ego takes its
name from config and each background AV takes its own. Duplicates (including one colliding
with the ego) are rejected by `BridgeConfig::validate`, because `acb_bridge` finds its vehicle
by `role_name` and a duplicate means two bridges racing for one vehicle.

### Background AV spawning (gap 9)

- [x] `background_avs` config section: role name, blueprint, spawn pose, goal pose, domain
- [x] Spawn them at `Initialize`, after map load and before the ego
- [x] They are tracked for teardown like every other spawned actor (invariant 2, phase 007)
- [x] They are **not** registered with SSv2 — no `EntityManager` entry that would reach
      `UpdateEntityStatus`
- [x] Their pose authority is CARLA PhysX, never teleport (invariant 5)
- [x] Empty list behaves exactly like today's single-ego run

`SpawnKind::entity_type` returns `Option<EntityType>` and `None` for a background AV. That is
the mechanism keeping it out of SSv2's view: registering it would put it into
`UpdateEntityStatus`, and SSv2 would start teleporting a vehicle Autoware is already driving —
two pose authorities on one actor.

Spawning happens after the map (a reload destroys actors) and before the ego, so the
background Autoware instances can be finding their vehicles while SSv2 is still setting up. A
background AV that fails to spawn is reported loudly but does not abort the scenario; the ego
can still run, and killing a whole scenario because a secondary vehicle would not fit is the
wrong trade.

### Per-domain launch

- [x] Launch file bringing up Autoware + `acb_bridge` for one background AV in a given domain
- [x] `publish_clock:=true` in background domains, `false` in the ego's — already supported
- [x] A pilot per background domain to set route and engage, since no concealer is present
- [x] Document the startup order across domains

`csb_launch/launch/background_av.launch.xml` brings up Autoware, `acb_bridge` and the pilot
for one AV.

The pilot is wired but unverified. `acb_pilot`'s `auto_drive` node now launches per domain:
it waits for the AD API services and GNSS-driven localization, sets the route, engages, and
exits on arrival — the concealer's job, minus the concealer. It runs unless
`use_pilot:=false`.

The goal does **not** come from `bridge_config.yaml`. The pilot's only input is a
`poses_file` ROS parameter — a YAML with `goal_pose: {x, y, z, qx, qy, qz, qw}` — while the
config records `goal_pose: {x, y, z, yaw}` inline. A file path versus an inline pose, a
quaternion versus a yaw: aligning them would mean either changing the pilot (a read-only
submodule) or generating pose files at launch time. So the launch takes a
`goal_poses_file` argument, and the config's `goal_pose` stays what it always was to the
bridge — an informational record of intent, read by nobody.

What still needs a live stack: that the pilot's engage actually succeeds in a background
domain (its timing waits were tuned in single-domain runs), and that failure is loud. The
failure mode is bounded, though — an empty `goal_poses_file` is a fatal log and a nonzero
exit, not a vehicle that silently never moves.

### Concealer web-addr (gap 10)

- [x] Parameterise the hardcoded `--web-addr 0.0.0.0:8082` in our SSv2 `launch.hpp` patch
- [ ] Confirm no port collision when several play_launch-managed stacks run on one host
- [x] Keep the patch minimal — it is rebased onto upstream regularly

Now reads `PLAY_LAUNCH_WEB_ADDR`, falling back to `0.0.0.0:8082`. Four added lines, so the
rebase burden is unchanged. Confirming no collision needs two stacks actually running.

### Documentation

- [x] Record that background AVs are invisible to SSv2 conditions, with the misc-object
      registration escape hatch described in the design doc
- [x] Worked two-domain example

Both live in [multi-instance-architecture.md](../design/multi-instance-architecture.md), and
the invisibility warning is repeated in `bridge_config.yaml` where an operator adding a
background AV will actually read it.

### Tests

- [x] Unit: config parsing, including background AV lists and duplicate-name rejection
- [x] Unit: `blueprint_map` resolution
- [x] Unit: a background AV is not an SSv2 entity, and is physics-driven
- [ ] Integration: configured background AVs appear with correct role names
- [ ] Integration: background AVs are absent from SSv2 entity status
- [ ] Integration: all background AVs are destroyed at teardown

## Acceptance Criteria

- [x] `bridge_config.yaml` is read, and every key in it does something
- [x] Ego role name is configurable and defaults to `hero`
- [ ] A two-domain run works: scenario ego plus one background AV, both localize and drive
- [ ] Each domain has exactly one `/clock` publisher
- [ ] The background AV is perceived by the ego's Autoware through its sensors
- [x] SSv2 neither sees nor controls the background AV
- [x] An empty `background_avs` list is byte-for-byte the current behaviour
- [ ] Teardown leaves no background AV behind
- [x] `just test` passes

The unchecked criteria all need a live two-domain run. The mechanism keeping background AVs
out of SSv2's view is enforced by construction and unit-tested, and the per-domain pilot is
wired into the launch — but **no background AV has ever been spawned**, so whether the pilot
actually engages and drives one has never been observed.
