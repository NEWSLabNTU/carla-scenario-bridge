# Phase 008: Entity Fidelity

Pedestrians, misc objects, and NPCs that behave the way SSv2 believes they do.

**Source**: workflow audit 2026-07-28, findings A1, A2, C1, C3.
**Supersedes**: [003-npc-pedestrian-support.md](003-npc-pedestrian-support.md), which
predates the authority model.
**Depends on**: [006](006-honest-failures.md) for honest rejection, [007](007-repeatable-runs.md)
for repeatable testing.

## Problem

Two entity types are unimplemented (phase 006 makes that visible), and the one type that *is*
implemented is puppeteered incorrectly.

Invariant 5 says a vehicle's pose has exactly one authority: SSv2 teleport, or CARLA PhysX.
For scenario NPCs it is SSv2 — `update_entity_status` calls `set_transform()` each frame. But
the actors are spawned with CARLA physics **enabled**, so PhysX also acts on them between
ticks: gravity pulls, collisions shove, suspension settles. Two authorities, one actor.

The AWSIM pattern this design follows makes puppeteered actors kinematic. Without
`set_simulate_physics(false)` the NPC's actual pose drifts from the commanded pose, and the
pose reported back to SSv2 is the commanded one anyway — so SSv2 cannot see the drift.

Ego readback also reports three constants:

```rust
current_action: String::new(),
linear_jerk: 0.0,
angular: Some(geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 }),
```

Any SSv2 condition reading action state or jerk is reading a constant, not the simulation.

## Work Items

### NPC physics (C1)

- [ ] Disable physics on scenario NPCs at spawn, so `set_transform` is the only authority
- [ ] Decide and document the pedestrian equivalent — CARLA walkers differ from vehicles
- [ ] Verify the commanded pose and CARLA's reported pose agree within tolerance after a tick
- [ ] Document the consequence in the design doc: NPC-to-NPC and NPC-to-ego collisions are
      not simulated by CARLA, matching AWSIM; SSv2 owns collision detection

### Pedestrians (A1)

- [ ] `spawn_pedestrian_entity` spawns a CARLA walker at the converted pose
- [ ] Register in `EntityManager` as `EntityType::Pedestrian`
- [ ] `update_entity_status` drives walkers by pose, consistent with the NPC path
- [ ] Walker despawn works through the existing `despawn_entity`
- [ ] Decide whether an AI walker controller is used at all — under SSv2 authority it should
      not be, since SSv2 computes the path

### Misc objects (A2)

- [ ] `spawn_misc_object_entity` spawns a static prop
- [ ] Register as `EntityType::MiscObject`
- [ ] Map SSv2 asset keys to CARLA prop blueprints, with a documented fallback
- [ ] Static objects are not teleported per frame unless SSv2 moves them

### Ego readback fidelity (C3)

- [ ] Populate `current_action` from real state, or document why empty is correct
- [ ] Compute `linear_jerk` from acceleration history, or document why zero is acceptable
- [ ] Populate angular acceleration, or document the omission
- [ ] Whatever is left constant is recorded as a known limitation, not left silent

### Blueprint mapping

- [ ] SSv2 asset key → CARLA blueprint resolution for all four entity types
- [ ] Unresolvable keys log a warning naming the key and the fallback used
- [ ] Depends on config loading from [010](010-multi-instance.md); until then the mapping
      table lives in code with a TODO pointing at that phase

### Tests

- [ ] Unit: asset key → blueprint resolution, including fallback and unknown-key paths
- [ ] Integration: spawned pedestrian appears in CARLA and tracks its commanded pose
- [ ] Integration: spawned misc object appears and stays put
- [ ] Integration: NPC pose after a tick matches the commanded pose within tolerance
- [ ] Integration: all three types despawn cleanly, no leak (ties to 007)

## Acceptance Criteria

- [ ] A pedestrian-crossing scenario runs with a pedestrian actually present in CARLA
- [ ] Autoware's perception detects the pedestrian through its sensors
- [ ] NPC vehicles follow their scripted trajectory without sinking, jittering or drifting
- [ ] A misc object appears as an obstacle Autoware perceives
- [ ] Every entity type spawns, updates and despawns without leaking
- [ ] No handler introduced here returns success without acting
- [ ] `just test` passes
