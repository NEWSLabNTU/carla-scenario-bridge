# Phase 3: NPC + Pedestrian Support

> **Superseded by [008-entity-fidelity.md](008-entity-fidelity.md)** (2026-07-28). This
> document predates the authority model in
> [design/multi-instance-architecture.md](../design/multi-instance-architecture.md) and does
> not cover the NPC physics problem — puppeteered actors are still spawned with CARLA physics
> enabled, so PhysX and SSv2 teleport both act on the same actor. Retained for context; work
> items live in 008.

Multi-actor scenarios with NPC vehicles, pedestrians, and static objects visible in CARLA and detectable by Autoware's perception pipeline.

## Design

### NPC Vehicle Puppeteering

SSv2's `traffic_simulator` computes NPC poses each frame using behavior plugins (BehaviorTree, ContextGamma). The adapter receives these poses in `UpdateEntityStatus` and teleports each NPC via `set_transform()`.

**Teleport vs physics**: NPCs are kinematic (teleported), not physics-driven. CARLA's collision detection does not apply to teleported actors. This matches the AWSIM design where SSv2 owns NPC dynamics and handles collision checks via bounding box intersection.

**Rotation interpolation**: `set_transform()` snaps orientation instantly. At 20Hz this can look jerky for turning NPCs. Optionally interpolate rotation between frames using SLERP on quaternions, but this is a polish item - SSv2 already sends smoothly interpolated poses.

### Pedestrian Spawning

`SpawnPedestrianEntity` spawns a CARLA walker:
1. Map `asset_key` to a `walker.pedestrian.*` blueprint
2. Spawn the walker actor at the given pose
3. **Do NOT attach AI controller** - SSv2 controls pedestrian positions via `UpdateEntityStatus` (same as AWSIM puppeteering pattern)
4. Register in `EntityManager` as `EntityType::Pedestrian`

Pedestrians are teleported each frame like NPC vehicles. CARLA's AI Walker Controller is not used since SSv2 owns the behavior.

### Misc Object Spawning

`SpawnMiscObjectEntity` spawns static props (barriers, cones, boxes):
1. Map `asset_key` to a CARLA static prop blueprint (e.g., `static.prop.constructioncone`)
2. Spawn at given pose
3. Register as `EntityType::MiscObject`

Static objects are set once and typically not moved, but `UpdateEntityStatus` supports updating their positions.

### Blueprint Mapping

SSv2 sends `asset_key` strings that may not match CARLA blueprint names. The mapping strategy:

1. **Direct match**: If `asset_key` is a valid CARLA blueprint (e.g., `vehicle.tesla.model3`), use it
2. **Config lookup**: Check `bridge_config.yaml` `blueprint_map` section
3. **Category fallback**: If no match, pick a random blueprint from the appropriate category (`vehicle.*` for vehicles, `walker.pedestrian.*` for pedestrians, `static.prop.*` for misc objects)
4. **Log warning**: Always log when falling back so the user knows to add a mapping

### Bounding Box Consistency

SSv2 sends `VehicleParameters` with `bounding_box` (center + dimensions). CARLA vehicles have their own bounding boxes. Mismatches could affect SSv2's collision detection (which uses the bounding box it sent, not CARLA's). This is acceptable since SSv2's collision detection is bounding-box-based regardless of the backend.

## Tasks

### Pedestrian + Misc Object Handlers
- [x] `SpawnPedestrianEntity` handler: spawn walker, register in entity manager
      `town01_pedestrian.xosc` spawns `pedestrian_0` (audit 2026-10-06)
- [x] `SpawnMiscObjectEntity` handler: spawn static prop, register in entity manager
      same scenario spawns `barrier_0`, `barrier_1` (`static.prop.streetbarrier`) (audit 2026-10-06)
- [x] `DespawnEntity`: handle all entity types (vehicle, pedestrian, misc)
      bridge log despawns pedestrian, misc objects and vehicles at every scenario end (audit 2026-10-06)

### Blueprint Mapping
- [x] Direct match: check if `asset_key` is a valid CARLA blueprint
      `choose_blueprint` (coordinator.rs), unit-tested (audit 2026-10-06)
- [x] Config lookup: load `blueprint_map` from `bridge_config.yaml`
      `BridgeConfig::blueprint_for` (config.rs), unit-tested (audit 2026-10-06)
- [x] Category fallback: random blueprint from matching category
      implemented as a deterministic fallback blueprint instead of a random one, so runs are repeatable (audit 2026-10-06)
- [x] Log warnings on fallback with the unresolved `asset_key`
      `choose_blueprint` logs the unresolved key (audit 2026-10-06)

### NPC Updates
- [x] `UpdateEntityStatus`: handle `VEHICLE` type NPCs with `set_transform()`
      kinematic NPCs, see design multi-instance-architecture.md (audit 2026-10-06)
- [x] `UpdateEntityStatus`: handle `PEDESTRIAN` type with `set_transform()`
      walker teleported every frame, ground height fixed in 014 (audit 2026-10-06)
- [x] `UpdateEntityStatus`: handle `MISC_OBJECT` type with `set_transform()`
      barriers (audit 2026-10-06)
- [x] Verify NPC rotation updates look correct in CARLA spectator view
      measured rather than eyeballed: the 014 pose probe placed a heading-180° NPC with Δ 0.000 m along heading (audit 2026-10-06)

### Integration Testing
- [x] Test scenario: ego + 1 NPC vehicle driving toward ego (cut-in or opposing lane)
      `town01_rear_contact.xosc` (Town01 has no adjacent same-direction lane for a cut-in) (audit 2026-10-06)
- [x] Test scenario: ego + 1 pedestrian crossing the road
      `town01_pedestrian.xosc` (audit 2026-10-06)
- [x] Test scenario: ego + static obstacle (barrier or cone on road)
      barriers in `town01_pedestrian.xosc` (audit 2026-10-06)
- [x] Verify Autoware perception detects NPC vehicles in LiDAR pointcloud
      014: `obstacle_stop` named the parked `bg_av_1` from LiDAR detection and stopped for it (audit 2026-10-06)
- [x] Verify Autoware perception detects pedestrians
      the ego holds for the walker in `town01_pedestrian.xosc` (008) (audit 2026-10-06)
- [x] Verify Autoware planning reacts to detected objects (slows down or stops)
      same two stops (audit 2026-10-06)

## Acceptance Criteria

- [x] NPC vehicles appear in CARLA at SSv2-commanded positions and move smoothly
      pose Δ 0.000 m (014 probe); kinematic, so no physics jitter (audit 2026-10-06)
- [x] Pedestrians appear in CARLA at SSv2-commanded positions
      014 walker ground height fix (audit 2026-10-06)
- [x] Static objects (barriers, cones) appear in CARLA at correct positions
      barriers in the pedestrian scenario (audit 2026-10-06)
- [x] Autoware's LiDAR-based perception detects NPC vehicles (visible in `/perception/object_recognition/detection/objects`)
      see the `bg_av_1` stop above (audit 2026-10-06)
- [x] Autoware's planning module reacts to a stationary NPC blocking the lane (ego slows or stops)
      `route-obstacle` stop behind `bg_av_1` (014) (audit 2026-10-06)
- [x] A multi-actor scenario (ego + 2 NPCs + 1 pedestrian) runs for 60 seconds without crashes
      pedestrian scenario: ego, walker and two barriers, ~300 s, no crash; two_av adds a second AV (audit 2026-10-06)
- [x] Unrecognized `asset_key` falls back gracefully with a log warning (no crash)
      `choose_blueprint` fallback path, unit-tested (audit 2026-10-06)
- [x] `DespawnEntity` removes any entity type cleanly from CARLA
      every scenario end; orphan sweep (`reap_parentless_sensors`) covers leftovers (audit 2026-10-06)
