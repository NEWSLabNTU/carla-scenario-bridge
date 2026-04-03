# Phase 3: NPC + Pedestrian Support

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
- [ ] `SpawnPedestrianEntity` handler: spawn walker, register in entity manager
- [ ] `SpawnMiscObjectEntity` handler: spawn static prop, register in entity manager
- [ ] `DespawnEntity`: handle all entity types (vehicle, pedestrian, misc)

### Blueprint Mapping
- [ ] Direct match: check if `asset_key` is a valid CARLA blueprint
- [ ] Config lookup: load `blueprint_map` from `bridge_config.yaml`
- [ ] Category fallback: random blueprint from matching category
- [ ] Log warnings on fallback with the unresolved `asset_key`

### NPC Updates
- [ ] `UpdateEntityStatus`: handle `VEHICLE` type NPCs with `set_transform()`
- [ ] `UpdateEntityStatus`: handle `PEDESTRIAN` type with `set_transform()`
- [ ] `UpdateEntityStatus`: handle `MISC_OBJECT` type with `set_transform()`
- [ ] Verify NPC rotation updates look correct in CARLA spectator view

### Integration Testing
- [ ] Test scenario: ego + 1 NPC vehicle driving toward ego (cut-in or opposing lane)
- [ ] Test scenario: ego + 1 pedestrian crossing the road
- [ ] Test scenario: ego + static obstacle (barrier or cone on road)
- [ ] Verify Autoware perception detects NPC vehicles in LiDAR pointcloud
- [ ] Verify Autoware perception detects pedestrians
- [ ] Verify Autoware planning reacts to detected objects (slows down or stops)

## Acceptance Criteria

- [ ] NPC vehicles appear in CARLA at SSv2-commanded positions and move smoothly
- [ ] Pedestrians appear in CARLA at SSv2-commanded positions
- [ ] Static objects (barriers, cones) appear in CARLA at correct positions
- [ ] Autoware's LiDAR-based perception detects NPC vehicles (visible in `/perception/object_recognition/detection/objects`)
- [ ] Autoware's planning module reacts to a stationary NPC blocking the lane (ego slows or stops)
- [ ] A multi-actor scenario (ego + 2 NPCs + 1 pedestrian) runs for 60 seconds without crashes
- [ ] Unrecognized `asset_key` falls back gracefully with a log warning (no crash)
- [ ] `DespawnEntity` removes any entity type cleanly from CARLA
