# Phase 044: NPCs and Traffic Lights on Isaac Sim

The scenario's other road users and its signals, as the CARLA backend gives them: NPC
vehicles that are physics bodies tracking SSv2's pose, pedestrians that look like
pedestrians, and traffic lights whose state SSv2 sets, shown in the world and given to
Autoware. Stage 2 of three ([design/isaac-stack.svg](../design/isaac-stack.svg), yellow).

**Source**: user direction 2026-10-18.
**Depends on**: [043](043-isaac-single-vehicle-e2e.md).
**Feeds**: [039](039-isaac-multi-autoware.md) (the gap campaign on Isaac).

## Design

- **NPC vehicles, tracked**: on CARLA an NPC is a physics body driven toward SSv2's pose
  (023, `npc_motion: tracked`); on Isaac it is kinematic today, so a contact with the ego
  differs (a kinematic body shoves without limit). NPCs become dynamic rigid bodies (or
  PhysX vehicles) tracking the pose with bounded force; kinematic stays as an option.
- **Pedestrians**: CARLA's walkers exported with SimReady (skeletal meshes), animated by
  speed; the collider stays the declared box. Lidar and perception then see a person.
- **Two-wheelers and misc objects**: the exported bikes and props by SSv2's model, as for
  cars (038).
- **Traffic lights in the world**: a signal table from each town's OpenDRIVE signals and the
  Lanelet2 map (lanelet traffic-light id ↔ the exported lamp prims). `UpdateTrafficLights`
  becomes a new `SetSignals` message; the world server sets the lamps' emissive state.
- **Traffic lights to Autoware**: the world server publishes
  `/perception/traffic_light_recognition/traffic_signals` on the ego's domain from the same
  state, as acb does on CARLA (ground truth standing in for perception). Lanelet2 ids, so no
  CARLA id table is needed for this part.
- **Simulator autopilot**: refused with a reason (Isaac has no traffic of its own) unless the
  campaign needs it; then csb drives such NPCs along Lanelet2 itself.

## Steps

- [ ] Tracked dynamic NPCs; contact scenario on Isaac matches CARLA's verdict.
- [ ] Walkers and two-wheelers exported and used by model.
- [ ] Signal table, `SetSignals`, lamps; traffic signals to Autoware.
- [ ] Exit runs.

## Acceptance

- `town01_traffic_light`, `basic/*`, `demo/incidents/*`, AWF `UC-ACC-001` and `UC-AEB-001`
  pass on Isaac, with SSv2's verdict agreeing with CARLA's for each.
