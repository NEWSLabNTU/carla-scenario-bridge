# Phase 044: NPCs and Traffic Lights on Isaac Sim

The scenario's other road users and its signals, as the CARLA backend gives them: NPC
vehicles that are physics bodies tracking SSv2's pose, pedestrians that look like
pedestrians, and traffic lights whose state SSv2 sets, shown in the world and given to
Autoware. Stage 2 of three ([design/isaac-stack.svg](../design/isaac-stack.svg), yellow).

**Source**: user direction 2026-10-18.
**Depends on**: [043](043-isaac-single-vehicle-e2e.md).
**Feeds**: [039](039-isaac-multi-autoware.md) (the gap campaign on Isaac).

## Where it starts (survey, 2026-10-22)

- **NPCs are kinematic boxes.** `IsaacBackend` (`backend/isaac/mod.rs`) sends each NPC's
  pose in the next `Step`; the world server (`csb_isaac/world.py`) writes it to a
  `kinematicEnabled` rigid body with a box collider. `EntityPose` carries a pose only. Only
  the ego reports contacts. A kinematic NPC shoving the dynamic ego reads 37.5 m/s^2 to SSv2
  (038), which is why `isaac/town01_rear_contact` still needs a box ego.
- **Pedestrians are yellow boxes**; no CARLA walker has been exported (the vehicle export,
  `isaac_vehicles.sh`, covers `VehicleFactory` only).
- **Two-wheelers**: 7 are exported (`vehicles.json`, `wheels_n: 2`), but `nearest()` matches
  4-wheel cars only, so a bicycle box wears a car. **MiscObjects** are grey boxes even with
  `model3d="vehicle.*"` (incident 2's parked van); no `static.prop.*` is exported.
- **Signals**: `update_traffic_lights` refuses a non-empty list (`Unsupported`), and Autoware
  on Isaac gets no signals at all (aib launches acb's stack without `acb_bridge`, the
  publisher on CARLA). Town01's USD has 34 `Intersections/TrafficLight<N>` lamps with nested
  Green -> Yellow -> Red bulb prims named by UE label, not OpenDRIVE id; the `.xodr` has 36
  dynamic signals; `<map>/carla/traffic_lights.yaml` (lanelet way -> OpenDRIVE id -> CARLA
  position) exists for Town01 and Town02.
- **Simulator autopilot** and `agent` drivers are refused on Isaac.
- The topic acb publishes for Autoware is
  `/perception/traffic_light_recognition/external/traffic_signals` (the V2X input to the
  arbiter; `traffic_light_fusion_only` is the default), not `.../traffic_signals`.

| needs | scenarios |
|---|---|
| tracked NPCs | `basic/town01_rear_contact`, incidents 1, 4, 5, 6, `UC-ACC-001` |
| signals | `basic/town01_traffic_light`, incident 3 |
| pedestrians | `basic/town01_pedestrian`, incident 2, `UC-AEB-001` |
| two-wheelers, misc by model | incident 6 (cyclist), incident 2 (van, car), `town01_pedestrian` (barriers) |
| simulator autopilot | `basic/town01_simulator_autopilot` |

## Design (decisions: user, 2026-10-22)

### Who controls what

**SSv2 decides; csb relays; Isaac executes** -- as on CARLA, where csb freezes CARLA's own
light cycling and gives walkers no AI.

| | decides | relays | executes |
|---|---|---|---|
| NPC vehicles, walkers | SSv2: pose and speed each frame | csb (`npc_tracking.rs`) -> world server | Isaac: the body (velocity-driven or kinematic), the walk cycle played at that speed |
| traffic lights | SSv2: each signal's colour | csb -> `SetSignals` | Isaac: the lamp bulbs; aib's `EgoRos`: the signals to Autoware |

Isaac must not decide: scenario triggers (a walker darting when the ego is 7 s away, a red
as it approaches) only hold if SSv2 is the authority; SSv2 judges against its own world
model; and the acceptance compares SSv2's verdict on CARLA and on Isaac per scenario.

Isaac Sim 6.1's built-in automation (installed extensions, checked 2026-10-22) and its use
here: **Actor SDG** (`isaacsim.replicator.agent.core`, formerly `omni.anim.people`: animated
characters with Patrol / Idle / LookAround / go-to behaviours for synthetic-data crowds) --
its character assets are C's fallback, its behaviours are not used; **Navigation**
(`omni.anim.navigation`: NavMesh, A*) -- not used, SSv2 gives the path; **Animation Graph**
(`omni.anim.graph`: idle/walk blending on a speed variable) -- plays the walk cycle at SSv2's
speed (C); **Behavior scripting / OmniGraph action graphs** -- could switch lamp materials on
`SetSignals`, never cycle them on a timer; **Event Generation**
(`isaacsim.replicator.incident`) -- workplace incidents for training data, not used. Isaac
has **no traffic-light or road-traffic controller** (no Traffic Manager equivalent).

Isaac's own automation belongs to background life outside the scenario -- the Isaac
counterpart of CARLA's simulator autopilot (E, deferred): Actor SDG crowds on a NavMesh
would be that phase's starting point for pedestrians; road traffic would still be ours.

### A. Tracked NPCs: velocity-driven dynamic bodies

An NPC vehicle (and a two-wheeler) is a **dynamic rigid body** (mass from the vehicle
catalogue or the box, gravity, road friction) whose linear and angular velocity the world
server sets each step from what csb sends: the Isaac analogue of CARLA's
`set_target_velocity`. The control law stays in csb, unchanged: `npc_tracking.rs` (gains
5/s and 4/s, aim at `p* - v* dt`, the force bound `max(|a|,|d|) + 2 m/s^2`, curvature
feed-forward, spawn at speed, snap past 0.5 m / 5 deg, the blocked -> kinematic fallback),
so both backends share one law and one set of numbers.

- `csb_world.proto`: `Spawn.motion` (`KINEMATIC` | `TRACKED`); `EntityPose` gains `twist`
  (linear, angular) and `teleport` (a snap: set the pose this step, then resume); a
  per-entity switch to kinematic for the fallback. Backward compatible field additions.
- World server: velocities are written through PhysX (not USD ops) before the substeps;
  poses and velocities read back as today. Each tracked NPC gets a contact report (NPC-NPC,
  NPC-static, the ego's as before; the ground filter stays), as CARLA's per-NPC collision
  sensors (032 gap E).
- `kinematic` stays selectable (`npc_motion: kinematic` in the bridge config, and for
  benches and gap traces).
- **Risk, spiked first**: a box sliding on the town's triangle-mesh roads may catch on
  kerbs and seams (pitch, roll, stalls). The spike drives `town01_rear_contact` and the
  20-NPC bench on Isaac before the rest is built; remedies in order: a lower friction
  material on the NPC collider's sides, a rounded bottom (capsule or chamfer), a small
  hover with a downforce.

A PhysX vehicle per NPC (aib's model) would steer and drive truly, at a per-NPC wheel
simulation and tuning cost; not now.

### B. Traffic signals

- `csb_world.proto`: `SetSignals { repeated Signal { lanelet_id, opendrive_id, colour } }`,
  sent by `IsaacBackend::update_traffic_lights` (it stops refusing).
- **Signal table**: from `<map>/carla/traffic_lights.yaml` (lanelet way -> OpenDRIVE id ->
  CARLA position), the same artefact the CARLA backend checks; each lamp prim in the town
  USD matched to an OpenDRIVE signal by position (CARLA's Y flipped), or by the
  `omni:simready:signalID` attribute if it holds the id. Built once per town at
  `LoadWorld`, logged (matched / unmatched). Town03/05/10 need their table generated when
  a scenario there uses signals.
- **Lamps**: each bulb's emissive material switched on or off (not visibility: the bulbs
  are nested Green -> Yellow -> Red); off by default; the bulb meshes leave the collider
  layer if they carry colliders.
- **To Autoware**: aib's `EgoRos` publishes `autoware_perception_msgs/TrafficLightGroupArray`
  on `/perception/traffic_light_recognition/external/traffic_signals` in the ego's domain,
  from the state the world server holds, keyed by the Lanelet2 regulatory element ids as
  acb's publisher is. One publisher per ego, beside its vehicle interface.

### C. Pedestrians

A time-boxed **spike** first: export CARLA's walkers with SimReady as skeletal meshes with
their walk and idle animation sequences (UsdSkel), and play the animation at the walker's
speed on the world server. If SimReady does not carry the skeleton and animation over,
**fall back** to Isaac's own animated characters (`omni.anim.people` / NVIDIA's character
assets) chosen by size. Either way the collider stays the declared box, and walkers get a
`model` (CARLA blueprint or character id) the way vehicles do, with a `walkers.json` index.
CARLA's standing-walker turn-round (033: velocity flips at `WALKER_SPEED_FLOOR`) is not
carried over: a standing walker on Isaac faces SSv2's heading.

### D. Two-wheelers and misc objects

- `nearest()` filters by subtype: a bicycle or motorcycle box picks among the exported
  two-wheelers (check first whether the 1 MB bike exports carry a rider; if not, a rider
  mesh is added or the export fixed).
- A MiscObject with `model3d="vehicle.*"` wears that vehicle's mesh (static, kinematic).
- `static.prop.*` blueprints (at least `streetbarrier`, CARLA's MiscObject default) are
  exported by extending `isaac_vehicles.sh` / `ue_vehicle_levels.py` to prop blueprints.

### E. Simulator autopilot: deferred

`town01_simulator_autopilot` stays refused on Isaac and leaves 044's acceptance, recorded
as such. Driving NPCs along Lanelet2 on a backend with no traffic of its own may deserve a
phase of its own (user, 2026-10-22).

### F. Exit runs

The acceptance set on Isaac, SSv2's verdict compared with CARLA's per scenario, plus 043's
open item: its Town02-Town10 routes run on CARLA too (including the untested
`Town10: Town10HD` alias). CARLA and Isaac never run at the same time.

**Budget.** Isaac's own step is 6.0 / 7.1 ms (p50 / p95) with 20 NPCs wearing CARLA cars
(043, server139); SSv2's own frame costs ~20-25 ms per moving vehicle and ~10 ms per walking
pedestrian (033 step 8), so SSv2, not Isaac, limits busy scenarios on either backend. Tracked
NPCs add a velocity write and a read per NPC per step; measured in the A spike.

## Steps

1. [x] **Docs.** This design (2026-10-22).
2. [ ] **A spike**: a velocity-driven dynamic NPC on Town01; `town01_rear_contact` with the
   car-bodied ego and the 20-NPC bench, frame time; remedy kerb catching if seen.
3. [ ] **A**: proto, world server, `IsaacBackend` through `npc_tracking.rs`, NPC contacts;
   incidents 1, 4, 5 and `UC-ACC-001` on Isaac.
4. [ ] **B**: signal table, `SetSignals`, lamps, aib's publisher; `town01_traffic_light`
   and incident 3 on Isaac.
5. [ ] **D**: two-wheelers by subtype (rider check), MiscObjects by model, props export;
   incident 6, incident 2's parked cars, `town01_pedestrian`'s barriers.
6. [ ] **C spike, then C**: CARLA walkers through SimReady, else NVIDIA characters;
   incident 2, `town01_pedestrian`, `UC-AEB-001`.
7. [ ] **F**: exit runs on both backends; 043's CARLA routes.

## Acceptance

- `town01_traffic_light`, `basic/*` (except `town01_simulator_autopilot`, deferred),
  `demo/incidents/*`, AWF `UC-ACC-001` and `UC-AEB-001` pass on Isaac, with SSv2's verdict
  agreeing with CARLA's for each.
- 043's routes on Town02, Town03, Town05 and Town10 (`basic/town0X_ego_drive`, `town10_ego_drive`)
  run on CARLA too, with tracking of the same order as on Isaac. That includes the
  `Town10: Town10HD` alias in `bridge_config.yaml`, which has not been tested on CARLA.
- CARLA and Isaac never run at the same time (GPU and RAM): one backend's batch ends, and its
  stacks are stopped, before the other's starts.
