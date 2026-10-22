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
2. [x] **A spike**: a velocity-driven dynamic NPC on Town01; `town01_rear_contact` with the
   car-bodied ego and the 20-NPC bench, frame time; remedy kerb catching if seen (skids,
   decals out; 2026-10-22).
3. [ ] **A**: proto, world server, `IsaacBackend` through `npc_tracking.rs`, NPC contacts;
   incidents 1, 4, 5 and `UC-ACC-001` on Isaac.
4. [ ] **B**: signal table, `SetSignals`, lamps, aib's publisher; `town01_traffic_light`
   and incident 3 on Isaac.
5. [ ] **D**: two-wheelers by subtype (rider check), MiscObjects by model, props export;
   incident 6, incident 2's parked cars, `town01_pedestrian`'s barriers.
6. [ ] **C spike, then C**: CARLA walkers through SimReady, else NVIDIA characters;
   incident 2, `town01_pedestrian`, `UC-AEB-001`.
7. [ ] **F**: exit runs on both backends; 043's CARLA routes.

## Step 2: the A spike

**Plan** (2026-10-22). The spike builds the part of A it has to prove and nothing it does
not: a tracked NPC on Isaac is a dynamic rigid body the world server drives by velocity,
and csb drives it with `npc_tracking.rs`. What it adds, kept for step 3:

- `csb_world.proto` (protocol 2): `Spawn.motion` (`KINEMATIC`, the default and every entity
  so far, or `TRACKED`); `EntityPose.twist` (linear and angular velocity, world frame, at the
  entity origin), `teleport` (the pose is set this step, then the twist) and `kinematic`
  (hand the body to kinematic for the rest of its life: the fallback); `EntityState.twist`,
  the body's velocity after the step at its origin, from PhysX.
- World server: a tracked entity is a dynamic body (mass from CARLA's `physics.json` for its
  model, else by kind; centre of mass at the box centre), its velocity written with PhysX's
  tensor API (`omni.physics.tensors` rigid body views, not USD attributes) once before the
  substeps, as `set_target_velocity` is once per CARLA tick; the twist at the origin becomes
  the centre of mass's (`v + w x r`). Gravity and the road keep z, roll and pitch.
- csb: `IsaacBackend` tracks every scenario vehicle under `npc_motion: tracked` with the
  CARLA backend's law (`npc_tracking::command`, `bounded_command`, `first_frame_change`,
  `pose_yaw_rate`), in the world frame (ROS: the law is frame-free as long as yaw and yaw
  rate agree). No steer: a box has no tyres to undo a set yaw rate.
- Measured: `basic/town01_rear_contact` with the car-bodied ego (aib's PhysX vehicle, the
  ego stack driving), the 20-NPC bench (tracking error p95/max, snaps, pitch and roll of the
  boxes, stalls), and the world server's step time against 043's 6.0 / 7.1 ms.

Kerb catching, if seen, gets the remedies in the design's order. If none works, the spike
stops and the choice goes back to the user.

**Results** (2026-10-22, server139, Town01 SimReady world, world server with aib's ego,
`just scenario ... backend:=isaac`, logs and gap traces in `/mnt/wd/aeon/csb-044/`).

1. **The tensor API works for this.** One `omni.physics.tensors` rigid body view over every
   tracked body, rebuilt when the set changes; a body spawned since the last step is not in
   PhysX yet (`create_rigid_body_view` matches nothing), so its first drive goes through USD
   (pose, initial velocity, read when PhysX creates it) and the view takes it after one
   step. Two traps, both fixed: the server's first rigid body view zeroed the velocity of the
   body it was made for (the first tracked NPC of a server's life started at rest), so a
   throwaway tracked body takes that view at start-up; and the view needs the stage id
   (`create_simulation_view("numpy", stage_id=...)`).
2. **Kerb and seam catching: seen, and remedied by the second remedy.** With the declared box
   flat on the road (its bottom at the entity origin), boxes in a standalone spike (20 boxes
   driven at 8 m/s along Town01's y = -130 road, `IsaacWorld` directly) pitched up to 4 deg,
   jumped up to 1.65 m and stalled; in `town01_rear_contact` the NPC stopped dead 1.4 m short
   of the ego (pitched 1.7 deg, launched at 2.2 m/s up), fell back to kinematic as blocked,
   and SSv2's blind NPC then drove it through the ego. Friction was already low (remedy 1:
   0.1, combine `min`) and did not help: the box's front face meets an edge, a normal force.
   Remedy 2, a rounded bottom: the box rides on two capsules along its length (radius 0.25 m,
   left and right, bottoms on the declared box's bottom; `motion.skids`), its own bottom
   raised by the radius; the top, length and width stay the declared box's. The same spike:
   17 of 17 free boxes error p95 0.007-0.014 m, roll/pitch at most 1.6 deg, z within 4 cm;
   a box driven sideways into the kerb at 2 m/s climbs it (pitch 0.9 deg) instead of being
   launched (z 1.46 m, pitch 11.9 deg before). Remedy 3 (hover) was not needed.
3. **Town01's decals were the seams.** The NPCs' own contacts named
   `/World/env/Decals/dirtDebris*/SM_DirtDebris01`: dirt and debris meshes a few millimetres
   over the road, render-only in CARLA, which `physics_layer` had given colliders.
   `physics_layer` now skips `Decals` and `SM_DirtDebris*`/`dirtDebris*` (Town01 22 meshes,
   Town05 10); the five towns' layers were regenerated (the old ones kept as
   `*_physics.usda.pre044`).
4. **SSv2's ego acceleration.** The first car-bodied runs failed on SSv2's
   `current acceleration (35.3) is out of range [-15, 15]` for the ego at the contact: the
   Isaac backend reported the raw differenced acceleration, while the CARLA backend reports
   the median of the last three frames (`ACCEL_SMOOTHING_FRAMES`: a one-frame jolt is
   discarded). Ported to Isaac; but differencing poses smears a velocity step over two
   frames, which the median passes (16-57 m/s2 in the next three runs). The world server now
   returns the ego vehicle's own velocity (aib's, from its last substep) in `EntityState.twist`
   and csb takes the motion from a body's twist where it has one: a shove is one frame, the
   median drops it.

| run | verdict | NPC tracking before the contact | after it | NPC roll/pitch max | world server step p50 |
|---|---|---|---|---|---|
| `town01_rear_contact` x3 (decals in) | 3/3 PASS | no snap, no fallback | held by the ego (error p95 0.85-3.8 m, as SSv2's blind NPC drives on) | 0.28-1.0 deg | 28-31 ms |
| `town01_rear_contact` x3 (decals out) | 3/3 PASS | no snap, no fallback | as above | 0.28 deg | 29-31 ms |
| bench `town01_npc_20` x2 (decals in) | 2/2 PASS | error p95 median 0.026 m, yaw p95 median 0.04 deg; 0 snaps, 0 fallbacks | | median 1.6 deg | 14.0 / 14.1 ms |
| bench `town01_npc_20` x1 (decals out) | PASS | error p95 median 0.026 m (worst 1.26), yaw p95 median 0.04 deg (worst 0.45); 0 snaps, 0 fallbacks | | median 1.6 deg | 14.0 ms (p95 15.6) |

The bench's worst NPC (`npc_07`, error max 2.9 m, yaw up to 20.6 deg on `npc_10`, z up to
0.96 m) is a collision SSv2 makes: from frame 1050 of 1201 its own boxes for `npc_07` and
`npc_10` overlap at a junction (the generated bench drives them into each other), and Isaac's
bodies meet there. The ego in `town01_rear_contact` is shoved 0.6-0.7 m/s when standing, from
0.9 to 5.3 m/s when moving; CARLA's numbers for the same contact are 4 -> ~9 m/s.

**Cost.** 20 tracked NPCs: 14.0 ms p50 per step against 043's 6.0 ms for 20 kinematic ones
(no ego either time): ~0.4 ms per tracked NPC (one batched velocity write and read, and
their contact reports, whose ground filter is now a per-path cache; uncached it was 30 ms).
With the ego's sensors the step is the ego's, 28-31 ms, as in 043.

**Decision**: velocity-driven dynamic NPCs work on Town01's roads with skids; step 3 builds on
this code as it stands.

## Acceptance

- `town01_traffic_light`, `basic/*` (except `town01_simulator_autopilot`, deferred),
  `demo/incidents/*`, AWF `UC-ACC-001` and `UC-AEB-001` pass on Isaac, with SSv2's verdict
  agreeing with CARLA's for each.
- 043's routes on Town02, Town03, Town05 and Town10 (`basic/town0X_ego_drive`, `town10_ego_drive`)
  run on CARLA too, with tracking of the same order as on Isaac. That includes the
  `Town10: Town10HD` alias in `bridge_config.yaml`, which has not been tested on CARLA.
- CARLA and Isaac never run at the same time (GPU and RAM): one backend's batch ends, and its
  stacks are stopped, before the other's starts.
