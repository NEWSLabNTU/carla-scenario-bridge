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
  are nested Green -> Yellow -> Red); green until SSv2 commands a signal, as the CARLA
  backend sets every mapped light (step 3 found a dark default stopping the ego); the bulb
  meshes leave the collider layer if they carry colliders.
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
3. [x] **A**: proto, world server, `IsaacBackend` through `npc_tracking.rs`, NPC contacts;
   incidents 1, 4, 5 and `UC-ACC-001` on Isaac (2026-10-22: all PASS, as on CARLA).
4. [x] **B**: signal table, `SetSignals`, lamps, aib's publisher; `town01_traffic_light`
   and incident 3 on Isaac (2026-10-22: 2/2 PASS each).
5. [x] **D**: two-wheelers by subtype (rider check), MiscObjects by model, props export;
   incident 6, incident 2's parked cars, `town01_pedestrian`'s barriers (2026-10-22; the
   rider stays in its bind pose, open).
6. [x] **C spike, then C**: CARLA walkers through SimReady, else NVIDIA characters;
   incident 2, `town01_pedestrian`, `UC-AEB-001` (2026-10-22: CARLA's walkers, no fallback;
   all SSv2 PASS; the ego touches `town01_pedestrian`'s walker in 4 of 5, open).
7. [x] **F**: exit runs on both backends; 043's CARLA routes (2026-10-22: every scenario
   passes on Isaac; SSv2 agrees with CARLA everywhere but `UC-ACC-001` on the CenterPoint
   stack, which CARLA fails by 0.02 m/s^2; Town02-10 PASS on CARLA, the alias works).

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

## Step 3: A

**Plan** (2026-10-22). The spike's code is A's (proto, world server, `IsaacBackend` through
`npc_tracking`, NPC contacts, kinematic selectable). What step 3 adds:

- The fallback checked on Isaac itself: a tracked body made kinematic mid-run follows the
  poses it is sent and stops reporting a twist (standalone, `IsaacWorld`).
- The contact report on Isaac carries the episode's `tracking` rows and the NPCs' own
  contacts (`npc_contacts`), as on CARLA (done in the spike's code; checked in a report).
- Runs on Isaac, with the ego stack as the demo tour runs it on CARLA (`just ego-isaac
  max_velocity:=11.11 lidar_detection_model:=centerpoint use_multi_channel_tracker_merger:=false`),
  gap trace on: incidents 1, 4 and 5 (`incident_judge` on each trace and its
  `/simulation/context` log) and `UC-ACC-001-0001` cut to one variant as 024 and 032 ran it
  (ego 8.333 m/s, NPC 5.556 -> 2.778 m/s); SSv2's verdict against CARLA's (033 step 5
  verification: incidents 1-5 25/25 PASS; 032 step 7: UC-ACC 1 variant PASS).

**Results** (2026-10-22, server139; one world server, `just run backend:=isaac` traced, the
ego stack as the tour runs it: `max_velocity:=11.11 lidar_detection_model:=centerpoint
use_multi_channel_tracker_merger:=false`; runs, traces, judges in `/mnt/wd/aeon/csb-044/runs/`).

- **The fallback on Isaac** (standalone `IsaacWorld`): a tracked body handed to kinematic
  goes to the poses it is sent from that step (5.0, then 9.0, 10.0, ... m) and its state
  stops carrying a twist; the other tracked body keeps its velocity throughout.
- **Despawning a tracked body** printed `physics.tensors simulationView was invalidated` (a
  prim deleted under a live rigid body view); the view is now released before the prim
  goes, and a body's neighbours keep their velocity across a despawn (7.95 m/s before and
  after, standalone).
- **The contact report** carries `tracking` (e.g. UC-ACC's `Npc1`: 812 frames, error p95
  0.013 m) and `npc_contacts`, as on CARLA.

| scenario | Isaac: SSv2 verdict | incident_judge | NPC tracking (error p95 / max, snaps) | gap oracle | CARLA (033 step 5, 032 step 7) |
|---|---|---|---|---|---|
| incident 1, lead brakes | 2/2 PASS | PASS: lead 6.05 m/s2, ego 8.4 m/s 50 m back at the brake, stood 7.4-7.6 s, 5.15 m behind | 0.019 / 0.037 m, 0 | G1-G3 0, G4 agree | PASS |
| incident 4, blocked lane | 2/2 PASS | PASS: waited 12.0 s 5.2 m behind the broken car, oncoming 1.8 m clear, moved 2.1-2.2 s after | 0.013 / 0.020 m, 0 | G1-G3 0, G4 agree | PASS |
| incident 5, cross turn | 2/2 PASS | PASS: yielded 1.5-1.6 s, crossing car 2.0 m clear | 0.013 / 0.019 m, 0 | G1-G3 0, G4 agree | PASS |
| `UC-ACC-001-0001`, 1 variant (8.333 / 5.556 -> 1.944 m/s) | 2/2 PASS | | `Npc1` 0.013 / 0.278 m (spawned moving), 0 | G1-G3 0 (1 info), G4 agree | PASS |

SSv2's verdict agrees with CARLA's on all four. Incident 1 first failed twice on the 300 s
timeout: the ego stopped at the Town01 junction at (76, -331) and never moved on. Step 4's
signal code was already in this build, and every lamp started dark, which reached Autoware
as UNKNOWN on every group -- a stop line. The CARLA backend sets every mapped light GREEN
before a scenario commands any (`set_mapped_lights_green`); the world server now does the
same (B's "off by default" is changed to that), and incident 1 passed 2/2.

## Step 4: B

**Plan** (2026-10-22). What the survey of Town01's export found first: each of the 36
`Root/Intersections/TrafficLight*` prims carries `omni:simready:signalID` = its OpenDRIVE
signal id (375, 377, ...) and `signalOrder` `[red, amber, green]`, and sits at CARLA's
actor position with Y flipped (`TrafficLight11` at (102.72, -52.85), the table's `377` at
CARLA (102.72, 52.85)). Its bulbs are `Scene/Cylinder/GreenLight`, `.../YellowLight`,
`.../RedLight`, nested; each has `SM_TLights_Leds/Section1`, the lens, bound to CARLA's
`MI_StreetLightsOff.mdl` with `LightColor` (1, 0.016, 0) where lit (every red, as exported)
and 0.001 grey where not.

- **Wire** (`csb_world.proto`): `LoadWorld.signals` -- the map's signal table as csb reads it
  from `<map>/carla/traffic_lights.yaml` (way id, regulatory element ids, OpenDRIVE id,
  position in the world frame); `SetSignals { repeated SignalState { opendrive_id, colour } }`
  (`Colour`: `OFF`, `RED`, `AMBER`, `GREEN`). Field and message additions; protocol 3.
- **csb**: `IsaacBackend::initialize` sends the table with `LoadWorld` (none when the map has
  no table: a scenario with no lights still runs); `update_traffic_lights` reduces each SSv2
  signal to one colour as the CARLA backend does (a lit red wins, then amber, then green;
  arrows flattened) and sends the changes as `SetSignals`, applied at the next step. An
  unmapped way id is warned once, as on CARLA.
- **World server**: at `LoadWorld` each table entry is matched to a lamp by `signalID`, else
  by position (nearest within 1.5 m), logged matched / unmatched. Each lens is bound to one
  of four materials made from the export's own (`MI_StreetLightsOff.mdl`, `LightColor`
  red / amber / green / off); every lamp off at load. The bulbs leave the collider layer
  (`physics_layer` skips `GreenLight`, which holds all three).
- **To Autoware**: the world server hands aib's `EgoRos` the state each step;
  `EgoRos.publish_signals` publishes `autoware_perception_msgs/TrafficLightGroupArray` on
  `/perception/traffic_light_recognition/external/traffic_signals` in the ego's domain, one
  group per regulatory element (the most restrictive colour of its lights, a solid circle at
  confidence 1, as acb's publisher), every frame the ego publishes. aib builds
  `autoware_perception_msgs` for Isaac's Python (`build_msgs.sh`).
- Runs: `town01_traffic_light` and incident 3 on Isaac.

**Results** (2026-10-22, server139, the same stacks as step 3's incident 1 reruns).

- **Lamps**: 36 of 36 table signals matched on Town01, all by `signalID` (108 lenses). A
  standalone check bound the four materials on `TrafficLight11` (OpenDRIVE 377) and rendered
  it from 4 m: dark, red, amber, green as commanded
  (`/mnt/wd/aeon/csb-044/town01_signal_lamps.png`). `physics_layer` now leaves the heads out
  (Town01 5943 -> 5727 colliders, Town02 3732 -> 3588; Town03/05/10HD name their lamps
  otherwise and have no signal table).
- **To Autoware**: aib `defb5d0` (`EgoRos.publish_signals`, `aib.signals`,
  `autoware_perception_msgs` built for Isaac's Python); the topic has the world server as its
  one publisher and `traffic_light_arbiter` as its subscriber in domain 1.

| scenario | Isaac: SSv2 verdict | judge / oracle | CARLA (033 step 5, 032 step 7) |
|---|---|---|---|
| `town01_traffic_light` | 2/2 PASS (3409, 3394 frames) | G1-G3 0, G4 agree | PASS (3441 frames) |
| incident 3, red light | 2/2 PASS | stopped 0.17 / 0.27 m before the line for 8.1 s, moved 1.6 / 1.5 s after green | PASS |

## Step 5: D

**Plan** (2026-10-22).

- **Two-wheelers**: `vehicles.json` has 7 (`base_type` bicycle 3, motorcycle 4); their
  exports are 78-253 MB with four UsdSkel prims each and a render width of 1.95 m, so the
  rider is in them, skinned; the check is a render of each kind: does the rider sit, or
  stand in its bind pose. Their CARLA boxes in the export's metadata are degenerate (x
  extent 0), so `nearest(dims, subtype)` picks among the two-wheelers of the box's kind
  (SSv2's `bicycle` / `motorcycle`) by length alone. Tracked, they get 200 kg (CARLA's
  bicycles weigh 1 kg in `physics.json`).
- **MiscObjects** wear their model: csb sends the asset key (`model3d`) for them too, CARLA's
  default `static.prop.streetbarrier` when it names neither a vehicle nor a prop; the world
  server puts a vehicle's body on a vehicle model (incident 2's parked van and car) and a
  prop's on a prop, its CARLA box centred on the declared box, bottom on bottom.
- **Props export**: `isaac_vehicles.sh props:=true` (`just isaac-vehicles props:=true`):
  `ue_vehicle_levels.py` with `CSB_VEH_KIND=props` makes one level per `static.prop.*` of
  the editor's `Default.Package.json` (its static mesh alone at the origin, the mesh bounds
  as its CARLA box), the same SimReady commandlet exports them, `prop_usd_assets.py` wraps
  them and writes `$CSB_VEHICLE_USD/props/props.json`. Crash uploads stay off (the engine
  ini and the `-ini:` override, 043).
- Runs on Isaac: incident 6 (the cyclist), incident 2 (its parked cars; its pedestrian is a
  box until C), `town01_pedestrian` (its barriers).

**Results** (2026-10-22, server139).

- **Props export**: `just isaac-vehicles props:=true`: 98 props in `Default.Package.json`, 97
  exported (`Bike Helmet` has a space in its id, which no shell word, path or SimReady map
  name takes: skipped and logged); levels and export 4 min, `props.json` beside them
  (`static.prop.streetbarrier` 1.22 x 0.37 x 1.07 m). No crash report written; the engine
  ini keeps `bSendUnattendedBugReports=False` / `bSendUsageData=False` and
  `DataRouterUrl=""`. Found on the way: an export with `export:=true` over levels an earlier
  run made exported the old levels (the editor cannot delete them: "exists but is not
  accessible"); `isaac_vehicles.sh` now removes the `.umap` files it is about to make again.
- **MiscObjects**: incident 2's `vehicle.mercedes.sprinter` and `vehicle.tesla.model3` wear
  those bodies, `town01_pedestrian`'s barriers the street barrier, a cone its cone
  (`/mnt/wd/aeon/csb-044/misc_objects.png`).
- **Two-wheelers: the rider is there but not seated.** Each of the 7 exports has two
  skeletons, the bike's and the rider's (`SK_Female`, `SK_euroM_` ...); the rider renders in
  its bind pose, arms out, standing over the bike (`two_wheelers_bind_pose.png`). In CARLA
  the rider is posed by `AB_Biker` (IK to the handlebars and pedals), which an export does
  not run. Two tries to export a pose failed, and were taken out again: the rider component
  set to single-node animation with CARLA's seated `PedestrianOnSidecar` (SimReady exports a
  single-node animation, `ExportAnimationAssetAsUSDToPathInternal`, but the blueprint's
  component kept its anim blueprint through the export's load: no `SkelAnimation` in the
  USD), and a separate `SkeletalMeshActor` for the rider (not in the export either). The
  rider stays as exported: it is visual only (the collider is the declared box), and the
  lidar still sees a person-sized shape on the bike. **Open**, for the user: a seated rider
  needs a pose exported from the editor (C's walker work may give the means).
- `i6_cyclist` (1.66 x 0.6 x 1.62 m box, no model) wears `vehicle.diamondback.century`, the
  bicycle nearest in length, tracked at 200 kg on 0.15 m skids.

| scenario | Isaac: SSv2 verdict | judge / oracle | CARLA (033) |
|---|---|---|---|
| incident 6, cyclist inside the turn | 2/2 PASS | ego yielded (stood 1.5 / 3.0 s), cyclist 0.13 m at the closest; cyclist error p95 0.028 m | PASS 3/3 on centerpoint with the 042 overlay |
| incident 2, pedestrian dart (parked van and car) | 1/1 PASS | ego stopped 4.1 s, pedestrian (a box) 5.2 m away | PASS |
| `town01_pedestrian` (barriers) | 0/1: timeout | the ego touched the pedestrian box at frame 373 and never reached its goal | PASS |

`town01_pedestrian` is C's: its pedestrian is still 037's kinematic yellow box, which the ego
ran into; the barriers did their part. It is rerun in step 6.

## Step 6: C

**Plan** (2026-10-22). Spike first, time-boxed to about half a day.

- **CARLA's walkers through SimReady.** SimReady can export animation: its level export writes
  a `SkelAnimation` for a skeletal mesh component that plays an animation asset in
  single-node mode (`FSimReadyUSDHelper::ExportAnimationAssetAsUSDToPathInternal`). The spike:
  per walker (`WalkerFactory` definitions; `walker.pedestrian.0001` first), two levels with a
  plain `SkeletalMeshActor` of its mesh at the origin playing its walk and its idle sequence
  (CARLA's `GEN3/AS_walking04_G3` / `AS_idleAXz02_G3`, or the Gen2 female / male cycles, as the
  skeleton allows), exported by the same commandlet. It works when the USD has a
  `SkelAnimation` bound to the walker's skeleton and Isaac renders the walker in stride.
- **Playing it at SSv2's speed** without Isaac's timeline (one clock for every walker, and
  the world server's own): each walker gets its own `SkelAnimation` prim whose joint
  rotations and translations are set each step from the exported walk (or idle, below
  `WALKER_SPEED_FLOOR`) sampled at the walker's own phase, advanced by `speed / v0` (v0 the
  walk's own speed). `omni.anim.graph` is the alternative if writing the joints per step
  costs too much.
- **Fallback**, if CARLA's animation does not come through: Isaac's own characters
  (`Isaac/People/Characters`, Actor SDG's assets) chosen by the box's height, with their
  walk and idle, played the same way.
- Either way the collider stays the declared box, the walker gets a `model` and a
  `walkers.json` index as vehicles do, and a standing walker faces SSv2's heading. Runs:
  incident 2, `town01_pedestrian`, `UC-AEB-001`.

**Spike result: CARLA's walkers come through; no fallback.** SimReady's level export writes
the single-node `SkelAnimation` (female walk 41 samples at 30 fps over 66 joints, G3 walk 73
over 26; idles 591 and 791). The cycles are in place (the root joint does not move), so a
walker is played, not driven by them. Isaac renders a referenced walker with its skeleton
bound to a per-walker `SkelAnimation` whose default rotations and translations are written
each frame: correct pose, ~1 ms for three walkers' writes through `Vt.*Array.FromNumpy`
(`omni.anim.graph` is not needed). The export faces -y (CARLA's blueprint turns its mesh
-90 degrees): the walker goes on turned +90 degrees about z, feet on the declared box's bottom.

**What `walker.pedestrian.NNNN` is, is not readable.** WalkerFactory builds its definitions
in its Blueprint graph (`MakePedestrianDefinitions`); its CDO has no `Parameters` /
`ParameterArray` property and `GenerateDefinitions()` returns none in a commandlet. So a
walker id picks among the exported walkers by its number (`csb_isaac.walkers.pick`: stable,
spread), a pedestrian with no CARLA key by its name; a key that is an exported walker
(`bp_walker_*`) is that walker. Three were exported at first (all 52 since; open item 3) (`bp_walker_female1_v1`,
`bp_walker_maleeuro`, `bp_walker_ab001_g3`, 255 s); `isaac_vehicles.sh walkers:=true` takes
any `BP_Walker*` with a Gen2 or Gen3 cycle for its skeleton.

**Built:**

- `isaac_vehicles.sh walkers:=true` (`ue_vehicle_levels.py` walkers mode: per blueprint a
  `_walk` and an `_idle` level; `walker_usd_assets.py`: `/Walker` wrappers, the cycles as
  `.npz`, `$CSB_VEHICLE_USD/walkers/walkers.json`).
- `csb_isaac.walkers`: `Cycle.sample` (nlerp between recorded frames, shorter arc), `Gait`
  (speed smoothed over 0.2 s; idle below 0.2 m/s; walk at `speed / 1.3 m/s` times its
  recorded rate, at most 2.5x; a change of cycle starts it from the top), `pick`. Unit tests.
- The world: a pedestrian's box (kinematic, collider) goes invisible under its walker; each
  step the walker's speed is its pose's ground-plane change over the frame. csb sends a
  pedestrian's asset key as `model` when it names a `walker.*`.

Measured in an empty world (`spike_walkers2`, three pedestrians at 1.3, 0 and 2.5 m/s, 90
frames of 2 substeps): step p50 1.10 ms, p95 1.95; gaits walk / idle / walk. Picture:
`/mnt/wd/aeon/csb-044/walkers_in_isaac.png`.

**Results** (2026-10-22, Isaac, Town01, the car-bodied ego with CenterPoint at
`max_velocity:=11.11`; runs under `/mnt/wd/aeon/csb-044/runs/s6-*`):

| Scenario | SSv2 | Judge / oracle | CARLA |
|---|---|---|---|
| incident 2 (`town01_i2_ped_dart`) | 2/2 PASS | judge PASS 2/2: stopped, walker 2.14 / 5.11 m (origins), waited 7.2 / 5.8 s; 0 contacts | PASS |
| `town01_pedestrian` | 5/5 PASS | oracle `PASS+contact` 4/5: the ego touches the walker | PASS, stops 4.66 m short (014) |
| `UC-AEB-001` (vxi_ego 8.333, dxi 30) | 2/2 PASS | stopped 9.0 m (origins) short; 0 contacts | PASS |

The walker wears `bp_walker_female1_v1` (`walker.pedestrian.0001`) in all three.

**`town01_pedestrian`'s contact is perception, not the walker.** The ego meets the walker
while it is still crossing into the lane (0.69 m/s). Logged from the ego's domain
(`/perception/object_recognition/objects`, `/api/planning/velocity_factors`, runs 3-5):
over the run CenterPoint calls the walker `pedestrian` (1622-1667 messages) but on the
approach, 20-30 m out, `unknown` (27-30), so no crossing is predicted; the first stop factor
is `route-obstacle` at 3.0 m once the walker is inside the ego's path, too late from 8.4 m/s.
Step 5's run with the plain yellow box failed the same way (and then SSv2 failed too), so
the walker does not cause it. Why CenterPoint loses the class at that range on Isaac's lidar
and not on CARLA's is left open (the lidar's returns off a skinned mesh, intensity).

## Step 7: F

**Plan** (2026-10-22). One run of every acceptance scenario on each backend with the final
code (steps 3-6 ran the A-D scenarios twice each as they were built), SSv2's verdict side by
side, and the oracle's physical verdict. One backend at a time.

1. **Isaac, Town01** (the stacks steps 3-6 used: CenterPoint ego at `max_velocity:=11.11`):
   `basic/town01_*` except `town01_simulator_autopilot`, `demo/incidents/*`, `UC-ACC-001`
   and `UC-AEB-001` (one variant each, as in steps 3 and 6).
2. **Isaac, the other towns**: Isaac stacks stopped, then `town02_episode_change`,
   `town02_ego_drive`, `town03_ego_drive`, `town05_ego_drive`, `town10_ego_drive`, each with
   `just scenario <xosc> backend:=isaac` bringing up the town's stacks (`isaac_up.sh`), which
   are stopped before the next town.
3. **CARLA**: Isaac stacks stopped and reaped, our CARLA simulation side back on 8084
   (`just run weather:=ClearNoon`), then the same list on `just ego-av` (Town01; for the
   other towns an ego stack on that town's map), the 043 routes scored for tracking with
   `just acceptance <xosc> 1` as 043 scored them on Isaac. `town10_ego_drive` is the first
   CARLA run through the `Town10: Town10HD` alias.

**Found on the way: despawned entities stayed in the lidar's world.** The first Town01
batch failed `town01_ego_drive`, `town01_engage_state` and `UC-ACC-001`, all three because
Autoware stopped for obstacles nobody had spawned: `route-obstacle` stop factors, and
perceived objects standing exactly where an earlier scenario's entities had ended
(`town01_rear_contact`'s `npc_rear` at (187.6, -130.3), `UC-ACC-001`'s `Npc1` at (201.4,
-133.4), `town01_pedestrian`'s `barrier_1` at (120, -129.8)). A fresh world server passed;
`town01_rear_contact` then `town01_ego_drive` reproduced it. The deleted prims were gone
from the stage, PhysX overlap queries at those places found only road, and an RGB camera
did not see them: only the RTX lidar kept them. Fix (`57950ce`, `7eb38b8`): `despawn` makes
the body kinematic, moves it 1000 m under the world and makes it invisible, and deletes it
after the next step. After it: `rear_contact` -> `ego_drive`, `pedestrian` -> `ego_drive`
and `episode_change` -> `ego_drive` all pass. Also fixed: a `LoadWorld` of the world already
loaded no longer leaves the lamps uncoloured. Steps 3-6 ran each scenario on a server that
had run others before, so some of their runs carried ghosts too; none of their verdicts
needed one.

**Results** (2026-10-22, one run each with the final code unless noted; runs under
`/mnt/wd/aeon/csb-044/runs/`, `s7j-*`/`s7k-*` Isaac, `c7-*` CARLA). Town01 on the demo ego
stack (CenterPoint, `max_velocity:=11.11`) on both backends; the other towns on the default
ego stack, as 043 ran them. "Oracle" is the gap oracle's physical verdict.

| Scenario | Isaac SSv2 | Isaac oracle | CARLA SSv2 | CARLA oracle |
|---|---|---|---|---|
| `town01_ego_drive` | PASS (+ after `rear_contact`, `pedestrian`, `episode_change`) | agree | PASS (+ after `episode_change`) | agree |
| `town01_engage_state` | PASS | agree | PASS | agree |
| `town01_rear_contact` | PASS | agree, 38 contacts | PASS | agree |
| `town01_traffic_light` | PASS | agree | PASS | agree |
| `town01_pedestrian` | PASS (6/6 over steps 6-7) | PASS+contact | PASS | PASS+overlap |
| `town02_episode_change` | PASS | agree | PASS | agree |
| incident 1-6 | PASS 6/6, judge PASS 6/6 | agree, 0 contacts | PASS 6/6 | agree |
| `UC-ACC-001` (1 variant) | PASS (s7j; 2/2 in step 3) | agree | **FAIL 0/3** (accel -1.51, -1.50, -1.52 < -1.5); default stack PASS 2/2 | agree (fail_other) |
| `UC-AEB-001` (1 variant) | PASS (+2/2 step 6) | agree | PASS | agree |
| `town02/03/05/10_ego_drive` | PASS 4/4 | | PASS 4/4 (`just acceptance`, below) | |

- **`town01_pedestrian`'s contact is the scenario's, not Isaac's**: on CARLA the ego meets
  the crossing walker at 10 m/s too (5 contacts, frames 418-423, walker at y -130.6 entering
  the lane), and SSv2 passes it there as well. Step 6's perception reading stands (the
  walker is `unknown` on the approach) and is no longer an Isaac difference.
- **`UC-ACC-001` on the CenterPoint stack** is the one disagreement: CARLA's ego brakes
  0.00-0.02 m/s^2 past the scenario's -1.5 m/s^2 comfort limit in all three runs; on the
  default stack it passes 2/2 on CARLA, and Isaac passes it on the CenterPoint stack.
- **043's routes on CARLA** (`just acceptance <xosc> 1`, acb's acceptance.py, default ego
  stack on the town's map): all PASS, tracking of the order Isaac measured in 043.

| Route | CARLA | longitudinal, m/s^2 | cross-track, m | localization, m | Isaac (043, medians) |
|---|---|---|---|---|---|
| `town02_ego_drive` | PASS | 0.003 | 0.153 | 1.25 | 0.017 / 0.01 / 1.35 |
| `town03_ego_drive` | PASS | 0.027 | 0.088 | 0.89 | 0.025 / 0.01 / 0.95 |
| `town05_ego_drive` | PASS | 0.012 | 0.150 | 0.73 | 0.028 / 0.03 / 0.71 |
| `town10_ego_drive` | PASS | 0.020 | 0.153 | 0.65 | 0.031 / 0.04 / 0.42 |

`town10_ego_drive` on CARLA is the first run through the `Town10: Town10HD` alias: csb
logged `Loading map 'Town10HD' (CARLA currently holds 'Town05')` and `Map 'Town10HD'
loaded`. Cross-track on CARLA is 0.09-0.15 m against Isaac's 0.01-0.04, inside acceptance.py's
1.0 m.

## Open items (2026-10-22, after step 7)

1. **`UC-ACC-001` on CARLA, CenterPoint stack** (the demo tour's: CenterPoint, tracker merger
   off): FAIL 0/3, peak deceleration -1.51 / -1.50 / -1.52 m/s^2 against the scenario's -1.5
   limit. The default stack passes 2/2 on CARLA, and Isaac passes on CenterPoint. The only
   place SSv2's verdict differs between backends. Taken first. **Closed (user, 2026-10-22):
   a known limitation of the CenterPoint stack against this scenario, not a simulator gap;
   no scenario variant, no Autoware tuning.**

   **Finding (2026-10-22): Isaac's pass is not a pass.** On Isaac the ego stops dead the
   moment `Npc1` spawns: 8.32 -> 0.19 m/s in one 50 ms frame, its position continuous
   (`s7j-acc` frame 357; `s3-acc-*`, `s7i-acc*` the same). It then starts again from rest
   behind an NPC that has already slowed to 1.9 m/s, and never closes on it at speed (peak
   deceleration -0.6 to -0.9). On CARLA the ego meets `Npc1` at 8.3 m/s, 29 m behind it at
   5.5 m/s, and brakes at -1.5 to -1.6. Every Isaac run with a mid-run spawn has it: incidents
   1, 2, 4, 5, 6, `UC-AEB-001` (`Pedestrian0`), `UC-ACC-001`; no CARLA run does. So the A, C,
   D and F verdicts for those scenarios on Isaac were taken with an ego that had been stopped
   by the spawn.

   Cause, isolated headless (aib's ego on the flat world, driven to 9.7 m/s, one edit, no
   stack): **a prim with a USD reference composed in** -- a catalogue body (car, walker,
   prop), or a trivial `.usda` with one cube -- makes PhysX rest every PhysX vehicle when the
   change reaches it (`flush_changes`, or the next simulate): the chassis' linear velocity
   goes to 0 and its wheels keep spinning (26.6 rad/s, i.e. 9.8 m/s). A prim defined in
   place (a plain cube, a kinematic box with no body) does not; a free dynamic rigid body
   keeps its velocity; opening the layer beforehand (`Sdf.Layer.FindOrOpen`) does not help,
   nor does writing the chassis' USD velocity. Writing the chassis' velocity back through
   the tensor API after the flush does: 9.85 -> 9.90 m/s, no dip.

   Fix (`IsaacWorld._egos_keep_motion`): every structural edit of the world server's stage
   (`spawn`, `_bury`) is handed to PhysX on the spot by a 1 us `update_simulation` and each
   ego's chassis velocity is put back through the tensor API. (`flush_changes()` hands it
   over too, but after a walker's spawn the next step crashed Kit: "terminate called without
   an active exception".) Headless, the ego held 9.74 m/s through a tracked car, a walker, a
   kinematic car and their despawn and burial.

   Isaac again, the step 7 stacks (CenterPoint, `max_velocity:=11.11`), 2026-10-22 (`fx-*`):
   no spawn drop in any run.

   | Scenario | SSv2 | peak deceleration (trace, median of 3) | oracle |
   |---|---|---|---|
   | `UC-ACC-001` (1 variant) | 1/3 (`fx-acc-1..3`: FAIL, FAIL, PASS) | -1.76, -1.48, -1.46 | agree |
   | `UC-AEB-001` (1 variant) | PASS | | agree, 0 contacts |
   | incident 2 | PASS, judge PASS | | agree, 0 contacts |

   So **`UC-ACC-001` on the CenterPoint stack fails on both backends** (CARLA 0/3, Isaac 1/3).
   Not a gap between the simulators any more; what fails is the scenario against Autoware on
   this stack, two ways (`c8-acc-1` on CARLA with a bag of perception, planning and control,
   `/mnt/wd/aeon/csb-044/bags/acc-carla-1`):

   - **The spawn's braking peak.** SSv2 spawns `Npc1` 29-30 m ahead (5.5 m/s) when the ego,
     at 8.3 m/s, reaches the start position. For its first 0.3-0.5 s the tracker has it as an
     UNKNOWN object at 0 m/s, Autoware plans a `route-obstacle` stop 20.6 m out and the command
     goes to -0.69 then -1.1 m/s^2; the tracker has 5.5 m/s (a car) by +0.58 s, but the command
     stays near -1.1 for another second. Autoware never commands below -1.13; the plant and
     the measurement put SSv2's reading at -1.46 to -1.76 (CARLA's PhysX acceleration, median
     of 3 frames; Isaac's from aib's position differences), on the -1.5 limit, 1.2-1.5 s after
     the spawn every time (`c7-acc*`, `fx-acc-1`).
   - **Missing the success window.** Success is the ego at lane 5230 s=170 (x 270.4) *while*
     matching `Npc1`'s speed (relative speed < 0.05), and `Npc1` only slows to 1.94 m/s once
     the ego has first matched its 5.5. An ego that settles onto 5.5 late meets s=170 still
     closing (3.5 m/s against 1.9), the trigger misses, it drives on to Autoware's goal at
     x 301 and stands there: 60 s standstill (`fx-acc-2`) or the 180 s limit (`c8-acc-1`).

   The default stack's passes on CARLA do not test this: the scenario's ego speed is 8.333
   m/s (`vxi_ego`), the default `max_velocity` is 4.17, below `Npc1`'s 5.5, so that ego never
   closes on it, and `Npc1` slows at once. The other step 3-7 Isaac verdicts for scenarios
   with a mid-run spawn (incidents 1, 2, 4, 5, 6, `UC-AEB-001`) were taken with the ego
   stopped by the spawn. All pass again with the fix (`fx-i1`, `fx-i2`, `fx-i4`, `fx-i5`,
   `fx-i6`, `fx-aeb`: SSv2 PASS, judge PASS, oracle agree, 0 contacts, no spawn drop).
2. **The two-wheeler rider stays in its bind pose** (step 5, `two_wheelers_bind_pose.png`).
   **Closed 2026-10-22: seated, below.** Was:
   two UE attempts reverted. Next to try: the walkers' route (a plain `SkeletalMeshActor`
   playing the rider's seated sequence, exported as a `SkelAnimation`).

   **Plan** (2026-10-22). Found first (UE probe, `/mnt/wd/aeon/csb-044/riders/`): every
   rider (`SK_Female`, `SK_femaleAfro_`, `SK_euroM_`, `SK_euroM_v2_`) is on CARLA's Gen2
   skeleton `SK_Pedestrian_Generan` (66 `crl_*` joints), and **no seated sequence exists for
   it**: `PedestrianOnSidecar` has no skeleton at all (Mixamo bone names, which is why step 5's
   try exported nothing), `HandBikePose` and `man_manosMoto` are on the old `man_xfull`
   skeleton. The one biker asset on the rider's skeleton, `AS_Pedestrian_BikeHands`, exports
   through the walkers' route (a `SkeletalMeshActor` level, `SkelAnimation` of 126 frames) but
   is static and differs from the rest pose only in the hands (wrists and fingers: the grip).
   So `AB_Biker` is that grip plus IK, and the rest of the rider's component sits about 1 m
   behind the bike (the anim blueprint moves the pelvis onto the saddle). The bike's own
   skeleton carries the IK targets: `Seat`, `HandlerLeftSocket` / `HandlerRightSocket`,
   `LeftPedalGeo` / `RightPedalGeo`.

   So the rider is posed offline, once per two-wheeler, as `AB_Biker` would:
   - `isaac_vehicles.sh riders:=true`: the grip exported from the editor (the walkers' route),
     kept as `$CSB_VEHICLE_USD/riders/grip.npz`.
   - `csb_isaac.rider` (numpy, unit-tested): from the rider's rest pose, the hips onto the
     saddle, the spine leant forward until both hands reach the bars, two-bone IK of each arm
     to its handlebar socket and each leg to its pedal (knees forward, elbows down and out),
     feet kept level, the grip on the hands.
   - `vehicle_usd_assets.py` writes the pose as a static `SkelAnimation` in each two-wheeler's
     wrapper (`/Vehicle/RiderPose`, bound to the rider's `SkelRoot`), so the world server
     needs no pose code. Collider unchanged (the box).
   - Check: a headless render of all seven (`two_wheelers_seated.png` beside
     `two_wheelers_bind_pose.png`), then incident 6 once on Isaac.

   **Result** (2026-10-22): **the riders sit.** `just isaac-vehicles riders:=true` (levels and
   export about 3 min, then every wrapper written again) seats all seven:
   `/mnt/wd/aeon/csb-044/two_wheelers_seated.png` (side on, against
   `two_wheelers_bind_pose_all.png`), `two_wheelers_seated_quarter.png` and
   `two_wheelers_seated_world.png` (spawned through `IsaacWorld.spawn`, as a scenario's
   cyclist is). Hips on the saddle, hands on the grips with the grip pose, feet on the pedals
   or pegs, head up. Leans chosen: city bikes 10-40 degrees with no slide, Harley 14, Vespa 30,
   the road bike, Ninja and YZF 48-60 degrees with the hips 0.15 m forward (their bars are low
   and far); `vehicles.json` keeps them per entry (`rider_pose`).

   - **A SkelAnimation inside an instance is not applied**: with the body instanceable the
     rider stayed in its rest pose. A two-wheeler with `rider_pose` is therefore not
     instanceable (`vehicle_assets.instanceable`); cars and props still are.
   - Static: the legs do not pedal and the bars do not steer (CARLA's AB_Biker animates both).
     Visual only; the collider is the declared box, as before.
   - Also removed: the leftover of step 5's attempt in `ue_vehicle_levels.py` (the vehicles
     mode still referenced an undefined `RIDER_ANIM` for two-wheelers, a `NameError` the next
     time the vehicles' levels were made).
   - Incident 6 on Isaac **not run yet**: another workload held 13 GB of the GPU for hours
     (15.5 GB free, under the 16 GB the Isaac stacks need). Instead, headless through
     `IsaacWorld`: incident 6's cyclist box (1.66 x 0.6 x 1.62 m, bicycle, TRACKED) wears
     `vehicle.diamondback.century` with its rider seated, not instanceable, and tracks 4 m/s
     over 150 frames (error p95 0.016 m, step p50 2.0 ms with a render every third frame),
     then despawns cleanly.
   - **Incident 6 on Isaac with the seated rider (2026-10-23): PASS.** SSv2 PASS, judge PASS
     (the ego stood 1.4 s and yielded, no contact, cyclist 0.17 m at the closest), gap oracle
     G1-G3 0, G4 agree, 1071 frames; the ego stack as the tour runs it, the world server
     sharing the GPU with other work (`/mnt/wd/aeon/csb-044/runs/w-i6-1`).
   - **Incident 2 on Isaac with walkers (2026-10-24): first run FAIL, rerun PASS.** The
     first run ended at frame 309: the van, the parked car and the pedestrian spawned, and the
     next Step took over 40 s while Isaac compiled their MDL materials for the first time
     (the pedestrian, `i2_ped` with no model, wears `bp_walker_maleamer_cop`: MetaHuman eyes,
     tear film, ...), past the bridge's 10 s `request_timeout_seconds`; the bridge exited 75
     and SSv2 ended the scenario. The rerun on the same world server, the materials now
     compiled, passed: SSv2 PASS, judge PASS (the ego stood 7.5 s, pedestrian 4.49 m away),
     G4 agree, 0 contacts (`runs/w-i2-1`, `w-i2-2`). **Open**: a first spawn of a model whose
     materials this world server has not compiled can stall Step past the timeout.
     **Fix (2026-10-24, design):** the compile moves into `Spawn`, which the bridge now waits
     on for `load_timeout_seconds` (120 s) for every entity, not only an ego. The world
     server renders once (`world.render()`, no physics step, as `preload` does for a
     town) after it puts on a catalogue body (vehicle, prop or walker) it has not rendered
     before in this process, and logs how long it took; a body it has rendered answers as
     before. So a Step stays bounded by the 10 s `request_timeout_seconds` and a real hang
     is still caught there. Compiled materials are not kept between world-server starts
     (Isaac's cache holds 2.4 MB), so warming the whole catalogue at preload would cost
     every start, including the respawn after an OOM kill, for models most scenarios never
     use; it was not taken. SSv2 bounds each request at `SIMULATOR_RESPONSE_TIMEOUT` (90 s),
     so a body that compiles longer than that still ends the scenario, from SSv2's side.
     Simulated time does not move while a spawn compiles; whether the ego stack's
     wall-clock timers notice the pause is checked by the first run with a body this world
     server has not rendered.
3. **`walker.pedestrian.NNNN` -> blueprint** (step 6 found it unreadable from the editor:
   WalkerFactory builds it in its Blueprint graph, so `csb_isaac.walkers.pick` chose by the
   id's number). **Read from the package instead (2026-10-22).** The graph compiles each
   `FPedestrianParameters` to bytecode in `WalkerFactory.uasset`: `EX_StringConst` with the id
   (`"0001"`), `EX_ObjectConst` with the class (an import), `EX_ByteConst` gender and age.
   `scripts/walker_factory_ids.py` parses the package's name and import tables (UE 4.26,
   version 522: 36-byte imports, the editor-only `PackageName` included) and that bytecode:
   52 ids, `0001` = `BP_Walker_Female1_v1` ... `0052` = `BP_Walker_AsiaM03_G3`, every one a
   distinct blueprint; CARLA's catalogue agrees where it says (0009 and 0012 children, 0030
   the police officer, 0049 a child). The table is `csb_isaac/walker_blueprints.json`.
   `pick`: the id's own blueprint when it is exported; else an exported walker of the same
   gender and age, then the same age; else the numeric spread as before.
   **All 52 exported (2026-10-23):** `just isaac-vehicles walkers:=true only:=<the 49
   missing>`, 1420 s of SimReady export, 14 GB in `$CSB_VEHICLE_USD/walkers/`; `walkers.json`
   has 52, `ue_meta.json` no error, every id's `pick` is its own blueprint. The 8 children are
   1.22-1.35 m. Not yet seen rendered on Isaac (GPU held by other work).
   The whole set (towns, vehicles, props, walkers) now moves as one directory: relative
   paths in its indexes, a `MANIFEST.sha256`, `just isaac-assets check|manifest`, obtainable
   from NEWSLab (docs/design/isaac-assets.md, 2026-10-23).
4. **Perception on the CenterPoint stack**: the crossing walker is `unknown` on the approach
   (20-30 m), so no crossing is predicted, on both backends; `town01_pedestrian` passes in
   SSv2 while the ego touches the walker (Isaac 4 of 5, CARLA too). The scenario's and the
   ego config's, not Isaac's.
5. **CARLA free-runs while our simulation side is stopped**: the server goes asynchronous
   at ~79 % GPU. Harmless to runs, costly to a host shared with Isaac; left alone.
6. **A transient colcon failure**: `acb_bridge` once failed with "no matching package
   `autoware_adapi_v1_msgs`"; a plain retry passed. Not reproduced.

## Acceptance

- `town01_traffic_light`, `basic/*` (except `town01_simulator_autopilot`, deferred),
  `demo/incidents/*`, AWF `UC-ACC-001` and `UC-AEB-001` pass on Isaac, with SSv2's verdict
  agreeing with CARLA's for each.
- 043's routes on Town02, Town03, Town05 and Town10 (`basic/town0X_ego_drive`, `town10_ego_drive`)
  run on CARLA too, with tracking of the same order as on Isaac. That includes the
  `Town10: Town10HD` alias in `bridge_config.yaml`, which has not been tested on CARLA.
- CARLA and Isaac never run at the same time (GPU and RAM): one backend's batch ends, and its
  stacks are stopped, before the other's starts.
