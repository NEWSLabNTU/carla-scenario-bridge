# Phase 043: Single-Vehicle End to End on Isaac Sim

One Autoware ego on Isaac Sim with the CARLA experience: CARLA's own towns and CARLA's own car,
driven through every scenario that needs no NPC and no traffic light, by the same recipes as
on CARLA, repeatably and unattended. Stage 1 of three toward the full SSv2 + Isaac + Autoware
stack ([design/isaac-stack.svg](../design/isaac-stack.svg)): this, then NPCs and traffic
lights ([044](044-isaac-npcs-and-traffic-lights.md)), then several vehicles, V2X and the gap
campaign ([039](039-isaac-multi-autoware.md)).

**Source**: user direction 2026-10-18 ("finish single vehicle end-to-end run first, in which
map, vehicle model and driving are done first. We then process npc and traffic lights. The
multiple vehicle and v2x the last.").
**Depends on**: [038](038-isaac-vehicle-bridge.md) (the ego, Town01 as a SimReady world,
CARLA's vehicles in Isaac).
**Feeds**: [044](044-isaac-npcs-and-traffic-lights.md), [039](039-isaac-multi-autoware.md),
[040](040-ntu-offroad-field.md).

![Target stack](../design/isaac-stack.svg)

Source of the diagram: `docs/design/isaac-stack.typ` (`typst compile` it to the SVG). Blue
boxes are this phase.

## Where 038 left it

- Town01 exported with NVIDIA's SimReady converter from a CARLA 0.9.16 editor build
  (`/mnt/disk1/carla-src`, `-run=SimReadyExport`) to `/mnt/disk1/carla-usd/Town01`. The export
  has no physics; a collider layer (`Town01_physics.usda`, sublayering the export and applying
  `PhysicsCollisionAPI` to its meshes) was written once by hand-run code, not by anything
  committed. `csb_isaac.usd_world` writes the world file beside the Autoware map.
- 41 vehicles exported (`/mnt/disk1/carla-usd/vehicles`, catalogue `vehicles.json`); the ego
  wears the Tesla Model 3 body. Its physics are acb's `vehicle_info` geometry plus a mass and
  centre of mass typed into `acb_ego.yaml`.
- `town01_ego_drive` 5/5 on the Town01 world, 0 MRM, tracking within acceptance.py's limits.
  Run by hand: `just isaac-world`, `just run backend:=isaac`, `just ego-isaac`, then
  `ros2 launch scenario_test_runner ...`; `just scenario`, `just acceptance` and `just e2e`
  refuse without CARLA.

## Design

### Map

- **Towns**: Town02, Town03, Town05 and Town10HD exported as Town01 was, one directory each
  under `/mnt/disk1/carla-usd/<Town>`. Each has an Autoware map in
  `data/carla-autoware-bridge/<Town>` already, in the same frame (SimReady converts
  centimetres to metres and flips CARLA's Y).
- **Collider layer as code**: `csb_isaac.physics_layer` writes `<Town>_physics.usda`: a
  sublayer of the export, `PhysicsCollisionAPI` on every mesh (triangle mesh, static), nothing
  copied. The export stays untouched.
- **One recipe**: `just isaac-town <Town>` exports (when the USD is missing), writes the
  collider layer and the world file. Paths come from the environment (`CARLA_SRC`,
  `CARLA_USD`), the town from the argument.
- **Checks per town**: the world loads in the world server, the ego settles on the road at
  the scenario's start pose (z within 5 cm of the lanelet), NDT converges on the town's
  `pointcloud_map.pcd` from the Isaac lidar.

### Vehicle model

- **Physics from CARLA's blueprint**: CARLA's `get_physics_control()` per vehicle (mass,
  centre of mass, drag, per-wheel radius, max steer angle, max brake torque, tire friction,
  suspension) dumped into `vehicles.json` under `physics`. The ego's `VehicleParams` take
  them from the catalogue for the ego's model; `acb_ego.yaml` names the model
  (`model: vehicle.tesla.model3`) instead of typing numbers.
- **One car, two descriptions that must agree**: Autoware plans with acb's `vehicle_info`
  (wheel base, tread, wheel radius, overhangs). The world server compares it with the
  catalogue at start and refuses a mismatch beyond 5 cm with both numbers in the message, so
  a different model cannot be driven with the Tesla's description by accident.
- **Same car, same response**: open-loop step responses (throttle, brake, steer) on CARLA
  and on Isaac from the same commands, compared: speed and yaw rate within 10 % after the
  first second.

### Driving and operations

- **The world server is supervised**: `isaac_world.launch.xml` (csb_launch) runs it under
  play_launch, `respawn`, logs in `play_log/`, a health probe (Hello round trip). `just
  isaac-world` starts that launch; launch args are project settings (`ego:=`, `world:=`,
  `follow:=`), port and display come from the environment (`ISAAC_PORT`, `DISPLAY`).
- **Backend-aware recipes**: `just scenario`, `just acceptance`, `just e2e` take
  `backend:=isaac` (default `carla`): the CARLA precheck becomes a check of the chosen
  backend, the ego stack is `ego-isaac`, the world server is started if absent.
- **The agent waits for slow starts**: acb's `autoware_port.py` gives `initialize` 8 s; NDT on
  a loaded host misses it and the pilot retries in a storm. Raised to 60 s (acb commit, pushed,
  then the pin).
- **Map change**: `town02_episode_change` swaps worlds mid-session. `LoadWorld` of a town that
  was not preloaded compiles its shaders (minutes): the load timeout covers it, and the world
  server preloads every town listed in `world:=` (comma separated).

## Steps

- [ ] Phase docs, target diagram.
- [ ] Collider layer generator, `just isaac-town`; Town02, 03, 05, 10HD exported and their
      world files written.
- [ ] CARLA physics in the catalogue; the ego's params from it; the `vehicle_info` check;
      step-response comparison CARLA vs Isaac.
- [ ] World server under play_launch with respawn; backend-aware `scenario`, `acceptance`,
      `e2e`; acb initialize timeout.
- [ ] Exit runs (below), recorded; screencast of the demo.

## Acceptance

- `town01_ego_drive`, `town01_engage_state`, `town02_episode_change` and one route per
  exported town: 5/5 PASS each on Isaac, 0 MRM, tracking within acceptance.py's limits.
- The same car on the same route on CARLA and Isaac: longitudinal and cross-track tracking of
  the same order (recorded here).
- `just scenario <xosc> backend:=isaac` from a cold host (no world server, no ego stack) to
  a verdict with no other command.

### As built: vehicle

**Basis: a plausible real car, not CARLA parity.** The Isaac ego is meant to be realistic, not
to reproduce CARLA's vehicle (CARLA's grips at 3.5 g and brakes at 700 N m per wheel). A real
powertrain is a separate, later piece; the actuation stays "drive none" (acb's contract: the
requested acceleration and tire angle are realized, `vehicle.py`). `csb_isaac/vehicle_params.py`
(unit-tested in `test/test_vehicle_params.py`) holds the car:

| | value | basis |
|---|---|---|
| geometry | wheel base 3.005, tread 1.667, wheel radius 0.370 m, steer limit 0.70 rad | acb's `vehicle_info` = the CARLA Model 3 body the ego wears. A real Model 3 is 2.875 / 1.58 / 0.334 m (235/45R18); kept the description's so Autoware, the body and the physics are one car |
| mass | 1845 kg | Model 3 Long Range AWD curb mass 1830-1851 kg by model year ([evspecifications](https://www.evspecifications.com/en/model/9f3b3db)) |
| centre of mass | 48/52 front/rear, 0.46 m high: (1.442, 0, 0.46) in base_link | assumption: road-test weight splits around 47-48 % front; a floor battery pack puts the CoM low |
| tires | friction 1.0; lateral stiffness 17 x rest load at 2 rest loads; longitudinal 10 x rest load | dry asphalt; the PhysX vehicle's usual passenger-car tire |
| brakes | 3000 N m per wheel, split by the dynamic axle load (ideal distribution) | a stop is limited by the tires (~1 g), as on a real car, not by brake torque |
| drive | 1500 N m per wheel cap | ~8 m/s^2, beyond what Autoware asks |
| suspension | 1.6 Hz, damping ratio 0.45, 0.2 m travel, per axle on its sprung mass | a sedan's ride; each axle at one frequency, so the car sits level |
| inertia | box of the body, mass/12 x (l^2 + w^2) etc.: yaw 4097 kg m^2 | close to m a b = 4155 kg m^2, the usual estimate for a sedan |

The YAML's `vehicle:` overrides any field by name (empty in the shipped one).

**One car, two descriptions that must agree.** `acb_ego.yaml` names the body (`model:
vehicle.tesla.model3`). `IsaacWorld` compares vehicle_info's wheel base, tread and radius with
the catalogue's (CARLA's wheels, `physics` below) at start, and refuses more than 5 cm with
both numbers:

```
VehicleMismatch: the ego's vehicle description is not vehicle.lincoln.mkz_2020: wheel_base
3.005 m in vehicle_info, 2.860 m in vehicle.lincoln.mkz_2020's physics; wheel_tread 1.667 m
in vehicle_info, 1.593 m in vehicle.lincoln.mkz_2020's physics (tolerance 5 cm). ...
```

The ego wears the YAML's model whatever body the scenario names (a warning says so).

**A braking bug, fixed.** PhysX's vehicle at 1 substep above 5 m/s and a 0.1 m/s active slip
denominator (UE's settings too) went unstable under brake torque: the wheels locked and the car
stopped at the friction limit. A -3 m/s^2 request from 7.4 m/s stopped it in 0.7 s (about
-11 m/s^2). Every Autoware braking command on Isaac before this was realized several times
harder than asked. Now 8 vehicle substeps at every speed and a 4 m/s active slip denominator
(`physx_substeps`, `min_active_slip_denominator`).

**Step responses** (`src/csb_isaac/scripts/step_response/`, Isaac: `IsaacWorld` with the ego
YAML, flat ground, 0.05 s frames of 3 PhysX steps as csb steps it, commands into the ego's
command slot; data in `/mnt/disk1/csb-043-vehicle/out/isaac_real_*`):

| test | asked | Isaac |
|---|---|---|
| accelerate | +1.5 m/s^2 | +1.48 m/s^2 |
| brake from 7.4 m/s | -3 m/s^2 | -3.02 m/s^2, stopped at 8.50 s (CARLA 8.55 s) |
| AEB-like from 15 m/s | -8 m/s^2 | -8.05 m/s^2 (-7.86 mean to a stop in 1.9 s), no lock-up |
| beyond the tires from 15 m/s | -12 m/s^2 | -9.7 m/s^2: friction-limited at ~0.99 g |
| 5 m/s, steer 0.2 rad | yaw rate | 0.332 rad/s; kinematic 0.334; CARLA 0.333 |
| 10 m/s, steer 0.1 rad | yaw rate | 0.326 rad/s; kinematic 0.328; CARLA 0.329 |
| 15 m/s, steer 0.05 rad (3.6 m/s^2 lateral) | yaw rate | 0.245 rad/s; kinematic 0.247 |

The car is close to neutral steer up to 3.6 m/s^2 lateral (0.7 % short of kinematic): a real
Model 3 understeers a little more. Its tires are the stiff end of plausible.

**CARLA's vehicle, for reference** (CARLA 0.9.16 and its UE 4.26 under `/mnt/disk1/carla-src`).
`scripts/dump_vehicle_physics.py` writes `get_physics_control()` for all 41 blueprints into the
catalogue's `physics` and `physics.json` (actor frame, m, Y flipped; wheel positions come back
in world cm). `scripts/ue_vehicle_physics.py`, an editor commandlet script, dumps what the API
does not return (`/mnt/disk1/csb-043-vehicle/ue/ue_physics_all.json`, all 41). What it found,
traced through the source:

- Centre of mass: UE computes the chassis body's from its PhysicsAsset shapes at uniform
  density, then adds `COMNudge` (`Engine/Source/Runtime/Engine/Private/PhysicsEngine/BodyUtils.cpp:159`),
  and `get_physics_control().center_of_mass` is only that nudge
  (`carla/.../Vehicle/CarlaWheeledVehicle.cpp:464`). The Tesla's actual CoM, read from the body:
  (0.336, 0, 0.412) m from the actor origin, nudge (0.45, 0, -0.30) included = 1.72 m ahead of
  the rear axle (57 % front) and 0.47 m above the ground. Yaw inertia 2415 kg m^2 (the body's,
  scaled to 1845 kg).
- Suspension: sprung masses from `PxVehicleComputeSprungMasses`, spring = w^2 m_sprung
  (`PhysXVehicles/.../WheeledVehicleMovementComponent.cpp:449,460`); the Tesla's wheels: natural
  frequency 9.5 rad/s (1.5 Hz), damping ratio 1.0, 7.5 cm raise and drop, 20 kg.
- Tires: friction = surface friction x the tire config's scale (`TireConfig.cpp:122`); CARLA sets
  the scale to 3.5 on every vehicle (`CarlaWheeledVehicle.cpp:158`) on UE's default 0.7: 2.45.
  Lateral stiffness at rest is 20 x 0.70 = 14 x rest load per rad, longitudinal 3000 kg x g.
- Braking beyond 700 N m: engine braking through the clutch. PhysX solves wheels, clutch
  (strength 10) and engine (damping 2.0 at zero throttle, MOI 1) implicitly
  (`PhysX_3.4/.../PxVehicleUpdate.cpp:3850`); through the 9:1 drive the engine's inertia adds
  ~590 kg to the car. Coasting is ~0.35 m/s^2 per m/s, full brake ~9 m/s^2 at 15 m/s (acb's maps
  agree). Aerodynamic drag is 0.5 rho v^2 Cd w h with Cd 0.15 (`...MovementComponent.cpp:992`):
  ~0.1 m/s^2 at 24 m/s, negligible.
- The ~0.15 s brake onset on CARLA: two rate limiters in series (UE's `InterpInputValue`, PhysX's
  pad smoothing: throttle and brake rise 6/s, fall 10/s; `...MovementComponent.cpp:116`) and the
  engine side's time constant I/(K + D) = 1/12 s after a lift-off.
- Steering: inner wheel = command x 70 deg x steering curve(speed), outer by Ackermann
  (accuracy 1, `PxVehicleUpdate.cpp:1797`); steering input rises 2.5/s and falls 5/s
  (`...MovementComponent.cpp:120`), ~175 deg/s at the wheel. acb inverts the Ackermann mean and
  divides by the curve.

Open: the CoM (48/52, 0.46 m) and the tires are assumptions; a real Model 3's measured weight
split, CoM height and tire data would replace them. The wheel base and radius are the CARLA
body's, 13 cm and 3.6 cm off a real Model 3. Not yet measured with Autoware driving.
