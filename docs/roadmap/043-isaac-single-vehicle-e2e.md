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

**CARLA's physics in the catalogue.** `src/csb_isaac/scripts/dump_vehicle_physics.py` (also
`/mnt/disk1/carla-usd/vehicles/tools/dump_physics.py`) connects to a running CARLA 0.9.16,
spawns every blueprint in `vehicles.json`, reads `get_physics_control()` and writes it into the
entry's `physics` and into `physics.json` beside it (`make_assets.py` rewrites `vehicles.json`
without it; `vehicle_assets.py` falls back to the sidecar). Frame: the actor origin, metres,
CARLA's Y flipped; wheel positions come back from CARLA in world centimetres and are made
actor-relative. All 41 blueprints dumped in ~1 min. CARLA 0.9.16 exposes no suspension, and
its `center_of_mass` is a nudge from the centre UE computes from the collision bodies, not
the centre itself: `com_estimate` adds it to the bounding box centre, an estimate.

vehicle.tesla.model3, CARLA against what the ego had before:

| | CARLA blueprint | before (hand-typed / vehicle.py) |
|---|---|---|
| mass | 1845 kg | 1845 kg |
| centre of mass (base_link) | (1.866, 0, 0.498) m, estimated (nudge (0.45, 0, -0.30)) | (1.45, 0, 0.55) m |
| wheel base / tread / radius | 3.0046 / 1.6668 / 0.370 m | vehicle_info 3.005 / 1.667 / 0.370 |
| steering limit | 70 deg inner wheel = 1.000 rad bicycle | 0.70 rad (vehicle_info's planning limit) |
| brake torque | 700 N m per wheel (handbrake 1400 rear) | 4000 N m |
| drive torque | 743 N m peak x final drive 9 = 1672 N m per wheel | 2500 N m |
| tire friction | 3.5 | 1.0 |
| tire stiffness | lateral 20 x rest load at 3 rest loads; longitudinal 3000 kg x g | 17 x rest load at 2; 10 x rest load |
| wheel damping | 0.25 | 0.25 |
| drag coefficient | 0.15 | (not modelled: the actuation realizes the requested force) |

**The ego from the catalogue.** `acb_ego.yaml` names the car (`model: vehicle.tesla.model3`);
`vehicle_params.py` (pure, unit-tested in `test/test_vehicle_params.py`) takes geometry from
`vehicle_info` and physics from the model's `physics`, then the YAML's `vehicle:` overrides by
field name (documented in the YAML; empty in the shipped one). `IsaacWorld` resolves this at
start and refuses a wheel base, tread or radius more than 5 cm apart, naming both numbers:

```
VehicleMismatch: the ego's vehicle description is not vehicle.lincoln.mkz_2020: wheel_base
3.005 m in vehicle_info, 2.860 m in vehicle.lincoln.mkz_2020's physics; wheel_tread 1.667 m
in vehicle_info, 1.593 m in vehicle.lincoln.mkz_2020's physics (tolerance 5 cm). ...
```

The ego wears the YAML's model whatever body the scenario names (a warning says so).
Suspension stays ours, now per axle: each spring on its own sprung mass (the mass split at
the centre of mass) at the same frequency, so the car sits level and base_link on the ground.

**A braking bug found on the way.** PhysX's vehicle at 1 substep above 5 m/s (and a 0.1 m/s
active slip denominator) went unstable under brake torque: the wheels locked and the car
stopped at the tire friction limit. A -3 m/s^2 request from 7.4 m/s stopped it in 0.7 s
(about -11 m/s^2) with the old friction of 1.0, and in 0.25 s (about -30) at CARLA's 3.5.
So every Autoware braking command on Isaac before this was realized several times harder
than asked. Now 8 vehicle substeps at every speed and an active slip denominator of 4 m/s
(`physx_substeps`, `min_active_slip_denominator`): -3.02 m/s^2, stopping when CARLA does.

**Same car, same response.** `src/csb_isaac/scripts/step_response/`: the same command
sequences, 0.05 s frames, on CARLA (Town06's straight; acb_bridge's actuation replicated:
pedal maps, Ackermann inverse, steering-curve compensation, handbrake hold; ticks with
physics substeps of 0.05/16 as csb runs them) and on Isaac (`IsaacWorld` with the ego YAML,
flat ground, commands into the ego's command slot, 3 PhysX steps per frame as csb steps it).
Speed is a P loop on the commanded acceleration where held, the same on both. Data and
plots: `/mnt/disk1/csb-043-vehicle/out/`. Windows start a second after the step they measure;
the error is the largest difference over the window, against CARLA's peak there:

| test | window | CARLA | Isaac now | error now | Isaac before | error before |
|---|---|---|---|---|---|---|
| +1.5 m/s^2 for 5 s: slope | 2-6 s | 1.446 m/s^2 | 1.481 | 2.3 % (speed) | 1.468 | 3.1 % |
| -3 m/s^2 to a stop: slope, stop | 6.5-7.5 s | -2.962, stops 8.55 s | -3.021, 8.50 s | 8.8 % (speed) | ~-11, 6.80 s | 100 % |
| 5 m/s, steer 0.2 rad: yaw rate | 9-12 s | 0.333 rad/s | 0.332 | 0.6 % | 0.329 | 1.8 % |
| 10 m/s, steer 0.1 rad: yaw rate | 9-11 s | 0.329 rad/s | 0.326 | 0.7 % | 0.274 | 18.8 % |
| speed held under steer (5 / 10 m/s) | | 4.96 / 9.82 m/s | 4.93 / 9.77 | 1.7 / 0.5 % | 4.92 / 9.72 | 3.6 / 2.8 % |

All within 10 % now. The braking error left is CARLA's: its speed goes on rising for one
tick after the brake command and the deceleration builds over ~0.15 s; Isaac's brake acts
at once. "Before" is the pre-043 vehicle (hand-typed mass and centre of mass, the old tires,
1 substep at speed). Its 10 m/s understeer (yaw rate 17 % short) went with CARLA's tires.

Open: the centre of mass is estimated (CARLA does not expose it; the steady-state yaw rates
agree, so the front/rear split is not visibly wrong); the brake authority is now CARLA's 700
N m per wheel, about 4 m/s^2 on its own, where CARLA adds engine braking at speed (8.8 m/s^2
coasting at 24 m/s) -- Isaac realizes a hard request at speed with the brakes alone and tops
out near 4 m/s^2, so an AEB-strength stop is weaker than on CARLA (override
`max_brake_torque` if a scenario needs it); not yet measured with Autoware driving.
