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
