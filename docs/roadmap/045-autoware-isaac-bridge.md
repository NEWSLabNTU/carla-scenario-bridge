# Phase 045: autoware_isaac_bridge (aib)

The Isaac ego as its own repository, the way acb is CARLA's: the PhysX vehicle, its sensors
and Autoware's vehicle interface, with the vehicle described once in a spec that generates
Autoware's vehicle and sensor kit files and the Isaac rig. csb's world server then imports
it instead of carrying its own copy in `src/csb_isaac`.

**Source**: user direction 2026-10-18 (a standalone public repository in NEWSLabNTU; the
single source of truth for the vehicle lives in aib).
**Depends on**: [043](043-isaac-single-vehicle-e2e.md) (the plausible real Model 3, the
supervised world server, the exit runs).
**Feeds**: [044](044-isaac-npcs-and-traffic-lights.md), [039](039-isaac-multi-autoware.md)
(several egos, one rig each), [040](040-ntu-offroad-field.md) (another vehicle is another
spec).

Repository: [NEWSLabNTU/autoware_isaac_bridge](https://github.com/NEWSLabNTU/autoware_isaac_bridge)
(its `docs/design.md` and `docs/spec-format.md` are the reference; this phase is csb's side).

## Design

### What moves, what stays

| moves to aib | stays in csb |
|---|---|
| `csb_isaac/vehicle.py`: the PhysX vehicle (`aib.isaac.vehicle`), its actuation split into `aib.control` (steering, Ackermann) and `aib.powertrain` (pure) | `world.py`, `world_server.py`, `server.py`, `protocol.py`, `csb_world.proto`: the lockstep world, NPCs, contacts, the world protocol |
| `csb_isaac/vehicle_params.py`: `VehicleParams` (`aib.params`, same field names, so `vehicle:` overrides still apply) | `physics_params` and the CARLA catalogue (`vehicle_assets.py`, `dump_vehicle_physics.py`): CARLA's blueprints are csb's |
| `csb_isaac/ego.py`: `EgoRos` (`aib.ros`), `Ego` and the sensors (`aib.isaac.ego`, `aib.isaac.sensors`; IMU and GNSS maths pure in `aib.sensing`) | the body the ego wears (`vehicle_assets.add_ego_body`) and the vehicle_info-vs-catalogue check, now against the rig |
| `csb_isaac/frames.py` (`aib.frames`) | `physics_layer.py`, `usd_world.py`, `pcd_world.py`, `isaac_town.sh`: worlds |
| `scripts/isaac_env.sh`, `scripts/build_msgs.sh` (aib carries copies; csb can source aib's) | the step-response scripts (they compare CARLA and Isaac; they can import aib) |
| `config/acb_ego.yaml`: the vehicle part becomes aib's spec and rig; the run part (domain, clock offset) aib's `config/ego.yaml` | `isaac_world.launch.xml`, `just isaac-world`, `scripts/isaac_up.sh` |

### The single source of truth

`vehicles/tesla_model3.yaml` in aib is the car: geometry, mass and centre of mass, wheels,
tires, brakes, powertrain, steering, suspension, PhysX settings, the sensor mounts and
intrinsics, and a provenance entry per number (`assumed: true` on the centre of mass, the
inertia box, the wheel mass, the tires and the suspension). The values are 043's "As built:
vehicle" table. `aib.gen` writes from it, each file headed "generated from ..., do not edit":

- `aib_vehicle_description/config/vehicle_info.param.yaml`, `mirror.param.yaml`
- `aib_sensor_kit_description/config/{sensors,sensor_kit}_calibration.yaml`, `urdf/sensor_kit.xacro`,
  and the same calibration under `aib_individual_params/config/default/aib_sensor_kit/`
- `src/aib/config/tesla_model3.rig.yaml`: the flat `VehicleParams`, the chassis box, each
  sensor with its base_link transform

`just check-generated` (and a unit test, and aib's CI) fails when any is stale. Later, acb's
`vehicle_config.yaml` and vehicle description can be generated from the same spec, so CARLA
and Isaac drive one car description; not part of this phase.

### Autoware side

`aib_launch isaac_simulator.launch.xml` includes acb's `carla_simulator.launch.xml` with
`vehicle_model:=aib_vehicle sensor_model:=aib_sensor_kit launch_vehicle_interface:=false`;
every other argument passes through (`relay`, `map_path`, `rviz`, ...). The agent is acb's
`acb_pilot`, unchanged. A `play_launch dump` of it resolves the same 205 nodes as
`carla_simulator.launch.xml launch_vehicle_interface:=false`, with every vehicle_info
reference aib's. Topics and frames are acb's (`top`, `carla/imu_link`, `carla/gnss_link`),
as csb_isaac publishes them today.

### Powertrain

A seam (`aib.powertrain`): `ideal` is today's behaviour (the requested acceleration as
per-wheel torque from m a, braking split by dynamic axle load). `physx` (DriveStandard),
`ev` and `fmu` are reserved names that fail at load.

`ev` comes next, modelled on the Tesla Model 3 drivetrain (user direction 2026-10-19), since
there are no drive logs to fit against. It covers the motor torque/power curve, regenerative
braking, the brake hydraulic lag and ABS.

- [x] `ev` in aib (aib `7816ba6`, 2026-10-19): the Model 3 RWD drive unit (3D7 motor, 340 N m /
      194 kW, 9.03:1 scaled to the CARLA body's wheel, torque lag 40 ms), regen (130 N m /
      60 kW, ~0.2 g, faded below 3 m/s) blended with friction brakes through 80 ms dead time
      + 60 ms lag, ABS per wheel and traction control on PhysX slip; road loads (Crr 0.0095,
      Cd 0.23 x 2.2 m^2) as a body force for every model. Unit-tested (93 tests, system and
      Isaac Python); point-mass 0-100 km/h 5.9 s against the published 5.8-6.1 s.
- [x] `ev` validated in Isaac and the default for tesla_model3 (aib `54cace1`, 2026-10-19;
      aib docs/design.md, Validation). `just isaac-steptest`: 0-100 km/h 6.45 s (Tesla 6.1 s;
      0.27 s of it the hold release), 100-0 km/h 42.9 m with ABS, no lock (44.1 m locked; ~40 m
      published), regen at most ~0.20 g at 55 km/h, -1.5 m/s^2 all regen, brake step t63
      0.10 s, coast-down matching the road-load model. Closed loop on Town01 (Autoware +
      acb's pilot, town01_ego_drive's route): median longitudinal error 0.018 m/s^2 (`ideal`
      0.016; limit 0.35), one run each. The tyre grip now falls to 0.8 at lock (user
      decision), for `ideal` too: csb's 043 numbers were taken on the flat curve.

The other options (`physx` DriveStandard, an FMU,
Project Chrono for rough or deformable ground) wait until a need shows.

### Decisions (user, 2026-10-19)

- **IMU/GNSS frames** keep `carla/imu_link` and `carla/gnss_link` for now, because acb's
  sensing launch, which aib reuses, names them. Rename them in the spec at integration.
- **Camera images** are stamped in `camera6/camera_optical_link` (REP-103: Autoware's
  projection nodes assume the optical frame). acb stamps `camera_link`, which is likely
  wrong there too. The camera is off on both bridges today.
- **aib depends on acb at run time** (`acb_launch`, `acb_pilot`) instead of copying them;
  standalone use needs `ACB_SETUP`.
- **Sensor poses come from the spec only.** A hand edit to a generated calibration does not
  reach Isaac, and `just check-generated` flags it.
- **The Isaac message build is shared**: aib and csb both use `$ISAAC_ROOT/msgs`.

## Steps

- [x] aib repository: library, spec, generator with check mode, Autoware packages, unit
      tests without Isaac, docs; builds with colcon against Autoware 1.5.0 (2026-10-18).
- [x] aib's Isaac code run once from aib itself: `just isaac-ego` on Town01 with `just
      autoware`, a drive to a goal (standalone, no csb) (2026-10-19, aib `54cace1`): arrived,
      `ev` and `ideal`. Needed: the chassis box hidden from the lidar (Autoware stopped for
      the ego's own roof), `just autoware` keeping the caller's CYCLONEDDS_URI, and
      `realtime:=true` (unpaced, the world ran 3x real time and localization never started).
- [ ] Integration: aib as a submodule at `src/autoware_isaac_bridge` (pushed first, then the
      pin); `csb_isaac` imports `aib` (world server's `PYTHONPATH` gets
      `src/autoware_isaac_bridge/src/aib`); `IsaacWorld` builds `aib.isaac.ego.Ego` from an
      aib ego config (`ego:=` names one; default aib's `config/ego.yaml`); the catalogue
      check runs against the rig's geometry; `just ego-isaac` runs `aib_launch
      isaac_simulator.launch.xml`.
- [ ] Remove the duplicated code from csb: `csb_isaac/{ego,vehicle,vehicle_params,frames}.py`,
      `config/acb_ego.yaml`, their tests (aib has them), and point docs (038, 043, the user
      guide) at aib.
- [ ] Exit runs through aib.

## Acceptance

- 043's exit runs (`town01_ego_drive`, `town01_engage_state`, `town02_episode_change`, one
  route per exported town) still pass on Isaac with the ego built by aib: 5/5 each, 0 MRM,
  tracking within acceptance.py's limits, and of the same order as 043's recorded numbers.
- csb carries no copy of the ego: `grep -r PhysxVehicleAPI src/csb_isaac` finds nothing.
- A change to the vehicle is one edit to aib's spec plus `just generate`; aib's
  `check-generated` fails if the generated files were not refreshed.
