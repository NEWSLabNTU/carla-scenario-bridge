# The vehicle side: one spec per vehicle, Autoware with or without SSv2

How the ego's car is described once and used by Autoware, by CARLA, by Isaac Sim and by
SSv2's scenarios. Also how Autoware drives that car on either simulator with no scenario
runner at all. Phases: [048](../roadmap/048-vehicle-side-profile.md) (the layer and the
launch profile), [049](../roadmap/049-vehicle-spec.md) (the spec as the single source).
Background: [simulator-interface-split.md](simulator-interface-split.md), "The vehicle
side".

**Status**: proposed 2026-10-23. Directions from the user that day:

- share packages across simulators only if the design guarantees that a future simulator
  can use them; otherwise keep one package per simulator;
- one source for vehicle dynamics, physics and the sensor setup, kept consistent across
  Autoware, the simulators and SSv2;
- CARLA gets the spec's physics;
- Autoware must be able to connect to CARLA and to Isaac Sim without SSv2.

## What exists (2026-10-23)

| what | Autoware reads | CARLA | Isaac Sim | SSv2 |
|---|---|---|---|---|
| geometry | acb `vehicle_info` hand-written from a CARLA measurement; aib's generated from its spec | the blueprint's body and `physics_control` | aib spec | 132 inline `<Vehicle>` copies in 33 `.xosc` files; 130 declare 4.5 x 2.1 x 1.8 m with a 3.1 m wheelbase against the car's 4.792 x 2.163 x 1.488 m and 3.005 m |
| dynamics | none (planning limits only) | the blueprint's own physics, plus acb's pedal maps measured on it | aib spec (Model 3 RWD: motor, regen, ABS, traction control, brake lag) | `Performance`, four different sets |
| sensors | acb: URDF and calibration (13 sensors, 4 used); aib: generated | acb `vehicle_config.yaml`, keyed by URDF link | aib spec | none |
| sensing launch | five files per kit, identical in acb and aib except comments and package names | | | |

aib's `vehicles/tesla_model3.yaml` is already a single source for Isaac and for Autoware's
files there (`just generate`, `just check-generated`). acb and the scenarios agree with it
only by hand. CARLA and Isaac drive two different cars.

Without SSv2 today:

- **CARLA**: acb `just sim`, or `scenario` (`demo_scenario.py` spawns the hero),
  `acb_bridge`, `carla_simulator.launch.xml`, then `auto_drive` or a goal in RViz.
- **Isaac**: aib `just isaac-ego` (`aib.runner`: one ego in a world, publishing `/clock`,
  the status and the sensors) and `just autoware`.

Both work. The design must keep them working, and must not make acb or aib depend on
anything that carries SSv2.

## Layers

```
  SSv2 ─ scenario-world-bridge (adapter, world servers, relay, agent)   needs SSv2
                │ uses
  acb (CARLA vehicle bridge)    aib (Isaac vehicle library)           one per simulator
                │ both use
  vehicle layer: spec + generators, generated Autoware packages,
                 the Autoware launch profile, the I5 contract and its check   no SSv2, no simulator
```

**The vehicle layer** knows Autoware and the vehicle, and no simulator and no SSv2. acb and
aib depend on it. Neither depends on swb. swb depends on all three. A run without SSv2 is
the vehicle layer plus one simulator package, which is what acb's and aib's standalone runs
already are.

**Where it lives**: its own repository (decision V1). acb and aib each need it without
swb, and swb needs it beside them. A submodule in all three would put the same packages
into swb's workspace two or three times, and colcon refuses duplicate packages. So it is a
submodule of swb only. acb's and aib's `just setup` fetch the commit pinned in a
`vehicle.repos` file for their standalone workspaces (`vcs import`, or a plain clone into
the git-ignored `external/`). The pin follows the same rules as a submodule pin.

## The rule for sharing

A package is shared only when its correctness depends on nothing but the vehicle layer's
contract. A simulator that cannot meet the contract adapts in its own bridge, as a hardware
driver does for a real car. The guarantee comes from a contract plus a check that any
simulator's bridge can be run against:

| shared (vehicle layer) | why it is safe for any simulator |
|---|---|
| the vehicle spec, its format and generators | describes the car, not the simulator; per-simulator sections are optional and owned by that simulator |
| per-vehicle Autoware packages: `<vehicle>_description`, `<vehicle>_launch` (empty vehicle interface), `<kit>_description`, `<kit>_launch` (the sensing pipeline), `individual_params` | generated from the spec; they read only the topics, types and frames the contract fixes |
| the Autoware launch profile | Autoware plus our tuning for a simulated car; starts no vehicle interface, needs no relay |
| the I5 contract and `vehicle_check` | the definition every bridge is checked against |

| per simulator | why it is not shared |
|---|---|
| the vehicle interface (acb_bridge; aib's in-process interface) | it is the simulator's API |
| sensor creation and the conversion to the contract's formats | each simulator's sensors differ |
| how the spec's physics is realised (`carla:`, `isaac:` sections and code) | each engine's model differs |
| standalone bring-up (acb `demo_scenario.py`, `aib.runner`) | spawning is simulator API |

Autoware selects packages by `vehicle_model` and `sensor_model`. Today those name a
simulator (`acb_vehicle`, `aib_vehicle`). Afterwards they name a car (`tesla_model3`),
the same on every simulator. The profile always passes `launch_vehicle_interface:=false`.
The simulator side starts its interface itself, as acb's wrapper and aib already do.

## The I5 contract

What a vehicle bridge provides on the ego's ROS domain, for any simulator:

- **Sensors**: one topic per spec sensor, with the spec's name, message type and point
  layout. Lidar: `PointCloud2` with Autoware's `PointXYZIRC` at the spec's `topic`. GNSS:
  `NavSatFix` in the map projection the map's `map_projector_info.yaml` names. IMU: `Imu`.
  Camera: `Image` and `CameraInfo`. All in the spec's frames.
- **Vehicle status**: `/vehicle/status/{velocity_status, steering_status, gear_status,
  control_mode, turn_indicators_status, hazard_lights_status}` at 20 Hz or more.
- **Commands taken**: `/control/command/{control_cmd, gear_cmd, turn_indicators_cmd,
  hazard_lights_cmd, emergency_cmd}`. `control_cmd`'s acceleration is realised against the
  spec's physics, not passed through ideal.
- **Time**: one `/clock` per domain. Every stamp above comes from it. A sensor sample
  carries the time of the frame it was taken in. With a world server, the clock is that
  server's time. Standalone, it is the simulator's. (CARLA's episode epoch is the one
  exception kept for now: 048, decision 3.)
- **Frames**: `base_link` on the ground at the rear axle centre, x forward, z up; the
  sensor frames from the generated URDF. The bridge publishes no TF that the URDF owns.

**`vehicle_check`** (in the vehicle layer, no SSv2) runs against a live vehicle side:

- topics, types, rates, frames and stamps;
- `/robot_description` against the spec;
- a dynamics run: acceleration steps, braking to a stop, a steering sweep, sent on
  `/control/command/control_cmd`, read back from `/vehicle/status/*`, and compared to the
  spec's limits and to acceptance.py's tracking limit (0.35 m/s²).

A new simulator is compatible when `vehicle_check` passes. That is the guarantee the shared
packages rest on.

## The spec as the single source

aib's spec format (`docs/spec-format.md`, `spec_version: 1`) grows into version 2 and
moves to the vehicle layer, with `vehicles/<name>.yaml` per car:

- **shared sections**: `geometry`, `mirror`, `inertia`, `wheels`, `tires`, `brakes`,
  `powertrain`, `resistance`, `steering`, `suspension`, `sensor_kit`, `sensors` (mount,
  topic, frame, type, rate, range, FOV), `provenance`;
- **per-simulator sections**: `isaac:` (today's `physx:` and per-sensor `isaac:`) and
  `carla:` (new: the blueprint, the sensor blueprint attributes, and the pedal maps'
  calibration below).

What is generated from it, and who checks it:

| output | for | stale check |
|---|---|---|
| Autoware's per-vehicle packages | Autoware | `check-generated` (byte compare) |
| `isaac` rig | aib | same |
| `carla_physics.yaml`: every `VehiclePhysicsControl` field CARLA 0.9.16 takes | acb, carla_world | same |
| `carla_sensors.yaml`: today's `vehicle_config.yaml` | acb_bridge | same |
| SSv2 catalog `catalogs/vehicles.xosc`: `BoundingBox`, `Axles`, `Performance` | scenarios | same, plus a lint: no inline ego `<Vehicle>` in any `.xosc` |
| pedal maps (`accel_map.csv`, `brake_map.csv`) for CARLA | acb_bridge | measured, not generated: a calibration run on CARLA with the spec's physics writes them with the spec's physics hash; `check-generated` fails when the hash no longer matches |

### CARLA gets the spec's physics

`carla_physics.yaml` maps the spec to CARLA's model:

- mass and centre of mass;
- drag;
- a single forward gear at `gear_ratio`, `final_ratio` 1, no autobox switching;
- the torque curve from the motor's peak torque and power;
- `max_rpm`, `moi`;
- per wheel: radius, `tire_friction`, `lat_stiff_*`, `long_stiff_value`, `damping_rate`,
  `max_brake_torque`, `max_steer_angle`.

What CARLA's model cannot express (regen blending, ABS, traction control, brake dead time
and build-up, motor lag) is either realised in acb_bridge's command path (the lags, as
`steer_time_constant` is today) or listed in the file as `not_realised`. The dynamics run in
`vehicle_check` measures what remains.

**One writer**. Whoever spawns the vehicle applies `carla_physics.yaml` once, right after
the spawn:

- with SSv2, `carla_world`;
- standalone, `demo_scenario.py`.

Both only load the generated file and copy fields, with no mapping logic of their own.
acb_bridge reads `physics_control` back on attach and refuses to drive a vehicle whose
physics differ from the file beyond float tolerance, naming the fields. carla_world's road
friction (047) scales the friction it reads after that write. Today it caches the
vehicle's "own" friction at its first scaling. It must take the spec's value instead, so
the two can never race.

Isaac already works this way: the world server and `aib.runner` both build the vehicle
from the rig through aib's library.

### SSv2 uses the same car

The catalog's ego entry is named after the spec (`tesla_model3`). Its `BoundingBox` comes
from the geometry: the centre at the box's middle in the rear-axle frame, so 1.532,
0, 0.744. Its `Axles` come from the wheel base, track and radius, and its `Performance`
from the powertrain envelope. Scenarios use `CatalogReference`. The world servers resolve
the entry name to the spec: CARLA's blueprint and physics, or Isaac's rig.

At spawn, the world server already reports the model's own box in `SpawnReply.body`
(`box_location`, `box_extent`). It adds the wheel layout: CARLA's from `physics_control`,
Isaac's from the rig. For the ego, the bridge compares both with the declared box and
axles. Past a tolerance (2 cm by default) it refuses the spawn and puts both boxes in the
message. That catches an asset or blueprint that drifted
from the spec.

## Consistency, end to end

1. **One writer**: every value is edited in the spec and nowhere else; everything else is
   generated or measured against the spec's hash.
2. **Build time**: `check-generated` in every repository that holds output, and the `.xosc`
   lint.
3. **Spawn time**: SSv2's box against the world server's measurement; CARLA's physics
   against the file, read back by acb_bridge.
4. **Behaviour**: `vehicle_check`'s dynamics run on each simulator, without SSv2; its
   numbers go in the phase's results.

## Decisions (for the user)

- **V1. Repository** for the vehicle layer, and its name. [proposed: a new NEWSLabNTU
  repository `autoware_sim_vehicle`, a submodule of swb, fetched by acb's and aib's setup]
- **V2. Package names per car.** Autoware needs `<vehicle_model>_description` and
  `_launch`. [proposed: `tesla_model3_description`, `tesla_model3_launch`,
  `tesla_model3_sensor_kit_description`, `tesla_model3_sensor_kit_launch`]
- **V3. Fix the scenarios' ego box first**, before the spec moves. It changes every SSv2
  distance and collision measurement slightly. [proposed: yes, as 049's first step, with a
  before and after of the suite]
