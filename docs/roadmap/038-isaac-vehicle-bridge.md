# Phase 038: Isaac Vehicle Bridge (one Autoware)

Autoware drives an ego vehicle in Isaac Sim through an SSv2 scenario: a PhysX vehicle with
Autoware's sensor set, the vehicle interface acb provides for CARLA, the simulation clock on
the ego's domain, and the unchanged agent (`acb_pilot`) registered with the relay.

**Source**: user direction 2026-10-16.
**Depends on**: [036](036-isaac-bringup.md) (domain mechanism, RMW, cost),
[037](037-isaac-world-server.md) (world server, backend).
**Feeds**: [039](039-isaac-multi-autoware.md), [040](040-ntu-offroad-field.md),
[041](041-competition-director.md).

## Design

- **Vehicle**: a USD vehicle with PhysX vehicle schemas (drive, Ackermann steering,
  suspension, tires) whose dimensions, wheelbase, mass and steering limits match the Autoware
  vehicle description we drive in CARLA (the vehicle model acb uses; one parameter file, two
  simulators). Tire friction per ground material, so 040 can set mud, grass and gravel zones.
- **Control in** (`/control/command/control_cmd`, gear, turn indicators, hazard, emergency):
  longitudinal acceleration and steering angle mapped to drive torque / brake and steer
  through the same actuation model acb uses for CARLA, so control tuning carries over.
  Measured as acb's tracking was (CLAUDE.md: longitudinal tracking 0.051 m/s² on CARLA):
  the target is the same order on Isaac.
- **Status out** (`/vehicle/status/velocity_status`, `steering_status`, `gear_status`,
  `control_mode`, `turn_indicators_status`, `hazard_lights_status`), from the PhysX vehicle.
- **Sensors**: RTX lidar with a profile matching the sensor kit's lidar, cameras with the
  kit's intrinsics and extrinsics, IMU, GNSS; frames and topics as the sensor kit expects
  (acb_sensor_kit_launch is the reference). Stamps on the sim clock.
- **Clock**: `/clock` on the ego's domain from Isaac's sim time, as acb publishes CARLA's
  (015).
- **Domain**: the ego's ROS 2 context on its own domain by the mechanism 036 chose.
- **Agent**: `acb_pilot` unchanged (or its successor from [030](030-agent-api.md)'s Agent API);
  the agent relay and the agent protocol unchanged. The
  ego's spawn goes through `IsaacBackend` like any entity, flagged ego, physics driven.
- **Packaging**: the vehicle bridge is part of the world server (sensors render there),
  configured by a vehicle YAML per ego (topic prefix, domain, sensor set, vehicle params).
  The off-road WG simulator's per-vehicle topic remapping is the model.

## As built (2026-10-18)

What changed from the design above, and why:

1. **Autoware's own stack, minus acb_bridge.** `just ego-isaac` runs acb's
   `carla_simulator.launch.xml` with `launch_vehicle_interface:=false`: the same Autoware,
   sensor kit launch, vehicle description and agent (`acb_pilot`, registered with the relay as
   `ego`) as `just ego-av`, and nothing CARLA in it. The world server is the vehicle interface.
2. **The vehicle bridge is rclpy, not OmniGraph ROS 2 nodes.** Autoware's vehicle and control
   messages exist for the system's Python 3.10 only; Humble ships the C sources rosidl
   generated for their Python bindings, so `just isaac-msgs` (`scripts/build_msgs.sh`)
   compiles those against Isaac's 3.12 into `$ISAAC_ROOT/msgs` (two packages,
   `autoware_vehicle_msgs` and `autoware_control_msgs`; seconds). One rclpy context on the
   ego's domain (036's mechanism, `domain_id` explicit) publishes everything: one code path,
   and the lidar points can be laid out as Autoware's `PointXYZIRC`, which the bridge
   helper node cannot do.
3. **The context lives as long as the world server, not the ego.** A new DDS participant in a
   domain of ~200 Autoware ones is discovered in tens of seconds, so an ego's first scans went
   nowhere; and `/clock` now runs across scenarios instead of starting at each spawn.
4. **`/clock` never goes backwards.** Its offset is the wall clock when the world server
   starts (`clock_offset: wall`). With a fixed offset, every world server restart sent the
   ego stack's time back; Autoware cleared its TF buffers and EKF ("Detected jump back in
   time"), and the stack never drove again until restarted (two failed runs on 2026-10-18).
5. **The test world is the map's point cloud.** Autoware localizes by NDT against
   `pointcloud_map.pcd`, which a flat plane cannot match. `just isaac-world-gen [map]`
   (`csb_isaac.pcd_world`) turns the PCD into 0.4 m voxels (Town01: 5.4 M points to 815 k
   cubes, one `PointInstancer`, render only) over a flat collision ground at the road level,
   and writes `<map>/isaac/world.{usd,yaml}` (037's world file; the map dir is git-ignored, so
   each host generates it, in 9 s). The lidar sees the map's own shape: NDT converges in
   2-4 s at the scenario's pose, as on CARLA.
6. **The vehicle: PhysX "drive none", torques set directly.** `csb_isaac/vehicle.py` builds a
   four-wheel PhysX vehicle whose body frame is `base_link` (rear axle on the ground) from
   `vehicle_info.param.yaml` (wheel base 3.005, tread 1.667, radius 0.37, steering limit
   0.70) plus the mass and centre of mass the description lacks (CARLA's model3, 1845 kg).
   Actuation is acb's contract, not its pedal maps: Autoware's acceleration becomes a force
   `m a` (plus the wheels' inertia) split over four wheels as drive torque, or brake torque
   when it opposes motion, never the motor in reverse; the tire angle becomes Ackermann
   angles for the front wheels. Flat-ground checks: 1.0 m/s^2 asked gives 0.97 (before the
   wheel inertia term), -2.0 gives -1.92 and stops dead; yaw rate within 2-6 % of
   kinematic at tire angles 0.05-0.3 rad. Steering status echoes the actuator, as acb's
   default.
7. **Sensors: acb's set as CARLA runs it**: the 128-channel 20 Hz lidar (Ouster OS1 REV7
   128ch 20 Hz, the same 2.6 M points/s) at `base_link -> top` from the sensor kit's
   calibration files, IMU and GNSS. The camera is not spawned, as on CARLA
   (`traffic_light_fusion_only`). Topics, frames and stamps are acb's; GNSS is the map's
   TransverseMercator(0, 0) inverted near the origin (millimetres within a town). The IMU is
   differenced from the body's pose per physics step.
8. **Rendering is on whenever an ego has a lidar**, whatever csb's `isaac.render` says. The
   first frames of a new lidar compile its pipeline (2.8 s on a loaded host; past csb's 10 s
   step timeout once), so the world server renders them inside `Spawn`, and csb gives an ego's
   `Spawn` the load timeout (120 s).
9. **Configuration** is one ego YAML (`src/csb_isaac/config/acb_ego.yaml`: domain, the vehicle
   description and calibration files, physics, sensors), passed as `just isaac-world`'s
   `ego` (the default). `ego=""` keeps 037's kinematic box: `isaac/town01_rear_contact.xosc`
   needs it, since a kinematic NPC shoving a dynamic ego reads as 37.5 m/s^2 to SSv2's
   motion limits (±15).
10. **Watching.** `just isaac-world 5590 :8 <ego yaml> 1` opens a UI-less window on the VNC
    display whose viewport chases the ego, with a sun and sky (the voxel world has no light of
    its own). It costs frames: 854x480 is what this host keeps near real time under load.
11. **Tracking is scored from a bag** (`scripts/tracking_probe.py`): acb's `acceptance.py`
    formulas and limits, but run by run from one recording, on message stamps. A per-run
    observer joining the domain is discovered too late to see a run's start, and the wall
    clock misreads acceleration when the world runs slower than real time.

## Steps

- [x] Vehicle USD from the Autoware vehicle parameters; step response and steering sweep on
      flat ground (point 6; against the kinematic model rather than CARLA's vehicle: the
      tracking comparison below is the CARLA side by side).
- [x] Control and status interface; acb's tracking measurement (its formulas) run against
      Isaac and CARLA.
- [x] Sensor set and `/clock`; Autoware's sensing and localization come up on the test world
      generated from the map's PCD.
- [x] One ego through a starter scenario (`basic/town01_ego_drive`): route, engage, drive,
      arrive; SSv2 verdict PASS.
- [x] The Isaac ego launch, `just ego-isaac`, beside `just ego-av` (README of csb_examples,
      CLAUDE.md).

## Results (2026-10-18)

Run like this (CLAUDE.md's process rules apply; the world server first):

```bash
just isaac-msgs && just isaac-world-gen        # once per host
just isaac-world &                             # world server, ego = acb_ego.yaml
CSB_BACKEND=isaac just run &                   # csb on Isaac, agent relay
just ego-isaac &                               # Autoware, domain 1
ros2 run csb_launch run_suite src/csb_examples/scenarios/basic/town01_ego_drive.xosc --output <dir>
```

**Verdicts.** `basic/town01_ego_drive` (route 70 m, goal within 5 m), in order:

- 2 failures while building it: the first lidar frame outlasted csb's step timeout (exit 75,
  point 8); then Autoware never initialized on a freshly started stack (NDT busy aligning for
  minutes at 20 % per thread under a load of 55; not seen again once the context outlived
  the ego, point 3).
- 7 passes on one world server: 1, then 3 (with per-run probes), then the 3 measured below.
- 3 failures after world server restarts under the same ego stack: the ego engaged and stood,
  its controller holding -1.5 m/s^2; the clock had jumped back (point 4).
- After the clock fix and an ego stack restart: 4 passes of 4, the recorded run and the final
  batch of three, the batch after restarting the world server under the running stack.

**Tracking, the same scenario on both simulators** (`tracking_probe.py`, three runs each,
medians per run; acceptance.py's limits in brackets):

| | longitudinal error, m/s^2 (0.35) | cross-track, m (1.0) | EKF to GNSS, m (10) | driven |
|---|---|---|---|---|
| Isaac, first batch | 0.017, 0.018, 0.017 | 0.21, 0.20, 0.21 | 0.74, 0.74, 0.74 | 65.7 m |
| Isaac, final batch | 0.076, 0.073, 0.071 | 0.28, 0.21, 0.06 | 0.83, 0.82, 0.79 | 65.7 m |
| CARLA 0.9.16 | 0.006, 0.007, 0.008 | 0.04, 0.04, 0.04 | 0.91, 0.91, 0.91 | 66.1 m |

Both well inside the limits; CARLA's actuation, from measured pedal maps, tracks tighter. The
Isaac batches differ by the command stream, not the vehicle: delivered acceleration has the
same mean and spread in both (0.13, sd 0.44 m/s^2 while moving), while the final batch's commands
spent longer at the stop hold (mean -0.56 against -0.14) on a host at load 40-67 from other
work.

**Frame cost.** Headless, one ego with the lidar, IMU and GNSS in the 815 k-voxel Town01, a
50 ms frame (3 physics steps, one render) costs 20-22 ms p50 and 24-31 ms p95 at the world
server (one batch: 53 ms p95 under load): about 9 ms physics and render, 5 ms ROS callbacks,
2 ms lidar publish, the rest the Python vehicle. Runs are at real time (SSv2 paces them), with
2.3x headroom. With the chase viewport on VNC it is 50-74 ms p50 on this host: a recording
runs at 0.7-1x real time and still passes, on sim time.

## Acceptance

- Autoware engages and completes a route scenario on the Isaac test world, three runs
  unattended, same verdict.
- Longitudinal tracking error and lateral error comparable to the same scenario on CARLA
  (reported side by side).
- Real-time factor with the full sensor set at or above what 036 measured for one set.


**Outcome**:

- Engages and completes the route scenario, three unattended runs, same verdict: **met** (3/3
  in three batches; the last on the final code, after a world server restart).
- Longitudinal and lateral error comparable to CARLA's, side by side: **met** in the sense of
  the limits (both an order under them), with Isaac 2-10x CARLA's longitudinal error and
  1.5-7x its cross-track (table above).
- Real-time factor at or above 036's for one set: **not met as worded.** 036 measured a bare
  lidar-and-camera loop at 7.8 ms per frame (6.4x real time); the full ego costs 20-22 ms
  (2.3x), the vehicle, its ROS side and a larger world included. Real time holds headless
  with headroom; a watched (rendered-to-window) run does not on a loaded host. 039's ego
  count per GPU should be sized on 20 ms per ego, not 036's 5 ms per sensor set.
