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

## Steps

- [ ] Vehicle USD from the Autoware vehicle parameters; step response and steering sweep
      against CARLA's vehicle on flat ground.
- [ ] Control and status interface; acb's tracking measurement script run against Isaac.
- [ ] Sensor set and `/clock` (with `isaac.render` on: 037 turns rendering off by default,
      which the sensors need back); Autoware's sensing and localization come up on the test
      world's PCD (generated with the test world in 037).
- [ ] One ego through a starter scenario: route, engage, drive, arrive; SSv2 verdict.
- [ ] Document the Isaac ego launch alongside `just ego-av`.

## Acceptance

- Autoware engages and completes a route scenario on the Isaac test world, three runs
  unattended, same verdict.
- Longitudinal tracking error and lateral error comparable to the same scenario on CARLA
  (reported side by side).
- Real-time factor with the full sensor set at or above what 036 measured for one set.
