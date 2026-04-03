# carla-scenario-bridge

ZMQ+Protobuf adapter that makes CARLA a backend for [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) (SSv2). Run OpenSCENARIO scenarios in CARLA with Autoware processing real sensor data through its full perception pipeline.

## Architecture

```
SSv2 (scenario interpreter + traffic simulator)
    | ZMQ + Protobuf
    v
carla-scenario-bridge (this project)
    | CARLA API (carla-rust)
    v
CARLA Server
    | sensor callbacks
    v
autoware_carla_bridge (sensor/vehicle interface)
    | ROS 2 topics
    v
Autoware (full AD stack)
```

SSv2 controls scenario logic and NPC behavior. CARLA provides ego vehicle physics and sensor rendering. Autoware processes real sensor data (not ground-truth pseudo sensors).

## Status

**Phase**: Project scaffold. See [roadmap](docs/roadmap/README.md) for implementation plan.

## Prerequisites

- CARLA 0.9.16
- ROS 2 Humble
- [autoware_carla_bridge](https://github.com/NEWSLabNTU/ros_zenoh_bridge) (for sensor/vehicle interface)
- [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) (for scenario execution)

## Build

```bash
just build
```

## Usage

```bash
# 1. Start CARLA
just carla-start

# 2. Start this adapter (connects to CARLA, listens for SSv2)
just run

# 3. Start autoware_carla_bridge (sensor/vehicle interface)
# (in autoware_carla_bridge repo)
just bridge

# 4. Run an SSv2 scenario
just scenario /path/to/scenario.xosc
```

## Documentation

- [Architecture](docs/design/architecture.md)
- [SSv2 Protocol Reference](docs/design/ssv2-protocol.md)
- [Roadmap](docs/roadmap/README.md)

## Related

- [autoware_carla_bridge](https://github.com/NEWSLabNTU/ros_zenoh_bridge) - Sensor/vehicle interface between CARLA and Autoware
- [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) - Autoware scenario testing framework
- [AWF odd-usecase-scenario#114](https://github.com/autowarefoundation/odd-usecase-scenario/issues/114) - AWF CARLA integration issue
