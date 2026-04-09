# carla-scenario-bridge

## Project Overview

ZMQ+Protobuf adapter that makes CARLA a backend for tier4/scenario_simulator_v2 (SSv2). Implements SSv2's `simulation_interface` protocol, translating protobuf requests into CARLA API calls via carla-rust.

**Design doc**: `docs/design/architecture.md`
**Roadmap**: `docs/roadmap/`
**Parent project**: [autoware_carla_bridge](https://github.com/NEWSLabNTU/ros_zenoh_bridge) (sensor/vehicle interface layer)
**SSv2 protocol reference**: See `proto/` directory

## Build & Run

```bash
just build    # Build with colcon + cargo
just clean    # Clean artifacts
just run      # Run the bridge (connects to CARLA, listens for SSv2)
just check    # Format + clippy
just test     # Run tests
```

## Key Design Decisions

- **AWSIM pattern**: SSv2 puppeteers NPCs via `set_transform()`, CARLA owns ego physics
- **Sensors**: Reject SSv2 `AttachSensor` requests (`Success=false`); CARLA sensors publish via autoware_carla_bridge
- **Traffic lights**: Freeze CARLA's built-in cycling, set states per SSv2 commands
- **Coordinate conversion**: SSv2 uses ROS right-handed frame; CARLA uses left-handed (Y-flip)
- **Vehicle interface**: autoware_carla_bridge handles `/vehicle/status/*` and `/control/command/*`; SSv2's `AutowareUniverse` (concealer) should be disabled

## Repository Structure

```
.
├── src/carla_scenario_bridge/  # Rust crate (ament_cargo package)
│   ├── src/
│   │   ├── main.rs             # Entry point, CARLA retry loop, shutdown
│   │   ├── zmq_server.rs       # ZMQ REP socket, protobuf dispatch
│   │   ├── coordinator.rs      # 14 SSv2 handlers → CARLA API calls
│   │   ├── entity_manager.rs   # SSv2 name ↔ CARLA actor ID mapping
│   │   ├── coordinate_conversion.rs  # ROS ↔ CARLA frame conversion
│   │   ├── traffic_light_mapper.rs   # Lanelet signal ID ↔ CARLA actor (Phase 4)
│   │   └── proto.rs            # Generated protobuf type re-exports
│   ├── build.rs                # prost-build proto compilation
│   ├── config/bridge_config.yaml
│   ├── Cargo.toml
│   └── package.xml
├── proto/                  # SSv2 protobuf definitions (8 .proto files)
├── scenarios/              # Example OpenSCENARIO test files
├── docs/
│   ├── design/             # Architecture, protocol, launch config docs
│   └── roadmap/            # Phase 1-5 roadmap with task checklists
├── Cargo.toml              # Workspace root
├── justfile                # Build and run commands
└── CLAUDE.md               # This file
```

## Coding Practices

### Use play_launch instead of ros2 launch
All launch commands use `play_launch launch` (with `--web-addr` for the web UI), not `ros2 launch` directly.

### Use just build
Always use `just build` instead of `colcon build` directly (ensures `--symlink-install`).
