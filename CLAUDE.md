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

**IMPORTANT**: Always use `just build` instead of `colcon build` directly (ensures `--symlink-install`).

## Key Design Decisions

- **AWSIM pattern**: SSv2 puppeteers NPCs via `set_transform()`, CARLA owns ego physics
- **Sensors**: Reject SSv2 `AttachSensor` requests (`Success=false`); CARLA sensors publish via autoware_carla_bridge
- **Traffic lights**: Freeze CARLA's built-in cycling, set states per SSv2 commands
- **Coordinate conversion**: SSv2 uses ROS right-handed frame; CARLA uses left-handed (Y-flip)
- **Vehicle interface**: autoware_carla_bridge handles `/vehicle/status/*` and `/control/command/*`; SSv2's `AutowareUniverse` (concealer) should be disabled

## Repository Structure

```
.
├── src/                    # Rust source (carla-rust + ZMQ + protobuf)
├── proto/                  # SSv2 protobuf definitions (copied from SSv2)
├── config/                 # Runtime configuration
├── docs/
│   ├── design/             # Architecture and design docs
│   └── roadmap/            # Phase-based roadmap
├── justfile                # Build and run commands
└── CLAUDE.md               # This file
```
