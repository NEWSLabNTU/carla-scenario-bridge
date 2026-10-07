# carla-scenario-bridge

Run [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario-v200/) scenarios in CARLA with Autoware processing **real sensor data** through its full perception pipeline.

## What is this?

[Autoware](https://autowarefoundation.github.io/autoware-documentation/main/) is an open-source autonomous driving stack. It is typically tested with [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) (SSv2), a framework that runs OpenSCENARIO files against a simulator backend. The default backend is a lightweight "simple sensor simulator" that feeds Autoware **ground-truth data** — perfect detections, exact positions — skipping the perception pipeline entirely.

[CARLA](https://carla.org/) is a photorealistic driving simulator with GPU-rendered cameras, ray-cast lidar, and PhysX vehicle dynamics. It produces realistic, noisy sensor streams that exercise Autoware's full stack: perception, localization (GNSS → NDT scan matching), planning, and control.

**carla-scenario-bridge** connects the two. It implements SSv2's ZMQ+Protobuf `simulation_interface` protocol, translating each request into CARLA API calls via [carla-rust](https://github.com/jerry73204/carla-rust). This makes CARLA a drop-in replacement for SSv2's built-in simulator or [AWSIM](https://github.com/tier4/AWSIM), letting you run the same `.xosc` scenario files against a high-fidelity environment where Autoware must actually *see* and *react* to the world.

### How it works

<p align="center">
  <img src="docs/architecture.svg" alt="System architecture" width="820">
</p>

Two sides, which meet only in CARLA and the map directory -- no shared ROS domain, host or
start order:

- **Simulation side** -- **SSv2** runs the OpenSCENARIO file, NPC behavior and the verdict;
  **carla-scenario-bridge** (this project) is CARLA's adapter for SSv2's protocol (spawns,
  moves, signals, time); the **agent relay** passes the scenario's goals for the ego to
  whatever drives it.
- **Vehicle side** -- your **Autoware** with this project's sensor kit and vehicle interface:
  **[autoware_carla_bridge](https://github.com/NEWSLabNTU/autoware_carla_bridge)** publishes
  CARLA's sensors, `/clock` and signals and applies Autoware's control; its **vehicle agent**
  takes goals from the relay and drives Autoware's API locally.
- **CARLA** owns physics, sensors and simulation time.

## Getting Started

### Prerequisites

- **CARLA 0.9.16** — [installation guide](https://carla.readthedocs.io/en/0.9.16/start_quickstart/)
- **ROS 2 Humble** — [installation guide](https://docs.ros.org/en/humble/Installation.html)
- **Autoware** — pre-built packages or source install ([docs](https://autowarefoundation.github.io/autoware-documentation/main/installation/))

Clone the repository with submodules:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/carla-scenario-bridge.git
cd carla-scenario-bridge
```

Install build dependencies (Rust toolchain, colcon-cargo, system libraries):

```bash
just install-deps
```

This installs the Rust stable + nightly toolchains, `cargo-nextest`, `colcon-cargo`/`colcon-ros-cargo` (for building Rust ament packages), and system libraries (`libclang-dev`, `protobuf-compiler`, `libzmq3-dev`).

### Build

Source the ROS environment, then build:

```bash
# If using direnv, the environment is sourced automatically.
# Otherwise, source manually:
source /opt/ros/humble/setup.bash

just build
```

### Run

See the **[User Guide](docs/user-guide.md)**: prepare a map directory and a scenario
directory, then

```bash
./CarlaUE4.sh -RenderOffScreen                                        # CARLA
ROS_DOMAIN_ID=9 play_launch launch csb_launch simulation.launch.xml  # bridge + relay
ROS_DOMAIN_ID=1 play_launch launch acb_launch carla_simulator.launch.xml \
    map_path:=<map dir> relay:=tcp://localhost:5560                   # your Autoware
ROS_DOMAIN_ID=9 play_launch launch csb_launch scenario.launch.xml scenario:=<file>.xosc
```

`play_launch launch` is a drop-in for `ros2 launch` with crash recovery and a web UI.
Developers have the same as `just run`, `just ego-av` and `just scenario <file>`.

## Project Structure

```
.
├── src/
│   ├── carla_scenario_bridge/      # Rust crate — ZMQ server + CARLA adapter
│   │   ├── src/
│   │   │   ├── main.rs             # Entry point, CARLA retry loop
│   │   │   ├── zmq_server.rs       # ZMQ REP socket, protobuf dispatch
│   │   │   ├── coordinator.rs      # 14 SSv2 handlers → CARLA API calls
│   │   │   ├── entity_manager.rs   # SSv2 name ↔ CARLA actor ID mapping
│   │   │   ├── coordinate_conversion.rs
│   │   │   └── traffic_light_mapper.rs
│   │   └── build.rs                # prost-build proto compilation
│   ├── csb_launch/                 # simulation, scenario, ego and demo launch files
│   ├── scenario_agent_relay/       # the scenario's side of the vehicle agent protocol
│   ├── autoware_carla_bridge/      # Submodule — sensor/vehicle interface
│   └── scenario_simulator_v2/      # Submodule — scenario framework
├── proto/                          # SSv2 protobuf definitions (8 .proto files)
├── scenarios/                      # Example OpenSCENARIO files
├── docs/
│   ├── design/                     # Architecture, protocol, launch config
│   ├── roadmap/                    # phased roadmap
│   └── user-guide.md
├── justfile                        # Build and run commands
└── Cargo.toml                      # Workspace root
```

## Documentation

- [Architecture](docs/design/architecture.md) — design principles, protocol details, responsibility split
- [SSv2 Protocol Reference](docs/design/ssv2-protocol.md) — the 14 ZMQ message types
- [User Guide](docs/user-guide.md) — install, map and scenario directories, running
- [User workflow and the vehicle agent](docs/design/user-workflow.md) — the two sides, the interfaces, swapping CARLA or Autoware
- [Writing scenarios](docs/design/scenario-authoring.md) — maps, the ego, other vehicles, signals, collisions
- [Roadmap](docs/roadmap/README.md) — phased implementation plan

## Development

### Available commands

```bash
just              # List all recipes
just build        # Build with colcon + cargo
just clean        # Remove build artifacts
just check        # Format check (nightly) + clippy
just test         # Run tests with cargo-nextest
just ci           # Build + check + test
just format       # Auto-format with cargo +nightly fmt
just install-deps # Install build prerequisites
```

## Related Projects

- [autoware_carla_bridge](https://github.com/NEWSLabNTU/ros_zenoh_bridge) — sensor/vehicle interface between CARLA and Autoware
- [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) — Autoware scenario testing framework
- [AWSIM](https://github.com/tier4/AWSIM) — Unity-based simulator with SSv2 support (alternative backend)
- [carla-rust](https://github.com/jerry73204/carla-rust) — Rust bindings for the CARLA C++ API
