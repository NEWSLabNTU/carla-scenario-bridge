# carla-scenario-bridge -- SSv2 ZMQ backend for CARLA
set dotenv-load

carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')
carla_port := env_var_or_default('CARLA_PORT', '2000')
ssv2_port := env_var_or_default('SSV2_PORT', '5555')
project := justfile_directory()

# List available recipes
default:
    @just --list

# Build all packages
build:
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    source /opt/autoware/1.5.0/setup.bash
    colcon build \
        --base-paths src \
        --symlink-install \
        --cargo-args --profile dev-release

# Remove build artifacts
clean:
    rm -rf build install log .cargo/config.toml target

# Format code
format:
    #!/usr/bin/env bash
    source install/setup.bash
    cargo +nightly fmt

# Run format check and clippy
check:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    cargo +nightly fmt --check
    cargo clippy --all-targets -- -D warnings

# Run tests
test:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    cargo nextest run --no-tests pass --no-fail-fast

# Run CI checks: build, check (format + clippy), and tests
ci: build check test

# Run the CARLA scenario bridge (connects to CARLA and listens for SSv2)
run:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    ros2 run carla_scenario_bridge carla_scenario_bridge \
        --ros-args \
        -p carla_port:={{carla_port}} \
        -p ssv2_port:={{ssv2_port}}

# Start CARLA simulator as a background service
carla-start:
    #!/usr/bin/env bash
    set -e
    # Reuse autoware_carla_bridge's CARLA management if available
    if [ -f "$HOME/repos/autoware_carla_bridge/scripts/carla_start.sh" ]; then
        "$HOME/repos/autoware_carla_bridge/scripts/carla_start.sh" {{carla_port}}
    else
        echo "CARLA start script not found. Start CARLA manually on port {{carla_port}}."
    fi

# Stop CARLA simulator service
carla-stop:
    #!/usr/bin/env bash
    set -e
    if [ -f "$HOME/repos/autoware_carla_bridge/scripts/carla_stop.sh" ]; then
        "$HOME/repos/autoware_carla_bridge/scripts/carla_stop.sh" {{carla_port}}
    else
        systemctl --user stop "carla-run-{{carla_port}}" 2>/dev/null || true
    fi

# Check CARLA service status
carla-status:
    systemctl --user status "carla-run-{{carla_port}}" || true

# Run SSv2 scenario test runner with CARLA backend
# Usage: just scenario /path/to/scenario.xosc
scenario scenario_file:
    #!/usr/bin/env bash
    set -e
    source /opt/autoware/1.5.0/setup.bash
    source "{{project}}/install/setup.bash"
    ros2 launch scenario_test_runner scenario_test_runner.launch.py \
        architecture_type:=awf/universe/20250130 \
        launch_simple_sensor_simulator:=false \
        port:={{ssv2_port}} \
        scenario:="{{scenario_file}}" \
        sensor_model:=sample_sensor_kit \
        vehicle_model:=sample_vehicle

# Generate protobuf Rust bindings from SSv2 proto files
generate-proto:
    #!/usr/bin/env bash
    set -e
    echo "Generating protobuf bindings..."
    # Proto source from SSv2 (must be cloned or symlinked)
    PROTO_DIR="{{project}}/proto"
    OUT_DIR="{{project}}/src/generated"
    mkdir -p "$OUT_DIR"
    protoc \
        --proto_path="$PROTO_DIR" \
        --rust_out="$OUT_DIR" \
        "$PROTO_DIR/simulation_api_schema.proto"
    echo "Generated to $OUT_DIR"
