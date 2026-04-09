# carla-scenario-bridge -- SSv2 ZMQ backend for CARLA
set dotenv-load

carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')
carla_port := env_var_or_default('CARLA_PORT', '2000')
ssv2_port := env_var_or_default('SSV2_PORT', '5555')
map_name := env_var_or_default('MAP_NAME', 'Town01')
data_dir := env_var_or_default('DATA_DIR', justfile_directory() + '/data')
project := justfile_directory()
acb_src := justfile_directory() + '/src/autoware_carla_bridge'

# List available recipes
default:
    @just --list

# Install prerequisites (Rust toolchain, colcon-cargo, system libraries)
install-deps:
    #!/usr/bin/env bash
    set -e

    # Rust toolchain (via rustup)
    if ! command -v rustup &>/dev/null; then
        echo "Installing Rust toolchain via rustup..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        source "$HOME/.cargo/env"
    else
        echo "Rust toolchain already installed ($(rustc --version))"
    fi

    # Nightly toolchain (for cargo fmt)
    if ! rustup toolchain list | grep -q nightly; then
        echo "Installing Rust nightly toolchain..."
        rustup toolchain install nightly
    else
        echo "Rust nightly toolchain already installed"
    fi

    # cargo-nextest (for just test)
    if ! command -v cargo-nextest &>/dev/null; then
        echo "Installing cargo-nextest..."
        cargo install cargo-nextest --locked
    else
        echo "cargo-nextest already installed"
    fi

    # System libraries
    echo "Installing system libraries..."
    sudo apt-get update
    sudo apt-get install -y \
        libclang-dev \
        protobuf-compiler \
        libzmq3-dev

    # colcon-cargo for ament_cargo build type
    if ! python3 -c "import colcon_cargo" &>/dev/null || ! python3 -c "import colcon_ros_cargo" &>/dev/null; then
        echo "Installing colcon-cargo and colcon-ros-cargo..."
        pip install colcon-cargo colcon-ros-cargo
    else
        echo "colcon-cargo and colcon-ros-cargo already installed"
    fi

    echo "All prerequisites installed."

# Build all packages
build:
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    source "{{project}}/install/setup.bash"
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
    colcon build \
        --base-paths src \
        --symlink-install \
        --cargo-args --profile dev-release \
        --cmake-args -DBUILD_TESTING=OFF

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

# Run the CARLA scenario bridge adapter only
run:
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    export CARLA_HOST="${CARLA_HOST:-localhost}"
    export CARLA_PORT="{{carla_port}}"
    export SSV2_PORT="{{ssv2_port}}"
    cargo run \
        --manifest-path "{{project}}/src/carla_scenario_bridge/Cargo.toml"

# Start CARLA simulator as a background service
carla-start:
    "{{acb_src}}/scripts/carla_start.sh" {{carla_port}}

# Stop CARLA simulator service
carla-stop:
    "{{acb_src}}/scripts/carla_stop.sh" {{carla_port}}

# Check CARLA service status
carla-status:
    systemctl --user status "carla-run-{{carla_port}}" || true

# Run SSv2 scenario (adapter + bridge must be running separately)
# Usage: just scenario /path/to/scenario.xosc
scenario scenario_file:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8081 \
        csb_launch carla_scenario.launch.xml \
        scenario:="{{scenario_file}}" \
        port:={{ssv2_port}}

# Run the full stack: adapter + bridge + SSv2 + Autoware (CARLA must be running)
# Usage: just e2e [scenario_file]
e2e scenario_file=(project + "/scenarios/town01_ego_drive.xosc"):
    #!/usr/bin/env bash
    set -e
    source "{{project}}/install/setup.bash"
    exec play_launch launch --web-addr 0.0.0.0:8080 \
        csb_launch demo.launch.xml \
        scenario:="{{scenario_file}}" \
        carla_port:={{carla_port}} \
        ssv2_port:={{ssv2_port}}

# Proto bindings are generated automatically by prost-build in build.rs during cargo build.
# No manual generation step needed.
