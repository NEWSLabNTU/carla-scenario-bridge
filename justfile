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

    # colcon-cargo-ros2 for ament_cargo build type (generates rosidl_cargo
    # bindings for custom msg packages like autoware_adapi_v1_msgs; the older
    # colcon-cargo/colcon-ros-cargo combo only patches prebuilt /opt/ros crates
    # and cannot see workspace-local or Autoware message packages)
    if python3 -c "import colcon_cargo" &>/dev/null || python3 -c "import colcon_ros_cargo" &>/dev/null; then
        echo "Removing conflicting colcon-cargo/colcon-ros-cargo..."
        pip uninstall -y colcon-cargo colcon-ros-cargo
    fi
    if ! python3 -c "import colcon_cargo_ros2" &>/dev/null; then
        echo "Installing colcon-cargo-ros2..."
        pip install colcon-cargo-ros2
    else
        echo "colcon-cargo-ros2 already installed"
    fi

    # pip may have pulled a newer setuptools into ~/.local, which shadows the apt one and
    # breaks colcon --symlink-install. Keep the system setuptools in front.
    setuptools_path=$(python3 -c 'import setuptools; print(setuptools.__file__)')
    case "$setuptools_path" in
        /usr/lib/python3/dist-packages/*) ;;
        *)
            echo ""
            echo "WARNING: setuptools now resolves to $setuptools_path"
            echo "  A pip-installed setuptools shadows the system one and makes every"
            echo "  Python package fail with 'option --editable not recognized'."
            echo "  Run: pip uninstall -y setuptools"
            ;;
    esac

    echo "All prerequisites installed."

# Fail fast if a pip setuptools shadows the system one.
#
# colcon's --symlink-install runs `setup.py develop --editable`, which setuptools removed
# in v80. When a pip-installed setuptools in ~/.local takes precedence over the apt one,
# every Python package in the workspace dies with "option --editable not recognized" --
# a message that says nothing about setuptools. Check up front instead.
_check-setuptools:
    #!/usr/bin/env bash
    set -e
    path=$(python3 -c 'import setuptools; print(setuptools.__file__)')
    case "$path" in
        /usr/lib/python3/dist-packages/*) ;;
        *)
            echo "ERROR: setuptools resolves to $path" >&2
            echo "  Expected the system package (/usr/lib/python3/dist-packages/setuptools)." >&2
            echo "  colcon --symlink-install will fail with 'option --editable not recognized'." >&2
            echo "  Fix: pip uninstall -y setuptools" >&2
            exit 1
            ;;
    esac

# Build all packages
build: _check-setuptools
    #!/usr/bin/env bash
    set -e
    export CARLA_VERSION={{carla_version}}
    #source "{{project}}/install/setup.bash"
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

# Launch the ego's Autoware + acb_bridge in SSv2's ROS domain (no domain override here:
# SSv2's concealer needs plain ROS reachability). Long-lived: start it once, BEFORE any
# `just scenario`, and reuse it across scenario runs.
# Usage: just ego-av [map_path]
ego-av map_path=(data_dir + "/carla-autoware-bridge/" + map_name):
    #!/usr/bin/env bash
    set -e
    # Three overlaid workspaces: base Autoware, then acb (acb_launch, sensor kit,
    # vehicle description, acb_bridge), then this one (csb_launch). The ego stack
    # resolves packages from all three.
    source /opt/autoware/1.5.0/setup.bash
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count - each run randomly failed to match
    # a different ADAPI service. Every ROS process in the pipeline must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # --parser python: play_launch's Rust parser fails on tier4_perception_component
    # (KeyError 'front_overhang' evaluating its Python sub-launches)
    # The API adaptors run as their own processes: inside the big launch the
    # concealer could not match their services in time (see ego_av.launch.xml).
    # internal: serves /api/autoware/set/velocity_limit (what the concealer
    # actually calls); external: /api/external/* including rtc_auto_mode.
    ros2 launch autoware_iv_internal_api_adaptor internal_api_adaptor.launch.py &
    ros2 launch autoware_iv_external_api_adaptor external_api_adaptor.launch.py &
    exec play_launch launch --parser python --web-addr 0.0.0.0:8082 \
        csb_launch ego_av.launch.xml \
        map_path:="{{map_path}}" \
        carla_port:={{carla_port}}

# Run SSv2 scenario (adapter and the ego stack — `just run`, `just ego-av` — must already
# be up; SSv2 no longer launches Autoware)
# Usage: just scenario /path/to/scenario.xosc
scenario scenario_file:
    #!/usr/bin/env bash
    set -e
    # SSv2 no longer launches Autoware, but the interpreter still resolves the
    # sensor/vehicle model description packages from the acb workspace.
    source /opt/autoware/1.5.0/setup.bash
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count - each run randomly failed to match
    # a different ADAPI service. Every ROS process in the pipeline must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # --parser python: scenario_test_runner.launch.py imports launch.actions the
    # Rust parser's embedded Python cannot resolve (EmitEvent)
    exec play_launch launch --parser python --web-addr 0.0.0.0:8081 \
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

# Download pre-converted CARLA maps for Autoware
download-maps:
    "{{project}}/scripts/download_maps.sh"

# Proto bindings are generated automatically by prost-build in build.rs during cargo build.
# No manual generation step needed.
