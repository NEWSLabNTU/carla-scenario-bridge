# carla-scenario-bridge -- SSv2 ZMQ backend for CARLA
set dotenv-load

carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')
carla_port := env_var_or_default('CARLA_PORT', '2000')
ssv2_port := env_var_or_default('SSV2_PORT', '5555')
map_name := env_var_or_default('MAP_NAME', 'Town01')
# ROS domain for the ego stack, which is also SSv2's: the concealer reaches the ego over
# plain ROS, so `just ego-av` and `just scenario` must agree on this. Domain 0 is left
# free deliberately -- it is where every unconfigured ROS process on the host lands, and
# a stray node there joins the scenario's graph without anyone asking. Background AVs get
# 2 and up (see `just bg-av`).
ego_domain := env_var_or_default('EGO_ROS_DOMAIN_ID', '1')
data_dir := env_var_or_default('DATA_DIR', justfile_directory() + '/data')
project := justfile_directory()
acb_src := justfile_directory() + '/src/autoware_carla_bridge'
# Where Autoware's own setup.bash lives. The NEWSLabNTU 1.5.0 Debian installs under
# /opt/autoware/1.5.0; other hosts install the same packages straight into the ROS
# prefix (/opt/ros/humble). Override with AUTOWARE_SETUP when yours is elsewhere.
autoware_setup := env_var_or_default('AUTOWARE_SETUP', '/opt/autoware/1.5.0/setup.bash')

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
    # carla-rust exposes a different API per CARLA version, selected by CARLA_VERSION:
    # 0.9.x has WheelPhysicsControl::position, 0.10 has offset. Without this export
    # clippy compiles against 0.10 while `just build` and `just run` compile against
    # 0.9.16, so `just check` can pass on code the real build rejects -- and can reject
    # code the real build accepts, which is how a correct field access was "fixed" into
    # a broken one and back again.
    export CARLA_VERSION={{carla_version}}
    cargo +nightly fmt --check
    cargo clippy --all-targets -- -D warnings

# Run tests
test:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    # Same reason as `check`: tests must compile against the CARLA API the build uses.
    export CARLA_VERSION={{carla_version}}
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
    # Without this the bridge looks for bridge_config.yaml in ./config -- which exists
    # (it holds the DDS xml) but has no bridge config in it, so the run silently comes up
    # with no background AVs and no blueprint aliases.
    export CSB_CONFIG_DIR="${CSB_CONFIG_DIR:-{{project}}/src/carla_scenario_bridge/config}"
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

# Report whether CARLA is fit to run a scenario against, and why if not.
carla-health:
    "{{acb_src}}/scripts/carla_health.py" --port {{carla_port}}

# Refuse to start a run against a CARLA that is missing or wedged.
#
# `pgrep CarlaUE4` is not enough, and neither is counting actors: a healthy server in
# synchronous mode reports zero actors to a client that has seen no tick, which reads exactly
# like a wedged one. That mistake cost two unnecessary restarts and a bogus entry in
# docs/issues/017. carla_health.py reads the mode first and only trusts the census where it
# means something.
_require-carla:
    #!/usr/bin/env bash
    set -e
    if ! "{{acb_src}}/scripts/carla_health.py" --port {{carla_port}}; then
        echo "[just] Refusing to start: a run against a dead or wedged CARLA produces a full"
        echo "[just] set of logs and a verdict that means nothing. See docs/issues/017."
        exit 1
    fi

# Launch the ego's Autoware + acb_bridge in SSv2's ROS domain (no domain override here:
# SSv2's concealer needs plain ROS reachability). Long-lived: start it once, BEFORE any
# `just scenario`, and reuse it across scenario runs.
# Usage: just ego-av [map_path]
ego-av map_path=(data_dir + "/carla-autoware-bridge/" + map_name): _require-carla
    #!/usr/bin/env bash
    set -e
    # Three overlaid workspaces: base Autoware, then acb (acb_launch, sensor kit,
    # vehicle description, acb_bridge), then this one (csb_launch). The ego stack
    # resolves packages from all three.
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count - each run randomly failed to match
    # a different ADAPI service. Every ROS process in the pipeline must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # play_launch forks a process per composable node and SIGKILLs it if it has not
    # reported ready within this window. The default is 30 s, and Autoware's traffic
    # light classifier needs ~45 s to construct even with its TensorRT engine cached
    # (~33 s of that is the engine build itself on a cold cache). At the default the
    # three inference nodes were killed seconds before they would have reported, and
    # the only symptom was a perception pipeline publishing empty results forever.
    # The ego and SSv2 share this domain; `just scenario` sets the same one. Not 0 --
    # see the ego_domain comment at the top of this file.
    export ROS_DOMAIN_ID={{ego_domain}}
    # --parser python: play_launch's Rust parser fails on tier4_perception_component
    # (KeyError 'front_overhang' evaluating its Python sub-launches)
    # The API adaptors run as their own processes: inside the big launch the
    # concealer could not match their services in time (see ego_av.launch.xml).
    # internal: serves /api/autoware/set/velocity_limit (what the concealer
    # actually calls); external: /api/external/* including rtc_auto_mode.
    # These outlive the recipe unless something kills them: the shell used to exec
    # play_launch, replacing itself, so no trap could ever fire and every restart of this
    # stack left another pair behind. Sixteen of them accumulated in one session, each
    # holding the same ADAPI node names, and a fresh stack then stalled with
    # /adapi/node/autoware_state stuck "pending" and the concealer never seeing
    # WAITING_FOR_ENGAGE. Own them: no exec below, and a trap that takes the whole process
    # group so the nodes go with the launcher.
    # setsid, so each adaptor leads its own process group and the trap can take the whole
    # tree. Killing the `ros2 launch` process alone is not enough: the nodes it spawns
    # survive it, and their command lines look nothing like the launch file's --
    # `--plugin external_api::RTCController`, `__ns:=/default_adapi/helpers` -- so they
    # also survive every pkill aimed at "api_adaptor". Sixty-four of them were found alive
    # in one domain, five deep on /internal/operator and /internal/velocity.
    setsid ros2 launch autoware_iv_internal_api_adaptor internal_api_adaptor.launch.py &
    internal_api_pid=$!
    setsid ros2 launch autoware_iv_external_api_adaptor external_api_adaptor.launch.py &
    external_api_pid=$!
    trap 'kill -- -$internal_api_pid -$external_api_pid 2>/dev/null' EXIT INT TERM
    # --load-node-timeout 120: the startup burst (93 composables + CARLA on one
    # host) can push a container's first LoadNode reply past the 30 s default;
    # a timed-out load falls into play_launch's awaiting-ComponentEvent limbo
    # and the member stays "pending" forever, silently missing ADAPI services.
    # --load-total-budget 600: this is the per-composable budget while a container is
    # busy, and it is what actually kills the traffic-light inference nodes. At 180 the
    # car classifier, the pedestrian classifier and the fine detector were reported
    # "still constructing" at 90 s, 125 s and 160 s and then LOAD_FAILED at exactly
    # 180 s -- three of 89 composables missing, with the rest of the stack healthy.
    #
    # The symptom is not an error anywhere downstream. The map-based detector still
    # publishes regions of interest, fusion still publishes, and the classifier simply
    # never publishes at all: a full 215 m drive past a commanded red produced regions
    # on 27 of 139 samples and not one colour, because the node that assigns colours was
    # never loaded. play_launch's own help names this case -- "raise for containers whose
    # composable ctors block for minutes (e.g. TensorRT engine builds)" -- and 600 is its
    # default.
    #
    # The cost is that a genuinely lost load self-heals in ten minutes rather than three.
    # That is the right trade here: a lost load is rare and visible, while a silently
    # missing classifier looks like a perception problem and has cost days.
    # Not exec: the trap above has to survive to clean up the API adaptors.
    play_launch launch --parser python --web-addr 0.0.0.0:8082 \
        --load-node-timeout 120 \
        --load-total-budget 600 \
        csb_launch ego_av.launch.xml \
        map_path:="{{map_path}}" \
        carla_port:={{carla_port}} \
        report_measured_steering:="${REPORT_MEASURED_STEERING:-false}" \
        steering_multiplier:="${STEERING_MULTIPLIER:-1.0}" \
        publish_ground_truth_objects:="${GROUND_TRUTH_OBJECTS:-false}" \
        ground_truth_range_m:="${GROUND_TRUTH_RANGE_M:-100.0}"

# Launch one background AV's Autoware + acb_bridge + pilot in its own ROS domain.
# The bridge spawns the vehicle (see background_avs in bridge_config.yaml); this brings up
# the stack that drives it. Long-lived, like `just ego-av`, and startable before or after
# the scenario -- only CARLA has to be up first.
#
#
# Domains start at 2: 1 belongs to the ego and SSv2, and 0 is left free so that a stray
# unconfigured ROS process cannot join a run's graph. A second background AV goes on 3.
#
# Restart this before every scenario run. SSv2 restarts sim time at ~0 each run, and a
# stack that has already seen a later clock stalls on the backward jump -- its pilot then
# waits for a fresh pose that never arrives. See docs/CHECKPOINT.md.
#
# Usage: just bg-av [vehicle_name] [domain] [web_port]
bg-av vehicle_name="bg_av_1" domain="2" web_port="8083" map_path=(data_dir + "/carla-autoware-bridge/" + map_name) goals=(project + "/scenarios/bg_av_1_poses.yaml"): _require-carla
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Same loopback-unicast DDS config as every other process in the pipeline. DomainGain
    # 1000 in it is what keeps domain 1's unicast ports out of domain 0's range.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # play_launch forks a process per composable node and SIGKILLs it if it has not
    # reported ready within this window. The default is 30 s, and Autoware's traffic
    # light classifier needs ~45 s to construct even with its TensorRT engine cached
    # (~33 s of that is the engine build itself on a cold cache). At the default the
    # three inference nodes were killed seconds before they would have reported, and
    # the only symptom was a perception pipeline publishing empty results forever.
    export ROS_DOMAIN_ID={{domain}}
    # Off the ego stack's web port; the concealer's own default is 8082.
    export PLAY_LAUNCH_WEB_ADDR=0.0.0.0:{{web_port}}
    exec play_launch launch --parser python --web-addr 0.0.0.0:{{web_port}} \
        --load-node-timeout 120 \
        --load-total-budget 180 \
        csb_launch background_av.launch.xml \
        vehicle_name:={{vehicle_name}} \
        map_path:="{{map_path}}" \
        goal_poses_file:="{{goals}}" \
        carla_port:={{carla_port}}

# Run SSv2 scenario (adapter and the ego stack — `just run`, `just ego-av` — must already
# be up; SSv2 no longer launches Autoware)
# Clear leftovers from a previous scenario run.
#
# An interrupted run -- Ctrl-C, a timeout, a killed harness -- can leave its interpreter
# alive. It keeps the /simulation namespace and the simulator connection, so the *next*
# run's interpreter starts, reports "all nodes ready", and then never initialises: the
# adapter receives no requests, no ego is spawned, and the scenario hangs until its global
# timeout with nothing in any log to explain it. That failure mode cost several hours before
# it was tracked down, and the only visible hint was a duplicate `Address already in use` on
# the web UI port.
#
# Matching is deliberately narrow. `carla_scenario.launch.xml` belongs to this recipe, so
# `just ego-av` (ego_av.launch.xml, its own web port) is never touched -- killing the ego
# stack here would be far worse than the problem being fixed.
_clear-stale-scenario:
    #!/usr/bin/env bash
    set -u
    stale=0
    for pat in "carla_scenario.launch.xml" "openscenario_interpreter_node" \
               "openscenario_preprocessor" "scenario_test_runner"; do
        for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
            # Never the shell doing the killing, nor its parents: pgrep -f matches full
            # command lines, and a shell's own line usually contains the pattern it was
            # asked to search for.
            [ "$pid" = "$$" ] && continue
            [ "$pid" = "${PPID:-0}" ] && continue
            cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || continue
            case "$cmd" in *just*|*pgrep*) continue ;; esac
            kill -9 "$pid" 2>/dev/null && stale=$((stale + 1))
        done
    done
    if [ "$stale" -gt 0 ]; then
        echo "[scenario] cleared $stale stale process(es) from a previous run"
        # ROS needs a moment to drop the old graph entries, or the new interpreter can
        # still collide with a name that is on its way out.
        sleep 3
    fi

# Usage: just scenario /path/to/scenario.xosc
scenario scenario_file: _require-carla _clear-stale-scenario
    #!/usr/bin/env bash
    set -e
    # SSv2 no longer launches Autoware, but the interpreter still resolves the
    # sensor/vehicle model description packages from the acb workspace.
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count - each run randomly failed to match
    # a different ADAPI service. Every ROS process in the pipeline must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # play_launch forks a process per composable node and SIGKILLs it if it has not
    # reported ready within this window. The default is 30 s, and Autoware's traffic
    # light classifier needs ~45 s to construct even with its TensorRT engine cached
    # (~33 s of that is the engine build itself on a cold cache). At the default the
    # three inference nodes were killed seconds before they would have reported, and
    # the only symptom was a perception pipeline publishing empty results forever.
    # Must match `just ego-av`: the concealer reaches the ego over plain ROS, and a
    # scenario in another domain simply never finds it.
    export ROS_DOMAIN_ID={{ego_domain}}
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
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # play_launch forks a process per composable node and SIGKILLs it if it has not
    # reported ready within this window. The default is 30 s, and Autoware's traffic
    # light classifier needs ~45 s to construct even with its TensorRT engine cached
    # (~33 s of that is the engine build itself on a cold cache). At the default the
    # three inference nodes were killed seconds before they would have reported, and
    # the only symptom was a perception pipeline publishing empty results forever.
    export ROS_DOMAIN_ID={{ego_domain}}
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
