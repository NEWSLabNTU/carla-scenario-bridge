# carla-scenario-bridge -- SSv2 ZMQ backend for CARLA
set dotenv-load

carla_version := env_var_or_default('CARLA_VERSION', '0.9.16')
carla_port := env_var_or_default('CARLA_PORT', '2000')
ssv2_port := env_var_or_default('SSV2_PORT', '5555')
map_name := env_var_or_default('MAP_NAME', 'Town01')
# ROS domain for the ego stack, which is also SSv2's: the concealer reaches the ego over
# plain ROS, so `just ego-av` and `just scenario` must agree on this. Domain 0 is left
# free deliberately -- it is where every unconfigured ROS process on the host lands, and
# a stray node there joins the scenario's graph without anyone asking. Background AVs
# (scenario entities with controller `agent`) get 2 and up (see `just bg-av`).
ego_domain := env_var_or_default('EGO_ROS_DOMAIN_ID', '1')
# Where SSv2 and the agent relay run (roadmap 017). Separate from every vehicle's domain:
# the scenario reaches vehicles only through the relay's TCP port (`agent_port`).
scenario_domain := env_var_or_default('SCENARIO_ROS_DOMAIN_ID', '9')
agent_port := env_var_or_default('AGENT_PORT', '5560')
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

# Two Autoware stacks, one scenario, end to end. RECORD=1 for a screencast.
two-av scenario_file=(project + "/scenarios/town01_two_av.xosc"): _require-carla
    #!/usr/bin/env bash
    set -u
    # What "two Autoware" means here: the ego's stack and one background AV's, each a full
    # Autoware in its OWN ROS domain with its own acb_bridge, /clock and vehicle agent. Both
    # are scenario entities (roadmap 017): the ego by isEgo, bg_av_1 by its controller
    # `agent`. csb spawns bg_av_1 with role_name bg_av_1 and drives it through the agent
    # relay -- teleported, then the scenario's goal -- so SSv2 sees it (collisions, reach
    # conditions) and the ego's Autoware perceives real, Autoware-driven traffic.
    #
    # Usage: just two-av [scenario_file]
    #        KEEP_STACKS=1 just two-av        # leave the stacks this run started up afterwards
    #        RECORD=1 just two-av             # screencast to play_log/two-av/two-av.mp4
    #
    # Stacks already running are reused and left running: the simulation side (`just run`),
    # the ego stack (`just ego-av`, web 8082) and the background AV stack (`just bg-av`, web
    # 8083). Only stacks this recipe starts are stopped at the end.
    ego_dom={{ego_domain}}
    logs="{{project}}/play_log/two-av"
    mkdir -p "$logs"
    rec_display="${RECORD_DISPLAY:-:1}"
    launch_rviz=$([ "${RECORD:-0}" = "1" ] && echo true || echo false)
    echo "[two-av] scenario domain {{scenario_domain}}, ego domain $ego_dom, background AV domain 2"

    # The play_launch serving web port $1, if any (stacks are told apart by their port).
    stack_on_port() {
        for p in $(pgrep -x play_launch); do
            tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- "--web-addr [0-9.]*:$1 " \
                && { echo "$p"; return 0; }
        done
        return 1
    }

    # Kill the whole process group of each stack we started, not the launcher alone:
    # play_launch's children outlive it often enough that CLAUDE.md has a section about it.
    ego_pg=""; bg_pg=""
    cleanup() {
        [ -z "$bg_pg$ego_pg" ] && return
        if [ "${KEEP_STACKS:-0}" = "1" ]; then
            echo "[two-av] KEEP_STACKS=1, leaving the stacks this run started up"
            return
        fi
        echo "[two-av] stopping the stacks this run started"
        for pg in $bg_pg $ego_pg; do kill -TERM -"$pg" 2>/dev/null || true; done
        sleep 15
        for pg in $bg_pg $ego_pg; do kill -KILL -"$pg" 2>/dev/null || true; done
    }
    trap cleanup EXIT INT TERM

    # The simulation side (bridge + relay) is long-lived and not part of any scenario launch.
    if ! pgrep -x carla_scenario_ >/dev/null 2>&1; then
        echo "[two-av] no bridge running; starting the simulation side (just run)"
        setsid just run > "$logs/bridge.log" 2>&1 &
        for _ in $(seq 1 120); do
            grep -q "ZMQ server ready" "$logs/bridge.log" 2>/dev/null && break
            sleep 5
        done
    fi
    pgrep -x carla_scenario_ >/dev/null || { echo "[two-av] bridge failed to start; see $logs/bridge.log"; exit 1; }

    # One stack at a time, deliberately: two Autoware stacks building their TensorRT nodes at
    # once pushed both past their load budget ("composable 88/89 loaded (1 pending)").
    wait_for_stack() {
        local name="$1" pg="$2" deadline=$((SECONDS + 1200))
        until grep -q "Startup complete" "$logs/$name.log" 2>/dev/null; do
            if ! kill -0 -"$pg" 2>/dev/null; then
                echo "[two-av] $name stack exited before starting up; see $logs/$name.log"
                tail -3 "$logs/$name.log"
                return 1
            fi
            if [ $SECONDS -gt $deadline ]; then
                echo "[two-av] $name stack did not come up within 1200s; see $logs/$name.log"
                grep -oE "waiting on [0-9]+ member\(s\): [^.]*" "$logs/$name.log" | tail -1
                return 1
            fi
            sleep 10
        done
        echo "[two-av] $name stack up at $(date +%T)"
    }

    if stack_on_port 8082 >/dev/null; then
        echo "[two-av] reusing the running ego stack (web 8082)"
    else
        echo "[two-av] starting the ego stack (several minutes)"
        setsid env LAUNCH_RVIZ="$launch_rviz" DISPLAY="$rec_display" \
            just ego-av > "$logs/ego.log" 2>&1 &
        ego_pg=$!
        wait_for_stack ego "$ego_pg" || exit 1
    fi

    if stack_on_port 8083 >/dev/null; then
        echo "[two-av] reusing the running background AV stack (web 8083)"
    else
        echo "[two-av] starting the background AV stack"
        setsid just bg-av > "$logs/bg.log" 2>&1 &
        bg_pg=$!
        wait_for_stack bg "$bg_pg" || exit 1
    fi

    # Each stack has to be able to drive before a scenario is worth starting, and the
    # background AV's agent has to be registered: its goal fails the scenario otherwise.
    just _require-ego-stack || exit 1
    just _require-agent bg_av_1 || exit 1

    # RECORD=1 grabs the display the ego stack's RViz is drawing on (LAUNCH_RVIZ above; a
    # standalone rviz2 cannot load autoware.rviz without the stack's vehicle parameters).
    # RECORDING CHANGES THE RESULT ON THIS MACHINE (RViz on llvmpipe costs ~4 cores): do
    # not record a run you intend to judge.
    rec_pid=""
    stop_recording() {
        # SIGINT, not SIGKILL: mp4 writes its index when the muxer closes.
        [ -n "$rec_pid" ] && kill -INT -"$rec_pid" 2>/dev/null && sleep 5
        [ -n "$rec_pid" ] && echo "[two-av] screencast: $logs/two-av.mp4"
    }
    if [ "${RECORD:-0}" = "1" ]; then
        geom=$(DISPLAY="$rec_display" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')
        if [ -z "$geom" ]; then
            echo "[two-av] RECORD=1 but display $rec_display is not usable; skipping the screencast"
        else
            echo "[two-av] recording $rec_display ($geom) to $logs/two-av.mp4"
            setsid ffmpeg -y -f x11grab -video_size "$geom" -framerate 8 -i "$rec_display.0" \
                -vf scale=1280:-2 -c:v libx264 -preset veryfast -crf 30 -pix_fmt yuv420p \
                "$logs/two-av.mp4" > "$logs/ffmpeg.log" 2>&1 &
            rec_pid=$!
        fi
    fi

    echo "[two-av] running $(basename "{{scenario_file}}")"
    junit=/tmp/scenario_test_runner/result.junit.xml
    rm -f "$junit"
    just scenario "{{scenario_file}}" 2>&1 | tail -5 || true

    stop_recording
    if [ -f "$junit" ]; then
        if grep -q 'failures="0" errors="0"' "$junit"; then
            echo "[two-av] scenario PASSED"
        else
            echo "[two-av] scenario FAILED -- junit:"
            sed -n "1,6p" "$junit"
        fi
    else
        echo "[two-av] no junit written: the run did not reach a verdict"
    fi

# Succeed if an agent is registered with the agent relay as `entity` (the relay's commander
# `query`; docs/design/user-workflow.md). A scenario entity with controller `agent` fails its
# first goal without one.
_require-agent entity:
    #!/usr/bin/env python3
    import json, socket, sys
    try:
        s = socket.create_connection(("localhost", {{agent_port}}), timeout=3)
    except OSError as e:
        sys.exit(f"[just] agent relay on port {{agent_port}} unreachable ({e}); run `just run`")
    f = s.makefile("rw")
    for m in ({"type": "register", "role": "commander", "agent": "just"},
              {"type": "query", "id": 1, "entity": "{{entity}}"}):
        f.write(json.dumps({**m, "v": 1}) + "\n")
    f.flush()
    for line in f:
        m = json.loads(line)
        if m.get("type") == "reply":
            if m.get("registered"):
                print(f"[just] agent for '{{entity}}' registered "
                      f"({(m.get('state') or {}).get('phase', 'no state yet')})")
                sys.exit(0)
            sys.exit("[just] no agent registered as '{{entity}}': start its vehicle side "
                     "(`just bg-av {{entity}}`) and wait for 'Startup complete'")
    sys.exit("[just] the relay closed the connection")

# Run the CARLA scenario bridge adapter only
run:
    #!/usr/bin/env bash
    # As users start it (roadmap 017): csb_launch simulation.launch.xml under play_launch,
    # which restarts the bridge on a crash (respawn="true"). `just build` after changes.
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # The simulation side: the bridge and the agent relay, in the scenario domain.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export ROS_DOMAIN_ID={{scenario_domain}}
    exec play_launch launch --enforce-rules off --web-addr 0.0.0.0:8084 \
        --log-dir play_log/bridge \
        csb_launch simulation.launch.xml agent_port:={{agent_port}} \
        carla_host:="${CARLA_HOST:-localhost}" carla_port:={{carla_port}} ssv2_port:={{ssv2_port}} \
        ${CSB_CONFIG_DIR:+config_file:="$CSB_CONFIG_DIR/bridge_config.yaml"}

# Start CARLA simulator as a background service
carla-start:
    "{{acb_src}}/scripts/carla_start.sh" {{carla_port}}

# Stop CARLA simulator service
carla-stop:
    "{{acb_src}}/scripts/carla_stop.sh" {{carla_port}}

# Check CARLA service status
carla-status:
    systemctl --user status "carla-run-{{carla_port}}" || true

# Measure a CARLA blueprint's geometry and print it as Autoware vehicle_info parameters.
#
# Pass --write to update acb_vehicle_description's vehicle_info.param.yaml, --list to see the
# available blueprints. Refuses to run while CARLA is in synchronous mode, because settling a
# spawned car there would steal the tick from a running scenario.
#
# Usage: just vehicle-params [blueprint] [args...]
#        just vehicle-params vehicle.audi.etron --write
vehicle-params blueprint="vehicle.tesla.model3" *args:
    "{{acb_src}}/scripts/extract_vehicle_params.py" {{blueprint}} --port {{carla_port}} {{args}}

# Run a scenario against a live stack and judge it, with the reasons attached.
#
# Needs `just run` and `just ego-av` already up -- it judges a stack, it does not build one.
# Checks the scenario's own verdict, that the ego actually drove somewhere, that no node died,
# and reports non-OK diagnostics. Exit status is 0 only if every run passed every check.
#
# Usage: just acceptance [scenario] [runs]
#        just acceptance scenarios/town01_ego_drive.xosc 3
acceptance scenario=(project + "/scenarios/town01_ego_drive.xosc") runs="1" domain="":
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    args=(--scenario "{{scenario}}" --runs {{runs}})
    # acceptance.py watches the ego's domain (default 1, `ego_domain`).
    [ -n "{{domain}}" ] && args+=(--domain "{{domain}}")
    python3 "{{acb_src}}/scripts/acceptance.py" "${args[@]}"

# Generate an Autoware point cloud map from the town CARLA currently has loaded.
#
# Takes CARLA over: it switches the server to synchronous mode with rendering off, spawns
# LiDARs, teleports them across every spawn point, and restores the settings afterwards. So
# stop the bridge and any scenario first, or it will fight them for the tick.
#
# Writes pointcloud_map.pcd only. A usable map directory also needs lanelet2_map.osm,
# map_config.yaml and map_projector_info.yaml, which come from the map conversion tooling or
# from an existing map for the same town.
#
# Usage: just pcd-gen [out_dir] [extra args...]
#        just pcd-gen /tmp/Town01 --voxel-size 0.1
pcd-gen out_dir *args:
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    ros2 run carla_pcd_gen carla_pcd_gen --map-dir "{{out_dir}}" {{args}}

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
# Refuse to start a scenario before the ego's Autoware is up.
#
# With launch_autoware:=false the concealer attaches to an Autoware it did not launch, and
# its constructor queues a ChangeToStop against an ADAPI service with a 180 s timeout. Start
# a scenario too early and the run does not fail fast: it evaluates nothing and dies three
# minutes later with an AutowareError naming a service rather than the ordering mistake.
_require-ego-stack:
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # Look where the ego actually is: its own domain (roadmap 017).
    export ROS_DOMAIN_ID={{ego_domain}}
    # --reload-failed: play_launch does not respawn a composable that crashed inside its
    # container, and behavior_path_planner does that about once in 60 route resets
    # (autoware_universe#12460, an rclcpp race with no upstream fix). Without it the stack
    # looks up, the ego spawns and never moves, and the run dies at the storyboard timeout.
    if ! "{{project}}/scripts/ego_stack_health.py" --reload-failed; then
        echo "[just] Refusing to start: run \`just ego-av\` first and wait for"
        echo "[just] 'Startup complete'. See phase 012, startup order."
        exit 1
    fi

_require-carla:
    #!/usr/bin/env bash
    set -e
    if ! "{{acb_src}}/scripts/carla_health.py" --port {{carla_port}}; then
        echo "[just] Refusing to start: a run against a dead or wedged CARLA produces a full"
        echo "[just] set of logs and a verdict that means nothing. See docs/issues/017."
        exit 1
    fi

# Launch the ego's Autoware + acb_bridge + vehicle agent in the ego domain
# (`ego_domain`). Long-lived: start it once and reuse it across scenario runs; it reaches
# the scenario through the agent relay `just run` starts (EGO_RELAY, default
# tcp://localhost:<agent_port>; empty with EGO_GOAL_POSES_FILE drives to a local goal).
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
    goal_poses_file="${EGO_GOAL_POSES_FILE:-}"
    clock="${ACB_PUBLISH_CLOCK:-true}"
    # Any domain works since roadmap 017: the scenario reaches this ego through the agent
    # relay (TCP), not over ROS. Kept apart from the scenario domain on purpose.
    export ROS_DOMAIN_ID={{ego_domain}}
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
    # A ROS launch argument cannot carry an empty value -- `goal_poses_file:=` is rejected
    # as "malformed launch argument", and the failure comes minutes in, after the whole
    # parse. The launch file already declares a default for it, so pass the argument only
    # when there is something to pass.
    optional_args=()
    if [ -n "$goal_poses_file" ]; then
        optional_args+=(goal_poses_file:="$goal_poses_file")
    fi
    # The agent initializes localization at the scenario's start pose (teleported); seeding
    # from CARLA on attach as well would race it.
    seed_default=false
    # CONTROL_TRACE_PATH writes per-stage control latency as CSV; see acb's control_trace.
    if [ -n "${CONTROL_TRACE_PATH:-}" ]; then
        optional_args+=(control_trace_path:="$CONTROL_TRACE_PATH")
    fi
    # The pedal maps have the same trap from the other direction: these use ${VAR-default}
    # rather than ${VAR:-default}, so `ACCEL_MAP_PATH= just ego-av` passes an empty value
    # and fails the same way. The bridge spells "off" as `none`, so map empty onto that.
    accel_map="${ACCEL_MAP_PATH-$(ros2 pkg prefix --share acb_vehicle_description)/config/accel_map.csv}"
    brake_map="${BRAKE_MAP_PATH-$(ros2 pkg prefix --share acb_vehicle_description)/config/brake_map.csv}"
    accel_map="${accel_map:-none}"
    brake_map="${brake_map:-none}"
    # acb publishes signal state from CARLA's lights (roadmap 015) from the table csb writes
    # beside the Lanelet2 map at every Initialize: this map_path's real directory, the one
    # the scenario's map symlinks resolve to. TRAFFIC_LIGHT_MAP_PATH overrides it (csb logs
    # where it wrote, e.g. its fallback for a read-only map dir); empty turns it off.
    tl_map="${TRAFFIC_LIGHT_MAP_PATH-{{map_path}}/carla/traffic_lights.yaml}"
    tl_map="${tl_map:-none}"
    # Not exec: the trap above has to survive to clean up the API adaptors.
    # --enforce-rules off: play_launch 0.12 defaults to `warn`, which turns on LD_PRELOAD
    # interception of every DDS take and logs each event at DEBUG into play_launch.log and
    # interception/events.jsonl. With no contract files to enforce that is pure overhead, and
    # on 2026-09-27 one 50-minute ego stack wrote 169 GB of it and filled the disk.
    #
    # --composable-respawn on-crash (play_launch >= 873b0744): reload a composable whose
    # isolated process crashed (behavior_path_planner, autoware_universe#12460), backing off
    # and giving up after 5 crashes in 300 s. Autoware's containers declare no respawn, so
    # `inherit` would never fire. The scenario gate's ego_stack_health.py --reload-failed
    # stays as the backstop for a composable that gave up.
    play_launch launch --enforce-rules off --parser python --web-addr 0.0.0.0:8082 \
        --composable-respawn on-crash \
        --load-node-timeout 120 \
        --load-total-budget 600 \
        --log-dir play_log/ego \
        csb_launch ego_av.launch.xml \
        map_path:="{{map_path}}" \
        carla_port:={{carla_port}} \
        relay:="${EGO_RELAY-tcp://localhost:{{agent_port}}}" \
        publish_clock:=$clock \
        report_measured_steering:="${REPORT_MEASURED_STEERING:-false}" \
        launch_rviz:="${LAUNCH_RVIZ:-false}" \
        steering_multiplier:="${STEERING_MULTIPLIER:-1.0}" \
        steer_rate_limit_deg_s:="${STEER_RATE_LIMIT_DEG_S:-0.0}" \
        steer_time_constant_s:="${STEER_TIME_CONSTANT_S:-0.0}" \
        publish_ground_truth_objects:="${GROUND_TRUTH_OBJECTS:-false}" \
        seed_localization_on_attach:="${SEED_LOCALIZATION:-$seed_default}" \
        ground_truth_range_m:="${GROUND_TRUTH_RANGE_M:-100.0}" \
        traffic_light_map_path:="$tl_map" \
        accel_map_path:="$accel_map" \
        brake_map_path:="$brake_map" \
        "${optional_args[@]}"

# Launch one background AV's vehicle side -- Autoware + acb_bridge + vehicle agent -- in its
# own ROS domain. The background AV is a scenario entity whose controller is `agent`
# (scenarios/town01_two_av.xosc): csb spawns it with role_name = the entity name, and this
# stack's agent registers with the agent relay under that name and takes the scenario's goals.
# Long-lived, like `just ego-av`, and startable before or after the scenario.
#
# Domains start at 2: 9 is the scenario's, 1 the ego's, and 0 is left free so that a stray
# unconfigured ROS process cannot join a run's graph. A second background AV goes on 3 with
# its own web port: `just bg-av bg_av_2 3 8085`.
#
# Usage: just bg-av [entity] [domain] [web_port] [map_path]
bg-av entity="bg_av_1" domain="2" web_port="8083" map_path=(data_dir + "/carla-autoware-bridge/" + map_name): _require-carla
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Same loopback-unicast DDS config as every other process in the pipeline. DomainGain
    # 1000 in it is what keeps domain 1's unicast ports out of domain 0's range.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export ROS_DOMAIN_ID={{domain}}
    # BG_RELAY empty with BG_GOAL_POSES_FILE drives once to a local goal (no scenario).
    # play_launch rejects an empty `name:=`, so optional arguments go in only when set.
    optional_args=()
    [ -n "${BG_GOAL_POSES_FILE:-}" ] && optional_args+=(goal_poses_file:="$BG_GOAL_POSES_FILE")
    exec play_launch launch --enforce-rules off --parser python --web-addr 0.0.0.0:{{web_port}} \
        --composable-respawn on-crash \
        --load-node-timeout 120 \
        --load-total-budget 180 \
        --log-dir play_log/bg-{{entity}} \
        csb_launch background_av.launch.xml \
        entity:={{entity}} \
        relay:="${BG_RELAY-tcp://localhost:{{agent_port}}}" \
        map_path:="{{map_path}}" \
        carla_port:={{carla_port}} \
        "${optional_args[@]}"

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
scenario scenario_file: _require-carla _require-ego-stack _clear-stale-scenario
    #!/usr/bin/env bash
    set -e
    # The interpreter resolves the vehicle description package (acb_vehicle_description).
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count. Every ROS process must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # SSv2 runs in the scenario domain with the agent relay (`just run`); the ego is
    # reached through the relay, wherever it runs (roadmap 017).
    export ROS_DOMAIN_ID={{scenario_domain}}
    # The repo's scenarios name their map as $(env CARLA_MAPS)/<Town>.
    export CARLA_MAPS="${CARLA_MAPS:-{{data_dir}}/carla-autoware-bridge}"
    # The scenario file is read, never modified: SSv2's runner preprocesses a copy in its
    # output directory (fork, roadmap 017).
    # --parser python: scenario_test_runner.launch.py imports launch.actions the
    # Rust parser's embedded Python cannot resolve (EmitEvent)
    exec play_launch launch --enforce-rules off --parser python --web-addr 0.0.0.0:8081 \
        --log-dir play_log/scenario \
        csb_launch scenario.launch.xml \
        scenario:="$(realpath "{{scenario_file}}")" \
        port:={{ssv2_port}}

# Run the full stack: adapter + bridge + SSv2 + Autoware (CARLA must be running)
# Usage: just e2e [scenario_file]
e2e scenario_file=(project + "/scenarios/town01_ego_drive.xosc") map_name=map_name:
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export ROS_DOMAIN_ID={{ego_domain}}
    export CARLA_MAPS="${CARLA_MAPS:-{{data_dir}}/carla-autoware-bridge}"
    exec play_launch launch --enforce-rules off --parser python --web-addr 0.0.0.0:8080 \
        csb_launch demo.launch.xml \
        scenario:="$(realpath "{{scenario_file}}")" \
        map_path:="$CARLA_MAPS/{{map_name}}" \
        carla_port:={{carla_port}} \
        ssv2_port:={{ssv2_port}}

# Download pre-converted CARLA maps for Autoware
download-maps:
    "{{project}}/scripts/download_maps.sh"

# Proto bindings are generated automatically by prost-build in build.rs during cargo build.
# No manual generation step needed.
