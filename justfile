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
# Where an UNMANAGED ego lives (phase 013). With the concealer inert nothing in SSv2 talks
# to this stack, so it leaves SSv2's domain and SSv2's contains only SSv2. Distinct from
# SSv2 (1) and from bg_av_1 (2) so all three can run at once.
ego_unmanaged_domain := env_var_or_default('EGO_UNMANAGED_ROS_DOMAIN_ID', '3')
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
    # Autoware in its OWN ROS domain with its own acb_bridge and its own /clock. SSv2 knows
    # only about the ego -- background AVs are spawned by csb_bridge from bridge_config.yaml
    # and are invisible to the scenario, whose collision and ReachPosition conditions never
    # see them. So the second Autoware is real traffic rather than scenery.
    #
    # Usage: just two-av [scenario_file]
    #        EGO_MANAGED=false just two-av    # unmanaged ego (phase 013)
    #        KEEP_STACKS=1 just two-av        # leave both stacks up afterwards
    #        RECORD=1 just two-av             # screencast to play_log/two-av/two-av.mp4
    #
    # The steps below exist because doing this by hand goes wrong the same four ways every
    # time: a stale vehicle the next pilot latches onto, a bridge nobody restarted, a
    # scenario started before a stack is up, and stacks left running afterwards.
    managed="${EGO_MANAGED:-true}"
    if [ "$managed" = "true" ]; then ego_dom={{ego_domain}}; else ego_dom={{ego_unmanaged_domain}}; fi
    logs="{{project}}/play_log/two-av"
    mkdir -p "$logs"
    rec_display="${RECORD_DISPLAY:-:1}"
    launch_rviz=$([ "${RECORD:-0}" = "1" ] && echo true || echo false)
    echo "[two-av] ego domain $ego_dom ($([ "$managed" = true ] && echo managed || echo unmanaged)), background AV domain 2"

    # Kill the whole process group of each stack, not the launcher alone: play_launch's
    # children outlive it often enough that CLAUDE.md has a section about it.
    ego_pg=""; bg_pg=""
    cleanup() {
        if [ "${KEEP_STACKS:-0}" = "1" ]; then
            echo "[two-av] KEEP_STACKS=1, leaving both stacks up"
            return
        fi
        echo "[two-av] stopping stacks"
        for pg in $bg_pg $ego_pg; do kill -TERM -"$pg" 2>/dev/null || true; done
        sleep 15
        for pg in $bg_pg $ego_pg; do kill -KILL -"$pg" 2>/dev/null || true; done
    }
    trap cleanup EXIT INT TERM

    # A vehicle left behind by a killed run is the single most expensive thing to miss: the
    # next pilot localizes to it, routes it, engages it, and spends its whole budget driving
    # a car this run is not about. See acb 195467a.
    "{{project}}/scripts/clear_stale_vehicles.py" --port {{carla_port}} || {
        echo "[two-av] could not clear the world; refusing to start a run in it"
        exit 1
    }

    # The bridge is not part of any launch file and nothing restarts it between runs.
    if ! pgrep -x carla_scenario_ >/dev/null 2>&1; then
        echo "[two-av] no bridge running; starting one"
        setsid just run > "$logs/bridge.log" 2>&1 &
        sleep 20
    fi
    pgrep -x carla_scenario_ >/dev/null || { echo "[two-av] bridge failed to start; see $logs/bridge.log"; exit 1; }

    # One stack at a time, deliberately. Starting both together was tried and does not
    # work: each Autoware loads three TensorRT inference nodes (the traffic-light
    # classifiers and fine detector, plus lidar_centerpoint on the background AV) that take
    # ~45 s each to construct on a warm engine cache, and two stacks building them at once
    # pushed both past their load budget. Both sat at "composable 88/89 loaded (1 pending)"
    # until the wait expired, with the pending members every time being exactly those
    # inference nodes. Serial costs a few more minutes and finishes.
    wait_for_stack() {
        local name="$1" deadline=$((SECONDS + 1200))
        until grep -q "Startup complete" "$logs/$name.log" 2>/dev/null; do
            if [ $SECONDS -gt $deadline ]; then
                echo "[two-av] $name stack did not come up within 1200s; see $logs/$name.log"
                grep -oE "waiting on [0-9]+ member\(s\): [^.]*" "$logs/$name.log" | tail -1
                return 1
            fi
            sleep 10
        done
        echo "[two-av] $name stack up at $(date +%T)"
    }

    echo "[two-av] starting the ego stack (several minutes)"
    setsid env EGO_MANAGED="$managed" EGO_GOAL_POSES_FILE="{{project}}/scenarios/ego_poses.yaml" \
        LAUNCH_RVIZ="$launch_rviz" DISPLAY="$rec_display" \
        just ego-av > "$logs/ego.log" 2>&1 &
    ego_pg=$!
    wait_for_stack ego || exit 1

    echo "[two-av] starting the background AV stack"
    setsid just bg-av > "$logs/bg.log" 2>&1 &
    bg_pg=$!
    wait_for_stack bg || exit 1

    # Each stack has to be able to drive before a scenario is worth starting. The ego gate
    # also requires its pilot when unmanaged, because nothing else would route it.
    EGO_MANAGED="$managed" just _require-ego-stack || exit 1

    # RECORD=1 grabs the display the ego stack's RViz is drawing on. RViz has to come from
    # INSIDE that stack (LAUNCH_RVIZ=1 above): a standalone `rviz2 -d autoware.rviz` cannot
    # load the config at all, because the config's VehicleModel display needs vehicle
    # parameters the stack's launch provides -- it fails with "Statically typed parameter
    # 'wheel_radius' must be initialized" and falls back to a bare 450x250 window, which is
    # exactly what the first recorded attempt captured.
    #
    # RECORDING CHANGES THE RESULT ON THIS MACHINE, so do not record a run you intend to
    # judge. RViz renders through llvmpipe on the VNC display -- Mesa software rasterizing,
    # not the RTX 5090 sitting next to it, which CARLA has -- and costs ~4 cores on its own.
    # Measured: without recording, load ~5 and the scenario passed with both AVs reaching
    # their goals; with recording, load 59 on 32 cores, 134 "machine is not powerful enough
    # to run at 10 Hz" warnings from the interpreter, and the run died on the storyboard's
    # own 300 s timeout with the background AV still sitting at its spawn point.
    # VirtualGL is installed (`vglrun`, also `-d egl`) but did not engage for rviz2 here --
    # it stayed on llvmpipe and in neither case appeared in `nvidia-smi`. Getting RViz onto
    # the GPU is the fix and is not done.
    rec_pid=""
    stop_recording() {
        # SIGINT, not SIGKILL: mp4 writes its index when the muxer closes, and a killed
        # ffmpeg leaves a file no player will open.
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
    rm -f /tmp/scenario_test_runner/result.junit.xml
    EGO_MANAGED="$managed" just scenario "{{scenario_file}}" 2>&1 | tail -5 || true

    stop_recording
    if [ -f /tmp/scenario_test_runner/result.junit.xml ]; then
        if grep -q 'failures="0" errors="0"' /tmp/scenario_test_runner/result.junit.xml; then
            echo "[two-av] scenario PASSED"
        else
            echo "[two-av] scenario FAILED -- junit:"
            sed -n "1,6p" /tmp/scenario_test_runner/result.junit.xml
        fi
    else
        echo "[two-av] no junit written: the run did not reach a verdict"
    fi

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
# For an unmanaged ego (phase 013), bring the stack up that way first and pass EGO_MANAGED:
#   EGO_MANAGED=false EGO_GOAL_POSES_FILE=$PWD/scenarios/ego_poses.yaml just ego-av
#   EGO_MANAGED=false just acceptance $PWD/scenarios/town01_unmanaged.xosc
acceptance scenario=(project + "/scenarios/town01_ego_drive.xosc") runs="1" domain="":
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    source "{{acb_src}}/install/setup.bash"
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    args=(--scenario "{{scenario}}" --runs {{runs}})
    # Match the stack: an unmanaged ego lives in its own domain, and the scenario has to be
    # launched the same way or SSv2 routes an ego that is not listening.
    [ "${EGO_MANAGED:-true}" = "false" ] && args+=(--unmanaged)
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
    # Look where the ego actually is. An unmanaged ego lives in its own domain, so checking
    # SSv2's would find nothing and refuse a run that was correctly set up.
    # An unmanaged ego has no concealer to route or engage it, so the pilot is not
    # optional there and the gate checks for it too.
    require_pilot=""
    if [ "${EGO_MANAGED:-true}" = "true" ]; then
        export ROS_DOMAIN_ID={{ego_domain}}
    else
        export ROS_DOMAIN_ID={{ego_unmanaged_domain}}
        require_pilot="--require-pilot"
    fi
    if ! "{{project}}/scripts/ego_stack_health.py" $require_pilot; then
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

# Launch the ego's Autoware + acb_bridge in SSv2's ROS domain (no domain override here:
# SSv2's concealer needs plain ROS reachability). Long-lived: start it once, BEFORE any
# `just scenario`, and reuse it across scenario runs.
# Usage: just ego-av [map_path]
#   EGO_MANAGED=false EGO_GOAL_POSES_FILE=... just ego-av   -> unmanaged (phase 013):
#   own domain, own /clock, driven by acb_pilot instead of SSv2's concealer.
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
    # Domain follows `managed`. A managed ego must share SSv2's domain -- the concealer
    # talks plain ROS and a stack anywhere else is invisible to it. An unmanaged ego is
    # driven by its own pilot and has no reason to be there, so it gets a domain of its
    # own and SSv2's contains only SSv2 (phase 013). Not 0 either way; see the ego_domain
    # comment at the top of this file.
    # Env vars, not recipe parameters: just's parameters are positional, so
    # `just ego-av managed=false` assigns "managed=false" to map_path and silently runs a
    # managed stack against a nonexistent map. EGO_MANAGED cannot be passed by accident.
    managed="${EGO_MANAGED:-true}"
    goal_poses_file="${EGO_GOAL_POSES_FILE:-}"
    if [ "$managed" = "true" ]; then
        export ROS_DOMAIN_ID={{ego_domain}}
        clock=false      # SSv2 publishes /clock in its domain; a second publisher makes
                         # localization log backwards jumps.
    else
        export ROS_DOMAIN_ID={{ego_unmanaged_domain}}
        clock=true       # No SSv2 here, so without this the domain has no clock at all.
        if [ -z "$goal_poses_file" ]; then
            echo "[just] EGO_MANAGED=false needs EGO_GOAL_POSES_FILE: with the concealer" >&2
            echo "[just] inert, acb_pilot routes the ego and exits fatally on an empty file." >&2
            echo "[just] Try: EGO_MANAGED=false \\" >&2
            echo "[just]      EGO_GOAL_POSES_FILE=\$PWD/scenarios/ego_poses.yaml just ego-av" >&2
            exit 1
        fi
    fi
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
    # A ROS launch argument cannot carry an empty value -- `goal_poses_file:=` is rejected
    # as "malformed launch argument", and the failure comes minutes in, after the whole
    # parse. The launch file already declares a default for it, so pass the argument only
    # when there is something to pass. A managed run leaves it unset, which is why this
    # broke `just ego-av` outright rather than only the unmanaged path.
    optional_args=()
    if [ -n "$goal_poses_file" ]; then
        optional_args+=(goal_poses_file:="$goal_poses_file")
    fi
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
    # Not exec: the trap above has to survive to clean up the API adaptors.
    play_launch launch --parser python --web-addr 0.0.0.0:8082 \
        --load-node-timeout 120 \
        --load-total-budget 600 \
        --log-dir play_log/ego \
        csb_launch ego_av.launch.xml \
        map_path:="{{map_path}}" \
        carla_port:={{carla_port}} \
        managed:=$managed \
        publish_clock:=$clock \
        report_measured_steering:="${REPORT_MEASURED_STEERING:-false}" \
        launch_rviz:="${LAUNCH_RVIZ:-false}" \
        steering_multiplier:="${STEERING_MULTIPLIER:-1.0}" \
        publish_ground_truth_objects:="${GROUND_TRUTH_OBJECTS:-false}" \
        seed_localization_on_attach:="${SEED_LOCALIZATION:-true}" \
        ground_truth_range_m:="${GROUND_TRUTH_RANGE_M:-100.0}" \
        accel_map_path:="$accel_map" \
        brake_map_path:="$brake_map" \
        "${optional_args[@]}"

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
    export ROS_DOMAIN_ID={{domain}}
    # --web-addr keeps this stack's UI off the port the ego's Autoware already took.
    # There used to be a PLAY_LAUNCH_WEB_ADDR export here as well: it existed only so the
    # concealer's patched launch.hpp could pass an address when SSv2 forked Autoware
    # itself. With launch_autoware:=false the concealer launches nothing, so nothing reads
    # it -- the flag below is what actually sets the port. See phase 012, gap 10.
    exec play_launch launch --parser python --web-addr 0.0.0.0:{{web_port}} \
        --load-node-timeout 120 \
        --load-total-budget 180 \
        --log-dir play_log/bg-{{vehicle_name}} \
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
scenario scenario_file: _require-carla _require-ego-stack _clear-stale-scenario
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
    # Same toggle as `just ego-av` reads, and for the same reason it is an env var there.
    # An unmanaged ego lives in its own domain, so SSv2 no longer shares one with it.
    managed="${EGO_MANAGED:-true}"
    export ROS_DOMAIN_ID={{ego_domain}}
    # --parser python: scenario_test_runner.launch.py imports launch.actions the
    # Rust parser's embedded Python cannot resolve (EmitEvent)
    exec play_launch launch --parser python --web-addr 0.0.0.0:8081 \
        --log-dir play_log/scenario \
        csb_launch carla_scenario.launch.xml \
        managed_ego:=$managed \
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
