# scenario-world-bridge -- SSv2 on CARLA and Isaac Sim, through one world interface (roadmap 046)
set dotenv-load

# Host settings come from the environment (or .env / .envrc): ports, displays, install and
# data paths, the log root. Project settings -- map, vehicle, rviz, scenario output, ... --
# are recipe arguments, name:=value as for a launch file; the environment variables that
# used to carry them are refused by name (scripts/just_lib.bash, SWB_RETIRED_ENV).
carla_port := env_var_or_default('CARLA_PORT', '2000')
ssv2_port := env_var_or_default('SSV2_PORT', '5555')
# The town a recipe uses when no map is given
default_map := 'Town01'
# ROS domain for the ego stack, which is also SSv2's: the concealer reaches the ego over
# plain ROS, so `just ego-av` and `just scenario` must agree on this. Domain 0 is left
# free deliberately -- it is where every unconfigured ROS process on the host lands, and
# a stray node there joins the scenario's graph without anyone asking. Background AVs
# (scenario entities with controller `agent`) get BG_ROS_DOMAIN_ID (2) and up (`just bg-av`).
ego_domain := env_var_or_default('EGO_ROS_DOMAIN_ID', '1')
# Where SSv2 and the agent relay run (roadmap 017). Separate from every vehicle's domain:
# the scenario reaches vehicles only through the relay's TCP port (`agent_port`).
scenario_domain := env_var_or_default('SCENARIO_ROS_DOMAIN_ID', '9')
agent_port := env_var_or_default('AGENT_PORT', '5560')
# The first background AV's ROS domain (`just bg-av`, `just two-av`); a second one is passed
# as `just bg-av <entity> <domain>`. A host setting, so two checkouts keep their AVs apart.
bg_domain := env_var_or_default('BG_ROS_DOMAIN_ID', '2')
# The play_launch web UIs, one port per stack from SWB_WEB_PORT up (a host setting, so two
# checkouts can run side by side): +0 the demo, +1 the scenario stack, +2 the ego stack,
# +3 the background AV (bg_av_1), +4 the simulation side (bridge + relay), +5 the Isaac world
# server; a second background AV takes +6 (`just bg-av bg_av_2 3 <port>`). Recipes compute
# them as $((SWB_WEB_PORT + n)); scripts/isaac_up.sh and run_suite read the same variable.
export SWB_WEB_PORT := env_var_or_default('SWB_WEB_PORT', '8080')
# The world servers' REP ports on 127.0.0.1 (roadmap 046): Isaac's (`just isaac-world`) and
# CARLA's (inside `just run sim:=carla`). The bridge drives the one its simulation side names.
isaac_port := env_var_or_default('ISAAC_PORT', '5590')
world_port := env_var_or_default('WORLD_PORT', '5591')
data_dir := env_var_or_default('DATA_DIR', justfile_directory() + '/data')
# The converted CARLA maps, one directory per town; the repo's scenarios name theirs as
# $(env CARLA_MAPS)/<Town>.
maps_dir := env_var_or_default('CARLA_MAPS', data_dir + '/carla-autoware-bridge')
project := justfile_directory()
# Where every stack's play_launch logs go, <log root>/<kind>/<time>/. A host setting: keep it
# on a disk whose fsync latency cannot stall a node (CLAUDE.md, "The log disk can stall a
# node"). When it is not the repo's play_log, play_log/<kind> links to it.
export SWB_LOG_ROOT := env_var_or_default('SWB_LOG_ROOT', justfile_directory() + '/play_log')
lib := justfile_directory() + '/scripts/just_lib.bash'
acb_src := justfile_directory() + '/src/autoware_carla_bridge'
# Where Autoware's own setup.bash lives. The NEWSLabNTU 1.5.0 Debian installs under
# /opt/autoware/1.5.0; other hosts install the same packages straight into the ROS
# prefix (/opt/ros/humble). Override with AUTOWARE_SETUP when yours is elsewhere.
autoware_setup := env_var_or_default('AUTOWARE_SETUP', '/opt/autoware/1.5.0/setup.bash')
# Autoware packages carried with fixes 1.5.0 lacks (roadmap 042), built by
# `just build-autoware-overlay` from the commit pinned in scripts/autoware_overlay.bash.
autoware_overlay_dir := justfile_directory() + '/third_party/autoware_overlay'
# The TensorRT runtime Autoware 1.5.0 was compiled against (10.8.0), put first on the
# Autoware stacks' LD_LIBRARY_PATH so their cached engines validate (scripts/
# autoware_tensorrt.bash). A host setting: AUTOWARE_TENSORRT_LIB, empty for the host's own.
autoware_tensorrt_lib := env_var_or_default('AUTOWARE_TENSORRT_LIB', justfile_directory() + '/third_party/tensorrt/lib')

# List available recipes
default:
    @just --list

# The user install of docs/user-guide.md section 1 (rustup, colcon-cargo-ros2,
# submodules, `rosdep install`), plus developer extras (nightly rustfmt for
# `just check`/`format`, cargo-nextest for `just test`). Safe to re-run.
#
# One-time setup: rustup, colcon plugin, submodules, rosdep, dev tools
setup:
    #!/usr/bin/env bash
    set -e
    cd "{{project}}"

    # Rust: the same rustup install the user guide gives
    if ! command -v rustup &>/dev/null; then
        echo "Installing Rust via rustup..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        source "$HOME/.cargo/env"
    fi

    # colcon's Rust plugin. The older colcon-cargo/colcon-ros-cargo pair conflicts with it.
    if python3 -c "import colcon_cargo" &>/dev/null || python3 -c "import colcon_ros_cargo" &>/dev/null; then
        echo "Removing conflicting colcon-cargo/colcon-ros-cargo..."
        pip uninstall -y colcon-cargo colcon-ros-cargo
    fi
    python3 -c "import colcon_cargo_ros2" &>/dev/null || pip install colcon-cargo-ros2

    # Initialise missing submodules only: a plain `update` would move a submodule you are
    # working in back to its pinned commit.
    if git submodule status --recursive | grep -q '^-'; then
        git submodule update --init --recursive
    fi

    # System dependencies, exactly as a user installs them
    source /opt/ros/humble/setup.bash
    source "{{autoware_setup}}"
    rosdep install --from-paths src --ignore-src -y

    # Developer extras
    rustup toolchain list | grep -q nightly || rustup toolchain install nightly
    command -v cargo-nextest &>/dev/null || cargo install cargo-nextest --locked

    # pip may have pulled a newer setuptools into ~/.local, which shadows the apt one and
    # breaks `colcon build --symlink-install`.
    just _check-setuptools || true
    echo "Setup done. Next: just build"

alias install-deps := setup

# Fail fast if a pip setuptools shadows the system one.
#
# colcon's --symlink-install runs `setup.py develop --editable`, which setuptools removed
# in v80. When a pip-installed setuptools in ~/.local takes precedence over the apt one,
# every Python package in the workspace dies with "option --editable not recognized" --
# a message that says nothing about setuptools. Check up front instead.
_check-setuptools:
    #!/usr/bin/env bash
    set -e
    # A Python virtualenv first on PATH (a project's .venv) hides the system python3 that
    # ROS and colcon need; drop it for this recipe.
    PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '/\.venv/bin$' | paste -sd:)
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

# The user guide's `colcon build`, plus developer flags: --symlink-install (edit Python
# and launch files without rebuilding), the dev-release cargo profile, no tests. The CARLA
# version comes from the `carla-0916` feature in Cargo.toml, as for users; an exported
# CARLA_VERSION overrides it.
#
# Build all packages (developer flags on the user's colcon build)
build: _check-setuptools
    #!/usr/bin/env bash
    set -e
    PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '/\.venv/bin$' | paste -sd:)
    cd "{{project}}"
    source /opt/ros/humble/setup.bash
    source "{{autoware_setup}}"
    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
    colcon build \
        --base-paths src \
        --symlink-install \
        --cargo-args --profile dev-release \
        --cmake-args -DBUILD_TESTING=OFF

    # colcon-cargo-ros2 writes this workspace's message patches into acb's Cargo.lock too,
    # as `[[patch.unused]]` entries (swb_interfaces is not an acb dependency). Undo that, and
    # only that, so the acb submodule does not show a modified lock after every build here.
    lock=src/autoware_carla_bridge/Cargo.lock
    if ! git -C src/autoware_carla_bridge diff --quiet -- Cargo.lock; then
        if git -C src/autoware_carla_bridge diff -U0 -- Cargo.lock | grep -E '^[+-][^+-]' \
            | grep -vqE '^[+-](\[\[patch\.unused\]\]|name = "[a-z0-9_]+"|version = "0\.0\.0")$'; then
            echo "note: $lock changed beyond [[patch.unused]]; left as is" >&2
        else
            git -C src/autoware_carla_bridge checkout -- Cargo.lock
        fi
    fi

# Fetch the Autoware fixes 1.5.0 lacks (the fork commit pinned in
# scripts/autoware_overlay.bash, only the packages listed there) and build them into
# third_party/autoware_overlay against Autoware alone (roadmap 042). `just ego-av`, `just
# bg-av` and the scenario precheck source the result after Autoware (AUTOWARE_OVERLAY=0: off).
# Stop the ego stack first if it runs: it has the old libraries loaded.
#
# Build the Autoware fix overlay (blind_spot, autoware_universe#13509)
build-autoware-overlay:
    #!/usr/bin/env bash
    set -e
    PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '/\.venv/bin$' | paste -sd:)
    source "{{project}}/scripts/autoware_overlay.bash"
    swb_autoware_overlay_build "{{autoware_overlay_dir}}" "{{autoware_setup}}"

# Download TensorRT 10.8.0 (NVIDIA's tensorrt-cu12-libs wheel, 3.1 GB; 2.6 GB extracted) into
# AUTOWARE_TENSORRT_LIB (default third_party/tensorrt/lib), the runtime `just ego-av`,
# `ego-isaac` and `bg-av` put first on LD_LIBRARY_PATH. Autoware 1.5.0 was compiled against
# 10.8.0 and rejects an engine any other TensorRT built, so without it CenterPoint rebuilds
# its engines at every start. Idempotent.
#
# Fetch the TensorRT runtime Autoware was built against
setup-tensorrt:
    #!/usr/bin/env bash
    set -e
    source "{{project}}/scripts/autoware_tensorrt.bash"
    swb_tensorrt_setup "{{autoware_tensorrt_lib}}"

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
    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
    cargo +nightly fmt --check
    cargo clippy --all-targets -- -D warnings

# Run tests
test:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    # CARLA_DIR (a host setting, .env) is CARLA's install; carla-sys would take it for a
    # CARLA source tree to build from, and the build fails.
    unset CARLA_DIR
    cargo nextest run --no-tests pass --no-fail-fast
    # run_suite's discovery and JUnit aggregation (stdlib only, no ROS graph needed)
    python3 -m pytest -q src/swb_launch/test
    # The agent SDK and protocol, and the relay against it (roadmap 030)
    python3 -m pytest -q src/swb_agent/test src/scenario_agent_relay/test
    # The world server shell, on a fake world (roadmaps 037, 046; protobuf only), and the Isaac
    # world's pure modules
    python3 -m unittest discover -s src/swb_world_server/test
    python3 -m unittest discover -s src/isaac_world/test
    # Every vehicle named by a CARLA blueprint declares that car's measured box and axles
    # (roadmap 050), and the module the generators take them from
    python3 -m pytest -q scripts/test_scenario_vehicle.py
    python3 scripts/check_scenario_vehicles.py

# Run CI checks: build, check (format + clippy), and tests
ci: build check test

# Two Autoware stacks, one scenario, end to end. record:=true for a screencast.
[positional-arguments]
two-av *args: _require-carla
    #!/usr/bin/env bash
    set -u
    source "{{lib}}"; swb_reject_env
    # What "two Autoware" means here: the ego's stack and one background AV's, each a full
    # Autoware in its OWN ROS domain with its own acb_bridge, /clock and vehicle agent. Both
    # are scenario entities (roadmap 017): the ego by isEgo, bg_av_1 by its controller
    # `agent`. swb spawns bg_av_1 with role_name bg_av_1 and drives it through the agent
    # relay -- teleported, then the scenario's goal -- so SSv2 sees it (collisions, reach
    # conditions) and the ego's Autoware perceives real, Autoware-driven traffic.
    #
    # Usage: just two-av [scenario_file] [keep_stacks:=true] [record:=true]
    #            [ego_rviz_config:=<file>] [bg_rviz_config:=<file>] [output_directory:=<dir>]
    #   keep_stacks:=true  leave the stacks this run started up afterwards
    #   record:=true       screencast to play_log/two-av/two-av.mp4, both stacks' RViz on
    #                      $RECORD_DISPLAY (a host setting; default $DISPLAY);
    #                      ego_rviz_config / bg_rviz_config choose their configs (layouts)
    #   output_directory   where the scenario writes its verdict (default /tmp)
    #
    # Stacks already running are reused and left running: the simulation side (`just run`),
    # the ego stack (`just ego-av`, web SWB_WEB_PORT+2) and the background AV stack (`just
    # bg-av`, web SWB_WEB_PORT+3). Only stacks this recipe starts are stopped at the end.
    scenario_file="{{project}}/src/swb_examples/scenarios/multi_av/town01_two_av.xosc"
    keep_stacks=false; record=false; ego_rviz=(); bg_rviz=(); output_directory=/tmp; n=0
    for a in "$@"; do
        case "$a" in
            keep_stacks:=*) keep_stacks="${a#*:=}" ;;
            record:=*) record="${a#*:=}" ;;
            ego_rviz_config:=?*) ego_rviz=("rviz_config:=${a#*:=}") ;;
            bg_rviz_config:=?*) bg_rviz=("rviz_config:=${a#*:=}") ;;
            output_directory:=?*) output_directory="${a#*:=}" ;;
            *:=*) echo "two-av: unknown argument '$a' (keep_stacks:=, record:=, ego_rviz_config:=, bg_rviz_config:=, output_directory:=)" >&2; exit 2 ;;
            *) n=$((n + 1)); [ $n = 1 ] || { echo "two-av: too many arguments" >&2; exit 2; }
               scenario_file="$a" ;;
        esac
    done
    ego_dom={{ego_domain}}
    logs=$(swb_log_dir "{{project}}" two-av)
    rec_display="${RECORD_DISPLAY:-${DISPLAY:-:1}}"
    launch_rviz=$([ "$record" = true ] && echo true || echo false)
    echo "[two-av] scenario domain {{scenario_domain}}, ego domain $ego_dom, background AV domain {{bg_domain}}"

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
        if [ "$keep_stacks" = true ]; then
            echo "[two-av] keep_stacks:=true, leaving the stacks this run started up"
            return
        fi
        echo "[two-av] stopping the stacks this run started"
        for pg in $bg_pg $ego_pg; do kill -TERM -"$pg" 2>/dev/null || true; done
        sleep 15
        for pg in $bg_pg $ego_pg; do kill -KILL -"$pg" 2>/dev/null || true; done
    }
    trap cleanup EXIT INT TERM

    # The simulation side (bridge + relay) is long-lived and not part of any scenario launch.
    # This checkout's bridge, by its SSv2 port: another checkout's on the host is not ours.
    if ! ss -lnt | grep -q ":{{ssv2_port}} "; then
        echo "[two-av] no bridge running; starting the simulation side (just run)"
        setsid just run > "$logs/bridge.log" 2>&1 &
        for _ in $(seq 1 120); do
            grep -q "ZMQ server ready" "$logs/bridge.log" 2>/dev/null && break
            sleep 5
        done
    fi
    pgrep -x scenario_world_ >/dev/null || { echo "[two-av] bridge failed to start; see $logs/bridge.log"; exit 1; }

    # One stack at a time, deliberately: two Autoware stacks building their TensorRT nodes at
    # once pushed both past their load budget ("composable 88/89 loaded (1 pending)").
    wait_for_stack() {
        local name="$1" pg="$2" deadline=$((SECONDS + 1200))
        until grep -q "Startup complete" "$logs/$name.log" 2>/dev/null; do
            # The pid, not the group: right after `setsid ... &` the child may not have
            # called setsid() yet, and `kill -0 -$pg` then fails on a healthy stack. The pid
            # is `just` (setsid and env exec into it), alive exactly as long as the stack.
            if ! kill -0 "$pg" 2>/dev/null; then
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

    if stack_on_port $((SWB_WEB_PORT + 2)) >/dev/null; then
        echo "[two-av] reusing the running ego stack (web $((SWB_WEB_PORT + 2)))"
    else
        echo "[two-av] starting the ego stack (several minutes)"
        setsid env DISPLAY="$rec_display" \
            just ego-av rviz:="$launch_rviz" "${ego_rviz[@]}" > "$logs/ego.log" 2>&1 &
        ego_pg=$!
        wait_for_stack ego "$ego_pg" || exit 1
    fi

    if stack_on_port $((SWB_WEB_PORT + 3)) >/dev/null; then
        echo "[two-av] reusing the running background AV stack (web $((SWB_WEB_PORT + 3)))"
    else
        echo "[two-av] starting the background AV stack"
        # With record:=true the background AV gets an RViz too, on the same display: two
        # Autoware windows, one per vehicle (bg_rviz_config / ego_rviz_config place them).
        setsid env DISPLAY="$rec_display" \
            just bg-av launch_rviz:="$launch_rviz" "${bg_rviz[@]}" > "$logs/bg.log" 2>&1 &
        bg_pg=$!
        wait_for_stack bg "$bg_pg" || exit 1
    fi

    # Each stack has to be able to drive before a scenario is worth starting, and the
    # background AV's agent has to be registered: its goal fails the scenario otherwise.
    just _require-ego-stack || exit 1
    just _require-agent bg_av_1 || exit 1

    # record:=true grabs the display both stacks' RViz are drawing on (rviz above; a
    # standalone rviz2 cannot load autoware.rviz without the stack's vehicle parameters).
    # RECORDING CHANGES THE RESULT ON THIS MACHINE (RViz on llvmpipe costs ~4 cores): do
    # not record a run you intend to judge.
    rec_pid=""
    stop_recording() {
        # SIGINT, not SIGKILL: mp4 writes its index when the muxer closes.
        [ -n "$rec_pid" ] && kill -INT -"$rec_pid" 2>/dev/null && sleep 5
        [ -n "$rec_pid" ] && echo "[two-av] screencast: $logs/two-av.mp4"
    }
    if [ "$record" = true ]; then
        geom=$(DISPLAY="$rec_display" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')
        if [ -z "$geom" ]; then
            echo "[two-av] record:=true but display $rec_display is not usable; skipping the screencast"
        else
            echo "[two-av] recording $rec_display ($geom) to $logs/two-av.mp4"
            setsid ffmpeg -y -f x11grab -video_size "$geom" -framerate 8 -i "$rec_display.0" \
                -vf scale=1280:-2 -c:v libx264 -preset veryfast -crf 30 -pix_fmt yuv420p \
                "$logs/two-av.mp4" > "$logs/ffmpeg.log" 2>&1 &
            rec_pid=$!
        fi
    fi

    echo "[two-av] running $(basename "$scenario_file")"
    junit="$output_directory/scenario_test_runner/result.junit.xml"
    rm -f "$junit"
    just scenario "$scenario_file" output_directory:="$output_directory" 2>&1 | tail -5 || true

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
# One continuous drive of Town01 through six incidents, ambient traffic and walkers (roadmap
# 033), in tutorial 4's layout -- CARLA's chase camera top left, the ego's RViz on the right
# half, a caption strip (demo_captions) under the chase view.
# The demo tour, in tutorial 4's layout: [record:=true] [rviz:=] [keep_stacks:=] [scenario:=]
[positional-arguments]
demo-tour *args: _require-carla
    #!/usr/bin/env bash
    # Usage: just demo-tour [record:=true] [rviz:=true|false] [keep_stacks:=true|false]
    #            [scenario:=<xosc>] [output_directory:=<dir>]
    #   record:=true       screencast of the display to play_log/demo/<time>/demo-tour.mp4
    #   rviz:=false        no RViz (default true when a display is set)
    #   keep_stacks:=false stop the stacks this run started afterwards (default: keep them;
    #                      the ego stack takes minutes to come up)
    # The windows go to RECORD_DISPLAY, else DISPLAY (host settings); none set: no windows,
    # the tour runs headless. The ego stack is the tour's (max_velocity:=11.11, CenterPoint
    # with the lidar merger); a running one (web SWB_WEB_PORT+2) is reused, so start it with those
    # arguments. The simulation side is reused or started, and set to ClearNoon either way.
    set -e
    source "{{lib}}"; swb_reject_env
    record=false; rviz=""; keep_stacks=true; output_directory=""
    scenario="{{project}}/src/swb_examples/scenarios/demo/town01_tour.xosc"
    for a in "$@"; do
        case "$a" in
            record:=*) record="${a#*:=}" ;;
            rviz:=*) rviz="${a#*:=}" ;;
            keep_stacks:=*) keep_stacks="${a#*:=}" ;;
            scenario:=?*) scenario="${a#*:=}" ;;
            output_directory:=?*) output_directory="${a#*:=}" ;;
            *) echo "demo-tour: unknown argument '$a' (record:=, rviz:=, keep_stacks:=, scenario:=, output_directory:=)" >&2; exit 2 ;;
        esac
    done
    logs="$(swb_log_dir "{{project}}" demo-tour)/$(date +%Y-%m-%d_%H-%M-%S)"
    mkdir -p "$logs"; ln -sfn "$logs" "$(dirname "$logs")/latest"
    output_directory="${output_directory:-$logs/out}"
    display="${RECORD_DISPLAY:-${DISPLAY:-}}"
    if [ -n "$display" ] && ! DISPLAY="$display" xdpyinfo > /dev/null 2>&1; then
        echo "demo-tour: display $display is not usable; running headless" >&2; display=""
    fi
    [ -z "$rviz" ] && rviz=$([ -n "$display" ] && echo true || echo false)
    [ -z "$display" ] && rviz=false
    if [ "$record" = true ] && [ -z "$display" ]; then
        echo "demo-tour: record:=true needs RECORD_DISPLAY or DISPLAY" >&2; exit 2
    fi
    source "{{autoware_setup}}"
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

    stack_on_port() {
        for p in $(pgrep -x play_launch); do
            tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- "--web-addr [0-9.]*:$1 " \
                && { echo "$p"; return 0; }
        done
        return 1
    }
    # What this run started, killed by process group (setsid) at the end.
    started=(); windows=(); rec_pid=""
    cleanup() {
        if [ -n "$rec_pid" ]; then
            # SIGINT, not SIGKILL: mp4 writes its index when the muxer closes.
            kill -INT -"$rec_pid" 2>/dev/null || true; sleep 5
            echo "[demo-tour] screencast: $logs/demo-tour.mp4"
        fi
        for pg in "${windows[@]}"; do kill -TERM -"$pg" 2>/dev/null || true; done
        if [ "$keep_stacks" != true ] && [ ${#started[@]} -gt 0 ]; then
            echo "[demo-tour] stopping the stacks this run started"
            for pg in "${started[@]}"; do kill -TERM -"$pg" 2>/dev/null || true; done
            sleep 15
            for pg in "${started[@]}"; do kill -KILL -"$pg" 2>/dev/null || true; done
        fi
    }
    trap cleanup EXIT INT TERM

    # The simulation side, ClearNoon.
    if ! stack_on_port $((SWB_WEB_PORT + 4)) > /dev/null; then
        echo "[demo-tour] starting the simulation side (just run weather:=ClearNoon)"
        setsid just run weather:=ClearNoon > "$logs/bridge.log" 2>&1 < /dev/null &
        started+=($!)
        for _ in $(seq 1 120); do
            ss -lnt 2>/dev/null | grep -q ":{{ssv2_port}} " && break
            sleep 2
        done
    fi
    ROS_DOMAIN_ID={{scenario_domain}} timeout 30 ros2 service call /scenario_world_bridge/set_weather \
        swb_interfaces/srv/SetWeather "{preset: ClearNoon}" 2>/dev/null | grep -o "success=[A-Za-z]*" \
        || echo "[demo-tour] note: could not set ClearNoon (bridge not answering yet?)"

    # The ego stack, the tour's arguments, RViz on the display.
    if pid=$(stack_on_port $((SWB_WEB_PORT + 2))); then
        args=$(tr '\0' ' ' < "/proc/$pid/cmdline")
        case "$args" in *max_velocity:=11.11*) ;; *)
            echo "[demo-tour] warning: the running ego stack (web $((SWB_WEB_PORT + 2))) was not started with" \
                 "max_velocity:=11.11 -- the tour is timed for 40 km/h" >&2 ;;
        esac
        echo "[demo-tour] reusing the running ego stack (web $((SWB_WEB_PORT + 2)))"
    else
        echo "[demo-tour] starting the ego stack (several minutes; CenterPoint)"
        rviz_args=(rviz:="$rviz")
        [ "$rviz" = true ] && rviz_args+=(rviz_config:="{{project}}/src/swb_launch/rviz/autoware_cast.rviz")
        setsid env ${display:+DISPLAY=$display} just ego-av max_velocity:=11.11 \
            lidar_detection_model:=centerpoint use_multi_channel_tracker_merger:=false \
            "${rviz_args[@]}" > "$logs/ego.log" 2>&1 < /dev/null &
        started+=($!)
        deadline=$((SECONDS + 1500))
        until grep -q "Startup complete" "$logs/ego.log" 2>/dev/null; do
            kill -0 "${started[-1]}" 2>/dev/null || { echo "[demo-tour] ego stack exited; see $logs/ego.log"; exit 1; }
            [ $SECONDS -gt $deadline ] && { echo "[demo-tour] ego stack not up in 1500 s; see $logs/ego.log"; exit 1; }
            sleep 10
        done
    fi
    just _require-ego-stack

    # The windows: chase camera top left, captions under it (tutorial 4's layout; RViz's
    # autoware_cast.rviz takes the right half of a 1920x1080 screen).
    if [ -n "$display" ]; then
        setsid env DISPLAY="$display" SDL_VIDEO_WINDOW_POS=70,28 VIEW_W=740 VIEW_H=416 \
            CARLA_PORT={{carla_port}} ros2 run swb_launch chase_view > "$logs/chase_view.log" 2>&1 < /dev/null &
        windows+=($!)
        setsid env DISPLAY="$display" ROS_DOMAIN_ID={{scenario_domain}} ros2 run swb_launch demo_captions \
            --geometry 740x190+70+478 --log "$logs/captions.jsonl" > "$logs/captions.log" 2>&1 < /dev/null &
        windows+=($!)
        sleep 5
    else
        setsid env ROS_DOMAIN_ID={{scenario_domain}} ros2 run swb_launch demo_captions --headless \
            --log "$logs/captions.jsonl" > "$logs/captions.log" 2>&1 < /dev/null &
        windows+=($!)
    fi
    if [ "$record" = true ]; then
        geom=$(DISPLAY="$display" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')
        echo "[demo-tour] recording $display ($geom) to $logs/demo-tour.mp4"
        setsid ffmpeg -y -f x11grab -video_size "$geom" -framerate 15 -i "$display.0" \
            -vf scale=1280:-2 -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
            "$logs/demo-tour.mp4" > "$logs/ffmpeg.log" 2>&1 < /dev/null &
        rec_pid=$!
    fi

    echo "[demo-tour] running $(basename "$scenario")"
    junit="$output_directory/scenario_test_runner/result.junit.xml"
    rm -f "$junit"
    # The runner's cap is wall time and the tour runs ~5 min of simulation at a real-time
    # factor of ~0.8 (SSv2's own frame, roadmap 033 step 8): 400 s is too tight, and a loaded
    # host stretches it further. The tour's own stop trigger is 600 s of simulation time.
    just scenario "$scenario" output_directory:="$output_directory" global_timeout:=1200 2>&1 | tail -3 || true
    if [ -f "$junit" ] && grep -q 'failures="0" errors="0"' "$junit"; then
        echo "[demo-tour] tour PASSED"
    else
        echo "[demo-tour] tour did not pass: ${junit}"
    fi
    echo "[demo-tour] captions: $logs/captions.jsonl"

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

# The simulation side (roadmaps 017, 046), long-lived, in the scenario domain. sim:=carla
# (default): swb_launch carla_simulation.launch.xml -- the CARLA world server, the bridge
# driving it and the agent relay, one launch; CARLA itself runs apart (`just carla-start`).
# sim:=isaac: isaac_simulation.launch.xml -- the bridge and the relay, driving the Isaac
# world server (`just isaac-world`). play_launch restarts a server or bridge that died.
# Other arguments are the launch file's, name:=value (config_file:=, weather:=,
# gap_trace_dir:=, npc_contact_sensors:= on CARLA, world_render:= on Isaac). Ports and
# CARLA_HOST are the host's. `just build` after changes; restart this after a build.
#
# Usage: just run [sim:=carla|isaac] [name:=value ...]
[positional-arguments]
run *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    sim=carla; args=()
    for a in "$@"; do
        case "$a" in
            sim:=*) sim="${a#sim:=}" ;;
            backend:=*) echo "run: backend:= is now sim:= (roadmap 046)" >&2; exit 2 ;;
            *) args+=("$a") ;;
        esac
    done
    case "$sim" in
        carla) launch=(carla_simulation.launch.xml carla_host:="${CARLA_HOST:-localhost}"
                       carla_port:={{carla_port}} world_port:={{world_port}}) ;;
        isaac) launch=(isaac_simulation.launch.xml isaac_port:={{isaac_port}}) ;;
        *) echo "run: sim '$sim' is not carla or isaac" >&2; exit 2 ;;
    esac
    log_dir=$(swb_log_dir "{{project}}" bridge)
    source "{{autoware_setup}}"
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # On CycloneDDS like every other process of the stack; an unset RMW is Fast DDS, whose
    # relay SSv2 cannot see.
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    export ROS_DOMAIN_ID={{scenario_domain}}
    exec play_launch launch --web-addr 0.0.0.0:$((SWB_WEB_PORT + 4)) \
        --log-dir "$log_dir" \
        swb_launch "${launch[@]}" agent_port:={{agent_port}} ssv2_port:={{ssv2_port}} \
        "${args[@]}"


# quality:=low saves memory but must not change towns: Low segfaults on load_world to another
# town, so a scenario on another map, town02_episode_change or a suite that mixes towns needs
# Epic (acb's scripts/carla_args.bash). CARLA_DIR, DISPLAY and CARLA_PORT are the host's.
# Start CARLA as a background service: [map:=Town01] [quality:=epic|low] [render:=offscreen|window] [extra_args:=]
[positional-arguments]
carla-start *args:
    "{{acb_src}}/scripts/carla_start.sh" {{carla_port}} "$@"

# Isaac Sim (roadmap 036): install root, and the VNC display Isaac opens its window on
isaac_root := env_var_or_default("ISAAC_ROOT", "/mnt/disk1/isaac-sim")
isaac_display := env_var_or_default("ISAAC_DISPLAY", ":8")

# Isaac Sim smoke test: version, start time, step time (headless; DISPLAY set = windowed)
isaac-check:
    ISAAC_ROOT={{isaac_root}} "{{justfile_directory()}}/src/isaac_world/spikes/run_spike.sh" fastrtps {{isaac_root}}/logs/isaac-check.log "{{justfile_directory()}}/src/isaac_world/spikes/smoke.py"; grep SMOKE {{isaac_root}}/logs/isaac-check.log

# The Isaac Sim world server (roadmaps 037, 038, 046), supervised (043):
# swb_launch isaac_world.launch.xml under play_launch (web SWB_WEB_PORT+5, logs in $SWB_LOG_ROOT/isaac/),
# which respawns it when it dies. Long-lived: start it once, before `just run sim:=isaac`
# and `just ego-isaac`; `just scenario ... sim:=isaac` starts it when it is not up.
#
# Where it runs is environment, set per shell or kept in .env / .envrc:
#   ISAAC_PORT=5590   REP port on 127.0.0.1 (`just run` hands swb the same port)
#   EGO_ROS_DOMAIN_ID the ego's ROS domain (1; the same as `just ego-isaac`), over ego.yaml's
#   DISPLAY=:8        Isaac opens its window there (the VNC display); unset: headless.
#                     ISAAC_HEADLESS=1 forces headless with DISPLAY set.
# What it simulates is arguments, name:=value as for a launch file:
#   ego:=<yaml>       an aib ego config (default src/autoware_isaac_bridge/src/aib/config/ego.yaml:
#                     aib's Tesla Model 3, a PhysX vehicle with Autoware's sensors and vehicle
#                     interface on its domain; roadmap 045); ego:=none (or empty) keeps 037's
#                     kinematic box
#   follow:=true      with a window: no UI, the full-window viewport chases the ego
#   render:=auto      window | headless; auto: a window when DISPLAY is set (see above)
#   world:=<w>[,<w>]  worlds to load and render before serving: world files
#                     (<map>/isaac/world.yaml), map directories or USD files. A world's first
#                     render compiles its materials (CARLA Town01: ~9 min on a new host), which
#                     no request timeout covers; swb's LoadWorld of a preloaded one is quick.
#                     The first listed stays loaded.
# DDS as the ego stack's: Cyclone with the localhost config (036: both sides must match).
# Ready when it answers Hello: `just isaac-health`. Stop it by its play_launch's pid
# (SIGTERM); swb only empties its world.
#
# Usage: [DISPLAY=:8] just isaac-world [ego:=<yaml>] [follow:=true] [world:=<w>,...] [render:=...]
[positional-arguments]
isaac-world *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"
    args=()
    for a in "$@"; do
        case "$a" in
            ego:=) args+=("ego:=none") ;;
            world:=) ;;
            ego:=*|world:=*|follow:=*|render:=*) args+=("$a") ;;
            *) echo "isaac-world: unknown argument '$a' (ego:=, world:=, follow:=, render:=; the port and display are ISAAC_PORT and DISPLAY)" >&2; exit 2 ;;
        esac
    done
    source /opt/ros/humble/setup.bash
    source "{{project}}/install/setup.bash"
    # The ego's ROS domain is the ego stack's (`just ego-isaac`), not aib's ego.yaml's.
    export ISAAC_ROOT={{isaac_root}} ISAAC_PORT={{isaac_port}} EGO_ROS_DOMAIN_ID={{ego_domain}}
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # Built beside this checkout's isaac_world; a moved checkout points it here again.
    export SWB_ISAAC_DIR="{{project}}/src/isaac_world"
    log_dir=$(swb_log_dir "{{project}}" isaac)
    exec play_launch launch --web-addr 0.0.0.0:$((SWB_WEB_PORT + 5)) --log-dir "$log_dir" \
        swb_launch isaac_world.launch.xml "${args[@]}"

# Is a world server up? One probe Hello (it changes nothing on the server): its simulator,
# capabilities, owner and readiness. sim:=carla (default, WORLD_PORT) or sim:=isaac
# (ISAAC_PORT); wait:=SECS retries that long. `just world-check` runs the conformance check.
#
# Usage: just world-health [sim:=carla|isaac] [wait:=SECS]
[positional-arguments]
world-health *args:
    #!/usr/bin/env bash
    sim=carla; wait=0
    for a in "$@"; do
        case "$a" in
            sim:=*) sim="${a#sim:=}" ;;
            wait:=*) wait="${a#wait:=}" ;;
            *) echo "world-health: unknown argument '$a' (sim:=, wait:=SECS)" >&2; exit 2 ;;
        esac
    done
    case "$sim" in carla) port={{world_port}} ;; isaac) port={{isaac_port}} ;;
        *) echo "world-health: sim '$sim' is not carla or isaac" >&2; exit 2 ;; esac
    exec "{{project}}/install/scenario_world_bridge/lib/scenario_world_bridge/swb_world_check" \
        hello "tcp://127.0.0.1:$port" --wait "$wait"

# The Isaac world server's health (`just world-health sim:=isaac`).
[positional-arguments]
isaac-health *args:
    @just world-health sim:=isaac "$@"

# Check a world server against the world protocol (roadmap 046): ownership, the core, each
# capability it declares. It takes the server over and moves bodies: never during a scenario
# (a server a bridge owns refuses it, which is part of the check). map:=<map dir> is what it
# loads (default the Town01 map).
#
# Usage: just world-check [sim:=carla|isaac] [map:=<dir>]
[positional-arguments]
world-check *args:
    #!/usr/bin/env bash
    sim=carla; map="{{maps_dir}}/{{default_map}}"
    for a in "$@"; do
        case "$a" in
            sim:=*) sim="${a#sim:=}" ;;
            map:=*) map="${a#map:=}" ;;
            *) echo "world-check: unknown argument '$a' (sim:=, map:=)" >&2; exit 2 ;;
        esac
    done
    case "$sim" in carla) port={{world_port}} ;; isaac) port={{isaac_port}} ;;
        *) echo "world-check: sim '$sim' is not carla or isaac" >&2; exit 2 ;; esac
    exec "{{project}}/install/scenario_world_bridge/lib/scenario_world_bridge/swb_world_check" \
        conformance "tcp://127.0.0.1:$port" --map "$map"


# Build Autoware's message bindings for Isaac's Python (once per Isaac install; roadmap 038)
isaac-msgs:
    ISAAC_ROOT={{isaac_root}} "{{justfile_directory()}}/src/isaac_world/scripts/build_msgs.sh"

# <map>/isaac/world.yaml: from the map's point cloud as voxels (default), or usd:=<file> -- a
# USD world in the map frame, e.g. CARLA's SimReady export with its physics layer, ground
# from its semantic labels. Either replaces the other.
#
# Usage: just isaac-town <Town> [map_path:=<dir>] [export:=true]
#
# SimReady export when missing, collider layer, world file beside the town's Autoware map;
# host paths from CARLA_SRC and CARLA_USD.
# A CARLA town as an Isaac world (roadmap 043)
[positional-arguments]
isaac-town town *args:
    ISAAC_ROOT={{isaac_root}} "{{justfile_directory()}}/src/isaac_world/scripts/isaac_town.sh" "$@"

# Usage: just isaac-vehicles [only:=<id>,<id>...] [export:=true] [props|walkers|riders:=true]
#
# SimReady export of each vehicle, render-only asset, vehicles.json, into SWB_VEHICLE_USD;
# host paths from CARLA_SRC and SWB_VEHICLE_USD. Physics afterwards, with CARLA up:
# src/isaac_world/scripts/dump_vehicle_physics.py --root "$SWB_VEHICLE_USD".
# CARLA's vehicles as Isaac assets (roadmap 043)
[positional-arguments]
isaac-vehicles *args:
    ISAAC_ROOT={{isaac_root}} "{{justfile_directory()}}/src/isaac_world/scripts/isaac_vehicles.sh" "$@"

# Usage: just isaac-assets <check|manifest> [root:=<dir>]
#
# check: the set at root (default CARLA_USD) against its MANIFEST.sha256, each missing or
# changed file listed. manifest: write README.md and MANIFEST.sha256 there, after the set
# changes. docs/design/isaac-assets.md: what the set holds and how to obtain or make it.
# Check or seal the Isaac asset set: CARLA's towns, vehicles, props and walkers as USD
[positional-arguments]
isaac-assets command *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    command=$1; shift
    case "$command" in check|manifest) ;; *) echo "isaac-assets: check or manifest, not '$command'" >&2; exit 2 ;; esac
    root="${CARLA_USD:-/mnt/disk1/carla-usd}"
    for a in "$@"; do
        case "$a" in
            root:=*) root=$(realpath "${a#root:=}") ;;
            *) echo "isaac-assets: unknown argument '$a' (root:=)" >&2; exit 2 ;;
        esac
    done
    cd "{{justfile_directory()}}/src/isaac_world"
    python3 -m isaac_world.asset_set "$command" "$root"

# Usage: just isaac-world-gen [map_path:=<dir>] [usd:=<file>]
#
# The Isaac world for a map (roadmap 038): <map>/isaac/world.yaml
[positional-arguments]
isaac-world-gen *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    map_path="{{maps_dir}}/{{default_map}}"; usd=""
    for a in "$@"; do
        case "$a" in
            map_path:=*) map_path="${a#map_path:=}" ;;
            usd:=*) usd="${a#usd:=}" ;;
            *) echo "isaac-world-gen: unknown argument '$a' (map_path:=, usd:=)" >&2; exit 2 ;;
        esac
    done
    export ISAAC_ROOT={{isaac_root}}
    source "{{justfile_directory()}}/src/isaac_world/scripts/isaac_env.sh"
    cd "{{justfile_directory()}}/src/isaac_world"
    if [ -n "$usd" ]; then python -m isaac_world.usd_world "$map_path" "$usd"
    else python -m isaac_world.pcd_world "$map_path"; fi

# Usage: just surfaces-check [map_path:=<dir>]
#
# Refuses what the world server would refuse (every problem listed), warns about overlapping
# zones and zones off the Lanelet2 map; docs/design/off-road-surfaces.md has the format.
# Check a map's surface map, <map>/surfaces.yaml (roadmap 047)
[positional-arguments]
surfaces-check *args:
    @just _surfaces check "$@"

# Usage: just surfaces-plot [map_path:=<dir>] [out:=<png>]
#
# Zones (filled by class, hatched where soil), rasters and the label defaults over the
# Lanelet2 map; default out: <map>/surfaces.png.
# Plot a map's surface map over its lanes (roadmap 047)
[positional-arguments]
surfaces-plot *args:
    @just _surfaces plot "$@"

[positional-arguments]
_surfaces command *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    command=$1; shift
    map_path="{{maps_dir}}/{{default_map}}"; out=()
    for a in "$@"; do
        case "$a" in
            map_path:=*) map_path=$(realpath "${a#map_path:=}") ;;
            out:=*) [ "$command" = plot ] || { echo "surfaces-$command: out:= is plot's" >&2; exit 2; }
                    out=(-o "$(realpath -m "${a#out:=}")") ;;
            *) echo "surfaces-$command: unknown argument '$a' (map_path:=$([ "$command" = plot ] && echo ', out:='))" >&2; exit 2 ;;
        esac
    done
    cd "{{justfile_directory()}}/src/isaac_world"
    python3 -m isaac_world.surfaces "$command" "$map_path" "${out[@]}"

# GNOME with 3 static workspaces (Ctrl+Alt+1/2/3), OpenGL on the NVIDIA GPU through
# VirtualGL (-vgl; scripts/vnc_xstartup.sh). A GL app started from a terminal in the
# session needs `vglrun` (gnome-terminal does not inherit the preload).
#
# TurboVNC display for Isaac's window ($ISAAC_DISPLAY, default :8), localhost only (SSH tunnel)
isaac-vnc:
    /opt/TurboVNC/bin/vncserver -list | grep -q "^{{isaac_display}}\s" || /opt/TurboVNC/bin/vncserver {{isaac_display}} -localhost -securitytypes None -geometry 1920x1080 -vgl -xstartup "{{justfile_directory()}}/scripts/vnc_xstartup.sh"
    @echo "DISPLAY={{isaac_display}}"

# Place Isaac Sim, RViz and the terminals on the Isaac VNC display ($ISAAC_DISPLAY).
# mode:=tile (one screen, default) or mode:=ws (workspace 1 Isaac, 2 RViz, 3 terminals;
# Ctrl+Alt+1/2/3); wait:=SECS keeps placing while windows appear. A recording (x11grab)
# sees only the workspace on screen: record with mode:=tile.
#
# Usage: just vnc-layout [mode:=tile|ws] [wait:=SECS]
[positional-arguments]
vnc-layout *args:
    ISAAC_DISPLAY={{isaac_display}} "{{justfile_directory()}}/scripts/vnc_layout.sh" "$@"

# A gnome-terminal titled "swb: <title>" in the Isaac VNC session, running <cmd>
#
# Usage: just vnc-term title:=ego-av "cmd:=tail -f play_log/ego/latest/play_launch.log"
[positional-arguments]
vnc-term *args:
    ISAAC_DISPLAY={{isaac_display}} "{{justfile_directory()}}/scripts/vnc_layout.sh" term "$@"

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
# Brings up what is missing first (scripts/<sim>_up.sh, as `just scenario` does), then:
# sim:=carla (default): acb's acceptance.py -- the scenario's own verdict, that the ego
# actually drove somewhere, that no node died, tracking, non-OK diagnostics. sim:=isaac:
# scripts/acceptance_bag.py, the same checks and limits (and no MRM) scored from one bag of
# the ego's domain across the batch. Exit status is 0 only if every run passed every check.
#
# Usage: just acceptance [scenario] [runs] [domain] [sim:=carla|isaac] [worlds:=...]
#        just acceptance src/swb_examples/scenarios/basic/town01_ego_drive.xosc 3 sim:=isaac
[positional-arguments]
acceptance *args:
    #!/usr/bin/env bash
    set -e
    scenario="{{project}}/src/swb_examples/scenarios/basic/town01_ego_drive.xosc"
    source "{{lib}}"; swb_reject_env
    runs=1; domain=""; sim=carla; worlds=""; n=0
    for a in "$@"; do
        case "$a" in
            sim:=*) sim="${a#sim:=}" ;;
            backend:=*) echo "acceptance: backend:= is now sim:= (roadmap 046)" >&2; exit 2 ;;
            worlds:=*) worlds="${a#worlds:=}" ;;
            *:=*) echo "acceptance: unknown argument '$a' (sim:=, worlds:=)" >&2; exit 2 ;;
            *) n=$((n + 1)); case $n in 1) scenario="$a" ;; 2) runs="$a" ;; 3) domain="$a" ;;
                   *) echo "acceptance: too many arguments" >&2; exit 2 ;; esac ;;
        esac
    done
    source "{{autoware_setup}}"
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export ISAAC_PORT={{isaac_port}} WORLD_PORT={{world_port}} SSV2_PORT={{ssv2_port}} AGENT_PORT={{agent_port}}
    case "$sim" in
        carla)
            export CARLA_PORT={{carla_port}} CARLA_MAPS="{{maps_dir}}"
            "{{project}}/scripts/carla_up.sh" "$(just _scenario-map "$scenario")"
            args=(--scenario "$scenario" --runs "$runs" --play-log "$SWB_LOG_ROOT/ego")
            # acceptance.py watches the ego's domain (default 1, `ego_domain`).
            [ -n "$domain" ] && args+=(--domain "$domain")
            exec python3 "{{acb_src}}/scripts/acceptance.py" "${args[@]}" ;;
        isaac)
            exec python3 "{{project}}/scripts/acceptance_bag.py" --scenario "$scenario" \
                --runs "$runs" --sim isaac --domain "${domain:-{{ego_domain}}}" \
                ${worlds:+--worlds "$worlds"} ;;
        *) echo "acceptance: sim '$sim' is not carla or isaac" >&2; exit 2 ;;
    esac


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
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
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
    source "{{project}}/scripts/autoware_overlay.bash"
    swb_autoware_overlay_source "{{autoware_overlay_dir}}" true
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # Look where the ego actually is: its own domain (roadmap 017), over its RMW.
    export ROS_DOMAIN_ID={{ego_domain}}
    if [ -n "${EGO_RMW_IMPLEMENTATION:-}" ]; then export RMW_IMPLEMENTATION="$EGO_RMW_IMPLEMENTATION"; fi
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
# the scenario through the agent relay `just run` starts.
#
# Arguments are acb_launch carla_simulator.launch.xml's, name:=value, and win over the
# defaults here: map_path:=<dir> (default the Town01 map; a bare directory as the first
# argument is the same), rviz:=true, rviz_config:=<file>, relay:=<endpoint> (relay:= for
# none, with goal_poses_file:=<file> to drive to a local goal), publish_clock:=,
# control_trace_path:=, lidar_detection_model:=, steering_multiplier:=, ... ;
# accel_map_path:=, brake_map_path:= and traffic_light_map_path:= with no value turn that
# input off. The recipe's own:
#   overlay:=<setup.bash>         a workspace sourced between Autoware and acb, so its
#                                 packages shadow Autoware's but not acb's
#   autoware_overlay:=false       stock Autoware, without `just build-autoware-overlay`'s fixes
#   container_mode:=<mode>        play_launch's, default isolated
#   play_launch_args:="<opts>"    further play_launch options, split on whitespace
#
# Usage: just ego-av [map_path:=<dir>] [rviz:=true] [name:=value ...]
[positional-arguments]
ego-av *args: _require-carla
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    overlay=""; autoware_overlay=true; container_mode=isolated; play_launch_args=""
    given=()
    for a in "$@"; do
        case "$a" in
            overlay:=*) overlay="${a#*:=}" ;;
            autoware_overlay:=*) autoware_overlay="${a#*:=}" ;;
            container_mode:=?*) container_mode="${a#*:=}" ;;
            play_launch_args:=*) play_launch_args="${a#*:=}" ;;
            # The bridge spells "off" as `none`: an empty map path means that.
            accel_map_path:=|brake_map_path:=|traffic_light_map_path:=) given+=("${a}none") ;;
            relay:=none) given+=("relay:=") ;;
            *:=*) given+=("$a") ;;
            *) [ ${#given[@]} = 0 ] || { echo "ego-av: '$a' is not name:=value" >&2; exit 2; }
               given+=("map_path:=$a") ;;
        esac
    done
    # Three overlaid workspaces: base Autoware, then acb (acb_launch, sensor kit,
    # vehicle description, acb_bridge), then this one (swb_launch). The ego stack
    # resolves packages from all three. Right after Autoware comes the Autoware fix
    # overlay (`just build-autoware-overlay`, roadmap 042), then overlay:=, when given, so
    # its packages shadow Autoware's but not acb's: e.g. a tier4_system_launch with the
    # stock MRM nodes removed (a safety island's).
    source "{{autoware_setup}}"
    source "{{project}}/scripts/autoware_overlay.bash"
    swb_autoware_overlay_source "{{autoware_overlay_dir}}" "$autoware_overlay"
    if [ -n "$overlay" ]; then source "$overlay"; fi
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    # TensorRT 10.8.0 first on LD_LIBRARY_PATH, after the setups that rewrite it: the version
    # Autoware was compiled against, so cached engines validate (`just setup-tensorrt`).
    source "{{project}}/scripts/autoware_tensorrt.bash"
    swb_tensorrt_source "{{autoware_tensorrt_lib}}"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count - each run randomly failed to match
    # a different ADAPI service. Every ROS process in the pipeline must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # Any domain works since roadmap 017: the scenario reaches this ego through the agent
    # relay (TCP), not over ROS. Kept apart from the scenario domain on purpose.
    export ROS_DOMAIN_ID={{ego_domain}}
    # EGO_RMW_IMPLEMENTATION (a host setting, like the domain) puts the ego on another RMW
    # (e.g. rmw_zenoh_cpp, which then needs its router up). Only the ego: the scenario side
    # stays on Cyclone, since the two meet over the relay's TCP socket. Exported after the
    # setups, which on some Autoware installs export RMW_IMPLEMENTATION themselves.
    # `_require-ego-stack` reads the same variable, so `just scenario` finds the ego.
    if [ -n "${EGO_RMW_IMPLEMENTATION:-}" ]; then export RMW_IMPLEMENTATION="$EGO_RMW_IMPLEMENTATION"; fi
    # The user's vehicle side, exactly (docs/user-guide.md section 4, Terminal 3):
    # acb_launch carla_simulator.launch.xml under play_launch with the guide's flags, so a
    # dev run proves what users get. Defaults that differ from the launch file's: no RViz,
    # the relay `just run` starts, and no localization seed on attach (the agent
    # initializes localization at the scenario's start pose, teleported; seeding from CARLA
    # as well would race it). acb publishes signal state from CARLA's lights (roadmap 015)
    # from the table swb writes beside the Lanelet2 map at every Initialize, which the
    # launch file's traffic_light_map_path default (<map_path>/carla/traffic_lights.yaml)
    # finds. A ROS launch argument cannot carry an empty value, so an empty one given here
    # drops the default and is not passed (swb_with_defaults).
    swb_with_defaults launch_args \
        map_path:="{{maps_dir}}/{{default_map}}" vehicle_name:=hero carla_port:={{carla_port}} \
        release_notify_endpoint:="tcp://127.0.0.1:$(({{ssv2_port}} + 1))" \
        release_ack_endpoint:="tcp://127.0.0.1:$(({{ssv2_port}} + 2))" \
        relay:="tcp://localhost:{{agent_port}}" rviz:=false seed_localization_on_attach:=false \
        -- "${given[@]}"
    log_dir=$(swb_log_dir "{{project}}" ego)
    #
    # --container-mode isolated --composable-respawn on-crash: Autoware 1.5.0's
    # behavior_path_planner aborts now and then on a new route (autoware_universe#12460, once
    # in 27 UC-ACC variants on 2026-10-10); its containers declare no respawn, so without a
    # reload the stack never plans again. A composable can only be reloaded alone when it is
    # a process of its own, which is play_launch's isolated mode (0.15 defaults to observable,
    # one process per container, as ros2 launch). The guide's command carries both.
    # container_mode:=observable: on rmw_zenoh_cpp every process is a zenoh peer and the
    # peers mesh, so isolated's ~200 processes cost far more than observable's ~80 (one per
    # container); observable gives up reloading one crashed composable alone.
    exec play_launch launch --web-addr 0.0.0.0:$((SWB_WEB_PORT + 2)) \
        --container-mode "$container_mode" --composable-respawn on-crash \
        --log-dir "$log_dir" \
        $play_launch_args \
        acb_launch carla_simulator.launch.xml "${launch_args[@]}"

# The ego's Autoware on Isaac Sim (roadmaps 038, 045): aib_launch isaac_simulator.launch.xml,
# which is `just ego-av`'s stack (acb's carla_simulator.launch.xml) with aib's vehicle and
# sensor kit (aib_vehicle, aib_sensor_kit) and no acb_bridge -- the world server (`just
# isaac-world`) is the vehicle interface, the sensors and /clock on the ego's domain -- and
# the same vehicle agent registered with the relay. Long-lived like `just ego-av`; start it
# once the world server is up.
#
# Arguments pass through to carla_simulator.launch.xml, name:=value, appended last so they
# win: map_path:=<dir> (default the Town01 map), rviz:=true, rviz_config:=<file>,
# relay:=tcp://host:port, goal_poses_file:=<file>, ... (no acb_bridge: nothing CARLA in the
# stack, no V2X signals; Isaac has no lights, 037).
# overlay:=, autoware_overlay:=, container_mode:= and play_launch_args:= as for `just ego-av`.
#
# Usage: just ego-isaac [map_path:=<dir>] [rviz:=true] [name:=value ...]
[positional-arguments]
ego-isaac *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    overlay=""; autoware_overlay=true; container_mode=isolated; play_launch_args=""
    given=()
    for a in "$@"; do
        case "$a" in
            overlay:=*) overlay="${a#*:=}" ;;
            autoware_overlay:=*) autoware_overlay="${a#*:=}" ;;
            container_mode:=?*) container_mode="${a#*:=}" ;;
            play_launch_args:=*) play_launch_args="${a#*:=}" ;;
            relay:=none) given+=("relay:=") ;;
            *:=*) given+=("$a") ;;
            *) echo "ego-isaac: '$a' is not name:=value" >&2; exit 2 ;;
        esac
    done
    source "{{autoware_setup}}"
    source "{{project}}/scripts/autoware_overlay.bash"
    swb_autoware_overlay_source "{{autoware_overlay_dir}}" "$autoware_overlay"
    if [ -n "$overlay" ]; then source "$overlay"; fi
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    source "{{project}}/scripts/autoware_tensorrt.bash"
    swb_tensorrt_source "{{autoware_tensorrt_lib}}"
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export ROS_DOMAIN_ID={{ego_domain}}
    # Defaults, each dropped when an argument names the same launch argument.
    swb_with_defaults launch_args \
        map_path:="{{maps_dir}}/{{default_map}}" \
        relay:="tcp://localhost:{{agent_port}}" rviz:=false \
        -- "${given[@]}"
    log_dir=$(swb_log_dir "{{project}}" ego)
    exec play_launch launch --web-addr 0.0.0.0:$((SWB_WEB_PORT + 2)) \
        --container-mode "$container_mode" --composable-respawn on-crash \
        --log-dir "$log_dir" \
        $play_launch_args \
        aib_launch isaac_simulator.launch.xml "${launch_args[@]}"

# Launch one background AV's vehicle side -- Autoware + acb_bridge + vehicle agent -- in its
# own ROS domain. The background AV is a scenario entity whose controller is `agent`
# (src/swb_examples/scenarios/multi_av/town01_two_av.xosc): swb spawns it with role_name = the entity name, and this
# stack's agent registers with the agent relay under that name and takes the scenario's goals.
# Long-lived, like `just ego-av`, and startable before or after the scenario.
#
# Domains start at BG_ROS_DOMAIN_ID (2): 9 is the scenario's, 1 the ego's, and 0 is left free so that a stray
# unconfigured ROS process cannot join a run's graph. A second background AV goes on 3 with
# its own web port: `just bg-av bg_av_2 3 $((SWB_WEB_PORT + 6))` (+5 is the Isaac world server's).
#
# Up to three positional arguments, then background_av.launch.xml's as name:=value:
# map_path:=<dir> (default the Town01 map), launch_rviz:=true, rviz_config:=<file>,
# relay:=<endpoint> (relay:=none with goal_poses_file:=<file> drives once to a local goal, no
# scenario), ...; autoware_overlay:=false runs stock Autoware.
#
# Usage: just bg-av [entity] [domain] [web_port] [map_path:=<dir>] [name:=value ...]
[positional-arguments]
bg-av *args: _require-carla
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    entity=bg_av_1; domain={{bg_domain}}; web_port=$((SWB_WEB_PORT + 3)); autoware_overlay=true; given=(); n=0
    for a in "$@"; do
        case "$a" in
            autoware_overlay:=*) autoware_overlay="${a#*:=}" ;;
            relay:=) given+=("relay:=none") ;;
            *:=*) given+=("$a") ;;
            *) n=$((n + 1))
               case $n in 1) entity="$a" ;; 2) domain="$a" ;; 3) web_port="$a" ;;
                   *) echo "bg-av: too many positional arguments (entity domain web_port)" >&2; exit 2 ;; esac ;;
        esac
    done
    source "{{autoware_setup}}"
    # The same Autoware fix overlay as the ego's (roadmap 042)
    source "{{project}}/scripts/autoware_overlay.bash"
    swb_autoware_overlay_source "{{autoware_overlay_dir}}" "$autoware_overlay"
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    source "{{project}}/scripts/autoware_tensorrt.bash"
    swb_tensorrt_source "{{autoware_tensorrt_lib}}"
    # Same loopback-unicast DDS config as every other process in the pipeline. DomainGain
    # 1000 in it is what keeps domain 1's unicast ports out of domain 0's range.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    export ROS_DOMAIN_ID="$domain"
    swb_with_defaults launch_args \
        entity:="$entity" relay:="tcp://localhost:{{agent_port}}" \
        map_path:="{{maps_dir}}/{{default_map}}" carla_port:={{carla_port}} launch_rviz:=false \
        release_notify_endpoint:="tcp://127.0.0.1:$(({{ssv2_port}} + 1))" \
        release_ack_endpoint:="tcp://127.0.0.1:$(({{ssv2_port}} + 2))" \
        -- "${given[@]}"
    log_dir=$(swb_log_dir "{{project}}" "bg-$entity")
    exec play_launch launch --web-addr 0.0.0.0:"$web_port" \
        --container-mode isolated --composable-respawn on-crash \
        --load-node-timeout 120 \
        --load-total-budget 180 \
        --log-dir "$log_dir" \
        swb_launch background_av.launch.xml "${launch_args[@]}"

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
            # Only this scenario domain's: a host shared with other runs (other domains, other
            # ports) has scenario stacks of its own that are not stale (roadmap 043).
            dom=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')
            [ "${dom:-0}" = "{{scenario_domain}}" ] || continue
            kill -9 "$pid" 2>/dev/null && stale=$((stale + 1))
        done
    done
    if [ "$stale" -gt 0 ]; then
        echo "[scenario] cleared $stale stale process(es) from a previous run"
        # ROS needs a moment to drop the old graph entries, or the new interpreter can
        # still collide with a name that is on its way out.
        sleep 3
    fi

# Run one scenario against the stacks, bringing up what is missing first: each simulator's
# scripts/<sim>_up.sh (roadmap 046) starts or reuses its simulator, the simulation side
# (`just run sim:=<sim>`) and the ego's Autoware, each waited for, and leaves them up for the
# next run. sim:=carla (default) or sim:=isaac. worlds:=<map>[,<map>] (Isaac): what a world
# server started here preloads, map names under CARLA_MAPS or paths (default: the scenario's
# map). output_directory:=<dir> (default /tmp): the verdict is
# <dir>/scenario_test_runner/result.junit.xml. global_timeout:=<s>: the runner's wall-clock cap
# (default 400 s; raise it for a scenario that drives longer).
#
# Usage: just scenario <xosc> [sim:=carla|isaac] [worlds:=Town01,..] [output_directory:=<dir>]
[positional-arguments]
scenario scenario_file *args:
    #!/usr/bin/env bash
    set -e
    shift
    source "{{lib}}"; swb_reject_env
    sim=carla; worlds=""; output_directory=/tmp; timeout_args=()
    for a in "$@"; do
        case "$a" in
            sim:=*) sim="${a#sim:=}" ;;
            backend:=*) echo "scenario: backend:= is now sim:= (roadmap 046)" >&2; exit 2 ;;
            worlds:=*) worlds="${a#worlds:=}" ;;
            output_directory:=?*) output_directory="${a#*:=}" ;;
            global_timeout:=?*) timeout_args=("global_timeout:=${a#*:=}") ;;
            *) echo "scenario: unknown argument '$a' (sim:=carla|isaac, worlds:=, output_directory:=, global_timeout:=)" >&2; exit 2 ;;
        esac
    done
    up="{{project}}/scripts/${sim}_up.sh"
    [ -x "$up" ] || { echo "scenario: no simulator '$sim' (no scripts/${sim}_up.sh)" >&2; exit 2; }
    # The repo's scenarios name their map as $(env CARLA_MAPS)/<Town>.
    export CARLA_MAPS="{{maps_dir}}"
    map_dir=$(just _scenario-map "{{scenario_file}}")
    resolved=()
    IFS=',' read -ra entries <<< "${worlds:-$map_dir}"
    for w in "${entries[@]}"; do
        case "$w" in */*) resolved+=("$w") ;; *) resolved+=("$CARLA_MAPS/$w") ;; esac
    done
    CARLA_PORT={{carla_port}} WORLD_PORT={{world_port}} ISAAC_PORT={{isaac_port}} \
        SSV2_PORT={{ssv2_port}} AGENT_PORT={{agent_port}} \
        "$up" "$map_dir" "$(IFS=,; echo "${resolved[*]}")"
    just _clear-stale-scenario
    # The interpreter resolves the vehicle description package (acb_vehicle_description).
    source "{{autoware_setup}}"
    if [ -f "{{acb_src}}/install/setup.bash" ]; then source "{{acb_src}}/install/setup.bash"; fi
    source "{{project}}/install/setup.bash"
    # Loopback-unicast DDS: lo multicast is disabled on this host and NIC-multicast
    # discovery flakes at this participant count. Every ROS process must share this.
    export CYCLONEDDS_URI="file://{{project}}/config/cyclonedds-localhost.xml"
    # SSv2 runs in the scenario domain with the agent relay (`just run`); the ego is
    # reached through the relay, wherever it runs (roadmap 017).
    export ROS_DOMAIN_ID={{scenario_domain}}
    # The scenario file is read, never modified: SSv2's runner preprocesses a copy in its
    # output directory (fork, roadmap 017).
    log_dir=$(swb_log_dir "{{project}}" scenario)
    exec play_launch launch --web-addr 0.0.0.0:$((SWB_WEB_PORT + 1)) \
        --log-dir "$log_dir" \
        swb_launch scenario.launch.xml \
        scenario:="$(realpath "{{scenario_file}}")" \
        output_directory:="$output_directory" \
        port:={{ssv2_port}} "${timeout_args[@]}"


# The map directory a scenario names (its LogicFile, $(env CARLA_MAPS) expanded); the
# default map when it names none.
_scenario-map scenario_file:
    #!/usr/bin/env bash
    maps="{{maps_dir}}"
    f=$(grep -oE 'LogicFile filepath="[^"]*"|filepath: *.*' "{{scenario_file}}" | head -1 \
        | sed -E 's/^LogicFile filepath="//; s/"$//; s/^filepath: *//')
    f="${f//\$(env CARLA_MAPS)/$maps}"
    echo "${f:-$maps/{{default_map}}}"

# Run the full stack and one scenario, then take down what it started: the stacks
# `just scenario sim:=<sim>` brings up (scripts/<sim>_up.sh; those already running are reused
# and left), the scenario, its verdict, then the stacks this run started are stopped
# (keep_stacks:=true leaves them; CARLA itself, a long-lived unit, is never stopped here).
# output_directory:=<dir> (default $SWB_LOG_ROOT/e2e/<time>) gets the scenario's output.
# swb_launch demo.launch.xml is the one-launch version of the same on CARLA, for a first try.
#
# Usage: just e2e [xosc] [map] [sim:=carla|isaac] [worlds:=..] [keep_stacks:=true] [output_directory:=<dir>]
[positional-arguments]
e2e *args:
    #!/usr/bin/env bash
    set -e
    source "{{lib}}"; swb_reject_env
    scenario="{{project}}/src/swb_examples/scenarios/basic/town01_ego_drive.xosc"
    map="{{default_map}}"; sim=carla; extra=(); keep_stacks=false; out=""; n=0
    for a in "$@"; do
        case "$a" in
            sim:=*) sim="${a#sim:=}" ;;
            backend:=*) echo "e2e: backend:= is now sim:= (roadmap 046)" >&2; exit 2 ;;
            worlds:=*) extra+=("$a") ;;
            keep_stacks:=*) keep_stacks="${a#*:=}" ;;
            output_directory:=?*) out="${a#*:=}" ;;
            *:=*) echo "e2e: unknown argument '$a' (sim:=, worlds:=, keep_stacks:=, output_directory:=)" >&2; exit 2 ;;
            *) n=$((n + 1)); case $n in 1) scenario="$a" ;; 2) map="$a" ;;
                   *) echo "e2e: too many arguments" >&2; exit 2 ;; esac ;;
        esac
    done
    up="{{project}}/scripts/${sim}_up.sh"
    [ -x "$up" ] || { echo "e2e: no simulator '$sim' (no scripts/${sim}_up.sh)" >&2; exit 2; }
    export CARLA_MAPS="{{maps_dir}}"
    started=$(mktemp)
    [ -n "$out" ] || out="$(swb_log_dir "{{project}}" e2e)/$(date +%Y-%m-%d_%H-%M-%S)"
    mkdir -p "$out"
    teardown() {
        if [ "$keep_stacks" = true ]; then
            echo "[e2e] keep_stacks:=true: leaving the stacks this run started up"
        elif [ -s "$started" ]; then
            echo "[e2e] stopping the stacks this run started"
            for pg in $(tac "$started"); do kill -TERM -"$pg" 2>/dev/null || true; done
            for _ in $(seq 30); do
                alive=0; for pg in $(cat "$started"); do kill -0 -"$pg" 2>/dev/null && alive=1; done
                [ $alive = 0 ] && break; sleep 1
            done
            for pg in $(cat "$started"); do kill -KILL -"$pg" 2>/dev/null || true; done
        fi
        rm -f "$started"
    }
    trap teardown EXIT
    SIM_UP_STARTED="$started" CARLA_PORT={{carla_port}} WORLD_PORT={{world_port}} \
        ISAAC_PORT={{isaac_port}} SSV2_PORT={{ssv2_port}} AGENT_PORT={{agent_port}} \
        "$up" "$CARLA_MAPS/$map" "$CARLA_MAPS/$map"
    just scenario "$scenario" sim:="$sim" output_directory:="$out" "${extra[@]}" || true
    junit="$out/scenario_test_runner/result.junit.xml"
    if [ ! -f "$junit" ]; then echo "[e2e] no verdict: $junit not written"; exit 1; fi
    if grep -q 'failures="0"' "$junit" && grep -q 'errors="0"' "$junit"; then
        echo "[e2e] PASS ($junit)"
    else
        echo "[e2e] FAIL ($junit)"; exit 1
    fi


# Move play_launch runs older than `days` out of $SWB_LOG_ROOT into an archive, kind by
# kind (<dest>/<kind>/<time>). Never deletes, never overwrites; skips each kind's `latest`,
# and runs a live process still has open (a running play_launch). dest defaults to
# $SWB_LOG_ARCHIVE (a host setting; default /mnt/disk1/csb-play_log-archive, a spinning disk,
# fine for an archive and not for live logs: CLAUDE.md). dry_run:=true lists only.
#
# Usage: just prune-logs [days:=7] [dest:=<dir>] [dry_run:=true]
[positional-arguments]
prune-logs *args:
    #!/usr/bin/env bash
    set -e
    days=7; dest="${SWB_LOG_ARCHIVE:-/mnt/disk1/csb-play_log-archive}"; dry=()
    for a in "$@"; do
        case "$a" in
            days:=?*) days="${a#*:=}" ;;
            dest:=?*) dest="${a#*:=}" ;;
            dry_run:=true) dry=(--dry-run) ;;
            dry_run:=false) ;;
            *) echo "prune-logs: unknown argument '$a' (days:=, dest:=, dry_run:=true)" >&2; exit 2 ;;
        esac
    done
    exec python3 "{{project}}/scripts/prune_logs.py" --root "$SWB_LOG_ROOT" --dest "$dest" \
        --days "$days" "${dry[@]}"

# Download pre-converted CARLA maps for Autoware (present ones: apply any newer repairs)
download-maps:
    "{{project}}/scripts/download_maps.sh"

# Proto bindings are generated automatically by prost-build in build.rs during cargo build.
# No manual generation step needed.
