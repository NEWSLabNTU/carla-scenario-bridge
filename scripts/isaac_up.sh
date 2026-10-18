#!/usr/bin/env bash
# Bring up what a scenario on Isaac Sim needs, reusing whatever already runs (roadmap 043):
# the world server (`just isaac-world`, web 8085), the simulation side on backend isaac
# (`just run backend:=isaac`, web 8084) and the ego's Autoware (`just ego-isaac`, web 8082),
# in that order, each waited for. Run by `just scenario|acceptance|e2e ... backend:=isaac`.
#
#   scripts/isaac_up.sh <map dir> [<world>[,<world>...]]
#
# The worlds are what a world server started here preloads (default: the map dir's); an
# already running one is used as it is. A stack on one of those ports that is not the Isaac
# kind (the CARLA bridge, `just ego-av`) is refused, not replaced. Stacks started here stay
# up for the next run; their pids are appended to $ISAAC_UP_STARTED when that is set, so a
# caller can take them down (`just e2e`).
#
# Environment (from the justfile): ISAAC_PORT, SSV2_PORT, AGENT_PORT, and the waits
# ISAAC_READY_TIMEOUT (s, default 1800: a town's first render on a host compiles its
# materials), EGO_READY_TIMEOUT (s, default 1200).
set -euo pipefail
map_dir="$1"
worlds="${2:-$map_dir}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
logs="$here/play_log"
mkdir -p "$logs"
isaac_port="${ISAAC_PORT:-5590}"
ssv2_port="${SSV2_PORT:-5555}"
agent_port="${AGENT_PORT:-5560}"
say() { echo "[isaac-up] $*" >&2; }

# The play_launch serving web port $1, if any: stacks are told apart by their port.
stack_on_port() {
    local p
    for p in $(pgrep -x play_launch); do
        tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- "--web-addr [0-9.]*:$1 " \
            && { echo "$p"; return 0; }
    done
    return 1
}
cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null; }
started() { [ -n "${ISAAC_UP_STARTED:-}" ] && echo "$1" >> "$ISAAC_UP_STARTED"; return 0; }
# `setsid` so each stack is its own process group (kill -TERM -<pid> takes the whole tree).
start() {
    local name="$1"; shift
    say "starting $name: just $*"
    setsid just "$@" > "$logs/$name.console" 2>&1 < /dev/null &
    local pid=$!
    started "$pid"
    echo "$pid"
}
alive() { kill -0 "$1" 2>/dev/null; }

# 1. The world server: ready when it answers Hello.
if just isaac-health > /dev/null 2>&1; then
    say "world server on $isaac_port is up"
else
    if pid=$(stack_on_port 8085); then
        say "world server launch (pid $pid) is up but not answering yet: waiting"
    else
        world_args=()
        if [ -n "$worlds" ]; then
            # Only worlds that exist: a map with no isaac/world.yaml runs on a ground plane.
            keep=()
            IFS=',' read -ra entries <<< "$worlds"
            for w in "${entries[@]}"; do
                if [ -d "$w" ]; then
                    [ -f "$w/isaac/world.yaml" ] && keep+=("$w") \
                        || say "no $w/isaac/world.yaml: not preloaded (a ground plane)"
                elif [ -e "$w" ]; then keep+=("$w")
                else say "no world $w: not preloaded"; fi
            done
            [ ${#keep[@]} -gt 0 ] && world_args=("world:=$(IFS=,; echo "${keep[*]}")")
        fi
        pid=$(start isaac isaac-world "${world_args[@]}")
    fi
    deadline=$((SECONDS + ${ISAAC_READY_TIMEOUT:-1800}))
    until just isaac-health > /dev/null 2>&1; do
        if ! alive "$pid"; then
            say "the world server launch exited before answering; see $logs/isaac.console" \
                "and play_log/isaac/latest/"
            exit 1
        fi
        if [ $SECONDS -gt $deadline ]; then
            say "the world server did not answer within ${ISAAC_READY_TIMEOUT:-1800} s"
            exit 1
        fi
        sleep 10
    done
    say "world server on $isaac_port is up ($((SECONDS)) s)"
fi

# 2. The simulation side, on backend isaac: ready when the bridge and relay listen.
listening() { ss -ltn 2>/dev/null | grep -q ":$1 "; }
if pid=$(stack_on_port 8084); then
    if ! cmdline "$pid" | grep -q "backend:=isaac"; then
        say "the simulation side on 8084 (pid $pid) is not on backend isaac:"
        say "  $(cmdline "$pid" | cut -c1-200)"
        say "stop it (kill $pid) and run again, or start it with \`just run backend:=isaac\`"
        exit 1
    fi
    say "simulation side (pid $pid) is up on backend isaac"
else
    if listening "$ssv2_port"; then
        say "something outside play_launch holds port $ssv2_port:"
        ss -ltnp 2>/dev/null | grep ":$ssv2_port " || true
        exit 1
    fi
    pid=$(start bridge run backend:=isaac)
fi
deadline=$((SECONDS + 120))
until listening "$ssv2_port" && listening "$agent_port"; do
    alive "$pid" || { say "the simulation side exited; see $logs/bridge.console"; exit 1; }
    [ $SECONDS -gt $deadline ] && { say "bridge/relay not listening after 120 s"; exit 1; }
    sleep 2
done

# 3. The ego's Autoware on Isaac: ready when its play_launch reports startup complete and
#    the ADAPI answers (`_require-ego-stack`), then its agent registered with the relay.
if pid=$(stack_on_port 8082); then
    line=$(cmdline "$pid")
    if ! grep -q "launch_vehicle_interface:=false" <<< "$line"; then
        say "the ego stack on 8082 (pid $pid) is not the Isaac one (just ego-av?): stop it first"
        exit 1
    fi
    case "$line" in
        *"map_path:=$map_dir "*|*"map_path:=$map_dir") ;;
        *) say "note: the running ego stack has another map: $(grep -oE 'map_path:=[^ ]*' <<< "$line")" ;;
    esac
    say "ego stack (pid $pid) is up"
else
    pid=$(start ego ego-isaac "map_path:=$map_dir")
    deadline=$((SECONDS + ${EGO_READY_TIMEOUT:-1200}))
    until grep -q "Startup complete" "$logs/ego.console" 2>/dev/null; do
        alive "$pid" || { say "the ego stack exited; see $logs/ego.console"; exit 1; }
        [ $SECONDS -gt $deadline ] && { say "the ego stack is not up after ${EGO_READY_TIMEOUT:-1200} s"; exit 1; }
        sleep 10
    done
    say "ego stack up"
fi
deadline=$((SECONDS + 300))
until just _require-ego-stack > /dev/null 2>&1 && just _require-agent ego > /dev/null 2>&1; do
    [ $SECONDS -gt $deadline ] && {
        say "the ego's ADAPI or its agent is not ready after 300 s:"
        just _require-ego-stack || true; just _require-agent ego || true
        exit 1; }
    sleep 5
done
say "ready: world server, simulation side (isaac), ego stack and its agent"
