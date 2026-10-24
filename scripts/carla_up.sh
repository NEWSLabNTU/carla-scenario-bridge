#!/usr/bin/env bash
# Bring up what a scenario on CARLA needs, reusing whatever already runs (roadmap 046): CARLA
# itself (`just carla-start`, a systemd unit), the simulation side on CARLA (`just run
# sim:=carla`, web SWB_WEB_PORT+4: the CARLA world server, the bridge and the agent relay in
# one launch) and the ego's Autoware (`just ego-av`, +2), in that order, each waited for. Run
# by `just scenario|acceptance|e2e ... sim:=carla` (the default). Each simulator's bring-up is
# scripts/<sim>_up.sh with this contract; isaac_up.sh is Isaac's.
#
#   scripts/carla_up.sh <map dir> [<worlds>]
#
# <worlds> is accepted for the contract's sake and unused: CARLA loads a scenario's town when
# the scenario asks. A stack on one of those ports that is not the CARLA kind (`just run
# sim:=isaac`, `just ego-isaac`) is refused, not replaced. Stacks started here stay up for the
# next run; their pids are appended to $SIM_UP_STARTED when that is set, so a caller can take
# them down (`just e2e`).
#
# The ego stack runs under `swb-agent supervise` (roadmap 051): a CARLA episode change (a world
# load, a restarted CARLA) ends it -- acb_bridge exits and takes the stack with it -- and the
# supervisor starts it again, on the same map, in the new episode. A stack found on another
# map than the scenario's is replaced by one on the scenario's: that one ends once more at the
# scenario's load of its town and comes back in that town's episode, so a town change costs
# two stack starts.
#
# Environment, host settings all (from the justfile): CARLA_PORT, WORLD_PORT, SSV2_PORT,
# AGENT_PORT, SWB_WEB_PORT, SWB_LOG_ROOT, and the waits CARLA_READY_TIMEOUT (s, default 300:
# a first start on a host serves RPC in 50-180 s) and EGO_READY_TIMEOUT (s, default 1200).
set -euo pipefail
map_dir="$1"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
logs="${SWB_LOG_ROOT:-$here/play_log}"
mkdir -p "$logs"
carla_port="${CARLA_PORT:-2000}"
world_port="${WORLD_PORT:-5591}"
ssv2_port="${SSV2_PORT:-5555}"
agent_port="${AGENT_PORT:-5560}"
web_base="${SWB_WEB_PORT:-8080}"
web_ego=$((web_base + 2)); web_bridge=$((web_base + 4))
say() { echo "[carla-up] $*" >&2; }

# The play_launch serving web port $1, if any: stacks are told apart by their port. Its Python
# wrapper counts too (`comm` python3): on a loaded host it ran three minutes before the real
# play_launch started, and a run that looked only for the latter started a second simulation
# side, whose bridge then fought the first for the world server.
stack_on_port() {
    local f p
    for f in /proc/[0-9]*/cmdline; do
        p=${f#/proc/}; p=${p%/cmdline}
        { tr '\0' ' ' < "$f"; } 2>/dev/null \
            | grep -qE -- "play_launch launch .*--web-addr [0-9.]*:$1 " \
            && { echo "$p"; return 0; }
    done
    return 1
}
cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null; }
started() { [ -n "${SIM_UP_STARTED:-}" ] && echo "$1" >> "$SIM_UP_STARTED"; return 0; }
start() {
    local name="$1"; shift
    say "starting $name: just $*"
    setsid just "$@" > "$logs/$name.console" 2>&1 < /dev/null &
    local pid=$!
    started "$pid"
    echo "$pid"
}
alive() { kill -0 "$1" 2>/dev/null; }
# The `swb-agent supervise` above play_launch $1, if any (it would start the stack again).
supervisor_of() {
    local p=$1
    while [ -n "$p" ] && [ "$p" -gt 1 ]; do
        p=$(ps -o ppid= "$p" 2>/dev/null | tr -d ' ')
        [ -n "$p" ] && cmdline "$p" | grep -q "swb_agent supervise" && { echo "$p"; return 0; }
    done
    return 1
}
# Stop a stack and wait for it: through its supervisor when it has one (which stops its group
# and does not start it again), else by its process group (the play_launch `start` put in one).
stop_stack() {
    local sup pgid
    if sup=$(supervisor_of "$1"); then
        kill -TERM "$sup" 2>/dev/null
    else
        pgid=$(ps -o pgid= "$1" 2>/dev/null | tr -d ' ')
        [ -n "$pgid" ] || return 0
        kill -TERM -"$pgid" 2>/dev/null
    fi
    for _ in $(seq 1 30); do alive "$1" || return 0; sleep 2; done
    pgid=$(ps -o pgid= "$1" 2>/dev/null | tr -d ' ')
    [ -n "$pgid" ] && kill -KILL -"$pgid" 2>/dev/null
    return 0
}
# The ego stack under its supervisor, in a session of its own; the supervisor's pid is the one
# recorded (SIGTERM to it stops the stack).
start_ego() {
    say "starting ego (supervised): just ego-av map_path:=$map_dir"
    PYTHONPATH="$here/src/swb_agent${PYTHONPATH:+:$PYTHONPATH}" setsid python3 -m swb_agent \
        supervise --entity ego --relay "tcp://localhost:$agent_port" \
        --agent "just ego-av map_path:=$map_dir" > "$logs/ego.console" 2>&1 < /dev/null &
    local pid=$!
    started "$pid"
    echo "$pid"
}
listening() { ss -ltn 2>/dev/null | grep -q ":$1 "; }
carla_ok() { "$here/src/autoware_carla_bridge/scripts/carla_health.py" --port "$carla_port" > /dev/null 2>&1; }

# 1. CARLA: ready when it serves RPC with a map loaded.
if carla_ok; then
    say "CARLA on $carla_port is up"
else
    if systemctl --user is-active --quiet "carla-run-$carla_port" 2>/dev/null; then
        say "CARLA's unit on $carla_port is starting: waiting"
    else
        say "starting CARLA on $carla_port: just carla-start"
        just carla-start > "$logs/carla.console" 2>&1
    fi
    deadline=$((SECONDS + ${CARLA_READY_TIMEOUT:-300}))
    until carla_ok; do
        [ $SECONDS -gt $deadline ] && {
            say "CARLA did not serve RPC within ${CARLA_READY_TIMEOUT:-300} s (just carla-logs)"; exit 1; }
        sleep 5
    done
    say "CARLA on $carla_port is up ($SECONDS s)"
fi

# 2. The simulation side on CARLA: ready when the world server answers, the bridge and the
#    relay listen.
if pid=$(stack_on_port "$web_bridge"); then
    if ! cmdline "$pid" | grep -q "carla_simulation.launch.xml"; then
        say "the simulation side on $web_bridge (pid $pid) is not the CARLA one:"
        say "  $(cmdline "$pid" | cut -c1-200)"
        say "stop it (kill $pid) and run again, or start it with \`just run sim:=carla\`"
        exit 1
    fi
    say "simulation side (pid $pid) is up on CARLA"
else
    if listening "$ssv2_port"; then
        say "something outside play_launch holds port $ssv2_port:"
        ss -ltnp 2>/dev/null | grep ":$ssv2_port " || true
        exit 1
    fi
    pid=$(start bridge run sim:=carla)
fi
deadline=$((SECONDS + 180))
until listening "$ssv2_port" && listening "$agent_port"; do
    alive "$pid" || { say "the simulation side exited; see $logs/bridge.console"; exit 1; }
    [ $SECONDS -gt $deadline ] && {
        say "bridge/relay not listening after 180 s: $(just world-health sim:=carla 2>&1 | tail -1)"; exit 1; }
    sleep 2
done

# A scenario with no ego (the NPC benches) needs no ego stack: with SIM_UP_EGO=0, which `just
# scenario` and `just e2e` set from the scenario (`_scenario-has-ego`), this step is skipped and
# an ego stack already up is left alone.
if [ "${SIM_UP_EGO:-1}" = 0 ]; then
    say "ready: CARLA, simulation side (CARLA world server, bridge, relay) (the scenario has no ego: no ego stack)"
    exit 0
fi

# 3. The ego's Autoware on CARLA: ready when its play_launch reports startup complete and the
#    ADAPI answers (`_require-ego-stack`), then its agent registered with the relay.
if pid=$(stack_on_port "$web_ego"); then
    line=$(cmdline "$pid")
    if ! grep -q "acb_launch carla_simulator.launch.xml" <<< "$line"; then
        say "the ego stack on $web_ego (pid $pid) is not the CARLA one (just ego-isaac?): stop it first"
        exit 1
    fi
    case "$line" in
        *"map_path:=$map_dir "*|*"map_path:=$map_dir") say "ego stack (pid $pid) is up" ;;
        *)  # A stack's map is its episode's (roadmap 051): one on another map starts again.
            say "the ego stack (pid $pid) has another map ($(grep -oE 'map_path:=[^ ]*' <<< "$line")): restarting it on $map_dir"
            stop_stack "$pid"
            pid="" ;;
    esac
fi
if [ -z "${pid:-}" ]; then
    pid=$(start_ego)
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
say "ready: CARLA, simulation side (CARLA world server, bridge, relay), ego stack and its agent"
