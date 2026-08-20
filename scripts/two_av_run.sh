#!/usr/bin/env bash
# Two AVs in one lane: bring the background stack up fresh (its pilot exits after one
# run), start the long scenario, and watch the ego's perception while both drive.
set -u
cd "$(dirname "$0")/.."
S="${TWO_AV_LOG_DIR:-/tmp/two_av}"
mkdir -p "$S"
export DISPLAY=:1

say() { echo "[$(date +%H:%M:%S)] $*"; }

# `just run` must already be up: this harness does not own the bridge.
if ! pgrep -x carla_scenario_ >/dev/null; then
    say "the bridge is not running -- start it with \`just run\` first"
    exit 1
fi
say "bridge up, NRestarts=$(systemctl --user show carla-run-2000 -p NRestarts --value)"

# Stop whatever background stack is already up before starting one. This used to say
# "restarting" and only ever start: a second invocation left two full Autoware stacks in
# domain 2, both bridges attached to bg_av_1, and the ego's own perception then broke
# down badly enough that the scenario timed out. Match on the web port, since `pkill -f`
# on a pattern this script also contains would kill the script.
# SIGTERM only, and then WAIT. play_launch tears down its own children when it catches a
# signal -- measured at 86 component_node processes reaped in 4 s -- but SIGKILL cannot be
# caught, so killing it outright strands the whole tree on init. Polling until the stack is
# actually gone is what keeps a run from inheriting the previous one's processes.
kill_stack_on_port() {
    local port="$1" waited=0 alive
    for p in $(pgrep -x play_launch); do
        tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q "$port" && kill "$p"
    done
    while [ "$waited" -lt 90 ]; do
        alive=0
        for p in $(pgrep -x play_launch); do
            tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q "$port" && alive=1
        done
        [ "$alive" -eq 0 ] && return 0
        sleep 2; waited=$((waited + 2))
    done
    say "stack on $port did not exit after 90s"
    return 1
}

# Anything reparented to init outlived its stack. Report it rather than letting the next
# run inherit it silently.
orphan_report() {
    local n
    n=$(ps -eo ppid,comm --no-headers | awk '$1==1 && ($2=="component_node" ||
        $2=="component_conta" || $2=="play_launch")' | wc -l)
    [ "$n" -gt 0 ] && say "WARNING: $n orphaned process(es) left behind"
    return 0
}

say "restarting background AV stack for a fresh pilot"
kill_stack_on_port 8083
orphan_report
just bg-av > "$S/same_lane_bg.log" 2>&1 &
for i in $(seq 1 90); do
    grep -q "Startup complete" "$S/same_lane_bg.log" 2>/dev/null && break
    sleep 10
done
grep -q "Startup complete" "$S/same_lane_bg.log" || { say "bg stack never came up"; exit 1; }
say "bg up"

kill_stack_on_port 8081
orphan_report

say "scenario start"
just scenario "$PWD/scenarios/town01_two_av.xosc" > "$S/same_lane_run.log" 2>&1 &
sleep 20

# Long enough to outlast the scenario's own 300 s timeout, so a run that stalls is still
# on the record rather than cut off mid-crawl.
bash -c "source /opt/autoware/1.5.0/setup.bash >/dev/null 2>&1
         source $PWD/src/autoware_carla_bridge/install/setup.bash >/dev/null 2>&1
         export CYCLONEDDS_URI=file://$PWD/config/cyclonedds-localhost.xml
         export ROS_DOMAIN_ID=1
         python3 -u $PWD/scripts/perception_probe.py 290" > "$S/same_lane_perception.txt" 2>&1

for i in $(seq 1 40); do
    [ /tmp/scenario_test_runner/result.junit.xml -nt "$S/same_lane_run.log" ] && break
    sleep 10
done
say "verdict: $(sed -n '2,7p' /tmp/scenario_test_runner/result.junit.xml | tr -d '\n' | sed 's/  */ /g' | cut -c1-200)"
say "NRestarts=$(systemctl --user show carla-run-2000 -p NRestarts --value)"
