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

say "restarting background AV stack for a fresh pilot"
just bg-av > "$S/same_lane_bg.log" 2>&1 &
for i in $(seq 1 90); do
    grep -q "Startup complete" "$S/same_lane_bg.log" 2>/dev/null && break
    sleep 10
done
grep -q "Startup complete" "$S/same_lane_bg.log" || { say "bg stack never came up"; exit 1; }
say "bg up"

for p in $(pgrep -x play_launch); do
    tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q 8081 && kill "$p"
done
sleep 5

say "scenario start"
just scenario "$PWD/scenarios/town01_two_av.xosc" > "$S/same_lane_run.log" 2>&1 &
sleep 20

bash -c "source /opt/autoware/1.5.0/setup.bash >/dev/null 2>&1
         source $PWD/src/autoware_carla_bridge/install/setup.bash >/dev/null 2>&1
         export CYCLONEDDS_URI=file://$PWD/config/cyclonedds-localhost.xml
         export ROS_DOMAIN_ID=1
         python3 $PWD/scripts/perception_probe.py 170" > "$S/same_lane_perception.txt" 2>&1

for i in $(seq 1 40); do
    [ /tmp/scenario_test_runner/result.junit.xml -nt "$S/same_lane_run.log" ] && break
    sleep 10
done
say "verdict: $(sed -n '2,7p' /tmp/scenario_test_runner/result.junit.xml | tr -d '\n' | sed 's/  */ /g' | cut -c1-200)"
say "NRestarts=$(systemctl --user show carla-run-2000 -p NRestarts --value)"
