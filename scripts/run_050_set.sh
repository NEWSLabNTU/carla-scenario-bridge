#!/usr/bin/env bash
# Roadmap 050's measurement set, before or after the box change (S1, S7):
#
#   scripts/run_050_set.sh <label> [groups...]      groups: default ego, tour, bench, two_av
#
# Runs the scenarios against this checkout's stacks and leaves, under play_log/050/<label>/:
# one directory per scenario (SSv2's output, the verdict), the bridge's gap traces
# (traces/, one per Initialize), runs.tsv (scenario, start, end, exit, verdict) and the
# stack logs. The simulation side is started here with gap_trace_dir set, and the ego stack
# per group with the arguments that group is timed for; both are stopped at the end.
# Results are read by scripts/report_050.py.
set -u
cd "$(dirname "$0")/.."
label=${1:?label}; shift
groups=("$@"); [ ${#groups[@]} -eq 0 ] && groups=(default tour bench two_av)
set -a; . ./.env; set +a
out="$PWD/play_log/050/$label"
mkdir -p "$out/traces"
web=${SWB_WEB_PORT:-8080}
ssv2=${SSV2_PORT:-5555}

say() { echo "[050 $label] $(date +%T) $*" | tee -a "$out/driver.log"; }

stack_pid() {  # the play_launch serving web port $1
    for p in $(pgrep -x play_launch); do
        tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- "--web-addr [0-9.]*:$1 " \
            && { echo "$p"; return 0; }
    done
    return 1
}

stop_stack() {  # $1 web port
    local pid
    pid=$(stack_pid "$1") || return 0
    say "stopping the stack on web $1 (pid $pid)"
    kill -TERM -"$(ps -o pgid= "$pid" | tr -d ' ')" 2>/dev/null || kill -TERM "$pid"
    for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || return 0; sleep 2; done
    kill -KILL -"$(ps -o pgid= "$pid" | tr -d ' ')" 2>/dev/null || kill -KILL "$pid"
}

start_sim() {
    if ss -lnt | grep -q ":$ssv2 "; then
        say "a simulation side already holds $ssv2: stop it first (its traces would go elsewhere)"
        exit 1
    fi
    say "starting the simulation side, gap traces to $out/traces"
    setsid just run gap_trace_dir:="$out/traces" > "$out/sim.log" 2>&1 < /dev/null &
    for _ in $(seq 1 300); do
        ss -lnt | grep -q ":$ssv2 " && ss -lnt | grep -q ":${WORLD_PORT:-5591} " && return 0
        sleep 2
    done
    say "the simulation side did not come up; see $out/sim.log"; exit 1
}

start_ego() {  # $1 log name, then ego-av arguments
    local name=$1; shift
    stop_stack $((web + 2))
    say "starting the ego stack ($name): $*"
    setsid just ego-av "$@" > "$out/ego_$name.log" 2>&1 < /dev/null &
    local pg=$! deadline=$((SECONDS + 1500))
    until grep -q "Startup complete" "$out/ego_$name.log" 2>/dev/null; do
        kill -0 "$pg" 2>/dev/null || { say "ego stack exited; see $out/ego_$name.log"; exit 1; }
        [ $SECONDS -gt $deadline ] && { say "ego stack not up in 1500 s"; exit 1; }
        sleep 10
    done
    # Startup complete is the launch; Autoware answers the ADAPI a little later.
    sleep 60
    say "ego stack up"
}

weather() {
    ROS_DOMAIN_ID=$SCENARIO_ROS_DOMAIN_ID CYCLONEDDS_URI="file://$PWD/config/cyclonedds-localhost.xml" \
        bash -c "source /opt/autoware/1.5.0/setup.bash; source install/setup.bash; \
        timeout 30 ros2 service call /scenario_world_bridge/set_weather \
        swb_interfaces/srv/SetWeather '{preset: $1}'" 2>&1 | grep -o "success=[A-Za-z]*" \
        | tee -a "$out/driver.log"
}

run() {  # name xosc [scenario args...]
    local name=$1 xosc=$2; shift 2
    local dir="$out/$name" start end code verdict
    rm -rf "$dir"; mkdir -p "$dir"
    say "run $name"
    start=$(date +%s.%N)
    just scenario "$xosc" output_directory:="$dir" "$@" > "$dir/run.log" 2>&1 < /dev/null
    code=$?
    end=$(date +%s.%N)
    verdict=missing
    if [ -f "$dir/scenario_test_runner/result.junit.xml" ]; then
        grep -q 'failures="0" errors="0"' "$dir/scenario_test_runner/result.junit.xml" \
            && verdict=pass || verdict=fail
    fi
    printf '%s\t%s\t%s\t%s\t%s\n' "$name" "$start" "$end" "$code" "$verdict" >> "$out/runs.tsv"
    say "$name: exit $code, $verdict"
    sleep 5
}

two_av() {
    local dir="$out/town01_two_av" start end code verdict
    rm -rf "$dir"; mkdir -p "$dir"
    say "run town01_two_av (just two-av)"
    start=$(date +%s.%N)
    just two-av output_directory:="$dir" > "$dir/run.log" 2>&1 < /dev/null
    code=$?
    end=$(date +%s.%N)
    verdict=missing
    if [ -f "$dir/scenario_test_runner/result.junit.xml" ]; then
        grep -q 'failures="0" errors="0"' "$dir/scenario_test_runner/result.junit.xml" \
            && verdict=pass || verdict=fail
    fi
    printf '%s\t%s\t%s\t%s\t%s\n' town01_two_av "$start" "$end" "$code" "$verdict" >> "$out/runs.tsv"
    say "town01_two_av: exit $code, $verdict"
}

basic=src/swb_examples/scenarios/basic
inc=src/swb_examples/scenarios/demo/incidents

start_sim
for g in "${groups[@]}"; do
    case "$g" in
        default)
            start_ego default
            for s in town01_ego_drive town01_pedestrian town01_traffic_light town01_simulator_autopilot; do
                run "$s" "$basic/$s.xosc"
            done
            for s in town01_entity_bodies town01_motorbike_body town01_npc_shoulder town01_truck_pass; do
                run "gap_$s" "scenarios/gap/$s.xosc"
            done
            ;;
        truck)
            stack_pid $((web + 2)) > /dev/null || start_ego default
            for i in 1 2 3 4; do
                run "gap_town01_truck_pass_$i" scenarios/gap/town01_truck_pass.xosc
            done
            ;;
        two_av)
            stack_pid $((web + 2)) > /dev/null || start_ego default
            two_av
            ;;
        tour)
            start_ego tour max_velocity:=11.11 lidar_detection_model:=centerpoint \
                use_multi_channel_tracker_merger:=false
            weather ClearNoon
            for f in "$inc"/*.xosc; do
                run "$(basename "$f" .xosc)" "$f"
            done
            run town01_tour src/swb_examples/scenarios/demo/town01_tour.xosc global_timeout:=1200
            weather Default
            ;;
        bench)
            stop_stack $((web + 2))
            run bench_town01_npc_20 scenarios/bench/town01_npc_20.xosc
            ;;
        *) say "unknown group $g"; exit 2 ;;
    esac
done
stop_stack $((web + 2))
stop_stack $((web + 4))
say "done"
