#!/usr/bin/env bash
# A CARLA town as an Isaac world (roadmap 043): export it with SimReady when its USD is
# missing, write its collider layer (csb_isaac.physics_layer) and its world file
# (csb_isaac.usd_world) for the town's Autoware map.
#
#   isaac_town.sh <Town> [map_path:=<dir>] [export:=true]
#
#   <Town>          CARLA's level name: Town01, Town02, Town03, Town05, Town10HD, ...
#   map_path:=<dir> the Autoware map the world file goes with (default
#                   data/carla-autoware-bridge/<Town>, or <Town> without "HD": Town10)
#   export:=true    export again even if <Town>.usd exists
#
# Host paths from the environment:
#   CARLA_SRC   CARLA 0.9.16 editor build with SimReady (default /mnt/disk1/carla-src), with
#               an env.sh exporting UE4_ROOT and CARLA_ROOT; building one on a new host:
#               roadmap 043, "A new host"
#   CARLA_USD   where towns are exported, one directory each (default /mnt/disk1/carla-usd)
#   ISAAC_ROOT  Isaac Sim's install (isaac_env.sh; default /mnt/disk1/isaac-sim)
#
# The export runs the UE editor as a commandlet (15-60 min a town, most of it shader
# compilation; one at a time), logs to $CARLA_USD/<Town>/export.log, and counts as done
# when the log says "<Town> exported to USD!": the commandlet exits 1 on SimReady's
# material warnings even then, and sometimes hangs at exit, so it is killed if it does.
set -euo pipefail

usage() { sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage
town=$1; shift
here=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)    # src/csb_isaac
project=$(cd "$here/../.." && pwd)
map_path=""; force=false
for a in "$@"; do
    case "$a" in
        map_path:=*) map_path=${a#map_path:=} ;;
        export:=*) force=${a#export:=} ;;
        *) echo "isaac_town: unknown argument '$a' (map_path:=, export:=)" >&2; exit 2 ;;
    esac
done
CARLA_SRC=${CARLA_SRC:-/mnt/disk1/carla-src}
CARLA_USD=${CARLA_USD:-/mnt/disk1/carla-usd}
if [ -z "$map_path" ]; then
    for m in "$town" "${town%HD}"; do
        if [ -d "$project/data/carla-autoware-bridge/$m" ]; then
            map_path="$project/data/carla-autoware-bridge/$m"; break
        fi
    done
    [ -n "$map_path" ] || { echo "isaac_town: no Autoware map for $town under data/carla-autoware-bridge (map_path:=)" >&2; exit 1; }
fi
out="$CARLA_USD/$town"
usd="$out/$town.usd"

if [ "$force" = true ] || [ ! -f "$usd" ]; then
    [ -f "$CARLA_SRC/env.sh" ] || { echo "isaac_town: no $CARLA_SRC/env.sh (CARLA_SRC)" >&2; exit 1; }
    for p in $(pgrep -x UE4Editor-Cmd || true); do
        echo "isaac_town: a UE editor commandlet already runs (pid $p): one export at a time" >&2
        exit 1
    done
    mkdir -p "$out"
    log="$out/export.log"
    echo "isaac_town: exporting $town to $out (log $log)"
    started=$(date +%s)
    # Its own process group, so a hang at exit is killed with its shader workers.
    setsid bash -c '
        source "$1/env.sh"
        # On a host with another GPU the editor enumerates it too, and an AMD iGPU trips
        # an ensure that starts CrashReportClient (which then uploads a report to Epic).
        export VK_ICD_FILENAMES=${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}
        exec nice -n 15 "$UE4_ROOT/Engine/Binaries/Linux/UE4Editor-Cmd" \
            "$CARLA_ROOT/Unreal/CarlaUE4/CarlaUE4.uproject" -run=SimReadyExport \
            -AllowCommandletRendering "-map=$2.umap" "-path=$3" -MDL=true -COLLISION=true \
            -log -Unattended -NoSound -RenderOffScreen -vulkan' _ "$CARLA_SRC" "$town" "$out" \
        > "$log" 2>&1 < /dev/null &
    pid=$!
    trap 'kill -TERM -$pid 2>/dev/null || true' INT TERM
    # The commandlet's own end, not "LogExit: Exiting": CrashReportClient logs that line to
    # the same output when it exits, mid-export.
    until ! kill -0 $pid 2>/dev/null || grep -q "Execution of commandlet took" "$log"; do sleep 5; done
    for _ in $(seq 24); do kill -0 $pid 2>/dev/null || break; sleep 5; done
    if kill -0 $pid 2>/dev/null; then
        echo "isaac_town: the editor hangs at exit: killing its group" | tee -a "$log"
        kill -TERM -$pid 2>/dev/null || true
        # SIGTERM is not always enough: an editor has been seen to log "Exiting
        # abnormally (error code: 143)" and then keep running on eight cores for minutes.
        for _ in $(seq 12); do kill -0 $pid 2>/dev/null || break; sleep 5; done
        kill -KILL -$pid 2>/dev/null || true
    fi
    wait $pid 2>/dev/null || true
    trap - INT TERM
    if ! grep -q "$town exported to USD!" "$log" || [ ! -f "$usd" ]; then
        echo "isaac_town: export of $town failed; see $log" >&2; exit 1
    fi
    echo "isaac_town: exported $town in $(( $(date +%s) - started )) s"
fi

export ISAAC_ROOT=${ISAAC_ROOT:-/mnt/disk1/isaac-sim}
set +u
source "$here/scripts/isaac_env.sh"
set -u
cd "$here"
# The export's props are crate 0.7.0; pxr warns once per file without this.
export PXR_USDC_EMIT_DEPRECATION_WARNINGS=0
python -m csb_isaac.physics_layer "$usd"
python -m csb_isaac.usd_world "$map_path" "$out/${town}_physics.usda"
