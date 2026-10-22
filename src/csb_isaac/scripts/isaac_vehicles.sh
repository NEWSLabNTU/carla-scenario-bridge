#!/usr/bin/env bash
# CARLA's vehicles as Isaac assets (roadmap 043, "Vehicle model"): every blueprint in CARLA
# 0.9.16's VehicleFactory exported to USD with SimReady and wrapped as a render-only asset,
# with the catalogue csb_isaac.vehicle_assets reads (vehicles.json: each car's CARLA box,
# wheels, wheel base).
#
#   isaac_vehicles.sh [only:=<id>,<id>...] [export:=true] [props:=true]
#
#   only:=<ids>     these blueprint ids only (vehicle.tesla.model3,...); default all
#   export:=true    export again even if a vehicle's USD exists
#   props:=true     CARLA's props instead (roadmap 044): every static.prop.* of the editor's
#                   Content/Carla/Config/Default.Package.json, into $CSB_VEHICLE_USD/props/
#                   with its props.json (prop_usd_assets.py); only:= takes prop ids
#   walkers:=true   CARLA's walkers instead (roadmap 044 C): every BP_Walker* blueprint with
#                   a walk and an idle cycle, into $CSB_VEHICLE_USD/walkers/ with its
#                   walkers.json (walker_usd_assets.py); only:= takes ids (bp_walker_female1_v1)
#
# Host paths from the environment:
#   CARLA_SRC        CARLA 0.9.16 editor build with SimReady (default /mnt/disk1/carla-src),
#                    with an env.sh exporting UE4_ROOT and CARLA_ROOT (roadmap 043, "A new host")
#   CSB_VEHICLE_USD  the output, one directory per blueprint id (default
#                    /mnt/disk1/carla-usd/vehicles); logs in its logs/
#   ISAAC_ROOT       Isaac Sim's install, for pxr (isaac_env.sh; default /mnt/disk1/isaac-sim)
#
# Three steps, each skipped when its output is there:
#   1. levels: the UE editor (commandlet, ue_vehicle_levels.py) saves one level per blueprint,
#      the car alone at the origin, under /Game/CSBVehicleExport/, and writes ue_meta.json;
#   2. export: SimReadyExport of those levels into <id>/ (one commandlet for all of them,
#      COLLISION=false: the entity's declared box is the collider);
#   3. assets: vehicle_usd_assets.py writes <id>/<id>.usda, <id>/wheels.usda, vehicles.json and
#      vehicles.md.
# The vehicles' physics (CARLA's get_physics_control) is a separate step against a running
# CARLA: scripts/dump_vehicle_physics.py --root "$CSB_VEHICLE_USD" (writes physics.json).
#
# The editor runs one at a time (isaac_town.sh's exports included): 41 vehicles took 11 min to
# prepare and 7 min to export on the first host, 2.6 GB in all; 5 and 6.5 min on
# newslab-server139 (the preparing editor hangs at exit there and is killed after 2 min).
set -euo pipefail

usage() { sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
here=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)    # src/csb_isaac
only=""; force=false; props=false; walkers=false
for a in "$@"; do
    case "$a" in
        only:=*) only=${a#only:=} ;;
        export:=*) force=${a#export:=} ;;
        props:=*) props=${a#props:=} ;;
        walkers:=*) walkers=${a#walkers:=} ;;
        -h|--help) usage ;;
        *) echo "isaac_vehicles: unknown argument '$a' (only:=, export:=, props:=, walkers:=)" >&2; exit 2 ;;
    esac
done
CARLA_SRC=${CARLA_SRC:-/mnt/disk1/carla-src}
root=${CSB_VEHICLE_USD:-/mnt/disk1/carla-usd/vehicles}
kind=vehicles
if [ "$props" = true ]; then root="$root/props"; kind=props; fi
if [ "$walkers" = true ]; then root="$root/walkers"; kind=walkers; fi
meta="$root/ue_meta.json"
mkdir -p "$root/logs"

# Run a UE editor commandlet in its own process group; $1 log, $2 a line its log must have,
# the rest the commandlet's arguments. Same handling as isaac_town.sh: it exits 1 on
# SimReady's material warnings even when done, and sometimes hangs at exit. An ensure in an
# editor build starts CrashReportClient, which uploads to Epic unless the editor's privacy
# settings say no (BaseEditorSettings.ini says yes): `-ini:` overrides them for this run.
commandlet() {
    local log=$1 done_line=$2; shift 2
    [ -f "$CARLA_SRC/env.sh" ] || { echo "isaac_vehicles: no $CARLA_SRC/env.sh (CARLA_SRC)" >&2; exit 1; }
    for p in $(pgrep -x UE4Editor-Cmd || true); do
        echo "isaac_vehicles: a UE editor commandlet already runs (pid $p): one at a time" >&2
        exit 1
    done
    setsid bash -c '
        source "$1/env.sh"; shift
        export VK_ICD_FILENAMES=${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}
        exec nice -n 15 "$UE4_ROOT/Engine/Binaries/Linux/UE4Editor-Cmd" \
            "$CARLA_ROOT/Unreal/CarlaUE4/CarlaUE4.uproject" "$@" \
            -AllowCommandletRendering -log -Unattended -NoSound -RenderOffScreen -vulkan \
            "-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:bSendUnattendedBugReports=False,[/Script/UnrealEd.AnalyticsPrivacySettings]:bSendUsageData=False"' \
        _ "$CARLA_SRC" "$@" > "$log" 2>&1 < /dev/null &
    local pid=$!
    trap "kill -TERM -$pid 2>/dev/null || true" INT TERM
    until ! kill -0 $pid 2>/dev/null || grep -q "Execution of commandlet took" "$log"; do sleep 5; done
    for _ in $(seq 24); do kill -0 $pid 2>/dev/null || break; sleep 5; done
    if kill -0 $pid 2>/dev/null; then
        echo "isaac_vehicles: the editor hangs at exit: killing its group" | tee -a "$log"
        kill -TERM -$pid 2>/dev/null || true
        for _ in $(seq 12); do kill -0 $pid 2>/dev/null || break; sleep 5; done
        kill -KILL -$pid 2>/dev/null || true
    fi
    wait $pid 2>/dev/null || true
    trap - INT TERM
    grep -q "$done_line" "$log" || { echo "isaac_vehicles: failed; see $log" >&2; exit 1; }
}

# The blueprint ids ue_meta.json has a level for, restricted to only:=.
ids() {
    python3 - "$meta" "$only" <<'EOF'
import json, sys
only = [s for s in sys.argv[2].split(",") if s]
for m in json.load(open(sys.argv[1])):
    if m.get("level") and (not only or m["id"] in only):
        print(m["id"])
EOF
}

# 1. levels
need_levels=false
if [ "$force" = true ] || [ ! -f "$meta" ]; then
    need_levels=true
elif [ -n "$only" ]; then
    for id in ${only//,/ }; do
        python3 -c 'import json,sys; sys.exit(0 if any(m["id"]==sys.argv[2] and m.get("level") for m in json.load(open(sys.argv[1]))) else 1)' "$meta" "$id" || need_levels=true
    done
fi
if [ "$need_levels" = true ]; then
    echo "isaac_vehicles: preparing levels (log $root/logs/levels.log)"
    # A level made by an earlier run is not replaced by the editor (it exists but is not
    # accessible to delete_asset, so the new one is never made and the old one exported):
    # remove the ones being made again first.
    content=$(bash -c 'source "$1/env.sh" >/dev/null 2>&1; echo "$CARLA_ROOT/Unreal/CarlaUE4/Content"' _ "$CARLA_SRC")
    dir="$content/CSBVehicleExport"; [ "$kind" = props ] && dir="$content/CSBPropExport"
    [ "$kind" = walkers ] && dir="$content/CSBWalkerExport"
    if [ -d "$dir" ]; then
        if [ -n "$only" ]; then
            for id in ${only//,/ }; do
                rm -f "$dir/${id//./_}.umap" "$dir/${id//./_}_BuiltData.uasset" \
                    "$dir/${id}_walk.umap" "$dir/${id}_walk_BuiltData.uasset" \
                    "$dir/${id}_idle.umap" "$dir/${id}_idle_BuiltData.uasset"
            done
        elif [ "$force" = true ]; then
            rm -f "$dir"/*.umap "$dir"/*_BuiltData.uasset
        fi
    fi
    stamp="$root/logs/.levels-start"; touch "$stamp"
    CSB_VEH_META="$meta" CSB_VEH_ONLY="$only" CSB_VEH_KIND="$kind" commandlet "$root/logs/levels.log" \
        "Execution of commandlet took" -run=pythonscript "-script=$here/scripts/ue_vehicle_levels.py"
    # The script's own log lines do not reach the commandlet's output: judge by its file.
    [ "$meta" -nt "$stamp" ] || { echo "isaac_vehicles: no new $meta; see $root/logs/levels.log" >&2; exit 1; }
    rm -f "$stamp"
fi

# 2. export
# The levels an id is exported from: a vehicle's or prop's one, a walker's walk and idle.
units() {
    if [ "$kind" = walkers ]; then echo "${1}_walk ${1}_idle"; else echo "${1//./_}"; fi
}
todo=(); maps=()
for id in $(ids); do
    first=$(units "$id"); first=${first%% *}
    if [ "$force" = true ] || { [ ! -f "$root/$id/$first.usd" ] && [ ! -f "$root/$id/$first/$first.usd" ]; }; then
        todo+=("$id"); maps+=($(units "$id"))
    fi
done
if [ ${#todo[@]} -gt 0 ]; then
    raw="$root/raw"
    rm -rf "$raw"
    echo "isaac_vehicles: exporting ${#todo[@]} $kind (log $root/logs/export.log)"
    started=$(date +%s)
    commandlet "$root/logs/export.log" "Execution of commandlet took" \
        -run=SimReadyExport "-maps=$(IFS=+; echo "${maps[*]}")" "-path=$raw" -MDL=true -COLLISION=false
    # SimReady exports several maps into one directory each under -path, a single map straight
    # into -path: give that one its directory too.
    s=${maps[0]}
    if [ ${#maps[@]} -eq 1 ] && [ -f "$raw/$s.usd" ]; then
        mkdir "$raw/.one" && mv "$raw"/* "$raw/.one/" && mv "$raw/.one" "$raw/$s"
    fi
    failed=0
    for id in "${todo[@]}"; do
        ok=true
        for s in $(units "$id"); do
            grep -q "$s exported to USD!" "$root/logs/export.log" && [ -f "$raw/$s/$s.usd" ] || ok=false
        done
        if [ $ok = false ]; then echo "isaac_vehicles: $id was not exported" >&2; failed=1; continue; fi
        rm -rf "${root:?}/$id"
        if [ "$kind" = walkers ]; then
            mkdir -p "$root/$id"
            for s in $(units "$id"); do mv "$raw/$s" "$root/$id/$s"; done
        else
            mv "$raw/${id//./_}" "$root/$id"
        fi
    done
    rmdir "$raw" 2>/dev/null || true
    echo "isaac_vehicles: exported in $(( $(date +%s) - started )) s"
    [ $failed = 0 ] || { echo "isaac_vehicles: see $root/logs/export.log" >&2; exit 1; }
fi

# 3. assets
export ISAAC_ROOT=${ISAAC_ROOT:-/mnt/disk1/isaac-sim}
set +u
source "$here/scripts/isaac_env.sh"
set -u
# The export's props are crate 0.7.0; pxr warns once per file without this.
export PXR_USDC_EMIT_DEPRECATION_WARNINGS=0
if [ "$kind" = props ]; then
    python -I "$here/scripts/prop_usd_assets.py" "$root"
    exit 0
fi
if [ "$kind" = walkers ]; then
    python -I "$here/scripts/walker_usd_assets.py" "$root"
    exit 0
fi
python -I "$here/scripts/vehicle_usd_assets.py" "$root"
[ -f "$root/physics.json" ] || echo "isaac_vehicles: no physics.json yet: with CARLA 0.9.16 up," \
    "run $here/scripts/dump_vehicle_physics.py --root $root"
