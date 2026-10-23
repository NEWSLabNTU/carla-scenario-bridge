#!/usr/bin/env bash
# Resolve every stack's launch without starting it (roadmap 051, K0/K1): each recipe runs as
# usual with scripts/launch_dump/play_launch first in PATH, which turns its `play_launch
# launch` into `play_launch dump`. Writes <out>/<name>.yaml (play_launch's SystemModel).
#
#   scripts/dump_launches.sh <out_dir>
#
# Compare two sets with scripts/compare_launch_dumps.py.
set -u
cd "$(dirname "$0")/.."
out=$(realpath -m "${1:?out_dir}"); mkdir -p "$out"
export PATH="$PWD/scripts/launch_dump:$PATH"
dump() {  # name, then the command
    local name=$1; shift
    if LAUNCH_DUMP_OUT="$out/$name.yaml" "$@" > "$out/$name.log" 2>&1 && [ -s "$out/$name.yaml" ]; then
        echo "$name: ok"
    else
        echo "$name: FAILED (see $out/$name.log)"
    fi
}
dump ego-av just ego-av
dump bg-av just bg-av
dump ego-isaac just ego-isaac
dump run-carla just run
dump run-isaac just run sim:=isaac
dump isaac-world just isaac-world
dump acb-sim bash -c 'cd src/autoware_carla_bridge && just sim'
dump acb-autoware bash -c 'cd src/autoware_carla_bridge && just autoware'
dump aib-autoware bash -c 'cd src/autoware_isaac_bridge && ACB_SETUP=$PWD/../autoware_carla_bridge/install/setup.bash just autoware map_path:=$PWD/../../data/carla-autoware-bridge/Town01'
