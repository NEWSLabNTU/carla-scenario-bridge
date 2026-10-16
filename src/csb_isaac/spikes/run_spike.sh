#!/bin/bash
# Phase 036: run a spike under Isaac's Python with the bundled ROS 2 libraries.
# Usage: run_spike.sh <rmw: fastrtps|cyclonedds> <log> <spike.py> [args...]
set -u
rmw=$1; log=$2; shift 2
export RMW_IMPLEMENTATION=rmw_${rmw}_cpp
source $(dirname $0)/../scripts/isaac_env.sh
cd ${ISAAC_ROOT:-/mnt/disk1/isaac-sim}
exec python "$@" > "$log" 2>&1
