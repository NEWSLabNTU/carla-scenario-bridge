#!/bin/bash
# Phase 036: what a ROS 2 client on each domain sees from the domains spike.
# Usage: check_domains.sh <domain>... ; RMW_IMPLEMENTATION / CYCLONEDDS_URI as for the stack.
set -u
set +u; source /opt/ros/humble/setup.bash; set -u
for d in "$@"; do
  echo "== domain $d (${RMW_IMPLEMENTATION:-default rmw})"
  ROS_DOMAIN_ID=$d timeout 20 ros2 topic list --no-daemon 2>&1 | grep -E "^/(clock|tf)" | sort | tr '\n' ' '; echo
  frames=$(ROS_DOMAIN_ID=$d timeout 15 ros2 topic echo --no-daemon /tf 2>/dev/null | head -400            | grep -oE "child_frame_id: [a-z_0-9]+" | sort -u | sed 's/child_frame_id: //' | tr '
' ' ')
  echo "   /tf child frames: ${frames:-none}"
  clock=$(ROS_DOMAIN_ID=$d timeout 15 ros2 topic echo --no-daemon /clock --once 2>/dev/null | grep -A1 "sec:" | head -1)
  echo "   /clock: ${clock:-no messages}"
done
