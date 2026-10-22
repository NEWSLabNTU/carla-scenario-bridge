#!/usr/bin/env bash
# Autoware's ROS message packages for Isaac's Python 3.12 (roadmap 038), into $ISAAC_ROOT/msgs:
# aib's script (autoware_isaac_bridge, roadmap 045), which builds them for both repositories.
#
#     src/isaac_world/scripts/build_msgs.sh [package...]
exec "$(dirname "${BASH_SOURCE[0]}")/../../autoware_isaac_bridge/src/aib/scripts/build_msgs.sh" "$@"
