# Isaac Sim 6.1 environment (roadmap 036), for swb's Isaac side: aib's (autoware_isaac_bridge,
# the submodule at src/autoware_isaac_bridge; roadmap 045) -- the pip install's venv, Isaac's
# bundled ROS 2 Humble libraries, Autoware's messages rebuilt for it ($ISAAC_ROOT/msgs, shared
# by both repositories) -- plus aib's Python package on PYTHONPATH, which the world server
# imports for its ego, and the world server shell (`swb_world_server`). Do not source the system Humble into it. RMW_IMPLEMENTATION and
# CYCLONEDDS_URI pass through, so the stack's DDS settings apply.
_swb_aib="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../autoware_isaac_bridge/src/aib" && pwd)" || {
    echo "isaac_env.sh: no aib at src/autoware_isaac_bridge (git submodule update --init)" >&2
    return 1 2>/dev/null || exit 1
}
# shellcheck source=/dev/null
source "$_swb_aib/scripts/isaac_env.sh"
# The world protocol's server shell (roadmap 046), beside this package.
_swb_shell="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../swb_world_protocol/swb_world_server" && pwd)"
export PYTHONPATH="$_swb_shell:$_swb_aib:$PYTHONPATH"
unset _swb_aib _swb_shell
