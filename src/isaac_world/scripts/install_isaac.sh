#!/bin/bash
# Isaac Sim 6.1 pip install (roadmap 036): Python 3.12 from uv, everything under ISAAC_ROOT.
# Kit's caches default to ~/.cache/ov, ~/.local/share/ov and ~/.nvidia-omniverse; on this host
# they are symlinks into ISAAC_ROOT so the shader cache stays off /home. Then: pip install pyzmq.
set -eu
I=${ISAAC_ROOT:-/mnt/disk1/isaac-sim}
export UV_CACHE_DIR=$I/uv-cache
[ -d $I/venv ] || uv venv --python 3.12 $I/venv
source $I/venv/bin/activate
uv pip install --upgrade pip
uv pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128
uv pip install "isaacsim[all,extscache]==6.1.0.0" --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match --prerelease=allow
uv pip install pyzmq  # the swb world protocol (037) and the lockstep spike
python -c "import importlib.metadata as m; print('isaacsim', m.version('isaacsim'))"
echo INSTALL-DONE
