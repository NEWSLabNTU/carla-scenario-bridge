#!/bin/bash
# Regenerate csb_isaac/csb_world.desc from proto/csb_world.proto (roadmap 037). The Python
# side reads this descriptor set instead of generated _pb2 code (see csb_isaac/protocol.py).
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
protoc --proto_path="$here/../../proto" --descriptor_set_out="$here/csb_isaac/csb_world.desc" \
    "$here/../../proto/csb_world.proto"
echo "wrote $here/csb_isaac/csb_world.desc"
