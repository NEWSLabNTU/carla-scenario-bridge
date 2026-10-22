#!/bin/bash
# Regenerate csb_world_server/csb_world.desc from proto/csb_world.proto (roadmaps 037, 046).
# The Python side reads this descriptor set instead of generated _pb2 code (see
# csb_world_server/protocol.py).
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
protoc --proto_path="$here/../../proto" --descriptor_set_out="$here/csb_world_server/csb_world.desc" \
    "$here/../../proto/csb_world.proto"
echo "wrote $here/csb_world_server/csb_world.desc"
