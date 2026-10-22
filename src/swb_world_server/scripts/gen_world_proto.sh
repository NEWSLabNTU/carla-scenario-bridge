#!/bin/bash
# Regenerate swb_world_server/swb_world.desc from proto/swb_world.proto (roadmaps 037, 046).
# The Python side reads this descriptor set instead of generated _pb2 code (see
# swb_world_server/protocol.py).
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
protoc --proto_path="$here/../../proto" --descriptor_set_out="$here/swb_world_server/swb_world.desc" \
    "$here/../../proto/swb_world.proto"
echo "wrote $here/swb_world_server/swb_world.desc"
