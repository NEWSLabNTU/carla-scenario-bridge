#!/usr/bin/env bash
# Download pre-converted CARLA maps for Autoware (lanelet2 + PCD).
# Source: TUMFTM carla-autoware-bridge map pack.
set -e

URL="https://syncandshare.lrz.de/dl/fiBgYSNkmsmRB28meoX3gZ/.dir"
SHA256="6e582242198ae50aa1d1fd410b53609c8b7ecfc6b9696294c498646783ed4838"
DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
ZIP="$DIR/carla-autoware-bridge.zip"
OUT="$DIR/carla-autoware-bridge"

# Skip if already extracted
if [ -d "$OUT" ] && [ "$(ls -A "$OUT" 2>/dev/null)" ]; then
    echo "Maps already downloaded at $OUT"
    ls "$OUT"
    exit 0
fi

mkdir -p "$DIR"

echo "Downloading CARLA maps for Autoware..."

# Download with resume (aria2c preferred, wget fallback)
if command -v aria2c &>/dev/null; then
    aria2c -c -x4 -s4 --file-allocation=none --auto-file-renaming=false \
        -d "$DIR" -o "$(basename "$ZIP")" "$URL"
else
    wget -c -O "$ZIP" "$URL"
fi

# Verify checksum
echo "$SHA256  $ZIP" | sha256sum -c --quiet
echo "Checksum verified."

# Extract
unzip -q -o "$ZIP" -d "$OUT"
echo "Maps extracted to $OUT"
ls "$OUT"

# Clean up zip
rm -f "$ZIP"
echo "Done."
