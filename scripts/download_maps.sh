#!/usr/bin/env bash
# Obtain pre-converted CARLA maps for Autoware (lanelet2 + PCD).
# Source: TUMFTM carla-autoware-bridge map pack.
#
# The upstream LRZ Sync+Share link is DEAD -- it returns HTTP 404 with an HTML error
# page, so the download path below cannot work until someone supplies a live URL.
# Set CSB_MAP_SOURCE to a directory holding the town folders (Town01, Town02, ...) and
# they are linked or copied from there instead.
set -e

# Optional local source, e.g. a NAS mount:
#   CSB_MAP_SOURCE=~/nas/autoveh/dataset/CARLA-map-from-TUM just download-maps
MAP_SOURCE="${CSB_MAP_SOURCE:-}"

# Dead as of 2026-07-28. Kept so the intended origin stays documented.
URL="https://syncandshare.lrz.de/dl/fiBgYSNkmsmRB28meoX3gZ/.dir"
SHA256="6e582242198ae50aa1d1fd410b53609c8b7ecfc6b9696294c498646783ed4838"

DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
ZIP="$DIR/carla-autoware-bridge.zip"
OUT="$DIR/carla-autoware-bridge"

# Skip if already present
if [ -d "$OUT" ] && [ "$(ls -A "$OUT" 2>/dev/null)" ]; then
    echo "Maps already available at $OUT"
    ls "$OUT"
    exit 0
fi

mkdir -p "$DIR"

# --- Local source ---------------------------------------------------------------
if [ -n "$MAP_SOURCE" ]; then
    if [ ! -d "$MAP_SOURCE" ]; then
        echo "ERROR: CSB_MAP_SOURCE='$MAP_SOURCE' is not a directory." >&2
        exit 1
    fi

    echo "Linking maps from $MAP_SOURCE"
    mkdir -p "$OUT"
    found=0
    for town in "$MAP_SOURCE"/*/; do
        name="$(basename "$town")"
        if [ -f "$town/lanelet2_map.osm" ]; then
            # Per-file symlinks rather than a directory link, so the projector
            # yaml can be corrected without touching the source pack.
            mkdir -p "$OUT/$name"
            for f in "$town"/*; do
                ln -sfn "$(cd "$(dirname "$f")" && pwd)/$(basename "$f")" "$OUT/$name/$(basename "$f")"
            done
            # The NEWSLab pack declares LocalCartesianUTM, which SSv2's
            # lanelet_loader rejects (TransverseMercator/MGRS only) and which
            # does not match CARLA's +proj=tmerc +lat_0=0 +lon_0=0 geolocation
            # that acb_bridge's GNSS path assumes. Same origin, so the numbers
            # are unchanged; only the declared projector differs.
            if grep -q 'LocalCartesianUTM' "$town/map_projector_info.yaml" 2>/dev/null; then
                rm -f "$OUT/$name/map_projector_info.yaml"
                cat > "$OUT/$name/map_projector_info.yaml" <<'YAML'
# Rewritten by download_maps.sh: the source pack declares LocalCartesianUTM,
# which SSv2 rejects and the CARLA GNSS pipeline does not use. CARLA exports
# geolocation against +proj=tmerc +lat_0=0 +lon_0=0.
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
YAML
                echo "  $name (projector rewritten to TransverseMercator)"
            else
                echo "  $name"
            fi
            found=$((found + 1))
        else
            echo "  skipping $name (no lanelet2_map.osm)"
        fi
    done

    if [ "$found" -eq 0 ]; then
        echo "ERROR: no town directories with lanelet2_map.osm under $MAP_SOURCE" >&2
        exit 1
    fi
    echo "Linked $found map(s) into $OUT"
    exit 0
fi

# --- Download -------------------------------------------------------------------
echo "Downloading CARLA maps for Autoware..."
echo
echo "NOTE: the upstream URL is known to be dead (HTTP 404) as of 2026-07-28."
echo "      If this fails, point CSB_MAP_SOURCE at a directory holding the town"
echo "      folders, e.g.:"
echo
echo "        CSB_MAP_SOURCE=/path/to/CARLA-map-from-TUM $0"
echo

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
