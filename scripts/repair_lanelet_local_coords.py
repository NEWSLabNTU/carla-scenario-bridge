#!/usr/bin/env python3
"""Give every node in a Lanelet2 map `local_x` / `local_y` tags.

Autoware reads a map's geometry from these tags, not from lat/lon: its OSM parser
(`autoware_lanelet2_extension::AutowareOsmParser`) overwrites each point's x and y with
`local_x` / `local_y` when they are present, and only projects lat/lon when they are not.
The map pack tags almost every node, and those local coordinates are CARLA's world frame
(Y flipped). It leaves a few without them -- 50 in Town10, all on stop lines and traffic
signs -- so those, and only those, are placed from lat/lon.

The pack's lat/lon are not Autoware's projection. They are CARLA's own geo transform, a
spherical Mercator on R = 6 378 137 m about (0, 0):

    x = R * lon,  y = R * ln(tan(pi/4 + lat/2))          (radians)

which reproduces every tagged node's local_x/local_y to 0.1 mm. Autoware projects with
TransverseMercatorExact on the WGS84 ellipsoid (k0 0.9996), which reads the same lat/lon
0.709 % short in y and 0.04 % in x: up to 2.4 m in Town01. So a stop line can sit metres
from the lane it crosses, while the lanelets around it are exact.

The repair adds the local tags to the untagged nodes from the formula above, which is the
transform their lat/lon came from, so they land where the rest of the map already is.
Edits are text insertions into the affected `<node>` blocks; the rest of the file stays
byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with the other
repair_lanelet_*.py scripts, whichever runs first).

    scripts/repair_lanelet_local_coords.py data/carla-autoware-bridge
"""
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

R = 6378137.0


def carla_local(lat_deg, lon_deg):
    """Local (ROS-frame) metres for a lat/lon made by CARLA's geo transform about (0, 0)."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    return R * lon, R * math.log(math.tan(math.pi / 4 + lat / 2))


def targets(path):
    """(id, local_x, local_y) for every node with no local_x tag."""
    out = []
    for _, el in ET.iterparse(path):
        if el.tag == "node":
            if not any(t.get("k") == "local_x" for t in el.findall("tag")):
                x, y = carla_local(float(el.get("lat")), float(el.get("lon")))
                out.append((el.get("id"), x, y))
            el.clear()
    return out


def add_local(text, node_id, x, y):
    """Insert local_x/local_y as the first tags of one <node id=node_id> element."""
    tags = '<tag k="local_x" v="%.4f"/>\n%s<tag k="local_y" v="%.4f"/>\n'
    block = re.compile(r'(<node id="%s"[^>]*?)(/?)>' % re.escape(node_id))
    m = block.search(text)
    if not m:
        return text, False
    if m.group(2):  # self-closing: <node .../> -> <node ...> tags </node>
        line_start = text.rfind("\n", 0, m.start()) + 1
        indent = text[line_start:m.start()]
        inner = indent + "  "
        body = ">\n" + inner + tags % (x, inner, y) + indent + "</node>"
        return text[:m.end(1)] + body + text[m.end():], True
    nxt = re.compile(r'\n([ \t]*)<').search(text, m.end())
    if not nxt:
        return text, False
    indent = nxt.group(1)
    at = nxt.start() + 1
    return text[:at] + indent + tags % (x, indent, y) + text[at:], True


def repair(path):
    nodes = targets(path)
    if not nodes:
        return 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    fixed = 0
    for node_id, x, y in nodes:
        text, ok = add_local(text, node_id, x, y)
        fixed += ok
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return fixed


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        n = repair(m)
        total += n
        print(f"  {m.parent.name}: {n} node(s) given local_x/local_y" if n
              else f"  {m.parent.name}: every node has local_x/local_y")
    print(f"{total} node local coordinate(s) repaired")


if __name__ == "__main__":
    main()
