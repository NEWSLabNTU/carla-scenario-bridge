#!/usr/bin/env python3
"""Give every lanelet in a Lanelet2 map a `subtype` tag.

The map pack's converter leaves some lanelets with `type=lanelet` and nothing else. In
Town01 that is 65 of 300: the 0.3 m kerb strips between road and sidewalk (OpenDRIVE lane
type `shoulder`) and 17 4 m strips that border only walkways. lanelet2 tolerates the
missing tag, but Autoware's map_based_prediction does not:
`PredictorVru::getPredictedObjectAsCrosswalkUser` calls
`lanelet.attribute(AttributeName::Subtype)` on every lanelet near a pedestrian, which
throws `lanelet::NoSuchAttributeError` ("Could not find 1" -- 1 is the enum value of
`Subtype`) as soon as one of those strips is in range. The node aborts, predicted objects
stop, and every later scenario waits in PLANNING for a route that is never planned.

The repair adds `subtype=unknown`. Neither lanelet2's traffic rules nor Autoware know that
value, so routing and every subtype query treat these lanelets exactly as they treated the
missing tag -- not passable, not road, not crosswalk -- except that asking no longer
throws.

Edits are text insertions into the affected `<relation>` blocks, so the rest of the file
stays byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with
repair_lanelet_traffic_lights.py, whichever runs first).

    scripts/repair_lanelet_subtypes.py data/carla-autoware-bridge
"""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SUBTYPE = "unknown"


def targets(path):
    """Ids of the lanelet relations that have no subtype tag."""
    root = ET.parse(path).getroot()
    ids = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "lanelet" and "subtype" not in tags:
            ids.append(rel.get("id"))
    return ids


def add_subtype(text, rel_id):
    """Insert the subtype tag before the type tag of one <relation id=rel_id> block."""
    block = re.compile(r'(<relation id="%s"[^>]*>)(.*?)(</relation>)' % re.escape(rel_id), re.S)
    m = block.search(text)
    if not m:
        return text, False
    body = m.group(2)
    tag = re.search(r'([ \t]*)<tag k="type" v="lanelet"\s*/>', body)
    if not tag:
        return text, False
    insert = '%s<tag k="subtype" v="%s"/>\n' % (tag.group(1), SUBTYPE)
    body = body[:tag.start()] + insert + body[tag.start():]
    return text[:m.start(2)] + body + text[m.end(2):], True


def repair(path):
    ids = targets(path)
    if not ids:
        return 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    fixed = 0
    for rel_id in ids:
        text, ok = add_subtype(text, rel_id)
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
        print(f"  {m.parent.name}: {n} lanelet(s) given subtype={SUBTYPE}" if n
              else f"  {m.parent.name}: every lanelet has a subtype")
    print(f"{total} lanelet subtype(s) repaired")


if __name__ == "__main__":
    main()
