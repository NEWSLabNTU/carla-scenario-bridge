#!/usr/bin/env python3
"""Tag every junction lanelet in a Lanelet2 map with `turn_direction`.

Autoware launches its `intersection` module (behavior_velocity) on a lanelet of the path
only if that lanelet carries `turn_direction` = `straight`, `left` or `right`
(`IntersectionModuleManager::launchNewModules`: anything else reads as "else" and is
skipped), and its `blind_spot` module only on `left` / `right`. The same tag drives the
turn signals (behavior_path's turn signal decider blinks 30 m before a tagged lanelet) and
groups a junction's lanelets (`getAssociativeIntersectionLanelets`: the following lanelets
of the parent's neighbours with the same direction). Nothing else is required to launch
them -- with no `right_of_way` element every conflicting lanelet is an attention lane, with
no stop line the module derives one from the first conflict, and `intersection_area`
polygons are optional (used by the avoidance modules and `use_intersection_area`, off by
default). Autoware's map requirements (vm-03-01 / vm-03-02) ask for the tag on *every*
lanelet in an intersection, driven or not.

The map pack's converter writes no `turn_direction` at all (Town01: 0 of 300 lanelets), so
on these maps both modules never run: the ego crosses every junction as if it were open
road and ignores crossing traffic (roadmap 033 step 3).

Which lanelets lie in a junction. The pack carries no trace of OpenDRIVE's junctions, so
the topology decides, read as lanelet2's routing graph reads it: lanelet B follows A when
A's bounds end on the points B's start on (shared point ids). An OpenDRIVE junction is
where one lane splits into several connectors (straight on, turn) and several connectors
join one lane, so a driveable lanelet (subtype road / highway) is a junction lanelet if

  - one of its predecessors has more than one successor (it leaves a fork), or
  - one of its successors has more than one predecessor (it joins a merge).

In Town01 and Town02 every junction lanelet is both; the second clause catches a turn from
a lane that only turns (Town03 / 05 / 10). Two kinds of lanelet are knowingly wrong:
a connector that neither leaves a fork nor joins a merge (a straight-only middle lane
through a multi-lane junction: 6 in Town03, 7 in Town05, none in the other towns) stays
untagged, and a lane added or dropped mid-road is tagged as if it were a junction (a fork
whose branches all run straight; 15 in Town03, none elsewhere). A connector the pack drew
with a stray first point (11 in Town01) has no predecessor, so lanelet2 cannot route
through it either; it stays untagged.

Which direction. The heading change from the lanelet's entry to its exit, on its
centreline, positive counter-clockwise (ROS frame): above +TURN_DEG is `left`, below
-TURN_DEG `right`, else `straight`. TURN_DEG = 45 is the bisector of straight (0) and a
right-angle turn (90): across the five towns the junction lanelets fall at 0 +- 20 deg and
90 +- 20 deg, with only a handful of Town03's oblique and roundabout connectors between 30
and 60. Positions come from lat/lon, which every node has, through the spherical Mercator
of repair_lanelet_local_coords.py (so they equal the pack's `local_x` / `local_y` where
those exist). That frame is 0.67 % long in y against CARLA's lanes (033 step 3), which
bends a heading by at most 0.2 deg: nothing against a 45 deg threshold. Checked on Town01:
all 72 tagged lanelets match a CARLA junction segment (`get_topology`) turning the same way.

The repair adds `turn_direction` to the junction lanelets that have none; a tag already
present is left alone. Edits are text insertions into the affected `<relation>` blocks,
so the rest of the file stays byte-identical. Idempotent; the first edit keeps a `.orig`
copy (shared with the other repair_lanelet_*.py scripts, whichever runs first).

    scripts/repair_lanelet_turn_direction.py data/carla-autoware-bridge
"""
import collections
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

R = 6378137.0
TURN_DEG = 45.0
DRIVEABLE = ("road", "highway")


def carla_local(lat_deg, lon_deg):
    """Local (ROS-frame) metres for a lat/lon made by CARLA's geo transform about (0, 0)."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    return R * lon, R * math.log(math.tan(math.pi / 4 + lat / 2))


def direction(change):
    """`left`, `right` or `straight` for a heading change in radians (CCW positive)."""
    if change > math.radians(TURN_DEG):
        return "left"
    if change < -math.radians(TURN_DEG):
        return "right"
    return "straight"


def _resample(pts, n):
    """n points evenly spaced by arc length along a polyline."""
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    out, j = [], 0
    for i in range(n):
        s = cum[-1] * i / (n - 1)
        while j < len(pts) - 2 and cum[j + 1] < s:
            j += 1
        seg = cum[j + 1] - cum[j]
        f = (s - cum[j]) / seg if seg else 0.0
        out.append((pts[j][0] + f * (pts[j + 1][0] - pts[j][0]),
                    pts[j][1] + f * (pts[j + 1][1] - pts[j][1])))
    return out


def heading_change(left, right):
    """Exit heading minus entry heading of the centreline between two bounds, in (-pi, pi]."""
    n = max(len(left), len(right), 2)
    c = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
         for a, b in zip(_resample(left, n), _resample(right, n))]
    h0 = math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0])
    h1 = math.atan2(c[-1][1] - c[-2][1], c[-1][0] - c[-2][0])
    return (h1 - h0 + math.pi) % (2 * math.pi) - math.pi


def targets(path):
    """(id, turn_direction) for every junction lanelet that has no turn_direction tag."""
    root = ET.parse(path).getroot()
    nodes = {n.get("id"): carla_local(float(n.get("lat")), float(n.get("lon")))
             for n in root.findall("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    lanes, tagged = {}, set()
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") not in DRIVEABLE:
            continue
        bounds = {m.get("role"): m.get("ref") for m in rel.findall("member")
                  if m.get("type") == "way"}
        left, right = ways.get(bounds.get("left")), ways.get(bounds.get("right"))
        if not left or not right or len(left) < 2 or len(right) < 2:
            continue
        # The right bound gives the driving direction; a left bound stored the other way
        # round is read reversed, as lanelet2 does.
        if math.dist(nodes[left[0]], nodes[right[0]]) > math.dist(nodes[left[-1]], nodes[right[0]]):
            left = left[::-1]
        lanes[rel.get("id")] = (left, right)
        if "turn_direction" in tags:
            tagged.add(rel.get("id"))

    starts, ends = collections.defaultdict(list), collections.defaultdict(list)
    for lid, (left, right) in lanes.items():
        starts[(left[0], right[0])].append(lid)
        ends[(left[-1], right[-1])].append(lid)
    succ = {lid: starts.get((l[-1], r[-1]), []) for lid, (l, r) in lanes.items()}
    pred = {lid: ends.get((l[0], r[0]), []) for lid, (l, r) in lanes.items()}

    out = []
    for lid, (left, right) in lanes.items():
        if lid in tagged:
            continue
        fork = any(len(succ[p]) > 1 for p in pred[lid])
        merge = any(len(pred[s]) > 1 for s in succ[lid])
        if fork or merge:
            change = heading_change([nodes[i] for i in left], [nodes[i] for i in right])
            out.append((lid, direction(change)))
    return out


def add_turn_direction(text, rel_id, value):
    """Insert the turn_direction tag before the type tag of one <relation id=rel_id> block."""
    block = re.compile(r'(<relation id="%s"[^>]*>)(.*?)(</relation>)' % re.escape(rel_id), re.S)
    m = block.search(text)
    if not m:
        return text, False
    body = m.group(2)
    tag = re.search(r'([ \t]*)<tag k="type" v="lanelet"\s*/>', body)
    if not tag:
        return text, False
    insert = '%s<tag k="turn_direction" v="%s"/>\n' % (tag.group(1), value)
    body = body[:tag.start()] + insert + body[tag.start():]
    return text[:m.start(2)] + body + text[m.end(2):], True


def repair(path, report=None):
    """Tag the junction lanelets of one map; returns how many were tagged. `report`, if
    given, is a Counter that receives the count per direction."""
    found = targets(path)
    if not found:
        return 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    fixed = 0
    for rel_id, value in found:
        text, ok = add_turn_direction(text, rel_id, value)
        fixed += ok
        if ok and report is not None:
            report[value] += 1
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return fixed


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        counts = collections.Counter()
        n = repair(m, counts)
        total += n
        detail = ", ".join(f"{counts[d]} {d}" for d in ("straight", "left", "right"))
        print(f"  {m.parent.name}: {n} junction lanelet(s) given turn_direction ({detail})" if n
              else f"  {m.parent.name}: every junction lanelet has a turn_direction")
    print(f"{total} lanelet turn direction(s) repaired")


if __name__ == "__main__":
    main()
