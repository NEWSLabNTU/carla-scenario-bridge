#!/usr/bin/env python3
"""Join consecutive road lanelets whose ends meet but share no nodes.

Lanelet2 routes from one lanelet to the next only when the next one's bounds start at the
very nodes the first one's bounds end at (`lanelet::geometry::follows` compares point ids).
The map pack's converter sometimes starts a lanelet at fresh nodes placed exactly where
its predecessor ends. Routing then stops there: in Town03 nothing routes from the
roundabout's north entry to its south exit, because the ring's exit lanelet 128008 ends at
the nodes of (-6.6, -41.6) and the south arm's lanelet 129957 starts at two other nodes
at the same place (found by the crdt-tensor testbed, its phase 017).

The repair finds every pair of `road` lanelets where one's left and right bounds end
within 5 cm of the other's left and right starts, heading on in the same direction, and
makes the second start at the first one's end nodes: every reference to the second pair
of nodes, in every way, is replaced by the first pair, so lanelets that share those
bounds stay joined too. Walkways, crosswalks and `unknown` strips (kerbs, shoulders) are
left as they are: joining them would let Autoware route over space it must not drive.

Edits are text replacements of `<nd ref=.../>` lines, so the rest of the file stays
byte-identical. Idempotent (a second run finds nothing left to join); the first edit keeps
a `.orig` copy (shared with the other repair scripts, whichever runs first).

    scripts/repair_lanelet_joins.py data/carla-autoware-bridge [--dry-run]
"""
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

TOL_M = 0.05
SUBTYPES = {"road"}


def plan(path):
    """The node merges that join meeting road lanelets: {old node id: new node id}."""
    root = ET.parse(path).getroot()
    pos = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            pos[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    lanelets = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") not in SUBTYPES:
            continue
        m = {x.get("role"): x.get("ref") for x in rel.findall("member") if x.get("type") == "way"}
        left, right = ways.get(m.get("left"), []), ways.get(m.get("right"), [])
        if len(left) < 2 or len(right) < 2 or not all(n in pos for n in left + right):
            continue
        lanelets.append((rel.get("id"), left, right))

    def centre(left, right, k):
        a, b = pos[left[k]], pos[right[k]]
        return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

    # Lanelet starts by a coarse grid cell of their left start node.
    cell = lambda p: (math.floor(p[0]), math.floor(p[1]))
    starts = {}
    for lid, left, right in lanelets:
        starts.setdefault(cell(pos[left[0]]), []).append((lid, left, right))
    merges = {}
    for lid, left, right in lanelets:
        le, re_ = left[-1], right[-1]
        pe = pos[le]
        d_out = [a - b for a, b in zip(centre(left, right, -1), centre(left, right, -2))]
        cands = []
        cx, cy = cell(pe)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                cands.extend(starts.get((cx + dx, cy + dy), []))
        for jid, jl, jr in cands:
            if jid == lid or (jl[0] == le and jr[0] == re_):
                continue
            if math.dist(pos[jl[0]], pe) > TOL_M or math.dist(pos[jr[0]], pos[re_]) > TOL_M:
                continue
            d_in = [a - b for a, b in zip(centre(jl, jr, 1), centre(jl, jr, 0))]
            if d_out[0] * d_in[0] + d_out[1] * d_in[1] <= 0:
                continue
            for old, new in ((jl[0], le), (jr[0], re_)):
                if old != new:
                    merges.setdefault(old, new)
    # Resolve chains so every old id maps to a node that is not itself merged away.
    def final(n, seen=()):
        while n in merges and n not in seen:
            seen = seen + (n,)
            n = merges[n]
        return n
    return {old: final(new) for old, new in merges.items() if final(new) != old}


def repair(path, dry_run=False):
    merges = plan(path)
    if not merges or dry_run:
        return len(merges)
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    nd = re.compile(r'(<nd ref=")(\d+)("\s*/>)')
    text = nd.sub(lambda m: m.group(1) + merges.get(m.group(2), m.group(2)) + m.group(3), text)
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return len(merges)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    dry = "--dry-run" in sys.argv
    root = Path(args[0] if args else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        n = repair(m, dry)
        total += n
        verb = "would merge" if dry else "merged"
        print(f"  {m.parent.name}: {verb} {n} node(s) at road-lanelet joins" if n
              else f"  {m.parent.name}: every meeting road lanelet already joined")
    print(f"{total} node merge(s){' (dry run)' if dry else ''}")


if __name__ == "__main__":
    main()
