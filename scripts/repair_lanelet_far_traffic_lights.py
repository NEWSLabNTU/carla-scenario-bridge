#!/usr/bin/env python3
"""Unlink traffic-light regulatory elements whose lights stand far from their stop line.

An OpenDRIVE -> Lanelet2 conversion can attach a traffic-light element to a lanelet whose
stop line is nowhere near the light it refers to. On CARLA's Town04 (converted with
CommonRoad Scenario Designer 0.8.5, 2026-10-14) 69 of 128 elements were such, one at the
foot of the highway ramp pointing at a light 400 m away, and Autoware stopped at the merge
waiting for a signal that never comes.

An element is far when every light it `refers` to has its centre more than FAR metres
(default 60) from the centre of its `ref_line`. The lanelets stop referencing far elements;
the elements themselves stay, so their ids remain valid for the signal table. Distances use
the nodes' `local_x` / `local_y` tags (run `repair_lanelet_local_coords.py` first if the map
has none). Idempotent: a second run finds nothing to unlink.

  scripts/repair_lanelet_far_traffic_lights.py <lanelet2_map.osm> [--far 60] [--dry-run]

Roadmap 031, stage 1 (defects to expect).
"""

import argparse
import math
import xml.etree.ElementTree as ET


def tags(el):
    return {t.get("k"): t.get("v") for t in el.iter("tag")}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osm")
    ap.add_argument("--far", type=float, default=60.0, help="metres, light to stop line")
    ap.add_argument("--dry-run", action="store_true", help="report, do not write")
    args = ap.parse_args()

    tree = ET.parse(args.osm)
    root = tree.getroot()
    nodes = {}
    for n in root.iter("node"):
        t = tags(n)
        if "local_x" in t and "local_y" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.iter("nd")] for w in root.iter("way")}

    def centre(way_id):
        pts = [nodes[i] for i in ways.get(way_id, []) if i in nodes]
        if not pts:
            return None
        return (sum(x for x, _ in pts) / len(pts), sum(y for _, y in pts) / len(pts))

    far = set()
    total = 0
    for r in root.iter("relation"):
        if tags(r).get("subtype") != "traffic_light":
            continue
        total += 1
        lines = [centre(m.get("ref")) for m in r.iter("member") if m.get("role") == "ref_line"]
        lights = [centre(m.get("ref")) for m in r.iter("member") if m.get("role") == "refers"]
        if lines and lines[0] and lights and all(
            light and math.dist(light, lines[0]) > args.far for light in lights
        ):
            far.add(r.get("id"))

    dropped = 0
    for r in root.iter("relation"):
        for m in list(r.findall("member")):
            if m.get("role") == "regulatory_element" and m.get("ref") in far:
                r.remove(m)
                dropped += 1
    print(
        f"{len(far)} of {total} traffic-light elements farther than {args.far} m from their "
        f"stop line; {dropped} lanelet references {'to drop' if args.dry_run else 'dropped'}"
    )
    if dropped and not args.dry_run:
        tree.write(args.osm, encoding="UTF-8", xml_declaration=True)


if __name__ == "__main__":
    main()
