#!/usr/bin/env python3
"""Link every signalised approach lanelet to the traffic light CARLA actually controls it by.

The map pack's lanelets reference traffic-light regulatory elements, but not the right
ones. On Town05's junction at (30, 0) (OpenDRIVE junction 1574), measured 2026-10-16:
two of the four approaches reference no traffic light at all, one references the element
of a neighbouring approach's light, and two elements are referenced only by walkways. The
signal table (`--generate-signal-table`) pairs each element with the CARLA light nearest
its light way, which is right; what is wrong is which lanelets point at which element.
Autoware then treats a signalised junction as unsignalised, or stops for another
approach's red and drives on it green: three Autoware vehicles under a signal controller
crept into the junction box on red and blocked each other in it.

CARLA knows the truth: `TrafficLight.get_stop_waypoints()` gives, per light, the stop
point of each lane it governs. For each light this script

- finds the light's regulatory element through the resolved signal table
  (`<map dir>/carla/traffic_lights.yaml`, opendrive_id -> regulatory_element_ids);
- finds the road lanelet containing each stop waypoint (heading within 37 degrees);
- gives the element a new stop line (`ref_line`) across those lanes at the stop
  waypoints, and makes those lanelets reference it.

Road lanelets keep only the traffic-light elements assigned here (a wrong link is
removed); walkways and other regulatory elements are untouched. The light ways and the
element ids stay, so the signal table stays valid. New nodes carry lat/lon and
local_x/local_y, as `regenerate_light_geometry.py` writes them. Edits are text edits;
the first run keeps `<map>.orig-signal-links` and every run starts from it, so a rerun
writes the same file.

Needs CARLA holding the town with something ticking it (swb during a scenario, or a
private server): this script never ticks, it waits for a frame, since a client that has
not seen one sees no actors.

    scripts/repair_lanelet_signal_links.py Town05 [--map data/carla-autoware-bridge/Town05]
        [--port 2000] [--dry-run]
"""
import argparse
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import carla
except ImportError:
    sys.exit("carla PythonAPI not importable")
try:
    import yaml
except ImportError:
    sys.exit("PyYAML not importable (python3-yaml)")

EARTH_RADIUS = 6378137.0
HEADING_COS = 0.8  # a stop waypoint's lane and the lanelet agree within ~37 degrees


def to_latlon(x, y):
    return math.degrees(y / EARTH_RADIUS), math.degrees(x / EARTH_RADIUS)


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / max(ax * ax + ay * ay, 1e-12)
    t = min(1.0, max(0.0, t))
    q = (a[0] + t * ax, a[1] + t * ay)
    return math.hypot(p[0] - q[0], p[1] - q[1]), q, (ax, ay)


def poly_near(p, pts):
    """Distance from p to a polyline, the nearest point and that segment's direction."""
    return min((seg_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1)),
               key=lambda r: r[0])


def parse(path):
    root = ET.parse(path).getroot()
    nodes = {}
    max_id = 0
    for el in root:
        if el.get("id"):
            max_id = max(max_id, int(el.get("id")))
    for n in root.findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        if "local_x" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    elements = set()
    for rel in root.findall("relation"):
        t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
        if t.get("type") == "regulatory_element" and t.get("subtype") == "traffic_light":
            elements.add(rel.get("id"))
    lanelets = {}
    for rel in root.findall("relation"):
        t = {x.get("k"): x.get("v") for x in rel.findall("tag")}
        if t.get("type") != "lanelet" or t.get("subtype") != "road":
            continue
        m = {x.get("role"): x.get("ref") for x in rel.findall("member") if x.get("role") in ("left", "right")}
        if "left" not in m or "right" not in m:
            continue
        left = [nodes[r] for r in ways.get(m["left"], []) if r in nodes]
        right = [nodes[r] for r in ways.get(m["right"], []) if r in nodes]
        if len(left) < 2 or len(right) < 2:
            continue
        regs = [x.get("ref") for x in rel.findall("member")
                if x.get("role") == "regulatory_element" and x.get("ref") in elements]
        lanelets[rel.get("id")] = (left, right, regs)
    return nodes, ways, elements, lanelets, max_id


def lanelet_at(lanelets, p, f):
    """The road lanelet containing p whose direction is f (unit, map frame).

    The direction is taken from which side the left bound is on, not from the bounds' node
    order: two opposite lanelets share their middle line, so its order is one lanelet's
    and backwards for the other. A stop waypoint on that shared line is inside both, and
    an unsigned heading test then took the oncoming lanelet about half the time (Town01's
    light 643, roadmap 051).
    """
    best = None
    for lid, (left, right, _) in lanelets.items():
        dl, ql, _ = poly_near(p, left)
        dr, qr, _ = poly_near(p, right)
        width = math.hypot(ql[0] - qr[0], ql[1] - qr[1])
        # 1 m of slack: where the map's lanes sit half a lane off CARLA's, the stop waypoint
        # lies on a bound or just past it (Town01's lights 366 and 373, 0.2 m outside). The
        # direction test below keeps that from reaching the oncoming lanelet.
        if width < 1.5 or dl + dr > width + 1.0:
            continue
        # Right to left, turned a quarter clockwise: the way the lanelet runs.
        d = (ql[1] - qr[1], -(ql[0] - qr[0]))
        if (d[0] * f[0] + d[1] * f[1]) / width < HEADING_COS:
            continue
        slack = dl + dr - width
        if best is None or slack < best[0]:
            best = (slack, lid)
    return best[1] if best else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("town")
    ap.add_argument("--map", help="map dir (default data/carla-autoware-bridge/<town>)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    mdir = Path(a.map or f"data/carla-autoware-bridge/{a.town}")
    osm = mdir / "lanelet2_map.osm"
    table = yaml.safe_load((mdir / "carla" / "traffic_lights.yaml").read_text())
    od_elements = {}
    for s in table["signals"]:
        od_elements.setdefault(str(s["opendrive_id"]), []).extend(
            str(e) for e in s.get("regulatory_element_ids", []))

    client = carla.Client(a.host, a.port)
    client.set_timeout(30.0)
    world = client.get_world()
    loaded = world.get_map().name.rsplit("/", 1)[-1]
    if loaded != a.town:
        sys.exit(f"CARLA holds {loaded}, not {a.town}")
    try:
        world.wait_for_tick(60.0)
    except RuntimeError:
        sys.exit("no frame within 60 s: run this while something ticks the world")

    # Always from the map as it was before the first run, so a rerun gives the same file.
    backup = osm.with_suffix(osm.suffix + ".orig-signal-links")
    source = backup if backup.exists() else osm
    nodes, ways, elements, lanelets, max_id = parse(source)
    plan = {}  # element -> (stop line endpoints, [lanelet ids], od id)
    missing = []
    for tl in world.get_actors().filter("traffic.traffic_light*"):
        od = str(tl.get_opendrive_id())
        els = [e for e in od_elements.get(od, []) if e in elements]
        if not els:
            missing.append(f"light {od}: no element in the signal table")
            continue
        wps = tl.get_stop_waypoints()
        if not wps:
            continue
        lids, pts = [], []
        fwd = wps[0].transform.get_forward_vector()
        f = (fwd.x, -fwd.y)
        n = math.hypot(*f) or 1.0
        f = (f[0] / n, f[1] / n)
        r = (f[1], -f[0])  # right of travel, map frame
        for wp in wps:
            p = (wp.transform.location.x, -wp.transform.location.y)
            lid = lanelet_at(lanelets, p, f)
            if lid is None:
                missing.append(f"light {od}: no road lanelet at stop waypoint ({p[0]:.1f}, {p[1]:.1f})")
                continue
            lids.append(lid)
            half = wp.lane_width / 2.0
            s = p[0] * r[0] + p[1] * r[1]
            pts.append((s - half, p, wp.transform.location.z))
            pts.append((s + half, p, wp.transform.location.z))
            # And across the lanelet's own bounds there: where the map's lane sits off
            # CARLA's, a line only as wide as CARLA's lane misses Autoware's path (Town01's
            # light 373, 2.2 m off: the ego drove through SSv2's red, roadmap 051).
            for bound in lanelets[lid][:2]:
                q = poly_near(p, bound)[1]
                pts.append((q[0] * r[0] + q[1] * r[1], p, wp.transform.location.z))
        if not lids:
            continue
        lo, hi = min(pts), max(pts)
        z = sum(q[2] for q in pts) / len(pts)

        def at(s, p):
            k = s - (p[0] * r[0] + p[1] * r[1])
            return (p[0] + k * r[0], p[1] + k * r[1])

        plan[els[0]] = ((at(lo[0], lo[1]), at(hi[0], hi[1]), z), sorted(set(lids)), od)

    assign = {}
    for e, (_, lids, _) in plan.items():
        for lid in lids:
            assign.setdefault(lid, []).append(e)
    changed_links = [
        (lid, regs, sorted(assign.get(lid, [])))
        for lid, (_, _, regs) in lanelets.items()
        if sorted(regs) != sorted(assign.get(lid, [])) and (regs or lid in assign)
    ]
    print(f"{a.town}: {len(plan)} light(s) placed on {len(assign)} lanelet(s); "
          f"{len(changed_links)} lanelet(s) change their traffic-light links")
    for lid, old, new in changed_links:
        print(f"  lanelet {lid}: {old or '-'} -> {new or '-'}")
    for m in missing:
        print(f"  {m}")
    if a.dry_run:
        return

    if not backup.exists():
        shutil.copy2(osm, backup)
    text = backup.read_text(encoding="utf-8", errors="surrogateescape")
    new_xml = []
    nid = max_id
    for e, ((p0, p1, z), _, od) in sorted(plan.items()):
        ids = []
        for x, y in (p0, p1):
            nid += 1
            lat, lon = to_latlon(x, y)
            new_xml.append(
                f'  <node id="{nid}" lat="{lat:.12f}" lon="{lon:.12f}">\n'
                f'    <tag k="ele" v="{z:.3f}"/>\n'
                f'    <tag k="local_x" v="{x:.4f}"/>\n'
                f'    <tag k="local_y" v="{y:.4f}"/>\n'
                f'  </node>')
            ids.append(nid)
        nid += 1
        new_xml.append(
            f'  <way id="{nid}">\n'
            f'    <nd ref="{ids[0]}"/>\n    <nd ref="{ids[1]}"/>\n'
            f'    <tag k="type" v="stop_line"/>\n'
            f'    <tag k="swb_signal_link" v="{od}"/>\n'
            f'  </way>')
        rel_re = re.compile(r'<relation id="%s"[^>]*>.*?</relation>' % re.escape(e), re.S)
        m = rel_re.search(text)
        block = m.group(0)
        if 'role="ref_line"' in block:
            block = re.sub(r'<member type="way" ref="\d+" role="ref_line"\s*/>',
                           f'<member type="way" ref="{nid}" role="ref_line"/>', block, count=1)
            block = re.sub(r'\s*<member type="way" ref="\d+" role="ref_line"\s*/>(?=(.|\n)*'
                           r'role="ref_line")', "", block)
        else:
            block = block.replace("</relation>",
                                  f'  <member type="way" ref="{nid}" role="ref_line"/>\n  </relation>')
        text = text[:m.start()] + block + text[m.end():]
    for lid, old, new in changed_links:
        rel_re = re.compile(r'<relation id="%s"[^>]*>.*?</relation>' % re.escape(lid), re.S)
        m = rel_re.search(text)
        block = m.group(0)
        for e in old:
            block = re.sub(r'\s*<member type="relation" ref="%s" role="regulatory_element"\s*/>'
                           % re.escape(e), "", block)
        for e in new:
            block = block.replace(
                "</relation>",
                f'  <member type="relation" ref="{e}" role="regulatory_element"/>\n  </relation>')
        text = text[:m.start()] + block + text[m.end():]
    text = text.replace("</osm>", "\n".join(new_xml) + "\n</osm>")
    osm.write_text(text, encoding="utf-8", errors="surrogateescape")
    print(f"  wrote {osm} ({len(plan)} stop line(s)); original kept as {backup.name}")


if __name__ == "__main__":
    main()
