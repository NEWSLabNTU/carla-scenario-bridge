#!/usr/bin/env python3
"""Rebuild a Lanelet2 map's traffic light geometry from CARLA's own light actors.

The map pack's traffic light linestrings are placeholders. Every light in every town is
the same stub -- 0.141 m long (0.035 m on Town10) at a bearing of exactly 135 degrees --
even though Town01's thirty-six lights govern roads running in all four directions. What
a linestring is supposed to encode, Autoware reads like this
(`autoware_traffic_light_map_based_detector`, `traffic_light_utils`):

    front            bottom-left corner of the signal head, as the driver sees it
    back             bottom-right corner
    height (way tag) how far the head extends above those points
    tl_yaw           atan2(back - front) + 90 deg, i.e. the direction the head faces,
                     which must be within car_traffic_light_max_angle_range (40 deg) of
                     the camera's viewing direction or the light is skipped outright

So the stub fails twice over: it faces 135 degrees regardless of the road, which is more
than 40 degrees off every approach, and at 0.141 m it would project to about 5 px at 30 m
even if it were pointed the right way.

CARLA has the real thing. `TrafficLight.get_light_boxes()` returns the signal head in
world coordinates -- centre, extent and rotation -- so the width and the height and the
mounting position are all measured rather than guessed. Facing is *not* taken from CARLA:
it comes from the map, as the direction of travel of the lanelet that references the
regulatory element, because that is the definition Autoware is checking against and it
sidesteps having to pin down CARLA's own actor-yaw convention.

Positions are written as lat/lon *and* local_x/local_y/ele. lat/lon is what Autoware's
loader projects (map_projector_info.yaml declares TransverseMercator about 0,0, so
x = radians(lon) * 6378137 and y = radians(lat) * 6378137); local_x/local_y is what csb's
own parser reads. Writing one and not the other leaves the two halves of the stack
disagreeing about where the light is.

Needs CARLA running with the town already loaded, and the CARLA PythonAPI importable:

    PYTHONPATH=/tmp/carlapy scripts/regenerate_light_geometry.py Town01
"""
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import carla
except ImportError:
    sys.exit("carla PythonAPI not importable -- see this script's docstring")

EARTH_RADIUS = 6378137.0
# How far a CARLA light head may sit from the map's stub and still be the same light.
# The stubs are roughly in the right place; this only has to beat the spacing between
# neighbouring signals at one junction.
MATCH_TOLERANCE = 8.0


def to_latlon(x, y):
    return math.degrees(y / EARTH_RADIUS), math.degrees(x / EARTH_RADIUS)


def carla_heads(world):
    """Signal heads in the ROS frame: (x, y, z_centre, width, height)."""
    heads = []
    for actor in world.get_actors().filter("traffic.traffic_light*"):
        for box in actor.get_light_boxes():
            # extent is half-size; the head's width is its wider horizontal dimension,
            # the narrower one being its depth.
            width = 2.0 * max(box.extent.x, box.extent.y)
            height = 2.0 * box.extent.z
            # CARLA is left-handed; the ROS frame flips Y.
            heads.append((box.location.x, -box.location.y, box.location.z, width, height))
    return heads


def parse_map(path):
    root = ET.parse(path).getroot()
    nodes, ways = {}, {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if "local_x" in tags:
            nodes[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
        elif n.get("lat"):
            nodes[n.get("id")] = (
                math.radians(float(n.get("lon"))) * EARTH_RADIUS,
                math.radians(float(n.get("lat"))) * EARTH_RADIUS,
            )
    for w in root.findall("way"):
        ways[w.get("id")] = [nd.get("ref") for nd in w.findall("nd")]

    elements = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "traffic_light":
            continue
        refers = [m.get("ref") for m in rel.findall("member") if m.get("role") == "refers"]
        ref_line = [m.get("ref") for m in rel.findall("member") if m.get("role") == "ref_line"]
        elements[rel.get("id")] = {"refers": refers, "ref_line": ref_line}

    # Direction of travel of each lanelet that references a regulatory element, taken as
    # centroid -> stop line. A vehicle approaches its stop line, so that vector points the
    # way it is driving whichever order the boundary ways happen to be stored in -- which
    # is not consistent in this pack, so a centreline built by hand can come out reversed.
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        # Any lanelet, not just subtype=road. Half of Town01's signals are bound to
        # 0.3 m slivers of curb that no vehicle drives on, but a sliver still runs
        # alongside the lane it belongs to and still has the element's stop line at one
        # end, so it gives the same approach direction as the lane beside it. Requiring
        # subtype=road here left exactly those eighteen lights untouched.
        if tags.get("type") != "lanelet":
            continue
        regs = [m.get("ref") for m in rel.findall("member")
                if m.get("role") == "regulatory_element" and m.get("ref") in elements]
        if not regs:
            continue
        pts = []
        for m in rel.findall("member"):
            if m.get("role") in ("left", "right"):
                pts += [nodes[r] for r in ways.get(m.get("ref"), []) if r in nodes]
        if not pts:
            continue
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        for g in regs:
            stop = [nodes[r] for r in ways.get(elements[g]["ref_line"][0], []) if r in nodes]
            if not stop:
                continue
            sx = sum(p[0] for p in stop) / len(stop)
            sy = sum(p[1] for p in stop) / len(stop)
            elements[g].setdefault("approach", []).append(math.atan2(sy - cy, sx - cx))
    return nodes, ways, elements


def set_tag(block, key, value):
    """Replace <tag k=key v=.../> in an element block, or append one if absent."""
    pattern = re.compile(r'<tag k="%s" v="[^"]*"/>' % re.escape(key))
    new = '<tag k="%s" v="%s"/>' % (key, value)
    if pattern.search(block):
        return pattern.sub(new, block, count=1)
    return block.replace("</way>", "    " + new + "\n  </way>")


def rewrite_node(text, node_id, x, y, z):
    lat, lon = to_latlon(x, y)
    block_re = re.compile(r'<node id="%s"[^>]*>.*?</node>' % re.escape(node_id), re.S)
    m = block_re.search(text)
    if not m:
        return text, False
    block = m.group(0)
    block = re.sub(r'lat="[^"]*"', 'lat="%.12f"' % lat, block, count=1)
    block = re.sub(r'lon="[^"]*"', 'lon="%.12f"' % lon, block, count=1)
    for key, val in (("local_x", "%.4f" % x), ("local_y", "%.4f" % y), ("ele", "%.3f" % z)):
        pattern = re.compile(r'<tag k="%s" v="[^"]*"/>' % key)
        new = '<tag k="%s" v="%s"/>' % (key, val)
        block = pattern.sub(new, block, count=1) if pattern.search(block) else \
            block.replace("</node>", "    " + new + "\n  </node>")
    return text[:m.start()] + block + text[m.end():], True


def main():
    town = sys.argv[1] if len(sys.argv) > 1 else "Town01"
    map_path = Path(sys.argv[2] if len(sys.argv) > 2
                    else f"data/carla-autoware-bridge/{town}/lanelet2_map.osm")

    client = carla.Client("localhost", 2000)
    client.set_timeout(40.0)
    world = client.get_world()
    loaded = world.get_map().name.rsplit("/", 1)[-1]
    if loaded != town:
        sys.exit(f"CARLA holds {loaded}, not {town}; load the town first")
    # A client's actor list stays empty until it has seen a tick.
    world.tick() if world.get_settings().synchronous_mode else world.wait_for_tick()

    heads = carla_heads(world)
    if not heads:
        sys.exit("CARLA reported no traffic light heads")
    nodes, ways, elements = parse_map(map_path)
    print(f"{town}: {len(heads)} CARLA signal head(s), "
          f"{len(elements)} lanelet traffic light element(s)")

    backup = map_path.with_suffix(map_path.suffix + ".stub-geometry")
    if not backup.exists():
        shutil.copy2(map_path, backup)
    text = map_path.read_text(encoding="utf-8", errors="surrogateescape")

    done, skipped = 0, []
    for eid, el in sorted(elements.items()):
        way_id = el["refers"][0]
        stub = [nodes[r] for r in ways.get(way_id, []) if r in nodes]
        if len(stub) < 2 or "approach" not in el:
            skipped.append((eid, "no stub geometry or no referencing lanelet"))
            continue
        sx = sum(p[0] for p in stub) / len(stub)
        sy = sum(p[1] for p in stub) / len(stub)

        best = min(heads, key=lambda h: math.hypot(h[0] - sx, h[1] - sy))
        gap = math.hypot(best[0] - sx, best[1] - sy)
        if gap > MATCH_TOLERANCE:
            skipped.append((eid, f"nearest CARLA head {gap:.1f} m away"))
            continue

        hx, hy, hz, width, height = best
        # Average the approaches in case two lanes share the element; they point the same
        # way when they do.
        ax = sum(math.cos(a) for a in el["approach"])
        ay = sum(math.sin(a) for a in el["approach"])
        approach = math.atan2(ay, ax)
        bar = approach - math.pi / 2.0
        dx, dy = math.cos(bar) * width / 2.0, math.sin(bar) * width / 2.0
        bottom = hz - height / 2.0

        node_ids = ways[way_id]
        text, ok_a = rewrite_node(text, node_ids[0], hx - dx, hy - dy, bottom)
        text, ok_b = rewrite_node(text, node_ids[-1], hx + dx, hy + dy, bottom)
        if not (ok_a and ok_b):
            skipped.append((eid, "node blocks not found"))
            continue

        way_re = re.compile(r'<way id="%s"[^>]*>.*?</way>' % re.escape(way_id), re.S)
        m = way_re.search(text)
        if m:
            text = text[:m.start()] + set_tag(m.group(0), "height", "%.3f" % height) + text[m.end():]
        done += 1

    map_path.write_text(text, encoding="utf-8", errors="surrogateescape")
    print(f"  {done} light(s) regenerated from CARLA geometry")
    for eid, why in skipped:
        print(f"  skipped {eid}: {why}")


if __name__ == "__main__":
    main()
