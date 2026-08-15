#!/usr/bin/env python3
"""Put the `traffic_light` tags back into a Lanelet2 map that shipped without them.

Four of the five towns in the map pack declare their traffic lights with the two tags
lanelet2 keys on left **empty**:

    <relation id="43856">                      <way id="43763">
      <member type="way" ref="43763" role="refers"/>   <tag k="subtype" v="red_redYellow_green_yellow"/>
      <member type="way" ref="43855" role="ref_line"/> <tag k="type" v=""/>        <-- traffic_light
      <tag k="subtype" v=""/>   <-- traffic_light      <tag k="height" v="1.2"/>
      <tag k="type" v="regulatory_element"/>    </way>
    </relation>

Everything else is there -- the bulb layout, the height, the stop line, the lanelet that
references it. Only the two type strings are blank, and blank is exactly what lanelet2
needs to build a `TrafficLight` regulatory element rather than a generic one. The
consequences are silent and total: SSv2 rejects the id with *"Given lanelet ID N is
neither a traffic light ID not a traffic relation ID"*, and Autoware's recognition and
planning find no traffic lights on the map at all, so a commanded red is invisible no
matter how good the camera is.

Town10 in the same pack is tagged correctly, which is where the target format comes from.

The repair is structural rather than a guess at the text, and deliberately narrow: a
regulatory element with an empty subtype, exactly one `refers` way, at least one
`ref_line`, and a referred way whose own `type` is empty. That admits the corrupted
subtypes (Town03's reads `_OLYeaShicfaTNEGeaShicfaTWLE_E.tttgLifr_E.tttgLifr` where a
bulb layout should be) and skips the `traffic_sign` / `right_of_way` elements, whose way
type is already set.

Edits are made as text against the two specific tags, not by re-serialising the document,
so a 12 MB map comes back otherwise byte-identical. Idempotent: a map that is already
correct is left alone and reported as such.

    scripts/repair_lanelet_traffic_lights.py data/carla-autoware-bridge
"""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def targets(path):
    """Return (relation ids, way ids) that need the traffic_light tag put back."""
    root = ET.parse(path).getroot()

    way_tags = {}
    for w in root.findall("way"):
        way_tags[w.get("id")] = {t.get("k"): t.get("v") for t in w.findall("tag")}

    rels, ways = [], []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "regulatory_element":
            continue
        if tags.get("subtype"):          # already typed -- traffic_light, right_of_way, ...
            continue
        refers = [m.get("ref") for m in rel.findall("member") if m.get("role") == "refers"]
        ref_line = [m.get("ref") for m in rel.findall("member") if m.get("role") == "ref_line"]
        if len(refers) != 1 or not ref_line:
            continue
        if way_tags.get(refers[0], {}).get("type"):   # a traffic sign, not a light
            continue
        rels.append(rel.get("id"))
        ways.append(refers[0])
    return rels, ways


def retag(text, kind, element_id, key, value):
    """Set an empty <tag k=key v=""/> to `value`, inside one <kind id=element_id> block."""
    block = re.compile(
        r'<%s id="%s"[^>]*>(.*?)</%s>' % (kind, re.escape(element_id), kind), re.S)
    m = block.search(text)
    if not m:
        return text, False
    body = m.group(1)
    empty = '<tag k="%s" v=""/>' % key
    if empty not in body:
        return text, False
    body = body.replace(empty, '<tag k="%s" v="%s"/>' % (key, value), 1)
    return text[:m.start(1)] + body + text[m.end(1):], True


def repair(path):
    rels, ways = targets(path)
    if not rels:
        return 0

    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)

    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    fixed = 0
    for rel_id, way_id in zip(rels, ways):
        text, a = retag(text, "relation", rel_id, "subtype", "traffic_light")
        text, b = retag(text, "way", way_id, "type", "traffic_light")
        fixed += 1 if (a and b) else 0
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
        print(f"  {m.parent.name}: {n} traffic light(s) retagged" if n
              else f"  {m.parent.name}: already correct")
    print(f"{total} traffic light(s) repaired")


if __name__ == "__main__":
    main()
