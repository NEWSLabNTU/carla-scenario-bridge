#!/usr/bin/env python3
"""Remove right_of_way regulatory elements that name no lanelet with right of way.

Town05 (131372) and Town10 (37142) each carry one `right_of_way` regulatory element with a
`yield` lanelet and a `refers` way but no `right_of_way` member. Autoware's loader logs the
error and goes on; scenario_simulator_v2's does not: every scenario on those towns ends at
once with

    Failed to load lanelet map ... Error parsing primitive 131372: Creating a regulatory
    element of type right_of_way failed: A maneuver must refer to at least one lanelet that
    has right of way!

(roadmap 043, the exit runs). An element with nobody to yield to means nothing, and in the
pack no lanelet even refers to either one, so the repair deletes the element and any
`regulatory_element` reference to it.

Edits are text deletions of the affected `<relation>` block and member lines, so the rest of
the file stays byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with
the other repair_lanelet_*.py scripts, whichever runs first).

    scripts/repair_lanelet_right_of_way.py data/carla-autoware-bridge
"""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def targets(path):
    """Ids of the right_of_way regulatory elements with no right_of_way member."""
    root = ET.parse(path).getroot()
    ids = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "right_of_way":
            continue
        if not any(m.get("role") == "right_of_way" for m in rel.findall("member")):
            ids.append(rel.get("id"))
    return ids


def remove(text, rel_id):
    """Delete one <relation id=rel_id> block and every regulatory_element member naming it."""
    block = re.compile(r'[ \t]*<relation id="%s"[^>]*>.*?</relation>[ \t]*\n?' % re.escape(rel_id),
                       re.S)
    text, n = block.subn("", text, count=1)
    member = re.compile(r'[ \t]*<member type="relation" ref="%s" role="regulatory_element"\s*/>'
                        r'[ \t]*\n?' % re.escape(rel_id))
    text = member.sub("", text)
    return text, bool(n)


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
        text, ok = remove(text, rel_id)
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
        print(f"  {m.parent.name}: removed {n} right_of_way element(s) with no right of way"
              if n else f"  {m.parent.name}: every right_of_way element names one")
    print(f"{total} right_of_way element(s) removed")


if __name__ == "__main__":
    main()
