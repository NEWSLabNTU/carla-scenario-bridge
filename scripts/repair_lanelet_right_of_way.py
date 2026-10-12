#!/usr/bin/env python3
"""Drop right_of_way regulatory elements that name no lanelet with right of way.

The map pack's converter emits, in Town05 and Town10, one `right_of_way` regulatory
element whose members are only `refers` and `yield`: no lanelet carries the
`right_of_way` role. lanelet2's `RightOfWay` constructor rejects that ("A maneuver must
refer to at least one lanelet that has right of way!"), and scenario_simulator_v2's
`lanelet_loader` treats a parse error as fatal, so every scenario on those towns dies at
`Initialize` with `Failed to load lanelet map`. Autoware's own loader drops the element
with a warning and carries on, which is why the vehicle side comes up and only the
scenario fails.

The repair removes the relation and every `regulatory_element` member that points at it
from the lanelets that referenced it. Nothing else in the file changes: an element with no
right-of-way lanelet grants nothing, so routing and traffic rules are as before.

Edits are text deletions of whole lines, so the rest of the file stays byte-identical.
Idempotent; the first edit keeps a `.orig` copy (shared with the other repair scripts,
whichever runs first).

    scripts/repair_lanelet_right_of_way.py data/carla-autoware-bridge
"""
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def targets(path):
    """Ids of right_of_way relations with no member in the right_of_way role."""
    root = ET.parse(path).getroot()
    ids = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "right_of_way":
            continue
        roles = {m.get("role") for m in rel.findall("member")}
        if "right_of_way" not in roles:
            ids.append(rel.get("id"))
    return ids


def drop(text, rel_id):
    """Delete one <relation id=rel_id> block and every member line referring to it."""
    block = re.compile(r'[ \t]*<relation id="%s"[^>]*>.*?</relation>\n' % re.escape(rel_id), re.S)
    text, n = block.subn("", text, count=1)
    member = re.compile(
        r'[ \t]*<member\b(?=[^>]*\brole="regulatory_element")(?=[^>]*\bref="%s")[^>]*/>\n' % re.escape(rel_id))
    text, refs = member.subn("", text)
    return text, n == 1, refs


def repair(path):
    ids = targets(path)
    if not ids:
        return 0, 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    dropped = refs = 0
    for rel_id in ids:
        text, ok, n = drop(text, rel_id)
        dropped += ok
        refs += n
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return dropped, refs


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        dropped, refs = repair(m)
        total += dropped
        print(f"  {m.parent.name}: {dropped} right_of_way element(s) dropped, {refs} reference(s) removed"
              if dropped else f"  {m.parent.name}: every right_of_way element names a lanelet with right of way")
    print(f"{total} right_of_way element(s) repaired")


if __name__ == "__main__":
    main()
