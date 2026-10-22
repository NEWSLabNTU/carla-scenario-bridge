"""Unit tests for scripts/repair_lanelet_turn_direction.py (roadmap 033 step 4a).

A small synthetic Lanelet2 map, written the way the map pack writes one: an eastbound
approach that forks three ways (straight, right, left), a northbound approach that turns
right into the straight branch's exit (a merge), and a walkway. No ROS:

    python3 -m pytest src/swb_launch/test/test_repair_lanelet_turn_direction.py
"""

import importlib.machinery
import importlib.util
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

_SCRIPT = (Path(__file__).resolve().parents[3] / "scripts"
           / "repair_lanelet_turn_direction.py")
_loader = importlib.machinery.SourceFileLoader("repair_lanelet_turn_direction", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("repair_lanelet_turn_direction", _loader)
rtd = importlib.util.module_from_spec(_spec)
_loader.exec_module(rtd)

R = 6378137.0


class _Map:
    """Builds the OSM text. Positions are ROS-frame metres, written as lat/lon through
    CARLA's spherical Mercator (the pack's). local_x / local_y are written *mirrored*
    (local_y = -y), so a script that took its geometry from them would get every turn
    backwards: the pack's local_* are not where the lanes are (033 step 3)."""

    def __init__(self):
        self.next_id = 1
        self.nodes, self.ways, self.rels = [], [], []
        self.ids = {}

    def _id(self):
        self.next_id += 1
        return self.next_id

    def node(self, x, y):
        key = (round(x, 3), round(y, 3))
        if key in self.ids:
            return self.ids[key]
        nid = self._id()
        lat = math.degrees(2 * math.atan(math.exp(y / R)) - math.pi / 2)
        lon = math.degrees(x / R)
        self.nodes.append(
            f'  <node id="{nid}" visible="true" version="1" lat="{lat:.11f}" lon="{lon:.11f}">\n'
            f'    <tag k="local_x" v="{x:.4f}"/>\n'
            f'    <tag k="local_y" v="{-y:.4f}"/>\n'
            f'    <tag k="ele" v="0.0"/>\n'
            f'  </node>\n')
        self.ids[key] = nid
        return nid

    def way(self, pts):
        wid = self._id()
        refs = "".join(f'    <nd ref="{self.node(x, y)}"/>\n' for x, y in pts)
        self.ways.append(f'  <way id="{wid}" visible="true" version="1">\n{refs}'
                         f'    <tag k="type" v="line_thin"/>\n'
                         f'    <tag k="subtype" v="solid"/>\n  </way>\n')
        return wid

    def lanelet(self, left, right, subtype="road", extra=""):
        lid = self._id()
        lw, rw = self.way(left), self.way(right)
        self.rels.append(f'  <relation id="{lid}" action="modify" visible="true" version="1">\n'
                         f'    <member type="way" ref="{rw}" role="right"/>\n'
                         f'    <member type="way" ref="{lw}" role="left"/>\n'
                         f'    <tag k="type" v="lanelet"/>\n'
                         f'    <tag k="subtype" v="{subtype}"/>\n{extra}'
                         f'    <tag k="location" v="urban"/>\n  </relation>\n')
        return str(lid)

    def text(self):
        return ('<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6" generator="test">\n'
                + "".join(self.nodes) + "".join(self.ways) + "".join(self.rels) + "</osm>\n")


def _line(a, b, n=6):
    return [(a[0] + (b[0] - a[0]) * i / (n - 1), a[1] + (b[1] - a[1]) * i / (n - 1))
            for i in range(n)]


def _arc(c, r, a0, a1, n=9):
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * i / (n - 1)),
             c[1] + r * math.sin(a0 + (a1 - a0) * i / (n - 1))) for i in range(n)]


def build():
    """The map and the ids of its lanelets by role."""
    m = _Map()
    h = math.pi / 2
    ids = {}
    # Eastbound approach (left bound y 0, right bound y -4), then a fork at x -5.
    ids["approach"] = m.lanelet(_line((-50, 0), (-5, 0)), _line((-50, -4), (-5, -4)))
    ids["straight"] = m.lanelet(_line((-5, 0), (5, 0)), _line((-5, -4), (5, -4)))
    ids["exit_e"] = m.lanelet(_line((5, 0), (50, 0)), _line((5, -4), (50, -4)))
    # Right into a southbound road: quarter circles about (-5, -8).
    ids["right"] = m.lanelet(_arc((-5, -8), 8, h, 0), _arc((-5, -8), 4, h, 0))
    ids["exit_s"] = m.lanelet(_line((3, -8), (3, -50)), _line((-1, -8), (-1, -50)))
    # Left into a northbound road: quarter circles about (-5, 4). Its left bound is
    # stored reversed, as the pack stores some (lanelet2 reads it the right way round).
    ids["left"] = m.lanelet(_arc((-5, 4), 4, -h, 0)[::-1], _arc((-5, 4), 8, -h, 0))
    ids["exit_n"] = m.lanelet(_line((-1, 4), (-1, 50)), _line((3, 4), (3, 50)))
    # A northbound approach with one successor, which turns right into exit_e: that
    # lanelet joins a merge (exit_e has two predecessors) without leaving a fork.
    ids["approach_n"] = m.lanelet(_line((-3, -50), (-3, -8)), _line((1, -50), (1, -8)))
    ids["merge_right"] = m.lanelet(_arc((5, -8), 8, math.pi, h), _arc((5, -8), 4, math.pi, h))
    # Not driveable: never tagged, whatever its topology.
    ids["walkway"] = m.lanelet(_line((-50, 4), (-5, 4)), _line((-50, 0.5), (-5, 0.5)),
                               subtype="walkway")
    return m.text(), ids


def tags_of(path):
    root = ET.parse(path).getroot()
    return {r.get("id"): {t.get("k"): t.get("v") for t in r.findall("tag")}
            for r in root.findall("relation")}


class RepairTurnDirection(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "Town99" / "lanelet2_map.osm"
        self.path.parent.mkdir()
        self.original, self.ids = build()
        self.path.write_text(self.original, encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def test_junction_lanelets_tagged_by_heading_change(self):
        self.assertEqual(rtd.repair(self.path), 4)
        tags = tags_of(self.path)
        want = {"straight": "straight", "right": "right", "left": "left",
                "merge_right": "right"}
        for role, lid in self.ids.items():
            with self.subTest(role=role):
                self.assertEqual(tags[lid].get("turn_direction"), want.get(role))

    def test_rest_of_file_byte_identical(self):
        rtd.repair(self.path)
        fixed = self.path.read_text(encoding="utf-8")
        added = [ln for ln in fixed.splitlines() if ln not in self.original.splitlines()]
        self.assertEqual(len(added), 4)
        self.assertTrue(all(ln.strip().startswith('<tag k="turn_direction"') for ln in added))
        stripped = "".join(ln + "\n" for ln in fixed.splitlines() if ln not in added)
        self.assertEqual(stripped, self.original)

    def test_idempotent_and_orig_kept(self):
        rtd.repair(self.path)
        once = self.path.read_bytes()
        backup = self.path.with_suffix(".osm.orig")
        self.assertEqual(backup.read_text(encoding="utf-8"), self.original)
        self.assertEqual(rtd.repair(self.path), 0)
        self.assertEqual(self.path.read_bytes(), once)
        self.assertEqual(backup.read_text(encoding="utf-8"), self.original)

    def test_existing_orig_not_overwritten(self):
        # Another repair_lanelet_*.py ran first: its .orig is the pack's file, keep it.
        backup = self.path.with_suffix(".osm.orig")
        backup.write_text("pristine", encoding="utf-8")
        rtd.repair(self.path)
        self.assertEqual(backup.read_text(encoding="utf-8"), "pristine")

    def test_existing_tag_left_alone(self):
        # A map that already says what a junction lanelet is keeps its word.
        lid = self.ids["right"]
        marked = self.original.replace(
            f'<relation id="{lid}" action="modify" visible="true" version="1">\n',
            f'<relation id="{lid}" action="modify" visible="true" version="1">\n'
            '    <tag k="turn_direction" v="straight"/>\n')
        self.path.write_text(marked, encoding="utf-8")
        self.assertEqual(rtd.repair(self.path), 3)
        self.assertEqual(tags_of(self.path)[lid]["turn_direction"], "straight")

    def test_threshold_between_straight_and_turn(self):
        self.assertEqual(rtd.direction(math.radians(rtd.TURN_DEG - 1)), "straight")
        self.assertEqual(rtd.direction(math.radians(rtd.TURN_DEG + 1)), "left")
        self.assertEqual(rtd.direction(-math.radians(rtd.TURN_DEG + 1)), "right")


if __name__ == "__main__":
    unittest.main()
