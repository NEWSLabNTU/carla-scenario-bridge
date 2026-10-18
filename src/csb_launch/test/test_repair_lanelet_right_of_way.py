"""Unit tests for scripts/repair_lanelet_right_of_way.py (roadmap 043, exit runs). No ROS:

    python3 -m pytest src/csb_launch/test/test_repair_lanelet_right_of_way.py
"""

import importlib.machinery
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

_SCRIPT = (Path(__file__).resolve().parents[3] / "scripts" / "repair_lanelet_right_of_way.py")
_loader = importlib.machinery.SourceFileLoader("repair_lanelet_right_of_way", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("repair_lanelet_right_of_way", _loader)
rrw = importlib.util.module_from_spec(_spec)
_loader.exec_module(rrw)

MAP = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <relation id="10" action="modify" version="1" visible="true">
    <member type="way" ref="1" role="left"/>
    <member type="way" ref="2" role="right"/>
    <member type="relation" ref="30" role="regulatory_element"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
  </relation>
  <relation id="20" action="modify" version="1" visible="true">
    <member type="way" ref="3" role="refers"/>
    <member type="relation" ref="10" role="yield"/>
    <member type="relation" ref="11" role="right_of_way"/>
    <tag k="subtype" v="right_of_way"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
  <relation id="30" action="modify" version="1" visible="true">
    <member type="way" ref="3" role="refers"/>
    <member type="relation" ref="10" role="yield"/>
    <tag k="subtype" v="right_of_way"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
</osm>
"""


class RepairRightOfWay(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "Town05" / "lanelet2_map.osm"
        self.path.parent.mkdir()
        self.path.write_text(MAP)

    def tearDown(self):
        self.dir.cleanup()

    def test_removes_only_the_element_with_no_right_of_way(self):
        self.assertEqual(rrw.repair(self.path), 1)
        root = ET.parse(self.path).getroot()
        ids = [r.get("id") for r in root.findall("relation")]
        self.assertEqual(ids, ["10", "20"])

    def test_drops_the_lanelets_reference_to_it(self):
        rrw.repair(self.path)
        lanelet = ET.parse(self.path).getroot().find("relation[@id='10']")
        refs = [m.get("ref") for m in lanelet.findall("member")
                if m.get("role") == "regulatory_element"]
        self.assertEqual(refs, [])

    def test_idempotent_and_keeps_the_original(self):
        rrw.repair(self.path)
        once = self.path.read_text()
        self.assertEqual(rrw.repair(self.path), 0)
        self.assertEqual(self.path.read_text(), once)
        self.assertEqual(self.path.with_suffix(".osm.orig").read_text(), MAP)


if __name__ == "__main__":
    unittest.main()
