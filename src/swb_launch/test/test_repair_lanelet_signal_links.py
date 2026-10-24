"""Unit tests for scripts/repair_lanelet_signal_links.py's lanelet choice (roadmap 051). No
ROS, no CARLA server (the script imports the carla module; without it these skip):

    python3 -m pytest src/swb_launch/test/test_repair_lanelet_signal_links.py
"""

import importlib.machinery
import importlib.util
import unittest
from pathlib import Path

_SCRIPT = (Path(__file__).resolve().parents[3] / "scripts" / "repair_lanelet_signal_links.py")
try:
    _loader = importlib.machinery.SourceFileLoader("repair_lanelet_signal_links", str(_SCRIPT))
    _spec = importlib.util.spec_from_loader("repair_lanelet_signal_links", _loader)
    rsl = importlib.util.module_from_spec(_spec)
    _loader.exec_module(rsl)
except SystemExit:  # no carla PythonAPI on this host
    rsl = None

# A two-lane road along x, as Town01 has it: one middle line (way 1, stored east-bound),
# the east-bound lanelet south of it (left bound the middle line), the west-bound one north
# of it (left bound the same line, which runs backwards for it).
MIDDLE = [(0.0, 0.0), (50.0, 0.0)]
EAST = (MIDDLE, [(0.0, -4.0), (50.0, -4.0)], [])
WEST = (MIDDLE, [(50.0, 4.0), (0.0, 4.0)], [])


@unittest.skipIf(rsl is None, "carla PythonAPI not importable")
class LaneletAtTest(unittest.TestCase):
    def test_a_point_on_the_middle_line_takes_the_lanelet_running_its_way(self):
        lanelets = {"west": WEST, "east": EAST}
        self.assertEqual(rsl.lanelet_at(lanelets, (25.0, -0.2), (1.0, 0.0)), "east")
        self.assertEqual(rsl.lanelet_at(lanelets, (25.0, -0.2), (-1.0, 0.0)), "west")
        self.assertEqual(rsl.lanelet_at(lanelets, (25.0, 0.2), (1.0, 0.0)), "east")

    def test_inside_a_lane_but_against_it_is_no_match(self):
        self.assertIsNone(rsl.lanelet_at({"west": WEST}, (25.0, 2.0), (1.0, 0.0)))

    def test_a_point_just_outside_its_lane_still_takes_it(self):
        self.assertEqual(rsl.lanelet_at({"west": WEST, "east": EAST}, (25.0, 0.2), (1.0, 0.0)), "east")
        self.assertIsNone(rsl.lanelet_at({"east": EAST}, (25.0, 1.5), (1.0, 0.0)))
        self.assertEqual(rsl.lanelet_at({"west": WEST}, (25.0, 2.0), (-1.0, 0.0)), "west")


if __name__ == "__main__":
    unittest.main()
