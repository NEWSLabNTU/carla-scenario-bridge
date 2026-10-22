"""Traffic signals in the Isaac world (roadmap 044 B); no Isaac needed."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaac_world import signals  # noqa: E402

TABLE = [
    {"way_id": 43763, "regulatory_element_ids": [43856], "opendrive_id": "377",
     "position": (102.72, -52.85, 0.1)},
    {"way_id": 43766, "regulatory_element_ids": [43856], "opendrive_id": "376",
     "position": (85.73, -45.78, 0.1)},
    {"way_id": 43800, "regulatory_element_ids": [43900], "opendrive_id": "999",
     "position": (500.0, 500.0, 0.0)},
]


class SignalsTest(unittest.TestCase):
    def test_lamps_match_by_signal_id_then_by_position(self):
        lamps = [("/World/env/Intersections/TrafficLight11", "377", (102.72, -52.85, 0.1)),
                 # no id: matched by position, 0.4 m off
                 ("/World/env/Intersections/TrafficLight12", None, (85.73, -45.38, 0.1)),
                 ("/World/env/Intersections/TrafficLight99", "123", (0.0, 0.0, 0.0))]
        by_id, how, unmatched = signals.match_lamps(TABLE, lamps)
        self.assertEqual(by_id["377"], "/World/env/Intersections/TrafficLight11")
        self.assertEqual(how, {"377": "id", "376": "position"})
        self.assertEqual(unmatched, ["999"])

    def test_a_lamp_is_used_once(self):
        lamps = [("/a", None, (102.72, -52.85, 0.1))]
        twins = [dict(TABLE[0]), dict(TABLE[0], opendrive_id="378")]
        by_id, _, unmatched = signals.match_lamps(twins, lamps)
        self.assertEqual(list(by_id.values()), ["/a"])
        self.assertEqual(unmatched, ["378"])

    def test_a_group_shows_its_most_restrictive_colour(self):
        g = dict(signals.groups(TABLE, {"377": "GREEN", "376": "RED"}))
        self.assertEqual(g, {43856: "RED", 43900: "OFF"})
        g = dict(signals.groups(TABLE, {"377": "GREEN"}))
        self.assertEqual(g[43856], "GREEN")


if __name__ == "__main__":
    unittest.main()
