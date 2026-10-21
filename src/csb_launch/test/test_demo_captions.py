"""Unit tests for scripts/demo_captions (roadmap 033 step 6): event diffing on
/simulation/context snapshots, per-incident numbers, the closing caption. No ROS:

    python3 -m pytest src/csb_launch/test/test_demo_captions.py
"""

import importlib.machinery
import importlib.util
import unittest
from pathlib import Path

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "demo_captions"
_loader = importlib.machinery.SourceFileLoader("demo_captions", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("demo_captions", _loader)
cap = importlib.util.module_from_spec(_spec)
_loader.exec_module(cap)


def snapshot(states):
    """a /simulation/context-shaped JSON with one maneuver holding `states` {event: state}"""
    return {"OpenSCENARIO": {"Storyboard": {"Story": [{"Act": [{"ManeuverGroup": [{"Maneuver": [{
        "Event": [{"name": n, "currentState": s, "currentExecutionCount": 1 if s == "completeState" else 0}
                  for n, s in states.items()]}]}]}]}]}}}


def box(x, y, yaw=0.0, length=4.5, width=2.0, speed=0.0):
    return (x, y, yaw, length, width, 0.0, 0.0, speed)


class DemoCaptions(unittest.TestCase):
    def setUp(self):
        self.out = []
        self.c = cap.Captions(lambda h, d, col: self.out.append((h, d)))

    def test_box_gap(self):
        self.assertAlmostEqual(cap.box_gap(box(0, 0)[:7], box(10, 0)[:7]), 5.5, places=6)
        self.assertEqual(cap.box_gap(box(0, 0)[:7], box(1, 0)[:7]), 0.0)

    def test_an_event_counts_once(self):
        st = {"i2_ped_start": "standbyState", "i2_ped_end": "standbyState"}
        self.c.on_context(snapshot(st), 0.0)
        self.assertEqual(self.out, [])
        st["i2_ped_start"] = "completeState"
        self.c.on_context(snapshot(st), 10.0)
        self.c.on_context(snapshot(st), 10.05)  # the same state again: no second caption
        self.assertEqual(len(self.out), 1)
        self.assertIn("Pedestrian darts out", self.out[0][0])

    def test_incident_numbers(self):
        self.c.on_context(snapshot({"i2_ped_start": "completeState"}), 10.0)
        # the ego closes in, stops 5 m short of the walker (box to box), stands 3 s
        for k in range(61):
            t = 10.0 + 0.05 * k
            v = 5.0 if k < 20 else 0.0
            self.c.on_status({"ego": box(0, 0, speed=v), "i2_ped": box(7.55, 0, length=0.6, width=0.6),
                              "i2_van": box(3.0, -3.0)}, t)
        self.c.on_context(snapshot({"i2_ped_start": "completeState", "i2_ped_end": "completeState"}), 14.0)
        head, detail = self.out[-1]
        self.assertIn("Pedestrian", head)
        self.assertIn("stopped 5.0 m short", detail)  # the van beside the ego is not the gap

    def test_watchdog_and_closing_caption(self):
        self.c.on_context(snapshot({"i5_start": "completeState"}), 1.0)
        self.c.on_context(snapshot({"i5_start": "completeState", "i5_watchdog": "completeState"}), 30.0)
        self.assertIn("watchdog", self.out[-1][1])
        self.c.on_context(snapshot({"i5_start": "completeState", "i5_watchdog": "completeState",
                                    "goal_reached": "completeState"}), 300.0)
        self.assertIn("Tour complete", self.out[-1][0])
        self.assertIn("0 of 1 incidents handled", self.out[-1][1])

    def test_red_light(self):
        self.c.on_context(snapshot({"i3_red": "completeState"}), 100.0)
        for k in range(200):
            self.c.on_status({"ego": box(0, 0, speed=0.0)}, 100.0 + 0.05 * k)
        self.c.on_context(snapshot({"i3_red": "completeState", "i3_green": "completeState"}), 110.0)
        self.assertIn("went on green after 10 s", self.out[-1][1])


if __name__ == "__main__":
    unittest.main()
