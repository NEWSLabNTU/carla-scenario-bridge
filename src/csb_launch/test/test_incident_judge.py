"""Unit tests for incident_judge, the demo-tour incident judge (roadmap 033 step 4).

No ROS, no CARLA:

    python3 -m pytest src/csb_launch/test/test_incident_judge.py
"""

import importlib.machinery
import importlib.util
import json
import sys
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(SCRIPTS))

import gap_trace as gt  # noqa: E402

_loader = importlib.machinery.SourceFileLoader("incident_judge", str(SCRIPTS / "incident_judge"))
_spec = importlib.util.spec_from_loader("incident_judge", _loader)
ij = importlib.util.module_from_spec(_spec)
_loader.exec_module(ij)

BOX = {"location": [0.0, 0.0, 0.7], "extent": [2.25, 1.0, 0.7]}


def entity(name, kind="vehicle"):
    return {"type": "entity", "frame": 0, "name": name, "kind": kind,
            "declared": {"center": [0.0, 0.0, 0.9], "dimensions": [4.5, 2.0, 1.8]},
            "origin_offset": [0.0, 0.0, 0.0], "actor_id": 100 + len(name), "blueprint": "vehicle.tesla.model3",
            "blueprint_box": BOX}


def state(x, v, y=0.0):
    p = [x, y, 0.0, 0.0, 0.0, 0.0]
    return {"ssv2": p, "carla": p, "velocity": [v, 0.0, 0.0]}


def trace_lines(frames, actors=("i1_lead",), contacts=()):
    out = [{"type": "episode", "schema": 1, "episode": 1, "start_unix": 0.0, "npc_motion": "tracked"},
           entity("ego", "ego"), *[entity(a) for a in actors]]
    for k, (t, ents) in enumerate(frames, start=1):
        out.append({"type": "frame", "frame": k, "sim_time": t, "entities": ents})
    out.extend(contacts)
    out.append({"type": "end", "frame": len(frames), "sim_time": frames[-1][0], "frames": len(frames),
                "dropped_frames": 0, "complete": True})
    return [json.dumps(o) for o in out]


def lead_brake_frames(stop_gap):
    """ego cruises at 4 m/s behind a lead at 3 m/s; the lead brakes at 6 m/s2 at t=2 and the
    ego stops `stop_gap` m (box to box) behind it, both standing from t=4"""
    frames, dt = [], 0.05
    ego_x, lead_x, ev, lv = 0.0, 20.0, 4.0, 3.0
    lead_stop = None
    for k in range(int(8 / dt)):
        t = round(k * dt, 2)
        if t >= 2.0:
            lv = max(0.0, lv - 6.0 * dt)
        lead_x += lv * dt
        if lv == 0.0 and lead_stop is None:
            lead_stop = lead_x
        target = (lead_stop if lead_stop is not None else lead_x + 30) - 4.5 - stop_gap
        if ego_x + ev * dt >= target:
            ego_x, ev = target, 0.0
        else:
            ego_x += ev * dt
        frames.append((t, {"ego": state(ego_x, ev), "i1_lead": state(lead_x, lv)}))
    return frames


def context(*events):
    return [json.dumps({"ev": "state", "sim": t, "element": f"incident/act/g/m/{n}", "to": "completeState"})
            for n, t in events]


class IncidentJudge(unittest.TestCase):
    def judge(self, lines, incident, ctx, **kw):
        trace = gt.load_trace(lines)
        done, _ = ij.events_from_context(ctx)
        return ij.judge(trace, incident, done, **kw)

    def test_lead_brake_pass(self):
        m = self.judge(trace_lines(lead_brake_frames(3.0)), 1, context(("i1_start", 0.5), ("i1_lead_brake", 2.0)))
        self.assertEqual(m["verdict"], "PASS", m["reasons"])
        self.assertAlmostEqual(m["stop_gap"], 3.0, places=1)
        self.assertGreaterEqual(m["lead_peak_decel"], 5.5)

    def test_lead_brake_short_gap_fails(self):
        m = self.judge(trace_lines(lead_brake_frames(1.2)), 1, context(("i1_lead_brake", 2.0)))
        self.assertEqual(m["verdict"], "FAIL")
        self.assertTrue(any("stop gap" in r for r in m["reasons"]))

    def test_contact_and_watchdog_fail(self):
        contact = {"type": "contact", "frame": 10, "carla_frame": 10, "sim_time": 0.5, "ego_id": 103,
                   "other_id": 107, "name": "i1_lead", "other_type": "vehicle.tesla.model3",
                   "other_kind": "vehicle", "impulse": 50.0}
        m = self.judge(trace_lines(lead_brake_frames(3.0), contacts=[contact]), 1,
                       context(("i1_lead_brake", 2.0), ("i1_watchdog", 7.0)))
        self.assertEqual(m["verdict"], "FAIL")
        self.assertEqual([c["actor"] for c in m["contacts_carla"]], ["i1_lead"])
        self.assertEqual(m["watchdog"], ["i1_watchdog"])

    def test_red_light(self):
        # ego drives east at 4 m/s, stands with its front 1.5 m short of a stop line at x 40
        # from t~9 to t=15 (red 2..14), then goes on
        frames, x = [], 0.0
        for k in range(int(24 / 0.05)):
            t = round(k * 0.05, 2)
            cap = 40 - 1.5 - 2.25
            if t < 15.0:
                x = min(x + 4.0 * 0.05, cap)
                v = 0.0 if x >= cap else 4.0
            else:
                v = 4.0
                x += v * 0.05
            frames.append((t, {"ego": state(x, v)}))
        ctx = context(("i3_red", 2.0), ("i3_green", 14.0))
        m = self.judge(trace_lines(frames, actors=()), 3, ctx, stop_line=(40.0, 0.0, 0.0))
        self.assertEqual(m["verdict"], "PASS", m["reasons"])
        self.assertAlmostEqual(m["stop_line_gap_m"], 1.5, places=1)
        m = self.judge(trace_lines(frames, actors=()), 3, ctx, stop_line=(36.0, 0.0, 0.0))
        self.assertEqual(m["verdict"], "FAIL")

    def test_staged_clear_is_not_degraded(self):
        frames = []
        for k in range(400):
            t = round(k * 0.05, 2)
            v = 0.0 if 2.0 <= t < 12.0 else 3.0
            frames.append((t, {"ego": state(k * 0.05 * v if t < 2 else (6.0 if t < 12 else 6.0 + (t - 12) * 3), v),
                               "i4_broken": state(14.0, 0.0)}))
        m = self.judge(trace_lines(frames, actors=("i4_broken",)), 4,
                       context(("i4_broken_start", 0.5), ("i4_broken_clear", 12.0)))
        self.assertEqual(m["verdict"], "PASS", m["reasons"])
        self.assertGreaterEqual(m["ego_wait_s"], 9.0)


if __name__ == "__main__":
    unittest.main()
