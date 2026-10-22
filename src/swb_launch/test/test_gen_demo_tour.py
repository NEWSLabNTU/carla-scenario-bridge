"""Unit tests for scripts/gen_demo_tour.py, the demo tour (roadmap 033 steps 5 and 8). Generates
the tour from the repo's sites and ambient files and checks what a run relies on. No ROS:

    python3 -m pytest src/swb_launch/test/test_gen_demo_tour.py
"""

import importlib.machinery
import importlib.util
import unittest
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import yaml

_SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "gen_demo_tour.py"
_loader = importlib.machinery.SourceFileLoader("gen_demo_tour", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("gen_demo_tour", _loader)
gen = importlib.util.module_from_spec(_spec)
_loader.exec_module(gen)

SITES = yaml.safe_load(gen.SITES.read_text())
AMBIENT = yaml.safe_load(gen.AMBIENT.read_text()) if gen.AMBIENT.exists() else None


def parse(ambient=AMBIENT):
    return ET.fromstring(gen.tour(SITES, ambient))


class GenDemoTour(unittest.TestCase):
    def test_every_incident_composed(self):
        names = {g.get("name") for g in parse(None).iter("ManeuverGroup")}
        for g in ("ego_group", "i1", "i2_parked", "i2_ped", "i3", "i4_broken", "i4_oncoming", "i5", "i6"):
            self.assertIn(g, names)

    def test_names_unique_and_parameters_declared(self):
        for amb in (None, AMBIENT):
            root = parse(amb)
            for tag in ("ScenarioObject", "ManeuverGroup", "Event"):
                dup = [n for n, c in Counter(e.get("name") for e in root.iter(tag)).items() if c > 1]
                self.assertEqual(dup, [], tag)
            decl = {p.get("name") for p in root.iter("ParameterDeclaration")}
            refs = {e.get("parameterRef") for e in root.iter() if e.get("parameterRef")}
            self.assertLessEqual(refs, decl)
            # every entity an event names is declared
            ents = {o.get("name") for o in root.iter("ScenarioObject")}
            used = {e.get("entityRef") for e in root.iter() if e.get("entityRef")}
            self.assertLessEqual(used, ents)

    def test_single_stop_trigger(self):
        root = parse()
        kinds = Counter(a.get("type") for a in root.iter("CustomCommandAction") if a.get("type", "").startswith("exit"))
        self.assertEqual(kinds, Counter({"exitSuccess": 1, "exitFailure": 1}))
        # contacts are logged, never an exit: no event that holds a CollisionCondition exits
        for ev in root.iter("Event"):
            if list(ev.iter("CollisionCondition")):
                self.assertEqual([a for a in ev.iter("CustomCommandAction") if a.get("type", "").startswith("exit")], [])

    def test_ego_route_and_start(self):
        root = parse()
        wps = list(root.iter("Waypoint"))
        self.assertEqual(len(wps), len(gen.WAYPOINTS) + 1)
        perf = next(o for o in root.iter("ScenarioObject") if o.get("name") == "ego").find("Vehicle/Performance")
        self.assertGreater(float(perf.get("maxSpeed")), 11.11)

    @unittest.skipIf(AMBIENT is None, "no ambient file")
    def test_ambient_layer(self):
        root = parse()
        objs = {o.get("name"): o for o in root.iter("ScenarioObject")}
        for g in AMBIENT["groups"]:
            for a in g["actors"]:
                o = objs[a["name"]]
                want = {"cars": "Vehicle", "parked": "MiscObject", "walkers": "Pedestrian"}[g["kind"]]
                if g["kind"] == "walkers" and len(a["path"]) == 1:
                    want = "MiscObject"  # a person standing still: free for SSv2 (step 8)
                self.assertIsNotNone(o.find(want), a["name"])
        # parked cars and standing people never get a speed action (they are misc objects)
        parked = {a["name"] for g in AMBIENT["groups"] for a in g["actors"]
                  if g["kind"] == "parked" or (g["kind"] == "walkers" and len(a["path"]) == 1)}
        for grp in root.iter("ManeuverGroup"):
            actors = {e.get("entityRef") for e in grp.find("Actors")}
            if actors & parked:
                self.assertEqual(list(grp.iter("SpeedAction")), [], grp.get("name"))
        # an adult blueprint for every ambient person (0009-0014, 0048-0049 are children)
        for g in AMBIENT["groups"]:
            for a in g["actors"]:
                if g["kind"] == "walkers":
                    self.assertNotIn(int(a["bp"].rsplit(".", 1)[1]), (9, 10, 11, 12, 13, 14, 48, 49), a["name"])
        # ambient groups have no watchdog: only incident groups delete on the ego standing still
        for ev in root.iter("Event"):
            if ev.get("name", "").endswith("_watchdog") or ev.get("name", "").endswith("_clear"):
                self.assertTrue(ev.get("name").startswith("i"), ev.get("name"))


if __name__ == "__main__":
    unittest.main()
