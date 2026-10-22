"""Unit tests for scripts/gen_demo_incidents.py, the demo tour's incident templates (roadmap 033
step 4d). Generates every template from the repo's sites file and from a step 4b-shaped one
(one parked car, no speeds), and checks what the 40 km/h tour relies on. No ROS:

    python3 -m pytest src/swb_launch/test/test_gen_demo_incidents.py
"""

import copy
import importlib.machinery
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

_SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "gen_demo_incidents.py"
_loader = importlib.machinery.SourceFileLoader("gen_demo_incidents", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("gen_demo_incidents", _loader)
gen = importlib.util.module_from_spec(_spec)
_loader.exec_module(gen)

SITES = yaml.safe_load(gen.SITES.read_text())


def parse(i, sites=SITES):
    return ET.fromstring(gen.template(i, sites))


def objects(root):
    return {o.get("name"): o for o in root.iter("ScenarioObject")}


class GenDemoIncidents(unittest.TestCase):
    def test_every_incident_generates(self):
        self.assertEqual(sorted(SITES["incidents"]), [1, 2, 3, 4, 5, 6])
        for i in range(1, 7):
            root = parse(i)
            names = [g.get("name") for g in root.iter("ManeuverGroup")]
            self.assertIn("ego_group", names, i)
            # every ParameterRef is declared
            decl = {p.get("name") for p in root.iter("ParameterDeclaration")}
            refs = {e.get("parameterRef") for e in root.iter() if e.get("parameterRef")}
            self.assertLessEqual(refs, decl, i)

    def test_ego_max_speed_above_cap(self):
        for i in range(1, 7):
            perf = objects(parse(i))["ego"].find("Vehicle/Performance")
            self.assertEqual(float(perf.get("maxSpeed")), gen.EGO_MAX_SPEED)
            self.assertGreater(gen.EGO_MAX_SPEED, 11.11)

    def test_parked_vehicles_are_misc_objects(self):
        obj = objects(parse(2))
        for name, bp in (("i2_van", "vehicle.mercedes.sprinter"), ("i2_parked", "vehicle.tesla.model3")):
            misc = obj[name].find("MiscObject")
            self.assertIsNotNone(misc, name)
            self.assertEqual(misc.get("model3d"), bp)
            dims = misc.find("BoundingBox/Dimensions")
            self.assertAlmostEqual(float(dims.get("length")), gen.BLUEPRINT_BOX[bp][0])
        self.assertIsNotNone(obj["i2_ped"].find("Pedestrian"))
        # no speed action on a misc object
        root = parse(2)
        grp = next(g for g in root.iter("ManeuverGroup") if g.get("name") == "i2_parked")
        self.assertEqual(list(grp.iter("SpeedAction")), [])

    def test_step4b_sites_still_generate(self):
        old = copy.deepcopy(SITES["incidents"])
        del old[2]["van"]
        for k in ("lead_mps", "brake_gap_m"):
            old[1].pop(k)
        old[6].pop("cyclist_mps")
        obj = objects(parse(2, {"incidents": old}))
        self.assertNotIn("i2_van", obj)
        self.assertIn("i2_parked", obj)
        text = gen.template(1, {"incidents": old})
        self.assertIn('value="3.0"', text)  # the step 4b lead speed

    def test_scalars_from_the_sites_file(self):
        s = copy.deepcopy(SITES)
        s["incidents"][1]["lead_mps"] = 7.25
        s["incidents"][4]["oncoming_mps"] = 6.75
        self.assertIn('<AbsoluteTargetSpeed value="7.25" />', gen.template(1, s))
        self.assertIn('<AbsoluteTargetSpeed value="6.75" />', gen.template(4, s))

    def test_incident_6_installed(self):
        with tempfile.TemporaryDirectory() as d:
            for i in range(1, 7):
                (Path(d) / gen.FILES[i]).write_text(gen.template(i, SITES))
            self.assertTrue((Path(d) / "town01_i6_cyclist_inside.xosc").exists())
        self.assertTrue((gen.DEFAULT_OUT / "town01_i6_cyclist_inside.xosc").exists())


if __name__ == "__main__":
    unittest.main()
