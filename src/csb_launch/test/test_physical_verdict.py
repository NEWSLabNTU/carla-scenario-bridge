"""Unit tests for physical_verdict and run_suite --physical-verdict (no ROS, no CARLA).

    python3 -m pytest src/csb_launch/test
"""

import importlib.machinery
import importlib.util
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"


def _load(name):
    loader = importlib.machinery.SourceFileLoader(name, str(_SCRIPTS / name))
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


pv = _load("physical_verdict")
rs = _load("run_suite")

JUNIT = """<?xml version="1.0"?>
<testsuites name="/x/scenario_test_runner" failures="0" errors="0" tests="1">
  <testsuite name="town01_rear_contact" failures="0" errors="0" tests="1">
    <testcase name="town01_rear_contact" />
  </testsuite>
</testsuites>
"""

XOSC = """<?xml version="1.0"?>
<OpenSCENARIO>
  <ParameterDeclarations>{params}</ParameterDeclarations>
  <Entities>
    <ScenarioObject name="ego">
      <ObjectController><Controller name="Autoware"><Properties>
        <Property name="isEgo" value="true"/>
      </Properties></Controller></ObjectController>
    </ScenarioObject>
    <ScenarioObject name="npc_rear"/>
  </Entities>
</OpenSCENARIO>
"""

NO_EGO_XOSC = """<?xml version="1.0"?>
<OpenSCENARIO><ParameterDeclarations/><Entities/></OpenSCENARIO>
"""

YAML = """OpenSCENARIO:
  ParameterDeclarations:
    ParameterDeclaration:
      - name: csbExpectedContacts
        parameterType: string
        value: "Pedestrian0, static"
  Entities:
    ScenarioObject:
      - name: ego
        ObjectController:
          Controller:
            name: Autoware
"""


def contact(name="npc_rear", impulse=1843.2, static=False, frame=412):
    return {"other_id": None if static else 233, "name": None if static else name,
            "type": "" if static else "vehicle.tesla.model3", "static": static,
            "first_ssv2_frame": frame, "first_sim_time": 21.05, "count": 37,
            "max_impulse": impulse}


def report(start, contacts=(), complete=True, episode=1):
    return {"schema": 1, "episode": episode, "ego": "ego", "start_unix": start,
            "end_unix": start + 60, "complete": complete, "contacts": list(contacts)}


class Scratch(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.contacts = self.dir / "contacts"
        self.contacts.mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def write_report(self, r):
        path = self.contacts / f"{r['start_unix']:.3f}_e{r['episode']}.json"
        path.write_text(json.dumps(r))
        return path

    def write(self, name, text):
        path = self.dir / name
        path.write_text(text)
        return path


class LoadReports(Scratch):
    def test_window_selects_by_ego_spawn_time_oldest_first(self):
        for t in (100.0, 200.0, 300.0):
            self.write_report(report(t))
        got = pv.load_reports(self.contacts, 150.0, 300.0)
        self.assertEqual([r["start_unix"] for r in got], [200.0, 300.0])

    def test_no_bounds_means_the_newest_alone(self):
        for t in (100.0, 300.0, 200.0):
            self.write_report(report(t))
        self.assertEqual([r["start_unix"] for r in pv.load_reports(self.contacts)], [300.0])

    def test_unknown_schema_and_garbage_are_skipped(self):
        self.write_report(dict(report(100.0), schema=99))
        (self.contacts / "bad.json").write_text("{not json")
        self.assertEqual(pv.load_reports(self.contacts, 0, 1e12), [])

    def test_a_missing_directory_is_no_reports(self):
        self.assertEqual(pv.load_reports(self.dir / "nope", 0, 1), [])

    def test_iso_times_parse(self):
        self.assertEqual(pv.parse_time("1791683599.5"), 1791683599.5)
        self.assertEqual(pv.parse_time("2026-10-11T01:53:19.572Z"), 1791683599.572)


class Judge(unittest.TestCase):
    def test_no_contacts_passes(self):
        kind, _, msg = pv.judge([report(1.0)])
        self.assertEqual(kind, "pass", msg)

    def test_an_undeclared_contact_fails_and_names_it(self):
        kind, type_, msg = pv.judge([report(1.0, [contact()])])
        self.assertEqual((kind, type_), ("failure", "CarlaContact"))
        self.assertIn("'npc_rear'", msg)
        self.assertIn("SSv2 frame 412", msg)
        self.assertIn("1843.2", msg)

    def test_a_declared_contact_passes(self):
        kind, _, msg = pv.judge([report(1.0, [contact()])], expected={"npc_rear"})
        self.assertEqual(kind, "pass", msg)

    def test_static_world_needs_its_own_declaration(self):
        r = [report(1.0, [contact(static=True)])]
        self.assertEqual(pv.judge(r, expected={"npc_rear"})[0], "failure")
        self.assertIn("static world", pv.judge(r)[2])
        self.assertEqual(pv.judge(r, expected={"static"})[0], "pass")

    def test_min_impulse_filters(self):
        r = [report(1.0, [contact(impulse=5.0)])]
        self.assertEqual(pv.judge(r, min_impulse=10.0)[0], "pass")
        self.assertEqual(pv.judge(r, min_impulse=5.0)[0], "failure")

    def test_no_report_is_an_error_unless_there_is_no_ego(self):
        kind, type_, _ = pv.judge([])
        self.assertEqual((kind, type_), ("error", "NoContactReport"))
        kind, _, msg = pv.judge([], has_ego=False)
        self.assertEqual((kind, msg), ("pass", "no ego in scenario"))

    def test_an_incomplete_report_is_an_error(self):
        kind, type_, msg = pv.judge([report(1.0), report(2.0, complete=False)])
        self.assertEqual((kind, type_), ("error", "IncompleteContactReport"), msg)

    def test_a_contact_outranks_an_incomplete_report(self):
        kind, _, _ = pv.judge([report(1.0, [contact()], complete=False)])
        self.assertEqual(kind, "failure")

    def test_variants_are_numbered_in_the_message(self):
        _, _, msg = pv.judge([report(1.0), report(2.0, [contact()])])
        self.assertIn("run 2/2", msg)


class ScenarioFiles(Scratch):
    def test_xosc_expected_contacts_and_ego(self):
        param = ('<ParameterDeclaration name="csbExpectedContacts" parameterType="string" '
                 'value="npc_rear, static"/>')
        path = self.write("s.xosc", XOSC.format(params=param))
        self.assertEqual(pv.expected_contacts(path), {"npc_rear", "static"})
        self.assertTrue(pv.scenario_has_ego(path))

    def test_xosc_without_the_parameter_expects_nothing(self):
        path = self.write("s.xosc", XOSC.format(params=""))
        self.assertEqual(pv.expected_contacts(path), set())

    def test_yaml_expected_contacts_and_ego(self):
        path = self.write("s.yaml", YAML)
        self.assertEqual(pv.expected_contacts(path), {"Pedestrian0", "static"})
        self.assertTrue(pv.scenario_has_ego(path))

    def test_no_ego(self):
        self.assertFalse(pv.scenario_has_ego(self.write("n.xosc", NO_EGO_XOSC)))


class ApplyToJunit(Scratch):
    def cases(self, path):
        root = ET.parse(path).getroot()
        return root, root.find("testsuite").findall("testcase")

    def test_a_failure_is_added_and_counts_follow(self):
        junit = self.write("result.junit.xml", JUNIT)
        pv.apply_to_junit(junit, "failure", "CarlaContact", "line one\nline two")
        root, cases = self.cases(junit)
        self.assertEqual([c.get("name") for c in cases], ["town01_rear_contact", "carla_contact"])
        self.assertEqual(cases[0].find("failure"), None, "SSv2's testcase untouched")
        fail = cases[1].find("failure")
        self.assertEqual(fail.get("message"), "line one")
        self.assertEqual(fail.text, "line one\nline two")
        for node in (root, root.find("testsuite")):
            self.assertEqual((node.get("tests"), node.get("failures"), node.get("errors")),
                             ("2", "1", "0"))

    def test_applying_twice_replaces(self):
        junit = self.write("result.junit.xml", JUNIT)
        pv.apply_to_junit(junit, "failure", "CarlaContact", "x")
        pv.apply_to_junit(junit, "pass", "", "ok")
        root, cases = self.cases(junit)
        self.assertEqual(len(cases), 2)
        self.assertEqual(root.get("failures"), "0")


class Cli(Scratch):
    def test_end_to_end_failure_then_declared_pass(self):
        junit = self.write("result.junit.xml", JUNIT)
        self.write_report(report(1000.0, [contact()]))
        scen = self.write("s.xosc", XOSC.format(params=""))
        argv = [str(junit), "--contacts", str(self.contacts), "--scenario", str(scen)]
        self.assertEqual(pv.main(argv), 1)
        param = ('<ParameterDeclaration name="csbExpectedContacts" parameterType="string" '
                 'value="npc_rear"/>')
        scen.write_text(XOSC.format(params=param))
        self.assertEqual(pv.main(argv), 0)

    def test_missing_junit_is_a_usage_error(self):
        self.assertEqual(pv.main([str(self.dir / "none.xml")]), 2)


class RunSuiteJoin(Scratch):
    def result(self, start, end, params=""):
        scen = self.write("town01_rear_contact.xosc", XOSC.format(params=params))
        r = rs.ScenarioResult(stem="town01_rear_contact", path=scen, output=self.dir)
        r.started_unix, r.ended_unix = start, end
        r.cases = [rs.Case(name="town01_rear_contact", classname="town01_rear_contact")]
        return r

    def test_the_run_window_picks_this_runs_reports(self):
        self.write_report(report(50.0, [contact()]))   # an earlier scenario's
        self.write_report(report(150.0))
        r = self.result(100.0, 200.0)
        rs.add_physical_verdict(r, self.contacts, pv=pv)
        self.assertEqual([c.kind for c in r.cases], ["pass", "pass"])
        self.assertEqual(r.cases[1].name, "carla_contact")

    def test_ssv2_pass_and_carla_contact_fail_side_by_side(self):
        self.write_report(report(150.0, [contact()]))
        r = self.result(100.0, 200.0)
        rs.add_physical_verdict(r, self.contacts, pv=pv)
        self.assertEqual([c.kind for c in r.cases], ["pass", "failure"])
        self.assertFalse(r.passed)
        suite = rs.build_suite_xml([r]).getroot()
        self.assertEqual((suite.get("tests"), suite.get("failures")), ("2", "1"))

    def test_a_scenario_preflight_refused_gets_no_testcase(self):
        r = self.result(None, None)
        rs.add_physical_verdict(r, self.contacts, pv=pv)
        self.assertEqual(len(r.cases), 1)


if __name__ == "__main__":
    unittest.main()
