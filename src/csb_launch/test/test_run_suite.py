"""Unit tests for run_suite's scenario discovery and JUnit aggregation (no ROS needed).

    python3 -m pytest src/csb_launch/test      # or: python3 -m unittest discover src/csb_launch/test
"""

import importlib.machinery
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "run_suite"
_loader = importlib.machinery.SourceFileLoader("run_suite", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("run_suite", _loader)
rs = importlib.util.module_from_spec(_spec)
_loader.exec_module(rs)

PASS_ONE = """<?xml version="1.0"?>
<testsuites name="/x/scenario_test_runner" failures="0" errors="0" tests="1">
  <testsuite name="town01_ego_drive" failures="0" errors="0" tests="1">
    <testcase name="town01_ego_drive" />
  </testsuite>
</testsuites>
"""

# A YAML scenario with modifiers: one testcase per variant, one of them failed.
VARIANTS = """<?xml version="1.0"?>
<testsuites name="/x/scenario_test_runner" failures="1" errors="1" tests="3">
  <testsuite name="UC-ACC" failures="1" errors="1" tests="3">
    <testcase name="UC-ACC.0" />
    <testcase name="UC-ACC.1">
      <failure type="SimulationFailure" message="exitFailure" />
    </testcase>
    <testcase name="UC-ACC.2">
      <error type="AutowareError" message="engage timed out" />
    </testcase>
  </testsuite>
</testsuites>
"""


def write_junit(results_dir: Path, stem: str, text: str) -> None:
    d = results_dir / stem / "scenario_test_runner"
    d.mkdir(parents=True, exist_ok=True)
    (d / "result.junit.xml").write_text(text)


def make_result(results_dir: Path, stem: str, **kw) -> "rs.ScenarioResult":
    r = rs.ScenarioResult(stem=stem, path=Path(f"/s/{stem}.xosc"), output=results_dir / stem,
                          seconds=kw.pop("seconds", 10.0), **kw)
    rs.collect_result(r)
    return r


class AggregationTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_pass(self):
        write_junit(self.dir, "town01_ego_drive", PASS_ONE)
        r = make_result(self.dir, "town01_ego_drive", exit_code=0)
        self.assertTrue(r.passed)
        self.assertEqual([c.name for c in r.cases], ["town01_ego_drive"])
        self.assertTrue(rs.summary_line(r).startswith("[run_suite] PASS town01_ego_drive: 1/1"))

    def test_variants_each_become_a_testcase(self):
        write_junit(self.dir, "UC-ACC", VARIANTS)
        r = make_result(self.dir, "UC-ACC", exit_code=0)
        self.assertFalse(r.passed)
        self.assertEqual([c.kind for c in r.cases], ["pass", "failure", "error"])
        self.assertEqual(r.cases[1].type, "SimulationFailure")
        self.assertEqual(r.cases[2].message, "engage timed out")
        self.assertIn("FAIL UC-ACC: 1/3", rs.summary_line(r))

    def test_missing_junit_is_an_error_with_reason(self):
        r = make_result(self.dir, "town01_pedestrian", exit_code=1)
        self.assertFalse(r.passed)
        self.assertEqual(len(r.cases), 1)
        self.assertEqual(r.cases[0].kind, "error")
        self.assertEqual(r.cases[0].type, "NoVerdict")
        self.assertIn("exit code 1", r.cases[0].message)

    def test_timeout_without_junit(self):
        r = make_result(self.dir, "slow", exit_code=-9, timed_out=True, seconds=1800)
        self.assertEqual(r.cases[0].kind, "error")
        self.assertIn("timed out after 1800 s", r.cases[0].message)

    def test_timeout_keeps_finished_variants(self):
        write_junit(self.dir, "UC-ACC", PASS_ONE)
        r = make_result(self.dir, "UC-ACC", exit_code=-9, timed_out=True)
        self.assertEqual([c.kind for c in r.cases], ["pass", "error"])
        self.assertFalse(r.passed)

    def test_empty_and_corrupt_junit(self):
        write_junit(self.dir, "empty", '<?xml version="1.0"?><testsuites tests="0"/>')
        write_junit(self.dir, "corrupt", "<testsuites><testsuite>")
        e = make_result(self.dir, "empty", exit_code=0)
        c = make_result(self.dir, "corrupt", exit_code=0)
        self.assertIn("has no testcases", e.cases[0].message)
        self.assertIn("unreadable", c.cases[0].message)
        self.assertFalse(e.passed or c.passed)

    def test_suite_xml_counts_and_roundtrip(self):
        write_junit(self.dir, "a", PASS_ONE)
        write_junit(self.dir, "b", VARIANTS)
        results = [make_result(self.dir, "a", exit_code=0),
                   make_result(self.dir, "b", exit_code=0),
                   make_result(self.dir, "c", exit_code=2)]
        out = self.dir / "suite.junit.xml"
        rs.build_suite_xml(results).write(out, encoding="utf-8", xml_declaration=True)
        root = ET.parse(out).getroot()
        self.assertEqual(root.tag, "testsuites")
        self.assertEqual((root.get("tests"), root.get("failures"), root.get("errors")),
                         ("5", "1", "2"))
        suite = root.find("testsuite")
        cases = suite.findall("testcase")
        self.assertEqual([(c.get("classname"), c.get("name")) for c in cases],
                         [("a", "town01_ego_drive"), ("b", "UC-ACC.0"), ("b", "UC-ACC.1"),
                          ("b", "UC-ACC.2"), ("c", "c")])
        self.assertIsNotNone(cases[2].find("failure"))
        self.assertEqual(cases[4].find("error").get("type"), "NoVerdict")


class DiscoveryTest(unittest.TestCase):
    def test_collect_and_unique_stems(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "sub").mkdir()
            for name in ("b.xosc", "a.yaml", "notes.md", "sub/b.yml"):
                (d / name).write_text("x")
            found = rs.collect_scenarios([str(d)])
            self.assertEqual([p.relative_to(d).as_posix() for p in found],
                             ["a.yaml", "b.xosc", "sub/b.yml"])
            self.assertEqual(rs.unique_stems(found), ["a", "b", "b_2"])
            with self.assertRaises(FileNotFoundError):
                rs.collect_scenarios([str(d / "missing.xosc")])

    def test_args_split_at_double_dash(self):
        a = rs.parse_args(["x.xosc", "--output", "/r", "--domain", "7", "--",
                           "global_timeout:=900"])
        self.assertEqual(a.domain, 7)
        self.assertEqual(a.extra, ["global_timeout:=900"])
        self.assertEqual(a.timeout_per_scenario, 1800.0)

    def test_launch_command_carries_output_directory(self):
        cmd = rs.launch_command(Path("/s/a.xosc"), Path("/r/a"), ["port:=5555"])
        self.assertIn("scenario.launch.xml", cmd)
        self.assertIn("scenario:=/s/a.xosc", cmd)
        self.assertIn("output_directory:=/r/a", cmd)
        self.assertEqual(cmd[-1], "port:=5555")
        if cmd[0] == "play_launch":
            self.assertEqual(cmd[cmd.index("--enforce-rules") + 1], "off")


if __name__ == "__main__":
    unittest.main()
