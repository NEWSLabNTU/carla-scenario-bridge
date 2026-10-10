"""Unit tests for run_suite's scenario discovery and JUnit aggregation (no ROS needed).

    python3 -m pytest src/csb_launch/test      # or: python3 -m unittest discover src/csb_launch/test
"""

import importlib.machinery
import importlib.util
import json
import socket
import tempfile
import threading
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
        self.assertIn("TIMEOUT after 1800 s", r.cases[0].message)

    def test_timeout_names_the_cap_on_the_console_line(self):
        r = make_result(self.dir, "slow", exit_code=-9, timed_out=True, seconds=3612,
                        timeout=3600)
        line = rs.summary_line(r)
        self.assertIn("FAIL slow", line)
        self.assertIn("TIMEOUT after 3600 s", line)

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
        self.assertEqual(a.timeout_per_scenario, 0.0)
        self.assertEqual(a.relay, "tcp://localhost:5560")
        self.assertFalse(a.no_preflight)
        b = rs.parse_args(["x.xosc", "-o", "/r", "--no-preflight", "--relay", "tcp://h:1"])
        self.assertTrue(b.no_preflight)
        self.assertEqual(b.relay, "tcp://h:1")

    def test_launch_command_carries_output_directory(self):
        cmd = rs.launch_command(Path("/s/a.xosc"), Path("/r/a"), ["port:=5555"])
        self.assertIn("scenario.launch.xml", cmd)
        self.assertIn("scenario:=/s/a.xosc", cmd)
        self.assertIn("output_directory:=/r/a", cmd)
        self.assertEqual(cmd[-1], "port:=5555")
        if cmd[0] == "play_launch":
            self.assertEqual(cmd[cmd.index("--enforce-rules") + 1], "off")


EGO_XOSC = """<OpenSCENARIO><Entities>
  <ScenarioObject name="ego"><Vehicle/><ObjectController>
    <Controller name="Autoware"><Properties/></Controller></ObjectController></ScenarioObject>
  <ScenarioObject name="npc"><Vehicle/><ObjectController>
    <Controller name=""><Properties/></Controller></ObjectController></ScenarioObject>
  <ScenarioObject name="bg_av_1"><Vehicle/><ObjectController>
    <Controller name="agent"><Properties/></Controller></ObjectController></ScenarioObject>
</Entities></OpenSCENARIO>"""

NO_EGO_XOSC = """<OpenSCENARIO><Entities>
  <ScenarioObject name="npc"><Vehicle/></ScenarioObject>
</Entities></OpenSCENARIO>"""

EGO_YAML = """OpenSCENARIO:
  Entities:
    ScenarioObject:
      - name: car
        ObjectController:
          Controller:
            name: ''
            Properties:
              Property:
                - name: isEgo
                  value: 'true'
      - name: Npc1
        ObjectController:
          Controller:
            name: ''
            Properties:
              Property: []
"""


class FakeRelay:
    """A relay that answers commander queries from a fixed set of registered entities."""

    def __init__(self, registered):
        self.registered = registered
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen()
        self.url = f"tcp://127.0.0.1:{self.sock.getsockname()[1]}"
        self.queries = []
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            with conn:
                # Separate reader and writer: a write on a "rw" text file drops the lines
                # it has already read ahead.
                f, w = conn.makefile("r"), conn.makefile("w")
                for line in f:
                    m = json.loads(line)
                    assert m["v"] == 1
                    if m["type"] == "register":
                        reply = {"type": "registered", "entity": ""}
                    else:
                        self.queries.append(m["entity"])
                        ok = m["entity"] in self.registered
                        reply = {"type": "reply", "id": m["id"], "status": "ok" if ok
                                 else "failed", "registered": ok}
                        if ok:
                            reply["state"] = {"phase": "idle"}
                    w.write(json.dumps({**reply, "v": 1}) + "\n")
                    w.flush()

    def close(self):
        self.sock.close()


class PreflightTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def scenario(self, name, text):
        p = self.dir / name
        p.write_text(text)
        return p

    def test_required_agents(self):
        self.assertEqual(rs.required_agents(self.scenario("a.xosc", EGO_XOSC)),
                         ["ego", "bg_av_1"])
        self.assertEqual(rs.required_agents(self.scenario("b.xosc", NO_EGO_XOSC)), [])
        # The relay serves the ego under its own entity, whatever the scenario calls it.
        self.assertEqual(rs.required_agents(self.scenario("c.yaml", EGO_YAML), "hero"),
                         ["hero"])
        self.assertEqual(rs.required_agents(self.scenario("d.xosc", "<broken")), [])

    def test_installed_starter_scenarios_need_their_agents(self):
        root = Path(__file__).resolve().parents[2] / "csb_examples" / "scenarios"
        if not root.is_dir():
            self.skipTest("csb_examples not beside csb_launch")
        for f in sorted(root.rglob("*")):
            if f.suffix in rs.SCENARIO_SUFFIXES:
                want = {"town01_two_av": ["ego", "bg_av_1"],
                        "town02_episode_change": []}.get(f.stem, ["ego"])  # no entities
                self.assertEqual(rs.required_agents(f), want, f.name)

    def test_relay_address(self):
        self.assertEqual(rs.relay_address("tcp://localhost:5560"), ("localhost", 5560))
        self.assertEqual(rs.relay_address("10.0.0.2:7"), ("10.0.0.2", 7))
        with self.assertRaises(ValueError):
            rs.relay_address("udp://h:1")

    def test_preflight_registered_and_missing(self):
        relay = FakeRelay({"ego"})
        try:
            xosc = self.scenario("a.xosc", EGO_XOSC)
            err = rs.preflight(xosc, relay.url)
            self.assertIn("'bg_av_1'", err)
            self.assertNotIn("'ego'", err)
            self.assertEqual(relay.queries, ["ego", "bg_av_1"])
            relay.registered.add("bg_av_1")
            self.assertIsNone(rs.preflight(xosc, relay.url))
        finally:
            relay.close()

    def test_preflight_unreachable_relay(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()  # nothing listens there now
        err = rs.preflight(self.scenario("a.xosc", EGO_XOSC), f"tcp://127.0.0.1:{port}")
        self.assertIn("unreachable", err)

    def test_run_one_fails_fast_without_launching(self):
        relay = FakeRelay(set())
        try:
            xosc = self.scenario("a.xosc", EGO_XOSC)
            r = rs.run_one(xosc, "a", self.dir / "out", [], 0, {}, relay=relay.url)
        finally:
            relay.close()
        self.assertFalse(r.passed)
        self.assertEqual([(c.kind, c.type) for c in r.cases], [("error", "Preflight")])
        self.assertIn("no vehicle agent registered", rs.summary_line(r))
        self.assertFalse((self.dir / "out" / "a" / "launch.log").exists())


if __name__ == "__main__":
    unittest.main()
