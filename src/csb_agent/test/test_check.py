"""`csb-agent check` passes a good agent, names the rule a bad one breaks; `new` passes it."""

import sys
import time

import csb_agent
from csb_agent import protocol as P
from csb_agent.check import load_class, run_check
from csb_agent.cli import main as cli
from csb_agent.skeleton import create


class Good(csb_agent.Agent):
    def on_reset(self, ep):
        pass

    def on_goal(self, goal, waypoints):
        self.report(P.DRIVING)

    def on_speed_limit(self, mps):
        pass

    def on_stop(self):
        pass


class NeverReady(Good):
    def on_reset(self, ep):
        self.report(P.INITIALIZING, detail="localization never converges")


class SlowStop(Good):
    def on_stop(self):
        time.sleep(3.5)


class V1Style(csb_agent.Agent):
    """No reset hook: the boundary is teleported, and the SDK answers it."""

    def on_goal(self, goal, waypoints):
        pass

    def on_speed_limit(self, mps):
        pass

    def on_stop(self):
        pass


def failed_rules(report):
    return [rule for ok, rule, _ in report.results if not ok]


def test_a_good_agent_passes():
    report = run_check(Good)
    assert report.passed, report.text()


def test_an_agent_that_never_gets_ready_fails_the_reset_rule():
    report = run_check(NeverReady)
    assert failed_rules(report) == ["reset: IDLE before the reply",
                                    "second reset: IDLE before the reply"], report.text()


def test_a_slow_stop_fails_its_window():
    report = run_check(SlowStop)
    assert "stop answered within 2.5 s" in failed_rules(report), report.text()


def test_an_agent_without_reset_is_checked_with_teleported():
    report = run_check(V1Style)
    assert report.passed, report.text()
    assert any("teleported answered OK" in rule for _, rule, _ in report.results)


def test_new_generates_a_project_that_passes_check(tmp_path, capsys):
    root, spec = create("road-runner", tmp_path)
    assert (root / "pyproject.toml").exists() and (root / "agent.yaml").exists()
    sys.path.insert(0, str(root))
    try:
        assert load_class(spec).__name__ == "RoadRunnerAgent"
        assert cli(["check", spec, "--config", str(root / "agent.yaml")]) == 0
    finally:
        sys.path.remove(str(root))
    assert "PASS: 16/16 rules" in capsys.readouterr().out


def test_check_loads_a_file_path(tmp_path):
    path = tmp_path / "solo.py"
    path.write_text("import csb_agent\n\nclass Solo(csb_agent.Agent):\n    pass\n")
    assert load_class(f"{path}:Solo").__name__ == "Solo"
