"""csb_agent.Agent and the client's command dispatch, without sockets."""

import pytest

import csb_agent
from csb_agent import protocol as P
from csb_agent.client import Client

POSE = {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


class Minimal(csb_agent.Agent):
    def on_goal(self, goal, waypoints):
        self.goal = goal

    def on_stop(self):
        self.stopped = True

    def on_speed_limit(self, mps):
        self.limit = mps


class Full(Minimal):
    def __init__(self, config=None):
        super().__init__(config)
        self.calls = []

    def on_reset(self, ep):
        self.calls.append(("reset", ep))

    def on_goal(self, goal, waypoints, allow_goal_modification=False):
        self.calls.append(("goal", goal, waypoints, allow_goal_modification))
        self.report(P.DRIVING)

    def on_episode_end(self, ep, result):
        self.calls.append(("end", ep, result))

    def on_restart(self):
        raise csb_agent.Failed("cannot restart")


def client(agent):
    c = Client(agent, "127.0.0.1", 1)  # never connected: execute() needs no socket
    return c


def test_capabilities_follow_the_overridden_hooks():
    assert csb_agent.Agent().capabilities() == list(P.REQUIRED_COMMANDS)
    caps = Full().capabilities()
    assert P.RESET in caps and P.RESTART in caps and P.COOPERATE not in caps


def test_a_hook_not_overridden_is_unsupported():
    c = client(Minimal())
    assert c.execute(P.RESET, {"pose": POSE})[0] == P.UNSUPPORTED
    assert c.execute(P.COOPERATE, {"module": "X", "command": "ACTIVATE"})[0] == P.UNSUPPORTED


def test_teleported_without_reset_is_an_event():
    a = Minimal()
    assert client(a).execute(P.TELEPORTED, {"pose": POSE}) == (P.OK, "nothing to do")
    assert a.phase == P.IDLE


def test_teleported_with_reset_runs_it_with_the_pose():
    a = Full()
    assert client(a).execute(P.TELEPORTED, {"pose": POSE})[0] == P.OK
    ((kind, ep),) = a.calls
    assert kind == "reset" and ep.pose == POSE and ep.episode == ""


def test_reset_carries_the_episode_and_leaves_idle():
    a = Full()
    a.report(P.DRIVING)
    status, _ = client(a).execute(P.RESET, {"episode": "e1", "scenario": "s", "map": "Town01",
                                            "pose": POSE, "parameters": {"v": 3}})
    assert status == P.OK and a.phase == P.IDLE
    ep = a.calls[0][1]
    assert (ep.episode, ep.scenario, ep.map, ep.parameters) == ("e1", "s", "Town01", {"v": 3})
    assert a.episode is ep


def test_default_phases_unless_the_hook_reported():
    a = Minimal()
    c = client(a)
    c.execute(P.SET_GOAL, {"goal": POSE})
    assert a.phase == P.PLANNING
    c.execute(P.STOP, {})
    assert a.phase == P.STOPPED
    c.execute(P.CLEAR_GOAL, {})  # defaults to on_stop, then IDLE
    assert a.phase == P.IDLE
    f = Full()
    client(f).execute(P.SET_GOAL, {"goal": POSE})
    assert f.phase == P.DRIVING  # the hook's own report wins


def test_goal_options_reach_hooks_that_accept_them():
    a, m = Full(), Minimal()
    args = {"goal": POSE, "waypoints": [POSE], "allow_goal_modification": True,
            "segments": [{"id": 1}]}
    assert client(a).execute(P.SET_GOAL, args)[0] == P.OK
    assert a.calls[0] == ("goal", POSE, [POSE], True)
    assert client(m).execute(P.SET_GOAL, args)[0] == P.OK  # extras dropped, no TypeError


def test_failed_and_exceptions_become_failed_replies():
    a = Full()
    assert client(a).execute(P.RESTART, {}) == (P.FAILED, "cannot restart")

    class Buggy(Minimal):
        def on_stop(self):
            raise KeyError("x")

    status, text = client(Buggy()).execute(P.STOP, {})
    assert status == P.FAILED and "KeyError" in text


def test_a_repeated_goal_is_answered_from_memory():
    a = Full()
    c = client(a)
    c.execute(P.SET_GOAL, {"goal": POSE})
    assert c.execute(P.SET_GOAL, {"goal": POSE}) == (P.OK, "already done")
    assert len(a.calls) == 1
    c.execute(P.SET_GOAL, {"goal": {**POSE, "x": 9.0}})
    assert len(a.calls) == 2


def test_episode_end_is_optional_and_gets_the_current_episode():
    assert client(Minimal()).execute(P.EPISODE_END, {"result": {"passed": True}})[0] == P.OK
    a = Full()
    c = client(a)
    c.execute(P.RESET, {"episode": "e7", "pose": POSE})
    c.execute(P.EPISODE_END, {"episode": "e7", "result": {"passed": False}})
    kind, ep, result = a.calls[-1]
    assert kind == "end" and ep.episode == "e7" and result == {"passed": False}


def test_report_validates_and_clears_fault_details():
    a = Minimal()
    with pytest.raises(ValueError):
        a.report("FLYING")
    a.report(fault=P.MINIMAL_RISK_MANEUVER, fault_behavior="EMERGENCY_STOP",
             fault_progress=P.OPERATING)
    assert a.state_message()["fault_behavior"] == "EMERGENCY_STOP"
    a.report(fault=P.FAULT_NONE)
    message = a.state_message()
    assert "fault_behavior" not in message and "fault_progress" not in message
    P.validate(message)


def test_parse_relay():
    assert csb_agent.parse_relay("tcp://h:6000") == ("h", 6000)
    assert csb_agent.parse_relay("h") == ("h", 5560)
    with pytest.raises(ValueError):
        csb_agent.parse_relay("udp://h:1")
