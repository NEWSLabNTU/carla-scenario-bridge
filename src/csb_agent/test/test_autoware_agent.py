"""AutowareAgent on a fake Autoware and a fake clock (the acb_pilot core's tests, moved with
it in roadmap 030 step 6), and through `csb-agent check`."""

import math
import threading

import pytest

from csb_agent import Episode, Failed
from csb_agent import protocol as P
from csb_agent.adapters import autoware as C
from csb_agent.check import run_check
from fake_autoware import Clock, FakeAutoware

GOAL = {"x": 50.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
START = {"x": 190.8, "y": -130.1, "z": 0.3, "qx": 0.0, "qy": 0.0,
         "qz": math.sin(1.5708), "qw": math.cos(1.5708)}


@pytest.fixture
def rig():
    clock = Clock()
    autoware = FakeAutoware(clock)
    agent = C.AutowareAgent({"step_period": 0}, port=autoware, clock=clock,
                            sleep=clock.sleep)
    return clock, autoware, agent


def run(clock, agent, seconds, dt=0.1):
    for _ in range(int(seconds / dt)):
        clock.advance(dt)
        agent.step()


def test_phase_derivation_table():
    s = C.Snapshot(ready=False)
    assert C.derive_phase(s, False, False) == P.UNAVAILABLE
    s = C.Snapshot(ready=True, localization=C.LOC_INITIALIZING)
    assert C.derive_phase(s, False, False) == P.INITIALIZING
    s = C.Snapshot(ready=True, localization=C.LOC_INITIALIZED, route=C.ROUTE_UNSET)
    assert C.derive_phase(s, False, False) == P.IDLE
    assert C.derive_phase(s, True, False) == P.INITIALIZING  # re-localizing
    s.route, s.mode = C.ROUTE_SET, C.MODE_STOP
    assert C.derive_phase(s, False, False) == P.PLANNING
    s.autonomous_available = True
    assert C.derive_phase(s, False, False) == P.READY
    assert C.derive_phase(s, False, True) == P.STOPPED
    s.mode, s.control_enabled = C.MODE_AUTONOMOUS, True
    assert C.derive_phase(s, False, False) == P.DRIVING
    s.route = C.ROUTE_ARRIVED
    assert C.derive_phase(s, False, False) == P.ARRIVED


def test_reset_goal_engage_arrive(rig):
    clock, autoware, agent = rig
    agent.on_reset(Episode(episode="ego-1", pose=START))
    assert "initialize" in autoware.calls
    assert autoware.init_stamp > 50.0  # stamped with a scan newer than the old estimate
    assert agent.phase == P.IDLE
    assert C.consistent(START, agent.state_message()["pose"])
    agent.on_goal(GOAL, [])
    assert agent.phase in (P.PLANNING, P.READY)
    run(clock, agent, 2.0)
    assert autoware.calls.count("change_to_autonomous") >= 1
    assert agent.phase == P.DRIVING
    autoware.arrive()
    agent.step()
    assert agent.phase == P.ARRIVED


def test_reset_drops_a_driven_route_first(rig):
    clock, autoware, agent = rig
    agent.on_goal(GOAL, [])
    run(clock, agent, 2.0)
    assert agent.phase == P.DRIVING
    agent.on_reset(Episode(pose=START))
    calls = autoware.calls
    assert calls.index("change_to_stop") < calls.index("clear_route") < calls.index("initialize")
    assert agent.goal is None and agent.phase == P.IDLE


def test_reset_waits_for_fresh_scans_but_not_forever(rig):
    clock, autoware, agent = rig
    autoware.publish_scans = False
    started = clock()
    agent.on_reset(Episode(pose=START))
    assert "initialize" in autoware.calls
    assert clock() - started >= agent.cfg["scan_wait"]


def test_reset_fails_when_localization_does_not_converge(rig):
    clock, autoware, agent = rig
    autoware.CONVERGE_TIME = 1e9
    with pytest.raises(Failed, match="did not reach"):
        agent.on_reset(Episode(pose=START))
    assert "did not reach" in agent.state_message()["detail"]


def test_reset_retries_a_refused_initialize(rig):
    clock, autoware, agent = rig
    autoware.refuse.add("initialize")
    calls = []
    original = autoware.initialize

    def initialize(pose, stamp):  # refused twice, then accepted
        calls.append(clock())
        if len(calls) == 3:
            autoware.refuse.clear()
        return original(pose, stamp)

    autoware.initialize = initialize
    agent.on_reset(Episode(pose=START))
    assert len(calls) == 3 and agent.phase == P.IDLE


def test_reset_gives_up_after_the_initialize_attempts(rig):
    clock, autoware, agent = rig
    autoware.refuse.add("initialize")
    with pytest.raises(Failed, match="refused"):
        agent.on_reset(Episode(pose=START))
    assert autoware.calls.count("initialize") == agent.cfg["initialize_attempts"]


def test_reset_waits_for_autoware(rig):
    clock, autoware, agent = rig
    autoware.s.ready = False
    agent.cfg["ready_timeout"] = 3.0
    with pytest.raises(Failed, match="did not answer"):
        agent.on_reset(Episode(pose=START))


def test_overridden_step():
    clock = Clock()
    autoware = FakeAutoware(clock)
    seen = []

    class Quick(C.AutowareAgent):
        def relocalize(self, pose):  # trust the simulator's teleport
            seen.append(pose)

    agent = Quick({"step_period": 0}, port=autoware, clock=clock, sleep=clock.sleep)
    agent.on_reset(Episode(pose=START))
    assert seen == [START] and "initialize" not in autoware.calls


def test_duplicate_goal_is_not_resent(rig):
    clock, autoware, agent = rig
    agent.on_goal(GOAL, [])
    calls = autoware.calls.count("set_route")
    agent.on_goal(GOAL, [])
    assert autoware.calls.count("set_route") == calls


def test_new_goal_while_driving_stops_clears_and_reroutes(rig):
    clock, autoware, agent = rig
    agent.on_goal(GOAL, [])
    run(clock, agent, 2.0)
    assert agent.phase == P.DRIVING
    other = dict(GOAL, x=80.0)
    agent.on_goal(other, [])
    assert autoware.calls[-3:] == ["change_to_stop", "clear_route", "set_route"]
    run(clock, agent, 2.0)
    assert agent.phase == P.DRIVING and autoware.goal == other


def test_stop_holds_until_the_next_goal(rig):
    clock, autoware, agent = rig
    agent.on_goal(GOAL, [])
    run(clock, agent, 2.0)
    agent.on_stop()
    run(clock, agent, 5.0)
    assert agent.phase == P.STOPPED  # no re-engage
    assert autoware.calls.count("change_to_autonomous") == 1


def test_engage_retries_while_autoware_refuses(rig):
    clock, autoware, agent = rig
    autoware.refuse.add("change_to_autonomous")
    agent.on_goal(GOAL, [])
    run(clock, agent, 8.0)
    assert autoware.calls.count("change_to_autonomous") >= 2
    assert agent.phase == P.READY
    autoware.refuse.clear()
    run(clock, agent, 4.0)
    assert agent.phase == P.DRIVING


def test_goal_refused_by_autoware_fails(rig):
    clock, autoware, agent = rig
    autoware.refuse.add("set_route")
    with pytest.raises(Failed, match="refused"):
        agent.on_goal(GOAL, [])


def test_unavailable_autoware(rig):
    clock, autoware, agent = rig
    autoware.s.ready = False
    agent.step()
    assert agent.phase == P.UNAVAILABLE
    with pytest.raises(Failed):
        agent.on_goal(GOAL, [])


def test_speed_limit(rig):
    clock, autoware, agent = rig
    agent.on_speed_limit(8.0)
    assert autoware.velocity_limit == 8.0
    agent.on_speed_limit(None)  # no limit: nothing sent
    assert autoware.calls.count("velocity_limit") == 1


def test_rtc_only_with_the_services(rig):
    clock, autoware, agent = rig
    assert P.COOPERATE not in agent.capabilities()
    with pytest.raises(Failed):
        agent.on_cooperate("INTERSECTION", "ACTIVATE")
    autoware.s.rtc = True
    agent.on_cooperate("INTERSECTION", "ACTIVATE")
    assert P.COOPERATE in agent.capabilities()


@pytest.mark.parametrize("emergency, mrm, behavior, fault, progress", [
    (False, C.MRM_NORMAL, 1, P.FAULT_NONE, None),
    (False, C.MRM_OPERATING, 2, P.MINIMAL_RISK_MANEUVER, P.OPERATING),
    (False, C.MRM_SUCCEEDED, 3, P.MINIMAL_RISK_MANEUVER, P.SUCCEEDED),
    (True, C.MRM_OPERATING, 2, P.EMERGENCY, None),
])
def test_fault_reporting(rig, emergency, mrm, behavior, fault, progress):
    clock, autoware, agent = rig
    autoware.s.emergency, autoware.s.mrm_state, autoware.s.mrm_behavior = (emergency, mrm,
                                                                          behavior)
    agent.step()
    message = agent.state_message()
    assert message["fault"] == fault
    assert message.get("fault_progress") == progress
    P.validate(message)


def test_state_message_is_valid_protocol(rig):
    clock, autoware, agent = rig
    autoware.s.turn_indicators = 2
    agent.step()
    message = agent.state_message()
    assert message["turn_indicators"] == P.LEFT
    P.decode(P.encode(message))


class _RealTimeFake(C.AutowareAgent):
    """For `csb-agent check`: a fake Autoware on its own real-time clock."""

    def make_port(self):
        clock = Clock()
        port = FakeAutoware(clock)
        port.CONVERGE_TIME = 0.3

        def tick():
            while not self._closing.wait(0.05):
                clock.advance(0.05)
        threading.Thread(target=tick, daemon=True).start()
        return port


def test_passes_check_on_a_fake_autoware():
    report = run_check(_RealTimeFake, {"step_period": 0.05, "settle": 0.5}, 15.0)
    assert report.passed, report.text()
