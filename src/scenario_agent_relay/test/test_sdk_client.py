"""The csb_agent SDK against the relay's real AgentServer, in-process (roadmap 030)."""

import threading
import time

import pytest

import csb_agent
from csb_agent import protocol as P
from scenario_agent_relay.server import AgentServer

POSE = {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


class Recorder(csb_agent.Agent):
    def __init__(self, config=None):
        super().__init__(config)
        self.calls = []

    def on_reset(self, ep):
        self.calls.append(("reset", ep.episode))
        time.sleep(0.3)  # long enough that the relay sees the state before the reply

    def on_goal(self, goal, waypoints):
        self.calls.append(("goal", goal["x"]))

    def on_stop(self):
        self.calls.append(("stop",))

    def on_speed_limit(self, mps):
        self.calls.append(("limit", mps))


def wait_until(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def relay(request):
    versions = getattr(request, "param", P.VERSIONS)
    states = []
    s = AgentServer("127.0.0.1", 0, versions=versions,
                    on_state=lambda e, st, t: states.append(st["phase"]))
    s.states = states
    s.start()
    yield s
    s.stop()


@pytest.fixture
def agent_client(relay):
    agent = Recorder({"entity": "ego"})
    client = csb_agent.Client(agent, "127.0.0.1", relay.port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    assert wait_until(lambda: relay.session("ego") is not None)
    yield agent, client
    client.stop()
    thread.join(timeout=3)


def test_registers_v2_with_its_capabilities(relay, agent_client):
    session = relay.session("ego")
    assert session.version == 2
    assert P.RESET in session.capabilities and P.RESTART not in session.capabilities
    assert session.agent == "Recorder"


def test_reset_round_trip_and_state_before_reply(relay, agent_client):
    agent, _ = agent_client
    agent.report(P.DRIVING)
    assert wait_until(lambda: relay.session("ego").state["phase"] == P.DRIVING)
    status, _ = relay.command("ego", P.RESET, 5.0, episode="e1", pose=POSE)
    assert status == P.OK and agent.calls == [("reset", "e1")]
    # The ordering rule: the state the command produced arrived before the reply.
    assert relay.session("ego").state["phase"] == P.IDLE


def test_commands_map_to_hooks(relay, agent_client):
    agent, _ = agent_client
    assert relay.command("ego", P.SET_GOAL, 5.0, goal={**POSE, "x": 7.0})[0] == P.OK
    assert relay.command("ego", P.SET_SPEED_LIMIT, 5.0, mps=None)[0] == P.OK
    assert relay.command("ego", P.STOP, 5.0)[0] == P.OK
    assert relay.command("ego", P.RESTART, 5.0)[0] == P.UNSUPPORTED  # not offered
    assert agent.calls == [("goal", 7.0), ("limit", None), ("stop",)]
    assert relay.session("ego").state["phase"] == P.STOPPED


def test_heartbeats_keep_a_quiet_agent_registered(relay, agent_client):
    session = relay.session("ego")
    time.sleep(P.PEER_TIMEOUT + 1.0)
    assert relay.session("ego") is session


@pytest.mark.parametrize("relay", [(1,)], indirect=True)
def test_falls_back_to_v1_on_a_v1_relay(relay, agent_client):
    agent, client = agent_client
    assert client.version == 1 and relay.session("ego").version == 1
    # v1: the boundary arrives as teleported; the SDK runs on_reset with the pose only.
    assert relay.command("ego", P.TELEPORTED, 5.0, pose=POSE)[0] == P.OK
    assert agent.calls == [("reset", "")]


def test_reconnects_after_the_relay_drops_it(relay, agent_client):
    first = relay.session("ego")
    first.close("test drop")
    assert wait_until(lambda: relay.session("ego") not in (None, first), timeout=8.0)
