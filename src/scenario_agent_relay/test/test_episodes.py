"""SSv2's initialize -> reset (v2) or teleported (v1), with the reset bounded (roadmap 030)."""

import socket
import threading
import time

import pytest

import csb_agent
from csb_agent import protocol as P
from scenario_agent_relay.episodes import Episodes
from scenario_agent_relay.server import AgentServer

POSE = {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


class SlowReset(csb_agent.Agent):
    def __init__(self, seconds=0.0, fail=False):
        super().__init__({"entity": "ego"})
        self.seconds, self.fail = seconds, fail
        self.resets = []

    def on_reset(self, ep):
        self.resets.append(ep)
        time.sleep(self.seconds)
        if self.fail:
            raise csb_agent.Failed("localization did not converge")

    def on_stop(self):
        pass


class NoReset(csb_agent.Agent):
    def __init__(self):
        super().__init__({"entity": "ego"})
        self.teleports = 0

    def on_stop(self):
        pass


def wait_until(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def relay():
    s = AgentServer("127.0.0.1", 0)
    s.start()
    yield s
    s.stop()


def run(relay, agent):
    client = csb_agent.Client(agent, "127.0.0.1", relay.port)
    threading.Thread(target=client.run, daemon=True).start()
    assert wait_until(lambda: relay.session("ego") is not None)
    return client


def test_a_quick_reset_is_answered_with_its_outcome(relay):
    agent = SlowReset()
    client = run(relay, agent)
    status, _ = Episodes(relay).initialize("ego", POSE, reply_window=2.0)
    assert status == P.OK and len(agent.resets) == 1
    assert agent.resets[0].episode.startswith("ego-1-") and agent.resets[0].pose == POSE
    record = relay.last_reset("ego")
    assert record["status"] == P.OK and record["seconds"] is not None
    client.stop()


def test_a_failed_reset_fails_the_initialize(relay):
    client = run(relay, SlowReset(fail=True))
    status, message = Episodes(relay).initialize("ego", POSE, reply_window=2.0)
    assert status == P.FAILED and "converge" in message
    assert relay.last_reset("ego")["status"] == P.FAILED
    client.stop()


def test_a_slow_reset_answers_ok_then_goes_on_and_a_retry_joins_it(relay):
    agent = SlowReset(seconds=1.5)
    client = run(relay, agent)
    episodes = Episodes(relay, reset_timeout=10.0)
    status, message = episodes.initialize("ego", POSE, reply_window=0.3)
    assert status == P.OK and "in progress" in message
    # Meanwhile the agent shows INITIALIZING, which holds the concealer.
    assert wait_until(lambda: relay.session("ego").state["phase"] == P.INITIALIZING)
    status, _ = episodes.initialize("ego", POSE, reply_window=3.0)  # the concealer's retry
    assert status == P.OK and len(agent.resets) == 1
    assert wait_until(lambda: relay.session("ego").state["phase"] == P.IDLE)
    client.stop()


def test_a_reset_past_its_timeout_is_recorded(relay):
    client = run(relay, SlowReset(seconds=2.0))
    episodes = Episodes(relay, reset_timeout=0.5)
    status, _ = episodes.initialize("ego", POSE, reply_window=1.5)
    assert status == P.FAILED
    record = relay.last_reset("ego")
    assert record["timed_out"] and record["status"] == P.FAILED
    client.stop()


def test_an_agent_without_reset_gets_teleported(relay):
    agent = NoReset()
    client = run(relay, agent)
    assert P.RESET not in relay.session("ego").capabilities
    status, message = Episodes(relay).initialize("ego", POSE, reply_window=2.0)
    assert status == P.OK and message == "nothing to do"  # the SDK's answer to teleported
    assert relay.last_reset("ego") is None
    client.stop()


def test_no_agent(relay):
    status, message = Episodes(relay).initialize("ego", POSE, reply_window=0.1)
    assert status is None and "no agent" in message


def test_a_commander_query_carries_the_last_reset(relay):
    client = run(relay, SlowReset())
    Episodes(relay).initialize("ego", POSE, reply_window=2.0)
    sock = socket.create_connection(("127.0.0.1", relay.port), timeout=5)
    sock.sendall(P.encode(P.register_commander("test")))
    sock.sendall(P.encode(P.query(1, "ego")))
    reader, replies = P.LineReader(), []
    while len(replies) < 2:
        replies += [P.decode(line) for line in reader.feed(sock.recv(65536))]
    answer = replies[1]
    assert answer["type"] == P.REPLY and answer["reset"]["status"] == P.OK
    sock.close()
    client.stop()


class Ender(SlowReset):
    def __init__(self):
        super().__init__()
        self.ends = []

    def on_episode_end(self, ep, result):
        self.ends.append((ep.episode, result))


def test_episode_end_through_a_commander_reaches_the_agent(relay):
    from csb_agent.commander import Commander
    agent = Ender()
    client = run(relay, agent)
    Episodes(relay).initialize("ego", POSE, reply_window=2.0)
    with Commander(f"tcp://127.0.0.1:{relay.port}", "run_suite") as c:
        reply = c.episode_end("ego", passed=False, failures=["carla_contact"], contacts=1)
        assert reply["status"] == P.OK
        assert c.query("ego")["reset"]["status"] == P.OK
        assert c.episode_end("nobody", passed=True)["registered"] is False
    ((episode, result),) = agent.ends
    assert episode == agent.resets[0].episode
    assert result == {"passed": False, "failures": ["carla_contact"], "contacts": 1}
    client.stop()
