import socket
import threading
import time

import pytest

from scenario_agent_relay import protocol as P
from scenario_agent_relay.server import AgentServer


class FakeAgent:
    def __init__(self, port, entity="ego", register=True):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.reader = P.LineReader()
        self.pending = []
        if register:
            self.send(P.register(entity, P.REQUIRED_COMMANDS, "fake"))
            assert self.recv()["type"] == P.REGISTERED

    def send(self, message):
        self.sock.sendall(P.encode(message))

    def send_raw(self, data):
        self.sock.sendall(data)

    def recv(self, skip_heartbeats=True):
        while True:
            while not self.pending:
                data = self.sock.recv(65536)
                if not data:
                    raise ConnectionError("closed")
                self.pending += self.reader.feed(data)
            message = P.decode(self.pending.pop(0))
            if not (skip_heartbeats and message["type"] == P.HEARTBEAT):
                return message

    def close(self):
        self.sock.close()


@pytest.fixture
def server():
    events = {"states": [], "presence": []}
    s = AgentServer("127.0.0.1", 0, peer_timeout=0.6, heartbeat_period=0.2,
                    on_state=lambda e, st, t: events["states"].append((e, st["phase"])),
                    on_presence=lambda e, p: events["presence"].append((e, p)))
    s.events = events
    s.start()
    yield s
    s.stop()


def wait_until(predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_register_command_reply(server):
    agent = FakeAgent(server.port)
    assert wait_until(lambda: server.session("ego") is not None)
    result = {}

    def call():
        result["r"] = server.command("ego", P.STOP, 2.0)

    thread = threading.Thread(target=call)
    thread.start()
    message = agent.recv()
    assert message["type"] == P.COMMAND and message["command"] == P.STOP
    agent.send(P.reply(message["id"], P.UNSUPPORTED, "nope"))
    thread.join()
    assert result["r"] == (P.UNSUPPORTED, "nope")
    agent.close()


def test_state_reaches_the_callback(server):
    agent = FakeAgent(server.port)
    agent.send(P.state(P.IDLE))
    assert wait_until(lambda: ("ego", P.IDLE) in server.events["states"])
    assert server.session("ego").state["phase"] == P.IDLE
    agent.close()


def test_no_agent_and_no_reply():
    s = AgentServer("127.0.0.1", 0, peer_timeout=5.0)
    s.start()
    try:
        status, message = s.command("ego", P.STOP, 0.2)
        assert status is None and "no agent" in message
        agent = FakeAgent(s.port)
        status, message = s.command("ego", P.STOP, 0.3)  # agent never answers
        assert status is None and "did not reply" in message
        agent.close()
    finally:
        s.stop()


def test_silent_agent_times_out(server):
    agent = FakeAgent(server.port)
    assert wait_until(lambda: ("ego", True) in server.events["presence"])
    # The fake agent sends nothing after registering; the relay's heartbeats do not
    # count as the agent being alive.
    assert wait_until(lambda: ("ego", False) in server.events["presence"], timeout=3.0)
    assert server.session("ego") is None
    agent.close()


def test_heartbeats_keep_an_agent_and_the_relay_sends_them(server):
    agent = FakeAgent(server.port)
    end = time.monotonic() + 1.5
    saw_heartbeat = False
    agent.sock.settimeout(0.1)
    while time.monotonic() < end:
        agent.send(P.heartbeat())
        try:
            saw_heartbeat |= agent.recv(skip_heartbeats=False)["type"] == P.HEARTBEAT
        except socket.timeout:
            pass
    assert saw_heartbeat
    assert server.session("ego") is not None
    agent.close()


def test_a_new_registration_replaces_the_old(server):
    first = FakeAgent(server.port)
    assert wait_until(lambda: server.session("ego") is not None)
    old = server.session("ego")
    second = FakeAgent(server.port)
    assert wait_until(lambda: server.session("ego") is not old)
    assert old.closed.is_set()
    first.close()
    second.close()


def test_closing_the_connection_drops_the_agent(server):
    agent = FakeAgent(server.port)
    assert wait_until(lambda: server.session("ego") is not None)
    agent.close()
    assert wait_until(lambda: server.session("ego") is None)


def test_wrong_version_gets_an_error_and_is_dropped(server):
    agent = FakeAgent(server.port, register=False)
    agent.send_raw(b'{"v": 2, "type": "register", "entity": "ego"}\n')
    message = agent.recv()
    assert message["type"] == P.ERROR and "version" in message["reason"]
    assert server.session("ego") is None
    agent.close()


class FakeCommander(FakeAgent):
    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.reader = P.LineReader()
        self.pending = []
        self.send(P.register_commander("fake_commander"))
        assert self.recv()["type"] == P.REGISTERED


def test_commander_query_without_and_with_an_agent(server):
    commander = FakeCommander(server.port)
    commander.send(P.query(1, "bg_av_1"))
    message = commander.recv()
    assert message["type"] == P.REPLY and message["id"] == 1
    assert message["status"] == P.FAILED and "no agent" in message["message"]
    assert message["registered"] is False
    agent = FakeAgent(server.port, entity="bg_av_1")
    agent.send(P.state(P.IDLE))
    assert wait_until(lambda: ("bg_av_1", P.IDLE) in server.events["states"])
    commander.send(P.query(2, "bg_av_1"))
    message = commander.recv()
    assert message["status"] == P.OK and message["state"]["phase"] == P.IDLE
    assert message["registered"] is True
    agent.close()
    commander.close()


def test_commander_command_reaches_the_agent_and_the_reply_comes_back(server):
    agent = FakeAgent(server.port, entity="bg_av_1")
    commander = FakeCommander(server.port)
    pose = {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    commander.send(P.command_for(7, "bg_av_1", P.TELEPORTED, pose=pose))
    command = agent.recv()
    assert command["type"] == P.COMMAND and command["command"] == P.TELEPORTED
    assert command["args"]["pose"] == pose
    # Ordering rule: the state the command produced arrives before the reply.
    agent.send(P.state(P.INITIALIZING))
    agent.send(P.reply(command["id"], P.OK))
    message = commander.recv()
    assert message["type"] == P.REPLY and message["id"] == 7  # the commander's id
    assert message["status"] == P.OK and message["state"]["phase"] == P.INITIALIZING
    agent.close()
    commander.close()


def test_commander_command_without_an_agent_fails_clearly(server):
    commander = FakeCommander(server.port)
    commander.send(P.command_for(1, "nobody", P.STOP))
    message = commander.recv()
    assert message["status"] == P.FAILED
    assert "no agent registered for entity 'nobody'" in message["message"]
    assert message["registered"] is False and "state" not in message
    commander.close()


def test_commander_command_timeout(server):
    agent = FakeAgent(server.port, entity="bg_av_1")
    commander = FakeCommander(server.port)
    commander.send(P.command_for(1, "bg_av_1", P.STOP, timeout=0.3))
    assert agent.recv()["command"] == P.STOP  # never answered
    message = commander.recv()
    assert message["status"] == P.FAILED and "did not reply" in message["message"]
    agent.close()
    commander.close()


def test_a_silent_commander_is_kept_and_does_not_take_an_entity(server):
    commander = FakeCommander(server.port)
    time.sleep(1.0)  # longer than the fixture's 0.6 s peer timeout
    commander.send(P.query(1, "ego"))
    assert commander.recv()["id"] == 1
    assert server.entities() == []
    commander.close()


def test_register_and_query_in_one_packet(server):
    sock = socket.create_connection(("127.0.0.1", server.port), timeout=5)
    sock.sendall(P.encode(P.register_commander()) + P.encode(P.query(5, "ego")))
    commander = FakeCommander.__new__(FakeCommander)
    commander.sock, commander.reader, commander.pending = sock, P.LineReader(), []
    assert commander.recv()["type"] == P.REGISTERED
    assert commander.recv()["id"] == 5
    sock.close()


def test_a_commander_cannot_send_agent_messages(server):
    commander = FakeCommander(server.port)
    commander.send(P.state(P.IDLE))
    message = commander.recv()
    assert message["type"] == P.ERROR and "commander" in message["reason"]
    commander.close()
