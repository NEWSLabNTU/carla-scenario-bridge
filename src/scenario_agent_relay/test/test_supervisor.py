"""The supervisor role (roadmap 030 step 8): `csb-agent supervise` registers for an entity,
the relay routes `restart` to it when the agent cannot restart itself, and the supervisor
replaces the whole process group, after which the agent registers again."""

import os
import sys
import tempfile
import threading
import time

import csb_agent
from csb_agent import protocol as P
from csb_agent.supervise import Group, Supervisor
from scenario_agent_relay.server import AgentServer


def wait_until(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def start_server():
    server = AgentServer(host="127.0.0.1", port=0, peer_timeout=3.0, heartbeat_period=0.5)
    server.start()
    return server


def agent_command(port):
    config = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    config.write("world: fake\n")
    config.close()
    return (f"{sys.executable} -m csb_agent.adapters.carla_basic --relay tcp://127.0.0.1:{port}"
            f" --entity ego --config {config.name} --log-level WARNING")


def test_restart_goes_to_the_supervisor_and_the_agent_comes_back():
    server = start_server()
    group = Group(agent_command(server.port), grace=3.0)
    sup = Supervisor("ego", f"tcp://127.0.0.1:{server.port}", group, respawn_delay=1.0)
    thread = threading.Thread(target=sup.run, daemon=True)
    thread.start()
    try:
        assert wait_until(lambda: server.session("ego") is not None
                          and server.supervisor("ego") is not None, 20.0)
        first_pgid, first = group.pgid, server.session("ego")
        with csb_agent.Commander(f"tcp://127.0.0.1:{server.port}", "test") as c:
            answer = c.query("ego")
            assert answer["registered"] and answer["supervised"]
            reply = c.command("ego", P.RESTART, timeout=20.0)
        assert reply["status"] == P.OK, reply
        assert group.pgid != first_pgid
        try:
            os.killpg(first_pgid, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        assert not alive  # the old group is gone, by process group
        assert wait_until(lambda: server.session("ego") is not None
                          and server.session("ego") is not first, 20.0)
    finally:
        sup.stop()
        thread.join(timeout=15.0)
        server.stop()
    assert group.pgid is None  # stopped with the supervisor


def test_an_agent_that_exits_is_started_again():
    server = start_server()
    group = Group(f"{sys.executable} -c 'import time; time.sleep(0.5)'", grace=1.0)
    sup = Supervisor("ego", f"tcp://127.0.0.1:{server.port}", group, respawn_delay=0.5)
    thread = threading.Thread(target=sup.run, daemon=True)
    thread.start()
    try:
        assert wait_until(lambda: group.starts >= 2, 10.0)
    finally:
        sup.stop()
        thread.join(timeout=10.0)
        server.stop()


def test_restart_without_supervisor_or_capability_is_unsupported():
    server = start_server()

    class Plain(csb_agent.Agent):
        def on_stop(self):
            pass

    agent = Plain({"entity": "ego"})
    client = csb_agent.Client(agent, "127.0.0.1", server.port)
    threading.Thread(target=client.run, daemon=True).start()
    try:
        assert wait_until(lambda: server.session("ego") is not None)
        with csb_agent.Commander(f"tcp://127.0.0.1:{server.port}", "test") as c:
            assert c.query("ego")["supervised"] is False
            reply = c.command("ego", P.RESTART, timeout=2.0)
        assert reply["status"] == P.UNSUPPORTED
        assert "supervisor" in reply["message"]
    finally:
        client.stop()
        server.stop()


def test_supervisor_role_needs_v2():
    import pytest
    with pytest.raises(P.ProtocolError):
        P.encode(P.register_supervisor("ego"), version=1)
