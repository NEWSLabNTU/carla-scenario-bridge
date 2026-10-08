import math

import pytest

from scenario_agent_relay import protocol as P

POSE = {"x": 1.0, "y": -2.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0}

MESSAGES = [
    P.register("ego", P.REQUIRED_COMMANDS, "acb_agent"),
    P.registered("ego"),
    P.error("bad"),
    P.heartbeat(),
    P.command(1, P.SET_GOAL, goal=POSE, waypoints=[POSE], allow_goal_modification=True),
    P.command(2, P.SET_GOAL, goal=POSE, waypoints=[],
              segments=[{"id": 7, "type": "lane", "alternatives": []}]),
    P.command(3, P.CLEAR_GOAL),
    P.command(4, P.SET_SPEED_LIMIT, mps=8.3),
    P.command(5, P.SET_SPEED_LIMIT, mps=None),
    P.command(6, P.STOP),
    P.command(7, P.TELEPORTED, pose=POSE),
    P.command(8, P.COOPERATE, module="INTERSECTION", command="ACTIVATE", uuid="00" * 16),
    P.command(9, P.COOPERATE_AUTO, module="CROSSWALK", enable=True),
    P.reply(1),
    P.reply(2, P.UNSUPPORTED, "no RTC"),
    P.state(P.DRIVING),
    P.state(P.IDLE, fault=P.MINIMAL_RISK_MANEUVER, fault_behavior="EMERGENCY_STOP",
            fault_progress=P.SUCCEEDED, turn_indicators=P.LEFT,
            capabilities=P.COMMANDS, detail="x", pose=POSE,
            rtc_status=[{"module": "INTERSECTION", "uuid": "ab" * 16}]),
    # commanders (a simulator adapter driving "agent"-controlled entities)
    P.register_commander("carla_scenario_bridge"),
    P.command_for(1, "bg_av_1", P.SET_GOAL, goal=POSE, waypoints=[]),
    P.command_for(2, "bg_av_1", P.TELEPORTED, timeout=5.0, pose=POSE),
    P.query(3, "bg_av_1"),
    P.reply(3, P.OK, "", P.state(P.IDLE, pose=POSE), registered=True),
    P.reply(4, P.FAILED, "no agent registered for entity 'x'", registered=False),
]


@pytest.mark.parametrize("message", MESSAGES, ids=lambda m: m["type"])
def test_round_trip(message):
    line = P.encode(message)
    assert line.endswith(b"\n") and line.count(b"\n") == 1
    decoded = P.decode(line)
    assert decoded.pop("v") == P.VERSION
    assert decoded == message


def test_every_command_has_a_round_trip_case():
    covered = {m["command"] for m in MESSAGES if m["type"] == P.COMMAND}
    assert covered == set(P.COMMANDS)


def test_wrong_version_is_rejected():
    with pytest.raises(P.ProtocolError, match="version"):
        P.decode(b'{"v": 2, "type": "heartbeat"}')
    with pytest.raises(P.ProtocolError, match="version"):
        P.decode(b'{"type": "heartbeat"}')


@pytest.mark.parametrize("line", [
    b"not json",
    b"[1, 2]",
    b'{"v": 1, "type": "nope"}',
    b'{"v": 1, "type": "register", "entity": ""}',
    b'{"v": 1, "type": "command", "id": 1, "command": "fly"}',
    b'{"v": 1, "type": "command", "id": 1, "command": "set_goal", "args": {}}',
    b'{"v": 1, "type": "command", "id": 1, "command": "teleported", "args": {"pose": {"x": 1}}}',
    b'{"v": 1, "type": "reply", "id": 1, "status": "MAYBE"}',
    b'{"v": 1, "type": "state", "phase": "FLYING"}',
    b'{"v": 1, "type": "state", "phase": "IDLE", "fault": "FIRE"}',
    b'{"v": 1, "type": "command", "id": 1, "command": "cooperate", "args": {"module": "X", "command": "MAYBE"}}',
    b'{"v": 1, "type": "register", "role": "observer", "entity": "ego"}',
    b'{"v": 1, "type": "command_for", "id": 1, "entity": "", "command": "stop"}',
    b'{"v": 1, "type": "command_for", "id": 1, "entity": "a", "command": "fly"}',
    b'{"v": 1, "type": "command_for", "id": 1, "entity": "a", "command": "stop", "timeout": 0}',
    b'{"v": 1, "type": "query", "id": 1}',
    b'{"v": 1, "type": "reply", "id": 1, "status": "OK", "registered": "yes"}',
    b'{"v": 1, "type": "reply", "id": 1, "status": "OK", "state": {"phase": "FLYING"}}',
])
def test_invalid_messages_are_rejected(line):
    with pytest.raises(P.ProtocolError):
        P.decode(line)


def test_non_finite_numbers_become_null():
    line = P.encode(P.command(1, P.SET_SPEED_LIMIT, mps=math.inf))
    assert P.decode(line)["args"]["mps"] is None


def test_line_reader_splits_and_buffers():
    reader = P.LineReader()
    a, b = P.encode(P.heartbeat()), P.encode(P.registered("ego"))
    assert reader.feed(a + b[:5]) == [a.rstrip(b"\n")]
    assert reader.feed(b[5:]) == [b.rstrip(b"\n")]
    assert reader.feed(b"\n\n") == []


def test_a_commander_registers_without_an_entity():
    message = P.decode(P.encode(P.register_commander("csb")))
    assert message["role"] == P.ROLE_COMMANDER and "entity" not in message


def test_an_agent_register_without_a_role_is_still_valid():
    # Agents written before commanders existed send no role.
    P.decode(b'{"v": 1, "type": "register", "entity": "ego", "capabilities": []}')
