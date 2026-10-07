"""The agent protocol (I3): newline-delimited JSON over plain TCP.

Spec: docs/design/user-workflow.md, "The agent protocol (I3)". This module is the
executable form of that section and depends on nothing but the standard library, so an
agent for any autopilot can copy it (acb_pilot/agent_protocol.py is such a copy).

Every message is one JSON object on one line, UTF-8, terminated by "\\n", and carries
"v": VERSION and "type". A receiver drops a connection whose messages carry another
version, after sending an "error" message naming the reason.
"""

import json
import math

VERSION = 1

# -- message types -------------------------------------------------------------------
REGISTER = "register"  # agent -> relay, first message on a connection
REGISTERED = "registered"  # relay -> agent, answer to register
ERROR = "error"  # either way; the sender closes the connection after it
COMMAND = "command"  # relay -> agent
REPLY = "reply"  # agent -> relay, answer to a command (same id)
STATE = "state"  # agent -> relay, on change and periodically
HEARTBEAT = "heartbeat"  # either way, when nothing else was sent for HEARTBEAT_PERIOD

MESSAGE_TYPES = (REGISTER, REGISTERED, ERROR, COMMAND, REPLY, STATE, HEARTBEAT)

# Each side sends something at least this often, and treats a peer that sent nothing for
# PEER_TIMEOUT as gone.
HEARTBEAT_PERIOD = 1.0
PEER_TIMEOUT = 3.0

# -- commands (relay -> agent) -------------------------------------------------------
SET_GOAL = "set_goal"
CLEAR_GOAL = "clear_goal"
SET_SPEED_LIMIT = "set_speed_limit"
STOP = "stop"
TELEPORTED = "teleported"
COOPERATE = "cooperate"  # optional (RTC)
COOPERATE_AUTO = "cooperate_auto"  # optional (RTC)

REQUIRED_COMMANDS = (SET_GOAL, CLEAR_GOAL, SET_SPEED_LIMIT, STOP, TELEPORTED)
OPTIONAL_COMMANDS = (COOPERATE, COOPERATE_AUTO)
COMMANDS = REQUIRED_COMMANDS + OPTIONAL_COMMANDS

# -- reply status ---------------------------------------------------------------------
OK = "OK"
FAILED = "FAILED"
UNSUPPORTED = "UNSUPPORTED"
REPLY_STATUSES = (OK, FAILED, UNSUPPORTED)

# -- state ----------------------------------------------------------------------------
UNAVAILABLE = "UNAVAILABLE"
INITIALIZING = "INITIALIZING"
IDLE = "IDLE"
PLANNING = "PLANNING"
READY = "READY"
DRIVING = "DRIVING"
ARRIVED = "ARRIVED"
STOPPED = "STOPPED"
PHASES = (UNAVAILABLE, INITIALIZING, IDLE, PLANNING, READY, DRIVING, ARRIVED, STOPPED)

FAULT_NONE = "NONE"
MINIMAL_RISK_MANEUVER = "MINIMAL_RISK_MANEUVER"
EMERGENCY = "EMERGENCY"
FAULTS = (FAULT_NONE, MINIMAL_RISK_MANEUVER, EMERGENCY)

# How far a minimal-risk maneuver has got. Optional; OPERATING when absent.
OPERATING = "OPERATING"
SUCCEEDED = "SUCCEEDED"
FAULT_PROGRESSES = (OPERATING, SUCCEEDED, FAILED)

INDICATORS_NONE = "NONE"
LEFT = "LEFT"
RIGHT = "RIGHT"
HAZARD = "HAZARD"
TURN_INDICATORS = (INDICATORS_NONE, LEFT, RIGHT, HAZARD)

POSE_KEYS = ("x", "y", "z", "qx", "qy", "qz", "qw")


class ProtocolError(ValueError):
    """A message that does not follow the schema."""


def _finite_or_none(value):
    """JSON has no NaN or infinity: encode a non-finite number as null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _finite_or_none(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_or_none(v) for v in value]
    return value


def encode(message: dict) -> bytes:
    """One message as one line. Adds "v"."""
    if message.get("type") not in MESSAGE_TYPES:
        raise ProtocolError(f"unknown message type {message.get('type')!r}")
    body = dict(message)
    body["v"] = VERSION
    return (json.dumps(_finite_or_none(body), allow_nan=False, separators=(",", ":"))
            + "\n").encode("utf-8")


def decode(line) -> dict:
    """One line to a validated message. Raises ProtocolError."""
    if isinstance(line, bytes):
        try:
            line = line.decode("utf-8")
        except UnicodeDecodeError as e:
            raise ProtocolError(f"not UTF-8: {e}") from e
    try:
        message = json.loads(line)
    except json.JSONDecodeError as e:
        raise ProtocolError(f"not JSON: {e}") from e
    if not isinstance(message, dict):
        raise ProtocolError("a message is a JSON object")
    if message.get("v") != VERSION:
        raise ProtocolError(f"protocol version {message.get('v')!r}, expected {VERSION}")
    validate(message)
    return message


def _require(message, key, types):
    if key not in message:
        raise ProtocolError(f"{message.get('type')}: missing {key!r}")
    if not isinstance(message[key], types):
        raise ProtocolError(f"{message.get('type')}: {key!r} has the wrong type")
    return message[key]


def _one_of(message, key, allowed, optional=False):
    if optional and message.get(key) is None:
        return
    value = _require(message, key, str)
    if value not in allowed:
        raise ProtocolError(f"{message.get('type')}: {key}={value!r} not in {allowed}")


def validate_pose(pose, where="pose"):
    if not isinstance(pose, dict):
        raise ProtocolError(f"{where}: a pose is an object")
    for key in POSE_KEYS:
        if not isinstance(pose.get(key), (int, float)) or isinstance(pose.get(key), bool):
            raise ProtocolError(f"{where}: {key!r} must be a number")


def validate(message: dict) -> None:
    kind = message.get("type")
    if kind not in MESSAGE_TYPES:
        raise ProtocolError(f"unknown message type {kind!r}")
    if kind == REGISTER:
        if not _require(message, "entity", str):
            raise ProtocolError("register: empty entity")
        caps = message.get("capabilities", [])
        if not isinstance(caps, list) or not all(isinstance(c, str) for c in caps):
            raise ProtocolError("register: capabilities is a list of command names")
    elif kind == REGISTERED:
        _require(message, "entity", str)
    elif kind == ERROR:
        _require(message, "reason", str)
    elif kind == COMMAND:
        _require(message, "id", int)
        _one_of(message, "command", COMMANDS)
        args = message.get("args", {})
        if not isinstance(args, dict):
            raise ProtocolError("command: args is an object")
        _validate_args(message["command"], args)
    elif kind == REPLY:
        _require(message, "id", int)
        _one_of(message, "status", REPLY_STATUSES)
    elif kind == STATE:
        _one_of(message, "phase", PHASES)
        _one_of(message, "fault", FAULTS, optional=True)
        _one_of(message, "fault_progress", FAULT_PROGRESSES, optional=True)
        _one_of(message, "turn_indicators", TURN_INDICATORS, optional=True)
        if message.get("pose") is not None:
            validate_pose(message["pose"], "state.pose")
        caps = message.get("capabilities", [])
        if not isinstance(caps, list):
            raise ProtocolError("state: capabilities is a list")
        rtc = message.get("rtc_status")
        if rtc is not None and not isinstance(rtc, list):
            raise ProtocolError("state: rtc_status is a list")


def _validate_args(command, args):
    if command == SET_GOAL:
        validate_pose(args.get("goal"), "set_goal.goal")
        for i, waypoint in enumerate(args.get("waypoints", [])):
            validate_pose(waypoint, f"set_goal.waypoints[{i}]")
        segments = args.get("segments")
        if segments is not None:
            if not isinstance(segments, list):
                raise ProtocolError("set_goal: segments is a list")
            for segment in segments:
                if not isinstance(segment, dict) or not isinstance(segment.get("id"), int):
                    raise ProtocolError("set_goal: a segment has an integer id")
    elif command == SET_SPEED_LIMIT:
        mps = args.get("mps")
        if mps is not None and (not isinstance(mps, (int, float)) or isinstance(mps, bool)):
            raise ProtocolError("set_speed_limit: mps is a number or null (no limit)")
    elif command == TELEPORTED:
        validate_pose(args.get("pose"), "teleported.pose")
    elif command == COOPERATE:
        _require(args, "module", str)
        _one_of(args, "command", ("ACTIVATE", "DEACTIVATE"))
    elif command == COOPERATE_AUTO:
        _require(args, "module", str)
        _require(args, "enable", bool)


# -- constructors -----------------------------------------------------------------------

def register(entity, capabilities=(), agent=""):
    return {"type": REGISTER, "entity": entity, "agent": agent,
            "capabilities": list(capabilities)}


def registered(entity):
    return {"type": REGISTERED, "entity": entity}


def error(reason):
    return {"type": ERROR, "reason": reason}


def command(command_id, name, **args):
    return {"type": COMMAND, "id": command_id, "command": name, "args": args}


def reply(command_id, status=OK, message=""):
    return {"type": REPLY, "id": command_id, "status": status, "message": message}


def heartbeat():
    return {"type": HEARTBEAT}


def state(phase, fault=FAULT_NONE, fault_behavior=None, fault_progress=None,
          turn_indicators=None, capabilities=(), detail="", pose=None, rtc_status=None):
    message = {"type": STATE, "phase": phase, "fault": fault,
               "capabilities": list(capabilities), "detail": detail}
    if fault_behavior is not None:
        message["fault_behavior"] = fault_behavior
    if fault_progress is not None:
        message["fault_progress"] = fault_progress
    if turn_indicators is not None:
        message["turn_indicators"] = turn_indicators
    if pose is not None:
        message["pose"] = pose
    if rtc_status is not None:
        message["rtc_status"] = rtc_status
    return message


class LineReader:
    """Splits a byte stream into lines; bounded so a peer cannot grow it forever."""

    MAX_LINE = 1 << 20

    def __init__(self):
        self._buffer = b""

    def feed(self, data: bytes):
        self._buffer += data
        lines = self._buffer.split(b"\n")
        self._buffer = lines.pop()
        if len(self._buffer) > self.MAX_LINE:
            raise ProtocolError("line longer than 1 MiB")
        return [line for line in lines if line.strip()]
