"""`csb_agent.Agent`: the one interface an autopilot implements to drive a scenario.

Subclass it and override the hooks the autopilot can honour; `csb_agent.main(MyAgent)`
then registers with the scenario's relay and runs them (client.py). A hook returns
normally for OK, raises `Failed(message)` for FAILED; a hook that is not overridden is
answered UNSUPPORTED. Hooks run one at a time, on the client's worker thread.

    class MyAgent(csb_agent.Agent):
        def on_reset(self, ep):            ...  # clear plan, re-localize at ep.pose
        def on_goal(self, goal, waypoints): ...  # plan and drive
        def on_stop(self):                 ...

The agent tells the relay where it is with `self.report(phase=..., ...)`, from any
thread, whenever something changes. A hook that reports nothing gets the phase the
protocol expects of its command (reset -> IDLE, goal -> PLANNING, clear_goal -> IDLE,
stop -> STOPPED); after a goal it is the agent's to report READY / DRIVING / ARRIVED.

Spec: docs/design/user-workflow.md, "The agent protocol (I3)" and "Protocol v2".
"""

import logging
import threading
from dataclasses import dataclass, field

from . import protocol as P


class Failed(Exception):
    """Raised by a hook: the command is answered FAILED with this message."""


@dataclass
class Episode:
    """One scenario run, as the agent learns of it at its reset.

    On a v1 relay, or from a v1-style `teleported`, only `pose` is known.
    """

    episode: str = ""
    scenario: str = ""
    map: str = ""
    pose: dict | None = None  # {x, y, z, qx, qy, qz, qw}, map frame
    parameters: dict = field(default_factory=dict)


# Hook -> the protocol commands it answers, and the capability it adds when overridden.
_OPTIONAL_HOOKS = {
    "on_reset": (P.RESET,),
    "on_restart": (P.RESTART,),
    "on_cooperate": (P.COOPERATE,),
    "on_cooperate_auto": (P.COOPERATE_AUTO,),
}


class Agent:
    """Base class. Override hooks; call `report()`; never block a hook forever."""

    #: Phase reported from registration until the agent reports one of its own.
    initial_phase = P.IDLE
    #: Name sent at registration (informational); the class name when empty.
    name = ""

    def __init__(self, config=None):
        self.config = dict(config or {})
        self.entity = self.config.get("entity", "ego")
        self.log = logging.getLogger(f"csb_agent.{type(self).__name__}")
        self.episode: Episode | None = None  # the current one, set before on_reset
        self._lock = threading.Lock()
        self._state = {"phase": self.initial_phase, "fault": P.FAULT_NONE, "detail": ""}
        self._reports = 0  # how many report() calls so far
        self._on_report = lambda: None  # set by the client: push the state now

    # -- hooks ------------------------------------------------------------------------------
    def on_reset(self, ep: Episode):
        """A scenario boundary: drop goal, plan and controller state; re-localize at
        `ep.pose`; return when ready for a goal."""
        raise NotImplementedError

    def on_goal(self, goal, waypoints):
        """Plan to `goal` through `waypoints` (poses) and drive there."""
        raise NotImplementedError

    def on_speed_limit(self, mps):
        """Drive no faster than `mps` (None: no limit)."""
        raise NotImplementedError

    def on_stop(self):
        """Stop driving and hold until the next goal."""
        raise NotImplementedError

    def on_clear_goal(self):
        """Drop the goal. Defaults to `on_stop` when only that is overridden."""
        if self.overrides("on_stop"):
            return self.on_stop()
        raise NotImplementedError

    def on_episode_end(self, ep: Episode, result: dict):
        """The scenario's verdict (`passed`, `failures`, `contacts`). Optional."""

    def on_restart(self):
        """Restart the autopilot in place (capability `restart`)."""
        raise NotImplementedError

    def on_cooperate(self, module, command, uuid=None):
        """RTC: ACTIVATE / DEACTIVATE `module` (capability `cooperate`)."""
        raise NotImplementedError

    def on_cooperate_auto(self, module, enable):
        """RTC auto mode for `module`."""
        raise NotImplementedError

    # -- state ------------------------------------------------------------------------------
    def report(self, phase=None, *, fault=None, fault_behavior=None, fault_progress=None,
               turn_indicators=None, detail=None, pose=None, rtc_status=None):
        """Update the state the relay sees; only the fields given change. Thread-safe."""
        if phase is not None and phase not in P.PHASES:
            raise ValueError(f"phase {phase!r} not in {P.PHASES}")
        fields = {"phase": phase, "fault": fault, "fault_behavior": fault_behavior,
                  "fault_progress": fault_progress, "turn_indicators": turn_indicators,
                  "detail": detail, "pose": pose, "rtc_status": rtc_status}
        with self._lock:
            for key, value in fields.items():
                if value is not None:
                    self._state[key] = value
            if fault == P.FAULT_NONE:  # a cleared fault takes its details with it
                self._state.pop("fault_behavior", None)
                self._state.pop("fault_progress", None)
            self._reports += 1
        self._on_report()

    @property
    def phase(self):
        with self._lock:
            return self._state["phase"]

    def state_message(self) -> dict:
        with self._lock:
            state = dict(self._state)
        return P.state(capabilities=self.capabilities(), **state)

    # -- introspection ----------------------------------------------------------------------
    def overrides(self, hook) -> bool:
        return getattr(type(self), hook) is not getattr(Agent, hook)

    def capabilities(self):
        """The commands this agent answers: the required ones plus each optional hook
        overridden (`reset`, `restart`, RTC)."""
        caps: list[str] = list(P.REQUIRED_COMMANDS)
        for hook, commands in _OPTIONAL_HOOKS.items():
            if self.overrides(hook):
                caps.extend(commands)
        return caps
