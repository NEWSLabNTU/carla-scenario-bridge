"""`AutowareAgent`: Autoware as a `swb_agent.Agent`, through its ADAPI (roadmap 030 step 6).

What SSv2's concealer used to do across domains, done on the vehicle side against
Autoware's own clock and sensors (moved here from autoware_carla_bridge's acb_pilot):

- reset (v2) / teleported (v1): stop and drop the route; wait until Autoware's services
  answer; wait for two LiDAR scans newer than the current estimate (the first after a
  respawn can belong to the previous vehicle); initialize localization at the pose; report
  INITIALIZING until the estimate agrees with it (1 m, 0.2 rad, newer than before). Only
  then IDLE.
- goal: (stop and clear, then) set the route; once Autoware reports autonomous mode
  available, engage by itself, retrying while availability flickers.
- the state the relay sees: the phase derived from Autoware's localization, routing and
  operation-mode states, its MRM and emergency as faults, turn indicators, RTC statuses,
  the localization estimate as the pose.

The steps are methods a subclass overrides -- `wait_ready(timeout)`, `relocalize(pose)`,
`set_route(goal, waypoints, allow_goal_modification, segments)`, `engage()` -- and the
bounds are config keys (DEFAULTS). The Autoware side is an `AutopilotPort`: the ROS one
(`autoware_ros.AutowarePort`, ADAPI in the vehicle's ROS domain) by default, a fake in the
tests. This module needs no ROS; the port does.

    ros2 run swb_agent autoware_agent --relay tcp://sim-host:5560 --entity ego
    (with use_sim_time: `--ros-args -p use_sim_time:=true`; see autoware_ros.main)
"""

import math
import threading
import time
from dataclasses import dataclass, field

import swb_agent
from swb_agent import protocol as P

# autoware_adapi_v1_msgs constants (duplicated so this module needs no ROS)
LOC_UNKNOWN, LOC_UNINITIALIZED, LOC_INITIALIZING, LOC_INITIALIZED = 0, 1, 2, 3
ROUTE_UNKNOWN, ROUTE_UNSET, ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING = 0, 1, 2, 3, 4
MODE_UNKNOWN, MODE_STOP, MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE = 0, 1, 2, 3, 4
MRM_UNKNOWN, MRM_NORMAL, MRM_OPERATING, MRM_SUCCEEDED, MRM_FAILED = 0, 1, 2, 3, 4
MRM_BEHAVIORS = {1: "NONE", 2: "EMERGENCY_STOP", 3: "COMFORTABLE_STOP", 4: "PULL_OVER"}
# autoware_vehicle_msgs/TurnIndicatorsCommand
TURN = {1: P.INDICATORS_NONE, 2: P.LEFT, 3: P.RIGHT}

DEFAULTS = {
    # the concealer's bounds (field_operator_application.cpp)
    "position_tolerance": 1.0,  # m: the estimate agrees with the reset pose
    "yaw_tolerance": 0.2,  # rad
    "scans_required": 2,  # fresh LiDAR scans before initializing localization
    "scan_wait": 10.0,  # s: initialize anyway after this without them
    # Localization has this long to agree with the pose; the concealer allowed 10 s after
    # WAITING_FOR_ROUTE, but that clock now only starts once the agent reports IDLE.
    "converge_timeout": 30.0,
    "initialize_attempts": 5,
    "initialize_retry": 2.0,  # s between refused initialize calls
    "ready_timeout": 60.0,  # s at a reset for Autoware's services to answer
    "engage_retry": 3.0,  # s between engage attempts while availability flickers
    "settle": 2.0,  # s a command waits for its effect; below the relay's 2.5 s window
    "step_period": 0.1,  # s; 0: no background thread (the tests step by hand)
}


@dataclass
class Snapshot:
    """What the autopilot reports, at one instant. Stamps are in the autopilot's time."""

    ready: bool = False  # the autopilot's core services are callable
    localization: int = LOC_UNKNOWN
    route: int = ROUTE_UNKNOWN
    mode: int = MODE_UNKNOWN
    control_enabled: bool = False
    autonomous_available: bool = False
    pose: dict | None = None  # localization estimate, protocol pose
    pose_stamp: float = 0.0
    scan_stamp: float = 0.0  # newest localization scan
    scan_count: int = 0
    mrm_state: int = MRM_UNKNOWN
    mrm_behavior: int = 0
    emergency: bool = False
    turn_indicators: int = 0  # 0 = no command
    rtc_status: list | None = None
    rtc: bool = False  # cooperate / cooperate_auto services exist


class AutopilotPort:
    """The Autoware interface the agent drives. Each call returns (ok, message)."""

    def snapshot(self) -> Snapshot:
        raise NotImplementedError

    def initialize(self, pose, stamp):
        raise NotImplementedError

    def set_route(self, goal, waypoints, allow_goal_modification, segments):
        raise NotImplementedError

    def clear_route(self):
        raise NotImplementedError

    def change_to_autonomous(self):
        raise NotImplementedError

    def change_to_stop(self):
        raise NotImplementedError

    def set_velocity_limit(self, mps):
        raise NotImplementedError

    def cooperate(self, module, command, uuid):
        raise NotImplementedError

    def cooperate_auto(self, module, enable):
        raise NotImplementedError

    def close(self):
        pass


def yaw_of(pose):
    return math.atan2(2.0 * (pose["qw"] * pose["qz"] + pose["qx"] * pose["qy"]),
                      1.0 - 2.0 * (pose["qy"] ** 2 + pose["qz"] ** 2))


def consistent(expected, actual, position_tolerance=1.0, yaw_tolerance=0.2):
    """The concealer's isLocalizationConsistentWith: planar distance and yaw."""
    yaw = math.remainder(yaw_of(actual) - yaw_of(expected), 2.0 * math.pi)
    return (math.hypot(actual["x"] - expected["x"], actual["y"] - expected["y"])
            <= position_tolerance and abs(yaw) <= yaw_tolerance)


def same_pose(a, b, tolerance=0.05):
    return a is not None and b is not None and all(
        abs(a[k] - b[k]) <= tolerance for k in P.POSE_KEYS)


def derive_phase(s: Snapshot, relocalizing: bool, stopped: bool) -> str:
    if not s.ready:
        return P.UNAVAILABLE
    if relocalizing or s.localization != LOC_INITIALIZED:
        return P.INITIALIZING
    if s.route == ROUTE_ARRIVED:
        return P.ARRIVED
    if s.route in (ROUTE_SET, ROUTE_CHANGING):
        if s.mode in (MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE) and s.control_enabled:
            return P.DRIVING
        if stopped:
            return P.STOPPED
        return P.READY if s.autonomous_available else P.PLANNING
    if s.route == ROUTE_UNSET:
        return P.IDLE
    return P.INITIALIZING


@dataclass
class Relocalization:
    pose: dict
    before: float  # estimate stamp at the start
    scan_count: int
    started: float
    stage: str = "scans"  # scans -> initialize -> converge -> done | failed
    fresh_scans: int = 0
    attempts: int = 0
    next_try: float = 0.0
    deadline: float = 0.0
    last_scan_count: int = field(default=0)
    failure: str = ""


class AutowareAgent(swb_agent.Agent):
    """Autoware through an `AutopilotPort`. See the module docstring."""

    name = "autoware"
    initial_phase = P.UNAVAILABLE  # until Autoware's services answer

    def __init__(self, config=None, port: AutopilotPort | None = None, clock=time.monotonic,
                 sleep=time.sleep):
        super().__init__(config)
        self.cfg = {**DEFAULTS, **{k: v for k, v in self.config.items() if k in DEFAULTS}}
        self.clock, self.sleep = clock, sleep
        self._closing = threading.Event()
        self.port = port if port is not None else self.make_port()
        self._core = threading.RLock()
        self.relocalization: Relocalization | None = None
        self.resetting = False  # on_reset runs: INITIALIZING whatever Autoware shows
        self.goal = None  # the active route's arguments
        self.stopped = False
        self.detail = ""
        self._last_engage = -math.inf
        self._thread = None
        if self.cfg["step_period"] > 0:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def make_port(self) -> AutopilotPort:
        """The ROS port over ADAPI (needs rclpy and Autoware's messages)."""
        from .autoware_ros import make_ros_port
        return make_ros_port(self)

    # -- state ----------------------------------------------------------------------------
    def derived_phase(self, snapshot=None) -> str:
        s = snapshot or self.port.snapshot()
        with self._core:
            return derive_phase(s, self.resetting or self.relocalization is not None,
                                self.stopped)

    def publish_state(self, snapshot=None):
        """Report what Autoware shows now: phase, faults, indicators, RTC, pose."""
        s = snapshot or self.port.snapshot()
        if s.emergency:
            fault, behavior, progress = P.EMERGENCY, MRM_BEHAVIORS.get(s.mrm_behavior), None
        elif s.mrm_state in (MRM_OPERATING, MRM_SUCCEEDED, MRM_FAILED):
            fault = P.MINIMAL_RISK_MANEUVER
            behavior = MRM_BEHAVIORS.get(s.mrm_behavior)
            progress = {MRM_OPERATING: P.OPERATING, MRM_SUCCEEDED: P.SUCCEEDED,
                        MRM_FAILED: P.FAILED}[s.mrm_state]
        else:
            fault, behavior, progress = P.FAULT_NONE, None, None
        with self._core:
            detail = self.detail
        self.report(self.derived_phase(s), fault=fault, fault_behavior=behavior,
                    fault_progress=progress, turn_indicators=TURN.get(s.turn_indicators),
                    detail=detail, pose=s.pose,
                    rtc_status=s.rtc_status if s.rtc else None)

    def _wait_for(self, predicate, timeout=None):
        """Let Autoware's state show a command's effect before replying (bounded).

        Not at all while Autoware has published no state yet: its ADAPI state is latched
        but only appears once /clock runs, i.e. once a scenario ticks the simulator, and a
        command that waited for it here would outlive the relay's reply window every time.
        """
        first = self.port.snapshot()
        if first.mode == MODE_UNKNOWN and first.route == ROUTE_UNKNOWN:
            return predicate(first)
        end = self.clock() + (self.cfg["settle"] if timeout is None else timeout)
        while self.clock() < end:
            if predicate(self.port.snapshot()):
                return True
            self.sleep(0.05)
        return predicate(self.port.snapshot())

    # -- steps a subclass may override --------------------------------------------------
    def wait_ready(self, timeout):
        """Until Autoware's core services answer, or Failed."""
        end = self.clock() + timeout
        while not self.port.snapshot().ready:
            if self.clock() >= end:
                raise swb_agent.Failed(f"Autoware's services did not answer in {timeout:.0f} s")
            self.sleep(0.2)

    def relocalize(self, pose):
        """Re-localize at `pose`; return when the estimate agrees with it, or Failed."""
        s = self.port.snapshot()
        with self._core:
            self.relocalization = Relocalization(
                pose=pose, before=s.pose_stamp, scan_count=s.scan_count,
                started=self.clock(), last_scan_count=s.scan_count)
            self.detail = "re-localizing at the reset pose"
        self.log.info(f"re-localizing at ({pose['x']:.2f}, {pose['y']:.2f}, yaw "
                      f"{yaw_of(pose):.2f})")
        while True:
            with self._core:
                r = self.relocalization
            if r is None or r.stage in ("done", "failed"):
                break
            self._step_relocalization(r)
            if r.stage not in ("done", "failed"):
                self.sleep(0.1)
        with self._core:
            self.relocalization = None
            self.detail = r.failure if r is not None else ""
        if r is not None and r.stage == "failed":
            raise swb_agent.Failed(r.failure)

    def set_route(self, goal, waypoints, allow_goal_modification=False, segments=None):
        """Set Autoware's route (clearing a previous one first), or Failed."""
        s = self.port.snapshot()
        if s.route in (ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING):
            ok, message = self._clear(s)
            if not ok:
                raise swb_agent.Failed(f"could not clear the previous route: {message}")
        ok, message = self.port.set_route(goal, list(waypoints), allow_goal_modification,
                                          segments)
        if not ok:
            raise swb_agent.Failed(message)
        self._wait_for(lambda s: s.route in (ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING))

    def engage(self):
        """Ask Autoware for autonomous mode: (ok, message). Called while the goal is wanted
        and Autoware is READY, every `engage_retry` s until it drives."""
        return self.port.change_to_autonomous()

    # -- hooks ------------------------------------------------------------------------------
    def on_reset(self, ep):
        with self._core:
            self.goal = None
            self.stopped = False
            self.resetting = True
        try:
            self._reset(ep)
        finally:
            with self._core:
                self.resetting = False
            self.publish_state()

    def _reset(self, ep):
        self.wait_ready(float(self.cfg["ready_timeout"]))
        s = self.port.snapshot()
        if s.route in (ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING) or s.mode in (
                MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE):
            ok, message = self._clear(s)
            if not ok:
                self.log.warning(f"reset: previous route not cleared ({message}); "
                                 "re-localizing drops it")
        if ep.pose is None:
            raise swb_agent.Failed("reset without a pose")
        self.relocalize(ep.pose)

    def on_goal(self, goal, waypoints, allow_goal_modification=False, segments=None):
        s = self.port.snapshot()
        phase = self.derived_phase(s)
        if phase in (P.UNAVAILABLE, P.INITIALIZING):
            raise swb_agent.Failed(f"not ready for a goal: {phase}")
        with self._core:
            if (self.goal is not None and same_pose(self.goal["goal"], goal)
                    and phase in (P.PLANNING, P.READY, P.DRIVING)):
                self.publish_state()
                return  # the concealer's retry of a goal already being driven to
        self.set_route(goal, waypoints, allow_goal_modification, segments)
        with self._core:
            self.goal = {"goal": goal, "waypoints": list(waypoints), "segments": segments}
            self.stopped = False
            self.detail = ""
            self._last_engage = -math.inf
        self.log.info(f"goal set: ({goal['x']:.2f}, {goal['y']:.2f}); engaging once ready")
        self.publish_state()

    def on_clear_goal(self):
        with self._core:
            self.goal = None
        s = self.port.snapshot()
        if s.route != ROUTE_UNSET:
            ok, message = self._clear(s)
            if not ok:
                raise swb_agent.Failed(message)
        self.publish_state()

    def on_stop(self):
        with self._core:
            self.stopped = True
        if self.port.snapshot().mode != MODE_STOP:
            ok, message = self.port.change_to_stop()
            if not ok:
                raise swb_agent.Failed(message)
            self._wait_for(lambda s: s.mode == MODE_STOP)
        self.publish_state()

    def on_speed_limit(self, mps):
        if mps is None:
            return
        ok, message = self.port.set_velocity_limit(float(mps))
        if not ok:
            raise swb_agent.Failed(message)

    def on_cooperate(self, module, command, uuid=None):
        if not self.port.snapshot().rtc:
            raise swb_agent.Failed("this Autoware offers no RTC commands")
        ok, message = self.port.cooperate(module, command, uuid)
        if not ok:
            raise swb_agent.Failed(message)

    def on_cooperate_auto(self, module, enable):
        if not self.port.snapshot().rtc:
            raise swb_agent.Failed("this Autoware offers no RTC auto mode")
        ok, message = self.port.cooperate_auto(module, enable)
        if not ok:
            raise swb_agent.Failed(message)

    def capabilities(self):
        """RTC only when this Autoware offers its services."""
        caps = super().capabilities()
        if not self.port.snapshot().rtc:
            caps = [c for c in caps if c not in (P.COOPERATE, P.COOPERATE_AUTO)]
        return caps

    def close(self):
        self._closing.set()
        self.port.close()

    # -- internals ------------------------------------------------------------------------
    def _clear(self, s):
        """Clear Autoware's route, stopping first: the routing API refuses it while
        autonomous."""
        if s.mode in (MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE):
            ok, message = self.port.change_to_stop()
            if not ok:
                return False, f"change_to_stop: {message}"
            self._wait_for(lambda s: s.mode == MODE_STOP)
        ok, message = self.port.clear_route()
        if not ok and self.port.snapshot().route != ROUTE_UNSET:
            return False, message
        self._wait_for(lambda s: s.route == ROUTE_UNSET)
        return True, ""

    def _run(self):
        while not self._closing.wait(self.cfg["step_period"]):
            try:
                self.step()
            except Exception as e:  # pragma: no cover - keep the loop alive
                self.log.warning(f"step failed: {e}")

    def step(self):
        """Engage a ready goal and report the state. A few times a second."""
        s = self.port.snapshot()
        with self._core:
            relocalizing = self.relocalization is not None
            want = self.goal is not None and not self.stopped
        if not relocalizing and want and self.derived_phase(s) == P.READY and \
                self.clock() - self._last_engage >= self.cfg["engage_retry"]:
            self._last_engage = self.clock()
            ok, message = self.engage()
            self.log.info("engaged" if ok else f"engage refused ({message}); retrying")
        self.publish_state()

    def _step_relocalization(self, r: Relocalization):
        c = self.cfg
        now = self.clock()
        s = self.port.snapshot()
        if r.stage == "scans":
            if s.scan_count != r.last_scan_count:
                if s.scan_stamp > r.before:
                    r.fresh_scans += s.scan_count - r.last_scan_count
                r.last_scan_count = s.scan_count
            if r.fresh_scans >= c["scans_required"] or now - r.started >= c["scan_wait"]:
                if r.fresh_scans < c["scans_required"]:
                    self.log.info(f"only {r.fresh_scans} fresh scan(s) in "
                                  f"{c['scan_wait']:.0f} s; initializing localization anyway")
                r.stage, r.next_try = "initialize", now
        if r.stage == "initialize" and now >= r.next_try:
            r.attempts += 1
            ok, message = self.port.initialize(r.pose, s.scan_stamp or None)
            if ok:
                r.stage, r.deadline = "converge", self.clock() + c["converge_timeout"]
            elif r.attempts >= c["initialize_attempts"]:
                r.stage = "failed"
                r.failure = f"localization initialize refused: {message}"
                self.log.warning(f"localization initialize refused {r.attempts} times: "
                                 f"{message}")
                return
            else:
                r.next_try = now + c["initialize_retry"]
        if r.stage == "converge":
            s = self.port.snapshot()
            if (s.localization == LOC_INITIALIZED and s.pose is not None
                    and s.pose_stamp > r.before
                    and consistent(r.pose, s.pose, c["position_tolerance"],
                                   c["yaw_tolerance"])):
                self.log.info(f"localized at the reset pose after "
                              f"{self.clock() - r.started:.1f} s")
                r.stage = "done"
            elif self.clock() >= r.deadline:
                where = ("no estimate" if s.pose is None else
                         f"estimate at ({s.pose['x']:.2f}, {s.pose['y']:.2f})")
                r.stage = "failed"
                r.failure = (f"localization did not reach the reset pose in "
                             f"{c['converge_timeout']:.0f} s ({where})")
                self.log.warning(r.failure)
