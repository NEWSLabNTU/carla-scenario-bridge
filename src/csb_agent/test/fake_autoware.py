"""A fake Autoware behind the AutopilotPort, on a fake clock: enough of its routing,
operation-mode and localization behaviour to drive AutowareAgent through a scenario.
Moved from autoware_carla_bridge's acb_pilot (roadmap 030 step 6)."""

from csb_agent.adapters import autoware as C


class Clock:
    def __init__(self):
        self.t = 100.0
        self.hooks = []

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.advance(dt)

    def advance(self, dt):
        self.t += dt
        for hook in self.hooks:
            hook()


class FakeAutoware(C.AutopilotPort):
    PLAN_TIME = 1.0  # route SET -> autonomous mode available
    CONVERGE_TIME = 1.0  # initialize -> estimate at the pose

    def __init__(self, clock, rtc=False):
        self.clock = clock
        self.s = C.Snapshot(ready=True, localization=C.LOC_INITIALIZED, route=C.ROUTE_UNSET,
                            mode=C.MODE_STOP, mrm_state=C.MRM_NORMAL, mrm_behavior=1,
                            pose={"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0,
                                  "qw": 1},
                            pose_stamp=50.0, scan_stamp=50.0, rtc=rtc)
        self.calls = []
        self.route_set_at = None
        self.initialized_at = None
        self.init_pose = None
        self.refuse = set()
        self.publish_scans = True
        clock.hooks.append(self.tick)

    def tick(self):
        t = self.clock()
        self.s.pose_stamp = t  # the EKF keeps publishing
        if self.publish_scans:
            self.s.scan_stamp, self.s.scan_count = t, self.s.scan_count + 1
        if self.route_set_at is not None and t - self.route_set_at >= self.PLAN_TIME:
            self.s.autonomous_available = True
        if self.initialized_at is not None and t - self.initialized_at >= self.CONVERGE_TIME:
            self.s.localization = C.LOC_INITIALIZED
            self.s.pose = dict(self.init_pose)
            self.initialized_at = None

    def snapshot(self):
        return C.Snapshot(**vars(self.s))

    def _call(self, name):
        self.calls.append(name)
        if name in self.refuse:
            return False, f"{name} refused"
        return True, ""

    def initialize(self, pose, stamp):
        ok, msg = self._call("initialize")
        if ok:
            self.s.localization = C.LOC_INITIALIZING
            self.s.route = C.ROUTE_UNSET
            self.initialized_at, self.init_pose = self.clock(), pose
            self.init_stamp = stamp
        return ok, msg

    def set_route(self, goal, waypoints, allow_goal_modification, segments):
        ok, msg = self._call("set_route")
        if ok:
            if self.s.route != C.ROUTE_UNSET:
                return False, "route exists"
            self.s.route = C.ROUTE_SET
            self.route_set_at = self.clock()
            self.goal = goal
        return ok, msg

    def clear_route(self):
        ok, msg = self._call("clear_route")
        if ok:
            if self.s.mode == C.MODE_AUTONOMOUS:
                return False, "cannot clear while autonomous"
            self.s.route = C.ROUTE_UNSET
            self.route_set_at = None
            self.s.autonomous_available = False
        return ok, msg

    def change_to_autonomous(self):
        ok, msg = self._call("change_to_autonomous")
        if ok:
            if not self.s.autonomous_available:
                return False, "not available"
            self.s.mode, self.s.control_enabled = C.MODE_AUTONOMOUS, True
        return ok, msg

    def change_to_stop(self):
        ok, msg = self._call("change_to_stop")
        if ok:
            self.s.mode = C.MODE_STOP
        return ok, msg

    def set_velocity_limit(self, mps):
        self.velocity_limit = mps
        return self._call("velocity_limit")

    def cooperate(self, module, command, uuid):
        return self._call("cooperate")

    def cooperate_auto(self, module, enable):
        return self._call("cooperate_auto")

    def arrive(self):
        self.s.route = C.ROUTE_ARRIVED
