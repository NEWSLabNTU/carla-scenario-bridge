"""ros_adapter against a small test autopilot node: YAML only, and a subclass changing one
step. Runs in its own ROS domain on localhost so it never meets a live stack."""

import os

os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import threading  # noqa: E402

import pytest  # noqa: E402

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from csb_agent import protocol as P  # noqa: E402
from csb_agent.check import run_check  # noqa: E402
from csb_agent.ros_adapter import ConfigError, RosAdapter  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakeAutopilot:
    """idle -> (goal) driving -> (stop) stopped; reset -> idle once /initialpose arrives."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("test_autopilot")
        self.state = "idle"
        self.goals, self.limits, self.initial_poses = [], [], []
        self.pub = self.node.create_publisher(String, "/autopilot/state", 10)
        n = self.node
        n.create_subscription(PoseStamped, "/planning/goal", self._goal, LATCHED)
        n.create_subscription(Float32, "/planning/max_velocity",
                              lambda m: self.limits.append(m.data), LATCHED)
        n.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initial, LATCHED)
        n.create_service(Trigger, "/autopilot/stop", self._stop)
        n.create_service(Trigger, "/autopilot/reset", self._reset)
        n.create_timer(0.1, lambda: self.pub.publish(String(data=self.state)))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _goal(self, msg):
        self.goals.append(msg.pose.position.x)
        self.state = "driving"

    def _initial(self, msg):
        self.initial_poses.append(msg.pose.pose.position.x)
        self.state = "idle"

    def _stop(self, _req, res):
        self.state = "stopped"
        res.success = True
        return res

    def _reset(self, _req, res):
        self.state = "initializing"
        res.success = True
        return res

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()


CONFIG = {
    "entity": "ego",
    "goal": {"publish": "/planning/goal", "type": "geometry_msgs/msg/PoseStamped",
             "from": "goal"},
    "speed_limit": {"publish": "/planning/max_velocity", "type": "std_msgs/msg/Float32",
                    "from": "mps", "null": 1000.0},
    "stop": {"call": "/autopilot/stop", "type": "std_srvs/srv/Trigger"},
    "reset": [{"call": "/autopilot/reset", "type": "std_srvs/srv/Trigger"},
              {"wait_phase": "INITIALIZING", "timeout": 5},
              {"publish": "/initialpose",
               "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "from": "pose"},
              {"wait_phase": "IDLE", "timeout": 5}],
    "phase": {"subscribe": "/autopilot/state", "type": "std_msgs/msg/String",
              "map": {"idle": "IDLE", "initializing": "INITIALIZING", "driving": "DRIVING",
                      "stopped": "STOPPED"}},
}


@pytest.fixture
def autopilot():
    a = FakeAutopilot()
    yield a
    a.close()


def test_yaml_only_adapter_passes_check(autopilot):
    report = run_check(RosAdapter, CONFIG, reset_timeout=10.0)
    assert report.passed, report.text()
    assert autopilot.goals == [50.0]
    assert autopilot.limits == [5.0]
    assert len(autopilot.initial_poses) == 2  # two resets


def test_reset_is_only_offered_when_mapped(autopilot):
    config = {k: v for k, v in CONFIG.items() if k != "reset"}
    adapter = RosAdapter(config)
    try:
        assert P.RESET not in adapter.capabilities()
    finally:
        adapter.close()


def test_a_lifted_limit_publishes_the_null_value(autopilot):
    adapter = RosAdapter(CONFIG)
    try:
        adapter.on_speed_limit(None)
        import time
        deadline = time.monotonic() + 3
        while not autopilot.limits and time.monotonic() < deadline:
            time.sleep(0.05)
        assert autopilot.limits == [1000.0]
    finally:
        adapter.close()


class GoalAhead(RosAdapter):
    """One step changed in code: the goal is moved 10 m ahead, the rest is the YAML's."""

    def on_goal(self, goal, waypoints):
        self.run_hook("goal", goal={**goal, "x": goal["x"] + 10.0}, waypoints=waypoints)


def test_a_subclass_overrides_one_step(autopilot):
    report = run_check(GoalAhead, CONFIG, reset_timeout=10.0)
    assert report.passed, report.text()
    assert autopilot.goals == [60.0]


@pytest.mark.parametrize("bad", [
    {"stop": {"call": "/x"}},                                  # no type
    {"stop": {"frobnicate": 1}},                               # not an action
    {"reset": {"wait_phase": "IDLE"}},                         # no phase subscription
])
def test_bad_configs_are_refused(bad):
    with pytest.raises(ConfigError):
        RosAdapter({"entity": "ego", **bad})
