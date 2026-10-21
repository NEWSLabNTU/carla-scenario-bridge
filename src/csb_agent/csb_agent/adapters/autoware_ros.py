"""`AutowarePort`: the AutowareAgent's port over Autoware's ADAPI, in the vehicle's own ROS
domain (moved from autoware_carla_bridge's acb_pilot, roadmap 030 step 6).

The same services and topics SSv2's concealer used to call across domains (see
scenario_simulator_v2 concealer/field_operator_application.cpp), now called locally.

    ros2 run csb_agent autoware_agent --relay tcp://sim-host:5560 --entity ego \
        [--config agent.yaml]          # use_sim_time: true in the config for a simulator
"""

import struct
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time

from autoware_adapi_v1_msgs.msg import (LocalizationInitializationState, MrmState,
                                        OperationModeState, RouteState)
from autoware_adapi_v1_msgs.msg import RoutePrimitive, RouteSegment
from autoware_adapi_v1_msgs.srv import (ChangeOperationMode, ClearRoute,
                                        InitializeLocalization, SetRoute, SetRoutePoints)
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

import csb_agent

from .autoware import AutopilotPort, AutowareAgent, Snapshot

# tier4 interfaces are optional: the agent degrades (no RTC, no emergency flag, no speed
# limit) on an Autoware without them rather than refusing to start.
try:
    from tier4_external_api_msgs.msg import Emergency
    from tier4_external_api_msgs.msg import ResponseStatus as Tier4Status
    from tier4_external_api_msgs.srv import SetVelocityLimit
except ImportError:  # pragma: no cover - depends on the installed Autoware
    Emergency = SetVelocityLimit = Tier4Status = None
try:
    from tier4_rtc_msgs.msg import CooperateCommand, CooperateStatusArray
    from tier4_rtc_msgs.srv import AutoModeWithModule, CooperateCommands
except ImportError:  # pragma: no cover
    CooperateStatusArray = CooperateCommands = AutoModeWithModule = None

RTC_MODULES = {
    "NONE": 0, "LANE_CHANGE_LEFT": 1, "LANE_CHANGE_RIGHT": 2, "AVOIDANCE_LEFT": 3,
    "AVOIDANCE_RIGHT": 4, "GOAL_PLANNER": 5, "START_PLANNER": 6, "TRAFFIC_LIGHT": 7,
    "INTERSECTION": 8, "INTERSECTION_OCCLUSION": 9, "CROSSWALK": 10, "BLIND_SPOT": 11,
    "DETECTION_AREA": 12, "NO_STOPPING_AREA": 13, "OCCLUSION_SPOT": 14,
    "EXT_REQUEST_LANE_CHANGE_LEFT": 15, "EXT_REQUEST_LANE_CHANGE_RIGHT": 16,
    "AVOIDANCE_BY_LC_LEFT": 17, "AVOIDANCE_BY_LC_RIGHT": 18, "ROUNDABOUT": 19,
}
RTC_MODULE_NAMES = {v: k for k, v in RTC_MODULES.items()}
RTC_STATES = {0: "WAITING_FOR_EXECUTION", 1: "RUNNING", 2: "ABORTING", 3: "SUCCEEDED",
              4: "FAILED"}

# The services without which the agent cannot do its job; until all answer, the agent
# reports UNAVAILABLE and the relay withholds the ADAPI from the scenario.

# How long one /api/localization/initialize may take. The call returns when NDT's align
# server has searched its particles (200 by default), which is CPU-bound wall time: 9.4 s
# measured on a loaded host (the safety island demo, unit C3), where 8 s timed out every
# attempt and each retry queued another align behind the first. On Isaac Sim (csb roadmap
# 043) a freshly started stack on a host at load 40-60 kept NDT aligning for well over 30 s,
# and each expired call again queued a retry behind the first. 60 s outlasts that; the
# scenario's initialize_duration (120 s) bounds the whole start anyway.
INITIALIZE_TIMEOUT = 60.0

CORE_SERVICES = ("initialize", "set_route_points", "clear_route", "change_to_autonomous",
                 "change_to_stop")


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cdr_header_stamp(data: bytes) -> float:
    """header.stamp of a serialized message whose first field is a std_msgs/Header.

    The scan is only needed for its stamp; reading it from the raw CDR spares the agent
    deserializing a point cloud ten times a second.
    """
    little = data[1] == 1  # encapsulation: 0x0001 = CDR little-endian
    sec, nanosec = struct.unpack_from("<iI" if little else ">iI", data, 4)
    return sec + nanosec * 1e-9


def _pose_msg(d, pose):
    pose.position.x, pose.position.y, pose.position.z = (float(d["x"]), float(d["y"]),
                                                         float(d["z"]))
    pose.orientation.x, pose.orientation.y = float(d["qx"]), float(d["qy"])
    pose.orientation.z, pose.orientation.w = float(d["qz"]), float(d["qw"])
    return pose


def _pose_dict(pose):
    return {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "qx": pose.orientation.x, "qy": pose.orientation.y,
            "qz": pose.orientation.z, "qw": pose.orientation.w}


class AutowarePort(AutopilotPort):
    def __init__(self, node, call_timeout=5.0):
        self.node = node
        self.call_timeout = call_timeout
        self._lock = threading.Lock()
        self._s = Snapshot()
        self._have = set()  # which state topics have arrived
        self._ready_checked = 0.0
        self._services_ready = False
        self._rtc_ready = False
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        volatile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        sub = node.create_subscription
        sub(LocalizationInitializationState, "/api/localization/initialization_state",
            self._on_localization, latched, callback_group=group)
        sub(RouteState, "/api/routing/state", self._on_route, latched, callback_group=group)
        sub(OperationModeState, "/api/operation_mode/state", self._on_mode, latched,
            callback_group=group)
        sub(Odometry, "/localization/kinematic_state", self._on_kinematic, volatile,
            callback_group=group)
        sub(PointCloud2, "/localization/util/downsample/pointcloud", self._on_scan,
            qos_profile_sensor_data, callback_group=group, raw=True)
        sub(MrmState, "/api/fail_safe/mrm_state", self._on_mrm, volatile,
            callback_group=group)
        sub(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", self._on_turn,
            volatile, callback_group=group)
        if Emergency is not None:
            sub(Emergency, "/api/external/get/emergency", self._on_emergency, volatile,
                callback_group=group)
        if CooperateStatusArray is not None:
            sub(CooperateStatusArray, "/api/external/get/rtc_status", self._on_rtc,
                volatile, callback_group=group)

        client = lambda t, name: node.create_client(t, name, callback_group=group)  # noqa
        self.clients = {
            "initialize": client(InitializeLocalization, "/api/localization/initialize"),
            "set_route_points": client(SetRoutePoints, "/api/routing/set_route_points"),
            "set_route": client(SetRoute, "/api/routing/set_route"),
            "clear_route": client(ClearRoute, "/api/routing/clear_route"),
            "change_to_autonomous": client(ChangeOperationMode,
                                           "/api/operation_mode/change_to_autonomous"),
            "change_to_stop": client(ChangeOperationMode,
                                     "/api/operation_mode/change_to_stop"),
        }
        if SetVelocityLimit is not None:
            self.clients["velocity_limit"] = client(SetVelocityLimit,
                                                    "/api/autoware/set/velocity_limit")
        if CooperateCommands is not None:
            self.clients["rtc_commands"] = client(CooperateCommands,
                                                  "/api/external/set/rtc_commands")
            self.clients["rtc_auto_mode"] = client(AutoModeWithModule,
                                                   "/api/external/set/rtc_auto_mode")

    # -- subscriptions ------------------------------------------------------------------
    def _on_localization(self, msg):
        with self._lock:
            self._s.localization = msg.state
            self._have.add("localization")

    def _on_route(self, msg):
        with self._lock:
            self._s.route = msg.state
            self._have.add("route")

    def _on_mode(self, msg):
        with self._lock:
            self._s.mode = msg.mode
            self._s.control_enabled = msg.is_autoware_control_enabled
            self._s.autonomous_available = msg.is_autonomous_mode_available
            self._have.add("mode")

    def _on_kinematic(self, msg):
        with self._lock:
            self._s.pose = _pose_dict(msg.pose.pose)
            self._s.pose_stamp = stamp_seconds(msg.header.stamp)

    def _on_scan(self, data):
        try:
            stamp = cdr_header_stamp(data)
        except (struct.error, IndexError):
            return
        with self._lock:
            self._s.scan_stamp = stamp
            self._s.scan_count += 1

    def _on_mrm(self, msg):
        with self._lock:
            self._s.mrm_state, self._s.mrm_behavior = msg.state, msg.behavior

    def _on_turn(self, msg):
        with self._lock:
            self._s.turn_indicators = msg.command

    def _on_emergency(self, msg):
        with self._lock:
            self._s.emergency = msg.emergency

    def _on_rtc(self, msg):
        statuses = [{
            "module": RTC_MODULE_NAMES.get(s.module.type, str(s.module.type)),
            "uuid": bytes(s.uuid.uuid).hex(), "safe": s.safe, "requested": s.requested,
            "command_status": "ACTIVATE" if s.command_status.type == 1 else "DEACTIVATE",
            "state": RTC_STATES.get(s.state.type, str(s.state.type)),
            "auto_mode": s.auto_mode, "start_distance": float(s.start_distance),
            "finish_distance": float(s.finish_distance)} for s in msg.statuses]
        with self._lock:
            self._s.rtc_status = statuses

    # -- port -----------------------------------------------------------------------------
    def snapshot(self):
        now = time.monotonic()
        if now - self._ready_checked >= 1.0:
            self._ready_checked = now
            self._services_ready = all(self.clients[n].service_is_ready()
                                       for n in CORE_SERVICES)
            self._rtc_ready = ("rtc_commands" in self.clients
                               and self.clients["rtc_commands"].service_is_ready()
                               and self.clients["rtc_auto_mode"].service_is_ready())
        with self._lock:
            s = Snapshot(**vars(self._s))
            # Services alone, not the state topics: Autoware's ADAPI state is latched but
            # only published once /clock runs, and /clock runs only once a scenario ticks
            # CARLA -- a scenario the relay would not start while this reported UNAVAILABLE.
            # Until the states arrive the phase is INITIALIZING (localization unknown).
            s.ready = self._services_ready
            s.rtc = self._rtc_ready
            if not s.rtc:
                s.rtc_status = None
            return s

    def _call(self, name, request, timeout=None):
        client = self.clients.get(name)
        if client is None:
            return None, f"{name}: not offered by this Autoware"
        if not client.service_is_ready():
            return None, f"{client.srv_name}: service not available"
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout or self.call_timeout):
            future.cancel()
            return None, f"{client.srv_name}: no response in {timeout or self.call_timeout} s"
        return future.result(), ""

    @staticmethod
    def _adapi(result):
        response, error = result
        if response is None:
            return False, error
        return response.status.success, response.status.message

    @staticmethod
    def _tier4(result):
        response, error = result
        if response is None:
            return False, error
        return response.status.code == Tier4Status.SUCCESS, response.status.message

    def initialize(self, pose, stamp):
        request = InitializeLocalization.Request()
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        # Autoware's time base: the newest scan's stamp when there is one, else the
        # node's (sim) clock.
        p.header.stamp = (Time(seconds=stamp).to_msg() if stamp
                          else self.node.get_clock().now().to_msg())
        _pose_msg(pose, p.pose.pose)
        request.pose = [p]
        return self._adapi(self._call("initialize", request, timeout=INITIALIZE_TIMEOUT))

    def set_route(self, goal, waypoints, allow_goal_modification, segments):
        if segments:
            request = SetRoute.Request()
            request.segments = [RouteSegment(
                preferred=RoutePrimitive(id=s["id"], type=s.get("type", "lane")),
                alternatives=[RoutePrimitive(id=a["id"], type=a.get("type", "lane"))
                              for a in s.get("alternatives", [])]) for s in segments]
            name = "set_route"
        else:
            request = SetRoutePoints.Request()
            request.waypoints = [_pose_msg(w, type(request.goal)()) for w in waypoints]
            name = "set_route_points"
        request.header.frame_id = "map"
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.option.allow_goal_modification = bool(allow_goal_modification)
        _pose_msg(goal, request.goal)
        return self._adapi(self._call(name, request, timeout=6.0))

    def clear_route(self):
        return self._adapi(self._call("clear_route", ClearRoute.Request(), timeout=2.0))

    def change_to_autonomous(self):
        return self._adapi(self._call("change_to_autonomous",
                                      ChangeOperationMode.Request()))

    def change_to_stop(self):
        return self._adapi(self._call("change_to_stop", ChangeOperationMode.Request(),
                                      timeout=2.0))

    def set_velocity_limit(self, mps):
        if SetVelocityLimit is None:
            return False, "this Autoware has no velocity limit API"
        return self._tier4(self._call("velocity_limit",
                                      SetVelocityLimit.Request(velocity=float(mps)),
                                      timeout=2.0))

    def cooperate(self, module, command, uuid):
        if module not in RTC_MODULES:
            return False, f"unknown RTC module {module}"
        item = CooperateCommand()
        item.module.type = RTC_MODULES[module]
        item.command.type = 1 if command == "ACTIVATE" else 0
        if uuid:
            item.uuid.uuid = list(bytes.fromhex(uuid))
        request = CooperateCommands.Request(commands=[item])
        request.stamp = self.node.get_clock().now().to_msg()
        response, error = self._call("rtc_commands", request, timeout=2.0)
        if response is None:
            return False, error
        return all(r.success for r in response.responses), ""

    def cooperate_auto(self, module, enable):
        if module not in RTC_MODULES:
            return False, f"unknown RTC module {module}"
        request = AutoModeWithModule.Request(enable=bool(enable))
        request.module.type = RTC_MODULES[module]
        response, error = self._call("rtc_auto_mode", request, timeout=2.0)
        if response is None:
            return False, error
        return response.success, ""


class _NodePort(AutowarePort):
    """An AutowarePort on a node of its own, spun on a thread; closed with the agent."""

    def __init__(self, name, use_sim_time):
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node(name, parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))])
        super().__init__(node)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def close(self):
        self._executor.shutdown()
        self.node.destroy_node()


def make_ros_port(agent):
    """The default port of an AutowareAgent: its own node (`node_name`, default
    `csb_autoware_agent`), `use_sim_time` from the config (default false)."""
    return _NodePort(agent.config.get("node_name", "csb_autoware_agent"),
                     agent.config.get("use_sim_time", False))


def main(argv=None):
    """`ros2 run csb_agent autoware_agent --relay ... --entity ... [--config ...]`."""
    return csb_agent.main(AutowareAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
