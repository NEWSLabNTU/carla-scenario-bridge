"""scenario_agent_relay: serves the concealer's ADAPI subset (I2) for one entity, backed by
the agent registered under that entity's name (I3).

Agents registered under other names are served to commanders (a simulator adapter driving
entities whose controller is `agent`) by the AgentServer itself; this node adds nothing.

Simulator-neutral and autopilot-neutral: nothing here knows CARLA, and Autoware appears
only as the shape of the ROS interface SSv2's concealer expects in the scenario domain.
See docs/design/user-workflow.md, "The agent protocol (I3)".
"""

import math
import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from autoware_adapi_v1_msgs.msg import (LocalizationInitializationState, MrmState,
                                        OperationModeState, RouteState)
from autoware_adapi_v1_msgs.srv import (ChangeOperationMode, ClearRoute,
                                        InitializeLocalization, SetRoute, SetRoutePoints)
from autoware_system_msgs.msg import AutowareState
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tier4_external_api_msgs.msg import Emergency
from tier4_external_api_msgs.msg import ResponseStatus as Tier4Status
from tier4_external_api_msgs.srv import Engage, SetVelocityLimit
from tier4_rtc_msgs.msg import CooperateResponse, CooperateStatus, CooperateStatusArray
from tier4_rtc_msgs.srv import AutoModeWithModule, CooperateCommands

from . import mapping as M
from . import protocol as P
from .server import AgentServer

# How long a service handler waits for the agent's reply. Each is under the concealer's
# own per-attempt wait (concealer/service.hpp: 3 s by default, 10 s for the two routing
# services and initialize_localization's 10 s), so a slow answer is a failed attempt the
# concealer retries rather than a reply that lands after it gave up.
QUICK_REPLY = 2.5
SLOW_REPLY = 9.0

RTC_STATES = {"WAITING_FOR_EXECUTION": 0, "RUNNING": 1, "ABORTING": 2, "SUCCEEDED": 3,
              "FAILED": 4}


def pose_to_dict(pose):
    return {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "qx": pose.orientation.x, "qy": pose.orientation.y,
            "qz": pose.orientation.z, "qw": pose.orientation.w}


def dict_to_pose(d, pose):
    pose.position.x, pose.position.y, pose.position.z = (float(d["x"]), float(d["y"]),
                                                         float(d["z"]))
    pose.orientation.x, pose.orientation.y = float(d["qx"]), float(d["qy"])
    pose.orientation.z, pose.orientation.w = float(d["qz"]), float(d["qw"])
    return pose


class RelayNode(Node):
    def __init__(self):
        super().__init__("scenario_agent_relay")
        self.declare_parameter("entity", "ego")
        self.declare_parameter("agent_host", "0.0.0.0")
        self.declare_parameter("agent_port", 5560)
        self.declare_parameter("agent_timeout", P.PEER_TIMEOUT)
        self.declare_parameter("publish_rate", 10.0)
        self.entity = self.get_parameter("entity").value
        host = self.get_parameter("agent_host").value
        port = int(self.get_parameter("agent_port").value)
        timeout = float(self.get_parameter("agent_timeout").value)
        rate = float(self.get_parameter("publish_rate").value)

        self._lock = threading.Lock()
        self._publish_lock = threading.Lock()
        self._state = None  # latest state of self.entity's agent, None when absent
        self._pose_stamp = None  # ROS time the latest pose arrived
        self._phase = None
        self._arrived_at = None  # ROS time the agent was first seen ARRIVED
        self._latched = {}  # topic -> last published key

        self._group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        volatile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        pub = self.create_publisher
        self.pub_localization = pub(LocalizationInitializationState,
                                    "/api/localization/initialization_state", latched)
        self.pub_route = pub(RouteState, "/api/routing/state", latched)
        self.pub_mode = pub(OperationModeState, "/api/operation_mode/state", latched)
        self.pub_mrm = pub(MrmState, "/api/fail_safe/mrm_state", volatile)
        self.pub_emergency = pub(Emergency, "/api/external/get/emergency", volatile)
        self.pub_legacy = pub(AutowareState, "/autoware/state", volatile)
        self.pub_turn = pub(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd",
                            volatile)
        self.pub_rtc = pub(CooperateStatusArray, "/api/external/get/rtc_status", volatile)
        self.pub_kinematic = pub(Odometry, "/localization/kinematic_state", volatile)
        # The concealer counts and stamps "scans" here: it waits for two newer than the
        # last kinematic state before initializing, and measures ARRIVED_GOAL's 2 s
        # window against the newest. Both only need a clock that is the same one the
        # relay stamps kinematic_state and routing/state with, so the relay publishes an
        # empty cloud stamped with its own time. The real scan wait happens on the
        # vehicle side, in the agent, against the autopilot's own sensors.
        self.pub_scan = pub(PointCloud2, "/localization/util/downsample/pointcloud",
                            qos_profile_sensor_data)

        self._service_specs = [
            (InitializeLocalization, "/api/localization/initialize", self._on_initialize),
            (SetRoutePoints, "/api/routing/set_route_points", self._on_set_route_points),
            (SetRoute, "/api/routing/set_route", self._on_set_route),
            (ClearRoute, "/api/routing/clear_route", self._on_clear_route),
            (ChangeOperationMode, "/api/operation_mode/enable_autoware_control",
             self._on_enable_autoware_control),
            (ChangeOperationMode, "/api/operation_mode/change_to_stop",
             self._on_change_to_stop),
            (Engage, "/api/external/set/engage", self._on_engage),
            (SetVelocityLimit, "/api/autoware/set/velocity_limit", self._on_velocity_limit),
            (CooperateCommands, "/api/external/set/rtc_commands", self._on_rtc_commands),
            (AutoModeWithModule, "/api/external/set/rtc_auto_mode", self._on_rtc_auto_mode),
        ]
        self._services = []

        self.server = AgentServer(host, port, on_state=self._on_agent_state,
                                  on_presence=self._on_presence, peer_timeout=timeout,
                                  logger=_RosLogger(self.get_logger()))
        self.server.start()
        self.get_logger().info(
            f"Relaying entity {self.entity!r}: agents register on tcp://{host}:"
            f"{self.server.port}; ADAPI served in ROS domain "
            f"{os.environ.get('ROS_DOMAIN_ID', '0')} only while that agent is up")
        self._publish_all()
        self.create_timer(1.0 / rate, self._tick, callback_group=self._group)

    # -- agent side ----------------------------------------------------------------------
    def _on_agent_state(self, entity, state, _received_at):
        if entity != self.entity:
            return
        now = self.get_clock().now()
        with self._lock:
            self._state = state
            if state.get("pose") is not None:
                self._pose_stamp = now
            if state["phase"] == P.ARRIVED and self._phase != P.ARRIVED:
                self._arrived_at = now
            if state["phase"] != self._phase:
                self.get_logger().info(
                    f"{entity}: {self._phase or 'no agent'} -> {state['phase']}"
                    + (f" ({state['detail']})" if state.get("detail") else ""))
            self._phase = state["phase"]
        # Publish now, not on the next tick: the agent pushes the state a command caused
        # before its reply, and the concealer's next queued task reads these topics as
        # soon as the service returns. A tick later it read the state from before (a
        # STOPPED ego still PLANNING after clear_route, so initialize() refused it).
        self._publish_all()

    def _on_presence(self, entity, present):
        if entity != self.entity or present:
            return
        with self._lock:
            self._state = None
            self._phase = None
            self._pose_stamp = None
        self.get_logger().warning(f"{entity}: agent gone -- reporting Autoware not up")
        self._publish_all()

    def _agent_up(self):
        with self._lock:
            return self._state is not None and self._state["phase"] != P.UNAVAILABLE

    # -- publishing ---------------------------------------------------------------------
    def _tick(self):
        up = self._agent_up()
        if up and not self._services:
            self._services = [self.create_service(t, name, cb, callback_group=self._group)
                              for t, name, cb in self._service_specs]
            self.get_logger().info(f"{self.entity}: agent up -- ADAPI services offered")
        elif not up and self._services:
            for service in self._services:
                self.destroy_service(service)
            self._services = []
            self.get_logger().warning(f"{self.entity}: agent unavailable -- ADAPI "
                                      "services withdrawn")
        self._publish_all()

    def _publish_once(self, topic, key, publisher, message):
        """Latched topics: publish on change only, as Autoware does."""
        if getattr(self, "closing", False) or not rclpy.ok():
            return
        if self._latched.get(topic) != key:
            self._latched[topic] = key
            publisher.publish(message)

    def _publish_all(self):
        with self._publish_lock:
            self._publish_all_locked()

    def _publish_all_locked(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        with self._lock:
            state = self._state
            pose_stamp = self._pose_stamp
            arrived_at = self._arrived_at
        view = M.adapi_view(state)

        m = LocalizationInitializationState(stamp=stamp, state=view.localization)
        self._publish_once("localization", view.localization, self.pub_localization, m)

        route_stamp = stamp
        route_key = (view.route,)
        if view.route == M.ROUTE_ARRIVED and arrived_at is not None:
            route_stamp = arrived_at.to_msg()
            route_key = (view.route, arrived_at.nanoseconds)
        self._publish_once("route", route_key, self.pub_route,
                           RouteState(stamp=route_stamp, state=view.route))

        mode = OperationModeState(
            stamp=stamp, mode=view.mode,
            is_autoware_control_enabled=view.autoware_control_enabled,
            is_in_transition=False, is_stop_mode_available=view.stop_mode_available,
            is_autonomous_mode_available=view.autonomous_mode_available,
            is_local_mode_available=False, is_remote_mode_available=False)
        self._publish_once("mode", (view.mode, view.autoware_control_enabled,
                                    view.autonomous_mode_available,
                                    view.stop_mode_available), self.pub_mode, mode)

        self.pub_scan.publish(PointCloud2(header=_header(stamp, "base_link")))
        self.pub_mrm.publish(MrmState(stamp=stamp, state=view.mrm_state,
                                      behavior=view.mrm_behavior))
        self.pub_emergency.publish(Emergency(stamp=stamp, emergency=view.emergency))
        arrived_age = ((now - arrived_at).nanoseconds * 1e-9 if arrived_at is not None
                       else math.inf)
        self.pub_legacy.publish(AutowareState(stamp=stamp,
                                              state=M.legacy_state(view, arrived_age)))
        if state is None:
            return
        self.pub_turn.publish(TurnIndicatorsCommand(stamp=stamp,
                                                    command=view.turn_indicators))
        if state.get("rtc_status") is not None:
            self.pub_rtc.publish(_rtc_status(stamp, state["rtc_status"]))
        if state.get("pose") is not None and pose_stamp is not None:
            odom = Odometry(header=_header(pose_stamp.to_msg(), "map"),
                            child_frame_id="base_link")
            dict_to_pose(state["pose"], odom.pose.pose)
            self.pub_kinematic.publish(odom)

    # -- services -------------------------------------------------------------------------
    def _command(self, name, timeout, **args):
        status, message = self.server.command(self.entity, name, timeout, **args)
        text = f"{name} -> {status or 'no reply'}" + (f": {message}" if message else "")
        (self.get_logger().info if status == P.OK else self.get_logger().warning)(text)
        return status, message

    @staticmethod
    def _adapi(response, status, message):
        response.status.success = status == P.OK
        response.status.message = message if status in (P.OK, P.FAILED) else (
            f"{status or 'NO_REPLY'}: {message}")
        return response

    @staticmethod
    def _tier4(response, status, message):
        response.status.code = Tier4Status.SUCCESS if status == P.OK else Tier4Status.ERROR
        response.status.message = message or (status or "no reply")
        return response

    def _on_initialize(self, request, response):
        if not request.pose:
            return self._adapi(response, P.FAILED,
                               "the relay needs an explicit pose (teleported carries one)")
        pose = pose_to_dict(request.pose[0].pose.pose)
        return self._adapi(response, *self._command(P.TELEPORTED, SLOW_REPLY, pose=pose))

    def _goal(self, request, **extra):
        return self._command(
            P.SET_GOAL, SLOW_REPLY, goal=pose_to_dict(request.goal),
            allow_goal_modification=bool(request.option.allow_goal_modification), **extra)

    def _on_set_route_points(self, request, response):
        waypoints = [pose_to_dict(p) for p in request.waypoints]
        return self._adapi(response, *self._goal(request, waypoints=waypoints))

    def _on_set_route(self, request, response):
        segments = [{"id": int(s.preferred.id), "type": s.preferred.type,
                     "alternatives": [{"id": int(a.id), "type": a.type}
                                      for a in s.alternatives]}
                    for s in request.segments]
        return self._adapi(response, *self._goal(request, waypoints=[], segments=segments))

    def _on_clear_route(self, request, response):
        return self._adapi(response, *self._command(P.CLEAR_GOAL, QUICK_REPLY))

    def _on_enable_autoware_control(self, request, response):
        # Part of the goal: an agent drives its goal on its own (set_goal covers route,
        # control and engage), so there is nothing to forward.
        return self._adapi(response, P.OK, "")

    def _on_change_to_stop(self, request, response):
        return self._adapi(response, *self._command(P.STOP, QUICK_REPLY))

    def _on_engage(self, request, response):
        if request.engage:
            # The agent engages by itself once its goal is ready (set_goal).
            return self._tier4(response, P.OK, "the agent engages on its own")
        return self._tier4(response, *self._command(P.STOP, QUICK_REPLY))

    def _on_velocity_limit(self, request, response):
        mps = float(request.velocity)
        return self._tier4(response, *self._command(
            P.SET_SPEED_LIMIT, QUICK_REPLY, mps=mps if math.isfinite(mps) else None))

    def _on_rtc_commands(self, request, response):
        for item in request.commands:
            module = M.RTC_MODULE_NAMES.get(item.module.type, str(item.module.type))
            status, _ = self._command(
                P.COOPERATE, QUICK_REPLY, module=module,
                command="ACTIVATE" if item.command.type == 1 else "DEACTIVATE",
                uuid=bytes(item.uuid.uuid).hex())
            r = CooperateResponse(success=status == P.OK)
            r.module.type = item.module.type
            r.uuid.uuid = item.uuid.uuid
            response.responses.append(r)
        return response

    def _on_rtc_auto_mode(self, request, response):
        module = M.RTC_MODULE_NAMES.get(request.module.type, str(request.module.type))
        status, _ = self._command(P.COOPERATE_AUTO, QUICK_REPLY, module=module,
                                  enable=bool(request.enable))
        response.success = status == P.OK
        return response


def _header(stamp, frame_id):
    from std_msgs.msg import Header
    return Header(stamp=stamp, frame_id=frame_id)


def _rtc_status(stamp, statuses):
    array = CooperateStatusArray(stamp=stamp)
    for s in statuses:
        status = CooperateStatus(stamp=stamp, safe=bool(s.get("safe", False)),
                                 requested=bool(s.get("requested", False)),
                                 auto_mode=bool(s.get("auto_mode", False)),
                                 start_distance=float(s.get("start_distance", 0.0)),
                                 finish_distance=float(s.get("finish_distance", 0.0)))
        status.module.type = M.RTC_MODULES.get(s.get("module", "NONE"), 0)
        status.uuid.uuid = list(bytes.fromhex(s.get("uuid", "00" * 16)))
        status.command_status.type = 1 if s.get("command_status") == "ACTIVATE" else 0
        status.state.type = RTC_STATES.get(s.get("state", ""), 0)
        array.statuses.append(status)
    return array


class _RosLogger:
    """logging-style facade over an rclpy logger, for the ROS-free server module."""

    def __init__(self, logger):
        self._logger = logger

    def info(self, text):
        self._logger.info(text)

    def warning(self, text):
        self._logger.warning(text)


def main():
    rclpy.init()
    node = RelayNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C has already shut the context down; stopping the server closes agent
        # sessions, whose presence callbacks would publish into it and exit non-zero.
        node.closing = True
        node.server.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
