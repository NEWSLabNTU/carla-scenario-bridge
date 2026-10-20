"""ROS 2 for agents: an rclpy node spun on its own thread, and message helpers.

    class MyAgent(csb_agent.ros.RosNode, csb_agent.Agent):
        def __init__(self, config=None):
            super().__init__(config)
            self.start_node("my_agent")
            self.pub = self.node.create_publisher(...)

Only this module and ros_adapter.py import rclpy; the rest of csb_agent needs nothing but
the standard library.
"""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service

POSE_TYPES = ("geometry_msgs/msg/Pose", "geometry_msgs/msg/PoseStamped",
              "geometry_msgs/msg/PoseWithCovariance",
              "geometry_msgs/msg/PoseWithCovarianceStamped", "nav_msgs/msg/Odometry")


class RosNode:
    """Mixin: `self.node`, spun by a multi-threaded executor on a daemon thread."""

    node = None

    def start_node(self, name, namespace=""):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name, namespace=namespace)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        return self.node

    def stop_node(self):
        if self.node is not None:
            self._executor.shutdown()
            self.node.destroy_node()
            self.node = None

    def close(self):
        self.stop_node()
        super().close()


def message_type(name):
    return get_message(name)


def service_type(name):
    return get_service(name)


def _type_name(msg):
    module = type(msg).__module__.split(".")  # e.g. geometry_msgs.msg._pose_stamped
    return f"{module[0]}/msg/{type(msg).__name__}"


def set_pose(msg, pose, frame_id="map"):
    """Write a protocol pose ({x, y, z, qx, qy, qz, qw}) into any pose-carrying message."""
    kind = _type_name(msg)
    if kind == "geometry_msgs/msg/PoseStamped":
        msg.header.frame_id = frame_id
        target = msg.pose
    elif kind == "geometry_msgs/msg/PoseWithCovarianceStamped":
        msg.header.frame_id = frame_id
        target = msg.pose.pose
    elif kind == "geometry_msgs/msg/PoseWithCovariance":
        target = msg.pose
    elif kind == "nav_msgs/msg/Odometry":
        msg.header.frame_id = frame_id
        target = msg.pose.pose
    elif kind == "geometry_msgs/msg/Pose":
        target = msg
    else:
        raise TypeError(f"{kind} carries no pose (one of {', '.join(POSE_TYPES)})")
    target.position.x, target.position.y, target.position.z = (
        float(pose["x"]), float(pose["y"]), float(pose["z"]))
    target.orientation.x, target.orientation.y = float(pose["qx"]), float(pose["qy"])
    target.orientation.z, target.orientation.w = float(pose["qz"]), float(pose["qw"])
    return msg


def get_pose(msg):
    """A protocol pose from any pose-carrying message."""
    kind = _type_name(msg)
    p = {"geometry_msgs/msg/PoseStamped": lambda m: m.pose,
         "geometry_msgs/msg/PoseWithCovarianceStamped": lambda m: m.pose.pose,
         "geometry_msgs/msg/PoseWithCovariance": lambda m: m.pose,
         "nav_msgs/msg/Odometry": lambda m: m.pose.pose,
         "geometry_msgs/msg/Pose": lambda m: m}[kind](msg)
    return {"x": p.position.x, "y": p.position.y, "z": p.position.z,
            "qx": p.orientation.x, "qy": p.orientation.y, "qz": p.orientation.z,
            "qw": p.orientation.w}


def fill(msg, fields=None):
    """Static field values from YAML, as `ros2 topic pub` takes them."""
    if fields:
        set_message_fields(msg, fields)
    return msg


def field_path(msg, path):
    """(parent, attribute) for a dotted field path; the message itself for ''."""
    if not path:
        return None, msg
    *parents, last = path.split(".")
    obj = msg
    for part in parents:
        obj = getattr(obj, part)
    return obj, last


__all__ = ["RosNode", "message_type", "service_type", "set_pose", "get_pose", "fill",
           "field_path", "POSE_TYPES"]
