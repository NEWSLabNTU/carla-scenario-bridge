#!/usr/bin/env python3
"""Did the ego's Autoware see the other vehicle, and did it react?

Watches the ego's perception output and its own speed, and reports the closest tracked
object ahead. Run this in the ego's domain while a two-AV scenario is going.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
    rclpy.init()
    n = rclpy.create_node("perception_probe")

    state = {"objects": 0, "frames": 0, "ego": None, "speed": 0.0,
             "closest": None, "max_objects": 0}

    def on_objects(msg):
        state["frames"] += 1
        state["objects"] = len(msg.objects)
        state["max_objects"] = max(state["max_objects"], len(msg.objects))
        ego = state["ego"]
        if ego is None:
            return
        best = None
        for obj in msg.objects:
            p = obj.kinematics.initial_pose_with_covariance.pose.position
            d = math.hypot(p.x - ego[0], p.y - ego[1])
            if best is None or d < best[0]:
                best = (d, p.x, p.y)
        state["closest"] = best

    def on_odom(msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        state["ego"] = (p.x, p.y)
        state["speed"] = math.sqrt(v.x**2 + v.y**2)

    n.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                          on_objects, 1)
    n.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 1)

    end = time.time() + seconds
    last = 0.0
    start = time.time()
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time() - start
        if now - last >= 10.0:
            last = now
            ego = state["ego"]
            where = f"ego({ego[0]:.1f}, {ego[1]:.1f})" if ego else "ego(?)"
            closest = state["closest"]
            near = (f"closest object {closest[0]:.1f} m at ({closest[1]:.1f}, {closest[2]:.1f})"
                    if closest else "no objects")
            print(f"t+{now:3.0f}s  {where}  {state['speed']:.1f} m/s  "
                  f"tracked={state['objects']}  {near}", flush=True)
    print(f"perception frames={state['frames']}  most objects seen at once="
          f"{state['max_objects']}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
