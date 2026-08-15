#!/usr/bin/env python3
"""Did the ego's Autoware see the other vehicle, and did it react?

Run this in the ego's domain while a two-AV scenario is going.

It reports every tracked object, not the nearest one. The nearest object is always the
ego's own body -- a ghost that trails about a metre off its bumper for the whole run --
so "closest object 1.1 m" says nothing about the vehicle ahead. What answers the question
is the closest object that is *ahead* of the ego, *in its lane*, and far enough away not
to be the self-ghost. That is what BEST LANE HIT reports.

`LANE_Y` is the ego lane's y in the ROS frame; pass it as the second argument for a
scenario on a different street.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects

SELF_GHOST_RADIUS = 5.0  # m: anything closer than this is the ego's own body
LANE_HALF_WIDTH = 4.0    # m: how far off the lane centre still counts as "in lane"


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
    lane_y = float(sys.argv[2]) if len(sys.argv) > 2 else -129.8

    rclpy.init()
    n = rclpy.create_node("perception_probe")
    state = {"ego": None, "speed": 0.0, "objs": [], "frames": 0,
             "best_hit": None, "max_objs": 0, "t": 0.0}

    def on_objects(msg):
        state["frames"] += 1
        state["max_objs"] = max(state["max_objs"], len(msg.objects))
        out = []
        for o in msg.objects:
            p = o.kinematics.initial_pose_with_covariance.pose.position
            v = o.kinematics.initial_twist_with_covariance.twist.linear
            cls = o.classification[0].label if o.classification else -1
            out.append((p.x, p.y, math.hypot(v.x, v.y), cls))
        state["objs"] = out

        ego = state["ego"]
        if ego is None:
            return
        for (x, y, sp, _cls) in out:
            d = math.hypot(x - ego[0], y - ego[1])
            # The ego drives along -x on this map, so "ahead" is a smaller x.
            if d > SELF_GHOST_RADIUS and x < ego[0] and abs(y - lane_y) < LANE_HALF_WIDTH:
                if state["best_hit"] is None or d < state["best_hit"][0]:
                    state["best_hit"] = (d, x, y, sp, state["t"])

    def on_odom(msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        state["ego"] = (p.x, p.y)
        state["speed"] = math.hypot(v.x, v.y)

    n.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                          on_objects, 1)
    n.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 1)

    start = time.time()
    end = start + seconds
    last = -99.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time() - start
        state["t"] = now
        if now - last >= 5.0:
            last = now
            ego = state["ego"]
            where = f"ego({ego[0]:7.1f},{ego[1]:7.1f})" if ego else "ego(      ?      )"
            objs = " | ".join(f"({x:.1f},{y:.1f}) v={sp:.1f} c={cls}"
                              for (x, y, sp, cls) in state["objs"]) or "none"
            print(f"t+{now:3.0f}s {where} {state['speed']:4.1f} m/s  "
                  f"n={len(state['objs'])}  {objs}", flush=True)

    hit = state["best_hit"]
    print(f"frames={state['frames']} max_objects={state['max_objs']}")
    if hit:
        print(f"BEST LANE HIT: {hit[0]:.1f} m ahead at ({hit[1]:.1f},{hit[2]:.1f}) "
              f"speed {hit[3]:.1f} m/s, first seen t+{hit[4]:.0f}s")
    else:
        print("BEST LANE HIT: none -- no tracked object ahead of the ego in its lane")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
