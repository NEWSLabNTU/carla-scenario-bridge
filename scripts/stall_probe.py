#!/usr/bin/env python3
"""When the ego stops mid-route, what is telling it to stop?

acb issue 016 ("the ego stack degrades after its first run") has the ego stall partway
along its route and never resume. Its leads are that the planned trajectory is truncated
at the stopping point, or that something is being perceived in the lane, or that a
behaviour-velocity module has inserted a stop. This samples all three on one timeline, so
a stall can be read as a mechanism rather than guessed at from a pose trace.

Per that issue's method note: this establishes MECHANISM on a single run, which is fair.
It cannot establish CAUSE across conditions -- that needs n >= 10 per arm with run order
held fixed.

    ROS_DOMAIN_ID=1 python3 scripts/stall_probe.py 240
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from autoware_adapi_v1_msgs.msg import VelocityFactorArray
from autoware_perception_msgs.msg import PredictedObjects


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    rclpy.init()
    n = rclpy.create_node("stall_probe")
    g = {"ego": None, "speed": 0.0, "traj": None, "factors": [], "objs": []}

    def on_odom(m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        g["ego"] = (p.x, p.y)
        g["speed"] = math.hypot(v.x, v.y)

    def on_traj(m):
        if not m.points:
            g["traj"] = (0, None, None, None)
            return
        first, last = m.points[0], m.points[-1]
        g["traj"] = (len(m.points),
                     (first.pose.position.x, first.pose.position.y),
                     (last.pose.position.x, last.pose.position.y),
                     max(p.longitudinal_velocity_mps for p in m.points))

    def on_factors(m):
        g["factors"] = [(f.behavior, round(f.distance, 1), f.status) for f in m.factors]

    def on_objs(m):
        out = []
        for o in m.objects:
            p = o.kinematics.initial_pose_with_covariance.pose.position
            out.append((p.x, p.y))
        g["objs"] = out

    n.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 1)
    n.create_subscription(Trajectory, "/planning/scenario_planning/trajectory", on_traj, 1)
    n.create_subscription(VelocityFactorArray, "/api/planning/velocity_factors", on_factors, 1)
    n.create_subscription(PredictedObjects,
                          "/perception/object_recognition/objects", on_objs, 1)

    start = time.time()
    end = start + seconds
    last_t = -99.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time() - start
        if now - last_t < 2.0:
            continue
        last_t = now
        ego = g["ego"]
        where = f"ego({ego[0]:6.1f},{ego[1]:6.1f})" if ego else "ego(     ?      )"
        tr = g["traj"]
        if tr is None:
            traj = "traj[none]"
        elif tr[0] == 0:
            traj = "traj[EMPTY]"
        else:
            gap = math.hypot(tr[1][0] - ego[0], tr[1][1] - ego[1]) if ego else float("nan")
            traj = (f"traj[n={tr[0]} start=({tr[1][0]:.0f},{tr[1][1]:.0f}) "
                    f"end=({tr[2][0]:.0f},{tr[2][1]:.0f}) vmax={tr[3]:.1f} "
                    f"start_gap={gap:.1f}m]")
        # Objects within 30 m ahead-ish, to catch a phantom obstacle in the lane.
        near = 0
        if ego:
            near = sum(1 for (x, y) in g["objs"]
                       if math.hypot(x - ego[0], y - ego[1]) < 30.0)
        fac = ",".join(f"{b}@{d}m/st{s}" for b, d, s in g["factors"]) or "none"
        print(f"t+{now:3.0f}s {where} {g['speed']:4.1f} m/s  {traj}  "
              f"objs<30m={near}  factors[{fac}]", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
