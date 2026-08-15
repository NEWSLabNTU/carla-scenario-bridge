#!/usr/bin/env python3
"""CARLA's ground truth beside Autoware's estimate, sampled together.

Answers three questions about a two-AV run in one line per sample: is the ego localized
(`pose_err`, estimate against CARLA truth), is it actually following the other vehicle
(`gap_to_bg`, `speed`), and is its perception sane (`objects` -- a healthy run on Town01
sits at 2-7; a broken one climbs past 40 as buildings stop being subtracted from the
pointcloud map).

Needs both the ego's ROS domain *and* the CARLA PythonAPI. The packaged server ships
wheels but does not install them, so unpack the one matching your Python and put it on
PYTHONPATH:

    unzip -o "$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl" -d /tmp/carlapy
    PYTHONPATH=/tmp/carlapy ROS_DOMAIN_ID=1 python3 scripts/lead_vehicle_probe.py 300
"""
import math
import os
import sys
import time

try:
    import carla
except ImportError:
    sys.exit(__doc__.split("Needs both", 1)[1].strip())

import rclpy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    ego_role = os.environ.get("EGO_ROLE_NAME", "hero")
    bg_role = os.environ.get("BG_ROLE_NAME", "bg_av_1")

    client = carla.Client(os.environ.get("CARLA_HOST", "localhost"),
                          int(os.environ.get("CARLA_PORT", "2000")))
    client.set_timeout(20.0)
    world = client.get_world()
    # A client's actor list stays empty until it has seen a tick, and the world is in
    # synchronous mode, so asking straight after connecting reports zero actors and looks
    # exactly like a world that has been torn down.
    world.wait_for_tick()

    rclpy.init()
    n = rclpy.create_node("lead_vehicle_probe")
    g = {"ego": None, "speed": 0.0, "objs": 0}

    def on_odom(msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        g["ego"] = (p.x, p.y)
        g["speed"] = math.hypot(v.x, v.y)

    n.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 1)
    n.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                          lambda m: g.__setitem__("objs", len(m.objects)), 1)

    start = time.time()
    end = start + seconds
    last = -99.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time() - start
        if now - last < 2.0:
            continue
        last = now
        try:
            truth = {}
            for a in world.get_actors().filter("vehicle.*"):
                t = a.get_transform()
                # CARLA is left-handed; the ROS frame flips Y.
                truth[a.attributes.get("role_name", "?")] = (t.location.x, -t.location.y)
        except Exception as e:
            print(f"t+{now:3.0f}s CARLA read failed: {e}", flush=True)
            continue

        hero, bg, est = truth.get(ego_role), truth.get(bg_role), g["ego"]
        err = math.hypot(est[0] - hero[0], est[1] - hero[1]) if (hero and est) else float("nan")
        gap = math.hypot(hero[0] - bg[0], hero[1] - bg[1]) if (hero and bg) else float("nan")
        hx = f"{hero[0]:7.2f}" if hero else "      ?"
        ex = f"{est[0]:7.2f}" if est else "      ?"
        print(f"t+{now:3.0f}s truth_x={hx} est_x={ex} pose_err={err:5.2f} "
              f"gap_to_bg={gap:6.1f} speed={g['speed']:4.1f} objects={g['objs']}",
              flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
