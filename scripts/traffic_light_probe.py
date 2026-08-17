#!/usr/bin/env python3
"""Does Autoware see the signal state SSv2 commanded, and does the ego stop for it?

Phase 009 gets the command as far as CARLA -- a commanded red is red on the CARLA actor,
verified. What this answers is the rest of the chain: whether the ego's camera and
traffic-light recognition read that red back, and whether planning acts on it.

Three things per sample: what recognition is reporting per signal group, where the ego is
and how fast, and (when the CARLA PythonAPI is importable) what CARLA's own traffic light
actors say, which is the ground truth SSv2 wrote.

    PYTHONPATH=/tmp/carlapy ROS_DOMAIN_ID=1 python3 scripts/traffic_light_probe.py 300
"""
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrafficLightGroupArray
from tier4_perception_msgs.msg import TrafficLightArray, TrafficLightRoiArray

try:
    import carla
except ImportError:
    carla = None

# autoware_perception_msgs/TrafficLightElement colour, used by the fused topics.
COLOURS = {0: "UNKNOWN", 1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE"}

# tier4_perception_msgs/TrafficLightElement, used by the per-camera classifier, is a
# DIFFERENT and flat enum: colours, shapes and statuses share one numbering and UNKNOWN
# is 18, not 0. Decoding a classifier message with the table above prints "?18" for what
# is simply an unknown light, which is exactly how an ordinary result gets mistaken for
# a corrupt one.
T4_ELEMENT = {
    0: "UNKNOWN_0", 1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE",
    5: "CIRCLE", 6: "LEFT_ARROW", 7: "RIGHT_ARROW", 8: "UP_ARROW", 9: "UP_LEFT_ARROW",
    10: "UP_RIGHT_ARROW", 11: "DOWN_ARROW", 12: "DOWN_LEFT_ARROW", 13: "DOWN_RIGHT_ARROW",
    14: "CROSS", 15: "SOLID_OFF", 16: "SOLID_ON", 17: "FLASHING", 18: "UNKNOWN",
}
# CARLA's own enum, for the ground-truth column
CARLA_STATES = {"Red": "RED", "Yellow": "AMBER", "Green": "GREEN", "Off": "OFF"}


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0

    world = None
    if carla is not None:
        try:
            client = carla.Client(os.environ.get("CARLA_HOST", "localhost"),
                                  int(os.environ.get("CARLA_PORT", "2000")))
            client.set_timeout(20.0)
            world = client.get_world()
            world.wait_for_tick()
        except Exception as e:
            print(f"CARLA ground truth unavailable: {e}", flush=True)
            world = None

    rclpy.init()
    n = rclpy.create_node("traffic_light_probe")
    g = {"ego": None, "speed": 0.0, "groups": {}, "frames": 0}

    def on_signals(msg):
        g["frames"] += 1
        seen = {}
        for grp in msg.traffic_light_groups:
            colours = [COLOURS.get(e.color, f"?{e.color}") for e in grp.elements]
            seen[grp.traffic_light_group_id] = colours or ["(no elements)"]
        g["groups"] = seen

    def on_odom(msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        g["ego"] = (p.x, p.y)
        g["speed"] = math.hypot(v.x, v.y)

    n.create_subscription(TrafficLightGroupArray,
                          "/perception/traffic_light_recognition/traffic_signals",
                          on_signals, 1)
    n.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 1)

    # The two stages before classification. `expect` is what the map-based detector
    # projects into the image from the map alone; `rois` is what the fine detector
    # actually found there. Which of the two is empty says whether the problem is the
    # map's geometry or the image.
    camera_ns = os.environ.get("TL_CAMERA_NS", "camera6")
    g["expect"], g["rois"] = 0, 0
    for key, topic in (("expect", f"/perception/traffic_light_recognition/{camera_ns}"
                                  "/detection/expect/rois"),
                       ("rois", f"/perception/traffic_light_recognition/{camera_ns}"
                                "/detection/rois")):
        n.create_subscription(
            TrafficLightRoiArray, topic,
            (lambda k: lambda m: g.__setitem__(k, len(m.rois)))(key), 1)

    # And the two hops after classification, so a colour that is read but then dropped
    # looks different from one that was never read.
    g["classified"], g["judged"] = "-", "-"

    def summarise(msg):
        # The car classifier publishes TrafficLightArray (`signals`); the fused topics
        # publish TrafficLightGroupArray (`traffic_light_groups`). Same question of both.
        groups = getattr(msg, "traffic_light_groups", None)
        table = COLOURS
        if groups is None:                      # tier4 TrafficLightArray, flat enum
            groups = getattr(msg, "signals", [])
            table = T4_ELEMENT
        out = []
        for grp in groups:
            gid = getattr(grp, "traffic_light_group_id", getattr(grp, "traffic_light_id", "?"))
            out.append(f"{gid}:" + "/".join(
                table.get(e.color, f"?{e.color}") for e in grp.elements))
        return ",".join(out) if out else "empty"

    # The *car* classifier's own output. The merged `classification/traffic_signals` is a
    # later stage; watching that one cannot tell "the classifier never ran" from "the
    # classifier ran and something downstream dropped it".
    n.create_subscription(
        TrafficLightArray,
        f"/perception/traffic_light_recognition/{camera_ns}/classification/car/traffic_signals",
        lambda m: g.__setitem__("classified", summarise(m)), 1)
    n.create_subscription(
        TrafficLightGroupArray,
        "/perception/traffic_light_recognition/judged/traffic_signals",
        lambda m: g.__setitem__("judged", summarise(m)), 1)

    # What recognition reported, and the ego's slowest speed while it reported it.
    history = {}
    start = time.time()
    end = start + seconds
    last = -99.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time() - start
        if now - last < 2.0:
            continue
        last = now

        truth = ""
        if world is not None:
            try:
                states = {}
                for a in world.get_actors().filter("traffic.traffic_light*"):
                    s = CARLA_STATES.get(str(a.state), str(a.state))
                    states[s] = states.get(s, 0) + 1
                truth = "  carla=" + ",".join(f"{k}x{v}" for k, v in sorted(states.items()))
            except Exception as e:
                truth = f"  carla=<{e}>"

        ego = g["ego"]
        where = f"ego({ego[0]:6.1f},{ego[1]:6.1f})" if ego else "ego(     ?      )"
        rec = "  ".join(f"{gid}:{'/'.join(c)}" for gid, c in sorted(g["groups"].items())) or "none"
        print(f"t+{now:3.0f}s {where} {g['speed']:4.1f} m/s  "
              f"expect={g['expect']} rois={g['rois']} class[{g['classified']}] "
              f"judged[{g['judged']}]  recognised[{rec}]{truth}", flush=True)

        for gid, cols in g["groups"].items():
            key = (gid, "/".join(cols))
            prev = history.get(key)
            if prev is None or g["speed"] < prev[0]:
                history[key] = (g["speed"], now, ego)

    print()
    print(f"traffic_signals frames={g['frames']}")
    if not history:
        print("recognition never reported a signal group")
    for (gid, cols), (spd, t, ego) in sorted(history.items()):
        at = f"({ego[0]:.1f},{ego[1]:.1f})" if ego else "(?)"
        print(f"  group {gid} reported {cols:12s} -- ego slowest {spd:.2f} m/s at {at}, t+{t:.0f}s")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
