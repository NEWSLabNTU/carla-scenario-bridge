#!/usr/bin/env python3
"""Does the classifier's sync callback fire, and does it ever return a colour?

Counts three things directly, with no latching and no inference:

  rois        TrafficLightRoiArray in, and how many carried a region
  debug       the classifier's debug image, which it publishes only when it runs
  signals     its output, and how many carried a non-empty signal list

If debug counts climb while signals stay empty, it ran and found nothing. If debug stays
at zero while rois arrive, the synchroniser never fired.
"""
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from tier4_perception_msgs.msg import TrafficLightArray, TrafficLightRoiArray

# tier4_perception_msgs/TrafficLightElement is a flat enum shared by colour, shape and
# status, and UNKNOWN is 18 -- not the 0..4 layout autoware_perception_msgs uses. Decode a
# classifier message with the wrong table and every ordinary unknown reads as corrupt.
COLOURS = {
    0: "UNKNOWN_0", 1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE",
    5: "CIRCLE", 6: "LEFT_ARROW", 7: "RIGHT_ARROW", 8: "UP_ARROW", 9: "UP_LEFT_ARROW",
    10: "UP_RIGHT_ARROW", 11: "DOWN_ARROW", 12: "DOWN_LEFT_ARROW", 13: "DOWN_RIGHT_ARROW",
    14: "CROSS", 15: "SOLID_OFF", 16: "SOLID_ON", 17: "FLASHING", 18: "UNKNOWN",
}
NS = "/perception/traffic_light_recognition/camera6"


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0
    rclpy.init()
    n = rclpy.create_node("classifier_probe")
    best_effort = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
    c = {"rois": 0, "rois_nonempty": 0, "debug": 0, "sig": 0, "sig_nonempty": 0,
         "colours": {}}

    def on_rois(m):
        c["rois"] += 1
        if m.rois:
            c["rois_nonempty"] += 1

    def on_debug(_m):
        c["debug"] += 1

    def on_sig(m):
        c["sig"] += 1
        if m.signals:
            c["sig_nonempty"] += 1
            for s in m.signals:
                for e in s.elements:
                    name = (f"{COLOURS.get(e.color, e.color)}"
                            f"/{COLOURS.get(e.shape, e.shape)}")
                    c["colours"][name] = c["colours"].get(name, 0) + 1

    n.create_subscription(TrafficLightRoiArray, f"{NS}/detection/rois", on_rois, 20)
    # The classifier publishes its output RELIABLE; the debug image goes out over
    # image_transport, which is best-effort.
    n.create_subscription(Image,
                          f"{NS}/classification/car_traffic_light_classifier/output/debug/image",
                          on_debug, best_effort)
    n.create_subscription(TrafficLightArray, f"{NS}/classification/car/traffic_signals",
                          on_sig, 20)

    start = time.time()
    end = start + seconds
    last = -99.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time() - start
        if now - last >= 10.0:
            last = now
            print(f"t+{now:3.0f}s  rois={c['rois']} (non-empty {c['rois_nonempty']})  "
                  f"debug_images={c['debug']}  signals_msgs={c['sig']} "
                  f"(non-empty {c['sig_nonempty']})", flush=True)

    print(f"\nrois={c['rois']} non-empty={c['rois_nonempty']}")
    print(f"classifier debug images (it ran this many times): {c['debug']}")
    print(f"classifier output messages: {c['sig']}, of which non-empty: {c['sig_nonempty']}")
    print(f"colours reported: {c['colours'] or 'none'}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
