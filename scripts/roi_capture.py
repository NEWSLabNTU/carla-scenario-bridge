#!/usr/bin/env python3
"""Save what the traffic light classifier is actually handed.

The recognition chain runs end to end -- the signal is projected from the map, a region
of interest reaches the classifier every frame -- and the colour still comes back
UNKNOWN. That leaves one question that no topic rate or message count can answer: what do
those pixels look like?

This pairs each `detection/rois` message with the camera frame it refers to and writes
three things per sample:

    NNN_full.png   the frame with the region drawn on it, for context
    NNN_crop.png   exactly the pixels inside the region -- the classifier's input
    NNN_crop8.png  the same crop at 8x, nearest-neighbour, so it is inspectable

and prints the region's size in pixels, which is the number that decides whether this is
worth pursuing at all: a 0.451 m signal head at 100 m is a couple of pixels wide, and no
classifier recovers a colour from that.

Images are matched to regions by nearest timestamp rather than exact equality, since the
two topics are stamped by different nodes.

    ROS_DOMAIN_ID=1 python3 scripts/roi_capture.py 240 /tmp/roi_capture
"""
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from tier4_perception_msgs.msg import TrafficLightRoiArray

SAVE_EVERY = float(os.environ.get("ROI_SAVE_INTERVAL", "2.0"))


def to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray, for the encodings CARLA cameras emit."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgra8", "rgba8"):
        img = buf.reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if msg.encoding == "bgra8"
                            else cv2.COLOR_RGBA2BGR)
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img if msg.encoding == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unhandled encoding {msg.encoding!r}")


def stamp_secs(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    outdir = Path(sys.argv[2] if len(sys.argv) > 2 else "/tmp/roi_capture")
    outdir.mkdir(parents=True, exist_ok=True)
    camera_ns = os.environ.get("TL_CAMERA_NS", "camera6")

    rclpy.init()
    n = rclpy.create_node("roi_capture")

    sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)

    recent = []          # (stamp, bgr) ring of the last few frames
    state = {"saved": 0, "last_save": 0.0, "images": 0, "roi_msgs": 0, "with_roi": 0}

    def on_image(msg):
        state["images"] += 1
        try:
            recent.append((stamp_secs(msg.header), to_bgr(msg)))
        except ValueError as e:
            print(f"image: {e}", flush=True)
            return
        del recent[:-10]

    def on_rois(msg):
        state["roi_msgs"] += 1
        if not msg.rois:
            return
        state["with_roi"] += 1
        now = time.time()
        if now - state["last_save"] < SAVE_EVERY or not recent:
            return
        state["last_save"] = now

        want = stamp_secs(msg.header)
        stamp, frame = min(recent, key=lambda p: abs(p[0] - want))
        skew_ms = (stamp - want) * 1000.0

        idx = state["saved"]
        annotated = frame.copy()
        for roi in msg.rois:
            r = roi.roi
            x, y, w, h = int(r.x_offset), int(r.y_offset), int(r.width), int(r.height)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 0, 255), 2)
            crop = frame[max(y, 0):y + h, max(x, 0):x + w]
            print(f"[{idx:03d}] id={roi.traffic_light_id} roi=({x},{y}) {w}x{h} px  "
                  f"image_skew={skew_ms:+.0f}ms  crop={crop.shape if crop.size else 'EMPTY'}",
                  flush=True)
            if crop.size:
                cv2.imwrite(str(outdir / f"{idx:03d}_crop.png"), crop)
                cv2.imwrite(str(outdir / f"{idx:03d}_crop8.png"),
                            cv2.resize(crop, (w * 8, h * 8), interpolation=cv2.INTER_NEAREST))
        cv2.imwrite(str(outdir / f"{idx:03d}_full.png"), annotated)
        state["saved"] += 1

    n.create_subscription(Image, f"/sensing/camera/{camera_ns}/image_raw", on_image,
                          sensor_qos)
    n.create_subscription(TrafficLightRoiArray,
                          f"/perception/traffic_light_recognition/{camera_ns}/detection/rois",
                          on_rois, 1)

    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)

    print(f"\nimages={state['images']} roi_msgs={state['roi_msgs']} "
          f"non_empty={state['with_roi']} saved={state['saved']} -> {outdir}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
