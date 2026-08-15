#!/usr/bin/env python3
"""Does the ego's Autoware perceive the pedestrians and props SSv2 spawned?

Phase 008 asks whether a pedestrian and a misc object are detected *through the ego's
sensors*, which needs both halves of the picture at once: where CARLA actually put each
entity, and what the ego's perception is reporting. This reads CARLA's walkers and props
directly and matches each one against the nearest tracked object.

Per entity it reports the closest match seen over the whole run, the ego's range at that
moment, and the classification label Autoware assigned. A match within a couple of metres
of a 0.6 m-wide walker is a detection; "no match" with the ego having driven past at
20 m is a real negative, not a missed sample.

Needs the ego's ROS domain and the CARLA PythonAPI:

    unzip -o "$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl" -d /tmp/carlapy
    PYTHONPATH=/tmp/carlapy ROS_DOMAIN_ID=1 python3 scripts/entity_perception_probe.py 300
"""
import math
import os
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("carla PythonAPI not importable -- see this script's docstring")

import rclpy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import PredictedObjects

# autoware_perception_msgs/ObjectClassification
LABELS = {0: "UNKNOWN", 1: "CAR", 2: "TRUCK", 3: "BUS", 4: "TRAILER",
          5: "MOTORCYCLE", 6: "BICYCLE", 7: "PEDESTRIAN"}

# How close a tracked object has to be to count as that entity. A walker is 0.6 m wide
# and a barrier 1.2 m, and perception reports a cluster centroid, so a couple of metres
# is generous without being able to confuse the two -- they are 40 m apart.
MATCH_RADIUS = 4.0

# Town01 ships ~150 `static.prop.mesh` actors -- the map's own scenery, in the pointcloud
# map and subtracted from perception by design. Reporting them buries the two or three
# props the scenario actually spawned in a wall of noise.
IGNORED_TYPES = ("static.prop.mesh",)

# Beyond this the ego cannot have seen it, so a "match" is an artefact. The one that
# matters: for the first few seconds after Initialize, perception is still publishing the
# *previous* run's object list, and an object left over at these coordinates will sit on
# top of a new entity 200 m away. Scoring that as a detection is how a probe reports a
# hit for a prop the ego never came within 20 m of.
MAX_DETECTION_RANGE = 80.0


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    ego_role = os.environ.get("EGO_ROLE_NAME", "hero")

    client = carla.Client(os.environ.get("CARLA_HOST", "localhost"),
                          int(os.environ.get("CARLA_PORT", "2000")))
    client.set_timeout(20.0)
    world = client.get_world()
    # A client's actor list is empty until it has seen a tick, and the world is
    # synchronous -- without this every entity looks absent.
    world.wait_for_tick()

    rclpy.init()
    n = rclpy.create_node("entity_perception_probe")
    g = {"ego": None, "speed": 0.0, "objs": []}

    def on_objects(msg):
        out = []
        for o in msg.objects:
            p = o.kinematics.initial_pose_with_covariance.pose.position
            cls = o.classification[0].label if o.classification else -1
            out.append((p.x, p.y, cls))
        g["objs"] = out

    def on_odom(msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        g["ego"] = (p.x, p.y)
        g["speed"] = math.hypot(v.x, v.y)

    n.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                          on_objects, 1)
    n.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 1)

    best = {}   # entity key -> (match distance, ego range, label, t, entity xy)
    seen = {}   # entity key -> last known truth position

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
            truth, hero = {}, None
            for a in world.get_actors():
                t = a.get_transform()
                # CARLA is left-handed; the ROS frame flips Y.
                xy = (t.location.x, -t.location.y)
                if a.type_id in IGNORED_TYPES:
                    continue
                if a.type_id.startswith("walker.") or a.type_id.startswith("static.prop."):
                    truth[f"{a.type_id}#{a.id}"] = xy
                elif a.attributes.get("role_name") == ego_role:
                    hero = xy
        except Exception as e:
            print(f"t+{now:3.0f}s CARLA read failed: {e}", flush=True)
            continue

        seen.update(truth)
        report = []
        for key, xy in sorted(truth.items()):
            match = None
            for (ox, oy, cls) in g["objs"]:
                d = math.hypot(ox - xy[0], oy - xy[1])
                if match is None or d < match[0]:
                    match = (d, cls)
            rng = math.hypot(hero[0] - xy[0], hero[1] - xy[1]) if hero else float("nan")
            short = key.split("#")[0].replace("static.prop.", "").replace("walker.pedestrian.", "ped")
            at = f"{short}@({xy[0]:.0f},{xy[1]:.1f}) ego{rng:4.0f}m"
            if match and match[0] <= MATCH_RADIUS and rng <= MAX_DETECTION_RANGE:
                label = LABELS.get(match[1], f"?{match[1]}")
                report.append(f"{at} HIT d={match[0]:.1f} {label}")
                if key not in best or match[0] < best[key][0]:
                    best[key] = (match[0], rng, label, now, xy)
            else:
                d = f"{match[0]:.0f}" if match else "-"
                report.append(f"{at} miss(nearest {d})")

        where = f"ego({hero[0]:6.1f})" if hero else "ego(     ?)"
        print(f"t+{now:3.0f}s {where} {g['speed']:4.1f} m/s  n={len(g['objs'])}  "
              + "  ".join(report), flush=True)

    print()
    print(f"entities seen in CARLA: {len(seen)}")
    for key, xy in sorted(seen.items()):
        if key in best:
            d, rng, label, t, _ = best[key]
            print(f"  {key:34s} at ({xy[0]:.1f},{xy[1]:.1f})  DETECTED: "
                  f"closest match {d:.1f} m, classified {label}, ego {rng:.0f} m away, t+{t:.0f}s")
        else:
            print(f"  {key:34s} at ({xy[0]:.1f},{xy[1]:.1f})  NOT DETECTED")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
