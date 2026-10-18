#!/usr/bin/env python3
"""How well the ego tracked, run by run, from a rosbag of its domain (roadmap 038).

acb's `acceptance.py` scores a run from ROS alone -- longitudinal tracking (median
|commanded - delivered| acceleration while moving), cross-track error (median distance from
the EKF pose to the planned trajectory) and the EKF-to-GNSS localization gap -- but it also
launches the scenario and watches CARLA for the ego, and a fresh observer joining a domain
of ~200 Autoware participants takes tens of seconds to be discovered, so per-run observers
miss the start of a run. This scores a bag recorded across a whole batch instead, with
acceptance.py's own formulas and limits, so Isaac Sim and CARLA are measured the same way.
A run is a stretch of `/vehicle/status/velocity_status` with no gap over `--gap` seconds.

Times are the messages' own stamps (sim time), not when they arrived: acceptance.py's wall
clock reads a world running at 0.8x real time as delivering 0.8x the acceleration, which on a
loaded host turned a 0.017 m/s^2 tracking error into 0.07. At real time the two agree.

    ros2 bag record -o batch $(scripts/tracking_probe.py --topics)   # then run the scenarios
    scripts/tracking_probe.py batch [--json out.json]
"""
import argparse
import json
import math
import sys
from pathlib import Path

ACB_SCRIPTS = Path(__file__).resolve().parents[1] / "src/autoware_carla_bridge/scripts"
TOPICS = {
    "/vehicle/status/velocity_status": "autoware_vehicle_msgs/msg/VelocityReport",
    "/control/command/control_cmd": "autoware_control_msgs/msg/Control",
    "/localization/kinematic_state": "nav_msgs/msg/Odometry",
    "/planning/scenario_planning/trajectory": "autoware_planning_msgs/msg/Trajectory",
    "/sensing/gnss/pose_with_covariance": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/api/fail_safe/mrm_state": "autoware_adapi_v1_msgs/msg/MrmState",
}
MRM_NORMAL = 1  # autoware_adapi_v1_msgs/MrmState.NORMAL
SEATED_M = 5.0  # as acceptance.py: an EKF pose this far from GNSS is not this run's yet


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in TOPICS}
    arrivals = []  # (arrival, stamp) of the vehicle status: runs are split on arrival gaps
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in out:
            m = deserialize_message(data, types[topic])
            stamp = m.header.stamp if hasattr(m, "header") else m.stamp
            t = stamp.sec + stamp.nanosec * 1e-9
            out[topic].append((t, m))
            if topic == "/vehicle/status/velocity_status":
                arrivals.append((t_ns * 1e-9, t))
    for msgs in out.values():
        msgs.sort(key=lambda tm: tm[0])
    return out, arrivals


def split_runs(arrivals, gap):
    """Runs as (first, last) stamps: a new run where the status stopped arriving for `gap` s
    (between scenarios the world, and so its clock, stands still)."""
    runs = []
    last = None
    for arrived, stamp in arrivals:
        if last is None or arrived - last > gap:
            runs.append([stamp, stamp])
        else:
            runs[-1][1] = max(runs[-1][1], stamp)
        last = arrived
    return [(a, b) for a, b in runs if b - a > 5.0]


def score(msgs, start, end, acceptance):
    def within(topic):
        return [(t, m) for t, m in msgs[topic] if start <= t <= end]

    speeds = [(t, m.longitudinal_velocity) for t, m in within("/vehicle/status/velocity_status")]
    commands = [(t, m.longitudinal.acceleration) for t, m in within("/control/command/control_cmd")]
    lon = acceptance.longitudinal_error(commands, speeds)
    gnss = [(t, (m.pose.pose.position.x, m.pose.pose.position.y))
            for t, m in within("/sensing/gnss/pose_with_covariance")]
    trajs = [(t, [(p.pose.position.x, p.pose.position.y) for p in m.points])
             for t, m in within("/planning/scenario_planning/trajectory")]
    gap, cross, path = [], [], []
    gi = ti = 0
    for t, m in within("/localization/kinematic_state"):
        while gi + 1 < len(gnss) and gnss[gi + 1][0] <= t:
            gi += 1
        while ti + 1 < len(trajs) and trajs[ti + 1][0] <= t:
            ti += 1
        if not gnss or gnss[gi][0] > t:
            continue
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        d = math.dist(p, gnss[gi][1])
        gap.append(d)
        if d > SEATED_M:
            continue
        path.append(p)
        if trajs and trajs[ti][0] <= t and len(trajs[ti][1]) >= 2:
            cross.append(min(math.dist(p, q) for q in trajs[ti][1]))
    # Entries into an MRM (state leaving NORMAL) during the run: 043 asks for none.
    mrm, prev = 0, None
    for t, m in msgs["/api/fail_safe/mrm_state"]:  # published on change: carry the state in
        if start <= t <= end and prev == MRM_NORMAL and m.state != MRM_NORMAL:
            mrm += 1
        prev = m.state
    med = acceptance.median
    return {
        "mrm_entries": mrm,
        "duration_s": round(end - start, 1),
        "longitudinal_median_error": lon["median_error"],
        "longitudinal_samples": lon["samples"],
        "cross_track_median": med(cross) if cross else None,
        "localization_gap_median": med(gap) if gap else None,
        "peak_speed": max((abs(v) for _, v in speeds), default=0.0),
        # The farthest it got from where it was first placed: a run's last pose can be the
        # next spawn's (the estimate outlives the vehicle, acb issue 022).
        "distance": max((math.dist(path[0], p) for p in path), default=0.0),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", nargs="?")
    ap.add_argument("--topics", action="store_true", help="print the topics to record")
    ap.add_argument("--gap", type=float, default=3.0, help="s without vehicle status: a new run")
    ap.add_argument("--json", help="write the result here")
    args = ap.parse_args()
    if args.topics:
        print(" ".join(TOPICS))
        return 0
    if not args.bag:
        ap.error("a bag directory, or --topics")
    sys.path.insert(0, str(ACB_SCRIPTS))
    import acceptance  # noqa: E402  (acb's harness: its formulas and limits)

    msgs, arrivals = read_bag(args.bag)
    runs = [score(msgs, a, b, acceptance) for a, b in split_runs(arrivals, args.gap)]
    out = {"runs": runs,
           "limits": {"longitudinal": acceptance.MAX_LONGITUDINAL_ERROR,
                      "cross_track": acceptance.MAX_CROSS_TRACK,
                      "localization_gap": acceptance.MAX_LOCALIZATION_GAP}}
    print(json.dumps(out, indent=2))
    if args.json:
        Path(args.json).write_text(json.dumps(out, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
