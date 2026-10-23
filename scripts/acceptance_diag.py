#!/usr/bin/env python3
"""Why an acceptance run stopped (docs/issues/002), from the bag `acceptance_bag.py` records.

    acceptance_diag.py drops <bag>   each time availability.autonomous fell: the graph's nodes
                                     and leaves whose level rose against the status 1.5 s before
    acceptance_diag.py ndt <bag>     NDT's iterations and score against the ego's x, per 2 m,
                                     while it drives (a lone 60 is a scan at the limit: a WARN,
                                     and in Autoware 1.5's graph an emergency stop)

Source Autoware first (the message types).
"""
import bisect
import collections
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def level(x):
    return x if isinstance(x, int) else ord(x)


def messages(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic in topics:
            yield topic, t_ns * 1e-9, deserialize_message(data, types[topic])


def drops(bag):
    struct, avail, events, hist = None, None, [], []
    for topic, t, m in messages(bag, {"/diagnostics_graph/struct", "/diagnostics_graph/status",
                                      "/system/operation_mode/availability"}):
        if topic == "/diagnostics_graph/struct":
            struct = m
        elif topic == "/system/operation_mode/availability":
            if avail is True and m.autonomous is False:
                events.append(t)
            avail = m.autonomous
        else:
            hist.append((t, [level(n.level) for n in m.nodes],
                         [(level(d.level), d.message) for d in m.diags]))
    if struct is None:
        sys.exit("no /diagnostics_graph/struct in the bag")
    for t0 in events:
        before = [h for h in hist if h[0] < t0 - 1.5]
        after = [h for h in hist if t0 - 1.5 <= h[0] <= t0 + 0.3]
        if not before:
            continue
        print(f"=== availability.autonomous false at {t0:.3f}")
        seen = set()
        for t, nodes, leaves in after:
            for i, (a, b) in enumerate(zip(before[-1][1], nodes)):
                if b > a and ("n", i, b) not in seen:
                    seen.add(("n", i, b))
                    print(f"  {t - t0:+.3f} node {struct.nodes[i].path or '(root)'}: {a} -> {b}")
            for i, (a, b) in enumerate(zip(before[-1][2], leaves)):
                if b[0] > a[0] and ("l", i, b[0]) not in seen:
                    seen.add(("l", i, b[0]))
                    print(f"  {t - t0:+.3f} leaf {struct.diags[i].name}: {a[0]} -> {b[0]} "
                          f"{b[1][:160]}")


def ndt(bag):
    poses, scans = [], []
    for topic, t, m in messages(bag, {"/localization/kinematic_state", "/diagnostics"}):
        if topic == "/localization/kinematic_state":
            poses.append((t, m.pose.pose.position.x, m.twist.twist.linear.x))
            continue
        for s in m.status:
            if "scan_matching_status" in s.name:
                kv = {k.key: k.value for k in s.values}
                if "iteration_num" in kv:
                    scans.append((t, int(float(kv["iteration_num"])),
                                  float(kv.get("nearest_voxel_transformation_likelihood", "nan"))))
    times = [p[0] for p in poses]
    bins = collections.defaultdict(list)
    for t, n, nvtl in scans:
        i = bisect.bisect_left(times, t)
        if 0 < i < len(poses) and abs(poses[i][0] - t) < 0.2 and poses[i][2] > 1.0:
            bins[int(poses[i][1] // 2) * 2].append((n, nvtl))
    for x in sorted(bins, reverse=True):
        ns = sorted(n for n, _ in bins[x])
        nv = sorted(v for _, v in bins[x])
        print(f"x {x:5d}: {len(ns):3d} scans, iterations p50 {ns[len(ns) // 2]:2d} max {ns[-1]:2d}, "
              f"NVTL p50 {nv[len(nv) // 2]:.2f}")


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("drops", "ndt"):
        sys.exit(__doc__)
    {"drops": drops, "ndt": ndt}[sys.argv[1]](sys.argv[2])
