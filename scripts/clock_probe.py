#!/usr/bin/env python3
"""Characterise /clock in one ROS domain.

Phase 013 left this open: with both modes on a rebuilt bridge, a MANAGED ego tracks
longitudinally at 0.051 m/s^2 and an UNMANAGED one at 0.235-0.385, five to seven times
worse. The obvious difference between the two paths is who publishes simulation time --
SSv2's interpreter in a managed run, `acb_bridge` in an unmanaged one (publish_clock=true,
because there is no SSv2 in that domain) -- and acb measured that bridge's main loop
turning at 13.9 Hz, which would make its clock coarser than SSv2's.

If that is the mechanism, the two clocks must differ measurably. This says whether they do.
It does NOT establish causation: a difference here makes the hypothesis worth pursuing, and
no difference kills it.

Reports, over the sample window:
  * publish rate in WALL time -- how often a clock message actually arrives
  * the distribution of SIM-time steps between consecutive messages, which is the
    granularity control and planning actually see
  * backwards or repeated steps, which are their own kind of broken

    ROS_DOMAIN_ID=3 clock_probe.py [seconds] [--label unmanaged]
"""

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seconds", nargs="?", type=float, default=120.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--settle", type=float, default=180.0,
                    help="how long to wait for the clock to start moving; simulation time "
                         "only advances once a scenario is running")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("clock_probe_%d" % int(time.time()))
    samples: list[tuple[float, float]] = []   # (wall, sim)

    def on_clock(msg: Clock):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        samples.append((time.monotonic(), sim))

    # /clock is best-effort in some stacks and reliable in others; ask for best-effort so a
    # mismatch cannot silently give us nothing.
    node.create_subscription(
        Clock, "/clock", on_clock,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))

    # Wait for the clock to be MOVING, not merely present: a stopped clock republishing the
    # same instant would otherwise be measured as a healthy stream of zero-sized steps.
    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(samples) >= 2 and samples[-1][1] > samples[0][1]:
            break
    if not (len(samples) >= 2 and samples[-1][1] > samples[0][1]):
        print(f"CLOCK {args.label} no moving clock within {args.settle:.0f}s "
              f"({len(samples)} message(s) seen)")
        return 1

    samples.clear()
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    if len(samples) < 10:
        print(f"CLOCK {args.label} only {len(samples)} samples; nothing to say")
        return 1

    wall_span = samples[-1][0] - samples[0][0]
    sim_span = samples[-1][1] - samples[0][1]
    steps = [b[1] - a[1] for a, b in zip(samples, samples[1:])]
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    forward = [s for s in steps if s > 0]
    backwards = sum(1 for s in steps if s < 0)
    repeats = sum(1 for s in steps if s == 0)

    print(f"CLOCK {args.label} samples={len(samples)} over {wall_span:.1f}s wall, "
          f"{sim_span:.1f}s sim (real-time factor {sim_span / wall_span:.2f})")
    print(f"CLOCK {args.label} publish rate {len(samples) / wall_span:.2f} Hz "
          f"(wall gap median {statistics.median(gaps) * 1e3:.1f} ms, "
          f"max {max(gaps) * 1e3:.1f} ms)")
    if forward:
        print(f"CLOCK {args.label} sim step median {statistics.median(forward) * 1e3:.2f} ms, "
              f"mean {statistics.mean(forward) * 1e3:.2f} ms, "
              f"max {max(forward) * 1e3:.2f} ms")
    print(f"CLOCK {args.label} backwards={backwards} repeated={repeats}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
