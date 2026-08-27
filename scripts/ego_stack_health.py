#!/usr/bin/env python3
"""Decide whether the ego's Autoware is ready for SSv2 to drive, and say why if not.

Exit 0 if ready, 1 if not. One line of reasoning either way, in the shape of
`acb/scripts/carla_health.py`.

This exists because of how the failure looks without it. With `launch_autoware:=false` the
concealer does not fork Autoware; it attaches to one already running. Its constructor
queues a `ChangeToStop` against an ADAPI service with a **180 second** timeout, so starting
a scenario before the ego stack is up does not fail fast -- the run sits there, evaluates
nothing, and dies three minutes later with an `AutowareError` that names a service rather
than the ordering mistake. Phase 012 asked for that ordering to be enforced rather than
merely documented.

What counts as ready is the thing the concealer actually needs: the ADAPI operation-mode
topic being published. That is later than "the process exists" and earlier than "the ego is
engaged", which is the window the concealer expects to attach in.

    ROS_DOMAIN_ID=1 scripts/ego_stack_health.py [--timeout 10]
"""

import argparse
import sys

# What the concealer talks to. Operation mode is the one it drives through
# (stop -> autonomous) and is published by the ADAPI adaptors, so its presence means the
# API layer is up rather than merely the launch having been started.
REQUIRED_TOPIC = "/api/operation_mode/state"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait for the topic to appear with a live publisher",
    )
    args = ap.parse_args()

    try:
        import rclpy
    except ImportError as e:
        print(f"[ego-health] cannot import rclpy: {e}")
        return 1

    import time

    # Node creation can fail outright -- an unusable ROS_DOMAIN_ID puts Cyclone's ports out
    # of range, for one -- and a check that answers with a traceback is not a check.
    try:
        rclpy.init(args=[])
        node = rclpy.create_node("ego_stack_health")
    except Exception as e:
        print(f"[ego-health] cannot join the ROS graph: {e}")
        return 1

    try:
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            if node.count_publishers(REQUIRED_TOPIC) > 0:
                print(f"[ego-health] ok: {REQUIRED_TOPIC} has a publisher; "
                      "the ego stack's ADAPI is up")
                return 0
            rclpy.spin_once(node, timeout_sec=0.2)
        print(f"[ego-health] not ready: nothing publishes {REQUIRED_TOPIC} after "
              f"{args.timeout:.0f}s. Start the ego stack first (`just ego-av`) and wait "
              "for 'Startup complete'.")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
