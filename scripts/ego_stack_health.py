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

Ready also means planning is alive. play_launch runs each composable node as its own child
process (`--container-mode isolated`), so one of them can die while its container, the
ADAPI and the operation-mode topic carry on. behavior_path_planner did exactly that on
2026-09-28 -- `rclcpp::exceptions::RCLError: failed to add guard condition to wait set:
guard condition implementation is invalid`, raised while a new route reset its modules
(autoware_universe#12460, rclcpp#2163, both open) -- and this check answered "ok" for two
more scenarios that then sat at the spawn until their timeouts. So the check also wants a
publisher on each planning output and asks play_launch's ledger for crashed composables.
With `--reload-failed` it asks play_launch to load a crashed composable again and waits for
the publishers to come back, which is the containment for a race nothing upstream fixes.

An unmanaged ego needs one more thing. With the concealer inert nothing in SSv2
routes or engages it -- `acb_pilot`'s `auto_drive` is the only thing that does, and it is
an ordinary node that can exit on its own (it has a deadline, and it fails if no vehicle
ever appears). If it is already gone when a scenario starts, the ego spawns, sits still,
and the run dies at the storyboard's timeout naming nothing. `--require-pilot` turns that
into a refusal that names the pilot.

    ROS_DOMAIN_ID=1 scripts/ego_stack_health.py [--timeout 10] [--require-pilot]
                                                [--web-port 8082] [--reload-failed]
"""

import argparse
import json
import sys
import urllib.error
import urllib.request

# What the concealer talks to. Operation mode is the one it drives through
# (stop -> autonomous) and is published by the ADAPI adaptors, so its presence means the
# API layer is up rather than merely the launch having been started.
REQUIRED_TOPIC = "/api/operation_mode/state"

# Planning outputs whose publisher disappears with the node that owns it. Each one is a
# different composable, so a dead behavior_path_planner, behavior_velocity_planner or
# motion planner shows up here even though the ADAPI still answers.
PLANNING_TOPICS = (
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
    "/planning/scenario_planning/lane_driving/behavior_planning/path",
    "/planning/scenario_planning/trajectory",
)

# The node `acb_pilot`'s auto_drive entry point creates. Only required for an
# unmanaged ego, where it stands in for the concealer.
PILOT_NODE = "auto_drive"


def play_launch_get(port: int, path: str):
    """One GET against the stack's play_launch web API, or None if it cannot be asked."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=2) as r:
            return json.load(r)
    except (urllib.error.URLError, OSError, ValueError):
        return None


def failed_composables(port: int):
    """Names of composables play_launch saw crash (state Failed), or None if unknown.

    play_launch 0.12 does not respawn a crashed composable; it only records the state.
    """
    nodes = play_launch_get(port, "/api/nodes")
    if nodes is None:
        return None
    if isinstance(nodes, dict):
        nodes = nodes.get("nodes", list(nodes.values()))
    failed = []
    for n in nodes:
        status = n.get("status") or {}
        if status.get("type") != "Composable":
            continue
        value = status.get("value")
        state = value.get("status") if isinstance(value, dict) else value
        if str(state).lower() == "failed":
            failed.append(n.get("name") or n.get("id"))
    return failed


def reload_composable(port: int, name: str) -> bool:
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/nodes/{name}/load", method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return 200 <= r.status < 300
    except (urllib.error.URLError, OSError):
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait for the topic to appear with a live publisher",
    )
    ap.add_argument(
        "--require-pilot",
        action="store_true",
        help="also require acb_pilot's auto_drive node (an unmanaged ego has no other "
        "way to be routed or engaged)",
    )
    ap.add_argument(
        "--web-port",
        type=int,
        default=8082,
        help="the stack's play_launch --web-addr port, asked for crashed composables",
    )
    ap.add_argument(
        "--reload-failed",
        action="store_true",
        help="ask play_launch to load a crashed composable again instead of refusing",
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

    reloaded = []
    try:
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            if node.count_publishers(REQUIRED_TOPIC) > 0:
                dead = [t for t in PLANNING_TOPICS if node.count_publishers(t) == 0]
                failed = failed_composables(args.web_port) or []
                if failed and args.reload_failed:
                    for name in failed:
                        if name in reloaded:
                            continue
                        ok = reload_composable(args.web_port, name)
                        print(f"[ego-health] composable {name} had crashed; asked "
                              f"play_launch to load it again: {'accepted' if ok else 'refused'}")
                        reloaded.append(name)
                    # Loading takes a few seconds; give the publishers time to come back.
                    deadline = max(deadline, time.time() + 30.0)
                    rclpy.spin_once(node, timeout_sec=1.0)
                    continue
                if dead or failed:
                    if time.time() + 0.2 < deadline:  # discovery may still be settling
                        rclpy.spin_once(node, timeout_sec=0.2)
                        continue
                    print(
                        "[ego-health] not ready: the ADAPI is up but planning is not -- "
                        + (f"no publisher on {', '.join(dead)}" if dead else "")
                        + ("; " if dead and failed else "")
                        + (f"play_launch reports crashed composable(s): {', '.join(failed)}"
                           if failed else "")
                        + ". A composable died inside its container (see "
                        "play_log/ego/latest/play_launch.log for 'crashed:'); rerun with "
                        "--reload-failed, `curl -X POST localhost:"
                        f"{args.web_port}/api/nodes/<name>/load`, or restart `just ego-av`."
                    )
                    return 1
                if args.require_pilot and PILOT_NODE not in (
                    n for n, _ns in node.get_node_names_and_namespaces()
                ):
                    print(
                        f"[ego-health] not ready: the ADAPI is up but {PILOT_NODE} is "
                        "not running. An unmanaged ego is routed and engaged only by "
                        "acb_pilot; without it the ego would spawn and never move. "
                        "Check the pilot's log under play_log/ego/*/node/auto_drive."
                    )
                    return 1
                print(f"[ego-health] ok: {REQUIRED_TOPIC} has a publisher and planning "
                      "publishes; the ego stack is up"
                      + (f" (reloaded {', '.join(reloaded)})" if reloaded else "")
                      + (f", and {PILOT_NODE} is running" if args.require_pilot else ""))
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
