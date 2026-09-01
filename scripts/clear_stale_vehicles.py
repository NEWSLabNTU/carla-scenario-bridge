#!/usr/bin/env python3
"""Remove vehicles left in CARLA by a run that did not clean up after itself.

SSv2 despawns nothing when its scenario stack is killed, so an interrupted run leaves its
ego in the world. The next stack's pilot then finds that vehicle's GNSS, localizes to it,
routes it, engages it, and spends its whole budget driving a car the new run is not about
-- the scenario later despawns it and spawns the real ego, which nothing is left to drive.
Measured once as an ego that "drove for ten minutes and never arrived"; see
acb `195467a` and phase 013's notes.

Only vehicles are removed. Sensors belong to whatever attached them and are cleaned up with
their vehicle; traffic lights and the spectator are part of the map.

    scripts/clear_stale_vehicles.py [--host H] [--port P] [--role NAME ...]

Exits 0 when the world is clear, including when there was nothing to do, so it is safe to
run unconditionally before a stack comes up. Exits 1 only if CARLA could not be reached --
a caller that cannot clear the world should not go on to start a run in it.
"""

import argparse
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument(
        "--role", action="append", default=None,
        help="only remove vehicles with this role_name; repeatable. Default: every vehicle, "
             "because a background AV left behind is as much of a problem as a stale ego -- "
             "it parks in the lane the next run drives down.")
    args = ap.parse_args()

    try:
        import carla
    except ImportError as e:
        print(f"[clear-vehicles] cannot import carla: {e}")
        return 1

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        vehicles = list(world.get_actors().filter("vehicle.*"))
    except Exception as e:
        print(f"[clear-vehicles] cannot reach CARLA at {args.host}:{args.port}: {e}")
        return 1

    removed, kept = [], []
    for v in vehicles:
        role = v.attributes.get("role_name", "")
        if args.role is not None and role not in args.role:
            kept.append(role or str(v.id))
            continue
        try:
            v.destroy()
            removed.append(role or str(v.id))
        except Exception as e:
            print(f"[clear-vehicles] could not destroy {v.id} ({role}): {e}")

    if removed:
        print(f"[clear-vehicles] removed {len(removed)}: {', '.join(sorted(removed))}")
    else:
        print("[clear-vehicles] ok: no vehicles left over")
    if kept:
        print(f"[clear-vehicles] left alone: {', '.join(sorted(kept))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
