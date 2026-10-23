#!/usr/bin/env python3
"""Measure every vehicle, walker and prop blueprint's bounding box (roadmap 022) and every
vehicle's axles (roadmap 050).

CARLA gives a blueprint no box; only a spawned actor has one. This spawns each `vehicle.*`,
`walker.pedestrian.*` and `static.prop.*` blueprint once, high above the map where nothing
can touch it, reads `bounding_box` (and a vehicle's wheels from `physics_control`), destroys
it, and writes the table the world server picks bodies from and scenarios take their boxes
from (`scripts/scenario_vehicle.py`):

    scripts/measure_blueprint_boxes.py [--host H] [--port P]
        [--output src/carla_world/config/blueprint_boxes_<version>.yaml]

A vehicle's `axles` are in the actor's frame (x forward, as ROS; metres): `front_x` and
`rear_x`, the axle centres' x from the actor origin; `track`, the rear wheels' distance;
`wheel_radius`, the rear wheels'. Front wheels are the steered ones. The rear axle is not at
`-wheel_base / 2`: on the Model 3 it is at -1.386 with the front at +1.618.

Rerun it for every CARLA version, and whenever the bridge warns that a spawned actor's box
differs from the table ("stale blueprint box table").

Run it while no scenario is active: it never ticks, so it does not fight the bridge for a
synchronous world, but a scenario's entities would share the world with the probes. It
attaches nothing, so destroying each actor leaves nothing behind.
"""

import argparse
import datetime
import math
import sys
from pathlib import Path

PATTERNS = ("vehicle.*", "walker.pedestrian.*", "static.prop.*")
KIND = {"vehicle": "vehicle", "walker": "pedestrian", "static": "misc_object"}
# Above every town's terrain; one place, reused, since each probe is gone before the next.
PROBE_Z = 500.0


def attribute(bp, name, cast):
    if not bp.has_attribute(name):
        return None
    try:
        return cast(bp.get_attribute(name))
    except (RuntimeError, ValueError):
        return None


def axles(actor, carla):
    """The vehicle's axles in its own frame, or None when its wheels are not one steered and
    one fixed axle (or there are none)."""
    try:
        wheels = actor.get_physics_control().wheels
    except RuntimeError:
        return None
    if not wheels:
        return None
    tf = actor.get_transform()
    points = []
    for w in wheels:
        p = carla.Location(w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0)
        # CARLA reports wheel positions in world centimetres once the vehicle has been
        # simulated, and in the actor's frame before that: tell them apart by height (the
        # probe is PROBE_Z up).
        if abs(p.z - tf.location.z) < 10.0:
            p = tf.inverse_transform(p)
        points.append((p, w))
    front = [(p, w) for p, w in points if w.max_steer_angle > 0.0]
    rear = [(p, w) for p, w in points if w.max_steer_angle <= 0.0]
    if not front or not rear:
        return None

    def mean_x(group):
        return sum(p.x for p, _ in group) / len(group)

    ys = [p.y for p, _ in rear]
    return {
        "front_x": round(mean_x(front), 3),
        "rear_x": round(mean_x(rear), 3),
        "track": round(max(ys) - min(ys), 3),
        "wheel_radius": round(rear[0][1].radius / 100.0, 3),
    }


def measure(world, bp, carla):
    """The blueprint's box from a spawned probe, or None if CARLA refused the spawn or the
    actor has no finite box."""
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "swb_box_probe")
    actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(0.0, 0.0, PROBE_Z)))
    if actor is None:
        return None
    try:
        box = actor.bounding_box
        extent = (box.extent.x, box.extent.y, box.extent.z)
        if not all(math.isfinite(e) and e > 0.0 for e in extent):
            return None  # static.prop.mesh: an empty mesh has a box of -inf
        result = {
            "length": round(2.0 * box.extent.x, 3),
            "width": round(2.0 * box.extent.y, 3),
            "height": round(2.0 * box.extent.z, 3),
            # Box centre relative to the actor's origin, CARLA (left-handed) frame.
            "center": [round(box.location.x, 3), round(box.location.y, 3),
                       round(box.location.z, 3)],
        }
        if bp.id.startswith("vehicle."):
            axle = axles(actor, carla)
            if axle is not None:
                result["axles"] = axle
        return result
    finally:
        actor.destroy()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--output", type=Path,
                    help="default: src/carla_world/config/"
                         "blueprint_boxes_<server version>.yaml")
    args = ap.parse_args()

    import carla
    import yaml

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    version = client.get_server_version()
    world = client.get_world()
    library = world.get_blueprint_library()

    table, skipped = {}, []
    for pattern in PATTERNS:
        for bp in sorted(library.filter(pattern), key=lambda b: b.id):
            box = measure(world, bp, carla)
            if box is None:
                skipped.append(bp.id)
                continue
            entry = {"kind": KIND[bp.id.split(".")[0]]}
            # CARLA spells it inconsistently ("Bus") and leaves it empty on a few cars
            # (bmw.grandtourer, mini.cooper_s); the bridge treats a missing one on four
            # wheels as a car.
            base_type = attribute(bp, "base_type", lambda a: a.as_str().strip().lower())
            if base_type:
                entry["base_type"] = base_type
            wheels = attribute(bp, "number_of_wheels", lambda a: a.as_int())
            if wheels is not None:
                entry["wheels"] = wheels
            entry.update(box)
            table[bp.id] = entry
            print(f"{bp.id:48s} {box['length']:7.3f} {box['width']:6.3f} {box['height']:6.3f}"
                  f" {entry.get('base_type', '')}")

    output = args.output or (Path(__file__).resolve().parent.parent
                             / "src/carla_world/config"
                             / f"blueprint_boxes_{version}.yaml")
    header = (
        f"# Bounding boxes of CARLA {version}'s blueprints, measured from spawned actors.\n"
        f"# Generated by scripts/measure_blueprint_boxes.py on "
        f"{datetime.date.today().isoformat()} ({world.get_map().name.split('/')[-1]});\n"
        "# do not edit by hand. Lengths in metres; `center` is the box centre relative to\n"
        "# the actor's origin in CARLA's (left-handed) frame; `axles` are a vehicle's axle x\n"
        "# from the actor origin (x forward), rear track and wheel radius. Read by the world\n"
        "# server (roadmap 022) and scripts/scenario_vehicle.py (roadmap 050).\n"
    )
    body = {"carla_version": version, "blueprints": table}
    output.write_text(header + yaml.safe_dump(body, sort_keys=False, width=100, default_flow_style=None))
    print(f"\n{len(table)} blueprints -> {output}")
    if skipped:
        print(f"skipped {len(skipped)} (spawn refused, or no finite box): {', '.join(skipped)}",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
