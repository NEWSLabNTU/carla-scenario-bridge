#!/usr/bin/env python3
"""A CARLA blueprint's box and axles as an OpenSCENARIO `<Vehicle>` declares them (roadmap 050).

SSv2's vehicle origin is the rear axle on the ground; CARLA's actor origin is wherever the
blueprint put it. The measured table (`src/carla_world/config/blueprint_boxes_<version>.yaml`,
written by `scripts/measure_blueprint_boxes.py`) has both the box (centre from the actor
origin) and the axles (x from the actor origin), so:

    BoundingBox Center x = box centre x - rear axle x
                Center y = -(box centre y)          CARLA is left-handed
                Center z = height / 2               the box stands on the ground
    FrontAxle   positionX = front axle x - rear axle x
    both axles  positionZ = wheel radius, wheelDiameter = 2 * radius, trackWidth = rear track

For the Model 3: Center (1.415, 0, 0.744), 4.792 x 2.163 x 1.488, wheel base 3.004.

Used by the scenario generators (`geometry`), by the one-off rewrite of the hand-written
scenarios (`rewrite`, `--rewrite FILE...`) and by the lint (`scripts/check_scenario_vehicles.py`).
`maxSteering` is not measured here and is kept as the scenario has it (051 takes it from the
vehicle spec).

    scripts/scenario_vehicle.py vehicle.tesla.model3         # print the declaration's numbers
    scripts/scenario_vehicle.py --rewrite a.xosc b.xosc      # fix every <Vehicle> named by a blueprint
"""

import argparse
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
TABLE = ROOT / "src/carla_world/config/blueprint_boxes_0.9.16.yaml"


def fmt(v):
    """Millimetres, trailing zeros off, at least one decimal: 1.415, 0.0, 4.792, 2.1."""
    text = f"{v:.3f}".rstrip("0")
    text = text + "0" if text.endswith(".") else text
    return "0.0" if text in ("-0.0", "0.0") else text


def load_table(path=TABLE):
    return yaml.safe_load(Path(path).read_text())["blueprints"]


def geometry(blueprint, table=None):
    """The declaration's numbers for a blueprint, as floats, or None if the table has no
    box and axles for it."""
    table = table if table is not None else load_table()
    bp = table.get(blueprint)
    if not bp or "axles" not in bp:
        return None
    a = bp["axles"]
    return {
        "center_x": bp["center"][0] - a["rear_x"],
        "center_y": -bp["center"][1],
        "center_z": bp["height"] / 2.0,
        "length": bp["length"],
        "width": bp["width"],
        "height": bp["height"],
        "front_x": a["front_x"] - a["rear_x"],
        "axle_z": a["wheel_radius"],
        "wheel_diameter": 2.0 * a["wheel_radius"],
        "track": a["track"],
    }


def strings(blueprint, table=None):
    """`geometry` formatted the way the scenarios write numbers."""
    g = geometry(blueprint, table)
    return None if g is None else {k: fmt(v) for k, v in g.items()}


# The attributes each element of a <Vehicle> takes from the table.
ATTRIBUTES = {
    "Center": {"x": "center_x", "y": "center_y", "z": "center_z"},
    "Dimensions": {"length": "length", "width": "width", "height": "height"},
    "FrontAxle": {"wheelDiameter": "wheel_diameter", "trackWidth": "track",
                  "positionX": "front_x", "positionZ": "axle_z"},
    "RearAxle": {"wheelDiameter": "wheel_diameter", "trackWidth": "track",
                 "positionZ": "axle_z"},
}
VEHICLE = re.compile(r'(<Vehicle\b[^>]*\bname="(?P<name>[^"]+)"[^>]*>)(?P<body>.*?)(</Vehicle>)',
                     re.S)
ELEMENT = re.compile(r"<(?P<tag>Center|Dimensions|FrontAxle|RearAxle)\b(?P<attrs>[^>]*)/>")


def _set(attrs, key, value):
    pattern = re.compile(rf'(\b{key}=")[^"]*(")')
    if not pattern.search(attrs):
        raise ValueError(f"no {key}= in {attrs!r}")
    return pattern.sub(lambda m: m.group(1) + value + m.group(2), attrs, count=1)


def rewrite(text, table=None):
    """`text` with every `<Vehicle>` whose name is a blueprint with axles in the table given
    that blueprint's box and axles, attribute values only: layout, order and every other
    attribute stay. Returns (new text, number of vehicles rewritten)."""
    table = table if table is not None else load_table()
    count = 0

    def vehicle(m):
        nonlocal count
        s = strings(m.group("name"), table)
        if s is None:
            return m.group(0)

        def element(e):
            attrs = e.group("attrs")
            for key, field in ATTRIBUTES[e.group("tag")].items():
                attrs = _set(attrs, key, s[field])
            return f"<{e.group('tag')}{attrs}/>"

        count += 1
        return m.group(1) + ELEMENT.sub(element, m.group("body")) + m.group(4)

    return VEHICLE.sub(vehicle, text), count


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("blueprint", nargs="?")
    ap.add_argument("--rewrite", nargs="+", type=Path, metavar="XOSC")
    ap.add_argument("--table", type=Path, default=TABLE)
    args = ap.parse_args()
    table = load_table(args.table)
    if args.rewrite:
        for path in args.rewrite:
            text = path.read_text()
            new, count = rewrite(text, table)
            if new != text:
                path.write_text(new)
            print(f"{path}: {count} vehicle(s){'' if new != text else ', unchanged'}")
        return 0
    if not args.blueprint:
        ap.error("give a blueprint or --rewrite")
    s = strings(args.blueprint, table)
    if s is None:
        print(f"{args.blueprint}: no box and axles in {args.table}", file=sys.stderr)
        return 1
    for k, v in s.items():
        print(f"{k:15s} {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
