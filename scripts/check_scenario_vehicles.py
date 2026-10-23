#!/usr/bin/env python3
"""Lint: every `<Vehicle>` named by a CARLA blueprint declares that blueprint's box and axles
(roadmap 050).

    scripts/check_scenario_vehicles.py [PATH ...] [--tolerance M]

PATHs are `.xosc` files or directories searched for them (default: `src/swb_examples` and
`scenarios`). For each `<Vehicle name="...">` whose name is a blueprint with axles in the
measured table, the declared `BoundingBox` and `Axles` must equal what
`scripts/scenario_vehicle.py` derives from the table, within the tolerance (default 0.02 m):
Center x/y/z, Dimensions, both axles' wheelDiameter, trackWidth and positionZ, the front
axle's positionX, and the rear axle's positionX (0, the vehicle origin).

A vehicle that differs on purpose carries an OpenSCENARIO property naming why, and is
listed rather than failed:

    <Properties><Property name="swb.box_exempt" value="why" /></Properties>

Exit status 1 on any failure, each named by file, entity, field, declared and measured value.
Run by `just test`.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import scenario_vehicle  # scripts/, beside this file

ROOT = scenario_vehicle.ROOT
DEFAULT_PATHS = [ROOT / "src/swb_examples", ROOT / "scenarios"]
EXEMPT = "swb.box_exempt"

# (element path under <Vehicle>, attribute, geometry key; None = expected 0)
FIELDS = [
    ("BoundingBox/Center", "x", "center_x"),
    ("BoundingBox/Center", "y", "center_y"),
    ("BoundingBox/Center", "z", "center_z"),
    ("BoundingBox/Dimensions", "length", "length"),
    ("BoundingBox/Dimensions", "width", "width"),
    ("BoundingBox/Dimensions", "height", "height"),
    ("Axles/FrontAxle", "positionX", "front_x"),
    ("Axles/FrontAxle", "positionZ", "axle_z"),
    ("Axles/FrontAxle", "wheelDiameter", "wheel_diameter"),
    ("Axles/FrontAxle", "trackWidth", "track"),
    ("Axles/RearAxle", "positionX", None),
    ("Axles/RearAxle", "positionZ", "axle_z"),
    ("Axles/RearAxle", "wheelDiameter", "wheel_diameter"),
    ("Axles/RearAxle", "trackWidth", "track"),
]


def scenario_files(paths):
    for path in paths:
        path = Path(path)
        if path.is_dir():
            yield from sorted(path.rglob("*.xosc"))
        elif path.suffix == ".xosc":
            yield path


def entity_of(parents, vehicle):
    """The ScenarioObject (or CatalogEntry) name a <Vehicle> belongs to."""
    node = parents.get(vehicle)
    while node is not None:
        if node.tag in ("ScenarioObject", "Vehicle", "CatalogEntry") and node.get("name"):
            if node is not vehicle:
                return node.get("name")
        node = parents.get(node)
    return "?"


def check_file(path, table, tolerance):
    """(failures, exemptions, vehicles checked) for one file."""
    failures, exemptions, checked = [], [], 0
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        return [f"{path}: not XML ({e})"], [], 0
    parents = {child: parent for parent in root.iter() for child in parent}
    for vehicle in root.iter("Vehicle"):
        geometry = scenario_vehicle.geometry(vehicle.get("name", ""), table)
        if geometry is None:
            continue
        entity = entity_of(parents, vehicle)
        where = f"{path.relative_to(ROOT) if path.is_relative_to(ROOT) else path}: {entity}"
        exempt = [p.get("value", "") for p in vehicle.findall("Properties/Property")
                  if p.get("name") == EXEMPT]
        if exempt:
            exemptions.append(f"{where} ({vehicle.get('name')}): {exempt[0]}")
            continue
        checked += 1
        for element, attribute, key in FIELDS:
            node = vehicle.find(element)
            expected = 0.0 if key is None else geometry[key]
            if node is None or node.get(attribute) is None:
                failures.append(f"{where}: {element} has no {attribute} "
                                f"(measured {scenario_vehicle.fmt(expected)})")
                continue
            try:
                declared = float(node.get(attribute))
            except ValueError:
                failures.append(f"{where}: {element} {attribute}={node.get(attribute)!r} "
                                "is not a number")
                continue
            if abs(declared - expected) > tolerance:
                failures.append(f"{where}: {element} {attribute} declared "
                                f"{node.get(attribute)}, measured {scenario_vehicle.fmt(expected)}")
    return failures, exemptions, checked


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="*", type=Path)
    ap.add_argument("--tolerance", type=float, default=0.02)
    ap.add_argument("--table", type=Path, default=scenario_vehicle.TABLE)
    args = ap.parse_args()
    table = scenario_vehicle.load_table(args.table)
    failures, exemptions, checked, files = [], [], 0, 0
    for path in scenario_files(args.paths or DEFAULT_PATHS):
        f, e, c = check_file(path.resolve(), table, args.tolerance)
        failures += f
        exemptions += e
        checked += c
        files += 1
    for line in exemptions:
        print(f"exempt: {line}")
    for line in failures:
        print(f"FAIL: {line}")
    print(f"scenario vehicles: {checked} checked in {files} files, {len(exemptions)} exempt, "
          f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
