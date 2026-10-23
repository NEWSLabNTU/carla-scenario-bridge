#!/usr/bin/env python3
"""Scenarios refer to the vehicle catalog (roadmap 051): an inline `<Vehicle>` whose body is a
cataloged car becomes a `CatalogReference` to its entry.

    scripts/catalog_scenarios.py --rewrite a.xosc b.xosc   # rewrite in place
    scripts/catalog_scenarios.py --check a.xosc            # exit 1 if a file would change

The catalog is sim_vehicle_spec's `catalogs/vehicles.xosc`, generated from the vehicle specs:
one entry per car, its box and axles in the spec's base_link, `model3d` its CARLA body. A
scenario reaches it through `$(find-pkg-share sim_vehicle_spec)/catalogs` (CATALOG_DIR), so
the box and axles are the spec's, never a copy. What a scenario may choose stays its own and
is assigned: `maxSpeed`, `maxAcceleration`, `maxDeceleration` (its `Performance`) and
`maxSteering` (its front axle's). The rewrite assigns all four from the inline declaration, so
nothing a scenario drives changes.

A vehicle is cataloged when its `name` or `model3d` is an entry's `model3d`. One carrying the
`swb.box_exempt` property (scripts/check_scenario_vehicles.py) is left inline.
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CATALOG = ROOT / "src/sim_vehicle_spec/catalogs/vehicles.xosc"
CATALOG_NAME = "vehicles"
CATALOG_DIR = "$(find-pkg-share sim_vehicle_spec)/catalogs"
EXEMPT = "swb.box_exempt"
# The parameters a scenario assigns: (parameter, element under <Vehicle>, attribute)
ASSIGNED = (("maxSpeed", "Performance", "maxSpeed"),
            ("maxAcceleration", "Performance", "maxAcceleration"),
            ("maxDeceleration", "Performance", "maxDeceleration"),
            ("maxSteering", "Axles/FrontAxle", "maxSteering"))

VEHICLE = re.compile(r'(?P<indent>[ \t]*)<Vehicle\b(?P<attrs>[^>]*)>.*?</Vehicle>', re.S)
EMPTY_LOCATIONS = re.compile(r'<CatalogLocations\s*/>')
LOCATIONS = re.compile(r'<CatalogLocations\s*>(?P<body>.*?)</CatalogLocations>', re.S)


def entries(path=CATALOG):
    """{body (model3d): entry name} of the catalog."""
    root = ET.parse(path).getroot()
    return {v.get("model3d"): v.get("name") for v in root.iter("Vehicle") if v.get("model3d")}


def body_of(vehicle):
    """The CARLA body a <Vehicle> element names: its model3d, else its name."""
    return vehicle.get("model3d") or vehicle.get("name", "")


def exempt(vehicle):
    return any(p.get("name") == EXEMPT for p in vehicle.findall("Properties/Property"))


def reference(vehicle, entry, indent):
    """The CatalogReference replacing an inline <Vehicle>, with its assignments."""
    step = "\t" if indent.startswith("\t") else "  "
    lines = [f'{indent}<CatalogReference catalogName="{CATALOG_NAME}" entryName="{entry}">',
             f"{indent}{step}<ParameterAssignments>"]
    for param, element, attribute in ASSIGNED:
        node = vehicle.find(element)
        if node is None or node.get(attribute) is None:
            raise ValueError(f"vehicle {vehicle.get('name')!r}: no {element} {attribute}")
        lines.append(f'{indent}{step}{step}<ParameterAssignment parameterRef="{param}" '
                     f'value="{node.get(attribute)}" />')
    lines += [f"{indent}{step}</ParameterAssignments>", f"{indent}</CatalogReference>"]
    return "\n".join(lines)


def with_location(text):
    """`text` whose CatalogLocations has the vehicle catalog's directory."""
    if CATALOG_DIR in text:
        return text
    vehicle_catalog = f'<VehicleCatalog><Directory path="{CATALOG_DIR}" /></VehicleCatalog>'
    if EMPTY_LOCATIONS.search(text):
        return EMPTY_LOCATIONS.sub(f"<CatalogLocations>{vehicle_catalog}</CatalogLocations>",
                                   text, count=1)
    m = LOCATIONS.search(text)
    if m is None:
        raise ValueError("no CatalogLocations")
    if "<VehicleCatalog" in m.group("body"):
        raise ValueError("a VehicleCatalog other than sim_vehicle_spec's")
    return text[:m.end("body")] + vehicle_catalog + text[m.end("body"):]


def rewrite(text, catalog=None):
    """(new text, number of vehicles moved to the catalog)."""
    catalog = catalog if catalog is not None else entries()
    count = 0

    def vehicle(m):
        nonlocal count
        node = ET.fromstring(m.group(0).strip())
        entry = catalog.get(body_of(node))
        if entry is None or exempt(node):
            return m.group(0)
        count += 1
        return reference(node, entry, m.group("indent"))

    new = VEHICLE.sub(vehicle, text)
    return (with_location(new) if count else new), count


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--rewrite", nargs="+", type=Path, metavar="XOSC")
    mode.add_argument("--check", nargs="+", type=Path, metavar="XOSC")
    ap.add_argument("--catalog", type=Path, default=CATALOG)
    args = ap.parse_args()
    catalog = entries(args.catalog)
    changed = 0
    for path in args.rewrite or args.check:
        text = path.read_text()
        new, count = rewrite(text, catalog)
        if new != text:
            changed += 1
            if args.rewrite:
                path.write_text(new)
        print(f"{path}: {count} vehicle(s) {'to the catalog' if new != text else 'unchanged'}")
    return 1 if args.check and changed else 0


if __name__ == "__main__":
    sys.exit(main())
