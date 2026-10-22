"""Traffic signals in the Isaac world (roadmap 044 B), pure Python so it is tested without Isaac.

swb sends the map's signal table with `LoadWorld` (per Lanelet2 traffic light way: its
regulatory elements, OpenDRIVE id, CARLA position with Y flipped) and the colours SSv2
commands with `SetSignals`, by OpenDRIVE id. A CARLA town exported with SimReady has one
prim per CARLA traffic light carrying `omni:simready:signalID` (its OpenDRIVE id) at the
actor's position; `match_lamps` pairs table and lamps, `groups` turns the lamps' colours
into what Autoware gets: one colour per regulatory element.
"""
import math

COLOURS = ("OFF", "RED", "AMBER", "GREEN")
# Most restrictive last: a regulatory element whose lights disagree shows the most
# restrictive known colour (acb's `group_colour`); OFF is "unknown", the least.
RESTRICTIVENESS = {"OFF": 0, "GREEN": 1, "AMBER": 2, "RED": 3}
# A lamp found by position must stand this close to the table's, m.
POSITION_TOLERANCE = 1.5


def match_lamps(signals, lamps, tolerance=POSITION_TOLERANCE):
    """Pair the table's signals with the world's lamps.

    `signals`: dicts with `opendrive_id` and `position` ((x, y, z) or None).
    `lamps`: `(path, signal_id, (x, y, z))`, `signal_id` a string or None.

    Returns `(by_id, how, unmatched)`: OpenDRIVE id -> lamp path, OpenDRIVE id -> "id" or
    "position", and the OpenDRIVE ids with no lamp. A lamp is used once: by its id first,
    else the nearest free one within `tolerance` of the table's position."""
    by_signal_id = {}
    for path, sid, _ in lamps:
        if sid is not None:
            by_signal_id.setdefault(str(sid), path)
    by_id, how, used = {}, {}, set()
    for s in signals:
        od = str(s["opendrive_id"])
        if od in by_id:
            continue
        path = by_signal_id.get(od)
        if path is not None:
            by_id[od], how[od] = path, "id"
            used.add(path)
    for s in signals:
        od = str(s["opendrive_id"])
        if od in by_id or s.get("position") is None:
            continue
        x, y = s["position"][0], s["position"][1]
        best = None
        for path, _, p in lamps:
            if path in used:
                continue
            d = math.hypot(p[0] - x, p[1] - y)
            if d <= tolerance and (best is None or d < best[0]):
                best = (d, path)
        if best is not None:
            by_id[od], how[od] = best[1], "position"
            used.add(best[1])
    unmatched = sorted({str(s["opendrive_id"]) for s in signals} - set(by_id))
    return by_id, how, unmatched


def groups(signals, colours):
    """`[(regulatory element id, colour)]` for Autoware: each regulatory element's lights'
    most restrictive colour (`colours`: OpenDRIVE id -> colour name; a lamp not set is OFF),
    in table order."""
    out = {}
    for s in signals:
        c = colours.get(str(s["opendrive_id"]), "OFF")
        for reg in s.get("regulatory_element_ids", ()):
            prev = out.get(reg)
            if prev is None or RESTRICTIVENESS[c] > RESTRICTIVENESS[prev]:
                out[reg] = c
    return list(out.items())
