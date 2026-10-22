"""The surface layer's map and model (roadmap 047 S1; docs/design/off-road-surfaces.md).
Pure Python (numpy, yaml; Pillow for PNG rasters): no Isaac, so the check and the plot run
anywhere and the maths is unit-tested.

`<map>/surfaces.yaml` (047 D1) says what every piece of ground is:

    classes: surface_classes.yaml      # optional: a class table beside the map; default the
                                       # one shipped in src/isaac_world/config/
    defaults: {Road: asphalt, RoadLine: asphalt, SideWalk: concrete, Terrain: grass,
               Ground: dirt, unlabelled: grass}
    zones:
      - id: mud_pit                    # [a-z0-9_]+: scenarios name it (swb.zone.<id>.<field>)
        class: mud_wet
        soil: true                     # the soil layer (S3), not only friction
        polygon: [[120.0, -40.0], [160.0, -40.0], [160.0, -10.0], [120.0, -10.0]]
      - id: ford_1
        water_depth: raster/ford_1_depth.png    # or a number, m
        scale: 0.0001                  # m per pixel count (PNG; a .npy holds metres)
        origin: [300.0, 55.0]          # the raster's lower-left corner, map frame
        resolution: 0.25               # m per pixel

`defaults` maps SimReady's semantic labels to classes (Road, RoadLine, SideWalk, Terrain,
Ground; `unlabelled` for ground without one, default the reference class). A zone is a
map-frame polygon, or a raster's extent when it has no polygon; every field but `id` is
optional and falls through to the zone below it, the label's default and the world's road
condition. Zones listed later lie on top.

A wheel's friction, every physics step (`WheelSurfaces`):

    mu = tire_friction * friction_scale * mu_class * wet_class(wetness) * f_hydroplane(v, depth)
         / mu_reference

filtered over `blend_length` of travel, then shaped by the class's slip curve. On dry
reference ground (asphalt) that is the rig's own tire_friction.

    python -m isaac_world.surfaces check <map dir | surfaces.yaml>
    python -m isaac_world.surfaces plot <map dir | surfaces.yaml> [-o out.png]
"""
import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_CLASSES = HERE.parent / "config" / "surface_classes.yaml"
FILE = "surfaces.yaml"
WETNESS = ("dry", "moist", "wetWithPuddles", "lowFlooded", "highFlooded")
LABELS = ("Road", "RoadLine", "SideWalk", "Terrain", "Ground")
UNLABELLED = "unlabelled"
# SimReady's Wikidata Q-codes for CARLA's ground classes (as isaac_world.usd_world reads them).
QCODE_LABELS = {"Q34442": "Road", "Q177749": "SideWalk", "Q1392287": "RoadLine",
                "Q186131": "Terrain", "Q36133": "Ground"}
ZONE_ID = re.compile(r"^[a-z0-9_]+$")
ZONE_KEYS = {"id", "class", "wetness", "water_depth", "soil", "polygon", "scale", "origin",
             "resolution"}


class SurfaceError(ValueError):
    """A surface map or class table that cannot be used; the message lists every problem."""


# --- Class table ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SurfaceClass:
    name: str
    mu: float
    wet: tuple  # factor per WETNESS level
    slip: tuple = None  # ((0, f0), (s_peak, 1), (1, f_lock)), or None: the vehicle's
    hydroplaning: bool = False


@dataclass(frozen=True)
class Hydroplaning:
    tire_pressure_kpa: float = 250.0
    onset: float = 0.5
    residual: float = 0.1
    depth_min: float = 0.0005
    depth_full: float = 0.0025

    @property
    def speed(self):
        """Horne's full hydroplaning speed, m/s."""
        return 6.35 * math.sqrt(self.tire_pressure_kpa) / 3.6


@dataclass(frozen=True)
class ClassTable:
    classes: dict
    reference: str
    wetness_depth: dict
    hydroplaning: Hydroplaning
    blend_length: float

    def __getitem__(self, name):
        return self.classes[name]


def _graph(pts, where, problems):
    try:
        g = tuple((float(x), float(y)) for x, y in pts)
    except (TypeError, ValueError):
        problems.append(f"{where}: not a list of [slip, friction] pairs")
        return None
    if len(g) != 3 or not (0.0 <= g[0][0] < g[1][0] < g[2][0]):
        problems.append(f"{where}: three points of rising slip, as PhysX's frictionVsSlipGraph")
        return None
    return g


def parse_classes(data, where="classes"):
    problems = []
    if not isinstance(data, dict) or not isinstance(data.get("classes"), dict):
        raise SurfaceError(f"{where}: no `classes:` mapping")
    classes = {}
    for name, c in data["classes"].items():
        if not isinstance(c, dict) or not isinstance(c.get("mu"), (int, float)) or c["mu"] <= 0:
            problems.append(f"{where}: class {name}: needs mu > 0")
            continue
        wet_in = c.get("wet") or {}
        unknown = set(wet_in) - set(WETNESS)
        if unknown:
            problems.append(f"{where}: class {name}: unknown wetness {sorted(unknown)} "
                            f"(one of {', '.join(WETNESS)})")
        wet, last = [], 1.0
        for level in WETNESS:
            last = float(wet_in.get(level, last if level != "dry" else 1.0))
            wet.append(last)
        slip = _graph(c["slip"], f"{where}: class {name}: slip", problems) \
            if c.get("slip") is not None else None
        classes[name] = SurfaceClass(name, float(c["mu"]), tuple(wet), slip,
                                     bool(c.get("hydroplaning", False)))
    reference = data.get("reference", "asphalt")
    if reference not in classes:
        problems.append(f"{where}: reference class {reference!r} is not in the table")
    depth = {w: float((data.get("wetness_depth") or {}).get(w, 0.0)) for w in WETNESS}
    hp = Hydroplaning(**{k: float(v) for k, v in (data.get("hydroplaning") or {}).items()
                         if k in Hydroplaning.__dataclass_fields__})
    if problems:
        raise SurfaceError("\n".join(problems))
    return ClassTable(classes, reference, depth, hp, float(data.get("blend_length", 0.2)))


def load_classes(path=DEFAULT_CLASSES):
    import yaml

    with open(path) as f:
        return parse_classes(yaml.safe_load(f), str(path))


# --- Rasters and zones ---------------------------------------------------------------------


@dataclass
class Raster:
    """Values on a grid: row 0 is the top (+y) edge, column 0 the left (-x); `origin` is the
    lower-left corner in the map frame, `resolution` m per pixel. Sampled bilinearly at pixel
    centres; 0 outside."""

    values: np.ndarray
    origin: tuple
    resolution: float

    @property
    def extent(self):
        h, w = self.values.shape
        x0, y0 = self.origin
        return (x0, x0 + w * self.resolution, y0, y0 + h * self.resolution)

    def polygon(self):
        x0, x1, y0, y1 = self.extent
        return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])

    def sample(self, x, y):
        h, w = self.values.shape
        u = (x - self.origin[0]) / self.resolution - 0.5
        v = (self.origin[1] + h * self.resolution - y) / self.resolution - 0.5  # row
        if u < -0.5 or v < -0.5 or u > w - 0.5 or v > h - 0.5:
            return 0.0
        u = min(max(u, 0.0), w - 1.0)
        v = min(max(v, 0.0), h - 1.0)
        c0, r0 = int(u), int(v)
        c1, r1 = min(c0 + 1, w - 1), min(r0 + 1, h - 1)
        fu, fv = u - c0, v - r0
        a = self.values
        top = a[r0, c0] * (1 - fu) + a[r0, c1] * fu
        bottom = a[r1, c0] * (1 - fu) + a[r1, c1] * fu
        return float(top * (1 - fv) + bottom * fv)


def load_raster(path, scale, origin, resolution):
    if str(path).endswith(".npy"):
        values = np.load(path).astype(np.float64)
    else:
        from PIL import Image

        with Image.open(path) as im:
            values = np.asarray(im, dtype=np.float64)
        if values.ndim == 3:
            values = values[..., 0]
        values = values * scale
    if values.ndim != 2:
        raise ValueError(f"{path}: not a one-channel image")
    return Raster(values, (float(origin[0]), float(origin[1])), float(resolution))


def inside(polygon, x, y):
    """Point in polygon (even-odd), `polygon` an (n, 2) array."""
    px, py = polygon[:, 0], polygon[:, 1]
    qx, qy = np.roll(px, -1), np.roll(py, -1)
    crosses = (py > y) != (qy > y)
    with np.errstate(divide="ignore", invalid="ignore"):
        xs = px + (y - py) * (qx - px) / (qy - py)
    return bool(np.count_nonzero(crosses & (x < xs)) % 2)


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return (orient(a, b, c) * orient(a, b, d) < 0) and (orient(c, d, a) * orient(c, d, b) < 0)


def self_intersects(polygon):
    n = len(polygon)
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if _segments_cross(polygon[i], polygon[(i + 1) % n], polygon[j], polygon[(j + 1) % n]):
                return True
    return False


@dataclass
class Zone:
    id: str
    polygon: np.ndarray
    cls: str = None
    wetness: str = None
    water_depth: object = None  # float, Raster or None
    soil: bool = False
    bbox: tuple = field(init=False)

    def __post_init__(self):
        lo, hi = self.polygon.min(axis=0), self.polygon.max(axis=0)
        self.bbox = (lo[0], hi[0], lo[1], hi[1])

    def contains(self, x, y):
        x0, x1, y0, y1 = self.bbox
        return x0 <= x <= x1 and y0 <= y <= y1 and inside(self.polygon, x, y)

    def depth_at(self, x, y):
        if isinstance(self.water_depth, Raster):
            return self.water_depth.sample(x, y)
        return self.water_depth


# --- The map -------------------------------------------------------------------------------


@dataclass(frozen=True)
class Surface:
    """The ground at one point: its class, and the water on it (None: the world's)."""

    cls: str
    wetness: str = None
    water_depth: float = None
    zone: str = None  # the top zone there, if any
    soil: bool = False


@dataclass
class SurfaceMap:
    path: str
    table: ClassTable
    defaults: dict  # label (or UNLABELLED) -> class
    zones: list

    def at(self, x, y, label=None):
        """The surface at map point (x, y) over ground labelled `label` (None: unlabelled)."""
        cls = wetness = depth = top = None
        soil = None
        for z in reversed(self.zones):
            if not z.contains(x, y):
                continue
            top = top or z.id
            cls = cls or z.cls
            wetness = wetness or z.wetness
            if depth is None and z.water_depth is not None:
                depth = z.depth_at(x, y)
            if soil is None and z.cls is not None:
                soil = z.soil
            if cls and wetness and depth is not None:
                break
        if cls is None:
            cls = self.defaults.get(label or UNLABELLED, self.defaults[UNLABELLED])
        return Surface(cls, wetness, depth, top, bool(soil))


def find(map_or_file):
    """`surfaces.yaml` for a map directory, or the file itself."""
    p = Path(map_or_file)
    return p / FILE if p.is_dir() else p


def parse(data, path=FILE, table=None, base=None):
    """A SurfaceMap from surfaces.yaml's contents; SurfaceError lists every problem."""
    base = Path(base or Path(path).parent)
    if not isinstance(data, dict):
        raise SurfaceError(f"{path}: not a mapping")
    problems = []
    unknown = set(data) - {"classes", "defaults", "zones"}
    if unknown:
        problems.append(f"{path}: unknown key(s) {sorted(unknown)} (classes, defaults, zones)")
    if table is None:
        classes = data.get("classes")
        try:
            table = load_classes(base / classes if classes else DEFAULT_CLASSES)
        except (OSError, SurfaceError) as e:
            raise SurfaceError(f"{path}: class table: {e}") from e

    def known(cls, where):
        if cls not in table.classes:
            problems.append(f"{where}: unknown class {cls!r} (the table has "
                            f"{', '.join(sorted(table.classes))})")

    defaults = dict(data.get("defaults") or {})
    for label, cls in defaults.items():
        if label not in LABELS and label != UNLABELLED:
            problems.append(f"{path}: defaults: unknown label {label!r} "
                            f"({', '.join(LABELS)}, {UNLABELLED})")
        known(cls, f"{path}: defaults.{label}")
    defaults.setdefault(UNLABELLED, table.reference)
    for label in LABELS:
        defaults.setdefault(label, defaults[UNLABELLED])

    zones, ids = [], set()
    for n, z in enumerate(data.get("zones") or []):
        where = f"{path}: zones[{n}]"
        if not isinstance(z, dict):
            problems.append(f"{where}: not a mapping")
            continue
        zid = str(z.get("id", ""))
        where = f"{path}: zone {zid or n}"
        if not ZONE_ID.match(zid):
            problems.append(f"{where}: id must match [a-z0-9_]+ (scenarios name it)")
        elif zid in ids:
            problems.append(f"{where}: id used twice")
        ids.add(zid)
        unknown = set(z) - ZONE_KEYS
        if unknown:
            problems.append(f"{where}: unknown key(s) {sorted(unknown)}")
        if z.get("class") is not None:
            known(z["class"], where)
        if z.get("wetness") is not None and z["wetness"] not in WETNESS:
            problems.append(f"{where}: wetness {z['wetness']!r} is not one of {', '.join(WETNESS)}")
        depth = z.get("water_depth")
        raster = None
        if isinstance(depth, str):
            missing = [k for k in ("origin", "resolution") if k not in z]
            if not str(depth).endswith(".npy") and "scale" not in z:
                missing.append("scale")
            if missing:
                problems.append(f"{where}: a raster needs {', '.join(missing)}")
            else:
                try:
                    raster = load_raster(base / depth, float(z.get("scale", 1.0)), z["origin"],
                                         z["resolution"])
                    depth = raster
                except (OSError, ValueError) as e:
                    problems.append(f"{where}: raster {depth}: {e}")
        elif depth is not None:
            if not isinstance(depth, (int, float)) or depth < 0:
                problems.append(f"{where}: water_depth is metres >= 0, or a raster file")
            else:
                depth = float(depth)
        if "polygon" in z:
            try:
                poly = np.array(z["polygon"], dtype=np.float64)
            except (TypeError, ValueError):
                poly = np.zeros((0,))
            if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3:
                problems.append(f"{where}: polygon needs three or more [x, y] points")
                continue
            if self_intersects(poly):
                problems.append(f"{where}: polygon crosses itself")
        elif raster is not None:
            poly = raster.polygon()
        else:
            problems.append(f"{where}: needs a polygon (or a raster, whose extent it then is)")
            continue
        if not any(z.get(k) is not None for k in ("class", "wetness", "water_depth")):
            problems.append(f"{where}: sets none of class, wetness, water_depth")
        if z.get("soil") and not z.get("class"):
            problems.append(f"{where}: soil needs the zone's own class")
        zones.append(Zone(zid, poly, z.get("class"), z.get("wetness"),
                          depth if isinstance(depth, (float, Raster)) else None,
                          bool(z.get("soil", False))))
    if problems:
        raise SurfaceError("\n".join(problems))
    return SurfaceMap(str(path), table, defaults, zones)


def load(map_or_file):
    import yaml

    path = find(map_or_file)
    with open(path) as f:
        return parse(yaml.safe_load(f), str(path))


# --- Friction ------------------------------------------------------------------------------


@dataclass
class RoadCondition:
    """The world's road condition (OpenSCENARIO's RoadCondition, 047 S2): applied where no
    zone sets its own wetness."""

    friction_scale: float = 1.0
    wetness: str = "dry"


def hydroplane_factor(hp: Hydroplaning, speed, depth):
    """Grip left at `speed` (m/s) on `depth` (m) of standing water, 0..1."""
    if depth <= hp.depth_min:
        return 1.0
    w = min(1.0, (depth - hp.depth_min) / (hp.depth_full - hp.depth_min))
    vp = hp.speed
    r = min(1.0, max(0.0, (abs(speed) - hp.onset * vp) / ((1.0 - hp.onset) * vp)))
    return 1.0 - (1.0 - hp.residual) * w * r * r


def peak_mu(table: ClassTable, s: Surface, condition: RoadCondition, speed):
    """Peak friction on surface `s`, relative to dry reference ground (1.0 there)."""
    c = table[s.cls]
    wetness = s.wetness or condition.wetness
    level = WETNESS.index(wetness)
    depth = s.water_depth if s.water_depth is not None else table.wetness_depth[wetness]
    if depth > table.wetness_depth[wetness]:
        # Standing water at least as wet as the deepest level it reaches.
        level = max(level, max(i for i, w in enumerate(WETNESS)
                               if table.wetness_depth[w] <= depth))
    mu = c.mu * c.wet[level]
    if c.hydroplaning:
        mu *= hydroplane_factor(table.hydroplaning, speed, depth)
    return condition.friction_scale * mu / table[table.reference].mu


def graph_at(graph, slip):
    """PhysX's friction-vs-slip graph at |slip|: linear between its points, flat past them."""
    s = abs(slip)
    (x0, y0), (x1, y1), (x2, y2) = graph
    if s <= x0:
        return y0
    if s <= x1:
        return y0 + (y1 - y0) * (s - x0) / (x1 - x0)
    if s <= x2:
        return y1 + (y2 - y1) * (s - x1) / (x2 - x1)
    return y2


class WheelSurfaces:
    """One vehicle's wheels on a surface map: what each wheel's friction value should be for
    the next physics step. `tire_friction` and `vehicle_graph` are the rig's."""

    def __init__(self, surface_map: SurfaceMap, tire_friction, vehicle_graph, wheels=4):
        self.map = surface_map
        self.tire_friction = tire_friction
        self.vehicle_graph = tuple(tuple(p) for p in vehicle_graph)
        self.n = wheels
        self.reset()

    def reset(self):
        """Forget the filters (a teleport): the next update snaps to the ground."""
        self.mu = [None] * self.n
        self.surfaces = [None] * self.n

    def update(self, i, surface: Surface, condition: RoadCondition, speed, slip, dt):
        """Wheel `i` on `surface` (None: in the air, kept as it was) at forward `speed`
        (m/s) with longitudinal `slip` after a step of `dt` s: its friction value."""
        if surface is None:
            surface = self.surfaces[i]
            if surface is None:
                return self.tire_friction
        target = peak_mu(self.map.table, surface, condition, speed)
        if self.mu[i] is None:
            self.mu[i] = target
        else:
            k = min(1.0, abs(speed) * dt / max(self.map.table.blend_length, 1e-6))
            self.mu[i] += (target - self.mu[i]) * k
        self.surfaces[i] = surface
        mu = self.tire_friction * self.mu[i]
        graph = self.map.table[surface.cls].slip
        if graph is not None:
            mu *= graph_at(graph, slip) / max(graph_at(self.vehicle_graph, slip), 1e-6)
        return mu


# --- Check and plot ------------------------------------------------------------------------


def lanelet_ways(osm):
    """[(n, 2) array] per way of a Lanelet2 map, from its nodes' local_x / local_y."""
    import xml.etree.ElementTree as ET

    nodes, ways = {}, []
    for ev, el in ET.iterparse(osm, events=("end",)):
        if el.tag == "node":
            tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
            if "local_x" in tags and "local_y" in tags:
                nodes[el.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
            el.clear()
        elif el.tag == "way":
            pts = [nodes[nd.get("ref")] for nd in el.findall("nd") if nd.get("ref") in nodes]
            if len(pts) >= 2:
                ways.append(np.array(pts))
            el.clear()
    return ways


def warnings(surface_map, ways=None):
    """Things a check reports without refusing the map."""
    out = []
    zs = surface_map.zones
    for i, a in enumerate(zs):
        for b in zs[i + 1:]:
            ax0, ax1, ay0, ay1 = a.bbox
            bx0, bx1, by0, by1 = b.bbox
            both = [f for f in ("cls", "wetness", "water_depth")
                    if getattr(a, f) is not None and getattr(b, f) is not None]
            if both and ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1:
                names = ", ".join("class" if f == "cls" else f for f in both)
                out.append(f"zones {a.id} and {b.id} may overlap, both setting {names}: "
                           f"{b.id}'s wins")
    if ways:
        pts = np.concatenate(ways)
        lo, hi = pts.min(axis=0) - 50.0, pts.max(axis=0) + 50.0
        for z in zs:
            x0, x1, y0, y1 = z.bbox
            if x1 < lo[0] or x0 > hi[0] or y1 < lo[1] or y0 > hi[1]:
                out.append(f"zone {z.id} lies outside the Lanelet2 map's extent")
    return out


def plot(surface_map, out, ways=()):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    names = sorted(surface_map.table.classes)
    cmap = plt.get_cmap("tab10" if len(names) <= 10 else "tab20")
    colour = {n: cmap(i % cmap.N) for i, n in enumerate(names)}
    fig, ax = plt.subplots(figsize=(12, 12))
    for w in ways:
        ax.plot(w[:, 0], w[:, 1], color="0.6", lw=0.4)
    for z in surface_map.zones:
        if isinstance(z.water_depth, Raster):
            r = z.water_depth
            ax.imshow(r.values * 1000.0, extent=r.extent, origin="upper", cmap="Blues",
                      alpha=0.7, zorder=2)
        fc = colour[z.cls] if z.cls else (0.3, 0.55, 0.9, 0.25)  # water only: light blue
        ax.add_patch(Polygon(z.polygon, closed=True, fc=fc, ec="k", alpha=0.45 if z.cls else None,
                             hatch="//" if z.soil else None, zorder=3))
        cx, cy = z.polygon.mean(axis=0)
        extra = [z.cls or "", z.wetness or "",
                 "" if z.water_depth is None else
                 ("depth raster" if isinstance(z.water_depth, Raster)
                  else f"{z.water_depth * 1000:.1f} mm")]
        ax.text(cx, cy, "\n".join([z.id] + [e for e in extra if e]), ha="center", va="center",
                fontsize=7, zorder=4)
    handles = [Polygon([[0, 0]], fc=colour[n], alpha=0.45, label=n) for n in names
               if any(z.cls == n for z in surface_map.zones)]
    defaults = ", ".join(f"{k} {v}" for k, v in surface_map.defaults.items())
    ax.set_title(f"{surface_map.path}\nlabel defaults: {defaults}", fontsize=8)
    if handles:
        ax.legend(handles=handles, loc="upper right", fontsize=7)
    ax.set_aspect("equal")
    ax.autoscale()
    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m)")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check or plot a map's surfaces.yaml (047)")
    ap.add_argument("command", choices=("check", "plot"))
    ap.add_argument("map", help="a map directory (its surfaces.yaml) or a surfaces.yaml")
    ap.add_argument("-o", "--output", default="",
                    help="plot: the image (default <map>/surfaces.png)")
    args = ap.parse_args(argv)
    path = find(args.map)
    if not path.exists():
        sys.exit(f"{path}: no surface map")
    try:
        smap = load(path)
    except SurfaceError as e:
        sys.exit(f"{path}: refused\n{e}")
    osm = path.parent / "lanelet2_map.osm"
    ways = lanelet_ways(osm) if osm.exists() else []
    for w in warnings(smap, ways):
        print(f"warning: {w}")
    print(f"{path}: {len(smap.zones)} zone(s), {sum(z.soil for z in smap.zones)} with soil; "
          f"{len(smap.table.classes)} classes")
    if args.command == "plot":
        out = args.output or str(path.parent / "surfaces.png")
        plot(smap, out, ways)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
