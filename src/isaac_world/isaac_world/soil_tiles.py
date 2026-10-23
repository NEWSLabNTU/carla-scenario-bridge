"""A soil zone's own ground on Isaac (047 S3): the ruts of its grid (`soil.SoilGrid`) under the
egos' wheels and in the render mesh, so every ego wheel and the lidar meet what the wheels
made. Needs pxr and aib (import after `SimulationApp`).

A zone with `ground_z` is covered by square tiles of `TILE_BLOCKS` x `TILE_BLOCKS` grid blocks
(1.6 m at 5 cm) at that height over its bounding box: the map must have no ground collider
inside that box (a wheel's suspension ray takes the first hit): `cut_stage` cuts a town's out
at load (`ground_cut`, the geometry). The ruts go to three places, none of which PhysX cooks
again (docs/issues/001: it keeps every mesh it cooks, ~64 KB a 10 cm tile, freed by nothing):

  render     a mesh per tile at the grid's resolution (33 x 33 vertices), no collision, its
             points written in place: what cameras and the RTX lidar see
  flat       one box per zone, its top at `ground_z` over the tiles' box, in a collision group
             the vehicles' suspension rays do not hit: what every other body stands on (a
             tracked NPC, a dropped object), without ruts
  plates     a thin box under each ego wheel in the box (`place`, before every substep): the
             plane fitted to the ruts over the tire's footprint where the wheel will be after
             the substep, so a tire half over a rut sits half in it, tilted toward it (the pull
             of a rut). What the wheel's single suspension ray meets; a box is never cooked,
             so moving it keeps nothing

A tile whose blocks the wheels moved more than `soil.DIRTY` is rebuilt (its render mesh and the
heights the plates read, `shown`) once no wheel on soil is within `margin` of it (3 m: a car's
axles both gone, so one rebuild a pass, not one per axle; ruts appearing under a wheel shook
the car to a stop). At most `budget` tiles a frame, oldest first; the rest wait, while the
soil's forces carry on from the grid.
"""
import logging
import time

import numpy as np
from aib.isaac.vehicle import GROUPS as AIB_GROUPS
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from . import ground_cut, soil

log = logging.getLogger("isaac_world")

ROOT = "/World/soil"
TILE_BLOCKS = 2
# A wheel's plate, m: wide enough for a substep's travel either way (0.33 m at 20 m/s and 60 Hz)
# with its tilt fitted where the ray will land; thin, so it is never a step of its own.
PLATE = 0.8
PLATE_THICK = 0.02
PARK_Z = -1000.0  # a plate no wheel needs: far below everything
# The footprint a plate is fitted over: across the tire's width, and this far along it, m.
FOOTPRINT_ACROSS = (-0.5, -0.25, 0.0, 0.25, 0.5)  # shares of the width
FOOTPRINT_ALONG = (-0.08, 0.0, 0.08)
FOOTPRINT = tuple(g.ravel() for g in np.meshgrid(FOOTPRINT_ALONG, FOOTPRINT_ACROSS,
                                                 indexing="ij"))  # (along m, across share)
# The vehicles' suspension rays query as this group (aib.isaac.vehicle.ensure_vehicle_scene).
GROUND_QUERY = f"{AIB_GROUPS}/groundQuery"
# The ruts the tiles show thin to nothing over this much of the zone's edge, m (047 S6): the
# soft layer feathers out against the rigid ground beside it, as the soil's forces fade in over
# 1.5 wheel diameters (~1.1 m), instead of ending in a rut-deep wall loose soil cannot hold
# (on Town01 a car leaving a 10 cm rut onto the road there drew Autoware's emergency stop).
# The grid keeps the whole ruts.
FEATHER = 1.0
# Sand-coloured, so a camera tells the zone from the road.
COLOR = Gf.Vec3f(0.76, 0.66, 0.48)


def _faces(n):
    """Quad faces of an n x n cell grid over (n + 1)^2 vertices, x first."""
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    a = (i * (n + 1) + j).ravel()
    idx = np.stack([a, a + n + 1, a + n + 2, a + 1], axis=1).ravel()
    return Vt.IntArray.FromNumpy(idx.astype(np.int32)), Vt.IntArray([4] * (n * n))


QCODE_ATTR = "semantics:labels:wikidata_qcode"  # as surface_layer's
# A mesh reaching this far below or above the zone's ground in its box is ground: cut.
CUT_BELOW = soil.MAX_SINKAGE + 0.5
CUT_ABOVE = 0.05


def _flat(pv):
    """A primvar's values, per element of its interpolation (indices applied)."""
    v = pv.ComputeFlattened()
    return None if v is None else np.array(v)


def _label(prim):
    """The SimReady label (Q-codes) on `prim` or its nearest ancestor that has one."""
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        attr = prim.GetAttribute(QCODE_ATTR)
        if attr and attr.HasAuthoredValue():
            return attr.Get()
        prim = prim.GetParent()
    return None


def cut_stage(stage, root, box, z_lo, z_hi, skip=(), copy_root=None):
    """Cut `box` (stage frame x0, x1, y0, y1) out of every mesh under `root` that reaches into
    it between heights `z_lo` and `z_hi`, face by face: only faces lying wholly in that band
    are ground (a road, a bridge's deck), not a guardrail's upright faces or a lamp post's.
    The cut geometry
    goes over the prim's in the stage's edit target, materials and subsets with it: what
    cameras and the lidar see. PhysX does not re-cook a mesh whose points change, so a
    colliding prim's collision is disabled and the cut collision is a new invisible mesh under
    `copy_root`, in the stage frame, wearing the prim's SimReady label (the surface layer's
    classes). {mesh path: (faces before, faces after, attributes written)}."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    x0, x1, y0, y1 = box
    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.IsA(UsdGeom.Mesh) or any(str(prim.GetPath()).startswith(k) for k in skip):
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        lo, hi = r.GetMin(), r.GetMax()
        if hi[0] <= x0 or lo[0] >= x1 or hi[1] <= y0 or lo[1] >= y1 or hi[2] < z_lo \
                or lo[2] > z_hi:
            continue
        mesh = UsdGeom.Mesh(prim)
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        local = mesh.GetPointsAttr().Get()
        if not counts or local is None:
            continue
        m = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        pts = np.array(local, dtype=np.float64)
        world = np.c_[pts, np.ones(len(pts))] @ m
        groups = {"vertex": {}, "faceVarying": {}, "uniform": {}}
        attrs = {}
        for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
            interp = pv.GetInterpolation()
            interp = "vertex" if interp == "varying" else interp
            if interp not in groups or not pv.HasValue():
                continue
            v = _flat(pv)
            if v is None or v.dtype.kind not in "fiu":
                continue
            groups[interp][pv.GetPrimvarName()] = v
            attrs[pv.GetPrimvarName()] = pv
        normals = mesh.GetNormalsAttr()
        if normals.HasValue():
            interp = mesh.GetNormalsInterpolation()
            interp = "vertex" if interp == "varying" else interp
            if interp in groups:
                groups[interp]["__normals"] = np.array(normals.Get())
        cut = ground_cut.cut_mesh(world[:, :3], counts, indices, box, groups["vertex"],
                                  groups["faceVarying"], groups["uniform"], (z_lo, z_hi))
        if cut is None:
            continue
        new_pts, new_counts, new_idx, vx, fv, uni, fmap = cut
        if len(new_counts) == 0:
            # wholly inside (a lane marking): hidden, no collision; a mesh with no faces and a
            # CollisionAPI is a PhysX error ("does not have indices")
            vis = UsdGeom.Imageable(prim).GetVisibilityAttr()
            vis.Set(UsdGeom.Tokens.invisible)
            written = [vis]
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                enabled = UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr()
                enabled.Set(False)
                written.append(enabled)
            out[str(prim.GetPath())] = (len(counts), 0, written)
            continue
        back = np.c_[new_pts, np.ones(len(new_pts))] @ np.linalg.inv(m)
        written = [mesh.GetPointsAttr(), mesh.GetFaceVertexCountsAttr(),
                   mesh.GetFaceVertexIndicesAttr(), mesh.GetExtentAttr()]
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(back[:, :3].astype(np.float32)))
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(new_counts))
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(new_idx))
        for values in (vx, fv, uni):
            for name, v in values.items():
                attr = normals if name == "__normals" else attrs[name].GetAttr()
                if name != "__normals" and attrs[name].IsIndexed():
                    attrs[name].BlockIndices()
                    written.append(attrs[name].GetIndicesAttr())
                cls = attr.GetTypeName().type.pythonClass
                attr.Set(cls.FromNumpy(np.ascontiguousarray(v)))
                written.append(attr)
        for subset in UsdGeom.Subset.GetAllGeomSubsets(mesh):
            subset.GetIndicesAttr().Set(Vt.IntArray.FromNumpy(
                ground_cut.remap_subset(subset.GetIndicesAttr().Get(), fmap)))
            written.append(subset.GetIndicesAttr())
        mesh.GetExtentAttr().Set(UsdGeom.PointBased.ComputeExtent(
            Vt.Vec3fArray.FromNumpy(back[:, :3].astype(np.float32))))
        if prim.HasAPI(UsdPhysics.CollisionAPI) and copy_root is not None:
            enabled = UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr()
            enabled.Set(False)
            written.append(enabled)
            copy = UsdGeom.Mesh.Define(stage, f"{copy_root}/m{len(out)}")
            copy.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(new_pts.astype(np.float32)))
            copy.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(new_counts))
            copy.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(new_idx))
            copy.MakeInvisible()
            UsdPhysics.CollisionAPI.Apply(copy.GetPrim())
            approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() \
                if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None
            UsdPhysics.MeshCollisionAPI.Apply(copy.GetPrim()).CreateApproximationAttr(
                approx or "none")
            label = _label(prim)
            if label:
                copy.GetPrim().CreateAttribute(QCODE_ATTR, Sdf.ValueTypeNames.TokenArray).Set(label)
        out[str(prim.GetPath())] = (len(counts), len(new_counts), written)
    return out


def uncut(cut):
    """Undo `cut_stage`: the opinions it wrote cleared, so each mesh is the map's again."""
    for _, _, written in cut.values():
        for attr in written:
            if attr:
                attr.Clear()


class SoilTiles:
    def __init__(self, stage, zone, spacing, origin=(0.0, 0.0, 0.0), budget=4, cut_root=None):
        """Tiles over `zone`'s bounding box (map frame; the stage is the map minus `origin`).
        `cut_root`: the map's ground under it, cut out of the tiles' box (`cut_stage`)."""
        self.stage = stage
        self.zone = zone
        self.h = spacing
        self.n = TILE_BLOCKS * soil.BLOCK  # grid cells per tile side
        self.origin = origin
        self.budget = budget
        self.root = f"{ROOT}/{zone.id}"
        self.gen = {}  # (tx, ty) -> rebuilds so far
        self.shown = {}  # (tx, ty) -> the ruts its render mesh and the plates show, (n+1)^2 m
        self.plates = {}  # wheel key -> (translate, orient, parked)
        self.full = set()  # tiles whose render mesh is at full resolution
        self.queue = []  # tiles waiting for a rebuild, oldest first
        self.feathers = {}  # (tx, ty) -> its vertices' share of the ruts (FEATHER)
        self.faces = {n: _faces(n) for n in (1, self.n)}
        x0, x1, y0, y1 = zone.bbox
        side = self.n * spacing
        # 145.6 / 1.6 is 90.99999...: a polygon on whole tiles must not gain a row of them
        lo = lambda v: int(np.floor(v / side + 1e-6))  # noqa: E731
        hi = lambda v: int(np.ceil(v / side - 1e-6))  # noqa: E731
        self.tiles = [(tx, ty) for tx in range(lo(x0), hi(x1)) for ty in range(lo(y0), hi(y1))]
        self.tiles_set = set(self.tiles)
        UsdGeom.Scope.Define(stage, self.root)
        ox, oy, _ = origin
        txs, tys = [t[0] for t in self.tiles], [t[1] for t in self.tiles]
        self.box = (min(txs) * side - ox, (max(txs) + 1) * side - ox,
                    min(tys) * side - oy, (max(tys) + 1) * side - oy)  # stage frame
        self.cut = {}
        if cut_root is not None:
            t0 = time.perf_counter()
            self.cut = cut_stage(stage, cut_root, self.box, zone.ground_z - CUT_BELOW,
                                 zone.ground_z + CUT_ABOVE, skip=(ROOT,),
                                 copy_root=f"{self.root}/cut")
            log.info("soil zone %s: cut (%.1f, %.1f, %.1f, %.1f) out of %d mesh(es) in %.2f s: "
                     "%s",
                     zone.id, *self.box, len(self.cut), time.perf_counter() - t0,
                     ", ".join(f"{p.rsplit('/', 1)[-1]} {a}->{b}"
                               for p, (a, b, _) in self.cut.items()))
        flat = np.zeros((2, 2), np.float32)
        for t in self.tiles:
            self._render(t, flat, 1)
        self._flat_ground()
        log.info("soil zone %s: %d tiles of %.1f m at z %.2f", zone.id, len(self.tiles), side,
                 zone.ground_z)

    def _name(self, t):
        return f"{t[0]}_{t[1]}".replace("-", "m")

    def _points(self, t, depth, n):
        """(n+1)^2 vertices of tile `t` sunk by `depth` (m), stage frame."""
        ox, oy, _ = self.origin
        side = self.n * self.h
        xs = t[0] * side + np.linspace(0.0, side, n + 1) - ox
        ys = t[1] * side + np.linspace(0.0, side, n + 1) - oy
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        return Vt.Vec3fArray.FromNumpy(np.stack(
            [gx.ravel(), gy.ravel(), self.zone.ground_z - depth.ravel()], axis=1)
            .astype(np.float32))

    def _render(self, t, depth, n):
        path = f"{self.root}/r{self._name(t)}"
        mesh = UsdGeom.Mesh(self.stage.GetPrimAtPath(path))
        if not mesh:
            mesh = UsdGeom.Mesh.Define(self.stage, path)
            mesh.CreateDisplayColorAttr([COLOR])
        if (n == self.n) != (t in self.full) or not mesh.GetFaceVertexCountsAttr().HasValue():
            idx, cnt = self.faces[n]
            mesh.CreateFaceVertexIndicesAttr().Set(idx)
            mesh.CreateFaceVertexCountsAttr().Set(cnt)
            if n == self.n:
                self.full.add(t)
            else:
                self.full.discard(t)
        mesh.CreatePointsAttr().Set(self._points(t, depth, n))

    def _flat_ground(self):
        """The zone's ground for everything but the suspension rays: a box over the tiles."""
        group = UsdPhysics.CollisionGroup.Define(self.stage, f"{self.root}/flatGroup")
        group.CreateFilteredGroupsRel().AddTarget(GROUND_QUERY)
        x0, x1, y0, y1 = self.box
        cube = UsdGeom.Cube.Define(self.stage, f"{self.root}/flat")
        cube.CreateSizeAttr(1.0)
        cube.MakeInvisible()
        xf = UsdGeom.XformCommonAPI(cube.GetPrim())
        xf.SetTranslate(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, self.zone.ground_z - 0.5))
        xf.SetScale(Gf.Vec3f(x1 - x0, y1 - y0, 1.0))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        group.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(cube.GetPath())

    def _plate(self, key):
        p = self.plates.get(key)
        if p is None:
            name = "_".join(str(k) for k in key).strip("/").replace("/", "_").replace("-", "m")
            cube = UsdGeom.Cube.Define(self.stage, f"{self.root}/plates/{name}")
            cube.CreateSizeAttr(1.0)
            cube.MakeInvisible()
            prim = cube.GetPrim()
            translate = cube.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
            orient = cube.AddOrientOp()
            cube.AddScaleOp().Set(Gf.Vec3f(PLATE, PLATE, PLATE_THICK))
            translate.Set(Gf.Vec3d(0.0, 0.0, PARK_Z))
            UsdPhysics.CollisionAPI.Apply(prim)
            p = self.plates[key] = [translate, orient, True]
        return p

    def depths(self, xs, ys):
        """The ruts shown at map-frame points (arrays), m below `ground_z`, bilinear; NaN off
        the tiles."""
        side = self.n * self.h
        tx, ty = np.floor(xs / side).astype(int), np.floor(ys / side).astype(int)
        out = np.full(len(xs), np.nan)
        for t in set(zip(tx.tolist(), ty.tolist())):
            if t not in self.tiles_set:
                continue
            m = (tx == t[0]) & (ty == t[1])
            depth = self.shown.get(t)
            if depth is None:
                out[m] = 0.0
                continue
            u, v = xs[m] / self.h - t[0] * self.n, ys[m] / self.h - t[1] * self.n
            i = np.minimum(u.astype(int), self.n - 1)
            j = np.minimum(v.astype(int), self.n - 1)
            fu, fv = u - i, v - j
            out[m] = (depth[i, j] * (1 - fu) * (1 - fv) + depth[i + 1, j] * fu * (1 - fv)
                      + depth[i, j + 1] * (1 - fu) * fv + depth[i + 1, j + 1] * fu * fv)
        return out

    def footprint(self, x, y, hx, hy, width):
        """The plane of the ruts under a tire `width` m wide at map-frame (x, y) heading
        (hx, hy): (z at the centre, dz/dx, dz/dy), stage-frame z; None if its centre is more
        than half a plate outside the tiles' box. Points off the tiles count as flat ground:
        a wheel leaving the zone keeps its plate until it is clear of the box, or its ray,
        landing a few cm from where the plate's centre was predicted, finds the box's hole
        and nothing in it (a substep in the air per axle at the exit, 1.1 cm of heave, and
        NDT at its iteration limit on Town01: issue 002). The footprint is a grid symmetric
        about its centre, so the least-squares plane is the mean and two independent
        slopes."""
        ox, oy, _ = self.origin
        x0, x1, y0, y1 = self.box
        m = PLATE / 2
        if not (x0 + ox - m <= x <= x1 + ox + m and y0 + oy - m <= y <= y1 + oy + m):
            return None
        a, c = FOOTPRINT[0], FOOTPRINT[1] * width
        z = np.nan_to_num(-self.depths(x + hx * a - hy * c, y + hy * a + hx * c))
        along = float(a @ z / (a @ a))
        across = float(c @ z / (c @ c))
        return (self.zone.ground_z + float(z.mean()), along * hx - across * hy,
                along * hy + across * hx)

    def place(self, wheels):
        """Before a substep: each wheel's plate under where it will be (`wheels`: key ->
        map-frame (x, y, heading x, heading y, tire width)), a wheel off the tiles' plate
        parked."""
        ox, oy, _ = self.origin
        for key, (x, y, hx, hy, width) in wheels.items():
            g = self.footprint(x, y, hx, hy, width)
            if g is None and key not in self.plates:
                continue
            p = self._plate(key)
            if g is None:
                if not p[2]:
                    p[0].Set(Gf.Vec3d(0.0, 0.0, PARK_Z))
                    p[2] = True
                continue
            z, gx, gy = g
            n = Gf.Vec3d(-gx, -gy, 1.0).GetNormalized()
            centre = Gf.Vec3d(x - ox, y - oy, z) - n * (PLATE_THICK / 2)
            p[0].Set(centre)
            p[1].Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), n).GetQuat()))
            p[2] = False
        for key in set(self.plates) - set(wheels):  # a vehicle gone
            p = self.plates[key]
            if not p[2]:
                p[0].Set(Gf.Vec3d(0.0, 0.0, PARK_Z))
                p[2] = True

    def _feather(self, t):
        """Tile `t`'s vertices' share of the grid's ruts ((n+1)^2): `FEATHER` at the edge."""
        w = self.feathers.get(t)
        if w is None:
            side = self.n * self.h
            w = self.feathers[t] = soil.edge_weights(
                self.zone.polygon, t[0] * side + np.linspace(0.0, side, self.n + 1),
                t[1] * side + np.linspace(0.0, side, self.n + 1), FEATHER).astype(np.float32)
        return w

    def _rebuild(self, t, depth):
        """Tile `t` at the grid's `depth` ((n+1)^2): its render mesh and the plates' heights."""
        depth = depth * self._feather(t)
        self._render(t, depth, self.n)
        self.shown[t] = depth
        self.gen[t] = self.gen.get(t, 0) + 1

    def update(self, grid, wheels=(), margin=3.0):
        """Queue the tiles over the grid's dirty blocks, rebuild up to `budget` of them that no
        wheel is near (`wheels`: map-frame (x, y) of every wheel on soil, kept `margin` m
        away); the tiles rebuilt. A wheel's own pass is the grid's forces' to carry; its rut is
        felt from the next pass on, by it and every other vehicle."""
        per = TILE_BLOCKS
        outside = []
        for bx, by in grid.dirty_blocks():
            t = (bx // per, by // per)
            if t not in self.tiles_set:
                outside.append((bx, by))  # past the zone's tiles: the forces only
            elif t not in self.queue:
                self.queue.append(t)
        grid.handed_out(outside)
        side = self.n * self.h
        busy = {(tx, ty) for x, y in wheels
                for tx in range(int(np.floor((x - margin) / side)),
                                int(np.floor((x + margin) / side)) + 1)
                for ty in range(int(np.floor((y - margin) / side)),
                                int(np.floor((y + margin) / side)) + 1)}
        done, waiting = [], []
        while self.queue and len(done) < self.budget:
            t = self.queue.pop(0)
            if t in busy:
                waiting.append(t)
                continue
            self._rebuild(t, grid.heights(t[0] * self.n, t[1] * self.n, self.n))
            grid.handed_out([(t[0] * per + i, t[1] * per + j)
                             for i in range(per) for j in range(per)])
            done.append(t)
        self.queue = waiting + self.queue
        return done

    def show(self, grid):
        """Every tile at the grid's ruts now, unbudgeted (a rut set going in)."""
        for t in self.tiles:
            depth = grid.heights(t[0] * self.n, t[1] * self.n, self.n)
            if depth.max() > 0 or t in self.shown:
                self._rebuild(t, depth)
        self.queue = []
        grid.handed_out(list(grid.blocks))

    def clear(self):
        """Back to flat ground (fresh soil)."""
        flat = np.zeros((2, 2), np.float32)
        for t in list(self.shown):
            self._render(t, flat, 1)
            self.gen[t] += 1
        self.shown = {}
        self.queue = []

    def remove(self):
        """The tiles off the stage and the map's ground back under them."""
        self.stage.RemovePrim(self.root)
        self.plates = {}
        uncut(self.cut)
        self.cut = {}
