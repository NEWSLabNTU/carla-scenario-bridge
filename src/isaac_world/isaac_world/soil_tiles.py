"""A soil zone's own ground on Isaac (047 S3): the ruts of its grid (`soil.SoilGrid`) in the
collision and the render mesh, so every wheel and the lidar meet what the wheels made. Needs
pxr (import after `SimulationApp`).

A zone with `ground_z` is covered by square tiles of `TILE_BLOCKS` x `TILE_BLOCKS` grid blocks
(1.6 m at 5 cm) at that height over its bounding box, made at load as flat two-triangle quads:
the map must have no ground collider inside that box (a wheel's suspension ray takes the
highest hit): `cut_stage` cuts a town's out at load (`ground_cut`, the geometry). Each tile
is two meshes:

  render     at the grid's resolution (33 x 33 vertices), no collision, one prim whose points
             are written in place: what cameras and the RTX lidar see
  collision  every `COLLISION_STEP`-th node (17 x 17 at 10 cm), invisible. PhysX cooks a mesh
             once and does not see its points change (047 spike), so a rebuild is a new prim,
             made before the old one is removed: the wheel never stands on nothing

A tile whose blocks the wheels moved more than `soil.DIRTY` is rebuilt once no wheel on soil
is within `margin` of it (3 m: a car's axles both gone, so one rebuild a pass, not one per
axle; rebuilding under a wheel shook the car to a stop). At most `budget` tiles a frame, oldest
first; the rest wait, while the soil's forces carry on from the grid.

Memory (docs/issues/001): PhysX keeps every mesh it has cooked, released or not, about 465 KB
for a 5 cm tile and 64 KB for a 10 cm one (`spikes/tile_memory.py`); the render mesh's edits
keep none. Hence the coarser collision and one rebuild a pass.
"""
import logging
import time

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from . import ground_cut, soil

log = logging.getLogger("isaac_world")

ROOT = "/World/soil"
TILE_BLOCKS = 2
COLLISION_STEP = 2  # collision vertices every this many grid nodes
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
        self.gen = {}  # (tx, ty) -> collision rebuilds so far (the prim's name)
        self.full = set()  # tiles whose render mesh is at full resolution
        self.queue = []  # tiles waiting for a rebuild, oldest first
        self.faces = {n: _faces(n) for n in (1, self.n, self.n // COLLISION_STEP)}
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
            self._collision(t, flat, 1)
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

    def _collision_path(self, t):
        return f"{self.root}/c{self._name(t)}_{self.gen.get(t, 0)}"

    def _collision(self, t, depth, n):
        mesh = UsdGeom.Mesh.Define(self.stage, self._collision_path(t))
        mesh.CreatePointsAttr(self._points(t, depth, n))
        idx, cnt = self.faces[n]
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateFaceVertexCountsAttr(cnt)
        mesh.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")

    def _rebuild(self, t, depth):
        """Tile `t` at the grid's `depth` ((n+1)^2): render in place, collision anew."""
        self._render(t, depth, self.n)
        old = self._collision_path(t)
        self.gen[t] = self.gen.get(t, 0) + 1
        self._collision(t, depth[::COLLISION_STEP, ::COLLISION_STEP], self.n // COLLISION_STEP)
        self.stage.RemovePrim(old)

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

    def clear(self):
        """Back to flat ground (fresh soil)."""
        flat = np.zeros((2, 2), np.float32)
        for t in self.tiles:
            if self.gen.get(t):
                self._render(t, flat, 1)
                old = self._collision_path(t)
                self.gen[t] += 1
                self._collision(t, flat, 1)
                self.stage.RemovePrim(old)
        self.queue = []

    def remove(self):
        """The tiles off the stage and the map's ground back under them."""
        self.stage.RemovePrim(self.root)
        uncut(self.cut)
        self.cut = {}
