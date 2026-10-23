"""A soil zone's own ground on Isaac (047 S3): the ruts of its grid (`soil.SoilGrid`) in the
collision and the render mesh, so every wheel and the lidar meet what the wheels made. Needs
pxr (import after `SimulationApp`).

A zone with `ground_z` is covered by square tiles of `TILE_BLOCKS` x `TILE_BLOCKS` grid blocks
(1.6 m at 5 cm) at that height over its bounding box, made at load as flat two-triangle quads:
the map must have no ground collider inside that box (a wheel's suspension ray takes the
highest hit). A tile
whose blocks the wheels moved more than `soil.DIRTY` is rebuilt at full resolution once no
wheel stands on it, its vertex
heights the grid's nodes (33 x 33, shared with its neighbours' edges). PhysX cooks a mesh's
collision once and does not see its points change (047 spike), so a rebuilt tile is a new prim,
made before the old one is removed: the wheel never stands on nothing. At most `budget` tiles
a frame, oldest first; the rest wait, while the soil's forces carry on from the grid.
"""
import logging

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, Vt

from . import soil

log = logging.getLogger("isaac_world")

ROOT = "/World/soil"
TILE_BLOCKS = 2
# Sand-coloured, so a camera tells the zone from the road.
COLOR = Gf.Vec3f(0.76, 0.66, 0.48)


def _faces(n):
    """Quad faces of an n x n cell grid over (n + 1)^2 vertices, x first."""
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    a = (i * (n + 1) + j).ravel()
    idx = np.stack([a, a + n + 1, a + n + 2, a + 1], axis=1).ravel()
    return Vt.IntArray.FromNumpy(idx.astype(np.int32)), Vt.IntArray([4] * (n * n))


class SoilTiles:
    def __init__(self, stage, zone, spacing, origin=(0.0, 0.0, 0.0), budget=4):
        """Tiles over `zone`'s bounding box (map frame; the stage is the map minus `origin`)."""
        self.stage = stage
        self.zone = zone
        self.h = spacing
        self.n = TILE_BLOCKS * soil.BLOCK  # cells per tile side
        self.origin = origin
        self.budget = budget
        self.root = f"{ROOT}/{zone.id}"
        self.gen = {}  # (tx, ty) -> rebuilds so far (the prim's name)
        self.queue = []  # tiles waiting for a rebuild, oldest first
        self.full_faces = _faces(self.n)
        x0, x1, y0, y1 = zone.bbox
        side = self.n * spacing
        self.tiles = [(tx, ty) for tx in range(int(np.floor(x0 / side)), int(np.ceil(x1 / side)))
                      for ty in range(int(np.floor(y0 / side)), int(np.ceil(y1 / side)))]
        self.tiles_set = set(self.tiles)
        UsdGeom.Scope.Define(stage, self.root)
        flat = np.zeros((2, 2), np.float32)
        for t in self.tiles:
            self._define(t, flat, 1)
        log.info("soil zone %s: %d tiles of %.1f m at z %.2f", zone.id, len(self.tiles), side,
                 zone.ground_z)

    def _path(self, t):
        return f"{self.root}/t{t[0]}_{t[1]}_{self.gen.get(t, 0)}".replace("-", "m")

    def _define(self, t, depth, n):
        """Tile `t` as a mesh of n x n cells whose vertices sink by `depth` ((n+1)^2, m)."""
        ox, oy, _ = self.origin
        side = self.n * self.h
        xs = t[0] * side + np.linspace(0.0, side, n + 1) - ox
        ys = t[1] * side + np.linspace(0.0, side, n + 1) - oy
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        pts = np.stack([gx.ravel(), gy.ravel(), self.zone.ground_z - depth.ravel()],
                       axis=1).astype(np.float32)
        mesh = UsdGeom.Mesh.Define(self.stage, self._path(t))
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        if n == self.n:
            idx, cnt = self.full_faces
        else:
            idx, cnt = _faces(n)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateFaceVertexCountsAttr(cnt)
        mesh.CreateDisplayColorAttr([COLOR])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
        return mesh

    def update(self, grid, wheels=(), margin=0.5):
        """Queue the tiles over the grid's dirty blocks, rebuild up to `budget` of them that no
        wheel stands on (`wheels`: map-frame (x, y) of every wheel on soil, kept off by
        `margin` m); the tiles rebuilt. A tile under a wheel waits for it to leave: the ground
        dropping under a wheel every frame shook the car to a stop (47 S3, pad), and the
        wheel's own pass is the grid's forces' to carry; its rut is felt from the next pass on,
        by it and every other vehicle."""
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
            old = self._path(t)
            self.gen[t] = self.gen.get(t, 0) + 1
            self._define(t, grid.heights(t[0] * self.n, t[1] * self.n, self.n), self.n)
            self.stage.RemovePrim(old)
            grid.handed_out([(t[0] * per + i, t[1] * per + j)
                             for i in range(per) for j in range(per)])
            done.append(t)
        self.queue = waiting + self.queue
        return done

    def clear(self):
        """Back to flat ground (fresh soil)."""
        for t in self.tiles:
            if self.gen.get(t):
                old = self._path(t)
                self.gen[t] += 1
                self._define(t, np.zeros((2, 2), np.float32), 1)
                self.stage.RemovePrim(old)
        self.queue = []

    def remove(self):
        self.stage.RemovePrim(self.root)
