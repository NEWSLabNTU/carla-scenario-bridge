"""An Isaac test world from a map's point cloud (roadmap 038): `python -m isaac_world.pcd_world`.

Autoware localizes by NDT against the map's `pointcloud_map.pcd`, so the world its lidar sees
in Isaac has to be the same shape. This builds one from the point cloud itself: every
occupied voxel becomes a cube (one `PointInstancer`, rendered, so the RTX lidar sees it; no
colliders), and a flat collision ground at the road level carries the vehicles. Voxels
within `--ground-band` of the ground are left out, since the ground plane stands in for them.

Writes `<map dir>/isaac/world.usd` and `<map dir>/isaac/world.yaml` (037's world file), so
`IsaacBackend` loads it for that map. The map frame is the world frame (`map_origin` 0).

    python -m isaac_world.pcd_world data/carla-autoware-bridge/Town01 [--voxel 0.4]
"""
import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

GROUND = "/Env/ground"


def lzf_decompress(data, size):
    """LZF (liblzf), as PCL writes `binary_compressed`."""
    out = bytearray(size)
    i = o = 0
    n = len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:  # literal run of ctrl + 1 bytes
            ln = ctrl + 1
            out[o:o + ln] = data[i:i + ln]
            i += ln
            o += ln
        else:  # back reference
            ln = ctrl >> 5
            ref = o - ((ctrl & 0x1F) << 8) - 1
            if ln == 7:
                ln += data[i]
                i += 1
            ref -= data[i]
            i += 1
            ln += 2
            if ref + ln <= o:
                out[o:o + ln] = out[ref:ref + ln]
                o += ln
            else:  # overlapping copy, byte by byte
                for _ in range(ln):
                    out[o] = out[ref]
                    o += 1
                    ref += 1
    if o != size:
        raise ValueError(f"LZF: {o} bytes out, header says {size}")
    return bytes(out)


def read_pcd(path):
    """The x, y, z columns of a PCD file (ascii, binary or binary_compressed), float64 N x 3."""
    raw = Path(path).read_bytes()
    header = {}
    pos = 0
    while True:
        end = raw.index(b"\n", pos)
        line = raw[pos:end].decode("ascii").strip()
        pos = end + 1
        if not line or line.startswith("#"):
            continue
        key, *vals = line.split()
        header[key.upper()] = vals
        if key.upper() == "DATA":
            break
    fields = header["FIELDS"]
    sizes = [int(s) for s in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(c) for c in header.get("COUNT", ["1"] * len(fields))]
    points = int(header["POINTS"][0])
    kind = header["DATA"][0]
    dtype = np.dtype([(f, {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2",
                           ("U", 4): "<u4", ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}[(t, s)],
                       (c,) if c > 1 else ())
                      for f, s, t, c in zip(fields, sizes, types, counts)])
    if kind == "ascii":
        table = np.loadtxt(raw[pos:].decode("ascii").splitlines(), ndmin=2)
        cols = {f: table[:, i] for i, f in enumerate(fields)}
        return np.stack([cols["x"], cols["y"], cols["z"]], axis=1).astype(np.float64)
    if kind == "binary":
        arr = np.frombuffer(raw[pos:pos + points * dtype.itemsize], dtype=dtype)
    elif kind == "binary_compressed":
        csize, usize = struct.unpack("<II", raw[pos:pos + 8])
        data = lzf_decompress(raw[pos + 8:pos + 8 + csize], usize)
        # Column-major: every point's first field, then every point's second, ...
        arr = np.empty(points, dtype=dtype)
        off = 0
        for f in fields:
            ft = dtype.fields[f][0]
            arr[f] = np.frombuffer(data, dtype=ft, count=points, offset=off)
            off += ft.itemsize * points
    else:
        raise ValueError(f"PCD DATA {kind} not supported")
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)


def ground_level(points, cell=2.0):
    """The road level: the median over 2 m cells of each cell's lowest point."""
    keys = np.floor(points[:, :2] / cell).astype(np.int64)
    order = np.lexsort((points[:, 2], keys[:, 1], keys[:, 0]))
    k = keys[order]
    first = np.ones(len(k), dtype=bool)
    first[1:] = np.any(k[1:] != k[:-1], axis=1)
    return float(np.median(points[order][first, 2]))


def voxels(points, size):
    keys = np.unique(np.floor(points / size).astype(np.int64), axis=0)
    return (keys + 0.5) * size


def write_world(map_dir, centers, size, ground_z, extent):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    out = Path(map_dir) / "isaac"
    out.mkdir(exist_ok=True)
    usd = out / "world.usd"
    stage = Usd.Stage.CreateNew(str(usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Env")
    stage.SetDefaultPrim(root.GetPrim())

    (x0, y0), (x1, y1) = extent
    ground = UsdGeom.Mesh.Define(stage, GROUND)
    m = 50.0  # margin around the map
    ground.CreatePointsAttr([Gf.Vec3f(x0 - m, y0 - m, ground_z), Gf.Vec3f(x1 + m, y0 - m, ground_z),
                             Gf.Vec3f(x1 + m, y1 + m, ground_z), Gf.Vec3f(x0 - m, y1 + m, ground_z)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.22, 0.24)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    inst = UsdGeom.PointInstancer.Define(stage, "/Env/voxels")
    cube = UsdGeom.Cube.Define(stage, "/Env/voxels/cube")
    cube.CreateSizeAttr(size)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.60, 0.55)])
    inst.CreatePrototypesRel().SetTargets([cube.GetPath()])
    inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(centers.astype(np.float32)))
    inst.CreateProtoIndicesAttr(Vt.IntArray.FromNumpy(np.zeros(len(centers), dtype=np.int32)))
    stage.GetRootLayer().Save()

    (out / "world.yaml").write_text(
        "# Generated by isaac_world.pcd_world from pointcloud_map.pcd (roadmap 038): the map's\n"
        f"# point cloud as {size} m voxels, and a flat ground at z = {ground_z:.3f}.\n"
        f"usd: isaac/world.usd\nground_paths: [\"/World/env/ground\"]\nmap_origin: [0.0, 0.0, 0.0]\n")
    return usd


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("map_dir")
    ap.add_argument("--voxel", type=float, default=0.4, help="cube edge, m")
    ap.add_argument("--ground-band", type=float, default=0.25,
                    help="voxels whose centre is this close to the ground are dropped, m")
    args = ap.parse_args(argv)
    t0 = time.monotonic()
    pts = read_pcd(Path(args.map_dir) / "pointcloud_map.pcd")
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    g = ground_level(pts)
    c = voxels(pts, args.voxel)
    c = c[c[:, 2] - g > args.ground_band]
    extent = (pts[:, :2].min(axis=0), pts[:, :2].max(axis=0))
    usd = write_world(args.map_dir, c, args.voxel, g, extent)
    print(f"{len(pts)} points -> {len(c)} voxels of {args.voxel} m, ground z {g:.3f}; "
          f"wrote {usd} in {time.monotonic() - t0:.0f} s")


if __name__ == "__main__":
    sys.exit(main())
