#!/usr/bin/env python3
"""Align an Autoware route PCD (MGRS-local) to the re-tiled vendor cloud (roadmap 019 step 0).

Loads the route's pointcloud_map.pcd, picks up to --tiles vendor tiles that the route
covers most (spread over the route), voxel-thins them, and solves:
  1. translation-only ICP (what the survey reported),
  2. rigid 6-DoF ICP (Kabsch, trimmed),
  3. translation-only ICP per tile, to show whether the offset varies across the site.
Output is a JSON report; T maps route PCD -> vendor tiles (vendor = R @ pcd + t).

Usage: align_route.py ROUTE.pcd TILES_DIR --out report.json [--tiles 16]
"""
import argparse
import glob
import json
import os
from multiprocessing import Pool

import laspy
import numpy as np
from scipy.spatial import cKDTree

TILE = 50.0


def read_pcd(path):
    raw = open(path, "rb").read()
    i = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    hdr = raw[:i].decode(errors="replace")
    fields = hdr.split("FIELDS")[1].split("\n")[0].split()
    a = np.frombuffer(raw[i:], dtype=np.float32).reshape(-1, len(fields))
    return a[:, :3].astype(np.float64)


def voxel(p, s):
    k = np.floor(p / s).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return p[idx]


def load_tile(path, vox):
    out = []
    with laspy.open(path) as r:
        for ch in r.chunk_iterator(20_000_000):
            out.append(voxel(np.c_[ch.x, ch.y, ch.z], vox))
    return voxel(np.concatenate(out), vox)


def icp_translation(src, tree, dst, t0, iters=40):
    t = t0.copy()
    for _ in range(iters):
        d, idx = tree.query(src + t)
        m = d < max(0.5, np.percentile(d, 50))
        t += np.median(dst[idx[m]] - (src[m] + t), axis=0)
    d, _ = tree.query(src + t)
    return t, d


def icp_rigid(src, tree, dst, t0, iters=40):
    R = np.eye(3)
    t = t0.copy()
    for _ in range(iters):
        cur = src @ R.T + t
        d, idx = tree.query(cur)
        m = d < max(0.3, np.percentile(d, 50))
        a, b = src[m], dst[idx[m]]
        ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        t = cb - R @ ca
    d, _ = tree.query(src @ R.T + t)
    return R, t, d


def stats(d):
    return {"median_m": round(float(np.median(d)), 4), "p90_m": round(float(np.percentile(d, 90)), 4),
            "within_0.2m": round(float((d < 0.2).mean()), 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("tiles_dir")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tiles", type=int, default=16)
    ap.add_argument("--vox", type=float, default=0.1)
    ap.add_argument("--sample", type=int, default=150_000)
    ap.add_argument("--t0", default="-0.29,0.22,17.26", help="initial PCD->vendor shift")
    a = ap.parse_args()
    rng = np.random.default_rng(0)

    pcd = read_pcd(a.pcd)
    t0 = np.array([float(v) for v in a.t0.split(",")])
    have = {tuple(int(v) for v in os.path.basename(p)[5:-4].split("_")): p
            for p in glob.glob(os.path.join(a.tiles_dir, "tile_*.laz"))}
    key = np.floor((pcd[:, :2] + t0[:2]) / TILE).astype(np.int64)
    uniq, cnt = np.unique(key, axis=0, return_counts=True)
    cand = [(tuple(k), c) for k, c in zip(uniq, cnt) if tuple(k) in have and c > 2000]
    cand.sort(key=lambda kc: -kc[1])
    # spread: take every n-th of the covered tiles, ordered by count
    pick = cand[:: max(1, len(cand) // a.tiles)][: a.tiles]
    rep = {"pcd": os.path.abspath(a.pcd), "pcd_points": int(len(pcd)),
           "covered_tiles": len(cand), "used_tiles": [list(k) for k, _ in pick]}
    print(f"route covers {len(cand)} tiles; using {len(pick)}", flush=True)

    with Pool(min(6, len(pick))) as pool:  # ~2 GB each at peak
        loaded = pool.starmap(load_tile, [(have[k], a.vox) for k, _ in pick])
    V, S, tile_of = [], [], []
    for ((ix, iy), _), v in zip(pick, loaded):
        V.append(v)
        m = np.all(key == (ix, iy), axis=1)
        s = pcd[m]
        s = s[rng.choice(len(s), min(len(s), a.sample // len(pick)), replace=False)]
        S.append(s)
        tile_of.append(np.full(len(s), len(tile_of)))
        print(f"tile {ix},{iy}: vendor {len(v):,} (vox {a.vox} m), route sample {len(s):,}", flush=True)
    dst = np.concatenate(V)
    src = np.concatenate(S)
    tid = np.concatenate(tile_of)
    tree = cKDTree(dst)

    d0, _ = tree.query(src + t0)
    rep["initial"] = {"t": t0.tolist(), **stats(d0)}
    t, d = icp_translation(src, tree, dst, t0)
    rep["translation"] = {"t": np.round(t, 4).tolist(), **stats(d)}
    R, tr, dr = icp_rigid(src, tree, dst, t)
    yaw, pitch, roll = (np.degrees(np.arctan2(R[1, 0], R[0, 0])),
                        np.degrees(-np.arcsin(R[2, 0])),
                        np.degrees(np.arctan2(R[2, 1], R[2, 2])))
    c = src.mean(0)
    rep["rigid"] = {"R": np.round(R, 8).tolist(), "t": np.round(tr, 4).tolist(),
                    "yaw_deg": round(float(yaw), 5), "pitch_deg": round(float(pitch), 5),
                    "roll_deg": round(float(roll), 5),
                    "shift_at_route_centroid": np.round(R @ c + tr - c, 4).tolist(), **stats(dr)}
    per = []
    for i, ((ix, iy), _) in enumerate(pick):
        m = tid == i
        ti, di = icp_translation(src[m], tree, dst, t, iters=25)
        per.append({"tile": [ix, iy], "t": np.round(ti, 4).tolist(), **stats(di)})
    rep["per_tile_translation"] = per
    ts = np.array([p["t"] for p in per])
    rep["per_tile_spread_m"] = np.round(ts.max(0) - ts.min(0), 4).tolist()
    with open(a.out, "w") as f:
        json.dump(rep, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(json.dumps({k: rep[k] for k in ("initial", "translation", "rigid", "per_tile_spread_m")}, default=str,
                     indent=1))


if __name__ == "__main__":
    main()
