#!/usr/bin/env python3
"""Re-tile the NTU vendor MLS scan into 50 m LAZ tiles in the MGRS 51RUH local frame.

Why this exists: the vendor file is LAS 1.2, whose 32-bit point count saturates at
4294967295, while the file holds ~5.52 G points. Readers that trust the header (PDAL,
laspy) stop early. This script walks the LAZ chunk table instead, decodes every chunk, and
writes LAS 1.4 tiles whose counts are 64-bit.

Frame: TWD97/TM2 (EPSG:3826) -> UTM 51N (EPSG:32651), easting/northing mod 100000, i.e.
MGRS 51RUH local, the frame the Autoware maps r01/r02 use (projector_type MGRS). Z is left
in the vendor's vertical datum. The ICP shift to the Autoware maps (~ -0.3, +0.2, -17.26 m)
is NOT applied: it is a fitted value that may differ per route, so downstream exports apply
it. Over this 1.1 km site the projection change is fitted and checked
against pyproj with a quadratic polynomial (max error printed; must be < 1 mm).

The last chunk is partial and the chunk table does not record its point count (fixed-size
chunks). Its count is found by decoding it from a stream that ends at the chunk's last byte;
the decoder fails within ~2 points of the true end, and the final 2 decoded points are
dropped (see --tail-drop).

Two passes:
  1. N workers decode chunk ranges, transform, and append raw 34-byte format-3 records with
     X/Y/Z rewritten in the output frame to per-(worker, tile) fragment files in --work.
  2. Per tile, concatenate fragments and write <out>/tile_<ix>_<iy>.laz (LAS 1.4, fmt 3).

Usage:
  retile_laz.py IN.laz OUT_DIR --work /var/tmp/ntu-retile --workers 12
"""
import argparse
import glob
import io
import json
import os
import struct
import sys
import time
from multiprocessing import Pool

import laspy
import lazrs
import numpy as np
from pyproj import Transformer

TILE = 50.0
OUT_SCALE = 0.001
OUT_OFFSET = (50000.0, 65000.0, 0.0)  # MGRS-local x, y range ~51.9k-53.1k / 67.4k-68.1k
PS = 34  # point format 3 record size


def poly_terms(x, y):
    """Quadratic basis; x, y are TWD97 minus the source offsets (|x|,|y| < ~600 m)."""
    return np.c_[x, y, np.ones_like(x), x * x, x * y, y * y]


def fit_affine(h):
    """Quadratic TWD97 -> MGRS-local fit over the file bbox; returns (A[2x6], max_err_m)."""
    t = Transformer.from_crs("EPSG:3826", "EPSG:32651", always_xy=True)
    gx, gy = np.meshgrid(np.linspace(h.mins[0] - 50, h.maxs[0] + 50, 60),
                         np.linspace(h.mins[1] - 50, h.maxs[1] + 50, 60))
    gx, gy = gx.ravel(), gy.ravel()
    e, n = t.transform(gx, gy)
    e, n = np.asarray(e) % 1e5, np.asarray(n) % 1e5
    M = poly_terms(gx - h.offsets[0], gy - h.offsets[1])
    ce, *_ = np.linalg.lstsq(M, e, rcond=None)
    cn, *_ = np.linalg.lstsq(M, n, rcond=None)
    err = np.hypot(M @ ce - e, M @ cn - n).max()
    return np.vstack([ce, cn]), float(err)


def last_chunk_count(path, h, vlr, lv, ct):
    start = h.offset_to_point_data + 8 + sum(c[1] for c in ct[:-1])
    with open(path, "rb") as f:
        f.seek(start)
        data = f.read(ct[-1][1])
    tab = io.BytesIO()
    lazrs.write_chunk_table(tab, [(ct[-1][0], len(data))], lv)
    s = io.BytesIO(struct.pack("<q", 8 + len(data)) + data + tab.getvalue())
    dec = lazrs.LasZipDecompressor(s, vlr.record_data)
    buf = bytearray(PS)
    n = 0
    while n < ct[-1][0]:
        try:
            dec.decompress_many(buf)
        except lazrs.LazrsError:
            break
        n += 1
    return n


def worker(args):
    wid, path, chunks, last_n, A, h_scales, h_offsets, work = args
    with open(path, "rb") as f:
        h = laspy.LasHeader.read_from(f)
        vlr = [v for v in h.vlrs if v.record_id == 22204][0]
        f.seek(h.offset_to_point_data)
        dec = lazrs.LasZipDecompressor(f, vlr.record_data)
        c0, c1 = chunks
        dec.seek(c0 * 50000)
        files = {}
        stats = {"points": 0, "zmin": 1e9, "zmax": -1e9}
        batch = 20  # chunks per batch (1 M points)
        c = c0
        while c < c1:
            nc = min(batch, c1 - c)
            npts = nc * 50000
            if c + nc == len_ct_global[0]:
                npts = (nc - 1) * 50000 + last_n
            buf = bytearray(npts * PS)
            dec.decompress_many(buf)
            rec = np.frombuffer(buf, dtype=np.uint8).reshape(npts, PS)
            ints = rec[:, :12].copy().view(np.int32).reshape(-1, 3).astype(np.float64)
            x = ints[:, 0] * h_scales[0]
            y = ints[:, 1] * h_scales[1]
            z = ints[:, 2] * h_scales[2] + h_offsets[2]
            T = poly_terms(x, y)
            ex = T @ A[0]
            ny = T @ A[1]
            ix = np.floor(ex / TILE).astype(np.int64)
            iy = np.floor(ny / TILE).astype(np.int64)
            out = rec.copy()
            xyz = np.empty((npts, 3), dtype=np.int32)
            xyz[:, 0] = np.round((ex - OUT_OFFSET[0]) / OUT_SCALE)
            xyz[:, 1] = np.round((ny - OUT_OFFSET[1]) / OUT_SCALE)
            xyz[:, 2] = np.round((z - OUT_OFFSET[2]) / OUT_SCALE)
            out[:, :12] = xyz.view(np.uint8).reshape(npts, 12)
            key = ix * 100000 + iy
            order = np.argsort(key, kind="stable")
            ks = key[order]
            bounds = np.r_[0, np.flatnonzero(np.diff(ks)) + 1, len(ks)]
            for a, b in zip(bounds[:-1], bounds[1:]):
                k = int(ks[a])
                fh = files.get(k)
                if fh is None:
                    tx, ty = k // 100000, k % 100000
                    fh = open(os.path.join(work, f"t_{tx}_{ty}.w{wid:02d}.bin"), "ab")
                    files[k] = fh
                fh.write(out[order[a:b]].tobytes())
            stats["points"] += npts
            stats["zmin"] = min(stats["zmin"], float(z.min()))
            stats["zmax"] = max(stats["zmax"], float(z.max()))
            c += nc
        for fh in files.values():
            fh.close()
    return stats


len_ct_global = [0]


def init_worker(n):
    len_ct_global[0] = n


def write_tile(args):
    """Stream one tile's fragments into a LAZ file in slices (bounded memory)."""
    key, frags, outdir = args
    tx, ty = key
    hdr = laspy.LasHeader(point_format=3, version="1.4")
    hdr.scales = np.array([OUT_SCALE] * 3)
    hdr.offsets = np.array(OUT_OFFSET)
    path = os.path.join(outdir, f"tile_{tx}_{ty}.laz")
    tmp = path + ".part"
    n = 0
    slice_pts = 5_000_000
    with laspy.open(tmp, mode="w", header=hdr, laz_backend=laspy.LazBackend.Lazrs) as w:
        for p in sorted(frags):
            with open(p, "rb") as fh:
                while True:
                    data = fh.read(slice_pts * PS)
                    if not data:
                        break
                    k = len(data) // PS
                    w.write_points(laspy.PackedPointRecord.from_buffer(data, hdr.point_format, count=k))
                    n += k
    os.replace(tmp, path)
    for p in frags:
        os.remove(p)
    return tx, ty, n


def tile_summary(outdir):
    tiles = []
    for p in sorted(glob.glob(os.path.join(outdir, "tile_*.laz"))):
        with laspy.open(p) as r:
            h = r.header
            _, tx, ty = os.path.basename(p)[:-4].split("_")
            tiles.append({"ix": int(tx), "iy": int(ty), "points": int(h.point_count),
                          "min": [float(v) for v in h.mins], "max": [float(v) for v in h.maxs]})
    return tiles


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("--work", required=True)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--tail-drop", type=int, default=2)
    ap.add_argument("--max-chunks", type=int, default=0, help="debug: only the first N chunks")
    ap.add_argument("--pass2-workers", type=int, default=3,
                    help="tile writers; each holds one 5 M-point slice (~0.5 GB)")
    ap.add_argument("--resume-pass2", action="store_true",
                    help="skip pass 1; write tiles from fragments left in --work")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    os.makedirs(a.work, exist_ok=True)
    if glob.glob(os.path.join(a.work, "*.bin")) and not a.resume_pass2:
        sys.exit(f"{a.work} holds fragments from an earlier run; remove them or --resume-pass2")

    with open(a.inp, "rb") as f:
        h = laspy.LasHeader.read_from(f)
        vlr = [v for v in h.vlrs if v.record_id == 22204][0]
        lv = lazrs.LazVlr(vlr.record_data)
        f.seek(h.offset_to_point_data)
        ct = lazrs.read_chunk_table(f, lv)
    A, err = fit_affine(h)
    print(f"quadratic TWD97->MGRS-local max error {err * 1000:.3f} mm", flush=True)
    if err > 1e-3:
        sys.exit("fit error above 1 mm; use per-point pyproj instead")
    raw_last = last_chunk_count(a.inp, h, vlr, lv, ct)
    last_n = raw_last - a.tail_drop
    nchunks = len(ct) if not a.max_chunks else a.max_chunks
    total = (len(ct) - 1) * 50000 + last_n if not a.max_chunks else nchunks * 50000
    print(f"chunks {len(ct)}, last chunk decodes {raw_last}, keeping {last_n}; total {total:,}",
          flush=True)

    t0 = time.time()
    zmin = zmax = None
    if not a.resume_pass2:
        step = -(-nchunks // (a.workers * 8))
        ranges = [(c, min(c + step, nchunks)) for c in range(0, nchunks, step)]
        jobs = [(i, a.inp, r, last_n, A, tuple(h.scales), tuple(h.offsets), a.work)
                for i, r in enumerate(ranges)]
        t0 = time.time()
        done = 0
        zmin, zmax, pts = 1e9, -1e9, 0
        with Pool(a.workers, initializer=init_worker, initargs=(len(ct) if not a.max_chunks else -1,)) as pool:
            for st in pool.imap_unordered(worker, jobs):
                done += 1
                pts += st["points"]
                zmin, zmax = min(zmin, st["zmin"]), max(zmax, st["zmax"])
                el = time.time() - t0
                print(f"pass1 {done}/{len(jobs)} ranges, {pts:,} points, {el:.0f}s, "
                      f"{pts / el / 1e6:.1f} Mpts/s", flush=True)
        if pts != total:
            sys.exit(f"decoded {pts} != expected {total}")


    frags = {}
    for p in glob.glob(os.path.join(a.work, "t_*.bin")):
        name = os.path.basename(p).split(".")[0]
        _, tx, ty = name.split("_")
        frags.setdefault((int(tx), int(ty)), []).append(p)
    print(f"pass2: {len(frags)} tiles to write", flush=True)
    with Pool(a.pass2_workers) as pool:
        for i, r in enumerate(pool.imap_unordered(
                write_tile, [(k, v, a.out) for k, v in frags.items()])):
            print(f"pass2 {i + 1}/{len(frags)} tile_{r[0]}_{r[1]} {r[2]:,}", flush=True)
    tiles = tile_summary(a.out)
    written = sum(t["points"] for t in tiles)
    meta = {
        "source": os.path.abspath(a.inp),
        "frame": "MGRS 51RUH local (EPSG:32651 easting/northing mod 100000); "
                 "Z in vendor vertical datum; Autoware ICP shift NOT applied",
        "tile_size_m": TILE,
        "tile_name": "tile_<floor(x/50)>_<floor(y/50)>.laz",
        "quadratic_twd97_minus_offset_to_mgrs_local": {"basis": "x, y, 1, x*x, x*y, y*y",
                                                       "coef": A.tolist()},
        "source_offsets": list(h.offsets),
        "fit_max_error_m": err,
        "last_chunk_decoded": raw_last,
        "tail_drop": a.tail_drop,
        "total_points": written,
        "tiles": tiles,
    }
    with open(os.path.join(a.out, "tiles.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(f"done: {written:,} points in {len(tiles)} tiles, {time.time() - t0:.0f}s", flush=True)
    if written != total:
        sys.exit(f"written {written} != expected {total}")


if __name__ == "__main__":
    main()
