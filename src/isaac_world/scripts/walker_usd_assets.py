"""Wrap each SimReady walker export as an asset, keep its cycles as arrays, and tabulate them
(plain pxr and numpy; `isaac_vehicles.sh walkers:=true`, roadmap 044 C). isaac_world's world
reads what this writes.

    python -I walker_usd_assets.py $SWB_VEHICLE_USD/walkers

For every `<root>/<id>/` holding `<id>_walk/<id>_walk.usd` and `<id>_idle/<id>_idle.usd` (the
SimReady map exports of a level holding only that walker's skeletal mesh at the origin,
playing its walk or idle cycle), writes:

- `<root>/<id>/<id>.usda`: default prim `/Walker` referencing the walk export's `/Root`,
  lights off. Its feet are at z = 0, it faces -y (CARLA's mesh is turned -90 degrees in the
  blueprint), metres, Z up.
- `<root>/<id>/<cycle>.npz` per cycle: `joints`, `rotations` (frames x joints x 4, x y z w),
  `translations` (frames x joints x 3), `scales` (joints x 3), `fps`; the world plays them by
  writing one frame into a per-walker SkelAnimation.

and `<root>/walkers.json`: per walker its USD, height and cycles.
"""
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdSkel

root = Path(sys.argv[1])
meta = {m["id"]: m for m in json.loads((root / "ue_meta.json").read_text())}


def cycle(usd):
    """The export's SkelAnimation as arrays, sampled at its own time codes."""
    stage = Usd.Stage.Open(str(usd))
    prim = next(p for p in stage.Traverse() if p.IsA(UsdSkel.Animation))
    a = UsdSkel.Animation(prim)
    times = a.GetRotationsAttr().GetTimeSamples() or [Usd.TimeCode.Default()]
    rots = np.array([np.array(a.GetRotationsAttr().Get(t)) for t in times], dtype=np.float32)
    trs = np.array([np.array(a.GetTranslationsAttr().Get(t)) for t in times], dtype=np.float32)
    scl = np.array(a.GetScalesAttr().Get(times[0]), dtype=np.float32)
    fps = stage.GetTimeCodesPerSecond() or 30.0
    return dict(joints=np.array(list(a.GetJointsAttr().Get())), rotations=rots,
                translations=trs, scales=scl, fps=np.float32(fps))


rows = []
for d in sorted(p for p in root.iterdir() if p.is_dir() and p.name.startswith("bp_walker")):
    wid = d.name
    exports = {c: d / f"{wid}_{c}" / f"{wid}_{c}.usd" for c in ("walk", "idle")}
    if not all(p.exists() for p in exports.values()):
        print("skip (no export):", wid)
        continue
    cycles = {}
    for c, usd in exports.items():
        arrays = cycle(usd)
        np.savez(d / f"{c}.npz", **arrays)
        frames = len(arrays["rotations"])
        cycles[c] = {"npz": str(d / f"{c}.npz"), "frames": frames,
                     "length": round((frames - 1) / float(arrays["fps"]), 4)}
    stage = Usd.Stage.Open(str(exports["walk"]))
    top = stage.GetPrimAtPath("/Root")
    drop = [p.GetPath() for p in Usd.PrimRange(top)
            if p.IsA(UsdLux.BoundableLightBase) or p.IsA(UsdLux.NonboundableLightBase)]
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bc.ComputeWorldBound(top).ComputeAlignedRange()
    lo, hi = [round(v, 4) for v in rng.GetMin()], [round(v, 4) for v in rng.GetMax()]
    path = d / f"{wid}.usda"
    if path.exists():
        path.unlink()
    layer = Sdf.Layer.CreateNew(str(path))
    w = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(w, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(w, 1.0)
    x = UsdGeom.Xform.Define(w, "/Walker")
    w.SetDefaultPrim(x.GetPrim())
    Usd.ModelAPI(x.GetPrim()).SetKind("component")
    x.GetPrim().GetReferences().AddReference(f"./{wid}_walk/{wid}_walk.usd", "/Root")
    for p in drop:
        w.OverridePrim(str(p).replace("/Root", "/Walker", 1)).SetActive(False)
    layer.Save()
    m = meta.get(wid, {})
    rows.append({"id": wid, "usd": str(path), "prim": "/Walker", "mesh": m.get("mesh"),
                 "render_min": lo, "render_max": hi, "height": round(hi[2] - lo[2], 4),
                 "cycles": cycles})
    print(f"{wid}: height {rows[-1]['height']} m, walk {cycles['walk']['frames']} frames, "
          f"idle {cycles['idle']['frames']}")

(root / "walkers.json").write_text(json.dumps(rows, indent=1))
print(len(rows), "walkers")
