"""Wrap each SimReady prop export as a render-only asset, and tabulate them (plain pxr;
`isaac_vehicles.sh props:=true`, roadmap 044). isaac_world.vehicle_assets reads what this writes.

    python -I prop_usd_assets.py $SWB_VEHICLE_USD/props

For every `<root>/<blueprint id>/<blueprint_id>.usd` (the SimReady map export of a level
holding only that prop's static mesh at the origin), writes `<root>/<blueprint id>/<blueprint
id>.usda`: default prim `/Prop` referencing the export's `/Root`, lights off. Writes
`<root>/props.json` (paths relative to `<root>`): per prop its USD, render bounds and CARLA
box (the mesh's bounds, which CARLA's `bounding_box` of a prop actor is), metres, Z up,
CARLA's Y flipped.
"""
import json
import sys
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdLux

root = Path(sys.argv[1])


def indexed(path):
    """`path` as the index names it: relative to the root, so the set can move."""
    return str(Path(path).relative_to(root))


meta = {m["id"]: m for m in json.loads((root / "ue_meta.json").read_text())}
rows = []
for d in sorted(p for p in root.iterdir() if p.is_dir() and p.name.startswith("static.prop.")):
    bp_id = d.name
    raw = d / (bp_id.replace(".", "_") + ".usd")
    if not raw.exists():
        print("skip (no export):", bp_id)
        continue
    stage = Usd.Stage.Open(str(raw))
    top = stage.GetPrimAtPath("/Root")
    drop = [p.GetPath() for p in Usd.PrimRange(top)
            if p.IsA(UsdLux.BoundableLightBase) or p.IsA(UsdLux.NonboundableLightBase)]
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bc.ComputeWorldBound(top).ComputeAlignedRange()
    lo, hi = [round(v, 4) for v in rng.GetMin()], [round(v, 4) for v in rng.GetMax()]
    bb = meta.get(bp_id, {}).get("carla_bbox")
    carla_bbox = None
    if bb:  # UE cm, left-handed -> m, right-handed
        o, e = bb["origin"], bb["extent"]
        carla_bbox = {"center": [round(o[0] / 100, 4), round(-o[1] / 100, 4), round(o[2] / 100, 4)],
                      "extent": [round(v / 100, 4) for v in e]}
    info = {"id": bp_id, "usd": indexed(d / f"{bp_id}.usda"), "raw_usd": indexed(raw),
            "prim": "/Prop",
            "render_min": lo, "render_max": hi,
            "render_size": [round(hi[i] - lo[i], 4) for i in range(3)],
            "carla_bbox": carla_bbox,
            "size_mb": round(sum(f.stat().st_size for f in d.rglob("*") if f.is_file()
                                 and f.name != f"{bp_id}.usda") / 1e6, 2)}
    rows.append(info)
    path = d / f"{bp_id}.usda"
    if path.exists():
        path.unlink()
    layer = Sdf.Layer.CreateNew(str(path))
    w = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(w, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(w, 1.0)
    x = UsdGeom.Xform.Define(w, "/Prop")
    w.SetDefaultPrim(x.GetPrim())
    Usd.ModelAPI(x.GetPrim()).SetKind("component")
    x.GetPrim().GetReferences().AddReference("./" + raw.name, "/Root")
    for p in drop:
        w.OverridePrim(str(p).replace("/Root", "/Prop", 1)).SetActive(False)
    layer.Save()
    print(f"{bp_id}: {info['render_size']}")

(root / "props.json").write_text(json.dumps(rows, indent=1))
print(len(rows), "props")
