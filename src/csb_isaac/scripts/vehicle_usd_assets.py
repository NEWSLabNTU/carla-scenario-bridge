"""Wrap each SimReady vehicle export as a render-only asset, and tabulate them (plain pxr;
isaac_vehicles.sh, roadmap 043). csb_isaac.vehicle_assets reads what this writes.

    python -I vehicle_usd_assets.py $CSB_VEHICLE_USD

For every `<root>/<blueprint id>/<blueprint_id>.usd` (the SimReady map export of a level
holding only that blueprint at the origin), writes `<root>/<blueprint id>/<blueprint id>.usda`:
default prim `/Vehicle` referencing the export's `/Root`, with the actor's lamps
(SphereLights), its collision mesh, arrow and bounds box deactivated, and `customData` `csb`
carrying the geometry below. Writes `<root>/vehicles.json` and `<root>/vehicles.md`.

Frame (the export's, unchanged): metres, Z up, right-handed (CARLA's Y flipped), origin = the
CARLA actor origin (on the ground, roughly mid-length); x forward, y left.
"""
import json
import sys
from pathlib import Path

from pxr import Gf, Pcp, Sdf, Usd, UsdGeom, UsdLux

root = Path(sys.argv[1])
meta = {m["id"]: m for m in json.loads((root / "ue_meta.json").read_text())}
DROP_NAMES = {"CustomCollision", "ArrowHolder", "VehicleBounds"}
# Emissive lamp meshes the export splits by function: keep the running lamps, drop the ones
# that would otherwise shine forever (blinkers, brake, reverse, fog, high beam).
LAMPS_OFF = {"Left_Blinker", "Right_Blinker", "Brake", "Reverse", "Front_Fog", "Back_Fog", "High_Beam"}
WHEEL_ORDER = ["Wheel_Front_Left", "Wheel_Front_Right", "Wheel_Middle_Left", "Wheel_Middle_Right",
               "Wheel_Rear_Left", "Wheel_Rear_Right"]
rows = []
for d in sorted(p for p in root.iterdir() if p.is_dir() and p.name.startswith("vehicle.")):
    bp_id = d.name
    raw = d / (bp_id.replace(".", "_") + ".usd")
    if not raw.exists():
        print("skip (no export):", bp_id)
        continue
    stage = Usd.Stage.Open(str(raw))
    stage.SetEditTarget(stage.GetSessionLayer())  # never edit the export itself
    top = stage.GetPrimAtPath("/Root")
    actors = [c for c in top.GetChildren() if c.GetName() != "Looks"]
    if len(actors) != 1:
        print("skip (actors):", bp_id, [a.GetName() for a in actors])
        continue
    actor = actors[0]
    drop, wheels, wheel_parent = [], {}, None
    for p in Usd.PrimRange(actor):
        if (p.GetName() in LAMPS_OFF and p.GetParent().GetName() == "Lights") or \
                p.GetName() in DROP_NAMES or p.IsA(UsdLux.BoundableLightBase) or p.IsA(UsdLux.NonboundableLightBase):
            drop.append(p.GetPath())
        elif p.GetName() in WHEEL_ORDER and p.IsA(UsdGeom.Xform):
            t = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            wheels[p.GetName()] = (str(p.GetPath()).replace("/Root", "/Vehicle", 1), [round(v, 4) for v in t])
            wheel_parent = p.GetParent()
    # render bounds: what stays active
    for path in drop:
        stage.GetPrimAtPath(path).SetActive(False)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bc.ComputeWorldBound(top).ComputeAlignedRange()
    lo, hi = [round(v, 4) for v in rng.GetMin()], [round(v, 4) for v in rng.GetMax()]
    m = meta.get(bp_id, {})
    ue_wheels = {w["bone"]: w for w in m.get("wheels", [])}
    wl = []
    for name in WHEEL_ORDER:
        if name in wheels:
            path, pos = wheels[name]
            w = ue_wheels.get(name, {})
            wl.append({"name": name, "path": path, "pos": pos,
                       "radius": round(w.get("radius", 0) / 100, 4) if w else None,
                       "width": round(w.get("width", 0) / 100, 4) if w else None,
                       "max_steer_deg": w.get("max_steer")})
    bb = m.get("carla_bbox")
    carla_bbox = None
    if bb:  # UE cm, left-handed -> m, right-handed
        o, e = bb["origin"], bb["extent"]
        carla_bbox = {"center": [round(o[0] / 100, 4), round(-o[1] / 100, 4), round(o[2] / 100, 4)],
                      "extent": [round(v / 100, 4) for v in e]}
    front = [w for w in wl if "Front" in w["name"]]
    rear = [w for w in wl if "Rear" in w["name"]]
    rear_x = sum(w["pos"][0] for w in rear) / len(rear) if rear else None
    info = {
        "id": bp_id, "blueprint": m.get("class"), "base_type": m.get("base_type"),
        "wheels_n": m.get("wheels_n"),
        "usd": str(d / f"{bp_id}.usda"), "raw_usd": str(raw), "prim": "/Vehicle",
        "actor_prim": str(actor.GetPath()).replace("/Root", "/Vehicle", 1),
        "render_min": lo, "render_max": hi,
        "render_size": [round(hi[i] - lo[i], 4) for i in range(3)],
        "carla_bbox": carla_bbox,
        "wheels": wl,
        "rear_axle_x": round(rear_x, 4) if rear_x is not None else None,
        "wheel_base": round(front[0]["pos"][0] - rear_x, 4) if front and rear else None,
        "track": round(abs(rear[0]["pos"][1] - rear[1]["pos"][1]), 4) if len(rear) == 2 else None,
        "size_mb": round(sum(f.stat().st_size for f in d.rglob("*") if f.is_file()) / 1e6, 1),
    }
    # Wheels alone, each centred on its own origin (x forward, y the axle), materials kept:
    # `wheels.usda</Wheel_Front_Left>` etc. reference the skeletal mesh's prop file whole, with
    # every other part deactivated and the wheel's offset zeroed.
    if wheel_parent is not None:
        mesh_prim = wheel_parent.GetParent()  # <SK name> (wheel_parent is its "Mesh")
        prop = None
        for arc in Usd.PrimCompositionQuery(mesh_prim).GetCompositionArcs():
            if arc.GetArcType() in (Pcp.ArcTypePayload, Pcp.ArcTypeReference) and arc.GetTargetLayer():
                prop = (arc.GetTargetLayer().realPath, arc.GetTargetPrimPath())
        if prop:
            rel = "./" + str(Path(prop[0]).relative_to(d))
            wl_layer = Sdf.Layer.CreateNew(str(d / "wheels.usda"))
            ws = Usd.Stage.Open(wl_layer)
            UsdGeom.SetStageUpAxis(ws, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(ws, 1.0)
            prop_root = prop[1].GetParentPath()  # the prop's /Root, which holds its Looks
            inner = str(prop[1])[len(str(prop_root)):]  # /SM_x
            for wh in wl:
                x = UsdGeom.Xform.Define(ws, "/" + wh["name"])
                x.GetPrim().GetPayloads().AddPayload(rel, prop_root)
                for c in wheel_parent.GetChildren():
                    q = f"/{wh['name']}{inner}/Mesh/{c.GetName()}"
                    if c.GetName() != wh["name"]:
                        ws.OverridePrim(q).SetActive(False)
                    else:
                        ws.OverridePrim(q).CreateAttribute(
                            "xformOp:translate", Sdf.ValueTypeNames.Double3).Set(Gf.Vec3d(0, 0, 0))
                wh["usd"] = str(d / "wheels.usda")
                wh["usd_prim"] = "/" + wh["name"]
            wl_layer.Save()
            info["prop"] = prop[0]
    rows.append(info)
    # The wrapper
    layer = Sdf.Layer.CreateNew(str(d / f"{bp_id}.usda"))
    w = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(w, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(w, 1.0)
    v = UsdGeom.Xform.Define(w, "/Vehicle")
    w.SetDefaultPrim(v.GetPrim())
    Usd.ModelAPI(v.GetPrim()).SetKind("component")
    v.GetPrim().GetReferences().AddReference("./" + raw.name, "/Root")
    for path in drop:
        w.OverridePrim(str(path).replace("/Root", "/Vehicle", 1)).SetActive(False)
    csb = {k: info[k] for k in ("id", "render_min", "render_max", "rear_axle_x", "wheel_base", "track")
           if info[k] is not None}
    csb["frame"] = "CARLA actor origin; metres, Z up, x forward, y left (CARLA Y flipped)"
    if carla_bbox:
        csb["carla_bbox_center"] = carla_bbox["center"]
        csb["carla_bbox_extent"] = carla_bbox["extent"]
    for wh in wl:
        csb[wh["name"]] = {"path": wh["path"], "pos": wh["pos"],
                           **({"radius": wh["radius"]} if wh["radius"] else {})}
    v.GetPrim().SetCustomDataByKey("csb", csb)
    layer.Save()
    print(f"{bp_id}: {info['render_size']} wheels {len(wl)} dropped {len(drop)}")

(root / "vehicles.json").write_text(json.dumps(rows, indent=1))
lines = ["| blueprint id | USD | render size L x W x H (m) | CARLA bbox centre / half-extent (m) | "
         "wheels: radius, rear axle x, wheelbase, track (m) | MB |", "|---|---|---|---|---|---|"]
for r in rows:
    bb = r["carla_bbox"]
    rad = r["wheels"][0]["radius"] if r["wheels"] else None
    lines.append(f"| {r['id']} | {r['usd']} | {' x '.join('%.2f' % x for x in r['render_size'])} | "
                 + (f"{bb['center']} / {bb['extent']}" if bb else "-") + " | "
                 + f"{len(r['wheels'])}w r={rad} rear_x={r['rear_axle_x']} wb={r['wheel_base']} "
                 + f"tr={r['track']} | {r['size_mb']} |")
(root / "vehicles.md").write_text("\n".join(lines) + "\n")
print(len(rows), "vehicles")
