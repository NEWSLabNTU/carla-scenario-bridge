"""CARLA's vehicle bodies on Isaac entities: render only, physics stays the boxes.

The assets are CARLA 0.9.16's vehicle blueprints exported with NVIDIA's SimReady converter
(CARLA's editor, one level per blueprint, `-run=SimReadyExport`), one directory per
blueprint id under `CSB_VEHICLE_USD` (default `/mnt/disk1/carla-usd/vehicles`), indexed by
its `vehicles.json`:

- `<id>/<id>.usda`, default prim `/Vehicle`: the whole car, lamps and collision mesh off.
  Frame: the CARLA actor origin (on the ground, about mid-length), metres, Z up, x forward,
  y left -- CARLA's Y already flipped, as in the SimReady town exports.
- `<id>/wheels.usda</Wheel_Front_Left>` etc.: one wheel each, centred on its own origin,
  axle along y.

An NPC's body sits with its CARLA bounding box centred, in x and y, on the declared box's,
its wheels on the ground plane of the entity origin: where csb's CARLA backend puts the same
blueprint (`origin_offset`). The ego's sits with its rear axle on base_link and wears the
car's own wheels on the PhysX wheel prims, which follow suspension, steering and spin. The
boxes stay the colliders, made invisible (to cameras and the RTX lidar alike) when a body
covers them; the meshes carry no collision.
"""
import json
import logging
import os
from pathlib import Path

from pxr import Gf, UsdGeom

log = logging.getLogger("csb_isaac")

DEFAULT_ROOT = "/mnt/disk1/carla-usd/vehicles"
# vehicle.py's PhysX wheel prims -> the export's wheel names
PHYSX_WHEELS = {"frontLeft": "Wheel_Front_Left", "frontRight": "Wheel_Front_Right",
                "rearLeft": "Wheel_Rear_Left", "rearRight": "Wheel_Rear_Right"}


class VehicleAssets:
    """The exported bodies by CARLA blueprint id; empty (every entity a box) without them."""

    def __init__(self, root=None):
        self.root = Path(root or os.environ.get("CSB_VEHICLE_USD", DEFAULT_ROOT))
        index = self.root / "vehicles.json"
        self.by_id = {}
        if index.exists():
            self.by_id = {e["id"]: e for e in json.loads(index.read_text())}
            log.info("%d CARLA vehicle bodies from %s", len(self.by_id), index)
        else:
            log.info("no CARLA vehicle bodies (%s missing): entities are boxes", index)

    def get(self, model):
        """The asset for `model` (a CARLA blueprint id), or None."""
        entry = self.by_id.get(model or "")
        if model and entry is None:
            log.warning("no CARLA body for %r: a box", model)
        return entry

    def nearest(self, dims):
        """The four-wheeled car whose CARLA box is closest in size to `dims` (length, width,
        height): what the CARLA backend does for a scenario vehicle that names no blueprint
        (OpenSCENARIO's `model3d` unset; roadmap 022)."""
        cars = [e for e in self.by_id.values() if e.get("wheels_n") == 4 and e.get("carla_bbox")]
        if not cars:
            return None

        def misfit(e):
            size = [2 * x for x in e["carla_bbox"]["extent"]]
            return sum((a - b) ** 2 for a, b in zip(size, dims))

        return min(cars, key=misfit)


def _invisible(prim):
    UsdGeom.Imageable(prim).MakeInvisible()


def add_npc_body(stage, entity_path, entry, center, collider=None):
    """`entry`'s car under the entity Xform at `entity_path` (the SSv2 entity origin): its
    CARLA bounding box centred on the declared box centre `center` in x and y, its ground at
    the origin's. Instanceable, so every NPC of one model shares its meshes. `collider`, the
    box, is made invisible."""
    bb = entry.get("carla_bbox") or {"center": [0.0, 0.0, 0.0]}
    path = f"{entity_path}/model"
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(center[0] - bb["center"][0], center[1] - bb["center"][1], 0.0))
    prim = x.GetPrim()
    prim.GetReferences().AddReference(entry["usd"])
    prim.SetInstanceable(True)
    if collider is not None:
        _invisible(collider)
    return prim


def add_ego_body(stage, vehicle_path, entry, collider=None):
    """`entry`'s car on the PhysX vehicle at `vehicle_path` (whose frame is base_link: rear
    axle centre on the ground): the body with its rear axle on base_link and its own wheels
    removed, and each wheel on vehicle.py's PhysX wheel prim of the same corner in place of
    the stand-in cylinder. `collider`, the chassis box, is made invisible."""
    path = f"{vehicle_path}/model"
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(-(entry.get("rear_axle_x") or 0.0), 0.0, 0.0))
    prim = x.GetPrim()
    prim.GetReferences().AddReference(entry["usd"])
    wheels = {w["name"]: w for w in entry.get("wheels", [])}
    for w in wheels.values():
        # "/Vehicle/..." in the asset is `path` here
        stage.OverridePrim(path + w["path"][len("/Vehicle"):]).SetActive(False)
    dressed = 0
    for physx_name, name in PHYSX_WHEELS.items():
        wp = f"{vehicle_path}/{physx_name}"
        w = wheels.get(name)
        if w is None or not w.get("usd") or not stage.GetPrimAtPath(wp):
            continue
        mesh = UsdGeom.Xform.Define(stage, f"{wp}/model")
        mesh.GetPrim().GetPayloads().AddPayload(w["usd"], w["usd_prim"])
        stand_in = stage.GetPrimAtPath(f"{wp}/render")
        if stand_in:
            _invisible(stand_in)
        dressed += 1
    if collider is not None:
        _invisible(collider)
    log.info("ego wears %s (rear axle at x=%.3f in the asset, %d wheel(s))",
             entry["id"], entry.get("rear_axle_x") or 0.0, dressed)
    return prim
