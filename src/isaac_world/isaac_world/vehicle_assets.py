"""CARLA's vehicle bodies on Isaac entities: render only, physics stays the boxes.

The assets are CARLA 0.9.16's vehicle blueprints exported with NVIDIA's SimReady converter
(CARLA's editor, one level per blueprint, `-run=SimReadyExport`), one directory per
blueprint id under `SWB_VEHICLE_USD` (default `/mnt/disk1/carla-usd/vehicles`), indexed by
its `vehicles.json`:

- `<id>/<id>.usda`, default prim `/Vehicle`: the whole car, lamps and collision mesh off.
  Frame: the CARLA actor origin (on the ground, about mid-length), metres, Z up, x forward,
  y left -- CARLA's Y already flipped, as in the SimReady town exports.
- `<id>/wheels.usda</Wheel_Front_Left>` etc.: one wheel each, centred on its own origin,
  axle along y.

An NPC's body sits with its CARLA bounding box centred, in x and y, on the declared box's,
its wheels on the ground plane of the entity origin: where swb's CARLA backend puts the same
blueprint (`origin_offset`). The ego's sits with its rear axle on base_link and wears the
car's own wheels on the PhysX wheel prims, which follow suspension, steering and spin. The
boxes stay the colliders, made invisible (to cameras and the RTX lidar alike) when a body
covers them; the meshes carry no collision.

Two-wheelers (roadmap 044 D): a bicycle or motorcycle box with no model wears the nearest
exported two-wheeler of its kind (CARLA's `base_type`), by length -- their CARLA boxes are
degenerate in the export's metadata and their render width is the rider's. CARLA's props
(`isaac_vehicles.sh props:=true`, `<root>/props/props.json`) dress MiscObjects that name a
`static.prop.*` blueprint; one naming a vehicle wears that vehicle. CARLA's walkers
(`isaac_vehicles.sh walkers:=true`, `<root>/walkers/walkers.json`, roadmap 044 C) dress
pedestrians, played by `isaac_world.walkers`. A two-wheeler's rider sits on it
(`isaac_world.rider`, baked into the wrapper by `vehicle_usd_assets.py`).
"""
import json
import logging
import os
from pathlib import Path

log = logging.getLogger("isaac_world")

DEFAULT_ROOT = "/mnt/disk1/carla-usd/vehicles"
# vehicle.py's PhysX wheel prims -> the export's wheel names
PHYSX_WHEELS = {"frontLeft": "Wheel_Front_Left", "frontRight": "Wheel_Front_Right",
                "rearLeft": "Wheel_Rear_Left", "rearRight": "Wheel_Rear_Right"}


class VehicleAssets:
    """The exported bodies by CARLA blueprint id; empty (every entity a box) without them."""

    def __init__(self, root=None):
        self.root = Path(root or os.environ.get("SWB_VEHICLE_USD", DEFAULT_ROOT))
        index = self.root / "vehicles.json"
        self.by_id = {}
        if index.exists():
            self.by_id = {e["id"]: e for e in json.loads(index.read_text())}
            log.info("%d CARLA vehicle bodies from %s", len(self.by_id), index)
        else:
            log.info("no CARLA vehicle bodies (%s missing): entities are boxes", index)
        # CARLA's physics per blueprint (scripts/dump_vehicle_physics.py, roadmap 043): in each
        # entry's `physics`, and in the `physics.json` beside the index, which survives the
        # index being regenerated.
        sidecar = self.root / "physics.json"
        self.physics_by_id = json.loads(sidecar.read_text()) if sidecar.exists() else {}
        props = self.root / "props" / "props.json"
        self.props = {e["id"]: e for e in json.loads(props.read_text())} if props.exists() else {}
        if self.props:
            log.info("%d CARLA props from %s", len(self.props), props)
        walkers = self.root / "walkers" / "walkers.json"
        self.walkers = json.loads(walkers.read_text()) if walkers.exists() else []
        if self.walkers:
            log.info("%d CARLA walkers from %s", len(self.walkers), walkers)

    def physics(self, model):
        """CARLA's `get_physics_control()` for `model`, in the catalogue's frame, or None."""
        entry = self.by_id.get(model or "") or {}
        return entry.get("physics") or self.physics_by_id.get(model or "")

    def get(self, model):
        """The asset for `model` (a CARLA blueprint id), or None."""
        entry = self.by_id.get(model or "")
        if model and entry is None:
            log.warning("no CARLA body for %r: a box", model)
        return entry

    def prop(self, model):
        """The exported prop for a `static.prop.*` blueprint id, or None."""
        entry = self.props.get(model or "")
        if model and entry is None:
            log.warning("no CARLA prop for %r: a box", model)
        return entry

    def nearest(self, dims, subtype=""):
        """The four-wheeled car whose CARLA box is closest in size to `dims` (length, width,
        height): what the CARLA backend does for a scenario vehicle that names no blueprint
        (OpenSCENARIO's `model3d` unset; roadmap 022). A `bicycle` or `motorcycle` gets the
        two-wheeler of that kind closest in length (044 D)."""
        if subtype in TWO_WHEELERS:
            bikes = [e for e in self.by_id.values() if e.get("wheels_n") == 2
                     and e.get("render_size")]
            same = [e for e in bikes if e.get("base_type") == subtype]
            if not (same or bikes):
                return None
            return min(same or bikes, key=lambda e: abs(e["render_size"][0] - dims[0]))
        cars = [e for e in self.by_id.values() if e.get("wheels_n") == 4 and e.get("carla_bbox")]
        if not cars:
            return None

        def misfit(e):
            size = [2 * x for x in e["carla_bbox"]["extent"]]
            return sum((a - b) ** 2 for a, b in zip(size, dims))

        return min(cars, key=misfit)


TWO_WHEELERS = ("bicycle", "motorcycle")


def _invisible(prim):
    from pxr import UsdGeom

    UsdGeom.Imageable(prim).MakeInvisible()


def add_npc_body(stage, entity_path, entry, center, collider=None):
    """`entry`'s car under the entity Xform at `entity_path` (the SSv2 entity origin): its
    CARLA bounding box centred on the declared box centre `center` in x and y, its ground at
    the origin's. Instanceable, so every NPC of one model shares its meshes, except a
    two-wheeler whose rider is seated (`rider_pose`: its SkelAnimation is not applied inside an
    instance). `collider`, the box, is made invisible."""
    from pxr import Gf, UsdGeom

    bb = entry.get("carla_bbox") or {"center": [0.0, 0.0, 0.0]}
    path = f"{entity_path}/model"
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(center[0] - bb["center"][0], center[1] - bb["center"][1], 0.0))
    prim = x.GetPrim()
    prim.GetReferences().AddReference(entry["usd"])
    prim.SetInstanceable(instanceable(entry))
    if collider is not None:
        _invisible(collider)
    return prim


def instanceable(entry):
    """Whether an NPC's body may be an instance: not a seated rider's (044 open item 2)."""
    return not entry.get("rider_pose")


def add_ego_body(stage, vehicle_path, entry, collider=None):
    """`entry`'s car on the PhysX vehicle at `vehicle_path` (whose frame is base_link: rear
    axle centre on the ground): the body with its rear axle on base_link and its own wheels
    removed, and each wheel on vehicle.py's PhysX wheel prim of the same corner in place of
    the stand-in cylinder. `collider`, the chassis box, is made invisible."""
    from pxr import Gf, UsdGeom

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


def prop_offset(entry, center, dims):
    """Where a prop's asset frame goes in the entity's: its CARLA box centred on the declared
    box in x and y, its bottom on the declared box's bottom."""
    bb = entry.get("carla_bbox") or {"center": [0.0, 0.0, 0.0], "extent": [0.0, 0.0, 0.0]}
    (bx, by, bz), ez = bb["center"], bb["extent"][2]
    return (center[0] - bx, center[1] - by, (center[2] - dims[2] / 2) - (bz - ez))


def add_prop_body(stage, entity_path, entry, center, dims, collider=None):
    """`entry`'s prop under the entity Xform (render only; `prop_offset`). Instanceable, so
    every entity of one prop shares its meshes. `collider`, the box, is made invisible."""
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, f"{entity_path}/model")
    x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*prop_offset(entry, center, dims)))
    prim = x.GetPrim()
    prim.GetReferences().AddReference(entry["usd"])
    prim.SetInstanceable(True)
    if collider is not None:
        _invisible(collider)
    return prim


# The walker export faces -y (CARLA's blueprint turns its mesh -90 degrees): this turns it to
# face the entity's x, degrees about z.
WALKER_YAW = 90.0


def add_walker_body(stage, entity_path, entry, joints, scales, center, dims, collider=None):
    """`entry`'s walker under the entity Xform, feet on the declared box's bottom, centred on
    it in x and y, facing x; with its own SkelAnimation (`<entity>/walkerAnim`) bound to its
    skeleton (`joints`, `scales`: the export's), which the world writes each frame. Not instanceable: each walker poses its own
    skeleton. `collider`, the box, is made invisible. Returns the SkelAnimation."""
    from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

    x = UsdGeom.Xform.Define(stage, f"{entity_path}/model")
    x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(center[0], center[1], center[2] - dims[2] / 2))
    x.AddRotateZOp().Set(WALKER_YAW)
    prim = x.GetPrim()
    prim.GetReferences().AddReference(entry["usd"])
    skel_root = next((p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Root)), None)
    if skel_root is None:
        log.warning("walker %s has no SkelRoot: it stands still", entry["id"])
        anim = None
    else:
        anim = UsdSkel.Animation.Define(stage, f"{entity_path}/walkerAnim")
        anim.CreateJointsAttr(Vt.TokenArray([str(j) for j in joints]))
        anim.CreateScalesAttr(Vt.Vec3hArray([Gf.Vec3h(*map(float, v)) for v in scales]))
        anim.CreateRotationsAttr()
        anim.CreateTranslationsAttr()
        UsdSkel.BindingAPI.Apply(skel_root).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    if collider is not None:
        _invisible(collider)
    return anim
