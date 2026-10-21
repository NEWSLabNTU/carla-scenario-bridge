"""UE4 editor Python (run with -run=pythonscript; isaac_vehicles.sh, roadmap 043): one blank
level per CARLA vehicle in the VehicleFactory, the blueprint spawned at the origin, saved under
/Game/CSBVehicleExport/ for SimReady's map export. Writes metadata (blueprint id, BP path,
CARLA bounding box, wheels) as JSON to CSB_VEH_META, in UE units (cm, left-handed: x forward,
y right, z up). With CSB_VEH_ONLY (blueprint ids, comma separated) only those are prepared,
and their entries replace theirs in an existing CSB_VEH_META."""
import json
import os
import sys

import unreal

OUT = os.environ["CSB_VEH_META"]
ONLY = [s for s in os.environ.get("CSB_VEH_ONLY", "").split(",") if s]

def v(x):
    return [x.x, x.y, x.z]

fac_cls = unreal.EditorAssetLibrary.load_blueprint_class("/Game/Carla/Blueprints/Vehicles/VehicleFactory")
fac = unreal.get_default_object(fac_cls)
vehicles = fac.get_editor_property("Vehicles")
unreal.log("CSB: factory has %d vehicles" % len(vehicles))

meta = []
for p in vehicles:
    make = p.get_editor_property("make")
    model = p.get_editor_property("model")
    bp_id = ("vehicle.%s.%s" % (make, model)).lower()
    if ONLY and bp_id not in ONLY:
        continue
    cls = p.get_editor_property("class")
    nw = p.get_editor_property("number_of_wheels")
    entry = {"id": bp_id, "class": cls.get_path_name() if cls else None, "wheels_n": nw}
    try:
        entry["base_type"] = p.get_editor_property("base_type")
        entry["object_type"] = p.get_editor_property("object_type")
        entry["generation"] = p.get_editor_property("generation")
    except Exception as e:  # noqa
        pass
    if cls is None:
        meta.append(entry)
        continue
    safe = bp_id.replace(".", "_")
    level = "/Game/CSBVehicleExport/" + safe
    # From a template, emptied: a blank level has a zero editor camera, which SimReady's
    # map export (SetCameraToCustomLayerData) takes for "unsaved" and reads from a level
    # viewport a commandlet does not have (segfault).
    if unreal.EditorAssetLibrary.does_asset_exist(level):
        unreal.EditorAssetLibrary.delete_asset(level)
    unreal.EditorLevelLibrary.new_level_from_template(level, "/Engine/Maps/Templates/Template_Default")
    world = unreal.EditorLevelLibrary.get_editor_world()
    for a in unreal.EditorLevelLibrary.get_all_level_actors():
        if not isinstance(a, (unreal.WorldSettings, unreal.Brush)):
            unreal.EditorLevelLibrary.destroy_actor(a)
    # EditorLevelLibrary.spawn_actor_from_class needs a viewport (crashes in a commandlet);
    # CarlaTools' static helper is a plain UWorld::SpawnActor in the editor world.
    xf = unreal.Transform(unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1))
    actor = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, xf)
    entry["level"] = level
    try:
        bb = unreal.BoundingBoxCalculator.get_vehicle_bounding_box(actor)
        entry["carla_bbox"] = {"origin": v(bb.origin), "extent": v(bb.extent)}
    except Exception as e:
        entry["carla_bbox_err"] = str(e)
    for box in actor.get_components_by_class(unreal.BoxComponent):
        if box.get_name() == "VehicleBounds":
            entry["bp_vehicle_bounds"] = {"loc": v(box.get_editor_property("relative_location")),
                                          "extent": v(box.get_unscaled_box_extent())}
    sk = actor.get_components_by_class(unreal.SkeletalMeshComponent)
    if sk:
        sk = sk[0]
        mesh = sk.get_editor_property("skeletal_mesh")
        entry["skeletal_mesh"] = mesh.get_path_name() if mesh else None
        entry["mesh_comp_loc"] = v(sk.get_editor_property("relative_location"))
        b = mesh.get_bounds() if mesh else None
        if b:
            entry["mesh_bounds"] = {"origin": v(b.origin), "extent": v(b.box_extent)}
        wheels = []
        mv = actor.get_component_by_class(unreal.WheeledVehicleMovementComponent)
        if mv:
            for ws in mv.get_editor_property("wheel_setups"):
                bone = ws.get_editor_property("bone_name")
                wc = ws.get_editor_property("wheel_class")
                w = {"bone": str(bone), "pos": v(sk.get_socket_location(bone))}
                if wc:
                    cdo = unreal.get_default_object(wc)
                    w["radius"] = cdo.get_editor_property("shape_radius")
                    w["width"] = cdo.get_editor_property("shape_width")
                    w["max_steer"] = cdo.get_editor_property("steer_angle")
                wheels.append(w)
        entry["wheels"] = wheels
        entry["bones"] = [str(n) for n in sk.get_all_socket_names()][:40]
    unreal.EditorLevelLibrary.save_current_level()
    unreal.log("CSB: saved %s" % level)
    meta.append(entry)

if ONLY and os.path.exists(OUT):
    with open(OUT) as f:
        done = {m["id"] for m in meta}
        meta = [m for m in json.load(f) if m["id"] not in done] + meta
with open(OUT, "w") as f:
    json.dump(meta, f, indent=1)
unreal.log("CSB: wrote %s (%d)" % (OUT, len(meta)))
