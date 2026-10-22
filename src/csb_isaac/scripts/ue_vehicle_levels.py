"""UE4 editor Python (run with -run=pythonscript; isaac_vehicles.sh, roadmap 043): one blank
level per CARLA vehicle in the VehicleFactory, the blueprint spawned at the origin, saved under
/Game/CSBVehicleExport/ for SimReady's map export. Writes metadata (blueprint id, BP path,
CARLA bounding box, wheels) as JSON to CSB_VEH_META, in UE units (cm, left-handed: x forward,
y right, z up). With CSB_VEH_ONLY (blueprint ids, comma separated) only those are prepared,
and their entries replace theirs in an existing CSB_VEH_META.

With CSB_VEH_KIND=props (roadmap 044) the same for CARLA's props instead: every
`static.prop.*` blueprint of Content/Carla/Config/Default.Package.json, its static mesh alone
at the origin, under /Game/CSBPropExport/; the metadata has the mesh's bounds.

With CSB_VEH_KIND=walkers (roadmap 044 C), CARLA's walkers: per `BP_Walker*` blueprint two
levels under /Game/CSBWalkerExport/, `<id>_walk` and `<id>_idle`, each a plain
SkeletalMeshActor of the walker's mesh playing a walk or an idle cycle in single-node mode
(SimReady exports that as a SkelAnimation; the blueprint's own anim blueprint it does not).
The id is the blueprint's name, lower case: which `walker.pedestrian.NNNN` a blueprint is
lives in WalkerFactory's graph, not in a property an editor script reads."""
import json
import os
import sys

import unreal

OUT = os.environ["CSB_VEH_META"]
ONLY = [s for s in os.environ.get("CSB_VEH_ONLY", "").split(",") if s]
KIND = os.environ.get("CSB_VEH_KIND", "vehicles")

def v(x):
    return [x.x, x.y, x.z]


def blank_level(level):
    """A new level `level`, from a template, emptied: a blank level has a zero editor camera,
    which SimReady's map export (SetCameraToCustomLayerData) takes for "unsaved" and reads from
    a level viewport a commandlet does not have (segfault)."""
    if unreal.EditorAssetLibrary.does_asset_exist(level):
        unreal.EditorAssetLibrary.delete_asset(level)
    unreal.EditorLevelLibrary.new_level_from_template(level, "/Engine/Maps/Templates/Template_Default")
    for a in unreal.EditorLevelLibrary.get_all_level_actors():
        if not isinstance(a, (unreal.WorldSettings, unreal.Brush)):
            unreal.EditorLevelLibrary.destroy_actor(a)


def write_meta(meta):
    if ONLY and os.path.exists(OUT):
        with open(OUT) as f:
            done = {m["id"] for m in meta}
            meta = [m for m in json.load(f) if m["id"] not in done] + meta
    with open(OUT, "w") as f:
        json.dump(meta, f, indent=1)
    unreal.log("CSB: wrote %s (%d)" % (OUT, len(meta)))


if KIND == "props":
    package = os.path.join(unreal.Paths.project_content_dir(), "Carla/Config/Default.Package.json")
    with open(package) as f:
        props = json.load(f)["props"]
    unreal.log("CSB: %s has %d props" % (package, len(props)))
    meta = []
    xf = unreal.Transform(unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1))
    for p in props:
        bp_id = "static.prop." + p["name"].lower()
        if ONLY and bp_id not in ONLY:
            continue
        if " " in bp_id:  # "Bike Helmet": no shell, path or SimReady map name takes it
            unreal.log_warning("CSB: skipped %r (a space in its id)" % bp_id)
            continue
        entry = {"id": bp_id, "mesh": p["path"], "size": p.get("size")}
        mesh = unreal.EditorAssetLibrary.load_asset(p["path"].split(".")[0])
        if mesh is None:
            entry["error"] = "no static mesh"
            meta.append(entry)
            continue
        level = "/Game/CSBPropExport/" + bp_id.replace(".", "_")
        blank_level(level)
        actor = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(unreal.StaticMeshActor, xf)
        actor.static_mesh_component.set_static_mesh(mesh)
        b = mesh.get_bounds()
        entry["level"] = level
        entry["carla_bbox"] = {"origin": v(b.origin), "extent": v(b.box_extent)}
        unreal.EditorLevelLibrary.save_current_level()
        unreal.log("CSB: saved %s" % level)
        meta.append(entry)
    write_meta(meta)


# Walk and idle cycles, by the skeleton they are for (044 C).
WALKER_ANIMS = [
    ("/Game/Carla/Static/Pedestrian/Animations/GEN3/AS_walking04_G3",
     "/Game/Carla/Static/Pedestrian/Animations/GEN3/AS_idleAXz02_G3"),
    ("/Game/Carla/Static/Pedestrian/Animations/Female/AS_Carla_WalkCicle",
     "/Game/Carla/Static/Pedestrian/Animations/Female/AS_Carla_Idle0B"),
    ("/Game/Carla/Static/Pedestrian/Animations/Male/AS_male_walkCicle0E",
     "/Game/Carla/Static/Pedestrian/Animations/Male/AS_male_IdleCicle0C"),
]


def walkers():
    reg = unreal.AssetRegistryHelpers.get_asset_registry()
    bps = sorted(str(a.asset_name) for a in reg.get_assets_by_path("/Game/Carla/Blueprints/Walkers")
                 if str(a.asset_name).startswith("BP_Walker") and str(a.asset_name) != "BP_Walker")
    anims = [(unreal.EditorAssetLibrary.load_asset(w), unreal.EditorAssetLibrary.load_asset(i))
             for w, i in WALKER_ANIMS]
    meta = []
    xf = unreal.Transform(unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1))
    for bp in bps:
        wid = bp.lower()
        if ONLY and wid not in ONLY:
            continue
        cls = unreal.EditorAssetLibrary.load_blueprint_class("/Game/Carla/Blueprints/Walkers/" + bp)
        entry = {"id": wid, "class": cls.get_path_name() if cls else None}
        blank_level("/Game/CSBWalkerExport/_probe")
        actor = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, xf) if cls else None
        sk = actor.get_components_by_class(unreal.SkeletalMeshComponent) if actor else []
        mesh = sk[0].get_editor_property("skeletal_mesh") if sk else None
        if mesh is None:
            entry["error"] = "no skeletal mesh"
            meta.append(entry)
            continue
        skel = mesh.get_editor_property("skeleton")
        entry["mesh"] = mesh.get_path_name()
        # The blueprint's mesh sits below and turned from the actor (capsule centre, x ahead).
        entry["mesh_rel"] = {"z": sk[0].get_editor_property("relative_location").z,
                             "yaw": sk[0].get_editor_property("relative_rotation").yaw}
        entry["skeleton"] = skel.get_path_name() if skel else None
        b = mesh.get_bounds()
        entry["mesh_bounds"] = {"origin": v(b.origin), "extent": v(b.box_extent)}
        pair = next(((w, i) for w, i in anims if w and i
                     and w.get_editor_property("skeleton") == skel
                     and i.get_editor_property("skeleton") == skel), None)
        if pair is None:
            entry["error"] = "no walk/idle cycle for skeleton %s" % entry["skeleton"]
            meta.append(entry)
            continue
        entry["levels"], entry["anims"] = {}, {}
        for which, anim in zip(("walk", "idle"), pair):
            level = "/Game/CSBWalkerExport/%s_%s" % (wid, which)
            blank_level(level)
            a = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(unreal.SkeletalMeshActor, xf)
            c = a.skeletal_mesh_component
            c.set_skeletal_mesh(mesh)
            c.set_editor_property("animation_mode", unreal.AnimationMode.ANIMATION_SINGLE_NODE)
            data = unreal.SingleAnimationPlayData()
            data.set_editor_property("anim_to_play", anim)
            data.set_editor_property("saved_looping", True)
            data.set_editor_property("saved_playing", True)
            c.set_editor_property("animation_data", data)
            c.set_animation_mode(unreal.AnimationMode.ANIMATION_SINGLE_NODE)
            c.set_animation(anim)
            unreal.EditorLevelLibrary.save_current_level()
            entry["levels"][which] = level
            entry["level"] = True
            entry["anims"][which] = {"path": anim.get_path_name(),
                                     "length": anim.get_editor_property("sequence_length")}
        meta.append(entry)
    write_meta(meta)


def vehicles():

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
        blank_level(level)
        # EditorLevelLibrary.spawn_actor_from_class needs a viewport (crashes in a commandlet);
        # CarlaTools' static helper is a plain UWorld::SpawnActor in the editor world.
        xf = unreal.Transform(unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1))
        actor = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, xf)
        entry["level"] = level
        if nw == 2:
            # The rider (roadmap 044 D): a two-wheeler's second skeletal mesh, posed in-game
            # by AB_Biker (IK to the handlebars), which an export does not run: left alone it
            # is exported in its bind pose, arms out and standing. SimReady exports the
            # animation a single-node component plays, so the rider plays a seated pose.
            anim = unreal.EditorAssetLibrary.load_asset(RIDER_ANIM)
            for c in actor.get_components_by_class(unreal.SkeletalMeshComponent):
                if c.get_name() == "VehicleMesh" or anim is None:
                    continue
                # As saved properties, which the export's load of the level initializes from.
                c.set_editor_property("animation_mode", unreal.AnimationMode.ANIMATION_SINGLE_NODE)
                data = unreal.SingleAnimationPlayData()
                data.set_editor_property("anim_to_play", anim)
                data.set_editor_property("saved_looping", True)
                data.set_editor_property("saved_playing", True)
                c.set_editor_property("animation_data", data)
                c.set_animation_mode(unreal.AnimationMode.ANIMATION_SINGLE_NODE)
                c.set_animation(anim)
                entry["rider_anim"] = RIDER_ANIM
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

    write_meta(meta)


if KIND == "walkers":
    walkers()
elif KIND != "props":
    vehicles()
