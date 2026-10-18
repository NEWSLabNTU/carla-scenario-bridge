"""UE4 editor Python (-run=pythonscript): what CARLA 0.9.16's vehicle blueprints configure
that get_physics_control() does not return -- the movement component's and the wheels'
UPROPERTYs, and the chassis body's mass properties (UE units: cm, kg, left-handed)."""
import json
import os
import traceback

import unreal

OUT = os.environ.get("CSB_PHYS_OUT", "/mnt/disk1/csb-043-vehicle/ue/ue_physics.json")
ONLY = [s for s in os.environ.get("CSB_VEH_ONLY", "").split(",") if s]


def conv(x, depth=0):
    if depth > 4:
        return str(x)
    if isinstance(x, (bool, int, float, str)) or x is None:
        return x
    if isinstance(x, unreal.Vector):
        return [x.x, x.y, x.z]
    if isinstance(x, unreal.Rotator):
        return [x.roll, x.pitch, x.yaw]
    if isinstance(x, unreal.Name):
        return str(x)
    if isinstance(x, (list, tuple, unreal.Array)):
        return [conv(e, depth + 1) for e in x]
    if isinstance(x, unreal.Object):
        return x.get_path_name()
    if isinstance(x, unreal.StructBase):
        out = {}
        for name in dir(x):
            if name.startswith("_") or name in ("cast", "copy", "assign", "export_text", "import_text", "static_struct", "to_tuple"):
                continue
            try:
                v = getattr(x, name)
            except Exception:
                continue
            if callable(v):
                continue
            out[name] = conv(v, depth + 1)
        return out
    if isinstance(x, unreal.EnumBase):
        return str(x)
    return str(x)


def props(obj, names):
    out = {}
    for n in names:
        try:
            out[n] = conv(obj.get_editor_property(n))
        except Exception as e:
            out[n] = "ERR " + str(e)[:80]
    return out


MOVE = ["mass", "drag_coefficient", "chassis_width", "chassis_height", "inertia_tensor_scale",
        "differential_setup", "ackermann_accuracy", "engine_setup", "transmission_setup",
        "steering_curve", "throttle_input_rate", "brake_input_rate", "handbrake_input_rate",
        "steering_input_rate", "idle_brake_input", "stop_threshold", "wrong_direction_threshold",
        "b_reverse_as_brake", "b_deprecated_spring_offset_mode", "min_normalized_tire_load",
        "min_normalized_tire_load_filtered", "max_normalized_tire_load",
        "max_normalized_tire_load_filtered", "threshold_longitudinal_speed",
        "low_forward_speed_sub_step_count", "high_forward_speed_sub_step_count",
        "wheel_setups", "use_rvo_avoidance"]
WHEEL = ["shape_radius", "shape_width", "mass", "damping_rate", "steer_angle",
         "affected_by_handbrake", "tire_config", "lat_stiff_max_load", "lat_stiff_value",
         "long_stiff_value", "suspension_force_offset", "suspension_max_raise",
         "suspension_max_drop", "suspension_natural_frequency", "suspension_damping_ratio",
         "sweep_type", "max_brake_torque", "max_hand_brake_torque", "offset", "b_dont_create_shape",
         "camber_angle_at_rest", "suspension_travel_direction" ]

fac_cls = unreal.EditorAssetLibrary.load_blueprint_class("/Game/Carla/Blueprints/Vehicles/VehicleFactory")
fac = unreal.get_default_object(fac_cls)
level = "/Game/CSBPhysicsDump/scratch"
if unreal.EditorAssetLibrary.does_asset_exist(level):
    unreal.EditorAssetLibrary.delete_asset(level)
unreal.EditorLevelLibrary.new_level_from_template(level, "/Engine/Maps/Templates/Template_Default")
for a in unreal.EditorLevelLibrary.get_all_level_actors():
    if not isinstance(a, (unreal.WorldSettings, unreal.Brush)):
        unreal.EditorLevelLibrary.destroy_actor(a)
try:
    dpm = unreal.PhysicalMaterial.get_default_object()
    default_pm = props(dpm, ["friction", "static_friction", "density", "raise_mass_to_power"])
except Exception as e:
    default_pm = str(e)
result = {"default_physical_material": default_pm, "vehicles": []}
for k, p in enumerate(fac.get_editor_property("Vehicles")):
    bp_id = ("vehicle.%s.%s" % (p.get_editor_property("make"), p.get_editor_property("model"))).lower()
    if ONLY and bp_id not in ONLY:
        continue
    cls = p.get_editor_property("class")
    entry = {"id": bp_id, "class": cls.get_path_name() if cls else None}
    if cls is None:
        result["vehicles"].append(entry)
        continue
    try:
        xf = unreal.Transform(unreal.Vector(0, 0, 200), unreal.Rotator(0, 0, 0), unreal.Vector(1, 1, 1))
        actor = unreal.OpenDriveToMap.spawn_actor_with_check_no_collisions(cls, xf)
        move = None
        for c in actor.get_components_by_class(unreal.ActorComponent):
            if "WheeledVehicleMovementComponent" in c.get_class().get_name() or "MovementComponent4W" in c.get_class().get_name():
                move = c
        entry["movement_class"] = move.get_class().get_name() if move else None
        if move is not None:
            entry["movement"] = props(move, MOVE)
            m = entry["movement"]
            m["engine_setup"] = props(move.get_editor_property("engine_setup"), ["max_rpm", "moi", "damping_rate_full_throttle", "damping_rate_zero_throttle_clutch_engaged", "damping_rate_zero_throttle_clutch_disengaged"])
            try:
                curve = move.get_editor_property("engine_setup").get_editor_property("torque_curve").get_rich_curve()
                m["engine_setup"]["torque_curve"] = [[k.time, k.value] for k in curve.get_copy_of_keys()] if hasattr(curve, "get_copy_of_keys") else str(curve)
            except Exception as e:
                m["engine_setup"]["torque_curve"] = "ERR " + str(e)[:80]
            ts = move.get_editor_property("transmission_setup")
            m["transmission_setup"] = props(ts, ["b_use_gear_auto_box", "use_gear_auto_box", "gear_switch_time", "gear_auto_box_latency", "final_ratio", "reverse_gear_ratio", "neutral_gear_up_ratio", "clutch_strength"])
            m["transmission_setup"]["forward_gears"] = [props(g, ["ratio", "down_ratio", "up_ratio"]) for g in ts.get_editor_property("forward_gears")]
            m["differential_setup"] = props(move.get_editor_property("differential_setup"), ["differential_type", "front_rear_split", "front_left_right_split", "rear_left_right_split", "centre_bias", "front_bias", "rear_bias"])
            for r in ("throttle_input_rate", "brake_input_rate", "handbrake_input_rate", "steering_input_rate"):
                m[r] = props(move.get_editor_property(r), ["rise_rate", "fall_rate"])
            m["wheel_setups"] = None
            wheels = []
            for ws in move.get_editor_property("wheel_setups"):
                wc = ws.get_editor_property("wheel_class")
                cdo = unreal.get_default_object(wc)
                w = {"class": wc.get_path_name(), "bone": str(ws.get_editor_property("bone_name")),
                     "additional_offset": conv(ws.get_editor_property("additional_offset")),
                     "disable_steering": ws.get_editor_property("disable_steering")}
                w.update(props(cdo, WHEEL))
                tc = cdo.get_editor_property("tire_config")
                if tc:
                    w["tire_config_props"] = props(tc, ["friction_scale", "tire_friction_scales"])
                wheels.append(w)
            entry["wheels"] = wheels
        mesh = actor.get_editor_property("mesh")
        entry["mesh"] = mesh.get_editor_property("skeletal_mesh").get_path_name() if mesh.get_editor_property("skeletal_mesh") else None
        entry["actor_location"] = conv(actor.get_actor_location())
        bi = mesh.get_editor_property("body_instance")
        entry["body_instance"] = props(bi, ["com_nudge", "b_override_mass", "mass_in_kg_override",
                                            "mass_scale", "inertia_tensor_scale", "linear_damping",
                                            "angular_damping", "phys_material_override"])
        try:
            entry["com_world"] = conv(mesh.get_center_of_mass())
            entry["mass_body"] = mesh.get_mass()
            entry["inertia"] = conv(mesh.get_inertia_tensor())
        except Exception as e:
            entry["com_err"] = str(e)
        pa = mesh.get_editor_property("skeletal_mesh").get_editor_property("physics_asset")
        bodies = []
        if False:
            for bs in pa.get_editor_property("skeletal_body_setups"):
                ag = bs.get_editor_property("agg_geom")
                b = {"bone": str(bs.get_editor_property("bone_name")),
                     "phys_material": conv(bs.get_editor_property("phys_material")),
                     "boxes": [props(e, ["center", "rotation", "x", "y", "z"]) for e in ag.get_editor_property("box_elems")],
                     "spheres": [props(e, ["center", "radius"]) for e in ag.get_editor_property("sphere_elems")],
                     "sphyls": [props(e, ["center", "rotation", "radius", "length"]) for e in ag.get_editor_property("sphyl_elems")],
                     "convex_n": len(ag.get_editor_property("convex_elems"))}
                cv = []
                for e in ag.get_editor_property("convex_elems"):
                    try:
                        cv.append({"vertices": conv(e.get_editor_property("vertex_data")),
                                   "transform": conv(e.get_editor_property("transform"))})
                    except Exception as ex:
                        cv.append({"err": str(ex)[:100]})
                b["convex"] = cv
                try:
                    b["default_instance"] = props(bs.get_editor_property("default_instance"), ["com_nudge", "b_override_mass", "mass_in_kg_override", "mass_scale"])
                except Exception as e:
                    b["default_instance"] = str(e)
                bodies.append(b)
        entry["physics_asset"] = pa.get_path_name() if pa else None
        entry["bodies"] = bodies
        unreal.EditorLevelLibrary.destroy_actor(actor)
    except Exception:
        entry["error"] = traceback.format_exc()
    result["vehicles"].append(entry)
    unreal.log("CSBPHYS: %s" % bp_id)
with open(OUT, "w") as f:
    json.dump(result, f, indent=1)
unreal.EditorAssetLibrary.delete_asset(level)
unreal.log("CSBPHYS: wrote %s" % OUT)
