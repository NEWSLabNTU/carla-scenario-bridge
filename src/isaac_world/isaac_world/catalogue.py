"""The ego against the CARLA vehicle catalogue (roadmaps 043, 045): pure data, no Isaac.

The ego itself is aib's (autoware_isaac_bridge, `src/autoware_isaac_bridge`): an ego config
(`aib/config/ego.yaml`) names a rig, the rig is the car (generated from aib's vehicle spec).
What stays swb's is the CARLA side: the body the ego wears comes from the vehicle catalogue
(SWB_VEHICLE_USD/vehicles.json, CARLA's blueprints exported to USD), and the catalogue's
`physics` (`scripts/dump_vehicle_physics.py`) must describe the same car as the rig's
geometry, or the body's wheels would not sit on the PhysX wheels. `check_ego` refuses a
mismatch at world server start; `physics_params` is CARLA's car as VehicleParams fields, for
comparison only.

Import with aib on `sys.path` (the world server's PYTHONPATH has `.../src/aib`).
"""
import math

from aib.params import GEOMETRY_KEYS, bicycle_limit, check_geometry
from aib.rig import EgoConfig

DEFAULT_MODEL = "vehicle.tesla.model3"


def load_ego(path):
    """(EgoConfig, Rig) for an aib ego config: the rig with the config's overrides applied."""
    cfg = EgoConfig.load(path)
    return cfg, cfg.load_rig()


def ego_model(rig):
    """The catalogue id of the body the ego wears: the rig's, else the Tesla."""
    return rig.catalogue_model or rig.vehicle.model or DEFAULT_MODEL


def check_ego(rig, physics):
    """Raise aib.params.VehicleMismatch when the rig's geometry and the catalogue's `physics`
    for its body differ on wheel base, tread or radius by more than 5 cm. No physics (the
    catalogue lacks the model): nothing to check."""
    if not physics:
        return
    ours = {k: getattr(rig.vehicle, k) for k in GEOMETRY_KEYS}
    theirs = {k: physics[k] for k in GEOMETRY_KEYS if k in physics}
    check_geometry(f"{ego_model(rig)}'s catalogue physics", theirs, ours)


def physics_params(physics):
    """What CARLA's blueprint would give as VehicleParams fields (dump_vehicle_physics.py):
    reference only, for comparing the ego with CARLA's car; the ego never applies it."""
    out = {"mass": float(physics["mass"])}
    wheels = physics.get("wheels") or []
    front = [w for w in wheels if w.get("max_steer_angle_deg", 0) > 0] or wheels[:2]
    rear = [w for w in wheels if w not in front]
    if wheels:
        w0 = front[0] if front else wheels[0]
        out.update(
            tire_friction=float(w0["tire_friction"]),
            lat_stiff_max_load=float(w0["lat_stiff_max_load"]),
            lat_stiff_value=float(w0["lat_stiff_value"]),
            long_stiff_value=float(w0["long_stiff_value"]),
            wheel_damping_rate=float(w0["damping_rate"]),
            max_brake_torque=float(max(w["max_brake_torque"] for w in wheels)),
        )
    if "wheel_base" in physics and "wheel_tread" in physics and front:
        inner = math.radians(max(w["max_steer_angle_deg"] for w in front))
        out["max_steer_angle"] = bicycle_limit(inner, physics["wheel_base"],
                                               physics["wheel_tread"])
    com = physics.get("com_estimate")
    if com and rear and "rear_axle_x" in physics:
        # base_link: the rear axle centre on the ground (the wheel centre less its radius).
        ground = sum(w["position"][2] - w["radius"] for w in rear) / len(rear)
        out["com"] = (com[0] - physics["rear_axle_x"], com[1], com[2] - ground)
    peak = max((t for _, t in physics.get("torque_curve") or []), default=0.0)
    gear = max((g["ratio"] for g in physics.get("forward_gears") or []), default=1.0)
    if peak > 0 and len(wheels) >= 4:
        out["max_drive_torque"] = peak * float(physics.get("final_ratio", 1.0)) * gear / len(wheels)
    return out
