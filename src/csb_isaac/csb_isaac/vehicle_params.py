"""What the ego's PhysX vehicle is (roadmap 038, 043): pure data, no Isaac.

Two descriptions of the one car meet here. Autoware plans with acb's `vehicle_info`
(`vehicle_info.param.yaml`: wheel base, tread, wheel radius and width); CARLA's blueprint has
the physics (`get_physics_control()`, dumped into the vehicle catalogue's `physics` by
`scripts/dump_vehicle_physics.py`: mass, centre of mass, wheel radius and steering limit, brake
torque, tire friction and stiffness, the engine's peak torque). `VehicleParams.resolve` takes
the geometry from the first and the physics from the second, and refuses the pair when they
describe different cars (`GEOMETRY_TOLERANCE`), so a different model is never driven with the
Tesla's description by accident. CARLA 0.9.16 does not expose suspension: that stays ours.

The ego YAML's `vehicle:` mapping overrides any field afterwards (by its name here), for an
experiment; the check is on what the two sources say, not on the overrides.
"""
import math
from dataclasses import dataclass, fields

G = 9.81
# How far vehicle_info and the catalogue may disagree on wheel base, tread or radius, m.
GEOMETRY_TOLERANCE = 0.05
GEOMETRY_KEYS = ("wheel_base", "wheel_tread", "wheel_radius")


class VehicleMismatch(ValueError):
    """vehicle_info and the CARLA blueprint describe different cars."""


@dataclass
class VehicleParams:
    """Defaults are CARLA 0.9.16's vehicle.tesla.model3 and acb's vehicle_info for it."""

    model: str = ""  # the CARLA blueprint the physics came from
    # Geometry: vehicle_info
    wheel_base: float = 3.005
    wheel_tread: float = 1.667
    wheel_radius: float = 0.370
    wheel_width: float = 0.235
    # Physics: CARLA's blueprint
    max_steer_angle: float = 1.0005  # bicycle tire angle at the inner wheel's 70 deg, rad
    mass: float = 1845.0
    com: tuple = (1.8656, 0.0, 0.4980)  # centre of mass in base_link, m (an estimate)
    tire_friction: float = 3.5
    lat_stiff_max_load: float = 3.0  # in rest loads: where lateral stiffness stops growing
    lat_stiff_value: float = 20.0  # lateral stiffness there, N/rad per N of rest load
    long_stiff_value: float = 3000.0  # longitudinal stiffness per g, kg (x g: N per unit slip)
    wheel_damping_rate: float = 0.25
    max_drive_torque: float = 1671.75  # per wheel, N m: peak engine torque x final drive / 4
    max_brake_torque: float = 700.0  # per wheel, N m
    # Ours: CARLA 0.9.16 does not expose these
    suspension_frequency: float = 1.6  # Hz
    suspension_damping_ratio: float = 0.45
    suspension_travel: float = 0.2  # m
    hold_brake_torque: float = 1500.0  # per wheel, N m: standing still
    rolling_resistance: float = 0.0  # N per (m/s), added to the requested force
    steer_time_constant: float = 0.0  # s; 0: none
    steer_rate_limit: float = 0.0  # rad/s; 0: none
    # PhysX vehicle integration. The wheel's spin is stiff against the tire (the slip goes as
    # 1/speed): at 1 substep per 1/60 s it went unstable under braking, locked the wheels and
    # stopped the car at the friction limit -- -11 m/s^2 for a -3 request at friction 1, -30
    # at CARLA's 3.5 (roadmap 043). 8 substeps at every speed, and a slip denominator of 4 m/s
    # while braked or driven, track the request.
    physx_substeps: int = 8
    min_active_slip_denominator: float = 4.0  # m/s

    @classmethod
    def from_vehicle_info(cls, path, **overrides):
        """`vehicle_info.param.yaml` (ROS parameter file) plus `overrides`; no catalogue."""
        return cls(**{**read_vehicle_info(path), **overrides})

    @classmethod
    def resolve(cls, model, physics, vehicle_info=None, overrides=None):
        """The ego's parameters: geometry from `vehicle_info` (a dict, as `read_vehicle_info`
        gives it), physics from the catalogue's `physics` for `model`, then `overrides`.

        Raises `VehicleMismatch` when the two disagree on wheel base, tread or radius by more
        than `GEOMETRY_TOLERANCE`, naming both numbers."""
        values = {"model": model or ""}
        if physics:
            check_geometry(model, physics, vehicle_info)
            values.update(physics_params(physics))
        if vehicle_info:
            values.update(vehicle_info)
        known = {f.name for f in fields(cls)}
        overrides = dict(overrides or {})
        unknown = set(overrides) - known
        if unknown:
            raise ValueError(f"unknown vehicle override(s) {sorted(unknown)}; known: "
                             f"{sorted(known)}")
        values.update({k: (tuple(v) if isinstance(v, list) else v) for k, v in overrides.items()})
        return cls(**values)

    def axle_loads(self):
        """Sprung mass per wheel (front, rear), kg: the mass split by the centre of mass."""
        front = min(max(self.com[0] / self.wheel_base, 0.05), 0.95)
        return self.mass * front / 2, self.mass * (1 - front) / 2


def read_vehicle_info(path):
    """The geometry Autoware plans with, from `vehicle_info.param.yaml`."""
    import yaml

    with open(path) as f:
        doc = yaml.safe_load(f)
    p = next(iter(doc.values()))["ros__parameters"]
    return {k: float(p[k]) for k in (*GEOMETRY_KEYS, "wheel_width")}


def catalogue_geometry(physics):
    """Wheel base, tread and radius as CARLA's wheels give them."""
    return {k: physics[k] for k in GEOMETRY_KEYS if k in physics}


def check_geometry(model, physics, vehicle_info):
    if not vehicle_info:
        return
    theirs = catalogue_geometry(physics)
    bad = [f"{k} {vehicle_info[k]:.3f} m in vehicle_info, {theirs[k]:.3f} m in {model}'s "
           f"physics" for k in GEOMETRY_KEYS
           if k in theirs and k in vehicle_info
           and abs(vehicle_info[k] - theirs[k]) > GEOMETRY_TOLERANCE]
    if bad:
        raise VehicleMismatch(
            f"the ego's vehicle description is not {model}: " + "; ".join(bad)
            + f" (tolerance {GEOMETRY_TOLERANCE * 100:.0f} cm). Point the ego YAML's "
            "`vehicle_info` at this model's, or `model` at the car it describes.")


def bicycle_limit(inner_limit, wheel_base, wheel_tread):
    """The bicycle tire angle at which the inner wheel reaches `inner_limit` (rad), under
    vehicle.py's Ackermann split: tan(inner) = L / (R - t/2), tan(bicycle) = L / R."""
    if inner_limit <= 0:
        return 0.0
    if inner_limit >= math.pi / 2 - 1e-6:
        return math.atan(wheel_base / (wheel_tread / 2))
    radius = wheel_base / math.tan(inner_limit) + wheel_tread / 2
    return math.atan(wheel_base / radius)


def physics_params(physics):
    """VehicleParams fields from a catalogue entry's `physics` (dump_vehicle_physics.py)."""
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
