"""Tracked bodies' arithmetic (roadmap 044), pure Python so it is tested without Isaac.

A tracked entity is a dynamic rigid body whose Xform sits at the SSv2 entity origin (the rear
axle, for a vehicle) while PhysX's linear velocity is its centre of mass's, at the declared
box centre. csb speaks of the origin's velocity, PhysX of the centre's; `v_com = v_o + w x r`
with `r` the centre's offset turned into the world frame, and back.

Quaternions are `(w, x, y, z)`, vectors `(x, y, z)`, world frame.
"""
import math

# Mass of a body whose CARLA blueprint has no catalogue entry, by kind, kg.
DEFAULT_MASS = {"VEHICLE": 1500.0, "EGO": 1500.0, "PEDESTRIAN": 80.0, "MISC_OBJECT": 100.0}
# A two-wheeler's, rider included, kg (CARLA's bicycles weigh 1 kg in physics.json: their
# physics is not a car's, and a 1 kg box would fly off the first kerb).
TWO_WHEELER_MASS = 200.0
TWO_WHEELERS = {"bicycle", "motorcycle"}


def rotate(q, v):
    """`v` turned by the unit quaternion `q`."""
    w, x, y, z = q
    # v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    cx = y * v[2] - z * v[1]
    cy = z * v[0] - x * v[2]
    cz = x * v[1] - y * v[0]
    return (v[0] + 2 * (w * cx + y * cz - z * cy),
            v[1] + 2 * (w * cy + z * cx - x * cz),
            v[2] + 2 * (w * cz + x * cy - y * cx))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def com_velocity(v_origin, w, q, com_body):
    """The centre of mass's linear velocity for an origin moving at `v_origin` and turning at
    `w`, with the centre at `com_body` (body frame) and the body at attitude `q`."""
    r = rotate(q, com_body)
    c = cross(w, r)
    return (v_origin[0] + c[0], v_origin[1] + c[1], v_origin[2] + c[2])


def origin_velocity(v_com, w, q, com_body):
    """The inverse of `com_velocity`: the origin's linear velocity."""
    r = rotate(q, com_body)
    c = cross(w, r)
    return (v_com[0] - c[0], v_com[1] - c[1], v_com[2] - c[2])


def body_mass(kind, subtype, physics):
    """The mass a tracked body gets: CARLA's for its blueprint (`physics`, the catalogue's
    entry, or None), else by kind; a two-wheeler's is `TWO_WHEELER_MASS` whatever CARLA's."""
    if subtype in TWO_WHEELERS:
        return TWO_WHEELER_MASS
    mass = (physics or {}).get("mass")
    if mass and math.isfinite(mass) and mass > 10.0:
        return float(mass)
    return DEFAULT_MASS.get(kind, DEFAULT_MASS["VEHICLE"])


def yaw_of(q):
    """Yaw (rad) of the quaternion `q`."""
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def roll_pitch_of(q):
    """Roll and pitch (rad) of `q`."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def normalized(q):
    """`q` scaled to unit length (the identity for a zero quaternion)."""
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q) if n > 0 else (1.0, 0.0, 0.0, 0.0)


# A tracked body's skids (044 step 2): its box on a flat bottom caught on every seam and kerb
# edge of a town's triangle-mesh road (pitched up, launched, stopped dead), so it rides on two
# capsules along its length, left and right, whose rounded ends climb a step lower than
# their radius; the box's bottom is raised by the radius, and stays the collider above.
SKID_RADIUS = 0.25


def skids(center, dims, radius=SKID_RADIUS):
    """`(box_center, box_dims, [(capsule_centre, half_height, radius), ...])` for a declared
    box: the box with its bottom raised by the skid radius, and two capsules along x whose
    bottoms are the declared box's bottom. A box too small for skids keeps its shape and gets
    none."""
    cx, cy, cz = center
    length, width, height = dims
    r = min(radius, height / 4, width / 4, length / 4)
    if r < 0.1 or width / 2 - r - 0.05 <= 0:
        return tuple(center), tuple(dims), []
    bottom = cz - height / 2
    box_center = (cx, cy, cz + r / 2)
    box_dims = (length, width, height - r)
    half_height = max(length / 2 - r - 0.05, 0.0)
    y = width / 2 - r - 0.05
    caps = [((cx, cy + side * y, bottom + r), half_height, r) for side in (1, -1)]
    return box_center, box_dims, caps
