"""2D oriented-box geometry for the gap oracle (roadmap 024).

Library module, not an executable: `gap_oracle` and the tests import it from this
directory. Pure Python, stdlib only, no ROS or CARLA.

Everything is in the ROS map frame (x east, y north, right-handed, yaw in radians,
counter-clockwise), which is the frame the gap trace records both worlds in.

The two geometries the oracle judges each frame in:

  - SSv2's: `ssv2_box(entity_pose, declared_center, declared_dims)`. The entity pose is
    SSv2's origin (the rear axle for vehicles); the declared `bounding_box.center` shifts
    the box in the entity frame. This is the polygon SSv2's `checkCollision2D` builds
    (common/math/geometry: `toPolygon2D`), and `z_separated` is the height gate it applies
    before it.
  - CARLA's: `carla_box(actor_pose, blueprint_location, blueprint_extent)`. The actor pose
    is CARLA's transform (the body origin) converted to the ROS frame; the blueprint box is
    `actor.bounding_box()` in the actor's own frame, also ROS-handed.

`actor_pose_from_entity` is the bridge's `entity_origin_to_actor_origin` plus the
spawn-time turn (`OriginOffset`): the pose CARLA should report for an SSv2 pose. The trace
is raw and the oracle applies the offset once, so an offset the bridge applied twice or not
at all shows as a pose error of the offset's length.

`signed_distance` is the margin: positive is the clearance between the boxes, negative the
penetration depth (the shortest translation that separates them), zero touching. Touching
counts as overlapping, as boost::geometry::intersects does in SSv2's check.
"""

import math
from dataclasses import dataclass

# Below this a margin is "touching": float noise on boxes built from rotated poses.
TOUCH_TOLERANCE = 1e-9


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # rad, ROS (counter-clockwise from +x)


@dataclass(frozen=True)
class OrientedBox:
    cx: float
    cy: float
    yaw: float  # rad
    half_length: float  # along the box's own x
    half_width: float  # along the box's own y

    def axes(self):
        """The box's unit x and y axes in the map frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c, s), (-s, c)

    def corners(self):
        """Corners, counter-clockwise from front-left."""
        (ux, uy), (vx, vy) = self.axes()
        hl, hw = self.half_length, self.half_width
        return [
            (self.cx + sl * hl * ux + sw * hw * vx, self.cy + sl * hl * uy + sw * hw * vy)
            for sl, sw in ((1, 1), (-1, 1), (-1, -1), (1, -1))
        ]


def _rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def box_at(pose, center_x, center_y, length, width):
    """A `length` x `width` box whose centre sits at (center_x, center_y) in `pose`'s frame."""
    dx, dy = _rotate(center_x, center_y, pose.yaw)
    return OrientedBox(pose.x + dx, pose.y + dy, pose.yaw, length / 2.0, width / 2.0)


def ssv2_box(entity_pose, declared_center, declared_dims):
    """SSv2's box: `declared_center` (x, y[, z]) and `declared_dims` (length, width[, height])."""
    return box_at(entity_pose, declared_center[0], declared_center[1], *declared_dims[:2])


def carla_box(actor_pose, location, extent):
    """CARLA's box: blueprint `location` (x, y[, z]) and half-size `extent` (x, y[, z])."""
    return box_at(actor_pose, location[0], location[1], 2.0 * extent[0], 2.0 * extent[1])


def actor_pose_from_entity(entity_pose, offset_x, offset_y, offset_yaw):
    """Where the bridge puts the CARLA actor for an SSv2 entity pose (OriginOffset)."""
    dx, dy = _rotate(offset_x, offset_y, entity_pose.yaw)
    return Pose2D(entity_pose.x + dx, entity_pose.y + dy, entity_pose.yaw + offset_yaw)


def entity_pose_from_actor(actor_pose, offset_x, offset_y, offset_yaw):
    """The inverse of `actor_pose_from_entity`."""
    yaw = actor_pose.yaw - offset_yaw
    dx, dy = _rotate(offset_x, offset_y, yaw)
    return Pose2D(actor_pose.x - dx, actor_pose.y - dy, yaw)


def wrap_angle(a):
    """`a` in [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_error(a, b):
    """(in-plane distance m, absolute yaw difference rad) between two poses."""
    return math.hypot(a.x - b.x, a.y - b.y), abs(wrap_angle(a.yaw - b.yaw))


def _project(corners, ax, ay):
    ps = [x * ax + y * ay for x, y in corners]
    return min(ps), max(ps)


def _point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _polygon_distance(pa, pb):
    """Distance between two disjoint convex polygons: the closest pair always has a vertex
    of one on an edge of the other."""
    best = math.inf
    for poly, other in ((pa, pb), (pb, pa)):
        edges = list(zip(other, other[1:] + other[:1]))
        for px, py in poly:
            for (ax, ay), (bx, by) in edges:
                best = min(best, _point_segment_distance(px, py, ax, ay, bx, by))
    return best


def signed_distance(a, b):
    """Clearance between boxes `a` and `b` (> 0), or minus their penetration depth (< 0).

    Separating axes: the four edge normals of two rectangles. If any separates them the
    boxes are disjoint and the clearance is the Euclidean distance between the polygons (the
    separating gap alone undercounts a corner-to-corner approach). Otherwise the depth is
    the smallest translation along any of those axes that pulls them apart, which for two
    convex polygons is the exact penetration depth.
    """
    ca, cb = a.corners(), b.corners()
    depth = math.inf
    for ax, ay in (*a.axes(), *b.axes()):
        mina, maxa = _project(ca, ax, ay)
        minb, maxb = _project(cb, ax, ay)
        if maxa < minb or maxb < mina:
            return _polygon_distance(ca, cb)
        depth = min(depth, maxa - minb, maxb - mina)
    return -depth if depth > TOUCH_TOLERANCE else 0.0


def overlaps(a, b):
    """True when the boxes overlap or touch (boost::geometry::intersects)."""
    return signed_distance(a, b) <= TOUCH_TOLERANCE


def overlaps_in_one_only(d_ssv2, d_carla):
    """C1 broken: one geometry has the pair overlapping and the other has it clear."""
    return (d_ssv2 <= TOUCH_TOLERANCE) != (d_carla <= TOUCH_TOLERANCE)


def z_separated(z0, center_z0, height0, z1, center_z1, height1):
    """SSv2's height gate: True when the boxes cannot meet in z, so no 2D check is made."""
    return abs((z0 + center_z0) - (z1 + center_z1)) > abs(height0 + height1) * 0.5
