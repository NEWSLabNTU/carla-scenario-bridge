"""Frames and transforms for the ego's sensors (roadmap 038). Pure Python: no Isaac.

Quaternions are (w, x, y, z); a transform is (translation, quaternion), parent to child.
"""
import math

import yaml

EARTH_RADIUS = 6378137.0  # m, WGS84 equatorial


def rpy_quat(roll, pitch, yaw):
    """URDF fixed-joint rpy (R = Rz(yaw) Ry(pitch) Rx(roll)) as (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qrot(q, v):
    w, x, y, z = q
    p = qmul(qmul(q, (0.0, *v)), (w, -x, -y, -z))
    return p[1:]


def compose(a, b):
    """Transform a then b, each (translation, quaternion): parent->a, a->b gives parent->b."""
    (ta, qa), (tb, qb) = a, b
    r = qrot(qa, tb)
    return (tuple(x + y for x, y in zip(ta, r)), qmul(qa, qb))


def kit_transform(sensors_calibration, sensor_kit_calibration, link):
    """base_link -> `link` (a sensor_kit_base_link child) from the two calibration files."""
    def tf(entry):
        return ((entry["x"], entry["y"], entry["z"]),
                rpy_quat(entry["roll"], entry["pitch"], entry["yaw"]))

    base = yaml.safe_load(open(sensors_calibration))["base_link"]["sensor_kit_base_link"]
    kit = yaml.safe_load(open(sensor_kit_calibration))["sensor_kit_base_link"][link]
    return compose(tf(base), tf(kit))


def lat_lon(x, y):
    """Map x/y (east, north, m) to WGS84 degrees about origin (0, 0): the map's projector is
    TransverseMercator at (0, 0), which within a town of the origin is this to millimetres."""
    lat = math.degrees(y / EARTH_RADIUS)
    lon = math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(lat))))
    return lat, lon
