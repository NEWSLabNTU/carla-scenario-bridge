"""Step responses of vehicle.tesla.model3 on CARLA 0.9.16 through acb_bridge's actuation
(vehicle_control.rs / longitudinal_map.rs, replicated here): pedal maps, Ackermann steer
inverse, steering-curve compensation, handbrake hold. Timing as csb: 0.05 s ticks, physics
substeps of 0.05/16.

    python3 carla_step.py OUT_DIR [TOWN]
"""
import csv
import math
import sys
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).parent))
from step_common import DT, TESTS, command  # noqa: E402

ACB = (Path(__file__).resolve().parents[3] / "autoware_carla_bridge/src/acb_vehicle_launch/"
       "acb_vehicle_description/config")
OUT = Path(sys.argv[1])
TOWN = sys.argv[2] if len(sys.argv) > 2 else "Town06"
OUT.mkdir(parents=True, exist_ok=True)


class PedalMap:
    def __init__(self, path):
        rows = [l.strip() for l in open(path) if l.strip() and not l.startswith("#")]
        self.speeds = [float(x) for x in rows[0].split(",")[1:]]
        self.pedals, self.accel = [], []
        for r in rows[1:]:
            c = [float(x) for x in r.split(",")]
            self.pedals.append(c[0])
            self.accel.append(c[1:])

    def at(self, i, v):
        row, s = self.accel[i], self.speeds
        for j, sj in enumerate(s):
            if sj >= v:
                if j == 0:
                    return row[0]
                t = (v - s[j - 1]) / (sj - s[j - 1])
                return row[j - 1] + t * (row[j] - row[j - 1])
        return row[-1]

    def pedal_for(self, target, v, rising):
        last = len(self.pedals) - 1
        at = lambda i: self.at(i, v)  # noqa: E731
        if (target >= at(last)) if rising else (target <= at(last)):
            return self.pedals[last]
        if (target <= at(0)) if rising else (target >= at(0)):
            return self.pedals[0]
        for i in range(1, last + 1):
            a0, a1 = at(i - 1), at(i)
            if (a0 <= target <= a1) if rising else (a1 <= target <= a0):
                if abs(a1 - a0) < 1e-12:
                    return self.pedals[i - 1]
                return self.pedals[i - 1] + (target - a0) / (a1 - a0) * (self.pedals[i] - self.pedals[i - 1])
        return self.pedals[min(range(last + 1), key=lambda i: abs(at(i) - target))]


ACCEL, BRAKE = PedalMap(ACB / "accel_map.csv"), PedalMap(ACB / "brake_map.csv")


def pedals(a, v):
    brake_coast = BRAKE.at(0, v)
    coasting = max(ACCEL.at(0, v), brake_coast)
    if a >= coasting:
        return min(max(ACCEL.pedal_for(a, v, True), 0.0), 1.0), 0.0
    return 0.0, min(max(BRAKE.pedal_for(brake_coast + (a - coasting), v, False), 0.0), 1.0)


def curve_factor(curve, kmh):
    if kmh <= curve[0][0]:
        return curve[0][1]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if kmh <= x1:
            return y0 + (y1 - y0) * (kmh - x0) / (x1 - x0)
    return curve[-1][1]


def effective(cmd, max_steer, tow):
    inner = min(abs(cmd), 1.0) * max_steer
    if inner <= 1e-7:
        return 0.0
    outer = math.atan(1.0 / (1.0 / math.tan(inner) + tow))
    return math.copysign(0.5 * (inner + outer), cmd)


def steer_cmd_for(angle, max_steer, tow):
    if abs(angle) < 1e-7:
        return 0.0
    if abs(angle) >= effective(1.0, max_steer, tow):
        return math.copysign(1.0, angle)
    lo, hi = 0.0, 1.0
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if effective(mid, max_steer, tow) < abs(angle) else (lo, mid)
    return math.copysign(0.5 * (lo + hi), angle)


client = carla.Client("localhost", 2000)
client.set_timeout(300.0)
world = client.get_world()
if TOWN not in world.get_map().name:
    world = client.load_world(TOWN)
if world.get_settings().synchronous_mode:
    sys.exit("CARLA is already synchronous")
orig = world.get_settings()
s = world.get_settings()
s.synchronous_mode, s.fixed_delta_seconds, s.substepping = True, DT, True
s.max_substep_delta_time, s.max_substeps = DT / 16, 16
world.apply_settings(s)
cmap = world.get_map()


def straight_spawn():
    """A spawn point in a rightmost lane with 200 m of straight road ahead and two lanes of
    the same direction to its left."""
    best = None
    for sp in cmap.get_spawn_points():
        wp = cmap.get_waypoint(sp.location)
        left = wp.get_left_lane()
        if not left or left.lane_type != carla.LaneType.Driving or left.lane_id * wp.lane_id <= 0:
            continue
        ll = left.get_left_lane()
        if not ll or ll.lane_type != carla.LaneType.Driving or ll.lane_id * wp.lane_id <= 0:
            continue
        r = wp.get_right_lane()
        if r and r.lane_type == carla.LaneType.Driving:
            continue
        ahead = wp.next(200.0)
        if not ahead:
            continue
        dyaw = abs((ahead[0].transform.rotation.yaw - wp.transform.rotation.yaw + 180) % 360 - 180)
        if dyaw < 1.0:
            best = wp.transform
            break
    return best


start = straight_spawn()
if start is None:
    world.apply_settings(orig)
    sys.exit("no straight spawn")
start.location.z += 0.3
print("start", start)
bp = world.get_blueprint_library().find("vehicle.tesla.model3")
actor = world.spawn_actor(bp, start)
try:
    for _ in range(5):
        world.tick()
    pc = actor.get_physics_control()
    max_steer = math.radians(pc.wheels[0].max_steer_angle)
    tf = actor.get_transform()
    pos = [(w.position.x / 100, w.position.y / 100) for w in pc.wheels]
    track = math.dist(pos[0], pos[1])
    wb = math.dist(((pos[0][0] + pos[1][0]) / 2, (pos[0][1] + pos[1][1]) / 2),
                   ((pos[2][0] + pos[3][0]) / 2, (pos[2][1] + pos[3][1]) / 2))
    tow = track / wb
    curve = sorted((p.x, p.y) for p in pc.steering_curve)
    print(f"max steer {math.degrees(max_steer):.1f} deg, track/wheelbase {tow:.4f}, curve {curve}")
    for test, length in [("accel_brake", 8.0)] + list(TESTS.items()):  # the first: a warm-up
        actor.set_target_velocity(carla.Vector3D(0, 0, 0))
        actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        actor.set_transform(start)
        for _ in range(40):
            actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            world.tick()
        rows = []
        t = 0.0
        v = 0.0
        while t < length - 1e-9:
            tf = actor.get_transform()
            vel = actor.get_velocity()
            fwd = tf.get_forward_vector()
            v = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z
            speed = math.hypot(vel.x, vel.y)
            a, vcmd, angle = command(test, t, v)
            throttle, brake = pedals(a, speed)
            factor = curve_factor(curve, abs(v) * 3.6)
            steer = max(-1.0, min(1.0, -steer_cmd_for(angle, max_steer, tow) / factor))
            hand = abs(vcmd) <= 0.01 and a <= 0.01 and speed < 0.1
            actor.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer,
                                                     hand_brake=hand))
            world.tick()
            t += DT
            tf = actor.get_transform()
            vel = actor.get_velocity()
            fwd = tf.get_forward_vector()
            v = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z
            yaw_rate = -math.radians(actor.get_angular_velocity().z)  # ROS: left positive
            loc = tf.location
            rows.append((round(t, 3), v, yaw_rate, a, angle, throttle, brake, steer, loc.x, -loc.y))
        if test == "accel_brake" and length == 8.0:
            print("warm-up run done", flush=True)
            continue
        with open(OUT / f"carla_{test}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "v", "yaw_rate", "cmd_accel", "cmd_tire_angle", "throttle", "brake",
                        "steer", "x", "y"])
            w.writerows(rows)
        print(test, "done; final v %.2f" % v, flush=True)
finally:
    for a in world.get_actors():
        if a.parent is not None and a.parent.id == actor.id:
            a.destroy()
    actor.destroy()
    world.tick()
    world.apply_settings(orig)
