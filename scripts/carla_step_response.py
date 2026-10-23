#!/usr/bin/env python3
"""The Model 3's step responses on CARLA, controlled directly (roadmap 051, K0 and K3).

    scripts/carla_step_response.py [--port 2010] [--blueprint vehicle.tesla.model3]
        [--physics carla_physics.yaml] --out result.json

What `vehicle_check`'s dynamics run will measure through Autoware, measured here on CARLA's
own controls, so K3 (CARLA on the spec's physics) has a before and after:

- throttle steps 0.3 / 0.6 / 1.0 from rest, 8 s each: speed at 1, 2, 4, 8 s, peak
  acceleration (0.25 s window);
- brake steps 0.3 / 0.6 / 1.0 from 10 m/s: mean deceleration, stopping distance and time;
- steering 0.05 / 0.1 / 0.15 at 5 m/s, 2 s each: steady yaw rate and the curvature it gives.

It takes CARLA over synchronous (0.05 s, 16 substeps of 3.125 ms, as the world server runs
it), so nothing else may drive this CARLA meanwhile; it restores the settings and destroys
its car at the end. `--physics` applies a carla_physics.yaml first (K3). Throwaway: deleted
when `vehicle_check` lands.
"""

import argparse
import json
import math
import sys

import carla

DT = 0.05
STRAIGHT_M = 160.0


def speed(v):
    return math.hypot(v.x, v.y)


def straight_spawn(world):
    """A spawn point with STRAIGHT_M of lane ahead turning less than 2 degrees."""
    m = world.get_map()
    for sp in m.get_spawn_points():
        wp = m.get_waypoint(sp.location)
        yaw0, ok, run = wp.transform.rotation.yaw, True, 0.0
        while run < STRAIGHT_M:
            nxt = wp.next(5.0)
            if len(nxt) != 1:
                ok = False
                break
            wp = nxt[0]
            run += 5.0
            if abs((wp.transform.rotation.yaw - yaw0 + 180) % 360 - 180) > 2.0:
                ok = False
                break
        if ok:
            return sp
    raise SystemExit("no straight spawn point")


class Rig:
    def __init__(self, world, bp, start, physics):
        self.world, self.start = world, start
        start.location.z += 0.3
        self.car = world.spawn_actor(bp, start)
        if physics:
            apply_physics(self.car, physics)
        self.tick(20)

    def tick(self, n=1):
        for _ in range(n):
            self.world.tick()

    def reset(self):
        self.car.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        self.car.set_target_velocity(carla.Vector3D())
        self.car.set_target_angular_velocity(carla.Vector3D())
        self.car.set_transform(self.start)
        self.tick(40)
        self.car.apply_control(carla.VehicleControl(brake=1.0))
        self.tick(20)

    def run(self, control_of, seconds):
        """[(t, speed, yaw_rate_deg_s, x, y)] while `control_of(t, v)` drives."""
        out, t = [], 0.0
        while t < seconds:
            v = speed(self.car.get_velocity())
            self.car.apply_control(control_of(t, v))
            self.tick()
            t += DT
            tf = self.car.get_transform()
            out.append((round(t, 3), v, self.car.get_angular_velocity().z, tf.location.x,
                        tf.location.y))
        return out


def apply_physics(car, path):
    import yaml
    spec = yaml.safe_load(open(path))
    pc = car.get_physics_control()
    for k, v in spec.get("vehicle", {}).items():
        setattr(pc, k, v)
    car.apply_physics_control(pc)


def at(trace, t):
    return min(trace, key=lambda r: abs(r[0] - t))[1]


def peak_accel(trace, window=0.25):
    n = max(1, int(round(window / DT)))
    return max((trace[i + n][1] - trace[i][1]) / (n * DT) for i in range(len(trace) - n))


def throttle_steps(rig):
    out = {}
    for th in (0.3, 0.6, 1.0):
        rig.reset()
        tr = rig.run(lambda t, v: carla.VehicleControl(throttle=th), 8.0)
        out[str(th)] = {f"v_{s}s": round(at(tr, s), 3) for s in (1, 2, 4, 8)}
        out[str(th)]["peak_accel"] = round(peak_accel(tr), 3)
    return out


def brake_steps(rig):
    out = {}
    for br in (0.3, 0.6, 1.0):
        rig.reset()
        rig.run(lambda t, v: carla.VehicleControl(throttle=1.0), 0.0)
        tr = []
        # Up to 10 m/s, then the brake step until standing.
        while True:
            tr = rig.run(lambda t, v: carla.VehicleControl(throttle=1.0), DT)
            if tr[-1][1] >= 10.0:
                break
        v0 = speed(rig.car.get_velocity())
        p0 = rig.car.get_transform().location
        tr = rig.run(lambda t, v: carla.VehicleControl(brake=br), 15.0)
        stop = next((r for r in tr if r[1] < 0.05), tr[-1])
        out[str(br)] = {
            "v0": round(v0, 3),
            "stop_time": stop[0],
            "stop_distance": round(math.hypot(stop[3] - p0.x, stop[4] - p0.y), 3),
            "mean_decel": round(v0 / stop[0], 3) if stop[0] > 0 else None,
        }
    return out


def steering(rig):
    out = {}
    for st in (0.05, 0.1, 0.15):
        rig.reset()

        def hold5(t, v, steer=0.0):
            th = max(0.0, min(1.0, 0.5 * (5.0 - v) + 0.25))
            return carla.VehicleControl(throttle=th if v < 5.0 else 0.15, steer=steer)

        rig.run(hold5, 4.0)
        tr = rig.run(lambda t, v: hold5(t, v, st), 2.0)
        tail = tr[len(tr) // 2:]
        v = sum(r[1] for r in tail) / len(tail)
        yaw = sum(r[2] for r in tail) / len(tail)  # deg/s, CARLA's (left-handed) sign
        out[str(st)] = {"speed": round(v, 3), "yaw_rate_deg_s": round(abs(yaw), 3),
                        "curvature": round(abs(math.radians(yaw)) / v, 4) if v > 0.5 else None}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2010)
    ap.add_argument("--blueprint", default="vehicle.tesla.model3")
    ap.add_argument("--physics")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    old = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, DT
    s.substepping, s.max_substep_delta_time, s.max_substeps = True, 0.003125, 16
    world.apply_settings(s)
    rig = None
    try:
        bp = world.get_blueprint_library().find(args.blueprint)
        bp.set_attribute("role_name", "step_response")
        rig = Rig(world, bp, straight_spawn(world), args.physics)
        pc = rig.car.get_physics_control()
        result = {
            "blueprint": args.blueprint,
            "physics": args.physics,
            "mass": pc.mass,
            "throttle": throttle_steps(rig),
            "brake": brake_steps(rig),
            "steering": steering(rig),
        }
    finally:
        if rig is not None:
            rig.car.destroy()
        world.apply_settings(old)
    json.dump(result, open(args.out, "w"), indent=1)
    print(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
