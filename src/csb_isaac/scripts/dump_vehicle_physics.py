#!/usr/bin/env python3
"""CARLA's vehicle physics into the Isaac vehicle catalogue (roadmap 043, "Vehicle model").

Connects to a running CARLA 0.9.16 (starts nothing), spawns each blueprint listed in the
catalogue's `vehicles.json`, reads `get_physics_control()` and writes it into that entry under
`"physics"`, in the catalogue's frame (the CARLA actor origin, metres, x forward, y left: CARLA's
Y flipped). Also writes `physics.json` beside it (blueprint id -> physics): `make_assets.py`
regenerates `vehicles.json` without it, and `vehicle_assets.py` falls back to the sidecar.

    dump_vehicle_physics.py [--root /mnt/disk1/carla-usd/vehicles] [--host H] [--port 2000]
                            [--only vehicle.tesla.model3,...]

What CARLA gives, and what it means here:

- `mass` kg; `drag_coefficient`; `moi` (the engine's, kg m^2), `max_rpm`, `torque_curve`
  [(rpm, N m)], `final_ratio`, `forward_gears`, `steering_curve` [(km/h, factor)]: as given.
- `center_of_mass`: CARLA's *nudge* of the centre of mass from the one UE computes from the
  collision bodies (`BodyInstance.COMNudge`), metres. The absolute centre is not exposed;
  `com_estimate` is the nudge added to the centre of CARLA's bounding box -- an estimate.
- per wheel: `tire_friction`, `damping_rate`, `max_steer_angle` (deg, the wheel's own limit),
  `radius` m (CARLA: cm), `max_brake_torque` / `max_handbrake_torque` N m,
  `lat_stiff_max_load` (rest loads), `lat_stiff_value` (N/rad per N of rest load),
  `long_stiff_value` (N per unit slip per g: kg), and `position`, the wheel centre relative to
  the actor (CARLA reports it in world centimetres).
- Suspension: not exposed by CARLA 0.9.16's Python API; nothing is written for it.

Derived, for the ego's geometry check: `wheel_base` (front to rear axle), `wheel_tread` (the
front wheels' spacing), `wheel_radius` (the front wheels'), `rear_axle_x`.

Each actor is destroyed with its children first (none are attached here; CLAUDE.md: never
destroy a vehicle with sensors on it). The world is put in synchronous mode for the duration
and restored; refuses if another client already owns the tick.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import carla


def v2(points):
    return [[round(p.x, 4), round(p.y, 4)] for p in points]


def to_actor(transform, world_cm):
    """A world point in centimetres -> the actor frame, metres, Y flipped (ours)."""
    m = transform.get_inverse_matrix()
    x, y, z = world_cm.x / 100.0, world_cm.y / 100.0, world_cm.z / 100.0
    lx = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3]
    ly = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3]
    lz = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]
    return [round(lx, 4), round(-ly, 4), round(lz, 4)]


def physics_of(actor, server):
    pc = actor.get_physics_control()
    tf = actor.get_transform()
    bb = actor.bounding_box
    wheels = []
    for w in pc.wheels:
        wheels.append({
            "tire_friction": round(w.tire_friction, 4),
            "damping_rate": round(w.damping_rate, 4),
            "max_steer_angle_deg": round(w.max_steer_angle, 3),
            "radius": round(w.radius / 100.0, 4),
            "max_brake_torque": round(w.max_brake_torque, 2),
            "max_handbrake_torque": round(w.max_handbrake_torque, 2),
            "lat_stiff_max_load": round(w.lat_stiff_max_load, 4),
            "lat_stiff_value": round(w.lat_stiff_value, 4),
            "long_stiff_value": round(w.long_stiff_value, 2),
            "position": to_actor(tf, w.position),
        })
    nudge = [round(pc.center_of_mass.x, 4), round(-pc.center_of_mass.y, 4),
             round(pc.center_of_mass.z, 4)]
    centre = [bb.location.x, -bb.location.y, bb.location.z]
    out = {
        "mass": round(pc.mass, 2),
        "drag_coefficient": round(pc.drag_coefficient, 4),
        "center_of_mass_nudge": nudge,
        "com_estimate": [round(a + b, 4) for a, b in zip(centre, nudge)],
        "moi": round(pc.moi, 4),
        "max_rpm": round(pc.max_rpm, 1),
        "torque_curve": v2(pc.torque_curve),
        "steering_curve": v2(pc.steering_curve),
        "final_ratio": round(pc.final_ratio, 4),
        "forward_gears": [{"ratio": round(g.ratio, 4), "down_ratio": round(g.down_ratio, 4),
                           "up_ratio": round(g.up_ratio, 4)} for g in pc.forward_gears],
        "use_gear_autobox": bool(pc.use_gear_autobox),
        "gear_switch_time": round(pc.gear_switch_time, 4),
        "clutch_strength": round(pc.clutch_strength, 4),
        "damping_rate_full_throttle": round(pc.damping_rate_full_throttle, 4),
        "damping_rate_zero_throttle_clutch_engaged":
            round(pc.damping_rate_zero_throttle_clutch_engaged, 4),
        "damping_rate_zero_throttle_clutch_disengaged":
            round(pc.damping_rate_zero_throttle_clutch_disengaged, 4),
        "wheels": wheels,
        "frame": "CARLA actor origin, m, x forward, y left (CARLA's Y flipped)",
        "source": f"CARLA {server} get_physics_control()",
    }
    if len(wheels) == 4:
        # CARLA's order: front left, front right, rear left, rear right
        fl, fr, rl, rr = (w["position"] for w in wheels)
        front_x, rear_x = (fl[0] + fr[0]) / 2, (rl[0] + rr[0]) / 2
        out["wheel_base"] = round(front_x - rear_x, 4)
        out["wheel_tread"] = round(math.dist(fl[:2], fr[:2]), 4)
        out["wheel_radius"] = wheels[0]["radius"]
        out["rear_axle_x"] = round(rear_x, 4)
    return out


def destroy(world, actor):
    for a in world.get_actors():
        if a.parent is not None and a.parent.id == actor.id:
            a.destroy()
    actor.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default="/mnt/disk1/carla-usd/vehicles")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--only", default="", help="comma-separated blueprint ids")
    args = ap.parse_args()
    root = Path(args.root)
    index = root / "vehicles.json"
    rows = json.loads(index.read_text())
    only = set(filter(None, args.only.split(",")))

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    if world.get_settings().synchronous_mode:
        sys.exit("CARLA is already synchronous: another client owns the tick. Stop it first.")
    orig = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    world.apply_settings(s)
    library = world.get_blueprint_library()
    spawn = world.get_map().get_spawn_points()[0]
    spawn.location.z += 0.5
    sidecar_path = root / "physics.json"
    sidecar = json.loads(sidecar_path.read_text()) if sidecar_path.exists() else {}
    done = 0
    try:
        for row in rows:
            bp_id = row["id"]
            if only and bp_id not in only:
                continue
            try:
                bp = library.find(bp_id)
            except IndexError:
                print(f"{bp_id}: no such blueprint on this server, skipped")
                continue
            actor = world.try_spawn_actor(bp, spawn)
            if actor is None:
                print(f"{bp_id}: could not spawn (spawn point occupied?), skipped")
                continue
            try:
                for _ in range(3):
                    world.tick()
                phys = physics_of(actor, client.get_server_version())
            finally:
                destroy(world, actor)
                world.tick()
            row["physics"] = phys
            sidecar[bp_id] = phys
            done += 1
            print(f"{bp_id}: mass {phys['mass']} kg, {len(phys['wheels'])} wheels"
                  + (f", wheel base {phys['wheel_base']} m, tread {phys['wheel_tread']} m"
                     if "wheel_base" in phys else ""), flush=True)
    finally:
        world.apply_settings(orig)
    index.write_text(json.dumps(rows, indent=1))
    sidecar_path.write_text(json.dumps(sidecar, indent=1))
    print(f"{done} vehicle(s) -> {index}, {sidecar_path}")


if __name__ == "__main__":
    main()
