#!/usr/bin/env python3
"""Probe a CARLA 0.10 server for what the bridge stack depends on (roadmap 019 step 6).

Reports: client/server version, maps, vehicle blueprints, synchronous ticking time with an
Autoware-like sensor set (1 lidar + N cameras), the steering-angle getter, the vehicle
physics control (Chaos in 0.10), traffic lights, and saves one camera frame.

Usage: carla010_probe.py --port 2100 [--map Town10HD_Opt] [--cameras 6] [--out DIR]
Run with the 0.10 client wheel (a venv built from PythonAPI/carla/dist).
"""
import argparse
import json
import os
import queue
import statistics
import time

import carla


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2100)
    ap.add_argument("--map", default="Town10HD_Opt")
    ap.add_argument("--cameras", type=int, default=6)
    ap.add_argument("--ticks", type=int, default=200)
    ap.add_argument("--out", default=".")
    ap.add_argument("--throttle", type=float, default=0.4)
    ap.add_argument("--steer", type=float, default=0.3)
    ap.add_argument("--cam-size", default="1920x1080")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rep = {}

    c = carla.Client(a.host, a.port)
    c.set_timeout(120.0)
    rep["client_version"] = c.get_client_version()
    rep["server_version"] = c.get_server_version()
    rep["maps"] = sorted(c.get_available_maps())
    w = c.get_world()
    if not w.get_map().name.endswith(a.map):
        t0 = time.time()
        w = c.load_world(a.map)
        rep["load_world_s"] = round(time.time() - t0, 1)
    rep["map"] = w.get_map().name
    bl = w.get_blueprint_library()
    rep["vehicle_blueprints"] = sorted(b.id for b in bl.filter("vehicle.*"))
    rep["sensor_blueprints"] = sorted(b.id for b in bl.filter("sensor.*"))
    rep["traffic_lights"] = len(w.get_actors().filter("traffic.traffic_light*"))
    rep["spawn_points"] = len(w.get_map().get_spawn_points())

    orig = w.get_settings()
    s = w.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    w.apply_settings(s)
    actors = []
    try:
        vbp = None
        for name in ("vehicle.lincoln.mkz", "vehicle.tesla.model3", "vehicle.taxi.ford"):
            if bl.filter(name):
                vbp = bl.filter(name)[0]
                break
        vbp = vbp or bl.filter("vehicle.*")[0]
        rep["ego_blueprint"] = vbp.id
        sp = w.get_map().get_spawn_points()[0]
        v = w.spawn_actor(vbp, sp)
        actors.append(v)
        w.tick()

        pc = v.get_physics_control()
        rep["physics_control"] = {
            "mass": pc.mass,
            "max_rpm": getattr(pc, "max_rpm", None),
            "wheels": [{k: getattr(wh, k) for k in dir(wh)
                        if not k.startswith("_") and isinstance(getattr(wh, k), (int, float))}
                       for wh in pc.wheels],
            "fields": sorted(k for k in dir(pc) if not k.startswith("_")),
        }

        q = queue.Queue()
        lbp = bl.find("sensor.lidar.ray_cast")
        lbp.set_attribute("channels", "128")
        lbp.set_attribute("range", "200")
        lbp.set_attribute("points_per_second", "2600000")
        lbp.set_attribute("rotation_frequency", "20")
        lid = w.spawn_actor(lbp, carla.Transform(carla.Location(z=2.0)), attach_to=v)
        actors.append(lid)
        lid.listen(lambda d: q.put(("lidar", d.frame, len(d))))
        cams = []
        for i in range(a.cameras):
            cbp = bl.find("sensor.camera.rgb")
            cw, chh = a.cam_size.split("x")
            cbp.set_attribute("image_size_x", cw)
            cbp.set_attribute("image_size_y", chh)
            cam = w.spawn_actor(cbp, carla.Transform(carla.Location(x=1.5, z=1.8),
                                                     carla.Rotation(yaw=i * 360.0 / a.cameras)),
                                attach_to=v)
            actors.append(cam)
            cams.append(cam)
        saved = {"done": False}

        def save(img):
            if not saved["done"] and img.frame > 20:
                img.save_to_disk(os.path.join(a.out, "carla010_front.png"))
                saved["done"] = True
            q.put(("cam0", img.frame, 0))
        cams[0].listen(save)
        for i, cam in enumerate(cams[1:], 1):
            cam.listen(lambda img, i=i: q.put((f"cam{i}", img.frame, 0)))
        nsens = 1 + len(cams)

        v.apply_control(carla.VehicleControl(throttle=a.throttle, steer=a.steer))
        dts = []
        steer = []
        npts = []
        late = 0
        for k in range(a.ticks):
            t0 = time.time()
            frame = w.tick()
            got = set()
            while len(got) < nsens:
                try:
                    name, fr, cnt = q.get(timeout=10.0)
                except queue.Empty:
                    late += 1
                    break
                if fr == frame:
                    got.add(name)
                    if name == "lidar":
                        npts.append(cnt)
            dts.append(time.time() - t0)
            try:
                steer.append(v.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
            except RuntimeError as e:
                rep["wheel_steer_angle_error"] = str(e)
        rep["sensor_timeouts"] = late
        rep["sim_time_s"] = round(a.ticks * 0.05, 1)
        rep["tick_ms"] = {"note": "tick + wait for every sensor's frame",
                          "cam_size": a.cam_size,"median": round(statistics.median(dts) * 1000, 1),
                          "p95": round(sorted(dts)[int(0.95 * len(dts))] * 1000, 1),
                          "cameras": a.cameras, "lidar": "128ch 2.6Mpts/s 20Hz"}
        rep["lidar_points_per_frame_median"] = statistics.median(npts) if npts else None
        rep["steer_cmd"] = a.steer
        rep["throttle_cmd"] = a.throttle
        rep["wheel_steer_angle_fl_deg_last"] = steer[-1] if steer else None
        rep["wheel_steer_angle_fl_deg_max"] = max(steer) if steer else None
        rep["speed_mps_end"] = round(v.get_velocity().length(), 2)
        rep["camera_saved"] = saved["done"]
    finally:
        for x in actors:
            if x.type_id.startswith("sensor"):
                x.stop()
        for x in reversed(actors):
            x.destroy()
        w.apply_settings(orig)
    print(json.dumps(rep, indent=1, default=str))
    with open(os.path.join(a.out, "carla010_probe.json"), "w") as f:
        json.dump(rep, f, indent=1, default=str)


if __name__ == "__main__":
    main()
