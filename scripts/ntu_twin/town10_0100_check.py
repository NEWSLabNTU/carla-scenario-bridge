#!/usr/bin/env python3
"""Check a CARLA 0.10 Town10HD_Opt against the TUM Town10 Autoware map (roadmap 019 step 6).

1. Lanelet fit: CARLA waypoints (every 2 m, ROS frame = CARLA with y negated) against the
   lanelet centerlines of lanelet2_map.osm.
2. PCD fit: lidar sweeps at several spawn points, in world coordinates, against
   pointcloud_map.pcd.
3. Ego geometry for a blueprint: rear-axle x in the actor frame from a steady turn (the
   point on the body x axis with zero lateral velocity), bounding box, wheel radius; plus
   whether bone/component transforms expose wheel positions.
4. A start/goal pair on one lane (CARLA waypoints, ROS frame) for an ego-drive scenario.

Usage: town10_0100_check.py MAP_DIR --port 2100 --out report.json [--blueprint vehicle.lincoln.mkz]
"""
import argparse
import json
import math
import queue
import xml.etree.ElementTree as ET

import carla
import numpy as np
from scipy.spatial import cKDTree


def ros(loc):
    return np.array([loc.x, -loc.y, loc.z])


def lanelet_centerlines(path):
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall("node"):
        t = {g.get("k"): g.get("v") for g in n.findall("tag")}
        if "local_x" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]), float(t.get("ele", 0)))
    ways = {w.get("id"): [nodes[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in nodes]
            for w in root.findall("way")}
    pts = []
    for r in root.findall("relation"):
        t = {g.get("k"): g.get("v") for g in r.findall("tag")}
        if t.get("type") != "lanelet" or t.get("subtype") not in ("road", "highway", None, "unknown"):
            continue
        mem = {m.get("role"): m.get("ref") for m in r.findall("member") if m.get("type") == "way"}
        if "left" not in mem or "right" not in mem:
            continue
        L, R = np.array(ways.get(mem["left"], [])), np.array(ways.get(mem["right"], []))
        if len(L) < 2 or len(R) < 2:
            continue

        def resample(P, n):
            d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))]
            s = np.linspace(0, d[-1], n)
            return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])]
        length = max(np.linalg.norm(np.diff(L[:, :2], axis=0), axis=1).sum(), 1.0)
        n = max(2, int(length / 0.5))
        pts.append(0.5 * (resample(L, n) + resample(R, n)))
    return np.concatenate(pts)


def read_pcd(path):
    """float32 x y z (+ more float32 fields) PCD, DATA binary or binary_compressed."""
    raw = open(path, "rb").read()
    j = raw.index(b"\nDATA ") + 1
    end = raw.index(b"\n", j) + 1
    hdr = raw[:end].decode(errors="replace")
    fields = hdr.split("FIELDS")[1].split("\n")[0].split()
    n = int(hdr.split("POINTS")[1].split("\n")[0])
    kind = raw[j:end].split()[1].decode()
    body = raw[end:]
    if kind == "binary":
        return np.frombuffer(body, dtype=np.float32, count=n * len(fields)).reshape(n, len(fields))[:, :3].astype(np.float64)
    if kind == "binary_compressed":
        import lzf
        csize, usize = np.frombuffer(body[:8], dtype=np.uint32)
        data = lzf.decompress(body[8:8 + int(csize)], int(usize))
        cols = np.frombuffer(data, dtype=np.float32).reshape(len(fields), n)  # field-major
        return cols[:3].T.astype(np.float64)
    raise ValueError(f"unsupported PCD DATA {kind}")


def stats(d):
    return {"median_m": round(float(np.median(d)), 3), "p90_m": round(float(np.percentile(d, 90)), 3),
            "p99_m": round(float(np.percentile(d, 99)), 3), "n": int(len(d))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_dir")
    ap.add_argument("--port", type=int, default=2100)
    ap.add_argument("--out", required=True)
    ap.add_argument("--blueprint", default="vehicle.lincoln.mkz")
    ap.add_argument("--route-length", type=float, default=90.0)
    a = ap.parse_args()
    rep = {}
    c = carla.Client("127.0.0.1", a.port)
    c.set_timeout(120.0)
    w = c.get_world()
    m = w.get_map()
    rep["map"] = m.name

    # 1. lanelet fit
    cl = lanelet_centerlines(f"{a.map_dir}/lanelet2_map.osm")
    wps = m.generate_waypoints(2.0)
    wp_ros = np.array([ros(wp.transform.location)[:2] for wp in wps])
    d, _ = cKDTree(cl).query(wp_ros)
    rep["lanelet_fit_waypoints_to_centerlines"] = stats(d)
    d2, _ = cKDTree(wp_ros).query(cl)
    rep["lanelet_fit_centerlines_to_waypoints"] = stats(d2)

    orig = w.get_settings()
    s = w.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    w.apply_settings(s)
    bl = w.get_blueprint_library()
    actors = []
    try:
        # 2. PCD fit
        pcd = read_pcd(f"{a.map_dir}/pointcloud_map.pcd")
        tree = cKDTree(pcd)
        sps = m.get_spawn_points()
        lbp = bl.find("sensor.lidar.ray_cast")
        lbp.set_attribute("channels", "64")
        lbp.set_attribute("range", "80")
        lbp.set_attribute("points_per_second", "1300000")
        lbp.set_attribute("rotation_frequency", "20")
        lbp.set_attribute("upper_fov", "10")
        lbp.set_attribute("lower_fov", "-30")
        q = queue.Queue()
        lid = w.spawn_actor(lbp, carla.Transform(carla.Location(z=-1000)))
        actors.append(lid)
        lid.listen(lambda dd: q.put(dd))
        per = []
        allpts = []
        for sp in sps[:: max(1, len(sps) // 12)][:12]:
            lid.set_transform(carla.Transform(sp.location + carla.Location(z=2.0), sp.rotation))
            for _ in range(3):
                w.tick()
            while not q.empty():
                q.get()
            w.tick()
            meas = q.get(timeout=10)
            tr = meas.transform
            loc = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
            M = np.array(tr.get_matrix())
            world = (M[:3, :3] @ loc.T).T + M[:3, 3]
            world[:, 1] *= -1  # CARLA -> ROS
            dd, _ = tree.query(world)
            per.append({"spawn": [round(sp.location.x, 1), round(-sp.location.y, 1)], **stats(dd)})
            allpts.append(dd)
        rep["pcd_fit_lidar_to_pcd"] = stats(np.concatenate(allpts))
        rep["pcd_fit_per_pose"] = per

        # 3. ego geometry
        sp = sps[5]
        v = w.spawn_actor(bl.find(a.blueprint), sp)
        actors.append(v)
        for _ in range(20):
            w.tick()
        bb = v.bounding_box
        geo = {"blueprint": a.blueprint,
               "bbox_center_actor": [bb.location.x, bb.location.y, bb.location.z],
               "bbox_extent": [bb.extent.x, bb.extent.y, bb.extent.z]}
        pc = v.get_physics_control()
        geo["wheel_radius_m"] = [wh.wheel_radius / 100.0 for wh in pc.wheels]
        geo["max_steer_angle_deg"] = [wh.max_steer_angle for wh in pc.wheels]
        for api in ("get_bone_names", "get_component_names"):
            fn = getattr(v, api, None)
            if fn is None:
                geo[api] = "absent"
                continue
            try:
                names = list(fn())
                geo[api] = names[:40]
            except Exception as e:  # noqa: BLE001
                geo[api] = f"error: {e}"
        wheel_world = {}
        for getter in ("get_bone_world_transform", "get_component_world_transform"):
            fn = getattr(v, getter, None)
            names = geo.get("get_bone_names" if "bone" in getter else "get_component_names")
            if fn is None or not isinstance(names, list):
                continue
            for nm in names:
                if "wheel" in nm.lower() or "tire" in nm.lower() or "tyre" in nm.lower():
                    try:
                        t = fn(nm)
                        wheel_world[f"{getter}:{nm}"] = [t.location.x, t.location.y, t.location.z]
                    except Exception as e:  # noqa: BLE001
                        wheel_world[f"{getter}:{nm}"] = f"error: {e}"
        tr = v.get_transform()
        inv = np.linalg.inv(np.array(tr.get_matrix()))
        geo["wheels_in_actor_frame"] = {
            k: (np.round((inv @ np.r_[p, 1.0])[:3], 3).tolist() if isinstance(p, list) else p)
            for k, p in wheel_world.items()}
        # steady turn: rear axle = point on body x with zero lateral velocity
        samples = []
        for k in range(400):
            v.apply_control(carla.VehicleControl(throttle=0.25, steer=0.4))
            w.tick()
            if k > 250:
                t = v.get_transform()
                vel = v.get_velocity()
                om = v.get_angular_velocity()
                yaw = math.radians(t.rotation.yaw)
                vx = vel.x * math.cos(yaw) + vel.y * math.sin(yaw)
                vy = -vel.x * math.sin(yaw) + vel.y * math.cos(yaw)
                wz = math.radians(om.z)
                if abs(wz) > 0.05:
                    samples.append((vx, vy, wz))
        if samples:
            S = np.array(samples)
            # lateral velocity at body x = vy + wz * x  ->  zero at x = -vy / wz
            xr = -S[:, 1] / S[:, 2]
            geo["rear_axle_x_from_turn_m"] = round(float(np.median(xr)), 3)
            geo["rear_axle_x_spread_m"] = round(float(np.percentile(xr, 90) - np.percentile(xr, 10)), 3)
            geo["turn_speed_mps"] = round(float(np.median(S[:, 0])), 2)
            geo["turn_yaw_rate_rad_s"] = round(float(np.median(S[:, 2])), 3)
        rep["ego_geometry"] = geo

        # 4. route: from a spawn point, follow the lane ahead
        best = None
        for sp in sps:
            wp = m.get_waypoint(sp.location)
            nxt = wp.next(a.route_length)
            if not nxt:
                continue
            g = nxt[0]
            path = [wp]
            cur = wp
            ok = True
            for _ in range(int(a.route_length / 5)):
                n = cur.next(5.0)
                if not n:
                    ok = False
                    break
                cur = n[0]
                if cur.is_junction:
                    ok = False
                    break
                path.append(cur)
            if not ok:
                continue
            P = np.array([ros(p.transform.location)[:2] for p in path])
            dd, _ = cKDTree(cl).query(P)
            if dd.max() < 0.5:
                best = (sp, g, float(dd.max()))
                break
        if best:
            sp, g, dmax = best
            rep["route"] = {
                "start_ros": [round(sp.location.x, 2), round(-sp.location.y, 2), round(sp.location.z, 2),
                              round(-math.radians(sp.rotation.yaw), 5)],
                "goal_ros": [round(g.transform.location.x, 2), round(-g.transform.location.y, 2),
                             round(g.transform.location.z, 2),
                             round(-math.radians(g.transform.rotation.yaw), 5)],
                "length_m": a.route_length, "max_dist_to_lanelet_m": round(dmax, 3),
                "no_junction": True}
    finally:
        for x in actors:
            if x.type_id.startswith("sensor"):
                x.stop()
        for x in reversed(actors):
            x.destroy()
        w.apply_settings(orig)
    with open(a.out, "w") as f:
        json.dump(rep, f, indent=1, default=str)
    print(json.dumps({k: v for k, v in rep.items() if k != "pcd_fit_per_pose"}, indent=1, default=str))


if __name__ == "__main__":
    main()
