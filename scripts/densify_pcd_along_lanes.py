#!/usr/bin/env python3
"""Densify an Autoware point cloud map along every driving lane of the loaded CARLA town.

A point cloud sampled only at CARLA's spawn points (`carla_pcd_gen`) leaves stretches of
road with no point near them: on Town04 the ramp had none within 20 m of its middle and NDT
lost the vehicle there. This script carries N lidars along every driving lane, teleported
to a waypoint every STEP metres, voxelises what they see at VOXEL metres in the ROS frame
(CARLA's y flipped) and merges it into the map directory's `pointcloud_map.pcd` (binary,
fields x y z). Town04 went from 3.0 M to 21.0 M points at 0.2 m, step 6 m (2026-10-14).

  scripts/densify_pcd_along_lanes.py <map dir> [--step 6] [--voxel 0.2]
                                     [--host localhost] [--port 2000]

The town CARLA has loaded must be the map directory's: the script checks the name and
refuses otherwise. It switches the world to synchronous mode and ticks it, so nothing else
may drive CARLA meanwhile: stop the simulation side (or run between scenarios with no
scenario stack up). CARLA's settings are restored on exit. Run the signal table generator
after it with CARLA freshly reloaded: after this collector CARLA once reported no traffic
lights until `reload_world`.

Needs the CARLA 0.9.16 Python client and numpy, e.g.

  uv run --no-project --python 3.10 --with numpy \\
    --with <CARLA_0.9.16>/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl \\
    python scripts/densify_pcd_along_lanes.py $CARLA_MAPS/Town04

Roadmap 031, stage 1 (the point cloud step).
"""

import argparse
import os
import sys
import time

import numpy as np

import carla

LIDAR = {
    "channels": "64",
    "range": "85",
    "points_per_second": "2600000",
    "rotation_frequency": "20",
    "upper_fov": "10",
    "lower_fov": "-30",
    "dropoff_general_rate": "0.0",
    "noise_stddev": "0.0",
}


def read_pcd_xyz(path):
    """Points of a binary x y z float32 PCD, as float64 (N, 3)."""
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                sys.exit(f"{path}: no DATA line")
            if line.startswith(b"#"):
                continue
            key, _, value = line.decode().strip().partition(" ")
            header[key] = value
            if key == "DATA":
                break
        if header.get("DATA") != "binary" or header.get("FIELDS") != "x y z" or header.get(
            "SIZE"
        ) != "4 4 4":
            sys.exit(
                f"{path}: expected a binary PCD with fields x y z (float32), found "
                f"DATA={header.get('DATA')} FIELDS={header.get('FIELDS')} SIZE={header.get('SIZE')}"
            )
        n = int(header["POINTS"])
        return np.frombuffer(f.read(n * 12), dtype=np.float32).reshape(-1, 3).astype(np.float64)


def write_pcd_xyz(path, points):
    with open(path, "wb") as g:
        g.write(
            b"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z\n"
            b"SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        )
        g.write(
            f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(points)}\nDATA binary\n".encode()
        )
        g.write(points.astype(np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir")
    ap.add_argument("--step", type=float, default=6.0, help="metres between lidar stops")
    ap.add_argument("--voxel", type=float, default=0.2, help="voxel size, metres")
    ap.add_argument("--lidars", type=int, default=8, help="lidars carried at once")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    pcd = os.path.join(args.map_dir, "pointcloud_map.pcd")
    old = read_pcd_xyz(pcd) if os.path.exists(pcd) else np.empty((0, 3))

    client = carla.Client(args.host, args.port)
    client.set_timeout(120)
    world = client.get_world()
    town = world.get_map().name.split("/")[-1]
    want = os.path.basename(os.path.normpath(args.map_dir))
    if town != want:
        sys.exit(f"CARLA has {town} loaded, the map directory is {want}: load_world it first")
    wps = [
        p
        for p in world.get_map().generate_waypoints(args.step)
        if p.lane_type == carla.LaneType.Driving
    ]
    print(f"{town}: {len(wps)} waypoints every {args.step} m", flush=True)

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    for k, v in LIDAR.items():
        bp.set_attribute(k, v)
    buf = {}
    sensors = []
    voxels = set()
    try:
        for i in range(args.lidars):
            sensor = world.spawn_actor(bp, carla.Transform(carla.Location(0, 0, 500)))
            sensor.listen(lambda meas, i=i: buf.__setitem__(i, meas))
            sensors.append(sensor)
        for b in range(0, len(wps), args.lidars):
            batch = wps[b : b + args.lidars]
            for i, p in enumerate(batch):
                t = p.transform
                loc = carla.Location(t.location.x, t.location.y, t.location.z + 2.0)
                sensors[i].set_transform(carla.Transform(loc, t.rotation))
            world.tick()  # the move
            buf.clear()
            world.tick()  # a sweep from the new place
            t0 = time.time()
            while len(buf) < len(batch) and time.time() - t0 < 5:
                time.sleep(0.01)
            for i in range(len(batch)):
                meas = buf.get(i)
                if meas is None:
                    continue
                local = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
                tf = np.array(meas.transform.get_matrix())
                pts = local @ tf[:3, :3].T + tf[:3, 3]
                pts[:, 1] = -pts[:, 1]  # CARLA (left-handed) to ROS
                voxels.update(map(tuple, np.floor(pts / args.voxel).astype(np.int64)))
            if (b // args.lidars) % 50 == 0:
                print(f"{b} / {len(wps)}: {len(voxels)} voxels", flush=True)
    finally:
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        world.apply_settings(original)

    new = (np.array(list(voxels), dtype=np.float64) + 0.5) * args.voxel
    merged = np.unique(np.floor(np.vstack([old, new]) / args.voxel).astype(np.int64), axis=0)
    out = (merged + 0.5) * args.voxel
    write_pcd_xyz(pcd, out)
    print(f"old {len(old)}, lane voxels {len(new)}, merged {len(out)} -> {pcd}", flush=True)


if __name__ == "__main__":
    main()
