#!/usr/bin/env python3
"""Measure what a CARLA launch configuration costs.

Produces the numbers a launch-flag decision needs: seconds from exec to an RPC the
bridge can use, seconds for the map load the bridge would otherwise have to do,
peak host RSS, GPU memory, sync-mode step rate, and -- the part that decides whether
a rendering-free configuration is usable at all -- whether a ray-cast LiDAR still
returns points.

Usage:
    scripts/carla_bench.py --label baseline
    scripts/carla_bench.py --label town01-direct --launch-map Town01
    scripts/carla_bench.py --label nullrhi --launch-map Town01 --extra -nullrhi

Requires the CARLA PythonAPI wheel for this interpreter:
    pip install $CARLA_DIR/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import carla

# The town every measurement ends on, so configurations are compared at the same
# workload. It is also the town the E2E scenario uses.
TARGET_MAP = "Town01"


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()


def server_pid(port):
    """PID of the shipping binary serving this RPC port (CarlaUE4.sh is only a wrapper)."""
    out = sh(f"pgrep -f 'CarlaUE4-Linux-Shipping.*carla-rpc-port={port}'")
    return int(out.split()[0]) if out else None


def kill_server(port, grace=8.0):
    """Make sure no shipping binary holds this RPC port.

    Kills by pid rather than `pkill -f`: the pattern would also match the shell that
    runs pkill, and killing that shell is a trap this project has already paid for.
    """
    deadline = time.monotonic() + grace
    while True:
        pid = server_pid(port)
        if pid is None:
            return
        try:
            os.kill(pid, signal.SIGKILL if time.monotonic() > deadline else signal.SIGTERM)
        except ProcessLookupError:
            return
        time.sleep(1.0)


def host_mem_mb(pid):
    """(current RSS, peak RSS) in MiB for a pid, from /proc."""
    if pid is None:
        return None, None
    try:
        with open(f"/proc/{pid}/status") as f:
            fields = dict(
                line.split(":", 1) for line in f if line.startswith(("VmRSS", "VmHWM"))
            )
    except FileNotFoundError:
        return None, None
    rss = int(fields.get("VmRSS", "0 kB").split()[0]) // 1024
    hwm = int(fields.get("VmHWM", "0 kB").split()[0]) // 1024
    return rss, hwm


def gpu_mem_mb(pid):
    """GPU memory the pid holds, MiB. 0 when the process has no GPU context at all."""
    if pid is None:
        return None
    # nvidia-smi's csv query covers compute apps only; a rendering process shows up
    # as type G in the plain process table, so parse that instead.
    # Rows look like: | 0  N/A  N/A  35686  C  python  4464MiB |
    for line in sh("nvidia-smi").splitlines():
        fields = line.split()
        if str(pid) in fields[:6]:
            for f in fields:
                if f.endswith("MiB"):
                    return int(f[:-3])
    return 0


def wait_for_rpc(port, deadline_s):
    """Seconds until the server answers, or None if it never did."""
    start = time.monotonic()
    while time.monotonic() - start < deadline_s:
        try:
            client = carla.Client("localhost", port)
            client.set_timeout(5.0)
            client.get_server_version()
            return time.monotonic() - start, client
        except Exception:
            time.sleep(0.5)
    return None, None


def lidar_check(world, ticks=20):
    """Spawn a vehicle + ray-cast LiDAR and report the largest scan seen.

    This is the acceptance test for any no-rendering configuration: acb's ego runs
    perception_mode=lidar, so a configuration that starts fast but returns no points
    is not a configuration at all.
    """
    bp = world.get_blueprint_library()
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(bp.find("vehicle.tesla.model3"), spawn)
    if vehicle is None:
        return {"error": "vehicle spawn failed"}

    lidar_bp = bp.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("channels", "128")
    lidar_bp.set_attribute("range", "200.0")
    lidar_bp.set_attribute("points_per_second", "2621440")
    lidar_bp.set_attribute("rotation_frequency", "20.0")
    lidar = world.spawn_actor(
        lidar_bp, carla.Transform(carla.Location(z=2.0)), attach_to=vehicle
    )

    seen = []
    lidar.listen(lambda scan: seen.append(len(scan)))
    for _ in range(ticks):
        world.tick()
    time.sleep(1.0)  # let the last callbacks land

    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.tick()
    return {"scans": len(seen), "max_points": max(seen) if seen else 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True, help="name for this configuration")
    ap.add_argument("--carla-dir", default=os.path.expanduser("~/Downloads/CARLA_0.9.16"))
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--launch-map", default=None, help="town to boot into (default: server default)")
    ap.add_argument("--quality", default="Low")
    ap.add_argument(
        "--extra",
        default="",
        help="extra CarlaUE4 flags, one space-separated string (e.g. --extra='-nullrhi')",
    )
    ap.add_argument("--no-offscreen", action="store_true")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--out", default=None, help="append the JSON result to this file")
    args = ap.parse_args()

    # CarlaUE4.sh hardcodes "CarlaUE4" as the binary's first argument, which is the slot
    # UE4 reads the startup map from -- a map passed through the wrapper is ignored.
    # Call the shipping binary directly so that slot is ours.
    cmd = [
        "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping",
        f"/Game/Carla/Maps/{args.launch_map}" if args.launch_map else "CarlaUE4",
        f"-quality-level={args.quality}",
        f"-carla-rpc-port={args.port}",
    ]
    if not args.no_offscreen:
        cmd.append("-RenderOffScreen")
    cmd += args.extra.split()

    env = dict(os.environ)
    env.setdefault("VK_ICD_FILENAMES", "/usr/share/vulkan/icd.d/nvidia_icd.json")

    result = {"label": args.label, "cmd": " ".join(cmd)}
    print(f"[{args.label}] {' '.join(cmd)}", flush=True)

    # A server left over from an earlier measurement answers instantly and makes the
    # next configuration look free. Start from an empty port.
    kill_server(args.port)

    proc = subprocess.Popen(
        cmd,
        cwd=args.carla_dir,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    try:
        t_ready, client = wait_for_rpc(args.port, args.timeout)
        if client is None:
            result["error"] = f"no RPC within {args.timeout}s"
            return finish(result, args, proc)
        result["t_rpc_ready_s"] = round(t_ready, 1)

        pid = server_pid(args.port)
        result["pid"] = pid

        # The RPC answers get_server_version before the startup map has finished
        # loading; the world is only usable once its map can be fetched.
        client.set_timeout(180.0)
        t0 = time.monotonic()
        while True:
            try:
                world = client.get_world()
                map_name = world.get_map().name
                break
            except Exception:
                if time.monotonic() - t0 > args.timeout:
                    result["error"] = "world never became available"
                    return finish(result, args, proc)
                time.sleep(0.5)
        result["t_world_ready_s"] = round(t_ready + (time.monotonic() - t0), 1)
        result["booted_map"] = map_name.rsplit("/", 1)[-1]
        rss, hwm = host_mem_mb(pid)
        result["rss_after_boot_mb"] = rss
        result["gpu_after_boot_mb"] = gpu_mem_mb(pid)

        # What the bridge pays at Initialize when the server booted on another town.
        if result["booted_map"] != TARGET_MAP:
            t0 = time.monotonic()
            world = client.load_world(TARGET_MAP)
            result["t_load_target_s"] = round(time.monotonic() - t0, 1)
        else:
            result["t_load_target_s"] = 0.0
        client.set_timeout(30.0)
        result["t_total_to_usable_s"] = round(
            result["t_world_ready_s"] + result["t_load_target_s"], 1
        )

        # Synchronous mode is how the bridge drives the world.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        t0 = time.monotonic()
        for _ in range(100):
            world.tick()
        result["tick_hz_empty"] = round(100 / (time.monotonic() - t0), 1)

        result["lidar"] = lidar_check(world)

        rss, hwm = host_mem_mb(pid)
        result["rss_final_mb"] = rss
        result["rss_peak_mb"] = hwm
        result["gpu_final_mb"] = gpu_mem_mb(pid)
    except Exception as exc:  # a measurement that dies must still free the port
        result["error"] = f"{type(exc).__name__}: {exc}"

    return finish(result, args, proc)


def finish(result, args, proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    # The shipping binary outlives a SIGTERM to its group often enough to poison the
    # next measurement; make sure the port is free before returning.
    kill_server(args.port)
    time.sleep(3)

    print(json.dumps(result, indent=2), flush=True)
    if args.out:
        with open(args.out, "a") as f:
            f.write(json.dumps(result) + "\n")
    return 0 if "error" not in result else 1


if __name__ == "__main__":
    sys.exit(main())
