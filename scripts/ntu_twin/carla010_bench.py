#!/usr/bin/env python3
"""Benchmark CARLA 0.10 launch settings: startup time, VRAM, sensor-synced frame time.

For each named profile: start CARLA with the profile's flags (own process group), time
RPC-up and first synchronous tick, read the server's VRAM from nvidia-smi (per pid), run
carla010_probe.py with N cameras, record peak VRAM during it, then stop the server.

Usage (with the 0.10 client venv):
  carla010_bench.py --carla-dir ~/Downloads/Carla-0.10.0-Linux-Shipping --port 2100 \
      --profiles epic,low,nort --cameras 1 --out DIR
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

import carla

RS = "[/Script/Engine.RendererSettings]"
PROFILES = {
    # shipped config: Epic, hardware ray tracing on
    "epic": [],
    "low": ["-quality-level=Low"],
    # Epic, no hardware ray tracing (Lumen falls back to software), VSM off, smaller atlas
    "nort": [f"-ini:Engine:{RS}:r.RayTracing=False",
             f"-ini:Engine:{RS}:r.Lumen.HardwareRayTracing=False",
             f"-ini:Engine:{RS}:r.RayTracing.Shadows=False",
             f"-ini:Engine:{RS}:r.Shadow.Virtual.Enable=0",
             f"-ini:Engine:{RS}:r.DistanceFields.AtlasSizeXY=256",
             f"-ini:Engine:{RS}:r.DistanceFields.AtlasSizeZ=512"],
    # as nort, plus Lumen off entirely and a smaller texture pool
    "lean": [f"-ini:Engine:{RS}:r.RayTracing=False",
             f"-ini:Engine:{RS}:r.Lumen.HardwareRayTracing=False",
             f"-ini:Engine:{RS}:r.RayTracing.Shadows=False",
             f"-ini:Engine:{RS}:r.Shadow.Virtual.Enable=0",
             f"-ini:Engine:{RS}:r.DistanceFields.AtlasSizeXY=256",
             f"-ini:Engine:{RS}:r.DistanceFields.AtlasSizeZ=512",
             f"-ini:Engine:{RS}:r.DynamicGlobalIlluminationMethod=0",
             f"-ini:Engine:{RS}:r.ReflectionMethod=2",
             f"-ini:Engine:{RS}:r.Streaming.PoolSize=1500"],
    "lean_low": ["-quality-level=Low",
                 f"-ini:Engine:{RS}:r.RayTracing=False",
                 f"-ini:Engine:{RS}:r.Lumen.HardwareRayTracing=False",
                 f"-ini:Engine:{RS}:r.RayTracing.Shadows=False",
                 f"-ini:Engine:{RS}:r.Shadow.Virtual.Enable=0",
                 f"-ini:Engine:{RS}:r.Streaming.PoolSize=1500"],
}


def vram_of(pids):
    try:
        out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory",
                              "--format=csv,noheader,nounits"], capture_output=True, text=True,
                             timeout=10).stdout
    except Exception:
        return None
    tot = 0
    for line in out.strip().splitlines():
        p, m = [x.strip() for x in line.split(",")]
        if int(p) in pids:
            tot += int(m)
    return tot


def server_pids(pgid):
    pids = set()
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                if os.getpgid(int(d)) == pgid:
                    pids.add(int(d))
            except ProcessLookupError:
                pass
    return pids


def run_profile(name, a):
    flags = PROFILES[name]
    env = dict(os.environ, VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json")
    log = open(os.path.join(a.out, f"server_{name}.log"), "w")
    cmd = [os.path.join(a.carla_dir, "CarlaUnreal.sh"), f"-carla-rpc-port={a.port}", "-nosound",
           "-RenderOffScreen"] + flags
    t0 = time.time()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            env=env)
    pgid = proc.pid
    res = {"profile": name, "flags": flags}
    try:
        # A fresh client per attempt: an rpclib client created before the server listens
        # never reconnects, so one long-lived client would wait forever.
        while True:
            if proc.poll() is not None:
                res["error"] = "server exited during startup"
                return res
            try:
                c = carla.Client("127.0.0.1", a.port)
                c.set_timeout(5.0)
                c.get_server_version()
                break
            except RuntimeError:
                time.sleep(2)
            if time.time() - t0 > 400:
                res["error"] = "no RPC in 400 s"
                return res
        res["rpc_up_s"] = round(time.time() - t0, 1)
        c.set_timeout(120.0)
        w = c.get_world()
        s = w.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = 0.05
        w.apply_settings(s)
        w.tick()
        res["first_tick_s"] = round(time.time() - t0, 1)
        s.synchronous_mode = False
        w.apply_settings(s)
        time.sleep(5)
        res["vram_idle_mib"] = vram_of(server_pids(pgid))
        peak = {"v": 0}
        stop = threading.Event()

        def sampler():
            while not stop.is_set():
                v = vram_of(server_pids(pgid)) or 0
                peak["v"] = max(peak["v"], v)
                time.sleep(1)
        th = threading.Thread(target=sampler, daemon=True)
        th.start()
        pout = os.path.join(a.out, f"probe_{name}")
        r = subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), "carla010_probe.py"),
                            "--port", str(a.port), "--cameras", str(a.cameras), "--ticks", "200",
                            "--throttle", "0.0", "--steer", "0.0", "--out", pout],
                           capture_output=True, text=True, timeout=900)
        stop.set()
        th.join()
        res["vram_peak_with_sensors_mib"] = peak["v"]
        if r.returncode != 0:
            res["error"] = "probe failed: " + r.stderr[-400:]
        else:
            pr = json.load(open(os.path.join(pout, "carla010_probe.json")))
            res["tick_ms"] = pr["tick_ms"]
            res["sensor_timeouts"] = pr.get("sensor_timeouts")
        res["alive_after"] = proc.poll() is None
        return res
    finally:
        try:
            os.killpg(pgid, signal.SIGTERM)
            proc.wait(timeout=30)
        except Exception:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--carla-dir", required=True)
    ap.add_argument("--port", type=int, default=2100)
    ap.add_argument("--profiles", default="epic,nort,lean,low")
    ap.add_argument("--cameras", type=int, default=1)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    results = []
    for name in a.profiles.split(","):
        r = run_profile(name, a)
        print(json.dumps(r, default=str), flush=True)
        results.append(r)
        with open(os.path.join(a.out, "bench.json"), "w") as f:
            json.dump(results, f, indent=1, default=str)


if __name__ == "__main__":
    main()
