#!/usr/bin/env python3
"""Low-speed pedal maps for a blueprint on CARLA 0.10, in acb's accel_map/brake_map format.

acb's maps (acb_vehicle_description/config/*.csv) are vehicle.tesla.model3 on 0.9.16, measured
on Town01's 224 m straight by acb's scripts/probe_longitudinal.py. CARLA 0.10 ships no Town01 and
no model3, and its Chaos vehicles differ: vehicle.lincoln.mkz does not leave gear 0 below about
0.35 throttle from standstill, so the 0.9.16 map's 0.19 for +0.5 m/s^2 never moves it
(roadmap 019). This measures the same quantities on Town10HD_Opt's 80 m straight, which bounds the
speed range to what a short straight covers -- enough for a low-speed baseline, not for highway
speeds.

Throttle: each pedal from standstill (the launch dead zone), and rolling from about 3 m/s (a car
already moving is in gear). Brake: from about 7 m/s with each brake pedal. Every tick's (v, dv/dt)
is binned into the speed columns; empty cells take the nearest measured column of the same row.
Physics timing matches carla_scenario_bridge's (dt 0.05 s, 16 substeps).

Usage (0.10 client):  probe_pedals_0100.py --port 2100 --out DIR [--blueprint vehicle.lincoln.mkz]
"""
import argparse
import json
import math
import os

import carla
import numpy as np

DT = 0.05
SPEEDS = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]
THROTTLES = [0.0, 0.1, 0.2, 0.3, 0.35, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
BRAKES = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
# Town10HD_Opt: spawn point at CARLA (-24.34, -57.79), eastbound, 80 m without a junction.
START = (-24.34, -57.79)
LENGTH_M = 75.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2100)
    ap.add_argument("--blueprint", default="vehicle.lincoln.mkz")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    c = carla.Client("127.0.0.1", a.port)
    c.set_timeout(60)
    w = c.get_world()
    m = w.get_map()
    orig = w.get_settings()
    if orig.synchronous_mode:
        raise SystemExit("CARLA is synchronous: someone else owns the tick (stop the bridge)")
    s = w.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = DT
    s.substepping = True
    s.max_substep_delta_time = DT / 16
    s.max_substeps = 16
    w.apply_settings(s)
    bp = w.get_blueprint_library().find(a.blueprint)
    bp.set_attribute("role_name", "acb_probe")
    sp = [p for p in m.get_spawn_points()
          if abs(p.location.x - START[0]) < 1 and abs(p.location.y - START[1]) < 1][0]
    samples = {"throttle_standstill": {}, "throttle_rolling": {}, "brake": {}}

    def run(pre_pedal, pre_until, pedal, kind):
        v = w.spawn_actor(bp, sp)
        try:
            for _ in range(20):
                w.tick()
            x0 = v.get_location()
            prev = 0.0
            out = []
            phase_pre = pre_until > 0
            for _ in range(int(20 / DT)):
                spd = v.get_velocity().length()
                if phase_pre and spd >= pre_until:
                    phase_pre = False
                if phase_pre:
                    ctl = carla.VehicleControl(throttle=pre_pedal)
                elif kind == "brake":
                    ctl = carla.VehicleControl(throttle=0.0, brake=pedal)
                else:
                    ctl = carla.VehicleControl(throttle=pedal)
                v.apply_control(ctl)
                w.tick()
                spd2 = v.get_velocity().length()
                if not phase_pre:
                    out.append((spd2, (spd2 - prev) / DT))
                prev = spd2
                if v.get_location().distance(x0) > LENGTH_M:
                    break
                if kind == "brake" and not phase_pre and spd2 < 0.05:
                    break
            return out
        finally:
            v.destroy()
            w.tick()

    try:
        for p in THROTTLES:
            samples["throttle_standstill"][p] = run(0.0, 0, p, "throttle")
            samples["throttle_rolling"][p] = run(0.6, 3.0, p, "throttle")
            print(f"throttle {p}: standstill end v "
                  f"{samples['throttle_standstill'][p][-1][0] if samples['throttle_standstill'][p] else 0:.2f}",
                  flush=True)
        for p in BRAKES:
            samples["brake"][p] = run(0.6, 7.0, p, "brake")
            print(f"brake {p}: {len(samples['brake'][p])} samples", flush=True)
    finally:
        w.apply_settings(orig)

    def smooth(seq):
        if len(seq) < 5:
            return seq
        v = np.array([s[0] for s in seq])
        acc = np.convolve(np.array([s[1] for s in seq]), np.ones(5) / 5, mode="same")
        return list(zip(v, acc))

    def grid(rows, kinds):
        G = np.full((len(rows), len(SPEEDS)), np.nan)
        for i, p in enumerate(rows):
            pts = []
            for k in kinds:
                pts += smooth(samples[k].get(p, []))
            for j, sc in enumerate(SPEEDS):
                if sc == 0.0 and "throttle_standstill" in kinds:
                    # standstill column: the acceleration over the first 0.5 s from rest
                    st = smooth(samples["throttle_standstill"].get(p, []))[:10]
                    G[i, j] = float(np.mean([x[1] for x in st])) if st else 0.0
                    continue
                sel = [acc for (vv, acc) in pts if abs(vv - sc) <= 0.5]
                if len(sel) >= 3:
                    G[i, j] = float(np.median(sel))
            known = np.where(~np.isnan(G[i]))[0]
            for j in range(len(SPEEDS)):
                if np.isnan(G[i, j]) and len(known):
                    G[i, j] = G[i, known[np.abs(known - j).argmin()]]
        return np.nan_to_num(G)

    A = grid(THROTTLES, ["throttle_standstill", "throttle_rolling"])
    B = grid(BRAKES, ["brake"])
    hdr = ("# {kind} response of {bp} on CARLA 0.10.0 (Town10HD_Opt, 75 m straight), measured by\n"
           "# carla-scenario-bridge scripts/ntu_twin/probe_pedals_0100.py (roadmap 019). Low speeds only.\n"
           "# CARLA tick 0.050 s, physics substeps of 0.0031 s (max 16) -- the timing carla_scenario_bridge uses\n"
           "# Rows: pedal position. Columns: vehicle speed in m/s.\n"
           "# Cells: measured longitudinal acceleration in m/s^2.\n")
    for name, kind, rows, G in (("accel_map.csv", "Throttle", THROTTLES, A),
                                ("brake_map.csv", "Brake", BRAKES, B)):
        with open(os.path.join(a.out, name), "w") as f:
            f.write(hdr.format(kind=kind, bp=a.blueprint))
            f.write("default," + ",".join(f"{s:.1f}" for s in SPEEDS) + "\n")
            for p, row in zip(rows, G):
                f.write(f"{p:.2f}," + ",".join(f"{x:.3f}" for x in row) + "\n")
    with open(os.path.join(a.out, "pedal_samples.json"), "w") as f:
        json.dump({k: {str(p): v for p, v in d.items()} for k, d in samples.items()}, f)
    print(open(os.path.join(a.out, "accel_map.csv")).read())
    print(open(os.path.join(a.out, "brake_map.csv")).read())


if __name__ == "__main__":
    main()
