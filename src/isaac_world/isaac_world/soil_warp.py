"""`soil.SoilGrid`'s sweep as one compiled warp kernel on the CPU (047 S3): the numpy version
spends ~100 us per sweep position in numpy's per-call overhead on ~100 nodes; compiled, the
same loops cost a few microseconds. The same algorithm (`SoilGrid._patch`), but the wheel
bottom found by bisection: `test_soil.py` checks the two agree where warp imports (Isaac's
Python).

Imported by `soil` only when a grid asks for it; importing it compiles the kernel once (warp
caches it on disk).
"""
import warp as wp

import numpy as np

wp.init()

BISECTIONS = 20  # 0.3 m / 2^20: 0.3 um


@wp.func
def _carried(sink: wp.array2d(dtype=float), i0: int, i1: int, j0: int, j1: int, ox: int,
             oy: int, h: float, x: float, y: float, c: float, s: float, half_len: float,
             half_w: float, radius: float, zb: float, k: float, n: float, ek: float):
    total = float(0.0)
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            rx = float(i + ox) * h - x
            ry = float(j + oy) * h - y
            dx = rx * c + ry * s
            dy = -rx * s + ry * c
            if wp.abs(dx) <= half_len and wp.abs(dy) <= half_w:
                sp = sink[i, j]
                rim = radius - wp.sqrt(wp.max(radius * radius - dx * dx, float(0.0)))
                d = zb - rim - sp
                if d > 0.0:
                    total += wp.min(ek * d, k * wp.pow(sp + d, n))
    return total * h * h


@wp.kernel
def _sweep(sink: wp.array2d(dtype=float), shear: wp.array2d(dtype=float),
           touched: wp.array2d(dtype=int), px: wp.array(dtype=float),
           py: wp.array(dtype=float), ox: int, oy: int, h: float, c: float, s: float,
           half_len: float, width: float, radius: float, load: float, k: float, n: float,
           ek: float, cohesion: float, tan_phi: float, janosi_k: float, slip_dt: float,
           step_id: int, direction: float, max_sinkage: float,
           out: wp.array(dtype=float)):
    half_w = width * 0.5
    ex = wp.abs(c) * half_len + wp.abs(s) * half_w
    ey = wp.abs(s) * half_len + wp.abs(c) * half_w
    area = h * h
    for m in range(px.shape[0]):
        x = px[m]
        y = py[m]
        i0 = int(wp.floor((x - ex) / h)) - ox
        i1 = int(wp.ceil((x + ex) / h)) - ox
        j0 = int(wp.floor((y - ey) / h)) - oy
        j1 = int(wp.ceil((y + ey) / h)) - oy
        # the wheel bottom: bisection (numpy scans candidates; they agree within 0.1 mm)
        zb = float(max_sinkage)
        if _carried(sink, i0, i1, j0, j1, ox, oy, h, x, y, c, s, half_len, half_w, radius,
                    zb, k, n, ek) >= load:
            lo = float(0.0)
            hi = float(max_sinkage)
            for _it in range(BISECTIONS):
                mid = 0.5 * (lo + hi)
                if _carried(sink, i0, i1, j0, j1, ox, oy, h, x, y, c, s, half_len, half_w,
                            radius, mid, k, n, ek) < load:
                    lo = mid
                else:
                    hi = mid
            zb = 0.5 * (lo + hi)
        res = float(0.0)
        thrust = float(0.0)
        limit = float(0.0)
        carried = float(0.0)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                rx = float(i + ox) * h - x
                ry = float(j + oy) * h - y
                dx = rx * c + ry * s
                dy = -rx * s + ry * c
                if wp.abs(dx) <= half_len and wp.abs(dy) <= half_w:
                    sp = sink[i, j]
                    rim = radius - wp.sqrt(wp.max(radius * radius - dx * dx, float(0.0)))
                    d = wp.max(zb - rim - sp, float(0.0))
                    plastic = k * wp.pow(sp + d, n)
                    sigma = wp.min(ek * d, plastic)
                    if d > 0.0:
                        j_prev = float(0.0)
                        if touched[i, j] >= step_id - 1:
                            j_prev = shear[i, j]
                        jj = j_prev + slip_dt
                        tau_max = cohesion + sigma * tan_phi
                        tau = tau_max * (1.0 - wp.exp(-wp.abs(jj) / janosi_k)) * wp.sign(jj)
                        thrust += tau
                        limit += tau_max
                        shear[i, j] = jj
                        touched[i, j] = step_id
                    res += sigma * wp.clamp(dx / radius, -1.0, 1.0) * direction
                    carried += sigma
                    if plastic < ek * d:
                        sink[i, j] = sp + d - sigma / ek
        out[0] = zb
        out[1] += res * area
        out[2] += thrust * area
        out[3] += limit * area
        out[4] += carried * area


def sweep(sink, shear, touched, path, ox, oy, h, c, s, half_len, width, radius, load, k, p,
          slip_dt, step_id, direction, max_sinkage):
    """Run the sweep over the window arrays (sinkage, shear, touched), updating them; (sinkage,
    resistance sum, thrust sum, limit sum, carried sum) over the positions. float32 inside, as
    the blocks keep them."""
    out = np.zeros(5, np.float32)
    sink32 = np.ascontiguousarray(sink, np.float32)
    shear32 = np.ascontiguousarray(shear, np.float32)
    touched32 = np.ascontiguousarray(touched, np.int32)
    xs = np.array([q[0] for q in path], dtype=np.float32)
    ys = np.array([q[1] for q in path], dtype=np.float32)
    arr = {name: wp.array(a, dtype=t, device="cpu", copy=False)
           for name, a, t in (("sink", sink32, float), ("shear", shear32, float),
                              ("touched", touched32, int), ("px", xs, float),
                              ("py", ys, float), ("out", out, float))}
    wp.launch(_sweep, dim=1, device="cpu",
              inputs=[arr["sink"], arr["shear"], arr["touched"], arr["px"], arr["py"], int(ox),
                      int(oy), float(h), float(c), float(s), float(half_len), float(width),
                      float(radius), float(load), float(k), float(p.n), float(p.elastic_k),
                      float(p.cohesion), float(np.tan(p.friction_angle)), float(p.janosi_k),
                      float(slip_dt), int(step_id), float(direction), float(max_sinkage),
                      arr["out"]])
    sink[...], shear[...], touched[...] = sink32, shear32, touched32
    return out.astype(np.float64)
