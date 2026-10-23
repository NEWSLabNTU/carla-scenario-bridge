"""The soil layer's wheel models (roadmap 047 S3, D2; docs/design/off-road-surfaces.md, "Soil
layer"). Pure Python and numpy: no Isaac, so the maths is unit-tested and timed anywhere.

Two models of one rigid wheel on Bekker-Wong-Janosi soil, both returning a `WheelSoil`:

  SoilGrid     (D2 option A) Chrono SCM's scheme on our own sparse grid: nodes 16 x 16 to a
               block, a block allocated the first time a wheel touches it. Each node keeps its
               plastic sinkage (the rut), so a second pass rides in the first one's rut and sinks
               less, and every wheel on the grid meets every other wheel's ruts. Per wheel and
               step: the wheel's bottom is lowered until the nodes under it carry its load,
               then the nodes take their new sinkage and shear.
  wong_reece   (D2's companion C) the closed-form wheel: the same soil, integrated over the
               contact arc of fresh, flat ground. No memory: the first-pass reference the grid
               is checked against, and the fallback where no grid is wanted.

A node's pressure at penetration d below its current level (plastic sinkage sp):

    sigma = min(K_e d, k (sp + d)^n),   k = k_c / b + k_phi     (Bekker; b the wheel width)

elastic until the Bekker curve at total sinkage sp + d is reached, plastic beyond it; on
unloading the elastic part comes back (sp grows by d - sigma / K_e). The shear on a node in
contact is Janosi's on its accumulated slip displacement j:

    tau = (c + sigma tan phi) (1 - exp(-|j| / K)) sign(j)

Outputs, for the surface layer to hand PhysX (the spike's three channels):
  traction_limit  sum of (c + sigma tan phi) dA: the most the soil can take in shear. Over the
                  load, the wheel's soil-limited friction (lateral grip falls with it too: PhysX
                  has one friction per wheel).
  resistance      the soil's pressure on the front of the wheel against the travel (compaction,
                  N >= 0): a force at the contact (`apply_force_at_pos`).
  sinkage         the wheel bottom below the undisturbed surface, m; the rut is in the grid.

Slip sinkage (a spinning or locked wheel digs in) is not in Bekker's equation. Both models
take Lyasko's factor (2010): the static sinkage times (1 + |i|) / (1 - 0.5 |i|), through
k / factor^n. Engineered, as everything here, until S5 calibrates it.
"""
import math
from dataclasses import dataclass

import numpy as np

BLOCK = 16  # nodes per block side (Chrono SCM's)
# Elastic stiffness of soil on reloading, Pa/m (Chrono SCM's default order).
ELASTIC_K = 2.0e7
# Deepest a wheel bottom is lowered, m: beyond, the chassis is on the ground.
MAX_SINKAGE = 0.3
# A node moved by more than this since its block was last handed out is a dirty block, m.
DIRTY = 0.005


class SoilError(ValueError):
    """Soil parameters that cannot be used; the message lists every problem."""


@dataclass(frozen=True)
class SoilParams:
    """Bekker-Wong-Janosi parameters, SI: n, k_c N/m^(n+1), k_phi N/m^(n+2), cohesion Pa,
    friction angle rad, Janosi shear modulus K m, elastic reload stiffness Pa/m."""

    n: float
    k_c: float
    k_phi: float
    cohesion: float
    friction_angle: float
    janosi_k: float
    elastic_k: float = ELASTIC_K

    def k(self, width):
        return self.k_c / width + self.k_phi


def parse_params(d, where="soil"):
    """SoilParams from a class's `soil:` mapping (friction_angle in degrees)."""
    problems = []
    keys = ("n", "k_c", "k_phi", "cohesion", "friction_angle", "janosi_k")
    if not isinstance(d, dict):
        raise SoilError(f"{where}: not a mapping of {', '.join(keys)}")
    unknown = set(d) - set(keys) - {"elastic_k"}
    if unknown:
        problems.append(f"{where}: unknown key(s) {sorted(unknown)}")
    vals = {}
    for k in (*keys, "elastic_k"):
        if k not in d:
            if k != "elastic_k":
                problems.append(f"{where}: needs {k}")
            continue
        v = d[k]
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 \
                or (k in ("n", "janosi_k", "elastic_k") and v == 0):
            problems.append(f"{where}: {k} {v!r} is not a number "
                            f"{'> 0' if k in ('n', 'janosi_k', 'elastic_k') else '>= 0'}")
            continue
        vals[k] = float(v)
    if "friction_angle" in vals and vals["friction_angle"] >= 90:
        problems.append(f"{where}: friction_angle {vals['friction_angle']} is not below 90 deg")
    if "k_c" in vals and "k_phi" in vals and vals["k_c"] + vals["k_phi"] == 0:
        problems.append(f"{where}: k_c and k_phi are both 0")
    if problems:
        raise SoilError("\n".join(problems))
    vals["friction_angle"] = math.radians(vals["friction_angle"])
    return SoilParams(**vals)


@dataclass(frozen=True)
class WheelSoil:
    sinkage: float  # m, the wheel bottom below the undisturbed surface
    resistance: float  # N, against the travel
    thrust: float  # N, the soil's shear on the wheel, forward positive
    traction_limit: float  # N
    load: float  # N, what the soil carries (less than asked when the wheel bottoms)

    def mu(self, load):
        """The soil-limited friction for a wheel load (N)."""
        return self.traction_limit / load if load > 0 else 0.0


def slip_factor(slip):
    """Lyasko's slip sinkage: static sinkage times this at longitudinal slip `slip`."""
    i = min(abs(slip), 0.95)
    return (1.0 + i) / (1.0 - 0.5 * i)


def slip_ratio(spin_speed, speed):
    """PhysX-style longitudinal slip (r omega - v) / max(|r omega|, |v|), in [-1, 1]; 0 at rest."""
    m = max(abs(spin_speed), abs(speed))
    return 0.0 if m < 1e-3 else (spin_speed - speed) / m


# --- C: the closed-form wheel ----------------------------------------------------------------

THETA_POINTS = 48
# numpy 2 renamed trapz; the host's tests run numpy 1.21, Isaac's 2.3.
_trapz = getattr(np, "trapezoid", None) or np.trapz


def _wr_forces(p, k, r, b, theta1, slip_vel, speed, n_pts=THETA_POINTS):
    """Wong-Reece stresses over the arc [0, theta1] of a rigid wheel (rear exit angle 0):
    (Fz, Fx thrust, resistance, traction limit)."""
    th = np.linspace(0.0, theta1, n_pts)
    i = slip_ratio(speed + slip_vel, speed)
    theta_m = (0.4 + 0.15 * abs(i)) * theta1  # where the normal stress peaks
    # behind the peak, the stress of the mirrored front angle
    eq = np.where(th >= theta_m, th, theta1 - th / max(theta_m, 1e-9) * (theta1 - theta_m))
    sigma = k * (r ** p.n) * np.clip(np.cos(eq) - math.cos(theta1), 0.0, None) ** p.n
    # shear displacement since the soil element entered at theta1 (Wong: time is rotation)
    u = speed + slip_vel  # the rim speed r omega
    j = r * ((theta1 - th) * u - speed * (math.sin(theta1) - np.sin(th))) / max(abs(u), 1e-3)
    tau_max = p.cohesion + sigma * math.tan(p.friction_angle)
    tau = tau_max * (1.0 - np.exp(-np.abs(j) / p.janosi_k)) * np.sign(j)
    w = r * b
    fz = w * _trapz(sigma * np.cos(th) + tau * np.sin(th), th)
    fx = w * _trapz(tau * np.cos(th), th)
    res = w * _trapz(sigma * np.sin(th), th)
    lim = w * _trapz(tau_max * np.cos(th), th)
    return fz, fx, res, lim


def wong_reece(p: SoilParams, load, radius, width, spin_speed=0.0, speed=0.0):
    """A rigid wheel of `radius` and `width` (m) under `load` (N) on fresh flat soil, its rim at
    `spin_speed` (r omega) and its axle at `speed` (m/s, forward positive)."""
    if load <= 0:
        return WheelSoil(0.0, 0.0, 0.0, 0.0, 0.0)
    sign = -1.0 if speed < 0 or (speed == 0 and spin_speed < 0) else 1.0
    v, u = sign * speed, sign * spin_speed
    k = p.k(width) / slip_factor(slip_ratio(u, v)) ** p.n
    lo, hi = 0.0, math.acos(1.0 - min(MAX_SINKAGE, radius) / radius)
    if _wr_forces(p, k, radius, width, hi, u - v, v)[0] < load:
        lo = hi
    else:
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if _wr_forces(p, k, radius, width, mid, u - v, v)[0] < load:
                lo = mid
            else:
                hi = mid
    theta1 = 0.5 * (lo + hi) if lo < hi else lo
    fz, fx, res, lim = _wr_forces(p, k, radius, width, theta1, u - v, v)
    return WheelSoil(radius * (1.0 - math.cos(theta1)), max(0.0, res), sign * fx, lim, fz)


# --- A: the grid -----------------------------------------------------------------------------


class _Block:
    __slots__ = ("sinkage", "shear", "touched", "handed")

    # float32 and int32: 4 kB a block, ~10 MB per km of a wheel's track at 5 cm
    def __init__(self):
        self.sinkage = np.zeros((BLOCK, BLOCK), np.float32)  # plastic, m, >= 0 (the rut)
        self.shear = np.zeros((BLOCK, BLOCK), np.float32)  # Janosi j, m
        self.touched = np.full((BLOCK, BLOCK), -1, np.int32)  # last step in contact
        self.handed = np.zeros((BLOCK, BLOCK), np.float32)  # sinkage last handed out


# The wheel bottom's height: two passes over this many candidates each, then interpolated.
_CANDIDATES = 24
# Most sweep positions in one step (see SoilGrid.wheel).
MAX_SWEEP = 32
# Sweep positions are this many node spacings apart.
SWEEP_SPACING = 1.0


class SoilGrid:
    """One soil zone's sparse node grid, map frame (x east, y north), heights relative to the
    undisturbed ground. Wheels update it one at a time, in the order the caller gives: run them
    in a fixed order every step and the result repeats.

    A wheel moves |v| dt per physics step, 42 cm at 25 m/s and 60 Hz, eight node spacings: the
    nodes it passes over between two steps would never feel its front face. So each call sweeps
    the wheel from where it was in the last step to where it is now, one position per node
    spacing (at most MAX_SWEEP), and averages the forces over the sweep (D4: the patch is
    integrated along the path, the physics rate stays the world's)."""

    def __init__(self, params: SoilParams, spacing=0.05, kernel="auto"):
        """`kernel`: "numpy", "warp" (`soil_warp`, compiled: the world server's), or "auto":
        warp where it imports."""
        self.p = params
        self.h = float(spacing)
        self.warp = None
        if kernel in ("warp", "auto"):
            try:
                from . import soil_warp
                self.warp = soil_warp
            except ImportError:
                if kernel == "warp":
                    raise
        self.blocks = {}  # (bx, by) -> _Block
        self.step_id = 0
        self.last = {}  # wheel key -> (x, y, step_id) of its last call

    # node (ix, iy) is at (ix h, iy h); its block is (ix // BLOCK, iy // BLOCK)

    def depth(self, x, y):
        """The rut depth (plastic sinkage) at the node nearest map point (x, y), m."""
        ix, iy = int(round(x / self.h)), int(round(y / self.h))
        b = self.blocks.get((ix // BLOCK, iy // BLOCK))
        return 0.0 if b is None else float(b.sinkage[ix % BLOCK, iy % BLOCK])

    def begin_step(self):
        """Call once per physics step before the wheels: shear on nodes left behind resets."""
        self.step_id += 1

    def _window(self, ix0, ix1, iy0, iy1, create):
        """Copies of the blocks over node box [ix0, ix1] x [iy0, iy1]: (origin node, sinkage,
        shear, touched, blocks), missing blocks as fresh soil (allocated when `create`)."""
        bx0, bx1, by0, by1 = ix0 // BLOCK, ix1 // BLOCK, iy0 // BLOCK, iy1 // BLOCK
        shape = ((bx1 - bx0 + 1) * BLOCK, (by1 - by0 + 1) * BLOCK)
        sink, shear = np.zeros(shape), np.zeros(shape)
        touched = np.full(shape, -1, dtype=np.int64)
        blocks = []
        for bx in range(bx0, bx1 + 1):
            for by in range(by0, by1 + 1):
                b = self.blocks.get((bx, by))
                if b is None and create:
                    b = self.blocks[(bx, by)] = _Block()
                if b is not None:
                    sl = (slice((bx - bx0) * BLOCK, (bx - bx0 + 1) * BLOCK),
                          slice((by - by0) * BLOCK, (by - by0 + 1) * BLOCK))
                    sink[sl], shear[sl], touched[sl] = b.sinkage, b.shear, b.touched
                    blocks.append((b, sl))
        return (bx0 * BLOCK, by0 * BLOCK), sink, shear, touched, blocks

    def wheel(self, key, x, y, heading, load, radius, width, spin_speed, speed, dt,
              commit=True):
        """Wheel `key` (any hashable, the same every step) at contact point (x, y), heading
        (rad, from +x toward +y), carrying `load` (N), its rim at `spin_speed` and axle at
        `speed` (m/s, forward positive) for `dt` s. Updates the nodes under it (unless `commit`
        is false) and returns its WheelSoil, averaged over the sweep from its last position."""
        last = self.last.get(key)
        if commit:
            self.last[key] = (x, y, self.step_id)
        if load <= 0:
            return WheelSoil(0.0, 0.0, 0.0, 0.0, 0.0)
        path = [(x, y)]
        if last is not None and last[2] == self.step_id - 1:
            dist = math.hypot(x - last[0], y - last[1])
            if dist <= max(1.0, 3.0 * abs(speed) * dt):  # not a teleport
                n = min(MAX_SWEEP, max(1, math.ceil(dist / (SWEEP_SPACING * self.h))))
                path = [(last[0] + (x - last[0]) * (m + 1) / n,
                         last[1] + (y - last[1]) * (m + 1) / n) for m in range(n)]
        c, s = math.cos(heading), math.sin(heading)
        half_len = min(radius, math.sqrt(max(0.0, 2.0 * radius * MAX_SINKAGE
                                             - MAX_SINKAGE ** 2)))
        ex = abs(c) * half_len + abs(s) * width / 2
        ey = abs(s) * half_len + abs(c) * width / 2
        xs, ys = [q[0] for q in path], [q[1] for q in path]
        h = self.h
        ix0, ix1 = math.floor((min(xs) - ex) / h), math.ceil((max(xs) + ex) / h)
        iy0, iy1 = math.floor((min(ys) - ey) / h), math.ceil((max(ys) + ey) / h)
        (ox, oy), sink, shear, touched, blocks = self._window(ix0, ix1, iy0, iy1, commit)
        sub_dt = dt / len(path)
        res = thrust = limit = carried = 0.0
        zb = 0.0
        if self.warp is not None:
            k = self.p.k(width) / slip_factor(slip_ratio(spin_speed, speed)) ** self.p.n
            zb, res, thrust, limit, carried = (float(v) for v in self.warp.sweep(
                sink, shear, touched, path, ox, oy, h, c, s, half_len, width, radius, load, k,
                self.p, (spin_speed - speed) * sub_dt, self.step_id,
                1.0 if speed >= 0 else -1.0, MAX_SINKAGE))
        else:
            for px, py in path:
                zb, r_, t_, l_, f_ = self._patch(px, py, c, s, half_len, load, radius, width,
                                                 spin_speed, speed, sub_dt, ox, oy, sink,
                                                 shear, touched)
                res, thrust, limit, carried = res + r_, thrust + t_, limit + l_, carried + f_
        if commit:
            for b, sl in blocks:
                b.sinkage[:], b.shear[:], b.touched[:] = sink[sl], shear[sl], touched[sl]
        n = len(path)
        return WheelSoil(zb, max(0.0, res / n), thrust / n, limit / n, carried / n)

    def _patch(self, x, y, c, s, half_len, load, radius, width, spin_speed, speed, dt, ox, oy,
               sink, shear, touched):
        """One sweep position: settle the wheel, update the window's nodes in place;
        (sinkage, resistance, thrust, traction limit, load carried)."""
        p, h = self.p, self.h
        ex = abs(c) * half_len + abs(s) * width / 2
        ey = abs(s) * half_len + abs(c) * width / 2
        i0, i1 = math.floor((x - ex) / h) - ox, math.ceil((x + ex) / h) - ox
        j0, j1 = math.floor((y - ey) / h) - oy, math.ceil((y + ey) / h) - oy
        gi, gj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
        rx, ry = (gi + ox) * h - x, (gj + oy) * h - y
        dx = rx * c + ry * s  # along the heading
        dy = -rx * s + ry * c
        keep = (np.abs(dx) <= half_len) & (np.abs(dy) <= width / 2)
        gi, gj, dx = gi[keep], gj[keep], dx[keep]
        if len(gi) == 0:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        sp = sink[gi, gj]
        # the rim's height above the wheel bottom at each node
        rim = radius - np.sqrt(np.maximum(radius * radius - dx * dx, 0.0))
        area = h * h
        k = p.k(width) / slip_factor(slip_ratio(spin_speed, speed)) ** p.n

        def carried(zb):  # load carried for each candidate bottom (rows), N
            d = np.maximum(zb[:, None] - rim - sp, 0.0)
            return np.minimum(p.elastic_k * d, k * (sp + d) ** p.n).sum(axis=1) * area

        lo, hi, flo, fhi = 0.0, MAX_SINKAGE, 0.0, load
        for _ in range(2):
            zs = np.linspace(lo, hi, _CANDIDATES)
            f = carried(zs)
            above = np.nonzero(f >= load)[0]
            if len(above) == 0:
                lo = hi = MAX_SINKAGE  # bottomed: the chassis takes the rest
                break
            m = above[0]
            if m == 0:
                lo = hi = zs[0]
                break
            lo, hi, flo, fhi = zs[m - 1], zs[m], f[m - 1], f[m]
        zb = lo if hi == lo else lo + (hi - lo) * (load - flo) / max(fhi - flo, 1e-9)
        d = np.maximum(zb - rim - sp, 0.0)
        plastic = k * (sp + d) ** p.n
        sigma = np.minimum(p.elastic_k * d, plastic)
        contact = d > 0
        # Shear: each node's slip displacement since it came into contact.
        j = np.where(contact, np.where(touched[gi, gj] >= self.step_id - 1, shear[gi, gj], 0.0)
                     + (spin_speed - speed) * dt, 0.0)
        tau_max = np.where(contact, p.cohesion + sigma * math.tan(p.friction_angle), 0.0)
        tau = tau_max * (1.0 - np.exp(-np.abs(j) / p.janosi_k)) * np.sign(j)
        # Pressure on the rim's front face pushes back against the travel.
        sin_t = np.clip(dx / radius, -1.0, 1.0) * (1.0 if speed >= 0 else -1.0)
        resistance = float((sigma * sin_t).sum() * area)
        yielded = plastic < p.elastic_k * d
        sink[gi, gj] = np.where(yielded, sp + d - sigma / p.elastic_k, sp)
        shear[gi, gj] = np.where(contact, j, shear[gi, gj])
        touched[gi, gj] = np.where(contact, self.step_id, touched[gi, gj])
        return (zb, resistance, float(tau.sum() * area), float(tau_max.sum() * area),
                float(sigma.sum() * area))

    def heights(self, ix0, iy0, n):
        """Rut depths (m, >= 0) at nodes [ix0, ix0 + n] x [iy0, iy0 + n], (n + 1) x (n + 1),
        x first: a tile's vertices, its edges shared with its neighbours'."""
        out = np.zeros((n + 1, n + 1), np.float32)
        for bx in range(ix0 // BLOCK, (ix0 + n) // BLOCK + 1):
            for by in range(iy0 // BLOCK, (iy0 + n) // BLOCK + 1):
                b = self.blocks.get((bx, by))
                if b is None:
                    continue
                x0, y0 = max(ix0, bx * BLOCK), max(iy0, by * BLOCK)
                x1, y1 = min(ix0 + n, bx * BLOCK + BLOCK - 1), min(iy0 + n, by * BLOCK + BLOCK - 1)
                lx, ly = bx * BLOCK, by * BLOCK
                out[x0 - ix0:x1 - ix0 + 1, y0 - iy0:y1 - iy0 + 1] = \
                    b.sinkage[x0 - lx:x1 - lx + 1, y0 - ly:y1 - ly + 1]
        return out

    def dirty_blocks(self):
        """Blocks whose ruts moved more than DIRTY since they were last handed out."""
        return [k for k, b in self.blocks.items() if np.abs(b.sinkage - b.handed).max() > DIRTY]

    def handed_out(self, keys):
        """These blocks' ruts are in the collision now."""
        for k in keys:
            b = self.blocks.get(k)
            if b is not None:
                b.handed[:] = b.sinkage

    def dirty(self, take=True):
        """Blocks whose ruts moved more than DIRTY since last handed out: {(bx, by): sinkage
        array (BLOCK x BLOCK, m)}; `take` marks them handed out."""
        out = {}
        for key, b in self.blocks.items():
            if np.abs(b.sinkage - b.handed).max() > DIRTY:
                out[key] = b.sinkage.copy()
                if take:
                    b.handed[:] = b.sinkage
        return out

    def nbytes(self):
        return sum(b.sinkage.nbytes + b.shear.nbytes + b.touched.nbytes + b.handed.nbytes
                   for b in self.blocks.values())

    def save(self):
        """The ruts, {(bx, by): sinkage array}: what persists across runs (S4)."""
        return {k: b.sinkage.copy() for k, b in self.blocks.items()}

    def load(self, ruts):
        self.blocks = {}
        for k, v in ruts.items():
            b = self.blocks[tuple(k)] = _Block()
            b.sinkage[:] = v
            b.handed[:] = v


# --- The zone edge -----------------------------------------------------------------------------


def edge_distance(polygon, x, y):
    """Distance (m) from (x, y) to the nearest edge of `polygon` (N x 2)."""
    a = np.asarray(polygon, dtype=np.float64)
    b = np.roll(a, -1, axis=0)
    ab = b - a
    t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1])
                / np.maximum((ab * ab).sum(axis=1), 1e-12), 0.0, 1.0)
    px, py = a[:, 0] + t * ab[:, 0], a[:, 1] + t * ab[:, 1]
    return float(np.sqrt((px - x) ** 2 + (py - y) ** 2).min())


def edge_weight(polygon, x, y, blend):
    """The soil's share at (x, y) inside a zone: 0 on its edge, 1 from `blend` m inside on."""
    if blend <= 0:
        return 1.0
    return min(1.0, edge_distance(polygon, x, y) / blend)
