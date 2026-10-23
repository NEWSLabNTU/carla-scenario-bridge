"""Calibration (roadmap 047 S5): our soil grid (`isaac_world.soil`) against Chrono's single-wheel
test rig on the same wheel, at the same load and slips. Two references:

  scm   Chrono's SCM terrain with the class's own Bekker-Wong-Janosi parameters: the same
        model family, so a check of our implementation (same inputs, should agree)
  crm   Chrono's CRM (SPH continuum, mu(I) rheology) with the class's density, friction angle
        and cohesion: a different model of the same soil, the "virtual bevameter" the
        survey called for; where the grid and CRM disagree, the grid's parameters are fitted

Runs in Chrono's own environment (pychrono 10 from the projectchrono conda channel, CUDA for
CRM), not Isaac's:

    <chrono env>/bin/python src/isaac_world/spikes/crm_calibrate.py [scm|crm|grid ...] \\
        [--classes sand,dirt] [--slips 0,0.1,0.2,0.3,0.5] [--spacing 0.02] [--out res.json]
        [--lugs 24 --lug-h 0.02]          # a tread: bars on the rigid tire (default smooth)
    # prints one SPIKE-RESULT json per terrain

Per class and slip, at steady state (the last 40 % of the test): sinkage (the wheel bottom
below the undisturbed surface, m), drawbar pull (N, forward positive) and the wheel's
driving torque (N m). The rig pins the slip: carrier at 1 m/s, the wheel spun at
v / (1 - slip) / R; the normal load includes the wheel's weight.
"""
import argparse
import json
import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaac_world import soil, surfaces  # noqa: E402

R, B = 0.37, 0.235  # aib's default wheel (test_soil.py)
LOAD = 4500.0  # N, a Model 3 wheel
SPEED = 1.0  # m/s, the carrier
DELAY, TEST = 1.0, 4.0  # s: settle, then the test at the set slip
STEADY = 0.4  # the share of the test averaged
# CRM's soil per class (mu(I) rheology): the class's friction angle and cohesion, a density
# for the soil the Bekker row was measured on, and the elastic modulus Chrono's CRM wheel
# validations use (Hu et al. 2024, single-wheel tests on GRC-1 and sand: 1-2 MPa); mu_2 =
# mu_s, so no rate dependence, as Mohr-Coulomb.
CRM_SOIL = {
    "sand": {"density": 1700.0, "young": 2e6},
    "dirt": {"density": 1600.0, "young": 2e6},
    "mud_wet": {"density": 1700.0, "young": 1e6},
}


KEEP = []


def write_cylinder(path, segments=96, lugs=0, lug_h=0.0):
    """A closed cylinder of the wheel's size, axis y (Chrono's spin axis), as an OBJ: CRM makes
    the tire's boundary markers from its contact mesh, and a rigid tire without one
    segfaults the rig's Initialize."""
    lines = []
    per = segments // lugs if lugs else 0
    for side in (-B / 2, B / 2):
        for k in range(segments):
            a = 2 * math.pi * k / segments
            # `lugs` tread bars `lug_h` proud of the radius R (a third of each pitch), so the
            # tread's outer surface is at R and the root at R - lug_h
            r = R - lug_h if per and (k % per) >= per // 3 else R
            lines.append(f"v {r * math.cos(a):.6f} {side:.6f} {r * math.sin(a):.6f}")
    lines += [f"v 0 {-B / 2:.6f} 0", f"v 0 {B / 2:.6f} 0"]
    c0, c1 = 2 * segments + 1, 2 * segments + 2
    for k in range(segments):
        a, b = k + 1, (k + 1) % segments + 1
        lines.append(f"f {a} {b} {b + segments} {a + segments}")
        lines.append(f"f {c0} {b} {a}")
        lines.append(f"f {c1} {a + segments} {b + segments}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_wheel(d, lugs=0, lug_h=0.0):
    import pychrono.vehicle as veh

    # under Chrono's vehicle data, as its JSONs name meshes
    mesh = f"swb/swb_tire_{lugs}x{lug_h:g}.obj"
    os.makedirs(os.path.dirname(veh.GetVehicleDataFile(mesh)), exist_ok=True)
    write_cylinder(veh.GetVehicleDataFile(mesh), 96, lugs, lug_h)
    wheel = {"Name": "swb wheel", "Type": "Wheel", "Template": "Wheel", "Mass": 18.0,
             "Inertia": [0.6, 1.2, 0.6]}
    tire = {"Name": "swb rigid tire", "Type": "Tire", "Template": "RigidTire", "Radius": R,
            "Width": B, "Mass": 12.0, "Inertia": [0.6, 1.2, 0.6],
            "Contact Material": {"Coefficient of Friction": 0.9,
                                 "Coefficient of Restitution": 0.1,
                                 "Properties": {"Young Modulus": 2e7, "Poisson Ratio": 0.3},
                                 "Coefficients": {"Normal Stiffness": 2e5,
                                                  "Normal Damping": 400.0,
                                                  "Tangential Stiffness": 2e5,
                                                  "Tangential Damping": 400.0}},
            "Contact Mesh": {"Mesh Filename": mesh, "Sweep Sphere Radius": 0.005}}
    paths = []
    for name, data in (("wheel.json", wheel), ("tire.json", tire)):
        p = os.path.join(d, name)
        with open(p, "w") as f:
            json.dump(data, f)
        paths.append(p)
    return paths


def chrono_run(terrain, cls, params, slip, spacing, tmp, lugs=0, lug_h=0.0):
    import pychrono as chrono
    import pychrono.vehicle as veh

    wheel_json, tire_json = write_wheel(tmp, lugs, lug_h)
    wheel = veh.ReadWheelJSON(wheel_json)
    tire = veh.ReadTireJSON(tire_json)
    sys_ = chrono.ChSystemNSC()
    step = 2e-4
    sys_.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    tire.SetStepsize(step)
    rig = veh.ChWheelTestRig(sys_, wheel, tire)
    rig.SetGravitationalAcceleration(9.81)
    rig.SetNormalLoad(LOAD)
    rig.SetStepsize(step)
    size = veh.TerrainPatchSize()
    size.length = SPEED * (DELAY + TEST + 2.0) + 4.0
    size.width = 1.0
    size.depth = 0.3
    if terrain == "scm":
        size.depth = 0.5
        rig.SetTerrainSCM(size, params.k_phi, params.k_c, params.n, params.cohesion,
                          math.degrees(params.friction_angle), params.janosi_k, 0.02)
    elif terrain == "crm":
        p = veh.TerrainParamsCRM()
        sph = p.sph_params
        sph.initial_spacing = spacing
        p.sph_params = sph
        m = p.mat_props
        m.density = CRM_SOIL[cls]["density"]
        m.Young_modulus = CRM_SOIL[cls]["young"]
        m.Poisson_ratio = 0.3
        m.mu_fric_s = m.mu_fric_2 = math.tan(params.friction_angle)
        m.mu_I0 = 0.04
        m.average_diam = 0.005
        m.cohesion_coeff = params.cohesion
        p.mat_props = m
        rig.SetTerrainCRM(size, p)
        rig.SetWheelActiveDomain()
    else:
        rig.SetTerrainRigid(size, 0.9, 0.1, 2e7)
    rig.SetConstantLongitudinalSlip(slip, SPEED)
    rig.SetTimeDelay(DELAY)
    rig.Initialize(veh.ChWheelTestRig.Mode_TEST, 0.05)
    t0 = time.perf_counter()
    rows, start = [], None
    while start is None or sys_.GetChTime() < start + TEST:
        rig.Advance(step)
        t = sys_.GetChTime()
        if start is None:
            if rig.OutputEnabled():
                start = t  # the drop and the delay done: the slip is on
                if terrain == "crm":
                    # the particles' top centres, half a spacing below the surface
                    crm_top = veh.CastToCRMTerrain(rig.GetTerrain()).GetSPHBoundingBox().max.z \
                        + spacing / 2
            continue
        if t > start + TEST * (1 - STEADY):
            pos = rig.GetWheelPos()
            ground = crm_top if terrain == "crm" else \
                rig.GetTerrain().GetHeight(chrono.ChVector3d(pos.x + 1.5, pos.y, 0.0))
            # the tire's force report: the terrain's on SCM, nothing from CRM's markers
            rows.append((pos.z - ground, rig.GetDBP(), -rig.ReportWheelForce().moment.y,
                         rig.GetLongitudinalSlip()))
    wall = time.perf_counter() - t0
    KEEP.extend((sys_, rig, wheel, tire))  # never destroyed: see one_case
    n = len(rows)
    z = sum(r[0] for r in rows) / n
    return {"sinkage": round(R - z, 4), "dbp": round(sum(r[1] for r in rows) / n, 1),
            "torque": round(sum(r[2] for r in rows) / n, 1) if terrain != "crm" else None,
            "slip_seen": round(sum(r[3] for r in rows) / n, 3), "wall_s": round(wall, 1)}


def grid_run(params, slip, dt=1 / 60, length=6.0):
    """The grid as the world server runs it: one wheel straight along +x, the steady second
    half (test_soil.drive)."""
    g = soil.SoilGrid(params)
    out, x = [], 0.0
    while x < length:
        g.begin_step()
        out.append(g.wheel(0, x, 0.013, 0.0, LOAD, R, B, SPEED / (1 - slip), SPEED, dt))
        x += SPEED * dt
    half = out[len(out) // 2:]
    mean = {f: sum(getattr(o, f) for o in half) / len(half)
            for f in ("sinkage", "resistance", "thrust")}
    return {"sinkage": round(float(mean["sinkage"]), 4),
            "dbp": round(mean["thrust"] - mean["resistance"], 1),
            "torque": round(mean["thrust"] * R, 1)}


def one_case(terrain, cls, slip, spacing, lugs=0, lug_h=0.0):
    """One Chrono case in a process of its own: pychrono segfaults tearing a rig down, so the
    child prints its result and leaves without destructors."""
    import subprocess

    out = subprocess.run([sys.executable, os.path.abspath(__file__), "--one", terrain, cls,
                          str(slip), str(spacing), str(lugs), str(lug_h)],
                         capture_output=True, text=True)
    for line in out.stdout.splitlines():
        if line.startswith("CASE "):
            return json.loads(line[5:])
    return {"error": (out.stdout + out.stderr)[-600:]}


def main(argv=None):
    if argv is None and len(sys.argv) > 1 and sys.argv[1] == "--one":
        terrain, cls, slip, spacing, lugs, lug_h = sys.argv[2:8]
        with tempfile.TemporaryDirectory() as tmp:
            r = chrono_run(terrain, cls, surfaces.load_classes()[cls].soil, float(slip),
                           float(spacing), tmp, int(lugs), float(lug_h))
        print("CASE " + json.dumps(r), flush=True)
        os._exit(0)
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("terrains", nargs="*", default=["grid", "scm", "crm"])
    ap.add_argument("--classes", default="sand,dirt")
    ap.add_argument("--slips", default="0,0.1,0.2,0.3,0.5")
    ap.add_argument("--spacing", type=float, default=0.02, help="CRM particle spacing, m")
    ap.add_argument("--lugs", type=int, default=0, help="tread bars on the tire (0: smooth)")
    ap.add_argument("--lug-h", type=float, default=0.02, help="their height, m")
    ap.add_argument("--out", default="")
    args = ap.parse_args(argv)
    table = surfaces.load_classes()
    slips = [float(s) for s in args.slips.split(",")]
    result = {}
    for terrain in args.terrains:
        res = result[terrain] = {}
        for cls in args.classes.split(","):
            params = table[cls].soil
            res[cls] = {}
            for slip in slips:
                r = grid_run(params, slip) if terrain == "grid" else \
                    one_case(terrain, cls, slip, args.spacing, args.lugs,
                             args.lug_h if args.lugs else 0.0)
                res[cls][str(slip)] = r
                print(f"SPIKE-PROGRESS {terrain} {cls} slip {slip}: {r}", flush=True)
        print("SPIKE-RESULT " + json.dumps({terrain: res}), flush=True)
        if args.out:
            with open(args.out, "w") as f:
                json.dump(result, f, indent=1)


if __name__ == "__main__":
    main()
