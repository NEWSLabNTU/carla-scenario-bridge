"""Drive test (roadmap 047 S3): aib's Model 3 through the soil layer (`isaac_world.soil`, via
`isaac_world.surface_layer`, the world server's own code) on a flat pad, one lane per zone.

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/soil_drive.py
    # prints one SPIKE-RESULT json

The pad is one mesh labelled Road, so outside a zone every wheel is on dry asphalt; zones
start at x = 100 and run to 400. No ruts in the collision yet (the pad stays flat): the soil
reaches PhysX as friction and as a resistance force at each contact, as the world server
applies it. Cases:

  coast_<lane>   enter the zone at 15 m/s with no throttle and no brake: the deceleration
                 (asphalt: rolling resistance and drag only)
  hold_<lane>    hold 10 m/s through it on aib's speed controller: drive demand, slip,
                 sinkage, resistance, friction; `hold_sand_2` is the same line again (ruts)
  launch_<lane>  full throttle from rest in the zone for 3 s (traction control)
  brake_<lane>   full braking from 15 m/s in the zone (ABS)
  entry          the largest change of the soil's resistance between two substeps on entry
                 at 20 m/s, against the edge blend, and in steady state 10 m on
  rut_1, rut_2   the `sand_rut` lane: sand whose ground is the server's tiles (`ground_z`, a
                 hole in the pad), 8 m/s through it twice on one line: the rear wheels' ground
                 hit (the rut they meet), the body's height, the speed's largest drop in a
                 frame (a tile swap must not stop the car), tiles rebuilt, rebuild cost
  cost           the surface layer's update per vehicle per substep, in and out of a zone
"""
import json
import math
import os
import sys
import tempfile
import time

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics  # noqa: E402

from aib.control import GEAR_DRIVE, Command, State  # noqa: E402
from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene  # noqa: E402
from aib.rig import EgoConfig  # noqa: E402
from isaac_world import soil as soil_model, surfaces  # noqa: E402
from isaac_world.soil_tiles import SoilTiles  # noqa: E402
from isaac_world.surface_layer import QCODE_ATTR, SurfaceLayer  # noqa: E402

FRAME, SUB = 0.05, 3
dt = FRAME / SUB
ZONE_X0, ZONE_X1 = 100.0, 400.0
LANES = {"asphalt": 0.0, "sand": 10.0, "dirt": 20.0, "sand_cf": 30.0, "sand_rut": 40.0}
RUT_X1 = 160.0


def band(y0, y1):
    return [[ZONE_X0, y0], [ZONE_X1, y0], [ZONE_X1, y1], [ZONE_X0, y1]]


SURFACES = {
    "defaults": {"Road": "asphalt"},
    "zones": [
        {"id": "sand", "class": "sand", "soil": "grid", "polygon": band(6, 14)},
        {"id": "dirt", "class": "dirt", "soil": "grid", "polygon": band(16, 24)},
        {"id": "sand_cf", "class": "sand", "soil": "closed_form", "polygon": band(26, 34)},
        {"id": "sand_rut", "class": "sand", "soil": "grid", "ground_z": 0.0,
         "polygon": [[ZONE_X0, 36], [RUT_X1, 36], [RUT_X1, 44], [ZONE_X0, 44]]},
    ],
}

world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
stage = omni.usd.get_context().get_stage()
# The pad, less the sand_rut zone's box (its ground is the soil tiles); the zone's box is
# whole tiles of 1.6 m: x 99.2..160.0, y 35.2..44.8.
for name, (x0, x1, y0, y1) in {"main": (-100, 500, -40, 35.2), "top": (-100, 500, 44.8, 60),
                               "west": (-100, 99.2, 35.2, 44.8),
                               "east": (160.0, 500, 35.2, 44.8)}.items():
    mesh = UsdGeom.Mesh.Define(stage, f"/World/env/Road_pad/{name}")
    mesh.CreatePointsAttr([Gf.Vec3f(x0, y0, 0), Gf.Vec3f(x1, y0, 0), Gf.Vec3f(x1, y1, 0),
                           Gf.Vec3f(x0, y1, 0)])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateFaceVertexCountsAttr([4])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
stage.GetPrimAtPath("/World/env/Road_pad").CreateAttribute(
    QCODE_ATTR, Sdf.ValueTypeNames.TokenArray).Set(["Q34442"])
if not any(pr.IsA(UsdPhysics.Scene) for pr in stage.Traverse()):
    UsdPhysics.Scene.Define(stage, "/World/physicsScene")
ensure_vehicle_scene(stage)
cfg = EgoConfig.load(os.environ.get("AIB_EGO") or os.path.join(
    os.path.dirname(__file__), "../../autoware_isaac_bridge/src/aib/config/ego.yaml"))
rig = cfg.load_rig()
veh = Vehicle(stage, "/World/ego", rig.vehicle, rig.box_center, rig.box_dims,
              ((0.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0)))
sim = omni.physx.get_physx_simulation_interface()
forces = []  # the soil's resistance this substep, N (for the entry case)


def apply_force(path, force, position):
    """As IsaacWorld._apply_force."""
    forces.append(math.sqrt(sum(f * f for f in force)))
    sim.apply_force_at_pos(omni.usd.get_context().get_stage_id(),
                           PhysicsSchemaTools.sdfPathToInt(path), force, position)


with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "surfaces.yaml")
    with open(path, "w") as f:
        json.dump(SURFACES, f)  # JSON is YAML
    layer = SurfaceLayer(stage, surfaces.load(path), apply_force=apply_force,
                         make_tiles=lambda z, h, o: SoilTiles(stage, z, h, o))
world.reset()
world.play()
physx = omni.physx.get_physx_interface()
state = State()
cmd = Command(received=True, gear=GEAR_DRIVE)
update_s = {"in": [], "out": []}
step_forces = []  # per substep: total resistance applied
flush_s = []  # per frame with a rebuild: the flush's time
rebuilt = [0]


def frame():
    for _ in range(SUB):
        veh.apply(cmd, dt, state)
        world.step(render=False)
        veh.sample(physx, dt, state)
        forces.clear()
        layer.begin_step()
        t0 = time.perf_counter()
        layer.update(veh, state.velocity[0], dt)
        el = time.perf_counter() - t0
        soil = any(layer.soil_under(veh))
        update_s["in" if soil else "out"].append(el)
        step_forces.append(sum(forces))
    t0 = time.perf_counter()
    done = layer.flush()
    if done:
        flush_s.append(time.perf_counter() - t0)
        rebuilt[0] += sum(len(v) for v in done.values())


def soil_now():
    """Mean over the wheels on soil: sinkage cm, resistance N, friction written."""
    under = [s for s in layer.soil_under(veh) if s]
    if not under:
        return None
    return {"sink_cm": round(sum(s[0].sinkage for s in under) / len(under) * 100, 2),
            "res_n": round(sum(s[0].resistance * s[1] for s in under), 0),
            "mu": [round(m, 3) for m in veh.friction_values]}


def start(lane, x, settle=30):
    veh.teleport(((x, LANES[lane], 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
    layer.reset(veh.path)
    cmd.tire_angle = 0.0
    for _ in range(settle):
        cmd.velocity, cmd.acceleration = 0.0, 0.0
        frame()


def reach(speed, x_until):
    while state.position[0] < x_until:
        cmd.velocity = speed
        cmd.acceleration = max(-3.0, min(3.0, 1.5 * (speed - state.velocity[0])))
        frame()


def coast(lane):
    start(lane, 0.0)
    reach(15.0, ZONE_X0 + 3.0)
    v0, t, trace = state.velocity[0], 0.0, []
    while t < 3.0 and state.velocity[0] > 0.5:
        cmd.velocity, cmd.acceleration = state.velocity[0], 0.0  # no correction
        frame()
        t += FRAME
        trace.append(round(state.velocity[0], 2))
    return {"v0": round(v0, 2), "v_after": trace[-1], "t_s": round(t, 2),
            "decel": round((v0 - trace[-1]) / t, 2), "soil": soil_now(), "trace": trace[::5]}


def hold(lane, length=80.0):
    start(lane, 0.0)
    reach(10.0, ZONE_X0 + 20.0)
    acc, slips, vs, samples = [], [], [], []
    x_end = ZONE_X0 + 20.0 + length
    while state.position[0] < x_end:
        cmd.velocity = 10.0
        cmd.acceleration = max(-3.0, min(3.0, 1.5 * (10.0 - state.velocity[0])))
        frame()
        acc.append(cmd.acceleration)
        vs.append(state.velocity[0])
        if veh.wheel_states:
            slips.append(sum(w.slip for w in veh.wheel_states[2:]) / 2)
        s = soil_now()
        if s:
            samples.append(s)
    n = len(vs)
    return {"v_mean": round(sum(vs) / n, 2), "accel_demand": round(sum(acc) / n, 2),
            "rear_slip": round(sum(slips) / max(len(slips), 1), 3),
            "sink_cm": round(sum(s["sink_cm"] for s in samples) / max(len(samples), 1), 2),
            "res_n": round(sum(s["res_n"] for s in samples) / max(len(samples), 1), 0),
            "mu_last": samples[-1]["mu"] if samples else None,
            "grid_kb": {k: round(g.nbytes() / 1e3) for k, g in layer.grids.items()}}


def launch(lane):
    start(lane, ZONE_X0 + 150.0)
    slips = []
    for _ in range(60):
        cmd.velocity, cmd.acceleration = 30.0, 4.0
        frame()
        if state.velocity[0] > 1.0 and veh.wheel_states:
            slips.append(max(w.slip for w in veh.wheel_states[2:]))
    return {"v_3s": round(state.velocity[0], 2),
            "max_slip": round(max(slips), 3) if slips else None,
            "mean_slip": round(sum(slips) / len(slips), 3) if slips else None,
            "soil": soil_now()}


def brake(lane):
    start(lane, 0.0)
    reach(15.0, ZONE_X0 + 220.0)
    x0, v0, t = state.position[0], state.velocity[0], 0.0
    while state.velocity[0] > 0.3 and t < 20.0:
        cmd.velocity, cmd.acceleration = 0.0, -9.81
        frame()
        t += FRAME
    return {"v0": round(v0, 2), "stop_m": round(state.position[0] - x0, 2), "t_s": round(t, 2)}


def entry():
    start("sand", 0.0)
    reach(20.0, ZONE_X0 - 5.0)
    step_forces.clear()
    while state.position[0] < ZONE_X0 + 10.0:
        cmd.velocity, cmd.acceleration = 20.0, 0.0
        frame()
    entry_forces = list(step_forces)
    step_forces.clear()
    while state.position[0] < ZONE_X0 + 20.0:
        cmd.velocity, cmd.acceleration = 20.0, 0.0
        frame()
    steady = [abs(b - a) for a, b in zip(step_forces, step_forces[1:])]
    f = entry_forces
    jumps = [abs(b - a) for a, b in zip(f, f[1:])]
    return {"max_total_n": round(max(f), 0), "max_step_change_n": round(max(jumps), 0),
            "steady_max_step_change_n": round(max(steady), 0),
            "steady_mean_n": round(sum(step_forces) / len(step_forces), 0),
            "substeps_to_full": next((i for i, v in enumerate(f) if v > 0.9 * max(f)), None)
            - next((i for i, v in enumerate(f) if v > 0), 0),
            "blend_m": round(1.5 * 2 * rig.vehicle.wheel_radius, 2)}


def rut(line_y=0.3):
    """8 m/s through the sand_rut lane on one line."""
    veh.teleport(((60.0, LANES["sand_rut"] + line_y, 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
    layer.reset(veh.path)
    for _ in range(20):
        cmd.velocity, cmd.acceleration = 0.0, 0.0
        frame()
    print(f"SPIKE-PROGRESS rut start pos {[round(c, 2) for c in state.position]}", flush=True)
    reach(8.0, ZONE_X0 + 10.0)
    print(f"SPIKE-PROGRESS rut reached pos {[round(c, 2) for c in state.position]}", flush=True)
    n0, f0 = rebuilt[0], len(flush_s)
    rss0 = int(open("/proc/self/statm").read().split()[1]) * 4096
    hits, bodies, vs, res = [], [], [], []
    n_frames = 0
    while state.position[0] < RUT_X1 - 15.0 and n_frames < 600:
        n_frames += 1
        if n_frames % 40 == 0:
            print(f"SPIKE-PROGRESS rut frame {n_frames} pos {[round(c, 2) for c in state.position]}"
                  f" v {state.velocity[0]:.2f} on_ground "
                  f"{[c.on_ground for c in veh.contacts] if veh.contacts else None}", flush=True)
        cmd.velocity = 8.0
        cmd.acceleration = max(-3.0, min(3.0, 1.5 * (8.0 - state.velocity[0])))
        frame()
        if veh.contacts:
            hits.append(sum(c.position[2] for c in veh.contacts[2:]) / 2)
        bodies.append(state.position[2])
        vs.append(state.velocity[0])
        s = soil_now()
        if s:
            res.append(s["res_n"])
    drops = [a - b for a, b in zip(vs, vs[1:])]
    fl = sorted(flush_s[f0:])
    late = len(hits) // 2
    return {"v_mean": round(sum(vs) / len(vs), 2), "largest_drop_per_frame": round(max(drops), 3),
            "rear_hit_z_cm": round(sum(hits[late:]) / len(hits[late:]) * 100, 2),
            "body_z_cm": round(sum(bodies[late:]) / len(bodies[late:]) * 100, 2),
            "res_n": round(sum(res) / max(len(res), 1), 0),
            "tiles_rebuilt": rebuilt[0] - n0,
            "rss_growth_mb": round((int(open("/proc/self/statm").read().split()[1]) * 4096 - rss0)
                                   / 1e6, 1),
            "flush_ms": {"p50": round(fl[len(fl) // 2] * 1e3, 1) if fl else None,
                         "max": round(fl[-1] * 1e3, 1) if fl else None, "frames": len(fl)},
            "rut_cm_at_130": round(layer.grids["sand_rut"].depth(
                130.0, LANES["sand_rut"] + line_y + rig.vehicle.wheel_tread / 2) * 100, 2)}


def progress(name):
    rss = int(open("/proc/self/statm").read().split()[1]) * 4096 / 1e9
    print(f"SPIKE-PROGRESS {name} rss {rss:.1f} GB t {time.perf_counter() - T0:.0f} s", flush=True)


T0 = time.perf_counter()
out = {}
out["kernel"] = "warp" if soil_model.SoilGrid(surfaces.load_classes()["sand"].soil).warp \
    else "numpy"
ONLY = os.environ.get("SOIL_SPIKE_ONLY")  # e.g. "rut": just those cases
for lane in ("asphalt", "sand", "dirt", "sand_cf") if not ONLY else ():
    progress(f"coast_{lane}")
    out[f"coast_{lane}"] = coast(lane)
layer.fresh_soil()
for lane in ("asphalt", "sand", "dirt") if not ONLY else ():
    progress(f"hold_{lane}")
    out[f"hold_{lane}"] = hold(lane)
if not ONLY:
    progress("hold_sand_2")
    out["hold_sand_2"] = hold("sand")
layer.fresh_soil()
for lane in ("asphalt", "sand", "dirt") if not ONLY else ():
    progress(f"launch_{lane}")
    out[f"launch_{lane}"] = launch(lane)
layer.fresh_soil()
for lane in ("asphalt", "sand", "dirt") if not ONLY else ():
    progress(f"brake_{lane}")
    out[f"brake_{lane}"] = brake(lane)
layer.fresh_soil()
if not ONLY:
    progress("entry")
    out["entry"] = entry()
layer.fresh_soil()
out["pad_body_z_cm"] = None
start("asphalt", 0.0)
out["pad_body_z_cm"] = round(state.position[2] * 100, 2)
progress("rut_1")
out["rut_1"] = rut()
progress("rut_2")
out["rut_2"] = rut()
cost = {}
for k, v in update_s.items():
    v = sorted(v)
    if v:
        cost[k] = {"p50_us": round(v[len(v) // 2] * 1e6, 1),
                   "p99_us": round(v[int(len(v) * 0.99)] * 1e6, 1), "n": len(v)}
out["cost"] = cost
print("SPIKE-RESULT " + json.dumps(out), flush=True)
os._exit(0)
