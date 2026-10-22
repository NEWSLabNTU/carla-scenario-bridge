"""Drive test (roadmap 047 S1): aib's Model 3 through the surface layer (`csb_isaac.surfaces`,
`csb_isaac.surface_layer`, the world server's own code) on a flat pad, one lane per case.

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/csb_isaac/spikes/surface_drive.py
    # prints one SPIKE-RESULT json

The pad is one mesh labelled Road (SimReady's Q-code), so outside a zone every wheel is on dry
asphalt; zones start at x = 150. Each case brakes at full demand (aib's powertrain, ABS on)
from 20 m/s once the car is in its lane's zone, unless it says otherwise:

  dry        no zone: the baseline
  dry_no_abs dry, aib's powertrain with ABS off (the brakes lock the wheels)
  wet        wetWithPuddles
  gravel     gravel; and gravel_no_abs
  ice        ice
  step       dry braking that runs into a wet zone 10 m later
  mu_split   6 mm of water under the left wheels only, steering held straight
  flood_15 / flood_25   10 mm of water, braking from 15 and 25 m/s (hydroplaning)
  launch_ice / launch_dry   full throttle from rest for 3 s (traction control)
"""
import dataclasses
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
from pxr import Gf, Sdf, UsdGeom, UsdPhysics  # noqa: E402

from aib.control import GEAR_DRIVE, Command, State  # noqa: E402
from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene  # noqa: E402
from aib.rig import EgoConfig  # noqa: E402
from csb_isaac import surfaces  # noqa: E402
from csb_isaac.surface_layer import QCODE_ATTR, SurfaceLayer  # noqa: E402

FRAME, SUB = 0.05, 3
dt = FRAME / SUB
ZONE_X = 150.0
LANES = {"dry": 0.0, "dry_no_abs": 0.0, "wet": 10.0, "gravel": 20.0, "gravel_no_abs": 20.0,
         "ice": 30.0, "launch_ice": 30.0, "launch_dry": 0.0, "mu_split": 40.0, "step": -10.0, "flood_15": -20.0, "flood_25": -20.0}


def band(y0, y1, x0=ZONE_X, x1=900.0):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


SURFACES = {
    "defaults": {"Road": "asphalt"},
    "zones": [
        {"id": "wet", "wetness": "wetWithPuddles", "polygon": band(7, 13)},
        {"id": "gravel", "class": "gravel", "polygon": band(17, 23)},
        {"id": "ice", "class": "ice", "polygon": band(27, 33)},
        {"id": "puddle_left", "water_depth": 0.006, "polygon": band(40.0, 43)},
        {"id": "step_wet", "wetness": "wetWithPuddles", "polygon": band(-13, -7, ZONE_X + 12)},
        {"id": "flood", "water_depth": 0.010, "polygon": band(-23, -17)},
    ],
}

world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
stage = omni.usd.get_context().get_stage()
mesh = UsdGeom.Mesh.Define(stage, "/World/env/Road_pad/mesh")
mesh.CreatePointsAttr([Gf.Vec3f(-100, -60, 0), Gf.Vec3f(900, -60, 0), Gf.Vec3f(900, 60, 0),
                       Gf.Vec3f(-100, 60, 0)])
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
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "surfaces.yaml")
    with open(path, "w") as f:
        json.dump(SURFACES, f)  # JSON is YAML
    layer = SurfaceLayer(stage, surfaces.load(path))
world.reset()
world.play()
physx = omni.physx.get_physx_interface()
state = State()
cmd = Command(received=True, gear=GEAR_DRIVE)
update_s = []


def frame(surface=True):
    for _ in range(SUB):
        veh.apply(cmd, dt, state)
        world.step(render=False)
        veh.sample(physx, dt, state)
        if surface:
            t0 = time.perf_counter()
            layer.update(veh, state.velocity[0], dt)
            update_s.append(time.perf_counter() - t0)


def yaw():
    w, x, y, z = state.quat
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def case(name, speed=20.0, abs_on=True):
    y = LANES[name]
    veh.powertrain.p = dataclasses.replace(veh.powertrain.p, abs_enabled=abs_on)
    start = -60.0 if speed > 20 else 0.0
    veh.teleport(((start, y, 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
    layer.reset(veh.path)
    cmd.tire_angle = 0.0
    for _ in range(30):
        cmd.velocity, cmd.acceleration = 0.0, 0.0
        frame()
    brake_x = ZONE_X + 2.0
    while state.position[0] < brake_x:
        cmd.velocity = speed
        cmd.acceleration = max(-3.0, min(3.0, 1.5 * (speed - state.velocity[0])))
        frame()
    x0, y0, v0, yaw0 = state.position[0], state.position[1], state.velocity[0], yaw()
    trace, mus, t = [], [], 0.0
    while state.velocity[0] > 0.3 and t < 30.0:
        cmd.velocity, cmd.acceleration = 0.0, -9.81
        frame()
        t += FRAME
        trace.append((round(state.position[0] - x0, 2), round(state.velocity[0], 2)))
        mus.append([round(m, 3) for m in veh.friction_values])
    hi = next((i for i, (_, v) in enumerate(trace) if v < v0 - 1.0), 0)
    lo = next((i for i, (_, v) in enumerate(trace) if v < 3.0), len(trace) - 1)
    decel = (trace[hi][1] - trace[lo][1]) / max((lo - hi) * FRAME, 1e-6)
    under = [s and (s.cls, s.wetness, s.water_depth, s.zone) for s in layer.surfaces_under(veh)]
    return {"v0": round(v0, 2), "stop_m": trace[-1][0], "t_s": round(t, 2),
            "decel": round(decel, 2), "lateral_m": round(state.position[1] - y0, 3),
            "yaw_deg": round(yaw() - yaw0, 2), "mu_first": mus[0], "mu_mid": mus[len(mus) // 2],
            "under": under, "trace": trace[::4]}


def launch(name, x=ZONE_X + 20.0):
    """Full throttle from rest at `x` in the lane's zone for 3 s: speed, and the rear
    (driven) wheels' largest slip once moving."""
    veh.teleport(((x, LANES[name], 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
    layer.reset(veh.path)
    for _ in range(30):
        cmd.velocity, cmd.acceleration = 0.0, 0.0
        frame()
    slips = []
    for _ in range(60):
        cmd.velocity, cmd.acceleration = 30.0, 4.0
        frame()
        if state.velocity[0] > 1.0 and veh.wheel_states:
            slips.append(max(w.slip for w in veh.wheel_states[2:]))
    return {"v_3s": round(state.velocity[0], 2), "max_slip": round(max(slips), 3),
            "mean_slip": round(sum(slips) / len(slips), 3)}


out = {}
out["dry"] = case("dry")
out["dry_no_abs"] = case("dry_no_abs", abs_on=False)
out["wet"] = case("wet")
out["gravel"] = case("gravel")
out["gravel_no_abs"] = case("gravel_no_abs", abs_on=False)
out["ice"] = case("ice")
out["step"] = case("step")
out["mu_split"] = case("mu_split")
out["flood_15"] = case("flood_15", speed=15.0)
out["flood_25"] = case("flood_25", speed=25.0)
u = sorted(update_s)
veh.powertrain.p = dataclasses.replace(veh.powertrain.p, abs_enabled=True)
out["launch_dry"] = launch("launch_dry", x=0.0)
out["launch_ice"] = launch("launch_ice")
out["update_us"] = {"p50": round(u[len(u) // 2] * 1e6, 1), "p99": round(u[int(len(u) * 0.99)] * 1e6, 1),
                    "n": len(u)}
# A frame with and without the surface layer, on dry asphalt at 20 m/s
veh.teleport(((0.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
for _ in range(150):
    cmd.velocity, cmd.acceleration = 15.0, 1.0
    frame()
for flag in (True, False, True, False):
    t0 = time.perf_counter()
    for _ in range(60):
        cmd.velocity, cmd.acceleration = 15.0, 0.0
        frame(surface=flag)
    out[f"frame_ms_{'with' if flag else 'without'}"] = round((time.perf_counter() - t0) / 60 * 1e3, 2)
print("SPIKE-RESULT " + json.dumps(out))
app.close()
