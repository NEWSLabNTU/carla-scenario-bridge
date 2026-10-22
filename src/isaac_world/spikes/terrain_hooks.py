"""Spike (off-road surfaces): which per-wheel terrain hooks Isaac Sim 6.1's PhysX vehicle honours
at run time. docs/design/off-road-surfaces.md has the results (2026-10-22, newslab-server-241).

    ISAAC_ROOT=... run_spike.sh fastrtps <log> terrain_hooks.py   # prints one SPIKE-RESULT json


T1 material zones: asphalt strip then a mud strip, each a PhysicsMaterial in the tire friction
   table; per-wheel GROUND_MATERIAL / TIRE_FRICTION as the car crosses.
T2 live table edit: change the asphalt friction value mid-run.
T3 per-wheel tables: left wheels get their own table; change only it.
T4 sinkage by suspension frame: raise one wheel's suspension frame (wheel sits lower) mid-run.
T5 external force at the wheel contact (soil resistance) via apply_force_at_pos.
T6 live collision mesh: a 41x41 vertex tile under the path; dig a 5 cm rut by editing its
   points, by re-applying CollisionAPI, by replacing the prim; does the wheel's ground hit
   follow, and what does each cost.
"""
import json
import os
import time

os.environ.pop("DISPLAY", None)
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from omni.physx.bindings._physx import (VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION,  # noqa: E402
                                        VEHICLE_WHEEL_STATE_GROUND_MATERIAL,
                                        VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE,
                                        VEHICLE_WHEEL_STATE_TIRE_FRICTION)
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

from aib.control import GEAR_DRIVE, Command, State  # noqa: E402
from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene  # noqa: E402
from aib.rig import EgoConfig  # noqa: E402

FRAME, SUB = 0.05, 3
dt = FRAME / SUB
world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
stage = omni.usd.get_context().get_stage()
out = {}


def material(name, mu):
    path = f"/World/Materials/{name}"
    m = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(mu)
    api.CreateDynamicFrictionAttr(mu)
    return m


def quad_mesh(path, x0, x1, y0, y1, nx=2, ny=2, mat=None):
    xs, ys = np.linspace(x0, x1, nx), np.linspace(y0, y1, ny)
    pts = [Gf.Vec3f(float(x), float(y), 0.0) for y in ys for x in xs]
    idx, cnt = [], []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            idx += [a, a + 1, a + nx + 1, a + nx]
            cnt.append(4)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateFaceVertexCountsAttr(cnt)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    return mesh, xs, ys


asphalt = material("asphalt", 1.0)
mud = material("mud", 0.35)
quad_mesh("/World/ground/asphalt", -20, 40, -10, 10, mat=asphalt)
quad_mesh("/World/ground/mud", 40, 80, -10, 10, mat=mud)
# T6 tile, asphalt-material, 4 x 4 m at x 100..104 (approached on the second run)
quad_mesh("/World/ground/approach", 80, 100, -10, 10, mat=asphalt)
tile, txs, tys = quad_mesh("/World/ground/tile", 100, 104, -2, 2, 41, 41, mat=asphalt)
quad_mesh("/World/ground/after", 104, 140, -10, 10, mat=asphalt)
if not any(pr.IsA(UsdPhysics.Scene) for pr in stage.Traverse()):
    UsdPhysics.Scene.Define(stage, "/World/physicsScene")
ensure_vehicle_scene(stage)
# The spike's own shared table (aib at 9949e4e had one; it now gives each wheel its own).
FRICTION_TABLE = "/World/vehicleTireFriction"
table = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, FRICTION_TABLE)
table.CreateGroundMaterialsRel().SetTargets([asphalt.GetPath(), mud.GetPath()])
table.CreateFrictionValuesAttr([1.0, 0.35])
# T3: a second table for the left wheels, identical at first
left_path = "/World/vehicleTireFrictionLeft"
left = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, left_path)
left.CreateGroundMaterialsRel().SetTargets([asphalt.GetPath(), mud.GetPath()])
left.CreateFrictionValuesAttr([1.0, 0.35])
left.CreateDefaultFrictionValueAttr(1.0)

cfg = EgoConfig.load(os.environ.get("AIB_EGO") or os.path.join(
    os.path.dirname(__file__), "../../autoware_isaac_bridge/src/aib/config/ego.yaml"))
rig = cfg.load_rig()
pose = ((0.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0))
veh = Vehicle(stage, "/World/ego", rig.vehicle, rig.box_center, rig.box_dims, pose)
table.CreateDefaultFrictionValueAttr(1.0)
for i, wp in enumerate(veh.wheel_paths):  # frontLeft, rearLeft: the left table
    PhysxSchema.PhysxVehicleTireAPI(stage.GetPrimAtPath(wp)).GetFrictionTableRel().SetTargets(
        [Sdf.Path(left_path if i % 2 == 0 else FRICTION_TABLE)])
world.reset()
world.play()
physx = omni.physx.get_physx_interface()
sim = omni.physx.get_physx_simulation_interface()
stage_id = omni.usd.get_context().get_stage_id()
body_int = PhysicsSchemaTools.sdfPathToInt(veh.path)
state = State()
cmd = Command(received=True, gear=GEAR_DRIVE)
veh.teleport(pose, physx)


def wheels():
    r = []
    for wp in veh.wheel_paths:
        w = physx.get_wheel_state(wp)
        r.append({"mat": str(w[VEHICLE_WHEEL_STATE_GROUND_MATERIAL]).split("/")[-1],
                  "mu": round(float(w[VEHICLE_WHEEL_STATE_TIRE_FRICTION]), 3),
                  "jounce": round(float(w[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]), 4),
                  "hit_z": round(float(w[VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION][2]), 4)})
    return r


def frame(extra=None):
    for _ in range(SUB):
        veh.apply(cmd, dt, state)
        if extra:
            extra()
        world.step(render=False)
        veh.sample(physx, dt, state)


def drive(speed, frames, extra=None, log=None):
    for _ in range(frames):
        cmd.velocity = speed
        cmd.acceleration = max(-3.0, min(2.0, 1.0 * (speed - state.velocity[0])))
        frame(extra)
        if log is not None:
            log.append((round(state.position[0], 2), round(state.velocity[0], 3),
                        round(state.position[2], 4), wheels()))


for _ in range(40):
    frame()
# --- T1 + T2 + T3 + T4 + T5 on the first run --------------------------------------------
log = []
drive(8.0, 60, log=log)                       # accelerate on asphalt
out["T1_asphalt"] = log[-1]
table.GetFrictionValuesAttr().Set([0.6, 0.35])  # T2 (right wheels' table)
drive(8.0, 2, log=log)
out["T2_after_edit"] = log[-1]
left.GetFrictionValuesAttr().Set([0.3, 0.35])   # T3 (left wheels only)
drive(8.0, 2, log=log)
out["T3_left_only"] = log[-1]
table.GetFrictionValuesAttr().Set([1.0, 0.35])
left.GetFrictionValuesAttr().Set([1.0, 0.35])
drive(8.0, 2, log=log)
# T4: raise front-left suspension frame by 5 cm (a 5 cm sinkage of that wheel)
fl = veh.wheel_paths[0]
att = PhysxSchema.PhysxVehicleWheelAttachmentAPI(stage.GetPrimAtPath(fl))
p0 = att.GetSuspensionFramePositionAttr().Get()
before = wheels()[0]
z_before = state.position[2]
att.GetSuspensionFramePositionAttr().Set(Gf.Vec3f(p0[0], p0[1], p0[2] + 0.05))
drive(8.0, 10, log=log)
out["T4_sink_fl"] = {"before": before, "after": wheels()[0],
                     "body_z_before": round(z_before, 4), "body_z_after": round(state.position[2], 4)}
att.GetSuspensionFramePositionAttr().Set(p0)
drive(8.0, 10, log=log)
# T5: hold 8 m/s, then add 1500 N resistance at each wheel contact for 1 s; compare accel
# requested by the speed hold (it must push harder) and the speed sag.
drive(8.0, 20, log=log)
v0 = state.velocity[0]
F = 1500.0


def resist():
    for w in wheels_hits():
        sim.apply_force_at_pos(stage_id, body_int, (-F, 0.0, 0.0), w)


def wheels_hits():
    return [tuple(float(c) for c in physx.get_wheel_state(wp)[VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION])
            for wp in veh.wheel_paths]


cmd_log = []
for _ in range(20):
    cmd.velocity = 8.0
    cmd.acceleration = 0.0  # no correction: see the force act
    frame(resist)
    cmd_log.append(round(state.velocity[0], 3))
out["T5_force"] = {"v0": round(v0, 3), "v_after_1s": cmd_log[-1],
                   "expected_dv": round(-4 * F / rig.vehicle.mass * 1.0, 3)}
# --- T1: carry on into the mud ------------------------------------------------------------
log2 = []
drive(8.0, 140, log=log2)
cross = [r for r in log2 if r[0] > 38 and r[0] < 46]
out["T1_crossing"] = cross[::3]
# --- T6b: replace the tile prim with a rutted copy (forces a new cook) ----------------
veh.teleport(((95.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
for _ in range(40):
    frame()
half = rig.vehicle.wheel_tread / 2
res = {}
for variant in ("edit_points", "toggle_collision", "new_prim"):
    pts = np.array(tile.GetPointsAttr().Get()) if variant != "new_prim" else None
    t0 = time.perf_counter()
    if variant == "edit_points":
        p = np.array(tile.GetPointsAttr().Get()); m = np.abs(np.abs(p[:, 1]) - half) < 0.15
        p[m, 2] = -0.05
        tile.GetPointsAttr().Set([Gf.Vec3f(*map(float, q)) for q in p])
    elif variant == "toggle_collision":
        prim = tile.GetPrim()
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
        world.step(render=False)
        UsdPhysics.CollisionAPI.Apply(prim)
    else:
        stage.RemovePrim("/World/ground/tile")
        newt, _, _ = quad_mesh("/World/ground/tile2", 100, 104, -2, 2, 41, 41, mat=asphalt)
        p = np.array(newt.GetPointsAttr().Get()); m = np.abs(np.abs(p[:, 1]) - half) < 0.15
        p[m, 2] = -0.05
        newt.GetPointsAttr().Set([Gf.Vec3f(*map(float, q)) for q in p])
    t_edit = time.perf_counter() - t0
    t0 = time.perf_counter(); world.step(render=False); t_step = time.perf_counter() - t0
    # probe: put the car's rear axle on the tile at rest and read the rear wheels' hit z
    veh.teleport(((101.5, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
    for _ in range(20):
        frame()
    w = wheels()
    res[variant] = {"edit_s": round(t_edit, 4), "next_step_s": round(t_step, 4),
                    "rear_hit_z": [w[2]["hit_z"], w[3]["hit_z"]], "body_z": round(state.position[2], 4)}
out["T6b"] = res
# --- T4b: frame raise held 3 s, body height and jounce
veh.teleport(((0.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0)), physx)
for _ in range(40):
    frame()
fl = veh.wheel_paths[0]
att = PhysxSchema.PhysxVehicleWheelAttachmentAPI(stage.GetPrimAtPath(fl))
p0 = att.GetSuspensionFramePositionAttr().Get()
b = (wheels()[0], round(state.position[2], 4))
att.GetSuspensionFramePositionAttr().Set(Gf.Vec3f(p0[0], p0[1], p0[2] + 0.05))
for _ in range(60):
    frame()
out["T4b"] = {"before": b, "after": (wheels()[0], round(state.position[2], 4)),
              "frame_read_back": list(att.GetSuspensionFramePositionAttr().Get())}
print("SPIKE-RESULT " + json.dumps(out))
app.close()
