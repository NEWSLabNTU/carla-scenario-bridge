"""Phase 037 spike: what the world server needs from Isaac, checked before it is written.

1. An entity is an Xform at the SSv2 entity origin with a rigid body (kinematic) and a child
   cube collider offset to the declared box centre. Its pose is written through USD xform ops
   each frame; PhysX's own pose for it (``get_rigidbody_transformation``) must follow.
2. Contact reports between two kinematic bodies (the puppeteered ego and an NPC that drives
   through it), and kinematic vs static, with the scene's report-kinematic-pairs flags.
3. Cost of writing N poses and reading them back from PhysX, per frame.

    python kinematic_contacts.py --npcs 20 --frames 200
"""
import argparse
import time

ap = argparse.ArgumentParser()
ap.add_argument("--npcs", type=int, default=20)
ap.add_argument("--frames", type=int, default=200)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics  # noqa: E402

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
scene = None
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        scene = prim
print("SCENE", scene.GetPath() if scene else None, flush=True)
sapi = PhysxSchema.PhysxSceneAPI.Apply(scene)
print("SCENE attrs", [a for a in sapi.GetSchemaAttributeNames() if "eport" in a], flush=True)
for name in ("reportKinematicKinematicPairs", "reportKinematicStaticPairs"):
    attr = scene.GetAttribute(f"physxScene:{name}")
    if not attr:
        attr = scene.CreateAttribute(f"physxScene:{name}", Sdf.ValueTypeNames.Bool)
    attr.Set(True)


def entity(path, pos, yaw, center, dims, report=False):
    root = UsdGeom.Xform.Define(stage, path)
    t = root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    o = root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    t.Set(Gf.Vec3d(*pos))
    o.Set(Gf.Quatd(np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)))
    rb = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    cube = UsdGeom.Cube.Define(stage, f"{path}/body")
    cube.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(*center))
    UsdGeom.XformCommonAPI(cube.GetPrim()).SetScale(Gf.Vec3f(*dims))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if report:
        cr = PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim())
        cr.CreateThresholdAttr(0.0)
    return t, o


box = ((1.5, 0.0, 0.9), (4.5, 2.1, 1.8))
ego = entity("/World/ego", (0.0, 0.0, 0.0), 0.0, *box, report=True)
npcs = [entity(f"/World/npc_{i}", (-30.0 - 8 * i, 0.0 if i == 0 else 10.0 + i, 0.0), 0.0, *box)
        for i in range(args.npcs)]
# A static wall on npc_1's line, to see kinematic-static contacts.
wall = UsdGeom.Cube.Define(stage, "/World/wall")
UsdGeom.XformCommonAPI(wall.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 11.0, 1.0))
UsdGeom.XformCommonAPI(wall.GetPrim()).SetScale(Gf.Vec3f(1.0, 4.0, 2.0))
UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath("/World/npc_1")).CreateThresholdAttr(0.0)

world.reset()
world.play()
physx = omni.physx.get_physx_interface()
sim = omni.physx.get_physx_simulation_interface()
events = []
write_ms, step_ms, read_ms, err = [], [], [], []
dt = 0.05  # one SSv2 frame: 3 physics steps of 1/60
for f in range(args.frames):
    t0 = time.perf_counter()
    with Sdf.ChangeBlock():
        for i, (t, o) in enumerate(npcs):
            x = -30.0 - 8 * i + 10.0 * dt * f  # 10 m/s along +x
            y = 0.0 if i == 0 else 10.0 + i
            t.Set(Gf.Vec3d(x, y, 0.0))
            o.Set(Gf.Quatd(1, 0, 0, 0))
    t1 = time.perf_counter()
    for s in range(3):
        world.step(render=(s == 2))
    t2 = time.perf_counter()
    worst = 0.0
    for i in range(args.npcs):
        p = physx.get_rigidbody_transformation(f"/World/npc_{i}")
        x = -30.0 - 8 * i + 10.0 * dt * f
        worst = max(worst, abs(p["position"][0] - x))
    t3 = time.perf_counter()
    headers, data = sim.get_contact_report()
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        events.append((f, str(h.type), a0, a1))
    write_ms.append(1e3 * (t1 - t0))
    step_ms.append(1e3 * (t2 - t1))
    read_ms.append(1e3 * (t3 - t2))
    err.append(worst)

p = physx.get_rigidbody_transformation("/World/npc_0")
print("READBACK npc_0", p, flush=True)
print(f"COST npcs {args.npcs}: write p95 {np.percentile(write_ms, 95):.2f} ms, step(3, render last) "
      f"p50 {np.median(step_ms):.2f} p95 {np.percentile(step_ms, 95):.2f} ms, readback p95 "
      f"{np.percentile(read_ms, 95):.2f} ms; physx-vs-commanded x error max {max(err[1:]):.6f} m", flush=True)
print(f"CONTACTS {len(events)}", flush=True)
for e in [e for e in events if "Ground" not in e[3] and "Ground" not in e[2]][:20]:
    print("  ", e, flush=True)
app.close()
