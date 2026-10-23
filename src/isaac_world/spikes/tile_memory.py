"""Where does a soil tile rebuild's memory go, and which way of rebuilding keeps none
(roadmap 047 S3, docs/issues/001)?

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/tile_memory.py
    # prints one SPIKE-RESULT json

One 1.6 m tile (33 x 33 vertices) rebuilt N times per variant, a physics step after each,
alternating between two rut shapes; per variant the resident memory gained per rebuild (after
a warm-up) and whether a downward ray onto the tile meets the new shape every time:

  new_path      a new prim at a new path, the old one removed (soil_tiles today)
  same_path     the old prim removed, a new one defined at the same path
  pair_toggle   two prims at fixed paths, the new shape written into the idle one and its
                collision enabled (`physics:collisionEnabled`) while the other's is disabled
  edit_enable   one prim: points written, collision disabled then enabled in the same step
  edit_approx   one prim: points written, then the cooking forced by touching the mesh
                collision approximation attribute
  new_path_nophys  new_path without CollisionAPI (render and USD only: whose memory is it?)
  release_local / release_runtime   same_path, then the cooking interface's
                local mesh cache / the private runtime mesh cache released after the step
  edit_refresh  one prim: points written, the prim put in the cooking refresh set
  edit_refresh_release  edit_refresh, then both caches released after the step
"""
import json
import os
import sys
import time

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import carb  # noqa: E402

if os.environ.get("TILE_NO_UJITSO") == "1":  # PhysX's legacy cooking, not UJITSO's
    carb.settings.get_settings().set_bool("/physics/cooking/ujitsoCollisionCooking", False)
import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Sdf, UsdGeom, UsdPhysics, Vt  # noqa: E402

N = int(os.environ.get("TILE_N", "300"))
WARM = 30
CELLS = int(os.environ.get("TILE_CELLS", "32"))
SIDE = 1.6
world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
    UsdPhysics.Scene.Define(stage, "/World/physicsScene")
query = omni.physx.get_physx_scene_query_interface()
i, j = np.meshgrid(np.arange(CELLS), np.arange(CELLS), indexing="ij")
a = (i * (CELLS + 1) + j).ravel()
IDX = Vt.IntArray.FromNumpy(np.stack([a, a + CELLS + 1, a + CELLS + 2, a + 1], 1).ravel()
                            .astype(np.int32))
CNT = Vt.IntArray([4] * CELLS * CELLS)


UNIQUE = os.environ.get("TILE_UNIQUE", "1") == "1"


def shape(k, x0):
    """Tile vertices at x0 with a rut whose depth alternates 5 / 10 cm (k even / odd), plus a
    hair more each time when UNIQUE (every shape new to PhysX's cooking, as in a drive)."""
    xs = x0 + np.linspace(0, SIDE, CELLS + 1)
    ys = np.linspace(0, SIDE, CELLS + 1)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    d = (0.05 if k % 2 == 0 else 0.10) + (1e-5 * (k + 2) if UNIQUE else 0.0)
    z = np.where(np.abs(gy - 0.8) < 0.15, -d, 0.0)
    return Vt.Vec3fArray.FromNumpy(np.stack([gx.ravel(), gy.ravel(), z.ravel()], 1)
                                   .astype(np.float32)), d


def define(path, pts, collide=True):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexIndicesAttr(IDX)
    m.CreateFaceVertexCountsAttr(CNT)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")
    return m


def rss():
    return int(open("/proc/self/statm").read().split()[1]) * 4096


def ray(x0):
    h = query.raycast_closest((x0 + 0.8, 0.8, 1.0), (0.0, 0.0, -1.0), 5.0)
    return round(-float(h["position"][2]), 3) if h and h.get("hit") else None


def run(name, x0, rebuild, check=True):
    state = {"gen": 0}
    rebuild(-1, state)  # the first tile
    world.step(render=False)
    seen_ok, base, t_edit, marks = 0, None, [], []
    for k in range(N + WARM):
        if k == WARM:
            base = rss()
        if k > WARM and (k - WARM) % max(1, N // 5) == 0:
            marks.append(round((rss() - base) / 1e6, 1))
        t0 = time.perf_counter()
        depth = rebuild(k, state)
        t_edit.append(time.perf_counter() - t0)
        world.step(render=False)
        if check and k >= WARM and ray(x0) is not None and abs(ray(x0) - depth) < 0.005:
            seen_ok += 1
    t = sorted(t_edit)
    return {"kb_per_rebuild": round((rss() - base) / N / 1e3, 1),
            "ray_sees_new_shape": f"{seen_ok}/{N}" if check else "n/a",
            "edit_ms_p50": round(t[len(t) // 2] * 1e3, 2), "mb_at_fifths": marks}


def new_path(x0, collide=True):
    def f(k, st):
        pts, d = shape(k, x0)
        old = f"/World/t{int(x0)}_{st['gen']}"
        st["gen"] += 1
        define(f"/World/t{int(x0)}_{st['gen']}", pts, collide)
        if k >= 0 or st["gen"] > 1:
            stage.RemovePrim(old)
        return d
    return f


def same_path(x0):
    path = f"/World/same_{int(x0)}"

    def f(k, st):
        pts, d = shape(k, x0)
        if k >= 0:
            stage.RemovePrim(path)
        define(path, pts)
        return d
    return f


def pair_toggle(x0):
    paths = [f"/World/pair_{int(x0)}_a", f"/World/pair_{int(x0)}_b"]

    def f(k, st):
        pts, d = shape(k, x0)
        if k < 0:
            for p in paths:
                define(p, pts)
            UsdPhysics.CollisionAPI(stage.GetPrimAtPath(paths[1])).GetCollisionEnabledAttr() \
                .Set(False)
            st["live"] = 0
            return d
        idle = 1 - st["live"]
        UsdGeom.Mesh(stage.GetPrimAtPath(paths[idle])).GetPointsAttr().Set(pts)
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(paths[idle])).GetCollisionEnabledAttr() \
            .Set(True)
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(paths[st["live"]])).GetCollisionEnabledAttr() \
            .Set(False)
        st["live"] = idle
        return d
    return f


def edit_enable(x0):
    path = f"/World/edit_{int(x0)}"

    def f(k, st):
        pts, d = shape(k, x0)
        if k < 0:
            define(path, pts)
            return d
        prim = stage.GetPrimAtPath(path)
        UsdGeom.Mesh(prim).GetPointsAttr().Set(pts)
        attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        attr.Set(False)
        attr.Set(True)
        return d
    return f


def edit_approx(x0):
    path = f"/World/approx_{int(x0)}"

    def f(k, st):
        pts, d = shape(k, x0)
        if k < 0:
            define(path, pts)
            return d
        prim = stage.GetPrimAtPath(path)
        UsdGeom.Mesh(prim).GetPointsAttr().Set(pts)
        UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Set("none")
        return d
    return f


from omni.physx.bindings import _physx  # noqa: E402

cooking = omni.physx.get_physx_cooking_interface()
cooking_private = _physx.acquire_physx_cooking_private_interface()


def same_path_release(x0, which):
    inner = same_path(x0)

    def f(k, st):
        d = inner(k, st)
        if k >= 0:
            world.step(render=False)  # cooked, then the caches released
            (cooking.release_local_mesh_cache if which == "local"
             else cooking_private.release_runtime_mesh_cache)()
        return d
    return f


def edit_refresh(x0):
    path = f"/World/refresh_{int(x0)}"

    def f(k, st):
        pts, d = shape(k, x0)
        if k < 0:
            define(path, pts)
            return d
        UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Set(pts)
        cooking_private.add_prim_to_cooking_refresh_set(stage.GetPrimAtPath(path).GetPath())
        return d
    return f


def edit_refresh_release(x0):
    inner = edit_refresh(x0)

    def f(k, st):
        d = inner(k, st)
        if k >= 0:
            world.step(render=False)
            cooking_private.release_runtime_mesh_cache()
            cooking.release_local_mesh_cache()
        return d
    return f


world.reset()
world.play()
out = {"n": N}
cases = [("new_path", new_path(0.0)), ("same_path", same_path(10.0)),
         ("pair_toggle", pair_toggle(20.0)), ("edit_enable", edit_enable(30.0)),
         ("edit_approx", edit_approx(40.0)),
         ("release_local", same_path_release(60.0, "local")),
         ("release_runtime", same_path_release(70.0, "runtime")),
         ("edit_refresh", edit_refresh(80.0)),
         ("edit_refresh_release", edit_refresh_release(90.0))]
only = os.environ.get("TILE_ONLY")
for name, f in cases:
    if only and name not in only.split(","):
        continue
    x0 = {"new_path": 0.0, "same_path": 10.0, "pair_toggle": 20.0, "edit_enable": 30.0,
          "edit_approx": 40.0, "release_local": 60.0, "release_runtime": 70.0,
          "edit_refresh": 80.0, "edit_refresh_release": 90.0}[name]
    out[name] = run(name, x0, f)
    print(f"SPIKE-PROGRESS {name} {out[name]}", flush=True)
if not only or "new_path_nophys" in only:
    out["new_path_nophys"] = run("new_path_nophys", 50.0, new_path(50.0, collide=False),
                                 check=False)
print("SPIKE-RESULT " + json.dumps(out), flush=True)
os._exit(0)
