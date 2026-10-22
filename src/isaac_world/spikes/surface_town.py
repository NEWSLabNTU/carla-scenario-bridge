"""Town drive (roadmap 047 S1): aib's Model 3 on Town01's SimReady world through the surface
layer, scripted, at speeds Autoware's routes never reach. What the pad (`surface_drive.py`)
cannot show: the town's own colliders and SimReady labels resolving to classes, the map
frame lining up, and the same surface cases on real road geometry.

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/surface_town.py \
        [<Town01_physics.usda>]          # default $CARLA_USD/Town01/Town01_physics.usda
    # prints one SPIKE-RESULT json

The lane is Town01's lanelet 13397, the east road heading +y (x = 396.35, y -320 -> -10).
Each case starts at y = -315, reaches its speed, and brakes at full demand (ABS on) at
y = -150; its zones cover the lane from y = -160 on (the step's from -140). `no_map` runs with
no surface layer at all, `dry` with a map of label defaults only: they must match.
"""
import json
import math
import os
import sys
import time

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

from aib.control import GEAR_DRIVE, Command, State  # noqa: E402
from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene  # noqa: E402
from aib.rig import EgoConfig  # noqa: E402
from isaac_world import surfaces  # noqa: E402
from isaac_world.surface_layer import SurfaceLayer  # noqa: E402

USD = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.environ.get("CARLA_USD", "/mnt/disk1/carla-usd"), "Town01", "Town01_physics.usda")
FRAME, SUB = 0.05, 3
dt = FRAME / SUB
LANE_X, START_Y, BRAKE_Y = 396.35, -315.0, -150.0
HEADING = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))  # +y
DEFAULTS = {"Road": "asphalt", "RoadLine": "asphalt", "SideWalk": "concrete",
            "Terrain": "grass", "Ground": "dirt", "unlabelled": "asphalt"}


def lane(y0=-160.0, x0=LANE_X - 2.2, x1=LANE_X + 2.2):
    return [[x0, y0], [x1, y0], [x1, 0.0], [x0, 0.0]]


CASES = {  # name: (speed, zones, defaults override)
    "no_map": (20.0, None, None),
    "dry": (20.0, [], None),
    "wet": (20.0, [{"id": "wet", "wetness": "wetWithPuddles", "polygon": lane()}], None),
    "gravel": (20.0, [{"id": "gravel", "class": "gravel", "polygon": lane()}], None),
    "road_gravel": (20.0, [], {"Road": "gravel"}),  # by label alone, no zone
    "ice": (12.0, [{"id": "ice", "class": "ice", "polygon": lane()}], None),
    "step": (20.0, [{"id": "step", "wetness": "wetWithPuddles", "polygon": lane(-140.0)}], None),
    # heading +y the left wheels are at x < LANE_X
    "mu_split": (20.0, [{"id": "puddle", "water_depth": 0.006,
                         "polygon": lane(x1=LANE_X)}], None),
    "flood_15": (15.0, [{"id": "flood", "water_depth": 0.010, "polygon": lane()}], None),
    "flood_25": (25.0, [{"id": "flood", "water_depth": 0.010, "polygon": lane()}], None),
}

t_load = time.monotonic()
world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(usd_path=USD, prim_path="/World/env")
if not any(pr.IsA(UsdPhysics.Scene) for pr in stage.Traverse()):
    UsdPhysics.Scene.Define(stage, "/World/physicsScene")
ensure_vehicle_scene(stage)
cfg = EgoConfig.load(os.environ.get("AIB_EGO") or os.path.join(
    os.path.dirname(__file__), "../../autoware_isaac_bridge/src/aib/config/ego.yaml"))
rig = cfg.load_rig()
veh = Vehicle(stage, "/World/ego", rig.vehicle, rig.box_center, rig.box_dims,
              ((LANE_X, START_Y, 0.3), HEADING))
world.reset()
world.play()
physx = omni.physx.get_physx_interface()
state = State()
cmd = Command(received=True, gear=GEAR_DRIVE)
out = {"usd": USD, "load_s": round(time.monotonic() - t_load, 1)}
layer = None
update_s = []


def frame():
    for _ in range(SUB):
        veh.apply(cmd, dt, state)
        world.step(render=False)
        veh.sample(physx, dt, state)
        if layer is not None:
            t0 = time.perf_counter()
            layer.update(veh, state.velocity[0], dt)
            update_s.append(time.perf_counter() - t0)


def yaw():
    w, x, y, z = state.quat
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def case(name):
    global layer
    speed, zones, override = CASES[name]
    if zones is None:
        layer = None
    else:
        smap = surfaces.parse({"defaults": {**DEFAULTS, **(override or {})}, "zones": zones},
                              path=f"<{name}>")
        layer = SurfaceLayer(stage, smap)
    veh.teleport(((LANE_X, START_Y, 0.3), HEADING), physx)
    cmd.tire_angle = 0.0
    for _ in range(40):
        cmd.velocity, cmd.acceleration = 0.0, 0.0
        frame()
    while state.position[1] < BRAKE_Y:
        cmd.velocity = speed
        cmd.acceleration = max(-3.0, min(3.0, 1.5 * (speed - state.velocity[0])))
        # hold the lane: steer on the lateral offset and heading
        cmd.tire_angle = max(-0.1, min(0.1, -0.05 * (state.position[0] - LANE_X)
                                       - 0.5 * math.radians(yaw() - 90.0)))
        frame()
    cmd.tire_angle = 0.0  # straight through the stop: mu-split yaw shows
    p0, v0, yaw0 = state.position, state.velocity[0], yaw()
    trace, mus, t = [], [], 0.0
    while state.velocity[0] > 0.3 and t < 30.0:
        cmd.velocity, cmd.acceleration = 0.0, -9.81
        frame()
        t += FRAME
        trace.append((round(state.position[1] - p0[1], 2), round(state.velocity[0], 2)))
        mus.append([round(m, 3) for m in veh.friction_values])
    hi = next((i for i, (_, v) in enumerate(trace) if v < v0 - 1.0), 0)
    lo = next((i for i, (_, v) in enumerate(trace) if v < 3.0), len(trace) - 1)
    under = [s and [s.cls, s.wetness, s.water_depth, s.zone] for s in
             layer.surfaces_under(veh)] if layer else None
    labels = [layer.label(c.shape) for c in veh.contacts] if layer else None
    return {"v0": round(v0, 2), "stop_m": trace[-1][0], "t_s": round(t, 2),
            "decel": round((trace[hi][1] - trace[lo][1]) / max((lo - hi) * FRAME, 1e-6), 2),
            "lateral_m": round(state.position[0] - p0[0], 3), "yaw_deg": round(yaw() - yaw0, 2),
            "z": round(p0[2], 3), "mu_mid": mus[len(mus) // 2], "under": under,
            "labels": labels, "shapes": sorted({c.shape for c in veh.contacts}),
            "trace": trace[::4]}


for name in CASES:
    out[name] = case(name)
u = sorted(update_s)
out["update_us"] = {"p50": round(u[len(u) // 2] * 1e6, 1),
                    "p99": round(u[int(len(u) * 0.99)] * 1e6, 1), "n": len(u)}
print("SPIKE-RESULT " + json.dumps(out))
app.close()
