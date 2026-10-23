"""Soil on a town (roadmap 047 S3): a dirt zone with its own ground on Town01's
SimReady world, the town's ground cut out under it at load (`soil_tiles.cut_stage`), aib's
Model 3 driven through it on town01_ego_drive's road. What the pad cannot show: the cut on real
road tiles, lane markings, a crosswalk and the landscape under them, and the frame time of a
whole town with a zone in it.

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/soil_town.py \\
        [<Town01_physics.usda>]          # default $CARLA_USD/Town01/Town01_physics.usda
    # prints one SPIKE-RESULT json

The road is town01_ego_drive's, y = -130 heading -x; the zone is x 145.6..169.6,
y -134.4..-128.0 (whole 1.6 m tiles), dirt (sandy loam) on the grid. In order:

  frame_ms_no_zone   frame wall time driving x 205 -> 180 at 8 m/s, no surface layer
  cut                meshes cut and their faces before -> after, and the time it took
  rays               downward rays inside the zone (they must hit a soil tile at z 0) and
                     just outside it (the town's road)
  frame_ms_zone_out  the same drive with the zone loaded, the car outside it
  pass_1, pass_2     8 m/s through the zone twice on one line: speed, ground under the rear
                     wheels, resistance, tiles rebuilt, frame time in the zone
  uncut              the rays again after the zone is removed: the town's road is back
"""
import json
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
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import PhysicsSchemaTools, UsdPhysics  # noqa: E402

from aib.control import GEAR_DRIVE, Command, State  # noqa: E402
from aib.isaac.vehicle import Vehicle, ensure_vehicle_scene  # noqa: E402
from aib.rig import EgoConfig  # noqa: E402
from isaac_world import surfaces  # noqa: E402
from isaac_world.soil_tiles import SoilTiles  # noqa: E402
from isaac_world.surface_layer import SurfaceLayer  # noqa: E402

USD = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.environ.get("CARLA_USD", "/mnt/disk1/carla-usd"), "Town01", "Town01_physics.usda")
FRAME, SUB = 0.05, 3
dt = FRAME / SUB
LINE_Y = -130.0
HEADING = (0.0, 0.0, 0.0, 1.0)  # -x
ZONE = [[145.6, -134.4], [169.6, -134.4], [169.6, -128.0], [145.6, -128.0]]
SURFACES = {"defaults": {"Road": "asphalt", "RoadLine": "asphalt", "SideWalk": "concrete",
                         "Terrain": "grass", "Ground": "dirt"},
            "zones": [{"id": "dirt_road", "class": "dirt", "soil": "grid", "ground_z": 0.0,
                       "polygon": ZONE}]}

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
              ((210.0, LINE_Y, 0.3), HEADING))
world.reset()
world.play()
physx = omni.physx.get_physx_interface()
sim = omni.physx.get_physx_simulation_interface()
query = omni.physx.get_physx_scene_query_interface()
for _ in range(10):
    world.step(render=False)
out = {"load_s": round(time.monotonic() - t_load, 1)}
state = State()
cmd = Command(received=True, gear=GEAR_DRIVE)
layer = None
frame_s = []


def apply_force(path, force, position):
    sim.apply_force_at_pos(omni.usd.get_context().get_stage_id(),
                           PhysicsSchemaTools.sdfPathToInt(path), force, position)


def frame():
    t0 = time.perf_counter()
    for _ in range(SUB):
        veh.apply(cmd, dt, state)
        world.step(render=False)
        veh.sample(physx, dt, state)
        if layer is not None:
            layer.begin_step()
            layer.update(veh, state.velocity[0], dt)
    if layer is not None:
        layer.flush()
    frame_s.append(time.perf_counter() - t0)


def start(x=210.0):
    veh.teleport(((x, LINE_Y, 0.3), HEADING), physx)
    if layer is not None:
        layer.reset(veh.path)
    for _ in range(30):
        cmd.velocity, cmd.acceleration = 0.0, 0.0
        frame()


def drive_to(speed, x_end, record=None):
    """Hold `speed` heading -x until x < x_end; `record(frame index)` after each frame."""
    n = 0
    while state.position[0] > x_end and n < 2000:
        cmd.velocity = speed
        cmd.acceleration = max(-3.0, min(3.0, 1.5 * (speed - state.velocity[0])))
        frame()
        n += 1
        if record:
            record(n)


def frame_ms(x0=205.0, x1=180.0):
    start(215.0)
    drive_to(8.0, x0)
    n0 = len(frame_s)
    drive_to(8.0, x1)
    f = sorted(frame_s[n0:])
    return {"p50": round(f[len(f) // 2] * 1e3, 2), "p95": round(f[int(len(f) * 0.95)] * 1e3, 2),
            "n": len(f)}


def rays():
    def hit(x, y):
        h = query.raycast_closest((x, y, 2.0), (0.0, 0.0, -1.0), 30.0)
        if not h or not h.get("hit"):
            return None
        return {"z": round(float(h["position"][2]), 3),
                "what": str(h.get("collision", "")).split("/")[-1][:40]}
    return {"inside_lane_left": hit(157.0, -130.8), "inside_crosswalk": hit(157.3, -132.0),
            "inside_edge": hit(146.0, -128.3), "outside_x": hit(172.0, -130.8),
            "outside_y": hit(157.0, -134.9)}


out["frame_ms_no_zone"] = frame_ms()
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "surfaces.yaml")
    with open(path, "w") as f:
        json.dump(SURFACES, f)
    t0 = time.perf_counter()
    tiles = {}
    layer = SurfaceLayer(stage, surfaces.load(path), apply_force=apply_force,
                         make_tiles=lambda z, h, o: tiles.setdefault(
                             z.id, SoilTiles(stage, z, h, o, cut_root="/World/env")))
    out["zone_load_s"] = round(time.perf_counter() - t0, 2)
for _ in range(3):
    world.step(render=False)  # the cut meshes re-cooked
cut = tiles["dirt_road"].cut
out["cut"] = {p.split("/")[-2] + "/" + p.split("/")[-1]: f"{a}->{b}"
              for p, (a, b, _) in cut.items()}
out["rays"] = rays()
out["frame_ms_zone_out"] = frame_ms()


def a_pass():
    start(215.0)
    drive_to(8.0, 175.0)
    hits, vs, res, n0 = [], [], [], len(frame_s)
    gen0 = sum(tiles["dirt_road"].gen.values())

    def rec(_):
        if 165.0 > state.position[0] > 150.0:
            if veh.contacts:
                hits.append(sum(c.position[2] for c in veh.contacts[2:]) / 2)
            vs.append(state.velocity[0])
            under = [s for s in layer.soil_under(veh) if s]
            res.append(sum(s[0].resistance * s[1] for s in under))
    drive_to(8.0, 140.0, rec)
    for _ in range(40):  # let the last tiles rebuild once the car is clear
        cmd.velocity, cmd.acceleration = 0.0, -3.0
        frame()
    f = sorted(frame_s[n0:])
    return {"v_mean": round(sum(vs) / max(len(vs), 1), 2),
            "rear_hit_z_cm": round(sum(hits) / max(len(hits), 1) * 100, 2),
            "res_n": round(sum(res) / max(len(res), 1), 0),
            "tiles_rebuilt": sum(tiles["dirt_road"].gen.values()) - gen0,
            "frame_ms": {"p50": round(f[len(f) // 2] * 1e3, 2),
                         "p95": round(f[int(len(f) * 0.95)] * 1e3, 2)},
            "rut_cm": round(layer.grids["dirt_road"].depth(157.0, LINE_Y - 0.8) * 100, 2)
            if "dirt_road" in layer.grids else None}


out["pass_1"] = a_pass()
out["pass_2"] = a_pass()
layer.remove_tiles()
layer = None
for _ in range(3):
    world.step(render=False)
out["uncut"] = rays()
print("SPIKE-RESULT " + json.dumps(out), flush=True)
os._exit(0)
