"""Does the RTX lidar see the ruts the wheels feel (roadmap 047 S3)? And what does a tile
rebuild cost with an RTX sensor rendering?

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/soil_lidar.py
    # prints one SPIKE-RESULT json

A sand zone whose ground is the server's tiles (`soil_tiles.SoilTiles`, `ground_z` 0) on an
empty stage; two wheel tracks driven into its grid (`soil.SoilGrid`, 4500 N, 20 % slip, as a
car's pair 1.6 m apart) from x = 2 to 14 m; then the tiles rebuilt, a frame's budget at a time,
rendering, and an OS1 lidar (aib's) 2 m up at the origin. Per track: the rut depth in the grid
and the lidar's ground points over the track's centre line against those between the tracks.
"""
import json
import os
import sys
import time

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

from isaac_world import soil, surfaces  # noqa: E402
from isaac_world.soil_tiles import SoilTiles  # noqa: E402

H = 2.0
TRACKS = (-0.8, 0.8)
world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 20)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

smap = surfaces.parse({"zones": [{"id": "pit", "class": "sand", "soil": "grid", "ground_z": 0.0,
                                  "polygon": [[-4, -4], [20, -4], [20, 4], [-4, 4]]}]},
                      "spike.yaml")
zone = smap.zones[0]
grid = soil.SoilGrid(smap.table["sand"].soil)
tiles = SoilTiles(stage, zone, grid.h, budget=4)

from isaacsim.sensors.experimental.rtx import Lidar as RtxLidar  # noqa: E402

mount = UsdGeom.Xform.Define(stage, "/World/mount")
mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, H))
lidar = RtxLidar.create("/World/mount/lidar", config="OS1", variant="OS1_REV7_128ch20hz1024res")
lidar_rp = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
points = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
points.attach([lidar_rp.path])
world.reset()
world.play()


def frames(n):
    for _ in range(n):
        world.step(render=True)


def scan(n=10):
    """The lidar's points (map frame) over n frames."""
    out = []
    for _ in range(n):
        world.step(render=True)
        d = points.get_data()
        p = np.asarray(d.get("data", []) if isinstance(d, dict) else [], dtype=np.float64)
        if p.ndim == 2 and len(p):
            q = p[:, :3].copy()
            q[:, 2] += H  # the annotator's points are in the sensor frame
            out.append(q)
    return np.concatenate(out) if out else np.zeros((0, 3))


def ground_z(pts, y, half=0.06, x0=4.0, x1=12.0):
    m = (np.abs(pts[:, 1] - y) < half) & (pts[:, 0] > x0) & (pts[:, 0] < x1) \
        & (np.abs(pts[:, 2]) < 0.5)
    return (round(float(np.median(pts[m, 2])) * 100, 2), int(m.sum())) if m.any() else (None, 0)


out = {}
frames(30)  # RTX warm-up
flat = scan()
out["flat"] = {str(y): ground_z(flat, y) for y in (*TRACKS, 0.0)}
# Drive the two tracks, as a car at 8 m/s would, a physics step at a time.
dt = 1 / 60
x = 2.0
while x < 14.0:
    grid.begin_step()
    for k, y in enumerate(TRACKS):
        grid.wheel(k, x, y, 0.0, 4500.0, 0.37, 0.235, 8.0 / 0.8, 8.0, dt)
    x += 8.0 * dt
rebuild = []
queued = len({(bx // 2, by // 2) for bx, by in grid.dirty_blocks()})
while grid.dirty_blocks():
    t0 = time.perf_counter()
    done = tiles.update(grid)
    t_edit = time.perf_counter() - t0
    t0 = time.perf_counter()
    world.step(render=True)  # the cook happens here
    rebuild.append({"tiles": len(done), "edit_ms": round(t_edit * 1e3, 1),
                    "frame_ms": round((time.perf_counter() - t0) * 1e3, 1)})
frames(5)
base = []
for _ in range(10):
    t0 = time.perf_counter()
    world.step(render=True)
    base.append((time.perf_counter() - t0) * 1e3)
rutted = scan()
out["rut_depth_cm"] = {str(y): round(grid.depth(8.0, y) * 100, 2) for y in TRACKS}
out["rutted"] = {str(y): ground_z(rutted, y) for y in (*TRACKS, 0.0)}
out["tiles_to_rebuild"] = queued
out["rebuild_frames"] = rebuild
out["render_frame_ms_without_rebuild"] = round(float(np.median(base)), 1)
out["lidar_points_per_scan"] = int(len(rutted) / 10)
print("SPIKE-RESULT " + json.dumps(out), flush=True)
os._exit(0)
