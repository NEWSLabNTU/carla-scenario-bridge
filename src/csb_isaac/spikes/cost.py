"""Phase 036 spike: what one Autoware sensor set costs in Isaac Sim, and how it scales.

acb's set on CARLA (acb_bridge/config/vehicle_config.yaml): one 128-channel lidar at 20 Hz
(2.6 M points/s), one 1280x720 camera at 10 Hz, IMU, GNSS. Here each set is an RTX lidar
(Ouster OS1 REV7 128ch 20 Hz 1024 res: the same point rate) and an RTX camera, each published
over ROS 2 on its own domain (set k on domain k), mounted on a static box in a field of boxes.
Physics at 60 Hz, one render every 3 steps (20 Hz, the lidar rate); the camera publishes every
other render (10 Hz). Prints wall time per render frame (real time = 50 ms) and GPU memory.

    python cost.py --sets 2 --frames 200
"""
import argparse
import subprocess
import time

ap = argparse.ArgumentParser()
ap.add_argument("--sets", type=int, default=1)
ap.add_argument("--frames", type=int, default=200)  # render frames (20 Hz)
ap.add_argument("--camera", type=int, default=1)
ap.add_argument("--lidar", type=int, default=1)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.experimental.rtx")
app.update()
from isaacsim.sensors.experimental.rtx import Lidar  # noqa: E402


def gpu_mib():
    out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout
    return int(out.split()[0])


mem0 = gpu_mib()
world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
rng = np.random.default_rng(0)
for i in range(200):  # something for the sensors to see
    x, y = rng.uniform(-60, 60, 2)
    world.scene.add(FixedCuboid(prim_path=f"/World/box_{i}", name=f"box_{i}", position=[x, y, 1.0],
                                scale=[2.0, 2.0, rng.uniform(1, 4)]))
stage = world.stage
keys = og.Controller.Keys
for k in range(1, args.sets + 1):
    base = f"/World/ego_{k}"
    world.scene.add(FixedCuboid(prim_path=base, name=f"ego_{k}", position=[0.0, 8.0 * k, 0.75], scale=[4.5, 1.8, 1.5]))
    nodes = [("tick", "omni.graph.action.OnPlaybackTick"), ("context", "isaacsim.ros2.bridge.ROS2Context")]
    values = [("context.inputs:useDomainIDEnvVar", False), ("context.inputs:domain_id", k)]
    links = []
    if args.lidar:
        lidar = Lidar.create(f"/World/lidar_{k}", config="OS1", variant="OS1_REV7_128ch20hz1024res")
        UsdGeom.XformCommonAPI(stage.GetPrimAtPath(lidar.paths[0])).SetTranslate(Gf.Vec3d(0.0, 8.0 * k, 2.2))
        rp = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
        nodes.append(("lidar", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
        values += [("lidar.inputs:renderProductPath", rp.path), ("lidar.inputs:type", "point_cloud"),
                   ("lidar.inputs:topicName", "/sensing/lidar/top/pointcloud_raw"),
                   ("lidar.inputs:frameId", "velodyne_top")]
        links += [("tick.outputs:tick", "lidar.inputs:execIn"), ("context.outputs:context", "lidar.inputs:context")]
    if args.camera:
        cam = UsdGeom.Camera.Define(stage, f"/World/camera_{k}")
        cam.CreateFocalLengthAttr(18.0)
        UsdGeom.XformCommonAPI(cam.GetPrim()).SetTranslate(Gf.Vec3d(2.5, 8.0 * k, 1.8))
        UsdGeom.XformCommonAPI(cam.GetPrim()).SetRotate(Gf.Vec3f(90.0, 0.0, -90.0))
        rpc = rep.create.render_product(f"/World/camera_{k}", resolution=(1280, 720))
        nodes.append(("camera", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        values += [("camera.inputs:renderProductPath", rpc.path), ("camera.inputs:type", "rgb"),
                   ("camera.inputs:topicName", "/sensing/camera/camera6/image_raw"),
                   ("camera.inputs:frameId", "camera6/camera_optical_link"), ("camera.inputs:frameSkipCount", 1)]
        links += [("tick.outputs:tick", "camera.inputs:execIn"), ("context.outputs:context", "camera.inputs:context")]
    og.Controller.edit({"graph_path": f"/World/ros_{k}", "evaluator_name": "execution"},
                       {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: links})

world.reset()
world.play()
for _ in range(30):  # warm-up: shader compiles, first renders
    world.step(render=True)
mem1 = gpu_mib()
t = []
for f in range(args.frames):
    t0 = time.perf_counter()
    world.step(render=False)
    world.step(render=False)
    world.step(render=True)
    t.append(1e3 * (time.perf_counter() - t0))
t = np.array(t)
print(f"COST sets {args.sets} lidar {args.lidar} camera {args.camera}: per 20 Hz frame median {np.median(t):.1f} ms "
      f"p95 {np.percentile(t, 95):.1f} ms (real time 50 ms); GPU {mem1 - mem0} MiB for the scene "
      f"({mem1} MiB used)", flush=True)
app.close()
