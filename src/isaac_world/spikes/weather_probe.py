"""Weather and time-of-day probe (roadmap 047, after S2): what Isaac Sim 6.1 itself does with
the parts of OpenSCENARIO's Environment swb does not send yet, measured on a test pad.

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/weather_probe.py
    # prints one SPIKE-RESULT json

A flat pad with 2 m boxes at 10-80 m down +x, an RTX lidar (OS1, as aib's ego) and an RGB
camera 2 m up looking down +x. Three probes:

  rain   the lidar's Mie-scattering rain model, `/app/sensors/nv/atmospherics/rainRate` (mm/h):
         points per scan, far returns (> 30 m), and returns from empty air near the sensor
         (false positives: above the pad, short of the first box)
  fog    RTX fog (`/rtx/fog/*`): the camera image's brightness and contrast, and the lidar's
         points, fog off and on
  sky    the dynamic sky (NVIDIA's ClearSky.usd, as Isaac's Environment menu adds it) driven by
         `omni.kit.environment.core`'s sun-study player: the sun's direction and the camera's
         brightness at a few times of day
"""
import json
import math
import os
import sys

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

SKY_URL = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Skies/Dynamic/ClearSky.usd"
BOXES = (10.0, 20.0, 40.0, 60.0, 80.0)
H = 2.0  # sensor height, m
settings = carb.settings.get_settings()
world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 20)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)


def cube(path, center, size):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddScaleOp().Set(Gf.Vec3f(*size))


cube("/World/pad", (50.0, 0.0, -0.05), (200.0, 60.0, 0.1))
for x in BOXES:
    cube(f"/World/box_{int(x)}", (x, 0.0, 1.0), (2.0, 2.0, 2.0))
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

from isaacsim.sensors.experimental.rtx import Lidar as RtxLidar  # noqa: E402

mount = UsdGeom.Xform.Define(stage, "/World/mount")
mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, H))
lidar = RtxLidar.create("/World/mount/lidar", config="OS1", variant="OS1_REV7_128ch20hz1024res")
lidar_rp = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
points = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
points.attach([lidar_rp.path])

cam = UsdGeom.Camera.Define(stage, "/World/mount/camera")
# USD cameras look down -Z with +Y up: turn it to look down +x with +z up.
cam.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(0.5, 0.5, -0.5, -0.5))
cam.CreateFocalLengthAttr(18.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000.0))
cam_rp = rep.create.render_product(str(cam.GetPath()), resolution=(320, 180))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([cam_rp.path])

world.reset()
world.play()


def frames(n):
    for _ in range(n):
        world.step(render=True)


def scan(n=10):
    """Per-scan means over n frames: points, far (> 30 m), phantom (in the empty air short of
    the first box: 1-8 m out, 0.3 m above the pad)."""
    total, far, phantom = [], [], []
    for _ in range(n):
        world.step(render=True)
        d = points.get_data()
        p = np.asarray(d.get("data", []) if isinstance(d, dict) else [], dtype=np.float64)
        if p.ndim != 2 or len(p) == 0:
            continue
        r = np.linalg.norm(p[:, :2], axis=1)
        z = p[:, 2] + H  # the annotator's points are in the sensor frame
        total.append(len(p))
        far.append(int(np.sum(r > 30.0)))
        phantom.append(int(np.sum((r > 1.0) & (r < 8.0) & (z > 0.3))))
    m = lambda v: round(float(np.mean(v)), 1) if v else None  # noqa: E731
    return {"points": m(total), "far": m(far), "phantom": m(phantom), "scans": len(total)}


def image(n=8):
    frames(n)
    d = rgb.get_data()
    a = np.asarray(d, dtype=np.float64)[..., :3] if d is not None and len(d) else None
    if a is None or a.size == 0:
        return {"mean": None}
    lum = a @ np.array([0.2126, 0.7152, 0.0722])
    return {"mean": round(float(lum.mean()), 1), "std": round(float(lum.std()), 1),
            "horizon_band_std": round(float(lum[80:100].std()), 1)}


out = {}
frames(30)  # RTX warm-up

# --- rain ----------------------------------------------------------------------------------
out["rain"] = {}
for rate in (0.0, 2.5, 25.0, 100.0):
    settings.set_float("/app/sensors/nv/atmospherics/rainRate", rate)
    frames(5)
    out["rain"][str(rate)] = scan()
settings.set_float("/app/sensors/nv/atmospherics/rainRate", 0.0)
frames(5)
out["rain"]["0.0 again"] = scan()  # does the rain go when the rate does?

# --- fog -----------------------------------------------------------------------------------
out["fog"] = {"off": {"image": image(), "lidar": scan(5)}}
settings.set_bool("/rtx/fog/enabled", True)
settings.set_bool("/rtx/fog/fogDistanceBased/enabled", True)
settings.set_float("/rtx/fog/fogStartDist", 0.0)
settings.set_float("/rtx/fog/fogEndDist", 60.0)
settings.set_float("/rtx/fog/fogDistanceDensity", 1.0)
settings.set_float("/rtx/fog/fogColorIntensity", 1.0)
out["fog"]["on"] = {"image": image(), "lidar": scan(5)}
settings.set_bool("/rtx/fog/enabled", False)

# --- dynamic sky and time of day -----------------------------------------------------------
sky = {"extension": None}
try:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.kit.environment.core", True)
    sky["extension"] = manager.is_extension_enabled("omni.kit.environment.core")
    from omni.kit.environment.core import get_sunstudy_player

    stage.GetPrimAtPath("/World/sun").SetActive(False)
    omni.kit.commands.execute("CreateDynamicSkyCommand", sky_url=SKY_URL, sky_path="/Environment/sky")
    frames(20)
    player = get_sunstudy_player()
    if player is None:  # headless, the extension's UI side never made one
        from omni.kit.environment.core.sunstudy_player.player import SunstudyPlayer

        player = SunstudyPlayer()
    player._reload_sky()
    sky["sky_type"] = player.sky_type
    light = stage.GetPrimAtPath(
        "/Environment/sky/AxisNorth/AxisLatitude/AxisSHA/AxisDeclination/DistantLight")
    for hour in (6.0, 9.0, 12.0, 18.0, 22.0):
        player.current_time = hour
        frames(10)
        m = UsdGeom.Xformable(light).ComputeLocalToWorldTransform(0.0) if light else None
        # A distant light shines down its local -Z: the sun is up its +Z.
        up = m.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized() if m else None
        sky[str(hour)] = {"sun_elevation_deg": round(math.degrees(math.asin(up[2])), 1)
                          if up is not None else None, "image": image()}
except Exception as e:  # noqa: BLE001 - the probe reports what failed
    sky["error"] = repr(e)
out["sky"] = sky

print("SPIKE-RESULT " + json.dumps(out), flush=True)
# app.close() never returns once the sun-study player has run (stop() and destroy() did not
# help): the result is out, so leave.
os._exit(0)
