"""The scenario sky on the world server's own code (roadmap 047): `IsaacWorld.set_environment`
on a small world that brings its own sun and dome, as a SimReady town does, seen by a camera.

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/sky_world.py
    # prints one SPIKE-RESULT json

Cases, each a camera's mean brightness (0-255) and which lights are on: the world's own lights;
noon, 07:00 and 21:30 on 21 June (equator: the map has no projector info); eight oktas of
cloud at noon; fog to 60 m; and after a reset, the world's own lights again.
"""
import json
import os
import sys
import tempfile

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

from isaac_world.world import ENVIRONMENT, SUN, IsaacWorld  # noqa: E402

ENV_USDA = """#usda 1.0
(
    upAxis = "Z"
    metersPerUnit = 1
    defaultPrim = "World"
)
def Xform "World"
{
    def Cube "ground"
    {
        double size = 1
        double3 xformOp:scale = (200, 200, 0.1)
        double3 xformOp:translate = (0, 0, -0.05)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
    def Cube "box"
    {
        double size = 2
        double3 xformOp:translate = (15, 0, 1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def DistantLight "town_sun"
    {
        float inputs:intensity = 3000
        float inputs:angle = 0.5
        float3 xformOp:rotateXYZ = (-40, 0, 20)
        uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
    }
    def DomeLight "town_sky"
    {
        float inputs:intensity = 800
    }
}
"""

d = tempfile.mkdtemp()
usd = os.path.join(d, "env.usda")
with open(usd, "w") as f:
    f.write(ENV_USDA)

world = IsaacWorld()
world.load(usd, [])
stage = world.stage
cam = UsdGeom.Camera.Define(stage, "/World/probe_cam")
xf = UsdGeom.Xformable(cam)
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 2.0))
xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(0.5, 0.5, -0.5, -0.5))
rp = rep.create.render_product(str(cam.GetPath()), resolution=(160, 90))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp.path])
t = [0.0]


def frames(n=12):
    for _ in range(n):
        world.step({}, 1, 1 / 60, True, t[0])
        t[0] += 1 / 60


def look():
    frames()
    a = np.asarray(rgb.get_data(), dtype=np.float64)[..., :3]
    town = stage.GetPrimAtPath(f"{ENVIRONMENT}/town_sun")
    sun = UsdLux.DistantLight(stage.GetPrimAtPath(SUN))
    return {"mean": round(float(a.mean()), 1),
            "town_sun_on": bool(town and town.IsActive()),
            "our_sun_on": bool(sun.GetPrim().IsActive()),
            "our_sun_intensity": round(float(sun.GetIntensityAttr().Get()), 1),
            "town_dome": round(float(UsdLux.DomeLight(
                stage.GetPrimAtPath(f"{ENVIRONMENT}/town_sky")).GetIntensityAttr().Get()), 1)}


def day(hours):
    return {"hours": hours, "day_of_year": 172, "animation": False,
            "date_time": f"2026-06-21T{hours:05.2f}"}


out = {}
frames(30)
out["own"] = look()
world.set_environment(None, day(12.0), t[0])
out["noon"] = look()
world.set_environment(None, day(7.0), t[0])
out["07:00"] = look()
world.set_environment(None, day(21.5), t[0])
out["21:30"] = look()
world.set_environment({"cloud_cover_oktas": 8}, day(12.0), t[0])
out["noon_8_oktas"] = look()
world.set_environment({"cloud_cover_oktas": 0, "fog_visual_range": 60.0}, None, t[0])
out["noon_fog_60m"] = look()
try:
    world.set_environment({"precipitation": ("snow", 2.0)}, None, t[0])
    out["snow"] = "taken"
except ValueError as e:
    out["snow"] = f"refused: {e}"
world.reset([])
out["after_reset"] = look()
print("SPIKE-RESULT " + json.dumps(out), flush=True)
os._exit(0)
