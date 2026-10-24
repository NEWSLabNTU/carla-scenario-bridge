"""Where does a scenario's world load leave memory behind (docs/issues/003)?

    ISAAC_ROOT=... run_spike.sh fastrtps <log> $PWD/src/isaac_world/spikes/reload_memory.py \\
        [<Town01 world.yaml> [<surfaces.yaml>]]
    # prints one SPIKE-RESULT json

The world server's own `IsaacWorld`, Town01 preloaded once, then what a batch of scenarios
does to it, phase by phase, RELOAD_N loads each (default 6), the resident memory after every
load (the first of a phase is its warm-up):

  bare          LoadWorld of the same world with no surface map, nothing spawned
  bare_ego      the same, then the ego spawned, RELOAD_FRAMES frames rendered, despawned
  soil          LoadWorld with the surface map (town01_soil's two zones: the town cut out under
                them, tiles, plates), nothing spawned
  soil_ego      the same with the ego, as a scenario does
  bare_nosensor bare_ego with every sensor of the rig off: the PhysX vehicle alone
  lidar_raw     no ego: an RTX lidar created, its render product and point cloud annotator
                attached, RELOAD_FRAMES frames rendered, torn down as aib's `Lidar.close` did
                before issue 003's fix, and its prim removed
  lidar_keep_rp lidar_raw with one render product and annotator for every cycle: each new
                lidar prim made its camera, a parked one between cycles
  lidar_static  one lidar, render product and annotator made in the first cycle and kept:
                each cycle only renders RELOAD_FRAMES frames (is it the frames or the prims?)
  lidar_same    lidar_keep_rp with every cycle's lidar at the same path
  lidar_pause   lidar_same, the product paused between cycles (its updates off) and left on
                the removed prim instead of a parked one
  lidar_parkpause  lidar_same, paused while parked
  aib_lidar     aib's own Lidar (the rig's) made and closed for a fixed vehicle path, no vehicle

RELOAD_PARKED_FRAMES: frames rendered after each despawn (no ego: the bridge's steps between
a scenario's end and the next load). The ego publishes on ROS domain RELOAD_DOMAIN (default
77), which no stack listens on.
"""
import gc
import json
import os
import sys
import time

os.environ.pop("DISPLAY", None)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "extra_args": ["--/crashreporter/url="]})
import logging  # noqa: E402

import yaml  # noqa: E402

from isaac_world.surface_layer import register  # noqa: E402
from isaac_world.world import IsaacWorld  # noqa: E402

logging.basicConfig(level=logging.INFO, stream=sys.stdout,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../../.."))
WORLD = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    REPO, "data/carla-autoware-bridge/Town01/isaac/world.yaml")
SURFACES = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    REPO, "scenarios/surfaces/town01_soil.yaml")
N = int(os.environ.get("RELOAD_N", "6"))
FRAMES = int(os.environ.get("RELOAD_FRAMES", "100"))
PHASES = os.environ.get("RELOAD_PHASES", "bare,bare_ego,soil,soil_ego").split(",")
EGO_YAML = os.path.join(REPO, "src/autoware_isaac_bridge/src/aib/config/ego.yaml")
POSE = ((200.0, -130.0, 0.3), (0.0, 0.0, 0.0, 1.0))  # town01_soil's road, heading -x


def rss_mb():
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return float("nan")


with open(WORLD) as f:
    w = yaml.safe_load(f)
usd, ground = os.path.realpath(w["usd"]), list(w.get("ground_paths") or [])
soil_maps = {usd: (SURFACES, tuple(w.get("map_origin") or (0.0, 0.0, 0.0)))}
assert register([WORLD])  # the world file reads as the server reads it
world = IsaacWorld(ego_config=EGO_YAML, ego_domain=int(os.environ.get("RELOAD_DOMAIN", "77")),
                   surface_maps={})
t = time.monotonic()
world.preload(usd)
out = {"preload_s": round(time.monotonic() - t, 1), "preload_rss_mb": round(rss_mb())}
print("preloaded", out, flush=True)

print("sensors", {k: (v.type, v.enabled) for k, v in world.ego_rig.sensors.items()},
      flush=True)
enabled = {k: v.enabled for k, v in world.ego_rig.sensors.items()}


def lidar_cycle(k):
    import omni.replicator.core as rep
    from isaacsim.sensors.experimental.rtx import Lidar as RtxLidar

    path = f"/World/lidar_raw_{k}"
    lidar = RtxLidar.create(path, config="OS1", variant="OS1_REV7_128ch20hz1024res")
    rp = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
    ann = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloud")
    ann.attach([rp.path])
    for _ in range(FRAMES):
        world.step({}, 1, 1 / 60, True)
        ann.get_data()
    ann.detach([rp.path])
    rp.destroy()
    world.stage.RemovePrim(path)


KEPT = {}


def lidar_pause_cycle(k):
    import omni.replicator.core as rep
    from isaacsim.sensors.experimental.rtx import Lidar as RtxLidar

    path = "/World/lidar_pause"
    lidar = RtxLidar.create(path, config="OS1", variant="OS1_REV7_128ch20hz1024res")
    if "pause" not in KEPT:
        KEPT["pause"] = rp = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
        KEPT["pause_ann"] = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloud")
        KEPT["pause_ann"].attach([rp.path])
    else:
        rp = KEPT["pause"]
        world.stage.GetPrimAtPath(rp.path).GetRelationship("camera").SetTargets([lidar.paths[0]])
        rp.hydra_texture.set_updates_enabled(True)
    for _ in range(FRAMES):
        world.step({}, 1, 1 / 60, True)
        KEPT["pause_ann"].get_data()
    rp.hydra_texture.set_updates_enabled(False)
    world.stage.RemovePrim(path)


def aib_lidar_cycle(k):
    from aib.isaac import sensors
    from pxr import UsdGeom

    rig = next(r for r in world.ego_rig.sensors.values() if r.type == "lidar")
    UsdGeom.Xform.Define(world.stage, "/World/aibveh")
    lidar = sensors.Lidar(rig, world.ego_ros, world.stage, "/World/aibveh")
    for _ in range(FRAMES):
        world.step({}, 1, 1 / 60, True)
        lidar.ready()
    lidar.close()
    world.stage.RemovePrim("/World/aibveh")
    print("aib kept", list(sensors.Lidar.kept), flush=True)


def lidar_keep_cycle(k, mode="new"):
    import omni.replicator.core as rep
    from isaacsim.sensors.experimental.rtx import Lidar as RtxLidar

    def camera(path):
        world.stage.GetPrimAtPath(KEPT["rp"].path).GetRelationship("camera").SetTargets([path])

    if "rp" not in KEPT:
        park = RtxLidar.create("/World/lidar_park", config="OS1",
                               variant="OS1_REV7_128ch20hz1024res")
        KEPT["park"] = park.paths[0]
        KEPT["rp"] = rep.create.render_product(park.paths[0], resolution=(128, 128))
        KEPT["ann"] = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloud")
        KEPT["ann"].attach([KEPT["rp"].path])
    if mode == "static":
        if "static" not in KEPT:
            KEPT["static"] = RtxLidar.create("/World/lidar_static", config="OS1",
                                             variant="OS1_REV7_128ch20hz1024res").paths[0]
        camera(KEPT["static"])
        path = None
    else:
        path = f"/World/lidar_keep_{k}" if mode == "new" else "/World/lidar_same"
        lidar = RtxLidar.create(path, config="OS1", variant="OS1_REV7_128ch20hz1024res")
        camera(lidar.paths[0])
    if mode == "parkpause":
        KEPT["rp"].hydra_texture.set_updates_enabled(True)
    got = 0
    for _ in range(FRAMES):
        world.step({}, 1, 1 / 60, True)
        d = KEPT["ann"].get_data()
        got = max(got, len(d.get("data", [])) if isinstance(d, dict) else 0)
    rp = KEPT["rp"]
    camera(KEPT["park"])
    world.step({}, 1, 1 / 60, True)
    if mode == "parkpause":
        rp.hydra_texture.set_updates_enabled(False)
    if path:
        world.stage.RemovePrim(path)
    print("lidar_keep points", got, flush=True)


handle = 100
for phase in PHASES:
    for k, v in world.ego_rig.sensors.items():
        v.enabled = enabled[k] and phase != "bare_nosensor"
    world.surface_maps = soil_maps if phase.startswith("soil") else {}
    rows = []
    for k in range(N):
        t = time.monotonic()
        world.load(usd, ground)
        if phase == "lidar_raw":
            lidar_cycle(k)
        if phase == "lidar_keep_rp":
            lidar_keep_cycle(k)
        if phase == "lidar_pause":
            lidar_pause_cycle(k)
        if phase == "aib_lidar":
            aib_lidar_cycle(k)
        if phase in ("lidar_static", "lidar_same", "lidar_parkpause"):
            lidar_keep_cycle(k, phase.split("_")[1])
        if phase.endswith("_ego") or phase == "bare_nosensor":
            handle += 1
            world.spawn(handle, "ego", "EGO", "", (1.5, 0.0, 0.9), (4.8, 2.1, 1.5), POSE)
            for _ in range(FRAMES):
                world.step({}, 3, 1 / 60, True)
            world.despawn(handle)
            for _ in range(int(os.environ.get("RELOAD_PARKED_FRAMES", "0"))):
                world.step({}, 1, 1 / 60, True)  # the bridge steps on after the despawn
            from aib.isaac import sensors

            print("ego kept", list(sensors.Lidar.kept), flush=True)
        for _ in range(3):
            world.step({}, 1, 1 / 60, False)
        gc.collect()
        rows.append((round(rss_mb(), 1), round(time.monotonic() - t, 1)))
        print(phase, k, rows[-1], flush=True)
    rss = [r[0] for r in rows]
    out[phase] = {"rss_mb": rss, "per_load_mb": round((rss[-1] - rss[0]) / max(1, N - 1), 1),
                  "load_s": [r[1] for r in rows]}

print("SPIKE-RESULT " + json.dumps(out), flush=True)
app.close()
