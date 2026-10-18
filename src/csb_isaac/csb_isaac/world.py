"""The Isaac Sim world behind the world server (roadmap 037). Import after `SimulationApp`.

Every entity is an Xform at the SSv2 entity origin carrying a kinematic rigid body, with one
child cube collider placed and scaled to the declared box. A pose is written into the Xform's
translate and orient ops (PhysX follows a kinematic body's USD pose), and read back from
PhysX after the step, so the state csb gets is the physics engine's, not an echo. The ego
reports contacts; the scene reports kinematic pairs (NPCs against NPCs and the ground).
Contacts with the ground (the flat plane, or a world's `ground_paths`) are not reported: a
body resting on its road is not a contact.

With an ego YAML (`--ego`, roadmap 038) the ego is not a box but a PhysX vehicle with
Autoware's sensors and vehicle interface on its own ROS domain (`ego.py`): physics moves it,
from Autoware's commands, and SSv2 reads its pose back; a pose csb sends for it is a
teleport. Rendering is then on every frame, whatever the request asks: the lidar needs it.
"""
import logging
import math
import time

import omni.kit.app
import omni.physx
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

log = logging.getLogger("csb_isaac")

# A step slower than this is logged with where the time went.
SLOW_STEP_S = 0.5
ENTITIES = "/World/entities"
CHASE_CAMERA = "/World/chaseCamera"
ENVIRONMENT = "/World/env"
GROUND_PLANE = "/World/defaultGroundPlane"
# Kind -> display colour (sRGB): what an operator watching on VNC tells apart.
COLOURS = {
    "EGO": (0.10, 0.45, 0.85),
    "VEHICLE": (0.80, 0.30, 0.15),
    "PEDESTRIAN": (0.95, 0.80, 0.10),
    "MISC_OBJECT": (0.55, 0.55, 0.55),
}


def _isaac_version():
    """`6.1.0 (Kit 110.3.0)`: the pip package's version and Kit's."""
    from importlib.metadata import PackageNotFoundError, version

    kit = omni.kit.app.get_app().get_build_version().split("+")[0]
    try:
        return f"{version('isaacsim')} (Kit {kit})"
    except PackageNotFoundError:
        return f"Kit {kit}"


def _quat(wxyz):
    w, x, y, z = wxyz
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return Gf.Quatd(w / n, x / n, y / n, z / n)


class IsaacWorld:
    def __init__(self, physics_dt=1 / 60, ego_config=None, follow=False):
        self.isaac_version = _isaac_version()
        self.follow = follow
        self.ego_config = None
        self.ego_ros = None
        if ego_config:
            from isaacsim.core.utils.extensions import enable_extension

            from .ego import EgoConfig

            self.ego_config = EgoConfig.load(ego_config)
            for ext in ("isaacsim.sensors.experimental.rtx", "isaacsim.sensors.rtx.nodes"):
                enable_extension(ext)
            omni.kit.app.get_app().update()
            from .ego import EgoRos

            self.ego_ros = EgoRos(self.ego_config)
            self.ego_ros.publish_clock(0.0)
            log.info("egos are PhysX vehicles with Autoware's interface (%s)", ego_config)
        self.world = World(stage_units_in_meters=1.0, physics_dt=physics_dt,
                           rendering_dt=physics_dt)
        self.dt = physics_dt
        self.stage = omni.usd.get_context().get_stage()
        self.world.scene.add_default_ground_plane(prim_path=GROUND_PLANE)
        UsdGeom.Xform.Define(self.stage, ENTITIES)
        # Light for cameras (a viewport, 038's watching; the lidar needs none): a loaded world
        # replaces the default ground plane, and with it the only light.
        sun = UsdLux.DistantLight.Define(self.stage, "/World/sun")
        sun.CreateIntensityAttr(2500.0)
        sun.CreateAngleAttr(1.0)
        UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(Gf.Vec3f(-50.0, 20.0, 30.0))
        UsdLux.DomeLight.Define(self.stage, "/World/sky").CreateIntensityAttr(600.0)
        for prim in self.stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                PhysxSchema.PhysxSceneAPI.Apply(prim)
                for name in ("reportKinematicKinematicPairs", "reportKinematicStaticPairs"):
                    attr = prim.GetAttribute(f"physxScene:{name}")
                    if not attr:
                        attr = prim.CreateAttribute(f"physxScene:{name}", Sdf.ValueTypeNames.Bool)
                    attr.Set(True)
        self.ground = [GROUND_PLANE]
        self.entities = {}  # handle -> (path, translate op, orient op)
        self.by_path = {}  # rigid body path -> handle
        self.reporters = set()  # handles whose contacts are reported (the ego)
        self.egos = {}  # handle -> Ego (038)
        self.physx = omni.physx.get_physx_interface()
        self.sim = omni.physx.get_physx_simulation_interface()
        if self.ego_config is not None:
            # Vehicle context, collision groups, friction table: before the simulation starts
            # (PhysX ignores the vehicle context's axes once it runs).
            from .vehicle import ensure_vehicle_scene

            ensure_vehicle_scene(self.stage)
        self.world.reset()
        self.world.play()

    # --- World -----------------------------------------------------------------------------

    def load(self, usd, ground_paths):
        for handle in list(self.entities):
            self.despawn(handle)
        if self.stage.GetPrimAtPath(ENVIRONMENT):
            self.stage.RemovePrim(ENVIRONMENT)
        plane = self.stage.GetPrimAtPath(GROUND_PLANE)
        if usd:
            add_reference_to_stage(usd_path=usd, prim_path=ENVIRONMENT)
            plane.SetActive(False)
            self.ground = list(ground_paths)
        else:
            plane.SetActive(True)
            self.ground = [GROUND_PLANE, *ground_paths]

    def spawn(self, handle, name, kind, subtype, center, dims, pose):
        path = f"{ENTITIES}/e{handle}"
        if kind == "EGO" and self.ego_config is not None:
            from .ego import Ego

            ego = Ego(self.ego_config, self.ego_ros, self.stage, path, name, center, dims, pose)
            prim = self.stage.GetPrimAtPath(path)
            prim.SetCustomDataByKey("csb:name", name)
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            self.reporters.add(handle)
            self.egos[handle] = ego
            self.entities[handle] = (path, None, None)
            self.by_path[path] = handle
            t = time.perf_counter()
            frames = ego.warm_up(self.world.render)
            log.info("spawned ego %r as PhysX vehicle %s (box %s at %s); sensors warm after %d "
                     "render(s), %.1f s", name, path, dims, center, frames, time.perf_counter() - t)
            return
        root = UsdGeom.Xform.Define(self.stage, path)
        root.GetPrim().SetCustomDataByKey("csb:name", name)
        translate = root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        orient = root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        translate.Set(Gf.Vec3d(*pose[0]))
        orient.Set(_quat(pose[1]))
        body = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        body.CreateKinematicEnabledAttr(True)
        cube = UsdGeom.Cube.Define(self.stage, f"{path}/body")
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*COLOURS.get(kind, COLOURS["MISC_OBJECT"]))])
        UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(*center))
        UsdGeom.XformCommonAPI(cube.GetPrim()).SetScale(Gf.Vec3f(*dims))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if kind == "EGO":
            PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.0)
            self.reporters.add(handle)
        self.entities[handle] = (path, translate, orient)
        self.by_path[path] = handle
        log.info("spawned %s %r as %s (box %s at %s)", kind, name, path, dims, center)

    def despawn(self, handle):
        path, _, _ = self.entities.pop(handle)
        self.by_path.pop(path, None)
        ego = self.egos.pop(handle, None)
        if ego is not None:
            ego.close()
        self.reporters.discard(handle)
        self.stage.RemovePrim(path)

    def step(self, poses, substeps, dt, render, sim_time=0.0):
        """`sim_time`: the server's clock at the start of the step (the egos' stamps)."""
        if abs(dt - self.dt) > 1e-12:
            self.world.set_simulation_dt(physics_dt=dt, rendering_dt=dt)
            self.dt = dt
        with Sdf.ChangeBlock():
            for handle, (position, quat) in poses.items():
                if handle in self.egos:
                    continue
                _, translate, orient = self.entities[handle]
                translate.Set(Gf.Vec3d(*position))
                orient.Set(_quat(quat))
        for handle, pose in poses.items():
            if handle in self.egos:
                self.egos[handle].vehicle.teleport(pose, self.physx)
        egos = list(self.egos.values())
        t0 = time.perf_counter()
        for ego in egos:
            ego.ros.take_commands()
        t1 = time.perf_counter()
        render = render or any(e.lidar is not None for e in egos)
        touching = {}  # (handle, other, other path) -> impulse
        for s in range(substeps):
            for ego in egos:
                ego.before_substep(dt)
            self.world.step(render=render and s == substeps - 1)
            for ego in egos:
                ego.after_substep(self.physx, dt)
            self._collect_contacts(touching)
        states = []
        for handle, (path, _, _) in self.entities.items():
            ego = self.egos.get(handle)
            if ego is not None:
                states.append((handle, (ego.state.position, ego.state.quat)))
                continue
            t = self.physx.get_rigidbody_transformation(path)
            p, r = t["position"], t["rotation"]  # rotation: x, y, z, w
            states.append((handle, ((p[0], p[1], p[2]), (r[3], r[0], r[1], r[2]))))
        t2 = time.perf_counter()
        if self.follow and egos:
            self._follow(egos[0])
        end = sim_time + substeps * dt
        if self.ego_ros is not None:
            stamp = self.ego_ros.publish_clock(end)
            for ego in egos:
                ego.publish(end, stamp)
        t3 = time.perf_counter()
        if t3 - t0 > SLOW_STEP_S:
            log.warning("slow step at sim time %.2f: %.0f ms (commands %.0f, physics and render "
                        "%.0f, publishing %.0f)", end, 1e3 * (t3 - t0), 1e3 * (t1 - t0),
                        1e3 * (t2 - t1), 1e3 * (t3 - t2))
        contacts = [(h, o, path, impulse) for (h, o, path), impulse in touching.items()]
        return states, contacts

    def _follow(self, ego):
        """Put the viewport's camera behind and above the ego, looking at it."""
        if not hasattr(self, "_chase"):
            from omni.kit.viewport.utility import get_active_viewport

            cam = UsdGeom.Camera.Define(self.stage, CHASE_CAMERA)
            cam.CreateFocalLengthAttr(18.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.5, 2000.0))
            self._chase = (UsdGeom.XformCommonAPI(cam.GetPrim()), None)
            get_active_viewport().camera_path = CHASE_CAMERA
        api, smooth = self._chase
        (x, y, z), (w, qx, qy, qz) = ego.state.position, ego.state.quat
        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        if smooth is not None:  # a little lag, so the picture does not shake with the body
            d = math.atan2(math.sin(yaw - smooth), math.cos(yaw - smooth))
            yaw = smooth + 0.15 * d
        self._chase = (api, yaw)
        back, up = 14.0, 6.5
        eye = (x - back * math.cos(yaw), y - back * math.sin(yaw), z + up)
        api.SetTranslate(Gf.Vec3d(*eye))
        # A USD camera looks down -z with +y up: rotate it to look along yaw, pitched down.
        pitch = math.degrees(math.atan2(up - 1.0, back))
        api.SetRotate(Gf.Vec3f(90.0 - pitch, 0.0, math.degrees(yaw) - 90.0),
                      UsdGeom.XformCommonAPI.RotationOrderXYZ)

    def close(self):
        for handle in list(self.egos):
            self.despawn(handle)
        if self.ego_ros is not None:
            self.ego_ros.close()
        self.world.stop()

    # --- Contacts --------------------------------------------------------------------------

    def _collect_contacts(self, touching):
        headers, data = self.sim.get_contact_report()
        for header in headers:
            if str(header.type).endswith("CONTACT_LOST"):
                continue
            a0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            for mine, theirs in ((a0, a1), (a1, a0)):
                handle = self.by_path.get(mine)
                if handle not in self.reporters:
                    continue
                if any(theirs == g or theirs.startswith(g + "/") for g in self.ground):
                    continue
                other = self.by_path.get(theirs, 0)
                impulse = 0.0
                for i in range(header.contact_data_offset,
                               header.contact_data_offset + header.num_contact_data):
                    v = data[i].impulse
                    impulse += math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
                key = (handle, other, theirs)
                touching[key] = touching.get(key, 0.0) + impulse
                # A pair is one contact, the reporting side's.
                break
