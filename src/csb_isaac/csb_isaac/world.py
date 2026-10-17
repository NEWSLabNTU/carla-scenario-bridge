"""The Isaac Sim world behind the world server (roadmap 037). Import after `SimulationApp`.

Every entity is an Xform at the SSv2 entity origin carrying a kinematic rigid body, with one
child cube collider placed and scaled to the declared box. A pose is written into the Xform's
translate and orient ops (PhysX follows a kinematic body's USD pose), and read back from
PhysX after the step, so the state csb gets is the physics engine's, not an echo. The ego
reports contacts; the scene reports kinematic pairs, since every body is kinematic until the
PhysX vehicle (038). Contacts with the ground (the flat plane, or a world's `ground_paths`)
are not reported: a body resting on its road is not a contact.
"""
import logging
import math

import omni.kit.app
import omni.physx
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics

log = logging.getLogger("csb_isaac")

ENTITIES = "/World/entities"
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
    def __init__(self, physics_dt=1 / 60):
        self.isaac_version = _isaac_version()
        self.world = World(stage_units_in_meters=1.0, physics_dt=physics_dt,
                           rendering_dt=physics_dt)
        self.dt = physics_dt
        self.stage = omni.usd.get_context().get_stage()
        self.world.scene.add_default_ground_plane(prim_path=GROUND_PLANE)
        UsdGeom.Xform.Define(self.stage, ENTITIES)
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
        self.physx = omni.physx.get_physx_interface()
        self.sim = omni.physx.get_physx_simulation_interface()
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
        self.reporters.discard(handle)
        self.stage.RemovePrim(path)

    def step(self, poses, substeps, dt, render):
        if abs(dt - self.dt) > 1e-12:
            self.world.set_simulation_dt(physics_dt=dt, rendering_dt=dt)
            self.dt = dt
        with Sdf.ChangeBlock():
            for handle, (position, quat) in poses.items():
                _, translate, orient = self.entities[handle]
                translate.Set(Gf.Vec3d(*position))
                orient.Set(_quat(quat))
        touching = {}  # (handle, other, other path) -> impulse
        for s in range(substeps):
            self.world.step(render=render and s == substeps - 1)
            self._collect_contacts(touching)
        states = []
        for handle, (path, _, _) in self.entities.items():
            t = self.physx.get_rigidbody_transformation(path)
            p, r = t["position"], t["rotation"]  # rotation: x, y, z, w
            states.append((handle, ((p[0], p[1], p[2]), (r[3], r[0], r[1], r[2]))))
        contacts = [(h, o, path, impulse) for (h, o, path), impulse in touching.items()]
        return states, contacts

    def close(self):
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
