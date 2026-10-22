"""The Isaac Sim world behind the world server (roadmap 037). Import after `SimulationApp`.

Every entity is an Xform at the SSv2 entity origin carrying a rigid body, with one child cube
collider placed and scaled to the declared box. A kinematic body's pose is written into the
Xform's translate and orient ops (PhysX follows a kinematic body's USD pose), and read back
from PhysX after the step, so the state csb gets is the physics engine's, not an echo. A
tracked body (roadmap 044) is dynamic -- gravity, mass, the road under it -- and driven by
the velocity csb sends, written through PhysX's tensor API before the substeps, as CARLA's
`set_target_velocity` is before its tick; csb's fallback can make it kinematic for good. The
ego and tracked bodies report contacts; the scene reports kinematic pairs too.
Contacts with the ground (the flat plane, or a world's `ground_paths`) are not reported: a
body resting on its road is not a contact.

With an ego config (`--ego`, roadmaps 038, 045) the ego is not a box but aib's ego
(autoware_isaac_bridge: `aib.isaac.ego.Ego`, built from the config's rig): a PhysX vehicle
with Autoware's sensors and vehicle interface on its own ROS domain. Physics moves it, from
Autoware's commands, and SSv2 reads its pose back; a pose csb sends for it is a teleport.
Rendering is then on every frame its sensors need, whatever the request asks.
"""
import logging
import math
import os
import time

import omni.kit.app
import omni.physx
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

from . import motion as body_motion, vehicle_assets
from .catalogue import check_ego, ego_model, load_ego

log = logging.getLogger("csb_isaac")

# A step slower than this is logged with where the time went.
SLOW_STEP_S = 0.5
ENTITIES = "/World/entities"
CHASE_CAMERA = "/World/chaseCamera"
SUN = "/World/sun"
SKY = "/World/sky"
# Renders after a preload: the first compiles the world's materials, the rest settle it.
PRELOAD_RENDERS = 5
# A handle the server never gives out (they start at 1): the preload's throwaway ego.
WARM_UP_HANDLE = 0
ENVIRONMENT = "/World/env"
GROUND_PLANE = "/World/defaultGroundPlane"
# A tracked body's collider material (roadmap 044): friction against the road. Lower than
# PhysX's default 0.5 so the velocity csb sets once per frame is not eaten by sliding friction
# over the frame's substeps; a body held back by a contact is held by the contact, not this.
TRACKED_MATERIAL = "/World/Looks/csbTrackedBody"
TRACKED_FRICTION = float(os.environ.get("CSB_TRACKED_FRICTION", "0.1"))
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
    def __init__(self, physics_dt=1 / 60, ego_config=None, ego_domain=None, follow=False):
        self.isaac_version = _isaac_version()
        self.follow = follow
        self.ego_config = None
        self.ego_rig = None
        self.ego_ros = None
        if ego_config:
            from isaacsim.core.utils.extensions import enable_extension

            # aib's ego config and the rig it names (the vehicle, generated from aib's spec).
            self.ego_config, self.ego_rig = load_ego(ego_config)
            if ego_domain is not None:
                # The ego stack's domain wins over the config's: both come from one setting
                # (EGO_ROS_DOMAIN_ID) so two checkouts on a host do not share a graph.
                self.ego_config.domain = int(ego_domain)
            for ext in ("isaacsim.sensors.experimental.rtx", "isaacsim.sensors.rtx.nodes"):
                enable_extension(ext)
            omni.kit.app.get_app().update()
            from aib.ros import EgoRos

            self.ego_ros = EgoRos.from_config(self.ego_config, self.ego_rig)
            self.ego_ros.publish_clock(0.0)
            log.info("egos are aib's %s (%s, rig %s)", self.ego_rig.name, ego_config,
                     self.ego_rig.path)
        self.world = World(stage_units_in_meters=1.0, physics_dt=physics_dt,
                           rendering_dt=physics_dt)
        self.dt = physics_dt
        self.stage = omni.usd.get_context().get_stage()
        # CARLA's vehicles exported to USD (CSB_VEHICLE_USD): an entity whose Spawn names one
        # wears it; any other stays the box.
        self.vehicle_assets = vehicle_assets.VehicleAssets()
        if self.ego_rig is not None:
            # The check that the body the ego wears is the car the rig (and Autoware's
            # vehicle_info, generated from the same spec) describes: a mismatch stops the
            # server here, not a drive later (043).
            model = ego_model(self.ego_rig)
            physics = self.vehicle_assets.physics(model)
            if physics is None:
                log.warning("no CARLA physics for the ego's %s in the catalogue (run "
                            "scripts/dump_vehicle_physics.py): its geometry is not checked", model)
            check_ego(self.ego_rig, physics)
            log.info("ego vehicle: %s", self.ego_rig.vehicle)
        self.world.scene.add_default_ground_plane(prim_path=GROUND_PLANE)
        UsdGeom.Xform.Define(self.stage, ENTITIES)
        # Light for cameras (a viewport, 038's watching; the lidar needs none): a loaded world
        # replaces the default ground plane, and with it the only light.
        sun = UsdLux.DistantLight.Define(self.stage, SUN)
        sun.CreateIntensityAttr(2500.0)
        sun.CreateAngleAttr(1.0)
        UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(Gf.Vec3f(-50.0, 20.0, 30.0))
        UsdLux.DomeLight.Define(self.stage, SKY).CreateIntensityAttr(600.0)
        self.loaded_usd = None
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
        self.tracked = {}  # handle -> centre of mass in the body frame (dynamic bodies, 044)
        self.sim_view = None  # omni.physics.tensors simulation view, made on first use
        self.tracked_view = None  # (handles, rigid body view) over the tracked bodies in PhysX
        # Tracked bodies spawned since the last step: not in PhysX until it simulates, so
        # their first drive goes through USD (pose, initial velocity), read at their creation.
        self.fresh = set()
        self.ground_cache = {}  # prim path -> whether it is ground (contacts)
        self.by_path = {}  # rigid body path -> handle
        self.reporters = set()  # handles whose contacts are reported (the ego)
        self.egos = {}  # handle -> Ego (038)
        self.physx = omni.physx.get_physx_interface()
        self.sim = omni.physx.get_physx_simulation_interface()
        if self.ego_config is not None:
            # Vehicle context, collision groups, friction table: before the simulation starts
            # (PhysX ignores the vehicle context's axes once it runs).
            from aib.isaac.vehicle import ensure_vehicle_scene

            ensure_vehicle_scene(self.stage)
        self.world.reset()
        self.world.play()
        self._warm_tracking()

    # --- World -----------------------------------------------------------------------------

    def load(self, usd, ground_paths):
        usd = os.path.realpath(usd) if usd else usd
        for handle in list(self.entities):
            self.despawn(handle)
        self.ground_cache.clear()
        plane = self.stage.GetPrimAtPath(GROUND_PLANE)
        if usd and usd == self.loaded_usd and self.stage.GetPrimAtPath(ENVIRONMENT):
            # The same world (preloaded, or the last episode's): keep it, shaders and all.
            self.ground = list(ground_paths)
            return
        if self.stage.GetPrimAtPath(ENVIRONMENT):
            self.stage.RemovePrim(ENVIRONMENT)
        self.loaded_usd = None
        if usd:
            add_reference_to_stage(usd_path=usd, prim_path=ENVIRONMENT)
            plane.SetActive(False)
            self.ground = list(ground_paths)
            self.loaded_usd = usd
        else:
            plane.SetActive(True)
            self.ground = [GROUND_PLANE, *ground_paths]
        # Our sun and sky only where the world brings no light of its own (a CARLA town
        # exported with SimReady has its sky and sun; doubling them washes the picture out).
        env = self.stage.GetPrimAtPath(ENVIRONMENT)
        own = bool(env) and any(p.HasAPI(UsdLux.LightAPI) for p in Usd.PrimRange(env))
        for path in (SUN, SKY):
            self.stage.GetPrimAtPath(path).SetActive(not own)

    def preload(self, usd, renders=PRELOAD_RENDERS):
        """Load `usd` and render it before csb asks for it: the first render of a large world
        compiles its materials (about 9 minutes for CARLA's Town01), which no request timeout
        should have to cover. csb's `LoadWorld` of the same path then keeps it."""
        t = time.perf_counter()
        self.load(usd, [])
        for _ in range(renders):
            self.world.render()
        if self.ego_config is not None:
            # The lidar's first frames in a large world trace every material once (90 s for
            # CARLA's Town01, past SSv2's own request timeout): a throwaway ego takes them.
            self.spawn(WARM_UP_HANDLE, "preload", "EGO", "", (1.5, 0.0, 0.9), (4.5, 2.1, 1.8),
                       ((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0)))
            self.despawn(WARM_UP_HANDLE)
        log.info("preloaded %s in %.0f s", usd, time.perf_counter() - t)

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="", motion="KINEMATIC"):
        path = f"{ENTITIES}/e{handle}"
        if kind == "EGO" and self.ego_config is not None:
            from aib.isaac.ego import Ego

            # The scenario's declared box is the ego's chassis collider, as for any entity.
            ego = Ego(self.ego_rig, self.ego_ros, self.stage, path, pose, name=name,
                      box_center=center, box_dims=dims)
            # The ego is the rig's car (the Tesla Model 3 of aib's spec), the one its physics,
            # vehicle description and sensor kit are of, as the CARLA backend spawns it,
            # whatever body the scenario names.
            own_model = ego_model(self.ego_rig)
            if model and model != own_model and self.vehicle_assets.by_id.get(model):
                log.warning("ego %r: the scenario names %s, the ego's rig is %s: it wears "
                            "%s", name, model, own_model, own_model)
            asset = self.vehicle_assets.get(own_model)
            if asset is not None:
                # Its wheels go on the PhysX wheels: only a car of the ego's own wheelbase fits.
                wb, own = asset.get("wheel_base") or 0.0, ego.vehicle.p.wheel_base
                if abs(wb - own) <= 0.05:
                    vehicle_assets.add_ego_body(self.stage, path, asset,
                                                collider=self.stage.GetPrimAtPath(f"{path}/body"))
                else:
                    log.warning("ego %r stays a box: %s's wheelbase %.3f m is not the vehicle's "
                                "%.3f m", name, asset["id"], wb, own)
            # The chassis box is the collider, not a body: hidden from rendering whether or
            # not a catalogue body went on, or the roof lidar sees the ego itself (aib 54cace1:
            # 11400 points of it in Autoware's obstacle cloud, a stop for "route-obstacle").
            UsdGeom.Imageable(self.stage.GetPrimAtPath(ego.vehicle.body_path)).MakeInvisible()
            prim = self.stage.GetPrimAtPath(path)
            prim.SetCustomDataByKey("csb:name", name)
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            self.reporters.add(handle)
            self.egos[handle] = ego
            self.entities[handle] = (path, None, None)
            self.by_path[path] = handle
            t = time.perf_counter()
            def warm_step():
                ego.before_substep(self.dt)  # no command yet: the hold brake
                self.world.step(render=True)
                ego.after_substep(self.physx, self.dt)

            frames = ego.warm_up(warm_step)
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
        tracked = motion == "TRACKED" and kind != "EGO"
        body.CreateKinematicEnabledAttr(not tracked)
        cube = UsdGeom.Cube.Define(self.stage, f"{path}/body")
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*COLOURS.get(kind, COLOURS["MISC_OBJECT"]))])
        UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(*center))
        UsdGeom.XformCommonAPI(cube.GetPrim()).SetScale(Gf.Vec3f(*dims))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        asset = None
        if kind in ("VEHICLE", "EGO"):
            # No model named: the car closest to the declared box, as the CARLA backend picks.
            asset = self.vehicle_assets.get(model) if model else self.vehicle_assets.nearest(dims)
        if asset is not None:
            vehicle_assets.add_npc_body(self.stage, path, asset, center, collider=cube.GetPrim())
        mass = None
        if tracked:
            self._skids(path, cube.GetPrim(), center, dims)
            mass = body_motion.body_mass(
                kind, subtype, self.vehicle_assets.physics(asset["id"]) if asset else None)
            m = UsdPhysics.MassAPI.Apply(root.GetPrim())
            m.CreateMassAttr(mass)
            m.CreateCenterOfMassAttr(Gf.Vec3f(*center))
            from pxr import UsdShade

            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
                UsdShade.Material(self._tracked_material()), UsdShade.Tokens.weakerThanDescendants,
                "physics")
            self.tracked[handle] = tuple(center)
            self.fresh.add(handle)
        if kind == "EGO" or tracked:
            PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.0)
            self.reporters.add(handle)
        self.entities[handle] = (path, translate, orient)
        self.by_path[path] = handle
        log.info("spawned %s %r as %s (box %s at %s%s%s)", kind, name, path, dims, center,
                 f", body {asset['id']}" if asset is not None else "",
                 f", tracked, {mass:.0f} kg" if tracked else "")

    def _skids(self, path, cube, center, dims):
        """A tracked body's box raised on two capsules (`motion.skids`), all with the tracked
        material; the capsules are colliders only, never rendered."""
        box_center, box_dims, caps = body_motion.skids(center, dims)
        UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(*box_center))
        UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(*box_dims))
        from pxr import UsdShade

        material = UsdShade.Material(self._tracked_material())
        for i, (c, half, r) in enumerate(caps):
            cap = UsdGeom.Capsule.Define(self.stage, f"{path}/skid{i}")
            cap.CreateAxisAttr(UsdGeom.Tokens.x)
            cap.CreateRadiusAttr(r)
            cap.CreateHeightAttr(2 * half)
            UsdGeom.XformCommonAPI(cap.GetPrim()).SetTranslate(Gf.Vec3d(*c))
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            UsdGeom.Imageable(cap.GetPrim()).MakeInvisible()
            UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(
                material, UsdShade.Tokens.weakerThanDescendants, "physics")

    def _tracked_material(self):
        prim = self.stage.GetPrimAtPath(TRACKED_MATERIAL)
        if prim:
            return prim
        from pxr import UsdShade

        mat = UsdShade.Material.Define(self.stage, TRACKED_MATERIAL)
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(TRACKED_FRICTION)
        api.CreateDynamicFrictionAttr(TRACKED_FRICTION)
        api.CreateRestitutionAttr(0.0)
        # The lower of the two surfaces' friction, whatever the road's.
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr("min")
        return mat.GetPrim()

    # --- Tracked bodies (roadmap 044) ------------------------------------------------------

    def _view(self):
        """`(handles, view)` over every tracked body, rebuilt when the set changed; None while
        PhysX does not have them all yet (a body spawned since the last step)."""
        if not set(self.tracked) - self.fresh:
            return None
        if self.sim_view is None or not self.sim_view.is_valid:
            self._sim_view()
        if self.tracked_view is None:
            self.sim.flush_changes()
            handles = sorted(set(self.tracked) - self.fresh)
            view = self.sim_view.create_rigid_body_view(
                [self.entities[h][0] for h in handles])
            if view is None or getattr(view, "_backend", True) is None \
                    or view.count != len(handles):
                log.warning("PhysX has not got every tracked body yet: no velocities this step")
                return None
            self.tracked_view = (handles, view)
        return self.tracked_view

    def _warm_tracking(self):
        """The tensor API's first rigid body view, on a throwaway tracked body: the first view
        of the server's life zeroed the velocity of the body it was made for, so the first
        tracked NPC of a session started at rest (044 step 2)."""
        self._sim_view()
        self.spawn(WARM_UP_HANDLE, "warm-up", "VEHICLE", "", (0.0, 0.0, 0.5), (1.0, 1.0, 1.0),
                   ((0.0, 0.0, -1000.0), (1.0, 0.0, 0.0, 0.0)), motion="TRACKED")
        self.world.step(render=False)
        self.fresh.clear()
        self._view()
        self.despawn(WARM_UP_HANDLE)

    def _sim_view(self):
        import omni.physics.tensors as tensors

        self.sim_view = tensors.create_simulation_view(
            "numpy", stage_id=omni.usd.get_context().get_stage_id())
        self.sim_view.set_subspace_roots("/")
        self.tracked_view = None

    def _drive_tracked(self, drives):
        """Teleports and velocities of the tracked bodies, before the substeps."""
        for h in self.fresh:
            d = drives.get(h)
            if d is not None:
                self._drive_fresh(h, d)
        view = self._view()
        if view is None:
            return
        import numpy as np

        handles, rb = view
        tf = rb.get_transforms()  # x, y, z, qx, qy, qz, qw
        vel = rb.get_velocities()  # vx, vy, vz (centre of mass), wx, wy, wz
        moved, sped = [], []
        for i, h in enumerate(handles):
            d = drives.get(h)
            if d is None:
                continue
            q = (float(tf[i, 6]), float(tf[i, 3]), float(tf[i, 4]), float(tf[i, 5]))
            if d.teleport and d.pose is not None:
                (x, y, z), q = d.pose[0], body_motion.normalized(d.pose[1])
                tf[i] = (x, y, z, q[1], q[2], q[3], q[0])
                moved.append(i)
            if d.twist is not None:
                v, w = d.twist
                vel[i, 0:3] = body_motion.com_velocity(v, w, q, self.tracked[h])
                vel[i, 3:6] = w
                sped.append(i)
        if moved:
            rb.set_transforms(tf, np.array(moved, dtype=np.int32))
        if sped:
            rb.set_velocities(vel, np.array(sped, dtype=np.int32))

    def _drive_fresh(self, handle, d):
        """A drive for a body PhysX has not created yet: its USD pose and initial velocity."""
        path, translate, orient = self.entities[handle]
        prim = self.stage.GetPrimAtPath(path)
        if d.teleport and d.pose is not None:
            translate.Set(Gf.Vec3d(*d.pose[0]))
            orient.Set(_quat(d.pose[1]))
        if d.twist is not None:
            q = orient.Get()
            q = (q.GetReal(), *q.GetImaginary())
            v, w = d.twist
            body = UsdPhysics.RigidBodyAPI(prim)
            body.GetVelocityAttr().Set(Gf.Vec3f(*body_motion.com_velocity(v, w, q,
                                                                          self.tracked[handle])))
            # UsdPhysics' angular velocity is in degrees per second.
            body.GetAngularVelocityAttr().Set(Gf.Vec3f(*(math.degrees(c) for c in w)))

    def _to_kinematic(self, handle):
        """csb's fallback: the tracked body is kinematic from now on."""
        path, _, _ = self.entities[handle]
        UsdPhysics.RigidBodyAPI(self.stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Set(True)
        del self.tracked[handle]
        self.fresh.discard(handle)
        self.tracked_view = None
        log.info("%s: tracked body handed to kinematic (csb's fallback)", path)

    def despawn(self, handle):
        path, _, _ = self.entities.pop(handle)
        self.by_path.pop(path, None)
        ego = self.egos.pop(handle, None)
        if ego is not None:
            ego.close()
        self.reporters.discard(handle)
        if self.tracked.pop(handle, None) is not None:
            self.tracked_view = None
        self.fresh.discard(handle)
        self.stage.RemovePrim(path)

    def step(self, drives, substeps, dt, render, sim_time=0.0):
        """`drives`: `{handle: server.Drive}`. `sim_time`: the server's clock at the start of
        the step (the egos' stamps)."""
        if abs(dt - self.dt) > 1e-12:
            self.world.set_simulation_dt(physics_dt=dt, rendering_dt=dt)
            self.dt = dt
        for handle, d in drives.items():
            if d.kinematic and handle in self.tracked:
                self._to_kinematic(handle)
        with Sdf.ChangeBlock():
            for handle, d in drives.items():
                if handle in self.egos or handle in self.tracked or d.pose is None:
                    continue
                (position, quat) = d.pose
                _, translate, orient = self.entities[handle]
                translate.Set(Gf.Vec3d(*position))
                orient.Set(_quat(quat))
        self._drive_tracked(drives)
        for handle, d in drives.items():
            if handle in self.egos and d.pose is not None:
                self.egos[handle].teleport(d.pose, self.physx)
        egos = list(self.egos.values())
        t0 = time.perf_counter()
        for ego in egos:
            ego.take_commands()
        t1 = time.perf_counter()
        render = render or any(e.renders for e in egos)
        touching = {}  # (handle, other, other path) -> impulse
        for s in range(substeps):
            for ego in egos:
                ego.before_substep(dt)
            self.world.step(render=render and s == substeps - 1)
            for ego in egos:
                ego.after_substep(self.physx, dt)
            self._collect_contacts(touching)
        if self.fresh:  # simulated once: PhysX has them now
            self.fresh.clear()
            self.tracked_view = None
        states = []
        tracked = self._tracked_states()
        for handle, (path, _, _) in self.entities.items():
            ego = self.egos.get(handle)
            if ego is not None:
                # aib's velocities are base_link's, from its last substep: into the world.
                q = ego.state.quat
                twist = (body_motion.rotate(q, ego.state.velocity),
                         body_motion.rotate(q, ego.state.angular_velocity))
                states.append((handle, (ego.state.position, q), twist))
                continue
            if handle in tracked:
                states.append((handle, *tracked[handle]))
                continue
            t = self.physx.get_rigidbody_transformation(path)
            p, r = t["position"], t["rotation"]  # rotation: x, y, z, w
            states.append((handle, ((p[0], p[1], p[2]), (r[3], r[0], r[1], r[2])), None))
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

    def _tracked_states(self):
        """`{handle: (pose, twist at the origin)}` of the tracked bodies, from PhysX."""
        view = self._view()
        if view is None:
            return {}
        handles, rb = view
        tf = rb.get_transforms()
        vel = rb.get_velocities()
        out = {}
        for i, h in enumerate(handles):
            q = (float(tf[i, 6]), float(tf[i, 3]), float(tf[i, 4]), float(tf[i, 5]))
            w = (float(vel[i, 3]), float(vel[i, 4]), float(vel[i, 5]))
            v = body_motion.origin_velocity((float(vel[i, 0]), float(vel[i, 1]), float(vel[i, 2])),
                                       w, q, self.tracked[h])
            out[h] = (((float(tf[i, 0]), float(tf[i, 1]), float(tf[i, 2])), q), (v, w))
        return out

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

    def _is_ground(self, path):
        g = self.ground_cache.get(path)
        if g is None:
            g = any(path == p or path.startswith(p + "/") for p in self.ground)
            self.ground_cache[path] = g
        return g

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
                if self._is_ground(theirs):
                    continue
                other = self.by_path.get(theirs, 0)
                impulse = 0.0
                for i in range(header.contact_data_offset,
                               header.contact_data_offset + header.num_contact_data):
                    v = data[i].impulse
                    impulse += math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
                key = (handle, other, theirs)
                touching[key] = touching.get(key, 0.0) + impulse
                # Each reporting side has its contact: two reporters touching are two (044).
