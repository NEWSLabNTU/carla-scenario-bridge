"""The ego's PhysX vehicle and its actuation (roadmap 038). Import after `SimulationApp`.

The vehicle's rigid body frame is Autoware's `base_link`: the rear axle centre on the ground,
x forward, z up, so the pose csb reads back is the SSv2 ego origin and the pose the vehicle
status reports is the one Autoware expects. Geometry comes from the Autoware vehicle
description (`vehicle_info.param.yaml`: wheel base, tread, wheel radius and width), so the
model Autoware plans with and the one Isaac drives are the same one; the physics (mass, centre
of mass, steering limit, brake and drive torque, tires) is CARLA's blueprint's, from the
vehicle catalogue (roadmap 043, `vehicle_params.py`); suspension is ours (CARLA does not expose
it), tuned per axle to the same frequency so the car sits level on its sprung masses.

PhysX drives it with no engine model ("drive none"): each wheel's drive torque, brake torque
and steer angle are set directly. The actuation follows acb's contract on CARLA: Autoware's
`Control` asks for a longitudinal acceleration and a tire angle, and the vehicle realizes
them. Longitudinally that is a force `m a + F_resist(v)` split over the four wheels as drive
torque, or as brake torque when it opposes the motion; laterally the bicycle tire angle is
split into Ackermann angles for the two front wheels, after the optional actuator lag and
rate limit acb also has (both off by default).
"""
import math
from dataclasses import dataclass, field

from omni.physx.scripts.utils import set_custom_metadata
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from .vehicle_params import G, VehicleParams  # noqa: F401  (VehicleParams: re-exported)

GROUPS = "/World/vehicleCollisionGroups"
WHEEL_MASS = 20.0  # kg
FRICTION_TABLE = "/World/vehicleTireFriction"

# Autoware's GearCommand / GearReport values
GEAR_NEUTRAL, GEAR_DRIVE, GEAR_REVERSE, GEAR_PARK = 1, 2, 20, 22


@dataclass
class Command:
    """The latest of Autoware's commands, as the vehicle bridge keeps them."""

    acceleration: float = 0.0
    velocity: float = 0.0
    tire_angle: float = 0.0
    gear: int = GEAR_DRIVE
    received: bool = False


@dataclass
class State:
    """The vehicle as Autoware's status topics and the IMU see it, in base_link."""

    position: tuple = (0.0, 0.0, 0.0)
    quat: tuple = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
    velocity: tuple = (0.0, 0.0, 0.0)  # base_link frame, m/s
    angular_velocity: tuple = (0.0, 0.0, 0.0)  # base_link frame, rad/s
    acceleration: tuple = (0.0, 0.0, 0.0)  # base_link frame, m/s^2, gravity not included
    tire_angle: float = 0.0
    accel_history: list = field(default_factory=list)


def _rotate(q, v):
    """Rotate v by unit quaternion q (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


def _conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def ensure_vehicle_scene(stage):
    """The physics scene's vehicle context, the collision groups and the friction table."""
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            ctx = PhysxSchema.PhysxVehicleContextAPI.Apply(prim)
            ctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
            ctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posZ)
            ctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posX)
    if not stage.GetPrimAtPath(GROUPS):
        UsdGeom.Scope.Define(stage, GROUPS)
        chassis = UsdPhysics.CollisionGroup.Define(stage, f"{GROUPS}/chassis")
        query = UsdPhysics.CollisionGroup.Define(stage, f"{GROUPS}/groundQuery")
        # A vehicle's suspension raycasts do not hit its own (or any) chassis.
        chassis.CreateFilteredGroupsRel().AddTarget(query.GetPath())
        query.CreateFilteredGroupsRel().AddTarget(chassis.GetPath())
    if not stage.GetPrimAtPath(FRICTION_TABLE):
        table = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, FRICTION_TABLE)
        table.CreateGroundMaterialsRel()
        table.CreateFrictionValuesAttr([])
        table.CreateDefaultFrictionValueAttr(1.0)


class Vehicle:
    def __init__(self, stage, path, params: VehicleParams, box_center, box_dims, pose):
        self.stage = stage
        self.path = path
        self.p = params
        ensure_vehicle_scene(stage)
        FRICTION = stage.GetPrimAtPath(FRICTION_TABLE)
        PhysxSchema.PhysxVehicleTireFrictionTable(FRICTION).GetDefaultFrictionValueAttr().Set(
            params.tire_friction)
        root = UsdGeom.Xform.Define(stage, path)
        prim = root.GetPrim()
        self.translate = root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        self.orient = root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        self._set_xform(pose)
        set_custom_metadata(prim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(params.mass)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*params.com))
        lx, ly, lz = 4.8, 1.9, 1.4
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(ly * ly + lz * lz, lx * lx + lz * lz, lx * lx + ly * ly)
                                       * (params.mass / 12.0))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        body.CreateSleepThresholdAttr(0)
        body.CreateStabilizationThresholdAttr(0)
        body.CreateDisableGravityAttr(True)  # the vehicle simulation applies gravity

        v = PhysxSchema.PhysxVehicleAPI.Apply(prim)
        v.CreateVehicleEnabledAttr(True)
        v.CreateSuspensionLineQueryTypeAttr(PhysxSchema.Tokens.raycast)
        v.CreateSubStepThresholdLongitudinalSpeedAttr(5.0)
        v.CreateLowForwardSpeedSubStepCountAttr(params.physx_substeps)
        v.CreateHighForwardSpeedSubStepCountAttr(params.physx_substeps)
        v.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0)
        v.CreateMinActiveLongitudinalSlipDenominatorAttr(params.min_active_slip_denominator)
        v.CreateMinLateralSlipDenominatorAttr(1.0)
        v.CreateLongitudinalStickyTireThresholdSpeedAttr(0.2)
        v.CreateLongitudinalStickyTireThresholdTimeAttr(1.0)
        v.CreateLongitudinalStickyTireDampingAttr(200.0)
        v.CreateLateralStickyTireThresholdSpeedAttr(0.2)
        v.CreateLateralStickyTireThresholdTimeAttr(1.0)
        v.CreateLateralStickyTireDampingAttr(20.0)

        r, half_t, L = params.wheel_radius, params.wheel_tread / 2, params.wheel_base
        omega = 2 * math.pi * params.suspension_frequency
        # Every spring at the same frequency on its own sprung mass sags the same, sag = g/w^2:
        # the car sits level whatever the front/rear split.
        sag = G / omega ** 2
        # The suspension frame sits so that, sagged under the static load, the wheel centre is
        # at z = r: base_link on the ground at rest.
        frame_z = r + (params.suspension_travel - sag)
        front_sprung, rear_sprung = params.axle_loads()
        self.wheels = []
        for i, (name, x, y) in enumerate((("frontLeft", L, half_t), ("frontRight", L, -half_t),
                                          ("rearLeft", 0.0, half_t), ("rearRight", 0.0, -half_t))):
            sprung = front_sprung if i < 2 else rear_sprung
            k = sprung * omega ** 2
            c = 2 * params.suspension_damping_ratio * math.sqrt(k * sprung)
            rest_load = sprung * G
            wp = f"{path}/{name}"
            w = UsdGeom.Xform.Define(stage, wp)
            w.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(x, y, r))
            wprim = w.GetPrim()
            att = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(wprim)
            wheel = PhysxSchema.PhysxVehicleWheelAPI.Apply(wprim)
            wheel.CreateRadiusAttr(r)
            wheel.CreateWidthAttr(params.wheel_width)
            wheel.CreateMassAttr(WHEEL_MASS)
            wheel.CreateMoiAttr(0.5 * WHEEL_MASS * r * r)
            wheel.CreateDampingRateAttr(params.wheel_damping_rate)
            # CARLA's PhysX 4 tire, in PhysX 5's terms: lateral stiffness rises with load up
            # to lat_stiff_max_load rest loads, where it is lat_stiff_value per N of rest
            # load; longitudinal stiffness is given per g.
            tire = PhysxSchema.PhysxVehicleTireAPI.Apply(wprim)
            tire.CreateLateralStiffnessGraphAttr(
                Gf.Vec2f(params.lat_stiff_max_load, params.lat_stiff_value * rest_load))
            tire.CreateLongitudinalStiffnessAttr(params.long_stiff_value * G)
            tire.CreateCamberStiffnessAttr(0.0)
            tire.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
            tire.CreateFrictionTableRel().AddTarget(FRICTION_TABLE)
            susp = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wprim)
            susp.CreateSprungMassAttr(sprung)
            susp.CreateSpringStrengthAttr(k)
            susp.CreateSpringDamperRateAttr(c)
            susp.CreateTravelDistanceAttr(params.suspension_travel)
            att.CreateIndexAttr(i)
            att.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, 0, -1))
            att.CreateSuspensionFramePositionAttr(Gf.Vec3f(x, y, frame_z))
            att.CreateSuspensionFrameOrientationAttr(Gf.Quatf(1, 0, 0, 0))
            att.CreateCollisionGroupRel().AddTarget(f"{GROUPS}/groundQuery")
            comp = PhysxSchema.PhysxVehicleSuspensionComplianceAPI.Apply(wprim)
            comp.CreateWheelToeAngleAttr([])
            comp.CreateWheelCamberAngleAttr([])
            app_point = Gf.Vec4f(0.0, 0.0, 0.0, r - frame_z)
            comp.CreateSuspensionForceAppPointAttr([app_point])
            comp.CreateTireForceAppPointAttr([app_point])
            ctl = PhysxSchema.PhysxVehicleWheelControllerAPI.Apply(wprim)
            self.wheels.append((ctl.CreateDriveTorqueAttr(0.0), ctl.CreateBrakeTorqueAttr(0.0),
                                ctl.CreateSteerAngleAttr(0.0)))
            # what an operator sees: a wheel
            cyl = UsdGeom.Cylinder.Define(stage, f"{wp}/render")
            cyl.CreateRadiusAttr(r)
            cyl.CreateHeightAttr(params.wheel_width)
            cyl.CreateAxisAttr(UsdGeom.Tokens.y)
            cyl.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.08)])

        # The chassis: the declared box, kept clear of the ground so only the wheels touch it.
        cx, cy, cz = box_center
        dx, dy, dz = box_dims
        bottom = max(cz - dz / 2, r + 0.05)
        top = cz + dz / 2
        cube = UsdGeom.Cube.Define(stage, f"{path}/body")
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.45, 0.85)])
        UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(cx, cy, (bottom + top) / 2))
        UsdGeom.XformCommonAPI(cube.GetPrim()).SetScale(Gf.Vec3f(dx, dy, top - bottom))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionGroup.Get(stage, f"{GROUPS}/chassis").GetCollidersCollectionAPI() \
            .CreateIncludesRel().AddTarget(cube.GetPath())
        self.tire_angle = 0.0
        self.last = None  # (position, quat, velocity in world) of the last substep

    # --- Pose ------------------------------------------------------------------------------

    def _set_xform(self, pose):
        (x, y, z), (w, qx, qy, qz) = pose
        self.translate.Set(Gf.Vec3d(x, y, z))
        n = math.sqrt(w * w + qx * qx + qy * qy + qz * qz) or 1.0
        self.orient.Set(Gf.Quatd(w / n, qx / n, qy / n, qz / n))

    def teleport(self, pose, physx):
        """Put it at `pose` at rest (a scenario's teleport, or SSv2 driving the ego)."""
        self._set_xform(pose)
        body = self.stage.GetPrimAtPath(self.path)
        UsdPhysics.RigidBodyAPI(body).GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        UsdPhysics.RigidBodyAPI(body).GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        try:
            physx.set_vehicle_to_rest_state(self.path)
        except Exception:  # not simulated yet: the USD pose is all there is
            pass
        self.last = None

    # --- Actuation -------------------------------------------------------------------------

    def apply(self, cmd: Command, dt, state: State):
        p = self.p
        # Lateral: tire angle through the optional actuator, then Ackermann.
        target = max(-p.max_steer_angle, min(p.max_steer_angle, cmd.tire_angle))
        a = self.tire_angle
        if p.steer_time_constant > 0:
            target = a + (target - a) * (1 - math.exp(-dt / p.steer_time_constant))
        if p.steer_rate_limit > 0:
            step = p.steer_rate_limit * dt
            target = max(a - step, min(a + step, target))
        self.tire_angle = target
        if abs(target) < 1e-5:
            left = right = 0.0
        else:
            radius = p.wheel_base / math.tan(target)
            half_t = p.wheel_tread / 2
            left = math.atan(p.wheel_base / (radius - half_t))
            right = math.atan(p.wheel_base / (radius + half_t))
        # Longitudinal
        v = state.velocity[0]
        r = p.wheel_radius
        drive = brake = 0.0
        hold = (cmd.gear == GEAR_PARK
                or (abs(cmd.velocity) < 0.01 and cmd.acceleration <= 0.01 and abs(v) < 0.1))
        if hold:
            brake = p.hold_brake_torque
        elif cmd.gear != GEAR_NEUTRAL:
            direction = -1.0 if cmd.gear == GEAR_REVERSE else 1.0
            # Along the gear's direction: positive pushes that way, negative slows it down.
            # The wheels spin up with the car: their inertia is mass the torque also moves.
            mass = p.mass + 4 * 0.5 * WHEEL_MASS
            push = mass * cmd.acceleration + p.rolling_resistance * abs(v)
            if push >= 0:
                drive = direction * min(p.max_drive_torque, push * r / 4)
            else:
                # Brakes, never the motor: decelerating to a stop does not drive it backwards.
                brake = min(-push * r / 4, p.max_brake_torque)
        for i, (d, b, s) in enumerate(self.wheels):
            d.Set(drive)
            b.Set(brake)
            s.Set(left if i == 0 else right if i == 1 else 0.0)

    # --- State -----------------------------------------------------------------------------

    def sample(self, physx, dt, state: State):
        """Read the body after one physics step; velocities by difference (base_link frame)."""
        t = physx.get_rigidbody_transformation(self.path)
        pos = tuple(float(c) for c in t["position"])
        rx, ry, rz, rw = (float(c) for c in t["rotation"])
        q = (rw, rx, ry, rz)
        if self.last is None or dt <= 0:
            vel_w = (0.0, 0.0, 0.0)
            omega = (0.0, 0.0, 0.0)
            acc = (0.0, 0.0, 0.0)
        else:
            lp, lq, lv = self.last
            vel_w = tuple((a - b) / dt for a, b in zip(pos, lp))
            dq = _qmul(_conj(lq), q)  # body-frame rotation over the step
            if dq[0] < 0:
                dq = tuple(-c for c in dq)
            s = math.sqrt(dq[1] ** 2 + dq[2] ** 2 + dq[3] ** 2)
            angle = 2 * math.atan2(s, dq[0])
            omega = (0.0, 0.0, 0.0) if s < 1e-12 else tuple(c / s * angle / dt for c in dq[1:])
            acc = _rotate(_conj(q), tuple((a - b) / dt for a, b in zip(vel_w, lv)))
        self.last = (pos, q, vel_w)
        state.position = pos
        state.quat = q
        state.velocity = _rotate(_conj(q), vel_w)
        state.angular_velocity = omega
        state.acceleration = acc
        state.tire_angle = self.tire_angle
