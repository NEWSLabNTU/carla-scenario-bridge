"""The ego on Isaac (roadmap 038): its PhysX vehicle, its sensors, and its ROS 2 side.

What acb_bridge is to a CARLA vehicle, this is to an Isaac one, in the world server's
process (the sensors render there): on the ego's own ROS domain (036's mechanism: an rclpy
context with `domain_id` explicit, here made with the world server, `EgoRos`) it publishes Autoware's vehicle status, `/clock`, the lidar, the
IMU and the GNSS, and takes `/control/command/*`; Autoware runs `acb_launch
carla_simulator.launch.xml` with `launch_vehicle_interface:=false`, so nothing else does.

Configured by an ego YAML (`config/acb_ego.yaml`): the domain, the Autoware vehicle
description and sensor calibration it reads geometry from, the vehicle's physics, the
sensors. Topic names and frames are acb's (acb_bridge `vehicle_config.yaml`), so the same
sensor kit launch consumes them:

| topic | type | frame |
|---|---|---|
| `/sensing/lidar/top/pointcloud_before_sync` | PointCloud2 (x y z intensity return_type channel) | `top` |
| `/sensing/imu/carla/imu_link/imu_raw` | Imu | `carla/imu_link` |
| `/sensing/gnss/carla/gnss_link/nav_sat_fix` | NavSatFix | `carla/gnss_link` |
| `/vehicle/status/{velocity,steering,gear,control_mode,turn_indicators,hazard_lights}_status` | Autoware | `base_link` |
| `/clock` | Clock | |

Stamps are the frame's sim time plus an offset fixed when the world server starts: by default
(`clock_offset: wall`) the wall clock at that moment, so the ego's time never goes backwards
across world server restarts -- a long-lived Autoware stack that sees `/clock` jump back
clears its TF buffers and its EKF, and then never drives again (seen in 038). It also keeps
the clock clear of zero, where Autoware misbehaves (acb holds `/clock` back below 60 s).
Everything is published once per frame from the world server's loop, after the frame's
physics steps and render, and the commands that arrived before a frame drive it: lockstep,
as acb follows CARLA's ticks. The lidar scan alone goes out one frame late, after the next
frame's `/clock`, with its own stamp: sent with its frame's `/clock` it raced the EKF pose of
that stamp and NDT could not interpolate, which flapped MRM all through a drive (038).
"""
import array
import logging
import time
from dataclasses import dataclass, field, fields
from pathlib import Path

import numpy as np
import yaml

from .frames import kit_transform, lat_lon, qconj, qmul, qrot
from .vehicle import GEAR_DRIVE, GEAR_NEUTRAL, GEAR_PARK, GEAR_REVERSE, Command, State, Vehicle
from .vehicle_params import VehicleParams, read_vehicle_info

log = logging.getLogger("csb_isaac")

MAX_CALLBACKS_PER_FRAME = 16
WARM_UP_FRAMES = 60


@dataclass
class EgoConfig:
    domain: int = 1
    # The CARLA blueprint whose body the ego wears (its geometry is checked against vehicle_info).
    model: str = "vehicle.tesla.model3"
    vehicle_info: str = ""
    sensors_calibration: str = ""
    sensor_kit_calibration: str = ""
    clock_offset: object = "wall"  # s, or "wall": the wall clock at world server start
    vehicle: dict = field(default_factory=dict)
    lidar: dict = field(default_factory=dict)
    imu: dict = field(default_factory=dict)
    gnss: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path):
        path = Path(path)
        doc = yaml.safe_load(path.read_text()) or {}
        unknown = set(doc) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"{path}: unknown key(s) {sorted(unknown)}")
        cfg = cls(**doc)
        for name in ("vehicle_info", "sensors_calibration", "sensor_kit_calibration"):
            value = getattr(cfg, name)
            if value and not Path(value).is_absolute():
                setattr(cfg, name, str((path.parent / value).resolve()))
        return cfg


class EgoRos:
    """The ego's ROS 2 side: its own context on its domain, one node.

    Made once, with the world, and kept across egos: a new DDS participant in a domain of some
    200 Autoware ones takes seconds to be discovered (the first scans of a fresh ego would go
    nowhere), and `/clock` stays continuous between scenarios instead of starting at spawn."""

    def __init__(self, cfg: EgoConfig):
        import rclpy
        from autoware_control_msgs.msg import Control
        from autoware_vehicle_msgs.msg import (ControlModeReport, GearCommand, GearReport,
                                               HazardLightsCommand, HazardLightsReport,
                                               SteeringReport, TurnIndicatorsCommand,
                                               TurnIndicatorsReport, VelocityReport)
        from autoware_vehicle_msgs.srv import ControlModeCommand
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField

        self.m = dict(Clock=Clock, VelocityReport=VelocityReport, SteeringReport=SteeringReport,
                      GearReport=GearReport, ControlModeReport=ControlModeReport,
                      TurnIndicatorsReport=TurnIndicatorsReport,
                      HazardLightsReport=HazardLightsReport, Imu=Imu, NavSatFix=NavSatFix,
                      NavSatStatus=NavSatStatus, PointCloud2=PointCloud2, PointField=PointField)
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=cfg.domain)
        self.node = rclpy.create_node("isaac_vehicle", context=self.ctx)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        n = self.node
        self.pub = {
            "clock": n.create_publisher(Clock, "/clock", 10),
            "velocity": n.create_publisher(VelocityReport, "/vehicle/status/velocity_status", reliable),
            "steering": n.create_publisher(SteeringReport, "/vehicle/status/steering_status", reliable),
            "gear": n.create_publisher(GearReport, "/vehicle/status/gear_status", reliable),
            "mode": n.create_publisher(ControlModeReport, "/vehicle/status/control_mode", reliable),
            "turn": n.create_publisher(TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", reliable),
            "hazard": n.create_publisher(HazardLightsReport, "/vehicle/status/hazard_lights_status", reliable),
            "lidar": n.create_publisher(PointCloud2, cfg.lidar.get("topic", "/sensing/lidar/top/pointcloud_before_sync"),
                                        qos_profile_sensor_data),
            "imu": n.create_publisher(Imu, cfg.imu.get("topic", "/sensing/imu/carla/imu_link/imu_raw"), reliable),
            "gnss": n.create_publisher(NavSatFix, cfg.gnss.get("topic", "/sensing/gnss/carla/gnss_link/nav_sat_fix"),
                                       reliable),
        }
        self.cfg = cfg
        if cfg.clock_offset == "wall":
            self.clock_offset = float(int(time.time()))
        else:
            self.clock_offset = float(cfg.clock_offset)
        self.reset()

        def on_control(msg):
            self.cmd.acceleration = float(msg.longitudinal.acceleration)
            self.cmd.velocity = float(msg.longitudinal.velocity)
            self.cmd.tire_angle = float(msg.lateral.steering_tire_angle)
            self.cmd.received = True

        def on_gear(msg):
            if msg.command in (GEAR_DRIVE, GEAR_REVERSE, GEAR_PARK, GEAR_NEUTRAL):
                self.cmd.gear = int(msg.command)
            elif 2 <= msg.command <= 19:  # DRIVE_2 ... DRIVE_18
                self.cmd.gear = GEAR_DRIVE

        def on_turn(msg):
            # Command NO_COMMAND(0) keeps the last; DISABLE 1, LEFT 2, RIGHT 3 as the report's
            if msg.command:
                self.turn = int(msg.command)

        def on_hazard(msg):
            if msg.command:
                self.hazard = int(msg.command)

        def on_mode(req, res):
            res.success = req.mode == 1  # AUTONOMOUS, as acb (manual has no driver here)
            return res

        n.create_subscription(Control, "/control/command/control_cmd", on_control, reliable)
        n.create_subscription(GearCommand, "/control/command/gear_cmd", on_gear, reliable)
        n.create_subscription(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", on_turn, reliable)
        n.create_subscription(HazardLightsCommand, "/control/command/hazard_lights_cmd", on_hazard, reliable)
        n.create_service(ControlModeCommand, "/control/control_mode_request", on_mode)
        log.info("egos: ROS 2 on domain %d", cfg.domain)

    def reset(self):
        """A new ego starts from no command: standing, in drive."""
        self.cmd = Command()
        self.turn = 1  # TurnIndicatorsReport.DISABLE
        self.hazard = 1  # HazardLightsReport.DISABLE

    def publish_clock(self, sim_time):
        """`/clock` at `sim_time` (+ the offset); returns that stamp."""
        stamp = self.stamp(sim_time + self.clock_offset)
        self.pub["clock"].publish(self.m["Clock"](clock=stamp))
        return stamp

    def take_commands(self):
        """Run the callbacks that are ready, without waiting: one per `spin_once`, and a frame
        sees a handful (control at 30 Hz, the rest at 10 Hz, against 20 frames a second)."""
        for _ in range(MAX_CALLBACKS_PER_FRAME):
            self.executor.spin_once(timeout_sec=0)

    def stamp(self, t):
        from builtin_interfaces.msg import Time

        ns = int(round(t * 1e9))
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def close(self):
        import rclpy

        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def ego_params(cfg: EgoConfig, physics=None):
    """The ego's VehicleParams: geometry from `cfg.vehicle_info`, physics from `physics` (the
    catalogue's for `cfg.model`), then the YAML's `vehicle:` overrides. Raises
    `VehicleMismatch` when vehicle_info and the catalogue describe different cars."""
    info = read_vehicle_info(cfg.vehicle_info) if cfg.vehicle_info else None
    return VehicleParams.resolve(cfg.model, physics, info, cfg.vehicle)


class Ego:
    """One ego: vehicle, sensors and ROS side, built at spawn and removed at despawn."""

    def __init__(self, cfg: EgoConfig, ros: EgoRos, stage, path, name, box_center, box_dims, pose,
                 params: VehicleParams = None):
        self.cfg = cfg
        self.name = name
        self.path = path
        self.vehicle = Vehicle(stage, path, params or ego_params(cfg), box_center, box_dims, pose)
        self.state = State()
        self.lidar = None
        self.lidar_frame = cfg.lidar.get("frame", "top")
        if cfg.lidar.get("enabled", True):
            self._make_lidar(stage)
        self.imu_tf = self._kit_tf(cfg.imu.get("link", "carla/imu_link"))
        self.gnss_tf = self._kit_tf(cfg.gnss.get("link", "carla/gnss_link"))
        self.gnss_period = float(cfg.gnss.get("period", 0.1))
        self.next_gnss = 0.0
        self.ros = ros
        ros.reset()
        self.prev_vel = None
        # The lidar scan of the last frame, sent one frame late (see `publish`).
        self.pending_cloud = None

    def _kit_tf(self, link):
        if self.cfg.sensors_calibration and self.cfg.sensor_kit_calibration:
            return kit_transform(self.cfg.sensors_calibration, self.cfg.sensor_kit_calibration, link)
        return ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))

    def _make_lidar(self, stage):
        import omni.replicator.core as rep
        from isaacsim.sensors.experimental.rtx import Lidar
        from pxr import Gf, UsdGeom

        (t, q) = self._kit_tf(self.cfg.lidar.get("link", "velodyne_top_base_link"))
        mount = UsdGeom.Xform.Define(stage, f"{self.path}/lidar_mount")
        mount.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*t))
        mount.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*q))
        lidar = Lidar.create(f"{self.path}/lidar_mount/lidar", config=self.cfg.lidar.get("config", "OS1"),
                             variant=self.cfg.lidar.get("variant", "OS1_REV7_128ch20hz1024res"))
        self.render_product = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
        self.lidar = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloud")
        self.lidar.attach([self.render_product.path])
        self.channels = int(self.cfg.lidar.get("channels", 128))
        log.info("ego %r: lidar %s at %s in base_link", self.name, lidar.paths[0], t)

    def warm_up(self, step, frames=WARM_UP_FRAMES):
        """Step (physics and a render) until the lidar delivers: its first frames set up the
        RTX sensor pipeline, seconds on a loaded host, which belong in the spawn rather than
        in the first frame's step (csb times a step out). Rendering alone produces no scan:
        the lidar fires on simulation time. The vehicle stands on its hold brake meanwhile
        (no command yet), which also lets its suspension settle."""
        if self.lidar is None:
            return 0
        for i in range(frames):
            step()
            d = self.lidar.get_data()
            if isinstance(d, dict) and len(d.get("data", [])):
                return i + 1
        log.warning("ego %r: lidar gave no points in %d warm-up frames", self.name, frames)
        return frames

    # --- the frame -------------------------------------------------------------------------

    def before_substep(self, dt):
        self.vehicle.apply(self.ros.cmd, dt, self.state)

    def after_substep(self, physx, dt):
        self.vehicle.sample(physx, dt, self.state)

    def publish(self, sim_time, stamp):
        """Everything Autoware gets for this frame but `/clock` (the world's), at `stamp`."""
        m = self.ros.m
        pub = self.ros.pub
        s = self.state
        v = m["VelocityReport"]()
        v.header.stamp = stamp
        v.header.frame_id = "base_link"
        v.longitudinal_velocity = float(s.velocity[0])
        v.lateral_velocity = float(s.velocity[1])
        v.heading_rate = float(s.angular_velocity[2])
        pub["velocity"].publish(v)
        st = m["SteeringReport"]()
        st.stamp = stamp
        st.steering_tire_angle = float(s.tire_angle)
        pub["steering"].publish(st)
        g = m["GearReport"]()
        g.stamp = stamp
        g.report = self.ros.cmd.gear
        pub["gear"].publish(g)
        cm = m["ControlModeReport"]()
        cm.stamp = stamp
        cm.mode = 1  # AUTONOMOUS
        pub["mode"].publish(cm)
        tr = m["TurnIndicatorsReport"]()
        tr.stamp = stamp
        tr.report = self.ros.turn
        pub["turn"].publish(tr)
        hz = m["HazardLightsReport"]()
        hz.stamp = stamp
        hz.report = self.ros.hazard
        pub["hazard"].publish(hz)
        self._publish_imu(stamp)
        if sim_time + 1e-9 >= self.next_gnss:
            self._publish_gnss(stamp)
            self.next_gnss = sim_time + self.gnss_period
        if self.lidar is not None:
            # One frame of sensor latency, stamp unchanged (the scan's capture time). Sent in
            # the same instant as `/clock` of its own frame, a scan races the EKF pose of that
            # stamp to ndt_scan_matcher, whose pose buffer then holds one pose and cannot
            # interpolate ("Couldn't interpolate pose": WARN -> /autoware/localization not OK
            # -> autonomous unavailable -> MRM emergency_stop for a cycle). On CARLA the scan
            # arrives after the tick's /clock; on a vehicle, 50-100 ms after capture.
            if self.pending_cloud is not None:
                self.ros.pub["lidar"].publish(self.pending_cloud)
            self.pending_cloud = self._lidar_msg(stamp)

    def _publish_imu(self, stamp):
        s = self.state
        (_, q_bi) = self.imu_tf
        # Specific force: acceleration minus gravity, in base_link, then in the IMU frame.
        g_body = qrot(qconj(s.quat), (0.0, 0.0, -9.81))
        f = tuple(a - gg for a, gg in zip(s.acceleration, g_body))
        acc = qrot(qconj(q_bi), f)
        gyro = qrot(qconj(q_bi), s.angular_velocity)
        q = qmul(s.quat, q_bi)
        msg = self.ros.m["Imu"]()
        msg.header.stamp = stamp
        msg.header.frame_id = self.cfg.imu.get("frame", "carla/imu_link")
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = (float(c) for c in q)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(c) for c in gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(c) for c in acc)
        self.ros.pub["imu"].publish(msg)

    def _publish_gnss(self, stamp):
        s = self.state
        (t, _) = self.gnss_tf
        p = tuple(a + b for a, b in zip(s.position, qrot(s.quat, t)))
        lat, lon = lat_lon(p[0], p[1])
        msg = self.ros.m["NavSatFix"]()
        msg.header.stamp = stamp
        msg.header.frame_id = self.cfg.gnss.get("frame", "carla/gnss_link")
        msg.status.status = 2  # GBAS_FIX, as acb
        msg.status.service = 1
        msg.latitude, msg.longitude, msg.altitude = lat, lon, float(p[2])
        msg.position_covariance_type = 0
        self.ros.pub["gnss"].publish(msg)

    def _lidar_msg(self, stamp):
        d = self.lidar.get_data()
        if not isinstance(d, dict):
            return None
        xyz = np.asarray(d["data"], dtype=np.float32).reshape(-1, 3)
        if not len(xyz):
            return None
        info = d.get("info", {})
        n = len(xyz)
        cloud = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "u1"),
                                   ("return_type", "u1"), ("channel", "<u2")])
        cloud["x"], cloud["y"], cloud["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        inten = np.asarray(info.get("intensity", []), dtype=np.float32)
        if len(inten) == n:
            cloud["intensity"] = np.clip(inten * 255.0, 0, 255).astype(np.uint8)
        elev = np.asarray(info.get("elevation", []), dtype=np.float32)
        if len(elev) == n:
            lo, hi = float(elev.min()), float(elev.max())
            if hi > lo:
                cloud["channel"] = np.rint((elev - lo) / (hi - lo) * (self.channels - 1)).astype(np.uint16)
        cloud["return_type"] = 1
        m = self.ros.m
        PF = m["PointField"]
        msg = m["PointCloud2"]()
        msg.header.stamp = stamp
        msg.header.frame_id = self.lidar_frame
        msg.height = 1
        msg.width = n
        msg.fields = [PF(name="x", offset=0, datatype=PF.FLOAT32, count=1),
                      PF(name="y", offset=4, datatype=PF.FLOAT32, count=1),
                      PF(name="z", offset=8, datatype=PF.FLOAT32, count=1),
                      PF(name="intensity", offset=12, datatype=PF.UINT8, count=1),
                      PF(name="return_type", offset=13, datatype=PF.UINT8, count=1),
                      PF(name="channel", offset=14, datatype=PF.UINT16, count=1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        # array.array, not bytes: rclpy copies an array in C, bytes element by element (40 ms)
        msg.data = array.array("B", cloud.tobytes())
        return msg

    def close(self):
        if self.lidar is not None:
            try:
                self.lidar.detach([self.render_product.path])
                self.render_product.destroy()
            except Exception:  # noqa: BLE001 - the stage is going anyway
                log.exception("ego %r: lidar teardown", self.name)
