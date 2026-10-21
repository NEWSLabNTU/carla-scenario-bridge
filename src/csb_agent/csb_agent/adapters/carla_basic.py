"""`CarlaBasicAgent`: drive the scenario's vehicle with CARLA's own BasicAgent (roadmap 030).

The SDK's reference adapter and the agent of a run with no autopilot at all: no ROS, no
Autoware. It finds the vehicle carla-scenario-bridge spawned for the entity (by CARLA
`role_name`, the bridge's `ego.role_name`, default `hero`), and drives it with
`agents.navigation.basic_agent.BasicAgent` from CARLA's PythonAPI: the route is planned
through the scenario's waypoints with BasicAgent's global planner, the target speed is the
scenario's speed limit (else `target_speed_kmh`), a stop brakes and holds.

    python3 -m csb_agent.adapters.carla_basic --relay tcp://localhost:5560 --entity ego \\
        [--config carla_basic.yaml]

Config (JSON or YAML, all optional):

    carla_host: localhost          # CARLA's RPC endpoint
    carla_port: 2000
    role_name: hero                # the vehicle's CARLA role_name
    carla_pythonapi: <dir>         # CARLA's PythonAPI/carla (holds `agents`); default
                                   # $CARLA_DIR/PythonAPI/carla
    target_speed_kmh: 30           # with no speed limit from the scenario
    ignore_traffic_lights: false   # BasicAgent's own hazard checks
    ignore_vehicles: false
    find_timeout: 30               # s to wait at a reset for the vehicle to exist
    world: carla                   # `fake`: a kinematic stand-in, no CARLA (csb-agent check)

What it never does to the simulator: the bridge owns the world. The agent never ticks it,
never changes its settings, never spawns or destroys an actor, and applies control to its
own vehicle only (BasicAgent reads the other actors to avoid them). Control runs on world
ticks: a `world.on_tick` callback only wakes the control thread -- a synchronous server
waits for its subscribers, so the callback must be quick -- and the thread runs
`run_step()` and applies the result. A CARLA client must not outlive its server (a dead
client's stream can hang the next server on the port): when the server stops answering,
the process exits with status 3, for whoever runs it to start it again.

The pose it reports is the vehicle's in the map frame (ROS: CARLA's y and yaw negated),
moved from CARLA's actor origin to the scenario's vehicle origin by the offset measured at
each reset, where both are known to be the same pose.
"""

import math
import os
import sys
import threading
import time

import csb_agent
from csb_agent import protocol as P

DEFAULTS = {
    "carla_host": "localhost",
    "carla_port": 2000,
    "role_name": "hero",
    "carla_pythonapi": "",
    "target_speed_kmh": 30.0,
    "ignore_traffic_lights": False,
    "ignore_vehicles": False,
    "find_timeout": 30.0,
    "world": "carla",
}
PROBE_PERIOD = 10.0  # s between liveness probes while no tick arrives
TICK_STALE = 2.0  # s without a tick before probing the server
POSE_PERIOD = 0.2  # s between pose reports


# -- frames --------------------------------------------------------------------------------
def yaw_of(pose):
    return math.atan2(2.0 * (pose["qw"] * pose["qz"] + pose["qx"] * pose["qy"]),
                      1.0 - 2.0 * (pose["qy"] ** 2 + pose["qz"] ** 2))


def map_pose(x, y, z, yaw):
    """A protocol pose (map frame) from map-frame x, y, z, yaw."""
    return {"x": x, "y": y, "z": z, "qx": 0.0, "qy": 0.0,
            "qz": math.sin(yaw / 2.0), "qw": math.cos(yaw / 2.0)}


# -- the world, real and fake --------------------------------------------------------------
class FakeWorld:
    """A kinematic stand-in for CARLA: one vehicle that moves straight at the target speed
    to the end of its route. Enough for `csb-agent check`, which tests the protocol."""

    TICK = 0.05

    def __init__(self, config, log):
        self.log = log
        self.position = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.route = []
        self.speed = 0.0
        self.on_tick = None
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop.wait(self.TICK):
            if self.on_tick:
                self.on_tick()

    def find_vehicle(self):
        return True

    def vehicle_pose(self):
        x, y, z = self.position
        return x, y, z, self.yaw

    def place(self, pose):
        self.position = (pose["x"], pose["y"], pose["z"])
        self.yaw = yaw_of(pose)

    def plan(self, points, speed_kmh, options):
        self.route = [(p["x"], p["y"], p["z"]) for p in points]
        self.speed = speed_kmh / 3.6

    def set_speed(self, speed_kmh):
        self.speed = speed_kmh / 3.6

    def step(self, dt=TICK):
        """Drive one tick; True when the route is done."""
        while self.route:
            tx, ty, tz = self.route[0]
            dx, dy = tx - self.position[0], ty - self.position[1]
            d = math.hypot(dx, dy)
            reach = self.speed * dt
            if d <= max(reach, 1e-6):
                self.position = (tx, ty, tz)
                self.route.pop(0)
                continue
            self.yaw = math.atan2(dy, dx)
            self.position = (self.position[0] + dx / d * reach,
                             self.position[1] + dy / d * reach, tz)
            return False
        return True

    def hold(self):
        pass

    def alive(self):
        return True

    def close(self):
        self._stop.set()


class CarlaWorld:
    """CARLA through its Python client: the vehicle, BasicAgent, the tick subscription."""

    def __init__(self, config, log):
        self.config = config
        self.log = log
        api = config["carla_pythonapi"] or os.path.join(os.environ.get("CARLA_DIR", ""),
                                                        "PythonAPI", "carla")
        if api and os.path.isdir(os.path.join(api, "agents")) and api not in sys.path:
            sys.path.insert(0, api)
        import carla  # noqa: F401 -- fail at start, not at the first reset
        from agents.navigation.basic_agent import BasicAgent  # needs shapely, networkx
        self.carla = carla
        self.BasicAgent = BasicAgent
        self.client = carla.Client(config["carla_host"], int(config["carla_port"]))
        self.client.set_timeout(10.0)
        self.world = None
        self.world_id = None
        self.map = None
        self.grp = None
        self.tick_id = None
        self.vehicle = None
        self.agent = None
        self.on_tick = None
        self.last_tick = time.monotonic()

    def _tick(self, _snapshot):
        self.last_tick = time.monotonic()
        if self.on_tick:
            self.on_tick()  # only wakes the control thread

    def _attach_world(self):
        world = self.client.get_world()
        if world.id == self.world_id:
            return
        if self.tick_id is not None and self.world is not None:
            try:
                self.world.remove_on_tick(self.tick_id)
            except RuntimeError:
                pass
        self.world, self.world_id = world, world.id
        self.map = world.get_map()
        from agents.navigation.global_route_planner import GlobalRoutePlanner
        self.grp = GlobalRoutePlanner(self.map, 2.0)
        self.tick_id = world.on_tick(self._tick)
        self.log.info(f"world {world.id}: {self.map.name}")

    def find_vehicle(self):
        self._attach_world()
        role = self.config["role_name"]
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == role:
                if self.vehicle is None or self.vehicle.id != actor.id:
                    self.log.info(f"vehicle {actor.id} ({actor.type_id}) role_name={role}")
                self.vehicle = actor
                self.agent = None
                return True
        self.vehicle = None
        return False

    def place(self, pose):
        """Nothing: the bridge has placed the vehicle (the agent never moves actors)."""

    def vehicle_pose(self):
        if self.vehicle is None:
            return None
        t = self.vehicle.get_transform()
        return t.location.x, -t.location.y, t.location.z, -math.radians(t.rotation.yaw)

    def plan(self, points, speed_kmh, options):
        agent = self.BasicAgent(self.vehicle, target_speed=speed_kmh,
                                opt_dict={"ignore_traffic_lights": options["lights"],
                                          "ignore_vehicles": options["vehicles"]},
                                map_inst=self.map, grp_inst=self.grp)
        agent.follow_speed_limits(False)
        start = self.map.get_waypoint(self.vehicle.get_location())
        plan = []
        for p in points:
            end = self.map.get_waypoint(self.carla.Location(x=p["x"], y=-p["y"], z=p["z"]))
            plan += agent.trace_route(start, end)
            start = end
        if not plan:
            raise csb_agent.Failed("no route from the vehicle to the goal on CARLA's map")
        agent.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
        self.agent = agent

    def set_speed(self, speed_kmh):
        if self.agent is not None:
            self.agent.set_target_speed(speed_kmh)

    def step(self, dt=None):
        if self.agent is None or self.vehicle is None:
            return True
        if self.agent.done():
            return True
        self.vehicle.apply_control(self.agent.run_step())
        return False

    def hold(self):
        if self.vehicle is not None:
            self.vehicle.apply_control(self.carla.VehicleControl(throttle=0.0, brake=1.0))

    def alive(self):
        """The server answers. Probed only while no tick arrives."""
        if time.monotonic() - self.last_tick < TICK_STALE:
            return True
        try:
            self.client.get_server_version()
            return True
        except RuntimeError:
            return False

    def close(self):
        if self.tick_id is not None and self.world is not None:
            try:
                self.world.remove_on_tick(self.tick_id)
            except RuntimeError:
                pass
        self.vehicle = self.agent = self.world = None


# -- the agent -----------------------------------------------------------------------------
class CarlaBasicAgent(csb_agent.Agent):
    """BasicAgent on the scenario's vehicle. See the module docstring."""

    name = "carla_basic"

    def __init__(self, config=None):
        super().__init__(config)
        self.cfg = {**DEFAULTS, **{k: v for k, v in self.config.items() if k in DEFAULTS}}
        world = FakeWorld if self.cfg["world"] == "fake" else CarlaWorld
        self.world = world(self.cfg, self.log)
        self.world.on_tick = self._wake
        self.mode = "hold"  # hold | drive
        self.limit_kmh = None
        self.offset = 0.0  # CARLA actor origin ahead of the scenario's vehicle origin, m
        self._ctl = threading.RLock()
        self._tick = threading.Event()
        self._closing = threading.Event()
        self._last_pose = 0.0
        self._last_probe = time.monotonic()
        self._thread = threading.Thread(target=self._control, daemon=True)
        self._thread.start()

    # -- control thread --------------------------------------------------------------------
    def _wake(self):
        self._tick.set()

    def _control(self):
        while not self._closing.is_set():
            ticked = self._tick.wait(1.0)
            self._tick.clear()
            if not ticked:
                now = time.monotonic()
                if now - self._last_probe >= PROBE_PERIOD:
                    self._last_probe = now
                    if not self.world.alive():
                        self.log.error("CARLA stopped answering; exiting so this client "
                                       "does not outlive its server (status 3)")
                        os._exit(3)
                continue
            try:
                self._on_world_tick()
            except Exception as e:  # a vehicle destroyed between scenarios, a lost server
                self.log.warning(f"control step failed: {e}")
                with self._ctl:
                    self.mode = "hold"

    def _on_world_tick(self):
        with self._ctl:
            mode = self.mode
            if mode == "drive":
                if self.world.step():
                    self.mode = mode = "hold"
                    self.report(P.ARRIVED, detail="")
                    self.log.info("arrived")
            if mode == "hold":
                self.world.hold()
        now = time.monotonic()
        if now - self._last_pose >= POSE_PERIOD:
            self._last_pose = now
            pose = self._pose()
            if pose is not None:
                self.report(pose=pose)

    def _pose(self):
        at = self.world.vehicle_pose()
        if at is None:
            return None
        x, y, z, yaw = at
        return map_pose(x - self.offset * math.cos(yaw), y - self.offset * math.sin(yaw),
                        z, yaw)

    def _speed_kmh(self):
        return self.limit_kmh if self.limit_kmh is not None else \
            float(self.cfg["target_speed_kmh"])

    # -- hooks -----------------------------------------------------------------------------
    def on_reset(self, ep):
        with self._ctl:
            self.mode = "hold"
        end = time.monotonic() + float(self.cfg["find_timeout"])
        while not self.world.find_vehicle():
            if time.monotonic() > end:
                raise csb_agent.Failed(f"no vehicle with role_name "
                                       f"{self.cfg['role_name']!r} in CARLA after "
                                       f"{self.cfg['find_timeout']:.0f} s")
            time.sleep(0.5)
        with self._ctl:
            self.limit_kmh = None
            if ep.pose is not None:
                self.world.place(ep.pose)
                x, y, _, yaw = self.world.vehicle_pose()
                along = (x - ep.pose["x"]) * math.cos(yaw) + (y - ep.pose["y"]) * math.sin(yaw)
                # the same pose for the bridge's actor and the scenario's vehicle; a
                # vehicle not yet at it (a slow teleport) gives no offset
                self.offset = along if abs(along) < 3.0 else 0.0
            self.world.hold()
        self.report(P.IDLE, detail="")

    def on_goal(self, goal, waypoints):
        with self._ctl:
            self.world.plan(list(waypoints) + [goal], self._speed_kmh(),
                            {"lights": bool(self.cfg["ignore_traffic_lights"]),
                             "vehicles": bool(self.cfg["ignore_vehicles"])})
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        self.log.info(f"driving to ({goal['x']:.1f}, {goal['y']:.1f}) through "
                      f"{len(waypoints)} waypoint(s) at {self._speed_kmh():.0f} km/h")

    def on_speed_limit(self, mps):
        with self._ctl:
            self.limit_kmh = None if mps is None else float(mps) * 3.6
            self.world.set_speed(self._speed_kmh())

    def on_stop(self):
        with self._ctl:
            self.mode = "hold"
            self.world.hold()

    def on_clear_goal(self):
        self.on_stop()
        self.report(P.IDLE)

    def close(self):
        self._closing.set()
        self.world.close()


def main(argv=None):
    return csb_agent.main(CarlaBasicAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
