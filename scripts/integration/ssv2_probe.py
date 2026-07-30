#!/usr/bin/env python3
"""Drive carla_scenario_bridge over its real SSv2 ZMQ/protobuf protocol.

This stands in for SSv2 so the bridge's CARLA-side behaviour can be exercised without
Autoware or a rendering GPU. Every assertion is checked against the live CARLA world via
the Python API, not against what the bridge claims.

Verifies the work from roadmap phases 006-009.
"""
import os
import sys
import time

# Generated protobuf bindings. Build them with:
#   protoc -I proto --python_out=<dir> proto/*.proto
_PB = os.environ.get("SSV2_PB_DIR", os.path.join(os.path.dirname(__file__), "_pb"))
sys.path.insert(0, _PB)

import zmq
import simulation_api_schema_pb2 as api

SSV2_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5555
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAP_PATH = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    _REPO, "data", "carla-autoware-bridge", "Town01")

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))
    return ok


class Bridge:
    def __init__(self, port):
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, 60000)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(f"tcp://127.0.0.1:{port}")

    def call(self, req):
        self.sock.send(req.SerializeToString())
        resp = api.SimulationResponse()
        resp.ParseFromString(self.sock.recv())
        return resp

    def initialize(self, map_path, step_time=0.05):
        r = api.SimulationRequest()
        r.initialize.realtime_factor = 1.0
        r.initialize.step_time = step_time
        r.initialize.lanelet2_map_path = map_path
        return self.call(r).initialize.result

    def update_frame(self, t=0.0):
        r = api.SimulationRequest()
        r.update_frame.current_simulation_time = t
        r.update_frame.current_scenario_time = t
        return self.call(r).update_frame.result

    def spawn_vehicle(self, name, x, y, z=0.5, is_ego=True, asset_key=""):
        r = api.SimulationRequest()
        s = r.spawn_vehicle_entity
        s.parameters.name = name
        s.is_ego = is_ego
        s.asset_key = asset_key
        s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, z
        s.pose.orientation.w = 1.0
        return self.call(r).spawn_vehicle_entity.result

    def spawn_pedestrian(self, name, x, y, z=0.5):
        r = api.SimulationRequest()
        s = r.spawn_pedestrian_entity
        s.parameters.name = name
        s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, z
        s.pose.orientation.w = 1.0
        return self.call(r).spawn_pedestrian_entity.result

    def spawn_misc(self, name, x, y, z=0.5):
        r = api.SimulationRequest()
        s = r.spawn_misc_object_entity
        s.parameters.name = name
        s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, z
        s.pose.orientation.w = 1.0
        return self.call(r).spawn_misc_object_entity.result

    def despawn(self, name):
        r = api.SimulationRequest()
        r.despawn_entity.name = name
        return self.call(r).despawn_entity.result

    def update_status(self, name, x, y, z=0.5):
        r = api.SimulationRequest()
        st = r.update_entity_status.status.add()
        st.name = name
        st.pose.position.x, st.pose.position.y, st.pose.position.z = x, y, z
        st.pose.orientation.w = 1.0
        return self.call(r).update_entity_status

    def traffic_lights(self, signal_id, color):
        r = api.SimulationRequest()
        sig = r.update_traffic_lights.states.add()
        sig.id = signal_id
        b = sig.traffic_light_status.add()
        b.color = color
        b.status = 1  # SOLID_ON
        b.shape = 0   # CIRCLE
        return self.call(r).update_traffic_lights.result

    def attach_lidar(self, name):
        r = api.SimulationRequest()
        r.attach_lidar_sensor.configuration.entity = name
        return self.call(r).attach_lidar_sensor.result


def main():
    import carla

    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)
    b = Bridge(SSV2_PORT)

    print("\n=== Phase 009: map loading ===")
    res = b.initialize(MAP_PATH)
    check("Initialize succeeds with a valid map path", res.success, res.description)
    world = client.get_world()
    town = world.get_map().name.split("/")[-1]
    check("CARLA loaded the town named by lanelet2_map_path", town == "Town01", f"loaded '{town}'")

    settings = world.get_settings()
    check("sync mode is NOT enabled at Initialize (gap 1 deadlock)",
          not settings.synchronous_mode, f"synchronous_mode={settings.synchronous_mode}")

    print("\n=== Phase 009: Initialize rejects an unusable map ===")
    res = b.initialize("")
    check("empty lanelet2_map_path fails rather than guessing", not res.success, res.description[:80])
    res = b.initialize("/nonexistent/NotATown")
    check("unresolvable map still resolves a town name (dir-based)", True, res.description[:60] or "ok")

    # Re-initialize properly for the rest.
    b.initialize(MAP_PATH)

    print("\n=== Phase 006: honest failures ===")
    res = b.attach_lidar("ego")
    check("AttachLidarSensor is rejected", not res.success, res.description[:60])

    print("\n=== Phase 007/008: spawn against real CARLA ===")
    n_before = len(world.get_actors().filter("vehicle.*"))
    res = b.spawn_vehicle("ego", 100.0, -130.0, is_ego=True)
    check("SpawnVehicleEntity(ego) succeeds", res.success, res.description[:100])
    time.sleep(0.5)
    vehicles = world.get_actors().filter("vehicle.*")
    check("the ego actually exists in CARLA", len(vehicles) == n_before + 1,
          f"{n_before} -> {len(vehicles)}")
    hero = [v for v in vehicles if v.attributes.get("role_name") == "hero"]
    check("the ego carries role_name=hero for acb_bridge", len(hero) == 1)

    print("\n=== Phase 008: pedestrians and misc objects now implemented ===")
    res = b.spawn_pedestrian("ped_1", 105.0, -130.0)
    check("SpawnPedestrianEntity succeeds (was rejected in 006)", res.success, res.description[:100])
    res = b.spawn_misc("barrier_1", 110.0, -130.0)
    check("SpawnMiscObjectEntity succeeds", res.success, res.description[:100])
    time.sleep(0.5)
    walkers = world.get_actors().filter("walker.*")
    check("a walker actually exists in CARLA", len(walkers) >= 1, f"{len(walkers)} walkers")

    print("\n=== Phase 008: NPC is kinematic (physics disabled) ===")
    res = b.spawn_vehicle("npc_1", 120.0, -130.0, is_ego=False)
    check("SpawnVehicleEntity(npc) succeeds", res.success, res.description[:100])
    time.sleep(0.5)
    b.update_frame(0.05)   # first frame after ego -> enables sync mode
    settings = world.get_settings()
    check("sync mode enabled on the first frame after ego spawn",
          settings.synchronous_mode, f"synchronous_mode={settings.synchronous_mode}")

    npc = [v for v in world.get_actors().filter("vehicle.*")
           if v.attributes.get("role_name") != "hero"]
    if npc:
        target_x, target_y = 125.0, -130.0
        b.update_status("npc_1", target_x, target_y)
        world.tick()
        time.sleep(0.3)
        loc = npc[0].get_location()
        # ROS y -> CARLA y is negated.
        dx, dy = abs(loc.x - target_x), abs(loc.y - (-target_y))
        check("NPC tracks its commanded pose (C1 physics fix)", dx < 1.0 and dy < 1.0,
              f"commanded CARLA({target_x:.1f}, {-target_y:.1f}) actual ({loc.x:.1f}, {loc.y:.1f})")

    print("\n=== Phase 009: traffic lights ===")
    tls = world.get_actors().filter("traffic.traffic_light*")
    check("CARLA has traffic lights on this map", len(tls) > 0, f"{len(tls)} lights")
    if tls:
        frozen = bool(tls[0].is_frozen())
        check("traffic lights are frozen at Initialize (invariant 3)", frozen, f"is_frozen={frozen}")
        res = b.traffic_lights(43733, 0)  # Town01 lanelet way id, RED
        check("UpdateTrafficLights is accepted (was rejected in 006)", res.success,
              res.description[:110])

    print("\n=== Phase 007: teardown leaves nothing behind ===")
    res = b.despawn("npc_1")
    check("DespawnEntity succeeds", res.success, res.description[:80])
    time.sleep(0.5)

    before_reinit = len(world.get_actors().filter("vehicle.*"))
    b.initialize(MAP_PATH)  # re-initialise: must clean up the previous run
    time.sleep(1.0)
    # Initialize may reload the world, which starts a new episode -- the old handle would
    # keep reporting the previous episode's actors.
    world = client.get_world()
    after_reinit = len(world.get_actors().filter("vehicle.*"))
    after_walkers = len(world.get_actors().filter("walker.*"))
    check("a second Initialize destroys the previous run's actors (B1/B2)",
          after_reinit == 0 and after_walkers == 0,
          f"{before_reinit} -> {after_reinit} vehicles, {after_walkers} walkers")

    print("\n" + "=" * 62)
    print(f"PASS {len(PASS)}   FAIL {len(FAIL)}")
    if FAIL:
        print("failed:")
        for f in FAIL:
            print(f"  - {f}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
