#!/usr/bin/env python3
"""Measure the entity-origin offset between SSv2 and CARLA through the bridge.

Sends Initialize + SpawnVehicleEntity (rear-axle bbox, Center x=1.5) at a known pose, then
reads the spawned CARLA actor's transform, bounding box and rear-wheel positions with the
CARLA Python API and reports the along-heading delta between the commanded SSv2 pose and
what CARLA holds. Also spawns an is_ego vehicle to measure the readback side.

Roadmap 014, pose reference point measurement. Needs tmp/proto_py (protoc -I proto --python_out=tmp/proto_py proto/*.proto),
pyzmq, protobuf and the CARLA Python module; run with PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python if protoc is older
than the protobuf runtime. Usage: pose_offset_probe.py --map /any/path/Town01  (only the final directory is used).
"""
import argparse, math, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tmp", "proto_py"))
import zmq, carla
import simulation_api_schema_pb2 as api
import traffic_simulator_msgs_pb2 as traf

def send(sock, req):
    w = api.SimulationRequest()
    for f in ("initialize", "spawn_vehicle_entity", "update_entity_status", "despawn_entity", "update_frame"):
        if type(req).__name__.lower().startswith(f.replace("_", "")[:8]):
            pass
    if isinstance(req, api.InitializeRequest): w.initialize.CopyFrom(req)
    elif isinstance(req, api.SpawnVehicleEntityRequest): w.spawn_vehicle_entity.CopyFrom(req)
    elif isinstance(req, api.UpdateEntityStatusRequest): w.update_entity_status.CopyFrom(req)
    elif isinstance(req, api.DespawnEntityRequest): w.despawn_entity.CopyFrom(req)
    elif isinstance(req, api.UpdateFrameRequest): w.update_frame.CopyFrom(req)
    else: raise TypeError(type(req))
    sock.send(w.SerializeToString())
    r = api.SimulationResponse(); r.ParseFromString(sock.recv()); return r

def yaw_quat(h):
    return (0.0, 0.0, math.sin(h / 2), math.cos(h / 2))

def vehicle_params(name, cx, length, width, height, ego=False):
    p = traf.VehicleParameters()
    p.name = name
    p.bounding_box.center.x = cx; p.bounding_box.center.y = 0.0; p.bounding_box.center.z = height / 2
    p.bounding_box.dimensions.x = length; p.bounding_box.dimensions.y = width; p.bounding_box.dimensions.z = height
    p.performance.max_speed = 50; p.performance.max_acceleration = 5; p.performance.max_deceleration = 8
    p.axles.front_axle.max_steering = 0.5; p.axles.front_axle.wheel_diameter = 0.6; p.axles.front_axle.track_width = 1.6; p.axles.front_axle.position_x = 3.0; p.axles.front_axle.position_z = 0.3
    p.axles.rear_axle.wheel_diameter = 0.6; p.axles.rear_axle.track_width = 1.6; p.axles.rear_axle.position_x = 0.0; p.axles.rear_axle.position_z = 0.3
    return p

def spawn(sock, name, x, y, z, h, cx, ego):
    req = api.SpawnVehicleEntityRequest()
    req.parameters.CopyFrom(vehicle_params(name, cx, 4.5, 2.1, 1.8, ego))
    req.pose.position.x = x; req.pose.position.y = y; req.pose.position.z = z
    qx, qy, qz, qw = yaw_quat(h)
    req.pose.orientation.x = qx; req.pose.orientation.y = qy; req.pose.orientation.z = qz; req.pose.orientation.w = qw
    req.is_ego = ego
    req.initial_speed = 0.0
    req.asset_key = "sample_vehicle"
    r = send(sock, req)
    ok = r.spawn_vehicle_entity.result.success
    print(f"spawn {name} ego={ego}: success={ok} {r.spawn_vehicle_entity.result.description}")
    return ok

def find_actor(world, role):
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == role:
            return a
    return None

def report(tag, actor, x_ros, y_ros, h):
    t = actor.get_transform()
    # CARLA left-handed: y_carla = -y_ros, yaw_carla = -yaw_ros(deg)
    cx, cy = t.location.x, t.location.y
    print(f"  {tag}: commanded ROS ({x_ros:.2f},{y_ros:.2f}) h={math.degrees(h):.1f}deg -> CARLA actor ({cx:.3f},{cy:.3f},{t.location.z:.3f}) yaw={t.rotation.yaw:.1f}")
    # commanded in CARLA frame
    ex, ey = x_ros, -y_ros
    dx, dy = cx - ex, cy - ey
    fx, fy = math.cos(math.radians(t.rotation.yaw)), math.sin(math.radians(t.rotation.yaw))
    along = dx * fx + dy * fy
    across = -dx * fy + dy * fx
    print(f"  {tag}: delta along heading = {along:+.3f} m, across = {across:+.3f} m, dz = {t.location.z - 0.0:+.3f}")
    bb = actor.bounding_box
    print(f"  {tag}: CARLA bbox center=({bb.location.x:.3f},{bb.location.y:.3f},{bb.location.z:.3f}) extent=({bb.extent.x:.3f},{bb.extent.y:.3f},{bb.extent.z:.3f}) -> length {2*bb.extent.x:.2f} m")
    try:
        pc = actor.get_physics_control()
        ws = pc.wheels
        # wheel positions are world coords in cm
        rear = [(w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0) for w in ws[2:4]]
        front = [(w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0) for w in ws[0:2]]
        rx = sum(p[0] for p in rear) / 2; ry = sum(p[1] for p in rear) / 2
        ffx = sum(p[0] for p in front) / 2; ffy = sum(p[1] for p in front) / 2
        rear_along = (rx - cx) * fx + (ry - cy) * fy
        front_along = (ffx - cx) * fx + (ffy - cy) * fy
        print(f"  {tag}: rear axle is {rear_along:+.3f} m along heading from actor origin; front axle {front_along:+.3f} m; wheelbase {front_along - rear_along:.3f} m")
    except Exception as e:
        print(f"  {tag}: wheel physics unavailable: {e}")
    return along

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="lanelet2 map directory (final dir = town)")
    ap.add_argument("--zmq", type=int, default=5555)
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--x", type=float, default=320.0); ap.add_argument("--y", type=float, default=-129.8)
    ap.add_argument("--h", type=float, default=math.pi)
    ap.add_argument("--keep", action="store_true")
    a = ap.parse_args()

    ctx = zmq.Context(); sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 120000); sock.setsockopt(zmq.SNDTIMEO, 5000)
    sock.connect(f"tcp://localhost:{a.zmq}")

    r = send(sock, api.InitializeRequest(realtime_factor=1.0, step_time=0.05, initialize_time=0.0, lanelet2_map_path=a.map))
    print("initialize:", r.initialize.result.success, r.initialize.result.description)
    if not r.initialize.result.success: sys.exit(1)

    client = carla.Client("localhost", a.carla_port); client.set_timeout(30.0)
    world = client.get_world()
    print("CARLA map:", world.get_map().name)

    # NPC 50 m ahead (heading pi -> -x), same lane
    npc_x = a.x - 30.0
    ok_npc = spawn(sock, "npc_probe", npc_x, a.y, 0.0, a.h, 1.5, ego=False)
    ok_ego = spawn(sock, "ego_probe", a.x, a.y, 0.0, a.h, 1.5, ego=True)
    time.sleep(1.0)

    results = {}
    if ok_npc:
        npc = find_actor(world, "autopilot")
        if npc is None:
            for act in world.get_actors().filter("vehicle.*"): print("   actor", act.id, act.type_id, act.attributes.get("role_name"))
        else:
            print("NPC (kinematic, set_transform):")
            results["npc"] = report("npc", npc, npc_x, a.y, a.h)
            # One UpdateEntityStatus teleport to the same pose, then re-read (post-spawn path)
            u = api.UpdateEntityStatusRequest()
            st = u.status.add(); st.name = "npc_probe"; st.type.type = traf.EntityType.VEHICLE
            st.pose.position.x = npc_x; st.pose.position.y = a.y; st.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_quat(a.h); st.pose.orientation.z = qz; st.pose.orientation.w = qw
            u.npc_logic_started = True; u.overwrite_ego_status = False
            rr = send(sock, u); print("update_entity_status:", rr.update_entity_status.result.success, rr.update_entity_status.result.description)
            time.sleep(0.5)
            print("NPC after UpdateEntityStatus teleport:")
            results["npc_after_update"] = report("npc2", npc, npc_x, a.y, a.h)
            for s in rr.update_entity_status.status:
                print(f"  echoed {s.name}: pose ({s.pose.position.x:.3f},{s.pose.position.y:.3f},{s.pose.position.z:.3f})")
    if ok_ego:
        ego = find_actor(world, "hero") or find_actor(world, "ego_probe")
        if ego is not None:
            print("EGO (PhysX):")
            results["ego"] = report("ego", ego, a.x, a.y, a.h)
            # readback via UpdateEntityStatus with ego status not overwritten
            u = api.UpdateEntityStatusRequest()
            st = u.status.add(); st.name = "ego_probe"; st.type.type = traf.EntityType.EGO
            st.pose.position.x = a.x; st.pose.position.y = a.y
            qx, qy, qz, qw = yaw_quat(a.h); st.pose.orientation.z = qz; st.pose.orientation.w = qw
            u.npc_logic_started = True; u.overwrite_ego_status = False
            rr = send(sock, u)
            for s in rr.update_entity_status.status:
                if s.name == "ego_probe":
                    t = ego.get_transform()
                    print(f"  ego readback to SSv2: ({s.pose.position.x:.3f},{s.pose.position.y:.3f},{s.pose.position.z:.3f}); CARLA actor ({t.location.x:.3f},{-t.location.y:.3f} ROS-y,{t.location.z:.3f})")
                    # ROS frame: heading a.h, so along = d . (cos h, sin h)
                    rdx, rdy = s.pose.position.x - a.x, s.pose.position.y - a.y
                    r_along = rdx * math.cos(a.h) + rdy * math.sin(a.h)
                    r_across = -rdx * math.sin(a.h) + rdy * math.cos(a.h)
                    print(f"  ego readback - commanded: along heading = {r_along:+.3f} m, across = {r_across:+.3f} m (0 = readback is the SSv2 entity origin)")
                    results["ego_readback_minus_commanded"] = r_along

    print("\nSUMMARY (positive = CARLA actor origin is ahead of the SSv2 pose along heading)")
    for k, v in results.items(): print(f"  {k}: {v:+.3f} m")

    if not a.keep:
        for n in ("npc_probe", "ego_probe"):
            rr = send(sock, api.DespawnEntityRequest(name=n)); print(f"despawn {n}:", rr.despawn_entity.result.success)

if __name__ == "__main__":
    main()
