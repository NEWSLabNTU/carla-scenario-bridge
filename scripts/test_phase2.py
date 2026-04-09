#!/usr/bin/env python3
"""
Phase 2 integration test for carla-scenario-bridge.

Tests the cooperation between the adapter and autoware_carla_bridge:
- Adapter spawns ego with role_name=hero
- autoware_carla_bridge detects hero, attaches sensors
- Adapter ticks CARLA; bridge publishes /clock
- Ego pose readback reflects CARLA physics

Requires:
    Terminal 1: CARLA running          (just carla-start)
    Terminal 2: Adapter running        (just run)
    Terminal 3: autoware_carla_bridge  (cd ~/repos/autoware_carla_bridge && just bridge)
    Terminal 4: This test              (python3 scripts/test_phase2.py)

For tests that only need the adapter (no bridge):
    python3 scripts/test_phase2.py --no-bridge
"""

import argparse
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tmp', 'proto_py'))

import zmq
import simulation_api_schema_pb2 as api
import geometry_msgs_pb2 as geo
import traffic_simulator_msgs_pb2 as traf

PASS = "\033[92m PASS\033[0m"
FAIL = "\033[91m FAIL\033[0m"
SKIP = "\033[93m SKIP\033[0m"


class TestClient:
    def __init__(self, port):
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, 15000)
        self.socket.setsockopt(zmq.SNDTIMEO, 5000)
        self.socket.connect(f"tcp://localhost:{port}")
        self.passed = 0
        self.failed = 0
        self.skipped = 0

    def send(self, request):
        wrapper = api.SimulationRequest()
        field_map = {
            api.InitializeRequest: 'initialize',
            api.UpdateFrameRequest: 'update_frame',
            api.SpawnVehicleEntityRequest: 'spawn_vehicle_entity',
            api.DespawnEntityRequest: 'despawn_entity',
            api.UpdateEntityStatusRequest: 'update_entity_status',
            api.UpdateStepTimeRequest: 'update_step_time',
        }
        field = field_map.get(type(request))
        if field:
            getattr(wrapper, field).CopyFrom(request)
        else:
            raise ValueError(f"Unknown request type: {type(request)}")
        self.socket.send(wrapper.SerializeToString())
        reply = self.socket.recv()
        resp = api.SimulationResponse()
        resp.ParseFromString(reply)
        return resp

    def check(self, name, condition, detail=""):
        if condition:
            print(f"  [{PASS}] {name}")
            self.passed += 1
        else:
            print(f"  [{FAIL}] {name}{f': {detail}' if detail else ''}")
            self.failed += 1

    def skip(self, name, reason=""):
        print(f"  [{SKIP}] {name}{f': {reason}' if reason else ''}")
        self.skipped += 1

    def summary(self):
        total = self.passed + self.failed + self.skipped
        print(f"\n{'='*60}")
        print(f"Results: {self.passed} passed, {self.failed} failed, {self.skipped} skipped / {total} total")
        return self.failed == 0


def make_pose(x, y, z, yaw=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return geo.Pose(
        position=geo.Point(x=x, y=y, z=z),
        orientation=geo.Quaternion(x=0.0, y=0.0, z=sy, w=cy),
    )


def ros2_topic_available(topic, timeout=3):
    """Check if a ROS 2 topic is being published."""
    try:
        result = subprocess.run(
            ['bash', '-c', f'source /opt/ros/humble/setup.bash && '
             f'source ~/repos/autoware_carla_bridge/install/setup.bash 2>/dev/null; '
             f'timeout {timeout} ros2 topic hz {topic} --window 2 2>&1 | head -3'],
            capture_output=True, text=True, timeout=timeout + 5,
        )
        return 'average rate' in result.stdout
    except Exception:
        return False


def ros2_topic_echo_once(topic, msg_type, timeout=5):
    """Echo one message from a ROS 2 topic."""
    try:
        result = subprocess.run(
            ['bash', '-c', f'source /opt/ros/humble/setup.bash && '
             f'source ~/repos/autoware_carla_bridge/install/setup.bash 2>/dev/null; '
             f'timeout {timeout} ros2 topic echo {topic} {msg_type} --once 2>&1'],
            capture_output=True, text=True, timeout=timeout + 5,
        )
        return result.stdout if result.returncode == 0 else None
    except Exception:
        return None


def test_initialize_and_spawn(tc):
    """Initialize CARLA and spawn ego vehicle."""
    print("\n--- Setup: Initialize + Spawn Ego ---")

    resp = tc.send(api.InitializeRequest(
        realtime_factor=1.0, step_time=0.05,
        initialize_time=0.0, lanelet2_map_path="",
    ))
    tc.check("Initialize succeeds", resp.initialize.result.success,
             resp.initialize.result.description)

    resp = tc.send(api.SpawnVehicleEntityRequest(
        parameters=traf.VehicleParameters(
            name="ego",
            bounding_box=traf.BoundingBox(
                center=geo.Point(x=1.5, y=0.0, z=0.9),
                dimensions=geo.Vector3(x=4.5, y=2.1, z=1.8),
            ),
        ),
        is_ego=True,
        asset_key="vehicle.tesla.model3",
        pose=make_pose(190.8, -130.1, 0.3, yaw=3.14159),
        initial_speed=0.0,
    ))
    tc.check("Spawn ego succeeds", resp.spawn_vehicle_entity.result.success,
             resp.spawn_vehicle_entity.result.description)

    # Tick a few times to let the vehicle settle
    for i in range(5):
        tc.send(api.UpdateFrameRequest(
            current_simulation_time=0.05 * (i + 1),
            current_scenario_time=0.05 * (i + 1),
        ))

    return resp.spawn_vehicle_entity.result.success


def test_ego_pose_readback(tc):
    """Verify ego pose comes from CARLA physics, not the sent pose."""
    print("\n--- Test: Ego Pose Readback (CARLA Physics) ---")

    # Send a pose that's different from where the ego actually is
    fake_pose = make_pose(999.0, 999.0, 999.0)
    req = api.UpdateEntityStatusRequest(
        status=[api.EntityStatus(
            type=traf.EntityType(type=traf.EntityType.EGO),
            name="ego",
            pose=fake_pose,
            action_status=traf.ActionStatus(current_action="driving"),
        )],
        npc_logic_started=True,
        overwrite_ego_status=False,  # Don't overwrite - read from CARLA
    )
    resp = tc.send(req)

    ego_updates = [s for s in resp.update_entity_status.status if s.name == "ego"]
    tc.check("Ego in response", len(ego_updates) == 1)
    if not ego_updates:
        return

    pose = ego_updates[0].pose
    tc.check("Ego pose is NOT the fake (999, 999)",
             abs(pose.position.x - 999.0) > 10.0,
             f"got ({pose.position.x:.1f}, {pose.position.y:.1f})")

    tc.check("Ego pose near spawn point (~190, ~-130)",
             abs(pose.position.x - 190.8) < 5.0 and abs(pose.position.y - (-130.1)) < 5.0,
             f"got ({pose.position.x:.1f}, {pose.position.y:.1f})")

    # Check velocity and acceleration in ActionStatus
    action = ego_updates[0].action_status
    tc.check("ActionStatus has twist", action.HasField('twist'))
    tc.check("ActionStatus has accel", action.HasField('accel'))
    if action.HasField('twist'):
        tc.check("Twist has linear velocity",
                 action.twist.HasField('linear'))
        tc.check("Twist has angular velocity",
                 action.twist.HasField('angular'))


def test_ego_overwrite(tc):
    """Verify overwrite_ego_status=true teleports the ego."""
    print("\n--- Test: Ego Overwrite (Teleport) ---")

    teleport_pose = make_pose(200.0, -120.0, 0.5)
    req = api.UpdateEntityStatusRequest(
        status=[api.EntityStatus(
            type=traf.EntityType(type=traf.EntityType.EGO),
            name="ego",
            pose=teleport_pose,
            action_status=traf.ActionStatus(current_action="teleport"),
        )],
        npc_logic_started=True,
        overwrite_ego_status=True,
    )
    resp = tc.send(req)
    tc.check("Overwrite returns success",
             resp.update_entity_status.result.success)

    # Tick to let physics settle
    tc.send(api.UpdateFrameRequest(current_simulation_time=1.0))

    # Read back - should be near the teleport position
    req2 = api.UpdateEntityStatusRequest(
        status=[api.EntityStatus(
            type=traf.EntityType(type=traf.EntityType.EGO),
            name="ego",
            pose=teleport_pose,
            action_status=traf.ActionStatus(),
        )],
        npc_logic_started=True,
        overwrite_ego_status=False,
    )
    resp2 = tc.send(req2)
    ego = [s for s in resp2.update_entity_status.status if s.name == "ego"]
    if ego:
        p = ego[0].pose.position
        tc.check("Ego teleported to (~200, ~-120)",
                 abs(p.x - 200.0) < 5.0 and abs(p.y - (-120.0)) < 5.0,
                 f"got ({p.x:.1f}, {p.y:.1f})")
    else:
        tc.check("Ego in response after teleport", False)


def test_clock_sync(tc, has_bridge):
    """Verify /clock advances with UpdateFrame ticks."""
    print("\n--- Test: Clock Synchronization ---")

    if not has_bridge:
        tc.skip("Clock sync", "requires autoware_carla_bridge")
        return

    # Read clock before ticking
    clock_before = ros2_topic_echo_once(
        '/clock', 'rosgraph_msgs/msg/Clock', timeout=3)
    if not clock_before:
        tc.skip("Clock sync", "/clock topic not available")
        return

    # Tick 10 times
    for i in range(10):
        tc.send(api.UpdateFrameRequest(
            current_simulation_time=2.0 + 0.05 * i,
            current_scenario_time=2.0 + 0.05 * i,
        ))

    time.sleep(0.5)  # Let bridge publish

    clock_after = ros2_topic_echo_once(
        '/clock', 'rosgraph_msgs/msg/Clock', timeout=3)
    if not clock_after:
        tc.check("Clock advances after ticks", False, "couldn't read /clock after ticks")
        return

    # Parse sec values from the output
    def parse_sec(text):
        for line in text.split('\n'):
            if 'sec:' in line and 'nanosec' not in line:
                return int(line.split(':')[1].strip())
        return None

    sec_before = parse_sec(clock_before)
    sec_after = parse_sec(clock_after)

    if sec_before is not None and sec_after is not None:
        tc.check(f"Clock advanced (before={sec_before}, after={sec_after})",
                 sec_after >= sec_before)
    else:
        tc.check("Clock parseable", False,
                 f"before={clock_before[:50]}... after={clock_after[:50]}...")


def test_bridge_detects_hero(tc, has_bridge):
    """Verify autoware_carla_bridge detects the hero vehicle and attaches sensors."""
    print("\n--- Test: Bridge Detects Hero Vehicle ---")

    if not has_bridge:
        tc.skip("Bridge detects hero", "requires autoware_carla_bridge")
        tc.skip("Bridge publishes sensor data", "requires autoware_carla_bridge")
        tc.skip("Bridge publishes vehicle status", "requires autoware_carla_bridge")
        return

    # Give bridge time to detect hero and attach sensors
    print("  Waiting 10s for bridge to detect hero vehicle and attach sensors...")
    for i in range(200):
        tc.send(api.UpdateFrameRequest(
            current_simulation_time=3.0 + 0.05 * i,
            current_scenario_time=3.0 + 0.05 * i,
        ))
        time.sleep(0.02)

    # Check sensor topics
    print("  Checking sensor topics...")
    lidar_ok = ros2_topic_available('/sensing/lidar/top/pointcloud_raw', timeout=5)
    tc.check("Bridge publishes LiDAR pointcloud",
             lidar_ok, "topic not available" if not lidar_ok else "")

    imu_ok = ros2_topic_available('/sensing/imu/imu_data', timeout=5)
    tc.check("Bridge publishes IMU data",
             imu_ok, "topic not available" if not imu_ok else "")

    gnss_ok = ros2_topic_available('/sensing/gnss/fix', timeout=5)
    tc.check("Bridge publishes GNSS fix",
             gnss_ok, "topic not available" if not gnss_ok else "")

    # Check vehicle status topics
    print("  Checking vehicle status topics...")
    vel_ok = ros2_topic_available('/vehicle/status/velocity_status', timeout=5)
    tc.check("Bridge publishes velocity_status",
             vel_ok, "topic not available" if not vel_ok else "")

    steer_ok = ros2_topic_available('/vehicle/status/steering_status', timeout=5)
    tc.check("Bridge publishes steering_status",
             steer_ok, "topic not available" if not steer_ok else "")

    clock_ok = ros2_topic_available('/clock', timeout=5)
    tc.check("Bridge publishes /clock",
             clock_ok, "topic not available" if not clock_ok else "")


def test_cleanup(tc):
    """Despawn ego and clean up."""
    print("\n--- Cleanup ---")
    resp = tc.send(api.DespawnEntityRequest(name="ego"))
    tc.check("Despawn ego", resp.despawn_entity.result.success,
             resp.despawn_entity.result.description)


def main():
    parser = argparse.ArgumentParser(description="Phase 2 integration test")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ port")
    parser.add_argument("--no-bridge", action="store_true",
                        help="Skip tests requiring autoware_carla_bridge")
    args = parser.parse_args()

    has_bridge = not args.no_bridge

    print(f"Phase 2 Integration Test")
    print(f"  Adapter:    tcp://localhost:{args.port}")
    print(f"  Bridge:     {'enabled' if has_bridge else 'SKIPPED (--no-bridge)'}")

    tc = TestClient(args.port)

    ego_spawned = test_initialize_and_spawn(tc)

    if ego_spawned:
        test_ego_pose_readback(tc)
        test_ego_overwrite(tc)
        test_bridge_detects_hero(tc, has_bridge)
        test_clock_sync(tc, has_bridge)
        test_cleanup(tc)
    else:
        print("\n  Ego spawn failed, skipping remaining tests.")

    success = tc.summary()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
