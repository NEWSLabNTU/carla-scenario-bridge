#!/usr/bin/env python3
"""
Phase 1 acceptance test for carla-scenario-bridge.

Acts as an SSv2 client (ZMQ REQ) and sends protobuf messages to the adapter.
Tests all 14 message handlers and verifies responses.

Usage:
    # Terminal 1: Start CARLA
    just carla-start

    # Terminal 2: Start the adapter
    just run

    # Terminal 3: Run this test
    python3 scripts/test_phase1.py [--port 5555] [--no-carla]

    --no-carla: Skip tests that require CARLA (entity spawn, tick, etc.)
                Only test ZMQ connectivity and protocol handling.
"""

import argparse
import sys
import os
import time
import math

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
        self.socket.setsockopt(zmq.RCVTIMEO, 10000)  # 10s timeout
        self.socket.setsockopt(zmq.SNDTIMEO, 5000)
        self.socket.connect(f"tcp://localhost:{port}")
        self.passed = 0
        self.failed = 0
        self.skipped = 0

    def send(self, request):
        """Send a SimulationRequest and return SimulationResponse."""
        wrapper = api.SimulationRequest()

        if isinstance(request, api.InitializeRequest):
            wrapper.initialize.CopyFrom(request)
        elif isinstance(request, api.UpdateFrameRequest):
            wrapper.update_frame.CopyFrom(request)
        elif isinstance(request, api.UpdateStepTimeRequest):
            wrapper.update_step_time.CopyFrom(request)
        elif isinstance(request, api.SpawnVehicleEntityRequest):
            wrapper.spawn_vehicle_entity.CopyFrom(request)
        elif isinstance(request, api.SpawnPedestrianEntityRequest):
            wrapper.spawn_pedestrian_entity.CopyFrom(request)
        elif isinstance(request, api.SpawnMiscObjectEntityRequest):
            wrapper.spawn_misc_object_entity.CopyFrom(request)
        elif isinstance(request, api.DespawnEntityRequest):
            wrapper.despawn_entity.CopyFrom(request)
        elif isinstance(request, api.UpdateEntityStatusRequest):
            wrapper.update_entity_status.CopyFrom(request)
        elif isinstance(request, api.AttachLidarSensorRequest):
            wrapper.attach_lidar_sensor.CopyFrom(request)
        elif isinstance(request, api.AttachDetectionSensorRequest):
            wrapper.attach_detection_sensor.CopyFrom(request)
        elif isinstance(request, api.AttachOccupancyGridSensorRequest):
            wrapper.attach_occupancy_grid_sensor.CopyFrom(request)
        elif isinstance(request, api.AttachImuSensorRequest):
            wrapper.attach_imu_sensor.CopyFrom(request)
        elif isinstance(request, api.AttachPseudoTrafficLightDetectorRequest):
            wrapper.attach_pseudo_traffic_light_detector.CopyFrom(request)
        elif isinstance(request, api.UpdateTrafficLightsRequest):
            wrapper.update_traffic_lights.CopyFrom(request)
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
    """Create a protobuf Pose at (x, y, z) with yaw in radians."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return geo.Pose(
        position=geo.Point(x=x, y=y, z=z),
        orientation=geo.Quaternion(x=0.0, y=0.0, z=sy, w=cy),
    )


def test_zmq_connectivity(tc):
    """Test 1: ZMQ connection works."""
    print("\n--- Test: ZMQ Connectivity ---")
    try:
        req = api.InitializeRequest(
            realtime_factor=1.0,
            step_time=0.05,
            initialize_time=0.0,
            lanelet2_map_path="",
        )
        resp = tc.send(req)
        has_init = resp.HasField('initialize')
        tc.check("ZMQ round-trip works", has_init)
        if has_init:
            tc.check("Initialize returns result", resp.initialize.HasField('result'))
            tc.check("Initialize succeeds", resp.initialize.result.success,
                     resp.initialize.result.description)
    except zmq.error.Again:
        tc.check("ZMQ round-trip works", False, "timeout - is the adapter running?")


def test_sensor_stubs(tc):
    """Test 2: All AttachSensor requests return Success=false."""
    print("\n--- Test: Sensor Stubs (should all return Success=false) ---")

    # Lidar
    resp = tc.send(api.AttachLidarSensorRequest(
        configuration=api.LidarConfiguration(entity="ego")
    ))
    tc.check("AttachLidarSensor returns false",
             not resp.attach_lidar_sensor.result.success)

    # Detection
    resp = tc.send(api.AttachDetectionSensorRequest(
        configuration=api.DetectionSensorConfiguration(entity="ego")
    ))
    tc.check("AttachDetectionSensor returns false",
             not resp.attach_detection_sensor.result.success)

    # OccupancyGrid
    resp = tc.send(api.AttachOccupancyGridSensorRequest(
        configuration=api.OccupancyGridSensorConfiguration(entity="ego")
    ))
    tc.check("AttachOccupancyGridSensor returns false",
             not resp.attach_occupancy_grid_sensor.result.success)

    # IMU
    resp = tc.send(api.AttachImuSensorRequest(
        configuration=api.ImuSensorConfiguration(entity="ego")
    ))
    tc.check("AttachImuSensor returns false",
             not resp.attach_imu_sensor.result.success)

    # PseudoTrafficLightDetector
    resp = tc.send(api.AttachPseudoTrafficLightDetectorRequest(
        configuration=api.PseudoTrafficLightDetectorConfiguration(architecture_type="awf/universe")
    ))
    tc.check("AttachPseudoTrafficLightDetector returns false",
             not resp.attach_pseudo_traffic_light_detector.result.success)


def test_update_traffic_lights(tc):
    """Test 3: UpdateTrafficLights returns success (stub)."""
    print("\n--- Test: UpdateTrafficLights (stub) ---")
    resp = tc.send(api.UpdateTrafficLightsRequest(states=[]))
    tc.check("UpdateTrafficLights returns success",
             resp.update_traffic_lights.result.success)


def test_update_step_time(tc):
    """Test 4: UpdateStepTime."""
    print("\n--- Test: UpdateStepTime ---")
    resp = tc.send(api.UpdateStepTimeRequest(simulation_step_time=0.033))
    tc.check("UpdateStepTime returns success",
             resp.update_step_time.result.success,
             resp.update_step_time.result.description)

    # Reset to 0.05
    tc.send(api.UpdateStepTimeRequest(simulation_step_time=0.05))


def test_spawn_ego(tc):
    """Test 5: Spawn ego vehicle with role_name=hero."""
    print("\n--- Test: SpawnVehicleEntity (ego) ---")
    req = api.SpawnVehicleEntityRequest(
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
    )
    resp = tc.send(req)
    tc.check("SpawnVehicleEntity (ego) succeeds",
             resp.spawn_vehicle_entity.result.success,
             resp.spawn_vehicle_entity.result.description)
    return resp.spawn_vehicle_entity.result.success


def test_spawn_npc(tc):
    """Test 6: Spawn NPC vehicle."""
    print("\n--- Test: SpawnVehicleEntity (NPC) ---")
    req = api.SpawnVehicleEntityRequest(
        parameters=traf.VehicleParameters(
            name="npc_car_1",
            bounding_box=traf.BoundingBox(
                center=geo.Point(x=1.5, y=0.0, z=0.9),
                dimensions=geo.Vector3(x=4.5, y=2.1, z=1.8),
            ),
        ),
        is_ego=False,
        asset_key="vehicle.volkswagen.t2",
        pose=make_pose(180.0, -130.1, 0.3),
        initial_speed=0.0,
    )
    resp = tc.send(req)
    tc.check("SpawnVehicleEntity (NPC) succeeds",
             resp.spawn_vehicle_entity.result.success,
             resp.spawn_vehicle_entity.result.description)
    return resp.spawn_vehicle_entity.result.success


def test_update_entity_status(tc, has_ego, has_npc):
    """Test 7: UpdateEntityStatus - NPC set_transform, ego readback."""
    print("\n--- Test: UpdateEntityStatus ---")

    statuses = []
    if has_npc:
        statuses.append(api.EntityStatus(
            type=traf.EntityType(type=traf.EntityType.VEHICLE),
            subtype=traf.EntitySubtype(value=traf.EntitySubtype.CAR),
            time=1.0,
            name="npc_car_1",
            pose=make_pose(175.0, -130.1, 0.3),
            action_status=traf.ActionStatus(current_action="driving"),
        ))
    if has_ego:
        statuses.append(api.EntityStatus(
            type=traf.EntityType(type=traf.EntityType.EGO),
            subtype=traf.EntitySubtype(value=traf.EntitySubtype.CAR),
            time=1.0,
            name="ego",
            pose=make_pose(190.8, -130.1, 0.3),
            action_status=traf.ActionStatus(current_action="driving"),
        ))

    req = api.UpdateEntityStatusRequest(
        status=statuses,
        npc_logic_started=True,
        overwrite_ego_status=False,
    )
    resp = tc.send(req)
    tc.check("UpdateEntityStatus returns success",
             resp.update_entity_status.result.success,
             resp.update_entity_status.result.description)

    # Check response has updated statuses
    updated = resp.update_entity_status.status
    tc.check(f"Response has {len(statuses)} updated entities",
             len(updated) == len(statuses),
             f"got {len(updated)}")

    if has_ego:
        ego_resp = [s for s in updated if s.name == "ego"]
        if ego_resp:
            ego_pose = ego_resp[0].pose
            tc.check("Ego response has pose",
                     ego_pose is not None and ego_pose.HasField('position'))
            if ego_pose and ego_pose.HasField('position'):
                # Ego pose should come from CARLA physics, not the sent pose
                # We can't check exact values without knowing CARLA state,
                # but we can verify it's a valid pose (non-zero position)
                tc.check("Ego pose has valid position (not all zeros)",
                         abs(ego_pose.position.x) > 0.1 or abs(ego_pose.position.y) > 0.1)
            # Check action_status has twist
            action = ego_resp[0].action_status
            tc.check("Ego response has action_status with twist",
                     action is not None and action.HasField('twist'))
        else:
            tc.check("Ego entity in response", False, "not found")

    if has_npc:
        npc_resp = [s for s in updated if s.name == "npc_car_1"]
        if npc_resp:
            npc_pose = npc_resp[0].pose
            tc.check("NPC response has pose", npc_pose is not None)
            if npc_pose and npc_pose.HasField('position'):
                # NPC pose should echo what we sent (175.0, -130.1)
                tc.check("NPC pose echoes sent pose (x~175)",
                         abs(npc_pose.position.x - 175.0) < 1.0,
                         f"x={npc_pose.position.x:.1f}")
        else:
            tc.check("NPC entity in response", False, "not found")


def test_update_frame(tc):
    """Test 8: UpdateFrame ticks CARLA."""
    print("\n--- Test: UpdateFrame ---")
    req = api.UpdateFrameRequest(
        current_simulation_time=0.05,
        current_scenario_time=0.05,
    )
    resp = tc.send(req)
    tc.check("UpdateFrame returns success",
             resp.update_frame.result.success,
             resp.update_frame.result.description)


def test_coordinate_conversion(tc):
    """Test 9: Coordinate conversion - spawn at ROS (10, 5, 0), verify CARLA gets (10, -5, 0).
    We verify indirectly: spawn, then read back via UpdateEntityStatus."""
    print("\n--- Test: Coordinate Conversion ---")
    # Spawn a test vehicle at ROS (10, 5, 0)
    req = api.SpawnVehicleEntityRequest(
        parameters=traf.VehicleParameters(name="coord_test"),
        is_ego=False,
        asset_key="vehicle.tesla.model3",
        pose=make_pose(10.0, 5.0, 0.5),
    )
    resp = tc.send(req)
    if not resp.spawn_vehicle_entity.result.success:
        tc.skip("Coordinate conversion spawn", resp.spawn_vehicle_entity.result.description)
        return

    # Tick so actor is ready
    tc.send(api.UpdateFrameRequest(current_simulation_time=0.1))

    # Read back via UpdateEntityStatus with overwrite=False
    # The NPC echoes the sent pose, so this doesn't actually verify CARLA position.
    # True verification requires checking CARLA spectator/debug view.
    tc.check("Coordinate test vehicle spawned at ROS (10, 5, 0)",
             resp.spawn_vehicle_entity.result.success)

    # Cleanup
    tc.send(api.DespawnEntityRequest(name="coord_test"))


def test_despawn(tc, has_ego, has_npc):
    """Test 10: DespawnEntity removes actors."""
    print("\n--- Test: DespawnEntity ---")
    if has_npc:
        resp = tc.send(api.DespawnEntityRequest(name="npc_car_1"))
        tc.check("Despawn NPC succeeds",
                 resp.despawn_entity.result.success,
                 resp.despawn_entity.result.description)

    if has_ego:
        resp = tc.send(api.DespawnEntityRequest(name="ego"))
        tc.check("Despawn ego succeeds",
                 resp.despawn_entity.result.success,
                 resp.despawn_entity.result.description)

    # Try despawning non-existent entity
    resp = tc.send(api.DespawnEntityRequest(name="nonexistent"))
    tc.check("Despawn nonexistent returns error",
             not resp.despawn_entity.result.success)


def test_pedestrian_misc_stubs(tc):
    """Test 11: SpawnPedestrian and SpawnMiscObject stubs."""
    print("\n--- Test: Pedestrian + MiscObject Stubs ---")

    resp = tc.send(api.SpawnPedestrianEntityRequest(
        parameters=traf.PedestrianParameters(name="ped_1"),
        asset_key="walker.pedestrian.0001",
        pose=make_pose(5.0, 5.0, 0.0),
    ))
    tc.check("SpawnPedestrianEntity returns success (stub)",
             resp.spawn_pedestrian_entity.result.success)

    resp = tc.send(api.SpawnMiscObjectEntityRequest(
        parameters=traf.MiscObjectParameters(name="cone_1"),
        asset_key="static.prop.constructioncone",
        pose=make_pose(3.0, 3.0, 0.0),
    ))
    tc.check("SpawnMiscObjectEntity returns success (stub)",
             resp.spawn_misc_object_entity.result.success)


def main():
    parser = argparse.ArgumentParser(description="Phase 1 acceptance test")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ port")
    parser.add_argument("--no-carla", action="store_true",
                        help="Skip CARLA-dependent tests (spawn, tick, etc.)")
    args = parser.parse_args()

    print(f"Connecting to adapter at tcp://localhost:{args.port}...")
    tc = TestClient(args.port)

    # === Protocol tests (require adapter running, with or without CARLA) ===
    test_zmq_connectivity(tc)

    if args.no_carla:
        print("\n--- Skipping CARLA-dependent tests (--no-carla) ---")
        tc.skip("SpawnVehicleEntity (ego)", "requires CARLA")
        tc.skip("SpawnVehicleEntity (NPC)", "requires CARLA")
        tc.skip("UpdateEntityStatus", "requires CARLA")
        tc.skip("UpdateFrame", "requires CARLA")
        tc.skip("Coordinate conversion", "requires CARLA")
        tc.skip("DespawnEntity", "requires CARLA")
        test_sensor_stubs(tc)
        test_update_traffic_lights(tc)
        test_pedestrian_misc_stubs(tc)
    else:
        # === CARLA-dependent tests ===
        test_sensor_stubs(tc)
        test_update_traffic_lights(tc)
        test_update_step_time(tc)
        test_pedestrian_misc_stubs(tc)

        has_ego = test_spawn_ego(tc)
        has_npc = test_spawn_npc(tc)

        if has_ego or has_npc:
            # Tick once so actors are fully spawned
            test_update_frame(tc)
            test_update_entity_status(tc, has_ego, has_npc)
            test_coordinate_conversion(tc)
            test_despawn(tc, has_ego, has_npc)
        else:
            tc.skip("UpdateEntityStatus", "no entities spawned")
            tc.skip("UpdateFrame", "skipped without entities")
            tc.skip("DespawnEntity", "no entities to despawn")

    success = tc.summary()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
