"""The CARLA reference adapter over its fake world: passes the check, drives to the goal and
reports ARRIVED, holds on stop, and reports poses in the map frame. CARLA itself is
exercised live (roadmap 030 step 5); nothing here imports `carla`."""

import time

from swb_agent import Episode
from swb_agent import protocol as P
from swb_agent.adapters.carla_basic import CarlaBasicAgent, map_pose, yaw_of
from swb_agent.check import run_check


def pose(x, y, yaw=0.0):
    return map_pose(x, y, 0.0, yaw)


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def test_passes_check_with_the_fake_world():
    report = run_check(CarlaBasicAgent, {"world": "fake"}, 10.0)
    assert report.passed, report.text()


def test_drives_through_waypoints_to_arrived():
    agent = CarlaBasicAgent({"world": "fake", "target_speed_kmh": 72.0})
    try:
        agent.on_reset(Episode(pose=pose(0.0, 0.0)))
        assert agent.phase == P.IDLE
        agent.on_goal(pose(10.0, 0.0), [pose(5.0, 5.0)])
        assert agent.phase == P.DRIVING
        assert wait_for(lambda: agent.phase == P.ARRIVED)
        x, y, _, _ = agent.world.vehicle_pose()
        assert abs(x - 10.0) < 1e-6 and abs(y) < 1e-6
        assert wait_for(lambda: abs(agent.state_message()["pose"]["x"] - 10.0) < 1e-6)
    finally:
        agent.close()


def test_speed_limit_and_stop():
    agent = CarlaBasicAgent({"world": "fake"})
    try:
        agent.on_reset(Episode(pose=pose(0.0, 0.0)))
        agent.on_speed_limit(2.0)
        assert agent.world.speed == 2.0 or agent._speed_kmh() == 2.0 * 3.6
        agent.on_goal(pose(100.0, 0.0), [])
        assert abs(agent.world.speed - 2.0) < 1e-9
        time.sleep(0.3)
        agent.on_stop()
        x0 = agent.world.vehicle_pose()[0]
        time.sleep(0.3)
        assert agent.world.vehicle_pose()[0] == x0  # held
        agent.on_speed_limit(None)
        assert agent._speed_kmh() == 30.0
    finally:
        agent.close()


def test_map_pose_yaw():
    assert abs(yaw_of(map_pose(0, 0, 0, 1.2)) - 1.2) < 1e-9
