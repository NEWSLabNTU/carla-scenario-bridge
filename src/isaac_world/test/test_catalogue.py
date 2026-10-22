"""The ego (aib's rig) against CARLA's catalogue (roadmaps 043, 045): pure data, no Isaac.

    python -m unittest discover -s src/isaac_world/test
"""
import copy
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
AIB = HERE.parent / "autoware_isaac_bridge/src/aib"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(AIB))

if (AIB / "aib").is_dir():
    from aib.params import VehicleMismatch, bicycle_limit  # noqa: E402

    from isaac_world.catalogue import check_ego, ego_model, load_ego, physics_params  # noqa: E402

DEFAULT_EGO = AIB / "config/ego.yaml"


def wheel(x, y, steer, handbrake=0.0):
    return {"tire_friction": 3.5, "damping_rate": 0.25, "max_steer_angle_deg": steer,
            "radius": 0.37, "max_brake_torque": 700.0, "max_handbrake_torque": handbrake,
            "lat_stiff_max_load": 3.0, "lat_stiff_value": 20.0, "long_stiff_value": 3000.0,
            "position": [x, y, 0.3079]}


# CARLA 0.9.16's vehicle.tesla.model3, as scripts/dump_vehicle_physics.py writes it.
TESLA = {
    "mass": 1845.0, "drag_coefficient": 0.15, "center_of_mass_nudge": [0.45, 0.0, -0.3],
    "com_estimate": [0.4792, 0.0, 0.4359], "max_rpm": 15000.0,
    "torque_curve": [[-9.31, 743.0], [9000.0, 743.0], [15000.0, 502.0]],
    "final_ratio": 9.0, "forward_gears": [{"ratio": 1.0, "down_ratio": 0.0, "up_ratio": 0.0}],
    "wheels": [wheel(1.6182, 0.8334, 70.0), wheel(1.6182, -0.8334, 70.0),
               wheel(-1.3864, 0.8334, 0.0, 1400.0), wheel(-1.3864, -0.8334, 0.0, 1400.0)],
    "wheel_base": 3.0046, "wheel_tread": 1.6668, "wheel_radius": 0.37, "rear_axle_x": -1.3864,
}


@unittest.skipUnless((AIB / "aib").is_dir(), "no aib submodule (git submodule update --init)")
class PhysicsTest(unittest.TestCase):
    def test_the_blueprint_gives_the_physics(self):
        p = physics_params(TESLA)
        self.assertEqual(p["mass"], 1845.0)
        self.assertEqual(p["max_brake_torque"], 700.0)
        self.assertEqual(p["tire_friction"], 3.5)
        self.assertAlmostEqual(p["max_drive_torque"], 743.0 * 9.0 / 4)
        # The centre of mass in base_link: from the rear axle, above the ground.
        x, y, z = p["com"]
        self.assertAlmostEqual(x, 0.4792 + 1.3864, places=6)
        self.assertAlmostEqual(z, 0.4359 - (0.3079 - 0.37), places=6)

    def test_the_steering_limit_is_the_inner_wheels(self):
        # 70 deg at the inner wheel: the bicycle angle whose inner wheel sits there.
        p = physics_params(TESLA)
        radius = 3.0046 / math.tan(p["max_steer_angle"])
        inner = math.atan(3.0046 / (radius - 1.6668 / 2))
        self.assertAlmostEqual(math.degrees(inner), 70.0, places=6)
        self.assertEqual(bicycle_limit(0.0, 3.0, 1.6), 0.0)


@unittest.skipUnless(DEFAULT_EGO.exists(), "no aib submodule (git submodule update --init)")
class EgoTest(unittest.TestCase):
    def setUp(self):
        self.cfg, self.rig = load_ego(DEFAULT_EGO)

    def test_the_default_ego_wears_the_tesla(self):
        self.assertEqual(ego_model(self.rig), "vehicle.tesla.model3")
        self.assertEqual(self.cfg.vehicle or {}, {})

    def test_the_rig_is_the_catalogues_car(self):
        check_ego(self.rig, TESLA)
        check_ego(self.rig, None)

    def test_a_different_car_is_refused_with_both_numbers(self):
        other = copy.deepcopy(TESLA)
        other["wheel_base"] = 2.8604
        with self.assertRaises(VehicleMismatch) as e:
            check_ego(self.rig, other)
        msg = str(e.exception)
        self.assertIn("3.005", msg)
        self.assertIn("2.860", msg)
        self.assertIn("vehicle.tesla.model3", msg)

    def test_within_tolerance_passes(self):
        check_ego(self.rig, dict(TESLA, wheel_tread=self.rig.vehicle.wheel_tread + 0.049))
        with self.assertRaises(VehicleMismatch):
            check_ego(self.rig, dict(TESLA, wheel_radius=self.rig.vehicle.wheel_radius + 0.051))


if __name__ == "__main__":
    unittest.main()
