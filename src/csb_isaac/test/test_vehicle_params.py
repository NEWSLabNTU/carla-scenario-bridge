"""The ego's vehicle parameters from CARLA's blueprint (roadmap 043): pure data, no Isaac.

    python -m unittest discover -s src/csb_isaac/test
"""
import copy
import math
import sys
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac.vehicle_params import (VehicleMismatch, VehicleParams, bicycle_limit,  # noqa: E402
                                      physics_params, read_vehicle_info)

HERE = Path(__file__).resolve().parents[1]
VEHICLE_INFO = (HERE.parent / "autoware_carla_bridge/src/acb_vehicle_launch/"
                "acb_vehicle_description/config/vehicle_info.param.yaml")


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
INFO = {"wheel_base": 3.005, "wheel_tread": 1.667, "wheel_radius": 0.370, "wheel_width": 0.235}


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

    def test_the_catalogue_is_reference_only(self):
        # The ego is a real Model 3, not CARLA's: the catalogue's physics is not applied.
        p = VehicleParams.resolve("vehicle.tesla.model3", TESLA, INFO)
        self.assertEqual(p.tire_friction, 1.0)
        self.assertGreater(p.max_brake_torque, 700.0)


class ResolveTest(unittest.TestCase):
    def test_geometry_is_vehicle_infos(self):
        p = VehicleParams.resolve("vehicle.tesla.model3", TESLA, INFO)
        self.assertEqual(p.wheel_base, 3.005)  # Autoware's, not the catalogue's 3.0046
        self.assertEqual(p.wheel_width, 0.235)
        self.assertEqual(p.mass, 1845.0)

    def test_a_different_car_is_refused_with_both_numbers(self):
        other = copy.deepcopy(TESLA)
        other["wheel_base"] = 2.8604
        with self.assertRaises(VehicleMismatch) as e:
            VehicleParams.resolve("vehicle.lincoln.mkz_2020", other, INFO)
        msg = str(e.exception)
        self.assertIn("3.005", msg)
        self.assertIn("2.860", msg)
        self.assertIn("vehicle.lincoln.mkz_2020", msg)

    def test_within_tolerance_passes(self):
        near = dict(TESLA, wheel_tread=1.667 + 0.049)
        VehicleParams.resolve("x", near, INFO)
        far = dict(TESLA, wheel_radius=0.37 + 0.051)
        with self.assertRaises(VehicleMismatch):
            VehicleParams.resolve("x", far, INFO)

    def test_overrides_win_and_unknown_ones_are_refused(self):
        p = VehicleParams.resolve("vehicle.tesla.model3", TESLA, INFO,
                                  {"mass": 1900.0, "com": [1.45, 0.0, 0.55]})
        self.assertEqual(p.mass, 1900.0)
        self.assertEqual(p.com, (1.45, 0.0, 0.55))
        with self.assertRaises(ValueError):
            VehicleParams.resolve("vehicle.tesla.model3", TESLA, INFO, {"masss": 1.0})

    def test_no_catalogue_is_vehicle_info_and_defaults(self):
        p = VehicleParams.resolve("vehicle.tesla.model3", None, INFO)
        self.assertEqual(p.mass, VehicleParams().mass)
        self.assertEqual(p.wheel_base, 3.005)

    def test_a_hard_stop_can_reach_the_tire_limit(self):
        # Brake torque is not the limit: ~1 g at friction 1.0 needs less than the cap.
        p = VehicleParams()
        need = p.mass * 9.81 * p.tire_friction * p.wheel_radius
        front = p.brake_split(9.81)
        self.assertLess(need * front / 2, p.max_brake_torque)

    def test_braking_moves_load_and_brake_force_forward(self):
        p = VehicleParams()
        static = p.brake_split(0.0)
        self.assertAlmostEqual(static, p.com[0] / p.wheel_base)
        self.assertGreater(p.brake_split(8.0), static + 0.1)

    def test_axle_loads_split_by_the_centre_of_mass(self):
        p = VehicleParams(mass=2000.0, wheel_base=3.0, com=(1.5, 0.0, 0.5))
        self.assertEqual(p.axle_loads(), (500.0, 500.0))
        front, rear = VehicleParams(mass=2000.0, wheel_base=3.0, com=(2.0, 0.0, 0.5)).axle_loads()
        self.assertAlmostEqual(2 * (front + rear), 2000.0)
        self.assertGreater(front, rear)


@unittest.skipUnless(VEHICLE_INFO.exists(), "no acb submodule")
class ShippedTest(unittest.TestCase):
    def test_acbs_vehicle_info_is_the_tesla(self):
        VehicleParams.resolve("vehicle.tesla.model3", TESLA, read_vehicle_info(VEHICLE_INFO))

    def test_the_ego_yaml_names_the_model_and_overrides_nothing(self):
        doc = yaml.safe_load((HERE / "config/acb_ego.yaml").read_text())
        self.assertEqual(doc["model"], "vehicle.tesla.model3")
        self.assertEqual(doc["vehicle"] or {}, {})


if __name__ == "__main__":
    unittest.main()
