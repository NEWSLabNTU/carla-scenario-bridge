"""The ego's sensor frames (roadmap 038): pure maths, no Isaac.

    python -m unittest discover -s src/csb_isaac/test
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac import frames  # noqa: E402

CAL = Path(__file__).resolve().parents[2] / "autoware_carla_bridge/src/individual_params/config/default/acb_sensor_kit"


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


class FramesTest(unittest.TestCase):
    def test_rpy_is_urdf_order(self):
        # yaw 90 deg then roll 90 deg about the rotated x: x stays on +y, z goes to -x... as
        # R = Rz(yaw) Ry(pitch) Rx(roll) applied to vectors.
        q = frames.rpy_quat(math.pi / 2, 0.0, math.pi / 2)
        self.assertTrue(close(frames.qrot(q, (1, 0, 0)), (0, 1, 0)))
        self.assertTrue(close(frames.qrot(q, (0, 0, 1)), (1, 0, 0)))

    def test_compose(self):
        a = ((1.0, 0.0, 0.0), frames.rpy_quat(0, 0, math.pi / 2))
        b = ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        t, _ = frames.compose(a, b)
        self.assertTrue(close(t, (1.0, 1.0, 0.0)))

    @unittest.skipUnless(CAL.is_dir(), "no acb submodule")
    def test_the_lidar_sits_where_the_kit_says(self):
        t, q = frames.kit_transform(CAL / "sensors_calibration.yaml",
                                    CAL / "sensor_kit_calibration.yaml", "velodyne_top_base_link")
        self.assertTrue(close(t, (0.9, 0.0, 2.0)))
        # yaw: the kit's -0.0364 plus the lidar's 1.575
        fwd = frames.qrot(q, (1, 0, 0))
        self.assertAlmostEqual(math.atan2(fwd[1], fwd[0]), 1.575 - 0.0364, places=2)

    def test_lat_lon_round_trip_near_the_origin(self):
        lat, lon = frames.lat_lon(190.0, -130.1)
        y = math.radians(lat) * frames.EARTH_RADIUS
        x = math.radians(lon) * frames.EARTH_RADIUS * math.cos(math.radians(lat))
        self.assertAlmostEqual(x, 190.0, places=6)
        self.assertAlmostEqual(y, -130.1, places=6)


if __name__ == "__main__":
    unittest.main()
