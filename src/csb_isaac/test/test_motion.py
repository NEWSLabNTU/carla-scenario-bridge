"""Tracked bodies' arithmetic (roadmap 044); no Isaac needed."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac import motion  # noqa: E402

YAW_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


class MotionTest(unittest.TestCase):
    def test_rotate_turns_x_into_y_at_90_degrees(self):
        self.assertTrue(close(motion.rotate(YAW_90, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0)))

    def test_the_centre_of_a_turning_body_moves_faster_than_its_origin(self):
        # Origin at the rear axle, centre 1.5 m ahead, heading +y, turning left at 1 rad/s:
        # the centre also moves at 1.5 m/s toward -x.
        v = motion.com_velocity((0.0, 8.0, 0.0), (0.0, 0.0, 1.0), YAW_90, (1.5, 0.0, 0.9))
        self.assertTrue(close(v, (-1.5, 8.0, 0.0)), v)
        back = motion.origin_velocity(v, (0.0, 0.0, 1.0), YAW_90, (1.5, 0.0, 0.9))
        self.assertTrue(close(back, (0.0, 8.0, 0.0)), back)

    def test_mass_from_the_catalogue_else_by_kind(self):
        self.assertEqual(motion.body_mass("VEHICLE", "car", {"mass": 1845.0}), 1845.0)
        self.assertEqual(motion.body_mass("VEHICLE", "car", None), 1500.0)
        # CARLA's bicycles weigh a kilogram: a two-wheeler with its rider is 200 kg.
        self.assertEqual(motion.body_mass("VEHICLE", "bicycle", {"mass": 1.0}), 200.0)
        self.assertEqual(motion.body_mass("PEDESTRIAN", "pedestrian", None), 80.0)

    def test_yaw_roll_pitch(self):
        self.assertAlmostEqual(motion.yaw_of(YAW_90), math.pi / 2)
        self.assertEqual(motion.roll_pitch_of((1.0, 0.0, 0.0, 0.0)), (0.0, 0.0))
        self.assertEqual(motion.normalized((2.0, 0.0, 0.0, 0.0)), (1.0, 0.0, 0.0, 0.0))

    def test_skids_raise_the_box_and_keep_its_top_and_bottom(self):
        box_c, box_d, caps = motion.skids((1.532, 0.0, 0.744), (4.792, 2.163, 1.488))
        r = motion.SKID_RADIUS
        self.assertAlmostEqual(box_c[2] + box_d[2] / 2, 1.488)  # the top stays
        self.assertAlmostEqual(box_c[2] - box_d[2] / 2, r)  # the bottom is raised by r
        self.assertEqual(len(caps), 2)
        for (cx, cy, cz), half, cr in caps:
            self.assertAlmostEqual(cz - cr, 0.0)  # capsules on the declared bottom
            self.assertLessEqual(abs(cx - 1.532) + half + cr, 4.792 / 2)
            self.assertLessEqual(abs(cy) + cr, 2.163 / 2)
        # A body too small for skids keeps its box.
        self.assertEqual(motion.skids((0, 0, 0.1), (0.2, 0.2, 0.2))[2], [])


if __name__ == "__main__":
    unittest.main()
