"""Unit tests for gap_geometry, the oracle's 2D box geometry (roadmap 024 step 2).

No ROS, no CARLA:

    python3 -m pytest src/csb_launch/test/test_gap_geometry.py
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import gap_geometry as gg  # noqa: E402

DEG = math.pi / 180.0


def box(cx, cy, length, width, yaw_deg=0.0):
    return gg.OrientedBox(cx, cy, yaw_deg * DEG, length / 2.0, width / 2.0)


class SignedDistanceAxisAligned(unittest.TestCase):
    def test_separated(self):
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(5, 0, 4, 2)), 1.0)
        self.assertFalse(gg.overlaps(box(0, 0, 4, 2), box(5, 0, 4, 2)))

    def test_separated_laterally(self):
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(0, 2.5, 4, 2)), 0.5)

    def test_touching_is_zero_and_counts_as_overlap(self):
        # SSv2's check is boost::geometry::intersects, which is true for touching boxes.
        d = gg.signed_distance(box(0, 0, 4, 2), box(4, 0, 4, 2))
        self.assertAlmostEqual(d, 0.0)
        self.assertTrue(gg.overlaps(box(0, 0, 4, 2), box(4, 0, 4, 2)))

    def test_penetration_is_negative_depth(self):
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(3, 0, 4, 2)), -1.0)
        self.assertTrue(gg.overlaps(box(0, 0, 4, 2), box(3, 0, 4, 2)))

    def test_penetration_takes_the_shallow_axis(self):
        # 3.5 m deep along x, 0.2 m along y: the way out is sideways.
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(0.5, 1.8, 4, 2)), -0.2)

    def test_containment(self):
        # A 1 x 1 box in the middle of a 4 x 2: the shortest way out is 1.5 m sideways.
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(0, 0, 1, 1)), -1.5)

    def test_corner_to_corner_is_euclidean(self):
        # Separating-axis gap is 1 m on each axis; the true distance is the diagonal.
        self.assertAlmostEqual(
            gg.signed_distance(box(0, 0, 2, 2), box(3, 3, 2, 2)), math.sqrt(2.0)
        )

    def test_symmetric(self):
        a, b = box(0, 0, 4, 2, 10), box(3, 2, 5, 2.5, 70)
        self.assertAlmostEqual(gg.signed_distance(a, b), gg.signed_distance(b, a))


class SignedDistanceRotated(unittest.TestCase):
    def test_90_degrees(self):
        # B turned 90 deg spans x 2.5..4.5; A ends at 2.0.
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(3.5, 0, 4, 2, 90)), 0.5)
        self.assertAlmostEqual(gg.signed_distance(box(0, 0, 4, 2), box(2.5, 0, 4, 2, 90)), -0.5)

    def test_45_degrees_corner_to_face(self):
        r2 = math.sqrt(2.0)
        # B's corner points at A's face: its leftmost point is at cx - sqrt(2).
        self.assertAlmostEqual(
            gg.signed_distance(box(0, 0, 2, 2), box(1 + r2 + 0.5, 0, 2, 2, 45)), 0.5
        )
        self.assertAlmostEqual(
            gg.signed_distance(box(0, 0, 2, 2), box(1 + r2 - 0.3, 0, 2, 2, 45)), -0.3
        )
        self.assertAlmostEqual(
            gg.signed_distance(box(0, 0, 2, 2), box(1 + r2, 0, 2, 2, 45)), 0.0
        )

    def test_45_degrees_corner_to_corner(self):
        # Both turned 45 deg, side by side along x: corners meet along the line y = 0.
        r2 = math.sqrt(2.0)
        self.assertAlmostEqual(
            gg.signed_distance(box(0, 0, 2, 2, 45), box(2 * r2 + 0.25, 0, 2, 2, 45)), 0.25
        )

    def test_invariant_under_rigid_motion_of_the_scene(self):
        a, b = box(0, 0, 4.7, 2.1, 0), box(5.5, 1.0, 12, 2.5, 20)
        d = gg.signed_distance(a, b)
        for turn_deg in (30, 45, 90, 180, -135):
            t = turn_deg * DEG
            c, s = math.cos(t), math.sin(t)

            def moved(o):
                return gg.OrientedBox(
                    c * o.cx - s * o.cy + 7.0,
                    s * o.cx + c * o.cy - 3.0,
                    o.yaw + t,
                    o.half_length,
                    o.half_width,
                )

            self.assertAlmostEqual(gg.signed_distance(moved(a), moved(b)), d, places=9)

    def test_yaw_wraps(self):
        self.assertAlmostEqual(
            gg.signed_distance(box(0, 0, 4, 2), box(3.5, 0, 4, 2, 90)),
            gg.signed_distance(box(0, 0, 4, 2), box(3.5, 0, 4, 2, -270)),
        )


class BoxesFromPoses(unittest.TestCase):
    def test_ssv2_box_center_is_shifted_in_the_entity_frame(self):
        # Rear-axle origin, box centre 1.5 m ahead; heading 90 deg puts it 1.5 m along +y.
        b = gg.ssv2_box(gg.Pose2D(10, 5, 90 * DEG), (1.5, 0.0), (4.7, 2.1))
        self.assertAlmostEqual(b.cx, 10.0)
        self.assertAlmostEqual(b.cy, 6.5)
        self.assertAlmostEqual(b.half_length, 2.35)
        self.assertAlmostEqual(b.half_width, 1.05)

    def test_carla_box_uses_location_offset_and_extent(self):
        b = gg.carla_box(gg.Pose2D(0, 0, 180 * DEG), (0.1, 0.2), (2.35, 1.05))
        self.assertAlmostEqual(b.cx, -0.1)
        self.assertAlmostEqual(b.cy, -0.2)
        self.assertAlmostEqual(b.half_length, 2.35)

    def test_corners(self):
        cs = box(0, 0, 4, 2, 90).corners()
        xs = sorted(round(x, 9) for x, _ in cs)
        ys = sorted(round(y, 9) for _, y in cs)
        self.assertEqual(xs, [-1, -1, 1, 1])
        self.assertEqual(ys, [-2, -2, 2, 2])


class DeclaredVersusBlueprintSize(unittest.TestCase):
    """A declared 12 m truck that CARLA spawns as a 4.7 m Model 3 (022's override)."""

    EGO_CENTER, EGO_DIMS = (1.5, 0.0), (4.9, 1.9)
    TRUCK_CENTER, TRUCK_DIMS = (4.0, 0.0), (12.0, 2.5)
    MODEL3_EXTENT = (2.35, 1.05)

    def geometries(self, truck_x):
        ego_pose = gg.Pose2D(0.0, 0.0, 0.0)
        truck_pose = gg.Pose2D(truck_x, 0.0, 0.0)
        ego = gg.ssv2_box(ego_pose, self.EGO_CENTER, self.EGO_DIMS)
        d_ssv2 = gg.signed_distance(
            ego, gg.ssv2_box(truck_pose, self.TRUCK_CENTER, self.TRUCK_DIMS)
        )
        # The bridge places the actor at the entity origin shifted by OriginOffset.
        truck_actor = gg.actor_pose_from_entity(truck_pose, self.TRUCK_CENTER[0], 0.0, 0.0)
        ego_actor = gg.actor_pose_from_entity(ego_pose, self.EGO_CENTER[0], 0.0, 0.0)
        d_carla = gg.signed_distance(
            gg.carla_box(ego_actor, (0.0, 0.0), (2.45, 0.95)),
            gg.carla_box(truck_actor, (0.0, 0.0), self.MODEL3_EXTENT),
        )
        return d_ssv2, d_carla

    def test_overlap_only_in_ssv2(self):
        # Ego front at 3.95; the declared truck's rear at 3.0, the Model 3's at 6.65.
        d_ssv2, d_carla = self.geometries(5.0)
        self.assertAlmostEqual(d_ssv2, -0.95)
        self.assertAlmostEqual(d_carla, 2.7)
        self.assertTrue(gg.overlaps_in_one_only(d_ssv2, d_carla))
        self.assertAlmostEqual(d_ssv2 - d_carla, -3.65)

    def test_clear_in_both(self):
        d_ssv2, d_carla = self.geometries(8.0)
        self.assertGreater(d_ssv2, 0.0)
        self.assertGreater(d_carla, 0.0)
        self.assertFalse(gg.overlaps_in_one_only(d_ssv2, d_carla))

    def test_matching_sizes_agree(self):
        ego = gg.ssv2_box(gg.Pose2D(0, 0, 0), (1.5, 0.0), (4.7, 2.1))
        npc_pose = gg.Pose2D(5.0, 0.0, 0.0)
        d_ssv2 = gg.signed_distance(ego, gg.ssv2_box(npc_pose, (1.5, 0.0), (4.7, 2.1)))
        d_carla = gg.signed_distance(
            gg.carla_box(gg.actor_pose_from_entity(gg.Pose2D(0, 0, 0), 1.5, 0, 0), (0, 0), (2.35, 1.05)),
            gg.carla_box(gg.actor_pose_from_entity(npc_pose, 1.5, 0, 0), (0, 0), (2.35, 1.05)),
        )
        self.assertAlmostEqual(d_ssv2, d_carla)


class OriginOffset(unittest.TestCase):
    """The trace is raw; the oracle applies OriginOffset once. A wrong count must show."""

    ENTITY = gg.Pose2D(10.0, 5.0, 30 * DEG)
    OFFSET = (1.5, 0.0, 0.0)

    def carla_actor(self):
        # Where the bridge puts the actor (coordinate_conversion::entity_origin_to_actor_origin).
        c, s = math.cos(30 * DEG), math.sin(30 * DEG)
        return gg.Pose2D(10.0 + 1.5 * c, 5.0 + 1.5 * s, 30 * DEG)

    def test_applied_once_matches(self):
        expected = gg.actor_pose_from_entity(self.ENTITY, *self.OFFSET)
        dist, dyaw = gg.pose_error(expected, self.carla_actor())
        self.assertAlmostEqual(dist, 0.0)
        self.assertAlmostEqual(dyaw, 0.0)

    def test_round_trip(self):
        actor = gg.actor_pose_from_entity(self.ENTITY, 1.5, 0.3, 0.0)
        back = gg.entity_pose_from_actor(actor, 1.5, 0.3, 0.0)
        dist, dyaw = gg.pose_error(back, self.ENTITY)
        self.assertAlmostEqual(dist, 0.0)
        self.assertAlmostEqual(dyaw, 0.0)

    def test_applied_twice_is_visible(self):
        once = gg.actor_pose_from_entity(self.ENTITY, *self.OFFSET)
        twice = gg.actor_pose_from_entity(once, *self.OFFSET)
        dist, _ = gg.pose_error(twice, self.carla_actor())
        self.assertAlmostEqual(dist, 1.5)
        self.assertGreater(dist, 0.2)  # beyond G3's in-plane budget

    def test_not_applied_is_visible(self):
        dist, _ = gg.pose_error(self.ENTITY, self.carla_actor())
        self.assertAlmostEqual(dist, 1.5)
        self.assertGreater(dist, 0.2)

    def test_wrong_offset_in_carla_flips_the_overlap_verdict(self):
        # An NPC 1.0 m ahead of the ego in SSv2's geometry, both declared 4.7 x 2.1 with
        # the rear-axle origin 1.5 m behind the box centre.
        center, dims, extent = (1.5, 0.0), (4.7, 2.1), (2.35, 1.05)
        ego_pose, npc_pose = gg.Pose2D(0, 0, 0), gg.Pose2D(5.7, 0, 0)
        d_ssv2 = gg.signed_distance(
            gg.ssv2_box(ego_pose, center, dims), gg.ssv2_box(npc_pose, center, dims)
        )
        self.assertAlmostEqual(d_ssv2, 1.0)
        ego_carla = gg.carla_box(gg.actor_pose_from_entity(ego_pose, 1.5, 0, 0), (0, 0), extent)

        def d_carla(npc_actor):
            return gg.signed_distance(ego_carla, gg.carla_box(npc_actor, (0, 0), extent))

        once = gg.actor_pose_from_entity(npc_pose, 1.5, 0, 0)
        self.assertAlmostEqual(d_carla(once), 1.0)
        # Not applied: the NPC's body sits 1.5 m back and overlaps the ego by 0.5 m.
        self.assertAlmostEqual(d_carla(npc_pose), -0.5)
        self.assertTrue(gg.overlaps_in_one_only(d_ssv2, d_carla(npc_pose)))
        # Applied twice: 1.5 m too far ahead; no overlap flip, but a 1.5 m margin difference.
        twice = gg.actor_pose_from_entity(once, 1.5, 0, 0)
        self.assertAlmostEqual(d_carla(twice) - d_ssv2, 1.5)

    def test_turned_prop(self):
        # 022: a prop whose long axis is across its blueprint is spawned turned 90 deg
        # (OriginOffset yaw); its box in the actor's own frame is then the declared box turned.
        entity = gg.Pose2D(3.0, -2.0, 20 * DEG)
        declared = gg.ssv2_box(entity, (0.0, 0.0), (1.21, 0.37))
        actor = gg.actor_pose_from_entity(entity, 0.0, 0.0, 90 * DEG)
        self.assertAlmostEqual(actor.yaw, 110 * DEG)
        body = gg.carla_box(actor, (0.0, 0.0), (0.185, 0.605))
        self.assertAlmostEqual(body.cx, declared.cx)
        self.assertAlmostEqual(body.cy, declared.cy)
        probe = box(3.0, 0.0, 1.0, 1.0, 0)
        self.assertAlmostEqual(
            gg.signed_distance(declared, probe), gg.signed_distance(body, probe)
        )
        # Forgetting the turn shows as a 90 deg yaw error and a different margin.
        unturned = gg.carla_box(entity, (0.0, 0.0), (0.185, 0.605))
        _, dyaw = gg.pose_error(entity, actor)
        self.assertAlmostEqual(dyaw, 90 * DEG)
        self.assertNotAlmostEqual(
            gg.signed_distance(declared, probe), gg.signed_distance(unturned, probe)
        )


class PoseError(unittest.TestCase):
    def test_yaw_difference_wraps(self):
        _, dyaw = gg.pose_error(gg.Pose2D(0, 0, 179 * DEG), gg.Pose2D(0, 0, -179 * DEG))
        self.assertAlmostEqual(dyaw, 2 * DEG)


class HeightGate(unittest.TestCase):
    """SSv2's checkCollision2D first rejects pairs whose boxes do not meet in z."""

    def test_apart_in_z(self):
        # Box centres at z 0.75 and 3.0: 2.25 m apart, half-heights sum to 1.5.
        self.assertTrue(gg.z_separated(0.0, 0.75, 1.5, 3.0, 0.0, 1.5))

    def test_overlapping_in_z(self):
        self.assertFalse(gg.z_separated(0.0, 0.75, 1.5, 0.5, 0.75, 1.5))


if __name__ == "__main__":
    unittest.main()
