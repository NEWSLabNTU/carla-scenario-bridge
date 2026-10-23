"""Cutting a soil zone's box out of a map's ground meshes (047 S3); no Isaac needed."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isaac_world import ground_cut  # noqa: E402


def area(points, counts, indices):
    total, k = 0.0, 0
    for c in counts:
        v = points[indices[k:k + c]]
        x, y = v[:, 0], v[:, 1]
        total += 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
        k += c
    return total


def grid(n=4, size=8.0):
    """An n x n quad grid over [0, size]^2, x first, with a UV per corner and an id per face."""
    xs = np.linspace(0, size, n + 1)
    gx, gy = np.meshgrid(xs, xs, indexing="ij")
    pts = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], 1)
    counts, idx = [], []
    for i in range(n):
        for j in range(n):
            a = i * (n + 1) + j
            idx += [a, a + n + 1, a + n + 2, a + 1]
            counts.append(4)
    uv = pts[np.array(idx), :2] / size
    return pts, np.array(counts), np.array(idx), uv


class CutTest(unittest.TestCase):
    def test_the_box_is_gone_and_the_rest_is_all_there(self):
        pts, counts, idx, uv = grid()
        box = (3.0, 5.0, 1.0, 6.5)  # across face edges on every side
        p, c, i, _, fv, _, fmap = ground_cut.cut_mesh(pts, counts, idx, box,
                                                      face_varying={"st": uv})
        self.assertAlmostEqual(area(p, c, i), 64.0 - 2.0 * 5.5, places=6)
        # no face reaches into the box
        k = 0
        for n in c:
            v = p[i[k:k + n]]
            cx, cy = v[:, 0].mean(), v[:, 1].mean()
            self.assertFalse(3.0 < cx < 5.0 and 1.0 < cy < 6.5)
            k += n
        # the UVs went with the corners: still position / 8 everywhere
        np.testing.assert_allclose(fv["st"], p[i, :2] / 8.0, atol=1e-9)
        self.assertEqual(len(fmap), len(c))

    def test_faces_wholly_outside_keep_their_indices(self):
        pts, counts, idx, uv = grid()
        p, c, i, _, _, _, fmap = ground_cut.cut_mesh(pts, counts, idx, (2.0, 4.0, 2.0, 4.0))
        # the box is exactly faces (1,1): everything else is untouched, nothing added
        self.assertEqual(len(p), len(pts))
        self.assertEqual(len(c), 15)
        self.assertNotIn(5, fmap.tolist())

    def test_a_mesh_away_from_the_box_is_left_alone(self):
        pts, counts, idx, _ = grid()
        self.assertIsNone(ground_cut.cut_mesh(pts, counts, idx, (20.0, 30.0, 0.0, 8.0)))

    def test_vertex_and_uniform_primvars_and_subsets_follow(self):
        pts, counts, idx, _ = grid(n=2)
        normals = np.tile([0.0, 0.0, 1.0], (len(pts), 1)).astype(np.float32)
        face_id = np.arange(4)
        p, c, i, vx, _, uni, fmap = ground_cut.cut_mesh(
            pts, counts, idx, (3.0, 5.0, 3.0, 5.0), vertex={"normals": normals},
            uniform={"id": face_id})
        self.assertEqual(len(vx["normals"]), len(p))
        np.testing.assert_allclose(vx["normals"][:, 2], 1.0)
        np.testing.assert_array_equal(uni["id"], face_id[fmap])
        sub = ground_cut.remap_subset([0], fmap)
        self.assertTrue(all(fmap[s] == 0 for s in sub))
        self.assertAlmostEqual(area(p, c, i), 64.0 - 4.0, places=6)

    def test_only_faces_in_the_ground_band_are_cut(self):
        pts, counts, idx, _ = grid()
        # an upright face (a guardrail) standing in the box, and the ground under it
        up = np.array([[4, 2, 0], [4, 4, 0], [4, 4, 0.8], [4, 2, 0.8]], float) + [0.5, 0, 0]
        pts2 = np.vstack([pts, up])
        counts2 = np.append(counts, 4)
        idx2 = np.append(idx, np.arange(len(pts), len(pts) + 4))
        p, c, i, _, _, _, fmap = ground_cut.cut_mesh(pts2, counts2, idx2, (2.0, 6.0, 2.0, 6.0),
                                                     z_band=(-0.8, 0.05))
        self.assertIn(16, fmap.tolist())  # the upright face is kept, whole
        self.assertAlmostEqual(area(p, c[:-1], i[:-4]), 64.0 - 16.0, places=6)

    def test_a_triangle_across_a_corner(self):
        pts = np.array([[0, 0, 0], [6, 0, 0], [0, 6, 0]], float)
        p, c, i, _, _, _, _ = ground_cut.cut_mesh(pts, [3], [0, 1, 2], (1.0, 9.0, 1.0, 9.0))
        # the triangle less its part in x > 1, y > 1: 18 - (4 * 4 / 2)
        self.assertAlmostEqual(area(p, c, i), 18.0 - 8.0, places=6)


if __name__ == "__main__":
    unittest.main()
