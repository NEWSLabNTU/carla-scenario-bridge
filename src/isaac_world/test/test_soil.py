"""The soil layer's wheel models (047 S3): the survey's trends, on the shipped class table's
sand and sandy loam, for aib's default wheel (0.37 m radius, 0.235 m wide)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isaac_world import soil, surfaces  # noqa: E402
from isaac_world.soil import SoilError, SoilGrid, wong_reece  # noqa: E402

TABLE = surfaces.load_classes()
SAND = TABLE["sand"].soil
LOAM = TABLE["dirt"].soil
R, B = 0.37, 0.235


def drive(grid, load=4500.0, slip=0.2, speed=5.0, dt=1 / 60, length=6.0, y=0.013, key=0):
    """One wheel straight along +x; the steady second half's mean WheelSoil fields."""
    out, x = [], 0.0
    while x < length:
        grid.begin_step()
        out.append(grid.wheel(key, x, y, 0.0, load, R, B, speed / (1 - slip), speed, dt))
        x += speed * dt
    half = out[len(out) // 2:]
    return {f: sum(getattr(o, f) for o in half) / len(half)
            for f in ("sinkage", "resistance", "thrust", "traction_limit", "load")}


def wr(p, load=4500.0, slip=0.2, speed=5.0):
    return wong_reece(p, load, R, B, speed / (1 - slip), speed)


class ParamsTest(unittest.TestCase):
    def test_the_table_has_soil_for_sand_dirt_and_snow_and_none_for_mud(self):
        self.assertAlmostEqual(SAND.friction_angle, math.radians(28.0))
        self.assertEqual(LOAM.n, 0.7)
        self.assertIsNotNone(TABLE["snow_loose"].soil)
        self.assertIsNone(TABLE["mud_wet"].soil)  # 047 D6
        self.assertIsNone(TABLE["asphalt"].soil)

    def test_every_problem_is_listed(self):
        with self.assertRaises(SoilError) as e:
            soil.parse_params({"n": 0, "k_c": -1, "k_phi": "x", "friction_angle": 95,
                               "janosi_k": 0.01, "colour": 1}, "sand: soil")
        text = str(e.exception)
        for needle in ("n 0 is not a number > 0", "k_c -1", "k_phi 'x'", "needs cohesion",
                       "friction_angle 95", "unknown key(s) ['colour']", "sand: soil"):
            self.assertIn(needle, text)


class ClosedFormTest(unittest.TestCase):
    """C, Wong-Reece on fresh flat soil."""

    def test_the_soil_carries_the_load(self):
        for p in (SAND, LOAM):
            self.assertAlmostEqual(wr(p).load, 4500.0, delta=20.0)

    def test_sinkage_and_resistance_grow_with_load_and_slip(self):
        for p in (SAND, LOAM):
            loads = [wr(p, load=w) for w in (3000.0, 4500.0, 6000.0)]
            self.assertEqual(loads, sorted(loads, key=lambda o: o.sinkage))
            self.assertEqual(loads, sorted(loads, key=lambda o: o.resistance))
            slips = [wr(p, slip=i).sinkage for i in (0.0, 0.2, 0.5)]
            self.assertEqual(slips, sorted(slips))
            self.assertLess(slips[0] * 1.3, slips[2])

    def test_resistance_outgrows_thrust_as_the_wheel_sinks(self):
        ratios = [wr(SAND, load=w).resistance / wr(SAND, load=w).thrust
                  for w in (3000.0, 4500.0, 6000.0)]
        self.assertEqual(ratios, sorted(ratios))

    def test_soft_soil_grips_less_than_asphalt(self):
        for p in (SAND, LOAM):
            mu = wr(p).mu(4500.0)
            self.assertLess(mu, TABLE["asphalt"].mu)
            self.assertGreater(mu, 0.3)

    def test_a_locked_wheel_pulls_back_and_reversing_mirrors(self):
        locked = wong_reece(SAND, 4500.0, R, B, 0.0, 5.0)
        self.assertLess(locked.thrust, -0.9 * locked.traction_limit)
        fwd, back = wr(SAND, speed=3.0), wr(SAND, speed=-3.0)
        self.assertAlmostEqual(fwd.sinkage, back.sinkage)
        self.assertAlmostEqual(fwd.thrust, -back.thrust)

    def test_no_load_no_soil(self):
        self.assertEqual(wr(SAND, load=0.0).sinkage, 0.0)


class GridTest(unittest.TestCase):
    """A, the sparse grid."""

    def test_sinkage_grows_with_load_and_slip(self):
        z = [drive(SoilGrid(SAND), load=w)["sinkage"] for w in (3000.0, 4500.0, 6000.0)]
        self.assertEqual(z, sorted(z))
        z = [drive(SoilGrid(SAND), slip=i)["sinkage"] for i in (0.0, 0.2, 0.5)]
        self.assertEqual(z, sorted(z))
        self.assertLess(z[0] * 1.3, z[2])

    def test_the_rut_stays_and_a_second_pass_sinks_less_and_rolls_easier(self):
        for p in (SAND, LOAM):
            g = SoilGrid(p)
            first = drive(g)
            rut = g.depth(3.0, 0.0)
            self.assertGreater(rut, 0.8 * first["sinkage"])
            second = drive(g)
            # sinkage below the undisturbed surface: the second pass adds less than the first
            self.assertLess(second["sinkage"] - rut, 0.5 * first["sinkage"])
            self.assertLess(second["resistance"], 0.8 * first["resistance"])

    def test_resistance_outgrows_thrust_as_the_wheel_sinks(self):
        r = [drive(SoilGrid(SAND), load=w) for w in (3000.0, 6000.0)]
        self.assertLess(r[0]["resistance"] / r[0]["traction_limit"],
                        r[1]["resistance"] / r[1]["traction_limit"])

    def test_first_pass_is_near_the_closed_form(self):
        # Not the same stress field: Wong-Reece's normal stress falls to 0 at the bottom of the
        # wheel (exit angle 0) while the grid's nodes there carry the most, so the grid sinks
        # less and pushes less soil (sand 0.72 and 0.53 of it, loam 0.60 and 0.40); S5's
        # calibration against Chrono decides which one to trust. The grip agrees.
        for p in (SAND, LOAM):
            g, c = drive(SoilGrid(p)), wr(p)
            self.assertAlmostEqual(g["load"], 4500.0, delta=45.0)
            for f, lo in (("sinkage", 0.5), ("resistance", 0.35)):
                self.assertTrue(lo < g[f] / getattr(c, f) < 1.0, (f, g[f], getattr(c, f)))
            self.assertAlmostEqual(g["traction_limit"] / c.traction_limit, 1.0, delta=0.3)

    def test_the_physics_rate_does_not_matter(self):
        # D4: the sweep integrates the patch along the path, so 60 Hz reads as 600 Hz.
        for speed in (5.0, 25.0):
            slow = drive(SoilGrid(SAND), slip=0.1, speed=speed, dt=1 / 60, length=12.0)
            fast = drive(SoilGrid(SAND), slip=0.1, speed=speed, dt=1 / 600, length=12.0)
            self.assertAlmostEqual(slow["sinkage"] / fast["sinkage"], 1.0, delta=0.03)
            self.assertAlmostEqual(slow["resistance"] / fast["resistance"], 1.0, delta=0.08)

    def test_it_repeats(self):
        a, b = SoilGrid(SAND), SoilGrid(SAND)
        self.assertEqual(drive(a), drive(b))
        for k in a.blocks:
            np.testing.assert_array_equal(a.blocks[k].sinkage, b.blocks[k].sinkage)

    def test_blocks_are_allocated_only_where_wheels_went(self):
        g = SoilGrid(SAND)
        drive(g, length=40.0, speed=10.0)
        # a 0.235 m track along y = 0.013 touches two block rows (block = 0.8 m)
        rows = {by for _, by in g.blocks}
        self.assertLessEqual(len(rows), 2)
        self.assertLess(g.nbytes() / 40.0 * 1000.0, 12e6)  # per km of one wheel's track
        self.assertEqual(g.depth(10.0, 5.0), 0.0)

    def test_dirty_blocks_are_handed_out_once(self):
        g = SoilGrid(SAND)
        drive(g, length=3.0)
        dirty = g.dirty()
        self.assertTrue(dirty)
        self.assertGreater(max(float(v.max()) for v in dirty.values()), 0.05)
        self.assertEqual(g.dirty(), {})

    def test_tile_heights_span_blocks_and_share_edges(self):
        g = SoilGrid(SAND)
        drive(g, length=3.0, y=0.4)
        n = 32  # a 1.6 m tile
        left, right = g.heights(0, 0, n), g.heights(n, 0, n)
        self.assertEqual(left.shape, (n + 1, n + 1))
        np.testing.assert_array_equal(left[n, :], right[0, :])
        self.assertAlmostEqual(float(left[20, 8]), g.depth(20 * 0.05, 8 * 0.05), places=6)
        self.assertEqual(float(g.heights(0, 64, n).max()), 0.0)
        keys = g.dirty_blocks()
        self.assertTrue(keys)
        g.handed_out(keys[:1])
        self.assertEqual(len(g.dirty_blocks()), len(keys) - 1)

    def test_a_teleport_is_not_swept(self):
        g = SoilGrid(SAND)
        g.begin_step()
        g.wheel(0, 0.0, 0.0, 0.0, 4500.0, R, B, 5.0, 5.0, 1 / 60)
        g.begin_step()
        g.wheel(0, 20.0, 0.0, 0.0, 4500.0, R, B, 5.0, 5.0, 1 / 60)
        self.assertEqual(g.depth(10.0, 0.0), 0.0)

    def test_saved_ruts_load_into_a_fresh_grid(self):
        g = SoilGrid(SAND)
        first = drive(g)
        fresh = SoilGrid(SAND)
        fresh.load(g.save())
        self.assertEqual(fresh.dirty(), {})  # loaded ruts are already in the collision
        self.assertLess(drive(fresh)["resistance"], 0.8 * first["resistance"])

    def test_a_dry_run_changes_nothing(self):
        g = SoilGrid(SAND)
        g.begin_step()
        out = g.wheel(0, 0.0, 0.0, 0.0, 4500.0, R, B, 5.0, 5.0, 1 / 60, commit=False)
        self.assertGreater(out.sinkage, 0.0)
        self.assertEqual(g.blocks, {})

    def test_too_heavy_bottoms_out(self):
        g = SoilGrid(SAND)
        g.begin_step()
        out = g.wheel(0, 0.0, 0.0, 0.0, 2e6, R, B, 0.0, 0.0, 1 / 60)
        self.assertAlmostEqual(out.sinkage, soil.MAX_SINKAGE, places=6)
        self.assertLess(out.load, 2e6)


try:
    from isaac_world import soil_warp  # noqa: F401 -- Isaac's Python has warp
    HAVE_WARP = True
except ImportError:
    HAVE_WARP = False


@unittest.skipUnless(HAVE_WARP, "warp: run under Isaac's Python")
class WarpTest(unittest.TestCase):
    """The compiled sweep is the numpy one: float32 against float64, and the wheel bottom by
    bisection against interpolated candidates (nodes within 0.2 mm, measured)."""

    def test_the_kernels_agree(self):
        for p in (SAND, LOAM):
            for slip, speed in ((0.2, 5.0), (0.1, 25.0), (-0.3, 8.0)):
                a, b = SoilGrid(p, kernel="numpy"), SoilGrid(p, kernel="warp")
                ra = drive(a, slip=slip, speed=speed, length=8.0)
                rb = drive(b, slip=slip, speed=speed, length=8.0)
                for f in ra:
                    self.assertAlmostEqual(rb[f] / ra[f], 1.0, delta=2e-3, msg=(f, slip, speed))
                self.assertEqual(set(a.blocks), set(b.blocks))
                for k in a.blocks:
                    np.testing.assert_allclose(a.blocks[k].sinkage, b.blocks[k].sinkage,
                                               atol=5e-4)

    def test_compiled_is_fast(self):
        import time
        g = SoilGrid(SAND, kernel="warp")
        drive(g, speed=25.0, length=2.0)  # compiled and warm
        t0 = time.perf_counter()
        n = 0
        x = 2.0
        while x < 20.0:
            g.begin_step()
            g.wheel(0, x, 0.013, 0.0, 4500.0, R, B, 27.7, 25.0, 1 / 60)
            x += 25.0 / 60
            n += 1
        self.assertLess((time.perf_counter() - t0) / n, 400e-6)  # 141 us measured, loaded host


class EdgeTest(unittest.TestCase):
    def test_the_soil_fades_in_from_the_zone_edge(self):
        sq = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.assertAlmostEqual(soil.edge_distance(sq, 5, 1), 1.0)
        self.assertAlmostEqual(soil.edge_weight(sq, 0.0, 5.0, 0.74), 0.0)
        self.assertAlmostEqual(soil.edge_weight(sq, 0.37, 5.0, 0.74), 0.5)
        self.assertEqual(soil.edge_weight(sq, 5.0, 5.0, 0.74), 1.0)
        self.assertEqual(soil.edge_weight(sq, 0.1, 5.0, 0.0), 1.0)


if __name__ == "__main__":
    unittest.main()
