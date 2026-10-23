import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isaac_world import sky  # noqa: E402
from isaac_world.sky import Sky  # noqa: E402


def deg(x):
    return math.degrees(x)


class SunTest(unittest.TestCase):
    """The same cases as swb_core::environment's: both servers put the sun alike."""

    def test_the_sun_stands_where_it_should(self):
        az, el = sky.sun_position(12.0, 172, 0.0)
        self.assertAlmostEqual(deg(el), 66.56, delta=0.1)
        self.assertTrue(deg(az) < 0.5 or deg(az) > 359.5, deg(az))
        az, el = sky.sun_position(6.0, 81, 0.0)
        self.assertAlmostEqual(deg(el), 0.0, delta=1.0)
        self.assertAlmostEqual(deg(az), 90.0, delta=1.0)
        az, el = sky.sun_position(12.0, 81, 45.0)
        self.assertAlmostEqual(deg(el), 45.0, delta=1.0)
        self.assertAlmostEqual(deg(az), 180.0, delta=0.5)
        self.assertAlmostEqual(deg(sky.sun_position(18.0, 81, 0.0)[0]), 270.0, delta=1.0)
        self.assertLess(sky.sun_position(0.0, 81, 45.0)[1], 0.0)

    def test_an_animated_time_runs_on(self):
        t = {"hours": 23.5, "day_of_year": 10, "animation": True}
        self.assertEqual(sky.hours_after(t, 3600.0), (0.5, 11))
        self.assertEqual(sky.hours_after({**t, "animation": False}, 3600.0), (23.5, 10))

    def test_toward_the_sun_in_the_map_frame(self):
        x, y, z = sky.toward_sun(math.pi / 2, 0.0)  # east, on the horizon
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(z, 0.0)
        x, y, z = sky.toward_sun(0.0, math.pi / 4)  # north, 45 up
        self.assertAlmostEqual(y, z)

    def test_the_map_latitude(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(sky.map_latitude(d), 0.0)
            (Path(d) / "map_projector_info.yaml").write_text(
                "projector_type: MGRS\nmap_origin:\n  latitude: 35.68\n  longitude: 139.7\n")
            self.assertEqual(sky.map_latitude(d), 35.68)


class SkyTest(unittest.TestCase):
    def test_parts_not_given_stay(self):
        s = Sky().with_({"fog_visual_range": 80.0, "precipitation": ("rain", 10.0)})
        self.assertEqual(s.changed, ("rain", "fog"))
        s = s.with_({"cloud_cover_oktas": 4})
        self.assertEqual((s.fog_visual_range, s.rain_rate, s.cloud_oktas), (80.0, 10.0, 4))
        s = s.with_({"precipitation": ("dry", 0.0)})
        self.assertEqual(s.rain_rate, 0.0)
        self.assertFalse(s.is_default())
        self.assertTrue(Sky().is_default())

    def test_snow_is_refused_and_nothing_changes(self):
        s = Sky().with_({"fog_visual_range": 80.0})
        with self.assertRaises(ValueError) as e:
            s.with_({"precipitation": ("snow", 2.0), "cloud_cover_oktas": 8})
        self.assertIn("snow", str(e.exception))
        self.assertIsNone(s.cloud_oktas)

    def test_the_lights_follow_the_time_and_a_placed_sun_wins(self):
        noon = {"hours": 12.0, "day_of_year": 81, "animation": False, "date_time": ""}
        direction, sun_f, dome_f = Sky().with_(time_of_day=noon).lights(0.0)
        self.assertAlmostEqual(direction[2], 1.0, places=2)  # overhead at the equator
        self.assertEqual((sun_f, dome_f), (1.0, 1.0))
        night = {**noon, "hours": 23.0}
        _, sun_f, dome_f = Sky().with_(time_of_day=night).lights(0.0)
        self.assertEqual((sun_f, dome_f), (0.0, sky.NIGHT_DOME))
        low = Sky().with_({"sun": (math.pi, math.radians(10.0), 0.0)}, time_of_day=noon)
        direction, sun_f, _ = low.lights(0.0)
        self.assertLess(direction[1], 0.0)  # south
        self.assertAlmostEqual(sun_f, math.sin(math.radians(10)) / 0.5)
        # An animated time moves the sun with the clock.
        dawn = {**noon, "hours": 6.0, "animation": True}
        s = Sky().with_(time_of_day=dawn, now=100.0)
        self.assertLess(s.lights(0.0, now=100.0)[1], s.lights(0.0, now=100.0 + 3 * 3600)[1])

    def test_clouds_dim_the_sun_and_nine_oktas_hide_it(self):
        self.assertAlmostEqual(Sky().with_({"cloud_cover_oktas": 8}).lights(0.0)[1], 0.2)
        self.assertEqual(Sky().with_({"cloud_cover_oktas": 9}).lights(0.0)[1], 0.0)
        self.assertIsNone(Sky().with_({"fog_visual_range": 50.0}).lights(0.0))

    def test_fog(self):
        self.assertIsNone(Sky().fog_settings())
        self.assertIsNone(Sky().with_({"fog_visual_range": 100000.0}).fog_settings())
        f = Sky().with_({"fog_visual_range": 60.0}).fog_settings()
        self.assertEqual(f["/rtx/fog/fogEndDist"], 60.0)
        self.assertTrue(f["/rtx/fog/enabled"])


if __name__ == "__main__":
    unittest.main()
