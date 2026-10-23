"""soil_tiles' plates (docs/issues/001): the plane a wheel's plate takes from the ruts under its
tire. The module needs pxr and aib (Isaac's); this stubs both, so only the numpy runs."""
import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def load():
    pxr = types.ModuleType("pxr")
    for name in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdPhysics", "Vt"):
        setattr(pxr, name, mock.MagicMock())
    vehicle = types.ModuleType("aib.isaac.vehicle")
    vehicle.GROUPS = "/World/vehicleCollisionGroups"
    stubs = {"pxr": pxr, "aib": types.ModuleType("aib"),
             "aib.isaac": types.ModuleType("aib.isaac"), "aib.isaac.vehicle": vehicle}
    with mock.patch.dict(sys.modules, stubs):
        sys.modules.pop("isaac_world.soil_tiles", None)
        module = importlib.import_module("isaac_world.soil_tiles")
        sys.modules.pop("isaac_world.soil_tiles", None)
    return module


class FootprintTest(unittest.TestCase):
    """A tire on a rut sits in it; half over its edge, half in and tilted toward it; beside it,
    flat. The plane is the least-squares one over the footprint."""

    @classmethod
    def setUpClass(cls):
        cls.st = load()

    def tiles(self):
        t = self.st.SoilTiles.__new__(self.st.SoilTiles)
        t.n, t.h, t.tiles_set = 32, 0.05, {(0, 0)}
        t.zone = types.SimpleNamespace(ground_z=1.0)
        t.origin, t.box = (0.0, 0.0, 0.0), (0.0, 1.6, 0.0, 1.6)
        ys = np.linspace(0.0, 1.6, 33)
        depth = np.zeros((33, 33))
        depth[:, np.abs(ys - 0.8) < 0.125] = 0.08  # an 8 cm rut along x at y = 0.8
        t.shown = {(0, 0): depth}
        return t

    def test_on_the_edge_and_beside_a_rut(self):
        t = self.tiles()
        z, gx, gy = t.footprint(0.8, 0.8, 1.0, 0.0, 0.235)
        self.assertLess(z, 1.0 - 0.06)
        self.assertAlmostEqual(gy, 0.0)
        z, gx, gy = t.footprint(0.8, 0.9, 1.0, 0.0, 0.235)  # the left half over the rut
        self.assertTrue(1.0 - 0.06 < z < 1.0 - 0.02)
        self.assertGreater(gy, 0.2)  # rising away from the rut: the tire leans into it
        self.assertAlmostEqual(gx, 0.0)
        self.assertEqual(t.footprint(0.8, 1.1, 1.0, 0.0, 0.235), (1.0, 0.0, 0.0))
        self.assertIsNone(t.footprint(5.0, 5.0, 1.0, 0.0, 0.235))  # off the tiles
        # just past the box: still a plate, flat (the ray must not find the box's hole)
        self.assertEqual(t.footprint(1.9, 0.8, 1.0, 0.0, 0.235), (1.0, 0.0, 0.0))
        self.assertIsNone(t.footprint(2.1, 0.8, 1.0, 0.0, 0.235))

    def test_the_closed_form_is_least_squares(self):
        t = self.tiles()
        hx, hy = np.cos(0.3), np.sin(0.3)
        z, gx, gy = t.footprint(0.8, 0.88, hx, hy, 0.235)
        a, c = self.st.FOOTPRINT[0], self.st.FOOTPRINT[1] * 0.235
        px, py = 0.8 + hx * a - hy * c, 0.88 + hy * a + hx * c
        zs = 1.0 - t.depths(px, py)
        sol = np.linalg.lstsq(np.c_[px - 0.8, py - 0.88, np.ones_like(px)], zs, rcond=None)[0]
        np.testing.assert_allclose((gx, gy, z), sol, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
