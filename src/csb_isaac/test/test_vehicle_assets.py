"""Which body an entity wears (roadmaps 043, 044 D): pure data, no Isaac."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac import vehicle_assets  # noqa: E402

CATALOGUE = [
    {"id": "vehicle.mini.cooper_s_2021", "wheels_n": 4, "base_type": "car",
     "carla_bbox": {"center": [0, 0, 0.7], "extent": [1.9, 0.95, 0.75]},
     "render_size": [3.9, 2.0, 1.5]},
    {"id": "vehicle.carlamotors.european_hgv", "wheels_n": 4, "base_type": "truck",
     "carla_bbox": {"center": [0, 0, 1.9], "extent": [4.0, 1.4, 1.9]},
     "render_size": [8.0, 2.8, 3.8]},
    {"id": "vehicle.bh.crossbike", "wheels_n": 2, "base_type": "bicycle",
     "carla_bbox": {"center": [-0.74, 0.09, 0.18], "extent": [0.0, 0.34, 0.18]},
     "render_size": [2.21, 1.96, 2.03]},
    {"id": "vehicle.gazelle.omafiets", "wheels_n": 2, "base_type": "bicycle",
     "render_size": [1.9, 1.95, 2.4]},
    {"id": "vehicle.kawasaki.ninja", "wheels_n": 2, "base_type": "motorcycle",
     "render_size": [2.4, 1.95, 2.16]},
]
PROPS = [{"id": "static.prop.streetbarrier", "usd": "/x.usda",
          "carla_bbox": {"center": [0.0, 0.0, 0.5], "extent": [0.2, 0.9, 0.5]}}]


class AssetsTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        root = Path(self.dir.name)
        (root / "vehicles.json").write_text(json.dumps(CATALOGUE))
        (root / "props").mkdir()
        (root / "props" / "props.json").write_text(json.dumps(PROPS))
        self.assets = vehicle_assets.VehicleAssets(root)

    def tearDown(self):
        self.dir.cleanup()

    def test_a_car_box_wears_the_nearest_four_wheeler(self):
        self.assertEqual(self.assets.nearest((4.5, 2.1, 1.8))["id"], "vehicle.mini.cooper_s_2021")
        self.assertEqual(self.assets.nearest((8.0, 2.8, 3.8), "truck")["id"],
                         "vehicle.carlamotors.european_hgv")

    def test_a_two_wheeler_wears_a_two_wheeler_of_its_kind_by_length(self):
        self.assertEqual(self.assets.nearest((1.9, 0.6, 1.6), "bicycle")["id"],
                         "vehicle.gazelle.omafiets")
        self.assertEqual(self.assets.nearest((2.2, 0.6, 1.6), "bicycle")["id"],
                         "vehicle.bh.crossbike")
        self.assertEqual(self.assets.nearest((1.9, 0.8, 1.5), "motorcycle")["id"],
                         "vehicle.kawasaki.ninja")

    def test_props_by_id_and_where_they_sit(self):
        p = self.assets.prop("static.prop.streetbarrier")
        self.assertEqual(p["usd"], "/x.usda")
        self.assertIsNone(self.assets.prop("static.prop.nothing"))
        # Declared box 0.4 x 2.0 x 1.0 centred 0.5 up: the prop's box bottom on the ground.
        self.assertEqual(vehicle_assets.prop_offset(p, (0.0, 0.0, 0.5), (0.4, 2.0, 1.0)),
                         (0.0, 0.0, 0.0))
        for a, b in zip(vehicle_assets.prop_offset(p, (1.0, 0.0, 0.6), (0.4, 2.0, 1.0)),
                        (1.0, 0.0, 0.1)):
            self.assertAlmostEqual(a, b)


if __name__ == "__main__":
    unittest.main()
