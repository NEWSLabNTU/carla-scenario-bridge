"""Which body an entity wears (roadmaps 043, 044 D): pure data, no Isaac."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaac_world import vehicle_assets  # noqa: E402

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


class RelocatedSetTest(unittest.TestCase):
    """An index names its files relative to itself: a set moved elsewhere still resolves."""

    def test_index_paths_resolve_against_the_index(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / "moved" / "vehicles"
            (root / "walkers").mkdir(parents=True)
            (root / "vehicles.json").write_text(json.dumps([{
                "id": "vehicle.audi.a2", "wheels_n": 4,
                "usd": "vehicle.audi.a2/vehicle.audi.a2.usda",
                "raw_usd": "vehicle.audi.a2/vehicle_audi_a2.usd",
                "wheels": [{"name": "Wheel_Front_Left", "usd": "vehicle.audi.a2/wheels.usda"}]}]))
            (root / "walkers" / "walkers.json").write_text(json.dumps([{
                "id": "bp_walker_maleeuro", "usd": "bp_walker_maleeuro/bp_walker_maleeuro.usda",
                "cycles": {"walk": {"npz": "bp_walker_maleeuro/walk.npz"}}}]))
            assets = vehicle_assets.VehicleAssets(root)
            car = assets.get("vehicle.audi.a2")
            self.assertEqual(car["usd"], str(root / "vehicle.audi.a2/vehicle.audi.a2.usda"))
            self.assertEqual(car["raw_usd"], str(root / "vehicle.audi.a2/vehicle_audi_a2.usd"))
            self.assertEqual(car["wheels"][0]["usd"], str(root / "vehicle.audi.a2/wheels.usda"))
            walker = assets.walkers[0]
            self.assertEqual(walker["usd"],
                             str(root / "walkers/bp_walker_maleeuro/bp_walker_maleeuro.usda"))
            self.assertEqual(walker["cycles"]["walk"]["npz"],
                             str(root / "walkers/bp_walker_maleeuro/walk.npz"))

    def test_an_absolute_path_stays(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "vehicles.json").write_text(json.dumps([{"id": "a", "usd": "/old/a.usda"}]))
            self.assertEqual(vehicle_assets.VehicleAssets(d).get("a")["usd"], "/old/a.usda")


if __name__ == "__main__":
    unittest.main()
