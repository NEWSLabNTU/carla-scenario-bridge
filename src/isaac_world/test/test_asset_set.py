"""The asset set's manifest (docs/design/isaac-assets.md): pure files, no Isaac."""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaac_world import asset_set  # noqa: E402


class AssetSetTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.root = Path(self.dir.name)
        for name, text in {"Town01/Town01.usd": "town", "Town01/Town01_physics.usda": "layer",
                           "vehicles/vehicles.json": "[]", "vehicles/a/a.usda": "car",
                           "vehicles/logs/export.log": "log", "vehicles/raw/x.usd": "raw",
                           "Town01/Town01_physics.usda.pre044": "old"}.items():
            (self.root / name).parent.mkdir(parents=True, exist_ok=True)
            (self.root / name).write_text(text)

    def tearDown(self):
        self.dir.cleanup()

    def test_manifest_lists_the_set_and_skips_logs_scratch_and_backups(self):
        n = asset_set.manifest(self.root, commit="abc1234")
        listed = asset_set.read_manifest(self.root)
        self.assertEqual(sorted(listed), ["README.md", "Town01/Town01.usd",
                                          "Town01/Town01_physics.usda", "vehicles/a/a.usda",
                                          "vehicles/vehicles.json"])
        self.assertEqual(n, 5)
        readme = (self.root / "README.md").read_text()
        self.assertIn("abc1234", readme)
        self.assertIn("Towns: Town01", readme)

    def test_a_clean_copy_checks_out(self):
        asset_set.manifest(self.root, commit="abc1234")
        self.assertEqual(asset_set.check(self.root), ([], [], []))
        self.assertEqual(asset_set.main(["check", str(self.root)]), 0)

    def test_check_names_missing_changed_and_unlisted_files(self):
        asset_set.manifest(self.root, commit="abc1234")
        (self.root / "vehicles/a/a.usda").unlink()
        (self.root / "Town01/Town01.usd").write_text("truncated")
        (self.root / "vehicles/new.usda").write_text("new")
        self.assertEqual(asset_set.check(self.root), (["vehicles/a/a.usda"],
                                                      ["Town01/Town01.usd"],
                                                      ["vehicles/new.usda"]))
        self.assertEqual(asset_set.main(["check", str(self.root)]), 1)

    def test_an_unlisted_file_alone_passes(self):
        asset_set.manifest(self.root, commit="abc1234")
        (self.root / "vehicles/new.usda").write_text("new")
        self.assertEqual(asset_set.main(["check", str(self.root)]), 0)


if __name__ == "__main__":
    unittest.main()
