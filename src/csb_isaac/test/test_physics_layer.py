"""The collider layer over a town's export (roadmap 043), on a small synthetic stage.

Needs pxr, which Isaac's venv has and the system Python does not (skipped there):

    source src/csb_isaac/scripts/isaac_env.sh && python -m unittest discover -s src/csb_isaac/test
"""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac import physics_layer  # noqa: E402

try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None


class SkippedTest(unittest.TestCase):
    def test_scenery_by_path_component(self):
        self.assertTrue(physics_layer.skipped("/Root/Lake/SM_TileLake3/Section0"))
        self.assertTrue(physics_layer.skipped("/Root/BackgroundPlanes_Tow01_2/x"))
        self.assertTrue(physics_layer.skipped("/Root/SM_sea2M/SM_sea2M"))
        self.assertFalse(physics_layer.skipped("/Root/Buildings/Bl_Lakeside_House/Section0"))
        self.assertFalse(physics_layer.skipped("/Root/Roads/Road_12/Section0"))


@unittest.skipIf(Usd is None, "no pxr (run in Isaac's venv)")
class LayerTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.src = os.path.join(self.dir.name, "Town99.usd")
        stage = Usd.Stage.CreateNew(self.src)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(stage, "/Root")
        stage.SetDefaultPrim(root.GetPrim())
        for path in ("/Root/Road/Section0", "/Root/Buildings/House/Section0",
                     "/Root/Lake/SM_TileLake/Section0", "/Root/BackgroundPlanes_T/Plane",
                     "/Root/Foliage/Grass/Blade"):
            UsdGeom.Mesh.Define(stage, path)
        UsdGeom.PointInstancer.Define(stage, "/Root/Foliage")
        UsdGeom.Mesh.Define(stage, "/Root/Crosswalk/Plane").MakeInvisible()
        stage.GetRootLayer().Save()

    def tearDown(self):
        self.dir.cleanup()

    def test_colliders_on_meshes_not_on_scenery_or_foliage(self):
        out = os.path.join(self.dir.name, "Town99_physics.usda")
        self.assertEqual(physics_layer.write_layer(self.src, out), (2, 1))
        stage = Usd.Stage.Open(out)
        self.assertEqual(str(stage.GetDefaultPrim().GetPath()), "/Root")
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
        with_collider = sorted(str(p.GetPath()) for p in stage.Traverse()
                               if p.HasAPI(UsdPhysics.CollisionAPI))
        self.assertEqual(with_collider, ["/Root/Buildings/House/Section0", "/Root/Road/Section0"])
        approx = UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath("/Root/Road/Section0"))
        self.assertEqual(approx.GetApproximationAttr().Get(), UsdPhysics.Tokens.none)
        # A layer over the export, by a relative path: nothing copied.
        layer = stage.GetRootLayer()
        self.assertEqual(list(layer.subLayerPaths), ["./Town99.usd"])
        self.assertEqual(layer.GetPrimAtPath("/Root/Road/Section0").specifier, Sdf.SpecifierOver)

    def test_main_writes_beside_the_export(self):
        physics_layer.main([self.src])
        self.assertTrue(os.path.exists(os.path.join(self.dir.name, "Town99_physics.usda")))


if __name__ == "__main__":
    unittest.main()
