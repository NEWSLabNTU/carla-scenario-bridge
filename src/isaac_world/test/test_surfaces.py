"""The surface layer's map, model and Isaac glue (roadmap 047 S1); no Isaac needed."""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaac_world import surface_layer, surfaces  # noqa: E402
from isaac_world.surfaces import RoadCondition, Surface, SurfaceError  # noqa: E402

TABLE = surfaces.load_classes()
SQUARE = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
VEHICLE_GRAPH = ((0.0, 1.0), (0.1, 1.0), (1.0, 0.8))  # aib's default


def smap(zones=(), defaults=None, **kw):
    data = {"defaults": defaults or {"Road": "asphalt", "Terrain": "grass"},
            "zones": list(zones)}
    data.update(kw)
    return surfaces.parse(data, table=TABLE)


def refused(data):
    """The refusal's message for surfaces.yaml contents that must be refused."""
    try:
        surfaces.parse(data, table=TABLE)
    except SurfaceError as e:
        return str(e)
    raise AssertionError("not refused")


class ClassTableTest(unittest.TestCase):
    def test_the_shipped_table_loads_with_its_reference(self):
        self.assertEqual(TABLE.reference, "asphalt")
        self.assertIn("mud_wet", TABLE.classes)
        self.assertEqual(TABLE["asphalt"].wet[0], 1.0)

    def test_a_missing_wetness_level_takes_the_drier_ones(self):
        t = surfaces.parse_classes({"classes": {"asphalt": {"mu": 0.8, "wet": {"moist": 0.9}}}})
        self.assertEqual(t["asphalt"].wet, (1.0, 0.9, 0.9, 0.9, 0.9))

    def test_bad_classes_are_listed_together(self):
        with self.assertRaises(SurfaceError) as cm:
            surfaces.parse_classes({"reference": "tarmac", "classes": {
                "a": {"mu": 0}, "b": {"mu": 0.5, "slip": [[0, 1], [1, 1]]},
                "c": {"mu": 0.5, "wet": {"soaked": 0.5}}}})
        msg = str(cm.exception)
        for part in ("class a", "class b: slip", "soaked", "tarmac"):
            self.assertIn(part, msg)


class MapTest(unittest.TestCase):
    def test_labels_take_their_defaults_and_unlabelled_the_reference(self):
        m = smap()
        self.assertEqual(m.at(1, 1, "Road").cls, "asphalt")
        self.assertEqual(m.at(1, 1, "Terrain").cls, "grass")
        self.assertEqual(m.at(1, 1, None).cls, "asphalt")
        self.assertEqual(m.at(1, 1, "SideWalk").cls, "asphalt")  # unset: the unlabelled one
        self.assertEqual(smap(defaults={"unlabelled": "dirt"}).at(1, 1, "Road").cls, "dirt")

    def test_a_zone_overrides_the_label_inside_it_only(self):
        m = smap([{"id": "pit", "class": "sand", "soil": True, "polygon": SQUARE}])
        s = m.at(5, 5, "Road")
        self.assertEqual((s.cls, s.zone, s.soil), ("sand", "pit", "grid"))
        self.assertEqual(m.at(11, 5, "Road"), Surface("asphalt"))

    def test_later_zones_lie_on_top_and_unset_fields_fall_through(self):
        m = smap([{"id": "gravel_pad", "class": "gravel", "wetness": "moist", "polygon": SQUARE},
                  {"id": "puddle", "water_depth": 0.004,
                   "polygon": [[2, 2], [4, 2], [4, 4], [2, 4]]}])
        s = m.at(3, 3, "Road")
        self.assertEqual((s.cls, s.wetness, s.water_depth, s.zone),
                         ("gravel", "moist", 0.004, "puddle"))
        self.assertEqual(m.at(8, 8, "Road").water_depth, None)

    def test_point_in_a_concave_polygon(self):
        u = np.array([[0, 0], [3, 0], [3, 3], [2, 3], [2, 1], [1, 1], [1, 3], [0, 3]], float)
        self.assertTrue(surfaces.inside(u, 0.5, 2.5))
        self.assertFalse(surfaces.inside(u, 1.5, 2.5))
        self.assertTrue(surfaces.inside(u, 1.5, 0.5))

    def test_problems_are_listed_together(self):
        msg = refused({"defaults": {"Road": "tarmac", "Sky": "asphalt"}, "zoness": [],
                       "zones": [{"id": "Bad-Id", "class": "mud_wet", "polygon": SQUARE},
                                 {"id": "a", "class": "mud_wet", "polygon": [[0, 0], [1, 1]]},
                                 {"id": "a", "class": "mud_wet", "polygon": SQUARE},
                                 {"id": "bow", "class": "sand",
                                  "polygon": [[0, 0], [1, 1], [1, 0], [0, 1]]},
                                 {"id": "empty", "polygon": SQUARE},
                                 {"id": "wet", "wetness": "soaked", "polygon": SQUARE},
                                 {"id": "soft", "soil": True, "water_depth": 0.1,
                                  "polygon": SQUARE},
                                 {"id": "nowhere", "class": "sand"},
                                 {"id": "mud", "class": "mud_wet", "soil": True,
                                  "polygon": SQUARE},
                                 {"id": "deep", "class": "sand", "soil": "deep",
                                  "polygon": SQUARE}]})
        for part in ("zoness", "tarmac", "'Sky'", "Bad-Id", "three or more", "used twice",
                     "crosses itself", "sets none", "soaked", "soil needs", "needs a polygon",
                     "class 'mud_wet' has no soil parameters (soil classes: dirt, sand",
                     "soil 'deep' is not one of"):
            self.assertIn(part, msg)


class WarningsTest(unittest.TestCase):
    def test_overlaps_warn_only_where_both_set_a_field(self):
        m = smap([{"id": "wet", "wetness": "moist", "polygon": SQUARE},
                  {"id": "puddle", "water_depth": 0.004, "polygon": [[2, 2], [4, 2], [4, 4]]},
                  {"id": "ice", "class": "ice", "wetness": "dry", "polygon": [[5, 5], [12, 5],
                                                                              [12, 12]]},
                  {"id": "far", "class": "sand", "polygon": [[900, 900], [901, 900],
                                                             [901, 901]]}])
        ways = [np.array([[0.0, 0.0], [100.0, 100.0]])]
        self.assertEqual(surfaces.warnings(m, ways), [
            "zones wet and ice may overlap, both setting wetness: ice's wins",
            "zone far lies outside the Lanelet2 map's extent"])


class RasterTest(unittest.TestCase):
    def test_png_rows_run_from_the_top_edge_and_scale_to_metres(self):
        from PIL import Image

        with tempfile.TemporaryDirectory() as d:
            img = np.zeros((4, 8), dtype=np.uint16)
            img[0, :] = 100  # top row: +y edge
            Image.fromarray(img).save(Path(d) / "depth.png")
            m = surfaces.parse({"zones": [{"id": "ford", "water_depth": "depth.png",
                                           "scale": 0.0001, "origin": [10.0, 20.0],
                                           "resolution": 0.5}]},
                               path=str(Path(d) / "surfaces.yaml"), table=TABLE)
        z = m.zones[0]
        self.assertEqual(z.bbox, (10.0, 14.0, 20.0, 22.0))  # the raster's extent
        self.assertAlmostEqual(m.at(12.0, 21.9).water_depth, 0.01)  # top row
        self.assertAlmostEqual(m.at(12.0, 20.1).water_depth, 0.0)  # bottom row
        self.assertIsNone(m.at(9.0, 21.0).water_depth)  # outside

    def test_bilinear_between_pixel_centres(self):
        r = surfaces.Raster(np.array([[0.0, 1.0]]), (0.0, 0.0), 1.0)
        self.assertAlmostEqual(r.sample(1.0, 0.5), 0.5)
        self.assertAlmostEqual(r.sample(0.25, 0.5), 0.0)
        self.assertEqual(r.sample(2.5, 0.5), 0.0)

    def test_a_raster_needs_its_placement(self):
        msg = refused({"zones": [{"id": "ford", "water_depth": "d.png", "polygon": SQUARE}]})
        self.assertIn("origin, resolution, scale", msg)


class FrictionTest(unittest.TestCase):
    dry = RoadCondition()

    def test_dry_reference_ground_is_one(self):
        self.assertAlmostEqual(surfaces.peak_mu(TABLE, Surface("asphalt"), self.dry, 30.0), 1.0)

    def test_classes_and_wetness_scale_relative_to_the_reference(self):
        wet = RoadCondition(wetness="wetWithPuddles")
        self.assertAlmostEqual(surfaces.peak_mu(TABLE, Surface("asphalt"), wet, 0.0), 0.75)
        self.assertAlmostEqual(surfaces.peak_mu(TABLE, Surface("ice"), self.dry, 0.0), 0.1 / 0.85)
        self.assertAlmostEqual(
            surfaces.peak_mu(TABLE, Surface("asphalt"), RoadCondition(0.5), 0.0), 0.5)
        # a zone's wetness beats the world's
        self.assertAlmostEqual(
            surfaces.peak_mu(TABLE, Surface("asphalt", wetness="dry"), wet, 0.0), 1.0)

    def test_hydroplaning_grows_with_speed_and_depth_on_paved_ground_only(self):
        hp = TABLE.hydroplaning
        vp = hp.speed
        self.assertAlmostEqual(vp, 6.35 * 250 ** 0.5 / 3.6)
        f = [surfaces.hydroplane_factor(hp, v, 0.005) for v in (0.4 * vp, 0.75 * vp, vp, 2 * vp)]
        self.assertEqual(f[0], 1.0)
        self.assertGreater(f[0], f[1])
        self.assertGreater(f[1], f[2])
        self.assertAlmostEqual(f[2], hp.residual)
        self.assertAlmostEqual(f[3], hp.residual)
        self.assertEqual(surfaces.hydroplane_factor(hp, vp, 0.0004), 1.0)
        shallow = surfaces.hydroplane_factor(hp, vp, 0.0015)
        self.assertGreater(shallow, hp.residual)
        deep = Surface("asphalt", water_depth=0.005)
        grass = Surface("grass", water_depth=0.005)
        self.assertLess(surfaces.peak_mu(TABLE, deep, self.dry, vp), 0.1)
        # a ford on grass soaks in: wet grass, no aquaplaning
        self.assertAlmostEqual(surfaces.peak_mu(TABLE, grass, self.dry, vp),
                               0.55 * 0.5 / 0.85)

    def test_standing_water_counts_as_at_least_the_level_it_reaches(self):
        s = Surface("asphalt", water_depth=0.004)  # past lowFlooded's 3 mm
        self.assertAlmostEqual(surfaces.peak_mu(TABLE, s, self.dry, 0.0), 0.65)

    def test_graph_is_linear_between_points_and_flat_past_them(self):
        g = ((0.0, 0.5), (0.2, 1.0), (1.0, 0.6))
        self.assertAlmostEqual(surfaces.graph_at(g, 0.1), 0.75)
        self.assertAlmostEqual(surfaces.graph_at(g, -0.6), 0.8)
        self.assertAlmostEqual(surfaces.graph_at(g, 3.0), 0.6)


class WheelSurfacesTest(unittest.TestCase):
    def wheels(self, m=None):
        return surfaces.WheelSurfaces(m or smap(), 1.0, VEHICLE_GRAPH)

    def test_dry_asphalt_keeps_the_rigs_tire_friction_at_any_slip(self):
        ws = surfaces.WheelSurfaces(smap(), 1.3, VEHICLE_GRAPH)
        for slip in (0.0, 0.05, -0.7, 1.0):
            self.assertAlmostEqual(ws.update(0, Surface("asphalt"), RoadCondition(), 20, slip,
                                             1 / 60), 1.3)

    def test_a_class_edge_is_followed_over_the_blend_length(self):
        ws = self.wheels()
        cond = RoadCondition()
        ws.update(0, Surface("asphalt"), cond, 10.0, 0.0, 0.01)
        target = 0.6 / 0.85  # gravel at slip where its graph is 1 and the vehicle's too:
        mu = ws.update(0, Surface("gravel"), cond, 10.0, 0.5, 0.01)  # 0.1 m: half the length
        self.assertGreater(mu, 0.0)
        raw = [ws.mu[0]]
        for _ in range(9):  # 0.9 m more: half the gap left at every 0.1 m
            ws.update(0, Surface("gravel"), cond, 10.0, 0.5, 0.01)
            raw.append(ws.mu[0])
        self.assertAlmostEqual(raw[0], 1.0 + (target - 1.0) * 0.5)
        self.assertAlmostEqual(raw[-1], target + (1.0 - target) * 0.5 ** 10)
        # standing still, nothing changes
        ws.update(0, Surface("asphalt"), cond, 0.0, 0.0, 0.01)
        self.assertAlmostEqual(ws.mu[0], raw[-1])

    def test_a_reset_snaps_and_the_air_keeps_the_last_ground(self):
        ws = self.wheels()
        ws.update(1, Surface("ice"), RoadCondition(), 5.0, 0.0, 0.01)
        held = ws.update(1, None, RoadCondition(), 5.0, 0.0, 0.01)
        self.assertAlmostEqual(held, 0.1 / 0.85 * surfaces.graph_at(TABLE["ice"].slip, 0.0))
        ws.reset()
        self.assertEqual(ws.update(1, None, RoadCondition(), 5.0, 0.0, 0.01), 1.0)
        self.assertAlmostEqual(ws.update(1, Surface("asphalt"), RoadCondition(), 5.0, 0.0,
                                         0.01), 1.0)

    def test_the_class_slip_curve_replaces_the_vehicles(self):
        ws = self.wheels()
        s, cond = Surface("gravel"), RoadCondition()
        ws.update(0, s, cond, 10.0, 0.0, 0.01)
        base = ws.mu[0]
        locked = ws.update(0, s, cond, 10.0, -0.78, 0.01)
        # PhysX applies the vehicle's graph on top: their product is gravel's own curve
        self.assertAlmostEqual(locked * surfaces.graph_at(VEHICLE_GRAPH, 0.78),
                               base * surfaces.graph_at(TABLE["gravel"].slip, 0.78))


class FakeAttr:
    def __init__(self, value):
        self.value = value

    def HasAuthoredValue(self):
        return True

    def Get(self):
        return self.value


class FakePrim:
    def __init__(self, stage, path):
        self.stage, self.path = stage, path

    def IsValid(self):
        return True

    def IsPseudoRoot(self):
        return self.path == "/"

    def GetAttribute(self, name):
        codes = self.stage.labels.get(self.path)
        return FakeAttr(codes) if codes is not None and name == surface_layer.QCODE_ATTR \
            else None

    def GetParent(self):
        return FakePrim(self.stage, self.path.rsplit("/", 1)[0] or "/")


class FakeStage:
    def __init__(self, labels):
        self.labels = labels

    def GetPrimAtPath(self, path):
        return FakePrim(self, path)


class FakeVehicle:
    path = "/World/entities/e1"
    wheel_paths = ["fl", "fr", "rl", "rr"]

    def __init__(self, contacts):
        self.p = SimpleNamespace(tire_friction=1.0, friction_vs_slip=VEHICLE_GRAPH,
                                 wheel_radius=0.37, wheel_width=0.235)
        self.contacts = contacts
        self.wheel_states = [SimpleNamespace(slip=0.0, rotation_speed=0.0)] * 4
        self.written = {}

    def set_wheel_friction(self, i, mu):
        self.written[i] = mu


class SurfaceLayerTest(unittest.TestCase):
    def test_a_soil_zone_sets_friction_and_pushes_back_fading_in_from_its_edge(self):
        stage = FakeStage({"/World/env/Road_1": ["Q34442"]})
        forces = []
        m = smap([{"id": "sand_pit", "class": "sand", "soil": "grid",
                   "polygon": [[0, -5], [40, -5], [40, 5], [0, 5]]},
                  {"id": "puddle", "water_depth": 0.004, "polygon": SQUARE}])
        layer = surface_layer.SurfaceLayer(stage, m, apply_force=lambda *f: forces.append(f))
        c = SimpleNamespace
        v = FakeVehicle([])
        speed = 5.0

        def at(x):
            v.contacts = [c(on_ground=True, position=(x, y, 0.0), shape="/World/env/Road_1/m",
                            load=4500.0, direction=(1.0, 0.0, 0.0)) for y in (-0.8, 0.8)] * 2
            v.wheel_states = [SimpleNamespace(slip=0.2, rotation_speed=speed / 0.8 / 0.37)] * 4
            forces.clear()
            layer.begin_step()
            layer.update(v, speed, 1 / 60)
            return layer.soil_under(v)[0]

        edge = at(0.05)  # under the puddle zone too: the soil is the sand zone's
        self.assertLess(edge[1], 0.1)
        x = 0.05
        while x < 20.0:
            inside = at(x)
            x += speed / 60
        self.assertEqual(inside[1], 1.0)
        self.assertEqual(len(forces), 4)
        self.assertLess(forces[0][1][0], -500.0)  # against the travel, at the contact
        self.assertEqual(forces[0][2][0], v.contacts[0].position[0])
        mu = inside[0].mu(4500.0)
        shape = surfaces.graph_at(TABLE["sand"].slip, 0.2) / surfaces.graph_at(VEHICLE_GRAPH, 0.2)
        self.assertAlmostEqual(v.written[0], mu / TABLE["asphalt"].mu * shape, delta=0.02)
        self.assertGreater(layer.grids["sand_pit"].depth(10.0, 0.8), 0.05)
        layer.fresh_soil()
        self.assertEqual(layer.grids, {})
        self.assertIsNone(at(50.0))  # off the zone: no soil

    def test_each_wheel_from_its_colliders_label_and_the_zone_under_it(self):
        stage = FakeStage({"/World/env/Road_1": ["Q34442"], "/World/env/Grass": ["Q186131"],
                           "/World/env/Building": ["Q41176"]})
        m = smap([{"id": "ice_patch", "class": "ice", "polygon": [[100, -5], [110, -5],
                                                                  [110, 5], [100, 5]]}])
        layer = surface_layer.SurfaceLayer(stage, m, map_origin=(100.0, 0.0, 0.0))
        c = SimpleNamespace
        v = FakeVehicle([c(on_ground=True, position=(5.0, 0.0, 0.0), shape="/World/env/Road_1/m"),
                         c(on_ground=True, position=(-5.0, 0.0, 0.0), shape="/World/env/Road_1/m"),
                         c(on_ground=True, position=(-5.0, 0.0, 0.0), shape="/World/env/Grass/m"),
                         c(on_ground=False, position=(0.0, 0.0, 0.0), shape="")])
        layer.update(v, 10.0, 1 / 60)
        under = layer.surfaces_under(v)
        self.assertEqual([s and s.cls for s in under], ["ice", "asphalt", "grass", None])
        self.assertAlmostEqual(v.written[1], 1.0)
        self.assertLess(v.written[0], 0.2)
        self.assertEqual(v.written[3], 1.0)
        self.assertIsNone(layer.label("/World/env/Building/wall"))  # not ground: unlabelled
        self.assertEqual(layer.label("/World/env/Road_1/m"), "Road")

    def test_register_maps_a_worlds_usd_to_its_surface_map(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "isaac").mkdir()
            (Path(d) / "isaac" / "world.yaml").write_text(
                "usd: town.usd\nmap_origin: [1.0, 2.0, 0.0]\n")
            entry = [(str(Path(d) / "surfaces.yaml"), (1.0, 2.0, 0.0))]
            # none yet: registered all the same, looked for at each LoadWorld
            got = surface_layer.register([str(Path(d) / "isaac" / "world.yaml")])
            self.assertEqual(list(got.values()), entry)
            (Path(d) / "surfaces.yaml").write_text("defaults: {Road: asphalt}\n")
            got = surface_layer.register([str(Path(d) / "isaac" / "world.yaml")])
            self.assertEqual(list(got.values()), entry)
            (Path(d) / "surfaces.yaml").write_text("defaults: {Road: tarmac}\n")
            with self.assertRaises(SurfaceError):
                surface_layer.register([str(Path(d) / "isaac" / "world.yaml")])


if __name__ == "__main__":
    unittest.main()


class RoadConditionTest(unittest.TestCase):
    """A scenario's RoadCondition on a map (047 S2)."""

    MAP = [{"id": "ford", "water_depth": 0.02, "polygon": SQUARE},
           {"id": "pit", "class": "sand", "soil": True,
            "polygon": [[20, 0], [30, 0], [30, 10], [20, 10]]}]

    def test_properties_set_zone_fields_and_leave_the_map_as_it_was(self):
        m = smap(self.MAP)
        cond, changed = surfaces.road_condition(
            m, 0.8, "moist", {"swb.zone.ford.water_depth": "0.12",
                              "swb.zone.ford.class": "gravel", "esmini.anything": "x"})
        self.assertEqual(cond, RoadCondition(0.8, "moist"))
        self.assertEqual(changed.at(5, 5, "Road"), Surface("gravel", None, 0.12, "ford"))
        # The map itself is untouched: the next condition starts from it.
        self.assertEqual(m.at(5, 5, "Road"), Surface("asphalt", None, 0.02, "ford"))
        cond, again = surfaces.road_condition(m)
        self.assertEqual(cond, RoadCondition())
        self.assertEqual(again.at(5, 5, "Road").water_depth, 0.02)

    def test_every_problem_is_listed(self):
        with self.assertRaises(SurfaceError) as e:
            surfaces.road_condition(smap(self.MAP), -1, "soaked", {
                "swb.zone.nowhere.class": "ice", "swb.zone.ford.colour": "red",
                "swb.zone.ford.water_depth": "deep", "swb.zone.ford.wetness": "damp",
                "swb.zone.ford.class": "lava", "swb.zone.pit.class": "dirt",
                "swb.ruts": "keep"})
        text = str(e.exception)
        for needle in ("wetness 'soaked'", "frictionScaleFactor -1.0",
                       "no zone 'nowhere'", "zones: ford, pit", "no field 'colour'",
                       "water depth is metres", "'damp' is not one of",
                       "unknown class 'lava'", "zone 'pit' is soil", "swb.ruts: unknown"):
            self.assertIn(needle, text)

    def test_a_world_without_a_map_takes_world_wide_state_only(self):
        empty = surfaces.parse({}, surfaces.NO_MAP)
        cond, _ = surfaces.road_condition(empty, 0.5, "lowFlooded")
        self.assertEqual(cond, RoadCondition(0.5, "lowFlooded"))
        with self.assertRaises(SurfaceError) as e:
            surfaces.road_condition(empty, 1.0, "dry", {"swb.zone.ford.water_depth": "0.1"})
        self.assertIn("in (no surfaces.yaml) (zones: none)", str(e.exception))

    def test_the_layer_applies_it_from_the_next_update(self):
        stage = FakeStage({"/World/env/Road_1": ["Q34442"]})
        layer = surface_layer.SurfaceLayer(stage, smap(self.MAP))
        c = SimpleNamespace
        on_road = [c(on_ground=True, position=(50.0, 0.0, 0.0), shape="/World/env/Road_1/m")] * 4
        v = FakeVehicle(on_road)
        layer.update(v, 10.0, 1 / 60)
        self.assertAlmostEqual(v.written[0], 1.0)
        layer.set_condition(0.5, "wetWithPuddles")
        layer.reset(v.path)
        layer.update(v, 10.0, 1 / 60)
        self.assertAlmostEqual(v.written[0], 0.5 * TABLE["asphalt"].wet[2])
        with self.assertRaises(SurfaceError):
            layer.set_condition(1.0, "dry", {"swb.zone.ford.class": "lava"})
        self.assertEqual(layer.condition, RoadCondition(0.5, "wetWithPuddles"))  # unchanged
        layer.set_condition()
        layer.reset(v.path)
        layer.update(v, 10.0, 1 / 60)
        self.assertAlmostEqual(v.written[0], 1.0)
