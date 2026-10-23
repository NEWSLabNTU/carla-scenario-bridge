"""Tests for scripts/catalog_scenarios.py and the catalog half of
scripts/check_scenario_vehicles.py (roadmap 051)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import catalog_scenarios as cs  # noqa: E402
import check_scenario_vehicles as lint  # noqa: E402
import scenario_vehicle as sv  # noqa: E402
from test_scenario_vehicle import OLD  # noqa: E402

SCENARIO = OLD.replace("<OpenSCENARIO><Entities>",
                       "<OpenSCENARIO><CatalogLocations />\n<Entities>")


def check(tmp_path, text):
    path = tmp_path / "s.xosc"
    path.write_text(text)
    return lint.check_file(path, sv.load_table(), 0.02, lint.catalog_parameters(), cs.entries())


def test_the_catalog_has_the_model3():
    assert cs.entries() == {"vehicle.tesla.model3": "tesla_model3"}
    assert lint.catalog_parameters() == {"tesla_model3": {p for p, _, _ in cs.ASSIGNED}}


def test_rewrite_makes_a_reference_assigning_the_scenarios_values():
    new, count = cs.rewrite(SCENARIO)
    assert count == 1
    assert '<CatalogReference catalogName="vehicles" entryName="tesla_model3">' in new
    for param, value in (("maxSpeed", "50"), ("maxAcceleration", "15.0"),
                         ("maxDeceleration", "15.0"), ("maxSteering", "0.5")):
        assert f'<ParameterAssignment parameterRef="{param}" value="{value}" />' in new
    assert f'<Directory path="{cs.CATALOG_DIR}" />' in new
    # The vehicle that is not the Model 3 stays inline; a second rewrite changes nothing.
    assert '<Vehicle name="lead"' in new and 'name="vehicle.tesla.model3"' not in new
    assert cs.rewrite(new) == (new, 0)


def test_an_exempt_body_stays_inline():
    text = SCENARIO.replace("<Properties />", '<Properties><Property name="swb.box_exempt" '
                            'value="test" /></Properties>')
    assert cs.rewrite(text) == (text, 0)


def test_the_lint_refuses_an_inline_cataloged_body(tmp_path):
    failures, _, _ = check(tmp_path, SCENARIO)
    assert any("ego: an inline vehicle.tesla.model3; refer to the catalog" in f for f in failures)
    by_model3d = SCENARIO.replace('<Vehicle name="lead" vehicleCategory="car">',
                                  '<Vehicle name="lead" vehicleCategory="car" '
                                  'model3d="vehicle.tesla.model3">')
    failures, _, _ = check(tmp_path, by_model3d)
    assert any("lead: an inline vehicle.tesla.model3" in f for f in failures)


def test_the_lint_passes_a_reference_and_refuses_a_bad_one(tmp_path):
    good = cs.rewrite(SCENARIO)[0]
    assert check(tmp_path, good)[0] == []
    missing = good.replace('<ParameterAssignment parameterRef="maxSteering" value="0.5" />', "")
    assert any("needs ['maxSteering'] assigned" in f for f in check(tmp_path, missing)[0])
    unknown = good.replace('parameterRef="maxSteering"', 'parameterRef="colour"')
    assert any("has no parameter 'colour'" in f for f in check(tmp_path, unknown)[0])
    entry = good.replace('entryName="tesla_model3"', 'entryName="tesla_model4"')
    assert any("no catalog entry 'tesla_model4'" in f for f in check(tmp_path, entry)[0])
    location = good.replace(cs.CATALOG_DIR, "/somewhere/else")
    assert any("VehicleCatalog is ['/somewhere/else']" in f for f in check(tmp_path, location)[0])
