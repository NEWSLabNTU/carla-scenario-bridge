"""Tests for scripts/scenario_vehicle.py and scripts/check_scenario_vehicles.py (roadmap 050)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_scenario_vehicles as lint  # noqa: E402
import scenario_vehicle as sv  # noqa: E402

M3 = "vehicle.tesla.model3"

OLD = """<OpenSCENARIO><Entities>
<ScenarioObject name="ego">
  <Vehicle name="vehicle.tesla.model3" vehicleCategory="car">
    <BoundingBox><Center x="1.5" y="0.0" z="0.9" /><Dimensions width="2.1" height="1.8" length="4.5" /></BoundingBox>
    <Performance maxSpeed="50" maxAcceleration="15.0" maxDeceleration="15.0" />
    <Axles>
      <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3" />
      <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3" />
    </Axles>
    <Properties />
  </Vehicle>
</ScenarioObject>
<ScenarioObject name="lead">
  <Vehicle name="lead" vehicleCategory="car">
    <BoundingBox><Center x="1.5" y="0.0" z="0.9" /><Dimensions width="2.1" height="1.8" length="4.5" /></BoundingBox>
  </Vehicle>
</ScenarioObject>
</Entities></OpenSCENARIO>
"""


def test_the_model3_is_its_measured_car():
    g = sv.geometry(M3)
    # Box centre 0.029 ahead of the actor origin, rear axle 1.386 behind it.
    assert abs(g["center_x"] - 1.415) < 1e-9
    assert abs(g["center_z"] - 0.744) < 1e-9
    assert (g["length"], g["width"], g["height"]) == (4.792, 2.163, 1.488)
    assert abs(g["front_x"] - 3.004) < 1e-9
    assert abs(g["wheel_diameter"] - 0.74) < 1e-9
    assert g["track"] == 1.667
    assert sv.geometry("not.a.blueprint") is None


def test_numbers_are_written_as_the_scenarios_write_them():
    assert [sv.fmt(v) for v in (1.415, 0.0, -0.0, 4.792, 2.1, 0.37, 3.0)] == \
        ["1.415", "0.0", "0.0", "4.792", "2.1", "0.37", "3.0"]


def test_rewrite_touches_only_the_named_vehicle_numbers():
    new, count = sv.rewrite(OLD)
    assert count == 1
    assert '<Center x="1.415" y="0.0" z="0.744" />' in new
    assert '<Dimensions width="2.163" height="1.488" length="4.792" />' in new
    assert ('<FrontAxle maxSteering="0.5" wheelDiameter="0.74" trackWidth="1.667" '
            'positionX="3.004" positionZ="0.37" />') in new
    assert ('<RearAxle maxSteering="0.0" wheelDiameter="0.74" trackWidth="1.667" '
            'positionX="0.0" positionZ="0.37" />') in new
    # The vehicle declared by size keeps its box; everything else is byte for byte.
    assert new.count('<Center x="1.5" y="0.0" z="0.9" />') == 1
    assert sv.rewrite(new) == (new, 1)
    assert len(new.splitlines()) == len(OLD.splitlines())


def test_the_lint_names_the_entity_field_and_both_values(tmp_path):
    path = tmp_path / "s.xosc"
    path.write_text(OLD)
    failures, exempt, checked = lint.check_file(path, sv.load_table(), 0.02)
    assert checked == 1 and not exempt
    assert any("ego: BoundingBox/Dimensions length declared 4.5, measured 4.792" in f
               for f in failures)
    assert not any("lead" in f for f in failures)
    path.write_text(sv.rewrite(OLD)[0])
    assert lint.check_file(path, sv.load_table(), 0.02) == ([], [], 1)


def test_an_exempt_vehicle_is_listed_not_failed(tmp_path):
    path = tmp_path / "s.xosc"
    path.write_text(OLD.replace("<Properties />",
                                '<Properties><Property name="swb.box_exempt" value="test" /></Properties>'))
    failures, exempt, checked = lint.check_file(path, sv.load_table(), 0.02)
    assert (failures, checked) == ([], 0)
    assert exempt and exempt[0].endswith(": test")
