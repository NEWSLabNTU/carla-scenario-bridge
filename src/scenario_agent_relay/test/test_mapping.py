import pytest

from scenario_agent_relay import mapping as M
from scenario_agent_relay import protocol as P


def reading(state, arrived_age=0.0):
    v = M.adapi_view(state)
    return M.concealer_reading(v.localization, v.route, v.mode, v.autoware_control_enabled,
                               v.autonomous_mode_available, arrived_age)


@pytest.mark.parametrize("phase, legacy", [
    (P.UNAVAILABLE, "INITIALIZING"),
    (P.INITIALIZING, "INITIALIZING"),
    (P.IDLE, "WAITING_FOR_ROUTE"),
    (P.PLANNING, "PLANNING"),
    (P.READY, "WAITING_FOR_ENGAGE"),
    (P.DRIVING, "DRIVING"),
    (P.ARRIVED, "ARRIVED_GOAL"),
    (P.STOPPED, "PLANNING"),
])
def test_each_phase_reads_as_the_legacy_state_the_concealer_expects(phase, legacy):
    assert reading(P.state(phase)) == legacy


def test_no_agent_reads_as_autoware_not_up():
    view = M.adapi_view(None)
    assert (view.localization, view.route, view.mode) == (M.LOC_UNKNOWN, M.ROUTE_UNKNOWN,
                                                          M.MODE_UNKNOWN)
    assert reading(None) == "INITIALIZING"
    assert view.mrm_state == M.MRM_UNKNOWN and not view.emergency


def test_arrived_goal_lasts_two_seconds_then_waiting_for_route():
    state = P.state(P.ARRIVED)
    assert reading(state, 1.9) == "ARRIVED_GOAL"
    assert reading(state, 2.1) == "WAITING_FOR_ROUTE"
    view = M.adapi_view(state)
    assert M.legacy_state(view, 1.0) == M.AW_ARRIVED_GOAL
    assert M.legacy_state(view, 3.0) == M.AW_WAITING_FOR_ROUTE


def test_engage_gating_follows_the_concealer_sequence():
    # plan() waits WAITING_FOR_ROUTE -> PLANNING -> WAITING_FOR_ENGAGE, engage() waits
    # for DRIVING; the agent's phases have to produce exactly that order.
    order = [reading(P.state(p)) for p in (P.IDLE, P.PLANNING, P.READY, P.DRIVING)]
    assert order == ["WAITING_FOR_ROUTE", "PLANNING", "WAITING_FOR_ENGAGE", "DRIVING"]


@pytest.mark.parametrize("fault, behavior, progress, mrm, mrm_behavior, emergency", [
    (P.FAULT_NONE, None, None, M.MRM_NORMAL, M.BEHAVIOR_NONE, False),
    (P.MINIMAL_RISK_MANEUVER, "COMFORTABLE_STOP", None, M.MRM_OPERATING,
     M.BEHAVIOR_COMFORTABLE_STOP, False),
    (P.MINIMAL_RISK_MANEUVER, "EMERGENCY_STOP", P.SUCCEEDED, M.MRM_SUCCEEDED,
     M.BEHAVIOR_EMERGENCY_STOP, False),
    (P.MINIMAL_RISK_MANEUVER, None, P.FAILED, M.MRM_FAILED, M.BEHAVIOR_EMERGENCY_STOP,
     False),
    (P.EMERGENCY, "EMERGENCY_STOP", None, M.MRM_OPERATING, M.BEHAVIOR_EMERGENCY_STOP, True),
])
def test_fault_mapping(fault, behavior, progress, mrm, mrm_behavior, emergency):
    view = M.adapi_view(P.state(P.DRIVING, fault=fault, fault_behavior=behavior,
                                fault_progress=progress))
    assert (view.mrm_state, view.mrm_behavior, view.emergency) == (mrm, mrm_behavior,
                                                                   emergency)


@pytest.mark.parametrize("indicators, command", [
    (None, M.TURN_NO_COMMAND), (P.INDICATORS_NONE, M.TURN_DISABLE), (P.LEFT, M.TURN_LEFT),
    (P.RIGHT, M.TURN_RIGHT), (P.HAZARD, M.TURN_DISABLE)])
def test_turn_indicators(indicators, command):
    assert M.adapi_view(P.state(P.DRIVING, turn_indicators=indicators)).turn_indicators \
        == command


def test_constants_match_the_ros_messages():
    msgs = pytest.importorskip("autoware_adapi_v1_msgs.msg")
    system = pytest.importorskip("autoware_system_msgs.msg")
    vehicle = pytest.importorskip("autoware_vehicle_msgs.msg")
    rtc = pytest.importorskip("tier4_rtc_msgs.msg")
    L, R, O, Mrm = (msgs.LocalizationInitializationState, msgs.RouteState,
                    msgs.OperationModeState, msgs.MrmState)
    assert (L.UNKNOWN, L.UNINITIALIZED, L.INITIALIZING, L.INITIALIZED) == (
        M.LOC_UNKNOWN, M.LOC_UNINITIALIZED, M.LOC_INITIALIZING, M.LOC_INITIALIZED)
    assert (R.UNKNOWN, R.UNSET, R.SET, R.ARRIVED, R.CHANGING) == (
        M.ROUTE_UNKNOWN, M.ROUTE_UNSET, M.ROUTE_SET, M.ROUTE_ARRIVED, M.ROUTE_CHANGING)
    assert (O.UNKNOWN, O.STOP, O.AUTONOMOUS, O.LOCAL, O.REMOTE) == (
        M.MODE_UNKNOWN, M.MODE_STOP, M.MODE_AUTONOMOUS, M.MODE_LOCAL, M.MODE_REMOTE)
    assert (Mrm.UNKNOWN, Mrm.NORMAL, Mrm.MRM_OPERATING, Mrm.MRM_SUCCEEDED,
            Mrm.MRM_FAILED) == (M.MRM_UNKNOWN, M.MRM_NORMAL, M.MRM_OPERATING,
                                M.MRM_SUCCEEDED, M.MRM_FAILED)
    assert (Mrm.NONE, Mrm.EMERGENCY_STOP, Mrm.COMFORTABLE_STOP, Mrm.PULL_OVER) == (
        M.BEHAVIOR_NONE, M.BEHAVIOR_EMERGENCY_STOP, M.BEHAVIOR_COMFORTABLE_STOP,
        M.BEHAVIOR_PULL_OVER)
    A = system.AutowareState
    assert (A.INITIALIZING, A.WAITING_FOR_ROUTE, A.PLANNING, A.WAITING_FOR_ENGAGE,
            A.DRIVING, A.ARRIVED_GOAL) == (M.AW_INITIALIZING, M.AW_WAITING_FOR_ROUTE,
                                           M.AW_PLANNING, M.AW_WAITING_FOR_ENGAGE,
                                           M.AW_DRIVING, M.AW_ARRIVED_GOAL)
    T = vehicle.TurnIndicatorsCommand
    assert (T.NO_COMMAND, T.DISABLE, T.ENABLE_LEFT, T.ENABLE_RIGHT) == (
        M.TURN_NO_COMMAND, M.TURN_DISABLE, M.TURN_LEFT, M.TURN_RIGHT)
    for name, value in M.RTC_MODULES.items():
        assert getattr(rtc.Module, name) == value
