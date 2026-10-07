"""Agent state (I3) -> the Autoware ADAPI values the concealer reads (I2).

Pure functions on plain values, so the mapping is testable without ROS. The constants
mirror autoware_adapi_v1_msgs, autoware_system_msgs, autoware_vehicle_msgs and
tier4_rtc_msgs; test_mapping.py checks them against the real messages when those are
importable.
"""

from dataclasses import dataclass

from . import protocol as P

# autoware_adapi_v1_msgs/LocalizationInitializationState
LOC_UNKNOWN, LOC_UNINITIALIZED, LOC_INITIALIZING, LOC_INITIALIZED = 0, 1, 2, 3
# autoware_adapi_v1_msgs/RouteState
ROUTE_UNKNOWN, ROUTE_UNSET, ROUTE_SET, ROUTE_ARRIVED, ROUTE_CHANGING = 0, 1, 2, 3, 4
# autoware_adapi_v1_msgs/OperationModeState
MODE_UNKNOWN, MODE_STOP, MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE = 0, 1, 2, 3, 4
# autoware_adapi_v1_msgs/MrmState.state
MRM_UNKNOWN, MRM_NORMAL, MRM_OPERATING, MRM_SUCCEEDED, MRM_FAILED = 0, 1, 2, 3, 4
# autoware_adapi_v1_msgs/MrmState.behavior
BEHAVIOR_UNKNOWN, BEHAVIOR_NONE, BEHAVIOR_EMERGENCY_STOP, BEHAVIOR_COMFORTABLE_STOP, \
    BEHAVIOR_PULL_OVER = 0, 1, 2, 3, 4
# autoware_system_msgs/AutowareState (the legacy /autoware/state)
(AW_INITIALIZING, AW_WAITING_FOR_ROUTE, AW_PLANNING, AW_WAITING_FOR_ENGAGE, AW_DRIVING,
 AW_ARRIVED_GOAL) = 1, 2, 3, 4, 5, 6
# autoware_vehicle_msgs/TurnIndicatorsCommand
TURN_NO_COMMAND, TURN_DISABLE, TURN_LEFT, TURN_RIGHT = 0, 1, 2, 3

# Autoware reports ARRIVED_GOAL for this long after arrival, then WAITING_FOR_ROUTE; the
# concealer applies the same window to the route state's stamp.
ARRIVED_GOAL_WINDOW = 2.0

BEHAVIORS = {
    "NONE": BEHAVIOR_NONE,
    "EMERGENCY_STOP": BEHAVIOR_EMERGENCY_STOP,
    "COMFORTABLE_STOP": BEHAVIOR_COMFORTABLE_STOP,
    "PULL_OVER": BEHAVIOR_PULL_OVER,
}

# tier4_rtc_msgs/Module.type, by name. The concealer and the agent both name modules this
# way; the protocol carries names, never Autoware's numbers.
RTC_MODULES = {
    "NONE": 0, "LANE_CHANGE_LEFT": 1, "LANE_CHANGE_RIGHT": 2, "AVOIDANCE_LEFT": 3,
    "AVOIDANCE_RIGHT": 4, "GOAL_PLANNER": 5, "START_PLANNER": 6, "TRAFFIC_LIGHT": 7,
    "INTERSECTION": 8, "INTERSECTION_OCCLUSION": 9, "CROSSWALK": 10, "BLIND_SPOT": 11,
    "DETECTION_AREA": 12, "NO_STOPPING_AREA": 13, "OCCLUSION_SPOT": 14,
    "EXT_REQUEST_LANE_CHANGE_LEFT": 15, "EXT_REQUEST_LANE_CHANGE_RIGHT": 16,
    "AVOIDANCE_BY_LC_LEFT": 17, "AVOIDANCE_BY_LC_RIGHT": 18, "ROUNDABOUT": 19,
}
RTC_MODULE_NAMES = {v: k for k, v in RTC_MODULES.items()}


@dataclass(frozen=True)
class AdapiView:
    """What the relay publishes for one agent state."""

    localization: int
    route: int
    mode: int
    autoware_control_enabled: bool
    autonomous_mode_available: bool
    stop_mode_available: bool
    mrm_state: int
    mrm_behavior: int
    emergency: bool
    turn_indicators: int
    legacy: int  # /autoware/state, outside the ARRIVED_GOAL window


# phase -> (localization, route, mode, control enabled, autonomous available, legacy)
_PHASES = {
    P.UNAVAILABLE: (LOC_UNKNOWN, ROUTE_UNKNOWN, MODE_UNKNOWN, False, False, AW_INITIALIZING),
    P.INITIALIZING: (LOC_INITIALIZING, ROUTE_UNSET, MODE_STOP, False, False,
                     AW_INITIALIZING),
    P.IDLE: (LOC_INITIALIZED, ROUTE_UNSET, MODE_STOP, False, False, AW_WAITING_FOR_ROUTE),
    P.PLANNING: (LOC_INITIALIZED, ROUTE_SET, MODE_STOP, False, False, AW_PLANNING),
    P.READY: (LOC_INITIALIZED, ROUTE_SET, MODE_STOP, False, True, AW_WAITING_FOR_ENGAGE),
    P.DRIVING: (LOC_INITIALIZED, ROUTE_SET, MODE_AUTONOMOUS, True, True, AW_DRIVING),
    P.ARRIVED: (LOC_INITIALIZED, ROUTE_ARRIVED, MODE_AUTONOMOUS, True, True,
                AW_ARRIVED_GOAL),
    # A route is held but the autopilot was told to stop: not engageable until a new
    # goal, so it must not read as WAITING_FOR_ENGAGE.
    P.STOPPED: (LOC_INITIALIZED, ROUTE_SET, MODE_STOP, False, False, AW_PLANNING),
}


def adapi_view(state) -> AdapiView:
    """Map an agent state message (or None: no agent, or a silent one) to ADAPI values."""
    phase = P.UNAVAILABLE if state is None else state.get("phase", P.UNAVAILABLE)
    loc, route, mode, control, available, legacy = _PHASES[phase]

    fault = P.FAULT_NONE if state is None else (state.get("fault") or P.FAULT_NONE)
    behavior_name = None if state is None else state.get("fault_behavior")
    progress = None if state is None else state.get("fault_progress")
    if phase == P.UNAVAILABLE:
        mrm_state, mrm_behavior = MRM_UNKNOWN, BEHAVIOR_UNKNOWN
    elif fault == P.FAULT_NONE:
        mrm_state, mrm_behavior = MRM_NORMAL, BEHAVIOR_NONE
    else:
        mrm_state = {P.SUCCEEDED: MRM_SUCCEEDED, P.FAILED: MRM_FAILED}.get(
            progress, MRM_OPERATING)
        mrm_behavior = BEHAVIORS.get(behavior_name or "", BEHAVIOR_EMERGENCY_STOP)

    indicators = None if state is None else state.get("turn_indicators")
    turn = {None: TURN_NO_COMMAND, P.INDICATORS_NONE: TURN_DISABLE, P.LEFT: TURN_LEFT,
            P.RIGHT: TURN_RIGHT, P.HAZARD: TURN_DISABLE}[indicators]

    return AdapiView(
        localization=loc, route=route, mode=mode, autoware_control_enabled=control,
        autonomous_mode_available=available, stop_mode_available=phase != P.UNAVAILABLE,
        mrm_state=mrm_state, mrm_behavior=mrm_behavior, emergency=fault == P.EMERGENCY,
        turn_indicators=turn, legacy=legacy)


def legacy_state(view: AdapiView, arrived_age: float) -> int:
    """/autoware/state: ARRIVED_GOAL for ARRIVED_GOAL_WINDOW, then WAITING_FOR_ROUTE."""
    if view.legacy == AW_ARRIVED_GOAL and arrived_age >= ARRIVED_GOAL_WINDOW:
        return AW_WAITING_FOR_ROUTE
    return view.legacy


def concealer_reading(localization, route, mode, control_enabled, autonomous_available,
                      arrived_age):
    """The concealer's LegacyAutowareState, transcribed from legacy_autoware_state.hpp.

    Used by the tests (and logs) to show what the concealer will make of the relay's
    topics; it must stay a faithful copy of that function.
    """
    if localization == LOC_UNKNOWN or route == ROUTE_UNKNOWN or mode == MODE_UNKNOWN:
        return "INITIALIZING"
    if localization in (LOC_UNINITIALIZED, LOC_INITIALIZING):
        return "INITIALIZING"
    if localization != LOC_INITIALIZED:
        return ""
    if route == ROUTE_ARRIVED:
        if arrived_age < ARRIVED_GOAL_WINDOW:
            return "ARRIVED_GOAL"
        return "WAITING_FOR_ROUTE"
    if route == ROUTE_UNSET:
        return "WAITING_FOR_ROUTE"
    if route in (ROUTE_SET, ROUTE_CHANGING):
        if mode in (MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE) and control_enabled:
            return "DRIVING"
        if mode in (MODE_AUTONOMOUS, MODE_LOCAL, MODE_REMOTE, MODE_STOP):
            return "WAITING_FOR_ENGAGE" if autonomous_available else "PLANNING"
    return ""
