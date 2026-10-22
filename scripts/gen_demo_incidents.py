#!/usr/bin/env python3
"""Generate the demo tour's incident templates (roadmap 033 step 4).

Each incident is a short standalone OpenSCENARIO file on Town01: the ego (Autoware, through
the agent relay) starts on the tour's route C far enough before the incident's trigger to
arrive the way the tour brings it there (at the 40 km/h cruise, or out of the corner before
it), and is routed through the site to a nearby goal. Timings are for the tour's ego stack:
`just ego-av max_velocity:=11.11 lidar_detection_model:=centerpoint` (step 4d). The incident is triggered by the ego's arrival
(`ReachPositionCondition`), its actors are added then (`AddEntityAction`) and deleted when it
is over (`DeleteEntityAction`), and a per-incident watchdog removes them if the ego has stood
still too long while the incident is active (logged as degraded). The only stop trigger is
the ego's goal (exitSuccess) or a timeout (exitFailure); a contact is logged, never fatal.

Rules the templates keep (033 step 2 spike, step 4b):

  - Every actor has a name of its own (`i1_lead`, `i2_ped`, ...): SSv2 never re-adds a name.
  - Private actions in an event apply to every actor of its ManeuverGroup, so actors that
    move differently get groups of their own. A `StoryboardElementStateCondition` only names
    an event of its own group (a bare name from another group fails the load); groups are
    tied together by entity conditions (the ego's position) and by global parameters
    (`ParameterSetAction` / `ParameterCondition`), which every group sees.
  - "Has not happened yet" is a parameter, never `state="standbyState"`: SSv2 keeps that
    condition true for good once evaluated against an event that started in standby.
  - "The ego stood still N s" is a chain of delayed SpeedConditions, not
    StandStillCondition, which SSv2 counts only while |v| <= DBL_EPSILON (never, for a
    CARLA-physics ego).
  - No entity is deleted while an action of its own still runs (a trajectory, a speed
    change): SSv2 then ends the scenario with "Entity ... does not exist".
  - Logging is `CustomCommandAction type="test(<incident>, <what>[, <actor>])"`: SSv2's
    TestCommand prints its parameters to the interpreter's log; `/simulation/context` shows
    the event's state change with its simulation time.

`incident_groups(n, sites)` returns one incident's entities and ManeuverGroups, so the tour
(step 5) composes the same pieces under one route.

Usage:
    scripts/gen_demo_incidents.py                 # writes src/swb_examples/scenarios/demo/incidents/
    scripts/gen_demo_incidents.py --out /tmp/x    # elsewhere
"""

import argparse
import math
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
SITES = ROOT / "scripts/demo_town01_sites.yaml"
DEFAULT_OUT = ROOT / "src/swb_examples/scenarios/demo/incidents"

TIMEOUT_S = 300  # sim time; every template's drive is well under 2 min
GOAL_TOLERANCE_M = 3.0
# The ego's Performance maxSpeed: above Autoware's max_vel (11.11 on the tour), since SSv2 ends
# a run whose ego exceeds it and the CARLA ego overshoots the cap by up to 0.4 m/s (step 4d);
# SSv2 also hands it to Autoware as the external velocity limit, which max_vel then clamps.
EGO_MAX_SPEED = 13.89
WATCHDOG_S = 20.0  # ego stood still this long while an incident is active -> clear it
I4_CLEAR_S = 12.0  # incident 4's staged "tow truck": the watchdog, on purpose
I3_GREEN_AFTER_STOP_S = 8.0
I3_GREEN_FALLBACK_S = 45.0
# A walker starts walking at this rate, m/s2, never with a step: SSv2 otherwise sets it to
# full pace in one frame (unbounded acceleration) and CARLA's walker body, which ramps from
# rest at ~1.6-4 m/s2, trails it 0.27 m for 7 frames -- a G3 that is the scenario's, not
# the bridge's (step 4c; 032 N6's kind). About a person's own start (~1 s to 1.5 m/s).
WALKER_START_RATE = 1.5

FILES = {
    1: "town01_i1_lead_brake.xosc",
    2: "town01_i2_ped_dart.xosc",
    3: "town01_i3_red_light.xosc",
    4: "town01_i4_blocked_lane.xosc",
    5: "town01_i5_cross_turn.xosc",
    6: "town01_i6_cyclist_inside.xosc",
}
# Incident 6 is in the set again (2026-10-18): it needs a classifying perception (CenterPoint)
# and the 042 overlay's blind_spot fix. scenarios/demo/town01_i6_cyclist_inside.xosc is the
# step 4b version, frozen as 042's reproducer of the stock module's right-turn bug.

# CARLA 0.9.16's boxes (config/blueprint_boxes_0.9.16.yaml): length, width, height
BLUEPRINT_BOX = {
    "vehicle.tesla.model3": (4.792, 2.163, 1.488),
    "vehicle.mercedes.sprinter": (5.915, 1.988, 2.561),
}

# ---------------------------------------------------------------------------- entities

CAR = """<Vehicle name="{bp}" vehicleCategory="car">
				<BoundingBox><Center x="1.5" y="0.0" z="0.9" /><Dimensions width="2.1" height="1.8" length="4.5" /></BoundingBox>
				<Performance maxSpeed="{vmax}" maxAcceleration="15.0" maxDeceleration="15.0" />
				<Axles>
					<FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3" />
					<RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3" />
				</Axles>
				<Properties />
			</Vehicle>"""

# vehicle.diamondback.century's box (022 picks it for a bicycle of this size).
BICYCLE = """<Vehicle name="{bp}" vehicleCategory="bicycle">
				<BoundingBox><Center x="0.0" y="0.0" z="0.81" /><Dimensions width="0.6" height="1.62" length="1.66" /></BoundingBox>
				<Performance maxSpeed="12" maxAcceleration="4.0" maxDeceleration="8.0" />
				<Axles>
					<FrontAxle maxSteering="0.5" wheelDiameter="0.66" trackWidth="0.1" positionX="0.5" positionZ="0.33" />
					<RearAxle maxSteering="0.0" wheelDiameter="0.66" trackWidth="0.1" positionX="-0.5" positionZ="0.33" />
				</Axles>
				<Properties />
			</Vehicle>"""

WALKER = """<Pedestrian model="unspecified" mass="70.0" name="walker.pedestrian.0001" pedestrianCategory="pedestrian">
				<BoundingBox><Center x="0.0" y="0.0" z="0.9" /><Dimensions width="0.6" length="0.6" height="1.8" /></BoundingBox>
				<Properties />
			</Pedestrian>"""


def ego_object():
    return f"""<ScenarioObject name="ego">
			{CAR.format(bp="vehicle.tesla.model3", vmax=EGO_MAX_SPEED)}
			<ObjectController><Controller name="Autoware"><Properties>
				<Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />
			</Properties></Controller></ObjectController>
		</ScenarioObject>"""


def npc_object(name, body, props=None):
    """`props`: controller properties (SSv2 reads e.g. isBlind, maxSpeed from them)"""
    ps = "".join(f'<Property name="{k}" value="{v}" />' for k, v in (props or {}).items())
    return f"""<ScenarioObject name="{name}">
			{body}
			<ObjectController><Controller name=""><Properties>{ps}</Properties></Controller></ObjectController>
		</ScenarioObject>"""


def car(name, props=None):
    return npc_object(name, CAR.format(bp=name, vmax=20), props)


def bicycle(name):
    return npc_object(name, BICYCLE.format(bp=name))


def walker(name):
    return npc_object(name, WALKER)


def parked(name, bp):
    """a parked vehicle as a MiscObject (no behaviour; the bridge spawns blueprint `bp` with
    physics off and holds it where SSv2 puts it). Not a vehicle NPC: standing vehicle entities
    off the lane cost SSv2 ~0.1 s a frame each (step 4d: 17 took it to RTF 0.02). Its origin
    is its box centre, as for every misc object."""
    length, width, height = BLUEPRINT_BOX[bp]
    return f"""<ScenarioObject name="{name}">
			<MiscObject mass="1800" miscObjectCategory="obstacle" name="{bp}" model3d="{bp}">
				<BoundingBox><Center x="0.0" y="0.0" z="{height / 2:.3f}" /><Dimensions width="{width}" height="{height}" length="{length}" /></BoundingBox>
				<Properties />
			</MiscObject>
		</ScenarioObject>"""


# ---------------------------------------------------------------------------- positions


def world(p, z=0.0, yaw=None):
    h = p["yaw"] if yaw is None else yaw
    return f'<WorldPosition x="{p["x"]}" y="{p["y"]}" z="{z:.2f}" h="{h:.4f}" p="0" r="0" />'


def dist(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


# ---------------------------------------------------------------------------- conditions


def reach(name, entity, p, tol):
    return f"""<Condition name="{name}" delay="0" conditionEdge="none"><ByEntityCondition>
							<TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{entity}" /></TriggeringEntities>
							<EntityCondition><ReachPositionCondition tolerance="{tol}"><Position>{world(p)}</Position></ReachPositionCondition></EntityCondition>
						</ByEntityCondition></Condition>"""


STILL_MPS = 0.1
STILL_TAP_S = 2.0


def standstill(name, entity, duration, tap=STILL_TAP_S):
    """`entity` has stood still (< STILL_MPS) for `duration` s, sampled every `tap` s: one
    SpeedCondition per tap, each `delay`ed by its tap (SSv2's delay reads the condition's value
    that many seconds ago). Not StandStillCondition: SSv2 counts standstill only while
    |v| <= DBL_EPSILON, which a CARLA-physics ego never reports (step 4b: never true in ~25 s
    stopped), so it would never fire."""
    n = max(1, int(round(duration / tap)))
    return [f"""<Condition name="{name}_{k}" delay="{k * duration / n:.2f}" conditionEdge="none"><ByEntityCondition>
							<TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{entity}" /></TriggeringEntities>
							<EntityCondition><SpeedCondition value="{STILL_MPS}" rule="lessThan" /></EntityCondition>
						</ByEntityCondition></Condition>""" for k in range(n + 1)]


def gap_below(name, entity, other, metres):
    """freespace (box to box) distance between two entities below `metres`"""
    return f"""<Condition name="{name}" delay="0" conditionEdge="none"><ByEntityCondition>
							<TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{entity}" /></TriggeringEntities>
							<EntityCondition><RelativeDistanceCondition entityRef="{other}" freespace="true" relativeDistanceType="euclidianDistance" coordinateSystem="entity" rule="lessThan" value="{metres}" /></EntityCondition>
						</ByEntityCondition></Condition>"""


def collision(name, entity, other):
    return f"""<Condition name="{name}" delay="0" conditionEdge="none"><ByEntityCondition>
							<TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{entity}" /></TriggeringEntities>
							<EntityCondition><CollisionCondition><EntityRef entityRef="{other}" /></CollisionCondition></EntityCondition>
						</ByEntityCondition></Condition>"""


def state(name, event, st="completeState", delay=0):
    return f"""<Condition name="{name}" delay="{delay}" conditionEdge="none"><ByValueCondition>
							<StoryboardElementStateCondition storyboardElementType="event" storyboardElementRef="{event}" state="{st}" />
						</ByValueCondition></Condition>"""


def param_is(name, param, value):
    """a global parameter's current value (not sticky: unlike a StoryboardElementStateCondition)"""
    return f'<Condition name="{name}" delay="0" conditionEdge="none"><ByValueCondition><ParameterCondition parameterRef="{param}" value="{value}" rule="equalTo" /></ByValueCondition></Condition>'


def simtime(name, t):
    return f'<Condition name="{name}" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{t}" rule="greaterThan" /></ByValueCondition></Condition>'


# ---------------------------------------------------------------------------- actions


def add(entity, p, z=0.0):
    return f'<Action name="add_{entity}"><GlobalAction><EntityAction entityRef="{entity}"><AddEntityAction><Position>{world(p, z)}</Position></AddEntityAction></EntityAction></GlobalAction></Action>'


def delete(entity):
    return f'<Action name="delete_{entity}"><GlobalAction><EntityAction entityRef="{entity}"><DeleteEntityAction /></EntityAction></GlobalAction></Action>'


def speed(name, v, rate=None):
    dyn = ('<SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time" />' if rate is None
           else f'<SpeedActionDynamics dynamicsShape="linear" value="{rate}" dynamicsDimension="rate" />')
    return f"""<Action name="{name}"><PrivateAction><LongitudinalAction><SpeedAction>
							{dyn}
							<SpeedActionTarget><AbsoluteTargetSpeed value="{v}" /></SpeedActionTarget>
						</SpeedAction></LongitudinalAction></PrivateAction></Action>"""


def speed_profile(name, v, accel, decel, decel_rate, accel_rate=10.0, vmax=20.0):
    """a speed change under the given dynamic constraints. A SpeedAction's `rate` sets only
    the acceleration: SSv2 ramps to it at the default 3 m/s3 jerk (`max_deceleration_rate`),
    and resets the entity to the default constraints whenever a linear speed change ends, so
    a controller property cannot raise it either. SpeedProfileAction applies its
    DynamicConstraints as the entity's behavior parameter, then changes speed under them."""
    return f"""<Action name="{name}"><PrivateAction><LongitudinalAction><SpeedProfileAction followingMode="follow">
							<DynamicConstraints maxAcceleration="{accel}" maxDeceleration="{decel}" maxAccelerationRate="{accel_rate}" maxDecelerationRate="{decel_rate}" maxSpeed="{vmax}" />
							<SpeedProfileEntry speed="{v}" />
						</SpeedProfileAction></LongitudinalAction></PrivateAction></Action>"""


def acquire(name, p):
    return f'<Action name="{name}"><PrivateAction><RoutingAction><AcquirePositionAction><Position>{world(p)}</Position></AcquirePositionAction></RoutingAction></PrivateAction></Action>'


def trajectory(name, vertices, z=0.0, yaw=None):
    """FollowTrajectoryAction over (time s, pose) vertices, time relative to the event's start;
    a time of None leaves the vertex untimed (followed at the entity's target speed)"""
    vs = "".join((f'<Vertex time="{t:.2f}">' if t is not None else "<Vertex>") +
                 f"<Position>{world(p, z, yaw)}</Position></Vertex>" for t, p in vertices)
    return f"""<Action name="{name}"><PrivateAction><RoutingAction><FollowTrajectoryAction initialDistanceOffset="0">
							<TimeReference><Timing domainAbsoluteRelative="relative" offset="0" scale="1" /></TimeReference>
							<TrajectoryFollowingMode followingMode="position" />
							<TrajectoryRef><Trajectory name="{name}_path" closed="false"><ParameterDeclarations />
								<Shape><Polyline>{vs}</Polyline></Shape>
							</Trajectory></TrajectoryRef>
						</FollowTrajectoryAction></RoutingAction></PrivateAction></Action>"""


def signal(name, way_id, st):
    return f'<Action name="{name}"><GlobalAction><InfrastructureAction><TrafficSignalAction><TrafficSignalStateAction name="{way_id}" state="{st}" /></TrafficSignalAction></InfrastructureAction></GlobalAction></Action>'


def set_param(name, param, value):
    return f'<Action name="{name}"><GlobalAction><ParameterAction parameterRef="{param}"><SetAction value="{value}" /></ParameterAction></GlobalAction></Action>'


def log(name, *params):
    """a line in the interpreter's log (SSv2's TestCommand prints its parameters)"""
    return f'<Action name="{name}"><UserDefinedAction><CustomCommandAction type="test({", ".join(params)})" /></UserDefinedAction></Action>'


def exit_action(name, kind):
    return f'<Action name="{name}"><UserDefinedAction><CustomCommandAction type="{kind}" /></UserDefinedAction></Action>'


# ---------------------------------------------------------------------------- structure


def event(name, actions, *groups):
    """an event started when any condition group holds (each group: all its conditions)"""
    cgs = "".join(f"<ConditionGroup>{''.join(g)}</ConditionGroup>" for g in groups)
    return f"""<Event name="{name}" priority="parallel" maximumExecutionCount="1">
						{"".join(actions)}
						<StartTrigger>{cgs}</StartTrigger>
					</Event>"""


def group(name, actors, events):
    refs = "".join(f'<EntityRef entityRef="{a}" />' for a in actors)
    return f"""<ManeuverGroup maximumExecutionCount="1" name="{name}">
				<Actors selectTriggeringEntities="false">{refs}</Actors>
				<Maneuver name="{name}_m">
					{"".join(events)}
				</Maneuver>
			</ManeuverGroup>"""


def actor_group(i, gname, actors, start_actions, start_cond, end_actions, end_conds,
                extra=(), watchdog_s=WATCHDOG_S, watchdog_tag="watchdog", busy=None):
    """One group of one incident's actors: `<g>_start` adds them, `<g>_end` deletes them, and
    `<g>_<watchdog_tag>` deletes them instead when the ego stood still `watchdog_s` while they
    were live (logged `degraded`, or `staged` for incident 4's deliberate clearing). Each
    actor's contact with the ego is logged once.

    The group's phase is a global parameter `<g>_phase` (0 before, 1 live, 2 over): end and
    watchdog each need phase 1 and set 2, so only one of them deletes. Not the events' own
    states: SSv2's StoryboardElementStateCondition for `standbyState` is true for good (its
    remembered state starts as standby and is only re-read when it does not match), so "has
    not happened yet" cannot be asked of an event.

    `busy`: an event of this group whose action moves the actors over time (a trajectory).
    Deleting an entity while such an action runs ends the scenario ("Entity ... does not
    exist"), so `<g>_busy` (0 not started, 1 running, 2 done) holds the watchdog off while
    it runs; `end_conds` must see to the same (e.g. the busy event's completion)."""
    tag, phase = f"i{i}", f"{gname}_phase"
    s, e, w = f"{gname}_start", f"{gname}_end", f"{gname}_{watchdog_tag}"
    evs = [event(s, [*start_actions, set_param(f"{s}_live", phase, 1), log(f"{s}_log", tag, "start", gname)],
                 [start_cond])]
    evs += list(extra)
    over = set_param(f"{e}_over", phase, 2)
    evs.append(event(e, [*end_actions, over, log(f"{e}_log", tag, "end", gname)],
                     *[[*c, param_is(f"{e}_live_{k}", phase, 1)] for k, c in enumerate(end_conds)]))
    verdict = "staged" if watchdog_tag == "clear" else "degraded"
    wd = [param_is(f"{w}_live", phase, 1), *standstill(f"{w}_ego_still", "ego", watchdog_s)]
    guards = [[]] if busy is None else [[param_is(f"{w}_idle_{v}", f"{gname}_busy", v)] for v in (0, 2)]
    evs.append(event(w, [*[delete(a) for a in actors], set_param(f"{w}_over", phase, 2),
                         log(f"{w}_log", tag, watchdog_tag, verdict, gname)],
                     *[[*wd, *g] for g in guards]))
    for a in actors:
        evs.append(event(f"{a}_contact", [log(f"{a}_contact_log", tag, "contact", a)],
                         [param_is(f"{a}_contact_live", phase, 1), collision(f"{a}_hits_ego", a, "ego")]))
    return group(gname, actors, evs)


GO_DELAY_S = 0.5
GO_ACCEL = 4.0


def go_event(gname, v, accel=GO_ACCEL):
    """`<gname>_go`: the group's actors, added standing, drive off to `v` m/s at `accel`
    GO_DELAY_S after the add. An NPC added already moving (a step SpeedAction in the add's
    event) can be flagged blocked by the bridge before its body settles -- 0 m/s along SSv2's
    6 for its first frames -- and handed to kinematic for the episode (032's fallback; step
    4b, 1 of 3 `i1_lead` spawns); one accelerated from rest is tracked from its first frame."""
    return event(f"{gname}_go", [speed(f"{gname}_drive_off", v, accel)],
                 [state(f"{gname}_added", f"{gname}_start", delay=GO_DELAY_S)])


def busy_event(gname, name, actions, *conds):
    """an event of group `gname` running `actions` over time, and its completion, flagged in
    `<gname>_busy` (see actor_group)"""
    b = f"{gname}_busy"
    return [event(name, [*actions, set_param(f"{name}_running", b, 1)],
                  *[[*c, param_is(f"{name}_live_{k}", f"{gname}_phase", 1)] for k, c in enumerate(conds)]),
            event(f"{name}_done", [set_param(f"{name}_finished", b, 2)], [state(f"{name}_complete", name)])]


def incident_groups(i, S):
    """(entities, groups) of incident `i` from the sites file's poses `S`"""
    if i == 1:
        lead = "i1_lead"
        on_road, brake, resume = "i1_lead_on_road", "i1_lead_brake", "i1_lead_resume"
        lead_mps, brake_gap = S.get("lead_mps", 3.0), S.get("brake_gap_m", 12.0)
        extra = [
            # merged into the ego's lane: on to `lead_mps` (below the ego's cruise, so the
            # ego closes in behind and follows)
            event(on_road, [speed("i1_cruise", lead_mps, 2.0)], [reach("i1_lead_merged", lead, S["lead_on_road"], 3.0)]),
            # brake ~6 m/s2 to a stop when the ego is within `brake_gap` m (box to box), or at
            # the brake point whatever the gap
            event(brake, [speed_profile("i1_brake_hard", 0.0, 3.0, 6.0, 60.0), log("i1_brake_log", "i1", "brake", lead)],
                  [state("i1_brake_after_merge", on_road), gap_below("i1_ego_close", "ego", lead, brake_gap)],
                  [reach("i1_lead_at_brake_point", lead, S["brake_at"], 2.5)]),
            # drive off once the ego has stood behind it for 4 s (or 20 s after the brake)
            event(resume, [speed("i1_drive_off", 4.5, 2.0)],
                  [state("i1_resume_after_brake", brake), *standstill("i1_ego_waited", "ego", 4.0)],
                  [state("i1_resume_fallback", brake, delay=20.0)]),
        ]
        g = actor_group(1, "i1", [lead],
                        [add(lead, S["lead_spawn"], 0.0), speed("i1_lead_hold", 0.0),
                         acquire("i1_lead_route", S["lead_target"])],
                        reach("i1_trigger", "ego", S["trigger"], 3.0),
                        [delete(lead)], [[reach("i1_lead_gone", lead, S["lead_end"], 4.0)]],
                        [go_event("i1", 6.0), *extra])
        return [car(lead)], [g]

    if i == 2:
        ped = "i2_ped"
        line = S["ped_spawn"]
        # A van at the kerb hides the walker from the approaching ego; a second car stands
        # just beyond him, so he steps out of the gap between the two. Both are misc objects
        # (`parked`) at the offset where obstacle_slow_down leaves them alone (step 4d).
        # Older sites files (step 4b) have one car, `parked`, and no van.
        cars = [(n, bp, S[k]) for n, bp, k in (("i2_van", "vehicle.mercedes.sprinter", "van"),
                                               ("i2_parked", "vehicle.tesla.model3", "parked")) if k in S]
        # Untimed trajectories at a commanded walking speed: with vertex times SSv2 started
        # the walker late and then ended the scenario for missing a vertex time (step 4b).
        # Out from behind the van into the ego's lane, stop there 4 s (startled), then
        # on across the oncoming lane to the far kerb.
        b = "i2_ped_busy"
        # he faces the way he walks: from the kerb across the ego's lane
        walk_yaw = math.atan2(S["ped_far"]["y"] - line["y"], S["ped_far"]["x"] - line["x"])
        ped_extra = [
            event("i2_ped_dart", [speed("i2_ped_walk", 1.5, rate=WALKER_START_RATE), trajectory("i2_dart", [(None, S["ped_spawn"]), (None, S["ped_in_lane"])], yaw=walk_yaw),
                                  set_param("i2_dart_running", b, 1), log("i2_dart_log", "i2", "dart", ped)],
                  [reach("i2_ego_near", "ego", S["dart"], 2.0), param_is("i2_dart_live", "i2_ped_phase", 1)]),
            event("i2_ped_hold", [speed("i2_ped_stop", 0.0)], [state("i2_ped_in_lane", "i2_ped_dart")]),
            event("i2_ped_cross", [speed("i2_ped_walk_on", 1.5, rate=WALKER_START_RATE), trajectory("i2_cross", [(None, S["ped_hold"]), (None, S["ped_far"])], yaw=walk_yaw)],
                  [state("i2_ped_held", "i2_ped_hold", delay=4.0)]),
            event("i2_ped_across", [speed("i2_ped_stop_far", 0.0), set_param("i2_ped_walked_far", b, 2)],
                  [state("i2_ped_crossed", "i2_ped_cross")]),
        ]
        trigger = reach("i2_trigger", "ego", S["trigger"], 3.0)
        past = [[reach("i2_ego_past", "ego", S["ego_past"], 3.0)]]
        names = [n for n, _, _ in cars]
        g_parked = actor_group(2, "i2_parked", names, [add(n, p) for n, _, p in cars], trigger,
                               [delete(n) for n in names], past)
        # no hold SpeedAction on the walker: a target speed of 0 left it standing through its
        # FollowTrajectoryAction (step 4b, first run); a walker added without one stands
        g_ped = actor_group(2, "i2_ped", [ped],
                            [add(ped, dict(line, yaw=walk_yaw))],
                            reach("i2_trigger_ped", "ego", S["trigger"], 3.0),
                            [delete(ped)],
                            [[param_is("i2_ped_walked", "i2_ped_busy", 2)],
                             [reach("i2_ego_past_ped", "ego", S["ego_past"], 3.0),
                              param_is("i2_ped_never_darted", "i2_ped_busy", 0)]],
                            ped_extra, busy=True)
        return [*[parked(n, bp) for n, bp, _ in cars], walker(ped)], [g_parked, g_ped]

    if i == 3:
        sid = str(S["signal"])
        ph = "i3_phase"  # 1 red, 2 green again
        evs = [
            event("i3_red", [signal("i3_set_red", sid, "red"), set_param("i3_is_red", ph, 1), log("i3_red_log", "i3", "red", sid)],
                  [reach("i3_trigger", "ego", S["trigger"], 3.0)]),
            event("i3_green", [signal("i3_set_green", sid, "green"), set_param("i3_is_green", ph, 2), log("i3_green_log", "i3", "green", sid)],
                  [param_is("i3_green_while_red", ph, 1), *standstill("i3_ego_waited", "ego", I3_GREEN_AFTER_STOP_S)]),
            event("i3_watchdog", [signal("i3_force_green", sid, "green"), set_param("i3_forced_green", ph, 2),
                                  log("i3_watchdog_log", "i3", "watchdog", "degraded", sid)],
                  [param_is("i3_still_red", ph, 1), state("i3_red_too_long", "i3_red", delay=I3_GREEN_FALLBACK_S)]),
        ]
        return [], [group("i3", ["ego"], evs)]

    if i == 4:
        broken, onc = "i4_broken", ["i4_oncoming_a", "i4_oncoming_b"]
        trigger = lambda n: reach(n, "ego", S["trigger"], 3.0)  # noqa: E731
        g_broken = actor_group(4, "i4_broken", [broken], [add(broken, S["broken"]), speed("i4_dead", 0.0)],
                               trigger("i4_trigger"), [delete(broken)],
                               [[reach("i4_ego_got_by", "ego", S["goal"], 15.0)]],
                               watchdog_s=I4_CLEAR_S, watchdog_tag="clear")
        g_onc = actor_group(4, "i4_oncoming", onc,
                            [add(onc[0], S["oncoming_a"]), add(onc[1], S["oncoming_b"]),
                             speed("i4_oncoming_hold", 0.0), acquire("i4_oncoming_route", S["oncoming_target"])],
                            trigger("i4_trigger_oncoming"), [delete(a) for a in onc],
                            [[reach("i4_oncoming_gone", onc[1], S["oncoming_end"], 4.0)]],
                            [go_event("i4_oncoming", S.get("oncoming_mps", 6.0))])
        return [car(broken), *[car(a) for a in onc]], [g_broken, g_onc]

    if i == 5:
        cross = ["i5_cross_a", "i5_cross_b"]
        g = actor_group(5, "i5", cross,
                        [add(cross[0], S["cross_a"]), add(cross[1], S["cross_b"]),
                         speed("i5_cross_hold", 0.0), acquire("i5_cross_route", S["cross_target"])],
                        reach("i5_trigger", "ego", S["trigger"], 3.0),
                        [delete(a) for a in cross], [[reach("i5_cross_gone", cross[1], S["cross_end"], 4.0)]],
                        [go_event("i5", 8.0)])
        return [car(a) for a in cross], [g]

    if i == 6:
        bike = "i6_cyclist"
        v = S.get("cyclist_mps", 5.5)
        # Added already riding at `v` (a step SpeedAction with the add) just behind the ego as
        # it brakes for the turn, then along the kerb side of its lane, straight on through
        # the junction, untimed. Not from rest on a timed trajectory: a bicycle ramping from 0
        # at its 4 m/s2 misses its first vertex time at 8 m/s and SSv2 fails the run (step
        # 4d), and one spawned further back cannot catch a 40 km/h ego. Close behind it is also
        # where CenterPoint sees it: ~10 m for a bicycle on this host (step 4d).
        ride = trajectory("i6_ride", [(None, S["cyclist_spawn"]), (None, S["cyclist_turn"]),
                                      (None, S["cyclist_far"])])
        g = actor_group(6, "i6", [bike], [add(bike, S["cyclist_spawn"]), speed("i6_riding", v)],
                        reach("i6_trigger", "ego", S["trigger"], 3.0),
                        [delete(bike)], [[param_is("i6_rode", "i6_busy", 2)]],
                        busy_event("i6", "i6_ride", [ride], [state("i6_ride_go", "i6_start")]), busy=True)
        return [bicycle(bike)], [g]

    raise ValueError(i)


def template(i, sites):
    S = sites["incidents"][i]
    entities, groups = incident_groups(i, S)
    ego_group = group("ego_group", ["ego"], [
        event("goal_reached", [exit_action("exit_success", "exitSuccess")],
              [reach("at_goal", "ego", S["goal"], GOAL_TOLERANCE_M)]),
        event("timeout", [exit_action("exit_timeout", "exitFailure")], [simtime("too_long", TIMEOUT_S)]),
    ])
    params = sorted(set(re.findall(r'parameterRef="([^"]+)"', "".join(groups))))
    decls = "".join(f'<ParameterDeclaration name="{p}" parameterType="integer" value="0" />' for p in params)
    desc = (f"033 incident {i} ({S['title']}): Town01, ego from route m {S['start']['route_m']:.0f} "
            f"to {S['goal']['route_m']:.0f}; generated by scripts/gen_demo_incidents.py")
    return f"""<?xml version="1.0"?>
<!-- Phase 033 demo tour, incident template {i}: {S['title']}. Generated by
     scripts/gen_demo_incidents.py from scripts/demo_town01_sites.yaml; edit those, not this. -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-10-14T00:00:00+00:00" description="{desc}" revMajor="1" revMinor="2" />
	<ParameterDeclarations>{decls}</ParameterDeclarations>
	<CatalogLocations />
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town01" /></RoadNetwork>
	<Entities>
		{ego_object()}
		{"".join(entities)}
	</Entities>
	<Storyboard>
		<Init><Actions>
			<Private entityRef="ego">
				<PrivateAction><TeleportAction><Position>{world(S['start'], 0.3)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{world(S['goal'])}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
		</Actions></Init>
		<Story name="incident_{i}">
			<Act name="incident_{i}_act">
			{ego_group}
			{"".join(groups)}
				<StartTrigger><ConditionGroup>{simtime("start", 0)}</ConditionGroup></StartTrigger>
			</Act>
		</Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--sites", type=Path, default=SITES)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--only", type=int, nargs="*", help="incident ids (default: all)")
    a = ap.parse_args()
    sites = yaml.safe_load(a.sites.read_text())
    a.out.mkdir(parents=True, exist_ok=True)
    for i in a.only or sorted(FILES):
        path = a.out / FILES[i]
        path.write_text(template(i, sites))
        print(path)


if __name__ == "__main__":
    main()
