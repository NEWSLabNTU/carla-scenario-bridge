#!/usr/bin/env python3
"""Generate the demo tour (roadmap 033 step 5): one continuous drive of route C through Town01
in which the ego meets the six incidents in route order (2, 1, 3, 4, 5, 6).

The incidents are composed from the same pieces as their standalone templates --
`gen_demo_incidents.incident_groups(n, sites)` on `scripts/demo_town01_sites.yaml` -- so a
template and its incident in the tour cannot drift apart. What the tour adds: the ego's
start on route C, one `AssignRouteAction` through step 3's four waypoints to the goal, and
the single stop trigger (the goal, or 600 s of simulation time). Each incident keeps its own
triggers (the ego reaching a pose), adds and deletes, watchdogs and contact logs; see
gen_demo_incidents.py for the rules every group keeps.

Ambient traffic and walkers (step 8) come from `scripts/demo_town01_ambient.yaml` when it
exists, as groups of their own (`ambient_groups`): added when the ego reaches a route pose,
deleted when they are done or the ego has passed, no watchdog (an ambient actor never holds
the ego up; step 8 checks that).

Usage:
    scripts/gen_demo_tour.py                      # writes src/swb_examples/scenarios/demo/town01_tour.xosc
    scripts/gen_demo_tour.py --out /tmp/tour.xosc
    scripts/gen_demo_tour.py --no-ambient         # incidents only
"""

import argparse
import importlib.machinery
import importlib.util
import math
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
_loader = importlib.machinery.SourceFileLoader("gen_demo_incidents", str(ROOT / "scripts/gen_demo_incidents.py"))
_spec = importlib.util.spec_from_loader("gen_demo_incidents", _loader)
G = importlib.util.module_from_spec(_spec)
_loader.exec_module(G)

SITES = G.SITES
AMBIENT = ROOT / "scripts/demo_town01_ambient.yaml"
DEFAULT_OUT = ROOT / "src/swb_examples/scenarios/demo/town01_tour.xosc"

TIMEOUT_S = 600  # sim time: the tour is ~5 min of driving and handling
ROUTE_ORDER = (2, 1, 3, 4, 5, 6)

# Route C (step 4d): start on the west road heading south, step 3's four waypoints, the goal.
START = dict(x=-2.03, y=-19.96, yaw=-1.5704)
WAYPOINTS = [dict(x=213.62, y=-330.6, yaw=-0.0001), dict(x=396.3, y=-164.54, yaw=1.571),
             dict(x=366.59, y=1.99, yaw=3.1411), dict(x=334.84, y=-94.03, yaw=-1.5714)]
GOAL = dict(x=175.67, y=-195.15, yaw=3.1415)

AMBIENT_CAR_MPS = 8.0
WALKER_BOX = (0.6, 0.6, 1.8)  # length, width, height, as the incident walker's


def pedestrian(name, bp="walker.pedestrian.0001"):
    """a walking ambient pedestrian; `bp` the CARLA walker blueprint (SSv2 sends model3d as
    the asset key)"""
    return f"""<ScenarioObject name="{name}">
			<Pedestrian model="unspecified" model3d="{bp}" mass="70.0" name="{bp}" pedestrianCategory="pedestrian">
				<BoundingBox><Center x="0.0" y="0.0" z="0.9" /><Dimensions width="0.6" length="0.6" height="1.8" /></BoundingBox>
				<Properties />
			</Pedestrian>
			<ObjectController><Controller name=""><Properties /></Controller></ObjectController>
		</ScenarioObject>"""


def standing_walker(name, bp):
    """a person standing still, as a MiscObject: a pedestrian entity costs SSv2 ~10 ms of its
    frame each on Town01 (step 8), a misc object nothing measurable. The bridge spawns the
    walker blueprint with physics off where SSv2 puts it (pose z = the walker's centre)."""
    length, width, height = WALKER_BOX
    return f"""<ScenarioObject name="{name}">
			<MiscObject mass="70" miscObjectCategory="obstacle" name="{bp}" model3d="{bp}">
				<BoundingBox><Center x="0.0" y="0.0" z="0.0" /><Dimensions width="{width}" height="{height}" length="{length}" /></BoundingBox>
				<Properties />
			</MiscObject>
		</ScenarioObject>"""


# ---------------------------------------------------------------------------- ambient (step 8)


def _pose(p):
    return dict(x=p[0], y=p[1], yaw=p[2]) if isinstance(p, (list, tuple)) else p


def _ambient_group(name, actors, start_actions, conds, ends, extra=()):
    """one ambient group: `<name>_start` adds, `<name>_end` deletes (any of `ends`), each
    actor's contact with the ego logged. No watchdog: an ambient actor must never be what
    holds the ego up (step 8 checks that), and a deletion in front of the camera would only
    hide it."""
    phase = f"{name}_phase"
    s, e = f"{name}_start", f"{name}_end"
    evs = [G.event(s, [*start_actions, G.set_param(f"{s}_live", phase, 1), G.log(f"{s}_log", "ambient", "start", name)],
                   conds)]
    evs += list(extra)
    evs.append(G.event(e, [*[G.delete(a) for a in actors], G.set_param(f"{e}_over", phase, 2),
                           G.log(f"{e}_log", "ambient", "end", name)],
                       *[[*c, G.param_is(f"{e}_live_{k}", phase, 1)] for k, c in enumerate(ends)]))
    for a in actors:
        evs.append(G.event(f"{a}_contact", [G.log(f"{a}_contact_log", "ambient", "contact", a)],
                           [G.param_is(f"{a}_contact_live", phase, 1), G.collision(f"{a}_hits_ego", a, "ego")]))
    return G.group(name, actors, evs)


def _passed(tag, g):
    """(event, condition): `<tag>_ego_by` becomes 1 when the ego reaches the group's `ego_gone`
    pose, and stays: a ReachPositionCondition holds only while the ego is there, so a deletion
    that also waits for something else (a walk to end, a drive-off) would miss it."""
    flag = f"{tag}_ego_by"
    ev = G.event(f"{tag}_ego_passed", [G.set_param(f"{tag}_ego_by_set", flag, 1)],
                 [G.reach(f"{tag}_ego_at_gone", "ego", _pose(g["ego_gone"]), 3.0)])
    return ev, G.param_is(f"{tag}_ego_gone", flag, 1)


def ambient_groups(A):
    """(entities, groups) from the ambient file's `groups`, each started when the ego reaches
    its `trigger` pose (and, with `after_param: [p, v]`, only once global parameter p is v --
    e.g. `[i3_phase, 1]`, incident 3's red). Kinds:

    - `cars`: vehicles added standing at their `spawn` poses, driven off GO_DELAY_S later to
      `mps` along a lanelet route to `target`; deleted when the last one reaches `end`, or
      when the ego reaches `ego_gone` (if given) once their drive-off has long finished (no
      entity is deleted while a speed change of its own runs).
    - `parked`: misc objects (`vehicle.*` blueprints, physics off; step 4d), deleted when the
      ego reaches `ego_gone`.
    - `walkers`: pedestrians. One with a `path` of one pose stands -- a MiscObject with a walker
      blueprint (`standing_walker`), all such in one group, deleted when the ego reaches
      `ego_gone`; one with a longer path is a pedestrian entity and walks it untimed at
      `mps` (started at WALKER_START_RATE, `delay` s after the add), in a group of its own
      (a group's private actions apply to all its actors), stops at its end and is deleted
      then, or once the ego has reached the group's `ego_gone` as well, if given.
    """
    entities, groups = [], []
    for g in A["groups"]:
        name, kind = g["name"], g["kind"]

        def conds(tag=name):
            c = [G.reach(f"{tag}_trigger", "ego", _pose(g["trigger"]), g.get("trigger_tol", 3.0))]
            if "after_param" in g:
                p, v = g["after_param"]
                c.append(G.param_is(f"{tag}_after", p, v))
            return c

        if kind == "cars":
            actors = [a["name"] for a in g["actors"]]
            entities += [G.car(a) for a in actors]
            start = [G.add(a["name"], _pose(a["spawn"])) for a in g["actors"]]
            start += [G.speed(f"{name}_hold", 0.0), G.acquire(f"{name}_route", _pose(g["target"]))]
            go = G.event(f"{name}_go", [G.speed(f"{name}_drive_off", g.get("mps", AMBIENT_CAR_MPS), G.GO_ACCEL)],
                         [G.state(f"{name}_added", f"{name}_start", delay=G.GO_DELAY_S)])
            ends = [[G.reach(f"{name}_done", actors[-1], _pose(g["end"]), g.get("end_tol", 4.0))]]
            extra = [go]
            if "ego_gone" in g:
                ev, gone = _passed(name, g)
                extra.append(ev)
                ends.append([gone, G.state(f"{name}_went", f"{name}_go", delay=5.0)])
            groups.append(_ambient_group(name, actors, start, conds(), ends, extra))
        elif kind == "parked":
            actors = [a["name"] for a in g["actors"]]
            entities += [G.parked(a["name"], a.get("bp", "vehicle.tesla.model3")) for a in g["actors"]]
            start = [G.add(a["name"], _pose(a["pose"])) for a in g["actors"]]
            ev, gone = _passed(name, g)
            groups.append(_ambient_group(name, actors, start, conds(), [[gone]], [ev]))
        elif kind == "walkers":
            standing = [a for a in g["actors"] if len(a["path"]) == 1]
            walking = [a for a in g["actors"] if len(a["path"]) > 1]
            entities += [standing_walker(a["name"], a.get("bp", "walker.pedestrian.0001")) for a in standing]
            entities += [pedestrian(a["name"], a.get("bp", "walker.pedestrian.0001")) for a in walking]
            if standing:
                path0 = [_pose(a["path"][0]) for a in standing]
                # a standing walker faces `yaw` of its pose; its misc-object origin is its centre
                ev, gone = _passed(name, g)
                groups.append(_ambient_group(name, [a["name"] for a in standing],
                                             [G.add(a["name"], p, z=WALKER_BOX[2] / 2 + g.get("kerb_z", 0.15))
                                              for a, p in zip(standing, path0)], conds(), [[gone]], [ev]))
            for a in walking:
                w = a["name"]
                path = [_pose(p) for p in a["path"]]
                yaw = math.atan2(path[1]["y"] - path[0]["y"], path[1]["x"] - path[0]["x"])
                walk = G.event(f"{w}_walk", [G.speed(f"{w}_pace", a.get("mps", 1.3), rate=G.WALKER_START_RATE),
                                             G.trajectory(f"{w}_path", [(None, p) for p in path], yaw=yaw)],
                               [G.state(f"{w}_added", f"{w}_start", delay=G.GO_DELAY_S + a.get("delay", 0.0))])
                # at the end of its path it stops (a step to 0: SSv2 otherwise keeps it walking
                # on), and stands there until it is deleted -- at once, or once the ego has
                # reached `ego_gone`, so it does not vanish in front of the camera
                halt = G.event(f"{w}_halt", [G.speed(f"{w}_stand", 0.0)], [G.state(f"{w}_walked", f"{w}_walk")])
                end, extra = [G.state(f"{w}_halted", f"{w}_halt")], [walk, halt]
                if "ego_gone" in g:
                    ev, gone = _passed(w, g)
                    end.append(gone)
                    extra.append(ev)
                groups.append(_ambient_group(w, [w], [G.add(w, dict(path[0], yaw=yaw))], conds(w),
                                             [end], extra))
        else:
            raise ValueError(f"{name}: kind {kind}")
    return entities, groups


# ---------------------------------------------------------------------------- the tour


def tour(sites, ambient=None):
    entities, groups = [], []
    for i in ROUTE_ORDER:
        ents, grps = G.incident_groups(i, sites["incidents"][i])
        entities += ents
        groups += grps
    if ambient:
        ents, grps = ambient_groups(ambient)
        entities += ents
        groups += grps
    ego_group = G.group("ego_group", ["ego"], [
        G.event("goal_reached", [G.log("tour_goal_log", "tour", "goal"), G.exit_action("exit_success", "exitSuccess")],
                [G.reach("at_goal", "ego", GOAL, G.GOAL_TOLERANCE_M)]),
        G.event("timeout", [G.log("tour_timeout_log", "tour", "timeout"), G.exit_action("exit_timeout", "exitFailure")],
                [G.simtime("too_long", TIMEOUT_S)]),
    ])
    body = "".join(groups)
    params = sorted(set(re.findall(r'parameterRef="([^"]+)"', body)))
    decls = "".join(f'<ParameterDeclaration name="{p}" parameterType="integer" value="0" />' for p in params)
    route = "".join(f'<Waypoint routeStrategy="shortest"><Position>{G.world(p)}</Position></Waypoint>'
                    for p in WAYPOINTS + [GOAL])
    desc = ("033 demo tour: Town01 route C (1438 m), incidents 2, 1, 3, 4, 5, 6"
            + (", ambient traffic and walkers" if ambient else "")
            + "; generated by scripts/gen_demo_tour.py")
    return f"""<?xml version="1.0"?>
<!-- Phase 033 demo tour. Generated by scripts/gen_demo_tour.py from scripts/demo_town01_sites.yaml
     (and scripts/demo_town01_ambient.yaml); edit those, not this. Ego stack:
     just ego-av max_velocity:=11.11 lidar_detection_model:=centerpoint use_multi_channel_tracker_merger:=false
     Sim side: just run weather:=ClearNoon -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-10-21T00:00:00+00:00" description="{desc}" revMajor="1" revMinor="2" />
	<ParameterDeclarations>{decls}</ParameterDeclarations>
	<CatalogLocations />
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town01" /></RoadNetwork>
	<Entities>
		{G.ego_object()}
		{"".join(entities)}
	</Entities>
	<Storyboard>
		<Init><Actions>
			<Private entityRef="ego">
				<PrivateAction><TeleportAction><Position>{G.world(START, 0.3)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AssignRouteAction><Route name="route_c" closed="false">{route}</Route></AssignRouteAction></RoutingAction></PrivateAction>
			</Private>
		</Actions></Init>
		<Story name="tour">
			<Act name="tour_act">
			{ego_group}
			{body}
				<StartTrigger><ConditionGroup>{G.simtime("start", 0)}</ConditionGroup></StartTrigger>
			</Act>
		</Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--sites", type=Path, default=SITES)
    ap.add_argument("--ambient", type=Path, default=AMBIENT)
    ap.add_argument("--no-ambient", action="store_true")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    a = ap.parse_args()
    sites = yaml.safe_load(a.sites.read_text())
    amb = None if a.no_ambient or not a.ambient.exists() else yaml.safe_load(a.ambient.read_text())
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(tour(sites, amb))
    print(a.out)


if __name__ == "__main__":
    main()
