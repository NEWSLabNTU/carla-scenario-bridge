# Phase 033: Demo Tour

One continuous 5-8 minute drive through Town01 in which Autoware meets a sequence of
incidents -- a car braking hard ahead, a pedestrian darting out from behind a parked car, a
red light, a blocked lane, crossing traffic at a turn, a cyclist on the inside -- and handles
each, for an audience watching the CARLA view and Autoware's RViz side by side.

**Source**: user direction 2026-10-14: a demo run "going through many scenarios so the
audience can see many incidents and handling"; decisions the same day: **one continuous
tour** (not a playlist of scenarios), **Town01 only** for now, **ramp merge deferred**
(needs a highway town, [031](031-maps-for-scenarios.md)), **5-8 minutes**.
**Depends on**: [020](020-starter-scenarios-and-suites.md) (starter scenarios the incidents
grow from), [023](023-npc-motion-parity.md) / [032](032-npc-tracking-fixes.md) (tracked NPCs
that move like vehicles on camera), [024](024-gap-trace-and-oracle.md) (rehearsal judged by
the oracle), [027](027-user-documentation.md) tutorial 4 (`chase_view`, RViz layouts,
recording).

## What the audience sees

One window layout (tutorial 4's: CARLA chase camera left, ego RViz right), plus a caption
strip naming the incident as it starts and how Autoware handled it ("Pedestrian darts out
-- emergency stop, 4.1 m to spare"). The ego never teleports, the screen never goes blank,
and the tour ends at a goal with a closing caption.

## Why Town01, and what it limits

Town01 is the town this stack is proven on. Its Lanelet2 map has 36 signals, T-junctions
and two-lane roads, but **no crosswalk lanelets and no multi-lane same-direction roads**
(counted 2026-10-14). So: a pedestrian incident is a dart-out handled by `run_out` /
`obstacle_stop`, not a crosswalk yield; a blocked lane cannot be passed by a lane change;
cut-ins and lane changes wait for a multi-lane town (a later tour). Ramp merge needs a
highway town (CARLA Town04 / Town06) that the map pack lacks -- deferred with 031.

## Incidents

Order follows the route; each is a self-contained **incident template** (a short `.xosc`
that runs on its own under `run_suite`, then is composed into the tour).

| # | Incident | What the actors do | Autoware's expected handling (module) | Pass in rehearsal |
|---|---|---|---|---|
| 1 | **Lead car brakes hard** | an NPC merges ahead from a side street, cruises, then brakes ~6 m/s² to a stop | follows, then stops behind it (`obstacle_cruise`, `obstacle_stop`) | no contact; stop gap ≥ 2 m |
| 2 | **Pedestrian darts out** | a parked car at the kerb hides a pedestrian who steps out at ~1.5 m/s when the ego is ~3.5 s away | brakes for the emerging pedestrian (`run_out`, `obstacle_stop`, `dynamic_obstacle_stop`) | no contact; min distance reported |
| 3 | **Red light** | the signal at the next junction turns red as the ego approaches | stops at the light, goes on green (`traffic_light`); as `town01_traffic_light` already does | stops before the junction; resumes on green |
| 4 | **Blocked lane** | a broken-down vehicle stands in the ego's lane, hazards on, oncoming traffic in the other lane | waits behind it (`static_obstacle_avoidance`: no same-direction lane to use; opposite-lane passing is off by default) -- after N s the tow truck "clears" it (the watchdog, on camera) | no contact; waits, then proceeds |
| 5 | **Crossing traffic at a turn** | NPCs cross the junction the ego turns into | yields, then turns (`intersection`, `intersection_collision_checker`) | no contact; turns after the gap |
| 6 | **Cyclist on the inside** | a cyclist rides alongside on the inside as the ego turns | waits for the cyclist (`blind_spot`) -- only if the bicycle blueprint (022) tracks well; else dropped | no contact |

Each incident is triggered by the ego's arrival at a point on the route (OpenSCENARIO
`ReachPositionCondition` / `DistanceCondition`), so its timing holds whatever the ego's
speed was on the way. Its actors are added when it starts (`AddEntityAction`, proven by
UC-ACC / UC-AEB on this bridge) and removed when it is over (`DeleteEntityAction`), so the
town stays empty between incidents and CARLA is never carrying a crowd.

## The route and the clock

- One route through Town01, picked with `carla_poses` and the Lanelet2 map: ~2.5-3 km at
  Autoware's cruise (~30 km/h) passes the six incident sites, about 5-6 min of driving plus
  ~1-2 min of incident handling -- inside 5-8 min.
- The ego gets the whole route once, as waypoints (`AssignRouteAction`; the relay already
  passes `waypoints` to the agent), so Autoware never stops at an intermediate goal.
  Fallback if multi-waypoint routing misbehaves: chained goals, each set before the
  previous one is reached.
- Incident sites are at least ~300 m apart, so each starts from cruise and the audience
  gets a breath between them.

## Never stall

A demo must finish even when one incident goes wrong.

- **Collisions are logged, not fatal**: `CollisionCondition` raises a caption and a log
  line, never the scenario's stop trigger.
- **Watchdog per incident**: if the ego has stood still for N s (per incident, e.g. 20 s)
  while the incident is active, its actors are removed (the "clears" in incident 4 is this
  watchdog, staged deliberately) and the tour continues; the watchdog firing is logged as a
  degraded incident.
- **One stop trigger**: the final goal reached, or a global timeout (10 min).
- The stack is the proven one: Town01, `npc_motion: tracked`, ego through the relay.

## Captions

A small `demo_captions` window (Python, ROS) shows the current incident's title and, when
it ends, how it went (stop distance, min gap, time waited) -- from SSv2's storyboard state
(the interpreter's published event states; which topic carries them is a spike item) and
the bridge's per-incident numbers. Captions are text on a plain background, positioned in
the tutorial-4 layout's free strip so the recording picks them up.

## Rehearsal and acceptance

- Each incident template alone: 5 runs, 5 passes by its row's criterion, gap trace on,
  `gap_oracle` clean (no G1 / G2; G3 only of the known artefact kind).
- The full tour: 3 runs in a row, each finishing in 5-8 min with every incident passed and
  no watchdog firing except incident 4's staged one.
- One recorded take (tutorial 4's recording), cut with `scripts/make_showreel.sh` into a
  short GIF / MP4 for the README if it is good.

## Steps

1. [ ] **Docs.** This phase doc; README entry.
2. [ ] **Spike: the SSv2 features the tour needs** on this bridge, in a scratch scenario:
   `DeleteEntityAction` mid-run (actor gone from CARLA, sensors first), a multi-waypoint
   `AssignRouteAction` for the ego through the relay (Autoware drives through without
   stopping), and where SSv2 publishes storyboard event states for captions.
3. [ ] **Route.** The Town01 route and the six sites, as poses in a scratch file; one drive
   of the bare route (no incidents) timed.
4. [ ] **Incident templates** 1-6, each runnable alone; 5/5 rehearsal each; drop 6 if the
   cyclist does not track.
5. [ ] **Compose the tour** (`src/csb_examples/scenarios/demo/town01_tour.xosc`): route,
   triggers, add / delete, watchdogs, the single stop trigger.
6. [ ] **Captions** (`demo_captions`) and a `demo` launch / justfile recipe bringing up the
   layout.
7. [ ] **Rehearsal**: 3 full tours; one recorded take; README / tutorial 4 updated.

## Later

A second tour on a multi-lane town with crosswalks (Town05 or Town10) once a drive spike
there passes: crosswalk yield, cut-in, lane change, blocked lane passed by a lane change.
Ramp merge with 031's highway maps.

### Results
