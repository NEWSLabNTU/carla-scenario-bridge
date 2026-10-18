# Phase 033: Demo Tour

One continuous 5-8 minute drive through Town01 in which Autoware meets a sequence of
incidents -- a car braking hard ahead, a pedestrian darting out from behind a parked car, a
red light, a blocked lane, crossing traffic at a turn, a cyclist on the inside -- and handles
each, for an audience watching the CARLA view and Autoware's RViz side by side.

**Source**: user direction 2026-10-14: a demo run "going through many scenarios so the
audience can see many incidents and handling"; decisions the same day: **one continuous
tour** (not a playlist of scenarios), **Town01 only** for now, **ramp merge deferred**
(needs a highway town, [031](031-maps-for-scenarios.md)), **5-8 minutes**. Revised
2026-10-18 (user, after options A/B/C): **route C**, **40 km/h** cruise, **cyclist kept**
(CenterPoint + the [042](042-autoware-overlay.md) overlay), **one weather for the whole run
(ClearNoon)**, and **ambient traffic and walkers, dense** -- see "The tour, revised" below,
which supersedes the route, clock and empty-town parts of the sections after it.
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

## The tour, revised (2026-10-18)

![Revised tour: route, incidents, ambient actors](img/033-tour-design.png)

Design sketch (scratch `tour_opts/design2.py`); sidewalk offsets are approximate and are
fitted to CARLA's sidewalks in step 8.

**Route C, 1438 m.** Start on the west road at (-2, -20) heading south (the step 3 route
began at y -150), then the step 3 lanelets unchanged: round the bottom-left corner, east
along the bottom road, north up the east road, left at the top-right T, left into the x 337
road, right at y -198 to the goal (175.67, -195.15). The pedestrian dart moves to the west
road, so the tour opens on an incident ~25 s in.

| Route order | Incident | Site | Incident at (route m) | Ego trigger (route m) |
|---|---|---|---|---|
| 1 | 2 pedestrian dart | west road; a van and a second car parked at the ego-side kerb, the walker steps out from behind the van, stands, then crosses to the far side | 190 | 130 |
| 2 | 1 lead car brakes | lead merges from the x 90 stem, brakes 6 m/s^2 near x 200 | 508 | 329 |
| 3 | 3 red light | signal 43757, stop line x 325.7 | 633 | 558 (75 m before) |
| 4 | 4 blocked lane | east road at y -175, broken car in the ego's lane, oncoming pair | 855 | 775 |
| 5 | 5 crossing traffic, left turn | two cars eastbound on y -2, 15 m apart | 1075 | 1007 |
| 6 | 6 cyclist inside the right turn | cyclist from (333.2, -125.8) south in the ego's lane, 1.4 m right of centre, straight on | 1272 | 1172 |

Triggers are rescaled for 40 km/h: from 11.1 m/s at 2.5 m/s^2 the ego needs ~25 m plus
latency to stop, against ~4 m at 15 km/h. Gaps between incidents are 125-318 m.

**Clock.** ~3.2 min of driving (corners still ~3-4 m/s) plus ~2 min of handling: **~5-5.5 min
of simulation time**, inside 5-8; wall time longer on a loaded host (step 3).

**40 km/h.** The 15 km/h cap is Autoware's `max_vel: 4.17` in autoware_launch's
`common.param.yaml` (the external velocity limit cannot exceed it; Town01's Lanelet2 map has
no `speed_limit` tags). acb gets a `max_velocity` launch argument through a generated copy of
that file (the sync-script pattern of `carla_control` / `object_recognition` / the tracker
param), **default 4.17**, so every suite keeps 15 km/h; the tour passes 11.11, and the ego's
`Performance maxSpeed` in the scenario matches.

**Perception: CenterPoint + the 042 overlay, for the whole tour** (user decision): clustering
labels everything UNKNOWN, which `blind_spot` ignores, so incident 6 needs CenterPoint.
CenterPoint lost 7 of 9 UC-AEB variants on this host, so incidents 1-5 are rehearsed again
on it, at 40 km/h; ground-truth objects (`perception:=false`, acb labels BICYCLE) are the
fallback if they do not hold, decided then with the user.

**Weather: ClearNoon**, the bridge's `weather` parameter, for the whole run (no change
mid-tour).

### Ambient traffic and walkers

The town is no longer empty between incidents: about 18 moving vehicles, 17 parked cars and
27 walkers over the tour, with ~10-15 actors in CARLA at any time. All are **SSv2 NPCs
scripted in the tour** (no CARLA autopilot, no Traffic Manager): deterministic, tracked
(`npc_motion: tracked`), each with a name of its own, added before its segment and deleted
after it, like the incident actors. None may make the ego stop outside an incident; step 8
checks that.

| Group | Who | Where / path (route m) | Role |
|---|---|---|---|
| a1-a4 | oncoming cars, two pairs | west road, 175 -> 15 early, 330 -> 205 after incident 1 | traffic in the other lane |
| a5-a7 | oncoming cars | bottom road, 610 -> 360, after the lead clears | |
| w1 | oncoming car | waits at its own red across the junction (incident 3) | the light holds both sides |
| s1-s2 | cars from the x 337 stem | turn east on the ego's red, drive ahead, deleted at 740 (before incident 4's trigger) | the green phase is visible |
| f1-f2 | followers | join behind the ego at ~545, queue behind it at the red light and the blocked lane, deleted at ~1035 | a queue forms on camera |
| b1-b2 | oncoming cars | east road, 1035 -> 990, after incident 4 | |
| n1-n2 | oncoming cars | x 337 road, 1250 -> 1110, clear before the cyclist's trigger | |
| g1-g2 | oncoming cars | closing road, goal -> 1290 | |
| cross | 8 cars, 4 pairs | inner-block roads y -57 / -54 / -129.5 / -133, past the x 337 junctions before the ego reaches them | |
| parked | ~17 static cars | kerbs: west road (incl. incident 1's two), bottom road, east road, x 337 road, closing road | clutter for perception |
| p1-p8 | walkers | west road: two on the far sidewalk with the ego, two on the ego side towards it, a group of three waiting near the start, one after incident 1 | sidewalk walkers do not stop the ego |
| p9-p10 | walkers | bottom road ego-side sidewalk, 440 -> 590 | |
| p11-p14 | walkers | cross the ego's road just past its stop line **on the ego's red**, clear before green | |
| p15-p17 | walkers | east road sidewalks; p17 passes the broken car | |
| p18-p20 | walkers | a group waiting at the top-right T corner, not crossing | |
| p21-p22 | walkers | x 337 road, oncoming-side sidewalk only | |
| p23-p27 | walkers | closing road, both sidewalks | closing scene |

Town01 has no sidewalk or crosswalk lanelets, so walkers follow fixed polylines
(`FollowTrajectoryAction`, as `town01_pedestrian` does), and cars off the route follow
lanelet routes. The **inside of the cyclist's right turn stays clear** of walkers (green strip
in the sketch), and the stem cars s1-s2 only turn east, away from p11-p14. This is the most
CenterPoint has to see in one run; if perception or the frame budget suffers, density goes
down group by group (parked cars and far-side walkers first).

## Incidents

Numbers are ids; along the step 3 route they come in the order 1, 3, 2, 4, 5, 6. Each is a
self-contained **incident template** (a short `.xosc` that runs on its own under
`run_suite`, then is composed into the tour).

| # | Incident | What the actors do | Autoware's expected handling (module) | Pass in rehearsal |
|---|---|---|---|---|
| 1 | **Lead car brakes hard** | an NPC merges ahead from a side street, cruises at 3 m/s, then brakes 6 m/s² to a stop when the ego is within 12 m (or at a brake point); drives off once the ego has stood 4 s | follows, then stops behind it (`obstacle_stop`; step 4b) | no contact; stop gap ≥ 2 m |
| 2 | **Pedestrian darts out** | a car parked half on the kerb hides a pedestrian who steps out (ramping to 1.5 m/s at 1.5 m/s²) when the ego is ~7 s away (origin 30 m from its line; step 4c), stops in the lane 4 s, then crosses | brakes for the emerging pedestrian (`obstacle_stop`; step 4b) | no contact; min distance reported |
| 3 | **Red light** | the signal at the next junction turns red as the ego approaches | stops at the light, goes on green (`traffic_light`); as `town01_traffic_light` already does | stops before the junction; resumes on green |
| 4 | **Blocked lane** | a broken-down vehicle stands in the ego's lane (hazards not commanded: no light action on this path), oncoming traffic in the other lane | waits behind it (`obstacle_stop`; no avoidance attempted) -- after 12 s standing the tow truck "clears" it (the watchdog, on camera) | no contact; waits, then proceeds |
| 5 | **Crossing traffic at a turn** | two NPCs, 15 m apart, cross the junction the ego turns into | yields to both, then turns (module not observable: `intersection` RTC safe throughout, no planning factor; step 4b) | no contact; turns after them |
| 6 | **Cyclist on the inside** | a cyclist rides alongside on the inside as the ego turns | waits for the cyclist (`blind_spot`) -- dropped in step 4b (clustering reports it UNKNOWN; `blind_spot` misjudged right turns, step 4c); **back in the tour 2026-10-18**: CenterPoint + the 042 overlay, 3/3 PASS | no contact |

Each incident is triggered by the ego's arrival at a point on the route (OpenSCENARIO
`ReachPositionCondition` / `DistanceCondition`), so its timing holds whatever the ego's
speed was on the way. Its actors are added when it starts (`AddEntityAction`, proven by
UC-ACC / UC-AEB on this bridge) and removed when it is over (`DeleteEntityAction`), so the
town stays empty between incidents and CARLA is never carrying a crowd (revised 2026-10-18:
ambient actors fill the gaps, added and deleted the same way; see above). **Every actor gets a
name of its own** (`i1_lead`, `i2_ped`, ...): SSv2 never lets a deleted name be added again
(step 2 spike), so no name is reused across incidents.

## The route and the clock

*As first planned (15 km/h, step 3's route); superseded by "The tour, revised" above.*

- One route through Town01, picked with `carla_poses` and the Lanelet2 map, passing the six
  incident sites: about 5-6 min of driving plus ~1-2 min of incident handling -- inside
  5-8 min. The ego cruises at **15 km/h** on Town01 (4.17 m/s measured in the step 2 spike,
  4.0 m/s average with three turns; the same 15 km/h as every 020 run), not the ~30 km/h
  first assumed, so 5-6 min of driving is **~1.2-1.4 km**, not 2.5-3 km -- unless step 3 raises
  the cruise (the Town01 Lanelet2 map carries no `speed_limit` tags; find where the 15 km/h
  comes from first).
- The ego gets the whole route once, as waypoints (`AssignRouteAction`; the relay already
  passes `waypoints` to the agent), so Autoware never stops at an intermediate goal --
  proven in the step 2 spike (one Autoware route through all four waypoints, no stop); the
  chained-goals fallback is not needed.
- Incident sites are at least ~300 m apart (~70 s at 15 km/h; ~200 m if the route must
  stay near 1.3 km), so each starts from cruise and the audience
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
and the bridge's per-incident numbers. The storyboard state is `/simulation/context`
(`openscenario_interpreter_msgs/msg/Context`, scenario domain 9, transient-local depth 1,
one ~4.5 KB JSON snapshot of the whole storyboard per frame at 20 Hz; step 2 spike), so
`demo_captions` subscribes to that and diffs `Event.currentState` by event name. Captions
are text on a plain background, positioned in the tutorial-4 layout's free strip so the
recording picks them up.

## Rehearsal and acceptance

- Each incident template alone: 5 runs, 5 passes by its row's criterion, gap trace on,
  `gap_oracle` clean (no G1 / G2; G3 only of the known artefact kind).
- The full tour: 3 runs in a row, each finishing in 5-8 min with every incident passed and
  no watchdog firing except incident 4's staged one.
- One recorded take (tutorial 4's recording), cut with `scripts/make_showreel.sh` into a
  short GIF / MP4 for the README if it is good.

## Steps

1. [x] **Docs.** This phase doc; README entry.
2. [x] **Spike: the SSv2 features the tour needs** on this bridge, in a scratch scenario:
   `DeleteEntityAction` mid-run (actor gone from CARLA, sensors first), a multi-waypoint
   `AssignRouteAction` for the ego through the relay (Autoware drives through without
   stopping), and where SSv2 publishes storyboard event states for captions.
3. [x] **Route.** The Town01 route and the six sites, as poses in a scratch file; one drive
   of the bare route (no incidents) timed. Cruise: **15 km/h kept** (user decision
   2026-10-14) -- a ~1.2-1.4 km route, sites ~200 m apart; raising the cap is left for a
   later tour.
4. [x] **Incident templates** 1-6, each runnable alone; 5/5 rehearsal each; drop 6 if the
   cyclist does not track. Placed at the step 3 sites (route order 1, 3, 2, 4, 5, 6). The
   Town01 map has **no `turn_direction` tag on any lanelet**, so Autoware's `intersection`
   and `blind_spot` modules never launch on it. **Decision (2026-10-14): tag them.** First
   (4a) `scripts/repair_lanelet_turn_direction.py`, the fourth repair pass beside the other
   three: tags every lanelet that lies inside a junction (OpenDRIVE junction connector:
   a driveable lanelet that leaves a fork -- a predecessor with more than one successor --
   or joins a merge -- a successor with more than one predecessor; Autoware's map
   requirement vm-03-02 asks for the tag on every lanelet in an intersection, and nothing
   else is needed to launch the modules) `turn_direction=left|right|
   straight` from its heading change, text insertion, idempotent, `.orig` kept; wired into
   `download_maps.sh` and the user guide; pytest on a small synthetic map; verified on
   Town01 by listing the route's junction lanelets and by Autoware launching
   `intersection` / `blind_spot` at them (planning debug or `/planning/scenario_planning/
   status/...` module states). Then re-run the Town01 starters once (the modules now slow
   the ego at junctions) and note any change. **4a done (2026-10-14)**: 72 Town01 lanelets
   tagged, both modules launch at the route's junctions, starters 6/6 unchanged (Results).
   Then the incident templates. **4b done (2026-10-14)**: incidents 1-5 in
   `src/csb_examples/scenarios/demo/incidents/` (generated by `scripts/gen_demo_incidents.py`),
   5/5 each, judged by `csb_launch incident_judge`; **6 dropped** -- the bicycle tracks, but
   Autoware turns across it (Results).
4d. [ ] **Revised design (2026-10-18).** This doc section ("The tour, revised"); then
   - acb: `max_velocity` launch argument (generated `common.param.yaml` copy, default 4.17,
     sync-script check), pushed and pinned;
   - sites: `scripts/demo_town01_sites.yaml` re-cut for route C (incident 2 on the west road,
     incident 4 alone on the east road), triggers rescaled for 40 km/h, templates regenerated;
   - each template 5/5 on **CenterPoint + overlay at 40 km/h**, ClearNoon; incident 6 back in
     the judged set. If CenterPoint does not hold 1-5, stop and take the fallback question to
     the user.
5. [ ] **Compose the tour** (`src/csb_examples/scenarios/demo/town01_tour.xosc`): route,
   triggers, add / delete, watchdogs, the single stop trigger. One ManeuverGroup per
   incident holding its add, delete and watchdog events: a `StoryboardElementStateCondition`
   naming an event in another ManeuverGroup by its bare name fails the load (step 2), and the
   runner then ends in ~2 s with no junit and no log line. (Step 4b: actors that move
   differently need groups of their own -- a group's private actions apply to all its actors
   -- and those groups are tied by global parameters, which every group sees; compose the
   tour from `gen_demo_incidents.incident_groups`.) Load-check every edit with
   `openscenario_preprocessor`'s `load` service (it returns the reason) before a full run.
   Remove the ~300 ms frame stall each NPC despawn costs (step 2: `sensor_release` waits out
   its timeout for an ack nobody sends for an NPC).
6. [ ] **Captions** (`demo_captions`, subscribing `/simulation/context` in domain 9) and a
   `demo` launch / justfile recipe bringing up the layout. Per-incident numbers from the
   bridge: see the step 2 results for what it has and what it would need.
7. [ ] **Rehearsal**: 3 full tours; one recorded take; README / tutorial 4 updated.
8. [ ] **Ambient layer** (between 5 and 7): the groups in "Ambient traffic and walkers" as
   their own ManeuverGroups in the tour (generated, unique names), walker polylines fitted to
   CARLA's sidewalks (`get_waypoint(..., lane_type=Sidewalk)`), each segment checked to cause
   no ego stop or slowdown outside an incident (planning factors), and the bridge's frame
   time at the densest point within budget. Rehearsal (7) runs with it.

## Later

A second tour on a multi-lane town with crosswalks (Town05 or Town10) once a drive spike
there passes: crosswalk yield, cut-in, lane change, blocked lane passed by a lane change.
Ramp merge with 031's highway maps.

### Results

**Step 2 spike (2026-10-14).** Town01, the running bridge (`just run`, 8084, current build), a
fresh `just ego-av`, three runs of two scratch scenarios (not in the repo): `spike_route_delete`
(a four-waypoint ~600 m loop with three turns: road 4 west, left at x 92, road 12 east, left
up x 339; `npc_a` added at waypoint 1 and deleted at 2, `npc_b` added at 3 and deleted at
the 500 m mark) and `spike_readd` (`npc_a` added, deleted, added again under the same name, 5 s
apart). A read-only CARLA client polled `get_actors()` at 5 Hz; an rclpy subscriber logged
every storyboard state change.

| Question | Answer | Evidence |
|---|---|---|
| `DeleteEntityAction` mid-run | **Works.** SSv2 sends `DespawnEntity`; the bridge stops and destroys the tracked NPC's own collision sensor (`collision_monitor.detach_npc`, 032 step 5), announces the release, then `destroy_with_children()`. The run carries on (verdict PASS). | bridge: `Despawned 'npc_a' (actor_id=8945)` at frame 1832, `'npc_b' (8947)` at 2790. CARLA: vehicle and its `sensor.other.collision` (8945 + 8946, 8947 + 8948) left `get_actors()` on the same frame; a fresh client's `get_actors([ids])` returned `[]` and `get_actor(id)` `None` for all. (The polling client's own `get_actors(ids)` still returned them, alive-flagged: its episode cache, not the server.) |
| ... its cost | **One ~300 ms frame stall per despawned NPC.** | `Frame 1832 processing over 25 ms: processing 301.83`, `Frame 2790 ... 301.63`, `Frame 255 ... 412.23`. That is `sensor_release.timeout_ms: 300` paid in full: nothing acks an NPC's release (acb only cares about the ego). Six hitches in a tour, visible as a brief freeze on camera. Fix in step 5: skip the announcement for an NPC no sensor bridge has claimed, or have acb ack ids it does not hold. |
| Re-add a deleted name | **Not possible: SSv2 refuses it.** | junit: `SemanticError: Applying action AddEntityAction to an entity "npc_a" that has already been added.` `DeleteEntityAction` never clears `ScenarioObject::is_added` (`add_entity_action.cpp`). The bridge had cleaned up; CARLA was empty afterwards. Plan: a unique entity name per incident actor. |
| Multi-waypoint `AssignRouteAction` (Init) through the relay | **Works; no stop at intermediate waypoints. Chained goals not needed.** | SSv2 -> `SetRoutePoints` (goal = last waypoint, `waypoints` = the first three) -> relay `set_goal -> OK` -> agent `goal set: (338.83, -107.76)` -> Autoware `/planning/mission_planning/route`: start (299.2, -129.5), goal (338.83, -107.76), **9 segments covering the whole loop**. Ego speed never below 0.3 m/s between first move and goal; at the four waypoints 4.16-4.17 m/s (passed within 0.2-1.1 m). |
| ... timing | 604 m in **152.6 s** (first move to the goal check point), cruise **4.17 m/s = 15 km/h**, ~3.0 m/s through the turns, average 4.0 m/s. | set_goal OK -> first move 7.2 s. Waypoints at +24.8, +77.8, +102.0 s, the 500 m mark at +126.1, goal at +152.6 s. 15 km/h is the stack's usual Town01 cruise (020 proof screenshots); see "The route and the clock". |
| Caption source | **`/simulation/context`**, `openscenario_interpreter_msgs/msg/Context` (`stamp`, `data` = JSON string, `time` = simulation time, NaN before the start), published by `/simulation/openscenario_interpreter` in the **scenario domain (9)**, QoS reliable + transient-local depth 1, **every frame (20.0 Hz measured; `global_frame_rate` 20)**, ~4.5 KB. | Live subscriber on domain 9 with `config/cyclonedds-localhost.xml`. The JSON is the whole storyboard: `CurrentStates` (counts per state), then `OpenSCENARIO.Storyboard.Story.Act.ManeuverGroup.Maneuver.Event[]`, each with `name`, `currentState`, `currentExecutionCount`, `maximumExecutionCount`, `Action[]` (`name`, `currentState`, `type`) and `StartTrigger` conditions (`currentValue` and a human-readable `currentEvaluation`, e.g. the simulation-time comparison). |
| ... what an event looks like | An event of instant actions (add, delete) goes **`standbyState` -> `completeState` in one frame**; `runningState` is seen only when an action lasts (`exitSuccess`: `runningState` / `startTransition`). | `wp1_add_npc_a` complete at sim 26.55 s, `wp2_delete_npc_a` 79.1, `wp3_add_npc_b` 103.05, `wp4_delete_npc_b` 127.0, `goal_reached` running at 153.75; CARLA's actors appeared / vanished in the same frames. So `demo_captions` keys on an event's change to `completeState` (or a rise of `currentExecutionCount`), by event name; an incident's "start" caption is its add event completing, its "end" the delete event. |
| What the bridge can report per incident | Today, nothing per incident: per **ego run** it logs a CARLA collision summary (`CARLA collision summary for ego 'ego': no collisions`), per tracked NPC a tracking line (`Tracked 'npc_a': 1051 frames, error p95 0.000 m ...`, written at the ego's despawn, not the NPC's), and with 032's sensors any NPC contact by CARLA frame. | It already sees every entity's pose each frame (`UpdateEntityStatus`), so it can compute per NPC entity, from spawn to despawn: minimum ego-to-NPC gap (bounding boxes), ego speed at that moment, ego stopped time, contacts. A despawn is the incident's end, so the natural output is one line / message per despawned NPC (e.g. a `csb_interfaces` topic in domain 9 beside the weather services); stop distance follows from the minimum gap while the ego is stopped. A step 6 item. |

Also learnt: a scenario that fails to load ends in ~2 s with no junit and no log line --
`scenario_test_runner` drops the `openscenario_preprocessor/load` response's `message`. Here
it was a `StoryboardElementStateCondition` naming an event in another ManeuverGroup by its bare
name (`No such variable named "ev_delete_again" of type StoryboardElement.`), found by calling
the `load` service on a private domain.

Runs: `spike_route_delete` PASS (1 run); `spike_readd` twice, the first failing at load (above),
the second ending at the re-add with the `SemanticError`. Scratch: `/tmp/claude-1000005/
-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-80a742b40cbd/scratchpad/s033_2`
(scenarios, generator, watchers, per-run logs).

**Step 3: route (2026-10-14).** One Town01 route, **1307 m**, four waypoints plus the goal,
picked from the Lanelet2 map (a lanelet graph walk) and checked against CARLA's
`get_waypoint`: every route lanelet's centreline is on CARLA's driving-lane centre to 0.00 m.
Start on the west edge road heading south (lanelet 24444, (-2.03, -149.96)); left round the
bottom-left corner; east along the bottom road (9379); left at the bottom-right corner, north
up the east edge road (13397, 309 m straight); left at the top-right corner, west along the
top (149); **left** into the x 337 road (35653) and south down it through three junctions;
**right** at y -198 (39793) onto the y -198 road west (21828) to the goal at (175.67, -195.15).
Waypoints on 9379, 13397, 149 and 28257. Autoware's `/planning/mission_planning/route` was
exactly the 19 lanelets planned (24444 31089 7768 36717 9379 38294 11783 23145 13397 19510 149
35653 27676 40484 28257 42063 28958 39793 21828). Three signals on the way (43751, 43784, 43793)
are not commanded and stay green.

| Route order | Incident | Site | Ego trigger (route m, lanelet) | Actors |
|---|---|---|---|---|
| 1 | 1 lead car merges, brakes | bottom-left T (x 90): ego straight through (36717) | 212 m, 7768 | `i1_lead` on the stem 33476 s 80, turns left (37233) into 9379 ahead; brake at 9379 s 55 (334 m); 224 m straight after the T |
| 2 | 3 red light | bottom-right T (x 337), straight-through approach | 429 m, 9379 | signal **43757** (reg 43852 on 9379; CARLA 373), stop line at 503 m |
| 3 | 2 pedestrian dart-out | east edge road, NB straight, kerb on the right | 601 m, 13397 | `i2_parked` 13397 s 80 at the kerb, `i2_ped` behind it (s 84, 3.6 m right); ped at ~665 m |
| 4 | 4 blocked lane | same straight, 160 m on; oncoming lane 15257 | 761 m, 13397 | `i4_broken` 13397 s 240 (821 m); oncoming on 15257 |
| 5 | 5 crossing traffic at a turn | top-right T: ego turns **left** from the through road (149) into the stem (35653), unsignalled | 871 m, 13397 | `i5_cross_a/b` EB on 2113 (s 95 / 60) through 35576 |
| 6 | 6 cyclist on the inside | T at x 337, y -198: ego turns **right** (39793) | 1082 m, 42063 | `i6_cyclist` SB on 28958 s 0, 1.4 m right of centre, straight on (39961) |

Spacing: brake 334 m, red light 503, pedestrian ~665, blocked lane 821, left turn 945, right
turn 1142 -- 125-200 m apart, not the 200 m hoped: 3 must sit at a signalled straight approach
and 5 / 6 at the route's only left and right turns into a stem, which fixes their places.
The order changed from the table's (1, 3, 2, ...): incident 2 needs a straight without
junctions and the only one after 3 is the east edge road. Exact poses (lanelet, s, world
x / y / yaw for each trigger and actor): `route_sites.yaml` in the scratch directory below.

**Timing (one bare drive, Autoware via the relay, no incidents): PASS.** set_goal -> first move
6.6 s; first move -> goal **321.4 s of simulation time (5.4 min), 1299 m, 4.04 m/s average**,
cruise 4.17 m/s, slowest 2.07 m/s in the right turn at y -198; **no stop** (no sample under
0.5 m/s between first move and goal); every site's trigger, turn entry and stop line passed
within 0.2-1.3 m (0.5 s sampling). Site passing times (s after first move): 52 (1 trigger),
81 (1 brake point), 105 / 122 (3 trigger / stop line), 147 (2), 186 (4), 211 / 231 (5 trigger /
turn), 265 / 279 (6 trigger / turn). With ~1-2 min of incident handling the tour is
**~6.5-7.5 min**, inside 5-8; no re-drive needed.

Wall clock is longer than simulation time when the host is loaded: this drive took 376.7 s of
wall time (real-time factor 0.85 overall, **0.19-0.42 for about 70 s** while other users'
builds held the disk: load 120-180, `/proc/pressure/io` some avg60 80). The demo is watched in
wall time, so rehearse and record on a quiet host; on this one, as loaded, the tour would run
~8 min.

Found on the way:
- **No `turn_direction` on any Town01 lanelet** (0 of 300; neither the repaired map nor
  `.orig`). Autoware's `intersection` and `blind_spot` modules key on it, so incidents 5 and
  6 as written cannot be handled by them; step 4 now says so.
- The map's `local_x` / `local_y` tags are not where the stack drives on east-west roads: they
  are stretched 0.67 % in y (2.2 m off CARLA's lane on the bottom road, 1.3 m on y -198, 0 on
  the top road), while the ego (this drive and the step 2 spike) runs on the lanes' projected
  lat/lon, which match CARLA to 0.00 m. Site poses here use the projected lat/lon; any tool that
  reads `local_*` (as `repair_lanelet_local_coords.py`'s note says Autoware does) puts things
  on the bottom road at the kerb. Not chased further here.

Runs: `tour_bare` PASS (1 run; junit 0 failures). Load-checked first with
`openscenario_preprocessor`'s `load` (success). Scratch: `/tmp/claude-1000005/
-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-80a742b40cbd/scratchpad/s033_3`
(`route_sites.yaml`, the map tools `ll.py` / `route.py` / `sites.py`, `gen_bare.py`,
`tour_bare.xosc`, `analyse.py`, `run1/` with the CARLA track, Autoware route and timing).

**Step 4a: `turn_direction` (2026-10-14).** `scripts/repair_lanelet_turn_direction.py`, the
fourth repair pass (wired into `download_maps.sh` beside the other three and listed in the
user guide; `src/csb_launch/test/test_repair_lanelet_turn_direction.py`, 6 tests on a
synthetic fork-and-merge map, run by `just test`).

*The rule.* Autoware 1.5 (universe source read at 0.40, same code path) launches
`intersection` on any path lanelet whose `turn_direction` is `straight` / `left` / `right`
(`IntersectionModuleManager::launchNewModules`; anything else reads "else" and is skipped)
and `blind_spot` on `left` / `right`. Nothing else is required: no `right_of_way` means every
conflicting lanelet is an attention lane, a missing stop line is derived from the first
conflict, `intersection_area` is optional (`use_intersection_area: false`). The map
requirements (vm-03-01/02) want the tag on *every* lanelet in an intersection. Junction
lanelet = a road/highway lanelet that leaves a fork (a predecessor with > 1 successor, the
doc's first heuristic) **or** joins a merge (a successor with > 1 predecessor), succession
read as lanelet2 reads it (shared bound end points). In Town01/02 both clauses pick the same
set; the merge clause adds turn-only connectors in Town03/05/10. Direction from the
centreline's heading change entry -> exit, threshold **45 deg** (bisector of straight and a
right angle; every town's junction lanelets sit at 0 +- 20 and 90 +- 20 deg), geometry from
lat/lon. Known misses, in the docstring: straight-only middle lanes through multi-lane
junctions (6 in Town03, 7 in Town05) stay untagged; mid-road lane adds tagged as junctions
(15 in Town03); Town01's 11 converter-malformed connectors (stray first point, no
predecessor, unroutable anyway) stay untagged.

*Counts* (map pack, local only; second run tags 0, diff is insertions only):

| Town | tagged | straight | left | right |
|---|---|---|---|---|
| Town01 | 72 | 24 | 24 | 24 |
| Town02 | 48 | 16 | 16 | 16 |
| Town03 | 183 | 80 | 44 | 59 |
| Town05 | 248 | 116 | 64 | 68 |
| Town10 | 71 | 26 | 19 | 26 |

Town01 is twelve T junctions x 6 connectors. Checked against CARLA (read-only): all 72 match
a CARLA `get_topology()` junction segment turning the same way; of the 81 untagged 4 m road
lanelets, 79 have their midpoint outside any CARLA junction, the other 2 are malformed
connectors. Route junction lanelets: **36717 straight** (bottom-left T, site 1), **38294
straight** (bottom-right T, signalled, site 3), **35653 left** (top-right T, site 5), **40484
straight**, **42063 straight** (x 337 road), **39793 right** (y -198, site 6); CARLA yaw
change 0 / 0 / +90 / 0 / 0 / -90 deg, junction ids 87 / 143 / 26 / 222 / 278 / 194.

*Live* (a fresh `just ego-av` on the tagged map, `tour_bare.xosc` once; a watcher on domain 1
logging `/planning/cooperate_status/{intersection,intersection_occlusion,blind_spot}`,
planning factors and `/control/command/turn_indicators_cmd`): **PASS**. One `intersection`
(+ `intersection_occlusion`) instance at each of the six route junctions, created ~4 m before
each and gone after it, all `safe` throughout; **`blind_spot` at both turns** (35653 left,
39793 right). Turn signals now blink: left 228-246 s, right 277-295 s (wall time from the first odometry).
Timing: first move -> goal **322.2 s** (step 3: 321.4), 1300 m, 4.04 m/s average, **no
stop** (no sample under 0.5 m/s), slowest 1.89 m/s in the right turn (2.07 before); every
site passed within 0.9 s of step 3's times. No planning factor from either module
(nothing to yield to in an empty town; `occlusion.enable: false` in autoware_launch 1.5).

*Starters* (`run_suite` over the six Town01 `basic/` scenarios on the same ego stack): **6/6
PASS** (ego_drive 40 s, engage_state 44, pedestrian 129, rear_contact 72, simulator_autopilot
51, traffic_light 185), as before 4a; their routes passed 2 `intersection` and 1
`blind_spot` instances, no stop added. `just test`: 300 nextest + 169 pytest pass.

Scratch: `/tmp/claude-1000005/-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-
80a742b40cbd/scratchpad/s033_4a` (`pre/` maps before 4a, `survey*.py`, `check_topology.py`,
`module_watch.py`, `analyse_modules.py`, `run1/` tour, `starters/` suite).

**Step 4b: incident templates (2026-10-14).** Incidents 1-5 as standalone templates in
`src/csb_examples/scenarios/demo/incidents/` (`town01_i1_lead_brake.xosc` ...
`town01_i5_cross_turn.xosc`, installed with `csb_examples`), generated by
`scripts/gen_demo_incidents.py` from `scripts/demo_town01_sites.yaml` (every pose, from the
map's projected lat/lon along the step 3 route). Each puts the ego ~125-145 m before its
site (60-110 m before its trigger), routes it through the site to a goal beyond it, adds the incident's actors when
the ego arrives and deletes them when the incident is over, has a watchdog per actor group
(ego stood still 20 s while live: actors deleted, `test(iN, watchdog, degraded)` in the
interpreter log; incident 4's is the staged 12 s "tow truck", logged `staged`), logs any
contact with `CollisionCondition` (never the stop trigger), and stops only on its goal or
a 300 s timeout. `gen_demo_incidents.incident_groups(n, sites)` returns an incident's entities and
groups, so step 5 composes the same pieces. Judge: `ros2 run csb_launch incident_judge
<trace> --incident N --context <context log>` (new; reads the gap trace for CARLA's bodies
and contacts and a `/simulation/context` state log for when each event ran, aligned on the
actors' spawn frames; 5 unit tests). `just test`: 300 nextest + 174 pytest pass.

*Rehearsal*: a fresh `just ego-av` (restarted after ~40 min), the sim side restarted on the
rebuilt bridge with `GAP_TRACE_DIR` for the runs and untraced again after; 5 runs per
incident, interleaved; `gap_oracle` on every trace.

| # | Pass | Numbers (5 runs) | Module (planning factor) | Oracle |
|---|---|---|---|---|
| 1 lead brakes | **5/5** | lead's peak deceleration **5.9 m/s²** (CARLA body); brake fired with the ego 13.4-13.6 m behind at 3.3 m/s; **stop gap 4.8-5.3 m**; ego stood 5.7-5.9 s, then followed the lead off | `obstacle_stop` | clean (G1 0, G2 0, G3 0) |
| 2 pedestrian | **5/5** | ego stopped every run; **min distance to the pedestrian 1.2-4.2 m**; stood 1.4-4.3 s; passed the parked car at 0.37 m (slowing) | `obstacle_stop` (not `run_out` / `dynamic_obstacle_stop`) | G1 0, G2 0; **G3 2 per run** on `i2_ped`: 0.27 m for 7 frames as the walker starts from rest, twice (new G3 kind, walker start-up lag) |
| 3 red light | **5/5** | front **0.08 m past to 0.48 m short of the stop line** at its furthest while red; green 8.1 s after it stood; moving 0.9-1.1 s after green | `traffic_light` | clean |
| 4 blocked lane | **5/5** | **stop gap 4.7-5.3 m** behind the broken-down car; waited 12.0 s (staged clear), moving 1.2-1.3 s after; oncoming traffic passed at 1.66 m | `obstacle_stop`; no avoidance attempted | clean |
| 5 crossing at a turn | **5/5** | stood 3.5-3.6 s for the pair, then turned; **min gap 1.0-1.2 m** (the ego stops close to the crossing lane) | not observable: `intersection` / `intersection_occlusion` / `blind_spot` RTC `safe` throughout, no planning or velocity factor | clean |
| 6 cyclist | **dropped** | 2/2 shakedown runs: contact, the ego turned right across the cyclist (min gap -0.6 / -0.7 m); one run ended in SSv2's acceleration bound (-19 m/s², the ego knocked) | `blind_spot` launched at 39793, `safe` the whole time; no factor, no slowing | cyclist tracking clean (G3 0): not a 022 problem |

Wall time per run 83-179 s (each includes SSv2 start-up and engage). No watchdog fired in the
25 runs except incident 4's staged one; no contact.

*Adjustments* (the Incidents table is updated): 1 -- the lead merges, slows to 3 m/s and brakes
when the ego is within 12 m freespace (or at 9379 s 70), then drives off once the ego has
stood 4 s; 2 -- the parked car stands 2.4 m right of the lane centre (half on the kerb), the
pedestrian steps out when the ego's origin is 24 m from its line (17 m was too late: a contact
at ~0 m/s in the shakedown), stops in the lane 4 s, then crosses; 4 -- no hazard lights (no
light action on this path), cleared after 12 s; 5 -- the two crossing cars 15 m apart, not 35
(at 35 the ego took the gap and was hit by the second, shakedown); 6 dropped. Triggers moved
for timing: 1 at route 205 m, 5 at 907 m (`demo_town01_sites.yaml`).

*SSv2 behaviours found (each cost a run; all handled in the generator, documented in it)*:
- `StoryboardElementStateCondition state="standbyState"` is **true for good**: the condition
  remembers standby from construction and only re-reads the element when that does not match.
  "Not yet happened" cannot be asked of an event; the templates keep each group's phase in a
  global parameter (`ParameterSetAction` / `ParameterCondition`, which every group sees --
  also the way to tie groups together in the tour).
- `StandStillCondition` never holds for the ego: SSv2 counts standstill only while
  |v| <= DBL_EPSILON, which a CARLA-physics ego never reports. Replaced by a chain of delayed
  `SpeedCondition < 0.1` (every 2 s).
- Deleting an entity while one of its actions still runs (a trajectory) ends the scenario
  (`Entity "i2_ped" does not exist`).
- A `SpeedAction` with `rate 6` is jerk-limited to ~3.7 m/s² peak (default
  `max_deceleration_rate` 3 m/s³, and every linear speed change resets the entity to the
  default constraints when it ends, so a controller property does not stick); the brake is a
  `SpeedProfileAction` with `DynamicConstraints` (6 m/s², 60 m/s³), measured 5.9 in CARLA.
- A walker on a timed `FollowTrajectoryAction` started late and then failed the scenario
  ("completed a trajectory to the nearest waypoint ... greater than this step distance");
  untimed vertices at a commanded speed work, and a hold `SpeedAction 0` on a walker before
  its trajectory kept it standing.
- An NPC added with a step `SpeedAction` (already moving) was, once in 3, declared blocked by
  the bridge before its body settled and handed to kinematic for the episode (032's
  fallback; 0 m/s along SSv2's 6 for 4 frames). NPCs are now added standing and driven off
  0.5 s later at 4 m/s².
- `openscenario_preprocessor`'s `load` writes `raw/` beside the file it loads: load-check a
  copy, not the repo's file.

*Found on the way (not fixed here)*:
- **A leaked vehicle in CARLA.** The first incident-1 run's ego spawn took 31 s and reported
  failure, but CARLA had created it: actor **9004** (`vehicle.tesla.model3`, `role_name=hero`)
  stands at CARLA (-1.98, 261.46), route m 110 on the west road; the retries then spawned the
  ego on top of it (contact, run failed). The bridge does not destroy an actor whose spawn
  call timed out. *Correction 2026-10-15*: the next `Initialize`'s reaper destroyed it at
  04:28:36Z ("Destroyed orphaned 'hero' (actor 9004)"), and a by-id query finds no 9004;
  moving incident 1's start to route m 125 was harmless but unneeded. Cause and fix:
  [034](034-bridge-failure-paths.md) step 4 (a timed-out spawn completes on the server).
- The acb submodule pin had moved (3883f86) without a rebuild: rebuilt acb's workspace, then
  this one, before the runs.

Scratch: `/tmp/claude-1000005/-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-
80a742b40cbd/scratchpad/s033_4b` (`poses.py`, `run_one.sh` / `batch.sh`, `loadcheck.sh`,
`aggregate.py` and `agg_reh.txt`; `i<N>/reh_<k>/` the rehearsal runs with trace, context and
module logs, judge and oracle output; `shake*`/`pre_*` the shakedowns; `traces/`).

**Step 4c: the two open items of 4b (2026-10-17).** Plan first, results below it.

*A -- incident 2's G3 on `i2_ped`.* The 4b traces (reh_1, frames 672-692 and 813-830) show it
frame by frame: SSv2's walker goes from 0 to **1.50 m/s in one frame** (0.075 m per 0.05 s
step from the first moving frame) at the dart and again at the cross -- `speed()` in the
generator is a `step` SpeedAction. The bridge commands the walker at once (`WalkerControl`,
SSv2's velocity plus 6/s of the position error, no force bound on walkers), and CARLA's
walker body ramps 0.08, 0.19, 0.33, 0.49, 0.68, 0.88, 1.10 ... m/s, peaks 2.42 m/s to win the
lag back and settles in ~1 s; the error peaks 0.27 m at the 7th-8th frame. So the body
cannot follow an unbounded acceleration and the bridge already asks for all it can give:
**a scenario artefact of 032 N6's kind (a speed step), not a tracking gap**. Fix in the
generator: the walker's start speeds become `linear` at a pace a person (and CARLA's walker)
starts with, and the dart trigger moves out by what the ramp costs so the incident's timing
holds. Accept: `town01_i2_*` 3+ runs, 0 G3, judge PASS.

*B -- why `blind_spot` did not react to the cyclist.* Read in Autoware 0.48.0's module (the
1.5.0 package; `scene.cpp`, `util.cpp`): it keeps an object only if
`classification[0].label` is BICYCLE, PEDESTRIAN or MOTORCYCLE (CAR / TRUCK / BUS / TRAILER
optional by `detect_non_vru`, all false; **UNKNOWN never**), whose *centre* lies in the
attention area -- the strip from the ego path's right edge (path + ego width / 2 = 1.08 m)
to the right bound of the rightmost lane (or a right shoulder / bicycle lane if the map has
one) over the 100 m before the turn, carried through the junction -- and whose predicted
path crosses the turning path's right bound / entry line and then its left bound within
the TTC margins (2 s / 1 s) of the ego's own passage. The stack's perception is
`lidar_detection_model: clustering`, which labels what it finds UNKNOWN; incident 2's
walker was stopped for by `obstacle_stop`, never `run_out` (which also wants a VRU label),
consistent with that. Hypothesis: **the cyclist reaches planning as UNKNOWN and the module
filters it out before any geometry**. Check on a live run: the labels on
`/perception/object_recognition/objects`, the attention area (`.../behavior_velocity_planner/
debug/blind_spot` markers) against the cyclist's position, and
`~/debug/blind_spot/decision_state`. If it is the label: the map and the scenario cannot
fix it; the fix is a perception that classifies (or ground truth that labels a bicycle
BICYCLE -- acb's ground-truth path today labels every `vehicle.*` CAR).

*Results A (2026-10-17): a scenario artefact, fixed in the generator.* `WALKER_START_RATE =
1.5` m/s² on both of the walker's starts (`i2_ped_walk`, `i2_ped_walk_on`). SSv2 follows it
exactly (0.075 m/s per frame); the CARLA body still starts slowly (0.01-0.10 m/s per frame
over the first 10 frames, its own ramp: the bridge's command is already SSv2's speed plus
6/s of the error) and the walker's largest error is now **0.13 m** (was 0.27), under G3's
0.2. Not a bridge gap, so no bridge change: a walker's `WalkerControl` carries no force
bound to lift, and the lag is the body's. The ramp delays the pedestrian's arrival in the
lane by ~0.5 s and its clearing of the ego's path by ~0.4 s, which the timing felt: with
the dart trigger moved only 24 -> 26.5 m (the arrival delay at 4.17 m/s) the first run
touched the pedestrian (`reh_1`, -0.16 m): the ego stopped 3.0 m short, stood 1 s, then
crept on just as the pedestrian started across. The 4b runs show the same creep after ~1 s
standing in every run (Autoware), so their 1.2 m worst case was the pedestrian leaving in
time, not the ego waiting. **Dart trigger 30 m** (origin to the pedestrian's line; route m
635.6): the pedestrian arrives ~1 s earlier relative to the ego and leaves as it stops.

Runs (same stack, sim side traced): `reh_1` (26.5 m) FAIL contact, G3 0; at 30 m `d30_1`-
`d30_5` **4/5 PASS**, pedestrian min distance 1.12 / 3.75 / 2.43 / 2.00 m, ego stood
2.4-4.4 s, **G3 0 in all six runs**. The one failure (`d30_2`) is perception, not timing: the
ego slowed to 2.8 m/s and then accelerated into the standing pedestrian, which
`/perception/object_recognition/objects` reported only intermittently between 10 and 4 m
(clustering: present 877.88, gone 878.48-879.08, back at 879.38 -- the walker standing
still in the lane flickers in and out; a walking one does not). That failure mode exists
for any timing in which the pedestrian stands in the ego's lane while it approaches; it is
the incident's residual risk and is recorded here, not fixed (Autoware perception).

*Results A, the flicker (2026-10-18): Autoware's tracker parameters, fixed in acb.* Traced
stage by stage on live runs (bag in the ego domain, the walker's CARLA body per tick): the
lidar, ground removal, map filter, clustering and the obstacle pointcloud validator have the
standing walker in **every** frame; `multi_object_tracker` drops it. Its overlap pruning
merges two UNKNOWN trackers within 9 m when their generalized IoU (over the footprints'
convex hull) exceeds `pruning_generalized_iou_thresholds[UNKNOWN]`, -0.3 in Autoware 1.5.0
-- so pairs that do not overlap at all. The walker stands 1.4 m from the parked car's front
corner; GIoU of the car's track and the walker's detection hovers at -0.27..-0.37, and the
walker's track is published exactly when it is below -0.30: each crossing merges the young
walker track into the settled car track, and the next detection starts a new one -- the
flicker. Not CARLA (points on the walker 544 per frame), not the cluster size or the
validator. `obstacle_stop` stops for objects only (`check_inside.pointcloud: false`) and
holds a lost one 1 s, so a gap longer than that releases the stop. Upstream moved the
threshold to 0.1 itself (autoware_launch #1885, then #1907). Fix: acb 7429bbc (acb issue
027), acb_launch's generated copy of the tracker's parameters with UNKNOWN at 0.1, no code
change. Walker present while standing in the lane 2-30 m ahead: **0.30 before (69/230
frames, five bagged runs at 0.22-0.35; one contact in six runs) -> 1.00 after (276/276, six
runs, no contact, the ego stood 4.9-5.2 m short)**. AEB 9/9, starters 6/6, incident 6
unchanged (contact 2/2, as before).

*Results B (2026-10-17): Autoware, twice over; i6 stays a documented reproducer.*
1. **The cyclist never reaches `blind_spot` as a cyclist.** Live (`probe_1`, the stock
   stack): every object on `/perception/object_recognition/objects` is label 0
   (**UNKNOWN**, 84 603 of 84 603), and the cyclist's twist is 0 with a single predicted
   path ending where it stands (`multi_object_tracker`'s
   `enable_unknown_object_motion_output: false`). `blind_spot` keeps only BICYCLE /
   PEDESTRIAN / MOTORCYCLE (CAR / TRUCK / BUS / TRAILER by `detect_non_vru`; UNKNOWN never),
   so it filters the cyclist out before any geometry: decision `Safe` from launch to the pass
   judge line (ego 5 m before the turn at 4 m/s), then `OverPassJudge`. Geometry is not the
   problem: the attention area is the strip x 332.72-333.64 (ego path + 1.08 m to the lane's
   right bound; no shoulder or bicycle lane on Town01) and the cyclist rides at x 333.1-333.2,
   inside it.
2. **With a perception that classifies, the module still misjudges a right turn.**
   `EGO_LAUNCH_ARGS=lidar_detection_model:=centerpoint` (no code change): the cyclist is
   BICYCLE at 5.2-6.6 m/s with map-based predicted paths from ~7 m behind the ego, yet
   `blind_spot` stays `Safe`: its debug `ego_passage_interval` reads e.g. `[5.57, 5.52]` --
   exit *before* entry -- every frame. The module times the ego between two lines, the ego's
   own straight-on edge (`virtual_ego_straight_path_after_turning`, x 333.46 here) and the
   blind side's road edge carried through the junction
   (`virtual_blind_lane_boundary_after_turning`, x 328.36); for a **left** turn it takes
   entry at the first and exit at the second, for a **right** turn it swaps them
   (`scene.cpp` / `experimental/scene.cpp`, `first_line` / `second_line`), so on a right turn
   "entry" is the last moment and the interval collapses to the end of the turn; the
   cyclist's own passage (e.g. [2.8, 4.9] s, read in the fixed runs) then ends more than the 1 s
   margin before it and the module reads the
   cyclist as gone. The object side is mirrored correctly; only the ego side is not. The
   same lines are in Autoware 0.47.0, 0.48.0 (1.5.0's package), 0.49.0 and `main` as of
   2026-10-16 -- a right-hand-traffic bug in a module written for left-hand traffic, where
   the blind spot is on the left. (1.5.0 runs the module's `experimental` plugin.) Proof:
   the 0.48.0 module rebuilt in the scratch directory with the two lines in the left-turn
   order for both sides, overlaid with `EGO_OVERLAY`, on centerpoint: the interval reads
   `[5.13, 11.14]`, the decision goes `Unsafe` (collision 4.0 -> 1.4 s), and the ego stops
   ~4 m before the turn and lets the cyclist by: **3/3 PASS** (`cpfix_4`-`_6`, min gap
   0.13-0.15 m -- the cyclist passes the standing ego inside the same lane, which is the
   scenario's geometry -- ego stood 0.8-2.0 s, G1/G2/G3 0), against 2/2 contact on
   centerpoint with the stock module (`cp_2`, `cp_3`; `cp_1` ran before CenterPoint's
   engine finished building and saw no objects).

*Decision.* Nothing in csb, acb's sensor path, the map or the scenario causes this, and
neither fix belongs to this repo: incident 6 needs a classifying perception (CenterPoint
lost 7 of 9 UC-AEB variants on this host -- acb's launch note -- so the demo keeps
clustering) **and** a fixed `blind_spot`. Incident 6 stays out of the tour and out of the
judged set, generated to `scenarios/demo/town01_i6_cyclist_inside.xosc` as the reproducer
of both; the upstream report (Autoware universe, `blind_spot`: right-turn ego passage
interval uses the lines in left-turn-mirrored order) is a separate step, not filed from
here (filed since as autoware_universe#13509; the fix is carried by [042](042-autoware-overlay.md),
on by default, 3/3 PASS on centerpoint). Found on the way, not fixed: acb's ground-truth object path labels every `vehicle.*`
CAR, bicycles and motorbikes included (`ground_truth_objects.rs`; CARLA's `base_type`
attribute would give BICYCLE / MOTORCYCLE / TRUCK / BUS) -- irrelevant to the demo stack,
which runs real perception, but the same filter would drop a ground-truth cyclist.

*Verification (2026-10-17, a fresh stock `just ego-av`, sim side traced, then untraced
again).* Incidents 1-5, five interleaved rounds: **25/25 PASS** by `incident_judge`, every
trace G1 0 / G2 0 / G3 0, G4 agree; incident 2 pedestrian min distance 0.92 / 1.01 / 2.50 /
1.43 / 2.42 m, ego stood 3.1-3.9 s, the walker's largest SSv2-CARLA error 0.135 m in every
run (was 0.27). Starters (`run_suite`, six Town01 `basic/`): **6/6 PASS** (ego_drive 41 s,
engage_state 45, pedestrian 127, rear_contact 24, simulator_autopilot 48, traffic_light
184). Host load 7-82 (1 min) across the runs, no failure in a loaded one. `just test`:
329 nextest + 174 pytest pass; `just check` clean.

Scratch: `/tmp/claude-1000005/-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-
80a742b40cbd/scratchpad/s_inc` (`run_one.sh` / `batch.sh` / `rounds.sh`; `bs_watch.py`, the
objects / blind_spot debug / path watcher; `pedtl.py`, `objnear.py`; `i2/{reh,d30,fin}_*`,
`i6/{probe,cp,cpfix}_*` with trace, judge, oracle and `bs.jsonl`; `bsfix/`, the 0.48.0
module with the line order fixed and a `[bs_probe]` print, built for the overlay;
`starters/`).
