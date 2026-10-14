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

Numbers are ids; along the step 3 route they come in the order 1, 3, 2, 4, 5, 6. Each is a
self-contained **incident template** (a short `.xosc` that runs on its own under
`run_suite`, then is composed into the tour).

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
town stays empty between incidents and CARLA is never carrying a crowd. **Every actor gets a
name of its own** (`i1_lead`, `i2_ped`, ...): SSv2 never lets a deleted name be added again
(step 2 spike), so no name is reused across incidents.

## The route and the clock

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
4. [ ] **Incident templates** 1-6, each runnable alone; 5/5 rehearsal each; drop 6 if the
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
   Then the incident templates.
5. [ ] **Compose the tour** (`src/csb_examples/scenarios/demo/town01_tour.xosc`): route,
   triggers, add / delete, watchdogs, the single stop trigger. One ManeuverGroup per
   incident holding its add, delete and watchdog events: a `StoryboardElementStateCondition`
   naming an event in another ManeuverGroup by its bare name fails the load (step 2), and the
   runner then ends in ~2 s with no junit and no log line. Load-check every edit with
   `openscenario_preprocessor`'s `load` service (it returns the reason) before a full run.
   Remove the ~300 ms frame stall each NPC despawn costs (step 2: `sensor_release` waits out
   its timeout for an ack nobody sends for an NPC).
6. [ ] **Captions** (`demo_captions`, subscribing `/simulation/context` in domain 9) and a
   `demo` launch / justfile recipe bringing up the layout. Per-incident numbers from the
   bridge: see the step 2 results for what it has and what it would need.
7. [ ] **Rehearsal**: 3 full tours; one recorded take; README / tutorial 4 updated.

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
