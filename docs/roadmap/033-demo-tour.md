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
3. [ ] **Route.** The Town01 route and the six sites, as poses in a scratch file; one drive
   of the bare route (no incidents) timed. Cruise: **15 km/h kept** (user decision
   2026-10-14) -- a ~1.2-1.4 km route, sites ~200 m apart; raising the cap is left for a
   later tour.
4. [ ] **Incident templates** 1-6, each runnable alone; 5/5 rehearsal each; drop 6 if the
   cyclist does not track.
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
