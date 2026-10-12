# Phase 028: Run Tooling

Seven tools that make a run cheaper to prepare, cheaper to diagnose and easier to scale out,
ordered from the smallest to the largest. Each stands alone; the later ones reuse the
earlier ones (the map check feeds the lint and the generator, the lint guards the generator's
output).

**Source**: user direction 2026-10-13 (review of run tooling against what this project's
suites cost today).
**Depends on**: [017](017-user-workflow.md) (the agent protocol, the relay),
[020](020-starter-scenarios-and-suites.md) (`run_suite`, starter scenarios),
[021](021-physical-verdict.md) / [022](022-entity-geometry-parity.md) (contact report,
blueprint box table), [027](027-user-documentation.md) (tutorials, `carla_poses`,
`chase_view`).

## Why

What a suite costs today, from the 2026-10-13 sessions (023 step 5):

- **A broken scenario is found late.** An ego stack takes ~4 min to come up; a scenario that
  names a lanelet id the map does not have, or an entity no blueprint matches, fails only
  after that. UC-ACC's lane-id port (020) and the AWF ego box were found this way, by hand.
- **A failure arrives as one line.** UC-ACC variant 15's `velocity_limit` error took a log
  dig to trace to a crashed relay; seconds of video would have shown "drives on, then
  errors". Video exists only when someone records a screencast by hand.
- **Infrastructure faults cost whole reruns.** A `load_world` timeout on a loaded host
  failed `town02_episode_change` and the next Initialize; both passed on a rerun.
- **Scenarios are placed by hand.** Each `.xosc` sits at one place on one map; a bridge
  fault that appears only at some places (ground height, junctions, signals) is found only
  if someone happens to put a scenario there.
- **Only Autoware drives.** The relay's agent protocol (I3) is generic, but nothing outside
  acb implements it, and it is specified in a design doc, not offered to users.

## Design

### A. Scenario-writing skill (`.claude/skills/write-xosc/`)

A Claude Code skill in the repository that writes and reviews scenarios for this project:
the `.xosc` skeleton of tutorial 2, poses from `ros2 run csb_launch carla_poses`, body sizes
from `blueprint_boxes_0.9.16.yaml` (so a declared box matches a CARLA blueprint within 022's
tolerance), `csbExpectedContacts`, the `agent` / `simulator_autopilot` controllers, and the
TIER IV `ScenarioModifiers` form for parameter sweeps. A review mode checks a given file
against the same rules. Once the lint (D) exists the skill runs it on what it wrote.

Acceptance: from a one-paragraph use-case description (e.g. "a vehicle cuts in from the
left lane at 30 km/h, 10 m ahead"), the skill produces a Town01 scenario that passes the
lint and runs to a verdict under `run_suite`.

### B. The agent interface for users -- moved to 030

Moved to [030: Agent API](030-agent-api.md) (user direction 2026-10-13): the user page and
the reference agent are part of a Python agent SDK there, with reset and restart between
scenarios.

### C. Infrastructure retry in `run_suite`

`run_suite` classifies each error before recording it:

| Class | Matches (SSv2 JUnit message / bridge log) | Action |
|---|---|---|
| infrastructure | `load world ... time-out`, Initialize unanswered, CARLA RPC unreachable, the relay or bridge restarted mid-scenario (its log) | retry once after a cool-down, marked `retried="1"` with the first error kept |
| verdict | SSv2 failure, `carla_contact` failure | recorded, never retried |
| stack | `AutowareError` (a service, engage, localization) | recorded, never retried; the summary names the stack log to read |

A retry never hides a fault: the suite XML keeps both attempts' messages and the summary
line counts retries. `--no-retry` turns it off.

Acceptance: a unit test per class from captured JUnit and log snippets (the 2026-10-13
Town02 timeout and UC-ACC variant 15 among them); one live suite with CARLA paused for
longer than the load timeout retries and passes.

### D. Scenario lint (`ros2 run csb_launch scenario_lint`)

A dry run of everything about a scenario that can be checked without Autoware or a running
scenario, in seconds:

- the `RoadNetwork` map resolves (`$(env CARLA_MAPS)/...`) and holds a Lanelet2 map;
- every `LanePosition` names a lanelet of that map, with `s` inside its length and `offset`
  inside its width; every `WorldPosition` lies on or near a lanelet;
- every entity's declared box matches a blueprint within 022's tolerance (the same table and
  rule as the bridge, so the lint and the contact report cannot disagree);
- every traffic signal id the scenario commands is in the map and in the town's
  `traffic_lights.yaml`;
- `ScenarioModifiers` expand (every variant is linted);
- with `--carla HOST:PORT` (optional): each spawn pose has CARLA ground under it, read
  through the map check's report (E) when one exists, else probed directly.

`run_suite` runs it as part of its preflight (before the scenario stack starts) and refuses
a scenario with errors (`--no-lint` to override); warnings are reported, not refused.

Acceptance: every starter and `awf/` scenario lints clean; a set of broken copies (bad lane
id, `s` past the lanelet's end, unmatched body, unknown signal) each yields exactly its
error; a full `run_suite` on `basic/` adds under 10 s.

### E. Map check (`ros2 run csb_launch map_check`)

A per-town report of how the Lanelet2 map and the CARLA town agree, written next to the
map (`<map_dir>/carla/map_check.yaml`) and read by the lint and the generator:

- per lanelet: centreline offset to CARLA's nearest driving waypoint (p95 / max), heading
  difference, CARLA ground present under the centreline (downward ray every few metres);
- per traffic light: matched CARLA light and distance (today logged at each Initialize
  only);
- a summary line and a non-zero exit when a threshold is exceeded, so a converted map can be
  gated before anyone writes a scenario on it (the user guide's map-preparation section
  points to it).

Acceptance: Town01 and Town02 reports; a lanelet known to be off (if any) is flagged; the
user guide's "bring your own map" path ends with this check.

### F. Recorded runs, rendered on demand

The bridge records every episode with CARLA's recorder (`start_recorder` at the ego's spawn,
`stop_recorder` at its despawn; the file name joins the contact report's, under
`~/.ros/csb/recordings/`). After a scenario, `run_suite --video failed|all` renders the
recording to an MP4 next to the result: replay the log, follow the ego with the chase
camera of `chase_view`, write frames, encode. The run itself carries no camera.

Open questions, settled by a spike before anything is wired:

- **Replay on the shared server.** Replaying spawns the recorded actors, an ego with
  role name `hero` among them, in the world the live ego stack is attached to: acb must not
  attach to a replayed actor, and sensors must not be replayed onto it (never destroy a
  vehicle with sensors attached). The spike decides between replaying between scenarios on
  the same server with the ego stack's vehicle side paused, filtering the replay
  (`replay_file` with `follow_id`, `set_replayer_ignore_hero`), or a second CARLA instance
  for rendering.
- **Cost and disk.** Recorder overhead on the tick (016's frame budget), file size per
  minute; retention: keep the recording of a failed scenario, delete a passed one unless
  `--video all` (the log disk runs near full).

Acceptance: one failing and one passing scenario under `--video failed`: one MP4 for the
failure, none for the pass, no recordings left behind for the pass; tick p95 with the
recorder on within 1 ms of off.

### G. Location sweeps (`ros2 run csb_launch scenario_expand`)

One scenario template, placed at every place on a map that fits it:

- **Template**: a `.xosc` whose ego and NPC positions are `LanePosition`s with parameter
  lanelet ids (`$ego_lanelet`, `$npc_lanelet`, ...).
- **Query**: constraints on the ego's lanelet -- has a left / right neighbour, length at
  least L, not in a junction, turn direction, has a signal, in / not in a set -- and
  bindings for the others (the left neighbour, the successor, the lanelet N m ahead along
  the route), evaluated on the Lanelet2 routing graph (`lanelet2` Python, already in the ROS
  environment). Lanelets the map check (E) flags are excluded.
- **Output**: one concrete scenario per match (or a `ScenarioModifiers` list for small
  sweeps), each linted (D), plus an index of match -> file so a failure names its place.
- **Running**: `run_suite` over the output directory, as any scenario directory; with
  `--video failed` (F) each failing place comes with its video.

Acceptance: a cut-in template on Town01 expands to every qualifying lanelet, every case
lints clean, and a full sweep runs under `run_suite` with a per-place result table; any
place-dependent bridge fault it finds is logged as a gap (024's format).

## Steps

1. [x] **Docs.** This phase doc.
2. [ ] **A. Scenario-writing skill.** Skill, its review mode, the acceptance run.
3. [-] **B. Agent interface.** Moved to [030](030-agent-api.md).
4. [ ] **C. Infrastructure retry.** Classification with tests, `retried` in the suite XML,
   the paused-CARLA live check.
5. [ ] **D. Scenario lint.** Offline checks with tests, `--carla`, the `run_suite`
   preflight, broken-copy set.
6. [ ] **E. Map check.** Town01 and Town02 reports, the lint reading them, user guide.
7. [ ] **F. Recorded runs.** Spike on replay safety and cost; then recorder in the bridge,
   `--video`, retention.
8. [ ] **G. Location sweeps.** Template and query format, the expander with tests, the
   Town01 cut-in sweep.

The skill (A) is revisited after D to run the lint on what it writes.

## Acceptance

Each tool's acceptance above, and the user guide and tutorials updated where a user meets
the tool (lint and video in tutorial 3, the map check in
map preparation, sweeps in a section of their own).

### Results
