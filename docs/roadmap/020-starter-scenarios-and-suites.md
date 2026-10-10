# Phase 020: Starter Scenarios, Suites, and a First-User Proof Run

Close the gap between "installed" and "ran my scenarios": ship scenarios a user can run
without cloning anything, let one command run many scenarios and keep every verdict, and
prove the whole user guide works by having a fresh agent follow it as a first-time user.

**Source**: user direction 2026-10-10: "focus on completing the project for users ... we
can use CARLA maps and those converted for Autoware. Have starter scenario sets and fix the
output path so the many scenario usage can work. Last, do the proof run on the host ...
ask an agent to follow the instructions as though it's a first time user." SafetyPool
(awaiting database access) and map provisioning (its own effort) are deferred.
**Depends on**: [017](017-user-workflow.md), [018](018-install-and-world-config.md).
**Relates to**: [005](005-hardening-awf-contribution.md) (SafetyPool, setup guide, AWF
outreach).

## Problem

- **No scenarios without a clone.** `scenarios/*.xosc` live in the repository root and are
  not installed; a user with the built workspace has nothing to run.
- **One verdict, overwritten.** `scenario.launch.xml` always writes
  `/tmp/scenario_test_runner/result.junit.xml`; the next run replaces it. SSv2's runner also
  empties the contents of its output directory's subdirectories at startup
  (`scenario_test_runner.py:112`), so two runs cannot share one.
- **No way to run a set.** SSv2's runner takes one scenario; a suite means a hand-written
  loop over launches.
- **The guide is unproven end to end.** Every step was verified by the people who wrote it,
  on a machine they configured.

Maps stay as they are: the CARLA towns converted for Autoware (TUM pack, the
`data/carla-autoware-bridge/` set). The guide says where they come from; a map-provisioning
effort is separate.

## Design

### Starter scenarios: package `csb_examples`

A new ament_cmake package installing `share/csb_examples/scenarios/`:

- **`basic/`** -- this repo's Town01/Town02 scenarios: ego drive, traffic light, pedestrian,
  rear contact, engage state, simulator autopilot, episode change (Town02). Two-AV stays
  out of the default set (it needs a second vehicle side) but ships under `multi_av/`.
- **`awf/`** -- the AWF ODD WG use cases published in
  `autowarefoundation/odd-usecase-scenario` (Apache-2.0, attribution kept):
  `UC-ACC-001-0001` (preceding vehicle partial deceleration) and `UC-AEB-001-0001`
  (standing pedestrian), TIER IV YAML format with `ScenarioModifiers`. They are written for
  AWF's own `maps/Town01.osm`; this phase checks their lanelet IDs against our Town01 map
  and adapts positions if they differ (recorded per file). `LogicFile` becomes
  `$(env CARLA_MAPS)/Town01`.
- Every file names its map as `$(env CARLA_MAPS)/<Town>`; `CARLA_MAPS` is the user's map
  root. A `README.md` in the share lists each scenario, its town, what it checks, and how
  long it takes.

### Output path

`scenario.launch.xml` gains `output_directory:=` (default `/tmp`, today's behaviour). The
verdict is `<output_directory>/scenario_test_runner/result.junit.xml`, beside the
preprocessed copy and logs.

### Suites: `ros2 run csb_launch run_suite`

```bash
ros2 run csb_launch run_suite <scenario file or dir>... --output <results dir> \
    [--domain 9] [--timeout-per-scenario S] [-- <extra scenario.launch.xml args>]
```

- Runs each scenario in turn through `play_launch launch csb_launch scenario.launch.xml`
  (falling back to `ros2 launch` when play_launch is absent), each with its own
  `output_directory:=<results>/<scenario stem>/`.
- Waits for each launch to end on its own (the runner's `on_exit` shutdown), with a
  per-scenario wall-clock cap that kills that launch's process group and records a failure.
- Writes `<results>/suite.junit.xml`: one testsuite aggregating every scenario's testcases
  (a YAML scenario with modifiers contributes one testcase per variant), plus a one-line
  summary per scenario on stdout; exit code 0 only if all passed.
- Assumes the simulation side and the vehicle side(s) are up, as a single scenario does.

## Steps

1. - [x] `output_directory:=` on `scenario.launch.xml`; verified with a non-default path.
   2026-10-10: `OUTPUT_DIR=<scratch>/020-step1 just scenario .../town01_ego_drive.xosc` left
   its PASS verdict at `<dir>/scenario_test_runner/result.junit.xml`; `/tmp`'s untouched.
   `just scenario` passes `OUTPUT_DIR` (default `/tmp`), `two-av` reads it.
2. - [x] `csb_examples` with `basic/` and `multi_av/`; installed; README; guide points at it.
   `git mv` from `scenarios/` (which keeps `bench/` and the goal-pose files); justfile
   defaults, design docs and acb's `run_scenarios.sh` follow. Installs to
   `share/csb_examples/scenarios/{basic,multi_av,awf}` + `README.md`.
3. - [x] AWF use cases in `awf/`: lanelet IDs checked against our Town01, adapted, both run
   (all modifier variants) on the CARLA pipeline; results recorded.
   No shared ids; AWF 1071 = our 5230, same geometry to 0.01 m (by lat/lon -- our pack's
   `local_x/y` tags are 0.9 m stale), so ids remapped, `s` kept. Ego box placeholders set to
   acb_vehicle (upstream's put the car 1.39 m off the localization pose: 9/9 AEB failed
   before engage). Then AEB variant 5 failed on a -6.6 m/s^2 stop spike: acb locked the
   handbrake while the car still rolled at ~0.8 m/s (pipeline fault, not Autoware: it
   commanded <= 3.4); acb now holds only below 0.1 m/s. Final: ACC 27/27, AEB 9/9
   (`csb_examples/README.md`, `docs/proof/020/`).
4. - [x] `run_suite`: per-scenario output dirs, timeout, `suite.junit.xml`, exit code; run on
   `basic/` end to end. `src/csb_launch/scripts/run_suite` (stdlib only), 10 unit tests in
   `src/csb_launch/test/` (run by `just test`). `basic/` 7/7 PASS back to back, twice
   (540 s, and 563 s after the acb handbrake fix), one testcase each in
   `suite.junit.xml`; `awf/` 36 testcases, 2/2 PASS.
5. - [x] User guide: starter scenarios, `output_directory`, `run_suite`, map source note
   (TUM pack, LGPL-3.0, download link dead, `CARLA_MAPS`); README command list.
6. - [ ] **First-user proof run**: an agent given only the repository URL, the user guide and
   the host's prerequisites (ROS, Autoware 1.5.0, CARLA 0.9.16, the converted maps) follows
   the guide verbatim -- clone, rosdep, colcon build, CARLA, simulation side, vehicle side,
   `run_suite` over `basic/` -- in a fresh directory, without this repo's justfile or
   scripts, and reports every place the guide was wrong, missing or ambiguous. Fixes land in
   the guide; the run is repeated until it is clean.
   Proof runs record screenshots on a local VNC display
   (`vncserver :5 -geometry 1920x1080 -SecurityTypes None -localhost`, captured with
   `import -window root -display :5`) under `docs/proof/020/`, with an index (`README.md`)
   saying what each one shows.

   **Proof run 1 findings and fixes (2026-10-10).** Run 1 (screenshots in the scratch index,
   not yet under `docs/proof/020/`) got `basic/` to 4/7 and `awf/` to 0/36. Fixed since, and
   verified on this host with the guide's own commands (vehicle side
   `acb_launch carla_simulator.launch.xml`, not `just ego-av`):

   - *Velocity limit missing on the guide's vehicle side* -- `/api/autoware/set/velocity_limit`
     (internal API adaptor) was started by `just ego-av` outside any launch; the agent's
     `set_speed_limit` failed and every AWF variant aborted. acb `carla_simulator.launch.xml`
     now passes Autoware's own `launch_deprecated_api` (default true) to
     `autoware.launch.xml`; the justfile's out-of-launch `ros2 launch` adaptors are gone.
   - *`map_based_prediction` abort* (`NoSuchAttributeError: Could not find 1`) in
     town01_pedestrian, after which later scenarios hung in PLANNING: 65 of Town01's 300
     lanelets (kerb strips) had no `subtype`; `PredictorVru` calls `attribute(Subtype)` on
     every lanelet near a pedestrian. `scripts/repair_lanelet_subtypes.py` (run by
     `download_maps.sh`) tags them `subtype=unknown`. Dev runs had masked it with
     `--composable-respawn on-crash`; `basic/` then went 7/7 on the guide's command
     without that flag.
   - *`behavior_path_planner` abort on a new route* (`failed to add guard condition to wait
     set`, autoware_universe#12460): once in 27 UC-ACC variants on the guide path, after
     which 18 variants failed in PLANNING. Not fixable here; the guide's vehicle-side
     command now carries `--composable-respawn on-crash` (Autoware's containers declare
     no respawn), and so does `just ego-av`.
   - *play_launch flags*: every command shows `--enforce-rules off`, `--parser python`
     (vehicle side, scenario) and a distinct `--web-addr`; install via `pip install
     play_launch==0.13.1`. The Rust parser's "dropped `frame_id`" was really a
     wrong-prefix lookup: it resolved `$(find-pkg-share autoware_vehicle_velocity_converter)`
     to `/opt/ros/humble` (the apt package 1.9.0, whose param file has no `frame_id`) while
     running Autoware 1.5.0's binary from `/opt/autoware/1.5.0` -- to be fixed upstream.
   - *Guide*: map town per scenario set, RViz (`rviz:=false`), readiness signals, PLANNING /
     WAITING_FOR_ENGAGE troubleshooting, `pose_instability_detector`'s exit after a
     teleport (harmless, unfixed Autoware behaviour), "crash recovery" wording.
   - *run_suite*: relay pre-flight (no agent: 7 scenarios failed in 15 s, was 480 s each),
     no default per-file cap (UC-ACC needs 2258 s), `TIMEOUT after N s` on the line.
   - *Lidar detector default*: `centerpoint` passed `basic/` 7/7 and UC-ACC 27/27 but
     UC-AEB only 2/9 (ego hit the standing pedestrian); `clustering` 9/9. acb's profile
     now defaults to `clustering`, as dev runs always had.
   - *CARLA wedged by an early client*: a restarted CARLA serves RPC before its first map
     is loaded; the bridge reconnected then and sent settings / `load_world`, wedging it
     three times in a row. The bridge now waits for a map name before talking to it.
   - *Long waits*, measured and bounded (table in the user guide, section 4 "Timeouts"):

     | Wait | Old | New | Why |
     |---|---|---|---|
     | `initialize_duration` | 480 s | 120 s | start to engage 52 s worst, ~20 s median (33 starts) |
     | concealer first `change_to_stop` | max(180 s, budget) | budget | relay offers it only while an agent is registered |
     | concealer service availability | 180 s | 30 s | relay services discovered in 1-2 s |
     | velocity limit / route retries | 30 | 5 | persistent refusal is not transient |
     | clear route / engage / enable / RTC retries | 30 | 10 | ~30-60 s |
     | `SIMULATOR_RESPONSE_TIMEOUT` (non-Initialize) | 420 s | 90 s | bridge's CARLA RPC timeout 30 s + 10 s |
     | `SIMULATOR_INITIALIZE_TIMEOUT` (new) | 420 s | 300 s | CARLA wait 120 s + town load |
     | bridge `reconnect_wait_seconds` | 240 s | 120 s | CARLA restarts served a map in 25-51 s |
     | `global_timeout` | 600 s | 400 s | longest variant 190 s |
     | `run_suite --timeout-per-scenario` | 1800 s | 0 (none) | SSv2 bounds each variant |

     A single scenario with no agent registered now fails in 132 s with "the ego's Autoware is
     not reachable: is its vehicle side up and its agent registered" (was 480 s+).

   Results with the fixes: `basic/` 7/7 in one run (570 s; again 605 s on 2026-10-11 after the last SSv2 change); `awf/` ACC 27/27 (2258 s) with
   `centerpoint`, AEB 9/9 (541 s) with `clustering`; on the guide's command with the `clustering` default and `--composable-respawn on-crash`, ACC 27/27 (2337 s) and AEB 9/9 (623 s). Step 6 stays open
   for a fresh proof run.

## Acceptance

- `ros2 run csb_launch run_suite $(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/basic --output <dir>`
  runs every basic scenario and leaves one result directory per scenario and a
  `suite.junit.xml` that matches them.
- Both AWF use cases pass on CARLA Town01 (or each failure is explained as an Autoware
  behaviour, not a pipeline fault).
- The first-user proof run completes from the guide alone with no undocumented step.
