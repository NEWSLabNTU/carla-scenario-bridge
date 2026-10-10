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

## Acceptance

- `ros2 run csb_launch run_suite $(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/basic --output <dir>`
  runs every basic scenario and leaves one result directory per scenario and a
  `suite.junit.xml` that matches them.
- Both AWF use cases pass on CARLA Town01 (or each failure is explained as an Autoware
  behaviour, not a pipeline fault).
- The first-user proof run completes from the guide alone with no undocumented step.
