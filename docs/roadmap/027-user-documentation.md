# Phase 027: User Documentation

A user who has never seen this project reads the README, follows one tutorial from a map
directory to a scenario of their own, runs it, and ends with a demo they can show -- using
ROS commands they already know. Today the README mixes a pitch, developer recipes and a
`play_launch`-only quick start; the user guide is a complete reference but not a path.

**Source**: user direction 2026-10-12 ("start user documentation campaign").
**Depends on**: [020](020-starter-scenarios-and-suites.md) (starter scenarios, `run_suite`,
the first-user proof method), [021](021-physical-verdict.md) / [022](022-entity-geometry-parity.md)
(what a run reports), the 2026-10-12 screencasts (lab NAS, `autoveh/logs/2026 SSv2 on
CARLA/2026-10-12 Starter scenario screencasts/`).

## Decisions (user, 2026-10-12)

- **Showreel**: two GIFs in the README, cut from `town01_pedestrian` and `town01_two_av`,
  sped up and cropped to the CARLA and Autoware (RViz) windows. Committed under
  `docs/media/` with the script that makes them.
- **Commands**: `ros2 launch` / `ros2 run` are primary everywhere a user types. `play_launch
  launch` is recommended as a drop-in replacement (same arguments; per-node logs, web UI),
  shown once per page, not as the main path. `just` recipes are developer-only.
- **CARLA versions**: 0.9.16 is tested. 0.9.14, 0.9.15 and 0.10.0 are selectable at build
  time (`CARLA_VERSION`, or the `carla-0XYZ` feature) and documented as build-verified,
  untested -- and only those that actually build are listed.
- **Developers have their own guide** (user, 2026-10-12): `docs/development.md` becomes a
  contributor/developer guide -- build and test (`just`), repository layout, how the
  pieces fit, and an index of the design docs (architecture, SSv2 protocol, time and
  ticking, launch configuration, fork policy, ...) and the roadmap -- and the README ends
  with a "For developers" section pointing to it.

## Design

**README (user-facing)**, top to bottom: showreel; one-paragraph pitch; how it works (the
existing diagram, three bullets); supported versions; prerequisites; setup (clone, rosdep,
colcon); quick start (four terminals, `ros2 launch`); where to next (tutorials, guide,
troubleshooting); for developers (a short section linking the developer guide). Project
tree, `just` recipes and dev notes move to `docs/development.md`, the contributor/developer
guide: prerequisites for development, `just setup/build/check/test`, the workspace and
overlay rules (from `CLAUDE.md`, rewritten for humans), repository layout, how a request
flows (SSv2 → ZMQ → coordinator → CARLA), testing and live runs, where the design docs and
roadmap are, commit and submodule conventions.

**Tutorials** (`docs/tutorials/`), each runnable top to bottom and ending with the output to
expect:

1. *First run* -- CARLA, a map directory, the simulation side, the vehicle side, a starter
   scenario, its verdict.
2. *Prepare a scenario* -- pick a town, find poses (a helper printing CARLA waypoints as
   OpenSCENARIO `WorldPosition`), write the `.xosc` (ego, NPC, route, success/failure,
   `csbExpectedContacts`), run it.
3. *Run and judge* -- `run_suite`, `--physical-verdict`, the body notes, result files.
4. *Demo run* -- the two-AV scenario and recording your own screencast (chase camera, RViz
   layouts, ffmpeg).

`docs/user-guide.md` stays the reference the tutorials link into; `docs/README.md` indexes
user docs, design docs and the roadmap.

## Steps

1. [x] **Docs.** This phase doc.
2. [x] **Showreel GIFs.** Script `scripts/make_showreel.sh` (ffmpeg crop + stack, 6-8x,
   8 fps, palette); `docs/media/showreel_pedestrian.gif`, `docs/media/showreel_two_av.gif`,
   each about 5-6 MB. Re-record at 1920x1080 if the 1280x720 sources crop soft (they did not).
3. [x] **ROS commands verified.** Every `ros2 launch` / `ros2 run` line the README and
   tutorial 1 give, run as written on this host (ego stack included, which so far has only
   run under play_launch); fix the launch files where `ros2 launch` differs.
4. [x] **CARLA versions build-verified.** Both workspaces built with `CARLA_VERSION` =
   0.9.14, 0.9.15, 0.10.0; record which build; README table from the result.
5. [ ] **README rewrite** and the contributor/developer guide `docs/development.md`.
6. [ ] **Tutorials 1-4**, the pose helper, `docs/README.md`.
7. [x] **Fresh-user check.** One agent with only the README and tutorials, on this host:
   build, first run, a scenario of its own, a suite, a demo recording. Fix what it trips on;
   its clean pass closes the phase.

### Results

**Step 2 (2026-10-12).** `scripts/make_showreel.sh <screencast dir> [out dir]`: crops the
CARLA chase view and the RViz window(s) out of the 1280x720 screencasts, labels them, and
writes palette GIFs at 800 px wide, 8 fps. `docs/media/showreel_pedestrian.gif` (8x, 17 s,
5.8 MB: the ego stops for the crossing pedestrian, `obstacle_stop` in RViz, and resumes) and
`docs/media/showreel_two_av.gif` (6x, 23 s, 5.6 MB: CARLA's view of the ego beside both
Autoware stacks). First cut at 960 px / 10 fps / 128 colours was 14 and 9.9 MB -- lidar
points dither badly; the 1280x720 sources were sharp enough, no re-record needed. **Then cut smaller** (user, 2026-10-12: on GitHub each took ~20 s to load before
playing): 640 px, 6 fps, trimmed to the action -- pedestrian 0:24-1:48 (drive, stop for the
crossing, restart) 10 s / 1.9 MB; two-AV 0:35-2:05, 15 s / 2.5 MB -- and the two-AV GIF sits
in a collapsed `<details>` block, so the README loads ~2 MB of animation up front, not ~12.

**Step 3 (2026-10-12).** On this host, each in its own terminal environment (Autoware's
`setup.bash`, then the workspace's; plus this host's loopback `CYCLONEDDS_URI`, which a
host with working multicast does not need):

- `ROS_DOMAIN_ID=9 ros2 launch csb_launch simulation.launch.xml`: bridge `ZMQ server ready`,
  relay `Relaying entity 'ego'`, both in the launch's console (unlike play_launch, which
  puts them in per-node logs).
- `ROS_DOMAIN_ID=1 ros2 launch acb_launch carla_simulator.launch.xml
  map_path:=$CARLA_MAPS/Town01 vehicle_name:=hero relay:=tcp://localhost:5560 rviz:=false`:
  agent registered and up within 60 s, no process died. The first time this stack ran
  under plain `ros2 launch` here; no launch change was needed.
- `ROS_DOMAIN_ID=9 ros2 launch csb_launch scenario.launch.xml scenario:=$(ros2 pkg prefix
  csb_examples)/share/csb_examples/scenarios/basic/town01_ego_drive.xosc
  output_directory:=...`: PASSED, and the launch ended by itself after 103 s (SSv2's
  `on_exit=ShutdownOnce` is native to `ros2 launch`).
- `ros2 run csb_launch run_suite ... --physical-verdict` with play_launch removed from PATH
  (so it falls back to `ros2 launch`): `town01_rear_contact` failed once -- the blind NPC's
  shove pushed the ego into a pole (CARLA contact, 37 603 N*s; SSv2 `current acceleration
  15.6 out of range [-15, 15]`) -- and passed 2/2 on the rerun. An intermittent outcome of
  the scenario's own collision, not of the launcher; noted for 024.

**Step 4 (2026-10-12).** `cargo build` of each workspace with `CARLA_VERSION` set, each in a
target directory of its own (the installed binaries and running stacks untouched), carla
0.15.0 from crates.io:

| CARLA | csb (`carla_scenario_bridge`) | acb (vehicle side) |
|---|---|---|
| 0.9.14 | builds (342 s) | builds (225 s) |
| 0.9.15 | builds (243 s) | builds (351 s) |
| 0.10.0 | builds (297 s) | **fails**: `no field 'position' on type '&carla::rpc::WheelPhysicsControl'` (x3) |

0.10.0 is therefore listed as "not yet": acb reads wheel `position` to find the rear axle,
which 0.10 replaced; the port is acb's unmerged `carla-0100` branch (roadmap 019). The README
lists 0.9.14 and 0.9.15 as builds-untested, 0.9.16 as tested.

**Step 7 (2026-10-12).** One agent, Opus, given only the README and the tutorials, on a copy
of the working tree without build outputs; CARLA 0.9.16 shared and already running, maps and
the host's loopback DDS profile given as facts about its machine. Everything with
`ros2 launch` as written. 59 min.

| Step | Result |
|---|---|
| README setup: `colcon build` | 51 packages, 20 min 47 s, first try (`rosdep install` failed for lack of sudo; its one missing package, `python3-pygame`, was present via pip) |
| Tutorial 1: both sides, `town01_ego_drive` | PASS; readiness lines as the doc shows; vehicle side up in 39 s |
| Tutorial 2: `carla_poses`, own `follow_to_goal.xosc` | output identical to the doc; own scenario PASS first run (45 s) |
| Tutorial 3: `run_suite basic/ + own --physical-verdict` | 7/8, then 8/8: `town02_episode_change` failed once (`load world 'Town02': time-out of 120000ms` at load average 50), passed alone in 16 s |
| Tutorial 4: second vehicle side, two-AV, RViz layouts, chase view, recording | PASS, once the layouts were found (finding 1) |

Findings and what was done:

1. *Blocker*: tutorial 4's RViz layouts were not installed -- they had been put under
   `csb_launch/data`, a symlink to the untracked maps directory. Moved to
   `src/csb_launch/rviz`, installed to `share/csb_launch/rviz` (fixed before the report).
2. DDS discovery: the doc pointed to User Guide §6, which did not mention it. §6 now has
   "Nothing ever connects" with the loopback `CYCLONEDDS_URI` profile; tutorial 1's
   environment has it as a commented line.
3. Tutorial 1 sourced `~/carla-scenario-bridge/install`; now `<your clone>/install`.
4. `rosdep install` needs sudo: said in the README, with what to do without it.
5. `pip install carla==0.9.16` moved to the README's prerequisites.
6. Tutorial 2 did not say a scenario needs OpenSCENARIO's frame (FileHeader, Storyboard
   nesting, the Act's StartTrigger, StopTrigger): it now says to start from the shipped
   file's copy and write the four parts.
7. `carla_poses ... | head` printed a BrokenPipeError from `ros2 run` itself: the doc no
   longer pipes it.
8. Tutorial 3's results tree now shows `play_log/` (play_launch only).
9. Tutorial 4's ffmpeg recorded 1920x1080 while `make_showreel.sh` crops a 1280x720
   recording: the command now scales to 1280x720.
10. Stopping was undocumented: tutorial 1 has a "Stopping" section (Ctrl-C; again after a
    minute).

Not re-run after the fixes; each fix is to the text that misled, and finding 1's install path
was checked on this tree.

**Follow-ups (code, not docs)**, from the same run:

- On shutdown by Ctrl-C to the whole process group, the vehicle side did not end within
  60 s, the simulation side's bridge exited cleanly and was then *respawned* by
  `respawn="true"`, and the relay raised `RCLError: Failed to publish: publisher's context
  is invalid` (`_publish_all_locked` -> `pub_scan.publish`) and was respawned too.
- After a `load_world` timeout under load the bridge logged `CARLA was restarted (new
  episode ...)` although CARLA's pid was unchanged: the message misreads a slow map load.
- The bridge logs `No traffic light mapping at .../config/traffic_lights_Town01.yaml` (the
  fallback path) on Initializes that do find the map directory's table on another path;
  to a new user it reads as an error.

**027 complete (2026-10-12).** The README shows the showreel and its setup and quick start
work as written with `ros2 launch`; the tutorials took a fresh user from a map directory to a
passing scenario of their own and a recorded demo; the versions table lists 0.9.16 as tested
and only versions that were built. The follow-ups above are bridge work, not documentation.

## Acceptance

- The README shows the two GIFs, and its prerequisites, setup and quick start work as
  written with `ros2 launch`.
- The tutorials take a fresh user from a map directory to a passing scenario of their own
  and a recorded demo.
- The versions table states 0.9.16 tested and lists only versions that were built.
