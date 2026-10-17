# Phase 042: Autoware Fixes Carried as an Overlay

Run the ego on Autoware 1.5.0 with upstream fixes it does not have yet, one package at a
time, reproducibly, without rebuilding Autoware. First (and so far only) package:
`autoware_behavior_velocity_blind_spot_module`, for the right-turn bug 033 found
([autoware_universe#13509](https://github.com/autowarefoundation/autoware_universe/issues/13509)).

**Source**: 033 Results B (2026-10-17): with a classifying perception the `blind_spot` module
still never stops for a cyclist on the inside of a right turn; a scratch build of the module
with the ego passage lines in the left-turn order yielded 3/3.
**Depends on**: 033 (reproducer `scenarios/demo/town01_i6_cyclist_inside.xosc`).
**Feeds**: 033 incident 6 (still needs a classifying perception as well).

## The bug and the fix

`blind_spot` times the ego across the conflict area between two lines, both generated on the
turning side: the ego's turning-side edge carried straight on
(`virtual_ego_straight_path_after_turning`) and the outermost blind-side lane's turning-side
bound carried through the junction (`virtual_blind_lane_boundary_after_turning`). The ego
crosses the first one first for either turn direction, but `scene.cpp` and
`experimental/scene.cpp` swap them for a right turn, so the interval collapses or reverses
(`ego_passage_interval` `[5.57, 5.52]`) and never overlaps the cyclist's. The object side is
mirrored correctly. Fix: first line = ego straight path, second line = blind boundary, for
both directions.

## Plan

1. **Upstream-style fix on the fork** `NEWSLabNTU/autoware_universe`, branch
   `fix/blind-spot-right-turn-ego-passage` from upstream `main`: the line choice moves into
   `compute_ego_passage_time_interval()` (legacy and experimental), a gtest checks a right turn
   against its mirrored left turn (fails on the stock order, passes with the fix), DCO
   signed. PR upstream is the user's call; nothing outward-facing from here.
2. **The same two commits on 0.48.0**, branch `csb/blind-spot-right-turn-0.48.0`: the module
   version Autoware 1.5.0 ships, so the overlay is that release plus the fix and nothing else.
3. **Overlay workspace** `third_party/autoware_overlay/` (git-ignored, built on demand):
   `just build-autoware-overlay` fetches the fork sparse at a commit pinned in
   `scripts/autoware_overlay.bash` (only the listed package directories), and builds them
   against `/opt/autoware/1.5.0`. `just ego-av`, `just bg-av` and the scenario precheck
   source it right after Autoware, before `EGO_OVERLAY`, acb and csb. **On by default** when
   built (`AUTOWARE_OVERLAY=0` turns it off): it replaces one pluginlib plugin with the same
   release's sources plus a two-line behaviour change that only affects right turns, and the
   regression runs below must show no change elsewhere. Not built, or built from another
   commit than the pin, prints a warning and runs what it has.
4. **Verify**: centerpoint (`EGO_LAUNCH_ARGS=lidar_detection_model:=centerpoint`, engine built
   first), `town01_i6_cyclist_inside.xosc` at least 3 times: the ego yields, no contact,
   `ego_passage_interval` ordered. Default clustering stack with the overlay: incidents 1-5 by
   `incident_judge` and the six starters, no regression.
5. **Drafts, not posted**: the upstream PR and a comment on #13509 pointing at the fork branch.

Retire the overlay package by package as Autoware releases carry the fixes: drop the path
from the package list, bump the pin, rebuild.

## Results

*2026-10-17: steps 1-4 done; step 5 drafted (in the session report, not posted).*

- **Fork branches** (pushed): `fix/blind-spot-right-turn-ego-passage` on upstream `main`
  f81a3bf089 (`6244a3f79` test, `1c761ab4a` fix) and `csb/blind-spot-right-turn-0.48.0` on tag
  0.48.0 (`68af89e4c` test, `e35db1ab3` fix, the overlay pin). Both commits DCO-signed.
- **Unit test** `test/test_ego_passage_interval.cpp` (legacy and experimental): on the test
  commit, i.e. the stock line order, both cases fail -- left turn `[3.08, 5.60]` s, mirrored
  right turn `[4.56, 4.40]` s; with the fix both pass, on 0.48.0 (full module build against
  Autoware 1.5.0) and on `main`. `main`'s module does not build against 1.5.0 (newer
  `vehicle_info_utils`, `lanelet2_utils`, `trajectory` APIs), so there the test ran against
  `main`'s two `time_to_collision.cpp` alone, the only translation units the fix touches that
  compile on 1.5.0; the two `scene.cpp` call sites produced no error of their own (only the
  unrelated 1.5.0 API ones). clang-format (17 for 0.48.0, 22.1.5 for `main`) and cpplint clean.
- **Overlay** loaded as intended: `behavior_velocity_planner`'s maps show
  `third_party/autoware_overlay/install/.../libautoware_behavior_velocity_blind_spot_module.so`.
- **Incident 6 on centerpoint, overlay on: 3/3 PASS** (`incident_judge`), cyclist BICYCLE,
  min gap 0.15 m each (the cyclist passes the standing ego inside the lane, as in 033), ego
  stood 1.8 / 0.9 / 0.8 s, G1-G3 0. `ego_passage_interval` ordered in every frame (442 / 479 /
  437 markers, none reversed); decisions `Safe -> Unsafe -> Safe -> Unsafe -> OverPassJudge`.
  Stock on the same stack (033): 2/2 contact.
- **Default clustering stack, overlay on: incidents 1-5, three rounds, 13/15 PASS**; the two
  failures are not the overlay's -- `blind_spot` was never instantiated in either run (no
  decision or marker): `i2 ov_1` the ego stood 4.4 s, then crept into the pedestrian (min
  -0.38 m; 033's residual risk), `i3 ov_2` `behavior_path_planner` aborted 13 s in
  (autoware_universe#12460, reloaded by play_launch) and the ego drove on, through the red
  light, into the guardrail at the T junction (ego acceleration -18 m/s^2, SSv2 error). Only
  incident 5 runs `blind_spot` (a left turn): ordered intervals as on stock, `Safe ->
  OverPassJudge`, 3/3 PASS. **Starters 6/6 PASS** (ego_drive 51 s, engage_state 46,
  pedestrian 130, rear_contact 28, simulator_autopilot 57, traffic_light 192).
- Not fixed, recorded: in `i3 ov_2` the ego kept accelerating (to ~15 m/s, x 347 -> 378 m in
  2 s) while `behavior_path_planner` reloaded, i.e. control followed a stale or default
  trajectory without a stop; worth its own look (acb/Autoware), independent of this phase.

Scratch: `/tmp/claude-1000005/-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-
80a742b40cbd/scratchpad/s_bsfix` (`au/` the fork checkout, `ws_048/` and `main_check/` the
test builds, `test_*_{before,after}.log`, `runs/i<N>/{cp,ov}_*`, `starters/`, `drafts.md`).
