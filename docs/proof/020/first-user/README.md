# First-user proof run (020 step 6), 2026-10-11

An agent with only the repository URL and `docs/user-guide.md` cloned, built (`rosdep` +
`colcon build`, 51 packages, 9 min 21 s) and ran the guide's commands on play_launch's
defaults (0.15.1 behaviour: Rust parser, no interception). Result: `basic/` 7/7 (532 s),
`awf/` UC-ACC 27/27 (2200 s) and UC-AEB 9/9 (720 s), no workaround beyond the guide.

| File | Shows |
|---|---|
| `rviz_midscenario.jpg` | RViz (vehicle side) in a `basic/` scenario: route set, Autonomous |
| `rviz_ego_driving_town01.jpg` | ego driving at 15 km/h in Town01 mid-scenario, route 90 m |
| `rviz_rear_contact_emergency_stop.jpg` | `town01_rear_contact`: ego at 56 km/h with MRM emergency_stop (expected) |
| `rviz_awf_midrun.jpg` | RViz during `awf/` UC-ACC |
| `xterm_suite_summary.jpg` | `run_suite` verdicts: basic 7/7, awf 27/27 and 9/9, EXIT=0 |
