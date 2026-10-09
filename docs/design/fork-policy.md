# Fork Policy: NEWSLabNTU/scenario_simulator_v2

The `src/scenario_simulator_v2` submodule points at
[`NEWSLabNTU/scenario_simulator_v2`](https://github.com/NEWSLabNTU/scenario_simulator_v2),
not at tier4's. This document says what that fork is allowed to carry, how it is kept
current, and what has to be true before a patch is added to it.

## Current state

| | |
|---|---|
| Branch | `managed-ego-unforked` |
| Upstream base | `3370c817e` (version 25.0.22) |
| Diff against base | 30 files, +1659/-142 (2026-10-09) |
| Series | 20 commits, `4bb48c980` … `efe7f87f8` |

What the series carries, by theme (each commit is one reviewable change):

- **Engage and startup** (012-013, 017): `arrived_goal` as a successful engage outcome; the
  concealer waits for localization to reach the initial pose, leaves a reused Autoware in
  STOP, and waits for `change_to_stop` within `initialize_duration`.
- **`managed_ego`** (013): an inert concealer for an externally driven ego. Kept, default
  `true`, but no longer used here -- the agent relay (017) replaced the domain split it
  existed for.
- **Simulator time** (014-015): `simulation_time_ns` in the protocol, `clock_source`
  parameter (ROS time from the simulator), initial pose and ARRIVED_GOAL judged in
  Autoware's time base.
- **Honest failures** (016): a bounded wait for simulator responses
  (`SIMULATOR_RESPONSE_TIMEOUT`), a failed frame ends the scenario.
- **Scenario files** (017): the `$(env NAME [default])` substitution; the runner never
  rewrites the user's `.xosc`.
- **Simulator-driven entities** (017): controllers `simulator_autopilot` and `agent`, the
  `behavior` field at spawn and the `UpdateEntityGoal` request.

Every carried patch is behavioural and offerable upstream: no build hacks, no vendoring, no
local-environment adjustments, and each is inert unless its option or controller is used.
`concealer/launch.hpp` is byte-identical to upstream.

## What the fork may carry

**Yes:**

- Features we intend to offer upstream, kept as a clean commit series that a maintainer
  could read as a pull request. `managed_ego` is the example: "run scenarios against an
  externally-managed ego" is a legitimate upstream feature that AWSIM-style deployments
  want too.
- Bug fixes, which should be offered upstream immediately rather than accumulated.

**No:**

- Anything that only makes sense in this deployment. Adjustments for our launch layout,
  our domains, our CARLA setup or our host belong in this repo — in `csb_launch`, the
  justfile, or config — where they cost nothing to rebase.
- Anything that changes stock behavior when the new option is not used. Every patch must be
  inert by default: a scenario that uses none of the new options or controllers runs as
  upstream's does.
- Anything that cannot be re-derived by hand. A series small enough to reapply manually is
  the ceiling, because that is the actual fallback when a rebase goes badly.

## Rebase cadence

Rebase onto each upstream release we pin, not continuously. The steps:

1. Rebase the series onto the new upstream tag on a branch named for it.
2. Build it (colcon Release, ROS humble + Autoware 1.5.0).
3. Push the branch to NEWSLabNTU **before** bumping the pin here — a pin to a commit that
   exists only locally breaks `git submodule update` for everyone else.
4. Run the regression set (`town01_ego_drive`, `town01_engage_state`,
   `town01_simulator_autopilot`, `town01_two_av`): the ego must engage and drive through the
   relay, and the simulator-driven entities must reach their goals.
5. Bump the submodule pin here in its own commit.

The last rebase (2026-08-28) was clean despite two series touching
`field_operator_application.cpp`, and cost one build plus the two runs above.

## Upstream intent

Every release the feature stays out of tree is rebase work we chose. Offer it via
[roadmap 005](../roadmap/005-hardening-awf-contribution.md). If upstream takes it, the fork
returns to clean and the submodule can point back at tier4.

## Why fork at all

Phase 012 established that SSv2 cannot be made to leave the ego's autonomy alone from the
outside: the storyboard is gated on concealer engagement, so an unmanaged ego either never
starts NPC logic or dies on a service timeout. The gate is three lines in
`openscenario_interpreter.cpp`. Everything else in this repo is an adapter that upstream
never has to know about; this one thing is not adaptable from outside, which is the bar a
patch has to clear to live on the fork.
