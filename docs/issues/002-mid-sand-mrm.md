# 002: A 0.4 s emergency stop in the middle of the sand on town01_soil

**Severity**: low (the run passes in SSv2; acceptance's MRM count fails it). **Status**: fixed
(2026-10-24): NDT's solver cycled between two points until its iteration limit; the fix rides
in the Autoware overlay (`NEWSLabNTU/autoware_core`, branch `csb/ndt-step-reversal-1.5.0`).
**Found**: 2026-10-23, roadmap 047 S6.

## What is wrong

`scenarios/surfaces/town01_soil.xosc` on Isaac with Autoware, the lead car on the ego's line,
the fitted soil and feathered zone edges: one run of two had a 0.4 s MRM (emergency stop on,
then off) at x 176.5, in the middle of the sand zone, at ~4 m/s. There the ego had been pulled
0.2 m sideways into the lead's fresh ruts (y -129.58 against its line's -129.37). Runs on
packed ruts (rut set `worn`, 2/2) or with the lead beside the line (2/2) had none.

## Why it matters

047's acceptance asks for no MRM, and an off-road stage is where ruts like these are normal.
If Autoware stops whenever its car is shoved into a rut, every soil scenario is flaky; if the
cause is something else (the host load, a stale topic), it is not the soil's to fix.

## What is known

Run `play_log/acceptance/2026-10-23_20-04-15`, ego stack `play_log/ego/2026-10-23_19-03-41`.

- `mrm_handler`: `NORMAL -> MRM_OPERATING` at 12:07:24.949 UTC, `-> NORMAL` at 25.344, with
  no `operation_mode_availability is timeout` before it. Every other MRM in that stack's
  life (between runs, at world resets) came with the timeout line. So the availability
  arrived on time and said autonomous was not available: some diagnostic in Autoware's graph
  went to ERROR (or stale) for ~0.4 s.
- play_launch's `diagnostics.csv` over 12:07:22-25 holds only standing errors (door state
  unknown, traffic-light recognition) and one new warning: `ndt_scan_matcher:
  scan_matching_status`, "The number of iterations has reached its upper limit" (60), at
  12:07:24.760, 0.19 s before the MRM. NDT working that hard means the scan no longer fits
  the map well: the car rolling and pitching in a rut, or the soil's ground lower than the
  PCD's road (the ruts are 9-10 cm; the PCD is CARLA's flat road).
- The acceptance bag recorded only the scored topics, not the availability or the graph,
  and a `/diagnostics` listener started by hand on the ego domain received nothing (probably
  the wrong DDS transport: see CLAUDE.md, "Talking to the stack from outside a launch").

## Reruns (2026-10-23 evening)

`scripts/acceptance_bag.py` now also records `/system/operation_mode/availability`,
`/diagnostics_graph/struct`, `/diagnostics_graph/status` and `/diagnostics` (never scored).
The same scenario, code and lead-on-line setup, two batches
(`play_log/acceptance/2026-10-23_21-38-14`, `..._21-52-08`), host load 60-75 on 24 cores
(the MRM run had 25-175):

| batch | runs | MRM | longitudinal (m/s^2) |
|---|---|---|---|
| 1 | 3 of 4 drove (run 1: the first ego spawn on a fresh world server took 92.9 s, past SSv2's 90 s) | 0 | 0.29, 0.27, 0.19 |
| 2 | 4/4 PASS | 0 | 0.19, 0.19, 0.20, 0.19 |

Seven clean runs: it does not reproduce on demand. In both bags, while autonomous was
available, only two leaves of the graph were ever at ERROR, and both stayed there throughout
without touching availability (`control_command_gate: timeout_output`,
`service_log_checker: response_status`). availability went false only between runs (the
world reset), never during one. That is the baseline the next occurrence will be read
against.

## The trigger, read from the graph (issue 001's plates, 2026-10-23)

The first batch on issue 001's plates (`play_log/acceptance/2026-10-23_22-19-35`) had two
MRMs in three runs, both at x 144-148, where the ego leaves the dirt zone. The graph
recorded each (`scripts/acceptance_diag.py drops`: the levels that rose against the status
1.5 s before):

```
leaf ndt_scan_matcher: scan_matching_status: 0->1 The number of iterations has reached its
     upper limit. The number of iterations: 60, Limit: 60.; There is a possibility of
     oscillation in a local minimum
node /autoware/localization/scan_matching_status: 0->1
node /autoware/localization: 0->1
node /autoware/modes/autonomous: 0->1        -> availability.autonomous false -> EMERGENCY_STOP
```

So in Autoware 1.5's graph a single WARN from NDT makes autonomous unavailable, and one scan
that needs 60 iterations is an emergency stop. That is the warning `diagnostics.csv` showed
0.19 s before the original MRM, so it was very likely this trigger too.

NDT otherwise converges in 2-5 iterations everywhere on the route, soil or road, with the same
score (NVTL ~3.0, `scripts/acceptance_diag.py ndt`); the 60s are single-scan outliers. The
plates' first batch had a cause of its own for two of them: a wheel with no ground for one
substep at a zone's edge (fixed, issue 001). The next batch (`..._22-35-55`) had 4/4 PASS and 0
MRM, and still one 60-iteration scan at x 172 that this time did not reach the availability.

## The cause: NDT's two-point cycle (2026-10-24)

An earlier reading here said the seven reruns on the cooked tiles had no scan above 9
iterations. That was wrong. `scripts/acceptance_diag.py ndt` counts every scan, and every
batch, on either build, has at least one at 60. Each of them has the same signature:

| batch | build | 60-iteration scans (ego x) | oscillation num | NVTL | initial to result |
|---|---|---|---|---|---|
| `21-38-14` | cooked tiles | 147.6 | 58 | 3.01 | 7.8 cm |
| `21-52-08` | cooked tiles | 147.4 | 58 | 2.97 | 8.9 cm |
| `22-19-35` | plates, no margin | 144.0, 147.7, 177.9 | 56-58 | 2.82-2.96 | 3.5-5.7 cm |
| `22-35-55` | plates | 172.9 | 57 | 2.85 | 1.1 cm |

`local_optimal_solution_oscillation_num` is NDT's count of consecutive steps that reverse
direction. At 56-58 of 60, the solver spent the whole scan jumping between two poses. The
result is good: NVTL 2.8-3.0 against a ~3.0 norm, 1-9 cm from the initial guess. NDT
publishes it, because Autoware counts an oscillation as converged
(`is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score`).
But the same scan raises two WARNs, the iteration limit and the oscillation, and in Autoware
1.5's graph that WARN is the emergency stop above.

The solver is autoware_core's `multigrid_ndt_omp` (1.5.0, and unchanged up to 1.9.0). Line
search is off (`use_line_search = false`, "not tested well"). So every Newton step is
`min(|newton|, step_size)` along its direction, and the bound never shrinks. Near an optimum
where the Newton step is longer than `step_size` on both sides, as with a flat or non-smooth
score from a noise-free lidar on a 2 m voxel map, the solution jumps `step_size` one way, then
`step_size` back, until `max_iterations`. acb met the same cycle on CARLA in roadmap 014. Its
NDT parameters (`step_size` 0.05 with 60 iterations, where upstream has 0.1 with 30) made the
cycle rarer, not impossible.

Here it comes where the ego pitches. On the cooked tiles both were at x 147.5, near the end of
the dirt zone (it ends at 145.6). On the plates there were also some in the sand. In the 4 m bins where they fall, the EKF's pitch spans 0.3-1.1 degrees, against 0.02
on the road. That is the ruts, as they should be: the soil is not the bug.

## Fix

In the solver: halve the step bound each time the step direction reverses (cosine below
-0.9, the threshold `count_oscillation` uses). A two-point cycle then settles in about four
more iterations: 0.05, 0.025, 0.0125, then 0.006, which is below `trans_epsilon` (0.01). A
search that never reverses takes exactly the steps it took before. The patch
(`multigrid_ndt_omp_impl.hpp`, 17 lines) is commit `fe45f96` on `csb/ndt-step-reversal-1.5.0`
of `NEWSLabNTU/autoware_core`, a fork made for it: since Autoware 1.5 the package lives in
autoware_core, not autoware_universe. `scripts/autoware_overlay.bash` now takes one source per
fork, and `just build-autoware-overlay` builds it beside the blind_spot fix. `just ego-isaac`,
`ego-av` and `bg-av` source the overlay, so the CARLA ego gets the fix too.

Not changed: the diagnostic graph. A scan that really fails still makes autonomous
unavailable, as it should. acb's NDT parameters (`step_size` 0.05, 60 iterations, tuned
around the cycle in roadmap 014) are not changed either. With the fix, upstream's 0.1 and 30
may do as well; that is for acb to measure on CARLA.

## Validation (2026-10-24)

`town01_soil.xosc` x8 on Isaac with the overlay's NDT (`play_log/acceptance/2026-10-24_10-26-28`;
the ego stack's `ndt_scan_matcher` loaded `third_party/autoware_overlay/.../libmultigrid_ndt_omp.so`):
8/8 PASS, 0 MRM, longitudinal 0.18-0.29 m/s^2, localization gap 0.97-1.10 m (it was
1.03-1.10).

| bags | scans | at 60 | oscillation num above 10 | most iterations while driving |
|---|---|---|---|---|
| four batches before the fix (14 runs that drove) | 3513 | 6 | 6 (56-58) | 60 |
| `2026-10-24_10-26-28`, 8 runs | 2213 | 0 | 0 (most: 4) | 10 |

Every availability drop in the new bag (7) is at a run's end, standing at x 124 while the
world resets. An earlier batch on the same build (`2026-10-24_09-25-29`, 3 runs before
issue 003's first fix hung the world server) had none above 25 either.

The recording stays on in every acceptance bag.
