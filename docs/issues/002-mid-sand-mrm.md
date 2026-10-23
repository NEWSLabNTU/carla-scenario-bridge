# 002: A 0.4 s emergency stop in the middle of the sand on town01_soil

**Severity**: low (the run passes in SSv2; acceptance's MRM count fails it). **Status**: open:
the trigger is known (one NDT scan at its iteration limit), its cause in the original run is
not.
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
score (NVTL ~3.0, `scripts/acceptance_diag.py ndt`); the 60s are single-scan outliers. In the
seven reruns on the cooked tiles there were none (none above 9). Those two at the dirt exit had
a cause: the plates left a wheel with no ground for one substep at a zone's edge (fixed, issue
001), and the next batch (`..._22-35-55`) had 4/4 PASS and 0 MRM. That batch still had one
60-iteration scan, at x 172 (the sand zone's exit), which this time did not reach the
availability.

## Next

- What still makes a lone scan need 60 iterations near a zone's edge, on either build.
  Candidates: the car pitching where it climbs out of a rut (the ruts feather out over 1 m),
  the cut road's seam in the lidar's view.
- Whether Autoware should stop on one such scan is Autoware's configuration
  (`/autoware/localization` in the graph); changing it would hide the symptom, not the
  cause, so it stays.

The recording stays on in every acceptance bag.
