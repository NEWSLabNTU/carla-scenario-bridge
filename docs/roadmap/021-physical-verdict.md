# Phase 021: Physical Verdict

First fix of the [gap campaign](../design/gap-campaign.md): a run in which the ego
physically hit something in CARLA can no longer pass unnoticed. The bridge writes what
CARLA's collision sensor saw at the end of every episode; the runner turns it into a
testcase of its own next to SSv2's verdict, opt-in.

**Source**: user direction 2026-10-11 ("fix scenario spec and Carla gap first", proposal
approved the same day).
**Depends on**: [014](014-feature-completeness.md) (`collision_monitor.rs`),
[020](020-starter-scenarios-and-suites.md) (`run_suite`, `output_directory:=`).
**Feeds**: [024](024-gap-trace-and-oracle.md) (its contact lines extend this report).

## Problem

- **SSv2 cannot see a CARLA contact, and nothing else judges one.** `CollisionCondition`
  is a 2D overlap of declared boxes at SSv2's poses (`api.cpp:345`). The ego hitting a
  curb, a pole, a wall, or an NPC whose CARLA body is larger than its declared box is
  invisible to it, and a scenario without a `CollisionCondition` checks nothing at all.
- **The CARLA half exists but only as a log line.** `collision_monitor.rs` logs every ego
  contact and an end-of-run table at INFO, and its own doc says "a human compares it against
  the scenario's JUnit result". Nobody does, in a suite of 27.
- **There is no protocol slot to tell SSv2.** The simulator cannot fail an SSv2 run, so the
  verdict has to be joined after the run, where the runner already merges JUnit files.

## Design

### Contact report (bridge)

When the ego's run ends -- `finish_run`, at the ego's despawn (SSv2 despawns every entity
when a scenario ends, so this is the run's end in practice) or teardown, and `abandon` when
CARLA restarted -- the bridge writes one JSON file:

```
<contact_report_dir>/<UTC ego spawn time, %Y%m%dT%H%M%S%.3fZ>_e<episode>.json
{
  "schema": 1, "episode": 3, "ego": "ego",
  "start_unix": 1791683599.572, "end_unix": 1791683637.104,   // ego spawn, run end
  "complete": true,                  // false: sensor abandoned (CARLA restarted)
  "contacts": [
    {"other_id": 233, "name": "npc_rear", "type": "vehicle.tesla.model3",
     "static": false, "first_ssv2_frame": 412, "first_sim_time": 21.05,
     "count": 37, "max_impulse": 1843.2}
  ]
}
```

- `other_id` null and `static: true` for the static world. Same data as the summary table,
  from the same `CollisionLog`; the log line stays.
- Written on a short-lived thread, never on the ZMQ thread (`/home` stalls,
  CLAUDE.md "The log disk can stall a node"); the file is written to `*.tmp` and renamed,
  so a reader never sees half a report.
- `contact_report_dir`: config key and ROS parameter (launch argument on
  `simulation.launch.xml`). Default `$ROS_HOME/csb/contacts` (`~/.ros/csb/contacts`), on:
  the file is a few hundred bytes per run and a report that was never written cannot be
  joined later. `off` disables it (an empty launch argument means "not given", as for every
  other bridge argument). The bridge keeps the newest 500 reports and deletes older ones at
  write time. Requires `collision_monitor` on (the default); with it off, no report.
- A run without an ego (`town02_episode_change`) has no report; the join settles that case
  from the scenario file, below.

### Verdict join (runner)

`ros2 run csb_launch physical_verdict <result.junit.xml> [--contacts DIR] [--since T]
[--until T] [--scenario FILE] [--min-impulse N]`: finds the reports whose `start_unix` lies
in `[since, until]` (default: the newest report), and adds one testcase, `carla_contact`, to
the JUnit file's first testsuite in place. Several reports in the window (a YAML scenario's
variants) are judged together; the message numbers each "run i/N":

| Reports found | Contacts after filtering | Testcase |
|---|---|---|
| -- (the scenario declares no ego entity) | -- | pass, "no ego in scenario" |
| ≥ 1, all complete | none | pass |
| ≥ 1 | ≥ 1 | **failure**, message lists each: entity (or `static world`), first SSv2 frame and sim time, max impulse |
| ≥ 1, one incomplete | none | error: "CARLA restarted mid-run; contact record incomplete" |
| none | -- | error: "no contact report from the bridge in the run window (contact_report_dir off, or a different bridge?)" |

Filtering: contacts below `--min-impulse` (default 0: every contact counts until the
baseline says otherwise), and contacts with entities the scenario **declares as expected**.
A scenario that is meant to collide says so in its own parameters:

```xml
<ParameterDeclaration name="csbExpectedContacts" parameterType="string" value="npc_rear" />
```

(comma-separated entity names; `static` allows static-world contact). Read from the `.xosc`
or the `.yaml` given by `--scenario`, which also says whether the scenario has an ego entity
(`isEgo=true`, or SSv2's Autoware controller, as `run_suite` reads it); SSv2 ignores the
parameter. `town01_rear_contact`
gets it.

`run_suite --physical-verdict` runs the join for every scenario variant, with the variant's
own start and end wall time as the window, before merging into `suite.junit.xml`. Default
off, so a suite's verdict means what it meant before unless asked for. The single-scenario
path (user guide Terminal 4) is the standalone command after the run.

The join never edits SSv2's own testcases. A run SSv2 failed stays failed; a contact adds a
failure next to it, which is what makes the disagreement visible in the suite table.

## Steps

1. [x] **Docs.** This phase doc; gap-campaign design and roadmap README updated.
2. [x] **Report writer, test-driven.** `CollisionLog` → report JSON (pure, unit tested:
   empty run, static contact, named and unnamed actors, abandoned run); atomic write and
   retention on a temp dir.
3. [x] **Bridge wiring.** `contact_report_dir` config key + parameter + launch argument;
   episode counter at `Initialize`, spawn time at the ego's spawn; a report on every run
   end path (despawn, teardown, abandon). Unit test for the config default and `off`.
4. [x] **`physical_verdict`, test-driven.** Window selection, expected-contacts parsing
   (`.xosc` and `.yaml`), impulse filter, the four outcomes; synthetic reports and JUnit
   files, no CARLA.
5. [x] **`run_suite --physical-verdict`.** Per-variant window; suite merge carries the extra
   testcase; `--help` and user guide section.
6. [x] **Live.** `town01_rear_contact` passes with the join (its contact is expected);
   the same scenario with `csbExpectedContacts` removed fails `carla_contact` while SSv2's
   testcase passes -- the disagreement, now in the JUnit. All nine starter scenarios with
   `--physical-verdict`: record every contact found; each is either a real gap (kept as a
   finding for 022/023) or a reason to set `--min-impulse` (stated with the number).

### Live results (2026-10-11)

CARLA 0.9.16 Town01, the dev ego stack, bridge rebuilt with this phase; `run_suite
--physical-verdict` over `basic/` (7), one UC-ACC and one UC-AEB variant, and a copy of
`town01_rear_contact` with `csbExpectedContacts` removed:

| Scenario | SSv2 | `carla_contact` | CARLA contacts |
|---|---|---|---|
| town01_ego_drive | pass | pass | none |
| town01_engage_state | pass | pass | none |
| town01_pedestrian | pass | pass | none |
| town01_rear_contact | pass | pass | `npc_rear`, 3 events, max 34 269 N·s -- declared |
| town01_simulator_autopilot | pass | pass | none |
| town01_traffic_light | pass | pass | none |
| town02_episode_change | pass | pass | no ego in scenario |
| UC-ACC-001-0001, one variant | pass | pass | none |
| UC-AEB-001-0001, one variant | pass | pass | none |
| **town01_rear_contact, undeclared** | **pass** | **failure** | `npc_rear` at SSv2 frame 223, 33 411 N·s |

The last row is the disagreement this phase exists for, now in one `suite.junit.xml`. No
starter scenario touched anything else, static world included, so `--min-impulse` stays 0:
there is nothing yet to filter. (`town01_ego_drive` first failed at `Initialize`:
`load_world('Town01')` from Town02 timed out at 120 s while other users held the disk at
IO pressure ~36; it passed on the rerun. Not this phase's code: the scenario never started.)

## Acceptance

- Every ego run the bridge serves leaves exactly one report, also after a CARLA restart
  (marked incomplete); a run with no ego passes the join from its scenario file.
- `town01_rear_contact` without its expectation: SSv2 PASS, `carla_contact` FAIL, in one
  `suite.junit.xml`.
- The nine starter scenarios' results with `--physical-verdict` are recorded here, every
  contact explained.
- With `--physical-verdict` unset, `suite.junit.xml` has the same testcases and verdicts
  as 020 produced for the same results.
