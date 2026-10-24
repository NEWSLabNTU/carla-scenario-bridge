# 006: The 20-NPC bench never reached a verdict

**Severity**: low (the frame budget was still measured). **Status**: fixed (2026-10-24).
**Found**: roadmap 050, carried through every 051 suite as "missing".

## What was wrong

`scenarios/bench/town01_npc_20.xosc` succeeds after 60 s of simulation time, but no run
wrote a `result.junit.xml`. SSv2's runner stopped each run at its 400 s wall-clock cap
(`The simulation has timed out. Forcibly inactivate.`) about 55 s into the scenario. A run
the runner times out has no verdict.

Two things made it that slow:

- **SSv2 itself.** The interpreter spends ~370 ms per frame on 20 NPCs: in the run below,
  `update` was 367 ms mean and 547 ms max, `evaluate` 0.05 ms. That is one core at ~92 %,
  while the bridge processes a frame in ~1 ms and ticks CARLA in ~8 ms. So 1200 frames take
  ~7.5 min at best.
- **An ego stack nobody uses.** The bench has no ego, but `just scenario` started a
  supervised ego stack for it anyway (`scripts/carla_up.sh`), about 90 Autoware processes
  that compete with the interpreter for the CPU.

## Fix

- `just scenario` and `just e2e` ask `_scenario-has-ego`. It uses SSv2's own test: a
  Controller named `Autoware`, or one with the property `isEgo` true. For a scenario without
  an ego they pass `SIM_UP_EGO=0`, and `carla_up.sh` / `isaac_up.sh` then start no ego stack.
  One that is already up is left alone. The benches (`town01_npc_10/20/50`),
  `town01_npc_shoulder` and `town02_episode_change` have no ego.
- `scripts/run_050_set.sh` runs the bench with `global_timeout:=1200`.

Verified 2026-10-24: with no ego stack and `global_timeout:=1200`, the bench passed
(`failures="0" errors="0"`) in ~7.5 min wall time. Frame budget p50 processing 0.9-1.1 ms,
tick 7.8-8.1 ms.

The same suite now reports `town01_truck_pass` as `expected-fail` when its failure is the
designed SSv2 collision (`ssv2_collision_held`), since the gap oracle's positive control
fails by design (its xosc). Any other failure, such as a timeout, stays `fail`.
