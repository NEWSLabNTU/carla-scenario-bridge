# Session checkpoint — 2026-08-10

State of the SSv2+CARLA integration after the exitSuccess campaign. For the full
per-phase anatomy see `docs/roadmap/012-ssv2-unmanaged-autoware.md`; this file is
the cross-machine handoff summary.

## Where things stand

- **E2E green and reproducible**: `town01_ego_drive.xosc` has passed three times
  on fresh stacks (SSv2 `Passed`, JUnit clean, ~39 s spawn-to-goal). Working
  recipe: CARLA (systemd unit, offscreen) → `carla_scenario_bridge`
  (`CSB_CONFIG_DIR=src/carla_scenario_bridge/config`) → `just ego-av` → wait for
  play_launch "Startup complete" + `/api/operation_mode/change_to_stop` in the
  service list → `just scenario ...`. Verdict: interpreter err log +
  /tmp/scenario_test_runner/result.junit.xml.
- **011 (acb re-attach) done**: acb detects ego despawn, releases sensors,
  re-attaches to the next spawn — across map reloads and server replacements.
  Fresh ego stack per scenario is no longer required.
- **007 half-verified**: run A reproduces; run B on a reused stack has never
  received a verdict — every attempt was cut down by CARLA OOM-kills from the
  shared GPU's training jobs (ten in one day). All pieces are committed; it
  needs one quiet ~15 min window.
- **010 mostly verified**: two-domain run works — D0 ego + D1 background stack
  coexist (after the DomainGain fix below), the bridge spawns `bg_av_1`, its
  acb attaches and GNSS-initializes localization, and the ego scenario passed
  with the background AV in-world. Unverified: the pilot's engage leg (the bg
  AV physically driving) — same GPU story.

## Pinned fixes worth knowing about

- Two `/clock` publishers (double acb_bridge include) was the great
  availability killer — `launch_vehicle_interface:=false` in ego_av/
  background_av launch files. Never run two clock publishers in one domain.
- CycloneDDS `DomainGain 1000` (config/cyclonedds-localhost.xml): default 250
  overlaps domain 0's port range with domain 1's base once
  MaxAutoParticipantIndex is 300 → "Failed to find a free participant index".
- Bridge smooths reported acceleration over 3 frames; xosc Performance bounds
  15/15. PhysX one-frame contact jolts are otherwise scenario-fatal.
- play_launch: lost-load rescue landed upstream; `just ego-av` passes
  `--load-node-timeout 120 --load-total-budget 180`.
- acb rebuilds MUST set `CARLA_VERSION=0.9.16` or the 0.10.0 prebuilt links in
  and crashes with `std::bad_array_new_length` at connect.

## Operational traps (hard-won, avoid relearning)

- Failed scenario runs leave interpreter zombies that hijack the next run's
  lifecycle services and desync the bridge's ZMQ REP socket. Kill scenario
  processes by comm name, then restart the bridge process (it is stateless).
- Stale per-domain `ros2 daemon`s lie; use `--no-daemon` when checking D1.
- The world only ticks during a scenario (SSv2 owns the tick) — a background
  AV cannot drive between runs.
- `pkill -f` with any pattern that appears in your own wrapper's command line
  kills the wrapper. Match on comm, not args.

## Next actions

1. Quiet GPU (`nvidia-smi` < ~2 GB): 007-B verdict, then the bg-AV-drives leg
   of 010. Both are pure reruns.
2. CARLA sensor-teardown crash fix in the jerry73204/carla fork — ends the
   OOM/teardown crash class that has eaten two verification sessions.
3. Upstream batch: play_launch rescue, SSv2 `carla-compat` arrived_goal patch.

## Repo pins at checkpoint

- csb main `3b20205` (+ this checkpoint)
- acb main `aeb2031`
- SSv2 fork branch `carla-compat` `f77cd12c3`
- play_launch main `9d06d08`
- carla-rust master `2718365`, CARLA fork branch
  `worker-thread-exception-containment`, release `carla-rust/0.9.16-3`
