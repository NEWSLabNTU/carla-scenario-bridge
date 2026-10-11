# Roadmap

Files are named `NNN-name.md`. Numbers are identifiers, not a strict execution order —
dependencies are stated in each document.

## Original plan (001-005)

Written before the multi-instance authority model. 001 and 002 are largely delivered; 003 and
004 are superseded by the audit-driven phases below.

- [001: Core ZMQ Adapter](001-core-zmq-adapter.md) — ZMQ server, CARLA connection, entity lifecycle, coordinate conversion
- [002: Autoware Integration](002-autoware-integration.md) — Sensor pipeline, ego physics readback, end-to-end driving
- [003: NPC + Pedestrian Support](003-npc-pedestrian-support.md) — superseded by [008](008-entity-fidelity.md)
- [004: Traffic Lights + Environment](004-traffic-lights-environment.md) — superseded by [009](009-map-and-traffic-lights.md); its ambient-traffic section is rejected
- [005: Hardening + AWF Contribution](005-hardening-awf-contribution.md) — upstream contribution still valid; robustness moved to [011](011-robustness.md)

## Audit-driven plan (006-011)

From the workflow audit of 2026-07-28 and the gap list in
[design/multi-instance-architecture.md](../design/multi-instance-architecture.md). Ordered by
what unblocks what.

- [006: Honest Failures](006-honest-failures.md) — stop returning success for work not done ⭐ **start here**
- [007: Repeatable Runs](007-repeatable-runs.md) — actor teardown, spawn retry, reconnection
- [008: Entity Fidelity](008-entity-fidelity.md) — pedestrians, misc objects, NPC physics
- [009: Map and Traffic Lights](009-map-and-traffic-lights.md) — map loading, signal mapping, recognition
- [010: Multi-Instance](010-multi-instance.md) — config loading, role names, background AVs
- [011: Robustness](011-robustness.md) — tick timeouts, ego respawn, duplicate-publisher hazards
- [012: SSv2-Unmanaged Autoware](012-ssv2-unmanaged-autoware.md) — SSv2 stops *launching* Autoware (`launch_autoware:=false` against our externally-launched ego stack); kills the last SSv2 patch. Spike done 2026-08-08
- [013: Unmanaged Ego on the Fork](013-forked-unmanaged-ego.md) — `managed_ego:=false` patch on the NEWSLabNTU fork; ego gets its own domain, pilot, and clock like every background AV — superseded by 017
- [014: Feature Completeness](014-feature-completeness.md) — pose reference-point offset, collision truth, NPC velocity, and a hardening batch from the 2026-09-25 audit ([design/ssv2-feature-completeness.md](../design/ssv2-feature-completeness.md))
- [015: CARLA as Time and Signal Source](015-carla-as-time-and-signal-source.md) — one simulation clock (CARLA's, reported to SSv2 and published by acb in every domain) and signal state read from CARLA's lights, replacing 014's learned offsets and SSv2-domain-only V2X
- [016: Robustness and Performance](016-robustness-and-performance.md) — every fault ends the scenario in bounded time and the next one runs unattended; csb supervised, epoch persisted; frame budget measured (20 NPCs p95 1.79 ms)
- [017: User Workflow and the Vehicle Agent](017-user-workflow.md) — ROS-native workflow from a scenario dir and a map dir; csb as a ROS node; an agent protocol and relay so SSv2 and Autoware need no shared domain and CARLA/Autoware are swappable ([design/user-workflow.md](../design/user-workflow.md))
- [018: Install and World Config](018-install-and-world-config.md) — `rosdep install` + `colcon build` for users (CARLA version as a carla-rust feature, color_names submodule), `ros2 run acb_launch setup_autoware_data`, weather as bridge services — **complete 2026-10-10**
- [020: Starter Scenarios and Suites](020-starter-scenarios-and-suites.md) — **complete** (first-user proof run clean, 2026-10-11) — installed starter scenarios (`csb_examples`, incl. AWF ODD use cases), `output_directory:=`, `ros2 run csb_launch run_suite`, and a first-user proof run of the guide

## Gap campaign (021-)

Closing the gap between SSv2's verdict and CARLA's physical outcome
([design/gap-campaign.md](../design/gap-campaign.md)): fix the known gaps first, each with a
reproducer; then measure the residual; then fuzz for unknown ones with an adaptive generator
borrowed from greybox fuzzing.

- [021: Physical Verdict](021-physical-verdict.md) — **done 2026-10-11** — the bridge writes CARLA's ego contacts per episode; `physical_verdict` / `run_suite --physical-verdict` add a `carla_contact` testcase next to SSv2's, with scenario-declared expected contacts
- [022: Entity Geometry Parity](022-entity-geometry-parity.md) — blueprint chosen by subtype and declared size from a measured box table (ego and pedestrians excluded); residual reported at spawn and in the JUnit
- [023: NPC Motion Parity](023-npc-motion-parity.md) — NPCs as tracked physics bodies with real velocity and walk cycles, within a 0.2 m pose budget; spike-gated, kinematic stays as fallback
- [024: Gap Trace and Oracle](024-gap-trace-and-oracle.md) — the bridge records SSv2's and CARLA's view of every entity per frame; an offline oracle judges each frame in both geometries and classifies the run against its verdict; baselined on the starter scenarios and the 021-023 reproducers
- 025: Adaptive Generator and Exploit Pipeline — scenario genomes, seed/operator bandits, MAP-Elites archive, annealed energy; confirm, boundary bisection, metamorphic root-cause probes, shrink (phase doc after 024)
- 026: Parallel Isolated Pipelines — N complete CARLA+bridge+ego pipelines on one host (phase doc after 025)

## Real-site twin and realistic rendering (019-)

- [019: NTU Campus Digital Twin](019-ntu-campus-twin.md) — twin of the NTU campus from the vendor MLS scan and our Autoware maps; full re-tiling (the vendor's tiles are truncated at 2^32-1 points), road/building meshes, vegetation proxies, Lanelet2 regulatory elements + xodr, CARLA 0.10 import, lidar and camera realism measured against the real cloud ([design/survey-isaac-sim-and-digital-twin.md](../design/survey-isaac-sim-and-digital-twin.md))

### Why this order

006 comes first because three handlers currently report success while doing nothing, so a
scenario suite can score fiction — no later phase can be validated until that stops.

007 came second because nothing destroyed spawned actors and spawn had no retry, so a leaked
actor blocked the next run's spawn point and the second run of any scenario failed until CARLA
was restarted. Done as of 2026-08-12; iterative work on 008-011 no longer pays that tax.

The rest follow their dependencies. 011 can be pulled forward if spurious CARLA reconnects
start appearing — 009 adds per-frame work and makes that likelier.

012's spike (2026-08-08) found that `launch_autoware:=false` is upstream's supported
"reuse a running Autoware" mode — the concealer still initializes, routes and engages, so
012 needs no pilot and no longer depends on 010. It can run as soon as a live single-ego
stack exists to verify against.

013 layers on 012 and does depend on 010's pilot: with the concealer's engage gate patched
out on the NEWSLabNTU fork, the pilot is the only thing that routes and engages the ego.
Ship 012 first — it stands alone and shrinks 013's fork patch to the minimum.

## Status (2026-10-11 — read this first)

**The bridge is user-ready.** A first user, following only `docs/user-guide.md`, installs,
builds and runs every starter scenario and both AWF use-case suites clean (020's proof run,
2026-10-11; screencasts in `docs/proof/020/screencast/`). Operational traps: `CLAUDE.md`.

Per-phase state:

- **Done:** 001, 002, 006, 007, 010, 011, 012, 015, 016, 017, 018, 020. 003, 004 and 013
  superseded as noted above.
- **008:** acceptance closed; objects come back UNKNOWN until 009's camera classification.
- **009:** light detection works end to end; classification is the open hop.
- **014:** pose offset and collision monitor delivered; NPC velocity (gap 5) moves to 023.
- **005:** open items are outward-facing (SafetyPool access, AWF outreach); deferred by the
  user's decision, as is map provisioning.
- **019:** in progress (re-tile, route alignment, CARLA 0.10 probe done).

**Priority order:**

1. ~~021 Physical verdict~~ -- done 2026-10-11.
2. **022 Entity geometry parity** -- in progress: box table and chooser done (steps 1-3);
   next the SSv2 fork's subtype field and the bridge wiring.
3. **023 NPC motion parity** -- spike first; may close as a measured no.
4. **024 Gap trace and oracle**, then **025**, **026**.
5. 019 and 009's classification, interleaved where a gap phase waits on a long run.
