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
- [022: Entity Geometry Parity](022-entity-geometry-parity.md) — **done 2026-10-12** — blueprint chosen by subtype and declared size from a measured box table (ego and pedestrians excluded); residual reported at spawn and in the JUnit
- [023: NPC Motion Parity](023-npc-motion-parity.md) — NPCs as tracked physics bodies with real velocity and walk cycles, within a 0.2 m pose budget; spike-gated, kinematic stays as fallback — **done 2026-10-13**, `tracked` is the default
- [024: Gap Trace and Oracle](024-gap-trace-and-oracle.md) — the bridge records SSv2's and CARLA's view of every entity per frame; an offline oracle judges each frame in both geometries and classifies the run against its verdict; baselined on the starter scenarios and the 021-023 reproducers
- 025: Adaptive Generator and Exploit Pipeline — scenario genomes, seed/operator bandits, MAP-Elites archive, annealed energy; confirm, boundary bisection, metamorphic root-cause probes, shrink (phase doc after 024)
- 026: Parallel Isolated Pipelines — N complete CARLA+bridge+ego pipelines on one host (phase doc after 025)

## User documentation (027-)

- [027: User Documentation](027-user-documentation.md) — **done 2026-10-12** — user-facing README with a showreel, `ros2 launch` as the primary command (play_launch as drop-in), CARLA versions build-verified, tutorials from scenario preparation to demo run, checked by a fresh user
- [028: Run Tooling](028-run-tooling.md) — scenario-writing skill, infrastructure retry in `run_suite`, scenario lint, map check, recorded runs rendered to video on failure, location sweeps from one template; smallest first
- [029: Prebuilt Releases](029-prebuilt-releases.md) — release tarballs of the built workspace, split by side: the simulation side per ROS distro (Autoware-free, its 13 Autoware message/library packages vendored), the vehicle side per Autoware release, the agent SDK as one wheel; every CARLA version's bridge binary in the same tarball behind a launcher; support tiers; CI build and smoke test
- [030: Agent API](030-agent-api.md) — one Python `Agent` interface for any autopilot (own class, YAML-configured ROS adapter, or a subclass of ours), protocol v2 `reset` / `episode_end`, `csb-agent check` / `new` / `supervise`, reset between scenarios with restart as an option
- [031: Maps for Scenarios](031-maps-for-scenarios.md) — **deferred 2026-10-13**: build Autoware maps from any CARLA town, find a place on the maps for a designed scenario, generate a road no town has; open-source status, expected map defects and estimates noted
- [032: NPC Tracking Fixes](032-npc-tracking-fixes.md) — the five tracking gaps 024's baseline found: no snap into the static world (kinematic fall-back, upright on the ground), curvature feed-forward in corners, standing walkers still, spawn at speed, NPC contact sensors; before 025
- [033: Demo Tour](033-demo-tour.md) — one continuous 5-8 min Town01 drive through six incidents (hard brake ahead, pedestrian dart-out, red light, blocked lane, crossing traffic at a turn, cyclist on the inside), captioned, never stalling; ramp merge and a multi-lane tour later
- [034: Bridge Failure Paths](034-bridge-failure-paths.md) — a hung tick (a lost wakeup in CARLA 0.9.16's Traffic Manager) ends in an error, a drain and a restart inside SSv2's timeouts; no TM unless a scenario needs one; a timed-out spawn's late actor adopted, actors looked up by id; then upstream's TM fix backported into carla-rust's prebuilt LibCarla
- [042: Autoware Fixes Carried as an Overlay](042-autoware-overlay.md) — upstream fixes Autoware 1.5.0 lacks, built one package at a time from the NEWSLabNTU fork at a pinned commit (`just build-autoware-overlay`) and sourced by `just ego-av` by default; first: `blind_spot`'s right-turn ego passage interval (autoware_universe#13509, from 033)

## Simulator backends and Isaac Sim (035-041, 043-047)

SSv2 on more than one simulator, CARLA and Isaac Sim, chosen per run; the repository keeps
its name ([design/simulator-backends.md](../design/simulator-backends.md)). Driven by a
sponsor's off-road competition on Isaac Sim (user direction 2026-10-16).

- [035: Backend Seam](035-backend-seam.md) — every carla-rust call behind a `SimBackend` trait, CARLA the only implementation, no behaviour change (done 2026-10-17: gap trace identical, frame budget not worse)
- [036: Isaac Sim Bring-up and the Domain Spike](036-isaac-bringup.md) — Isaac Sim 6.1 on this host (VNC and headless); several ROS domains in one process, Cyclone, lockstep, GPU cost per sensor set
- [037: Isaac World Server and `IsaacBackend`](037-isaac-world-server.md) — SSv2 scenarios on Isaac through a lockstep world server and a small batched protocol; puppeteered NPCs, contacts, no Autoware yet (done 2026-10-17: NPC poses within 0.04 mm, round trip p95 3.7 ms with 20 NPCs)
- [038: Isaac Vehicle Bridge](038-isaac-vehicle-bridge.md) — Autoware drives a PhysX vehicle: sensors, vehicle interface, `/clock` on its domain; agent unchanged (done 2026-10-18: `town01_ego_drive` 3/3 on Isaac, tracking inside acceptance.py's limits beside CARLA's, 20 ms per frame for one ego)
- [043: Single-Vehicle End to End on Isaac Sim](043-isaac-single-vehicle-e2e.md) — stage 1: CARLA's towns (SimReady, collider layer as code), CARLA's car with its blueprint physics, supervised world server, backend-aware `scenario`/`acceptance`/`e2e`
- [044: NPCs and Traffic Lights on Isaac Sim](044-isaac-npcs-and-traffic-lights.md) — stage 2: tracked dynamic NPCs, CARLA walkers, lamps set from SSv2 and signals given to Autoware
- [045: autoware_isaac_bridge (aib)](045-autoware-isaac-bridge.md) — the Isaac ego (PhysX vehicle, sensors, vehicle interface) as its own repository, acb's counterpart; one vehicle spec generates Autoware's vehicle and sensor kit files and the Isaac rig; csb_isaac then imports it
- [039: Several Autoware Instances on Isaac Sim](039-isaac-multi-autoware.md) — stage 3: one domain per ego, relay-coordinated, ego count per GPU measured; V2X; the gap campaign on Isaac
- [040: NTU Off-road Test Field](040-ntu-offroad-field.md) — the vendor's NTU mesh as an Isaac world, aligned to our Autoware maps, terrain friction zones, Lanelet2 course
- [041: Competition Director](041-competition-director.md) — heats, team isolation, scoring from ground truth, Isaac simulation-control services for the operator
- [046: One World Interface, then the Rename](046-unified-world-interface.md) — **parts A and B done** (2026-10-23; `just e2e sim:=carla` PASS; Isaac ego runs owed on a host that renders Town01; frame bound restated as the whole frame's, met) — every simulator a world server behind one protocol (CARLA moved out of the bridge into `carla_world`, Isaac on the same protocol, one conformance check), proven on both against a baseline; then the project and its packages renamed away from "carla" ([design/simulator-interface-split.md](../design/simulator-interface-split.md))
- [047: Off-road Surfaces](047-off-road-surfaces.md) — wet roads, puddles, gravel and mud/sand on CARLA-scale maps: a surface class per ground region everywhere (per-wheel PhysX friction), soil physics (SCM-style, sparse grid, ruts) only in authored zones, the road condition named in the scenario's `RoadCondition` ([design/off-road-surfaces.md](../design/off-road-surfaces.md))

035 and 036 can run in parallel; 037 needs both. After 038 the order is 043, 044, 039 (user
direction 2026-10-18); the target stack is [design/isaac-stack.svg](../design/isaac-stack.svg). 040 can start on the import script as soon as
the vendor delivers, but its acceptance needs 030.

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
2026-10-11; screencasts on the lab NAS, `autoveh/logs/2026 SSv2 on CARLA/2026-10-12 Starter scenario screencasts/`). Operational traps: `CLAUDE.md`.

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
2. ~~022 Entity geometry parity~~ -- done 2026-10-12.
3. ~~023 NPC motion parity~~ -- done 2026-10-13: `npc_motion: tracked` is the default
   (CARLA tick ~+7 ms p95 on the 20-NPC bench, accepted); `kinematic` stays as the fallback.
4. **024 Gap trace and oracle**, then **025**, **026**.
5. 019 and 009's classification, interleaved where a gap phase waits on a long run.
6. ~~027 User documentation~~ -- done 2026-10-12; three bridge follow-ups in its doc (shutdown
   respawn and relay traceback, a misleading "CARLA was restarted", a fallback-path log line).
7. **028 Run tooling** -- phase doc 2026-10-13; steps in order, smallest first (skill, retry,
   lint, map check, recorded runs, location sweeps).
8. **029 Prebuilt releases**, **030 Agent API** -- phase docs 2026-10-13; 030 takes 028's
   agent-interface step.
9. 031 Maps for scenarios -- deferred (user, 2026-10-13); analysis kept in its doc.
10. **032 NPC tracking fixes** -- phase doc 2026-10-13, from 024's baseline triage; before 025.
11. **033 Demo tour** -- phase doc 2026-10-14; Town01, continuous, 5-8 min. Done 2026-10-21:
    `just demo-tour` (route C at 40 km/h, six incidents, ambient layer, captions), 3/3
    rehearsals and one recorded take.
12. **034 Bridge failure paths** -- phase doc 2026-10-15; steps 1-5 before 033 step 5, then
    step 6 (LibCarla rebuild in carla-rust). Steps 1-5 done 2026-10-15, step 6 2026-10-16
    (carla 0.15.1, prebuilt 0.9.16-4).
