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
- [012: SSv2-Unmanaged Autoware](012-ssv2-unmanaged-autoware.md) — SSv2 stops launching Autoware; every stack launched externally, one lifecycle mechanism

### Why this order

006 comes first because three handlers currently report success while doing nothing, so a
scenario suite can score fiction — no later phase can be validated until that stops.

007 comes second because nothing destroys spawned actors and spawn has no retry, so a leaked
actor blocks the next run's spawn point. The second run of any scenario currently fails until
CARLA is restarted, which makes iterative work on 008-011 impractical.

The rest follow their dependencies. 011 can be pulled forward if spurious CARLA reconnects
start appearing — 009 adds per-frame work and makes that likelier.

012 comes after 010 because it generalises 010's per-domain pilot to the ego; the pilot must
exist before every run depends on it. Its spike (what `launch_autoware:=false` actually does
in SSv2 25.0.22) can run any time — it only reads code and bounds the phase.

## Status

Gaps 1-3 from the design doc are fixed (deferred sync mode, `/clock` ownership,
sensor-timestamp epochs). Gaps 4-11 are distributed across 009, 010 and 011.
