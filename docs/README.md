# Documentation

## Design
- [Architecture](design/architecture.md) - System overview, component responsibilities, AWSIM comparison
- [Multi-Instance Architecture](design/multi-instance-architecture.md) - Authority model, background AVs, clock ownership, gap list ⭐
- [SSv2 Protocol](design/ssv2-protocol.md) - ZMQ+Protobuf message reference
- [SSv2 Launch Configuration](design/ssv2-launch-configuration.md) - Launch parameters, process order, topic conflicts

## Roadmap
- [Overview](roadmap/README.md) - Phase index and ordering rationale

Original plan, pre-dating the authority model:
- [001: Core ZMQ Adapter](roadmap/001-core-zmq-adapter.md) - ZMQ server, CARLA connection, entity lifecycle, coordinate conversion
- [002: Autoware Integration](roadmap/002-autoware-integration.md) - Sensor pipeline, ego physics readback, end-to-end driving
- [003: NPC + Pedestrian Support](roadmap/003-npc-pedestrian-support.md) - superseded by 008
- [004: Traffic Lights + Environment](roadmap/004-traffic-lights-environment.md) - superseded by 009
- [005: Hardening + AWF Contribution](roadmap/005-hardening-awf-contribution.md) - SafetyPool scenarios, AWF upstream

Audit-driven, from the 2026-07-28 workflow audit:
- [006: Honest Failures](roadmap/006-honest-failures.md) - stop returning success for work not done ⭐
- [007: Repeatable Runs](roadmap/007-repeatable-runs.md) - actor teardown, spawn retry, reconnection
- [008: Entity Fidelity](roadmap/008-entity-fidelity.md) - pedestrians, misc objects, NPC physics
- [009: Map and Traffic Lights](roadmap/009-map-and-traffic-lights.md) - map loading, signal mapping, recognition
- [010: Multi-Instance](roadmap/010-multi-instance.md) - config loading, role names, background AVs
- [011: Robustness](roadmap/011-robustness.md) - tick timeouts, ego respawn, duplicate publishers
