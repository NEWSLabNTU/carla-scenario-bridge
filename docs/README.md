# Documentation

## Design
- [Architecture](design/architecture.md) - System overview, component responsibilities, AWSIM comparison
- [SSv2 Protocol](design/ssv2-protocol.md) - ZMQ+Protobuf message reference
- [SSv2 Launch Configuration](design/ssv2-launch-configuration.md) - Launch parameters, process order, topic conflicts

## Roadmap
- [Overview](roadmap/README.md)
- [Phase 1: Core ZMQ Adapter](roadmap/1-core-zmq-adapter.md) - ZMQ server, CARLA connection, entity lifecycle, coordinate conversion
- [Phase 2: Autoware Integration](roadmap/2-autoware-integration.md) - Sensor pipeline, ego physics readback, end-to-end driving
- [Phase 3: NPC + Pedestrian Support](roadmap/3-npc-pedestrian-support.md) - Multi-actor scenarios
- [Phase 4: Traffic Lights + Environment](roadmap/4-traffic-lights-environment.md) - Traffic light control, weather, ambient traffic
- [Phase 5: Hardening + AWF Contribution](roadmap/5-hardening-awf-contribution.md) - Error recovery, SafetyPool scenarios, upstream
