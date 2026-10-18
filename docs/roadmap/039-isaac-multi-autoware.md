# Phase 039: Several Autoware Instances on Isaac Sim

What 010 and 017 deliver on CARLA, on Isaac: several Autoware egos in one Isaac world, each in
its own ROS domain, all registered with the agent relay, SSv2 in the scenario domain (9)
coordinating them.

**Source**: user direction 2026-10-16 ("each Autoware instance runs in a separate ROS
domain").
**Depends on**: [038](038-isaac-vehicle-bridge.md); 036's cost measurement.
**Feeds**: [041](041-competition-director.md) (competitors are egos).

## Design

- **Domains**: ego k on domain k (k = 1, 2, ...), SSv2 and csb on 9, as on CARLA (017). The
  world server holds one ROS 2 context per ego (036's mechanism); nothing of ego k is visible
  on domain j.
- **Ego roles**: the scenario's ego plus background AVs as SSv2 entities with controller
  `agent` (017); each is spawned physics driven through `IsaacBackend` with its vehicle YAML.
- **Autoware stacks**: one per ego, launched as on CARLA (`just ego-av` with a domain); each
  needs the map dir and the vehicle's sensor kit.
- **Per-ego cost, from 038**: one ego frame costs 20-22 ms p50 at the world server (physics
  and render 9 ms, its ROS side 5, lidar 2, the Python vehicle the rest), not 036's 5 ms per
  bare sensor set: size the ego count on that, and move the per-ego ROS work off the loop if
  it binds. 038 makes the ego's rclpy context once, at server start, for its one domain
  (`EgoRos`): 039 makes one per configured ego, keyed by the ego YAML's domain.
- **GPU budget**: from 036, the number of full sensor sets one 5090 holds at real time. Beyond
  it: reduced sensor sets for background AVs (lidar only, or none: a background AV that only
  needs to drive can run on ground-truth localization), or a second host with the world server
  rendering split by ego (later; not in this phase).
- **Process hygiene**: the world server and each ego stack under their own process group
  (CLAUDE.md "Account for every process a run starts"); the world server is a tracked
  singleton like the bridge.

## Steps

- [ ] Two egos, two domains: isolation test (topics and `/clock` per domain only).
- [ ] Two Autoware stacks drive an interaction scenario (the multi-AV starter scenario,
      adapted to the Isaac test world); SSv2 verdict.
- [ ] Scale test: maximum egos at real time with full and with reduced sensor sets; recorded.
- [ ] Teardown and repeat: five consecutive runs unattended, no orphan processes.

## Acceptance

- The multi-AV starter scenario passes on Isaac with two Autoware stacks, five runs in a row.
- Measured ego count per host at real time, full and reduced sensor sets, in this doc.
