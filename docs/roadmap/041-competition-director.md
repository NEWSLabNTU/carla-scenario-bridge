# Phase 041: Competition Director

Run a sponsor's off-road driving competition on Isaac Sim: teams bring an Autoware-based
stack, the organizer runs heats on the NTU field, and results come from the simulation's
ground truth, reproducibly. The CARLA-API-like control the organizer needs is assembled from
Isaac's ROS 2 simulation-control services, csb and the world server.

**Source**: user direction 2026-10-16 ("host an off-road driving competition on Isaac Sim,
required by the sponsor; ways to control the simulation like the CARLA API").
**Depends on**: [039](039-isaac-multi-autoware.md), [040](040-ntu-offroad-field.md);
[024](024-gap-trace-and-oracle.md) (per-frame trace, reused for scoring).
**Feeds**: the competition itself; rules come from the sponsor (open).

## Control surfaces

| Need | Surface |
|---|---|
| Scenario per heat (start grid, course, NPCs, time limit, success and failure) | OpenSCENARIO through SSv2 and csb |
| Operator actions: load world, pause, resume, stop, reset, spawn and remove entities, step | Isaac `isaacsim.ros2.sim_control` (ROS `simulation_interfaces`: 19 services and the `SimulateSteps` action) on the organizer's domain |
| Ground truth for scoring | the gap trace (024) and the world server's contacts |
| Team interface | the team's Autoware on its own domain (039): sensors and vehicle interface in, control out; nothing else crosses |

## Design

- **Director** (`csb_launch competition_director`, Python): reads a heat definition (teams,
  domains, vehicle YAMLs, scenario, rules), brings up the world server and csb, starts team
  stacks or waits for remote ones, runs countdown, start, monitoring and stop, then scores.
- **Scoring** from the per-frame trace: course time, checkpoints in order, off-course time
  (outside the Lanelet2 course polygon), contacts with impulse (021), stops, interventions;
  rule parameters per competition. Offline and re-runnable on the trace.
- **Fairness and reproducibility**: fixed physics step and lockstep (037); each team's sensor
  set and vehicle from one approved YAML; seeds recorded; the heat's trace, logs and a video
  kept per heat.
- **Remote teams**: team stacks on their own hosts connect over DDS to the organizer's
  domain for that team, or over a bridge (Zenoh, as the remote-driving work does) when not on
  the LAN; a team can never reach another team's domain or the organizer's.
- **Spectating**: Isaac livestream allows one client per instance and has no authentication
  or encryption; spectating goes through a recorded or relayed view behind a TLS reverse
  proxy, never the raw stream endpoint.
- **Scoreboard**: generated from the scores per heat (static page or artifact).

## Steps

- [ ] Rules from the sponsor: vehicle, sensors, course, scoring, heat format (open).
- [ ] Heat definition format and the director's state machine; dry run with two of our stacks.
- [ ] Scoring from the trace; validated on hand-made traces with known outcomes.
- [ ] Remote team path: one external stack completes a heat over the bridge.
- [ ] Recording and scoreboard.
- [ ] Rehearsal: a full mock competition with at least three stacks, unattended between heats.

## Acceptance

- A mock competition of several heats runs unattended; scores re-computed from the stored
  traces match the live ones exactly.
- No team can see or publish into another team's domain (isolation test, as 039).
