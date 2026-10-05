# Phase 5: Hardening + AWF Contribution

Production-quality adapter with error recovery, performance validation, and upstream contribution to the Autoware Foundation's safety validation pipeline.

## Design

### Error Handling and Recovery

The adapter must handle failures gracefully since it sits between two complex systems:

**CARLA disconnection**: CARLA may crash or be restarted during a scenario.
- Detect disconnection via failed API calls (spawn, tick, set_transform)
- Enter a reconnection loop (same infinite retry pattern as `autoware_carla_bridge`)
- On reconnect: re-initialize sync mode, but entity state is lost (scenario must restart)
- Report failure to SSv2 via `Result { success: false, description: "CARLA disconnected" }`

**Entity spawn failures**: Blueprint not found, spawn location occupied, actor limit reached.
- Return `Result { success: false }` with a descriptive error message
- SSv2's interpreter handles spawn failures based on scenario logic
- Don't crash the adapter - log the error and continue processing other requests

**Protobuf deserialization failures**: Malformed or unknown message types.
- Log the error with raw message bytes for debugging
- Return a generic error response if possible
- Don't crash - continue polling for next message

**ZMQ connection issues**: SSv2 disconnects mid-scenario.
- ZMQ REP socket handles reconnection automatically
- Reset entity state if SSv2 sends a new `Initialize` (clean slate)

### Performance Profiling

The adapter must not be a bottleneck in the simulation loop. Key metrics:

**Per-frame latency**: Time from receiving `UpdateEntityStatus` to sending response.
- Target: < 5ms for 20 entities (20Hz sim = 50ms budget per frame)
- Bottlenecks: CARLA `set_transform()` calls (one per NPC), `actor.transform()` readback
- Mitigation: batch CARLA API calls if carla-rust supports it

**Tick latency**: Time for `world.tick()` to complete.
- This is CARLA's rendering/physics time, not controllable by the adapter
- Log tick duration for profiling; warn if > 100ms

**Entity update throughput**: Number of entities updated per second.
- Profiled with 10, 50, 100 entities to find scaling limits
- CARLA's `set_transform()` is fast (no physics), but many actors stress the rendering pipeline

**Instrumentation**: Use `tracing` spans for key operations:
```rust
#[tracing::instrument(skip(self))]
fn update_entity_status(&mut self, req: UpdateEntityStatusRequest) -> UpdateEntityStatusResponse {
    // ...
}
```

### SafetyPool Scenario Integration

The AWF ODD Working Group is working on running [SafetyPool](https://www.safetypool.ai/) scenarios for Autoware validation. These scenarios use OpenSCENARIO with specific patterns:

- `LanePosition` references with OpenDRIVE `roadId`/`laneId` (not Lanelet2)
- `ReachPositionCondition` for pass/fail
- `CollisionCondition` for safety violations
- `SimulationTimeCondition` for timeout

Integration steps:
1. Use the scenario conversion script from [AWF odd-usecase-scenario#117](https://github.com/autowarefoundation/odd-usecase-scenario/issues/117) to convert SafetyPool scenarios to SSv2-compatible format
2. Run converted scenarios through SSv2 + this adapter + CARLA + Autoware
3. Verify scenario conditions evaluate correctly with CARLA physics

### Lane-to-Lanelet Mapping Tool

AWF [odd-usecase-scenario#118](https://github.com/autowarefoundation/odd-usecase-scenario/issues/118) needs a tool that outputs `(OpenDRIVE road_id, lane_id) -> lanelet_id` mappings as CSV. This is needed for:
1. Scenario conversion (Phase 5 SafetyPool scenarios)
2. Traffic light mapping (Phase 4)

We have experience with CommonRoad Scenario Designer (used in `autoware_carla_bridge` for map conversion). The approach:
1. Parse the OpenDRIVE file and the Lanelet2 map (both generated from the same source)
2. For each OpenDRIVE lane, find the corresponding lanelet by geometric matching (centerline proximity)
3. Output as CSV: `road_id,lane_id,lanelet_id`

This is a standalone tool, not part of the runtime adapter.

### AWF Contribution

The adapter and lane-to-lanelet mapping tool address two open AWF issues:
- [odd-usecase-scenario#114](https://github.com/autowarefoundation/odd-usecase-scenario/issues/114) - "Integrate the scenario interpreter with the CARLA simulator"
- [odd-usecase-scenario#118](https://github.com/autowarefoundation/odd-usecase-scenario/issues/118) - "Adapt OpenDRIVE to Lanelet2 conversion to output lane-to-lanelet mapping"

Contribution preparation:
1. Clean up code for external consumption (documentation, examples, CI)
2. Write a setup guide for AWF reviewers
3. Open PRs or link the adapter repo in the AWF issues
4. Coordinate with Simulation WG (they own the runtime integration per #84)

## Tasks

**Scope note (2026-10-06).** Patches to scenario_simulator_v2 and autoware_launch stay on our
forks (NEWSLabNTU SSv2 `managed-ego-unforked`; acb_launch's generated perception component);
fixes to play_launch go to its `main` when they break nothing. "AWF contribution" below
therefore means offering *this adapter* and its tools to the community, not upstreaming the
fork patches.

### Error Handling
- [x] CARLA disconnection detection and reconnection loop
      `note_carla_failure` / `reconnect_carla` (coordinator.rs) on repeated tick failures (audit 2026-10-06)
- [x] Entity spawn failure: return `Result { success: false }` with description
      phase 006: every handler returns `success: false` with a description on failure (audit 2026-10-06)
- [x] Protobuf deserialization failure: log and return error response
      `zmq_server.rs` logs and replies `Failed to decode request` (audit 2026-10-06)
- [x] ZMQ reconnection: handle SSv2 disconnect/reconnect (reset state on new `Initialize`)
      each `Initialize` resets the session; consecutive scenarios run on one bridge daily (audit 2026-10-06)
- [ ] Graceful degradation: adapter never crashes from CARLA or SSv2 errors
      **Open.** Not provable: carla-rust/LibCarla can segfault the process (a callback across a `load_world` did, 2026-10-05; `load_world` of an unknown town does, which csb guards by validating the name) (audit 2026-10-06)

### Performance
- [ ] Add `tracing` spans to all 14 handlers
      **Open**: no spans yet (audit 2026-10-06)
- [ ] Log per-frame timing: `UpdateEntityStatus` duration, `world.tick()` duration
      **Open** (audit 2026-10-06)
- [ ] Benchmark with 10 NPC entities at 20Hz: measure frame budget usage
      **Open**. Context: two full stacks now run CARLA at 19.96 Hz (014) (audit 2026-10-06)
- [ ] Benchmark with 50 NPC entities: identify scaling limits
      **Open** (audit 2026-10-06)
- [ ] Warn if frame processing exceeds 25ms (50% of 20Hz budget)
      **Open** (audit 2026-10-06)

### SafetyPool Scenarios
- [ ] Convert 1 SafetyPool scenario to SSv2 format using AWF conversion script
      **Open** (audit 2026-10-06)
- [ ] Run converted scenario end-to-end: SSv2 -> adapter -> CARLA -> bridge -> Autoware
      **Open** (audit 2026-10-06)
- [x] Verify `ReachPositionCondition` passes when ego arrives at goal
      ego_drive and traffic_light pass on it (audit 2026-10-06)
- [x] Verify `CollisionCondition` triggers on collision with NPC
      `town01_rear_contact.xosc`; CARLA agrees if the contact lasts more than a frame (scenario-authoring.md) (audit 2026-10-06)
- [x] Verify `SimulationTimeCondition` triggers on timeout
      every timed-out run of 014 failed on it as intended (audit 2026-10-06)
- [ ] Document conversion workflow and any manual adjustments needed
      **Open** (audit 2026-10-06)

### Lane-to-Lanelet Mapping Tool
- [ ] Parse OpenDRIVE file (road/lane structure)
      **Open**. Only traffic signals are matched today (csb's position matcher, lanelet way ↔ CARLA light, 36/36 on Town01) (audit 2026-10-06)
- [ ] Parse Lanelet2 map (lanelet IDs and centerlines)
- [ ] Geometric matching: for each OpenDRIVE lane, find nearest lanelet by centerline proximity
- [ ] Output CSV: `road_id,lane_id,lanelet_id`
- [ ] Validate on Town01: compare with manually verified mappings
- [ ] Test on Town10HD: verify generalization

### AWF Contribution Preparation
- [ ] Repository cleanup: README, LICENSE, CI, examples
      **Partly**: README and example scenarios exist; no LICENSE file, no CI (audit 2026-10-06)
- [ ] Setup guide: prerequisites, build, run with SSv2 + CARLA + Autoware
- [x] Example scenario: minimal .xosc that runs end-to-end
      `scenarios/town01_ego_drive.xosc` (audit 2026-10-06)
- [ ] Comment on AWF issues #114 and #118 with links to the adapter and mapping tool
      **Open**; an external action, your call (audit 2026-10-06)
- [ ] Coordinate with Simulation WG on integration testing

## Acceptance Criteria

- [ ] Adapter recovers from CARLA crash without itself crashing (reconnects and waits for new `Initialize`)
      **Not verified live**: after this week's CARLA crashes the bridge was restarted by hand each time (audit 2026-10-06)
- [x] Entity spawn failure returns a descriptive error to SSv2 (not a crash)
      phase 006 (audit 2026-10-06)
- [ ] 20 NPC entities at 20Hz: per-frame processing < 10ms (excluding CARLA tick time)
- [ ] At least 1 SafetyPool scenario runs end-to-end with correct pass/fail result
- [ ] Lane-to-lanelet mapping tool produces correct CSV for Town01 (validated against known mappings)
- [ ] Documentation sufficient for an AWF contributor to set up and run the adapter
- [ ] AWF issues #114 and #118 updated with links to this project's contributions
