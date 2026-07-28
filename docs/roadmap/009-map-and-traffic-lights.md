# Phase 009: Map Loading and Traffic Lights

Load the map the scenario asks for, and make SSv2 authoritative over signals.

**Source**: gaps 4-7 from
[multi-instance-architecture.md](../design/multi-instance-architecture.md), plus audit
findings A3 and B4.
**Supersedes**: [004-traffic-lights-environment.md](004-traffic-lights-environment.md), whose
ambient-traffic section is rejected outright.
**Depends on**: [006](006-honest-failures.md), [007](007-repeatable-runs.md).

## Problem

`InitializeRequest` carries `lanelet2_map_path`, and the bridge ignores it. `initialize()`
never calls `load_world()`, so whichever town CARLA happens to have loaded is the town the
scenario runs on. A scenario written for Town01 against a CARLA holding Town05 does not fail —
it runs, and every pose is meaningless.

Traffic lights are dead in all three places at once: the csb handler is a stub, acb's
`TrafficLightBridge` is `#[allow(dead_code)]`, and `acb_launch` sets
`use_traffic_light_recognition=false`. CARLA's own cycling runs unfrozen throughout, so lights
change on CARLA's schedule while SSv2 believes it controls them.

The mapping problem is the real work. Lanelet2 regulatory element IDs and CARLA's
OpenDRIVE-derived `TrafficLight` actors share no ID space.

## Work Items

### Map resolution and loading (gap 4)

- [ ] Resolve a CARLA town from `InitializeRequest.lanelet2_map_path`
- [ ] Configurable alias table for paths that do not follow the convention
- [ ] `load_world()` when the resolved town differs from the loaded one
- [ ] Skip the reload when it already matches — reloading is slow and destroys actors
- [ ] Unresolvable or failed load fails `Initialize` with a descriptive `Result`, rather than
      running the scenario on the wrong map
- [ ] Document the interaction with [007](007-repeatable-runs.md): `load_world` wipes all
      actors, so teardown bookkeeping must be reset

### Verify carla-rust coverage

- [x] Confirm `Client::load_world`, `TrafficLight::freeze` and `TrafficLight::set_state` are
      exposed — verified 2026-07-28 during phase 007
- [x] If any binding is missing, add it upstream — **not needed**, everything required exists
- [x] Record the outcome in the design doc's open-items section

Note the crate in use is **not** crates.io `carla` 0.14.0: a `[patch.crates-io]` in the
workspace root redirects to `jerry73204/carla-rust` rev `73f5e16`, whose API differs (every
call returns `crate::Result<T>`). Check that source, not docs.rs.

Confirmed available:

| Need | API |
|---|---|
| Load a map | `Client::load_world`, `load_world_if_different`, `avaiable_maps` (upstream typo) |
| Freeze signals | `World::freeze_all_traffic_lights(bool)`, `World::reset_all_traffic_lights()` |
| Per-signal control | `TrafficLight::{freeze, is_frozen, set_state, state}` |
| Signal discovery | `World::{traffic_light_at, traffic_light_from_open_drive, traffic_lights_in_junction, traffic_lights_from_waypoint}` |
| Lanelet matching | `Map::{all_landmarks, landmarks_from_id, all_landmarks_of_type}`, `TrafficLight::{opendrive_id, sign_id, affected_lane_waypoints, stop_waypoints}` |

`World::traffic_light_from_open_drive(sign_id)` is worth trying before position matching —
if the Lanelet2 map preserves OpenDRIVE signal IDs, it makes the mapping exact rather than
approximate.

### Signal mapping (gap 5)

- [ ] Enumerate CARLA `TrafficLight` actors at `Initialize`
- [ ] Parse the Lanelet2 map named in `lanelet2_map_path` for signal positions
- [ ] Position-based matching within a documented tolerance
- [ ] Per-map YAML fallback in `config/` for what matching misses
- [ ] Unmapped IDs warn once per ID and never abort
- [ ] Report mapped/unmapped counts at `Initialize` so coverage is visible before the run

### Traffic light control (A3, gap 5)

- [ ] Freeze all CARLA traffic lights at `Initialize`
- [ ] `update_traffic_lights` converts SSv2 `TrafficSignal` to CARLA state and applies it
- [ ] Handle multiple bulbs per signal — SSv2 can send arrow and circle states together
- [ ] Document the arrow-state limitation: CARLA does not distinguish arrows from circles
- [ ] Unfreeze at shutdown (shared with [007](007-repeatable-runs.md) B4)
- [ ] Remove the phase-006 honest-failure rejection once this lands

### Autoware perception (gap 7)

- [ ] Re-enable `use_traffic_light_recognition` in `acb_launch/carla_simulator.launch.xml`
- [ ] Restore the traffic-light topic in the `component_state_monitor` config
- [ ] Re-check the diagnostics consequences — the flag was turned off for a reason, and
      `duplicated_node_checker` errors were attributed to it
- [ ] Confirm the camera sensor's placement and resolution can actually resolve signals
- [ ] If recognition proves impractical, record the decision and fall back to ground-truth
      injection rather than leaving the flag silently off

### Dead code (gap 6)

- [ ] Remove `acb_bridge`'s `TrafficLightBridge` stub, or implement it with a stated purpose
- [ ] Confirm removal does not violate invariant 3 — csb remains the only writer

### Tests

- [ ] Unit: map path → town resolution, including the unresolvable case
- [ ] Unit: SSv2 `TrafficSignal` → CARLA state, every row of the table
- [ ] Unit: lanelet ID → actor matching against a fixture
- [ ] Integration: `Initialize` loads the right town and freezes every light
- [ ] Integration: a commanded red is observable in CARLA
- [ ] Integration: lights return to cycling after shutdown

## Acceptance Criteria

- [ ] A Town01 scenario run against a CARLA holding another town loads Town01 or fails clearly
- [ ] All CARLA traffic lights are frozen for the duration of a run
- [ ] An SSv2-commanded red is red in CARLA; green is green
- [ ] Position matching resolves at least 80% of Town01 signal IDs automatically
- [ ] Unmapped IDs warn without aborting
- [ ] Autoware's traffic light recognition reports the state SSv2 commanded
- [ ] Ego stops at an SSv2-commanded red in a full-stack run
- [ ] Lights cycle normally again after the scenario ends
- [ ] `just test` passes
