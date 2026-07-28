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

- [x] Resolve a CARLA town from `InitializeRequest.lanelet2_map_path`
- [x] Configurable alias table for paths that do not follow the convention
- [x] `load_world()` when the resolved town differs from the loaded one
- [x] Skip the reload when it already matches — reloading is slow and destroys actors
- [x] Unresolvable or failed load fails `Initialize` with a descriptive `Result`, rather than
      running the scenario on the wrong map
- [x] Document the interaction with [007](007-repeatable-runs.md): `load_world` wipes all
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

- [x] Enumerate CARLA `TrafficLight` actors at `Initialize`
- [x] Parse the Lanelet2 map named in `lanelet2_map_path` for signal positions
- [x] Position-based matching within a documented tolerance
- [x] Per-map YAML fallback in `config/` for what matching misses
- [x] Unmapped IDs warn once per ID and never abort
- [x] Report mapped/unmapped counts at `Initialize` so coverage is visible before the run

Keyed on **OpenDRIVE sign ID, not CARLA actor ID**. Actor IDs are assigned at spawn and
differ every session, so the mapping the phase 004 design described would be stale the moment
CARLA restarts. `World::traffic_light_from_open_drive` resolves a stable sign ID to a live
actor at runtime.

The YAML file takes precedence over position matching — it exists to correct what matching
gets wrong, so an automatic result must never silently replace a hand-written one.

Two findings from the real TUM maps, neither of which could have been guessed:

- **Identification cannot use the `subtype` tag.** Town01 reads
  `red_redYellow_green_yellow`, but Town03's is corrupted to
  `_OLYeaShicfaTNEGeaShicfaTWLE_E.tttgLifr_E.tttgLifr` — mangled text where a bulb layout
  should be. A subtype allowlist silently hid all 38 of Town03's lights. Identification is
  now structural: a way referenced by a `regulatory_element` with `role="refers"`.
- **Node coordinates are not always local.** Town03's traffic light nodes carry only
  `lat`/`lon`, no `local_x`/`local_y`. The maps declare `projector_type: local` about the
  origin, so `local = radians(lat|lon) × 6378137.0`. Validated against the 20,000 Town01
  nodes carrying both forms: **maximum error 0.0000 m**.

Parser output across the pack, before and after:

| Town | subtype allowlist | structural + lat/lon |
|---|---|---|
| Town01 | 36 | 36 |
| Town02 | 24 | 24 |
| Town03 | **0** | **38** |
| Town05 | 54 | **55** |
| Town10 | 17 | **18** |

An `#[ignore]`d test parses the real pack when it is present; run with
`cargo test -- --ignored --nocapture`.

### Traffic light control (A3, gap 5)

- [x] Freeze all CARLA traffic lights at `Initialize`
- [x] `update_traffic_lights` converts SSv2 `TrafficSignal` to CARLA state and applies it
- [x] Handle multiple bulbs per signal — SSv2 can send arrow and circle states together
- [x] Document the arrow-state limitation: CARLA does not distinguish arrows from circles
- [x] Unfreeze at shutdown (shared with [007](007-repeatable-runs.md) B4)
- [x] Remove the phase-006 honest-failure rejection once this lands

Bulbs reduce by **most restrictive wins**: any lit red makes the light red, then amber, then
green. A red circle beside a green left arrow is a red light to everything except a
left-turning vehicle, and reporting green would invite the ego through a red.

Freezing leaves uncommanded lights in an arbitrary but *fixed* state. That is the deliberate
trade — a cycling light near the ego's route would make the run non-deterministic, which is
what invariant 3 exists to prevent. A scenario that cares about a signal must command it.

### Autoware perception (gap 7)

- [ ] Re-enable `use_traffic_light_recognition` in `acb_launch/carla_simulator.launch.xml`
- [ ] Restore the traffic-light topic in the `component_state_monitor` config
- [ ] Re-check the diagnostics consequences — the flag was turned off for a reason, and
      `duplicated_node_checker` errors were attributed to it
- [ ] Confirm the camera sensor's placement and resolution can actually resolve signals
- [ ] If recognition proves impractical, record the decision and fall back to ground-truth
      injection rather than leaving the flag silently off

### Dead code (gap 6)

- [x] Replace csb's `TrafficLightMapper` stub, which mapped to actor IDs and could not work
- [ ] Remove `acb_bridge`'s `TrafficLightBridge` stub, or implement it with a stated purpose
- [x] Confirm removal does not violate invariant 3 — csb remains the only writer

### Tests

- [x] Unit: map path → town resolution, including the unresolvable case
- [x] Unit: SSv2 `TrafficSignal` → CARLA state, every row of the table
- [x] Unit: lanelet ID → CARLA signal matching, including the Y-flip and double-claim cases
- [x] Unit: Lanelet2 parsing, including the corrupted-subtype and lat/lon-only cases
- [x] Real-map: all five TUM towns parse (`--ignored`, needs the map pack)
- [ ] Integration: `Initialize` loads the right town and freezes every light
- [ ] Integration: a commanded red is observable in CARLA
- [ ] Integration: lights return to cycling after shutdown

## Acceptance Criteria

- [ ] A Town01 scenario run against a CARLA holding another town loads Town01 or fails clearly
- [ ] All CARLA traffic lights are frozen for the duration of a run
- [ ] An SSv2-commanded red is red in CARLA; green is green
- [ ] Position matching resolves at least 80% of Town01 signal IDs automatically
- [x] Unmapped IDs warn without aborting
- [ ] Autoware's traffic light recognition reports the state SSv2 commanded
- [ ] Ego stops at an SSv2-commanded red in a full-stack run
- [ ] Lights cycle normally again after the scenario ends
- [x] `just test` passes

Everything unchecked needs a live CARLA. The 80% matching criterion in particular cannot be
evaluated without one: the Lanelet2 side is verified against the real pack (171 lights across
five towns), but nothing has been paired against actual CARLA light positions.

## Still Open

**Gap 7 — Autoware traffic light recognition** is untouched. `acb_launch` still sets
`use_traffic_light_recognition=false` and excludes the topic from `component_state_monitor`,
so nothing in Autoware reacts to a commanded signal yet. Re-enabling needs a live stack to
re-check the diagnostics consequences, which is why it was not attempted blind.

**The map pack URL is dead.** `scripts/download_maps.sh` returns HTTP 404 from LRZ
Sync+Share. The script now accepts `CSB_MAP_SOURCE=<dir>` and symlinks town folders from a
local or NAS copy instead; symlinks rather than copies, since the pack is ~1 GB.
