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

Two hardening fixes landed 2026-08-08, both found by the probe against carla-rust
`575f6aa`: `load_world` on a town the server does not have **segfaults** inside the binding
(the C++ exception never crosses the FFI as an `Err`), so the town is now validated against
`avaiable_maps()` before loading; and a first map load under rendering took 66 s, past the
30 s RPC timeout, so the timeout is widened to 120 s for the load and restored afterwards.
The FFI exception problem is tracked in [007](007-repeatable-runs.md)'s verification notes.

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

- [x] Re-enable `use_traffic_light_recognition` in `acb_launch/carla_simulator.launch.xml`
      — it was already on; what was missing was everything downstream of it, below
- [x] Restore the traffic-light topic in the `component_state_monitor` config
- [x] Re-check the diagnostics consequences — no `duplicated_node_checker` errors
      appeared; the ego stack comes up 50/50 with recognition running
- [x] Confirm the camera sensor's placement and resolution can actually resolve signals
      — it could not, and now can: 160x80 and yawed 59 deg off the centreline, now
      1280x720 and forward-facing
- [x] If recognition proves impractical, record the decision and fall back to ground-truth
      injection rather than leaving the flag silently off — **it is impractical against
      this map pack, for a reason no amount of camera work fixes.** See below. The
      fallback is not yet built.

Four separate defects sat between a commanded signal and Autoware, each silent:

1. **The map's traffic lights were untyped.** Four of the five towns ship regulatory
   elements with `subtype=""` and light ways with `type=""` where both should read
   `traffic_light`. lanelet2 then builds a `GenericRegulatoryElement`, SSv2 rejects the id
   outright — *"Given lanelet ID 43763 is neither a traffic light ID not a traffic relation
   ID"* — and Autoware finds no signals on the map at all. Repaired by
   `scripts/repair_lanelet_traffic_lights.py`, wired into `download_maps.sh`: 152 lights
   across Town01/02/03/05, Town10 already correct and left alone.
2. **Recognition ran on camera namespaces nothing published.** Upstream defaults to
   `[camera6, camera7]`, the namespaces of a real vehicle's sensor kit.
3. **A camera namespace with no parameter file kills the detector silently.** Autoware
   derives `config/perception/.../<namespace>_traffic_light_map_based_detector.param.yaml`
   from the namespace and ships only `camera6` and `camera7`. Pointing it at `camera0`
   made `traffic_light_map_based_detector` exit at startup — the launch reported *"Exited
   without code"* and carried on, and the pipeline published empty signal arrays forever.
   That is why acb's traffic-light camera is named **camera6**: it is the namespace with a
   file, not a description.
4. **acb published its camera on the wrong topic.** The topic was built from the whole TF
   frame, giving `/sensing/camera/camera6/camera_link/image_raw`, while Autoware
   subscribes to `/sensing/camera/camera6/image_raw`. No image had ever reached the
   pipeline under any camera name. Fixed in `acb_bridge`'s `autoware.rs`; images now
   arrive at 9.3 Hz.

With all four fixed the pipeline is genuinely alive — 14 nodes, 50/50 at startup, the
map-based detector publishing `expect/rois` every frame — and it still reports nothing,
because of a fifth problem the pack cannot be talked out of.

**The pack's traffic light geometry is a placeholder.** A lanelet2 traffic light is a
linestring across the face of the signal head: its length is the head's width and its
bearing is what the light faces. In this pack every light in every town is the same stub:

| town | bar length | distinct bearings |
|---|---|---|
| Town01 | 0.141 m | 1 (135 deg) |
| Town02 | 0.141 m | 1 (135 deg) |
| Town05 | 0.141 m | 1 (135 deg) |
| Town10 | 0.035 m | 1 (135 deg) |

Thirty-six lights on Town01 govern roads running north, south, east and west, and all
thirty-six claim to face 135 degrees. Two consequences, either one fatal:
`car_traffic_light_max_angle_range` is 40 degrees, so a light 45 degrees off every
approach is rejected before projection; and a 0.141 m bar at 30 m subtends 0.27 degrees,
about 5 px at 1280x720, which is not something the fine detector can find or the
classifier can read. Note Town10 is *correctly tagged* and still has stub geometry, so
this is not the same corruption as the missing type tags — it is what the converter emits.

Recognition against this pack is therefore not a tuning problem. It needs either light
geometry regenerated from CARLA's own `TrafficLight` actors (their `bounding_box` and
transform give both width and facing) or the ground-truth injection path this work item
allows for.

### Dead code (gap 6)

- [x] Replace csb's `TrafficLightMapper` stub, which mapped to actor IDs and could not work
- [x] Remove `acb_bridge`'s `TrafficLightBridge` stub, or implement it with a stated purpose
      — removed 2026-08-08 (acb `c578eb8`); the actor factory now returns an explicit error
      for traffic-light actors instead of a do-nothing bridge
- [x] Confirm removal does not violate invariant 3 — csb remains the only writer

### Tests

- [x] Unit: map path → town resolution, including the unresolvable case
- [x] Unit: SSv2 `TrafficSignal` → CARLA state, every row of the table
- [x] Unit: lanelet ID → CARLA signal matching, including the Y-flip and double-claim cases
- [x] Unit: Lanelet2 parsing, including the corrupted-subtype and lat/lon-only cases
- [x] Real-map: all five TUM towns parse (`--ignored`, needs the map pack)
- [x] Integration: `Initialize` loads the right town and freezes every light — probe,
      re-verified 2026-08-08 (fresh server holding Town10HD_Opt loaded Town01; 36 frozen)
- [x] Integration: a commanded red is observable in CARLA — verified live 2026-07-30, all
      36 signals, RED/GREEN/AMBER (see below)
- [x] Integration: lights return to cycling after shutdown — verified 2026-08-08: after
      SIGINT, 0 of 36 lights frozen

## Acceptance Criteria

- [x] A Town01 scenario run against a CARLA holding another town loads Town01 or fails clearly
- [x] All CARLA traffic lights are frozen for the duration of a run
- [x] An SSv2-commanded red is red in CARLA; green is green
- [x] Position matching resolves at least 80% of Town01 signal IDs automatically — **100%**
- [x] Unmapped IDs warn without aborting
- [ ] Autoware's traffic light recognition reports the state SSv2 commanded — **blocked
      by the pack's stub light geometry**, not by wiring; see gap 7 above
- [ ] Ego stops at an SSv2-commanded red in a full-stack run — **not earned.** The ego
      does stop at the stop line, with `behavior: traffic-signal` and 0.35 m to go, but it
      stops the same way whatever the light says: SSv2 turned the signal green in CARLA
      mid-run (`GREENx1` on the ground-truth actor) and the ego sat there until the 300 s
      timeout. That is the traffic-signal module being conservative about a signal it has
      no state for, which is the correct behaviour and the wrong evidence
- [x] Lights cycle normally again after the scenario ends — after SIGINT, 0 of 36 frozen
- [x] `just test` passes

Signal matching resolved **36 of 36** Town01 signals, well past the 80% criterion.

`scenarios/town01_traffic_light.xosc` is the scenario for the two open criteria: the ego
drives 219 m west to a signal SSv2 holds red, and SSv2 turns it green at sim 150 s. It
fails today, and it should keep failing until recognition works — it is the test, not a
regression.

## Still Open

**Gap 7 — Autoware traffic light recognition** is wired end to end and produces nothing,
for the map reason above. The next move is one of two, and it is a design decision rather
than a bug to chase:

1. **Regenerate the light geometry from CARLA.** Each CARLA `TrafficLight` actor carries a
   transform and a `bounding_box`, which is exactly the width and facing the lanelet2
   linestring is supposed to encode, and csb already resolves lanelet element to CARLA
   actor for all 36 Town01 signals. A generator pass over the map would make recognition
   possible for every town in the pack. It also makes the maps diverge from the pack, so
   it belongs beside `repair_lanelet_traffic_lights.py` as an explicit, re-runnable step.
2. **Ground-truth injection**, which this phase's work items already allow: publish the
   commanded states onto `/perception/traffic_light_recognition/traffic_signals` and skip
   the camera entirely. Cheaper, and it tests planning's reaction rather than perception's.
   It needs a decision about who writes it — invariant 3 makes csb the only writer of
   signal state, and this would be a second interface into Autoware.

Option 1 is the one worth doing if traffic-light *perception* is ever in scope; option 2
if only the ego's *reaction* is. Nothing about the camera path needs revisiting either
way: the four wiring defects are fixed and the pipeline is alive.

**The map pack URL is dead.** `scripts/download_maps.sh` returns HTTP 404 from LRZ
Sync+Share. The script now accepts `CSB_MAP_SOURCE=<dir>` and symlinks town folders from a
local or NAS copy instead; symlinks rather than copies, since the pack is ~1 GB.


## Verified against live CARLA (2026-07-30)

`scripts/integration/ssv2_probe.py` drives the bridge over the real protocol and asserts
against the CARLA world. See that script's README.

Town01 loads from `lanelet2_map_path`; an empty path fails with a description rather than
running on the wrong map; an unknown town fails with `Map not found`. All **36** Town01
traffic lights report `is_frozen=True` after `Initialize`, and `UpdateTrafficLights` is
accepted.

Still unverified: whether a commanded state is *observable* on the right light. That needs
the signal mapping resolved against real CARLA positions, which needs rendering for the
Autoware side to matter.

Signal matching, live on Town01:

```
Found 36 traffic light element(s) in .../Town01/lanelet2_map.osm
Traffic light matching: 36 lanelet element(s), 36 CARLA light(s), 36 newly mapped by position (tolerance 5m)
CARLA traffic light cycling frozen; SSv2 is now the only writer.
```

**36 of 36 matched** — the 5 m tolerance, the Y-flip, the structural Lanelet2 parsing and the
OpenDRIVE sign-ID resolution are all correct together, which no unit test could establish.

Commanding every signal and reading the CARLA world back:

| commanded | observed |
|---|---|
| RED | 36 Red |
| GREEN | 36 Green |
| AMBER | 36 Yellow |

So the full chain works: lanelet way ID → position match → OpenDRIVE sign ID → CARLA actor →
`set_state`.

**Still open**: whether *Autoware* reacts to a commanded signal. That needs
`use_traffic_light_recognition=true` and a camera, so it needs rendering — gap 7 stays open.
