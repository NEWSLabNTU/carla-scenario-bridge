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
geometry regenerated from CARLA's own `TrafficLight` actors or the ground-truth injection
path this work item allows for.

#### Geometry regenerated from CARLA (2026-08-16)

`scripts/regenerate_light_geometry.py` takes the first option. CARLA's
`TrafficLight.get_light_boxes()` returns the signal head in world coordinates -- centre,
extent and rotation -- so width, height and mounting position are measured rather than
guessed. Facing is *not* taken from CARLA: it comes from the map, as the travel direction
of the lanelet that references the regulatory element, since that is the definition
Autoware checks against. Travel direction is derived as centroid -> stop line rather than
from boundary point order, which this pack does not store consistently.

The contract, read out of `autoware_traffic_light_map_based_detector` and
`traffic_light_utils` rather than assumed:

| linestring | meaning |
|---|---|
| `front` | bottom-left corner of the head, as the driver sees it |
| `back` | bottom-right corner |
| way tag `height` | how far the head extends above those two points |
| `atan2(back - front) + 90 deg` | the direction the head faces; must be within `car_traffic_light_max_angle_range` of the camera's view direction |

Town01 after regeneration: 36 of 36 lights, heads 0.451 m wide and 1.221 m tall with
their bottoms at 2.00 m, and facings spread across the compass (0, ±90, ±180, each within
about 10 degrees) instead of all 36 claiming 135. The test signal 43763 now reads bar yaw
90 deg, so it faces 180 deg -- exactly the westbound approach that drives at it.

A fifth wiring defect surfaced immediately after: acb stamped its camera topics with the
*mounting* frame, `camera6/camera_link`. Everything that projects a 3D point into an
image assumes the REP-103 optical convention -- z forward, x right, y down -- and reads
that frame off the header, so the detector's forward ray pointed at the sky and every
projected point landed behind the image plane. Fixed in acb; the header now carries
`camera6/camera_optical_link`.

**With both fixes the projection works.** Across a full approach the map-based detector
projects the signal and the fine detector finds it in the image, continuously from 195 m
out to 7 m out:

```
t+ 20s ego( 297.4, -55.4)  3.8 m/s  expect=1 rois=1   <- 195 m from the signal
t+ 53s ego( 173.3, -55.4)  3.8 m/s  expect=1 rois=1
t+ 67s ego( 119.1, -55.4)  3.8 m/s  expect=1 rois=1
t+ 71s ego( 107.9, -55.4)  1.7 m/s  expect=1 rois=1   <- braking for the stop line
t+ 73s ego( 106.4, -55.4)  0.0 m/s  expect=0 rois=0   <- head now above the camera's FOV
```

That is the stage which was dead before, and it stays alive for the whole 190 m.

**Still open: classification.** `.../camera6/classification/traffic_signals` never
publishes, and the fused `judged/traffic_signals` publishes empty groups, so no colour
reaches planning and the two acceptance criteria stay unticked. Two distinct causes, one
fixed:

**Fixed -- the model directory has to be writable.** The classifier never loaded at all.
Running it standalone shows why, and it is not what the composable-node loader reports:

```
[I] [TRT] Engine generation completed in 32.9556 seconds.
[E] [TRT] Fail to open engine file
[ERROR] Component constructor threw an exception: Failed to setup TensorRT engine
```

TensorRT builds the engine from the ONNX and then writes it *next to that file*. acb
pointed `data_path` at the packaged `/opt/autoware/1.5.0/data`, which is root-owned, so
the build succeeded and the write failed, the constructor threw, and the node never came
up. Nothing downstream said a word: the rest of the pipeline runs and publishes empty
results forever. Upstream's own default for `data_path` is `$HOME/autoware_data` for
exactly this reason; acb now matches it, and `scripts/link_autoware_data.sh` mirrors the
packaged tree there as real directories full of symlinks so engines are cached in a
writable place instead of being rebuilt (33 s each) every launch. With that, both
classifiers and the fine detector load and appear in the graph.

**Fixed -- play_launch killed them for taking too long to construct.** play_launch forks a
`component_node` process per composable node and waits on a ready pipe. That wait was a
fixed 30 s, after which it sends SIGKILL
(`play_launch_container/src/clone_isolated_component_manager.cpp`):

```cpp
constexpr int kReadyTimeoutMs = 30000;  // 30s matches LoadNode service timeout
...
if (!got_response || ready_buf.empty()) {
  kill(child_pid, SIGKILL);
  throw std::runtime_error("component_node did not respond (timeout or crash)");
}
```

A constructor runs before the node can answer, and these constructors are slow: **33 s**
to build the TensorRT engine on a cold cache, and **45 s** to construct even with the
engine cached. So the child was killed seconds short of reporting ready, the LoadNode
retry forked a fresh child that died identically, and all three inference nodes never
existed. The 30 s also undercut play_launch's own `LoadTimings`, whose 60 s retry and
600 s total budget name TensorRT loads as their reason but never get the chance to apply.

Fixed in play_launch_container `480f5fc`: the budget is now
`PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS`, default unchanged at 30 s. csb's launch recipes
export 180000. With that, both classifiers and the fine detector stay up for the whole
run, and the fused `traffic_signals` topic finally carries the commanded signal:

```
t+23s ego(300.5,-55.4) 3.9 m/s expect=1 rois=1 recognised[43856:UNKNOWN]
t+61s ego(153.2,-55.4) 3.8 m/s expect=1 rois=1 recognised[43856:UNKNOWN]
```

Regulatory element 43856 is the one the scenario commands. The whole chain now runs:
map -> projection -> ROI -> classifier -> fusion -> planning.

**Open: the colour is UNKNOWN, and the pictures say why.**
`scripts/roi_capture.py` pairs each `detection/rois` message with the camera frame it
refers to and writes the region the classifier is handed. The signal *is* in there and is
unmistakably red to a human eye -- top bulb lit, lower two dark -- but it occupies only
about a third of the crop, off to one side, with a palm tree and a fence taking the rest.
The classifier resizes exactly that region to 224x224, so most of its input is scenery.

The region is not mis-aimed, it is inflated, and by a knowable amount.
`traffic_light_map_based_detector` pads its projection by a vibration margin of
`sin(max_vibration_yaw/2) * depth + max_vibration_width/2` on each side. Those defaults
(0.01745 rad, 0.5 m) are sized for a real vehicle's mount against a roughly 1 m signal
head. Against CARLA's 0.451 m head:

| depth | ROI width | head's share of the crop |
|---|---|---|
| 10 m | 1.13 m (2.5x the head) | 40% |
| 30 m | 1.47 m (3.3x) | 31% |
| 100 m | 2.70 m (6.0x) | 17% |
| 190 m | 4.27 m (9.5x) | 11% |

31% at 30 m is exactly what the captured crops show.

The obvious fix is `use_high_accuracy_detection`, whose whole job is to find the real box
inside that padding -- and **it makes things worse**. Turning it on rewires
`detection/rois` to the fine detector's refined output, and that node does not stay up:
the topic went from 529 non-empty messages in a run to zero, and the chain produced
nothing at all. It is back off, with the reason recorded in the launch file. Two ways
forward, neither tried: work out why the fine detector exits (the classifiers, loaded the
same way, now survive), or shrink the vibration margins, which live in
`<namespace>_traffic_light_map_based_detector.param.yaml` under autoware_launch's share
and are neither writable nor overridable from acb's include -- the same wall the
per-camera parameter file presented earlier.

**Daylight was tried and is not the answer.** CARLA's default on this map is cloudiness
60, precipitation 40 and sun altitude 20 degrees -- overcast and raining in every frame
of every run so far, which is not what a classifier trained on real daylight footage
expects. `scripts/set_weather.py` sets a preset; ClearNoon takes it to cloudiness 5,
precipitation 0, sun altitude 45.

The captured regions improve enormously: bright, sharp, the head well framed with its top
bulb plainly lit. Classification did not change. Keep the weather setting anyway -- there
is no reason to test perception against a wet dusk by accident -- but the colour is not
being lost to the lighting.

#### The classifier was never silent, and it does read the light

Two earlier readings in this document were wrong, both from probes rather than from the
stack, and the correction is the actual finding.

**"The classifier topic goes silent between runs" was a measurement artefact.** Counting
messages directly over a full run:

```
rois=1868  non-empty=529
classifier debug images (it ran this many times): 516
classifier output messages: 1826, of which non-empty: 516
```

It fires on essentially every region it is given -- 516 of 529 -- and publishes
throughout. Nothing goes silent.

**"Colour 18 is outside the enum" was the wrong enum.** The per-camera classifier
publishes `tier4_perception_msgs/TrafficLightElement`, and the *installed* definition is a
single flat numbering shared by colour, shape and status in which **UNKNOWN is 18**:

```
RED=1 AMBER=2 GREEN=3 WHITE=4 CIRCLE=5 LEFT_ARROW=6 ... FLASHING=17 UNKNOWN=18
```

`autoware_perception_msgs`, which the fused topics use, numbers UNKNOWN as 0 instead.
Decoding one with the other's table turns every ordinary unknown into an apparent
corruption. `scripts/classifier_probe.py` and `traffic_light_probe.py` now carry both
tables.

Decoded properly, one run reads:

| classifier output | count |
|---|---|
| RED circle | 13 |
| AMBER circle | 23 |
| GREEN arrow | 4 |
| UNKNOWN | 480 |

So the chain is complete and the classifier genuinely recognises CARLA's signal -- about
7% of the time. The rest is UNKNOWN, which is what the fusion stage forwards, which is
why planning has no state to obey. Note also that a commanded **red** comes back as AMBER
more often than RED: CARLA renders its red bulb bright orange, and the classifier is not
wrong to hesitate.

That makes the remaining criteria a question of recognition *rate*, not of plumbing.

#### Shrinking the vibration margins makes it worse, not better

The region handed to the classifier is 2.5-9.5x wider than the signal head, entirely
because of `traffic_light_map_based_detector`'s vibration padding, so cutting that padding
looks like free accuracy. It was tried and the result is unambiguous:

| margins | non-empty regions | non-empty classifier outputs |
|---|---|---|
| upstream (yaw 0.01745, width 0.5) | 529 | **516** |
| shrunk (yaw 0.001, width 0.05) | 534 | **0** |

Zero. The padding is load-bearing. A 0.451 m head subtends about 13 px at 30 m and 4 px
at 100 m, so with the margins gone the region collapses to the head's true size and there
is nothing left to resize into the classifier's 224x224 input; 4x7 px crops were captured.
What looked like the classifier wasting its input on scenery is closer to the padding
being the only reason the crop is large enough to read at all.

The margins are back at upstream's defaults, with this recorded in the launch file so the
idea is not retried from scratch. The lever that remains is the head's *apparent* size --
a longer focal length on the traffic light camera, or a scenario that brings the ego
closer before it needs the answer -- not the padding. Note the ego already stops 4 m short
of the stop line, where the head is above the camera's field of view entirely, so
"closer" has a hard limit here.

The other lever is unchanged: a commanded red renders bright orange in CARLA and comes
back as AMBER more often than RED.

Three traps cost time here and are worth repeating. `ros2 node list` without `--no-daemon`
reports nodes that are already gone. `pgrep -f classifier` matches the shell running the
pgrep, so zero surviving processes read as one. And a composable node that dies in its
constructor is reported by the loader only as a missing LOADED event -- run it under
`ros2 component standalone` to see the real exception.

One practical note for whoever picks this up: several ego stacks in a row came up
degraded during this work -- 88/89 composable nodes, or localization never publishing, or
the ego routed but never engaging. Restarting the stack cleared it every time. Check that
the run under test actually drove before reading anything into a silent pipeline.

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
- [ ] Autoware's traffic light recognition reports the state SSv2 commanded — detection
      now works end to end (the signal is projected from the map and found in the image
      for a whole 190 m approach); **classification is the last silent hop**. See gap 7
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
