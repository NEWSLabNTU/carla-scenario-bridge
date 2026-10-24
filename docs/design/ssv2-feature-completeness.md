# SSv2 Feature Completeness and Lessons from the TIER IV Fork

What scenario_simulator_v2 (SSv2) can ask of a simulator backend, what this bridge delivers,
where the gaps are, and which mechanisms from TIER IV's `carla-autoware-native` fork transfer
to a client-API-only design.

**Status**: audit 2026-09-25, refreshed 2026-10-24. The audit's ranked list became
[roadmap 014](../roadmap/014-feature-completeness.md) and the gap phases 021-024; what is
left is in [Open gaps](#open-gaps-2026-10-24). The bridge is simulator-free since 046, so
the tables name what the bridge does and, where it differs, what each world server does
(`swb_carla_world`, `swb_isaac_world`). Code is cited by file, not line: the 2026-09-25 line
numbers referred to a bridge that no longer exists.

## Method

Four sources define "what SSv2 supports":

1. **Protocol slots**: every field of the 14 request types in `proto/simulation_api_schema.proto`.
   Verified identical to `src/scenario_simulator_v2/simulation/simulation_interface/proto/` (no drift).
2. **Function slots**: what `traffic_simulator` actually sends (`simulation/traffic_simulator/src/api/api.cpp:29-332`,
   `api.hpp:120-175`) and what the stock backend `simple_sensor_simulator` does with it.
3. **OpenSCENARIO support list**: `src/scenario_simulator_v2/docs/developer_guide/OpenSCENARIOSupport.md`.
4. **Ego contract**: `external/concealer` (what SSv2 expects of Autoware) and `simple_sensor_simulator`'s
   `EgoEntitySimulation` (what the stock backend supplies on Autoware's behalf).

AWSIM is referenced by SSv2 only as `simulator_type:=awsim` in `random_test_runner` and one line in
`ConfiguringPerceptionTopics.md` ("sensing and perception emulation is skipped"). There is no
in-tree list of which requests AWSIM implements; the AWSIM pattern this bridge follows is inferred
from that: puppeteer NPCs, reject sensor attaches, let the engine own ego physics.

## 1. Protocol slots

The backend surface is narrow: 14 requests, and per frame only `UpdateEntityStatus`,
`UpdateTrafficLights` (on change) and `UpdateFrame`. Every OpenSCENARIO action — speed, lane
change, routing, FollowTrajectory, AcquirePosition, behaviour trees, every Condition — is
resolved inside `traffic_simulator` and reaches the backend only as a finished pose plus
`action_status` (twist, accel, jerk, `current_action` string).

| Request | Field | Bridge (2026-10-24) | Stock backend | Gap |
|---|---|---|---|---|
| Initialize | `step_time`, `lanelet2_map_path` | honored | stored / activates map | – |
| | `realtime_factor`, `initialize_time`, `initialize_ros_time` | `realtime_factor` passed to the world server; times ignored | stored; ROS time stamps sensors | harmless: the vehicle bridges stamp from the simulator's frame time |
| UpdateFrame | `current_simulation_time` | sent with every `Step`, recorded in the gap trace (024) | drives sensor timing | – |
| UpdateStepTime | `simulation_step_time` | honored; ticks per frame from `carla_tick_seconds`, substeps applied (014, `ddf64d7`) | stored | – |
| Spawn* | `name`, `asset_key`, `pose`, `is_ego` | honored | | – |
| | `bounding_box` | origin offset (rear axle, 014); body chosen to fit it, residual reported (022) | carried into per-frame status | – |
| | `VehicleParameters.performance/axles` | NPCs: tracking limits; ego: the spec's physics (`sim_vehicle_spec`), checked against the catalog (2 cm) | feeds ego vehicle model | – |
| | second `is_ego` | rejected (one ego per scenario) | rejected | – (several Autoware egos are background AVs, or 039 on Isaac) |
| UpdateEntityStatus | `status[].pose`, `action_status` (non-ego) | tracked physics toward pose and twist (023), teleport under `npc_motion: kinematic` | stored | – |
| | `npc_logic_started` | honored: NPCs held still until true | ego model frozen until true | – |
| | `overwrite_ego_status` | honored | honored, **and** when ControlModeReport is MANUAL | MANUAL fallback dropped: the bridge has no ROS node (014) |
| | unknown entity name | rejected | throws | – |
| | ego reply | CARLA: the server's read-back with acceleration and jerk (`READS_BACK`); Isaac: the bridge differences velocity | model state | Isaac: jerk and angular acceleration 0; `current_action` empty on both |
| UpdateTrafficLights | `id`, `color` | honored via lanelet→OpenDRIVE (`<map>/carla/traffic_lights.yaml`), both simulators; freeze plus long phase times on CARLA | passed to pseudo detector | – |
| | `shape` (arrows), `status` FLASHING, `relation_ids`, `confidence` | flattened / lit / ignored / ignored | full | CARLA has no arrow bulbs; protected turns and flashing lost |
| Attach{Lidar,Detection,OccupancyGrid,Imu,PseudoTrafficLightDetector} | all fields | **rejected**, `Success=false` | publishes the topic | by design: the vehicle bridges publish real sensors. Detection knobs: §3 |
| UpdateEnvironment (fork, 047) | weather, time of day, road condition | honored on both simulators | – (not upstream) | `DomeImage` not sent |

## 2. OpenSCENARIO features

| Feature | SSv2 | Reaches backend? | Bridge / world servers (2026-10-24) |
|---|---|---|---|
| Entity motion (all actions) | yes/partial | pose per frame | tracked physics toward the pose (023): CARLA tracks on the server (`TRACKS_TARGETS`), Isaac from the bridge's velocity (`TRACKED_MOTION`); teleport under `npc_motion: kinematic` |
| Vehicle `model3d`, EntitySubtype (TRUCK, BUS, BICYCLE…) | yes | `asset_key`, subtype, bbox | catalogue body by `model3d`, else subtype and size (022); Isaac from the exported CARLA set |
| Pedestrian | partial | pose per frame | CARLA: `WalkerControl`, walk cycle; Isaac: 52 exported walkers, gait from speed (044; not all seen rendered) |
| MiscObject | partial | `asset_key`, bbox | `static.prop.*` by size (022) |
| TrafficSignalStateAction / Controller | 1.3 | UpdateTrafficLights | §1; Autoware gets the state from the vehicle bridge (015) |
| Environment / Weather / Fog / Sun / TimeOfDay / RoadCondition | parse-only upstream; fork (047) sends `UpdateEnvironment` | yes (fork) | road friction and wetness on both, zones on Isaac; weather and time of day on both (Isaac: sun, fog for cameras, rain for the lidar, snow on soil (052); CARLA: all but snow, cameras only) (`off-road-surfaces.md`) |
| LightStateAction / VehicleLight | unimplemented in SSv2 | no | – (CARLA `set_light_state` exists) |
| AnimationAction / Appearance / Visibility | unimplemented in SSv2 | no | – |
| Override*Action, ActivateController | unimplemented in SSv2 | no | n/a: the ego is Autoware; a takeover is acb's `allow_manual` |
| Trailer | unimplemented in SSv2 | TRAILER subtype only | – |
| CollisionCondition | partial | no: SSv2 checks its own boxes | world servers report contacts (`CONTACTS`); `physical_verdict` joins them to SSv2's verdict (021) and the gap oracle compares the two (024) |
| Sensors | yes | Attach* | rejected; acb / aib own the sensors |

Each unimplemented row is SSv2's gap, not the backend's: making one reach a simulator
needs a fork protocol slot, as `UpdateEnvironment` was for weather (047).

**Two computations run twice, unreconciled:**

- **Collision.** SSv2 judges collisions from its declared bboxes and canonical poses. CARLA
  resolves the PhysX ego against the NPCs' bodies (tracked physics by default since 023;
  kinematic colliders under `npc_motion: kinematic`, where `set_simulate_physics(false)` keeps
  colliders active). The ego can be shoved by an NPC that SSv2 never registers as a
  collision, or SSv2 can report one that CARLA never rendered. **Since 021/024** the world
  servers report contacts, `physical_verdict` joins them to SSv2's verdict, and the gap
  oracle's G1-G4 compare the two geometries on every traced run.
- **Pose reference point.** Scenario vehicles declare `BoundingBox/Center x="1.5"` (e.g.
  `src/swb_examples/scenarios/multi_av/town01_two_av.xosc:13`): the SSv2 entity origin is the rear axle. A CARLA vehicle's
  origin is near its bbox centre. `ros_pose_to_carla_transform` (C:2399) and
  `coordinate_conversion.rs` apply no offset. **Measured 2026-09-25** (`scripts/pose_offset_probe.py`):
  the bridge maps the SSv2 pose to the CARLA actor origin 1:1 (Δ 0.000 m on spawn, teleport and
  ego readback), and on `vehicle.tesla.model3` the rear axle is 1.386 m behind that origin
  (wheelbase 3.005 m). So NPC meshes sit 1.39 m behind their SSv2 model and SSv2 takes the ego's
  centre for its rear axle. For two same-lane vehicles the real bumper gap is ≈2.9 m larger than
  SSv2's computed gap, so every distance-based condition so far has been conservative by that
  much. The same offset exists on the Autoware side: acb publishes `base_link`
  at the CARLA actor origin (`acb_bridge/src/autoware.rs:608-613`), where Autoware assumes the
  rear axle. Decision 2026-09-25: rear axle everywhere ([roadmap 014](../roadmap/014-feature-completeness.md)).
- **Lane position.** SSv2 re-canonicalises the ego pose from CARLA onto lanelet2; CARLA works
  in OpenDRIVE. Any misregistration between the two maps appears as lane-matching failures.
  **Still unmeasured** (2026-10-24).

## 3. Ego / Autoware contract

Updated 2026-10-09 (roadmap 017). The concealer's ADAPI role (initialize, route, engage,
RTC, velocity limit, emergency and MRM monitoring) is now served by `scenario_agent_relay`
in SSv2's domain and carried out by the ego's vehicle agent in its own; the ego data path
below is unchanged. `managed_ego:=false` is retired.

| SSv2 / Autoware expects | Provider (2026-10-24) | Gap |
|---|---|---|
| `/vehicle/status/{velocity,steering,gear,control_mode,turn_indicators,hazard_lights}` | acb / aib; `control_mode` reports the real mode | – |
| `/control/control_mode_request` service | acb: AUTONOMOUS succeeds; MANUAL fails as stock, or with `allow_manual:=true` succeeds and the driver takes over until AUTONOMOUS | – |
| `control_cmd`, `gear_cmd`, `turn_indicators_cmd`, `hazard_lights_cmd`, `emergency_cmd` | acb / aib apply all | – |
| `/localization/kinematic_state`, `/localization/acceleration` | GNSS→NDT→EKF by default | – |
| `/clock` | the vehicle bridge, from the simulator's frame time, in every vehicle domain (015) | – |
| Traffic-light recognition topic | the vehicle bridge, from the simulator's lights (015, 017) | – |
| Detection-sensor scenario knobs (`isClairvoyant`, `detectedObject*Delay/StdDev/MissingProbability`, `randomSeed`) | **unsupported by decision** (014): Attach rejected, perception runs on real sensors | scenarios that tune perception faults have no effect |
| Ego velocity limit, emergency / MRM / `/autoware/state` monitoring | relay ↔ agent (017) | – |

## 4. Lessons from `tier4/carla-autoware-native`

Their design compiles the adapter into the engine; the mechanisms below are the ones that
survive translation to the client API. Sources: `autoware-support` branch and feature branches;
SSv2 #1848/#1849; autoware_universe #13276, #13286, #13305, #13406, #13408, #13410, #13319.
The "Transfer" column is the 2026-09-25 assessment; every "Yes" was carried out in 014.

| Mechanism | Their finding | Transfer |
|---|---|---|
| **Steering root cause** | A 26-point desired→actual LUT (`AutowareSteeringCompensation.h`) was a symptom fix, later reverted: Chaos squares steer input, so command must be `sign·sqrt(|steer|)`. They also flatten the speed-dependent `steering_curve` to 1.0 at ego registration | **Yes, acb.** 0.9.16 exposes `VehiclePhysicsControl.steering_curve` and per-wheel `max_steer_angle`; acb uses geometry only (`vehicle_control.rs:54-63`), curve not flattened. Measure command vs `get_wheel_steer_angle` first; flatten curve at spawn; keep LUT as optional config, not default |
| **Input shaping** (SSv2 #1849) | steer rate limit 20 °/s, steer lag τ=0.2 s, jerk +4/−6 m/s³, accel lag τ=0.2 s, per-axle friction | **Yes, acb**, before `apply_control`. Pure math; friction = `WheelPhysicsControl.tire_friction` once at spawn |
| **Longitudinal control** | Server-side kinematic velocity override (`forward_speed += a·dt`, `SetPhysicsLinearVelocity`), exact by construction | **No.** Client equivalent `set_target_velocity` per tick kills tyre dynamics. Don't compare their tracking numbers to ours |
| **Vehicle status** | Six `/vehicle/status/*` topics from one post-physics hook, same stamp, base_link at rear axle; steering reported as commanded×max (0.10 getter returns 0) | Already ours. Keep measured wheel angle (works on 0.9.x); stamp from the tick's sim time |
| **`tick_follower`** (#13286) | Never `world.tick()`; `on_tick` callback → queue, drain to newest, warn on skips. `wait_for_tick` loses frames that complete while busy. Bridge must start before the ticker (`load_world` ticks) | **Yes, acb.** Matches the single-ticker invariant; acb currently uses `wait_for_tick_or_timeout` (`main.rs:461,586`), which can drop frames under load and is a candidate cause of the 10 Hz LiDAR at two stacks |
| **Attach to existing ego** (#13410) | find by `role_name` until timeout, never destroy | Already ours |
| **`max_substep_delta_time`** (#13406, tested 0.9.16) | default 0.01 s gives phantom yaw rate up to 0.9 °/s in IMU / `get_angular_velocity`; 0.002 s with `max_substeps = ceil(dt/0.002)` → 0.003 °/s | **Yes, bridge, cheap.** Bridge sets neither today. Probably part of the ego acceleration jitter that C:19-28 masks with smoothing |
| **Sensor stamping** (#13408) | key by capture `frame`, hold later frames; keying by sensor id drops IMU samples | Check acb `sensor_bridge.rs` |
| **V2I publisher** (AWSIM port) | `TrafficLightGroupArray` on `/v2x/traffic_signals`, group id = lanelet relation id, 6-step predictions; **must not** run alongside SSv2's own V2I | Not for publishing (SSv2 owns that topic when V2I is configured). The RPC-cost lesson transfers: per-actor state RPC is 7–10 ms; read one snapshot per tick |
| **Traffic-light freeze hole** | `freeze_all_traffic_lights` skipped dynamically placed groups; they overrode `set_state` within ~22 s. Workaround: `set_{green,yellow,red}_time(99999)` | **Yes, belt-and-braces** in `UpdateTrafficLights` |
| **Lanelet ID channel** | lights carry `lanelet2_id` actor attribute; `get_opendrive_id()` = way id | Cleaner than geometry matching if maps are authored that way; ours match by position |
| **MGRS / map transform** | Odaiba needed an 11-constant affine (scale 1.000111, two yaw corrections) between Autoware map and CARLA world, not a pure translation | Expect the same when leaving Town01 |
| **gRPC scenario handshake** (#13319) | Two RPCs: `GetMission` (pose+goal, pulled), `ReportReadiness` | Analogous to our pilot; nothing to borrow |
| **CycloneDDS, ros-domain-id** | engine-side DDS backend selection; topic names sanitised | Not applicable: acb is a plain rclrs node, RMW and domain are ROS config |

## Ranked gaps (2026-09-25), and where they went

| # | Gap | Outcome |
|---|---|---|
| 1 | Traffic-light state never reaches Autoware | done: the vehicle bridge publishes it from the simulator (015); SSv2 fork publishes conventional signals |
| 2 | Pose reference point | done: rear axle everywhere (014) |
| 3 | Collision truth split | done: contacts and the physical verdict (021), compared by the oracle (024) |
| 4 | `UpdateStepTime` ignores substeps | done (014) |
| 5 | NPC actors carry zero velocity | done: tracked physics (023) |
| 6 | `max_substep_delta_time` unset | done (014) |
| 7 | acb a `wait_for_tick` follower | done: tick-driven (014) |
| 8 | Steering curve, input shaping | done: curve compensated, rate/lag limits optional (014) |
| 9 | Blueprint mapping thin | done: by `model3d`, subtype and size; props (022) |
| 10 | Pedestrian z | done (014) |
| 11 | Detection-sensor knobs | unsupported by decision (014) |
| 12 | `control_mode_request` absent | done, plus `allow_manual` takeovers |
| 13 | Protocol strictness | done, except the MANUAL overwrite (no ROS node in the bridge) |
| 14 | Arrows, flashing, `relation_ids` | open: CARLA has no arrow bulbs |
| 15 | Weather / TimeOfDay / RoadCondition | done in the fork (047) |
| 16 | Traffic-light freeze hole | done (014) |

## Open gaps (2026-10-24)

**SSv2-side** (each needs SSv2 or a fork slot first; no backend can fix them alone):
LightStateAction, AnimationAction / Appearance / Visibility, Override* / ActivateController,
Trailer; detection-sensor knobs (unsupported by decision); `DomeImage` (fork does not send it).

**Backend, CARLA**: arrows / flashing / `relation_ids` (CARLA limit); lanelet2 vs OpenDRIVE
registration unmeasured; `current_action` empty in replies; CARLA free-runs at ~79 % GPU
while idle (044 open item 5).

**Backend, Isaac against CARLA**: no `AUTOPILOT` (simulator traffic; `town01_simulator_autopilot`
deferred), no `AGENT_VEHICLES` (background AVs), no `READS_BACK` (jerk and angular
acceleration 0 in the ego reply), no `WEATHER` presets; several Autoware egos (039); a lead
NPC on soil sinking and riding in ruts (052); first-render material compile moved into
`Spawn` (044, unverified live); the 52 walkers not all seen rendered.

**Both**: CenterPoint labels the crossing walker `unknown`, and `town01_pedestrian` passes in
SSv2 while the ego touches the walker (044 open item 4); unknown disagreements are the gap
campaign's search ([025](../roadmap/025-adaptive-generator-and-exploit-pipeline.md)), and
SSv2's `random_test_runner` on this backend is its step 2.

## What not to borrow

- Engine patches. Everything above is reachable through the client API on 0.9.16.
- Server-side velocity override for the ego. It reports perfect tracking because it bypasses
  the physics the scenario is supposed to exercise.
- Publishing `/v2x/traffic_signals` from the bridge when SSv2's V2I is configured. Two writers
  on the same topic; SSv2's own docs warn against it.
- The steering LUT as a default. It was a symptom fix, and they reverted it.
