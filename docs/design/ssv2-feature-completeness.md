# SSv2 Feature Completeness and Lessons from the TIER IV Fork

What scenario_simulator_v2 (SSv2) can ask of a simulator backend, what this bridge delivers,
where the gaps are, and which mechanisms from TIER IV's `carla-autoware-native` fork transfer
to a client-API-only design.

**Status**: audit, 2026-09-25. Nothing here is implemented; the ranked gap list at the end is
input to the next roadmap entry. Line numbers refer to `src/carla_scenario_bridge/src/coordinator.rs`
(`C:`), SSv2 `463c02946` and acb `7e899ab` unless stated.

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

| Request | Field | Bridge | Stock backend | Gap |
|---|---|---|---|---|
| Initialize | `step_time`, `lanelet2_map_path` | honored (C:1171, C:1230) | stored / activates map | – |
| | `realtime_factor`, `initialize_time`, `initialize_ros_time` | ignored | stored; ROS time stamps sensors | harmless: bridge publishes no stamped topics |
| UpdateFrame | `current_simulation_time` | ignored | drives sensor timing | no drift check against CARLA elapsed time |
| UpdateStepTime | `simulation_step_time` | **partial** | stored | **bug**: sync-mode branch sets `fixed_delta_seconds = step_time` (C:1504); `enable_sync_mode` uses `step_time / substeps` (C:1417). With `substeps: 2` a mid-run change runs CARLA 2× fast. Latent: SSv2 sends it once before sync mode (`api.cpp:47`), so the async branch stores it |
| Spawn* | `name`, `asset_key`, `pose`, `is_ego` | honored | | – |
| | `bounding_box` | **ignored** (no reads in bridge) | carried into per-frame status | see §2 pose offset |
| | `VehicleParameters.performance/axles` | ignored | feeds ego vehicle model | CARLA blueprint physics instead; acceptable |
| | second `is_ego` | accepted | rejected | should reject |
| UpdateEntityStatus | `status[].pose` (non-ego) | `set_transform`, physics off | stored | – |
| | `status[].action_status` (twist/accel) | ignored, echoed back | stored | CARLA actor velocity stays 0: radar Doppler, wheel spin, walker animation all dead |
| | `npc_logic_started` | ignored | ego model frozen until true | ego under PhysX from spawn (handbrake-parked) |
| | `overwrite_ego_status` | honored (C:1943) | honored, **and** when ControlModeReport is MANUAL | MANUAL fallback missing |
| | unknown entity name | echoed back (C:1911-1939) | throws | weaker than stock |
| | ego reply | pose, twist, median-smoothed accel, differenced jerk (C:2286-2385) | model state | angular accel always 0; `current_action` empty |
| UpdateTrafficLights | `id`, `color` | honored via lanelet→OpenDRIVE map, 36/36 on Town01 | passed to pseudo detector | unmapped ids warned once, then silent |
| | `shape` (arrows), `status` FLASHING, `relation_ids`, `confidence` | flattened / lit / ignored / ignored | full | protected turns and flashing lost |
| Attach{Lidar,Detection,OccupancyGrid,Imu,PseudoTrafficLightDetector} | all fields | **rejected**, `Success=false` (C:2005-2048) | publishes the topic | interpreter ignores the bool (`simulator_core.hpp:365-443`), so safe. Consequences in §3 |

## 2. OpenSCENARIO features

| Feature | SSv2 | Reaches backend? | Bridge | CARLA 0.9.16 via carla-rust |
|---|---|---|---|---|
| Entity motion (all actions) | yes/partial | pose per frame | teleport | `set_transform`, `set_target_velocity` |
| TeleportAction, AcquirePosition, LaneChange, FollowTrajectory | yes/partial | pose per frame | teleport | – |
| Vehicle spawn `model3d` | yes | `asset_key` | `blueprint_map` has only `sample_vehicle`; else `vehicle.tesla.model3` (C:98, C:1559-1568) | blueprint library, `Vehicle::bounding_box` |
| EntitySubtype (TRUCK, BUS, BICYCLE…) | yes | only in EntityStatus | ignored | could select blueprint |
| Pedestrian | partial | pose per frame | `walker.pedestrian.0001`, kinematic, no controller (C:1764-1788) | `WalkerControl`, `set_bones`, AI controller |
| MiscObject | partial | `asset_key`, bbox | `static.prop.streetbarrier` default (C:102) | `static.prop.*` |
| TrafficSignalStateAction / Controller | 1.3 | UpdateTrafficLights | partial (§1) | `TrafficLight::set_state`, freeze |
| Environment / Weather / Fog / Sun / TimeOfDay / RoadCondition | listed "1.3", **parse-only no-op** (`environment_action.cpp` `start()`/`run()` are `LINE()`) | **no slot** | – | `World::set_weather`, `tire_friction` |
| LightStateAction / VehicleLight | unimplemented | no | – | `Vehicle::set_light_state` |
| AnimationAction / Appearance / Visibility | unimplemented | no | – | walker bones |
| Override*Action, ActivateController | unimplemented | no | n/a (ego is Autoware) | `apply_control` |
| Trailer | unimplemented | TRAILER subtype only | – | trailer blueprints |
| CollisionCondition | partial | **no**: SSv2 does its own 2D bbox check (`api.cpp:345`) | not reported | `sensor.other.collision` |
| Sensors | yes | Attach* | rejected | acb owns real sensors |

**Two computations run twice, unreconciled:**

- **Collision.** SSv2 judges collisions from its declared bboxes and canonical poses. CARLA
  resolves the PhysX ego against kinematic NPC colliders (`set_simulate_physics(false)` keeps
  colliders active). The ego can be shoved by a teleported NPC that SSv2 never registers as a
  collision, or SSv2 can report one that CARLA never rendered. Nothing compares the two.
- **Pose reference point.** Scenario vehicles declare `BoundingBox/Center x="1.5"` (e.g.
  `src/csb_examples/scenarios/multi_av/town01_two_av.xosc:13`): the SSv2 entity origin is the rear axle. A CARLA vehicle's
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

## 3. Ego / Autoware contract

Updated 2026-10-09 (roadmap 017). The concealer's ADAPI role (initialize, route, engage,
RTC, velocity limit, emergency and MRM monitoring) is now served by `scenario_agent_relay`
in SSv2's domain and carried out by the ego's vehicle agent in its own; the ego data path
below is unchanged. `managed_ego:=false` is retired.

| SSv2 / Autoware expects | Provider today | Gap |
|---|---|---|
| `/vehicle/status/{velocity,steering,gear,control_mode,turn_indicators,hazard_lights}` | acb `vehicle_control.rs:382-399`, ~20 Hz; measured single publisher | control_mode hard-coded AUTONOMOUS |
| `/control/control_mode_request` service (stock `autoware_universe.cpp:52`) | **nobody** | manual↔autonomous transitions untestable |
| `control_cmd`, `gear_cmd`, `turn_indicators_cmd`, `hazard_lights_cmd`, `emergency_cmd` | acb applies all | – |
| `/localization/kinematic_state` | GNSS→NDT→EKF by default; direct publish behind `publish_direct_localization` | – |
| `/localization/acceleration` | EKF only | – |
| `/clock` | acb, from CARLA's frame time, in every vehicle domain (015) | – |
| Traffic-light recognition topic (`/perception/traffic_light_recognition/external/traffic_signals`) | acb, from CARLA's lights via the map dir's `carla/traffic_lights.yaml` (015, 017); camera recognition off (`fusion_only`) | – |
| Detection-sensor scenario knobs (`isClairvoyant`, `detectedObject*Delay/StdDev/MissingProbability`, `randomSeed`) | dead: Attach rejected, acb ground-truth objects optional and un-noised | scenarios that tune perception faults have no effect |
| Ego velocity limit (`/api/autoware/set/velocity_limit`) | relay → agent → Autoware (017) | – |
| Emergency / MRM / `/autoware/state` monitoring | agent `fault` → relay → concealer (017) | – |

## 4. Lessons from `tier4/carla-autoware-native`

Their design compiles the adapter into the engine; the mechanisms below are the ones that
survive translation to the client API. Sources: `autoware-support` branch and feature branches;
SSv2 #1848/#1849; autoware_universe #13276, #13286, #13305, #13406, #13408, #13410, #13319.

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

## Ranked gaps

Ordered by effect on scenario verdicts, then by cost.

| # | Gap | Effect | Fix sketch | Cost |
|---|---|---|---|---|
| 1 | Traffic-light state never reaches Autoware | every signal scenario fails or passes by accident | implement the 009 decision: bridge publishes `TrafficLightGroupArray` from `UpdateTrafficLights`, keyed by lanelet ID, flag default off | M |
| 2 | Pose reference-point offset (rear axle vs centre) | all distance conditions off by ~1.5 m, both directions | measure first (spawn NPC at known pose, read CARLA transform); then apply `bounding_box.center` offset in both conversions | S measure, S fix |
| 3 | Collision truth split | ego shoved by NPC colliders; SSv2 collision ≠ CARLA collision | attach `sensor.other.collision` to ego, log mismatches; decide policy (disable NPC↔ego collision, or report to SSv2 via CollisionCondition) | M |
| 4 | `UpdateStepTime` ignores `substeps` (C:1504) | latent 2× time-rate bug | use `substep_delta()` | XS |
| 5 | NPC actors carry zero velocity | dead radar Doppler, wheels, walker animation; acb ground-truth velocity wrong if read from CARLA | `set_target_velocity` from `action_status.twist`; `WalkerControl` speed for walkers | S |
| 6 | `max_substep_delta_time` unset | phantom yaw rate, accel jitter | set 0.002 s and `max_substeps` in world settings | XS |
| 7 | acb is a `wait_for_tick` follower | frame loss under load (10 Hz LiDAR at two stacks) | `on_tick` queue, drain to newest, warn on skips | S |
| 8 | Steering curve not flattened; no input shaping | lateral tracking error; unmeasured | measure, flatten `steering_curve`, add rate/lag limits in acb config | S |
| 9 | Blueprint mapping thin; bbox ignored | wrong vehicle size for perception; trucks become Teslas | map by EntitySubtype or bbox dimensions; MiscObject props | S |
| 10 | Pedestrian z after first update | walkers may sink (capsule origin ~0.9 m) | verify; apply walker origin offset per frame | XS |
| 11 | Detection-sensor knobs dead | perception-fault scenarios inert | either accept `AttachDetectionSensor` and forward noise/delay params to acb ground-truth publisher, or document as unsupported | M |
| 12 | `/control/control_mode_request` absent | manual-override tests impossible | small service in acb | XS |
| 13 | `npc_logic_started`, MANUAL overwrite, second ego, unknown entity | protocol strictness | reject/handle per stock semantics | XS each |
| 14 | Arrows, flashing, `relation_ids` | protected-turn scenarios wrong | CARLA has no arrow bulbs; document; publish full state on the V2X path from #1 | – |
| 15 | Weather / TimeOfDay / RoadCondition | no slot in protocol; SSv2 itself no-ops | bridge-side `<Environment>` parse from `.xosc` or config key, applied at Initialize | S, opt-in |
| 16 | Traffic-light freeze hole | lights may resume cycling | set long phase times as backup | XS |

Items 4, 6, 10, 12, 13, 16 are small enough to batch into one hardening entry. Items 1–3 each
need a measurement before a fix and should be their own roadmap entries. Item 15 is a protocol
gap on SSv2's side; it belongs in the upstreaming discussion (roadmap 005), not in code first.

## What not to borrow

- Engine patches. Everything above is reachable through the client API on 0.9.16.
- Server-side velocity override for the ego. It reports perfect tracking because it bypasses
  the physics the scenario is supposed to exercise.
- Publishing `/v2x/traffic_signals` from the bridge when SSv2's V2I is configured. Two writers
  on the same topic; SSv2's own docs warn against it.
- The steering LUT as a default. It was a symptom fix, and they reverted it.
