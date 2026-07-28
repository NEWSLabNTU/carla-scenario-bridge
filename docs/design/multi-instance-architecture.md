# Multi-Instance Architecture

How CARLA, `csb_bridge`, `acb_bridge`, SSv2 and N Autoware instances divide responsibility
over a single shared CARLA world.

This document supersedes the parts of [architecture.md](architecture.md) that assume a
single Autoware instance and a single ROS domain. Where the two disagree, this document wins.

**Status**: design agreed 2026-07-28. Not yet implemented — see [Gaps](#gaps-between-this-design-and-the-code).

## Scope

One SSv2 instance drives one scenario ego. Additional Autoware instances run as *background
AVs*: real Autoware stacks driving real CARLA vehicles, outside SSv2's model. CARLA's Traffic
Manager is not used anywhere.

## Authority model

The architecture reduces to five invariants. Everything else is a consequence of them.

1. **One ticker.** Only `csb_bridge` calls `world.tick()` or `apply_settings()`.
   Every `acb_bridge` is passive.
2. **One vehicle spawner.** Only `csb_bridge` creates or destroys *vehicles*. An `acb_bridge`
   creates and destroys *sensors* only, always as children of a vehicle it did not create.
3. **One traffic-light writer.** Only `csb_bridge` sets signal state. CARLA's built-in
   cycling is frozen for the whole run.
4. **One `/clock` publisher per ROS domain.** Never two in the same domain.
5. **One pose authority per vehicle.** Either SSv2-teleport-via-`csb_bridge`, or CARLA PhysX.
   Never both for the same actor.

Invariant 5 is what the current code already gets right. Invariants 1–4 are what it currently
violates; see [Gaps](#gaps-between-this-design-and-the-code).

## Roles

| Component | Owns | Explicitly does not own |
|---|---|---|
| CARLA server | Physics, rendering, ground truth geometry | Traffic Manager is unused |
| `csb_bridge` | World settings, map load, tick, vehicle lifecycle, NPC pose, traffic lights | Sensors, ROS topics, Autoware lifecycle |
| SSv2 | Scenario semantics for the ego and declared NPCs; ego Autoware lifecycle | Anything in domains other than its own |
| `acb_bridge` | Sensors and vehicle interface for exactly one vehicle, in exactly one domain | Simulation control, vehicle lifecycle |
| Autoware | Perception, planning, control for one vehicle | — |

`csb_bridge` is a single process holding the only authoritative CARLA client. It is barely a
ROS participant. SSv2 forks the ego's Autoware into its own domain, `D0`. Background AVs live
in `D1..Dn`, brought up by launch.

## Actor classes

| Class | Pose authority | Spawned by | Autoware perceives | SSv2 knows |
|---|---|---|---|---|
| Scenario ego | PhysX, driven by Autoware `D0` | `csb_bridge`, on `SpawnVehicleEntity(is_ego=true)` | — | yes |
| Scenario NPC / pedestrian / misc | SSv2 teleport via `csb_bridge` | `csb_bridge`, on `Spawn*Entity` | yes, through sensors | yes |
| Background AV | PhysX, driven by Autoware `Dk` | `csb_bridge`, from config at `Initialize` | yes, through sensors | **no** |

### Consequence: background AVs are invisible to SSv2

SSv2's collision detection, `ReachPositionCondition`, `DistanceCondition` and the rest do not
see background AVs. They exist to the other stacks only as LiDAR returns and camera pixels. A
background AV can collide with the scenario ego and SSv2 will still report `exitSuccess`.

This is accepted for v1. The extension point, if they later need to be scored: register each
background AV with SSv2 as a **misc-object entity** whose pose `csb_bridge` writes from CARLA
readback every frame. SSv2 then tracks them without trying to control them. That is the same
read-PhysX-report-to-SSv2 path already used for the ego, so no new machinery is required.

### Why no Traffic Manager

TM-driven ambient traffic would give realistic physics and light-obeying behaviour, but makes
scenario execution non-deterministic and SSv2's position and timing conditions unreliable.
Determinism wins: every moving actor is either scripted by the `.xosc` or driven by a real
Autoware stack. The "Ambient Background Traffic" option in
[roadmap/4-traffic-lights-environment.md](../roadmap/4-traffic-lights-environment.md) is
rejected by this decision.

## Components

### csb_bridge

Current `coordinator.rs` (520 lines) already carries every handler. Map loading, traffic-light
mapping and background-AV spawning would push it well past the point where it can be reasoned
about in one piece. Split along authority lines:

| Module | Responsibility |
|---|---|
| `zmq_server.rs` | REP socket, protobuf dispatch (exists) |
| `coordinator.rs` | Thin dispatch over the modules below |
| `world_authority.rs` | Map resolution and load, world settings, sync-mode state machine, tick |
| `entity_manager.rs` | SSv2 name ↔ CARLA actor ID (exists) |
| `npc_control.rs` | Kinematic teleport for scenario NPCs |
| `ego_readback.rs` | Pose, velocity, acceleration readback for PhysX-driven actors |
| `traffic_light_mapper.rs` | Lanelet signal ID ↔ CARLA actor, state conversion (exists, stub) |
| `background_av.rs` | Config-driven background-AV spawn and teardown |
| `coordinate_conversion.rs` | ROS ↔ CARLA frame conversion (exists) |

New configuration in `bridge_config.yaml`:

```yaml
# Map resolution: lanelet2_map_path basename -> CARLA town
map_alias:
  Town01: Town01
  # ...

# Background AVs. Empty list = single-ego scenario.
background_avs:
  - role_name: bg_av_1
    ros_domain_id: 1          # informational; launch owns the actual domain
    blueprint: vehicle.tesla.model3
    spawn_pose: {x: 0.0, y: 0.0, z: 0.5, yaw: 0.0}   # ROS frame
    goal_pose:  {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}   # consumed by that domain's pilot
```

`ros_domain_id` and `goal_pose` are not used by `csb_bridge` itself — it only spawns the
vehicle with the right `role_name`. They are recorded here so one file describes the whole
run, and the per-domain launch reads the same file.

### acb_bridge

Changes required, all small and all additive:

| Change | Why |
|---|---|
| `publish_clock` bool parameter | Invariant 4. False in `D0` where SSv2 publishes `/clock`; true in `D1..Dn` where nothing else does |
| Clock epoch offset | Background domains have no SSv2, so `/clock` must start near zero rather than at CARLA server uptime |
| Sensor stamps from `node.get_clock().now()` | `data.timestamp()` is CARLA server uptime; SSv2's `/clock` starts at 0. Currently wrong at 5 sites in `sensor_bridge.rs` |
| Tick-wait timeout is not a disconnect | See [Failure modes](#failure-modes) |

`vehicle_name` is already a parameter defaulting to `hero`, so per-instance vehicle selection
needs no change — each background `acb_bridge` is launched with its own `role_name`.

### SSv2 / concealer

Our `launch.hpp` patch hardcodes `--web-addr 0.0.0.0:8082` for the Autoware it forks. This is
fine for one ego and collides for anything more. Parameterise it before a second play_launch-
managed Autoware exists in the same host.

## Data flow

### Startup

```
1. CARLA server up
2. csb_bridge up          -> connect, bind ZMQ. Does NOT touch world settings yet
3. acb_bridge D0..Dn up   -> each waits for /robot_description in its own domain
4. Autoware D1..Dn up     -> background AV stacks, via launch
5. SSv2 up -> Initialize(lanelet2_map_path, step_time)
     csb: resolve town from lanelet2_map_path, load_world() if different
     csb: enumerate + freeze all traffic lights, build lanelet <-> actor map
     csb: spawn background AV vehicles with their configured role_names
     csb: does NOT enable sync mode yet
6. SSv2 -> SpawnVehicleEntity(ego, is_ego=true)
     csb: spawn with role_name=hero
7. acb_bridge D0 finds hero; acb_bridge Dk find their role_names; each attaches sensors
8. First UpdateFrame after ego spawn
     csb: NOW enable sync mode + fixed_delta_seconds, then tick
9. Concealer (D0) initialises localization, sets route, engages
   Background pilots do the same in D1..Dn
```

Step 8 is load-bearing. Enabling sync mode at `Initialize` deadlocks: `acb_bridge` polls
`world.actors()` to discover its vehicle, and in sync mode CARLA does not advance until
something ticks — but `csb_bridge` will not tick until SSv2 sends a frame, and SSv2 does not
send frames until the ego exists. Sync mode must be deferred until after the ego spawn.

### Per frame

```
SSv2 UpdateFrame(current_simulation_time, current_ros_time)
  -> csb world.tick()                     [blocks until CARLA completes the step]
  -> CARLA advances one physics/render step
  -> every acb_bridge's wait_for_tick returns
  -> acb_bridge Dk publishes /clock       [Dk only; D0 stays silent]
  -> sensors fire, stamped from the node's ROS clock
  -> csb returns UpdateFrameResponse

SSv2 UpdateEntityStatus
  -> scenario NPCs: csb set_transform
  -> ego:           csb reads PhysX pose/twist/accel -> response

SSv2 UpdateTrafficLights
  -> csb maps lanelet signal ID -> CARLA actor, set_state
```

### Clock ownership

| Domain | `/clock` publisher | Epoch |
|---|---|---|
| `D0` (scenario ego) | SSv2 `traffic_simulator` | Scenario time, starts at 0 |
| `D1..Dn` (background AVs) | `acb_bridge`, `publish_clock:=true` | `snapshot.elapsed_seconds - epoch_k`, where `epoch_k` is captured on that bridge's first tick |

Cross-domain clock agreement is not required, because domains share no topics. Each Autoware
needs a clock that is monotonic, starts near zero, and agrees with the sensor stamps *in its
own domain*. Physics time is shared through CARLA regardless of what any `/clock` says.

Sensor messages are always stamped from `node.get_clock().now()`, which with
`use_sim_time=true` reads that domain's `/clock`. Never from `data.timestamp()`.

### Traffic lights

`csb_bridge` freezes all CARLA traffic lights at `Initialize` and is thereafter the only
writer. Autoware runs with `use_traffic_light_recognition=true` and perceives them through the
camera pipeline — so the scenario controls the lights, and the stack under test genuinely has
to see them.

This requires two things that do not exist yet:

- **Signal mapping.** Lanelet2 regulatory element IDs and CARLA's OpenDRIVE-derived
  `TrafficLight` actors do not share an ID space. Position-based matching against the Lanelet2
  map named in `InitializeRequest.lanelet2_map_path`, with a per-map YAML fallback for what
  it misses. Detail in [roadmap/4-traffic-lights-environment.md](../roadmap/4-traffic-lights-environment.md).
- **Re-enabling recognition.** `acb_launch/carla_simulator.launch.xml` currently sets
  `use_traffic_light_recognition=false`, and `acb_launch`'s component_state_monitor config
  excludes the traffic-light topic. Both must be reverted for the ego, and the diagnostics
  consequences re-checked — that flag was turned off for a reason.

## Failure modes

| Failure | Handling |
|---|---|
| Sync mode enabled before ego exists | Prevented by construction: sync mode deferred to first `UpdateFrame` after ego spawn |
| `wait_for_tick_or_timeout` expires | **Not** a disconnect. SSv2 routinely pauses between frames — Autoware init alone is up to `initialize_duration` (120 s default). Only genuine transport `CarlaError`s count toward the reconnect counter; a bare timeout logs and keeps waiting |
| Unmapped lanelet signal ID | Warn once per ID, continue. Never fatal |
| Resolved town differs from loaded and reload fails | Fail `Initialize` with a descriptive `Result`, rather than silently running the scenario on the wrong map |
| Background AV stack crashes | Its domain dies alone. `csb_bridge` keeps the vehicle actor; the scenario ego is unaffected. This is the point of domain isolation |
| Ego destroyed mid-run | Not handled — `acb_bridge` has a `TODO` and asks for a restart. Out of scope; see [Gaps](#gaps-between-this-design-and-the-code) |
| Shutdown | `csb_bridge` unfreezes traffic lights, restores async mode, destroys every actor it spawned (scenario + background AVs). Each `acb_bridge` destroys only its own sensors |

### Known conflict: AutowareUniverse vehicle status

Separate from invariant 4, but the same class of problem. SSv2's `AutowareUniverse` concealer
node publishes `/vehicle/status/*` at 30 Hz from its internal bicycle model, while
`acb_bridge` publishes the same topics from real CARLA PhysX state. Both land in `D0`.

[ssv2-launch-configuration.md](ssv2-launch-configuration.md) currently accepts this. It should
not be accepted indefinitely — the two publishers disagree whenever CARLA physics and the
bicycle model diverge, which is exactly the condition a scenario is meant to detect. Tracked
as a gap, not resolved by this document.

## Testing

### Unit

- Coordinate conversion round-trips (exists)
- `lanelet2_map_path` → CARLA town resolution, including the unmapped case
- Lanelet signal ID → CARLA actor matching against a fixture map
- SSv2 `TrafficSignal` → CARLA `TrafficLightState` conversion table, all rows

### Integration (CARLA required, Autoware not)

- `Initialize` loads the correct town and freezes every traffic light
- Sync mode is enabled only after ego spawn; async is restored on exit
- Spawn/despawn lifecycle leaves `EntityManager` and the CARLA world consistent
- NPC teleport tracks the commanded pose within tolerance
- `UpdateTrafficLights` actually changes observable CARLA state
- Background AVs from config appear with the right `role_name`s

### End-to-end

- Single-ego scenario reaches `exitSuccess` (`scenarios/town01_ego_drive.xosc`)
- Exactly one `/clock` publisher per domain; no "jump back in time" in any localization node
- Sensor stamp epoch agrees with `/clock` epoch to within one step time
- Two-domain run: scenario ego plus one background AV, both localize and drive
- Traffic light scenario: ego stops at an SSv2-commanded red

### Regression guards

Three fixes were made in April 2026 and are absent from both repositories' history — deferred
sync mode, `publish_clock`, and the sensor-timestamp epoch. Each gets a test, so that losing
the fix fails the build rather than resurfacing as a localization bug weeks later.

## Gaps between this design and the code

Verified against `carla-scenario-bridge@5890cae` and `autoware_carla_bridge@c3844bd`.

| # | Gap | Where | Invariant |
|---|---|---|---|
| 1 | Sync mode enabled at `Initialize`, not deferred | `coordinator.rs:45` | 1 |
| 2 | `/clock` published unconditionally; no `publish_clock` param | `acb_bridge/src/main.rs:595` | 4 |
| 3 | Sensor stamps use `data.timestamp()` (CARLA server uptime) | `acb_bridge/src/bridge/sensor_bridge.rs`, 5 sites | — |
| 4 | `lanelet2_map_path` ignored; no `load_world()` | `coordinator.rs::initialize` | — |
| 5 | `update_traffic_lights` is a stub; CARLA cycling never frozen | `coordinator.rs:395` | 3 |
| 6 | `TrafficLightBridge` is dead code | `acb_bridge/src/bridge/trafficlight_bridge.rs` | 3 |
| 7 | `use_traffic_light_recognition=false` | `acb_launch/launch/carla_simulator.launch.xml` | — |
| 8 | Tick timeout counts toward disconnect after 3 strikes | `acb_bridge/src/main.rs:578` | — |
| 9 | No background-AV support | `bridge_config.yaml`, `csb_launch` | — |
| 10 | `--web-addr 0.0.0.0:8082` hardcoded in the concealer patch | SSv2 `external/concealer/include/concealer/launch.hpp` | — |
| 11 | Ego respawn unimplemented | `acb_bridge/src/main.rs:564` | — |

Gaps 1, 2 and 3 are regressions of work that was done and lost, not new features.

### Open items requiring verification

- Whether carla-rust 0.14 exposes `Client::load_world`, `TrafficLight::freeze` and
  `TrafficLight::set_state`. The crate is not vendored in this workspace and was not available
  offline when this document was written. If any binding is missing, it must be added upstream
  in [carla-rust](https://github.com/jerry73204/carla-rust) before gaps 4 and 5 can close.
- Diagnostic consequences of re-enabling traffic-light recognition, given `acb_launch`
  currently excludes that topic from `component_state_monitor` and reports a
  `duplicated_node_checker` error attributed to the flag being off.

## Out of scope

- Multiple egos inside one SSv2. `entity_manager.hpp:130` throws
  `"Multiple egos in the simulation are unsupported yet."` Lifting it means a long-lived fork
  of a repository we already rebase onto upstream regularly.
- Per-ego ROS domains inside SSv2. `concealer::ros2_launch` is a plain `fork()` + `exec`, so
  the Autoware it starts always inherits SSv2's `ROS_DOMAIN_ID`.
- CARLA Traffic Manager, in any role.
- Weather and `EnvironmentAction`. SSv2 has no support; see the Phase 4 roadmap.
