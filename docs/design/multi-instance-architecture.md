# Multi-Instance Architecture

How CARLA, `csb_bridge`, `acb_bridge`, SSv2 and N Autoware instances divide responsibility
over a single shared CARLA world.

This document supersedes the parts of [architecture.md](architecture.md) that assume a
single Autoware instance and a single ROS domain. Where the two disagree, this document wins.

**Status**: implemented and verified end to end on 2026-08-27 with two Autoware instances
driving two CARLA vehicles at once — see [Verified two-instance run](#verified-two-instance-run).
The gap table below is older than that run and marks what has since been closed.

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
ROS participant. The ego's Autoware shares SSv2's domain, `D0` below. Background AVs live in
`D1..Dn`, brought up by launch.

`D0`, `D1`… are roles, not `ROS_DOMAIN_ID` values. The mapping in this repo is **ego and
SSv2 = domain 1, background AVs = 2 and up, and domain 0 unused**: an unconfigured ROS
process on the host lands in 0, and one that joins a run's graph uninvited is a debugging
session nobody wants. `just ego-av` and `just scenario` both export `ego_domain` (default
1); `just bg-av` takes its domain as an argument and defaults to 2.

## Actor classes

| Class | Pose authority | Spawned by | Autoware perceives | SSv2 knows |
|---|---|---|---|---|
| Scenario ego | PhysX, driven by Autoware `D0` | `csb_bridge`, on `SpawnVehicleEntity(is_ego=true)` | — | yes |
| Scenario NPC / pedestrian / misc | SSv2 teleport via `csb_bridge` | `csb_bridge`, on `Spawn*Entity` | yes, through sensors | yes |
| Background AV | PhysX, driven by Autoware `Dk` | `csb_bridge`, from config at `Initialize` | yes, through sensors | **no** |

"Spawned by" is about the CARLA *actor*. The **Autoware stacks are all launched by us** —
`just ego-av` for the ego's, `just bg-av` for each background AV's — and none by SSv2, which
runs with `launch_autoware:=false` (phase 012). The ego's stack is therefore no different in
lifecycle from a background AV's: started before the scenario, reused across scenarios, and
never torn down by SSv2. What still differs is *who drives the autonomy*: the concealer for
the ego, `acb_pilot` for each background AV.

### Consequence: background AVs are invisible to SSv2

SSv2's collision detection, `ReachPositionCondition`, `DistanceCondition` and the rest do not
see background AVs. They exist to the other stacks only as LiDAR returns and camera pixels. A
background AV can collide with the scenario ego and SSv2 will still report `exitSuccess`.

This is accepted for v1. The extension point, if they later need to be scored: register each
background AV with SSv2 as a **misc-object entity** whose pose `csb_bridge` writes from CARLA
readback every frame. SSv2 then tracks them without trying to control them. That is the same
read-PhysX-report-to-SSv2 path already used for the ego, so no new machinery is required.

### Consequence: puppeteered actors are kinematic

Everything SSv2 teleports -- NPC vehicles, pedestrians, misc objects -- is spawned with CARLA
physics **disabled**, and that is the only consistent combination. A physics-driven vehicle
carries state PhysX integrates every tick (linear and angular velocity, wheel spin,
suspension compression, contact constraints); a per-tick `set_transform` cuts across all of
it. PhysX moves the body by `v·Δt` and the teleport overwrites the result, so CARLA's
velocity no longer describes the motion; the suspension never settles and the car jitters;
a teleport into an overlap becomes a one-step separation impulse. Driving NPCs by velocity
instead of pose keeps physics intact but lets CARLA integrate its own path, so the NPC
drifts from where the scenario puts it and SSv2 stops being the authority (invariant 5).
Two authorities on one actor, with the reported pose the commanded one, is exactly the
divergence the scenario cannot see.

A physics-off actor is a PhysX **kinematic** body: it keeps its collider and is visible to
every sensor, but nothing pushes it.

| Case | Behaviour | Evidence |
|---|---|---|
| Ego (physics on) hits an NPC | Contact generated; the ego's collision sensor fires; the ego is stopped or deflected | live, roadmap 014: `ego hit 'npc_rear'`, 1021 N·s |
| LiDAR, camera, radar | See the NPC (raycasts and rendering use geometry, not physics) | every scenario's perception and ground truth |
| The NPC when hit | Does not move: an infinite-mass body on SSv2's path | PhysX kinematic semantics |
| NPC vs NPC, NPC vs walker | No contact between two kinematic bodies; they interpenetrate | PhysX semantics (not tested here) |
| Gravity, ground | None: the NPC sits exactly at the commanded pose, so z must be right (014 fixed walker height) | 014 hardening |
| Wheels, walk cycle | Wheels do not turn, walkers glide | cosmetic |

What follows:

- **Collision verdicts are SSv2's.** Its bounding-box `CollisionCondition` covers every pair,
  NPC-NPC included, as in AWSIM. csb's collision sensor on the ego is diagnostic, and it
  agrees with SSv2 on ego contacts **provided the contact lasts more than one frame**: csb
  applies SSv2's NPC pose one frame after SSv2 computes it, so a scenario that ends on the
  first frame of overlap ends before CARLA ever ticks with the bodies touching
  (`scenarios/town01_rear_contact.xosc`; [scenario-authoring.md](scenario-authoring.md)).
- **NPC velocity comes from acb, not CARLA.** CARLA ignores `set_target_velocity` and
  `WalkerControl` on a physics-off actor (`get_velocity()` stays 0.000; 1.504 m/s for the
  same call with physics on, roadmap 014). acb differentiates each actor's snapshot pose on
  simulation time and publishes that in ground-truth objects (acb `028d8a8`; a walker read
  0.695 m/s against 0.692 m/s from its displacement). CARLA's own velocity readout for NPCs
  stays 0, which only matters to tools that ask CARLA directly.
- **Post-crash dynamics are wrong by construction**: an NPC is never shoved, spun or
  stopped by an impact. Fine for "did they collide", not for "what happened after". If a
  scenario ever needs that, the right shape is a per-entity handover -- physics on and SSv2's
  control released at the moment of impact -- not a global change.

The ego is the exception and keeps physics: Autoware drives it, CARLA moves it, and its pose
is read back rather than commanded.

### Worked example: two domains

One scenario ego plus one background AV. CARLA must already be running.

**1. Declare the background AV** in `config/bridge_config.yaml`:

```yaml
ego:
  role_name: "hero"

background_avs:
  - role_name: "bg_av_1"
    ros_domain_id: 2
    blueprint: "vehicle.tesla.model3"
    spawn_pose: {x: 140.0, y: -55.5, z: 0.5, yaw: 180.0}
    goal_pose:  {x: 110.0, y: -55.5, z: 0.0, yaw: 180.0}
```

`role_name` must be unique — it is how each `acb_bridge` finds its own vehicle, and a
duplicate is rejected at startup.

**2. Start the bridge and the scenario** in the ego's domain (`D0`, i.e. `ROS_DOMAIN_ID=1`),
with background AVs switched on -- they are opt-in:

```bash
CSB_BACKGROUND_AVS=all just e2e scenarios/town01_ego_drive.xosc   # exports ROS_DOMAIN_ID=1 itself
```

`csb_bridge` loads the map, spawns `bg_av_1` at `Initialize`, then spawns the ego when SSv2
asks. `acb_bridge` in this domain runs with `publish_clock:=false`, because SSv2 owns
`/clock` here. This applies to a **managed** ego, which shares SSv2's domain by necessity;
an unmanaged one gets its own domain and publishes its own clock like any background AV.

Nothing in `csb_bridge` drives `bg_av_1`: until its own stack (step 3) is up it sits at its
spawn pose, in the lane it was placed in, and a scenario ego in that lane stops behind it. On
2026-09-27 that failed `town01_pedestrian.xosc` three times and was misread as a planner
stall. So background AVs are **opt-in**: `background_avs` in the config declares them, and
`CSB_BACKGROUND_AVS`, read once at bridge startup, selects which to spawn -- `none` (the
default, also when unset or empty), `all`, or a comma-separated list of `role_name`s (names
the config does not declare are warned about and ignored). A plain `just run` is a
single-ego run; the two-domain workflow asks explicitly, `CSB_BACKGROUND_AVS=all just run`,
and `just two-av` does that itself (and refuses to run against an already-running bridge
that does not spawn `bg_av_1`; `just bg-av` warns in the same case). The bridge logs one
startup line with the enabled set and how to enable the rest, an INFO at each `Initialize`
naming declared-but-disabled AVs, and a WARN naming the pose for each one it does spawn.

**3. Start the background AV's stack** in its own domain (`D1`, i.e. `ROS_DOMAIN_ID=2`):

```bash
ROS_DOMAIN_ID=2 \
  play_launch launch --web-addr 0.0.0.0:8083 csb_launch background_av.launch.xml \
      vehicle_name:=bg_av_1 \
      map_path:=$PWD/data/carla-autoware-bridge/Town01 \
      goal_poses_file:=$PWD/scenarios/bg_av_1_poses.yaml
```

`publish_clock` is `true` here — there is no SSv2 in this domain, so without it there would
be no simulation clock at all. `--web-addr` keeps the web UI off the port the ego's Autoware
already took. (This was a `PLAY_LAUNCH_WEB_ADDR` environment variable while the concealer
still forked Autoware and needed to pass the address through its patched `launch.hpp`; with
`launch_autoware:=false` nothing reads it, so the flag says it directly.)

`goal_poses_file` feeds this domain's pilot (`acb_pilot`'s `auto_drive`), which replaces the
concealer here: it waits for localization, sets the route, and engages. The file is in
acb_pilot's own format — `goal_pose: {x, y, z, qx, qy, qz, qw}`, quaternion in the map
frame — not the `{x, y, z, yaw}` shorthand recorded in `bridge_config.yaml`. See
[Per-domain pilot](#per-domain-pilot) below.

Order matters only in that CARLA comes first. Each `acb_bridge` waits for both its Autoware
and its vehicle, and the pilot waits for the AD API and localization, so `D1` may start
before or after the scenario.

**What you get**: the ego's Autoware perceives `bg_av_1` through its own sensors, as an
ordinary obstacle. SSv2 does not know it exists.

### Why no Traffic Manager

TM-driven ambient traffic would give realistic physics and light-obeying behaviour, but makes
scenario execution non-deterministic and SSv2's position and timing conditions unreliable.
Determinism wins: every moving actor is either scripted by the `.xosc` or driven by a real
Autoware stack. The "Ambient Background Traffic" option in
[roadmap/004-traffic-lights-environment.md](../roadmap/004-traffic-lights-environment.md) is
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
    goal_pose:  {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}   # informational; the pilot reads its own file
```

`ros_domain_id` and `goal_pose` are not used by `csb_bridge` itself — it only spawns the
vehicle with the right `role_name`. They are recorded here so one file describes the whole
run. Launch does not parse this file either: the operator passes matching values
(`vehicle_name`, `ROS_DOMAIN_ID`, `goal_poses_file`) as launch arguments, and this config is
the single place to read off what they should be.

### acb_bridge

Changes required, all small and all additive:

| Change | Why |
|---|---|
| `publish_clock` bool parameter | Invariant 4. False wherever SSv2 shares the domain — `D0` with a managed ego — and true everywhere else, which with `managed_ego:=false` is every AV domain including the ego's |
| Clock epoch offset | Background domains have no SSv2, so `/clock` must start near zero rather than at CARLA server uptime |
| Sensor stamps from `node.get_clock().now()` | `data.timestamp()` is CARLA server uptime; SSv2's `/clock` starts at 0. Currently wrong at 5 sites in `sensor_bridge.rs` |
| Tick-wait timeout is not a disconnect | See [Failure modes](#failure-modes) |

`vehicle_name` is already a parameter defaulting to `hero`, so per-instance vehicle selection
needs no change — each background `acb_bridge` is launched with its own `role_name`.

### Per-domain pilot

The concealer exists only in the ego's domain, so every background domain needs something
else to set a route and engage. That is `acb_pilot`'s `auto_drive` node, launched by
`background_av.launch.xml` (skip with `use_pilot:=false`). It waits for the AD API services,
waits for GNSS-driven localization to reach `INITIALIZED`, sets the route, engages
autonomous mode, and exits when the route state reaches `ARRIVED` — nonzero on any failure,
so a pilot that could not engage is visible in the process table, not just in a vehicle that
never moves.

Its one input is the `poses_file` parameter, exposed as the `goal_poses_file` launch
argument: a YAML whose `goal_pose` is `{x, y, z, qx, qy, qz, qw}` in the map frame. This is
deliberately not derived from `bridge_config.yaml`'s `goal_pose: {x, y, z, yaw}` — the pilot
lives in a read-only submodule and takes a file path, not an inline pose, and converting at
launch time would mean generating files from XML. acb_pilot ships per-town examples under
its `config/poses/` share directory, and `ros2 run acb_pilot capture_poses` records new ones
from RViz goal clicks.

The pilot adds no startup-order constraint: it only ever waits, so it comes up with the rest
of the domain's stack.

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
4. Autoware D0..Dn up     -> EVERY stack, ego included, via our launch files.
                             `just ego-av` for D0, `just bg-av` for the rest.
                             SSv2 launches none of them (launch_autoware:=false).
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

Step 4 changed with phase 012. The ego's Autoware used to be forked by SSv2's concealer
during step 9; it is now brought up with every other stack, before SSv2 starts. The ordering
is enforced rather than trusted: `just scenario` refuses to start unless
`/api/operation_mode/state` has a publisher (`_require-ego-stack`). Without that check the
failure is slow and points at the wrong thing — the concealer queues a `ChangeToStop` with a
180 s service timeout, so a late stack means three minutes of evaluating nothing and then an
`AutowareError` naming a service.

The ego stack also **outlives the scenario**: with no child process, SSv2 cannot tear it
down, and consecutive runs re-engage the same stack (verified live, phase 012).

Steps 5 and 8 are superseded by [time-and-ticking.md](time-and-ticking.md) (roadmap 015):
CARLA is synchronous from csb's first `Initialize` on and csb is its only ticker; idle time
between scenarios is a pause. The deadlock this paragraph used to warn about -- acb polling
`world.actors()` in a world nothing ticks -- is gone: acb follows `on_tick`, and the ego's
spawn tick delivers the first frame.

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

Superseded by [time-and-ticking.md](time-and-ticking.md) (roadmap 015). In every domain
`acb_bridge` publishes `/clock` from CARLA's frame time plus an episode epoch and stamps
everything it publishes with the frame's time; SSv2 takes the same time from csb and
publishes no `/clock`.

### Traffic lights

`csb_bridge` freezes all CARLA traffic lights at `Initialize` and is thereafter the only
writer. Autoware runs with `use_traffic_light_recognition=true` and perceives them through the
camera pipeline — so the scenario controls the lights, and the stack under test genuinely has
to see them.

This requires two things that do not exist yet:

- **Signal mapping.** Lanelet2 regulatory element IDs and CARLA's OpenDRIVE-derived
  `TrafficLight` actors do not share an ID space. Position-based matching against the Lanelet2
  map named in `InitializeRequest.lanelet2_map_path`, with a per-map YAML fallback for what
  it misses. Detail in [roadmap/004-traffic-lights-environment.md](../roadmap/004-traffic-lights-environment.md).
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


## Verified two-instance run

Run on 2026-08-27: one CARLA, one `carla_scenario_bridge`, and **two complete Autoware stacks**
in separate ROS domains, each driving its own vehicle through its own `acb_bridge`.

```
just carla-start                     # one server, shared
CSB_BACKGROUND_AVS=all just run      # csb: the only ticker, the only vehicle spawner
just ego-av                          # Autoware + acb_bridge, ROS domain 1, web UI 8082
just bg-av                           # Autoware + acb_bridge + pilot, domain 2, web UI 8083
just scenario scenarios/town01_ego_drive.xosc
```

Both vehicles drove, under their own stacks, at the same time:

```
bg_av_1    start (230.0,-129.8) -> end (130.5,-129.4)  travelled 99.5 m  peak 5.03 m/s
hero       start (190.8,-130.1) -> end (124.7,-129.7)  travelled 66.2 m  peak 3.99 m/s
```

`hero` is SSv2's scenario ego, routed by the scenario. `bg_av_1` is outside SSv2's model
entirely: `csb` spawns it from `background_avs` in `bridge_config.yaml`, and its own Autoware
drives it to the goal in `scenarios/bg_av_1_poses.yaml`.

### Invariants, as measured

| Invariant | Check | Result |
|---|---|---|
| 4. One `/clock` per domain | `ros2 topic info /clock` in each domain | 1 publisher in domain 1, 1 in domain 2 |
| — One bridge per vehicle | `ros2 node list \| grep acb_bridge` | 1 in each domain |
| — Domain isolation | SSv2's `openscenario_interpreter` | present in domain 1, absent in domain 2 |
| — Per-vehicle status | `/vehicle/status/velocity_status` | published in domain 2 by that vehicle's own bridge |
| 1. One ticker | CARLA in synchronous mode, driven only by csb | the run held its step rate; a second ticker would double-step |

Domain 1 carried 211 nodes (ego Autoware, SSv2 and its bridge), domain 2 carried 180 (the
background AV's Autoware and bridge). Two stacks plus CARLA used about 42 GB of RAM.

### Operational constraints this run established

**Give each stack its own `--log-dir`.** play_launch names a run directory by timestamp at
one-second resolution, in the working directory. Two stacks whose setup finishes in the same
second collide, and the loser dies several minutes in with

```
Error: unable to create directory play_log/2026-08-27_23-08-40
Caused by: 0: File exists (os error 17)
```

Staggering the launches does not fix it — the collision is at the end of the parse phase, not
at launch, and parse durations converge. The justfile now passes `--log-dir play_log/ego`,
`play_log/bg-<vehicle>` and `play_log/scenario`, after which two stacks started in the same
second both come up. Anything reading launch parameters back out of play_log gains one path
component.

**Restart `just bg-av` before every scenario run.** SSv2 restarts simulation time near zero
each run, and a stack that has already seen a later clock stalls on the backward jump: its
pilot waits for a fresh pose that never arrives. Measured on a second consecutive run without
restarting, the background AV managed 3.3 m at 1.67 m/s where its first run covered 99.5 m at
5.03 m/s. The ego stack is unaffected because SSv2 respawns its ego each run.

**Start the bridge with a real detach.** `setsid` alone does not always create a new session,
so a bridge started that way can be taken down with the shell that launched it. `setsid --fork`
is reliable. This matters more than it sounds: nothing restarts `csb`, and a scenario run
against a dead bridge spawns no ego at all while still producing a full set of logs.

## Gaps between this design and the code

Verified against `carla-scenario-bridge@5890cae` and `autoware_carla_bridge@c3844bd`.

| # | Gap | Where | Invariant |
|---|---|---|---|
| 1 | ~~Sync mode enabled at `Initialize`, not deferred~~ **fixed** | `coordinator.rs` — `FrameAction` state machine | 1 |
| 2 | ~~`/clock` published unconditionally; no `publish_clock` param~~ **fixed** | `acb_bridge/src/main.rs`, `acb_bridge.launch.xml` | 4 |
| 3 | ~~Sensor stamps use `data.timestamp()` (CARLA server uptime)~~ **fixed** | `acb_bridge/src/bridge/sensor_bridge.rs`, 5 sites | — |
| 4 | ~~`lanelet2_map_path` ignored; no `load_world()`~~ **fixed** | `coordinator.rs::load_scenario_map` | — |
| 5 | ~~`update_traffic_lights` is a stub; CARLA cycling never frozen~~ **fixed** | `coordinator.rs::freeze_traffic_lights` | 3 |
| 6 | `TrafficLightBridge` is dead code | `acb_bridge/src/bridge/trafficlight_bridge.rs` | 3 |
| 7 | `use_traffic_light_recognition=false` | `acb_launch/launch/carla_simulator.launch.xml` | — |
| 8 | Tick timeout counts toward disconnect after 3 strikes | `acb_bridge/src/main.rs:578` | — |
| 9 | ~~No background-AV support~~ **fixed** | `bridge_config.yaml` `background_avs`, `csb_launch/background_av.launch.xml`, `just bg-av` | — |
| 10 | ~~`--web-addr 0.0.0.0:8082` hardcoded in the concealer patch~~ **closed** — the concealer no longer launches anything (`launch_autoware:=false`), so `launch.hpp` is never called and the address it hardcoded is unreachable | SSv2 `external/concealer/include/concealer/launch.hpp` | — |
| 11 | Ego respawn unimplemented | `acb_bridge/src/main.rs:564` | — |

Gaps 1, 2 and 3 were regressions of work that was done and lost, not new features. Fixed
2026-07-28, each with a unit test so losing the fix fails the build:

- **Gap 1** — `coordinator.rs` no longer touches world settings in `initialize()`. A
  `FrameAction` state machine keeps CARLA async until the ego is spawned, enables sync mode on
  the first `UpdateFrame` after it, and ticks thereafter. `update_step_time` only applies
  settings once sync mode is on; `restore_async_mode` is a no-op if we never enabled it.
- **Gap 2** — `publish_clock` bool parameter on `acb_bridge` (default `true`), plumbed through
  `acb_bridge.launch.xml` and set to `false` in `csb_launch/demo.launch.xml` where SSv2 owns
  `/clock`. A `ClockEpoch` helper rebases CARLA server uptime onto scenario time and resets on
  reconnect.
- **Gap 3** — all five sensor callbacks stamp from `utils::create_ros_header_from_node`, which
  reads the node's ROS clock. `autoware.tick()` and `vehicle_control.publish_status()` take the
  same source. The old helper is renamed `create_ros_header_from_epoch_seconds` so the two
  remaining callers in the dead `vehicle_bridge.rs` are explicit about owning their epoch.

### Open items requiring verification

- ~~Whether carla-rust exposes `Client::load_world`, `TrafficLight::freeze` and
  `TrafficLight::set_state`.~~ **Resolved 2026-07-28: all present, no upstream work needed.**
  Note the crate is not crates.io `carla` 0.14.0 — a `[patch.crates-io]` redirects to
  `jerry73204/carla-rust` rev `73f5e16`, whose API differs (every call returns
  `crate::Result<T>`). Detail in
  [roadmap/009](../roadmap/009-map-and-traffic-lights.md#verify-carla-rust-coverage).
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
