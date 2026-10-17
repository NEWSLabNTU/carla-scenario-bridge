# Phase 035: Backend Seam

First step of running SSv2 on more than one simulator
([design/simulator-backends.md](../design/simulator-backends.md)): put everything csb does
through carla-rust behind one Rust trait, with CARLA as its only implementation, and change
no behaviour. The repository keeps its name; the seam is where it would later split.

**Source**: user direction 2026-10-16 ("revise csb so SSv2 works with Isaac Sim ... keeps the
csb name but works with various simulator backends").
**Depends on**: [034](034-bridge-failure-paths.md) (done 2026-10-16: the tick and spawn
paths moved once, after it); on top of 024's gap trace.
**Feeds**: [037](037-isaac-world-server.md) (the Isaac backend implements the trait).

## Problem

- `coordinator.rs` (5.3 k lines) mixes SSv2 semantics with CARLA calls: about 30 `carla::`
  uses there, plus `collision_monitor.rs`, `weather.rs`, `traffic_light_mapper.rs`,
  `carla_version.rs`, `main.rs`. There is no place an Isaac implementation could go.
- Several CARLA facts leak into simulator-free logic: actor ids (`u32`) as entity handles,
  the left-handed Y-flip in `coordinate_conversion.rs`, CARLA waypoints in `autopilot.rs`,
  blueprint boxes in `blueprint_boxes.rs`, town names in map loading.

## Design

- **`SimBackend` trait** (new `src/backend/mod.rs`), object-safe, one instance per csb
  process. Operations, all in the ROS map frame (right-handed, metres, radians):
  - `connect() / version()`; `load_world(map: &MapRef)` where `MapRef` carries the scenario's
    map dir, the Lanelet2 path and the backend's world id (town, or USD path);
  - `spawn(&SpawnRequest) -> EntityHandle` (kind, subtype, declared box, pose, physics
    driven or puppeteered, ego flag, role name); `despawn(handle)`;
  - `step(&FrameInput) -> FrameOutput`: puppeteered poses in (batch), one frame advanced,
    out: sim time, frame number, states of every entity (pose, twist, accel), contacts since
    the last step. One call per SSv2 `UpdateFrame`;
  - `signals()` / `set_signals(..)` (CARLA: the 009/015 machinery; others may return
    `Unsupported`); `weather()` / `set_weather(..)` likewise; `entity_box(handle)` (the
    simulator's own box for 022's residual).
- **`EntityHandle`** is opaque (`u64`); CARLA's actor id stays inside `CarlaBackend`.
- **Errors**: `BackendError::{Unsupported(what), Disconnected, Failed(msg)}`; the coordinator
  maps `Unsupported` to an honest failure reply (006), never to success.
- **What stays in the coordinator**: SSv2 names and the entity table, the episode clock (015),
  frame accounting, entity geometry choice (022) through a per-backend asset table, the gap
  trace (024) fed from `FrameOutput`, contact reports (021) fed from `FrameOutput.contacts`.
- **`CarlaBackend`** (`src/backend/carla/`): today's code moved — Y-flip, blueprints,
  sync settings, signal mapper, weather, collision sensors, reconnect. CARLA-only features
  (autopilot on CARLA waypoints) stay behind it as backend extensions the coordinator asks
  for by capability.
- **Selection**: ROS param `backend` (`carla` default), argument of `simulation.launch.xml`.
  The Cargo feature for the CARLA version (018) stays; a build without CARLA is not a goal yet.

## As built (2026-10-17)

What changed from the design above, and why:

1. **No `EntityHandle`: entities cross the seam by their SSv2 name.** Every SSv2 request
   names its entity, and names are unique per scenario. A handle would be a second key the
   coordinator keeps in step with the backend's own table -- including across a CARLA
   restart, which forgets every entity inside the backend. CARLA's actor id stays inside
   `CarlaBackend`, which is what the handle was for.
2. **Two calls per frame, as SSv2 sends them, not one `step`.** `update_entity_status`
   carries every entity's pose in and every state out; `update_frame` advances the world.
   Both are batched, so a backend behind a socket (037) makes one round trip per request.
3. **The coordinator keeps less than planned.** It owns the protocol: decoding each request
   into `Episode` / `SpawnRequest` / `StatusBatch` / `Goal`, the driver rule (`Driver`,
   from `is_ego` and the controller), and mapping every `BackendError` to a failed reply.
   The episode clock, frame accounting, entity geometry, the gap trace and the contact
   reports stay in `CarlaBackend`: each is built on CARLA specifics -- the per-episode reset
   of CARLA's elapsed time and acb's epoch marker, CARLA's collision sensors, its world
   snapshots, its blueprints. Lifting them now would change behaviour with one backend to
   judge by; 037 lifts what the two backends turn out to share.
4. **Errors**: `BackendError::{Failed, Unsupported, Disconnected}`, each a failed reply with
   the backend's text. CARLA uses `Unsupported` for SSv2's sensors (acb publishes CARLA's)
   and `Disconnected` for an `Initialize` whose reconnect failed.
5. **Weather presets** are plain data in the shared `weather.rs` (the `/carla/*` services'
   model); `backend/carla/weather.rs` converts to carla-rust's `WeatherParameters` and a
   test checks every preset against carla-rust's.
6. **`backend: isaac`** parses, and the bridge refuses it at startup ("not built yet
   (roadmap 037)") until 037 lands; any other name fails as a bad parameter.

## Steps

- [x] Inventory every CARLA call in csb (file, purpose, per-frame or per-episode) into an
      appendix of this doc; mark which become trait methods and which stay CARLA-only.
- [x] Introduce the trait (entities by name, see "As built"); map-frame conversion is the
      CARLA side's: every Y-flip and map-origin call is inside `CarlaBackend`.
- [x] Move code into `CarlaBackend`, passing `just test`: `coordinator.rs` became
      `backend/carla/backend.rs` with the CARLA-only modules beside it (`git mv`); a new
      `coordinator.rs` decodes requests, so git's history of the old handlers is
      `coordinator.rs`'s up to 035.
- [x] Coordinator talks only to `dyn SimBackend`; `carla::` appears nowhere outside
      `src/backend/carla/` (`backend::tests::carla_rust_is_used_only_by_the_carla_backend`).
- [x] A fake backend (`src/backend/fake.rs`, in-memory, deterministic) for unit tests of the
      coordinator without CARLA (`coordinator::tests`, 7 tests).
- [x] `backend` parameter in config, `bridge.launch.xml`, `simulation.launch.xml` and
      `just run` (`CSB_BACKEND`); an unknown value fails at startup.
- [x] Docs: architecture.md gains the seam; CLAUDE.md repository structure updated.

## Acceptance

- `just check` and `just test` pass; new coordinator tests run on the fake backend.
- The nine starter scenarios and both AWF suites pass on CARLA as before (020's proof run),
  and the gap trace of one scenario is identical in content to a pre-refactor run.
- Frame budget (016: 20 NPCs p95 1.79 ms) not worse by more than 10 %.

## Results (2026-10-17)

Same host, same CARLA 0.9.16 server, same Autoware 1.5.0 ego stack, acb at the checked-out
`3ed0a8a` (not the pinned `10d03a1`; left as found) and no 042 overlay built, before
(`385520b`) and after (`7602626`) the change; `just build` for each, the gap trace on.

- **`just check`, `just test`**: pass; 341 Rust tests (9 new: 7 coordinator tests on the
  fake, the `carla::` check, the preset check) and 174 Python.
- **Gap trace identical in content.** `scenarios/bench/town01_npc_20.xosc` (20 tracked NPCs,
  no ego, 1201 frames) three times before and three times after: every record of every run
  is the same -- 1224 records, every SSv2 and CARLA pose and velocity equal to the last
  digit -- once the episode number, `start_unix`, `actor_id` and `carla_frame` (server
  counters) are set aside. The three runs before were identical to each other the same way,
  so the comparison can tell a change.
- **Frame budget not worse.** The same benchmark, processing excluding the tick, p50 / p95:
  before 0.47 / 0.64, 0.47 / 0.62, 0.46 / 0.62 ms; after 0.42 / 0.54, 0.43 / 0.55 ms (the
  third run's summary went with a bridge respawn). Tick 5.3-5.5 / 6.2-6.4 ms both.
- **Starter scenarios, as before**: `basic/` + `tutorial/` 6/8 after, the **same 6/8, with
  the same times to the second, before**: `ego_drive` 48 s, `engage_state` 44, `pedestrian`
  127, `rear_contact` 24, `simulator_autopilot` 47, `follow_to_goal` 41 PASS. The two
  failures are the same on both binaries and are not the seam's: `town01_traffic_light`
  times out at 300 s, and `town02_episode_change` ends on a failed first frame after the
  load to Town02 -- a tick that never returned within the watchdog, so the bridge exited 75
  and respawned (034's recovery path). 042 ran `traffic_light` to PASS with acb pinned and
  the overlay built; neither is so here. `multi_av/town01_two_av` (two Autowares, the
  background AV through the agent relay): **PASS**.
- **AWF**: UC-ACC **27/27** (1976 s), UC-AEB **9/9** (543 s).
- `backend:=isaac` stops the bridge at startup with "the Isaac Sim backend is not built yet
  (roadmap 037)"; `backend:=gazebo` as a bad parameter naming the value and the two
  accepted ones.

## Appendix: CARLA calls in csb (2026-10-17)

All in `backend/carla/` after this phase (`backend.rs` unless named). "When" is how often a
scenario makes them; the trait method is the request that leads to them.

| Area | CARLA calls | When | Trait method |
|---|---|---|---|
| Connect, version (`mod.rs`, `version.rs`, `reconnect_carla`) | `Client::connect`, `set_timeout`, `server_version`, `world`, `map().name()`, `world.id()` | start-up, reconnect | constructor `backend::carla::connect`; `initialize` (probe, reconnect) |
| Idle check | `server_version` with a short timeout | every few s between scenarios | `probe_connection_idle`, `server_lost` |
| Synchronous mode (`enable_sync_mode`, `restore_async_mode`) | `world.settings`, `apply_settings` (watched), TM `set_synchronous_mode` | `Initialize`, ego spawn, map load, shutdown | `initialize`, `spawn`, `update_step_time`, `shutdown` |
| Tick (`tick_frame`, `watched_tick`) | `tick_or_timeout` on the watchdog thread, `snapshot().timestamp()` | every frame, x substeps; one settling tick per spawn | `update_frame`, `take_tick_time`, `wedged`, `abandon_after_hang` |
| Map load (`load_scenario_map`, `prepare_map`, `reload_as_pause`) | `avaiable_maps` (once), `load_world_opt` (watched), `snapshot` | `Initialize` on a town change | `initialize` |
| Episode clock, epoch marker | `actors().filter(marker)`, `actor_builder(..).spawn`, `destroy`, `wait_for_tick_or_timeout` | start-up, map load, CARLA restart, first tick after | `before_serving`, `initialize`, `update_frame` |
| Teardown, reaper (`destroy_all_spawned`, `reap_orphaned_actors`, `census`) | `actor`, `destroy_with_children`, `apply_batch_sync(DestroyActor)`, a probe client's `actors_by_ids`, `attributes`, `parent_id` | `Initialize`, shutdown | `initialize`, `shutdown` |
| Spawn (`spawn_entity_within`, `CarlaSpawn`, `build_actor`, `resolve_blueprint_key`, `resolve_spawn_height`) | `actor_builder`, `set_attribute`, `apply_batch_sync(SpawnActor)`, `map().waypoint_at`, `actor`, `bounding_box`, `set_simulate_physics`, `apply_control` (park), `physics_control` | per spawn | `spawn` |
| Collision sensors (`collision_monitor.rs`) | spawn `sensor.other.collision` attached, `listen`, `stop`, `destroy` | ego and tracked-NPC spawn, despawn, teardown | `spawn`, `despawn`, `initialize` |
| Despawn (`despawn_entity_inner`) | `actors().find`, `destroy_with_children` | per despawn | `despawn` |
| State readback (`read_actor_state`) | `actors().find`, `transform`, `velocity`, `angular_velocity`, `acceleration` | every frame, per ego / simulator- / agent-driven vehicle | `update_entity_status` |
| Kinematic teleport (`set_actor_transform`, `walker_teleport_z`) | `actors().find`, `set_transform`, `map().waypoint_at` (per 2 m) | every frame, per kinematic entity | `update_entity_status` |
| Tracked NPCs (`track_npc`, `fresh_snapshot`) | one `snapshot` and one `actors_by_ids` per frame; per NPC `set_target_velocity`, `set_target_angular_velocity`, `apply_control` (steer, walker), `set_transform` (snap), `set_simulate_physics(false)` (fallback) | every frame | `update_entity_status` |
| Traffic Manager (`simulator_autopilot`) | `instance_tm`, `register_vehicles`, `unregister_vehicles`, `set_desired_speed`, `set_custom_path`, `set_auto_lane_change`, `shutdown`; `map().topology`, `waypoint_at` for the lane graph | spawn, every frame, goal, despawn | `spawn`, `update_entity_status`, `update_entity_goal`, `despawn` -- **CARLA-only** (`Driver::SimulatorAutopilot`) |
| Traffic lights (freeze, hold, set, table) | `freeze_all_traffic_lights`, `actors().filter(traffic_light)`, `set_{green,yellow,red}_time`, `traffic_light_from_open_drive`, `set_state`, `opendrive_id` | `Initialize`, every `UpdateTrafficLights`, shutdown | `initialize`, `update_traffic_lights`, `shutdown` |
| Agent-driven vehicles | `actors()` for the `role_name` check | spawn | `spawn` (the agent link itself is simulator-free) |
| Weather (`reapply_weather`, `weather.rs`) | `set_weather`, `weather` | connect, map load, reconnect, `/carla/*` services | `before_serving`, `get_weather`, `set_weather` |
| Gap trace (`gap_trace.rs`, `trace_*`) | `snapshot.find` per entity (one snapshot per frame) | every frame, when on | `update_entity_status`, `update_frame`, `despawn` |
| Signal table generator | as map load, plus the light enumeration | `--generate-signal-table` | **CARLA-only**: `CarlaBackend::generate_signal_table` |

CARLA-shaped modules that use no carla-rust and so stay where they are until the split, or
until 037 shows what they share: `coordinate_conversion` (the Y-flip), `blueprint_boxes`,
`npc_tracking` (CARLA degrees and PhysX gains), `spawn_ladder`, `actor_census`,
`epoch_marker`, `episode_clock` and `clock_store`, `autopilot` (CARLA's lane graph),
`sensor_release` (acb), `body_clearance`, `tick_watchdog` and `budgets` (LibCarla's
timeouts).
