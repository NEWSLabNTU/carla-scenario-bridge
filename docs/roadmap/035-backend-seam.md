# Phase 035: Backend Seam

First step of running SSv2 on more than one simulator
([design/simulator-backends.md](../design/simulator-backends.md)): put everything csb does
through carla-rust behind one Rust trait, with CARLA as its only implementation, and change
no behaviour. The repository keeps its name; the seam is where it would later split.

**Source**: user direction 2026-10-16 ("revise csb so SSv2 works with Isaac Sim ... keeps the
csb name but works with various simulator backends").
**Depends on**: [034](034-bridge-failure-paths.md) (in progress in the coordinator's tick and
spawn paths; land the seam after it to avoid moving code twice); on top of 024's gap trace.
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

## Steps

- [ ] Inventory every CARLA call in csb (file, purpose, per-frame or per-episode) into an
      appendix of this doc; mark which become trait methods and which stay CARLA-only.
- [ ] Introduce the trait and `EntityHandle`; move map-frame conversion to the CARLA side.
- [ ] Move code into `CarlaBackend` in small commits, each passing `just test`.
- [ ] Coordinator talks only to `dyn SimBackend`; `carla::` appears nowhere outside
      `src/backend/carla/` (a test greps for it).
- [ ] A fake backend (`src/backend/fake.rs`, in-memory, deterministic) for unit tests of the
      coordinator's SSv2 semantics without CARLA.
- [ ] `backend` parameter in config and launch; unknown value fails at startup.
- [ ] Docs: architecture.md gains the seam; CLAUDE.md repository structure updated.

## Acceptance

- `just check` and `just test` pass; new coordinator tests run on the fake backend.
- The nine starter scenarios and both AWF suites pass on CARLA as before (020's proof run),
  and the gap trace of one scenario is identical in content to a pre-refactor run.
- Frame budget (016: 20 NPCs p95 1.79 ms) not worse by more than 10 %.
