# One SSv2 simulator interface, one glue per simulator

**Date**: 2026-10-22. **Status**: proposal (user direction 2026-10-22: SSv2 and the
simulators modular; csb is the CARLA glue, a second glue for Isaac Sim, both standalone
packages on one interface defined on SSv2's side). Builds on
[simulator-backends.md](simulator-backends.md) (the seam, roadmap 035) and the as-built
Isaac stack (037, 038, 043, 045). Nothing here is implemented yet.

## What exists

Roadmap 035 drew the seam as a Rust trait, `backend::SimBackend`, inside the one crate
`carla_scenario_bridge`, and said that "a later split (an SSv2 adapter crate plus one crate
per simulator) is a move, not a rewrite". The crate today, by role (lines of Rust):

| role | where | size | depends on a simulator? |
|---|---|---|---|
| SSv2 protocol | `proto/` (SSv2's schemas + `csb_world.proto`), `build.rs`, `proto.rs` | | no (SSv2's), except `csb_world` (Isaac's) |
| adapter: socket, frame budget, panics, drain | `zmq_server.rs` | 826 | no |
| adapter: request -> trait call -> reply, `Driver` rules | `coordinator.rs`, `backend/mod.rs`, `backend/fake.rs` | 883 | no |
| policy both backends use | `agent_link`, `autopilot`, `budgets`, `body_clearance`, `contact_report`, `coordinate_conversion`, `entity_manager`, `gap_trace`, `map_resolver`, `npc_tracking`, `weather`, `world_services`, `config` | ~6 900 | no in substance, **CARLA in name and detail** (below) |
| policy only the CARLA backend uses | `actor_census`, `blueprint_boxes`, `episode_clock`, `clock_store`, `epoch_marker`, `lanelet_map`, `sensor_release`, `spawn_ladder`, `tick_gate`, `tick_watchdog`, `frame_stats` | ~3 300 | mostly CARLA (acb's epoch marker, CARLA's tick, blueprints) |
| CARLA glue | `backend/carla/` (backend, collision monitor, traffic-light mapper, resolved signals, version, weather) | 7 600 | carla-rust |
| Isaac glue, Rust half | `backend/isaac/` (backend, tracking, tests) | 2 400 | `csb_world.proto` |
| Isaac glue, Python half | `src/csb_isaac/` (world server, lockstep world, NPC motion, physics layer, USD worlds, vehicle catalogue; 1 700 lines) + `scripts/` (Isaac install, town and vehicle export) | 1 700 | Isaac Sim, aib |

Around the crate: the SSv2 fork (`managed-ego-unforked`, 23 patches: the `agent` and
`simulator_autopilot` controllers, `UpdateEntityGoal`, the vehicle subtype in
`VehicleParameters`, bounded waits, `$(env)` in scenarios), the agent relay and `csb_agent`
(simulator-free), `csb_launch` (launch files for both simulators, `backend:=` chooses), the
justfile (CARLA and Isaac recipes side by side), one `bridge_config.yaml` holding both
simulators' keys, and the two vehicle bridges, already their own repositories: acb (CARLA)
and aib (Isaac, a submodule the world server imports).

## The gap

What stops csb and an Isaac glue from being two standalone packages on one interface:

1. **The interface is a trait inside one binary, not a package boundary.** Both backends
   are modules of `carla_scenario_bridge`; the backend is chosen by a ROS parameter at run
   time; the binary links carla-rust whichever backend runs. Nothing a third party could
   depend on exists: no crate publishes `SimBackend`, no package publishes the protocol.
2. **The shared policy is written in CARLA's terms.** `episode_clock` ("simulation time as
   CARLA reports it", continued by acb's epoch marker), `contact_report` ("the run's CARLA
   contacts"), `gap_trace` ("SSv2's world and CARLA's"), `map_resolver` ("a CARLA town name
   from the map path", `map_alias`), `budgets` ("every time bound the bridge puts on
   CARLA"), `world_services` (`/carla/set_weather`, `/carla/get_weather`), and
   `coordinate_conversion` (the Y-flip) are all used by `IsaacBackend` too, with the CARLA
   parts stepped around. 035 noted this and left it to 037; 037 lifted what Isaac needed
   without renaming.
3. **Weather leaks through the trait.** `SimBackend::set_weather` takes
   `weather::WeatherRequest` and returns `world_services::WeatherReply`, both CARLA's
   parameter set (`csb_interfaces/Weather.msg` is CARLA's `WeatherParameters`). Isaac
   answers `Unsupported`. A sky or lighting preset is the general notion; CARLA's fourteen
   floats are its own.
4. **One configuration file for two simulators.** `bridge_config.yaml` has `carla:`,
   `isaac:`, `blueprint_map`, `traffic_manager`, `carla_tick_seconds`, `substeps`,
   `collision_monitor`, `npc_motion` side by side; `simulation.launch.xml` carries
   `isaac_*` and `carla_*` arguments both. A glue cannot ship its own schema.
5. **The Isaac glue is in two halves joined by a private protocol.** `IsaacBackend` (Rust,
   in csb) and the world server (Python, in csb) talk `csb_world.proto`. That is a fine
   internal shape, but today the protocol, its generator (`gen_world_proto.sh`,
   `csb_world.desc`), the Rust client and the Python server are spread over `proto/`,
   `backend/isaac/` and `src/csb_isaac/`, and none of it can leave csb on its own.
6. **The protocol lives in the glue, not with SSv2.** `proto/` is a copy of SSv2's
   `simulation_interface` schemas plus the fork's additions. The fork and the copy drift
   independently; a second glue would copy them again. There is no package that says "this
   is the interface, version N".
7. **No conformance test for a glue.** The coordinator's tests run on `fake.rs`; each
   backend has its own unit tests; the only cross-simulator check is running the starter
   scenarios on both (`just acceptance ... backend:=`). Nothing says what a new backend must
   pass before it is a backend.
8. **Simulator-free Autoware-side pieces live in the CARLA repo.** aib depends on acb at
   run time for `acb_launch`'s Autoware profile and `acb_pilot` (the vehicle agent). The
   agent itself is already `csb_agent`'s (`acb_agent runs csb_agent's AutowareAgent`); the
   launch profile is not.
9. **Launch and run tooling are entangled.** `csb_launch` holds `isaac_world.launch.xml`
   beside the CARLA launches; the justfile holds `isaac-*` recipes, `isaac_up.sh`, the
   world-server stack; `run_suite`, `acceptance.py` and the gap oracle are simulator-free but
   sit in the CARLA package.

Not a gap: the vehicle bridges (acb, aib) are already one package per simulator with the
same Autoware-facing contract; the agent protocol and relay are simulator-free; the trait's
vocabulary (entities by SSv2 name, map-frame poses, batched calls per request,
`BackendError` as a failed reply) is right and both backends honour it.

## Proposed layering

Three layers, each a package someone else can depend on. "Glue" below is what the repo
calls a scenario bridge.

```
SSv2 (fork, pinned to the interface version)
   │ simulation_interface: ZMQ + protobuf          L0  ssv2_sim_interface   (protocol)
   ▼
adapter: socket, budgets, drain, coordinator,      L1  ssv2_sim_adapter     (library)
         entity table, Driver rules, agent link,
         autopilot commander, contacts, gap trace,
         episode clock, generic weather
   │ SimBackend (Rust trait, semver'd)
   ├── carla_scenario_bridge  = CarlaBackend + CARLA policy     L2  one binary per simulator
   └── isaac_scenario_bridge  = IsaacBackend + world server (Python)
agent relay + csb_agent SDK (L1, ROS packages, as today)
vehicle bridges: acb (CARLA), aib (Isaac)          unchanged: one per simulator
```

### L0: `ssv2_sim_interface`, the protocol

One package holding the `simulation_interface` protobuf schemas exactly as the SSv2 fork
builds them (the fork's `simulation_interface` package points at it, or vendors a copy with
a test that the two are byte-identical), with a Rust crate (`prost` types, today's
`proto.rs`) and the Python descriptor. Its version is the interface version: the fork's
23 patches define v1 (controllers `agent` and `simulator_autopilot`, `UpdateEntityGoal`,
the subtype, bounded waits). Upstreaming to tier4 is the long-term home; until then this
package is the spec, and a glue states which version it implements. `csb_world.proto` is
not here: it is the Isaac glue's internal wire, and moves with it.

### L1: `ssv2_sim_adapter`, the simulator-free core

A library crate (ament_cargo package, path dependency for the glues) that is everything in
the crate that knows no simulator, made to say so:

- `zmq_server`, `coordinator`, `SimBackend` and its types, `fake` (exported, for tests), and
  a `serve(config, backend)` entry that is today's `main.rs` minus the backend choice.
- The policy modules both backends use, renamed to what they are: `episode_clock`
  ("simulation time continued across reloads"; the epoch marker becomes a backend hook,
  `fn epoch_marker(&self) -> Option<...>`), `contact_report` (the report format; who
  produces contacts is the backend), `gap_trace` ("SSv2's world against the simulator's"),
  `map_resolver` (the map directory and its origin; the *world* for it is the backend's),
  `budgets` (SSv2's deadlines; the backend declares its own inner bounds),
  `coordinate_conversion` (the ROS-frame helpers; the Y-flip moves into the CARLA glue).
- Weather as a generic request: a preset name plus an optional `map<string, double>` of
  parameters, replied with what the backend applied; the ROS services become
  `/<node>/set_weather`, `/<node>/get_weather` on a generic message (`csb_interfaces`
  keeps `Weather.msg` for CARLA's parameter set, declared by the CARLA glue). The adapter
  re-applies "the requested weather after every map load and reconnect" as today, by name.
- Configuration split in two: the adapter's keys (`ssv2`, `agent_relay`, `npc_motion`,
  `gap_trace_dir`, `contact_report_dir`, `sensor_release`, `weather`) and a `backend:`
  subtree the glue defines and parses (`carla:`, `blueprint_map`, `traffic_manager`,
  `substeps`, `carla_tick_seconds`, `collision_monitor` for CARLA; `isaac:` for Isaac).
- The agent relay, `csb_agent`, `run_suite`, `acceptance.py`, the gap oracle, the starter
  scenarios (`csb_examples`) and the scenario launch (`scenario.launch.xml`): already
  simulator-free, they belong beside the adapter.
- A conformance kit: the coordinator's fake-backend tests restated as "what a backend must
  do" (spawn by name, echo or read back poses as its `Driver` says, refuse what it cannot
  do with `Unsupported`, report time monotonic across `initialize`, batched calls), plus the
  scenario-level check that the starter scenarios pass on it.

Cargo enforces the boundary: the adapter crate has no simulator dependency, so the
"`carla::` appears nowhere outside `backend/carla/`" test becomes the dependency graph.

### L2: one glue per simulator, standalone

Each glue is a binary crate depending on `ssv2_sim_adapter`, plus whatever that simulator
needs beside it, in its own package and, in time, repository:

- **`carla_scenario_bridge`** (csb keeps the name): `CarlaBackend`, `blueprint_boxes`,
  `actor_census`, `spawn_ladder`, `tick_gate`, `tick_watchdog`, `lanelet_map` (traffic
  light positions for the mapper), `sensor_release` (acb's channel), the CARLA weather
  presets and `Weather.msg`, carla-rust and its version pin, `carla_scenario.launch.xml`,
  `simulation.launch.xml` with CARLA arguments only, the CARLA recipes (`carla-start`,
  `ego-av`, `bg-av`, `demo-tour`), `download_maps.sh` and the town maps.
- **`isaac_scenario_bridge`** (new; "isb"): `IsaacBackend` and `tracking`, `csb_world.proto`
  renamed with it and its generator, the world server and its Python package
  (`csb_isaac` as is, renamed), the Isaac install, town and vehicle export scripts, the
  vehicle catalogue, `isaac_world.launch.xml`, an `isaac_simulation.launch.xml` (the
  bridge on Isaac, Isaac arguments only), `isaac_up.sh` and the `isaac-*` recipes, with aib
  as its submodule. The backend is chosen by which binary a launch starts, not by a
  parameter: the `backend` parameter goes.
- A third simulator is a third binary on the same crate; the conformance kit says when it
  is one.

The vehicle bridges stay one per simulator. The one simulator-free piece inside acb that
aib needs, the Autoware launch profile (`carla_simulator.launch.xml` minus `acb_bridge`),
moves next to `csb_agent` as a `scenario_vehicle_launch` package both profiles include, so
aib stops depending on acb.

### What the interface guarantees (unchanged, now written down)

Entities by SSv2 name; poses in the map frame at the entity origin, in SSv2's message types;
`Episode` carries SSv2's map path and the backend finds its world and origin for it; one
call per SSv2 request, batched as SSv2 batches; `Driver` decided by the adapter from the
controller, executed by the backend; every failure a `BackendError` with text, never a
success the simulator did not deliver (006); sensors answered by the adapter's policy
(refused: the vehicle bridge owns them); time reported by the backend, kept continuous by
the adapter; contacts in the adapter's report format from whatever the backend senses;
weather by preset name. The lifecycle hooks (`before_serving`, `probe_connection_idle`,
`wedged`, `server_lost`, `abandon_after_hang`, `shutdown`) stay: both backends use them.

## How to get there

Package-level first, repository-level second: the first is a move inside this workspace
that the existing tests and exit runs can prove changed nothing; the second is then a
`git mv` with history.

1. **Protocol package** `ssv2_sim_interface` (proto, build, Rust types, Python descriptor);
   the SSv2 submodule's `simulation_interface` checked against it by a test.
2. **Adapter crate** `ssv2_sim_adapter`: lift `zmq_server`, `coordinator`, `backend/mod.rs`,
   `fake`, the shared policy modules; rename as above; generic weather; config split;
   `serve()`; the conformance kit from the coordinator tests. `carla_scenario_bridge`
   becomes a binary crate using it with no behaviour change (the CARLA suite and the 033
   demo tour as the check).
3. **Isaac glue as a package** `isaac_scenario_bridge` in this workspace: `IsaacBackend`,
   `csb_world`, `csb_isaac`, launches, recipes; `backend:=` removed; the 043/045 exit runs
   as the check (`town01_ego_drive` 5/5, tracking within acceptance.py's limits).
4. **Vehicle launch profile** out of acb into `scenario_vehicle_launch`; aib's
   `isaac_simulator.launch.xml` includes it; acb's `carla_simulator.launch.xml` too.
5. **Repositories**: `isaac_scenario_bridge` to its own repository (aib as its submodule),
   then `ssv2_sim_adapter` + interface + relay + agent SDK + examples + run tooling to a
   `scenario_sim_adapter` repository that both glues pin; csb keeps CARLA. Each glue repo
   builds as acb does today (its own colcon workspace, the adapter as a submodule or a
   released crate).

Open points for the user: the names (`ssv2_sim_adapter` says what it is; the repo could keep
"csb" for the adapter and rename the CARLA glue instead); whether to upstream the interface
patches to tier4 before or after the split; whether the gap campaign (024-026, 039) wants the
gap oracle in the adapter repo or beside the scenarios.

## Risks

- **ament_cargo with path dependencies**: `cargo-ament-build` builds one Cargo package per
  colcon package; a library crate the glue depends on by path works when both are in one
  workspace (acb's workspace has several Rust packages), and across repositories needs the
  adapter vendored as a submodule or published. To be tried in step 2 before the rename.
- **Trait churn**: 044 is adding tracked NPCs, signals and contacts on Isaac, which touches
  `SimBackend` (contacts per entity, signal lamps). Do the split after 044's trait changes
  land, or 044 on the split crate; not both at once.
- **The renames are large diffs** over code with measured behaviour (budgets, episode clock,
  npc tracking). Rename without reflowing; keep each module's tests; re-run the CARLA
  suite and the Isaac exit runs after each step, not at the end.
