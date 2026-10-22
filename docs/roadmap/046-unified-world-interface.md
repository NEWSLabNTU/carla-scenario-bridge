# Phase 046: One World Interface, then the Rename

Make every simulator a world server behind one protocol, with CARLA and Isaac Sim as its two
implementations, prove both still run what they ran, and only then rename the project and
its packages away from "carla". Design: [design/simulator-interface-split.md](../design/simulator-interface-split.md)
(the revised layering: one way in, the world protocol).

**Source**: user direction 2026-10-22 ("change the csb name because it has carla in it;
refactor csb first to have the unified interface and test it against CARLA and Isaac Sim;
then rename csb, both in package name and in the code").
**Depends on**: [044](044-isaac-npcs-and-traffic-lights.md)'s trait changes (landed
2026-10-22; its open items do not block); [035](035-backend-seam.md) (the seam this replaces).
**Feeds**: [039](039-isaac-multi-autoware.md), which then adds its protocol fields to the
unified interface instead of to Isaac's private one; the repository split (design step 8),
a later phase.

Two parts, in order, never interleaved:

- **Part A, unify.** All in this repository, under today's names. CARLA moves out of the
  bridge into its own world server process; the bridge keeps one backend that speaks the
  world protocol. No behaviour change on either simulator, measured.
- **Part B, rename.** Mechanical: package, crate, module, node, service, environment and
  repository names. No behaviour change, checked by a shorter run.

Not in this phase: splitting repositories, moving the Autoware launch profile out of acb
(`scenario_vehicle_launch`), a third simulator. The design lists them; each is its own phase.

## Where it starts (2026-10-22)

| | today |
|---|---|
| bridge | one crate `carla_scenario_bridge`, links carla-rust; `backend` ROS parameter picks `CarlaBackend` (in process, 6 100-line `backend.rs` plus 5 modules) or `IsaacBackend` (ZMQ to the world server) |
| world protocol | `proto/csb_world.proto`, version 3, private to Isaac: `usd` field, `isaac_version`, 60 Hz substep constant in the bridge, motions `KINEMATIC` and `TRACKED` only, no goals, no weather |
| Python server | `src/csb_isaac`: `server.py` and `protocol.py` simulator-free, `world_server.py` Isaac's entry, `world.py` and the rest Isaac's |
| CARLA-only modules in the shared crate | `actor_census`, `blueprint_boxes`, `clock_store`, `epoch_marker`, `lanelet_map`, `sensor_release`, `spawn_ladder`, `tick_gate`, `tick_watchdog`, the Y-flip in `coordinate_conversion` |
| map files | `<map>/carla/traffic_lights.yaml` (read by both), `<map>/carla/map_origin.yaml`, `<map>/isaac/world.yaml`; CARLA's town from the directory name and `map_alias` |
| weather | ROS `/carla/set_weather`, `/carla/get_weather` on CARLA's `Weather.msg`; Isaac `Unsupported` |
| run tooling | `backend:=carla|isaac` in `scenario`, `acceptance`, `e2e`, `run`; `isaac_up.sh` brings up Isaac's stacks; CARLA's are brought up by hand or `demo.launch.xml` |

## Design (proposed, for the user's review)

### The protocol, version 4

`csb_world.proto` becomes the simulator interface. Changes against v3, every one additive
except the two renames, which the version bump covers:

| message | change |
|---|---|
| `HelloReply` | `isaac_version` renamed `simulator_version`; adds `simulator_name` (`carla`, `isaac`: the `<map>/<name>/world.yaml` directory) and `capabilities` (`SIGNALS`, `AUTOPILOT`, `AGENT_VEHICLES`, `TRACKED_MOTION`, `WEATHER`, `CONTACTS`) |
| `LoadWorld` | `usd` renamed `world` (the server's own string from `world.yaml`: a USD path, a town name); adds `step_time`, `realtime_factor` from SSv2's `Initialize`, and `map_dir` |
| `Spawn` | `Motion` gains `EXTERNAL`: a vehicle the simulator's physics moves, driven by its own vehicle bridge or by the simulator's autopilot (today's `Driver::Ego`, `Agent`, `SimulatorAutopilot`), read back every step; adds `role_name` (CARLA's acb attaches by it) |
| `Step` | adds `substep_dt`: the bridge sends it, the server stops assuming 60 Hz |
| `StepReply` | unchanged; `time` stays the server's and monotonic over its lifetime |
| `SetGoal` (new) | entity handle, waypoints, `clear`, `target_speed`: `UpdateEntityGoal` for an `AUTOPILOT` vehicle |
| `SetWeather`, `GetWeather` (new) | a preset name and a `map<string, double>` of parameters; the reply carries what was applied |

Rules the protocol states, so both servers keep them: entities by handle, the bridge keeps
the SSv2 name; poses at the entity origin in the server's world frame, right-handed, Z up
(the CARLA server flips Y inside itself); the bridge applies `map_origin`; a refusal is
`ok = false` with text, never a success that did not happen (006); a request outside the
declared capabilities is refused by the bridge without being sent.

### The bridge after part A

`carla_scenario_bridge` keeps its name until part B and loses every simulator:

- `IsaacBackend` becomes `RemoteBackend`, the only `SimBackend`; `tracking.rs` and
  `signals.rs` go with it. `SimBackend` stays as the coordinator's internal seam (the fake
  backend still tests the coordinator).
- The coordinator checks `capabilities` before a gated call; `Driver::of_vehicle` demotes
  `simulator_autopilot` to `Scenario` with a logged reason when `AUTOPILOT` is absent.
- Time continuity across `Initialize` (`episode_clock`) stays in the bridge, fed by the
  server's monotonic time.
- Weather: generic ROS services on the bridge node (`~/set_weather`, `~/get_weather`, a new
  `csb_interfaces/WorldWeather` message: preset plus key-value parameters), forwarded as
  `SetWeather`. The bridge re-applies the requested weather after every `LoadWorld` and
  reconnect, as today. `/carla/set_weather` keeps working through part A as an alias that
  maps CARLA's fields to the generic parameters, and is removed in part B.
- `bridge_config.yaml` keeps the adapter's keys and gains `world: {endpoint,
  request_timeout_seconds, load_timeout_seconds}`; `carla:`, `isaac:`, `blueprint_map`,
  `traffic_manager`, `substeps`, `carla_tick_seconds`, `collision_monitor`,
  `npc_contact_sensors`, `sensor_release`, `tick_gate` move to the servers' configs. The
  bridge refuses a config that still carries one, naming where it went (the retired-key
  pattern `CSB_RETIRED_ENV` uses for the environment).
- `Cargo.toml` loses `carla`; a test checks the dependency graph has no simulator crate.
- The bridge's frame budget and wedge handling stay: a world request that misses its
  timeout wedges the bridge, which drains and exits 75, as with Isaac today.

### The CARLA world server

A new Rust package in this workspace, `carla_world` (renamed in part B only if the prefix
changes), a binary that serves the world protocol on carla-rust:

- A small library crate `csb_world_server` (Rust shell): the REP socket loop, request
  decode, `Hello` and version check, and a `World` trait with the protocol's methods.
  `carla_world` implements it; it is also the shell for any future Rust server.
- Today's `CarlaBackend` moves in, with `blueprint_boxes`, `actor_census`, `spawn_ladder`,
  `tick_gate`, `tick_watchdog`, `clock_store`, `epoch_marker`, `sensor_release`,
  `lanelet_map`, the collision monitor, the traffic-light mapper and resolved signals, CARLA
  weather presets, the version check and the Y-flip. It is re-cut from "SSv2 handler" to
  "world method" (`Step` instead of `update_entity_status` then `update_frame`), keeping
  each module's code and tests.
- **Time**: CARLA's frame time restarts on a map load; `carla_world` continues it (today's
  `episode_clock` and epoch marker logic, which acb reads for `/clock`) so its `StepReply`
  time is monotonic over the server's life, as the protocol requires.
- **Hung tick** (034): the tick watchdog and the `_exit(75)` move with the tick. The server
  exits; the bridge's request times out, it drains and exits 75 too; both are respawned.
  The bridge's request timeout must exceed the server's tick bound, and the launch says so.
- **CARLA client lifetime** (CLAUDE.md): the idle probe and "exit when the CARLA connection
  died" move with the client. The bridge holds no CARLA connection any more.
- **Sensor release** (acb's channel, `ssv2_port + 1`, `+ 2`): served by `carla_world`. The
  ports keep their defaults and their derivation from `SSV2_PORT`, so acb changes nothing.
- **Signal table**: `carla_world --generate-signal-table <map dir>` replaces the bridge's
  flag; it writes `<map>/traffic_lights.yaml` (below).
- Launch: `carla_world.launch.xml` (respawned, its own web port), started by
  `scripts/carla_up.sh`, which also starts or reuses CARLA itself (`just carla-start`), the
  simulation side and `just ego-av`, as `isaac_up.sh` does for Isaac.

### The Isaac world server

- `src/csb_isaac` splits: the simulator-free shell (`server.py`, `protocol.py`, the
  descriptor set, a `World` abstract base class, `main(world_factory)`) moves to a new
  Python package `csb_world_server`; Isaac's `world.py` and its modules stay in `csb_isaac`
  and subclass `World`. The shell stays pure Python and reads the descriptor set, not
  generated code, so Isaac's protobuf runtime and the host's both load it (037's reason).
- Isaac-specific constants leave the bridge: the 60 Hz substep comes from the server's
  config and is reported in `HelloReply`; `render` becomes a server option.

### The conformance check

`csb_world_conformance <endpoint>` (Python, in `csb_world_server`): a wire-level client
that runs against any running server. Groups: hello and version; load; spawn by name and
kind; kinematic pose echo; step and read back; tracked motion (if declared); external
vehicle read back (if declared); contacts between two overlapping bodies (if declared);
signals set and read back from the server's own state (if declared); goal accepted (if
`AUTOPILOT`); weather round trip (if declared); time monotonic across two loads; despawn of
an unknown handle refused; a request after `Shutdown` refused. It replaces neither the
coordinator's unit tests nor the scenario runs; it is the check a server passes before a
scenario is worth running.

### The map contract

`<map>/<simulator_name>/world.yaml` is read by the bridge, which takes `map_origin` and
sends the rest as `LoadWorld.world`:

- `carla/world.yaml`: `town` (replaces the directory-name rule and `map_alias`; absent, the
  directory name, so the TUM towns need no file), `map_origin` (replaces
  `carla/map_origin.yaml`).
- `isaac/world.yaml`: unchanged (`usd` read as `world` for one release).
- `<map>/traffic_lights.yaml`: moved up from `carla/`; schema unchanged.

Fallbacks for one release: the old file names are read with a warning naming the new one.
acb's `map_origin_path` and `traffic_light_map_path` defaults follow (an acb commit, pushed
before the pin moves).

## Steps

### Part A

1. **Baseline.** On `main` before any change, both simulators, `just build` fresh, acb
   rebuilt (CLAUDE.md order), stacks started clean:
   - CARLA: `scenarios/bench/town01_npc_20.xosc` 3x with the gap trace on (035's
     comparison); frame budget p50 / p95 from `frame_stats`; `basic/` + `tutorial/`,
     `multi_av/town01_two_av`, AWF UC-ACC and UC-AEB; one demo tour.
   - Isaac: 043's `town01_ego_drive` 5x with `acceptance.py`'s tracking numbers; 044's exit
     set (`town01_traffic_light`, incidents, `UC-AEB-001`); the 20-NPC bench with the trace.
   Record commits, verdicts, times and numbers in this doc. Everything after compares to it.
2. **Protocol v4** in `csb_world.proto`, with both halves of today's Isaac path moved to it
   in one commit (`PROTOCOL_VERSION` 4 on both sides). The CARLA path is untouched.
   Check: `just test`; Isaac's `town01_ego_drive` 1x.
3. **Python shell** `csb_world_server` out of `csb_isaac`; `World` base class; Isaac's world
   subclasses it. **Conformance CLI** written against it and run on a fake `World` in CI and
   on the Isaac server by hand. Check: conformance passes on Isaac; Isaac 1x.
4. **`RemoteBackend`**: `IsaacBackend` renamed and stripped of Isaac constants; capability
   gating in the coordinator; driver demotion. Check: `just test` (new coordinator tests for
   gating on the fake); Isaac 1x.
5. **Rust shell** `csb_world_server` crate and the **`carla_world` binary**: `CarlaBackend`
   and the CARLA-only modules moved, re-cut to world methods; episode clock, hung tick, idle
   probe, sensor release, collision monitor, signals, Traffic Manager goals, weather inside.
   Built beside the old bridge; nothing launches it yet. Check: unit tests moved and
   passing; conformance passes against `carla_world` on a live CARLA.
6. **Switch CARLA over.** The bridge's CARLA path becomes `RemoteBackend` to `carla_world`;
   `CarlaBackend` deleted from the bridge; `carla` removed from its `Cargo.toml`;
   `carla_world.launch.xml`, `carla_up.sh`; `simulation.launch.xml` takes the world endpoint;
   `demo.launch.xml` and `two-av`, `demo-tour`, `bg-av`, `ego-av` start or require
   `carla_world`. Check: the CARLA baseline set (step 1) in full.
7. **Config split and generic weather.** Retired keys refused with their new home; the
   servers' configs; the generic weather services, `/carla/*` kept as an alias. Check:
   `just test`; weather set and read back on both simulators by service call.
8. **Map contract.** `world.yaml` per simulator, `traffic_lights.yaml` moved up, fallbacks
   with warnings; acb defaults (acb commit first); `download_maps.sh` and the generator
   write the new layout. Check: both simulators on Town01 and Town10HD (the alias case).
9. **One run interface.** `backend:=` becomes `sim:=` in `scenario`, `acceptance`, `e2e`,
   `run`; each simulator's `up.sh` is found by convention; `backend:=` refused naming
   `sim:=`. Check: `just e2e sim:=carla` and `just e2e sim:=isaac`.
10. **Part A exit runs**: the step 1 set again on both simulators, compared (Acceptance).
    Docs: architecture.md, simulator-backends.md, user-guide.md, CLAUDE.md's structure and
    process notes (a fourth CARLA-side process to account for).

### Part B

11. **Name.** User's decision, before any rename commit. Proposal below.
12. **Rename inside this repository**, one commit per kind so each is reviewable and
    revertible: Cargo crates and binaries; colcon packages and `package.xml`s; Python
    packages; launch files and their arguments; ROS node and service names (the `/carla/*`
    alias removed); the proto package; config file names; environment variables (`CSB_*`
    to the new prefix, the old names added to `CSB_RETIRED_ENV` so recipes refuse them,
    naming the new one; `.env` on each host updated by its user); log directory kinds;
    docs and CLAUDE.md. History is kept (`git mv`), and code is renamed without reflowing.
13. **Submodules.** acb and aib refer to csb's packages (acb: 38 files, mostly docs and
    `csb_agent` imports; aib: 8). Commit and push each submodule on its tracked branch
    first, then move the pins here (CLAUDE.md order).
14. **Repository rename** on GitHub (outward-facing: the user does it or confirms it;
    GitHub redirects the old URL). Update `.gitmodules` URLs in acb and aib that point here,
    README badges and links.
15. **Part B check**: `just check`, `just test`, a clean `just build` from an empty
    `install/`, then `just e2e sim:=carla`, `just e2e sim:=isaac`, the CARLA 20-NPC bench
    gap trace identical to step 10's, and `multi_av/town01_two_av` on CARLA.

## The name (step 11, proposal)

The project is a bridge between SSv2 and any simulator, through one world protocol. Candidates,
all without a simulator in them:

| repository | Rust and colcon prefix | short | note |
|---|---|---|---|
| `scenario-world-bridge` | `swb_`, bridge `scenario_world_bridge` | swb | says what it bridges; recommended |
| `ssv2-sim-bridge` | `ssb_` | ssb | ties the name to SSv2, which the project depends on anyway |
| `scenario-sim-adapter` | `ssa_` | ssa | the design doc's word for L1 |

What keeps its name: `scenario_agent_relay` (already simulator-free), acb and aib (their own
repositories), `carla_world` (CARLA's server, rightly named), `csb_isaac` becomes
`isaac_world`. Design-doc names (`ssv2_world_bridge`, `ssv2_world_protocol`) follow the
chosen prefix.

## Risks

- **The CARLA hop.** Every frame gains a local ZMQ round trip and a protobuf encode of
  every entity. Isaac measured 3.7 ms p95 round trip with 20 NPCs including its step (037);
  the hop alone should be well under 1 ms. If the budget is worse by more than step 1's
  bound, profile before accepting.
- **Re-cutting `CarlaBackend`** (6 100 lines) from two SSv2 calls per frame to one `Step`
  can move when a pose is applied relative to the tick, which the gap trace would show as a
  one-frame shift. The identical-trace check (035's) is the guard; a shift is a bug, not a
  tolerance.
- **One more process to supervise on CARLA.** `carla_world` and the bridge both respawn;
  a wedge now takes two restarts. CLAUDE.md's process accounting gains `carla_world`.
- **Isaac work in flight.** 044's open items and 039 touch the Isaac path. Land part A step
  2-4 quickly, or hold Isaac protocol changes until step 4 is in.
- **The rename touches every host's `.env`** and every user's muscle memory. The retired-env
  refusal names the new variable; the user guide lists the mapping once.

## Acceptance

Part A:

- `just check` and `just test` pass; the bridge crate has no simulator dependency (test).
- The conformance CLI passes against `carla_world` and the Isaac world server.
- CARLA: the 20-NPC bench's gap trace identical in content to step 1's (server counters set
  aside, as in 035); frame processing p95 not worse than step 1's by more than 10 %, and the
  world round trip p95 recorded; `basic/` + `tutorial/`, `town01_two_av`, UC-ACC, UC-AEB
  with the same verdicts as step 1; one demo tour end to end.
- Isaac: `town01_ego_drive` 5/5 with tracking within `acceptance.py`'s limits; 044's exit
  set with step 1's verdicts; the 20-NPC bench trace identical to step 1's.
- The run tooling knows no simulator by name except through `sim:=` and `<sim>/up.sh`.

Part B:

- No package, crate, module, binary, node, service or variable this repository owns
  carries `carla` except `carla_world` and CARLA-specific code inside it (a grep in CI,
  with that allowlist), and none carries `csb`.
- Step 15's runs pass; acb and aib build against the renamed packages from their own
  workspaces.
