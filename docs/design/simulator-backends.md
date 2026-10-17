# Simulator Backends: SSv2 on CARLA and on Isaac Sim

**Date**: 2026-10-16. **Status**: design; phases 036 (bring-up) and 035 (the seam) done
2026-10-17. Phases [035](../roadmap/035-backend-seam.md)-[041](../roadmap/041-competition-director.md)
implement it. Background: [survey-isaac-sim-and-digital-twin.md](survey-isaac-sim-and-digital-twin.md)
(2026-10-09) and the Isaac Sim 6.1 study of 2026-10-16 (summarized under "Study notes").

## Goal

The same patched SSv2 (submodule `managed-ego-unforked`), the same scenarios and the same
Autoware stacks run against either CARLA or Isaac Sim, chosen per run. The repository keeps
the name carla-scenario-bridge (csb) for now; the simulator-specific parts are kept behind a
seam so that a later split (an SSv2 adapter crate plus one crate per simulator) is a move,
not a rewrite.

Why Isaac Sim: a sponsor requires it for an off-road driving competition. Its RTX sensors
(material-aware lidar, radar, path-traced cameras) and native Gaussian-splat rendering are
the technical gains; its missing scenario layer (no OpenSCENARIO, no traffic lights, no road
network) is what csb brings.

## What already does not depend on CARLA

Roadmap 017 split the system along three interfaces, and two of them are simulator-free:

| Interface | Between | Simulator-specific? |
|---|---|---|
| `simulation_interface` (ZMQ + protobuf, SSv2's) | SSv2 ↔ csb | No (SSv2 side); csb's handlers are |
| Agent protocol (TCP, JSON lines) | agent relay ↔ vehicle agent (`acb_pilot`) | No |
| Autopilot native (ADAPI, the ego's domain) | agent ↔ Autoware | No |
| Vehicle bridge (sensors, `/vehicle/status/*`, `/control/command/*`, `/clock`) | simulator ↔ Autoware | **Yes**: acb for CARLA |
| World control (spawn, pose, tick, lights, map) | csb ↔ simulator | **Yes**: carla-rust today |

So SSv2, the agent relay, `acb_pilot`, `run_suite`, the starter scenarios and the gap oracle
carry over. Two pieces are rewritten for Isaac: csb's world control (as a backend) and the
vehicle bridge.

## Architecture

```
SSv2 (scenario domain 9)                       Autoware ego k (domain k)
   │ simulation_interface (ZMQ)                    ▲ sensors, vehicle status, /clock
   ▼                                               │ control commands
csb  ── SSv2 handlers, entity table, episode clock, gap trace, contacts (simulator-free)
   │  SimBackend trait
   ├── CarlaBackend  ── carla-rust ──▶ CARLA server ──▶ acb_bridge per ego ──▶ domain k
   └── IsaacBackend  ── csb world protocol (ZMQ + protobuf, localhost)
                          ▼
              Isaac Sim 6.1 world server (standalone Python app, owns the loop)
                 ├── stage: world USD, entity prims, ego vehicles (PhysX vehicle)
                 ├── lockstep: Step(n) = n physics steps + one render, then reply
                 └── vehicle bridge per ego: ROS 2 context on domain k
                       (RTX sensors, /vehicle/status/*, /control/command/*, /clock)
agent relay (5560) ◀── agent protocol ──▶ acb_pilot per ego (unchanged)
```

### The seam in csb (phase 035, built 2026-10-17)

A Rust trait, `backend::SimBackend` (`src/backend/mod.rs`), is everything csb asks of a
simulator, one method per SSv2 request: `initialize(Episode)`, `update_frame`,
`update_step_time`, `spawn(SpawnRequest)`, `despawn(name)`,
`update_entity_status(StatusBatch) -> StatusOutcome`, `update_entity_goal(name, Goal)`,
`update_traffic_lights`, `attach_sensor`, the weather services, and the lifecycle
(`before_serving`, `probe_connection_idle`, `wedged`, `server_lost`, `abandon_after_hang`,
`shutdown`). Entities cross it by their SSv2 name; poses are map-frame, at the entity
origin, in SSv2's own message types. `BackendError::{Failed, Unsupported, Disconnected}`
each become a failed reply with the backend's text (006).

`coordinator.rs` decodes the protobuf requests into those calls -- including who drives a
vehicle (`Driver`: ego, scenario, `simulator_autopilot`, `agent`) -- and knows no simulator;
its tests run on an in-memory fake. `CarlaBackend` (`src/backend/carla/`) is the former
coordinator moved, with no behaviour change, and the only code that uses carla-rust (a test
checks). The backend is chosen by the ROS parameter `backend` (`carla` default, `isaac`
refused until 037).

Calls are **per request and batched**: `UpdateEntityStatus` carries every puppeteered pose
in and every state out, `UpdateFrame` advances one frame, as SSv2 sends them. CARLA works
this way already (one snapshot per frame, 024's cost rule); Isaac must, because a per-entity
round trip would cost more than the frame.

What 035 left in `CarlaBackend` because it is built on CARLA specifics -- the episode clock
and acb's epoch marker, contact reports from CARLA's collision sensors, the gap trace on
CARLA's snapshots, blueprint geometry -- is 037's to lift where the Isaac backend needs the
same (roadmap 035, "As built").

### The Isaac world server (phase 037)

A standalone Isaac Sim Python application (`csb_isaac/world_server.py`, run under Isaac's
Python), not a GUI extension: it owns the simulation loop, so lockstep is natural. It waits
for a request, applies it, steps and replies, as CARLA does in synchronous mode. It can run
headless or on a VNC display for inspection. Phase 036 measured this loop: deterministic
(bitwise-identical states from fresh servers), 6.8 ms median per 10 Hz frame with 20
puppeteered NPCs and 6 physics substeps, 4.3 ms without rendering.

The world protocol is ours, small and versioned (`proto/csb_world.proto`): `Hello` (versions),
`LoadWorld` (USD path, map origin), `Spawn`, `Despawn`, `Step` (poses in, n physics substeps,
states, contacts and sim time out), `SetSignals`, `SetWeather` (where supported). Isaac's own
ROS 2 simulation-control services (`isaacsim.ros2.sim_control`, the ROS `simulation_interfaces`
standard) cover world loading, spawning and stepping too, and the competition director (041)
uses them for operator actions; csb does not, because they set one entity per call and step
only while paused.

Frames: Isaac's world is right-handed and Z-up, the same convention as ROS, so the CARLA
Y-flip disappears. The map frame is the world frame plus the map origin offset (the same
role `carla/map_origin.yaml` plays for CARLA).

NPCs are kinematic rigid bodies positioned every frame (the AWSIM pattern this repo uses for
CARLA); the ego is a PhysX vehicle (Ackermann steering, drive, suspension, tire friction per
material). Contacts come from PhysX contact reports on the ego.

### The Isaac vehicle bridge (phases 036, 038)

Lives inside the world server, because the sensors are rendered there. Per ego it creates a
ROS 2 context on that ego's domain and publishes what acb publishes for CARLA: lidar, cameras,
IMU, GNSS, `/vehicle/status/*` (velocity, steering, gear, control mode, turn indicators),
`/clock`; it subscribes to `/control/command/*`. Isaac's `ROS2 Context` node takes a
`domain_id` per context, and several contexts coexist in one stage: verified in 036 (each
domain sees only its own vehicle, with Cyclone under the stack's localhost config and with
Fast DDS). The rclpy fallback (an rclpy context per ego) was not needed. Isaac bundles ROS 2
Humble libraries built for its Python 3.12; the system Humble is not sourced into it.

### What Isaac does not have, and how csb answers it

| SSv2 / Autoware need | CARLA | Isaac answer |
|---|---|---|
| Traffic lights | CARLA's lights, frozen and set per command (009, 015) | None in the off-road ODD: `UpdateTrafficLights` replies with an honest failure (006) unless a scenario world provides signal prims (later) |
| Road network for SSv2 | Lanelet2 map alongside the town | The same: SSv2 routes on the Lanelet2 map; the USD world only has to be aligned to it |
| NPC traffic | SSv2 puppeteers; Traffic Manager unused | The same |
| Entity geometry | Blueprint table (022) | USD asset per subtype, scaled to the declared box |
| Map loading | `load_world` by town name | `LoadWorld` by USD path, resolved from the scenario's map dir |
| Weather | `/carla/set_weather` | Lighting and sky presets, later; UNSUPPORTED until then |
| Simulation clock | CARLA's, published by acb in every domain (015) | Isaac's sim time, published by the vehicle bridge in every ego domain |

## Risks

- ~~Several ROS domains in one Isaac process~~: works (036), Cyclone included; both sides must
  use the same RMW.
- **GPU budget.** About 5 ms and 0.26 GB per full sensor set (128-channel lidar + 720p camera)
  per 20 Hz frame on a light world (036): eight sets in real time on one RTX 5090. To be
  remeasured on the NTU mesh (040) and with the full sensor kit (039).
- **API churn.** Isaac 6.x moved sensor APIs to `isaacsim.sensors.experimental.rtx` (lidar
  profiles are now USD variants, e.g. `config="OS1"`, `variant="OS1_REV7_128ch20hz1024res"`);
  pin the Isaac version per phase and record it. 036 runs Isaac Sim 6.1.0 (Kit 110.3.0) on
  driver 580.105.08, below NVIDIA's tested 595.
- **Off-road physics.** Rigid terrain with friction zones only; deformable terrain is
  experimental (Newton solvers in 6.1).

## Study notes (2026-10-16)

- Isaac Sim 6.1 is current (6.0 GA June 2026: NuRec built in, Kit 110; 6.1: Newton 1.5).
- ROS 2 domain per context: `ROS2 Context` node inputs `domain_id` and `useDomainIDEnvVar`
  (default true: the environment variable wins).
- Simulation control: `isaacsim.ros2.sim_control` 1.6.8, 19 services + `SimulateSteps` action
  (state, step, reset, spawn, delete, get/set entity state, load/unload world, spawnables).
- No OpenSCENARIO anywhere in Isaac; NVIDIA's partner route is Foretellix Foretify on the
  Omniverse AV blueprint (commercial). SSv2 through csb is our OpenSCENARIO.
- Real-site scenes: the NTU mesh comes from a vendor (decision 2026-10-16); Gaussian splats
  render natively (ParticleField recommended over NuRec USDZ), camera only; RTX lidar and
  wheels need the mesh. Keep coordinates near the origin (precision loss far from it).
- Remote viewing: livestream allows one client per instance and has no authentication.
- The Autoware off-road WG simulator (`autowarefoundation/autoware_off-road_sim`) isolates
  vehicles by topic prefix, has no scenario or reset API and no race management; its
  per-vehicle topic remapping is worth reusing.
