# Architecture

## Overview

`carla-scenario-bridge` is a ZMQ+Protobuf server that implements the `simulation_interface` protocol defined by [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) (SSv2). It translates SSv2's 14 protobuf request types into CARLA API calls, making CARLA a drop-in replacement for SSv2's built-in `simple_sensor_simulator` or AWSIM.

## System Context

```
SSv2 openscenario_interpreter
    | parses .xosc, drives scenario logic
    v
traffic_simulator (NPC behavior, Lanelet2 routing)
    | ZMQ + Protobuf (tcp://host:SSV2_PORT)
    v
+----------------------------------------------------------+
|  carla-scenario-bridge                                    |
|                                                           |
|  ZMQ Server (REP socket)                                  |
|  +-- Entity Manager (SSv2 name <-> CARLA actor ID)        |
|  +-- Traffic Light Mapper (lanelet signal ID <-> actor)   |
|  +-- Coordinate Converter (ROS <-> CARLA frames)          |
|  +-- Frame Sync (UpdateFrame -> world.tick())             |
+-----------------------------+----------------------------+
                              |
                              v
                         CARLA Server (sync mode, port CARLA_PORT)
                              |
                              v
                    CARLA sensors (lidar, camera, IMU, GNSS)
                              |
                              v
                    autoware_carla_bridge (existing, separate process)
                    +-- Sensor callbacks -> ROS 2 topics
                    +-- Vehicle status -> /vehicle/status/*
                    +-- Control commands -> apply_control()
                    +-- Clock -> /clock
                              |
                              v
                         Autoware (full perception pipeline)
```

## Design Principles (Following AWSIM Precedent)

### 1. SSv2 Puppeteers NPCs

SSv2's `traffic_simulator` computes all NPC positions using its behavior plugins. The adapter places NPCs at those positions via `set_transform()`. CARLA's own Traffic Manager is not used for scenario-controlled actors.

NPCs are **kinematic** (teleported each frame), not physics-driven. This means NPC-to-NPC collisions are not simulated by CARLA, same as AWSIM. SSv2 handles collision detection via bounding box checks.

### 2. CARLA Owns Ego Physics

The ego vehicle uses CARLA's PhysX engine. Autoware publishes control commands; `autoware_carla_bridge` translates them to `apply_control()`. The adapter reads the ego's actual pose from CARLA and reports it back to SSv2 via `UpdateEntityStatusResponse`.

SSv2's built-in ego vehicle models (bicycle model variants in `EgoEntitySimulation`) are NOT used. The `overwrite_ego_status` flag in `UpdateEntityStatusRequest` is respected for teleport/manual mode.

### 3. CARLA Sensors, Not SSv2 Pseudo Sensors

All `AttachSensor` requests return `Success=false`. CARLA's own sensors (GPU lidar, rendered camera, IMU, GNSS) are spawned by `autoware_carla_bridge` and publish directly to ROS 2 topics. Autoware processes real sensor data through its full perception pipeline (not ground-truth pseudo detection).

### 4. SSv2 Controls Traffic Lights

CARLA's built-in traffic light cycling is frozen. The adapter maps SSv2 lanelet signal IDs to CARLA `TrafficLight` actors and sets their states per `UpdateTrafficLights` commands.

### 5. SSv2 Drives Autoware Lifecycle

SSv2's `FieldOperatorApplication` (concealer) launches Autoware, initializes localization, sets routes, and engages autonomous mode. This replaces the `auto_drive.py` pilot script from `autoware_carla_bridge`.

SSv2's `AutowareUniverse` (concealer) must be **disabled** for vehicle status topics, since `autoware_carla_bridge` publishes them from real CARLA state. The `simulate_localization` parameter must be `false` so Autoware runs its real GNSS->NDT pipeline.

## ZMQ Protocol: 14 Message Handlers

| # | Request | CARLA Action | Response |
|---|---------|-------------|----------|
| 1 | `Initialize` | Connect to CARLA, set sync mode, load map metadata | `Result` |
| 2 | `UpdateFrame` | `world.tick()` | `Result` |
| 3 | `UpdateStepTime` | Update tick rate | `Result` |
| 4 | `SpawnVehicleEntity` | `world.spawn_actor()` with vehicle blueprint | `Result` |
| 5 | `SpawnPedestrianEntity` | `world.spawn_actor()` + attach AI walker controller | `Result` |
| 6 | `SpawnMiscObjectEntity` | `world.spawn_actor()` with static prop | `Result` |
| 7 | `DespawnEntity` | `actor.destroy()` | `Result` |
| 8 | `UpdateEntityStatus` | NPCs: `set_transform()`; Ego: read pose from CARLA | Updated poses |
| 9 | `AttachLidarSensor` | `Success=false` (CARLA sensors via bridge) | `Result` |
| 10 | `AttachDetectionSensor` | `Success=false` | `Result` |
| 11 | `AttachOccupancyGridSensor` | `Success=false` | `Result` |
| 12 | `AttachImuSensor` | `Success=false` | `Result` |
| 13 | `AttachPseudoTrafficLightDetector` | `Success=false` | `Result` |
| 14 | `UpdateTrafficLights` | `freeze()` + `set_state()` per signal | `Result` |

## Entity Manager

Maps SSv2 entity names to CARLA actor IDs:

```
SSv2 name     CARLA actor    Type        Physics
---------     -----------    ----        -------
"ego"         vehicle #42    EGO         CARLA PhysX (apply_control)
"npc_car_1"   vehicle #58    VEHICLE     Kinematic (set_transform)
"ped_1"       walker #71     PEDESTRIAN  Kinematic (set_transform)
"barrier_1"   prop #83       MISC        Static (set_transform)
```

### Blueprint Mapping

SSv2 sends `asset_key` (e.g., `"vehicle.volkswagen.t2"`). If it matches a CARLA blueprint directly, use it. Otherwise, maintain a mapping table from SSv2 model names to CARLA blueprints.

## Coordinate Conversion

CARLA uses a left-handed coordinate system (Y = right). SSv2/ROS uses right-handed (Y = left). The conversion is a Y-axis flip on positions and corresponding rotation adjustments.

This is identical to the conversion already implemented in `autoware_carla_bridge`'s `coordinate_conversion.rs`.

## Interaction with autoware_carla_bridge

Both processes connect to the same CARLA server. Responsibilities are split:

| Responsibility | carla-scenario-bridge | autoware_carla_bridge |
|---|---|---|
| Entity spawn/despawn | Yes | No (hero vehicle pattern removed) |
| NPC pose updates | Yes (set_transform) | No |
| Ego pose readback | Yes (for SSv2 response) | Yes (for vehicle status) |
| Sensor attachment | No (returns false) | Yes (spawns CARLA sensors) |
| Sensor data -> ROS 2 | No | Yes |
| Vehicle status -> ROS 2 | No | Yes |
| Control commands -> CARLA | No | Yes (actuation_cmd -> apply_control) |
| Clock publishing | No | Yes |
| Traffic light control | Yes (freeze + set_state) | No |
| CARLA world.tick() | Yes (via UpdateFrame) | No (passive wait_for_tick) |

The bridge must switch from active `world.tick()` (current `demo_scenario.py` role) to passive `wait_for_tick_or_timeout()` since this adapter now owns the tick.
