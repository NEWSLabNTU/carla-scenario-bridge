# Phase 4: Traffic Lights + Environment

Scenario-controlled traffic lights and weather conditions. Optionally, CARLA's Traffic Manager provides ambient background traffic.

## Design

### Traffic Light Control

SSv2 sends `UpdateTrafficLights` with a list of `TrafficSignal` messages, each containing:
- `id`: Lanelet signal ID (from the Lanelet2 map)
- `traffic_light_status[]`: List of bulb states (color, shape, status)

The adapter must map these lanelet IDs to CARLA `TrafficLight` actors and set their visual states.

**Mapping challenge**: Lanelet2 and OpenDRIVE reference traffic lights differently. Lanelet2 uses regulatory element IDs referencing traffic light linestrings. OpenDRIVE uses signal IDs at specific road positions. CARLA creates `TrafficLight` actors from OpenDRIVE signals at map load time.

**Mapping strategies** (in order of implementation priority):

1. **Position-based matching**: At `Initialize`, enumerate all CARLA `TrafficLight` actors and their positions. Load the Lanelet2 map (path from `InitializeRequest.lanelet2_map_path`). Match lanelet traffic light positions to nearest CARLA actors within a tolerance (e.g., 5m). This handles most cases where the Lanelet2 map was generated from the same OpenDRIVE.

2. **Manual mapping file**: A YAML/CSV file mapping `lanelet_signal_id -> carla_actor_id` for each map. Generated once per map, checked into `config/`.

3. **Lane-to-lanelet CSV** (Phase 5): Use the mapping from [AWF odd-usecase-scenario#118](https://github.com/autowarefoundation/odd-usecase-scenario/issues/118) to derive signal correspondence automatically.

**State conversion**: SSv2 traffic light states map to CARLA states:

| SSv2 Color | SSv2 Shape | SSv2 Status | CARLA State |
|---|---|---|---|
| GREEN | CIRCLE | SOLID_ON | `Green` |
| RED | CIRCLE | SOLID_ON | `Red` |
| AMBER | CIRCLE | SOLID_ON | `Yellow` |
| * | * | SOLID_OFF | `Off` |
| GREEN | LEFT_ARROW | SOLID_ON | `Green` (CARLA doesn't distinguish arrows) |

**CARLA traffic light lifecycle**:
1. On `Initialize`: freeze all traffic lights (`traffic_light.freeze(True)`) to prevent CARLA's built-in cycling from interfering
2. On `UpdateTrafficLights`: set each referenced light's state
3. On shutdown: unfreeze all traffic lights (restore normal cycling)

### Weather / Environment

SSv2 currently has **no `EnvironmentAction` support** (noted as a gap in the AWF evaluation). However, OpenSCENARIO defines `EnvironmentAction` for weather control. When SSv2 adds support, it would likely come as a new protobuf message type or a `CustomCommandAction`.

For now, implement weather control as a **custom extension**:
- Accept weather parameters via a config file or ROS parameter
- Optionally, handle SSv2 `CustomCommandAction` with a weather payload
- Map to CARLA's `world.set_weather(WeatherParameters(...))`:
  - `cloudiness`, `precipitation`, `fog_density`, `sun_altitude_angle`, `wetness`, etc.

### Ambient Background Traffic (Optional)

CARLA's Traffic Manager can provide realistic background traffic alongside SSv2-controlled scenario actors:

1. On `Initialize` (if configured): spawn N ambient NPC vehicles with `set_autopilot(True)`
2. These NPCs are NOT registered in `EntityManager` (SSv2doesn't know about them)
3. They follow CARLA's Traffic Manager rules (lane following, traffic lights, speed limits)
4. Autoware perceives them via LiDAR just like SSv2-controlled NPCs
5. On shutdown: destroy all ambient NPCs

**Risk**: Ambient NPCs may interfere with scenario-controlled NPCs or the ego vehicle. This feature should be opt-in via config.

## Tasks

### Traffic Light Mapping
- [ ] Enumerate all CARLA `TrafficLight` actors at startup (`world.actors().filter("traffic.traffic_light*")`)
- [ ] Extract CARLA traffic light positions and OpenDRIVE signal references
- [ ] Position-based matching: given lanelet signal ID + Lanelet2 map, find nearest CARLA actor
- [ ] Fallback: load manual mapping from `config/traffic_light_map_{town}.yaml`
- [ ] Log unmapped lanelet signal IDs as warnings

### Traffic Light Control
- [ ] Freeze all CARLA traffic lights on `Initialize`
- [ ] `UpdateTrafficLights` handler: convert SSv2 `TrafficSignal` to CARLA `TrafficLightState`
- [ ] Handle multiple bulbs per signal (SSv2 can send arrow + circle states)
- [ ] Unfreeze traffic lights on shutdown
- [ ] Verify: CARLA traffic light visuals match SSv2 commands

### Weather Extension
- [ ] Define weather config schema in `bridge_config.yaml`
- [ ] Implement `set_weather()` call on `Initialize` (static weather from config)
- [ ] Optional: handle weather changes via `CustomCommandAction` or ROS parameter

### Ambient Traffic (Optional)
- [ ] Config option: `ambient_traffic.enabled`, `ambient_traffic.num_vehicles`
- [ ] Spawn ambient NPCs with `set_autopilot(True)` after `Initialize`
- [ ] Destroy ambient NPCs on shutdown
- [ ] Verify ambient NPCs don't collide with scenario entities

## Acceptance Criteria

- [ ] CARLA traffic lights are frozen (no built-in cycling) during scenario execution
- [ ] SSv2 `UpdateTrafficLights` with `RED` state makes CARLA traffic light display red
- [ ] SSv2 `UpdateTrafficLights` with `GREEN` state makes CARLA traffic light display green
- [ ] Traffic light mapping resolves at least 80% of lanelet signal IDs automatically (position-based) on Town01
- [ ] Unmapped signal IDs produce a warning (not a crash)
- [ ] Autoware's traffic light recognition detects the correct state (if `use_traffic_light_recognition=true`)
- [ ] Traffic lights return to normal cycling after scenario ends (unfreeze on shutdown)
- [ ] Weather can be set via config (e.g., rain scenario with `precipitation=80`)
- [ ] (Optional) Ambient traffic NPCs drive around without interfering with scenario logic
