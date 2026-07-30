# Integration probe

`ssv2_probe.py` stands in for SSv2, speaking the real ZMQ + protobuf `simulation_interface`
protocol to `carla_scenario_bridge`, and checks every assertion against the live CARLA world
through the Python API rather than against what the bridge reports about itself.

It exists because the unit tests cannot reach the interesting behaviour: `Coordinator` needs
a live `World` to construct, so map loading, spawn, teardown, sync-mode sequencing and traffic
light control were all unverified until this ran.

It needs **no Autoware and no GPU rendering**, which is what makes it usable on a shared or
headless machine.

## Running it

```bash
# 1. CARLA. -nullrhi skips rendering entirely: much faster to start, and it works when
#    the GPU has no free VRAM. Sensors produce nothing in this mode, which is fine here
#    because the probe never uses them.
cd /path/to/CARLA_0.9.16
./CarlaUE4.sh -nullrhi -carla-rpc-port=2000

# 2. Generated protobuf bindings
protoc -I proto --python_out=scripts/integration/_pb proto/*.proto

# 3. The bridge
just run

# 4. The probe
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 scripts/integration/ssv2_probe.py
```

`PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` is needed when the system `protoc` is older
than the installed `protobuf` runtime (3.12 vs 6.x here), which otherwise refuses to load the
generated modules.

## What it covers

| Phase | Checks |
|---|---|
| 006 | `AttachLidarSensor` is rejected rather than silently accepted |
| 007 | spawn, despawn, and that a second `Initialize` destroys the previous run's actors |
| 008 | pedestrians and misc objects spawn; an NPC tracks its commanded pose (the physics fix) |
| 009 | the town named by `lanelet2_map_path` is loaded; an unusable path fails; lights are frozen; `UpdateTrafficLights` is accepted |
| gap 1 | sync mode is **off** at `Initialize` and **on** from the first frame after the ego spawns |

## Start it from a dirty world

Run the probe twice in a row without restarting CARLA. The second run starts with CARLA in
synchronous mode, left there by the first — which is precisely the state that used to
deadlock startup, and is why `initialize()` now forces async rather than assuming it.
