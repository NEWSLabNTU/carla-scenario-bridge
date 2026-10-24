# 005: Isaac Sim loses the GPU rendering a SimReady CARLA town on driver 615.71.09

**Severity**: high on the host it hits (the Isaac SimReady world cannot run there). **Status**:
open; the fix is the host's NVIDIA driver, which needs root. **Found**: 2026-10-23 on
newslab-118-server (roadmap 051 K5).

## What happens

Any Isaac Sim 6.1 render of CARLA's Town01 SimReady world (`$CARLA_USD/Town01/
Town01_physics.usda`) loses the GPU, usually by the 2nd to 10th frame, about 30 s after the
stage opens:

```
[Error] [omni.rtx] VkResult: ERROR_DEVICE_LOST
NVRM: Xid (PCI:0000:41:00): 109, ... errorString CTX SWITCH TIMEOUT, Info 0x1c006
NVRM: Xid (PCI:0000:41:00): 31, ... MMU Fault: ENGINE GRAPHICS GPC1 UNRECOGNIZED_CLIENT ...
    Fault is of type UNRECOGNIZED_FAULT UNRECOGNIZED_ACCESS_TYPE
```

In a run that is the Isaac world server dying at its first rendered frame: `just e2e
sim:=isaac` on the SimReady world failed all 36 respawns. The headless preload survives only
because nothing renders after its 5 warm-up frames.

## What it is not

Not swb's or aib's code: a plain `SimulationApp` that opens the stage headless and calls
`app.update()`, with no swb code, no ego and no physics, does the same. That is
swb_isaac_world's `spikes/render_probe.py`. None of these made a difference:

- the ego's lidar on or off;
- GPU 0 or GPU 1, an otherwise idle GPU, PhysX CUDA on the other GPU;
- synchronous material loads, PathTracing or RaytracedLighting;
- DLSS off; reflections, GI, shadows and AO off.

A 50-cube stage rendered 20000 frames clean. Town01 subsets were mixed:

- Buildings alone or the ground alone rendered 5000 frames clean.
- Props alone or Walls alone crashed intermittently, with no single asset to blame.

A related fault, seen once on 2026-10-24 on the point-cloud world, which otherwise runs
here: aib's runner hung in warm-up with `[Fatal] [omni.rtx] Out of resource descriptors!`
(`play_log/051/proof/aib_signals_g20.descriptors`). The same command passed before and
after it.

## Cause (suspected)

The driver. On 2026-10-15 the host went from 595.45.04 to **615.71.09** (open kernel module,
GSP). NVIDIA lists 595.58.03 for Isaac Sim 6.1 on Linux. NVIDIA already tracks Xid 109 bugs
against 615 (6756147, 6869685, 6878084, 6899071). newslab-server139 runs 580.105.08 and
renders the same world. 615.71.10 is out.

## Closing it

Someone with root installs a driver Isaac 6.1 is tested on (595.58.03, or 580.x as on
server139), or 615.71.10 to try that one, and reboots. Then, from swb_isaac_world:

```bash
spikes/run_spike.sh cyclonedds /tmp/probe.log spikes/render_probe.py \
    $CARLA_USD/Town01/Town01_physics.usda 2000
grep PROBE /tmp/probe.log | tail -2      # PROBE PASS 2000 frames
```

If it passes, `just e2e town01_ego_drive sim:=isaac` on the SimReady world (default
`CARLA_MAPS`) closes this. Until then, on this host the Isaac runs use the point-cloud world
(`swb_isaac_world.pcd_world`, roadmap 038), as roadmap 051's proof runs did.
