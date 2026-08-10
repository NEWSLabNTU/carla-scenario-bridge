# CARLA server tuning

What the CARLA 0.9.16 server costs to start and to hold, what changes that, and what was
tried and rejected. Every number here comes from `scripts/carla_bench.py`, which launches
one configuration, times it to the first usable world, samples host and GPU memory, ticks
100 empty frames, and spawns a ray-cast LiDAR to prove the configuration still produces the
data the stack needs.

Measured 2026-08-10 on the RTX 5090 host (32 GB VRAM, 60 GB RAM, CARLA 0.9.16 base package,
another tenant's training job holding ~4.4 GB of VRAM throughout).

## Result

| Configuration | boot map | time to usable | peak RSS | steady RSS | VRAM | empty tick |
|---|---|---|---|---|---|---|
| Package default | Town10HD_Opt, then bridge loads Town01 | 10.8 s | 7368 MB | 2952 MB | 5591 MB | 286 Hz |
| **Town01 from boot** | Town01 | 11.1 s | **3292 MB** | 2444 MB | **1740 MB** | 318 Hz |

Same LiDAR output in both (20 scans, 69475 points at 128 channels / 2.62 M pts/s).

Booting on the town the run uses is worth **4.1 GB of peak RSS and 3.9 GB of VRAM**, and
~11 % more headroom on the tick rate. Wall-clock start-up is unchanged: the default
configuration reaches the RPC sooner but then pays for a map load, and the two cancel out.

The memory is the point. The recurring failure on the shared GPU host was CARLA being
OOM-killed mid-run by other tenants' jobs (ten kills in one day, per
`docs/roadmap/010-multi-instance.md`); a server that holds 1.7 GB instead of 5.6 GB of VRAM
is a much smaller target. It also matters directly for the multi-Autoware work, where two
Autoware stacks and CARLA share one 60 GB host.

## How the boot map is set

Not on the command line. In 0.9.16 the packaged build ignores a startup map argument
entirely -- `./CarlaUE4.sh Town01`, `./CarlaUE4.sh /Game/Carla/Maps/Town01`, and passing the
map directly to `CarlaUE4-Linux-Shipping` as argv[1] all still boot `Town10HD_Opt`. (Worth
knowing separately: `CarlaUE4.sh` hardcodes `CarlaUE4` as the binary's first argument, so a
map given to the wrapper never even reaches the slot UE4 reads it from.)

What works is `GameDefaultMap` in `CarlaUE4/Config/DefaultEngine.ini`:

```ini
GameDefaultMap=/Game/Carla/Maps/Town01.Town01
```

`third_party/carla/run.sh` applies this from `CARLA_MAP` (default `Town01`) before every
launch, idempotently, keeping the untouched original as `DefaultEngine.ini.orig`. Set
`CARLA_MAP=` (empty) to leave the installation's own setting alone.

The bridge already skips `load_world` when the server is on the right town
(`coordinator.rs::load_scenario_map`), so booting on the scenario's town removes the load
rather than duplicating it.

## Rejected

- **`-nullrhi`** -- would drop the rendering backend and with it nearly all of the VRAM.
  CARLA 0.9.16 segfaults on startup with it: `Signal 11 caught` / `CommonUnixCrashHandler:
  Signal=11`, before the RPC server comes up. Not usable at any quality level.
  `-RenderOffScreen` remains the way to run without a window.
- **Client-side `no_rendering_mode`** -- not measured, and not applicable while any camera
  is in the sensor kit. It saves GPU *work*, not the GPU memory already allocated by the
  RHI, so it does not address the OOM-kill problem.

## Not installed on purpose

`AdditionalMaps_0.9.16.tar.gz` (14.8 GB) is **not extracted**. The base package ships
Town01-Town05 and Town10HD, which covers every town in the TUM map pack under
`data/carla-autoware-bridge/` (Town01, Town02, Town03, Town05, Town10). AdditionalMaps only
adds towns nothing in this repo references, and `/home` on this host is at 96 %.

If a scenario ever needs Town06/07/11/12/13/15, extract it into the same
`CARLA_0.9.16` directory -- it is an overlay, not a separate installation.

## Reproducing

```bash
python3 -m venv /tmp/carlaenv
/tmp/carlaenv/bin/pip install ~/Downloads/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl

# One line of JSON per configuration, appended to bench.jsonl
DISPLAY=:1 /tmp/carlaenv/bin/python scripts/carla_bench.py --label default --out bench.jsonl
DISPLAY=:1 /tmp/carlaenv/bin/python scripts/carla_bench.py --label nullrhi --extra=-nullrhi --out bench.jsonl
```

The harness kills any server already holding the port before it starts, and by pid rather
than `pkill -f` -- a `-f` pattern containing `CarlaUE4-Linux-Shipping` also matches the
shell running the pkill, which is a trap this project has paid for before.
