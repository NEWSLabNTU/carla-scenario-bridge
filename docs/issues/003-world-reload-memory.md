# 003: The Isaac world server grows ~130-150 MB with every scenario's world load

**Severity**: low. **Status**: fixed (2026-10-24): each ego's RTX lidar left its render
product's memory behind. The lidar is now made once per ego slot and kept, beside the vehicle
rather than under it (aib `sensors.Lidar`, isaac_world `_ego_path`). **Found**: 2026-10-23,
while measuring issue 001's fix.

## What is wrong

The world server reloads the world for each scenario (`LoadWorld`). Over a batch of
`town01_soil.xosc` runs its resident memory rises by about the same amount every run and
never comes back:

| batch | ground in the soil zones | runs | RSS (play_launch metrics) |
|---|---|---|---|
| `play_log/isaac/2026-10-23_21-38-24` | cooked collision tiles | 8 | 18.23 -> 19.26 GB |
| `play_log/isaac/2026-10-23_22-36-02` | plates (issue 001) | 4 | 18.41 -> 19.00 GB |

So it is not the ruts (issue 001's plates removed that cost: 0.1 MB a pass on the pad).

## Why it matters

The server already holds ~17.5 GB with Town01, and the OOM killer has picked it before
(CLAUDE.md, "On Isaac"). A suite of a few hundred scenarios on one server adds tens of GB.

## Where it goes

`src/swb_isaac_world/spikes/reload_memory.py` runs the world server's own `IsaacWorld` with
Town01 preloaded. In each phase it repeats one part of what a scenario does six to eight
times and reads the resident memory after each (MB per cycle, after the first):

| phase | what each cycle does | MB per cycle |
|---|---|---|
| `bare` | LoadWorld, no surface map | 0.0 |
| `soil` | LoadWorld with town01_soil's surface map (the cut, the tiles, the plates) | 0.3 after the first |
| `bare_nosensor` | the ego spawned and despawned, every sensor of its rig off | -1.1 |
| `bare_ego` | the same with the rig's sensors (lidar, IMU, GNSS) | 107 |
| `soil_ego` | `soil` plus the ego, as a scenario does | 109 |
| `lidar_raw` | no ego: an RTX lidar, its render product and annotator made, 100 frames, torn down as aib's `Lidar.close` did | 107 |

So it is neither the world load nor the soil. It is the ego's RTX lidar. Bisecting the lidar
(same spike, one render product held across cycles unless said otherwise):

| phase | the lidar each cycle | its render product | MB per cycle |
|---|---|---|---|
| `lidar_raw` | new, at a new path | new, destroyed after | 107 |
| `lidar_keep_rp` | new, at a new path | kept, pointed at it | 105 |
| `lidar_pause` | new, at the same path | kept, paused between cycles (updates off) | 90 |
| `lidar_parkpause` | new, at the same path | kept, pointed at a parked lidar and paused | 80 |
| `lidar_same` | new, at the same path | kept, pointed at a parked lidar, live | 0.2 |

Isaac never gives back what a render product of an RTX lidar cost. It keeps it whether the
product is destroyed, kept and pointed at a lidar at a new path, or paused. The only cycle
that costs nothing is the same product, kept live and pointed at a lidar made again at the
same path. Each scenario's ego had a new path (`/World/entities/e<handle>`) and a new lidar
with a new render product: ~107 MB of the 130-150 MB a run measured.

## Fix

A first fix took that one leak-free cycle literally. An ego's lidar stayed under the vehicle,
and the render product outlived it: pointed at a parked lidar under the world, then at the
next ego's new lidar prim at the same path. The spike was flat over 116 such takeovers. The
real world server hung on its third: the warm-up renders of the fourth ego timed out
(`Failed to begin render graph. Waiting for frame submission semaphore timed out after 60
seconds`, then `Sensor beginFrame() failed`), and it never answered again
(`play_log/isaac/2026-10-24_09-25-33`). In none of the world server's earlier logs had that
error ever appeared. A render product's camera changing to a new prim is evidently something
the RTX sensor pipeline does not always survive.

So nothing is re-pointed or remade:

- aib (`aib.isaac.sensors.Lidar`): the lidar is not under its vehicle. Its mount is
  `/World/aib_sensors/<vehicle path>_<sensor>`. The lidar prim and its render product are made
  there once, by the first ego at that vehicle path, and kept for the next ego at the same
  path. Before each substep, `Ego.before_substep` puts the mount where it will be after the
  substep: the body's pose after the last one, moved on at its velocity and angular velocity
  (`aib.frames.advance`; for a spawn or a teleport, its pose as set). That is the inverse of
  how `Vehicle.sample` reads those velocities. The extrapolation error is second order: at
  3 m/s^2, 0.4 mm per 1/60 s substep. A kept lidar waits two renders before it is ready,
  because its annotator still holds the last ego's frame. `close` leaves the lidar as it is.
  The camera, off in the shipped rig, still rides under the vehicle and is made and
  destroyed each time.
- isaac_world (`IsaacWorld._ego_path`): an ego is spawned at the lowest free ego slot,
  `/World/entities/ego0`, `ego1`, ..., not at its handle's path. So every scenario's ego
  finds its slot's lidar.

The spike after the fix, 40 cycles each, no hang:

| phase | RSS over the cycles (MB) | per cycle after the first |
|---|---|---|
| `bare_ego` | 17310 -> 17312 | 0.05 MB |
| `soil_ego` | 17474 -> 17537 (17522 after the first) | 0.4 MB |

## Also found

Removing a zone's tiles removed its collision group, which PhysX reports as "Removing a
CollisionGroup while the simulation is running results in undefined behavior!" (twice per
load). The flat ground's group (issue 001) is now one group for every zone,
`/World/soil_flatGroup`, never removed; a zone's box joins and leaves it with the zone.

## Validation

`town01_soil.xosc` x8 on Isaac, a fresh world server (`play_log/isaac/2026-10-24_10-26-31`,
`play_log/acceptance/2026-10-24_10-26-28`): 8/8 PASS, 0 MRM, every ego after the first on the
kept lidar ("kept from the last vehicle at this path"), and no render error. The world
server's resident memory at each world load:

| run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| RSS (GB) | 17.646 | 17.713 | 17.722 | 17.728 | 17.728 | 17.743 | 17.733 | 17.740 |

After the first run that is 4 MB a run, with a scatter of about 10 MB, where it was 130-150.
Localization is as before (the gap to ground truth 0.97-1.10 m, against 1.03-1.10), so the
lidar extrapolated beside the vehicle sees what the one under it saw.
