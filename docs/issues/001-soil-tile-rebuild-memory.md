# 001: Each soil tile rebuild leaves its cooked collision in the Isaac world server

**Severity**: medium, then low. **Status**: fixed 2026-10-23 (no collision mesh is cooked
after load; the cause, PhysX keeping every cooked mesh, stays). **Found**: 2026-10-23, roadmap
047 S3.

## What is wrong

A soil zone with its own ground (`ground_z`, `isaac_world.soil_tiles`) puts the grid's ruts
into the collision by rebuilding a 1.6 m tile (33 x 33 vertices, 2048 triangles) as a new mesh
prim and removing the old one. PhysX cooks a mesh once and ignores later point edits (047
spike), so a new prim is the only way found to get a new cook without the tile disappearing
for a step. The world server's resident memory grows with every rebuild and does not come
back when the old prim is removed.

## Why it matters

A pass rebuilds about 2.5 tiles per metre of track (front then rear wheels, two tracks).
At ~0.3 MB a rebuild that is ~1 GB per kilometre driven in a tile zone. The Isaac world
server already holds ~17.5 GB with Town01, and the OOM killer has picked it before (CLAUDE.md,
"On Isaac"). A long rally stage in a zone would run the host out of memory.

## How it was confirmed

`src/isaac_world/spikes/soil_drive.py` (`SOIL_SPIKE_ONLY=rut`), pad, aib's Model 3 at 8 m/s
through a 60 m sand zone with tiles, two passes:

| pass | tiles rebuilt | RSS growth |
|---|---|---|
| 1 | 88 | 23.1 MB (33.5 in another run) |
| 2 | 88 | 29.3 MB (36.3) |

Before the fix that waits for wheels to leave a tile (047 S3), tiles under a stuck car were
rebuilt every frame (676 in one pass) and RSS grew ~15 MB/s, to 17 GB in ten minutes. The
soil grid itself does not grow (`SoilGrid.wheel` 5500 times, numpy and warp: 0.0 MB).

## Where the memory goes (`spikes/tile_memory.py`)

One tile rebuilt 300-1500 times per variant, a physics step after each, a downward ray
checking the new shape took:

| variant | KB kept per rebuild | ray sees the new shape |
|---|---|---|
| new prim, new path (soil_tiles before), shapes repeating | 8.6 | all |
| new prim, new path, every shape new (as in a drive) | 382 | all |
| same path, removed and defined again, every shape new | 465 | all |
| the same, without CollisionAPI (render and USD only) | 3.3 | n/a |
| legacy cooking (`/physics/cooking/ujitsoCollisionCooking` false) | 215 | all |
| `release_local_mesh_cache()` after each rebuild | 216 (and ~9 ms) | all |
| private `release_runtime_mesh_cache()` after each | 289 | all |
| points edited in place: collision toggled, approximation re-set, or a second prim's collision swapped in | 0 | never re-cooked |
| `add_prim_to_cooking_refresh_set` | | refused: its binding wants an SdfPath from another pxr build |
| 10 cm collision tile (17 x 17), every shape new | 64 | all |
| the same, cache released after each | 45 (and ~9 ms) | all |

So PhysX keeps the cooked data of every distinct mesh it has cooked, prim removed or not,
in proportion to the triangles, linearly over 1500 rebuilds (no ceiling seen), and nothing
reachable from Python frees it or re-cooks a mesh in place. Repeated shapes hit the cache
(the first spike's 8.6 KB was that).

## Fix (047 S3)

- The collision tile is 10 cm (17 x 17 vertices, 512 triangles); the render tile stays at
  the grid's 5 cm, one prim whose points are written in place (no cooking, no memory kept),
  so cameras and the lidar keep the full detail (the lidar still reads the rut 9.36 cm deep
  against the grid's 9.43).
- A tile is rebuilt once no wheel is within 3 m: one rebuild per tile per pass, not one per
  axle.

Pad, two passes of 35 m at 8 m/s: 44 rebuilds and 3.7 / 2.9 MB a pass, against 88 and 23-36 MB
before, ~100 MB per km per pass in a tile zone. The second pass still rides in its ruts
(-9.3 cm).

## Fix 2: no cooked collision at all (2026-10-23)

The first fix left the memory kept, twelve times slower: a long event on one zone (many cars,
many passes) still grew the server by ~100 MB per km of tracks rebuilt. Of the ways left (rut
heights quantised so shapes repeat, an NVIDIA API to drop cooked meshes, a collision that
needs no cooking), the third works with what Isaac has. The only bodies that must feel the
ruts are the egos, and an ego meets the ground through one suspension ray per wheel (aib's
vehicle, `raycast`). So `soil_tiles` now gives a zone three things, none cooked after load:

- **render**: unchanged, a mesh per tile at the grid's 5 cm, points written in place.
- **flat**: one box per zone, its top at `ground_z` over the tiles' box, in a collision group
  that filters aib's `groundQuery`: tracked NPCs and anything dropped stand on it; the egos'
  rays go through it.
- **plates**: a thin box (0.8 x 0.8 x 0.02 m) per ego wheel, placed before every substep
  (`SurfaceLayer.place`, `soil_tiles.SoilTiles.place`) where the wheel will be after it
  (`surface_layer.wheel_points`: body pose plus velocity times the step), its top the
  least-squares plane of the ruts over the tire's footprint (5 points across its width, 3
  along). A box is never cooked; moving a static one keeps nothing.

The heights the plates read are the tiles' `shown` arrays, refreshed when a tile is rebuilt,
under the same rule as before (once no wheel on soil is within 3 m): a wheel's own pass is the
grid's forces' to carry, its rut felt from the next pass on.

Why a footprint and not the point under the wheel: the grid's rut is a box, 0.25 m wide with
vertical walls (one pass of the Model 3's wheel, 8 m/s). The old 10 cm collision mesh made each
wall a 10 cm slope, so a wheel whose ray landed just beside a rut slid into it. A plate at the
single ray point did the opposite: a tire overlapping a rut by 9 cm (shifted lines in the
endurance passes below) stayed on the untouched ground beside it. Over the footprint, a tire
half over a rut sits half in it, tilted toward it (`test_soil_tiles.py`), and on the pad the
car settles into ruts it overlaps.

Pad (`spikes/soil_drive.py`, `SOIL_SPIKE_ONLY=rut`, new `SOIL_SPIKE_PASSES`: more passes,
each 0.15 m beside the last, so every rebuild is a new shape), the same seeds on both builds:

| build | 2nd pass: rear wheels / body | endurance passes | rebuilds | RSS growth |
|---|---|---|---|---|
| cooked 10 cm tiles (HEAD) | -9.0 cm | 20 | 827 | 105 MB, 2.5-6.7 a pass (one 18.5) |
| plates, footprint fit | -7.8 / -8.1 cm | 10 | 331 | 0.9 MB, 0.0-0.2 a pass |
| plates, footprint fit | | 20 | 537 | 52 MB: 51.5 on the first, 0.0-0.5 after |
| plates, point under the wheel | -9.2 / -8.9 cm | 20 | 903 | 18.8 MB (one 13.9) |

One run of the point-under-the-wheel build also grew 574 and 1057 MB on two passes and nothing
on the other seventeen; a second run with the same seeds did not. Such one-off jumps appear on
both builds and are not per rebuild (the next passes add nothing): an allocator's, not a mesh's.
Second-pass speed 7.67 m/s, largest drop in a frame 0.02 m/s (0.25 with the first cooked
tiles). Cost: the fit 26 us a wheel (100 us a substep for four, p50), the plates' USD writes
included up to 1.2 ms at p99 on a host at load 70-130; a tile rebuild is now its render mesh
only (0.6-1 ms a frame).

A plate stays up until its wheel is half a plate (0.4 m) clear of the tiles' box, flat at
`ground_z` past the tiles. The first version parked it as soon as the wheel's predicted
centre left the box, and the ray, landing a few cm from the prediction, found the box's hole
(the town's ground cut out, the flat box filtered): one substep in the air per axle at the
pad's zone exit and 1.1 cm of heave (0.01 cm after the fix, 0 substeps in the air). On Town01
that was two emergency stops at the dirt zone's exit (issue 002: NDT at its iteration limit).

Town01 with Autoware (`town01_soil.xosc`, lead on the ego's line, `play_log/acceptance/
2026-10-23_22-35-55`): 4/4 PASS, 0 MRM, longitudinal 0.19-0.28 m/s^2 (the cooked tiles:
0.19-0.29). The world server grew ~150 MB a run, as it did with the cooked tiles (~130 MB a
run, 18.23 to 19.26 GB over eight): that growth is the world's reload each scenario, not the
ruts ([issue 003](003-world-reload-memory.md)).

What the plates do not carry: ruts under a tracked NPC (it stands on the flat box; it never
felt the soil either) and the chassis meeting rut ridges (aib keeps the chassis at least
0.05 m above the wheel bottom; the deepest ruts seen are 17 cm, against a chassis 0.42 m up).

## Still open

Nothing for this issue. PhysX still keeps every mesh it cooks; anything that cooks meshes
at run time (a future deformable obstacle) meets the same wall.
