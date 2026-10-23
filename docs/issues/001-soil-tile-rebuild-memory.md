# 001: Each soil tile rebuild leaves its cooked collision in the Isaac world server

**Severity**: medium, now low. **Status**: mitigated (12x less); the cause is PhysX's and
stays. **Found**: 2026-10-23, roadmap 047 S3.

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

## Still open

The memory is still kept, only twelve times slower. A long event on one zone (many cars,
many passes) grows the server by ~100 MB per km of tracks rebuilt. Ways left: quantise rut
heights so shapes repeat (cache hits), a NVIDIA fix or API to drop cooked meshes, or a
collision representation that needs no cooking.
