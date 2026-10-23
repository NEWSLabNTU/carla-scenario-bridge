# 001: Each soil tile rebuild leaves ~0.3 MB in the Isaac world server

**Severity**: medium. **Status**: open. **Found**: 2026-10-23, roadmap 047 S3.

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

## Suspects and next steps

- PhysX's cooked-mesh data (or the UJITSO cooking cache) for removed prims is kept.
  Isaac 6.1 exposes no local mesh-cache size setting (only `/physics/cooking/ujitso*`).
- Try: reuse two prims per tile and toggle which one collides (`CollisionAPI` removed on the
  other) instead of making a new path each time; a stable set of paths may let PhysX free or
  reuse the cooked data. The spike's "toggle collision" variant re-cooked in 25 / 7 ms but
  needs a step between remove and apply, so the tile would be absent for one step unless the
  two prims overlap during the swap.
- Try: `omni.physx` `release_physics_objects` or a cooking-service purge after a batch of
  removals, if one exists in 110.3.
- Measure per rebuild with a longer pass (200+ rebuilds) to tell a cache with a ceiling from
  a leak.
