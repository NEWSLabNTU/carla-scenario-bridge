# 003: The Isaac world server grows ~130-150 MB with every scenario's world load

**Severity**: low. **Status**: open. **Found**: 2026-10-23, while measuring issue 001's fix.

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

## What to check

- Whether a world without a surface map grows too (`town01_ego_drive`, same batch size): if
  so, it is the reload itself, not the soil zones.
- With soil zones: each load cuts the zones' boxes out of the town's meshes and makes new
  invisible collision copies of the cut meshes (`soil_tiles.cut_stage`); Landscape_5 alone has
  ~100k faces. PhysX keeps every mesh it cooks (issue 001); identical copies should hit its
  cache, unless the copy differs from load to load.
