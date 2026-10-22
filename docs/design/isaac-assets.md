# Isaac assets: CARLA's towns, vehicles, props and walkers as USD

The Isaac backend (roadmaps 043-045) renders CARLA's content: its towns, its vehicle
blueprints, its `static.prop.*` props and its walkers, each exported from CARLA 0.9.16's
editor with NVIDIA's SimReady converter and wrapped for Isaac. Making them takes a UE 4.26
plus CARLA editor build (~150 GB, hours; roadmap 043, "A new host") and a few hours of
export. The finished set is ~24 GB and moves between hosts as one directory.

**Obtaining it.** The set is available from NEWSLab on request; or build it yourself with the
recipes below. Either way it is one directory, the asset root, wherever you put it.

## Layout

```
<asset root>/                       CARLA_USD
├── README.md                       what, from which versions and commits (written by `manifest`)
├── MANIFEST.sha256                 every file's checksum (`just isaac-assets check`)
├── Town01/ Town02/ Town03/ Town05/ Town10HD/
│   ├── <Town>.usd, Materials/, Props/, HDRi_Neutral.dds    the SimReady export
│   ├── <Town>_physics.usda         the collider layer over it (isaac_world.physics_layer)
│   ├── main.xodr
│   └── export.log
└── vehicles/                       SWB_VEHICLE_USD
    ├── vehicle.<make>.<model>/     41: <id>.usda (/Vehicle), wheels.usda, the raw export
    ├── vehicles.json, vehicles.md  the catalogue: CARLA box, wheels, rider pose
    ├── physics.json                CARLA's get_physics_control() per blueprint
    ├── ue_meta.json                what the editor reported per blueprint
    ├── props/                      97 static.prop.*: <id>/<id>.usda (/Prop), props.json
    ├── walkers/                    52 walkers: <id>/<id>.usda (/Walker), walk.npz, idle.npz,
    │                               walkers.json (walker.pedestrian.NNNN -> blueprint is
    │                               isaac_world/walker_blueprints.json, in the repo)
    └── riders/grip.npz             CARLA's bike-grip pose, for seated riders
```

Every path inside the set is relative: USD layers reference their neighbours as `@./...@`,
and `vehicles.json`, `props.json` and `walkers.json` name files relative to the index's own
directory (`isaac_world.vehicle_assets` resolves them on load; an absolute path, as indexes
written before 2026-10-23 have, is taken as it is). So the set works from any directory.

## Using a set

1. Put it anywhere (a fast disk: a town preload reads ~1-2 GB of small files), and name it
   in `.env`, both host settings:

   ```
   CARLA_USD=/data/carla-usd
   SWB_VEHICLE_USD=/data/carla-usd/vehicles
   ```

2. Check the copy: `just isaac-assets check` (root `CARLA_USD`; `root:=<dir>` for another).
   It reports each missing or changed file and exits non-zero if there is one.
3. Point each town's Autoware map at its world: `just download-maps`, then `just isaac-town
   <Town>` for Town01, Town02, Town03, Town05 and Town10HD. With the export present it does
   not run the editor (no `CARLA_SRC` needed): it rewrites `<Town>_physics.usda` (the same
   bytes: the layer is a function of the export) and writes `<map>/isaac/world.yaml`, which
   names the layer by its absolute path on this host.

The world server and the bridge then find everything through `CARLA_USD` and
`SWB_VEHICLE_USD`; nothing else names the set.

## Making a set

| part | recipe | editor time (server139) | size |
|---|---|---|---|
| a town | `just isaac-town <Town> [export:=true]` | 5-15 min each | 0.9-1.9 GB |
| vehicles | `just isaac-vehicles` | 12 min | 2.6 GB |
| vehicle physics | `src/isaac_world/scripts/dump_vehicle_physics.py --root "$SWB_VEHICLE_USD"`, with a CARLA 0.9.16 of its own up | 2 s | - |
| props | `just isaac-vehicles props:=true` | - | - |
| walkers | `just isaac-vehicles walkers:=true` | 24 min for 49 | 14 GB |
| rider pose | `just isaac-vehicles riders:=true` | - | - |

`riders:=true` also rewrites the vehicles' wrappers, seating the riders, so run it after
the vehicles. Then `just isaac-assets manifest` writes `README.md` (the CARLA version,
the swb commit, the counts) and `MANIFEST.sha256`: run it after every change to the set,
before handing it on. `manifest` and `check` skip `logs/`, `raw/` and `*.pre*` backups.
