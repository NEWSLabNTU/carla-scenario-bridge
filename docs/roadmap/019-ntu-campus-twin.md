# Phase 019: NTU Campus Digital Twin

Build a drivable twin of the NTU campus from the data we already hold, and use it to examine
how realistic CARLA's camera and lidar can get on a real site. Everything is measured against
the real point cloud, which is ground truth for geometry and for the lidar.

**Source**: realistic-rendering survey, user direction 2026-10-09/10: "build the digital twin
fast", "build the lanelet map easy", "enable realistic sensor rendering", "start from our NTU
campus data"; NuRec ruled out on one RTX 5090; "our bridge also supports 0.10".
**Design**: [survey-isaac-sim-and-digital-twin](../design/survey-isaac-sim-and-digital-twin.md)
(§3.5 rendering options, §5 NTU data assessment).
**Data**: `/mnt/disk1/ntu-campus-twin/` (local copy; `README.md` there has provenance and the
frame transform). NAS originals: `~/nas/autoveh/dataset/2026-04_ntu_map`,
`~/nas/autoveh/dataset/2026-05-15 NTU pointcloud map`.
**Depends on**: [009](009-map-and-traffic-lights.md) (map resolution, signal tables),
[017](017-user-workflow.md) (map dir layout, `--generate-signal-table`).
**Relates to**: [018](018-install-and-world-config.md) (CARLA version as a carla-rust feature,
run-time version check), acb vehicle model
(0.10 uses Chaos vehicles; roadmap [014](014-feature-completeness.md) gap 8).

## Problem

- **Every map we run on is a CARLA town.** Autoware localizes on a PCD rendered from
  synthetic geometry, and the camera sees UE4 content. Nothing tells us how perception or
  localization behaves on a real site.
- **The obvious realism routes do not fit.** NuRec and Cosmos Transfer need far more than one
  RTX 5090 (Cosmos-Transfer2.5-2B: 65 GB VRAM minimum).
- **An earlier attempt split the sensors.** The vendor cloud imported into CARLA 0.9.x (UE4)
  as a point-cloud asset (June 2026) gives a recognisable but noisy camera view over a white
  floor, while CARLA's raycast lidar sees only the floor: point-cloud assets have no collision.
- **The vendor's derived files are silently truncated.** The scan is LAS 1.2, whose 32-bit
  header count saturates at 4 294 967 295; its PDAL-made `tiles_50m`, `tiles_100m` and
  `downsample` sets each hold exactly that many points (or a downsample of them) out of
  ~5.52 G, and end at X = 304653. ~22 % of the points and the east ~200 m of route r01 are
  missing from everything built so far.
- **Our Lanelet2 maps have no regulatory elements.** r01 and r02 hold `road` lanelets with thin
  solid boundaries only: no traffic lights, stop lines or crosswalks, and no xodr for CARLA.
- **No campus images.** The vendor colorized from a camera we do not have; our GLIM bags
  (2026-08-20) are a basement, lidar + IMU only.

## Data

| Source | Content | Frame |
|---|---|---|
| `autoware_map_2026-04/r01`, `r02` | Lanelet2 + PCD (x y z intensity), 4.1 M / 2.5 M points | MGRS 51RUH |
| `vendor_mls_2026-05-15/ntu_mls.laz` (51 GB) | LiDAR360MLS scan, ~5.52 G points, RGB, GPS time, ASPRS classes; covers both routes, 1113 m x 649 m | TWD97/TM2, EPSG:3826 |
| `vendor_mls_2026-05-15/carla/` | June 2026 CARLA import: RGB and lidar videos | — |

Vendor → Autoware map frame: EPSG:3826 → EPSG:32651, easting/northing mod 100 000, then
subtract T = (−0.288, +0.216, +17.26) m (solved on r02; 17 m is a vertical datum difference).

## Steps

### 0. Survey and data
- [x] Survey of Isaac Sim, NuRec, CARLA 0.10, mesh/splat routes (design doc §1–4)
- [x] Inventory both datasets; find the truncation; find RGB and classes (2026-10-10)
- [x] Copy source data to `/mnt/disk1/ntu-campus-twin/`; NAS derived tiles not copied
      (truncated, ~400 GB). LAZ size matches the NAS original
- [x] Align vendor cloud to r02: translation-only ICP, median 0.068 m, p90 0.227 m, 87 %
      within 0.2 m
- [ ] Re-check alignment on r01 with full tiles. Sampled from the truncated tiles, r01 solves
      to (−0.477, +0.184, +17.27) m, median 0.128 m but p90 5.1 m; decide whether one T
      serves both routes or each needs its own (and whether rotation matters)

### 1. Full tiles in the map frame
- [ ] Reader that streams the LAZ to end of file, not to the header count (laspy + lazrs, or
      PDAL with the count overridden); report the true point count
- [ ] Re-tile to 50 m in the Autoware frame (shift applied), keeping RGB, intensity, class,
      GPS time; LAS 1.4 output so counts are 64-bit
- [ ] Sanity: tiles reach X 304853 (east end of r01); summed count equals the true count
- [ ] Write a 0.2 m `pointcloud_map.pcd` (+ metadata) covering both routes; compare NDT
      localization on it against r01/r02's own PCDs in a rosbag replay or in CARLA

### 2. Scene classes
- [ ] Per-class point counts over the whole site (sample: high vegetation 46 %,
      unclassified 40 %, ground 5 %, building 4 %, low vegetation 3 %)
- [ ] Inspect class 0 and the rare codes (15, 16, 21, 22, 26): poles, signs, parked cars,
      people. Decide per kind: keep as static mesh, remove, or replace with a CARLA prop
- [ ] Remove transient objects (parked cars, pedestrians) from the ground and road areas,
      or record that they stay as static obstacles

### 3. Road surface
- [ ] Mesh from class 2 (2.5D Delaunay or heightfield), tiled to match step 1
- [ ] Orthophoto texture rasterized from ground-point RGB at 2–3 cm; check lane paint is
      legible
- [ ] Collision on the road mesh; CARLA lidar returns the road at the real height

### 4. Buildings and vegetation
- [ ] Buildings (class 6): Poisson mesh per tile, vertex colour or baked texture, collision on
- [ ] Vegetation (classes 3, 5): camera layer as points (UE point-cloud plugin) with coarse
      voxel or convex proxies as collision; compare against instanced UE foliage placed at
      trunk positions
- [ ] Find the white floor in the June import (scene default plane, or point-cloud plugin
      material) and confirm the road mesh replaces it

### 5. Lanelet2 and OpenDRIVE
- [ ] Extend r01/r02 (or the merged site) with traffic lights, stop lines and crosswalks
      over the vendor cloud; validate with `autoware_lanelet2_validation`
- [ ] Merge r01 and r02 into one site map if they share an origin and frame (both MGRS
      51RUH)
- [ ] Produce an xodr that agrees with the Lanelet2 map (CommonRoad Scenario Designer or
      hand); check alignment with `--generate-signal-table`
- [ ] Assemble a map dir per [user-guide](../user-guide.md) §2 (`lanelet2_map.osm`,
      `pointcloud_map.pcd`, projector, `carla/traffic_lights.yaml`)

### 6. Import into CARLA
- [ ] Stand up CARLA 0.10.0 on this host (16 GB+ VRAM recommended, 130 GB disk; package in
      `~/Downloads`). Build csb/acb against it by switching the `carla` dependency feature
      `carla-0916` → `carla-0100` in both workspaces (018); the bridges refuse a version
      mismatch at connect. Record what breaks (steering getter returns 0, Chaos vehicle
      tuning, town list)
- [ ] Import route r02 first (smaller): fbx tiles + xodr into a CARLA 0.10 source build,
      Nanite on the meshes
- [ ] Same scene into 0.9.16 for comparison, if the import path allows
- [ ] The bridge resolves the map dir to the new CARLA map (009 alias table)

### 7. Measure realism
- [ ] Lidar: at fixed poses along the route, compare a CARLA lidar scan with the real cloud
      cropped to the same sensor model (range, FOV). Report median point-to-surface distance
      and the fraction of real returns with no simulated counterpart (vegetation is the
      expected gap)
- [ ] Camera: frames at the same poses from CARLA 0.10, CARLA 0.9.16, and the June
      point-sprite import, side by side; vendor render as reference
- [ ] Localization: Autoware NDT on the twin with the real-site PCD; report score and drift
      along r02
- [ ] Frame budget: tick time and VRAM with the full ego sensor set on the twin

### 8. Camera data for the next layer
- [ ] Ask the vendor for the panoramic images, camera calibration and trajectory used to
      colorize the scan
- [ ] If received: better building and road textures, and a gsplat reconstruction for a
      camera-only splat layer over the mesh (survey §3.5 C)

## Acceptance

- One SSv2 scenario runs on route r02 of the NTU twin in CARLA 0.10, with Autoware
  localizing on the real-site PCD and engaging along the route.
- The lidar comparison (step 7) is published as a table: simulated vs real at fixed poses,
  with the vegetation gap quantified rather than assumed.
- Camera frames from the twin are archived next to the vendor reference render, so the next
  step (splats or better textures) can be judged against them.
- The data README records the true point count, the frame transform per route, and every
  derived artifact with the command that made it.
