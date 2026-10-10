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
- [x] Re-checked both routes against the full tiles (2026-10-11,
      `scripts/ntu_twin/align_route.py`, reports in `/mnt/disk1/ntu-campus-twin/alignment/`;
      16 tiles per route, 0.1 m voxels, route PCD → vendor):

      | Route | Translation-only t (m) | Median | p90 | Rigid yaw / pitch / roll (deg) |
      |---|---|---|---|---|
      | r02 | (−0.565, +0.207, +17.302) | 0.084 | 3.68* | 0.028 / 0.007 / 0.093 |
      | r01 | (−0.517, +0.185, +17.289) | 0.096 | 0.45 | −0.027 / −0.016 / 0.031 |

      \* Two south tiles of r02 (1043_1350, 1044_1350) have no matching vendor surface, and
      ICP diverges there by 7–9 m. They account for r02's p90; elsewhere per-tile p90 is
      0.1–0.5 m.

  - **Mean shifts agree to 5 cm**, so a common shift of about (−0.54, +0.20, +17.30) m
    serves both routes on average. Rotation is negligible: 0.03° over 500 m is about 0.25 m.
  - **Our route PCDs drift internally against the vendor scan.** Per-tile shifts vary by
    0.3–0.6 m in X, about 0.2 m in Y and 0.3 m in Z. Over r02 the variation is close to
    linear (rms 3–4 cm), consistent with that map being about 0.05% larger than the scan.
    On r01 the pattern differs and is less linear (rms 5–9 cm). The drift is per route,
    not one global similarity; the UTM grid scale (k ≈ 0.99987) explains only 0.013%
  - **Decision:** export the twin's Autoware `pointcloud_map.pcd` from the vendor tiles
    (common shift applied). The PCD Autoware localizes on and the world CARLA renders are
    then the same geometry by construction. The r01/r02 Lanelet2 maps were drawn on the
    old PCDs and inherit their drift, up to ±0.3 m. Re-register their nodes with a
    per-route correction field from these reports, or redraw them over the new PCD (step 5)

### 1. Full tiles in the map frame
- [x] Reader that walks the LAZ chunk table instead of the header count
      (`scripts/ntu_twin/retile_laz.py`, laspy 2.7 + lazrs in
      `/mnt/disk1/ntu-campus-twin/.venv`). **True count: 5 523 612 787 points**, which is
      110 472 full 50 000-point chunks plus a partial last chunk. Fixed-size chunk tables do
      not record the last chunk's count. Decoding it from a stream cut at its last byte fails
      after 12 789 points; the final 2 are dropped as possible decoder overrun, so the count
      is exact to within 2 points (2026-10-11)
- [x] Re-tiled to 167 tiles of 50 m in `/mnt/disk1/ntu-campus-twin/tiles_50m_mgrs/`
      (`tile_<floor(x/50)>_<floor(y/50)>.laz`, LAS 1.4, point format 3, 1 mm, all
      attributes kept; `tiles.json` has per-tile counts and bounds). Frame: MGRS 51RUH local,
      fitted from pyproj by a quadratic (max error 0.001 mm; per-point check against pyproj
      max 0.7 mm = output quantization). **Z stays in the vendor datum and the ICP shift is
      not applied**: it is a fitted value that may differ per route, so exports apply it.
      122 GB (the source LAZ is 51 GB; per-tile ordering compresses worse)
- [x] Sanity: summed tile count equals the true count exactly; extent X 51965.0–53081.8,
      Y 67424.4–68065.9 (MGRS-local), past r01's east end (53038.6)
- Process note: pass 2 first held whole tiles in RAM in 12 workers. The kernel OOM killer
  then took a CARLA server and four of another user's processes. It now streams 5 M-point
  slices; ten writers stayed under 4 GB in total. On this shared host, bound memory first
  and check `free -g` before adding workers
- [ ] Write a 0.2 m `pointcloud_map.pcd` (+ metadata) covering both routes from the tiles,
      with the common shift (−0.54, +0.20, +17.30) m applied (see the step 0 decision); compare NDT
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
- [x] CARLA 0.10.0 Linux package downloaded and extracted to
      `~/Downloads/Carla-0.10.0-Linux-Shipping` (2026-10-10; 10.4 GB tarball, 20 GB
      extracted; Python wheels cp38–cp312 in `PythonAPI/carla/dist`). It ships four maps
      only: `Town10HD_Opt`, `Mine_01`, `OpenDriveMap`, `EmptyMap`. **No Town01**, so the
      existing Town01 scenarios and map dirs do not carry over; a 0.10 baseline needs a
      Town10HD map dir (Lanelet2 + PCD + signal table) or the twin itself
- [x] CARLA 0.10.0 runs on this host (2026-10-11; `scripts/ntu_twin/carla010_probe.py`,
      client venv `/mnt/disk1/ntu-campus-twin/.venv-carla010`, results in
      `/mnt/disk1/ntu-campus-twin/carla010_probe/`). Start:
      `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json setsid ./CarlaUnreal.sh
      -carla-rpc-port=2100 -nosound -RenderOffScreen`. RPC is up in 15–30 s. Findings:
  - **VRAM is the constraint on this shared GPU.** Idle Town10HD takes 6.7 GB, and 10.5 GB
    with sensors. With another user holding 23.5 GB, six 1080p cameras crashed the server:
    `Out of memory on Vulkan`, then SIGSEGV. Check `nvidia-smi` before a run
  - **Frame cost, with each tick waiting for every sensor's frame:** median 38 ms for one
    1920x1080 camera plus a 128-channel lidar at 20 Hz, and 61 ms with two cameras. That is
    about 23 ms per extra camera, so an Autoware-like six-camera rig would run near
    0.33x real time at 20 Hz. Tick without sensors: 1.4 ms
  - The server lists two maps, `Town10HD_Opt` (15 traffic lights, 155 spawn points) and
    `Mine_01`. There are 11 vehicle blueprints, and `vehicle.lincoln.mkz` is present
  - `get_wheel_steer_angle()` raises `RuntimeError: std::exception` on every call. That is
    worse than the "returns 0" recorded in ssv2-feature-completeness
  - Wheel physics has no world position: `offset`, `location` and `old_location` all
    read zero. Chaos wheel fields replace PhysX's (`cornering_stiffness`, `spring_rate`,
    `suspension_*`, `wheel_load_ratio`, ...). Vehicle physics control adds
    `torque_curve`, `steering_curve`, `forward_gear_ratios` and others
  - Control works and is deterministic: throttle 0.6 reached 13.1 m/s in 5 s, and runs
    that apply control once or every tick matched exactly
  - **csb compiles unchanged against 0.10**: `CARLA_VERSION=0.10.0 cargo check` passes
    with a separate target dir. **acb fails in one place**: `carla_vehicle.rs:162-164`
    reads wheel `position` to find the rear axle. 0.10 has no such field and no non-zero
    replacement (above), so the rear-axle reference needs another source there, such as
    the bounding box or a per-blueprint table
- [ ] Fix acb's rear-axle lookup for 0.10, build both workspaces with `carla-0100`, and run
      one scenario on Town10HD_Opt. Map dir source: TUM's CARLA→Autoware pack, which has
      Town10 (Lanelet2 with 34 traffic-light tags, PCD, projector `local`). The pristine copy
      is in `/mnt/disk1/carla-maps-tum/` (from NAS `CARLA-map-from-TUM`, checksums
      verified 2026-10-11; LGPL-3.0, TUM). It is identical to `data/carla-autoware-bridge/Town10`.
      Still needed: a signal table generated against 0.10's Town10HD_Opt, and a check that
      the PCD matches 0.10's remodeled Town10
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
