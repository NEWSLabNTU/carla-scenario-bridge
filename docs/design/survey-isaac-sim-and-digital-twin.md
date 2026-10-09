# Survey: Autoware on Isaac Sim, and a fast digital-twin pipeline for our stack

**Date**: 2026-10-09, revised 2026-10-10 (NuRec ruled out; §3.5 added; §5 NTU data). **Status**: survey,
nothing built or measured on this host. Claims are from the sources linked
at the end; anything marked *unverified* is inference that a spike must confirm.

## Questions

1. What did the Autoware Off-road/Racing Working Group actually build on Isaac Sim, and how
   does it compare with this stack (SSv2 + carla-scenario-bridge + acb + CARLA 0.9.16)?
2. Which Isaac Sim features matter for us?
3. Can we go from a real site to a running scenario quickly:
   capture → point-cloud map (GLIM) → Lanelet2 map → simulator scene → realistic sensors →
   several interacting vehicles?

## Bottom line

- **The off-road demo is a vehicle sandbox, not a scenario simulator.** Two vehicles, one
  hand-built USD track, RTX sensors, Autoware drives it through `autoware_control_msgs/Control`.
  No Lanelet2, no traffic, no OpenSCENARIO/SSv2, no traffic lights. It does not replace what
  this repo does; it shows Isaac Sim can host Autoware with little glue.
- **The valuable Isaac features are rendering, not orchestration**: RTX lidar/radar/camera/
  ultrasonic sensor models with material response, and NuRec (Gaussian-splat scenes from real
  sensor logs) loaded as USDZ with a collision mesh. Isaac Sim 6.0 ships NuRec built in.
- **NuRec and Cosmos Transfer are out for us.** CARLA 0.9.16 ships NuRec through a gRPC
  render container, but the stack is too heavy for our RTX 5090 (team assessment,
  2026-10-09), and Cosmos-Transfer2.5-2B needs 65 GB VRAM minimum. Realistic rendering has to
  come from lighter routes; see §3.5.
- **The digital-twin bottleneck is the Lanelet2 map, not the point cloud.** GLIM produces an
  Autoware-ready `pointcloud_map.pcd` in hours. No mature open-source tool draws lanelets
  automatically; the realistic path is semi-manual (Vector Map Builder or MapToolbox over the
  GLIM cloud) plus FlexMap Fusion for georeferencing and OSM attributes.
- **Multi-vehicle interaction is already our strength**: SSv2 puppeteers NPCs, several
  Autoware instances drive in one CARLA world through the agent relay (roadmaps 010, 017).
  Isaac would have to re-earn all of that through a new SSv2 backend.

**Recommendation**: keep CARLA + SSv2 as the scenario backbone. Get realism from **CARLA 0.10
(UE 5.5, Lumen + Nanite) plus a real-site textured mesh** reconstructed from our own capture.
A mesh is the one representation that both the renderer and CARLA's raycast lidar see, so
camera and lidar stay consistent. Gaussian splats in UE5 are an add-on for camera background
only. Treat an Isaac Sim SSv2 backend as a later option for RTX lidar/radar fidelity, not a
rewrite.

## 1. The Off-road WG Isaac simulator

Repo: `autowarefoundation/autoware_off-road_sim` (Apache-2.0, with UPenn Autoware CoE).

| Aspect | What it does |
|---|---|
| Base | Isaac Sim 6.0 early-developer build (5.1 optional), built from source in Docker with ROS 2 Humble; first build 30–60 min |
| Launcher | `scripts/launch_sim.py` under Isaac's Python: loads env USD, spawns vehicles, remaps topics, starts ROS 2 bridge, injects keyboard ActionGraph |
| Vehicles | 1/5-scale RoboRacer-Max USD; Ego + Opponent share one asset, isolated by topic prefix |
| Sensors | 3D lidar, RGB camera, IMU, odometry (RTX renderer); GNSS via a separate rclpy bridge at ~10 Hz; per-vehicle enable flags |
| Autoware interface | Subscribes `/<prefix>/control` (`autoware_control_msgs/Control`), converts to Ackermann; also raw `AckermannDriveStamped` |
| Environment | `pumptrack_simple.usd`, non-planar; swap via config; per-material friction overrides |
| Physics | PhysX, PGS solver (TGS called unstable for racing), 60 Hz default, real-time factor printed |
| Distribution | Autoware on another PC or Jetson over CycloneDDS multicast (HIL) |
| Extras | Headless mode, split-screen, semantic-segmentation recording |
| Hardware | RTX 4070+, driver 580+, 45 GB disk |

What it lacks, relative to us:

- No scenario layer. Nothing like SSv2, no success/failure conditions, no repeatable runs.
- No Lanelet2 or road network; off-road ODD does not need it.
- No vehicle status interface comparable to acb's `/vehicle/status/*`; odometry only.
- Free-running physics with a printed RT factor; no lockstep tick the way we run CARLA
  synchronous (see [time-and-ticking.md](time-and-ticking.md)).
- Shader compile minutes on first launch; Isaac 6 EA still churning APIs.

Worth borrowing: per-vehicle topic-prefix remapping of a single USD asset, and the
semantic-segmentation recorder for training data.

## 2. Isaac Sim features relevant to us

| Feature | Isaac Sim | CARLA 0.9.16 (ours) |
|---|---|---|
| Lidar | RTX lidar: ray-traced, models returns off transparent/reflective materials, JSON profiles for real units, rotating and solid-state | Raycast lidar; semantic lidar; no material response |
| Radar | RTX radar: emissivity/reflectivity in RF band (needs Motion BVH) | Simple raycast radar |
| Ultrasonic | RTX acoustic sensor (6.0 experimental API) | None |
| Camera | RTX path-traced/real-time camera, optical params, Motion BVH motion blur | UE4 raster camera; Cosmos Transfer post-hoc restyle |
| Real-scene rendering | NuRec 3DGUT/3DGRT USDZ native in renderer (5.0+, built in 6.0); mesh for collision | NuRec via gRPC container, camera replay example |
| Physics | PhysX 5, plus Newton backend in 6.0 | PhysX via UE4 vehicle model |
| ROS 2 | Built-in bridge (Humble, Jazzy) via OmniGraph | None built in; acb does it |
| Traffic / road network | None | OpenDRIVE, Traffic Manager, traffic lights |
| Scenario | None (Replicator is for data generation) | ScenarioRunner; we use SSv2 |
| Map import | Any USD/USDZ: drag in | fbx + xodr through `make import` (source build) or Docker import for packaged builds |

Notes on the sensor API: Isaac 5.0's `isaacsim.sensors.rtx` is deprecated in 6.0 in favour of
`isaacsim.sensors.experimental.rtx`, which covers lidar, radar, acoustic and camera. Expect to
pin a version.

## 3. Digital twin pipeline

Target: a site we can drive in simulation with Autoware localizing on a real PCD and SSv2
routing on a real Lanelet2 map.

```
capture (lidar + IMU [+ cams + RTK GNSS])
   |
   +-- GLIM ------------------> pointcloud_map.pcd + poses          [Autoware localization]
   |      |
   |      +-- FlexMap Fusion (kiss_icp_georef) --> georeferenced PCD + map_projector_info.yaml
   |
   +-- Lanelet2 authoring over the PCD --> lanelet2_map.osm          [SSv2 + Autoware planning]
   |      (Vector Map Builder / MapToolbox, then FlexMap Fusion for OSM attributes,
   |       autoware_lanelet2_validation to check)
   |
   +-- appearance (§3.5):
   |      a) textured mesh (colorized GLIM cloud, Meshroom, MILo) --> .fbx + .xodr
   |                                                   [CARLA 0.10 import, Nanite]  <- main route
   |      b) gsplat splats over the mesh, camera only  [UE5 plugin, YaGS]          <- optional
   |      (NuRec USDZ: Isaac-native / CARLA gRPC -- too heavy for our GPU, §3.4)
   v
simulator scene + Autoware map dir (layout in user-guide.md §2)
```

### 3.1 Point-cloud map: GLIM

GLIM (koide3, `glim` / `glim_ros2` / `glim_ext`) is GPU-accelerated lidar(-inertial) mapping:
fixed-lag smoothing odometry, keyframe matching, global submap optimization, interactive
manual correction of failures. Works with spinning, Livox and solid-state lidars and RGB-D;
tested on Ubuntu 22.04/24.04 and Jetson Orin; Docker image `koide3/glim_ros2`.

Fit for us: its dense map exports to PCD, which is Autoware's `pointcloud_map.pcd`.
Autoware also wants `pointcloud_map_metadata.yaml` (tiling) and a projector; tiling needs
Autoware's `pointcloud_divider` *(unverified that GLIM output needs downsampling first;
NDT expects ~0.2 m voxels)*. Georeferencing: FlexMap Fusion's `kiss_icp_georef` aligns SLAM
poses and PCD to RTK GNSS without needing a lanelet map.

### 3.2 Lanelet2 map: the slow part

| Option | Automation | Comment |
|---|---|---|
| TIER IV Vector Map Builder | Manual over PCD | Official route; web tool; lanes, stop lines, lights |
| MapToolbox (Unity plugin) | Manual over PCD | Open source; Autoware docs name it |
| JOSM | Manual | Needs Autoware-specific fixes; docs advise against |
| FlexMap Fusion (TUM) | Automatic *enrichment* | Georeferences + conflates OSM attributes into an existing lanelet map |
| SZE ROS 2 semi-automatic framework (2025) | Semi-automatic | Drivable-area segmentation → Lanelet2 primitives; claims open source, repo not checked |
| CommonRoad Scenario Designer | Conversion | OpenDRIVE ↔ CommonRoad ↔ Lanelet2; usable to move between CARLA xodr and lanelet *(output vs Autoware tags unverified)* |

No tool found that turns a GNSS track or point cloud into an Autoware-valid Lanelet2 map
without a human. For small sites (campus, test track) manual drawing in Vector Map Builder is
hours, not weeks; it is the pragmatic choice. A cheap accelerator worth a spike: generate
centerline-based lanelets from driven GLIM trajectories with the Lanelet2 Python API, then
fix by hand.

### 3.3 Getting the scene into a simulator

**CARLA** needs geometry (`.fbx`) and road network (`.xodr`). The xodr carries waypoints,
Traffic Manager lanes and traffic-light actors; our bridge's signal table
(`--generate-signal-table`) maps Lanelet2 signals onto those. Import is `make import` in a
source build, or a Docker import for packaged builds (no editor). The 0.9.16 Digital Twin
Tool builds maps procedurally from OSM, which gives roads but not the real site's look.
Mesh-from-point-cloud pipelines (OpenTwinMap, TU Wien) exist but are research grade.

Practical consequence: every twin needs both a lanelet map and an xodr that agree. Author
one, convert the other (CommonRoad Scenario Designer), and check alignment with the signal
table generator, which already refuses mismatches.

**Isaac Sim** takes any USD/USDZ. A NuRec USDZ drops in with its collision mesh. No road
network is needed by the simulator itself, because SSv2 routes on Lanelet2. Traffic lights
would be scene prims we toggle.

### 3.4 Sensor rendering: what exists, and why the NVIDIA neural routes are out

The Cosmos and NuRec rows are ruled out on this host: NuRec is too heavy for one RTX 5090,
Cosmos-Transfer2.5-2B needs 65 GB VRAM. The Isaac RTX rows stay open as the long-term option.

| Route | Cameras | Lidar | Closed-loop with Autoware | Effort |
|---|---|---|---|---|
| CARLA raster sensors (today) | Synthetic look | Raycast, no materials | Yes | 0 |
| CARLA + Cosmos Transfer1 | Restyled video, offline | — | No (post-process) | Low; dataset use only |
| CARLA + NuRec gRPC | Real-scene splats | NuRec renderer supports lidar sweeps (`render-grpc --lidar`) but CARLA example shows cameras only | *Unverified*: example is replay; gRPC renders arbitrary poses, so live ego rendering looks feasible | Medium: acb would call the gRPC renderer per frame instead of CARLA cameras |
| Isaac RTX sensors on mesh scene | Path-traced | Material-aware RTX lidar, radar | Yes (ROS 2 bridge) | High: new simulator backend |
| Isaac RTX + NuRec scene | Real-scene splats | RTX lidar hits collision mesh *(unverified whether RTX lidar sees splats)* | Yes | High |

Constraints on NuRec, for the record: own reconstructions need data converted to NCore V4
and run through the NuRec container (NGC); sample AV scenes are on Hugging Face (full set ~1.5 TB; this host has
254 GB free on `/home`, so pull single scenes). Dynamic objects in a capture are baked tracks;
inserting new actors means compositing synthetic objects, and their lighting will not match
the splats exactly.

### 3.5 Realistic rendering without NuRec

Constraint: one RTX 5090 (32 GB), Linux, closed loop with Autoware at sensor rate, and both
CARLA versions available to the bridge (carla-rust feature `carla-0916` default, `carla-0100` selectable; roadmap 018).

| Route | Camera realism | Lidar consistency | Closed loop | VRAM / effort | Verdict |
|---|---|---|---|---|---|
| **A. CARLA 0.10 (UE 5.5)** stock | Lumen GI + Nanite; big step over UE4 | Same raycast lidar | Yes | ≥16 GB recommended; low effort (bridge already compiles for 0.10) | **Do first** |
| **B. Real-site textured mesh in CARLA 0.10** | Real geometry + real texture, lit by Lumen | Lidar hits the same mesh: consistent | Yes | Medium: reconstruct, clean, import as Nanite | **Main route** |
| C. 3DGS plugin in UE5 (YaGS, SplatRenderer, UnrealSplat) | Photoreal background | Splats are not physics; lidar needs a collision mesh anyway | Yes | Plugin port into CARLA's UE fork; Windows/DX12-first plugins | Optional layer on B |
| D. Image post-process (CARLA2Real / EPE) | Restyles toward Cityscapes/KITTI look, not *our* site | Unchanged | Near: ~13 FPS reported | Medium; 0.9.x tool | Training data, not closed loop |
| E. External gsplat renderer (HUGSIM/SplatAD-style) | Photoreal from our capture | SplatAD renders lidar with intensity and ray drop | Possible; actors must be composited | High: new render service called by acb per frame | Research track |
| F. Cosmos Transfer / NuRec | — | — | — | 65 GB+ / too heavy | Out |

**A. CARLA 0.10.** UE 5.5 with Lumen and Nanite, remodeled vehicles, upgraded Town10 and a
Mine map; 16 GB VRAM and driver ≥550 recommended, 130 GB disk. Known costs for us:
- 0.10 uses Chaos vehicle physics, not 0.9's PhysX model. acb's steering model, the measured
  steering curve (roadmap 014 gap 8) and the longitudinal tuning are 0.9.16 measurements and
  must be redone. The 0.10 steering-angle getter returns 0 (ssv2-feature-completeness.md).
- Town availability differs from 0.9.16; our scenarios and map dirs are Town01-based.
  *Check which towns 0.10.x ships before porting a scenario.*
- Traffic-light signal tables must be regenerated per map; the generator already refuses
  stale tables.

**B. Real-site mesh.** Ways to get a textured mesh from a capture, cheapest first:

| Source | Tool | Notes |
|---|---|---|
| Lidar + cameras (our rig) | GLIM map → colorize points from cameras (pose-optimized, e.g. OmniColor) → TSDF/Poisson mesh → bake texture | Geometry is metric and matches `pointcloud_map.pcd`, so NDT localizes on the same world it sees. Texture is the weak part |
| Photos/video | Meshroom (AliceVision): open source, Linux | Photo-only; scale and georeference via GLIM poses |
| Photos + lidar | RealityScan 2.0 (ex-RealityCapture): free under revenue threshold, imports LAS/LAZ/E57 | Best quality; Windows, Linux unverified |
| Photos → splats → mesh | gsplat / 2DGS / PGSR / MILo mesh extraction | MILo gives compact detailed meshes; designed for object/indoor captures, so tile a driving scene |

Then: center at origin, split into tiles for large sites, import as Nanite static meshes,
pair with the xodr from §3.2. CARLA import still needs a source build of CARLA (or the
Docker importer for packaged builds) — budget for that once.

**C. Splats inside UE5.** Open plugins exist for UE 5.5: YaGS (Yandex, Apache-2.0,
5.5–5.7, DX12 or Vulkan, correct depth against meshes), SplatRenderer (Apache-2.0, DX12,
2M+ splats at 100+ FPS on an RTX 4080), UnrealSplat (MIT, Niagara, early). CARLA 0.10 on
Linux renders with Vulkan, so YaGS is the only candidate with a stated Vulkan path.
Splats render only for cameras; CARLA's lidar raycasts physics geometry, so B's mesh stays
underneath as collision and lidar target. Training splats on our capture fits a 5090
(gsplat). *Porting a plugin into CARLA's UE fork is unverified.*

**D. Post-process.** CARLA2Real wraps Intel's EPE with CARLA G-buffers, ~13 FPS. It shifts
appearance toward a public dataset's style, not toward our site, and is too slow for 10 Hz
multi-camera closed loop. Useful for perception training data only.

**E. External splat renderer.** HUGSIM shows closed-loop 3DGS driving simulation (ego
re-rendered from control commands); SplatAD's gsplat extension renders lidar (intensity,
ray drop, rolling shutter) and cameras. For us this means acb sends ego and NPC poses to a
render service instead of reading CARLA sensors, and NPCs must be composited. Highest
realism for our own site and the most new code.

### 3.6 Multi-vehicle interaction

| | Ours (CARLA) | Off-road Isaac sim |
|---|---|---|
| Scenario-driven NPCs | SSv2 puppeteers via `set_transform` (kinematic, collidable) | None |
| Several Autoware egos | Yes: per-domain stacks, agent relay, background AVs as SSv2 entities with controller `agent` (017) | Two vehicles by topic prefix, no orchestration |
| Autopilot traffic | CARLA Traffic Manager | None |
| Ground truth collision verdict | SSv2 bounding boxes | None |

An Isaac backend would keep this by implementing SSv2's `simulation_interface` the way this
bridge does: same 14 handlers, `set_transform` → USD prim pose, `UpdateFrame` → stepping the
Isaac world. Isaac's Python API can step deterministically *(unverified at Autoware's sensor
rates with RTX on; RTX lidar needs one render product per sensor)*.

## 4. Proposed spikes

Ordered by value per effort. Each ends in a measurement, not an opinion.

1. **CARLA 0.10 baseline.** Bring up 0.10.x on this host, build with the `carla-0100` feature,
   run one SSv2 scenario on a 0.10 town. Measure tick time with the full ego sensor set and
   VRAM headroom; list what breaks in acb (steering, Chaos tuning).
2. **GLIM → Autoware map dir.** One capture of a known site (campus loop). GLIM → PCD →
   divider → `kiss_icp_georef`. Pass: Autoware NDT localizes on it in a rosbag replay.
3. **Mesh from the same capture.** Colorized GLIM cloud → mesh → texture; in parallel one
   photo route (Meshroom or MILo). Compare visually and by triangle count.
4. **Lanelet2 over the GLIM cloud.** Vector Map Builder by hand; time it. Validate with
   `autoware_lanelet2_validation`; convert to xodr.
5. **Twin into CARLA 0.10.** Mesh (Nanite) + xodr → import → `--generate-signal-table` →
   one SSv2 scenario with an NPC. Pass: scenario completes and the ego localizes on the real
   PCD inside the simulated twin.
6. **Splat layer, only if 5 looks too synthetic.** YaGS in CARLA's UE fork with a gsplat
   reconstruction over the mesh; check camera FPS and lidar unchanged.
7. **Isaac feasibility, only if lidar/radar fidelity becomes the blocker.** Run the off-road
   sim; prototype a minimal SSv2 backend (Initialize, Spawn, UpdateEntityStatus, UpdateFrame)
   against Isaac Python and measure lockstep tick time with one RTX lidar + 6 cameras.

## 5. First site: NTU campus (data on hand, 2026-10-10)

Local copy and provenance: `/mnt/disk1/ntu-campus-twin/README.md`.

### What we have

| Source | Content | Frame | Notes |
|---|---|---|---|
| Autoware map `2026-04_ntu_map` r01, r02 | Lanelet2 + PCD (x y z intensity, no color), 4.1 M and 2.5 M points | MGRS 51RUH | Lanelets are `road` with thin solid lines only; **no regulatory elements** (lights, stop lines, crosswalks) |
| Vendor MLS `2026-05-15`, `台大las.laz` (51 GB) | ~5.52 G points, **RGB**, GPS time, ASPRS classes | TWD97/TM2 (EPSG:3826) | LiDAR360MLS; 1113 m x 649 m; covers both routes |
| Earlier CARLA experiment (June 2026) | 0.05 downsample imported into CARLA UE4 as a point-cloud asset; RGB + lidar videos | — | See below |
| `2026-08-20 GLIM pointcloud mapping bags` | Basement, lidar + IMU only | — | Not campus; no images |

No camera images of the campus are on hand. LiDAR360MLS systems colorize from a panoramic
camera, so the vendor almost certainly has the images and the camera trajectory.

### Findings

1. **The vendor's derived files are truncated.** The LAZ is LAS 1.2, whose 32-bit header
   count saturates at 4 294 967 295. The PDAL-made `tiles_50m`, `tiles_100m` and `downsample`
   sets each sum to exactly that count (or a downsample of it) and end at X = 304653. That
   loses ~22 % of the points, and the east ~200 m of route r01 (it runs to X 304853).
   Regenerate from the LAZ with a reader that streams to end of file.
2. **The vendor cloud aligns with our Autoware map by a constant shift.** TWD97 → UTM 51N →
   MGRS-local, then subtract (−0.288, +0.216, +17.26) m. After the shift, r02's PCD matches
   the vendor cloud: median 0.068 m, p90 0.227 m, 87 % within 0.2 m. The 17 m in Z is a
   vertical datum difference. So the colored cloud, and anything built from it, drops into the
   frame Autoware already localizes in. Only translation was solved. On r01 (sampled from the
   truncated tiles) the shift solves to (−0.477, +0.184, +17.27) m, within 0.19 m of r02's,
   with median 0.128 m but p90 5.1 m. The tail is likely r01 map areas missing from the
   sampled vendor tiles; recheck once full tiles exist (next step 1).
3. **The vendor RGB is usable.** Ground points are mid-grey (mean RGB 94/90/84), not white.
   The white floor in the earlier CARLA clips is the scene, not the data.
4. **The earlier CARLA import proves the gap §3.5 predicts.** Point sprites give a recognisable
   but noisy camera view, and CARLA's raycast lidar returns only the floor: the
   point-cloud asset has no collision. Camera and lidar disagree completely.
5. **Classes make a split scene cheap.** In a 6-tile sample: high vegetation 46 %,
   unclassified 40 %, ground 5 %, building 4 %, low vegetation 3 %. NTU's roads are lined
   with trees, so vegetation dominates.

### Next steps for this site

Tracked as roadmap phase [019](../roadmap/019-ntu-campus-twin.md).

Each produces something we can look at in CARLA, in order:

1. **Regenerate full tiles** from the LAZ in the Autoware frame (shift applied), keeping
   RGB and class. Also write a 0.2 m `pointcloud_map.pcd` covering both routes and compare
   it with r01/r02 for localization.
2. **Road surface from class 2.** 2.5D Delaunay or heightfield mesh, textured by a top-down
   orthophoto rasterized from ground-point RGB at 2–3 cm. Lane paint should survive at that
   density. This replaces the white floor and gives the lidar the real road.
3. **Buildings from class 6.** Poisson mesh per tile, vertex colour or baked texture.
4. **Vegetation (classes 3, 5).** Keep as points for the camera (UE point-cloud plugin, or
   splats later), with coarse convex or voxel proxies as collision so the lidar sees trees.
   Instanced UE foliage matched to trunk positions is the alternative if points look poor.
5. **Unclassified (40 %).** Inspect: poles, signs, parked cars, people. Parked cars and
   pedestrians are baked into the scan and must be removed or accepted as static.
6. **Lanelet2 completion.** Add traffic lights, stop lines and crosswalks to r01/r02, then
   produce an xodr that agrees with it for CARLA.
7. **Import into CARLA 0.10 first** (Nanite meshes, Lumen) and into 0.9.16 only for comparison.
   Measure: camera frames vs the vendor render, and lidar scans in CARLA vs the real cloud at
   the same pose (the real cloud is the ground truth for the lidar).
8. **Ask the vendor for the panoramic images and camera trajectory.** They unlock better
   textures and a gsplat reconstruction for the camera layer (§3.5 C/E). This is the
   single most valuable data request.

## Sources

- Off-road sim: <https://github.com/autowarefoundation/autoware_off-road_sim>
- WG meeting notes: <https://github.com/orgs/autowarefoundation/discussions/6728>,
  <https://github.com/orgs/autowarefoundation/discussions/6403>
- Isaac RTX sensors: <https://docs.isaacsim.omniverse.nvidia.com/5.0.0/sensors/isaacsim_sensors_rtx.html>,
  <https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/index.html>,
  <https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_rtx_lidar.html>
- Isaac 6.0 release notes: <https://docs.isaacsim.omniverse.nvidia.com/6.0.1/overview/release_notes.html>,
  <https://radiancefields.com/nvidia-s-isaac-sim-6.0-ships-with-nurec-gaussian-splatting>
- Isaac NuRec assets: <https://docs.isaacsim.omniverse.nvidia.com/6.1.0/assets/usd_assets_nurec.html>,
  <https://developer.nvidia.com/blog/how-to-instantly-render-real-world-scenes-in-interactive-simulation>
- NuRec: <https://docs.nvidia.com/nurec/index.html>, <https://github.com/nv-tlabs/3dgrut>
- CARLA 0.9.16 NuRec/Cosmos: <https://carla.org/2025/09/16/release-0.9.16>,
  <https://carla.readthedocs.io/en/latest/nvidia_nurec/>,
  <https://carla.readthedocs.io/en/latest/ai_rendering/>
- CARLA map import: <https://carla.readthedocs.io/en/0.9.16/tuto_content_authoring_maps/>,
  <https://www.mathworks.com/help/roadrunner/ug/export-to-carla.html>,
  OpenTwinMap <https://arxiv.org/html/2511.21925v1>
- CARLA 0.10: <https://carla.org/2024/12/19/release-0.10.0>,
  <https://carla-ue5.readthedocs.io/en/latest/start_quickstart/>
- UE5 splat plugins: <https://radiancefields.com/yags-yandex-open-sources-a-3dgs-plugin-for-unreal-engine-5.5%E2%80%935.7>,
  <https://radiancefields.com/splatrenderer-plugin-for-unreal-engine>, <https://github.com/JI20/unreal-splat>
- Mesh reconstruction: MILo <https://arxiv.org/abs/2506.24096v2>, Meshroom
  <https://hal.archives-ouvertes.fr/hal-03351139>, RealityScan 2.0
  <https://www.realityscan.com/news/realityscan-20-new-release-brings-powerful-new-features-to-a-rebranded-realitycapture>,
  OmniColor <https://arxiv.org/html/2404.04693v2>
- Splat simulators: HUGSIM <https://arxiv.org/abs/2412.01718v1>, SplatAD <https://arxiv.org/abs/2411.16816>
- Post-process: CARLA2Real <https://arxiv.org/abs/2410.18238v1>
- Cosmos Transfer2.5 VRAM: <https://docs.nvidia.com/cosmos/latest/transfer2.5/model_matrix.html>
- GLIM: <https://github.com/koide3/glim>, <https://arxiv.org/abs/2407.10344>
- Lanelet2 tooling: <https://docs.autoware.org/main/tutorials/integrating-autoware/creating-maps/>,
  FlexMap Fusion <https://arxiv.org/html/2404.10879v2>,
  SZE framework <https://papers.sze.hu/research-portal/publications/10.3390/engproc2025113013>,
  <https://github.com/zubxxr/av-map-creation-workflow>,
  CommonRoad Scenario Designer <https://pypi.org/project/commonroad-scenario-designer/0.7.2>
