# Phase 040: NTU Off-road Test Field on Isaac Sim

The off-road evaluation field as an Isaac world: the vendor's NTU mesh imported as USD with
colliders and terrain materials, aligned to our Autoware maps so Autoware localizes on the
real point cloud, with the Lanelet2 trail map SSv2 routes on.

**Source**: user direction 2026-10-16 (the NTU mesh is processed by a vendor; the field is
the competition venue).
**Depends on**: [019](019-ntu-campus-twin.md) (data, frame alignment, Lanelet2 work),
[037](037-isaac-world-server.md) (`LoadWorld`), [038](038-isaac-vehicle-bridge.md) (ego and
sensors to measure with).
**Feeds**: [041](041-competition-director.md).

## Inputs

- **Vendor mesh** (pending): ask for, per tile, a textured mesh (glTF or USD) in a stated CRS,
  ground and non-ground separate (or labelled from the ASPRS classes), with collision-grade
  ground geometry (watertight, no holes under the drivable area), texture resolution stated,
  and a per-tile manifest. Agree the tiling with our 50 m MGRS tiles (019) or provide the
  transform.
- **Our data** (019): vendor MLS (EPSG:3826, RGB, classes), re-tiled to MGRS 51RUH local;
  Autoware maps r01 and r02 (PCD, Lanelet2, MGRS); measured shift vendor → map frame
  T = (-0.288, +0.216, +17.26) m (translation-only ICP on r02).

## Design

- **Frame**: Isaac world origin at the map frame origin of the chosen route (MGRS local),
  so coordinates stay within ~1 km of the origin (precision loss far from it was reported for
  USDZ imports). The vendor mesh is transformed EPSG:3826 → MGRS local → minus T, the same
  chain 019 measured; a check script compares mesh height to the Autoware PCD.
- **Physics**: ground tiles get colliders (triangle mesh or heightfield, whichever PhysX
  handles at the tile count with real-time stepping); buildings and trunks get colliders;
  foliage gets none (it should not stop a vehicle) but stays visible to RTX lidar.
- **Terrain materials**: friction zones (asphalt, gravel, grass, mud) as PhysX materials on
  ground regions, painted from the vendor's classes or drawn by hand; these drive the
  off-road difficulty and the wheel model in 038.
- **Route network**: Lanelet2 lanelets along the off-road course (trails), drawn over the
  PCD as in 019's semi-manual path; SSv2 needs it for routing and entity placement, Autoware
  for planning.
- **Realism checks**: RTX lidar against the real cloud (point-to-mesh residual along the
  route); camera frames against the vendor's own renders as a reference (no campus photos to
  compare with, 019). Optional later layer: coloured points as Gaussian splats
  (ParticleField) for camera background only.
- **Asset handling**: the USD world lives on `/mnt/disk1` with a payload per tile, loaded by
  path from the scenario's map dir (`LoadWorld`).

## Steps

- [ ] Write the vendor requirements above into the request; record what is delivered.
- [ ] Import script: vendor tiles → USD stage with transforms, colliders, materials; idempotent.
- [ ] Alignment check against the Autoware PCD; residual in this doc.
- [ ] Lanelet2 course map for the competition route; validated in SSv2 and Autoware.
- [ ] Autoware localizes and drives the course on Isaac (038's ego), RTX lidar residual
      measured.
- [ ] Frame time on the full field with one and with N egos (039's measurement on the real
      world).

## Acceptance

- Autoware drives the course end to end on the field, localizing on the real PCD (NDT
  converged throughout), in a scenario with SSv2's verdict.
- Mesh-to-PCD height residual and RTX-lidar-to-PCD residual recorded; median under 0.1 m on
  the drivable area.
