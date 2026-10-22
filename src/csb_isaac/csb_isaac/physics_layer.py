"""Static colliders for a CARLA town exported to USD (roadmap 043):
`python -m csb_isaac.physics_layer <Town>.usd [-o <Town>_physics.usda]`.

CARLA 0.9.16's SimReady exporter (`UE4Editor-Cmd ... -run=SimReadyExport -COLLISION=true`)
parses its "export physics data" switch into `bExportPhysics`, and nothing reads it: the
stage has no UsdPhysics at all, and a car dropped on it falls through the road. This writes
`<Town>_physics.usda` next to the export: a text layer that sublayers it and applies
`PhysicsCollisionAPI` + `PhysicsMeshCollisionAPI` (approximation `none`: the triangle mesh
itself, static, as CARLA's own complex collision is) to its meshes. Nothing is copied; the
export stays untouched and can be re-exported under the layer.

Left without a collider:

- whatever sits under a `PointInstancer`: SimReady writes CARLA's foliage (grass, bushes,
  trees placed with the foliage tool) as instancers, and CARLA does not collide with them
  either; a triangle mesh per blade of grass would also cost PhysX more than the town.
- the far scenery and water: `BackgroundPlanes` (the skyline cards around the map), Town01's
  `Lake` tiles and Town10HD's sea (`SM_seaM`, `SM_sea2M`), which nothing drives on and whose
  huge flat triangles only slow the broad phase.
- decals: Town01's `Decals` (dirt and debris laid on the road) and the `SM_DirtDebris*` meshes
  of other towns, render-only in CARLA. As triangle meshes a few millimetres over the road
  they were seams a tracked NPC's box caught on, and its contacts with them read as hits on
  the static world (roadmap 044).
- traffic light heads (`GreenLight`, which holds the nested yellow and red bulbs): their
  lenses' materials are switched per signal state (roadmap 044), and nothing drives 3 m up.
- invisible meshes: CARLA's crosswalk areas (2 mm planes over the road that its pedestrian
  navigation is generated from) and the like. What the lidar cannot see should not hold a
  car up either.

The layer is the file the Isaac world names (`csb_isaac.usd_world`).

    python -m csb_isaac.physics_layer /mnt/disk1/carla-usd/Town02/Town02.usd
"""
import argparse
import os
import sys

# Scenery left without colliders: prims of these names, and of names with these prefixes.
SKIPPED_NAMES = {"Lake", "Decals", "GreenLight"}
SKIPPED_PREFIXES = ("BackgroundPlanes", "SM_sea", "SM_DirtDebris", "dirtDebris")


def skipped(path):
    """True for a prim at or under a scenery prim that should not collide."""
    return any(part in SKIPPED_NAMES or part.startswith(SKIPPED_PREFIXES)
               for part in str(path).split("/"))


def write_layer(src, out):
    """Write the collider layer `out` over the export `src`; (meshes, instancers skipped)."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(src)
    if not stage or not stage.GetDefaultPrim():
        raise ValueError(f"{src}: no stage or no default prim")
    if os.path.exists(out):
        os.remove(out)
    layer = Sdf.Layer.CreateNew(out)
    rel = os.path.relpath(os.path.realpath(src), os.path.dirname(os.path.realpath(out)))
    layer.subLayerPaths.append(rel if rel.startswith(".") else "./" + rel)
    over = Usd.Stage.Open(layer)
    over.SetEditTarget(Usd.EditTarget(layer))
    over.SetDefaultPrim(over.GetPrimAtPath(stage.GetDefaultPrim().GetPath()))
    # The export's own units and axis (SimReady: metres, Z up), stated again: a stage's
    # metadata comes from its root layer, which is this one.
    UsdGeom.SetStageMetersPerUnit(over, UsdGeom.GetStageMetersPerUnit(stage))
    UsdGeom.SetStageUpAxis(over, UsdGeom.GetStageUpAxis(stage))
    meshes = instancers = 0
    it = iter(stage.Traverse())
    for prim in it:
        if prim.IsA(UsdGeom.PointInstancer):
            it.PruneChildren()
            instancers += 1
            continue
        if skipped(prim.GetPath()) or (
                prim.IsA(UsdGeom.Imageable)
                and UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible):
            it.PruneChildren()
            continue
        if prim.IsA(UsdGeom.Mesh):
            q = over.GetPrimAtPath(prim.GetPath())
            UsdPhysics.CollisionAPI.Apply(q)
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr(UsdPhysics.Tokens.none)
            meshes += 1
    layer.Save()
    return meshes, instancers


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("usd", help="the town's SimReady export, <Town>.usd")
    ap.add_argument("-o", "--output", default="",
                    help="the layer to write (default <Town>_physics.usda beside the export)")
    args = ap.parse_args(argv)
    src = os.path.realpath(args.usd)
    stem = os.path.splitext(os.path.basename(src))[0]
    out = args.output or os.path.join(os.path.dirname(src), f"{stem}_physics.usda")
    try:
        meshes, instancers = write_layer(src, out)
    except ValueError as e:
        sys.exit(str(e))
    print(f"{out}: colliders on {meshes} mesh(es); {instancers} PointInstancer(s) left without")


if __name__ == "__main__":
    sys.exit(main())
