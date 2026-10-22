"""Keep the two-wheelers' riders' grip as arrays (plain pxr and numpy; `isaac_vehicles.sh
riders:=true`, roadmap 044 open item 2). vehicle_usd_assets.py reads what this writes.

    python -I rider_usd_assets.py $CSB_VEHICLE_USD/riders

From `<root>/rider_grip/rider_grip.usd` (the SimReady map export of a level holding a rider
playing CARLA's `AS_Pedestrian_BikeHands`, a static pose) writes `<root>/grip.npz`: `joints`
(the rider skeleton's joint paths) and `rotations` (joints x 4, x y z w), its first frame.
"""
import sys
from pathlib import Path

import numpy as np
from pxr import Usd, UsdSkel

root = Path(sys.argv[1])
usd = root / "rider_grip" / "rider_grip.usd"
stage = Usd.Stage.Open(str(usd))
prim = next((p for p in stage.Traverse() if p.IsA(UsdSkel.Animation)), None)
if prim is None:
    sys.exit(f"rider_usd_assets: no SkelAnimation in {usd}")
a = UsdSkel.Animation(prim)
times = a.GetRotationsAttr().GetTimeSamples() or [Usd.TimeCode.Default()]
joints = np.array([str(j) for j in a.GetJointsAttr().Get()])
rotations = np.array(a.GetRotationsAttr().Get(times[0]), dtype=np.float32)
np.savez(root / "grip.npz", joints=joints, rotations=rotations)
print(f"grip: {len(joints)} joints from {usd} -> {root / 'grip.npz'}")
