"""Step responses of the Isaac ego (csb_isaac's PhysX vehicle and actuation) for the same
command sequences as carla_step.py: the world as the world server builds it (flat ground),
the ego spawned as csb spawns it, commands put straight into the ego's command slot (what its
/control/command/control_cmd subscriber does), 0.05 s frames of 3 PhysX steps as csb steps it.

    source src/csb_isaac/scripts/isaac_env.sh
    python isaac_step.py EGO_YAML OUT_DIR LABEL

EGO_YAML: an ego YAML with `lidar: {enabled: false}` and a private `domain`. Experiments:
HI_SUB / LO_SUB (PhysX vehicle substeps), ACT_DEN (active slip denominator), THRESH,
ONLY=test,... override what vehicle.py sets.
"""
import csv
import logging
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).parent))
from step_common import DT, TESTS, command  # noqa: E402

EGO, OUT, LABEL = sys.argv[1], Path(sys.argv[2]), sys.argv[3]
OUT.mkdir(parents=True, exist_ok=True)
logging.basicConfig(level=logging.INFO, stream=sys.stdout,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
os.environ.pop("DISPLAY", None)
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
from csb_isaac.vehicle import GEAR_DRIVE  # noqa: E402
from csb_isaac.world import IsaacWorld  # noqa: E402

world = IsaacWorld(ego_config=EGO)
SUB = 3
try:
    only = os.environ.get("ONLY", "").split(",")
    for n, (test, length) in enumerate(TESTS.items()):
        if only != [""] and test not in only:
            continue
        handle = 10 + n
        world.spawn(handle, "ego", "EGO", "", (1.5, 0.0, 0.9), (4.8, 2.1, 1.5),
                    ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        ego = world.egos[handle]
        if os.environ.get("HI_SUB"):
            prim = world.stage.GetPrimAtPath(ego.path)
            prim.GetAttribute("physxVehicle:highForwardSpeedSubStepCount").Set(int(os.environ["HI_SUB"]))
            prim.GetAttribute("physxVehicle:lowForwardSpeedSubStepCount").Set(int(os.environ.get("LO_SUB", 3)))
        if os.environ.get("ACT_DEN"):
            prim = world.stage.GetPrimAtPath(ego.path)
            prim.GetAttribute("physxVehicle:minActiveLongitudinalSlipDenominator").Set(float(os.environ["ACT_DEN"]))
        if os.environ.get("THRESH"):
            prim = world.stage.GetPrimAtPath(ego.path)
            prim.GetAttribute("physxVehicle:subStepThresholdLongitudinalSpeed").Set(float(os.environ["THRESH"]))
        for _ in range(40):  # settle on the hold brake
            world.step({}, SUB, DT / SUB, False)
        rows = []
        t = 0.0
        while t < length - 1e-9:
            v = ego.state.velocity[0]
            a, vel, angle = command(test, t, v)
            c = ego.ros.cmd
            c.acceleration, c.velocity, c.tire_angle, c.gear, c.received = a, vel, angle, GEAR_DRIVE, True
            world.step({}, SUB, DT / SUB, False)
            t += DT
            s = ego.state
            rows.append((round(t, 3), s.velocity[0], s.angular_velocity[2], a, angle, s.position[0],
                         s.position[1], s.position[2]))
        with open(OUT / f"isaac_{LABEL}_{test}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "v", "yaw_rate", "cmd_accel", "cmd_tire_angle", "x", "y", "z"])
            w.writerows(rows)
        print(test, "done; final v %.2f" % ego.state.velocity[0], flush=True)
        world.despawn(handle)
finally:
    world.close() if hasattr(world, "close") else None
    app.close()
