"""Phase 036 spike: several ROS 2 domains from one Isaac Sim process.

Two vehicles (moving cubes); vehicle k gets its own action graph whose ROS2 Context has
useDomainIDEnvVar=false and domain_id k, publishing /clock and /tf (the vehicle's pose) in that
domain. Pass: on domain k a ROS 2 client sees only vehicle k's frame. Runs for --seconds of
sim time after printing READY, then exits.

    python domains.py --domains 1 2 --seconds 60
"""
import argparse
import time

ap = argparse.ArgumentParser()
ap.add_argument("--domains", type=int, nargs="+", default=[1, 2])
ap.add_argument("--seconds", type=float, default=60.0)
ap.add_argument("--headless", type=int, default=1)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": bool(args.headless)})

import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
app.update()

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
cubes = []
for i, dom in enumerate(args.domains):
    path = f"/World/vehicle_{dom}"
    cubes.append(world.scene.add(DynamicCuboid(prim_path=path, name=f"vehicle_{dom}",
                                               position=[0.0, 4.0 * i, 0.5], scale=[2.0, 1.0, 1.0])))
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": f"/World/ros_{dom}", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("context.inputs:useDomainIDEnvVar", False),
                ("context.inputs:domain_id", dom),
                ("tf.inputs:targetPrims", [path]),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("tick.outputs:tick", "tf.inputs:execIn"),
                ("context.outputs:context", "clock.inputs:context"),
                ("context.outputs:context", "tf.inputs:context"),
                ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
                ("simtime.outputs:simulationTime", "tf.inputs:timeStamp"),
            ],
        },
    )

world.reset()
world.play()  # OnPlaybackTick fires only while the timeline plays
for c in cubes:
    c.set_linear_velocity([2.0, 0.0, 0.0])
print("READY", flush=True)
t0 = time.time()
while world.current_time < args.seconds and app.is_running():
    world.step(render=True)
print(f"DONE sim {world.current_time:.1f} s in {time.time() - t0:.1f} s wall", flush=True)
app.close()
