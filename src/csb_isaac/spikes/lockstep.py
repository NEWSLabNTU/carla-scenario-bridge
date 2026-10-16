"""Phase 036 spike: lockstep stepping driven over ZMQ, and its determinism.

Server (run under Isaac's Python): a stage with N kinematic boxes (puppeteered NPCs) and one
dynamic box launched at 6 m/s and left to physics (stands in for a physics-driven ego). Each request
carries the NPC poses for the next frame and the number of physics substeps; the server sets
the poses, steps `substeps` times (rendering on the last), and replies with the sim time and
the dynamic box's pose. Between requests nothing advances.

Client (plain Python): drives `frames` requests with a scripted NPC motion, records the
replies, and prints round-trip statistics. Run it twice against fresh servers and compare the
two recordings for determinism.

    python lockstep.py server --port 5590 --npcs 20
    python lockstep.py client --port 5590 --frames 600 --out run1.json
"""
import argparse
import json
import time

ap = argparse.ArgumentParser()
ap.add_argument("role", choices=["server", "client"])
ap.add_argument("--port", type=int, default=5590)
ap.add_argument("--npcs", type=int, default=20)
ap.add_argument("--frames", type=int, default=600)
ap.add_argument("--substeps", type=int, default=6)  # 60 Hz physics, 10 Hz frames
ap.add_argument("--render", type=int, default=1)
ap.add_argument("--out", default="")
args = ap.parse_args()


def npc_poses(frame, n):
    t = frame * 0.1
    return [[5.0 + 2.0 * t, -10.0 + 1.5 * i, 0.75, 0.05 * t * (i % 3)] for i in range(n)]  # x, y, z, yaw


if args.role == "client":
    import numpy as np
    import zmq

    sock = zmq.Context().socket(zmq.REQ)
    sock.connect(f"tcp://127.0.0.1:{args.port}")
    replies, rtt = [], []
    for f in range(args.frames):
        req = {"frame": f, "substeps": args.substeps, "npcs": npc_poses(f, args.npcs)}
        t0 = time.perf_counter()
        sock.send_string(json.dumps(req))
        rep = json.loads(sock.recv_string())
        rtt.append(1e3 * (time.perf_counter() - t0))
        replies.append(rep)
    sock.send_string(json.dumps({"quit": True}))
    sock.recv_string()
    r = np.array(rtt[10:])
    print(f"CLIENT frames {args.frames} rtt median {np.median(r):.2f} ms p95 {np.percentile(r, 95):.2f} ms "
          f"max {r.max():.2f} ms; last sim time {replies[-1]['sim_time']:.3f} s", flush=True)
    if args.out:
        json.dump(replies, open(args.out, "w"))
    raise SystemExit(0)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import zmq  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # noqa: E402
from isaacsim.core.prims import XFormPrim  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
npcs = []
for i in range(args.npcs):
    c = world.scene.add(FixedCuboid(prim_path=f"/World/npc_{i}", name=f"npc_{i}", position=[5.0, -10.0 + 1.5 * i, 0.75],
                                    scale=[4.5, 1.8, 1.5]))
    UsdPhysics.RigidBodyAPI.Apply(c.prim).CreateKinematicEnabledAttr(True)  # puppeteered, still collides
    npcs.append(c)
npc_view = XFormPrim("/World/npc_.*")
ego = world.scene.add(DynamicCuboid(prim_path="/World/ego", name="ego", position=[0.0, 20.0, 0.75],
                                    scale=[4.5, 1.8, 1.5], mass=1500.0))
world.reset()
world.play()
world.step(render=False)  # a velocity set before the first physics step is dropped
ego.set_linear_velocity([6.0, 0.0, 0.0])
ego.set_angular_velocity([0.0, 0.0, 1.5])  # then free physics: slides, rubs, rocks on the ground

sock = zmq.Context().socket(zmq.REP)
sock.bind(f"tcp://127.0.0.1:{args.port}")
print("READY", flush=True)
step_ms = []
while True:
    req = json.loads(sock.recv_string())
    if req.get("quit"):
        sock.send_string("{}")
        break
    t0 = time.perf_counter()
    p = np.array(req["npcs"], dtype=np.float64)
    pos = p[:, :3]
    yaw = p[:, 3]
    quat = np.stack([np.cos(yaw / 2), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2)], 1)  # wxyz
    npc_view.set_world_poses(positions=pos, orientations=quat)
    for s in range(req["substeps"]):
        world.step(render=bool(args.render) and s == req["substeps"] - 1)
    ep, eq = ego.get_world_pose()
    step_ms.append(1e3 * (time.perf_counter() - t0))
    sock.send_string(json.dumps({"frame": req["frame"], "sim_time": world.current_time,
                                 "ego": [float(x) for x in ep] + [float(x) for x in eq]}))
s = np.array(step_ms[10:])
print(f"SERVER frames {len(step_ms)} apply+step median {np.median(s):.2f} ms p95 {np.percentile(s, 95):.2f} ms", flush=True)
app.close()
