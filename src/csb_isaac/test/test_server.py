"""The world server's protocol side on a fake world (roadmap 037); no Isaac needed.

    python -m unittest discover -s src/csb_isaac/test     # with protobuf (Isaac's venv has it)
"""
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac import protocol as pb  # noqa: E402
from csb_isaac.server import Drive, WorldServer  # noqa: E402


class FakeWorld:
    """Moves bodies exactly to their poses; the ego touches whatever is within 1 m."""

    isaac_version = "fake-0"

    def __init__(self):
        self.bodies = {}
        self.loads = []
        self.steps = []

    def load(self, usd, ground_paths, signals=()):
        self.loads.append((usd, ground_paths))
        self.signals = list(signals)
        self.colours = {}
        self.bodies.clear()

    def set_signals(self, colours):
        self.colours.update(colours)

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="", motion="KINEMATIC"):
        self.bodies[handle] = {"name": name, "kind": kind, "pose": pose, "dims": dims,
                               "model": model, "motion": motion, "twist": None}

    def despawn(self, handle):
        del self.bodies[handle]

    def step(self, drives, substeps, dt, render, sim_time):
        self.steps.append((dict(drives), substeps, dt, render, sim_time))
        for h, d in drives.items():
            b = self.bodies[h]
            if d.kinematic:
                b["motion"] = "KINEMATIC"
            if b["motion"] == "TRACKED":
                if d.teleport:
                    b["pose"] = d.pose
                if d.twist is not None:
                    (x, y, z), q = b["pose"]
                    v = d.twist[0]
                    t = substeps * dt
                    b["pose"] = ((x + v[0] * t, y + v[1] * t, z + v[2] * t), q)
                    b["twist"] = d.twist
            elif d.pose is not None:
                b["pose"] = d.pose
        states = [(h, b["pose"], b["twist"] if b["motion"] == "TRACKED" else None)
                  for h, b in self.bodies.items()]
        contacts = []
        for h, b in self.bodies.items():
            if b["kind"] != "EGO":
                continue
            for o, other in self.bodies.items():
                if o != h and abs(other["pose"][0][0] - b["pose"][0][0]) < 1.0:
                    contacts.append((h, o, f"/World/entities/e{o}", 2.5))
        return states, contacts


def call(server, **request):
    reply = pb.Reply.FromString(server.handle(pb.Request(**request).SerializeToString()))
    return reply


def hello(server):
    return call(server, hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION))


def spawn(server, name, kind, x):
    return call(server, spawn=pb.Spawn(
        name=name, kind=pb.KIND[kind], box_center=pb.Vec3(x=1.5, z=0.9),
        box_dimensions=pb.Vec3(x=4.5, y=2.1, z=1.8),
        pose=pb.make_pose((x, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))))


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.world = FakeWorld()
        self.server = WorldServer(self.world)

    def test_hello_first_and_only_in_our_version(self):
        r = call(self.server, load_world=pb.LoadWorld())
        self.assertFalse(r.ok)
        self.assertIn("before hello", r.error)
        r = call(self.server, hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION + 1))
        self.assertFalse(r.ok)
        self.assertIn(f"speaks {pb.PROTOCOL_VERSION}", r.error)
        r = hello(self.server)
        self.assertTrue(r.ok)
        self.assertEqual(r.hello.isaac_version, "fake-0")
        self.assertEqual(r.hello.protocol_version, pb.PROTOCOL_VERSION)

    def test_spawn_step_despawn(self):
        hello(self.server)
        self.assertTrue(call(self.server, load_world=pb.LoadWorld()).ok)
        ego = spawn(self.server, "ego", "EGO", 0.0).spawn.handle
        npc = spawn(self.server, "npc", "VEHICLE", -10.0).spawn.handle
        self.assertEqual((ego, npc), (1, 2))
        self.assertEqual(self.world.bodies[ego]["dims"], (4.5, 2.1, 1.8))
        r = call(self.server, step=pb.Step(
            poses=[pb.EntityPose(handle=npc, pose=pb.make_pose((-0.5, 0, 0), (1, 0, 0, 0)))],
            substeps=3, dt=1 / 60, render=True))
        self.assertTrue(r.ok, r.error)
        self.assertAlmostEqual(r.step.sim_time, 0.05)
        self.assertEqual(r.step.frame, 1)
        states = {s.handle: pb.pose_tuple(s.pose) for s in r.step.states}
        self.assertEqual(states[npc][0], (-0.5, 0.0, 0.0))
        self.assertEqual(len(r.step.contacts), 1)
        c = r.step.contacts[0]
        self.assertEqual((c.handle, c.other, c.impulse), (ego, npc, 2.5))
        self.assertTrue(call(self.server, despawn=pb.Despawn(handle=npc)).ok)
        r = call(self.server, despawn=pb.Despawn(handle=npc))
        self.assertFalse(r.ok)
        self.assertIn("no entity with handle 2", r.error)

    def test_a_tracked_body_takes_twists_teleports_and_the_fallback(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        r = call(self.server, spawn=pb.Spawn(
            name="npc", kind=pb.KIND["VEHICLE"], box_center=pb.Vec3(x=1.5, z=0.9),
            box_dimensions=pb.Vec3(x=4.5, y=2.1, z=1.8), motion=pb.MOTION["TRACKED"],
            pose=pb.make_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))))
        npc = r.spawn.handle
        self.assertEqual(self.world.bodies[npc]["motion"], "TRACKED")
        twist = pb.make_twist((8.0, 0.0, 0.0), (0.0, 0.0, 0.1))
        r = call(self.server, step=pb.Step(
            poses=[pb.EntityPose(handle=npc, twist=twist)], substeps=3, dt=1 / 60))
        self.assertTrue(r.ok, r.error)
        drive = self.world.steps[-1][0][npc]
        self.assertEqual(drive, Drive(pose=None, twist=((8.0, 0.0, 0.0), (0.0, 0.0, 0.1))))
        state = r.step.states[0]
        self.assertAlmostEqual(state.pose.position.x, 0.4)
        self.assertEqual(pb.twist_tuple(state.twist), ((8.0, 0.0, 0.0), (0.0, 0.0, 0.1)))
        r = call(self.server, step=pb.Step(poses=[pb.EntityPose(
            handle=npc, teleport=True, twist=twist,
            pose=pb.make_pose((10.0, 1.0, 0.0), (1.0, 0.0, 0.0, 0.0)))], substeps=3, dt=1 / 60))
        self.assertAlmostEqual(r.step.states[0].pose.position.x, 10.4)
        r = call(self.server, step=pb.Step(poses=[pb.EntityPose(
            handle=npc, kinematic=True,
            pose=pb.make_pose((20.0, 1.0, 0.0), (1.0, 0.0, 0.0, 0.0)))], substeps=3, dt=1 / 60))
        self.assertEqual(self.world.bodies[npc]["motion"], "KINEMATIC")
        self.assertAlmostEqual(r.step.states[0].pose.position.x, 20.0)
        self.assertFalse(r.step.states[0].HasField("twist"))

    def test_signals_come_with_the_world_and_take_colours(self):
        hello(self.server)
        r = call(self.server, load_world=pb.LoadWorld(usd="/maps/x.usd", signals=[
            pb.Signal(way_id=43763, regulatory_element_ids=[43856], opendrive_id="377",
                      position=pb.Vec3(x=102.72, y=-52.85, z=0.1)),
            pb.Signal(way_id=43766, opendrive_id="376")]))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(self.world.signals[0], {
            "way_id": 43763, "regulatory_element_ids": [43856], "opendrive_id": "377",
            "position": (102.72, -52.85, 0.1)})
        self.assertIsNone(self.world.signals[1]["position"])
        r = call(self.server, set_signals=pb.SetSignals(states=[
            pb.SignalState(opendrive_id="377", colour=pb.COLOUR["RED"]),
            pb.SignalState(opendrive_id="376", colour=pb.COLOUR["OFF"])]))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(self.world.colours, {"377": "RED", "376": "OFF"})

    def test_bad_requests_are_refused_and_the_server_goes_on(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        spawn(self.server, "npc", "VEHICLE", 0.0)
        r = spawn(self.server, "npc", "VEHICLE", 5.0)
        self.assertFalse(r.ok)
        self.assertIn("exists", r.error)
        r = call(self.server, spawn=pb.Spawn(name="flat", kind=pb.KIND["MISC_OBJECT"]))
        self.assertFalse(r.ok)
        self.assertIn("not a box", r.error)
        r = call(self.server, step=pb.Step(
            poses=[pb.EntityPose(handle=99)], substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("unknown handle", r.error)
        r = call(self.server, step=pb.Step(substeps=0, dt=0.05))
        self.assertFalse(r.ok)
        self.assertFalse(pb.Reply.FromString(self.server.handle(b"\xff\xff")).ok)
        self.assertTrue(call(self.server, step=pb.Step(substeps=1, dt=0.05)).ok)

    def test_a_world_failure_is_an_error_reply(self):
        hello(self.server)

        def broken(*_args):
            raise RuntimeError("PhysX is gone")

        self.world.step = broken
        r = call(self.server, step=pb.Step(substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("PhysX is gone", r.error)

    def test_a_load_forgets_every_entity_and_the_clock_goes_on(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        spawn(self.server, "npc", "VEHICLE", 0.0)
        call(self.server, step=pb.Step(substeps=2, dt=0.025))
        call(self.server, load_world=pb.LoadWorld(usd="/maps/x.usd", ground_paths=["/World/env/road"]))
        self.assertEqual(self.world.loads[-1], ("/maps/x.usd", ["/World/env/road"]))
        r = call(self.server, step=pb.Step(substeps=2, dt=0.025))
        self.assertEqual(len(r.step.states), 0)
        self.assertAlmostEqual(r.step.sim_time, 0.1)
        # Handles stay unique over the server's lifetime.
        self.assertEqual(spawn(self.server, "npc", "VEHICLE", 0.0).spawn.handle, 2)

    def test_steps_get_the_clock_at_their_start_and_are_timed(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        npc = spawn(self.server, "npc", "VEHICLE", 0.0).spawn.handle
        for _ in range(3):
            call(self.server, step=pb.Step(substeps=2, dt=0.025))
        self.assertEqual([s[4] for s in self.world.steps], [0.0, 0.05, 0.1])
        self.assertEqual(len(self.server.step_ms), 3)
        # Despawning the last entity ends the scenario: the step times are summarized.
        call(self.server, despawn=pb.Despawn(handle=npc))
        self.assertEqual(self.server.step_ms, [])

    def test_shutdown_stops_the_loop(self):
        hello(self.server)
        self.assertTrue(call(self.server, shutdown=pb.Shutdown()).ok)
        self.assertTrue(self.server.stopped)


class DescriptorTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("protoc"), "no protoc")
    def test_the_descriptor_set_is_the_protos(self):
        root = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "csb_world.desc"
            subprocess.run(["protoc", f"--proto_path={root / 'proto'}",
                            f"--descriptor_set_out={out}", str(root / "proto/csb_world.proto")],
                           check=True)
            self.assertEqual(out.read_bytes(), pb.DESCRIPTOR_SET.read_bytes(),
                             "csb_world.desc is stale: run src/csb_isaac/scripts/gen_world_proto.sh")


if __name__ == "__main__":
    unittest.main()
