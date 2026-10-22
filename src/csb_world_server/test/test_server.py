"""The world server's protocol side on a fake world (roadmaps 037, 046); no simulator needed.

    python3 -m unittest discover -s src/csb_world_server/test     # protobuf only
"""
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_world_server import lifetime  # noqa: E402
from csb_world_server import protocol as pb  # noqa: E402
from csb_world_server.server import Drive, WorldServer  # noqa: E402
from csb_world_server.world import World  # noqa: E402


class FakeWorld(World):
    """Moves bodies exactly to their poses; the ego touches whatever is within 1 m."""

    simulator_name = "fake"
    capabilities = ("SIGNALS", "TRACKED_MOTION", "CONTACTS")
    max_physics_dt = 1 / 60

    def __init__(self):
        self.bodies = {}
        self.loads = []
        self.steps = []
        self.resets = []

    @property
    def simulator_version(self):
        return "fake-0"

    def load(self, world, ground_paths, signals=(), request=None):
        self.loads.append((world, ground_paths))
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

    def reset(self, handles):
        self.resets.append(sorted(handles))
        super().reset(handles)

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


BRIDGE = pb.ClientId(host="testhost", pid=100, start_time=1)
OTHER = pb.ClientId(host="testhost", pid=200, start_time=2)


def call(server, client=BRIDGE, **request):
    return pb.Reply.FromString(server.handle(
        pb.Request(client=client, **request).SerializeToString()))


def hello(server, client=BRIDGE, probe=False):
    return call(server, client=client,
                hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION, probe=probe))


def spawn(server, name, kind, x, motion="KINEMATIC"):
    return call(server, spawn=pb.Spawn(
        name=name, kind=pb.KIND[kind], box_center=pb.Vec3(x=1.5, z=0.9),
        box_dimensions=pb.Vec3(x=4.5, y=2.1, z=1.8), motion=pb.MOTION[motion],
        pose=pb.make_pose((x, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))))


def step(server, *poses, substeps=3, dt=1 / 60, render=False):
    """An Apply with `poses`, then a Step: one SSv2 frame."""
    r = call(server, apply=pb.Apply(poses=list(poses)))
    assert r.ok, r.error
    return call(server, step=pb.Step(substeps=substeps, dt=dt, render=render))


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.world = FakeWorld()
        self.clock = Clock()
        self.gone = set()
        self.server = WorldServer(self.world, lease_seconds=15, clock=self.clock,
                                  is_gone=lambda c: c.pid in self.gone)

    def test_hello_first_and_only_in_our_version(self):
        r = call(self.server, load_world=pb.LoadWorld())
        self.assertFalse(r.ok)
        self.assertIn("before hello", r.error)
        r = call(self.server, hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION + 1))
        self.assertFalse(r.ok)
        self.assertIn(f"speaks {pb.PROTOCOL_VERSION}", r.error)
        r = hello(self.server)
        self.assertTrue(r.ok)
        h = r.hello
        self.assertEqual((h.simulator_name, h.simulator_version), ("fake", "fake-0"))
        self.assertEqual(h.protocol_version, pb.PROTOCOL_VERSION)
        self.assertEqual({pb.CAPABILITY_NAME[c] for c in h.capabilities},
                         {"SIGNALS", "TRACKED_MOTION", "CONTACTS"})
        self.assertAlmostEqual(h.max_physics_dt, 1 / 60)
        self.assertEqual(h.owner.pid, BRIDGE.pid)
        self.assertTrue(h.ready)
        self.assertEqual(len(h.instance), 32)

    def test_spawn_step_despawn(self):
        hello(self.server)
        r = call(self.server, load_world=pb.LoadWorld())
        self.assertTrue(r.ok)
        self.assertEqual(r.load_world.sim_time_ns, 0)
        r = spawn(self.server, "ego", "EGO", 0.0, motion="EXTERNAL")
        ego = r.spawn.handle
        self.assertEqual(r.spawn.body.model, "fake:box")
        self.assertAlmostEqual(r.spawn.body.box_extent.x, 2.25)
        npc = spawn(self.server, "npc", "VEHICLE", -10.0).spawn.handle
        self.assertEqual((ego, npc), (1, 2))
        self.assertEqual(self.world.bodies[ego]["dims"], (4.5, 2.1, 1.8))
        r = step(self.server, pb.EntityPose(handle=npc, pose=pb.make_pose((-0.5, 0, 0), (1, 0, 0, 0))),
                 pb.EntityPose(handle=ego), render=True)
        self.assertTrue(r.ok, r.error)
        self.assertAlmostEqual(r.step.sim_time, 0.05)
        self.assertEqual(r.step.frame, 1)
        # An EXTERNAL body is listed with no drive: the step does not touch it.
        self.assertEqual(set(self.world.steps[-1][0]), {npc})
        states = {s.handle: pb.pose_tuple(s.pose) for s in r.step.states}
        self.assertEqual(states[npc][0], (-0.5, 0.0, 0.0))
        self.assertEqual(len(r.step.contacts), 1)
        c = r.step.contacts[0]
        self.assertEqual((c.handle, c.other, c.impulse), (ego, npc, 2.5))
        # Drives are used once: the next step has none.
        step(self.server)
        self.assertEqual(self.world.steps[-1][0], {})
        self.assertTrue(call(self.server, despawn=pb.Despawn(handle=npc)).ok)
        r = call(self.server, despawn=pb.Despawn(handle=npc))
        self.assertFalse(r.ok)
        self.assertIn("no entity with handle 2", r.error)

    def test_a_tracked_body_takes_twists_teleports_and_the_fallback(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        npc = spawn(self.server, "npc", "VEHICLE", 0.0, motion="TRACKED").spawn.handle
        self.assertEqual(self.world.bodies[npc]["motion"], "TRACKED")
        twist = pb.make_twist((8.0, 0.0, 0.0), (0.0, 0.0, 0.1))
        r = step(self.server, pb.EntityPose(handle=npc, twist=twist))
        self.assertTrue(r.ok, r.error)
        drive = self.world.steps[-1][0][npc]
        self.assertEqual(drive, Drive(pose=None, twist=((8.0, 0.0, 0.0), (0.0, 0.0, 0.1))))
        state = r.step.states[0]
        self.assertAlmostEqual(state.pose.position.x, 0.4)
        self.assertEqual(pb.twist_tuple(state.twist), ((8.0, 0.0, 0.0), (0.0, 0.0, 0.1)))
        r = step(self.server, pb.EntityPose(
            handle=npc, teleport=True, twist=twist,
            pose=pb.make_pose((10.0, 1.0, 0.0), (1.0, 0.0, 0.0, 0.0))))
        self.assertAlmostEqual(r.step.states[0].pose.position.x, 10.4)
        r = step(self.server, pb.EntityPose(
            handle=npc, kinematic=True, pose=pb.make_pose((20.0, 1.0, 0.0), (1.0, 0.0, 0.0, 0.0))))
        self.assertEqual(self.world.bodies[npc]["motion"], "KINEMATIC")
        self.assertAlmostEqual(r.step.states[0].pose.position.x, 20.0)
        self.assertFalse(r.step.states[0].HasField("twist"))

    def test_what_a_server_does_not_offer_is_refused(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        r = spawn(self.server, "ap", "VEHICLE", 0.0, motion="AUTOPILOT")
        self.assertFalse(r.ok)
        self.assertIn("AUTOPILOT bodies are not offered", r.error)
        r = spawn(self.server, "bg_av_1", "VEHICLE", 0.0, motion="EXTERNAL")
        self.assertFalse(r.ok)
        self.assertIn("other than the ego", r.error)
        for request in ({"set_goal": pb.SetGoal(handle=1)},
                        {"set_weather": pb.SetWeather()}, {"get_weather": pb.GetWeather()}):
            r = call(self.server, **request)
            self.assertFalse(r.ok)
            self.assertIn("not offered", r.error)
        self.world.capabilities = ()
        bare = WorldServer(self.world, clock=self.clock)
        hello(bare)
        r = call(bare, set_signals=pb.SetSignals())
        self.assertFalse(r.ok)
        self.assertIn("signals are not offered", r.error)
        r = spawn(bare, "npc", "VEHICLE", 0.0, motion="TRACKED")
        self.assertFalse(r.ok)
        # No SIGNALS: the map's signals are not handed to the world.
        call(bare, load_world=pb.LoadWorld(signals=[pb.Signal(way_id=1, opendrive_id="1")]))
        self.assertEqual(self.world.signals, [])

    def test_signals_come_with_the_world_and_take_colours(self):
        hello(self.server)
        r = call(self.server, load_world=pb.LoadWorld(world="/maps/x.usd", signals=[
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
        r = call(self.server, apply=pb.Apply(poses=[pb.EntityPose(handle=99)]))
        self.assertFalse(r.ok)
        self.assertIn("unknown handle", r.error)
        r = call(self.server, step=pb.Step(substeps=0, dt=0.05))
        self.assertFalse(r.ok)
        r = call(self.server, step=pb.Step(poses=[pb.EntityPose(handle=1)], substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("by Apply", r.error)
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
        r = call(self.server, load_world=pb.LoadWorld(world="/maps/x.usd",
                                                      ground_paths=["/World/env/road"]))
        self.assertEqual(r.load_world.sim_time_ns, 50_000_000)
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
            r = call(self.server, step=pb.Step(substeps=2, dt=0.025))
        self.assertGreaterEqual(r.step.tick_seconds, 0.0)
        self.assertEqual([s[4] for s in self.world.steps], [0.0, 0.05, 0.1])
        self.assertEqual(len(self.server.step_ms), 3)
        # Despawning the last entity ends the scenario: the step times are summarized.
        call(self.server, despawn=pb.Despawn(handle=npc))
        self.assertEqual(self.server.step_ms, [])

    def test_shutdown_stops_the_loop(self):
        hello(self.server)
        self.assertTrue(call(self.server, shutdown=pb.Shutdown()).ok)
        self.assertTrue(self.server.stopped)


class LeaseTest(unittest.TestCase):
    """One bridge drives a server at a time (roadmap 046)."""

    def setUp(self):
        self.world = FakeWorld()
        self.clock = Clock()
        self.gone = set()
        self.server = WorldServer(self.world, lease_seconds=15, clock=self.clock,
                                  is_gone=lambda c: c.pid in self.gone)
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        self.npc = spawn(self.server, "npc", "VEHICLE", 0.0).spawn.handle

    def test_a_second_bridge_is_refused_while_the_owner_lives(self):
        r = hello(self.server, client=OTHER)
        self.assertFalse(r.ok)
        self.assertIn("owned by testhost:100", r.error)
        r = call(self.server, client=OTHER, step=pb.Step(substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("refused: this world server is owned by testhost:100", r.error)
        # The owner is undisturbed.
        self.assertTrue(call(self.server, step=pb.Step(substeps=1, dt=0.05)).ok)
        self.assertIn(self.npc, self.world.bodies)

    def test_a_probe_changes_nothing(self):
        r = hello(self.server, client=OTHER, probe=True)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.hello.owner.pid, BRIDGE.pid)
        self.clock.now += 20
        # The probe did not refresh the owner's lease, nor take one.
        r = hello(self.server, client=OTHER, probe=True)
        self.assertEqual(r.hello.owner.pid, BRIDGE.pid)
        self.assertEqual(self.world.resets, [])

    def test_the_owners_hello_keeps_its_lease_and_its_world(self):
        for _ in range(4):
            self.clock.now += 10
            self.assertTrue(hello(self.server).ok)
        self.assertFalse(hello(self.server, client=OTHER).ok)
        self.assertIn(self.npc, self.world.bodies)
        self.assertEqual(self.world.resets, [])

    def test_a_dead_owner_is_replaced_at_once_and_its_entities_go(self):
        self.gone.add(BRIDGE.pid)
        r = hello(self.server, client=OTHER)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.hello.owner.pid, OTHER.pid)
        self.assertEqual(self.world.resets, [[self.npc]])
        self.assertEqual(self.world.bodies, {})
        # The old owner, should it wake up, is refused.
        r = call(self.server, step=pb.Step(substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("owned by testhost:200", r.error)

    def test_a_silent_owner_lapses(self):
        self.clock.now += 14
        self.assertFalse(hello(self.server, client=OTHER).ok)
        self.clock.now += 2
        r = hello(self.server, client=OTHER)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.hello.owner.pid, OTHER.pid)

    def test_release_leaves_no_owner_and_no_entities(self):
        self.assertTrue(call(self.server, release=pb.Release()).ok)
        self.assertEqual(self.world.bodies, {})
        r = hello(self.server, client=OTHER, probe=True)
        self.assertFalse(r.hello.HasField("owner"))
        self.assertTrue(hello(self.server, client=OTHER).ok)


class LifetimeTest(unittest.TestCase):
    def test_this_process_is_not_gone_and_a_reused_pid_is(self):
        me = lifetime.client_id(pb)
        self.assertFalse(lifetime.is_gone(me))
        reused = pb.ClientId(host=me.host, pid=me.pid, start_time=me.start_time + 1)
        self.assertTrue(lifetime.is_gone(reused))
        self.assertTrue(lifetime.is_gone(pb.ClientId(host=me.host, pid=2**22 + 7, start_time=1)))
        # Another host is never provably gone.
        self.assertFalse(lifetime.is_gone(pb.ClientId(host="elsewhere", pid=1, start_time=1)))


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
                             "csb_world.desc is stale: run "
                             "src/csb_world_server/scripts/gen_world_proto.sh")


if __name__ == "__main__":
    unittest.main()
