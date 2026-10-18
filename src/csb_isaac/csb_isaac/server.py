"""The world server's protocol side (roadmap 037): requests in, a `World` driven, replies out.

Pure Python, so it is tested without Isaac (`test/test_server.py`, on a fake world). The
Isaac world is `world.py`; `world_server.py` puts the two together on a socket.

A `World` provides:

- ``isaac_version``: str
- ``load(usd, ground_paths)``: replace the environment, remove every entity
- ``spawn(handle, name, kind, subtype, center, dims, pose)``: one kinematic body
- ``despawn(handle)``
- ``step(poses, substeps, dt, render, sim_time)`` -> ``(states, contacts)``: apply
  ``{handle: pose}`` (``sim_time`` is the clock at the step's start), run the physics steps,
  and return ``[(handle, pose)]`` for every entity and
  ``[(handle, other, other_path, impulse)]`` for the contacts (``other`` 0: environment)

Poses are ``((x, y, z), (qw, qx, qy, qz))`` in the world frame.
"""
import logging
import time

from . import protocol as pb

log = logging.getLogger("csb_isaac")


class RequestError(Exception):
    """A request the server refuses: the reply's error, the server goes on."""


class WorldServer:
    def __init__(self, world):
        self.world = world
        self.next_handle = 1
        self.entities = {}  # handle -> name
        self.sim_time = 0.0
        self.frame = 0
        self.greeted = False
        self.stopped = False
        self.step_ms = []  # wall time of each step since the last summary (038)

    def handle(self, data: bytes) -> bytes:
        """One request's bytes to its reply's bytes. Never raises."""
        try:
            request = pb.Request.FromString(data)
        except Exception as e:  # noqa: BLE001 - a malformed request is the peer's error
            return pb.Reply(ok=False, error=f"undecodable request ({len(data)} bytes): {e}") \
                .SerializeToString()
        which = request.WhichOneof("request")
        try:
            if which is None:
                raise RequestError("empty request")
            if which != "hello" and not self.greeted:
                raise RequestError(f"{which} before hello: say hello first")
            reply = getattr(self, f"_{which}")(getattr(request, which))
        except RequestError as e:
            reply = pb.Reply(ok=False, error=str(e))
        except Exception as e:  # noqa: BLE001 - the world failed; csb fails the frame
            log.exception("%s failed", which)
            reply = pb.Reply(ok=False, error=f"{which} failed in the world server: {e}")
        return reply.SerializeToString()

    def _hello(self, req):
        if req.protocol_version != pb.PROTOCOL_VERSION:
            raise RequestError(
                f"protocol version {req.protocol_version} refused: this world server speaks "
                f"{pb.PROTOCOL_VERSION}")
        self.greeted = True
        return pb.Reply(ok=True, hello=pb.HelloReply(protocol_version=pb.PROTOCOL_VERSION,
                                                     isaac_version=self.world.isaac_version))

    def _load_world(self, req):
        self.world.load(req.usd, list(req.ground_paths))
        self.entities.clear()
        log.info("world %s loaded", req.usd or "flat ground plane")
        return pb.Reply(ok=True)

    def _spawn(self, req):
        if req.name in self.entities.values():
            raise RequestError(f"an entity named {req.name!r} exists")
        d = req.box_dimensions
        if not (d.x > 0 and d.y > 0 and d.z > 0):
            raise RequestError(f"{req.name!r}: box dimensions {d.x}, {d.y}, {d.z} are not a box")
        handle = self.next_handle
        c = req.box_center
        self.world.spawn(handle, req.name, pb.KIND_NAME.get(req.kind, "KIND_UNSPECIFIED"),
                         req.subtype, (c.x, c.y, c.z), (d.x, d.y, d.z), pb.pose_tuple(req.pose))
        self.next_handle += 1
        self.entities[handle] = req.name
        return pb.Reply(ok=True, spawn=pb.SpawnReply(handle=handle))

    def step_summary(self):
        """Wall time per step since the last summary, logged and returned (p50, p95, max ms)."""
        if not self.step_ms:
            return None
        t = sorted(self.step_ms)
        p50, p95, top = t[len(t) // 2], t[min(len(t) - 1, int(0.95 * len(t)))], t[-1]
        log.info("%d step(s): wall time per step p50 %.1f ms, p95 %.1f ms, max %.0f ms",
                 len(t), p50, p95, top)
        self.step_ms.clear()
        return p50, p95, top

    def _despawn(self, req):
        if req.handle not in self.entities:
            raise RequestError(f"no entity with handle {req.handle}")
        if len(self.entities) == 1:  # the last one: SSv2 ends its scenario this way
            self.step_summary()
        self.world.despawn(req.handle)
        del self.entities[req.handle]
        return pb.Reply(ok=True)

    def _step(self, req):
        if req.substeps < 1 or not req.dt > 0:
            raise RequestError(f"step of {req.substeps} x {req.dt} s")
        unknown = [p.handle for p in req.poses if p.handle not in self.entities]
        if unknown:
            raise RequestError(f"poses for unknown handle(s) {unknown}")
        poses = {p.handle: pb.pose_tuple(p.pose) for p in req.poses}
        started = time.perf_counter()
        states, contacts = self.world.step(poses, req.substeps, req.dt, req.render, self.sim_time)
        self.step_ms.append(1e3 * (time.perf_counter() - started))
        self.sim_time += req.substeps * req.dt
        self.frame += 1
        reply = pb.StepReply(sim_time=self.sim_time, frame=self.frame)
        for handle, pose in states:
            reply.states.add(handle=handle, pose=pb.make_pose(*pose))
        for handle, other, path, impulse in contacts:
            reply.contacts.add(handle=handle, other=other, other_path=path, impulse=impulse)
        return pb.Reply(ok=True, step=reply)

    def _shutdown(self, _req):
        self.stopped = True
        return pb.Reply(ok=True)


def serve(socket, server, should_stop=lambda: False, poll_ms=100):
    """Answer requests on a bound REP `socket` until `Shutdown` or `should_stop()`."""
    import zmq

    while not server.stopped and not should_stop():
        if not socket.poll(poll_ms, zmq.POLLIN):
            continue
        socket.send(server.handle(socket.recv()))
