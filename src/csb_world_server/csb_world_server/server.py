"""The world server's protocol side (roadmaps 037, 046): requests in, a `World` driven,
replies out.

Pure Python, so it is tested without a simulator (`test/test_server.py`, on a fake world).
`run()` puts a `World` on a socket: what a simulator's entry point calls (the Isaac one is
`csb_isaac.world_server`).

The shell owns the protocol so a `World` does not have to: the version check, the owner lease
(one bridge at a time; `proto/csb_world.proto`, "Ownership"), handles, request validation
against the declared capabilities, the drives an `Apply` hands the next `Step`, the clock.
"""
import logging
import os
import signal
import time
import uuid

from . import lifetime
from . import protocol as pb
from .world import Drive, World  # noqa: F401 - re-exported for World implementations

log = logging.getLogger("csb_world_server")

# How long an owner may say nothing before another client may take the server over, s. The
# bridge probes every few seconds between scenarios (its idle check), so a live one never
# lapses.
LEASE_SECONDS = 15.0

# How long a refused server (its port held) waits before it exits 1.
REFUSED_PAUSE = 30


def drive_of(entity_pose):
    """The `Drive` an `EntityPose` asks for."""
    return Drive(
        pose=pb.pose_tuple(entity_pose.pose) if entity_pose.HasField("pose") else None,
        twist=pb.twist_tuple(entity_pose.twist) if entity_pose.HasField("twist") else None,
        teleport=entity_pose.teleport, kinematic=entity_pose.kinematic)


def signal_dict(signal_):
    """A `Signal` of `LoadWorld` as the world takes it."""
    return {"way_id": signal_.way_id,
            "regulatory_element_ids": list(signal_.regulatory_element_ids),
            "opendrive_id": signal_.opendrive_id,
            "position": ((signal_.position.x, signal_.position.y, signal_.position.z)
                         if signal_.HasField("position") else None)}


def describe(client):
    return f"{client.host}:{client.pid}" if client.pid else "(unnamed client)"


def same_client(a, b):
    return (a.host, a.pid, a.start_time) == (b.host, b.pid, b.start_time)


class RequestError(Exception):
    """A request the server refuses: the reply's error, the server goes on."""


class WorldServer:
    def __init__(self, world, lease_seconds=LEASE_SECONDS, clock=time.monotonic,
                 is_gone=lifetime.is_gone):
        self.world = world
        self.capabilities = {pb.CAPABILITY[c] for c in world.capabilities}
        self.lease_seconds = lease_seconds
        self.clock = clock
        self.is_gone = is_gone
        self.instance = uuid.uuid4().hex
        self.owner = None  # ClientId
        self.owner_seen = 0.0
        self.next_handle = 1
        self.entities = {}  # handle -> name
        self.pending = {}  # handle -> Drive, from the last Apply
        self.sim_time = 0.0
        self.frame = 0
        self.stopped = False
        self.step_ms = []  # wall time of each step since the last summary (038)

    def offers(self, name):
        return pb.CAPABILITY[name] in self.capabilities

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
            if which == "hello":
                reply = self._hello(request.hello, request.client)
            else:
                self._check_owner(which, request.client)
                reply = getattr(self, f"_{which}")(getattr(request, which))
        except RequestError as e:
            reply = pb.Reply(ok=False, error=str(e))
        except Exception as e:  # noqa: BLE001 - the world failed; the bridge fails the frame
            log.exception("%s failed", which)
            reply = pb.Reply(ok=False, error=f"{which} failed in the world server: {e}")
        return reply.SerializeToString()

    # --- Ownership -------------------------------------------------------------------------

    def _check_owner(self, which, client):
        if self.owner is None:
            raise RequestError(f"{which} before hello: say hello first")
        if not same_client(client, self.owner):
            raise RequestError(f"{which} from {describe(client)} refused: this world server is "
                               f"owned by {describe(self.owner)}")
        self.owner_seen = self.clock()

    def _take(self, client, why):
        if self.owner is not None:
            log.info("owner %s replaced by %s (%s)", describe(self.owner), describe(client), why)
        else:
            log.info("owner: %s", describe(client))
        self.owner = pb.ClientId()
        self.owner.CopyFrom(client)
        self.owner_seen = self.clock()
        self._reset()

    def _reset(self):
        """Every entity of the last owner goes; the world stays."""
        if self.entities:
            log.info("removing the %d entit%s of the last owner", len(self.entities),
                     "y" if len(self.entities) == 1 else "ies")
            self.world.reset(list(self.entities))
        self.entities.clear()
        self.pending.clear()

    def _hello(self, req, client):
        if req.protocol_version != pb.PROTOCOL_VERSION:
            raise RequestError(
                f"protocol version {req.protocol_version} refused: this world server speaks "
                f"{pb.PROTOCOL_VERSION}")
        if not req.probe:
            if self.owner is None:
                self._take(client, "first hello")
            elif same_client(client, self.owner):
                self.owner_seen = self.clock()
            elif self.is_gone(self.owner):
                self._take(client, "the owner's process is gone")
            elif self.clock() - self.owner_seen > self.lease_seconds:
                self._take(client, f"the owner was silent for more than {self.lease_seconds:g} s")
            else:
                raise RequestError(
                    f"owned by {describe(self.owner)} (seen {self.clock() - self.owner_seen:.0f} s "
                    f"ago): one bridge drives a world server at a time")
        ready, why = self.world.ready()
        reply = pb.HelloReply(
            protocol_version=pb.PROTOCOL_VERSION,
            simulator_version=self.world.simulator_version,
            simulator_name=self.world.simulator_name,
            capabilities=sorted(self.capabilities),
            instance=self.instance, max_physics_dt=self.world.max_physics_dt,
            ready=ready, not_ready_reason=why)
        if self.owner is not None:
            reply.owner.CopyFrom(self.owner)
        return pb.Reply(ok=True, hello=reply)

    def _release(self, _req):
        log.info("released by %s", describe(self.owner))
        self.step_summary()
        self._reset()
        self.owner = None
        return pb.Reply(ok=True)

    # --- The world ---------------------------------------------------------------------------

    def _load_world(self, req):
        ready, why = self.world.ready()
        if not ready:
            raise RequestError(f"not ready to load a world: {why}")
        signals = [signal_dict(s) for s in req.signals] if self.offers("SIGNALS") else []
        self.world.load(req.world, list(req.ground_paths), signals, request=req)
        self.entities.clear()
        self.pending.clear()
        log.info("world %s loaded", req.world or "(default)")
        return pb.Reply(ok=True, load_world=pb.LoadWorldReply(
            sim_time_ns=round(self.sim_time * 1e9)))

    def _spawn(self, req):
        if req.name in self.entities.values():
            raise RequestError(f"an entity named {req.name!r} exists")
        d = req.box_dimensions
        if not (d.x > 0 and d.y > 0 and d.z > 0):
            raise RequestError(f"{req.name!r}: box dimensions {d.x}, {d.y}, {d.z} are not a box")
        kind = pb.KIND_NAME.get(req.kind, "KIND_UNSPECIFIED")
        motion = pb.MOTION_NAME.get(req.motion, "KINEMATIC")
        needs = {"TRACKED": ("TRACKED_MOTION", "TRACKS_TARGETS"), "AUTOPILOT": ("AUTOPILOT",)}
        if motion in needs and not any(self.offers(c) for c in needs[motion]):
            raise RequestError(f"{req.name!r}: {motion} bodies are not offered by this server")
        if motion == "EXTERNAL" and kind != "EGO" and not self.offers("AGENT_VEHICLES"):
            raise RequestError(f"{req.name!r}: externally driven vehicles other than the ego "
                               "are not offered by this server")
        handle = self.next_handle
        c = req.box_center
        body = self.world.spawn(handle, req.name, kind, req.subtype, (c.x, c.y, c.z),
                                (d.x, d.y, d.z), pb.pose_tuple(req.pose), model=req.model,
                                motion=motion)
        self.next_handle += 1
        self.entities[handle] = req.name
        body = body or {}

        def vec(v):
            return pb.Vec3(x=v[0], y=v[1], z=v[2])

        reply = pb.SpawnReply(handle=handle, body=pb.Body(
            model=body.get("model", f"{self.world.simulator_name}:box"),
            box_location=vec(body.get("box_location", (c.x, c.y, c.z))),
            box_extent=vec(body.get("box_extent", (d.x / 2, d.y / 2, d.z / 2))), has_box=True,
            origin_offset=vec(body.get("origin_offset", (0.0, 0.0, 0.0))),
            declared_center=vec((c.x, c.y, c.z)), declared_dimensions=vec((d.x, d.y, d.z))))
        return pb.Reply(ok=True, spawn=reply)

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
        self.pending.pop(req.handle, None)
        return pb.Reply(ok=True)

    def _apply(self, req):
        unknown = [p.handle for p in req.poses if p.handle not in self.entities]
        if unknown:
            raise RequestError(f"poses for unknown handle(s) {unknown}")
        for p in req.poses:
            if p.HasField("pose") or p.HasField("twist") or p.kinematic:
                self.pending[p.handle] = drive_of(p)
        return pb.Reply(ok=True, apply=pb.ApplyReply())

    def _step(self, req):
        if req.substeps < 1 or not req.dt > 0:
            raise RequestError(f"step of {req.substeps} x {req.dt} s")
        if req.poses:
            raise RequestError("Step carries no poses since protocol 4: send them by Apply")
        drives, self.pending = self.pending, {}
        started = time.perf_counter()
        states, contacts = self.world.step(drives, req.substeps, req.dt, req.render,
                                           self.sim_time)
        elapsed = time.perf_counter() - started
        self.step_ms.append(1e3 * elapsed)
        self.sim_time += req.substeps * req.dt
        self.frame += 1
        reply = pb.StepReply(sim_time=self.sim_time, frame=self.frame, tick_seconds=elapsed)
        for handle, pose, twist in states:
            state = reply.states.add(handle=handle, pose=pb.make_pose(*pose))
            if twist is not None:
                state.twist.CopyFrom(pb.make_twist(*twist))
        for handle, other, path, impulse in contacts:
            reply.contacts.add(handle=handle, other=other, other_path=path, impulse=impulse)
        return pb.Reply(ok=True, step=reply)

    def _set_step_time(self, _req):
        return pb.Reply(ok=True)

    def _set_signals(self, req):
        if not self.offers("SIGNALS"):
            raise RequestError("traffic signals are not offered by this server")
        self.world.set_signals({s.opendrive_id: pb.COLOUR_NAME.get(s.colour, "OFF")
                                for s in req.states})
        return pb.Reply(ok=True)

    def _set_goal(self, _req):
        raise RequestError("goals are not offered by this server (no AUTOPILOT)")

    def _set_weather(self, _req):
        raise RequestError("weather is not offered by this server")

    def _get_weather(self, _req):
        raise RequestError("weather is not offered by this server")

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


def bind(port, host="127.0.0.1"):
    """The REP socket on `tcp://host:port`, bound now: a server binds before its simulator's
    long start, so a second one on the port fails at once (`Address already in use`) and
    exits 1, which its supervisor does not respawn. Requests that arrive before `run` wait
    in the socket's queue."""
    import zmq

    socket = zmq.Context.instance().socket(zmq.REP)
    try:
        socket.bind(f"tcp://{host}:{port}")
    except zmq.ZMQError as e:
        # A supervisor respawns on any exit: wait, so a refusal is not a loop.
        log.error("cannot bind tcp://%s:%d: %s (another world server holds it); exiting in "
                  "%d s", host, port, e, REFUSED_PAUSE)
        time.sleep(REFUSED_PAUSE)
        raise SystemExit(1) from e
    return socket


def run(world, socket, should_stop=lambda: False):
    """Serve `world` on the bound `socket` until `Shutdown`, SIGINT/SIGTERM or
    `should_stop()`."""
    server = WorldServer(world)
    stop = {"now": False}

    def on_signal(signum, _frame):
        log.info("signal %d: stopping", signum)
        stop["now"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    import zmq

    log.info("%s world server ready on %s (%s, pid %d, capabilities %s)",
             world.simulator_name, socket.getsockopt_string(zmq.LAST_ENDPOINT),
             world.simulator_version, os.getpid(),
             ", ".join(sorted(world.capabilities)) or "core only")
    try:
        serve(socket, server, should_stop=lambda: stop["now"] or should_stop())
    finally:
        log.info("world server stopping after %d step(s), sim time %.3f s",
                 server.frame, server.sim_time)
        socket.close(linger=0)
        world.close()
    return server
