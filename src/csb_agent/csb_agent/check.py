"""`csb-agent check`: an agent against a fake relay and a scripted episode.

No CARLA, SSv2 or autopilot: only the protocol. The agent class is loaded from
`module:Class`, run with the SDK's Client against a fake relay on localhost, and taken
through register -> reset -> set_goal -> set_speed_limit -> stop -> episode_end -> a
second reset -> a dropped connection. Each rule broken is reported with what was seen.

Rules (docs/design/user-workflow.md, "The agent protocol (I3)"):
  - registers within 5 s, offering every required command;
  - pushes state at least every HEARTBEAT_PERIOD (1 s; 1.5 s tolerated) while connected;
  - answers each command within its window (reset: --reset-timeout; others 9 s / 2.5 s);
  - shows the state a command produced before its reply (reset -> IDLE, set_goal -> not
    IDLE, stop -> STOPPED);
  - registers again after the relay drops the connection.
"""

import importlib
import importlib.util
import pathlib
import socket
import sys
import threading
import time
from dataclasses import dataclass, field

from . import protocol as P
from .client import RECONNECT_MAX, Client

POSE = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
GOAL = {"x": 50.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
QUICK, SLOW = 2.5, 9.0
STATE_GAP = P.HEARTBEAT_PERIOD * 1.5


class FakeRelay:
    """One agent at a time; records every state with its arrival time."""

    def __init__(self):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(2)
        self.port = self.listener.getsockname()[1]
        self.sock = None
        self.version = 1
        self.register = None
        self.registrations = 0
        self.states = []  # (time, state)
        self.gaps = []  # longest silence per connection
        self._replies = {}
        self._cond = threading.Condition()
        self._ids = iter(range(1, 1 << 30))
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(sock,), daemon=True).start()

    def _serve(self, sock):
        reader, version, last = P.LineReader(), None, None
        sock.settimeout(0.2)
        longest = 0.0
        try:
            while True:
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    if last is not None:
                        longest = max(longest, time.monotonic() - last)
                    continue
                if not data:
                    break
                now = time.monotonic()
                for line in reader.feed(data):
                    m = P.decode(line)
                    if version is None:
                        if m["type"] != P.REGISTER:
                            sock.sendall(P.encode(P.error("register first"), 1))
                            return
                        version = m["v"]
                        sock.sendall(P.encode(P.registered(m.get("entity", "")), version))
                        with self._cond:
                            self.sock, self.version = sock, version
                            self.register = m
                            self.registrations += 1
                            self._cond.notify_all()
                        last = now
                        continue
                    if last is not None:
                        longest = max(longest, now - last)
                    last = now
                    with self._cond:
                        if m["type"] == P.STATE:
                            self.states.append((now, m))
                        elif m["type"] == P.REPLY:
                            self._replies[m["id"]] = (now, m)
                        self._cond.notify_all()
        except (OSError, P.ProtocolError):
            pass
        finally:
            with self._cond:
                self.gaps.append(longest)
                if self.sock is sock:
                    self.sock = None
                self._cond.notify_all()

    def wait_registered(self, count, timeout):
        with self._cond:
            return self._cond.wait_for(lambda: self.registrations >= count, timeout)

    def command(self, name, timeout, **args):
        """(reply or None, the last state before the reply, seconds)."""
        command_id = next(self._ids)
        start = time.monotonic()
        with self._cond:
            sock, version = self.sock, self.version
        if sock is None:
            return None, None, 0.0
        sock.sendall(P.encode(P.command(command_id, name, **args), version))
        with self._cond:
            self._cond.wait_for(lambda: command_id in self._replies, timeout)
            got = self._replies.get(command_id)
            if got is None:
                return None, None, time.monotonic() - start
            at, reply = got
            before = [s for t, s in self.states if t <= at]
        return reply, (before[-1] if before else None), at - start

    def heartbeat_loop(self, stop):
        while not stop.wait(P.HEARTBEAT_PERIOD / 2):
            with self._cond:
                sock, version = self.sock, self.version
            if sock is not None:
                try:
                    sock.sendall(P.encode(P.heartbeat(), version))
                except OSError:
                    pass

    def drop(self):
        with self._cond:
            sock = self.sock
        if sock is not None:
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()

    def close(self):
        self.listener.close()
        self.drop()


@dataclass
class Report:
    results: list = field(default_factory=list)  # (ok, rule, detail)

    def check(self, ok, rule, detail=""):
        self.results.append((bool(ok), rule, detail))
        return ok

    @property
    def passed(self):
        return all(ok for ok, _, _ in self.results)

    def text(self):
        lines = [f"  {'PASS' if ok else 'FAIL'}  {rule}" + (f" -- {detail}" if detail else "")
                 for ok, rule, detail in self.results]
        verdict = "PASS" if self.passed else "FAIL"
        return "\n".join(lines + [f"{verdict}: {sum(ok for ok, _, _ in self.results)}/"
                                  f"{len(self.results)} rules"])


def load_class(spec):
    """'package.module:Class' (or a file path 'dir/agent.py:Class')."""
    module_name, _, class_name = spec.partition(":")
    if not class_name:
        raise ValueError(f"expected module:Class, got {spec!r}")
    if module_name.endswith(".py"):
        path = pathlib.Path(module_name).resolve()
        sys.path.insert(0, str(path.parent))
        loader_spec = importlib.util.spec_from_file_location(path.stem, path)
        if loader_spec is None or loader_spec.loader is None:
            raise ImportError(f"cannot load {path}")
        module = importlib.util.module_from_spec(loader_spec)
        loader_spec.loader.exec_module(module)
    else:
        module = importlib.import_module(module_name)
    return getattr(module, class_name)


def _phase(state):
    return state["phase"] if state else None


def run_check(agent_class, config=None, reset_timeout=30.0):
    report = Report()
    relay = FakeRelay()
    stop = threading.Event()
    threading.Thread(target=relay.heartbeat_loop, args=(stop,), daemon=True).start()
    agent = agent_class(dict(config or {}))
    client = Client(agent, "127.0.0.1", relay.port)
    runner = threading.Thread(target=client.run, daemon=True)
    runner.start()
    try:
        if not report.check(relay.wait_registered(1, 5.0), "registers within 5 s"):
            return report
        reg = relay.register or {}
        report.check(reg["v"] == P.VERSION, f"speaks protocol v{P.VERSION}",
                     f"registered with v{reg['v']}")
        caps = reg.get("capabilities", [])
        missing = [c for c in P.REQUIRED_COMMANDS if c not in caps]
        report.check(not missing, "offers every required command",
                     f"missing {missing}" if missing else ", ".join(caps))
        has_reset = P.RESET in caps
        time.sleep(1.0)
        report.check(relay.states, "pushes state after registering")

        def boundary(label):
            if has_reset:
                reply, state, dt = relay.command(P.RESET, reset_timeout, episode=label,
                                                 scenario="check", map="none", pose=POSE)
            else:
                reply, state, dt = relay.command(P.TELEPORTED, SLOW, pose=POSE)
            name = P.RESET if has_reset else P.TELEPORTED
            ok = report.check(reply is not None and reply["status"] == P.OK,
                              f"{label}: {name} answered OK within "
                              f"{reset_timeout if has_reset else SLOW:.0f} s",
                              _describe(reply, dt))
            if ok:
                report.check(_phase(state) == P.IDLE, f"{label}: IDLE before the reply",
                             f"phase {_phase(state)}")

        boundary("reset")
        reply, state, dt = relay.command(P.SET_GOAL, SLOW, goal=GOAL, waypoints=[])
        if report.check(reply is not None and reply["status"] in P.REPLY_STATUSES,
                        f"set_goal answered within {SLOW:.0f} s", _describe(reply, dt)):
            if reply["status"] == P.OK:
                report.check(_phase(state) not in (P.IDLE, None),
                             "set_goal: a phase past IDLE before the reply",
                             f"phase {_phase(state)}")
        reply, _, dt = relay.command(P.SET_SPEED_LIMIT, QUICK, mps=5.0)
        report.check(reply is not None, f"set_speed_limit answered within {QUICK} s",
                     _describe(reply, dt))
        reply, state, dt = relay.command(P.STOP, QUICK)
        if report.check(reply is not None, f"stop answered within {QUICK} s",
                        _describe(reply, dt)) and reply["status"] == P.OK:
            report.check(_phase(state) == P.STOPPED, "stop: STOPPED before the reply",
                         f"phase {_phase(state)}")
        if relay.version >= 2:
            reply, _, dt = relay.command(P.EPISODE_END, QUICK, episode="reset",
                                         result={"passed": True, "failures": []})
            report.check(reply is not None and reply["status"] == P.OK,
                         f"episode_end answered OK within {QUICK} s", _describe(reply, dt))
        boundary("second reset")
        # Liveness over everything so far, before the drop.
        times = [t for t, _ in relay.states]
        worst = max((b - a for a, b in zip(times, times[1:])), default=0.0)
        report.check(worst <= STATE_GAP, f"state at least every {STATE_GAP:.1f} s",
                     f"longest gap {worst:.2f} s")
        relay.drop()
        report.check(relay.wait_registered(2, RECONNECT_MAX + 4.0),
                     "registers again after a dropped connection")
        return report
    finally:
        stop.set()
        client.stop()
        relay.close()
        runner.join(timeout=3.0)


def _describe(reply, dt):
    if reply is None:
        return f"no reply after {dt:.1f} s"
    return f"{reply['status']} in {dt:.2f} s" + (f": {reply['message']}"
                                                  if reply.get("message") else "")
