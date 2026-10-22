"""`swb-agent supervise`: run an entity's autopilot and agent, restart them on request.

    swb-agent supervise --entity ego [--relay tcp://localhost:5560] \\
        [--autopilot "ros2 launch my_stack bringup.launch.py"] \\
        --agent "python3 -m my_agent --config agent.yaml" [--grace 10]

Both commands run in one process group of the supervisor's own making (a new group),
so the whole tree is signalled as one: a restart, or the supervisor's own exit, stops the
group with SIGTERM, then SIGKILL after `--grace` seconds, always by process group and never
by name. The supervisor registers with the relay as the entity's **supervisor** (protocol
v2): the relay sends it `restart` when a commander (`run_suite --restart=...`) asks for one
and the agent cannot restart itself. It answers once the new group is started; the restart
is ready when the agent registers again, which the commander watches at the relay.

The group is also started again when the agent's process exits on its own (an agent that
lost its simulator, a crash), at most once every `--respawn-delay` seconds.
"""

import argparse
import logging
import os
import shlex
import signal
import socket
import subprocess
import threading
import time

from . import protocol as P
from .client import parse_relay

log = logging.getLogger("swb_agent.supervise")


class Group:
    """The autopilot and the agent, in one process group."""

    def __init__(self, agent_cmd, autopilot_cmd=None, grace=10.0):
        self.agent_cmd = agent_cmd
        self.autopilot_cmd = autopilot_cmd
        self.grace = grace
        self.pgid = None
        self.agent = None
        self.leader = None
        self.starts = 0

    def start(self):
        first = self.autopilot_cmd or self.agent_cmd
        # the first process leads a new process group; its pid is the group's id (the same
        # session as the supervisor, so the second can join it)
        self.leader = subprocess.Popen(first, shell=True, preexec_fn=lambda: os.setpgid(0, 0))
        self.pgid = self.leader.pid
        if self.autopilot_cmd:
            pgid = self.pgid
            self.agent = subprocess.Popen(self.agent_cmd, shell=True,
                                          preexec_fn=lambda: os.setpgid(0, pgid))
        else:
            self.agent = self.leader
        self.starts += 1
        log.info(f"started group {self.pgid} (start {self.starts}): agent pid {self.agent.pid}"
                 + (f", autopilot pid {self.leader.pid}" if self.autopilot_cmd else ""))

    def alive(self):
        try:
            os.killpg(self.pgid, 0)
            return True
        except (ProcessLookupError, TypeError):
            return False
        except PermissionError:
            return True

    def agent_exited(self):
        return self.agent is not None and self.agent.poll() is not None

    def stop(self):
        """TERM the group, KILL it after the grace period, reap our children."""
        if self.pgid is None:
            return
        pgid = self.pgid
        for sig, wait in ((signal.SIGTERM, self.grace), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            end = time.monotonic() + wait
            while time.monotonic() < end and self.alive():
                self._reap()
                time.sleep(0.1)
            if not self.alive():
                break
            log.warning(f"group {pgid} still alive {wait:.0f} s after {sig.name}")
        self._reap()
        log.info(f"stopped group {pgid}")
        self.pgid = self.agent = self.leader = None

    def _reap(self):
        for p in (self.agent, self.leader):
            if p is not None:
                p.poll()


class Supervisor:
    def __init__(self, entity, relay, group, respawn_delay=10.0, name="swb-agent supervise"):
        self.entity = entity
        self.host, self.port = parse_relay(relay)
        self.group = group
        self.respawn_delay = respawn_delay
        self.name = name
        self.stopping = threading.Event()
        self._lock = threading.Lock()  # one restart at a time
        self._sock = None
        self._send_lock = threading.Lock()
        self._last_start = 0.0

    def restart(self, why):
        with self._lock:
            log.info(f"restart ({why})")
            self.group.stop()
            if self.stopping.is_set():
                return
            self.group.start()
            self._last_start = time.monotonic()

    def run(self):
        self.group.start()
        self._last_start = time.monotonic()
        threading.Thread(target=self._watch, daemon=True).start()
        backoff = 1.0
        while not self.stopping.is_set():
            try:
                self._session()
                backoff = 1.0
            except (OSError, P.ProtocolError) as e:
                if not self.stopping.is_set():
                    log.warning(f"relay {self.host}:{self.port}: {e}; retrying in "
                                f"{backoff:.0f} s")
            self.stopping.wait(backoff)
            backoff = min(backoff * 2, 5.0)
        self.group.stop()

    def stop(self):
        self.stopping.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _watch(self):
        """Start the group again when the agent exits on its own."""
        while not self.stopping.wait(1.0):
            if self.group.agent_exited() and not self._lock.locked():
                code = self.group.agent.returncode if self.group.agent else None
                if time.monotonic() - self._last_start < self.respawn_delay:
                    continue
                self.restart(f"the agent exited with status {code}")

    def _send(self, message):
        with self._send_lock:
            self._sock.sendall(P.encode(message))

    def _session(self):
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        self._sock = sock
        try:
            self._send(P.register_supervisor(self.entity, self.name))
            reader = P.LineReader()
            sock.settimeout(P.HEARTBEAT_PERIOD)
            registered = False
            last_sent = time.monotonic()
            while not self.stopping.is_set():
                if time.monotonic() - last_sent >= P.HEARTBEAT_PERIOD:
                    self._send(P.heartbeat())
                    last_sent = time.monotonic()
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    raise OSError("the relay closed the connection")
                for line in reader.feed(data):
                    message = P.decode(line)
                    kind = message["type"]
                    if kind == P.REGISTERED:
                        registered = True
                        log.info(f"registered with relay {self.host}:{self.port} as the "
                                 f"supervisor of {self.entity!r}")
                    elif kind == P.ERROR:
                        raise P.ProtocolError(f"relay: {message['reason']}")
                    elif kind == P.COMMAND and registered:
                        threading.Thread(target=self._command, args=(message,),
                                         daemon=True).start()
        finally:
            self._sock = None
            sock.close()

    def _command(self, message):
        name = message["command"]
        if name != P.RESTART:
            self._send(P.reply(message["id"], P.UNSUPPORTED, "a supervisor only restarts"))
            return
        try:
            self.restart("requested through the relay")
            status, text = P.OK, f"group {self.group.pgid} started; the agent registers when ready"
        except OSError as e:
            status, text = P.FAILED, f"could not restart: {e}"
        try:
            self._send(P.reply(message["id"], status, text))
        except OSError:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(prog="swb-agent supervise", description=__doc__.split("\n\n")[0])
    ap.add_argument("--entity", required=True)
    ap.add_argument("--agent", required=True, help="the agent's command (a shell line)")
    ap.add_argument("--autopilot", default=None, help="the autopilot's command, if separate")
    ap.add_argument("--relay", default=os.environ.get("SWB_RELAY", "tcp://localhost:5560"))
    ap.add_argument("--grace", type=float, default=10.0,
                    help="seconds between SIGTERM and SIGKILL when stopping the group")
    ap.add_argument("--respawn-delay", type=float, default=10.0)
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=a.log_level.upper(),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    sup = Supervisor(a.entity, a.relay, Group(a.agent, a.autopilot, a.grace), a.respawn_delay)
    signal.signal(signal.SIGTERM, lambda *_: sup.stop())
    signal.signal(signal.SIGINT, lambda *_: sup.stop())
    log.info(f"supervising {a.entity!r}: agent {shlex.quote(a.agent)}"
             + (f", autopilot {shlex.quote(a.autopilot)}" if a.autopilot else ""))
    sup.run()
    return 0
