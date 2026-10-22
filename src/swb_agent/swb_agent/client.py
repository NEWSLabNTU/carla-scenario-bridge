"""The SDK's side of the agent protocol: one registered connection to a relay.

Owns everything an agent author should not write: registration, the version fallback
(a v2 client on a v1 relay speaks v1 from then on), heartbeats and state pushes,
reconnecting (backoff 1 -> 5 s), and command ordering -- one command at a time, the state
it produced pushed before its reply. Standard library only.
"""

import inspect
import logging
import queue
import socket
import threading
import time
import traceback
from urllib.parse import urlparse

from . import protocol as P
from .agent import Agent, Episode, Failed

STATE_PERIOD = 0.2  # state pushes: changes at once, otherwise at 5 Hz (also the heartbeat)
RECONNECT_MIN, RECONNECT_MAX = 1.0, 5.0
# A repeat of an idempotent command with the same arguments within this long of its
# success is answered from memory (the concealer retries a slow answer).
REPEAT_WINDOW = 10.0
IDEMPOTENT = (P.SET_GOAL, P.TELEPORTED, P.RESET)

# The phase a command leaves when its hook reported none.
DEFAULT_PHASE = {P.RESET: P.IDLE, P.TELEPORTED: P.IDLE, P.SET_GOAL: P.PLANNING,
                 P.CLEAR_GOAL: P.IDLE, P.STOP: P.STOPPED}


def parse_relay(url):
    """tcp://host:port (or host:port) -> (host, port)."""
    parsed = urlparse(url if "://" in url else "tcp://" + url)
    if parsed.scheme != "tcp" or not parsed.hostname:
        raise ValueError(f"relay must be tcp://host:port, got {url!r}")
    return parsed.hostname, parsed.port or 5560


class _VersionRefused(Exception):
    pass


class Client:
    """Runs `agent` against the relay at host:port as `entity` until `stop()`."""

    def __init__(self, agent: Agent, host, port, entity=None, logger=None):
        self.agent = agent
        self.host, self.port = host, port
        self.entity = entity or agent.entity
        self.log = logger or logging.getLogger("swb_agent")
        self.version = P.VERSION
        self.registered = threading.Event()  # set while a registration is live
        self._stop = threading.Event()
        self._sock = None
        self._send_lock = threading.Lock()
        self._wake = threading.Event()
        self._commands = queue.Queue()
        self._last = None  # (name, args, status, text, finished_at) of the last command
        self._last_phase = None
        agent._on_report = self._wake.set
        self._worker = threading.Thread(target=self._work, daemon=True)
        self._worker.start()

    # -- lifecycle ----------------------------------------------------------------------------
    def stop(self):
        self._stop.set()
        self._wake.set()
        self._commands.put(None)
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def run(self):
        delay = RECONNECT_MIN
        while not self._stop.is_set():
            try:
                self._session()
                delay = RECONNECT_MIN
            except _VersionRefused as e:
                self.log.warning(f"relay {self.host}:{self.port} refused protocol "
                                 f"v{self.version} ({e}); speaking v1")
                self.version = 1
                delay = 0.0
            except (OSError, P.ProtocolError) as e:
                if not self._stop.is_set():
                    self.log.warning(f"relay {self.host}:{self.port}: {e}; retrying in "
                                     f"{delay:.0f} s")
            finally:
                self.registered.clear()
                self._close()
            if self._stop.wait(delay):
                return
            delay = min(max(delay, RECONNECT_MIN / 2) * 2, RECONNECT_MAX)

    # -- connection ---------------------------------------------------------------------------
    def _close(self):
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _send(self, message):
        data = P.encode(message, self.version)
        with self._send_lock:
            if self._sock is None:
                raise OSError("not connected")
            self._sock.sendall(data)

    def push_state(self):
        message = self.agent.state_message()
        self._send(message)
        if message["phase"] != self._last_phase:
            detail = message.get("detail")
            self.log.info(f"phase {self._last_phase} -> {message['phase']}"
                          + (f" ({detail})" if detail else ""))
            self._last_phase = message["phase"]

    def _session(self):
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.5)
        self._sock = sock
        name = self.agent.name or type(self.agent).__name__
        self._send(P.register(self.entity, self.agent.capabilities(), name))
        reader = P.LineReader()
        last_received = time.monotonic()
        while not self._stop.is_set():
            try:
                data = sock.recv(65536)
            except socket.timeout:
                if time.monotonic() - last_received > P.PEER_TIMEOUT:
                    raise OSError(f"relay silent for {P.PEER_TIMEOUT:.0f} s")
                continue
            if not data:
                raise OSError("relay closed the connection")
            last_received = time.monotonic()
            for line in reader.feed(data):
                message = P.decode(line)
                kind = message["type"]
                if kind == P.REGISTERED:
                    self.log.info(f"registered with relay {self.host}:{self.port} as "
                                  f"{self.entity!r} (protocol v{self.version})")
                    self.registered.set()
                    threading.Thread(target=self._push_loop, args=(sock,),
                                     daemon=True).start()
                elif kind == P.ERROR:
                    reason = message["reason"]
                    if (not self.registered.is_set() and self.version > 1
                            and "version" in reason):
                        raise _VersionRefused(reason)
                    raise P.ProtocolError(f"relay refused: {reason}")
                elif kind == P.COMMAND:
                    if not self.registered.is_set():
                        raise P.ProtocolError("command before registration")
                    self._commands.put((sock, message))

    def _push_loop(self, sock):
        while not self._stop.is_set() and self._sock is sock:
            try:
                self.push_state()
            except OSError:
                return
            self._wake.wait(STATE_PERIOD)
            self._wake.clear()

    # -- commands -----------------------------------------------------------------------------
    def _work(self):
        while True:
            item = self._commands.get()
            if item is None:
                return
            sock, message = item
            name, args = message["command"], message.get("args", {})
            status, text = self.execute(name, args)
            if self._sock is not sock:
                continue  # the connection the command came on is gone; nobody waits
            try:
                self.push_state()  # state first: the relay answers with what it shows
                self._send(P.reply(message["id"], status, text))
            except OSError:
                pass

    def execute(self, name, args):
        """Run one command's hook: (status, message). Never raises."""
        now = time.monotonic()
        if (name in IDEMPOTENT and self._last is not None and self._last[:2] == (name, args)
                and self._last[2] == P.OK and now - self._last[4] < REPEAT_WINDOW):
            return P.OK, "already done"
        self.log.info(f"command {name}")
        if name in (P.RESET, P.TELEPORTED) and self.agent.overrides("on_reset"):
            # The relay may answer the scenario before the reset is done; until the
            # agent says otherwise it is re-initializing, not ready for a goal.
            self.agent.report(P.INITIALIZING)
        reports = self.agent._reports
        try:
            text = self._dispatch(name, args)
            status = P.OK
        except NotImplementedError:
            status, text = P.UNSUPPORTED, f"{type(self.agent).__name__} does not offer {name}"
        except Failed as e:
            status, text = P.FAILED, str(e)
        except Exception as e:  # a bug in a hook must not leave the relay waiting
            self.log.error(f"command {name}: {traceback.format_exc()}")
            status, text = P.FAILED, f"{type(e).__name__}: {e}"
        if status == P.OK and name in DEFAULT_PHASE and self.agent._reports == reports:
            self.agent.report(DEFAULT_PHASE[name])
        self._last = (name, args, status, text, time.monotonic())
        self.log.info(f"command {name} -> {status}" + (f": {text}" if text else ""))
        return status, text

    def _dispatch(self, name, args):
        a = self.agent
        if name in (P.RESET, P.TELEPORTED):
            if name == P.TELEPORTED and not a.overrides("on_reset"):
                return "nothing to do"  # an event; an agent without reset ignores it
            if name == P.RESET:
                ep = Episode(episode=args.get("episode", ""),
                             scenario=args.get("scenario", ""), map=args.get("map", ""),
                             pose=args["pose"], parameters=dict(args.get("parameters", {})))
            else:
                ep = Episode(pose=args["pose"])
            a.episode = ep
            a.on_reset(ep)
        elif name == P.SET_GOAL:
            extra = {k: v for k, v in args.items() if k not in ("goal", "waypoints")}
            _call(a.on_goal, args["goal"], list(args.get("waypoints", [])), **extra)
        elif name == P.CLEAR_GOAL:
            a.on_clear_goal()
        elif name == P.STOP:
            a.on_stop()
        elif name == P.SET_SPEED_LIMIT:
            a.on_speed_limit(args.get("mps"))
        elif name == P.EPISODE_END:
            a.on_episode_end(a.episode or Episode(episode=args.get("episode", "")),
                             args["result"])
        elif name == P.RESTART:
            a.on_restart()
        elif name == P.COOPERATE:
            a.on_cooperate(args["module"], args["command"], args.get("uuid"))
        elif name == P.COOPERATE_AUTO:
            a.on_cooperate_auto(args["module"], args["enable"])
        else:
            raise NotImplementedError
        return ""


def _call(hook, *args, **options):
    """Call `hook` with only the keyword options its signature accepts."""
    params = inspect.signature(hook).parameters.values()
    if not any(p.kind is p.VAR_KEYWORD for p in params):
        names = {p.name for p in params}
        options = {k: v for k, v in options.items() if k in names}
    return hook(*args, **options)
