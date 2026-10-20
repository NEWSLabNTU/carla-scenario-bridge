"""A commander: command or query the agent registered under an entity's name, through the
relay (protocol "Commanders"). `run_suite` uses it to send `episode_end` and to read how an
entity's last reset went; a supervisor to ask for restarts.

    with Commander("tcp://localhost:5560", "run_suite") as c:
        c.episode_end("ego", passed=False, failures=["contact"], contacts=1)
        c.query("ego")   # {"registered": ..., "state": ..., "reset": ...}

Requests are answered in order on one connection; standard library only.
"""

import itertools
import socket

from . import protocol as P
from .client import parse_relay


class CommanderError(Exception):
    pass


class Commander:
    def __init__(self, relay, name="commander", timeout=10.0):
        self.host, self.port = parse_relay(relay)
        self.name = name
        self.timeout = timeout
        self._ids = itertools.count(1)
        self._sock = None
        self._reader = P.LineReader()
        self._pending = []

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    def connect(self):
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._sock.sendall(P.encode(P.register_commander(self.name)))
        message = self._next()
        if message["type"] != P.REGISTERED:
            raise CommanderError(f"relay refused the commander: {message.get('reason')}")

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _conn(self) -> socket.socket:
        if self._sock is None:
            raise CommanderError("not connected")
        return self._sock

    def _next(self):
        while not self._pending:
            data = self._conn().recv(65536)
            if not data:
                raise CommanderError("relay closed the connection")
            self._pending += self._reader.feed(data)
        message = P.decode(self._pending.pop(0))
        if message["type"] == P.ERROR:
            raise CommanderError(f"relay: {message['reason']}")
        return message

    def _request(self, message, wait):
        self._conn().settimeout(wait + 5.0)
        self._conn().sendall(P.encode(message))
        while True:
            reply = self._next()
            if reply["type"] == P.REPLY and reply["id"] == message["id"]:
                return reply

    def command(self, entity, name, timeout=None, **args):
        """The entity's agent runs `name`: the relay's reply (status, message, state)."""
        wait = timeout or 9.0
        return self._request(P.command_for(next(self._ids), entity, name, timeout, **args),
                             wait)

    def query(self, entity):
        return self._request(P.query(next(self._ids), entity), 1.0)

    def episode_end(self, entity, passed, failures=(), contacts=None, episode=""):
        result = {"passed": bool(passed), "failures": [str(f) for f in failures]}
        if contacts is not None:
            result["contacts"] = contacts
        return self.command(entity, P.EPISODE_END, episode=episode, result=result)
