"""The relay's end of the agent protocol: a TCP server agents register with.

Standard library only (socket, threading) and no ROS, so the whole protocol can be tested
against a fake agent on localhost.
"""

import itertools
import logging
import socket
import threading
import time

from . import protocol as P

# A commander's command_for waits this long for the agent's reply unless it names its own
# timeout, which is capped at COMMANDER_MAX_TIMEOUT.
COMMANDER_TIMEOUT = 9.0
COMMANDER_MAX_TIMEOUT = 30.0


class AgentSession:
    """One registered agent connection."""

    def __init__(self, server, sock, address, entity, agent, capabilities):
        self.server = server
        self.sock = sock
        self.address = address
        self.entity = entity
        self.agent = agent
        self.capabilities = list(capabilities)
        self.state = None  # latest state message
        self.state_received_at = None  # time.monotonic() of it
        self.last_received = time.monotonic()
        self.last_sent = 0.0
        self.closed = threading.Event()
        self._send_lock = threading.Lock()
        self._pending = {}  # command id -> [Event, reply or None]
        self._pending_lock = threading.Lock()

    def send(self, message) -> bool:
        data = P.encode(message)
        with self._send_lock:
            if self.closed.is_set():
                return False
            try:
                self.sock.sendall(data)
                self.last_sent = time.monotonic()
                return True
            except OSError:
                self.close("send failed")
                return False

    def close(self, reason=""):
        if self.closed.is_set():
            return
        self.closed.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        with self._pending_lock:
            for slot in self._pending.values():
                slot[0].set()
        self.server._session_closed(self, reason)

    def command(self, command_id, name, args, timeout):
        """Send a command and wait for its reply: (status, message)."""
        slot = [threading.Event(), None]
        with self._pending_lock:
            self._pending[command_id] = slot
        try:
            if not self.send({"type": P.COMMAND, "id": command_id, "command": name,
                              "args": args}):
                return None, "agent connection lost"
            if not slot[0].wait(timeout) or slot[1] is None:
                if self.closed.is_set():
                    return None, "agent connection lost"
                return None, f"agent did not reply to {name} within {timeout:.1f} s"
            reply = slot[1]
            return reply["status"], reply.get("message", "")
        finally:
            with self._pending_lock:
                self._pending.pop(command_id, None)

    def _on_reply(self, message):
        with self._pending_lock:
            slot = self._pending.get(message["id"])
        if slot is not None:
            slot[1] = message
            slot[0].set()


class CommanderSession:
    """One commander connection: request/response, no heartbeat in either direction.

    A commander (e.g. a simulator adapter driving an entity whose controller is "agent")
    asks the relay to command the agent registered under an entity's name. It owns no
    entity and is never timed out for silence; a closed socket ends it.
    """

    def __init__(self, sock, address, agent):
        self.sock = sock
        self.address = address
        self.agent = agent
        self.closed = threading.Event()
        self._send_lock = threading.Lock()

    def send(self, message) -> bool:
        data = P.encode(message)
        with self._send_lock:
            if self.closed.is_set():
                return False
            try:
                self.sock.sendall(data)
                return True
            except OSError:
                self.close()
                return False

    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        for call in (lambda: self.sock.shutdown(socket.SHUT_RDWR), self.sock.close):
            try:
                call()
            except OSError:
                pass


class AgentServer:
    """Accepts agents, keeps one session per entity, relays commands and state.

    on_state(entity, state, received_at) is called for every state message, and
    on_presence(entity, present) when an entity gains or loses its agent. Both run on
    the server's threads.
    """

    def __init__(self, host="0.0.0.0", port=5560, on_state=None, on_presence=None,
                 peer_timeout=P.PEER_TIMEOUT, heartbeat_period=P.HEARTBEAT_PERIOD,
                 logger=None):
        self.host = host
        self.port = port
        self.on_state = on_state or (lambda *a: None)
        self.on_presence = on_presence or (lambda *a: None)
        self.peer_timeout = peer_timeout
        self.heartbeat_period = heartbeat_period
        self.log = logger or logging.getLogger("scenario_agent_relay")
        self._sessions = {}
        self._commanders = set()
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._stopping = threading.Event()
        self._listener = None
        self._threads = []

    # -- lifecycle --------------------------------------------------------------------
    def start(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.port))
        listener.listen(8)
        listener.settimeout(0.5)
        self.port = listener.getsockname()[1]  # resolves port 0 in tests
        self._listener = listener
        for target in (self._accept_loop, self._heartbeat_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self._stopping.set()
        with self._lock:
            sessions = list(self._sessions.values())
            commanders = list(self._commanders)
        for session in sessions:
            session.close("relay stopping")
        for commander in commanders:
            commander.close()
        if self._listener is not None:
            self._listener.close()
        for thread in self._threads:
            thread.join(timeout=2.0)

    # -- queries and commands -----------------------------------------------------------
    def session(self, entity):
        with self._lock:
            session = self._sessions.get(entity)
        if session is None or session.closed.is_set():
            return None
        return session

    def entities(self):
        with self._lock:
            return sorted(self._sessions)

    def command(self, entity, name, timeout, **args):
        """Command the entity's agent: (status, message); status None = no answer."""
        session = self.session(entity)
        if session is None:
            return None, f"no agent registered for entity {entity!r}"
        return session.command(next(self._ids), name, args, timeout)

    # -- threads ------------------------------------------------------------------------
    def _accept_loop(self):
        while not self._stopping.is_set():
            try:
                sock, address = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            thread = threading.Thread(target=self._serve, args=(sock, address), daemon=True)
            thread.start()

    def _serve(self, sock, address):
        reader = P.LineReader()
        session = None
        sock.settimeout(0.5)
        deadline = time.monotonic() + self.peer_timeout
        try:
            while not self._stopping.is_set():
                if session is not None and session.closed.is_set():
                    return
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    if session is None and time.monotonic() > deadline:
                        self._reject(sock, "no register message within "
                                     f"{self.peer_timeout:.0f} s")
                        return
                    continue
                except OSError:
                    data = b""
                if not data:
                    if session is not None:
                        session.close("agent closed the connection")
                    return
                lines = reader.feed(data)
                for index, line in enumerate(lines):
                    message = P.decode(line)
                    if session is None:
                        if message["type"] != P.REGISTER:
                            raise P.ProtocolError("the first message must be register")
                        if message.get("role") == P.ROLE_COMMANDER:
                            self._serve_commander(sock, address, message, reader,
                                                  lines[index + 1:])
                            return
                        session = self._register(sock, address, message)
                        continue
                    session.last_received = time.monotonic()
                    kind = message["type"]
                    if kind == P.STATE:
                        session.state = message
                        session.state_received_at = session.last_received
                        if message.get("capabilities"):
                            session.capabilities = list(message["capabilities"])
                        self.on_state(session.entity, message, session.last_received)
                    elif kind == P.REPLY:
                        session._on_reply(message)
                    elif kind == P.ERROR:
                        session.close(f"agent error: {message['reason']}")
                        return
                    elif kind == P.REGISTER:
                        raise P.ProtocolError("already registered")
        except P.ProtocolError as e:
            self.log.warning(f"agent {address[0]}:{address[1]}: {e}; closing")
            if session is not None:
                session.send(P.error(str(e)))
                session.close(str(e))
            else:
                self._reject(sock, str(e))

    # -- commanders -------------------------------------------------------------------------
    def _serve_commander(self, sock, address, message, reader, pending):
        commander = CommanderSession(sock, address, message.get("agent", ""))
        with self._lock:
            self._commanders.add(commander)
        commander.send(P.registered(""))
        self.log.info(f"commander {commander.agent or '?'} connected from "
                      f"{address[0]}:{address[1]}")
        reason = "commander closed the connection"
        try:
            # Lines that arrived together with the register message come first.
            while not self._stopping.is_set() and not commander.closed.is_set():
                for line in pending:
                    self._on_commander_message(commander, P.decode(line))
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    pending = []
                    continue
                except OSError:
                    data = b""
                if not data:
                    break
                pending = reader.feed(data)
        except P.ProtocolError as e:
            reason = str(e)
            self.log.warning(f"commander {address[0]}:{address[1]}: {e}; closing")
            commander.send(P.error(str(e)))
        finally:
            commander.close()
            with self._lock:
                self._commanders.discard(commander)
            self.log.info(f"commander {commander.agent or '?'} gone ({reason})")

    def _on_commander_message(self, commander, message):
        kind = message["type"]
        if kind == P.QUERY:
            session = self.session(message["entity"])
            if session is None:
                commander.send(P.reply(message["id"], P.FAILED, "no agent registered for "
                                       f"entity {message['entity']!r}", registered=False))
            else:
                commander.send(P.reply(message["id"], P.OK, "", session.state,
                                       registered=True))
        elif kind == P.COMMAND_FOR:
            # Its own thread: a slow agent must not hold up the commander's other requests.
            threading.Thread(target=self._command_for, args=(commander, message),
                             daemon=True).start()
        elif kind == P.HEARTBEAT:
            pass
        else:
            raise P.ProtocolError(f"a commander cannot send {kind}")

    def _command_for(self, commander, message):
        entity, name = message["entity"], message["command"]
        timeout = min(float(message.get("timeout") or COMMANDER_TIMEOUT),
                      COMMANDER_MAX_TIMEOUT)
        status, text = self.command(entity, name, timeout, **message.get("args", {}))
        if status is None:  # no agent, no answer, or the connection dropped
            status = P.FAILED
        session = self.session(entity)
        self.log.info(f"commander {commander.agent or '?'}: {name} for {entity!r} -> "
                      f"{status}" + (f": {text}" if text else ""))
        commander.send(P.reply(message["id"], status, text,
                               session.state if session is not None else None,
                               registered=session is not None))

    def _reject(self, sock, reason):
        try:
            sock.sendall(P.encode(P.error(reason)))
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _register(self, sock, address, message):
        entity = message["entity"]
        session = AgentSession(self, sock, address, entity, message.get("agent", ""),
                               message.get("capabilities", []))
        with self._lock:
            previous = self._sessions.get(entity)
            self._sessions[entity] = session
        if previous is not None and not previous.closed.is_set():
            # A restarted agent registers before its old connection times out; the
            # newest registration wins.
            self.log.warning(f"entity {entity!r}: a new agent registered; dropping the "
                             f"previous connection from {previous.address[0]}")
            previous.close("replaced by a new registration")
        session.send(P.registered(entity))
        self.log.info(f"entity {entity!r}: agent {session.agent or '?'} registered from "
                      f"{address[0]}:{address[1]} (capabilities: "
                      f"{', '.join(session.capabilities) or 'none'})")
        self.on_presence(entity, True)
        return session

    def _session_closed(self, session, reason):
        with self._lock:
            current = self._sessions.get(session.entity) is session
            if current:
                del self._sessions[session.entity]
        if current:
            self.log.warning(f"entity {session.entity!r}: agent gone ({reason})")
            self.on_presence(session.entity, False)

    def _heartbeat_loop(self):
        while not self._stopping.wait(min(0.2, self.heartbeat_period / 2)):
            now = time.monotonic()
            with self._lock:
                sessions = list(self._sessions.values())
            for session in sessions:
                if now - session.last_received > self.peer_timeout:
                    session.close(f"silent for {now - session.last_received:.1f} s")
                elif now - session.last_sent >= self.heartbeat_period:
                    session.send(P.heartbeat())
