"""Scenario boundaries: what the relay sends an agent on SSv2's `initialize` (roadmap 030).

An agent with the `reset` capability (protocol v2) gets one `reset` carrying a new episode
id; any other agent gets `teleported`, as in v1. A reset may take longer than the
concealer waits for an answer (an autopilot re-localizing), so the relay waits up to
`reply_window` and then answers the concealer OK while the reset goes on, bounded by
`reset_timeout`; the agent reports INITIALIZING meanwhile, which holds the concealer at
"Autoware initializing" until the agent is ready. A repeat of the initialize for the same
pose while that reset runs (the concealer retries) joins it instead of starting another.

How each reset went is kept per entity and served to commanders (`query` -> `reset`), which
is how `run_suite` learns that a reset failed or timed out.

ROS-free: relay_node calls `initialize()` from its service handler.
"""

import itertools
import threading
import time

from swb_agent import protocol as P

RESET_TIMEOUT = 60.0


def _same_pose(a, b, tolerance=0.05):
    return all(abs(float(a[k]) - float(b[k])) <= tolerance for k in P.POSE_KEYS)


class Episodes:
    def __init__(self, server, reset_timeout=RESET_TIMEOUT, clock=time.monotonic,
                 logger=None):
        self.server = server
        self.reset_timeout = reset_timeout
        self.clock = clock
        self.log = logger or (lambda text: None)
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._running = {}  # entity -> (pose, Event, record)

    def initialize(self, entity, pose, reply_window, scenario="", map_name="",
                   parameters=None):
        """The entity's scenario boundary: (status, message) for the concealer."""
        session = self.server.session(entity)
        if session is None:
            return None, f"no agent registered for entity {entity!r}"
        if not session.supports(P.RESET)[0]:
            return self.server.command(entity, P.TELEPORTED, reply_window, pose=pose)
        with self._lock:
            running = self._running.get(entity)
            if running is not None and _same_pose(running[0], pose):
                done, record = running[1], running[2]
                joined = True
            else:
                joined = False
                episode = f"{entity}-{next(self._ids)}-{int(time.time())}"
                record = {"episode": episode, "status": "RUNNING", "message": "",
                          "seconds": None, "timed_out": False}
                done = threading.Event()
                self._running[entity] = (pose, done, record)
                self.server.record_reset(entity, dict(record))
        if not joined:
            threading.Thread(target=self._reset, daemon=True,
                             args=(entity, pose, done, record, scenario, map_name,
                                   parameters or {})).start()
        if done.wait(reply_window):
            return record["status"], record["message"]
        return P.OK, f"reset {record['episode']} in progress"

    def _reset(self, entity, pose, done, record, scenario, map_name, parameters):
        start = self.clock()
        status, message = self.server.command(
            entity, P.RESET, self.reset_timeout, episode=record["episode"],
            scenario=scenario, map=map_name, pose=pose, parameters=parameters)
        record["seconds"] = round(self.clock() - start, 3)
        record["timed_out"] = status is None and "within" in message
        record["status"] = status or P.FAILED
        record["message"] = message
        with self._lock:
            if self._running.get(entity, (None, done))[1] is done:
                del self._running[entity]
        self.server.record_reset(entity, dict(record))
        self.log(f"{entity}: reset {record['episode']} -> {record['status']} in "
                 f"{record['seconds']:.1f} s" + (f": {message}" if message else ""))
        done.set()
