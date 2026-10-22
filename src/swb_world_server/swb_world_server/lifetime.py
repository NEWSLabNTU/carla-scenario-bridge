"""Process lifetime for world servers and their clients (roadmap 046).

- `die_with_parent()`: SIGTERM from the kernel when the parent dies (a play_launch killed
  with SIGKILL), and exit at once when it died while that was being set up.
- `client_id()`: this process as the protocol's `ClientId`.
- `is_gone(client)`: whether a `ClientId` names a process that provably no longer runs.
"""
import ctypes
import os
import signal
import socket
import sys

PR_SET_PDEATHSIG = 1


def die_with_parent():
    """Ask for SIGTERM when the parent dies; exit 1 if it died meanwhile. Linux only;
    elsewhere a no-op."""
    if not sys.platform.startswith("linux"):
        return
    parent = os.getppid()
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.prctl(PR_SET_PDEATHSIG, int(signal.SIGTERM), 0, 0, 0)
    except OSError:
        return
    # The parent died while the signal was being asked for: it will never come. A process
    # started detached (its parent already gone) has no supervisor to follow and runs on.
    if os.getppid() != parent:
        sys.stderr.write("parent process gone while starting: exiting\n")
        sys.exit(1)


def start_time(pid):
    """A process's start time, clock ticks since boot (`/proc/<pid>/stat` field 22), or None
    when it does not run."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            stat = f.read()
    except OSError:
        return None
    # The command name (field 2) may hold spaces and parentheses: split after its last ')'.
    fields = stat[stat.rindex(")") + 2:].split()
    return int(fields[19])


def hostname():
    return socket.gethostname()


def client_id(pb):
    """This process as a `ClientId` (`pb` is `swb_world_server.protocol`)."""
    return pb.ClientId(host=hostname(), pid=os.getpid(), start_time=start_time(os.getpid()) or 0)


def is_gone(client):
    """Whether `client` provably no longer runs: on this host, and its pid is free or now
    belongs to a process started at another time. A client on another host is never
    provably gone (its lease decides)."""
    if not client.host or client.host != hostname():
        return False
    started = start_time(client.pid)
    return started is None or (client.start_time != 0 and started != client.start_time)
