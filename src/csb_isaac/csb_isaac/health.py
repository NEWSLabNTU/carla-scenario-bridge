"""Is the world server up and answering? One Hello round trip (roadmap 043).

    python -m csb_isaac.health [--port 5590] [--timeout 2] [--wait SECONDS]

Exit 0 with the server's Isaac version when it answers Hello within `--timeout`, 1 when it
does not (not started, still preloading its worlds, or wedged). `--wait` retries until it
answers or that many seconds pass. A Hello changes nothing on the server, so this is safe
beside a running scenario (the REP socket queues it between csb's requests). Needs only
zmq and protobuf: Isaac's venv python runs it without Isaac's environment.
"""
import argparse
import sys
import time

import zmq

from . import protocol as pb


def hello(port, timeout):
    """The server's Isaac version, or None when it does not answer within `timeout` s."""
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    try:
        sock.connect(f"tcp://127.0.0.1:{port}")
        req = pb.Request(hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION))
        sock.send(req.SerializeToString())
        if not sock.poll(int(timeout * 1000), zmq.POLLIN):
            return None
        reply = pb.Reply.FromString(sock.recv())
        if not reply.ok:
            raise SystemExit(f"world server on {port} refused hello: {reply.error}")
        return reply.hello.isaac_version
    finally:
        sock.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=5590)
    ap.add_argument("--timeout", type=float, default=2.0, help="one round trip, s")
    ap.add_argument("--wait", type=float, default=0.0, help="retry for this long, s")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    deadline = time.monotonic() + args.wait
    while True:
        version = hello(args.port, args.timeout)
        if version is not None:
            if not args.quiet:
                print(f"world server on {args.port}: up, Isaac Sim {version}")
            return 0
        if time.monotonic() >= deadline:
            if not args.quiet:
                print(f"world server on {args.port}: no answer to hello in {args.timeout:g} s")
            return 1
        time.sleep(min(5.0, max(0.0, deadline - time.monotonic())))


if __name__ == "__main__":
    sys.exit(main())
