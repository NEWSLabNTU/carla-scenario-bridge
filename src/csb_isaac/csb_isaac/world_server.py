"""csb's Isaac Sim world server (roadmap 037): `python -m csb_isaac.world_server`.

Starts Isaac Sim (headless unless `--display`), builds the world, and serves csb's world
protocol on a ZMQ REP socket bound to localhost until csb sends `Shutdown` or the process
gets SIGINT/SIGTERM. Run it through `just isaac-world`, which sets Isaac's environment.

    python -m csb_isaac.world_server --port 5590 [--display :8]
"""
import argparse
import logging
import os
import signal
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=5590, help="REP port on 127.0.0.1")
    ap.add_argument("--display", default="",
                    help="X display for Isaac's window (e.g. :8, the VNC one); empty: headless")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, stream=sys.stdout,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("csb_isaac")

    if args.display:
        os.environ["DISPLAY"] = args.display
    else:
        os.environ.pop("DISPLAY", None)
    started = time.monotonic()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.display})

    import zmq

    from .server import WorldServer, serve
    from .world import IsaacWorld

    world = IsaacWorld()
    server = WorldServer(world)
    socket = zmq.Context.instance().socket(zmq.REP)
    socket.bind(f"tcp://127.0.0.1:{args.port}")
    stop = {"now": False}

    def on_signal(signum, _frame):
        log.info("signal %d: stopping", signum)
        stop["now"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    log.info("Isaac Sim %s up in %.1f s; world server ready on tcp://127.0.0.1:%d",
             world.isaac_version, time.monotonic() - started, args.port)
    try:
        serve(socket, server, should_stop=lambda: stop["now"] or not app.is_running())
    finally:
        log.info("world server stopping after %d step(s), sim time %.3f s",
                 server.frame, server.sim_time)
        socket.close(linger=0)
        world.close()
        app.close()


if __name__ == "__main__":
    main()
