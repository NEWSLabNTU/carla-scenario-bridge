"""csb's Isaac Sim world server (roadmap 037): `python -m csb_isaac.world_server`.

Starts Isaac Sim (headless unless `--display`), builds the world, and serves csb's world
protocol on a ZMQ REP socket bound to localhost until csb sends `Shutdown` or the process
gets SIGINT/SIGTERM. Run it through `just isaac-world`, which sets Isaac's environment.

    python -m csb_isaac.world_server --port 5590 [--display :8] [--ego <aib ego config>]
        [--world <world>[,<world>...]] [--follow]

A world is a world file (`<map>/isaac/world.yaml`, as csb's `LoadWorld` reads it), a map
directory (its `isaac/world.yaml`) or a USD file. Several, comma separated, are all loaded and
rendered once before the server answers (roadmap 043: a town's first render compiles its
materials, minutes, which a scenario's `LoadWorld` should not have to); the first one listed
stays loaded. A world file's map directory with a `surfaces.yaml` (roadmap 047) gives that
world its surface map, checked here before Isaac starts.
"""
import argparse
import logging
import os
import signal
import sys
import time


def world_file(entry):
    """The world file a `--world` entry names (a map directory's `isaac/world.yaml`), or
    None for a USD file."""
    path = os.path.abspath(os.path.expanduser(entry))
    if os.path.isdir(path):
        path = os.path.join(path, "isaac", "world.yaml")
    return path if path.endswith((".yaml", ".yml")) else None


def world_usd(entry):
    """The USD stage a `--world` entry names: a world file's `usd` (relative to its map
    directory, as csb resolves it), a map directory's `isaac/world.yaml`, or a USD file."""
    path = world_file(entry)
    if path is None:
        return os.path.abspath(os.path.expanduser(entry))
    import yaml

    with open(path) as f:
        usd = (yaml.safe_load(f) or {}).get("usd", "")
    if not usd:
        raise SystemExit(f"{entry}: the world file names no usd (a flat ground plane needs no "
                         "preload)")
    map_dir = os.path.dirname(os.path.dirname(path))
    return usd if os.path.isabs(usd) else os.path.join(map_dir, usd)


def preload_list(worlds):
    """`--world`'s comma-separated entries as USD paths, checked before Isaac starts."""
    usds = list(dict.fromkeys(world_usd(e.strip()) for e in worlds.split(",") if e.strip()))
    missing = [u for u in usds if not os.path.exists(u)]
    if missing:
        raise SystemExit(f"world(s) not found: {', '.join(missing)}")
    return usds


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=5590, help="REP port on 127.0.0.1")
    ap.add_argument("--display", default="",
                    help="X display for Isaac's window (e.g. :8, the VNC one); empty: headless")
    ap.add_argument("--follow", action="store_true",
                    help="with --display: the viewport follows the ego from behind, no UI "
                         "(for watching or recording a run)")
    ap.add_argument("--world", default="",
                    help="worlds to load and render before serving, comma separated: world "
                         "files, map directories or USD files (a first render compiles the "
                         "world's materials); the first stays loaded, and csb's LoadWorld of "
                         "it keeps it")
    ap.add_argument("--ego", default="",
                    help="aib ego config (autoware_isaac_bridge's config/ego.yaml): the ego "
                         "is aib's PhysX vehicle driven by Autoware (038, 045); empty: a "
                         "kinematic box (037). aib must be on PYTHONPATH")
    ap.add_argument("--ego-domain", type=int, default=0,
                    help="the ego's ROS domain, over the ego config's (the ego stack's "
                         "ROS_DOMAIN_ID); 0: the config's")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, stream=sys.stdout,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("csb_isaac")

    preload = preload_list(args.world)
    from .surface_layer import register
    from .surfaces import SurfaceError

    entries = [e.strip() for e in args.world.split(",") if e.strip()]
    try:
        # A world's surface map (<map>/surfaces.yaml, roadmap 047), checked before Isaac starts.
        surface_maps = register([w for w in map(world_file, entries) if w])
    except SurfaceError as e:
        raise SystemExit(f"surface map refused:\n{e}") from e
    for usd, (path, _) in surface_maps.items():
        log.info("surface map %s for %s", path, usd)
    if args.display:
        os.environ["DISPLAY"] = args.display
    else:
        os.environ.pop("DISPLAY", None)
    started = time.monotonic()
    from isaacsim import SimulationApp

    # Kit uploads a crash's minidump to NVIDIA by default: kept on disk only (its dumpDir).
    config = {"headless": not args.display, "extra_args": ["--/crashreporter/url="]}
    # Host-specific Kit settings (a host setting, ISAAC_KIT_ARGS): e.g. one GPU on a host
    # whose other GPU is busy, "--/renderer/multiGpu/enabled=false --/renderer/activeGpu=1".
    config["extra_args"] += os.environ.get("ISAAC_KIT_ARGS", "").split()
    if args.display and args.follow:
        # A small render keeps the frame inside its 50 ms beside the lidar (1080p: 49.5 ms
        # p50 on a loaded host, 720p: 52); the window scales it up.
        config.update(hide_ui=True, width=854, height=480, window_width=1920,
                      window_height=1080)
    app = SimulationApp(config)

    import zmq

    from .server import WorldServer, serve
    from .world import IsaacWorld

    world = IsaacWorld(ego_config=args.ego or None, ego_domain=args.ego_domain or None,
                       follow=bool(args.display and args.follow), surface_maps=surface_maps)
    # The first listed is preloaded last, so it is the one loaded when csb first asks.
    for usd in reversed(preload):
        world.preload(usd)
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
