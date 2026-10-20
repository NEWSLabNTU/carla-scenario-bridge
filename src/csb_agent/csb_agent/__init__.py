"""csb_agent: drive carla-scenario-bridge scenarios with any autopilot.

    import csb_agent

    class MyAgent(csb_agent.Agent):
        def on_reset(self, ep): ...
        def on_goal(self, goal, waypoints): ...
        def on_stop(self): ...

    if __name__ == "__main__":
        csb_agent.main(MyAgent)

Roadmap 030; spec in docs/design/user-workflow.md ("The agent protocol (I3)").
"""

import argparse
import json
import logging
import os
import signal
import threading

from . import protocol
from .agent import Agent, Episode, Failed
from .client import Client, parse_relay
from .commander import Commander
from .protocol import (ARRIVED, DRIVING, EMERGENCY, FAULT_NONE, IDLE, INITIALIZING,
                       MINIMAL_RISK_MANEUVER, PLANNING, READY, STOPPED, UNAVAILABLE)

__all__ = ["Agent", "Episode", "Failed", "Client", "Commander", "main", "load_config",
           "protocol",
           "UNAVAILABLE", "INITIALIZING", "IDLE", "PLANNING", "READY", "DRIVING", "ARRIVED",
           "STOPPED", "FAULT_NONE", "MINIMAL_RISK_MANEUVER", "EMERGENCY"]

DEFAULT_RELAY = "tcp://localhost:5560"


def load_config(path):
    """A JSON or YAML file (YAML needs PyYAML) as a dict."""
    with open(path) as f:
        text = f.read()
    if path.endswith((".yaml", ".yml")):
        import yaml  # optional dependency, only for YAML configs
        return yaml.safe_load(text) or {}
    return json.loads(text)


def main(agent_class, argv=None):
    """Command line entry point: build `agent_class(config)` and run it until signalled.

    --relay (default $CSB_RELAY, else tcp://localhost:5560), --entity (default the
    config's `entity`, else "ego"), --config (JSON or YAML, passed to the constructor).
    """
    parser = argparse.ArgumentParser(description=f"csb_agent: {agent_class.__name__}")
    parser.add_argument("--relay", default=None)
    parser.add_argument("--entity", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = load_config(args.config) if args.config else {}
    if args.entity:
        config["entity"] = args.entity
    relay = args.relay or config.get("relay") or os.environ.get("CSB_RELAY") or DEFAULT_RELAY
    host, port = parse_relay(relay)
    agent = agent_class(config)
    client = Client(agent, host, port)
    done = threading.Event()

    def stop(*_):
        done.set()
        client.stop()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    logging.getLogger("csb_agent").info(
        f"{agent_class.__name__} for entity {agent.entity!r}: relay tcp://{host}:{port}")
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    while not done.wait(0.5):
        pass
    thread.join(timeout=5.0)
    agent.close()
    return 0
