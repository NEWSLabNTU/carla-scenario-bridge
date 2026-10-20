"""`csb-agent new <name>`: a project for an agent, outside this workspace.

The generated agent stubs every hook and passes `csb-agent check` as generated; the user
fills in the bodies.
"""

import pathlib
import re

AGENT = '''"""{title}: a csb_agent agent. Fill in the hooks; run `csb-agent check {module}:{cls}`."""

import csb_agent


class {cls}(csb_agent.Agent):
    """Drives one scenario entity (default "ego") through the scenario's relay."""

    def __init__(self, config=None):
        super().__init__(config)
        # self.config is agent.yaml (or --config); connect to your autopilot here.

    def on_reset(self, ep):
        # A scenario boundary: drop the goal and plan, re-localize at ep.pose, and return
        # once ready for a goal (the SDK then reports IDLE). Raise csb_agent.Failed(...)
        # if the autopilot cannot get ready.
        self.log.info(f"reset for episode {{ep.episode!r}} at {{ep.pose}}")

    def on_goal(self, goal, waypoints):
        # Plan to `goal` through `waypoints` and start driving. Report the phase as it
        # goes: self.report(csb_agent.READY), DRIVING, and ARRIVED at the goal.
        self.log.info(f"goal {{goal}}")

    def on_speed_limit(self, mps):
        # Drive no faster than mps (None: no limit).
        pass

    def on_stop(self):
        # Stop and hold until the next goal (the SDK then reports STOPPED).
        pass

    def on_episode_end(self, ep, result):
        # The scenario's verdict: result["passed"], result["failures"]. Optional.
        self.log.info(f"episode {{ep.episode!r}}: {{'passed' if result['passed'] else 'failed'}}")


if __name__ == "__main__":
    csb_agent.main({cls})
'''

PYPROJECT = '''[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "{name}"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["csb-agent"]

[project.scripts]
{name} = "{module}:main"
'''

CONFIG = '''# Passed to {cls}(config); --relay and --entity on the command line override these.
relay: tcp://localhost:5560
entity: ego
'''

README = '''# {title}

An agent for carla-scenario-bridge scenarios (csb_agent SDK).

    csb-agent check {module}:{cls}                         # protocol conformance, no simulator
    python -m {module} --config agent.yaml                 # drive: register with the relay
'''

MAIN = '''import csb_agent

from .agent import {cls}


def main():
    return csb_agent.main({cls})
'''


def create(name, parent="."):
    """Write the project; return its directory."""
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", name):
        raise ValueError(f"name must be letters, digits, - or _ (got {name!r})")
    module = name.replace("-", "_").lower()
    cls = "".join(part.capitalize() for part in re.split(r"[-_]", name)) + "Agent"
    if cls.endswith("AgentAgent"):
        cls = cls[:-len("Agent")]
    root = pathlib.Path(parent) / name
    if root.exists():
        raise FileExistsError(f"{root} exists")
    pkg = root / module
    pkg.mkdir(parents=True)
    values = {"name": name, "module": module, "cls": cls, "title": name}
    (pkg / "agent.py").write_text(AGENT.format(**values))
    (pkg / "__init__.py").write_text(MAIN.format(**values))
    (pkg / "__main__.py").write_text(f"from . import main\n\nmain()\n")
    (root / "pyproject.toml").write_text(PYPROJECT.format(**values))
    (root / "agent.yaml").write_text(CONFIG.format(**values))
    (root / "README.md").write_text(README.format(**values))
    return root, f"{module}.agent:{cls}"
