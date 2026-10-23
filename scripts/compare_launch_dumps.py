#!/usr/bin/env python3
"""Compare two sets of resolved launches (roadmap 051, K1): same nodes, same parameters, but for
install paths and the names 051 renames.

    scripts/compare_launch_dumps.py <before dir> <after dir>    # dirs of scripts/dump_launches.sh

For every stack both sets resolved (`<name>.yaml`, play_launch's SystemModel), the nodes are
matched by name and compared on package, executable, parameters, remappings, respawn and
environment, after:

- install paths are reduced to `<install>/<package>/...` (this workspace's, acb's or aib's own
  install, wherever it lives);
- 051's renames are applied to the before side: the world servers (carla_world ->
  swb_carla_world, isaac_world -> swb_isaac_world), the profile (acb_launch's config, scripts
  and files -> autoware_sim_profile's; carla_* switches -> sim_*), the agent (acb_pilot agent,
  node acb_agent -> swb_agent vehicle_agent, node vehicle_agent).

Prints each stack's node count and every remaining difference; exit 1 if there is one.
"""

import json
import re
import sys
from pathlib import Path

import yaml

RENAMES = [
    (r"\bacb_launch/config/planning/preset/carla_preset\.yaml", "autoware_sim_profile/config/planning/preset/sim_preset.yaml"),
    (r"autoware-carla\.yaml", "autoware-sim.yaml"),
    (r"<install>/acb_launch/(share|lib)/acb_launch/(config|planning_common_param|launch/components)",
     r"<install>/autoware_sim_profile/\1/autoware_sim_profile/\2"),
    (r"\bacb_agent\b", "vehicle_agent"),
    (r"(?<!swb_)\bcarla_world\b", "swb_carla_world"),
    (r"(?<!swb_)\bisaac_world_server\b", "swb_isaac_world_server"),
    (r"(?<!swb_)\bisaac_world\b", "swb_isaac_world"),
]
INSTALL = re.compile(r"/[^\s\"']*?/install/")


def normalise(text, before):
    text = INSTALL.sub("<install>/", text)
    text = text.replace("/swb_launch/lib/swb_launch/swb_isaac_world_server",
                        "/swb_isaac_world/lib/swb_isaac_world/swb_isaac_world_server")
    if before:
        for pattern, repl in RENAMES:
            text = re.sub(pattern, repl, text)
        text = text.replace("/swb_launch/lib/swb_launch/swb_isaac_world_server",
                            "/swb_isaac_world/lib/swb_isaac_world/swb_isaac_world_server")
    return text


FIELDS = ("pkg", "exec", "params", "remaps", "remappings", "respawn", "env", "args",
          "namespace", "container")


def nodes(path, before):
    model = yaml.safe_load(Path(path).read_text())
    out = {}
    for name, node in (model.get("structure", {}).get("nodes") or {}).items():
        record = {k: node.get(k) for k in FIELDS if k in node}
        if before and record.get("pkg") == "acb_pilot" and record.get("exec") == "agent":
            record["pkg"], record["exec"] = "swb_agent", "vehicle_agent"
        if before and record.get("pkg") == "swb_launch" and record.get("exec") == "isaac_world_server":
            record["pkg"], record["exec"] = "swb_isaac_world", "swb_isaac_world_server"
        key = normalise(name, before)
        out[key] = json.loads(normalise(json.dumps(record, sort_keys=True), before))
    return out


def diff(a, b, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a:
                yield f"{path}.{k}: added {json.dumps(b[k])[:160]}"
            elif k not in b:
                yield f"{path}.{k}: removed {json.dumps(a[k])[:160]}"
            else:
                yield from diff(a[k], b[k], f"{path}.{k}")
    elif a != b:
        yield f"{path}: {json.dumps(a)[:160]} -> {json.dumps(b)[:160]}"


def main():
    before, after = Path(sys.argv[1]), Path(sys.argv[2])
    total = 0
    for b in sorted(before.glob("*.yaml")):
        a = after / b.name
        if not a.exists():
            print(f"{b.stem}: not resolved after")
            total += 1
            continue
        nb, na = nodes(b, True), nodes(a, False)
        lines = []
        for name in sorted(set(nb) | set(na)):
            if name not in na:
                lines.append(f"  node {name}: removed")
            elif name not in nb:
                lines.append(f"  node {name}: added")
            else:
                lines += [f"  node {name}{d}" for d in diff(nb[name], na[name])]
        print(f"{b.stem}: {len(nb)} -> {len(na)} nodes, {len(lines)} difference(s)")
        for line in lines:
            print(line)
        total += len(lines)
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
