#!/usr/bin/env python3
"""The pin check (roadmap 051): every component builds against the commits this workspace pins.

A component repository carries its own dependencies as submodules under `deps/`, so that it
builds on its own (decision 4). Here, where the same repositories are direct submodules, its
`deps/<repo>` pin must be the commit this repository pins for that repository: otherwise the
component is built and tested against other code than its neighbours. Repositories are
matched by URL. Reads the pins from git (the committed tree), not from checkouts.

    scripts/check_pins.py        exit 1 on a mismatch, naming component, dependency and both commits
"""

import configparser
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def submodules(repo, rev="HEAD"):
    """{url: (path, commit)} of `repo`'s submodules at `rev`."""
    try:
        text = subprocess.run(["git", "-C", str(repo), "show", f"{rev}:.gitmodules"],
                              capture_output=True, text=True, check=True).stdout
    except subprocess.CalledProcessError:
        return {}
    cfg = configparser.ConfigParser()
    cfg.read_string(text)
    out = {}
    for section in cfg.sections():
        path, url = cfg[section]["path"], cfg[section]["url"]
        tree = subprocess.run(["git", "-C", str(repo), "ls-tree", rev, path],
                              capture_output=True, text=True, check=True).stdout.split()
        if len(tree) >= 3 and tree[1] == "commit":
            out[url.removesuffix(".git").rstrip("/")] = (path, tree[2])
    return out


def main():
    ours = submodules(ROOT)
    bad, checked = [], 0
    for url, (path, commit) in sorted(ours.items()):
        component = ROOT / path
        if not (component / ".git").exists():
            continue
        for dep_url, (dep_path, dep_commit) in submodules(component, commit).items():
            if not dep_path.startswith("deps/"):
                continue
            checked += 1
            mine = ours.get(dep_url)
            if mine is None:
                bad.append(f"{path}/{dep_path}: {dep_url} is not a submodule here")
            elif mine[1] != dep_commit:
                bad.append(f"{path}/{dep_path} is {dep_commit[:10]}, but {mine[0]} here is "
                           f"{mine[1][:10]}: move one pin to the other")
    for line in bad:
        print(f"FAIL: {line}")
    print(f"pins: {checked} component dependencies checked, {len(bad)} mismatch(es)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
