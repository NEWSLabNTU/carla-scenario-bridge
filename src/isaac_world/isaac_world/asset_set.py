"""The Isaac asset set's manifest: write it, check a copy against it
(docs/design/isaac-assets.md; `just isaac-assets`). Plain Python, no Isaac.

    python3 -m isaac_world.asset_set manifest <root>   # README.md and MANIFEST.sha256
    python3 -m isaac_world.asset_set check <root>      # exit 1 on a missing or changed file

MANIFEST.sha256 is `sha256sum`'s format (`sha256sum -c` reads it too), paths relative to the
root. Skipped: `logs/` and `raw/` directories, `*.pre<N>` backups, the manifest itself.
"""
import argparse
import datetime
import fnmatch
import hashlib
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

MANIFEST = "MANIFEST.sha256"
SKIPPED_DIRS = {"logs", "raw"}
SKIPPED_FILES = ("*.pre[0-9]*", MANIFEST)


def files(root):
    """The set's files, relative to `root`, sorted."""
    root = Path(root)
    out = []
    for d, dirs, names in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x not in SKIPPED_DIRS)
        for n in names:
            if not any(fnmatch.fnmatch(n, p) for p in SKIPPED_FILES):
                out.append(str((Path(d) / n).relative_to(root)))
    return sorted(out)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def digests(root, names, jobs=4):
    """`{name: sha256}` for `names` under `root`."""
    with ThreadPoolExecutor(jobs) as pool:
        return dict(zip(names, pool.map(lambda n: sha256(Path(root) / n), names)))


def read_manifest(root):
    out = {}
    for line in (Path(root) / MANIFEST).read_text().splitlines():
        if line.strip():
            digest, name = line.split("  ", 1)
            out[name] = digest
    return out


def readme(root, swb_commit):
    """README.md: what the set is, from which versions, and what it holds."""
    root = Path(root)

    def count(index):
        p = root / index
        return len(json.loads(p.read_text())) if p.exists() else 0

    towns = sorted(d.name for d in root.iterdir()
                   if d.is_dir() and (d / f"{d.name}_physics.usda").exists())
    return f"""# CARLA 0.9.16 assets for Isaac Sim

CARLA 0.9.16's content exported from its UE 4.26 editor with NVIDIA's SimReady converter and
wrapped for scenario-world-bridge's Isaac backend. Layout, use and how it is made:
`docs/design/isaac-assets.md` in scenario-world-bridge. CARLA's assets are CC-BY 4.0
(carla-simulator/carla's content licence).

- Manifest written: {datetime.date.today().isoformat()}, scenario-world-bridge {swb_commit}
- Towns: {', '.join(towns) or 'none'}
- Vehicles: {count('vehicles/vehicles.json')} (`vehicles/vehicles.json`)
- Props: {count('vehicles/props/props.json')} (`vehicles/props/props.json`)
- Walkers: {count('vehicles/walkers/walkers.json')} (`vehicles/walkers/walkers.json`)
- Rider pose: {'yes' if (root / 'vehicles/riders/grip.npz').exists() else 'no'}

Use: set `CARLA_USD` to this directory and `SWB_VEHICLE_USD` to its `vehicles/`, then
`just isaac-assets check`.
"""


def swb_commit():
    try:
        return subprocess.run(["git", "-C", str(Path(__file__).parent), "describe", "--always",
                               "--dirty"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def manifest(root, commit=None):
    """Write README.md and MANIFEST.sha256 in `root`; the number of files listed."""
    root = Path(root)
    (root / "README.md").write_text(readme(root, commit or swb_commit()))
    names = files(root)
    sums = digests(root, names)
    (root / MANIFEST).write_text("".join(f"{sums[n]}  {n}\n" for n in names))
    return len(names)


def check(root):
    """`(missing, changed, extra)`: listed files absent, listed files whose checksum differs,
    files not listed."""
    root = Path(root)
    listed = read_manifest(root)
    present = set(files(root))
    missing = sorted(n for n in listed if n not in present)
    here = sorted(n for n in listed if n in present)
    sums = digests(root, here)
    changed = [n for n in here if sums[n] != listed[n]]
    extra = sorted(present - set(listed))
    return missing, changed, extra


def main(argv=None):
    ap = argparse.ArgumentParser(description="The Isaac asset set's manifest.")
    ap.add_argument("action", choices=("manifest", "check"))
    ap.add_argument("root", help="the asset root (CARLA_USD)")
    args = ap.parse_args(argv)
    root = Path(args.root)
    if not root.is_dir():
        sys.exit(f"{root}: no such directory")
    if args.action == "manifest":
        n = manifest(root)
        print(f"{root / MANIFEST}: {n} files; {root / 'README.md'}")
        return 0
    if not (root / MANIFEST).exists():
        sys.exit(f"{root}: no {MANIFEST} (a set is made with `manifest`)")
    missing, changed, extra = check(root)
    for n in missing:
        print("missing:", n)
    for n in changed:
        print("changed:", n)
    for n in extra:
        print("not in the manifest:", n)
    listed = len(read_manifest(root))
    print(f"{root}: {listed - len(missing) - len(changed)} of {listed} files as listed, "
          f"{len(missing)} missing, {len(changed)} changed, {len(extra)} not listed")
    return 1 if missing or changed else 0


if __name__ == "__main__":
    sys.exit(main())
