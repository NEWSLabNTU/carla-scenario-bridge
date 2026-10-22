#!/usr/bin/env python3
"""Move old play_launch runs out of the log root into an archive. Never deletes.

    prune_logs.py --root <log root> --dest <archive> [--days 7] [--dry-run]

A run is an entry of a kind directory, <root>/<kind>/<entry> (play_launch's
<time>/ directories, and the loose logs some recipes keep there), or a <time>/ directory
directly under the root (play_launch's layout before --log-dir named a kind). It moves to
<dest>/<kind>/<entry> (<dest>/<entry> for a root-level one) when nothing in it changed for
--days days. Skipped, and said so: a kind's `latest` and what it points to, any symlink, a
run some process still has a file or its working directory open in (a live play_launch
holds its play_launch.log), and a run whose name the archive already has (nothing is
overwritten). Files directly in the root (the *.console files) stay.

`just prune-logs` runs this with the justfile's SWB_LOG_ROOT and SWB_LOG_ARCHIVE.
"""
import argparse
import os
import re
import shutil
import sys
import time
from pathlib import Path

RUN_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}")


def newest_mtime(p: Path) -> float:
    """The newest mtime of p or anything under it (symlinks not followed)."""
    newest = p.lstat().st_mtime
    if p.is_dir() and not p.is_symlink():
        for dirpath, dirnames, filenames in os.walk(p):
            for n in dirnames + filenames:
                try:
                    newest = max(newest, os.lstat(os.path.join(dirpath, n)).st_mtime)
                except OSError:
                    pass
    return newest


def size(p: Path) -> int:
    if not p.is_dir():
        return p.lstat().st_size
    total = 0
    for dirpath, _, filenames in os.walk(p):
        for n in filenames:
            try:
                total += os.lstat(os.path.join(dirpath, n)).st_size
            except OSError:
                pass
    return total


def open_paths() -> set[str]:
    """Every file and working directory a process we can see has open."""
    paths = set()
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        base = f"/proc/{pid}"
        try:
            paths.add(os.readlink(f"{base}/cwd"))
        except OSError:
            pass
        try:
            fds = os.listdir(f"{base}/fd")
        except OSError:
            continue
        for fd in fds:
            try:
                paths.add(os.readlink(f"{base}/fd/{fd}"))
            except OSError:
                pass
    return paths


def in_use(run: Path, opened: set[str]) -> bool:
    prefix = str(run.resolve())
    return any(p == prefix or p.startswith(prefix + "/") for p in opened)


def candidates(root: Path):
    """(kind, entry) for each run under root; kind '' for a root-level <time>/."""
    for e in sorted(root.iterdir()):
        if e.is_symlink():
            # A kind linked elsewhere (SWB_LOG_ROOT moved) is pruned where it lives.
            continue
        if e.is_dir() and RUN_NAME.match(e.name):
            yield "", e, set()
        elif e.is_dir():
            keep = set()
            latest = e / "latest"
            if latest.is_symlink():
                keep.add((e / os.readlink(latest)).resolve())
            for r in sorted(e.iterdir()):
                yield e.name, r, keep


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", required=True, type=Path)
    ap.add_argument("--dest", required=True, type=Path)
    ap.add_argument("--days", type=float, default=7.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    root, dest = args.root.resolve(), args.dest.resolve()
    if not root.is_dir():
        sys.exit(f"prune-logs: no log root {root}")
    if dest == root or root in dest.parents:
        sys.exit(f"prune-logs: the archive {dest} is inside the log root {root}")
    cutoff = time.time() - args.days * 86400
    opened = open_paths()
    verb = "would move" if args.dry_run else "moved"
    moved = skipped = 0
    moved_bytes = 0
    for kind, run, keep in candidates(root):
        label = f"{kind}/{run.name}" if kind else run.name
        if run.name == "latest" or run.is_symlink():
            continue
        if newest_mtime(run) > cutoff:
            continue
        if run.resolve() in keep:
            print(f"  keep   {label} (latest)")
            skipped += 1
            continue
        if in_use(run, opened):
            print(f"  keep   {label} (open by a running process)")
            skipped += 1
            continue
        target = dest / kind / run.name if kind else dest / run.name
        if target.exists() or target.is_symlink():
            print(f"  keep   {label} (the archive already has {target})")
            skipped += 1
            continue
        n = size(run)
        if not args.dry_run:
            target.parent.mkdir(parents=True, exist_ok=True)
            try:
                # Across filesystems: a full copy, then the source goes. A failed copy raises
                # before anything is removed.
                shutil.move(str(run), str(target))
            except OSError as e:
                print(f"  FAIL   {label}: {e}", file=sys.stderr)
                return 1
        print(f"  {'move' if args.dry_run else 'moved'}  {label} ({n / 1e6:.1f} MB)")
        moved += 1
        moved_bytes += n
    print(f"prune-logs: {verb} {moved} run(s), {moved_bytes / 1e9:.2f} GB, older than "
          f"{args.days:g} days, {root} -> {dest}; {skipped} kept")
    return 0


if __name__ == "__main__":
    sys.exit(main())
