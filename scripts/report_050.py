#!/usr/bin/env python3
"""Roadmap 050's before/after report from `scripts/run_050_set.sh` output.

    scripts/report_050.py play_log/050/before [play_log/050/after] [--bridge-log DIR ...]
        [--json out.json] [--markdown out.md]

Per scenario run (runs.tsv), from the gap traces the bridge wrote during it:

- the verdict;
- per ego pair (ego and each other entity): the minimum box-to-box distance in SSv2's
  geometry (declared boxes at SSv2's poses) and in CARLA's (blueprint boxes at CARLA's
  transforms), and where SSv2's was smallest;
- the ego's CARLA contacts (by other entity);
- the gap oracle's counts (G1 ego/NPC, G2, G3) and, per G1 ego finding, the pair and the
  maximum penetration;
- the ego's distance driven;
- with --bridge-log (the bridge's play_log run directory, one per label), the frame-budget
  summary the bridge logged for the run.

With two labels, prints both side by side and the difference of each number.
"""

import argparse
import datetime
import json
import math
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "src/swb_launch/scripts"
sys.path.insert(0, str(SCRIPTS))
import gap_geometry as gg  # noqa: E402
import gap_trace  # noqa: E402


def runs_of(label_dir):
    rows = []
    for line in (label_dir / "runs.tsv").read_text().splitlines():
        name, start, end, code, verdict = line.split("\t")
        rows.append({"name": name, "start": float(start), "end": float(end),
                     "exit": int(code), "verdict": verdict})
    return rows


def traces_in(label_dir, start, end):
    out = []
    for path in sorted((label_dir / "traces").glob("*.jsonl")):
        try:
            with path.open() as f:
                first = json.loads(f.readline())
        except (OSError, ValueError):
            continue
        t = first.get("start_unix")
        if t is not None and start - 5 <= t <= end + 5:
            out.append(path)
    return out


def pose2d(p):
    return gg.Pose2D(p.x, p.y, p.yaw)


def pair_gaps(trace):
    """{other: {ssv2_min, ssv2_at, carla_min}} over every frame both are present."""
    headers = {}
    pending = list(trace.headers)
    egos = [h.name for h in trace.headers if h.kind == "ego"]
    gaps = {}
    driven, last = 0.0, None
    for frame in trace.frames:
        while pending and pending[0].frame <= frame.frame:
            h = pending.pop(0)
            headers[h.name] = h
        for ego in egos:
            es = frame.entities.get(ego)
            eh = headers.get(ego)
            if es is None or eh is None:
                continue
            if es.carla is not None:
                xy = (es.carla.x, es.carla.y)
                if last is not None:
                    driven += math.dist(xy, last)
                last = xy
            ebox_s = gg.ssv2_box(pose2d(es.ssv2), eh.declared.center, eh.declared.dimensions)
            ebox_c = None
            if es.carla is not None and eh.blueprint_box is not None:
                ebox_c = gg.carla_box(pose2d(es.carla), eh.blueprint_box.location,
                                      eh.blueprint_box.extent)
            for name, st in frame.entities.items():
                if name == ego or name not in headers:
                    continue
                h = headers[name]
                g = gaps.setdefault(name, {"ssv2_min": None, "ssv2_at": None,
                                           "carla_min": None, "kind": h.kind,
                                           "blueprint": h.blueprint})
                if all(d > 0 for d in h.declared.dimensions[:2]):
                    d = gg.signed_distance(
                        ebox_s, gg.ssv2_box(pose2d(st.ssv2), h.declared.center,
                                            h.declared.dimensions))
                    if g["ssv2_min"] is None or d < g["ssv2_min"]:
                        g["ssv2_min"], g["ssv2_at"] = d, round(frame.sim_time, 2)
                if ebox_c is not None and st.carla is not None and h.blueprint_box is not None:
                    d = gg.signed_distance(
                        ebox_c, gg.carla_box(pose2d(st.carla), h.blueprint_box.location,
                                             h.blueprint_box.extent))
                    if g["carla_min"] is None or d < g["carla_min"]:
                        g["carla_min"] = d
    return gaps, driven


def oracle(path):
    out = path.with_suffix(".oracle.json")
    if not out.exists():
        subprocess.run([sys.executable, str(SCRIPTS / "gap_oracle"), str(path), "--out", str(out)],
                       check=False, capture_output=True)
    try:
        return json.loads(out.read_text())
    except (OSError, ValueError):
        return None


BUDGET = re.compile(r"^(\S+Z)\s.*Frame budget, whole run: (.*)$")


def budgets(bridge_dirs):
    rows = []
    for d in bridge_dirs:
        for log in Path(d).glob("node/scenario_world_bridge/*"):
            if not log.is_file():
                continue
            for line in log.read_text(errors="replace").splitlines():
                m = BUDGET.match(re.sub(r"\x1b\[[0-9;]*m", "", line))
                if m:
                    t = datetime.datetime.fromisoformat(m.group(1).replace("Z", "+00:00"))
                    rows.append((t.timestamp(), m.group(2)))
    return sorted(rows)


def summarise(label_dir, bridge_dirs):
    budget_rows = budgets(bridge_dirs)
    result = {}
    for run in runs_of(label_dir):
        entry = {"verdict": run["verdict"], "exit": run["exit"],
                 "minutes": round((run["end"] - run["start"]) / 60, 1),
                 "pairs": {}, "contacts": {}, "oracle": {}, "g1_ego": [], "driven_m": 0.0,
                 "traces": 0}
        for path in traces_in(label_dir, run["start"], run["end"]):
            try:
                trace = gap_trace.load_trace(path, require_end=False)
            except gap_trace.TraceError as e:
                entry.setdefault("errors", []).append(str(e))
                continue
            if not trace.frames:
                continue
            entry["traces"] += 1
            gaps, driven = pair_gaps(trace)
            entry["driven_m"] += driven
            for name, g in gaps.items():
                old = entry["pairs"].get(name)
                if old is None or (g["ssv2_min"] is not None and
                                   (old["ssv2_min"] is None or g["ssv2_min"] < old["ssv2_min"])):
                    entry["pairs"][name] = g
            for c in trace.contacts:
                if c.ego_side:
                    key = c.name or c.other_type
                    entry["contacts"][key] = entry["contacts"].get(key, 0) + 1
            o = oracle(path)
            if o:
                for k, v in o["counts"].items():
                    entry["oracle"][k] = entry["oracle"].get(k, 0) + v
                for f in o["findings"]:
                    if f["check"] == "G1" and f["priority"] == "high":
                        entry["g1_ego"].append({"entities": f["entities"],
                                                "numbers": f.get("numbers", {})})
        # The bridge logs a run's frame budget at the next Initialize (the next run's start)
        # or at shutdown: the first one after the run ended is this run's.
        after = [b for t, b in budget_rows if t > run["end"]]
        if after:
            entry["frame_budget"] = after[0]
        entry["driven_m"] = round(entry["driven_m"], 1)
        for g in entry["pairs"].values():
            for k in ("ssv2_min", "carla_min"):
                if g[k] is not None:
                    g[k] = round(g[k], 3)
        result[run["name"]] = entry
    return result


def fmt(v):
    return "-" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v))


def markdown(results, labels):
    lines = []
    names = list(dict.fromkeys(n for r in results for n in r))
    for name in names:
        rs = [r.get(name) for r in results]
        lines.append(f"### {name}")
        lines.append("")
        lines.append("| | " + " | ".join(labels) + " |")
        lines.append("|---|" + "---|" * len(labels))
        lines.append("| verdict | " + " | ".join(fmt(r and r["verdict"]) for r in rs) + " |")
        lines.append("| minutes | " + " | ".join(fmt(r and r["minutes"]) for r in rs) + " |")
        lines.append("| ego driven, m | " + " | ".join(fmt(r and r["driven_m"]) for r in rs) + " |")
        lines.append("| ego contacts | " + " | ".join(
            (", ".join(f"{k} x{v}" for k, v in r["contacts"].items()) or "none") if r else "-"
            for r in rs) + " |")
        lines.append("| oracle G1 ego / NPC, G2, G3 | " + " | ".join(
            (f"{r['oracle'].get('G1_ego', 0)} / {r['oracle'].get('G1_npc', 0)}, "
             f"{r['oracle'].get('G2', 0)}, {r['oracle'].get('G3', 0)}") if r else "-"
            for r in rs) + " |")
        if any(r and r.get("frame_budget") for r in rs):
            lines.append("| frame budget | " + " | ".join(
                fmt(r and r.get("frame_budget")) for r in rs) + " |")
        others = list(dict.fromkeys(o for r in rs if r for o in r["pairs"]))
        for o in others:
            cells = []
            for r in rs:
                g = r and r["pairs"].get(o)
                cells.append("-" if not g else
                             f"{fmt(g['ssv2_min'])} / {fmt(g['carla_min'])}")
            lines.append(f"| min gap to {o}, SSv2 / CARLA, m | " + " | ".join(cells) + " |")
        lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("labels", nargs="+", type=Path)
    ap.add_argument("--bridge-log", nargs="*", default=[], type=Path,
                    help="one bridge play_log run directory per label, in order")
    ap.add_argument("--json", type=Path)
    ap.add_argument("--markdown", type=Path)
    args = ap.parse_args()
    results = []
    for i, label in enumerate(args.labels):
        bridge = [args.bridge_log[i]] if i < len(args.bridge_log) else []
        results.append(summarise(label, bridge))
    text = markdown(results, [l.name for l in args.labels])
    print(text)
    if args.markdown:
        args.markdown.write_text(text + "\n")
    if args.json:
        args.json.write_text(json.dumps(dict(zip([l.name for l in args.labels], results)),
                                        indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
