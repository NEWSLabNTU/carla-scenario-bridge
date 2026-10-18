#!/usr/bin/env python3
"""acb's acceptance checks on any backend, scored from one bag of the ego's domain (roadmap 043).

acb's `acceptance.py` judges a run with a CARLA client (is there an ego, CARLA's health) and a
fresh ROS observer per run, which on a domain of ~200 Autoware participants is discovered
tens of seconds into the run (038, point 11). Neither works on Isaac Sim. This keeps its
judgement -- the scenario's verdict, the ego drove and got somewhere, no node died, and
acceptance.py's own tracking formulas and limits -- but records the ego's domain once across
the batch (`tracking_probe.py`'s topics, plus the MRM state: 043 wants none) and scores each
run from the message stamps afterwards. The stacks must be up or startable: each run is
`just scenario <xosc> backend:=<backend>`.

    acceptance_bag.py --scenario x.xosc [--runs 3] [--backend isaac] [--worlds Town01,Town02]
                      [--domain 1] [--out DIR] [--json FILE]

Exit status is 0 only if every run passed every check.
"""
import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PROJECT / "src/autoware_carla_bridge/scripts"))

import acceptance  # noqa: E402  (acb's harness: limits, dead_nodes)
import tracking_probe  # noqa: E402


def junit_verdict(path):
    if not path.exists():
        return False, "no result.junit.xml: the scenario never reported"
    text = path.read_text()
    if 'failures="0"' in text and 'errors="0"' in text:
        return True, "scenario passed"
    return False, "scenario reported a failure or error"


def start_bag(out, domain, discovery):
    """`ros2 bag record` of the scored topics, waited for until it has subscribed to the
    ones that are published (or `discovery` s pass): a recorder that joins late misses a
    run's start, which is the whole reason for recording once."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    log = open(out / "bag.log", "w")
    proc = subprocess.Popen(["ros2", "bag", "record", "-o", str(out / "bag"),
                             *tracking_probe.TOPICS],
                            stdout=log, stderr=subprocess.STDOUT, env=env,
                            start_new_session=True)
    deadline = time.time() + discovery
    want = len(tracking_probe.TOPICS)
    while time.time() < deadline:
        n = (out / "bag.log").read_text(errors="replace").count("Subscribed to topic")
        if n >= want:
            break
        if proc.poll() is not None:
            raise SystemExit(f"ros2 bag record exited; see {out / 'bag.log'}")
        time.sleep(2)
    n = (out / "bag.log").read_text(errors="replace").count("Subscribed to topic")
    print(f"  recorder subscribed to {n}/{want} topics", flush=True)
    return proc


def stop(proc, sig=signal.SIGINT):
    """Signal `proc`'s process group (its own session) and reap it; SIGKILL after 30 s."""
    try:
        os.killpg(proc.pid, sig)  # the bag closes its storage on SIGINT
        proc.wait(timeout=30)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def judge(score):
    why = []
    if score is None:
        return ["no vehicle status recorded for this run"]
    if score["peak_speed"] < acceptance.MIN_PEAK_SPEED:
        why.append("ego never drove (peak %.2f m/s)" % score["peak_speed"])
    if score["distance"] < acceptance.MIN_DISTANCE:
        why.append("ego went nowhere (%.1f m)" % score["distance"])
    lon = score["longitudinal_median_error"]
    if lon is not None and lon > acceptance.MAX_LONGITUDINAL_ERROR:
        why.append("longitudinal tracking %.3f m/s^2 above the %.2f limit"
                   % (lon, acceptance.MAX_LONGITUDINAL_ERROR))
    xt = score["cross_track_median"]
    if xt is not None and xt > acceptance.MAX_CROSS_TRACK:
        why.append("cross-track %.3f m above the %.2f limit" % (xt, acceptance.MAX_CROSS_TRACK))
    gap = score["localization_gap_median"]
    if gap is not None and gap > acceptance.MAX_LOCALIZATION_GAP:
        why.append("localization is %.1f m from GNSS (median, limit %.1f)"
                   % (gap, acceptance.MAX_LOCALIZATION_GAP))
    if score["mrm_entries"]:
        why.append("%d MRM entr%s" % (score["mrm_entries"],
                                      "y" if score["mrm_entries"] == 1 else "ies"))
    return why


def fmt(v, f):
    return f % v if v is not None else "n/a"


def on_term(signum, _frame):
    raise KeyboardInterrupt(f"signal {signum}")


def main() -> int:
    # Stopped from outside (SIGTERM, a timeout): the scenario stack and the recorder run in
    # sessions of their own, so they are taken down here rather than left behind.
    signal.signal(signal.SIGTERM, on_term)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--backend", default="isaac")
    ap.add_argument("--worlds", default="", help="passed on as `just scenario worlds:=`")
    ap.add_argument("--domain", type=int, default=1, help="the ego's ROS domain")
    ap.add_argument("--out", help="run outputs and the bag (default play_log/acceptance/<time>)")
    ap.add_argument("--play-log", default=str(PROJECT / "play_log/ego"))
    ap.add_argument("--discovery", type=float, default=120.0,
                    help="s to wait for the recorder to subscribe")
    ap.add_argument("--run-timeout", type=float, default=900.0,
                    help="s one `just scenario` may take, bring-up included")
    ap.add_argument("--json", help="write the full result here")
    args = ap.parse_args()

    out = Path(args.out or PROJECT / "play_log/acceptance"
               / datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scenario = str(Path(args.scenario).resolve())
    extra = [f"backend:={args.backend}"] + ([f"worlds:={args.worlds}"] if args.worlds else [])
    print(f"=== {Path(scenario).name} x{args.runs} on {args.backend}, ego domain {args.domain};"
          f" output in {out}", flush=True)

    # Bring the stacks up (and wait for them) before the recorder joins, through the first
    # run's own bring-up path: `just scenario` of the Isaac backend starts what is missing.
    if args.backend == "isaac":
        maps = os.environ.get("CARLA_MAPS", str(PROJECT / "data/carla-autoware-bridge"))
        map_dir = subprocess.run(["just", "_scenario-map", scenario], capture_output=True,
                                 text=True, env=dict(os.environ, CARLA_MAPS=maps),
                                 cwd=PROJECT).stdout.strip()
        worlds = ",".join(w if "/" in w else f"{maps}/{w}"
                          for w in args.worlds.split(",") if w) or map_dir
        up = subprocess.run([str(HERE / "isaac_up.sh"), map_dir, worlds], cwd=PROJECT)
        if up.returncode != 0:
            print("  the Isaac stacks did not come up")
            return 1
    bag = start_bag(out, args.domain, args.discovery)

    verdicts = []
    try:
        for run in range(1, args.runs + 1):
            run_dir = out / f"run{run}"
            run_dir.mkdir(exist_ok=True)
            print(f"--- run {run}/{args.runs}", flush=True)
            started = time.time()
            with open(run_dir / "scenario.log", "w") as log:
                child = subprocess.Popen(["just", "scenario", scenario, *extra], cwd=PROJECT,
                                         stdout=log, stderr=subprocess.STDOUT,
                                         env=dict(os.environ, OUTPUT_DIR=str(run_dir)),
                                         start_new_session=True)
                try:
                    child.wait(timeout=args.run_timeout)
                except subprocess.TimeoutExpired:
                    print(f"    `just scenario` outlasted {args.run_timeout:.0f} s")
                    stop(child, signal.SIGTERM)
                except BaseException:
                    stop(child, signal.SIGTERM)  # the scenario stack goes with this harness
                    raise
            ok, why = junit_verdict(run_dir / "scenario_test_runner/result.junit.xml")
            dead = acceptance.dead_nodes(Path(args.play_log))
            verdicts.append({"run": run, "verdict": ok, "verdict_why": why, "dead": dead,
                             "wall_s": round(time.time() - started, 1)})
            print(f"    {why} in {time.time() - started:.0f} s", flush=True)
            time.sleep(10)  # as acceptance.py: let the last run's ego go before the next
    finally:
        stop(bag)

    msgs, arrivals = tracking_probe.read_bag(out / "bag")
    windows = tracking_probe.split_runs(arrivals, 3.0)
    scores = [tracking_probe.score(msgs, a, b, acceptance) for a, b in windows]
    if len(scores) != len(verdicts):
        print(f"  note: {len(scores)} driving stretch(es) in the bag for {len(verdicts)} run(s);"
              " paired in order")
    results = []
    for i, v in enumerate(verdicts):
        s = scores[i] if i < len(scores) else None
        why = ([] if v["verdict"] else [v["verdict_why"]]) + judge(s) \
            + ["node died -- %s" % d for d in v["dead"]]
        results.append({**v, "ok": not why, "why": why, "score": s})
        print("  run %d: %s   peak %s m/s   travelled %s m   longitudinal %s m/s^2   "
              "cross-track %s m   localization %s m   MRM %s"
              % (v["run"], "PASS" if not why else "FAIL",
                 fmt(s and s["peak_speed"], "%.2f"), fmt(s and s["distance"], "%.1f"),
                 fmt(s and s["longitudinal_median_error"], "%.3f"),
                 fmt(s and s["cross_track_median"], "%.2f"),
                 fmt(s and s["localization_gap_median"], "%.2f"),
                 fmt(s and s["mrm_entries"], "%d")))
        for w in why:
            print("    - %s" % w)
    passed = sum(r["ok"] for r in results)
    print(f"\n{passed}/{len(results)} runs passed")
    doc = {"scenario": scenario, "backend": args.backend, "runs": results,
           "limits": {"longitudinal": acceptance.MAX_LONGITUDINAL_ERROR,
                      "cross_track": acceptance.MAX_CROSS_TRACK,
                      "localization_gap": acceptance.MAX_LOCALIZATION_GAP}}
    (out / "acceptance.json").write_text(json.dumps(doc, indent=1) + "\n")
    if args.json:
        Path(args.json).write_text(json.dumps(doc, indent=1) + "\n")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
