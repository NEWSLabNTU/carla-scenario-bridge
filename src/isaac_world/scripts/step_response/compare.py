"""CARLA vs Isaac step responses: error after the first second of each transient.

    python3 compare.py OUT_DIR LABEL [LABEL...]
Writes OUT_DIR/compare_<label>.md and a PNG per test.
"""
import csv
import sys
from pathlib import Path

OUT = Path(sys.argv[1])
LABELS = sys.argv[2:]


def load(p):
    with open(p) as f:
        r = list(csv.DictReader(f))
    return {round(float(x["t"]), 3): (float(x["v"]), float(x["yaw_rate"])) for x in r}


# Windows: (test, quantity, t0, t1) -- each starts a second after the step it measures.
WINDOWS = [
    ("accel_brake", "speed, +1.5 m/s^2", 2.0, 6.0),
    ("accel_brake", "speed, -3 m/s^2", 7.0, 8.4),
    ("steer_step", "speed, 5 m/s held", 6.0, 12.0),
    ("steer_step", "yaw rate, 0.2 rad", 9.0, 12.0),
    ("steer_step_10", "speed, 10 m/s held", 8.0, 11.0),
    ("steer_step_10", "yaw rate, 0.1 rad", 9.0, 11.0),
]


def stats(c, i, t0, t1):
    ts = sorted(t for t in c if t0 - 1e-9 <= t <= t1 + 1e-9 and t in i)
    q = 1 if "yaw" in WHAT else 0
    cv = [c[t][q] for t in ts]
    iv = [i[t][q] for t in ts]
    ref = max(abs(x) for x in cv)
    worst = max(abs(a - b) for a, b in zip(cv, iv))
    mean_c, mean_i = sum(cv) / len(cv), sum(iv) / len(iv)
    return mean_c, mean_i, worst, worst / ref if ref else float("nan")


for label in LABELS:
    lines = [f"| test | quantity | window s | CARLA mean | Isaac mean | max abs diff | max diff / CARLA peak |",
             "|---|---|---|---|---|---|---|"]
    for test, WHAT, t0, t1 in WINDOWS:
        c = load(OUT / f"carla_{test}.csv")
        i = load(OUT / f"isaac_{label}_{test}.csv")
        mc, mi, w, rel = stats(c, i, t0, t1)
        unit = "rad/s" if "yaw" in WHAT else "m/s"
        lines.append(f"| {test} | {WHAT} | {t0:.1f}-{t1:.1f} | {mc:.3f} | {mi:.3f} | "
                     f"{w:.3f} {unit} | {100 * rel:.1f} % |")
    text = "\n".join(lines)
    (OUT / f"compare_{label}.md").write_text(text + "\n")
    print(f"## {label}\n{text}\n")

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for test in ("accel_brake", "steer_step", "steer_step_10"):
        fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        c = load(OUT / f"carla_{test}.csv")
        series = [("CARLA", c)] + [(f"Isaac ({l})", load(OUT / f"isaac_{l}_{test}.csv"))
                                   for l in LABELS]
        for name, d in series:
            ts = sorted(d)
            ax[0].plot(ts, [d[t][0] for t in ts], label=name)
            ax[1].plot(ts, [d[t][1] for t in ts], label=name)
        ax[0].set_ylabel("speed m/s")
        ax[1].set_ylabel("yaw rate rad/s")
        ax[1].set_xlabel("t s")
        ax[0].legend()
        ax[0].set_title(test)
        fig.tight_layout()
        fig.savefig(OUT / f"{test}.png", dpi=90)
except ImportError:
    print("no matplotlib: no plots")
