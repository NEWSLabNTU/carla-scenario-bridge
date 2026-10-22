"""CARLA's walkers on Isaac pedestrians (roadmap 044 C), pure Python and numpy so it is tested
without Isaac.

A pedestrian stays a kinematic box at the pose SSv2 sends; it wears a CARLA walker
(`isaac_vehicles.sh walkers:=true`, `<root>/walkers/walkers.json`): the skeletal mesh SimReady
exports, and its walk and idle cycles kept as arrays. The world plays them by speed, as
CARLA's walker animation blueprint does: each frame, `Gait.advance` takes the speed the
pedestrian moved at and returns which cycle and where in it, `Cycle.sample` that frame's
joint rotations and translations, which the world writes into the walker's own
SkelAnimation. Below `SPEED_FLOOR` the walker idles; walking, the cycle runs at
`speed / REFERENCE_SPEED` times its recorded rate (CARLA's walk cycles are in place, no root
motion, recorded at about a walking pace).
"""
import json
import zlib
from pathlib import Path

import numpy as np

# Below this the walker idles, m/s.
SPEED_FLOOR = 0.2
# The speed a walk cycle plays at its recorded rate, m/s (CARLA's walkers walk at 1.0-1.4).
REFERENCE_SPEED = 1.3
# The walk cycle's rate is held to this many times its recorded one.
MAX_RATE = 2.5
# The speed the gait sees is smoothed over about this long, s (a pose stream at 30 Hz with
# jitter would flicker between walking and idling otherwise).
SPEED_SMOOTHING_S = 0.2


class Cycle:
    """One looping cycle: `rotations` (frames x joints x 4, x y z w), `translations`
    (frames x joints x 3), `fps`."""

    def __init__(self, rotations, translations, fps):
        self.rotations = np.asarray(rotations, dtype=np.float32)
        self.translations = np.asarray(translations, dtype=np.float32)
        self.fps = float(fps)
        self.frames = len(self.rotations)
        # The last frame repeats the first in a looping export: one period is frames - 1.
        self.length = max(self.frames - 1, 1) / self.fps

    @classmethod
    def load(cls, npz):
        z = np.load(npz)
        return cls(z["rotations"], z["translations"], z["fps"])

    def sample(self, phase):
        """`(rotations, translations)` at `phase` (0..1, wrapped) of the cycle, linearly
        interpolated between the recorded frames (rotations normalised: nlerp, on the
        shorter arc)."""
        if self.frames == 1:
            return self.rotations[0], self.translations[0]
        i = (phase % 1.0) * (self.frames - 1)
        k = min(int(i), self.frames - 2)
        f = np.float32(i - k)
        a, b = self.rotations[k], self.rotations[k + 1]
        sign = np.where(np.sum(a * b, axis=1, keepdims=True) < 0, -1.0, 1.0).astype(np.float32)
        q = a * (1 - f) + b * sign * f
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        t = self.translations[k] * (1 - f) + self.translations[k + 1] * f
        return q, t


class Gait:
    """Which cycle a walker plays and where in it, from the speed it moves at."""

    def __init__(self, walk_length, idle_length):
        self.walk_length, self.idle_length = walk_length, idle_length
        self.speed = 0.0
        self.cycle = "idle"
        self.phase = 0.0

    def advance(self, speed, dt):
        """Move on by `dt` seconds at `speed` m/s; `(cycle name, phase)`."""
        if dt <= 0:
            return self.cycle, self.phase
        a = min(1.0, dt / SPEED_SMOOTHING_S)
        self.speed += a * (speed - self.speed)
        cycle = "walk" if self.speed >= SPEED_FLOOR else "idle"
        if cycle != self.cycle:
            self.cycle, self.phase = cycle, 0.0
        if cycle == "walk":
            rate = min(self.speed / REFERENCE_SPEED, MAX_RATE)
            self.phase += dt * rate / self.walk_length
        else:
            self.phase += dt / self.idle_length
        self.phase %= 1.0
        return self.cycle, self.phase


def blueprints(path=Path(__file__).with_name("walker_blueprints.json")):
    """CARLA's `walker.pedestrian.NNNN` -> its blueprint, gender and age, read from
    WalkerFactory's package (`scripts/walker_factory_ids.py`; 044, open item 3)."""
    return json.loads(Path(path).read_text())


BLUEPRINTS = blueprints()


def pick(walkers, model, name, table=BLUEPRINTS):
    """The walker a pedestrian wears: the one exported for `model` (an exported walker's id,
    `bp_walker_*`, or CARLA's `walker.pedestrian.NNNN` whose blueprint is exported); else one
    of the exported walkers of that id's gender and age, then of its age (a child stays a
    child); else, as for a pedestrian with no model (by `name`), one chosen stably and spread
    over the exported walkers."""
    if not walkers:
        return None
    by_id = {w["id"]: w for w in walkers}
    if model in by_id:
        return by_id[model]
    ordered = sorted(by_id)
    key = model or name or ""
    digits = "".join(c for c in key.rsplit(".", 1)[-1] if c.isdigit())
    n = int(digits) if model and digits else zlib.crc32(key.encode())
    wanted = table.get(model)
    if wanted is not None:
        own = wanted["blueprint"].lower()
        if own in by_id:
            return by_id[own]
        kinds = {e["blueprint"].lower(): (e["gender"], e["age"]) for e in table.values()}
        for same in (lambda k: k == (wanted["gender"], wanted["age"]),
                     lambda k: k[1] == wanted["age"]):
            alike = [i for i in ordered if i in kinds and same(kinds[i])]
            if alike:
                return by_id[alike[n % len(alike)]]
    return by_id[ordered[n % len(ordered)]]


def horizontal_speed(previous, position, dt):
    """Speed in the ground plane between two positions `dt` apart, m/s."""
    if previous is None or dt <= 0:
        return 0.0
    return float(np.hypot(position[0] - previous[0], position[1] - previous[1]) / dt)
