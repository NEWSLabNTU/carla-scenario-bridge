"""csb_isaac.walkers: cycles sampled, gait by speed, a walker picked (044 C)."""
import numpy as np
import pytest

from csb_isaac import walkers


def cycle(frames=5, fps=4.0):
    # One joint turning about z from 0 to 90 degrees over the cycle, moving up in z.
    angles = np.linspace(0, np.pi / 2, frames)
    rots = np.stack([[[0, 0, np.sin(a / 2), np.cos(a / 2)]] for a in angles])
    trs = np.stack([[[0, 0, float(i)]] for i in range(frames)])
    return walkers.Cycle(rots, trs, fps)


def test_a_cycle_loops_over_frames_minus_one():
    c = cycle()
    assert c.length == pytest.approx(1.0)  # 4 periods at 4 fps
    q0, t0 = c.sample(0.0)
    q1, t1 = c.sample(1.0)
    assert np.allclose(q0, q1) and np.allclose(t0, t1)


def test_a_cycle_interpolates_between_frames():
    c = cycle()
    q, t = c.sample(0.125)  # half way between frames 0 and 1
    assert t[0, 2] == pytest.approx(0.5)
    assert np.linalg.norm(q[0]) == pytest.approx(1.0)
    angle = 2 * np.arctan2(q[0, 2], q[0, 3])
    assert angle == pytest.approx(np.pi / 16, abs=1e-3)


def test_interpolation_takes_the_shorter_arc():
    rots = np.array([[[0, 0, 0, 1.0]], [[0, 0, 0, -1.0]]])  # the same rotation
    c = walkers.Cycle(rots, np.zeros((2, 1, 3)), 1.0)
    q, _ = c.sample(0.5)
    assert abs(q[0, 3]) == pytest.approx(1.0)


def test_the_gait_idles_when_still_and_walks_when_moving():
    g = walkers.Gait(walk_length=1.0, idle_length=10.0)
    assert g.advance(0.0, 0.1)[0] == "idle"
    for _ in range(20):
        name, _ = g.advance(1.3, 0.05)
    assert name == "walk"
    # At the reference speed the walk plays at its recorded rate: one cycle per second.
    p0 = g.phase
    g.advance(walkers.REFERENCE_SPEED, 0.25)
    assert (g.phase - p0) % 1.0 == pytest.approx(0.25, abs=0.02)


def test_the_gait_rate_is_held():
    g = walkers.Gait(walk_length=1.0, idle_length=10.0)
    g.speed = 50.0
    p0 = g.phase = 0.0
    g.cycle = "walk"
    g.advance(50.0, 0.1)
    assert g.phase - p0 == pytest.approx(0.1 * walkers.MAX_RATE)


def test_a_switch_of_cycle_starts_it_from_the_top():
    g = walkers.Gait(walk_length=1.0, idle_length=10.0)
    g.advance(0.0, 1.0)
    assert g.phase > 0
    for _ in range(10):
        g.advance(2.0, 0.1)
    assert g.cycle == "walk"


def test_pick_is_stable_and_spread():
    ws = [{"id": "bp_walker_a"}, {"id": "bp_walker_b"}, {"id": "bp_walker_c"}]
    assert walkers.pick(ws, "bp_walker_b", "x")["id"] == "bp_walker_b"
    assert walkers.pick(ws, "walker.pedestrian.0001", "x")["id"] == "bp_walker_b"
    assert walkers.pick(ws, "walker.pedestrian.0003", "x")["id"] == "bp_walker_a"
    assert walkers.pick(ws, "", "Pedestrian0") == walkers.pick(ws, "", "Pedestrian0")
    assert walkers.pick([], "", "x") is None


def test_horizontal_speed():
    assert walkers.horizontal_speed(None, (1, 1, 1), 0.1) == 0.0
    assert walkers.horizontal_speed((0, 0, 0), (3, 4, 9), 1.0) == pytest.approx(5.0)


def test_pick_by_carla_blueprint():
    table = {
        "walker.pedestrian.0001": {"blueprint": "BP_Walker_Female1_v1", "gender": "female", "age": "adult"},
        "walker.pedestrian.0002": {"blueprint": "BP_Walker_Male1_v1", "gender": "male", "age": "adult"},
        "walker.pedestrian.0012": {"blueprint": "BP_WalkerKid1_v1", "gender": "male", "age": "child"},
        "walker.pedestrian.0050": {"blueprint": "BP_Walker_AG001_G3", "gender": "female", "age": "child"},
        "walker.pedestrian.0051": {"blueprint": "BP_Walker_AB001_G3", "gender": "male", "age": "child"},
        "walker.pedestrian.0016": {"blueprint": "BP_Walker_MaleEuro", "gender": "male", "age": "adult"},
    }
    ws = [{"id": i} for i in ("bp_walker_female1_v1", "bp_walker_maleeuro", "bp_walker_ab001_g3")]
    pick = lambda model: walkers.pick(ws, model, "x", table)["id"]
    assert pick("walker.pedestrian.0001") == "bp_walker_female1_v1"  # its own blueprint
    assert pick("walker.pedestrian.0002") == "bp_walker_maleeuro"  # same gender and age
    assert pick("walker.pedestrian.0012") == "bp_walker_ab001_g3"  # a boy for a boy
    assert pick("walker.pedestrian.0050") == "bp_walker_ab001_g3"  # a girl: still a child
    assert pick("walker.pedestrian.9999") in {w["id"] for w in ws}  # not CARLA's: spread


def test_blueprint_table():
    t = walkers.blueprints()
    assert len(t) == 52
    assert t["walker.pedestrian.0001"]["blueprint"] == "BP_Walker_Female1_v1"
    assert t["walker.pedestrian.0030"]["blueprint"] == "BP_Walker_MaleAmer_Cop"
    assert t["walker.pedestrian.0012"]["age"] == "child"
    assert len({e["blueprint"] for e in t.values()}) == 52
