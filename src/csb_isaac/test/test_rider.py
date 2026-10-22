"""csb_isaac.rider: a two-wheeler's rider seated on its sockets (044 open item 2)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from csb_isaac import rider  # noqa: E402

# A stick rider on CARLA's Gen2 joint names, in a y-up skeleton frame (x: the rider's left),
# T-pose, joints' rotations identity: (name, parent, translation from the parent).
STICK = [
    ("crl_root", None, (0, 0, 0)),
    ("crl_hips__C", "crl_root", (0, 1.0, 0)),
    ("crl_spine__C", "crl_hips__C", (0, 0.1, 0)),
    ("crl_spine01__C", "crl_spine__C", (0, 0.17, 0)),
    ("crl_shoulder__L", "crl_spine01__C", (0.05, 0.15, 0)),
    ("crl_arm__L", "crl_shoulder__L", (0.1, 0, 0)),
    ("crl_foreArm__L", "crl_arm__L", (0.27, 0, 0)),
    ("crl_hand__L", "crl_foreArm__L", (0.25, 0, 0)),
    ("crl_handIndex__L", "crl_hand__L", (0.08, 0, 0)),
    ("crl_neck__C", "crl_spine01__C", (0, 0.2, 0)),
    ("crl_Head__C", "crl_neck__C", (0, 0.09, 0)),
    ("crl_shoulder__R", "crl_spine01__C", (-0.05, 0.15, 0)),
    ("crl_arm__R", "crl_shoulder__R", (-0.1, 0, 0)),
    ("crl_foreArm__R", "crl_arm__R", (-0.27, 0, 0)),
    ("crl_hand__R", "crl_foreArm__R", (-0.25, 0, 0)),
    ("crl_thigh__R", "crl_hips__C", (-0.08, -0.09, 0)),
    ("crl_leg__R", "crl_thigh__R", (0, -0.45, 0)),
    ("crl_foot__R", "crl_leg__R", (0, -0.42, 0)),
    ("crl_thigh__L", "crl_hips__C", (0.08, -0.09, 0)),
    ("crl_leg__L", "crl_thigh__L", (0, -0.45, 0)),
    ("crl_foot__L", "crl_leg__L", (0, -0.42, 0)),
]
# The bike: saddle, bars and pedals in the asset frame (x forward, y left, z up).
SOCKETS = {"seat": (-0.35, 0.0, 0.95), "hand_L": (0.2, 0.25, 1.05), "hand_R": (0.2, -0.25, 1.05),
           "pedal_L": (0.05, 0.12, 0.3), "pedal_R": (-0.2, -0.12, 0.3)}


def stick():
    paths, by_name = [], {}
    for name, parent, _ in STICK:
        by_name[name] = name if parent is None else by_name[parent] + "/" + name
        paths.append(by_name[name])
    rest = np.array([np.eye(4) for _ in STICK])
    for m, (_, _, t) in zip(rest, STICK):
        m[3, :3] = t  # USD row-vector: translation in the last row
    # The skeleton prim: y up -> z, the rider's left (x) -> y, pitched 20 degrees and standing a
    # metre behind the bike, as the exported rider components do.
    turn = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)  # rows: where x, y, z go
    tilt = rider.rot_y(20.0).T  # row-vector form of a column-convention rotation
    xf = np.eye(4)
    xf[:3, :3] = turn @ tilt
    xf[3, :3] = (-1.2, 0.0, 0.1)
    return paths, rest, xf


def posed(rotations, translations):
    paths, rest, xf = stick()
    p = rider.Pose(paths, rest, xf)
    p.rot = np.array([rider.matrix_xyzw(q) for q in rotations])
    p.trans = np.asarray(translations, float)
    p.fk()
    return p


def test_quaternions_round_trip():
    for axis, deg in (((0, 0, 1), 30), ((1, 0, 0), 170), ((0, 1, 0), -90), ((1, 1, 1), 200)):
        a = np.radians(deg)
        u = np.asarray(axis, float) / np.linalg.norm(axis)
        q = np.append(u * np.sin(a / 2), np.cos(a / 2))
        back = rider.quat_xyzw(rider.matrix_xyzw(q))
        assert min(np.abs(back - q).max(), np.abs(back + q).max()) < 1e-9


def test_rotation_between_takes_a_to_b():
    for a, b in (((1, 0, 0), (0, 1, 0)), ((0, 0, 1), (0, 0, -1)), ((1, 2, 3), (-3, 1, 0.5))):
        r = rider.rotation_between(a, b)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
        assert np.allclose(r @ (np.asarray(a) / np.linalg.norm(a)),
                           np.asarray(b) / np.linalg.norm(b), atol=1e-9)


def test_forward_kinematics_follows_usd_order():
    paths, rest, xf = stick()
    p = rider.Pose(paths, rest, xf)
    # Hips 1.0 up the skeleton's y: in the asset frame z, after the tilt about y.
    hips = p.pos("crl_hips__C")
    assert hips == pytest.approx(np.array((-1.2, 0, 0.1)) + rider.rot_y(20.0) @ (0, 0, 1.0))
    assert p.pos("crl_hand__L")[1] > 0.6  # the left arm is out on +y


def test_two_bone_reaches_a_target_within_reach():
    paths, rest, xf = stick()
    p = rider.Pose(paths, rest, xf)
    target = p.pos("crl_arm__L") + (0.3, 0.05, -0.2)
    p.two_bone("crl_arm__L", "crl_foreArm__L", "crl_hand__L", target, (0, 1, -1))
    assert p.pos("crl_hand__L") == pytest.approx(target, abs=1e-6)
    # bones keep their lengths, the elbow bends toward the pole
    assert np.linalg.norm(p.pos("crl_foreArm__L") - p.pos("crl_arm__L")) == pytest.approx(0.27)
    mid = (p.pos("crl_arm__L") + target) / 2
    assert np.dot(p.pos("crl_foreArm__L") - mid, (0, 1, -1)) > 0


def test_the_rider_sits_on_the_saddle_with_hands_on_the_bars_and_feet_on_the_pedals():
    paths, rest, xf = stick()
    rotations, translations, scales, (slide, lean) = rider.seat(paths, rest, xf, SOCKETS)
    assert rotations.shape == (len(paths), 4) and translations.shape == (len(paths), 3)
    assert np.allclose(np.linalg.norm(rotations, axis=1), 1.0, atol=1e-5)
    assert np.allclose(scales, 1.0)
    p = posed(rotations, translations)
    seat = np.array(SOCKETS["seat"]) + rider.SEAT_OFFSET + (slide, 0, 0)
    assert p.pos("crl_hips__C") == pytest.approx(seat, abs=1e-5)
    assert 0 <= lean <= rider.MAX_LEAN
    for s in "LR":
        wrist = p.pos(f"crl_hand__{s}")
        assert np.linalg.norm(wrist - SOCKETS["hand_" + s]) == pytest.approx(rider.WRIST_BACK, abs=1e-3)
        ankle = np.array(SOCKETS["pedal_" + s]) + rider.ANKLE_OFFSET
        assert p.pos(f"crl_foot__{s}") == pytest.approx(ankle, abs=1e-3)
        # knees forward of the hips, elbows below the shoulders
        assert p.pos(f"crl_leg__{s}")[0] > p.pos("crl_hips__C")[0]
        assert p.pos(f"crl_foreArm__{s}")[2] < p.pos(f"crl_arm__{s}")[2]
    # upright and facing x: the head above the hips, the left arm on +y
    assert p.pos("crl_Head__C")[2] > p.pos("crl_hips__C")[2] + 0.4
    assert p.pos("crl_arm__L")[1] > p.pos("crl_arm__R")[1]


def test_the_feet_stay_level_and_the_grip_is_kept():
    paths, rest, xf = stick()
    half_turn = np.array([0, 0, np.sin(0.4), np.cos(0.4)])
    rotations, translations, _, _ = rider.seat(paths, rest, xf, SOCKETS,
                                               grip={"crl_handIndex__L": half_turn})
    i = [n.rsplit("/", 1)[-1] for n in paths].index("crl_handIndex__L")
    assert min(np.abs(rotations[i] - half_turn).max(), np.abs(rotations[i] + half_turn).max()) < 1e-5
    p = posed(rotations, translations)
    for s in "LR":
        # the foot's frame as standing: its local y (the shin's direction) up the world z
        assert p.wrot[p.index[f"crl_foot__{s}"]] @ (0, 1, 0) == pytest.approx((0, 0, 1), abs=1e-6)


def test_bars_far_ahead_lean_further():
    paths, rest, xf = stick()
    near = rider.seat(paths, rest, xf, SOCKETS)[3]
    far = dict(SOCKETS, hand_L=(0.45, 0.25, 0.95), hand_R=(0.45, -0.25, 0.95))
    assert rider.seat(paths, rest, xf, far)[3] > near
