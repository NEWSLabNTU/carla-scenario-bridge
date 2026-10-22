"""A two-wheeler's rider seated on it (roadmap 044, open item 2), pure numpy so it is tested
without Isaac.

CARLA poses the rider of a bicycle or motorcycle with its `AB_Biker` anim blueprint: the hands'
grip (`AS_Pedestrian_BikeHands`, the one biker asset on the riders' Gen2 skeleton) and IK to
the bike's sockets. A SimReady export runs no anim blueprint, so the rider comes out in its rest
pose (a T-pose standing about a metre behind the saddle). `seat` does what `AB_Biker` does,
once, offline (`vehicle_usd_assets.py` writes the result as a static SkelAnimation):

1. the body turned upright, facing the bike's x, the hips onto the saddle (`Seat`);
2. the spine leant forward until both hands can reach the bars (`HandlerLeftSocket`,
   `HandlerRightSocket`) with the elbows still bent;
3. two-bone IK: each arm's wrist to its handlebar socket (elbows down, back and out), each
   leg's ankle to its pedal (`LeftPedalGeo`, `RightPedalGeo`; knees forward), the feet kept
   level, the grip on the hands.

Matrices in and out follow USD: a joint's local transform is a row-vector 4x4 (`p_parent = p
* M`), its skeleton-space one `M_local * M_parent`, and the skeleton prim's local-to-asset
transform applies last. The asset frame is the bike's: metres, z up, x forward, y left.
"""
import numpy as np

# Where the hips joint (the pelvis' centre) goes over the bike's Seat joint (the saddle's top),
# in the asset frame, m.
SEAT_OFFSET = (-0.04, 0.0, 0.05)
# The wrist sits this far from the handlebar socket, back toward the rider, m (the socket is the
# grip's centre, the hand joint the wrist).
WRIST_BACK = 0.07
# Where the ankle goes from a pedal (the ball of the foot on it), m.
ANKLE_OFFSET = (-0.10, 0.0, 0.08)
# The arms reach the bars at this fraction of their full length (a slight bend at the elbow),
# or, on a bike whose bars no lean brings within that, at MAX_REACH.
REACH = 0.92
MAX_REACH = 0.99
# The spine leans forward up to this far, degrees, before the hips slide forward on the saddle
# (by SLIDES, m), and at most MAX_LEAN.
COMFORT_LEAN = 40.0
MAX_LEAN = 60.0
SLIDES = (0.0, 0.05, 0.10, 0.15)
# The head turns back up by this fraction of the lean; the shoulders come forward by PROTRACT
# degrees per degree of lean, at most MAX_PROTRACT.
HEAD_UP = 0.7
PROTRACT = 0.5
MAX_PROTRACT = 20.0
# Poles: which way a knee and an elbow point (y is flipped for the right side).
KNEE_POLE = (1.0, 0.15, 0.3)
ELBOW_POLE = (-0.5, 0.6, -0.6)

HIPS, SPINE, SPINE1, NECK, HEAD = ("crl_hips__C", "crl_spine__C", "crl_spine01__C", "crl_neck__C",
                                   "crl_Head__C")
CLAVICLE = "crl_shoulder__{}"
SIDES = {"L": 1.0, "R": -1.0}
ARM = ("crl_arm__{}", "crl_foreArm__{}", "crl_hand__{}")
LEG = ("crl_thigh__{}", "crl_leg__{}", "crl_foot__{}")
# The bike's joints a rider is fitted to.
BIKE_SOCKETS = {"seat": "Seat", "hand_L": "HandlerLeftSocket", "hand_R": "HandlerRightSocket",
                "pedal_L": "LeftPedalGeo", "pedal_R": "RightPedalGeo"}


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def rotation_between(a, b):
    """The smallest rotation (3x3) taking direction `a` to direction `b`."""
    a, b = _unit(np.asarray(a, float)), _unit(np.asarray(b, float))
    v, c = np.cross(a, b), float(np.dot(a, b))
    if c < -1 + 1e-9:  # opposite: half a turn about any axis normal to a
        axis = _unit(np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0]))
        return 2 * np.outer(axis, axis) - np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def rot_y(degrees):
    """Rotation about y: positive tips +z toward +x (a forward lean, x forward)."""
    a = np.radians(degrees)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(degrees):
    """Rotation about z: positive turns +x toward +y."""
    a = np.radians(degrees)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_xyzw(r):
    """A rotation matrix (column convention) as a unit quaternion, x y z w."""
    m = r
    t = np.trace(m)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = s / 4
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def matrix_xyzw(q):
    """A unit quaternion (x y z w) as a rotation matrix (column convention)."""
    x, y, z, w = _unit(np.asarray(q, float))
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Pose:
    """A skeleton's local transforms (rotations, translations, scales) and their forward
    kinematics in the asset frame."""

    def __init__(self, joints, rest, skel_to_asset):
        """`joints`: USD joint paths (`a`, `a/b`, ...); `rest`: their local transforms, USD
        row-vector 4x4s; `skel_to_asset`: the skeleton prim's local-to-asset 4x4 (rigid)."""
        self.paths = [str(j) for j in joints]
        self.names = [p.rsplit("/", 1)[-1] for p in self.paths]
        self.index = {n: i for i, n in enumerate(self.names)}
        where = {p: i for i, p in enumerate(self.paths)}
        self.parent = [where.get(p.rsplit("/", 1)[0], -1) if "/" in p else -1 for p in self.paths]
        rest = np.asarray(rest, float)
        m = np.transpose(rest, (0, 2, 1))  # column convention
        self.scales = np.linalg.norm(m[:, :3, :3], axis=1)
        self.rot = m[:, :3, :3] / self.scales[:, None, :]
        self.trans = m[:, :3, 3].copy()
        x = np.asarray(skel_to_asset, float).T
        self.root_rot, self.root_trans = x[:3, :3], x[:3, 3]
        self.fk()

    def fk(self):
        n = len(self.names)
        self.wrot, self.wpos = np.zeros((n, 3, 3)), np.zeros((n, 3))
        for i in range(n):  # USD orders a parent before its children
            p = self.parent[i]
            pr, pt = (self.root_rot, self.root_trans) if p < 0 else (self.wrot[p], self.wpos[p])
            self.wrot[i] = pr @ self.rot[i]
            self.wpos[i] = pr @ self.trans[i] + pt

    def _parent_frame(self, i):
        p = self.parent[i]
        return (self.root_rot, self.root_trans) if p < 0 else (self.wrot[p], self.wpos[p])

    def pos(self, name):
        return self.wpos[self.index[name]].copy()

    def set_world_rot(self, name, r):
        i = self.index[name]
        pr, _ = self._parent_frame(i)
        self.rot[i] = pr.T @ r
        self.fk()

    def set_world_pos(self, name, p):
        i = self.index[name]
        pr, pt = self._parent_frame(i)
        self.trans[i] = pr.T @ (np.asarray(p, float) - pt)
        self.fk()

    def turn(self, name, r):
        """Turn joint `name` (and so everything below it) by `r`, in the asset frame, about
        itself."""
        self.set_world_rot(name, r @ self.wrot[self.index[name]])

    def two_bone(self, upper, mid, end, target, pole):
        """Bend the chain upper -> mid -> end so `end` reaches `target` (or points at it,
        straight, when out of reach), the bend toward `pole`."""
        a, b, c = self.pos(upper), self.pos(mid), self.pos(end)
        l1, l2 = np.linalg.norm(b - a), np.linalg.norm(c - b)
        to = np.asarray(target, float) - a
        d = np.clip(np.linalg.norm(to), abs(l1 - l2) + 1e-4, (l1 + l2) * 0.9999)
        u = _unit(to)
        pv = np.asarray(pole, float)
        pv = _unit(pv - np.dot(pv, u) * u)
        cos_a = np.clip((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d), -1.0, 1.0)
        b_new = a + l1 * (cos_a * u + np.sqrt(1 - cos_a * cos_a) * pv)
        self.turn(upper, rotation_between(b - a, b_new - a))
        b, c = self.pos(mid), self.pos(end)
        self.turn(mid, rotation_between(c - b, a + d * u - b))

    def local(self):
        """`(rotations x y z w, translations, scales)`, one row per joint, for a
        SkelAnimation."""
        return (np.array([quat_xyzw(r) for r in self.rot], dtype=np.float32),
                self.trans.astype(np.float32), self.scales.astype(np.float32))


def _reach_excess(pose, targets, reach=None):
    """How far beyond `reach` (REACH) of its full length each arm is from its wrist target, m
    (<= 0: both reach)."""
    reach = REACH if reach is None else reach
    worst = -np.inf
    for s in SIDES:
        a, m, h = (n.format(s) for n in ARM)
        length = np.linalg.norm(pose.pos(m) - pose.pos(a)) + np.linalg.norm(pose.pos(h) - pose.pos(m))
        worst = max(worst, np.linalg.norm(targets[s] - pose.pos(a)) - reach * length)
    return worst


def seat(joints, rest, skel_to_asset, sockets, grip=None):
    """The rider's local pose seated on the bike: `(rotations x y z w, translations, scales)`
    per joint of `joints`, and `(slide m, lean degrees)`: how far the hips moved forward of
    SEAT_OFFSET and the spine leant.

    `rest`: the rider's rest local transforms (USD 4x4s); `skel_to_asset`: its skeleton prim's
    local-to-asset transform; `sockets`: the bike's joints in the asset frame by `BIKE_SOCKETS`
    key (`seat`, `hand_L`, `hand_R`, `pedal_L`, `pedal_R`); `grip`: an optional `{joint name:
    rotation x y z w}` for the hands and fingers (the export of `AS_Pedestrian_BikeHands`)."""
    pose = Pose(joints, rest, skel_to_asset)
    if grip:
        for name, q in grip.items():
            if name in pose.index:
                pose.rot[pose.index[name]] = matrix_xyzw(q)
        pose.fk()
    # 1. upright, facing x: the rest pose's up (hips to head) to z, its left (right shoulder to
    # left) to y.
    up = _unit(pose.pos(HEAD) - pose.pos(HIPS))
    left = pose.pos(ARM[0].format("L")) - pose.pos(ARM[0].format("R"))
    left = _unit(left - np.dot(left, up) * up)
    frame = np.column_stack([np.cross(left, up), left, up])
    pose.turn(HIPS, frame.T)
    feet = {s: pose.wrot[pose.index[LEG[2].format(s)]].copy() for s in SIDES}
    # 2. lean (split over the two spine joints, the head kept up, the shoulders brought
    # forward): the least that lets both arms reach the bars; past COMFORT_LEAN the hips slide
    # forward on the saddle first (a sport bike's rider sits on its front).
    hips_at = np.asarray(sockets["seat"], float) + SEAT_OFFSET
    rest_rot = {n: pose.wrot[pose.index[n]].copy()
                for n in (HIPS, SPINE, SPINE1, NECK) + tuple(CLAVICLE.format(s) for s in SIDES)}

    def lean_to(slide, lean):
        # A third of the lean each at the pelvis and the two spine joints (the legs are bent
        # to the pedals afterwards whatever the pelvis does).
        pose.set_world_pos(HIPS, hips_at + (slide, 0.0, 0.0))
        for k, name in enumerate((HIPS, SPINE, SPINE1)):
            pose.set_world_rot(name, rot_y(lean * (k + 1) / 3) @ rest_rot[name])
        full = rot_y(lean)
        pose.set_world_rot(NECK, rot_y(-HEAD_UP * lean) @ full @ rest_rot[NECK])
        forward = min(MAX_PROTRACT, PROTRACT * lean)
        for s, side in SIDES.items():
            c = CLAVICLE.format(s)
            pose.set_world_rot(c, rot_z(-side * forward) @ full @ rest_rot[c])
        targets = {s: np.asarray(sockets["hand_" + s], float) for s in SIDES}
        return {s: t + WRIST_BACK * _unit(pose.pos(ARM[0].format(s)) - t)
                for s, t in targets.items()}

    tries = [(slide, lean) for slide in SLIDES for lean in range(0, int(COMFORT_LEAN) + 1)]
    tries += [(SLIDES[-1], lean) for lean in range(int(COMFORT_LEAN) + 1, int(MAX_LEAN) + 1)]
    best = (np.inf, tries[0])
    for reach in (REACH, MAX_REACH):  # a bent elbow if it can, else nearly straight arms
        for slide, lean in tries:
            excess = _reach_excess(pose, lean_to(slide, lean), reach)
            if excess <= 0:
                best = (excess, (slide, lean))
                break
            best = min(best, (excess, (slide, lean)))
        if best[0] <= 0:
            break
    slide, lean = best[1]
    wrists = lean_to(slide, lean)
    # 3. arms and legs
    for s, side in SIDES.items():
        target = wrists[s]
        pole = np.array(ELBOW_POLE) * (1.0, side, 1.0)
        pose.two_bone(*(n.format(s) for n in ARM), target, pole)
        ankle = np.asarray(sockets["pedal_" + s], float) + ANKLE_OFFSET
        pose.two_bone(*(n.format(s) for n in LEG), ankle, np.array(KNEE_POLE) * (1.0, side, 1.0))
        pose.set_world_rot(LEG[2].format(s), feet[s])
    rotations, translations, scales = pose.local()
    return rotations, translations, scales, (slide, lean)
