"""Unit tests for gap_oracle, the gap trace oracle (roadmap 024 step 6).

Synthetic traces built with gap_trace's records and written with its writer (`dumps`), so
the oracle reads exactly what the bridge would write. No ROS, no CARLA:

    python3 -m pytest src/csb_launch/test/test_gap_oracle.py
"""

import importlib.machinery
import importlib.util
import io
import json
import math
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(_SCRIPTS))

import gap_trace as gt  # noqa: E402


def _load(name):
    loader = importlib.machinery.SourceFileLoader(name, str(_SCRIPTS / name))
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


go = _load("gap_oracle")

# --- synthetic world ---------------------------------------------------------------------
# All entities drive along +x on y = 0 unless a test says otherwise. Boxes are those of the
# step 2-3 tests: a 4.5 m declared car whose CARLA body is a Model 3 (4.7 x 2.1), and a
# declared 12 m truck whose CARLA body is the same Model 3.

MODEL3 = gt.BlueprintBox(location=(0.0, 0.0, 0.72), extent=(2.35, 1.05, 0.72))


def header(name, kind, center, dims, actor_id, bp=MODEL3, blueprint="vehicle.tesla.model3",
           frame=0, offset_yaw=0.0):
    return gt.EntityHeader(
        frame=frame,
        name=name,
        kind=kind,
        declared=gt.DeclaredBox(center=tuple(center), dimensions=tuple(dims)),
        origin_offset=gt.OriginOffset(center[0], center[1], offset_yaw),
        actor_id=actor_id,
        blueprint=blueprint,
        blueprint_box=bp,
    )


EGO = header("ego", "ego", (1.5, 0.0, 0.9), (4.5, 2.1, 1.8), 211)
TRUCK = header("truck", "vehicle", (4.0, 0.0, 1.75), (12.0, 2.5, 3.5), 212)
CAR = header("car", "vehicle", (1.5, 0.0, 0.9), (4.5, 2.1, 1.8), 213)
# A declared 2.0 m motorbike whose CARLA body is still the Model 3 (the pre-022 body).
BIKE = header("bike", "vehicle", (0.6, 0.0, 0.75), (2.0, 0.8, 1.5), 214)
WALKER = header("walker", "pedestrian", (0.0, 0.0, 0.9), (0.6, 0.6, 1.8), 215,
                bp=gt.BlueprintBox(location=(0.0, 0.0, 0.0), extent=(0.19, 0.19, 0.93)),
                blueprint="walker.pedestrian.0001")
HEADERS = {h.name: h for h in (EGO, TRUCK, CAR, BIKE, WALKER)}


def state(name, x, y=0.0, yaw=0.0, z=0.0, *, dx=0.0, dy=0.0, dyaw=0.0, dz=0.0,
          carla=True, flags=()):
    """An entity's SSv2 pose and the CARLA pose the bridge should produce for it (origin
    offset applied, z on the same ground), perturbed by (dx, dy, dyaw, dz)."""
    h = HEADERS[name]
    ssv2 = gt.Pose6(x, y, z, 0.0, 0.0, yaw)
    if not carla:
        return gt.EntityState(ssv2=ssv2, carla=None, velocity=None, flags=tuple(flags))
    o = h.origin_offset
    ax = x + o.x * math.cos(yaw) - o.y * math.sin(yaw) + dx
    ay = y + o.x * math.sin(yaw) + o.y * math.cos(yaw) + dy
    # Same ground: the CARLA box bottom where SSv2's is.
    ssv2_bottom = z + h.declared.center[2] - h.declared.dimensions[2] / 2.0
    az = ssv2_bottom - (h.blueprint_box.location[2] - h.blueprint_box.extent[2]) + dz
    pose = gt.Pose6(ax, ay, az, 0.0, 0.0, yaw + o.yaw + dyaw)
    return gt.EntityState(ssv2=ssv2, carla=pose, velocity=(0.0, 0.0, 0.0), flags=tuple(flags))


def frame(n, *states):
    """`states`: (name, EntityState) pairs."""
    return gt.Frame(frame=n, sim_time=n * 0.05, entities=dict(states))


def contact(n, other=None, kind=None, impulse=500.0, ego_id=211, subject=None):
    """A contact the ego's sensor saw, or with `subject` the named NPC's own (032)."""
    if kind == "unknown":
        return gt.Contact(frame=n, carla_frame=1000 + n, sim_time=n * 0.05, ego_id=ego_id,
                          other_id=999, name=None, other_type="vehicle.audi.tt",
                          other_kind="unknown", impulse=impulse, subject=subject)
    if other is None:
        return gt.Contact(frame=n, carla_frame=1000 + n, sim_time=n * 0.05, ego_id=ego_id,
                          other_id=None, name=None, other_type="static.pole",
                          other_kind="static", impulse=impulse, subject=subject)
    h = HEADERS[other]
    return gt.Contact(frame=n, carla_frame=1000 + n, sim_time=n * 0.05, ego_id=ego_id,
                      other_id=h.actor_id, name=other, other_type=h.blueprint,
                      other_kind=h.kind, impulse=impulse, subject=subject)


def make_trace(names, frames, contacts=(), end=True):
    trace = gt.Trace(episode=gt.Episode(schema=1, episode=1, start_unix=1760000000.0,
                                        npc_motion="tracked"))
    for n in names:
        trace.entities[n] = HEADERS[n]
        trace.headers.append(HEADERS[n])
    trace.frames = list(frames)
    trace.contacts = sorted(contacts, key=lambda c: c.frame)
    if end:
        last = trace.frames[-1].frame
        trace.end = gt.End(frame=last, sim_time=last * 0.05, frames=len(trace.frames),
                           dropped_frames=0, complete=True)
    # Through the writer and the reader, as the oracle sees a real trace.
    return gt.load_trace(gt.dumps(trace).splitlines(), require_end=end)


def run(trace, verdict=None, **tol):
    return go.analyze(trace, go.Tolerances(**tol), verdict)


def checks(result, check, priority=None):
    return [f for f in result["findings"] if f["check"] == check
            and (priority is None or f["priority"] == priority)]


def info(result, check):
    return [f for f in result["info"] if f["check"] == check]


# Truck origin 8.5 m behind the ego's: CARLA's Model 3 front is 1.3 m clear of the ego's
# CARLA rear, SSv2's 12 m box reaches 2.25 m into the ego's declared box (longitudinal depth;
# the lateral one is 2.3).
def truck_behind(n, gap=8.5, ego_x=0.0, **kw):
    return frame(n, ("ego", state("ego", ego_x)), ("truck", state("truck", ego_x - gap, **kw)))


# --- G1 -----------------------------------------------------------------------------------


class G1OverlapDisagreement(unittest.TestCase):
    def test_ssv2_only_overlap_from_declared_vs_blueprint_size(self):
        t = make_trace(["ego", "truck"], [truck_behind(n, ego_x=n * 0.2) for n in range(1, 6)])
        r = run(t)
        g1 = checks(r, "G1")
        self.assertEqual(len(g1), 1)
        f = g1[0]
        self.assertEqual(f["entities"], ["ego", "truck"])
        self.assertEqual(f["priority"], "high")
        self.assertEqual(f["geometry"], "ssv2")
        self.assertEqual(f["frames"], [1, 5])
        self.assertEqual(f["count"], 5)
        nums = f["numbers"]
        self.assertAlmostEqual(nums["max_penetration"], 2.25, places=6)
        self.assertAlmostEqual(nums["min_gap"], 1.3, places=6)
        self.assertAlmostEqual(nums["margin_difference"], -3.55, places=6)
        self.assertIn("C1:vehicle:ssv2", r["signature"])

    def test_silent_when_both_geometries_clear(self):
        t = make_trace(["ego", "truck"], [truck_behind(n, gap=20.0) for n in range(1, 6)])
        r = run(t)
        self.assertEqual(checks(r, "G1"), [])
        self.assertAlmostEqual(r["margins"]["min_d_ssv2"], 20.0 - 10.75, places=6)

    def test_silent_when_both_geometries_overlap(self):
        t = make_trace(["ego", "truck"], [truck_behind(n, gap=5.0) for n in range(1, 6)])
        self.assertEqual(checks(run(t), "G1"), [])

    def test_carla_only_overlap_from_a_body_longer_than_declared(self):
        # Bike origin 3.6 m behind the ego's: its declared box (-4.0 to -2.0) is 1.25 m
        # clear of the ego's declared rear (-0.75); its Model 3 body (centre -3.0, front
        # -0.65) reaches 0.2 m into the ego's CARLA rear (-0.85).
        frames = [frame(n, ("ego", state("ego", 0.0)), ("bike", state("bike", -3.6)))
                  for n in range(1, 4)]
        r = run(make_trace(["ego", "bike"], frames))
        g1 = checks(r, "G1")
        self.assertEqual(len(g1), 1)
        self.assertEqual(g1[0]["geometry"], "carla")
        self.assertAlmostEqual(g1[0]["numbers"]["min_gap"], 1.25, places=6)
        self.assertAlmostEqual(g1[0]["numbers"]["max_penetration"], 0.2, places=6)
        self.assertAlmostEqual(g1[0]["numbers"]["margin_difference"], 1.45, places=6)
        self.assertIn("C1:vehicle:carla", r["signature"])

    def test_two_intervals_are_two_findings(self):
        frames = [truck_behind(n, gap=8.5 if n in (1, 2, 5, 6) else 20.0) for n in range(1, 7)]
        g1 = checks(run(make_trace(["ego", "truck"], frames)), "G1")
        self.assertEqual([f["frames"] for f in g1], [[1, 2], [5, 6]])

    def test_npc_pairs_are_judged_at_low_priority(self):
        frames = [frame(n, ("ego", state("ego", 100.0)), ("truck", state("truck", -8.5)),
                        ("car", state("car", 0.0))) for n in range(1, 4)]
        r = run(make_trace(["ego", "truck", "car"], frames))
        self.assertEqual(checks(r, "G1", "high"), [])
        low = checks(r, "G1", "low")
        self.assertEqual(len(low), 1)
        self.assertEqual(low[0]["entities"], ["truck", "car"])
        self.assertNotIn("C1:vehicle:ssv2", r["signature"])

    def test_height_separated_boxes_do_not_overlap_in_ssv2(self):
        # The truck 5 m up (a bridge deck): SSv2's height gate skips the 2D check; CARLA's
        # boxes are as far apart in z, so neither overlaps.
        frames = [frame(n, ("ego", state("ego", 0.0)), ("truck", state("truck", -5.0, z=5.0)))
                  for n in range(1, 4)]
        self.assertEqual(checks(run(make_trace(["ego", "truck"], frames)), "G1"), [])

    def test_a_recorded_contact_counts_as_overlap_in_carla(self):
        # CARLA's box says 1.3 m clear, but CARLA recorded a contact of the pair at frame 3:
        # the contact is CARLA's word, so frames 3-5 (it and the 2 after) agree; before it
        # there was no contact yet.
        t = make_trace(["ego", "truck"], [truck_behind(n) for n in range(1, 8)],
                       [contact(3, "truck")])
        g1 = checks(run(t), "G1")
        self.assertEqual([f["frames"] for f in g1], [[1, 2], [6, 7]])

    def test_disagreement_within_the_egos_one_tick_lag_is_info(self):
        # The ego's CARLA pose 0.4 m ahead of its SSv2 pose (one tick at 8 m/s, read back
        # before the tick). A car 4.4 m behind: SSv2 0.1 m into the ego, CARLA 0.1 m clear;
        # |d_ssv2 - d_carla| = 0.2 m, within 0.4 + 0.05: timing, not geometry.
        frames = [frame(n, ("ego", state("ego", 0.0, dx=0.4)), ("car", state("car", -4.4)))
                  for n in range(1, 4)]
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(checks(r, "G1"), [])
        band = info(r, "G1")
        self.assertEqual(len(band), 1)
        self.assertEqual((band[0]["kind"], band[0]["geometry"], band[0]["frames"]),
                         ("lag_band", "ssv2", [1, 3]))
        self.assertAlmostEqual(band[0]["numbers"]["max_band"], 0.45, places=6)
        # A band narrower than the difference (--touch -0.25: 0.15 m) makes it a finding.
        self.assertEqual(len(checks(run(make_trace(["ego", "car"], frames), touch=-0.25),
                                "G1")), 1)

    def test_the_lag_does_not_excuse_a_large_difference(self):
        # Same lag, the declared 12 m truck: CARLA 0.2 m clear, SSv2 3.75 m deep.
        frames = [frame(n, ("ego", state("ego", 0.0, dx=0.4)), ("truck", state("truck", -7.0)))
                  for n in range(1, 4)]
        g1 = checks(run(make_trace(["ego", "truck"], frames)), "G1")
        self.assertEqual(len(g1), 1)
        self.assertAlmostEqual(g1[0]["numbers"]["min_gap"], 0.2, places=6)

    def test_a_contact_only_carla_overlap_with_the_ego_airborne(self):
        # rear_contact run 3 (032 N3): the ego thrown 5 m up, so CARLA's height gate
        # separates the boxes (no margin at all), yet CARLA recorded a contact -- the
        # contact rule alone makes CARLA's side overlap. SSv2 1.5 m clear. The interval has
        # no CARLA margin to take a penetration from: reported as unknown, not a crash.
        frames = [frame(n, ("ego", state("ego", 0.0, dz=5.0)), ("car", state("car", -6.0)))
                  for n in range(1, 6)]
        r = run(make_trace(["ego", "car"], frames, [contact(2, "car")]))
        g1 = checks(r, "G1")
        self.assertEqual(len(g1), 1)
        f = g1[0]
        self.assertEqual((f["geometry"], f["frames"], f["priority"]), ("carla", [2, 4], "high"))
        self.assertIsNone(f["numbers"]["max_penetration"])
        self.assertIsNone(f["numbers"]["deepest_frame"])
        self.assertAlmostEqual(f["numbers"]["min_gap"], 1.5, places=6)
        self.assertIsNone(f["numbers"]["margin_difference"])

    def test_missing_carla_state_is_not_an_overlap_disagreement(self):
        frames = [frame(n, ("ego", state("ego", 0.0)),
                        ("truck", state("truck", -8.5, carla=False))) for n in range(1, 4)]
        r = run(make_trace(["ego", "truck"], frames))
        self.assertEqual(checks(r, "G1"), [])


# --- G2 -----------------------------------------------------------------------------------


class G2ContactSsv2CannotSee(unittest.TestCase):
    def frames(self, overlap_frames, n=12):
        return [truck_behind(i, gap=5.0 if i in overlap_frames else 20.0) for i in range(1, n)]

    def test_contact_with_ssv2_overlap_within_two_frames_is_silent(self):
        t = make_trace(["ego", "truck"], self.frames({5}), [contact(7, "truck")])
        self.assertEqual(checks(run(t), "G2"), [])

    def test_contact_without_ssv2_overlap_is_a_finding(self):
        t = make_trace(["ego", "truck"], self.frames({3}),
                       [contact(7, "truck", impulse=100.0), contact(8, "truck", impulse=300.0)])
        r = run(t)
        g2 = checks(r, "G2")
        self.assertEqual(len(g2), 1)
        f = g2[0]
        self.assertEqual(f["entities"], ["ego", "truck"])
        self.assertEqual(f["frames"], [7, 8])
        self.assertEqual(f["count"], 2)
        self.assertEqual(f["numbers"]["max_impulse"], 300.0)
        self.assertIn("C2:vehicle:carla", r["signature"])

    def test_every_static_hit_is_a_finding(self):
        t = make_trace(["ego", "truck"], self.frames({7}), [contact(7)])
        g2 = checks(run(t), "G2")
        self.assertEqual(len(g2), 1)
        self.assertEqual(g2[0]["entities"], ["ego", "static"])
        self.assertEqual(g2[0]["numbers"]["other_type"], "static.pole")

    def test_contact_with_an_actor_ssv2_does_not_know(self):
        t = make_trace(["ego", "truck"], self.frames(set()), [contact(4, kind="unknown")])
        g2 = checks(run(t), "G2")
        self.assertEqual(len(g2), 1)
        self.assertEqual(g2[0]["entities"], ["ego", "unknown:999"])

    def test_contact_window_is_a_flag(self):
        t = make_trace(["ego", "truck"], self.frames({5}), [contact(8, "truck")])
        self.assertEqual(len(checks(run(t), "G2")), 1)
        self.assertEqual(checks(run(t, contact_window=3), "G2"), [])


class G2NpcContacts(unittest.TestCase):
    """Contacts a tracked NPC's own collision sensor saw (`subject`, roadmap 032 step 5): low
    priority, as NPC-NPC G1 pairs are, and never a second copy of an ego contact."""

    def npc_frames(self, overlap_frames=(), n=12):
        # The ego far ahead; the car 4 m behind the truck's origin overlaps the truck's
        # declared 12 m box (SSv2), 20 m behind it does not. CARLA's Model 3s are 1.8 m clear
        # at 4 m.
        return [frame(i, ("ego", state("ego", 100.0)), ("truck", state("truck", 0.0)),
                      ("car", state("car", -4.0 if i in overlap_frames else -20.0)))
                for i in range(1, n)]

    def test_an_npc_static_hit_is_a_low_priority_finding(self):
        t = make_trace(["ego", "truck", "car"], self.npc_frames(),
                       [contact(5, subject="truck", impulse=4000.0),
                        contact(6, subject="truck", impulse=900.0)])
        r = run(t)
        g2 = checks(r, "G2")
        self.assertEqual(len(g2), 1)
        f = g2[0]
        self.assertEqual((f["priority"], f["entities"], f["frames"], f["count"]),
                         ("low", ["truck", "static"], [5, 6], 2))
        self.assertEqual(f["numbers"]["max_impulse"], 4000.0)
        self.assertEqual(f["numbers"]["other_kind"], "static")
        self.assertIn("C2:npc_static:carla", r["signature"])
        self.assertNotIn("C2:static:carla", r["signature"])
        self.assertEqual((r["counts"]["G2"], r["counts"]["G2_ego"], r["counts"]["G2_npc"]),
                         (1, 0, 1))

    def test_an_npc_seeing_the_ego_is_not_counted_twice(self):
        # The ego's sensor and the truck's both report their contact: one G2, the ego's.
        t = make_trace(["ego", "truck"], [truck_behind(i, gap=20.0) for i in range(1, 12)],
                       [contact(5, "truck"), contact(5, "ego", subject="truck")])
        g2 = checks(run(t), "G2")
        self.assertEqual([(f["priority"], f["entities"]) for f in g2],
                         [("high", ["ego", "truck"])])
        # Seen only from the NPC's side (the ego's sensor missed it): still not an NPC
        # finding -- the ego's contacts are the ego sensor's to report.
        t = make_trace(["ego", "truck"], [truck_behind(i, gap=20.0) for i in range(1, 12)],
                       [contact(5, "ego", subject="truck")])
        self.assertEqual(checks(run(t), "G2"), [])

    def test_both_sides_of_an_npc_pair_are_one_finding(self):
        t = make_trace(["ego", "truck", "car"], self.npc_frames(),
                       [contact(5, "car", subject="truck"), contact(5, "truck", subject="car")])
        g2 = checks(run(t), "G2")
        self.assertEqual(len(g2), 1)
        self.assertEqual((g2[0]["priority"], g2[0]["entities"], g2[0]["count"]),
                         ("low", ["truck", "car"], 2))
        self.assertIn("C2:npc_pair:carla", run(t)["signature"])

    def test_an_npc_pair_ssv2_saw_overlap_is_silent(self):
        t = make_trace(["ego", "truck", "car"], self.npc_frames({4}),
                       [contact(5, "car", subject="truck")])
        self.assertEqual(checks(run(t), "G2"), [])

    def test_an_npc_pair_contact_counts_as_overlap_in_carla(self):
        # SSv2's boxes overlap on 1-5, CARLA's bodies are 1.8 m apart; the truck's sensor
        # recorded the pair at frame 1, so 1-3 agree and only 4-5 are a (low) G1.
        t = make_trace(["ego", "truck", "car"], self.npc_frames({1, 2, 3, 4, 5}, n=6),
                       [contact(1, "car", subject="truck")])
        g1 = checks(run(t), "G1")
        self.assertEqual([(f["priority"], f["frames"]) for f in g1], [("low", [4, 5])])

    def test_npc_contacts_do_not_decide_the_verdict_class(self):
        t = make_trace(["ego", "truck", "car"], self.npc_frames(), [contact(5, subject="truck")])
        self.assertEqual(run(t, go.read_junit(self.junit(PASS)))["g4"]["class"], "agree")
        self.assertEqual(run(t, go.read_junit(self.junit(FAIL_COLLISION)))["g4"]["class"],
                         "FAIL(collision)+no-contact")

    def test_the_summary_line_splits_g2(self):
        t = make_trace(["ego", "truck", "car"], self.npc_frames(), [contact(5, subject="truck")])
        self.assertIn("G2 1 (0 ego, 1 npc)", go.summary_line("t", run(t)))

    def junit(self, text):
        p = Path(tempfile.mkdtemp()) / "result.junit.xml"
        p.write_text(text)
        return p


# --- G3 -----------------------------------------------------------------------------------


class G3PoseDivergence(unittest.TestCase):
    def frames(self, bad, n=10, **kw):
        return [frame(i, ("ego", state("ego", 50.0)),
                      ("car", state("car", 0.0, **(kw if i in bad else {}))))
                for i in range(1, n)]

    def test_sustained_in_plane_divergence_is_a_finding(self):
        r = run(make_trace(["ego", "car"], self.frames({3, 4, 5}, dy=0.5)))
        g3 = checks(r, "G3")
        self.assertEqual(len(g3), 1)
        f = g3[0]
        self.assertEqual(f["entities"], ["car"])
        self.assertEqual(f["frames"], [3, 5])
        self.assertEqual(f["count"], 3)
        self.assertAlmostEqual(f["numbers"]["max_xy"], 0.5, places=6)
        self.assertIn("C3:vehicle:pose", r["signature"])

    def test_brief_divergence_is_not(self):
        self.assertEqual(checks(run(make_trace(["ego", "car"], self.frames({3, 4}, dy=0.5))),
                                "G3"), [])

    def test_within_tolerance_is_not(self):
        self.assertEqual(checks(run(make_trace(["ego", "car"],
                                               self.frames(set(range(10)), dy=0.15))), "G3"), [])

    def test_yaw_divergence(self):
        g3 = checks(run(make_trace(["ego", "car"],
                                   self.frames({2, 3, 4, 5}, dyaw=math.radians(3.0)))), "G3")
        self.assertEqual(len(g3), 1)
        self.assertAlmostEqual(g3[0]["numbers"]["max_yaw_deg"], 3.0, places=6)

    def test_z_divergence(self):
        g3 = checks(run(make_trace(["ego", "car"], self.frames({2, 3, 4}, dz=0.6))), "G3")
        self.assertEqual(len(g3), 1)
        self.assertAlmostEqual(g3[0]["numbers"]["max_z"], 0.6, places=6)

    def test_walker_origin_at_body_centre_is_not_a_lift(self):
        # A CARLA walker's origin is its body centre, 0.93 m above its feet; z is compared
        # box bottom to box bottom, so a walker standing on the ground is no finding.
        frames = [frame(i, ("ego", state("ego", 50.0)), ("walker", state("walker", 0.0)))
                  for i in range(1, 6)]
        t = make_trace(["ego", "walker"], frames)
        self.assertGreater(t.frames[0].entities["walker"].carla.z, 0.9)
        self.assertEqual(checks(run(t), "G3"), [])

    def test_tolerances_are_flags(self):
        t = make_trace(["ego", "car"], self.frames({3, 4, 5}, dy=0.5))
        self.assertEqual(checks(run(t, pos=0.6), "G3"), [])
        self.assertEqual(checks(run(t, sustain=4), "G3"), [])

    def test_spawn_and_contact_hold_frames_are_info(self):
        frames = self.frames(set(), n=10)
        for i in (3, 4, 5, 6):
            flags = ("spawn",) if i == 3 else ("contact_hold",)
            frames[i - 1].entities["car"] = state("car", 0.0, dy=1.6, flags=flags)
        t = make_trace(["ego", "car"], [gt.Frame(f.frame, f.sim_time, dict(f.entities))
                                        for f in frames])
        r = run(t)
        self.assertEqual(checks(r, "G3"), [])
        notes = info(r, "G3")
        self.assertEqual({n["kind"] for n in notes}, {"spawn", "contact_hold"})
        hold = [n for n in notes if n["kind"] == "contact_hold"][0]
        self.assertEqual(hold["frames"], [4, 6])
        self.assertAlmostEqual(hold["numbers"]["max_xy"], 1.6, places=6)

    def test_kinematic_fallback_frames_are_info(self):
        # Roadmap 032: a blocked NPC handed to kinematic is at SSv2's pose by construction;
        # its hand-over (and anything left of the block) is info, not a finding.
        frames = self.frames(set(), n=12)
        for i in range(4, 12):
            frames[i - 1].entities["car"] = state("car", 0.0, dz=0.6 if i < 7 else 0.0,
                                                  dy=0.9 if i < 7 else 0.0,
                                                  flags=("kinematic",))
        t = make_trace(["ego", "car"], [gt.Frame(f.frame, f.sim_time, dict(f.entities))
                                        for f in frames])
        r = run(t)
        self.assertEqual(checks(r, "G3"), [])
        notes = info(r, "G3")
        self.assertEqual([n["kind"] for n in notes], ["kinematic"])
        self.assertEqual(notes[0]["frames"], [4, 6])
        # The same divergence without the flag is a finding.
        for i in range(4, 7):
            frames[i - 1].entities["car"] = state("car", 0.0, dy=0.9)
        t = make_trace(["ego", "car"], [gt.Frame(f.frame, f.sim_time, dict(f.entities))
                                        for f in frames])
        self.assertEqual(len(checks(run(t), "G3")), 1)

    def test_ego_is_not_judged(self):
        frames = [frame(i, ("ego", state("ego", 0.0, dx=0.43)), ("car", state("car", 30.0)))
                  for i in range(1, 10)]
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(checks(r, "G3"), [])

    def test_missed_carla_state_is_info(self):
        frames = [frame(i, ("ego", state("ego", 50.0)),
                        ("car", state("car", 0.0, carla=i not in (4, 5, 6, 7))))
                  for i in range(1, 10)]
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(checks(r, "G3"), [])
        missed = [n for n in info(r, "G3") if n["kind"] == "missed"]
        self.assertEqual(len(missed), 1)
        self.assertEqual((missed[0]["frames"], missed[0]["count"]), ([4, 7], 4))

    def test_a_read_back_entity_is_not_judged(self):
        # A simulator-driven NPC (controller simulator_autopilot, roadmap 017): SSv2 adopts
        # CARLA's pose, read back before the tick like the ego's, so its SSv2 pose at frame k
        # is CARLA's at k-1 -- one tick behind by protocol, 0.4 m at 8 m/s (baseline: 551
        # frames of G3 on town01_simulator_autopilot's npc_autopilot).
        frames = [frame(i, ("ego", state("ego", 50.0)), ("car", state("car", 0.4 * i, dx=0.4)))
                  for i in range(1, 15)]
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(checks(r, "G3"), [])
        notes = [n for n in info(r, "G3") if n["kind"] == "read_back"]
        self.assertEqual(len(notes), 1)
        self.assertEqual(notes[0]["entities"], ["car"])
        self.assertAlmostEqual(notes[0]["numbers"]["match_fraction"], 1.0, places=6)

    def test_a_moving_puppet_off_its_pose_is_still_judged(self):
        # Moving at 0.4 m per frame, 0.5 m beside its pose: neither on its own pose nor on
        # CARLA's previous one, so a pose divergence, not a read-back.
        frames = [frame(i, ("ego", state("ego", 50.0)), ("car", state("car", 0.4 * i, dy=0.5)))
                  for i in range(1, 15)]
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(len(checks(r, "G3")), 1)
        self.assertEqual([n for n in info(r, "G3") if n["kind"] == "read_back"], [])

    def test_a_read_back_entitys_lag_widens_the_band(self):
        # The car is read back (CARLA 0.4 m ahead of SSv2) and drives 4.4 m ahead of an ego
        # keeping pace: SSv2's boxes overlap 0.1 m, CARLA's are 0.1 m clear. |d_ssv2 -
        # d_carla| = 0.2 m, within the car's lag + touch (0.45): timing, so info; without
        # the car's lag the band would be 0.05 and this a finding.
        frames = [frame(i, ("ego", state("ego", 0.4 * i - 4.4)),
                        ("car", state("car", 0.4 * i, dx=0.4)))
                  for i in range(1, 15)]
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(checks(r, "G1"), [])
        self.assertEqual({n["kind"] for n in info(r, "G1")}, {"lag_band"})

    def test_recovery_right_after_a_contact_hold_is_info(self):
        # The hold ends with the body still off its pose; below the snap distance (0.5 m)
        # the tracker closes the error over a few frames instead of snapping (baseline:
        # town01_rear_contact npc_rear frames 291-293, 0.35 -> 0.2 m).
        frames = self.frames(set(), n=12)
        errs = {3: 1.0, 4: 1.0, 5: 0.6, 6: 0.4, 7: 0.3, 8: 0.25}
        for i, dy in errs.items():
            flags = ("contact_hold",) if i <= 5 else ()
            frames[i - 1].entities["car"] = state("car", 0.0, dy=dy, flags=flags)
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual(checks(r, "G3"), [])
        rec = [n for n in info(r, "G3") if n["kind"] == "hold_recovery"]
        self.assertEqual(len(rec), 1)
        self.assertEqual(rec[0]["frames"], [6, 8])

    def test_a_growing_error_after_a_hold_is_a_finding(self):
        frames = self.frames(set(), n=12)
        for i, dy in {4: 1.0, 5: 0.6, 6: 0.4, 7: 0.5, 8: 0.6}.items():
            flags = ("contact_hold",) if i <= 5 else ()
            frames[i - 1].entities["car"] = state("car", 0.0, dy=dy, flags=flags)
        r = run(make_trace(["ego", "car"], frames))
        self.assertEqual([f["frames"] for f in checks(r, "G3")], [[6, 8]])


# --- G4 -----------------------------------------------------------------------------------

PASS = """<?xml version="1.0"?>
<testsuites><testsuite name="s" tests="2" failures="0" errors="0">
  <testcase name="s" />
  <testcase name="carla_contact"><failure type="CarlaContact" message="hit" /></testcase>
</testsuite></testsuites>
"""
FAIL_COLLISION = """<?xml version="1.0"?>
<testsuites><testsuite name="s" tests="1" failures="1" errors="0">
  <testcase name="s"><failure type="SimulationFailure" message="CustomCommandAction typed
    &quot;exitFailure&quot; was triggered by the Condition named &quot;collision&quot;: Is any
    of [truck] colliding with another given entity ego?" /></testcase>
</testsuite></testsuites>
"""
FAIL_TIMEOUT = """<?xml version="1.0"?>
<testsuites><testsuite name="s" tests="1" failures="1" errors="0">
  <testcase name="s"><failure type="SimulationFailure" message="CustomCommandAction typed
    &quot;exitFailure&quot; was triggered by the Condition named &quot;timeout_condition&quot;:
    Is the simulation time (= 120.05) is greaterThan 120?" /></testcase>
</testsuite></testsuites>
"""


class G4VerdictClass(unittest.TestCase):
    def junit(self, text):
        d = tempfile.mkdtemp()
        p = Path(d) / "result.junit.xml"
        p.write_text(text)
        return go.read_junit(p)

    def clear(self, contacts=()):
        return make_trace(["ego", "truck"], [truck_behind(n, gap=20.0) for n in range(1, 8)],
                          contacts)

    def test_verdicts_read_from_junit(self):
        self.assertEqual(self.junit(PASS)[0], "pass", "carla_contact is not SSv2's verdict")
        self.assertEqual(self.junit(FAIL_COLLISION)[0], "fail_collision")
        self.assertEqual(self.junit(FAIL_TIMEOUT)[0], "fail_other")

    def test_without_junit_the_class_is_unknown(self):
        self.assertEqual(run(self.clear())["g4"]["class"], "unknown")

    def test_pass_with_a_contact_ssv2_cannot_see(self):
        r = run(self.clear([contact(3)]), self.junit(PASS))
        self.assertEqual(r["g4"]["class"], "PASS+contact")
        self.assertIn("C4:PASS+contact", r["signature"])

    def test_pass_with_overlap_only_in_carla(self):
        frames = [frame(n, ("ego", state("ego", 0.0)), ("bike", state("bike", -3.6)))
                  for n in range(1, 4)]
        r = run(make_trace(["ego", "bike"], frames), self.junit(PASS))
        self.assertEqual(r["g4"]["class"], "PASS+overlap-only-in-CARLA")

    def test_collision_failure_without_contact(self):
        r = run(self.clear(), self.junit(FAIL_COLLISION))
        self.assertEqual(r["g4"]["class"], "FAIL(collision)+no-contact")

    def test_collision_failure_with_overlap_only_in_ssv2(self):
        t = make_trace(["ego", "truck"], [truck_behind(n) for n in range(1, 6)])
        r = run(t, self.junit(FAIL_COLLISION))
        self.assertEqual(r["g4"]["class"], "FAIL(collision)+overlap-only-in-SSv2")
        self.assertEqual(r["g4"]["classes"],
                         ["FAIL(collision)+no-contact", "FAIL(collision)+overlap-only-in-SSv2"])

    def test_agree(self):
        self.assertEqual(run(self.clear(), self.junit(PASS))["g4"]["class"], "agree")
        self.assertEqual(run(self.clear(), self.junit(FAIL_TIMEOUT))["g4"]["class"], "agree")
        # A collision both worlds saw: overlap in both, a contact inside it.
        t = make_trace(["ego", "truck"], [truck_behind(n, gap=5.0) for n in range(1, 6)],
                       [contact(3, "truck")])
        r = run(t, self.junit(FAIL_COLLISION))
        self.assertEqual(r["g4"]["class"], "agree")
        self.assertNotIn("C4", " ".join(r["signature"]))


# --- CLI ----------------------------------------------------------------------------------


class Cli(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())

    def write(self, trace, name="t.jsonl"):
        p = self.dir / name
        p.write_text(gt.dumps(trace))
        return p

    def test_findings_are_data_exit_zero(self):
        t = make_trace(["ego", "truck"], [truck_behind(n) for n in range(1, 6)])
        out = self.dir / "findings.json"
        junit = self.dir / "result.junit.xml"
        junit.write_text(FAIL_COLLISION)
        buf = io.StringIO()
        with redirect_stdout(buf):
            code = go.main([str(self.write(t)), "--junit", str(junit), "--out", str(out)])
        self.assertEqual(code, 0)
        line = buf.getvalue().strip()
        self.assertEqual(len(line.splitlines()), 1)
        self.assertIn("G1 1", line)
        self.assertIn("FAIL(collision)+overlap-only-in-SSv2", line)
        doc = json.loads(out.read_text())
        self.assertEqual(doc["schema"], go.SCHEMA)
        self.assertEqual(doc["summary"], line)
        self.assertEqual(doc["counts"]["G1"], 1)
        self.assertEqual(doc["tolerances"]["pos"], 0.2)
        self.assertAlmostEqual(doc["margins"]["min_margin_difference"], -3.55, places=6)
        for f in doc["findings"]:
            self.assertTrue({"check", "entities", "frames", "numbers"} <= set(f))

    def test_unreadable_trace_is_non_zero(self):
        bad = self.dir / "bad.jsonl"
        bad.write_text('{"type": "frame"}\n')
        with redirect_stdout(io.StringIO()):
            self.assertNotEqual(go.main([str(bad)]), 0)
            self.assertNotEqual(go.main([str(self.dir / "missing.jsonl")]), 0)

    def test_a_trace_cut_short_is_judged_and_marked(self):
        t = make_trace(["ego", "truck"], [truck_behind(n) for n in range(1, 6)], end=False)
        p = self.dir / "cut.jsonl"
        p.write_text(gt.dumps(t))
        out = self.dir / "f.json"
        with redirect_stdout(io.StringIO()):
            self.assertEqual(go.main([str(p), "--out", str(out)]), 0)
        doc = json.loads(out.read_text())
        self.assertFalse(doc["complete"])
        self.assertEqual(doc["counts"]["G1"], 1)


if __name__ == "__main__":
    unittest.main()
