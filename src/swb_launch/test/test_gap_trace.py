"""Unit tests for gap_trace, the gap trace reader (roadmap 024 step 3).

No ROS, no CARLA:

    python3 -m pytest src/swb_launch/test/test_gap_trace.py
"""

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import gap_trace as gt  # noqa: E402

EPISODE = {
    "type": "episode",
    "schema": 1,
    "episode": 3,
    "start_unix": 1760000000.25,
    "npc_motion": "tracked",
}
EGO = {
    "type": "entity",
    "frame": 0,
    "name": "ego",
    "kind": "ego",
    "declared": {"center": [1.5, 0.0, 0.9], "dimensions": [4.9, 1.9, 1.8]},
    "origin_offset": [1.5, 0.0, 0.0],
    "actor_id": 211,
    "blueprint": "vehicle.lincoln.mkz_2020",
    "blueprint_box": {"location": [0.0, 0.0, 0.75], "extent": [2.45, 0.95, 0.75]},
}
TRUCK = {
    "type": "entity",
    "frame": 0,
    "name": "truck",
    "kind": "vehicle",
    "declared": {"center": [4.0, 0.0, 1.5], "dimensions": [12.0, 2.5, 3.0]},
    "origin_offset": [4.0, 0.0, 0.0],
    "actor_id": 212,
    "blueprint": "vehicle.tesla.model3",
    "blueprint_box": {"location": [0.0, 0.0, 0.72], "extent": [2.35, 1.05, 0.72]},
}
BARRIER = {
    "type": "entity",
    "frame": 1,
    "name": "barrier",
    "kind": "misc_object",
    "declared": {"center": [0.0, 0.0, 0.5], "dimensions": [1.21, 0.37, 1.0]},
    "origin_offset": [0.0, 0.0, 1.5707963267948966],
    "actor_id": 213,
    "blueprint": "static.prop.streetbarrier",
    "blueprint_box": None,
}
FRAME0 = {
    "type": "frame",
    "frame": 0,
    "sim_time": 0.0,
    "entities": {
        "ego": {
            "ssv2": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "carla": [1.5, 0.0, 0.02, 0.0, 0.0, 0.0],
            "velocity": [0.0, 0.0, 0.0],
            "flags": ["spawn"],
        },
        "truck": {
            "ssv2": [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "carla": [24.0, 0.0, 0.03, 0.0, 0.0, 0.0],
            "velocity": [0.0, 0.0, 0.0],
            "flags": ["spawn"],
        },
    },
}
FRAME1 = {
    "type": "frame",
    "frame": 1,
    "sim_time": 0.05,
    "entities": {
        "ego": {
            "ssv2": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            "carla": [1.6, 0.0, 0.02, 0.0, 0.0, 0.0],
            "velocity": [2.0, 0.0, 0.0],
        },
        "truck": {
            "ssv2": [20.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            "carla": [24.2, 0.0, 0.03, 0.0, 0.0, 0.0],
            "velocity": [4.0, 0.0, 0.0],
            "flags": ["contact_hold"],
        },
        "barrier": {
            "ssv2": [30.0, 3.0, 0.0, 0.0, 0.0, 0.3],
            "carla": None,
            "velocity": None,
            "flags": ["spawn"],
        },
    },
}
CONTACT_NPC = {
    "type": "contact",
    "frame": 1,
    "carla_frame": 5021,
    "sim_time": 0.05,
    "ego_id": 211,
    "other_id": 212,
    "name": "truck",
    "other_type": "vehicle.tesla.model3",
    "other_kind": "vehicle",
    "impulse": 812.5,
}
CONTACT_STATIC = {
    "type": "contact",
    "frame": 1,
    "carla_frame": 5021,
    "sim_time": 0.05,
    "ego_id": 211,
    "other_id": None,
    "name": None,
    "other_type": "static.pole",
    "other_kind": "static",
    "impulse": 30.0,
}
END = {
    "type": "end",
    "frame": 1,
    "sim_time": 0.05,
    "frames": 2,
    "dropped_frames": 0,
    "complete": True,
}

SAMPLE = [EPISODE, EGO, TRUCK, FRAME0, BARRIER, FRAME1, CONTACT_NPC, CONTACT_STATIC, END]


def lines(records):
    return [json.dumps(r) for r in records]


class ValidTrace(unittest.TestCase):
    def test_loads_typed_records(self):
        trace = gt.load_trace(lines(SAMPLE))
        self.assertIsInstance(trace.episode, gt.Episode)
        self.assertEqual(trace.episode.schema, 1)
        self.assertEqual(trace.episode.npc_motion, "tracked")
        self.assertEqual(list(trace.entities), ["ego", "truck", "barrier"])
        truck = trace.entities["truck"]
        self.assertIsInstance(truck, gt.EntityHeader)
        self.assertEqual(truck.kind, "vehicle")
        self.assertEqual(truck.declared.dimensions, (12.0, 2.5, 3.0))
        self.assertEqual(truck.origin_offset, gt.OriginOffset(4.0, 0.0, 0.0))
        self.assertEqual(truck.blueprint_box.extent, (2.35, 1.05, 0.72))
        self.assertIsNone(trace.entities["barrier"].blueprint_box)
        self.assertEqual([f.frame for f in trace.frames], [0, 1])
        ego0 = trace.frames[0].entities["ego"]
        self.assertIsInstance(ego0.ssv2, gt.Pose6)
        self.assertEqual(ego0.carla.x, 1.5)
        self.assertTrue(ego0.spawn)
        self.assertFalse(ego0.contact_hold)
        truck1 = trace.frames[1].entities["truck"]
        self.assertTrue(truck1.contact_hold)
        self.assertEqual(truck1.velocity, (4.0, 0.0, 0.0))
        self.assertIsNone(trace.frames[1].entities["barrier"].carla)
        self.assertEqual(len(trace.contacts), 2)
        self.assertEqual(trace.contacts[1].other_kind, "static")
        self.assertIsNone(trace.contacts[1].other_id)
        self.assertEqual(trace.end.dropped_frames, 0)

    def test_round_trips(self):
        trace = gt.load_trace(lines(SAMPLE))
        out = [gt.to_json(r) for r in trace.records()]
        self.assertEqual(out, [json.loads(line) for line in lines(SAMPLE)])
        self.assertEqual(gt.load_trace(gt.dumps(trace).splitlines()), trace)

    def test_kinematic_flag(self):
        # Roadmap 032: a tracked NPC handed to kinematic after a blocked snap. Added under
        # schema 1 -- traces without it still read (SAMPLE has none).
        trace = gt.load_trace(
            mutate(5, lambda r: r["entities"]["truck"].update(flags=["kinematic"])))
        truck = trace.frames[1].entities["truck"]
        self.assertTrue(truck.kinematic)
        self.assertFalse(truck.contact_hold)
        self.assertFalse(trace.frames[1].entities["ego"].kinematic)
        self.assertEqual(gt.load_trace(gt.dumps(trace).splitlines()), trace)

    def test_subject_marks_a_contact_an_npc_sensor_saw(self):
        # Roadmap 032 step 5: a tracked NPC's own collision sensor. Added under schema 1 --
        # a line without `subject` is the ego's (SAMPLE has none), and writes back without it.
        trace = gt.load_trace(mutate(7, lambda r: r.update(subject="truck")))
        self.assertEqual(trace.contacts[1].subject, "truck")
        self.assertIsNone(trace.contacts[0].subject)
        self.assertEqual(gt.load_trace(gt.dumps(trace).splitlines()), trace)
        self.assertNotIn("subject", gt.to_json(trace.contacts[0]))
        # null reads as absent.
        trace = gt.load_trace(mutate(7, lambda r: r.update(subject=None)))
        self.assertIsNone(trace.contacts[1].subject)

    def test_carla_frame_of_a_frame_line(self):
        # Roadmap 032 N2: the CARLA frame of the snapshot behind a frame line, so a contact's
        # carla_frame can be checked against its label. Optional under schema 1 (SAMPLE has
        # none), and written back only when present.
        self.assertIsNone(gt.load_trace(lines(SAMPLE)).frames[1].carla_frame)
        trace = gt.load_trace(mutate(5, lambda r: r.update(carla_frame=5021)))
        self.assertEqual(trace.frames[1].carla_frame, 5021)
        self.assertEqual(gt.to_json(trace.frames[1])["carla_frame"], 5021)
        self.assertNotIn("carla_frame", gt.to_json(trace.frames[0]))
        self.assertEqual(gt.load_trace(gt.dumps(trace).splitlines()), trace)

    def test_omitted_flags_are_empty(self):
        trace = gt.load_trace(lines(SAMPLE))
        self.assertEqual(trace.frames[1].entities["ego"].flags, ())

    def test_reads_a_file_and_skips_blank_lines(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "t.jsonl"
            path.write_text("\n".join(lines(SAMPLE)) + "\n\n")
            trace = gt.load_trace(path)
        self.assertEqual(trace.end.frames, 2)

    def test_iter_records_streams_in_order(self):
        kinds = [type(r).__name__ for r in gt.iter_records(lines(SAMPLE))]
        self.assertEqual(
            kinds,
            ["Episode", "EntityHeader", "EntityHeader", "Frame", "EntityHeader", "Frame",
             "Contact", "Contact", "End"],
        )

    def test_missing_end_is_allowed_when_asked(self):
        trace = gt.load_trace(lines(SAMPLE[:-1]), require_end=False)
        self.assertIsNone(trace.end)

    def test_respawn_replaces_the_header(self):
        again = dict(TRUCK, frame=1, actor_id=250)
        trace = gt.load_trace(lines([EPISODE, EGO, TRUCK, FRAME0, again, END]))
        self.assertEqual(trace.entities["truck"].actor_id, 250)


def mutate(index, change):
    """SAMPLE with record `index` changed by `change(record)`; returns the JSON lines."""
    records = copy.deepcopy(SAMPLE)
    change(records[index])
    return lines(records)


class MalformedTrace(unittest.TestCase):
    def assertTraceError(self, text_lines, line, field=None, **kw):
        with self.assertRaises(gt.TraceError) as cm:
            gt.load_trace(text_lines, **kw)
        err = cm.exception
        self.assertEqual(err.line, line, str(err))
        self.assertIn(f"line {line}", str(err))
        if field is not None:
            self.assertEqual(err.field, field, str(err))
            self.assertIn(field, str(err))
        return err

    def test_not_json(self):
        text = lines(SAMPLE)
        text[3] = '{"type": "frame", "frame": 0,'
        self.assertTraceError(text, 4)

    def test_not_an_object(self):
        text = lines(SAMPLE)
        text[2] = "[1, 2]"
        self.assertTraceError(text, 3)

    def test_unknown_type(self):
        self.assertTraceError(mutate(2, lambda r: r.update(type="entiti")), 3, "type")

    def test_first_line_is_not_episode(self):
        self.assertTraceError(lines(SAMPLE[1:]), 1, "type")

    def test_unsupported_schema(self):
        err = self.assertTraceError(mutate(0, lambda r: r.update(schema=2)), 1, "schema")
        self.assertIn("2", str(err))

    def test_second_episode(self):
        text = lines(SAMPLE)
        text.insert(3, json.dumps(EPISODE))
        self.assertTraceError(text, 4, "type")

    def test_missing_field(self):
        self.assertTraceError(mutate(1, lambda r: r.pop("actor_id")), 2, "actor_id")

    def test_wrong_type(self):
        self.assertTraceError(mutate(1, lambda r: r.update(actor_id="211")), 2, "actor_id")

    def test_bool_is_not_a_number(self):
        self.assertTraceError(mutate(6, lambda r: r.update(impulse=True)), 7, "impulse")

    def test_not_finite(self):
        text = lines(SAMPLE)
        text[6] = text[6].replace("812.5", "NaN")
        self.assertTraceError(text, 7, "impulse")

    def test_unknown_kind(self):
        self.assertTraceError(mutate(2, lambda r: r.update(kind="truck")), 3, "kind")

    def test_bad_vector_length(self):
        self.assertTraceError(
            mutate(2, lambda r: r["declared"].update(dimensions=[12.0, 2.5])),
            3,
            "declared.dimensions",
        )

    def test_negative_dimension(self):
        self.assertTraceError(
            mutate(2, lambda r: r["blueprint_box"].update(extent=[2.35, -1.05, 0.72])),
            3,
            "blueprint_box.extent[1]",
        )

    def test_bad_pose_in_frame(self):
        self.assertTraceError(
            mutate(5, lambda r: r["entities"]["truck"].update(ssv2=[20.2, 0.0, 0.0])),
            6,
            "entities.truck.ssv2",
        )

    def test_carla_without_velocity(self):
        self.assertTraceError(
            mutate(5, lambda r: r["entities"]["truck"].update(velocity=None)),
            6,
            "entities.truck.velocity",
        )

    def test_unknown_flag(self):
        self.assertTraceError(
            mutate(5, lambda r: r["entities"]["truck"].update(flags=["hold"])),
            6,
            "entities.truck.flags",
        )

    def test_undeclared_entity_in_frame(self):
        self.assertTraceError(
            mutate(3, lambda r: r["entities"].update(ghost=r["entities"]["ego"])),
            4,
            "entities.ghost",
        )

    def test_frames_go_backwards(self):
        self.assertTraceError(mutate(5, lambda r: r.update(frame=0)), 6, "frame")

    def test_unknown_contact_kind(self):
        self.assertTraceError(mutate(6, lambda r: r.update(other_kind="wall")), 7, "other_kind")

    def test_static_contact_with_actor_id(self):
        self.assertTraceError(mutate(7, lambda r: r.update(other_id=99)), 8, "other_id")

    def test_carla_frame_must_be_a_frame_number(self):
        self.assertTraceError(mutate(5, lambda r: r.update(carla_frame=-1)), 6, "carla_frame")
        self.assertTraceError(mutate(5, lambda r: r.update(carla_frame="x")), 6, "carla_frame")

    def test_subject_must_be_a_name(self):
        self.assertTraceError(mutate(7, lambda r: r.update(subject=5)), 8, "subject")
        self.assertTraceError(mutate(7, lambda r: r.update(subject="")), 8, "subject")

    def test_negative_impulse(self):
        self.assertTraceError(mutate(6, lambda r: r.update(impulse=-1.0)), 7, "impulse")

    def test_negative_dropped_frames(self):
        self.assertTraceError(mutate(8, lambda r: r.update(dropped_frames=-1)), 9, "dropped_frames")

    def test_record_after_end(self):
        text = lines(SAMPLE) + [json.dumps(CONTACT_NPC)]
        self.assertTraceError(text, 10, "type")

    def test_missing_end(self):
        # Reported at the line after the last one: where the end record should be.
        self.assertTraceError(lines(SAMPLE[:-1]), 9, "type")

    def test_empty(self):
        self.assertTraceError([], 1, "type")

    def test_error_names_the_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "bad.jsonl"
            path.write_text("\n".join(mutate(1, lambda r: r.pop("kind"))) + "\n")
            with self.assertRaises(gt.TraceError) as cm:
                gt.load_trace(path)
        self.assertIn("bad.jsonl line 2", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
