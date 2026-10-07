"""Offline post-contact lift/review contract; no robot or camera connection."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import test_guarded_contact_retreat as retreat

ROOT = Path(__file__).parents[1]
RUN = ROOT / "runs/cola_on_cup_post_contact_20261006_233500"
SPEC = importlib.util.spec_from_file_location(
    "post_contact_under_test", ROOT / "scripts/ros_guarded_post_contact_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)
ANCHOR = [53018, 109860, -48249, 0, -52889, 0]


class PostContactTests(retreat.RetreatTests):
    def setUp(self):
        super().setUp()
        self.control.feedback.close()
        self.piper.q = [53007, 109848, -48210, 0, -52872, 0]
        self.piper.goal = list(self.piper.q)
        self.piper.jaw = .05747
        self.session.update(generation=18, regrasp_stage="task", contact_stage="lifting",
                            held_raw=list(ANCHOR), scope_anchor_raw=list(ANCHOR),
                            lifting_completed=0, lifting_attempted=False,
                            post_contact_reviews=[])
        def register(name, callback):
            self.services[name] = callback
            return name
        self.control = entry.PostContactTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            retreat.readiness.recorded.wide.wide_limits(),
            retreat.readiness.recorded.probe.fixtures.fk,
            self.store, self.session, self.path, {"offline": True}, lambda: None,
            self.clock, service_factory=register, namespace="/offline_post_contact",
            token="post_contact_offline")
        self.addCleanup(self.control.feedback.close)

    def lift_target(self, amount=1300):
        raw = list(self.session["scope_anchor_raw"])
        raw[1] -= amount
        return raw

    def lift(self, amount=1300):
        return self.control.execute(self.target(self.lift_target(amount)))

    def write_contact_review(self, next_stage="lifting", clearance=False):
        path, material, source = self.write_review(next_stage=next_stage, contact_absent=clearance)
        material.update(stage="post_contact_lift", grasp_retained=True,
                        clearance_visible=clearance)
        raw_path = Path(material["raw_review"]["path"])
        raw_review = json.loads(raw_path.read_text())
        raw_review.update(generation=self.session["generation"], guard_checks_clean=True,
                          transport_violations=[], all_14_result_feedback_frames_matched_exactly=True,
                          target_raw=self.control.status()["receipts"][0]["target_raw"])
        raw_path.write_text(json.dumps(raw_review))
        material["raw_review"]["sha256"] = entry.sha(raw_path)
        for image in material["images"]:
            image["captured_at_unix_s"] = max(self.control.status()["result"]["after"]["stamps"])+.01
        path.write_text(json.dumps(material))
        return path, material, source

    def approve_lift(self, next_stage="lifting", clearance=False):
        self.write_contact_review(next_stage, clearance)
        before = len(self.piper.frames)
        started = self.clock.monotonic()
        result = self.control.review_contact(*self.review_args())
        self.assertTrue(result[0])
        self.assertEqual(len(self.piper.frames), before)
        self.assertGreaterEqual(self.clock.monotonic()-started, 3.)
        return result

    def test_only_bounded_j2_decrease_and_no_jaw_hold_or_resume(self):
        self.control.baseline()
        self.assertEqual(self.piper.frames, [])
        for amount in (0, -1, 1301):
            with self.subTest(amount=amount), self.assertRaises(RuntimeError):
                self.lift(amount)
        for axis in (0, 2, 3, 4, 5):
            raw = self.lift_target()
            raw[axis] += 1
            with self.subTest(axis=axis), self.assertRaises(RuntimeError):
                self.control.execute(self.target(raw))
        for call in (lambda: self.control.gripper(types.SimpleNamespace()),
                     lambda: self.control.send_hold(self.piper.snapshot()),
                     lambda: self.control.resume(0, 18, "invalid")):
            with self.assertRaises(RuntimeError):
                call()
        self.assertFalse(self.control.request_hold(0, self.control.token)[0])
        self.assertEqual(self.piper.frames, [])
        # Still inside original .003 rad drift/box, but final fresh-to-target
        # movement would exceed this narrower 1300 raw clearance-lift cap.
        record = self.control.record
        changed = []
        def move_after_intent(event, **fields):
            result = record(event, **fields)
            if event == "intent":
                self.piper.q[1] = ANCHOR[1]+1
                self.piper.goal = list(self.piper.q)
                changed.append(True)
            return result
        with patch.object(self.control, "record", side_effect=move_after_intent):
            with self.assertRaisesRegex(RuntimeError, "1300millidegrees"):
                self.lift()
        self.assertEqual(changed, [True])
        self.assertEqual(self.piper.frames, [])
        self.assertIsNotNone(self.session["failure"])

    def test_one_lift_is_four_frames_then_revokes_adoption_until_review(self):
        target = self.lift_target()
        result = self.lift()
        self.assertEqual(result["phase"], "completed")
        self.assertEqual(self.piper.frames, entry.support.frames_for(target))
        self.assertEqual(self.piper.jaw, .05747)
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.session["contact_stage"], "lift_review")
        self.assertGreaterEqual(result["stable_window"]["duration_s"], 3.)
        with self.assertRaises(RuntimeError):
            self.lift()
        self.assertEqual(len(self.piper.frames), 4)

    def test_three_lifts_each_require_new_review_and_task_requires_visible_clearance(self):
        for count in range(1, 4):
            self.lift()
            self.assertEqual(self.session["lifting_completed"], count)
            self.assertFalse(self.node.adopted)
            args = self.review_args()
            if count < 3:
                self.approve_lift()
                self.assertEqual(self.session["contact_stage"], "lifting")
                with self.assertRaises(RuntimeError):
                    self.control.review_contact(*args)
            else:
                self.write_contact_review()
                with self.assertRaises(RuntimeError):
                    self.control.review_contact(*args)
                self.write_contact_review(next_stage="task", clearance=False)
                with self.assertRaises(RuntimeError):
                    self.control.review_contact(*args)
                self.approve_lift(next_stage="task", clearance=True)
                self.assertEqual(self.session["contact_stage"], "task")
                self.assertTrue(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 12)

    def test_review_requires_bound_raw_new_images_grasp_and_fresh_unchanged_state(self):
        self.lift()
        args = self.review_args()
        path, material, source = self.write_contact_review()
        for field, value in (("grasp_retained", False), ("generation", -1),
                             ("completed_result_sha256", "wrong")):
            changed = copy.deepcopy(material)
            changed[field] = value
            path.write_text(json.dumps(changed))
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                self.control.review_contact(*args)
            self.assertFalse(self.node.adopted)
        changed = copy.deepcopy(material)
        changed["images"][0]["captured_at_unix_s"] = 0.
        path.write_text(json.dumps(changed))
        with self.assertRaises(RuntimeError):
            self.control.review_contact(*args)
        path.write_text(json.dumps(material))
        original = source.read_bytes()
        source.write_bytes(b"changed raw review source")
        with self.assertRaises(RuntimeError):
            self.control.review_contact(*args)
        source.write_bytes(original)
        self.piper.jaw += .0006
        with self.assertRaises(RuntimeError):
            self.control.review_contact(*args)
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 4)

    def test_partial_dispatch_is_not_reviewable_or_repeatable(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            self.lift()
        self.assertEqual(len(self.piper.frames), 1)
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        with self.assertRaises(RuntimeError):
            self.lift()
        with self.assertRaises(RuntimeError):
            self.control.review_contact(1, self.session["generation"], "invalid")
        self.assertEqual(len(self.piper.frames), 1)

    def test_rx_bad_then_good_never_restores_lift_permission(self):
        fixture = retreat.readiness.recorded.probe.rx_fixtures
        good = list(self.piper.q)
        bad = list(good)
        bad[2] = 1
        for raw in (bad, good):
            self.piper.ParseCANFrame(fixture.message(
                *fixture.joint_fragment(2, raw), self.clock.time()))
        with self.assertRaises(RuntimeError):
            self.lift()
        self.assertEqual(self.piper.frames, [])
        self.assertIsNotNone(self.piper.rx_latch.first_fault)

    def test_held_jaw_baseline_change_blocks_lift_without_send(self):
        self.piper.jaw += .0006
        with self.assertRaises(RuntimeError):
            self.lift()
        self.assertEqual(self.piper.frames, [])
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)

    def test_lifting_readiness_failure_is_hard_even_with_zero_frames(self):
        with patch.object(self.control, "baseline", side_effect=
                retreat.readiness.entry.stability.BaselineNotReady("offline unstable baseline")):
            with self.assertRaises(RuntimeError):
                self.lift()
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])
        with self.assertRaises(RuntimeError):
            self.lift()
        self.assertEqual(self.piper.frames, [])


class HandoffTests(retreat.readiness.recorded.wide.OfflineTests):
    def test_real_clean_retreat_and_thirteen_parent_bytes_preserved_with_unique_child(self):
        reviewed = json.loads((RUN / "reviewed_post_contact.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        session_root = entry.guard.v1.SESSION_ROOT
        parent = json.loads((session_root / entry.predecessor.child_name(boot)).read_text())
        endpoint = entry.reviewed_evidence(reviewed, parent)
        self.assertEqual(endpoint["raw_q"], [53007, 109848, -48210, 0, -52872, 0])
        self.assertEqual(endpoint["opening_m"], .05747)
        self.assertFalse(parent["status"]["result"]["ordinary_task_authorized"])
        for corrupt in ("partial", "raw_fault"):
            changed = copy.deepcopy(parent)
            if corrupt == "partial":
                changed["status"]["receipts"][0]["socket_send_returns"] = 3
            else:
                changed["raw_feedback_fault"] = {"reason": "synthetic new fault"}
            with self.subTest(corrupt=corrupt), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(reviewed, changed)
        readiness = retreat.readiness
        previous = readiness.recorded.wide.entry.previous
        interior = previous.motion.predecessor
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME, interior.CHILD_NAME, previous.motion.frozen.child_name(boot),
            previous.motion.child_name(boot), previous.release.child_name(boot),
            readiness.recorded.wide.entry.child_name(boot), readiness.recorded.relief.child_name(boot),
            readiness.entry.predecessor.child_name(boot), readiness.entry.child_name(boot),
            entry.predecessor.child_name(boot)]
        self.assertEqual(len(set(names)), 13)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (session_root/name).read_bytes() for name in names}
            for name, raw in originals.items():
                (root/name).write_bytes(raw)
            with entry.reserve(boot, reviewed, session_root=root) as (_, measured, store, session):
                self.assertEqual(measured["opening_m"], .05747)
                self.assertEqual(session["contact_stage"], "lifting")
                self.assertEqual(session["lifting_completed"], 0)
                self.assertFalse(session["lifting_attempted"])
                self.assertFalse(session["ordinary_task_authorized"])
                self.assertFalse(session["jaw_authorized"])
                self.assertEqual(session["original_contact_raw_fault"], parent["loaded_contact_raw_fault"])
                self.assertEqual(session["original_contact_failure"], parent["loaded_contact_failure"])
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, reviewed, session_root=root):
                    pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


for _name in dir(retreat.RetreatTests):
    if _name.startswith("test_") and _name not in PostContactTests.__dict__:
        setattr(PostContactTests, _name, None)


if __name__ == "__main__":
    unittest.main()
