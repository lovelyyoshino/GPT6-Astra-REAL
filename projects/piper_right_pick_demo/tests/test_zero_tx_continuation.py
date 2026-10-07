"""Pure fake/file checks for exact zero-TX continuation; no hardware access."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import test_guarded_post_contact as post

ROOT = Path(__file__).parents[1]
RUN = ROOT / "runs/cola_on_cup_zero_tx_continuation_20261007_010000"
SPEC = importlib.util.spec_from_file_location(
    "zero_tx_continuation_under_test", ROOT / "scripts/ros_zero_tx_continuation_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


class ContinuationTests(post.PostContactTests):
    def setUp(self):
        super().setUp()
        self.control.feedback.close()
        self.piper.q = [53007, 103863, -48210, 0, -39848, 0]
        self.piper.goal = list(self.piper.q)
        self.piper.jaw = .05747
        self.session.update(generation=20, contact_stage="task", regrasp_stage="task",
            held_raw=list(entry.LAST_TARGET), scope_anchor_raw=list(entry.LAST_TARGET),
            preflight_refusals=[], zero_tx_refusals=[], first_segment_verified=True,
            ordinary_task_authorized=True, jaw_authorized=True)
        def register(name, callback):
            self.services[name] = callback
            return name
        self.control = entry.ZeroTXContinuationTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            post.retreat.readiness.recorded.wide.wide_limits(),
            post.retreat.readiness.recorded.probe.fixtures.fk,
            self.store, self.session, self.path, {"offline": True}, lambda: None,
            self.clock, service_factory=register, namespace="/offline_zero_tx",
            token="zero_tx_offline")
        self.addCleanup(self.control.feedback.close)

    def j5_message(self, increment=150):
        raw = list(self.piper.q)
        raw[4] += increment
        return self.target(raw)

    def baseline_j5_drift(self):
        baseline = self.control.baseline
        def drift():
            baseline()
            self.piper.q[4] -= 160  # < .003 rad, but +150 target now exceeds 200 raw.
            self.piper.goal = list(self.piper.q)
            return self.control.read()
        return drift

    def test_initial_baseline_and_close_are_zero_tx_and_201raw_remains_forbidden(self):
        started = self.clock.monotonic()
        self.control.baseline()
        self.assertGreaterEqual(self.clock.monotonic()-started, 3.)
        with self.assertRaises(entry.PreDispatchJ5Refusal):
            self.control.execute(self.j5_message(201))
        self.assertEqual(self.piper.frames, [])
        self.assertEqual(self.piper.jaw, .05747)
        self.assertEqual(self.control.limits["max_rotation_step_rad"], .05)
        self.assertEqual(self.control.limits["max_translation_step_m"], .03)
        self.assertEqual(self.control.limits["joint_tracking_margin_rad"][4], entry.profile.MARGINS[4])
        self.control.feedback.close()
        self.assertEqual(self.piper.frames, [])

    def test_typed_baseline_cap_refusal_is_zero_tx_and_only_new_request_can_send(self):
        target = self.j5_message()
        with patch.object(self.control, "baseline", side_effect=self.baseline_j5_drift()):
            with self.assertRaises(entry.PreDispatchJ5Refusal):
                self.control.execute(target)
        self.assertEqual(self.control.status()["phase"], "rejected_preflight")
        self.assertIsNone(self.session["failure"])
        self.assertIsNone(self.session["pending"])
        self.assertTrue(self.node.adopted)
        self.assertFalse(self.node.active)
        self.assertEqual(self.piper.frames, [])
        refused = json.loads((self.path/"action_000001_preflight_refusal.json").read_text())
        self.assertEqual(refused["actuator_frames"], 0)
        self.assertFalse(refused["accepted_target_may_continue"])
        self.assertTrue(refused["new_explicit_request_required"])
        self.assertEqual(self.control.execute(self.j5_message())["phase"], "completed")
        self.assertEqual(len(self.piper.frames), 4)
        self.assertEqual(len(self.session["zero_tx_refusals"]), 1)

    def test_same_error_text_as_plain_runtime_error_remains_hard_failure(self):
        with patch.object(self.control, "baseline", side_effect=
                RuntimeError("J5 command increment exceeds200millidegrees")):
            with self.assertRaises(RuntimeError):
                self.control.execute(self.j5_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])
        with self.assertRaises(RuntimeError):
            self.control.execute(self.j5_message())

    def test_partial_dispatch_cannot_be_reclassified_or_retried(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            self.control.execute(self.j5_message())
        self.assertEqual(len(self.piper.frames), 1)
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        with self.assertRaises(RuntimeError):
            self.control.execute(self.j5_message())
        self.assertEqual(len(self.piper.frames), 1)

    def test_typed_error_after_dispatch_keeps_hard_failure_and_receipt(self):
        with patch.object(self.control, "wait_arrival", side_effect=
                entry.PreDispatchJ5Refusal("synthetic late typed exception")):
            with self.assertRaises(entry.PreDispatchJ5Refusal):
                self.control.execute(self.j5_message())
        self.assertEqual(len(self.piper.frames), 4)
        self.assertIsNotNone(self.session["failure"])
        self.assertTrue(self.session["failure"]["accepted_target_may_continue"])
        self.assertEqual(self.session["preflight_refusals"], [])
        with self.assertRaises(RuntimeError):
            self.control.execute(self.j5_message())

    def test_rx_fault_wins_over_typed_zero_tx_refusal(self):
        fixture = post.retreat.readiness.recorded.probe.rx_fixtures
        def fault():
            good = list(self.piper.q)
            bad = list(good)
            bad[2] = 1
            for raw in (bad, good):
                self.piper.ParseCANFrame(fixture.message(
                    *fixture.joint_fragment(2, raw), self.clock.time()))
            raise entry.PreDispatchJ5Refusal("fault must not soften")
        with patch.object(self.control, "baseline", side_effect=fault):
            with self.assertRaises(entry.PreDispatchJ5Refusal):
                self.control.execute(self.j5_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertIsNotNone(self.session["raw_feedback_fault"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])

    def test_refusal_persistence_error_is_hard_failure(self):
        record = self.control.record
        def fail_journal(event, **fields):
            if event == "rejected_preflight":
                raise OSError("synthetic refusal journal failure")
            return record(event, **fields)
        with patch.object(self.control, "record", side_effect=fail_journal), patch.object(
                self.control, "baseline", side_effect=self.baseline_j5_drift()):
            with self.assertRaisesRegex(OSError, "journal failure"):
                self.control.execute(self.j5_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])


class HandoffTests(post.retreat.readiness.recorded.wide.OfflineTests):
    def test_only_exact_zero_tx_evidence_and_unique_child_preserve_fourteen_parents(self):
        reviewed = json.loads((RUN/"reviewed_zero_tx.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        session_root = entry.guard.v1.SESSION_ROOT
        parent = json.loads((session_root/entry.predecessor.child_name(boot)).read_text())
        endpoint = entry.reviewed_evidence(reviewed, parent)
        self.assertEqual(endpoint["raw_q"], [53007, 103863, -48210, 0, -39848, 0])
        self.assertEqual(parent["status"]["sequence"], 93)
        self.assertEqual(parent["status"]["phase"], "failed")
        self.assertEqual(parent["status"]["receipts"], [])
        for field in ("source", "token", "sequence", "target", "partial", "rx", "attempt"):
            changed = copy.deepcopy(parent)
            if field == "source":
                changed["identity"]["source_sha256"] = "wrong"
            elif field == "token":
                changed["identity"]["adoption_token"] = "wrong"
            elif field == "sequence":
                changed["status"]["sequence"] = 94
            elif field == "target":
                changed["status"]["target_raw"][4] += 1
            elif field == "partial":
                changed["status"]["receipts"] = [{"attempted_frames": 1, "socket_send_returns": 0}]
            elif field == "rx":
                changed["raw_feedback_fault"] = {"reason": "synthetic fault"}
            else:
                changed["pending"]["attempted_frames"] = 1
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(reviewed, changed)
        changed_review = copy.deepcopy(reviewed)
        changed_review["zero_tx_continuation_raw_review_sha256"] = "wrong"
        with self.assertRaises(RuntimeError):
            entry.reviewed_evidence(changed_review, parent)
        readiness = post.retreat.readiness
        previous = readiness.recorded.wide.entry.previous
        interior = previous.motion.predecessor
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME, interior.CHILD_NAME, previous.motion.frozen.child_name(boot),
            previous.motion.child_name(boot), previous.release.child_name(boot),
            readiness.recorded.wide.entry.child_name(boot), readiness.recorded.relief.child_name(boot),
            readiness.entry.predecessor.child_name(boot), readiness.entry.child_name(boot),
            post.entry.predecessor.child_name(boot), entry.predecessor.child_name(boot)]
        self.assertEqual(len(set(names)), 14)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (session_root/name).read_bytes() for name in names}
            for name, raw in originals.items():
                (root/name).write_bytes(raw)
            with entry.reserve(boot, reviewed, session_root=root) as (_, current, store, session):
                self.assertEqual(current["raw_q"], endpoint["raw_q"])
                self.assertEqual(session["contact_stage"], "task")
                self.assertTrue(session["no_automatic_goal_replay"])
                self.assertEqual(session["prior_zero_tx_failure"], parent["failure"])
                self.assertEqual(session["post_contact_reviews"], parent["post_contact_reviews"])
                self.assertEqual(session["zero_tx_refusals"], [])
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, reviewed, session_root=root):
                    pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


for _name in dir(post.PostContactTests):
    if _name.startswith("test_") and _name not in ContinuationTests.__dict__:
        setattr(ContinuationTests, _name, None)


if __name__ == "__main__":
    unittest.main()
