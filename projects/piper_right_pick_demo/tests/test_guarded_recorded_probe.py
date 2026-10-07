"""Offline one-shot recorded wrist qualification; no device access."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest

import test_guarded_regrasp_entry as probe
import test_guarded_wide_entry as wide
import ros_guarded_relief_entry as relief

ROOT = Path(__file__).parents[1]
RUN = ROOT / "runs/cola_on_cup_probe_recorded_20261006_212800"
SPEC = importlib.util.spec_from_file_location(
    "recorded_probe_under_test", ROOT / "scripts/ros_guarded_recorded_probe_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


class RecordedTests(probe.ProbeTests):
    def setUp(self):
        super().setUp()
        self.control.feedback.close()
        self.piper = wide.profile.original.sdk_overlay(probe.rx_fixtures.BusPiper, wide.wide_limits())(self.clock)
        self.piper.q = [33118, 105287, -31482, 0, -52753, 0]
        self.piper.jaw = .067
        self.node.piper = self.piper
        self.session.update(generation=14, regrasp_stage="wrist_ready", probe_attempted=False,
            scope_anchor_raw=list(self.piper.q), held_raw=list(self.piper.q), probe_reviews=[],
            prior_seq3_independent_raw_review_complete=False,
            prior_seq3_result_remains_completed_but_evidence_incomplete=True)
        def register(name, callback):
            self.services[name] = callback
            return name
        self.control = entry.RecordedTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            wide.wide_limits(), probe.fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_recorded", token="recorded_offline")
        self.addCleanup(self.control.feedback.close)

    def probe_message(self):
        raw = list(self.session["scope_anchor_raw"])
        raw[4] -= 150
        return self.target(raw)

    def finish_probe(self):
        result = self.control.execute(self.probe_message())
        self.assertEqual(result["phase"], "completed")
        self.assertEqual(self.session["regrasp_stage"], "wrist_review")
        self.assertFalse(self.node.adopted)
        return result

    def test_zero_tx_startup_only_anchor_minus150_and_no_early_jaw(self):
        self.control.baseline()
        self.assertEqual(self.piper.frames, [])
        expected = list(self.session["scope_anchor_raw"])
        expected[4] -= 150
        for delta in (-1, 1):
            wrong = list(expected)
            wrong[4] += delta
            with self.assertRaises(RuntimeError):
                self.control.execute(self.target(wrong))
        with self.assertRaises(RuntimeError):
            self.control.gripper(types.SimpleNamespace())
        result = self.finish_probe()
        self.assertEqual(self.piper.targets, [expected])
        self.assertEqual(len(self.piper.frames), 4)
        self.assertTrue(result["probe_motion_evidence"]["sufficient"])
        with self.assertRaises(RuntimeError):
            self.control.execute(self.probe_message())
        with self.assertRaises(RuntimeError):
            self.control.gripper(types.SimpleNamespace())

    def test_completed_probe_requires_new_full_raw_and_fresh_review_to_task(self):
        self.finish_probe()
        args = self.review_args()
        with self.assertRaises(FileNotFoundError):
            self.control.review_probe(*args)
        path, material, _ = self.write_review(next_stage="task", contact_absent=True)
        raw_path = Path(material["raw_review"]["path"])
        raw = json.loads(raw_path.read_text())
        for key, value in (("full_raw_reviewed", False), ("transport_clean", False)):
            changed = dict(raw)
            changed[key] = value
            raw_path.write_text(json.dumps(changed))
            material["raw_review"]["sha256"] = entry.sha(raw_path)
            path.write_text(json.dumps(material))
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.control.review_probe(*args)
            with self.assertRaises(RuntimeError):
                self.control.execute(self.probe_message())
            self.assertEqual(len(self.piper.frames), 4)
        raw_path.write_text(json.dumps(raw))
        material["raw_review"]["sha256"] = entry.sha(raw_path)
        path.write_text(json.dumps(material))
        started = self.clock.monotonic()
        self.assertTrue(self.control.review_probe(*args)[0])
        self.assertGreaterEqual(self.clock.monotonic()-started, 3.)
        self.assertEqual(self.session["regrasp_stage"], "task")
        self.assertTrue(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 4)
        self.assertFalse(self.session["prior_seq3_independent_raw_review_complete"])
        with self.assertRaises(RuntimeError):
            self.control.review_probe(*args)

    def test_toleranced_zero_progress_never_qualifies_or_reissues(self):
        joint = self.piper.JointCtrl
        def no_progress(*raw):
            joint(*raw)
            self.piper.goal = list(self.piper.q)
        self.piper.JointCtrl = no_progress
        result = self.finish_probe()
        self.assertFalse(result["probe_motion_evidence"]["sufficient"])
        self.assertIsNone(self.control.status()["probe_review_service"])
        with self.assertRaises(RuntimeError):
            self.control.review_probe(*self.review_args())
        with self.assertRaises(RuntimeError):
            self.control.execute(self.probe_message())
        self.assertEqual(len(self.piper.frames), 4)

    def test_partial_dispatch_cannot_retry_or_review(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            self.control.execute(self.probe_message())
        self.assertEqual(len(self.piper.frames), 1)
        self.assertIsNotNone(self.session["failure"])
        for call in (lambda: self.control.execute(self.probe_message()),
                     lambda: self.control.review_probe(*self.review_args())):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual(len(self.piper.frames), 1)

    def test_rx_bad_then_good_does_not_reopen_qualification(self):
        good = list(self.piper.q)
        bad = list(good)
        bad[2] = 1
        for raw in (bad, good):
            self.piper.ParseCANFrame(probe.rx_fixtures.message(
                *probe.rx_fixtures.joint_fragment(2, raw), self.clock.time()))
        self.assertIsNotNone(self.piper.rx_latch.first_fault)
        for _ in range(2):
            with self.assertRaises(RuntimeError):
                self.control.execute(self.probe_message())
        self.assertEqual(self.piper.frames, [])


class HandoffTests(wide.OfflineTests):
    def setUp(self):
        super().setUp()
        self.boot = entry.guard.predecessor.PARENT_BOOT
        self.reviewed = json.loads((RUN / "reviewed_recorded_probe.json").read_text())
        self.parent = json.loads((entry.guard.v1.SESSION_ROOT / relief.child_name(self.boot)).read_text())

    def test_only_completed_evidence_gap_is_admitted_not_rx_failure_or_partial(self):
        endpoint = entry.reviewed_evidence(self.reviewed, self.parent)
        self.assertEqual(self.parent["status"]["phase"], "completed")
        self.assertEqual(self.parent["status"]["sequence"], 3)
        self.assertEqual(endpoint["raw_q"], self.parent["status"]["result"]["after"]["raw_q"])
        for kind in ("rx_fault", "failed", "partial"):
            parent = copy.deepcopy(self.parent)
            if kind == "rx_fault":
                parent["raw_feedback_fault"] = {"reason": "synthetic test fault"}
            elif kind == "failed":
                parent["status"]["phase"] = "failed"
            else:
                parent["status"]["receipts"][0]["socket_send_returns"] = 3
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(self.reviewed, parent)

    def test_ten_parents_and_missing_evidence_remain_immutable_with_unique_child(self):
        previous = wide.entry.previous
        interior = previous.motion.predecessor
        names = [self.boot + ".json", entry.guard.predecessor.CHILD_NAME,
            "guarded_task_" + self.boot + ".json", interior.predecessor.CHILD_NAME,
            interior.CHILD_NAME, previous.motion.frozen.child_name(self.boot),
            previous.motion.child_name(self.boot), previous.release.child_name(self.boot),
            wide.entry.child_name(self.boot), relief.child_name(self.boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (entry.guard.v1.SESSION_ROOT / name).read_bytes() for name in names}
            for name, data in originals.items():
                (root / name).write_bytes(data)
            with entry.reserve(self.boot, self.reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(session["regrasp_stage"], "wrist_ready")
                self.assertFalse(session["probe_attempted"])
                self.assertFalse(session["prior_seq3_independent_raw_review_complete"])
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertEqual(store.load()["regrasp_stage"], "wrist_ready")
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(self.boot, self.reviewed, session_root=root):
                    pass
            self.assertEqual({name: (root / name).read_bytes() for name in names}, originals)


for _name in dir(probe.ProbeTests):
    if _name.startswith("test_") and _name not in RecordedTests.__dict__:
        setattr(RecordedTests, _name, None)


if __name__ == "__main__":
    unittest.main()
