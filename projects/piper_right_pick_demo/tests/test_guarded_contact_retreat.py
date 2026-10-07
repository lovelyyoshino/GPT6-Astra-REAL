"""Offline single loaded-contact retreat contract; never releases or runs a device."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import test_guarded_readiness_entry as readiness

ROOT = Path(__file__).parents[1]
RUN = ROOT / "runs/cola_on_cup_contact_retreat_20261006_225500"
TARGET = [53018, 109860, -48249, 0, -52889, 0]
SPEC = importlib.util.spec_from_file_location(
    "contact_retreat_under_test", ROOT / "scripts/ros_guarded_contact_retreat_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


class RetreatTests(readiness.ReadinessTests):
    def setUp(self):
        super().setUp()
        self.control.feedback.close()
        self.piper.q = [53007, 110146, -48210, 0, -52876, 0]
        self.piper.jaw = .05747
        self.session.update(contact_retreat_attempted=False, contact_retreat_completed=False,
                            held_raw=list(self.piper.q), scope_anchor_raw=list(self.piper.q))
        def register(name, callback):
            self.services[name] = callback
            return name
        self.control = entry.ContactRetreatTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            readiness.recorded.wide.wide_limits(), readiness.recorded.probe.fixtures.fk,
            self.store, self.session, self.path, {"offline": True}, lambda: None, self.clock,
            service_factory=register, namespace="/offline_contact_retreat", token="contact_retreat_offline")
        self.addCleanup(self.control.feedback.close)

    def retreat_message(self):
        return self.target(TARGET)

    def test_exact_single_four_frame_retreat_holds_load_and_stops_for_review(self):
        started = self.clock.monotonic()
        before = self.control.baseline()
        self.assertGreaterEqual(self.clock.monotonic()-started, 3.)
        self.assertEqual(before["opening_m"], .05747)
        self.assertEqual(self.piper.frames, [])
        held = []
        def during_send(frame):
            if len(self.piper.frames) == 1:
                held.append(self.control.request_hold(self.control.status()["sequence"], self.control.token)[0])
        self.piper.on_frame = during_send
        result = self.control.execute(self.retreat_message())
        self.assertEqual(result["phase"], "completed")
        self.assertEqual(self.piper.targets, [TARGET])
        self.assertEqual(self.piper.frames, entry.support.frames_for(TARGET))
        self.assertEqual(self.piper.jaw, .05747)
        self.assertEqual(held, [False])
        self.assertFalse(self.node.adopted)
        for action in (lambda: self.control.execute(self.retreat_message()),
                       lambda: self.control.gripper(types.SimpleNamespace()),
                       lambda: self.control.resume(*self.review_args()),
                       lambda: self.control.review_probe(*self.review_args())):
            with self.assertRaises(RuntimeError):
                action()
        self.assertEqual(len(self.piper.frames), 4)

    def test_different_goal_jaw_resume_and_hold_never_send(self):
        for axis in range(6):
            changed = list(TARGET)
            changed[axis] += 1
            with self.subTest(axis=axis), self.assertRaises(RuntimeError):
                self.control.execute(self.target(changed))
        for action in (lambda: self.control.gripper(types.SimpleNamespace(
                           gripper_angle=.07, gripper_effort=.2, gripper_code=1, set_zero=0)),
                       lambda: self.control.resume(1, 16, "bad"),
                       lambda: self.control.send_hold(self.piper.snapshot())):
            with self.assertRaises(RuntimeError):
                action()
        self.assertEqual(self.piper.frames, [])

    def test_partial_dispatch_is_permanent_failure_without_retry(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            self.control.execute(self.retreat_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 1)
        with self.assertRaises(RuntimeError):
            self.control.execute(self.retreat_message())
        self.assertEqual(len(self.piper.frames), 1)

    def test_rx_bad_then_good_stays_latched_with_no_retreat_send(self):
        good = list(self.piper.q)
        bad = list(good)
        bad[2] = 1
        fixture = readiness.recorded.probe.rx_fixtures
        for raw in (bad, good):
            self.piper.ParseCANFrame(fixture.message(*fixture.joint_fragment(2, raw), self.clock.time()))
        for _ in range(2):
            with self.assertRaises(RuntimeError):
                self.control.execute(self.retreat_message())
        self.assertEqual(self.piper.frames, [])
        self.assertIsNotNone(self.piper.rx_latch.first_fault)

    def test_readiness_timeout_does_not_reopen_this_one_shot_recovery(self):
        with patch.object(self.control, "baseline", side_effect=readiness.entry.stability.BaselineNotReady("not ready")):
            with self.assertRaises(RuntimeError):
                self.control.execute(self.retreat_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])
        with self.assertRaises(RuntimeError):
            self.control.execute(self.retreat_message())
        self.assertEqual(self.piper.frames, [])

    def test_held_load_opening_change_blocks_and_consumes_retreat(self):
        self.piper.jaw = .05807
        with self.assertRaises(RuntimeError):
            self.control.execute(self.retreat_message())
        self.assertEqual(self.piper.frames, [])
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.piper.jaw = .05747
        with self.assertRaises(RuntimeError):
            self.control.execute(self.retreat_message())
        self.assertEqual(self.piper.frames, [])


class HandoffTests(readiness.recorded.wide.OfflineTests):
    def test_exact_loaded_fault_evidence_twelve_parents_and_single_child_preserved(self):
        reviewed = json.loads((RUN / "reviewed_contact_retreat.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        session_root = entry.guard.v1.SESSION_ROOT
        parent = json.loads((session_root / readiness.entry.child_name(boot)).read_text())
        endpoint = entry.reviewed_evidence(reviewed, parent)
        self.assertEqual(endpoint["opening_m"], .05747)
        self.assertEqual(parent["status"]["phase"], "failed")
        self.assertEqual(parent["status"]["sequence"], 60)
        self.assertEqual(parent["raw_feedback_fault"]["axis"], 5)
        self.assertEqual(parent["raw_feedback_fault"]["raw"], -52532)
        changed = copy.deepcopy(parent)
        changed["status"]["receipts"][0]["socket_send_returns"] = 3
        with self.assertRaises(RuntimeError):
            entry.reviewed_evidence(reviewed, changed)
        previous = readiness.recorded.wide.entry.previous
        interior = previous.motion.predecessor
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME, interior.CHILD_NAME, previous.motion.frozen.child_name(boot),
            previous.motion.child_name(boot), previous.release.child_name(boot),
            readiness.recorded.wide.entry.child_name(boot), readiness.recorded.relief.child_name(boot),
            readiness.entry.predecessor.child_name(boot), readiness.entry.child_name(boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (session_root/name).read_bytes() for name in names}
            for name, raw in originals.items():
                (root/name).write_bytes(raw)
            with entry.reserve(boot, reviewed, session_root=root) as (_, measured, store, session):
                self.assertEqual(measured["opening_m"], .05747)
                self.assertFalse(session["contact_retreat_attempted"])
                self.assertTrue(session["parent_failure_chain_preserved"])
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, reviewed, session_root=root):
                    pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


for _name in dir(readiness.ReadinessTests):
    if _name.startswith("test_") and _name not in RetreatTests.__dict__:
        setattr(RetreatTests, _name, None)


if __name__ == "__main__":
    unittest.main()
