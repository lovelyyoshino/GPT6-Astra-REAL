"""Pure fake transport/probe review tests; not physical contact/clearance proof."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import test_ros_interruptible_joint_entry as fixtures
import test_guarded_rx_latch as rx_fixtures
from test_guarded_j5_profile import configured_limits

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location("regrasp_under_test", ROOT/"scripts/ros_guarded_regrasp_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


class ProbeTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp(); self.control.feedback.close()
        tick = patch.object(entry.motion.frozen.overlay.time, "time", side_effect=self.clock.time)
        tick.start(); self.addCleanup(tick.stop)
        self.piper = entry.profile.sdk_overlay(rx_fixtures.BusPiper, configured_limits())(self.clock)
        self.piper.q = list(entry.release.PARENT_TARGET); self.piper.jaw = .05439
        self.node.piper = self.piper
        self.session.update(stage="task", generation=9, generations=[], commissioning_attempted=True,
            held_raw=list(self.piper.q), first_segment_verified=False, regrasp_stage="retreat_ready",
            scope_anchor_raw=list(self.piper.q), probe_attempted=False, retreat_completed=0)
        def register(name, callback): self.services[name] = callback; return name
        self.control = entry.RegraspTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a,k))),
            configured_limits(), fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock,
            service_factory=register, namespace="/offline_regrasp", token="regrasp_offline")
        self.addCleanup(self.control.feedback.close)

    def target(self, raw):
        return types.SimpleNamespace(position=[v*entry.support.RAD_PER_RAW for v in raw],
                                     velocity=[0.]*6+[1.], effort=[])

    def probe_message(self):
        return self.target([a+d for a,d in zip(self.session["scope_anchor_raw"],
                                             entry.DELTAS[self.session["regrasp_stage"]])])

    def review_args(self):
        s = self.control.status()
        return s["sequence"], self.session["generation"], s["result_sha256"]

    def write_review(self, *, next_stage=None, contact_absent=True):
        """Synthetic explicitly-offline review bytes exercise binding, not vision."""
        state = self.control.status(); result = state["result"]; receipt = state["receipts"][0]
        seq, generation, digest = self.review_args()
        directory = self.path/"reviews"; directory.mkdir(exist_ok=True)
        raw_source = directory/("raw_%s.jsonl"%seq)
        content = b'{"event":"frame","synthetic_offline_fixture":true}\n'
        raw_source.write_bytes(content)
        evidence = dict(sequence=seq, adoption_token=self.control.token,
            completed_result_sha256=digest, full_raw_reviewed=True, transport_clean=True,
            nominal_violations=[], joint_tracking_violations=[], jaw_guard_violations=[], health_violations=[],
            first_kernel_unix_s=receipt["started_unix_s"]-.01,
            last_kernel_unix_s=max(result["after"]["stamps"])+.01,
            source_windows=[dict(path=str(raw_source), first_byte=0, last_byte_exclusive=len(content),
                                 sha256=hashlib.sha256(content).hexdigest())])
        raw_review = directory/("raw_review_%s.json"%seq)
        raw_review.write_text(json.dumps(evidence))
        image = directory/("offline_image_%s.bin"%seq)
        image.write_bytes(b"Synthetic review binding fixture, not a camera image")
        if next_stage is None:
            next_stage = "wrist_ready" if self.session["regrasp_stage"] == "retreat_review" else "task"
        material = dict(reviewed=True, stage=self.session["regrasp_stage"], sequence=seq,
            generation=generation, adoption_token=self.control.token, completed_result_sha256=digest,
            prior_failures_preserved=True, object_contact_absent=contact_absent, next_stage=next_stage,
            table_supported=True, no_visible_hook_or_tension=True,
            additional_j5_violation_not_reclassified=True, review_authorization="offline test only",
            raw_review=dict(path=str(raw_review), sha256=entry.sha(raw_review)),
            images=[dict(path=str(image), sha256=entry.sha(image))])
        path = directory/("action_%06d_review.json"%seq)
        path.write_text(json.dumps(material))
        return path, material, raw_source

    def finish_retreat(self):
        result = self.control.execute(self.probe_message())
        self.assertEqual(result["phase"], "completed")
        self.assertEqual(self.session["regrasp_stage"], "retreat_review")
        self.assertFalse(self.node.adopted)
        return result

    def approve_current(self, **review_options):
        self.write_review(**review_options); before = len(self.piper.frames); started = self.clock.monotonic()
        result = self.control.review_probe(*self.review_args())
        self.assertTrue(result[0]); self.assertEqual(len(self.piper.frames), before)
        self.assertGreaterEqual(self.clock.monotonic()-started, 3.)
        return result

    def test_zero_tx_initialization_fixed_contact_retreat_and_early_jaw_rejection(self):
        self.control.baseline(); self.assertEqual(self.piper.frames, [])
        self.assertTrue(self.control.identity["contact_possible"])
        self.assertFalse(self.control.identity["initial_no_contact_claim"])
        self.assertNotIn("empty_gripper_operator_confirmed", self.control.identity)
        anchor = list(self.session["scope_anchor_raw"])
        target = [a+d for a,d in zip(anchor, [0,-350,350,0,0,0])]
        self.assertEqual(entry.scoped_target(self.target(target), self.control.read(), self.session), target)
        for axis in (0, 1, 2, 4):
            changed = list(target); changed[axis] += 1
            with self.subTest(axis=axis), self.assertRaises(RuntimeError):
                self.control.execute(self.target(changed))
        with self.assertRaises(RuntimeError): self.control.gripper(types.SimpleNamespace())
        self.assertEqual(self.piper.frames, [])

    def test_completed_retreat_cannot_continue_without_bound_independent_review(self):
        result = self.finish_retreat()
        self.assertTrue(result["probe_motion_evidence"]["sufficient"])
        self.assertEqual(len(self.piper.frames), 4)
        self.assertEqual(self.piper.q[4], self.control.status()["before"]["raw_q"][4])
        self.assertEqual(self.piper.jaw, .05439)
        with self.assertRaises(RuntimeError): self.control.execute(self.message(100))
        with self.assertRaises(RuntimeError): self.control.gripper(types.SimpleNamespace())
        with self.assertRaises(RuntimeError): self.control.resume(*self.review_args())
        with self.assertRaises(FileNotFoundError): self.control.review_probe(*self.review_args())
        self.assertEqual(len(self.piper.frames), 4)
        self.assertFalse(self.node.adopted)

    def test_both_reviews_are_fresh_zero_tx_single_use_then_task_j5_stays_bounded(self):
        self.finish_retreat(); first_review = self.review_args()
        self.approve_current()
        self.assertEqual(self.session["regrasp_stage"], "wrist_ready")
        self.assertEqual(self.session["generation"], 10)
        with self.assertRaises(RuntimeError): self.control.review_probe(*first_review)
        with self.assertRaises(RuntimeError): self.control.gripper(types.SimpleNamespace())
        anchor = list(self.piper.q)
        result = self.control.execute(self.probe_message())
        self.assertEqual(self.piper.targets[-1], [v+(-150 if i==4 else 0) for i,v in enumerate(anchor)])
        self.assertEqual(result["phase"], "completed")
        self.assertEqual(self.session["regrasp_stage"], "wrist_review")
        self.assertFalse(self.node.adopted)
        second_review = self.review_args(); self.approve_current()
        self.assertEqual(self.session["regrasp_stage"], "task")
        self.assertEqual(self.session["generation"], 11)
        self.assertEqual(len(self.session["probe_reviews"]), 2)
        with self.assertRaises(RuntimeError): self.control.review_probe(*second_review)
        target = list(self.piper.q); target[4] -= 201
        with self.assertRaisesRegex(RuntimeError, "200millidegrees"):
            self.control.execute(self.target(target))
        self.assertEqual(len(self.piper.frames), 8)
        target[4] += 1
        self.assertEqual(self.control.execute(self.target(target))["phase"], "completed")
        self.assertEqual(len(self.piper.frames), 12)

    def test_contact_retreat_requires_review_each_time_and_stops_after_eight_segments(self):
        for number in range(1, 9):
            self.finish_retreat()
            self.assertEqual(self.session["retreat_completed"], number)
            if number < 8:
                self.approve_current(next_stage="retreat_ready", contact_absent=False)
                self.assertEqual(self.session["regrasp_stage"], "retreat_ready")
            else:
                self.write_review(next_stage="retreat_ready", contact_absent=False)
                with self.assertRaisesRegex(RuntimeError, "budget exhausted"):
                    self.control.review_probe(*self.review_args())
                self.assertFalse(self.node.adopted)
                self.write_review(next_stage="wrist_ready", contact_absent=False)
                with self.assertRaises(RuntimeError): self.control.review_probe(*self.review_args())
                self.approve_current(next_stage="wrist_ready", contact_absent=True)
                self.assertEqual(self.session["regrasp_stage"], "wrist_ready")
        self.assertEqual(len(self.piper.frames), 32)

    def test_repeat_contact_retreat_requires_support_and_absence_of_hook_or_tension(self):
        self.finish_retreat()
        path, material, _ = self.write_review(next_stage="retreat_ready", contact_absent=False)
        for key in ("table_supported", "no_visible_hook_or_tension"):
            changed = dict(material); changed[key] = False; path.write_text(json.dumps(changed))
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.control.review_probe(*self.review_args())
            self.assertFalse(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 4)

    def test_changed_contact_review_or_raw_bytes_cannot_promote(self):
        self.finish_retreat(); path, material, source = self.write_review()
        for key, value in (("object_contact_absent", False), ("prior_failures_preserved", False),
                           ("adoption_token", "other"), ("generation", 8)):
            changed = dict(material); changed[key] = value; path.write_text(json.dumps(changed))
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.control.review_probe(*self.review_args())
            self.assertFalse(self.node.adopted)
        path.write_text(json.dumps(material)); source.write_bytes(b"changed")
        with self.assertRaisesRegex(RuntimeError, "Raw probe source changed"):
            self.control.review_probe(*self.review_args())
        self.assertEqual(len(self.piper.frames), 4)
        self.assertEqual(self.session["regrasp_stage"], "retreat_review")

    def test_actual_fresh_review_drift_latches_failure_without_promotion_or_retry(self):
        self.finish_retreat(); self.write_review(); args = self.review_args()
        self.piper.q[4] += 172; self.piper.goal = list(self.piper.q)
        with self.assertRaisesRegex(RuntimeError, "slip"):
            self.control.review_probe(*args)
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        with self.assertRaises(RuntimeError): self.control.review_probe(*args)
        self.assertEqual(len(self.piper.frames), 4)

    def test_partial_probe_never_reviews_or_retries(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            self.control.execute(self.probe_message())
        self.assertTrue(self.session["probe_attempted"])
        self.assertIsNotNone(self.session["failure"])
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (2,1))
        with self.assertRaises(RuntimeError): self.control.execute(self.probe_message())
        with self.assertRaises(RuntimeError): self.control.review_probe(*self.review_args())
        self.assertEqual(len(self.piper.frames), 1)

    def test_pending_failed_or_incomplete_receipt_cannot_review(self):
        self.finish_retreat(); self.write_review()
        clean_session = copy.deepcopy(self.session); clean_state = copy.deepcopy(self.control.state)
        for reason in ("pending", "failure", "receipt", "held"):
            self.session.clear(); self.session.update(copy.deepcopy(clean_session))
            self.control.state = copy.deepcopy(clean_state)
            if reason == "pending": self.session["pending"] = {"unresolved": True}
            if reason == "failure": self.session["failure"] = {"error": "unresolved"}
            if reason == "receipt": self.control.state["receipts"][0]["socket_send_returns"] = 3
            if reason == "held": self.session["stop_latched"] = True
            with self.subTest(reason=reason), self.assertRaises(RuntimeError):
                self.control.review_probe(*self.review_args())
            self.assertFalse(self.node.adopted)
            self.assertEqual(len(self.piper.frames), 4)

    def test_wrist_within_arrival_tolerance_but_insufficient_motion_cannot_review(self):
        self.finish_retreat(); self.approve_current()
        joint = self.piper.JointCtrl
        def shallow_motion(*raw):
            joint(*raw)
            self.piper.q[4] -= 40
            self.piper.goal = list(self.piper.q)
        self.piper.JointCtrl = shallow_motion
        result = self.control.execute(self.probe_message())
        self.assertEqual(result["phase"], "completed")
        self.assertFalse(result["probe_motion_evidence"]["sufficient"])
        self.assertIsNone(self.control.status()["probe_review_service"])
        self.assertFalse(self.node.adopted)
        with self.assertRaises(RuntimeError): self.control.review_probe(*self.review_args())
        self.assertEqual(len(self.piper.frames), 8)


class HandoffTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        run = ROOT/"runs/cola_on_cup_regrasp_20261006_201000"
        self.reviewed = json.loads((run/"reviewed_regrasp.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        self.parent = json.loads((entry.guard.v1.SESSION_ROOT/entry.release.child_name(boot)).read_text())

    def test_real_failed_release_and_stationary_tail_accept_without_claiming_success(self):
        endpoint = entry.reviewed_evidence(self.reviewed, self.parent)
        self.assertEqual(endpoint["opening_m"], .05439)
        self.assertEqual(endpoint["raw_q"], [33118,105639,-31835,0,-52597,0])
        self.assertFalse(self.parent["release_completed"])
        self.assertEqual(self.parent["status"]["phase"], "failed")
        for key, value in (("initial_no_contact_claim", True), ("post_probe_reviews_required", False),
                ("contact_separation_retreat_only", False), ("held_jaw_m", .055),
                ("prior_jaw_and_joint_failures_preserved", False),
                ("retreat_delta_raw", [0,-350,350,0,-200,0])):
            reviewed = copy.deepcopy(self.reviewed); reviewed[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(reviewed, self.parent)

    def test_partial_release_or_reclassified_success_cannot_be_adopted(self):
        for kind in ("partial", "success"):
            parent = copy.deepcopy(self.parent)
            if kind == "partial": parent["status"]["receipts"][0]["socket_send_returns"] = 0
            else: parent["release_completed"] = True; parent["status"]["phase"] = "completed"
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(self.reviewed, parent)
        reviewed = copy.deepcopy(self.reviewed); reviewed["visual_evidence"][0]["sha256"] = "0"*64
        with self.assertRaisesRegex(RuntimeError, "Visual evidence changed"):
            entry.reviewed_evidence(reviewed, self.parent)

    def test_eight_parent_files_immutable_and_boot_child_rejects_restart(self):
        boot = entry.guard.predecessor.PARENT_BOOT; interior = entry.motion.predecessor
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME, interior.CHILD_NAME, entry.motion.frozen.child_name(boot),
            entry.motion.child_name(boot), entry.release.child_name(boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (entry.guard.v1.SESSION_ROOT/name).read_bytes() for name in names}
            for name, data in originals.items(): (root/name).write_bytes(data)
            with entry.reserve(boot, self.reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(session["generation"], 9)
                self.assertEqual(session["regrasp_stage"], "retreat_ready")
                self.assertTrue(session["initial_contact_possible"])
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertFalse(session["feedback_excursion_physical_cause_resolved"])
                self.assertEqual(store.load()["prior_release_failure"], self.parent["failure"])
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, self.reviewed, session_root=root): pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_") and _name not in ProbeTests.__dict__:
        setattr(ProbeTests, _name, None)


if __name__ == "__main__": unittest.main()
