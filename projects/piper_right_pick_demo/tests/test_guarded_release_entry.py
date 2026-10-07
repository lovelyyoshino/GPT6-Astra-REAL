"""Offline one-shot release tests; no device/ROS connection or physical claim."""
import contextlib
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import types
import unittest
from unittest.mock import patch

import test_ros_interruptible_joint_entry as fixtures
import test_guarded_rx_latch as rx_fixtures
from test_guarded_j5_profile import configured_limits

ROOT = Path(__file__).parents[1]
RUN = ROOT/"runs/cola_on_cup_contact_release_20261006_195000"
SPEC = importlib.util.spec_from_file_location("release_entry_under_test", ROOT/"scripts/ros_guarded_release_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


def request(**changes):
    values = dict(gripper_angle=.055, gripper_effort=.2, gripper_code=1, set_zero=0)
    values.update(changes)
    return types.SimpleNamespace(**values)


class IntegrationTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp(); self.control.feedback.close()
        tick = patch.object(entry.frozen.overlay.time, "time", side_effect=self.clock.time)
        tick.start(); self.addCleanup(tick.stop)
        self.piper = entry.profile.sdk_overlay(rx_fixtures.BusPiper, configured_limits())(self.clock)
        self.piper.q = list(entry.PARENT_TARGET); self.piper.jaw = .0483
        self.node.piper = self.piper
        self.session.update(stage="task", generation=8, generations=[], commissioning_attempted=True,
            held_raw=list(self.piper.q), first_segment_verified=False, release_attempted=False,
            release_completed=False, release_only=True, joint_motion_authorized=False)
        def register(name, callback): self.services[name] = callback; return name
        self.control = entry.ReleaseTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a,k))),
            configured_limits(), fixtures.fk, self.store, self.session, self.path,
            {"offline": True, "empty_gripper_operator_confirmed": True}, lambda: None, self.clock,
            service_factory=register, namespace="/offline_release", token="release_offline")
        self.addCleanup(self.control.feedback.close)

    def test_constructor_baseline_zero_tx_and_startup_does_not_claim_empty_gripper(self):
        self.assertEqual(self.piper.frames, [])
        self.assertNotIn("empty_gripper_operator_confirmed", self.control.identity)
        self.assertTrue(self.control.identity["table_supported_contact_confirmed"])
        self.assertFalse(self.control.identity["joint_motion_authorized"])
        started = self.clock.monotonic(); self.control.baseline()
        self.assertGreaterEqual(self.clock.monotonic()-started, 3.)
        self.assertEqual(self.piper.frames, [])
        self.assertFalse(self.session["release_attempted"])

    def test_all_joint_hold_and_resume_methods_reject_before_any_frame(self):
        calls = (lambda: self.control.execute(self.message(100)),
                 lambda: self.control.send_once(None, None, None, None, "initial", moving=False),
                 lambda: self.control.send_hold(None),
                 lambda: self.control.resume(0, 8, "unused"))
        for call in calls:
            with self.assertRaisesRegex(RuntimeError, "Release-only"): call()
        self.assertEqual(self.piper.frames, [])
        self.assertFalse(self.session["release_attempted"])

    def test_only_exact_55mm_point_two_code_one_no_zero_parameters_allowed(self):
        for name, value in (("gripper_angle", .054), ("gripper_angle", .0550001),
                ("gripper_angle", math.nan), ("gripper_angle", True),
                ("gripper_effort", .5), ("gripper_code", 0), ("gripper_code", True), ("set_zero", 1)):
            with self.subTest(name=name, value=value), self.assertRaises(RuntimeError):
                self.control.gripper(request(**{name: value}))
            self.assertFalse(self.session["release_attempted"])
            self.assertEqual(self.piper.frames, [])

    def test_exact_single_frame_completion_disables_adoption_and_never_promotes_joint_motion(self):
        result = self.control.gripper(request())
        self.assertEqual(self.piper.frames, [(0x159, struct.pack(">iHBB", 55000, 200, 1, 0))])
        self.assertEqual(result["phase"], "completed")
        self.assertTrue(result["jaw_target_reached"])
        self.assertTrue(result["release_only"])
        self.assertFalse(result["release_verified"])
        self.assertFalse(result["grasp_verified"])
        self.assertTrue(result["visual_release_review_required"])
        self.assertGreaterEqual(result["stable_window"]["duration_s"], 3.)
        self.assertGreaterEqual(result["stable_window"]["new_feedback_groups"], 20)
        self.assertTrue(self.session["release_completed"])
        self.assertTrue(self.session["stop_latched"])
        self.assertIsNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertIsNone(self.node.failed)
        self.assertFalse(self.session["first_segment_verified"])
        for call in (lambda: self.control.gripper(request()), lambda: self.control.execute(self.message(100)),
                     lambda: self.control.resume(1, 8, "unused")):
            with self.assertRaises(RuntimeError): call()
        self.assertEqual(len(self.piper.frames), 1)

    def test_partial_gripper_send_consumes_opportunity_and_cannot_retry(self):
        self.piper.failure = "jaw_send_failure"
        with self.assertRaisesRegex(RuntimeError, "jaw send failed"):
            self.control.gripper(request())
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (1, 0))
        self.assertTrue(self.session["release_attempted"])
        self.assertFalse(self.session["release_completed"])
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertIsNone(self.piper.ticket)
        with self.assertRaises(RuntimeError): self.control.gripper(request())
        self.assertEqual(self.piper.frames, [])

    def test_release_retains_original_j5_point_zero_zero_three_rx_margin(self):
        def after_frame(_):
            if len(self.piper.frames) != 1: return
            bad = list(self.piper.q); bad[4] += 172
            for raw in (bad, self.piper.q):
                ident, data = rx_fixtures.joint_fragment(4, raw)
                self.piper.ParseCANFrame(rx_fixtures.message(ident, data, self.clock.time()))
        self.piper.on_frame = after_frame
        # The same send records a broken RX state before its transaction check;
        # it can reject there before reaching the next explicit latch check.
        with self.assertRaisesRegex(RuntimeError, "Incomplete jaw transaction"):
            self.control.gripper(request())
        self.assertEqual(self.piper.rx_latch.first_fault["tracking_tolerance_rad"], .003)
        self.assertEqual(len(self.piper.frames), 1)
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (1, 1))
        with self.assertRaises(RuntimeError): self.control.gripper(request())
        self.assertEqual(len(self.piper.frames), 1)

    def test_stable_but_not_opened_is_failure_and_no_retry(self):
        snapshot = self.piper.snapshot
        def blocked_opening():
            state = snapshot()
            if self.piper.frames: state["opening_m"] = .050
            return state
        self.piper.snapshot = blocked_opening
        with self.assertRaisesRegex(RuntimeError, "opening not reached"):
            self.control.gripper(request())
        self.assertFalse(self.session["release_completed"])
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        with self.assertRaises(RuntimeError): self.control.gripper(request())
        self.assertEqual(len(self.piper.frames), 1)

    def test_wrong_stage_is_zero_tx_failure_and_cannot_retry(self):
        self.session["stage"] = "commissioning"
        with self.assertRaises(RuntimeError): self.control.gripper(request())
        self.assertTrue(self.session["release_attempted"])
        self.assertIsNotNone(self.session["failure"])
        with self.assertRaises(RuntimeError): self.control.gripper(request())
        self.assertEqual(self.piper.frames, [])


class HandoffTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        self.reviewed = json.loads((RUN/"reviewed_release.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        self.parent = json.loads((entry.guard.v1.SESSION_ROOT/entry.previous.child_name(boot)).read_text())

    def test_real_seq82_review_preserves_jaw_and_additional_j5_failures(self):
        endpoint = entry.reviewed_evidence(self.reviewed, self.parent)
        self.assertEqual(endpoint["opening_m"], .0483)
        self.assertLessEqual(max(abs(a-b)*entry.support.RAD_PER_RAW
            for a,b in zip(endpoint["raw_q"], entry.PARENT_TARGET)), .003)
        for key, value in (("reviewed", False), ("release_only", False),
                ("table_supported_contact_confirmed", False), ("joint_motion_authorized", True),
                ("additional_j5_tracking_violation_acknowledged", False), ("release_target_m", .05),
                ("parent_sequence", 81), ("user_authorization", "")):
            reviewed = copy.deepcopy(self.reviewed); reviewed[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(reviewed, self.parent)

    def test_changed_visual_evidence_partial_parent_and_raw_review_digest_reject(self):
        reviewed = copy.deepcopy(self.reviewed); reviewed["visual_evidence"][0]["sha256"] = "0"*64
        with self.assertRaisesRegex(RuntimeError, "Visual evidence"):
            entry.reviewed_evidence(reviewed, self.parent)
        parent = copy.deepcopy(self.parent); parent["status"]["receipts"][0]["socket_send_returns"] = 3
        with self.assertRaises(RuntimeError): entry.reviewed_evidence(self.reviewed, parent)
        original_sha = entry.sha
        def changed_sha(path):
            return "0"*64 if Path(path).name == "seq82_raw_jaw_and_tracking_review.json" else original_sha(path)
        with patch.object(entry, "sha", side_effect=changed_sha), self.assertRaisesRegex(RuntimeError, "Reviewed evidence changed"):
            entry.reviewed_evidence(self.reviewed, self.parent)

    def test_seven_parent_files_unchanged_and_fixed_child_prevents_new_release_attempt(self):
        boot = entry.guard.predecessor.PARENT_BOOT; interior = entry.previous.predecessor
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME, interior.CHILD_NAME,
            entry.frozen.child_name(boot), entry.previous.child_name(boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (entry.guard.v1.SESSION_ROOT/name).read_bytes() for name in names}
            for name, data in originals.items(): (root/name).write_bytes(data)
            with entry.reserve(boot, self.reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(session["generation"], 8)
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertTrue(session["additional_j5_tracking_violation_unresolved"])
                self.assertFalse(session["joint_motion_authorized"])
                self.assertFalse(session["release_attempted"])
                self.assertFalse(session["feedback_excursion_physical_cause_resolved"])
                self.assertEqual(store.load()["prior_failure"], self.parent["failure"])
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, self.reviewed, session_root=root): pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)

    def test_startup_flag_requires_table_support_without_false_empty_confirmation(self):
        parser = entry.ReleaseParser()
        parser.add_argument("--empty-gripper-confirmed", action="store_true", required=True)
        parsed = parser.parse_args(["--table-supported-contact-confirmed"])
        self.assertTrue(parsed.table_supported_contact_confirmed)
        self.assertFalse(hasattr(parsed, "empty_gripper_confirmed"))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parser.parse_args(["--empty-gripper-confirmed"])


for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_") and _name not in IntegrationTests.__dict__:
        setattr(IntegrationTests, _name, None)


if __name__ == "__main__": unittest.main()
