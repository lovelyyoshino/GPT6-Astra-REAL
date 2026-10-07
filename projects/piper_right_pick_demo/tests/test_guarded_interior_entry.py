"""Assigned-state tests of interior admission; no hardware or dynamics claim."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import types
import unittest
from unittest.mock import patch

import test_ros_guarded_task_entry as guarded_fixtures
import test_ros_interruptible_joint_entry as fixtures

SPEC = importlib.util.spec_from_file_location(
    "interior_under_test", Path(__file__).parents[1]/"scripts/ros_guarded_interior_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


def reviewed():
    return dict(parent_session_sha256=entry.PARENT_SHA,
                parent_action_result_sha256=entry.RESULT_SHA,
                independent_can_sha256=entry.CAN_SHA, parent_sequence=45,
                parent_adoption_token=entry.PARENT_TOKEN,
                user_authorization=entry.AUTHORIZATION,
                first_target_raw=list(entry.FIXED_TARGET), offending_raw_sample_missing=True)


def saved_evidence():
    parent_path = entry.guard.v1.SESSION_ROOT/entry.predecessor.CHILD_NAME
    result_path = entry.PARENT_RUN/"runtime/action_000045_result.json"
    trace_path = entry.PARENT_RUN/"shoulder045_post_failure_can.json"
    for path, expected in ((parent_path, entry.PARENT_SHA), (result_path, entry.RESULT_SHA),
                           (trace_path, entry.CAN_SHA)):
        if entry.sha(path) != expected:
            raise AssertionError("Frozen archived evidence changed: "+str(path))
    return tuple(json.loads(path.read_text()) for path in (parent_path, result_path, trace_path))


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        loader = patch.object(entry.support, "load_base", return_value=fixtures.base)
        loader.start(); self.addCleanup(loader.stop)

    def test_real_frozen_saved_evidence_and_explicit_review(self):
        endpoint = entry.check_evidence(*saved_evidence(), reviewed())
        self.assertEqual(endpoint["raw_q"][1:3], [0, 0])
        for key in reviewed():
            consent = reviewed(); consent.pop(key)
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, "Explicit reviewed"):
                entry.check_evidence(*saved_evidence(), consent)

    def test_trace_one_raw_unit_outside_nominal_is_rejected(self):
        for axis, value in ((1, -1), (2, 1)):
            parent, result, trace = saved_evidence()
            row = trace["samples"][0]
            ident = 0x2a5+axis//2
            frame = next(f for f in row["frames"] if f["id"] == ident)
            raw = list(struct.unpack(">ii", bytes.fromhex(frame["data_hex"])))
            raw[axis % 2] = value
            frame["data_hex"] = struct.pack(">ii", *raw).hex()
            with self.subTest(axis=axis), self.assertRaisesRegex(RuntimeError, "nominal limits"):
                entry.check_evidence(parent, result, trace, reviewed())

    def test_partial_receipt_or_unresolved_hold_cannot_authorize_child(self):
        for case in ("partial", "hold", "other_failure"):
            parent, result, trace = saved_evidence()
            if case == "partial": result["receipts"][0]["socket_send_returns"] = 3
            elif case == "hold": result["hold_requested"] = True
            else:
                result["failure"] = "other failure"
                parent["failure"]["error"] = "other failure"
            parent["status"] = copy.deepcopy(result)
            with self.subTest(case=case), self.assertRaises(RuntimeError):
                entry.check_evidence(parent, result, trace, reviewed())

    def test_all_four_parent_files_unchanged_and_child_cannot_repeat(self):
        _, result, trace = saved_evidence()
        boot = entry.guard.predecessor.PARENT_BOOT
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME,
                 "guarded_task_"+boot+".json", entry.predecessor.CHILD_NAME]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (entry.guard.v1.SESSION_ROOT/name).read_bytes() for name in names}
            for name, data in originals.items(): (root/name).write_bytes(data)
            with entry.reserve(boot, reviewed(), result, trace, session_root=root) as (_, _, store, session):
                self.assertEqual(session["fixed_target_raw"], entry.FIXED_TARGET)
                self.assertFalse(session["boundary_failure_cause_resolved"])
                self.assertTrue(session["offending_raw_sample_missing"])
                self.assertEqual(store.load()["recovery_stage"], "pilot")
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, reviewed(), result, trace, session_root=root): pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


class BidirectionalPiper(guarded_fixtures.MultiPiper):
    def __init__(self, clock):
        super().__init__(clock)
        self.q = [69, 0, 0, 0, 7128, -3554]
        self.stay_boundary = False

    def snapshot(self):
        goal = self.goal
        if goal is not None:
            for i in range(6):
                if self.stay_boundary and i in (1, 2): continue
                self.q[i] += max(-self.step_raw, min(self.step_raw, goal[i]-self.q[i]))
        # Disable predecessor fake's J2-only negative-direction behavior.
        self.goal = None
        try: result = super().snapshot()
        finally: self.goal = goal
        result["motion_status"] = 0 if self.stay_boundary else int(goal is not None and self.q != goal)
        return result


class InteriorTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp(); self.control.feedback.close()
        self.piper = BidirectionalPiper(self.clock); self.node.piper = self.piper
        self.session.update(stage="task", generation=10, generations=[],
                            commissioning_attempted=True, held_raw=list(self.piper.q),
                            recovery_stage="pilot", pilot_attempted=False)
        def register(name, callback): self.services[name] = callback; return name
        self.control = entry.InteriorTask(
            self.node, types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            fixtures.limits(), fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_driver", token="interior_token")
        self.addCleanup(self.control.feedback.close)

    def message(self, raw=None):
        return types.SimpleNamespace(position=[v*entry.support.RAD_PER_RAW
                                               for v in (entry.FIXED_TARGET if raw is None else raw)],
                                     velocity=[0.]*6+[1.], effort=[])

    def review(self):
        state = self.control.status()
        return self.control.review_recovery(state["sequence"], state["generation"], state["result_sha256"])

    def test_actual_interior_three_seconds_then_independent_review_three_seconds(self):
        result = self.control.execute(self.message())
        self.assertEqual(result["phase"], "completed")
        self.assertTrue(result["interior_verified"])
        self.assertFalse(result["boundary_failure_cause_resolved"])
        window = result["stable_window"]
        self.assertGreaterEqual(window["duration_s"], 3.)
        self.assertGreaterEqual(window["new_feedback_groups"], 20)
        self.assertGreater(window["minimum_j2_raw"], 0)
        self.assertLess(window["maximum_j3_raw"], 0)
        self.assertFalse(self.node.adopted)
        self.assertEqual([ident for ident, _ in self.piper.frames], [0x151, 0x155, 0x156, 0x157])
        old = self.control.status(); tick = self.clock.time()
        self.assertTrue(self.review()[0])
        self.assertGreaterEqual(self.clock.time()-tick, 3.)
        self.assertTrue(self.node.adopted)
        self.assertEqual(self.session["recovery_stage"], "approved")
        self.assertGreaterEqual(self.session["generations"][0]["interior_window"]["duration_s"], 3.)
        with self.assertRaisesRegex(RuntimeError, "retired"):
            self.control.review_recovery(old["sequence"], old["generation"], old["result_sha256"])
        self.assertEqual(len(self.piper.frames), 4)

    def test_zero_feedback_within_arrival_tolerance_cannot_complete_or_retry(self):
        self.piper.stay_boundary = True
        with self.assertRaisesRegex(RuntimeError, "interior window"):
            self.control.execute(self.message())
        self.assertEqual(self.piper.q[1:3], [0, 0])
        self.assertIsNotNone(self.session["failure"])
        self.assertIsNone(self.control.status()["recovery_review_service"])
        with self.assertRaises(RuntimeError): self.review()
        with self.assertRaises(RuntimeError): self.control.execute(self.message())
        self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 4)

    def test_one_zero_group_resets_entire_interior_window(self):
        zero_at = []
        def hook(state):
            if (self.piper.goal is None and self.piper.targets and not zero_at
                    and self.control.status()["phase"] == "moving"):
                # Wait until 1.5 seconds of otherwise interior arrival samples.
                receipt = self.control.status()["receipts"][0]
                if self.clock.time()-receipt["finished_unix_s"] >= 1.5:
                    state["raw_q"][1] = 0; state["q"][1] = 0.
                    state["pose"] = fixtures.fk(state["q"])
                    zero_at.append(self.clock.time())
        self.piper.on_read = hook
        result = self.control.execute(self.message())
        self.assertEqual(len(zero_at), 1)
        self.assertGreaterEqual(self.clock.time()-zero_at[0], 3.)
        self.assertGreater(result["stable_window"]["first_fragment_unix_s"], zero_at[0])

    def test_review_return_to_zero_cannot_promote_and_latches(self):
        self.control.execute(self.message())
        self.piper.q[1:3] = [0, 0]; self.piper.goal = list(self.piper.q)
        with self.assertRaisesRegex(RuntimeError, "interior window"): self.review()
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        with self.assertRaises(RuntimeError): self.review()
        self.assertEqual(len(self.piper.frames), 4)

    def test_wrong_first_target_consumes_only_attempt_with_zero_frames(self):
        raw = list(entry.FIXED_TARGET); raw[1] += 1
        with self.assertRaisesRegex(RuntimeError, "exact interior"): self.control.execute(self.message(raw))
        self.assertTrue(self.session["pilot_attempted"])
        self.assertIsNotNone(self.session["failure"])
        with self.assertRaises(RuntimeError): self.control.execute(self.message())
        self.assertEqual(self.piper.frames, [])

    def test_partial_send_cannot_retry_review_or_move_jaw(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"): self.control.execute(self.message())
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (2, 1))
        for action in (lambda: self.control.execute(self.message()), self.review,
                       lambda: self.control.gripper(types.SimpleNamespace())):
            with self.assertRaises(RuntimeError): action()
        self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 1)

    def test_review_rejects_hold_pending_failure_or_partial_receipt_without_tx(self):
        self.control.execute(self.message())
        session, state = copy.deepcopy(self.session), copy.deepcopy(self.control.state)
        for case in ("hold", "pending", "failure", "partial"):
            self.session.clear(); self.session.update(copy.deepcopy(session)); self.control.state = copy.deepcopy(state)
            if case == "hold": self.session["stop_latched"] = True
            elif case == "pending": self.session["pending"] = {"unknown": True}
            elif case == "failure": self.session["failure"] = {"error": "failure"}
            else: self.control.state["receipts"][0]["socket_send_returns"] = 3
            with self.subTest(case=case), self.assertRaises(RuntimeError): self.review()
            self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 4)

    def test_jaw_blocked_until_review_then_ordinary_j_and_jaw_use_original_guard(self):
        request = types.SimpleNamespace(gripper_angle=.035, gripper_effort=.2, gripper_code=1, set_zero=0)
        with self.assertRaisesRegex(RuntimeError, "Jaw forbidden"): self.control.gripper(request)
        self.control.execute(self.message())
        with self.assertRaisesRegex(RuntimeError, "Jaw forbidden"): self.control.gripper(request)
        self.review()
        raw = list(entry.FIXED_TARGET); raw[1] = 300; raw[2] = -300
        self.assertEqual(self.control.execute(self.message(raw))["phase"], "completed")
        self.control.gripper(request)
        self.assertEqual([f[0] for f in self.piper.frames], [0x151, 0x155, 0x156, 0x157]*2+[0x159])


for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_") and _name not in InteriorTests.__dict__:
        setattr(InteriorTests, _name, None)


if __name__ == "__main__": unittest.main()
