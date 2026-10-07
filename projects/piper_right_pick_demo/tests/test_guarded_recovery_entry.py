"""Offline recovery authorization/state tests; no physical dynamics or I/O."""
import copy
import hashlib
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
    "recovery_under_test", Path(__file__).parents[1] / "scripts/ros_guarded_recovery_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


def reviewed():
    return dict(parent_session_sha256=entry.PARENT_SHA,
                parent_action_result_sha256=entry.RESULT_SHA,
                independent_can_sha256=entry.CAN_SHA, parent_sequence=52,
                parent_adoption_token=entry.PARENT_TOKEN,
                user_authorization=entry.guard.AUTHORIZATION)


def evidence():
    """Assigned raw CAN evidence, independent of any installed robot/session."""
    receipt = dict(kind="initial", attempted_frames=4, socket_send_returns=4,
                   target_raw=list(entry.FIXED_TARGET), finished_unix_s=100.)
    result = dict(failure="Outside original probe joint box", sequence=52,
                  phase="failed", active=False, hold_requested=False,
                  hold_confirmed=False, target_raw=list(entry.FIXED_TARGET),
                  receipts=[receipt])
    parent = dict(identity=dict(source_sha256=entry.GUARD_SHA,
                                config_sha256=entry.CONFIG_SHA,
                                adoption_token=entry.PARENT_TOKEN),
                  status=result, failure={"error": result["failure"]},
                  pending=dict(sequence=52, kind="initial", target_raw=list(entry.FIXED_TARGET),
                               attempted_frames=4), generation=7)
    payloads = {0x2a1: bytes.fromhex("0100010000000000"),
                0x2a8: struct.pack(">iHBB", 34440, 0, 64, 0)}
    for i in range(3):
        payloads[0x2a2+i] = struct.pack(">ii", *([250000, 0, 250000, 0, 0, 0][i*2:i*2+2]))
        payloads[0x2a5+i] = struct.pack(">ii", *entry.FIXED_TARGET[i*2:i*2+2])
    for ident in range(0x261, 0x267):
        payloads[ident] = bytes([0, 0, 0, 0, 0, 64, 0, 0])
    rows = []
    for i in range(100):
        sampled = 200. + i*.04
        rows.append(dict(sampled_at=sampled, frames=[
            dict(id=ident, data_hex=data.hex(), timestamp=sampled-.001,
                 timestamp_basis="kernel_socket_SO_TIMESTAMPNS_unix")
            for ident, data in payloads.items()]))
    trace = dict(frames_sent_by_this_script=0, trace_transport_clean=True,
                 kernel_timestamp_enabled=True, socket_dropped_total=0,
                 control_frames=[], bad_frames=[], missing_feedback_ids=[],
                 timestamp_backwards=[], sample_gaps=[], samples=rows)
    return parent, copy.deepcopy(result), trace


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        loader = patch.object(entry.support, "load_base", return_value=fixtures.base)
        loader.start(); self.addCleanup(loader.stop)

    def test_complete_fresh_independent_evidence_decodes_original_target(self):
        endpoint = entry.check_evidence(*evidence(), reviewed())
        self.assertEqual(endpoint["raw_q"], entry.FIXED_TARGET)
        self.assertEqual(endpoint["jaw_code"], 64)
        self.assertAlmostEqual(endpoint["opening_m"], .03444)

    def test_frozen_real_saved_evidence_is_accepted_read_only(self):
        boot = entry.guard.predecessor.PARENT_BOOT
        parent_path = entry.guard.v1.SESSION_ROOT/("guarded_task_"+boot+".json")
        result_path = entry.PARENT_RUN/"runtime/action_000052_result.json"
        trace_path = entry.PARENT_RUN/"home051_post_failure_can.json"
        self.assertEqual(entry.sha(parent_path), entry.PARENT_SHA)
        self.assertEqual(entry.sha(result_path), entry.RESULT_SHA)
        self.assertEqual(entry.sha(trace_path), entry.CAN_SHA)
        endpoint = entry.check_evidence(json.loads(parent_path.read_text()),
                                        json.loads(result_path.read_text()),
                                        json.loads(trace_path.read_text()), reviewed())
        self.assertEqual(endpoint["raw_q"], [69, 33714, -26383, 0, 7137, -3520])

    def test_every_explicit_review_binding_is_required(self):
        for key in reviewed():
            consent = reviewed(); consent.pop(key)
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, "Explicit reviewed"):
                entry.check_evidence(*evidence(), consent)

    def test_different_failure_identity_pending_or_partial_receipt_rejected(self):
        for case in ("source", "config", "token", "failure", "sequence", "pending", "partial", "second_receipt"):
            parent, result, trace = evidence()
            if case in ("source", "config", "token"):
                key = {"source": "source_sha256", "config": "config_sha256", "token": "adoption_token"}[case]
                parent["identity"][key] = "other"
            elif case == "failure":
                result["failure"] = "another failure"; parent["failure"]["error"] = result["failure"]
            elif case == "sequence": result["sequence"] = 51
            elif case == "pending": parent["pending"]["attempted_frames"] = 3
            elif case == "partial": result["receipts"][0]["socket_send_returns"] = 3
            elif case == "second_receipt": result["receipts"].append(copy.deepcopy(result["receipts"][0]))
            parent["status"] = copy.deepcopy(result)
            with self.subTest(case=case), self.assertRaises(RuntimeError):
                entry.check_evidence(parent, result, trace, reviewed())

    def test_missing_stale_pre_send_unhealthy_or_moving_evidence_rejected(self):
        for case in ("missing", "stale", "pre_send", "health", "target", "changed", "short", "transport"):
            parent, result, trace = evidence()
            row = trace["samples"][30]
            if case == "missing": row["frames"].pop()
            elif case == "stale": row["frames"][0]["timestamp"] -= .101
            elif case == "pre_send":
                result["receipts"][0]["finished_unix_s"] = 201.; parent["status"] = copy.deepcopy(result)
            elif case == "health": row["frames"][0]["data_hex"] = "0100010001000000"
            elif case in ("target", "changed"):
                frame = next(f for f in row["frames"] if f["id"] == 0x2a5)
                delta = 1000 if case == "target" else 1
                frame["data_hex"] = struct.pack(">ii", entry.FIXED_TARGET[0]+delta, entry.FIXED_TARGET[1]).hex()
            elif case == "short": trace["samples"] = trace["samples"][:70]
            elif case == "transport": trace["socket_dropped_total"] = 1
            with self.subTest(case=case), self.assertRaises(RuntimeError):
                entry.check_evidence(parent, result, trace, reviewed())

    def test_one_child_only_and_immutable_parent_bytes(self):
        parent, result, trace = evidence()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); boot = entry.guard.predecessor.PARENT_BOOT
            names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json"]
            contents = [b'{"ancestor":1}', b'{"ancestor":2}', json.dumps(parent).encode()]
            hashes = [hashlib.sha256(raw).hexdigest() for raw in contents]
            for name, raw in zip(names, contents): (root/name).write_bytes(raw)
            with patch.object(entry.guard.predecessor, "PARENT_SHA", hashes[0]), \
                    patch.object(entry.guard, "SUCCESS_SESSION_SHA", hashes[1]), \
                    patch.object(entry, "PARENT_SHA", hashes[2]):
                with entry.reserve(boot, reviewed(), result, trace, session_root=root) as (_, endpoint, store, session):
                    self.assertEqual(endpoint["raw_q"], entry.FIXED_TARGET)
                    self.assertEqual(session["generation"], 8)
                    self.assertEqual(session["recovery_stage"], "pilot")
                    self.assertTrue(session["parent_failure_preserved"])
                    self.assertFalse(session["pilot_attempted"])
                    self.assertIsNotNone(store.load())
                with self.assertRaisesRegex(RuntimeError, "already exists"):
                    with entry.reserve(boot, reviewed(), result, trace, session_root=root): pass
            self.assertEqual([(root/name).read_bytes() for name in names], contents)


class RecoveryTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp(); self.control.feedback.close()
        self.piper = guarded_fixtures.MultiPiper(self.clock)
        self.piper.q = [69, 33714, -26383, 0, 7137, -3520]
        self.node.piper = self.piper
        self.session.update(stage="task", generation=8, generations=[],
                            commissioning_attempted=True, held_raw=list(self.piper.q),
                            recovery_stage="pilot", pilot_attempted=False,
                            pilot_anchor_raw=list(self.piper.q))
        def register(name, callback): self.services[name] = callback; return name
        self.control = entry.RecoveryTask(
            self.node, types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            fixtures.limits(), fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_driver", token="recovery_token")
        self.addCleanup(self.control.feedback.close)

    def message(self, delta=700):
        anchor = self.session["pilot_anchor_raw"]
        raw = list(entry.FIXED_TARGET); scale = 1-delta/anchor[1]
        raw[1] = round(anchor[1]*scale); raw[2] = round(anchor[2]*scale)
        return types.SimpleNamespace(position=[v*entry.support.RAD_PER_RAW for v in raw],
                                     velocity=[0.]*6+[1.], effort=[])

    def review(self):
        state = self.control.status()
        return self.control.review_recovery(state["sequence"], state["generation"], state["result_sha256"])

    def jaw(self):
        return types.SimpleNamespace(gripper_angle=.035, gripper_effort=.2, gripper_code=1, set_zero=0)

    def test_valid_pilot_sends_four_frames_and_waits_for_explicit_review(self):
        result = self.control.execute(self.message())
        self.assertEqual(result["phase"], "completed")
        self.assertTrue(result["target_reached"])
        self.assertGreaterEqual(result["stable_window"]["duration_s"], 3.)
        self.assertGreaterEqual(result["stable_window"]["new_feedback_groups"], 20)
        self.assertEqual([ident for ident, _ in self.piper.frames], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.session["recovery_stage"], "awaiting_review")
        self.assertFalse(self.node.adopted)
        self.assertIsNotNone(self.control.status()["recovery_review_service"])
        with self.assertRaises(RuntimeError): self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames), 4)
        start = self.clock.time()
        self.assertTrue(self.review()[0])
        self.assertGreaterEqual(self.clock.time()-start, 3.)
        self.assertTrue(self.node.adopted)
        self.assertEqual(self.session["recovery_stage"], "approved")
        self.assertEqual(self.session["generation"], 9)
        self.assertTrue(self.session["generations"][0]["parent_failure_preserved"])
        self.assertEqual(len(self.piper.frames), 4)

    def test_jaw_and_ordinary_resume_forbidden_before_review(self):
        with self.assertRaisesRegex(RuntimeError, "Jaw forbidden"): self.control.gripper(self.jaw())
        with self.assertRaisesRegex(RuntimeError, "ordinary hold"): self.control.resume(0, 8, "none")
        self.assertEqual(self.piper.frames, [])
        self.control.execute(self.message())
        with self.assertRaisesRegex(RuntimeError, "Jaw forbidden"): self.control.gripper(self.jaw())
        self.assertEqual(len(self.piper.frames), 4)
        self.review(); self.control.gripper(self.jaw())
        self.assertEqual([f[0] for f in self.piper.frames], [0x151, 0x155, 0x156, 0x157, 0x159])

    def test_pilot_target_requires_fixed_other_axes_common_ratio_and_bounded_step(self):
        before = self.control.read(); anchor = self.session["pilot_anchor_raw"]
        for case in ("wrist", "ratio", "away", "large", "single_axis", "short"):
            msg = self.message()
            if case == "wrist": msg.position[4] += entry.support.RAD_PER_RAW
            elif case == "ratio": msg.position[2] += 10*entry.support.RAD_PER_RAW
            elif case == "away": msg = self.message(-200)
            elif case == "large": msg = self.message(801)
            elif case == "single_axis": msg.position[2] = before["q"][2]
            elif case == "short": msg.position.pop()
            with self.subTest(case=case), self.assertRaises(RuntimeError):
                entry.pilot_target(msg, before, anchor)
        self.assertEqual(self.piper.frames, [])

    def test_invalid_first_pilot_consumes_chance_without_any_transmission(self):
        msg = self.message(); msg.position[4] += .01
        with self.assertRaises(RuntimeError): self.control.execute(msg)
        self.assertTrue(self.session["pilot_attempted"])
        self.assertIsNotNone(self.session["failure"])
        with self.assertRaises(RuntimeError): self.control.execute(self.message())
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])

    def test_both_pilot_axes_need_at_least_point_two_degree_planned_progress(self):
        before = self.control.read(); anchor = self.session["pilot_anchor_raw"]
        for delta in (0, 1, 100, 199, 200):
            # With unequal J2/J3 magnitudes, J2=.2deg leaves J3 below .2deg.
            with self.subTest(delta=delta), self.assertRaises(RuntimeError):
                entry.pilot_target(self.message(delta), before, anchor)
        raw = entry.pilot_target(self.message(260), before, anchor)
        self.assertGreaterEqual(before["raw_q"][1]-raw[1], 200)
        self.assertGreaterEqual(raw[2]-before["raw_q"][2], 200)
        self.assertEqual(self.piper.frames, [])

    def test_within_arrival_tolerance_but_insufficient_actual_progress_cannot_promote(self):
        original = self.piper.JointCtrl
        def undershoot(*raw):
            original(*raw)
            # Within .003rad arrival tolerance, but J3 progress below .2deg.
            self.piper.goal[1] += 100
            self.piper.goal[2] -= 100
        self.piper.JointCtrl = undershoot
        with self.assertRaisesRegex(RuntimeError, "actual progress"):
            self.control.execute(self.message(260))
        self.assertIsNotNone(self.session["failure"])
        self.assertIsNone(self.control.status()["recovery_review_service"])
        with self.assertRaises(RuntimeError): self.review()
        with self.assertRaises(RuntimeError): self.control.execute(self.message())
        self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 4)

    def test_wrong_speed_is_rejected_before_send_and_latched(self):
        msg = self.message(); msg.velocity[-1] = 50
        with self.assertRaises(RuntimeError): self.control.execute(msg)
        self.assertIsNotNone(self.session["failure"])
        self.assertEqual(self.piper.frames, [])

    def test_partial_send_cannot_retry_or_review_or_send_jaw(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"): self.control.execute(self.message())
        self.assertIsNotNone(self.session["pending"])
        self.assertIsNotNone(self.session["failure"])
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (2, 1))
        self.assertIsNone(self.piper.ticket)
        for call in (lambda: self.control.execute(self.message()), self.review,
                     lambda: self.control.gripper(self.jaw())):
            with self.assertRaises(RuntimeError): call()
        self.assertEqual(len(self.piper.frames), 1)

    def test_review_rejects_partial_pending_failure_changed_result_and_stale_identity(self):
        self.control.execute(self.message())
        initial_session = copy.deepcopy(self.session); initial_state = copy.deepcopy(self.control.state)
        for case in ("partial", "pending", "failure", "digest", "generation", "sequence", "not_arrived"):
            self.session.clear(); self.session.update(copy.deepcopy(initial_session))
            self.control.state = copy.deepcopy(initial_state)
            state = self.control.status(); seq, gen, digest = state["sequence"], state["generation"], state["result_sha256"]
            if case == "partial": self.control.state["receipts"][0]["socket_send_returns"] = 3
            elif case == "pending": self.session["pending"] = {"unknown": True}
            elif case == "failure": self.session["failure"] = {"error": "failure"}
            elif case == "digest": digest = "wrong"
            elif case == "generation": gen += 1
            elif case == "sequence": seq += 1
            elif case == "not_arrived": self.control.state["result"]["target_reached"] = False
            with self.subTest(case=case), self.assertRaises(RuntimeError):
                self.control.review_recovery(seq, gen, digest)
            self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 4)

    def test_review_result_file_changed_is_rejected_without_frames(self):
        self.control.execute(self.message())
        path = self.path/"action_000001_result.json"; path.write_text(path.read_text()+"\n")
        with self.assertRaisesRegex(RuntimeError, "result changed"): self.review()
        self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 4)

    def test_review_fresh_slip_latches_failure_without_recovery_command(self):
        self.control.execute(self.message()); self.piper.q[0] += 200; self.piper.goal = list(self.piper.q)
        with self.assertRaisesRegex(RuntimeError, "slip"): self.review()
        self.assertIsNotNone(self.session["failure"])
        with self.assertRaises(RuntimeError): self.review()
        self.assertFalse(self.node.adopted); self.assertEqual(len(self.piper.frames), 4)

    def test_review_cannot_race_action_or_be_reused(self):
        self.control.execute(self.message()); state = self.control.status()
        with self.node.action_lock:
            with self.assertRaisesRegex(RuntimeError, "still active"): self.review()
        self.review()
        with self.assertRaisesRegex(RuntimeError, "retired"):
            self.control.review_recovery(state["sequence"], state["generation"], state["result_sha256"])
        self.assertEqual(len(self.piper.frames), 4)


# Reuse setup/helpers, not the different predecessor admission contract's tests.
for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_") and _name not in RecoveryTests.__dict__:
        setattr(RecoveryTests, _name, None)


if __name__ == "__main__":
    unittest.main()
