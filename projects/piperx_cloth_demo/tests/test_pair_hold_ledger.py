"""Pure SQLite same-worker hold facts; every socket is forbidden."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from robot_tools.hold_transaction import JointHoldTransaction, RAD_PER_RAW, joint_hold_frames
from robot_tools.pair_ledger import PairLedger, PairLedgerFault, platform_state
from test_hold_transaction import IDENTITY, RAW_GOAL, RAW_HOLD, event_and_claim
from test_pair_ledger import Clock


class PairHoldLedgerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.no_socket = patch("socket.socket", side_effect=AssertionError("Hardware forbidden"))
        self.no_socket.start()
        self.addCleanup(self.no_socket.stop)
        self.path = Path(self.directory.name) / "pair.sqlite"
        self.clock = Clock()
        self.contract = {"task": "offline-hold-test"}
        self.ledger = self.open()
        self.owner = IDENTITY["owner"]
        self.original, _ = event_and_claim()
        self.original.update(fault=None, deadline_at=1000.)
        self.event = self.original["event_id"]
        self.action = {"kind": "joint", "arm": "right", "operation": "approach",
                       "target": [value * RAD_PER_RAW for value in RAW_GOAL]}
        self.ledger.claim(self.owner)
        self.ledger.begin(self.owner, self.event, self.action)
        self.payload = {"operation": "joint_hold_current", "identity": copy.deepcopy(IDENTITY),
                        "target_raw": list(RAW_HOLD), "expected_frames": joint_hold_frames(RAW_HOLD),
                        "sample_ref": {"sample_id": "sample-current", "captured_at": 100.23, "sha256": "a"*64}}

    def open(self):
        return PairLedger(self.path, "run-1", self.contract, clock=self.clock)

    def source(self):
        self.clock.now = 100.2
        return self.ledger.record_original_send(self.owner, self.event, self.original)

    def request(self):
        self.clock.now = 100.21
        return self.ledger.request_hold_cancel(self.owner, self.event, "client requested cancellation")

    def begin(self):
        self.source()
        self.request()
        self.clock.now = 100.23
        return self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def frames(self):
        for index, frame in enumerate(self.payload["expected_frames"]):
            self.clock.now += .002
            self.ledger.begin_hold_frame(self.owner, "hold-1", index, frame)
            self.clock.now += .001
            self.ledger.finish_hold_frame(self.owner, "hold-1", index, "returned")

    def receipt(self, *, complete=True, observed=True):
        return {"hold_event_id": "hold-1", "original_event_id": self.event,
                "identity": copy.deepcopy(IDENTITY), "frames_complete": complete,
                "hold_observed": observed, "physical_stop_verified": None, "original_target_cancelled": None,
                "target_raw": list(RAW_HOLD), "expected_frames": copy.deepcopy(self.payload["expected_frames"])}

    def test_complete_same_worker_claim_matches_pure_helper_and_preserves_original_budget_fault(self):
        result = self.begin()
        before = self.ledger.status()
        self.assertEqual(before["steps"], 2)
        self.assertEqual(before["deadline_s"], 1000.)
        self.assertEqual(result["original_event"]["fault"]["reason"], before["fault"]["reason"])
        transaction = JointHoldTransaction(result["original_event"], result["claim"])
        self.assertEqual(transaction.report()["status"], "claimed")
        self.assertEqual(result["claim"]["original_event_sha256"], hashlib.sha256(json.dumps(
            result["original_event"], sort_keys=True, separators=(",", ":")).encode()).hexdigest())
        self.frames()
        self.ledger.finish_hold(self.owner, "hold-1", self.receipt())
        after = self.ledger.status()
        self.assertEqual(after["steps"], before["steps"])
        self.assertEqual(after["fault"], before["fault"])
        self.assertIsNone(after["physical_stop_verified"])
        self.assertEqual(self.ledger.event(self.event)["status"], "pending")
        self.assertEqual([r["outcome"] for r in self.ledger.hold_event("hold-1")["frame_receipts"]], ["returned"]*4)
        self.assertEqual(platform_state(self.path)["pending_events"][0]["event_id"], self.event)

    def test_full_original_facts_can_arrive_after_explicit_cancellation(self):
        self.request()
        self.clock.now = 100.22
        self.ledger.record_original_send(self.owner, self.event, self.original)
        self.clock.now = 100.23
        self.assertFalse(self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)["replayed"])

    def test_partial_original_has_no_hold_source_and_cannot_be_upgraded(self):
        self.original["frame_receipts"].pop()
        with self.assertRaisesRegex(PairLedgerFault, "original_send_incomplete"):
            self.source()
        self.assertEqual(self.ledger.status()["steps"], 1)
        with self.assertRaises(PairLedgerFault):
            self.request()
        self.assertIsNone(self.ledger.hold_event("hold-1"))

    def test_unknown_original_return_is_not_complete(self):
        self.original["frame_receipts"][2]["outcome"] = "unknown"
        with self.assertRaisesRegex(PairLedgerFault, "original_send_incomplete"):
            self.source()

    def test_original_target_frames_bound_to_original_radians(self):
        self.original["target_raw"][1] += 1
        self.original["frame_receipts"] = [dict(frame=frame, outcome="returned", returned_at=100.1+i*.001)
                                          for i, frame in enumerate(joint_hold_frames(self.original["target_raw"]))]
        with self.assertRaisesRegex(PairLedgerFault, "original_target_binding"):
            self.source()

    def test_original_extended_frame_is_not_whitelisted(self):
        self.original["frame_receipts"][0]["frame"]["is_extended_id"] = True
        with self.assertRaisesRegex(PairLedgerFault, "original_send_incomplete"):
            self.source()

    def test_sdk_accepted_quantization_uses_exact_official_formulas(self):
        target = copy.deepcopy(self.action["target"])
        target[0] = 0.010602875205865551  # SDK's three encodings agree on 607, a fourth formula gives 608.
        ledger = PairLedger(Path(self.directory.name)/"rounding.sqlite", "run-1", self.contract, clock=self.clock)
        ledger.claim(self.owner)
        ledger.begin(self.owner, self.event, {**self.action, "target": target})
        self.ledger = ledger
        self.original["target_raw"][0] = 607
        for receipt, frame in zip(self.original["frame_receipts"], joint_hold_frames(self.original["target_raw"])):
            receipt["frame"] = frame
        self.assertTrue(self.source()["recorded"])

    def test_original_fault_field_cannot_supply_a_cancellation(self):
        self.original["fault"] = {"reason": "user_cancel", "event_id": "fake", "at": 100.1}
        with self.assertRaisesRegex(PairLedgerFault, "original_send_binding"):
            self.source()

    def test_source_is_immutable_even_with_a_new_full_send_description(self):
        self.source()
        self.original["limits"]["max_translation_m"] = .02
        with self.assertRaisesRegex(PairLedgerFault, "original_send_conflict"):
            self.source()

    def test_source_replay_does_not_rebind_worker_or_budget(self):
        self.assertFalse(self.source()["replayed"])
        self.assertTrue(self.source()["replayed"])
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_reopened_ledger_cannot_reconstruct_original_worker_before_record(self):
        reopened = self.open()
        self.clock.now = 100.2
        with self.assertRaisesRegex(PairLedgerFault, "without_live_action"):
            reopened.record_original_send(self.owner, self.event, self.original)
        self.assertEqual(reopened.status()["steps"], 1)

    def test_reopened_ledger_cannot_reconstruct_worker_after_record(self):
        self.source()
        self.request()
        reopened = self.open()
        self.clock.now = 100.23
        with self.assertRaisesRegex(PairLedgerFault, "original_live_worker"):
            reopened.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def test_different_live_thread_cannot_claim_hold(self):
        self.source()
        self.request()
        self.clock.now = 100.23
        errors = []
        def foreign_worker():
            try:
                self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)
            except PairLedgerFault as exc:
                errors.append(str(exc))
        thread = threading.Thread(target=foreign_worker)
        thread.start()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIn("original_live_worker", errors[0])
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_forked_process_identity_cannot_claim_hold(self):
        self.source()
        self.request()
        self.clock.now = 100.23
        with patch("robot_tools.pair_ledger.os.getpid", return_value=-1):
            with self.assertRaisesRegex(PairLedgerFault, "original_live_worker"):
                self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def test_different_owner_cannot_borrow_hold(self):
        self.source()
        self.request()
        self.clock.now = 100.23
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin_hold("owner-2", self.event, "hold-1", self.payload)
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_ordinary_fault_cannot_be_relabelled_explicit_cancel(self):
        self.source()
        fault = self.ledger.fault(self.owner, "feedback_lost")["fault"]
        with self.assertRaisesRegex(PairLedgerFault, "Preexisting fault"):
            self.request()
        self.assertEqual(self.ledger.status()["fault"], fault)

    def test_full_send_without_explicit_cancel_cannot_claim_hold(self):
        self.source()
        self.clock.now = 100.23
        with self.assertRaisesRegex(PairLedgerFault, "only_explicit_cancel"):
            self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def test_later_fault_cannot_hide_behind_first_cancel_fault(self):
        self.source()
        first = self.request()["fault"]
        self.ledger.fault(self.owner, "watchdog_feedback_lost")
        self.clock.now = 100.23
        with self.assertRaisesRegex(PairLedgerFault, "only_explicit_cancel"):
            self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)
        self.assertEqual(self.ledger.status()["fault"], first)

    def test_cancel_request_without_full_source_does_not_grant_hold(self):
        self.request()
        self.clock.now = 100.23
        with self.assertRaisesRegex(PairLedgerFault, "original_live_worker"):
            self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def test_pending_hold_replay_latches_and_never_spends_twice(self):
        self.begin()
        with self.assertRaisesRegex(PairLedgerFault, "pending_hold_cannot_replay"):
            self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)
        self.assertEqual(self.ledger.status()["steps"], 2)

    def test_second_id_cannot_create_another_hold_for_same_action(self):
        self.begin()
        with self.assertRaisesRegex(PairLedgerFault, "already_reserved"):
            self.ledger.begin_hold(self.owner, self.event, "hold-2", self.payload)
        self.assertEqual(self.ledger.status()["steps"], 2)

    def test_completed_replay_returns_old_receipt_without_fresh_sample_or_budget(self):
        self.begin()
        self.frames()
        receipt = self.receipt()
        self.ledger.finish_hold(self.owner, "hold-1", receipt)
        self.clock.now += 20
        result = self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)
        self.assertTrue(result["replayed"])
        self.assertEqual(result["receipt"], receipt)
        self.assertEqual(self.ledger.status()["steps"], 2)

    def test_changed_completed_payload_is_not_replay(self):
        self.begin()
        self.frames()
        self.ledger.finish_hold(self.owner, "hold-1", self.receipt())
        self.payload["sample_ref"]["sample_id"] = "different"
        with self.assertRaisesRegex(PairLedgerFault, "payload_conflict"):
            self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def test_reopened_pending_hold_can_only_be_read_not_resumed(self):
        self.begin()
        self.ledger.begin_hold_frame(self.owner, "hold-1", 0, self.payload["expected_frames"][0])
        reopened = self.open()
        read = reopened.hold_event("hold-1")
        self.assertEqual(read["frame_receipts"][0]["outcome"], "pending")
        self.assertFalse(read["dispatch_authorized"])
        with self.assertRaisesRegex(PairLedgerFault, "original_live_worker"):
            reopened.finish_hold_frame(self.owner, "hold-1", 0, "returned")

    def test_repeated_or_skipped_frame_never_grants_next_send(self):
        self.begin()
        self.ledger.begin_hold_frame(self.owner, "hold-1", 0, self.payload["expected_frames"][0])
        with self.assertRaisesRegex(PairLedgerFault, "frame_order"):
            self.ledger.begin_hold_frame(self.owner, "hold-1", 1, self.payload["expected_frames"][1])
        self.assertEqual(len(self.ledger.hold_event("hold-1")["frame_receipts"]), 1)

    def test_wrong_frame_payload_is_refused_before_persistence(self):
        self.begin()
        wrong = copy.deepcopy(self.payload["expected_frames"][0])
        wrong["data_hex"] = "0101020000000000"
        with self.assertRaisesRegex(PairLedgerFault, "frame_order_or_payload"):
            self.ledger.begin_hold_frame(self.owner, "hold-1", 0, wrong)
        self.assertEqual(self.ledger.hold_event("hold-1")["frame_receipts"], [])

    def test_exception_return_preserves_first_fault_and_blocks_remaining_frames(self):
        self.begin()
        first = self.ledger.status()["fault"]
        self.ledger.begin_hold_frame(self.owner, "hold-1", 0, self.payload["expected_frames"][0])
        self.ledger.finish_hold_frame(self.owner, "hold-1", 0, "exception", error="transport failure")
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin_hold_frame(self.owner, "hold-1", 1, self.payload["expected_frames"][1])
        self.ledger.finish_hold(self.owner, "hold-1", self.receipt(complete=False, observed=False))
        self.assertEqual(self.ledger.status()["fault"], first)
        self.assertEqual(self.ledger.hold_event("hold-1")["frame_receipts"][0]["outcome"], "exception")

    def test_unknown_return_cannot_be_changed_to_returned(self):
        self.begin()
        self.ledger.begin_hold_frame(self.owner, "hold-1", 0, self.payload["expected_frames"][0])
        self.ledger.finish_hold_frame(self.owner, "hold-1", 0, "unknown")
        with self.assertRaisesRegex(PairLedgerFault, "without_pending_frame"):
            self.ledger.finish_hold_frame(self.owner, "hold-1", 0, "returned")
        self.assertEqual(self.ledger.hold_event("hold-1")["frame_receipts"][0]["outcome"], "unknown")

    def test_late_frame_return_can_be_recorded_but_deadline_never_renews(self):
        self.begin()
        self.ledger.begin_hold_frame(self.owner, "hold-1", 0, self.payload["expected_frames"][0])
        self.clock.now = 1000.
        self.ledger.finish_hold_frame(self.owner, "hold-1", 0, "returned")
        with self.assertRaisesRegex(PairLedgerFault, "deadline_exhausted"):
            self.ledger.begin_hold_frame(self.owner, "hold-1", 1, self.payload["expected_frames"][1])
        self.assertEqual(self.ledger.status()["deadline_s"], 1000.)

    def test_frozen_deadline_cannot_be_replaced_in_source(self):
        self.original["deadline_at"] = 1100.
        with self.assertRaisesRegex(PairLedgerFault, "deadline_changed"):
            self.source()

    def test_stale_or_pre_cancel_sample_ref_is_refused(self):
        self.source()
        self.request()
        self.clock.now = 100.23
        self.payload["sample_ref"]["captured_at"] = 100.2
        with self.assertRaisesRegex(PairLedgerFault, "sample_not_current"):
            self.ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)

    def test_step_budget_is_shared_with_original_action(self):
        # A separate fixture freezes max_steps=1 before any attempt.
        ledger = PairLedger(Path(self.directory.name)/"one.sqlite", "run-1", self.contract,
                            clock=self.clock, max_steps=1)
        ledger.claim(self.owner)
        ledger.begin(self.owner, self.event, self.action)
        self.ledger = ledger
        self.source()
        self.request()
        self.clock.now = 100.23
        with self.assertRaisesRegex(PairLedgerFault, "step_budget_exhausted"):
            ledger.begin_hold(self.owner, self.event, "hold-1", self.payload)
        self.assertEqual(ledger.status()["steps"], 1)

    def test_finish_original_prevents_later_hold_frames(self):
        self.begin()
        self.ledger.finish(self.owner, self.event, {"ok": False}, success=False)
        with self.assertRaisesRegex(PairLedgerFault, "original_not_pending"):
            self.ledger.begin_hold_frame(self.owner, "hold-1", 0, self.payload["expected_frames"][0])

    def test_false_full_frame_receipt_and_physical_stop_claim_are_rejected(self):
        self.begin()
        with self.assertRaisesRegex(PairLedgerFault, "frame_evidence"):
            self.ledger.finish_hold(self.owner, "hold-1", self.receipt())
        receipt = self.receipt(complete=False, observed=False)
        receipt["physical_stop_verified"] = True
        with self.assertRaisesRegex(PairLedgerFault, "receipt_binding"):
            self.ledger.finish_hold(self.owner, "hold-1", receipt)

    def test_zero_frame_prepare_failure_accepts_unprepared_helper_diagnostics(self):
        claim = self.begin()
        transaction = JointHoldTransaction(claim["original_event"], claim["claim"])
        report = transaction.invalidate("preparation unavailable", now=self.clock.now)
        self.assertIsNone(report["expected_frames"])
        self.ledger.finish_hold(self.owner, "hold-1", report)
        stored = self.ledger.hold_event("hold-1")
        self.assertEqual(stored["status"], "complete")
        self.assertEqual(stored["frame_receipts"], [])
        self.assertFalse(stored["receipt"]["hold_observed"])

    def test_model_geometry_provenance_is_frozen_in_source_and_claim_digest(self):
        self.original["geometry_source"] = {"mode": "model_joint_geometry_v1", "model": "piper_x",
                                            "sdk_commit": "a"*40, "constants_sha256": "b"*64}
        result = self.begin()
        self.assertEqual(result["original_event"]["geometry_source"], self.original["geometry_source"])
        self.assertEqual(result["claim"]["original_event_sha256"], hashlib.sha256(json.dumps(
            result["original_event"], sort_keys=True, separators=(",", ":")).encode()).hexdigest())

    def test_geometry_wrong_model_cannot_qualify_source(self):
        self.original["geometry_source"] = {"mode": "model_joint_geometry_v1", "model": "piper",
                                            "sdk_commit": "a"*40, "constants_sha256": "b"*64}
        with self.assertRaisesRegex(PairLedgerFault, "geometry_binding"):
            self.source()

    def test_normal_dispatch_and_detach_remain_blocked_after_successful_hold(self):
        self.begin()
        self.frames()
        self.ledger.finish_hold(self.owner, "hold-1", self.receipt())
        self.ledger.finish(self.owner, self.event, {"cancelled": True})
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin(self.owner, "next-motion", self.action)
        with self.assertRaises(PairLedgerFault):
            self.ledger.release(self.owner)
        self.assertIsNone(self.ledger.status()["physical_stop_verified"])


if __name__ == "__main__":
    unittest.main()
