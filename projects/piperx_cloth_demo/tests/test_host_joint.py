"""Offline cancellation publication races; real CAN/network is forbidden."""
import copy
from enum import IntEnum
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from robot_tools.hold_transaction import RAD_PER_RAW, joint_hold_frames
from robot_tools.host_joint import CANCEL_PUBLICATION_WAIT_S, HostJointBridge, HostJointBridgeError
from robot_tools.pair_ledger import PairLedger, PairLedgerFault
from test_hold_transaction import IDENTITY, RAW_GOAL, RAW_HOLD, event_and_claim
from test_pair_ledger import Clock


class FakeHost:
    def __init__(self, ledger, clock, event):
        self.ledger, self.clock = ledger, clock
        self.owner, self.run_id, self.deadline = IDENTITY["owner"], IDENTITY["run_id"], 1000.
        self.active_event_id = event
        self.dispatch_rgb_deadline = 130.
        self.fault_event = threading.Event()
        self._fault_record_lock = threading.Lock()
        self.state_lock, self.device_lock = threading.RLock(), threading.RLock()
        self._fault_record_attempted = False
        self._fault_reason = self._fault_record_error = None

    def _fault(self, reason):
        self.fault_event.set()
        if not self._fault_record_lock.acquire(blocking=False):
            return
        try:
            if self._fault_record_attempted:
                return
            self._fault_record_attempted = True
            self._fault_reason = reason
            self.ledger.fault(self.owner, reason)
        finally:
            self._fault_record_lock.release()


class HostJointBridgeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        no_socket = patch("socket.socket", side_effect=AssertionError("Hardware forbidden"))
        no_socket.start()
        self.addCleanup(no_socket.stop)
        self.clock = Clock()
        self.ledger = PairLedger(Path(self.directory.name)/"pair.sqlite", "run-1", {}, clock=self.clock)
        self.original, _ = event_and_claim()
        self.original.update(fault=None, deadline_at=1000.)
        self.event = self.original["event_id"]
        self.host = FakeHost(self.ledger, self.clock, self.event)
        self.ledger.claim(self.host.owner)
        self.ledger.begin(self.host.owner, self.event, {"kind": "joint", "arm": "right",
            "target": [v*RAD_PER_RAW for v in RAW_GOAL]})
        self.bridge = HostJointBridge(self.host, self.event)
        self.payload = {"operation": "joint_hold_current", "identity": copy.deepcopy(IDENTITY),
                        "target_raw": list(RAW_HOLD), "expected_frames": joint_hold_frames(RAW_HOLD),
                        "sample_ref": {"sample_id": "current-feedback", "captured_at": 100.23, "sha256": "a"*64}}

    def source(self):
        self.clock.now = 100.2
        return self.bridge.record_original(self.original)

    def request(self, reason="User requested cancellation"):
        self.clock.now = 100.21
        return self.bridge.cancel(reason)

    def claimed(self):
        self.source()
        result = self.request()
        self.clock.now = 100.23
        claim = self.bridge.claim_hold(self.event, result["hold_event_id"], self.payload)
        return result["hold_event_id"], claim

    def test_bridge_has_no_side_effect_until_callbacks_and_initial_poll_is_empty(self):
        self.assertIsNone(self.bridge.cancellation_request())
        self.assertFalse(self.host.fault_event.is_set())
        self.assertEqual(self.ledger.status()["steps"], 1)
        self.assertIsNone(self.ledger.status()["fault"])

    def test_constructor_requires_actual_finite_rgb_deadline_without_task_fallback(self):
        for value in (None, True, float("nan"), float("inf"), -1., 99.):
            with self.subTest(value=value):
                self.host.dispatch_rgb_deadline = value
                with self.assertRaisesRegex(HostJointBridgeError, "RGB deadline"):
                    HostJointBridge(self.host, self.event)
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_expired_rgb_refuses_claim_without_an_additional_hold_step(self):
        self.source()
        request = self.request()
        self.clock.now = 130.001
        with self.assertRaisesRegex(HostJointBridgeError, "expired"):
            self.bridge.claim_hold(self.event, request["hold_event_id"], self.payload)
        self.assertEqual(self.ledger.peek_status()["steps"], 1)
        self.assertIsNone(self.ledger.hold_event(request["hold_event_id"]))
        self.assertTrue(self.host.fault_event.is_set())

    def test_expiry_final_check_is_sticky_and_late_factual_return_is_recorded(self):
        hold_id, _ = self.claimed()
        original_fault = self.ledger.peek_status()["fault"]
        self.bridge.record_frame_begin(hold_id, 0, self.payload["expected_frames"][0])
        self.clock.now = 130.001
        with patch.object(self.ledger, "fault", side_effect=AssertionError("No check IO")):
            with self.assertRaisesRegex(HostJointBridgeError, "expired"):
                self.bridge.check_active()
            self.clock.now = 100.24
            with self.assertRaisesRegex(HostJointBridgeError, "expired"):
                self.bridge.check_active()
        self.clock.now = 130.002
        self.bridge.record_frame_return(hold_id, 0, "exception", "Expired before actual CAN call")
        receipt = {"hold_event_id": hold_id, "original_event_id": self.event,
                   "identity": copy.deepcopy(IDENTITY), "frames_complete": False,
                   "hold_observed": False, "physical_stop_verified": None,
                   "original_target_cancelled": None}
        self.bridge.finish_hold(hold_id, receipt)
        self.assertEqual(self.ledger.hold_event(hold_id)["receipt"], receipt)
        self.assertEqual(self.ledger.peek_status()["fault"], original_fault)
        with self.assertRaises(HostJointBridgeError):
            self.bridge.record_frame_begin(hold_id, 1, self.payload["expected_frames"][1])

    def test_rgb_deadline_extension_or_clear_is_sticky_not_a_renewal(self):
        self.claimed()
        self.host.dispatch_rgb_deadline = 200.
        with self.assertRaisesRegex(HostJointBridgeError, "deadline changed"):
            self.bridge.check_active()
        self.host.dispatch_rgb_deadline = 130.
        with self.assertRaisesRegex(HostJointBridgeError, "deadline changed"):
            self.bridge.check_active()

    def test_cleared_rgb_deadline_cannot_use_original_run_deadline(self):
        self.claimed()
        self.host.dispatch_rgb_deadline = None
        with self.assertRaisesRegex(HostJointBridgeError, "deadline changed"):
            self.bridge.check_active()
        self.assertTrue(self.host.fault_event.is_set())

    def test_changed_original_task_deadline_is_sticky_even_if_restored(self):
        self.claimed()
        self.host.deadline = 1001.
        with self.assertRaisesRegex(HostJointBridgeError, "scope"):
            self.bridge.check_active()
        self.host.deadline = 1000.
        with self.assertRaisesRegex(HostJointBridgeError, "scope"):
            self.bridge.check_active()
        self.assertTrue(self.host.fault_event.is_set())

    def test_clock_exception_latches_hold_and_cannot_resume_with_working_clock(self):
        hold_id, _ = self.claimed()
        with patch.object(self.host, "clock", side_effect=RuntimeError("clock unavailable")):
            with self.assertRaisesRegex(HostJointBridgeError, "clock failed"):
                self.bridge.check_active()
        with self.assertRaisesRegex(HostJointBridgeError, "clock failed"):
            self.bridge.check_active()
        self.assertEqual(self.ledger.hold_event(hold_id)["frame_receipts"], [])

    def test_invalid_hold_clock_cannot_send_or_recover(self):
        self.claimed()
        with patch.object(self.host, "clock", return_value=float("nan")):
            with self.assertRaisesRegex(HostJointBridgeError, "clock"):
                self.bridge.check_active()
        with self.assertRaisesRegex(HostJointBridgeError, "clock"):
            self.bridge.check_active()

    def test_regressed_hold_clock_is_rejected(self):
        self.claimed()
        self.clock.now = 100.24
        self.bridge.check_active()
        self.clock.now = 100.23
        with self.assertRaisesRegex(HostJointBridgeError, "regressed"):
            self.bridge.check_active()
    def test_boolean_hold_clock_is_not_numeric_time(self):
        self.claimed()
        with patch.object(self.host, "clock", return_value=True):
            with self.assertRaisesRegex(HostJointBridgeError, "clock"):
                self.bridge.check_active()

    def test_cancel_blocks_normal_dispatch_and_publishes_only_typed_request(self):
        self.source()
        receipt = self.request()
        self.assertTrue(self.host.fault_event.is_set())
        self.assertTrue(self.host._fault_record_attempted)
        request = self.bridge.cancellation_request()
        self.assertEqual(request, {"hold_event_id": receipt["hold_event_id"], "reason": "explicit_client_cancel"})
        first = self.ledger.status()["fault"]
        self.host._fault("Ordinary failure caused by the software cancel")
        self.assertEqual(self.ledger.status()["fault"], first)
        self.clock.now = 100.23
        self.assertFalse(self.bridge.claim_hold(self.event, request["hold_event_id"], self.payload)["replayed"])

    def test_long_reason_is_retained_but_adapter_receives_short_reason_identifier(self):
        receipt = self.request("用户请求结束当前操作 " * 100)
        self.assertIn("用户请求", receipt["cancellation"]["reason"])
        self.assertEqual(self.bridge.cancellation_request()["reason"], "explicit_client_cancel")

    def test_same_cancel_replays_without_creating_another_hold_id(self):
        one = self.request()
        two = self.request()
        self.assertEqual(one, two)
        self.assertEqual(self.ledger.status()["steps"], 1)
        with self.assertRaisesRegex(HostJointBridgeError, "immutable"):
            self.bridge.cancel("different reason")

    def test_pending_publication_blocks_normal_fault_writer_and_does_not_return_none(self):
        self.source()
        self.clock.now = 100.21
        entered, release, observed = threading.Event(), threading.Event(), threading.Event()
        original = self.ledger.request_hold_cancel
        errors, results = [], []
        def blocked(*args):
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test gate timed out")
            return original(*args)
        def cancel():
            try:
                results.append(self.bridge.cancel("explicit cancel"))
            except BaseException as exc:
                errors.append(exc)
        def poll():
            try:
                results.append(self.bridge.cancellation_request())
                observed.set()
            except BaseException as exc:
                errors.append(exc)
        with patch.object(self.ledger, "request_hold_cancel", blocked):
            cancel_thread = threading.Thread(target=cancel)
            cancel_thread.start()
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.host.fault_event.is_set())
            self.host._fault("guard observed cancellation before disk commit")
            self.assertIsNone(self.ledger.peek_status()["fault"])
            poll_thread = threading.Thread(target=poll)
            poll_thread.start()
            self.assertFalse(observed.wait(.005))
            release.set()
            cancel_thread.join(2)
            poll_thread.join(2)
        self.assertFalse(cancel_thread.is_alive())
        self.assertFalse(poll_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(len(results), 2)
        self.assertTrue(all(result is not None for result in results))
        self.clock.now = 100.23
        request = self.bridge.cancellation_request()
        self.bridge.claim_hold(self.event, request["hold_event_id"], self.payload)

    def test_timeout_cannot_be_revived_by_late_successful_database_publication(self):
        self.source()
        self.clock.now = 100.21
        entered, release = threading.Event(), threading.Event()
        original = self.ledger.request_hold_cancel
        errors = []
        def blocked(*args):
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test gate timed out")
            return original(*args)
        def cancel():
            try:
                self.bridge.cancel("slow cancel")
            except BaseException as exc:
                errors.append(exc)
        with patch.object(self.ledger, "request_hold_cancel", blocked), patch(
                "robot_tools.host_joint.CANCEL_PUBLICATION_WAIT_S", .02):
            thread = threading.Thread(target=cancel)
            thread.start()
            self.assertTrue(entered.wait(1))
            with self.assertRaisesRegex(HostJointBridgeError, "bounded wait"):
                self.bridge.cancellation_request()
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        with self.assertRaisesRegex(HostJointBridgeError, "bounded wait"):
            self.bridge.cancellation_request()
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_publication_wait_deadline_is_shorter_than_one_feedback_interval(self):
        self.source()
        self.clock.now = 100.21
        entered, release = threading.Event(), threading.Event()
        original = self.ledger.request_hold_cancel
        errors = []
        def blocked(*args):
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test gate timed out")
            return original(*args)
        def cancel():
            try:
                self.bridge.cancel("slow cancel")
            except BaseException as exc:
                errors.append(exc)
        with patch.object(self.ledger, "request_hold_cancel", blocked):
            thread = threading.Thread(target=cancel)
            thread.start()
            self.assertTrue(entered.wait(1))
            # Observe the requested timeout deterministically, without treating
            # test-runner scheduling latency as a real-time hardware guarantee.
            with patch.object(self.bridge._published, "wait", return_value=False) as waiter:
                with self.assertRaisesRegex(HostJointBridgeError, "bounded wait"):
                    self.bridge.cancellation_request()
                self.assertEqual(waiter.call_args.args, (CANCEL_PUBLICATION_WAIT_S,))
                self.assertGreater(CANCEL_PUBLICATION_WAIT_S, 0)
                self.assertLessEqual(CANCEL_PUBLICATION_WAIT_S, .025)
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_cancel_never_waits_on_device_or_state_lock(self):
        done, errors = threading.Event(), []
        def cancel():
            try:
                self.request()
            except BaseException as exc:
                errors.append(exc)
            finally:
                done.set()
        with self.host.state_lock, self.host.device_lock:
            thread = threading.Thread(target=cancel)
            thread.start()
            self.assertTrue(done.wait(1))
        thread.join(1)
        self.assertEqual(errors, [])

    def test_another_fault_writer_refuses_hold_without_waiting_for_it(self):
        with self.host._fault_record_lock:
            with self.assertRaisesRegex(HostJointBridgeError, "Another fault writer"):
                self.request()
        self.assertTrue(self.host.fault_event.is_set())
        with self.assertRaises(HostJointBridgeError):
            self.bridge.cancellation_request()

    def test_prior_host_fault_cannot_be_reclassified(self):
        self.source()
        self.host._fault("feedback_lost")
        first = self.ledger.status()["fault"]
        with self.assertRaisesRegex(HostJointBridgeError, "prior host fault"):
            self.request()
        self.assertEqual(self.ledger.status()["fault"], first)

    def test_external_durable_fault_refuses_typed_publication(self):
        self.source()
        first = self.ledger.fault(self.host.owner, "external_owner_fault")["fault"]
        with self.assertRaisesRegex(HostJointBridgeError, "Durable explicit cancellation failed"):
            self.request()
        self.assertEqual(self.ledger.status()["fault"], first)
        self.assertTrue(self.host.fault_event.is_set())

    def test_database_failure_latches_software_and_preserves_pending_original(self):
        with patch.object(self.ledger, "request_hold_cancel", side_effect=OSError("disk unavailable")):
            with self.assertRaisesRegex(HostJointBridgeError, "disk unavailable"):
                self.request()
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIn("disk unavailable", self.host._fault_record_error)
        self.assertEqual(self.ledger.event(self.event)["status"], "pending")
        self.assertTrue(self.ledger.status()["fault_latched"])

    def test_partial_original_cannot_reach_hold_even_if_cancel_is_typed(self):
        request = self.request()
        with self.assertRaisesRegex(HostJointBridgeError, "original action worker"):
            self.bridge.claim_hold(self.event, request["hold_event_id"], self.payload)
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_adapter_snapshot_intenum_is_frozen_as_json_without_changing_value(self):
        class Control(IntEnum):
            CAN = 1
        self.original["reference"]["arms"]["right"]["arm_status"]["ctrl_mode"] = Control.CAN
        self.assertTrue(self.source()["recorded"])

    def test_original_callback_cannot_change_host_event_or_deadline(self):
        self.original["deadline_at"] += 1
        with self.assertRaisesRegex(HostJointBridgeError, "frozen host binding"):
            self.source()
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_changed_host_scope_cannot_use_old_bridge(self):
        self.source()
        self.host.active_event_id = "different"
        with self.assertRaisesRegex(HostJointBridgeError, "scope changed"):
            self.bridge.cancellation_request()

    def test_cancellation_racing_original_completion_still_latches_further_dispatch(self):
        # PairHost may select this bridge just before the old worker finishes.
        self.host.active_event_id = "a-later-action"
        with self.assertRaisesRegex(HostJointBridgeError, "scope changed"):
            self.request()
        self.assertTrue(self.host.fault_event.is_set())
        self.assertTrue(self.ledger.status()["fault_latched"])
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_different_thread_cannot_use_original_workers_claim(self):
        self.source()
        request = self.request()
        errors = []
        def other_worker():
            try:
                self.bridge.claim_hold(self.event, request["hold_event_id"], self.payload)
            except BaseException as exc:
                errors.append(exc)
        thread = threading.Thread(target=other_worker)
        thread.start()
        thread.join(1)
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], HostJointBridgeError)
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_hold_id_is_host_generated_and_cannot_be_substituted(self):
        self.source()
        self.request()
        with self.assertRaisesRegex(HostJointBridgeError, "explicit cancellation"):
            self.bridge.claim_hold(self.event, "caller-chosen-hold", self.payload)

    def test_complete_bridge_frames_and_finish_leave_original_fault_set(self):
        hold_id, _ = self.claimed()
        first = self.ledger.status()["fault"]
        for index, frame in enumerate(self.payload["expected_frames"]):
            self.bridge.record_frame_begin(hold_id, index, frame)
            self.bridge.record_frame_return(hold_id, index, "returned")
        receipt = {"hold_event_id": hold_id, "original_event_id": self.event, "identity": copy.deepcopy(IDENTITY),
                   "frames_complete": True, "hold_observed": True, "physical_stop_verified": None,
                   "original_target_cancelled": None}
        self.bridge.finish_hold(hold_id, receipt)
        self.assertEqual(self.ledger.hold_event(hold_id)["receipt"], receipt)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.ledger.status()["fault"], first)
        self.assertIsNone(self.ledger.status()["physical_stop_verified"])

    def test_new_fault_after_typed_cancel_blocks_final_hold_check_without_database_io(self):
        hold_id, _ = self.claimed()
        self.bridge.record_frame_begin(hold_id, 0, self.payload["expected_frames"][0])
        with patch.object(self.ledger, "fault", side_effect=AssertionError("Not an IO entry")), patch.object(
                self.ledger, "status", side_effect=AssertionError("Not an IO entry")):
            self.bridge.check_active()
            self.bridge.invalidate("watchdog after explicit cancellation")
            with self.assertRaisesRegex(HostJointBridgeError, "watchdog"):
                self.bridge.check_active()
            with self.assertRaisesRegex(HostJointBridgeError, "watchdog"):
                self.bridge.cancellation_request()
        with self.assertRaises(HostJointBridgeError):
            self.bridge.record_frame_begin(hold_id, 1, self.payload["expected_frames"][1])
        # A late factual return is recordable, without renewing the failed hold.
        self.bridge.record_frame_return(hold_id, 0, "unknown", "abort raced with send")
        self.assertEqual(len(self.ledger.hold_event(hold_id)["frame_receipts"]), 1)

    def test_invalidation_before_cancel_cannot_be_reclassified_as_cancel(self):
        self.bridge.invalidate("EOF")
        with self.assertRaises(HostJointBridgeError):
            self.request()
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIsNone(self.ledger.hold_event("hold-1"))

    def test_invalidation_during_publication_cannot_be_revived_by_commit(self):
        self.source()
        self.clock.now = 100.21
        entered, release = threading.Event(), threading.Event()
        original = self.ledger.request_hold_cancel
        errors = []
        def blocked(*args):
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test gate timed out")
            return original(*args)
        def cancel():
            try:
                self.bridge.cancel("cancel first")
            except BaseException as exc:
                errors.append(exc)
        with patch.object(self.ledger, "request_hold_cancel", blocked):
            thread = threading.Thread(target=cancel)
            thread.start()
            self.assertTrue(entered.wait(1))
            self.bridge.invalidate("EOF after request")
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        with self.assertRaisesRegex(HostJointBridgeError, "EOF after request"):
            self.bridge.cancellation_request()
        self.assertTrue(self.ledger.status()["fault_latched"])
        self.assertEqual(self.ledger.status()["steps"], 1)

    def test_return_callback_can_record_fault_but_never_replay_frame(self):
        hold_id, _ = self.claimed()
        self.bridge.record_frame_begin(hold_id, 0, self.payload["expected_frames"][0])
        self.bridge.record_frame_return(hold_id, 0, "unknown", "return lost")
        with self.assertRaises(PairLedgerFault):
            self.bridge.record_frame_begin(hold_id, 1, self.payload["expected_frames"][1])
        receipt = {"hold_event_id": hold_id, "original_event_id": self.event, "identity": copy.deepcopy(IDENTITY),
                   "frames_complete": False, "hold_observed": False, "physical_stop_verified": None,
                   "original_target_cancelled": None}
        self.bridge.finish_hold(hold_id, receipt)
        self.assertEqual(self.ledger.hold_event(hold_id)["frame_receipts"][0]["outcome"], "unknown")


if __name__ == "__main__":
    unittest.main()
