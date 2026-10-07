"""Offline frame-state tests. No actual emitter, SDK, CAN, camera or model."""
import copy
import hashlib
import json
import struct
import threading
import unittest
from unittest.mock import patch

from robot_tools.hold_transaction import (HoldTransactionError, JointHoldTransaction,
                                         RAD_PER_RAW, joint_hold_frames)
from test_execution import healthy_arm


RAW_START = [0, 20000, -20000, 0, 0, 0]
RAW_GOAL = [0, 21000, -20000, 0, 0, 0]
RAW_HOLD = [0, 20400, -20000, 0, 0, 0]
IDENTITY = {"run_id": "run-1", "owner": "owner-1", "epoch": "epoch-1", "worker_id": "worker-1",
            "arm": "right", "connection_id": "original-sdk-connection", "model": "piper_x",
            "firmware_profile": "default"}
HASH = "a" * 64


def sample(at, *, raw=None, identity=None, sample_id=None, moving=0):
    identity = copy.deepcopy(identity or IDENTITY)
    states = {side: healthy_arm(at) for side in ("left", "right")}
    for side, state in states.items():
        values = raw if side == identity["arm"] and raw is not None else RAW_START
        state["joints_rad"] = [value * RAD_PER_RAW for value in values]
        state["arm_status"].update(mode_feedback=1, teach_status=0,
                                  motion_status=moving if side == identity["arm"] else 0)
    return {"sample_id": sample_id or "sample-" + str(at), "identity": identity,
            "captured_at": at, "arms": states}


def event_and_claim(identity=None):
    identity = copy.deepcopy(identity or IDENTITY)
    original = {"event_id": "original-move-1", "identity": identity, "worker_thread_id": threading.get_ident(),
                "send_state": "all_frames_returned", "target_raw": list(RAW_GOAL),
                "reference": sample(100., raw=RAW_START, identity=identity),
                "frame_receipts": [{"frame": frame, "outcome": "returned", "returned_at": 100.1 + index*.001}
                                   for index, frame in enumerate(joint_hold_frames(RAW_GOAL))],
                "limits": {"joint_limits_raw": {side: [[-150000, 150000], [0, 180000], [-170000, 0],
                             [-89000, 89000], [-70000, 70000], [-120000, 120000]] for side in ("left", "right")},
                           "workspace_min_m": [-.6, -.6, .05], "workspace_max_m": [.6, .6, .65],
                           "max_translation_m": .03, "max_rotation_rad": .05},
                "deadline_at": 150., "fault": {"reason": "user_cancel", "event_id": "cancel-1", "at": 100.19}}
    claim = {"hold_event_id": "hold-1", "original_event_id": original["event_id"], "identity": copy.deepcopy(identity),
             "claimed_at": 100.2, "original_event_sha256": digest(original)}
    return original, claim


def digest(original):
    return hashlib.sha256(json.dumps(original, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def envelope_for(state):
    return {"sample_id": state["sample_id"], "model": state["identity"]["model"],
            "source_ref": "test-only-geometry-evaluation", "source_sha256": HASH,
            "fk_pose_m_rad": list(state["arms"][state["identity"]["arm"]]["pose_m_rad"]),
            "joint_radius_bounds_m": [.9, .8, .6, .4, .3, .2]}


def prepared(identity=None):
    original, claim = event_and_claim(identity)
    transaction = JointHoldTransaction(original, claim)
    state = sample(100.21, raw=RAW_HOLD, identity=identity)
    transaction.prepare(state, envelope_for(state), current_identity=state["identity"], now=100.21)
    return transaction


def complete_frames(transaction, identity=None):
    identity = identity or IDENTITY
    for index, frame in enumerate(transaction.report()["expected_frames"]):
        now = 100.22 + .002 * index
        transaction.before_frame(frame, sample(now, raw=RAW_HOLD, identity=identity), current_identity=identity, now=now)
        transaction.record_frame_return(outcome="returned", current_identity=identity, now=now+.001)


class JointHoldTransactionTests(unittest.TestCase):
    def setUp(self):
        self.no_socket = patch("socket.socket", side_effect=AssertionError("Real network/CAN forbidden"))
        self.no_socket.start()
        self.addCleanup(self.no_socket.stop)

    def assert_code(self, code, function, *args, **kwargs):
        with self.assertRaises(HoldTransactionError) as caught:
            function(*args, **kwargs)
        self.assertEqual(caught.exception.code, code)

    def test_known_exact_j_protocol_bytes_and_all_six_current_joints(self):
        frames = joint_hold_frames(RAW_HOLD)
        self.assertEqual([f["arbitration_id"] for f in frames], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(frames[0]["data_hex"], "0101010000000000")
        decoded = [value for frame in frames[1:] for value in struct.unpack(">ii", bytes.fromhex(frame["data_hex"]))]
        self.assertEqual(decoded, RAW_HOLD)
        self.assertNotEqual(decoded, RAW_GOAL)
        self.assertEqual(prepared().report()["expected_frames"], frames)

    def test_prepare_never_emits_or_claims_physical_stop_or_adapter_integration(self):
        report = prepared().report()
        self.assertEqual(report["frame_attempts"], [])
        self.assertFalse(report["dispatch_authorized"])
        self.assertIsNone(report["physical_stop_verified"])
        self.assertEqual(report["adapter_integration"], "not_integrated")
        self.assertIn("MOVE_L", report["scope"])

    def test_completed_transaction_observes_hold_without_clearing_original_fault(self):
        tx = prepared()
        fault = tx.report()["original_fault"]
        complete_frames(tx)
        self.assertEqual(tx.report()["status"], "observing")
        self.assertTrue(tx.report()["frames_complete"])
        self.assertFalse(tx.report()["hold_observed"])
        for index in range(63):
            now = 100.25 + index*.05
            report = tx.observe(sample(now, raw=RAW_HOLD), now=now)
        self.assertEqual(report["status"], "hold_observed")
        self.assertGreaterEqual(report["hold_evidence"]["feedback_advances"], 20)
        self.assertGreaterEqual(report["hold_evidence"]["ended_at"]-report["hold_evidence"]["began_at"], 3.)
        self.assertEqual(report["original_fault"], fault)
        self.assertIsNone(report["accepted"])
        self.assertIsNone(report["original_target_cancelled"])
        self.assertIsNone(report["physical_stop_verified"])

    def test_original_partial_pending_unknown_or_wrong_target_refuses(self):
        for change in ("partial", "pending", "unknown", "short_receipts", "wrong_target"):
            original, claim = event_and_claim()
            if change == "short_receipts":
                original["frame_receipts"].pop()
            elif change == "wrong_target":
                original["frame_receipts"][2]["frame"]["data_hex"] = "00" * 8
            else:
                original["send_state"] = change
            claim["original_event_sha256"] = digest(original)
            self.assert_code("original_send_incomplete", JointHoldTransaction, original, claim)

    def test_original_receipt_digest_binds_the_entire_event(self):
        original, claim = event_and_claim()
        original["limits"]["max_translation_m"] = .02
        self.assert_code("original_event_digest_mismatch", JointHoldTransaction, original, claim)

    def test_original_goal_must_remain_within_frozen_limits(self):
        original, claim = event_and_claim()
        original["target_raw"][1] = 181000
        for receipt, frame in zip(original["frame_receipts"], joint_hold_frames(original["target_raw"])):
            receipt["frame"] = frame
        claim["original_event_sha256"] = digest(original)
        self.assert_code("original_target_limit", JointHoldTransaction, original, claim)

    def test_cartesian_mit_or_mode_switch_frames_cannot_enter_j_helper(self):
        for mode in (0, 2, 4):
            original, claim = event_and_claim()
            original["reference"]["arms"]["right"]["arm_status"]["mode_feedback"] = mode
            claim["original_event_sha256"] = digest(original)
            self.assert_code("requires_existing_move_j", JointHoldTransaction, original, claim)
        tx = prepared()
        bad = {"arbitration_id": 0x151, "data_hex": "010101ad00000000"}
        self.assert_code("unexpected_frame", tx.before_frame, bad, sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)

    def test_request_requires_same_owner_epoch_connection_arm_and_event(self):
        for field in ("owner", "epoch", "connection_id", "worker_id", "run_id", "arm", "model"):
            tx = prepared()
            identity = copy.deepcopy(IDENTITY)
            identity[field] = "left" if field == "arm" else "piper" if field == "model" else "changed"
            self.assert_code("identity_mismatch", tx.before_frame, tx.report()["expected_frames"][0],
                             sample(100.22, raw=RAW_HOLD), current_identity=identity, now=100.22)
        original, claim = event_and_claim()
        claim["original_event_id"] = "different-event"
        self.assert_code("event_binding_mismatch", JointHoldTransaction, original, claim)

    def test_second_worker_cannot_issue_hold_even_with_same_identity(self):
        tx = prepared()
        errors = []
        def attempt():
            try:
                tx.before_frame(tx.report()["expected_frames"][0], sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)
            except HoldTransactionError as error:
                errors.append(error.code)
        worker = threading.Thread(target=attempt)
        worker.start()
        worker.join()
        self.assertEqual(errors, ["worker_mismatch"])
        self.assertEqual(tx.report()["status"], "fault")
        self.assertEqual(tx.report()["frame_attempts"], [])

    def test_dispatch_needs_independently_advanced_fresh_witness(self):
        tx = prepared()
        self.assert_code("fresh_dispatch_witness_required", tx.before_frame, tx.report()["expected_frames"][0],
                         sample(100.21, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)
        tx = prepared()
        self.assert_code("stale_sample", tx.before_frame, tx.report()["expected_frames"][0],
                         sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.5)

    def test_post_io_freshness_is_checked_at_each_actual_frame(self):
        tx = prepared()
        first = tx.report()["expected_frames"][0]
        tx.before_frame(first, sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)
        tx.record_frame_return(outcome="returned", current_identity=IDENTITY, now=100.221)
        self.assert_code("hold_transaction_timeout", tx.before_frame, tx.report()["expected_frames"][1],
                         sample(100.221, raw=RAW_HOLD), current_identity=IDENTITY, now=100.5)
        self.assertEqual(len(tx.report()["frame_attempts"]), 1)

    def test_pending_frame_is_not_retried_or_followed_by_remaining_frames(self):
        tx = prepared()
        frame = tx.report()["expected_frames"][0]
        self.assertIsNone(tx.before_frame(frame, sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22))
        self.assert_code("hold_attempt_not_sendable", tx.before_frame, frame, sample(100.23, raw=RAW_HOLD),
                         current_identity=IDENTITY, now=100.23)
        self.assertEqual(len(tx.report()["frame_attempts"]), 1)
        self.assertEqual(tx.report()["frame_attempts"][0]["outcome"], "pending")

    def test_partial_exception_or_unknown_keeps_fault_and_independent_rx(self):
        for outcome in ("exception", "unknown"):
            tx = prepared()
            frame = tx.report()["expected_frames"][0]
            tx.before_frame(frame, sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)
            self.assert_code("hold_send_uncertain", tx.record_frame_return, outcome=outcome, current_identity=IDENTITY, now=100.221)
            report = tx.observe(sample(100.3, raw=RAW_HOLD), now=100.3)
            self.assertEqual(report["status"], "fault")
            self.assertEqual(report["last_feedback"]["status"], "observed")
            self.assertFalse(report["last_feedback"]["motion_permitted"])
            self.assertEqual(report["original_fault"]["reason"], "user_cancel")
            self.assertIsNone(report["physical_stop_verified"])
            self.assert_code("hold_attempt_not_sendable", tx.before_frame, tx.report()["expected_frames"][1],
                             sample(100.31, raw=RAW_HOLD), current_identity=IDENTITY, now=100.31)

    def test_wrong_frame_order_other_arm_jaw_reset_and_stop_frames_are_blocked(self):
        for frame in ({"arbitration_id": 0x150, "data_hex": "0100000000000000"},
                      {"arbitration_id": 0x159, "data_hex": "00"*8},
                      {"arbitration_id": 0x471, "data_hex": "00"*8},
                      joint_hold_frames(RAW_HOLD)[1]):
            tx = prepared()
            self.assert_code("unexpected_frame", tx.before_frame, frame, sample(100.22, raw=RAW_HOLD),
                             current_identity=IDENTITY, now=100.22)
            self.assertEqual(tx.report()["frame_attempts"], [])

    def test_extra_frame_after_four_local_returns_is_forbidden(self):
        tx = prepared()
        complete_frames(tx)
        self.assert_code("hold_attempt_not_sendable", tx.before_frame, tx.report()["expected_frames"][0],
                         sample(100.24, raw=RAW_HOLD), current_identity=IDENTITY, now=100.24)
        self.assertEqual(len(tx.report()["frame_attempts"]), 4)

    def test_each_frame_checks_current_health_mode_and_passive_arm(self):
        for change, code in (("driver_fault", "unhealthy_feedback"), ("mode", "requires_existing_move_j"),
                             ("peer_move", "passive_arm_moving"), ("peer_drift", "passive_arm_drift"),
                             ("jaw", "jaw_drift"), ("plan_slip", "hold_plan_slip")):
            tx = prepared()
            state = sample(100.22, raw=RAW_HOLD)
            if change == "driver_fault":
                state["arms"]["right"]["drivers"]["2"]["foc_status"]["driver_error_status"] = True
            elif change == "mode":
                state["arms"]["right"]["arm_status"]["mode_feedback"] = 2
            elif change == "peer_move":
                state["arms"]["left"]["arm_status"]["motion_status"] = 1
            elif change == "peer_drift":
                state["arms"]["left"]["joints_rad"][0] += .004
            elif change == "jaw":
                state["arms"]["right"]["gripper"]["width_m"] -= .001
            else:
                state["arms"]["right"]["joints_rad"][1] += .004
            self.assert_code(code, tx.before_frame, tx.report()["expected_frames"][0], state, current_identity=IDENTITY, now=100.22)

    def test_current_joint_bound_and_original_envelope_are_not_relaxed(self):
        for raw, code in (([0, -1, -20000, 0, 0, 0], "nominal_joint_limit"),
                          ([0, 23000, -20000, 0, 0, 0], "original_joint_envelope")):
            original, claim = event_and_claim()
            tx = JointHoldTransaction(original, claim)
            state = sample(100.21, raw=raw)
            self.assert_code(code, tx.prepare, state, envelope_for(state), current_identity=IDENTITY, now=100.21)

    def test_geometry_model_and_controller_pose_disagreement_still_blocks(self):
        for change, code in (("model", "envelope_binding_mismatch"), ("pose", "model_controller_pose_mismatch"),
                             ("radii", "hold_tracking_box_exceeds_original_bounds")):
            original, claim = event_and_claim()
            tx = JointHoldTransaction(original, claim)
            state = sample(100.21, raw=RAW_HOLD)
            envelope = envelope_for(state)
            if change == "model":
                envelope["model"] = "piper"
            elif change == "pose":
                envelope["fk_pose_m_rad"][0] += .004
            else:
                envelope["joint_radius_bounds_m"] = [10.] * 6
            self.assert_code(code, tx.prepare, state, envelope, current_identity=IDENTITY, now=100.21)

    def test_stability_without_controller_arrival_does_not_claim_hold(self):
        tx = prepared()
        complete_frames(tx)
        for index in range(63):
            now = 100.25 + index*.05
            report = tx.observe(sample(now, raw=RAW_HOLD, moving=1), now=now)
        self.assertFalse(report["hold_observed"])
        self.assertEqual(report["status"], "observing")
        self.assertIsNone(report["physical_stop_verified"])

    def test_feedback_gap_or_presend_feedback_cannot_form_stability_window(self):
        for now, stamp, code in ((100.25, 100.226, "post_hold_send_feedback_required"),
                                 (100.5, 100.5, "post_send_trace_gap")):
            tx = prepared()
            complete_frames(tx)
            report = tx.observe(sample(stamp, raw=RAW_HOLD), now=now)
            self.assertEqual(report["status"], "fault")
            self.assertEqual(report["hold_fault"]["code"], code)

    def test_clock_bool_nan_or_regression_faults_without_a_send(self):
        for now, code in ((False, "invalid_number"), (float("nan"), "invalid_number"), (100.1, "clock_regressed")):
            tx = prepared()
            self.assert_code(code, tx.before_frame, tx.report()["expected_frames"][0], sample(100.22, raw=RAW_HOLD),
                             current_identity=IDENTITY, now=now)
            self.assertEqual(tx.report()["frame_attempts"], [])

    def test_expired_original_budget_is_not_renewed_by_hold(self):
        tx = prepared()
        self.assert_code("original_budget_expired", tx.before_frame, tx.report()["expected_frames"][0],
                         sample(150., raw=RAW_HOLD), current_identity=IDENTITY, now=150.)
        self.assertEqual(tx.report()["original_deadline_at"], 150.)

    def test_eof_invalidates_without_stop_or_disconnect_claim(self):
        tx = prepared()
        report = tx.invalidate("host_eof", now=100.22)
        self.assertEqual(report["status"], "fault")
        self.assertEqual(report["frame_attempts"], [])
        self.assertIsNone(report["physical_stop_verified"])
        self.assertEqual(tx.observe(sample(100.3, raw=RAW_HOLD), now=100.3)["last_feedback"]["status"], "observed")

    def test_left_and_right_have_same_pure_protocol_without_qualification_transfer(self):
        identity = {**IDENTITY, "arm": "left"}
        tx = prepared(identity)
        complete_frames(tx, identity)
        self.assertTrue(tx.report()["frames_complete"])
        self.assertEqual(tx.report()["physical_validation"], "not_performed")
        self.assertIsNone(tx.report()["physical_stop_verified"])

    def test_frame_identity_types_and_all_standard_can_flags_are_enforced(self):
        for key, value in (("arbitration_id", float(0x151)), ("is_extended_id", True),
                           ("is_remote_frame", True), ("is_error_frame", True), ("is_fd", True),
                           ("is_extended_id", 0)):
            tx = prepared()
            frame = tx.report()["expected_frames"][0]
            frame[key] = value
            self.assert_code("unexpected_frame", tx.before_frame, frame, sample(100.22, raw=RAW_HOLD),
                             current_identity=IDENTITY, now=100.22)
            self.assertEqual(tx.report()["frame_attempts"], [])

    def test_feedback_failure_after_hold_preserves_historical_evidence_but_revokes_current_observation(self):
        tx = prepared()
        complete_frames(tx)
        for index in range(63):
            now = 100.25 + index*.05
            tx.observe(sample(now, raw=RAW_HOLD), now=now)
        self.assertTrue(tx.report()["hold_observed"])
        failed = sample(103.4, raw=RAW_HOLD)
        failed["arms"]["right"]["drivers"]["1"]["foc_status"]["driver_error_status"] = True
        report = tx.observe(failed, now=103.4)
        self.assertFalse(report["hold_observed"])
        self.assertEqual(report["status"], "fault")
        self.assertIsNotNone(report["hold_evidence"])
        self.assertFalse(report["last_feedback"]["diagnostics"]["right"]["health"]["healthy"])
        self.assertIsNone(report["physical_stop_verified"])

    def test_independently_arriving_fragments_accumulate_complete_advances(self):
        tx = prepared()
        complete_frames(tx)
        for index in range(313):
            now = 100.24 + index*.01
            state = sample(now, raw=RAW_HOLD)
            # Two asynchronous groups alternate; no single adjacent read advances
            # every fragment, but a complete pair advances every two reads.
            for side in ("left", "right"):
                for position, name in enumerate(state["arms"][side]["fragment_timestamps_s"]):
                    stamp = 100.24 + (index if (position + index) % 2 else index-1)*.01
                    state["arms"][side]["fragment_timestamps_s"][name] = stamp
            report = tx.observe(state, now=now)
        self.assertEqual(report["status"], "hold_observed")
        self.assertGreaterEqual(report["hold_evidence"]["feedback_advances"], 20)

    def test_official_sdk_status_enums_are_supported_without_allowing_bools(self):
        from robot_tools import arms
        from test_backend import PROFILE
        arms._load_sdk(PROFILE["sdk_path"])  # Import definitions only; all sockets remain blocked.
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatusEnum as E
        def enums(state):
            for value in state["arms"].values():
                value["arm_status"].update(ctrl_mode=E.CtrlMode.CAN_CTRL, arm_status=E.ArmStatus.NORMAL,
                    teach_status=E.TeachingState.DISABLED, mode_feedback=E.ModeFeedback.MOVE_J,
                    motion_status=E.MotionStatus.REACH_TARGET_POS_SUCCESSFULLY)
            return state
        original, claim = event_and_claim()
        enums(original["reference"])
        claim["original_event_sha256"] = digest(original)
        tx = JointHoldTransaction(original, claim)
        state = enums(sample(100.21, raw=RAW_HOLD))
        tx.prepare(state, envelope_for(state), current_identity=IDENTITY, now=100.21)
        tx.before_frame(tx.report()["expected_frames"][0], enums(sample(100.22, raw=RAW_HOLD)),
                        current_identity=IDENTITY, now=100.22)
        self.assertEqual(len(tx.report()["frame_attempts"]), 1)
        for name, value, code in (("teach_status", False, "teach_mode"),
                                  ("motion_status", False, "unknown_motion_status"),
                                  ("mode_feedback", True, "requires_existing_move_j")):
            tx = prepared()
            state = sample(100.22, raw=RAW_HOLD)
            state["arms"]["right"]["arm_status"][name] = value
            self.assert_code(code, tx.before_frame, tx.report()["expected_frames"][0], state,
                             current_identity=IDENTITY, now=100.22)

    def test_tracking_box_includes_already_used_translation_and_rotation(self):
        for position, rotation, admitted in ((.029, 0., False), (.02039, 0., True), (.02041, 0., False),
                                              (0., .0319, True), (0., .0321, False)):
            with self.subTest(position=position, rotation=rotation):
                original, claim = event_and_claim()
                tx = JointHoldTransaction(original, claim)
                state = sample(100.21, raw=RAW_HOLD)
                state["arms"]["right"]["pose_m_rad"][0] += position
                state["arms"]["right"]["pose_m_rad"][5] += rotation
                if admitted:
                    self.assertEqual(tx.prepare(state, envelope_for(state), current_identity=IDENTITY,
                                                now=100.21)["status"], "prepared")
                else:
                    self.assert_code("hold_tracking_box_exceeds_original_bounds", tx.prepare, state,
                                     envelope_for(state), current_identity=IDENTITY, now=100.21)
                    self.assertEqual(tx.report()["frame_attempts"], [])

    def test_external_cancel_after_frame_validation_cannot_revive_fault(self):
        tx = prepared()
        validate = tx._validate_sample
        def cancel_after_validation(*args, **kwargs):
            result = validate(*args, **kwargs)
            canceller = threading.Thread(target=lambda: tx.invalidate("external_cancel", now=100.22))
            canceller.start()
            canceller.join(1)
            self.assertFalse(canceller.is_alive(), "Cancellation must not wait on feedback-validation callbacks")
            return result
        with patch.object(tx, "_validate_sample", side_effect=cancel_after_validation):
            self.assert_code("hold_fault_latched", tx.before_frame, tx.report()["expected_frames"][0],
                             sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)
        self.assertEqual(tx.report()["status"], "fault")
        self.assertEqual(tx.report()["frame_attempts"], [])
        self.assertEqual(tx.report()["hold_fault"]["code"], "external_cancel")
        self.assert_code("no_pending_hold_frame", tx.record_frame_return,
                         outcome="returned", current_identity=IDENTITY, now=100.221)
        self.assertEqual(tx.observe(sample(100.3, raw=RAW_HOLD), now=100.3)["last_feedback"]["status"], "observed")

    def test_external_cancel_after_prepare_validation_cannot_create_plan(self):
        original, claim = event_and_claim()
        tx = JointHoldTransaction(original, claim)
        validate = tx._validate_sample
        def cancel_after_validation(*args, **kwargs):
            result = validate(*args, **kwargs)
            canceller = threading.Thread(target=lambda: tx.invalidate("external_cancel", now=100.21))
            canceller.start()
            canceller.join(1)
            self.assertFalse(canceller.is_alive())
            return result
        state = sample(100.21, raw=RAW_HOLD)
        with patch.object(tx, "_validate_sample", side_effect=cancel_after_validation):
            self.assert_code("hold_fault_latched", tx.prepare, state, envelope_for(state),
                             current_identity=IDENTITY, now=100.21)
        self.assertEqual(tx.report()["status"], "fault")
        self.assertIsNone(tx.report()["expected_frames"])

    def test_whole_window_spans_reject_band_to_band_oscillation_on_both_arms(self):
        for side in ("left", "right"):
            for component in ("joint", "position", "rotation", "jaw"):
                with self.subTest(side=side, component=component):
                    original, claim = event_and_claim()
                    # A mid-range jaw permits both +/-0.4 mm deviations inside
                    # the original anchor band without hitting the width cap.
                    for arm in original["reference"]["arms"].values():
                        arm["gripper"]["width_m"] = .04
                    claim["original_event_sha256"] = digest(original)
                    tx = JointHoldTransaction(original, claim)
                    def get_sample(at):
                        state = sample(at, raw=RAW_HOLD)
                        for arm in state["arms"].values():
                            arm["gripper"]["width_m"] = .04
                        return state
                    plan = get_sample(100.21)
                    tx.prepare(plan, envelope_for(plan), current_identity=IDENTITY, now=100.21)
                    for index, frame in enumerate(tx.report()["expected_frames"]):
                        at = 100.22 + index*.002
                        tx.before_frame(frame, get_sample(at), current_identity=IDENTITY, now=at)
                        tx.record_frame_return(outcome="returned", current_identity=IDENTITY, now=at+.001)
                    for index in range(3):
                        now = 100.25 + index*.05
                        state = get_sample(now)
                        sign = 0 if index == 0 else 1 if index == 1 else -1
                        if component == "joint":
                            state["arms"][side]["joints_rad"][5] += sign*100*RAD_PER_RAW
                        elif component == "position":
                            state["arms"][side]["pose_m_rad"][0] += sign*.0004
                        elif component == "rotation":
                            state["arms"][side]["pose_m_rad"][5] += sign*100*RAD_PER_RAW
                        else:
                            state["arms"][side]["gripper"]["width_m"] += sign*.0004
                        report = tx.observe(state, now=now)
                    self.assertEqual(report["status"], "fault")
                    self.assertEqual(report["hold_fault"]["code"], "unstable_hold_window")
                    self.assertFalse(report["hold_observed"])

    def test_malformed_rx_still_returns_fault_diagnostics(self):
        for bad in (None, {}, {"arms": None}, {"arms": []}, {"arms": {"left": None}}):
            tx = prepared()
            tx.invalidate("fault", now=100.22)
            report = tx.observe(bad, now=100.23)
            self.assertEqual(report["status"], "fault")
            self.assertIn(report["last_feedback"]["status"], ("read_error", "partial"))
            self.assertEqual(report["hold_fault"]["code"], "fault")
            self.assertFalse(report["hold_observed"])
            self.assertIsNone(report["physical_stop_verified"])

    def test_cancel_with_invalid_or_queued_old_time_always_latches(self):
        for now in (100.2, float("nan"), float("inf"), False):
            tx = prepared()
            report = tx.invalidate("host_eof", now=now)
            self.assertEqual(report["status"], "fault")
            self.assertEqual(report["hold_fault"]["code"], "host_eof")
            self.assertIn("timestamp", report["hold_fault"]["detail"])
            self.assert_code("hold_attempt_not_sendable", tx.before_frame, report["expected_frames"][0],
                             sample(100.22, raw=RAW_HOLD), current_identity=IDENTITY, now=100.22)
            self.assertEqual(tx.report()["frame_attempts"], [])


if __name__ == "__main__":
    unittest.main()
