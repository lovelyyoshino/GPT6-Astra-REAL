"""Pure evidence fixtures: no device imports, I/O or physical qualification.

These host-resolved fixtures describe a hypothetical adapter contract. They do
not make that contract available on GuardedPairDevice or authenticate RGB.
"""
import copy
import hashlib
import json
import unittest
from unittest.mock import patch

from robot_tools.grasp_episode import (GraspEpisodeError, admit_scope, apply_event,
                                      expire_episode, is_resolved_release, new_episode)


HASH = "a" * 64


def episode(arm="left"):
    return new_episode(episode_id="episode-" + arm, arm=arm, run_id="run-1", owner="owner-1",
                       epoch="epoch-1", object_id="strip" if arm == "left" else "plug",
                       created_at=100., deadline_at=200.)


def measurement(state, *, now=104., trace_id="trace-probe", width=.039):
    anchor = state["original_anchor"] or {
        "joints_rad": [0.] * 6, "pose_m_rad": [.2, 0., .1, 0., 0., 0.], "width_m": .039}
    observed = {**copy.deepcopy(anchor), "width_m": width}
    return {"identity": copy.deepcopy(state["identity"]), "trace_id": trace_id, "trace_sha256": HASH,
            "probe_event_id": "probe-" + state["identity"]["arm"], "started_at": now-3., "ended_at": now,
            "sample_count": 31, "feedback_advances": 30, "health": "healthy", "mode": "stationary",
            "anchor": copy.deepcopy(anchor), "observed": observed,
            "spans": {"joint_rad": 0., "position_m": 0., "rotation_rad": 0., "jaw_m": 0.},
            "anchor_deviation": {"joint_rad": 0., "position_m": 0., "rotation_rad": 0.,
                                 "jaw_m": abs(width-anchor["width_m"])}}


def scene(state, *, at=105., frame=2):
    return {"identity": copy.deepcopy(state["identity"]), "observation_id": "scene-" + str(frame),
            "captured_at": at, "frames": [{"camera_id": name, "capture_id": "capture-" + str(frame),
            "frame_number": frame, "host_received_at": at, "artifact_sha256": HASH}
            for name in ("front", "left_hand", "right_hand")]}


def event(state, event_id, kind, evidence):
    return {"event_id": event_id, "kind": kind, "identity": copy.deepcopy(state["identity"]),
            "evidence": evidence}


def release_visual(state, rgb, *, confirmed=False):
    return {"identity": copy.deepcopy(state["identity"]), "evidence_id": "visual-" + rgb["observation_id"],
            "observation_id": rgb["observation_id"], "source": "rgb_semantic_observation",
            "producer_ref": "existing-cycle-model-report", "artifact_sha256": HASH,
            "object_relation": "object_clear_of_fingers" if confirmed else "separation_not_assessed",
            "support_relation": "independent_support_present"}


def release_confirmation_event(state, *, now=120., frame=4, event_id="release-confirmed"):
    rgb = scene(state, at=now-1., frame=frame)
    return event(state, event_id, "confirm_release", {
        "action_event_id": state["release_opening"]["action_event_id"], "scene": rgb,
        "visual": release_visual(state, rgb, confirmed=True),
        "measurement": measurement(state, now=now, trace_id="separation-" + str(frame),
                                   width=state["release_opening"]["observed_width_m"])})


def candidate_event(state):
    return event(state, "candidate-" + state["identity"]["arm"], "record_candidate", {
        "probe": {"identity": copy.deepcopy(state["identity"]), "event_id": "probe-" + state["identity"]["arm"],
                  "observation_id": "scene-before-probe", "sent_at": 100.5, "completed_at": 104.,
                  "outcome": "settled_contact_candidate", "completion": "observation_only",
                  "requested_width_m": .035, "observed_width_m": .039, "trace_sha256": HASH,
                  "conservative_closure_displacement_m": .001, "target_may_remain_active": True},
        "measurement": measurement(state)})


def candidate(arm="left"):
    state = episode(arm)
    return apply_event(state, candidate_event(state), now=104.)


def retention_event(state, *, event_id="retain-1", now=108., frame=2, expires=150., renew=False):
    rgb = scene(state, at=now-1., frame=frame)
    return event(state, event_id, "renew_static" if renew else "retain_static", {
        "scene": rgb,
        "visual": {"identity": copy.deepcopy(state["identity"]), "evidence_id": "visual-" + str(frame),
                   "observation_id": rgb["observation_id"], "source": "rgb_semantic_observation",
                   "producer_ref": "existing-cycle-model-report", "artifact_sha256": HASH,
                   "object_relation": "between_fingers", "support_relation": "original_support_present"},
        "measurement": measurement(state, now=now, trace_id="trace-" + str(frame)),
        "retention_contract": {"identity": copy.deepcopy(state["identity"]), "contract_id": "retain-contract-" + str(frame),
                               "artifact_sha256": hashlib.sha256(("contract-" + str(frame)).encode()).hexdigest(),
                               "adapter_id": "hypothetical-test-adapter",
                               "adapter_code_sha256": HASH, "controller_id": "test-controller",
                               "issued_at": now, "valid_until": expires, "mode": "existing_target_monitored",
                               "probe_event_id": state["probe_ref"]["event_id"], "requested_width_m": .035,
                               "failure_behavior": "latch_no_new_targets",
                               "source": {"event_id": "adapter-retention-" + str(frame),
                                          "artifact_sha256": hashlib.sha256(("issuance-" + str(frame)).encode()).hexdigest(),
                                          "trace_id": "trace-" + str(frame), "trace_sha256": HASH}}})


def retained(arm="left"):
    state = candidate(arm)
    return apply_event(state, retention_event(state), now=108.)


def scope_request(state, operation="stationary_monitor", *, now=108.):
    return {"identity": copy.deepcopy(state["identity"]), "operation": operation,
            "observation_id": state["scene"]["observation_id"] if state["scene"] else "no-scene",
            "measurement": measurement(state, now=now, trace_id="current-health")}


class GraspEpisodeTests(unittest.TestCase):
    def test_delayed_candidate_record_preserves_history_but_cannot_authorize_retention(self):
        original = episode()
        ev = candidate_event(original)
        state = apply_event(original, ev, now=110.)
        self.assertEqual(state["measurement"]["ended_at"], 104.)
        self.assertEqual(state["status"], "contact_candidate")
        self.assertFalse(admit_scope(state, scope_request(state), now=110.)["allowed"])
        retained_event = retention_event(state, now=108.)
        with self.assertRaises(GraspEpisodeError):
            apply_event(state, retained_event, now=110.)

    def setUp(self):
        self.socket_guard = patch("socket.socket", side_effect=AssertionError("No hardware or network"))
        self.socket_guard.start()
        self.addCleanup(self.socket_guard.stop)

    def assert_rejected(self, state, ev, code, now=108.):
        before = copy.deepcopy(state)
        with self.assertRaises(GraspEpisodeError) as caught:
            apply_event(state, ev, now=now)
        self.assertEqual(caught.exception.code, code)
        self.assertEqual(state, before, "Rejected events must not mutate state")

    def test_candidate_without_prior_grasp_has_no_dispatch_permission(self):
        empty = episode()
        current = apply_event(empty, candidate_event(empty), now=104.)
        self.assertEqual(empty["status"], "empty")
        self.assertEqual(current["status"], "contact_candidate")
        self.assertTrue(current["target_may_remain_active"])
        self.assertIsNone(current["retained_scope"])
        self.assertFalse(current["dispatch_authorized"])
        self.assertIsNone(current["physical_stop_verified"])
        self.assertEqual(current, json.loads(json.dumps(current)))

    def test_static_retention_keeps_machine_and_rgb_separate(self):
        initial = candidate()
        current = apply_event(initial, retention_event(initial), now=108.)
        self.assertEqual(current["status"], "retained_static")
        self.assertEqual(current["original_anchor"], initial["original_anchor"])
        self.assertEqual(current["probe_ref"], initial["probe_ref"])
        self.assertEqual(current["visual_evidence"]["source"], "rgb_semantic_observation")
        self.assertNotIn("visual_evidence", current["measurement"])
        self.assertTrue(current["target_may_remain_active"])
        self.assertIsNone(current["object_progress_measurement"])
        self.assertFalse(current["retained_scope"]["loaded"])

    def test_missing_real_retention_contract_returns_specific_gap(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["retention_contract"] = None
        with self.assertRaises(GraspEpisodeError) as caught:
            apply_event(state, ev, now=108.)
        self.assertEqual(caught.exception.missing, ("adapter_existing_target_retention_contract",))

    def test_capability_boolean_cannot_replace_retention_contract(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["retention_contract"] = {"retained_target_stationary": True}
        self.assert_rejected(state, ev, "invalid_schema")

    def test_no_caller_grasp_or_support_claim_is_accepted(self):
        state = candidate()
        for key in ("grasp_verified", "contact_support_verified", "physical_stop_verified"):
            ev = retention_event(state)
            ev["evidence"]["visual"][key] = False
            self.assert_rejected(state, ev, "caller_assertion_forbidden")

    def test_event_and_nested_evidence_require_current_owner_epoch_object_arm(self):
        state = candidate()
        for name in ("owner", "epoch", "object_id", "run_id", "episode_id", "arm"):
            ev = retention_event(state)
            ev["identity"][name] = "right" if name == "arm" else "old-" + name
            self.assert_rejected(state, ev, "identity_mismatch")
            for evidence_name in ("scene", "visual", "measurement", "retention_contract"):
                ev = retention_event(state)
                ev["evidence"][evidence_name]["identity"][name] = "right" if name == "arm" else "old-" + name
                self.assert_rejected(state, ev, "identity_mismatch")

    def test_old_image_before_probe_cannot_retain(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["scene"]["captured_at"] = 100.
        self.assert_rejected(state, ev, "stale_scene")

    def test_visual_report_requires_exact_scene_reference(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["visual"]["observation_id"] = "old-scene"
        self.assert_rejected(state, ev, "visual_scene_mismatch")

    def test_source_support_loss_cannot_grant_static_retention(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["visual"]["support_relation"] = "unknown"
        self.assert_rejected(state, ev, "missing_static_visual_evidence")

    def test_probe_requires_exact_trace_and_actual_closure(self):
        state = episode()
        ev = candidate_event(state)
        ev["evidence"]["probe"]["trace_sha256"] = "b" * 64
        self.assert_rejected(state, ev, "probe_trace_mismatch", now=104.)
        ev = candidate_event(state)
        ev["evidence"]["probe"]["conservative_closure_displacement_m"] = 0.
        self.assert_rejected(state, ev, "not_contact_candidate", now=104.)

    def test_candidate_cannot_skip_to_static_without_trace(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["measurement"]["feedback_advances"] = 1
        self.assert_rejected(state, ev, "incomplete_trace")

    def test_finite_numbers_exclude_bools_nan_infinity(self):
        for bad in (True, False, float("nan"), float("inf")):
            with self.assertRaises(GraspEpisodeError):
                new_episode(episode_id="e", arm="left", run_id="r", owner="o", epoch="ep",
                            object_id="item", created_at=bad, deadline_at=200.)
            state = candidate()
            ev = retention_event(state)
            ev["evidence"]["measurement"]["ended_at"] = bad
            self.assert_rejected(state, ev, "invalid_number")

    def test_duplicate_event_neither_advances_nor_renews(self):
        state = episode()
        ev = candidate_event(state)
        current = apply_event(state, ev, now=104.)
        self.assertEqual(apply_event(current, copy.deepcopy(ev), now=1000.), current)
        changed = copy.deepcopy(ev)
        changed["evidence"]["probe"]["observation_id"] = "changed"
        self.assert_rejected(current, changed, "event_payload_changed")

    def test_old_event_replay_cannot_roll_back_newer_state(self):
        initial = episode()
        ev = candidate_event(initial)
        state = apply_event(initial, ev, now=104.)
        state = apply_event(state, retention_event(state), now=108.)
        self.assertEqual(apply_event(state, ev, now=109.), state)

    def test_renewal_preserves_original_anchor_probe_and_task_deadline(self):
        state = retained()
        ev = retention_event(state, event_id="renew", now=112., frame=3, expires=250., renew=True)
        renewed = apply_event(state, ev, now=112.)
        self.assertEqual(renewed["original_anchor"], state["original_anchor"])
        self.assertEqual(renewed["probe_ref"], state["probe_ref"])
        self.assertEqual(renewed["deadline_at"], 200.)
        self.assertEqual(renewed["retention_expires_at"], 200.)
        self.assertEqual(renewed["revision"], state["revision"] + 1)

    def test_renewal_cannot_rebase_drift_or_reuse_frames(self):
        state = retained()
        ev = retention_event(state, event_id="renew-1", now=112., frame=3, renew=True)
        ev["evidence"]["measurement"]["anchor"]["joints_rad"][1] += .001
        self.assert_rejected(state, ev, "anchor_rebase_forbidden", now=112.)
        ev = retention_event(state, event_id="renew-1", now=112., frame=3, renew=True)
        ev["evidence"]["scene"]["frames"][0]["frame_number"] = 2
        self.assert_rejected(state, ev, "stale_scene", now=112.)

    def test_measured_anchor_drift_cannot_hide_inside_reported_zero(self):
        state = candidate()
        ev = retention_event(state)
        ev["evidence"]["measurement"]["observed"]["pose_m_rad"][0] += .001
        self.assert_rejected(state, ev, "inconsistent_trace")

    def test_static_scope_is_current_prerequisite_not_dispatch(self):
        state = retained()
        response = admit_scope(state, scope_request(state), now=108.)
        self.assertTrue(response["allowed"])
        self.assertFalse(response["dispatch_authorized"])
        request = scope_request(state, "peer_unloaded_preparation")
        request["moving_arm"] = "right"
        self.assertTrue(admit_scope(state, request, now=108.)["allowed"])
        request["moving_arm"] = "left"
        self.assertEqual(admit_scope(state, request, now=108.)["missing"], ["moving_arm_mismatch"])

    def test_static_scope_checks_live_feedback_and_scene(self):
        state = retained()
        request = scope_request(state)
        self.assertFalse(admit_scope(state, request, now=109.)["allowed"])
        request = scope_request(state)
        request["observation_id"] = "old-scene"
        self.assertEqual(admit_scope(state, request, now=108.)["missing"], ["scene_reference_mismatch"])

    def test_first_loaded_proof_reports_engineering_gaps_not_prior_load_success(self):
        state = retained()
        for operation in ("begin_proof", "extract_segment", "insert_segment"):
            request = scope_request(state, operation)
            result = admit_scope(state, request, now=108.)
            self.assertFalse(result["allowed"])
            self.assertIn("adapter_applicable_hold_contract", result["missing"])
            self.assertIn("adapter_bounded_contact_contract", result["missing"])
            self.assertNotIn("previous_load_success", result["missing"])
            request.update(adapter_bounded_contact_contract={"contract_id": "test-contact", "artifact_sha256": HASH},
                           adapter_applicable_hold_contract={"contract_id": "test-hold", "artifact_sha256": HASH},
                           bounded_response_measurement_contract={"contract_id": "test-measurement", "artifact_sha256": HASH})
            self.assertEqual(admit_scope(state, request, now=108.)["missing"], ["loaded_transition_not_implemented"])
        self.assert_rejected(state, event(state, "proof", "begin_proof", {}), "loaded_transition_not_implemented")

    def test_expiry_is_terminal_and_preserves_residual_target(self):
        state = retained()
        expired = expire_episode(state, now=150.)
        self.assertEqual(expired["status"], "invalid")
        self.assertEqual(expired["residual_target"], state["residual_target"])
        self.assertTrue(expired["target_may_remain_active"])
        self.assertIsNone(expired["physical_stop_verified"])
        self.assertEqual(expire_episode(expired, now=151.), expired)
        ev = retention_event(expired, event_id="renew", now=151., frame=3, renew=True)
        self.assert_rejected(expired, ev, "terminal_episode", now=151.)

    def test_expired_lease_cannot_be_renewed_before_expire_is_persisted(self):
        state = retained()
        ev = retention_event(state, event_id="renew", now=151., frame=3, renew=True)
        self.assert_rejected(state, ev, "episode_expired", now=151.)

    def test_partial_tx_and_fault_are_irreversible(self):
        for send_status in ("partial", "unknown", "fault"):
            state = retained()
            invalid = apply_event(state, event(state, "fault", "invalidate", {
                "reason": "uncertain_send", "source_event_id": "motion-1", "send_status": send_status}), now=109.)
            self.assertEqual(invalid["status"], "invalid")
            self.assertEqual(invalid["probe_ref"], state["probe_ref"])
            self.assertFalse(admit_scope(invalid, scope_request(invalid, now=109.), now=109.)["allowed"])
            ev = candidate_event(invalid)
            ev["event_id"] = "candidate-after-fault"
            self.assert_rejected(invalid, ev, "terminal_episode", now=110.)

    def test_left_right_episodes_are_independent_and_not_interchangeable(self):
        left, right = retained("left"), candidate("right")
        before_right = copy.deepcopy(right)
        invalid_left = expire_episode(left, now=150.)
        self.assertEqual(right, before_right)
        self.assertEqual(invalid_left["status"], "invalid")
        self.assert_rejected(right, retention_event(left), "identity_mismatch")
        right = apply_event(right, retention_event(right), now=108.)
        self.assertEqual(right["status"], "retained_static")
        self.assertEqual(left["identity"]["object_id"], "strip")
        self.assertEqual(right["identity"]["object_id"], "plug")

    def release_pending(self):
        state = retained()
        rgb = scene(state, at=110., frame=3)
        ev = event(state, "release-begin", "begin_release", {
            "action_event_id": "jaw-open-1", "target_width_m": .042,
            "scene": rgb, "support_visual": release_visual(state, rgb),
            "measurement": measurement(state, now=112., trace_id="release-baseline")})
        return apply_event(state, ev, now=112.)

    def release_result(self, state):
        return event(state, "release-done", "finish_release", {
            "action_event_id": "jaw-open-1",
            "measurement": measurement(state, now=116., trace_id="release-settled", width=.042),
            "actual_opening_increase_m": .003, "arrival_confirmed": True,
            "target_calls_sent": 1, "passive_arm_commands_sent": 0})

    def test_release_needs_measured_opening_and_does_not_prove_stop(self):
        state = self.release_pending()
        released = apply_event(state, self.release_result(state), now=116.)
        self.assertEqual(released["status"], "release_opened")
        self.assertIsNone(released["pending"])
        self.assertTrue(released["target_may_remain_active"])
        self.assertIsNone(released["physical_stop_verified"])
        self.assertIsNone(released["object_progress_measurement"])
        self.assertEqual(released["original_anchor"], state["original_anchor"])
        self.assertEqual(released["residual_target"]["event_id"], "jaw-open-1")
        self.assertFalse(is_resolved_release(released))
        self.assertIsNone(released["retention_expires_at"])
        self.assertIsNone(released["release_confirmation"])
        self.assertEqual(released["retention_contract"], state["retention_contract"])
        self.assertEqual(released["release_opening"]["finished_at"], 116.)
        self.assertEqual(released["release_opening"]["observed_width_m"], .042)
        self.assertEqual(released["release_opening"]["trace_sha256"], HASH)

    def test_release_stability_without_measured_opening_fails(self):
        state = self.release_pending()
        ev = self.release_result(state)
        ev["evidence"]["actual_opening_increase_m"] = 0.
        self.assert_rejected(state, ev, "unconfirmed_release", now=116.)
        ev = self.release_result(state)
        ev["evidence"]["action_event_id"] = "wrong-event"
        self.assert_rejected(state, ev, "release_event_mismatch", now=116.)

    def test_uncertain_release_keeps_pending_and_refuses_new_candidate(self):
        state = self.release_pending()
        invalid = apply_event(state, event(state, "fault-release", "invalidate", {
            "reason": "partial_tx", "source_event_id": "jaw-open-1", "send_status": "partial"}), now=113.)
        self.assertEqual(invalid["pending"], state["pending"])
        self.assertEqual(invalid["status"], "invalid")
        self.assertTrue(invalid["target_may_remain_active"])

    def test_illegal_transitions_do_not_change_state(self):
        state = retained()
        self.assertEqual(apply_event(state, candidate_event(state), now=108.), state)
        ev = candidate_event(state)
        ev["event_id"] = "new-candidate"
        self.assert_rejected(state, ev, "illegal_transition")
        state = candidate()
        ev = retention_event(state, renew=True)
        self.assert_rejected(state, ev, "illegal_transition")

    def test_late_known_release_result_can_be_recorded_without_new_permission(self):
        state = self.release_pending()
        ev = self.release_result(state)
        ev["evidence"]["measurement"] = measurement(state, now=201., trace_id="late-result", width=.042)
        released = apply_event(state, ev, now=201.)
        self.assertEqual(released["status"], "release_opened")
        self.assertEqual(released["deadline_at"], 200.)
        self.assertFalse(released["dispatch_authorized"])
        self.assertFalse(admit_scope(released, scope_request(released, now=201.), now=201.)["allowed"])

    def test_release_request_remains_bounded_and_cannot_be_reissued(self):
        state = candidate()
        rgb = scene(state, at=105.)
        ev = event(state, "release-begin", "begin_release", {
            "action_event_id": "jaw-open-1", "target_width_m": .045,
            "scene": rgb, "support_visual": release_visual(state, rgb),
            "measurement": measurement(state, now=108., trace_id="new")})
        self.assert_rejected(state, ev, "release_out_of_bound")
        ev["evidence"]["target_width_m"] = .042
        pending = apply_event(state, ev, now=108.)
        self.assertEqual(apply_event(pending, ev, now=109.), pending)
        ev["event_id"] = "second-release"
        self.assert_rejected(pending, ev, "illegal_transition", now=109.)

    def opened(self):
        state = self.release_pending()
        return apply_event(state, self.release_result(state), now=116.)

    def next_opening_event(self, state, *, now=156., target=.045, frame=4):
        rgb = scene(state, at=now-1., frame=frame)
        return event(state, "release-begin-2", "begin_release", {
            "action_event_id": "jaw-open-2", "target_width_m": target, "scene": rgb,
            "support_visual": release_visual(state, rgb),
            "measurement": measurement(state, now=now, trace_id="next-opening-baseline", width=.042)})

    def test_confirmation_needs_separation_independent_support_and_new_trace(self):
        opened = self.opened()
        resolved = apply_event(opened, release_confirmation_event(opened), now=120.)
        self.assertEqual(resolved["status"], "released")
        self.assertTrue(is_resolved_release(resolved))
        self.assertEqual(resolved["release_opening"], opened["release_opening"])
        self.assertEqual(resolved["original_anchor"], opened["original_anchor"])
        self.assertEqual(resolved["release_confirmation"], {
            "action_event_id": "jaw-open-1", "observation_id": "scene-4", "trace_id": "separation-4",
            "trace_sha256": HASH, "confirmed_at": 120.})
        self.assertIsNone(resolved["physical_stop_verified"])
        self.assertIsNone(resolved["object_progress_measurement"])
        self.assertFalse(resolved["dispatch_authorized"])
        self.assertTrue(resolved["target_may_remain_active"])

    def test_opening_requires_current_support_without_prior_separation(self):
        state = candidate()
        rgb = scene(state, at=107.)
        ev = event(state, "begin", "begin_release", {"action_event_id": "open", "target_width_m": .042,
            "scene": rgb, "measurement": measurement(state, now=108., trace_id="baseline"),
            "support_visual": release_visual(state, rgb)})
        self.assertEqual(apply_event(state, ev, now=108.)["status"], "release_pending")
        for key, value in (("support_relation", "original_support_present"), ("support_relation", "unknown"),
                           ("object_relation", "between_fingers")):
            bad = copy.deepcopy(ev)
            bad["evidence"]["support_visual"][key] = value
            self.assert_rejected(state, bad, "missing_release_visual_evidence")
        del ev["evidence"]["support_visual"]
        self.assert_rejected(state, ev, "invalid_schema")

    def test_unknown_separation_or_original_support_does_not_finish_release(self):
        state = self.opened()
        for key, value in (("object_relation", "unknown"), ("object_relation", "between_fingers"),
                           ("support_relation", "unknown"), ("support_relation", "original_support_present")):
            ev = release_confirmation_event(state)
            ev["evidence"]["visual"][key] = value
            self.assert_rejected(state, ev, "missing_release_visual_evidence", now=120.)

    def test_confirmation_rejects_scene_or_any_frame_not_after_last_opening(self):
        state = self.opened()
        for at in (115.9, 116.):
            ev = release_confirmation_event(state)
            ev["evidence"]["scene"]["captured_at"] = at
            self.assert_rejected(state, ev, "stale_scene", now=120.)
        ev = release_confirmation_event(state)
        ev["evidence"]["scene"]["frames"][1]["host_received_at"] = 116.
        self.assert_rejected(state, ev, "stale_scene", now=120.)

    def test_confirmation_rejects_old_or_reused_stability_trace(self):
        state = self.opened()
        ev = release_confirmation_event(state, now=119.)  # Window begins exactly at opening completion.
        self.assert_rejected(state, ev, "stale_trace", now=119.)
        ev = release_confirmation_event(state)
        ev["evidence"]["measurement"]["trace_id"] = state["release_opening"]["trace_id"]
        self.assert_rejected(state, ev, "stale_trace", now=120.)
        ev = release_confirmation_event(state)
        self.assert_rejected(state, ev, "incomplete_or_stale_trace", now=120.11)

    def test_release_trace_uses_latest_jaw_but_original_body_anchor(self):
        state = self.opened()
        ev = release_confirmation_event(state)
        ev["evidence"]["measurement"] = measurement(state, now=120., trace_id="drift", width=.04251)
        self.assert_rejected(state, ev, "release_jaw_drift", now=120.)
        for key in ("joint_rad", "position_m", "rotation_rad", "jaw_m"):
            ev = release_confirmation_event(state)
            ev["evidence"]["measurement"]["spans"][key] = .004
            self.assert_rejected(state, ev, "measured_drift", now=120.)
        ev = release_confirmation_event(state)
        ev["evidence"]["measurement"]["anchor"]["width_m"] = .042
        self.assert_rejected(state, ev, "anchor_rebase_forbidden", now=120.)

    def test_continue_opening_after_retention_ttl_preserves_episode_budget(self):
        state = self.opened()
        pending = apply_event(state, self.next_opening_event(state), now=156.)
        self.assertEqual(pending["status"], "release_pending")
        self.assertEqual(pending["deadline_at"], 200.)
        self.assertEqual(pending["original_anchor"], state["original_anchor"])
        self.assertEqual(pending["release_opening"], state["release_opening"])
        self.assertIsNone(pending["retention_expires_at"])
        self.assertFalse(admit_scope(pending, scope_request(pending, now=156.), now=156.)["allowed"])
        self.assert_rejected(state, self.next_opening_event(state, now=201.), "episode_expired", now=201.)

    def test_continue_opening_keeps_bounds_support_and_latest_width_anchor(self):
        state = self.opened()
        self.assert_rejected(state, self.next_opening_event(state, target=.04701), "release_out_of_bound", now=156.)
        ev = self.next_opening_event(state)
        ev["evidence"]["support_visual"]["support_relation"] = "unknown"
        self.assert_rejected(state, ev, "missing_release_visual_evidence", now=156.)
        ev = self.next_opening_event(state)
        ev["evidence"]["measurement"] = measurement(state, now=156., trace_id="new", width=.04251)
        self.assert_rejected(state, ev, "release_jaw_drift", now=156.)
        ev = self.next_opening_event(state)
        ev["evidence"]["action_event_id"] = "jaw-open-1"
        self.assert_rejected(state, ev, "release_event_reused", now=156.)

    def test_second_opening_requires_confirmation_of_latest_physical_event(self):
        initial = self.opened()
        pending = apply_event(initial, self.next_opening_event(initial), now=156.)
        ev = event(pending, "release-done-2", "finish_release", {
            "action_event_id": "jaw-open-2", "measurement": measurement(pending, now=160., trace_id="open-two", width=.045),
            "actual_opening_increase_m": .003, "arrival_confirmed": True,
            "target_calls_sent": 1, "passive_arm_commands_sent": 0})
        second = apply_event(pending, ev, now=160.)
        self.assertEqual(second["release_opening"]["action_event_id"], "jaw-open-2")
        self.assertEqual(second["release_opening"]["before_width_m"], .042)
        self.assertIn("release-done", second["events"])
        self.assertIn("release-done-2", second["events"])
        confirm = release_confirmation_event(second, now=164., frame=5)
        confirm["evidence"]["action_event_id"] = "jaw-open-1"
        self.assert_rejected(second, confirm, "release_event_mismatch", now=164.)
        good = release_confirmation_event(second, now=164., frame=5)
        self.assertTrue(is_resolved_release(apply_event(second, good, now=164.)))

    def test_replay_cannot_replace_opening_evidence_or_reconfirm_with_changed_payload(self):
        state = self.opened()
        original = self.release_result(self.release_pending())
        self.assertEqual(apply_event(state, original, now=120.), state)
        confirm = release_confirmation_event(state)
        resolved = apply_event(state, confirm, now=120.)
        self.assertEqual(apply_event(resolved, confirm, now=201.), resolved)
        confirm["evidence"]["visual"]["artifact_sha256"] = "b" * 64
        self.assert_rejected(resolved, confirm, "event_payload_changed", now=201.)

    def test_delayed_opening_records_original_trace_without_renewal(self):
        pending = self.release_pending()
        ev = self.release_result(pending)
        state = apply_event(pending, ev, now=201.)
        self.assertEqual(state["release_opening"]["finished_at"], 116.)
        self.assertEqual(state["release_opening"]["recorded_at"], 201.)
        self.assertEqual(state["deadline_at"], 200.)
        self.assertEqual(state["status"], "release_opened")
        self.assert_rejected(state, release_confirmation_event(state, now=205.), "episode_expired", now=205.)
        invalid = expire_episode(state, now=202.)
        self.assertEqual(invalid["release_opening"], state["release_opening"])

    def test_old_mechanical_release_or_invented_v2_status_is_not_resolved(self):
        opened = self.opened()
        legacy = copy.deepcopy(opened)
        legacy["schema_version"], legacy["status"] = 1, "released"
        del legacy["release_opening"], legacy["release_confirmation"]
        self.assertFalse(is_resolved_release(legacy))
        invented = copy.deepcopy(opened)
        invented["status"] = "released"
        self.assertFalse(is_resolved_release(invented))
        self.assertFalse(is_resolved_release(None))
        resolved = apply_event(opened, release_confirmation_event(opened), now=120.)
        for field in ("action_event_id", "observation_id", "trace_id", "trace_sha256"):
            corrupt = copy.deepcopy(resolved)
            corrupt["release_confirmation"][field] = "b" * 64
            self.assertFalse(is_resolved_release(corrupt))

    def test_fault_after_opening_preserves_evidence_without_resurrection(self):
        state = self.opened()
        invalid = apply_event(state, event(state, "fault-after-open", "invalidate", {
            "reason": "feedback lost", "source_event_id": "jaw-open-1", "send_status": "unknown"}), now=117.)
        self.assertEqual(invalid["release_opening"], state["release_opening"])
        self.assertEqual(invalid["residual_target"], state["residual_target"])
        self.assertFalse(is_resolved_release(invalid))
        self.assert_rejected(invalid, release_confirmation_event(invalid), "terminal_episode", now=120.)

    def test_replay_still_rejects_invalid_clock(self):
        state = episode()
        ev = candidate_event(state)
        current = apply_event(state, ev, now=104.)
        for now in (False, float("nan"), float("inf")):
            self.assert_rejected(current, ev, "invalid_number", now=now)

    def test_restored_state_cannot_invent_loaded_scope_or_stop(self):
        state = retained()
        for key, value in (("physical_stop_verified", True), ("dispatch_authorized", True),
                           ("object_progress_measurement", {"delta": .001}), ("schema_version", True)):
            corrupt = copy.deepcopy(state)
            corrupt[key] = value
            self.assertFalse(admit_scope(corrupt, scope_request(state), now=108.)["allowed"])
        corrupt = copy.deepcopy(state)
        corrupt["retained_scope"]["loaded"] = True
        self.assertEqual(admit_scope(corrupt, scope_request(state), now=108.)["missing"], ["invalid_state"])
        corrupt = copy.deepcopy(state)
        corrupt["status"] = "retained_local"
        self.assertEqual(admit_scope(corrupt, scope_request(state), now=108.)["missing"], ["loaded_state_without_history"])

    def test_scope_rejects_unknown_payload_that_could_hide_contact_action(self):
        state = retained()
        request = scope_request(state, "peer_unloaded_preparation")
        request.update(moving_arm="right", kind="extract_segment")
        self.assertEqual(admit_scope(state, request, now=108.)["missing"], ["invalid_schema"])

    def test_boolean_cannot_satisfy_missing_loaded_contract(self):
        state = retained()
        request = scope_request(state, "begin_proof")
        request["adapter_applicable_hold_contract"] = True
        self.assertIn("adapter_applicable_hold_contract", admit_scope(state, request, now=108.)["missing"])

    def test_model_or_controller_binding_change_requires_new_contract(self):
        state = retained()
        for key in ("adapter_id", "controller_id", "adapter_code_sha256"):
            ev = retention_event(state, event_id="renew", now=112., frame=3, renew=True)
            ev["evidence"]["retention_contract"][key] = "b" * 64 if key.endswith("sha256") else "changed"
            self.assert_rejected(state, ev, "retention_binding_changed", now=112.)

    def test_returned_data_does_not_alias_input_evidence(self):
        state = candidate()
        ev = retention_event(state)
        updated = apply_event(state, ev, now=108.)
        ev["evidence"]["measurement"]["observed"]["joints_rad"][0] = 99.
        self.assertEqual(updated["measurement"]["observed"]["joints_rad"][0], 0.)
        updated["original_anchor"]["joints_rad"][0] = 1.
        self.assertEqual(state["original_anchor"]["joints_rad"][0], 0.)

    def test_candidate_window_must_be_entirely_after_send(self):
        state = episode()
        for sent_at in (101., 103.5):
            ev = candidate_event(state)
            ev["evidence"]["probe"]["sent_at"] = sent_at
            self.assert_rejected(state, ev, "post_send_window_required", now=104.)

    def test_old_contract_cannot_extend_expiry_in_place(self):
        state = retained()
        ev = retention_event(state, event_id="renew-inplace-contract", now=112., frame=3, renew=True)
        ev["evidence"]["retention_contract"] = copy.deepcopy(state["retention_contract"])
        ev["evidence"]["retention_contract"]["valid_until"] = 199.
        self.assert_rejected(state, ev, "retention_contract_mutated", now=112.)

    def test_unchanged_contract_can_refresh_observation_without_extending_expiry(self):
        state = retained()
        ev = retention_event(state, event_id="refresh-old-contract", now=112., frame=3, renew=True)
        ev["evidence"]["retention_contract"] = copy.deepcopy(state["retention_contract"])
        updated = apply_event(state, ev, now=112.)
        self.assertEqual(updated["retention_expires_at"], 150.)
        self.assertEqual(updated["retention_contract"], state["retention_contract"])
        self.assertEqual(updated["scene"]["observation_id"], "scene-3")

    def test_new_contract_needs_later_and_separate_issuance_source(self):
        state = retained()
        for field in ("artifact_sha256", "issued_at", "source_event_id", "source_artifact_sha256"):
            ev = retention_event(state, event_id="renew-contract", now=112., frame=3, expires=199., renew=True)
            contract = ev["evidence"]["retention_contract"]
            if field.startswith("source_"):
                name = field.removeprefix("source_")
                contract["source"][name] = state["retention_contract"]["source"][name]
            else:
                contract[field] = state["retention_contract"][field]
            self.assert_rejected(state, ev, "new_retention_issuance_required", now=112.)

    def test_new_hash_alone_cannot_supply_current_adapter_issuance(self):
        state = retained()
        ev = retention_event(state, event_id="renew-hash-only", now=112., frame=3, expires=199., renew=True)
        contract = ev["evidence"]["retention_contract"]
        contract["source"]["trace_id"] = state["measurement"]["trace_id"]
        self.assert_rejected(state, ev, "retention_source_mismatch", now=112.)
        ev = retention_event(state, event_id="renew-before-trace", now=112., frame=3, expires=199., renew=True)
        ev["evidence"]["retention_contract"]["issued_at"] = 111.
        self.assert_rejected(state, ev, "retention_source_mismatch", now=112.)

    def test_new_resolved_issuance_can_extend_lease_within_frozen_deadline(self):
        state = retained()
        ev = retention_event(state, event_id="renew-new-source", now=112., frame=3, expires=199., renew=True)
        updated = apply_event(state, ev, now=112.)
        self.assertEqual(updated["retention_expires_at"], 199.)
        self.assertEqual(updated["retention_contract"]["source"]["trace_id"], updated["measurement"]["trace_id"])
        self.assertEqual(updated["deadline_at"], state["deadline_at"])
        self.assertEqual(updated["original_anchor"], state["original_anchor"])

    def test_fault_cannot_be_restored_under_retained_static_status(self):
        state = retained()
        state["fault"] = {"reason": "feedback_lost", "source_event_id": "fault-1", "send_status": "fault", "at": 108.}
        self.assertEqual(admit_scope(state, scope_request(state), now=108.)["missing"], ["invalid_state"])
        ev = retention_event(state, event_id="renew", now=112., frame=3, renew=True)
        self.assert_rejected(state, ev, "invalid_state", now=112.)


if __name__ == "__main__":
    unittest.main()
