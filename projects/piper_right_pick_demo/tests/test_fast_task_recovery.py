"""Offline recovery-contract tests; evidence below is explicitly synthetic."""
import copy
import json
import unittest
from unittest.mock import patch

from right_pick.fast_pipeline import PipelineContractError
from right_pick.fast_task_recovery import build_recovery_contract


def fixture(category="no_progress", status="completed"):
    stage = dict(task="turn-fixture_v1", stage="turn-handle", skill="articulated_rotate", arm="left",
                 goal="Reach the declared visually observable handle orientation", cycles_left=3, model_calls_left=4)
    counts = (1, 1) if status == "completed" else (0, 0)
    result = dict(task=stage["task"], stage=stage["stage"], arm=stage["arm"], category=category,
                  observation_id="synthetic-new-observation", source="host_execution_report",
                  execution=dict(status=status, attempted=counts[0], sent=counts[1],
                                 receipt_ref="synthetic:host-receipt", fault_latched=False, hold_status="verified"))
    budget = dict(cycles=8, model_calls=4, recoveries=2, time_s=60)
    return stage, result, budget


class RecoveryContractTests(unittest.TestCase):
    def assert_blocked(self, plan, reason):
        self.assertEqual(plan["status"], "blocked")
        self.assertIn(reason, plan["blocking_reasons"])
        self.assertEqual(plan["candidate_operations"], [])
        self.assertTrue(all(value == 0 for value in plan["proposal_budget"].values()))
        self.assertIn("current_verified_hold_or_stop_evidence", plan["evidence_gates"])
        self.assertFalse(plan["execution_available"])
        self.assertFalse(plan["clears_failure_latch"])

    def test_visual_failures_request_new_rgb_without_automatic_motion_or_rollback(self):
        expected = {"perception_uncertain": ["inspect"], "occlusion": ["inspect"],
                    "grasp_failed": ["inspect", "align"],
                    "no_progress": ["inspect", "articulated_rotate"]}
        for category, operations in expected.items():
            with self.subTest(category=category):
                stage, result, budget = fixture(category)
                plan = build_recovery_contract(stage, result, budget)
                self.assertEqual(plan["status"], "proposed")
                self.assertEqual([p["operation"] for p in plan["candidate_operations"]], operations)
                self.assertTrue(plan["fresh_rgb_required"])
                self.assertTrue(all(p["motion_parameters"] is None for p in plan["candidate_operations"]))
                self.assertFalse(plan["phase_transition_applied"])
                self.assertFalse(plan["actual_regression_implemented"])
                self.assertFalse(plan["execution_available"])
                self.assertFalse(plan["automatic_retry"])

    def test_failed_grasp_does_not_assume_support_or_propose_automatic_open_close(self):
        plan = build_recovery_contract(*fixture("grasp_failed"))
        align = plan["candidate_operations"][1]
        self.assertIn("object_support_or_held_load_independently_reviewed", align["requires"])
        self.assertNotIn("set_gripper_once", json.dumps(plan["candidate_operations"]))
        self.assertEqual(plan["dispatched_action_count"], 0)

    def test_unknown_hold_allows_only_read_only_visual_diagnosis(self):
        for category in ("grasp_failed", "no_progress"):
            stage, result, budget = fixture(category)
            result["execution"]["hold_status"] = "unknown"
            plan = build_recovery_contract(stage, result, budget)
            self.assertEqual([c["operation"] for c in plan["candidate_operations"]], ["inspect"])
            self.assertIn("current_verified_hold_or_stop_evidence_before_motion_candidate", plan["evidence_gates"])
            self.assertFalse(plan["execution_available"])

    def test_confirmed_zero_tx_allows_only_new_evidence_and_explicit_new_proposal(self):
        plan = build_recovery_contract(*fixture("zero_tx_rejected", "confirmed_zero_tx"))
        self.assertEqual(plan["status"], "proposed")
        self.assertFalse(plan["replay_previous_action"])
        self.assertIn("new_explicit_proposal", plan["candidate_operations"][-1]["requires"])
        self.assertIn("no_target_clipping_or_force_escalation", plan["prohibitions"])
        self.assertFalse(plan["evidence_authentication_performed"])

    def test_zero_counts_or_model_label_cannot_manufacture_confirmed_zero_execution(self):
        stage, result, budget = fixture("zero_tx_rejected", "not_attempted")
        self.assert_blocked(build_recovery_contract(stage, result, budget), "zero_tx_not_confirmed")
        result["execution"].update(status="unknown")
        self.assert_blocked(build_recovery_contract(stage, result, budget), "unknown_send")
        result["execution"].update(status="confirmed_zero_tx", receipt_ref=None)
        with self.assertRaises(PipelineContractError):
            build_recovery_contract(stage, result, budget)
        result["execution"]["receipt_ref"] = "synthetic:report"
        result["source"] = "model_visual_report"
        with self.assertRaises(PipelineContractError):
            build_recovery_contract(stage, result, budget)

    def test_partial_unknown_and_latched_fault_override_perception_diagnosis(self):
        for change, blocker in (
                (dict(status="partial", attempted=4, sent=1), "partial_send"),
                (dict(status="unknown", attempted=4, sent=4), "unknown_send"),
                (dict(status="unknown", attempted=None, sent=None), "unknown_send"),
                (dict(fault_latched=True), "execution_fault"),
                (dict(hold_status="unverified"), "hold_unverified")):
            stage, result, budget = fixture("occlusion")
            result["execution"].update(change)
            with self.subTest(change=change):
                self.assert_blocked(build_recovery_contract(stage, result, budget), blocker)

    def test_timeout_and_unverified_hold_cannot_be_unlocked_by_a_verified_hold_label(self):
        for category in ("unknown_send", "partial_send", "timeout", "hold_unverified", "execution_fault"):
            with self.subTest(category=category):
                plan = build_recovery_contract(*fixture(category))
                self.assert_blocked(plan, category)
                self.assertEqual(plan["execution_evidence"]["hold_status"], "verified")
                self.assertIn("no_stop_reset_disable_command", plan["prohibitions"])

    def test_terminal_state_is_not_recovered_even_with_positive_remaining_budget(self):
        stage, result, budget = fixture()
        self.assert_blocked(build_recovery_contract(None, result, budget), "terminal_state")
        for reason in ("execution_failed_latched", "offline_contract_completed", "offline_session_ended"):
            stage["termination_reason"] = reason
            with self.subTest(reason=reason):
                self.assert_blocked(build_recovery_contract(stage, result, budget), "terminal_state")

    def test_every_exhausted_global_or_stage_budget_blocks_even_confirmed_zero_tx(self):
        for name in ("cycles", "model_calls", "recoveries", "time_s"):
            stage, result, budget = fixture("zero_tx_rejected", "confirmed_zero_tx")
            budget[name] = 0
            with self.subTest(name=name):
                plan = build_recovery_contract(stage, result, budget)
                self.assert_blocked(plan, "budget_exhausted")
                self.assertIn(name, plan["exhausted_budget_fields"])
        for name in ("cycles_left", "model_calls_left"):
            stage, result, budget = fixture()
            stage[name] = 0
            with self.subTest(name=name):
                self.assert_blocked(build_recovery_contract(stage, result, budget), "budget_exhausted")
        self.assert_blocked(build_recovery_contract(*fixture("budget_exhausted")), "budget_exhausted")

    def test_budget_exhaustion_does_not_hide_unresolved_execution(self):
        stage, result, budget = fixture("no_progress", "partial")
        result["execution"].update(attempted=4, sent=2)
        budget["time_s"] = 0
        plan = build_recovery_contract(stage, result, budget)
        self.assertEqual(plan["classification"], "partial_send")
        self.assertIn("budget_exhausted", plan["blocking_reasons"])
        self.assertIn("resolve_execution_outcome_and_fault_with_host", plan["evidence_gates"])

    def test_plan_is_one_choice_not_a_motion_sequence_and_does_not_consume_or_reset_budget(self):
        values = fixture()
        original = copy.deepcopy(values)
        with patch("builtins.open", side_effect=AssertionError("pure planner must not perform I/O")):
            first = build_recovery_contract(*values)
            second = build_recovery_contract(*values)
        self.assertEqual(first, second)
        self.assertEqual(values, original)
        self.assertEqual(first["proposal_budget"], dict(max_new_observations=1, max_model_proposals=1,
                                                       max_recovery_attempts=1))
        self.assertEqual(first["selection_policy"], "at_most_one_candidate_after_new_evidence_not_a_sequence")
        self.assertFalse(first["budget_consumed"])
        self.assertTrue(first["durable_accounting_required"])
        first["remaining_budget"]["cycles"] = 100
        self.assertEqual(values[2]["cycles"], 8)

    def test_current_goal_is_generic_and_result_must_bind_same_task_stage_arm(self):
        stage, result, budget = fixture()
        plan = build_recovery_contract(stage, result, budget)
        self.assertEqual(plan["candidate_operations"][-1]["goal"], stage["goal"])
        for key, value in (("task", "other_v1"), ("stage", "old-stage"), ("arm", "right")):
            bad = copy.deepcopy(result)
            bad[key] = value
            with self.subTest(key=key), self.assertRaises(PipelineContractError):
                build_recovery_contract(stage, bad, budget)

    def test_invalid_evidence_and_budget_types_fail_closed(self):
        changes = (
            lambda s, r, b: r["execution"].update(status="completed", attempted=4, sent=1),
            lambda s, r, b: r["execution"].update(status="confirmed_zero_tx", attempted=1, sent=0),
            lambda s, r, b: r["execution"].update(attempted=True),
            lambda s, r, b: r["execution"].update(attempted=1, sent=2),
            lambda s, r, b: r["execution"].update(fault_latched="false"),
            lambda s, r, b: r.update(category="ignore_limits"),
            lambda s, r, b: b.update(cycles=-1),
            lambda s, r, b: b.update(recoveries=True),
            lambda s, r, b: b.update(time_s=float("nan")),
            lambda s, r, b: b.update(time_s=float("inf")),
            lambda s, r, b: s.update(target_pose=[0] * 6),
            lambda s, r, b: r.update(allow_retry=True),
        )
        for index, change in enumerate(changes):
            values = fixture()
            change(*values)
            with self.subTest(case=index), self.assertRaises(PipelineContractError):
                build_recovery_contract(*values)


if __name__ == "__main__":
    unittest.main()
