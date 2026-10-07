"""Offline hierarchical contract tests: no SDK, camera, ROS or model invocation."""
import json
import unittest

from right_pick.fast_task_pipeline import (BoundedTaskPipeline, COMPOSITIONS, TASK_RECIPES,
    composition_contract, pen_phase_contract, task_contract)
from right_pick.fast_pipeline import atomic_skill
from right_pick.fast_policy import PHASES


def receipt(ledger, number, status="complete", **extra):
    current = ledger.current()
    event = dict(task=current["task"], stage=current["stage"], arm=current["arm"],
        status=status, observation_id="o-" + str(number), at=float(number * 10), model_called=True,
        prerequisites=current["needs"], evidence=current["expect"] if status == "complete" else [])
    if current["skill"] == "stable_verify":
        event["stability_samples"] = [
            dict(observation_id="stable-before-" + str(number), at=event["at"] - 2,
                 support_stable=True, gripper_clear=True),
            dict(observation_id=event["observation_id"], at=event["at"],
                 support_stable=True, gripper_clear=True)]
    event.update(extra)
    return event


class HierarchicalTaskTests(unittest.TestCase):
    def test_all_arx5_tasks_have_resolvable_bounded_compositions(self):
        self.assertEqual(len(TASK_RECIPES), 18)
        self.assertEqual(len({r.task_id for r in TASK_RECIPES}), 18)
        for recipe in TASK_RECIPES:
            contract = task_contract(recipe.task_id, mode="dual_arm" if recipe.dual_required else "single_arm")
            self.assertFalse(contract["execution_available"])
            self.assertTrue(contract["steps"])
            for step in contract["steps"]:
                composite = composition_contract(step["skill"])
                self.assertGreater(step["max_cycles"], 0)
                self.assertTrue(step["evidence"])
                for atom in composite["atomic_cycle"] + composite["guards"] + composite["primitives"]:
                    atomic_skill(atom)

    def test_pair_tasks_cannot_be_downgraded_to_observer(self):
        for task in ("cups", "pen-uncapping", "bottle-unscrewing", "bolt-screwing"):
            for mode in ("single_arm", "worker_with_observer"):
                with self.subTest(task=task, mode=mode), self.assertRaises(ValueError):
                    task_contract(task, mode=mode)
        fixed = task_contract("fixed-bottle-unscrewing", mode="worker_with_observer", worker_arm="left")
        self.assertEqual({s["arm"] for s in fixed["steps"]}, {"left"})
        self.assertEqual(fixed["observer_branch"], "observer_reposition")

    def test_preinsert_does_not_test_lift_and_variants_keep_distinct_evidence(self):
        preinsert = task_contract("charger-insert-only")
        self.assertNotIn("grip_test", [s["skill"] for s in preinsert["steps"]])
        bolt = task_contract("bolt-screwing", mode="dual_arm")
        facts = [e for s in bolt["steps"] for e in s["evidence"]]
        self.assertIn("three_actual_thread_turns", facts)
        self.assertNotIn("cap_separation_verified", facts)
        push = task_contract("blue-block-triangle-push")
        self.assertNotIn("grip_test", [s["skill"] for s in push["steps"]])
        self.assertIn("edge_push_only_no_pick_place", push["constraints"])

    def test_compact_current_stage_does_not_load_all_tasks_or_primitives(self):
        current = BoundedTaskPipeline("pen").current()
        self.assertLess(len(json.dumps(current)), 300)
        self.assertNotIn("steps", current)
        self.assertNotIn("atomic_cycle", current)
        self.assertEqual(set(pen_phase_contract("GRASP")), {"task", "skill", "phase"})
        for phase in PHASES:
            self.assertIn(pen_phase_contract(phase)["skill"], {c.name for c in COMPOSITIONS})

    def test_all_tasks_reach_only_offline_completion_with_full_host_evidence(self):
        for recipe in TASK_RECIPES:
            ledger = BoundedTaskPipeline(recipe.task_id, mode="dual_arm" if recipe.dual_required else "single_arm")
            for number in range(1, len(ledger.contract["steps"]) + 1):
                report = ledger.record_cycle(receipt(ledger, number))
            self.assertTrue(report["contract_completed"])
            self.assertFalse(report["physical_success_measurable"])
            with self.assertRaises(ValueError):
                ledger.record_cycle({})

    def test_wrong_stage_arm_missing_evidence_and_reused_frame_cannot_advance(self):
        ledger = BoundedTaskPipeline("pen")
        for key, value in (("stage", "4:grip_test"), ("arm", "left"), ("evidence", []),
                           ("at", float("nan")), ("model_called", 1)):
            bad = receipt(ledger, 1)
            bad[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                ledger.record_cycle(bad)
            self.assertEqual(ledger.index, 0)
        ledger.record_cycle(receipt(ledger, 1))
        with self.assertRaises(ValueError):
            ledger.record_cycle(receipt(ledger, 1))

    def test_repeated_observation_and_progress_are_finite(self):
        ledger = BoundedTaskPipeline("pen")
        ledger.record_cycle(receipt(ledger, 1, "unknown"))
        report = ledger.record_cycle(receipt(ledger, 2, "unknown"))
        self.assertEqual(report["termination_reason"], "no_progress_budget_exhausted")
        ledger = BoundedTaskPipeline("pen")
        ledger.record_cycle(receipt(ledger, 1, "progress"))
        report = ledger.record_cycle(receipt(ledger, 2, "progress"))
        self.assertEqual(report["termination_reason"], "no_progress_budget_exhausted")

    def test_measured_progress_requires_new_continuous_state_change(self):
        ledger = BoundedTaskPipeline("pen")
        ledger.record_cycle(receipt(ledger, 1))
        change = dict(metric="object_displacement", unit="mm", before=0, after=2,
                      observation_id="o-2")
        first = ledger.record_cycle(receipt(ledger, 2, "progress", progress_measurement=change))
        self.assertIsNone(first["termination_reason"])
        self.assertEqual(ledger.no_progress, 0)
        repeated = dict(change, observation_id="o-3")
        report = ledger.record_cycle(receipt(ledger, 3, "progress", progress_measurement=repeated))
        self.assertIsNone(report["termination_reason"])
        self.assertEqual(ledger.no_progress, 1)
        continued = dict(change, before=2, after=4, observation_id="o-4")
        report = ledger.record_cycle(receipt(ledger, 4, "progress", progress_measurement=continued))
        self.assertIsNone(report["termination_reason"])
        self.assertEqual(ledger.no_progress, 0)

    def test_operation_expands_only_active_stage(self):
        ledger = BoundedTaskPipeline("pen", mode="worker_with_observer")
        operation = ledger.current_operation()
        self.assertEqual(operation["stage"], ledger.current()["stage"])
        self.assertEqual(operation["contract"]["id"], "inspect")
        self.assertEqual(operation["contract"]["primitives"], ["observe_scene"])
        self.assertNotIn("steps", operation)

    def test_uncertain_pair_receipt_latches_instead_of_replaying(self):
        ledger = BoundedTaskPipeline("cups", mode="dual_arm")
        report = ledger.record_cycle(receipt(ledger, 1, "progress", outcome_uncertain=True))
        self.assertEqual(report["termination_reason"], "execution_failed_latched")
        with self.assertRaises(ValueError):
            ledger.record_cycle({})

    def test_model_call_and_wall_time_budgets_apply_without_new_dispatch(self):
        ledger = BoundedTaskPipeline("pen", budget={"max_model_calls": 1})
        self.assertEqual(ledger.record_cycle(receipt(ledger, 1))["termination_reason"], "model_call_budget_exhausted")
        clock = [0.0]
        ledger = BoundedTaskPipeline("pen", clock=lambda: clock[0], budget={"max_elapsed_s": 2})
        clock[0] = 2.0
        self.assertIsNone(ledger.current())
        self.assertEqual(ledger.report()["termination_reason"], "wall_time_budget_exhausted")

    def test_observer_branch_requires_hold_and_returns_to_same_worker_stage(self):
        ledger = BoundedTaskPipeline("pen", mode="worker_with_observer", worker_arm="left")
        ledger.record_cycle(receipt(ledger, 1))
        before = ledger.current()
        hold = {"arm": "left", "stationary": True, "hold_verified": True, "observation_id": "o-1", "at": 10.0}
        with self.assertRaises(ValueError):
            ledger.request_observer_view(dict(hold, hold_verified=False))
        ledger.request_observer_view(hold)
        self.assertEqual(ledger.current()["arm"], "right")
        self.assertEqual(ledger.current()["skill"], "observer_reposition")
        ledger.record_cycle(receipt(ledger, 2))
        self.assertEqual(ledger.current()["stage"], before["stage"])
        self.assertEqual(ledger.current()["arm"], "left")
        self.assertEqual(ledger.model_calls, 2)

    def test_observer_hold_cannot_relabel_an_old_scene_with_latest_time(self):
        ledger = BoundedTaskPipeline("pen", mode="worker_with_observer")
        ledger.record_cycle(receipt(ledger, 1))
        ledger.record_cycle(receipt(ledger, 2, "progress"))
        with self.assertRaises(ValueError):
            ledger.request_observer_view(dict(arm="right", stationary=True, hold_verified=True,
                observation_id="o-1", at=20.0))
        self.assertFalse(ledger.observing)

    def test_view_improvement_does_not_erase_worker_no_progress(self):
        ledger = BoundedTaskPipeline("pen", mode="worker_with_observer")
        ledger.record_cycle(receipt(ledger, 1))
        ledger.record_cycle(receipt(ledger, 2, "unknown"))
        worker_stage = ledger.current()["stage"]
        ledger.request_observer_view(dict(arm="right", stationary=True, hold_verified=True,
            observation_id="o-2", at=20.0))
        ledger.record_cycle(receipt(ledger, 3))
        self.assertEqual(ledger.current()["stage"], worker_stage)
        self.assertEqual(ledger.no_progress, 1)
        report = ledger.record_cycle(receipt(ledger, 4, "unknown"))
        self.assertEqual(report["termination_reason"], "no_progress_budget_exhausted")
        self.assertEqual(report["model_calls"], 4)

    def test_observer_failure_has_its_own_bounded_no_progress(self):
        ledger = BoundedTaskPipeline("pen", mode="worker_with_observer")
        ledger.record_cycle(receipt(ledger, 1))
        ledger.request_observer_view(dict(arm="right", stationary=True, hold_verified=True,
            observation_id="o-1", at=10.0))
        ledger.record_cycle(receipt(ledger, 2, "unknown"))
        self.assertEqual(ledger.no_progress, 0)
        report = ledger.record_cycle(receipt(ledger, 3, "unknown"))
        self.assertEqual(report["termination_reason"], "observer_no_progress_budget_exhausted")
        self.assertEqual(report["cycles"], 3)

    def test_missing_contact_prerequisite_does_not_consume_a_cycle(self):
        ledger = BoundedTaskPipeline("charger-insert-only")
        for number in range(1, 5):
            ledger.record_cycle(receipt(ledger, number))
        self.assertEqual(ledger.current()["skill"], "insert_segment")
        with self.assertRaises(ValueError):
            ledger.record_cycle(receipt(ledger, 5, prerequisites=[]))
        self.assertEqual(ledger.cycles, 4)

    def test_stability_fact_alone_cannot_advance(self):
        for change in (None, "short_gap", "same_frame", "unstable"):
            ledger = BoundedTaskPipeline("pen")
            number = 1
            while ledger.current()["skill"] != "stable_verify":
                ledger.record_cycle(receipt(ledger, number))
                number += 1
            event = receipt(ledger, number)
            if change is None:
                del event["stability_samples"]
            elif change == "short_gap":
                event["stability_samples"][0]["at"] = event["at"] - .1
            elif change == "same_frame":
                event["stability_samples"][0]["observation_id"] = event["observation_id"]
            else:
                event["stability_samples"][0]["support_stable"] = False
            with self.subTest(change=change), self.assertRaises(ValueError):
                ledger.record_cycle(event)
            self.assertEqual(ledger.current()["skill"], "stable_verify")

    def test_fault_without_usable_evidence_still_latches(self):
        ledger = BoundedTaskPipeline("pen")
        report = ledger.record_cycle(receipt(ledger, 1, "fault", evidence=None, prerequisites=None))
        self.assertEqual(report["termination_reason"], "execution_failed_latched")
        self.assertEqual(ledger.index, 0)
        self.assertEqual(ledger.cycles, 1)


if __name__ == "__main__":
    unittest.main()
