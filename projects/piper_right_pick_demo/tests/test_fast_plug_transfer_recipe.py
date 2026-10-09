"""Synthetic contract checks for two-task-arm plug transfer; no device or model I/O."""
import copy
from pathlib import Path
import tempfile
import unittest

from right_pick.fast_pipeline import PipelineContractError
from right_pick.fast_task_pipeline import BoundedTaskPipeline
from right_pick.fast_task_session import TaskSessionStore
from right_pick.fast_task_spec import load_recipe, recipe_contract


RECIPE = Path(__file__).resolve().parents[3] / "tasks" / "plug_transfer_left.json"


def synthetic_receipt(report, number):
    """Invented facts exercise the ledger only; they are not physical evidence."""
    stage = report["next"]
    stamp = float(number * 10)
    result = dict(task=stage["task"], stage=stage["stage"], arm=stage["arm"],
                  observation_id="synthetic-" + str(number), at=stamp,
                  model_called=False, status="complete",
                  prerequisites=list(stage["needs"]), evidence=list(stage["expect"]))
    if "run_id" in report:
        result["run_id"] = report["run_id"]
    if stage["skill"] == "stable_verify":
        result["stability_samples"] = [
            dict(observation_id="synthetic-first-" + str(number), at=stamp - 2,
                 support_stable=True, gripper_clear=True),
            dict(observation_id=result["observation_id"], at=stamp,
                 support_stable=True, gripper_clear=True),
        ]
    return result


class PlugTransferRecipeTests(unittest.TestCase):
    def setUp(self):
        self.source = load_recipe(RECIPE, mode="dual_arm")

    def ledger(self):
        return BoundedTaskPipeline(self.source["task_id"], mode="dual_arm",
                                   task_definition=self.source, clock=lambda: 0.)

    def advance_to(self, ledger, stage_id):
        report = ledger.report()
        while report["next"]["stage"] != stage_id:
            report = ledger.record_cycle(synthetic_receipt(report, report["cycles"] + 1))
        return report

    def test_stabilizer_is_task_arm_and_other_modes_cannot_execute_its_stages(self):
        contract = recipe_contract(self.source, mode="dual_arm")
        self.assertEqual(contract["roles"]["peer_arm"], "left")
        self.assertIsNone(contract["roles"]["observer_arm"])
        self.assertEqual(contract["dispatch_policy"], "one_arm_moves_at_a_time")
        self.assertFalse(contract["execution_available"])
        self.assertEqual(contract["implementation"], "offline_contract_only")
        for mode in ("single_arm", "worker_with_observer"):
            for worker in ("left", "right"):
                with self.subTest(mode=mode, worker=worker), self.assertRaises(PipelineContractError):
                    recipe_contract(self.source, mode=mode, worker_arm=worker)

    def test_supported_grasps_preserve_source_support_until_explicit_extraction(self):
        contract = self.ledger().contract
        stages = {stage["id"]: stage for stage in contract["steps"]}
        for stage_id in ("stabilizer_grasp", "plug_grasp"):
            self.assertEqual(stages[stage_id]["skill"], "grip_supported")
            self.assertIn("support_unchanged", stages[stage_id]["evidence"])
            self.assertNotIn("support_separated", stages[stage_id]["evidence"])
        self.assertEqual(stages["stabilizer_grasp"]["arm"], "left")
        self.assertEqual(stages["plug_grasp"]["arm"], "right")
        self.assertIn("plug_still_seated_in_source", stages["plug_grasp"]["evidence"])

    def test_closure_and_relative_motion_cannot_replace_complete_visual_extraction(self):
        ledger = self.ledger()
        report = self.advance_to(ledger, "extract_plug")
        for missing in ("source_socket_empty", "plug_pins_clear_of_source", "plug_retained"):
            event = synthetic_receipt(report, report["cycles"] + 1)
            event["evidence"].remove(missing)
            with self.subTest(missing=missing), self.assertRaises(PipelineContractError):
                ledger.record_cycle(event)
            self.assertEqual(ledger.current()["stage"], "extract_plug")
            self.assertEqual(ledger.report()["cycles"], report["cycles"])
        next_report = ledger.record_cycle(synthetic_receipt(report, report["cycles"] + 1))
        self.assertEqual(next_report["next"]["stage"], "transfer_left")

    def test_generic_insertion_progress_cannot_replace_correct_target_seating(self):
        ledger = self.ledger()
        report = self.advance_to(ledger, "insert_left_socket")
        event = synthetic_receipt(report, report["cycles"] + 1)
        event["evidence"].remove("plug_seated_in_left_socket")
        with self.assertRaises(PipelineContractError):
            ledger.record_cycle(event)
        self.assertEqual(ledger.current()["stage"], "insert_left_socket")

    def test_stability_is_after_both_releases_and_needs_two_fresh_samples(self):
        ledger = self.ledger()
        report = self.advance_to(ledger, "release_strip")
        self.assertEqual(ledger.evidence[-1]["stage"], "release_plug")
        report = ledger.record_cycle(synthetic_receipt(report, report["cycles"] + 1))
        self.assertEqual(report["next"]["stage"], "verify_transfer")
        valid = synthetic_receipt(report, report["cycles"] + 1)
        for defect in ("missing_clear", "one_sample", "short_span", "before_strip_release"):
            event = copy.deepcopy(valid)
            if defect == "missing_clear":
                event["evidence"].remove("both_grippers_clear")
            elif defect == "one_sample":
                event["stability_samples"].pop(0)
            elif defect == "short_span":
                event["stability_samples"][0]["at"] = event["at"] - 1
            else:
                event["stability_samples"][0]["at"] = ledger.last_at - 1
            with self.subTest(defect=defect), self.assertRaises(PipelineContractError):
                ledger.record_cycle(event)
        completed = ledger.record_cycle(valid)
        self.assertEqual(completed["termination_reason"], "offline_contract_completed")
        self.assertFalse(completed["physical_success_measurable"])
        self.assertFalse(completed["execution_available"])
        self.assertEqual(completed["model_calls"], 0)

    def test_reopen_keeps_frozen_roles_progress_and_partial_send_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.sqlite"
            store = TaskSessionStore(path, clock=lambda: 100.)
            report = store.initialize("plug-transfer", self.source["task_id"],
                                      mode="dual_arm", task_definition=self.source)
            while report["next"]["stage"] != "extract_plug":
                number = report["cycles"] + 1
                report = store.append("plug-transfer", expected_revision=report["revision"],
                    event_id="synthetic-cycle-" + str(number), kind="cycle",
                    payload=synthetic_receipt(report, number))
            reopened = TaskSessionStore(path, clock=lambda: 107.)
            current = reopened.current("plug-transfer", include_contract=True)
            self.assertEqual(current["next"], report["next"])
            self.assertEqual(current["contract"]["roles"]["peer_arm"], "left")
            self.assertEqual(current["seconds_left"], 10793.)
            self.assertEqual(current["contract_sha256"], report["contract_sha256"])
            event = synthetic_receipt(current, current["cycles"] + 1)
            event.update(status="unknown", evidence=[], partial_send=True)
            fault = reopened.append("plug-transfer", expected_revision=current["revision"],
                event_id="synthetic-partial-send", kind="cycle", payload=event)
            self.assertEqual(fault["termination_reason"], "execution_failed_latched")
            resumed = reopened.initialize("plug-transfer", self.source["task_id"],
                                          mode="dual_arm", task_definition=self.source)
            self.assertEqual(resumed["revision"], fault["revision"])
            self.assertEqual(resumed["termination_reason"], "execution_failed_latched")
            self.assertIsNone(resumed["next"])


if __name__ == "__main__":
    unittest.main()
