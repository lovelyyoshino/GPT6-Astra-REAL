"""Portable recipe contracts and durable offline sessions; no device/model I/O."""
import copy
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest

from right_pick.fast_pipeline import PipelineContractError
from right_pick.fast_task_pipeline import BoundedTaskPipeline, composition_contract, task_contract
from right_pick.fast_task_session import TaskSessionStore, main as session_main
from right_pick.fast_task_spec import load_recipe, recipe_contract


def definition():
    return {
        "schema_version": "astra_recipe_v1", "task_id": "marker-to-visible-region",
        "initial_condition": "Marker and destination boundary visible in current RGB",
        "goal": "Marker remains within the declared destination after withdrawal",
        "constraints": ["Do not infer calibrated object coordinates"],
        "steps": [
            {"id": "look", "operation": "inspect", "goal": "Verify the scene"},
            {"id": "place", "operation": "push_segment", "goal": "Move into the visible region",
             "evidence": ["marker_inside_declared_region"], "max_cycles": 3},
            {"id": "verify", "operation": "stable_verify", "goal": "Verify independent stability"},
            {"id": "return", "operation": "return_reference", "goal": "Verify return separately"},
        ],
    }


def receipt(report, number, *, status="complete"):
    stage = report["next"]
    value = dict(run_id=report["run_id"], task=stage["task"], stage=stage["stage"], arm=stage["arm"],
                 observation_id="frame-" + str(number), at=10. * number, model_called=True,
                 prerequisites=stage["needs"], evidence=stage["expect"] if status == "complete" else [],
                 status=status)
    if stage["skill"] == "stable_verify" and status == "complete":
        value["stability_samples"] = [
            dict(observation_id="first-" + str(number), at=value["at"] - 2,
                 support_stable=True, gripper_clear=True),
            dict(observation_id=value["observation_id"], at=value["at"],
                 support_stable=True, gripper_clear=True),
        ]
    return value


class PortableRecipeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.recipe_path = self.root / "recipe.json"
        self.store_path = self.root / "sessions.sqlite"
        self.now = [1000.]

    def store(self):
        return TaskSessionStore(self.store_path, clock=lambda: self.now[0])

    def write_recipe(self, value=None):
        self.recipe_path.write_text(json.dumps(definition() if value is None else value))
        return self.recipe_path

    def append(self, store, report, number, **kwargs):
        return store.append(report["run_id"], expected_revision=report["revision"],
                            event_id="cycle-" + str(number), kind="cycle",
                            payload=receipt(report, number, **kwargs))

    def test_generic_recipe_reuses_primitives_without_granting_execution(self):
        source = definition()
        saved = copy.deepcopy(source)
        contract = recipe_contract(source)
        self.assertEqual(source, saved)
        self.assertEqual(contract["source"], "user_declarative_recipe")
        self.assertEqual(contract["implementation"], "offline_contract_only")
        self.assertFalse(contract["execution_available"])
        self.assertEqual(contract["steps"][1]["max_cycles"], 3)
        expected = composition_contract("push_segment")
        self.assertTrue(set(expected["evidence"]) <= set(contract["steps"][1]["evidence"]))
        self.assertIn("marker_inside_declared_region", contract["steps"][1]["evidence"])
        self.assertEqual(contract["steps"][1]["requirements"], expected["requirements"])

    def test_wiping_motion_cannot_silently_stand_for_an_erasure_goal(self):
        motion = task_contract("blackboard-wiping")
        self.assertIn("erasure completion is a separate task variant", motion["goal"])
        self.assertIn("motion_not_erasure", motion["constraints"])
        source = definition()
        source["task_id"] = "blackboard_erasure"
        source["initial_condition"] = "Preheld eraser and the specified visible marks on a fixed board"
        source["goal"] = "Remove the initially specified visible marks and verify the result"
        source["steps"][1] = dict(id="erase", operation="wipe_segment",
            goal="Remove the specified marks through visible tool contact",
            evidence=["erasure_criterion_satisfied"])
        source["steps"][2]["evidence"] = ["erasure_criterion_satisfied"]
        store = self.store()
        report = store.initialize("erasure", source["task_id"], task_definition=source)
        report = self.append(store, report, 1)
        event = receipt(report, 2)
        event["evidence"].remove("erasure_criterion_satisfied")
        with self.assertRaises(PipelineContractError):
            store.append("erasure", expected_revision=report["revision"], event_id="wipe-only",
                         kind="cycle", payload=event)
        current = store.current("erasure")
        self.assertEqual(current["revision"], report["revision"])
        self.assertEqual(current["next"]["stage"], "erase")

    def test_target_support_and_articulation_do_not_inherit_wrong_object_semantics(self):
        for operation, required, forbidden in (
                ("place_on_support", "object_on_target_support", "object_on_original_support"),
                ("articulated_rotate", "visual_goal_reached", "thread_progress")):
            candidate = definition()
            candidate["steps"][1]["operation"] = operation
            step = recipe_contract(candidate)["steps"][1]
            with self.subTest(operation=operation):
                self.assertIn(required, step["evidence"])
                self.assertNotIn(forbidden, step["evidence"])
                self.assertEqual(composition_contract(operation)["primitives"], ["move_eef_once"])
                self.assertEqual(composition_contract(operation)["max_dispatches_per_cycle"], 1)
        # Existing operations retain their narrower meaning for old recipes.
        self.assertIn("object_on_original_support", composition_contract("lower_to_support")["evidence"])
        self.assertIn("thread_progress", composition_contract("rotate_segment")["evidence"])

    def test_exact_fields_prevent_executable_code_action_targets_and_site_permissions(self):
        for key, value in (("code", "print('untrusted')"), ("budget", {"max_cycles": 10000}),
                           ("execution_available", True), ("site_qualified", True)):
            candidate = definition()
            candidate[key] = value
            with self.subTest(top_level=key), self.assertRaises(PipelineContractError):
                recipe_contract(candidate)
        for key, value in (("action", "move_eef"), ("args", {"pose_m_rad": [0] * 6}),
                           ("python", "robot.move()"), ("speed_percent", 50), ("next_stage", "loop")):
            candidate = definition()
            candidate["steps"][1][key] = value
            with self.subTest(stage_field=key), self.assertRaises(PipelineContractError):
                recipe_contract(candidate)
        candidate = definition()
        del candidate["goal"]
        with self.assertRaises(PipelineContractError):
            recipe_contract(candidate)

    def test_unknown_operation_and_view_only_branch_cannot_be_task_skills(self):
        for operation in ("move_eef", "reset_driver", "observer_reposition", None):
            candidate = definition()
            candidate["steps"][1]["operation"] = operation
            with self.subTest(operation=operation), self.assertRaises(PipelineContractError):
                recipe_contract(candidate, mode="worker_with_observer")

    def test_step_budget_may_only_reduce_registered_cap(self):
        cap = composition_contract("push_segment")["max_cycles"]
        for value in (0, -1, cap + 1, True, 1.0):
            candidate = definition()
            candidate["steps"][1]["max_cycles"] = value
            with self.subTest(value=value), self.assertRaises(PipelineContractError):
                recipe_contract(candidate)
        candidate = definition()
        candidate["steps"][1]["max_cycles"] = 1
        self.assertEqual(recipe_contract(candidate)["steps"][1]["max_cycles"], 1)

    def test_inspection_and_final_stability_cannot_be_skipped_or_moved_before_task_work(self):
        candidates = []
        missing_inspect = definition()
        missing_inspect["steps"] = missing_inspect["steps"][1:]
        candidates.append(missing_inspect)
        missing_stability = definition()
        missing_stability["steps"].pop(2)
        candidates.append(missing_stability)
        early_stability = definition()
        early_stability["steps"][1:3] = reversed(early_stability["steps"][1:3])
        candidates.append(early_stability)
        late_release = definition()
        late_release["steps"].insert(3, dict(id="late-release", operation="release_retreat", goal="Release"))
        candidates.append(late_release)
        for index, candidate in enumerate(candidates):
            with self.subTest(case=index), self.assertRaises(PipelineContractError):
                recipe_contract(candidate)

    def test_roles_allow_left_worker_or_two_task_arms_but_not_observer_work(self):
        candidate = definition()
        candidate["steps"][1]["arm"] = "left"
        left = recipe_contract(candidate, worker_arm="left")
        self.assertEqual({s["arm"] for s in left["steps"]}, {"left"})
        dual = recipe_contract(candidate, mode="dual_arm", worker_arm="right")
        self.assertEqual({s["arm"] for s in dual["steps"]}, {"left", "right"})
        for mode in ("single_arm", "worker_with_observer"):
            with self.subTest(mode=mode), self.assertRaises(PipelineContractError):
                recipe_contract(candidate, mode=mode, worker_arm="right")
        for arm in ("both", "observer", "middle", True):
            candidate["steps"][1]["arm"] = arm
            with self.subTest(arm=arm), self.assertRaises(PipelineContractError):
                recipe_contract(candidate, mode="dual_arm")

    def test_file_loader_uses_requested_roles_and_rejects_ambiguous_duplicate_json_keys(self):
        candidate = definition()
        candidate["steps"][1]["arm"] = "left"
        path = self.write_recipe(candidate)
        self.assertEqual(load_recipe(path, mode="single_arm", worker_arm="left"), candidate)
        with self.assertRaises(PipelineContractError):
            load_recipe(path)
        self.recipe_path.write_text('{"goal":"shadowed",' + json.dumps(definition())[1:])
        with self.assertRaises((PipelineContractError, ValueError)):
            load_recipe(self.recipe_path)

    def test_source_is_bounded_and_identifiers_cannot_alias_stages(self):
        candidate = definition()
        candidate["steps"][1]["id"] = candidate["steps"][0]["id"]
        with self.assertRaises(PipelineContractError):
            recipe_contract(candidate)
        candidate = definition()
        candidate["steps"] = candidate["steps"] * 17
        with self.assertRaises(PipelineContractError):
            recipe_contract(candidate)
        self.recipe_path.write_text(" " * 65537)
        with self.assertRaises(PipelineContractError):
            load_recipe(self.recipe_path)

    def test_frozen_recipe_survives_changed_or_missing_source_and_store_reopen(self):
        source = load_recipe(self.write_recipe())
        store = self.store()
        first = store.initialize("recipe-run", source["task_id"], task_definition=source)
        report = self.append(store, first, 1)
        digest = first["contract_sha256"]
        source["goal"] = "External mutation is not the recorded goal"
        self.recipe_path.unlink()
        reopened = self.store().current("recipe-run", include_contract=True, include_operation=True)
        self.assertEqual(reopened["revision"], 1)
        self.assertEqual(reopened["next"], report["next"])
        self.assertEqual(reopened["contract"]["goal"], definition()["goal"])
        self.assertEqual(reopened["contract_sha256"], digest)
        self.assertEqual(reopened["operation"]["contract"]["id"], "push_segment")

    def test_changed_recipe_roles_or_budget_cannot_replace_existing_run_or_reset_debt(self):
        original = definition()
        store = self.store()
        report = store.initialize("same-run", original["task_id"], task_definition=original,
                                  budget={"max_elapsed_s": 20})
        report = self.append(store, report, 1)
        report = self.append(store, report, 2, status="unknown")
        self.now[0] += 7
        changed = definition()
        changed["goal"] = "A different task outcome"
        for kwargs in (dict(task_definition=changed, budget={"max_elapsed_s": 20}),
                       dict(task_definition=original, worker_arm="left", budget={"max_elapsed_s": 20}),
                       dict(task_definition=original, budget={"max_elapsed_s": 19})):
            with self.subTest(kwargs=kwargs), self.assertRaises(PipelineContractError):
                self.store().initialize("same-run", original["task_id"], **kwargs)
        report = self.store().initialize("same-run", original["task_id"], task_definition=original,
                                         budget={"max_elapsed_s": 20})
        self.assertEqual((report["revision"], report["cycles"], report["model_calls"]), (2, 2, 2))
        self.assertEqual(report["seconds_left"], 13)
        report = self.append(self.store(), report, 3, status="unknown")
        self.assertEqual(report["termination_reason"], "no_progress_budget_exhausted")
        self.assertEqual(self.store().initialize("same-run", original["task_id"], task_definition=original,
            budget={"max_elapsed_s": 20})["termination_reason"], "no_progress_budget_exhausted")

    def test_custom_completion_requires_new_stability_samples_and_remains_offline(self):
        source = definition()
        store = self.store()
        report = store.initialize("complete", source["task_id"], task_definition=source)
        report = self.append(store, report, 1)
        report = self.append(store, report, 2)
        bad = receipt(report, 3)
        bad.pop("stability_samples")
        with self.assertRaises(PipelineContractError):
            store.append("complete", expected_revision=2, event_id="bad-stable", kind="cycle", payload=bad)
        self.assertEqual(store.current("complete")["revision"], 2)
        report = self.append(store, report, 3)
        report = self.append(store, report, 4)
        self.assertTrue(report["contract_completed"])
        self.assertFalse(report["execution_available"])
        self.assertFalse(report["physical_success_measurable"])

    def test_existing_builtin_contract_and_recipe_task_id_binding_stay_compatible(self):
        builtin = task_contract("pen", worker_arm="left")
        ledger = BoundedTaskPipeline("pen", worker_arm="left")
        self.assertEqual(ledger.contract, builtin)
        report = self.store().initialize("builtin", "pen", worker_arm="left")
        self.assertEqual(report["next"]["arm"], "left")
        self.assertEqual(self.store().current("builtin", include_contract=True)["contract"], builtin)
        with self.assertRaises(PipelineContractError):
            BoundedTaskPipeline("another-task", task_definition=definition())

    def test_recipe_cli_keeps_explicit_left_role_and_reads_frozen_contract_without_file(self):
        source = definition()
        source["steps"][1]["arm"] = "left"
        self.write_recipe(source)
        args = ["--store", str(self.store_path), "init", "--run-id", "cli-left",
                "--recipe", str(self.recipe_path), "--worker-arm", "left"]
        with redirect_stdout(io.StringIO()) as output:
            code = session_main(args)
        self.assertEqual(code, 0, output.getvalue())
        first = json.loads(output.getvalue())
        self.assertEqual(first["next"]["arm"], "left")
        self.recipe_path.unlink()
        with redirect_stdout(io.StringIO()) as output:
            code = session_main(["--store", str(self.store_path), "contract", "--run-id", "cli-left"])
        self.assertEqual(code, 0, output.getvalue())
        frozen = json.loads(output.getvalue())
        self.assertEqual(frozen["contract_sha256"], first["contract_sha256"])
        self.assertEqual({s["arm"] for s in frozen["contract"]["steps"]}, {"left"})


if __name__ == "__main__":
    unittest.main()
