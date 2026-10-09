"""持久化 pipeline 的跨命令、幂等和预算回归；所有事件均为离线夹具。

@author Codex
@date 2026-10-07
@version v1.0.0
@last_modified 2026-10-07
@changelog
  - v1.0.0 (2026-10-07): 验证重启、重复事件、角色分支、事务冲突和总预算。
"""
from concurrent.futures import ThreadPoolExecutor
import copy
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from right_pick import fast_task_pipeline, fast_task_spec
from right_pick.fast_task_pipeline import BoundedTaskPipeline, TASK_RECIPES
from right_pick.fast_task_session import TaskSessionStore


def receipt(report, number=1, status="complete", **extra):
    current = report["next"]
    value = dict(run_id=report["run_id"], task=current["task"], stage=current["stage"], arm=current["arm"],
                 observation_id="frame-" + str(number), at=number * 10., model_called=True,
                 status=status, prerequisites=current["needs"],
                 evidence=current["expect"] if status == "complete" else [])
    if current["skill"] == "stable_verify":
        value["stability_samples"] = [
            dict(observation_id="stable-first-" + str(number), at=number * 10. - 2,
                 support_stable=True, gripper_clear=True),
            dict(observation_id=value["observation_id"], at=value["at"], support_stable=True, gripper_clear=True)]
    value.update(extra)
    return value


class PersistentTaskTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "tasks.sqlite"
        self.now = [1000.]
        self.store = TaskSessionStore(self.path, clock=lambda: self.now[0])

    def append(self, report, number=1, **changes):
        return self.store.append(report["run_id"], expected_revision=report["revision"],
                                 event_id="cycle-" + str(number), kind="cycle",
                                 payload=receipt(report, number, **changes))

    def test_legacy_default_budget_reopens_without_changing_frozen_history(self):
        definition = dict(schema_version="astra_recipe_v1", task_id="legacy-recipe",
                          initial_condition="Marker rests on table", goal="Inspect marker",
                          constraints=[], steps=[dict(id="look", operation="inspect", goal="Inspect marker"),
                                                  dict(id="verify", operation="stable_verify", goal="Verify stability")])
        for module, name, task_id, kwargs in (
                (fast_task_pipeline, "task_contract", "pen", {}),
                (fast_task_spec, "recipe_contract", "legacy-recipe", {"task_definition": definition})):
            with self.subTest(task_id=task_id):
                original = getattr(module, name)
                def legacy(*args, **options):
                    contract = original(*args, **options)
                    contract["budget"].update(max_cycles=128, max_model_calls=64, max_elapsed_s=900)
                    return contract
                with patch.object(module, name, side_effect=legacy):
                    initial = self.store.initialize(task_id, task_id, **kwargs)
                    progress = self.append(initial)
                with sqlite3.connect(self.path) as db:
                    frozen = db.execute("SELECT config, contract, created_wall FROM task_sessions WHERE run_id=?",
                                        (task_id,)).fetchone()
                self.now[0] += 5
                current = self.store.current(task_id, include_contract=True)
                self.assertEqual((current["cycles"], current["revision"], current["seconds_left"]), (1, 1, 895))
                self.assertEqual(current["contract"]["budget"]["max_cycles"], 128)
                self.assertEqual(current["contract_sha256"], progress["contract_sha256"])
                self.assertEqual(self.store.initialize(task_id, task_id, **kwargs)["seconds_left"], 895)
                with self.assertRaisesRegex(ValueError, "different frozen"):
                    self.store.initialize(task_id, task_id, budget={"max_elapsed_s":10800}, **kwargs)
                with sqlite3.connect(self.path) as db:
                    self.assertEqual(frozen, db.execute(
                        "SELECT config, contract, created_wall FROM task_sessions WHERE run_id=?", (task_id,)).fetchone())

    def test_legacy_compatibility_refuses_other_contract_changes(self):
        self.store.initialize("old", "pen")
        with sqlite3.connect(self.path) as db:
            stored = db.execute("SELECT contract FROM task_sessions WHERE run_id='old'").fetchone()[0]
        legacy = json.loads(stored)
        legacy["budget"].update(max_cycles=128, max_model_calls=64, max_elapsed_s=900)
        for mutate in (lambda c: c.update(goal="Different task goal"),
                       lambda c: c["budget"].update(max_no_progress=3),
                       lambda c: c["budget"].update(max_rejections=True),
                       lambda c: c["budget"].update(max_elapsed_s=901)):
            changed = copy.deepcopy(legacy)
            mutate(changed)
            with sqlite3.connect(self.path) as db:
                db.execute("UPDATE task_sessions SET contract=? WHERE run_id='old'", (json.dumps(changed),))
            with self.assertRaisesRegex(ValueError, "Task contract changed"):
                self.store.current("old")

    def test_reopening_and_reinitializing_preserve_progress_and_total_budget(self):
        report = self.store.initialize("pen-a", "pen", budget={"max_elapsed_s": 10})
        report = self.append(report)
        self.now[0] += 4
        reopened = TaskSessionStore(self.path, clock=lambda: self.now[0])
        same = reopened.initialize("pen-a", "pen", budget={"max_elapsed_s": 10})
        self.assertEqual(same["revision"], 1)
        self.assertEqual(same["next"]["stage"], report["next"]["stage"])
        self.assertEqual(same["model_calls"], 1)
        self.assertEqual(same["seconds_left"], 6)
        self.now[0] += 6
        ended = reopened.current("pen-a")
        self.assertEqual(ended["termination_reason"], "wall_time_budget_exhausted")
        self.assertIsNone(ended["next"])
        self.assertEqual(reopened.initialize("pen-a", "pen", budget={"max_elapsed_s": 10})["seconds_left"], 0)

    def test_frozen_run_cannot_change_task_arm_or_budget(self):
        self.store.initialize("a", "pen")
        for changes in ({"task_id": "hat"}, {"worker_arm": "left"}, {"budget": {"max_model_calls": 100}},
                        {"mode": "worker_with_observer"}):
            args = dict(task_id="pen")
            args.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.store.initialize("a", **args)
        self.assertEqual(self.store.current("a")["revision"], 0)

    def test_duplicate_receipt_is_idempotent_even_after_stage_advanced(self):
        first = self.store.initialize("a", "pen")
        second = self.append(first)
        duplicate = self.store.append("a", expected_revision=0, event_id="cycle-1", kind="cycle", payload=receipt(first))
        self.assertTrue(duplicate["duplicate_event"])
        self.assertEqual(duplicate["next"], second["next"])
        self.assertEqual(duplicate["cycles"], 1)
        self.assertEqual(duplicate["model_calls"], 1)
        with self.assertRaisesRegex(ValueError, "different payload"):
            self.store.append("a", expected_revision=0, event_id="cycle-1", kind="cycle",
                              payload=receipt(first, observation_id="different"))

    def test_old_revision_and_other_run_receipt_cannot_advance(self):
        initial = self.store.initialize("a", "pen")
        report = self.append(initial)
        with self.assertRaisesRegex(ValueError, "Stale revision"):
            self.store.append("a", expected_revision=0, event_id="old-stage", kind="cycle", payload=receipt(report, 2))
        other = self.store.initialize("b", "pen")
        with self.assertRaisesRegex(ValueError, "exact run_id"):
            self.store.append("b", expected_revision=0, event_id="copied", kind="cycle", payload=receipt(initial))
        self.assertEqual(self.store.current("b")["next"], other["next"])

    def test_invalid_event_rolls_back_then_valid_event_can_use_same_revision(self):
        initial = self.store.initialize("a", "pen")
        with self.assertRaisesRegex(ValueError, "expected evidence"):
            self.append(initial, evidence=[])
        self.assertEqual(self.store.current("a")["revision"], 0)
        self.assertEqual(self.append(initial)["revision"], 1)

    def test_two_writers_cannot_both_consume_one_stage(self):
        initial = self.store.initialize("a", "pen")
        def write(event_id):
            try:
                return TaskSessionStore(self.path, clock=lambda: self.now[0]).append(
                    "a", expected_revision=0, event_id=event_id, kind="cycle", payload=receipt(initial))
            except ValueError as exc:
                return str(exc)
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(write, ["writer-one", "writer-two"]))
        self.assertEqual(sum(isinstance(r, dict) for r in results), 1)
        self.assertEqual(self.store.current("a")["cycles"], 1)

    def test_clock_regression_latches_even_when_wall_clock_recovers(self):
        self.store.initialize("a", "pen")
        self.now[0] += 5
        self.store.current("a")
        self.now[0] -= 3
        report = self.store.current("a")
        self.assertEqual(report["termination_reason"], "session_clock_regressed")
        self.assertEqual(report["elapsed_s"], 5)
        self.now[0] += 10
        self.assertEqual(self.store.current("a")["termination_reason"], "session_clock_regressed")

    def test_observer_branch_survives_restart_and_returns_to_worker_stage(self):
        report = self.append(self.store.initialize("a", "pen", mode="worker_with_observer", worker_arm="left"))
        original = report["next"]
        hold = dict(run_id="a", arm="left", stationary=True, hold_verified=True, observation_id="frame-1", at=10.)
        report = self.store.append("a", expected_revision=1, event_id="view-1", kind="observer", payload=hold)
        reopened = TaskSessionStore(self.path, clock=lambda: self.now[0]).current("a")
        self.assertEqual(reopened["next"]["arm"], "right")
        self.assertEqual(reopened["next"]["skill"], "observer_reposition")
        report = self.append(report, 2)
        self.assertEqual(report["next"]["stage"], original["stage"])
        self.assertEqual(report["next"]["arm"], "left")
        self.assertEqual(report["model_calls"], 2)

    def test_all_recipes_can_replay_persisted_evidence_without_physical_success(self):
        for recipe in TASK_RECIPES:
            mode = "dual_arm" if recipe.dual_required else "single_arm"
            report = self.store.initialize(recipe.task_id, recipe.task_id, mode=mode)
            for number in range(1, len(BoundedTaskPipeline(recipe.task_id, mode=mode).contract["steps"]) + 1):
                report = self.append(report, number)
            restored = TaskSessionStore(self.path, clock=lambda: self.now[0]).current(recipe.task_id)
            self.assertTrue(restored["contract_completed"], recipe.task_id)
            self.assertFalse(restored["physical_success_measurable"])
            self.assertFalse(restored["execution_available"])

    def test_observer_completion_preserves_worker_budget_after_reopening(self):
        report = self.append(self.store.initialize("view-debt", "pen", mode="worker_with_observer"))
        report = self.append(report, 2, status="unknown")
        stage = report["next"]["stage"]
        hold = dict(run_id="view-debt", arm="right", stationary=True, hold_verified=True,
                    observation_id="frame-2", at=20.)
        report = self.store.append("view-debt", expected_revision=2, event_id="observer-1",
                                  kind="observer", payload=hold)
        report = self.append(report, 3)
        reopened = TaskSessionStore(self.path, clock=lambda: self.now[0])
        report = reopened.current("view-debt")
        self.assertEqual(report["next"]["stage"], stage)
        report = reopened.append("view-debt", expected_revision=report["revision"], event_id="cycle-4",
            kind="cycle", payload=receipt(report, 4, status="unknown"))
        self.assertEqual(report["termination_reason"], "no_progress_budget_exhausted")
        self.assertEqual(report["model_calls"], 4)

    def test_unknown_budget_and_uncertain_receipt_are_sticky(self):
        report = self.store.initialize("unknown", "pen")
        report = self.append(report, status="unknown")
        report = self.append(report, 2, status="unknown")
        self.assertEqual(self.store.current("unknown")["termination_reason"], "no_progress_budget_exhausted")
        report = self.store.initialize("fault", "cups", mode="dual_arm")
        report = self.append(report, status="fault", outcome_uncertain=True)
        self.assertEqual(self.store.current("fault")["termination_reason"], "execution_failed_latched")

    def test_progress_measurement_and_active_operation_survive_replay(self):
        report = self.store.initialize("progress", "pen")
        current = self.store.current("progress", include_operation=True)
        self.assertEqual(current["operation"]["contract"]["id"], "inspect")
        self.assertNotIn("operation", self.store.current("progress"))
        report = self.append(report, status="progress", progress_measurement=dict(
            metric="visible_displacement", unit="mm", before=0, after=1,
            observation_id="frame-1"))
        self.assertEqual(report["revision"], 1)
        reopened = TaskSessionStore(self.path, clock=lambda: self.now[0])
        report = reopened.append("progress", expected_revision=1, event_id="cycle-2", kind="cycle",
            payload=receipt(report, 2, status="progress", progress_measurement=dict(
                metric="visible_displacement", unit="mm", before=0, after=1,
                observation_id="frame-2")))
        self.assertEqual(report["termination_reason"], "stage_budget_exhausted")
        self.assertEqual(report["revision"], 2)

    def test_end_records_reason_and_cannot_restart(self):
        initial = self.store.initialize("a", "pen")
        self.store.append("a", expected_revision=0, event_id="end", kind="end",
                          payload={"reason": "offline analysis complete"})
        ended = self.store.initialize("a", "pen")
        self.assertEqual(ended["termination_reason"], "offline_session_ended")
        self.assertEqual(ended["termination_detail"], "offline analysis complete")
        with self.assertRaisesRegex(ValueError, "Stale revision"):
            self.append(initial)
        unchanged = self.store.append("a", expected_revision=1, event_id="late-cycle", kind="cycle",
                                      payload=receipt(initial))
        self.assertEqual(unchanged["revision"], 1)
        self.assertEqual(unchanged["cycles"], 0)
        self.assertFalse(unchanged["event_applied"])

    def test_contract_drift_and_corrupt_journal_never_silently_reset(self):
        self.store.initialize("a", "pen")
        with sqlite3.connect(self.path) as connection:
            connection.execute("UPDATE task_sessions SET contract='{}' WHERE run_id='a'")
        with self.assertRaisesRegex(ValueError, "contract changed"):
            self.store.current("a")

    def test_real_cli_processes_reuse_state_and_reject_malformed_receipts(self):
        prefix = [sys.executable, "-m", "right_pick.fast_task_session", "--store", str(self.path)]
        def cli(*args):
            result = subprocess.run(prefix + list(args), capture_output=True, text=True, timeout=10)
            return result.returncode, json.loads(result.stdout)
        code, first = cli("init", "--run-id", "cli", "--task", "pen")
        self.assertEqual(code, 0)
        file = Path(self.directory.name) / "receipt.json"
        file.write_text(json.dumps(receipt(first)))
        args = ["record", "--run-id", "cli", "--revision", "0", "--event-id", "cli-cycle", "--receipt", str(file)]
        code, second = cli(*args)
        self.assertEqual(code, 0)
        self.assertEqual(second["revision"], 1)
        code, duplicate = cli(*args)
        self.assertTrue(duplicate["duplicate_event"])
        code, current = cli("current", "--run-id", "cli")
        self.assertEqual(current["next"]["stage"], second["next"]["stage"])
        code, expanded = cli("current", "--run-id", "cli", "--operation")
        self.assertEqual(code, 0)
        self.assertEqual(expanded["operation"]["stage"], second["next"]["stage"])
        self.assertEqual(expanded["operation"]["contract"]["id"], second["next"]["skill"])
        file.write_text("{invalid-json")
        code, error = cli(*args)
        self.assertEqual(code, 2)
        self.assertFalse(error["execution_available"])


if __name__ == "__main__":
    unittest.main()
