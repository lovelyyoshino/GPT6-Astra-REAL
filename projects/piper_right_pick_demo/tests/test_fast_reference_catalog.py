"""Cross-check reviewed ARX5 evidence coverage without fetching or actuating."""
import ast
import json
from pathlib import Path
import re
import unittest

from right_pick.fast_task_pipeline import TASK_RECIPES, task_contract


ROOT = Path(__file__).resolve().parents[3]
MATRIX = ROOT / "research/arx5_experiment_patterns_round2.json"
PIN = "2a722748fa2adedd096c8fd7f9461d2b2ba48af6"
TASKS = {
    "cups", "pen", "charger", "charger-insert-only", "flower", "hat",
    "pen-uncapping", "pearl-pouring", "orange-pick-place", "drawer-push-pull",
    "jelly-pick-place", "bottle-unscrewing", "fixed-bottle-unscrewing",
    "bolt-screwing", "book-extraction", "blackboard-wiping",
    "blue-blocks-sweeping", "blue-block-triangle-push",
}


class ReferenceCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix = json.loads(MATRIX.read_text())
        cls.rows = {row["task_id"]: row for row in cls.matrix["experiments"]}
        cls.sources = {row["path"]: row for row in cls.matrix["source_manifest"]}

    def test_exactly_eighteen_source_tasks_and_builtin_contracts_are_covered(self):
        self.assertEqual(self.matrix["task_reference_count"], 18)
        self.assertEqual(len(self.matrix["experiments"]), 18)
        self.assertEqual(set(self.rows), TASKS)
        self.assertEqual({task.task_id for task in TASK_RECIPES}, TASKS)
        for row in self.rows.values():
            self.assertIn(row["reference_path"], self.sources)
            self.assertTrue(row["report_paths"])
            for report in row["report_paths"]:
                self.assertIn(report, self.sources)

    def test_sources_are_pinned_with_content_digests_not_moving_branch_links(self):
        self.assertEqual(self.matrix["head_commit"], PIN)
        self.assertEqual(self.matrix["previously_pinned_commit"], PIN)
        self.assertFalse(self.matrix["head_changed_since_first_review"])
        self.assertEqual(len(self.sources), len(self.matrix["source_manifest"]))
        for source in self.sources.values():
            self.assertEqual(source["commit"], PIN)
            self.assertIn("/" + PIN + "/", source["url"])
            self.assertIn("/" + PIN + "/", source["raw_url"])
            self.assertIsNotNone(re.fullmatch(r"[0-9a-f]{64}", source["sha256"]))
            self.assertGreater(source["bytes"], 0)
            self.assertTrue(source["review_scope"])

    def test_reviewed_operations_roles_and_facts_resolve_to_local_contracts(self):
        for row in self.rows.values():
            mapped = row["local_mapping"]
            actual = task_contract(row["task_id"], mode=mapped["mode"],
                                   worker_arm=mapped["worker_arm"])
            self.assertEqual(mapped["operations"],
                list(dict.fromkeys(step["skill"] for step in actual["steps"])))
            self.assertEqual(mapped["constraints"], actual["constraints"])
            self.assertEqual(mapped["steps"], [
                {key: step[key] for key in ("skill", "arm", "evidence", "requirements")}
                for step in actual["steps"]])
            self.assertEqual(mapped["implementation"], "offline_contract_only")
            self.assertFalse(mapped["execution_available"])
            self.assertFalse(actual["execution_available"])

    def test_object_specific_evidence_cannot_be_collapsed_to_robot_arrival(self):
        required = {
            "drawer-push-pull": {"drawer_relative_travel", "initial_opening_restored", "cabinet_stable"},
            "book-extraction": {"new_book_travel_verified", "other_books_stable"},
            "bolt-screwing": {"three_actual_thread_turns", "axial_feed_verified", "bolt_stable"},
            "fixed-bottle-unscrewing": {"threads_disengaged", "cap_separation_verified", "fixture_stable"},
            "hat": {"hat_on_specified_hook", "rack_stable"},
            "blue-blocks-sweeping": {"all_targets_inside"},
            "blue-block-triangle-push": {"block_inside_region", "edge_push_without_grasp"},
        }
        for task, facts in required.items():
            mapped = self.rows[task]["local_mapping"]
            available = {fact for step in mapped["steps"] for fact in step["evidence"]}
            self.assertTrue(facts <= available, task)

    def test_preinsert_and_preheld_tool_conditions_keep_distinct_operations(self):
        insert = self.rows["charger-insert-only"]["local_mapping"]["operations"]
        self.assertIn("grip_supported", insert)
        self.assertNotIn("grip_test", insert)
        for task in ("flower", "blackboard-wiping", "blue-blocks-sweeping"):
            self.assertIn("validate_preheld", self.rows[task]["local_mapping"]["operations"])
        self.assertNotIn("grip_test", self.rows["blue-block-triangle-push"]["local_mapping"]["operations"])
        self.assertEqual(self.rows["bottle-unscrewing"]["local_mapping"]["mode"], "dual_arm")
        self.assertEqual(self.rows["fixed-bottle-unscrewing"]["local_mapping"]["mode"], "worker_with_observer")

    def test_trial_claims_are_separate_from_required_evidence_and_local_success(self):
        for row in self.rows.values():
            self.assertTrue(row["required_success_evidence"])
            self.assertTrue(row["reported_variants"])
            self.assertTrue(row["source_roles_and_human_initial_conditions"])
            self.assertTrue(row["local_mapping"]["physical_implementation_missing"])
            self.assertIsNone(row["physical_success_in_this_repository"])
            for variant in row["reported_variants"]:
                self.assertTrue(variant["human_or_protocol_conditions"])
                self.assertTrue(variant["evidence_scope"])
                self.assertFalse(variant["independently_reproduced_here"])
        self.assertGreaterEqual(len(self.rows["flower"]["reported_variants"]), 3)
        self.assertEqual(len(self.rows["blackboard-wiping"]["reported_variants"]), 2)
        self.assertFalse(self.matrix["validation"]["hardware_used"])
        self.assertFalse(self.matrix["validation"]["model_called"])

    def test_every_test_mapping_names_a_real_offline_test(self):
        parsed = {}
        for row in self.rows.values():
            for entry in row["local_mapping"]["tests"]:
                file = ROOT / entry["file"]
                self.assertTrue(file.is_file(), entry["file"])
                if file not in parsed:
                    parsed[file] = {node.name for node in ast.walk(ast.parse(file.read_text()))
                                    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
                self.assertIn(entry["test"], parsed[file])

    def test_erasure_semantic_review_is_recorded_as_resolved_without_pixel_claim(self):
        finding = next(item for item in self.matrix["semantic_findings"]
                       if item["id"] == "blackboard_goal_criterion")
        self.assertEqual(finding["status"], "default_goal_semantics_fixed")
        goal = task_contract("blackboard-wiping")["goal"]
        self.assertIn("separate task variant", goal)
        self.assertTrue(finding["implementation_changed_in_this_round"])
        self.assertIn("pixel", finding["remaining_limitation"])


if __name__ == "__main__":
    unittest.main()
