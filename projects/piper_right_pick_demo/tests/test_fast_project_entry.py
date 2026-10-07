"""Actual offline CLI subprocesses; device/network/model startup is prohibited."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
ENTRY = ROOT / "astra"
# Run the actual executable script in a separate Python process, without site
# packages. Attempts to import live backends or open a device cannot silently
# pass these tests. This is dependency isolation, not a simulated robot.
BOOTSTRAP = r'''
import importlib.abc, os, runpy, sys
blocked = ("rospy", "roslib", "can", "piper_sdk", "pyrealsense2", "cv2", "openai",
           "right_pick.fast_model", "right_pick.fast_codex", "right_pick.fast_ros",
           "right_pick.fast_observation")
class NoLiveImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == name or fullname.startswith(name + ".") for name in blocked):
            raise RuntimeError("Live dependency imported: " + fullname)
sys.meta_path.insert(0, NoLiveImports())
def audit(event, args):
    if event.startswith("socket.") or event in ("subprocess.Popen", "os.system", "os.exec", "os.posix_spawn"):
        raise RuntimeError("Offline entry attempted external I/O: " + event)
    if event == "open" and args and isinstance(args[0], (str, bytes)):
        path = os.fsdecode(args[0])
        if path.startswith(("/dev/", "/proc/bus/", "/sys/class/video", "/sys/class/net/")):
            raise RuntimeError("Offline entry attempted device access: " + path)
sys.addaudithook(audit)
script = sys.argv[1]
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(script))
runpy.run_path(script, run_name="__main__")
'''


def evaluation_document(source="model_visual_report"):
    document = dict(schema_version="task_evaluation_v1", run_id="reviewed-run", task_id="visible-goal",
        mode="physical", claims=[dict(id="goal", required_facts=["goal_visible"], after_claims=[],
            after_events=[], min_observations=2, min_span_s=2, terminal=True)], observations=[], events=[])
    for index, stamp in enumerate((1, 3)):
        oid = "image-" + str(index)
        document["observations"].append(dict(id=oid, run_id="reviewed-run", at=stamp,
            origin="physical_rgb", rgb_references=["host-reference:" + oid]))
        event = dict(id="review-" + str(index), run_id="reviewed-run", at=stamp, source=source,
            reference="host-review:" + oid, observation_id=oid, facts={"goal_visible": True})
        if source == "independent_rgb_review":
            event["reviewer"] = "declared-reviewer"
        document["events"].append(event)
    return document


class ProjectEntryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.temp = Path(self.directory.name)

    def cli(self, *args, expected=0):
        env = dict(os.environ)
        env.pop("PYTHONPATH", None)
        result = subprocess.run([sys.executable, "-S", "-c", BOOTSTRAP, str(ENTRY), *map(str, args)],
            cwd=str(self.temp), env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        self.assertNotIn("Traceback", result.stderr)
        return json.loads(result.stdout)

    def write(self, filename, data):
        path = self.temp / filename
        path.write_text(json.dumps(data))
        return path

    def test_catalog_and_each_example_plan_replay_are_finite_offline_contracts(self):
        catalog = self.cli("catalog")
        self.assertFalse(catalog["execution_available"])
        self.assertIn("place_on_support", catalog["operations"])
        self.assertIn("articulated_rotate", catalog["operations"])
        self.assertGreaterEqual(len(catalog["recipes"]), 5)
        for recipe in catalog["recipes"]:
            with self.subTest(recipe=recipe):
                definition = json.loads((ROOT / recipe).read_text())
                assigned = {step.get("arm") for step in definition["steps"]} - {None}
                mode = ["--mode", "dual_arm"] if assigned == {"left", "right"} else []
                plan = self.cli("plan", "--recipe", ROOT / recipe, *mode)
                replay = self.cli("replay", "--recipe", ROOT / recipe, *mode)
                self.assertEqual(plan, replay["contract"])
                self.assertFalse(plan["execution_available"])
                self.assertEqual(plan["implementation"], "offline_contract_only")
                self.assertEqual(replay["input_kind"], "synthetic_contract_fixtures")
                self.assertEqual(len(replay["events"]), len(plan["steps"]))
                self.assertLessEqual(len(replay["events"]), 64)
                self.assertEqual(replay["summary"]["termination_reason"], "offline_contract_completed")
                self.assertFalse(replay["summary"]["physical_success_measurable"])
                self.assertIsNone(replay["task_success"])
                self.assertFalse(replay["hardware_accessed"])
                self.assertFalse(replay["image_interpretation_performed"])
                self.assertEqual(replay["actual_model_call_count"], 0)
                self.assertEqual(replay["dispatched_action_count"], 0)

    def test_reference_traceability_has_real_local_code_tests_and_explicit_limits(self):
        result = self.cli("references")
        self.assertEqual(len(result["sources"]), 8)
        self.assertEqual(len({item["id"] for item in result["sources"]}), 8)
        for item in result["sources"]:
            self.assertTrue(item["not_transferred"])
            self.assertTrue(item["remaining"])
            self.assertTrue((ROOT / item["evidence"]).is_file(), item["evidence"])
            for pattern in item["patterns"]:
                for name in pattern["code"] + pattern["tests"]:
                    self.assertTrue((ROOT / name).is_file(), name)
        single = self.cli("references", "--source", "arx5")
        self.assertEqual(single["source"]["id"], "arx5")
        self.assertFalse(single["hardware_accessed"])
        self.cli("references", "--source", "not-a-source", expected=2)
        drawer = self.cli("references", "--source", "arx5", "--task", "drawer-push-pull")
        self.assertEqual(drawer["experiment"]["task_id"], "drawer-push-pull")
        self.assertIsNone(drawer["experiment"]["physical_success_in_this_repository"])
        self.assertTrue(drawer["experiment"]["reported_variants"])
        self.cli("references", "--source", "zetta", "--task", "pen", expected=2)

    def test_recovery_cli_never_turns_a_timeout_or_exhausted_state_into_retry(self):
        fixture = json.loads((ROOT / "research/recovery_example_round2.json").read_text())
        initial = self.cli("recovery-plan", self.write("recovery.json", fixture))
        self.assertEqual(initial["status"], "proposed")
        self.assertFalse(initial["phase_transition_applied"])
        self.assertFalse(initial["execution_available"])
        self.assertFalse(initial["budget_consumed"])
        timeout = copy.deepcopy(fixture)
        timeout["execution_result"]["category"] = "timeout"
        blocked = self.cli("recovery-plan", self.write("timeout.json", timeout))
        self.assertEqual(blocked["status"], "blocked")
        self.assertEqual(blocked["candidate_operations"], [])
        self.assertFalse(blocked["automatic_retry"])
        exhausted = copy.deepcopy(fixture)
        exhausted["remaining_budget"]["recoveries"] = 0
        blocked = self.cli("recovery-plan", self.write("exhausted.json", exhausted))
        self.assertIn("budget_exhausted", blocked["blocking_reasons"])
        invalid = dict(fixture, command="move_to_historical_pose")
        self.cli("recovery-plan", self.write("extra-command.json", invalid), expected=2)

    def test_recording_plan_is_declarative_and_closed_report_alone_is_not_coverage(self):
        plan = self.cli("recording-plan", "--run-id", "future-1", "--task", "can_on_lid", "--include-left")
        self.assertFalse(plan["recorder_started"])
        self.assertFalse(plan["execution_available"])
        self.assertEqual(set(plan["requested_local_views"]), {"right_hand", "front", "left_hand"})
        self.assertFalse(plan["model_view_selection_affects_recording"])
        self.assertTrue(plan["exclude_initial_homing"])
        document = dict(schema_version="recording_audit_v1", contract=plan, events=[], frames=None,
            report=dict(run_id="future-1", task_id="can_on_lid", status="closed", cleanup_errors=[]))
        report = self.cli("recording-audit", self.write("recording.json", document))
        self.assertTrue(report["cleanup_closed"])
        self.assertFalse(report["metadata_coverage_complete"])
        self.assertFalse(report["video_decode_verified"])
        self.assertIsNone(report["task_success"])
        self.assertFalse((self.temp / "recordings").exists())
        self.cli("recording-plan", "--run-id", "../outside", "--task", "can_on_lid", expected=2)

    def test_multi_object_and_faucet_examples_require_their_actual_goal_evidence(self):
        sort = self.cli("plan", "--recipe", ROOT / "tasks/sort_two_objects.json")
        self.assertEqual(len(sort["steps"]), 18)
        self.assertTrue({"a_goal_visible", "b_goal_visible"} <= set(sort["steps"][-1]["evidence"]))
        faucet = self.cli("plan", "--recipe", ROOT / "tasks/turn_faucet.json")
        turn = next(step for step in faucet["steps"] if step["skill"] == "articulated_rotate")
        self.assertIn("visual_goal_reached", turn["evidence"])
        self.assertNotIn("thread_progress", turn["evidence"])
        can = self.cli("plan", "--recipe", ROOT / "tasks/can_on_lid.json")
        support = next(step for step in can["steps"] if step["skill"] == "place_on_support")
        self.assertIn("object_on_target_support", support["evidence"])
        self.assertNotIn("object_on_original_support", support["evidence"])

    def test_historical_receipts_keep_declared_counts_separate_from_actual_zero_calls(self):
        generated = self.cli("replay", "--task", "pen")
        history = [copy.deepcopy(item["receipt"]) for item in generated["events"]]
        for event in history:
            event["model_called"] = True
        output = self.cli("replay", "--task", "pen", "--receipts", self.write("history.json", history))
        self.assertEqual(output["input_kind"], "historical_receipts_unverified")
        self.assertEqual(output["summary"]["model_calls"], len(history))
        self.assertEqual(output["actual_model_call_count"], 0)
        self.assertEqual(output["dispatched_action_count"], 0)
        self.assertIsNone(output["task_success"])

    def test_historical_fault_terminates_and_extra_or_duplicate_events_are_rejected(self):
        generated = self.cli("replay", "--task", "pen")
        fault = dict(generated["events"][0]["receipt"], status="fault", outcome_uncertain=True)
        report = self.cli("replay", "--task", "pen", "--receipts", self.write("fault.json", [fault]))
        self.assertEqual(report["summary"]["termination_reason"], "execution_failed_latched")
        self.assertIsNone(report["task_success"])
        error = self.cli("replay", "--task", "pen", "--receipts",
            self.write("after-fault.json", [fault, generated["events"][1]["receipt"]]), expected=2)
        self.assertFalse(error["execution_available"])
        self.assertIn("termination", error["message"])
        duplicate = [generated["events"][0]["receipt"]] * 2
        self.cli("replay", "--task", "pen", "--receipts", self.write("duplicate.json", duplicate), expected=2)

    def test_output_creation_never_overwrites_existing_file_or_symlink_target(self):
        target = self.temp / "reports/replay.json"
        first = self.cli("replay", "--task", "pen", "--out", target)
        self.assertEqual(first["report"], str(target))
        before = target.read_bytes()
        self.cli("replay", "--task", "pen", "--out", target, expected=2)
        self.assertEqual(target.read_bytes(), before)
        alias = self.temp / "alias.json"
        alias.symlink_to(target)
        self.cli("replay", "--task", "pen", "--out", alias, expected=2)
        self.assertEqual(target.read_bytes(), before)

    def test_unified_session_reopens_with_progress_and_frozen_recipe(self):
        source = json.loads((ROOT / "tasks/pen_in_holder.json").read_text())
        recipe = self.write("recipe.json", source)
        store = self.temp / "session.sqlite"
        initial = self.cli("session", "--store", store, "init", "--run-id", "persist",
                           "--recipe", recipe, "--worker-arm", "left")
        current = initial["next"]
        event = dict(run_id="persist", task=current["task"], stage=current["stage"], arm="left",
            status="complete", observation_id="host-frame", at=10., model_called=False,
            evidence=current["expect"], prerequisites=current["needs"])
        record_args = ("session", "--store", store, "record", "--run-id", "persist",
                       "--revision", "0", "--event-id", "first", "--receipt", self.write("receipt.json", event))
        completed = self.cli(*record_args)
        same = self.cli(*record_args)
        self.assertTrue(same["duplicate_event"])
        self.assertEqual(same["revision"], 1)
        source["goal"] = "A changed goal cannot reset the prior run"
        self.write("recipe.json", source)
        self.cli("session", "--store", store, "init", "--run-id", "persist",
                 "--recipe", recipe, "--worker-arm", "left", expected=2)
        recipe.unlink()
        reopened = self.cli("session", "--store", store, "current", "--run-id", "persist", "--operation")
        self.assertEqual(reopened["revision"], 1)
        self.assertEqual(reopened["next"], completed["next"])
        self.assertEqual(reopened["contract_sha256"], initial["contract_sha256"])
        self.assertGreaterEqual(reopened["elapsed_s"], completed["elapsed_s"])
        self.assertLessEqual(reopened["seconds_left"], completed["seconds_left"])
        self.assertFalse(reopened["execution_available"])

    def test_evaluate_labels_declarations_and_never_upgrades_model_claims(self):
        document = evaluation_document()
        report = self.cli("evaluate", self.write("model.json", document))
        self.assertEqual(report["scope"], "declared_evidence_structure_review")
        self.assertFalse(report["authenticated_visual_truth"])
        self.assertTrue(report["result"]["model_reported_success"])
        self.assertIsNone(report["result"]["task_success"])
        self.assertFalse(report["result"]["phase_transition_applied"])
        independent = evaluation_document("independent_rgb_review")
        declared = self.cli("evaluate", self.write("independent.json", independent))
        self.assertTrue(declared["result"]["task_success"])
        self.assertFalse(declared["authenticated_visual_truth"])
        independent["events"].append(dict(id="later-motion", run_id="reviewed-run", at=4,
            source="robot_receipt", reference="host:later-motion", outcome="completed"))
        self.assertIsNone(self.cli("evaluate", self.write("stale.json", independent))["result"]["task_success"])

    def test_evaluate_rejects_cross_run_and_does_not_treat_simulation_as_physical(self):
        document = evaluation_document("independent_rgb_review")
        document["events"][0]["run_id"] = "different-run"
        self.cli("evaluate", self.write("cross-run.json", document), expected=2)
        document = evaluation_document("independent_rgb_review")
        for observation in document["observations"]:
            observation["origin"] = "simulation_rgb"
        self.assertIsNone(self.cli("evaluate", self.write("simulation.json", document))["result"]["task_success"])


if __name__ == "__main__":
    unittest.main()
