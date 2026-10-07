"""Only synthetic metadata and temporary files; camera/process access forbidden."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from right_pick.fast_pipeline import PipelineContractError
from right_pick.fast_recording_contract import (make_recording_contract, audit_recording,
                                                audit_recording_files, reserve_recording_directory)


class RecordingContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "bundle"
        self.root.mkdir()
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No hardware or worker in metadata audit"))
            blocker.start()
            self.addCleanup(blocker.stop)
        self.plan = make_recording_contract("run-1", "pen")
        self.events = [self.event(kind, at) for kind, at in (
            ("initial_pose_ready", 10), ("task_start", 10.2), ("action", 10.4),
            ("release_complete", 10.7), ("retreat_complete", 11.1), ("stability_verified", 11.5),
            ("controller_exit", 11.6))]
        self.document = {"schema_version": "recording_audit_v1", "contract": self.plan,
                         "events": self.events, "report": {}, "frames": []}
        self.record_frames(10.15, 11.65)

    def event(self, kind, at, **extra):
        return dict(id=kind + "-" + str(at), run_id="run-1", task_id="pen", kind=kind,
                    at=at, reference="synthetic host event", **extra)

    def record_frames(self, first, last, views=("right_hand", "front")):
        times, stamp = [], first
        while stamp <= last + 0.00001:
            times.append(round(stamp, 4))
            stamp += 0.1
        self.document["frames"] = [{"name": view, "host_received_at": at,
                                    "timestamp_kind": "host_receipt_wall_clock",
                                    "video_frame_index": index,
                                    "video_path": "/never/open/this/video.avi"}
                                   for index, at in enumerate(times) for view in views]
        self.document["report"] = {"run_id": "run-1", "task_id": "pen", "started_at": first - 0.01,
                                   "finished_at": last + 0.01, "status": "closed", "cleanup_errors": [],
                                   "failure": None, "requested_cameras": {v: {"fps": 15} for v in views},
                                   "frames_recorded": {v: len(times) for v in views}}

    def test_plan_declares_views_without_claiming_devices_or_scene(self):
        self.assertEqual(self.plan["required_views"], ["right_hand", "front"])
        self.assertEqual(self.plan["optional_views"], ["left_hand"])
        self.assertFalse(self.plan["model_view_selection_affects_recording"])
        self.assertTrue(self.plan["exclude_initial_homing"])
        self.assertFalse(self.plan["recorder_started"])
        self.assertFalse(self.plan["execution_available"])
        self.assertIn("all three", self.plan["existing_worker_constraint"])
        self.assertNotIn("scene_current", self.plan)

    def test_relative_directory_reservation_is_exclusive_and_no_start(self):
        result = reserve_recording_directory(self.plan, self.root)
        self.assertTrue(result["reserved"])
        self.assertFalse(result["recorder_started"])
        with self.assertRaises(FileExistsError):
            reserve_recording_directory(self.plan, self.root)
        self.assertTrue((self.root / self.plan["output_relative"] / "contract.json").is_file())

    def test_invalid_paths_and_symlink_escape_are_rejected(self):
        for value in ("../outside", "/absolute", "recordings/../other", ".", "x//y"):
            with self.subTest(path=value), self.assertRaises(PipelineContractError):
                make_recording_contract("run", "pen", output_relative=value)
        (self.root / "recordings").symlink_to(self.root.parent, target_is_directory=True)
        with self.assertRaises(PipelineContractError):
            reserve_recording_directory(self.plan, self.root)

    def test_complete_metadata_is_not_task_success_or_video_validation(self):
        result = audit_recording(self.document)
        self.assertTrue(result["metadata_coverage_complete"], result)
        self.assertEqual(result["required_interval"]["end_at"], 11.5)
        self.assertIsNone(result["task_success"])
        self.assertIsNone(result["physical_outcome_resolved"])
        self.assertFalse(result["video_decode_verified"])
        self.assertFalse(result["video_files_checked"])
        self.assertFalse(result["independent_recorder_supervisor_verified"])

    def test_cleanup_closed_cannot_hide_early_recording_end(self):
        self.record_frames(10.15, 10.95)
        result = audit_recording(self.document)
        self.assertTrue(result["cleanup_closed"])
        self.assertFalse(result["metadata_coverage_complete"])
        self.assertTrue(any("ended_before_required_terminal" in f for f in result["findings"]))

    def test_stability_and_retreat_are_both_required_for_normal_completion(self):
        for absent in ("retreat_complete", "stability_verified"):
            doc = deepcopy(self.document)
            doc["events"] = [e for e in doc["events"] if e["kind"] != absent]
            self.assertFalse(audit_recording(doc)["metadata_coverage_complete"])

    def test_fault_requires_later_settled_or_explicit_unresolved(self):
        self.events.append(self.event("fault", 11.7))
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])
        self.events.append(self.event("settled", 12))
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])
        self.record_frames(10.15, 12.15)
        result = audit_recording(self.document)
        self.assertTrue(result["metadata_coverage_complete"], result)
        self.assertEqual(result["required_interval"]["end_condition"], "settled")
        self.assertIsNone(result["physical_outcome_resolved"])

    def test_bare_settled_does_not_replace_normal_retreat_and_stability(self):
        self.events[:] = self.events[:3] + [self.event("settled", 11.3)]
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_stability_after_fault_is_not_exception_end_receipt(self):
        self.events[:] = self.events[:5] + [self.event("fault", 11.2),
                                          self.event("stability_verified", 11.5)]
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_same_timestamp_old_settled_cannot_cover_fault(self):
        self.events[:] = self.events[:3] + [self.event("settled", 11.3), self.event("fault", 11.3)]
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_same_timestamp_retreat_cannot_cover_new_action(self):
        self.events[:] = self.events[:3] + [self.event("retreat_complete", 11.1),
            self.event("action", 11.1), self.event("stability_verified", 11.5)]
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_exit_at_stability_time_needs_exception_endpoint(self):
        self.events[:] = self.events[:5] + [self.event("controller_exit", 11.5),
                                          self.event("stability_verified", 11.5)]
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_explicit_unresolved_documents_coverage_without_claiming_success(self):
        self.events[:] = self.events[:3] + [self.event("controller_exit", 10.6),
            self.event("unresolved", 11.3, reason="No independent settling observation available")]
        result = audit_recording(self.document)
        self.assertTrue(result["metadata_coverage_complete"], result)
        self.assertTrue(result["explicit_unresolved"])
        self.assertIsNone(result["task_success"])
        del self.events[-1]["reason"]
        with self.assertRaises(PipelineContractError):
            audit_recording(self.document)

    def test_controller_exit_alone_is_not_an_end_condition(self):
        self.events[:] = self.events[:3] + [self.event("controller_exit", 10.6)]
        result = audit_recording(self.document)
        self.assertFalse(result["metadata_coverage_complete"])
        self.assertIsNone(result["required_interval"]["end_at"])

    def test_later_action_invalidates_old_retreat_and_stability(self):
        self.events.append(self.event("action", 11.7))
        self.events.append(self.event("stability_verified", 12.1))
        self.record_frames(10.15, 12.15)
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_missing_binding_on_legacy_worker_report_remains_unknown(self):
        del self.document["report"]["run_id"]
        del self.document["report"]["task_id"]
        result = audit_recording(self.document)
        self.assertIsNone(result["metadata_coverage_complete"])
        self.assertIn("report_missing_run_task_binding", result["unknown_reasons"])

    def test_mismatched_task_report_or_events_cannot_be_relabelled(self):
        self.document["report"]["task_id"] = "charger"
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])
        self.events[-1]["run_id"] = "other"
        with self.assertRaises(PipelineContractError):
            audit_recording(self.document)

    def test_recording_during_homing_or_after_first_action_is_incomplete(self):
        self.record_frames(9.95, 11.65)
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])
        self.record_frames(10.35, 11.65)
        self.assertFalse(audit_recording(self.document)["metadata_coverage_complete"])

    def test_frame_time_gaps_and_counter_mismatch_are_detected(self):
        self.document["frames"] = [f for f in self.document["frames"] if not 10.4 < f["host_received_at"] < 10.9]
        result = audit_recording(self.document)
        self.assertFalse(result["metadata_coverage_complete"])
        self.assertIn("frame_time_gap_exceeds_contract", result["views"]["front"]["problems"])
        self.assertIn("noncontiguous_video_frame_index", result["views"]["front"]["problems"])

    def test_missing_timestamp_index_or_frame_metadata_is_unknown(self):
        for field in ("host_received_at", "timestamp_kind", "video_frame_index"):
            doc = deepcopy(self.document)
            del doc["frames"][0][field]
            self.assertIsNot(audit_recording(doc)["metadata_coverage_complete"], True)
        self.document["frames"] = None
        self.assertIsNot(audit_recording(self.document)["metadata_coverage_complete"], True)

    def test_left_is_optional_until_explicitly_requested(self):
        self.assertTrue(audit_recording(self.document)["metadata_coverage_complete"])
        self.document["contract"] = make_recording_contract("run-1", "pen", include_left=True)
        self.assertIsNot(audit_recording(self.document)["metadata_coverage_complete"], True)
        self.record_frames(10.15, 11.65, views=("right_hand", "front", "left_hand"))
        self.assertTrue(audit_recording(self.document)["metadata_coverage_complete"])

    def test_altered_contract_or_duplicate_events_are_rejected(self):
        self.plan["max_interframe_gap_s"] = 300
        with self.assertRaises(PipelineContractError):
            audit_recording(self.document)
        self.plan["max_interframe_gap_s"] = 0.25
        self.events.append(deepcopy(self.events[-1]))
        with self.assertRaises(PipelineContractError):
            audit_recording(self.document)

    def test_files_audit_reads_metadata_only_and_checks_declared_paths(self):
        directory = self.root / self.plan["output_relative"]
        directory.mkdir(parents=True)
        (directory / "report.json").write_text(json.dumps(self.document["report"]))
        (directory / "frames.jsonl").write_text("".join(json.dumps(f) + "\n" for f in self.document["frames"]))
        # No video file exists: the result must not claim it was inspected.
        result = audit_recording_files(self.plan, self.events, self.root,
                                       self.plan["paths"]["report"], self.plan["paths"]["frames"])
        self.assertTrue(result["metadata_coverage_complete"], result)
        self.assertFalse(result["video_files_checked"])
        with self.assertRaises(PipelineContractError):
            audit_recording_files(self.plan, self.events, self.root, "other.json", "other.jsonl")


if __name__ == "__main__":
    unittest.main()
