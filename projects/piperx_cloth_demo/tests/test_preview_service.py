"""No devices: contract, failure isolation, vendor FK, and non-execution regressions."""
import copy
import json
import math
import shutil
import socket
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_tools.preview import preview_plan, render_intent_svg
from robot_tools.service import TOOL_SCHEMAS, ToolService

ROOT = Path(__file__).resolve().parents[1]


def plan():
    # SYNTHETIC TEST DATA. Never used as a garment observation or physical plan.
    targets = [{"arm": arm, "frame": arm + "_base", "reference": "sdk_flange",
                "pose_m_rad": [0.2, 0.0, 0.3, 0.0, 0.0, 0.0], "motion": "move_p",
                "speed_percent": 5, "uncertainty": "synthetic fixture; not physical"}
               for arm in ("left", "right")]
    return {"schema_version": 1, "task": "SYNTHETIC fixture only",
            "observation_id": "obs_" + "0" * 32,
            "stages": [{"id": "fixture", "intent": "contract test", "expected_feedback": "none",
                        "coordination": "paired", "targets": targets}]}


class PreviewServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        shutil.copytree(ROOT / "configs", self.root / "configs")
        shutil.copytree(ROOT / "tasks", self.root / "tasks")
        shutil.copytree(ROOT / "docs", self.root / "docs")
        shutil.copytree(ROOT / "prompts", self.root / "prompts")
        self.service = ToolService(self.root)
        self.profile = self.service.profile
        self.p = plan()

    def tearDown(self):
        self.temp.cleanup()

    def test_supervised_actions_lock_and_persist_exact_model_request(self):
        from robot_tools.execution import ExclusiveExecution
        pose = [0.1, 0.02, 0.3, 0.0, 0.5, 0.0]
        cases = [("robot_move_once", "move_once", {"arm": "right", "target_pose_m_rad": pose}),
                 ("robot_gripper_once", "gripper_once", {"arm": "left", "width_m": 0.01, "nominal_force_N": 0.2})]
        for public, function, args in cases:
            with self.subTest(tool=public):
                existing_requests = set((self.root / "runs").glob("supervised_*/request.json"))
                def operation(profile, journal, **actual):
                    self.assertEqual(actual, args)
                    self.assertEqual(profile["arms"], self.profile["arms"])
                    requests = set((self.root / "runs").glob("supervised_*/request.json")) - existing_requests
                    self.assertEqual(len(requests), 1)
                    request = json.loads(requests.pop().read_text())
                    self.assertEqual(request["arguments"], args)
                    with self.assertRaises(RuntimeError):
                        with ExclusiveExecution(self.root / "runs"):
                            pass
                    journal("synthetic_observation", {"scope": "mock only; no devices"})
                    return {"ok": False, "hardware_commands_sent": 0, "observed_stable": False}
                with patch("robot_tools.supervised_actions." + function, side_effect=operation) as command:
                    result = self.service.call(public, args)
                    command.assert_called_once()
                self.assertEqual(json.loads(Path(result["record_path"]).read_text()), result)
                self.assertFalse(result["trajectory_execution_available"])

    def test_supervised_bad_force_or_width_rejected_before_device_adapter(self):
        for field, value in (("width_m", 0.056), ("width_m", -0.001), ("nominal_force_N", 1), ("nominal_force_N", True)):
            args = {"arm": "right", "width_m": 0.02, "nominal_force_N": 0.2, field: value}
            with patch("robot_tools.supervised_actions.gripper_once") as command:
                with self.assertRaises(ValueError):
                    self.service.call("robot_gripper_once", args)
                command.assert_not_called()

    def test_no_observation_is_preview_not_execution(self):
        result = preview_plan(self.p, self.profile, None)
        self.assertTrue(result["structure_valid"])
        self.assertFalse(result["executable"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("blocked", [c["status"] for c in result["checks"]])

    def test_stale_incomplete_future_observations_block(self):
        for age, complete in ((31, True), (0, False), (-1, True)):
            with self.subTest(age=age, complete=complete):
                obs = {"id": self.p["observation_id"], "complete": complete,
                       "capture_started_unix_s": 1000 - age}
                result = preview_plan(self.p, self.profile, obs, now=1000)
                self.assertEqual(result["checks"][1]["status"], "blocked")

    def test_recent_observation_does_not_certify_ik(self):
        obs = {"id": self.p["observation_id"], "complete": True, "capture_started_unix_s": 1000}
        result = preview_plan(self.p, self.profile, obs, now=1001)
        self.assertEqual(result["checks"][1]["status"], "passed")
        self.assertFalse(result["executable"])
        self.assertEqual(next(c for c in result["checks"] if c["name"] == "candidate_inverse_kinematics")["status"], "unavailable")

    def test_invalid_targets_rejected_without_silent_clipping(self):
        for field, value in (("frame", "right_base"), ("pose_m_rad", [0, 0, 1, 0, 1.58, 0]),
                             ("pose_m_rad", [0, 0, math.nan, 0, 0, 0]), ("speed_percent", True),
                             ("reference", "fingertip"), ("speed_percent", 6), ("motion", "eval")):
            with self.subTest(field=field, value=value):
                p = copy.deepcopy(self.p)
                p["stages"][0]["targets"][0][field] = value
                with self.assertRaises(ValueError):
                    preview_plan(p, self.profile, None)

    def test_duplicate_arms_or_missing_pair_rejected(self):
        for targets in ([self.p["stages"][0]["targets"][0]] * 2,
                        [self.p["stages"][0]["targets"][0]]):
            p = copy.deepcopy(self.p)
            p["stages"][0]["targets"] = targets
            with self.assertRaises(ValueError):
                preview_plan(p, self.profile, None)

    def test_unknown_code_properties_rejected(self):
        for args in ({"plan": self.p, "code": "ignored?"}, {"plan": {**self.p, "script": "anything"}}):
            with self.assertRaises(ValueError):
                self.service.call("robot_preview_plan", args)
        with self.assertRaises(ValueError):
            self.service.call("robot_move", {})

    def test_plan_saved_unchanged_and_submission_still_blocked(self):
        with patch.object(socket, "socket", side_effect=AssertionError("device access forbidden")):
            result = self.service.call("robot_preview_plan", {"plan": self.p})
            self.assertEqual(json.loads(Path(result["plan_path"]).read_text()), self.p)
            # Config changes cannot activate an absent motion dispatcher.
            self.profile["execution_implemented"] = True
            self.profile["verification"] = {k: True for k in self.profile["verification"]}
            submission = self.service.call("robot_submit_plan", {"preview_id": result["preview_id"]})
        self.assertEqual(submission["status"], "blocked")
        self.assertFalse(submission["ok"])
        self.assertFalse(submission["executable"])

    def test_modified_plan_cannot_reuse_preview(self):
        result = self.service.preview(self.p)
        Path(result["plan_path"]).write_text(json.dumps({**self.p, "task": "changed"}))
        with self.assertRaisesRegex(ValueError, "changed"):
            self.service.submit(result["preview_id"])

    def test_run_id_path_escape_rejected(self):
        with self.assertRaises(ValueError):
            self.service.call("robot_submit_plan", {"preview_id": "../../other"})

    def test_observe_camera_failure_still_reads_arms(self):
        fake = {"status": "complete", "arms": {"left": {}, "right": {}}}
        with patch("robot_tools.cameras.capture_cameras", side_effect=RuntimeError("camera unavailable")), \
             patch.object(self.service, "_read_arms", return_value=fake) as read:
            result = self.service.call("robot_observe", {"include_depth": False})
        read.assert_called_once()
        self.assertFalse(result["complete"])
        self.assertEqual(result["state"], fake)
        self.assertFalse(result["synchronized"])
        self.assertTrue(Path(result["record_path"]).is_file())

    def test_observe_preserves_all_views_and_partial_state(self):
        camera = {"complete": True, "cameras": {n: {"rgb_path": str(self.root / (n + ".png"))}
                  for n in ("front", "left_wrist", "right_wrist")}}
        with patch("robot_tools.cameras.capture_cameras", return_value=camera), \
             patch.object(self.service, "_read_arms", return_value={"status": "partial"}):
            result = self.service.observe()
        self.assertEqual(len(result["image_paths"]), 3)
        self.assertFalse(result["ok"])

    def test_vendor_fk_never_opens_socket(self):
        with patch.object(socket, "socket", side_effect=AssertionError("no hardware")):
            result = self.service.call("robot_fk", {"model": "piper", "joints_rad": [0] * 6})
        self.assertAlmostEqual(result["pose_m_rad"][0], 0.0561275, places=6)
        self.assertFalse(result["physical_model_verified"])

    def test_intent_chart_never_implies_common_world(self):
        svg = render_intent_svg(self.p)
        self.assertIn("left_base", svg)
        self.assertIn("right_base", svg)
        self.assertIn("no IK", svg)
        self.assertIn("not a geometric path", svg)

    def test_tools_have_explicit_strict_schemas(self):
        self.assertEqual(len(TOOL_SCHEMAS), 38)
        names = {tool["name"] for tool in TOOL_SCHEMAS}
        self.assertEqual(len(names), len(TOOL_SCHEMAS))
        self.assertTrue({"robot_single_arm_move_once", "robot_single_arm_gripper_once"} <= names)
        for tool in TOOL_SCHEMAS:
            self.assertFalse(tool["inputSchema"]["additionalProperties"])
            self.assertIn("description", tool)

    def test_mode_takeover_records_request_and_keeps_execution_blocked(self):
        def fake_takeover(profile, emit):
            self.assertEqual(profile["arms"], self.profile["arms"])
            emit("test_no_hardware", {"hardware_commands_sent": 0})
            return {"ok": True, "status": "already_can_control", "hardware_commands_sent": 0}
        with patch("robot_tools.takeover.request_can_control", side_effect=fake_takeover), \
             patch.object(socket, "socket", side_effect=AssertionError("No real device access")):
            result = self.service.call("robot_request_can_control", {})
            self.assertFalse(self.service.check_execution()["physical_execution_available"])
        directory = Path(result["record_path"]).parent
        self.assertTrue((directory / "request.json").is_file())
        self.assertTrue((directory / "events.jsonl").is_file())
        self.assertFalse(result["trajectory_execution_available"])
        self.assertEqual(json.loads(Path(result["record_path"]).read_text()), result)

    def test_mode_takeover_cannot_bypass_execution_lock(self):
        from robot_tools.execution import ExclusiveExecution, ExecutionFault
        with ExclusiveExecution(self.service.runs), \
             patch("robot_tools.takeover.request_can_control") as call:
            with self.assertRaises(ExecutionFault):
                self.service.call("robot_request_can_control", {})
            call.assert_not_called()

    def test_describe_embeds_policy_without_a_file_reader(self):
        with patch.object(socket, "socket", side_effect=AssertionError("no hardware")):
            result = self.service.call("robot_describe", {})
        self.assertFalse(result["execution_available"])
        self.assertEqual(set(result["reference_texts"]), {"robot_guide", "sdk_audit", "experiment_policy"})
        self.assertIn("短袖 T 恤", result["reference_texts"]["experiment_policy"])

    def test_startup_records_request_and_keeps_execution_blocked(self):
        def fake_startup(profile, emit):
            self.assertEqual(profile["arms"], self.profile["arms"])
            emit("test_no_hardware", {"hardware_commands_sent": 0})
            return {"ok": True, "status": "synthetic_startup", "hardware_commands_sent": 0}
        with patch("robot_tools.takeover.startup_arms", side_effect=fake_startup), \
             patch.object(socket, "socket", side_effect=AssertionError("No real device access")):
            result = self.service.call("robot_startup_arms", {})
            self.assertFalse(self.service.check_execution()["physical_execution_available"])
        directory = Path(result["record_path"]).parent
        self.assertTrue((directory / "request.json").is_file())
        self.assertTrue((directory / "events.jsonl").is_file())
        self.assertFalse(result["trajectory_execution_available"])
        self.assertEqual(json.loads(Path(result["record_path"]).read_text()), result)

    def test_startup_cannot_bypass_execution_lock(self):
        from robot_tools.execution import ExclusiveExecution, ExecutionFault
        with ExclusiveExecution(self.service.runs), \
             patch("robot_tools.takeover.startup_arms") as call:
            with self.assertRaises(ExecutionFault):
                self.service.call("robot_startup_arms", {})
            call.assert_not_called()

    def test_single_startup_rejects_missing_arm_or_target_before_adapter(self):
        for arguments in ({}, {"arm": "both"}, {"arm": True},
                          {"arm": "right", "target_joints_rad": [0.0] * 6},
                          {"arm": "right", "retry": True}):
            with self.subTest(arguments=arguments), \
                 patch("robot_tools.single_arm_startup.startup_arm") as operation:
                with self.assertRaises(ValueError):
                    self.service.call("robot_startup_arm", arguments)
                operation.assert_not_called()

    def test_firmware_and_gripper_preparation_are_locked_and_recorded(self):
        from robot_tools.execution import ExclusiveExecution, ExecutionFault
        cases = (("robot_inspect_firmware", "robot_tools.commissioning.inspect_firmware", {}),
                 ("robot_startup_arm", "robot_tools.single_arm_startup.startup_arm", {"arm": "right"}),
                 ("robot_prepare_gripper", "robot_tools.single_gripper_prepare.prepare_gripper", {"arm": "right"}),
                 ("robot_home_arm", "robot_tools.home_arm.home_arm", {"arm": "right"}),
                 ("robot_prepare_grippers", "robot_tools.gripper_prepare.prepare_grippers", {}),
                 ("robot_inspect_joint_limits", "robot_tools.joint_limits.inspect_joint_limits", {}),
                 ("robot_recover_joint_boundary", "robot_tools.joint_recovery.recover_joint_boundary", {"arm": "right", "target_joints_rad": [0.] * 6}),
                 ("robot_recover_joint_boundary", "robot_tools.joint_recovery.recover_joint_boundary", {"arm": "left", "target_joints_rad": [0.] * 6, "recovery_profile": "startup_j2_j3", "attachment_radius_m": 0.3, "available_clearance_m": 0.4}),
                 ("robot_bounded_joint_step", "robot_tools.bounded_joint_step.bounded_joint_step", {"arm": "right", "target_joints_rad": [0.] * 6, "attachment_radius_m": 0.3, "available_clearance_m": 0.05}),
                 ("robot_qualify_linear_hold", "robot_tools.linear_hold.qualify_linear_hold", {"arm": "right"}))
        for tool, target, arguments in cases:
            with self.subTest(tool=tool):
                with ExclusiveExecution(self.service.runs), patch(target) as operation:
                    with self.assertRaises(ExecutionFault):
                        self.service.call(tool, arguments)
                    operation.assert_not_called()
                def fake_operation(profile, emit, **kwargs):
                    self.assertEqual(kwargs, arguments)
                    emit("test_no_hardware", {"hardware_commands_sent": 0})
                    return {"ok": True, "hardware_commands_sent": 0}
                with patch(target, side_effect=fake_operation), \
                     patch.object(socket, "socket", side_effect=AssertionError("No real device access")):
                    result = self.service.call(tool, arguments)
                    self.assertFalse(self.service.check_execution()["physical_execution_available"])
                directory = Path(result["record_path"]).parent
                self.assertTrue((directory / "request.json").is_file())
                self.assertEqual(json.loads((directory / "request.json").read_text())["arguments"], arguments)
                self.assertTrue((directory / "events.jsonl").is_file())
                self.assertFalse(result["trajectory_execution_available"])
                self.assertEqual(json.loads(Path(result["record_path"]).read_text()), result)

    def test_single_gripper_preparation_rejects_target_and_non_arm_arguments(self):
        for arguments in ({}, {"arm": "both"}, {"arm": True},
                          {"arm": "right", "width_m": 0.0028},
                          {"arm": "right", "force": 0.2},
                          {"arm": "right", "retry": True},
                          {"arm": "right", "set_zero": True}):
            with self.subTest(arguments=arguments), \
                 patch("robot_tools.single_gripper_prepare.prepare_gripper") as operation:
                with self.assertRaises(ValueError):
                    self.service.call("robot_prepare_gripper", arguments)
                operation.assert_not_called()

    def test_home_arm_rejects_target_speed_zero_calibration_and_replay_arguments(self):
        for arguments in ({}, {"arm": "both"}, {"arm": True},
                          {"arm": "right", "target_joints_rad": [0.0] * 6},
                          {"arm": "right", "speed_percent": 1},
                          {"arm": "right", "retry": True},
                          {"arm": "right", "set_zero": True}):
            with self.subTest(arguments=arguments), \
                 patch("robot_tools.home_arm.home_arm") as operation:
                with self.assertRaises(ValueError):
                    self.service.call("robot_home_arm", arguments)
                operation.assert_not_called()

    def test_mode_only_receipt_is_loaded_and_durably_claimed_once_before_call(self):
        run_id = "linear_hold_" + "e" * 32
        directory = self.service.runs / run_id
        directory.mkdir(parents=True)
        (directory / "request.json").write_text(json.dumps({"arms": self.service.profile["arms"]}))
        (directory / "result.json").write_text(json.dumps({"arm": "right"}))
        event = {"event": "probe_send_intent", "kind": "mode", "frames": []}
        (directory / "events.jsonl").write_text(json.dumps(event) + "\n")
        def fake_operation(profile, emit, arm, prior_mode_only_record):
            self.assertTrue((directory / "target_replacement_claim.json").is_file())
            self.assertEqual(arm, "right")
            self.assertEqual(prior_mode_only_record["source_run_id"], run_id)
            self.assertEqual(prior_mode_only_record["events"], [event])
            return {"ok": False, "hardware_commands_sent": 0}
        with patch("robot_tools.linear_hold.qualify_linear_hold", side_effect=fake_operation) as operation, \
             patch.object(socket, "socket", side_effect=AssertionError("No real device access")):
            args = {"arm": "right", "prior_mode_only_run_id": run_id}
            result = self.service.call("robot_qualify_linear_hold", args)
            self.assertFalse(result["ok"])
            with self.assertRaises(FileExistsError):
                self.service.call("robot_qualify_linear_hold", args)
            self.assertEqual(operation.call_count, 1)

    def test_mode_only_receipt_cannot_cross_arm_or_device_binding(self):
        run_id = "linear_hold_" + "f" * 32
        directory = self.service.runs / run_id
        directory.mkdir(parents=True)
        (directory / "request.json").write_text(json.dumps({"arms": {}}))
        (directory / "result.json").write_text(json.dumps({"arm": "left"}))
        with patch("robot_tools.linear_hold.qualify_linear_hold") as operation:
            with self.assertRaisesRegex(ValueError, "bindings"):
                self.service.call("robot_qualify_linear_hold", {"arm": "right", "prior_mode_only_run_id": run_id})
            operation.assert_not_called()
        self.assertFalse((directory / "target_replacement_claim.json").exists())

    def test_gripper_actions_are_explicit_and_separate(self):
        p = copy.deepcopy(self.p)
        p["stages"][0]["targets"][0]["gripper_width_m"] = 0.02
        with self.assertRaisesRegex(ValueError, "separate"):
            self.service.preview(p)
        for t in p["stages"][0]["targets"]:
            t.update(action="gripper", gripper_width_m=0.02, gripper_force_N=0.5)
        self.assertTrue(self.service.preview(p)["structure_valid"])
        del p["stages"][0]["targets"][1]["gripper_force_N"]
        with self.assertRaisesRegex(ValueError, "explicit force"):
            self.service.preview(p)

    def test_backend_commissioning_is_not_a_config_switch(self):
        self.profile["verification"] = {k: True for k in self.profile["verification"]}
        record = self.root / "resolved.json"
        record.write_text(json.dumps({"status": "resolved_with_verified_stop"}))
        self.profile["legacy_hold_record"] = str(record)
        with patch.object(socket, "socket", side_effect=AssertionError("No hardware before commissioning")):
            result = self.service.check_execution()
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(result["reasons"])

    def _recent_observation(self):
        directory = self.root / "runs" / self.p["observation_id"]
        directory.mkdir(parents=True, exist_ok=True)
        record = {"id": self.p["observation_id"], "complete": True,
                  "capture_started_unix_s": time.time(), "state": {"status": "complete"}}
        (directory / "observation.json").write_text(json.dumps(record))
        return directory, record

    def test_submission_is_one_shot_even_after_process_restart(self):
        self._recent_observation()
        preview = self.service.preview(self.p)
        with patch("robot_tools.service.readiness", return_value=[]), \
             patch("robot_tools.service.run_plan", return_value={"ok": True, "status": "targets_reached"}) as run:
            first = self.service.submit(preview["preview_id"])
            other = ToolService(self.root)
            second = other.submit(preview["preview_id"])
        self.assertEqual(first, second)
        run.assert_called_once()

    def test_interrupted_claim_cannot_automatically_resend(self):
        self._recent_observation()
        preview = self.service.preview(self.p)
        with patch("robot_tools.service.readiness", return_value=[]), \
             patch("robot_tools.service.run_plan", side_effect=RuntimeError("worker lost")):
            with self.assertRaises(RuntimeError):
                self.service.submit(preview["preview_id"])
        result = self.service.submit(preview["preview_id"])
        self.assertEqual(result["status"], "in_flight_or_interrupted")
        self.assertFalse(result["automatic_resume"])
        cancelled = self.service.cancel_execution(preview_id=preview["preview_id"])
        self.assertFalse(cancelled["physical_stop_verified"])
        self.assertTrue(self.service.execution_status(result["execution_id"])["cancel_requested"])

    def test_cancellation_before_claim_prevents_later_submit(self):
        self._recent_observation()
        preview = self.service.preview(self.p)
        cancelled = self.service.cancel_execution(preview_id=preview["preview_id"])
        self.assertFalse(cancelled["physical_stop_verified"])
        with patch("robot_tools.service.run_plan") as run:
            result = self.service.submit(preview["preview_id"])
        self.assertEqual(result["status"], "cancelled_before_start")
        self.assertEqual(result["hardware_commands_sent"], 0)
        run.assert_not_called()

    def test_depth_probe_preserves_invalid_and_has_no_world_transform(self):
        import numpy as np
        directory, record = self._recent_observation()
        path = directory / "front.npy"
        data = np.full((480, 640), np.nan, np.float32)
        data[100, 200] = 0.4
        data[100, 201] = 65.535
        np.save(path, data, allow_pickle=False)
        record["cameras"] = {"cameras": {"front": {"depth_path": str(path), "depth_unit": "metre",
                                  "depth_aligned_to": "color", "depth_scale_m": 0.001, "host_receive_unix_s": time.time()}}}
        (directory / "observation.json").write_text(json.dumps(record))
        result = self.service.call("robot_depth_at_pixels", {"observation_id": self.p["observation_id"],
                                  "camera": "front", "pixels_uv": [[200,100],[0,0],[201,100]]})
        self.assertAlmostEqual(result["points"][0]["depth_m"], 0.4)
        self.assertIsNone(result["points"][1]["depth_m"])
        self.assertIsNone(result["points"][2]["depth_m"])
        self.assertEqual(result["points"][2]["quality"], "at_encoding_limit")
        self.assertIn("not robot", result["coordinate_frame"])
        with self.assertRaisesRegex(ValueError, "outside"):
            self.service.depth_at_pixels(self.p["observation_id"], "front", [[0, 480]])


if __name__ == "__main__":
    unittest.main()
