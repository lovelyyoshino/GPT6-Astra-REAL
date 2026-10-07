"""Live-check boundary tests use injected fakes and never import ROS."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from right_pick.fast_live_check import run_live_check
from right_pick.recording import Recorder


class SensorRobot:
    nonphysical = False

    def __init__(self):
        self.observations = 0
        self.prepared = []
        self.closed = False

    def observe(self):
        self.observations += 1
        return {"robot_state": {"pose_m_rad": [.2, 0, .3, 0, 1, 0],
                "joints_rad": [0] * 6, "sampled_at": float(self.observations),
                "enabled": True, "binding_verified": True, "moving": False,
                "arm_status": 0, "err_code": 0},
                "gripper_state": {"opening_m": .035, "effort": 0.0},
                "raw_telemetry": {"sequence": self.observations},
                "provenance": {"sequence": self.observations}, "nonphysical": False}

    def prepare_command(self, decision, state):
        self.prepared.append(copy.deepcopy(state))
        return {"nonphysical": False, "numeric_limits_verified": False}

    def execute(self, *args, **kwargs):
        raise AssertionError("live-check must not dispatch")

    def validate(self, *args, **kwargs):
        raise AssertionError("live-check cannot obtain execution permission")

    def close(self):
        self.closed = True


class SensorCameras:
    nonphysical = False

    def __init__(self, fail=False):
        self.closed = False
        self.fail = fail

    def capture(self):
        if self.fail:
            raise TimeoutError("test capture failed")
        return {"capture_id": "original-image-time", "cameras": {
            name: {"rgb_path": "/test/" + name + ".png", "timestamp": 1.0}
            for name in ("front", "left_hand", "right_hand")}}

    def close(self):
        self.closed = True


class DecisionBackend:
    source = "codex_cli"

    def __init__(self, recorder, decision, fail=False):
        self.recorder, self.decision, self.fail = recorder, decision, fail
        self.last_metrics = {}
        self.received = None

    def decide(self, state, observation):
        self.received = copy.deepcopy((state, observation))
        self.last_metrics = {"model_request_start": 1., "model_response_end": 1.02,
                             "agent_decide_s": .02, "image_encode_s": .001,
                             "input_tokens": 23, "output_tokens": 14,
                             "reasoning_output_tokens": None,
                             "selected_camera_views": ["front"], "reasoning_effort": "medium"}
        self.recorder.model_call(elapsed_s=.02)
        if self.fail:
            raise RuntimeError("test backend failed")
        return self.decision


class FastLiveCheckTests(unittest.TestCase):
    def run_check(self, root, decision, phase="INIT", camera_fail=False, model_fail=False):
        recorder = Recorder(root, {}, "fake transport boundary unit test")
        robot, cameras = SensorRobot(), SensorCameras(fail=camera_fail)
        model = DecisionBackend(recorder, decision, fail=model_fail)
        report = run_live_check(model=model, robot=robot, cameras=cameras,
                                recorder=recorder, phase=phase)
        rows = [json.loads(s) for s in (recorder.run_dir / "steps.jsonl").read_text().splitlines()]
        return report, rows[0], model, robot, cameras, recorder

    def test_no_motion_advance_never_commits_phase_or_success(self):
        decision = {"phase": "INIT", "action": "advance", "confidence": .9,
                    "arguments": {"next_phase": "APPROACH_PEN", "evidence": "phase_complete"}}
        with tempfile.TemporaryDirectory() as folder:
            report, row, model, robot, cameras, recorder = self.run_check(Path(folder), decision)
            self.assertTrue(report["live_check_passed"])
            self.assertEqual(report["model_call_count"], 1)
            self.assertEqual(report["control_commands_sent"], 0)
            self.assertEqual(report["final_phase"], "INIT")
            self.assertIsNone(report["task_success"])
            self.assertEqual(row["phase_transition"], {"from": "INIT", "to": "INIT",
                                                      "proposed_to": "APPROACH_PEN"})
            self.assertFalse(row["action_dispatched"])
            self.assertTrue(robot.closed and cameras.closed)
            self.assertEqual(robot.observations, 3)
            self.assertEqual(json.loads((recorder.run_dir / "report.json").read_text())["outcome"], "not_evaluated")

    def test_motion_preparation_uses_new_feedback_but_model_packet_stays_frozen(self):
        decision = {"phase": "APPROACH_PEN", "action": "move_eef", "confidence": .85,
                    "arguments": {"pose_m_rad": [.21, 0, .3, 0, 1, 0],
                                  "speed_percent": 3, "next_phase": None}}
        with tempfile.TemporaryDirectory() as folder:
            report, row, model, robot, cameras, recorder = self.run_check(Path(folder), decision, "APPROACH_PEN")
            self.assertTrue(report["live_check_passed"])
            self.assertEqual(len(robot.prepared), 1)
            self.assertEqual(robot.prepared[0]["raw_telemetry"]["sequence"], 3)
            self.assertEqual(robot.prepared[0]["provenance"]["sequence"], 3)
            packet, image = model.received
            self.assertNotIn("raw_telemetry", packet)
            self.assertNotIn("provenance", packet)
            self.assertEqual(packet["robot_state"]["sampled_at"], 2.)
            self.assertEqual(image["capture_id"], "original-image-time")
            self.assertFalse(row["proposal_assessment"]["model_visual_observation_refreshed"])
            self.assertFalse(row["proposal_assessment"]["physical_execution_allowed"])
            self.assertEqual(report["action_count"], 0)

    def test_schema_and_phase_errors_terminate_with_cleanup(self):
        for decision in ({"invalid": True},
                         {"phase": "ALIGN_PEN", "action": "observe", "confidence": .9,
                          "arguments": {"evidence": "unknown"}}):
            with tempfile.TemporaryDirectory() as folder:
                report, row, model, robot, cameras, recorder = self.run_check(Path(folder), decision)
                self.assertFalse(report["live_check_passed"])
                self.assertEqual(report["step_count"], 1)
                self.assertEqual(report["control_commands_sent"], 0)
                self.assertEqual(robot.prepared, [])
                self.assertTrue(robot.closed and cameras.closed)

    def test_capture_or_backend_failure_logs_and_closes(self):
        for flags in ({"camera_fail": True}, {"model_fail": True}):
            with tempfile.TemporaryDirectory() as folder:
                report, row, model, robot, cameras, recorder = self.run_check(Path(folder), None, **flags)
                self.assertFalse(report["live_check_passed"])
                self.assertEqual(report["control_commands_sent"], 0)
                self.assertTrue(robot.closed and cameras.closed)
                self.assertIsNone(report["task_success"])
                self.assertEqual(report["model_call_count"], int(flags.get("model_fail", False)))

    def test_failed_motion_encoding_cannot_count_as_integration_pass(self):
        decision = {"phase": "APPROACH_PEN", "action": "move_eef", "confidence": .85,
                    "arguments": {"pose_m_rad": [.21, 0, .3, 0, 1, 0],
                                  "speed_percent": 3, "next_phase": None}}
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(SensorRobot, "prepare_command", side_effect=RuntimeError("no route")):
                report, row, model, robot, cameras, recorder = self.run_check(Path(folder), decision, "APPROACH_PEN")
            self.assertFalse(report["live_check_passed"])
            assessment = row["proposal_assessment"]
            self.assertTrue(assessment["schema_valid"] and assessment["phase_valid"])
            self.assertFalse(assessment["command_encoding_valid"])
            self.assertEqual(report["control_commands_sent"], 0)


if __name__ == "__main__":
    unittest.main()
