"""Integration tests of historical RGB + compact model contract + mock execution."""
import base64
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from right_pick.fast_loop import FastClosedLoop
from right_pick.fast_observation import HistoricalRGBSource
from right_pick.fast_replay import MockRobot, ScriptedModel, mock_limits
from right_pick.recording import Recorder


def fixture(root):
    # Valid tiny PNG, only for transport-contract tests, never a visual eval.
    image = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aS1cAAAAASUVORK5CYII=")
    cameras = {}
    for role in ("front", "left_hand", "right_hand"):
        p = root / (role + ".png")
        p.write_bytes(image)
        cameras[role] = {"rgb_path": str(p), "timestamp": 100,
                         "intrinsics": {"fx": 100}, "red_candidates": [{"xyz": [9, 8, 7]}]}
    path = root / "historical.json"
    path.write_text(json.dumps({"cameras": cameras, "calibration": "secret-geometry"}))
    return path


class FastLoopTests(unittest.TestCase):
    def setup_run(self, root, decisions=None, options=None):
        path = fixture(root)
        recorder = Recorder(root / "runs", {}, "offline contract", mode="nonphysical_replay")
        model = ScriptedModel(recorder, decisions)
        robot = MockRobot()
        cameras = HistoricalRGBSource([path], recorder.run_dir / "observations")
        loop = FastClosedLoop(model=model, robot=robot, cameras=cameras,
                              recorder=recorder, limits=mock_limits(), options=options)
        return loop, recorder, model, robot

    def test_full_phase_cycle_recording_and_no_physical_success(self):
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder))
            report = loop.run()
            self.assertTrue(report["replay_completed"], report)
            self.assertIsNone(report["task_success"])
            self.assertEqual(report["model_call_count"], 0)
            self.assertEqual(report["scripted_decision_count"], 14)
            self.assertEqual(report["control_commands_sent"], 0)
            rows = [json.loads(s) for s in (recorder.run_dir / "steps.jsonl").read_text().splitlines()]
            required = {"step_id", "phase", "timestamp", "model_request_start", "model_response_end",
                        "agent_decide_s", "image_capture_s", "image_encode_s", "robot_execute_s",
                        "robot_wait_s", "total_step_s", "input_tokens", "output_tokens",
                        "reasoning_output_tokens", "selected_camera_views", "action",
                        "action_arguments", "previous_result", "confidence", "phase_transition", "retry_count"}
            self.assertTrue(all(required <= row.keys() for row in rows))
            self.assertEqual(len(list((recorder.run_dir / "observations").glob("*/observation.json"))), len(rows))
            prompt = (recorder.run_dir / "prompt_0003.json").read_text()
            for forbidden in ("secret-geometry", "red_candidates", "intrinsics", "recent_text_history"):
                self.assertNotIn(forbidden, prompt)
            self.assertIn("ALIGN_PEN", prompt)
            self.assertEqual(report["final_phase"], "DONE")
            self.assertGreaterEqual(report["other_total_s"], 0)

    def test_invalid_responses_cannot_loop_forever(self):
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder), decisions=[{"not": "an action"}] * 10)
            report = loop.run()
            self.assertEqual(report["termination_reason"], "retry_budget_exhausted")
            self.assertEqual(report["step_count"], 2)
            self.assertFalse(report["replay_completed"])
            self.assertEqual(robot.executions, [])

    def test_one_schema_correction_can_resume_without_resetting_run_budget(self):
        from right_pick.fast_replay import demonstration_decisions
        recover = dict(phase="RECOVERY", action="advance", confidence=.9,
                       arguments={"next_phase": "INIT", "evidence": "phase_complete"},
                       explanation="Fresh observation supports restarting phase evaluation; no action was sent.")
        with tempfile.TemporaryDirectory() as folder:
            loop, _, model, _ = self.setup_run(Path(folder),
                [{"bad": "schema"}, recover] + demonstration_decisions())
            report = loop.run()
            self.assertTrue(report["replay_completed"], report)
            self.assertEqual(report["recovery_count"], 1)
            self.assertEqual(model.calls, 16)

    def test_low_confidence_pause_ends_without_dispatch(self):
        decision = dict(phase="INIT", action="pause", arguments={"evidence": "unknown"}, confidence=.1)
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder), [decision])
            report = loop.run()
            self.assertEqual(report["termination_reason"], "model_requested_pause")
            self.assertEqual(model.calls, 1)
            self.assertEqual(robot.executions, [])

    def test_execution_exception_latches_and_never_retries(self):
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder))
            with patch.object(robot, "execute", side_effect=TimeoutError("mock timeout")) as execute:
                report = loop.run()
            self.assertEqual(execute.call_count, 1)
            self.assertEqual(report["termination_reason"], "execution_failed_latched")
            self.assertEqual(model.calls, 1)
            row = json.loads((recorder.run_dir / "steps.jsonl").read_text().splitlines()[0])
            self.assertGreater(row["robot_execute_s"], 0)

    def test_phase_stall_has_finite_exit(self):
        repeated = {"phase": "INIT", "action": "observe", "arguments": {"evidence": "unknown"}, "confidence": .9}
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder), decisions=[repeated] * 30,
                                                         options={"max_phase_steps": 2, "max_steps": 8})
            report = loop.run()
            self.assertLessEqual(report["step_count"], 8)
            self.assertGreaterEqual(report["recovery_count"], 1)
            self.assertFalse(report["replay_completed"])

    def test_model_call_budget_has_explicit_exit(self):
        repeated = {"phase": "INIT", "action": "observe", "arguments": {"evidence": "unknown"}, "confidence": .9}
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder), decisions=[repeated] * 8,
                options={"max_model_calls": 2, "max_steps": 8, "max_phase_steps": 8,
                         "max_observe_unknown": 8})
            report = loop.run()
            self.assertEqual(report["termination_reason"], "model_call_budget_exhausted")
            self.assertEqual(report["step_count"], 2)
            self.assertEqual(model.calls, 2)
            self.assertEqual(report["control_commands_sent"], 0)

    def test_unknown_observation_budget_is_configurable(self):
        repeated = {"phase": "INIT", "action": "observe", "arguments": {"evidence": "unknown"}, "confidence": .9}
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder), decisions=[repeated] * 8,
                options={"max_observe_unknown": 2, "max_phase_steps": 8, "max_steps": 8})
            report = loop.run()
            self.assertEqual(report["termination_reason"], "observe_unknown_budget_exhausted")
            self.assertEqual(model.calls, 2)
            rows = [json.loads(s) for s in (recorder.run_dir / "steps.jsonl").read_text().splitlines()]
            self.assertEqual([row["consecutive_observe_unknown_after"] for row in rows], [1, 2])
            self.assertEqual(report["control_commands_sent"], 0)

    def test_historical_source_never_fills_missing_view(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = fixture(root)
            obs = json.loads(path.read_text())
            del obs["cameras"]["left_hand"]
            path.write_text(json.dumps(obs))
            source = HistoricalRGBSource([path], root / "output")
            with self.assertRaises(ValueError):
                source.capture()

    def test_live_flag_cannot_unlock_loop(self):
        with tempfile.TemporaryDirectory() as folder:
            loop, recorder, model, robot = self.setup_run(Path(folder))
            robot.nonphysical = False
            with self.assertRaises(RuntimeError):
                FastClosedLoop(model=model, robot=robot, cameras=loop.cameras,
                               recorder=recorder, limits=mock_limits())


if __name__ == "__main__":
    unittest.main()
