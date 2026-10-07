"""CLI routing tests patch every live adapter and never touch ROS or cameras."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from right_pick.fast_cli import run_fast
from right_pick.fast_safety import FastSafetyError
from right_pick.cli import main


class FastCLIRoutes(unittest.TestCase):
    def test_execution_mode_is_required_before_loading_site_config(self):
        with patch("right_pick.cli.load_config") as load_config, \
                contextlib.redirect_stderr(io.StringIO()) as stderr:
            with self.assertRaises(SystemExit) as raised:
                main(["astra_fast_closed_loop", "--model", "codex"])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("--execution", stderr.getvalue())
        load_config.assert_not_called()

    def options(self, directory, execution="live", model="codex"):
        root = Path(directory)
        config = root / "fast.json"
        config.write_text(json.dumps({"mode": "astra_fast_closed_loop", "model": {
            "model_id": "gpt-6-astra", "protocol": "responses" if model == "responses" else "codex_cli"}, "controller": {"max_steps": 19}}))
        return SimpleNamespace(fast_config=str(config), execution=execution, model=model,
                               runs=str(root / "runs"), phase="INIT", observation=[])

    def readiness(self):
        return dict(physical_execution_ready=True, physical_motion_authorized=False,
                    timeout_policy_verified=True, hold_verified=False, blockers=[],
                    qualification={"scope": "offline_test_fixture_not_site_evidence"})

    def test_live_preflight_failure_prevents_model_camera_or_loop_construction(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.options(directory)
            with patch("right_pick.fast_ros.ROSRightArm") as arm, \
                    patch("right_pick.fast_codex.CodexDecisionClient") as model, \
                    patch("right_pick.fast_observation.SubprocessRGBCameras") as cameras, \
                    patch("right_pick.fast_live_loop.FastLiveClosedLoop") as loop:
                arm.return_value.preflight.side_effect = FastSafetyError("no current timeout qualification")
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    result = run_fast(args, {"physical_limits": {"site": "bounds"}})
                self.assertEqual(result, 2)
                self.assertFalse(arm.call_args.kwargs["proposal_only"])
                arm.return_value.close.assert_called_once()
                arm.return_value.execute.assert_not_called()
                model.assert_not_called()
                cameras.assert_not_called()
                loop.assert_not_called()
                report = json.loads(output.getvalue())["report"]
                self.assertEqual(report["termination_reason"], "preflight_blocked")
                self.assertEqual(report["control_commands_sent"], 0)
                self.assertEqual(report["model_call_count"], 0)
                self.assertIsNone(report["task_success"])

    def test_live_route_builds_requested_real_backend_and_passes_site_limits(self):
        for backend, path in (("codex", "right_pick.fast_codex.CodexDecisionClient"),
                              ("responses", "right_pick.fast_model.FastResponsesClient")):
            with tempfile.TemporaryDirectory() as directory:
                args = self.options(directory, model=backend)
                site = {"physical_limits": {"site": "explicit_bounds"}}
                with patch("right_pick.fast_ros.ROSRightArm") as arm, \
                        patch(path) as model, \
                        patch("right_pick.fast_observation.SubprocessRGBCameras") as cameras, \
                        patch("right_pick.fast_live_loop.FastLiveClosedLoop") as loop:
                    arm.return_value.preflight.return_value = self.readiness()
                    loop.return_value.run.return_value = {"task_success": True}
                    with contextlib.redirect_stdout(io.StringIO()):
                        self.assertEqual(run_fast(args, site), 0)
                    arm.return_value.preflight.assert_called_once()
                    self.assertFalse(arm.call_args.kwargs["proposal_only"])
                    model.assert_called_once()
                    cameras.assert_called_once()
                    self.assertIs(loop.call_args.kwargs["robot"], arm.return_value)
                    self.assertIs(loop.call_args.kwargs["model"], model.return_value)
                    self.assertIs(loop.call_args.kwargs["cameras"], cameras.return_value)
                    self.assertEqual(loop.call_args.kwargs["limits"], site["physical_limits"])
                    self.assertEqual(loop.call_args.kwargs["options"], {"max_steps": 19})
                    loop.return_value.run.assert_called_once()
                    arm.return_value.execute.assert_not_called()

    def test_boolean_without_qualification_does_not_reach_live_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.options(directory)
            with patch("right_pick.fast_ros.ROSRightArm") as arm, \
                    patch("right_pick.fast_codex.CodexDecisionClient") as model, \
                    patch("right_pick.fast_observation.SubprocessRGBCameras") as cameras:
                arm.return_value.preflight.return_value = {"physical_motion_authorized": True}
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(run_fast(args, {}), 2)
                model.assert_not_called()
                cameras.assert_not_called()

    def test_prepare_stays_report_only(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.options(directory, execution="prepare")
            with patch("right_pick.fast_ros.ROSRightArm") as arm, \
                    patch("right_pick.fast_codex.CodexDecisionClient") as model, \
                    patch("right_pick.fast_observation.SubprocessRGBCameras") as cameras:
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    self.assertEqual(run_fast(args, {}), 2)
                report = json.loads(output.getvalue())
                self.assertEqual(report["control_commands_sent"], 0)
                self.assertEqual(report["report"]["model_call_count"], 0)
                arm.assert_not_called()
                model.assert_not_called()
                cameras.assert_not_called()

    def test_scripted_live_model_is_rejected_before_ros_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.options(directory, model="scripted")
            with patch("right_pick.fast_ros.ROSRightArm") as arm:
                with self.assertRaisesRegex(ValueError, "live requires explicit"):
                    run_fast(args, {})
                arm.assert_not_called()

    def test_live_setup_failure_closes_existing_adapters(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.options(directory)
            with patch("right_pick.fast_ros.ROSRightArm") as arm, \
                    patch("right_pick.fast_codex.CodexDecisionClient"), \
                    patch("right_pick.fast_observation.SubprocessRGBCameras") as cameras, \
                    patch("right_pick.fast_live_loop.FastLiveClosedLoop", side_effect=ValueError("limits missing")):
                arm.return_value.preflight.return_value = self.readiness()
                with self.assertRaisesRegex(ValueError, "limits missing"):
                    run_fast(args, {})
                arm.return_value.close.assert_called_once()
                cameras.return_value.close.assert_called_once()
                arm.return_value.execute.assert_not_called()

    def test_unimplemented_modes_and_tasks_fail_before_any_runtime_resource(self):
        for extra in ({"task_id": "charger"}, {"worker_arm": "left"},
                      {"pipeline_id": "dual_arm_barrier_v1", "peer_arm": "left"},
                      {"pipeline_id": "single_worker_with_observer_v1", "observer_arm": "left"}):
            with self.subTest(extra=extra), tempfile.TemporaryDirectory() as directory:
                args = self.options(directory)
                path = Path(args.fast_config)
                config = json.loads(path.read_text())
                config.update(extra)
                path.write_text(json.dumps(config))
                with patch("right_pick.fast_cli.Recorder") as recorder, \
                        patch("right_pick.fast_ros.ROSRightArm") as arm, \
                        patch("right_pick.fast_codex.CodexDecisionClient") as model, \
                        patch("right_pick.fast_observation.SubprocessRGBCameras") as cameras:
                    with self.assertRaises(ValueError):
                        run_fast(args, {})
                    recorder.assert_not_called()
                    arm.assert_not_called()
                    model.assert_not_called()
                    cameras.assert_not_called()

    def test_codex_never_falls_back_to_an_api_config(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.options(directory, model="codex")
            path = Path(args.fast_config)
            config = json.loads(path.read_text())
            config["model"]["protocol"] = "responses"
            path.write_text(json.dumps(config))
            with patch("right_pick.fast_cli.Recorder") as recorder, patch("right_pick.fast_ros.ROSRightArm") as arm:
                with self.assertRaisesRegex(ValueError, "backend does not match"):
                    run_fast(args, {})
                recorder.assert_not_called()
                arm.assert_not_called()


if __name__ == "__main__":
    unittest.main()
