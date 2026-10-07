import contextlib
import io
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from right_pick import cli
from right_pick.config import load_config, configuration_blockers
from right_pick.model import ResponsesClient, build_payload, ModelConfigurationError
from right_pick.recording import Recorder
from right_pick.observation import require_fresh


ROOT = Path(__file__).resolve().parents[1]


class IntegrationTests(unittest.TestCase):
    def test_camera_retilt_is_explicit_and_no_old_transform_is_imported(self):
        config = load_config(ROOT / "configs/site.example.json")
        self.assertTrue(config["calibration"]["camera_geometry_changed"])
        self.assertIsNone(config["calibration"]["T_right_base_front"])
        self.assertIn("physical_motion_adapter_not_commissioned", configuration_blockers(config))

    def test_failed_environment_probe_never_reaches_observer(self):
        with tempfile.TemporaryDirectory() as temp:
            diagnostics = {"environment_blockers": ["socket denied"], "commissioning_blockers": []}
            with patch.object(cli, "preflight", return_value=diagnostics), patch.object(cli, "observe") as capture:
                with contextlib.redirect_stdout(io.StringIO()):
                    code = cli.main(["--config", str(ROOT/"configs/site.example.json"), "--runs", temp, "attempt"])
            self.assertEqual(code, 2)
            capture.assert_not_called()
            report = json.loads(next(Path(temp).glob("*/report.json")).read_text())
            self.assertEqual(report["outcome"], "blocked")
            self.assertIsNone(report["success"])
            self.assertEqual(report["physical_attempts"], 0)
            self.assertEqual(report["metrics"]["control_commands_sent"], 0)

    def test_endpoint_model_not_guessed(self):
        with self.assertRaises(ModelConfigurationError):
            build_payload({"protocol": None, "model_id": None}, "task", {})

    def test_invalid_model_json_still_records_usage_and_spent_call(self):
        config = {"protocol":"responses", "model_id":"explicit-test-id", "endpoint":"https://example.invalid/responses", "max_calls":1}
        payload = {"model":"reported-test-id", "usage":{"input_tokens":5,"output_tokens":3},
                   "output":[{"type":"message","content":[{"type":"output_text","text":"invalid json"}]}]}
        with tempfile.TemporaryDirectory() as temp:
            recorder = Recorder(temp, {}, "test", mode="nonphysical_replay")
            client = ResponsesClient(config, recorder)
            stamp = time.time()
            observation = {"robot_state_at":stamp,"cameras":{name:{"host_received_at":stamp} for name in ("front","left_hand","right_hand")}}
            with patch.dict("os.environ", {"OPENAI_API_KEY":"UNIT_TEST_PLACEHOLDER"}), patch("right_pick.model._open_request", return_value=io.BytesIO(json.dumps(payload).encode())) as transport:
                with self.assertRaises(json.JSONDecodeError):
                    client.decide("test", observation)
                with self.assertRaisesRegex(RuntimeError, "budget"):
                    client.decide("test", observation)
                self.assertEqual(transport.call_count, 1)
            self.assertEqual(recorder.usage_summary()["calls"], 1)
            self.assertEqual(recorder.usage_summary()["input_tokens"]["total"], 5)
            self.assertEqual(recorder.model_calls[0]["error"], "JSONDecodeError")

    def test_fresh_json_timestamp_does_not_refresh_old_images(self):
        observation = {"captured_at":100,"robot_state_at":100,"cameras":{name:{"host_received_at":90} for name in ("front","left_hand","right_hand")}}
        with self.assertRaisesRegex(ValueError, "stale"):
            require_fresh(observation, now=100)

    def test_history_zero_excludes_all_old_text(self):
        config={"protocol":"responses","model_id":"explicit-test-id","text_history_turns":0}
        result=build_payload(config,"task",{},["should disappear"])
        text=json.loads(result["input"][0]["content"][0]["text"])
        self.assertEqual(text["recent_text_history"], [])


if __name__ == "__main__":
    unittest.main()
