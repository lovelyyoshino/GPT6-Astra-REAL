"""Local files and mocked HTTP only: never hardware or a paid model request."""
import base64
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import urllib.error

from right_pick import fast_model
from right_pick.fast_policy import FastPolicyError
from right_pick.recording import Recorder


PNG = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII=")


class FastModelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = {"protocol": "responses", "model_id": "gpt-6-astra",
                       "endpoint": "https://example.invalid/v1/responses",
                       "api_key_env": "FAST_MODEL_TEST_KEY", "max_calls": 3,
                       "max_observation_age_s": 10, "max_sensor_skew_s": 0.15}
        self.env = patch.dict(os.environ, {"FAST_MODEL_TEST_KEY": "UNIT_TEST_CREDENTIAL"})
        self.env.start()
        self.addCleanup(self.env.stop)
        network = patch("socket.socket", side_effect=AssertionError("Real network forbidden in test"))
        network.start()
        self.addCleanup(network.stop)
        guard = patch.object(fast_model, "_open_request", side_effect=AssertionError("Unmocked request forbidden"))
        guard.start()
        self.addCleanup(guard.stop)
        stamp = time.time()
        self.state = {"phase": "INIT", "robot_state": {"sampled_at": stamp},
                      "gripper_state": {"opening_m": 0.035}, "memory": ""}
        self.observation = {"cameras": {}, "depth": "DO_NOT_SEND_DEPTH",
                            "object_coordinates": "DO_NOT_SEND_OBJECTS",
                            "calibration": "DO_NOT_SEND_CALIBRATION"}
        for view in ("front", "left_hand", "right_hand"):
            p = self.root / (view + ".png")
            p.write_bytes(PNG)
            self.observation["cameras"][view] = {"rgb_path": str(p), "host_received_at": stamp,
                                                   "intrinsics": "DO_NOT_SEND_INTRINSICS"}
        self.recorder = Recorder(self.root / "runs", {}, "mock API verification", mode="nonphysical_replay")

    def decision(self, phase="INIT", explanation=None):
        result = {"phase": phase, "action": "observe", "arguments": {"evidence": "unknown"}, "confidence": 0.8}
        if explanation is not None:
            result["explanation"] = explanation
        return result

    def response(self, decision=None):
        return {"id": "resp_fake", "model": "gpt-6-astra", "status": "completed",
                "usage": {"input_tokens": 123, "output_tokens": 45,
                          "input_tokens_details": {"cached_tokens": 20},
                          "output_tokens_details": {"reasoning_tokens": 30}},
                "output": [{"type": "message", "role": "assistant", "status": "completed",
                            "content": [{"type": "output_text", "text": json.dumps(decision or self.decision())}]}]}

    def client(self, **overrides):
        return fast_model.FastResponsesClient(dict(self.config, **overrides), self.recorder)

    def call(self, client, result=None, state=None, observation=None):
        raw = json.dumps(result if result is not None else self.response()).encode()
        with patch.object(fast_model, "_open_request", return_value=io.BytesIO(raw)) as transport:
            decision = client.decide(state or self.state, observation or self.observation)
        return decision, transport

    def test_payload_projects_state_uses_selected_original_rgb_and_strict_schema(self):
        state = dict(self.state, history=["DO_NOT_SEND_HISTORY"], object_pose="DO_NOT_SEND_OBJECT_POSE")
        payload = fast_model.build_fast_payload(self.config, state, self.observation)
        self.assertFalse(payload["store"])
        self.assertEqual(payload["reasoning"], {"effort": "medium"})
        self.assertNotIn("previous_response_id", payload)
        self.assertNotIn("stream", payload)
        self.assertEqual(len(payload["input"]), 1)
        content = payload["input"][0]["content"]
        text = json.loads(content[0]["text"])
        self.assertEqual(text["input_kind"], "current_rgb_and_state")
        self.assertIn("Do not plan later phases", text["instruction"])
        self.assertIn("no-contact", text["instruction"])
        images = [p for p in content if p["type"] == "input_image"]
        self.assertEqual(len(images), 2)
        for image in images:
            self.assertEqual(image["detail"], "high")
            self.assertEqual(base64.b64decode(image["image_url"].split(",", 1)[1]), PNG)
        self.assertNotIn("DO_NOT_SEND", json.dumps(payload))
        schema = payload["text"]["format"]
        self.assertEqual(schema["type"], "json_schema")
        self.assertTrue(schema["strict"])
        self.assertNotIn("explanation", schema["schema"]["properties"])

    def test_front_only_phase_does_not_open_unselected_files(self):
        state = dict(self.state, phase="APPROACH_HOLDER")
        obs = deepcopy(self.observation)
        for view in ("left_hand", "right_hand"):
            obs["cameras"][view]["rgb_path"] = str(self.root / "missing.png")
        payload = fast_model.build_fast_payload(self.config, state, obs)
        self.assertEqual(payload["reasoning"]["effort"], "low")
        self.assertEqual(sum(x["type"] == "input_image" for x in payload["input"][0]["content"]), 1)

    def test_exception_uses_three_views_xhigh_and_bounded_explanation(self):
        state = dict(self.state, phase="RECOVERY", retry_count=2)
        client = self.client()
        result, _ = self.call(client, self.response(self.decision("RECOVERY", "Target is occluded.")), state)
        self.assertEqual(result["explanation"], "Target is occluded.")
        self.assertEqual(client.last_metrics["selected_camera_views"], ["front", "left_hand", "right_hand"])
        self.assertEqual(client.last_metrics["reasoning_effort"], "xhigh")

    def test_timings_are_disjoint_and_normalized_usage_is_recorded(self):
        client = self.client()
        with patch.object(fast_model.time, "monotonic", side_effect=[10.0, 12.0, 20.0, 25.0]):
            result, transport = self.call(client)
        self.assertEqual(result, self.decision())
        self.assertEqual(transport.call_count, 1)
        m = client.last_metrics
        self.assertEqual(m["image_encode_s"], 2.0)
        self.assertEqual(m["agent_decide_s"], 5.0)
        self.assertLessEqual(m["model_request_start"], m["model_response_end"])
        self.assertEqual(m["actual_model"], "gpt-6-astra")
        self.assertEqual(m["input_tokens"], 123)
        self.assertEqual(m["output_tokens"], 45)
        self.assertEqual(m["reasoning_output_tokens"], 30)
        self.assertEqual(m["request_id"], "resp_fake")
        self.assertEqual(self.recorder.usage_summary()["interface_wait_time_s"], 5.0)

    def test_input_audit_has_no_image_bytes_credentials_or_provider_reasoning(self):
        result = self.response()
        result["output"].insert(0, {"type": "reasoning", "summary": [{"text": "PRIVATE_PROVIDER_REASONING"}]})
        client = self.client()
        self.call(client, result)
        audit = json.loads(Path(client.last_metrics["input_packet_path"]).read_text())
        self.assertEqual(len(audit["images"]), 2)
        self.assertEqual(audit["compact_text"]["controller_state"]["phase"], "INIT")
        logs = "\n".join(p.read_text() for p in self.recorder.run_dir.iterdir())
        for excluded in ("PRIVATE_PROVIDER_REASONING", "UNIT_TEST_CREDENTIAL", base64.b64encode(PNG).decode(), "DO_NOT_SEND_DEPTH"):
            self.assertNotIn(excluded, logs)

    def test_historical_builder_is_offline_and_provenance_is_explicit(self):
        obs = deepcopy(self.observation)
        obs.update(historical=True, nonphysical=True)
        with patch.dict(os.environ, {}, clear=True):
            payload = fast_model.build_fast_payload(self.config, self.state, obs)
        packet = json.loads(payload["input"][0]["content"][0]["text"])
        self.assertEqual(packet["input_kind"], "historical_rgb_with_offline_state")
        self.assertIn("not consequences of mock actions", packet["instruction"])
        client = self.client()
        with self.assertRaisesRegex(fast_model.FastModelConfigurationError, "Historical"):
            client.decide(self.state, obs)
        self.assertEqual(client.calls, 0)
        self.call(self.client(allow_historical=True), observation=obs)

    def test_invalid_configuration_or_missing_credential_never_requests(self):
        bad = [{"model_id": "other-model"}, {"protocol": None}, {"endpoint": "http://example.invalid"},
               {"endpoint": "https://user:secret@example.invalid/responses"}, {"max_calls": 0},
               {"max_calls": True}, {"timeout_s": 0}, {"max_output_tokens": False},
               {"allow_historical": "yes"}]
        for settings in bad:
            with self.subTest(settings=settings):
                client = self.client(**settings)
                with self.assertRaises(fast_model.FastModelConfigurationError):
                    client.decide(self.state, self.observation)
                self.assertEqual(client.calls, 0)
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(fast_model.FastModelConfigurationError, "credential"):
                self.client().decide(self.state, self.observation)

    def test_current_timestamp_missing_stale_future_and_skew_are_rejected(self):
        variants = []
        state = deepcopy(self.state); state["robot_state"].pop("sampled_at"); variants.append((state, self.observation))
        state = deepcopy(self.state); state["robot_state"]["sampled_at"] -= 100; variants.append((state, self.observation))
        obs = deepcopy(self.observation); obs["cameras"]["front"]["host_received_at"] += 100; variants.append((self.state, obs))
        obs = deepcopy(self.observation); obs["cameras"]["front"]["host_received_at"] -= 1; variants.append((self.state, obs))
        for state, obs in variants:
            with self.subTest(state=state, observation=obs):
                client = self.client()
                with self.assertRaises(fast_model.FastModelConfigurationError):
                    client.decide(state, obs)
                self.assertEqual(client.calls, 0)

    def test_timestamp_is_rechecked_after_encoding_and_audit_write(self):
        write = self.recorder._write_json
        def stale_after_write(*args):
            write(*args)
            self.observation["cameras"]["front"]["host_received_at"] -= 100
        client = self.client()
        with patch.object(self.recorder, "_write_json", side_effect=stale_after_write):
            with self.assertRaisesRegex(fast_model.FastModelConfigurationError, "stale"):
                client.decide(self.state, self.observation)
        self.assertEqual(client.calls, 0)
        self.assertIsNone(client.last_metrics["model_request_start"])

    def test_failures_consume_budget_and_do_not_retry(self):
        client = self.client(max_calls=1)
        with patch.object(fast_model, "_open_request", side_effect=urllib.error.URLError("test")) as transport:
            with self.assertRaisesRegex(fast_model.FastModelError, "no retry"):
                client.decide(self.state, self.observation)
            with self.assertRaisesRegex(fast_model.FastModelError, "budget"):
                client.decide(self.state, self.observation)
        self.assertEqual(transport.call_count, 1)
        self.assertEqual(client.calls, 1)
        self.assertEqual(len(self.recorder.model_calls), 1)
        self.assertEqual(self.recorder.model_calls[0]["error"], "transport_error")
        self.assertIsNone(self.recorder.model_calls[0]["usage"]["input_tokens"])

    def test_http_failure_records_end_time_and_has_no_retry(self):
        client = self.client()
        error = urllib.error.HTTPError(self.config["endpoint"], 429, "test", {}, None)
        with patch.object(fast_model, "_open_request", side_effect=error) as transport:
            with self.assertRaisesRegex(fast_model.FastModelError, "429"):
                client.decide(self.state, self.observation)
        self.assertEqual(transport.call_count, 1)
        self.assertIsNotNone(client.last_metrics["model_response_end"])
        self.assertEqual(client.last_metrics["error"], "http_error:429")

    def test_response_envelope_refuses_incomplete_refusal_multi_message_and_fallback(self):
        variants = []
        r = self.response(); r["status"] = "incomplete"; variants.append(r)
        r = self.response(); r["incomplete_details"] = {"reason": "max_output_tokens"}; variants.append(r)
        r = self.response(); r["output"][0]["content"] = [{"type": "refusal", "refusal": "no"}]; variants.append(r)
        r = self.response(); r["output"].append(deepcopy(r["output"][0])); variants.append(r)
        r = self.response(); r["output"][0]["content"].append(deepcopy(r["output"][0]["content"][0])); variants.append(r)
        r = self.response(); r["output"].append({"type": "function_call"}); variants.append(r)
        r = self.response(); r["model"] = "fallback-model"; variants.append(r)
        r = self.response(); r["output"][0]["status"] = "in_progress"; variants.append(r)
        for result in variants:
            with self.subTest(result=result):
                client = self.client()
                with self.assertRaises(fast_model.FastModelResponseError):
                    self.call(client, result)
                self.assertEqual(client.calls, 1)

    def test_invalid_action_wrong_phase_and_long_explanation_still_count(self):
        client = self.client()
        for text in ("not JSON", json.dumps(self.decision("ALIGN_PEN")),
                     json.dumps(dict(self.decision(), rationale="unrequested rationale"))):
            result = self.response(); result["output"][0]["content"][0]["text"] = text
            with self.assertRaises((FastPolicyError, fast_model.FastModelResponseError)):
                self.call(client, result)
        self.assertEqual(client.calls, 3)
        state = dict(self.state, phase="RECOVERY")
        with self.assertRaises(FastPolicyError):
            self.call(self.client(), self.response(self.decision("RECOVERY", "x" * 241)), state)

    def test_invalid_outer_json_duplicate_keys_and_size_limit_are_rejected(self):
        for raw, limit in ((b'not json', 1000), (b'{"status":1,"status":2}', 1000), (b'x' * 20, 10)):
            with self.subTest(raw=raw):
                client = self.client(max_response_bytes=limit)
                with patch.object(fast_model, "_open_request", return_value=io.BytesIO(raw)) as transport:
                    with self.assertRaises((json.JSONDecodeError, fast_model.FastModelResponseError)):
                        client.decide(self.state, self.observation)
                self.assertEqual(transport.call_count, 1)
                self.assertEqual(client.calls, 1)

    def test_input_audit_failure_prevents_http_and_no_budget_is_spent(self):
        client = self.client()
        with patch.object(self.recorder, "_write_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                client.decide(self.state, self.observation)
        self.assertEqual(client.calls, 0)


if __name__ == "__main__":
    unittest.main()
