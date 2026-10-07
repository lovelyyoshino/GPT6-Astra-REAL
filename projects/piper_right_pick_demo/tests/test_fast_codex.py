"""Pure local mocks; never invoke a model, camera, ROS or CAN."""
import base64
from copy import deepcopy
import json
import io
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from right_pick import fast_codex
from right_pick.fast_model import FastModelError
from right_pick.recording import Recorder


PNG = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII=")
INVOKE = fast_codex.invoke_codex


class FastCodexTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = {"model_id": "gpt-6-astra", "protocol": "codex_cli", "max_calls": 2,
                       "max_observation_age_s": 10, "max_sensor_skew_s": .2}
        stamp = time.time()
        self.state = {"phase": "INIT", "robot_state": {"sampled_at": stamp}, "memory": ""}
        self.obs = {"cameras": {}, "depth": "DO_NOT_SEND", "object_coordinates": "DO_NOT_SEND"}
        for view in ("front", "left_hand", "right_hand"):
            path = self.root / (view + ".png")
            path.write_bytes(PNG)
            self.obs["cameras"][view] = {"rgb_path": str(path), "host_received_at": stamp}
        self.recorder = Recorder(self.root / "runs", {}, "Codex unit test", mode="nonphysical_replay")
        for target in ("socket.socket", "subprocess.Popen"):
            guard = patch(target, side_effect=AssertionError("No real network/process in unit tests"))
            guard.start()
            self.addCleanup(guard.stop)
        version = patch.object(fast_codex.subprocess, "run", return_value=Mock(stdout=fast_codex.CLI_VERSION))
        version.start()
        self.addCleanup(version.stop)
        catalog = patch.object(fast_codex, "_catalog_bytes", return_value=json.dumps({
            "models": [{"slug": "gpt-6-astra", "apply_patch_tool_type": None,
            "experimental_supported_tools": [], "tool_mode": None, "node_repl_disabled": True,
            "supports_search_tool": False}]}).encode())
        catalog.start()
        self.addCleanup(catalog.stop)
        transport = patch.object(fast_codex, "invoke_codex", side_effect=AssertionError("Unmocked model forbidden"))
        transport.start()
        self.addCleanup(transport.stop)

    def decision(self, phase="INIT", explanation=None):
        d = {"phase": phase, "action": "observe", "arguments": {"evidence": "unknown"}, "confidence": .8}
        if explanation is not None:
            d["explanation"] = explanation
        return d

    def response(self, decision=None, usage=None):
        return {"text": json.dumps(decision or self.decision()), "usage": usage,
                "actual_model": None, "thread_id": "unit-thread", "event_count": 4}

    def client(self, **overrides):
        return fast_codex.CodexDecisionClient(dict(self.config, **overrides), self.recorder)

    def call(self, client, response=None, state=None, obs=None):
        with patch.object(fast_codex, "invoke_codex", return_value=response or self.response()) as invoke:
            result = client.decide(state or self.state, obs or self.obs)
        return result, invoke

    def test_live_capabilities_reach_actual_cli_schema_without_changing_backend(self):
        budget = dict(max_translation_m=.003, max_rotation_rad=.01, max_speed_percent=1,
            max_waypoints=1, gripper_min_m=0., gripper_max_m=.055, max_effort_parameter_nm=.2,
            allow_waypoint_chunks=False, required_effort_parameter_nm=.2)
        for phase in ("INIT", "APPROACH_PEN"):
            state = dict(self.state, phase=phase, action_budget=budget)
            _, invoke = self.call(self.client(), response=self.response(self.decision(phase)), state=state)
            schema = invoke.call_args.args[1]
            prompt = json.loads(invoke.call_args.args[2])
            self.assertEqual(schema["properties"]["phase"]["enum"], [phase])
            self.assertNotIn("move_eef_chunk", schema["properties"]["action"]["enum"])
            self.assertFalse(prompt["phase_contract"]["chunk_allowed"])
            variants = schema["properties"]["arguments"]["anyOf"]
            self.assertFalse(any("waypoints" in variant["properties"] for variant in variants))
            if phase == "INIT":
                gripper = next(v["properties"] for v in variants if "opening_m" in v["properties"])
                self.assertEqual(gripper["effort_parameter_nm"], {"type": "number", "enum": [.2]})

    def test_cli_argv_fixes_model_and_disables_tools_history_and_retries(self):
        args = fast_codex.codex_argv("codex", "/tmp/schema", "low", ["/tmp/rgb image.png"])
        for required in ["--no-daemon", "never", "--ephemeral", "--ignore-user-config",
                         "--ignore-rules", "--strict-config", "--json", "read-only",
                         "gpt-6-astra", "--image=/tmp/rgb image.png", "mcp_servers={}",
                         'web_search="disabled"', "project_doc_max_bytes=0",
                         "features.shell_tool=false", "features.unified_exec=false",
                         "features.code_mode_host=false", "features.plugins=false",
                         "features.multi_agent=false", "features.hooks=false",
                         "model_providers.piper_official_no_retry.request_max_retries=0",
                         "model_providers.piper_official_no_retry.stream_max_retries=0"]:
            self.assertIn(required, args)
        self.assertEqual(args[-1], "-")
        self.assertNotIn("resume", args)
        self.assertNotIn("--dangerously-bypass-approvals-and-sandbox", args)

    def test_catalog_preserves_model_identity_and_only_restricts_tools(self):
        original = {"models": [{"slug": "gpt-6-astra", "display_name": "Astra",
                     "base_instructions": "CLI_FIXED_PROMPT", "shell_type": "unified_exec",
                     "apply_patch_tool_type": "freeform", "experimental_supported_tools": ["clock"],
                     "tool_mode": "code_mode_only", "node_repl_disabled": False,
                     "supports_search_tool": True}, {"slug": "another"}], "etag": "metadata"}
        selected = fast_codex.isolated_catalog(original)
        self.assertEqual(len(selected["models"]), 1)
        model = selected["models"][0]
        self.assertEqual(model["slug"], "gpt-6-astra")
        self.assertEqual(model["base_instructions"], "CLI_FIXED_PROMPT")
        self.assertIsNone(model["apply_patch_tool_type"])
        self.assertEqual(model["experimental_supported_tools"], [])
        self.assertIsNone(model["tool_mode"])
        self.assertTrue(model["node_repl_disabled"])
        self.assertFalse(model["supports_search_tool"])
        self.assertEqual(original["models"][0]["experimental_supported_tools"], ["clock"])
        with self.assertRaises(FastModelError):
            fast_codex.isolated_catalog({"models": []})

    def test_error_diagnostic_redacts_values_and_is_bounded(self):
        event = {"type": "error", "message": "Bearer SECRET access_token=VALUE https://example.invalid/key?token=V " + "a" * 500}
        with self.assertRaises(FastModelError) as caught:
            fast_codex._Events().feed(json.dumps(event))
        text = str(caught.exception)
        self.assertNotIn("SECRET", text)
        self.assertNotIn("VALUE", text)
        self.assertNotIn("example.invalid", text)
        self.assertLess(len(text), 330)

    def test_packet_projection_selected_original_files_and_schema(self):
        state = dict(self.state, history=["DO_NOT_SEND"])
        result, invoke = self.call(self.client(), state=state)
        self.assertEqual(result, self.decision())
        args = invoke.call_args.args
        packet = json.loads(args[2])
        self.assertNotIn("DO_NOT_SEND", args[2])
        self.assertIn("Do not plan later phases", packet["instruction"])
        self.assertIn("after one observe(unknown)", packet["instruction"])
        self.assertIn("otherwise pause with the specific missing fact", packet["instruction"])
        self.assertNotIn("request observation or pause", packet["instruction"])
        self.assertEqual(packet["image_order"], ["front", "right_hand"])
        self.assertEqual(args[3], "medium")
        self.assertEqual(args[4], [str(Path(self.obs["cameras"][v]["rgb_path"]).resolve())
                                   for v in ("front", "right_hand")])
        self.assertEqual(packet["pipeline"]["id"], "single_arm_closed_loop_v1")
        self.assertEqual(packet["pipeline"]["stage"], "decide_one")
        self.assertFalse(args[1]["additionalProperties"])
        self.assertNotIn("explanation", args[1]["properties"])
        audits = list(self.recorder.run_dir.glob("fast_codex_input_*.json"))
        self.assertEqual(len(audits), 1)
        self.assertNotIn("base64", audits[0].read_text())

    def test_prepare_is_local_cached_and_preserves_prelaunch_freshness_gate(self):
        client = self.client()
        with patch.object(fast_codex, "_catalog_bytes", wraps=fast_codex._catalog_bytes) as catalog:
            first = client.prepare()
            self.assertEqual(first, client.prepare())
            self.assertEqual(catalog.call_count, 1)
            self.assertEqual(client.calls, 0)
            _, invoke = self.call(client)
            self.assertEqual(catalog.call_count, 1)
        self.obs["cameras"]["front"]["host_received_at"] -= 100
        with self.assertRaisesRegex(FastModelError, "stale"):
            invoke.call_args.kwargs["before_launch"]()

    def test_phase_camera_effort_exception_explanation(self):
        state = dict(self.state, phase="RECOVERY", retry_count=2)
        decision = self.decision("RECOVERY", "Target occluded.")
        client = self.client()
        result, invoke = self.call(client, self.response(decision), state)
        self.assertEqual(result, decision)
        self.assertEqual(invoke.call_args.args[3], "xhigh")
        self.assertEqual(client.last_metrics["selected_camera_views"], ["front", "left_hand", "right_hand"])
        self.assertIn("explanation", invoke.call_args.args[1]["required"])
        state = dict(self.state, phase="APPROACH_HOLDER")
        _, invoke = self.call(self.client(), self.response(self.decision("APPROACH_HOLDER")), state)
        self.assertEqual(invoke.call_args.args[3], "low")
        self.assertEqual(len(invoke.call_args.args[4]), 1)

    def test_unknown_usage_actual_model_and_cost_stay_unknown(self):
        client = self.client()
        self.call(client)
        self.assertIsNone(client.last_metrics["actual_model"])
        self.assertIsNone(client.last_metrics["cost"])
        self.assertTrue(client.last_metrics["cli_system_prompt_overhead"])
        self.assertTrue(all(v is None for v in client.last_metrics["usage"].values()))
        self.assertEqual(client.last_metrics["decision_source"], "codex_cli")

    def test_disjoint_timings_and_flat_cli_usage_mapping(self):
        client = self.client()
        usage = fast_codex._usage({"input_tokens": 100, "output_tokens": 20,
                                   "cached_input_tokens": 50, "reasoning_output_tokens": 12})
        with patch.object(fast_codex.time, "monotonic", side_effect=[10., 12., 20., 27.]):
            self.call(client, self.response(usage=usage))
        self.assertEqual(client.last_metrics["image_encode_s"], 2.)
        self.assertEqual(client.last_metrics["agent_decide_s"], 7.)
        self.assertEqual(client.last_metrics["input_tokens"], 100)
        self.assertEqual(client.last_metrics["reasoning_output_tokens"], 12)
        self.assertEqual(self.recorder.model_calls[0]["usage"]["cached_input_tokens"], 50)

    def test_failed_calls_consume_budget_no_retry_and_have_timing(self):
        client = self.client(max_calls=1)
        with patch.object(fast_codex, "invoke_codex", side_effect=FastModelError("timeout")) as invoke:
            with self.assertRaises(FastModelError):
                client.decide(self.state, self.obs)
            with self.assertRaisesRegex(FastModelError, "budget"):
                client.decide(self.state, self.obs)
        self.assertEqual(invoke.call_count, 1)
        self.assertEqual(client.calls, 1)
        row = self.recorder.model_calls[0]
        self.assertEqual(row["error"], "FastModelError")
        self.assertGreaterEqual(row["elapsed_s"], 0.)

    def test_stale_skew_and_historical_gates(self):
        for kind in ("old", "skew", "historical"):
            with self.subTest(kind=kind):
                obs = deepcopy(self.obs)
                if kind == "old":
                    obs["cameras"]["front"]["host_received_at"] -= 30
                elif kind == "skew":
                    obs["cameras"]["front"]["host_received_at"] -= 1
                else:
                    obs["historical"] = True
                with self.assertRaises(FastModelError):
                    self.client().decide(self.state, obs)
        obs = dict(self.obs, historical=True)
        _, invoke = self.call(self.client(allow_historical=True), obs=obs)
        self.assertIn("historical_rgb_with_offline_state", invoke.call_args.args[2])

    def test_freshness_rechecked_after_audit_write(self):
        def age_input(*args):
            self.obs["cameras"]["front"]["host_received_at"] -= 100
        with patch.object(self.recorder, "_write_json", side_effect=age_input):
            with self.assertRaisesRegex(FastModelError, "stale"):
                self.client().decide(self.state, self.obs)

    def test_model_version_config_and_malformed_response_fail_closed(self):
        for overrides in ({"model_id": "another"}, {"protocol": "responses"}, {"max_calls": 0},
                          {"timeout_s": float("nan")}, {"allow_historical": "yes"}):
            with self.subTest(overrides=overrides), self.assertRaises(FastModelError):
                self.client(**overrides).decide(self.state, self.obs)
        with patch.object(fast_codex.subprocess, "run", return_value=Mock(stdout="codex-cli 9.9")):
            with self.assertRaisesRegex(FastModelError, "version"):
                self.client().decide(self.state, self.obs)
        for text in ("not JSON", '{"phase":"INIT","phase":"INIT"}', '{"x":NaN}'):
            result = self.response()
            result["text"] = text
            with self.subTest(text=text), self.assertRaises((ValueError, FastModelError)):
                self.call(self.client(), result)

    def test_environment_drops_parent_remote_and_robot_state(self):
        with patch.dict(os.environ, {"CODEX_THREAD_ID": "parent", "ROS_MASTER_URI": "robot",
                                    "OPENAI_API_KEY": "NEVER_LOG", "CODEX_REMOTE_ADDR": "remote"}):
            env = fast_codex._environment()
        for key in ("CODEX_THREAD_ID", "ROS_MASTER_URI", "OPENAI_API_KEY", "CODEX_REMOTE_ADDR"):
            self.assertNotIn(key, env)

    def test_jsonl_only_one_final_no_reasoning_is_retained(self):
        events = fast_codex._Events()
        for event in [{"type": "thread.started", "thread_id": "t"}, {"type": "turn.started"},
                      {"type": "item.completed", "item": {"type": "reasoning", "text": "PRIVATE"}},
                      {"type": "item.completed", "item": {"type": "agent_message", "text": "{\"ok\":true}"}},
                      {"type": "turn.completed", "usage": {"input_tokens": 5}}]:
            events.feed(json.dumps(event))
        result = events.result()
        self.assertEqual(result["text"], '{"ok":true}')
        self.assertNotIn("PRIVATE", repr(vars(events)))

    def test_jsonl_refuses_all_tool_events_error_multiple_and_incomplete(self):
        for kind in ("command_execution", "file_change", "mcp_tool_call", "web_search", "todo_list"):
            with self.subTest(kind=kind), self.assertRaises(FastModelError):
                fast_codex._Events().feed(json.dumps({"type": "item.started", "item": {"type": kind}}))
        for event in ({"type": "turn.failed"}, {"type": "error"}, {"type": "refusal"}):
            with self.assertRaises(FastModelError):
                fast_codex._Events().feed(json.dumps(event))
        events = fast_codex._Events()
        events.feed('{"type":"turn.started"}')
        with self.assertRaises(FastModelError):
            events.feed('{"type":"turn.started"}')
        events = fast_codex._Events()
        with self.assertRaises(FastModelError):
            events.result()
        message = json.dumps({"type": "item.completed", "item": {"type": "agent_message", "text": "{}"}})
        events.feed(message)
        with self.assertRaises(FastModelError):
            events.feed(message)

    def test_cleanup_targets_only_own_process_group(self):
        proc = Mock(pid=765432)
        proc.poll.return_value = None
        with patch.object(fast_codex.os, "killpg") as kill:
            fast_codex._kill_child(proc)
        kill.assert_called_once_with(765432, fast_codex.signal.SIGKILL)
        proc.wait.assert_called_once_with(timeout=5)

    def test_subprocess_uses_empty_cwd_arg_list_and_strict_event_stream(self):
        proc = Mock(pid=765432, stdin=io.BytesIO(), stdout=Mock())
        proc.poll.return_value = 0
        proc.wait.return_value = 0
        selector = Mock()
        selector.get_map.side_effect = [True, True, False]
        selector.select.return_value = [(Mock(fd=123, fileobj=proc.stdout), 1)]
        stream = b'\n'.join(json.dumps(e).encode() for e in [
            {"type": "thread.started", "thread_id": "test"}, {"type": "turn.started"},
            {"type": "item.completed", "item": {"type": "agent_message", "text": '{"ok":true}'}},
            {"type": "turn.completed"}]) + b'\n'
        catalog = {"models": [{"slug": "gpt-6-astra"}]}
        def spawn(args, **kwargs):
            self.assertIsInstance(args, list)
            self.assertFalse(kwargs["shell"])
            self.assertTrue(kwargs["start_new_session"])
            self.assertEqual(list(Path(kwargs["cwd"]).iterdir()), [])
            self.assertIn("--ignore-user-config", args)
            model_arg = next(x for x in args if x.startswith("model_catalog_json="))
            path = json.loads(model_arg.split("=", 1)[1])
            self.assertIsNone(json.loads(Path(path).read_text())["models"][0]["apply_patch_tool_type"])
            return proc
        with patch.object(fast_codex.subprocess, "run", return_value=Mock(stdout=json.dumps(catalog))), \
                patch.object(fast_codex.subprocess, "Popen", side_effect=spawn), \
                patch.object(fast_codex.selectors, "DefaultSelector", return_value=selector), \
                patch.object(fast_codex.os, "read", side_effect=[stream, b""]), \
                patch.object(fast_codex.os, "killpg") as kill:
            result = INVOKE("codex", {}, "fixed JSON", "low", [], 10, 10000)
        self.assertEqual(result["text"], '{"ok":true}')
        self.assertEqual(len(result["tool_catalog_sha256"]), 64)
        kill.assert_not_called()

    def test_timeout_kills_only_spawned_cli_no_retry(self):
        proc = Mock(pid=765432, stdin=io.BytesIO(), stdout=Mock())
        proc.poll.return_value = None
        selector = Mock()
        selector.get_map.return_value = True
        catalog = {"models": [{"slug": "gpt-6-astra"}]}
        with patch.object(fast_codex.subprocess, "run", return_value=Mock(stdout=json.dumps(catalog))), \
                patch.object(fast_codex.subprocess, "Popen", return_value=proc) as spawn, \
                patch.object(fast_codex.selectors, "DefaultSelector", return_value=selector), \
                patch.object(fast_codex.time, "monotonic", side_effect=[0., 2.]), \
                patch.object(fast_codex.os, "killpg") as kill:
            with self.assertRaisesRegex(FastModelError, "timed out"):
                INVOKE("codex", {}, "fixed JSON", "low", [], 1, 10000)
        self.assertEqual(spawn.call_count, 1)
        kill.assert_called_once_with(765432, fast_codex.signal.SIGKILL)


if __name__ == "__main__":
    unittest.main()
