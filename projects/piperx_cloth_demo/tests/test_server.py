"""Protocol and output-boundary tests. Every service is fake; no device calls."""
import base64
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_tools.server import StdioServer, parse_json

SCHEMAS = [{"name": name, "description": "fake offline service",
            "inputSchema": {"type": "object"}, "annotations": {"readOnlyHint": True}}
           for name in ("robot_describe", "robot_observe", "robot_submit_plan")]
INITIALIZE = {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
    "protocolVersion": "2025-06-18", "capabilities": {},
    "clientInfo": {"name": "offline-test", "version": "1"}}}
READY = {"jsonrpc": "2.0", "method": "notifications/initialized"}


class FakeService:
    def __init__(self):
        self.calls, self.result, self.failure = [], {"ok": True, "hardware_access": False}, None

    def call(self, name, arguments):
        print("SDK diagnostic belongs on stderr")
        self.calls.append((name, arguments))
        if self.failure:
            raise RuntimeError(self.failure)
        return self.result


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.service = FakeService()
        self.server = StdioServer(self.service, SCHEMAS, self.root)

    def ready(self):
        self.server.handle(INITIALIZE)
        self.assertIsNone(self.server.handle(READY))

    def tool_call(self, name="robot_describe", arguments=None):
        return self.server.handle({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                                   "params": {"name": name, "arguments": arguments or {}}})

    def test_initialize_lists_fixed_tools_and_ping(self):
        initialized = self.server.handle(INITIALIZE)
        self.assertEqual(initialized["result"]["protocolVersion"], "2025-06-18")
        self.server.handle(READY)
        listed = self.server.handle({"jsonrpc": "2.0", "id": 3, "method": "tools/list"})
        self.assertEqual(listed["result"]["tools"], SCHEMAS)
        self.assertEqual(self.server.handle({"jsonrpc": "2.0", "id": "p", "method": "ping"}),
                         {"jsonrpc": "2.0", "id": "p", "result": {}})

    def test_tool_cannot_execute_before_initialized_notification(self):
        self.assertEqual(self.tool_call()["error"]["code"], -32002)
        self.server.handle(INITIALIZE)
        self.assertEqual(self.tool_call()["error"]["code"], -32002)
        self.assertEqual(self.service.calls, [])

    def test_unknown_method_and_tool_are_protocol_errors(self):
        self.ready()
        response = self.server.handle({"jsonrpc": "2.0", "id": 5, "method": "shell/execute"})
        self.assertEqual(response["error"]["code"], -32601)
        self.assertEqual(self.tool_call("noop")["error"]["code"], -32602)
        self.assertEqual(self.service.calls, [])

    def test_tool_call_notification_never_executes(self):
        self.ready()
        response = self.server.handle({"jsonrpc": "2.0", "method": "tools/call",
                                      "params": {"name": "robot_describe"}})
        self.assertIsNone(response)
        self.assertEqual(self.service.calls, [])

    def test_nonfinite_and_duplicate_json_rejected(self):
        for text in ('{"a":NaN}', '{"a":Infinity}', '{"a":-Infinity}', '{"a":1e999}',
                     '{"a":1,"a":2}'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_json(text)

    def test_malformed_wire_message_does_not_kill_next_ping(self):
        output = io.StringIO()
        self.server.serve(io.StringIO('not json\n{"jsonrpc":"2.0","id":8,"method":"ping"}\n'), output)
        responses = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(responses[0]["error"]["code"], -32700)
        self.assertEqual(responses[1]["result"], {})

    def test_batch_request_is_rejected_without_tool_calls(self):
        response = self.server.handle([INITIALIZE])
        self.assertEqual(response["error"]["code"], -32600)
        self.assertEqual(self.service.calls, [])

    def test_oversized_request_is_drained_before_next_ping(self):
        output = io.StringIO()
        messages = "x" * 100 + '\n{"jsonrpc":"2.0","id":8,"method":"ping"}\n'
        with patch("robot_tools.server.MAX_REQUEST_CHARS", 64):
            self.server.serve(io.StringIO(messages), output)
        responses = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(responses[0]["error"]["code"], -32700)
        self.assertEqual(responses[1]["result"], {})

    def test_invalid_id_and_arguments_never_execute(self):
        self.ready()
        for message in ({"jsonrpc": "2.0", "id": True, "method": "tools/list"},
                        {"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                         "params": {"name": "robot_describe", "arguments": []}}):
            self.assertIn("error", self.server.handle(message))
        self.assertEqual(self.service.calls, [])

    def test_tool_exception_and_blocked_result_are_iserror(self):
        self.ready()
        self.service.result = {"ok": False, "status": "blocked", "reason": "Motion unavailable"}
        self.assertTrue(self.tool_call("robot_submit_plan")["result"]["isError"])
        self.service.failure = "fake failure"
        result = self.tool_call()["result"]
        self.assertTrue(result["isError"])
        self.assertIn("fake failure", result["content"][0]["text"])

    def test_png_content_has_camera_label_and_roundtrips(self):
        self.ready()
        path = self.root / "front.png"
        data = b"\x89PNG\r\n\x1a\n" + b"offline-test-data"
        path.write_bytes(data)
        self.service.result = {"ok": True, "image_paths": [str(path)]}
        result = self.tool_call("robot_observe")["result"]
        self.assertFalse(result["isError"])
        self.assertEqual(result["content"][1]["text"], "Camera view: front")
        self.assertEqual(base64.b64decode(result["content"][2]["data"]), data)
        self.assertEqual(result["content"][2]["mimeType"], "image/png")

    def test_nonproject_image_is_never_read(self):
        self.ready()
        self.service.result = {"ok": True, "image_paths": ["/tmp/untrusted.png"]}
        result = self.tool_call("robot_observe")["result"]
        self.assertTrue(result["isError"])
        self.assertNotIn("image", [block["type"] for block in result["content"]])

    def test_subprocess_stdout_contains_only_protocol(self):
        program = r'''
import os, sys, types
fake = types.ModuleType("robot_tools.service")
fake.TOOL_SCHEMAS = [{"name":"robot_describe", "description":"offline fake", "inputSchema":{"type":"object"}}]
class ToolService:
    def __init__(self, root): print("constructor diagnostic")
    def call(self, name, args):
        print("SDK print diagnostic")
        os.write(1, b"native SDK diagnostic\n")
        return {"ok":True,"hardware_access":False}
fake.ToolService = ToolService
sys.modules["robot_tools.service"] = fake
from robot_tools.server import main
main()
'''
        messages = [INITIALIZE, READY, {"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                    "params": {"name": "robot_describe", "arguments": {}}}]
        result = subprocess.run([sys.executable, "-B", "-c", program],
                                input="".join(json.dumps(m) + "\n" for m in messages),
                                text=True, capture_output=True, timeout=5,
                                cwd=Path(__file__).resolve().parents[1])
        self.assertEqual(result.returncode, 0, result.stderr)
        responses = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(responses), 2)
        self.assertFalse(responses[1]["result"]["isError"])
        self.assertIn("constructor diagnostic", result.stderr)
        self.assertIn("SDK print diagnostic", result.stderr)
        self.assertIn("native SDK diagnostic", result.stderr)

    def test_cli_rejection_returns_failure_exit_code(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run([sys.executable, "-B", "-m", "robot_tools.server",
                                 "--call", "robot_check_execution"],
                                cwd=root, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 1)
        report = json.loads(result.stdout)
        self.assertFalse(report["physical_execution_available"])
        self.assertEqual(report["hardware_commands_sent"], 0)


if __name__ == "__main__":
    unittest.main()
