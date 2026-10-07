"""Small stdio MCP adapter; the ToolService owns all device and plan policy.

Wire contract: https://modelcontextprotocol.io/specification/2025-06-18
Only fixed tools are exposed. No shell, file editing, or arbitrary Python tool.
"""
from __future__ import annotations

import argparse
import base64
import contextlib
import io
import json
import math
import os
import sys
from pathlib import Path

PROTOCOL_VERSION = "2025-06-18"
MAX_REQUEST_CHARS = 1_048_576
MAX_IMAGE_BYTES = 16 * 1024 * 1024


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Non-finite JSON numbers are forbidden")
    return number


def _constant(value):
    raise ValueError("Invalid JSON constant: " + value)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def parse_json(value):
    return json.loads(value, parse_constant=_constant, parse_float=_finite_float,
                      object_pairs_hook=_unique_object)


def _encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


class ProtocolError(Exception):
    def __init__(self, code, message):
        self.code, self.message = code, message


def _error(request_id, code, message):
    return {"jsonrpc": "2.0", "id": request_id,
            "error": {"code": code, "message": message}}


@contextlib.contextmanager
def _protocol_stdout():
    """Keep one protocol pipe while routing Python AND native SDK stdout to stderr."""
    original = sys.stdout
    try:
        stdout_fd, stderr_fd = original.fileno(), sys.stderr.fileno()
    except (AttributeError, io.UnsupportedOperation):
        with contextlib.redirect_stdout(sys.stderr):
            yield original
        return
    original.flush()
    output = os.fdopen(os.dup(stdout_fd), "w", encoding="utf-8", buffering=1)
    try:
        os.dup2(stderr_fd, stdout_fd)
        with contextlib.redirect_stdout(sys.stderr):
            yield output
    finally:
        try:
            output.flush()
        finally:
            os.dup2(output.fileno(), stdout_fd)
            output.close()


class StdioServer:
    def __init__(self, service, schemas, root):
        self.service, self.schemas, self.root = service, schemas, Path(root).resolve()
        self.names = {tool["name"] for tool in schemas}
        self.negotiated = self.ready = False

    def _tool_result(self, result):
        if not isinstance(result, dict):
            raise ValueError("ToolService must return a JSON object")
        content = [{"type": "text", "text": _encode(result)}]
        paths = result.get("image_paths", [])
        if not isinstance(paths, list):
            raise ValueError("image_paths must be a list")
        for value in paths:
            if not isinstance(value, str):
                raise ValueError("Image paths must be strings")
            path = Path(value).resolve()
            if not path.is_relative_to(self.root) or path.suffix.lower() != ".png":
                raise ValueError("Only project-local PNG observations may be returned")
            if path.stat().st_size > MAX_IMAGE_BYTES:
                raise ValueError("Observation image exceeds size limit")
            data = path.read_bytes()
            if not data.startswith(b"\x89PNG\r\n\x1a\n"):
                raise ValueError("Observation is not a PNG: " + path.name)
            content.extend([{"type": "text", "text": "Camera view: " + path.stem},
                            {"type": "image", "mimeType": "image/png",
                             "data": base64.b64encode(data).decode("ascii")}])
        failed = (result.get("ok") is False or bool(result.get("isError"))
                  or bool(result.get("error"))
                  or result.get("status") in ("error", "blocked", "failed", "rejected"))
        return {"content": content, "isError": failed}

    def handle(self, message):
        request_id, notification = None, False
        try:
            if not isinstance(message, dict) or message.get("jsonrpc") != "2.0":
                raise ProtocolError(-32600, "Expected a JSON-RPC 2.0 object")
            notification = "id" not in message
            if not notification:
                request_id = message["id"]
                if type(request_id) not in (int, str):
                    request_id = None
                    raise ProtocolError(-32600, "Request id must be an integer or string")
            method, params = message.get("method"), message.get("params", {})
            if not isinstance(method, str) or not isinstance(params, dict):
                raise ProtocolError(-32600, "method must be a string; params must be an object")
            if method == "notifications/initialized":
                if not notification or not self.negotiated:
                    raise ProtocolError(-32600, "Unexpected initialized notification")
                self.ready = True
                return None
            if notification:
                # Never execute a tool without a request id and a return channel.
                raise ProtocolError(-32600, "Unsupported notification: " + method)
            if method == "initialize":
                info = params.get("clientInfo")
                if (self.negotiated or not isinstance(params.get("protocolVersion"), str)
                        or not isinstance(params.get("capabilities"), dict)
                        or not isinstance(info, dict)
                        or not isinstance(info.get("name"), str)
                        or not isinstance(info.get("version"), str)):
                    raise ProtocolError(-32602, "Invalid or repeated initialize request")
                self.negotiated = True
                result = {"protocolVersion": PROTOCOL_VERSION,
                          "capabilities": {"tools": {"listChanged": False}},
                          "serverInfo": {"name": "piperx-cloth-tools", "version": "0.1.0"}}
            elif method == "ping":
                result = {}
            elif method not in ("tools/list", "tools/call"):
                raise ProtocolError(-32601, "Unknown method: " + method)
            elif not self.ready:
                raise ProtocolError(-32002, "Initialize and send notifications/initialized first")
            elif method == "tools/list":
                if params.get("cursor") is not None:
                    raise ProtocolError(-32602, "This fixed tool list has no cursor")
                result = {"tools": self.schemas}
            else:
                name, arguments = params.get("name"), params.get("arguments", {})
                if not isinstance(name, str) or name not in self.names:
                    raise ProtocolError(-32602, "Unknown tool name")
                if not isinstance(arguments, dict):
                    raise ProtocolError(-32602, "Tool arguments must be an object")
                try:
                    with contextlib.redirect_stdout(sys.stderr):
                        result = self._tool_result(self.service.call(name, arguments))
                except Exception as exc:
                    result = {"content": [{"type": "text", "text": _encode({
                        "ok": False, "error": f"{type(exc).__name__}: {exc}"})}], "isError": True}
            return {"jsonrpc": "2.0", "id": request_id, "result": result}
        except ProtocolError as exc:
            if notification:
                print(exc.message, file=sys.stderr)
                return None
            return _error(request_id, exc.code, exc.message)

    def serve(self, input_stream, output_stream):
        while True:
            line = input_stream.readline(MAX_REQUEST_CHARS + 1)
            if not line:
                return
            try:
                if len(line) > MAX_REQUEST_CHARS:
                    while line and not line.endswith("\n"):
                        line = input_stream.readline(MAX_REQUEST_CHARS + 1)
                    raise ValueError("Request exceeds size limit")
                response = self.handle(parse_json(line))
            except (ValueError, RecursionError) as exc:
                response = _error(None, -32700, str(exc))
            if response is not None:
                output_stream.write(_encode(response) + "\n")
                output_stream.flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--call", help="Call a fixed tool once and print local JSON")
    parser.add_argument("--arguments", default="{}", help="JSON object for --call")
    args = parser.parse_args(argv)
    with _protocol_stdout() as protocol_output:
        from .service import TOOL_SCHEMAS, ToolService
        service = ToolService(args.root)
        if args.call:
            try:
                arguments = parse_json(args.arguments)
                if not isinstance(arguments, dict) or args.call not in {t["name"] for t in TOOL_SCHEMAS}:
                    raise ValueError("Unknown tool or non-object arguments")
                result = service.call(args.call, arguments)
                encoded = _encode(result)
                exit_code = 1 if result.get("ok") is False or result.get("error") else 0
            except Exception as exc:
                encoded = _encode({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
                exit_code = 2
            protocol_output.write(encoded + "\n")
            protocol_output.flush()
            return exit_code
        else:
            StdioServer(service, TOOL_SCHEMAS, args.root).serve(sys.stdin, protocol_output)


if __name__ == "__main__":
    raise SystemExit(main())
