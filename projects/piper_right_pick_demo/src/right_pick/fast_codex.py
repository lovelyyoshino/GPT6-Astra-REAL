"""Stateless Astra JSON proposals through the user's official Codex login.

This module never reads authentication files and has no robot/camera dependency.
Every invocation uses a new empty cwd and an ephemeral, tool-disabled CLI session.
CLI startup, its fixed system prompt, internal image encoding and transport are
included in agent_decide_s; image_encode_s measures local input preparation only.
"""
import json
import hashlib
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import tempfile
import threading
import time

from .fast_model import (FastModelConfigurationError, FastModelError,
                         FastModelResponseError, _image_bytes, _is_historical,
                         _positive_integer, _reject_constant, _require_current,
                         _unique_object)
from .fast_policy import (compact_controller_state, parse_response, controller_phase_spec,
                          reasoning_effort, requires_explanation,
                          response_schema, select_camera_views, ATOMIC_DECISION_RULE)
from .fast_pipeline import compact_pipeline, decision_stage
from .fast_task_pipeline import pen_phase_contract
from .fast_experience import build_historical_advisories
from .recording import normalize_usage


# Audited against codex-cli 0.160.0. A different binary version is rejected.
CLI_VERSION = "codex-cli 0.160.0"
DISABLED_FEATURES = (
    "shell_tool", "unified_exec", "unified_exec_tty", "shell_snapshot",
    "shell_snapshot_v2", "apps", "enable_mcp_apps", "plugins", "remote_plugin",
    "plugin_sharing", "recommended_plugins", "hooks", "multi_agent",
    "multi_agent_v2", "agent_message_board", "browser_use", "browser_use_external",
    "browser_use_full_cdp_access", "computer_use", "image_generation", "view_image",
    "code_mode", "code_mode_only", "code_mode_host", "code_mode_prewarm",
    "skill_search", "skill_mcp_dependency_install", "sleep_tool", "tool_suggest",
    "tool_call_mcp_elicitation", "request_permissions_tool",
    "default_mode_request_user_input", "standalone_web_search", "goals",
    "in_app_browser", "in_app_chat", "in_app_dictation", "in_app_local_automation",
    "in_app_updates", "realtime_conversation", "workspace_dependencies", "worktrees",
    "daemon_auto_start", "memories", "external_agent_memory_import", "mentions_v2",
    "unbounded_connection_retries", "step_model_switching", "fast_mode",
    "guardian_approval", "guardian_conversation_history_tools", "auth_elicitation",
    "context_management", "send_message_to_user_async",
)


def _environment():
    # Preserve login resolution without examining any auth value/file; drop parent
    # thread/remote-daemon IDs, API-provider overrides, ROS and CAN environment.
    names = ("PATH", "HOME", "USER", "LOGNAME", "LANG", "LC_ALL", "CODEX_HOME",
             "SSL_CERT_FILE", "SSL_CERT_DIR", "REQUESTS_CA_BUNDLE")
    return {name: os.environ[name] for name in names if name in os.environ}


def codex_argv(binary, schema_path, effort, images=(), catalog_path=None):
    """Fixed isolation options; callers cannot supply additional CLI arguments."""
    if effort not in ("low", "medium", "high", "xhigh"):
        raise FastModelConfigurationError("Unsupported reasoning effort")
    argv = [str(binary), "--no-daemon", "--ask-for-approval", "never", "exec",
            "--model", "gpt-6-astra", "--sandbox", "read-only",
            "--skip-git-repo-check", "--ephemeral", "--ignore-user-config",
            "--ignore-rules", "--strict-config", "--json", "--color", "never",
            "--output-schema", str(schema_path)]
    overrides = [
        'model_provider="piper_official_no_retry"', 'model_reasoning_effort=' + json.dumps(effort),
        'model_reasoning_summary="none"', 'web_search="disabled"',
        'project_doc_max_bytes=0', 'project_doc_fallback_filenames=[]',
        'suppress_unstable_features_warning=true',
        'mcp_servers={}', 'apps._default.enabled=false', 'notify=[]',
        # Reserved built-in provider IDs cannot be overridden. This named
        # provider uses the CLI's normal OpenAI authentication/default endpoint;
        # it has no token command, external URL, header or extracted credential.
        'model_providers.piper_official_no_retry.name="OpenAI no retry"',
        'model_providers.piper_official_no_retry.wire_api="responses"',
        'model_providers.piper_official_no_retry.requires_openai_auth=true',
        'model_providers.piper_official_no_retry.request_max_retries=0',
        'model_providers.piper_official_no_retry.stream_max_retries=0',
        'developer_instructions="Return only the requested JSON proposal. Never call any tool. '
        'Do not access files, commands, networks or hardware. Do not provide reasoning."',
        'features.skip_host_skill_discovery=true',
    ]
    overrides.extend("features." + feature + "=false" for feature in DISABLED_FEATURES)
    if catalog_path is not None:
        overrides.append("model_catalog_json=" + json.dumps(str(catalog_path)))
    for override in overrides:
        argv.extend(["-c", override])
    for path in images:
        argv.append("--image=" + str(path))
    return argv + ["-"]


def _usage(raw):
    if not isinstance(raw, dict):
        return None
    return {"input_tokens": raw.get("input_tokens"), "output_tokens": raw.get("output_tokens"),
            "input_tokens_details": {"cached_tokens": raw.get("cached_input_tokens")},
            "output_tokens_details": {"reasoning_tokens": raw.get("reasoning_output_tokens")}}


class _Events:
    """Consume JSONL in memory; never retain reasoning text or arbitrary events."""
    def __init__(self):
        self.final = None
        self.usage = None
        self.completed = False
        self.started = False
        self.thread_id = None
        self.actual_model = None
        self.event_count = 0
        self.event_types = []

    def feed(self, raw):
        event = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
        if not isinstance(event, dict):
            raise FastModelResponseError("Invalid CLI event")
        self.event_count += 1
        kind = event.get("type")
        # Type names only. Never retain arbitrary event contents or reasoning.
        self.event_types.append(str(kind)[:80])
        if self.completed:
            raise FastModelResponseError("CLI event after completed turn")
        if kind == "thread.started":
            if self.thread_id is not None:
                raise FastModelResponseError("Multiple CLI threads")
            self.thread_id = event.get("thread_id")
        elif kind == "turn.started":
            if self.started:
                raise FastModelResponseError("Multiple CLI turns rejected")
            self.started = True
        elif kind == "turn.completed":
            self.completed = True
            self.usage = _usage(event.get("usage"))
            self.actual_model = event.get("model")
            if self.actual_model is not None and self.actual_model != "gpt-6-astra":
                raise FastModelResponseError("Unexpected CLI response model")
        elif kind in ("item.started", "item.updated", "item.completed"):
            item = event.get("item")
            if not isinstance(item, dict):
                raise FastModelResponseError("Invalid CLI item")
            item_type = item.get("type")
            self.event_types.append("item:" + str(item_type)[:80])
            if item_type == "reasoning":
                return  # No reasoning, including summaries, is ever persisted.
            if item_type != "agent_message":
                if item_type == "error":
                    message = item.get("message", "")
                    # Infrastructure diagnostics only, stripped of credential-like values.
                    raise FastModelResponseError("CLI item error: " + _diagnostic(message))
                raise FastModelResponseError("CLI tool/non-message event rejected: " + str(item_type)[:80])
            if kind == "item.completed":
                if self.final is not None or not isinstance(item.get("text"), str):
                    raise FastModelResponseError("Exactly one final CLI message required")
                if item.get("phase") not in (None, "final_answer", "final"):
                    raise FastModelResponseError("Non-final CLI message rejected")
                self.final = item["text"]
        else:
            # Includes refusal, error, turn.failed and unknown tool event formats.
            message = event.get("message", event.get("error", {}))
            if isinstance(message, dict):
                message = message.get("message", message.get("code", ""))
            raise FastModelResponseError("CLI " + str(kind)[:80] + ": " + _diagnostic(message))

    def result(self):
        if not self.started or not self.completed or self.final is None:
            raise FastModelResponseError("CLI response incomplete")
        return {"text": self.final, "usage": self.usage, "actual_model": self.actual_model,
                "thread_id": self.thread_id, "event_count": self.event_count,
                "event_types": self.event_types}


def _diagnostic(message):
    # Error responses, never reasoning or tool output. Avoid raw URLs, bearer or
    # key-like fields even if an upstream diagnostic unexpectedly contains one.
    import re
    text = str(message)
    text = re.sub(r"(?i)Bearer\s+\S+", "Bearer [redacted]", text)
    text = re.sub(r"(?i)(?:api.?key|access.?token|refresh.?token|authorization)\s*[:=]\s*\S+",
                  "[redacted]", text)
    text = re.sub(r"https?://\S+", "[endpoint]", text)
    return text[:300]


def isolated_catalog(raw):
    """Preserve Astra metadata except its tool capabilities, never model identity."""
    if not isinstance(raw, dict) or not isinstance(raw.get("models"), list):
        raise FastModelConfigurationError("Invalid bundled CLI model catalog")
    matches = [m for m in raw["models"] if m.get("slug") == "gpt-6-astra"]
    if len(matches) != 1:
        raise FastModelConfigurationError("Exactly one bundled Astra model required")
    model = dict(matches[0])
    model.update(apply_patch_tool_type=None, experimental_supported_tools=[], tool_mode=None,
                 node_repl_disabled=True, supports_search_tool=False)
    return dict(raw, models=[model])


def _catalog_bytes(binary):
    with tempfile.TemporaryDirectory(prefix="piper_codex_catalog_", dir="/tmp") as cwd:
        result = subprocess.run([binary, "--no-daemon", "debug", "models", "--bundled"],
                                cwd=cwd, env=_environment(), shell=False, capture_output=True,
                                check=True, text=True, timeout=10)
    catalog = isolated_catalog(json.loads(result.stdout))
    return json.dumps(catalog, sort_keys=True, allow_nan=False).encode("utf-8")


def _kill_child(proc):
    # start_new_session gives only this invocation its own process group. Never
    # stop a shared daemon, ROS process, parent Codex, or another decision client.
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    proc.wait(timeout=5)


def invoke_codex(binary, schema, prompt, effort, images, timeout_s, max_bytes,
                 catalog_bytes=None, before_launch=None):
    """One isolated process, no retries; useful also for a fixed-JSON smoke check."""
    with tempfile.TemporaryDirectory(prefix="piper_codex_proposal_", dir="/tmp") as directory:
        cwd = Path(directory) / "empty"
        cwd.mkdir()
        schema_path = Path(directory) / "output_schema.json"
        schema_path.write_text(json.dumps(schema, allow_nan=False), encoding="utf-8")
        # --bundled performs no model request; no authentication data is read by
        # this adapter. Only the CLI's built-in public model metadata is copied.
        catalog_bytes = _catalog_bytes(binary) if catalog_bytes is None else catalog_bytes
        catalog = json.loads(catalog_bytes)
        catalog_path = Path(directory) / "restricted_models.json"
        catalog_path.write_bytes(catalog_bytes)
        argv = codex_argv(binary, schema_path, effort, images, catalog_path)
        if before_launch is not None:
            before_launch()  # Last freshness check after local filesystem preparation.
        proc = subprocess.Popen(argv, cwd=str(cwd), env=_environment(), shell=False,
                                start_new_session=True, stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        events = _Events()
        selector = selectors.DefaultSelector()
        selector.register(proc.stdout, selectors.EVENT_READ)
        deadline = time.monotonic() + timeout_s
        pending, total = b"", 0
        try:
            proc.stdin.write(prompt.encode("utf-8"))
            proc.stdin.close()
            while selector.get_map():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise FastModelError("Codex response timed out; no retry")
                for key, _ in selector.select(min(remaining, 0.25)):
                    chunk = os.read(key.fd, 65536)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    total += len(chunk)
                    if total > max_bytes:
                        raise FastModelResponseError("CLI response exceeds byte limit")
                    pending += chunk
                    while b"\n" in pending:
                        line, pending = pending.split(b"\n", 1)
                        if line.strip():
                            events.feed(line)
            if pending.strip():
                events.feed(pending)
            code = proc.wait(timeout=max(0.001, deadline - time.monotonic()))
            if code != 0:
                raise FastModelError("Codex process failed (exit %s); no retry" % code)
            result = events.result()
            result["tool_catalog_sha256"] = hashlib.sha256(catalog_bytes).hexdigest()
            result["tool_catalog_overrides"] = {key: catalog["models"][0][key] for key in (
                "apply_patch_tool_type", "experimental_supported_tools", "tool_mode",
                "node_repl_disabled", "supports_search_tool")}
            return result
        finally:
            selector.close()
            _kill_child(proc)
            if not proc.stdin.closed:
                proc.stdin.close()
            proc.stdout.close()


def build_codex_packet(controller_state, observation, pipeline="single_arm_closed_loop_v1"):
    state = compact_controller_state(controller_state)
    views = select_camera_views(state)
    packet = {"controller_state": state, "phase_contract": controller_phase_spec(state),
              "pipeline": compact_pipeline(pipeline, stage=decision_stage(pipeline)),
              "task_stage": pen_phase_contract(state["phase"]),
              "image_order": list(views),
              "input_kind": "historical_rgb_with_offline_state" if _is_historical(observation)
              else "current_rgb_and_state",
            "instruction": (
                ATOMIC_DECISION_RULE + " Return one next action for the current phase using only the attached RGB views. "
                  "Do not plan later phases unless recovery is required. Waypoint chunks are only "
                  "for the current allowed approach phase in visibly clear, no-contact free space. "
                  "All visual interpretation belongs to you. RGB is uncalibrated; do not claim "
                  "measured object coordinates, depth, clearance or contact force. "
                  "The pose reference is the driver's "
                  "end reference in right_base, metres/radians. Return JSON only; no tools or rationale.")}
    historical = build_historical_advisories(state, task_id="pen")
    if historical:
        packet["historical_advisories"] = historical
    if state.get("action_budget", {}).get("allow_waypoint_chunks") is False:
        packet["instruction"] = packet["instruction"].replace(
            "Waypoint chunks are only for the current allowed approach phase in visibly clear, no-contact free space. ",
            "This live transport accepts one move_eef endpoint per action; do not output move_eef_chunk. ")
        packet["instruction"] += (
            " A move_eef pose is a commanded robot endpoint, not a claim of measured object coordinates. "
            "You may propose one bounded, visually supported exploratory move using current robot state and action_budget. "
            "An observe action only acquires new images; it does not change the robot or camera pose.")
    if "required_effort_parameter_nm" in state.get("action_budget", {}):
        packet["instruction"] += " For gripper actions use exactly the required_effort_parameter_nm in action_budget."
    if requires_explanation(state):
        packet["instruction"] += " Supply only the required brief exception explanation (1..240 characters)."
    if _is_historical(observation):
        packet["instruction"] += (" Archived RGB is not the result of offline/mock actions and cannot "
                                  "establish physical task success.")
    return packet


class CodexDecisionClient:
    source = "codex_cli"

    def __init__(self, config, recorder):
        self.config = dict(config)
        self.recorder = recorder
        self.calls = 0
        self.last_metrics = {}
        self._lock = threading.Lock()
        self._version_checked = False
        self._prepared_catalog = None
        self._prepared_binary = None

    def prepare(self):
        """Local version/catalog validation before camera capture; no model call."""
        if not self._lock.acquire(False):
            raise FastModelError("Concurrent client preparation unsupported")
        try:
            return self._prepare()
        finally:
            self._lock.release()

    def _prepare(self):
        if self.config.get("model_id") != "gpt-6-astra" or self.config.get("protocol") != "codex_cli":
            raise FastModelConfigurationError("Explicit model_id=gpt-6-astra and protocol=codex_cli required")
        binary = self.config.get("codex_binary", "codex")
        if not isinstance(binary, str) or not binary:
            raise FastModelConfigurationError("codex_binary must be a path or executable name")
        if self._prepared_binary not in (None, binary):
            raise FastModelConfigurationError("Codex binary changed after preparation")
        if not self._version_checked:
            result = subprocess.run([binary, "--version"], shell=False, check=True,
                                    capture_output=True, text=True, timeout=10, env=_environment())
            if result.stdout.strip() != CLI_VERSION:
                raise FastModelConfigurationError("Codex version has not been audited for tool isolation")
            self._version_checked = True
        if self._prepared_catalog is None:
            self._prepared_catalog = _catalog_bytes(binary)
        self._prepared_binary = binary
        return {"prepared": True, "cli_version": CLI_VERSION, "requested_model": "gpt-6-astra",
                "tool_catalog_sha256": hashlib.sha256(self._prepared_catalog).hexdigest()}

    def decide(self, controller_state, observation):
        if not self._lock.acquire(False):
            raise FastModelError("Concurrent decisions unsupported")
        try:
            return self._decide(controller_state, observation)
        finally:
            self._lock.release()

    def _decide(self, controller_state, observation):
        metrics = self.last_metrics = {
            "decision_source": self.source, "model_request_start": None, "model_response_end": None,
            "agent_decide_s": 0.0, "image_encode_s": 0.0, "selected_camera_views": [],
            "reasoning_effort": None, "requested_model": self.config.get("model_id"),
            "actual_model": None, "actual_model_source": "not_reported_by_cli",
            "model_call_count": self.calls, "usage": normalize_usage(None), "error": None,
            "cli_version": CLI_VERSION, "cli_system_prompt_overhead": True,
            "cli_internal_image_encode_s": None, "cost": None,
            "timing_scope": "agent_decide_s includes CLI startup, internal image encoding and response parsing",
        }
        if self.config.get("model_id") != "gpt-6-astra" or self.config.get("protocol") != "codex_cli":
            raise FastModelConfigurationError("Explicit model_id=gpt-6-astra and protocol=codex_cli required")
        if self.calls >= _positive_integer(self.config.get("max_calls", 8), "max_calls"):
            raise FastModelError("Model call budget exhausted")
        timeout = self.config.get("timeout_s", 90)
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise FastModelConfigurationError("timeout_s must be finite and positive")
        max_bytes = _positive_integer(self.config.get("max_response_bytes", 1048576), "max_response_bytes")
        binary = self.config.get("codex_binary", "codex")
        if not isinstance(binary, str) or not binary:
            raise FastModelConfigurationError("codex_binary must be a path or executable name")
        historical_allowed = self.config.get("allow_historical", False)
        if type(historical_allowed) is not bool or not isinstance(observation, dict):
            raise FastModelConfigurationError("Invalid observation or allow_historical flag")
        historical = _is_historical(observation)
        if historical and not historical_allowed:
            raise FastModelConfigurationError("Historical input requires allow_historical=True")
        self._prepare()
        started = time.monotonic()
        try:
            packet = build_codex_packet(controller_state, observation,
                                        self.config.get("pipeline_id", "single_arm_closed_loop_v1"))
            state = packet["controller_state"]
            views = select_camera_views(state)
            exceptional = requires_explanation(state)
            if not historical:
                _require_current(self.config, state, observation, views)
            effort = reasoning_effort(state)
            schema = response_schema(require_explanation=exceptional, controller_state=state)
            frames = observation.get("cameras", {})
            images = []
            for view in views:
                _image_bytes(frames.get(view))  # Validate original local RGB; never resize or base64-log.
                images.append(str(Path(frames[view]["rgb_path"]).resolve()))
            prompt = json.dumps(packet, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
            metrics.update(selected_camera_views=list(views), reasoning_effort=effort,
                           require_explanation=exceptional)
            audit = {"source": self.source, "call_index": self.calls + 1, "compact_text": packet,
                     "schema": schema, "images": [{"camera": v, "rgb_path": p,
                     "original_file": True, "detail": "CLI-managed"} for v, p in zip(views, images)],
                     "requested_model": "gpt-6-astra", "reasoning_effort": effort,
                     "ephemeral": True, "new_session": True, "tools_disabled": list(DISABLED_FEATURES),
                     "project_doc_max_bytes": 0, "historical_input": historical,
                     "cli_system_prompt_overhead": True, "retry_limit": 0}
        finally:
            metrics["image_encode_s"] = time.monotonic() - started
        input_name = "fast_codex_input_%04d.json" % (self.calls + 1)
        self.recorder._write_json(input_name, audit)
        metrics["input_packet_path"] = str(self.recorder.run_dir / input_name)
        if not historical:
            _require_current(self.config, state, observation, views)
        self.calls += 1
        metrics["model_call_count"] = self.calls
        metrics["model_request_start"] = time.time()
        started = time.monotonic()
        usage, error = None, None
        try:
            response = invoke_codex(binary, schema, prompt, effort, images, timeout, max_bytes,
                                     catalog_bytes=self._prepared_catalog,
                                     before_launch=None if historical else lambda: _require_current(
                                         self.config, state, observation, views))
            usage = response["usage"]
            metrics.update(actual_model=response["actual_model"], request_id=response["thread_id"],
                           cli_event_count=response["event_count"])
            metrics.update(tool_catalog_sha256=response.get("tool_catalog_sha256"),
                           tool_catalog_overrides=response.get("tool_catalog_overrides"),
                           cli_event_types=response.get("event_types"))
            if response["actual_model"] is not None:
                metrics["actual_model_source"] = "cli_event"
            # Duplicate keys/nonfinite numbers fail before the policy parser.
            raw = json.loads(response["text"], object_pairs_hook=_unique_object, parse_constant=_reject_constant)
            decision = parse_response(raw, require_explanation=exceptional)
            if decision.phase != state["phase"]:
                raise FastModelResponseError("Response phase does not match current phase")
            result = decision.to_dict()
            self.recorder._write_json("fast_codex_output_%04d.json" % self.calls,
                                      {"decision": result, "source": self.source})
            return result
        except Exception as exc:
            error = type(exc).__name__
            raise
        finally:
            metrics["model_response_end"] = time.time()
            metrics["agent_decide_s"] = time.monotonic() - started
            metrics.update(error=error, usage=normalize_usage(usage))
            metrics.update(metrics["usage"])
            self.recorder.model_call(usage=usage, elapsed_s=metrics["agent_decide_s"],
                                     model_id="gpt-6-astra", error=error)
            self.recorder.event("fast_model_metrics", metrics)
