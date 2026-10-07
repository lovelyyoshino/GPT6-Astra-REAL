"""Stateless, opt-in Astra proposals. No robot, camera or paid call at import.

Payload construction is available offline. The client has no physical dispatch
authority; a runner must check new feedback and execution gates after a proposal.
"""
import base64
import json
import math
import os
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlparse

from .fast_policy import (compact_controller_state, parse_response, controller_phase_spec,
                          reasoning_effort, requires_explanation,
                          response_schema, select_camera_views, ATOMIC_DECISION_RULE)
from .fast_pipeline import compact_pipeline, decision_stage
from .fast_task_pipeline import pen_phase_contract
from .recording import normalize_usage


class FastModelError(RuntimeError):
    pass


class FastModelConfigurationError(FastModelError):
    pass


class FastModelResponseError(FastModelError):
    pass


def _positive_integer(value, name):
    if type(value) is not int or value <= 0:
        raise FastModelConfigurationError(name + " must be a positive integer")
    return value


def _model_config(config):
    if not isinstance(config, dict) or config.get("model_id") != "gpt-6-astra":
        raise FastModelConfigurationError("Explicit model_id='gpt-6-astra' required")
    if config.get("protocol") != "responses":
        raise FastModelConfigurationError("Explicit protocol='responses' required")
    return _positive_integer(config.get("max_output_tokens", 1200), "max_output_tokens")


def _image_bytes(frame):
    if not isinstance(frame, dict) or not isinstance(frame.get("rgb_path"), (str, Path)):
        raise FastModelConfigurationError("Selected camera requires a local rgb_path")
    path = Path(frame["rgb_path"])
    if not path.is_file():
        raise FastModelConfigurationError("Selected RGB file is missing")
    data = path.read_bytes()
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        mime = "image/png"
    elif data.startswith(b"\xff\xd8\xff"):
        mime = "image/jpeg"
    else:
        raise FastModelConfigurationError("Selected RGB must be a PNG or JPEG file")
    return data, mime


def _is_historical(observation):
    frames = observation.get("cameras", {})
    return bool(observation.get("historical") or observation.get("nonphysical") or
                isinstance(frames, dict) and any(isinstance(f, dict) and f.get("historical")
                                                for f in frames.values()))


def _require_current(config, state, observation, views):
    """Request-time freshness only; this never substitutes for execution checks."""
    age, skew = config.get("max_observation_age_s", 0.8), config.get("max_sensor_skew_s", 0.15)
    for value, name in ((age, "max_observation_age_s"), (skew, "max_sensor_skew_s")):
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise FastModelConfigurationError(name + " must be finite and nonnegative")
    if observation.get("host_receipt_skew_exceeded") is True:
        raise FastModelConfigurationError("Camera observation reports excessive skew")
    times = [state.get("robot_state", {}).get("sampled_at")]
    frames = observation.get("cameras", {})
    for view in views:
        frame = frames.get(view)
        if not isinstance(frame, dict) or frame.get("host_receipt_stale") is True:
            raise FastModelConfigurationError("Missing or stale selected camera metadata")
        times.append(frame.get("host_received_at", frame.get("host_received_at_s", frame.get("timestamp"))))
        if "timestamp" in frame:
            times.append(frame["timestamp"])
    now = time.time()
    if any(type(stamp) not in (int, float) or not math.isfinite(stamp) or
           not 0 <= now - stamp <= age for stamp in times):
        raise FastModelConfigurationError("Camera or robot timestamp is missing, stale or future-dated")
    if max(times) - min(times) > skew:
        raise FastModelConfigurationError("Camera/robot timestamps exceed allowed skew")


def build_fast_payload(config, controller_state, observation):
    """Return an API payload without network, credentials, writes or freshness gates.

    Historical/offline payload inspection is intentional. API permission and
    historical-input checks belong to FastResponsesClient.decide(). Only selected
    RGB bytes and projected controller state leave this boundary, never the full
    observation, detector/depth/calibration output, or a conversation history.
    """
    max_tokens = _model_config(config)
    state = compact_controller_state(controller_state)
    if not isinstance(observation, dict) or not isinstance(observation.get("cameras"), dict):
        raise FastModelConfigurationError("Observation requires camera metadata")
    views = select_camera_views(state)
    effort = reasoning_effort(state)
    if effort not in ("low", "medium", "high", "xhigh"):
        raise FastModelConfigurationError("Unsupported phase reasoning effort")
    exceptional = requires_explanation(state)
    pipeline_id = config.get("pipeline_id", "single_arm_closed_loop_v1")
    packet = {
        "controller_state": state,
        "phase_contract": controller_phase_spec(state),
        "pipeline": compact_pipeline(pipeline_id, stage=decision_stage(pipeline_id)),
        "task_stage": pen_phase_contract(state["phase"]),
        "input_kind": "historical_rgb_with_offline_state" if _is_historical(observation) else "current_rgb_and_state",
        "instruction": (
            ATOMIC_DECISION_RULE + " Return one next action for the current phase using the supplied current RGB views. "
            "Do not plan later phases unless recovery is required. Waypoint chunks are limited "
            "to the current allowed approach phase and visibly clear, no-contact free space. "
            "All visual interpretation and target selection belong to you. RGB is uncalibrated; "
            "do not claim measured object coordinates, depth, clearance or contact forces. "
            "The pose reference is the driver's end reference in right_base, metres/radians. "
            "Do not include rationale or reasoning."
        ),
    }
    if state.get("action_budget", {}).get("allow_waypoint_chunks") is False:
        packet["instruction"] = packet["instruction"].replace(
            "Waypoint chunks are limited to the current allowed approach phase and visibly clear, no-contact free space. ",
            "This live transport accepts one move_eef endpoint per action; do not output move_eef_chunk. ")
        packet["instruction"] += (
            " A move_eef pose is a commanded robot endpoint, not a claim of measured object coordinates. "
            "You may propose one bounded, visually supported exploratory move using current robot state and action_budget. "
            "An observe action only acquires new images; it does not change the robot or camera pose.")
    if "required_effort_parameter_nm" in state.get("action_budget", {}):
        packet["instruction"] += " For gripper actions use exactly the required_effort_parameter_nm in action_budget."
    if exceptional:
        packet["instruction"] += " Include only the required brief exception explanation (1..240 characters)."
    if _is_historical(observation):
        packet["instruction"] += (
            " This is archived RGB with offline controller state, not a live observation. "
            "Images are not consequences of mock actions and cannot establish physical task success."
        )
    content = [{"type": "input_text", "text": json.dumps(packet, ensure_ascii=False,
                                                         separators=(",", ":"), allow_nan=False)}]
    for view in views:
        if view not in observation["cameras"]:
            raise FastModelConfigurationError("Missing selected camera: " + view)
        data, mime = _image_bytes(observation["cameras"][view])
        content.extend([
            {"type": "input_text", "text": "camera=" + view},
            {"type": "input_image", "image_url": "data:" + mime + ";base64," +
             base64.b64encode(data).decode("ascii"), "detail": "high"},
        ])
    return {
        "model": config["model_id"], "store": False,
        "input": [{"role": "user", "content": content}],
        "reasoning": {"effort": effort},
        "text": {"format": {"type": "json_schema", "name": "piper_exception_action" if exceptional else
                             "piper_next_action", "strict": True,
                             "schema": response_schema(require_explanation=exceptional, controller_state=state)}},
        "max_output_tokens": max_tokens,
    }


def _open_request(request, timeout):
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, req, fp, code, msg, headers, newurl):
            raise urllib.error.HTTPError(req.full_url, code, "Redirect refused", headers, fp)
    return urllib.request.build_opener(NoRedirect()).open(request, timeout=timeout)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise FastModelResponseError("Duplicate response JSON key")
        result[key] = value
    return result


def _reject_constant(value):
    raise FastModelResponseError("Non-finite response JSON value")


def _response_text(result):
    if not isinstance(result, dict) or result.get("status") != "completed":
        raise FastModelResponseError("Response is not completed")
    if result.get("error") is not None or result.get("incomplete_details") is not None:
        raise FastModelResponseError("Response reports an error or incomplete output")
    output = result.get("output")
    if not isinstance(output, list):
        raise FastModelResponseError("Response output must be a list")
    messages = []
    for item in output:
        if not isinstance(item, dict):
            raise FastModelResponseError("Invalid response output item")
        if item.get("type") == "message":
            messages.append(item)
        elif item.get("type") != "reasoning":
            raise FastModelResponseError("Unexpected response output type")
        # Provider reasoning is neither interpreted nor persisted.
    if len(messages) != 1:
        raise FastModelResponseError("Exactly one assistant message is required")
    message = messages[0]
    if message.get("role") != "assistant" or message.get("status") != "completed":
        raise FastModelResponseError("Assistant message is not completed")
    content = message.get("content")
    if not isinstance(content, list) or len(content) != 1:
        raise FastModelResponseError("Exactly one output_text part is required")
    part = content[0]
    if not isinstance(part, dict) or part.get("type") != "output_text" or not isinstance(part.get("text"), str):
        raise FastModelResponseError("Response refusal or non-text output")
    return part["text"]


class FastResponsesClient:
    """One request per decision, a cumulative instance budget, and no retries."""
    def __init__(self, config, recorder):
        self.config = dict(config)
        self.recorder = recorder
        self.calls = 0
        self.last_metrics = {}
        self._lock = threading.Lock()

    def decide(self, controller_state, observation):
        if not self._lock.acquire(False):
            raise FastModelError("Concurrent decisions on one client are unsupported")
        try:
            return self._decide(controller_state, observation)
        finally:
            self._lock.release()

    def _decide(self, controller_state, observation):
        metrics = self.last_metrics = {
            "decision_source": "responses_api", "model_request_start": None,
            "model_response_end": None, "agent_decide_s": 0.0, "image_encode_s": 0.0,
            "selected_camera_views": [], "reasoning_effort": None,
            "requested_model": self.config.get("model_id"), "actual_model": None,
            "usage": normalize_usage(None), "model_call_count": self.calls,
            "request_id": None, "error": None,
        }
        _model_config(self.config)
        endpoint = self.config.get("endpoint")
        parsed = urlparse(endpoint or "")
        if not endpoint or parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or parsed.fragment:
            raise FastModelConfigurationError("Explicit HTTPS Responses endpoint without embedded credentials required")
        budget = _positive_integer(self.config.get("max_calls", 8), "max_calls")
        if self.calls >= budget:
            raise FastModelError("Model call budget exhausted")
        timeout = self.config.get("timeout_s", 30)
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise FastModelConfigurationError("timeout_s must be finite and positive")
        max_bytes = _positive_integer(self.config.get("max_response_bytes", 1048576), "max_response_bytes")
        allow_historical = self.config.get("allow_historical", False)
        if type(allow_historical) is not bool:
            raise FastModelConfigurationError("allow_historical must be boolean")
        if not isinstance(observation, dict):
            raise FastModelConfigurationError("Observation must be an object")
        frames = observation.get("cameras", {})
        if not isinstance(frames, dict):
            raise FastModelConfigurationError("Observation requires camera metadata")
        historical = _is_historical(observation)
        if historical and not allow_historical:
            raise FastModelConfigurationError("Historical/nonphysical API input requires allow_historical=True")
        key_name = self.config.get("api_key_env", "OPENAI_API_KEY")
        if not isinstance(key_name, str) or not key_name:
            raise FastModelConfigurationError("api_key_env must name a credential environment variable")
        key = os.environ.get(key_name)
        if not key or not key.strip():
            raise FastModelConfigurationError("Configured API credential environment variable is unset")
        if not historical:
            state = compact_controller_state(controller_state)
            _require_current(self.config, state, observation, select_camera_views(state))
        encode_started = time.monotonic()
        try:
            payload = build_fast_payload(self.config, controller_state, observation)
            compact = json.loads(payload["input"][0]["content"][0]["text"])
            views = list(select_camera_views(compact["controller_state"]))
            exceptional = requires_explanation(compact["controller_state"])
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
            request = urllib.request.Request(endpoint, data=body, method="POST",
                                             headers={"Content-Type": "application/json", "Authorization": "Bearer " + key})
            metrics.update(selected_camera_views=views, reasoning_effort=payload["reasoning"]["effort"],
                           payload_bytes=len(body), require_explanation=exceptional)
            audit = {"call_index": self.calls + 1, "requested_model": payload["model"],
                     "reasoning_effort": metrics["reasoning_effort"], "compact_text": compact,
                     "selected_camera_views": views, "images": [
                         {"camera": view, "rgb_path": str(Path(frames[view]["rgb_path"]).resolve()),
                          "detail": "high", "file_size_bytes": Path(frames[view]["rgb_path"]).stat().st_size}
                         for view in views], "schema": payload["text"]["format"],
                     "stateless": True, "historical_input": bool(historical)}
        finally:
            metrics["image_encode_s"] = time.monotonic() - encode_started
        # An input-audit write failure prevents the HTTP request. No base64,
        # credential, full observation, or provider reasoning is persisted.
        input_name = "fast_model_input_%04d.json" % (self.calls + 1)
        self.recorder._write_json(input_name, audit)
        metrics["input_packet_path"] = str(self.recorder.run_dir / input_name)
        if not historical:
            _require_current(self.config, compact["controller_state"], observation, views)
        self.calls += 1
        metrics["model_call_count"] = self.calls
        metrics["model_request_start"] = time.time()
        request_started = time.monotonic()
        usage, actual_model, error = None, None, None
        try:
            with _open_request(request, timeout=timeout) as response:
                raw = response.read(max_bytes + 1)
            if len(raw) > max_bytes:
                raise FastModelResponseError("Response exceeds configured byte limit")
            result = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
            if isinstance(result, dict):
                usage = result.get("usage")
                actual_model = result.get("model")
                metrics["request_id"] = result.get("id") if isinstance(result.get("id"), str) else None
            if actual_model != self.config["model_id"]:
                raise FastModelResponseError("Response model does not match the explicitly requested model")
            decision = parse_response(_response_text(result), require_explanation=exceptional)
            if decision.phase != compact["controller_state"]["phase"]:
                raise FastModelResponseError("Response phase does not match the current controller phase")
            return decision.to_dict()
        except urllib.error.HTTPError as exc:
            error = "http_error:" + str(exc.code)
            raise FastModelError("Model HTTP error " + str(exc.code) + "; no retry was made") from None
        except urllib.error.URLError:
            error = "transport_error"
            raise FastModelError("Model transport failed; no retry was made") from None
        except Exception as exc:
            error = type(exc).__name__
            raise
        finally:
            metrics["model_response_end"] = time.time()
            metrics["agent_decide_s"] = time.monotonic() - request_started
            metrics.update(actual_model=actual_model, error=error, usage=normalize_usage(usage))
            metrics.update(metrics["usage"])
            self.recorder.model_call(usage=usage, elapsed_s=metrics["agent_decide_s"],
                                     model_id=actual_model or self.config["model_id"], error=error)
            self.recorder.event("fast_model_metrics", metrics)
