"""Opt-in Responses API client. Endpoint/model are supplied, never guessed."""
import base64
import json
import os
import time
import urllib.request
import urllib.error
from pathlib import Path
from urllib.parse import urlparse
from .observation import require_fresh


class ModelConfigurationError(RuntimeError):
    pass


def build_payload(config, instruction, observation, history=()):
    if not config.get("model_id") or config.get("protocol") != "responses":
        raise ModelConfigurationError("Set model_id and explicit protocol='responses'")
    # Only current images; text history is bounded independently.
    turns = config.get("text_history_turns", 4)
    if type(turns) is not int or turns < 0:
        raise ModelConfigurationError("text_history_turns must be a nonnegative integer")
    content = [{"type": "input_text", "text": json.dumps({
        "instruction": instruction, "observation": observation,
        "recent_text_history": list(history)[-turns:] if turns else [],
        "contract": {
            "allowed_arm": "right", "length": "metres", "quaternion": "wxyz",
            "actions": ["observe", "move_tcp", "gripper", "wait", "stop"],
            "instruction": "Return one JSON action, not a complete task macro. Do not invent coordinates or calibration. stop is not success. Return only brief observation/purpose, never chain of thought.",
            "move_tcp": {"type": "move_tcp", "arm": "right", "pose": {"position_m": "3 numbers in right_base", "orientation_wxyz": "4 numbers, unit quaternion"}, "speed_m_s": "number", "issued_at": "current observation timestamp", "ttl_s": "short positive number", "calibration_version": "observation calibration version"},
            "gripper": {"type": "gripper", "arm": "right", "width_m": "total jaw opening", "issued_at": "current observation timestamp", "ttl_s": "short positive number", "calibration_version": "observation calibration version"}
        }
    }, ensure_ascii=False)}]
    for name, frame in observation.get("cameras", {}).items():
        path = frame.get("rgb_path")
        if path:
            encoded = base64.b64encode(Path(path).read_bytes()).decode("ascii")
            mime = "image/png" if str(path).lower().endswith(".png") else "image/jpeg"
            content.extend([{"type": "input_text", "text": "camera=" + name},
                            {"type": "input_image", "image_url": "data:" + mime + ";base64," + encoded}])
    return {"model": config["model_id"], "store": False,
            "input": [{"role": "user", "content": content}],
            "max_output_tokens": config.get("max_output_tokens", 600)}


def _open_request(request, timeout):
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, req, fp, code, msg, headers, newurl):
            raise urllib.error.HTTPError(req.full_url, code, "Redirect refused", headers, fp)
    return urllib.request.build_opener(NoRedirect()).open(request, timeout=timeout)


class ResponsesClient:
    def __init__(self, config, recorder):
        self.config = config
        self.recorder = recorder
        self.calls = 0

    def decide(self, instruction, observation, history=()):
        endpoint = self.config.get("endpoint")
        parsed = urlparse(endpoint or "")
        if not endpoint or parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password:
            raise ModelConfigurationError("Explicit HTTPS response endpoint required")
        key = os.environ.get(self.config.get("api_key_env", "OPENAI_API_KEY"))
        if not key:
            raise ModelConfigurationError("Configured API credential environment variable is unset")
        if self.calls >= self.config.get("max_calls", 8):
            raise RuntimeError("Model call budget exhausted")
        require_fresh(observation, self.config.get("max_observation_age_s", 0.8),
                      self.config.get("max_sensor_skew_s", 0.15))
        payload = build_payload(self.config, instruction, observation, history)
        request = urllib.request.Request(endpoint, data=json.dumps(payload).encode(), method="POST",
                                         headers={"Content-Type": "application/json", "Authorization": "Bearer " + key})
        self.calls += 1
        started = time.monotonic()
        usage = None
        response_model = None
        error = None
        try:
            with _open_request(request, timeout=self.config.get("timeout_s", 30)) as response:
                result = json.load(response)
            usage = result.get("usage")
            response_model = result.get("model")
            output = "".join(part.get("text", "") for item in result.get("output", [])
                             if item.get("type") == "message" for part in item.get("content", [])
                             if part.get("type") == "output_text")
            # An invalid action still counts as a request that occurred.
            return json.loads(output)
        except urllib.error.HTTPError as exc:
            error = "http_error:" + str(exc.code)
            raise RuntimeError("Model HTTP error " + str(exc.code)) from None
        except urllib.error.URLError:
            error = "transport_error"
            raise RuntimeError("Model transport failed; no retry was made") from None
        except Exception as exc:
            error = type(exc).__name__
            raise
        finally:
            self.recorder.model_call(usage=usage, elapsed_s=time.monotonic()-started,
                                     model_id=response_model or self.config["model_id"], error=error)
