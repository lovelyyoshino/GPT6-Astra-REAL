"""Per-run audit records, provider usage and evidence-based outcome reporting."""
import json
import math
import os
from pathlib import Path
import re
import time
import uuid
from dataclasses import asdict, is_dataclass


_SECRET_KEY = re.compile(r"(?:api.?key|authorization|password|secret|credential|access.?token|refresh.?token|private.?key|^token$)", re.I)
_THOUGHT_KEYS = {"chain_of_thought", "reasoning_content", "scratchpad", "internal_thought", "analysis", "thinking", "reasoning"}
_EVIDENCE_STAGES = ("lift", "transport", "release", "stable")


def _usage_integer(value):
    return value if type(value) is int and value >= 0 else None


def normalize_usage(usage):
    """Cached/reasoning are subsets, never added again to input/output totals."""
    usage = usage if isinstance(usage, dict) else {}
    input_details = usage.get("input_tokens_details", usage.get("prompt_tokens_details")) or {}
    output_details = usage.get("output_tokens_details", usage.get("completion_tokens_details")) or {}
    input_details = input_details if isinstance(input_details, dict) else {}
    output_details = output_details if isinstance(output_details, dict) else {}
    return {
        "input_tokens": _usage_integer(usage.get("input_tokens", usage.get("prompt_tokens"))),
        "output_tokens": _usage_integer(usage.get("output_tokens", usage.get("completion_tokens"))),
        "cached_input_tokens": _usage_integer(input_details.get("cached_tokens")),
        "reasoning_output_tokens": _usage_integer(output_details.get("reasoning_tokens")),
    }


class Recorder:
    def __init__(self, root, config, instruction, model_id=None, mode="physical", secret_values=()):
        self.started_at = time.time()
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(self.started_at))
        self.run_dir = Path(root) / (stamp + "_" + uuid.uuid4().hex[:12])
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.mode = mode
        self.model_id = model_id
        self.model_calls = []
        self.events_count = 0
        self._finished = False
        self._secrets = set(str(s) for s in secret_values if s and len(str(s)) >= 4)
        self._collect_secrets(config)
        for key, value in os.environ.items():
            if _SECRET_KEY.search(key) and len(value) >= 8:
                self._secrets.add(value)
        self._write_json("config.json", config)
        (self.run_dir / "instruction.txt").write_text(self._sanitize(str(instruction)), encoding="utf-8")
        self.event("run_started", {"mode": mode, "model_id": model_id,
                                   "nonphysical": mode != "physical"})

    def _collect_secrets(self, value):
        if isinstance(value, dict):
            for key, item in value.items():
                if _SECRET_KEY.search(str(key)) and isinstance(item, str) and len(item) >= 4:
                    self._secrets.add(item)
                self._collect_secrets(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                self._collect_secrets(item)

    def _sanitize(self, value):
        if is_dataclass(value):
            value = asdict(value)
        if isinstance(value, dict):
            return {str(k): ("[REDACTED]" if _SECRET_KEY.search(str(k)) else self._sanitize(v))
                    for k, v in value.items() if str(k).lower() not in _THOUGHT_KEYS}
        if isinstance(value, (list, tuple)):
            return [self._sanitize(item) for item in value]
        if isinstance(value, str):
            for secret in sorted(self._secrets, key=len, reverse=True):
                value = value.replace(secret, "[REDACTED]")
            value = re.sub(r"(?i)\bBearer\s+[^\s,;\"']+", "Bearer [REDACTED]", value)
            value = re.sub(r"(?i)([?&](?:api[_-]?key|token|secret|password)=)[^&\s]+", r"\1[REDACTED]", value)
            return value
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    def _write_json(self, name, data):
        path = self.run_dir / name
        path.write_text(json.dumps(self._sanitize(data), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")

    def event(self, event_type, payload=None, **kwargs):
        if self._finished:
            raise RuntimeError("run is already finalized")
        data = dict(payload or {})
        data.update(kwargs)
        self._collect_secrets(data)
        event = self._sanitize({"event": event_type, "at": time.time(), "data": data})
        with (self.run_dir / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        self.events_count += 1
        return event

    def model_call(self, usage=None, elapsed_s=None, model_id=None, error=None):
        if elapsed_s is not None and (isinstance(elapsed_s, bool) or not isinstance(elapsed_s, (int, float))
                                     or not math.isfinite(elapsed_s) or elapsed_s < 0):
            raise ValueError("elapsed_s must be nonnegative or unknown")
        row = {"call_index": len(self.model_calls) + 1, "model_id": model_id or self.model_id,
               "elapsed_s": elapsed_s, "usage": normalize_usage(usage), "error": error}
        self.event("model_call", row)
        self.model_calls.append(row)
        return row

    def usage_summary(self):
        count = len(self.model_calls)
        result = {"calls": count, "token_subset_semantics":
                  "cached_input_tokens and reasoning_output_tokens are subsets, not additive totals"}
        for field in normalize_usage(None):
            known = [r["usage"][field] for r in self.model_calls if r["usage"][field] is not None]
            result[field] = {"total": sum(known) if len(known) == count and count > 0 else None,
                             "known_subtotal": sum(known) if known else None,
                             "known_calls": len(known), "total_calls": count,
                             "coverage": len(known) / count if count else None}
        waits = [r["elapsed_s"] for r in self.model_calls if r["elapsed_s"] is not None]
        result["interface_wait_time_s"] = sum(waits) if len(waits) == count and count else None
        return result

    def finish(self, evidence=(), termination_reason="stopped", failure=None, metrics=None,
               outcome=None, reason=None, physical_attempts=None):
        """Evidence needs stage, confirmed=True, source, reference, and epoch at."""
        if self._finished:
            raise RuntimeError("run is already finalized")
        if outcome not in (None, "success", "failure", "blocked", "not_evaluated"):
            raise ValueError("unknown outcome")
        if physical_attempts is not None and (type(physical_attempts) is not int or physical_attempts < 0):
            raise ValueError("physical_attempts must be a nonnegative integer or unknown")
        if reason is not None:
            termination_reason = str(reason)
        evidence = list(evidence)
        confirmed = {}
        allowed_sources = {"manual_review", "vision_measurement", "sensor_measurement"}
        for row in evidence:
            if not isinstance(row, dict):
                continue
            stamp = row.get("at")
            if (row.get("stage") in _EVIDENCE_STAGES and row.get("confirmed") is True
                    and row.get("source") in allowed_sources
                    and isinstance(row.get("reference"), str) and row["reference"].strip()
                    and type(stamp) in (int, float) and math.isfinite(stamp)
                    and self.started_at <= stamp <= time.time()):
                confirmed[row["stage"]] = row
        order = [confirmed[s]["at"] for s in _EVIDENCE_STAGES if s in confirmed]
        ordered = len(order) == 4 and all(a < b for a, b in zip(order, order[1:]))
        success = self.mode == "physical" and ordered and failure is None
        if outcome == "success" and not success:
            raise ValueError("success requires ordered physical lift, transport, release, stable evidence")
        if outcome == "failure" and not failure:
            raise ValueError("failure requires a task failure reason; infrastructure blocks are not task failure")
        outcome = outcome or ("success" if success else ("failure" if failure else "not_evaluated"))
        report = {"mode": self.mode, "nonphysical": self.mode != "physical",
                  "outcome": outcome, "success": (True if outcome == "success" else
                                                   (False if outcome == "failure" else None)),
                  "physical_attempts": physical_attempts,
                  "termination_reason": termination_reason, "failure": failure,
                  "started_at": self.started_at, "finished_at": time.time(),
                  "real_elapsed_s": time.time() - self.started_at,
                  "stage_progress": {s: s in confirmed for s in _EVIDENCE_STAGES},
                  "evidence_order_valid": ordered, "evidence": evidence,
                  "usage": self.usage_summary(), "metrics": metrics or {},
                  "physical_motion_time_s": (metrics or {}).get("physical_motion_time_s"),
                  "limitations": ["Replay never establishes physical task success.",
                                   "Stop and TCP arrival alone are not task-success evidence."]}
        self.event("run_finished", {"outcome": outcome, "termination_reason": termination_reason})
        report["events_count"] = self.events_count
        self._write_json("report.json", report)
        self._finished = True
        return self._sanitize(report)
