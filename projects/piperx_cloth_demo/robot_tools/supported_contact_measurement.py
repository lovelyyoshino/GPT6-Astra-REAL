"""Pure evidence for an existing jaw target and newly observed supported contact.

This is not the closing-response classifier. It never infers a new send or a
resolved closing displacement, and cannot certify grip force or loaded motion.
The manager owns old-send/closed-owner and bilateral-contact provenance checks.
"""
import copy
import math
from pathlib import Path

from .feedback_tolerance import joints_within, rotation_tolerance
from .retention_receipt import anchor_deviation, digest, measured_anchor, summarize_retention_trace
from .single_supervised_actions import BOUNDS
from .takeover import LIMITS, SIDES


def _need(value, message):
    if not value:
        raise ValueError(message)


def _number(value):
    _need(type(value) in (int, float) and math.isfinite(value), "Finite existing-contact evidence required")
    return value


def _sha(value):
    _need(type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value),
          "Exact existing-contact SHA256 required")


def _source(value):
    _need(type(value) is dict and set(value) == {"path", "sha256"}, "Exact contact source provenance required")
    path = value["path"]
    _need(type(path) is str and 0 < len(path) <= 4096 and "\x00" not in path and Path(path).is_absolute(),
          "Absolute contact source path required")
    _sha(value["sha256"])


def validate_source(source, arm, now):
    """Validate the typed management envelope; do not read devices or files."""
    _need(type(source) is dict and arm in SIDES, "Explicit supported-contact source and arm required")
    value = source.get("audited_existing_contact")
    fields = {"schema", "arm", "source_receipt_sha256", "failed_event_id", "failed_receipt_sha256",
              "prior_sent_target_m", "failed_sent_at", "failed_finished_at", "audit_proposal_sha256",
              "consumed_probe_count", "remaining_probe_count", "residual_jaw_anchor", "bilateral_contact_source"}
    _need(type(value) is dict and set(value) == fields
          and value["schema"] == "piper_existing_supported_contact_admission_v1" and value["arm"] == arm,
          "Exact existing supported-contact schema and arm required")
    _need(type(value["consumed_probe_count"]) is int and value["consumed_probe_count"] == 3
          and type(value["remaining_probe_count"]) is int and value["remaining_probe_count"] == 0,
          "Existing contact observation must preserve three consumed and zero remaining probes")
    for key in ("source_receipt_sha256", "failed_receipt_sha256", "audit_proposal_sha256"):
        _sha(value[key])
    original = {key: item for key, item in source.items() if key != "audited_existing_contact"}
    _need(digest(original) == value["source_receipt_sha256"], "Original body source receipt changed")
    event = value["failed_event_id"]
    _need(type(event) is str and 0 < len(event) <= 128 and event.strip() == event
          and not any(ord(c) < 32 or ord(c) == 127 for c in event), "Original completed send event required")
    jaw = value["residual_jaw_anchor"]
    _need(type(jaw) is dict and set(jaw) == {"width_m", "observed_at", "source"},
          "Independent residual jaw observation required")
    _source(jaw["source"])
    _source(value["bilateral_contact_source"])
    completed = _number(source.get("candidate_probe", {}).get("completed_at"))
    for number in (now, jaw["width_m"], jaw["observed_at"], value["prior_sent_target_m"],
                   value["failed_sent_at"], value["failed_finished_at"]):
        _number(number)
    _need(0 <= jaw["width_m"] <= .070 and 0 <= value["prior_sent_target_m"] <= .055
          and 0 <= completed < value["failed_sent_at"] < value["failed_finished_at"] < jaw["observed_at"] <= now,
          "Existing target and newer residual observation chronology/width required")
    anchors = {side: copy.deepcopy(source["candidate_measurement"]["anchor"])
               if side == arm else measured_anchor(source["before"][side]) for side in SIDES}
    for anchor in anchors.values():
        _need(type(anchor) is dict and set(anchor) == {"joints_rad", "pose_m_rad", "width_m"},
              "Immutable original body and jaw anchor required")
        for key in ("joints_rad", "pose_m_rad"):
            _need(type(anchor[key]) is list and len(anchor[key]) == 6, "Six original body coordinates required")
            for number in anchor[key]:
                _number(number)
        _need(0 <= _number(anchor["width_m"]) <= .070, "Original jaw width required")
    return copy.deepcopy(value), anchors


def measure(*, arm, source, samples, trace_id, trace_sha256, now, feedback_policy=None):
    """Measure a new zero-TX window without reclassifying the failed closure."""
    admission, anchors = validate_source(source, arm, now)
    target = admission["prior_sent_target_m"]
    _need(type(samples) is list and len(samples) >= 21, "New complete supported-contact window required")
    _need(samples[0]["observed_at_s"] > admission["residual_jaw_anchor"]["observed_at"],
          "Contact window must follow the audited residual observation")
    for sample in samples:
        for side in SIDES:
            state, anchor = sample["arms"][side], anchors[side]
            difference = anchor_deviation(anchor, state)
            _need(joints_within(feedback_policy, side, anchor["joints_rad"], state["joints_rad"])
                  and difference["position_m"] <= BOUNDS["position_span_m"]
                  and difference["rotation_rad"] <= rotation_tolerance(feedback_policy, side),
                  "Supported contact moved from the original body reference: " + side)
            jaw = admission["residual_jaw_anchor"]["width_m"] if side == arm else anchor["width_m"]
            _need(abs(state["gripper"]["width_m"]-jaw) <= BOUNDS["jaw_span_m"],
                  "Supported contact jaw left its independent observed reference: " + side)
    width = samples[-1]["arms"][arm]["gripper"]["width_m"]
    gap = min(sample["arms"][arm]["gripper"]["width_m"]-target for sample in samples)
    _need(gap > LIMITS["gripper_m"], "Existing target lacks a persistent jaw shortfall beyond arrival tolerance")
    anchor = copy.deepcopy(anchors[arm])
    anchor["width_m"] = width  # Only the new jaw observation, never a new body origin.
    measurement = summarize_retention_trace(arm=arm, identity=None, probe_event_id=None,
        trace_id=trace_id, trace_sha256=trace_sha256, original_anchor=anchor,
        samples=samples, now=now, feedback_policy=feedback_policy)
    candidate = {"schema": "piper_existing_supported_contact_candidate_v1", "basis": "existing_target_observation",
        "existing_target_ref": {"event_id": admission["failed_event_id"],
            "receipt_sha256": admission["failed_receipt_sha256"], "sent_at": admission["failed_sent_at"],
            "finished_at": admission["failed_finished_at"], "requested_width_m": target},
        "trace_sha256": trace_sha256, "started_at": measurement["started_at"],
        "completed_at": measurement["ended_at"], "observed_width_m": width, "minimum_target_gap_m": gap,
        "completion": "observation_only", "target_may_remain_active": True}
    return candidate, measurement
