"""Offline recording requirements and metadata coverage audit, no camera I/O.

This is not an independent recorder process. A complete metadata interval does
not verify video decoding, exposure times, task success, or safe settling.
"""
import json
import math
from pathlib import Path, PurePosixPath
import re

from .fast_pipeline import PipelineContractError


_IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,95}")
_VIEWS = ("right_hand", "front", "left_hand")
_KINDS = {"initial_homing", "initial_pose_ready", "task_start", "action",
          "release_complete", "retreat_complete", "stability_verified",
          "controller_exit", "fault", "settled", "unresolved"}
_MOTION = {"action", "release_complete", "retreat_complete"}


def _identifier(value, name):
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise PipelineContractError(name + " must be a bounded identifier")
    return value


def _number(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def _relative(value):
    if not isinstance(value, str) or not value or len(value) > 512 or "\\" in value:
        raise PipelineContractError("Recording path must be a bounded relative path")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value or value == ".":
        raise PipelineContractError("Recording path must stay inside its run root")
    return value


def make_recording_contract(run_id, task_id, *, include_left=False, output_relative=None):
    """Declare future recording, without asserting any current scene or device."""
    _identifier(run_id, "run_id")
    _identifier(task_id, "task_id")
    if type(include_left) is not bool:
        raise PipelineContractError("include_left must be boolean")
    output = _relative(output_relative if output_relative is not None else "recordings/" + run_id)
    return {
        "schema_version": "task_recording_contract_v1", "run_id": run_id, "task_id": task_id,
        "output_relative": output, "overwrite": False,
        "required_views": ["right_hand", "front"], "optional_views": ["left_hand"],
        "requested_local_views": list(_VIEWS if include_left else _VIEWS[:2]),
        "model_view_selection_affects_recording": False,
        "exclude_initial_homing": True,
        "start_policy": "after_initial_pose_ready_before_first_task_action",
        "normal_end_policy": "after_retreat_and_stability_verification",
        "exception_end_policy": "continue_until_settled_or_explicit_unresolved",
        "clock": "host_receipt_wall_clock_seconds", "max_interframe_gap_s": 0.25,
        "paths": {"report": output + "/report.json", "frames": output + "/frames.jsonl",
                  "videos": {view: output + "/" + view + ".avi" for view in _VIEWS}},
        "implementation": "offline_contract_and_metadata_audit_only",
        "existing_worker_constraint": "Current continuous worker requires all three cameras; a two-view adapter is not implemented here.",
        "recorder_started": False, "execution_available": False,
    }


def _validate_contract(contract):
    if not isinstance(contract, dict):
        raise PipelineContractError("Recording contract must be an object")
    requested = contract.get("requested_local_views")
    if not isinstance(requested, list):
        raise PipelineContractError("Recording contract needs requested_local_views")
    expected = make_recording_contract(contract.get("run_id"), contract.get("task_id"),
                                      include_left="left_hand" in requested,
                                      output_relative=contract.get("output_relative"))
    if contract != expected:
        raise PipelineContractError("Recording contract has altered or unknown fields")


def _contained(root, relative):
    root = Path(root).resolve()
    path = root / _relative(relative)
    try:
        path.resolve().relative_to(root)
    except (OSError, ValueError, RuntimeError) as exc:
        raise PipelineContractError("Recording path escapes its root") from exc
    return path


def reserve_recording_directory(contract, root):
    """Reserve a fresh local directory atomically; no worker is started.

    An existing directory is refused, even if empty. This helper does not grant
    device ownership or replace a future recording supervisor's lifecycle.
    """
    _validate_contract(contract)
    path = _contained(root, contract["output_relative"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path = _contained(root, contract["output_relative"])
    path.mkdir(exist_ok=False)
    with (path / "contract.json").open("x", encoding="utf-8") as stream:
        json.dump(contract, stream, ensure_ascii=False, indent=2, allow_nan=False)
    return {"output_relative": contract["output_relative"], "reserved": True,
            "recorder_started": False, "execution_available": False}


def _events(contract, events):
    if not isinstance(events, list):
        raise PipelineContractError("Recording events must be a list")
    seen, previous = set(), -1
    for event in events:
        if not isinstance(event, dict):
            raise PipelineContractError("Recording event must be an object")
        fields = {"id", "run_id", "task_id", "kind", "at", "reference"}
        if event.get("kind") == "unresolved":
            fields.add("reason")
        if set(event) != fields or event.get("kind") not in _KINDS:
            raise PipelineContractError("Recording event has missing or unknown fields")
        _identifier(event["id"], "event id")
        if event["id"] in seen or any(event[key] != contract[key] for key in ("run_id", "task_id")):
            raise PipelineContractError("Duplicate event or mismatched recording run/task")
        if not _number(event["at"]) or event["at"] < previous:
            raise PipelineContractError("Recording events need ordered finite host times")
        for field in ("reference", "reason"):
            if field in event and (not isinstance(event[field], str) or not event[field].strip() or len(event[field]) > 1024):
                raise PipelineContractError(field + " must be a bounded nonempty string")
        seen.add(event["id"])
        previous = event["at"]


def _required_interval(events):
    findings = []
    ready = [e for e in events if e["kind"] == "initial_pose_ready"]
    starts = [e for e in events if e["kind"] == "task_start"]
    if len(ready) != 1 or len(starts) != 1:
        return None, None, None, ["missing_or_ambiguous_initial_pose_or_task_start"]
    start = starts[0]["at"]
    if ready[0]["at"] > start:
        findings.append("task_started_before_initial_pose_ready")
    if any(e["at"] < start for e in events if e["kind"] in _MOTION):
        findings.append("task_action_precedes_recording_start_requirement")
    if any(e["at"] >= ready[0]["at"] for e in events if e["kind"] == "initial_homing"):
        findings.append("initial_homing_overlaps_task_recording_scope")
    last_motion = max([start] + [e["at"] for e in events if e["kind"] in _MOTION])
    last_nonretreat_motion = max([start] + [e["at"] for e in events if e["kind"] in ("action", "release_complete")])
    retreats = [e["at"] for e in events if e["kind"] == "retreat_complete"]
    normal = [e for e in events if e["kind"] == "stability_verified" and
              e["at"] > last_motion and any(last_nonretreat_motion < t < e["at"] for t in retreats)]
    last_normal = normal[-1] if normal else None
    interrupts = [e["at"] for e in events if e["kind"] == "fault" or
                  e["kind"] == "controller_exit" and (last_normal is None or e["at"] <= last_normal["at"])]
    latest_interrupt = max(interrupts, default=-1)
    exceptional = bool(interrupts) or any(e["kind"] == "unresolved" for e in events)
    # A normal completion needs retreat plus stability; a bare 'settled' does
    # not replace those stages. After an interruption, old stability is not a
    # substitute for an explicitly newer settled/unresolved lifecycle receipt.
    endpoints = ([e for e in events if e["kind"] in ("settled", "unresolved") and
                  e["at"] > max(start, last_motion, latest_interrupt)] if exceptional else
                 ([last_normal] if last_normal is not None else []))
    if not endpoints:
        return ready[0]["at"], start, None, findings + ["no_covered_terminal_or_explicit_unresolved_event"]
    end = max(endpoints, key=lambda e: e["at"])
    return ready[0]["at"], start, end, findings


def _audit(contract, events, report, frames):
    _validate_contract(contract)
    _events(contract, events)
    if report is not None and not isinstance(report, dict):
        raise PipelineContractError("Recording report must be an object or null")
    report = report or {}
    ready, start, terminal, lifecycle_findings = _required_interval(events)
    findings, unknown = list(lifecycle_findings), set()
    end = terminal["at"] if terminal is not None else None
    binding = all(report.get(key) == contract[key] for key in ("run_id", "task_id"))
    if not binding:
        if any(key in report and report[key] != contract[key] for key in ("run_id", "task_id")):
            findings.append("report_run_or_task_mismatch")
        else:
            unknown.add("report_missing_run_task_binding")
    stats = {view: {"count": 0, "first_at": None, "last_at": None,
                    "max_gap_s": 0.0, "metadata_unknown": False, "problems": set()}
             for view in contract["requested_local_views"]}
    if frames is None:
        unknown.add("frame_metadata_missing")
        frames = ()
    for frame in frames:
        if not isinstance(frame, dict):
            unknown.add("malformed_frame_metadata")
            continue
        view = frame.get("name")
        if view not in stats:
            continue
        item = stats[view]
        index = item["count"]
        item["count"] += 1
        if any(key in frame and frame[key] != contract[key] for key in ("run_id", "task_id")):
            item["problems"].add("frame_run_or_task_mismatch")
        stamp = frame.get("host_received_at")
        if not _number(stamp) or frame.get("timestamp_kind") != "host_receipt_wall_clock":
            item["metadata_unknown"] = True
            continue
        frame_index = frame.get("video_frame_index")
        if type(frame_index) is not int:
            item["metadata_unknown"] = True
        elif frame_index != index:
            item["problems"].add("noncontiguous_video_frame_index")
        last = item["last_at"]
        if last is not None:
            gap = stamp - last
            if gap <= 0:
                item["problems"].add("nonmonotonic_host_frame_time")
            item["max_gap_s"] = max(item["max_gap_s"], gap)
        if item["first_at"] is None:
            item["first_at"] = stamp
        item["last_at"] = stamp
    counts = report.get("frames_recorded", {})
    cameras = report.get("requested_cameras", {})
    counts = counts if isinstance(counts, dict) else {}
    cameras = cameras if isinstance(cameras, dict) else {}
    report_start, report_end = report.get("started_at"), report.get("finished_at")
    for view, item in stats.items():
        if view not in cameras:
            unknown.add(view + ":requested_camera_metadata_missing")
        expected = counts.get(view)
        if type(expected) is not int or expected < 0:
            unknown.add(view + ":reported_frame_count_unknown")
        elif expected != item["count"]:
            item["problems"].add("report_frame_count_mismatch")
        first, last = item["first_at"], item["last_at"]
        if first is None or last is None or item["metadata_unknown"]:
            unknown.add(view + ":frame_timing_or_index_unknown")
        if first is not None:
            if ready is not None and first < ready:
                item["problems"].add("recording_precedes_initial_pose_ready")
            if start is not None and first > start:
                item["problems"].add("recording_missed_task_start")
            if _number(report_start) and first < report_start:
                item["problems"].add("frame_precedes_report_start")
        if last is not None:
            if end is not None and last < end:
                item["problems"].add("recording_ended_before_required_terminal")
            if _number(report_end) and last > report_end:
                item["problems"].add("frame_after_report_finish")
        if item["max_gap_s"] > contract["max_interframe_gap_s"]:
            item["problems"].add("frame_time_gap_exceeds_contract")
        item["problems"] = sorted(set(item["problems"]))
        findings.extend(view + ":" + problem for problem in item["problems"])
    if not _number(report_start) or not _number(report_end):
        unknown.add("report_time_bounds_unknown")
    elif report_end < report_start:
        findings.append("report_time_bounds_reversed")
    if report.get("status") not in ("closed", "failed"):
        unknown.add("recording_not_finalized")
    if "cleanup_errors" not in report:
        unknown.add("cleanup_status_unknown")
    elif not isinstance(report["cleanup_errors"], list) or report["cleanup_errors"]:
        unknown.add("cleanup_not_clean")
    if report.get("failure"):
        unknown.add("worker_reported_failure")
    complete = False if findings else (None if unknown else True)
    return {
        "schema_version": "recording_coverage_result_v1", "run_id": contract["run_id"],
        "task_id": contract["task_id"], "metadata_coverage_complete": complete,
        "coverage_status": "incomplete" if complete is False else "unknown" if complete is None else "metadata_verified",
        "required_interval": {"start_at": start, "end_at": end,
                              "end_condition": terminal["kind"] if terminal else None},
        "report_binding_verified": binding, "views": stats,
        "cleanup_closed": report.get("status") == "closed" and report.get("cleanup_errors") == [],
        "physical_outcome_resolved": None,
        "explicit_unresolved": terminal is not None and terminal["kind"] == "unresolved",
        "video_decode_verified": False, "video_files_checked": False,
        "independent_recorder_supervisor_verified": False,
        "task_success": None, "execution_available": False,
        "findings": sorted(set(findings)), "unknown_reasons": sorted(set(unknown)),
        "limitations": ["Host receipt metadata is not exposure synchronization or video decoding proof.",
                        "Caller-supplied lifecycle references and run labels are not authenticated physical evidence.",
                        "Cleanup success does not prove motion, retreat or stability coverage.",
                        "This module does not fix or supervise a recorder that exits with its controller."],
    }


def audit_recording(document):
    """Audit an explicit metadata document; no files/devices are opened."""
    if (not isinstance(document, dict) or set(document) != {"schema_version", "contract", "events", "report", "frames"}
            or document["schema_version"] != "recording_audit_v1"
            or document["frames"] is not None and not isinstance(document["frames"], list)):
        raise PipelineContractError("Invalid recording audit document")
    return _audit(document["contract"], document["events"], document["report"], document["frames"])


def audit_recording_files(contract, events, root, report_relative, frames_relative):
    """Stream existing report/frame metadata only; never open the AVI files."""
    _validate_contract(contract)
    if report_relative != contract["paths"]["report"] or frames_relative != contract["paths"]["frames"]:
        raise PipelineContractError("Metadata files must match the recording contract paths")
    report_path, frames_path = _contained(root, report_relative), _contained(root, frames_relative)
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else None
    if not frames_path.is_file():
        return _audit(contract, events, report, None)
    with frames_path.open(encoding="utf-8") as stream:
        return _audit(contract, events, report, (json.loads(line) for line in stream if line.strip()))
