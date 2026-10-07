"""Generic staged execution and feedback, with no perception or task planner.

The real backend refuses commissioning until a holding stop is validated.
Tests use a fake backend; that is a controller-contract test, not simulation.
"""
from __future__ import annotations

import fcntl
import json
import math
import os
import time
from pathlib import Path

from .arms import control_health


LIMITS = {"poll_s": 0.05, "stable_s": 0.3, "startup_timeout_s": 3.0, "stage_timeout_s": 30.0,
          "total_timeout_s": 180.0, "position_tolerance_m": 0.005,
          "rotation_tolerance_rad": 0.05, "joint_drift_rad": 0.03,
          "width_tolerance_m": 0.002, "max_dispatch_skew_s": 0.1}


class ExecutionFault(RuntimeError):
    pass


class ExclusiveExecution:
    """Cross-process advisory lock for all entrypoints in this platform."""
    def __init__(self, runs: Path):
        self.path = Path(runs) / "execution.lock"
        self.stream = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("a+")
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.stream.close()
            raise ExecutionFault("Another platform execution owns both arms")
        return self

    def __exit__(self, *args):
        self.stream.close()


class Journal:
    """Intent is durably written BEFORE a potentially partial CAN operation."""
    def __init__(self, directory: Path):
        self.directory = Path(directory)
        self.path = self.directory / "events.jsonl"

    def append(self, event, **data):
        record = {"event": event, "unix_s": time.time(), **data}
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())


def readiness(profile, backend):
    reasons = list(backend.commissioning_errors())
    for field, value in profile.get("verification", {}).items():
        if value is not True:
            reasons.append({"code": "unverified_profile", "field": field})
    required = {"physical_models_and_firmware", "base_frames_and_table",
                "tool_and_camera_geometry", "hold_stop_and_recovery", "gripper_units_and_limits"}
    for field in required - set(profile.get("verification", {})):
        reasons.append({"code": "missing_verification", "field": field})
    hold_path = profile.get("legacy_hold_record")
    try:
        hold = json.loads(Path(hold_path).read_text(encoding="utf-8"))
        if hold.get("status") != "resolved_with_verified_stop":
            reasons.append({"code": "legacy_incident_unresolved", "record": hold_path})
    except (OSError, TypeError, ValueError):
        reasons.append({"code": "legacy_incident_resolution_missing", "record": hold_path})
    # A record/boolean is never enough: the backend independently rejects an
    # unsupported stop, even if these configuration declarations are edited.
    return reasons


def check_feedback(states):
    if not isinstance(states, dict) or set(states.get("arms", {})) != {"left", "right"}:
        raise ExecutionFault("Both arm snapshots are required")
    stamps = []
    for side, state in states["arms"].items():
        health = control_health(state)
        if not health["healthy"]:
            raise ExecutionFault(side + " feedback rejected: " + json.dumps(health["reasons"]))
        stamps.extend(state["fragment_timestamps_s"].values())
        stamps.append(state["gripper"]["timestamp"])
    if not stamps or max(stamps) - min(stamps) > 0.1:
        raise ExecutionFault("Cross-arm feedback skew exceeds 0.1 s")


def _pose_close(backend, actual, expected):
    pos, rot = backend.pose_error(actual, expected)
    return pos <= LIMITS["position_tolerance_m"] and rot <= LIMITS["rotation_tolerance_rad"]


def _anchor_check(backend, current, anchor, sides=("left", "right"), check_gripper=True):
    """Check all joints too: identical flange poses can have different arm shapes."""
    for side in sides:
        now, expected = current["arms"][side], anchor["arms"][side]
        if not _pose_close(backend, now["pose_m_rad"], expected["pose_m_rad"]):
            raise ExecutionFault(side + " flange drifted from the reviewed/stationary start")
        a, b = now["joints_rad"], expected["joints_rad"]
        if not isinstance(b, list) or len(b) != 6 or any(type(q) not in (float, int) or not math.isfinite(q) for q in b):
            raise ExecutionFault(side + " reviewed joint snapshot is invalid")
        if max(abs(x-y) for x, y in zip(a, b)) > LIMITS["joint_drift_rad"]:
            raise ExecutionFault(side + " joint configuration drifted")
        old_width = expected.get("gripper", {}).get("width_m")
        if check_gripper and (old_width is None or abs(now["gripper"]["width_m"] - old_width) > LIMITS["width_tolerance_m"]):
            raise ExecutionFault(side + " gripper start changed or was not observed")


def _stamp_vector(states, sides=("left", "right")):
    return tuple(stamp for side in sides for _, stamp in
                 sorted(states["arms"][side]["fragment_timestamps_s"].items()))


def _advanced(current, old):
    return old is not None and len(current) == len(old) and all(a > b for a, b in zip(current, old))


def _all_new(current, after):
    stamps = list(current["fragment_timestamps_s"].values()) + [current["gripper"]["timestamp"]]
    return min(stamps) > after


def _goal_reached(backend, state, target):
    if target.get("action", "move") == "move":
        return (state["arm_status"]["motion_status"] == 0
                and _pose_close(backend, state["pose_m_rad"], target["pose_m_rad"]))
    # A stable, exact width is the only automatic gripper success criterion in
    # this version. Contact or a grasp needs visual assessment, never inferred.
    return (abs(state["gripper"]["width_m"] - target["gripper_width_m"])
            <= LIMITS["width_tolerance_m"])


def run_plan(plan, observation, backend, journal, cancelled=lambda: False):
    """Synchronous worker; independent process may request cancellation.

    No retry, automatic enable, home, pose modification or reverse trajectory.
    A backend must prove its hold capability BEFORE opening any device.
    """
    started = time.monotonic()
    possibly_sent = False
    report = {"ok": False, "status": "initializing", "completed_stages": [],
              "task_success": "not_assessed", "grasp_verified": False,
              "paired_semantics": "near-time dispatch plus completion barrier; not synchronized paths",
              "physical_stop_verified": False, "cleanup_errors": []}
    last = None
    def guard():
        if cancelled():
            raise ExecutionFault("Cancellation requested")
        if time.monotonic() - started > LIMITS["total_timeout_s"]:
            raise ExecutionFault("Plan time limit exceeded")
    try:
        errors = backend.commissioning_errors()
        if errors:
            raise ExecutionFault("Backend not commissioned: " + json.dumps(errors))
        guard()
        backend.connect()
        journal.append("connected_no_motion")
        startup_deadline = time.monotonic() + LIMITS["startup_timeout_s"]
        while True:
            guard()
            initial = backend.snapshot()
            values = initial.get("arms", {})
            if set(values) == {"left", "right"} and all(s.get("status") == "complete" for s in values.values()):
                break
            # A known fault must not be hidden by waiting for another arm.
            for side, state in values.items():
                status = state.get("arm_status") or {}
                if status.get("arm_status") not in (None, 0) or state.get("communication_error"):
                    raise ExecutionFault("Startup feedback fault: " + side)
            if time.monotonic() >= startup_deadline:
                raise ExecutionFault("Initial feedback timeout; no motion issued")
            time.sleep(LIMITS["poll_s"])
        check_feedback(initial)
        _anchor_check(backend, initial, observation["state"])
        anchor = initial

        def stable_start(reference):
            until = time.monotonic() + LIMITS["stable_s"]
            first_stamps = _stamp_vector(reference)
            while True:
                guard()
                current = backend.snapshot()
                check_feedback(current)
                _anchor_check(backend, current, reference)
                if any(s["arm_status"]["motion_status"] != 0 for s in current["arms"].values()):
                    raise ExecutionFault("A controller is still moving before dispatch")
                newest = _stamp_vector(current)
                if time.monotonic() >= until and _advanced(newest, first_stamps):
                    return current
                time.sleep(LIMITS["poll_s"])

        for stage in plan["stages"]:
            # A mixed arm move + grip stage is rejected by plan validation.
            targets = stage["targets"]
            groups = [targets] if stage["coordination"] == "paired" else [[t] for t in targets]
            for group in groups:
                last = stable_start(anchor)
                before = last
                first_dispatch = None
                sent_at = {}
                # Validate the WHOLE group before the first arm can transmit.
                for target in group:
                    if target.get("action", "move") == "gripper" and not _pose_close(
                            backend, before["arms"][target["arm"]]["pose_m_rad"], target["pose_m_rad"]):
                        raise ExecutionFault("Gripper-only target must match the current flange pose")
                for target in group:
                    guard()
                    journal.append("dispatch_intent", stage=stage["id"], target=target)
                    # A throw midway through SDK's multi-frame write is uncertain.
                    side = target["arm"]
                    begin = time.monotonic()
                    if first_dispatch is None:
                        first_dispatch = begin
                    if begin - first_dispatch > LIMITS["max_dispatch_skew_s"]:
                        raise ExecutionFault("Paired dispatch skew exceeded before next arm")
                    possibly_sent = True
                    if target.get("action", "move") == "gripper":
                        backend.grip(target)
                    else:
                        backend.move(target)
                    sent_at[side] = time.time()  # Require feedback after all target frames.
                    journal.append("sdk_send_returned", stage=stage["id"], arm=side,
                                   counts=backend.transmission_counts())
                if len(group) > 1 and time.monotonic() - first_dispatch > LIMITS["max_dispatch_skew_s"]:
                    raise ExecutionFault("Paired dispatch duration exceeded; some frames may have been sent")

                wait_started, stable_since = time.monotonic(), None
                first_stable_stamps = None
                last_logged = -math.inf
                while True:
                    guard()
                    last = backend.snapshot()
                    check_feedback(last)
                    active = {t["arm"] for t in group}
                    for side in {"left", "right"} - active:
                        # The other arm may be holding cloth: it cannot silently drift.
                        _anchor_check(backend, last, before, sides=(side,))
                        if last["arms"][side]["arm_status"]["motion_status"] != 0:
                            raise ExecutionFault("Inactive arm started moving: " + side)
                    for target in group:
                        side = target["arm"]
                        if target.get("action", "move") == "gripper":
                            _anchor_check(backend, last, before, sides=(side,), check_gripper=False)
                            if last["arms"][side]["arm_status"]["motion_status"] != 0:
                                raise ExecutionFault("Arm moved during gripper-only action")
                        elif abs(last["arms"][side]["gripper"]["width_m"] - before["arms"][side]["gripper"]["width_m"]) > LIMITS["width_tolerance_m"]:
                            raise ExecutionFault("Gripper changed during arm motion: " + side)
                    if time.monotonic() - last_logged >= 0.1:
                        journal.append("monitor", stage=stage["id"], feedback=last)
                        last_logged = time.monotonic()
                    fresh = all(_all_new(last["arms"][t["arm"]], sent_at[t["arm"]]) for t in group)
                    reached = fresh and all(_goal_reached(backend, last["arms"][t["arm"]], t) for t in group)
                    stamps = _stamp_vector(last)
                    if reached:
                        if stable_since is None:
                            stable_since = time.monotonic()
                            first_stable_stamps = stamps
                        elif time.monotonic() - stable_since >= LIMITS["stable_s"] and _advanced(stamps, first_stable_stamps):
                            break
                    else:
                        stable_since = None
                    if time.monotonic() - wait_started > LIMITS["stage_timeout_s"]:
                        raise ExecutionFault("Target timeout: " + stage["id"])
                    time.sleep(LIMITS["poll_s"])
                journal.append("targets_reached", stage=stage["id"], feedback=last)
                anchor = last
            report["completed_stages"].append(stage["id"])
        journal.append("plan_targets_reached", completed_stages=report["completed_stages"])
        report.update(ok=True, status="targets_reached", final_feedback=last)
    except BaseException as exc:
        report.update(ok=False, status="motion_state_unknown" if possibly_sent else "rejected_before_motion",
                      error=f"{type(exc).__name__}: {exc}", final_feedback=last)
        if possibly_sent:
            try:
                hold = backend.request_hold_all()
                report["hold_result"] = hold
                # Backend must distinguish send-returned from actual holding.
                report["physical_stop_verified"] = hold.get("all_stopped") is True
                if report["physical_stop_verified"]:
                    report["status"] = "aborted_and_hold_verified"
            except BaseException as stop_exc:
                report["hold_error"] = f"{type(stop_exc).__name__}: {stop_exc}"
        try:
            journal.append("execution_fault", report=report)
        except BaseException as log_exc:
            report["journal_error"] = f"{type(log_exc).__name__}: {log_exc}"
    finally:
        try:
            cleanup = backend.close()
            report["cleanup_result"] = cleanup
            if isinstance(cleanup, dict):
                report["cleanup_errors"] = [f"{side}: {value.get('error', value.get('status'))}"
                                            for side, value in cleanup.get("arms", {}).items()
                                            if value.get("status") != "disconnected"]
                if cleanup.get("status") != "complete" and not report["cleanup_errors"]:
                    report["cleanup_errors"] = ["Backend cleanup not complete"]
            else:
                report["cleanup_errors"] = cleanup or []
        except BaseException as exc:
            report["cleanup_errors"] = [f"{type(exc).__name__}: {exc}"]
        try:
            report["transmissions"] = backend.transmission_counts()
        except BaseException as exc:
            report["transmissions"] = {"status": "unknown", "error": f"{type(exc).__name__}: {exc}"}
            report.update(ok=False, status="transmission_accounting_failed")
        if report["cleanup_errors"] and report["ok"]:
            report.update(ok=False, status="targets_reached_cleanup_failed")
    return report
