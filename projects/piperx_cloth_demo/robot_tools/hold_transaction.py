"""A same-worker MOVE_J target-replacement transaction, with no transport I/O.

This is a foundation for a future joint adapter, NOT a MOVE_L cancellation
implementation. It neither constructs a robot nor calls a sender. The owning
adapter must durably reserve ONE hold attempt against the original event before
prepare(), interpose before_frame() immediately before each actual bus send,
then record_frame_return(). No blocking work may separate that last check from
the send. It must retain the normal task fault, platform lock and RX connection.
There is deliberately no reusable send token, resume API or fault-clear method.
Restart/pending/partial attempts must never reconstruct a sendable transaction.
State synchronization here prevents fault revival; it does not make the future
adapter's interval between returning from a check and actual bus I/O atomic.

Identity/receipts/envelope are trusted host-resolved data, not tool-call inputs.
References are checked for consistency, not authenticated by this module. The
host must resolve actual limits, attachment geometry, model/FK and controller
identity. A fabricated envelope, hash, or dictionary cannot qualify hardware.
Both arm snapshots use the existing arms.snapshot schema; no SDK is imported.

Protocol basis: official piper_sdk ArmMsgMotionCtrl_2 describes 0x151 as mode/
speed (not stop); pyAgxArm default parser encodes joint targets 0x155..0x157 as
signed big-endian millidegrees. The local ros_guarded_task_entry.py:86 send_hold
uses one unchanged J-mode frame then all six newly measured joints. Its historic
right-arm PiPER empty-load evidence does not qualify PiPER X, the other arm,
MOVE_L, contact loads or a general physical stop. The official ROS DEFAULT stop
callback may use move_js, which changes MIT behavior; that route is excluded.
"""
import copy
from functools import wraps
import hashlib
import json
import math
import struct
import threading

from . import arms
from .fault_feedback import FaultFeedback
from .joint_geometry import OfficialJointModel
from .model_compatibility import matrix_error, pose_matrix


RAD_PER_RAW = math.pi / 180000.
_FRAME_FLAGS = {"is_extended_id": False, "is_remote_frame": False, "is_error_frame": False, "is_fd": False}
MODE_FRAME = {"arbitration_id": 0x151, "data_hex": "0101010000000000", **_FRAME_FLAGS}
POLICY = {"age_s": .1, "skew_s": .1, "joint_rad": .003, "position_m": .0005,
          "rotation_rad": .003, "jaw_m": .0005, "stable_s": 3., "advances": 20,
          "hold_timeout_s": 20., "max_translation_m": .03, "max_rotation_rad": .05,
          "fk_position_error_m": .002, "fk_rotation_error_rad": .02}
_IDENTITY_KEYS = {"run_id", "owner", "epoch", "worker_id", "arm", "connection_id",
                  "model", "firmware_profile"}


class HoldTransactionError(RuntimeError):
    def __init__(self, code, message=None):
        self.code = code
        super().__init__(message or code)


def _require(condition, code, message=None):
    if not condition:
        raise HoldTransactionError(code, message)


def _number(value, label):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    _require(valid, "invalid_number", label)
    return value


def _identifier(value, label):
    _require(type(value) is str and 0 < len(value) <= 128 and value == value.strip()
             and all(ord(c) >= 32 and ord(c) != 127 for c in value), "invalid_identifier", label)
    return value


def _keys(value, expected, label):
    _require(type(value) is dict and set(value) == set(expected), "invalid_schema", label)


def _integer_in(value, allowed):
    return isinstance(value, int) and not isinstance(value, bool) and value in allowed


def _hash(value):
    _require(type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value),
             "invalid_source_hash")


def _six(values, label, *, raw=False):
    _require(type(values) is list and len(values) == 6, "invalid_values", label)
    for value in values:
        _number(value, label)
        if raw:
            _require(type(value) is int and -(2**31) <= value < 2**31, "invalid_raw_joint", label)
    return values


def _identity(value):
    _keys(value, _IDENTITY_KEYS, "identity")
    for key, item in value.items():
        _identifier(item, key)
    _require(value["arm"] in ("left", "right"), "invalid_arm")
    _require(value["model"] in ("piper", "piper_x") and value["firmware_profile"] == "default",
             "unsupported_protocol_profile")


def _quaternion(p):
    cr, cp, cy = (math.cos(v/2) for v in p[3:])
    sr, sp, sy = (math.sin(v/2) for v in p[3:])
    q = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
         cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    norm = math.sqrt(sum(v*v for v in q))
    return tuple(v/norm for v in q)


def _rotation(a, b):
    return 2*math.acos(min(1., abs(sum(x*y for x, y in zip(_quaternion(a), _quaternion(b))))))


def joint_hold_frames(raw_joints):
    """Offline byte description only; not an admission or transport interface."""
    _six(raw_joints, "raw_joints", raw=True)
    return [dict(MODE_FRAME)] + [{"arbitration_id": 0x155 + index,
            "data_hex": struct.pack(">ii", *raw_joints[2*index:2*index+2]).hex(), **_FRAME_FLAGS} for index in range(3)]


def _frame_matches(frame, expected):
    return (type(frame) is dict and set(frame) == set(expected)
            and type(frame.get("arbitration_id")) is int and type(frame.get("data_hex")) is str
            and all(frame.get(name) is False for name in _FRAME_FLAGS) and frame == expected)


def _locked(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._state_lock:
            return method(self, *args, **kwargs)
    return call


class JointHoldTransaction:
    """One complete-source event -> at most four exact frame attempts.

    original_event keys: event_id, identity, worker_thread_id, send_state,
    target_raw, reference (a pre-action pair feedback sample), frame_receipts,
    limits, deadline_at, fault. Each original frame receipt contains frame,
    outcome='returned', returned_at. Local send return is NOT driver acceptance.

    claim keys: hold_event_id, original_event_id, identity, claimed_at,
    original_event_sha256. The host, not this object, persists and authenticates
    that claim. This object cannot be resumed from report() after process exit.

    limits freeze joint_limits_raw for each arm, workspace_min_m/max_m and
    max_translation_m/max_rotation_rad. All ordinary nominal limits stay active.
    prepare envelope: sample_id, model, source_ref, source_sha256,
    fk_pose_m_rad, joint_radius_bounds_m (including actual attachments).
    With original.geometry_source and a pinned OfficialJointModel, the last
    two fields become geometry_mode, attachment_radius_m, link_body_allowance_m.
    Workspace/flange budget then use model geometry; raw telemetry remains
    unchanged with independent health and relative-drift checks. Optional
    frozen_target_raw retains the durable claim's integer target across a
    fresh sample, with its actual bounded error included in the envelope.
    """

    def __init__(self, original_event, claim, *, model_geometry=None):
        geometry_keys = ("geometry_source",) if isinstance(original_event, dict) and "geometry_source" in original_event else ()
        _keys(original_event, ("event_id", "identity", "worker_thread_id", "send_state", "target_raw", "reference",
                               "frame_receipts", "limits", "deadline_at", "fault") + geometry_keys, "original event")
        _keys(claim, ("hold_event_id", "original_event_id", "identity", "claimed_at", "original_event_sha256"), "claim")
        _identity(original_event["identity"])
        _identity(claim["identity"])
        _require(original_event["identity"] == claim["identity"], "identity_mismatch")
        if geometry_keys:
            _require(type(model_geometry) is OfficialJointModel
                     and original_event["geometry_source"] == model_geometry.source
                     and model_geometry.source["model"] == original_event["identity"]["model"],
                     "model_geometry_source_mismatch")
        else:
            _require(model_geometry is None, "unbound_model_geometry")
        self._model_geometry = model_geometry
        for key in ("event_id",):
            _identifier(original_event[key], key)
        _identifier(claim["hold_event_id"], "hold_event_id")
        _require(claim["original_event_id"] == original_event["event_id"]
                 and claim["hold_event_id"] != original_event["event_id"], "event_binding_mismatch")
        _hash(claim["original_event_sha256"])
        try:
            original_digest = hashlib.sha256(json.dumps(original_event, sort_keys=True,
                separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        except (TypeError, ValueError, OverflowError) as exc:
            raise HoldTransactionError("invalid_original_event") from exc
        _require(claim["original_event_sha256"] == original_digest, "original_event_digest_mismatch")
        _require(type(original_event["worker_thread_id"]) is int
                 and original_event["worker_thread_id"] == threading.get_ident(), "worker_mismatch")
        _require(original_event["send_state"] == "all_frames_returned", "original_send_incomplete")
        expected = joint_hold_frames(original_event["target_raw"])
        receipts = original_event["frame_receipts"]
        _require(type(receipts) is list and len(receipts) == 4, "original_send_incomplete")
        previous = None
        for receipt, frame in zip(receipts, expected):
            _keys(receipt, ("frame", "outcome", "returned_at"), "original frame receipt")
            at = _number(receipt["returned_at"], "returned_at")
            _require(at >= 0 and (previous is None or at >= previous), "invalid_original_send_time")
            _require(_frame_matches(receipt["frame"], frame) and receipt["outcome"] == "returned", "original_send_incomplete")
            previous = at
        claimed = _number(claim["claimed_at"], "claimed_at")
        deadline = _number(original_event["deadline_at"], "deadline_at")
        _require(0 <= previous <= claimed < deadline, "invalid_claim_time")
        limits = original_event["limits"]
        _keys(limits, ("joint_limits_raw", "workspace_min_m", "workspace_max_m", "max_translation_m", "max_rotation_rad"), "limits")
        _keys(limits["joint_limits_raw"], ("left", "right"), "joint limits")
        for side in ("left", "right"):
            pairs = limits["joint_limits_raw"][side]
            _require(type(pairs) is list and len(pairs) == 6, "invalid_joint_limits")
            for pair in pairs:
                _require(type(pair) is list and len(pair) == 2
                         and all(type(v) is int and -(2**31) <= v < 2**31 for v in pair)
                         and pair[0] < pair[1], "invalid_joint_limits")
        _require(all(lo <= raw <= hi for raw, (lo, hi) in zip(original_event["target_raw"],
                 limits["joint_limits_raw"][original_event["identity"]["arm"]])), "original_target_limit")
        for name in ("workspace_min_m", "workspace_max_m"):
            _require(type(limits[name]) is list and len(limits[name]) == 3, "invalid_workspace")
            for value in limits[name]:
                _number(value, name)
        _require(all(a < b for a, b in zip(limits["workspace_min_m"], limits["workspace_max_m"])), "invalid_workspace")
        for key, cap in (("max_translation_m", .03), ("max_rotation_rad", .05)):
            _require(0 < _number(limits[key], key) <= cap, "motion_bound_enlarged")
        self._original, self._claim = copy.deepcopy(original_event), copy.deepcopy(claim)
        self._identity = copy.deepcopy(claim["identity"])
        self._arm = self._identity["arm"]
        self._peer = "right" if self._arm == "left" else "left"
        self._limits = copy.deepcopy(limits)
        self._thread = threading.get_ident()
        self._last_time = claimed
        self._last_stamps = None
        self._plan = None
        self._frames = None
        self._pending = None
        self._attempts = []
        self._status = "claimed"
        self._failure = None
        self._state_lock = threading.RLock()
        self._window = []
        self._window_stats = {}
        self._post_advances = 0
        self._last_post = None
        self._last_advanced_post = None
        self._post_samples = 0
        self._last_diagnostic = None
        self._rx = FaultFeedback(max_age_s=.1, max_skew_s=.1)
        self._first_send_at = self._last_send_at = None
        self._hold_evidence = None
        # Validate historical reference independently at its own observation time.
        reference = self._original["reference"]
        _require(type(reference) is dict, "invalid_reference")
        self._validate_sample(reference, reference.get("captured_at"), original=True)
        _require(reference["captured_at"] <= receipts[0]["returned_at"], "invalid_reference_time")
        self._original_complete_at = previous
        self._last_stamps = None

    @_locked
    def _fail(self, code, message=None):
        if self._failure is None:
            self._failure = {"code": code, "detail": message or code, "at": self._last_time}
        self._status = "fault"
        raise HoldTransactionError(code, message)

    @_locked
    def _time(self, now):
        now = _number(now, "now")
        _require(now >= self._last_time, "clock_regressed")
        self._last_time = now
        return now

    def _caller(self, identity):
        _identity(identity)
        _require(identity == self._identity, "identity_mismatch")
        _require(threading.get_ident() == self._thread, "worker_mismatch")

    def _validate_sample(self, sample, now, *, original=False):
        _keys(sample, ("sample_id", "identity", "captured_at", "arms"), "feedback sample")
        _identifier(sample["sample_id"], "sample_id")
        _identity(sample["identity"])
        _require(sample["identity"] == self._identity, "identity_mismatch")
        captured = _number(sample["captured_at"], "captured_at")
        now = _number(now, "now")
        _require(0 <= now-captured <= .1, "stale_sample")
        _keys(sample["arms"], ("left", "right"), "pair feedback")
        stamps = {}
        for side, state in sample["arms"].items():
            _require(type(state) is dict and state.get("status") == "complete"
                     and arms.control_health(state, now_s=now, allowed_control_modes=(1,),
                                                                 require_enabled=True)["healthy"], "unhealthy_feedback")
            status = state["arm_status"]
            _require(_integer_in(status.get("teach_status"), (0,)), "teach_mode")
            _require(_integer_in(status.get("motion_status"), (0, 1)), "unknown_motion_status")
            if side == self._arm:
                _require(_integer_in(status.get("mode_feedback"), (1,)), "requires_existing_move_j")
            else:
                _require(status["motion_status"] == 0, "passive_arm_moving")
                _require(_integer_in(status.get("mode_feedback"), (0, 1, 2)), "unknown_passive_mode")
            for q, (low, high) in zip(state["joints_rad"], self._limits["joint_limits_raw"][side]):
                _require(low*RAD_PER_RAW <= q <= high*RAD_PER_RAW, "nominal_joint_limit")
            if self._model_geometry is None:
                _require(all(lo <= p <= hi for p, lo, hi in zip(state["pose_m_rad"][:3], self._limits["workspace_min_m"],
                                                               self._limits["workspace_max_m"])), "workspace_limit")
            _require(0 <= state["gripper"]["width_m"] <= .055, "jaw_limit")
            if self._model_geometry is not None:
                model_pose = self._model_geometry.matrix(state["joints_rad"])
                _require(all(lo <= model_pose[i][3] <= hi for i, (lo, hi) in enumerate(zip(
                    self._limits["workspace_min_m"], self._limits["workspace_max_m"]))), "model_workspace_limit")
            for name in arms.PARTS + arms.DRIVERS + ("gripper",):
                stamp = _number(state["fragment_timestamps_s"][name], "fragment time")
                _require(0 <= now-stamp <= .1, "stale_feedback")
                stamps[(side, name)] = stamp
        _require(max(stamps.values())-min(stamps.values()) <= .1, "feedback_skew")
        _require(max(stamps.values()) <= captured, "feedback_after_capture")
        if not original:
            _require(min(stamps.values()) > self._original_complete_at, "pre_original_send_feedback")
            if self._last_stamps is not None:
                _require(all(stamp >= self._last_stamps[key] for key, stamp in stamps.items()), "feedback_regressed")
            reference = self._original["reference"]["arms"]
            for side, state in sample["arms"].items():
                anchor = reference[side]
                _require(abs(state["gripper"]["width_m"]-anchor["gripper"]["width_m"]) <= .0005, "jaw_drift")
                if side == self._peer:
                    _require(state["arm_status"]["mode_feedback"] == anchor["arm_status"]["mode_feedback"], "passive_mode_changed")
                    self._stationary(anchor, state, "passive_arm_drift")
                else:
                    target = [raw*RAD_PER_RAW for raw in self._original["target_raw"]]
                    _require(all(min(a, b)-.003 <= q <= max(a, b)+.003 for a, b, q in
                                 zip(anchor["joints_rad"], target, state["joints_rad"])), "original_joint_envelope")
                    _require(math.dist(state["pose_m_rad"][:3], anchor["pose_m_rad"][:3]) <= self._limits["max_translation_m"]
                             and _rotation(state["pose_m_rad"], anchor["pose_m_rad"]) <= self._limits["max_rotation_rad"],
                             "original_pose_envelope")
                    if self._model_geometry is not None:
                        error = matrix_error(self._model_geometry.matrix(state["joints_rad"]),
                                             self._model_geometry.matrix(anchor["joints_rad"]))
                        _require(error["position_error_m"] <= self._limits["max_translation_m"]
                                 and error["so3_error_rad"] <= self._limits["max_rotation_rad"],
                                 "original_model_pose_envelope")
            if self._plan is not None:
                self._stationary(self._plan["sample"]["arms"][self._arm], sample["arms"][self._arm], "hold_plan_slip")
                if self._model_geometry is not None:
                    error = matrix_error(self._model_geometry.matrix(self._plan["sample"]["arms"][self._arm]["joints_rad"]),
                                         self._model_geometry.matrix(sample["arms"][self._arm]["joints_rad"]))
                    _require(error["position_error_m"] <= .0005 and error["so3_error_rad"] <= .003,
                             "model_hold_plan_slip")
            self._last_stamps = stamps
        return stamps

    @staticmethod
    def _stationary(anchor, state, reason):
        _require(max(abs(a-b) for a, b in zip(anchor["joints_rad"], state["joints_rad"])) <= .003
                 and math.dist(anchor["pose_m_rad"][:3], state["pose_m_rad"][:3]) <= .0005
                 and _rotation(anchor["pose_m_rad"], state["pose_m_rad"]) <= .003, reason)

    def prepare(self, sample, envelope, *, current_identity, now, frozen_target_raw=None):
        """Freeze all six current targets; no public setter, mode change or TX."""
        try:
            self._caller(current_identity)
            now = self._time(now)
            _require(self._status == "claimed", "hold_already_claimed_or_finished")
            _require(now < self._original["deadline_at"], "original_budget_expired")
            self._validate_sample(sample, now)
            fields = (("fk_pose_m_rad", "joint_radius_bounds_m") if self._model_geometry is None else
                      ("geometry_mode", "attachment_radius_m", "link_body_allowance_m"))
            _keys(envelope, ("sample_id", "model", "source_ref", "source_sha256") + fields, "envelope")
            _identifier(envelope["source_ref"], "source_ref")
            _hash(envelope["source_sha256"])
            _require(envelope["sample_id"] == sample["sample_id"] and envelope["model"] == self._identity["model"], "envelope_binding_mismatch")
            selected = sample["arms"][self._arm]
            if self._model_geometry is None:
                fk = _six(envelope["fk_pose_m_rad"], "fk_pose_m_rad")
                radii = _six(envelope["joint_radius_bounds_m"], "joint_radius_bounds_m")
                _require(all(value > 0 for value in radii), "invalid_geometry")
                position_error = math.dist(fk[:3], selected["pose_m_rad"][:3])
                rotation_error = _rotation(fk, selected["pose_m_rad"])
                _require(position_error <= .002 and rotation_error <= .02, "model_controller_pose_mismatch")
                centre = selected["pose_m_rad"][:3]
                original_pose = self._original["reference"]["arms"][self._arm]["pose_m_rad"]
                position_used = math.dist(centre, original_pose[:3])
                rotation_used = _rotation(selected["pose_m_rad"], original_pose)
                geometry_report = {"mode": "absolute_fk_controller_agreement"}
            else:
                _require(envelope["geometry_mode"] == "model_joint_geometry_v1", "model_geometry_mode_mismatch")
                try:
                    body_radii = self._model_geometry.radii(envelope["attachment_radius_m"], envelope["link_body_allowance_m"])
                    radii = self._model_geometry.flange_radii()
                except ValueError as exc:
                    raise HoldTransactionError("invalid_geometry", str(exc)) from exc
                model_current = self._model_geometry.matrix(selected["joints_rad"])
                model_origin = self._model_geometry.matrix(self._original["reference"]["arms"][self._arm]["joints_rad"])
                used = matrix_error(model_current, model_origin)
                centre = [row[3] for row in model_current[:3]]
                position_used, rotation_used = used["position_error_m"], used["so3_error_rad"]
                # No estimated correction is applied to measured telemetry.
                # Geometry is entirely the pinned model's joint-space contract.
                position_error, rotation_error = 0., 0.
                geometry_report = {"mode": "model_joint_geometry_v1", "source": self._model_geometry.source,
                    "model_origin_transform": model_origin, "model_current_transform": model_current,
                    "controller_pose_m_rad": copy.deepcopy(selected["pose_m_rad"]),
                    "body_joint_radius_bounds_m": body_radii,
                    "flange_joint_radius_bounds_m": radii,
                    "model_controller_difference_diagnostic": matrix_error(model_current, pose_matrix(selected["pose_m_rad"])),
                    "model_pose_is_independent_measurement": False}
            if frozen_target_raw is None:
                raw = [round(q/RAD_PER_RAW) for q in selected["joints_rad"]]
            else:
                _require(type(frozen_target_raw) is list and len(frozen_target_raw) == 6
                         and all(type(q) is int and -(2**31) <= q < 2**31 for q in frozen_target_raw),
                         "invalid_frozen_hold_target")
                raw = list(frozen_target_raw)
                _require(all(abs(q-value*RAD_PER_RAW) <= .003
                             for q, value in zip(selected["joints_rad"], raw)), "frozen_hold_target_drift")
            _require(all(low <= value <= high for value, (low, high) in zip(raw, self._limits["joint_limits_raw"][self._arm])),
                     "encoded_joint_limit")
            excursions = [abs(q-value*RAD_PER_RAW)+.003 for q, value in zip(selected["joints_rad"], raw)]
            position_bound = position_error + sum(r*delta for r, delta in zip(radii, excursions))
            rotation_bound = rotation_error + sum(excursions)
            _require(position_used+position_bound <= self._limits["max_translation_m"]
                     and rotation_used+rotation_bound <= self._limits["max_rotation_rad"],
                     "hold_tracking_box_exceeds_original_bounds")
            _require(all(lo <= p-position_bound and p+position_bound <= hi for p, lo, hi in
                         zip(centre, self._limits["workspace_min_m"], self._limits["workspace_max_m"])),
                     "hold_tracking_box_exceeds_workspace")
            with self._state_lock:
                _require(self._failure is None and self._status == "claimed", "hold_fault_latched")
                self._frames = joint_hold_frames(raw)
                self._plan = {"sample": copy.deepcopy(sample), "target_raw": raw, "envelope": copy.deepcopy(envelope),
                              "position_bound_m": position_bound, "rotation_bound_rad": rotation_bound,
                              "position_used_m": position_used, "rotation_used_rad": rotation_used,
                              "geometry": geometry_report}
                self._status = "prepared"
        except HoldTransactionError as exc:
            self._fail(exc.code, str(exc))
        return self.report()

    def before_frame(self, frame, sample, *, current_identity, now):
        """Consume exactly one frame attempt immediately before the actual send.

        Returns None, not a permit. The original worker's bus interposer must
        call this method for every frame, with post-durable-I/O fresh feedback.
        If the actual send does not return, leave the attempt pending forever.
        """
        try:
            self._caller(current_identity)
            now = self._time(now)
            _require(self._status in ("prepared", "sending") and self._pending is None, "hold_attempt_not_sendable")
            _require(now < self._original["deadline_at"], "original_budget_expired")
            _require(self._first_send_at is None or now-self._first_send_at <= .1, "hold_transaction_timeout")
            _require(len(self._attempts) < 4 and _frame_matches(frame, self._frames[len(self._attempts)]), "unexpected_frame")
            self._validate_sample(sample, now)
            if not self._attempts:
                planned = self._plan["sample"]
                _require(sample["sample_id"] != planned["sample_id"]
                         and all(sample["arms"][side]["fragment_timestamps_s"][name] >
                                 planned["arms"][side]["fragment_timestamps_s"][name]
                                 for side in ("left", "right") for name in arms.PARTS + arms.DRIVERS + ("gripper",)),
                         "fresh_dispatch_witness_required")
            with self._state_lock:
                # External invalidate/RX failure can arrive while the pure
                # feedback checks run. Never overwrite that sticky failure.
                _require(self._failure is None and self._status in ("prepared", "sending")
                         and self._pending is None, "hold_fault_latched")
                self._pending = {"index": len(self._attempts), "frame": copy.deepcopy(frame), "attempted_at": now,
                                 "outcome": "pending", "returned_at": None, "sample_id": sample["sample_id"]}
                self._attempts.append(self._pending)
                if self._first_send_at is None:
                    self._first_send_at = now
                self._status = "sending"
        except HoldTransactionError as exc:
            self._fail(exc.code, str(exc))

    @_locked
    def record_frame_return(self, *, outcome, current_identity, now):
        """Record returned/exception/unknown for the single pending attempt."""
        try:
            self._caller(current_identity)
            now = self._time(now)
            _require(self._status == "sending" and self._pending is not None, "no_pending_hold_frame")
            _require(outcome in ("returned", "exception", "unknown"), "invalid_frame_outcome")
            self._pending.update(outcome=outcome, returned_at=now)
            if outcome != "returned":
                self._fail("hold_send_uncertain", "Partial/unknown target replacement cannot be retried")
            self._pending = None
            self._last_send_at = now
            if len(self._attempts) == 4:
                self._status = "observing"
        except HoldTransactionError as exc:
            self._fail(exc.code, str(exc))
        return self.report()

    @_locked
    def invalidate(self, reason, *, now):
        """Latch cancellation/EOF/adapter failure without sending or clearing faults."""
        _identifier(reason, "reason")
        detail = reason
        try:
            self._time(now)
        except HoldTransactionError as exc:
            detail += "; invalid cancellation timestamp: " + exc.code
        try:
            self._fail(reason, detail)
        except HoldTransactionError:
            return self.report()

    def _window_spans(self, sample):
        """Exact component ranges and pairwise SO(3) diameter for BOTH arms.

        The immutable original anchors still apply separately. Tracking only
        distance to the first sample would admit oscillations across both sides
        of a tolerance band. Incremental extrema retain the whole window even
        when older full snapshots are compacted after observation completes.
        """
        spans = {}
        for side, state in sample["arms"].items():
            q, p, width = state["joints_rad"], state["pose_m_rad"][:3], state["gripper"]["width_m"]
            stats = self._window_stats.setdefault(side, {"qlo": list(q), "qhi": list(q), "plo": list(p),
                "phi": list(p), "wlo": width, "whi": width, "rotations": set(), "rotation_span": 0.})
            stats["qlo"] = [min(a, b) for a, b in zip(stats["qlo"], q)]
            stats["qhi"] = [max(a, b) for a, b in zip(stats["qhi"], q)]
            stats["plo"] = [min(a, b) for a, b in zip(stats["plo"], p)]
            stats["phi"] = [max(a, b) for a, b in zip(stats["phi"], p)]
            stats["wlo"], stats["whi"] = min(width, stats["wlo"]), max(width, stats["whi"])
            rotation = _quaternion(state["pose_m_rad"])
            if rotation not in stats["rotations"]:
                for previous in stats["rotations"]:
                    angle = 2*math.acos(min(1., abs(sum(a*b for a, b in zip(rotation, previous)))))
                    stats["rotation_span"] = max(stats["rotation_span"], angle)
                stats["rotations"].add(rotation)
            spans[side] = {"joint_rad": max(b-a for a, b in zip(stats["qlo"], stats["qhi"])),
                "position_m": math.dist(stats["plo"], stats["phi"]),
                "rotation_rad": stats["rotation_span"], "jaw_m": stats["whi"]-stats["wlo"]}
            _require(all(value <= POLICY[name] for name, value in spans[side].items()),
                     "unstable_hold_window", side + " complete-window span exceeds existing stability bounds")
        if self._model_geometry is not None:
            model = self._model_geometry.matrix(sample["arms"][self._arm]["joints_rad"])
            p = [row[3] for row in model[:3]]
            rotation = tuple(tuple(row[:3]) for row in model[:3])
            stats = self._window_stats.setdefault("model", {"lo": list(p), "hi": list(p), "rotations": set(), "angle": 0.})
            stats["lo"] = [min(a, b) for a, b in zip(stats["lo"], p)]
            stats["hi"] = [max(a, b) for a, b in zip(stats["hi"], p)]
            if rotation not in stats["rotations"]:
                for previous in stats["rotations"]:
                    cosine = (sum(rotation[i][j]*previous[i][j] for i in range(3) for j in range(3))-1.)/2.
                    stats["angle"] = max(stats["angle"], math.acos(max(-1., min(1., cosine))))
                stats["rotations"].add(rotation)
            model_span = {"position_m": math.dist(stats["lo"], stats["hi"]), "rotation_rad": stats["angle"]}
            _require(model_span["position_m"] <= .0005 and model_span["rotation_rad"] <= .003,
                     "unstable_model_hold_window")
            spans["selected_model_geometry"] = model_span
        return spans

    @_locked
    def observe(self, sample, *, now):
        """RX-only evidence stays usable for diagnosis after any fault."""
        # Diagnosis intentionally precedes action-state validation. A dead send
        # path must not block feedback reporting or suggest a physical stop.
        states = sample.get("arms") if isinstance(sample, dict) else None
        errors = None
        if type(states) is not dict:
            states = {"left": None, "right": None}
            errors = {"snapshot": "Missing or malformed arms mapping"}
        self._last_diagnostic = self._rx.capture(states, now, read_errors=errors)
        if self._status in ("fault", "claimed", "prepared", "sending"):
            return self.report()
        try:
            now = self._time(now)
            stamps = self._validate_sample(sample, now)
            self._post_samples += 1
            _require(self._post_samples <= 5000, "hold_observation_sample_budget")
            _require(now-self._last_send_at <= 20. or self._status == "hold_observed", "hold_observation_timeout")
            _require(min(stamps.values()) > self._last_send_at, "post_hold_send_feedback_required")
            if self._last_post is not None:
                _require(sample["captured_at"] > self._last_post["captured_at"], "observation_time_not_advanced")
                _require(sample["captured_at"]-self._last_post["captured_at"] <= .1 + 1e-12, "post_send_trace_gap")
            else:
                _require(sample["captured_at"]-self._last_send_at <= .1, "post_send_trace_gap")
            advanced = self._last_advanced_post is None or all(
                stamps[(side, name)] > self._last_advanced_post["arms"][side]["fragment_timestamps_s"][name]
                for side in ("left", "right") for name in arms.PARTS + arms.DRIVERS + ("gripper",))
            selected = sample["arms"][self._arm]
            arrived = (selected["arm_status"]["motion_status"] == 0 and
                       max(abs(q-raw*RAD_PER_RAW) for q, raw in zip(selected["joints_rad"], self._plan["target_raw"])) <= .003)
            if not arrived:
                self._window = []
                self._window_stats = {}
                self._post_advances = 0
                self._last_advanced_post = None
                if self._status == "hold_observed":
                    self._fail("hold_no_longer_observed")
            else:
                spans = self._window_spans(sample)  # Include intervening, partially advanced snapshots.
                if advanced:
                    self._window.append(copy.deepcopy(sample))
                    self._post_advances += 1
                    self._last_advanced_post = copy.deepcopy(sample)
                if (self._window and self._window[-1]["captured_at"]-self._window[0]["captured_at"] >= 3.
                        and self._post_advances >= 21):
                    self._status = "hold_observed"
                    self._hold_evidence = {"began_at": self._window[0]["captured_at"],
                                           "ended_at": sample["captured_at"], "feedback_advances": self._post_advances-1,
                                           "target_raw": list(self._plan["target_raw"]),
                                           "spans": spans,
                                           "scope": "Measured J-target stationarity; no load or cancellation acknowledgement"}
                    # Keep bounded memory, preserving the evidence and first anchor.
                    self._window = [self._window[0], self._window[-1]]
            self._last_post = copy.deepcopy(sample)
        except HoldTransactionError as exc:
            try:
                self._fail(exc.code, str(exc))
            except HoldTransactionError:
                pass
        return self.report()

    @_locked
    def report(self):
        """A diagnostic snapshot; cannot recreate or resume this transaction."""
        return copy.deepcopy({"status": self._status, "identity": self._identity,
            "original_event_id": self._original["event_id"], "hold_event_id": self._claim["hold_event_id"],
            "original_fault": self._original["fault"], "hold_fault": self._failure,
            "geometry": copy.deepcopy(self._plan["geometry"]) if self._plan else None,
            "original_deadline_at": self._original["deadline_at"], "claim": self._claim,
            "target_raw": self._plan["target_raw"] if self._plan else None,
            "expected_frames": self._frames, "frame_attempts": self._attempts,
            "frames_complete": len(self._attempts) == 4 and all(item["outcome"] == "returned" for item in self._attempts),
            "hold_observed": self._status == "hold_observed", "hold_evidence": self._hold_evidence,
            "last_feedback": self._last_diagnostic, "accepted": None,
            "physical_stop_verified": None, "original_target_cancelled": None,
            "target_may_remain_active": True, "automatic_retry": False, "dispatch_authorized": False,
            "adapter_integration": "not_integrated", "physical_validation": "not_performed",
            "scope": "MOVE_J same-mode helper only; current MOVE_L stop path remains unsupported"})
