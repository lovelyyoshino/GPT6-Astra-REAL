"""Persistent, attended pair transport; no task policy or physical stop claim.

The caller owns the device lock, persistent once-only claim and scene admission.
Construction is offline. open()/connect_for_preparation() connect without sending
actuator commands. A retained stationary target is not loaded support or a stop.
"""
import copy
from .feedback_tolerance import rotation_tolerance
from .feedback_tolerance import joints_within, window_joints_within
import hashlib
import json
import math
import threading
import time
import uuid
from pathlib import Path

from . import arms
from .contact_receipt import classify_gripper_probe, probe_closure_within_bound
from .fault_feedback import FaultFeedback
from .linear_hold import _LinearHold, _pose_frames
from .motion_effect import measure_motion_effect
from .retention_receipt import (anchor_deviation, digest, identity_checked, measured_anchor, grasp_body_anchor,
                                summarize_retention_trace, summarize_release_confirmation)
from .single_gripper_prepare import _SingleGripperPrepare
from .single_supervised_actions import BOUNDS, _SingleSupervisedAction
from .takeover import LIMITS, SIDES


CAPABILITIES = {"retained_target_stationary": True, "contact_support_verified": False,
                "contact_step_supported": False, "physical_stop_verified": False,
                "gripper_contact_observation": True, "gripper_static_retention": True}


def _require_later_recovery_target(continuation, target):
    prior = continuation["prior_opening_target_m"]
    previous_encoded = round(prior * 1e6) / 1e6
    if any(value <= max(prior, previous_encoded)
           for value in (target, round(target * 1e6) / 1e6)):
        raise RuntimeError("Continued opening must exceed the previous requested and encoded target; no replay")


def _validated_opening_continuation(source, arm, target, now):
    """Check the manager's bound provenance, not certify its archived history.

    The ledger manager verifies the complete old send, closed owner and passive
    trace. This adapter checks its exact source binding and fresh body/jaw RX;
    an envelope or hash alone is never evidence of physical support or stopping.
    """
    if type(source) is not dict:
        raise ValueError("Supported residual source must be an object")
    if "audited_opening_continuation" not in source:
        return None
    value = source["audited_opening_continuation"]
    required = {"schema", "arm", "source_receipt_sha256", "prior_opening_event_id",
                "prior_opening_receipt_sha256", "prior_opening_target_m", "prior_opening_finished_at",
                "audit_proposal_sha256", "residual_jaw_anchor"}
    if (type(value) is not dict or set(value) != required
            or value["schema"] != "piper_supported_opening_continuation_v1" or value["arm"] != arm):
        raise ValueError("Exact audited opening continuation schema and arm required")
    jaw = value["residual_jaw_anchor"]
    if type(jaw) is not dict or set(jaw) != {"width_m", "observed_at", "source"}:
        raise ValueError("Separate residual jaw observation and source provenance required")
    provenance = jaw["source"]
    if (type(provenance) is not dict or set(provenance) != {"path", "sha256"}
            or type(provenance["path"]) is not str or not 1 <= len(provenance["path"]) <= 4096
            or "\x00" in provenance["path"] or not Path(provenance["path"]).is_absolute()):
        raise ValueError("Absolute passive source path and SHA256 provenance required")
    for sha in (value["source_receipt_sha256"], value["prior_opening_receipt_sha256"],
                value["audit_proposal_sha256"], provenance["sha256"]):
        if type(sha) is not str or len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
            raise ValueError("Audited opening source provenance requires exact SHA256 digests")
    event = value["prior_opening_event_id"]
    if (type(event) is not str or not 1 <= len(event) <= 128 or event.strip() != event
            or any(ord(c) < 32 or ord(c) == 127 for c in event)):
        raise ValueError("Prior complete opening event identity required")
    original = {key: item for key, item in source.items() if key != "audited_opening_continuation"}
    if digest(original) != value["source_receipt_sha256"]:
        raise ValueError("Original failed source receipt changed after the continuation audit")
    completed = source.get("candidate_probe", {}).get("completed_at")
    numbers = (jaw["width_m"], jaw["observed_at"], value["prior_opening_target_m"],
               value["prior_opening_finished_at"], completed, now)
    if (any(type(v) not in (float, int) or not math.isfinite(v) for v in numbers)
            or not 0 <= jaw["width_m"] <= .070 or not 0 <= value["prior_opening_target_m"] <= .055
            or not 0 <= completed <= value["prior_opening_finished_at"] < jaw["observed_at"] <= now):
        raise ValueError("Audited opening and later residual observation chronology/width required")
    _require_later_recovery_target(value, target)
    return copy.deepcopy(value)


def _validated_supported_reacquisition(source, arm, now):
    """Bind audited opening evidence without interpreting it as separation."""
    if type(source) is not dict:
        raise ValueError("Supported reacquisition source must be an object")
    value = source.get("audited_supported_reacquisition")
    required = {"schema", "arm", "source_receipt_sha256", "opening_event_id",
                "opening_receipt_sha256", "opening_finished_at", "audit_proposal_sha256", "opening_jaw_anchor"}
    if (type(value) is not dict or set(value) not in (required, required | {"completed_probe_continuation"})
            or value["schema"] != "piper_supported_reacquisition_v1" or value["arm"] != arm):
        raise ValueError("Exact audited supported reacquisition schema and arm required")
    jaw = value["opening_jaw_anchor"]
    if type(jaw) is not dict or set(jaw) != {"width_m", "observed_at", "source"}:
        raise ValueError("Successful opening jaw observation and source provenance required")
    provenance = jaw["source"]
    if (type(provenance) is not dict or set(provenance) != {"path", "sha256"}
            or type(provenance["path"]) is not str or not 1 <= len(provenance["path"]) <= 4096
            or "\x00" in provenance["path"] or not Path(provenance["path"]).is_absolute()):
        raise ValueError("Absolute opening source path and SHA256 provenance required")
    for sha in (value["source_receipt_sha256"], value["opening_receipt_sha256"],
                value["audit_proposal_sha256"], provenance["sha256"]):
        if type(sha) is not str or len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
            raise ValueError("Audited reacquisition provenance requires exact SHA256 digests")
    event = value["opening_event_id"]
    if (type(event) is not str or not 1 <= len(event) <= 128 or event.strip() != event
            or any(ord(c) < 32 or ord(c) == 127 for c in event)):
        raise ValueError("Successful opening event identity required")
    original = {key: item for key, item in source.items() if key != "audited_supported_reacquisition"}
    if digest(original) != value["source_receipt_sha256"]:
        raise ValueError("Original failed source receipt changed after the reacquisition audit")
    completed = source.get("candidate_probe", {}).get("completed_at")
    numbers = (jaw["width_m"], jaw["observed_at"], value["opening_finished_at"], completed, now)
    if (any(type(v) not in (float, int) or not math.isfinite(v) for v in numbers)
            or not 0 <= jaw["width_m"] <= .070
            or not 0 <= completed < jaw["observed_at"] <= value["opening_finished_at"] <= now):
        raise ValueError("Successful opening after the original probe requires finite measured width and chronology")
    if "completed_probe_continuation" in value:
        continuation = value["completed_probe_continuation"]
        fields = {"schema", "arm", "failed_event_id", "failed_receipt_sha256", "prior_sent_target_m",
                  "failed_finished_at", "consumed_probe_count", "remaining_probe_count", "residual_jaw_anchor"}
        if (type(continuation) is not dict or set(continuation) != fields
                or continuation["schema"] != "piper_completed_contact_continuation_v1"
                or continuation["arm"] != arm
                or type(continuation["consumed_probe_count"]) is not int or continuation["consumed_probe_count"] != 2
                or type(continuation["remaining_probe_count"]) is not int or continuation["remaining_probe_count"] != 1):
            raise ValueError("Exact completed-probe continuation with two consumed and one remaining probe required")
        residual = continuation["residual_jaw_anchor"]
        if type(residual) is not dict or set(residual) != {"width_m", "observed_at", "source"}:
            raise ValueError("Separate completed-probe residual jaw provenance required")
        provenance = residual["source"]
        if (type(provenance) is not dict or set(provenance) != {"path", "sha256"}
                or type(provenance["path"]) is not str or not 1 <= len(provenance["path"]) <= 4096
                or "\x00" in provenance["path"] or not Path(provenance["path"]).is_absolute()):
            raise ValueError("Absolute residual jaw source path and SHA256 provenance required")
        for sha in (continuation["failed_receipt_sha256"], provenance["sha256"]):
            if type(sha) is not str or len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
                raise ValueError("Completed-probe provenance requires exact SHA256 digests")
        event = continuation["failed_event_id"]
        if (type(event) is not str or not 1 <= len(event) <= 128 or event.strip() != event
                or event == value["opening_event_id"] or any(ord(c) < 32 or ord(c) == 127 for c in event)):
            raise ValueError("Distinct completed-probe failure event identity required")
        numbers = (residual["width_m"], residual["observed_at"], continuation["prior_sent_target_m"],
                   continuation["failed_finished_at"])
        if (any(type(v) not in (float, int) or not math.isfinite(v) for v in numbers)
                or not 0 <= residual["width_m"] <= .070 or not 0 <= continuation["prior_sent_target_m"] <= .055
                or not value["opening_finished_at"] < continuation["failed_finished_at"] < residual["observed_at"] <= now):
            raise ValueError("Completed-probe failure and newer residual jaw require finite width and chronology")
    return copy.deepcopy(value)


class _PairAction(_SingleSupervisedAction):
    """One mutable action context, permanently owning the guarded SDK objects."""

    def __init__(self, profile, journal, guard):
        super().__init__(profile, journal, "right", "gripper", 0.0)
        self.guard = guard
        self.idle_anchor = None
        self.boundary_origins = {}
        self.archived_counts = copy.deepcopy(self.counts)
        self.probe_mode = None
        self.probe_trace = None
        self.grasps = {side: None for side in SIDES}
        self.retention_trace = None
        self.joint_executor = None
        self.auxiliary_executor = None
        self.supported_reacquisition = None
        self.supported_contact_observation = None

    def window_stable(self, window):
        return all(window_joints_within(self.feedback_policy, side, window[side])
                   and spans["position_m"] <= BOUNDS["position_span_m"]
                   and spans["jaw_m"] <= BOUNDS["jaw_span_m"]
                   and spans["rotation_rad"] <= rotation_tolerance(self.feedback_policy,side)
                   for side,spans in self.window_spans(window).items())

    def _executor(self):
        if self.joint_executor is not None and self.auxiliary_executor is not None:
            raise RuntimeError("Two pair operation contexts cannot share the encoder")
        return self.auxiliary_executor or self.joint_executor

    @property
    def unresolved_probe(self):
        return next((item for item in self.grasps.values()
                     if item is not None and item["status"] in (
                         "contact_candidate", "release_opened", "loaded_pending_visual")), None)

    def require_no_loaded_pending_tx(self):
        if any(item is not None and item["status"] == "loaded_pending_visual" for item in self.grasps.values()):
            raise RuntimeError("Loaded response awaits new visual confirmation; no new CAN transmission")

    def require_supported_reacquisition_tx(self, side, kind):
        if self.supported_contact_observation is not None:
            self._deny(side, "Existing supported contact permits only zero-TX observation and static retention")
        context = self.supported_reacquisition
        if context is not None and (context["closure_finished"] or side != context["arm"]
                or kind != "gripper" or self.probe_mode != "close" or self.target != context["target_m"]
                or sum(row["attempted_frames"] for row in self.totals().values()) != context["attempts_before"]):
            self._deny(side, "Supported reacquisition permits only its current selected-jaw closure")

    def connect(self):
        # Keep the inherited passive construction and exact comm/bus guards.
        # Unlike Single.connect(), do not permanently disable one chosen side.
        _LinearHold.connect(self)
        self.quaternion = _SingleGripperPrepare._manufacturer_quaternion
        for side in SIDES:
            robot, jaw = self.robots[side], self.grippers[side]
            if self.feedback_policy is not None:
                from .coherent_feedback import install
                cfg = self.profile['arms'][side]
                if cfg['model'] != 'piper_x' or cfg['firmware'] != 'default':
                    raise RuntimeError('Task feedback grouping requires PiPER X/default')
                install(robot)
            for name in ("_send_msg", "_send_msgs"):
                sender = getattr(robot, name)
                setattr(robot, name, self._sender(side, "move", sender))
            sender = getattr(type(jaw), "_send_msg", None)
            if not callable(sender):
                raise RuntimeError("Manufacturer effector encoder unavailable")
            jaw._send_msg = self._sender(side, "gripper", sender.__get__(jaw, type(jaw)))
            sender_many = getattr(type(jaw), "_send_msgs", None)
            if callable(sender_many):
                jaw._send_msgs = self._sender(side, "gripper", sender_many.__get__(jaw, type(jaw)))

    def _sender(self, side, kind, original):
        def send(*args, **kwargs):
            self.require_no_loaded_pending_tx()
            self.require_supported_reacquisition_tx(side, kind)
            executor = self._executor()
            if executor is None:
                self.guard()
            else:
                executor.encoder_guard()
            if self.ticket is None or side != self.arm or self.kind != kind:
                self._deny(side, "SDK TX outside current pair action and selected encoder")
            return original(*args, **kwargs)
        return send

    def _wrap_bus(self, side, comm):
        super()._wrap_bus(side, comm)
        original = comm.send_bus.send
        def send(*args, **kwargs):
            self.require_no_loaded_pending_tx()
            self.require_supported_reacquisition_tx(side, self.kind)
            executor = self._executor()
            if executor is not None:
                return executor.send_frame(side, original, *args, **kwargs)
            # The host's fault Event must not require this action's RLock.
            self.guard()
            ticket = self.ticket
            if (ticket is None or ticket["side"] != side or side != self.arm
                    or ticket["thread"] != threading.get_ident()):
                self._deny(side, "Pair bus TX outside the current action ticket")
            # A durable host guard may block on its database. Read the parser
            # copies AFTER it returns, not the snapshot used before dispatch.
            # snapshot() is RX-only and does not call SDK send or acquire our
            # action lock. No journal/host callback may run after this check.
            states = {name: arms.snapshot(self.robots[name], self.grippers[name]) for name in SIDES}
            self._checked_after_guard(states)
            self.report.setdefault("frame_preflight_feedback", []).append({
                "side": side, "frame_index": ticket["bus_calls"], "arms": copy.deepcopy(states)})
            # Include validation/report-copy time in the final freshness check.
            self.check_freshness(states)
            return original(*args, **kwargs)
        comm.send_bus.send = send

    def checked(self, states):
        executor = self._executor()
        if executor is None:
            self.guard()
        else:
            executor.encoder_guard()
        result = self._checked_after_guard(states)
        if self.retention_trace is not None:
            at = time.time()
            if self.retention_trace and at < self.retention_trace[-1]["observed_at_s"]:
                raise RuntimeError("Retention observation clock regressed")
            if not self.retention_trace or at > self.retention_trace[-1]["observed_at_s"]:
                if len(self.retention_trace) >= 5000:
                    raise RuntimeError("Retention trace exceeded its bounded sample budget")
                self.retention_trace.append({"observed_at_s": at, "arms": copy.deepcopy(states)})
        if self.probe_trace is not None:
            label = "baseline" if self.sent_at is None else "post"
            if label == "post" and not self.all_after(states, self.sent_at):
                if self.probe_trace["post"]:
                    raise RuntimeError("Probe post-send feedback timestamps regressed")
                return result
            samples = self.probe_trace[label]
            at = time.time()
            if samples and at < samples[-1]["observed_at_s"]:
                raise RuntimeError("Probe observation clock regressed")
            if not samples or at > samples[-1]["observed_at_s"]:
                if sum(map(len, self.probe_trace.values())) >= 5000:
                    raise RuntimeError("Probe trace exceeded its bounded sample budget")
                samples.append({"observed_at_s": at, "arms": copy.deepcopy(states)})
        return result

    def checked_current_completion(self):
        """Post-classification RX only: host guard BEFORE collecting feedback.

        A guard may read/hash durable evidence. Its latency must not age an
        already-read sample. Freshness/health/envelope limits are unchanged;
        stale actual feedback still fails, with no waiting, TX or retries.
        """
        if self._executor() is not None or self.probe_trace is not None or self.retention_trace is not None:
            raise RuntimeError("Current completion read requires finished probe classification")
        self.guard()
        states = self.read()
        checked = self._checked_after_guard(states)
        self.report["completion_feedback_order"] = "host_guard_then_read_then_validate"
        return checked

    def _stationary_body_reference(self, side, ordinary_reference):
        # An audited re-closing changes only the jaw reference. Reusing a new
        # connection/action body origin would intersect two equal envelopes
        # and reject feedback still inside the immutable audited body bound.
        if self.supported_reacquisition is not None:
            return self.supported_reacquisition["body_anchors"][side]
        if self.supported_contact_observation is not None:
            return self.supported_contact_observation["body_anchors"][side]
        return super()._stationary_body_reference(side, ordinary_reference)

    def _checked_after_guard(self, states):
        """Pure feedback checks; never reenter the host guard or sender."""
        if self.auxiliary_executor is not None:
            self._executor()  # Reject overlapping trusted operation contexts.
            return self.auxiliary_executor.check_states(states)
        for side in SIDES:
            health = arms.control_health(states[side], allowed_control_modes=(1,), require_enabled=True)
            if not health["healthy"]:
                raise RuntimeError("Pair requires both CAN arms and all drivers/jaws enabled: %s %r" %
                                   (side, health["reasons"]))
            self.boundary_origins.setdefault(side, list(states[side]["joints_rad"]))
        result = (super().checked(states) if self.joint_executor is None
                  else self.joint_executor.check_states(states))
        observed_contact = self.supported_contact_observation
        if observed_contact is not None:
            for side in SIDES:
                jaw = (observed_contact["jaw_anchor_m"] if side == observed_contact["arm"]
                       else observed_contact["body_anchors"][side]["width_m"])
                if abs(states[side]["gripper"]["width_m"]-jaw) > BOUNDS["jaw_span_m"]:
                    raise RuntimeError("Existing supported-contact jaw left its fixed observed anchor: " + side)
        reacquisition = self.supported_reacquisition
        if reacquisition is not None:
            # This survives per-action resets and candidate installation. The
            # opening changed only a jaw reference, never either body's origin.
            for side in SIDES:
                original = reacquisition["body_anchors"][side]
                deviation = anchor_deviation(original, states[side])
                if (not joints_within(self.feedback_policy, side, original["joints_rad"], states[side]["joints_rad"])
                        or deviation["position_m"] > BOUNDS["position_span_m"]
                        or deviation["rotation_rad"] > rotation_tolerance(self.feedback_policy, side)):
                    raise RuntimeError("Supported reacquisition body left its original anchor: " + side)
                selected = side == reacquisition["arm"]
                if (not selected or reacquisition["closure_finished"]
                        or sum(row["sent_frames"] for row in self.totals().values()) == reacquisition["sent_before"]):
                    jaw_anchor = reacquisition["jaw_anchor_m"] if selected else original["width_m"]
                    if abs(states[side]["gripper"]["width_m"]-jaw_anchor) > BOUNDS["jaw_span_m"]:
                        raise RuntimeError("Supported reacquisition jaw left its observed anchor: " + side)
        if self.probe_mode is not None and self.sent_at is None:
            width = states[self.arm]["gripper"]["width_m"]
            # Bound both the requested and manufacturer-encoded widths before
            # every frame. Quantization cannot enlarge a 5 mm probe or release.
            encoded = round(self.target * 1e6) / 1e6
            pending = self.grasps[self.arm]
            if pending is not None and pending.get("audited_opening_continuation") is not None:
                _require_later_recovery_target(pending["audited_opening_continuation"], self.target)
            for target in (self.target, encoded):
                before, after = (width, target) if self.probe_mode == "close" else (target, width)
                if not probe_closure_within_bound(before, after):
                    raise RuntimeError("Probe requires a strictly directional jaw step within 5 mm")
        # active becomes true BEFORE the SDK sends its frame. Until that frame
        # has actually returned, the old unresolved target must remain frozen.
        for side, pending in self.grasps.items():
            if pending is None:
                continue
            deviation = anchor_deviation(grasp_body_anchor(pending), states[side])
            delegated = (self.joint_executor is not None and self.active
                         and self.joint_executor.loaded_body_delegated(side))
            if not delegated and (not joints_within(self.feedback_policy, side, grasp_body_anchor(pending)["joints_rad"], states[side]["joints_rad"])
                    or deviation["position_m"] > BOUNDS["position_span_m"]
                    or deviation["rotation_rad"] > rotation_tolerance(self.feedback_policy,side)):
                raise RuntimeError("Grasp arm changed from its original candidate anchor: " + side)
            releasing = (self.active and side == self.arm and self.probe_mode == "release"
                         and self.sent_at is not None)
            jaw_anchor = (pending["release_opening"]["observed_width_m"]
                          if pending["status"] == "release_opened" else pending["original_anchor"]["width_m"])
            if pending["status"] == "recovery_residual" and pending.get("audited_opening_continuation") is not None:
                jaw_anchor = pending["audited_opening_continuation"]["residual_jaw_anchor"]["width_m"]
            if not releasing and abs(states[side]["gripper"]["width_m"]-jaw_anchor) > BOUNDS["jaw_span_m"]:
                raise RuntimeError("Unresolved probe jaw or retained jaw changed from its settled observation: " + side)
            if pending.get("retention_contract") is not None and time.time() >= pending["retention_contract"]["valid_until"]:
                raise RuntimeError("Static retention reached its frozen deadline: " + side)
        if self.idle_anchor is not None:
            for side in SIDES:
                current, origin = states[side], self.idle_anchor[side]
                body_origin = self._stationary_body_reference(side, origin)
                commanded = self.active and side == self.arm
                if not (commanded and self.kind == "move"):
                    if (not joints_within(self.feedback_policy, side, current["joints_rad"], body_origin["joints_rad"])
                            or math.dist(current["pose_m_rad"][:3], body_origin["pose_m_rad"][:3]) > BOUNDS["position_span_m"]
                            or self.rotation_distance(current["pose_m_rad"], body_origin["pose_m_rad"]) > rotation_tolerance(self.feedback_policy,side)):
                        raise RuntimeError(side + " changed from persistent stationary arm anchor")
                if not (commanded and self.kind == "gripper"):
                    if abs(current["gripper"]["width_m"]-origin["gripper"]["width_m"]) > LIMITS["gripper_m"]:
                        raise RuntimeError(side + " changed from persistent stationary jaw anchor")
        self.check_freshness(states)
        return result

    def check_selected_joint_bounds(self, state, violations):
        # Retain the original discrepancy cap across role switches and calls.
        self.boundary_reference = self.boundary_origins[self.arm][:]
        return super().check_selected_joint_bounds(state, violations)

    def dispatch(self):
        self.guard()
        return super().dispatch()

    def reset_action(self, arm, kind, target):
        # SDK closures reference this object. Keep connections, locks, modes,
        # original boundary evidence and every violation for its whole lifetime.
        fresh = _SingleSupervisedAction(self.profile, self.journal, arm, kind, target)
        for side in SIDES:
            for key, value in self.counts[side].items():
                self.archived_counts[side][key] += value
        for name in ("arm", "kind", "target", "passive_arm", "active", "dispatched", "sent_at",
                     "mode_l_confirmed", "baseline_window", "dispatch_anchor", "boundary_reference",
                     "frame_limit", "counts", "kind_counts", "max_drift", "report"):
            setattr(self, name, getattr(fresh, name))
        self.anchor = copy.deepcopy(self.idle_anchor)

    def totals(self):
        return {side: {key: self.archived_counts[side][key] + value
                       for key, value in self.counts[side].items()} for side in SIDES}


class GuardedPairDevice:
    """Long-lived real SDK adapter; never automatically enables or retries."""

    def __init__(self, profile, journal, guard=lambda: None):
        if not callable(journal) or not callable(guard):
            raise ValueError("Synchronous journal and guard callables required")
        self._action = _PairAction(profile, journal, guard)
        self._opened = self._closed = False
        self._fault = None
        self._previous = None
        self._sequence = 0
        self._baseline_duration = self._advances = 0
        self._operation = threading.Lock()
        self._fault_feedback = FaultFeedback(BOUNDS["feedback_age_s"], BOUNDS["feedback_skew_s"])
        self._connection_id = "pair_sdk_" + uuid.uuid4().hex
        self._joint_cache = {side: None for side in SIDES}
        self._joint_observations = {}
        self._initialization_observations = {}
        self._joint_initializations = {side: None for side in SIDES}
        self._release_confirmations = {side: None for side in SIDES}
        self._task_ready = False
        self._preparation = None
        self._supported_recovery_opening = None

    @property
    def capabilities(self):
        return dict(CAPABILITIES)

    @property
    def unresolved_gripper_probe(self):
        return copy.deepcopy(self._action.unresolved_probe)

    @property
    def unresolved_gripper_probes(self):
        return {side: copy.deepcopy(item) for side, item in self._action.grasps.items()
                if item is not None and item["status"] in (
                    "contact_candidate", "release_opened", "loaded_pending_visual")}

    @property
    def grasp_states(self):
        return copy.deepcopy(self._action.grasps)

    def _connected_usable(self):
        """Shared host-only prerequisite for preparation and non-actuating queries."""
        if not self._opened or self._closed or self._fault is not None:
            raise RuntimeError("Pair device unavailable: " + str(self._fault or "not open or closed"))
        if self._action.violations:
            self._fault = "persistent CAN violations"
            raise RuntimeError("Pair device has persistent CAN violations")
        try:
            self._action.guard()
        except BaseException as exc:
            self._fault = str(exc)
            raise

    def _usable(self):
        self._connected_usable()
        if not self._task_ready:
            raise RuntimeError("Pair is connected for preparation; task readiness has not been established")

    def _sample(self, states):
        self._action.check_freshness(states)
        self._sequence += 1
        self._previous = copy.deepcopy(states)
        self._fault_feedback.seed(states, time.time())
        return {"arms": copy.deepcopy(states), "stationary_observed": True,
                "task_ready": self._task_ready,
                "sequence": self._sequence, "baseline_duration_s": self._baseline_duration,
                "feedback_advances": self._advances, "capabilities": self.capabilities,
                "grasp_verified": False, "physical_stop_verified": None,
                "scope": "Fresh feedback against continuously retained stationary anchors; no loaded hold or stop qualification"}

    def open(self):
        with self._operation:
            if self._opened or self._closed or self._fault is not None:
                raise RuntimeError("Pair device can only be opened once")
            action = self._action
            try:
                action.guard()
                action.connect()
                action.prepare()
                action.idle_anchor = copy.deepcopy(action.anchor)
                self._baseline_duration = action.report["baseline_duration_s"]
                self._advances = action.report["baseline_feedback_advances"]
                self._opened = True
                self._task_ready = True
                return self._sample(action.report["after"])
            except BaseException as exc:
                self._fault = str(exc)
                raise

    def _observe(self):
        self._usable()
        action = self._action
        deadline = time.monotonic() + BOUNDS["feedback_age_s"]
        while True:
            states = action.checked(action.read())
            if self._previous is None or action.advanced(self._previous, states):
                return self._sample(states)
            if time.monotonic() >= deadline:
                raise RuntimeError("No independently advancing pair feedback; old samples cannot renew hold")
            time.sleep(BOUNDS["poll_s"])

    def observe(self):
        with self._operation:
            try:
                if self._opened and not self._task_ready:
                    from .pair_preparation import observe_preparation
                    return observe_preparation(self)
                return self._observe()
            except BaseException as exc:
                self._fault = str(exc)
                raise

    def connect_for_preparation(self):
        """Zero-TX connection and stationary feedback, without requiring ready jaws."""
        from .pair_preparation import connect_for_preparation
        with self._operation:
            return connect_for_preparation(self)

    def prepare_gripper(self, arm):
        """One empty jaw's existing measured-width preparation on this connection."""
        from .pair_preparation import prepare_gripper
        with self._operation:
            self._action.require_no_loaded_pending_tx()
            return prepare_gripper(self, arm)

    def promote_ready(self):
        """Establish the original strict task baseline without reconnecting or TX."""
        from .pair_preparation import promote_ready
        with self._operation:
            return promote_ready(self)

    def inspect_joint_limits(self):
        """Query all twelve joint limits once on these retained connections."""
        from .pair_limits import inspect_joint_limits
        with self._operation:
            self._action.require_no_loaded_pending_tx()
            return inspect_joint_limits(self)

    def observe_fault_feedback(self):
        """Copy existing RX caches despite a latched fault; never authorize TX.

        No dispatch guard, SDK query, reconnect, sleep, or action lock is used.
        The lifetime lock is attempted once so an active operation/cleanup
        defers this diagnostic instead of delaying cancellation or exit.
        """
        if not self._operation.acquire(blocking=False):
            return {"status": "deferred", "reason": "device_operation_active",
                    "hardware_commands_sent": 0, "motion_permitted": False,
                    "physical_stop_verified": None}
        try:
            if not self._opened or self._closed:
                return {"status": "unavailable", "reason": "device_not_open_or_closed",
                        "hardware_commands_sent": 0, "motion_permitted": False,
                        "physical_stop_verified": None}
            states, errors = {}, {}
            for side in SIDES:
                try:
                    robot = self._action.robots[side]
                    states[side] = arms.snapshot(robot, self._action.grippers.get(side))
                except Exception as exc:
                    states[side] = None
                    errors[side] = type(exc).__name__ + ": " + str(exc)
            return self._fault_feedback.capture(states, time.time(), errors)
        finally:
            self._operation.release()

    def execute(self, arm, kind, target, *, operation=None):
        if arm not in SIDES or kind not in ("move", "gripper"):
            raise ValueError("Explicit left/right arm and move/gripper kind required")
        if kind == "move":
            if self._action.feedback_policy is not None:
                raise RuntimeError("Task observation profile requires RGB-supervised joint motion")
            target = arms._six_finite(target)
            _pose_frames(target)
        elif type(target) not in (int, float) or not math.isfinite(target) or not 0 <= target <= 0.055:
            raise ValueError("Jaw target must be 0..0.055 m at fixed nominal force 0.2")
        with self._operation:
            self._usable()
            if self._action.unresolved_probe is not None:
                raise RuntimeError("Unresolved gripper probe requires explicit same-arm release")
            if any(self._action.grasps.values()):
                if self._action.grasps[arm] is not None:
                    raise RuntimeError("Retained arm permits only explicit release or zero-TX observation")
                if kind != "move" or operation not in ("approach", "align"):
                    raise RuntimeError("Peer static retention permits only explicit unloaded approach/align or a probe")
            action = self._action
            try:
                self._observe()  # Validate old anchors before any per-action reset.
            except BaseException as exc:
                self._fault = str(exc)
                raise
            try:
                action.reset_action(arm, kind, copy.deepcopy(target))
                action.prepare()
                status = action.perform()
                after = copy.deepcopy(action.report["after"])
                if kind == "move":
                    before = action.report["dispatch_feedback"][arm]
                    action.report["motion_effect"] = {
                        **measure_motion_effect(before["pose_m_rad"], target, after[arm]["pose_m_rad"]),
                        "feedback_source": "dispatch_feedback and after for the selected arm",
                        "before_fragment_timestamps_s": copy.deepcopy(before["fragment_timestamps_s"]),
                        "after_fragment_timestamps_s": copy.deepcopy(after[arm]["fragment_timestamps_s"])}
                arrived = (action.report.get("controller_at_target") is True
                           and action.report.get("feedback_all_after_send") is True)
                if kind == "move":
                    error = action.report["pose_error"]
                    arrived = arrived and error["position_m"] <= 0.005 and error["rotation_rad"] <= 0.05
                else:
                    arrived = arrived and action.report.get("width_error_m", math.inf) <= 0.002
                if not arrived:
                    raise RuntimeError("Stable feedback did not establish target arrival; contact-limited jaw closure is unsupported")
                # Only commanded components gain a new anchor. Repeated actions
                # must not accumulate drift of the peer, jaw or stationary arm.
                if kind == "move":
                    for key in ("pose_m_rad", "joints_rad"):
                        action.idle_anchor[arm][key] = copy.deepcopy(after[arm][key])
                else:
                    action.idle_anchor[arm]["gripper"] = copy.deepcopy(after[arm]["gripper"])
                action.active = False
                action.anchor = copy.deepcopy(action.idle_anchor)
                action.checked(after)
                self._baseline_duration = action.report["observed_stable_duration_s"]
                self._advances = action.report["observed_feedback_advances"]
                sample = self._sample(after)
                action.report.update(ok=True, status="target_arrived_and_stationary_observed",
                                     arrival_confirmed=True, observation_status=status, sample=sample)
            except BaseException as exc:
                self._fault = str(exc)
                action.report.update(ok=False, status="pair_device_fault", arrival_confirmed=False, sample=None)
                action.report["errors"].append({"type": type(exc).__name__, "detail": str(exc)})
            finally:
                action.ticket = None
                if kind == "move" and action.counts[arm]["attempted_frames"]:
                    # A Cartesian mode/target attempt invalidates any earlier
                    # same-mode joint-cache provenance, including partial TX.
                    self._joint_cache[arm] = None
                    self._joint_initializations[arm] = None
            result = copy.deepcopy(action.report)
            result.update(arm=arm, kind=kind, accepted=None, capabilities=self.capabilities,
                          physical_stop_verified=None, physical_stop_supported=False, grasp_verified=False,
                          transmission_counts=copy.deepcopy(action.counts),
                          session_transmission_counts=action.totals(),
                          hardware_commands_sent=sum(v["sent_frames"] for v in action.counts.values()),
                          passive_arm_commands_sent=action.counts[action.passive_arm]["sent_frames"],
                          guard_violations=copy.deepcopy(action.violations),
                          connected=not self._closed)
            return result

    def joint_binding(self, arm):
        """Host-only identity facts; an unknown controller target stays unknown."""
        if arm not in SIDES:
            raise ValueError("Explicit arm required")
        config = self._action.profile["arms"][arm]
        return {"connection_id": self._connection_id, "model": config["model"],
                "firmware_profile": config.get("firmware", "default"),
                "cached_target": copy.deepcopy(self._joint_cache[arm])}

    def joint_initialization_sources(self):
        """Host-only live provenance, invalid once its exact cache is replaced."""
        result = dict.fromkeys(SIDES)
        for side in SIDES:
            source, cache = self._joint_initializations[side], self._joint_cache[side]
            if type(source) is dict and cache is not None and cache == {
                    key: source.get(key) for key in ("event_id", "identity", "target_raw", "frame_receipts")}:
                result[side] = copy.deepcopy(source)
        return result

    def observe_joint(self, identity):
        """Issue a host-bound RX sample; this does not establish target cache history."""
        from .pair_joint_adapter import checked_identity, pair_sample
        identity = checked_identity(self, identity)
        with self._operation:
            self._usable()
            try:
                states = self._observe()["arms"]
                sample = pair_sample(identity, states, now=time.time())
                self._joint_observations[sample["sample_id"]] = copy.deepcopy(sample)
                if len(self._joint_observations) > 16:
                    del self._joint_observations[next(iter(self._joint_observations))]
                return sample
            except BaseException as exc:
                self._fault = str(exc)
                raise

    def observe_initialization(self, identity):
        """Issue preparation-aware RX evidence without creating target history."""
        from .pair_joint_adapter import checked_identity, pair_sample
        identity = checked_identity(self, identity)
        with self._operation:
            self._connected_usable()
            try:
                if self._task_ready:
                    observed = self._observe()
                else:
                    from .pair_preparation import observe_preparation
                    observed = observe_preparation(self)
                sample = pair_sample(identity, observed["arms"], now=time.time())
                self._initialization_observations[sample["sample_id"]] = copy.deepcopy(sample)
                if len(self._initialization_observations) > 16:
                    del self._initialization_observations[next(iter(self._initialization_observations))]
                return sample
            except BaseException as exc:
                self._fault = str(exc)
                raise

    def initialize_joint_target(self, arm, *, context, event_id, deadline_at):
        """One host-resolved supervised first target on the existing connection."""
        from .pair_initialization import initialize_joint_target
        with self._operation:
            self._action.require_no_loaded_pending_tx()
            return initialize_joint_target(self, arm, context=context, event_id=event_id,
                                           deadline_at=deadline_at)

    def execute_joint(self, arm, target, *, context, event_id, deadline_at,
                      operation="approach", recovery_mode=None, hold_bridge=None):
        """One host-resolved MOVE_J attempt on these same guarded SDK objects."""
        from .pair_joint_adapter import execute_joint
        with self._operation:
            return execute_joint(self, arm, target, context=context, event_id=event_id,
                                 deadline_at=deadline_at, operation=operation,
                                 recovery_mode=recovery_mode, hold_bridge=hold_bridge)

    def observe_grasp(self, arm, *, identity, probe_event_id):
        """Collect new stationary evidence without adopting or changing a target."""
        return self._observe_grasp(arm, identity=identity, probe_event_id=probe_event_id,
                                   upgrade=False, probe_trace_sha256=None, deadline_at=None)

    def confirm_loaded_response(self, arm, *, identity, action_event_id, plan_sha256):
        """Host-only completion after its durable new-RGB response evidence.

        Only this live adapter's pending event can become locally retained.
        The call sends nothing and cannot establish visual truth, support force,
        physical stop or a global capability. Original probe history survives.
        """
        identity = identity_checked(identity, arm)
        from .task_roles import PROFILE_KEY as TASK_ROLES_KEY, resolve_task_roles
        with self._operation:
            self._usable()
            action = self._action
            result = {"ok": False, "arm": arm, "completion_mode": "confirm_loaded_response",
                "hardware_commands_sent": 0, "target_calls_sent": 0,
                "physical_stop_verified": None, "contact_support_verified": False,
                "grasp_verified": False, "object_progress_measurement": None}
            try:
                record = action.grasps[arm]
                pending = record.get("loaded_pending") if record is not None else None
                if (arm != resolve_task_roles(action.profile.get(TASK_ROLES_KEY, {}))[0]
                        or record is None or record["status"] != "loaded_pending_visual"
                        or record["identity"] != identity or type(pending) is not dict
                        or pending["action_event_id"] != action_event_id or pending["plan_sha256"] != plan_sha256
                        or pending["local_anchor"] != record.get("local_anchor")):
                    raise RuntimeError("Loaded confirmation needs this exact pending event, identity and plan")
                sample = self._observe()
                action.check_freshness(sample["arms"])
                record.update(status="retained_local", loaded_pending=None)
                result.update(ok=True, status="retained_local", sample=sample,
                    confirmed_loaded_receipt=copy.deepcopy(pending))
            except BaseException as exc:
                self._fault = str(exc)
                result.update(error=type(exc).__name__+": "+str(exc), status="pair_device_fault")
            result.update(grasp_states=self.grasp_states, session_transmission_counts=action.totals())
            return copy.deepcopy(result)

    def observe_release(self, arm, *, identity, probe_event_id, release_trace_sha256):
        """Collect a new zero-TX window after the exact last tracked opening."""
        if (type(release_trace_sha256) is not str or len(release_trace_sha256) != 64
                or any(char not in "0123456789abcdef" for char in release_trace_sha256)):
            raise ValueError("Exact latest opening trace hash required")
        return self._observe_grasp(arm, identity=identity, probe_event_id=probe_event_id,
            upgrade=False, probe_trace_sha256=None, deadline_at=None,
            release_trace_sha256=release_trace_sha256)

    def retain_grasp(self, arm, *, identity, probe_event_id, probe_trace_sha256, deadline_at):
        """Monitor the existing candidate target; issue no CAN command or force change."""
        return self._observe_grasp(arm, identity=identity, probe_event_id=probe_event_id,
                                   upgrade=True, probe_trace_sha256=probe_trace_sha256,
                                   deadline_at=deadline_at)

    def _observe_grasp(self, arm, *, identity, probe_event_id, upgrade,
                       probe_trace_sha256, deadline_at, release_trace_sha256=None):
        identity = identity_checked(identity, arm)
        if (type(probe_event_id) is not str or not 1 <= len(probe_event_id) <= 128
                or probe_event_id.strip() != probe_event_id
                or any(ord(char) < 32 or ord(char) == 127 for char in probe_event_id)):
            raise ValueError("Host-resolved original probe event id required")
        if upgrade and (type(deadline_at) not in (int, float) or not math.isfinite(deadline_at)):
            raise ValueError("Frozen finite task deadline required")
        with self._operation:
            self._usable()
            action = self._action
            record = action.grasps[arm]
            if record is None:
                raise RuntimeError("Static observation requires this arm's existing candidate or retention")
            releasing = record["status"] == "release_opened"
            if record["status"] == "loaded_pending_visual":
                raise RuntimeError("Loaded response must be visually confirmed before grasp/release preparation")
            if upgrade and releasing:
                raise RuntimeError("An opened release cannot be upgraded back to static retention")
            if upgrade and record["status"] == "retained_local":
                raise RuntimeError("A transported grasp cannot be relabeled as original static retention")
            if release_trace_sha256 is not None and (
                    not releasing or record["release_opening"]["trace_sha256"] != release_trace_sha256):
                self._fault = "Release observation does not match the exact last opening"
                raise RuntimeError(self._fault)
            if (record["identity"] is not None and record["identity"] != identity
                    or record["probe_event_id"] is not None and record["probe_event_id"] != probe_event_id):
                if release_trace_sha256 is not None:
                    self._fault = "Release observation cannot change its original grasp binding"
                raise RuntimeError("Existing grasp owner/epoch/object/probe binding cannot change")
            if upgrade and probe_trace_sha256 != record["trace_sha256"]:
                raise RuntimeError("Retention must resolve the original local candidate trace")
            if upgrade and (deadline_at <= time.time() or
                    record.get("retention_contract") is not None and
                    deadline_at != record["retention_contract"]["valid_until"]):
                raise RuntimeError("Static retention cannot renew or replace its frozen task deadline")
            before_counts = action.totals()
            result = {"ok": False, "arm": arm, "completion_mode": "retain_static" if upgrade else
                      "observe_release" if release_trace_sha256 is not None else "observe_grasp",
                      "physical_stop_verified": None, "grasp_verified": False,
                      "contact_support_verified": False, "loaded": False, "target_calls_sent": 0}
            try:
                self._observe()
                if release_trace_sha256 is not None:
                    self._release_confirmations[arm] = None
                action.reset_action(arm, "gripper", record["release_opening"]["target_width_m"]
                                    if releasing else record["requested_width_m"])
                action.retention_trace = []
                action.prepare()  # Existing 3 s / 20-advance, dual-arm health and stationary checks.
                trace = action.retention_trace
                action.retention_trace = None
                trace_id = "static_" + uuid.uuid4().hex
                trace_sha = digest(trace)
                summary_args = dict(arm=arm, identity=identity,
                    probe_event_id=probe_event_id, trace_id=trace_id, trace_sha256=trace_sha,
                    original_anchor=grasp_body_anchor(record), samples=trace, now=time.time(),
                    feedback_policy=action.feedback_policy)
                measurement = (summarize_release_confirmation(release_opening=record["release_opening"],
                               **summary_args) if releasing else summarize_retention_trace(**summary_args))
                action.journal("static_grasp_trace", {"arm": arm, "identity": identity,
                    "probe_event_id": probe_event_id, "trace_id": trace_id, "sha256": trace_sha, "trace": trace})
                contract = None
                if upgrade:
                    issued_at = time.time()
                    code = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                            for name in ("pair_device.py", "retention_receipt.py", "contact_receipt.py")}
                    source = {"event_id": "static_issuance_" + uuid.uuid4().hex,
                              "trace_id": trace_id, "trace_sha256": trace_sha,
                              "identity": identity, "issued_at": issued_at,
                              "probe_event_id": probe_event_id,
                              "original_anchor": record["original_anchor"]}
                    contract = {"identity": identity, "contract_id": "static_contract_" + uuid.uuid4().hex,
                        "adapter_id": "GuardedPairDevice.existing_target_monitored",
                        "adapter_code_sha256": digest(code),
                        "controller_id": "configured_controller_" + digest(action.profile["arms"][arm])[:32],
                        "issued_at": issued_at, "valid_until": deadline_at,
                        "mode": "existing_target_monitored", "probe_event_id": probe_event_id,
                        "requested_width_m": record["requested_width_m"], "failure_behavior": "latch_no_new_targets",
                        "source": {"event_id": source["event_id"], "artifact_sha256": digest(source),
                                   "trace_id": trace_id, "trace_sha256": trace_sha}}
                    contract["artifact_sha256"] = digest(contract)
                    action.journal("static_grasp_contract", {"contract": contract, "source": source, "code": code})
                # Hashing, summary generation and durable recording cannot turn
                # their older samples into current feedback. Re-read and retain
                # the original timestamps, then enforce the same 100 ms bound.
                after = action.checked(action.read())
                now = time.time()
                if not 0 <= now-measurement["ended_at"] <= BOUNDS["feedback_age_s"]:
                    raise RuntimeError("Static trace became stale during receipt processing")
                if upgrade and now >= deadline_at:
                    raise RuntimeError("Static retention expired before its receipt completed")
                self._baseline_duration = measurement["ended_at"]-measurement["started_at"]
                self._advances = measurement["feedback_advances"]
                sample = self._sample(after)
                record["identity"], record["probe_event_id"] = copy.deepcopy(identity), probe_event_id
                if upgrade:
                    record.update(status="retained_static", retention_contract=copy.deepcopy(contract))
                if release_trace_sha256 is not None:
                    self._release_confirmations[arm] = {
                        "identity": copy.deepcopy(identity), "probe_event_id": probe_event_id,
                        "release_opening": copy.deepcopy(record["release_opening"]),
                        "measurement": copy.deepcopy(measurement)}
                result.update(ok=True, status=record["status"], measurement=measurement,
                              retention_contract=contract, sample=sample,
                              target_may_remain_active=True, original_target_resent=False,
                              object_release_verified=None,
                              scope="Measured opened jaw only; host visual confirmation still required" if releasing else
                              "Static original support and peer unloaded preparation only; no loaded contact or stop")
                if action.supported_contact_observation is not None:
                    result.update(candidate_basis="existing_target_observation",
                                  scope="Same existing target, static observation only; all new TX blocked")
                if releasing:
                    result["release_opening"] = copy.deepcopy(record["release_opening"])
            except BaseException as exc:
                self._fault = str(exc)
                result.update(error=type(exc).__name__ + ": " + str(exc), status="pair_device_fault")
            finally:
                action.retention_trace = None
                action.ticket = None
            after_counts = action.totals()
            result.update(hardware_commands_sent=sum(after_counts[side]["sent_frames"]-before_counts[side]["sent_frames"]
                          for side in SIDES), session_transmission_counts=after_counts,
                          grasp_states=self.grasp_states, guard_violations=copy.deepcopy(action.violations))
            return copy.deepcopy(result)

    def finalize_release(self, arm, *, identity, probe_event_id, release_trace_sha256,
                         confirmation_trace_sha256):
        """Host-only second phase, after its durable visual release confirmation.

        This token proves only adapter-owned feedback continuity. No Boolean or
        dictionary supplied here can prove object separation or physical stop.
        """
        identity = identity_checked(identity, arm)
        with self._operation:
            self._usable()
            action = self._action
            result = {"ok": False, "arm": arm, "completion_mode": "finalize_release",
                "hardware_commands_sent": 0, "target_calls_sent": 0,
                "physical_stop_verified": None, "object_release_verified": None,
                "grasp_verified": False, "target_may_remain_active": True,
                "target_cancellation_verified": False}
            try:
                record, token = action.grasps[arm], self._release_confirmations[arm]
                if (record is None or record["status"] != "release_opened" or token is None
                        or record["identity"] != identity or record["probe_event_id"] != probe_event_id
                        or token["identity"] != identity or token["probe_event_id"] != probe_event_id
                        or record["release_opening"] != token["release_opening"]
                        or release_trace_sha256 != record["release_opening"]["trace_sha256"]
                        or confirmation_trace_sha256 != token["measurement"]["trace_sha256"]):
                    raise RuntimeError("Release finalization requires the exact current adapter observation token")
                sample = self._observe()
                if not 0 <= time.time()-token["measurement"]["ended_at"] <= BOUNDS["feedback_age_s"]:
                    raise RuntimeError("Release confirmation trace became stale before finalization")
                if abs(sample["arms"][arm]["gripper"]["width_m"]-
                       record["release_opening"]["target_width_m"]) > LIMITS["gripper_m"]:
                    raise RuntimeError("Released jaw left its measured target arrival")
                # _observe read advancing feedback and checked both original body
                # anchors and the latest fixed jaw anchor. No callback or I/O may
                # turn that sample old between this check and the local clear.
                action.check_freshness(sample["arms"])
                action.grasps[arm] = None
                self._release_confirmations[arm] = None
                result.update(ok=True, status="release_record_cleared", sample=sample,
                              release_trace_sha256=release_trace_sha256,
                              confirmation_trace_sha256=confirmation_trace_sha256)
            except BaseException as exc:
                self._fault = str(exc)
                result.update(error=type(exc).__name__+": "+str(exc), status="pair_device_fault")
            result.update(grasp_states=self.grasp_states, session_transmission_counts=action.totals())
            return copy.deepcopy(result)

    def execute_gripper_probe(self, arm, target):
        """Send one small closure and classify measured response, never grasp."""
        return self._bounded_gripper_action(arm, target, release=False)

    def release_gripper_probe(self, arm, target):
        """Open once; tracked objects remain unresolved until host confirmation."""
        return self._bounded_gripper_action(arm, target, release=True)

    def recover_supported_gripper(self, arm, target, *, source_receipt):
        """Host-only audited residual opening; never transfer a grasp/cache."""
        return self._bounded_gripper_action(arm, target, release=True, recovery_source=source_receipt)

    def reacquire_supported_gripper(self, arm, target, *, source_receipt):
        """One of at most three audited closures; no empty/released/grasp claim."""
        return self._bounded_gripper_action(arm, target, release=False, reacquisition_source=source_receipt)

    def observe_supported_contact(self, arm, *, source_receipt):
        """Create a distinct current-contact candidate from new RX only.

        The old failed send remains a failed send. The manager binds its full
        history and bilateral-contact evidence; this connection can thereafter
        only observe/retain the same target, never close, release or move.
        """
        from .supported_contact_measurement import validate_source, measure
        with self._operation:
            self._usable()
            action = self._action
            result = {"ok": False, "arm": arm, "completion_mode": "supported_contact_observe",
                "kind": "observation", "candidate_basis": "existing_target_observation",
                "target_calls_sent": 0, "physical_stop_verified": None, "grasp_verified": False,
                "enable_commands_sent": 0, "stop_commands_sent": 0, "retries": 0,
                "contact_support_verified": False, "loaded": False, "nominal_force_N": None,
                "force_calibrated": False, "automatic_retry": False, "errors": [],
                "original_target_resent": False, "empty_jaw_verified": None, "object_release_verified": None}
            before = action.totals()
            try:
                if (action.supported_contact_observation is not None or action.supported_reacquisition is not None
                        or self._supported_recovery_opening is not None or any(action.grasps.values())
                        or any(self._joint_cache.values()) or any(v for row in before.values() for v in row.values())):
                    raise RuntimeError("Existing-contact observation requires a fresh connection without prior sends or candidates")
                admission, anchors = validate_source(source_receipt, arm, time.time())
                action.supported_contact_observation = {"arm": arm, "body_anchors": anchors,
                    "jaw_anchor_m": admission["residual_jaw_anchor"]["width_m"],
                    "audit": admission, "source_sha256": digest(source_receipt)}
                self._observe()
                action.reset_action(arm, "gripper", admission["prior_sent_target_m"])
                action.retention_trace = []
                action.prepare()  # Fresh 3 s/20 advances; no dispatch or force command.
                trace = action.retention_trace
                action.retention_trace = None
                trace_sha = digest(trace)
                candidate, measurement = measure(arm=arm, source=source_receipt, samples=trace,
                    trace_id="existing_contact_"+uuid.uuid4().hex, trace_sha256=trace_sha,
                    now=time.time(), feedback_policy=action.feedback_policy)
                action.journal("existing_supported_contact_trace", {"arm": arm, "sha256": trace_sha,
                    "trace": trace, "admission": copy.deepcopy(admission), "candidate": candidate})
                after = action.checked_current_completion()
                if (action.totals() != before or not 0 <= time.time()-measurement["ended_at"] <= BOUNDS["feedback_age_s"]
                        or abs(after[arm]["gripper"]["width_m"]-candidate["observed_width_m"]) > BOUNDS["jaw_span_m"]
                        or after[arm]["gripper"]["width_m"]-admission["prior_sent_target_m"] <= LIMITS["gripper_m"]):
                    raise RuntimeError("Current supported-contact observation changed before completion")
                # A new candidate has a new measured jaw reference. Both body
                # anchors and the historical send/failed receipt stay intact.
                action.supported_contact_observation["jaw_anchor_m"] = candidate["observed_width_m"]
                action.grasps[arm] = {"status": "contact_candidate", "arm": arm,
                    "candidate_basis": "existing_target_observation",
                    "requested_width_m": admission["prior_sent_target_m"],
                    "observed_width_m": candidate["observed_width_m"], "sent_at": admission["failed_sent_at"],
                    "trace_sha256": trace_sha, "original_anchor": copy.deepcopy(measurement["anchor"]),
                    "identity": None, "probe_event_id": None, "current_contact_candidate": copy.deepcopy(candidate),
                    "candidate_measurement": copy.deepcopy(measurement), "target_may_remain_active": True,
                    "grasp_verified": False, "physical_stop_verified": None}
                after = action.checked_current_completion()
                self._baseline_duration = measurement["ended_at"]-measurement["started_at"]
                self._advances = measurement["feedback_advances"]
                result.update(ok=True, status="observed_supported_contact_candidate",
                    current_contact_candidate=candidate, candidate_measurement=measurement,
                    audited_existing_contact=copy.deepcopy(admission), sample=self._sample(after),
                    before=copy.deepcopy(action.report["before"]), after=copy.deepcopy(after),
                    baseline_duration_s=self._baseline_duration, baseline_feedback_advances=self._advances,
                    target_may_remain_active=True)
            except BaseException as exc:
                self._fault = str(exc)
                result.update(status="pair_device_fault", error=type(exc).__name__+": "+str(exc))
                result["errors"].append({"type": type(exc).__name__, "detail": str(exc)})
            finally:
                action.retention_trace = None
                action.ticket = None
            totals = action.totals()
            counts = {side: {key: value-before[side][key] for key, value in row.items()}
                      for side, row in totals.items()}
            result.update(hardware_commands_sent=sum(row["sent_frames"] for row in counts.values()),
                passive_arm_commands_sent=sum(row["sent_frames"] for side, row in counts.items() if side != arm),
                transmission_counts=counts, session_transmission_counts=totals,
                grasp_states=self.grasp_states, guard_violations=copy.deepcopy(action.violations))
            return copy.deepcopy(result)

    def _install_supported_reacquisition(self, arm, source, *, target):
        action = self._action
        existing = action.supported_reacquisition
        if existing is not None:
            if (existing["arm"] != arm or existing["source_sha256"] != digest(source)
                    or not existing["closure_finished"] or existing["last_outcome"] != "target_arrived"
                    or existing["probe_count"] >= 3 or any(action.grasps.values()) or any(self._joint_cache.values())):
                raise RuntimeError("Reacquisition needs the same source and preceding target arrival without a candidate; at most three probes")
            prior = min(existing["target_m"], round(existing["target_m"]*1e6)/1e6)
            if any(value >= prior for value in (target, round(target*1e6)/1e6)):
                raise RuntimeError("Next supported closure must be below the previous requested and encoded target; no replay")
            self._observe()  # Keep both old bodies and the previous measured jaw.
            existing.update(target_m=target, closure_finished=False, last_outcome=None,
                probe_count=existing["probe_count"]+1,
                attempts_before=sum(row["attempted_frames"] for row in action.totals().values()),
                sent_before=sum(row["sent_frames"] for row in action.totals().values()))
            return
        if (self._supported_recovery_opening is not None
                or any(action.grasps.values()) or any(self._joint_cache.values())
                or any(v for row in action.totals().values() for v in row.values())):
            raise RuntimeError("Reacquisition must be the first physical operation of the fresh connection")
        envelope = _validated_supported_reacquisition(source, arm, time.time())
        continuation = envelope.get("completed_probe_continuation")
        jaw = envelope["opening_jaw_anchor"]
        consumed = 0
        if continuation is not None:
            prior = continuation["prior_sent_target_m"]
            if any(value >= min(prior, round(prior*1e6)/1e6) for value in (target, round(target*1e6)/1e6)):
                raise RuntimeError("Completed probe continuation requires a strictly smaller new target; no replay")
            jaw = continuation["residual_jaw_anchor"]
            consumed = continuation["consumed_probe_count"]
        sample = self._observe()
        origins = {side: copy.deepcopy(source["candidate_measurement"]["anchor"])
                   if side == arm else measured_anchor(source["before"][side]) for side in SIDES}
        context = {"arm": arm, "target_m": target, "body_anchors": origins,
                   "jaw_anchor_m": jaw["width_m"],
                   "closure_finished": False, "audit": envelope, "source_sha256": digest(source),
                   "probe_count": consumed+1, "last_outcome": None, "attempts_before": 0, "sent_before": 0}
        # Keep this separately from grasps: no old residual is promoted into a
        # candidate, and the ordinary closure path still requires no candidate.
        action.supported_reacquisition = context
        action._checked_after_guard(sample["arms"])

    def _install_supported_residual(self, arm, source, *, target):
        action = self._action
        if (self._supported_recovery_opening is not None or any(action.grasps.values())
                or any(self._joint_cache.values())
                or any(v for row in action.totals().values() for v in row.values())):
            raise RuntimeError("Recovery must be the first physical operation of the fresh connection")
        continuation = _validated_opening_continuation(source, arm, target, time.time())
        sample = self._observe()
        measurement = source["candidate_measurement"]
        for side in SIDES:
            current = sample["arms"][side]
            origin = measurement["anchor"] if side == arm else measured_anchor(source["before"][side])
            deviation = anchor_deviation(origin, current)
            jaw_anchor = (continuation["residual_jaw_anchor"]["width_m"]
                          if side == arm and continuation is not None else origin["width_m"])
            if (not joints_within(action.feedback_policy, side, origin["joints_rad"], current["joints_rad"])
                    or deviation["position_m"] > BOUNDS["position_span_m"]
                    or deviation["rotation_rad"] > rotation_tolerance(action.feedback_policy, side)
                    or abs(current["gripper"]["width_m"]-jaw_anchor) > BOUNDS["jaw_span_m"]):
                raise RuntimeError("Current body/jaw differs from the audited supported residual: " + side)
        action.grasps[arm] = {"status": "recovery_residual", "arm": arm,
            "requested_width_m": source["candidate_probe"]["requested_width_m"],
            "observed_width_m": measurement["observed"]["width_m"],
            "sent_at": source["candidate_probe"]["sent_at"],
            "trace_sha256": source["candidate_probe"]["trace_sha256"],
            "original_anchor": copy.deepcopy(measurement["anchor"]),
            "identity": None, "probe_event_id": None,
            "failed_receipt_preserved": True, "grasp_verified": False,
            "target_may_remain_active": True, "physical_stop_verified": None}
        if continuation is not None:
            action.grasps[arm]["audited_opening_continuation"] = continuation

    def confirm_supported_recovery_release(self):
        """Fresh three-second RX-only trace; host separately records separation RGB."""
        with self._operation:
            self._usable()
            action = self._action
            opening = self._supported_recovery_opening
            if opening is None or any(action.grasps.values()):
                raise RuntimeError("Confirmation requires this connection's completed recovery opening")
            arm, target = opening["arm"], opening["target_width_m"]
            before = action.totals()
            try:
                self._observe()
                action.reset_action(arm, "gripper", target)
                action.retention_trace = []
                action.prepare()
                trace = action.retention_trace
                action.retention_trace = None
                trace_sha = digest(trace)
                measurement = summarize_retention_trace(arm=arm, identity=None, probe_event_id=None,
                    trace_id="recovery_release_"+uuid.uuid4().hex, trace_sha256=trace_sha,
                    original_anchor=opening["anchor"], samples=trace, now=time.time(),
                    feedback_policy=action.feedback_policy)
                action.journal("supported_recovery_separation_trace", {"trace": trace,
                    "trace_sha256": trace_sha, "opening": copy.deepcopy(opening)})
                after = action.checked_current_completion()
                if (action.totals() != before
                        or abs(after[arm]["gripper"]["width_m"]-target) > LIMITS["gripper_m"]):
                    raise RuntimeError("Recovery opening changed before separation confirmation")
                self._baseline_duration = measurement["ended_at"]-measurement["started_at"]
                self._advances = measurement["feedback_advances"]
                return {"ok": True, "status": "recovery_release_observed", "arm": arm,
                    "hardware_commands_sent": 0, "target_calls_sent": 0,
                    "measurement": measurement, "opening": copy.deepcopy(opening),
                    "sample": self._sample(after), "physical_stop_verified": None,
                    "grasp_verified": False, "session_transmission_counts": action.totals()}
            except BaseException as exc:
                self._fault = str(exc)
                raise
            finally:
                action.retention_trace = None

    def _bounded_gripper_action(self, arm, target, *, release, recovery_source=None, reacquisition_source=None):
        if arm not in SIDES:
            raise ValueError("Explicit left/right arm required")
        if type(target) not in (int, float) or not math.isfinite(target) or not 0 <= target <= .055:
            raise ValueError("Jaw target must be 0..0.055 m at fixed nominal force 0.2")
        with self._operation:
            self._usable()
            action = self._action
            action.require_no_loaded_pending_tx()
            if reacquisition_source is not None:
                if release or recovery_source is not None:
                    raise RuntimeError("Supported reacquisition only allows one new closure")
                try:
                    self._install_supported_reacquisition(arm, reacquisition_source, target=target)
                except BaseException as exc:
                    self._fault = str(exc)
                    raise
            elif action.supported_reacquisition is not None:
                raise RuntimeError("Supported reacquisition cannot send another jaw target")
            if recovery_source is not None:
                if not release:
                    raise RuntimeError("Recovery only allows a supported opening")
                try:
                    self._install_supported_residual(arm, recovery_source, target=target)
                except BaseException as exc:
                    self._fault = str(exc)
                    raise
            pending = action.grasps[arm]
            if release and pending is None:
                raise RuntimeError("Release requires an unresolved probe on the same arm")
            if release and ((pending["identity"] is None) != (pending["probe_event_id"] is None)):
                self._fault = "Release has an incomplete original grasp binding"
                raise RuntimeError(self._fault)
            if not release and (pending is not None or action.unresolved_probe is not None):
                raise RuntimeError("Unresolved gripper probe requires explicit same-arm release")
            try:
                self._observe()  # Retain the old anchors through admission.
            except BaseException as exc:
                self._fault = str(exc)
                raise
            try:
                if release:
                    self._release_confirmations[arm] = None
                action.reset_action(arm, "gripper", target)
                action.probe_mode = "release" if release else "close"
                action.probe_trace = {"baseline": [], "post": []}
                action.prepare()
                status = action.perform()
                trace = action.probe_trace
                action.probe_trace = None
                digest = hashlib.sha256(json.dumps(trace, sort_keys=True, separators=(",", ":"),
                                                   allow_nan=False).encode()).hexdigest()
                # Keep exact classifier inputs outside the size-bounded event
                # ledger. Generic feedback records precede checked() and cannot
                # reconstruct its observation timestamps or trace grouping.
                action.journal("bounded_probe_trace", {"arm": arm, "requested_width_m": target,
                    "sent_at": action.sent_at, "release": release, "sha256": digest,
                    "trace": trace})
                if release:
                    baseline = [s["arms"][arm]["gripper"]["width_m"] for s in trace["baseline"]]
                    cutoff = trace["post"][-1]["observed_at_s"]-BOUNDS["stable_s"]
                    start = max(i for i, s in enumerate(trace["post"]) if s["observed_at_s"] <= cutoff)
                    settled = [s["arms"][arm]["gripper"]["width_m"] for s in trace["post"][start:]]
                    opening = min(settled)-max(baseline)
                    arrived = (action.report.get("controller_at_target") is True
                               and action.report.get("feedback_all_after_send") is True
                               and all(abs(width-target) <= LIMITS["gripper_m"] for width in settled))
                    observation = {"outcome": "release_arrived" if arrived and opening > BOUNDS["jaw_span_m"] else "unconfirmed",
                                   "completion": "observation_only", "arrival_confirmed": arrived,
                                   "conservative_opening_displacement_m": opening,
                                   "actual_opening_increase_m": opening,
                                   "observed_width_m": settled[-1], "requested_width_m": target,
                                   "accepted": None, "grasp_verified": False,
                                   "contact_support_verified": False, "physical_stop_verified": None,
                                   "target_cancellation_verified": False, "automatic_retry": False}
                else:
                    body_reference = None
                    if reacquisition_source is not None:
                        body_reference = {side: {key: copy.deepcopy(anchor[key])
                                                 for key in ("joints_rad", "pose_m_rad")}
                                          for side, anchor in action.supported_reacquisition["body_anchors"].items()}
                    observation = classify_gripper_probe(arm=arm, requested_width_m=target,
                        sent_at=action.sent_at, baseline_samples=trace["baseline"], post_samples=trace["post"],
                        feedback_policy=action.feedback_policy, stationary_body_reference=body_reference)
                observation["trace_summary"] = {"sha256": digest,
                    "baseline_samples": len(trace["baseline"]), "post_samples": len(trace["post"]),
                    "baseline_started_at_s": trace["baseline"][0]["observed_at_s"],
                    "last_observed_at_s": trace["post"][-1]["observed_at_s"]}
                action.report["contact_observation"] = observation
                expected = ("release_arrived",) if release else ("target_arrived", "settled_contact_candidate")
                if observation["outcome"] not in expected or observation["completion"] != "observation_only":
                    raise RuntimeError("Probe response unconfirmed: " + str(observation.get("reasons", observation["outcome"])))
                candidate = observation["outcome"] == "settled_contact_candidate"
                if candidate or release:
                    cutoff = trace["post"][-1]["observed_at_s"]-BOUNDS["stable_s"]
                    start = max(i for i, item in enumerate(trace["post"]) if item["observed_at_s"] <= cutoff)
                    stable_trace = trace["post"][start:]
                    original = grasp_body_anchor(pending) if release else measured_anchor(stable_trace[0]["arms"][arm])
                    if reacquisition_source is not None:
                        original = copy.deepcopy(action.supported_reacquisition["body_anchors"][arm])
                        # The new contact creates only a new jaw observation.
                        # Its body remains tied to the original source probe.
                        original["width_m"] = stable_trace[0]["arms"][arm]["gripper"]["width_m"]
                    measured = summarize_retention_trace(arm=arm,
                        identity=pending.get("identity") if release else None,
                        probe_event_id=pending.get("probe_event_id") if release else None,
                        trace_id=("release_" if release else "probe_")+digest[:32], trace_sha256=digest,
                        original_anchor=original, samples=stable_trace,
                        now=stable_trace[-1]["observed_at_s"], release=release, feedback_policy=action.feedback_policy)
                    if release:
                        action.report["release_measurement"] = measured
                    else:
                        action.report["candidate_measurement"] = measured
                        action.report["candidate_probe"] = {
                            "sent_at": action.sent_at, "completed_at": stable_trace[-1]["observed_at_s"],
                            "outcome": observation["outcome"], "completion": observation["completion"],
                            "requested_width_m": target, "observed_width_m": measured["observed"]["width_m"],
                            "trace_sha256": digest,
                            "conservative_closure_displacement_m": observation["conservative_closure_displacement_m"],
                            "target_may_remain_active": True}
                # Finish host bookkeeping before acquiring completion feedback.
                # Never retimestamp the classified trace or relax receive limits.
                after = action.checked_current_completion()
                width = after[arm]["gripper"]["width_m"]
                if abs(width-observation["observed_width_m"]) > BOUNDS["jaw_span_m"]:
                    raise RuntimeError("Jaw changed while classifying the probe response")
                if observation["arrival_confirmed"] and abs(width-target) > LIMITS["gripper_m"]:
                    raise RuntimeError("Jaw left the observed arrival tolerance")
                if release:
                    opening = min(observation["actual_opening_increase_m"], width-max(baseline))
                    observation["actual_opening_increase_m"] = opening
                    observation["conservative_opening_displacement_m"] = opening
                    if opening <= BOUNDS["jaw_span_m"]:
                        raise RuntimeError("Current jaw feedback no longer distinguishes an opening response")
                action.idle_anchor[arm]["gripper"] = copy.deepcopy(after[arm]["gripper"])
                if candidate:
                    action.grasps[arm] = {"status": "contact_candidate", "arm": arm, "requested_width_m": target,
                        "observed_width_m": width, "sent_at": action.sent_at, "trace_sha256": digest,
                        "original_anchor": copy.deepcopy(original), "identity": None, "probe_event_id": None,
                        "candidate_probe": copy.deepcopy(action.report["candidate_probe"]),
                        "candidate_measurement": copy.deepcopy(measured),
                        "target_may_remain_active": True, "grasp_verified": False,
                        "physical_stop_verified": None}
                elif release:
                    if pending["identity"] is not None:
                        if pending["probe_event_id"] is None:
                            raise RuntimeError("Tracked opening is missing its original probe event")
                        pending.update(status="release_opened", release_opening={
                            "trace_sha256": digest, "finished_at": measured["ended_at"],
                            "observed_width_m": measured["observed"]["width_m"],
                            "target_width_m": target})
                    else:
                        # Legacy observation-only probes have no object episode.
                        # Mechanical opening clears only that local residual.
                        action.grasps[arm] = None
                action.active = False
                action.probe_mode = None
                action.anchor = copy.deepcopy(action.idle_anchor)
                if reacquisition_source is not None:
                    action.supported_reacquisition.update(closure_finished=True, jaw_anchor_m=width)
                # The final guard can also block on the persistent ledger.
                # Read again after it, including under the newly installed
                # stationary/grasp anchors; never reuse its pre-guard sample.
                after = action.checked_current_completion()
                if recovery_source is not None and pending.get("audited_opening_continuation") is not None:
                    # Anonymous opening has cleared its local residual above.
                    # The last fresh read must still use the original body's
                    # anchor, not merely this connection's newer idle anchor.
                    original = pending["original_anchor"]
                    deviation = anchor_deviation(original, after[arm])
                    if (not joints_within(action.feedback_policy, arm, original["joints_rad"], after[arm]["joints_rad"])
                            or deviation["position_m"] > BOUNDS["position_span_m"]
                            or deviation["rotation_rad"] > rotation_tolerance(action.feedback_policy, arm)):
                        raise RuntimeError("Final opening body changed from its original candidate anchor: " + arm)
                if abs(after[arm]["gripper"]["width_m"]-width) > BOUNDS["jaw_span_m"]:
                    raise RuntimeError("Jaw changed during final probe completion")
                if observation["arrival_confirmed"] and abs(after[arm]["gripper"]["width_m"]-target) > LIMITS["gripper_m"]:
                    raise RuntimeError("Jaw left the final arrival tolerance")
                self._baseline_duration = action.report["observed_stable_duration_s"]
                self._advances = action.report["observed_feedback_advances"]
                action.report.update(ok=True, status=observation["outcome"],
                    arrival_confirmed=observation["arrival_confirmed"], observation_status=status,
                    sample=self._sample(after))
                if reacquisition_source is not None:
                    action.supported_reacquisition["last_outcome"] = observation["outcome"]
                if recovery_source is not None:
                    self._supported_recovery_opening = {"arm": arm, "target_width_m": target,
                        "trace_sha256": digest, "finished_at": time.time(),
                        "anchor": measured_anchor(after[arm])}
                    if pending.get("audited_opening_continuation") is not None:
                        # Retain the failed probe's entire body reference even
                        # after opening; only its independently observed jaw
                        # width changes for the RX-only separation check.
                        anchor = copy.deepcopy(pending["original_anchor"])
                        anchor["width_m"] = after[arm]["gripper"]["width_m"]
                        self._supported_recovery_opening.update(anchor=anchor,
                            original_anchor=copy.deepcopy(pending["original_anchor"]),
                            audited_opening_continuation=copy.deepcopy(pending["audited_opening_continuation"]))
            except BaseException as exc:
                self._fault = str(exc)
                action.report.update(ok=False, status="pair_device_fault", arrival_confirmed=False, sample=None)
                action.report["errors"].append({"type": type(exc).__name__, "detail": str(exc)})
            finally:
                action.ticket = None
                action.probe_mode = action.probe_trace = None
            result = copy.deepcopy(action.report)
            result.update(arm=arm, kind="gripper", accepted=None, capabilities=self.capabilities,
                completion_mode="contact_probe_release" if release else "contact_probe",
                physical_stop_verified=None, physical_stop_supported=False, grasp_verified=False,
                object_release_verified=None,
                unresolved_gripper_probe=self.unresolved_gripper_probe,
                grasp_states=self.grasp_states,
                transmission_counts=copy.deepcopy(action.counts), session_transmission_counts=action.totals(),
                hardware_commands_sent=sum(v["sent_frames"] for v in action.counts.values()),
                passive_arm_commands_sent=action.counts[action.passive_arm]["sent_frames"],
                guard_violations=copy.deepcopy(action.violations), connected=not self._closed)
            if reacquisition_source is not None:
                result.update(audited_supported_reacquisition=copy.deepcopy(action.supported_reacquisition["audit"]),
                    reacquisition_original_anchor=copy.deepcopy(action.supported_reacquisition["body_anchors"][arm]),
                    supported_probe_index=action.supported_reacquisition["probe_count"], max_supported_probes=3,
                    empty_jaw_verified=None, object_release_verified=None)
            if release and "contact_observation" in result:
                result["actual_opening_increase_m"] = result["contact_observation"]["actual_opening_increase_m"]
            return result

    def close(self):
        with self._operation:
            action = self._action
            action.ticket = None
            self._closed = True
            cleanup = {}
            for side, robot in list(action.robots.items()):
                try:
                    robot.disconnect()
                    cleanup[side] = {"status": "disconnected", "physically_stopped": None}
                    del action.robots[side]
                except Exception as exc:
                    cleanup[side] = {"status": "cleanup_failed", "error": str(exc), "physically_stopped": None}
            return {"arms": cleanup, "physical_stop_verified": None, "physical_stop_supported": False,
                    "unresolved_gripper_probe": self.unresolved_gripper_probe,
                    "grasp_states": self.grasp_states,
                    "requires_fault_latch": any(action.grasps.values()),
                    "session_transmission_counts": action.totals(),
                    "guard_violations": copy.deepcopy(action.violations),
                    "detail": "Disconnect is resource cleanup, not a physical stop"}
