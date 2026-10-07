"""One supervised action with the other arm continuously observed and TX-blocked.

The caller owns the device lock, durable journal and physical review of the full
segment, attachments and residual motion. CAN frames are not atomic. A feedback
envelope detects departures after motion; it does not verify an IK or collision
path. No task algorithm, retry, stop, reset, disable or automatic next action is
provided, and the original full-backend hold gate is unchanged.
"""
import copy
import math
import threading
import time

from . import arms
from .home_arm import _HomeArm
from .linear_hold import _LinearHold, _pose_frames
from .single_gripper_prepare import _SingleGripperPrepare
from .supervised_actions import BOUNDS as DUAL_BOUNDS, _SupervisedAction
from .takeover import LIMITS, SIDES, _allowed_integer


# This independent contract does not mutate the older dual-arm action limits.
BOUNDS = {**DUAL_BOUNDS, "feedback_age_s": 0.1, "feedback_skew_s": 0.1,
          "rotation_span_rad": 0.003, "minimum_feedback_advances": 20}

# Observation policy, not controller limits, calibration accuracy or IK proof.
# Larger start offsets are confined to an action that cannot send arm targets.
FEEDBACK_BOUNDARY_POLICY = {
    "observation_band_rad": 0.003,
    "static_gripper_offset_cap_rad": 0.1,
    "static_exception_directions": {"joint2": "below_minimum", "joint3": "above_maximum"},
    "source": "User-requested bounded feedback admission; software policy, not physical qualification",
    "reference_scope": "Frozen first healthy observation of this single action; not a cross-session allowance",
}


class _SingleSupervisedAction(_SupervisedAction):
    SCOPE = "One selected-arm supervised target; other arm observed with all TX forbidden"

    def __init__(self, profile, journal, arm, kind, target):
        if not isinstance(arm, str) or arm not in SIDES or kind not in ("move", "gripper"):
            raise ValueError("Explicit left/right arm and known action kind required")
        super().__init__(profile, journal, arm, kind, target)
        self.passive_arm = "left" if arm == "right" else "right"
        self.enable_states, self.control_modes = {}, {}
        self.baseline_window, self.dispatch_anchor = None, None
        self.boundary_reference = None
        self.frame_limit = 4 if kind == "move" else 1
        for drift in self.max_drift.values():
            drift["rotation_distance_rad"] = 0.0
        self.report.update(
            operation="single_supervised_" + kind, selected_arm=arm,
            passive_arm=self.passive_arm, bounds=dict(BOUNDS),
            passive_arm_commands_sent=0, task_motion_ready=False,
            joint_limits_changed=False, full_backend_hold_gate_unchanged=True,
            observed_joint_limit_violations={}, selected_arm_strictly_within_limits=None,
            selected_arm_within_feedback_tolerance=None,
            static_boundary_exception_accepted=False,
            static_boundary_exception_ever_used=False,
            static_boundary_exception_joint_indices=[],
            feedback_boundary_policy=copy.deepcopy(FEEDBACK_BOUNDARY_POLICY),
            boundary_recovery_required=False, boundary_recovery=None,
            passive_arm_motion_qualified=False,
            rotation_distance_basis="SO(3), manufacturer Rz(yaw) Ry(pitch) Rx(roll)",
            mode_frame_can_activate_cached_target=kind == "move",
            partial_frames_can_mix_old_targets=kind == "move",
            failure_policy="No retry, stop, reset, disable, target replacement or passive-arm command",
            ok_semantics="One dispatch and bounded stable observation completed; arrival, target error and grasp are separate",
            controller_at_target_scope="Selected arm motion_status only; not Cartesian accuracy, jaw arrival, contact or grasp")

    def check_selected_joint_bounds(self, state, violations):
        """Keep nominal truth separate from a bounded observation exception.

        No samples or manufacturer limits are adjusted. A jaw action may retain
        an existing J2/J3 start offset, but cannot acquire a larger exception as
        later samples arrive. The ordinary stationary envelope still applies.
        """
        band = FEEDBACK_BOUNDARY_POLICY["observation_band_rad"]
        cap = FEEDBACK_BOUNDARY_POLICY["static_gripper_offset_cap_rad"]
        if self.boundary_reference is None:
            self.boundary_reference = list(state["joints_rad"])
            self.report["boundary_reference_joints_rad"] = self.boundary_reference[:]
        excess = [max(v["minimum_rad"] - v["observed_rad"],
                      v["observed_rad"] - v["maximum_rad"], 0.0) for v in violations]
        within_band = all(value <= band for value in excess)
        accepted, static_indices = [], []
        for v, amount in zip(violations, excess):
            index = v["joint_index"]
            direction_ok = ((index == 2 and v["observed_rad"] < v["minimum_rad"])
                            or (index == 3 and v["observed_rad"] > v["maximum_rad"]))
            origin = self.boundary_reference[index - 1]
            initial_excess = max(v["minimum_rad"] - origin, origin - v["maximum_rad"], 0.0)
            static_ok = (self.kind == "gripper" and direction_ok and amount <= cap
                         and initial_excess > 0 and amount <= initial_excess + band)
            accepted.append(amount <= band or static_ok)
            if amount > band and static_ok:
                static_indices.append(index)
        self.report.update(selected_arm_strictly_within_limits=not violations,
                           selected_arm_within_feedback_tolerance=within_band,
                           static_boundary_exception_accepted=bool(static_indices) and all(accepted))
        if all(accepted):
            if static_indices:
                self.report["static_boundary_exception_ever_used"] = True
                self.report["static_boundary_exception_joint_indices"] = sorted(set(
                    self.report["static_boundary_exception_joint_indices"] + static_indices))
            return
        # This is a diagnostic candidate, never an automatic recovery target.
        # In-flight uncertainty must not produce an executable-looking retry.
        if not self.dispatched:
            from .joint_recovery import RECOVERY
            q = state["joints_rad"]
            nearest = [min(high, max(low, value)) for i, value in enumerate(q, 1)
                       for low, high in [self.joint_limits[self.arm]["joint%d" % i]]]
            step_ok = all(abs(a - b) <= RECOVERY["joint_excursion_rad"] for a, b in zip(q, nearest))
            from .startup_recovery import STARTUP_RECOVERY
            startup_candidate = all(
                ((v["joint_index"] == 2 and v["observed_rad"] < v["minimum_rad"])
                 or (v["joint_index"] == 3 and v["observed_rad"] > v["maximum_rad"]))
                and amount <= STARTUP_RECOVERY["joint_excursion_rad"]
                for v, amount in zip(violations, excess))
            self.report.update(boundary_recovery_required=True, boundary_recovery={
                "nearest_legal_joints_rad": nearest,
                "candidate_only": True, "automatic_dispatch": False,
                "existing_recovery_step_limit_rad": RECOVERY["joint_excursion_rad"],
                "within_existing_recovery_step_limit": step_ok,
                "existing_recovery_ready": False,
                "startup_recovery_profile": "startup_j2_j3" if startup_candidate else None,
                "within_startup_recovery_angle_contract": startup_candidate,
                "startup_recovery_ready": False,
                "startup_recovery_requires": ["current empty/no-contact arms", "fresh stable enabled joints",
                    "caller attachment bound and whole-arm relative clearance", "encoded target and FK contract",
                    "operator attendance for non-atomic cached/partial target risk"],
                "remaining_checks": (["initial offset exceeds existing recovery step limit"] if not step_ok else [])
                                    + ["fresh feedback and encoded target", "whole-arm clearance", "FK displacement and remaining recovery contract"],
            })
        raise RuntimeError("Selected joint feedback exceeds action-specific boundary allowance: " + repr(violations))

    def connect(self):
        _LinearHold.connect(self)
        self.quaternion = _SingleGripperPrepare._manufacturer_quaternion
        for side, robot in self.robots.items():
            if side == self.passive_arm or self.kind == "gripper":
                robot._send_msg = lambda *a, _side=side, **k: self._deny(
                    _side, "Arm SDK TX forbidden for this single action")
                robot._send_msgs = robot._send_msg
            gripper = self.grippers[side]
            gripper._send_msg = lambda *a, _side=side, **k: self._deny(
                _side, "Gripper SDK TX forbidden for this single action")
            gripper._send_msgs = gripper._send_msg
        if self.kind == "gripper":
            # Only the selected effector's own encoder is restored. Both arm
            # encoders and the passive jaw remain blocked at SDK/comm/bus levels.
            gripper = self.grippers[self.arm]
            sender = getattr(type(gripper), "_send_msg", None)
            if not callable(sender):
                raise RuntimeError("Selected manufacturer effector encoder unavailable")
            gripper._send_msg = sender.__get__(gripper, type(gripper))

    def _wrap_bus(self, side, comm):
        _LinearHold._wrap_bus(self, side, comm)
        guarded_original = comm.send_bus.send
        def guarded_send(frame, *args, **kwargs):
            with self.lock:
                if self.counts[side]["attempted_frames"] >= self.frame_limit:
                    self._deny(side, "Single action lifetime CAN frame limit reached")
                return guarded_original(frame, *args, **kwargs)
        comm.send_bus.send = guarded_send

    def check_freshness(self, states):
        # Called after journaling, FK/window calculations and immediately before
        # dispatch/completion, so computation time is included in receive age.
        stamps = [stamp for side in SIDES for stamp in states[side]["fragment_timestamps_s"].values()]
        now = time.time()
        age, skew = now - min(stamps), max(stamps) - min(stamps)
        self.report.update(last_checked_feedback_age_s=age, last_checked_feedback_skew_s=skew)
        self.report["max_checked_feedback_age_s"] = max(self.report.get("max_checked_feedback_age_s", 0.0), age)
        if max(stamps) > now or age > BOUNDS["feedback_age_s"] or skew > BOUNDS["feedback_skew_s"]:
            raise RuntimeError("Single action requires receive age/skew within 100 ms including processing; age=%.6f skew=%.6f" % (age, skew))

    def checked(self, states):
        if self.violations:
            raise RuntimeError("CAN ownership/TX guard violation: " + repr(self.violations))
        for side in SIDES:
            state, selected = states[side], side == self.arm
            health = arms.control_health(state, allowed_control_modes=(1,) if selected else (0, 1, 2),
                                         require_enabled=False)
            if not health["healthy"]:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, health["reasons"]))
            status = state["arm_status"]
            flags = _SingleGripperPrepare.enable_flags(state)
            if any(type(flag) is not bool for flag in flags):
                raise RuntimeError(side + " requires seven known enable flags")
            if selected and not all(flags):
                raise RuntimeError("Selected arm requires six joint drivers and gripper continuously enabled")
            if side not in self.enable_states:
                self.enable_states[side] = flags[:]
                self.control_modes[side] = status["ctrl_mode"]
            if flags != self.enable_states[side] or status["ctrl_mode"] != self.control_modes[side]:
                raise RuntimeError(side + " control mode or joint/gripper enable state changed")
            moving = self.active and self.kind == "move" and selected
            if (not _allowed_integer(status.get("teach_status"), (0,))
                    or not _allowed_integer(status.get("motion_status"), (0, 1) if moving else (0,))
                    or not _allowed_integer(status.get("mode_feedback"), (0, 1, 2))):
                raise RuntimeError(side + " invalid teaching, arrival or movement-mode feedback")
            mode = status["mode_feedback"]
            if side in self.expected_modes:
                allowed = ((self.expected_modes[side], 2) if moving and not self.mode_l_confirmed
                           else (self.expected_modes[side],))
                if mode not in allowed:
                    raise RuntimeError(side + " unexpected movement mode change")
            if moving and self.sent_at is not None and state["fragment_timestamps_s"]["arm_status"] > self.sent_at:
                if mode != 2:
                    raise RuntimeError("New feedback did not confirm MOVE_L mode")
                self.mode_l_confirmed, self.expected_modes[side] = True, 2
            violations = _HomeArm.outside(self, side, state["joints_rad"])
            self.report["observed_joint_limit_violations"][side] = violations
            if selected:
                self.check_selected_joint_bounds(state, violations)
            _pose_frames(state["pose_m_rad"])
            if not 0 <= state["gripper"]["width_m"] <= 0.070:
                raise RuntimeError(side + " jaw width outside 0..70 mm feedback range")
            if self.anchor is not None:
                self.check_envelope(side, state, moving)
        self.check_freshness(states)
        return states

    def check_envelope(self, side, state, moving):
        # Preserve the original baseline for every stationary component. Only
        # the selected moving arm uses the last checked dispatch pose as start.
        origin = self.dispatch_anchor if moving else self.anchor[side]
        pose, start = state["pose_m_rad"], origin["pose_m_rad"]
        qdelta = max(abs(a - b) for a, b in zip(state["joints_rad"], origin["joints_rad"]))
        distance, rotation = math.dist(pose[:3], start[:3]), self.rotation_distance(pose, start)
        width, baseline_width = state["gripper"]["width_m"], self.anchor[side]["gripper"]["width_m"]
        for name, value in (("joint_rad", qdelta), ("position_m", distance),
                            ("rotation_distance_rad", rotation), ("gripper_m", abs(width - baseline_width))):
            self.max_drift[side][name] = max(self.max_drift[side][name], value)
        if moving:
            vector = [b - a for a, b in zip(start[:3], self.target[:3])]
            length2 = sum(v * v for v in vector)
            fraction = max(0, min(1, sum((pose[i] - start[i]) * vector[i] for i in range(3)) / length2)) if length2 else 0
            nearest = [start[i] + fraction * vector[i] for i in range(3)]
            if (qdelta > BOUNDS["joint_excursion_rad"]
                    or math.dist(pose[:3], nearest) > BOUNDS["segment_margin_m"]
                    or rotation > self.rotation_distance(self.target, start) + BOUNDS["rotation_margin_rad"]):
                raise RuntimeError("Selected arm left observed segment/joint envelope; no stop command available")
        elif (qdelta > BOUNDS["joint_span_rad"] or distance > BOUNDS["position_span_m"]
              or rotation > BOUNDS["rotation_span_rad"]):
            raise RuntimeError(side + " stationary arm exceeded joint/XYZ/SO(3) envelope")
        if self.active and self.kind == "gripper" and side == self.arm:
            low, high = sorted((self.dispatch_anchor["gripper"]["width_m"], self.target))
            if not low - LIMITS["gripper_m"] <= width <= high + LIMITS["gripper_m"]:
                raise RuntimeError("Selected jaw left requested width interval")
        elif abs(width - baseline_width) > LIMITS["gripper_m"]:
            raise RuntimeError(side + " uncommanded jaw drift exceeds 2 mm")

    # Pure observed-window helpers: six joint spans, XYZ bounding diagonal,
    # jaw span and exact pairwise SO(3) diameter; no home dispatch is inherited.
    new_window = _HomeArm.new_window
    extend_window = _HomeArm.extend_window
    window_spans = staticmethod(_HomeArm.window_spans)

    @classmethod
    def window_stable(cls, window):
        return all(s["joint_rad"] <= BOUNDS["joint_span_rad"]
                   and s["position_m"] <= BOUNDS["position_span_m"]
                   and s["jaw_m"] <= BOUNDS["jaw_span_m"]
                   and s["rotation_rad"] <= BOUNDS["rotation_span_rad"]
                   for s in cls.window_spans(window).values())

    def prepare(self):
        deadline = time.monotonic() + LIMITS["warmup_s"]
        while True:
            state = self.read()
            if all(state[side].get("status") == "complete" for side in SIDES):
                self.checked(state)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("No complete fresh feedback from selected and passive arms")
            time.sleep(BOUNDS["poll_s"])
        self.anchor = copy.deepcopy(state)
        self.expected_modes = {s: state[s]["arm_status"]["mode_feedback"] for s in SIDES}
        self.report["before"] = copy.deepcopy(state)
        self.baseline_window = self.new_window(state)
        start, previous, advances = time.monotonic(), state, 0
        while time.monotonic() - start < BOUNDS["stable_s"]:
            time.sleep(BOUNDS["poll_s"])
            state = self.checked(self.read())
            if not self.extend_window(self.baseline_window, state):
                raise RuntimeError("Single-action baseline exceeded stable feedback spans")
            self.check_freshness(state)
            if self.advanced(previous, state):
                previous, advances = state, advances + 1
        if advances < BOUNDS["minimum_feedback_advances"]:
            raise RuntimeError("Baseline lacks 20 independently advancing complete samples in three seconds")
        self.report.update(baseline_duration_s=time.monotonic() - start,
                           baseline_spans=self.window_spans(self.baseline_window),
                           baseline_feedback_advances=advances)
        self.record_outcome(state, False)
        self.check_freshness(state)

    def dispatch(self):
        if self.dispatched:
            raise RuntimeError("Only one target call permitted; no retry")
        robot = self.robots[self.arm]
        if self.kind == "move":
            for key, expected in (("ctrl_mode", 1), ("mit_mode", 0), ("residence_time", 0), ("installation_pos", 0)):
                if not _allowed_integer(getattr(robot._msg_mode, key, None), (expected,)):
                    raise RuntimeError("Unexpected SDK mode cache " + key)
            robot.set_auto_set_motion_mode_enabled(True)  # In-memory setting only.
            robot._msg_mode.move_spd_rate_ctrl = 1
            frames = [(0x151, bytes((1, 2, 1, 0, 0, 0, 0, 0)))] + _pose_frames(self.target)
            sdk_call = lambda: robot.move_l(self.target[:])
        else:
            frames = [(0x159, round(self.target * 1e6).to_bytes(4, "big", signed=True) + bytes((0, 200, 1, 0)))]
            sdk_call = lambda: self.grippers[self.arm].move_gripper_m(value=self.target, force=0.2)
        self.emit("single_supervised_action_intent", {"arm": self.arm, "kind": self.kind,
                  "target": self.target, "frames": [{"id": i, "data_hex": data.hex()} for i, data in frames]})
        state = self.checked(self.read())
        if self.kind == "move":
            self.validate_move(state)  # Existing target bounds and manufacturer FK agreement.
        if not self.extend_window(self.baseline_window, state):
            raise RuntimeError("Starting feedback changed after durable intent")
        self.dispatch_anchor = copy.deepcopy(state[self.arm])
        self.report["dispatch_feedback"] = copy.deepcopy(state)
        ticket = {"side": self.arm, "thread": threading.get_ident(), "kind": "action", "frames": frames,
                  "comm_calls": 0, "bus_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN violation before single-action dispatch")
            self.check_freshness(state)
            self.active, self.dispatched, self.ticket = True, True, ticket
            try:
                sdk_call()
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["comm_calls"] != len(frames) or ticket["bus_calls"] != len(frames):
                    raise RuntimeError("Incomplete SDK target sequence; no retry or completion send")
            finally:
                self.ticket = None
        self.sent_at = time.time()
        self.report["target_calls_sent"] = 1
        self.emit("single_supervised_action_sent_unconfirmed", {"finished_unix_s": self.sent_at, "kind": self.kind})

    def record_outcome(self, state, stable):
        super().record_outcome(state, stable)
        self.report.update(raw_mode_feedback={s: state[s]["arm_status"]["mode_feedback"] for s in SIDES},
                           move_l_mode_confirmed=self.mode_l_confirmed,
                           feedback_all_after_send=self.sent_at is not None and self.all_after(state, self.sent_at))

    def perform(self):
        self.dispatch()
        deadline = time.monotonic() + BOUNDS["timeout_s"]
        began, previous, advances, window = None, None, 0, None
        while time.monotonic() < deadline:
            state = self.checked(self.read())
            self.record_outcome(state, False)
            if self.all_after(state, self.sent_at):
                if began is None:
                    began, previous, advances = time.monotonic(), state, 0
                    window = self.new_window(state)
                elif not self.extend_window(window, state):
                    began, previous, advances = time.monotonic(), state, 0
                    window = self.new_window(state)
                elif self.advanced(previous, state):
                    previous, advances = state, advances + 1
                    if time.monotonic() - began >= BOUNDS["stable_s"] and advances >= BOUNDS["minimum_feedback_advances"]:
                        # Deliberately retain bounded-observation semantics:
                        # stable may coexist with motion_status=1 or jaw error.
                        self.record_outcome(state, True)
                        self.report.update(observed_stable_duration_s=time.monotonic() - began,
                                           observed_spans=self.window_spans(window),
                                           observed_feedback_advances=advances)
                        self.check_freshness(state)
                        return "observed_stable_feedback_not_target_grasp_or_stop_certification"
            self.check_freshness(state)
            time.sleep(BOUNDS["poll_s"])
        raise RuntimeError("Observation timeout; no further target or stop sent; physical state requires supervision")

    def run(self):
        report = super().run()
        count = self.counts[self.arm]["sent_frames"]
        report.update(mode_commands_sent=int(self.kind == "move" and count > 0),
                      arm_target_commands_sent=max(0, count - 1) if self.kind == "move" else 0,
                      gripper_target_commands_sent=count if self.kind == "gripper" else 0,
                      passive_arm_commands_sent=self.counts[self.passive_arm]["sent_frames"],
                      original_enable_flags=copy.deepcopy(self.enable_states),
                      original_control_modes=copy.deepcopy(self.control_modes))
        return report


def move_once(profile, journal, arm, target_pose_m_rad):
    """One selected-base/flange MOVE_L, <=30 mm and <=0.05 rad, fixed 1%."""
    if not isinstance(arm, str) or arm not in SIDES or not callable(journal):
        raise ValueError("Explicit left/right arm and synchronous journal required")
    target = arms._six_finite(target_pose_m_rad)
    _pose_frames(target)
    return _SingleSupervisedAction(profile, journal, arm, "move", target).run()


def gripper_once(profile, journal, arm, width_m, nominal_force_N):
    """One enabled-jaw target in 0..55 mm at nominal 0.2; no grasp inference."""
    if not isinstance(arm, str) or arm not in SIDES or not callable(journal):
        raise ValueError("Explicit left/right arm and synchronous journal required")
    if (type(width_m) not in (int, float) or not math.isfinite(width_m) or not 0 <= width_m <= 0.055
            or type(nominal_force_N) not in (int, float) or nominal_force_N != 0.2):
        raise ValueError("Width must be 0..0.055 m and nominal SDK force exactly 0.2")
    return _SingleSupervisedAction(profile, journal, arm, "gripper", float(width_m)).run()
