"""One attended near-home MOVE_J to existing six-axis zero, never zero calibration.

No automatic collision verification or stopping capability is provided. The
caller reviews the full arm/tool/cable path and owns the device lock and durable
journal. One four-frame SDK call is non-atomic: cached or mixed targets remain
a physical risk. Failure never retries, stops, resets, disables or changes jaws.
"""
import copy
import math
import threading
import time

from . import arms
from .linear_hold import _LinearHold
from .single_gripper_prepare import _SingleGripperPrepare
from .takeover import LIMITS, SIDES, _Takeover, _allowed_integer


START_ABS_RAD = (math.pi / 4, math.pi / 18, math.pi / 18,
                 math.pi / 12, math.pi / 6, math.pi / 12)
HOME = {"initial_boundary_exception_rad": math.pi / 36,
        "joint_span_rad": 0.003, "position_span_m": 0.0005,
        "jaw_span_m": 0.0005, "rotation_span_rad": 0.003,
        "tracking_margin_rad": 0.01, "boundary_deepening_rad": 0.003,
        "fk_position_tolerance_m": 0.002, "fk_rotation_tolerance_rad": 0.02,
        "zero_joint_tolerance_rad": 0.003, "zero_rotation_tolerance_rad": 0.01,
        "stable_s": 3.0, "minimum_feedback_advances": 20,
        "feedback_age_s": 0.1, "feedback_skew_s": 0.1,
        "timeout_s": 120.0, "poll_s": 0.01, "speed_percent": 1}
ZERO = (0.0,) * 6
FRAMES = ((0x151, bytes((1, 1, 1, 0, 0, 0, 0, 0))),
          (0x155, bytes(8)), (0x156, bytes(8)), (0x157, bytes(8)))


class _HomeArm(_LinearHold):
    FRAME_KINDS = ("home",)
    SCOPE = "One operator-reviewed near-home move to existing joint zeros"

    def __init__(self, profile, journal, arm):
        if not isinstance(arm, str) or arm not in SIDES:
            raise ValueError("arm must be explicitly left or right")
        _Takeover.__init__(self, profile, journal)
        self.arm = arm
        self.passive_arm = "left" if arm == "right" else "right"
        self.dispatched, self.sent_at = False, None
        self.dispatch_anchor = None
        self.enable_states, self.control_modes = {}, {}
        self.mode_j_confirmed = False
        self.quaternion, self.joint_limits, self.zero_pose = None, {}, None
        self.baseline_window = None
        for drift in self.max_drift.values():
            drift["rotation_distance_rad"] = 0.0
        self.report.update(
            operation="home_arm", arm=arm, passive_arm=self.passive_arm,
            target_joints_rad=list(ZERO), home_limits=dict(HOME),
            start_abs_limit_rad=list(START_ABS_RAD), target_calls_sent=0,
            zero_target_observed=False, selected_arm_strictly_within_limits=None,
            task_motion_ready=False, general_stop_validated=False,
            path_collision_verified=False, operator_full_path_review_required=True,
            tracking_box_is_path_guarantee=False, joint_limits_changed=False,
            joint_zero_calibrated=False, automatic_retry=False,
            gripper_target_commands_sent=0, passive_arm_commands_sent=0,
            partial_frames_can_mix_old_targets=True,
            failure_policy="No retry, stop, reset, disable, jaw or other-arm command",
            completion_scope="Observed existing joint zeros; not task motion or stopping qualification")

    def connect(self):
        _LinearHold.connect(self)
        self.quaternion = _SingleGripperPrepare._manufacturer_quaternion
        passive = self.robots[self.passive_arm]
        passive._send_msg = lambda *a, **k: self._deny(self.passive_arm, "Passive arm SDK TX forbidden during home")
        passive._send_msgs = passive._send_msg
        for side, gripper in self.grippers.items():
            gripper._send_msg = lambda *a, _side=side, **k: self._deny(_side, "Gripper SDK TX forbidden during home")
        if self.outside(self.arm, ZERO):
            raise RuntimeError("Existing six-zero target is outside manufacturer joint limits")
        self.zero_pose = arms._six_finite(self.robots[self.arm].fk(list(ZERO)))
        self.report["zero_flange_m_rad"] = self.zero_pose[:]

    def _wrap_bus(self, side, comm):
        # Reuse the existing ordered exact-byte guard, with a stricter lifetime
        # ceiling of four instead of the linear probe's seven-frame ceiling.
        _LinearHold._wrap_bus(self, side, comm)
        original = comm.send_bus.send
        def guarded_send(frame, *args, **kwargs):
            with self.lock:
                if self.counts[side]["attempted_frames"] >= 4:
                    self._deny(side, "Home permits at most four attempted CAN frames")
                return original(frame, *args, **kwargs)
        comm.send_bus.send = guarded_send

    def outside(self, side, joints):
        return [{"joint_index": i + 1, "observed_rad": value,
                 "minimum_rad": low, "maximum_rad": high}
                for i, value in enumerate(joints)
                for low, high in [self.joint_limits[side]["joint%d" % (i + 1)]]
                if not low <= value <= high]

    def validate_start(self, state):
        q = state["joints_rad"]
        for i, value in enumerate(q):
            if abs(value) > START_ABS_RAD[i]:
                raise RuntimeError("J%d outside fixed near-home starting range" % (i + 1))
            low, high = self.joint_limits[self.arm]["joint%d" % (i + 1)]
            if low <= value <= high:
                continue
            exception = ((i == 1 and low - HOME["initial_boundary_exception_rad"] <= value < low)
                         or (i == 2 and high < value <= high + HOME["initial_boundary_exception_rad"]))
            if not exception:
                raise RuntimeError("J%d exceeds allowed initial manufacturer-boundary exception" % (i + 1))

    def checked(self, states):
        if self.violations:
            raise RuntimeError("CAN ownership/TX violation: " + repr(self.violations))
        stamps = []
        for side in SIDES:
            state, selected = states[side], side == self.arm
            status = state["arm_status"]
            health = arms.control_health(state, allowed_control_modes=(1,) if selected else (0, 1, 2),
                                         require_enabled=False)
            if not health["healthy"]:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, health["reasons"]))
            flags = _SingleGripperPrepare.enable_flags(state)
            if any(type(flag) is not bool for flag in flags):
                raise RuntimeError(side + " enable flags must be known booleans")
            if selected and not all(flags[:6]):
                raise RuntimeError("Selected six joint drivers must remain enabled")
            if side not in self.enable_states:
                self.enable_states[side] = list(flags)
                self.control_modes[side] = status["ctrl_mode"]
            if flags != self.enable_states[side] or status["ctrl_mode"] != self.control_modes[side]:
                raise RuntimeError(side + " mode or joint/gripper enable state changed")
            moving = selected and self.dispatched
            if (not _allowed_integer(status.get("teach_status"), (0,))
                    or not _allowed_integer(status.get("motion_status"), (0, 1) if moving else (0,))
                    or not _allowed_integer(status.get("mode_feedback"), (0, 1, 2))):
                raise RuntimeError(side + " invalid teaching or movement mode/status")
            mode = status["mode_feedback"]
            if side in self.expected_modes:
                allowed = ((1,) if self.mode_j_confirmed else (self.expected_modes[side], 1)) if moving else (self.expected_modes[side],)
                if mode not in allowed:
                    raise RuntimeError(side + " movement mode changed outside home request")
            if (moving and self.sent_at is not None and mode == 1
                    and state["fragment_timestamps_s"]["arm_status"] > self.sent_at):
                self.mode_j_confirmed = True
            if not 0 <= state["gripper"]["width_m"] <= 0.070:
                raise RuntimeError(side + " jaw feedback outside known 0..70 mm range")
            stamps.extend(state["fragment_timestamps_s"].values())
            if selected:
                if not self.dispatched:
                    self.validate_start(state)
                fk = arms._six_finite(self.robots[side].fk(state["joints_rad"][:]))
                error = {"position_m": math.dist(fk[:3], state["pose_m_rad"][:3]),
                         "rotation_rad": self.rotation_distance(fk, state["pose_m_rad"])}
                self.report["current_fk_feedback_error"] = error
                if error["position_m"] > HOME["fk_position_tolerance_m"] or error["rotation_rad"] > HOME["fk_rotation_tolerance_rad"]:
                    raise RuntimeError("Selected manufacturer FK disagrees with flange feedback")
            if self.anchor is not None:
                self.check_envelope(side, state, moving)
        if max(stamps) - min(stamps) > HOME["feedback_skew_s"]:
            raise RuntimeError("Cross-arm feedback skew exceeds 100 ms")
        age = time.time() - min(stamps)
        self.report["last_checked_feedback_age_s"] = age
        self.report["max_checked_feedback_age_s"] = max(
            self.report.get("max_checked_feedback_age_s", 0.0), age)
        if age > HOME["feedback_age_s"]:
            raise RuntimeError("Home requires all feedback fragments within 100 ms; observed %.6f s" % age)
        return states

    def check_envelope(self, side, state, moving):
        origin = self.dispatch_anchor if moving else self.anchor[side]
        q, q0 = state["joints_rad"], origin["joints_rad"]
        pose, pose0 = state["pose_m_rad"], origin["pose_m_rad"]
        differences = {"joint_rad": max(abs(a - b) for a, b in zip(q, q0)),
                       "position_m": math.dist(pose[:3], pose0[:3]),
                       "gripper_m": abs(state["gripper"]["width_m"] - self.anchor[side]["gripper"]["width_m"]),
                       "rotation_distance_rad": self.rotation_distance(pose, pose0)}
        for name, value in differences.items():
            self.max_drift[side][name] = max(self.max_drift[side][name], value)
        if differences["gripper_m"] > HOME["jaw_span_m"]:
            raise RuntimeError(side + " jaw changed more than 0.5 mm during home")
        if moving:
            for i, (value, start) in enumerate(zip(q, q0)):
                low, high = min(start, 0.0) - HOME["tracking_margin_rad"], max(start, 0.0) + HOME["tracking_margin_rad"]
                if not low <= value <= high:
                    raise RuntimeError("Selected J%d left independent start-to-zero tracking box" % (i + 1))
                nominal_low, nominal_high = self.joint_limits[side]["joint%d" % (i + 1)]
                if ((start < nominal_low and value < start - HOME["boundary_deepening_rad"])
                        or (start > nominal_high and value > start + HOME["boundary_deepening_rad"])):
                    raise RuntimeError("Selected J%d deepened its initial boundary violation" % (i + 1))
        elif (differences["joint_rad"] > HOME["joint_span_rad"]
              or differences["position_m"] > (HOME["position_span_m"] if side == self.arm else LIMITS["position_m"])
              or differences["rotation_distance_rad"] > HOME["rotation_span_rad"]):
            raise RuntimeError(side + " stationary arm drift exceeds home envelope")

    def new_window(self, states):
        return {side: {"qlow": s["joints_rad"][:], "qhigh": s["joints_rad"][:],
                       "low": s["pose_m_rad"][:3], "high": s["pose_m_rad"][:3],
                       "jaw_low": s["gripper"]["width_m"], "jaw_high": s["gripper"]["width_m"],
                       "poses": [s["pose_m_rad"][:]], "rotation_span_rad": 0.0}
                for side, s in states.items()}

    def extend_window(self, window, states):
        for side in SIDES:
            box, state = window[side], states[side]
            for name, values, fn in (("qlow", state["joints_rad"], min), ("qhigh", state["joints_rad"], max),
                                     ("low", state["pose_m_rad"][:3], min), ("high", state["pose_m_rad"][:3], max)):
                box[name] = [fn(a, b) for a, b in zip(box[name], values)]
            width, pose = state["gripper"]["width_m"], state["pose_m_rad"]
            box["jaw_low"], box["jaw_high"] = min(box["jaw_low"], width), max(box["jaw_high"], width)
            # Compare distinct orientations, keeping the exact pairwise SO(3)
            # span without accumulating identical static samples.
            if pose[3:] not in [old[3:] for old in box["poses"]]:
                box["rotation_span_rad"] = max(box["rotation_span_rad"],
                    max(self.rotation_distance(pose, previous) for previous in box["poses"]))
                box["poses"].append(pose[:])
        return self.window_stable(window)

    @staticmethod
    def window_spans(window):
        return {side: {"joint_rad": max(b - a for a, b in zip(box["qlow"], box["qhigh"])),
                       "position_m": math.dist(box["low"], box["high"]),
                       "jaw_m": box["jaw_high"] - box["jaw_low"],
                       "rotation_rad": box["rotation_span_rad"]} for side, box in window.items()}

    @classmethod
    def window_stable(cls, window):
        return all(s["joint_rad"] <= HOME["joint_span_rad"] and s["position_m"] <= HOME["position_span_m"]
                   and s["jaw_m"] <= HOME["jaw_span_m"] and s["rotation_rad"] <= HOME["rotation_span_rad"]
                   for s in cls.window_spans(window).values())

    def prepare(self):
        deadline = time.monotonic() + LIMITS["warmup_s"]
        while True:
            states = self.read()
            if all(states[side].get("status") == "complete" for side in SIDES):
                states = self.checked(states)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("No complete fresh feedback for home baseline")
            time.sleep(HOME["poll_s"])
        self.anchor = copy.deepcopy(states)
        self.expected_modes = {side: states[side]["arm_status"]["mode_feedback"] for side in SIDES}
        self.report["before"] = copy.deepcopy(states)
        self.report["initial_boundary_violations"] = {s: self.outside(s, states[s]["joints_rad"]) for s in SIDES}
        self.baseline_window = self.new_window(states)
        start, previous, advances = time.monotonic(), states, 0
        while time.monotonic() - start < HOME["stable_s"]:
            time.sleep(HOME["poll_s"])
            states = self.checked(self.read())
            if not self.extend_window(self.baseline_window, states):
                raise RuntimeError("Home baseline lacks three seconds of stable complete feedback")
            if self.advanced(previous, states):
                previous, advances = states, advances + 1
        if advances < HOME["minimum_feedback_advances"]:
            raise RuntimeError("Home baseline lacks 20 independently advancing complete samples")
        self.report.update(baseline_duration_s=time.monotonic() - start,
                           baseline_feedback_advances=advances, baseline_spans=self.window_spans(self.baseline_window))

    def at_zero(self, states, require_postsend):
        state = states[self.arm]
        errors = {"joint_rad": [abs(q) for q in state["joints_rad"]],
                  "position_m": math.dist(state["pose_m_rad"][:3], self.zero_pose[:3]),
                  "rotation_rad": self.rotation_distance(state["pose_m_rad"], self.zero_pose)}
        self.report.update(zero_target_errors=errors, raw_motion_status=state["arm_status"]["motion_status"],
                           raw_mode_feedback=state["arm_status"]["mode_feedback"],
                           move_j_mode_confirmed=self.mode_j_confirmed)
        after = not require_postsend or (self.mode_j_confirmed and all(
            stamp > self.sent_at for side in SIDES for stamp in states[side]["fragment_timestamps_s"].values()))
        return (after and state["arm_status"]["motion_status"] == 0
                and max(errors["joint_rad"]) <= HOME["zero_joint_tolerance_rad"]
                and errors["position_m"] <= HOME["fk_position_tolerance_m"]
                and errors["rotation_rad"] <= HOME["zero_rotation_tolerance_rad"])

    def dispatch(self):
        if self.dispatched:
            raise RuntimeError("Home target call already attempted; no retry")
        robot = self.robots[self.arm]
        for key, value in (("ctrl_mode", 1), ("mit_mode", 0), ("installation_pos", 0), ("residence_time", 0)):
            if not _allowed_integer(getattr(robot._msg_mode, key, None), (value,)):
                raise RuntimeError("Unexpected SDK mode cache " + key)
        robot.set_auto_set_motion_mode_enabled(True)
        robot._msg_mode.move_spd_rate_ctrl = 1  # In-memory; never call a separate speed setter.
        self.emit("home_intent", {"arm": self.arm, "target_joints_rad": list(ZERO),
                  "frames": [{"id": can_id, "data_hex": data.hex()} for can_id, data in FRAMES],
                  "path_collision_verified": False, "operator_full_path_review_required": True})
        states = self.checked(self.read())
        if not self.extend_window(self.baseline_window, states):
            raise RuntimeError("Home start changed after durable intent")
        self.dispatch_anchor = copy.deepcopy(states[self.arm])
        self.report["dispatch_feedback"] = copy.deepcopy(states)
        self.report["tracking_box_rad"] = [[min(q, 0) - HOME["tracking_margin_rad"], max(q, 0) + HOME["tracking_margin_rad"]]
                                            for q in self.dispatch_anchor["joints_rad"]]
        self.report["tracking_reference_joints_rad"] = self.dispatch_anchor["joints_rad"][:]
        ticket = {"side": self.arm, "thread": threading.get_ident(), "kind": "home",
                  "frames": list(FRAMES), "comm_calls": 0, "bus_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN guard violation before home dispatch")
            self.dispatched, self.ticket = True, ticket
            try:
                robot.move_j(list(ZERO))
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["comm_calls"] != 4 or ticket["bus_calls"] != 4:
                    raise RuntimeError("Incomplete four-frame SDK home sequence; no retry")
            finally:
                self.ticket = None
        self.sent_at = time.time()
        self.emit("home_dispatched_unconfirmed", {"finished_unix_s": self.sent_at})

    def observe_zero(self, require_postsend):
        deadline = time.monotonic() + HOME["timeout_s"]
        start, previous, advances, window = None, None, 0, None
        while time.monotonic() < deadline:
            states = self.checked(self.read())
            if not self.at_zero(states, require_postsend):
                if not require_postsend:
                    raise RuntimeError("Read-only zero confirmation left zero tolerance; no target sent")
                start, window = None, None
            elif start is None:
                start, previous, advances = time.monotonic(), states, 0
                window = self.new_window(states)
            elif not self.extend_window(window, states):
                start, window = None, None
            elif self.advanced(previous, states):
                previous, advances = states, advances + 1
                if time.monotonic() - start >= HOME["stable_s"] and advances >= HOME["minimum_feedback_advances"]:
                    self.report.update(zero_target_observed=True, stable_duration_s=time.monotonic() - start,
                                       stable_feedback_advances=advances, final_spans=self.window_spans(window),
                                       final_boundary_violations={s: self.outside(s, states[s]["joints_rad"]) for s in SIDES},
                                       selected_arm_strictly_within_limits=not self.outside(self.arm, states[self.arm]["joints_rad"]))
                    self.emit("home_zero_observed", {"arm": self.arm, "command_sent": self.dispatched,
                              "target_errors": self.report["zero_target_errors"],
                              "selected_arm_strictly_within_limits": self.report["selected_arm_strictly_within_limits"]})
                    return "existing_joint_zero_observed_not_task_ready" if require_postsend else "already_near_zero_observed_no_command"
            time.sleep(HOME["poll_s"])
        raise RuntimeError("Home observation timed out after 120 s; no retry or stop sent")

    def perform(self):
        states = self.checked(self.read())
        if not self.extend_window(self.baseline_window, states):
            raise RuntimeError("Home start changed after baseline")
        if self.at_zero(states, False):
            self.report["already_near_zero"] = True
            self.emit("home_already_near_zero_read_only", {"arm": self.arm, "mode_feedback": states[self.arm]["arm_status"]["mode_feedback"]})
            return self.observe_zero(False)
        self.report["already_near_zero"] = False
        self.dispatch()
        return self.observe_zero(True)

    def run(self):
        report = _Takeover.run(self)
        count = self.counts[self.arm]["sent_frames"]
        report.update(home_attempted=self.dispatched, mode_commands_sent=int(count >= 1),
                      target_commands_sent=max(0, count - 1), target_calls_sent=int(count == 4),
                      passive_arm_commands_sent=self.counts[self.passive_arm]["sent_frames"],
                      original_enable_flags=copy.deepcopy(self.enable_states))
        if not report["ok"]:
            report["zero_target_observed"] = False
        return report


def home_arm(profile, journal, arm):
    """Move one explicitly selected near-home arm to its six existing zeros once.

    The caller must review the complete physical path and maintain attendance.
    No target, speed, clearance claim, calibration, or retry parameter is accepted.
    """
    if not callable(journal):
        raise TypeError("A synchronous journal(event, data) is required")
    return _HomeArm(profile, journal, arm).run()
