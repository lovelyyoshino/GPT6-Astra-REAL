"""One caller-specified boundary recovery, never a zero calibration or stop."""
import copy
import math
import struct
import threading
import time

from . import arms
from .linear_hold import _LinearHold
from .takeover import LIMITS, SIDES, _Takeover, _allowed_integer

RECOVERY = {"joint_excursion_rad": 0.05, "joint_tolerance_rad": 0.003,
            "right_j4_observation_tolerance_rad": 0.008, "fk_position_tolerance_m": 0.002,
            "fk_orientation_tolerance_rad": 0.02, "target_displacement_m": 0.015,
            "active_displacement_m": 0.02, "stable_s": 3.0, "timeout_s": 10.0,
            "poll_s": 0.01, "feedback_age_s": 0.05, "speed_percent": 1}


class _JointRecovery(_LinearHold):
    CONTROL_MODES = (1,)
    REQUIRE_ENABLED = True
    FRAME_KINDS = ("recovery",)
    SCOPE = "One low-speed caller-specified joint boundary recovery; no follow-up commands"

    def __init__(self, profile, journal_callback, arm, target):
        _Takeover.__init__(self, profile, journal_callback)
        self.arm, self.target = arm, list(target)
        self.phase, self.dispatched, self.sent_at = "baseline", False, None
        self.quaternion, self.joint_limits, self.target_pose = None, {}, None
        self.mode_confirmed = False
        self.report.update(operation="recover_joint_boundary", arm=arm, recovery_observed=False,
                           target_joints_rad=self.target[:], recovery_limits=dict(RECOVERY),
                           target_calls_sent=0, mode_commands_sent=0,
                           general_stop_validated=False, limits_changed=False,
                           joint_zero_calibrated=False, hard_path_guarantee=False,
                           caller_clearance_and_attendance_required=True,
                           mode_frame_can_activate_cached_target=True,
                           partial_frames_can_mix_old_targets=True,
                           failure_policy="No retry, stop, reset or disable; operator attendance required",
                           completion_scope="Target feedback observed for 3 seconds; not a stop qualification")

    def tolerance(self, side, index):
        return RECOVERY["right_j4_observation_tolerance_rad"] if side == "right" and index == 3 else 0.003

    def allowed_motion_status(self, side, active):
        return (0, 1) if active else (0,)

    def stable_position_span(self):
        return 0.002

    def health(self, side, state):
        return arms.control_health(state, allowed_control_modes=(1,), require_enabled=True)

    def arrival_matches(self, state):
        return True

    def outside(self, side, joints):
        return [{"joint_index": i + 1, "observed_rad": value, "minimum_rad": low, "maximum_rad": high}
                for i, value in enumerate(joints)
                for low, high in [self.joint_limits[side]["joint%d" % (i + 1)]]
                if not low <= value <= high]

    def rotation_distance(self, a, b):
        qa, qb = self.quaternion(*a[3:]), self.quaternion(*b[3:])
        norm = math.sqrt(sum(x*x for x in qa) * sum(x*x for x in qb))
        return 2 * math.acos(min(1.0, abs(sum(x*y for x, y in zip(qa, qb))) / norm))

    def checked(self, states):
        if self.violations:
            raise RuntimeError("CAN ownership/TX violation: " + repr(self.violations))
        stamps = []
        for side in SIDES:
            state = states[side]
            health = self.health(side, state)
            if not health["healthy"]:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, health["reasons"]))
            active = self.phase == "active" and side == self.arm
            status = state["arm_status"]
            if (not _allowed_integer(status.get("teach_status"), (0,))
                    or not _allowed_integer(status.get("motion_status"), self.allowed_motion_status(side, active))):
                raise RuntimeError(side + " unexpected teaching/motion status")
            mode = status.get("mode_feedback")
            if not _allowed_integer(mode, (0, 1, 2)):
                raise RuntimeError(side + " unknown movement mode")
            if side in self.expected_modes:
                allowed = (1,) if active and self.mode_confirmed else ((self.expected_modes[side], 1) if active else (self.expected_modes[side],))
                if mode not in allowed:
                    raise RuntimeError(side + " movement mode changed unexpectedly")
            if (active and self.sent_at is not None and mode == 1
                    and state["fragment_timestamps_s"]["arm_status"] > self.sent_at):
                self.mode_confirmed = True
            stamps.extend(state["fragment_timestamps_s"].values())
            if self.anchor is not None:
                origin = self.anchor[side]
                joints, baseline = state["joints_rad"], origin["joints_rad"]
                distance = math.dist(state["pose_m_rad"][:3], origin["pose_m_rad"][:3])
                width = abs(state["gripper"]["width_m"] - origin["gripper"]["width_m"])
                self.max_drift[side]["joint_rad"] = max(self.max_drift[side]["joint_rad"], max(abs(a-b) for a,b in zip(joints, baseline)))
                self.max_drift[side]["position_m"] = max(self.max_drift[side]["position_m"], distance)
                self.max_drift[side]["gripper_m"] = max(self.max_drift[side]["gripper_m"], width)
                joint_ok = all((min(start, end)-self.tolerance(side, i) <= value <= max(start, end)+self.tolerance(side, i))
                               if active else abs(value-start) <= self.tolerance(side, i)
                               for i, (value, start, end) in enumerate(zip(joints, baseline, self.target)))
                if (not joint_ok or distance > (RECOVERY["active_displacement_m"] if active else LIMITS["position_m"])
                        or width > LIMITS["gripper_m"]):
                    raise RuntimeError(side + " left the recovery joint/position/gripper envelope")
        if max(stamps)-min(stamps) > arms.MAX_SKEW_S:
            raise RuntimeError("Cross-arm feedback skew exceeds limit")
        if self.phase == "active" and time.time()-min(stamps) > RECOVERY["feedback_age_s"]:
            raise RuntimeError("Recovery requires all feedback fragments within 50 ms")
        return states

    def validate_target(self, states):
        state = states[self.arm]
        q = state["joints_rad"]
        violations = self.outside(self.arm, q)
        if not violations:
            raise RuntimeError("Selected arm has no boundary violation; not a general movement tool")
        for i, (value, target) in enumerate(zip(q, self.target)):
            low, high = self.joint_limits[self.arm]["joint%d" % (i+1)]
            if not low <= target <= high:
                raise RuntimeError("Caller target outside manufacturer joint limits; no clamp allowed")
            if abs(target-value) > RECOVERY["joint_excursion_rad"]:
                raise RuntimeError("Requested recovery exceeds 0.05 rad")
            if not low <= value <= high:
                boundary = low if value < low else high
                if target != boundary:
                    raise RuntimeError("Out-of-bounds joint must target its nearest exact manufacturer boundary")
            elif abs(target-value) > RECOVERY["joint_tolerance_rad"]:
                raise RuntimeError("Originally legal joint target differs from fresh feedback by more than 0.003 rad")
        encoded_target = [math.radians(round(value * 180/math.pi * 1000)/1000) for value in self.target]
        if self.outside(self.arm, encoded_target):
            raise RuntimeError("SDK milli-degree quantization puts target outside manufacturer limits; no correction allowed")
        robot = self.robots[self.arm]
        current_fk = arms._six_finite(robot.fk(q[:]))
        target_fk = arms._six_finite(robot.fk(encoded_target[:]))
        position_error = math.dist(current_fk[:3], state["pose_m_rad"][:3])
        rotation_error = self.rotation_distance(current_fk, state["pose_m_rad"])
        target_distance = math.dist(target_fk[:3], state["pose_m_rad"][:3])
        if position_error > 0.002 or rotation_error > 0.02:
            raise RuntimeError("Manufacturer FK does not agree with current flange feedback")
        if target_distance > RECOVERY["target_displacement_m"]:
            raise RuntimeError("Manufacturer FK recovery target exceeds 15 mm displacement")
        self.target_pose = target_fk
        self.report.update(fresh_dispatch_state=copy.deepcopy(state), target_flange_m_rad=target_fk,
                           encoded_target_joints_rad=encoded_target,
                           fk_current_flange_m_rad=current_fk, fk_feedback_position_error_m=position_error,
                           fk_feedback_rotation_error_rad=rotation_error,
                           target_flange_displacement_m=target_distance,
                           observed_boundary_violations={side: self.outside(side, states[side]["joints_rad"]) for side in SIDES})

    def dispatch(self):
        if self.dispatched:
            raise RuntimeError("Recovery already attempted; no second operation allowed")
        robot = self.robots[self.arm]
        cache = robot._msg_mode
        for key, expected in (("ctrl_mode", 1), ("mit_mode", 0), ("installation_pos", 0), ("residence_time", 0)):
            if not _allowed_integer(getattr(cache, key, None), (expected,)):
                raise RuntimeError("Unexpected cached SDK mode field " + key)
        robot.set_auto_set_motion_mode_enabled(True)  # In memory; mode and targets are one public call.
        cache.move_spd_rate_ctrl = 1
        raw = [round(value * 180/math.pi * 1000) for value in self.target]
        frames = [(0x151, bytes((1, 1, 1, 0, 0, 0, 0, 0)))] + [
            (0x155+i, struct.pack(">ii", *raw[2*i:2*i+2])) for i in range(3)]
        self.emit("joint_recovery_intent", {"arm": self.arm, "target_joints_rad": self.target,
                  "frames": [{"id": can_id, "data_hex": payload.hex()} for can_id, payload in frames]})
        self.validate_target(self.checked(self.read()))  # The journal cannot make an old anchor sufficient.
        ticket = {"side": self.arm, "thread": threading.get_ident(), "kind": "recovery",
                  "frames": frames, "bus_calls": 0, "comm_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN violation before recovery dispatch")
            self.dispatched, self.phase, self.ticket = True, "active", ticket
            try:
                robot.move_j(self.target[:])
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["comm_calls"] != 4 or ticket["bus_calls"] != 4:
                    raise RuntimeError("Incomplete four-frame SDK recovery sequence; no retry")
            finally:
                self.ticket = None
        self.sent_at = time.time()
        self.emit("joint_recovery_dispatched_unconfirmed", {"finished_unix_s": self.sent_at})

    def perform(self):
        self.validate_target(self.checked(self.read()))
        self.dispatch()
        deadline = time.monotonic()+RECOVERY["timeout_s"]
        stable_start, previous, advances = None, None, 0
        while time.monotonic() < deadline:
            states = self.checked(self.read())
            state = states[self.arm]
            q, p = state["joints_rad"], state["pose_m_rad"][:3]
            qualifies = (self.all_after(states, self.sent_at) and self.mode_confirmed
                         and state["arm_status"]["motion_status"] == 0
                         and all(abs(a-b) <= self.tolerance(self.arm, i) for i,(a,b) in enumerate(zip(q,self.target)))
                         and math.dist(p, self.target_pose[:3]) <= 0.002
                         and self.arrival_matches(state))
            if not qualifies:
                stable_start = None
            elif stable_start is None:
                stable_start, previous, advances = time.monotonic(), states, 0
                low, high, qlow, qhigh = p[:], p[:], q[:], q[:]
            else:
                low, high = [min(a,b) for a,b in zip(low,p)], [max(a,b) for a,b in zip(high,p)]
                qlow, qhigh = [min(a,b) for a,b in zip(qlow,q)], [max(a,b) for a,b in zip(qhigh,q)]
                if (math.dist(low,high) > self.stable_position_span()
                        or any(b-a > self.tolerance(self.arm,i) for i,(a,b) in enumerate(zip(qlow,qhigh)))):
                    stable_start = None
                elif self.advanced(previous,states):
                    previous, advances = states, advances+1
                    if time.monotonic()-stable_start >= RECOVERY["stable_s"] and advances >= 20:
                        self.report.update(recovery_observed=True, stable_duration_s=time.monotonic()-stable_start,
                                           stable_feedback_advances=advances, stable_position_span_m=math.dist(low,high),
                                           stable_joint_span_rad=[b-a for a,b in zip(qlow,qhigh)],
                                           final_boundary_violations={side:self.outside(side,states[side]["joints_rad"]) for side in SIDES},
                                           selected_arm_strictly_within_limits=not self.outside(self.arm,q))
                        return "boundary_target_observed_not_hold_qualified"
            time.sleep(RECOVERY["poll_s"])
        raise RuntimeError("No fresh stable three-second boundary target observation within 10 seconds")

    def run(self):
        report = _Takeover.run(self)
        count = self.counts[self.arm]["sent_frames"]
        report.update(mode_commands_sent=int(count >= 1), target_commands_sent=max(0,count-1),
                      target_calls_sent=int(count == 4), recovery_attempted=self.dispatched)
        if not report["ok"]:
            report["recovery_observed"] = False
        return report


def recover_joint_boundary(profile, journal_callback, arm, target_joints_rad,
                           recovery_profile="standard", attachment_radius_m=None,
                           available_clearance_m=None):
    """Caller supplies the exact boundary target and owns device lock/clearance.

    Requires empty enabled jaws and enabled healthy CAN-controlled joints.
    No IK, automatic coordinate correction, limit change or stop fallback.
    A successful observation never unlocks the existing motion backend.
    """
    if arm not in SIDES:
        raise ValueError("arm must be left or right")
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    target = arms._six_finite(target_joints_rad)
    if recovery_profile == "startup_j2_j3":
        from .bounded_joint_step import _positive
        from .startup_recovery import _StartupBoundaryRecovery
        attachment = _positive(attachment_radius_m, "attachment_radius_m")
        clearance = _positive(available_clearance_m, "available_clearance_m")
        return _StartupBoundaryRecovery(profile, journal_callback, arm, target,
                                        attachment, clearance).run()
    if recovery_profile != "standard":
        raise ValueError("Unknown recovery_profile")
    if attachment_radius_m is not None or available_clearance_m is not None:
        raise ValueError("Clearance arguments require explicit startup_j2_j3 recovery_profile")
    return _JointRecovery(profile, journal_callback, arm, target).run()
