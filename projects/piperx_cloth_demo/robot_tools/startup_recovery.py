"""Explicit J2/J3 startup boundary recovery, separate from task motion limits.

Only the selected arm receives one target. Both arms must be unloaded and the
caller must establish actual whole-arm/attachment clearance and attendance.
There is no calibration, automatic recovery selection, retry, or stop fallback.
"""
import copy
import math
import time

from . import arms
from .bounded_joint_step import STEP, _BoundedJointStep
from .joint_envelope import tracking_sweep
from .joint_recovery import RECOVERY, _JointRecovery
from .takeover import SIDES

STARTUP_RECOVERY = {"joint_excursion_rad": 0.10, "joint_tolerance_rad": 0.003,
                    "relative_sweep_factor": 2.0, "clearance_reserve_m": 0.005}


class _StartupBoundaryRecovery(_BoundedJointStep):
    SCOPE = "One explicit unloaded J2/J3 boundary recovery from a nonzero startup pose"

    def __init__(self, profile, journal_callback, arm, target, attachment_radius_m,
                 available_clearance_m):
        super().__init__(profile, journal_callback, arm, target, attachment_radius_m,
                         available_clearance_m)
        self.gripper_enabled = {}
        self.report.pop("step_limits")
        self.report.pop("step_observed")
        self.report.update(operation="recover_joint_boundary", recovery_profile="startup_j2_j3",
                           recovery_limits={**RECOVERY, **STARTUP_RECOVERY,
                                            "right_j4_observation_tolerance_rad": 0.003},
                           recovery_observed=False, gripper_target_commands_sent=0,
                           passive_arm_commands_sent=0, task_motion_ready=False,
                           automatic_dispatch=False, gripper_enable_baseline={},
                           clearance_scope="Minimum surface gap to obstacles, the other arm, and non-adjacent robot/tool bodies; excludes intended joints/rigid attachments",
                           sweep_scope="Twice the all-axis tracking-box sweep bounds relative motion; caller supplies physical envelope and minimum gap",
                           completion_scope="Boundary target and ordinary feedback tolerance observed for 3 seconds; no task-motion or stop qualification")

    def health(self, side, state):
        # Keep all jaw freshness/error checks, while separating its enable bit
        # from the twelve joint enables required by this arm-only operation.
        health = arms.control_health(state, allowed_control_modes=(1,), require_enabled=False)
        if not health["healthy"]:
            return health
        for i in range(1, 7):
            if state["drivers"][str(i)]["foc_status"]["driver_enable_status"] is not True:
                raise RuntimeError(side + " recovery requires all six joint drivers enabled")
        grip = state["gripper"]
        flag = grip["foc_status"].get("driver_enable_status")
        if type(flag) is not bool or not 0 <= grip["width_m"] <= 0.070:
            raise RuntimeError(side + " requires a known jaw enable bit and width in [0, 70] mm")
        if side not in self.gripper_enabled:
            self.gripper_enabled[side] = flag
            self.report["gripper_enable_baseline"][side] = flag
        elif flag is not self.gripper_enabled[side]:
            raise RuntimeError(side + " jaw enable state changed during arm-only recovery")
        return health

    def allowed_motion_status(self, side, active):
        return (0, 1) if active else (0,)

    def checked(self, states):
        states = super().checked(states)
        for side in SIDES:
            state = states[side]
            fk = arms._six_finite(self.robots[side].fk(state["joints_rad"][:]))
            if (math.dist(fk[:3], state["pose_m_rad"][:3]) > RECOVERY["fk_position_tolerance_m"]
                    or self.rotation_distance(fk, state["pose_m_rad"]) > RECOVERY["fk_orientation_tolerance_rad"]):
                raise RuntimeError(side + " manufacturer FK disagrees with complete live flange feedback")
        stamps = [stamp for side in SIDES for stamp in states[side]["fragment_timestamps_s"].values()]
        if time.time()-min(stamps) > RECOVERY["feedback_age_s"]:
            raise RuntimeError("Startup recovery requires all feedback within 50 ms after FK checks")
        return states

    def arrival_matches(self, state):
        error = self.rotation_distance(state["pose_m_rad"], self.target_pose)
        self.report["target_flange_orientation_error_rad"] = error
        return error <= RECOVERY["fk_orientation_tolerance_rad"]

    def _wrap_bus(self, side, comm):
        super()._wrap_bus(side, comm)
        original = comm.send_bus.send
        def guarded_send(frame, *args, **kwargs):
            with self.lock:
                if self.counts[side]["attempted_frames"] >= 4:
                    self._deny(side, "Startup recovery permits at most four attempted CAN frames")
                return original(frame, *args, **kwargs)
        comm.send_bus.send = guarded_send

    def validate_target(self, states):
        state = states[self.arm]
        origin, q = self.anchor[self.arm]["joints_rad"], state["joints_rad"]
        if not self.outside(self.arm, origin) or not self.outside(self.arm, q):
            raise RuntimeError("Startup recovery requires a current boundary violation; not general motion")
        raw = [round(value*180/math.pi*1000) for value in self.target]
        if (raw != [round(value*(180/math.pi)*1000) for value in self.target]
                or raw != [round(math.degrees(value)*1000) for value in self.target]):
            raise RuntimeError("Floating half-millidegree target encodes inconsistently; refuse before mode TX")
        encoded = [math.radians(value/1000) for value in raw]
        if self.outside(self.arm, self.target) or self.outside(self.arm, encoded):
            raise RuntimeError("Caller and encoded recovery target must be within manufacturer limits")
        for i, (start, current, target) in enumerate(zip(origin, q, self.target)):
            low, high = self.joint_limits[self.arm]["joint%d" % (i+1)]
            for value in (start, current):
                if low <= value <= high:
                    if max(abs(target-value), abs(encoded[i]-value)) > STARTUP_RECOVERY["joint_tolerance_rad"]:
                        raise RuntimeError("Originally legal joint target differs from feedback by more than 0.003 rad")
                else:
                    allowed = (i == 1 and value < low) or (i == 2 and value > high)
                    if not allowed:
                        raise RuntimeError("Startup recovery only permits J2 below minimum or J3 above maximum")
                    boundary = low if value < low else high
                    if target != boundary:
                        raise RuntimeError("Out-of-bounds joint must target its nearest exact manufacturer boundary")
                    if abs(target-value) > STARTUP_RECOVERY["joint_excursion_rad"]:
                        raise RuntimeError("Startup recovery exceeds 0.10 rad")
        robot = self.robots[self.arm]
        current_fk, target_fk = arms._six_finite(robot.fk(q[:])), arms._six_finite(robot.fk(encoded[:]))
        position_error = math.dist(current_fk[:3], state["pose_m_rad"][:3])
        rotation_error = self.rotation_distance(current_fk, state["pose_m_rad"])
        target_distance = math.dist(target_fk[:3], state["pose_m_rad"][:3])
        if position_error > RECOVERY["fk_position_tolerance_m"] or rotation_error > RECOVERY["fk_orientation_tolerance_rad"]:
            raise RuntimeError("Manufacturer FK does not agree with current flange feedback")
        if target_distance > RECOVERY["target_displacement_m"]:
            raise RuntimeError("Manufacturer FK recovery target exceeds 15 mm displacement")
        rounding = [abs(requested-sent) for requested, sent in zip(self.target, encoded)]
        # The inherited monitor retains the caller's exact interval endpoints;
        # enclose those too, including when quantization moves toward origin.
        envelope = tracking_sweep(self.profile["arms"][self.arm]["model"], origin, encoded,
                                  self.attachment_radius,
                                  [self.tolerance(self.arm, i)+rounding[i] for i in range(6)],
                                  STEP["link_body_allowance_m"])
        # The other arm is monitored, not a rigid fixture. Include its allowed
        # all-axis drift too when bounding relative separation between arms.
        passive = next(side for side in SIDES if side != self.arm)
        passive_q = self.anchor[passive]["joints_rad"]
        passive_envelope = tracking_sweep(self.profile["arms"][passive]["model"], passive_q,
                                          passive_q, self.attachment_radius,
                                          [self.tolerance(passive, i) for i in range(6)],
                                          STEP["link_body_allowance_m"])
        relative = max(2*envelope["sweep_bound_m"], 2*passive_envelope["sweep_bound_m"],
                       envelope["sweep_bound_m"]+passive_envelope["sweep_bound_m"])
        budget = self.clearance-STARTUP_RECOVERY["clearance_reserve_m"]
        self.report.update(envelope, target_quantization_error_rad=rounding,
                           relative_sweep_bound_m=relative,
                           passive_sweep_bound_m=passive_envelope["sweep_bound_m"],
                           sweep_clearance_budget_m=budget)
        if relative > budget:
            raise RuntimeError("Whole-chain relative sweep bound exceeds clearance minus 5 mm reserve")
        # FK and journalling can consume the 50 ms budget. Recheck the SAME
        # samples after calculation; a new sample cannot renew an old target's
        # validation without actually checking it.
        self.checked(states)
        self.target_pose = target_fk
        self.report.update(fresh_dispatch_state=copy.deepcopy(state), target_flange_m_rad=target_fk,
                           encoded_target_joints_rad=encoded, fk_current_flange_m_rad=current_fk,
                           fk_feedback_position_error_m=position_error,
                           fk_feedback_rotation_error_rad=rotation_error,
                           target_flange_displacement_m=target_distance,
                           observed_boundary_violations={side: self.outside(side, states[side]["joints_rad"]) for side in SIDES})

    def perform(self):
        _JointRecovery.perform(self)
        # Arrival permits the ordinary 0.003 rad observation band, and retains
        # the independent strict nominal verdict. Neither is task permission.
        self.report["selected_arm_within_feedback_tolerance"] = True
        return "startup_boundary_target_observed_not_hold_qualified"

    def run(self):
        return _JointRecovery.run(self)
