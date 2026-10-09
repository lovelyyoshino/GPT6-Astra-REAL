"""Prepare one empty jaw at its measured width, with the other arm passive.

This separate preparation contract accepts measured widths of 0..70 mm. It does
not change the older dual-gripper preparation range or any task motion gate.
The one vendor position/enable command may move the fingers. The caller must
establish empty jaws and clearance, hold the device lock, and durably journal
events. No caller target, arm command, retry, calibration or stop is available.
"""
import copy
from .feedback_tolerance import rotation_tolerance
from .feedback_tolerance import PROFILE_KEY, validate_policy, joint_tolerances, joints_within, describe
import math

from .takeover import LIMITS, SIDES, _Takeover, _allowed_integer
from .linear_hold import _LinearHold


WIDTH_MIN_M = 0.0
WIDTH_MAX_M = 0.070
PRE_SEND_WIDTH_TOLERANCE_M = 0.0005
AFTER_SEND_WIDTH_TOLERANCE_M = 0.001
FORCE_NOMINAL_N = 0.2


class _SingleGripperPrepare(_Takeover):
    CONTROL_MODES = (0, 1, 2)
    REQUIRE_ENABLED = False  # Selected joints and passive flags are checked below.
    FRAME_KINDS = ("gripper",)
    SCOPE = "One empty jaw enable at measured width; other arm strictly passive"

    def __init__(self, profile, journal_callback, arm):
        if not isinstance(arm, str) or arm not in SIDES:
            raise ValueError("arm must be explicitly left or right")
        super().__init__(profile, journal_callback)
        self.feedback_policy = validate_policy(profile.get(PROFILE_KEY))
        if self.feedback_policy is not None:
            self.report["feedback_observation"] = describe(self.feedback_policy)
        self.arm = arm
        self.passive_arm = "left" if arm == "right" else "right"
        self.passive_enable_flags = None
        self.passive_control_mode = None
        self.jaw_phase = None
        self.jaw_seen_enabled = False
        self.width_target_raw = None
        self.joint_limits = {}
        self.quaternion = None
        for drift in self.max_drift.values():
            drift["pose_rpy_component_rad"] = 0.0
            drift["rotation_distance_rad"] = 0.0
        self.report.update(
            operation="prepare_gripper", arm=arm, selected_arm=arm,
            passive_arm=self.passive_arm, passive_arm_commands_sent=0,
            task_motion_ready=False, grasp_verified=False,
            arm_target_commands_sent=0, gripper_target_commands_sent=0,
            gripper_enable_commands_sent=0, mode_commands_sent=0,
            gripper_enabled=dict.fromkeys(SIDES),
            force_nominal_N=FORCE_NOMINAL_N, force_physically_calibrated=False,
            mode_frame_can_activate_cached_target=False,
            gripper_target_may_move_fingers=True,
            preparation_does_not_validate_position_hold=True,
            joint_limits_changed=False,
            joint_values_recorded_without_task_motion_qualification=True,
            joint_feedback_tolerances_rad={s: joint_tolerances(self.feedback_policy, s) for s in SIDES},
            rotation_distance_limit_rad=LIMITS["joint_rad"],
            rotation_distance_basis="SO(3) angle between manufacturer Rz(yaw) Ry(pitch) Rx(roll) quaternions",
            pose_rpy_component_rad_diagnostic_only=True,
            observed_joint_limit_violations={},
            failure_policy="No retry, stop, reset, disable or other-arm dispatch",
            gripper_limits={"target_width_min_m": WIDTH_MIN_M,
                            "target_width_max_m": WIDTH_MAX_M,
                            "pre_send_width_tolerance_m": PRE_SEND_WIDTH_TOLERANCE_M,
                            "after_send_width_tolerance_m": AFTER_SEND_WIDTH_TOLERANCE_M})

    def connect(self):
        super().connect()
        for side, robot in self.robots.items():
            if self.feedback_policy is not None:
                from .coherent_feedback import install
                if self.profile['arms'][side]['model'] != 'piper_x':
                    raise RuntimeError('Bounded feedback profile requires PiPER X')
                install(robot)
            robot._send_msg = lambda *a, _side=side, **k: self._deny(
                _side, "Arm SDK TX forbidden during single-gripper preparation")
            robot._send_msgs = robot._send_msg
        self.grippers[self.passive_arm]._send_msg = self.robots[self.passive_arm]._send_msg
        # Restore only the selected effector's encoder. Its actual comm and bus
        # remain under the inherited exact-frame guards, including initialization.
        gripper = self.grippers[self.arm]
        sender = getattr(type(gripper), "_send_msg", None)
        if not callable(sender):
            raise RuntimeError("Selected manufacturer effector encoder unavailable")
        gripper._send_msg = sender.__get__(gripper, type(gripper))
        from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET
        self.quaternion = self._manufacturer_quaternion
        self.joint_limits = {side: copy.deepcopy(ROBOT_JOINT_LIMIT_PRESET[
            self.profile["arms"][side]["model"]]) for side in SIDES}

    @staticmethod
    def enable_flags(state):
        return [state["drivers"][str(i)]["foc_status"].get("driver_enable_status")
                for i in range(1, 7)] + [state["gripper"]["foc_status"].get("driver_enable_status")]

    def checked(self, states):
        states = super().checked(states)
        self.report["observed_joint_limit_violations"] = {
            side: [{"joint_index": i + 1, "observed_rad": q,
                    "minimum_rad": low, "maximum_rad": high}
                   for i, q in enumerate(states[side]["joints_rad"])
                   for low, high in [self.joint_limits[side]["joint%d" % (i + 1)]]
                   if not low <= q <= high] for side in SIDES}
        return states

    def check_enable_state(self, side, state):
        flags = self.enable_flags(state)
        if any(type(flag) is not bool for flag in flags):
            raise RuntimeError(side + " joint/gripper enable flags must be known booleans")
        self.report["gripper_enabled"][side] = flags[6]
        if side == self.passive_arm:
            maintenance = self.profile.get('single_arm_maintenance_scope')
            if maintenance and (maintenance['selected_arm'] != self.arm
                    or maintenance['excluded_arm'] != self.passive_arm or flags[6]):
                raise RuntimeError('Isolated maintenance jaw must remain disabled and passive')
            mode = state["arm_status"]["ctrl_mode"]
            if self.passive_enable_flags is None:
                self.passive_enable_flags = list(flags)
                self.passive_control_mode = mode
            if flags != self.passive_enable_flags or mode != self.passive_control_mode:
                raise RuntimeError(side + " passive mode or enable state changed")
            return
        if not _allowed_integer(state["arm_status"]["ctrl_mode"], (1,)):
            raise RuntimeError(side + " selected arm requires CAN control mode 1")
        if not all(flags[:6]):
            raise RuntimeError(side + " requires all six joint drivers continuously enabled")
        width, enabled = state["gripper"]["width_m"], flags[6]
        if not WIDTH_MIN_M <= width <= WIDTH_MAX_M:
            raise RuntimeError(side + " measured width outside single-gripper preparation range")
        if self.jaw_phase is None:
            self.jaw_phase = "enabled" if enabled else "disabled"
            self.jaw_seen_enabled = enabled
            if not enabled:
                self.width_target_raw = round(width * 1e6)
        if self.jaw_phase == "disabled" and enabled:
            raise RuntimeError(side + " gripper enabled outside its authorized request")
        if self.jaw_phase == "enabling":
            if self.jaw_seen_enabled and not enabled:
                raise RuntimeError(side + " gripper enable feedback regressed")
            self.jaw_seen_enabled = self.jaw_seen_enabled or enabled
        elif self.jaw_phase == "enabled" and not enabled:
            raise RuntimeError(side + " enabled gripper became disabled")
        if self.width_target_raw is not None and self.jaw_phase in ("enabling", "enabled"):
            error = abs(width - self.width_target_raw / 1e6)
            self.report["width_error_m"] = error
            if error > AFTER_SEND_WIDTH_TOLERANCE_M:
                raise RuntimeError(side + " gripper width differs from measured-width target by more than 1 mm")

    # Reuse the platform's normalized, sign-invariant quaternion angle; no new
    # rotation convention, motion solver, or dependency is introduced here.
    rotation_distance = _LinearHold.rotation_distance

    @staticmethod
    def _manufacturer_quaternion(roll, pitch, yaw):
        from pyAgxArm.utiles import tf
        # The vendor converter reads a module-level convention on every call.
        # Fail closed if it is changed rather than silently measuring other axes.
        if tf.axes != "sxyz":
            raise RuntimeError("Manufacturer quaternion convention must remain sxyz")
        return tf.euler_convert_quat(roll, pitch, yaw)

    def check_drift(self, side, current, origin):
        if self.feedback_policy is None:
            super().check_drift(side, current, origin)
        else:
            drift = {"joint_rad": max(abs(a-b) for a,b in zip(current["joints_rad"], origin["joints_rad"])),
                     "position_m": math.dist(current["pose_m_rad"][:3], origin["pose_m_rad"][:3]),
                     "gripper_m": abs(current["gripper"]["width_m"]-origin["gripper"]["width_m"])}
            for key,value in drift.items():
                self.max_drift[side][key] = max(self.max_drift[side][key],value)
            if (not joints_within(self.feedback_policy, side, current["joints_rad"], origin["joints_rad"])
                    or any(drift[key] > LIMITS[key] for key in ("position_m","gripper_m"))):
                raise RuntimeError(side + " drift exceeds task-bound joint/position/jaw tolerance")
        component = max(abs(math.remainder(a - b, 2 * math.pi))
                        for a, b in zip(current["pose_m_rad"][3:], origin["pose_m_rad"][3:]))
        self.max_drift[side]["pose_rpy_component_rad"] = max(
            self.max_drift[side]["pose_rpy_component_rad"], component)
        angular = self.rotation_distance(current["pose_m_rad"], origin["pose_m_rad"])
        self.max_drift[side]["rotation_distance_rad"] = max(
            self.max_drift[side]["rotation_distance_rad"], angular)
        if angular > rotation_tolerance(self.feedback_policy,side):
            raise RuntimeError(side + " pose SO(3) rotation drift exceeds task observation bound")

    def frame_spec(self, kind, side):
        if side != self.arm:
            self._deny(side, "Passive arm CAN TX forbidden during single-gripper preparation")
        if kind != "gripper" or self.width_target_raw is None:
            raise RuntimeError("Only one selected observed-width gripper frame is available")
        if not 0 <= self.width_target_raw <= round(WIDTH_MAX_M * 1e6):
            raise RuntimeError("Measured-width target outside single-gripper preparation range")
        return 0x159, self.width_target_raw.to_bytes(4, "big", signed=True) + bytes((0, 200, 1, 0))

    def send_one(self, side, kind, sdk_call):
        if side != self.arm:
            self._deny(side, "Only the selected gripper may receive a request")
        return super().send_one(side, kind, sdk_call)

    def perform(self):
        state = self.checked(self.read())
        if self.jaw_phase == "enabled":
            self.report["arms"][self.arm] = {
                "status": "already_enabled_observed", "command_sent": False,
                "gripper_width_m": state[self.arm]["gripper"]["width_m"]}
            self.emit("single_gripper_already_enabled_observed", {"side": self.arm})
        else:
            if self.jaw_phase != "disabled":
                raise RuntimeError("Single-gripper preparation cannot be repeated")
            target = self.width_target_raw / 1e6
            can_id, expected = self.frame_spec("gripper", self.arm)
            self.emit("single_gripper_prepare_intent", {
                "side": self.arm, "arbitration_id": can_id, "data_hex": expected.hex(),
                "target_width_m": target, "force_nominal_N": FORCE_NOMINAL_N,
                "force_physically_calibrated": False, "set_zero": False,
                "sdk_api": "move_gripper_m(value=initial_observed_width, force=0.2)",
                "may_move_fingers": True, "other_arm_remains_passive": self.passive_arm})
            state = self.checked(self.read())  # Fresh after durable intent.
            if abs(state[self.arm]["gripper"]["width_m"] - target) > PRE_SEND_WIDTH_TOLERANCE_M:
                raise RuntimeError("Selected measured width changed more than 0.5 mm before send")
            self.jaw_phase = "enabling"
            sent_at, _ = self.send_one(self.arm, "gripper", lambda:
                self.grippers[self.arm].move_gripper_m(value=target, force=FORCE_NOMINAL_N))
            self.emit("single_gripper_frame_sent_unconfirmed", {
                "side": self.arm, "sent_at": sent_at, "transmission_counts": self.counts})
            state = self.observe_window(LIMITS["after_send_s"], requested_side=self.arm, sent_at=sent_at)
            if state[self.arm]["gripper"]["foc_status"]["driver_enable_status"] is not True:
                raise RuntimeError("Selected gripper did not confirm enabled after one request")
            self.jaw_phase = "enabled"
            self.report["arms"][self.arm] = {
                "status": "gripper_enabled_at_observed_width", "command_sent": True,
                "target_width_m": target, "gripper_width_m": state[self.arm]["gripper"]["width_m"],
                "grasp_verified": False}
            self.emit("single_gripper_enabled_at_observed_width", {
                "side": self.arm, "target_width_m": target, "drift": self.max_drift})
        final = self.checked(self.read())
        self.report["arms"][self.passive_arm] = {
            "status": "passive_state_preserved", "command_sent": False,
            "ctrl_mode": final[self.passive_arm]["arm_status"]["ctrl_mode"],
            "enable_flags": self.enable_flags(final[self.passive_arm])}
        return "selected_gripper_prepared_not_task_ready"

    def run(self):
        report = super().run()
        count = self.kind_counts[self.arm]["gripper"]["sent_frames"]
        report.update(target_commands_sent=count, gripper_target_commands_sent=count,
                      gripper_enable_commands_sent=count, target_width_raw=self.width_target_raw,
                      passive_arm_commands_sent=self.counts[self.passive_arm]["sent_frames"],
                      passive_initial_enable_flags=copy.deepcopy(self.passive_enable_flags))
        return report


def prepare_gripper(profile, journal_callback, arm):
    """One empty jaw's measured-width enable; caller owns device lock and clearance.

    The selected arm's six joints must be enabled in CAN mode. The other arm's
    known mode/enable state is preserved without transmission. Existing joint
    values are recorded; this never qualifies arm motion or changes joint limits.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _SingleGripperPrepare(profile, journal_callback, arm).run()
