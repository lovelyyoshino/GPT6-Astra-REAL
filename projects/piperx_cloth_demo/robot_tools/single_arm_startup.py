"""One standby arm's mode/enable requests; the other arm is observed only.

This separate commissioning contract does not change the dual-arm startup or
any task motion gate. A mode request or enable may activate cached controller
targets. No pose, joint, gripper, stop, reset, disable, or retry is available.
The caller must own the device lock and synchronously persist journal events.
"""
import copy
import math

from .takeover import LIMITS, SIDES, _Startup


class _SingleArmStartup(_Startup):
    CONTROL_MODES = (0, 1, 2)
    SCOPE = "One selected standby arm startup; other arm strictly passive"

    def __init__(self, profile, journal_callback, arm):
        if not isinstance(arm, str) or arm not in SIDES:
            raise ValueError("arm must be explicitly left or right")
        super().__init__(profile, journal_callback)
        self.arm = arm
        self.passive_arm = "left" if arm == "right" else "right"
        self.passive_enable_flags = None
        for drift in self.max_drift.values():
            drift["pose_rpy_component_rad"] = 0.0
        self.report.update(
            operation="startup_arm", arm=arm, selected_arm=arm,
            passive_arm=self.passive_arm, passive_arm_commands_sent=0,
            task_motion_ready=False, joint_limits_changed=False,
            additional_pose_rpy_component_limit_rad=LIMITS["joint_rad"],
            joint_values_recorded_without_task_motion_qualification=True,
            failure_policy="No retry, stop, reset, disable, or other-arm dispatch")

    def connect(self):
        super().connect()
        # The inherited comm/bus guards already reject any other-arm ticket.
        # Keep this arm's SDK entrypoints blocked as a separate defense, too.
        robot = self.robots[self.passive_arm]
        robot._send_msg = lambda *a, **k: self._deny(
            self.passive_arm, "Passive arm SDK TX forbidden during single-arm startup")
        robot._send_msgs = robot._send_msg

    def check_enable_state(self, side, state):
        if side == self.arm:
            return super().check_enable_state(side, state)
        flags = self.enable_flags(state)
        if any(type(flag) is not bool for flag in flags):
            raise RuntimeError(side + " passive arm enable flags must be known booleans")
        if self.passive_enable_flags is None:
            self.passive_enable_flags = list(flags)
        if flags != self.passive_enable_flags:
            raise RuntimeError(side + " passive arm enable state changed during single-arm startup")

    def check_drift(self, side, current, origin):
        super().check_drift(side, current, origin)
        # Additional guard specific to this new entrypoint: check every
        # reported RPY component with wraparound, at the existing angular
        # drift bound. Joint/XYZ/jaw limits and the original tools are unchanged.
        angular = max(abs(math.remainder(a - b, 2 * math.pi))
                      for a, b in zip(current["pose_m_rad"][3:], origin["pose_m_rad"][3:]))
        key = "pose_rpy_component_rad"
        self.max_drift[side][key] = max(self.max_drift[side][key], angular)
        if angular > LIMITS["joint_rad"]:
            raise RuntimeError("%s pose RPY drift %.6f exceeds %.6f" %
                               (side, angular, LIMITS["joint_rad"]))

    def frame_spec(self, kind, side):
        if side != self.arm:
            self._deny(side, "Passive arm CAN TX forbidden during single-arm startup")
        return super().frame_spec(kind, side)

    def send_one(self, side, kind, sdk_call):
        if side != self.arm:
            self._deny(side, "Only the selected arm may receive a startup request")
        return super().send_one(side, kind, sdk_call)

    def request_side(self, side):
        if side != self.arm:
            self._deny(side, "Other-arm mode request forbidden during single-arm startup")
        return super().request_side(side)

    def enable_side(self, side):
        if side != self.arm:
            self._deny(side, "Other-arm enable forbidden during single-arm startup")
        self.checked(self.read())
        if self.enable_phase[side] != "disabled" or self.required_ctrl.get(side) != 1:
            raise RuntimeError("Selected arm must confirm CAN mode while disabled before enable")
        can_id, expected = self.frame_spec("enable", side)
        self.emit("enable_request_intent", {
            "side": side, "arbitration_id": can_id, "data_hex": expected.hex(),
            "sdk_api": "enable(255)", "gripper_enable_may_change": True,
            "cached_target_activation_possible": True,
            "other_arm_remains_passive": self.passive_arm})
        # A durable journal write may take time. Recheck both arms afterwards.
        self.checked(self.read())
        self.enable_phase[side] = "enabling"
        sent_at, sdk_return = self.send_one(side, "enable", lambda: self.robots[side].enable(255))
        self.emit("enable_frame_sent_unconfirmed", {
            "side": side, "sent_at": sent_at, "sdk_cached_return": sdk_return,
            "transmission_counts": self.counts})
        state = self.observe_window(LIMITS["after_send_s"], requested_side=side, sent_at=sent_at)
        flags = self.enable_flags(state[side])
        if not all(flags[:6]):
            raise RuntimeError(side + " did not confirm all six drivers enabled after one request")
        self.enable_phase[side] = "enabled"
        self.enabled_seen[side] = list(flags)
        self.report["enabled_arms"].append(side)
        self.report["gripper_enabled"][side] = flags[6]
        self.report["arms"][side] = {
            "status": "joints_enabled_observed", "ctrl_mode": 1,
            "driver_enabled": flags[:6], "gripper_enabled": flags[6],
            "task_motion_ready": False}
        self.emit("joints_enabled_observed", {
            "side": side, "gripper_enabled": flags[6], "drift": self.max_drift})

    def perform(self):
        self.request_side(self.arm)
        self.enable_side(self.arm)
        final = self.checked(self.read())
        for side in SIDES:
            self.report["gripper_enabled"][side] = self.enable_flags(final[side])[6]
        passive = final[self.passive_arm]
        flags = self.enable_flags(passive)
        self.report["arms"][self.passive_arm] = {
            "status": "passive_state_preserved", "command_sent": False,
            "ctrl_mode": passive["arm_status"]["ctrl_mode"],
            "driver_enabled": flags[:6], "gripper_enabled": flags[6]}
        self.emit("passive_arm_state_preserved", {
            "side": self.passive_arm, "enable_flags": flags,
            "ctrl_mode": passive["arm_status"]["ctrl_mode"],
            "drift": self.max_drift[self.passive_arm]})
        return "selected_arm_joints_enabled_observed_not_task_ready"

    def run(self):
        report = super().run()
        report["passive_arm_commands_sent"] = self.counts[self.passive_arm]["sent_frames"]
        report["passive_initial_enable_flags"] = copy.deepcopy(self.passive_enable_flags)
        return report


def startup_arm(profile, journal_callback, arm):
    """Start only the named standby, fully disabled arm with two exact frames.

    Both arms must have fresh healthy, stable feedback. The passive arm may
    begin in control mode 0, 1, or 2 with any known enable state; those states
    must remain unchanged. Only the selected arm receives one low-speed mode
    request, then one enable(255) after fresh CAN-mode confirmation. Neither
    success nor the recorded joint values qualify subsequent task motion.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _SingleArmStartup(profile, journal_callback, arm).run()
