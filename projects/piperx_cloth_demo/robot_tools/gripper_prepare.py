"""Enable empty grippers at their observed width; no arm targets or stop fallback.

This authorized preparation operation sends at most one manufacturer 0x159 per
disabled gripper. It is a position command, so it may move the fingers slightly;
it does not establish grasp success, calibrated force, or a safe position hold.
The caller holds the exclusive device lock and durably persists every journal
event. Arm motion eligibility is unaffected.
"""
import copy
import math

from .takeover import LIMITS, SIDES, _Takeover


WIDTH_MIN_M = 0.005
WIDTH_MAX_M = 0.070
PRE_SEND_WIDTH_TOLERANCE_M = 0.0005
AFTER_SEND_WIDTH_TOLERANCE_M = 0.001
FORCE_NOMINAL_N = 0.2


class _GripperPrepare(_Takeover):
    CONTROL_MODES = (1,)
    REQUIRE_ENABLED = False  # Six joints are checked explicitly; jaw may be off.
    FRAME_KINDS = ("gripper",)
    SCOPE = "Empty gripper enable at initial measured width only"

    def __init__(self, profile, journal_callback):
        super().__init__(profile, journal_callback)
        self.jaw_phase = dict.fromkeys(SIDES)
        self.jaw_seen_enabled = dict.fromkeys(SIDES, False)
        self.width_targets_raw = {}
        configured = profile.get("gripper_prepare_stationary_joint_tolerance_rad", {})
        if not isinstance(configured, dict) or set(configured) - set(SIDES):
            raise ValueError("Invalid gripper-preparation joint tolerance configuration")
        self.joint_tolerances = {}
        for side in SIDES:
            values = configured.get(side, [LIMITS["joint_rad"]] * 6)
            if (not isinstance(values, list) or len(values) != 6
                    or any(type(v) not in (float, int) or not math.isfinite(v)
                           or not 0.003 <= v <= 0.008 for v in values)):
                raise ValueError("Preparation feedback tolerances must be six values in [0.003,0.008] rad")
            self.joint_tolerances[side] = list(values)
        self.report.update(
            operation="prepare_grippers", fold_ready=False, grasp_verified=False,
            arm_target_commands_sent=0, gripper_target_commands_sent=0,
            gripper_enable_commands_sent=0, gripper_enabled=dict.fromkeys(SIDES),
            force_nominal_N=FORCE_NOMINAL_N, force_physically_calibrated=False,
            mode_frame_can_activate_cached_target=False,
            gripper_target_may_move_fingers=True,
            preparation_does_not_validate_position_hold=True,
            joint_feedback_tolerances_rad=copy.deepcopy(self.joint_tolerances),
            observed_joint_changes_rad={side: [0.0] * 6 for side in SIDES},
            feedback_tolerance_does_not_establish_physical_stillness=True,
            gripper_limits={"target_width_min_m": WIDTH_MIN_M,
                            "target_width_max_m": WIDTH_MAX_M,
                            "pre_send_width_tolerance_m": PRE_SEND_WIDTH_TOLERANCE_M,
                            "after_send_width_tolerance_m": AFTER_SEND_WIDTH_TOLERANCE_M})

    def check_drift(self, side, current, origin):
        differences = [abs(a - b) for a, b in zip(current["joints_rad"], origin["joints_rad"])]
        drift = {"joint_rad": max(differences),
                 "position_m": math.dist(current["pose_m_rad"][:3], origin["pose_m_rad"][:3]),
                 "gripper_m": abs(current["gripper"]["width_m"] - origin["gripper"]["width_m"])}
        for key, value in drift.items():
            self.max_drift[side][key] = max(self.max_drift[side][key], value)
        for index, (value, tolerance) in enumerate(zip(differences, self.joint_tolerances[side])):
            maxima = self.report["observed_joint_changes_rad"][side]
            maxima[index] = max(maxima[index], value)
            if value > tolerance:
                raise RuntimeError("%s J%d drift %.6f exceeds %.6f" % (side, index + 1, value, tolerance))
        for key in ("position_m", "gripper_m"):
            if drift[key] > LIMITS[key]:
                raise RuntimeError("%s drift %s=%.6f exceeds %.6f" % (side, key, drift[key], LIMITS[key]))

    def connect(self):
        super().connect()
        for side, gripper in self.grippers.items():
            # The arm parser does not encode effector messages. Restore ONLY
            # this effector's audited vendor sender; it uses the same guarded
            # comm/bus. No ticket exists here, so initialization TX stays denied.
            sender = getattr(type(gripper), "_send_msg", None)
            if not callable(sender):
                raise RuntimeError(side + " manufacturer effector sender unavailable")
            gripper._send_msg = sender.__get__(gripper, type(gripper))

    def check_enable_state(self, side, state):
        flags = [state["drivers"][str(i)]["foc_status"].get("driver_enable_status")
                 for i in range(1, 7)]
        if any(flag is not True for flag in flags):
            raise RuntimeError(side + " requires all six joint drivers continuously enabled")
        enabled = state["gripper"]["foc_status"].get("driver_enable_status")
        if type(enabled) is not bool:
            raise RuntimeError(side + " gripper enable flag must be a known boolean")
        width = state["gripper"].get("width_m")
        if isinstance(width, bool) or not isinstance(width, (int, float)) or not math.isfinite(width):
            raise RuntimeError(side + " requires finite measured gripper width")
        phase = self.jaw_phase[side]
        if phase is None:
            phase = "enabled" if enabled else "disabled"
            self.jaw_phase[side] = phase
            self.jaw_seen_enabled[side] = enabled
            if not enabled:
                if not WIDTH_MIN_M <= width <= WIDTH_MAX_M:
                    raise RuntimeError(side + " disabled gripper width outside preparation range")
                self.width_targets_raw[side] = round(width * 1e6)
        if phase == "disabled" and enabled:
            raise RuntimeError(side + " gripper enabled outside its authorized request")
        if phase == "enabling":
            if self.jaw_seen_enabled[side] and not enabled:
                raise RuntimeError(side + " gripper enable feedback regressed")
            self.jaw_seen_enabled[side] = self.jaw_seen_enabled[side] or enabled
        elif phase == "enabled" and not enabled:
            raise RuntimeError(side + " enabled gripper became disabled")
        self.report["gripper_enabled"][side] = enabled

    def frame_spec(self, kind, side):
        if kind != "gripper" or side not in self.width_targets_raw:
            raise RuntimeError("Only one observed-width gripper frame is available")
        width_raw = self.width_targets_raw[side]
        if not round(WIDTH_MIN_M * 1e6) <= width_raw <= round(WIDTH_MAX_M * 1e6):
            raise RuntimeError("Gripper target outside preparation range")
        return 0x159, width_raw.to_bytes(4, "big", signed=True) + bytes((0, 200, 1, 0))

    def prepare_side(self, side):
        state = self.checked(self.read())
        if self.jaw_phase[side] == "enabled":
            self.report["arms"][side] = {"status": "already_enabled_observed",
                                         "command_sent": False,
                                         "gripper_width_m": state[side]["gripper"]["width_m"]}
            self.emit("gripper_already_enabled_observed", {"side": side})
            return
        if self.jaw_phase[side] != "disabled":
            raise RuntimeError("Gripper preparation cannot be repeated")
        target = self.width_targets_raw[side] / 1e6
        can_id, expected = self.frame_spec("gripper", side)
        self.emit("gripper_prepare_intent", {
            "side": side, "arbitration_id": can_id, "data_hex": expected.hex(),
            "target_width_m": target, "force_nominal_N": FORCE_NOMINAL_N,
            "force_physically_calibrated": False,
            "sdk_api": "move_gripper_m(value=initial_observed_width, force=0.2)",
            "set_zero": False, "may_move_fingers": True})
        # Use fresh feedback AFTER durable intent; journal latency must not let
        # an old measured width become a materially different position target.
        state = self.checked(self.read())
        if abs(state[side]["gripper"]["width_m"] - target) > PRE_SEND_WIDTH_TOLERANCE_M:
            raise RuntimeError(side + " measured width changed more than 0.5 mm before send")
        self.jaw_phase[side] = "enabling"
        sent_at, _ = self.send_one(side, "gripper", lambda: self.grippers[side].move_gripper_m(
            value=target, force=FORCE_NOMINAL_N))
        self.emit("gripper_prepare_frame_sent_unconfirmed", {
            "side": side, "sent_at": sent_at, "transmission_counts": self.counts})
        state = self.observe_window(LIMITS["after_send_s"], requested_side=side, sent_at=sent_at)
        gripper = state[side]["gripper"]
        if gripper["foc_status"]["driver_enable_status"] is not True:
            raise RuntimeError(side + " gripper did not confirm enabled after one request")
        if abs(gripper["width_m"] - target) > AFTER_SEND_WIDTH_TOLERANCE_M:
            raise RuntimeError(side + " gripper did not remain within 1 mm of measured-width target")
        self.jaw_phase[side] = "enabled"
        self.report["arms"][side] = {"status": "gripper_enabled_at_observed_width",
                                     "command_sent": True, "target_width_m": target,
                                     "gripper_width_m": gripper["width_m"],
                                     "grasp_verified": False}
        self.emit("gripper_enabled_at_observed_width", {"side": side, "drift": self.max_drift,
                                                        "gripper_width_m": gripper["width_m"]})

    def perform(self):
        for side in SIDES:
            self.prepare_side(side)
        self.checked(self.read())
        if any(self.jaw_phase[side] != "enabled" for side in SIDES):
            raise RuntimeError("Both grippers must still be enabled at completion")
        return "grippers_prepared_not_fold_ready"

    def run(self):
        report = super().run()
        count = sum(value["gripper"]["sent_frames"] for value in self.kind_counts.values())
        report.update(target_commands_sent=count, gripper_target_commands_sent=count,
                      gripper_enable_commands_sent=count,
                      targets_raw=copy.deepcopy(self.width_targets_raw))
        return report


def prepare_grippers(profile, journal_callback):
    """Prepare empty jaws once; caller owns the device lock and durable journal.

    Requires ctrl_mode=1 and six healthy enabled joints on each arm. Already
    enabled jaws are only observed. Disabled jaws receive their initial measured
    width at SDK nominal force 0.2, with no zeroing, arm frames, retries or stops.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _GripperPrepare(profile, journal_callback).run()
