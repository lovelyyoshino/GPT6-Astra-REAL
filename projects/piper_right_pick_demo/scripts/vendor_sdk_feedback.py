"""Thread-safe feedback and checked public calls for the vendor SDK only.

Creating an adapter does not open CAN, connect, enable, or move the robot.
The caller owns connection, freshness limits, action sequencing, and stopping.
"""

import logging
import math
import threading
import time


def create_arm():
    """Return a disconnected right-arm SDK adapter (can2, no automatic init)."""
    from piper_sdk import C_PiperInterface_V2

    class ErrorCapture(logging.Handler):
        def __init__(self, owner):
            super().__init__(logging.ERROR)
            self.owner = owner

        def emit(self, record):
            # The same CAN record may reach both its logger and the root logger.
            seen = getattr(record, "_vendor_feedback_seen", set())
            if id(self) in seen:
                return
            seen.add(id(self))
            record._vendor_feedback_seen = seen
            with self.owner.error_lock:
                self.owner.send_errors.append({
                    "time_s": time.time(), "mono_s": time.monotonic(),
                    "logger": record.name, "message": record.getMessage()})

    class FeedbackArm(C_PiperInterface_V2):
        # Keep this adapter separate from the SDK's base-class singleton cache.
        _instances = {}
        _lock = threading.Lock()

        def __init__(self, *args, **kwargs):
            if getattr(self, "_adapter_initialized", False):
                return
            self.rx_lock = threading.RLock()
            self.tx_lock = threading.Lock()
            self.error_lock = threading.Lock()
            self.fatal_error = None
            self.send_errors = []
            self.rx = {}
            self.rx_wall = {}
            self._error_capture = ErrorCapture(self)
            # PIPER does not propagate. Capture python-can/hardware errors too.
            self._capture_loggers = [logging.getLogger(name)
                                     for name in ("PIPER", "can", "")]
            for logger in self._capture_loggers:
                logger.addHandler(self._error_capture)
            super().__init__(*args, **kwargs)
            self._adapter_initialized = True

        def ParseCANFrame(self, message):
            with self.rx_lock:
                try:
                    can_id = message.arbitration_id
                    tracked = 0x2A1 <= can_id <= 0x2A8 or 0x261 <= can_id <= 0x266
                    if tracked and (message.is_error_frame or message.is_remote_frame
                                    or message.is_extended_id or message.dlc != 8
                                    or len(message.data) != 8):
                        raise ValueError("Invalid feedback CAN frame: %s" % hex(can_id))
                    stamp = float(message.timestamp)
                    if tracked and (not math.isfinite(stamp) or stamp <= 0):
                        raise ValueError("Invalid feedback timestamp")
                    super().ParseCANFrame(message)
                    if not tracked:
                        return
                    if can_id == 0x2A1:
                        accepted = self.GetArmStatus().time_stamp
                    elif can_id <= 0x2A4 and can_id >= 0x2A2:
                        accepted = self.GetArmEndPoseMsgs().time_stamp
                    elif 0x2A5 <= can_id <= 0x2A7:
                        accepted = self.GetArmJointMsgs().time_stamp
                    elif can_id == 0x2A8:
                        accepted = self.GetArmGripperMsgs().time_stamp
                    else:
                        # SDK has only one low-speed timestamp; this callback
                        # holds the lock and stores it separately for each motor.
                        accepted = self.GetArmLowSpdInfoMsgs().time_stamp
                    if accepted != stamp:
                        raise ValueError("SDK rejected feedback: %s" % hex(can_id))
                    key = str(can_id)
                    self.rx[key] = time.monotonic()
                    self.rx_wall[key] = stamp
                except Exception as exc:
                    # SDK ReadCanMessage swallows callback exceptions.
                    if self.fatal_error is None:
                        self.fatal_error = "%s: %s" % (type(exc).__name__, exc)

        def snapshot(self):
            """Copy numeric fields atomically relative to SDK frame parsing."""
            with self.rx_lock:
                status = self.GetArmStatus().arm_status
                pose = self.GetArmEndPoseMsgs().end_pose
                joints = self.GetArmJointMsgs().joint_state
                motors = self.GetArmLowSpdInfoMsgs()
                gripper = self.GetArmGripperMsgs().gripper_state
                return {
                    "time_s": time.time(), "mono_s": time.monotonic(),
                    "rx": dict(self.rx), "rx_wall": dict(self.rx_wall),
                    "status": {name: int(getattr(status, name)) for name in (
                        "ctrl_mode", "arm_status", "mode_feed", "teach_status",
                        "motion_status", "err_code")},
                    "pose_mm_deg": [getattr(pose, name) / 1000.0 for name in (
                        "X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")],
                    "joints_deg": [getattr(joints, "joint_%d" % i) / 1000.0
                                   for i in range(1, 7)],
                    "motor_codes": [int(getattr(motors, "motor_%d" % i).foc_status_code)
                                    for i in range(1, 7)],
                    "gripper": {"opening_mm": gripper.grippers_angle / 1000.0,
                                "effort_nm": gripper.grippers_effort / 1000.0,
                                "status_code": int(gripper.status_code)}}

        def send_checked(self, method, *args):
            """Call a whitelisted SDK method; None is not an acknowledgement."""
            if method not in ("MotionCtrl_2", "EndPoseCtrl", "GripperCtrl", "MotionCtrl_1"):
                raise ValueError("SDK method is not permitted: %s" % method)
            if method == "MotionCtrl_1" and args != (1, 0, 0):
                raise ValueError("Only vendor quick-stop MotionCtrl_1(1,0,0) is permitted")
            with self.tx_lock:
                with self.error_lock:
                    before = len(self.send_errors)
                result = getattr(super(), method)(*args)
                with self.error_lock:
                    errors = self.send_errors[before:]
                if errors:
                    raise RuntimeError("SDK reported an error: %s" % errors[-1]["message"])
                return result

        def close_error_capture(self):
            """Detach logging hooks after the caller disconnects the SDK."""
            for logger in self._capture_loggers:
                logger.removeHandler(self._error_capture)
            self._error_capture.close()

    return FeedbackArm("can2", judge_flag=False, can_auto_init=False)
