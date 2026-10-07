#!/usr/bin/env python3
"""Launch the pinned ROS vendor driver through a right-arm low-speed SDK adapter.

Usage: /usr/bin/python3 ros_low_speed_entry.py VENDOR_DRIVER.py [ROS remaps]
The existing health wrapper owns ROS initialization and the driver's main loop.
Importing this file opens no CAN interface. Running main DOES start the vendor
driver, including its initialization queries and now-capped J-mode command.

This is a parameter cap and device binding, not a feedback-envelope, target-count,
IK, collision, physical-force or stopping guarantee. ROS callbacks retain their
original behavior, including a jaw target in pos_cmd when gripper_exist is true.
The caller must review each single publication and its resulting fresh feedback.
"""
import hashlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import sys
import threading


HEALTH_WRAPPER = Path("/home/agilex/GPT6_bash_jiang/gpt_robot_eval_jiang/scripts/driver_with_health.py")
HEALTH_SHA256 = "cb60c162958c01948dd9ee0fae3b504aca1837e28b1986eb96a41cd5c85992ca"
CAN_NAME = "can1"
USB_INTERFACE = "1-6.3:1.0"
SPEED_CAP_PERCENT = 1
GRIPPER_EFFORT_CAP_RAW = 200  # Vendor nominal 0.2; not measured contact force.


def _integer(name, value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("%s must be an integer in [%d, %d]" % (name, low, high))
    return value


def _binding(can_name):
    if can_name != CAN_NAME or type(can_name) is not str:
        raise RuntimeError("This reviewed entry requires right-arm can1")
    interface = Path("/sys/class/net") / can_name
    device = (interface / "device").resolve(strict=True)
    if device.name != USB_INTERFACE or (interface / "type").read_text().strip() != "280":
        raise RuntimeError("can1 is not the reviewed CAN device at USB " + USB_INTERFACE)
    return str(device)


def _announce(event, **values):
    # Explicit, flushed ROS-process stdout record. This is an intent/cap log,
    # never a CAN-delivery receipt or proof of physical arrival.
    print(json.dumps({"source": "ros_low_speed_entry", "event": event, **values},
                     sort_keys=True, allow_nan=False), flush=True)


def _low_speed_class(original):
    construction_lock = threading.Lock()

    class LowSpeedPiper(original):
        def __new__(cls, *args, **kwargs):
            can_name = kwargs.get("can_name", args[0] if args else "can0")
            _binding(can_name)
            # Vendor __new__ caches by CAN name, not class. An already-cached
            # plain instance could bypass every override; never reuse one.
            with construction_lock:
                with original._lock:
                    if original._instances:
                        raise RuntimeError("SDK instance already exists; use one fresh ROS driver process")
                instance = super().__new__(cls, *args, **kwargs)
                if not isinstance(instance, cls):
                    raise RuntimeError("Vendor singleton returned an uncapped SDK instance")
                return instance

        def __init__(self, *args, **kwargs):
            import rospy
            can_name = kwargs.get("can_name", args[0] if args else "can0")
            device = _binding(can_name)
            if rospy.get_param("~can_port", None) != CAN_NAME:
                raise RuntimeError("ROS private can_port must explicitly equal can1")
            for parameter in ("~auto_enable", "~exit_teaching_mode"):
                if rospy.get_param(parameter, False) is not False:
                    raise RuntimeError(parameter + " must be boolean false; no automatic enable/teaching transition")
            _announce("adapter_bound_before_sdk_initialization", can_interface=can_name,
                      usb_device=device, speed_cap_percent=SPEED_CAP_PERCENT,
                      gripper_effort_cap_raw=GRIPPER_EFFORT_CAP_RAW,
                      gripper_width_range_raw=[0, 55000], force_calibrated=False,
                      collision_path_verified=False, feedback_envelope_enforced=False,
                      initialization_still_sends_queries_and_capped_mode=True)
            super().__init__(*args, **kwargs)

        def MotionCtrl_2(self, ctrl_mode=0x01, move_mode=0x01,
                         move_spd_rate_ctrl=50, is_mit_mode=0x00,
                         residence_time=0, installation_pos=0x00):
            # Only ordinary CAN P/J/L position-speed modes are covered by this
            # cap. Do not claim percent-speed protection for MIT/offline modes.
            _integer("ctrl_mode", ctrl_mode, 1, 1)
            _integer("move_mode", move_mode, 0, 2)
            _integer("is_mit_mode", is_mit_mode, 0, 0)
            _integer("residence_time", residence_time, 0, 0)
            _integer("installation_pos", installation_pos, 0, 0)
            speed = min(_integer("move_spd_rate_ctrl", move_spd_rate_ctrl, 0, 100), SPEED_CAP_PERCENT)
            _announce("motion_mode_before_sdk_call", ctrl_mode=ctrl_mode, move_mode=move_mode,
                      requested_speed_percent=move_spd_rate_ctrl, effective_speed_percent=speed,
                      is_mit_mode=is_mit_mode, residence_time=residence_time,
                      installation_pos=installation_pos)
            return super().MotionCtrl_2(ctrl_mode, move_mode, speed, is_mit_mode,
                                        residence_time, installation_pos)

        def GripperCtrl(self, gripper_angle=0, gripper_effort=0, gripper_code=0, set_zero=0):
            width = _integer("gripper_angle", gripper_angle, 0, 55000)
            effort = min(_integer("gripper_effort", gripper_effort, 0, 5000), GRIPPER_EFFORT_CAP_RAW)
            _integer("gripper_code", gripper_code, 1, 1)
            _integer("set_zero", set_zero, 0, 0)
            _announce("gripper_before_sdk_call", width_raw=width,
                      requested_effort_raw=gripper_effort, effective_effort_raw=effort,
                      gripper_code=gripper_code, set_zero=set_zero, force_calibrated=False)
            return super().GripperCtrl(width, effort, gripper_code, set_zero)

        def MotionCtrl_1(self, emergency_stop=0, track_ctrl=0, grag_teach_ctrl=0):
            for name, value in (("emergency_stop", emergency_stop), ("track_ctrl", track_ctrl),
                                ("grag_teach_ctrl", grag_teach_ctrl)):
                _integer(name, value, 0, 0)
            # The pinned ordinary pos_cmd callback emits this exact neutral
            # command. Reset, quick-stop, replay and teaching combinations fail.
            _announce("neutral_motion_ctrl_1_before_sdk_call")
            return super().MotionCtrl_1(0, 0, 0)

        def EnableArm(self, motor_num=7, enable_flag=0x02):
            # Retain the existing explicit ROS enable_flag path; this adapter
            # never calls it automatically and does not repeat a failed send.
            _integer("motor_num", motor_num, 7, 7)
            _integer("enable_flag", enable_flag, 2, 2)
            _announce("explicit_enable_before_sdk_call", motor_num=7, enable_flag=2)
            return super().EnableArm(motor_num, enable_flag)

        def DisableArm(self, motor_num=7, enable_flag=0x01):
            raise RuntimeError("Disable is outside this low-speed ROS entry")

    return LowSpeedPiper


def main():
    if len(sys.argv) < 2 or not Path(sys.argv[1]).is_file():
        raise SystemExit("Usage: ros_low_speed_entry.py VENDOR_DRIVER.py [ROS remaps]")
    if hashlib.sha256(HEALTH_WRAPPER.read_bytes()).hexdigest() != HEALTH_SHA256:
        raise SystemExit("Health wrapper changed; static re-audit required")
    spec = importlib.util.spec_from_file_location("piper_low_speed_existing_health", str(HEALTH_WRAPPER))
    health = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(health)
    if hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest() != health.EXPECTED_DRIVER_SHA256:
        raise SystemExit("Vendor driver changed; existing pinned health audit does not match")
    if importlib.metadata.version("piper-sdk") != health.EXPECTED_SDK_VERSION:
        raise SystemExit("SDK version does not match existing health wrapper")
    import piper_sdk
    original = piper_sdk.C_PiperInterface
    with original._lock:
        if original._instances:
            raise SystemExit("Existing SDK instance; refusing to replace a live class")
    piper_sdk.C_PiperInterface = _low_speed_class(original)
    try:
        # Its main imports our capped alias and subclasses it for receive-only
        # telemetry. It forwards argv and calls the vendor's init_node once.
        health.main()
    finally:
        piper_sdk.C_PiperInterface = original
        # No physical stop, reset, disable, retry or replacement on exit.


if __name__ == "__main__":
    main()
