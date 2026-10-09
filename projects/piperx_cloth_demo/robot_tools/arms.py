"""Read-only pyAgxArm adapter and manufacturer's offline FK. No motion API."""
import copy
import importlib
import math
import socket
import sys
import time
from pathlib import Path

MODELS = ("piper", "piper_h", "piper_l", "piper_x")
FIRMWARE = ("default", "v183", "v188", "v189")
MAX_AGE_S = 0.5
MAX_SKEW_S = 0.1
PARTS = ("arm_status", "joint_12", "joint_34", "joint_56",
         "end_pose_xy", "end_pose_zrx", "end_pose_ryrz")
DRIVERS = tuple("driver_state_%d" % i for i in range(1, 7))
# Manufacturer bit names, not inferred generic motor flags. Gripper bit 7 is
# homing_status (informational), whereas arm-driver bit 7 is stall_status.
DRIVER_ERRORS = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                 "driver_overheating", "collision_status", "driver_error_status", "stall_status")
GRIPPER_ERRORS = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                  "driver_overheating", "sensor_status", "driver_error_status")
ARM_ERRORS = tuple("joint_%d_angle_limit" % i for i in range(1, 7)) + tuple(
    "communication_status_joint_%d" % i for i in range(1, 7))


def _load_sdk(sdk_path):
    root = Path(sdk_path).expanduser().resolve(strict=True)
    if not (root / "pyAgxArm" / "__init__.py").is_file():
        raise ValueError("sdk_path must contain the pyAgxArm package")
    existing = sys.modules.get("pyAgxArm")
    if existing and not Path(existing.__file__).resolve().is_relative_to(root):
        raise RuntimeError("A different pyAgxArm is already imported")
    sys.path.insert(0, str(root))
    try:
        sdk = importlib.import_module("pyAgxArm")
    finally:
        sys.path.remove(str(root))
    if not Path(sdk.__file__).resolve().is_relative_to(root):
        raise RuntimeError("Imported SDK path does not match configured SDK")
    return sdk


def _six_finite(values):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError("joints_rad must be a list of six finite numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in values):
        raise ValueError("joints_rad must be a list of six finite numbers")
    return [float(v) for v in values]


def vendor_fk(model, joints_rad, sdk_path):
    """Pure vendor MDH forward kinematics; never creates a robot or CAN bus."""
    if model not in MODELS:
        raise ValueError("Unsupported Piper model")
    joints = _six_finite(joints_rad)
    sdk = _load_sdk(sdk_path)
    from pyAgxArm.utiles.mdh_kinematics import get_mdh, fk_from_mdh
    from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET
    limits = ROBOT_JOINT_LIMIT_PRESET[model]
    violations = [i + 1 for i, q in enumerate(joints)
                  if not limits["joint%d" % (i + 1)][0] <= q
                  <= limits["joint%d" % (i + 1)][1]]
    return {"status": "complete", "model": model, "joints_rad": joints,
            "pose_m_rad": fk_from_mdh(list(get_mdh(model)), joints),
            "frame": "arm_base", "reference": "flange",
            "rpy_convention": "Rz(yaw) Ry(pitch) Rx(roll)",
            "joint_limit_violations": violations,
            "source": "pyAgxArm.utiles.mdh_kinematics.fk_from_mdh",
            "sdk_version": sdk.__version__, "hardware_commands_sent": 0,
            "collision_check": "not_performed", "inverse_kinematics": "not_performed"}


def sdk_capabilities(sdk_path):
    sdk = _load_sdk(sdk_path)
    return {"sdk": "pyAgxArm", "version": sdk.__version__,
            "source_path": str(Path(sdk.__file__).resolve()),
            "models": list(MODELS), "firmware_profiles": list(FIRMWARE),
            "offline_fk": True, "offline_ik": False,
            "no_motion_ik_precheck": "unsupported_in_audited_source",
            "ik_joint_feedback": {"firmware": "v188 or v189",
                                  "requires_previous_move_p": True,
                                  "is_dry_run": False},
            "move_p_and_move_l": "send targets immediately; controller solves IK",
            "electronic_emergency_stop": "damped descent; not position hold",
            "reset": "powers off arm", "verified_position_hold": False,
            "exposed_motion_tools": [], "hardware_commands_sent": 0}


def _validate_arms(arms):
    if not isinstance(arms, dict) or not arms or set(arms) - {"left", "right"}:
        raise ValueError("arms must contain left and/or right configurations")
    channels, bindings = set(), set()
    for cfg in arms.values():
        if not isinstance(cfg, dict):
            raise ValueError("Each arm configuration must be a dict")
        if cfg.get("model") not in MODELS or cfg.get("firmware") not in FIRMWARE:
            raise ValueError("Explicit supported model and firmware profile required")
        channel, usb = cfg.get("channel"), cfg.get("usb_interface")
        if not isinstance(channel, str) or not channel.startswith("can") or not channel[3:].isdigit():
            raise ValueError("An explicit can<number> channel is required")
        if not isinstance(usb, str) or not usb or "/" in usb:
            raise ValueError("An explicit USB interface binding is required")
        if channel in channels or usb in bindings:
            raise ValueError("Arms must have distinct CAN channels and USB bindings")
        channels.add(channel)
        bindings.add(usb)


def _preflight(arms):
    """Check every binding, then open/bind/close passive sockets before SDK use."""
    for cfg in arms.values():
        interface = Path("/sys/class/net") / cfg["channel"]
        if (interface / "type").read_text().strip() != "280":
            raise ValueError("Not a CAN network interface: " + cfg["channel"])
        actual = (interface / "device").resolve(strict=True).name
        if actual != cfg["usb_interface"]:
            raise ValueError("USB binding mismatch for %s: expected %s, found %s" %
                             (cfg["channel"], cfg["usb_interface"], actual))
    probes = []
    try:
        for cfg in arms.values():
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            probes.append(sock)
            sock.bind((cfg["channel"],))
    finally:
        for sock in probes:
            sock.close()


def _blocked_send(*args, **kwargs):
    raise RuntimeError("TX is forbidden in the read-only arm tool")


def _plain(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (tuple, list)):
        return [_plain(v) for v in value]
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    return {k.lstrip("_"): _plain(v) for k, v in vars(value).items()}


def _motor_feedback(robot, now):
    """Optional RX cache diagnostics; no getters, SDK queries or conversion.

    The audited Piper default driver.get_motor_states(i) returns exactly the
    parser.motor_state_i cache (and updates Hz). Its parser already decodes
    current/position/velocity into A/rad/rad/s and estimates torque using the
    selected model's k*b*c. Copy that torque unchanged; it is not contact force.
    These independent 0x251..0x256 timestamps never join the required feedback
    fragments or any existing health/arrival predicate.
    """
    def finite(value):
        try:
            return (not isinstance(value, bool) and isinstance(value, (int, float))
                    and math.isfinite(value))
        except (ValueError, OverflowError, TypeError):
            return False

    motors = {}
    fields = {"current_A": "current", "velocity_rad_s": "velocity",
              "position_rad": "position", "estimated_torque_Nm": "torque"}
    for index in range(1, 7):
        item = {"status": "unavailable", "timestamp_s": None, "age_s": None,
                "fresh": None, **dict.fromkeys(fields), "invalid_fields": [], "read_error": None}
        motors[str(index)] = item
        try:
            fragment = copy.deepcopy(getattr(robot._parser, "motor_state_%d" % index, None))
            if fragment is None:
                continue
            stamp = getattr(fragment, "timestamp", None)
            if finite(stamp) and stamp > 0:
                item["timestamp_s"] = stamp
                if finite(now) and finite(now-stamp):
                    item["age_s"] = now-stamp
                    item["fresh"] = 0 <= item["age_s"] <= MAX_AGE_S
                else:
                    item["invalid_fields"].append("observed_at_s")
            else:
                item["invalid_fields"].append("timestamp_s")
            msg = getattr(fragment, "msg", None)
            for name, attribute in fields.items():
                value = getattr(msg, attribute, None)
                if finite(value):
                    item[name] = value
                else:
                    item["invalid_fields"].append(name)
            item["status"] = ("partial" if item["invalid_fields"] else
                              "fresh" if item["fresh"] else "stale")
        except Exception as exc:
            # An unsupported/malformed optional fragment must not break the
            # base snapshot or prevent RX diagnostics after a task fault.
            item["status"] = "read_error"
            item["read_error"] = type(exc).__name__
    states = [item["status"] for item in motors.values()]
    return {"schema": "piper_motor_feedback_v1",
            "status": "complete" if all(s == "fresh" for s in states) else
                      "unavailable" if all(s == "unavailable" for s in states) else "partial",
            "observed_at_s": now if finite(now) else None, "motors": motors,
            "timestamp_basis": "CAN_frame_host_unix_receive_time_per_motor",
            "freshness_limits_s": {"max_age": MAX_AGE_S},
            "source": "pyAgxArm.parser.motor_state_1..6 (get_motor_states RX cache)",
            "torque_basis": "manufacturer SDK estimate: current * model joint_torque_k * joint_torque_b * joint_torque_c; copied without additional scaling; not independently calibrated",
            "diagnostic_only": True, "contact_force_measured": False,
            "motion_permitted": False, "hardware_commands_sent": 0}


def snapshot(robot, gripper=None):
    """Copy manufacturer-decoded arm AND gripper feedback; never query or transmit.

    Pass the result of robot.init_effector('agx_gripper'), or an already attached
    robot._effector is used. Missing gripper feedback stays explicitly partial.
    The SDK timestamp is the CAN frame's host Unix receive time, not device time.
    """
    # These are manufacturer-decoded fragments, not our own CAN decoding.
    grouped = getattr(robot, '_pair_coherent_feedback', None)
    assembly = None
    if grouped is None:
        fragments = {name: copy.deepcopy(getattr(robot._parser, name, None))
                     for name in PARTS + DRIVERS}
    else:
        fragments, assembly = grouped.snapshot(PARTS + DRIVERS)
    if gripper is None:
        gripper = getattr(robot, "_effector", None)
    fragments["gripper"] = copy.deepcopy(getattr(getattr(gripper, "_parser", None), "gripper", None))
    now = time.time()
    missing = [name for name, msg in fragments.items() if msg is None]
    stamps = {name: msg.timestamp for name, msg in fragments.items() if msg is not None}
    ages = {name: now - stamp for name, stamp in stamps.items()}
    stale = [name for name, age in ages.items()
             if not math.isfinite(age) or not 0 <= age <= MAX_AGE_S]
    skew = max(stamps.values()) - min(stamps.values()) if stamps else None
    arm_stamps = [stamp for name, stamp in stamps.items() if name != "gripper"]
    arm_complete = (not set(PARTS + DRIVERS).intersection(missing + stale)
                    and bool(arm_stamps) and max(arm_stamps) - min(arm_stamps) <= MAX_SKEW_S)
    complete = not missing and not stale and skew is not None and skew <= MAX_SKEW_S
    result = {"status": "complete" if complete else "partial", "timestamp": now,
              "fragment_timestamps_s": stamps, "fragment_ages_s": ages,
              "timestamp_basis": "CAN_frame_host_unix_receive_time",
              "arm_telemetry_complete": arm_complete,
              "missing_fragments": missing, "stale_fragments": stale,
              "max_fragment_skew_s": skew, "snapshot_atomic": False,
              "freshness_limits_s": {"max_age": MAX_AGE_S, "max_skew": MAX_SKEW_S},
              "pose_m_rad": None, "joints_rad": None, "arm_status": None,
              "drivers": {}, "hardware_commands_sent": 0,
              "gripper": {"status": "unavailable", "reason": "no_passive_gripper_feedback_received"},
              "telemetry_scope": "arm_status, joints, flange_pose, six_driver_states, gripper"}
    if assembly is not None:
        result['feedback_assembly'] = assembly
    if all(fragments[n] is not None for n in PARTS[1:4]):
        result["joints_rad"] = [getattr(fragments["joint_" + pair].msg, "joint_%d" % i)
                                for pair, indexes in (("12", (1, 2)), ("34", (3, 4)), ("56", (5, 6)))
                                for i in indexes]
    if all(fragments[n] is not None for n in PARTS[4:]):
        result["pose_m_rad"] = [getattr(fragments[name].msg, attr)
                                for name, attrs in (("end_pose_xy", ("X_axis", "Y_axis")),
                                                    ("end_pose_zrx", ("Z_axis", "RX_axis")),
                                                    ("end_pose_ryrz", ("RY_axis", "RZ_axis")))
                                for attr in attrs]
    if fragments["arm_status"] is not None:
        result["arm_status"] = _plain(fragments["arm_status"].msg)
    result["drivers"] = {str(i): _plain(fragments["driver_state_%d" % i].msg)
                         for i in range(1, 7) if fragments["driver_state_%d" % i] is not None}
    if fragments["gripper"] is not None:
        msg = fragments["gripper"].msg
        result["gripper"] = {"status": "stale" if "gripper" in stale else "complete",
                             "timestamp": stamps["gripper"], "age_s": ages["gripper"],
                             "mode": msg.mode, "value": msg.value,
                             "value_unit": {"width": "m", "angle": "deg"}.get(msg.mode),
                             "width_m": msg.value if msg.mode == "width" else None,
                             "angle_deg": msg.value if msg.mode == "angle" else None,
                             "force_N": msg.force, "status_code": msg.status_code,
                             "foc_status": _plain(msg.foc_status),
                             "force_unit_basis": "manufacturer SDK: N; not independently calibrated"}
    if robot.has_comm_error():
        result["status"] = "partial"
        result["communication_error"] = str(robot.get_comm_error())
    result["motor_feedback"] = _motor_feedback(robot, time.time())
    result["motion_permitted"] = False  # Complete telemetry does not certify safety.
    return result


def _snapshot(robot):
    """Compatibility alias; new executors should call snapshot()."""
    return snapshot(robot)


def control_health(state, *, now_s=None, require_gripper=True,
                   allowed_control_modes=(1,), require_enabled=True):
    """Pure, conservative telemetry checks; NOT collision clearance or permission.

    Recomputes receive ages at call time, ignoring stored 'complete'/age verdicts.
    CAN ctrl_mode=1 and arm_status=0 are manufacturer enums. motion_status=0 alone
    does not prove arrival, and 1 means not reached, not a general driver fault.
    """
    now = time.time() if now_s is None else now_s
    if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
        raise ValueError("now_s must be finite Unix seconds")
    if type(require_gripper) is not bool or type(require_enabled) is not bool:
        raise ValueError("require_gripper and require_enabled must be booleans")
    if (not isinstance(allowed_control_modes, (tuple, list)) or not allowed_control_modes
            or any(type(mode) is not int for mode in allowed_control_modes)):
        raise ValueError("allowed_control_modes must contain explicit integer enum values")
    reasons = []
    def issue(code, field, detail):
        reasons.append({"code": code, "field": field, "detail": detail})
    if not isinstance(state, dict):
        issue("missing_snapshot", "snapshot", "Expected a snapshot object")
        state = {}
    required = PARTS + DRIVERS + (("gripper",) if require_gripper else ())
    stamps, valid_stamps, ages = state.get("fragment_timestamps_s", {}), {}, {}
    for name in required:
        stamp = stamps.get(name) if isinstance(stamps, dict) else None
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
            issue("missing_timestamp", name, "Missing or invalid CAN receive timestamp")
            continue
        age = now - stamp
        ages[name] = age
        if not 0 <= age <= MAX_AGE_S:
            issue("stale_feedback", name, "Receive age %.6f s outside [0, %.3f]" % (age, MAX_AGE_S))
        valid_stamps[name] = stamp
    skew = max(valid_stamps.values()) - min(valid_stamps.values()) if valid_stamps else None
    if skew is not None and skew > MAX_SKEW_S:
        issue("fragment_skew", "timestamps", "Receive skew %.6f s exceeds %.3f" % (skew, MAX_SKEW_S))
    if state.get("communication_error") or state.get("cleanup_error") or state.get("error"):
        issue("communication_or_capture_error", "snapshot", "Snapshot reports an error")
    for field in ("pose_m_rad", "joints_rad"):
        try:
            _six_finite(state.get(field))
        except ValueError:
            issue("invalid_values", field, "Six finite numeric values required")
    status = state.get("arm_status") or {}
    if not isinstance(status, dict):
        status = {}
    mode = status.get("ctrl_mode")
    if isinstance(mode, bool) or not isinstance(mode, int) or mode not in allowed_control_modes:
        issue("control_mode", "arm_status.ctrl_mode", "Expected %s; got %r" % (allowed_control_modes, mode))
    for field in ("arm_status", "err_code"):
        value = status.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value != 0:
            issue("arm_fault_or_unknown", "arm_status." + field, "Expected 0; got %r" % value)
    arm_errors = status.get("err_status")
    arm_errors = arm_errors if isinstance(arm_errors, dict) else {}
    for field in ARM_ERRORS:
        value = arm_errors.get(field)
        if value is not False:
            issue("arm_error_flag_or_unknown", "arm_status.err_status." + field, "Expected false; got %r" % value)
    drivers = state.get("drivers") or {}
    def check_flags(flags, path, errors):
        flags = flags if isinstance(flags, dict) else {}
        for field in errors:
            if flags.get(field) is not False:
                issue("driver_error_or_unknown", path + "." + field, "Expected false; got %r" % flags.get(field))
        if require_enabled and flags.get("driver_enable_status") is not True:
            issue("driver_disabled_or_unknown", path + ".driver_enable_status", "Driver must be enabled")
    for i in range(1, 7):
        driver = drivers.get(str(i), {}) if isinstance(drivers, dict) else {}
        check_flags(driver.get("foc_status") if isinstance(driver, dict) else None,
                    "drivers.%d.foc_status" % i, DRIVER_ERRORS)
    if require_gripper:
        grip = state.get("gripper") or {}
        if not isinstance(grip, dict):
            grip = {}
        if grip.get("mode") != "width":
            issue("gripper_mode_or_unknown", "gripper.mode", "Width mode required; no angle-to-width inference")
        for field in ("width_m", "force_N"):
            value = grip.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                issue("invalid_values", "gripper." + field, "Finite numeric feedback required")
        check_flags(grip.get("foc_status"), "gripper.foc_status", GRIPPER_ERRORS)
    return {"healthy": not reasons, "reasons": reasons, "checked_at_unix_s": now,
            "fragment_ages_s": ages, "max_fragment_skew_s": skew,
            "require_gripper": require_gripper, "motion_permitted": False,
            "scope": "Fresh manufacturer feedback only; not arrival, hold, collision, or path validation"}


def read_arms(arms, sdk_path, timeout_s=3.0):
    """Read both arms passively. No enable, query, reset, stop or motion command."""
    _validate_arms(arms)
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not 0.05 <= timeout_s <= 10:
        raise ValueError("timeout_s must be finite and between 0.05 and 10")
    report = {"status": "failed", "arms": {}, "hardware_commands_sent": 0,
              "synchronized": False, "motion_permitted": False}
    opened = {}
    try:
        _preflight(arms)
        sdk = _load_sdk(sdk_path)
        for side, cfg in arms.items():
            try:
                config = sdk.create_agx_arm_config(robot=cfg["model"],
                         firmeware_version=cfg["firmware"], channel=cfg["channel"],
                         interface="socketcan", auto_connect=False, enable_check_can=False)
                robot = sdk.AgxArmFactory.create_arm(config)
                opened[side] = robot
                robot._send_msg = _blocked_send
                robot._send_msgs = _blocked_send
                # Registration only: the audited effector constructor sends no CAN.
                gripper = robot.init_effector("agx_gripper")
                gripper._send_msg = _blocked_send
                # Audited connect starts RX/FPS threads and sends no initialization frames.
                robot.connect()
                comm = robot.get_context().get_comm()
                comm.send = _blocked_send
                comm.send_bus.send = _blocked_send
            except Exception as exc:
                report["arms"][side] = {"status": "failed", "error": str(exc),
                                        "hardware_commands_sent": 0}
        deadline = time.monotonic() + timeout_s
        while True:
            for side, robot in opened.items():
                if report["arms"].get(side, {}).get("status") == "failed":
                    continue
                try:
                    report["arms"][side] = _snapshot(robot)
                except Exception as exc:
                    report["arms"][side] = {"status": "failed", "error": str(exc),
                                            "hardware_commands_sent": 0}
            complete = sum(v.get("status") == "complete" for v in report["arms"].values())
            all_stamps = [stamp for value in report["arms"].values()
                          for stamp in value.get("fragment_timestamps_s", {}).values()]
            skew = max(all_stamps) - min(all_stamps) if all_stamps else None
            report["max_cross_arm_fragment_skew_s"] = skew
            ready = complete == len(arms) and skew is not None and skew <= MAX_SKEW_S
            if ready or time.monotonic() >= deadline:
                report["status"] = "complete" if ready else (
                    "partial" if any(v.get("status") != "failed" for v in report["arms"].values()) else "failed")
                break
            time.sleep(0.02)
    except Exception as exc:
        report["error"] = str(exc)
        report["arms"] = {side: {"status": "failed", "error": "Preflight failed; no SDK connection",
                                  "hardware_commands_sent": 0} for side in arms}
    finally:
        for side, robot in opened.items():
            try:
                robot.disconnect()
            except Exception as exc:
                report["arms"].setdefault(side, {})["cleanup_error"] = str(exc)
                if report["arms"][side].get("status") == "complete":
                    report["arms"][side]["status"] = "partial"
                if report["status"] == "complete":
                    report["status"] = "partial"
    for side, cfg in arms.items():
        report["arms"].setdefault(side, {}).update(
            {"configured_model": cfg["model"], "firmware_profile": cfg["firmware"],
             "model_verified": False, "firmware_verified": False,
             "channel": cfg["channel"], "usb_interface": cfg["usb_interface"]})
    report["timestamp"] = time.time()
    return report
