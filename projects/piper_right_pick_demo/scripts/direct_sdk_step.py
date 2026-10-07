#!/usr/bin/env python3
"""One finite, supervised right-arm SDK commissioning step; this is not a grasp.

Only can2 / USB 1-6.3:1.0 is accepted. Target: the measured vendor end reference
+15 mm in base Z, unchanged Euler angles, MOVE_L at 5%. No ROS, reset, stop,
disable, gripper, home, replay, or automatic recovery commands are available.
Stopping this process does NOT cancel a target already accepted by the arm.
--reviewed-joint-reentry selects a separately reviewed, fixed MOVE_J target
for one recorded near-limit starting posture; it is not a general range override.
"""
import argparse
import collections
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import types

CHANNEL = "can2"
USB_DEVICE = "1-6.3:1.0"
MAX_AGE_S = .1
MOVE_SECONDS = 5.0
POSE_NAMES = ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")
JOINT_NAMES = tuple("joint_%d" % i for i in range(1, 7))
JOINT_LIMITS_DEG = ((-150, 150), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120))
REENTRY_START = (36562, -1420, 2961, -6295, 19667, -3052)
REENTRY_TARGET = (36562, 288, -262, -5864, 21174, -3512)
REENTRY_POSE = (44620, 28912, 166421, -149980, 71334, -114259)
REENTRY_SOURCE = Path(__file__).resolve().parents[1] / "runs/geometry_review/ik_reentry_180614_analysis.json"
REENTRY_PROGRESS_SOURCE = Path(__file__).resolve().parents[1] / "runs/geometry_review/ik_reentry_180614_progress_box.json"
REENTRY_OBSERVATION = Path(__file__).resolve().parents[1] / "runs/sdk_step_20261003T180614_69681/report.json"
POSE_PARTS = {0x2A2: POSE_NAMES[:2], 0x2A3: POSE_NAMES[2:4], 0x2A4: POSE_NAMES[4:]}
JOINT_PARTS = {0x2A5: JOINT_NAMES[:2], 0x2A6: JOINT_NAMES[2:4], 0x2A7: JOINT_NAMES[4:]}
REQUIRED = (0x2A1,) + tuple(POSE_PARTS) + tuple(JOINT_PARTS) + tuple(range(0x261, 0x267))
FAULT_NAMES = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
               "driver_overheating", "collision_status", "driver_error_status", "stall_status")
FRAME = struct.Struct("=IB3x8s")


class Rejected(RuntimeError):
    pass


def control_identifier(identifier):
    return (0x150 <= identifier <= 0x19F or
            identifier in (0x470, 0x471, 0x472, 0x474, 0x475, 0x477, 0x479, 0x47A, 0x47D))


class Vendor:
    """Only the installed, inspected vendor version supplies wire decoding."""
    def __init__(self):
        version = importlib.metadata.version("piper_sdk")
        if version != "0.6.2":
            raise Rejected("Expected inspected piper_sdk 0.6.2, found " + version)
        from can import Message
        from piper_sdk import C_PiperInterface_V2, C_PiperParserV2, C_PiperForwardKinematics
        from piper_sdk.piper_msgs.msg_v2 import PiperMessage
        self.Message, self.PiperMessage = Message, PiperMessage
        self.parser = C_PiperParserV2()
        self.fk = C_PiperForwardKinematics(dh_is_offset=1)
        # No bus, reading thread, initialization query, or command is created here.
        self.sdk = C_PiperInterface_V2(CHANNEL, judge_flag=False, can_auto_init=False)
        self.sources = {}
        for obj in (C_PiperInterface_V2, C_PiperParserV2, C_PiperForwardKinematics):
            path = Path(sys.modules[obj.__module__].__file__).resolve()
            self.sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

    def fk_pose_raw(self, joints):
        return [x * 1000. for x in self.fk.CalFK([math.radians(x / 1000.) for x in joints])[-1]]

    def decode(self, identifier, payload):
        message = self.Message(arbitration_id=identifier, data=payload,
                               is_extended_id=False, timestamp=time.time())
        decoded = self.PiperMessage()
        if not self.parser.DecodeMessage(message, decoded):
            return None
        # Feed only observed nonlocal frames into the SDK's feedback cache. This
        # permits EnablePiper to inspect real motor states without an SDK thread.
        self.sdk.ParseCANFrame(message)
        return decoded


class Feedback:
    def __init__(self, vendor, reviewed_reentry=False):
        self.vendor = vendor
        self.reviewed_reentry = reviewed_reentry
        self.received = {}
        self.pose, self.joints, self.motors = {}, {}, {}
        self.status = None
        self.fault = None
        self.counts = collections.Counter()
        self.own_echo_budget = collections.Counter()
        self.raw_frame_latest = {}
        self.snapshot_rejections = collections.Counter()
        self.last_snapshot_rejection = None
        self.first_complete_diagnostic = None
        self.gripper = None

    def reject(self, reason):
        if self.fault is None:
            self.fault = reason
        raise Rejected(self.fault)

    def ingest(self, identifier, payload, local, received):
        self.counts["0x%03X" % identifier] += 1
        self.raw_frame_latest["0x%03X" % identifier] = {
            "payload_hex": bytes(payload).hex(), "origin": "local" if local else "nonlocal",
            "kernel_received_monotonic_s": received,
            "kernel_age_s_when_processed": time.monotonic() - received}
        key = (identifier, bytes(payload))
        if local:
            if self.own_echo_budget[key] > 0:
                self.own_echo_budget[key] -= 1
                return
            self.reject("Other local CAN sender observed: 0x%03X" % identifier)
        if control_identifier(identifier):
            self.reject("Control/configuration traffic already on bus: 0x%03X" % identifier)
        decoded = self.vendor.decode(identifier, payload)
        if decoded is None:
            return
        if identifier == 0x2A1:
            obj = decoded.arm_status_msgs
            self.status = {name: int(getattr(obj, name)) for name in
                           ("ctrl_mode", "arm_status", "mode_feed", "teach_status", "motion_status", "trajectory_num", "err_code")}
            if (self.status["ctrl_mode"] not in (0, 1) or self.status["arm_status"] != 0 or
                    self.status["mode_feed"] not in (0, 1, 2, 3) or
                    self.status["teach_status"] not in (0, 2) or self.status["err_code"] != 0 or
                    self.status["motion_status"] not in (0, 1)):
                self.reject("Non-normal/teach/MIT/unknown arm status: " + str(self.status))
        elif identifier in POSE_PARTS:
            for name in POSE_PARTS[identifier]:
                self.pose[name] = int(getattr(decoded.arm_end_pose, name))
        elif identifier in JOINT_PARTS:
            for name in JOINT_PARTS[identifier]:
                self.joints[name] = int(getattr(decoded.arm_joint_feedback, name))
        elif 0x261 <= identifier <= 0x266:
            number = identifier - 0x260
            obj = getattr(decoded, "arm_low_spd_feedback_%d" % number)
            faults = [name for name in FAULT_NAMES if getattr(obj.foc_status, name)]
            self.motors[number] = {"enabled": bool(obj.foc_status.driver_enable_status), "faults": faults,
                                   "vol_raw": obj.vol, "foc_temp": obj.foc_temp, "motor_temp": obj.motor_temp,
                                   "bus_current_raw": obj.bus_current, "foc_status_code": obj.foc_status_code}
            if faults:
                self.reject("Motor %d fault: %s" % (number, ", ".join(faults)))
        elif identifier == 0x2A8:
            obj = decoded.gripper_feedback
            # The same named error flags exist on the vendor gripper status.
            faults = [name for name in FAULT_NAMES + ("sensor_status",)
                      if getattr(obj.foc_status, name, False)]
            self.gripper = {"angle_raw": obj.grippers_angle, "effort_raw": obj.grippers_effort,
                            "status_code": obj.status_code, "faults": faults}
            if faults:
                self.reject("Gripper fault: " + ", ".join(faults))
        if identifier in REQUIRED:
            self.received[identifier] = received
        if self.first_complete_diagnostic is None and all(i in self.received for i in REQUIRED):
            self.first_complete_diagnostic = self.diagnostic(time.monotonic())

    def snapshot(self, now, require_enabled=False):
        if self.fault:
            raise Rejected(self.fault)
        stale = ["0x%03X" % i for i in REQUIRED
                 if i not in self.received or not 0 <= now - self.received[i] < MAX_AGE_S]
        if stale:
            raise Rejected("Missing/stale individual feedback frames: " + ", ".join(stale))
        if require_enabled and not all(self.motors[i]["enabled"] for i in range(1, 7)):
            raise Rejected("A joint motor is not enabled")
        result = {"monotonic_s": now, "pose_raw": [self.pose[n] for n in POSE_NAMES],
                  "joints_raw": [self.joints[n] for n in JOINT_NAMES], "status": dict(self.status),
                  "motor_enabled": [self.motors[i]["enabled"] for i in range(1, 7)],
                  "frame_age_s": {"0x%03X" % i: now - self.received[i] for i in REQUIRED}}
        if any(abs(x) > 1000000 for x in result["pose_raw"][:3]):
            raise Rejected("Implausible end position")
        if any(abs(x) > 360000 for x in result["pose_raw"][3:]):
            raise Rejected("Implausible end orientation")
        # A one-degree boundary tolerance accommodates encoder quantization and
        # near-zero noise; this is not permission to command beyond joint limits.
        self.validate_joint_range(result["joints_raw"])
        return result

    def validate_joint_range(self, joints):
        if self.reviewed_reentry:
            if not in_reentry_box(joints):
                raise Rejected("Measured joints left the specifically reviewed reentry interval")
        elif any(not low - 1 <= raw / 1000. <= high + 1 for raw, (low, high)
                 in zip(joints, JOINT_LIMITS_DEG)):
            raise Rejected("Measured joint outside inspected manufacturer range")

    def diagnostic(self, now):
        """Save evidence even when no sample qualifies for controlling motion."""
        raw_frames = {key: dict(value, kernel_age_s_at_report=now - value["kernel_received_monotonic_s"])
                      for key, value in self.raw_frame_latest.items()}
        return {"qualified_for_motion": False, "pose_raw": dict(self.pose),
                "joints_raw": dict(self.joints), "status": self.status,
                "motors": dict(self.motors), "gripper": self.gripper,
                "required_frame_age_s": {"0x%03X" % i: now - self.received[i]
                                         if i in self.received else None for i in REQUIRED},
                "raw_frame_latest": raw_frames,
                "snapshot_rejection_counts": dict(self.snapshot_rejections),
                "last_snapshot_rejection": self.last_snapshot_rejection}


class Receiver:
    def __init__(self, feedback):
        self.feedback = feedback
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Linux SO_TIMESTAMPNS=35; kernel receive timestamps prevent old queued
        # frames from looking fresh merely because Python read them recently.
        self.socket.setsockopt(socket.SOL_SOCKET, 35, 1)
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack("=I", 0x1FFFFFFF))
        self.socket.bind((CHANNEL,))

    def one(self, timeout=.01):
        self.socket.settimeout(timeout)
        try:
            raw, ancillary, flags, _ = self.socket.recvmsg(FRAME.size, 128)
        except (socket.timeout, BlockingIOError):
            return False
        if len(raw) != FRAME.size or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            self.feedback.reject("Truncated CAN frame/timestamp")
        identifier, dlc, payload = FRAME.unpack(raw)
        if identifier & socket.CAN_ERR_FLAG:
            self.feedback.reject("CAN bus error frame")
        if identifier & (socket.CAN_RTR_FLAG | socket.CAN_EFF_FLAG) or dlc != 8:
            self.feedback.reject("Unexpected CAN frame format")
        timestamp_ns = None
        for level, kind, value in ancillary:
            if level == socket.SOL_SOCKET and kind == 35 and len(value) >= struct.calcsize("@ll"):
                sec, ns = struct.unpack("@ll", value[:struct.calcsize("@ll")])
                timestamp_ns = sec * 1000000000 + ns
        if timestamp_ns is None:
            self.feedback.reject("Kernel CAN timestamp unavailable")
        age = (time.time_ns() - timestamp_ns) / 1e9
        if age < -.005:
            self.feedback.reject("Host clock changed; cannot assess feedback age")
        self.feedback.ingest(identifier, payload, bool(flags & socket.MSG_DONTROUTE),
                             time.monotonic() - max(0, age))
        return True

    def drain(self):
        deadline = time.monotonic() + .05
        while self.one(0):
            if time.monotonic() >= deadline:
                raise Rejected("CAN receive backlog did not clear")

    def close(self):
        self.socket.close()


def inspect_binding():
    device = (Path("/sys/class/net") / CHANNEL / "device").resolve().name
    if device != USB_DEVICE:
        raise Rejected("can2 USB binding mismatch: " + device)
    command = ["ip", "-details", "-json", "link", "show", "dev", CHANNEL]
    result = subprocess.run(command, capture_output=True, text=True, timeout=3, check=True)
    items = json.loads(result.stdout)
    if len(items) != 1:
        raise Rejected("Ambiguous CAN interface")
    link = items[0]
    info = link.get("linkinfo", {})
    data = info.get("info_data", {})
    if (info.get("info_kind") != "can" or "UP" not in link.get("flags", []) or
            data.get("bittiming", {}).get("bitrate") != 1000000 or data.get("state") != "ERROR-ACTIVE"):
        raise Rejected("can2 must already be UP, 1 Mbit/s, ERROR-ACTIVE: " + str(data))
    return {"channel": CHANNEL, "usb_device": device, "ifindex": link["ifindex"], "link": link}


def inspect_controllers():
    found = []
    for proc in Path("/proc").glob("[0-9]*"):
        if int(proc.name) == os.getpid():
            continue
        try:
            arguments = (proc / "cmdline").read_bytes().split(b"\0")
            names = [Path(a.decode(errors="replace")).name for a in arguments[:5] if a]
        except (OSError, ValueError):
            continue
        risky = [name for name in names if
                 (name.startswith("piper_") and name.endswith(".py") and
                  not name.startswith("piper_read_")) or
                 any(token in name.lower() for token in ("teleop", "master_right", "piper_start", "piper_ctrl")) or
                 name in ("cansend", "cangen", "roslaunch", "direct_sdk_step.py", "direct_sdk_live_step.py")]
        if risky:
            found.append({"pid": int(proc.name), "programs": risky})
    if found:
        raise Rejected("Stop existing controller processes first: " + str(found))
    return found


def pose_difference(a, b):
    position = math.sqrt(sum(((x - y) / 1000.) ** 2 for x, y in zip(a[:3], b[:3])))
    orientation = max(abs((x - y + 180000) % 360000 - 180000) / 1000. for x, y in zip(a[3:], b[3:]))
    return position, orientation


def stationary(samples):
    if len(samples) < 2 or samples[-1]["monotonic_s"] - samples[0]["monotonic_s"] < .25:
        return False
    for key, limits in (("pose_raw", [1000] * 3 + [500] * 3), ("joints_raw", [300] * 6)):
        for index, limit in enumerate(limits):
            values = [sample[key][index] for sample in samples]
            if max(values) - min(values) > limit:
                return False
    return all(sample["status"]["motion_status"] == 0 for sample in samples)


def collect_stationary(receiver, state, trace, require_enabled=False):
    deadline = time.monotonic() + 1.5
    samples = []
    last_record = 0.
    while time.monotonic() < deadline:
        receiver.one(.01)
        now = time.monotonic()
        if now - last_record < .01:
            continue
        try:
            sample = state.snapshot(now, require_enabled)
        except Rejected as exc:
            state.last_snapshot_rejection = str(exc)
            state.snapshot_rejections[str(exc)] += 1
            if state.fault:
                raise
            samples = []
            continue
        last_record = now
        trace.append(sample)
        samples.append(sample)
        samples = [s for s in samples if now - s["monotonic_s"] <= .4]
        if stationary(samples):
            receiver.drain()
            return state.snapshot(time.monotonic(), require_enabled)
    if not samples:
        reason = state.last_snapshot_rejection or "No complete snapshot was received"
    elif any(s["status"]["motion_status"] != 0 for s in samples):
        reason = "Feedback motion_status is not 0 (target not reached), despite normal fault status"
    else:
        spans = {key: [max(s[key][i] for s in samples) - min(s[key][i] for s in samples)
                       for i in range(6)] for key in ("pose_raw", "joints_raw")}
        reason = "No stable 0.25 s window; raw coordinate spans=" + str(spans)
    raise Rejected("No complete, fresh, stationary feedback window: " + reason)


class RecordingPort:
    """No socket: obtain exact approved frames from official SDK API encoding."""
    CAN_STATUS = types.SimpleNamespace(SEND_MESSAGE_SUCCESS=object())

    def __init__(self):
        self.frames = []

    def SendCanMessage(self, identifier, data, *args, **kwargs):
        self.frames.append((identifier, bytes(data)))
        return self.CAN_STATUS.SEND_MESSAGE_SUCCESS


def encoded_plan(sdk, pose, reviewed_reentry=False):
    # Version-pinned transport hook is used solely to preview and guard the
    # SDK's exact outgoing frames. We do not reimplement any Piper encoding.
    attribute = "_C_PiperInterface_V2__arm_can"
    original = getattr(sdk, attribute)
    capture = RecordingPort()
    setattr(sdk, attribute, capture)
    try:
        sdk.EnableArm(7)
        enable = capture.frames[:]
        capture.frames.clear()
        sdk.MotionCtrl_2(1, 1 if reviewed_reentry else 2, 5, 0)
        if reviewed_reentry:
            if not in_nominal_range(REENTRY_TARGET):
                raise Rejected("Reviewed joint target is outside nominal limits")
            sdk.JointCtrl(*REENTRY_TARGET)
        else:
            sdk.EndPoseCtrl(*pose)
        motion = capture.frames[:]
    finally:
        setattr(sdk, attribute, original)
    expected = [0x151, 0x155, 0x156, 0x157] if reviewed_reentry else [0x151, 0x152, 0x153, 0x154]
    if [i for i, _ in enable] != [0x471] or [i for i, _ in motion] != expected:
        raise Rejected("Unexpected SDK command encoding")
    return {"enable": enable, "motion": motion}


class TransmitGuard:
    def __init__(self, transport, plan, state, receiver, report, command_limits=None):
        self.original = transport.SendCanMessage
        self.success = transport.CAN_STATUS.SEND_MESSAGE_SUCCESS
        self.plan, self.state, self.receiver, self.report = plan, state, receiver, report
        self.pending = []
        self.calls = collections.Counter()
        self.deadline = None
        self.require_enabled = False
        self.command_limits = command_limits or {"enable": 3, "motion": 1}

    def allow(self, name):
        if self.pending or name not in self.plan or self.calls[name] >= self.command_limits.get(name, 0):
            raise Rejected("Command count/order budget exceeded")
        self.calls[name] += 1
        self.pending = list(self.plan[name])

    def send(self, identifier, data, dlc=8, is_extended_id=False):
        self.receiver.drain()
        self.state.snapshot(time.monotonic(), self.require_enabled)
        frame = (identifier, bytes(data))
        if not self.pending or self.pending[0] != frame or dlc != 8 or is_extended_id:
            raise Rejected("Outgoing frame differs from the one approved SDK step")
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise Rejected("Motion transmission deadline expired")
        self.pending.pop(0)
        entry = {"id": "0x%03X" % identifier, "data_hex": bytes(data).hex(),
                 "monotonic_s": time.monotonic(), "send_result": "attempting"}
        self.report["transmissions"].append(entry)
        self.state.own_echo_budget[frame] += 1
        result = self.original(identifier, data, dlc, is_extended_id)
        entry["send_result"] = "accepted_by_host_socket" if result is self.success else str(result)
        if result is not self.success:
            raise Rejected("SDK CAN send did not succeed")
        return result


def check_path(sample, start, reviewed_reentry=False):
    pose, origin = sample["pose_raw"], start["pose_raw"]
    if reviewed_reentry:
        outside = (math.hypot(pose[0] - origin[0], pose[1] - origin[1]) > 3000 or
                   not -7000 <= pose[2] - origin[2] <= 23000 or rotation_distance_deg(pose, origin) > 5)
    else:
        outside = (abs(pose[0] - origin[0]) > 3000 or abs(pose[1] - origin[1]) > 3000 or
                   not -2000 <= pose[2] - origin[2] <= 18000 or pose_difference(pose, origin)[1] > 2)
    if outside:
        raise Rejected("Measured pose left the reviewed joint-step corridor" if reviewed_reentry
                       else "Measured pose left the 15 mm vertical-step corridor")
    if reviewed_reentry and not in_reentry_box(sample["joints_raw"]):
        raise Rejected("Measured joints left the specifically reviewed reentry interval")
    if max(abs(x - y) for x, y in zip(sample["joints_raw"], start["joints_raw"])) > 10000:
        raise Rejected("A joint moved more than 10 degrees during the short step")


def require_same_pose(sample, reference, message):
    position, orientation = pose_difference(sample["pose_raw"], reference["pose_raw"])
    joints = max(abs(x - y) for x, y in zip(sample["joints_raw"], reference["joints_raw"]))
    if position > 1 or orientation > .5 or joints > 500:
        raise Rejected(message)


def in_nominal_range(joints):
    return all(low <= raw / 1000. <= high for raw, (low, high) in zip(joints, JOINT_LIMITS_DEG))


def in_reentry_box(joints):
    return len(joints) == 6 and all(min(a, b) - 150 <= q <= max(a, b) + 150
                                  for q, a, b in zip(joints, REENTRY_START, REENTRY_TARGET))


def rotation_distance_deg(a, b):
    def quaternion(pose):
        r, p, y = [math.radians(x / 1000.) / 2 for x in pose[3:]]
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
    dot = abs(sum(x * y for x, y in zip(quaternion(a), quaternion(b))))
    return math.degrees(2 * math.acos(min(1., max(0., dot))))


def validate_reviewed_start(sample, vendor, require_standby_disabled=False):
    if require_standby_disabled and (sample["status"]["ctrl_mode"] != 0 or any(sample["motor_enabled"])):
        raise Rejected("Reviewed recovery requires the recorded standby state with all joint motors disabled")
    if max(abs(q - reference) for q, reference in zip(sample["joints_raw"], REENTRY_START)) > 150:
        raise Rejected("This reviewed recovery only accepts its recorded starting posture (0.15 degree)")
    pos, rot = pose_difference(sample["pose_raw"], REENTRY_POSE)
    if pos > 1 or rot > .3:
        raise Rejected("Current end pose does not match the reviewed starting reference")
    current_fk = vendor.fk_pose_raw(sample["joints_raw"])
    pos, rot = pose_difference(current_fk, sample["pose_raw"])
    if pos > .3 or rot > .05:
        raise Rejected("Measured start and official SDK FK disagree for reviewed recovery")
    target_fk = vendor.fk_pose_raw(REENTRY_TARGET)
    desired = list(REENTRY_POSE)
    desired[2] += 15000
    pos, rot = pose_difference(target_fk, desired)
    if pos > .05 or rot > .01 or not in_nominal_range(REENTRY_TARGET):
        raise Rejected("Fixed reviewed endpoint no longer satisfies FK/nominal-limit check")
    return [round(x) for x in target_fk]


def arrived(sample, target, state, sent_at):
    position, orientation = pose_difference(sample["pose_raw"], target)
    reviewed = state.reviewed_reentry
    joint_arrival = (not reviewed or (in_nominal_range(sample["joints_raw"]) and
                     max(abs(q - goal) for q, goal in zip(sample["joints_raw"], REENTRY_TARGET)) <= 100))
    return (position <= 2 and orientation <= 1 and all(sample["motor_enabled"]) and
            joint_arrival and sample["status"]["ctrl_mode"] == 1 and
            sample["status"]["mode_feed"] == (1 if reviewed else 2) and
            sample["status"]["motion_status"] == 0 and
            all(state.received[i] > sent_at for i in REQUIRED))


def execute_step(vendor, receiver, state, report):
    receiver.drain()
    report["preflight"] = collect_stationary(receiver, state, report["trace"])
    require_same_pose(report["preflight"], report["preview"], "Pose changed after preview; no commands sent")
    if state.reviewed_reentry:
        target = validate_reviewed_start(report["preflight"], vendor, require_standby_disabled=True)
        if target != report["preview_target_raw"]:
            raise Rejected("Reviewed endpoint changed from preview")
    inspect_controllers()
    binding = inspect_binding()
    if binding["ifindex"] != report["binding"]["ifindex"]:
        raise Rejected("CAN interface changed after preflight")
    initial = report["preflight"]
    target = list(report["preview_target_raw"])
    plan = encoded_plan(vendor.sdk, target, state.reviewed_reentry)
    vendor.sdk.CreateCanBus(CHANNEL, expected_bitrate=1000000, judge_flag=False)
    transport = getattr(vendor.sdk, "_C_PiperInterface_V2__arm_can")
    guard = TransmitGuard(transport, plan, state, receiver, report)
    transport.SendCanMessage = guard.send
    try:
        if not all(initial["motor_enabled"]):
            for _ in range(3):
                guard.allow("enable")
                vendor.sdk.EnablePiper()
                deadline = time.monotonic() + .4
                while time.monotonic() < deadline:
                    receiver.one(.01)
                    sample = state.snapshot(time.monotonic())
                    position, orientation = pose_difference(sample["pose_raw"], initial["pose_raw"])
                    if position > 1 or orientation > .5 or max(
                            abs(x - y) for x, y in zip(sample["joints_raw"], initial["joints_raw"])) > 500:
                        raise Rejected("Arm drifted while enabling; no Cartesian target sent")
                    if all(sample["motor_enabled"]):
                        break
                if all(state.snapshot(time.monotonic())["motor_enabled"]):
                    break
            state.snapshot(time.monotonic(), require_enabled=True)
        guard.require_enabled = True
        start = collect_stationary(receiver, state, report["trace"], require_enabled=True)
        require_same_pose(start, initial, "Pose/orientation/joints changed before movement")
        require_same_pose(start, report["preview"], "Pose changed from the displayed preview")
        if state.reviewed_reentry:
            validate_reviewed_start(start, vendor)
        report["start"], report["target_raw"] = start, target
        report["phase"] = "sending_reviewed_joint_step" if state.reviewed_reentry else "sending_one_cartesian_step"
        guard.deadline = time.monotonic() + MOVE_SECONDS
        guard.allow("motion")
        vendor.sdk.MotionCtrl_2(1, 1 if state.reviewed_reentry else 2, 5, 0)
        if state.reviewed_reentry:
            vendor.sdk.JointCtrl(*REENTRY_TARGET)
        else:
            vendor.sdk.EndPoseCtrl(*target)
        if guard.pending:
            raise Rejected("SDK did not send all expected target frames")
        sent_at = time.monotonic()
        report["target_sent_at_monotonic_s"] = sent_at
        report["phase"] = "watching_feedback"
        settled = []
        last_record = 0.
        while time.monotonic() < guard.deadline:
            receiver.one(.01)
            now = time.monotonic()
            sample = state.snapshot(now, require_enabled=True)
            check_path(sample, start, state.reviewed_reentry)
            if now - last_record >= .01:
                report["trace"].append(sample)
                last_record = now
                if arrived(sample, target, state, sent_at):
                    settled.append(sample)
                    if stationary(settled):
                        receiver.drain()
                        final = state.snapshot(time.monotonic(), require_enabled=True)
                        check_path(final, start, state.reviewed_reentry)
                        if not arrived(final, target, state, sent_at):
                            settled = []
                            continue
                        report.update(status="arrived", arrival_verified=True, final=final, phase="complete")
                        return
                else:
                    settled = []
        raise Rejected("Five-second deadline: target arrival was not verified")
    finally:
        # Closing communication is NOT a physical stop or target cancellation.
        # Do not reset, disable, or E-stop here: these can allow the arm to fall.
        transport.Close()


def readiness_prompt(reviewed):
    if reviewed:
        print("本次执行已审查的一次 MOVE_J 小幅恢复，5% 速度；目标回到关节正常范围，末端约抬高 15 mm。")
        print("已审查的起点关节角(deg)：", [x / 1000. for x in REENTRY_START])
        print("固定目标关节角(deg)：", [x / 1000. for x in REENTRY_TARGET])
        print("记录中的起点 XYZ(mm), RPY(deg)，实时值将在 Enter 后读取：", [x / 1000. for x in REENTRY_POSE])
        print("仅接受上述记录附近的起点，不开合夹爪、不回零；这是运动验证，尚非抓取。")
        print("关节运动途中可能短暂下探约 15 mm，并非全程向上。")
        print("请确认：基座正装；夹爪空；整个运动路径中，运动部位、夹爪、腕相机及线缆与桌面和周围物体")
        print("在各方向（包括下方）至少有 30 mm 净空；")
    else:
        print("请确认：已重启且未进入示教；基座正装；夹爪空；末端上方至少 20 mm 空闲，整臂/相机/线缆无碰撞路径；")
    print("手和支撑物已移出运动范围，可随时切断电源（失电时机械臂可能下落）。")
    print("Ctrl-C 仅停止程序；已下发目标可能继续，脚本不会自动失能/复位。")
    if reviewed:
        print("按 Enter 后不再手动调整机械臂；程序读取稳定姿态，核验通过后直接执行一次，无第二次等待。")
    if not sys.stdin.isatty() or input("满足以上条件后按 Enter 开始一次；输入任何文字取消：").strip():
        raise Rejected("Operator cancelled or input is not an interactive terminal")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--reviewed-joint-reentry", action="store_true",
                        help="one fixed, reviewed MOVE_J endpoint from the recorded near-limit posture only")
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.output_dir / "report.json"
    if report_path.exists():
        parser.error("Refusing to overwrite an existing report; use a new output directory")
    report = {"status": "not_started", "arrival_verified": False, "phase": "preparation",
              "started_at_s": time.time(), "channel": CHANNEL, "sdk_version": "0.6.2",
              "motion": "one base-Z +15 mm MOVE_L at 5%, unchanged orientation",
              "this_is_a_grasp": False, "transmissions": [], "trace": [],
              "limits": {"enable_frames_max": 3, "cartesian_targets_max": 1, "move_timeout_s": MOVE_SECONDS},
              "exit_does_not_cancel_accepted_target": True,
              "reviewed_joint_reentry_requested": args.reviewed_joint_reentry,
              "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    if args.reviewed_joint_reentry:
        report["motion"] = "one fixed reviewed MOVE_J at 5%, nominal-limit reentry and approximately +15 mm end Z"
        report["limits"].update(cartesian_targets_max=0, joint_targets_max=1)
    receiver = None
    state = None
    code = 2
    try:
        report["binding"] = inspect_binding()
        report["controller_processes"] = inspect_controllers()
        vendor = Vendor()
        report["sdk_sources_sha256"] = vendor.sources
        if args.reviewed_joint_reentry:
            # Finish human preparation before reading the start. A disabled
            # wrist can move when a hand/support is removed; never freeze the
            # reviewed live pose first and then ask the human to release it.
            readiness_prompt(True)
        state = Feedback(vendor, reviewed_reentry=args.reviewed_joint_reentry)
        receiver = Receiver(state)
        report["phase"] = "receive_only_preview"
        report["preview"] = collect_stationary(receiver, state, report["trace"])
        target = list(report["preview"]["pose_raw"])
        target[2] += 15000
        if args.reviewed_joint_reentry:
            target = validate_reviewed_start(report["preview"], vendor, require_standby_disabled=True)
            report["motion"] = "one fixed reviewed MOVE_J at 5%, nominal-limit reentry and approximately +15 mm end Z"
            report["limits"]["cartesian_targets_max"] = 0
            report["limits"]["joint_targets_max"] = 1
            report["reviewed_joint_reentry"] = {
                "start_joints_raw": REENTRY_START, "target_joints_raw": REENTRY_TARGET,
                "reference_pose_raw": REENTRY_POSE, "dh_is_offset": 1,
                "start_tolerance_deg": .15, "joint_interval_padding_deg": .15,
                "monitored_end_corridor": {"xy_radius_mm": 3, "relative_z_mm": [-7, 23],
                                           "rotation_distance_deg": 5},
                "source_artifacts_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                           for path in (REENTRY_SOURCE, REENTRY_PROGRESS_SOURCE, REENTRY_OBSERVATION)}}
        report["preview_target_raw"] = target
        if not args.reviewed_joint_reentry:
            print("本次只让右臂当前末端沿基座 +Z 移动 15 mm，5% 速度，不开合夹爪、不回零。")
        print("当前 XYZ(mm), RPY(deg)：", [x / 1000. for x in report["preview"]["pose_raw"]])
        print("唯一目标 XYZ(mm), RPY(deg)：", [x / 1000. for x in target])
        if not args.reviewed_joint_reentry:
            readiness_prompt(False)
        report["phase"] = "receive_only_preflight"
        execute_step(vendor, receiver, state, report)
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        if isinstance(exc, KeyboardInterrupt):
            code = 130
        report.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "rejected_or_failed",
                      error=str(exc) or type(exc).__name__, error_type=type(exc).__name__)
        if report["transmissions"]:
            print("\a本次未确认成功，已停止发送新目标。已接受的目标可能继续；请观察实际机械臂，必要时现场断电并防止下落。", file=sys.stderr)
        print("结果：" + report["error"], file=sys.stderr)
    finally:
        if receiver:
            receiver.close()
        if state:
            report["feedback_frame_counts"] = dict(state.counts)
            report["latched_fault"] = state.fault
            report["latest_unqualified_feedback"] = state.diagnostic(time.monotonic())
            report["initial_unqualified_feedback"] = state.first_complete_diagnostic
        report["finished_at_s"] = time.time()
        report["frames_send_attempted"] = len(report["transmissions"])
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
        print("记录：" + str(report_path))
    if code == 0:
        print("已通过实际反馈确认右臂上移约 15 mm；这次是 SDK 运动验证，尚未抓取方块。")
    return code


if __name__ == "__main__":
    sys.exit(main())
