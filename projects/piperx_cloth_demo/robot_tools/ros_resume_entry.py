#!/usr/bin/env python3
"""ROS-only continuation of an already enabled right arm; importing opens no CAN.

Startup receives feedback without transmitting, including no SDK init queries.
The caller still reviews each path. This is not collision or stopping validation.
"""
import copy
import hashlib
import importlib.metadata
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time


DRIVER_SHA256 = "ecff6823dc6bbf55fe51708024ef88d993fead97572ad7c06e951020e83bc200"
SDK_VERSION = "0.6.2"
CAN_NAME, USB_INTERFACE = "can1", "1-6.3:1.0"
FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
AGE_LIMIT_S = 0.1
RAD_PER_RAW = math.pi / 180000
# Manufacturer piper-sdk JointCtrl documentation, degrees converted to raw mdeg.
JOINT_LIMITS_RAW = ((-150000, 150000), (0, 180000), (-170000, 0),
                    (-100000, 100000), (-70000, 70000), (-120000, 120000))


def emit(event, **values):
    print(json.dumps(dict(source="ros_resume_entry", event=event,
                          unix_s=time.time(), **values), allow_nan=False), flush=True)


def integer(value, lo, hi, name):
    if type(value) is not int or not lo <= value <= hi:
        raise ValueError("%s requires integer %d..%d" % (name, lo, hi))
    return value


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Finite numeric command required")
    return value


def binding():
    interface = Path("/sys/class/net") / CAN_NAME
    device = (interface / "device").resolve(strict=True)
    if device.name != USB_INTERFACE or (interface / "type").read_text().strip() != "280":
        raise RuntimeError("Right CAN/USB binding changed")


def quaternion(pose):
    # Same sxyz / Rz(yaw) Ry(pitch) Rx(roll) convention as the vendor ROS node.
    r, p, y = [v / 2 for v in pose[3:]]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def rotation(a, b):
    qa, qb = quaternion(a), quaternion(b)
    norm = math.sqrt(sum(x*x for x in qa) * sum(x*x for x in qb))
    return 2 * math.acos(min(1.0, abs(sum(x*y for x, y in zip(qa, qb))) / norm))


def mode_frame(mode, speed):
    return (0x151, bytes((1, mode, speed, 0, 0, 0, 0, 0)))


def pairs(first_id, values):
    return [(first_id + i, struct.pack(">ii", *values[2*i:2*i+2])) for i in range(3)]


def jaw_frame(width):
    return (0x159, struct.pack(">iHBB", width, 200, 1, 0))


def sdk_class(original):
    class ResumePiper(original):
        def __new__(cls, *args, **kwargs):
            binding()
            if kwargs.get("can_name", args[0] if args else None) != CAN_NAME:
                raise RuntimeError("Resume entry only owns can1")
            with original._lock:
                if original._instances:
                    raise RuntimeError("Existing SDK singleton; a fresh exclusive process is required")
            instance = super().__new__(cls, *args, **kwargs)
            if not isinstance(instance, cls):
                raise RuntimeError("Unreviewed cached SDK instance")
            return instance

        def __init__(self, *args, **kwargs):
            self.rx_lock = threading.RLock()
            self.received = {}
            self.sequence = 0
            self.ticket = None
            self.broken = None
            self.initializing = True
            self.suppressed_initial_modes = 0
            super().__init__(*args, **kwargs)
            comm = self.GetCanBus()
            original_send = comm.send_bus.send

            def guarded_send(frame, *a, **k):
                ticket = self.ticket
                if (self.broken or ticket is None or ticket["thread"] != threading.get_ident()
                        or not ticket["in_comm"] or not ticket["expected"]
                        or frame.is_extended_id or frame.is_remote_frame or frame.is_error_frame
                        or frame.dlc != 8
                        or (frame.arbitration_id, bytes(frame.data)) != ticket["expected"][0]):
                    self.broken = "Unexpected CAN send blocked"
                    raise RuntimeError(self.broken)
                ticket["attempted"] += 1
                try:
                    result = original_send(frame, *a, **k)
                except Exception:
                    self.broken = "CAN send failed; no retry or physical recovery"
                    raise
                ticket["expected"].pop(0)
                ticket["sent"] += 1
                return result

            comm.send_bus.send = guarded_send
            if comm.recv_bus is not comm.send_bus:
                def deny_receive_bus(*a, **k):
                    self.broken = "Receive bus transmission blocked"
                    raise RuntimeError(self.broken)
                comm.recv_bus.send = deny_receive_bus
            original_comm = comm.SendCanMessage

            def guarded_comm(can_id, data, dlc=8, is_extended_id=False):
                ticket = self.ticket
                if (self.broken or ticket is None or ticket["thread"] != threading.get_ident()
                        or ticket["in_comm"] or not ticket["expected"] or dlc != 8 or is_extended_id
                        or (can_id, bytes(data)) != ticket["expected"][0]):
                    self.broken = "Unexpected SDK transmission blocked"
                    raise RuntimeError(self.broken)
                ticket["in_comm"] = True
                try:
                    result = original_comm(can_id, data, dlc, is_extended_id)
                    if result is not comm.CAN_STATUS.SEND_MESSAGE_SUCCESS:
                        self.broken = "SDK send failed; later frames blocked"
                        raise RuntimeError(self.broken)
                    return result
                finally:
                    ticket["in_comm"] = False
            comm.SendCanMessage = guarded_comm

        def ConnectPort(self, can_init=False, piper_init=True, start_thread=True):
            # Guard installed before receive threads start. Never call PiperInit.
            if can_init or not start_thread:
                raise RuntimeError("Only initial receive-thread connection is supported")
            return super().ConnectPort(can_init=False, piper_init=False, start_thread=True)

        def ParseCANFrame(self, message):
            valid = (message is not None and message.arbitration_id in FEEDBACK_IDS
                     and len(message.data) == 8 and not message.is_extended_id
                     and not message.is_remote_frame and not message.is_error_frame
                     and getattr(message, "is_rx", False))
            with self.rx_lock:
                result = super().ParseCANFrame(message)
                if valid:
                    stamp = float(message.timestamp)
                    if not math.isfinite(stamp) or stamp <= 0:
                        self.broken = "Invalid kernel receive timestamp"
                        return result
                    # SocketCAN kernel host timestamp, not processing time. A
                    # delayed SDK callback must not refresh an old queued frame.
                    self.received[message.arbitration_id] = (stamp, bytes(message.data))
                    self.sequence += 1
                return result

        def snapshot(self):
            # Raw receive fragments avoid treating an SDK-filtered old value as new.
            with self.rx_lock:
                frames = dict(self.received)
                sequence = self.sequence
            if set(frames) != set(FEEDBACK_IDS):
                raise RuntimeError("Incomplete fresh feedback")
            stamps = [frames[k][0] for k in FEEDBACK_IDS]
            data = lambda k: frames[k][1]
            raw_q = sum((list(struct.unpack(">ii", data(k))) for k in range(0x2A5, 0x2A8)), [])
            raw_pose = sum((list(struct.unpack(">ii", data(k))) for k in range(0x2A2, 0x2A5)), [])
            status = data(0x2A1)
            jaw = data(0x2A8)
            codes = [data(k)[5] for k in range(0x261, 0x267)]
            return {"sequence": sequence, "stamps": stamps, "raw_q": raw_q,
                    "q": [v*RAD_PER_RAW for v in raw_q],
                    "pose": [v/1e6 for v in raw_pose[:3]] + [v*RAD_PER_RAW for v in raw_pose[3:]],
                    "opening_m": struct.unpack(">i", jaw[:4])[0]/1e6,
                    "gripper_torque_sdk_units": struct.unpack(">h", jaw[4:6])[0],
                    "ctrl_mode": status[0], "arm_status": status[1], "mode": status[2],
                    "teach_status": status[3], "motion_status": status[4],
                    "fault": int.from_bytes(status[6:8], "big"),
                    "driver_codes": codes, "jaw_code": jaw[6],
                    "enabled": [bool(v & 64) for v in codes]}

        def healthy(self, snapshot, allow_moving=False):
            if self.broken:
                raise RuntimeError(self.broken)
            now = time.time()
            stamps = snapshot["stamps"]
            if max(stamps) > now or now-min(stamps) > AGE_LIMIT_S or max(stamps)-min(stamps) > AGE_LIMIT_S:
                raise RuntimeError("Feedback age/skew exceeds 100ms")
            if (snapshot["ctrl_mode"] != 1 or snapshot["arm_status"] != 0
                    or snapshot["fault"] != 0 or snapshot["teach_status"] != 0
                    or snapshot["mode"] not in (0, 1, 2)
                    or snapshot["motion_status"] not in ((0, 1) if allow_moving else (0,))
                    or any(code != 64 for code in snapshot["driver_codes"])
                    or snapshot["jaw_code"] != 64):
                raise RuntimeError("Requires healthy enabled CAN arm and jaw, no teaching/fault")
            if any(not lo <= q <= hi for q, (lo, hi) in zip(snapshot["raw_q"], JOINT_LIMITS_RAW)):
                raise RuntimeError("Measured joints outside manufacturer nominal limits")
            if (not 0 <= snapshot["opening_m"] <= 0.07
                    or any(abs(x) > 1 for x in snapshot["pose"][:3])
                    or any(abs(x) > 2*math.pi for x in snapshot["pose"][3:])):
                raise RuntimeError("Invalid jaw/flange feedback range")
            return snapshot

        def MotionCtrl_2(self, ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=50,
                         is_mit_mode=0, residence_time=0, installation_pos=0):
            values = (ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode, residence_time, installation_pos)
            if self.initializing:
                if values != (1, 1, 30, 0, 0, 0) or self.suppressed_initial_modes:
                    raise RuntimeError("Unexpected initialization mode request")
                self.suppressed_initial_modes += 1
                emit("initial_j_mode_suppressed_zero_tx", requested=list(values))
                return
            if (ctrl_mode != 1 or move_mode not in (0, 1) or is_mit_mode != 0
                    or residence_time != 0 or installation_pos != 0 or self.ticket is None):
                raise RuntimeError("Only an explicit ordinary P/J transaction may set mode")
            return super().MotionCtrl_2(1, move_mode, self.ticket["speed"], 0, 0, 0)

        def GripperCtrl(self, gripper_angle=0, gripper_effort=0, gripper_code=0, set_zero=0):
            integer(gripper_angle, 0, 55000, "jaw width")
            if gripper_code != 1 or set_zero != 0 or self.ticket is None:
                raise RuntimeError("Only explicit jaw position with enable1/setzero0 is supported")
            return super().GripperCtrl(gripper_angle, 200, 1, 0)

        def MotionCtrl_1(self, emergency_stop=0, track_ctrl=0, grag_teach_ctrl=0):
            if (emergency_stop, track_ctrl, grag_teach_ctrl) != (0, 0, 0) or self.ticket is None:
                raise RuntimeError("Stop/reset/teaching/replay are forbidden")
            return super().MotionCtrl_1(0, 0, 0)

        def EnableArm(self, *a, **k):
            raise RuntimeError("Read-only adoption never enables hardware")

        def DisableArm(self, *a, **k):
            raise RuntimeError("Hardware disable is forbidden")

    return ResumePiper


def stable(samples):
    # Called only after appending one sample to an already-validated window.
    # Existing pairs passed previously; compare the new sample to every old one.
    for key, count, limit in (("q", 6, 0.003), ("pose", 3, 0.0005)):
        if any(max(s[key][i] for s in samples)-min(s[key][i] for s in samples) > limit for i in range(count)):
            return False
    if max(s["opening_m"] for s in samples)-min(s["opening_m"] for s in samples) > 0.0005:
        return False
    return all(rotation(samples[-1]["pose"], s["pose"]) <= 0.003 for s in samples[:-1])


def node_class(vendor, rospy):
    class ResumeNode(vendor.C_PiperRosNode):
        def __init__(self):
            self.action_lock = threading.Lock()
            self.adopted = False
            self.failed = None
            self.active = False
            self.command_sequence = 0
            super().__init__()
            if self.auto_enable or self.exit_teaching_mode or not self.gripper_exist or self.gripper_val_mutiple != 1:
                raise RuntimeError("Require auto_enable=false, exit_teaching=false, gripper_exist=true, multiplier1")
            if self.piper.suppressed_initial_modes != 1:
                raise RuntimeError("Expected exactly one suppressed constructor mode request")
            self.piper.initializing = False
            initial = self.wait_stable(3.0, 30.0, warmup=True)
            self._C_PiperRosNode__enable_flag = True  # Software only; never enable_callback.
            self.adopted = True
            emit("read_only_adoption_complete", initial=initial, actuator_frames=0, sdk_init_queries=0)

        def wait_stable(self, duration, timeout, after=0.0, goal=None, mode=None, warmup=False):
            deadline = time.monotonic()+timeout
            samples, since, last_stamp = [], None, None
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    s = self.piper.snapshot()
                except RuntimeError:
                    if warmup and not self.piper.received:
                        time.sleep(0.01)
                        continue
                    # The initial fragments may arrive in separate CAN periods.
                    if warmup and set(self.piper.received) != set(FEEDBACK_IDS):
                        time.sleep(0.01)
                        continue
                    raise
                self.piper.healthy(s, allow_moving=after > 0)
                ready = min(s["stamps"]) > after and s["motion_status"] == 0
                ready = ready and (mode is None or s["mode"] == mode) and (goal is None or goal(s))
                if not ready:
                    samples, since, last_stamp = [], None, None
                elif last_stamp is None or all(a > b for a, b in zip(s["stamps"], last_stamp)):
                    if samples and s["mode"] != samples[0]["mode"]:
                        raise RuntimeError("Mode changed inside stationary window")
                    samples.append(s)
                    if not stable(samples):
                        samples = [s]
                        since = None
                    if since is None:
                        since = time.monotonic()
                    last_stamp = s["stamps"]
                    if time.monotonic()-since >= duration and len(samples) >= (20 if duration >= 3 else 3):
                        self.piper.healthy(s)  # Include stability-computation delay in receive age.
                        return s
                time.sleep(0.01)
            raise RuntimeError("Stable fresh arrival not established; no automatic recovery")

        def run_action(self, kind, prepare, call):
            if not self.action_lock.acquire(False):
                raise RuntimeError("Another command is active; concurrent request rejected")
            dispatched = False
            try:
                if not self.adopted or self.failed or rospy.is_shutdown():
                    raise RuntimeError("Driver not adopted, failed, or shutting down")
                self.active = True
                binding()
                speed = integer(rospy.get_param("~speed_percent", 1), 1, 50, "~speed_percent")
                frames, goal, mode = prepare(speed)
                before = self.wait_stable(0.3, 5.0)
                self.command_sequence += 1
                ticket = dict(thread=threading.get_ident(), expected=list(frames), speed=speed,
                              in_comm=False, attempted=0, sent=0)
                emit("command_intent", sequence=self.command_sequence, kind=kind, speed_percent=speed,
                     frames=[{"id": k, "data_hex": v.hex()} for k, v in frames], before=before)
                self.piper.healthy(before)  # Logging must not make the dispatch snapshot stale.
                self.piper.ticket = ticket
                try:
                    dispatched = True
                    response = call()
                    if self.piper.broken or ticket["expected"] or ticket["sent"] != len(frames):
                        raise RuntimeError(self.piper.broken or "Incomplete SDK transaction")
                finally:
                    self.piper.ticket = None
                finished = time.time()
                emit("command_sent_unconfirmed", sequence=self.command_sequence,
                     attempted_frames=ticket["attempted"], socket_send_returns=ticket["sent"])
                if kind == "gripper":
                    goal = lambda s: (max(abs(a-b) for a, b in zip(s["q"], before["q"])) <= 0.003
                                      and max(abs(a-b) for a, b in zip(s["pose"][:3], before["pose"][:3])) <= 0.002
                                      and rotation(s["pose"], before["pose"]) <= 0.003)
                    mode = before["mode"]
                after = self.wait_stable(0.3, 120.0, after=finished, goal=goal, mode=mode)
                jaw_targets = [struct.unpack(">i", data[:4])[0]/1e6 for can_id, data in frames if can_id == 0x159]
                jaw_error = after["opening_m"]-jaw_targets[0] if jaw_targets else None
                emit("command_observed_stable", sequence=self.command_sequence, kind=kind, after=after,
                     arm_target_reached=kind in ("pose", "joint"),
                     jaw_target_m=jaw_targets[0] if jaw_targets else None,
                     jaw_width_error_m=jaw_error,
                     jaw_target_reached=abs(jaw_error) <= 0.0015 if jaw_error is not None else None,
                     jaw_target_report_tolerance_m=0.0015,
                     gripper_service_status_scope="request_sent_and_feedback_stable_not_grasp_or_release",
                     grasp_verified=False, general_stop_validated=False)
                return response
            except Exception as error:
                if dispatched or (hasattr(self, "piper") and self.piper.broken):
                    self.failed = str(error)
                    self._C_PiperRosNode__enable_flag = False
                emit("command_refused_or_failed", kind=kind, dispatched=dispatched,
                     error=str(error), further_commands_blocked=bool(self.failed), physical_recovery_sent=False)
                raise
            finally:
                if hasattr(self, "piper"):
                    self.piper.ticket = None
                self.active = False
                self.action_lock.release()

        def pos_callback(self, msg):
            def prepare(speed):
                values = [finite(getattr(msg, k)) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
                if any(abs(x) > 1 for x in values[:3]) or any(abs(x) > 2*math.pi for x in values[3:]):
                    raise ValueError("Cartesian input outside vendor encoding envelope")
                width_m = finite(msg.gripper)
                if not 0 <= width_m <= 0.055:
                    raise ValueError("Jaw width must be0..55mm")
                width = round(width_m*1000*1000)
                integer(width, 0, 55000, "jaw width")
                factor = 180 / 3.1415926
                raw = [round(v*1000)*1000 for v in values[:3]] + [round(v*1000*factor) for v in values[3:]]
                target = [v/1e6 for v in raw[:3]] + [v*RAD_PER_RAW for v in raw[3:]]
                goal = lambda s: (max(abs(a-b) for a, b in zip(s["pose"][:3], target[:3])) <= 0.002
                                  and rotation(s["pose"], target) <= 0.02)
                return [(0x150, bytes(8)), mode_frame(0, speed)] + pairs(0x152, raw) + [jaw_frame(width), mode_frame(0, speed)], goal, 0
            return self.run_action("pose", prepare, lambda: super(ResumeNode, self).pos_callback(msg))

        def joint_callback(self, msg):
            if len(msg.position) != 6:
                raise ValueError("Exactly six joint positions required; use separate gripper_srv")
            msg = copy.deepcopy(msg)
            def prepare(speed):
                factor = 1000 * 180 / math.pi
                raw = [round(finite(v)*factor) for v in msg.position]
                if any(not lo <= v <= hi for v, (lo, hi) in zip(raw, JOINT_LIMITS_RAW)):
                    raise ValueError("Joint target outside manufacturer nominal limits")
                msg.velocity = [0.0]*6 + [float(speed)]
                target = [v*RAD_PER_RAW for v in raw]
                return [mode_frame(1, speed)] + pairs(0x155, raw), lambda s: max(abs(a-b) for a, b in zip(s["q"], target)) <= 0.003, 1
            return self.run_action("joint", prepare, lambda: super(ResumeNode, self).joint_callback(msg))

        def handle_gripper_service(self, req):
            def prepare(speed):
                width_m = finite(req.gripper_angle)
                if not 0 <= width_m <= 0.055:
                    raise ValueError("Jaw width must be0..55mm")
                width = round(width_m*1e6)
                integer(width, 0, 55000, "jaw width")
                if req.gripper_code != 1 or req.set_zero != 0 or finite(req.gripper_effort) != 0.2:
                    raise ValueError("Gripper requires nominal effort.2/code1/setzero0")
                return [jaw_frame(width)], None, None
            return self.run_action("gripper", prepare, lambda: super(ResumeNode, self).handle_gripper_service(req))

        def denied(self, *a, **k):
            raise RuntimeError("Enable/disable/stop/reset/teaching/go-zero/block services are outside resume scope")

        enable_callback = denied
        handle_enable_service = denied
        handle_stop_service = denied
        handle_reset_service = denied
        handle_go_zero_service = denied
        handle_block_arm_service = denied

    return ResumeNode


def main():
    if len(sys.argv) < 2:
        raise SystemExit("Usage: ros_resume_entry.py PINNED_VENDOR_DRIVER.py [ROS remaps]")
    path = Path(sys.argv[1]).resolve(strict=True)
    if hashlib.sha256(path.read_bytes()).hexdigest() != DRIVER_SHA256:
        raise SystemExit("Vendor driver hash mismatch")
    if importlib.metadata.version("piper-sdk") != SDK_VERSION:
        raise SystemExit("Requires pinned piper-sdk0.6.2")
    import rospy
    import rosgraph
    from std_msgs.msg import String
    import piper_sdk
    # Run before vendor init_node: registering the same ROS name must not silently
    # evict an old driver. Root also verifies the old OS process has exited.
    master = rosgraph.Master("/piper_resume_preflight")
    pubs, subs, services = master.getSystemState()
    live = {node for _, nodes in pubs+subs+services for node in nodes}
    for param in master.getParamNames():
        if (param.endswith("/can_port") and param.rsplit("/", 1)[0] in live
                and master.getParam(param) == CAN_NAME):
            raise RuntimeError("Existing live ROS owner of can1; exit it before resume")
    original = piper_sdk.C_PiperInterface
    # Vendor init_node occurs once inside its constructor. Check private params
    # and existing live owners after that, before any SDK instance opens sockets.
    resumed_sdk = sdk_class(original)
    original_init = resumed_sdk.__init__
    def checked_init(self, *args, **kwargs):
        if rospy.get_param("~can_port", None) != CAN_NAME:
            raise RuntimeError("Private can_port must explicitly be can1")
        for name in ("~auto_enable", "~exit_teaching_mode"):
            if rospy.get_param(name, False) is not False:
                raise RuntimeError(name+" must be boolean false")
        integer(rospy.get_param("~speed_percent", 1), 1, 50, "~speed_percent")
        own = rospy.get_name()
        pubs, subs, services = rosgraph.Master(own).getSystemState()
        live = {node for _, nodes in pubs+subs+services for node in nodes}
        for param in rospy.get_param_names():
            owner = param.rsplit("/", 1)[0]
            if param.endswith("/can_port") and owner != own and owner in live and rospy.get_param(param) == CAN_NAME:
                raise RuntimeError("Another live ROS driver owns can1: "+owner)
        return original_init(self, *args, **kwargs)
    resumed_sdk.__init__ = checked_init
    sys.argv = sys.argv[1:]
    spec = importlib.util.spec_from_file_location("pinned_resumed_piper_driver", str(path))
    vendor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vendor)
    vendor.C_PiperInterface = resumed_sdk
    node = node_class(vendor, rospy)()
    publisher = rospy.Publisher("eval_telemetry", String, queue_size=1)
    def telemetry(_):
        try:
            s = node.piper.snapshot()
            s.update(can_interface=CAN_NAME, stamp=time.time(), source="sdk_receive_raw_frames",
                     source_sequence=s["sequence"], feedback_max_age_s=time.time()-min(s["stamps"]),
                     driver_accepts_commands=node.adopted and not node.failed and not node.piper.broken and not node.active,
                     active_command=node.active, failure=node.failed or node.piper.broken,
                     mode_feedback=s["mode"], driver_sha256=DRIVER_SHA256, sdk_version=SDK_VERSION)
            publisher.publish(String(data=json.dumps(s, allow_nan=False)))
        except Exception as error:
            rospy.logwarn_throttle(2, "Resume telemetry unavailable: %s", str(error))
    timer = rospy.Timer(rospy.Duration(0.02), telemetry)
    try:
        node.Pubilsh()
    finally:
        node.adopted = False
        timer.shutdown()
        # A shutdown is not a physical stop. Do not close CAN mid-transaction;
        # wait_stable exits on ROS shutdown, then releases this same action lock.
        with node.action_lock:
            node.piper.ticket = None
            node.piper.DisconnectPort()
        # Socket/thread cleanup only; no physical stop/reset/disable or retry.


if __name__ == "__main__":
    main()
