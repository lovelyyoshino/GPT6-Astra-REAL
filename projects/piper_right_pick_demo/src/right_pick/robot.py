"""Read-only Piper ROS observations and environment diagnostics.

Independent implementation using public message contracts. No manufacturer SDK,
ROS module, control publisher or service proxy is created during import.
"""
import importlib.metadata
import http.client
import math
import os
import platform
import socket
import threading
import time
import urllib.parse
import xmlrpc.client


class RobotUnavailable(RuntimeError):
    pass


class MotionUnconfigured(RobotUnavailable):
    pass


PROVENANCE = {
    "piper_sdk": {
        "inspected_version": "0.6.2",
        "source": "https://github.com/agilexrobotics/piper_sdk",
        "contracts": ["GetArmJointMsgs", "GetArmEndPoseMsgs", "GetArmStatus"],
    },
    "piper_ros": {
        "source": "https://github.com/agilexrobotics/piper_ros/tree/noetic",
        "installed_revision": None,
        "message_contracts": {
            "joint_states": "sensor_msgs/JointState",
            "end_pose": "geometry_msgs/PoseStamped",
            "status": "piper_msgs/PiperStatusMsg",
        },
    },
}


def inspect_environment():
    """Create and close sockets only; never bind CAN or instantiate the SDK."""
    versions = {}
    for package in ("piper_sdk", "python-can", "numpy", "pyrealsense2"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    probes = {}
    families = [("tcp_socket", socket.AF_INET, socket.SOCK_STREAM, 0)]
    if hasattr(socket, "AF_CAN"):
        families.append(("can_socket", socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW))
    for label, family, kind, protocol in families:
        try:
            sock = socket.socket(family, kind, protocol)
            sock.close()
            probes[label] = {"available": True, "connected": False}
        except OSError as exc:
            probes[label] = {"available": False, "errno": exc.errno,
                             "reason": str(exc), "connected": False}
    return {
        "python": platform.python_version(), "platform": platform.platform(),
        "machine": platform.machine(), "versions": versions,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "socket_probes": probes, "source_provenance": PROVENANCE,
        "hardware_connected": False, "can_frames_sent": 0,
        "control_commands_sent": 0,
        "note": "Socket creation availability does not verify a device connection.",
    }


def check_ros_master(timeout_s=2.0):
    """Bounded, read-only getPid request before rospy registration can wait."""
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise RobotUnavailable("ROS master timeout must be finite and positive")
    uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    parsed = urllib.parse.urlsplit(uri)
    if parsed.scheme != "http" or not parsed.hostname or parsed.username or parsed.password:
        raise RobotUnavailable("ROS_MASTER_URI must be an HTTP ROS master URI without credentials")

    class Transport(xmlrpc.client.Transport):
        def make_connection(self, host):
            return http.client.HTTPConnection(host, timeout=timeout_s)

    try:
        with xmlrpc.client.ServerProxy(uri, transport=Transport()) as master:
            response = master.getPid("/right_pick_readonly_probe")
        if not isinstance(response, (list, tuple)) or len(response) != 3 or response[0] != 1:
            raise RobotUnavailable("ROS master getPid did not succeed")
    except (OSError, xmlrpc.client.Error, ValueError) as exc:
        raise RobotUnavailable("ROS master read-only probe failed: " + str(exc)) from exc
    return {"connected": True, "operation": "getPid", "control_commands_sent": 0}


def _finite(values, label):
    result = [float(x) for x in values]
    if not all(math.isfinite(x) for x in result):
        raise RobotUnavailable(label + " contains non-finite numbers")
    return result


def _stamp(message):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    return float(stamp.to_sec()) if stamp is not None else None


def decode_joint_state(message):
    names = list(message.name)
    positions = _finite(message.position, "joint position")
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise RobotUnavailable("JointState names/positions are inconsistent")
    joints = dict(zip(names, positions))
    if any("joint%d" % i not in joints for i in range(1, 7)):
        raise RobotUnavailable("Piper joint1 through joint6 feedback is incomplete")
    return {"joint_names": ["joint%d" % i for i in range(1, 7)],
            "joint_positions_rad": [joints["joint%d" % i] for i in range(1, 7)],
            "gripper_driver_position_m": joints.get("gripper"),
            "gripper_total_opening_m": None,
            "gripper_mapping_verified": False,
            "velocity_driver_values": _finite(message.velocity, "joint velocity"),
            "effort_driver_values": _finite(message.effort, "joint effort"),
            "source_stamp_s": _stamp(message)}


def decode_end_pose(message):
    p, q = message.pose.position, message.pose.orientation
    position = _finite([p.x, p.y, p.z], "end pose")
    quat = _finite([q.w, q.x, q.y, q.z], "end quaternion")
    if abs(sum(x * x for x in quat) - 1) > 0.02:
        raise RobotUnavailable("End-pose quaternion is not normalized")
    return {"position_m": position, "quaternion_wxyz": quat,
            "frame_id": getattr(message.header, "frame_id", ""),
            "reference": "driver_end_reference_not_verified_as_gripper_tcp",
            "source_stamp_s": _stamp(message)}


def decode_status(message):
    numeric = ("ctrl_mode", "arm_status", "mode_feedback", "teach_status",
               "motion_status", "trajectory_num", "err_code")
    flags = ["joint_%d_angle_limit" % i for i in range(1, 7)]
    flags += ["communication_status_joint_%d" % i for i in range(1, 7)]
    required = list(numeric) + flags
    missing = [key for key in required if not hasattr(message, key)]
    if missing:
        raise RobotUnavailable("PiperStatusMsg missing fields: " + ", ".join(missing))
    result = {key: int(getattr(message, key)) for key in numeric}
    result.update({key: bool(getattr(message, key)) for key in flags})
    result["fault_reported"] = bool(result["err_code"] or result["arm_status"]
                                     or any(result[key] for key in flags))
    # CAN control mode is not a motor-enable measurement.
    result["enabled"] = None
    result["enabled_reason"] = "This ROS status message has no motor-enable feedback."
    result["source_stamp_s"] = _stamp(message)
    return result


def validate_freshness(samples, now_s, max_age_s, max_skew_s):
    stamps = []
    for name, sample in samples.items():
        received = sample["received_at_s"]
        if now_s - received > max_age_s or received > now_s + 0.1:
            raise RobotUnavailable("Stale or future receipt: " + name)
        stamp = sample["data"].get("source_stamp_s")
        if stamp is not None:
            if not math.isfinite(stamp) or stamp <= 0 or now_s - stamp > max_age_s or stamp > now_s + 0.1:
                raise RobotUnavailable("Stale, invalid or future ROS stamp: " + name)
            stamps.append(stamp)
    if len(stamps) > 1 and max(stamps) - min(stamps) > max_skew_s:
        raise RobotUnavailable("Joint and end-pose timestamps exceed configured skew")
    receipts = [sample["received_at_s"] for sample in samples.values()]
    if receipts and max(receipts) - min(receipts) > max_skew_s:
        raise RobotUnavailable("Robot feedback receipt times exceed configured skew")


class RosRightArm:
    """Subscribes only. Diagnostics do not grant permission to move the arm."""
    def __init__(self, config):
        self.config = dict(config)
        if self.config.get("arm") != "right":
            raise RobotUnavailable("This adapter requires arm='right'")
        self.topics = dict(self.config.get("topics", {}))
        for key in ("joint_states", "end_pose", "status"):
            topic = self.topics.get(key)
            if not isinstance(topic, str) or not topic.startswith("/") or topic == "/":
                raise RobotUnavailable("Explicit absolute robot topic required: " + key)
        if len(set(self.topics[key] for key in ("joint_states", "end_pose", "status"))) != 3:
            raise RobotUnavailable("Robot state topics must be distinct")
        for key, default in (("max_age_s", 1.0), ("max_skew_s", 0.2)):
            value = float(self.config.get(key, default))
            if not math.isfinite(value) or value <= 0:
                raise RobotUnavailable(key + " must be finite and positive")
            self.config[key] = value

    def observe(self, timeout_s=3.0):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise RobotUnavailable("timeout_s must be finite and positive")
        check_ros_master(min(timeout_s, 2.0))
        try:
            import rospy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import JointState
            from piper_msgs.msg import PiperStatusMsg
        except ImportError as exc:
            raise RobotUnavailable("Source the installed ROS Noetic and Piper message environment: " + str(exc)) from exc
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("right_pick_observer", anonymous=True, disable_signals=True)
            if rospy.get_param("/use_sim_time", False):
                raise RobotUnavailable("Real robot observations require wall-clock ROS time")
        except Exception as exc:
            raise RobotUnavailable("ROS connection failed: " + str(exc)) from exc
        latest, errors, subscriptions = {}, {}, []
        lock = threading.Lock()

        def callback(name, decoder):
            def receive(message):
                received = time.time()
                try:
                    data = decoder(message)
                    with lock:
                        latest[name] = {"received_at_s": received, "data": data}
                        errors.pop(name, None)
                except (ValueError, TypeError, AttributeError, RobotUnavailable) as exc:
                    with lock:
                        errors[name] = str(exc)
                        latest.pop(name, None)
            return receive

        try:
            for key, msg, decoder in (("joint_states", JointState, decode_joint_state),
                                      ("end_pose", PoseStamped, decode_end_pose),
                                      ("status", PiperStatusMsg, decode_status)):
                subscriptions.append(rospy.Subscriber(self.topics[key], msg,
                    callback(key, decoder), queue_size=1))
            deadline = time.monotonic() + timeout_s
            last_reason = "Waiting for all three feedback topics"
            samples = {}
            while time.monotonic() < deadline:
                with lock:
                    samples = dict(latest)
                    observed_errors = dict(errors)
                if len(samples) == 3:
                    try:
                        validate_freshness(samples, time.time(), self.config["max_age_s"], self.config["max_skew_s"])
                        return {"backend": "ros1", "arm": "right", "observed_at_s": time.time(),
                            "topics": self.topics, "samples": samples,
                            "right_binding_verified": bool(self.config.get("namespace_verified", False)),
                            "device_freshness_verified": False,
                            "freshness_note": "Driver stamps may date publication of cached CAN feedback; status has no hardware timestamp.",
                            "motion_ready": False, "enabled": None,
                            "fault_reported": samples["status"]["data"]["fault_reported"],
                            "tcp_verified": False}
                    except RobotUnavailable as exc:
                        last_reason = str(exc)
                elif observed_errors:
                    last_reason = str(observed_errors)
                if rospy.is_shutdown():
                    raise RobotUnavailable("ROS shut down while waiting for observations")
                time.sleep(0.01)
            missing = sorted(set(("joint_states", "end_pose", "status")) - set(samples))
            raise RobotUnavailable("ROS observation timed out; missing=%s; %s" % (missing, last_reason))
        finally:
            for subscriber in subscriptions:
                subscriber.unregister()

    def move(self, action):
        raise MotionUnconfigured("Motion dispatch is not configured: current camera extrinsics, measured TCP, gripper mapping, hardware feedback freshness and a validated stop strategy are required.")

    def stop(self):
        raise MotionUnconfigured("No verified physical stop adapter is configured; ending a decision loop does not stop a physical arm.")
