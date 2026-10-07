"""Bounded manufacturer limit queries; never set limits or authorize motion."""
import copy
import math
import threading
import time

from .commissioning import _FirmwareInspection
from .takeover import SIDES, _Takeover


class _JointLimitsInspection(_FirmwareInspection):
    FRAME_KINDS = tuple("joint_%d" % i for i in range(1, 7))
    SCOPE = "One manufacturer angle/velocity limit query per joint per arm; no configuration or actuation"

    def __init__(self, profile, journal_callback):
        super().__init__(profile, journal_callback)
        self.windows = {side: {} for side in SIDES}
        self.active_joint = dict.fromkeys(SIDES)
        self.report.pop("firmware")
        self.report.update(operation="inspect_joint_limits", joint_limits={side: {} for side in SIDES},
                           controller_limits_changed=False, sdk_joint_limits_changed=False,
                           limits_are_motion_permission=False,
                           speed_unit_note="Manufacturer parser scales raw speed by 0.01 rad/s; "
                                           "message comments say 0.001 rad/s. Both raw and SDK values "
                                           "are retained; this query does not authorize a speed.")

    def frame_spec(self, kind, side):
        if kind not in self.FRAME_KINDS:
            raise RuntimeError("Joint limit inspection exposes only fixed query frames")
        return 0x472, bytes((self.FRAME_KINDS.index(kind) + 1, 1, 0, 0, 0, 0, 0, 0))

    def _wrap_comm(self, side, comm):
        _Takeover._wrap_comm(self, side, comm)
        original_callback = comm.get_callback()

        def collect_reply(frame):
            if frame.arbitration_id != 0x473:
                return original_callback(frame)
            now = time.time()
            with self.lock:
                joint = self.active_joint[side]
                window = self.windows[side].get(joint)
                if window is None or not window["active"]:
                    self.before_window_frames[side] += 1
                    return
                record = {"timestamp": frame.timestamp, "received_unix_s": now,
                          "dlc": frame.dlc, "payload_hex": bytes(frame.data).hex()}
                fresh = (isinstance(frame.timestamp, (int, float))
                         and not isinstance(frame.timestamp, bool) and math.isfinite(frame.timestamp)
                         and window["request_started_unix_s"] <= frame.timestamp <= now)
                if not fresh:
                    window["ignored_stale_frames"].append(record)
                    return
                valid = (frame.dlc == 8 and len(frame.data) == 8 and frame.data[0] == joint
                         and not frame.is_extended_id and not frame.is_remote_frame
                         and not frame.is_error_frame and not frame.is_fd
                         and not frame.bitrate_switch and not frame.error_state_indicator
                         and frame.is_rx)
                if not valid or window["response_frames"]:
                    window["rejected_frames"].append(record)
                    self.violations.append({"side": side, "joint_index": joint,
                                            "detail": "Unexpected joint limit response frame"})
                    return
                window["response_frames"].append(record)
            return original_callback(frame)

        comm.set_callback(collect_reply)

    def _wrap_bus(self, side, comm):
        _Takeover._wrap_bus(self, side, comm)
        guarded = comm.send_bus.send

        def open_response_window(frame, *args, **kwargs):
            with self.lock:
                ticket = self.ticket
                if ticket is not None and ticket["kind"] in self.FRAME_KINDS:
                    joint = self.FRAME_KINDS.index(ticket["kind"]) + 1
                    if joint not in self.windows[side]:
                        self.active_joint[side] = joint
                        self.windows[side][joint] = {
                            "active": True, "request_started_unix_s": time.time(),
                            "response_frames": [], "ignored_stale_frames": [], "rejected_frames": []}
            return guarded(frame, *args, **kwargs)

        comm.send_bus.send = open_response_window

    def _retain_evidence(self, side, joint):
        window = self.windows[side][joint]
        entry = self.report["joint_limits"][side].setdefault(str(joint), {"status": "unconfirmed"})
        entry.update(response_evidence=copy.deepcopy(window),
                     raw_response_hex="".join(r["payload_hex"] for r in window["response_frames"]))
        return entry

    def query_joint(self, side, joint):
        self.checked(self.read())
        robot = self.robots[side]
        cached = getattr(robot._parser, "motor_angle_limit_max_spd", None)
        if cached is not None:
            cached.msg.joints[joint - 1].clear()  # In-memory cache only.
        kind = self.FRAME_KINDS[joint - 1]
        can_id, payload = self.frame_spec(kind, side)
        self.emit("joint_limits_query_intent", {"side": side, "joint_index": joint,
                                                "arbitration_id": can_id, "data_hex": payload.hex()})
        self.checked(self.read())
        ticket = {"side": side, "thread": threading.get_ident(), "kind": kind,
                  "comm_calls": 0, "bus_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN guard violation before joint limit query")
            self.ticket = ticket
        try:
            # Do not hold the observer lock: the SDK waits for the RX callback.
            parsed = robot.get_joint_angle_vel_limits(joint_index=joint, timeout=1.0, min_interval=0.0)
            if ticket["error"] is not None:
                raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
            if ticket["comm_calls"] != 1 or ticket["bus_calls"] != 1:
                raise RuntimeError("Expected exactly one manufacturer joint limit query")
        finally:
            with self.lock:
                self.ticket = None
        self.observe_window(0.2)
        with self.lock:
            window = self.windows[side][joint]
            window.update(active=False, finished_unix_s=time.time())
            self.active_joint[side] = None
            entry = self._retain_evidence(side, joint)
        frames = entry["response_evidence"]["response_frames"]
        if len(frames) != 1 or parsed is None or entry["response_evidence"]["rejected_frames"]:
            raise RuntimeError("%s joint %d limit response missing or invalid" % (side, joint))
        raw = bytes.fromhex(frames[0]["payload_hex"])
        result = {name: getattr(parsed.msg, name, None) for name in
                  ("joint_index", "min_angle_limit", "max_angle_limit", "max_joint_spd")}
        entry.update(manufacturer_result=result, manufacturer_timestamp=parsed.timestamp,
                     raw_min_angle_tenth_deg=int.from_bytes(raw[3:5], "big", signed=True),
                     raw_max_angle_tenth_deg=int.from_bytes(raw[1:3], "big", signed=True),
                     raw_max_joint_spd=int.from_bytes(raw[5:7], "big"))
        expected = {"max_angle_limit": entry["raw_max_angle_tenth_deg"] * 0.1 * math.pi / 180,
                    "min_angle_limit": entry["raw_min_angle_tenth_deg"] * 0.1 * math.pi / 180,
                    "max_joint_spd": entry["raw_max_joint_spd"] * 0.01}
        if (type(result["joint_index"]) is not int or result["joint_index"] != joint
                or parsed.timestamp != frames[0]["timestamp"]
                or any(isinstance(result[key], bool) or not isinstance(result[key], (int, float))
                       or not math.isfinite(result[key])
                       or not math.isclose(result[key], value, rel_tol=1e-12, abs_tol=1e-12)
                       for key, value in expected.items())
                or result["min_angle_limit"] > result["max_angle_limit"]):
            raise RuntimeError("Manufacturer joint limits do not match fresh signed response bytes")
        observed = self.report["after"][side]["joints_rad"][joint - 1]
        entry.update(status="confirmed", observed_joint_rad=observed,
                     observed_joint_within_reported_limits=(result["min_angle_limit"] <= observed
                                                           <= result["max_angle_limit"]),
                     motion_permission=False)
        self.emit("joint_limits_received", {"side": side, "joint_index": joint, "result": entry})

    def perform(self):
        for side in SIDES:
            for joint in range(1, 7):
                self.query_joint(side, joint)
        self.checked(self.read())
        return "joint_limits_received_no_motion_commands"

    def run(self):
        report = _Takeover.run(self)
        for side, windows in self.windows.items():
            for joint, window in windows.items():
                window["active"] = False
                self._retain_evidence(side, joint)
        report["pre_or_post_window_limit_frames"] = dict(self.before_window_frames)
        report["joint_limit_queries_sent"] = sum(v[k]["sent_frames"]
                                                  for v in self.kind_counts.values() for k in self.FRAME_KINDS)
        return report


def inspect_joint_limits(profile, journal_callback):
    """Query at most twelve joint limits under the caller's exclusive device lock.

    Reuses firmware inspection's fresh CAN-controlled/enabled/fault-free state
    contract. Drift is recorded, not used to reject a non-actuating query. No
    limits, modes, enables, targets or motion permissions are changed. Failure
    ends the sequence without retries; diagnostic success is not motion safety.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _JointLimitsInspection(profile, journal_callback).run()
