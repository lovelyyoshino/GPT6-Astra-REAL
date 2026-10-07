"""Bounded manufacturer firmware queries; no mode, enable or target commands."""
import copy
import hashlib
import math
import threading
import time
from pathlib import Path

from . import arms
from .takeover import LIMITS, SIDES, _Takeover


class _FirmwareInspection(_Takeover):
    CONTROL_MODES = (1,)
    REQUIRE_ENABLED = False  # Joint enable required below; gripper may be disabled.
    FRAME_KINDS = ("firmware",)
    SCOPE = "One manufacturer firmware query per arm; no actuator or mode changes"

    def __init__(self, profile, journal_callback):
        super().__init__(profile, journal_callback)
        self.windows = {side: None for side in SIDES}
        self.before_window_frames = dict.fromkeys(SIDES, 0)
        self.gripper_enable = {}
        self.sdk = None
        self.report.update(operation="inspect_firmware", firmware={},
                           mode_commands_sent=0, actuator_commands_sent=0,
                           mode_frame_can_activate_cached_target=False,
                           sdk_profile_changed=False,
                           stationary=True, drift_exceeds_stationary_limits={},
                           state_monitoring_scope="Before and after each blocking query; "
                                                  "not continuous drift sampling during its wait",
                           response_correlation="Single local request window and receive timestamps; "
                                                "protocol has no transaction/fragment sequence number")

    def frame_spec(self, kind, side):
        if kind != "firmware":
            raise RuntimeError("Firmware inspection exposes no actuator or mode command")
        return 0x4AF, b"\x01"  # Audited SDK emits DLC=1, unlike the old SDK's DLC=8.

    def check_enable_state(self, side, state):
        if any(state["drivers"][str(i)]["foc_status"].get("driver_enable_status") is not True
               for i in range(1, 7)):
            raise RuntimeError(side + " firmware inspection requires enabled joints")
        enabled = state["gripper"]["foc_status"].get("driver_enable_status")
        if type(enabled) is not bool:
            raise RuntimeError(side + " gripper enable state unknown")
        if side not in self.gripper_enable:
            self.gripper_enable[side] = enabled
        if enabled is not self.gripper_enable[side]:
            raise RuntimeError(side + " gripper enable state changed during firmware inspection")

    def check_drift(self, side, current, origin):
        # This operation can only ask for firmware. Record drift without
        # interpreting passive telemetry as a reason to suppress the query.
        # The inherited actuator entrypoints retain their rejecting hook.
        drift = {"joint_rad": max(abs(a - b) for a, b in zip(current["joints_rad"], origin["joints_rad"])),
                 "position_m": math.dist(current["pose_m_rad"][:3], origin["pose_m_rad"][:3]),
                 "gripper_m": abs(current["gripper"]["width_m"] - origin["gripper"]["width_m"])}
        for key, value in drift.items():
            self.max_drift[side][key] = max(self.max_drift[side][key], value)
            if value > LIMITS[key]:
                self.report["stationary"] = False
                self.report["drift_exceeds_stationary_limits"].setdefault(side, {})[key] = {
                    "maximum_observed": self.max_drift[side][key], "stationary_limit": LIMITS[key]}

    def _wrap_comm(self, side, comm):
        super()._wrap_comm(side, comm)
        original_callback = comm.get_callback()

        def collect_reply(frame):
            if frame.arbitration_id != 0x4AF:
                return original_callback(frame)
            now = time.time()
            with self.lock:
                window = self.windows[side]
                if window is None or not window["active"]:
                    self.before_window_frames[side] += 1
                    return  # Pre-existing/late firmware traffic is not SDK cache input.
                record = {"timestamp": frame.timestamp, "received_unix_s": now,
                          "dlc": frame.dlc, "payload_hex": bytes(frame.data).hex()}
                fresh = (isinstance(frame.timestamp, (int, float))
                         and not isinstance(frame.timestamp, bool) and math.isfinite(frame.timestamp)
                         and window["request_started_unix_s"] <= frame.timestamp <= now)
                if not fresh:
                    window["ignored_stale_frames"].append(record)
                    return  # Do not let an old queued response contaminate the vendor parser.
                valid = (frame.dlc == 8 and len(frame.data) == 8
                         and not frame.is_extended_id and not frame.is_remote_frame
                         and not frame.is_error_frame and not frame.is_fd
                         and not frame.bitrate_switch and not frame.error_state_indicator
                         and frame.is_rx and bytes(frame.data) != b"\x01" + bytes(7))
                if not valid or len(window["response_frames"]) >= 11:
                    window["rejected_frames"].append(record)
                    self.violations.append({"side": side, "detail": "Unexpected firmware response frame"})
                    return
                window["response_frames"].append(record)
            return original_callback(frame)

        comm.set_callback(collect_reply)

    def _wrap_bus(self, side, comm):
        super()._wrap_bus(side, comm)
        guarded = comm.send_bus.send

        def open_response_window(frame, *args, **kwargs):
            with self.lock:
                if self.windows[side] is None:
                    self.windows[side] = {"active": True, "request_started_unix_s": time.time(),
                                          "response_frames": [], "ignored_stale_frames": [],
                                          "rejected_frames": []}
            return guarded(frame, *args, **kwargs)

        comm.send_bus.send = open_response_window

    def connect(self):
        super().connect()
        self.sdk = arms._load_sdk(self.profile["sdk_path"])
        root = Path(self.sdk.__file__).resolve().parent
        paths = ("protocols/can_protocol/drivers/piper/default/driver.py",
                 "protocols/can_protocol/drivers/piper/default/parser.py",
                 "protocols/can_protocol/drivers/core/table_driven.py")
        self.report["sdk_source"] = {
            "package_path": str(root), "version": self.sdk.__version__,
            "configured_audited_commit": self.profile.get("sdk_commit_audited"),
            "files": [{"path": str(root / path),
                       "sha256": hashlib.sha256((root / path).read_bytes()).hexdigest()} for path in paths]}

    def query_side(self, side):
        self.checked(self.read())
        robot = self.robots[side]
        cached = getattr(robot._parser, "firmware_info", None)
        if cached is not None:
            cached.msg.clear()  # In-memory cache only; never transmit to clear it.
        self.emit("firmware_query_intent", {"side": side, "arbitration_id": 0x4AF,
                                            "dlc": 1, "data_hex": "01"})
        self.checked(self.read())
        # The manufacturer waits for RX inside get_firmware. Unlike immediate
        # commands, do not hold the shared observer lock over this blocking call.
        ticket = {"side": side, "thread": threading.get_ident(), "kind": "firmware",
                  "comm_calls": 0, "bus_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN guard violation before firmware request")
            self.ticket = ticket
        try:
            parsed = robot.get_firmware(timeout=2.0, min_interval=0.0)
            if ticket["error"] is not None:
                raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
            if ticket["comm_calls"] != 1 or ticket["bus_calls"] != 1:
                raise RuntimeError("Expected exactly one manufacturer firmware query")
        finally:
            with self.lock:
                self.ticket = None
        # Observe state and late/extra reply frames before permitting the next
        # query. Successful communication is not permission for robot motion.
        self.observe_window(0.2)
        with self.lock:
            window = self.windows[side]
            window["active"] = False
            window["finished_unix_s"] = time.time()
            evidence = copy.deepcopy(window)
        raw = b"".join(bytes.fromhex(row["payload_hex"]) for row in evidence["response_frames"])
        self.report["firmware"][side] = {"status": "unconfirmed", "manufacturer_result": parsed,
                                         "raw_response_hex": raw.hex(),
                                         "raw_response_text": raw.decode("ascii", errors="replace"),
                                         "response_evidence": evidence}
        if (len(raw) != 88 or not raw.startswith(b"H-V") or evidence["rejected_frames"]
                or not isinstance(parsed, dict)):
            raise RuntimeError(side + " firmware response incomplete or manufacturer parse unavailable")
        # Compare against the manufacturer's documented field layout; do not
        # substitute a homemade version parser or silently use historical data.
        fields = {"hardware_version": (0, 8), "motor_ratio_and_batch": (16, 18),
                  "node_type": (32, 38), "software_version": (60, 68),
                  "production_date": (68, 74), "node_number": (76, 78)}
        if any(parsed.get(key) != raw[start:end].decode("utf-8") for key, (start, end) in fields.items()):
            raise RuntimeError(side + " manufacturer result does not match fresh response bytes")
        suggested = self.sdk.resolve_firmware_profile(self.profile["arms"][side]["model"],
                                                       parsed["software_version"].strip("\x00 "))
        self.report["firmware"][side].update(status="confirmed", suggested_sdk_profile=suggested,
                                             configured_profile=self.profile["arms"][side]["firmware"],
                                             profile_automatically_changed=False)
        self.emit("firmware_received", {"side": side, "result": self.report["firmware"][side]})

    def perform(self):
        for side in SIDES:
            self.query_side(side)
        self.checked(self.read())
        return "firmware_received_no_motion_commands"

    def run(self):
        report = super().run()
        # Retain raw evidence even when SDK throws, times out, or clears its cache.
        for side, window in self.windows.items():
            if window is not None:
                window["active"] = False
                raw = b"".join(bytes.fromhex(row["payload_hex"]) for row in window["response_frames"])
                entry = report["firmware"].setdefault(side, {"status": "unconfirmed"})
                entry.update(raw_response_hex=raw.hex(), raw_response_text=raw.decode("ascii", errors="replace"),
                             response_evidence=copy.deepcopy(window))
        report["pre_or_post_window_firmware_frames"] = dict(self.before_window_frames)
        report["firmware_queries_sent"] = sum(v["firmware"]["sent_frames"] for v in self.kind_counts.values())
        return report


def inspect_firmware(profile, journal_callback):
    """Read firmware using at most one exact SDK query per arm under caller lock.

    Joint drives must be enabled, ctrl_mode=1, and feedback fresh/fault-free.
    Gripper enable state may be false, but must remain unchanged. No mode,
    actuator, target, reset, disable, or stop commands are exposed here.
    Drift is recorded as stationary=false and does not suppress this query.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _FirmwareInspection(profile, journal_callback).run()
