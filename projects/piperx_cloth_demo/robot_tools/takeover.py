"""Bounded CAN takeover and separate standby startup; no target/stop fallback.

This is a separately authorized commissioning operation, not a motion-gate
bypass. A mode frame can activate an old controller target; absence of target
frames does NOT prove absence of motion. The caller must hold the platform's
exclusive device lock and persist journal_callback(event, data) synchronously.

SDK coupling is deliberately narrow: its private _msg_mode cache supplies
speed=1 without the public speed setter's extra move_mode=255 transmission.
Only the public set_motion_mode performs encoding/transmission. The final bus
guard checks the exact eight bytes, so changed SDK encoding fails closed.
"""
import copy
import math
import threading
import time

from . import arms

SIDES = ("left", "right")
LIMITS = {"warmup_s": 3.0, "stable_s": 1.0, "after_send_s": 2.0,
          "failure_observe_s": 0.5, "poll_s": 0.05,
          "joint_rad": 0.003, "position_m": 0.002, "gripper_m": 0.002}
# Known motion/configuration command IDs, not normal feedback IDs. Listening
# cannot prove exclusive ownership: another sender can suppress local loopback.
COMMAND_IDS = frozenset(range(0x150, 0x160)) | frozenset(range(0x181, 0x187)) | {
    0x191, 0x470, 0x471, 0x474, 0x475, 0x479, 0x47A}


def _allowed_integer(value, allowed):
    """Accept SDK IntEnum and ordinary int; never bool/float or unknown values."""
    return isinstance(value, int) and not isinstance(value, bool) and value in allowed


class _Takeover:
    CONTROL_MODES = (1, 2)
    REQUIRE_ENABLED = True
    FRAME_KINDS = ("mode",)
    SCOPE = "CAN control-mode request only"

    def __init__(self, profile, journal_callback):
        self.profile = copy.deepcopy(profile)
        self.journal = journal_callback
        self.robots, self.grippers, self.comms, self.buses = {}, {}, {}, {}
        self.lock = threading.RLock()
        self.ticket = None
        self.violations = []
        self.anchor = None
        self.expected_modes = {}
        self.required_ctrl = {}
        self.pending_side = None
        self.pending_sent_at = None
        self.max_drift = {side: {"joint_rad": 0.0, "position_m": 0.0, "gripper_m": 0.0}
                          for side in SIDES}
        self.counts = {side: {"attempted_frames": 0, "sent_frames": 0,
                              "blocked_frames": 0} for side in SIDES}
        self.kind_counts = {side: {kind: {"attempted_frames": 0, "sent_frames": 0}
                                  for kind in self.FRAME_KINDS} for side in SIDES}
        self.report = {"ok": False, "status": "refused_before_send", "before": None,
                       "after": None, "errors": [], "arms": {}, "samples": 0,
                       "motion_gate_unlocked": False, "hold_not_validated": True,
                       "target_commands_sent": 0, "enable_commands_sent": 0,
                       "stop_commands_sent": 0, "retries": 0,
                       "mode_frame_can_activate_cached_target": True,
                       "external_sender_detection": "Known command frames visible on SDK RX only; "
                                                    "silence does not prove bus ownership",
                       "limits": dict(LIMITS)}

    def emit(self, event, data):
        self.journal(event, copy.deepcopy(data))

    def _deny(self, side, detail):
        with self.lock:
            self.counts[side]["blocked_frames"] += 1
            self.violations.append({"side": side, "detail": detail})
        raise RuntimeError(detail)

    def _wrap_comm(self, side, comm):
        original_send = comm.send
        original_callback = comm.get_callback()

        def guarded_send(*args, **kwargs):
            with self.lock:
                ticket = self.ticket
                if (ticket is None or ticket["side"] != side
                        or ticket["thread"] != threading.get_ident()
                        or comm.send_bus is not self.buses.get(side)):
                    self._deny(side, "Unexpected SDK TX outside one authorized mode request")
                ticket["comm_calls"] += 1
                if ticket["comm_calls"] != 1:
                    self._deny(side, "More than one SDK send in a mode request")
                result = original_send(*args, **kwargs)
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["bus_calls"] != 1:
                    raise RuntimeError("Expected exactly one underlying CAN frame")
                return result

        def observed_callback(frame):
            if frame.arbitration_id in COMMAND_IDS:
                with self.lock:
                    self.violations.append({"side": side, "detail": "Other command frame observed",
                                            "arbitration_id": frame.arbitration_id,
                                            "data_hex": bytes(frame.data).hex(),
                                            "timestamp": getattr(frame, "timestamp", None)})
            if original_callback is not None:
                original_callback(frame)

        comm.send = guarded_send
        comm.set_callback(observed_callback)

    def _wrap_bus(self, side, comm):
        bus, original_send = comm.send_bus, comm.send_bus.send
        self.buses[side] = bus

        def guarded_bus_send(frame, *args, **kwargs):
            with self.lock:
                ticket = self.ticket
                if (ticket is None or ticket["side"] != side
                        or ticket["thread"] != threading.get_ident()
                        or comm.send_bus is not bus or self.violations):
                    self._deny(side, "Underlying CAN TX outside guarded mode request")
                can_id, expected = self.frame_spec(ticket["kind"], side)
                kind_count = self.kind_counts[side][ticket["kind"]]
                exact = (frame.arbitration_id == can_id and frame.dlc == len(expected)
                         and not frame.is_extended_id and not frame.is_remote_frame
                         and not frame.is_error_frame and not frame.is_fd
                         and not frame.bitrate_switch and not frame.error_state_indicator
                         and bytes(frame.data) == expected)
                if not exact or ticket["bus_calls"] != 0 or kind_count["attempted_frames"]:
                    self._deny(side, "Frame rejected: only one exact standard 0x%03X %s frame allowed" %
                               (can_id, ticket["kind"]))
                ticket["bus_calls"] += 1
                self.counts[side]["attempted_frames"] += 1
                kind_count["attempted_frames"] += 1
                try:
                    result = original_send(frame, *args, **kwargs)
                except BaseException as exc:
                    ticket["error"] = exc
                    raise
                self.counts[side]["sent_frames"] += 1
                kind_count["sent_frames"] += 1
                return result

        bus.send = guarded_bus_send

    def connect(self):
        configs = self.profile["arms"]
        arms._validate_arms(configs)
        if set(configs) != set(SIDES):
            raise ValueError("Mode takeover requires both explicitly bound arms")
        arms._preflight(configs)
        sdk = arms._load_sdk(self.profile["sdk_path"])
        for side in SIDES:
            cfg = configs[side]
            config = sdk.create_agx_arm_config(
                robot=cfg["model"], firmeware_version=cfg["firmware"],
                channel=cfg["channel"], interface="socketcan", auto_connect=False,
                enable_check_can=False, receive_own_messages=False, local_loopback=False)
            robot = sdk.AgxArmFactory.create_arm(config)
            self.robots[side] = robot
            # Audited construction is passive. Deny initialization sends before
            # registering the effector, creating comm, or starting SDK threads.
            original_send, original_sends = robot._send_msg, robot._send_msgs
            robot._send_msg = lambda *a, _side=side, **k: self._deny(_side, "Initialization TX forbidden")
            robot._send_msgs = robot._send_msg
            self.grippers[side] = robot.init_effector("agx_gripper")
            self.grippers[side]._send_msg = robot._send_msg
            comm = robot.create_comm()
            self.comms[side] = comm
            self._wrap_comm(side, comm)  # No ticket: all sends still denied.
            robot.connect()
            self._wrap_bus(side, comm)
            robot._send_msg, robot._send_msgs = original_send, original_sends
        self.emit("connected_passively", {"sdk_version": sdk.__version__, "transmission_counts": self.counts})

    def read(self):
        state = {side: arms.snapshot(self.robots[side], self.grippers[side]) for side in SIDES}
        self.report["after"] = copy.deepcopy(state)
        self.report["samples"] += 1
        self.emit("feedback", state)
        return state

    def checked(self, state):
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN ownership/TX guard violation: " + repr(self.violations))
        stamps = []
        for side in SIDES:
            current = state[side]
            health = arms.control_health(current, allowed_control_modes=self.CONTROL_MODES,
                                         require_enabled=self.REQUIRE_ENABLED)
            if not health["healthy"]:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, health["reasons"]))
            status = current["arm_status"]
            allowed = ((self.required_ctrl.get(side), 1) if side == self.pending_side
                       else (self.required_ctrl.get(side),))
            if side in self.required_ctrl and status["ctrl_mode"] not in allowed:
                raise RuntimeError(side + " control mode changed outside its authorized request")
            if (side == self.pending_side and self.pending_sent_at is not None
                    and status["ctrl_mode"] == 1
                    and current["fragment_timestamps_s"]["arm_status"] > self.pending_sent_at):
                # Once a new controller response confirms takeover, reverting
                # to teaching during the observation window is not tolerated.
                self.required_ctrl[side] = 1
            for key in ("teach_status", "motion_status"):
                if not _allowed_integer(status.get(key), (0,)):
                    raise RuntimeError("%s %s must be 0; got %r" % (side, key, status.get(key)))
            mode = status.get("mode_feedback")
            if not _allowed_integer(mode, (0, 1, 2)):
                raise RuntimeError("%s requires known P/J/L mode_feedback; got %r" % (side, mode))
            if side in self.expected_modes and mode != self.expected_modes[side]:
                raise RuntimeError(side + " motion mode changed during observation")
            self.check_enable_state(side, current)
            stamps.extend(current["fragment_timestamps_s"].values())
            if self.anchor is not None:
                self.check_drift(side, current, self.anchor[side])
        if max(stamps) - min(stamps) > arms.MAX_SKEW_S:
            raise RuntimeError("Cross-arm feedback skew exceeds limit")
        return state

    def check_drift(self, side, current, origin):
        drift = {"joint_rad": max(abs(a - b) for a, b in zip(current["joints_rad"], origin["joints_rad"])),
                 "position_m": math.dist(current["pose_m_rad"][:3], origin["pose_m_rad"][:3]),
                 "gripper_m": abs(current["gripper"]["width_m"] - origin["gripper"]["width_m"])}
        for key, value in drift.items():
            self.max_drift[side][key] = max(self.max_drift[side][key], value)
        for key, value in drift.items():
            if value > LIMITS[key]:
                raise RuntimeError("%s drift %s=%.6f exceeds %.6f" % (side, key, value, LIMITS[key]))

    def check_enable_state(self, side, state):
        # Existing takeover requires every driver and gripper enabled through
        # control_health. Standby startup adds a separate phased contract.
        pass

    def frame_spec(self, kind, side):
        if kind != "mode":
            raise RuntimeError("Only a mode frame is available in CAN takeover")
        return 0x151, bytes((1, self.expected_modes[side], 1, 0, 0, 0, 0, 0))

    def send_one(self, side, kind, sdk_call):
        """Internal fixed operation only; callers cannot supply a CAN payload."""
        self.frame_spec(kind, side)  # Reject unavailable operation before TX.
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN guard violation before dispatch")
            ticket = {"side": side, "thread": threading.get_ident(), "kind": kind,
                      "comm_calls": 0, "bus_calls": 0, "error": None}
            self.ticket = ticket
            try:
                value = sdk_call()
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["comm_calls"] != 1 or ticket["bus_calls"] != 1:
                    raise RuntimeError("SDK did not send exactly one " + kind + " frame")
            finally:
                self.ticket = None
        return time.time(), value

    @staticmethod
    def advanced(previous, current):
        return all(current[side]["fragment_timestamps_s"][name] > stamp
                   for side in SIDES for name, stamp in previous[side]["fragment_timestamps_s"].items())

    def observe_window(self, duration, *, requested_side=None, sent_at=None):
        start, advances = time.monotonic(), 0
        previous = self.checked(self.read())
        while time.monotonic() - start < duration:
            time.sleep(LIMITS["poll_s"])
            current = self.checked(self.read())
            if self.advanced(previous, current):
                advances += 1
                previous = current
        if advances < 3:
            raise RuntimeError("Insufficient independently advancing complete feedback samples")
        if requested_side is not None:
            if current[requested_side]["arm_status"]["ctrl_mode"] != 1:
                raise RuntimeError(requested_side + " did not confirm CAN control after one request")
            if any(stamp <= sent_at for side in SIDES
                   for stamp in current[side]["fragment_timestamps_s"].values()):
                raise RuntimeError("Post-request feedback predates completed mode transmission")
        return current

    def prepare(self):
        deadline = time.monotonic() + LIMITS["warmup_s"]
        while True:
            state = self.read()
            if all(state[side].get("status") == "complete" for side in SIDES):
                self.checked(state)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("No complete fresh dual-arm feedback within warmup")
            time.sleep(LIMITS["poll_s"])
        self.anchor = copy.deepcopy(state)
        self.expected_modes = {side: state[side]["arm_status"]["mode_feedback"] for side in SIDES}
        self.required_ctrl = {side: state[side]["arm_status"]["ctrl_mode"] for side in SIDES}
        self.report["before"] = copy.deepcopy(state)
        return self.observe_window(LIMITS["stable_s"])

    def request_side(self, side):
        state = self.checked(self.read())
        if state[side]["arm_status"]["ctrl_mode"] == 1:
            self.report["arms"][side] = {"status": "already_can_control", "command_sent": False}
            self.emit("already_can_control", {"side": side})
            return
        robot, mode = self.robots[side], self.expected_modes[side]
        cache = robot._msg_mode
        # These SDK cache fields are audited explicitly, not silently repaired.
        for key, value in (("ctrl_mode", 1), ("mit_mode", 0),
                           ("installation_pos", 0), ("residence_time", 0)):
            if not _allowed_integer(getattr(cache, key, None), (value,)):
                raise RuntimeError("Unexpected SDK mode cache field: " + key)
        cache.move_spd_rate_ctrl = 1  # In memory only; set_speed_percent sends an extra frame.
        expected = bytes((1, mode, 1, 0, 0, 0, 0, 0))
        self.emit("mode_request_intent", {"side": side, "arbitration_id": 0x151,
                                         "data_hex": expected.hex(), "mode_feedback": mode,
                                         "speed_percent": 1, "cached_target_activation_possible": True})
        # Durable intent must precede dispatch, and logging latency must not let
        # feedback go stale. Recheck BOTH arms after the journal write.
        self.checked(self.read())
        self.pending_side = side
        sent_at, _ = self.send_one(side, "mode", lambda: robot.set_motion_mode(("p", "j", "l")[mode]))
        self.pending_sent_at = sent_at
        self.emit("mode_frame_sent_unconfirmed", {"side": side, "sent_at": sent_at,
                                                   "transmission_counts": self.counts})
        self.observe_window(LIMITS["after_send_s"], requested_side=side, sent_at=sent_at)
        self.required_ctrl[side] = 1
        self.pending_side = None
        self.pending_sent_at = None
        self.report["arms"][side] = {"status": "can_control_observed", "command_sent": True}
        self.emit("can_control_observed", {"side": side, "drift": self.max_drift})

    def perform(self):
        for side in SIDES:
            self.request_side(side)
        final = self.checked(self.read())
        if any(final[side]["arm_status"]["ctrl_mode"] != 1 for side in SIDES):
            raise RuntimeError("Both arms must still report CAN control at completion")
        return "can_control_observed"

    def run(self):
        try:
            self.emit("operation_started", {"scope": self.SCOPE,
                                           "hold_not_validated": True})
            self.connect()
            self.prepare()
            self.report.update(ok=True, status=self.perform())
        except (Exception, KeyboardInterrupt) as exc:
            self.report["errors"].append({"type": type(exc).__name__, "detail": str(exc)})
            attempted = any(v["attempted_frames"] for v in self.counts.values())
            self.report["status"] = "aborted_after_dispatch" if attempted else "refused_before_send"
            self.report["site_attention_required"] = attempted
            # No retry, second-arm dispatch, hold, disable, reset, or quick stop.
            # A short passive record may help the operator assess a partial send.
            if attempted and set(self.robots) == set(SIDES):
                deadline = time.monotonic() + LIMITS["failure_observe_s"]
                try:
                    while time.monotonic() < deadline:
                        self.read()
                        time.sleep(LIMITS["poll_s"])
                except (Exception, KeyboardInterrupt) as observation_error:
                    self.report["errors"].append({"type": "failure_observation",
                                                 "detail": str(observation_error)})
        finally:
            self.ticket = None
            cleanup = {}
            for side, robot in self.robots.items():
                try:
                    robot.disconnect()
                    cleanup[side] = {"status": "disconnected", "physically_stopped": None}
                except Exception as exc:
                    cleanup[side] = {"status": "cleanup_failed", "error": str(exc),
                                     "physically_stopped": None}
                    self.report["errors"].append({"type": "cleanup", "side": side, "detail": str(exc)})
                    self.report.update(ok=False, status="cleanup_failed")
            self.report.update(transmission_counts=copy.deepcopy(self.counts),
                               transmission_counts_by_kind=copy.deepcopy(self.kind_counts),
                               hardware_commands_sent=sum(v["sent_frames"] for v in self.counts.values()),
                               enable_commands_sent=sum(v.get("enable", {}).get("sent_frames", 0)
                                                        for v in self.kind_counts.values()),
                               drift=copy.deepcopy(self.max_drift),
                               guard_violations=copy.deepcopy(self.violations),
                               cleanup={"arms": cleanup, "detail": "Disconnect is cleanup, not a physical stop"})
            if self.violations and self.report["ok"]:
                self.report.update(ok=False, status="guard_violation_during_cleanup")
                self.report["errors"].append({"type": "guard_violation", "detail": repr(self.violations)})
        return self.report


class _Startup(_Takeover):
    """Explicitly separate startup of two standby, fully disabled arms.

    Both mode requests must confirm while disabled before either enable frame.
    Enabling can activate a cached controller target; no target is supplied here.
    """
    CONTROL_MODES = (0, 1)
    REQUIRE_ENABLED = False
    FRAME_KINDS = ("mode", "enable")
    SCOPE = "Standby startup: two mode confirmations, then sequential joint enables"

    def __init__(self, profile, journal_callback):
        super().__init__(profile, journal_callback)
        self.enable_phase = dict.fromkeys(SIDES, "disabled")
        self.enabled_seen = {side: [False] * 7 for side in SIDES}
        self.report.update(operation="startup_arms", fold_ready=False, grasp_verified=False,
                           gripper_target_commands_sent=0,
                           gripper_enable_side_effect_possible=True,
                           enabled_arms=[], gripper_enabled={side: None for side in SIDES},
                           startup_does_not_validate_position_hold=True)

    @staticmethod
    def enable_flags(state):
        return [state["drivers"][str(i)]["foc_status"].get("driver_enable_status") for i in range(1, 7)] + [
            state["gripper"]["foc_status"].get("driver_enable_status")]

    def check_enable_state(self, side, state):
        flags, phase = self.enable_flags(state), self.enable_phase[side]
        if any(type(flag) is not bool for flag in flags):
            raise RuntimeError(side + " joint/gripper enable flags must be known booleans")
        if self.anchor is None and state["arm_status"]["ctrl_mode"] != 0:
            raise RuntimeError(side + " startup requires initial standby ctrl_mode=0; no partial replay")
        if phase == "disabled":
            if any(flags):
                raise RuntimeError(side + " startup requires all drivers and gripper disabled before enable")
        elif phase == "enabling":
            # Some firmware may include the gripper in enable(255). Accept that
            # only for the requested arm, without issuing a gripper target.
            if any(was and not now for was, now in zip(self.enabled_seen[side], flags)):
                raise RuntimeError(side + " enable feedback regressed after becoming true")
            self.enabled_seen[side] = [was or now for was, now in zip(self.enabled_seen[side], flags)]
        elif phase == "enabled":
            if not all(flags[:6]) or flags[6] is not self.enabled_seen[side][6]:
                raise RuntimeError(side + " enabled arm/gripper changed outside its startup request")
        else:
            raise RuntimeError("Unknown internal startup phase")

    def frame_spec(self, kind, side):
        if kind == "enable":
            return 0x471, bytes((7, 2, 0, 0, 0, 0, 0, 0))
        return super().frame_spec(kind, side)

    def enable_side(self, side):
        self.checked(self.read())
        if self.enable_phase[side] != "disabled" or any(self.required_ctrl.get(s) != 1 for s in SIDES):
            raise RuntimeError("Both CAN mode requests must confirm before the first enable")
        can_id, expected = self.frame_spec("enable", side)
        self.emit("enable_request_intent", {"side": side, "arbitration_id": can_id,
                                           "data_hex": expected.hex(), "sdk_api": "enable(255)",
                                           "gripper_enable_may_change": True,
                                           "cached_target_activation_possible": True})
        self.checked(self.read())
        self.enable_phase[side] = "enabling"
        sent_at, sdk_return = self.send_one(side, "enable", lambda: self.robots[side].enable(255))
        self.emit("enable_frame_sent_unconfirmed", {"side": side, "sent_at": sent_at,
                                                    "sdk_cached_return": sdk_return,
                                                    "transmission_counts": self.counts})
        state = self.observe_window(LIMITS["after_send_s"], requested_side=side, sent_at=sent_at)
        flags = self.enable_flags(state[side])
        if not all(flags[:6]):
            raise RuntimeError(side + " did not confirm all six drivers enabled after one request")
        self.enable_phase[side] = "enabled"
        self.enabled_seen[side] = flags
        self.report["enabled_arms"].append(side)
        self.report["gripper_enabled"][side] = flags[6]
        self.report["arms"][side] = {"status": "joints_enabled_observed", "ctrl_mode": 1,
                                      "driver_enabled": flags[:6], "gripper_enabled": flags[6],
                                      "fold_ready": False}
        self.emit("joints_enabled_observed", {"side": side, "gripper_enabled": flags[6],
                                              "drift": self.max_drift})

    def perform(self):
        super().perform()  # Both arms must confirm mode=1 with all drives disabled.
        for side in SIDES:
            self.enable_side(side)
        final = self.checked(self.read())
        for side in SIDES:
            self.report["gripper_enabled"][side] = self.enable_flags(final[side])[6]
        return "joints_enabled_observed_not_fold_ready"

    def run(self):
        report = super().run()
        # Preserve the last actual flags even after partial startup or failure.
        last = report.get("after") or {}
        report["last_enable_feedback"] = {}
        for side, state in last.items():
            try:
                flags = self.enable_flags(state)
                report["last_enable_feedback"][side] = {"driver_enabled": flags[:6],
                                                       "gripper_enabled": flags[6]}
                report["gripper_enabled"][side] = flags[6]
            except (KeyError, TypeError):
                report["last_enable_feedback"][side] = {"status": "unavailable"}
        return report


def request_can_control(profile, journal_callback):
    """Request ctrl_mode=1 once per eligible arm under the caller's device lock.

    Requires two healthy enabled stationary arms, teach_status=0, known P/J/L,
    and a synchronous durable journal callback. Does not unlock motion execution.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _Takeover(profile, journal_callback).run()


def startup_arms(profile, journal_callback):
    """Explicit standby startup; caller holds the device lock and persists journal.

    Requires both arms ctrl_mode=0 with all joints/grippers disabled. Sends at
    most one mode and one enable frame per arm. Never supplies targets, retries,
    or a stop fallback. Successful startup does not authorize task motion.
    """
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _Startup(profile, journal_callback).run()
