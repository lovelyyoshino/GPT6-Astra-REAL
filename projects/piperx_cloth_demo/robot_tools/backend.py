"""Vendor command adapter. Real commissioning is BLOCKED until hold is validated.

No IK, perception, target repair, automatic enable, or emergency-stop fallback.
The command plumbing is testable with a fake SDK; that is not hardware validation.
"""
import copy
import math
import threading

from . import arms as _arms


class PyAgxBackend:
    def __init__(self, profile):
        self.profile = copy.deepcopy(profile)
        self.robots, self.grippers = {}, {}
        self._connected = False
        self._transmitting = False
        self._lock = threading.RLock()
        self._tx = threading.local()
        self._send_buses = {}
        self._counts = {side: {"attempted_frames": 0, "sent_frames": 0}
                        for side in ("left", "right")}

    def commissioning_errors(self):
        # This cannot be cleared with JSON flags. A verified stop implementation
        # and explicit code review are required before physical commissioning.
        return [{"code": "position_hold_unverified", "field": "hold_stop_and_recovery",
                 "detail": "No validated hold for these arms/firmware; prior emergency stop descended. "
                           "No emergency-stop, disable, reset, or current-target substitute is allowed."}]

    def _commissioning_guard(self):
        errors = self.commissioning_errors()
        if errors:
            raise RuntimeError("Physical backend blocked: " + repr(errors))

    def _wrap_send(self, side, comm):
        original = comm.send
        def counted_send(*args, **kwargs):
            with self._lock:
                self._commissioning_guard()
                if not self._connected or not self._transmitting:
                    raise RuntimeError("CAN TX outside an explicitly guarded command")
                if comm.send_bus is not self._send_buses.get(side):
                    raise RuntimeError("CAN send bus changed after connection")
                ticket = {"side": side, "calls": 0, "error": None}
                self._tx.active = ticket
                try:
                    result = original(*args, **kwargs)
                    # SDK comm.send may swallow a bus exception; RX can clear
                    # its shared last_error. Only this call's result is trusted.
                    if ticket["error"] is not None:
                        raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                    if ticket["calls"] != 1:
                        raise RuntimeError("Expected exactly one underlying CAN send")
                    return result
                finally:
                    self._tx.active = None
        comm.send = counted_send

    def _wrap_bus_send(self, side, comm):
        bus = comm.send_bus
        original = bus.send
        self._send_buses[side] = bus
        def counted_bus_send(*args, **kwargs):
            with self._lock:
                ticket = getattr(self._tx, "active", None)
                self._commissioning_guard()
                if ticket is None or ticket["side"] != side or not self._transmitting:
                    raise RuntimeError("Underlying CAN TX outside the guarded comm.send call")
                ticket["calls"] += 1
                self._counts[side]["attempted_frames"] += 1
                try:
                    result = original(*args, **kwargs)
                except BaseException as exc:
                    ticket["error"] = exc
                    raise
                self._counts[side]["sent_frames"] += 1
                return result
        bus.send = counted_bus_send

    def connect(self):
        with self._lock:
            self._commissioning_guard()
            if self._connected or self.robots:
                raise RuntimeError("Backend already connected or needs cleanup")
            configs = self.profile["arms"]
            _arms._validate_arms(configs)
            if set(configs) != {"left", "right"}:
                raise ValueError("Physical backend requires both arms")
            _arms._preflight(configs)  # All USB bindings and passive CAN probes first.
            sdk = _arms._load_sdk(self.profile["sdk_path"])
            try:
                for side in ("left", "right"):
                    cfg = configs[side]
                    config = sdk.create_agx_arm_config(
                        robot=cfg["model"], firmeware_version=cfg["firmware"],
                        channel=cfg["channel"], interface="socketcan",
                        auto_connect=False, enable_check_can=False)
                    robot = sdk.AgxArmFactory.create_arm(config)
                    self.robots[side] = robot
                    self.grippers[side] = robot.init_effector("agx_gripper")
                    # Creation is passive in the audited SDK. Install TX guard
                    # before connect; unexpected initialization TX fails closed.
                    comm = robot.create_comm()
                    self._wrap_send(side, comm)
                    robot.connect()
                    self._wrap_bus_send(side, comm)
                self._connected = True
            except Exception as exc:
                cleanup = self.close()
                raise RuntimeError("Connection failed: %s; cleanup=%r" % (exc, cleanup)) from exc
            return {"status": "connected", "enabled_automatically": False,
                    "transmission_counts": self.transmission_counts()}

    def snapshot(self):
        with self._lock:
            states = {}
            for side in ("left", "right"):
                try:
                    if not self._connected or side not in self.robots:
                        raise RuntimeError("Both arms must be connected")
                    states[side] = _arms.snapshot(self.robots[side], self.grippers[side])
                except Exception as exc:
                    states[side] = {"status": "failed", "error": str(exc)}
            stamps = [v for state in states.values()
                      for v in state.get("fragment_timestamps_s", {}).values()]
            skew = max(stamps) - min(stamps) if stamps else None
            complete = all(v.get("status") == "complete" for v in states.values())
            complete = complete and skew is not None and skew <= _arms.MAX_SKEW_S
            return {"status": "complete" if complete else "partial", "arms": states,
                    "max_cross_arm_fragment_skew_s": skew,
                    "transmission_counts": self.transmission_counts()}

    def _command_guard(self):
        self._commissioning_guard()
        if not self._connected or set(self.robots) != {"left", "right"}:
            raise RuntimeError("Both arms must be connected")
        state = self.snapshot()
        if state["status"] != "complete":
            raise RuntimeError("Incomplete or unsynchronized current feedback")
        for side, arm_state in state["arms"].items():
            health = _arms.control_health(arm_state)
            if not health["healthy"]:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, health["reasons"]))

    @staticmethod
    def _target_side(target):
        side = target.get("arm")
        if side not in ("left", "right"):
            raise ValueError("Explicit left/right arm required")
        return side

    def move(self, target):
        with self._lock:
            self._command_guard()
            side = self._target_side(target)
            if target.get("frame") != side + "_base" or target.get("reference") != "sdk_flange":
                raise ValueError("Target must use this arm's base and SDK flange")
            _arms._six_finite(target.get("pose_m_rad"))
            pose = list(target["pose_m_rad"])
            if abs(pose[3]) > math.pi or abs(pose[5]) > math.pi or abs(pose[4]) > math.pi / 2:
                raise ValueError("RPY outside vendor bounds; no clipping")
            motion, speed = target.get("motion"), target.get("speed_percent")
            ceiling = min(5, self.profile.get("preview_max_speed_percent", 5))
            if motion not in ("move_p", "move_l") or type(speed) is not int or not 1 <= speed <= ceiling:
                raise ValueError("Unsupported motion or speed")
            self._transmitting = True
            try:
                self.robots[side].set_speed_percent(speed)
                # Speed mode setting itself transmits. Recheck before the target.
                self._command_guard()
                getattr(self.robots[side], motion)(pose)
            finally:
                self._transmitting = False
            return {"status": "sent_unconfirmed", "controller_accepted": None,
                    "transmission_counts": self.transmission_counts()}

    def grip(self, target):
        with self._lock:
            self._command_guard()
            side = self._target_side(target)
            width, force = target.get("gripper_width_m"), target.get("gripper_force_N")
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in (width, force)):
                raise ValueError("Explicit finite gripper width_m and force_N required")
            # Matches the preview ceiling, not a safe force recommendation.
            # Commissioning blocks commands until units/hardware are verified.
            if not 0 <= width <= 0.1 or not 0 <= force <= 5:
                raise ValueError("Gripper request outside declared width/force bounds")
            self._transmitting = True
            try:
                self.grippers[side].move_gripper_m(value=width, force=force)
            finally:
                self._transmitting = False
            return {"status": "sent_unconfirmed", "controller_accepted": None,
                    "transmission_counts": self.transmission_counts()}

    def pose_error(self, actual, target):
        a, b = _arms._six_finite(actual), _arms._six_finite(target)
        _arms._load_sdk(self.profile["sdk_path"])
        from pyAgxArm.utiles.tf import euler_convert_quat
        qa, qb = euler_convert_quat(*a[3:]), euler_convert_quat(*b[3:])
        norm = math.sqrt(sum(q * q for q in qa) * sum(q * q for q in qb))
        dot = abs(sum(x * y for x, y in zip(qa, qb))) / norm
        return math.dist(a[:3], b[:3]), 2 * math.acos(min(1.0, dot))

    def request_hold_all(self):
        # Honest per-arm failure, including disconnected arms. Never substitute
        # a power-removing stop or cached-target motion for validated hold.
        return {"status": "unavailable", "held": False, "all_stopped": False,
                "arms": {side: {"status": "unavailable", "held": False,
                                 "connected": self._connected and side in self.robots,
                                 "reason": "No physically validated position hold; no command sent"}
                         for side in ("left", "right")},
                "transmission_counts": self.transmission_counts()}

    def close(self):
        with self._lock:
            self._connected = False
            results = {}
            for side, robot in list(self.robots.items()):
                try:
                    robot.disconnect()
                    results[side] = {"status": "disconnected", "stopped": None}
                    self.robots.pop(side)
                    self.grippers.pop(side, None)
                    self._send_buses.pop(side, None)
                except Exception as exc:
                    results[side] = {"status": "cleanup_failed", "error": str(exc), "stopped": None}
            return {"status": "partial" if self.robots else "complete", "arms": results,
                    "stopped": None, "detail": "Disconnect is resource cleanup, not motion stop",
                    "transmission_counts": self.transmission_counts()}

    def transmission_counts(self):
        with self._lock:
            result = copy.deepcopy(self._counts)
            return {"arms": result,
                    "attempted_frames": sum(v["attempted_frames"] for v in result.values()),
                    "sent_frames": sum(v["sent_frames"] for v in result.values()),
                    "controller_acceptance": "not_established_by_CAN_send"}
