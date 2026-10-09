"""Once-per-joint limit queries on the pair device's retained SDK connections.

Only twelve fixed 0x472 requests are available. Raw 0x473 receive windows are
the evidence, never the SDK's shared parser cache or its aggregate timestamp.
This operation changes neither readiness, targets, modes nor enable state.
"""
import copy
import math
import threading
import time

from . import arms
from .pair_preparation import _PreparationObserver, observe_preparation
from .single_supervised_actions import BOUNDS
from .takeover import SIDES


SCHEMA = "piper_pair_controller_limits_capture_v1"
RESPONSE_TIMEOUT_S = 1.0
DUPLICATE_WINDOW_S = 0.2
MAX_WINDOW_FRAMES = 32
_ACTION_FIELDS = ("arm", "kind", "target", "passive_arm", "active", "dispatched",
                  "sent_at", "mode_l_confirmed", "baseline_window", "dispatch_anchor",
                  "boundary_reference", "frame_limit", "max_drift", "report", "anchor",
                  "expected_modes", "enable_states", "control_modes")


class _LimitsExecutor:
    def __init__(self, device, initial, report):
        self.device, self.action, self.report = device, device._action, report
        self.lock = threading.RLock()
        self.window = self.side = self.joint = None
        self.closed = False
        self.rx_error = None
        self.callbacks = {}
        self.sent_monotonic = None
        self.observer = (device._preparation if not device._task_ready
                         else _PreparationObserver(self.action))
        if device._task_ready:
            self.observer.anchor = copy.deepcopy(self.action.idle_anchor)
            self.observer.expected_modes = copy.deepcopy(self.action.expected_modes)
            self.observer.required_ctrl = {s: initial[s]["arm_status"]["ctrl_mode"] for s in SIDES}
        self.check_states(initial)

    def encoder_guard(self):
        self.device._connected_usable()
        with self.lock:
            if self.rx_error:
                raise RuntimeError(self.rx_error)

    def check_states(self, states):
        with self.lock:
            if self.rx_error:
                raise RuntimeError(self.rx_error)
        self.observer.checked(states)
        for side in SIDES:
            if (states[side]["arm_status"]["ctrl_mode"] != 1
                    or not all(self.observer.enable_flags(states[side])[:6])):
                raise RuntimeError(side + " lost CAN control or joint enable state during limit query")
        if self.device._task_ready:
            # Preparation observation allows the original preparation bands;
            # an already-ready connection retains its stricter idle pose band.
            for side in SIDES:
                current, origin = states[side], self.action.idle_anchor[side]
                if (max(abs(a-b) for a, b in zip(current["joints_rad"], origin["joints_rad"])) > BOUNDS["joint_span_rad"]
                        or math.dist(current["pose_m_rad"][:3], origin["pose_m_rad"][:3]) > BOUNDS["position_span_m"]
                        or self.action.rotation_distance(current["pose_m_rad"], origin["pose_m_rad"]) > BOUNDS["rotation_span_rad"]):
                    raise RuntimeError(side + " left its retained task-ready stationary anchor during limit query")
        self.action.check_freshness(states)
        return states

    def read(self):
        self.encoder_guard()
        states = self.action.read()
        self.check_states(states)
        return states

    def _reject(self, side, record, detail):
        self.rx_error = detail
        self.action.violations.append({"side": side, "detail": detail, "arbitration_id": 0x473})
        if self.window is not None:
            if len(self.window["rejected_frames"]) < MAX_WINDOW_FRAMES:
                self.window["rejected_frames"].append(record)

    def collect(self, side, original, frame):
        if getattr(frame, "arbitration_id", None) != 0x473:
            if original is not None:
                return original(frame)
            return None
        now = time.time()
        with self.lock:
            if self.closed:
                return None
            try:
                stamp = frame.timestamp
                valid_stamp = type(stamp) in (int, float) and math.isfinite(stamp)
                record = {"timestamp": stamp if valid_stamp else None, "received_unix_s": now,
                          "dlc": frame.dlc, "payload_hex": bytes(frame.data).hex()}
                if not valid_stamp:
                    # Keep malformed evidence serializable so the host does
                    # not lose the partial receipt while persisting a fault.
                    record["timestamp_raw_repr"] = repr(stamp)[:256]
                if not valid_stamp or stamp > now:
                    self._reject(side, record, "Invalid joint-limit receive timestamp")
                    return None
                window = self.window
                if window is None or not window["active"]:
                    self.report["pre_or_post_window_limit_frames"][side] += 1
                    # A queued pre-request frame is diagnostic only. Once a
                    # query has begun, an unsolicited new reply is ambiguous.
                    if self.sent_monotonic is not None and stamp >= self.report["began_at"]:
                        self._reject(side, record, "Joint-limit response outside its request window")
                    return None
                if stamp < window["request_started_unix_s"]:
                    if len(window["ignored_stale_frames"]) >= MAX_WINDOW_FRAMES:
                        self._reject(side, record, "Joint-limit stale reply budget exceeded")
                    else:
                        window["ignored_stale_frames"].append(record)
                    return None
                valid = (side == self.side and frame.dlc == 8 and len(frame.data) == 8
                         and frame.data[0] == self.joint and frame.data[7] == 0 and frame.is_rx
                         and not any((frame.is_extended_id, frame.is_remote_frame, frame.is_error_frame,
                                      frame.is_fd, frame.bitrate_switch, frame.error_state_indicator)))
                if not valid:
                    self._reject(side, record, "Unexpected joint-limit response")
                    return None
                elapsed = time.monotonic()-self.sent_monotonic
                if not 0 <= elapsed <= RESPONSE_TIMEOUT_S:
                    self._reject(side, record, "Joint-limit response exceeded one-second deadline")
                    return None
                record.update(side=side, valid_can_data_frame=True, request_elapsed_s=elapsed)
                if window["response_frames"]:
                    first = window["response_frames"][0]
                    duplicates = window["identical_duplicate_frames"]
                    previous = duplicates[-1] if duplicates else first
                    if record["payload_hex"] != first["payload_hex"]:
                        self._reject(side, record, "Conflicting joint-limit response")
                    elif stamp < previous["timestamp"] or now < previous["received_unix_s"]:
                        self._reject(side, record, "Joint-limit response timestamp regressed")
                    elif 1 + len(duplicates) >= MAX_WINDOW_FRAMES:
                        self._reject(side, record, "Joint-limit identical reply budget exceeded")
                    else:
                        # 0x473 carries a value, not a unique transaction ID.
                        # Keep the first canonical value and all bounded equal
                        # RX evidence. No new request or window extension.
                        duplicates.append(record)
                    return None
                window["response_frames"].append(record)
            except Exception as exc:
                self._reject(side, {"malformed": type(exc).__name__}, "Malformed joint-limit response")
                return None
        # Preserve the existing parser/command-observer callback; no query is
        # made here. No action lock is acquired while our metadata lock is held.
        if original is not None:
            return original(frame)
        return None

    def install(self):
        for side in SIDES:
            comm = self.action.comms[side]
            original = comm.get_callback()
            callback = lambda frame, side=side, original=original: self.collect(side, original, frame)
            self.callbacks[side] = (comm, original, callback)
            comm.set_callback(callback)

    def restore(self):
        with self.lock:
            self.closed = True
            self.finish_window()
            errors = [self.rx_error] if self.rx_error else []
        for side, (comm, original, callback) in self.callbacks.items():
            try:
                if comm.get_callback() is not callback:
                    errors.append(side + " query callback ownership changed")
                comm.set_callback(original)
            except Exception as exc:
                errors.append(side + " callback restoration failed: " + str(exc))
        if errors:
            raise RuntimeError("; ".join(errors))

    def finish_window(self):
        if self.window is not None and self.window["active"]:
            self.window.update(active=False, finished_unix_s=time.time())
            row = self.report["joint_limits"][self.side][str(self.joint)]
            if len(self.window["response_frames"]) == 1:
                row["raw_response_hex"] = self.window["response_frames"][0]["payload_hex"]

    def send_frame(self, side, original, frame, *args, **kwargs):
        self.encoder_guard()  # May perform durable host I/O.
        states = {s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES}
        self.check_states(states)
        self.action.check_freshness(states)
        with self.lock:
            if side != self.side or self.window is not None:
                raise RuntimeError("Joint-limit query permits one selected request window")
            started = time.time()
            self.sent_monotonic = time.monotonic()
            self.window = {"active": True, "request_started_unix_s": started,
                           "response_frames": [], "ignored_stale_frames": [], "rejected_frames": [],
                           "duplicate_policy": "same_window_identical_payload_v1",
                           "identical_duplicate_frames": []}
            self.report["joint_limits"][side][str(self.joint)] = {
                "status": "unconfirmed", "response_evidence": self.window, "raw_response_hex": ""}
            receipt = self.report["query_receipts"][side][str(self.joint)]
            receipt.update(sent_at=started, outcome="uncertain")
        # The inherited exact-frame comm/bus guards still enclose actual send.
        result = original(frame, *args, **kwargs)
        with self.lock:
            receipt.update(outcome="returned", returned_at=time.time())
        return result

    def query(self, side, joint):
        self.read()
        self.action.reset_action(side, "move", [0.0]*6)
        self.action.frame_limit = 1
        payload = bytes((joint, 1, 0, 0, 0, 0, 0, 0))
        with self.lock:
            self.side, self.joint, self.window = side, joint, None
            self.report["query_receipts"][side][str(joint)] = {
                "arbitration_id": 0x472, "data_hex": payload.hex(), "outcome": "not_attempted",
                "sent_at": None, "returned_at": None}
        self.action.emit("pair_joint_limit_query_intent", {"side": side, "joint_index": joint,
                                                          "arbitration_id": 0x472, "data_hex": payload.hex()})
        self.read()
        action = self.action
        with action.lock:
            if action.ticket is not None or action.violations:
                raise RuntimeError("Joint-limit query cannot overlap a dispatch or CAN violation")
            ticket = {"side": side, "thread": threading.get_ident(), "kind": "action",
                      "frames": [(0x472, payload)], "comm_calls": 0, "bus_calls": 0, "error": None}
            action.ticket = ticket
            try:
                # Exactly one SDK request. Its return may reflect a shared old
                # parser entry, so it is deliberately not a confirmation.
                action.robots[side].get_joint_angle_vel_limits(joint_index=joint, timeout=0.0, min_interval=0.0)
                if ticket["error"] is not None:
                    raise RuntimeError("Joint-limit query send uncertain: " + str(ticket["error"]))
                if ticket["comm_calls"] != 1 or ticket["bus_calls"] != 1:
                    raise RuntimeError("Expected exactly one joint-limit request frame")
            finally:
                action.ticket = None
        while True:
            state = self.read()
            with self.lock:
                received = bool(self.window["response_frames"])
            if received:
                break
            if time.monotonic()-self.sent_monotonic >= RESPONSE_TIMEOUT_S:
                raise RuntimeError("Joint-limit response missing after one request")
            time.sleep(BOUNDS["poll_s"])
        end = time.monotonic() + DUPLICATE_WINDOW_S
        while time.monotonic() < end:
            time.sleep(min(BOUNDS["poll_s"], max(0.0, end-time.monotonic())))
            state = self.read()
        with self.lock:
            if self.rx_error:
                raise RuntimeError(self.rx_error)
            self.finish_window()
            row = self.report["joint_limits"][side][str(joint)]
            raw = bytes.fromhex(self.window["response_frames"][0]["payload_hex"])
            low, high = int.from_bytes(raw[3:5], "big", signed=True), int.from_bytes(raw[1:3], "big", signed=True)
            row.update(raw_response_hex=raw.hex(), raw_min_angle_tenth_deg=low,
                       raw_max_angle_tenth_deg=high, raw_max_joint_spd=int.from_bytes(raw[5:7], "big"))
            if low >= high:
                raise RuntimeError("Joint-limit response requires a strictly increasing signed range")
            limits = [low*.1*math.pi/180, high*.1*math.pi/180]
            row.update(status="confirmed", decoded_limits_rad=limits,
                       observed_joint_rad=state[side]["joints_rad"][joint-1], motion_permission=False)
            self.report["controller_limits_rad"][side].append(limits)
        self.action.emit("pair_joint_limit_reply", {"side": side, "joint_index": joint, "result": copy.deepcopy(row)})


def inspect_joint_limits(device):
    """Called under the existing device operation lock; no new connection."""
    action, runner, saved = device._action, None, None
    initial_totals = action.totals()
    report = {"schema": SCHEMA, "operation": "inspect_joint_limits", "ok": False,
              "status": "joint_limits_capture_failed", "began_at": time.time(), "ended_at": None,
              "joint_limits": {s: {} for s in SIDES}, "query_receipts": {s: {} for s in SIDES},
              "controller_limits_rad": {s: [] for s in SIDES},
              "pre_or_post_window_limit_frames": dict.fromkeys(SIDES, 0),
              "controller_limits_changed": False, "sdk_joint_limits_changed": False,
              "actuator_commands_sent": 0, "target_commands_sent": 0, "mode_commands_sent": 0,
              "enable_commands_sent": 0, "stop_commands_sent": 0,
              "motion_gate_unlocked": False, "limits_are_motion_permission": False,
              "task_ready": device._task_ready, "physical_stop_verified": None,
              "response_timeout_s": RESPONSE_TIMEOUT_S, "duplicate_observation_s": DUPLICATE_WINDOW_S,
              "scope": "Twelve same-connection queries; raw receive correlation is not a firmware transaction ID",
              "speed_unit_note": "Raw velocity field retained; manufacturer parser uses 0.01 rad/s while message comments say 0.001 rad/s; no speed authorization",
              "errors": []}
    try:
        device._connected_usable()
        if action.ticket is not None or action._executor() is not None or any(action.grasps.values()):
            raise RuntimeError("Limit inspection requires an idle pair without unresolved or retained grasps")
        initial = (device._observe() if device._task_ready else observe_preparation(device))["arms"]
        requirements = []
        for side in SIDES:
            if initial[side]["arm_status"]["ctrl_mode"] != 1:
                requirements.append(side + "_arm_CAN_control_mode_1")
            if not all(_PreparationObserver.enable_flags(initial[side])[:6]):
                requirements.append(side + "_arm_six_joint_drivers_enabled")
        if requirements:
            report.update(status="preparation_required", requirements=requirements)
        else:
            saved = {key: copy.deepcopy(getattr(action, key)) for key in _ACTION_FIELDS}
            runner = _LimitsExecutor(device, initial, report)
            action.auxiliary_executor = runner
            runner.install()
            for side in SIDES:
                for joint in range(1, 7):
                    runner.query(side, joint)
            runner.read()
            report.update(ok=True, status="joint_limits_received_no_motion_commands")
    except BaseException as exc:
        device._fault = str(exc)
        report["errors"].append({"type": type(exc).__name__, "detail": str(exc)})
    finally:
        if runner is not None:
            try:
                runner.restore()
            except BaseException as exc:
                device._fault = str(exc)
                report["errors"].append({"type": type(exc).__name__, "detail": str(exc)})
            action.auxiliary_executor = None
            action.ticket = None
        if saved is not None:
            # Archive the last query's counts, then restore the previous action
            # context. Never reset lifetime totals, violations or idle anchors.
            action.reset_action(saved["arm"], saved["kind"], saved["target"])
            for key, value in saved.items():
                setattr(action, key, value)
        totals = action.totals()
        delta = {s: {key: value-initial_totals[s][key] for key, value in row.items()}
                 for s, row in totals.items()}
        report.update(ended_at=time.time(), transmission_counts=delta, session_transmission_counts=totals,
                      hardware_commands_sent=sum(v["sent_frames"] for v in delta.values()),
                      joint_limit_queries_sent=sum(v["sent_frames"] for v in delta.values()),
                      joint_limit_queries_attempted=sum(v["attempted_frames"] for v in delta.values()),
                      guard_violations=copy.deepcopy(action.violations), fault_latched=device._fault is not None)
        if report["errors"] or device._fault is not None:
            report.update(ok=False, status="joint_limits_capture_failed")
    return copy.deepcopy(report)
