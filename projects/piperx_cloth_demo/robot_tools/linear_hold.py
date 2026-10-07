"""One bounded MOVE_L target replacement experiment, not a general stop tool.

The caller owns the device lock and must establish clearance for the entire
original +6 mm path. Mode switching can activate a cached controller target.
Failure never sends an emergency stop, disable, reset, retry, or extra target.
"""
import copy
import math
import struct
import threading
import time

from . import arms
from .takeover import COMMAND_IDS, LIMITS, SIDES, _Takeover, _allowed_integer

PROBE = {"travel_m": 0.006, "speed_percent": 1, "progress_m": 0.001,
         "remaining_m": 0.003, "original_separation_m": 0.0015,
         "hold_tolerance_m": 0.001, "stable_span_m": 0.0005,
         "stable_s": 3.0, "timeout_s": 10.0, "poll_s": 0.01,
         "feedback_age_s": 0.05,
         "lateral_m": 0.002, "joint_excursion_rad": 0.05,
         "orientation_rad": 0.02, "overwrite_sample_slip_m": 0.0005,
         "initial_sample_slip_m": 0.0005, "rejection_acceptance_s": 0.2,
         "acceptance_position_drift_m": 0.0005}


def _pose_frames(pose):
    pose = arms._six_finite(pose)
    if (any(abs(x) > 1.0 for x in pose[:3]) or abs(pose[3]) > math.pi
            or abs(pose[4]) > math.pi / 2 or abs(pose[5]) > math.pi):
        raise ValueError("Probe pose outside finite SDK bounds; no clamp allowed")
    raw = [round(value * 1e6) for value in pose[:3]] + [
        round(value * (180.0 / math.pi) * 1e3) for value in pose[3:]]
    return [(0x152 + index, struct.pack(">ii", *raw[index * 2:index * 2 + 2])) for index in range(3)]


class _LinearHold(_Takeover):
    CONTROL_MODES = (1,)
    REQUIRE_ENABLED = False
    FRAME_KINDS = ("mode", "travel", "overwrite")
    SCOPE = "Bounded +6 mm MOVE_L and one measured-position replacement; local evidence only"

    def __init__(self, profile, journal_callback, arm, prior_mode_only_record=None):
        super().__init__(profile, journal_callback)
        self.arm = arm
        self.phase = "baseline"
        self.operations = []
        self.origin_pose = self.original_target = self.hold_target = None
        self.travel_sent_at = self.overwrite_sent_at = None
        self.travel_finished_monotonic = None
        self.dispatch_anchor = None
        self.mode_l_confirmed = False
        self.gripper_enable = {}
        self.quaternion = None
        self.joint_limits = {}
        self.prior = copy.deepcopy(prior_mode_only_record)
        self.validate_prior_record()
        self.report.update(operation="qualify_linear_hold", arm=arm, qualified=False,
                           probe_limits=dict(PROBE), hold_scope="This one pose, 1% speed, unloaded +Z trial only",
                           general_stop_validated=False, motion_gate_unlocked=False,
                           original_target_approached=False, overwrite_sent=False,
                           caller_full_original_path_clearance_required=True,
                           inactive_known_limit_violations={}, inactive_arm_motion_qualified=False,
                           prior_mode_only_source_run_id=self.prior.get("source_run_id") if self.prior else None,
                           rejected_target_recovery_trial=self.prior is not None,
                           fresh_normal_feedback_observed=False, prior_rejection_recovery_observed=False,
                           target_processing_state="not_requested",
                           initial_dispatch="One public MOVE_L call: 151 then 152/153/154 without intermediate wait",
                           automatic_retry=False, target_calls_sent=0)

    def validate_prior_record(self):
        """Internal receipt, loaded and claimed by service, never a status override."""
        if self.prior is None:
            return
        prior = self.prior
        if not isinstance(prior, dict):
            raise ValueError("Prior mode-only receipt must be an internal saved record")
        result, request = prior.get("result", {}), prior.get("request", {})
        if (not isinstance(result, dict) or not isinstance(request, dict)
                or not isinstance(prior.get("source_run_id"), str)
                or prior["source_run_id"] != result.get("run_id")
                or prior["source_run_id"] != request.get("run_id")
                or request.get("arms") != self.profile["arms"]
                or request.get("arguments", {}).get("arm") != self.arm
                or result.get("operation") != "qualify_linear_hold" or result.get("arm") != self.arm
                or result.get("status") != "aborted_after_dispatch" or result.get("ok") is not False
                or result.get("qualified") is not False or result.get("guard_violations") != []):
            raise ValueError("Prior mode-only receipt identity/outcome mismatch")
        for key, expected in (("hardware_commands_sent", 1), ("target_commands_sent", 0),
                              ("target_calls_sent", 0), ("enable_commands_sent", 0),
                              ("stop_commands_sent", 0), ("retries", 0)):
            if not _allowed_integer(result.get(key), (expected,)):
                raise ValueError("Prior mode-only receipt has other or uncertain transmissions")
        counts, kinds = result.get("transmission_counts", {}), result.get("transmission_counts_by_kind", {})
        if set(counts) != set(SIDES) or set(kinds) != set(SIDES):
            raise ValueError("Prior mode-only receipt must cover both arms")
        for side in SIDES:
            expected = int(side == self.arm)
            if (set(counts[side]) != {"attempted_frames", "sent_frames", "blocked_frames"}
                    or any(not _allowed_integer(counts[side].get(key), (value,)) for key, value in
                           (("attempted_frames", expected), ("sent_frames", expected), ("blocked_frames", 0)))
                    or set(kinds[side]) != {"mode", "travel", "overwrite"}):
                raise ValueError("Prior mode-only receipt counts mismatch")
            for kind in ("mode", "travel", "overwrite"):
                value = expected if kind == "mode" else 0
                if (set(kinds[side][kind]) != {"attempted_frames", "sent_frames"}
                        or any(not _allowed_integer(v, (value,)) for v in kinds[side][kind].values())):
                    raise ValueError("Prior mode-only receipt contains target or partial frames")
        events = prior.get("events", [])
        intents = [event for event in events if event.get("event") == "probe_send_intent"]
        completions = [event for event in events if event.get("event") == "probe_send_complete_unconfirmed"]
        if (len(intents) != 1 or len(completions) != 1 or intents[0].get("kind") != "mode"
                or intents[0].get("arm") != self.arm or intents[0].get("target_pose_m_rad") is not None
                or intents[0].get("frames") != [{"id": 0x151, "data_hex": "0102010000000000"}]
                or completions[0].get("kind") != "mode"):
            raise ValueError("Prior mode-only receipt must prove the one exact mode frame")
        state = result.get("after", {}).get(self.arm, {})
        status = state.get("arm_status", {})
        if any(not _allowed_integer(status.get(key), (value,)) for key, value in
               (("ctrl_mode", 1), ("arm_status", 4), ("mode_feedback", 2),
                ("motion_status", 1), ("teach_status", 0), ("err_code", 0))):
            raise ValueError("Prior receipt is not the exact mode-only target rejection")
        arms._six_finite(state.get("pose_m_rad"))
        arms._six_finite(state.get("joints_rad"))
        stamps = state.get("fragment_timestamps_s", {})
        if not stamps or any(type(v) not in (int, float) or not math.isfinite(v) for v in stamps.values()):
            raise ValueError("Prior mode-only receipt lacks complete feedback timestamps")
        health = arms.control_health(state, now_s=max(stamps.values()), require_enabled=True)
        if any(reason["field"] != "arm_status.arm_status" or reason["code"] != "arm_fault_or_unknown"
               for reason in health["reasons"]):
            raise ValueError("Prior rejection includes another fault or incomplete feedback")

    def rotation_distance(self, a, b):
        qa, qb = self.quaternion(*a[3:]), self.quaternion(*b[3:])
        norm = math.sqrt(sum(v * v for v in qa) * sum(v * v for v in qb))
        return 2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(qa, qb))) / norm))

    def connect(self):
        super().connect()
        from pyAgxArm.utiles.tf import euler_convert_quat
        from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET
        self.quaternion = euler_convert_quat
        self.joint_limits = {side: ROBOT_JOINT_LIMIT_PRESET[self.profile["arms"][side]["model"]]
                             for side in SIDES}
        self.report["joint_limits_source"] = "Manufacturer ROBOT_JOINT_LIMIT_PRESET; caller must verify controller limits agree"

    def orientation_error(self, pose):
        return self.rotation_distance(pose, self.origin_pose)

    def _wrap_comm(self, side, comm):
        original_send, original_callback = comm.send, comm.get_callback()
        def guarded_send(*args, **kwargs):
            with self.lock:
                ticket = self.ticket
                if (ticket is None or ticket["side"] != side or side != self.arm
                        or ticket["thread"] != threading.get_ident()
                        or comm.send_bus is not self.buses.get(side)):
                    self._deny(side, "TX outside bounded probe operation")
                ticket["comm_calls"] += 1
                if ticket["comm_calls"] > len(ticket["frames"]):
                    self._deny(side, "Extra SDK frame in bounded probe")
                result = original_send(*args, **kwargs)
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["bus_calls"] != ticket["comm_calls"]:
                    raise RuntimeError("SDK did not send exactly one underlying frame per call")
                return result
        def observed_callback(frame):
            if frame.arbitration_id in COMMAND_IDS:
                with self.lock:
                    self.violations.append({"side": side, "detail": "Other command frame observed",
                                            "arbitration_id": frame.arbitration_id,
                                            "data_hex": bytes(frame.data).hex()})
            if original_callback is not None:
                original_callback(frame)
        comm.send = guarded_send
        comm.set_callback(observed_callback)

    def _wrap_bus(self, side, comm):
        bus, original_send = comm.send_bus, comm.send_bus.send
        self.buses[side] = bus
        def guarded_send(frame, *args, **kwargs):
            with self.lock:
                ticket = self.ticket
                if (ticket is None or side != self.arm or ticket["side"] != side
                        or ticket["thread"] != threading.get_ident()
                        or comm.send_bus is not bus or self.violations):
                    self._deny(side, "Underlying TX outside bounded probe")
                index = ticket["bus_calls"]
                if index >= len(ticket["frames"]) or ticket["comm_calls"] != index + 1:
                    self._deny(side, "Too many probe frames")
                expected_id, expected = ticket["frames"][index]
                if (frame.arbitration_id != expected_id or frame.dlc != 8 or bytes(frame.data) != expected
                        or frame.is_extended_id or frame.is_remote_frame or frame.is_error_frame
                        or frame.is_fd or frame.bitrate_switch or frame.error_state_indicator
                        or self.counts[side]["attempted_frames"] >= 7):
                    self._deny(side, "Probe frame differs from exact SDK target sequence")
                ticket["bus_calls"] += 1
                self.counts[side]["attempted_frames"] += 1
                frame_kind = "mode" if ticket["kind"] == "travel" and expected_id == 0x151 else ticket["kind"]
                self.kind_counts[side][frame_kind]["attempted_frames"] += 1
                try:
                    result = original_send(frame, *args, **kwargs)
                except BaseException as exc:
                    ticket["error"] = exc
                    raise
                self.counts[side]["sent_frames"] += 1
                self.kind_counts[side][frame_kind]["sent_frames"] += 1
                return result
        bus.send = guarded_send

    def checked(self, states):
        if self.violations:
            raise RuntimeError("CAN ownership/TX violation: " + repr(self.violations))
        stamps = []
        active = self.phase in ("travel", "overwrite", "settle")
        for side in SIDES:
            state = states[side]
            health = arms.control_health(state, allowed_control_modes=(1,), require_enabled=False)
            status = state["arm_status"]
            baseline_rejection = self.prior is not None and side == self.arm and not active
            awaiting_acceptance = (self.prior is not None and side == self.arm and self.phase == "travel"
                                   and not self.mode_l_confirmed and self.travel_finished_monotonic is not None
                                   and time.monotonic()-self.travel_finished_monotonic <= PROBE["rejection_acceptance_s"])
            expected_rejection = (baseline_rejection or awaiting_acceptance) and _allowed_integer(status.get("arm_status"), (4,))
            reasons = [reason for reason in health["reasons"] if not
                       (expected_rejection and reason["code"] == "arm_fault_or_unknown"
                        and reason["field"] == "arm_status.arm_status")]
            if reasons:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, reasons))
            if baseline_rejection:
                if (not expected_rejection or not _allowed_integer(status.get("motion_status"), (1,))
                        or not _allowed_integer(status.get("mode_feedback"), (2,))):
                    raise RuntimeError("Saved rejection no longer matches current baseline")
                previous = self.prior["result"]["after"][side]
                if (max(abs(a-b) for a,b in zip(state["joints_rad"], previous["joints_rad"])) > LIMITS["joint_rad"]
                        or math.dist(state["pose_m_rad"][:3], previous["pose_m_rad"][:3]) > LIMITS["position_m"]
                        or abs(state["gripper"]["width_m"]-previous["gripper"]["width_m"]) > LIMITS["gripper_m"]
                        or self.rotation_distance(state["pose_m_rad"], previous["pose_m_rad"]) > PROBE["orientation_rad"]):
                    raise RuntimeError("Current arm differs from saved mode-only rejection")
            elif expected_rejection:
                # Receive timestamps prove freshness, not which target the
                # firmware planner processed. Permit only this brief, quiet
                # persistence of the recorded rejection, never a different fault.
                previous = self.dispatch_anchor
                if (max(abs(a-b) for a,b in zip(state["joints_rad"], previous["joints_rad"])) > LIMITS["joint_rad"]
                        or math.dist(state["pose_m_rad"][:3], previous["pose_m_rad"][:3]) > PROBE["acceptance_position_drift_m"]
                        or self.rotation_distance(state["pose_m_rad"], previous["pose_m_rad"]) > PROBE["orientation_rad"]):
                    raise RuntimeError("Arm moved before normal target acceptance was observed")
                self.report["prior_rejection_wait_observed_s"] = time.monotonic()-self.travel_finished_monotonic
                self.report["target_processing_state"] = "awaiting_target_processing"
            violations = [{"joint": i + 1, "actual_rad": value,
                           "bounds_rad": list(self.joint_limits[side]["joint%d" % (i + 1)])}
                          for i, value in enumerate(state["joints_rad"])
                          if not self.joint_limits[side]["joint%d" % (i + 1)][0] <= value
                          <= self.joint_limits[side]["joint%d" % (i + 1)][1]]
            if side != self.arm:
                # This arm never receives a frame. Its existing static limit
                # violation is evidence, not permission or a recovery request;
                # every other health/stationarity requirement remains intact.
                self.report["inactive_known_limit_violations"][side] = violations
            elif violations:
                raise RuntimeError("%s known joint limit violations: %r; no recovery in this tool" % (side, violations))
            _pose_frames(state["pose_m_rad"])
            moving_values = (1,) if baseline_rejection else ((0, 1) if active and side == self.arm else (0,))
            if not _allowed_integer(status.get("teach_status"), (0,)) or not _allowed_integer(status.get("motion_status"), moving_values):
                raise RuntimeError(side + " unexpected teaching/motion status")
            mode = status.get("mode_feedback")
            if not _allowed_integer(mode, (0, 1, 2)):
                raise RuntimeError(side + " unknown movement mode")
            if side in self.expected_modes:
                expected = self.expected_modes[side]
                allowed = (expected, 2) if active and side == self.arm and not self.mode_l_confirmed else (expected,)
                if mode not in allowed:
                    raise RuntimeError(side + " movement mode changed unexpectedly")
            if (side == self.arm and active and self.travel_sent_at is not None
                    and state["fragment_timestamps_s"]["arm_status"] > self.travel_sent_at):
                if mode != 2:
                    raise RuntimeError("MOVE_L mode not confirmed by new feedback")
                if status["arm_status"] == 0:
                    self.mode_l_confirmed = True
                    self.expected_modes[side] = 2
                    self.report["fresh_normal_feedback_observed"] = True
                    self.report["target_processing_state"] = "normal_feedback_observed"
            if any(state["drivers"][str(i)]["foc_status"].get("driver_enable_status") is not True for i in range(1, 7)):
                raise RuntimeError(side + " requires all six drivers enabled")
            grip_enabled = state["gripper"]["foc_status"].get("driver_enable_status")
            if type(grip_enabled) is not bool:
                raise RuntimeError(side + " gripper enable state unknown")
            if self.prior is not None and side == self.arm and grip_enabled is not True:
                raise RuntimeError("Rejected-target recovery requires enabled gripper")
            self.gripper_enable.setdefault(side, grip_enabled)
            if grip_enabled is not self.gripper_enable[side]:
                raise RuntimeError(side + " gripper enable state changed")
            stamps.extend(state["fragment_timestamps_s"].values())
            if self.anchor is not None:
                if not active or side != self.arm:
                    super().check_drift(side, state, self.anchor[side])
                else:
                    self.check_active(state)
        if max(stamps) - min(stamps) > arms.MAX_SKEW_S:
            raise RuntimeError("Cross-arm feedback skew exceeds limit")
        if active and time.time() - min(stamps) > PROBE["feedback_age_s"]:
            raise RuntimeError("Probe requires all feedback fragments within 50 ms")
        return states

    def check_active(self, state):
        pose, origin = state["pose_m_rad"], self.origin_pose
        delta = [pose[i] - origin[i] for i in range(3)]
        joint = max(abs(a - b) for a, b in zip(state["joints_rad"], self.anchor[self.arm]["joints_rad"]))
        width = abs(state["gripper"]["width_m"] - self.anchor[self.arm]["gripper"]["width_m"])
        self.max_drift[self.arm] = {"joint_rad": max(joint, self.max_drift[self.arm]["joint_rad"]),
                                    "position_m": max(math.dist(pose[:3], origin[:3]), self.max_drift[self.arm]["position_m"]),
                                    "gripper_m": max(width, self.max_drift[self.arm]["gripper_m"])}
        if (math.hypot(*delta[:2]) > PROBE["lateral_m"] or not -0.001 <= delta[2] <= 0.007
                or joint > PROBE["joint_excursion_rad"] or width > LIMITS["gripper_m"]
                or self.orientation_error(pose) > PROBE["orientation_rad"]):
            raise RuntimeError("Active arm left the bounded probe envelope")
        distance = math.dist(pose[:3], self.original_target[:3])
        self.report["minimum_original_target_distance_m"] = min(
            distance, self.report.get("minimum_original_target_distance_m", math.inf))
        if distance <= PROBE["original_separation_m"]:
            self.report["original_target_approached"] = True
            raise RuntimeError("Original target was approached; replacement not demonstrated")

    @staticmethod
    def all_after(state, stamp):
        return all(value > stamp for side in SIDES for value in state[side]["fragment_timestamps_s"].values())

    def overwrite_eligible(self, state):
        pose = state[self.arm]["pose_m_rad"]
        progress = pose[2] - self.dispatch_anchor["pose_m_rad"][2]
        remaining = self.original_target[2] - pose[2]
        return (self.mode_l_confirmed and state[self.arm]["arm_status"]["arm_status"] == 0
                and self.all_after(state, self.travel_sent_at) and progress >= PROBE["progress_m"]
                and remaining >= PROBE["remaining_m"])

    def validate_start(self, state):
        current = state[self.arm]
        fk = arms._six_finite(self.robots[self.arm].fk(current["joints_rad"][:]))
        pos = math.dist(fk[:3], current["pose_m_rad"][:3])
        rot = self.rotation_distance(fk, current["pose_m_rad"])
        if pos > LIMITS["position_m"] or rot > PROBE["orientation_rad"]:
            raise RuntimeError("Manufacturer FK does not agree with current flange feedback")
        self.report.update(fk_current_flange_m_rad=fk, fk_feedback_position_error_m=pos,
                           fk_feedback_rotation_error_rad=rot)

    def dispatch(self, kind, sdk_call):
        order = ("travel", "overwrite")
        if (len(self.operations) >= len(order)
                or kind != order[len(self.operations)] or kind in self.operations):
            raise RuntimeError("Invalid/repeated probe operation")
        target = self.original_target if kind == "travel" else self.hold_target
        frames = ([(0x151, bytes((1, 2, 1, 0, 0, 0, 0, 0)))] if kind == "travel" else []) + _pose_frames(target)
        self.emit("probe_send_intent", {"arm": self.arm, "kind": kind,
                                        "target_pose_m_rad": target,
                                        "frames": [{"id": can_id, "data_hex": data.hex()} for can_id, data in frames]})
        state = self.checked(self.read())
        if kind == "travel":
            self.validate_start(state)
            if math.dist(state[self.arm]["pose_m_rad"][:3], self.origin_pose[:3]) > PROBE["initial_sample_slip_m"]:
                raise RuntimeError("Initial pose changed during journal write; fixed target not adjusted")
            self.dispatch_anchor = copy.deepcopy(state[self.arm])
            self.report["initial_dispatch_feedback"] = copy.deepcopy(self.dispatch_anchor)
        if kind == "overwrite" and (not self.overwrite_eligible(state)
                or math.dist(state[self.arm]["pose_m_rad"][:3], self.hold_target[:3]) > PROBE["overwrite_sample_slip_m"]):
            raise RuntimeError("Overwrite window/sample expired before dispatch")
        self.phase = kind
        ticket = {"side": self.arm, "thread": threading.get_ident(), "kind": kind,
                  "frames": frames, "bus_calls": 0, "comm_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN violation before probe dispatch")
            self.operations.append(kind)
            self.ticket = ticket
            try:
                sdk_call()
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["comm_calls"] != len(frames) or ticket["bus_calls"] != len(frames):
                    raise RuntimeError("Incomplete SDK pose sequence; no retry")
            finally:
                self.ticket = None
        finished = time.time()
        if kind == "travel":
            self.travel_finished_monotonic = time.monotonic()
        self.emit("probe_send_complete_unconfirmed", {"kind": kind, "finished_unix_s": finished})
        return finished

    def perform(self):
        robot = self.robots[self.arm]
        robot.set_auto_set_motion_mode_enabled(True)  # Public API; in-memory only.
        cache = robot._msg_mode
        for key, expected in (("ctrl_mode", 1), ("mit_mode", 0), ("residence_time", 0), ("installation_pos", 0)):
            if not _allowed_integer(getattr(cache, key, None), (expected,)):
                raise RuntimeError("Unexpected cached SDK mode field " + key)
        cache.move_spd_rate_ctrl = 1
        state = self.checked(self.read())
        self.validate_start(state)
        self.phase = "ready"
        self.origin_pose = list(state[self.arm]["pose_m_rad"])
        self.original_target = self.origin_pose[:]
        self.original_target[2] += PROBE["travel_m"]
        _pose_frames(self.original_target)  # Reject, never permit SDK clamping.
        self.report.update(origin_pose_m_rad=self.origin_pose[:], original_target_m_rad=self.original_target[:])
        self.travel_sent_at = self.dispatch("travel", lambda: robot.move_l(self.original_target[:]))
        robot.set_auto_set_motion_mode_enabled(False)  # No mode frame in replacement.
        deadline = time.monotonic() + PROBE["timeout_s"]
        while time.monotonic() < deadline:
            state = self.checked(self.read())
            if self.original_target[2] - state[self.arm]["pose_m_rad"][2] < PROBE["remaining_m"]:
                raise RuntimeError("Missed replacement window; no retry or extra target")
            if self.overwrite_eligible(state):
                self.hold_target = state[self.arm]["pose_m_rad"][:3] + self.origin_pose[3:]
                self.report["overwrite_source_feedback"] = copy.deepcopy(state[self.arm])
                self.report["hold_target_m_rad"] = self.hold_target[:]
                self.overwrite_sent_at = self.dispatch("overwrite", lambda: robot.move_l(self.hold_target[:]))
                self.report["overwrite_sent"] = True
                self.phase = "settle"
                return self.verify_settled(deadline)
            time.sleep(PROBE["poll_s"])
        raise RuntimeError("No qualifying motion within 10 seconds; hold unproven")

    def verify_settled(self, deadline):
        stable_start, low, high, qlow, qhigh, advances, previous = None, None, None, None, None, 0, None
        while time.monotonic() < deadline:
            state = self.checked(self.read())
            active = state[self.arm]
            pose, joints = active["pose_m_rad"], active["joints_rad"]
            qualifies = (self.all_after(state, self.overwrite_sent_at)
                         and active["arm_status"]["motion_status"] == 0
                         and math.dist(pose[:3], self.hold_target[:3]) <= PROBE["hold_tolerance_m"])
            if not qualifies:
                stable_start = None
            elif stable_start is None:
                stable_start = time.monotonic()
                low, high, qlow, qhigh = pose[:3], pose[:3], joints[:], joints[:]
                advances, previous = 0, state
            else:
                low = [min(a, b) for a, b in zip(low, pose[:3])]
                high = [max(a, b) for a, b in zip(high, pose[:3])]
                qlow = [min(a, b) for a, b in zip(qlow, joints)]
                qhigh = [max(a, b) for a, b in zip(qhigh, joints)]
                if math.dist(low, high) > PROBE["stable_span_m"] or max(b - a for a, b in zip(qlow, qhigh)) > LIMITS["joint_rad"]:
                    stable_start = None
                elif self.advanced(previous, state):
                    advances += 1
                    previous = state
                    if time.monotonic() - stable_start >= PROBE["stable_s"] and advances >= 20:
                        self.report.update(qualified=True, local_hold_observed=True,
                                           prior_rejection_recovery_observed=self.prior is not None,
                                           stable_duration_s=time.monotonic() - stable_start,
                                           stable_position_span_m=math.dist(low, high),
                                           stable_feedback_advances=advances)
                        self.emit("local_linear_hold_observed", {"arm": self.arm,
                                  "hold_target_m_rad": self.hold_target, "scope": self.report["hold_scope"]})
                        return "local_linear_hold_observed_not_general_stop_certified"
            time.sleep(PROBE["poll_s"])
        raise RuntimeError("No separated stable 3-second hold within 10 seconds")

    def run(self):
        report = super().run()
        if not report["ok"]:
            report["qualified"] = False
            report["local_hold_observed"] = False
            report["prior_rejection_recovery_observed"] = False
            if self.counts[self.arm]["attempted_frames"] and not report["fresh_normal_feedback_observed"]:
                report["target_processing_state"] = "not_confirmed"
        report["target_commands_sent"] = sum(self.kind_counts[self.arm][kind]["sent_frames"]
                                               for kind in ("travel", "overwrite"))
        report["target_calls_sent"] = sum(self.kind_counts[self.arm][kind]["sent_frames"] == 3
                                            for kind in ("travel", "overwrite"))
        return report


def qualify_linear_hold(profile, journal_callback, arm="right", prior_mode_only_record=None):
    """Fixed +6 mm probe and one replacement; caller must ensure full-path clearance.

    Holds no production motion permission and never modifies SDK/site policy.
    Both arm states are monitored; only the selected arm can receive seven
    whitelisted frames. Partial transmission is not retried or rolled back.
    """
    if arm not in SIDES:
        raise ValueError("arm must be left or right")
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    return _LinearHold(profile, journal_callback, arm, prior_mode_only_record).run()
