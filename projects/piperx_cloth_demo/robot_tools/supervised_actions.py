"""One supervised SDK action; caller reviews the entire segment and residual space.

No task perception, IK, retries, stop, target replacement or automatic next step.
Feedback envelopes detect problems after motion; they do not prove a safe IK path.
The caller owns the exclusive device lock and a synchronous durable journal.
"""
import copy
import math
import threading
import time

from . import arms
from .linear_hold import _LinearHold, _pose_frames
from .takeover import LIMITS, SIDES, _Takeover, _allowed_integer

BOUNDS = {"translation_m": 0.030, "rotation_rad": 0.05, "speed_percent": 1,
          "stable_s": 3.0, "position_span_m": 0.0005, "joint_span_rad": 0.003,
          "jaw_span_m": 0.0005, "timeout_s": 20.0, "poll_s": 0.01,
          "feedback_age_s": 0.05, "joint_excursion_rad": 0.15,
          "segment_margin_m": 0.003, "rotation_margin_rad": 0.02}


class _SupervisedAction(_LinearHold):
    FRAME_KINDS = ("action",)
    SCOPE = "One explicitly selected supervised Cartesian or jaw target"

    def __init__(self, profile, journal, arm, kind, target):
        _Takeover.__init__(self, profile, journal)
        self.arm, self.kind, self.target = arm, kind, target
        self.active, self.dispatched, self.sent_at = False, False, None
        self.mode_l_confirmed = False
        self.quaternion, self.joint_limits = None, {}
        self.report.update(operation="supervised_" + kind, arm=arm, bounds=dict(BOUNDS),
                           requested_target=copy.deepcopy(target), controller_at_target=None,
                           controller_at_target_scope="Arm motion_status, not jaw contact or grasp",
                           pose_error=None, observed_stable=False, raw_motion_status={},
                           general_stop_validated=False, grasp_verified=False,
                           manufacturer_ik_path_verified=False, collision_path_verified=False,
                           full_segment_and_residual_clearance_review_required=True,
                           force_calibrated=False, nominal_force_N=0.2 if kind == "gripper" else None,
                           automatic_retry=False, target_calls_sent=0,
                           ok_semantics="One dispatch and bounded observation completed; inspect feedback separately")

    def connect(self):
        _LinearHold.connect(self)
        if self.kind == "gripper":
            gripper = self.grippers[self.arm]
            sender = getattr(type(gripper), "_send_msg", None)
            if not callable(sender):
                raise RuntimeError("Manufacturer effector encoder unavailable")
            gripper._send_msg = sender.__get__(gripper, type(gripper))

    def checked(self, states):
        if self.violations:
            raise RuntimeError("CAN ownership/TX guard violation: " + repr(self.violations))
        stamps = []
        for side in SIDES:
            state, status = states[side], states[side]["arm_status"]
            health = arms.control_health(state, allowed_control_modes=(1,), require_enabled=True)
            if not health["healthy"]:
                raise RuntimeError("Unhealthy %s feedback: %r" % (side, health["reasons"]))
            if (not _allowed_integer(status.get("teach_status"), (0,))
                    or not _allowed_integer(status.get("motion_status"), (0, 1))
                    or not _allowed_integer(status.get("mode_feedback"), (0, 1, 2))):
                raise RuntimeError(side + " unknown motion mode/status or active teaching")
            for i, q in enumerate(state["joints_rad"], 1):
                low, high = self.joint_limits[side]["joint%d" % i]
                if not low <= q <= high:
                    raise RuntimeError("%s joint%d outside strict manufacturer limits" % (side, i))
            _pose_frames(state["pose_m_rad"])
            if not 0 <= state["gripper"]["width_m"] <= 0.070:
                raise RuntimeError(side + " jaw width outside physical feedback range")
            mode = status["mode_feedback"]
            moving = self.active and self.kind == "move" and side == self.arm
            if side in self.expected_modes:
                allowed = (self.expected_modes[side], 2) if moving and not self.mode_l_confirmed else (self.expected_modes[side],)
                if mode not in allowed:
                    raise RuntimeError(side + " unexpected movement mode change")
            if moving and self.sent_at is not None and state["fragment_timestamps_s"]["arm_status"] > self.sent_at:
                if mode != 2:
                    raise RuntimeError("New feedback did not confirm MOVE_L mode")
                self.mode_l_confirmed, self.expected_modes[side] = True, 2
            stamps.extend(state["fragment_timestamps_s"].values())
            if self.anchor is not None:
                self.check_envelope(side, state, moving)
        if max(stamps) - min(stamps) > arms.MAX_SKEW_S:
            raise RuntimeError("Cross-arm feedback skew exceeds limit")
        if time.time() - min(stamps) > BOUNDS["feedback_age_s"]:
            raise RuntimeError("All feedback fragments must be within 50 ms")
        return states

    def check_envelope(self, side, state, moving):
        anchor = self.anchor[side]
        pose, start = state["pose_m_rad"], anchor["pose_m_rad"]
        qdelta = max(abs(a-b) for a, b in zip(state["joints_rad"], anchor["joints_rad"]))
        distance = math.dist(pose[:3], start[:3])
        width, width0 = state["gripper"]["width_m"], anchor["gripper"]["width_m"]
        for name, value in (("joint_rad", qdelta), ("position_m", distance), ("gripper_m", abs(width-width0))):
            self.max_drift[side][name] = max(self.max_drift[side][name], value)
        if moving:
            vector = [b-a for a, b in zip(start[:3], self.target[:3])]
            length2 = sum(v*v for v in vector)
            fraction = max(0, min(1, sum((pose[i]-start[i])*vector[i] for i in range(3))/length2)) if length2 else 0
            nearest = [start[i]+fraction*vector[i] for i in range(3)]
            if (qdelta > BOUNDS["joint_excursion_rad"] or math.dist(pose[:3], nearest) > BOUNDS["segment_margin_m"]
                    or self.rotation_distance(pose, start) > self.rotation_distance(self.target, start)+BOUNDS["rotation_margin_rad"]):
                raise RuntimeError("Selected arm left observed segment/joint envelope; no stop command available")
        elif qdelta > BOUNDS["joint_span_rad"] or distance > BOUNDS["position_span_m"]:
            raise RuntimeError(side + " uncommanded arm drift exceeds stationary envelope")
        if self.active and self.kind == "gripper" and side == self.arm:
            low, high = sorted((width0, self.target))
            if not low-0.002 <= width <= high+0.002:
                raise RuntimeError("Selected jaw left requested width interval")
        elif abs(width-width0) > LIMITS["gripper_m"]:
            raise RuntimeError(side + " uncommanded jaw drift")

    @staticmethod
    def spans(samples):
        result = {}
        for side in SIDES:
            poses = [sample[side]["pose_m_rad"][:3] for sample in samples]
            joints = [sample[side]["joints_rad"] for sample in samples]
            widths = [sample[side]["gripper"]["width_m"] for sample in samples]
            result[side] = {"position_m": math.sqrt(sum((max(x)-min(x))**2 for x in zip(*poses))),
                            "joint_rad": max(max(x)-min(x) for x in zip(*joints)),
                            "jaw_m": max(widths)-min(widths)}
        return result

    @staticmethod
    def stable(spans):
        return all(s["position_m"] <= BOUNDS["position_span_m"] and s["joint_rad"] <= BOUNDS["joint_span_rad"]
                   and s["jaw_m"] <= BOUNDS["jaw_span_m"] for s in spans.values())

    def prepare(self):
        deadline = time.monotonic()+LIMITS["warmup_s"]
        while True:
            state = self.read()
            if all(state[side].get("status") == "complete" for side in SIDES):
                self.checked(state)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("No complete fresh dual-arm feedback")
            time.sleep(BOUNDS["poll_s"])
        self.anchor = copy.deepcopy(state)
        self.expected_modes = {s: state[s]["arm_status"]["mode_feedback"] for s in SIDES}
        self.report["before"] = copy.deepcopy(state)
        start, samples, previous, advances = time.monotonic(), [state], state, 0
        while time.monotonic()-start < BOUNDS["stable_s"]:
            time.sleep(BOUNDS["poll_s"])
            state = self.checked(self.read())
            samples.append(state)
            if self.advanced(previous, state):
                previous, advances = state, advances+1
        spans = self.spans(samples)
        if advances < 20 or not self.stable(spans):
            raise RuntimeError("Baseline lacks three seconds of independently fresh stable feedback")
        self.report.update(baseline_duration_s=time.monotonic()-start, baseline_spans=spans,
                           baseline_feedback_advances=advances)
        self.record_outcome(state, False)

    def validate_move(self, state):
        self.validate_start(state)  # Manufacturer FK, never an external IK solver.
        pose = state[self.arm]["pose_m_rad"]
        if (math.dist(pose[:3], self.target[:3]) > BOUNDS["translation_m"]
                or self.rotation_distance(pose, self.target) > BOUNDS["rotation_rad"]):
            raise RuntimeError("Explicit Cartesian target exceeds 30 mm / 0.05 rad; not adjusted")

    def dispatch(self):
        if self.dispatched:
            raise RuntimeError("Only one target call permitted")
        robot = self.robots[self.arm]
        if self.kind == "move":
            for key, expected in (("ctrl_mode", 1), ("mit_mode", 0), ("residence_time", 0), ("installation_pos", 0)):
                if not _allowed_integer(getattr(robot._msg_mode, key, None), (expected,)):
                    raise RuntimeError("Unexpected SDK mode cache " + key)
            robot.set_auto_set_motion_mode_enabled(True)  # In-memory only.
            robot._msg_mode.move_spd_rate_ctrl = 1
            frames = [(0x151, bytes((1, 2, 1, 0, 0, 0, 0, 0)))] + _pose_frames(self.target)
            sdk_call = lambda: robot.move_l(self.target[:])
        else:
            frames = [(0x159, round(self.target*1e6).to_bytes(4, "big", signed=True)+bytes((0, 200, 1, 0)))]
            sdk_call = lambda: self.grippers[self.arm].move_gripper_m(value=self.target, force=0.2)
        self.emit("supervised_action_intent", {"arm": self.arm, "kind": self.kind, "target": self.target,
                   "frames": [{"id": i, "data_hex": data.hex()} for i, data in frames]})
        state = self.checked(self.read())
        if self.kind == "move":
            self.validate_move(state)
        self.anchor = copy.deepcopy(state)
        self.report["dispatch_feedback"] = copy.deepcopy(state)
        ticket = {"side": self.arm, "thread": threading.get_ident(), "kind": "action", "frames": frames,
                  "comm_calls": 0, "bus_calls": 0, "error": None}
        with self.lock:
            if self.violations:
                raise RuntimeError("CAN violation before dispatch")
            self.active, self.dispatched, self.ticket = True, True, ticket
            try:
                sdk_call()
                if ticket["error"] is not None:
                    raise RuntimeError("CAN send failed: " + str(ticket["error"])) from ticket["error"]
                if ticket["comm_calls"] != len(frames) or ticket["bus_calls"] != len(frames):
                    raise RuntimeError("Incomplete SDK target sequence; no retry")
            finally:
                self.ticket = None
        self.sent_at = time.time()
        self.report["target_calls_sent"] = 1
        self.emit("supervised_action_sent_unconfirmed", {"finished_unix_s": self.sent_at, "kind": self.kind})

    def record_outcome(self, state, stable):
        pose = state[self.arm]["pose_m_rad"]
        goal = self.target if self.kind == "move" else self.anchor[self.arm]["pose_m_rad"]
        self.report.update(controller_at_target=state[self.arm]["arm_status"]["motion_status"] == 0,
                           raw_motion_status={s: state[s]["arm_status"]["motion_status"] for s in SIDES},
                           pose_error={"position_m": math.dist(pose[:3], goal[:3]),
                                       "rotation_rad": self.rotation_distance(pose, goal)},
                           observed_stable=stable)
        if self.kind == "gripper":
            self.report["width_error_m"] = abs(state[self.arm]["gripper"]["width_m"]-self.target)
            self.report["observed_width_m"] = state[self.arm]["gripper"]["width_m"]

    def perform(self):
        self.dispatch()
        deadline, samples, began = time.monotonic()+BOUNDS["timeout_s"], [], None
        previous, advances = None, 0
        while time.monotonic() < deadline:
            state = self.checked(self.read())
            self.record_outcome(state, False)
            if self.all_after(state, self.sent_at):
                samples.append(state)
                if began is None:
                    began, previous = time.monotonic(), state
                elif self.advanced(previous, state):
                    previous, advances = state, advances+1
                spans = self.spans(samples)
                if not self.stable(spans):
                    samples, began, previous, advances = [state], time.monotonic(), state, 0
                elif time.monotonic()-began >= BOUNDS["stable_s"] and advances >= 20:
                    self.record_outcome(state, True)
                    self.report.update(observed_stable_duration_s=time.monotonic()-began, observed_spans=spans,
                                       observed_feedback_advances=advances)
                    return "observed_stable_feedback_not_grasp_or_stop_certification"
            time.sleep(BOUNDS["poll_s"])
        raise RuntimeError("Observation timeout; no further target or stop sent; physical state requires supervision")

    def run(self):
        report = _Takeover.run(self)
        report["target_commands_sent"] = max(0, report["hardware_commands_sent"]-int(self.kind == "move"))
        if not report["ok"]:
            report["observed_stable"] = False
        return report


def move_once(profile, journal, arm, target_pose_m_rad):
    """One 1% MOVE_L in the selected base frame, flange reference; max 30 mm.

    Caller reviews this complete segment, attachments and possible residual motion.
    Four ordered frames are not atomic; mode/partial dispatch may expose old targets.
    """
    if arm not in SIDES or not callable(journal):
        raise ValueError("Explicit arm and synchronous journal required")
    target = arms._six_finite(target_pose_m_rad)
    _pose_frames(target)
    return _SupervisedAction(profile, journal, arm, "move", target).run()


def gripper_once(profile, journal, arm, width_m, nominal_force_N):
    """One enabled-jaw width target, 0..55 mm, nominal SDK force exactly 0.2.

    Contact force is uncalibrated; width feedback alone never proves a grasp.
    """
    if arm not in SIDES or not callable(journal):
        raise ValueError("Explicit arm and synchronous journal required")
    if (type(width_m) not in (int, float) or not math.isfinite(width_m) or not 0 <= width_m <= 0.055
            or type(nominal_force_N) not in (int, float) or nominal_force_N != 0.2):
        raise ValueError("Width must be 0..0.055 m and nominal SDK force exactly 0.2")
    return _SupervisedAction(profile, journal, arm, "gripper", float(width_m)).run()
