"""Same-connection supervised first MOVE_J, never an unknown-cache proof.

Only the owning PairHost calls this adapter after a durable event claim and
source/scene admission. It adopts no measured q as old target history. The four
non-atomic frames remain an attended initialization, with no hold/stop fallback.
"""
import copy
import math
import threading
import time

from . import arms
from .joint_initialization import plan_joint_initialization, validate_joint_initialization_sample
from .model_compatibility import fk_matrix, matrix_error
from .pair_joint_adapter import checked_identity, pair_sample, _finite, _identifier
from .single_supervised_actions import BOUNDS
from .takeover import LIMITS, SIDES, _allowed_integer
from .tracking_observation import TrackingObservation


def _observe(device):
    try:
        if device._task_ready:
            return device._observe()
        from .pair_preparation import observe_preparation
        return observe_preparation(device)
    except BaseException as exc:
        device._fault = str(exc)
        raise


class _Initializer:
    def __init__(self, device, context, event_id, deadline_at):
        self.device, self.action = device, device._action
        self.identity = checked_identity(device, context["identity"])
        self.context = copy.deepcopy(context)
        self.event_id, self.deadline = event_id, deadline_at
        self.thread = threading.get_ident()
        self.last_time = time.time()
        self.last_stamps = None
        self.last_sample = None
        self.plan = None
        self.frames = []
        self.mode_confirmed = False
        self.original_anchor = copy.deepcopy(self.action.idle_anchor)
        self.completed_dispatch = None
        self.tracking = TrackingObservation("Initialization")

    def now(self):
        now = time.time()
        if not _finite(now) or now < self.last_time or now >= self.deadline:
            raise RuntimeError("Initialization clock invalid, regressed or original deadline reached")
        self.last_time = now
        return now

    def encoder_guard(self):
        if threading.get_ident() != self.thread or self.action.violations:
            raise RuntimeError("Initialization worker or CAN ownership changed")
        self.now()
        self.action.guard()
        self.now()  # Include a potentially slow durable host guard.

    def final_fresh(self, sample):
        now = self.now()
        if any(not 0 <= now-stamp <= .05 for side in SIDES
               for stamp in sample["arms"][side]["fragment_timestamps_s"].values()):
            raise RuntimeError("Initialization feedback exceeds 50 ms including processing")

    def check_states(self, states):
        if self.tracking.first_failure is not None:
            raise RuntimeError("Initialization observation failure remains latched")
        sample = pair_sample(self.identity, states, now=self.now())
        selected = self.identity["arm"]
        # active is set before the SDK call, but the first frame's delayed
        # preflight must still satisfy the original stationary/P-mode state.
        active = self.action.active and bool(self.frames)
        settling = (active and self.completed_dispatch is not None
                    and self.plan["tracking_policy"]["mode"] == "bounded_postsend_settling")
        phase = "settling" if settling else "active" if active else "pre_dispatch"
        tracking = self.track_sample(sample)
        try:
            observation = validate_joint_initialization_sample(self.plan, sample, now=self.now(),
                phase=phase, mode_confirmed=self.mode_confirmed)
            if settling:
                self.account_tracking_time(sample, tracking)
                observation["tracking"].update(cumulative_time_checked=True,
                    cumulative_outside_nominal_band_s=self.tracking.cumulative_s)
        except BaseException as exc:
            self.record_failure(exc, sample, tracking)
            raise
        # The planner freezes its device-issued origin. In addition, all
        # uncommanded components keep the connection's older stationary anchor.
        for side in SIDES:
            state, anchor = states[side], self.original_anchor[side]
            if not (active and side == selected):
                position_limit = BOUNDS["position_span_m"] if self.device._task_ready else LIMITS["position_m"]
                if (max(abs(a-b) for a,b in zip(state["joints_rad"], anchor["joints_rad"])) > .003
                        or math.dist(state["pose_m_rad"][:3], anchor["pose_m_rad"][:3]) > position_limit
                        or self.action.rotation_distance(state["pose_m_rad"], anchor["pose_m_rad"]) > .003):
                    raise RuntimeError(side + " initialization changed an uncommanded persistent arm anchor")
            if abs(state["gripper"]["width_m"]-anchor["gripper"]["width_m"]) > LIMITS["gripper_m"]:
                raise RuntimeError(side + " initialization changed the persistent jaw anchor")
            observer = self.device._preparation
            if observer is not None:
                if side in observer.prepared_targets and abs(
                        state["gripper"]["width_m"]-observer.prepared_targets[side]) > .001:
                    raise RuntimeError(side + " initialization departed from its prepared jaw target")
        stamps = {(s, name): stamp for s in SIDES for name, stamp in
                  states[s]["fragment_timestamps_s"].items()}
        if self.last_stamps is not None and any(stamp < self.last_stamps[key] for key, stamp in stamps.items()):
            raise RuntimeError("Initialization feedback fragment regressed")
        self.final_fresh(sample)
        self.last_stamps, self.last_sample = stamps, sample
        self.action.report["initialization_observation"] = observation
        if (self.action.sent_at is not None and states[selected]["arm_status"]["mode_feedback"] == 1
                and states[selected]["fragment_timestamps_s"]["arm_status"] > self.action.sent_at):
            self.mode_confirmed = True
        return states

    def track_sample(self, sample):
        return self.tracking.track(self.plan, sample)

    def account_tracking_time(self, sample, tracking):
        """One event's conservative RX-only budget; never reset on reentry."""
        at = sample["captured_at"]
        proof = self.completed_dispatch
        if (proof is None or proof["comm_calls"] != 4 or proof["bus_calls"] != 4
                or len(self.frames) != 4 or self.action.sent_at != proof["sent_at"]
                or self.action.ticket is not None or self.tracking.previous_at is None
                or not proof["sent_at"] <= self.tracking.previous_at <= at):
            raise RuntimeError("Settling requires this event's complete four-frame dispatch")
        self.tracking.account(sample, tracking,
            maximum_s=self.plan["tracking_policy"]["max_cumulative_outside_band_s"])

    def record_failure(self, exc, sample=None, tracking=None):
        self.tracking.record_failure(exc, sample, tracking)

    def read(self, *, journal=True):
        self.encoder_guard()
        states = self.action.read() if journal else {
            s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES}
        self.check_states(states)
        return self.last_sample

    def send_frame(self, side, original, frame, *args, **kwargs):
        self.encoder_guard()
        ticket = self.action.ticket
        if (ticket is None or side != self.identity["arm"] or ticket["side"] != side
                or ticket["thread"] != self.thread):
            self.action._deny(side, "Initialization TX outside original selected-worker ticket")
        index = ticket["bus_calls"]
        if index >= 4:
            self.action._deny(side, "Initialization permits only four frames")
        sample = self.read(journal=False)
        self.final_fresh(sample)
        # Inner existing bus wrappers enforce exact bytes/flags/sequence and
        # count attempts, including a transport exception. No I/O follows RX.
        value = original(frame, *args, **kwargs)
        self.frames.append({"frame": copy.deepcopy(self.plan["frames"][index]),
                            "outcome": "returned", "returned_at": self.now()})
        return value

    def dispatch(self):
        action, robot = self.action, self.action.robots[self.identity["arm"]]
        if action.dispatched:
            raise RuntimeError("Initialization was already attempted; no retry")
        self.encoder_guard()
        for key, expected in (("ctrl_mode", 1), ("mit_mode", 0), ("installation_pos", 0), ("residence_time", 0)):
            if not _allowed_integer(getattr(robot._msg_mode, key, None), (expected,)):
                raise RuntimeError("Unexpected SDK mode cache " + key)
        robot.set_auto_set_motion_mode_enabled(True)  # Both mutations are memory-only.
        robot._msg_mode.move_spd_rate_ctrl = 1
        ticket = {"side": self.identity["arm"], "thread": self.thread, "kind": "action",
                  "frames": [(f["arbitration_id"], bytes.fromhex(f["data_hex"])) for f in self.plan["frames"]],
                  "comm_calls": 0, "bus_calls": 0, "error": None}
        with action.lock:
            action.active, action.dispatched, action.ticket = True, True, ticket
            try:
                robot.move_j(self.plan["requested_target_joints_rad"][:])
                if ticket["error"] is not None:
                    raise RuntimeError("Initialization CAN send failed") from ticket["error"]
                if (type(ticket["comm_calls"]) is not int or ticket["comm_calls"] != 4
                        or type(ticket["bus_calls"]) is not int or ticket["bus_calls"] != 4 or len(self.frames) != 4):
                    raise RuntimeError("Initialization four-frame sequence incomplete; no retry")
            finally:
                action.ticket = None
        action.sent_at = self.now()
        self.completed_dispatch = {"comm_calls": ticket["comm_calls"], "bus_calls": ticket["bus_calls"],
                                   "sent_at": action.sent_at}
        self.tracking.start(action.sent_at)
        action.report["target_calls_sent"] = 1

    def window(self, *, arrival=False):
        began, previous, window, advances = None, None, None, 0
        deadline = time.monotonic()+10.
        while time.monotonic() < deadline:
            sample = self.read()
            states = sample["arms"]
            state = states[self.identity["arm"]]
            qualifies = True
            if arrival:
                fk = fk_matrix(self.plan["model"]["mdh"], state["joints_rad"])
                error = matrix_error(fk, self.plan["model_target_flange_transform"])
                self.action.report["model_arrival_error"] = error
                qualifies = (self.action.all_after(states, self.action.sent_at) and self.mode_confirmed
                    and state["arm_status"]["motion_status"] == 0
                    and max(abs(a-b) for a,b in zip(state["joints_rad"], self.plan["encoded_target_joints_rad"])) <= .003
                    and error["position_error_m"] <= .002 and error["so3_error_rad"] <= .02)
            if not qualifies:
                began, previous, window, advances = None, None, None, 0
            elif window is None:
                began, previous, advances = time.monotonic(), states, 0
                window = self.action.new_window(states)
            elif not self.action.extend_window(window, states):
                if not arrival:
                    raise RuntimeError("Initialization baseline lost three-second stability")
                began, previous, advances = time.monotonic(), states, 0
                window = self.action.new_window(states)
            elif self.action.advanced(previous, states):
                previous, advances = states, advances+1
            if began is not None and time.monotonic()-began >= 3. and advances >= 20:
                self.final_fresh(sample)
                return sample, time.monotonic()-began, advances, window
            time.sleep(BOUNDS["poll_s"])
        raise RuntimeError("Initialization lacks a fresh three-second/20-advance " + ("arrival" if arrival else "baseline"))


def initialize_joint_target(device, arm, *, context, event_id, deadline_at):
    if arm not in SIDES:
        raise ValueError("Explicit initialization arm required")
    _identifier(event_id, "initialization event id")
    if not _finite(deadline_at) or deadline_at <= time.time():
        raise ValueError("Original finite initialization deadline required")
    device._connected_usable()
    action = device._action
    if any(action.grasps.values()):
        raise RuntimeError("Initialization requires both arms free of candidate/retained grasps")
    if device._joint_cache[arm] is not None:
        try:
            sample = _observe(device)
        except BaseException:
            device._joint_cache[arm] = None
            raise
        return {"ok": True, "status": "already_initialized", "arm": arm, "cache_established": True,
                "hardware_commands_sent": 0, "passive_arm_commands_sent": 0, "gripper_commands_sent": 0,
                "accepted": None, "physical_stop_verified": None, "sample": sample,
                "cached_target": copy.deepcopy(device._joint_cache[arm]),
                "frame_receipts": copy.deepcopy(device._joint_cache[arm]["frame_receipts"]),
                "initialization_source": copy.deepcopy(device._joint_initializations[arm])}
    if type(context) is not dict:
        raise ValueError("Host-resolved initialization context required for an unknown cache")
    identity = checked_identity(device, context["identity"])
    if identity["arm"] != arm:
        raise ValueError("Initialization arm differs from its source identity")
    if identity["worker_id"] != event_id:
        raise ValueError("Initialization worker identity differs from original event id")
    origin = context.get("origin")
    if (type(origin) is not dict or device._initialization_observations.get(origin.get("sample_id")) != origin):
        raise RuntimeError("Initialization origin was not issued by this live device")
    before = _observe(device)["arms"]
    runner = _Initializer(device, context, event_id, deadline_at)
    action.reset_action(arm, "move", origin["arms"][arm]["joints_rad"][:])
    action.report.update(operation="initialize_joint_target", event_id=event_id, before=copy.deepcopy(before))
    try:
        runner.context["current"] = pair_sample(identity, before, now=runner.now())
        runner.plan = plan_joint_initialization(runner.context, now=runner.now())
        action.target = runner.plan["requested_target_joints_rad"][:]
        action.report["initialization_plan"] = copy.deepcopy(runner.plan)
        action.auxiliary_executor = runner
        for side in SIDES:
            action.boundary_origins.setdefault(side, runner.original_anchor[side]["joints_rad"][:])
        baseline, duration, advances, _ = runner.window()
        action.report.update(baseline_duration_s=duration, baseline_feedback_advances=advances)
        action.emit("pair_joint_initialization_intent", {"event_id": event_id, "plan": runner.plan})
        runner.dispatch()
        action.emit("pair_joint_initialization_dispatched_unconfirmed", {"event_id": event_id,
                    "sent_at": action.sent_at, "frame_receipts": copy.deepcopy(runner.frames)})
        final, duration, advances, window = runner.window(arrival=True)
        # Persist evidence before publishing usable local history, then check a
        # genuinely current sample; logging cannot refresh the old sample time.
        action.emit("pair_joint_initialization_observed", {"event_id": event_id,
                    "sample": final, "duration_s": duration, "feedback_advances": advances})
        final = runner.read(journal=False)
        if not action.extend_window(window, final["arms"]):
            raise RuntimeError("Initialization moved while persisting its completion")
        state = final["arms"][arm]
        if (not action.all_after(final["arms"], action.sent_at) or not runner.mode_confirmed
                or state["arm_status"]["mode_feedback"] != 1 or state["arm_status"]["motion_status"] != 0
                or max(abs(a-b) for a,b in zip(state["joints_rad"], runner.plan["encoded_target_joints_rad"])) > .003):
            raise RuntimeError("Initialization arrival changed while persisting completion")
        arrival = matrix_error(fk_matrix(runner.plan["model"]["mdh"], state["joints_rad"]),
                               runner.plan["model_target_flange_transform"])
        if arrival["position_error_m"] > .002 or arrival["so3_error_rad"] > .02:
            raise RuntimeError("Initialization model arrival changed while persisting completion")
        limits = runner.plan["effective_joint_limits_rad"][arm]
        strict = all(lo <= q <= hi for q,(lo,hi) in zip(state["joints_rad"], limits))
        tolerance = all(lo-.003 <= q <= hi+.003 for q,(lo,hi) in zip(state["joints_rad"], limits))
        cache = {"event_id": event_id, "identity": copy.deepcopy(identity),
                 "target_raw": runner.plan["target_raw"][:], "frame_receipts": copy.deepcopy(runner.frames)}
        source = {"schema": "piper_pair_joint_initialization_receipt_v1", "event_id": event_id,
                  "identity": copy.deepcopy(identity), "purpose": runner.plan["purpose"],
                  "origin": copy.deepcopy(origin), "target_raw": cache["target_raw"][:],
                  "encoded_target_joints_rad": runner.plan["encoded_target_joints_rad"][:],
                  "completion_sample": copy.deepcopy(final), "plan_sha256": runner.plan["plan_sha256"],
                  "effective_joint_limits_rad": copy.deepcopy(runner.plan["effective_joint_limits_rad"]),
                  "frame_receipts": copy.deepcopy(runner.frames), "strict_nominal": strict,
                  "within_feedback_tolerance": tolerance, "completed_at": runner.now(),
                  "ordinary_motion_authorized": False, "physical_stop_verified": None}
        runner.final_fresh(final)
        # Commit only the commanded arm's q/pose/mode; no peer/jaw reanchoring.
        for anchor in (action.idle_anchor, action.anchor):
            if anchor is not None:
                for key in ("joints_rad", "pose_m_rad"):
                    anchor[arm][key] = copy.deepcopy(state[key])
                anchor[arm]["arm_status"]["mode_feedback"] = 1
        action.expected_modes[arm] = 1
        if device._preparation is not None:
            observer = device._preparation
            for key in ("joints_rad", "pose_m_rad"):
                observer.anchor[arm][key] = copy.deepcopy(state[key])
            observer.anchor[arm]["arm_status"]["mode_feedback"] = 1
            observer.expected_modes[arm] = 1
            observer.previous = copy.deepcopy(final["arms"])
        action.active = False
        device._baseline_duration, device._advances = duration, advances
        sample = device._sample(final["arms"])
        runner.final_fresh(final)
        device._joint_cache[arm], device._joint_initializations[arm] = cache, source
        action.report.update(ok=True, status="joint_target_initialized", cache_established=True,
            arrival_confirmed=True, controller_at_target=True, observed_stable=True,
            observed_stable_duration_s=duration, observed_feedback_advances=advances,
            feedback_all_after_send=True, after=copy.deepcopy(final["arms"]), sample=sample,
            strict_nominal=strict, within_feedback_tolerance=tolerance,
            selected_arm_strictly_within_limits=strict, selected_arm_within_feedback_tolerance=tolerance,
            cached_target=copy.deepcopy(cache), initialization_source=copy.deepcopy(source))
    except BaseException as exc:
        runner.record_failure(exc)
        device._fault = str(exc)
        device._joint_cache[arm] = None
        device._joint_initializations[arm] = None
        action.report.update(ok=False, status="initialization_failed", cache_established=False,
                             errors=[{"type": type(exc).__name__, "detail": str(exc),
                                      "code": getattr(exc, "code", None)}])
    finally:
        action.auxiliary_executor = None
        action.ticket = None
    result = copy.deepcopy(action.report)
    result.update(arm=arm, accepted=None, physical_stop_verified=None, task_motion_ready=False,
        tracking_observation=copy.deepcopy(runner.tracking.report),
        original_target_cancelled=None, hold_supported=False, frame_receipts=copy.deepcopy(runner.frames),
        mode_frame_can_activate_cached_target=True, partial_frames_can_mix_old_targets=True,
        unknown_cache_activation_bounded=False, retries=0, stop_commands_sent=0, gripper_commands_sent=0,
        transmission_counts=copy.deepcopy(action.counts), session_transmission_counts=action.totals(),
        hardware_commands_sent=sum(c["sent_frames"] for c in action.counts.values()),
        passive_arm_commands_sent=action.counts[action.passive_arm]["sent_frames"],
        guard_violations=copy.deepcopy(action.violations), fault_latched=device._fault is not None)
    return result
