"""Same-connection, empty-jaw preparation; no arm command or motion grant.

The host owns the scene/empty-jaw decision, durable once-only event and owner
guard. This module never constructs a second SDK driver, retries, changes mode,
enables arm motors or claims a physical stop. Known disabled flags are readiness
facts; malformed feedback, drift or an uncertain dispatch remain sticky faults.
"""
import copy
from .feedback_tolerance import rotation_tolerance
from .feedback_tolerance import window_joints_within
import threading
import time

from . import arms
from .single_gripper_prepare import (
    _SingleGripperPrepare, PRE_SEND_WIDTH_TOLERANCE_M,
    AFTER_SEND_WIDTH_TOLERANCE_M, WIDTH_MAX_M)
from .single_supervised_actions import BOUNDS, _SingleSupervisedAction
from .takeover import LIMITS, SIDES


class _PreparationObserver(_SingleGripperPrepare):
    """Original stationary anchor and known flags, with no enabled prerequisite."""

    def __init__(self, action):
        super().__init__(action.profile, action.journal, "right")
        self.action = action
        self.flags = {}
        self.prepared_targets = {}
        self.previous = None
        self.joint_limits = action.joint_limits
        self.quaternion = action.quaternion
        self.violations = action.violations

    def check_enable_state(self, side, state):
        flags = self.enable_flags(state)
        if any(type(flag) is not bool for flag in flags):
            raise RuntimeError(side + " requires seven known enable flags")
        if not 0 <= state["gripper"]["width_m"] <= WIDTH_MAX_M:
            raise RuntimeError(side + " preparation feedback width outside 0..70 mm")
        self.flags.setdefault(side, flags[:])
        if flags != self.flags[side]:
            raise RuntimeError(side + " enable flags changed outside same-connection preparation")
        if side in self.prepared_targets and abs(
                state["gripper"]["width_m"] - self.prepared_targets[side]) > AFTER_SEND_WIDTH_TOLERANCE_M:
            raise RuntimeError(side + " prepared jaw departed from its original measured-width target")
        self.report["gripper_enabled"][side] = flags[6]

    def timestamps(self, states):
        if self.previous is not None:
            for side in SIDES:
                old = self.previous[side]["fragment_timestamps_s"]
                new = states[side]["fragment_timestamps_s"]
                if any(new[key] < stamp for key, stamp in old.items()):
                    raise RuntimeError(side + " preparation feedback fragment regressed")
        self.action.check_freshness(states)
        self.previous = copy.deepcopy(states)

    def checked(self, states):
        result = super().checked(states)
        self.timestamps(states)
        return result

    new_window = _SingleSupervisedAction.new_window
    extend_window = _SingleSupervisedAction.extend_window
    window_spans = staticmethod(_SingleSupervisedAction.window_spans)

    def window_stable(self, window):
        return all(window_joints_within(self.feedback_policy, side, window[side])
                   and s["position_m"] <= LIMITS["position_m"]
                   and s["jaw_m"] <= LIMITS["gripper_m"]
                   and s["rotation_rad"] <= rotation_tolerance(self.feedback_policy,side)
                   for side,s in self.window_spans(window).items())


def _read(device, checker):
    action = device._action
    action.guard()
    states = action.read()
    checker.checked(states)
    action.check_freshness(states)  # Includes synchronous feedback journal time.
    return states


def _readiness(states):
    return {side: {"ctrl_mode": states[side]["arm_status"]["ctrl_mode"],
                   "mode_feedback": states[side]["arm_status"]["mode_feedback"],
                   "enable_flags": _SingleGripperPrepare.enable_flags(states[side])}
            for side in SIDES}


def _sample(device, states):
    result = device._sample(states)
    result.update(connected_for_preparation=True, task_ready=device._task_ready,
                  readiness=_readiness(states), hardware_commands_sent=0,
                  motion_gate_unlocked=False, grasp_verified=False,
                  observed_joint_limit_violations=copy.deepcopy(
                      device._preparation.report["observed_joint_limit_violations"]),
                  scope="Stationary preparation feedback only; no task-motion or physical-stop qualification")
    return result


def connect_for_preparation(device):
    if device._opened or device._closed or device._fault is not None:
        raise RuntimeError("Pair device can only be connected once")
    action = device._action
    try:
        action.guard()
        action.connect()
        observer = _PreparationObserver(action)
        device._preparation = observer
        deadline = time.monotonic() + LIMITS["warmup_s"]
        while True:
            action.guard()
            state = action.read()
            if all(state[side].get("status") == "complete" for side in SIDES):
                observer.checked(state)
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("No complete dual-arm preparation feedback within warmup")
            time.sleep(BOUNDS["poll_s"])
        observer.anchor = copy.deepcopy(state)
        observer.expected_modes = {s: state[s]["arm_status"]["mode_feedback"] for s in SIDES}
        observer.required_ctrl = {s: state[s]["arm_status"]["ctrl_mode"] for s in SIDES}
        window = observer.new_window(state)
        previous, advances, start = state, 0, time.monotonic()
        while time.monotonic() - start < BOUNDS["stable_s"]:
            time.sleep(BOUNDS["poll_s"])
            state = _read(device, observer)
            if not observer.extend_window(window, state):
                raise RuntimeError("Preparation baseline exceeded original stationary spans")
            if action.advanced(previous, state):
                previous, advances = state, advances + 1
        if advances < BOUNDS["minimum_feedback_advances"]:
            raise RuntimeError("Preparation baseline requires 20 complete advancing samples")
        device._baseline_duration = time.monotonic() - start
        device._advances = advances
        action.idle_anchor = copy.deepcopy(observer.anchor)
        device._opened = True
        device._task_ready = False
        return _sample(device, state)
    except BaseException as exc:
        device._fault = str(exc)
        raise


def observe_preparation(device):
    device._connected_usable()
    observer = device._preparation
    if observer is None:
        raise RuntimeError("Preparation connection has no original feedback anchor")
    deadline = time.monotonic() + BOUNDS["feedback_age_s"]
    while True:
        state = _read(device, observer)
        if device._previous is None or device._action.advanced(device._previous, state):
            return _sample(device, state)
        if time.monotonic() >= deadline:
            raise RuntimeError("Preparation observation lacks independently advancing feedback")
        time.sleep(BOUNDS["poll_s"])


class _JawExecutor:
    """Adopt the existing single-jaw checker into the persistent exact encoder."""

    def __init__(self, device, side, before):
        self.device, self.action, self.side = device, device._action, side
        action, observer = self.action, device._preparation
        self.checker = _SingleGripperPrepare(action.profile, action.journal, side)
        for name in ("robots", "grippers", "comms", "buses", "violations", "joint_limits", "quaternion", "counts"):
            setattr(self.checker, name, getattr(action, name))
        self.checker.anchor = copy.deepcopy(observer.anchor)
        self.checker.expected_modes = copy.deepcopy(observer.expected_modes)
        self.checker.required_ctrl = copy.deepcopy(observer.required_ctrl)
        self.checker.read = self.read
        self.checker.send_one = self.send_one
        self.returned = False
        self.before_flags = {s: self.checker.enable_flags(before[s]) for s in SIDES}
        self.checker.report["before"] = copy.deepcopy(before)
        self.checker.checked(before)

    def encoder_guard(self):
        self.device._connected_usable()

    def check_states(self, states):
        # The only allowed flag transition starts after the exact single frame
        # returned. Setting jaw_phase='enabling' before encoding is not an ACK.
        for side in SIDES:
            flags = self.checker.enable_flags(states[side])
            expected = self.before_flags[side]
            if flags[:6] != expected[:6] or (side != self.side or not self.returned) and flags[6] != expected[6]:
                raise RuntimeError(side + " enable state changed before authorized jaw response")
        result = self.checker.checked(states)
        observer = self.device._preparation
        for side, target in observer.prepared_targets.items():
            if abs(states[side]["gripper"]["width_m"] - target) > AFTER_SEND_WIDTH_TOLERANCE_M:
                raise RuntimeError(side + " prepared jaw departed from its original measured-width target")
        observer.timestamps(states)
        if not self.returned:
            target = self.checker.width_target_raw / 1e6
            if abs(states[self.side]["gripper"]["width_m"] - target) > PRE_SEND_WIDTH_TOLERANCE_M:
                raise RuntimeError("Measured width changed more than 0.5 mm before preparation frame")
        return result

    def read(self):
        self.encoder_guard()
        states = self.action.read()
        self.check_states(states)
        self.checker.report["after"] = copy.deepcopy(states)
        self.checker.report["samples"] += 1
        return states

    def send_frame(self, side, original, frame, *args, **kwargs):
        self.encoder_guard()
        # Read after a possibly slow durable guard; no journal before actual TX.
        states = {s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES}
        self.check_states(states)
        self.action.check_freshness(states)
        return original(frame, *args, **kwargs)

    def send_one(self, side, kind, sdk_call):
        if side != self.side or kind != "gripper" or self.returned:
            raise RuntimeError("Only one selected measured-width jaw request is available")
        action = self.action
        with action.lock:
            if action.ticket is not None or action.violations:
                raise RuntimeError("Preparation cannot overlap another dispatch or CAN violation")
            ticket = {"side": side, "thread": threading.get_ident(), "kind": "action",
                      "frames": [self.checker.frame_spec(kind, side)],
                      "comm_calls": 0, "bus_calls": 0, "error": None}
            action.ticket = ticket
            try:
                value = sdk_call()
                if ticket["error"] is not None:
                    raise RuntimeError("Preparation CAN result uncertain: " + str(ticket["error"]))
                if ticket["comm_calls"] != 1 or ticket["bus_calls"] != 1:
                    raise RuntimeError("Preparation requires exactly one returned standard 0x159 frame")
                self.returned = True
                action.sent_at = time.time()
                return action.sent_at, value
            finally:
                action.ticket = None


def _not_ready(device, state, requirements):
    return {"ok": False, "status": "preparation_required", "requirements": requirements,
            "task_ready": device._task_ready, "fault_latched": False,
            "hardware_commands_sent": 0, "readiness": _readiness(state),
            "physical_stop_verified": None, "grasp_verified": False}


def prepare_gripper(device, arm):
    if type(arm) is not str or arm not in SIDES:
        raise ValueError("arm must be explicitly left or right")
    device._connected_usable()
    action = device._action
    if any(action.grasps.values()):
        raise RuntimeError("Empty-jaw preparation cannot replace an unresolved or retained grasp")
    try:
        sample = device._observe() if device._task_ready else observe_preparation(device)
        state = sample["arms"]
        flags = _SingleGripperPrepare.enable_flags(state[arm])
        requirements = []
        if state[arm]["arm_status"]["ctrl_mode"] != 1:
            requirements.append("selected_arm_CAN_control_mode_1")
        if not all(flags[:6]):
            requirements.append("selected_arm_six_joint_drivers_enabled")
        if requirements:
            return _not_ready(device, state, requirements)
        if flags[6]:
            return {"ok": True, "status": "already_enabled_observed", "arm": arm,
                    "hardware_commands_sent": 0, "task_ready": device._task_ready,
                    "sample": sample, "physical_stop_verified": None, "grasp_verified": False}
        action.reset_action(arm, "gripper", state[arm]["gripper"]["width_m"])
        runner = _JawExecutor(device, arm, state)
        action.auxiliary_executor = runner
        try:
            status = runner.checker.perform()
            final = runner.read()
            observer = device._preparation
            # Preserve every original posture/width anchor. Only the one
            # explicitly observed enable transition changes expected flags.
            observer.flags[arm][6] = True
            observer.prepared_targets[arm] = runner.checker.width_target_raw / 1e6
            observer.checked(final)
            sample = _sample(device, final)
            report = copy.deepcopy(runner.checker.report)
            report.update(ok=True, status=status, sample=sample,
                          target_width_raw=runner.checker.width_target_raw)
        finally:
            action.auxiliary_executor = None
            action.ticket = None
        report.update(task_ready=False, accepted=None, physical_stop_verified=None,
                      hardware_commands_sent=action.counts[arm]["sent_frames"],
                      gripper_enable_commands_sent=action.counts[arm]["sent_frames"],
                      gripper_target_commands_sent=action.counts[arm]["sent_frames"],
                      target_commands_sent=action.counts[arm]["sent_frames"],
                      passive_arm_commands_sent=action.counts[runner.checker.passive_arm]["sent_frames"],
                      drift=copy.deepcopy(runner.checker.max_drift),
                      guard_violations=copy.deepcopy(action.violations),
                      transmission_counts=copy.deepcopy(action.counts),
                      session_transmission_counts=action.totals(), fault_latched=False)
        return report
    except BaseException as exc:
        device._fault = str(exc)
        raise


def promote_ready(device):
    device._connected_usable()
    if device._task_ready:
        return device._observe()
    try:
        state = observe_preparation(device)["arms"]
        missing = []
        for side in SIDES:
            if state[side]["arm_status"]["ctrl_mode"] != 1:
                missing.append(side + "_CAN_control_mode_1")
            if not all(_SingleGripperPrepare.enable_flags(state[side])):
                missing.append(side + "_six_joints_and_jaw_enabled")
        if missing:
            return _not_ready(device, state, missing)
        action = device._action
        action.reset_action("right", "gripper", state["right"]["gripper"]["width_m"])
        task_checked = action.checked
        def both_checked(states):
            device._preparation.checked(states)
            return task_checked(states)
        # Neither the new task baseline nor its first read may absorb a mode,
        # flag or measured-width change forbidden by the preparation contract.
        # The device operation lock excludes other operations during this
        # temporary composition; both checkers remain active for every sample.
        action.checked = both_checked
        try:
            action.prepare()  # Original task checks, no connection and no sending.
        finally:
            action.checked = task_checked
        after = action.report["after"]
        # Readiness changes no arm/jaw target. Its new sampling window therefore
        # cannot replace the connection's original stationary drift reference.
        # Check both contracts before switching to the normal task checker.
        device._preparation.checked(after)
        action.idle_anchor = copy.deepcopy(device._preparation.anchor)
        action.anchor = copy.deepcopy(action.idle_anchor)
        action.checked(after)
        device._baseline_duration = action.report["baseline_duration_s"]
        device._advances = action.report["baseline_feedback_advances"]
        device._task_ready = True
        sample = device._sample(after)
        sample.update(task_ready=True, connected_for_preparation=True, hardware_commands_sent=0)
        return sample
    except BaseException as exc:
        device._fault = str(exc)
        raise
