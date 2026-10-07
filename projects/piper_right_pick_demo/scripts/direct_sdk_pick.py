"""Finite, feedback-verified executor for one precomputed right-arm pick attempt.

The caller owns the single operator readiness prompt, cameras, planning, and
report persistence. This module never resets, disables, homes, or retries a
motion target. Closing communication does not cancel an accepted target.
"""
import math
import os
from pathlib import Path
import threading
import time

import direct_sdk_step as core
from direct_sdk_live_step import encode_joint_plan, require_no_drift, require_gripper_geometry

MAX_MOVES = 120
TOTAL_SECONDS = 450.
CAPTURES = ("initial", "pregrasp", "lifted", "placed")


def encode_gripper(sdk, width_raw):
    if width_raw not in (0, 23000, 55000):
        raise core.Rejected("Only the reviewed 0/23/55 mm gripper endpoints are allowed")
    attr = "_C_PiperInterface_V2__arm_can"
    original, capture = getattr(sdk, attr), core.RecordingPort()
    setattr(sdk, attr, capture)
    try:
        sdk.GripperCtrl(width_raw, 300, 1, 0)
    finally:
        setattr(sdk, attr, original)
    if [identifier for identifier, _ in capture.frames] != [0x159]:
        raise core.Rejected("Unexpected official SDK gripper encoding")
    return capture.frames


def _six_ints(values):
    return isinstance(values, (list, tuple)) and len(values) == 6 and all(type(v) is int for v in values)


def _stable(samples, seconds=.2):
    if not samples or samples[-1]["monotonic_s"] - samples[0]["monotonic_s"] < seconds:
        return False
    for key, tolerances in (("pose_raw", (1000,) * 3), ("joints_raw", (300,) * 6)):
        for axis, tolerance in enumerate(tolerances):
            if max(s[key][axis] for s in samples) - min(s[key][axis] for s in samples) > tolerance:
                return False
    if any(core.rotation_distance_deg(sample["pose_raw"], samples[0]["pose_raw"]) > .5 for sample in samples):
        return False
    return all(s["status"]["motion_status"] == 0 for s in samples)


def arrival_residuals(sample, target):
    """Actual minus requested coordinates, with explicit units for diagnostics."""
    joint_errors = [(actual - goal) / 1000. for actual, goal in
                    zip(sample["joints_raw"], target["target_joints_raw"])]
    return {"xyz_error_mm": [(actual - goal) / 1000. for actual, goal in
                             zip(sample["pose_raw"][:3], target["target_pose_raw"][:3])],
            "position_error_mm": core.pose_difference(sample["pose_raw"], target["target_pose_raw"])[0],
            "rotation_error_deg": core.rotation_distance_deg(sample["pose_raw"], target["target_pose_raw"]),
            "joint_error_deg": joint_errors, "max_joint_error_deg": max(abs(error) for error in joint_errors),
            "arrival_limits": {"position_mm": .5, "rotation_deg": .25, "each_joint_deg": .3}}


class PickController:
    def __init__(self, vendor, receiver, state, report):
        self.vendor, self.receiver, self.state, self.report = vendor, receiver, state, report
        self.transport = self.guard = None
        self.deadline = time.monotonic() + TOTAL_SECONDS
        self.held = None
        self.geometry = None
        self.clearance_fn = None
        self.move_count = 0
        self.gripper_purposes = set()
        self.failed = False
        self.last_target = None
        for key, default in (("trace", []), ("transmissions", []), ("stages", []), ("captures", [])):
            report.setdefault(key, default)
        report.update(enable_verified=False, enable_transmissions=0, protocol_completed=False,
                      grasp_success_verified=False, close_contact_candidate=False,
                      exit_does_not_cancel_accepted_target=True)
        if hasattr(state, "joint_box"):
            state.joint_box = None

    def _budget(self):
        if self.failed or time.monotonic() >= self.deadline:
            raise core.Rejected("Pick executor is sealed or its 450-second budget expired")

    def _sample(self, enabled=True):
        self._budget()
        sample = self.state.snapshot(time.monotonic(), enabled)
        require_gripper_geometry(self.state, time.monotonic())
        return sample

    def _send(self, name, frames, command, timeout):
        self._budget()
        if self.guard is None:
            raise core.Rejected("SDK enable preflight must precede every command")
        self.guard.plan[name] = frames
        self.guard.command_limits[name] = 1
        self.guard.deadline = min(self.deadline, time.monotonic() + timeout)
        self.guard.allow(name)
        command()
        if self.guard.pending:
            raise core.Rejected("Official SDK did not send the complete approved command")
        return time.monotonic()

    def enable(self):
        """Verify fresh actual pose, exact CAN binding, and six post-TX enables."""
        self.report["phase"] = "enabling"
        initial = core.collect_stationary(self.receiver, self.state, self.report["trace"])
        require_gripper_geometry(self.state, time.monotonic())
        core.inspect_controllers()
        for process in Path("/proc").glob("[0-9]*"):
            if int(process.name) == os.getpid():
                continue
            try:
                programs = [Path(a.decode(errors="replace")).name for a in
                            (process / "cmdline").read_bytes().split(b"\0")[:5] if a]
            except OSError:
                continue
            if "run_right_pick.py" in programs or "direct_sdk_pick.py" in programs:
                raise core.Rejected("Another pick executor process is already running")
        binding = core.inspect_binding()
        if "binding" in self.report and binding["ifindex"] != self.report["binding"]["ifindex"]:
            raise core.Rejected("CAN binding changed before SDK enable")
        self.report["binding"], self.report["initial"] = binding, initial
        self.vendor.sdk.CreateCanBus(core.CHANNEL, expected_bitrate=1000000, judge_flag=False)
        self.transport = getattr(self.vendor.sdk, "_C_PiperInterface_V2__arm_can")
        self.guard = core.TransmitGuard(self.transport, encode_joint_plan(self.vendor.sdk), self.state,
            self.receiver, self.report, command_limits={"enable": 20})
        self.transport.SendCanMessage = self.guard.send
        deadline, next_send, last_send = min(self.deadline, time.monotonic() + 2.), 0., None
        self.guard.deadline = deadline
        print("SDK 阶段：验证六个关节使能。", flush=True)
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send and self.guard.calls["enable"] < 20:
                self.guard.allow("enable")
                self.vendor.sdk.EnablePiper()
                self.report["enable_transmissions"] = self.guard.calls["enable"]
                last_send, next_send = self.report["transmissions"][-1]["monotonic_s"], now + .1
            self.receiver.one(.01)
            sample = self._sample(False)
            require_no_drift(sample, initial)
            if (last_send is not None and all(sample["motor_enabled"]) and
                    all(self.state.received[i] > last_send for i in range(0x261, 0x267))):
                self.report["enable_verified"] = True
                break
        if not self.report["enable_verified"]:
            raise core.Rejected("Six SDK motor enables were not verified within two seconds")
        self.guard.require_enabled, self.guard.deadline = True, self.deadline
        self.held = core.collect_stationary(self.receiver, self.state, self.report["trace"], True)
        require_no_drift(self.held, initial)
        self.report["held"] = self.held
        print("SDK 使能已由新鲜反馈确认；保持当前姿态。", flush=True)
        return self.held

    def observe(self, fn, timeout=30):
        """Run camera/planner work while the main thread receives held-arm CAN."""
        if not 0 < timeout <= 60:
            raise core.Rejected("Observation timeout must be within 60 seconds")
        reference = self._sample()
        result, finished = {}, threading.Event()

        def worker():
            try:
                result["value"] = fn()
            except BaseException as exc:
                result["error"] = exc
            finally:
                finished.set()

        threading.Thread(target=worker, daemon=True).start()
        deadline = min(self.deadline, time.monotonic() + timeout)
        while not finished.is_set():
            if time.monotonic() >= deadline:
                raise core.Rejected("Camera/planner work timed out while arm held position")
            self.receiver.one(.01)
            require_no_drift(self._sample(), reference)
        if "error" in result:
            raise result["error"]
        self.receiver.drain()
        self.held = self._sample()
        require_no_drift(self.held, reference)
        return result["value"]

    def gripper(self, width_raw, purpose):
        """One low-effort command, with measured opening/contact classification."""
        expected = {"observe": 23000, "open": 55000, "close": 0, "release": 55000}
        if purpose not in expected or width_raw != expected[purpose] or purpose in self.gripper_purposes:
            raise core.Rejected("Gripper purpose, endpoint, or single-attempt budget is invalid")
        if purpose == "observe" and self.gripper_purposes:
            raise core.Rejected("The 23 mm observation opening is only allowed before other gripper commands")
        if purpose == "close" and "open" not in self.gripper_purposes:
            raise core.Rejected("A verified 55 mm opening must precede closing")
        if purpose == "release" and self.report.get("grasp_contact_classification") not in ("contact_candidate", "empty"):
            raise core.Rejected("Release requires a recorded contact candidate or confirmed empty closure")
        reference = self._sample()
        self.gripper_purposes.add(purpose)
        self.report["phase"] = "gripper_" + purpose
        entry = {"kind": "gripper", "label": purpose, "width_raw": width_raw, "effort_raw": 300,
                 "command_sent": False, "feedback_verified": False}
        self.report["stages"].append(entry)
        sent_at = self._send("gripper_" + purpose, encode_gripper(self.vendor.sdk, width_raw),
            lambda: self.vendor.sdk.GripperCtrl(width_raw, 300, 1, 0), 5.)
        entry["command_sent"], entry["sent_at_monotonic_s"] = True, sent_at
        print("夹爪阶段：%s，目标 %.1f mm，力矩参数 0.3 N·m。" % (purpose, width_raw / 1000.), flush=True)
        settled = []
        while time.monotonic() < self.guard.deadline:
            self.receiver.one(.01)
            sample = self._sample()
            require_no_drift(sample, reference)
            frame = self.state.raw_frame_latest["0x2A8"]
            feedback = dict(self.state.gripper)
            if frame["kernel_received_monotonic_s"] <= sent_at or not feedback["status_code"] & 64:
                settled = []
                continue
            width, effort = feedback["angle_raw"], feedback["effort_raw"]
            now = time.monotonic()
            settled.append((now, width, effort))
            settled = [s for s in settled if now - s[0] <= .4]
            stable = (settled[-1][0] - settled[0][0] >= .25 and
                      max(s[1] for s in settled) - min(s[1] for s in settled) <= 500)
            if not stable:
                continue
            if purpose != "close":
                if abs(width - width_raw) > 2000:
                    continue
                classification = {"observe": "observation_opening", "open": "opened", "release": "released"}[purpose]
            elif width <= 8000:
                classification = "empty"
            elif 20000 <= width <= 45000 and all(abs(s[2]) > 0 for s in settled):
                classification = "contact_candidate"
            else:
                classification = "ambiguous"
            result = {"classification": classification, "sample": sample,
                      "width_raw": width, "effort_raw": effort, "feedback": feedback}
            entry.update(feedback_verified=True, result=result)
            self.held = sample
            if purpose == "close":
                self.report["close_contact_candidate"] = classification == "contact_candidate"
                self.report["grasp_contact_classification"] = classification
                self.report["grasp_outcome"] = classification
                self.report["physical_grasp_attempts"] = 1
            return result
        raise core.Rejected("Gripper opening/contact was not verified within five seconds")

    def _clearance(self, joints):
        if self.clearance_fn is None or self.geometry is None:
            raise core.Rejected("A validated geometric plan is required before arm motion")
        result = self.clearance_fn(joints, self.geometry)
        clearance = result["minimum_clearance_mm"]
        if not math.isfinite(clearance) or clearance < 10:
            raise core.Rejected("Measured geometry has exhausted the reserved table clearance")
        return result

    def _check_segment(self, start, target):
        if not _six_ints(target["joints_raw"]) or not _six_ints(target["pose_raw"]):
            raise core.Rejected("Every waypoint must contain six integer joint and pose coordinates")
        if not core.in_nominal_range(target["joints_raw"]):
            raise core.Rejected("Waypoint joints are outside manufacturer nominal ranges")
        position, _ = core.pose_difference(start["pose_raw"], target["pose_raw"])
        if (position > 15.05 or core.rotation_distance_deg(start["pose_raw"], target["pose_raw"]) > 5.05 or
                max(abs(a - b) for a, b in zip(start["joints_raw"], target["joints_raw"])) > 8050):
            raise core.Rejected("Waypoint exceeds the 15 mm / 5 degree / 8 joint-degree segmentation")
        fk = self.vendor.fk_pose_raw(target["joints_raw"])
        if core.pose_difference(fk, target["pose_raw"])[0] > .3 or core.rotation_distance_deg(fk, target["pose_raw"]) > .1:
            raise core.Rejected("Waypoint coordinates disagree with the official SDK forward kinematics")

    def validate_plan(self, plan, live_reference=None):
        from pick_trajectory import clearance_for_joints
        self.geometry, self.clearance_fn = plan["geometry"], clearance_for_joints
        if not _six_ints(plan["start_joints_raw"]) or not _six_ints(plan["start_pose_raw"]):
            raise core.Rejected("Plan start coordinates are malformed")
        start = {"joints_raw": plan["start_joints_raw"], "pose_raw": plan["start_pose_raw"]}
        require_no_drift(self._sample() if live_reference is None else live_reference, start)
        stages = plan["stages"]
        if not isinstance(stages, list) or not stages or len(stages) > 140:
            raise core.Rejected("Plan stage list is empty or exceeds the finite operation budget")
        moves, closed, released, captures = 0, False, False, set()
        for stage in stages:
            kind, label = stage.get("kind"), stage.get("label")
            if not isinstance(label, str) or not label:
                raise core.Rejected("Every stage needs a label")
            if kind == "move":
                moves += 1
                self._check_segment(start, stage)
                self._clearance(stage["joints_raw"])
                if not math.isfinite(stage["clearance_mm"]) or stage["clearance_mm"] < 10:
                    raise core.Rejected("Planned motion lacks nonnegative reserved table clearance")
                start = stage
            elif kind == "gripper":
                if stage.get("effort_raw") != 300:
                    raise core.Rejected("Gripper torque parameter must be 0.3 N·m")
                if label == "close" and stage.get("width_raw") == 0 and not closed and "pregrasp" in captures:
                    closed = True
                elif (label == "release" and stage.get("width_raw") == 55000 and closed and not released
                      and "lifted" in captures):
                    released = True
                else:
                    raise core.Rejected("Gripper stage order or width is invalid")
            elif kind == "capture":
                if label not in ("pregrasp", "lifted", "placed") or label in captures:
                    raise core.Rejected("Unknown or duplicate capture checkpoint")
                if (label == "pregrasp" and closed or label == "lifted" and (not closed or released) or
                        label == "placed" and not released):
                    raise core.Rejected("Capture checkpoint is out of order")
                captures.add(label)
            else:
                raise core.Rejected("Unknown stage type in precomputed plan")
        if not 1 <= moves <= MAX_MOVES or not released or captures != {"pregrasp", "lifted", "placed"}:
            raise core.Rejected("Plan must contain one finite pick/place sequence and its three checkpoints")
        self._clearance(plan["start_joints_raw"])
        self.report["plan"] = plan
        self.report["plan_validated"] = True

    def move(self, stage):
        start = self._sample()
        self._check_segment(start, stage)
        self._clearance(start["joints_raw"])
        self._clearance(stage["joints_raw"])
        if self.move_count >= MAX_MOVES:
            raise core.Rejected("Arm waypoint budget exceeded")
        self.move_count += 1
        label = stage["label"]
        self.report["phase"] = "move_" + label
        lower = [min(a, b) - 500 for a, b in zip(start["joints_raw"], stage["joints_raw"])]
        upper = [max(a, b) + 500 for a, b in zip(start["joints_raw"], stage["joints_raw"])]
        if hasattr(self.state, "joint_box"):
            self.state.joint_box = lower, upper
        entry = {"kind": "move", "label": label, "target_joints_raw": stage["joints_raw"],
                 "target_pose_raw": stage["pose_raw"], "command_sent": False, "arrival_verified": False,
                 "joint_lower_raw": lower, "joint_upper_raw": upper}
        self.report["stages"].append(entry)
        self.last_target = entry
        before = len(self.report["transmissions"])
        try:
            sent_at = self._send("move_%02d" % self.move_count,
                encode_joint_plan(self.vendor.sdk, stage["joints_raw"])["motion"],
                lambda: (self.vendor.sdk.MotionCtrl_2(1, 1, 5, 0), self.vendor.sdk.JointCtrl(*stage["joints_raw"])), 5.)
        finally:
            ids = [frame["id"] for frame in self.report["transmissions"][before:]]
            entry["target_frame_attempted"] = any(identifier in ("0x155", "0x156", "0x157") for identifier in ids)
        entry.update(command_sent=True, sent_at_monotonic_s=sent_at)
        print("运动 %d：%s，5%% 速度。" % (self.move_count, label), flush=True)
        settled, last_record = [], 0.
        while time.monotonic() < self.guard.deadline:
            self.receiver.one(.01)
            sample = self._sample()
            if any(not lo <= q <= hi for q, lo, hi in zip(sample["joints_raw"], lower, upper)):
                raise core.Rejected("Measured waypoint progress left its half-degree padded joint interval")
            # Raw feedback arrives in thousands of split frames per second.
            # Keep draining/checking it, but bound CAD FK and trace work to 50 Hz.
            now = time.monotonic()
            if now - last_record < .02:
                continue
            last_record = now
            self._clearance(sample["joints_raw"])
            entry["last_arrival_residuals"] = arrival_residuals(sample, entry)
            self.report["trace"].append(dict(sample, stage=label))
            if self._arrived(sample, entry):
                settled.append(sample)
                if _stable(settled):
                    self.receiver.drain()
                    final = self._sample()
                    if not self._arrived(final, entry):
                        settled = []
                        continue
                    self._clearance(final["joints_raw"])
                    entry.update(arrival_verified=True, final=final,
                                 last_arrival_residuals=arrival_residuals(final, entry))
                    self.held = final
                    return final
            else:
                settled = []
        raise core.Rejected("Waypoint arrival was not verified within five seconds; residuals=" +
                            str(entry.get("last_arrival_residuals", "no qualified post-command sample")))

    def _arrived(self, sample, target):
        return (target.get("command_sent", False) and
                core.pose_difference(sample["pose_raw"], target["target_pose_raw"])[0] <= .5 and
                core.rotation_distance_deg(sample["pose_raw"], target["target_pose_raw"]) <= .25 and
                max(abs(a - b) for a, b in zip(sample["joints_raw"], target["target_joints_raw"])) <= 300 and
                sample["status"]["ctrl_mode"] == 1 and sample["status"]["mode_feed"] == 1 and
                sample["status"]["motion_status"] == 0 and all(sample["motor_enabled"]) and
                all(self.state.received[i] > target["sent_at_monotonic_s"] for i in core.REQUIRED))

    def execute(self, plan, capture_callback):
        """Validate the whole path first, then perform it exactly once."""
        try:
            live_reference = self._sample()
            self.observe(lambda: self.validate_plan(plan, live_reference), timeout=30)
            if "open" not in self.gripper_purposes:
                raise core.Rejected("Initial verified opening and camera observation must precede the plan")
            for stage in plan["stages"]:
                self._budget()
                if stage["kind"] == "move":
                    self.move(stage)
                elif stage["kind"] == "gripper":
                    result = self.gripper(stage["width_raw"], stage["label"])
                    if stage["label"] == "close" and result["classification"] == "ambiguous":
                        self.report.update(status="grasp_" + result["classification"], phase="attempt_stopped")
                        return self.report
                else:
                    label, sample = stage["label"], self._sample()
                    result = self.observe(lambda: capture_callback(label, sample), timeout=30)
                    if not isinstance(result, dict):
                        raise core.Rejected("Camera checkpoint must return a diagnostic dictionary")
                    self.report["captures"].append({"label": label, "result": result, "sample": sample})
                    if result.get("abort"):
                        raise core.Rejected("Camera checkpoint stopped the attempt: " + str(result.get("reason", label)))
            self.report.update(status="protocol_completed", protocol_completed=True, phase="complete")
            return self.report
        except BaseException as exc:
            self.fail(exc)
            raise

    def fail(self, exc):
        """Seal all TX and observe for five seconds, including partial sends."""
        if self.failed:
            return
        self.failed = True
        self.report["failure"] = {"phase": self.report.get("phase"), "error": str(exc), "type": type(exc).__name__}
        self.report.update(status="failed", protocol_completed=False)
        if self.guard is not None:
            self.guard.pending.clear()
            self.guard.plan = {}
            self.guard.command_limits = {}
            self.guard.deadline = 0.
        if not self.report["transmissions"]:
            return
        print("停止下发，继续只读监测；不保证取消已发送目标。不会自动失能、复位或开爪。", flush=True)
        deadline = time.monotonic() + 5.
        observations, errors, settled = [], [], []
        self.report["post_failure_observations"] = observations
        self.report["post_failure_target_stable"] = False
        self.report["partial_motion_target"] = bool(self.last_target and
            self.last_target.get("target_frame_attempted") and not self.last_target["command_sent"])
        last_record = 0.
        while time.monotonic() < deadline:
            try:
                self.receiver.one(.01)
            except KeyboardInterrupt:
                self.report["post_failure_monitor_interrupted"] = True
                break
            except Exception as error:
                if len(errors) < 20:
                    errors.append(str(error))
                time.sleep(min(.01, max(0., deadline - time.monotonic())))
            now = time.monotonic()
            if now - last_record < .02:
                continue
            last_record = now
            diagnostic = self.state.diagnostic(now)
            record = {key: diagnostic[key] for key in ("pose_raw", "joints_raw", "status", "gripper", "required_frame_age_s")}
            record["monotonic_s"] = now
            try:
                sample = self.state.snapshot(now, True)
                if self.last_target and self._arrived(sample, self.last_target):
                    settled.append(sample)
                    self.report["post_failure_target_stable"] = _stable(settled)
                else:
                    settled = []
                    self.report["post_failure_target_stable"] = False
            except Exception as error:
                record["validation_error"] = str(error)
                settled = []
                self.report["post_failure_target_stable"] = False
            observations.append(record)
        self.report["post_failure_receive_errors"] = errors
        self.report["post_failure_final_diagnostic"] = self.state.diagnostic(time.monotonic())
        self.report["post_failure_monitor_finished_at_monotonic_s"] = time.monotonic()

    def close(self):
        if self.guard is not None:
            self.guard.pending.clear()
            self.guard.plan = {}
            self.guard.command_limits = {}
        if self.transport is not None:
            self.transport.Close()
