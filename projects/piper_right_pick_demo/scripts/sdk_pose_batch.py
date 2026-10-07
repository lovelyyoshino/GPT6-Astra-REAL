"""Finite Cartesian SDK output for an already specified pose list.

No vision, inverse kinematics, task planning, automatic recovery, or CLI.
The controller owns Cartesian interpolation/IK. Closing CAN or sealing TX does
not cancel an accepted target; no disable/reset/quick-stop is sent on failure.
"""
import math
import time

import direct_sdk_step as core
from direct_sdk_live_step import require_no_drift
from direct_sdk_pick import PickController, _six_ints, _stable


def encode_pose_frames(sdk, pose_raw, speed_percent=5):
    if not _six_ints(pose_raw) or type(speed_percent) is not int or speed_percent != 5:
        raise core.Rejected("Cartesian output requires six integer coordinates and 5% speed")
    attr = "_C_PiperInterface_V2__arm_can"
    original, capture = getattr(sdk, attr), core.RecordingPort()
    setattr(sdk, attr, capture)
    try:
        sdk.MotionCtrl_2(1, 2, speed_percent, 0)
        sdk.EndPoseCtrl(*pose_raw)
    finally:
        setattr(sdk, attr, original)
    if [identifier for identifier, _ in capture.frames] != [0x151, 0x152, 0x153, 0x154]:
        raise core.Rejected("Unexpected official SDK Cartesian encoding")
    return capture.frames


def pose_residuals(sample, target):
    return {"position_error_mm": core.pose_difference(sample["pose_raw"], target)[0],
            "rotation_error_deg": core.rotation_distance_deg(sample["pose_raw"], target),
            "xyz_error_mm": [(a - b) / 1000. for a, b in zip(sample["pose_raw"][:3], target[:3])],
            "arrival_limits": {"position_mm": 1., "rotation_deg": .3}}


def segment_distance_mm(pose, start, end):
    delta = [b - a for a, b in zip(start[:3], end[:3])]
    offset = [p - a for p, a in zip(pose[:3], start[:3])]
    length2 = sum(d * d for d in delta)
    fraction = max(0., min(1., sum(a * b for a, b in zip(offset, delta)) / length2)) if length2 else 0.
    return math.sqrt(sum((a - fraction * b) ** 2 for a, b in zip(offset, delta))) / 1000.


def motion_residuals(sample, reference, target):
    """Transient tracking bounds are distinct from final arrival precision."""
    return {"position_corridor_error_mm": segment_distance_mm(sample["pose_raw"], reference, target),
            "rotation_tracking_error_deg": core.rotation_distance_deg(sample["pose_raw"], reference),
            "motion_limits": {"position_mm": 5., "rotation_deg": .5}}


class PoseBatchController(PickController):
    def __init__(self, vendor, receiver, state, report):
        super().__init__(vendor, receiver, state, report)
        self.deadline = time.monotonic() + 180.
        self.executed = False
        report.update(motion_interface="SDK MOVE_L + EndPoseCtrl", python_inverse_kinematics=False)

    def _budget(self):
        if self.failed or time.monotonic() >= self.deadline:
            raise core.Rejected("Cartesian executor is sealed or its 180-second budget expired")

    def validate_pose_plan(self, plan, live_reference=None):
        from pick_trajectory import clearance_for_joints
        if not _six_ints(plan.get("start_joints_raw")) or not _six_ints(plan.get("start_pose_raw")):
            raise core.Rejected("Plan start must contain six integer joint and pose coordinates")
        if not core.in_nominal_range(plan["start_joints_raw"]):
            raise core.Rejected("Plan start joints are outside manufacturer nominal ranges")
        start = {"joints_raw": plan["start_joints_raw"], "pose_raw": plan["start_pose_raw"]}
        require_no_drift(self._sample() if live_reference is None else live_reference, start)
        self.geometry, self.clearance_fn = plan["geometry"], clearance_for_joints
        self._clearance(start["joints_raw"])
        stages = plan.get("stages")
        if not isinstance(stages, list) or not 1 <= len(stages) <= 16:
            raise core.Rejected("Cartesian plan requires a finite list of at most 16 stages")
        previous, moves, closed, released, labels = start["pose_raw"], 0, False, False, set()
        for stage in stages:
            if not isinstance(stage, dict) or not isinstance(stage.get("label"), str) or not stage["label"]:
                raise core.Rejected("Every stage requires a nonempty label")
            label, kind = stage["label"], stage.get("kind")
            if label in labels:
                raise core.Rejected("Stage labels must be unique")
            labels.add(label)
            if kind == "move":
                pose, lower, upper = (stage.get(k) for k in ("pose_raw", "joint_lower_raw", "joint_upper_raw"))
                if not all(_six_ints(value) for value in (pose, lower, upper, stage.get("reference_start_pose_raw"))):
                    raise core.Rejected("Cartesian stage coordinates and joint bounds must be six integers")
                if stage["reference_start_pose_raw"] != previous:
                    raise core.Rejected("Cartesian stage start must equal the preceding planned pose")
                if any(abs(a - b) > 180000 for a, b in zip(previous[3:], pose[3:])):
                    raise core.Rejected("Provide continuous Euler angles; equivalent 360-degree jumps are not sent")
                if (core.pose_difference(previous, pose)[0] > 85 or
                        core.rotation_distance_deg(start["pose_raw"], pose) > .3):
                    raise core.Rejected("Cartesian stage exceeds 85 mm or changes the fixed orientation")
                if (not core.in_nominal_range(lower) or not core.in_nominal_range(upper) or
                        any(lo > hi for lo, hi in zip(lower, upper))):
                    raise core.Rejected("Cartesian joint bounds exceed manufacturer nominal ranges")
                encode_pose_frames(self.vendor.sdk, pose, stage.get("speed_percent"))
                previous, moves = pose, moves + 1
            elif kind == "gripper":
                if stage.get("effort_raw") != 300:
                    raise core.Rejected("Gripper torque parameter must be 0.3 N.m")
                if label == "close" and stage.get("width_raw") == 0 and not closed:
                    closed = True
                elif label == "release" and stage.get("width_raw") == 55000 and closed and not released:
                    released = True
                else:
                    raise core.Rejected("Cartesian batch gripper stage order or width is invalid")
            elif kind == "capture":
                if (label not in ("pregrasp", "lifted", "placed") or
                        label == "pregrasp" and closed or label == "lifted" and (not closed or released) or
                        label == "placed" and not released):
                    raise core.Rejected("Camera checkpoint is unknown or out of order")
            else:
                raise core.Rejected("Unknown Cartesian batch stage kind")
        if not 1 <= moves <= 12 or not released:
            raise core.Rejected("Cartesian batch must contain 1–12 moves and one close/release sequence")
        self.report.update(plan=plan, plan_validated=True)

    def _arrived(self, sample, target):
        return (target.get("command_sent", False) and
                core.pose_difference(sample["pose_raw"], target["target_pose_raw"])[0] <= 1 and
                core.rotation_distance_deg(sample["pose_raw"], target["target_pose_raw"]) <= .3 and
                sample["status"]["ctrl_mode"] == 1 and sample["status"]["mode_feed"] == 2 and
                sample["status"]["motion_status"] == 0 and all(sample["motor_enabled"]) and
                all(self.state.received[i] > target["sent_at_monotonic_s"] for i in core.REQUIRED))

    def move_pose(self, stage):
        start = self._sample()
        target, reference = stage["pose_raw"], stage["reference_start_pose_raw"]
        if (not self.report.get("plan_validated") or self.move_count >= 12 or
                core.pose_difference(start["pose_raw"], reference)[0] > 2 or
                core.rotation_distance_deg(start["pose_raw"], reference) > .3 or
                core.pose_difference(start["pose_raw"], target)[0] > 85):
            raise core.Rejected("Cartesian move lacks validation or disagrees with its current starting pose")
        self._clearance(start["joints_raw"])
        lower, upper = stage["joint_lower_raw"], stage["joint_upper_raw"]
        if any(not lo <= q <= hi for q, lo, hi in zip(start["joints_raw"], lower, upper)):
            raise core.Rejected("Cartesian move starts outside its joint bounds")
        self.state.joint_box = lower, upper
        self.move_count += 1
        label = stage["label"]
        self.report["phase"] = "move_" + label
        entry = dict(kind="move", label=label, target_pose_raw=target, start=start,
                     joint_lower_raw=lower, joint_upper_raw=upper, command_sent=False, arrival_verified=False)
        self.report["stages"].append(entry)
        self.last_target = entry
        before = len(self.report["transmissions"])
        try:
            sent_at = self._send("pose_%02d" % self.move_count,
                encode_pose_frames(self.vendor.sdk, target, stage["speed_percent"]),
                lambda: (self.vendor.sdk.MotionCtrl_2(1, 2, 5, 0), self.vendor.sdk.EndPoseCtrl(*target)), 15.)
        finally:
            entry["target_frame_attempted"] = any(frame["id"] in ("0x152", "0x153", "0x154")
                for frame in self.report["transmissions"][before:])
        entry.update(command_sent=True, sent_at_monotonic_s=sent_at)
        print("SDK 末端运动 %d：%s，MOVE_L，5%% 速度。" % (self.move_count, label), flush=True)
        settled, last_record = [], 0.
        while time.monotonic() < self.guard.deadline:
            self.receiver.one(.01)
            sample = self._sample()
            tracking = motion_residuals(sample, reference, target)
            entry["last_motion_residuals"] = tracking
            if (tracking["position_corridor_error_mm"] > tracking["motion_limits"]["position_mm"] or
                    tracking["rotation_tracking_error_deg"] > tracking["motion_limits"]["rotation_deg"]):
                entry["motion_violation"] = dict(tracking, sample=sample)
                self.report["trace"].append(dict(sample, stage=label, motion_violation=tracking))
                raise core.Rejected("Measured Cartesian motion left its 5 mm / 0.5 degree corridor; residuals=" +
                                    str(tracking))
            now = time.monotonic()
            if now - last_record < .02:
                continue
            last_record = now
            self._clearance(sample["joints_raw"])
            entry["last_arrival_residuals"] = pose_residuals(sample, target)
            self.report["trace"].append(dict(sample, stage=label))
            if not self._arrived(sample, entry):
                settled = []
                continue
            settled.append(sample)
            if _stable(settled):
                self.receiver.drain()
                final = self._sample()
                if not self._arrived(final, entry):
                    settled = []
                    continue
                self._clearance(final["joints_raw"])
                entry.update(arrival_verified=True, final=final, last_arrival_residuals=pose_residuals(final, target))
                self.held = final
                return final
        raise core.Rejected("Cartesian arrival not verified within 15 seconds; residuals=" +
                            str(entry.get("last_arrival_residuals", "no post-command sample")))

    def execute_pose_plan(self, plan, capture_callback=None):
        try:
            if self.executed:
                raise core.Rejected("Cartesian batch may execute only once")
            self.executed = True
            reference = self._sample()
            self.observe(lambda: self.validate_pose_plan(plan, reference), timeout=30)
            if "open" not in self.gripper_purposes:
                raise core.Rejected("A verified 55 mm opening must precede the Cartesian batch")
            if any(stage["kind"] == "capture" for stage in plan["stages"]) and capture_callback is None:
                raise core.Rejected("Planned camera checkpoints require a callback before execution")
            for stage in plan["stages"]:
                self._budget()
                if stage["kind"] == "move":
                    self.move_pose(stage)
                elif stage["kind"] == "gripper":
                    result = self.gripper(stage["width_raw"], stage["label"])
                    if result["classification"] == "ambiguous":
                        raise core.Rejected("Ambiguous gripper contact; no subsequent targets will be sent")
                else:
                    label, sample = stage["label"], self._sample()
                    result = self.observe(lambda: capture_callback(label, sample), timeout=30)
                    if not isinstance(result, dict):
                        raise core.Rejected("Camera checkpoint must return a diagnostic dictionary")
                    self.report["captures"].append(dict(label=label, sample=sample, result=result))
                    if result.get("abort"):
                        raise core.Rejected("Camera checkpoint requested stop: " + str(result.get("reason", label)))
            self.report.update(status="protocol_completed", protocol_completed=True, phase="complete")
            return self.report
        except BaseException as exc:
            self.fail(exc)
            raise
