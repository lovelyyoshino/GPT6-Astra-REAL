#!/usr/bin/env python3
"""Live SDK commissioning: enable, measure held pose, solve one +15 mm lift.

No historical starting pose, arbitrary target input, ROS, reset, disable,
gripper command, or automatic recovery. Leaving the program does not cancel
an already accepted target. Only an on-site operator may start this script.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time

import direct_sdk_step as core

INITIAL_LIMITS = ((-150, 150), (-3, 180), (-170, 4), (-100, 100), (-70, 70), (-120, 120))


class LiveFeedback(core.Feedback):
    def __init__(self, vendor):
        super().__init__(vendor)
        self.joint_box = None

    def validate_joint_range(self, joints):
        if any(not lo <= q / 1000. <= hi for q, (lo, hi) in zip(joints, INITIAL_LIMITS)):
            raise core.Rejected("Live joint position is outside the bounded commissioning range")
        if self.joint_box and any(not lo <= q <= hi for q, lo, hi in zip(joints, *self.joint_box)):
            raise core.Rejected("Measured joints left this run's computed small-step interval")


def encode_joint_plan(sdk, target=None):
    attribute = "_C_PiperInterface_V2__arm_can"
    original = getattr(sdk, attribute)
    capture = core.RecordingPort()
    setattr(sdk, attribute, capture)
    try:
        sdk.EnableArm(7)
        enable = capture.frames[:]
        capture.frames.clear()
        if target is not None:
            if len(target) != 6 or not all(type(q) is int for q in target) or not core.in_nominal_range(target):
                raise core.Rejected("Computed joint target must be six nominal-range integers")
            sdk.MotionCtrl_2(1, 1, 5, 0)
            sdk.JointCtrl(*target)
        motion = capture.frames[:]
    finally:
        setattr(sdk, attribute, original)
    if [i for i, _ in enable] != [0x471] or (target is not None and
            [i for i, _ in motion] != [0x151, 0x155, 0x156, 0x157]):
        raise core.Rejected("Unexpected SDK encoding")
    return {"enable": enable, "motion": motion}


def require_no_drift(sample, reference):
    pos, _ = core.pose_difference(sample["pose_raw"], reference["pose_raw"])
    if (pos > 2 or core.rotation_distance_deg(sample["pose_raw"], reference["pose_raw"]) > .5 or
            max(abs(a - b) for a, b in zip(sample["joints_raw"], reference["joints_raw"])) > 500):
        raise core.Rejected("Arm moved while enabling/holding; no new movement target will be sent")


def require_same_held_pose(sample, reference):
    require_no_drift(sample, reference)
    if max(abs(a - b) for a, b in zip(sample["joints_raw"], reference["joints_raw"])) > 150:
        raise core.Rejected("Held pose changed during planning by more than 0.15 degree")


def require_gripper_geometry(state, now):
    frame = state.raw_frame_latest.get("0x2A8")
    if (not frame or frame["origin"] != "nonlocal" or
            not 0 <= now - frame["kernel_received_monotonic_s"] < core.MAX_AGE_S or
            not state.gripper or not 0 <= state.gripper["angle_raw"] <= 70000):
        raise core.Rejected("Fresh gripper width must lie in the modeled 0–70 mm interval")


def compute_while_receiving(planner, held, receiver, state):
    """Pure offline calculation in a daemon worker; CAN freshness stays monitored."""
    outcome = {}
    complete = threading.Event()

    def calculate():
        try:
            outcome["plan"] = planner(list(held["joints_raw"]), list(held["pose_raw"]))
        except Exception as exc:
            outcome["error"] = exc
        finally:
            complete.set()

    threading.Thread(target=calculate, daemon=True).start()
    deadline = time.monotonic() + 8.
    while not complete.is_set():
        if time.monotonic() >= deadline:
            raise core.Rejected("Offline planner exceeded eight seconds; no motion target sent")
        receiver.one(.01)
        require_same_held_pose(state.snapshot(time.monotonic(), True), held)
        require_gripper_geometry(state, time.monotonic())
    if "error" in outcome:
        raise outcome["error"]
    receiver.drain()
    require_same_held_pose(state.snapshot(time.monotonic(), True), held)
    return outcome["plan"]


def validate_plan(plan, held):
    target = plan["target_joints_raw"]
    if (plan["start_joints_raw"] != held["joints_raw"] or plan["start_pose_raw"] != held["pose_raw"] or
            len(target) != 6 or not all(type(q) is int for q in target) or not core.in_nominal_range(target) or
            max(abs(a - b) for a, b in zip(target, held["joints_raw"])) > 4000):
        raise core.Rejected("Planner output exceeds the single live-pose lift policy")
    goal = list(held["pose_raw"])
    goal[2] += 15000
    if plan["target_pose_raw"] != goal:
        raise core.Rejected("Planner changed the required +15 mm, same-orientation goal")
    bounds = plan["model_bounds"]
    if not all(math.isfinite(bounds[key]) for key in ("end_xy_max_mm", "end_z_delta_min_mm",
            "end_z_delta_max_mm", "orientation_max_deg", "gripper_point_dip_max_mm")):
        raise core.Rejected("Planner returned a non-finite motion bound")
    lower = [min(a, b) - 150 for a, b in zip(held["joints_raw"], target)]
    upper = [max(a, b) + 150 for a, b in zip(held["joints_raw"], target)]
    if bounds["joint_lower_raw"] != lower or bounds["joint_upper_raw"] != upper:
        raise core.Rejected("Planner interval differs from the 0.15-degree padded joint interval")
    if (bounds["end_xy_max_mm"] > 3 or bounds["end_z_delta_min_mm"] < -7 or
            bounds["end_z_delta_max_mm"] > 23 or bounds["orientation_max_deg"] > 5 or
            bounds["gripper_point_dip_max_mm"] > 15):
        raise core.Rejected("Planner swept-motion samples exceed the reviewed clearance budget")
    return lower, upper


def check_motion(sample, held):
    # The geometric corridor is shared with the audited small joint-space lift,
    # but the legacy fixed-joint box is intentionally not used.
    pose, origin = sample["pose_raw"], held["pose_raw"]
    if (math.hypot(pose[0] - origin[0], pose[1] - origin[1]) > 3000 or
            not -7000 <= pose[2] - origin[2] <= 23000 or core.rotation_distance_deg(pose, origin) > 5):
        raise core.Rejected("Measured end pose left this run's bounded lift corridor")


def arrived(sample, state, plan, sent_at):
    pos, _ = core.pose_difference(sample["pose_raw"], plan["target_pose_raw"])
    return (pos <= 2 and core.rotation_distance_deg(sample["pose_raw"], plan["target_pose_raw"]) <= .1 and
            core.in_nominal_range(sample["joints_raw"]) and
            max(abs(a - b) for a, b in zip(sample["joints_raw"], plan["target_joints_raw"])) <= 100 and
            all(sample["motor_enabled"]) and sample["status"]["ctrl_mode"] == 1 and
            sample["status"]["mode_feed"] == 1 and sample["status"]["motion_status"] == 0 and
            all(state.received[i] > sent_at for i in core.REQUIRED))


def observe_after_failure(receiver, state, report, deadline):
    """Keep observing an already sent target; this function has no SDK/port."""
    print("停止下发，继续只读监测；不保证取消已发送目标。", flush=True)
    report["post_failure_observations"] = []
    report["post_failure_first_diagnostic"] = state.diagnostic(time.monotonic())
    report["post_failure_receive_errors"] = []
    report["post_failure_receive_error_count"] = 0
    report["post_failure_target_stable"] = False
    settled, last_record = [], 0.
    while time.monotonic() < deadline:
        try:
            receiver.one(min(.01, max(.001, deadline - time.monotonic())))
        except KeyboardInterrupt:
            report["post_failure_monitor_interrupted"] = True
            break
        except Exception as exc:
            report["post_failure_receive_error_count"] += 1
            if len(report["post_failure_receive_errors"]) < 20:
                report["post_failure_receive_errors"].append({"error": str(exc),
                    "type": type(exc).__name__, "monotonic_s": time.monotonic()})
            # A closed/broken socket can fail immediately on every read.
            if isinstance(exc, OSError):
                time.sleep(min(.01, max(0., deadline - time.monotonic())))
        now = time.monotonic()
        if now - last_record < .02:
            continue
        last_record = now
        diagnostic = state.diagnostic(now)
        observation = {"monotonic_s": now, "pose_raw": diagnostic["pose_raw"],
                       "joints_raw": diagnostic["joints_raw"], "status": diagnostic["status"],
                       "motors": diagnostic["motors"], "frame_age_s": diagnostic["required_frame_age_s"]}
        try:
            sample = state.snapshot(now, True)
            if not report.get("motion_target_sent", False):
                observation["target_assessment"] = "unavailable: complete target was not sent"
                settled = []
                report["post_failure_target_stable"] = False
            elif arrived(sample, state, report["plan"], report["motion_sent_at_monotonic_s"]):
                settled.append(sample)
                report["post_failure_target_stable"] = core.stationary(settled)
            else:
                settled = []
                report["post_failure_target_stable"] = False
        except Exception as exc:
            observation["validation_error"] = str(exc)
            settled = []
            report["post_failure_target_stable"] = False
        report["post_failure_observations"].append(observation)
    report["post_failure_final_diagnostic"] = state.diagnostic(time.monotonic())
    report["post_failure_monitor_finished_at_monotonic_s"] = time.monotonic()
    report["post_failure_assessment_note"] = "Diagnostic only; original failure and arrival_verified=false remain unchanged"


def run_live(vendor, receiver, state, report, planner=None):
    print("阶段 1：读取当前姿态与状态，随后直接验证 SDK 使能。", flush=True)
    report["phase"] = "preflight"
    initial = core.collect_stationary(receiver, state, report["trace"])
    report["initial"] = initial
    core.inspect_controllers()
    binding = core.inspect_binding()
    if binding["ifindex"] != report["binding"]["ifindex"]:
        raise core.Rejected("CAN binding changed during preflight")
    vendor.sdk.CreateCanBus(core.CHANNEL, expected_bitrate=1000000, judge_flag=False)
    transport = getattr(vendor.sdk, "_C_PiperInterface_V2__arm_can")
    guard = core.TransmitGuard(transport, encode_joint_plan(vendor.sdk), state, receiver, report,
                               command_limits={"enable": 20, "motion": 1})
    transport.SendCanMessage = guard.send
    try:
        report["phase"] = "enabling"
        deadline = time.monotonic() + 2.
        guard.deadline = deadline
        next_send = 0.
        last_send = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send and guard.calls["enable"] < 20:
                guard.allow("enable")
                vendor.sdk.EnablePiper()
                report["enable_transmissions"] = guard.calls["enable"]
                last_send = report["transmissions"][-1]["monotonic_s"]
                next_send = now + .1
            receiver.one(.01)
            sample = state.snapshot(time.monotonic())
            require_no_drift(sample, initial)
            if (last_send is not None and all(sample["motor_enabled"]) and
                    all(state.received[i] > last_send for i in range(0x261, 0x267))):
                report["enable_verified"], report["enable_final"] = True, sample
                break
        if not report["enable_verified"]:
            raise core.Rejected("SDK enable not verified within two seconds; no joint target sent")
        guard.deadline = None
        print("SDK 使能成功：六个关节均由发送后的新鲜反馈确认。", flush=True)
        guard.require_enabled = True
        report["phase"] = "holding"
        held = core.collect_stationary(receiver, state, report["trace"], require_enabled=True)
        require_no_drift(held, initial)
        require_gripper_geometry(state, time.monotonic())
        report["held"] = held
        report["phase"] = "planning"
        print("阶段 2：以现在稳定的姿态求解唯一的 15 mm 抬升目标。", flush=True)
        if planner is None:
            from live_lift_plan import plan_lift
            planner = plan_lift
        plan = compute_while_receiving(planner, held, receiver, state)
        state.joint_box = validate_plan(plan, held)
        report["plan"] = plan
        receiver.drain()
        current = state.snapshot(time.monotonic(), True)
        require_same_held_pose(current, held)
        require_gripper_geometry(state, time.monotonic())
        guard.plan = encode_joint_plan(vendor.sdk, plan["target_joints_raw"])
        print("当前 XYZ(mm)/RPY(deg)：", [q / 1000. for q in held["pose_raw"]], flush=True)
        print("目标 XYZ(mm)/RPY(deg)：", [q / 1000. for q in plan["target_pose_raw"]], flush=True)
        print("目标关节角(deg)：", [q / 1000. for q in plan["target_joints_raw"]], flush=True)
        report["phase"] = "motion"
        guard.deadline = time.monotonic() + 5.
        report["motion_deadline_monotonic_s"] = guard.deadline
        guard.allow("motion")
        vendor.sdk.MotionCtrl_2(1, 1, 5, 0)
        vendor.sdk.JointCtrl(*plan["target_joints_raw"])
        if guard.pending:
            raise core.Rejected("SDK did not send the entire joint target")
        report["motion_target_sent"] = True
        sent_at = time.monotonic()
        report["motion_sent_at_monotonic_s"] = sent_at
        settled, last_record = [], 0.
        while time.monotonic() < guard.deadline:
            receiver.one(.01)
            now = time.monotonic()
            sample = state.snapshot(now, True)
            check_motion(sample, held)
            require_gripper_geometry(state, now)
            if now - last_record >= .01:
                report["trace"].append(sample)
                last_record = now
                if arrived(sample, state, plan, sent_at):
                    settled.append(sample)
                    if core.stationary(settled):
                        receiver.drain()
                        final = state.snapshot(time.monotonic(), True)
                        check_motion(final, held)
                        if arrived(final, state, plan, sent_at):
                            report.update(status="arrived", arrival_verified=True, final=final, phase="complete")
                            print("阶段 3 完成：实际反馈确认到达并稳定。", flush=True)
                            return
                        settled = []
                else:
                    settled = []
        raise core.Rejected("Five-second motion deadline: actual arrival was not verified")
    except Exception as exc:
        joint_frame_attempted = any(entry["id"] in ("0x155", "0x156", "0x157")
                                    for entry in report["transmissions"])
        if report["motion_target_sent"] or joint_frame_attempted:
            report["partial_motion_target"] = joint_frame_attempted and not report["motion_target_sent"]
            report["post_failure_trigger"] = {"phase": report["phase"], "error": str(exc),
                                              "monotonic_s": time.monotonic()}
            # Irrevocably seal the transmission guard before receiving more.
            guard.pending.clear()
            guard.plan = {}
            guard.command_limits = {}
            guard.deadline = 0.
            observe_after_failure(receiver, state, report, report["motion_deadline_monotonic_s"])
        raise
    finally:
        transport.Close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    path = args.output_dir / "report.json"
    if path.exists():
        parser.error("Use a new output directory; an existing report will not be overwritten")
    report = {"status": "not_started", "phase": "preparation", "enable_verified": False,
              "enable_transmissions": 0, "motion_target_sent": False, "arrival_verified": False,
              "this_is_a_grasp": False, "transmissions": [], "trace": [], "started_at_s": time.time(),
              "initial_limits_deg": INITIAL_LIMITS, "exit_does_not_cancel_target": True,
              "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "core_source_sha256": hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest()}
    receiver, state, code = None, None, 2
    try:
        # Fail on missing dependencies before any enable command. Import the
        # pure planner only after successful enable as specified by the workflow.
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        if importlib.util.find_spec("live_lift_plan") is None:
            raise core.Rejected("live_lift_plan module is unavailable")
        report["binding"] = core.inspect_binding()
        core.inspect_controllers()
        vendor = core.Vendor()
        report["sdk_sources_sha256"] = vendor.sources
        print("一次 SDK 验证：使能六关节 → 读取实时姿态 → 求解并执行一次约 15 mm 抬升，5% 速度。")
        print("夹爪保持空，基座正装；运动部位、夹爪、腕相机、线缆与桌面及周围物体各方向至少 30 mm 净空。")
        print("运动中可能短暂向下约 15 mm。请移开手和运动范围内的支撑物，准备必要时现场断电并防止下落。")
        print("按 Enter 后不再手动调整。没有第二次等待；脚本退出不会自动失能/复位，也不取消已接受目标。")
        if not sys.stdin.isatty() or input("准备好后按 Enter 开始；输入任何文字取消：").strip():
            raise core.Rejected("Operator cancelled or no interactive terminal")
        state = LiveFeedback(vendor)
        receiver = core.Receiver(state)
        run_live(vendor, receiver, state, report)
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        if isinstance(exc, KeyboardInterrupt):
            code = 130
        report.update(status="interrupted" if code == 130 else "failed", error=str(exc) or type(exc).__name__,
                      error_type=type(exc).__name__)
        print("结果：" + report["error"], file=sys.stderr)
        if report["enable_verified"]:
            print("曾确认六关节使能；脚本不会主动失能，实际状态以当前反馈为准。" + ("运动目标已发送，请观察实际机械臂。" if report["motion_target_sent"]
                  else "尚未发送完整运动目标。"), file=sys.stderr)
        if report["transmissions"]:
            print("\a已停止发送新指令；退出程序不取消已接受的目标，必要时现场处理。", file=sys.stderr)
    finally:
        if receiver:
            receiver.close()
        if state:
            report["latest_unqualified_feedback"] = state.diagnostic(time.monotonic())
            report["feedback_frame_counts"] = dict(state.counts)
            report["latched_fault"] = state.fault
        report["enable_transmissions"] = sum(e["id"] == "0x471" for e in report["transmissions"])
        report["frames_send_attempted"] = len(report["transmissions"])
        report["finished_at_s"] = time.time()
        path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
        print("记录：" + str(path))
    return code


if __name__ == "__main__":
    sys.exit(main())
