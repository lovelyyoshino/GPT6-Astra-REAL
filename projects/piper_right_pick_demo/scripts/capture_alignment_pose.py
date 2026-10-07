#!/usr/bin/env python3
"""Human-operated front RGBD preview with receive-only CAN pose capture.

No robot command, controller, interface activation, or calibration fitting.
Press S at a stationary pose to save evidence; Q closes only this observer.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def assess_trace(trace, image_time):
    """Reject moving/stale/unbracketed evidence; never authorize robot motion."""
    import numpy as np
    result = {"stationary_window_candidate": False,
              "calibration_accepted": False, "motion_authorized": False,
              "window_half_width_s": 0.4,
              "limits_are": "diagnostic filtering only, not calibrated grasp tolerances",
              "timing_basis": "host receipt monotonic; hardware exposure timing unverified"}
    try:
        if not math.isfinite(image_time):
            raise ValueError("Invalid image receipt timestamp")
        samples = [s for s in trace
                   if abs(s["received_monotonic_s"] - image_time) <= .4]
        if len(samples) < 20:
            raise ValueError("Insufficient CAN samples around image")
        timestamps = np.array([s["received_monotonic_s"] for s in samples])
        if not np.isfinite(timestamps).all() or (np.diff(timestamps) <= 0).any():
            raise ValueError("CAN sample times are invalid or unordered")
        if timestamps[0] > image_time - .3 or timestamps[-1] < image_time + .3:
            raise ValueError("CAN trace does not bracket image with a stationary window")
        if float(np.diff(timestamps).max()) > .1:
            raise ValueError("Gap in CAN trace")
        joint_names = ["joint_%d" % i for i in range(1, 7)]
        end_names = ["X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"]
        names = joint_names + end_names
        ages = [s["received_monotonic_s"] - s["field_received_monotonic_s"][name]
                for s in samples for name in names]
        if not all(math.isfinite(age) and 0 <= age <= .1 for age in ages):
            raise ValueError("Incomplete or stale individual CAN fields")
        joints = np.array([[s["joints_raw"][k] for k in joint_names] for s in samples], dtype=float)
        poses = np.array([[s["end_pose_raw"][k] for k in end_names] for s in samples], dtype=float)
        if not np.isfinite(joints).all() or not np.isfinite(poses).all():
            raise ValueError("Non-finite CAN values")
        positions = poses[:, :3] * 1e-6
        euler = np.unwrap(np.deg2rad(poses[:, 3:] * .001), axis=0)
        joint_angles = np.unwrap(np.deg2rad(joints * .001), axis=0)
        translation_span = float(np.linalg.norm(np.ptp(positions, axis=0)))
        euler_span = float(np.rad2deg(np.ptp(euler, axis=0)).max())
        joint_span = float(np.rad2deg(np.ptp(joint_angles, axis=0)).max())
        nearest = min(samples, key=lambda s: abs(s["received_monotonic_s"] - image_time))
        result.update(samples_in_window=len(samples),
                      translation_span_m=translation_span,
                      largest_euler_component_span_deg=euler_span,
                      largest_joint_span_deg=joint_span,
                      maximum_field_age_s=max(ages),
                      nearest_sample_host_delta_s=abs(nearest["received_monotonic_s"] - image_time),
                      nearest_raw_sample=nearest,
                      end_reference="vendor_feedback_end_reference_not_grasp_TCP")
        if translation_span > .002 or euler_span > 1.0 or joint_span > 1.0:
            raise ValueError("Arm moved during image window")
        result.update(stationary_window_candidate=True,
                      reason="Receipt-time stationary window passed; visual feature and exposure timing still require review")
    except (KeyError, TypeError, ValueError) as exc:
        result["reason"] = str(exc)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/site.local.json")
    parser.add_argument("--max-seconds", type=float, default=600)
    args = parser.parse_args()
    if not math.isfinite(args.max_seconds) or not 10 <= args.max_seconds <= 1800:
        parser.error("--max-seconds must be within [10,1800]")
    config = json.loads(args.config.read_text())
    host = config["host_capture"]
    channel = host["right_can_interface"]
    if channel != "can2":
        raise RuntimeError("Expected inspected right can2 binding")
    device = Path("/sys/class/net") / channel / "device"
    if not device.exists() or device.resolve().name != host["right_usb_port"]:
        raise RuntimeError("Right CAN USB binding unavailable or changed")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise RuntimeError("Run in the local desktop terminal for the camera preview")
    import cv2
    import numpy as np
    sys.path.insert(0, str(ROOT / "src"))
    from right_pick.camera import RealSenseRig
    run = ROOT / "runs" / ("alignment_" + uuid.uuid4().hex)
    run.mkdir(parents=True, exist_ok=False)
    front = dict(config["cameras"]["front"], depth_enabled=True)
    report = {"operation": "human_operated_read_only_alignment_evidence",
              "started_at_s": time.time(), "status": "starting",
              "robot_control_commands_sent": 0, "can_frames_sent_by_observer": 0,
              "physical_grasp_attempts_by_demo": 0, "calibration_accepted": False,
              "controller_started": False, "interface_changed": False,
              "source_config": str(args.config.resolve()),
              "can_channel": channel, "right_usb_port": host["right_usb_port"],
              "front_serial": front["serial"], "samples": [],
              "note": "Q/timeout/error closes observer only. It does not stop or disable the separate teleoperation controller."}
    write_json(run / "report.json", report)
    rig = RealSenseRig({"front": front}, depth_enabled=True)
    process = None
    stdout_file = None
    stderr_file = None
    title = "Front camera - READ ONLY - S save, Q quit observer"
    deadline = time.monotonic() + args.max_seconds
    exit_code = 0

    def preview(status):
        frames = rig._streams["front"]["pipeline"].wait_for_frames(3000)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("Missing front RGB frame")
        pixels = np.asanyarray(color.get_data()).copy()
        cv2.putText(pixels, status, (8, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(title, pixels)
        return cv2.waitKey(1) & 0xff

    try:
        rig._open()
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        report["status"] = "previewing"
        write_json(run / "report.json", report)
        print("只读预览已启动。使用原有示教方式移动；本程序不控制机械臂。", flush=True)
        print("先让完整右夹爪进入画面，在桌面上方空处停稳。按 S 并保持静止约 3 秒；按 Q 退出观察。", flush=True)
        print("第一轮仅需一组清晰夹爪画面，不必抓方块。保存目录：" + str(run), flush=True)
        while time.monotonic() < deadline:
            key = preview("READ ONLY | S: save stationary pose | Q: close observer")
            if key == ord("q") or key == 27 or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key != ord("s"):
                continue
            sample_dir = run / ("sample_%02d" % (len(report["samples"]) + 1))
            sample_dir.mkdir(exist_ok=False)
            stdout_file = (sample_dir / "passive_can.json").open("w")
            stderr_file = (sample_dir / "passive_can.stderr.txt").open("w")
            process = subprocess.Popen([
                host["robot_python"], str(ROOT / "scripts/passive_can_snapshot.py"),
                "--channel", channel, "--seconds", "3", "--pose-trace"],
                stdout=stdout_file, stderr=stderr_file)
            wait_until = time.monotonic() + 1.2
            while time.monotonic() < wait_until and process.poll() is None:
                preview("Capturing evidence - HOLD STILL")
            observation = rig.capture(sample_dir / "observations")
            process_deadline = time.monotonic() + 5
            while process.poll() is None and time.monotonic() < process_deadline:
                preview("Capturing evidence - HOLD STILL")
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2)
            stdout_file.close()
            stderr_file.close()
            stdout_file = stderr_file = None
            code = process.returncode
            process = None
            can_data = json.loads((sample_dir / "passive_can.json").read_text())
            image_time = observation["cameras"]["front"]["host_received_monotonic_s"]
            assessment = assess_trace(can_data.get("pose_trace", []), image_time)
            if code != 0:
                assessment.update(stationary_window_candidate=False,
                                  reason="CAN collector failed; see saved CAN report")
            assessment.update(observation_metadata=observation["metadata_path"],
                              can_report=str(sample_dir / "passive_can.json"),
                              collector_exit_code=code,
                              same_rigid_visual_feature_identified=False,
                              T_right_base_front=None, T_vendor_end_tcp=None)
            write_json(sample_dir / "assessment.json", assessment)
            report["samples"].append({"directory": str(sample_dir),
                                      "stationary_window_candidate": assessment["stationary_window_candidate"]})
            write_json(run / "report.json", report)
            print("已保存第 %d 组，静止窗口检查：%s。%s" % (
                len(report["samples"]), assessment["stationary_window_candidate"],
                assessment["reason"]), flush=True)
            if len(report["samples"]) >= 30:
                break
        report["status"] = "observer_closed"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        exit_code = 130
    except Exception as exc:
        report.update(status="capture_failed", error={"type": type(exc).__name__, "message": str(exc)})
        print(str(exc), file=sys.stderr, flush=True)
        exit_code = 2
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        for handle in (stdout_file, stderr_file):
            if handle is not None:
                handle.close()
        rig.close()
        cv2.destroyAllWindows()
        report.update(finished_at_s=time.time(), camera_close_errors=rig.close_errors)
        write_json(run / "report.json", report)
        print("采集结果：" + str(run), flush=True)
        print("观察程序已结束；原有示教控制器的状态未改变。", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
