#!/usr/bin/env python3
"""Capture serial-bound RGBD; default to front and right wrist, without ROS.

Run with a Python environment containing numpy, cv2 and pyrealsense2. Imports
the independent project relative to this file, so no PYTHONPATH setup is needed.
"""
import argparse
import hashlib
import json
import platform
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def write_json(path, content):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(content, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description="拍摄前视与右腕相机的 RGB 和对齐深度；不访问 ROS、CAN 或机械臂。")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "site.local.json")
    parser.add_argument("--output-root", type=Path, default=PROJECT_ROOT / "runs")
    parser.add_argument("--cameras", nargs="+", choices=("front", "right_hand", "left_hand"),
                        default=("front", "right_hand"),
                        help="only open these serial-bound cameras (default: front right_hand)")
    args = parser.parse_args(argv)
    if len(set(args.cameras)) != len(args.cameras):
        parser.error("camera names must not be repeated")
    started_at = time.time()
    run_id = "scene_" + uuid.uuid4().hex
    run_dir = args.output_root.resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    report_path = run_dir / "report.json"
    report = {
        "schema_version": 1, "run_id": run_id, "run_directory": str(run_dir),
        "operation": "one_serial_bound_rgbd_capture", "status": "started",
        "requested_cameras": list(args.cameras),
        "started_at": started_at,
        "started_at_utc": datetime.fromtimestamp(started_at, timezone.utc).isoformat(),
        "source": "user_terminal_direct_realsense_capture",
        "config_source": str(args.config.resolve()),
        "python_executable": sys.executable, "python_version": platform.python_version(),
        "machine": platform.node(), "platform": platform.platform(),
        "backend": "pyrealsense2", "robot_state": None,
        "robot_state_status": "not_observed",
        "robot_connection_attempted": False, "motion_commands_sent": 0,
        "ros_connection_attempted": False, "model_calls": 0,
        "video_recorded": False, "physical_grasp_attempts": 0,
        "calibration": {
            "camera_geometry_changed": True, "old_extrinsics_valid": False,
            "old_extrinsics_applied": False, "robot_base_transform": None,
            "reason": "User changed front camera pitch; previous extrinsics are invalid for this capture.",
            "evidence_source": "user_statement_in_current_conversation",
        },
    }
    write_json(report_path, report)
    rig = None
    observation = None
    exit_code = 1
    try:
        config_bytes = args.config.read_bytes()
        config = json.loads(config_bytes.decode("utf-8"))
        report["config_sha256"] = hashlib.sha256(config_bytes).hexdigest()
        bindings = config["cameras"]
        names = tuple(args.cameras)
        missing = [name for name in names if name not in bindings]
        if missing:
            raise ValueError("Missing configured camera bindings: " + ", ".join(missing))
        # Use only camera settings; robot/model settings are neither copied nor used.
        camera_configs = {
            name: {key: value for key, value in bindings[name].items()
                   if key in ("serial", "width", "height", "fps")}
            for name in names
        }
        for camera_config in camera_configs.values():
            camera_config["depth_enabled"] = True
        report["effective_cameras"] = camera_configs
        report["depth_configuration"] = "explicitly enabled for this RGBD-only capture request"
        write_json(run_dir / "camera_config.json", camera_configs)
        write_json(report_path, report)
        sys.path.insert(0, str(PROJECT_ROOT / "src"))
        from right_pick.camera import RealSenseRig
        rig = RealSenseRig(camera_configs, depth_enabled=True)
        observation = rig.capture(run_dir / "observations")
        report.update(status="capture_succeeded", observation_metadata=observation["metadata_path"],
                      captured_at=observation["captured_at"],
                      timestamp_kind=observation["timestamp_kind"],
                      cameras=list(observation["cameras"]),
                      host_receipt_skew_s=observation["host_receipt_skew_s"],
                      hardware_synchronized=observation["hardware_synchronized"])
        exit_code = 0
    except KeyboardInterrupt:
        report.update(status="interrupted", error={"type": "KeyboardInterrupt", "message": "Camera capture interrupted by user"})
        exit_code = 130
    except Exception as exc:
        report.update(status="capture_failed", error={"type": type(exc).__name__, "message": str(exc)})
    finally:
        if rig is not None:
            try:
                rig.close()
                report["camera_close_errors"] = list(rig.close_errors)
            except Exception as exc:
                report["camera_close_errors"] = [{"type": type(exc).__name__, "message": str(exc)}]
            if report["camera_close_errors"] and exit_code == 0:
                report["status"] = "capture_succeeded_cleanup_error"
                exit_code = 2
        report["finished_at"] = time.time()
        report["elapsed_s"] = report["finished_at"] - started_at
        write_json(report_path, report)
    summary = {"status": report["status"], "run_directory": str(run_dir), "report": str(report_path)}
    if observation is not None:
        summary["observation_metadata"] = observation["metadata_path"]
        summary["rgb_images"] = {name: frame["rgb_path"] for name, frame in observation["cameras"].items()}
    if "error" in report:
        summary["error"] = report["error"]
    if report.get("camera_close_errors"):
        summary["camera_close_errors"] = report["camera_close_errors"]
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
