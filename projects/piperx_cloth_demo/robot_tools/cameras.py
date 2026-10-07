"""Read-only RealSense snapshots. No detection, calibration, or robot commands."""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

# Confirmed in piper_right_pick_demo/configs/site.local.json (2026-10-03).
DEFAULT_CAMERA_SERIALS = {
    "front": "243622070374",
    "left_wrist": "336222071115",
    "right_wrist": "244222070415",
}
MAX_RECEIVE_SKEW_S = 0.2


class CameraCaptureError(RuntimeError):
    """The report includes partial captures and cleanup failures, never success."""

    def __init__(self, report: dict):
        self.report = report
        super().__init__(report.get("error", "Incomplete camera capture"))


def _validate(cameras: dict[str, str]) -> None:
    if not isinstance(cameras, dict) or set(cameras) != set(DEFAULT_CAMERA_SERIALS):
        raise ValueError("Require front, left_wrist, and right_wrist camera serials")
    if any(not isinstance(s, str) or not re.fullmatch(r"[0-9]+", s)
           for s in cameras.values()):
        raise ValueError("Camera serials must be nonempty digit strings")
    if len(set(cameras.values())) != len(cameras):
        raise ValueError("Every camera must have a distinct serial")


def _intrinsics(frame) -> dict:
    i = frame.profile.as_video_stream_profile().get_intrinsics()
    return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy,
                ppx=i.ppx, ppy=i.ppy, model=str(i.model), coeffs=list(i.coeffs))


def _frame_info(frame) -> dict:
    return {"frame_number": frame.get_frame_number(),
            "device_timestamp_ms": frame.get_timestamp(),
            "timestamp_domain": str(frame.get_frame_timestamp_domain())}


def _fresh_batch(pipelines: dict) -> tuple[dict, int]:
    """Drain buffered frames, then receive a batch with bounded host receive skew.

    Host receive times are NOT exposure times or cross-device clock calibration.
    Pipelines run concurrently; no first camera is held throughout others' warmup.
    """
    for attempt in range(1, 4):
        for pipeline in pipelines.values():
            for _ in range(32):
                if not pipeline.poll_for_frames():
                    break
            else:
                raise RuntimeError("Camera frame queue could not be drained")
        batch = {}
        for name, pipeline in pipelines.items():
            frames = pipeline.wait_for_frames(2000)
            batch[name] = (frames, time.monotonic(), time.time())
        times = [entry[1] for entry in batch.values()]
        if max(times) - min(times) <= MAX_RECEIVE_SKEW_S:
            return batch, attempt
    raise RuntimeError("Host receive skew exceeded 0.2 s in all three attempts")


def capture_cameras(output_dir: Path, cameras: dict[str, str],
                    include_depth: bool = True) -> dict:
    """Capture all three bound cameras to a NEW directory, then release devices.

    RGB PNG is 640x480 at a 15 fps stream setting. Optional depth is aligned by
    the RealSense driver to color, float32 metres. Raw zero and the uint16 encoding
    limit are stored as NaN; the latter is a conservative adapter policy, not an
    assertion about a manufacturer sentinel. File completeness does not imply
    usable depth, accurate range, or free space. See each camera's depth_quality.
    Raises CameraCaptureError on device, capture, write, or cleanup failures; its
    .report and output_dir/observation.json describe any partial result.
    All serials are checked before any pipeline starts. No hardware sync claimed.
    """
    _validate(cameras)
    if not isinstance(include_depth, bool):
        raise ValueError("include_depth must be a bool")
    output_dir = Path(output_dir).expanduser().absolute()
    output_dir.mkdir(parents=True, exist_ok=False)
    report = {"complete": False, "hardware_synchronized": False,
              "complete_scope": "requested frame files captured; depth usability is separate",
              "skew_basis": "host_receive_monotonic_not_exposure",
              "device_clocks_cross_calibrated": False,
              "exposure_skew_s": None, "exposure_age_s": None,
              "max_allowed_host_receive_skew_s": MAX_RECEIVE_SKEW_S,
              "requested_cameras": cameras.copy(), "cameras": {},
              "startup_attempted_cameras": [], "started_cameras": [],
              "cleanup_errors": [], "cleanup_results": {}, "phase": "dependencies",
              "started_host_unix_s": time.time(), "output_dir": str(output_dir)}
    pipelines, profiles = {}, {}
    failure = None
    try:
        import pyrealsense2 as rs
        import numpy as np
        import cv2

        report["phase"] = "serial_preflight"
        context = rs.context()
        devices = list(context.query_devices())
        available = {d.get_info(rs.camera_info.serial_number) for d in devices}
        report["available_serials"] = sorted(available)
        missing = sorted(set(cameras.values()) - available)
        if missing:
            raise RuntimeError("Missing camera serials before startup: " + ", ".join(missing))
        for name, serial in cameras.items():
            report["phase"] = "startup:" + name
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            if include_depth:
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            pipeline = rs.pipeline()
            # start() can throw after acquiring resources; always attempt stop().
            pipelines[name] = pipeline
            report["startup_attempted_cameras"].append(name)
            profile = pipeline.start(config)
            profiles[name] = profile
            report["started_cameras"].append(name)
            actual = profile.get_device().get_info(rs.camera_info.serial_number)
            if actual != serial:
                raise RuntimeError(f"Serial mismatch for {name}: {actual} != {serial}")
        report["phase"] = "warmup_and_capture"
        for _ in range(5):
            for pipeline in pipelines.values():
                pipeline.wait_for_frames(2000)
        batch, attempts = _fresh_batch(pipelines)
        receive_times = [entry[1] for entry in batch.values()]
        report["host_receive_skew_s"] = max(receive_times) - min(receive_times)
        report["capture_attempts"] = attempts
        for name, (frames, monotonic_s, unix_s) in batch.items():
            report["phase"] = "save:" + name
            if include_depth:
                frames = rs.align(rs.stream.color).process(frames)
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError(f"Missing color frame: {name}")
            item = {"serial": cameras[name], "rgb_path": None, "depth_path": None,
                    "host_receive_monotonic_s": monotonic_s,
                    "host_receive_unix_s": unix_s,
                    "color": _frame_info(color), "intrinsics": _intrinsics(color),
                    "stream": {"width": 640, "height": 480, "fps": 15},
                    "depth_quality": {"status": "pending" if include_depth else "not_requested"}}
            report["cameras"][name] = item
            rgb_path = output_dir / f"{name}.png"
            if not cv2.imwrite(str(rgb_path), np.asanyarray(color.get_data())):
                raise RuntimeError(f"Could not write RGB image: {name}")
            item["rgb_path"] = str(rgb_path)
            if include_depth:
                depth = frames.get_depth_frame()
                if not depth:
                    raise RuntimeError(f"Missing aligned depth frame: {name}")
                scale = profiles[name].get_device().first_depth_sensor().get_depth_scale()
                if not 0 < scale < 1:
                    raise RuntimeError(f"Invalid depth scale: {name}: {scale}")
                raw = np.asanyarray(depth.get_data())
                zero = raw == 0
                at_limit = raw == np.iinfo(np.uint16).max
                usable = ~(zero | at_limit)
                total, valid = int(raw.size), int(np.count_nonzero(usable))
                zero_count, limit_count = int(np.count_nonzero(zero)), int(np.count_nonzero(at_limit))
                flags = (["zero_depth"] if zero_count else []) + (["at_encoding_limit"] if limit_count else [])
                if valid == 0:
                    flags.append("no_usable_depth")
                item["depth_quality"] = {
                    "status": "unavailable" if not valid else ("partial" if valid < total else "available"),
                    "total_pixels": total, "valid_pixels": valid,
                    "zero_pixels": zero_count, "at_encoding_limit_pixels": limit_count,
                    "valid_fraction": valid / total if total else 0.0,
                    "quality_flags": flags, "at_encoding_limit_raw": int(np.iinfo(np.uint16).max),
                    "validity_scope": "encoding filter only; range and accuracy unverified",
                    "encoding_limit_semantics": "unavailable by adapter policy; manufacturer sentinel unverified",
                    "unknown_depth_is_free_space": False}
                metres = raw.astype(np.float32) * np.float32(scale)
                metres[~usable] = np.nan
                depth_path = output_dir / f"{name}.depth_m.npy"
                np.save(str(depth_path), metres, allow_pickle=False)
                item.update(depth_path=str(depth_path), depth=_frame_info(depth),
                            depth_aligned_to="color", depth_unit="metre",
                            invalid_depth="NaN", depth_scale_m=float(scale))
    except BaseException as exc:
        failure = exc
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        for name, pipeline in pipelines.items():
            try:
                pipeline.stop()
                report["cleanup_results"][name] = "stop_returned"
            except Exception as exc:
                report["cleanup_errors"].append(f"{name}: {type(exc).__name__}: {exc}")
                report["cleanup_results"][name] = "stop_failed_resource_state_unknown"
        finished = time.monotonic()
        for item in report["cameras"].values():
            item["host_receive_age_at_report_s"] = finished - item["host_receive_monotonic_s"]
        report["finished_host_unix_s"] = time.time()
        qualities = [item["depth_quality"]["status"] for item in report["cameras"].values()]
        report["depth_quality_status"] = ("not_requested" if not include_depth else
            "unavailable" if not qualities or all(q == "unavailable" for q in qualities) else
            "available" if len(qualities) == len(cameras) and all(q == "available" for q in qualities) else "partial")
        report["complete"] = (failure is None and not report["cleanup_errors"]
                              and len(report["cameras"]) == len(cameras))
        if report["complete"]:
            report["phase"] = "complete"
        if report["cleanup_errors"] and failure is None:
            report["error"] = "Camera cleanup failed: " + "; ".join(report["cleanup_errors"])
        try:
            (output_dir / "observation.json").write_text(
                json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
        except Exception as exc:
            report["complete"] = False
            report["report_write_error"] = f"{type(exc).__name__}: {exc}"
            if failure is None:
                failure, report["error"] = exc, report["report_write_error"]
    if not report["complete"]:
        if isinstance(failure, (KeyboardInterrupt, SystemExit)):
            raise failure
        raise CameraCaptureError(report) from failure
    return report
