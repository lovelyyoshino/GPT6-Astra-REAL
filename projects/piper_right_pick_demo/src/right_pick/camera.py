"""Read-only, serial-bound RealSense observations and colour candidates.

No SDK is imported or device opened until capture(). Historical extrinsics are
never applied. Camera points describe a visible surface, not a grasp target.
The optional video contains sampled capture() frames; it is not continuous video.
"""

import importlib
import importlib.util
import json
import math
import re
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path


def _dependency(name):
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise RuntimeError("Optional camera dependency missing: " + name) from exc


def environment_probe():
    """Inspect availability only; do not open cameras or initialize the SDK."""
    return {
        "dependencies": {
            name: importlib.util.find_spec(name) is not None
            for name in ("numpy", "cv2", "pyrealsense2")
        },
        "video_device_nodes": sorted(str(p) for p in Path("/dev").glob("video*")),
        "usb_bus_visible": Path("/dev/bus/usb").exists(),
        "device_access_verified": False,
    }


def depth_to_meters(depth_raw, depth_scale_m):
    """Convert actual sensor units to optical-axis metres; invalid -> NaN."""
    np = _dependency("numpy")
    if not math.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError("depth_scale_m must be the positive sensor-reported scale")
    values = np.asarray(depth_raw)
    if values.ndim != 2:
        raise ValueError("depth image must be a 2-D array")
    result = values.astype(np.float32) * depth_scale_m
    result[(values <= 0) | ~np.isfinite(values)] = np.nan
    return result


def depth_pixel_query(depth_raw, depth_scale_m, pixel, intrinsics=None,
                      deprojector=None):
    """Return measured surface Z and, when calibrated, camera optical XYZ.

    deprojector(pixel, z_m) may supply the manufacturer's calibrated operation.
    Offline pinhole deprojection is permitted only without nonzero distortion.
    Zero/invalid pixels remain invalid; neighbouring surfaces are not substituted.
    No extrinsic, TCP, object-centre, or robot-base conversion is performed.
    """
    np = _dependency("numpy")
    values = np.asarray(depth_raw)
    if values.ndim != 2:
        raise ValueError("depth image must be a 2-D array")
    if not math.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError("invalid sensor depth scale")
    if len(pixel) != 2 or any(isinstance(v, bool) or not float(v).is_integer()
                              for v in pixel):
        raise ValueError("pixel must contain integer [u, v]")
    u, v = (int(pixel[0]), int(pixel[1]))
    if not (0 <= u < values.shape[1] and 0 <= v < values.shape[0]):
        raise ValueError("pixel outside aligned depth image")
    raw = float(values[v, u])
    result = {
        "pixel": [u, v], "valid": False, "depth_z_m": None,
        "camera_point_m": None, "frame": "camera_optical",
        "axes": "x right, y down, z forward",
        "distance_definition": "optical-axis Z, not radial range",
        "surface_only": True, "is_grasp_target": False,
        "extrinsics_applied": False,
        "deprojection_status": "not_attempted",
    }
    if not math.isfinite(raw) or raw <= 0:
        result["reason"] = "invalid measured depth; no neighbouring fill used"
        return result
    z_m = raw * depth_scale_m
    if not math.isfinite(z_m):
        result["reason"] = "depth conversion is not finite"
        return result
    result.update(valid=True, depth_z_m=z_m)
    if deprojector is not None:
        point = list(deprojector([u, v], z_m))
        if len(point) != 3 or not all(math.isfinite(float(x)) for x in point):
            raise ValueError("deprojection returned an invalid camera point")
        result["camera_point_m"] = [float(x) for x in point]
        result["deprojection_status"] = "manufacturer_sdk"
    elif intrinsics:
        coefficients = intrinsics.get("coeffs")
        model = str(intrinsics.get("model", "unknown")).lower()
        undistorted = model in ("none", "distortion.none") or (
            coefficients is not None and len(coefficients) > 0
            and all(float(c) == 0.0 for c in coefficients)
        )
        if not undistorted:
            result["reason"] = "calibrated SDK deprojection required for distortion"
            result["deprojection_status"] = "unsupported_distortion"
            return result
        fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
        ppx, ppy = float(intrinsics["ppx"]), float(intrinsics["ppy"])
        if not all(math.isfinite(x) for x in (fx, fy, ppx, ppy)) or min(fx, fy) <= 0:
            raise ValueError("invalid camera intrinsics")
        result["camera_point_m"] = [(u - ppx) / fx * z_m,
                                     (v - ppy) / fy * z_m, z_m]
        result["deprojection_status"] = "undistorted_pinhole"
    else:
        result["reason"] = "no calibrated intrinsics supplied; Z only"
    return result


def detect_red_candidates(image, color_order="bgr", min_area_px=20):
    """Find red connected regions, without claiming cube identity or success."""
    np, cv2 = _dependency("numpy"), _dependency("cv2")
    pixels = np.asarray(image)
    if pixels.ndim != 3 or pixels.shape[2] != 3 or pixels.dtype != np.uint8:
        raise ValueError("image must be an H x W x 3 uint8 array")
    if color_order not in ("bgr", "rgb"):
        raise ValueError("color_order must be bgr or rgb")
    if min_area_px <= 0:
        raise ValueError("min_area_px must be positive")
    hsv = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV if color_order == "bgr"
                       else cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 100, 60), (10, 255, 255))
    mask |= cv2.inRange(hsv, (170, 100, 60), (179, 255, 255))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    result = []
    for index in range(1, count):
        x, y, width, height, area = [int(n) for n in stats[index]]
        if area < min_area_px:
            continue
        result.append({
            "bbox_xywh": [x, y, width, height],
            "center_uv": [float(n) for n in centers[index]],
            "area_px": area, "bbox_fill_ratio": area / float(width * height),
            "kind": "red_colour_candidate", "object_identity_verified": False,
            "grasp_success": None,
        })
    return sorted(result, key=lambda c: c["area_px"], reverse=True)


def detect_red_candidates_file(path, min_area_px=20):
    """Analyze an already saved image, without loading RealSense or hardware."""
    cv2 = _dependency("cv2")
    pixels = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if pixels is None:
        raise ValueError("Could not read image: " + str(path))
    return detect_red_candidates(pixels, min_area_px=min_area_px)


def _intrinsics_dict(intrinsics):
    return {
        "width": intrinsics.width, "height": intrinsics.height,
        "fx": intrinsics.fx, "fy": intrinsics.fy,
        "ppx": intrinsics.ppx, "ppy": intrinsics.ppy,
        "model": str(intrinsics.model), "coeffs": list(intrinsics.coeffs),
        "source": "active_device_stream_profile",
    }


class RealSenseRig:
    """Explicit serial bindings; no SDK import or hardware access in __init__."""

    def __init__(self, camera_configs, depth_enabled=True, rgb_only=False):
        self.rgb_only = bool(rgb_only)
        self.configs = {}
        serials = set()
        for name, original in camera_configs.items():
            if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
                raise ValueError("camera names must be safe path components")
            item = dict(original)
            serial = str(item.get("serial") or "").strip()
            if not serial or serial in serials:
                raise ValueError("each camera requires a unique nonempty serial")
            serials.add(serial)
            item.update(serial=serial, depth_enabled=(False if self.rgb_only else bool(
                item.get("depth_enabled", depth_enabled))))
            for key, default in (("width", 640), ("height", 480), ("fps", 15)):
                item[key] = int(item.get(key, default))
                if item[key] <= 0:
                    raise ValueError(key + " must be positive")
            self.configs[name] = item
        if not self.configs:
            raise ValueError("at least one serial-bound camera is required")
        self._streams = {}
        self._latest = {}
        self._rs = None
        self._record_dir = None
        self._writers = {}
        self._video_frames = {}
        self._video_metadata = None
        self.close_errors = []

    def _open(self):
        if self._streams:
            return
        rs = self._rs = _dependency("pyrealsense2")
        try:
            for name, conf in self.configs.items():
                pipeline, config = rs.pipeline(), rs.config()
                config.enable_device(conf["serial"])
                config.enable_stream(rs.stream.color, conf["width"], conf["height"],
                                     rs.format.bgr8, conf["fps"])
                if conf["depth_enabled"]:
                    config.enable_stream(rs.stream.depth, conf["width"], conf["height"],
                                         rs.format.z16, conf["fps"])
                profile = pipeline.start(config)
                self._streams[name] = {"pipeline": pipeline}
                device = profile.get_device()
                actual_serial = device.get_info(rs.camera_info.serial_number)
                if actual_serial != conf["serial"]:
                    raise RuntimeError("Camera serial mismatch: " + name)
                depth_scale = (float(device.first_depth_sensor().get_depth_scale())
                               if conf["depth_enabled"] else None)
                self._streams[name].update(
                    align=rs.align(rs.stream.color) if conf["depth_enabled"] else None,
                    depth_scale_m=depth_scale,
                    device_model=device.get_info(rs.camera_info.name),
                )
            # Let auto-exposure settle. This is camera initialization only.
            for stream in self._streams.values():
                for _ in range(5):
                    stream["pipeline"].wait_for_frames(5000)
        except Exception:
            self.close()
            raise

    def capture(self, output_dir):
        """Save RGB, aligned sensor depth and honest timing/calibration metadata."""
        np, cv2 = _dependency("numpy"), _dependency("cv2")
        self._open()
        capture_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + "_" + uuid.uuid4().hex[:8]
        folder = Path(output_dir).resolve() / capture_id
        folder.mkdir(parents=True, exist_ok=False)
        result = {
            "capture_id": capture_id, "directory": str(folder), "cameras": {},
            "extrinsics_applied": False,
            "historical_extrinsics_used": False,
            "robot_base_transform": None,
            "calibration_note": "No camera-to-robot transform implemented; moved cameras require new verification.",
            "hardware_synchronized": False,
        }
        receipts = []
        for name, stream in self._streams.items():
            frames = stream["pipeline"].wait_for_frames(5000)
            received_at = time.time()
            received_monotonic = time.monotonic()
            if stream["align"] is not None:
                frames = stream["align"].process(frames)
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError("Missing RGB frame: " + name)
            bgr = np.asanyarray(color.get_data()).copy()
            color_intrinsics = None if self.rgb_only else color.profile.as_video_stream_profile().intrinsics
            rgb_path = folder / (name + ".png")
            if not cv2.imwrite(str(rgb_path), bgr):
                raise RuntimeError("Failed to save RGB frame: " + str(rgb_path))
            metadata = {
                "name": name, "serial": self.configs[name]["serial"],
                "model": stream["device_model"], "rgb_path": str(rgb_path),
                "image_encoding": "PNG RGB (OpenCV arrays BGR)",
                "host_received_at": received_at,
                "timestamp": received_at,
                "timestamp_kind": "host_receipt_wall_clock",
                "host_received_monotonic_s": received_monotonic,
                "device_timestamp_ms": float(color.get_timestamp()),
                "timestamp_domain": str(color.get_frame_timestamp_domain()),
                "frame_number": int(color.get_frame_number()),
                "depth_enabled": stream["align"] is not None,
                "exposure_age_verified": False,
            }
            if not self.rgb_only:
                metadata["intrinsics"] = _intrinsics_dict(color_intrinsics)
                metadata["red_candidates"] = detect_red_candidates(bgr)
            receipts.append(received_monotonic)
            if stream["align"] is not None:
                depth = frames.get_depth_frame()
                if not depth:
                    raise RuntimeError("Missing aligned depth frame: " + name)
                raw = np.asanyarray(depth.get_data()).copy()
                if raw.shape != bgr.shape[:2]:
                    raise RuntimeError("Depth is not aligned to RGB dimensions")
                scale = stream["depth_scale_m"]
                raw_path, meters_path = folder / (name + "_depth_raw.npy"), folder / (name + "_depth_m.npy")
                np.save(str(raw_path), raw, allow_pickle=False)
                np.save(str(meters_path), depth_to_meters(raw, scale), allow_pickle=False)
                depth_intrinsics = depth.profile.as_video_stream_profile().intrinsics
                metadata.update(
                    depth_raw_path=str(raw_path), depth_m_path=str(meters_path),
                    depth_scale_m=scale, depth_invalid_raw=0, depth_invalid_m="NaN",
                    depth_definition="camera optical-axis Z in metres, not radial range",
                    depth_aligned_to="color", depth_intrinsics=_intrinsics_dict(depth_intrinsics),
                    depth_device_timestamp_ms=float(depth.get_timestamp()),
                    rgb_depth_device_skew_ms=abs(float(depth.get_timestamp()) - float(color.get_timestamp())),
                    depth_frame_number=int(depth.get_frame_number()),
                )
                self._latest[name] = (raw, scale, depth_intrinsics)
            self._write_sampled_video(name, bgr, metadata, cv2)
            result["cameras"][name] = metadata
        completed = time.monotonic()
        result["captured_at"] = min(frame["host_received_at"] for frame in result["cameras"].values())
        result["timestamp_kind"] = "earliest_host_receipt_wall_clock"
        result["capture_completed_at"] = time.time()
        result["host_receipt_skew_s"] = max(receipts) - min(receipts)
        result["max_host_receipt_skew_s"] = 0.15
        result["host_receipt_skew_exceeded"] = result["host_receipt_skew_s"] > 0.15
        result["timing_note"] = "Receipt times are recorded; exposure synchronization is unverified across camera clocks."
        for metadata in result["cameras"].values():
            metadata["host_receipt_age_at_capture_complete_s"] = completed - metadata["host_received_monotonic_s"]
            metadata["max_host_receipt_age_s"] = 0.6
            metadata["host_receipt_stale"] = metadata["host_receipt_age_at_capture_complete_s"] > 0.6
        result["metadata_path"] = str(folder / "observation.json")
        with (folder / "observation.json").open("x", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        return result

    def query_pixel(self, camera_name, u, v):
        """Read a pixel from the most recent capture without acquiring a frame."""
        if camera_name not in self._latest:
            raise RuntimeError("No measured depth captured for camera: " + camera_name)
        raw, scale, intrinsics = self._latest[camera_name]
        return depth_pixel_query(
            raw, scale, (u, v), _intrinsics_dict(intrinsics),
            deprojector=lambda pixel, z: self._rs.rs2_deproject_pixel_to_point(intrinsics, pixel, z),
        )

    def start_recording(self, output_dir):
        """Enable sampled RGB videos on subsequent capture() calls; no hardware open."""
        if self._record_dir is not None:
            raise RuntimeError("Sampled recording already active")
        folder = Path(output_dir).resolve()
        folder.mkdir(parents=True, exist_ok=False)
        metadata_file = (folder / "sample_timestamps.jsonl").open("x", encoding="utf-8")
        self._record_dir = folder
        self._video_metadata = metadata_file
        self._video_frames = {}

    def _write_sampled_video(self, name, bgr, metadata, cv2):
        if self._record_dir is None:
            return
        if name not in self._writers:
            path = self._record_dir / (name + "_sampled.avi")
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"),
                                     1.0, (bgr.shape[1], bgr.shape[0]))
            if not writer.isOpened():
                writer.release()
                raise RuntimeError("Could not create sampled video: " + str(path))
            self._writers[name] = writer
            self._video_frames[name] = 0
        self._writers[name].write(bgr)
        self._video_frames[name] += 1
        self._video_metadata.write(json.dumps({
            "camera": name, "sample_index": self._video_frames[name] - 1,
            "host_received_at": metadata["host_received_at"],
            "device_timestamp_ms": metadata["device_timestamp_ms"],
            "source_rgb_path": metadata["rgb_path"],
            "video_playback_fps": 1.0,
            "recording_kind": "sampled_capture_frames_not_continuous",
        }, allow_nan=False) + "\n")
        self._video_metadata.flush()

    def stop_recording(self):
        for name, writer in self._writers.items():
            try:
                writer.release()
            except Exception as exc:
                self.close_errors.append({"camera": name, "operation": "video_release", "error": str(exc)})
        self._writers = {}
        if self._video_metadata is not None:
            try:
                self._video_metadata.close()
            except Exception as exc:
                self.close_errors.append({"operation": "video_metadata_close", "error": str(exc)})
            self._video_metadata = None
        result = {"directory": str(self._record_dir) if self._record_dir else None,
                  "frames": dict(self._video_frames),
                  "recording_kind": "sampled_capture_frames_not_continuous"}
        self._record_dir = None
        return result

    def close(self):
        self.close_errors = []
        self.stop_recording()
        for name, stream in self._streams.items():
            try:
                stream["pipeline"].stop()
            except Exception as exc:
                self.close_errors.append({"camera": name, "error": str(exc)})
        self._streams.clear()
        self._latest.clear()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
