"""Subscribe to existing ROS image topics, without launching camera drivers."""
import json
import math
import re
import threading
import time
from pathlib import Path


def configured_streams(config, rgb_only=False):
    """Return explicitly enabled subscriptions; usable without ROS installed."""
    streams = [("rgb", "rgb_topic")] if rgb_only else [("rgb", "rgb_topic"), ("info", "info_topic")]
    if not rgb_only and config.get("depth_enabled", False):
        streams.append(("depth", "aligned_depth_topic"))
    return [(kind, config[key]) for kind, key in streams if config.get(key)]


def camera_info_metadata(info, rgb_message):
    """Validate CameraInfo without ROS and keep distortion in its ROS namespace."""
    width, height = int(info.width), int(info.height)
    if width <= 0 or height <= 0 or (width, height) != (rgb_message.width, rgb_message.height):
        raise ValueError("CameraInfo dimensions do not match RGB dimensions")
    rgb_frame = str(rgb_message.header.frame_id or "").strip()
    info_frame = str(info.header.frame_id or "").strip()
    if rgb_frame and info_frame and rgb_frame != info_frame:
        raise ValueError("CameraInfo frame_id does not match RGB frame_id")
    k, distortion = [float(x) for x in info.K], [float(x) for x in info.D]
    if len(k) != 9 or not all(math.isfinite(x) for x in k + distortion):
        raise ValueError("CameraInfo K/D must contain finite values and K must have 9 entries")
    if k[0] <= 0 or k[4] <= 0:
        raise ValueError("CameraInfo requires positive calibrated focal lengths")
    if k[1] != 0 or k[3] != 0 or k[6] != 0 or k[7] != 0 or k[8] != 1:
        raise ValueError("Unsupported nonstandard camera intrinsic matrix")
    stamp = float(info.header.stamp.to_sec())
    if not math.isfinite(stamp) or stamp < 0:
        raise ValueError("Invalid CameraInfo header stamp")
    model = str(info.distortion_model or "unknown")
    return {
        "width": width, "height": height,
        "fx": k[0], "fy": k[4], "ppx": k[2], "ppy": k[5],
        "model": model, "coeffs": distortion,
        "model_namespace": "ros_camera_info_not_realsense_enum",
        "K": k, "D": distortion, "distortion_model": model,
        "source": "ros_camera_info",
        "timestamp": stamp, "timestamp_kind": "ros_header_stamp_wall_clock" if stamp else "unknown_zero_stamp",
        "frame_id": info_frame or None, "rgb_frame_id": rgb_frame or None,
        "frame_id_match": "verified" if rgb_frame and info_frame else "unknown_empty_frame_id",
        "pinhole_deprojection_available": bool(distortion) and all(x == 0 for x in distortion),
        "nonzero_distortion_deprojection": "unsupported; no ROS-to-RealSense model conversion",
    }


def snapshot_is_fresh(snapshot, camera_names, now, max_age_s, max_skew_s, require_info=True):
    """Require fresh ROS RGB stamps and actual callback receipt timestamps."""
    stamps, arrivals = [], []
    for name in camera_names:
        if (name, "rgb") not in snapshot or (require_info and (name, "info") not in snapshot):
            return False
        message, arrival = snapshot[(name, "rgb")]
        stamp = float(message.header.stamp.to_sec())
        if not all(math.isfinite(value) for value in (stamp, arrival)):
            return False
        if stamp <= 0 or not (0 <= now - stamp <= max_age_s) or not (0 <= now - arrival <= max_age_s):
            return False
        stamps.append(stamp)
        arrivals.append(arrival)
    return bool(stamps) and max(stamps) - min(stamps) <= max_skew_s and max(arrivals) - min(arrivals) <= max_skew_s


class RosCameraRig:
    def __init__(self, camera_configs, max_age_s=0.8, max_skew_s=0.15, rgb_only=False):
        self.rgb_only = bool(rgb_only)
        if self.rgb_only:
            camera_configs = {name: dict(cfg, depth_enabled=False)
                              for name, cfg in camera_configs.items()}
        if not camera_configs:
            raise ValueError("At least one camera must be configured")
        for name, cfg in camera_configs.items():
            if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
                raise ValueError("Camera names must be safe path components")
            if not cfg.get("rgb_topic") or (not self.rgb_only and not cfg.get("info_topic")):
                raise ValueError("Each ROS camera needs RGB and CameraInfo topics")
        self.config = camera_configs
        self.max_age_s = max_age_s
        self.max_skew_s = max_skew_s
        self.frames = {}
        self.subscribers = []
        self.lock = threading.Lock()

    def _start(self):
        from .robot import check_ros_master
        check_ros_master(timeout_s=2)
        import rospy
        from sensor_msgs.msg import Image
        if not self.rgb_only:
            from sensor_msgs.msg import CameraInfo
        if not rospy.core.is_initialized():
            rospy.init_node("right_pick_observer", anonymous=True, disable_signals=True)
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("Physical observations require wall-clock ROS time")
        for name, cfg in self.config.items():
            for kind, topic in configured_streams(cfg, rgb_only=self.rgb_only):
                msgtype = CameraInfo if kind == "info" else Image

                def receive(message, camera=name, item=kind):
                    received_at = time.time()
                    with self.lock:
                        self.frames[(camera, item)] = (message, received_at)
                self.subscribers.append(rospy.Subscriber(topic, msgtype, receive, queue_size=1))

    def capture(self, output_dir, timeout_s=8):
        if not self.subscribers:
            self._start()
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        from .camera import detect_red_candidates, depth_to_meters
        deadline = time.monotonic() + timeout_s
        snapshot = None
        while time.monotonic() < deadline:
            with self.lock:
                candidate = dict(self.frames)
            if snapshot_is_fresh(candidate, self.config, time.time(), self.max_age_s, self.max_skew_s,
                                 require_info=not self.rgb_only):
                snapshot = candidate
                break
            time.sleep(0.02)
        if snapshot is None:
            raise RuntimeError("Fresh RGB/CameraInfo streams unavailable or RGB timestamp skew too large")
        root = Path(output_dir).resolve() / ("frames_" + str(time.time_ns()))
        root.mkdir(parents=True)
        bridge = CvBridge()
        result = {
            "captured_at": min(snapshot[(name, "rgb")][0].header.stamp.to_sec() for name in self.config),
            "timestamp_kind": "earliest_ros_header_stamp_wall_clock",
            "cameras": {}, "calibration": None,
            "extrinsics_applied": False, "historical_extrinsics_used": False,
            "camera_geometry_changed": True, "backend": "ros1", "video_recorded": False,
        }
        for name, cfg in self.config.items():
            rgb_msg, received = snapshot[(name, "rgb")]
            if not self.rgb_only:
                info, info_received = snapshot[(name, "info")]
                intrinsics = camera_info_metadata(info, rgb_msg)
                intrinsics["host_received_at"] = info_received
            bgr = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            if bgr.shape[:2] != (rgb_msg.height, rgb_msg.width):
                raise RuntimeError("Decoded RGB dimensions differ from its ROS header")
            path = root / (name + ".png")
            if not cv2.imwrite(str(path), bgr):
                raise RuntimeError("Image write failed")
            frame = {
                "rgb_path": str(path), "serial_configured": cfg.get("serial"),
                "serial_verified_live": False, "host_received_at": received,
                "timestamp": rgb_msg.header.stamp.to_sec(),
                "timestamp_kind": "ros_header_stamp_wall_clock",
                "frame_id": rgb_msg.header.frame_id or None,
                "frame_id_status": "provided" if rgb_msg.header.frame_id else "unknown_empty_frame_id",
                "depth_enabled": bool(cfg.get("depth_enabled", False)),
            }
            if not self.rgb_only:
                frame["intrinsics"] = intrinsics
                frame["red_candidates"] = detect_red_candidates(bgr, color_order="bgr")
            depth_item = snapshot.get((name, "depth")) if frame["depth_enabled"] else None
            if depth_item:
                depth_msg, depth_received = depth_item
                depth_stamp = float(depth_msg.header.stamp.to_sec())
                now = time.time()
                depth_fresh = (math.isfinite(depth_stamp) and depth_stamp > 0
                               and 0 <= now - depth_stamp <= self.max_age_s
                               and 0 <= now - depth_received <= self.max_age_s
                               and abs(depth_stamp - frame["timestamp"]) <= self.max_skew_s)
                if depth_fresh:
                    depth_frame_id = str(depth_msg.header.frame_id or "").strip()
                    rgb_frame_id = str(rgb_msg.header.frame_id or "").strip()
                    if depth_frame_id and rgb_frame_id and depth_frame_id != rgb_frame_id:
                        raise RuntimeError("Aligned depth frame_id does not match RGB frame_id")
                    raw = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                    if raw.ndim != 2 or raw.shape != bgr.shape[:2]:
                        raise RuntimeError("Aligned depth size does not match RGB")
                    if depth_msg.encoding not in ("16UC1", "32FC1"):
                        raise RuntimeError("Unknown depth encoding/units: " + depth_msg.encoding)
                    expected_dtype = np.uint16 if depth_msg.encoding == "16UC1" else np.float32
                    if raw.dtype != expected_dtype:
                        raise RuntimeError("Depth array dtype disagrees with ROS image encoding")
                    scale = 0.001 if depth_msg.encoding == "16UC1" else 1.0
                    depth_path = root / (name + "_depth_m.npy")
                    np.save(str(depth_path), depth_to_meters(raw, scale), allow_pickle=False)
                    frame.update(
                        depth_m_path=str(depth_path), depth_unit="metre", depth_kind="optical_z",
                        depth_timestamp=depth_stamp, depth_timestamp_kind="ros_header_stamp_wall_clock",
                        depth_host_received_at=depth_received, depth_intrinsics=intrinsics,
                        depth_frame_id=depth_frame_id or None,
                        depth_frame_id_match="verified" if depth_frame_id and rgb_frame_id else "unknown_empty_frame_id",
                        depth_scale_m=scale, depth_scale_source="ROS depth image encoding convention",
                    )
                else:
                    frame["depth_unavailable_reason"] = "depth timestamp stale, future or mismatched with RGB"
            elif frame["depth_enabled"]:
                frame["depth_unavailable_reason"] = "no aligned depth message received"
            result["cameras"][name] = frame
        result["capture_completed_at"] = time.time()
        result["metadata_path"] = str(root / "observation.json")
        (root / "observation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        return result

    def close(self):
        errors = []
        for subscriber in self.subscribers:
            try:
                subscriber.unregister()
            except Exception as exc:
                errors.append(str(exc))
        self.subscribers = []
        self.frames = {}
        if errors:
            raise RuntimeError("Failed to unregister ROS camera subscriptions: " + "; ".join(errors))
