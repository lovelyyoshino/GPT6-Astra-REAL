"""Validate actual sensor times, rather than the time a JSON file was saved."""
import math
import time


def require_fresh(observation, max_age_s=0.8, max_skew_s=0.15, now=None):
    now = time.time() if now is None else now
    if not isinstance(observation, dict):
        raise ValueError("Observation must be an object")
    cameras = observation.get("cameras", {})
    if set(cameras) != {"front", "left_hand", "right_hand"}:
        raise ValueError("Three current camera observations required")
    if observation.get("host_receipt_skew_exceeded") is True:
        raise ValueError("Camera capture reported excessive skew")
    times = [observation.get("robot_state_at")]
    for name, frame in cameras.items():
        if frame.get("host_receipt_stale") is True:
            raise ValueError("Stale camera receipt: " + name)
        times.append(frame.get("host_received_at"))
        # ROS epoch timestamps are comparable; RealSense device milliseconds are not.
        if "timestamp" in frame:
            times.append(frame["timestamp"])
    for stamp in times:
        if type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise ValueError("Missing or invalid sensor timestamp")
        if not 0 <= now-stamp <= max_age_s:
            raise ValueError("Sensor image or robot state is stale or future-dated")
    if max(times)-min(times) > max_skew_s:
        raise ValueError("Camera/robot timestamps exceed permitted skew")
    return observation
