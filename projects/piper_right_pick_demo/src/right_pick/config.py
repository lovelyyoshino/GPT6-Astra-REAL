"""Explicit site configuration: no fallback from physical to offline."""
import json
from pathlib import Path


def load_config(path):
    path = Path(path).resolve()
    with path.open(encoding="utf-8") as handle:
        config = json.load(handle)
    if config.get("schema_version") != 1:
        raise ValueError("Unsupported configuration schema")
    if config.get("robot", {}).get("arm") != "right":
        raise ValueError("This demo accepts only the right arm")
    if set(config.get("cameras", {})) != {"front", "left_hand", "right_hand"}:
        raise ValueError("Explicit three-camera bindings are required")
    serials = [c.get("serial") for c in config["cameras"].values()]
    if any(not s for s in serials) or len(set(serials)) != 3:
        raise ValueError("Camera serials must be present and distinct")
    if config.get("backend") not in ("ros1", "realsense", "offline"):
        raise ValueError("Unknown backend; physical mode never falls back to offline")
    return config


def configuration_blockers(config):
    """Report unknown measurements; do not turn them into default values."""
    blockers = []
    calibration = config.get("calibration", {})
    if calibration.get("camera_geometry_changed") is not False:
        blockers.append("front_camera_pose_changed: prior extrinsic invalid")
    if calibration.get("verified") is not True or not calibration.get("version"):
        blockers.append("current_camera_to_right_base_calibration_unverified")
    if calibration.get("T_right_base_front") is None:
        blockers.append("current_T_right_base_front_missing")
    robot = config.get("robot", {})
    for key in ("namespace_verified", "tcp_verified", "gripper_verified", "workspace_verified", "stop_policy_verified"):
        if robot.get(key) is not True:
            blockers.append("robot_" + key)
    for key in ("T_j6_tcp", "workspace_min_m", "workspace_max_m", "gripper_mapping", "firmware_version"):
        if robot.get(key) is None:
            blockers.append("robot_" + key + "_missing")
    if config.get("task", {}).get("placement_region_right_base_m") is None:
        blockers.append("placement_region_not_observed_or_selected")
    if config.get("safety", {}).get("allow_motion") is not True:
        blockers.append("motion_disabled_in_config")
    # This release does not pretend unvalidated stop/TCP semantics are supported.
    blockers.append("physical_motion_adapter_not_commissioned")
    return blockers
