"""Strict, hardware-independent right-arm action protocol (m, rad, wxyz)."""
import math
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional, Tuple


class ProtocolError(ValueError):
    pass


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("%s must be a finite number" % name)
    value = float(value)
    if not math.isfinite(value):
        raise ProtocolError("%s must be finite" % name)
    return value


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ProtocolError("%s must have length %d" % (name, length))
    return tuple(_number(v, name) for v in value)


def _quaternion(value, name):
    q = _vector(value, 4, name)
    if abs(sum(v * v for v in q) - 1.0) > 1e-5:
        raise ProtocolError("%s must be a unit quaternion in w,x,y,z order" % name)
    return q


@dataclass(frozen=True)
class Pose:
    position_m: Tuple[float, float, float]
    orientation_wxyz: Tuple[float, float, float, float]


@dataclass(frozen=True)
class Action:
    type: str
    arm: str = "right"
    pose: Optional[Pose] = None
    speed_m_s: Optional[float] = None
    width_m: Optional[float] = None
    duration_s: Optional[float] = None
    issued_at: Optional[float] = None
    ttl_s: Optional[float] = None
    calibration_version: Optional[str] = None
    reason: Optional[str] = None

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ProtocolError("action must be an object")
        kind = data.get("type")
        specific = {
            "observe": set(),
            "move_tcp": {"pose", "speed_m_s", "issued_at", "ttl_s", "calibration_version"},
            "gripper": {"width_m", "issued_at", "ttl_s", "calibration_version"},
            "wait": {"duration_s"},
            "stop": set(),
        }
        if not isinstance(kind, str) or kind not in specific:
            raise ProtocolError("unknown action type")
        extra = set(data) - ({"type", "arm", "reason"} | specific[kind])
        if extra:
            raise ProtocolError("unknown fields: %s" % ", ".join(sorted(str(k) for k in extra)))
        missing = specific[kind] - set(data)
        if missing:
            raise ProtocolError("missing fields: %s" % ", ".join(sorted(missing)))
        if data.get("arm", "right") != "right":
            raise ProtocolError("only right arm is permitted")
        values = {"type": kind, "arm": "right"}
        if "reason" in data:
            if not isinstance(data["reason"], str) or len(data["reason"]) > 500:
                raise ProtocolError("reason must be a brief string, at most 500 characters")
            values["reason"] = data["reason"]
        if kind in ("move_tcp", "gripper"):
            for key in ("issued_at", "ttl_s"):
                values[key] = _number(data[key], key)
            if values["issued_at"] < 0 or values["ttl_s"] <= 0:
                raise ProtocolError("issued_at must be nonnegative; ttl_s must be positive")
            version = data["calibration_version"]
            if not isinstance(version, str) or not version.strip():
                raise ProtocolError("calibration_version is required")
            values["calibration_version"] = version
        if kind == "move_tcp":
            pose = data["pose"]
            if not isinstance(pose, dict) or set(pose) != {"position_m", "orientation_wxyz"}:
                raise ProtocolError("pose requires exactly position_m and orientation_wxyz")
            values["pose"] = Pose(_vector(pose["position_m"], 3, "position_m"),
                                  _quaternion(pose["orientation_wxyz"], "orientation_wxyz"))
            values["speed_m_s"] = _number(data["speed_m_s"], "speed_m_s")
            if values["speed_m_s"] <= 0:
                raise ProtocolError("speed_m_s must be positive")
        elif kind == "gripper":
            values["width_m"] = _number(data["width_m"], "width_m")
            if values["width_m"] < 0:
                raise ProtocolError("width_m is total opening and cannot be negative")
        elif kind == "wait":
            values["duration_s"] = _number(data["duration_s"], "duration_s")
            if values["duration_s"] < 0:
                raise ProtocolError("duration_s cannot be negative")
        return cls(**values)

    def to_dict(self):
        return {k: v for k, v in asdict(self).items() if v is not None}


@dataclass(frozen=True)
class Policy:
    # Every physical calibration/permission gate defaults closed.
    allow_motion: bool = False
    calibration_verified: bool = False
    calibration_version: Optional[str] = None
    camera_geometry_changed: bool = True
    tcp_confirmed: bool = False
    gripper_confirmed: bool = False
    workspace_confirmed: bool = False
    workspace_min_m: Optional[Tuple[float, float, float]] = None
    workspace_max_m: Optional[Tuple[float, float, float]] = None
    max_translation_step_m: Optional[float] = None
    max_orientation_step_rad: Optional[float] = None
    max_speed_m_s: Optional[float] = None
    gripper_min_width_m: Optional[float] = None
    gripper_max_width_m: Optional[float] = None
    max_observation_age_s: float = 1.0
    max_sensor_skew_s: float = 0.2
    max_action_ttl_s: float = 10.0
    max_wait_s: float = 5.0
    future_tolerance_s: float = 0.1

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ProtocolError("policy must be an object")
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ProtocolError("unknown policy fields: %s" % ", ".join(sorted(unknown)))
        return cls(**data)


@dataclass(frozen=True)
class ValidationResult:
    accepted: bool
    code: str
    reason: str

    def require(self):
        if not self.accepted:
            raise ProtocolError("%s: %s" % (self.code, self.reason))
        return self


def validate_action(action, observation, policy, now=None):
    """Check permission and measured state; acceptance is not collision planning."""
    def reject(code, reason):
        return ValidationResult(False, code, reason)

    try:
        # Also validate Action instances constructed directly, not only JSON inputs.
        action = Action.from_dict(action.to_dict() if isinstance(action, Action) else action)
        if not isinstance(policy, Policy):
            policy = Policy.from_dict(policy)
        now = _number(time.time() if now is None else now, "now")
        for field in ("allow_motion", "calibration_verified", "camera_geometry_changed",
                      "tcp_confirmed", "gripper_confirmed", "workspace_confirmed"):
            if type(getattr(policy, field)) is not bool:
                raise ProtocolError("%s must be a boolean" % field)
        for field in ("max_observation_age_s", "max_sensor_skew_s", "max_action_ttl_s",
                      "max_wait_s", "future_tolerance_s"):
            if _number(getattr(policy, field), field) < 0:
                raise ProtocolError("%s cannot be negative" % field)
        if action.type in ("observe", "stop"):
            return ValidationResult(True, "accepted", "read-only action or decision stop")
        if action.type == "wait":
            if action.duration_s > policy.max_wait_s:
                return reject("wait_limit", "wait exceeds configured duration")
            return ValidationResult(True, "accepted", "bounded wait; no actuator command")
        if not policy.allow_motion:
            return reject("motion_disabled", "allow_motion is false")
        if policy.camera_geometry_changed:
            return reject("camera_geometry_changed", "camera geometry changed; old extrinsics are invalid")
        if not policy.calibration_verified or not policy.calibration_version:
            return reject("calibration_unverified", "physical calibration is not verified")
        if action.calibration_version != policy.calibration_version:
            return reject("calibration_version_mismatch", "action uses a different calibration version")
        for key in ("tcp_confirmed", "gripper_confirmed", "workspace_confirmed"):
            if not getattr(policy, key):
                return reject("unconfirmed_configuration", "%s is false" % key)
        if action.issued_at - now > policy.future_tolerance_s:
            return reject("future_action", "issued_at is in the future")
        if action.ttl_s > policy.max_action_ttl_s or now >= action.issued_at + action.ttl_s:
            return reject("expired_action", "action expired or TTL exceeds policy")
        if not isinstance(observation, dict):
            return reject("missing_observation", "measured observation is required")
        for key in ("device_freshness_verified", "right_binding_verified", "enabled"):
            if observation.get(key) is not True:
                return reject("device_state_unverified", "%s must be explicitly true" % key)
        if type(observation.get("fault")) is not int or observation["fault"] != 0:
            return reject("device_fault_or_unknown", "fault must be a measured integer zero")
        times = [_number(observation.get(k), k) for k in ("captured_at", "robot_state_at")]
        for stamp in times:
            if now - stamp > policy.max_observation_age_s or stamp - now > policy.future_tolerance_s:
                return reject("stale_observation", "camera or robot state is stale or future-dated")
        if abs(times[0] - times[1]) > policy.max_sensor_skew_s:
            return reject("sensor_skew", "camera and robot timestamps are not synchronized")
        if observation.get("calibration_version") != policy.calibration_version:
            return reject("observation_calibration_mismatch", "observation lacks matching calibration version")
        if observation.get("camera_geometry_changed", False) is not False:
            return reject("camera_geometry_changed", "observation reports changed camera geometry")
        if action.type == "move_tcp":
            lo = _vector(policy.workspace_min_m, 3, "workspace_min_m")
            hi = _vector(policy.workspace_max_m, 3, "workspace_max_m")
            if any(a >= b for a, b in zip(lo, hi)):
                raise ProtocolError("workspace bounds are invalid")
            target = action.pose.position_m
            if any(v < a or v > b for v, a, b in zip(target, lo, hi)):
                return reject("workspace_limit", "TCP target is outside confirmed workspace")
            measured = _vector(observation.get("right_tcp_position_m"), 3, "right_tcp_position_m")
            if any(v < a or v > b for v, a, b in zip(measured, lo, hi)):
                return reject("current_pose_outside_workspace", "measured TCP is outside confirmed workspace")
            step = _number(policy.max_translation_step_m, "max_translation_step_m")
            speed = _number(policy.max_speed_m_s, "max_speed_m_s")
            angle = _number(policy.max_orientation_step_rad, "max_orientation_step_rad")
            if min(step, speed, angle) <= 0:
                raise ProtocolError("positive translation, orientation, and speed limits are required")
            if action.speed_m_s > speed:
                return reject("speed_limit", "TCP speed exceeds confirmed limit")
            if math.sqrt(sum((a - b) ** 2 for a, b in zip(target, measured))) > step:
                return reject("step_limit", "TCP step exceeds confirmed limit")
            current_q = _quaternion(observation.get("right_tcp_orientation_wxyz"), "right_tcp_orientation_wxyz")
            dot = min(1.0, abs(sum(a * b for a, b in zip(current_q, action.pose.orientation_wxyz))))
            if 2 * math.acos(dot) > angle:
                return reject("orientation_limit", "TCP orientation step exceeds confirmed limit")
        elif action.type == "gripper":
            lo = _number(policy.gripper_min_width_m, "gripper_min_width_m")
            hi = _number(policy.gripper_max_width_m, "gripper_max_width_m")
            if lo < 0 or lo >= hi:
                raise ProtocolError("confirmed total gripper opening limits are invalid")
            if not lo <= action.width_m <= hi:
                return reject("gripper_limit", "total gripper opening is outside confirmed limits")
    except (ProtocolError, TypeError, ValueError, AttributeError) as exc:
        return reject("invalid_input", str(exc))
    return ValidationResult(True, "accepted", "protocol, calibration, freshness, and local bounds passed")
