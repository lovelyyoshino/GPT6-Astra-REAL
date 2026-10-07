"""Validate model-authored data and render intent. Never solve IK or send motion."""
from __future__ import annotations

import html
import math
import re
import time


TARGET_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["arm", "frame", "reference", "pose_m_rad", "motion", "speed_percent", "uncertainty"],
    "properties": {
        "arm": {"type": "string", "enum": ["left", "right"]},
        "action": {"type": "string", "enum": ["move", "gripper"]},
        "frame": {"type": "string", "enum": ["left_base", "right_base"]},
        "reference": {"type": "string", "enum": ["sdk_flange"]},
        "pose_m_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}},
        "motion": {"type": "string", "enum": ["move_p", "move_l"]},
        "speed_percent": {"type": "integer", "minimum": 1, "maximum": 5},
        "uncertainty": {"type": "string", "minLength": 1, "maxLength": 2000},
        "gripper_width_m": {"type": "number", "minimum": 0, "maximum": 0.1},
        "gripper_force_N": {"type": "number", "minimum": 0.001, "maximum": 5},
    },
}
PLAN_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["schema_version", "task", "observation_id", "stages"],
    "properties": {
        "schema_version": {"type": "integer", "const": 1},
        "task": {"type": "string", "minLength": 1, "maxLength": 2000},
        "observation_id": {"type": "string", "pattern": "^obs_[0-9a-f]{32}$"},
        "stages": {"type": "array", "minItems": 1, "maxItems": 24, "items": {
            "type": "object", "additionalProperties": False,
            "required": ["id", "intent", "expected_feedback", "coordination", "targets"],
            "properties": {
                "id": {"type": "string", "pattern": "^[a-zA-Z0-9_-]{1,48}$"},
                "intent": {"type": "string", "minLength": 1, "maxLength": 2000},
                "expected_feedback": {"type": "string", "minLength": 1, "maxLength": 2000},
                "coordination": {"type": "string", "enum": ["sequential", "paired"]},
                "targets": {"type": "array", "minItems": 1, "maxItems": 2, "items": TARGET_SCHEMA},
            },
        }},
    },
}


def validate(value, schema: dict, path: str = "$", depth: int = 0) -> None:
    """Small validator for exactly the JSON Schema subset our tools declare.

    It is not advertised as a general JSON Schema implementation. No coercion,
    clipping, eval, expressions, or unknown property acceptance.
    """
    if depth > 16:
        raise ValueError(f"{path}: nesting too deep")
    kind = schema.get("type")
    checks = {"object": lambda: isinstance(value, dict),
              "array": lambda: isinstance(value, list),
              "string": lambda: isinstance(value, str),
              "boolean": lambda: type(value) is bool,
              "integer": lambda: type(value) is int,
              "number": lambda: type(value) in (int, float) and math.isfinite(value)}
    if kind not in checks or not checks[kind]():
        raise ValueError(f"{path}: expected {kind} (finite, correctly typed)")
    if "const" in schema and value != schema["const"]:
        raise ValueError(f"{path}: must equal {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError(f"{path}: must be one of {schema['enum']}")
    if kind == "object":
        properties = schema.get("properties", {})
        missing = set(schema.get("required", [])) - set(value)
        extra = set(value) - set(properties)
        if missing or (extra and schema.get("additionalProperties") is False):
            raise ValueError(f"{path}: missing={sorted(missing)}, unexpected={sorted(extra)}")
        for key, child in value.items():
            if key in properties:
                validate(child, properties[key], f"{path}.{key}", depth + 1)
    if kind == "array":
        if not schema.get("minItems", 0) <= len(value) <= schema.get("maxItems", 1000):
            raise ValueError(f"{path}: invalid array length")
        for i, child in enumerate(value):
            validate(child, schema["items"], f"{path}[{i}]", depth + 1)
    if kind == "string":
        if not schema.get("minLength", 0) <= len(value) <= schema.get("maxLength", 10000):
            raise ValueError(f"{path}: invalid string length")
        if "pattern" in schema and not re.fullmatch(schema["pattern"], value):
            raise ValueError(f"{path}: invalid format")
    if kind in ("number", "integer"):
        if value < schema.get("minimum", -math.inf) or value > schema.get("maximum", math.inf):
            raise ValueError(f"{path}: outside declared range; value was NOT clipped")


def preview_plan(plan: dict, profile: dict, observation: dict | None,
                 now: float | None = None) -> dict:
    validate(plan, PLAN_SCHEMA)
    ids = set()
    for stage in plan["stages"]:
        if stage["id"] in ids:
            raise ValueError("Duplicate stage id")
        ids.add(stage["id"])
        arms = [target["arm"] for target in stage["targets"]]
        if len(set(arms)) != len(arms):
            raise ValueError(f"{stage['id']}: repeated arm")
        if stage["coordination"] == "paired" and len(arms) != 2:
            raise ValueError(f"{stage['id']}: paired requires both arms")
        actions = {t.get("action", "move") for t in stage["targets"]}
        if len(actions) != 1:
            raise ValueError(f"{stage['id']}: use separate stages for arm movement and gripper actions")
        for target in stage["targets"]:
            grip = target.get("action", "move") == "gripper"
            grip_fields = {"gripper_width_m", "gripper_force_N"}.intersection(target)
            if grip and len(grip_fields) != 2:
                raise ValueError(f"{stage['id']}: gripper action requires width and explicit force_N")
            if not grip and grip_fields:
                raise ValueError(f"{stage['id']}: gripper parameters require a separate action=gripper stage")
            if target["frame"] != target["arm"] + "_base":
                raise ValueError(f"{stage['id']}: arm/frame mismatch")
            if any(abs(a) > math.pi for a in target["pose_m_rad"][3:]):
                raise ValueError(f"{stage['id']}: RPY must be radians within [-pi, pi]; no clipping")
            if abs(target["pose_m_rad"][4]) > math.pi / 2:
                raise ValueError(f"{stage['id']}: pitch outside [-pi/2, pi/2]; vendor would clip it")
            if target["speed_percent"] > profile["preview_max_speed_percent"]:
                raise ValueError(f"{stage['id']}: exceeds local preview speed ceiling")

    checks = [{"name": "structure_and_units", "status": "passed",
               "detail": "Syntax and declared units only; no reachability claim"}]
    if observation is None:
        checks.append({"name": "observation", "status": "blocked", "detail": "Unknown observation id; acquire current views and states"})
    else:
        age = (time.time() if now is None else now) - observation["capture_started_unix_s"]
        current = (observation.get("id") == plan["observation_id"]
                   and observation.get("complete") is True
                   and 0 <= age <= profile["preview_max_observation_age_s"])
        checks.append({"name": "observation", "status": "passed" if current else "blocked",
                       "age_of_capture_start_s": age,
                       "detail": "Complete recent capture required; camera exposure and arm samples are not synchronized"})
    for key, verified in profile["verification"].items():
        checks.append({"name": key, "status": "declared_verified" if verified is True else "unknown",
                       "detail": "Profile declaration only; does not add solver or execution capability"})
    checks.extend([
        {"name": "controller_health_and_grasp_state", "status": "not_assessed",
         "detail": "Fresh complete arm+gripper health and start drift are checked again before dispatch; grasp is never inferred from width alone"},
        {"name": "candidate_inverse_kinematics", "status": "unavailable",
         "detail": "Local vendor API offers FK and post-command IK feedback, not pure pose-to-joint preview"},
        {"name": "controller_interpolation_and_collision", "status": "unavailable",
         "detail": "No simulated joint path, self/table/inter-arm/tool collision check, or dynamic cloth model"},
        {"name": "coordinated_execution_and_abort", "status": "unavailable",
         "detail": "Executor implements near-time send plus barriers, not synchronized paths; physical holding stop remains unvalidated"},
        {"name": "physical_dispatch", "status": "blocked", "detail": "SDK dispatcher implemented; backend commissioning and legacy incident hold still block this site"},
    ])
    return {"status": "preview_only", "structure_valid": True, "executable": False,
            "hardware_commands_sent": 0, "observation_id": plan["observation_id"],
            "checks": checks, "stage_count": len(plan["stages"]),
            "next_action": "Resolve unknown checks; do not change coordinates blindly after rejection",
            "visualization_kind": "stage intent chart, NOT kinematics or collision simulation"}


def render_intent_svg(plan: dict) -> str:
    """Separate arm columns, not a fictitious common Cartesian workspace."""
    height = 140 + len(plan["stages"]) * 102
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="{height}" viewBox="0 0 1080 {height}">',
           '<rect width="100%" height="100%" fill="#f6f8fa"/>',
           '<g font-family="sans-serif" fill="#152238">',
           '<text x="24" y="28" font-size="18">Intent preview only — no IK, interpolation, or collision validation</text>',
           '<text x="24" y="53" font-size="13">Each column uses its own arm base. Rows indicate stage order, not a geometric path.</text>',
           '<text x="24" y="82">LEFT / left_base</text><text x="550" y="82">RIGHT / right_base</text>']
    for index, stage in enumerate(plan["stages"]):
        y = 108 + index * 102
        for side, x in (("left", 24), ("right", 550)):
            target = next((t for t in stage["targets"] if t["arm"] == side), None)
            out.append(f'<rect x="{x}" y="{y}" width="505" height="88" rx="8" fill="white" stroke="#b9c5d0"/>')
            label = html.escape(f"{index+1}. {stage['id']} / {stage['coordination']}")
            out.append(f'<text x="{x+10}" y="{y+22}" font-size="14">{label}</text>')
            if target:
                pose = target["pose_m_rad"]
                detail = f"flange xyz (m): {pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}"
                detail2 = f"RPY (rad): {pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}; {target['motion']}"
            else:
                detail, detail2 = "No new target in this stage", "This does not command or prove holding position"
            out.append(f'<text x="{x+10}" y="{y+46}" font-size="13">{html.escape(detail)}</text>')
            out.append(f'<text x="{x+10}" y="{y+67}" font-size="13">{html.escape(detail2)}</text>')
    return "\n".join(out + ["</g></svg>"])
