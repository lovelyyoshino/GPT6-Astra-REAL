"""Pure contracts for model-owned, RGB-only pen manipulation decisions.

No camera, detector, calibration, kinematics, robot, or network module is imported.
Pose numbers refer to the driver's end reference in right_base (metres/radians),
not an inferred tool tip. This module never determines visual task success.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
import math


class FastPolicyError(ValueError):
    pass


PHASES = ("INIT", "APPROACH_PEN", "ALIGN_PEN", "PREGRASP", "GRASP",
          "VERIFY_GRASP", "LIFT", "APPROACH_HOLDER", "ALIGN_HOLDER", "INSERT",
          "RELEASE", "VERIFY_SUCCESS", "DONE", "RECOVERY")
EVIDENCE = ("unknown", "phase_complete", "no_progress", "target_lost",
            "grasp_failed", "success")
ACTIONS = ("observe", "move_eef", "move_eef_chunk", "gripper", "advance", "pause")
MAX_CHUNK_WAYPOINTS = 3  # Schema capacity only; does not authorize physical movement.
ATOMIC_DECISION_RULE = (
    "Atomic loop: observe once, decide one current-phase action, let the host admit it, "
    "dispatch at most once, read its receipt, then use fresh RGB. Observation is not progress: "
    "after one observe(unknown), advance only with phase_complete evidence; otherwise propose "
    "one bounded, phase-allowed visibility-improving action only if current RGB and robot "
    "state support assessing its risk; otherwise pause "
    "with the specific missing fact. A fixed-camera observation cannot resolve unchanged "
    "occlusion; do not repeat it without a reason to expect new evidence."
)
_FAILURES = frozenset(("no_progress", "target_lost", "grasp_failed"))
_NEXT = dict(zip(PHASES[:12], PHASES[1:13]))
_NEXT["RECOVERY"] = "INIT"
_MOVE_TRANSITIONS = frozenset(("APPROACH_PEN", "ALIGN_PEN", "PREGRASP", "LIFT",
                               "APPROACH_HOLDER", "ALIGN_HOLDER"))
_CHUNK_PHASES = frozenset(("APPROACH_PEN", "APPROACH_HOLDER"))
_MOTION_FRACTIONS = {
    "INIT": 0.0, "APPROACH_PEN": 1.0, "ALIGN_PEN": 0.5,
    "PREGRASP": 0.15, "GRASP": 0.0, "VERIFY_GRASP": 0.15,
    "LIFT": 0.5, "APPROACH_HOLDER": 1.0, "ALIGN_HOLDER": 0.5,
    "INSERT": 0.1, "RELEASE": 0.0, "VERIFY_SUCCESS": 0.5,
    "DONE": 0.0, "RECOVERY": 0.15,
}
_BOTH = ("front", "right_hand")
_ALL = ("front", "left_hand", "right_hand")
_SPEC = {
    "INIT": ("Inspect the current scene and measured robot state; identify the pen and holder.", _BOTH, "medium", ("gripper",)),
    "APPROACH_PEN": ("Choose one bounded approach or visibility-improving action toward the pen. Assess this motion using current RGB and robot state, including collision risks for the gripper and attachments; not every surface must be visible. Pause if unresolved occlusion prevents assessing this specific motion.", _BOTH, "low", ("move_eef", "move_eef_chunk")),
    "ALIGN_PEN": ("Use the current wrist RGB image to align the open fingers with the pen.", _BOTH, "medium", ("move_eef",)),
    "PREGRASP": ("Check finger, pen, and table separation before a small final approach.", _BOTH, "medium", ("move_eef", "gripper")),
    "GRASP": ("Choose one closing command; closure feedback alone does not prove a grasp.", _BOTH, "high", ("gripper",)),
    "VERIFY_GRASP": ("Inspect new post-close RGB evidence; a bounded test lift may be requested before judging retention.", _BOTH, "high", ("move_eef",)),
    "LIFT": ("Lift the retained pen with clearance and inspect it again before transport.", _BOTH, "low", ("move_eef",)),
    "APPROACH_HOLDER": ("Transport in visible free space toward a high holder observation position.", ("front",), "low", ("move_eef", "move_eef_chunk")),
    "ALIGN_HOLDER": ("Judge the pen tail against the actual opening; report unresolved fore-aft ambiguity.", _BOTH, "medium", ("move_eef",)),
    "INSERT": ("Use one small insertion step at a time and inspect new RGB before deciding to release.", _BOTH, "high", ("move_eef",)),
    "RELEASE": ("Release only after the preceding fresh visual decision supports placement.", ("front",), "high", ("gripper",)),
    "VERIFY_SUCCESS": ("Inspect retention after release; retreat if needed, then inspect new RGB before declaring success.", _BOTH, "high", ("move_eef",)),
    "DONE": ("The model has reported visual completion; retain evidence for independent review.", ("front",), "low", ()),
    "RECOVERY": ("Reobserve the whole scene and state; make a bounded recovery proposal or pause.", _ALL, "high", ("move_eef", "gripper")),
}


def phase_spec(phase):
    if not isinstance(phase, str) or phase not in _SPEC:
        raise FastPolicyError("unknown phase")
    goal, cameras, effort, extra = _SPEC[phase]
    common = ("observe", "pause") if phase == "DONE" else ("observe", "advance", "pause")
    return {"phase": phase, "goal": goal, "cameras": list(cameras),
            "reasoning_effort": effort, "allowed_actions": list(common + extra),
            "next_phase": _NEXT.get(phase), "pose_reference": "right_base_driver_end_reference",
            "pose_units": "metres_radians", "visual_input": "rgb_only_uncalibrated",
            "translation_fraction": phase_translation_fraction(phase),
            "rotation_fraction": phase_rotation_fraction(phase),
            "chunk_allowed": phase in _CHUNK_PHASES,
            "chunk_max_waypoints": MAX_CHUNK_WAYPOINTS if phase in _CHUNK_PHASES else 0,
            "chunk_policy": "Same-phase free-space approach only, without contact; cumulative translation and rotation cannot exceed the scaled single-action caps. Numerical acceptance does not prove tool, attachment, or whole-path clearance."}


def phase_translation_fraction(phase):
    """Fraction of explicitly supplied limits, never an absolute motion default."""
    if not isinstance(phase, str) or phase not in _MOTION_FRACTIONS:
        raise FastPolicyError("unknown phase")
    return _MOTION_FRACTIONS[phase]


def phase_rotation_fraction(phase):
    return phase_translation_fraction(phase)


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise FastPolicyError(name + " must be a finite number")
    return value


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise FastPolicyError(name + " must contain exactly " + str(size) + " numbers")
    return [_number(v, name) for v in value]


def _object(value, fields, name):
    if not isinstance(value, dict) or set(value) != set(fields):
        raise FastPolicyError(name + " has missing or unknown fields")


def _json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise FastPolicyError("duplicate JSON key: " + key)
        result[key] = value
    return result


@dataclass(frozen=True)
class Decision:
    phase: str
    action: str
    arguments: dict
    confidence: float
    explanation: str = None

    def to_dict(self):
        result = {"phase": self.phase, "action": self.action,
                  "arguments": deepcopy(self.arguments), "confidence": self.confidence}
        if self.explanation is not None:
            result["explanation"] = self.explanation
        return result


def _schema_object(properties):
    return {"type": "object", "properties": properties,
            "required": list(properties), "additionalProperties": False}


def response_schema(require_explanation=False, *, controller_state=None):
    if type(require_explanation) is not bool:
        raise FastPolicyError("require_explanation must be boolean")
    pose = {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6}
    next_phase = {"anyOf": [{"type": "string", "enum": list(PHASES)}, {"type": "null"}]}
    evidence = {"type": "string", "enum": list(EVIDENCE)}
    argument_variants = [
        _schema_object({"evidence": evidence}),
        _schema_object({"pose_m_rad": pose, "speed_percent": {"type": "integer", "minimum": 1, "maximum": 100}, "next_phase": next_phase}),
        _schema_object({"waypoints": {"type": "array", "items": pose, "minItems": 1, "maxItems": MAX_CHUNK_WAYPOINTS}, "speed_percent": {"type": "integer", "minimum": 1, "maximum": 100}, "next_phase": next_phase}),
        _schema_object({"opening_m": {"type": "number", "minimum": 0}, "effort_parameter_nm": {"type": "number", "exclusiveMinimum": 0}}),
        _schema_object({"next_phase": {"type": "string", "enum": list(PHASES)}, "evidence": evidence}),
    ]
    fields = {"phase": {"type": "string", "enum": list(PHASES)},
              "action": {"type": "string", "enum": list(ACTIONS)},
              "arguments": {"anyOf": argument_variants},
              "confidence": {"type": "number", "minimum": 0, "maximum": 1}}
    if controller_state is not None:
        state = compact_controller_state(controller_state)
        budget = state.get("action_budget", {})
        # Optional live transport capabilities narrow the contract. Replay
        # retains its original schema when these fields are absent.
        if _has_transport_capabilities(budget):
            contract = controller_phase_spec(state)
            allowed = contract["allowed_actions"]
            fields["phase"]["enum"] = [state["phase"]]
            fields["action"]["enum"] = list(allowed)
            variant_actions = (("observe", "pause"), ("move_eef",), ("move_eef_chunk",),
                               ("gripper",), ("advance",))
            variants = [variant for variant, actions in zip(argument_variants, variant_actions)
                        if any(action in allowed for action in actions)]
            for variant in variants:
                props = variant["properties"]
                if "effort_parameter_nm" in props and "required_effort_parameter_nm" in budget:
                    props["effort_parameter_nm"] = {"type": "number", "enum": [budget["required_effort_parameter_nm"]]}
                if "opening_m" in props:
                    props["opening_m"].update(minimum=budget["gripper_min_m"], maximum=budget["gripper_max_m"])
                if "speed_percent" in props:
                    props["speed_percent"]["maximum"] = budget["max_speed_percent"]
            fields["arguments"]["anyOf"] = variants
    if require_explanation:
        fields["explanation"] = {"type": "string", "minLength": 1, "maxLength": 240}
    return _schema_object(fields)


def _has_transport_capabilities(budget):
    return "allow_waypoint_chunks" in budget or "required_effort_parameter_nm" in budget


def controller_phase_spec(controller_state):
    """Current phase contract narrowed by explicit transport capabilities."""
    state = compact_controller_state(controller_state)
    contract = phase_spec(state["phase"])
    budget = state.get("action_budget", {})
    if _has_transport_capabilities(budget) and state["phase"] in ("GRASP", "RELEASE"):
        contract["allowed_actions"] = [action for action in contract["allowed_actions"] if action != "advance"]
    if budget.get("allow_waypoint_chunks") is False:
        contract["allowed_actions"] = [action for action in contract["allowed_actions"] if action != "move_eef_chunk"]
        contract.update(chunk_allowed=False, chunk_max_waypoints=0,
                        chunk_policy="This transport accepts only one move_eef endpoint per action; waypoint chunks are unavailable.")
    if "required_effort_parameter_nm" in budget:
        contract["gripper_effort_parameter_nm"] = budget["required_effort_parameter_nm"]
        contract["gripper_effort_rule"] = "The transport requires this exact effort parameter; it is not a measured contact force."
    return contract


def parse_response(raw, require_explanation=False):
    if type(require_explanation) is not bool:
        raise FastPolicyError("require_explanation must be boolean")
    if isinstance(raw, Decision):
        raw = raw.to_dict()
    if isinstance(raw, str):
        try:
            raw = json.loads(raw, object_pairs_hook=_json_object)
        except (ValueError, TypeError) as exc:
            raise FastPolicyError("invalid decision JSON: " + str(exc)) from exc
    fields = ("phase", "action", "arguments", "confidence") + (("explanation",) if require_explanation else ())
    _object(raw, fields, "decision")
    if raw["phase"] not in PHASES or raw["action"] not in ACTIONS:
        raise FastPolicyError("unknown phase or action")
    confidence = _number(raw["confidence"], "confidence")
    if not 0 <= confidence <= 1:
        raise FastPolicyError("confidence outside [0, 1]")
    explanation = raw.get("explanation")
    if require_explanation and (not isinstance(explanation, str) or not 1 <= len(explanation) <= 240):
        raise FastPolicyError("explanation must contain 1..240 characters")
    action, args = raw["action"], raw["arguments"]
    if action in ("observe", "pause", "advance"):
        _object(args, ("next_phase", "evidence") if action == "advance" else ("evidence",), "arguments")
        if args["evidence"] not in EVIDENCE:
            raise FastPolicyError("unknown evidence")
        if action == "advance" and args["next_phase"] not in PHASES:
            raise FastPolicyError("unknown next_phase")
    elif action in ("move_eef", "move_eef_chunk"):
        key = "pose_m_rad" if action == "move_eef" else "waypoints"
        _object(args, (key, "speed_percent", "next_phase"), "arguments")
        points = [args[key]] if action == "move_eef" else args[key]
        if not isinstance(points, list) or not 1 <= len(points) <= MAX_CHUNK_WAYPOINTS:
            raise FastPolicyError("waypoints must contain 1..3 poses")
        for pose in points:
            _vector(pose, 6, "pose_m_rad")
        if type(args["speed_percent"]) is not int or not 1 <= args["speed_percent"] <= 100:
            raise FastPolicyError("speed_percent must be an integer in 1..100")
        if args["next_phase"] is not None and args["next_phase"] not in PHASES:
            raise FastPolicyError("unknown next_phase")
    else:
        _object(args, ("opening_m", "effort_parameter_nm"), "arguments")
        if _number(args["opening_m"], "opening_m") < 0 or _number(args["effort_parameter_nm"], "effort_parameter_nm") <= 0:
            raise FastPolicyError("invalid gripper opening or effort parameter")
    return Decision(raw["phase"], action, deepcopy(args), confidence, explanation)


def _exceptional(state):
    result = state.get("previous_result") or {}
    return (state.get("phase") == "RECOVERY" or state.get("retry_count", 0) > 0
            or result.get("visual_progress") in (False, "no_progress")
            or result.get("target_visible") is False or result.get("grasp_verified") is False
            or result.get("status") in ("failed", "rejected", "timeout")
            or result.get("error_code") not in (None, "", "none", "ok", 0))


def select_camera_views(state):
    state = compact_controller_state(state)
    return _ALL if _exceptional(state) else tuple(phase_spec(state["phase"])["cameras"])


def reasoning_effort(state):
    state = compact_controller_state(state)
    if state["retry_count"] >= 2:
        return "xhigh"
    return "high" if _exceptional(state) else phase_spec(state["phase"])["reasoning_effort"]


def requires_explanation(state):
    return _exceptional(compact_controller_state(state))


def compact_controller_state(raw):
    """Project a runner-owned state; never copy arbitrary metadata or history.

    The runner must populate visual_progress/target_visible/grasp_verified only
    from explicit model reports. They are not detector output or scoring truth.
    """
    if not isinstance(raw, dict) or raw.get("phase") not in PHASES:
        raise FastPolicyError("state needs a known phase")
    compact = {"phase": raw["phase"], "robot_state": {}, "gripper_state": {}}
    robot = raw.get("robot_state", {})
    grip = raw.get("gripper_state", {})
    if not isinstance(robot, dict) or not isinstance(grip, dict):
        raise FastPolicyError("robot_state and gripper_state must be objects")
    for key in ("pose_m_rad", "joints_rad"):
        if key in robot:
            compact["robot_state"][key] = _vector(robot[key], 6, key)
    for key in ("arm_status", "err_code"):
        if key in robot:
            if type(robot[key]) is not int:
                raise FastPolicyError(key + " must be an integer")
            compact["robot_state"][key] = robot[key]
    for key in ("enabled", "moving", "binding_verified"):
        if key in robot:
            if type(robot[key]) is not bool:
                raise FastPolicyError(key + " must be boolean")
            compact["robot_state"][key] = robot[key]
    if "sampled_at" in robot:
        compact["robot_state"]["sampled_at"] = _number(robot["sampled_at"], "sampled_at")
    for key in ("opening_m", "effort"):
        if key in grip:
            compact["gripper_state"][key] = _number(grip[key], key)
    if "action_budget" in raw:
        budget = raw["action_budget"]
        keys = ("max_translation_m", "max_rotation_rad", "max_speed_percent", "max_waypoints",
                "gripper_min_m", "gripper_max_m", "max_effort_parameter_nm")
        optional_keys = {"allow_waypoint_chunks", "required_effort_parameter_nm"}
        if (not isinstance(budget, dict) or not set(keys) <= set(budget)
                or set(budget) - set(keys) - optional_keys):
            raise FastPolicyError("action_budget has missing or unknown fields")
        clean_budget = {}
        for key in ("max_translation_m", "max_rotation_rad", "gripper_max_m", "max_effort_parameter_nm"):
            value = _number(budget[key], key)
            may_be_zero = key in ("max_translation_m", "max_rotation_rad")
            if value < 0 or not may_be_zero and value == 0:
                raise FastPolicyError(key + " must be " + ("nonnegative" if may_be_zero else "positive"))
            clean_budget[key] = value
        for key, maximum in (("max_speed_percent", 100), ("max_waypoints", MAX_CHUNK_WAYPOINTS)):
            value = budget[key]
            if type(value) is not int or not 1 <= value <= maximum:
                raise FastPolicyError(key + " must be an integer within its bound")
            clean_budget[key] = value
        minimum = _number(budget["gripper_min_m"], "gripper_min_m")
        if not 0 <= minimum < clean_budget["gripper_max_m"]:
            raise FastPolicyError("action_budget gripper range is invalid")
        clean_budget["gripper_min_m"] = minimum
        if "allow_waypoint_chunks" in budget:
            if type(budget["allow_waypoint_chunks"]) is not bool:
                raise FastPolicyError("allow_waypoint_chunks must be boolean")
            clean_budget["allow_waypoint_chunks"] = budget["allow_waypoint_chunks"]
            if not budget["allow_waypoint_chunks"] and clean_budget["max_waypoints"] != 1:
                raise FastPolicyError("Single-endpoint transport requires max_waypoints=1")
        if "required_effort_parameter_nm" in budget:
            fixed = _number(budget["required_effort_parameter_nm"], "required_effort_parameter_nm")
            if not 0 < fixed <= clean_budget["max_effort_parameter_nm"]:
                raise FastPolicyError("Required effort must be positive and within the effort limit")
            clean_budget["required_effort_parameter_nm"] = fixed
        compact["action_budget"] = clean_budget
    previous = raw.get("previous_action")
    if previous is not None:
        if not isinstance(previous, dict):
            raise FastPolicyError("previous_action must be an object")
        minimal = {key: previous[key] for key in ("phase", "action", "arguments", "confidence") if key in previous}
        compact["previous_action"] = parse_response(minimal).to_dict()
    result = raw.get("previous_result")
    if result is not None:
        if not isinstance(result, dict):
            raise FastPolicyError("previous_result must be an object")
        clean = {}
        if "status" in result:
            allowed = ("unknown", "idle", "arrived", "completed", "gripper_settled", "command_observed_stable", "nonphysical_applied", "rejected", "failed", "timeout", "paused", "observed")
            if result["status"] not in allowed:
                raise FastPolicyError("unknown previous result status")
            clean["status"] = result["status"]
        if "error_code" in result:
            code = result["error_code"]
            if code is not None and not (type(code) is int or isinstance(code, str) and len(code) <= 64 and all(c.isalnum() or c in "_.-" for c in code)):
                raise FastPolicyError("error_code must be a short code")
            clean["error_code"] = code
        if "visual_progress" in result:
            value = result["visual_progress"]
            if value is not None and type(value) is not bool and value not in ("unknown", "progress", "no_progress"):
                raise FastPolicyError("visual_progress must be a model report enum or boolean")
            clean["visual_progress"] = value
        for key in ("target_visible", "grasp_verified"):
            if key in result:
                if result[key] is not None and type(result[key]) is not bool:
                    raise FastPolicyError(key + " must be boolean or null")
                clean[key] = result[key]
        compact["previous_result"] = clean
    retry = raw.get("retry_count", 0)
    if type(retry) is not int or retry < 0:
        raise FastPolicyError("retry_count must be a nonnegative integer")
    compact["retry_count"] = retry
    memory = raw.get("memory", "")
    if not isinstance(memory, str) or len(memory) > 240:
        raise FastPolicyError("memory must be a runner-owned string of at most 240 characters")
    compact["memory"] = memory
    return compact


def validate_phase_decision(decision, current_phase, *, controller_state=None):
    """Pure schema/phase semantics for either a proposal or execution pipeline.

    This does not check numerical motion limits, freshness, or permission to send.
    """
    if controller_state is not None and not isinstance(controller_state, dict):
        raise FastPolicyError("controller_state must be an object")
    if controller_state is not None and controller_state.get("phase", current_phase) != current_phase:
        raise FastPolicyError("controller state phase does not match current phase")
    has_explanation = isinstance(decision, Decision) and decision.explanation is not None or isinstance(decision, dict) and "explanation" in decision
    decision = parse_response(decision, require_explanation=has_explanation)
    spec = controller_phase_spec(dict(controller_state, phase=current_phase)) if controller_state is not None else phase_spec(current_phase)
    if decision.phase != current_phase:
        raise FastPolicyError("decision phase does not match current phase")
    if decision.action not in spec["allowed_actions"]:
        raise FastPolicyError("action is not allowed in this phase")
    args, action = decision.arguments, decision.action
    required_effort = (controller_state or {}).get("action_budget", {}).get("required_effort_parameter_nm")
    if action == "gripper" and required_effort is not None and args["effort_parameter_nm"] != required_effort:
        raise FastPolicyError("Gripper effort must equal the transport's required effort parameter")
    if action == "advance":
        target, evidence = args["next_phase"], args["evidence"]
        if target == "RECOVERY" and current_phase not in ("RECOVERY", "DONE"):
            if evidence not in _FAILURES:
                raise FastPolicyError("recovery requires an explicit model failure report")
        else:
            if target != spec["next_phase"] or current_phase in ("GRASP", "RELEASE"):
                raise FastPolicyError("advance must follow an allowed adjacent visual transition")
            if evidence != ("success" if target == "DONE" else "phase_complete"):
                raise FastPolicyError("advance requires the appropriate model evidence report")
            gates = {
                "VERIFY_GRASP": ("grasp_close_completed",),
                "LIFT": ("lift_completed",),
                "INSERT": ("insertion_move_completed",),
                "VERIFY_SUCCESS": ("release_open_completed", "retreat_after_release_completed"),
            }
            mechanical = (controller_state or {}).get("execution_evidence", {})
            if not isinstance(mechanical, dict):
                raise FastPolicyError("execution_evidence must be runner-owned flags")
            for flag in gates.get(current_phase, ()):
                if mechanical.get(flag) is not True:
                    raise FastPolicyError("missing runner-owned mechanical evidence: " + flag)
    elif action in ("move_eef", "move_eef_chunk"):
        target = args["next_phase"]
        if target is not None and (current_phase not in _MOVE_TRANSITIONS or target != spec["next_phase"]):
            raise FastPolicyError("motion cannot skip a required fresh visual decision")
        if action == "move_eef_chunk" and current_phase not in _CHUNK_PHASES:
            raise FastPolicyError("waypoint chunks are restricted to approach phases")
    if action == "gripper" and current_phase in ("INIT", "PREGRASP", "GRASP", "RELEASE"):
        if controller_state is None:
            raise FastPolicyError("measured controller state is required")
        grip = controller_state.get("gripper_state", {})
        if not isinstance(grip, dict):
            raise FastPolicyError("measured gripper_state must be an object")
        opening = _number(grip.get("opening_m"), "measured opening_m")
        if current_phase in ("INIT", "PREGRASP") and args["opening_m"] < opening:
            raise FastPolicyError("closing is reserved for GRASP or explicit RECOVERY")
        if current_phase == "GRASP" and args["opening_m"] >= opening:
            raise FastPolicyError("GRASP requires a closing target")
        if current_phase == "RELEASE" and args["opening_m"] <= opening:
            raise FastPolicyError("RELEASE requires an opening target")
    return decision


def validate_decision(decision, current_phase, *, limits=None, controller_state=None):
    """Phase semantics plus explicit execution context; still needs a motion guard.

    Requiring explicit limit/state context here is not numerical approval.
    Numerical bounds, freshness, commissioning, and dispatch belong to safety.
    """
    decision = validate_phase_decision(decision, current_phase, controller_state=controller_state)
    if decision.action in ("move_eef", "move_eef_chunk", "gripper"):
        if not isinstance(limits, dict) or not limits:
            raise FastPolicyError("explicit motion limits are required; no physical defaults")
        if controller_state is None:
            raise FastPolicyError("measured controller state is required")
    return decision


def completion_phase(decision):
    """Call only after verified mechanical completion; makes no visual claim."""
    has_explanation = isinstance(decision, Decision) and decision.explanation is not None or isinstance(decision, dict) and "explanation" in decision
    decision = parse_response(decision, require_explanation=has_explanation)
    if decision.action == "gripper" and decision.phase in ("GRASP", "RELEASE"):
        return _NEXT[decision.phase]
    if decision.action in ("move_eef", "move_eef_chunk"):
        return decision.arguments["next_phase"] or decision.phase
    if decision.action == "advance":
        return decision.arguments["next_phase"]
    return decision.phase
