"""Small, shared contracts for atomic skills and bounded execution pipelines.

The model proposes one action. The host owns the pipeline and must not let a
skill silently perform another skill's work. This module contains pure contracts;
it never imports a camera, robot, ROS client, SDK, or model backend.
"""
from dataclasses import dataclass
import math
import time


class PipelineContractError(ValueError):
    """Raised when a pipeline definition is incomplete or malformed."""


@dataclass(frozen=True)
class AtomicSkill:
    name: str
    purpose: str
    inputs: tuple
    outputs: tuple
    side_effect: str
    max_invocations: int
    failure: str
    source: str = "piper_arx5_mapped"

    def compact(self):
        return {
            "name": self.name,
            "inputs": list(self.inputs),
            "outputs": list(self.outputs),
            "side_effect": self.side_effect,
            "max_invocations": self.max_invocations,
            "failure": self.failure,
            "source": self.source,
            "layer": 1,
        }


ATOMIC_SKILLS = (
    AtomicSkill(
        "observe_scene", "Capture one fresh RGB/state snapshot.",
        ("task", "camera_request", "robot_state_request"),
        ("observation_id", "rgb_views", "measured_state"),
        "none", 1, "stop_if_missing_or_stale",
    ),
    AtomicSkill(
        "decide_one", "Choose one action for the current phase.",
        ("observation_id", "phase_contract", "compact_state", "rgb_views"),
        ("one_action", "confidence"),
        "none", 1, "pause_or_reject_invalid_json",
    ),
    AtomicSkill(
        "admit_action", "Check the proposal against fresh state and limits.",
        ("one_action", "measured_state", "action_budget"),
        ("admission_receipt",),
        "none", 1, "reject_without_dispatch",
    ),
    AtomicSkill(
        "dispatch_once", "Send at most one admitted physical command.",
        ("admission_receipt", "one_action"),
        ("dispatch_receipt",),
        "physical_command", 1, "latch_uncertain_outcome",
    ),
    AtomicSkill(
        "read_receipt", "Classify the command result without inferring task success.",
        ("dispatch_receipt", "feedback"),
        ("execution_receipt",),
        "none", 1, "latch_on_missing_or_partial_receipt",
    ),
    AtomicSkill(
        "verify_visual", "Use a new RGB snapshot to assess the expected evidence.",
        ("execution_receipt", "new_observation_id", "rgb_views"),
        ("visual_evidence",),
        "none", 1, "pause_if_evidence_is_unknown",
    ),
    AtomicSkill(
        "decide_pair", "Choose one bounded proposal per arm from one scene version.",
        ("observation_id", "phase_contract", "compact_state", "rgb_views"),
        ("left_proposal", "right_proposal", "confidence"),
        "none", 1, "pause_or_reject_if_either_proposal_is_invalid",
    ),
    AtomicSkill(
        "coordinate_pair", "Run joint preflight without sending either arm.",
        ("left_proposal", "right_proposal", "pair_state", "pair_limits"),
        ("paired_admission",),
        "none", 1, "latch_both_arms_on_any_fault",
    ),
    AtomicSkill(
        "dispatch_pair_once", "Send one admitted pair through one shared barrier.",
        ("paired_admission",),
        ("paired_dispatch_receipt",),
        "paired_physical_command", 1, "latch_both_arms_on_partial_or_uncertain_send",
    ),
    AtomicSkill(
        "coordinate_observer", "Enforce worker/observer role and worker hold before observer motion.",
        ("worker_proposal", "observer_proposal", "role_state", "observer_limits"),
        ("role_admission",),
        "none", 1, "reject_observer_task_action_or_unheld_worker",
    ),
    # These skills are the Piper-side equivalents of the ARX5 execution
    # primitives.  They are contracts only; the ROS/camera adapter supplies
    # the measured fields and remains the owner of physical side effects.
    AtomicSkill(
        "preflight_single", "Qualify one arm, owner, heartbeat and timeout policy before resources open.",
        ("mode_config", "robot_readiness", "session_readiness"),
        ("preflight_receipt",), "none", 1, "stop_before_observe_or_dispatch",
    ),
    AtomicSkill(
        "preflight_pair", "Qualify both arms and the shared coordinator as one admission.",
        ("pair_readiness", "role_state", "session_readiness"),
        ("pair_preflight_receipt",), "none", 1, "latch_both_arms_on_any_fault",
    ),
    AtomicSkill(
        "check_fresh_observation", "Verify advancing camera identities, freshness and sensor skew.",
        ("observation", "previous_observation", "freshness_limits"),
        ("observation_receipt",), "none", 1, "hold_and_request_new_observation",
    ),
    AtomicSkill(
        "plan_swept_corridor", "Check the complete proposed arm corridor against the current scene.",
        ("proposal", "measured_state", "scene_version", "workspace_limits"),
        ("corridor_receipt",), "none", 1, "reject_without_dispatch",
    ),
    AtomicSkill(
        "synchronize_duration", "Choose one bounded duration for admitted paired segments.",
        ("left_plan", "right_plan", "duration_limits"),
        ("common_duration_receipt",), "none", 1, "reject_pair_without_dispatch",
    ),
    AtomicSkill(
        "prepare_held_side", "Prove a null/held side is stationary before the other side moves.",
        ("arm_state", "hold_target", "owner"),
        ("held_side_receipt",), "none", 1, "latch_pair_if_hold_is_unknown",
    ),
    AtomicSkill(
        "renew_session", "Renew a bounded control session and both heartbeat leases.",
        ("session_receipt", "heartbeat_state", "arm_ids"),
        ("renewal_receipt",), "none", 1, "stop_before_next_dispatch",
    ),
    AtomicSkill(
        "latch_pair_fault", "Propagate one uncertain or failed arm outcome to the entire pair.",
        ("pair_state", "fault_reason", "arm_receipts"),
        ("pair_fault_latch",), "protective_latch", 1, "requires_operator_recovery",
    ),
    AtomicSkill(
        "freeze_observer", "Freeze an observing host while preserving the worker hold.",
        ("worker_hold", "observer_state", "handoff_receipt"),
        ("observer_freeze_receipt",), "none", 1, "latch_both_arms_on_race",
    ),
    AtomicSkill(
        "handoff_hold", "Transfer a stationary held session only after unchanged targets are proven.",
        ("saved_hold", "current_hold", "owner"),
        ("handoff_receipt",), "none", 1, "reject_without_releasing_hold",
    ),
    AtomicSkill(
        "verify_return", "Compare both arms with this run's initial pose and hold state.",
        ("initial_state", "current_state", "return_tolerance"),
        ("return_receipt",), "none", 1, "stop_with_return_unverified",
    ),
    AtomicSkill(
        "verify_task_evidence", "Require ordered visual evidence for grasp, lift, release and stable placement.",
        ("expected_evidence", "fresh_observation", "execution_receipt"),
        ("task_evidence_receipt",), "none", 1, "pause_without_claiming_success",
    ),
    AtomicSkill(
        "move_eef_once", "Execute one finite admitted endpoint through dispatch_once.",
        ("admission_receipt", "pose_m_rad", "speed_limit"),
        ("execution_receipt",), "physical_command", 1, "latch_uncertain_outcome",
    ),
    AtomicSkill(
        "set_gripper_once", "Execute one admitted opening target; never infer grasp from closure.",
        ("admission_receipt", "opening_m", "effort_parameter_nm"),
        ("execution_receipt",), "physical_command", 1, "latch_uncertain_outcome",
    ),
    AtomicSkill(
        "sample_stability", "Compare two distinct post-release observations separated in time.",
        ("fresh_samples", "minimum_gap_s", "expected_support"),
        ("stability_receipt",), "none", 1, "stop_with_stability_unverified",
    ),
)

_SKILLS = {skill.name: skill for skill in ATOMIC_SKILLS}
SINGLE_ARM_PIPELINE_ID = "single_arm_closed_loop_v1"
DUAL_ARM_PIPELINE_ID = "dual_arm_barrier_v1"
WORKER_OBSERVER_PIPELINE_ID = "single_worker_with_observer_v1"
SINGLE_ARM_PIPELINE = (
    "observe_scene", "decide_one", "admit_action", "dispatch_once",
    "read_receipt", "verify_visual",
)
DUAL_ARM_PIPELINE = (
    "observe_scene", "decide_pair", "coordinate_pair", "dispatch_pair_once",
    "read_receipt", "verify_visual",
)
WORKER_OBSERVER_PIPELINE = (
    "observe_scene", "decide_one", "coordinate_observer", "admit_action",
    "dispatch_once", "read_receipt", "verify_visual",
)

_PIPELINES = {
    SINGLE_ARM_PIPELINE_ID: ("single_arm", SINGLE_ARM_PIPELINE),
    DUAL_ARM_PIPELINE_ID: ("dual_arm", DUAL_ARM_PIPELINE),
    WORKER_OBSERVER_PIPELINE_ID: ("worker_with_observer", WORKER_OBSERVER_PIPELINE),
}
_PIPELINE_LIFECYCLE = {
    SINGLE_ARM_PIPELINE_ID: {
        "preflight": ("preflight_single",),
        "guards": ("check_fresh_observation", "plan_swept_corridor"),
        "cycle": SINGLE_ARM_PIPELINE,
        "close": ("verify_task_evidence", "verify_return"),
        "session": ("renew_session",),
    },
    DUAL_ARM_PIPELINE_ID: {
        "preflight": ("preflight_pair",),
        "guards": ("check_fresh_observation", "plan_swept_corridor", "synchronize_duration",
                    "prepare_held_side"),
        "cycle": DUAL_ARM_PIPELINE,
        "close": ("verify_task_evidence", "verify_return"),
        "session": ("renew_session", "latch_pair_fault"),
    },
    WORKER_OBSERVER_PIPELINE_ID: {
        "preflight": ("preflight_pair",),
        "guards": ("check_fresh_observation", "plan_swept_corridor", "prepare_held_side"),
        "cycle": WORKER_OBSERVER_PIPELINE,
        "observer_handoff": ("freeze_observer", "handoff_hold"),
        "close": ("verify_task_evidence", "verify_return"),
        "session": ("renew_session", "latch_pair_fault"),
    },
}
_PIPELINE_ALIASES = {
    "single_arm": SINGLE_ARM_PIPELINE_ID,
    "dual_arm": DUAL_ARM_PIPELINE_ID,
    "worker_with_observer": WORKER_OBSERVER_PIPELINE_ID,
    "observer": WORKER_OBSERVER_PIPELINE_ID,
}
_ARMS = frozenset(("left", "right"))


def atomic_skill(name):
    try:
        return _SKILLS[name]
    except KeyError:
        raise PipelineContractError("Unknown atomic skill: " + str(name))


def validate_pipeline(stages):
    if not isinstance(stages, (tuple, list)) or not stages:
        raise PipelineContractError("Pipeline must contain at least one stage")
    for stage in stages:
        atomic_skill(stage)
    if len(stages) != len(set(stages)):
        raise PipelineContractError("A bounded pipeline cannot repeat an atomic stage")
    return tuple(stages)


def pipeline_contract(pipeline="single_arm"):
    pipeline_id = _PIPELINE_ALIASES.get(pipeline, pipeline)
    try:
        execution_mode, stages = _PIPELINES[pipeline_id]
    except (KeyError, TypeError):
        raise PipelineContractError("Unknown pipeline: " + str(pipeline))
    validate_pipeline(stages)
    contract = {
        "id": pipeline_id,
        "execution_mode": execution_mode,
        "stages": list(stages),
        "max_dispatches_per_cycle": 1,
        "stop_on_uncertain_receipt": True,
        "lifecycle": {
            key: list(value) for key, value in _PIPELINE_LIFECYCLE[pipeline_id].items()
        },
        "required_atomic_skills": list(dict.fromkeys(
            skill for values in _PIPELINE_LIFECYCLE[pipeline_id].values() for skill in values
        )),
        "termination": {
            "uncertain_receipt": "execution_failed_latched",
            "stale_observation": "fresh_observation_required",
            "budget_exhausted": "budget_exhausted",
            "return_unverified": "return_unverified",
        },
    }
    if execution_mode == "dual_arm":
        contract.update({
            "dispatch_barrier": "shared",
            "receipt_scope": "left_and_right",
            "partial_send_policy": "latch_both_arms",
        })
    elif execution_mode == "worker_with_observer":
        contract.update({
            "worker_arm": "configured",
            "observer_arm": "configured",
            "observer_can_execute_task_action": False,
            "observer_requires_worker_hold": True,
            "observer_refreshes_scene": True,
        })
    return contract


def compact_pipeline(pipeline="single_arm", stage="decide_one"):
    """Return the small pipeline fragment safe to include in every model call."""
    contract = pipeline_contract(pipeline)
    if stage not in contract["stages"]:
        raise PipelineContractError("Stage is not part of pipeline: " + str(stage))
    stages = contract["stages"]
    index = stages.index(stage)
    return {
        "id": contract["id"],
        "mode": contract["execution_mode"],
        "stage": stage,
        "next": stages[index + 1] if index + 1 < len(stages) else None,
    }


def decision_stage(pipeline="single_arm"):
    """Return the only model stage for the selected pipeline."""
    mode = pipeline_contract(pipeline)["execution_mode"]
    return "decide_pair" if mode == "dual_arm" else "decide_one"


def validate_mode_config(config):
    """Validate role wiring before any model, camera, or robot is opened."""
    if not isinstance(config, dict):
        raise PipelineContractError("Pipeline config must be an object")
    pipeline_id = config.get("pipeline_id", SINGLE_ARM_PIPELINE_ID)
    contract = pipeline_contract(pipeline_id)
    if config.get("execution_mode", contract["execution_mode"]) != contract["execution_mode"]:
        raise PipelineContractError("execution_mode does not match pipeline_id")
    worker = config.get("worker_arm", "right")
    observer = config.get("observer_arm")
    peer = config.get("peer_arm")
    if worker not in _ARMS:
        raise PipelineContractError("worker_arm must be left or right")
    if contract["execution_mode"] == "single_arm":
        if observer is not None or peer is not None:
            raise PipelineContractError("single_arm cannot configure a second arm")
    elif contract["execution_mode"] == "dual_arm":
        if observer is not None or peer not in _ARMS or peer == worker:
            raise PipelineContractError("dual_arm requires a distinct task peer_arm, not observer_arm")
    else:
        if peer is not None or observer not in _ARMS or observer == worker:
            raise PipelineContractError("observer mode requires a distinct view-only observer_arm")
    return {"pipeline_id": contract["id"], "execution_mode": contract["execution_mode"],
            "worker_arm": worker, "observer_arm": observer, "peer_arm": peer}


def observer_action_allowed(action, *, worker_held):
    """Apply the observer role gate to a host-side action envelope."""
    if not isinstance(action, dict) or action.get("role") != "observer":
        return False
    if action.get("intent") != "view_only" or action.get("task_effect") is not False:
        return False
    if action.get("action") == "observe":
        return True
    return action.get("action") == "move_eef" and worker_held is True


def validate_runtime_config(config):
    """Prevent a declared pair/task recipe from falling through to the right-arm runner."""
    mode = validate_mode_config(config)
    if mode["execution_mode"] != "single_arm" or mode["worker_arm"] != "right":
        raise PipelineContractError("Current fast runner supports only single right arm; use offline task planning for coordinated modes")
    if config.get("task_id", "pen") != "pen":
        raise PipelineContractError("Current fast phase runner supports only pen; other recipes are offline contracts")
    return mode


def atomic_skill_catalog():
    """Return an audit-friendly catalog without mutable dataclass objects."""
    return [skill.compact() for skill in ATOMIC_SKILLS]


def _finite(value, name, *, minimum=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise PipelineContractError(name + " must be finite")
    if minimum is not None and value < minimum:
        raise PipelineContractError(name + " must be >= " + str(minimum))
    return float(value)


def _mapping(value, name):
    if not isinstance(value, dict):
        raise PipelineContractError(name + " must be an object")
    return value


def validate_fresh_observation(observation, *, required_views=("front", "left_hand", "right_hand"),
                               max_age_s=0.8, max_skew_s=0.15, now=None,
                               previous_observation=None, allow_historical=False,
                               require_identity=False):
    """Validate the ARX5-style fresh multi-camera receipt without opening a camera."""
    observation = _mapping(observation, "observation")
    if (observation.get("historical") is True or observation.get("nonphysical") is True) and not allow_historical:
        raise PipelineContractError("Historical observation cannot admit a physical action")
    observation_id = observation.get("observation_id", observation.get("capture_id"))
    if not isinstance(observation_id, str) or not observation_id:
        raise PipelineContractError("observation_id is required")
    cameras = _mapping(observation.get("cameras"), "observation.cameras")
    if not required_views or len(required_views) != len(set(required_views)):
        raise PipelineContractError("required_views must be nonempty and unique")
    if observation.get("host_receipt_skew_exceeded") is True:
        raise PipelineContractError("camera reports excessive skew")
    max_age_s, max_skew_s = _finite(max_age_s, "max_age_s", minimum=0), _finite(max_skew_s, "max_skew_s", minimum=0)
    now = time.time() if now is None else _finite(now, "now")
    stamps, sequences, devices = [], {}, {}
    for view in required_views:
        frame = _mapping(cameras.get(view), "camera " + view)
        if frame.get("host_receipt_stale") is True or frame.get("historical") is True and not allow_historical:
            raise PipelineContractError("camera " + view + " is historical or stale")
        stamp = frame.get("host_received_at", frame.get("host_received_at_s", frame.get("timestamp")))
        stamp = _finite(stamp, view + ".timestamp")
        age = now - stamp
        if age < 0 or age > max_age_s:
            raise PipelineContractError("camera " + view + " is stale or future-dated")
        stamps.append(stamp)
        sequence = frame.get("sequence", frame.get("frame_number"))
        if sequence is not None:
            if type(sequence) is not int or sequence <= 0:
                raise PipelineContractError(view + ".sequence must be a positive integer")
            sequences[view] = sequence
        device = frame.get("device", frame.get("serial"))
        if device is not None:
            if not isinstance(device, str) or not device:
                raise PipelineContractError(view + ".device must be a non-empty string")
            devices[view] = device
        if require_identity and (view not in sequences or view not in devices):
            raise PipelineContractError(view + " needs measured serial and frame_number")
    skew = max(stamps) - min(stamps)
    if skew > max_skew_s:
        raise PipelineContractError("camera timestamp skew exceeds limit")
    if devices and len(devices) != len(set(devices.values())):
        raise PipelineContractError("camera identities must be distinct")
    if previous_observation is not None:
        previous_observation = _mapping(previous_observation, "previous_observation")
        if previous_observation.get("observation_id") == observation_id:
            raise PipelineContractError("observation_id did not advance")
        previous_sequences = previous_observation.get("sequences", {})
        previous_devices = previous_observation.get("devices", {})
        if previous_devices and devices != previous_devices:
            raise PipelineContractError("camera identity changed during session")
        if sequences and isinstance(previous_sequences, dict):
            for view, sequence in sequences.items():
                if view in previous_sequences and sequence <= previous_sequences[view]:
                    raise PipelineContractError("camera sequence did not advance: " + view)
    return {
        "observation_id": observation_id,
        "views": list(required_views),
        "sequences": sequences,
        "devices": devices,
        "oldest_age_s": max(0.0, now - min(stamps)),
        "skew_s": skew,
        "fresh": True,
        "timestamp_basis": "host_receipt_not_synchronized_exposure",
    }


def validate_preflight(readiness, *, pair=False):
    """Require explicit readiness evidence; configuration booleans cannot bypass it."""
    readiness = _mapping(readiness, "readiness")
    required = ("physical_execution_ready", "timeout_policy_verified")
    if any(readiness.get(key) is not True for key in required):
        raise PipelineContractError("physical execution or timeout policy is not qualified")
    blockers = readiness.get("blockers", [])
    if not isinstance(blockers, list) or blockers:
        raise PipelineContractError("preflight has unresolved blockers")
    qualification = readiness.get("qualification")
    if not isinstance(qualification, dict) or not qualification:
        raise PipelineContractError("qualification receipt is required")
    if pair:
        arms = readiness.get("arms")
        if not isinstance(arms, dict) or set(arms) != set(_ARMS):
            raise PipelineContractError("pair preflight requires left and right readiness")
        if any(not isinstance(value, dict) or value.get("ready") is not True for value in arms.values()):
            raise PipelineContractError("both arms must be ready")
    return {
        "qualified": True,
        "pair": bool(pair),
        "timeout_policy_verified": True,
        "physical_motion_authorized": readiness.get("physical_motion_authorized") is True,
        "qualification": qualification,
    }


def validate_swept_corridors(corridors, *, scene_version, arms=("right",), require_clear=True):
    """Reject a proposal unless every complete swept corridor is checked in this scene."""
    corridors = _mapping(corridors, "corridors")
    if not isinstance(scene_version, str) or not scene_version:
        raise PipelineContractError("scene_version is required")
    checked = []
    for arm in arms:
        corridor = _mapping(corridors.get(arm), "corridor " + arm)
        if corridor.get("scene_version") != scene_version:
            raise PipelineContractError("corridor scene version mismatch: " + arm)
        if require_clear and corridor.get("clear") is not True:
            raise PipelineContractError("swept corridor is not clear: " + arm)
        checked.append(arm)
    return {"scene_version": scene_version, "arms": checked, "clear": True}


def synchronize_duration(left_plan, right_plan, *, max_duration_s=None):
    """Return a common positive duration; this does not claim hard real-time sync."""
    left_plan, right_plan = _mapping(left_plan, "left_plan"), _mapping(right_plan, "right_plan")
    left = _finite(left_plan.get("duration_s"), "left_plan.duration_s", minimum=0)
    right = _finite(right_plan.get("duration_s"), "right_plan.duration_s", minimum=0)
    common = max(left, right)
    if left <= 0 or right <= 0:
        raise PipelineContractError("active plans require positive duration")
    if max_duration_s is not None and common > _finite(max_duration_s, "max_duration_s", minimum=0):
        raise PipelineContractError("common duration exceeds limit")
    return {"common_duration_s": common, "left_duration_s": left,
            "right_duration_s": right, "hard_realtime": False}


def validate_pair_admission(left_proposal, right_proposal, *, observation_id,
                            corridor_receipt=None, duration_receipt=None,
                            held_sides=(), held_receipts=None):
    """Join two proposals only after shared-scene and held/null checks."""
    if not isinstance(observation_id, str) or not observation_id:
        raise PipelineContractError("pair admission requires observation_id")
    proposals = {"left": left_proposal, "right": right_proposal}
    held_sides = tuple(held_sides)
    if any(side not in _ARMS for side in held_sides) or len(set(held_sides)) != len(held_sides):
        raise PipelineContractError("held_sides must contain unique arm names")
    if len(held_sides) != 1:
        raise PipelineContractError("Current task pipelines require exactly one moving arm and one held side")
    held_receipts = _mapping(held_receipts if held_receipts is not None else {}, "held receipts")
    if set(held_receipts) != set(held_sides):
        raise PipelineContractError("every held side needs its own receipt")
    for side, proposal in proposals.items():
        if side in held_sides:
            if proposal not in (None, {}, {"action": "null"}, {"action": "hold"}):
                raise PipelineContractError(side + " held side must be null/hold")
            held = _mapping(held_receipts[side], side + " held receipt")
            if (held.get("arm") != side or held.get("observation_id") != observation_id
                    or held.get("stationary") is not True or held.get("hold_verified") is not True
                    or not isinstance(held.get("owner"), str) or not held["owner"]
                    or type(held.get("sequence")) is not int or held["sequence"] <= 0):
                raise PipelineContractError(side + " has no current verified hold")
            continue
        proposal = _mapping(proposal, side + "_proposal")
        if proposal.get("observation_id") != observation_id:
            raise PipelineContractError(side + " proposal uses a different observation")
        if proposal.get("action") in (None, "null", "hold"):
            raise PipelineContractError(side + " active proposal is null/hold")
    corridor_receipt = _mapping(corridor_receipt, "pair corridor receipt")
    duration_receipt = _mapping(duration_receipt, "pair duration receipt")
    if (corridor_receipt.get("clear") is not True or corridor_receipt.get("scene_version") != observation_id
            or set(corridor_receipt.get("arms", [])) != set(_ARMS)):
        raise PipelineContractError("pair corridors are not clear")
    if _finite(duration_receipt.get("common_duration_s"), "common_duration_s", minimum=0) <= 0:
        raise PipelineContractError("pair has no common duration")
    return {"observation_id": observation_id, "held_sides": list(held_sides),
            "dispatch_barrier": "shared", "common_duration_s":
            None if duration_receipt is None else duration_receipt["common_duration_s"],
            "admitted": True}


def validate_pair_receipt(receipt, *, cycle_id=None, arms=("left", "right")):
    """Require independent per-arm receipts before classifying a pair as arrived."""
    receipt = _mapping(receipt, "pair receipt")
    actual_cycle = receipt.get("cycle_id")
    if not isinstance(actual_cycle, str) or not actual_cycle or (cycle_id is not None and actual_cycle != cycle_id):
        raise PipelineContractError("pair receipt cycle_id is missing or mismatched")
    entries = _mapping(receipt.get("arms"), "pair receipt arms")
    if set(entries) != set(arms):
        raise PipelineContractError("pair receipt must contain every arm")
    sequences = {}
    for arm in arms:
        item = _mapping(entries[arm], arm + " receipt")
        sequence = item.get("sequence")
        if type(sequence) is not int or sequence <= 0:
            raise PipelineContractError(arm + " receipt sequence is invalid")
        if item.get("accepted") is not True or not (item.get("arrival_confirmed") is True
                                                    or item.get("stability_confirmed") is True):
            raise PipelineContractError(arm + " receipt lacks independent arrival/stability")
        if item.get("tracking_confirmed") is not True:
            raise PipelineContractError(arm + " receipt lacks independent tracking")
        sequences[arm] = sequence
    if receipt.get("partial") is True or receipt.get("outcome_uncertain") is True:
        raise PipelineContractError("pair receipt is partial or uncertain")
    return {"cycle_id": actual_cycle, "sequences": sequences,
            "arms": list(arms), "complete": True}


def latch_pair_fault(pair_state, reason):
    """Create an irreversible host latch; it never invents a stop or recovery receipt."""
    pair_state = _mapping(pair_state, "pair_state")
    if not isinstance(reason, str) or not reason.strip():
        raise PipelineContractError("fault reason is required")
    existing = pair_state.get("fault_latch")
    if isinstance(existing, dict) and existing.get("latched") is True:
        return existing
    return {"latched": True, "reason": reason.strip(), "requires_operator": True,
            "arms": {side: {"motion_allowed": False, "state": "latched"} for side in _ARMS}}


def validate_session_renewal(receipt, *, arms=("right",)):
    receipt = _mapping(receipt, "session renewal")
    if receipt.get("renewed") is not True:
        raise PipelineContractError("session renewal was not confirmed")
    heartbeat = receipt.get("heartbeat")
    if heartbeat is not True:
        raise PipelineContractError("heartbeat renewal is not confirmed")
    arm_receipts = receipt.get("arms", {})
    if not isinstance(arm_receipts, dict) or set(arm_receipts) != set(arms):
        raise PipelineContractError("session renewal must cover every active arm")
    if any(value is not True for value in arm_receipts.values()):
        raise PipelineContractError("an arm session renewal is missing")
    return {"renewed": True, "arms": list(arms)}


def _compare_arm_vectors(old, new, *, position_tolerance_m, rotation_tolerance_rad,
                         joint_tolerance_rad):
    for field in ("pose_m_rad", "joints_rad", "commanded_joints_rad"):
        a, b = old.get(field), new.get(field)
        if not isinstance(a, list) or len(a) != 6 or not isinstance(b, list) or len(b) != 6:
            raise PipelineContractError(field + " needs six measured/reference values")
        a, b = [_finite(v, field) for v in a], [_finite(v, field) for v in b]
        limits = ([position_tolerance_m] * 3 + [rotation_tolerance_rad] * 3
                  if field == "pose_m_rad" else [joint_tolerance_rad] * 6)
        if any(abs(x-y) > limit for x, y, limit in zip(a, b, limits)):
            raise PipelineContractError(field + " differs from the saved reference")


def validate_hold_handoff(saved_hold, current_hold, *, owner=None,
                          position_tolerance_m=0.005, rotation_tolerance_rad=0.05,
                          joint_tolerance_rad=0.05):
    """Validate unchanged stationary targets before replacing an observing host."""
    saved_hold, current_hold = _mapping(saved_hold, "saved_hold"), _mapping(current_hold, "current_hold")
    limits = {name: _finite(value, name, minimum=0) for name, value in (
        ("position_tolerance_m", position_tolerance_m), ("rotation_tolerance_rad", rotation_tolerance_rad),
        ("joint_tolerance_rad", joint_tolerance_rad))}
    if set(saved_hold) != set(current_hold) or not set(saved_hold) or not set(saved_hold).issubset(_ARMS):
        raise PipelineContractError("hold handoff must cover the same arms")
    for arm in saved_hold:
        old, new = _mapping(saved_hold[arm], arm + " saved hold"), _mapping(current_hold[arm], arm + " current hold")
        if old.get("moving") is not False or new.get("moving") is not False:
            raise PipelineContractError(arm + " is not stationary")
        if old.get("control_state") != "holding" or new.get("control_state") != "holding":
            raise PipelineContractError(arm + " is not in holding state")
        expected_owner = owner if owner is not None else old.get("owner")
        if not isinstance(expected_owner, str) or not expected_owner or old.get("owner") != expected_owner or new.get("owner") != expected_owner:
            raise PipelineContractError(arm + " owner mismatch")
        _compare_arm_vectors(old, new, **limits)
    return {"handoff_ready": True, "arms": list(saved_hold), "released_hold": False}


def validate_return(initial_state, current_state, *, position_tolerance_m=0.01,
                    rotation_tolerance_rad=0.03, joint_tolerance_rad=0.03,
                    require_stationary=True):
    """Return evidence is relative to this run's saved state, never a historical pose."""
    initial_state, current_state = _mapping(initial_state, "initial_state"), _mapping(current_state, "current_state")
    limits = {name: _finite(value, name, minimum=0) for name, value in (
        ("position_tolerance_m", position_tolerance_m), ("rotation_tolerance_rad", rotation_tolerance_rad),
        ("joint_tolerance_rad", joint_tolerance_rad))}
    if set(initial_state) != set(current_state) or not set(initial_state) or not set(initial_state).issubset(_ARMS):
        raise PipelineContractError("return evidence must cover the same arms")
    for arm in initial_state:
        old, new = _mapping(initial_state[arm], arm + " initial"), _mapping(current_state[arm], arm + " current")
        if require_stationary and new.get("moving") is not False:
            raise PipelineContractError(arm + " is still moving")
        _compare_arm_vectors(old, new, **limits)
    return {"return_verified": True, "arms": list(initial_state), "tolerances": limits}


def validate_task_evidence(evidence, *, required=("grasp", "lift", "release", "stable")):
    """Accept only ordered, fresh visual evidence; motor receipts alone are insufficient."""
    if not isinstance(evidence, list):
        raise PipelineContractError("task evidence must be a list")
    if any(not isinstance(item, dict) for item in evidence):
        raise PipelineContractError("each task evidence item must be an object")
    positions = {item.get("stage"): index for index, item in enumerate(evidence)
                 if isinstance(item, dict) and item.get("confirmed") is True}
    missing = [stage for stage in required if stage not in positions]
    if missing:
        raise PipelineContractError("missing visual evidence: " + ",".join(missing))
    if any(positions[a] >= positions[b] for a, b in zip(required, required[1:])):
        raise PipelineContractError("visual evidence order is invalid")
    seen = set()
    last_at = None
    for stage in required:
        item = evidence[positions[stage]]
        if item.get("source") not in ("vision_measurement", "model_visual_report"):
            raise PipelineContractError("task evidence must come from a visual report")
        observation_id = item.get("observation_id")
        if not isinstance(observation_id, str) or not observation_id or observation_id in seen:
            raise PipelineContractError("visual milestones need distinct observation IDs")
        seen.add(observation_id)
        stamp = _finite(item.get("at"), "evidence.at")
        if last_at is not None and stamp <= last_at:
            raise PipelineContractError("visual evidence timestamps must advance")
        last_at = stamp
    return {"task_evidence_verified": True, "stages": list(required)}
