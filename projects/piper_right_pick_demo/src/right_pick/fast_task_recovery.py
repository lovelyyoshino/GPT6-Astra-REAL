"""Bounded offline recovery proposals, never phase transitions or robot actions.

The caller supplies a frozen current stage and host execution evidence. Labels
and references are checked structurally, not authenticated. This module neither
reads RGB nor determines that a physical stop/hold is safe. A fresh host must
admit any later explicit request and consume the existing durable budget.
"""
import copy
import math

from .fast_pipeline import PipelineContractError
from .fast_task_pipeline import composition_contract


CATEGORIES = ("perception_uncertain", "occlusion", "grasp_failed", "no_progress",
              "zero_tx_rejected", "unknown_send", "partial_send", "timeout",
              "hold_unverified", "execution_fault", "budget_exhausted")
_STATUSES = ("not_attempted", "confirmed_zero_tx", "completed", "unknown", "partial")
_STAGE_FIELDS = {"task", "stage", "skill", "arm", "goal", "expect", "needs",
                 "cycles_left", "model_calls_left", "termination_reason"}


def _text(value, label, limit=400):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise PipelineContractError(label + " must be nonempty bounded text")
    return value


def _integer(value, label):
    if type(value) is not int or value < 0:
        raise PipelineContractError(label + " must be a nonnegative integer")
    return value


def _exact(value, fields, label):
    if not isinstance(value, dict) or set(value) != set(fields):
        raise PipelineContractError(label + " has missing or unknown fields")


def _validate(current, result, budget):
    if current is not None:
        if (not isinstance(current, dict) or not {"task", "stage", "skill", "arm"} <= set(current)
                or set(current) - _STAGE_FIELDS):
            raise PipelineContractError("Current stage must be the bounded host stage, or null when terminal")
        for name in ("task", "stage", "skill"):
            _text(current[name], "current " + name)
        composition_contract(current["skill"])
        if current["arm"] not in ("left", "right"):
            raise PipelineContractError("Current stage needs one explicit arm")
        if "goal" in current:
            _text(current["goal"], "current goal", 800)
        if current.get("termination_reason") is not None:
            _text(current["termination_reason"], "termination_reason")
        for name in ("cycles_left", "model_calls_left"):
            if name in current:
                _integer(current[name], name)
        for name in ("expect", "needs"):
            if name in current:
                if not isinstance(current[name], list) or len(current[name]) > 32:
                    raise PipelineContractError(name + " must be a bounded list")
                for item in current[name]:
                    _text(item, name + " item")
    _exact(result, ("task", "stage", "arm", "category", "observation_id", "source", "execution"),
           "Execution result")
    for name in ("task", "stage", "observation_id"):
        _text(result[name], "result " + name)
    if result["arm"] not in ("left", "right"):
        raise PipelineContractError("Execution result needs one explicit arm")
    if current is not None and any(result[k] != current[k] for k in ("task", "stage", "arm")):
        raise PipelineContractError("Execution result does not bind the current task/stage/arm")
    if result["source"] != "host_execution_report" or result["category"] not in CATEGORIES:
        raise PipelineContractError("Recovery requires a known failure category and host execution report")
    execution = result["execution"]
    _exact(execution, ("status", "attempted", "sent", "receipt_ref", "fault_latched", "hold_status"),
           "Execution evidence")
    if execution["status"] not in _STATUSES or execution["hold_status"] not in ("verified", "unverified", "unknown"):
        raise PipelineContractError("Unknown execution or hold status")
    if type(execution["fault_latched"]) is not bool:
        raise PipelineContractError("fault_latched must be a host boolean")
    for name in ("attempted", "sent"):
        if execution[name] is not None:
            _integer(execution[name], name)
    a, s = execution["attempted"], execution["sent"]
    if a is not None and s is not None and s > a:
        raise PipelineContractError("Sent count cannot exceed attempted count")
    ref = execution["receipt_ref"]
    if ref is not None:
        _text(ref, "receipt_ref", 2048)
    status = execution["status"]
    if status in ("not_attempted", "confirmed_zero_tx") and (a, s) != (0, 0):
        raise PipelineContractError("Zero-execution status contradicts dispatch counts")
    if status == "completed" and (a is None or s is None or a != s or s < 1 or ref is None):
        raise PipelineContractError("Completed execution requires a complete nonzero host receipt")
    if status == "confirmed_zero_tx" and ref is None:
        raise PipelineContractError("Confirmed zero-TX needs a host evidence reference")
    _exact(budget, ("cycles", "model_calls", "recoveries", "time_s"), "Remaining budget")
    for name in ("cycles", "model_calls", "recoveries"):
        _integer(budget[name], "remaining " + name)
    seconds = budget["time_s"]
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 0:
        raise PipelineContractError("Remaining time_s must be finite and nonnegative")


def build_recovery_contract(current_stage, execution_result, remaining_budget):
    """Return a diagnostic contract; never grant permission, replay or mutate.

    Dispatch counts use the caller's fixed unit (e.g. CAN frames or commands)
    consistently within one receipt. A zero count alone is not confirmed zero
    execution. Unknown/partial transactions and latched faults dominate visual
    diagnoses. Even supplied hold_verified evidence cannot unlock them here.
    """
    _validate(current_stage, execution_result, remaining_budget)
    current, result, budget = current_stage, execution_result, remaining_budget
    execution = result["execution"]
    category = result["category"]
    a, s, status = execution["attempted"], execution["sent"], execution["status"]
    blockers = []
    if status == "partial" or a is not None and s is not None and 0 < s < a:
        blockers.append("partial_send")
    elif status == "unknown" or a is None or s is None:
        blockers.append("unknown_send")
    if execution["fault_latched"] or category == "execution_fault":
        blockers.append("execution_fault")
    if execution["hold_status"] == "unverified":
        blockers.append("hold_unverified")
    if category in ("partial_send", "unknown_send", "timeout", "hold_unverified"):
        blockers.append(category)
    if current is None or current.get("termination_reason") is not None:
        blockers.append("terminal_state")
    exhausted = [name for name, amount in budget.items() if amount == 0]
    if current is not None:
        exhausted += [name for name in ("cycles_left", "model_calls_left") if current.get(name) == 0]
    if exhausted or category == "budget_exhausted":
        blockers.append("budget_exhausted")
    if category == "zero_tx_rejected" and status != "confirmed_zero_tx":
        blockers.append("zero_tx_not_confirmed")
    # Attempted but unconfirmed send must not be mistaken for an ordinary
    # visual failure, even when the caller omitted an explicit unknown label.
    if a and status not in ("completed", "partial", "unknown"):
        blockers.append("unknown_send")
    blockers = list(dict.fromkeys(blockers))
    blocked = bool(blockers)
    gates = ["fresh_rgb_after_reported_observation", "fresh_robot_and_gripper_feedback",
             "same_frozen_task_and_remaining_budget", "independent_host_action_admission"]
    if blocked:
        gates += ["current_verified_hold_or_stop_evidence", "resolve_execution_outcome_and_fault_with_host",
                  "explicit_reviewed_continuation_not_an_automatic_retry"]
    candidates = []
    if not blocked:
        candidates.append(dict(operation="inspect", goal="Resolve the current local uncertainty from new RGB",
                               requires=["fresh_rgb_after_reported_observation"], motion_parameters=None))
        # Unknown hold evidence still permits a read-only visual diagnosis, but
        # must not produce even a local motion candidate until the host checks it.
        if execution["hold_status"] == "unknown":
            gates.append("current_verified_hold_or_stop_evidence_before_motion_candidate")
        elif category == "grasp_failed":
            candidates.append(dict(operation="align", goal="Reconsider local grasp alignment from the current scene",
                requires=["object_support_or_held_load_independently_reviewed", "new_explicit_proposal",
                          "independent_host_action_admission"], motion_parameters=None))
        elif category in ("no_progress", "zero_tx_rejected") and current["skill"] != "inspect":
            candidates.append(dict(operation=current["skill"], goal=current.get("goal", "Reassess this operation's frozen local goal"),
                requires=["new_strategy_or_corrected_request_from_current_evidence", "new_explicit_proposal",
                          "independent_host_action_admission"], motion_parameters=None))
    cycles = min(budget["cycles"], current.get("cycles_left", budget["cycles"])) if current else 0
    calls = min(budget["model_calls"], current.get("model_calls_left", budget["model_calls"])) if current else 0
    return dict(schema_version="task_recovery_contract_v1", scope="offline_diagnostic_proposal",
        current_stage=copy.deepcopy(current), reported_category=category,
        classification=blockers[0] if blocked else category, status="blocked" if blocked else "proposed",
        blocking_reasons=blockers, exhausted_budget_fields=exhausted,
        execution_evidence=copy.deepcopy(execution), evidence_authentication_performed=False,
        observation_anchor=result["observation_id"], fresh_rgb_required=True,
        evidence_gates=gates, candidate_operations=candidates,
        selection_policy="at_most_one_candidate_after_new_evidence_not_a_sequence",
        remaining_budget=copy.deepcopy(budget),
        proposal_budget=dict(max_new_observations=0 if blocked else min(1, cycles),
            max_model_proposals=0 if blocked else min(1, calls),
            max_recovery_attempts=0 if blocked else min(1, budget["recoveries"])),
        budget_consumed=False, durable_accounting_required=True,
        execution_available=False, dispatched_action_count=0, phase_transition_applied=False,
        actual_regression_implemented=False, replay_previous_action=False,
        automatic_retry=False, clears_failure_latch=False,
        prohibitions=["no_historical_pose_return", "no_target_clipping_or_force_escalation",
                      "no_provider_retry_as_action_retry", "no_stop_reset_disable_command",
                      "no_budget_reset_on_observer_or_restart"],
        limitations=["Inputs are host declarations; references are not authenticated by this pure function.",
                     "A candidate operation is not a phase rollback, trajectory or action authorization.",
                     "A host must persist consumption in the existing session before any new admitted action.",
                     "Hold/stop gates request evidence, not a stop command; client exit is not hold verification."])
