"""Assess a live proposal without granting permission or contacting a device.

The supplied robot's prepare_command is a pure envelope builder. This module
never observes, executes, imports ROS, or substitutes fictional motion limits.
"""
from copy import deepcopy

from .fast_policy import (FastPolicyError, parse_response, requires_explanation,
                          validate_phase_decision)
from .fast_safety import physical_blockers


def assess_live_proposal(raw, state, robot):
    """Keep schema, phase, numerical evidence, and physical authority distinct.

    No phase transition or task-success flag is applied by this assessment.
    A pure prepared envelope may report numerical checks against explicit live
    limits; an absent or mock provenance can never make that flag true here.
    """
    report = {
        "assessment_mode": "live_readonly_proposal",
        "schema_valid": False, "phase_valid": False,
        "numeric_limits_verified": False,
        "numeric_validation_status": "not_checked",
        "command_encoding_valid": None,
        "physical_execution_allowed": False,
        "blockers": list(physical_blockers()),
        "command_proposal": None, "parsed_action": None,
        "action_dispatched": False, "control_commands_sent": 0,
        "task_success": None, "phase_transition_applied": False,
    }
    try:
        exceptional = requires_explanation(state)
    except (FastPolicyError, TypeError, ValueError):
        report["blockers"].append("controller_state_invalid")
        return report
    try:
        decision = parse_response(raw, require_explanation=exceptional)
    except (FastPolicyError, TypeError, ValueError):
        report["blockers"].append("model_schema_invalid")
        return report
    report["schema_valid"] = True
    report["parsed_action"] = decision.to_dict()
    try:
        validate_phase_decision(decision, state["phase"], controller_state=state)
    except (FastPolicyError, TypeError, ValueError):
        report["blockers"].append("model_phase_semantics_invalid")
        return report
    report["phase_valid"] = True
    if decision.action not in ("move_eef", "move_eef_chunk", "gripper"):
        report["numeric_validation_status"] = "not_applicable_no_motion"
        return report
    report["command_encoding_valid"] = False
    try:
        proposal = robot.prepare_command(decision, state)
        if not isinstance(proposal, dict):
            raise TypeError("prepare_command must return an envelope object")
        report["command_proposal"] = deepcopy(proposal)
        report["command_encoding_valid"] = True
    except Exception as exc:
        report["blockers"].append("command_preparation_failed:" + type(exc).__name__)
        return report
    # This metadata may document a pure check, never dispatch authority. The
    # real adapter owns measurement/limit provenance; do not use MockMotionGuard.
    verified = (proposal.get("numeric_limits_verified") is True
                and proposal.get("limits_source") == "explicit_physical"
                and proposal.get("nonphysical") is False)
    report["numeric_limits_verified"] = verified
    report["numeric_validation_status"] = "explicit_physical_limits_checked" if verified else "physical_limits_unverified"
    if not verified:
        report["blockers"].append("physical_numeric_limits_not_verified")
    return report
