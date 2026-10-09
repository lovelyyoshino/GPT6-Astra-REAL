"""Task-bound observation tolerances, never a calibration or dispatch grant.

No samples are filtered or rebased. Only the named right-J4 policy changes;
hardware limits, raw position/jaw checks and all timing/send guards stay external.
"""
import copy
import math

PROFILE = "right_j4_bounded_v1"
STANDARD_RAD = .003
RIGHT_J4_RAD = math.radians(.5)
PROFILE_KEY = "_pair_feedback_observation"


def validate_policy(value, *, task_id=None):
    if value is None:
        return None
    if (type(value) is not dict or set(value) != {"profile", "source", "statement"}
            or value["profile"] != PROFILE or value["source"] != "user"
            or type(value["statement"]) is not str
            or not 1 <= len(value["statement"].strip()) <= 2000
            or task_id is not None and task_id != "plug_transfer_left"):
        raise ValueError("Explicit user-authorized plug-task right-J4 observation policy required")
    return copy.deepcopy(value)


def task_policy(task):
    policy = task.get("site_context", {}).get("feedback_observation")
    if policy is not None and task.get("task_id") != "plug_transfer_left":
        raise ValueError("Observation profile is bound to plug_transfer_left")
    return validate_policy(policy, task_id=task.get("task_id"))


def joint_tolerances(policy, side):
    if side not in ("left", "right"):
        raise ValueError("Known arm required for observation tolerance")
    policy = validate_policy(policy)
    values = [STANDARD_RAD] * 6
    if policy is not None and side == "right":
        values[3] = RIGHT_J4_RAD
    return values


def rotation_tolerance(policy, side):
    """Same task-bound angular observation band for controller SO(3) feedback."""
    return max(joint_tolerances(policy, side))


def joints_within(policy, side, before, after):
    if (len(before) != 6 or len(after) != 6
            or any(type(v) not in (float, int) or not math.isfinite(v) for v in (*before, *after))):
        raise ValueError("Six finite unmodified joint values required")
    return all(abs(a-b) <= limit for a, b, limit in
               zip(before, after, joint_tolerances(policy, side)))


def window_joints_within(policy, side, window):
    return joints_within(policy, side, window["qlow"], window["qhigh"])


def describe(policy):
    policy = validate_policy(policy)
    return {"authorization": policy,
            "joint_tolerances_rad": {s: joint_tolerances(policy, s) for s in ("left", "right")},
            "stationary_rotation_tolerances_rad": {s: rotation_tolerance(policy,s) for s in ("left","right")},
            "reference": "fixed_original_anchor_or_commanded_target; never rolling",
            "raw_samples_preserved": True, "physical_error_cause_verified": False}


def model_still_limits(policy, side, mdh, position_m, rotation_rad):
    """Account for the explicitly accepted extra J4 angle in derived FK only.

    Modified-DH remaining translations bound flange displacement for J4.
    Raw translation limits are NOT enlarged. Active process/target limits
    are independently checked, and the planner includes the full tracking box.
    """
    extra = joint_tolerances(policy, side)[3] - STANDARD_RAD
    radius = abs(mdh[3][0]) + sum(abs(row[0]) + abs(row[1]) for row in mdh[4:])
    return position_m + radius * extra, rotation_rad + extra
