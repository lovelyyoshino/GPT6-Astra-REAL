"""Pure local flange IK, with no robot connection, target perception or TCP.

raw_q is six SDK millidegrees. Pose is [x,y,z,roll,pitch,yaw] in m/rad,
R = Rz(yaw) Ry(pitch) Rx(roll), relative to the arm base. Translation and
rotation-vector increments are both expressed in that same base frame:
R_requested = Exp(delta_rotvec_rad) * R_measured. No command is sent here.

One bounded solve starts at the supplied current branch. The quantized result
must pass the unchanged support.path_check. The caller must still reacquire
fresh feedback, repeat its execution guards, and review the physical path.
"""
import copy
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_joint_stop_probe_entry as support


POSITION_TOLERANCE_M = .0005
ROTATION_TOLERANCE_RAD = .005
MAX_JOINT_STEP_DEG = 2.
MIN_REQUESTED_DIRECTION_PROGRESS = .5


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def _vector(value, count, name):
    _require(isinstance(value, (list, tuple)) and len(value) == count,
             name + " must have exactly %d values" % count)
    _require(all(type(v) in (int, float) and math.isfinite(v) for v in value),
             name + " must contain finite numbers (not booleans)")
    return np.asarray(value, dtype=float)


def _checked_limits(limits):
    _require(isinstance(limits, dict), "Explicit original limits required")
    value = copy.deepcopy(limits)
    for name, outer in (("max_translation_step_m", .03), ("max_rotation_step_rad", .05),
                        ("max_state_age_s", .1)):
        v = value.get(name)
        _require(type(v) in (int, float) and math.isfinite(v) and 0 < v <= outer,
                 "Original limit cannot increase: " + name)
    low = _vector(value.get("workspace_min_m"), 3, "workspace_min_m")
    high = _vector(value.get("workspace_max_m"), 3, "workspace_max_m")
    _require(np.all(low < high) and np.all(low >= [-.6, -.6, .05]) and
             np.all(high <= [.6, .6, .65]), "Workspace must remain within original bounds")
    pairs = value.get("joint_limits_rad")
    _require(isinstance(pairs, (list, tuple)) and len(pairs) == 6, "Six joint limits required")
    for pair, nominal in zip(pairs, support.JOINT_LIMITS_RAW):
        lo, hi = _vector(pair, 2, "joint limit")
        _require(nominal[0]*support.RAD_PER_RAW <= lo < hi <= nominal[1]*support.RAD_PER_RAW,
                 "Joint limits must stay within strict manufacturer nominal bounds")
    return value


def _pose(fk, joints):
    return _vector(list(fk(list(joints))), 6, "FK pose")


def _rotation(pose):
    # as_rotvec/as_euler work in both system SciPy 1.3 and current SciPy;
    # no dependency on the renamed as_dcm/as_matrix interface.
    return Rotation.from_euler("xyz", pose[3:])


def solve_local_delta(raw_q, measured_pose_m_rad, delta_xyz_m, delta_rotvec_rad,
                      limits, *, max_joint_step_deg=2., fk=None):
    """Return a JSON-safe proposal or ok=False; never shrink/retry/send a target.

The absolute endpoint limits are 0.5 mm and 0.005 rad. For every nonzero
translation/rotation request the predicted physical change, relative to start
FK, must also project at least 50% onto that requested direction. This prevents
an absolute tolerance from accepting an essentially unchanged endpoint.

The returned endpoint is an FK prediction, not measured motion. A successful
result does not verify collision clearance, feedback age or dispatch authority.
"""
    result = {"ok": False, "status": "rejected", "stage": "input", "solver_calls": 0,
              "hardware_commands_sent": 0, "motion_authorized": False,
              "collision_verified": False, "fresh_feedback_verified": False,
              "frame": "arm_base", "reference": "flange", "raw_joint_unit": "0.001 degree",
              "pose_units": "m/rad", "delta_rotation": "base-frame rotation vector, left composition",
              "endpoint_source": "FK prediction of quantized joints, not measured motion",
              "position_tolerance_m": POSITION_TOLERANCE_M,
              "rotation_tolerance_rad": ROTATION_TOLERANCE_RAD,
              "minimum_direction_progress_fraction": MIN_REQUESTED_DIRECTION_PROGRESS,
              "fk_backend": "provided_fk" if fk is not None else "manufacturer_piper_fk_offset_1"}
    try:
        _require(isinstance(raw_q, (list, tuple)) and len(raw_q) == 6 and
                 all(type(v) is int for v in raw_q), "raw_q must be six integer SDK millidegrees")
        _require(all(lo <= v <= hi for v, (lo, hi) in zip(raw_q, support.JOINT_LIMITS_RAW)),
                 "Current joints are outside strict manufacturer nominal limits")
        _require(type(max_joint_step_deg) in (int, float) and math.isfinite(max_joint_step_deg)
                 and 0 < max_joint_step_deg <= MAX_JOINT_STEP_DEG, "Local joint step must be within (0,2] degrees")
        measured = _vector(measured_pose_m_rad, 6, "measured_pose_m_rad")
        delta_xyz = _vector(delta_xyz_m, 3, "delta_xyz_m")
        delta_rotation = _vector(delta_rotvec_rad, 3, "delta_rotvec_rad")
        checked_limits = _checked_limits(limits)
        distance, angle = math.hypot(*delta_xyz), math.hypot(*delta_rotation)
        _require(distance <= checked_limits["max_translation_step_m"] and
                 angle <= checked_limits["max_rotation_step_rad"], "Requested delta exceeds original motion limits")
        q = np.asarray(raw_q, dtype=float)*support.RAD_PER_RAW
        bounds = np.asarray(checked_limits["joint_limits_rad"], dtype=float)
        _require(np.all(q >= bounds[:, 0]) and np.all(q <= bounds[:, 1]), "Current joints outside site limits")
        _require(np.all(measured[:3] >= checked_limits["workspace_min_m"]) and
                 np.all(measured[:3] <= checked_limits["workspace_max_m"]), "Measured pose outside original workspace")
        fk = support.manufacturer_fk() if fk is None else fk
        result.update(start_raw_q=list(raw_q), measured_pose_m_rad=measured.tolist(),
                      delta_xyz_m=delta_xyz.tolist(), delta_rotvec_rad=delta_rotation.tolist(),
                      max_joint_step_deg=float(max_joint_step_deg),
                      caller_feedback_age_limit_s=checked_limits["max_state_age_s"])
        result["stage"] = "initial_fk"
        initial = _pose(fk, q)
        initial_rotation = _rotation(initial)
        measured_rotation = _rotation(measured)
        initial_position_error = float(np.linalg.norm(initial[:3]-measured[:3]))
        initial_rotation_error = float(np.linalg.norm((measured_rotation.inv()*initial_rotation).as_rotvec()))
        result.update(initial_fk_pose_m_rad=initial.tolist(), initial_fk_position_error_m=initial_position_error,
                      initial_fk_rotation_error_rad=initial_rotation_error)
        _require(initial_position_error <= .002 and initial_rotation_error <= .02, "Initial manufacturer FK/feedback mismatch")
        desired_rotation = Rotation.from_rotvec(delta_rotation)*measured_rotation
        desired = np.r_[measured[:3]+delta_xyz, desired_rotation.as_euler("xyz")]
        result["requested_pose_m_rad"] = desired.tolist()
        nonzero = distance > 0 or angle > 0
        if not nonzero:
            # A zero request cannot silently become a feedback-correction move.
            target_raw = list(raw_q)
        else:
            result["stage"] = "solve"
            cap_raw = math.floor(max_joint_step_deg*1000)
            low_raw = np.maximum(np.ceil(bounds[:, 0]/support.RAD_PER_RAW), np.asarray(raw_q)-cap_raw)
            high_raw = np.minimum(np.floor(bounds[:, 1]/support.RAD_PER_RAW), np.asarray(raw_q)+cap_raw)
            # Preserve the already-validated seed if floating division rounds a
            # nominal boundary a fraction of one raw unit inward. No other
            # endpoint is admitted; quantized targets are checked below again.
            low_raw = np.minimum(low_raw, raw_q)
            high_raw = np.maximum(high_raw, raw_q)
            _require(np.all(low_raw < high_raw), "No usable local quantized joint interval")
            lower, upper = low_raw*support.RAD_PER_RAW, high_raw*support.RAD_PER_RAW
            def residual(joints):
                pose = _pose(fk, joints)
                return np.r_[(pose[:3]-desired[:3])/POSITION_TOLERANCE_M,
                             (desired_rotation.inv()*_rotation(pose)).as_rotvec()/ROTATION_TOLERANCE_RAD]
            result["solver_calls"] = 1
            solution = least_squares(residual, q, bounds=(lower, upper), max_nfev=200,
                                     xtol=1e-11, ftol=1e-11, gtol=1e-11)
            result["solver_success"] = bool(solution.success)
            result["solver_evaluations"] = int(solution.nfev)
            _require(solution.success and np.all(np.isfinite(solution.x)), "Local IK did not converge")
            target_raw = np.rint(solution.x/support.RAD_PER_RAW).astype(int).tolist()
        result["stage"] = "quantized_endpoint"
        target = np.asarray(target_raw, dtype=float)*support.RAD_PER_RAW
        result["candidate_raw_q"] = target_raw
        _require(all(lo <= v <= hi for v, (lo, hi) in zip(target_raw, support.JOINT_LIMITS_RAW)) and
                 np.all(target >= bounds[:, 0]) and np.all(target <= bounds[:, 1]),
                 "Quantized target outside nominal/site joint limits")
        _require(max(abs(a-b) for a, b in zip(target_raw, raw_q)) <= max_joint_step_deg*1000,
                 "Quantized target exceeds local joint step")
        endpoint = _pose(fk, target)
        endpoint_rotation = _rotation(endpoint)
        position_error = float(np.linalg.norm(endpoint[:3]-desired[:3]))
        rotation_error = float(np.linalg.norm((desired_rotation.inv()*endpoint_rotation).as_rotvec()))
        translation_change = endpoint[:3]-initial[:3]
        rotation_change = (endpoint_rotation*initial_rotation.inv()).as_rotvec()
        translation_progress = float(np.dot(translation_change, delta_xyz/distance)/distance) if distance else None
        rotation_progress = float(np.dot(rotation_change, delta_rotation/angle)/angle) if angle else None
        _require(all(value is None or math.isfinite(value) for value in (translation_progress, rotation_progress)),
                 "Request is below numerically resolvable endpoint progress")
        result.update(endpoint_pose_m_rad=endpoint.tolist(), endpoint_position_error_m=position_error,
                      endpoint_rotation_error_rad=rotation_error,
                      predicted_translation_m=translation_change.tolist(),
                      predicted_rotation_vector_rad=rotation_change.tolist(),
                      translation_progress_fraction=translation_progress, rotation_progress_fraction=rotation_progress)
        _require(position_error <= POSITION_TOLERANCE_M and rotation_error <= ROTATION_TOLERANCE_RAD,
                 "Quantized endpoint residual exceeds tolerance")
        if nonzero:
            _require(target_raw != list(raw_q) and
                     all(value is None or value >= MIN_REQUESTED_DIRECTION_PROGRESS
                         for value in (translation_progress, rotation_progress)),
                     "Nonzero request produced insufficient physical endpoint progress")
        result["stage"] = "original_path_check"
        before = {"raw_q": list(raw_q), "q": q.tolist(), "pose": measured.tolist()}
        path = support.path_check(before, target_raw, checked_limits, fk)
        result.update(ok=True, status="proposal" if nonzero else "zero_delta_no_command", stage="complete",
                      target_raw=target_raw, target_joints_rad=target.tolist(), path_check=path,
                      should_send=False if not nonzero else None,
                      execution_requirement="Caller must reacquire fresh state and pass the unchanged dispatch guards")
    except Exception as exc:
        result.update(error_type=type(exc).__name__, error=str(exc))
    return result
