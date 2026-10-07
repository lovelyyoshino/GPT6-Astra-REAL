"""Measured robot response, separate from target arrival and object progress.

Pure arithmetic over host-selected feedback in one already reconciled frame.
This module does not authenticate samples, qualify a trajectory, infer force,
or turn a robot displacement into visual/object evidence. Its observation bands
reuse the existing stationary policy; they are not sensor accuracy guarantees.
"""
import math

from .single_supervised_actions import BOUNDS


def _pose(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 6
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ValueError("Six finite measured or requested pose values required")
    return [float(x) for x in value]


def _quaternion(pose):
    r, p, y = (x / 2 for x in pose[3:])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)


def _rotation_vector(before, after):
    # after * conjugate(before): shortest rotation, expressed in the base frame.
    w, x, y, z = _quaternion(after)
    a, b, c, d = _quaternion(before)
    relative = (w*a+x*b+y*c+z*d, -w*b+x*a-y*d+z*c,
                -w*c+x*d+y*a-z*b, -w*d-x*c+y*b+z*a)
    if relative[0] < 0:
        relative = tuple(-v for v in relative)
    norm = math.sqrt(sum(v*v for v in relative[1:]))
    if norm < 1e-15:
        return [0.0, 0.0, 0.0]
    angle = 2 * math.atan2(norm, max(0.0, relative[0]))
    return [angle*v/norm for v in relative[1:]]


def _norm(vector):
    length = math.hypot(*vector)
    if not math.isfinite(length):
        raise ValueError("Computed displacement must remain finite")
    return length


def _component(requested, measured, band):
    length, actual = _norm(requested), _norm(measured)
    # Compare two observations conservatively. A stable band is a policy,
    # not a calibrated error distribution or a confidence interval.
    distinguishable = 2 * band
    direction = [x/length for x in requested] if length > 0 else None
    axial = sum(a*b for a, b in zip(direction, measured)) if direction is not None else None
    transverse = _norm([b-axial*a for a, b in zip(direction, measured)]) if direction is not None else actual
    if length <= distinguishable:
        response = "below_observation_resolution" if actual <= distinguishable else "unrequested_response"
    elif transverse > distinguishable:
        response = "transverse_response_observed"
    elif axial > distinguishable:
        response = "requested_direction_observed"
    elif axial < -distinguishable:
        response = "opposite_direction_observed"
    else:
        response = "no_discriminable_response"
    return {"requested_vector": requested, "measured_vector": measured,
            "requested_norm": length, "measured_norm": actual,
            "along_requested_direction": axial, "transverse_norm": transverse,
            "transverse_exceeds_comparison_band": transverse > distinguishable,
            "comparison_band": distinguishable, "response": response}


def measure_motion_effect(before_pose, requested_pose, after_pose):
    """Return auditable endpoint deltas; never a physical/task success verdict.

The caller must bind the before/after samples, raw timestamps, frame identity
and dispatch event. Even a correctly directed response does not establish
contact, full arrival, a safe path or a successful extraction/insertion.
"""
    before, requested, after = map(_pose, (before_pose, requested_pose, after_pose))
    translation = _component([b-a for a, b in zip(before[:3], requested[:3])],
                             [b-a for a, b in zip(before[:3], after[:3])],
                             BOUNDS["position_span_m"])
    rotation = _component(_rotation_vector(before, requested),
                         _rotation_vector(before, after), BOUNDS["rotation_span_rad"])
    return {"translation": translation, "rotation": rotation,
            "units": {"translation": "m", "rotation": "rad"},
            "target_error": {"position_m": _norm([a-b for a, b in zip(requested[:3], after[:3])]),
                             "rotation_rad": _norm(_rotation_vector(requested, after))},
            "observation_basis": "Existing stationary-policy bands, not calibrated sensor accuracy",
            "object_progress_measurement": None, "visual_object_effect": None,
            "load_response": None, "task_success": None,
            "scope": "Robot endpoint response only; no contact or object progress certification"}
