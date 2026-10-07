"""Pure classification of an already-sent bounded jaw closing probe.

This module does not dispatch, admit, stop or retry hardware. Its input is the
adapter's measured trace, not caller assertions of contact or support. Width
shortfall can suggest contact; it cannot identify an object or prove acceptance.
"""
import math

from . import arms
from .single_supervised_actions import BOUNDS
from .takeover import LIMITS, SIDES, _allowed_integer


MAX_PROBE_CLOSURE_M = 0.005  # Narrow probe request bound; not a contact-force certification.


def probe_closure_within_bound(current_width_m, target_width_m):
    """Strict closing request, with only floating-point endpoint equality.

    The 1e-12 m equality allowance is a millionth of the SDK's 1 um encoding
    increment, not an observation tolerance or a larger physical probe limit.
    """
    if any(type(value) not in (int, float) or not math.isfinite(value)
           for value in (current_width_m, target_width_m)):
        return False
    closure = current_width_m-target_width_m
    return closure > 0 and (closure <= MAX_PROBE_CLOSURE_M
                           or math.isclose(closure, MAX_PROBE_CLOSURE_M, rel_tol=0, abs_tol=1e-12))


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + " must be a finite number, not a boolean")
    return value


def _quat(pose):
    # Same Rz(yaw) Ry(pitch) Rx(roll) convention as the bounded adapter.
    cr, cp, cy = (math.cos(value / 2) for value in pose[3:])
    sr, sp, sy = (math.sin(value / 2) for value in pose[3:])
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def _rotation(a, b):
    qa, qb = _quat(a), _quat(b)
    norm = math.sqrt(sum(x*x for x in qa) * sum(x*x for x in qb))
    return 2 * math.acos(min(1.0, abs(sum(x*y for x, y in zip(qa, qb))) / norm))


def _rotation_span(poses):
    # Normalize once, remove identical orientations, then find the minimum
    # absolute dot product. This preserves the pairwise SO(3) diameter without
    # repeating trig/acos for every pair in a 100 Hz three-second trace.
    quaternions = set()
    for pose in poses:
        q = _quat(pose)
        norm = math.sqrt(sum(x*x for x in q))
        quaternions.add(tuple(x/norm for x in q))
    values, minimum = list(quaternions), 1.0
    for index, (a, b, c, d) in enumerate(values):
        for e, f, g, h in values[index+1:]:
            minimum = min(minimum, abs(a*e+b*f+c*g+d*h))
    return 2*math.acos(min(1.0, minimum))


def _check_trace(samples, label):
    if not isinstance(samples, list) or len(samples) < 2:
        raise ValueError(label + " needs at least two measured samples")
    previous = None
    advance_reference = None
    advances = 0
    for sample in samples:
        if not isinstance(sample, dict) or set(sample) != {"observed_at_s", "arms"}:
            raise ValueError(label + " sample must contain observed_at_s and arms")
        at = _number(sample["observed_at_s"], label + " observed_at_s")
        states = sample["arms"]
        if not isinstance(states, dict) or set(states) != set(SIDES):
            raise ValueError(label + " requires both arm snapshots")
        stamps = []
        for side in SIDES:
            state = states[side]
            if not isinstance(state, dict) or state.get("status") != "complete":
                raise ValueError(label + " requires complete measured snapshots")
            health = arms.control_health(state, now_s=at, allowed_control_modes=(1,), require_enabled=True)
            if not health["healthy"]:
                raise ValueError(label + " unhealthy " + side + ": " + repr(health["reasons"]))
            status = state["arm_status"]
            if (not _allowed_integer(status.get("motion_status"), (0,))
                    or not _allowed_integer(status.get("teach_status"), (0,))
                    or not _allowed_integer(status.get("mode_feedback"), (0, 1, 2))):
                raise ValueError(label + " requires stationary arms and known unchanged modes")
            snapshot_at = _number(state.get("timestamp"), label + " snapshot timestamp")
            if not 0 <= at-snapshot_at <= BOUNDS["feedback_age_s"]:
                raise ValueError(label + " snapshot timestamp is stale or future")
            jaw = state["gripper"]
            if not 0 <= jaw["width_m"] <= 0.070:
                raise ValueError(label + " jaw feedback outside existing 0..70 mm range")
            jaw_at = _number(jaw.get("timestamp"), label + " jaw timestamp")
            fragments = state["fragment_timestamps_s"]
            if jaw_at != fragments["gripper"]:
                raise ValueError(label + " jaw and fragment timestamps disagree")
            stamps.extend(_number(stamp, label + " fragment timestamp") for stamp in fragments.values())
        if any(not 0 <= at-stamp <= BOUNDS["feedback_age_s"] for stamp in stamps):
            raise ValueError(label + " feedback exceeds existing age bound")
        if max(stamps)-min(stamps) > BOUNDS["feedback_skew_s"]:
            raise ValueError(label + " feedback exceeds existing cross-arm skew bound")
        if previous is not None:
            if at <= previous["observed_at_s"]:
                raise ValueError(label + " observation times must strictly increase")
            if at-previous["observed_at_s"] > BOUNDS["feedback_age_s"]:
                raise ValueError(label + " trace has a gap beyond the feedback age bound")
            pairs = [(previous["arms"][side]["fragment_timestamps_s"][name], stamp)
                     for side in SIDES for name, stamp in states[side]["fragment_timestamps_s"].items()]
            if any(new < old for old, new in pairs):
                raise ValueError(label + " feedback timestamps regressed")
            if any(states[side]["arm_status"]["mode_feedback"] != previous["arms"][side]["arm_status"]["mode_feedback"]
                   for side in SIDES):
                raise ValueError(label + " motion mode changed during a jaw-only probe")
            if all(stamp > advance_reference["arms"][side]["fragment_timestamps_s"][name]
                   for side in SIDES for name, stamp in states[side]["fragment_timestamps_s"].items()):
                advances += 1
                advance_reference = sample
        else:
            advance_reference = sample
        previous = sample
    return advances


def _stable(samples, *, selected=None, stationary_reference=None):
    """Existing stable spans; all uncommanded components retain their anchor."""
    spans = {}
    for side in SIDES:
        states = [sample["arms"][side] for sample in samples]
        poses = [state["pose_m_rad"] for state in states]
        widths = [state["gripper"]["width_m"] for state in states]
        joint_span = max(max(values)-min(values) for values in zip(*(state["joints_rad"] for state in states)))
        position_span = math.sqrt(sum((max(values)-min(values))**2 for values in zip(*(pose[:3] for pose in poses))))
        rotation_span = _rotation_span(poses)
        spans[side] = {"joint_rad": joint_span, "position_m": position_span,
                       "rotation_rad": rotation_span, "jaw_m": max(widths)-min(widths)}
        if (joint_span > BOUNDS["joint_span_rad"] or position_span > BOUNDS["position_span_m"]
                or rotation_span > BOUNDS["rotation_span_rad"] or max(widths)-min(widths) > BOUNDS["jaw_span_m"]):
            raise ValueError(side + " exceeds existing stable-window spans")
        if stationary_reference is not None:
            origin = stationary_reference[side]
            for state in states:
                if (max(abs(a-b) for a, b in zip(state["joints_rad"], origin["joints_rad"])) > BOUNDS["joint_span_rad"]
                        or math.dist(state["pose_m_rad"][:3], origin["pose_m_rad"][:3]) > BOUNDS["position_span_m"]
                        or _rotation(state["pose_m_rad"], origin["pose_m_rad"]) > BOUNDS["rotation_span_rad"]):
                    raise ValueError(side + " moved from the pre-send stationary arm anchor")
                if side != selected and abs(state["gripper"]["width_m"]-origin["gripper"]["width_m"]) > LIMITS["gripper_m"]:
                    raise ValueError(side + " uncommanded jaw moved from the pre-send anchor")
    return spans


def classify_gripper_probe(*, arm, requested_width_m, sent_at, baseline_samples, post_samples):
    """Return target_arrived, settled_contact_candidate, or unconfirmed.

Each sample is {'observed_at_s': host_read_time, 'arms': {left: raw, right: raw}}.
Traces must cover the complete baseline and post-send observation, including any
transient motion. Numerical policies are reused software observation bounds,
not manufacturer contact thresholds, force calibration or physical proof.
"""
    result = {"outcome": "unconfirmed", "accepted": None, "arrival_confirmed": False,
              "grasp_verified": False, "contact_verified": None, "contact_support_verified": False,
              "physical_stop_verified": None, "force_calibrated": False,
              "target_cancellation_verified": False, "automatic_retry": False,
              "observed_width_m": None, "observed_force_N": None, "closure_displacement_m": None,
              "completion": "unconfirmed", "max_probe_closure_m": MAX_PROBE_CLOSURE_M,
              "reasons": [], "scope": "Measured response classification only; no hardware action or admission"}
    try:
        if arm not in SIDES:
            raise ValueError("Explicit left/right selected arm required")
        target = _number(requested_width_m, "requested_width_m")
        sent_at = _number(sent_at, "sent_at")
        if sent_at <= 0 or not 0 <= target <= 0.055:
            raise ValueError("Positive send time and existing 0..55 mm target range required")
        baseline_advances = _check_trace(baseline_samples, "baseline")
        _check_trace(post_samples, "post-send")
        if post_samples[-1]["observed_at_s"]-sent_at > BOUNDS["timeout_s"]:
            raise ValueError("Post-send trace exceeds the existing observation timeout")
        if (baseline_samples[-1]["observed_at_s"] > sent_at
                or baseline_samples[-1]["observed_at_s"] < sent_at-BOUNDS["feedback_age_s"]):
            raise ValueError("Baseline does not end freshly before this send")
        if (post_samples[0]["observed_at_s"]-sent_at > BOUNDS["feedback_age_s"]
                or any(sent_at-stamp > BOUNDS["feedback_age_s"]
                       for state in baseline_samples[-1]["arms"].values()
                       for stamp in state["fragment_timestamps_s"].values())):
            raise ValueError("Trace does not cover the send boundary with fresh feedback")
        if any(stamp <= sent_at for sample in post_samples for state in sample["arms"].values()
               for stamp in state["fragment_timestamps_s"].values()):
            raise ValueError("Every post-send feedback fragment must follow the send")
        baseline_duration = baseline_samples[-1]["observed_at_s"]-baseline_samples[0]["observed_at_s"]
        if baseline_duration < BOUNDS["stable_s"] or baseline_advances < BOUNDS["minimum_feedback_advances"]:
            raise ValueError("Baseline lacks existing three-second/20-advance evidence")
        baseline_spans = _stable(baseline_samples)
        origin = baseline_samples[0]["arms"]
        baseline_widths = [sample["arms"][arm]["gripper"]["width_m"] for sample in baseline_samples]
        if any(not probe_closure_within_bound(width, target) for width in baseline_widths):
            raise ValueError("Every baseline width requires a closing probe within 5 mm")
        for sample in post_samples:
            # Full trace checks cannot be replaced by a quiet final suffix.
            _stable([sample], selected=arm, stationary_reference=origin)
            for side in SIDES:
                if sample["arms"][side]["arm_status"]["mode_feedback"] != origin[side]["arm_status"]["mode_feedback"]:
                    raise ValueError(side + " motion mode changed during a jaw-only probe")
            width = sample["arms"][arm]["gripper"]["width_m"]
            if not target-LIMITS["gripper_m"] <= width <= max(baseline_widths)+LIMITS["gripper_m"]:
                raise ValueError("Selected jaw left the existing requested-width interval")
        cutoff = post_samples[-1]["observed_at_s"]-BOUNDS["stable_s"]
        starts = [index for index, sample in enumerate(post_samples) if sample["observed_at_s"] <= cutoff]
        if not starts:
            raise ValueError("Post-send trace lacks the full stable observation duration")
        settled = post_samples[starts[-1]:]
        settled_advances = _check_trace(settled, "settled")
        if settled_advances < BOUNDS["minimum_feedback_advances"]:
            raise ValueError("Settled trace lacks 20 complete independently advancing samples")
        settled_spans = _stable(settled, selected=arm, stationary_reference=origin)
        final_jaw = post_samples[-1]["arms"][arm]["gripper"]
        final_widths = [sample["arms"][arm]["gripper"]["width_m"] for sample in settled]
        closure = baseline_widths[-1]-final_jaw["width_m"]
        resolved_closure = min(baseline_widths)-max(final_widths)
        result.update(requested_width_m=target, observed_width_m=final_jaw["width_m"],
                      observed_force_N=final_jaw["force_N"],
                      settled_force_samples_N=[sample["arms"][arm]["gripper"]["force_N"] for sample in settled],
                      baseline_width_m=baseline_widths[-1], closure_displacement_m=closure,
                      conservative_closure_displacement_m=resolved_closure,
                      width_error_m=abs(final_jaw["width_m"]-target),
                      baseline_duration_s=baseline_duration, baseline_feedback_advances=baseline_advances,
                      settled_duration_s=settled[-1]["observed_at_s"]-settled[0]["observed_at_s"],
                      settled_feedback_advances=settled_advances,
                      baseline_spans=baseline_spans, settled_spans=settled_spans,
                      observation_window_complete=True,
                      completion="observation_only",
                      target_may_remain_active=True)
        if all(abs(width-target) <= LIMITS["gripper_m"] for width in final_widths):
            result.update(outcome="target_arrived", arrival_confirmed=True)
        elif (min(final_widths) > target+LIMITS["gripper_m"]
              and resolved_closure > BOUNDS["jaw_span_m"]):
            result.update(outcome="settled_contact_candidate", reasons=[
                "Post-send closing response settled short of target; object contact and support require new visual evidence."])
        else:
            result["reasons"].append("No distinguishable closing-and-settling response or no unambiguous width outcome")
    except (ValueError, TypeError, KeyError, AttributeError) as exc:
        result["reasons"].append(str(exc))
    return result
