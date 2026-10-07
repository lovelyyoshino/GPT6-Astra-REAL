"""Pure summaries of device-owned static retention traces, never load proof.

The real adapter collects and guards the trace. This helper neither acquires
feedback nor grants dispatch; its identity and artifact references are resolved
by the owning host, not authenticated by a dictionary or hash.
"""
import copy
import hashlib
import json
import math

from .contact_receipt import _check_trace, _rotation, _stable
from .single_supervised_actions import BOUNDS
from .takeover import LIMITS, SIDES


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def identity_checked(identity, arm):
    if (type(identity) is not dict or set(identity) !=
            {"episode_id", "arm", "run_id", "owner", "epoch", "object_id"}):
        raise ValueError("Resolved six-field grasp identity required")
    for name, value in identity.items():
        if (type(value) is not str or not 1 <= len(value) <= 128 or value.strip() != value
                or any(ord(char) < 32 or ord(char) == 127 for char in value)):
            raise ValueError("Invalid grasp identity " + name)
    if arm not in SIDES or identity["arm"] != arm:
        raise ValueError("Grasp identity must select the same explicit arm")
    return copy.deepcopy(identity)


def measured_anchor(state):
    return {"joints_rad": copy.deepcopy(state["joints_rad"]),
            "pose_m_rad": copy.deepcopy(state["pose_m_rad"]),
            "width_m": state["gripper"]["width_m"]}


def grasp_body_anchor(record):
    """Select a completed event's local anchor without rewriting probe history.

    Only the dedicated loaded adapter creates local_anchor. It never moves the
    passive body's or either closed jaw's drift reference. Release summaries
    use this explicit local body reference after confirmed transport.
    """
    local = record.get("local_anchor")
    if local is not None and record["status"] in (
            "retained_local", "loaded_pending_visual", "release_pending", "release_opened"):
        return copy.deepcopy(local)
    return copy.deepcopy(record["original_anchor"])


def anchor_deviation(anchor, state):
    return {"joint_rad": max(abs(a-b) for a, b in zip(anchor["joints_rad"], state["joints_rad"])),
            "position_m": math.dist(anchor["pose_m_rad"][:3], state["pose_m_rad"][:3]),
            "rotation_rad": _rotation(anchor["pose_m_rad"], state["pose_m_rad"]),
            "jaw_m": abs(anchor["width_m"]-state["gripper"]["width_m"])}


def summarize_retention_trace(*, arm, identity, probe_event_id, trace_id, trace_sha256,
                              original_anchor, samples, now, release=False,
                              release_jaw_anchor_m=None):
    """Summarize a new dual-arm window against its explicit fixed body anchor.

    The adapter supplies the immutable probe anchor for static grasps, or the
    recorded local anchor after a completed dedicated loaded segment. This
    helper does not create/rebase either reference or certify object retention.
    """
    if arm not in SIDES:
        raise ValueError("Explicit left/right arm required")
    if identity is not None:
        identity = identity_checked(identity, arm)
    if (type(now) not in (int, float) or not math.isfinite(now)
            or type(samples) is not list or len(samples) < 21):
        raise ValueError("Finite read time and at least 21 complete measured samples required")
    advances = _check_trace(samples, "static retention")
    if (samples[-1]["observed_at_s"]-samples[0]["observed_at_s"] < BOUNDS["stable_s"]
            or advances < BOUNDS["minimum_feedback_advances"]
            or not 0 <= now-samples[-1]["observed_at_s"] <= BOUNDS["feedback_age_s"]):
        raise ValueError("Static retention needs a current three-second/20-advance dual-arm trace")
    spans = _stable(samples)
    deviations = [anchor_deviation(original_anchor, sample["arms"][arm]) for sample in samples]
    maximum = {name: max(item[name] for item in deviations) for name in deviations[0]}
    limits = {"joint_rad": BOUNDS["joint_span_rad"], "position_m": BOUNDS["position_span_m"],
              "rotation_rad": BOUNDS["rotation_span_rad"], "jaw_m": BOUNDS["jaw_span_m"]}
    if any(maximum[name] > value for name, value in limits.items()
           if not (release and name == "jaw_m")):
        raise ValueError("Static retention drifted from its original candidate anchor")
    if release_jaw_anchor_m is not None:
        if (not release or type(release_jaw_anchor_m) not in (int, float)
                or not math.isfinite(release_jaw_anchor_m) or not 0 <= release_jaw_anchor_m <= .070):
            raise ValueError("A finite measured jaw anchor is specific to release observation")
        if any(abs(sample["arms"][arm]["gripper"]["width_m"]-release_jaw_anchor_m)
               > BOUNDS["jaw_span_m"] for sample in samples):
            raise ValueError("Released jaw drifted from its last measured opening anchor")
    result = {"trace_id": trace_id, "trace_sha256": trace_sha256,
            "started_at": samples[0]["observed_at_s"],
            "ended_at": samples[-1]["observed_at_s"], "sample_count": len(samples),
            "feedback_advances": advances, "health": "healthy", "mode": "stationary",
            "anchor": copy.deepcopy(original_anchor), "observed": measured_anchor(samples[-1]["arms"][arm]),
            "spans": spans[arm], "anchor_deviation": maximum}
    if identity is not None:
        result["identity"] = identity
    if probe_event_id is not None:
        result["probe_event_id"] = probe_event_id
    return result


def summarize_release_confirmation(*, release_opening, **kwargs):
    """New feedback after one exact opening, not a visual object-release claim."""
    if type(release_opening) is not dict:
        raise ValueError("The adapter's latest measured opening is required")
    sha = release_opening.get("trace_sha256")
    if (type(sha) is not str or len(sha) != 64
            or any(char not in "0123456789abcdef" for char in sha)):
        raise ValueError("Exact opening trace hash required")
    ended = release_opening.get("finished_at")
    width, target = (release_opening.get(name) for name in ("observed_width_m", "target_width_m"))
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in (ended, width, target)):
        raise ValueError("Finite measured opening and target required")
    if not 0 <= target <= .055 or not 0 <= width <= .070 or abs(width-target) > LIMITS["gripper_m"]:
        raise ValueError("Last opening must retain its measured arrival")
    result = summarize_retention_trace(**kwargs, release=True, release_jaw_anchor_m=width)
    if (result["started_at"] <= ended
            or any(stamp <= ended for sample in kwargs["samples"] for state in sample["arms"].values()
                   for stamp in state["fragment_timestamps_s"].values())):
        raise ValueError("Release confirmation needs new dual-arm feedback after the last opening")
    if any(abs(sample["arms"][kwargs["arm"]]["gripper"]["width_m"]-target) > LIMITS["gripper_m"]
           for sample in kwargs["samples"]):
        raise ValueError("Released jaw left its measured target arrival")
    return result
