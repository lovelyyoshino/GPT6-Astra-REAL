"""Pure PiPER X six-axis joint candidate checking, with no transport or IK.

``plan_joint_path(context, target_joints_rad, *, now, recovery_mode=None)``
returns a hash-bound JSON plan or raises ``JointPathError(code, detail)``.
``validate_joint_path_sample(plan, sample, *, now, phase='active')`` checks
fresh feedback against that SAME original anchor; it cannot renew a plan.

Context schema ``piper_x_joint_path_context_v1``:
* identity: the eight JointHoldTransaction identity fields, model=piper_x;
* model_catalog: model_compatibility's pinned constants source specification;
* urdf_source: path, commit, source_url, sha256 (pinned base X URDF);
* origin/current: {sample_id, identity, captured_at, arms:{left,right}}, using
  the existing arms.snapshot schema; origin_sha256 is the host's commitment;
* cached_target: {event_id, identity, target_raw, frame_receipts}; four exact
  returned frames from THIS owner/epoch/connection, retaining the previous
  worker identity rather than relabelling it as this action's worker;
* controller_limits: {left:[[lo,hi],...], right:[...], source:{ref,sha256}};
* geometry: {attachment_radius_m:{left,right}, available_clearance_m,
  workspace_min_m, workspace_max_m, origin_sample_id, source:{ref,sha256}};
  workspace is explicitly in the physical model's arm-base coordinates;
  alternatively, explicit piper_rgb_supervised_joint_path_v1 binds current
  images, operation=approach/align and the encoded target; no metric bounds are
  inferred. That separate ordinary RGB contract supports latch-only cancel,
  keeps the same numeric hold reserve, and uses its own bounded post-send
  observation policy; it does not inherit startup's .1-radian origin allowance;
  explicit piper_rgb_supervised_coarse_approach_v1 additionally binds the
  current far-from-target observation, with only unloaded approach allowed.
  Its separate fixed profile permits 3 degrees per joint, 20 mm requested and
  encoded endpoints, and a 35 mm / .08 rad independent box and RX envelope.
  Only this latch-only profile has no extra hold reserve. Its complete strict
  box, real cache and partial updates remain checked. The host must establish
  both jaws have no episode; a text observation is not an execution permit;
  piper_rgb_supervised_loaded_joint_path_v1 instead binds loaded_observation
  and both host-resolved episode references for a right-arm extract_segment,
  transport or insert_segment. Extract/insert requested AND encoded endpoints
  are limited to 2 mm / .01 rad from origin and current model poses. These are
  software target limits, not force, object progress or grip qualification;
* budget: {max_translation_m, max_rotation_rad}, bounded by 20 mm / .05 rad
  for the existing profiles; the coarse tag requires exactly 35 mm / .08 rad;
* unloaded_evidence: null, or {origin_sample_id, source:{ref,sha256}}.
* initialization_sources: optional live, host-resolved first-target records for
  each side; only completed same-connection boundary targets can account for
  J2/J3 initialization feedback residuals, never arbitrary out-of-limit poses.

All provenance, the original commitment, and controller/geometry evidence are
HOST-RESOLVED, not user-supplied permissions. This module verifies the official
model bytes but does not authenticate a caller's scene or control history.
The adapter must establish the three-second dual-arm baseline, the operation's
actual empty/retained state, mapping applicability, durable once-only
reservation, and interpose every TX.
This module neither proves atomic controller updates nor solves unknown cache,
collision, physical hold/stop, grasp, or loaded contact. Ordinary MOVE_L stays
unchanged. Plans preserve raw controller telemetry separately from derived FK.
"""
import copy
from functools import wraps
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from . import arms
from .bounded_joint_step import STEP
from .joint_recovery import RECOVERY
from .startup_recovery import STARTUP_RECOVERY
from .hold_transaction import joint_hold_frames, RAD_PER_RAW, _frame_matches
from .model_compatibility import load_model_catalog, fk_matrix, pose_matrix, matrix_error
from .rgb_supervision import (JOINT_PATH_SCHEMA as VISUAL_GEOMETRY_SCHEMA,
                             LOADED_JOINT_PATH_SCHEMA, COARSE_JOINT_PATH_SCHEMA,
                             validate_rgb_geometry)


SCHEMA = "piper_x_joint_path_context_v1"
PLAN_SCHEMA = "piper_x_joint_path_plan_v1"
RECOVERY_MODE = "unloaded_startup_j2_j3"
SDK_COMMIT = "841a625f5f4920e776f20b934eb13048b747e6d0"
URDF_COMMIT = "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
URDF_SHA256 = "34126caac7d5b37bc2409f337ac246afbe0bb8cd47fc9f16df5038f19bd21e3a"
IDENTITY_KEYS = {"run_id", "owner", "epoch", "worker_id", "arm", "connection_id", "model", "firmware_profile"}
SIDES = ("left", "right")
MONITOR_RAD = STEP["monitor_tolerance_rad"]
POSITION_STILL_M = STEP["stable_span_m"]
ROTATION_STILL_RAD = .003
MAX_ROTATION_RAD = .05

# An explicit, separately tagged empty-arm RGB approach contract. These are
# software bounds, not manufacturer accuracy, force or collision certification.
# Ordinary, metric, initialization and loaded contracts retain their bounds.
COARSE_PROFILE_LIMITS = {
    "max_joint_change_rad": math.radians(3), "target_translation_m": .020,
    "process_translation_m": .035, "process_rotation_rad": .08,
    "target_margin_rad": STEP["joint_margin_rad"], "monitor_band_rad": MONITOR_RAD,
    "speed_percent": 1, "hold_reserve_applied": False,
}


def _geometry_kind(geometry):
    """An unknown tagged schema can never fall back to metric admission."""
    _need(type(geometry) is dict, "geometry_schema")
    tag = geometry.get("schema")
    if "schema" in geometry:
        _need(tag in (VISUAL_GEOMETRY_SCHEMA, LOADED_JOINT_PATH_SCHEMA,
                      COARSE_JOINT_PATH_SCHEMA), "geometry_schema")
    return tag in (VISUAL_GEOMETRY_SCHEMA, LOADED_JOINT_PATH_SCHEMA,
                   COARSE_JOINT_PATH_SCHEMA), tag == COARSE_JOINT_PATH_SCHEMA


def _tracking_policy(visual, coarse):
    step = COARSE_PROFILE_LIMITS["max_joint_change_rad"] if coarse else STEP["joint_change_rad"]
    return {"mode": "bounded_postsend_settling" if visual else "strict",
        "transient_band_rad": STEP["joint_change_rad"] if visual else MONITOR_RAD,
        "max_cumulative_outside_band_s": 1.0 if visual else 0.,
        "max_origin_excursion_rad": step + MONITOR_RAD,
        "settle_tolerance_rad": MONITOR_RAD}


def _coarse_step_within(start, target):
    """Closed three-degree interval, exact on the SDK millidegree grid.

    Canonical integer feedback/targets can differ by an arithmetic ULP when
    radians are subtracted or added. Only exact roundtrips use integer units;
    an off-grid request is never rounded into the allowed range.
    """
    a, b = round(math.degrees(start)*1000), round(math.degrees(target)*1000)
    if math.radians(a/1000) == start and math.radians(b/1000) == target:
        return abs(b-a) <= 3000
    cap = COARSE_PROFILE_LIMITS["max_joint_change_rad"]
    return start-cap <= target <= start+cap


class JointPathError(ValueError):
    def __init__(self, code, detail=None):
        self.code, self.detail = code, detail or code
        super().__init__(self.detail)


def _input_errors(function):
    @wraps(function)
    def call(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except JointPathError:
            raise
        except (KeyError, TypeError, IndexError, AttributeError, OverflowError) as exc:
            raise JointPathError("invalid_input_schema", str(exc)) from exc
    return call


def _need(condition, code, detail=None):
    if not condition:
        raise JointPathError(code, detail)


def _num(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    _need(valid, "invalid_number", name)
    return float(value)


def _vec(value, count, name):
    _need(type(value) is list and len(value) == count, "invalid_vector", name)
    return [_num(item, name) for item in value]


def _text(value, name):
    _need(type(value) is str and 0 < len(value) <= 256 and value == value.strip()
          and all(ord(c) >= 32 for c in value), "invalid_identifier", name)
    return value


def _hash(value):
    _need(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value), "invalid_sha256")
    return value


def evidence_sha256(value):
    """Canonical JSON digest, shared with host origin/plan commitments."""
    try:
        return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()).hexdigest()
    except (TypeError, ValueError, OverflowError) as exc:
        raise JointPathError("non_json_evidence", str(exc)) from exc


def _source(value):
    _need(type(value) is dict and set(value) == {"ref", "sha256"}, "missing_evidence_source")
    # Evidence references can be absolute paths beneath a run/epoch directory;
    # they are not the short identity fields validated by _text.
    ref = value["ref"]
    _need(type(ref) is str and 0 < len(ref) <= 4096 and ref == ref.strip()
          and all(ord(c) >= 32 and not 127 <= ord(c) <= 159 for c in ref),
          "invalid_identifier", "evidence ref")
    _hash(value["sha256"])
    return copy.deepcopy(value)


def _identity(value):
    _need(type(value) is dict and set(value) == IDENTITY_KEYS, "identity_schema")
    for name, item in value.items():
        _text(item, name)
    _need(value["arm"] in SIDES and value["model"] == "piper_x", "physical_model_required")
    # Match the reviewed hold/frame route; newer firmware needs its own route.
    _need(value["firmware_profile"] == "default", "unsupported_firmware_profile")
    return copy.deepcopy(value)


def _enum(value, allowed):
    return isinstance(value, int) and not isinstance(value, bool) and value in allowed


def encode_joint_target(target):
    """Exact existing SDK millidegree encoding, including tie inconsistency refusal."""
    q = _vec(target, 6, "joint target radians")
    _need(all(abs(v) <= (2**31)*RAD_PER_RAW for v in q), "joint_encoding_overflow")
    raw = [round(v * 180 / math.pi * 1000) for v in q]
    _need(raw == [round(v * (180 / math.pi) * 1000) for v in q]
          == [round(math.degrees(v) * 1000) for v in q], "ambiguous_joint_quantization")
    _need(all(-(2**31) <= v < 2**31 for v in raw), "joint_encoding_overflow")
    return raw, [math.radians(v / 1000) for v in raw]


def _load_model(catalog, urdf):
    _need(type(catalog) is dict and catalog.get("commit") == SDK_COMMIT, "unpinned_sdk_model")
    try:
        models, source = load_model_catalog(catalog)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        raise JointPathError("model_source_invalid", str(exc)) from exc
    _need(source["recognized_official_snapshot"], "unrecognized_sdk_model")
    url = "https://raw.githubusercontent.com/agilexrobotics/agx_arm_urdf/" + URDF_COMMIT + "/piper_x/urdf/piper_x_description.urdf"
    _need(type(urdf) is dict and set(urdf) == {"path", "commit", "source_url", "sha256"}
          and urdf["commit"] == URDF_COMMIT and urdf["source_url"] == url
          and urdf["sha256"] == URDF_SHA256, "unpinned_urdf_model")
    try:
        raw = Path(urdf["path"]).read_bytes()
        _need(len(raw) <= 1024 * 1024 and hashlib.sha256(raw).hexdigest() == URDF_SHA256,
              "urdf_hash_mismatch")
        root = ET.fromstring(raw)
        limits = []
        for index in range(1, 7):
            joint = root.find("joint[@name='joint%d']" % index)
            limit = joint.find("limit")
            limits.append([float(limit.attrib["lower"]), float(limit.attrib["upper"])])
    except (OSError, KeyError, AttributeError, ET.ParseError, TypeError, ValueError) as exc:
        if isinstance(exc, JointPathError):
            raise
        raise JointPathError("urdf_source_invalid", str(exc)) from exc
    return models["piper_x"], limits, {"sdk": source, "urdf": copy.deepcopy(urdf)}


def _pair(sample, identity, now, *, jaw_enabled):
    _need(type(sample) is dict and {"sample_id", "identity", "captured_at", "arms"} <= set(sample), "sample_schema")
    _text(sample["sample_id"], "sample_id")
    _need(sample["identity"] == identity, "sample_identity_changed")
    captured = _num(sample["captured_at"], "capture time")
    _need(0 <= now - captured <= RECOVERY["feedback_age_s"], "stale_sample")
    states = sample["arms"]
    _need(type(states) is dict and set(states) == set(SIDES), "pair_feedback_required")
    stamps = []
    for side, state in states.items():
        _need(type(state) is dict and state.get("status") == "complete", "incomplete_feedback", side)
        _need(arms.control_health(state, now_s=now, require_enabled=False)["healthy"], "unhealthy_feedback", side)
        status = state["arm_status"]
        _need(_enum(status.get("teach_status"), (0,)) and _enum(status.get("motion_status"), (0, 1)), "unexpected_status", side)
        _need(_enum(status.get("mode_feedback"), (1,) if side == identity["arm"] else (0, 1, 2)), "same_move_j_mode_required", side)
        _need(all(state["drivers"][str(i)]["foc_status"]["driver_enable_status"] is True for i in range(1, 7)), "joint_disabled", side)
        enabled = state["gripper"]["foc_status"]["driver_enable_status"]
        _need(type(enabled) is bool and (not jaw_enabled or enabled), "jaw_enable_unknown_or_disabled", side)
        _need(0 <= state["gripper"]["width_m"] <= .070, "jaw_width_limit", side)
        for part in arms.PARTS + arms.DRIVERS + ("gripper",):
            stamp = _num(state["fragment_timestamps_s"][part], "fragment timestamp")
            _need(0 <= now - stamp <= RECOVERY["feedback_age_s"] and stamp <= captured, "stale_feedback", side + "/" + part)
            stamps.append(stamp)
    _need(max(stamps) - min(stamps) <= RECOVERY["feedback_age_s"], "pair_feedback_skew")
    return states, stamps


def _limits(model, urdf, evidence):
    _need(type(evidence) is dict and set(evidence) == {"left", "right", "source"}, "controller_limits_required")
    _source(evidence["source"])
    result, raw = {}, {}
    for side in SIDES:
        _need(type(evidence[side]) is list and len(evidence[side]) == 6, "controller_limits_incomplete")
        result[side], raw[side] = [], []
        for i, pair in enumerate(evidence[side]):
            low, high = _vec(pair, 2, "controller joint limits")
            _need(low < high, "invalid_joint_limits")
            low = max(low, urdf[i][0], model["joint_limits_rad"][i][0])
            high = min(high, urdf[i][1], model["joint_limits_rad"][i][1])
            lo, hi = math.ceil(low / RAD_PER_RAW), math.floor(high / RAD_PER_RAW)
            _need(low < high and lo < hi, "empty_limit_intersection")
            result[side].append([low, high])
            raw[side].append([lo, hi])
    return result, raw


def _within(q, limits):
    return all(low <= value <= high for value, (low, high) in zip(q, limits))


def _radii(mdh, attachment):
    # The same modified-DH remaining-chain bound as joint_envelope.tracking_sweep.
    return [abs(row[0]) + sum(abs(link[0]) + abs(link[1]) for link in mdh[i+1:])
            + attachment + STEP["link_body_allowance_m"] for i, row in enumerate(mdh)]


def _error(a, b):
    result = matrix_error(a, b)
    return result["position_error_m"], result["so3_error_rad"]


def _check_origin(q, limits, recovery, side):
    for i, (v, (lo, hi)) in enumerate(zip(q, limits)):
        if lo <= v <= hi:
            continue
        allowed = recovery and ((i == 1 and v < lo) or (i == 2 and v > hi))
        _need(allowed and min(abs(v-lo), abs(v-hi)) <= STARTUP_RECOVERY["joint_excursion_rad"],
              "origin_joint_limit", side + "/joint%d" % (i+1))


def _cached(cache, identity, origin, limits):
    _need(type(cache) is dict and set(cache) == {"event_id", "identity", "target_raw", "frame_receipts"}, "known_cached_target_required")
    _text(cache["event_id"], "cached event")
    previous_identity = _identity(cache["identity"])
    _need(all(previous_identity[name] == identity[name] for name in IDENTITY_KEYS - {"worker_id"}),
          "cached_target_identity_changed")
    raw = cache["target_raw"]
    _need(type(raw) is list and len(raw) == 6 and all(type(v) is int and -(2**31) <= v < 2**31 for v in raw), "invalid_cached_target")
    expected = joint_hold_frames(raw)
    receipts = cache["frame_receipts"]
    _need(type(receipts) is list and len(receipts) == 4, "cached_target_partial_or_unknown")
    last = 0.
    for receipt, frame in zip(receipts, expected):
        _need(type(receipt) is dict and set(receipt) == {"frame", "outcome", "returned_at"}, "cached_receipt_schema")
        at = _num(receipt["returned_at"], "cached send time")
        _need(receipt["outcome"] == "returned" and _frame_matches(receipt["frame"], frame)
              and at > 0 and at >= last, "cached_target_partial_or_unknown")
        last = at
    _need(last < min(origin["arms"][s]["fragment_timestamps_s"][p] for s in SIDES for p in arms.PARTS + arms.DRIVERS + ("gripper",)),
          "origin_precedes_cached_send")
    q = [math.radians(v/1000) for v in raw]
    _need(_within(q, limits), "cached_target_limit")
    return q


@_input_errors
def plan_joint_path(context, target_joints_rad, *, now, recovery_mode=None):
    """Create one candidate; source qualification and actual sending stay external."""
    now = _num(now, "now")
    _need(type(context) is dict and context.get("schema") == SCHEMA, "context_schema")
    _need(recovery_mode in (None, RECOVERY_MODE), "unsupported_recovery_mode")
    recovery = recovery_mode is not None
    geom = copy.deepcopy(context["geometry"])
    visual, coarse = _geometry_kind(geom)
    if coarse:
        _need(not recovery, "visual_joint_recovery_unsupported")
    identity = _identity(context["identity"])
    arm, peer = identity["arm"], next(s for s in SIDES if s != identity["arm"])
    origin, current = copy.deepcopy(context["origin"]), copy.deepcopy(context["current"])
    _need(evidence_sha256(origin) == _hash(context["origin_sha256"]), "original_anchor_changed")
    _pair(origin, identity, _num(origin["captured_at"], "origin time"), jaw_enabled=not recovery)
    states, _ = _pair(current, identity, now, jaw_enabled=not recovery)
    _need(current["captured_at"] >= origin["captured_at"], "current_precedes_origin")
    _need(all(states[s]["arm_status"]["motion_status"] == 0 for s in SIDES), "stationary_baseline_required")
    model, urdf, sources = _load_model(context["model_catalog"], context["urdf_source"])
    limits, raw_limits = _limits(model, urdf, context["controller_limits"])
    ingress = None
    if not recovery and not coarse:
        from .joint_ingress import resolve_initialization_ingress
        ingress = resolve_initialization_ingress(context.get("initialization_sources"),
            identity=identity, origin=origin, current=current,
            effective_joint_limits_rad=limits, cached_target=context["cached_target"])
    elif recovery:
        _need(not context.get("initialization_sources"), "ingress_recovery_contracts_cannot_mix")
    for side in SIDES:
        origin_limits = ingress["feedback_limits_rad"][side] if ingress else limits[side]
        _check_origin(origin["arms"][side]["joints_rad"], origin_limits, recovery, side)
        if coarse:
            _need(_within(states[side]["joints_rad"], limits[side]), "coarse_current_joint_limit", side)
    q0, q = origin["arms"][arm]["joints_rad"], states[arm]["joints_rad"]
    requested = _vec(target_joints_rad, 6, "target radians")
    raw, encoded = encode_joint_target(requested)
    _need(_within(requested, limits[arm]) and _within(encoded, limits[arm])
          and all(lo <= v <= hi for v, (lo, hi) in zip(raw, raw_limits[arm])), "target_joint_limit")
    _need(raw != encode_joint_target(q)[0], "zero_joint_action")
    if recovery:
        proof = context.get("unloaded_evidence")
        _need(type(proof) is dict and set(proof) == {"origin_sample_id", "source"}
              and proof["origin_sample_id"] == origin["sample_id"], "explicit_unloaded_evidence_required")
        _source(proof["source"])
        _need(not _within(q0, limits[arm]) and not _within(q, limits[arm]), "recovery_violation_required")
    for i, (start, present, wanted, sent, (lo, hi)) in enumerate(zip(q0, q, requested, encoded, limits[arm])):
        if recovery and not lo <= start <= hi:
            boundary = lo if start < lo else hi
            _need(wanted == boundary and sent == boundary, "recovery_requires_exact_nearest_boundary")
            _need(max(abs(wanted-start), abs(wanted-present)) <= STARTUP_RECOVERY["joint_excursion_rad"], "recovery_excursion_limit")
        else:
            cap = (MONITOR_RAD if recovery else COARSE_PROFILE_LIMITS["max_joint_change_rad"]
                   if coarse else STEP["joint_change_rad"])
            within = (all(_coarse_step_within(s,t) for t in (wanted, sent) for s in (start, present))
                      if coarse else all(abs(t-s) <= cap for t in (wanted, sent) for s in (start, present)))
            _need(within, "joint_step_limit", "joint%d" % (i+1))
            if not recovery:
                _need(min(wanted-lo, hi-wanted, sent-lo, hi-sent) >= STEP["joint_margin_rad"], "target_margin", "joint%d" % (i+1))
    cached = _cached(context["cached_target"], identity, origin, limits[arm])
    for i, (a, b, (lo, hi)) in enumerate(zip(cached, q0, limits[arm])):
        if recovery and not lo <= b <= hi:
            boundary = lo if b < lo else hi
            _need(min(b, boundary) <= a <= max(b, boundary), "cached_target_outside_recovery_corridor")
        else:
            _need(abs(a-b) <= MONITOR_RAD, "cached_target_not_at_original_anchor")
    loaded = type(geom) is dict and geom.get("schema") == LOADED_JOINT_PATH_SCHEMA
    loaded_context = copy.deepcopy(context.get("loaded_context"))
    _need(loaded or loaded_context is None, "loaded_context_requires_loaded_schema")
    rgb_deadline = radii = clearance = None
    if visual:
        _need(not recovery, "visual_joint_recovery_unsupported")
        rgb_deadline = validate_rgb_geometry(geom, identity, origin, now,
            schema=geom["schema"], target_raw=raw, loaded_context=loaded_context)
    else:
        _need(type(geom) is dict and set(geom) == {"attachment_radius_m", "available_clearance_m", "workspace_min_m", "workspace_max_m", "origin_sample_id", "source"}, "geometry_schema")
        _source(geom["source"])
        _need(geom["origin_sample_id"] == origin["sample_id"], "geometry_anchor_mismatch")
        attachments = geom["attachment_radius_m"]
        _need(type(attachments) is dict and set(attachments) == set(SIDES), "attachment_evidence_required")
        radii = {s: _radii(model["mdh"], _num(attachments[s], "attachment radius")) for s in SIDES}
        _need(all(attachments[s] > 0 for s in SIDES), "invalid_attachment_radius")
        clearance = _num(geom["available_clearance_m"], "clearance") - STEP["clearance_reserve_m"]
        _need(clearance > 0, "clearance_reserve_missing")
        wlo, whi = _vec(geom["workspace_min_m"], 3, "workspace min"), _vec(geom["workspace_max_m"], 3, "workspace max")
        _need(all(a < b for a, b in zip(wlo, whi)), "invalid_workspace")
    budget = context["budget"]
    _need(type(budget) is dict and set(budget) == {"max_translation_m", "max_rotation_rad"}, "budget_schema")
    translation, rotation = _num(budget["max_translation_m"], "translation budget"), _num(budget["max_rotation_rad"], "rotation budget")
    if coarse:
        _need(translation == COARSE_PROFILE_LIMITS["process_translation_m"]
              and rotation == COARSE_PROFILE_LIMITS["process_rotation_rad"], "coarse_fixed_budget_required")
        budget = {"max_translation_m": COARSE_PROFILE_LIMITS["process_translation_m"],
                  "max_rotation_rad": COARSE_PROFILE_LIMITS["process_rotation_rad"]}
    else:
        _need(0 < translation <= RECOVERY["active_displacement_m"] and 0 < rotation <= MAX_ROTATION_RAD, "budget_enlarged")
    origin_fk = {s: fk_matrix(model["mdh"], origin["arms"][s]["joints_rad"]) for s in SIDES}
    target_fk = fk_matrix(model["mdh"], encoded)
    endpoint_m, endpoint_r = _error(origin_fk[arm], target_fk)
    if coarse:
        for start in (q0, q):
            start_fk = fk_matrix(model["mdh"], start)
            for goal in (requested, encoded):
                distance, _ = _error(start_fk, fk_matrix(model["mdh"], goal))
                _need(distance <= COARSE_PROFILE_LIMITS["target_translation_m"], "model_endpoint_displacement")
    else:
        _need(endpoint_m <= RECOVERY["target_displacement_m"], "model_endpoint_displacement")
    contact_target_limit = None
    if loaded and loaded_context["operation"] in ("extract_segment", "insert_segment"):
        # Software target bounds, not force, insertion-depth or object-progress
        # measurements. Quantization cannot enlarge this local contact step.
        contact_target_limit = {"translation_m": .002, "rotation_rad": .01}
        for start in (q0, q):
            start_fk = fk_matrix(model["mdh"], start)
            for goal in (requested, encoded):
                distance, rotation_distance = _error(start_fk, fk_matrix(model["mdh"], goal))
                _need(distance <= .002, "loaded_contact_target_translation")
                _need(rotation_distance <= .01, "loaded_contact_target_rotation")
    # Full independent-joint box includes caller endpoints, encoded endpoints,
    # known cache, all partial pair updates and quantization, from original q.
    low = [min(vals)-MONITOR_RAD for vals in zip(q0, q, requested, encoded, cached)]
    high = [max(vals)+MONITOR_RAD for vals in zip(q0, q, requested, encoded, cached)]
    excursions = [max(abs(lo-v), abs(hi-v)) for v, lo, hi in zip(q0, low, high)]
    active_sweep = passive_sweep = body_hold_sweep = relative = None
    if not visual:
        active_sweep = sum(r*d for r, d in zip(radii[arm], excursions))
        passive_sweep = sum(r*MONITOR_RAD for r in radii[peer])
        body_hold_sweep = sum(r*MONITOR_RAD for r in radii[arm])
    flange_radii = [r-STEP["link_body_allowance_m"] for r in _radii(model["mdh"], 0.)]
    flange_sweep = sum(r*d for r, d in zip(flange_radii, excursions))
    hold_sweep, hold_rotation = sum(r*MONITOR_RAD for r in flange_radii), 6*MONITOR_RAD
    tracking_rotation = sum(excursions)
    model_box = None
    if not recovery:
        from .joint_model_bounds import flange_box_bounds, sum_upper, sum_products_upper
        model_box = flange_box_bounds(model["mdh"], q0, low, high)
        flange_sweep, tracking_rotation = model_box["translation_m"], model_box["rotation_rad"]
        # Hold retains the original global-radius allowance. The tighter
        # flange-only calculation never changes body/attachment clearance.
        hold_sweep = sum_products_upper(model_box["global_radius_bounds_m"], [MONITOR_RAD]*6)
        hold_rotation = sum_upper([MONITOR_RAD]*6)
        if coarse:
            # This separate latch-only contract never dispatches a hold target.
            # The strict box still includes cache, partial pair updates and
            # every-axis tracking tolerance; only the EXTRA hold is absent.
            hold_sweep, hold_rotation = 0., 0.
        combined_translation = sum_upper([flange_sweep, hold_sweep])
        combined_rotation = sum_upper([tracking_rotation, hold_rotation])
    else:
        combined_translation, combined_rotation = flange_sweep+hold_sweep, tracking_rotation+hold_rotation
    if not visual:
        relative = max(2*(active_sweep+body_hold_sweep), 2*passive_sweep, active_sweep+body_hold_sweep+passive_sweep)
        _need(relative <= clearance, "relative_sweep_exceeds_clearance")
    # Both constructions bound the complete independent box, not sampled
    # endpoints or an assumed synchronized controller interpolation.
    _need(combined_translation <= translation and combined_rotation <= rotation,
          "insufficient_remaining_hold_budget")
    if not visual:
        for side in SIDES:
            bound = combined_translation if side == arm else hold_sweep
            _need(all(lo <= origin_fk[side][i][3]-bound and origin_fk[side][i][3]+bound <= hi
                      for i, (lo, hi) in enumerate(zip(wlo, whi))), "model_workspace_envelope", side)
    # Ordinary RGB RX has a separate, time-bounded observation policy. This
    # does not enlarge the target or the strict box checked before ANY frame.
    # Its complete box is diagnostic, not a claim that every point satisfies
    # the original pose budgets; each RX sample must still pass those guards.
    tracking_policy = _tracking_policy(visual, coarse)
    origin_cap = tracking_policy["max_origin_excursion_rad"]
    postsend_envelope = {"low_rad": low[:], "high_rad": high[:]}
    postsend_model_box = copy.deepcopy(model_box)
    if visual:
        band = tracking_policy["transient_band_rad"]
        postsend_envelope = {
            "low_rad": [max(min(a,b)-band,a-origin_cap) for a,b in zip(q0,encoded)],
            "high_rad": [min(max(a,b)+band,a+origin_cap) for a,b in zip(q0,encoded)]}
        postsend_model_box = flange_box_bounds(model["mdh"],q0,
            postsend_envelope["low_rad"],postsend_envelope["high_rad"])
    mixed = []
    for prefix in range(4):
        hybrid = encoded[:prefix*2] + cached[prefix*2:]
        transform = fk_matrix(model["mdh"], hybrid)
        mixed.append({"joint_pairs_updated": prefix, "joints_rad": hybrid,
                      "model_flange_transform": transform, "error_from_original": matrix_error(origin_fk[arm], transform)})
    plan = {"schema": PLAN_SCHEMA, "identity": identity, "recovery_mode": recovery_mode,
        "motion_profile": "coarse_approach" if coarse else "ordinary",
        "profile_limits": copy.deepcopy(COARSE_PROFILE_LIMITS) if coarse else None,
        "planning_joint_feedback_rad": {s: list(states[s]["joints_rad"]) for s in SIDES} if coarse else None,
        "origin": origin, "origin_sha256": context["origin_sha256"], "planned_sample_id": current["sample_id"],
        "model": model, "model_sources": sources, "controller_limits": copy.deepcopy(context["controller_limits"]),
        "effective_joint_limits_rad": limits, "effective_joint_limits_raw": raw_limits,
        "initialization_ingress": ingress,
        # The existing hold transaction validates BOTH arms' original nominal
        # limits. A sourced residual does not change that separate contract.
        "hold_reference_within_nominal_limits": all(
            _within(origin["arms"][s]["joints_rad"], limits[s]) for s in SIDES),
        "limit_policy": "explicit_intersection_of_pinned_sdk_urdf_and_controller",
        "j6_source_disagreement": {"sdk_rad": model["joint_limits_rad"][5], "urdf_rad": urdf[5], "resolved_by_widening": False},
        "requested_target_joints_rad": requested, "encoded_target_joints_rad": encoded, "target_raw": raw,
        "target_quantization_error_rad": [abs(a-b) for a, b in zip(requested, encoded)], "frames": joint_hold_frames(raw),
        "cached_target": copy.deepcopy(context["cached_target"]), "mixed_targets": mixed,
        "controller_flange_pose": {s: list(states[s]["pose_m_rad"]) for s in SIDES},
        "model_original_flange_transform": origin_fk, "model_target_flange_transform": target_fk,
        "model_endpoint_displacement_m": endpoint_m, "model_endpoint_rotation_rad": endpoint_r,
        "joint_envelope": {"low_rad": low, "high_rad": high}, "geometry": geom,
        "loaded_context": loaded_context, "loaded_observation_only": loaded,
        "contact_target_limit": contact_target_limit,
        "contact_force_verified": False, "object_progress_verified": False,
        "tracking_policy": tracking_policy, "postsend_joint_envelope": postsend_envelope,
        "postsend_flange_box_bound": postsend_model_box,
        "postsend_entire_box_admitted": False,
        "postsend_bound_scope": "RX_observation_only_each_sample_requires_original_pose_and_feedback_limits",
        "spatial_admission_mode": "rgb_supervised" if visual else "metric_geometry",
        "metric_clearance_checked": not visual, "absolute_workspace_checked": not visual,
        "metric_collision_checked": False, "visual_rgb_deadline": rgb_deadline,
        "workspace_policy": ("visual_corridor_plus_existing_relative_envelopes" if visual
                             else "metric_model_base_workspace"),
        "hold_supported": False if visual else None,
        "hold_policy": "latch_only" if visual else "metric_guarded",
        "sweep_axis_radii_m": radii, "active_sweep_bound_m": active_sweep,
        "flange_tracking_sweep_bound_m": flange_sweep,
        "flange_tracking_rotation_bound_rad": tracking_rotation,
        "flange_box_bound": model_box,
        "flange_box_bound_scope": "strict_sending_and_nominal_tracking_only",
        "passive_sweep_bound_m": passive_sweep, "relative_sweep_with_hold_bound_m": relative,
        "remaining_hold_budget": {"translation_m": translation-flange_sweep,
            "rotation_rad": rotation-tracking_rotation, "required_translation_m": hold_sweep,
            "required_rotation_rad": hold_rotation}, "budget": copy.deepcopy(budget),
        "unloaded_evidence": copy.deepcopy(context.get("unloaded_evidence")),
        "candidate_valid": True, "motion_permitted": False, "dispatch_authorized": False,
        "hardware_commands_sent": 0, "physical_stop_verified": None, "atomic_update_proven": False,
        "scope": "One six-axis model-space candidate and monitored box; host evidence is not authenticated; no IK, TX, retry, recovery selection or loaded-contact qualification"}
    plan["plan_sha256"] = evidence_sha256(plan)
    validate_joint_path_sample(plan, current, now=now, phase="pre_dispatch")
    return plan


def _check_plan_policy(plan, visual, coarse):
    """A recomputed content hash cannot enlarge a known software profile.

    This does not authenticate a caller's plan or scene. The adapter continues
    to bind the plan to its actual context, target, owner and connection.
    """
    expected_profile = "coarse_approach" if coarse else "ordinary"
    _need(plan.get("motion_profile", None if coarse else "ordinary") == expected_profile,
          "motion_profile_mismatch")
    expected_limits = COARSE_PROFILE_LIMITS if coarse else None
    _need(evidence_sha256(plan.get("profile_limits")) == evidence_sha256(expected_limits),
          "profile_limits_changed")
    budget = plan["budget"]
    _need(type(budget) is dict and set(budget) == {"max_translation_m", "max_rotation_rad"}, "budget_schema")
    translation = _num(budget["max_translation_m"], "translation budget")
    rotation = _num(budget["max_rotation_rad"], "rotation budget")
    if coarse:
        _need(translation == COARSE_PROFILE_LIMITS["process_translation_m"]
              and rotation == COARSE_PROFILE_LIMITS["process_rotation_rad"], "coarse_fixed_budget_required")
    else:
        _need(0 < translation <= RECOVERY["active_displacement_m"]
              and 0 < rotation <= MAX_ROTATION_RAD, "budget_enlarged")
    _need(evidence_sha256(plan["tracking_policy"]) == evidence_sha256(_tracking_policy(visual, coarse)),
          "tracking_policy_changed")
    if not coarse:
        return
    _need(plan["recovery_mode"] is None and plan["initialization_ingress"] is None,
          "coarse_boundary_exception_unsupported")
    _need(plan.get("loaded_context") is None and plan["hold_policy"] == "latch_only"
          and plan["hold_supported"] is False, "coarse_action_contract_changed")
    _need(plan["remaining_hold_budget"]["required_translation_m"] == 0.
          and plan["remaining_hold_budget"]["required_rotation_rad"] == 0., "coarse_hold_reserve_changed")
    arm = plan["identity"]["arm"]
    for side in SIDES:
        q0 = _vec(plan["origin"]["arms"][side]["joints_rad"], 6, "origin joints")
        present = _vec(plan["planning_joint_feedback_rad"][side], 6, "planned current joints")
        limits = plan["effective_joint_limits_rad"][side]
        _need(_within(q0, limits) and _within(present, limits), "coarse_nominal_start_required", side)
    q0 = plan["origin"]["arms"][arm]["joints_rad"]
    present = plan["planning_joint_feedback_rad"][arm]
    wanted = _vec(plan["requested_target_joints_rad"], 6, "requested joints")
    raw, sent = encode_joint_target(wanted)
    _need(raw == plan["target_raw"] and sent == plan["encoded_target_joints_rad"]
          and joint_hold_frames(raw) == plan["frames"], "coarse_target_encoding_changed")
    for i, (a, b, goal, encoded, (lo, hi)) in enumerate(zip(
            q0, present, wanted, sent, plan["effective_joint_limits_rad"][arm])):
        _need(min(goal-lo, hi-goal, encoded-lo, hi-encoded) >= STEP["joint_margin_rad"],
              "target_margin", "joint%d" % (i+1))
        _need(all(_coarse_step_within(s,t) for t in (goal, encoded) for s in (a, b)),
              "joint_step_limit", "joint%d" % (i+1))
    cached = _cached(plan["cached_target"], plan["identity"], plan["origin"],
                     plan["effective_joint_limits_rad"][arm])
    strict = {"low_rad": [min(values)-MONITOR_RAD for values in zip(q0,present,wanted,sent,cached)],
              "high_rad": [max(values)+MONITOR_RAD for values in zip(q0,present,wanted,sent,cached)]}
    cap = COARSE_PROFILE_LIMITS["max_joint_change_rad"]+MONITOR_RAD
    transient = STEP["joint_change_rad"]
    settling = {"low_rad": [max(min(a,b)-transient,a-cap) for a,b in zip(q0,sent)],
                "high_rad": [min(max(a,b)+transient,a+cap) for a,b in zip(q0,sent)]}
    _need(plan["joint_envelope"] == strict and plan["postsend_joint_envelope"] == settling,
          "coarse_envelope_changed")
    from .joint_model_bounds import flange_box_bounds
    whole_box = flange_box_bounds(plan["model"]["mdh"], q0, strict["low_rad"], strict["high_rad"])
    _need(whole_box["translation_m"] <= COARSE_PROFILE_LIMITS["process_translation_m"]
          and whole_box["rotation_rad"] <= COARSE_PROFILE_LIMITS["process_rotation_rad"],
          "coarse_independent_box_budget")
    for start in (q0, present):
        start_fk = fk_matrix(plan["model"]["mdh"], start)
        for goal in (wanted, sent):
            distance, _ = _error(start_fk, fk_matrix(plan["model"]["mdh"], goal))
            _need(distance <= COARSE_PROFILE_LIMITS["target_translation_m"], "model_endpoint_displacement")


@_input_errors
def validate_joint_path_sample(plan, sample, *, now, phase="active"):
    """Check one sample without renewing anchors or accounting elapsed time.

    ``settling`` is RGB-only and must be selected by the adapter only after its
    complete original four-frame attempt. This pure function cannot establish
    that fact or its cumulative one-second limit and never permits another TX.
    """
    _need(type(plan) is dict and plan.get("schema") == PLAN_SCHEMA, "plan_schema")
    _need(phase in ("pre_dispatch", "active", "settling"), "invalid_phase")
    claimed = plan.get("plan_sha256")
    _need(_hash(claimed) == evidence_sha256({k: v for k, v in plan.items() if k != "plan_sha256"}), "plan_changed")
    _need(plan["origin_sha256"] == evidence_sha256(plan["origin"]), "original_anchor_changed")
    now = _num(now, "now")
    identity, origin = plan["identity"], plan["origin"]
    arm, recovery = identity["arm"], plan["recovery_mode"] is not None
    visual, coarse = _geometry_kind(plan["geometry"])
    _check_plan_policy(plan, visual, coarse)
    settling = phase == "settling"
    _need(not settling or visual and plan["tracking_policy"]["mode"] == "bounded_postsend_settling",
          "settling_requires_rgb_supervision")
    if visual:
        _need(not recovery, "visual_joint_recovery_unsupported")
        _need(validate_rgb_geometry(plan["geometry"], identity, origin, now,
              schema=plan["geometry"]["schema"], target_raw=plan["target_raw"],
              loaded_context=plan.get("loaded_context"))
              == plan["visual_rgb_deadline"], "visual_rgb_deadline_changed")
    states, _ = _pair(sample, identity, now, jaw_enabled=not recovery)
    _need(sample["captured_at"] >= origin["captured_at"], "sample_precedes_origin")
    results, outside = {}, []
    for side in SIDES:
        state, base = states[side], origin["arms"][side]
        q, start = state["joints_rad"], base["joints_rad"]
        nominal_limits = plan["effective_joint_limits_rad"][side]
        ingress = plan.get("initialization_ingress")
        limits = ingress["feedback_limits_rad"][side] if ingress and not recovery else nominal_limits
        for i, (value, initial, (lo, hi)) in enumerate(zip(q, start, limits)):
            lower, upper = lo, hi
            if recovery and initial < lo and i == 1:
                lower = max(lo-STARTUP_RECOVERY["joint_excursion_rad"], initial-MONITOR_RAD)
            if recovery and initial > hi and i == 2:
                upper = min(hi+STARTUP_RECOVERY["joint_excursion_rad"], initial+MONITOR_RAD)
            _need(lower <= value <= upper, "feedback_joint_limit", side + "/joint%d" % (i+1))
        active = side == arm and phase in ("active", "settling")
        if active:
            envelope = plan["postsend_joint_envelope"] if settling else plan["joint_envelope"]
            _need(all(lo <= v <= hi for v, lo, hi in zip(q, envelope["low_rad"], envelope["high_rad"])), "joint_tracking_envelope")
            if settling:
                cap = plan["tracking_policy"]["max_origin_excursion_rad"]
                _need(all(a-cap <= v <= a+cap for v,a in zip(q,start)),
                      "joint_origin_excursion_limit")
        else:
            _need(max(abs(a-b) for a, b in zip(q, start)) <= MONITOR_RAD, "stationary_joint_anchor", side)
            _need(state["arm_status"]["motion_status"] == 0, "stationary_motion_status", side)
        _need(state["arm_status"]["mode_feedback"] == base["arm_status"]["mode_feedback"], "movement_mode_changed", side)
        _need(state["gripper"]["foc_status"]["driver_enable_status"] is base["gripper"]["foc_status"]["driver_enable_status"], "jaw_enable_changed", side)
        _need(abs(state["gripper"]["width_m"]-base["gripper"]["width_m"]) <= POSITION_STILL_M, "jaw_anchor_drift", side)
        for name in arms.PARTS + arms.DRIVERS + ("gripper",):
            _need(state["fragment_timestamps_s"][name] >= base["fragment_timestamps_s"][name], "feedback_before_original", side)
        raw_m, raw_r = _error(pose_matrix(base["pose_m_rad"]), pose_matrix(state["pose_m_rad"]))
        model_pose = fk_matrix(plan["model"]["mdh"], q)
        model_m, model_r = _error(plan["model_original_flange_transform"][side], model_pose)
        maximum_m = plan["budget"]["max_translation_m"] if active else POSITION_STILL_M
        maximum_r = plan["budget"]["max_rotation_rad"] if active else ROTATION_STILL_RAD
        _need(raw_m <= maximum_m and raw_r <= maximum_r, "controller_relative_pose_envelope", side)
        _need(model_m <= maximum_m and model_r <= maximum_r, "model_relative_pose_envelope", side)
        if side == arm:
            outside = [{"joint_index": i+1, "observed_rad": v, "low_rad": lo, "high_rad": hi,
                        "excess_rad": max(lo-v,v-hi)}
                       for i,(v,lo,hi) in enumerate(zip(q,plan["joint_envelope"]["low_rad"],
                                                       plan["joint_envelope"]["high_rad"]))
                       if not lo <= v <= hi]
        results[side] = {"controller_flange_pose": list(state["pose_m_rad"]), "model_flange_transform": model_pose,
                         "controller_displacement_m": raw_m, "model_displacement_m": model_m,
                         "strict_nominal": _within(q, nominal_limits)}
    return {"sample_id": sample["sample_id"], "origin_sha256": plan["origin_sha256"], "plan_sha256": claimed,
            "within_joint_path_envelope": True, "arms": results, "motion_permitted": False,
            "tracking": {"within_nominal_band": not outside, "outside_nominal_band": outside,
                         "max_excess_rad": max((row["excess_rad"] for row in outside),default=0.),
                         "cumulative_time_checked": False},
            "spatial_admission_mode": plan["spatial_admission_mode"],
            "metric_clearance_checked": not visual, "absolute_workspace_checked": not visual,
            "metric_collision_checked": False, "hold_supported": plan["hold_supported"],
            "loaded_observation_only": plan["loaded_observation_only"],
            "contact_force_verified": False, "object_progress_verified": False,
            "hold_policy": plan["hold_policy"],
            "physical_stop_verified": None, "qualification_granted": False}
