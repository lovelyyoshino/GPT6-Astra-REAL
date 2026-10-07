"""Pure PiPER X first-target candidates under the supervised startup scope.

No robot construction, sending, cache inference, or physical-stop qualification.
The persistent adapter owns the real baseline, once-only claim, per-frame fresh
feedback, and post-send J-mode/arrival/stability receipt. Unlike ordinary joint
motion, this separate legacy startup contract does not require a known old J
target, a global .05 rad orientation budget, or an exceptional hold transaction.
Unknown cached/mixed targets remain an explicit supervised-operation risk.
"""
import copy

from . import arms
from .bounded_joint_step import STEP
from .joint_recovery import RECOVERY
from .startup_recovery import STARTUP_RECOVERY
from .takeover import LIMITS
from .hold_transaction import joint_hold_frames
from .joint_path import (
    JointPathError, _input_errors, _need, _num, _vec, _text, _hash, _source,
    _identity, _enum, _load_model, _limits, _within, _radii, _error,
    encode_joint_target, evidence_sha256)
from .model_compatibility import fk_matrix, pose_matrix


SCHEMA = "piper_x_joint_initialization_context_v1"
PLAN_SCHEMA = "piper_x_joint_initialization_plan_v1"
VISUAL_GEOMETRY_SCHEMA = "piper_rgb_supervised_initialization_v1"
RGB_MAX_AGE_S = 30.0
RGB_MAX_SKEW_S = .15
SIDES = ("left", "right")
JointInitializationError = JointPathError
MONITOR_RAD = STARTUP_RECOVERY["joint_tolerance_rad"]
# A fixed origin envelope and a three-second stability window are distinct.
# Preserve the standalone startup's origin tolerances; the adapter also retains
# its existing ready/preparation anchor contract and the .0005 m stable window.
POSITION_STILL_M = LIMITS["position_m"]
JAW_STILL_M = LIMITS["gripper_m"]
ROTATION_STILL_RAD = .003
TRANSIENT_BAND_RAD = STEP["joint_change_rad"]
MAX_CUMULATIVE_OUTSIDE_BAND_S = 1.0
CONTEXT_KEYS = {"schema", "identity", "model_catalog", "urdf_source", "controller_limits",
                "origin", "current", "origin_sha256", "geometry", "unloaded_evidence"}


def _visual_geometry(geometry, identity, origin, now):
    """Retain the separate initialization schema and its existing checks."""
    from .rgb_supervision import validate_rgb_geometry
    return validate_rgb_geometry(geometry, identity, origin, now,
                                 schema=VISUAL_GEOMETRY_SCHEMA)

def _pair(sample, identity, now):
    _need(type(sample) is dict and {"sample_id", "identity", "captured_at", "arms"} <= set(sample), "sample_schema")
    _text(sample["sample_id"], "sample_id")
    _need(sample["identity"] == identity, "sample_identity_changed")
    captured = _num(sample["captured_at"], "capture time")
    _need(0 <= now-captured <= RECOVERY["feedback_age_s"], "stale_sample")
    states = sample["arms"]
    _need(type(states) is dict and set(states) == set(SIDES), "pair_feedback_required")
    stamps = []
    for side, state in states.items():
        _need(type(state) is dict and state.get("status") == "complete", "incomplete_feedback", side)
        _need(arms.control_health(state, now_s=now, allowed_control_modes=(1,), require_enabled=False)["healthy"], "unhealthy_feedback", side)
        status = state["arm_status"]
        _need(_enum(status.get("teach_status"), (0,)) and _enum(status.get("motion_status"), (0, 1)), "unexpected_status", side)
        _need(_enum(status.get("mode_feedback"), (0, 1, 2)), "unknown_movement_mode", side)
        _need(all(state["drivers"][str(i)]["foc_status"]["driver_enable_status"] is True for i in range(1, 7)), "joint_disabled", side)
        _need(type(state["gripper"]["foc_status"]["driver_enable_status"]) is bool, "jaw_enable_unknown", side)
        _need(0 <= state["gripper"]["width_m"] <= .070, "jaw_width_limit", side)
        for part in arms.PARTS + arms.DRIVERS + ("gripper",):
            stamp = _num(state["fragment_timestamps_s"][part], "fragment timestamp")
            _need(0 <= now-stamp <= RECOVERY["feedback_age_s"] and stamp <= captured, "stale_feedback", side+"/"+part)
            stamps.append(stamp)
    _need(max(stamps)-min(stamps) <= RECOVERY["feedback_age_s"], "pair_feedback_skew")
    return states


def _violations(q, limits):
    return [{"joint_index": i+1, "observed_rad": value, "minimum_rad": lo, "maximum_rad": hi}
            for i, (value, (lo, hi)) in enumerate(zip(q, limits)) if not lo <= value <= hi]


def _origin_bounds(q, limits, side):
    for i, (value, (lo, hi)) in enumerate(zip(q, limits)):
        if lo <= value <= hi:
            continue
        allowed = (i == 1 and value < lo) or (i == 2 and value > hi)
        _need(allowed and min(abs(value-lo), abs(value-hi)) <= STARTUP_RECOVERY["joint_excursion_rad"],
              "origin_joint_limit", side+"/joint%d" % (i+1))


def _feedback_bounds(q, start, limits, side):
    for i, (value, initial, (lo, hi)) in enumerate(zip(q, start, limits)):
        low, high = lo-MONITOR_RAD, hi+MONITOR_RAD
        if initial < lo:
            _need(i == 1, "invalid_boundary_origin", side)
            low = max(lo-STARTUP_RECOVERY["joint_excursion_rad"], initial-MONITOR_RAD)
        if initial > hi:
            _need(i == 2, "invalid_boundary_origin", side)
            high = min(hi+STARTUP_RECOVERY["joint_excursion_rad"], initial+MONITOR_RAD)
        _need(low <= value <= high, "feedback_joint_limit", side+"/joint%d" % (i+1))


@_input_errors
def plan_joint_initialization(context, *, now):
    """Freeze a measured legal seed or exact nearest J2/J3 boundary candidate.

    Sources retain the joint_path shapes, or geometry explicitly tags the
    attended RGB initialization branch. Cached targets and budgets are absent.
    Both branches require a host-resolved unloaded evidence source.
    The target is derived from the original sample, never reanchored by current.
    """
    _need(type(context) is dict and set(context) == CONTEXT_KEYS and context.get("schema") == SCHEMA, "context_schema")
    now, identity = _num(now, "now"), _identity(context["identity"])
    arm, peer = identity["arm"], next(s for s in SIDES if s != identity["arm"])
    origin, current = copy.deepcopy(context["origin"]), copy.deepcopy(context["current"])
    _need(evidence_sha256(origin) == _hash(context["origin_sha256"]), "original_anchor_changed")
    original = _pair(origin, identity, _num(origin["captured_at"], "origin time"))
    states = _pair(current, identity, now)
    _need(current["captured_at"] >= origin["captured_at"], "current_precedes_origin")
    _need(all(s["arm_status"]["motion_status"] == 0 for s in original.values())
          and all(s["arm_status"]["motion_status"] == 0 for s in states.values()), "stationary_baseline_required")
    proof = context["unloaded_evidence"]
    _need(type(proof) is dict and set(proof) == {"origin_sample_id", "source"}
          and proof["origin_sample_id"] == origin["sample_id"], "explicit_unloaded_evidence_required")
    _source(proof["source"])
    model, urdf, sources = _load_model(context["model_catalog"], context["urdf_source"])
    limits, raw_limits = _limits(model, urdf, context["controller_limits"])
    for side in SIDES:
        _origin_bounds(original[side]["joints_rad"], limits[side], side)
    q0 = original[arm]["joints_rad"]
    violations = {side: _violations(original[side]["joints_rad"], limits[side]) for side in SIDES}
    purpose = "startup_j2_j3" if violations[arm] else "seed_current"
    requested = [max(lo, min(hi, q)) for q, (lo, hi) in zip(q0, limits[arm])]
    raw, encoded = encode_joint_target(requested)
    _need(_within(requested, limits[arm]) and _within(encoded, limits[arm])
          and all(lo <= v <= hi for v, (lo, hi) in zip(raw, raw_limits[arm])), "target_joint_limit")
    for i, (initial, wanted, sent, (lo, hi)) in enumerate(zip(q0, requested, encoded, limits[arm])):
        if not lo <= initial <= hi:
            boundary = lo if initial < lo else hi
            _need(wanted == boundary and sent == boundary, "recovery_requires_exact_nearest_boundary")
            _need(abs(sent-initial) <= STARTUP_RECOVERY["joint_excursion_rad"], "recovery_excursion_limit")
        else:
            _need(max(abs(wanted-initial), abs(sent-initial)) <= MONITOR_RAD, "seed_quantization_excursion", "joint%d" % (i+1))
    geom = copy.deepcopy(context["geometry"])
    visual = type(geom) is dict and geom.get("schema") == VISUAL_GEOMETRY_SCHEMA
    rgb_deadline = None
    if visual:
        rgb_deadline = _visual_geometry(geom, identity, origin, now)
        radii = clearance = None
    else:
        _need(type(geom) is dict and set(geom) == {"attachment_radius_m", "available_clearance_m", "workspace_min_m", "workspace_max_m", "origin_sample_id", "source"}, "geometry_schema")
        _source(geom["source"])
        _need(geom["origin_sample_id"] == origin["sample_id"], "geometry_anchor_mismatch")
        attachments = geom["attachment_radius_m"]
        _need(type(attachments) is dict and set(attachments) == set(SIDES), "attachment_evidence_required")
        radii = {side: _radii(model["mdh"], _num(attachments[side], "attachment radius")) for side in SIDES}
        _need(all(attachments[side] > 0 for side in SIDES), "invalid_attachment_radius")
        clearance = _num(geom["available_clearance_m"], "clearance") - STARTUP_RECOVERY["clearance_reserve_m"]
        _need(clearance > 0, "clearance_reserve_missing")
        wlo, whi = _vec(geom["workspace_min_m"], 3, "workspace min"), _vec(geom["workspace_max_m"], 3, "workspace max")
        _need(all(lo < hi for lo, hi in zip(wlo, whi)), "invalid_workspace")
    model_origin = {side: fk_matrix(model["mdh"], original[side]["joints_rad"]) for side in SIDES}
    model_target = fk_matrix(model["mdh"], encoded)
    endpoint_m, endpoint_r = _error(model_origin[arm], model_target)
    _need(endpoint_m <= RECOVERY["target_displacement_m"], "model_endpoint_displacement")
    rounding = [abs(a-b) for a, b in zip(requested, encoded)]
    # Exactly the legacy startup sweep: actual delta plus observation band and
    # quantization. No ordinary MOVE_J hold reserve or .05-radian budget added.
    excursions = [abs(b-a)+MONITOR_RAD+error for a,b,error in zip(q0,encoded,rounding)]
    active_sweep = passive_sweep = relative = None
    if not visual:
        active_sweep = sum(r*d for r,d in zip(radii[arm],excursions))
        passive_sweep = sum(r*MONITOR_RAD for r in radii[peer])
        relative = max(2*active_sweep,2*passive_sweep,active_sweep+passive_sweep)
        _need(relative <= clearance, "relative_sweep_exceeds_clearance")
    flange_radii = [r-STEP["link_body_allowance_m"] for r in _radii(model["mdh"],0.)]
    flange_bounds = {arm: sum(r*d for r,d in zip(flange_radii,excursions)),
                     peer: sum(r*MONITOR_RAD for r in flange_radii)}
    transient_band = TRANSIENT_BAND_RAD if visual else MONITOR_RAD
    tracking_policy = {"mode": "bounded_postsend_settling" if visual else "strict",
        "transient_band_rad": transient_band,
        "max_cumulative_outside_band_s": MAX_CUMULATIVE_OUTSIDE_BAND_S if visual else 0.,
        "max_origin_excursion_rad": STARTUP_RECOVERY["joint_excursion_rad"],
        "settle_tolerance_rad": MONITOR_RAD}
    # A separate RX-only interval, never an enlarged target or a send envelope.
    # Its half-width is the TOTAL .025 band, not .003 + .025. Hard feedback
    # limits, the frozen-origin cap and both measured 20 mm envelopes still
    # intersect this interval in the monitor. The adapter owns cumulative time.
    strict_envelope = {"low_rad": [min(a,b)-MONITOR_RAD for a,b in zip(q0,requested)],
                       "high_rad": [max(a,b)+MONITOR_RAD for a,b in zip(q0,requested)]}
    postsend_envelope = {
        "low_rad": [min(a,b)-transient_band for a,b in zip(q0,encoded)],
        "high_rad": [max(a,b)+transient_band for a,b in zip(q0,encoded)]} if visual else copy.deepcopy(strict_envelope)
    postsend_excursions = [max(abs(lo-a),abs(hi-a)) for a,lo,hi in
        zip(q0,postsend_envelope["low_rad"],postsend_envelope["high_rad"])]
    postsend_flange_bounds = {arm: sum(r*d for r,d in zip(flange_radii,postsend_excursions)),
                             peer: flange_bounds[peer]} if visual else copy.deepcopy(flange_bounds)
    if not visual:
        for side in SIDES:
            _need(all(lo <= model_origin[side][i][3]-flange_bounds[side]
                      and model_origin[side][i][3]+flange_bounds[side] <= hi
                      for i,(lo,hi) in enumerate(zip(wlo,whi))), "model_workspace_envelope", side)
    plan = {"schema": PLAN_SCHEMA, "purpose": purpose, "identity": identity,
        "origin": origin, "origin_sha256": context["origin_sha256"], "model": model,
        "sources": {**sources, "controller_limits": copy.deepcopy(context["controller_limits"]["source"])},
        "effective_joint_limits_rad": limits, "effective_joint_limits_raw": raw_limits,
        "initial_boundary_violations": violations,
        "j6_source_disagreement": {"sdk_rad": model["joint_limits_rad"][5], "urdf_rad": urdf[5], "resolved_by_widening": False},
        "requested_target_joints_rad": requested, "encoded_target_joints_rad": encoded,
        "target_raw": raw, "target_quantization_error_rad": rounding, "frames": joint_hold_frames(raw),
        "controller_flange_pose": {side: copy.deepcopy(original[side]["pose_m_rad"]) for side in SIDES},
        "model_original_flange_transform": model_origin, "model_target_flange_transform": model_target,
        "model_endpoint_displacement_m": endpoint_m, "model_endpoint_rotation_rad": endpoint_r,
        "joint_envelope": strict_envelope,
        "tracking_policy": tracking_policy, "postsend_joint_envelope": postsend_envelope,
        "geometry": geom, "sweep_axis_radii_m": radii, "sweep_axis_excursions_rad": excursions,
        "active_sweep_bound_m": active_sweep, "passive_sweep_bound_m": passive_sweep,
        "relative_sweep_bound_m": relative, "clearance_budget_m": clearance,
        "flange_tracking_sweep_bound_m": flange_bounds,
        "flange_tracking_sweep_bound_scope": "strict_band_only_excludes_postsend_settling",
        "postsend_flange_tracking_sweep_bound_m": postsend_flange_bounds,
        "postsend_sweep_axis_excursions_rad": postsend_excursions,
        "spatial_admission_mode": "rgb_supervised" if visual else "metric",
        "metric_clearance_checked": not visual, "metric_collision_checked": False,
        "absolute_workspace_checked": not visual,
        "workspace_policy": "visual_corridor_plus_existing_relative_envelopes" if visual else "metric_model_base_workspace",
        "visual_rgb_deadline": rgb_deadline,
        "unloaded_evidence": copy.deepcopy(proof),
        "startup_limits": {**RECOVERY, **STARTUP_RECOVERY},
        "candidate_valid": True, "motion_permitted": False, "dispatch_authorized": False,
        "hardware_commands_sent": 0, "physical_stop_verified": None, "general_stop_validated": False,
        "cached_target_prior": "unknown", "mode_frame_can_activate_cached_target": True,
        "partial_frames_can_mix_old_targets": True, "hard_path_guarantee": False,
        "atomic_update_proven": False, "caller_clearance_and_attendance_required": True,
        "automatic_retry": False, "task_motion_ready": False,
        "scope": ("RGB-supervised local startup: visual corridor substitutes for unknown metric clearance and absolute workspace; "
                  if visual else "Metric-bounded supervised startup; ")+
                 "raw telemetry and physical-model FK remain independent; no TX, cache establishment, loaded hold, collision proof or physical stop"}
    plan["plan_sha256"] = evidence_sha256(plan)
    validate_joint_initialization_sample(plan,current,now=now,phase="pre_dispatch")
    return plan


@_input_errors
def validate_joint_initialization_sample(plan, sample, *, now, phase="active", mode_confirmed=False):
    """Classify one sample against the frozen origin, without time accounting.

    Only the adapter may select ``settling``, after all four original frames
    have returned. This pure classification proves neither that transition nor
    its cumulative one-second budget, and never authorizes another send.
    """
    _need(type(plan) is dict and plan.get("schema") == PLAN_SCHEMA, "plan_schema")
    _need(phase in ("pre_dispatch", "active", "settling") and type(mode_confirmed) is bool, "invalid_phase")
    claimed = _hash(plan.get("plan_sha256"))
    _need(claimed == evidence_sha256({k:v for k,v in plan.items() if k != "plan_sha256"}), "plan_changed")
    _need(plan["origin_sha256"] == evidence_sha256(plan["origin"]), "original_anchor_changed")
    now = _num(now,"now")
    identity, origin = plan["identity"], plan["origin"]
    visual = plan["geometry"].get("schema") == VISUAL_GEOMETRY_SCHEMA
    settling = phase == "settling"
    _need(not settling or visual and plan["tracking_policy"]["mode"] == "bounded_postsend_settling",
          "settling_requires_rgb_supervision")
    if visual:
        _need(_visual_geometry(plan["geometry"], identity, origin, now) == plan["visual_rgb_deadline"],
              "visual_rgb_deadline_changed")
    arm = identity["arm"]
    states = _pair(sample,identity,now)
    _need(sample["captured_at"] >= origin["captured_at"], "sample_precedes_origin")
    results, strict, outside = {}, {}, []
    for side in SIDES:
        state, base = states[side], origin["arms"][side]
        q,start = state["joints_rad"],base["joints_rad"]
        limits = plan["effective_joint_limits_rad"][side]
        _feedback_bounds(q,start,limits,side)
        active = side == arm and phase in ("active", "settling")
        if active:
            envelope = plan["postsend_joint_envelope"] if settling else plan["joint_envelope"]
            _need(all(lo <= v <= hi for v,lo,hi in zip(q,envelope["low_rad"],envelope["high_rad"])), "joint_tracking_envelope")
            if settling:
                _need(max(abs(v-a) for v,a in zip(q,start)) <= plan["tracking_policy"]["max_origin_excursion_rad"],
                      "recovery_excursion_limit")
            allowed_modes = (1,) if mode_confirmed else (base["arm_status"]["mode_feedback"],1)
        else:
            _need(max(abs(a-b) for a,b in zip(q,start)) <= MONITOR_RAD, "stationary_joint_anchor", side)
            _need(state["arm_status"]["motion_status"] == 0, "stationary_motion_status", side)
            allowed_modes = (base["arm_status"]["mode_feedback"],)
        _need(state["arm_status"]["mode_feedback"] in allowed_modes, "movement_mode_changed", side)
        _need(state["gripper"]["foc_status"]["driver_enable_status"] is base["gripper"]["foc_status"]["driver_enable_status"], "jaw_enable_changed", side)
        _need(abs(state["gripper"]["width_m"]-base["gripper"]["width_m"]) <= JAW_STILL_M, "jaw_anchor_drift", side)
        for part in arms.PARTS+arms.DRIVERS+("gripper",):
            _need(state["fragment_timestamps_s"][part] >= base["fragment_timestamps_s"][part], "feedback_before_original", side)
        raw_m,raw_r = _error(pose_matrix(base["pose_m_rad"]),pose_matrix(state["pose_m_rad"]))
        model_pose = fk_matrix(plan["model"]["mdh"],q)
        model_m,model_r = _error(plan["model_original_flange_transform"][side],model_pose)
        max_m = RECOVERY["active_displacement_m"] if active else POSITION_STILL_M
        _need(raw_m <= max_m and (active or raw_r <= ROTATION_STILL_RAD), "controller_relative_pose_envelope", side)
        _need(model_m <= max_m and (active or model_r <= ROTATION_STILL_RAD), "model_relative_pose_envelope", side)
        if not visual:
            wlo,whi = plan["geometry"]["workspace_min_m"],plan["geometry"]["workspace_max_m"]
            _need(all(lo <= model_pose[i][3] <= hi for i,(lo,hi) in enumerate(zip(wlo,whi))), "model_workspace_envelope", side)
        violation = _violations(q,limits)
        if side == arm:
            outside = [{"joint_index":i+1, "observed_rad":v, "low_rad":lo, "high_rad":hi,
                        "excess_rad":max(lo-v,v-hi)}
                       for i,(v,lo,hi) in enumerate(zip(q,plan["joint_envelope"]["low_rad"],
                                                       plan["joint_envelope"]["high_rad"]))
                       if not lo <= v <= hi]
        strict[side] = not violation
        results[side] = {"controller_flange_pose": copy.deepcopy(state["pose_m_rad"]),
            "model_flange_transform": model_pose, "controller_displacement_m":raw_m,
            "controller_rotation_rad":raw_r, "model_displacement_m":model_m,
            "model_rotation_rad":model_r, "strict_nominal":not violation,
            "nominal_violations":violation, "within_feedback_tolerance":True}
    target_m,target_r = _error(results[arm]["model_flange_transform"],plan["model_target_flange_transform"])
    if phase == "pre_dispatch":
        _need(target_m <= RECOVERY["target_displacement_m"], "model_endpoint_displacement")
        _need(max(abs(a-b) for a,b in zip(states[arm]["joints_rad"],plan["encoded_target_joints_rad"])) <= STARTUP_RECOVERY["joint_excursion_rad"], "recovery_excursion_limit")
    joint_error=max(abs(a-b) for a,b in zip(states[arm]["joints_rad"],plan["encoded_target_joints_rad"]))
    target_observed=(phase in ("active", "settling") and mode_confirmed and states[arm]["arm_status"]["mode_feedback"] == 1
        and states[arm]["arm_status"]["motion_status"] == 0 and joint_error <= MONITOR_RAD
        and target_m <= RECOVERY["fk_position_tolerance_m"] and target_r <= RECOVERY["fk_orientation_tolerance_rad"])
    return {"sample_id":sample["sample_id"], "origin_sha256":plan["origin_sha256"], "plan_sha256":claimed,
        "within_initialization_envelope":True, "arms":results, "strict_nominal":strict,
        "spatial_admission_mode": plan["spatial_admission_mode"],
        "absolute_workspace_checked":not visual, "metric_clearance_checked":not visual,
        "metric_collision_checked":False,
        "within_feedback_tolerance":True, "model_target_error":{"position_m":target_m,"rotation_rad":target_r,"joint_rad":joint_error},
        "tracking":{"within_nominal_band":not outside, "outside_nominal_band":outside,
                    "max_excess_rad":max((row["excess_rad"] for row in outside),default=0.),
                    "cumulative_time_checked":False},
        "selected_target_observed":target_observed, "postsend_and_stability_verified":False,
        "motion_permitted":False,"physical_stop_verified":None,"qualification_granted":False}
