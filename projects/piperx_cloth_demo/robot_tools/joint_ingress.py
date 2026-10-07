"""Pure, source-bound observation bands after a completed first joint target.

This checks consistency of host/device evidence, not its physical authenticity.
The live adapter must match each source to its still-current target cache. No
source grants an action, load/hold qualification, new target, or larger budget.
"""
import copy
import math

from . import arms
from .hold_transaction import joint_hold_frames, _frame_matches


SIDES = ("left", "right")
BAND_RAD = .003
SCHEMA = "piper_joint_initialization_ingress_v1"
SOURCE_SCHEMA = "piper_pair_joint_initialization_receipt_v1"


def resolve_initialization_ingress(initialization_sources, *, identity, origin, current,
                                   effective_joint_limits_rad, cached_target=None):
    """Return frozen per-side feedback limits, leaving nominal limits untouched.

Only a same-owner/connection completed seed/recovery targeting J2's exact lower
or J3's exact upper boundary permits that joint's .003-radian observation band.
Every sample still needs the planner's existing freshness, motion and original
anchor guards. Missing sources return the original strict nominal intervals.
"""
    # Lazy import avoids a cycle: joint_path calls this helper after validating
    # its model, source-resolved limits and current dual-arm sample.
    from .joint_path import (_input_errors, _need, _num, _vec, _text, _hash, _identity, _enum,
                             encode_joint_target, evidence_sha256)

    @_input_errors
    def resolve():
        selected = _identity(identity)["arm"]
        _need(type(effective_joint_limits_rad) is dict and set(effective_joint_limits_rad) == set(SIDES),
              "ingress_limits_schema")
        limits = {}
        for side in SIDES:
            rows = effective_joint_limits_rad[side]
            _need(type(rows) is list and len(rows) == 6, "ingress_limits_schema", side)
            limits[side] = [_vec(row, 2, "ingress joint bounds") for row in rows]
            _need(all(lo < hi for lo, hi in limits[side]), "ingress_limits_schema", side)
        if initialization_sources is None:
            initialization_sources_checked = dict.fromkeys(SIDES)
        else:
            _need(type(initialization_sources) is dict and set(initialization_sources) == set(SIDES),
                  "initialization_sources_schema")
            initialization_sources_checked = initialization_sources
        feedback_limits = copy.deepcopy(limits)
        evidence = dict.fromkeys(SIDES)
        for side, source in initialization_sources_checked.items():
            if source is None:
                continue
            _need(type(source) is dict and source.get("schema") == SOURCE_SCHEMA, "initialization_source_schema", side)
            old_identity = _identity(source["identity"])
            _need(old_identity["arm"] == side and all(old_identity[key] == identity[key] for key in
                  ("run_id", "owner", "epoch", "connection_id", "model", "firmware_profile")),
                  "initialization_source_identity", side)
            event_id = _text(source["event_id"], "initialization event")
            _need(old_identity["worker_id"] == event_id, "initialization_source_event", side)
            _hash(source["plan_sha256"])
            _need(source["purpose"] in ("startup_j2_j3", "seed_current"), "initialization_source_purpose", side)
            _need(source.get("ordinary_motion_authorized") is False and source.get("physical_stop_verified") is None,
                  "initialization_source_scope", side)
            _need(source.get("effective_joint_limits_rad") == effective_joint_limits_rad,
                  "initialization_source_limits_changed", side)
            target = _vec(source["encoded_target_joints_rad"], 6, "initialization encoded target")
            raw = source["target_raw"]
            _need(type(raw) is list and len(raw) == 6 and all(type(v) is int for v in raw),
                  "initialization_source_target", side)
            _need(encode_joint_target(target)[0] == raw and target == [math.radians(v/1000) for v in raw]
                  and all(lo <= v <= hi for v,(lo,hi) in zip(target, limits[side])),
                  "initialization_source_target", side)
            receipts = source["frame_receipts"]
            frames = joint_hold_frames(raw)
            _need(type(receipts) is list and len(receipts) == 4, "initialization_source_partial", side)
            last = 0.
            for receipt, frame in zip(receipts, frames):
                _need(type(receipt) is dict and set(receipt) == {"frame", "outcome", "returned_at"},
                      "initialization_source_frame", side)
                at = _num(receipt["returned_at"], "initialization return time")
                _need(receipt["outcome"] == "returned" and _frame_matches(receipt["frame"], frame)
                      and at >= last and at > 0, "initialization_source_frame", side)
                last = at
            expected_cache = {"event_id":event_id, "identity":old_identity, "target_raw":raw,
                              "frame_receipts":receipts}
            if side == selected:
                _need(cached_target == expected_cache, "initialization_source_cache_changed", side)
            original, completed = source["origin"], source["completion_sample"]
            for sample in (original, completed):
                _need(type(sample) is dict and sample.get("identity") == old_identity,
                      "initialization_source_sample_identity", side)
                _text(sample["sample_id"], "initialization sample")
            before_at = _num(original["captured_at"], "initialization origin time")
            captured = _num(completed["captured_at"], "initialization completion sample time")
            ended = _num(source["completed_at"], "initialization completion time")
            _need(0 < before_at < receipts[0]["returned_at"] <= last
                  and captured-last >= 3. and ended >= captured,
                  "initialization_source_time_order", side)
            states = completed["arms"]
            _need(type(states) is dict and set(states) == set(SIDES), "initialization_source_completion", side)
            for name in SIDES:
                state = states[name]
                _need(arms.control_health(state, now_s=captured, allowed_control_modes=(1,), require_enabled=False)["healthy"]
                      and _enum(state["arm_status"].get("motion_status"), (0,))
                      and _enum(state["arm_status"].get("teach_status"), (0,))
                      and all(state["drivers"][str(i)]["foc_status"]["driver_enable_status"] is True for i in range(1,7)),
                      "initialization_source_completion", name)
                for part in arms.PARTS+arms.DRIVERS+("gripper",):
                    stamp = _num(state["fragment_timestamps_s"][part], "completion fragment")
                    _need(last < stamp <= captured and 0 <= captured-stamp <= .05,
                          "initialization_source_completion_time", name)
            finished_q = _vec(states[side]["joints_rad"], 6, "completion joints")
            original_q = _vec(original["arms"][side]["joints_rad"], 6, "initialization origin joints")
            _need(_enum(states[side]["arm_status"].get("mode_feedback"), (1,))
                  and max(abs(a-b) for a,b in zip(finished_q,target)) <= BAND_RAD,
                  "initialization_source_not_arrived", side)
            strict = all(lo <= q <= hi for q,(lo,hi) in zip(finished_q, limits[side]))
            _need(type(source["strict_nominal"]) is bool and source["strict_nominal"] == strict
                  and source["within_feedback_tolerance"] is True,
                  "initialization_source_verdict", side)
            violations = []
            for i, (q, sent, (lo, hi)) in enumerate(zip(original_q,target,limits[side])):
                if not lo <= q <= hi:
                    _need(((i == 1 and q < lo and sent == lo) or (i == 2 and q > hi and sent == hi))
                          and abs(sent-q) <= .10, "initialization_source_original_boundary", side)
                    violations.append(i)
                else:
                    _need(abs(q-sent) <= BAND_RAD, "initialization_source_original_target", side)
            _need(bool(violations) == (source["purpose"] == "startup_j2_j3"),
                  "initialization_source_purpose", side)
            eligible = [i for i in (1,2) if target[i] == limits[side][i][0 if i == 1 else 1]]
            for sample in (origin, current):
                _need(type(sample) is dict and sample.get("identity") == identity,
                      "ingress_sample_identity", side)
                at = _num(sample["captured_at"], "ingress sample time")
                _need(at > ended, "ingress_sample_precedes_completion", side)
                state = sample["arms"][side]
                q = _vec(state["joints_rad"], 6, "ingress joints")
                _need(_enum(state["arm_status"].get("mode_feedback"), (1,)), "ingress_mode_changed", side)
                for part in arms.PARTS+arms.DRIVERS+("gripper",):
                    _need(_num(state["fragment_timestamps_s"][part], "ingress fragment") > ended,
                          "ingress_feedback_precedes_completion", side)
                for i,(value,goal,(lo,hi)) in enumerate(zip(q,target,limits[side])):
                    _need(abs(value-goal) <= BAND_RAD, "ingress_left_initial_target_band", side)
                    allowed = (i in eligible and ((i == 1 and lo-BAND_RAD <= value <= hi)
                                                  or (i == 2 and lo <= value <= hi+BAND_RAD)))
                    _need(lo <= value <= hi or allowed, "ingress_nonboundary_violation", side)
            for i in eligible:
                feedback_limits[side][i][0 if i == 1 else 1] += -BAND_RAD if i == 1 else BAND_RAD
            evidence[side] = {"event_id":event_id, "identity":copy.deepcopy(old_identity),
                "source_sha256":evidence_sha256(source), "completion_sample_id":completed["sample_id"],
                "completed_at":ended, "target_raw":raw[:], "joint_indices":[i+1 for i in eligible],
                "completion_joints_rad":finished_q, "strict_nominal":strict,
                "within_feedback_tolerance":True,
                "nominal_limits_changed":False, "ordinary_motion_authorized":False}
        return {"schema":SCHEMA, "feedback_limits_rad":feedback_limits, "evidence":evidence,
                "observation_band_rad":BAND_RAD, "nominal_limits_changed":False,
                "motion_permitted":False, "source_truth_authenticated":False}

    return resolve()
