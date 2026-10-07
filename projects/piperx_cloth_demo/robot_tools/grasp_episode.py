"""Pure, per-arm grasp evidence bookkeeping; never a dispatch permission.

TRUST BOUNDARY: only a host that has resolved durable probe events, measured
trace artifacts, its current scene and an adapter-owned retention contract may
call this module. Dictionaries, hashes and semantic labels are not signatures.
This module checks their structure, identity, chronology and numeric consistency;
it cannot authenticate images, reclassify a raw trace, or certify hardware.
Never expose ``evidence`` as an untrusted caller-supplied tool argument.

Public API uses JSON dictionaries and returns deep copies. ``new_episode`` makes
an empty state. ``apply_event`` consumes {event_id, kind, identity, evidence};
the identity is {episode_id, arm, run_id, owner, epoch, object_id}. Identical event
replay returns the current state without renewing evidence. Different payloads
under an existing event ID fail. The caller must persist each result using its
durable revision/CAS transaction before relying on it. This module provides no
concurrency control and does not own or reset the task/dispatch budget.

Supported event kinds and evidence keys:
* record_candidate: probe, measurement
* retain_static / renew_static: scene, visual, measurement, retention_contract
* begin_release: action_event_id, target_width_m, scene, measurement, support_visual
* finish_release: action_event_id, measurement, actual_opening_increase_m,
  arrival_confirmed, target_calls_sent, passive_arm_commands_sent
* confirm_release: action_event_id, scene, visual, measurement
* invalidate: reason, source_event_id, send_status

The host must obtain all machine summary fields from a checked complete trace,
not from a model report. RGB semantics remain separately labelled observations.
Static retention is confined to the original support and original drift anchor.
Retention contracts reference a host-resolved adapter issuance event in
``source`` (event_id, artifact_sha256, trace_id, trace_sha256). A new contract
must cite its issuance against the current measured trace. These references
and changed hashes are consistency checks, not signatures or authenticity proof.
Dedicated host-resolved RGB loaded events are implemented by loaded_episode;
no text scope, capability boolean or static receipt alone can enable extraction.
Opening records remain unresolved until new RGB observes finger separation and
independent support alongside a new adapter stability trace. Even the resolved
episode does not prove physical stopping, correct placement or object stability.
Version 1 records remain historical; their mechanical ``released`` state is not
a version 2 separation confirmation and is never silently upgraded.
"""
import copy
import hashlib
import json
import math


SCHEMA_VERSION = 2
STATUSES = frozenset(("empty", "contact_candidate", "retained_static", "proof_pending", "loaded_pending_visual",
                      "retained_local", "release_pending", "release_opened", "released", "invalid"))
# Existing software observation policy, not physical qualification or force bounds.
OBSERVATION_POLICY = {"stable_s": 3.0, "minimum_feedback_advances": 20,
                      "feedback_age_s": 0.1, "rgb_age_s": 30.0, "rgb_skew_s": 0.15,
                      "joint_rad": 0.003, "position_m": 0.0005,
                      "rotation_rad": 0.003, "jaw_m": 0.0005}
_IDENTITY = frozenset(("episode_id", "arm", "run_id", "owner", "epoch", "object_id"))
_SPANS = frozenset(("joint_rad", "position_m", "rotation_rad", "jaw_m"))
_LOADED = frozenset(("begin_proof", "extract_segment", "insert_segment", "grip_test",
                     "loaded_hold", "transport_loaded"))
_FORBIDDEN = frozenset(("grasp_verified", "contact_support_verified", "physical_stop_verified"))


class GraspEpisodeError(RuntimeError):
    """A rejected pure transition; ``code``/``missing`` are stable diagnostics."""

    def __init__(self, code, message=None, *, missing=()):
        self.code, self.missing = code, tuple(missing)
        super().__init__(message or code)


def _fail(code, message=None, *, missing=()):
    raise GraspEpisodeError(code, message, missing=missing)


def _json(value):
    def check(item, depth=0):
        if depth > 32:
            _fail("invalid_json", "JSON nesting exceeds 32")
        if item is None or type(item) in (str, bool):
            return
        if type(item) in (int, float):
            _number(item, "JSON number", nonnegative=False)
            return
        if type(item) is list:
            for child in item:
                check(child, depth + 1)
            return
        if type(item) is dict and all(type(key) is str for key in item):
            for child in item.values():
                check(child, depth + 1)
            return
        _fail("invalid_json", "Only finite JSON data is supported")
    check(value)
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    if len(encoded.encode()) > 1_048_576:
        _fail("invalid_json", "Episode input exceeds one MiB")
    return encoded


def _keys(value, required, label, optional=()):
    if type(value) is not dict or set(value) - set(required) - set(optional) or set(required) - set(value):
        _fail("invalid_schema", label + " has missing or unexpected fields")
    return value


def _id(value, label):
    if (type(value) is not str or not 1 <= len(value) <= 128 or value != value.strip()
            or any(ord(char) < 32 or ord(char) == 127 for char in value)):
        _fail("invalid_identifier", label)
    return value


def _hash(value, label):
    if type(value) is not str or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        _fail("invalid_hash", label)
    return value


def _number(value, label, *, nonnegative=True):
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and (not nonnegative or value >= 0)
    except OverflowError:
        valid = False
    if not valid:
        _fail("invalid_number", label + " must be finite and not a boolean")
    return value


def _identity(value, expected=None):
    _keys(value, _IDENTITY, "identity")
    for key, item in value.items():
        _id(item, key)
    if value["arm"] not in ("left", "right"):
        _fail("invalid_identity", "arm must be left or right")
    if expected is not None and value != expected:
        _fail("identity_mismatch", "Episode, arm, object, run, owner and epoch must all match")


def _reject_assertions(value):
    if type(value) is dict:
        if _FORBIDDEN & set(value):
            _fail("caller_assertion_forbidden", "Evidence must not supply grasp/support/stop assertions")
        for child in value.values():
            _reject_assertions(child)
    elif type(value) is list:
        for child in value:
            _reject_assertions(child)


def _pose(value, label):
    _keys(value, ("joints_rad", "pose_m_rad", "width_m"), label)
    for key in ("joints_rad", "pose_m_rad"):
        if type(value[key]) is not list or len(value[key]) != 6:
            _fail("invalid_schema", label + "." + key + " requires six measured values")
        for item in value[key]:
            _number(item, key, nonnegative=False)
    if not 0 <= _number(value["width_m"], "width_m") <= 0.070:
        _fail("measurement_out_of_range", "Measured jaw width outside existing 0..70 mm range")


def _rotation(a, b):
    def quat(p):
        cr, cp, cy = (math.cos(x/2) for x in p[3:])
        sr, sp, sy = (math.sin(x/2) for x in p[3:])
        return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    qa, qb = quat(a), quat(b)
    norm = math.sqrt(sum(x*x for x in qa)*sum(x*x for x in qb))
    return 2*math.acos(min(1.0, abs(sum(x*y for x, y in zip(qa, qb)))/norm))


def _measurement(state, data, now, *, release=False, historical=False):
    _keys(data, ("identity", "trace_id", "trace_sha256", "probe_event_id", "started_at", "ended_at",
                 "sample_count", "feedback_advances", "health", "mode", "anchor", "observed",
                 "spans", "anchor_deviation"), "measurement")
    _identity(data["identity"], state["identity"])
    _id(data["trace_id"], "trace_id")
    _hash(data["trace_sha256"], "trace_sha256")
    _id(data["probe_event_id"], "probe_event_id")
    start, end = (_number(data[key], key) for key in ("started_at", "ended_at"))
    if (end - start < OBSERVATION_POLICY["stable_s"] or start < state["created_at"]
            or now < end or (not historical and now-end > OBSERVATION_POLICY["feedback_age_s"])):
        _fail("incomplete_or_stale_trace", "Need a complete three-second trace ending at fresh feedback")
    for key, minimum in (("feedback_advances", 20), ("sample_count", 21)):
        if type(data[key]) is not int or data[key] < minimum:
            _fail("incomplete_trace", key)
    if data["feedback_advances"] >= data["sample_count"]:
        _fail("inconsistent_trace", "Advances cannot exceed the measured sample intervals")
    if data["health"] != "healthy" or data["mode"] != "stationary":
        _fail("unhealthy_trace")
    _pose(data["anchor"], "anchor")
    _pose(data["observed"], "observed")
    if state["probe_ref"] is not None and data["probe_event_id"] != state["probe_ref"]["event_id"]:
        _fail("probe_reference_mismatch")
    from .loaded_episode import body_anchor
    if state["original_anchor"] is not None and data["anchor"] != body_anchor(state):
        _fail("anchor_rebase_forbidden", "Renewal must retain the original measured anchor")
    for key in ("spans", "anchor_deviation"):
        _keys(data[key], _SPANS, key)
        for name, value in data[key].items():
            value = _number(value, key + "." + name)
            if value > OBSERVATION_POLICY[name] and not (release and key == "anchor_deviation" and name == "jaw_m"):
                _fail("measured_drift", key + "." + name)
    anchor, observed = data["anchor"], data["observed"]
    actual = {"joint_rad": max(abs(a-b) for a, b in zip(anchor["joints_rad"], observed["joints_rad"])),
              "position_m": math.dist(anchor["pose_m_rad"][:3], observed["pose_m_rad"][:3]),
              "rotation_rad": _rotation(anchor["pose_m_rad"], observed["pose_m_rad"]),
              "jaw_m": abs(anchor["width_m"]-observed["width_m"])}
    for name, value in actual.items():
        if value > data["anchor_deviation"][name] + 1e-12:
            _fail("inconsistent_trace", "Reported anchor deviation omits measured " + name)
    if state["measurement"] is not None and end < state["measurement"]["ended_at"]:
        _fail("stale_trace", "Trace time regressed")
    return copy.deepcopy(data)


def _scene(state, scene, now, *, after=None):
    _keys(scene, ("identity", "observation_id", "captured_at", "frames"), "scene")
    _identity(scene["identity"], state["identity"])
    _id(scene["observation_id"], "observation_id")
    at = _number(scene["captured_at"], "captured_at")
    floor = state["probe_ref"]["completed_at"]
    if after is not None:
        floor = max(floor, after)
    if state["scene"] is not None:
        floor = max(floor, state["scene"]["captured_at"])
        if scene["observation_id"] == state["scene"]["observation_id"]:
            _fail("stale_scene", "Each transition needs a new host observation")
    if (not floor <= at <= now or now-at > OBSERVATION_POLICY["rgb_age_s"]
            or (after is not None and at <= after)):
        _fail("stale_scene", "RGB must follow the probe and remain current")
    if type(scene["frames"]) is not list or not scene["frames"]:
        _fail("invalid_schema", "Scene requires source frame references")
    prior = {item["camera_id"]: item for item in state["scene"]["frames"]} if state["scene"] else {}
    cameras, stamps = set(), []
    for frame in scene["frames"]:
        _keys(frame, ("camera_id", "capture_id", "frame_number", "host_received_at", "artifact_sha256"), "frame")
        for key in ("camera_id", "capture_id"):
            _id(frame[key], key)
        if frame["camera_id"] in cameras:
            _fail("duplicate_camera")
        cameras.add(frame["camera_id"])
        if type(frame["frame_number"]) is not int or frame["frame_number"] < 0:
            _fail("invalid_frame_number")
        _hash(frame["artifact_sha256"], "frame artifact_sha256")
        stamp = _number(frame["host_received_at"], "host_received_at")
        if (not floor <= stamp <= now or now-stamp > OBSERVATION_POLICY["rgb_age_s"]
                or (after is not None and stamp <= after)):
            _fail("stale_scene", "All used frames must follow the probe and remain current")
        old = prior.get(frame["camera_id"])
        if old and (frame["frame_number"] <= old["frame_number"] or stamp <= old["host_received_at"]
                    or frame["capture_id"] == old["capture_id"]):
            _fail("stale_scene", "Frame identity/time must advance")
        stamps.append(stamp)
    if prior and cameras != set(prior):
        _fail("scene_binding_changed", "Changing cameras needs a new host evidence contract")
    if max(stamps)-min(stamps) > OBSERVATION_POLICY["rgb_skew_s"] or not min(stamps) <= at <= max(stamps):
        _fail("scene_skew")
    return copy.deepcopy(scene)


def _visual(state, visual, scene, *, purpose="retention"):
    _keys(visual, ("identity", "evidence_id", "observation_id", "source", "producer_ref",
                   "artifact_sha256", "object_relation", "support_relation"), "visual")
    _identity(visual["identity"], state["identity"])
    for key in ("evidence_id", "producer_ref"):
        _id(visual[key], key)
    _hash(visual["artifact_sha256"], "visual artifact_sha256")
    if visual["observation_id"] != scene["observation_id"]:
        _fail("visual_scene_mismatch")
    relations = {"retention": ("between_fingers", "original_support_present"),
                 "release_support": ("separation_not_assessed", "independent_support_present"),
                 "release_confirmation": ("object_clear_of_fingers", "independent_support_present")}
    if (visual["source"] != "rgb_semantic_observation"
            or (visual["object_relation"], visual["support_relation"]) != relations[purpose]):
        code = "missing_static_visual_evidence" if purpose == "retention" else "missing_release_visual_evidence"
        _fail(code, "Need the separately sourced RGB relations for " + purpose)
    return copy.deepcopy(visual)


def _retention(state, contract, now, *, measurement=None):
    if contract is None:
        _fail("missing_retention_contract", missing=("adapter_existing_target_retention_contract",))
    _keys(contract, ("identity", "contract_id", "artifact_sha256", "adapter_id", "adapter_code_sha256",
                     "controller_id", "issued_at", "valid_until", "mode", "probe_event_id",
                     "requested_width_m", "failure_behavior", "source"), "retention_contract")
    _identity(contract["identity"], state["identity"])
    for key in ("contract_id", "adapter_id", "controller_id"):
        _id(contract[key], key)
    for key in ("artifact_sha256", "adapter_code_sha256"):
        _hash(contract[key], key)
    source = contract["source"]
    _keys(source, ("event_id", "artifact_sha256", "trace_id", "trace_sha256"), "retention source")
    for key in ("event_id", "trace_id"):
        _id(source[key], "retention source." + key)
    for key in ("artifact_sha256", "trace_sha256"):
        _hash(source[key], "retention source." + key)
    if (not state["created_at"] <= _number(contract["issued_at"], "issued_at") <= now
            or _number(contract["valid_until"], "valid_until") <= now):
        _fail("expired_retention_contract")
    if (contract["mode"] != "existing_target_monitored" or contract["failure_behavior"] != "latch_no_new_targets"
            or contract["probe_event_id"] != state["probe_ref"]["event_id"]
            or _number(contract["requested_width_m"], "requested_width_m") != state["probe_ref"]["requested_width_m"]):
        _fail("retention_contract_mismatch", "Static adoption cannot silently send or replace a target")
    old = state["retention_contract"]
    if old is not None:
        for key in ("adapter_id", "adapter_code_sha256", "controller_id", "mode", "probe_event_id", "requested_width_m"):
            if contract[key] != old[key]:
                _fail("retention_binding_changed", key)
        if contract["contract_id"] == old["contract_id"]:
            if contract != old:
                _fail("retention_contract_mutated", "An issued contract is immutable, including its expiry")
            return copy.deepcopy(contract)
        if (contract["artifact_sha256"] == old["artifact_sha256"] or contract["issued_at"] <= old["issued_at"]
                or source["event_id"] == old["source"]["event_id"]
                or source["artifact_sha256"] == old["source"]["artifact_sha256"]):
            _fail("new_retention_issuance_required", "A new contract needs a later, separately sourced issuance event")
    if (measurement is None or source["trace_id"] != measurement["trace_id"]
            or source["trace_sha256"] != measurement["trace_sha256"]
            or contract["issued_at"] < measurement["ended_at"]):
        _fail("retention_source_mismatch", "Adapter issuance must reference this measured trace after it completes")
    return copy.deepcopy(contract)


def _opening(state, opening):
    _keys(opening, ("action_event_id", "target_width_m", "before_width_m", "observed_width_m",
                    "trace_id", "trace_sha256", "started_at", "finished_at", "recorded_at",
                    "actual_opening_increase_m"), "release_opening")
    for key in ("action_event_id", "trace_id"):
        _id(opening[key], key)
    _hash(opening["trace_sha256"], "release trace_sha256")
    for key in ("target_width_m", "before_width_m", "observed_width_m", "started_at", "finished_at",
                "recorded_at", "actual_opening_increase_m"):
        _number(opening[key], key)
    if (not state["created_at"] < opening["started_at"] < opening["finished_at"] <= opening["recorded_at"]
            or opening["recorded_at"] > state["last_event_at"]
            or opening["finished_at"] - opening["started_at"] < OBSERVATION_POLICY["stable_s"]
            or not 0 <= opening["target_width_m"] <= 0.055
            or not 0 <= opening["before_width_m"] <= 0.070
            or not 0 <= opening["observed_width_m"] <= 0.070
            or not 0 < opening["target_width_m"] - opening["before_width_m"] <= 0.005 + 1e-12
            or not OBSERVATION_POLICY["jaw_m"] < opening["actual_opening_increase_m"]
            <= opening["observed_width_m"] - opening["before_width_m"] + 1e-12
            or abs(opening["observed_width_m"] - opening["target_width_m"]) > 0.002
            or state["residual_target"] != {"event_id": opening["action_event_id"],
                                             "requested_width_m": opening["target_width_m"]}):
        _fail("invalid_state", "Opening reference no longer matches the measured release result")


def _release_measurement(state, data, now, *, after=None):
    measurement = _measurement(state, data, now, release=True)
    previous = state["measurement"]
    if (measurement["ended_at"] <= previous["ended_at"] or measurement["trace_id"] == previous["trace_id"]
            or (after is not None and measurement["started_at"] <= after)):
        _fail("stale_trace", "Release needs a new measured window after the latest opening")
    opening = state["release_opening"]
    if opening is not None and abs(measurement["observed"]["width_m"] - opening["observed_width_m"]) > OBSERVATION_POLICY["jaw_m"]:
        _fail("release_jaw_drift", "The last opening remains the uncommanded jaw drift anchor")
    return measurement


def _state(state):
    _keys(state, ("schema_version", "identity", "revision", "status", "created_at", "deadline_at", "last_event_at",
                  "probe_ref", "measurement", "scene", "visual_evidence", "original_anchor", "retention_contract",
                  "retention_expires_at", "retained_scope", "pending", "target_may_remain_active", "residual_target",
                  "release_opening", "release_confirmation", "fault", "events", "physical_stop_verified",
                  "object_progress_measurement", "dispatch_authorized"), "state", ("loaded",))
    if (type(state["schema_version"]) is not int or state["schema_version"] != SCHEMA_VERSION
            or type(state["status"]) is not str or state["status"] not in STATUSES):
        _fail("invalid_state")
    _json(state)
    _identity(state.get("identity"))
    if type(state.get("revision")) is not int or state["revision"] < 0:
        _fail("invalid_state", "Invalid revision")
    for key in ("created_at", "deadline_at", "last_event_at"):
        _number(state[key], key)
    if (state["created_at"] >= state["deadline_at"] or state["last_event_at"] < state["created_at"]
            or state["physical_stop_verified"] is not None or state["object_progress_measurement"] is not None
            or state["dispatch_authorized"] is not False):
        _fail("invalid_state", "Invalid time or unsupported physical claim")
    if state["status"] != "invalid" and state["fault"] is not None:
        _fail("invalid_state", "A fault-bearing episode cannot be restored as a live state")
    if type(state["events"]) is not dict:
        _fail("invalid_state", "Invalid event index")
    for event_id, item in state["events"].items():
        _id(event_id, "event_id")
        _keys(item, ("payload_sha256", "revision", "kind"), "event index")
        _hash(item["payload_sha256"], "payload_sha256")
        _id(item["kind"], "kind")
        if type(item["revision"]) is not int or not 1 <= item["revision"] <= state["revision"]:
            _fail("invalid_state", "Invalid event revision")
    if state["probe_ref"] is not None:
        if state["target_may_remain_active"] is not True or state["original_anchor"] is None:
            _fail("invalid_state", "A prior target/anchor cannot silently disappear")
        _identity(state["probe_ref"].get("identity"), state["identity"])
        _pose(state["original_anchor"], "original_anchor")
    elif state["status"] not in ("empty", "invalid"):
        _fail("invalid_state", "Missing probe reference")
    if state["retention_expires_at"] is not None:
        expiry = _number(state["retention_expires_at"], "retention_expires_at")
        if not state["created_at"] < expiry <= state["deadline_at"]:
            _fail("invalid_state", "Retention cannot outlive the episode budget")
    if state["status"] == "retained_static":
        if any(state[key] is None for key in ("measurement", "scene", "visual_evidence", "retention_contract",
                                               "retention_expires_at", "retained_scope")):
            _fail("invalid_state", "Static retention lacks its source evidence")
        if state["retained_scope"] != {"kind": "static_original_support", "arm": state["identity"]["arm"],
                                        "object_id": state["identity"]["object_id"], "loaded": False}:
            _fail("invalid_state", "Unsupported retention scope")
    if (state["status"] == "release_pending") != (state["pending"] is not None) and state["status"] != "invalid":
        _fail("invalid_state", "Pending release state is inconsistent")
    opening, confirmation = state["release_opening"], state["release_confirmation"]
    if opening is not None:
        _opening(state, opening)
        if state["status"] not in ("release_pending", "release_opened", "released", "invalid"):
            _fail("invalid_state", "A measured opening cannot restore earlier grasp qualification")
    if state["status"] in ("release_opened", "released"):
        if (opening is None or state["retained_scope"] is not None or state["retention_expires_at"] is not None):
            _fail("invalid_state", "An opened episode needs its measured reference without retained scope")
    if confirmation is not None:
        _keys(confirmation, ("action_event_id", "observation_id", "trace_id", "trace_sha256", "confirmed_at"),
              "release_confirmation")
        for key in ("action_event_id", "observation_id", "trace_id"):
            _id(confirmation[key], key)
        _hash(confirmation["trace_sha256"], "confirmation trace_sha256")
        at = _number(confirmation["confirmed_at"], "confirmed_at")
        if (opening is None or state["status"] not in ("released", "invalid")
                or not opening["recorded_at"] <= at <= state["last_event_at"] or at >= state["deadline_at"]
                or state["scene"] is None or state["measurement"] is None
                or confirmation["action_event_id"] != opening["action_event_id"]
                or confirmation["observation_id"] != state["scene"]["observation_id"]
                or confirmation["trace_id"] != state["measurement"]["trace_id"]
                or confirmation["trace_sha256"] != state["measurement"]["trace_sha256"]):
            _fail("invalid_state", "Separation confirmation lacks its exact latest opening/scene/trace")
        shadow = {**state, "scene": None}
        _scene(shadow, state["scene"], at, after=opening["finished_at"])
        _visual(state, state["visual_evidence"], state["scene"], purpose="release_confirmation")
        measurement = _measurement(state, state["measurement"], at, release=True)
        if (measurement["started_at"] <= opening["finished_at"] or measurement["trace_id"] == opening["trace_id"]
                or abs(measurement["observed"]["width_m"] - opening["observed_width_m"]) > OBSERVATION_POLICY["jaw_m"]
                or not any(item["kind"] == "confirm_release" for item in state["events"].values())):
            _fail("invalid_state", "Separation confirmation needs a new anchored trace and recorded event")
    elif state["status"] == "released":
        _fail("invalid_state", "Mechanical opening alone is not a resolved release")
    from .loaded_episode import validate as validate_loaded
    validate_loaded(state)
    return copy.deepcopy(state)


def new_episode(*, episode_id, arm, run_id, owner, epoch, object_id, created_at, deadline_at):
    """Make an immutable-budget, per-arm episode; no physical evidence yet."""
    identity = dict(episode_id=episode_id, arm=arm, run_id=run_id, owner=owner, epoch=epoch, object_id=object_id)
    _identity(identity)
    created_at, deadline_at = _number(created_at, "created_at"), _number(deadline_at, "deadline_at")
    if deadline_at <= created_at:
        _fail("invalid_deadline")
    return {"schema_version": SCHEMA_VERSION, "identity": identity, "revision": 0, "status": "empty",
            "created_at": created_at, "deadline_at": deadline_at, "last_event_at": created_at,
            "probe_ref": None, "measurement": None, "scene": None, "visual_evidence": None,
            "original_anchor": None, "retention_contract": None, "retention_expires_at": None,
            "retained_scope": None, "pending": None, "target_may_remain_active": None,
            "residual_target": None, "release_opening": None, "release_confirmation": None,
            "fault": None, "events": {}, "physical_stop_verified": None,
            "object_progress_measurement": None, "dispatch_authorized": False}


def is_resolved_release(state):
    """Recognize v2 separation bookkeeping, never physical stop or task success.

    Historical v1 ``released`` meant only mechanical opening. It remains
    readable but cannot make a new grasp or clean owner detach eligible.
    """
    try:
        return _state(state)["status"] == "released"
    except (GraspEpisodeError, KeyError, TypeError, AttributeError):
        return False


def _live(state, now):
    now = _number(now, "now")
    if now < state["last_event_at"]:
        _fail("clock_regressed")
    if state["status"] in ("invalid", "released"):
        _fail("terminal_episode", state["status"])
    expiry = state["retention_expires_at"]
    if now >= state["deadline_at"] or (expiry is not None and now >= expiry):
        _fail("episode_expired", "Expired evidence cannot be renewed or used for a new target")


def _loaded_missing(request):
    missing = []
    for name in ("adapter_bounded_contact_contract", "adapter_applicable_hold_contract",
                 "bounded_response_measurement_contract"):
        try:
            reference = request.get(name)
            _keys(reference, ("contract_id", "artifact_sha256"), name)
            _id(reference["contract_id"], name + ".contract_id")
            _hash(reference["artifact_sha256"], name + ".artifact_sha256")
        except GraspEpisodeError:
            missing.append(name)
    return missing + ["loaded_transition_not_implemented"]


def apply_event(state, event, *, now):
    """Validate one host-resolved evidence event; never mutate inputs or send.

    A failure leaves the input untouched. The host must latch actual hardware
    faults independently; rejection is not itself a physical stop. The explicit
    invalidate event preserves pending and residual-target facts permanently.
    """
    result = _state(state)
    _keys(event, ("event_id", "kind", "identity", "evidence"), "event")
    _id(event["event_id"], "event_id")
    _id(event["kind"], "kind")
    _identity(event["identity"], result["identity"])
    if type(event["evidence"]) is not dict:
        _fail("invalid_schema", "evidence must be a JSON object")
    _reject_assertions(event)
    encoded = _json(event)
    digest = hashlib.sha256(encoded.encode()).hexdigest()
    now = _number(now, "now")
    old = result["events"].get(event["event_id"])
    if old is not None:
        if old["payload_sha256"] != digest:
            _fail("event_payload_changed")
        return result
    kind, evidence = event["kind"], event["evidence"]
    if kind == "invalidate":
        _keys(evidence, ("reason", "source_event_id", "send_status"), "invalidate evidence")
        _id(evidence["reason"], "reason")
        _id(evidence["source_event_id"], "source_event_id")
        if evidence["send_status"] not in ("partial", "unknown", "fault", "not_sent"):
            _fail("invalid_send_status")
        if result["status"] == "invalid":
            _fail("terminal_episode")
        if now < result["last_event_at"]:
            _fail("clock_regressed")
        result.update(status="invalid", fault={**copy.deepcopy(evidence), "at": now})
    else:
        if (kind == "finish_release" and result["status"] == "release_pending"
                or kind == "finish_loaded" and result["status"] == "proof_pending"):
            # Recording an already-issued action's measured result is permitted
            # after lease expiry. It cannot admit or renew another target.
            if now < result["last_event_at"]:
                _fail("clock_regressed")
        else:
            _live(result, now)
        if kind in _LOADED:
            _fail("loaded_transition_not_implemented", missing=_loaded_missing(evidence))
        if kind in ("begin_loaded", "finish_loaded", "confirm_loaded"):
            from .loaded_episode import apply as apply_loaded
            apply_loaded(result, kind, evidence, now)
        elif kind == "record_candidate":
            if result["status"] != "empty":
                _fail("illegal_transition", "A candidate can only initialize an empty episode")
            _keys(evidence, ("probe", "measurement"), "candidate evidence")
            probe = evidence["probe"]
            _keys(probe, ("identity", "event_id", "observation_id", "sent_at", "completed_at", "outcome",
                          "completion", "requested_width_m", "observed_width_m", "trace_sha256",
                          "conservative_closure_displacement_m", "target_may_remain_active"), "probe")
            _identity(probe["identity"], result["identity"])
            for key in ("event_id", "observation_id"):
                _id(probe[key], key)
            _hash(probe["trace_sha256"], "probe trace_sha256")
            sent, completed = _number(probe["sent_at"], "sent_at"), _number(probe["completed_at"], "completed_at")
            if not result["created_at"] <= sent < completed <= now:
                _fail("invalid_probe_time")
            target, width = _number(probe["requested_width_m"], "requested_width_m"), _number(probe["observed_width_m"], "observed_width_m")
            if (not 0 <= target <= 0.055 or not 0 <= width <= 0.070 or width-target <= 0.002
                    or not OBSERVATION_POLICY["jaw_m"] < _number(probe["conservative_closure_displacement_m"], "closure_displacement") <= 0.005 + 1e-12
                    or probe["outcome"] != "settled_contact_candidate" or probe["completion"] != "observation_only"
                    or probe["target_may_remain_active"] is not True):
                _fail("not_contact_candidate")
            result["probe_ref"] = copy.deepcopy(probe)
            # Recording a completed probe is historical bookkeeping, not
            # admission. Disk flush/receipt processing must not retimestamp
            # its original trace to make it look like current feedback.
            # retain_static and every scope admission still require a fresh
            # independently measured window.
            measurement = _measurement(result, evidence["measurement"], now, historical=True)
            if measurement["started_at"] <= sent:
                _fail("post_send_window_required", "The entire stable measurement window must follow this send")
            if (measurement["trace_sha256"] != probe["trace_sha256"] or measurement["ended_at"] < completed
                    or measurement["observed"]["width_m"] != width):
                _fail("probe_trace_mismatch")
            result.update(status="contact_candidate", measurement=measurement, original_anchor=copy.deepcopy(measurement["anchor"]),
                          target_may_remain_active=True,
                          residual_target={"event_id": probe["event_id"], "requested_width_m": target})
        elif kind in ("retain_static", "renew_static"):
            expected = "contact_candidate" if kind == "retain_static" else "retained_static"
            if result["status"] != expected:
                _fail("illegal_transition", kind + " requires " + expected)
            _keys(evidence, ("scene", "visual", "measurement", "retention_contract"), "retention evidence")
            scene = _scene(result, evidence["scene"], now)
            visual = _visual(result, evidence["visual"], scene)
            measurement = _measurement(result, evidence["measurement"], now)
            if (measurement["ended_at"] <= result["measurement"]["ended_at"]
                    or measurement["trace_id"] == result["measurement"]["trace_id"]):
                _fail("stale_trace", "Retention requires a new measured trace")
            contract = _retention(result, evidence["retention_contract"], now, measurement=measurement)
            result.update(status="retained_static", scene=scene, visual_evidence=visual, measurement=measurement,
                          retention_contract=contract,
                          retention_expires_at=min(result["deadline_at"], contract["valid_until"]),
                          retained_scope={"kind": "static_original_support", "arm": result["identity"]["arm"],
                                          "object_id": result["identity"]["object_id"], "loaded": False})
        elif kind == "begin_release":
            if result["status"] not in ("contact_candidate", "retained_static", "retained_local", "release_opened"):
                _fail("illegal_transition", "Release needs a current candidate, static retention or measured opening")
            _keys(evidence, ("action_event_id", "target_width_m", "scene", "measurement", "support_visual"), "release evidence")
            _id(evidence["action_event_id"], "action_event_id")
            opening = result["release_opening"]
            after = opening["finished_at"] if opening else None
            if opening is not None and evidence["action_event_id"] == opening["action_event_id"]:
                _fail("release_event_reused")
            scene = _scene(result, evidence["scene"], now, after=after)
            visual = _visual(result, evidence["support_visual"], scene, purpose="release_support")
            measurement = (_release_measurement(result, evidence["measurement"], now, after=after)
                           if opening else _measurement(result, evidence["measurement"], now))
            target = _number(evidence["target_width_m"], "target_width_m")
            delta = target-measurement["observed"]["width_m"]
            if not 0 <= target <= 0.055 or not 0 < delta <= 0.005 + 1e-12:
                _fail("release_out_of_bound")
            result.update(status="release_pending", scene=scene, visual_evidence=visual, measurement=measurement,
                          pending={"action_event_id": evidence["action_event_id"], "requested_at": now,
                                   "target_width_m": target, "before_width_m": measurement["observed"]["width_m"]})
        elif kind == "finish_release":
            if result["status"] != "release_pending":
                _fail("illegal_transition", "Release requires a matching pending event")
            _keys(evidence, ("action_event_id", "measurement", "actual_opening_increase_m", "arrival_confirmed",
                             "target_calls_sent", "passive_arm_commands_sent"), "release result")
            pending = result["pending"]
            if evidence["action_event_id"] != pending["action_event_id"]:
                _fail("release_event_mismatch")
            # This is a historical receipt for an already issued physical
            # action, not fresh admission. Keep its original trace timestamps.
            measurement = _measurement(result, evidence["measurement"], now, release=True, historical=True)
            increase = _number(evidence["actual_opening_increase_m"], "actual_opening_increase_m")
            measured = measurement["observed"]["width_m"]-pending["before_width_m"]
            if (measurement["started_at"] <= pending["requested_at"] or increase <= OBSERVATION_POLICY["jaw_m"]
                    or increase > measured + 1e-12 or evidence["arrival_confirmed"] is not True
                    or type(evidence["target_calls_sent"]) is not int or evidence["target_calls_sent"] != 1
                    or type(evidence["passive_arm_commands_sent"]) is not int or evidence["passive_arm_commands_sent"] != 0
                    or abs(measurement["observed"]["width_m"]-pending["target_width_m"]) > 0.002):
                _fail("unconfirmed_release", "Need one send, measured opening and a complete arrived stable trace")
            opening = {"action_event_id": pending["action_event_id"], "target_width_m": pending["target_width_m"],
                       "before_width_m": pending["before_width_m"], "observed_width_m": measurement["observed"]["width_m"],
                       "trace_id": measurement["trace_id"], "trace_sha256": measurement["trace_sha256"],
                       "started_at": measurement["started_at"], "finished_at": measurement["ended_at"],
                       "recorded_at": now, "actual_opening_increase_m": increase}
            result.update(status="release_opened", measurement=measurement, pending=None, retained_scope=None,
                          retention_expires_at=None, release_opening=opening, release_confirmation=None,
                          residual_target={"event_id": pending["action_event_id"], "requested_width_m": pending["target_width_m"]})
        elif kind == "confirm_release":
            if result["status"] != "release_opened":
                _fail("illegal_transition", "Separation confirmation needs the latest measured opening")
            _keys(evidence, ("action_event_id", "scene", "visual", "measurement"), "release confirmation")
            opening = result["release_opening"]
            if evidence["action_event_id"] != opening["action_event_id"]:
                _fail("release_event_mismatch")
            scene = _scene(result, evidence["scene"], now, after=opening["finished_at"])
            visual = _visual(result, evidence["visual"], scene, purpose="release_confirmation")
            measurement = _release_measurement(result, evidence["measurement"], now, after=opening["finished_at"])
            result.update(status="released", scene=scene, visual_evidence=visual, measurement=measurement,
                          release_confirmation={"action_event_id": opening["action_event_id"],
                                                "observation_id": scene["observation_id"],
                                                "trace_id": measurement["trace_id"],
                                                "trace_sha256": measurement["trace_sha256"], "confirmed_at": now})
        else:
            _fail("unknown_event_kind")
    result["revision"] += 1
    result["last_event_at"] = now
    result["events"][event["event_id"]] = {"payload_sha256": digest, "revision": result["revision"], "kind": kind}
    return result


def expire_episode(state, *, now, reason="expired"):
    """Persist an expired lease as terminal; never clear residual/pending facts."""
    result = _state(state)
    now = _number(now, "now")
    _id(reason, "reason")
    if now < result["last_event_at"]:
        _fail("clock_regressed")
    expiry = result["retention_expires_at"]
    if result["status"] in ("released", "invalid") or (now < result["deadline_at"] and (expiry is None or now < expiry)):
        return result
    result.update(status="invalid", revision=result["revision"] + 1, last_event_at=now,
                  fault={"reason": reason, "source_event_id": None, "send_status": "unknown", "at": now})
    return result


def admit_scope(state, request, *, now):
    """Check an episode prerequisite, NOT hardware dispatch admission.

    Request: identity, operation, observation_id. Static scopes also require a
    freshly resolved measurement. ``peer_unloaded_preparation`` requires the
    opposite moving_arm. Loaded requests may include resolved contract refs for
    diagnostics, but always refuse until a loaded transition is implemented.
    All actual action limits, pair exclusion, budget and dispatch checks remain
    mandatory in the host. No model-provided progress number is accepted here.
    """
    result = {"allowed": False, "missing": [], "status": None, "dispatch_authorized": False,
              "physical_stop_verified": None, "object_progress_measurement": None}
    try:
        current = _state(state)
        result["status"] = current["status"]
        _keys(request, ("identity", "operation", "observation_id"), "scope request",
              ("measurement", "moving_arm", "adapter_bounded_contact_contract",
               "adapter_applicable_hold_contract", "bounded_response_measurement_contract"))
        _json(request)
        _reject_assertions(request)
        _identity(request.get("identity"), current["identity"])
        operation = request.get("operation")
        _id(operation, "operation")
        if operation in _LOADED:
            _live(current, now)
            if current["status"] != "retained_static":
                result["missing"].append("current_static_retention")
            result["missing"].extend(_loaded_missing(request))
            return result
        _live(current, now)
        if current["status"] != "retained_static":
            _fail("current_static_retention_required")
        if operation not in ("stationary_monitor", "peer_unloaded_preparation"):
            _fail("scope_not_supported")
        if request.get("observation_id") != current["scene"]["observation_id"]:
            _fail("scene_reference_mismatch")
        if now-current["scene"]["captured_at"] > OBSERVATION_POLICY["rgb_age_s"]:
            _fail("stale_scene")
        _retention(current, current["retention_contract"], now)
        _measurement(current, request.get("measurement"), now)
        if operation == "peer_unloaded_preparation":
            peer = "right" if current["identity"]["arm"] == "left" else "left"
            if request.get("moving_arm") != peer:
                _fail("moving_arm_mismatch")
        result["allowed"] = True
        result["scope"] = copy.deepcopy(current["retained_scope"])
    except GraspEpisodeError as error:
        result["missing"] = list(error.missing) or [error.code]
    return result
