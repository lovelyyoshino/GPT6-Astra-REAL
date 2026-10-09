"""Narrow RGB plug-transfer bookkeeping, never force or dispatch qualification.

Only the owning host supplies these records, after resolving device receipts and
saved images. Original probe anchors survive every segment. A local body anchor
changes only with a complete arrived segment; a fresh semantic response is
required before another segment. No controller or camera is opened here.
"""
import copy
from .task_roles import resolve_task_roles, role_fields

OPERATIONS = frozenset(("extract_segment", "transport", "insert_segment"))
MAX_NO_PROGRESS = 2  # Cumulative episode policy, never renewed by a new phase.
RELATIONS = frozenset(("source_engaged", "source_separated", "target_aligned",
                       "target_partly_inserted", "target_seated"))


def body_anchor(state):
    loaded = state.get("loaded")
    return loaded["local_anchor"] if loaded else state["original_anchor"]


def validate(state):
    from .grasp_episode import _keys, _id, _hash, _number, _pose, _fail
    data = state.get("loaded")
    if data is None:
        if state["status"] in ("proof_pending", "loaded_pending_visual", "retained_local"):
            _fail("loaded_state_without_history")
        return
    _keys(data, ("source_id", "target_id", "local_anchor", "history", "pending", "no_progress_count",
                 *role_fields(data)), "loaded")
    _id(data["source_id"], "source_id")
    _id(data["target_id"], "target_id")
    if data["source_id"] == data["target_id"] or state["identity"]["arm"] != resolve_task_roles(data)[0]:
        _fail("loaded_object_identity")
    _pose(data["local_anchor"], "local_anchor")
    if type(data["history"]) is not list or len(data["history"]) > 1024:
        _fail("loaded_history_invalid")
    count, prior_end, last_anchor, event_ids = 0, state["created_at"], state["original_anchor"], set()
    for item in data["history"]:
        _keys(item, ("action_event_id", "operation", "finished_at", "plan_sha256", "local_anchor", "response"), "loaded segment")
        _id(item["action_event_id"], "action_event_id")
        _hash(item["plan_sha256"], "plan_sha256")
        _pose(item["local_anchor"], "segment anchor")
        ended = _number(item["finished_at"], "finished_at")
        if item["operation"] not in OPERATIONS or ended < prior_end or item["action_event_id"] in event_ids:
            _fail("loaded_history_invalid")
        if abs(item["local_anchor"]["width_m"]-state["original_anchor"]["width_m"]) > .0005:
            _fail("loaded_jaw_changed")
        event_ids.add(item["action_event_id"])
        prior_end, last_anchor = ended, item["local_anchor"]
        response = item["response"]
        if response is not None:
            _keys(response, ("observation_id", "captured_at", "artifact_sha256", "response", "object_relation",
                             "support_relation", "task_relation", "confirmed_at"), "loaded response")
            _id(response["observation_id"], "observation_id")
            _hash(response["artifact_sha256"], "artifact_sha256")
            if (not ended < _number(response["captured_at"], "captured_at") <= _number(response["confirmed_at"], "confirmed_at")
                    or response["response"] not in ("progress", "no_progress")
                    or response["object_relation"] != "retained_between_fingers"
                    or response["support_relation"] != "table_supported_stationary"
                    or response["task_relation"] not in RELATIONS):
                _fail("loaded_response_invalid")
            count += response["response"] == "no_progress"
    if data["local_anchor"] != last_anchor or type(data["no_progress_count"]) is not int or data["no_progress_count"] != count:
        _fail("loaded_anchor_or_count_changed")
    pending = data["pending"]
    if pending is not None:
        _keys(pending, ("action_event_id", "operation", "requested_at", "target_raw"), "loaded pending")
        _id(pending["action_event_id"], "action_event_id")
        _number(pending["requested_at"], "requested_at")
        if (pending["operation"] not in OPERATIONS or type(pending["target_raw"]) is not list
                or len(pending["target_raw"]) != 6 or any(type(v) is not int for v in pending["target_raw"])):
            _fail("loaded_pending_invalid")
    if state["status"] != "invalid":
        if (state["status"] in ("proof_pending", "loaded_pending_visual")) != (pending is not None):
            _fail("loaded_pending_state_mismatch")
        if state["status"] == "loaded_pending_visual":
            if not data["history"] or data["history"][-1]["action_event_id"] != pending["action_event_id"] or data["history"][-1]["response"] is not None:
                _fail("loaded_response_not_pending")
        if state["status"] == "retained_local" and (not data["history"] or data["history"][-1]["response"] is None):
            _fail("loaded_response_required")


def allowed_next(state, operation, source_id, target_id, *, task_roles=None):
    from .grasp_episode import _fail
    data = state.get("loaded")
    roles = resolve_task_roles(task_roles if task_roles is not None else data or {})
    if state["identity"]["arm"] != roles[0] or operation not in OPERATIONS:
        _fail("loaded_operation_not_supported")
    if data is not None and resolve_task_roles(data) != roles:
        _fail("loaded_task_roles_changed")
    if state["status"] not in ("retained_static", "retained_local"):
        _fail("loaded_response_required")
    if data is None:
        if operation != "extract_segment":
            _fail("first_loaded_segment_must_extract")
        return
    if (source_id, target_id) != (data["source_id"], data["target_id"]):
        _fail("loaded_socket_identity_changed")
    if data["no_progress_count"] >= MAX_NO_PROGRESS:
        _fail("loaded_no_progress_budget_exhausted")
    relation = data["history"][-1]["response"]["task_relation"]
    allowed = {"extract_segment": {"source_engaged"}, "transport": {"source_separated", "target_aligned"},
               "insert_segment": {"target_aligned", "target_partly_inserted"}}
    if relation not in allowed[operation]:
        _fail("loaded_phase_response_required")


def apply(state, kind, evidence, now):
    from .grasp_episode import _keys, _id, _hash, _number, _pose, _scene, _fail
    if kind == "begin_loaded":
        _keys(evidence, ("action_event_id", "operation", "source_id", "target_id", "target_raw", "scene", "context_sha256",
                         *role_fields(evidence)), "loaded begin")
        for key in ("action_event_id", "source_id", "target_id"):
            _id(evidence[key], key)
        _hash(evidence["context_sha256"], "context_sha256")
        allowed_next(state, evidence["operation"], evidence["source_id"], evidence["target_id"], task_roles=evidence)
        if evidence["source_id"] == evidence["target_id"]:
            _fail("loaded_object_identity")
        scene = _scene(state, evidence["scene"], now)
        if not state.get("loaded"):
            state["loaded"] = {"source_id": evidence["source_id"], "target_id": evidence["target_id"],
                **role_fields(evidence),
                "local_anchor": copy.deepcopy(state["original_anchor"]), "history": [], "pending": None, "no_progress_count": 0}
        state["loaded"]["pending"] = {key: copy.deepcopy(evidence[key]) for key in ("action_event_id", "operation", "target_raw")}
        state["loaded"]["pending"]["requested_at"] = now
        state.update(status="proof_pending", scene=scene)
    elif kind == "finish_loaded":
        _keys(evidence, ("action_event_id", "operation", "finished_at", "plan_sha256", "local_anchor"), "loaded completion")
        data = state.get("loaded")
        if state["status"] != "proof_pending" or data is None:
            _fail("loaded_completion_without_pending")
        pending = data["pending"]
        if any(evidence[k] != pending[k] for k in ("action_event_id", "operation")):
            _fail("loaded_event_changed")
        ended = _number(evidence["finished_at"], "finished_at")
        _hash(evidence["plan_sha256"], "plan_sha256")
        _pose(evidence["local_anchor"], "local_anchor")
        if not pending["requested_at"] < ended <= now:
            _fail("loaded_completion_time")
        data["local_anchor"] = copy.deepcopy(evidence["local_anchor"])
        data["history"].append({**copy.deepcopy(evidence), "response": None})
        state.update(status="loaded_pending_visual")
    elif kind == "confirm_loaded":
        _keys(evidence, ("action_event_id", "scene", "response", "object_relation", "support_relation", "task_relation", "artifact_sha256"), "loaded confirmation")
        data = state.get("loaded")
        if state["status"] != "loaded_pending_visual" or data is None or evidence["action_event_id"] != data["pending"]["action_event_id"]:
            _fail("loaded_response_not_pending")
        item = data["history"][-1]
        scene = _scene(state, evidence["scene"], now, after=item["finished_at"])
        _hash(evidence["artifact_sha256"], "artifact_sha256")
        if (evidence["response"] not in ("progress", "no_progress")
                or evidence["object_relation"] != "retained_between_fingers"
                or evidence["support_relation"] != "table_supported_stationary"):
            _fail("loaded_response_adverse_or_unknown")
        options = {"extract_segment": {"source_engaged", "source_separated"},
                   "transport": {"source_separated", "target_aligned"},
                   "insert_segment": {"target_aligned", "target_partly_inserted", "target_seated"}}
        if evidence["task_relation"] not in options[item["operation"]]:
            _fail("loaded_task_relation_invalid")
        preceding = (data["history"][-2]["response"]["task_relation"]
                     if len(data["history"]) > 1 else "source_engaged")
        if evidence["response"] == "no_progress" and evidence["task_relation"] != preceding:
            _fail("no_progress_cannot_advance_loaded_phase")
        item["response"] = {k: evidence[k] for k in ("response", "object_relation", "support_relation", "task_relation", "artifact_sha256")}
        item["response"].update(observation_id=scene["observation_id"], captured_at=scene["captured_at"], confirmed_at=now)
        data["no_progress_count"] += evidence["response"] == "no_progress"
        data["pending"] = None
        state.update(status="retained_local", scene=scene, retained_scope=None)
    else:
        _fail("unknown_loaded_event")
    validate(state)
