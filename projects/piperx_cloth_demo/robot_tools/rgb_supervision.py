"""Pure binding checks for explicit RGB-supervised motion contracts.

No image interpretation, file access, robot construction or metric inference.
The host owns saved-image validation and actual scene/target provenance.
"""
from .task_roles import resolve_task_roles, role_fields

INITIALIZATION_SCHEMA = "piper_rgb_supervised_initialization_v1"
JOINT_PATH_SCHEMA = "piper_rgb_supervised_joint_path_v1"
COARSE_JOINT_PATH_SCHEMA = "piper_rgb_supervised_coarse_approach_v2"
LOADED_JOINT_PATH_SCHEMA = "piper_rgb_supervised_loaded_joint_path_v1"
LOADED_CONTEXT_SCHEMA = "piper_rgb_loaded_episode_v1"
LOADED_OPERATIONS = ("extract_segment", "transport", "insert_segment")
RGB_MAX_AGE_S = 30.0
RGB_MAX_SKEW_S = .15


def validate_loaded_context(context, identity, geometry):
    """Check host-resolved references; never authenticate retention or support."""
    from .joint_path import _need, _num, _text, _hash, _vec, evidence_sha256

    _need(type(context) is dict and set(context) ==
          {"schema", "event_id", "operation", "worker", "peer", "object_scene"} | set(role_fields(context))
          and context["schema"] == LOADED_CONTEXT_SCHEMA, "loaded_context_schema")
    worker_arm, support_arm = resolve_task_roles(context)
    _need(identity["arm"] == worker_arm, "loaded_worker_arm")
    _text(context["event_id"], "loaded event id")
    evidence = geometry["evidence"]
    _need(context["operation"] in LOADED_OPERATIONS
          and context["operation"] == evidence["operation"], "loaded_operation_mismatch")
    episode_fields = {"episode_id", "arm", "run_id", "owner", "epoch", "object_id"}
    for role, arm in (("worker", worker_arm), ("peer", support_arm)):
        item = context[role]
        _need(type(item) is dict and set(item) == {"identity", "revision", "probe_event_id",
              "probe_trace_sha256", "requested_width_m", "original_anchor", "local_anchor"},
              "loaded_episode_schema", role)
        episode = item["identity"]
        _need(type(episode) is dict and set(episode) == episode_fields, "loaded_episode_identity", role)
        for key, value in episode.items():
            _text(value, "episode " + key)
            _need(len(value) <= 128, "loaded_episode_identity", key)
        _need(episode["arm"] == arm and all(episode[key] == identity[key]
              for key in ("run_id", "owner", "epoch")), "loaded_episode_binding", role)
        _need(type(item["revision"]) is int and item["revision"] >= 0, "loaded_episode_revision", role)
        _text(item["probe_event_id"], "probe event id")
        _hash(item["probe_trace_sha256"])
        _need(0 <= _num(item["requested_width_m"], "requested width") <= .055, "loaded_jaw_target")
        for name in ("original_anchor", "local_anchor"):
            anchor = item[name]
            _need(type(anchor) is dict and set(anchor) == {"joints_rad", "pose_m_rad", "width_m"},
                  "loaded_anchor_schema", role + "/" + name)
            _vec(anchor["joints_rad"], 6, "anchor joints")
            _vec(anchor["pose_m_rad"], 6, "anchor pose")
            _need(0 <= _num(anchor["width_m"], "observed width") <= .070, "loaded_anchor_width")
    _need(context["worker"]["identity"]["episode_id"] != context["peer"]["identity"]["episode_id"],
          "loaded_episode_identity")
    scene = context["object_scene"]
    _need(type(scene) is dict and set(scene) ==
          {"source_id", "target_id", "observation_id", "description"}, "loaded_object_scene_schema")
    for key in ("source_id", "target_id", "observation_id"):
        _text(scene[key], key)
    description = scene["description"]
    _need(type(description) is str and 0 < len(description) <= 4000 and bool(description.strip())
          and "\x00" not in description, "visual_description_required", "object scene")
    _need(scene["observation_id"] == evidence["observation_id"], "loaded_object_scene_mismatch")
    _need(_hash(evidence["loaded_context_sha256"]) == evidence_sha256(context), "loaded_context_hash_mismatch")


def validate_rgb_geometry(geometry, identity, origin, now, *, schema,
                          operation=None, target_raw=None, loaded_context=None):
    """Bind current image evidence to its explicit contract and frozen origin."""
    from .joint_path import _need, _num, _text, _hash, _source, _identity, evidence_sha256

    _need(schema in (INITIALIZATION_SCHEMA, JOINT_PATH_SCHEMA, LOADED_JOINT_PATH_SCHEMA,
                     COARSE_JOINT_PATH_SCHEMA), "visual_geometry_schema")
    _need(type(geometry) is dict and set(geometry) ==
          {"schema", "origin_sample_id", "source", "evidence"}
          and geometry.get("schema") == schema, "visual_geometry_schema")
    _need(geometry["origin_sample_id"] == origin["sample_id"], "geometry_anchor_mismatch")
    _source(geometry["source"])
    evidence = geometry["evidence"]
    loaded = schema == LOADED_JOINT_PATH_SCHEMA
    coarse = schema == COARSE_JOINT_PATH_SCHEMA
    observation_field = "loaded_observation" if loaded else "unloaded_observation"
    fields = {
        "identity", "observation_id", "capture_id", "saved_rgb_evidence", "rgb_received_at",
        observation_field, "corridor_observation", "workspace_clearance_statement"}
    if schema in (JOINT_PATH_SCHEMA, LOADED_JOINT_PATH_SCHEMA, COARSE_JOINT_PATH_SCHEMA):
        fields |= {"operation", "target_raw"}
    if coarse:
        fields.add("far_from_target_observation")
    if loaded:
        fields.add("loaded_context_sha256")
    _need(type(evidence) is dict and set(evidence) == fields, "visual_evidence_schema")
    if schema in (JOINT_PATH_SCHEMA, LOADED_JOINT_PATH_SCHEMA, COARSE_JOINT_PATH_SCHEMA):
        operations = ("approach",) if coarse else (LOADED_OPERATIONS if loaded else ("approach", "align", "release_retreat"))
        _need(evidence["operation"] in operations, "visual_joint_operation")
        if operation is not None:
            _need(evidence["operation"] == operation, "visual_joint_operation")
        _need(type(evidence["target_raw"]) is list and len(evidence["target_raw"]) == 6
              and all(type(v) is int for v in evidence["target_raw"])
              and evidence["target_raw"] == target_raw,
              "visual_joint_target_changed")
    _need(_identity(evidence["identity"]) == identity, "visual_identity_mismatch")
    for key in ("observation_id", "capture_id"):
        _text(evidence[key], key)
    text_fields = (observation_field, "corridor_observation", "workspace_clearance_statement")
    for key in text_fields + (("far_from_target_observation",) if coarse else ()):
        text = evidence[key]
        _need(type(text) is str and 0 < len(text) <= 4000 and bool(text.strip()) and "\x00" not in text,
              "visual_description_required", key)
    saved = evidence["saved_rgb_evidence"]
    _need(type(saved) is dict and set(saved) == {"front", "left_hand", "right_hand"},
          "visual_rgb_views_required")
    stamps = []
    for view, frame in saved.items():
        _need(type(frame) is dict and set(frame) ==
              {"rgb_path", "artifact_sha256", "frame_number", "host_received_at"},
              "visual_rgb_schema", view)
        _text(frame["rgb_path"], "RGB path")
        _hash(frame["artifact_sha256"])
        _need(type(frame["frame_number"]) is int and frame["frame_number"] >= 0,
              "visual_rgb_frame_number", view)
        received = _num(frame["host_received_at"], "RGB receive time")
        _need(0 <= received <= now and now-received <= RGB_MAX_AGE_S, "visual_rgb_expired", view)
        stamps.append(received)
    received = _num(evidence["rgb_received_at"], "shared RGB receive time")
    _need(received == min(stamps) and max(stamps)-min(stamps) <= RGB_MAX_SKEW_S,
          "visual_rgb_scene_mismatch")
    _need(geometry["source"]["sha256"] == evidence_sha256(evidence), "visual_evidence_hash_mismatch")
    if loaded:
        validate_loaded_context(loaded_context, identity, geometry)
    return received+RGB_MAX_AGE_S
