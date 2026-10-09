"""Host-owned RGB plug segments and zero-TX post-action object observations."""
import copy
import uuid

from .host_grasp import GraspBindingError, digest
from .loaded_episode import OPERATIONS, allowed_next, body_anchor
from .task_roles import resolve_task_roles, role_fields


class HostLoaded:
    def __init__(self, host):
        self.host = host
        self.requests = {}

    def block_pending(self):
        if any(s["status"] in ("proof_pending", "loaded_pending_visual") for s in self.host.grasps.states()):
            raise ValueError("The previous loaded segment requires its new visual response; no additional target")

    def context(self, event_id, payload, scene):
        host = self.host
        worker_arm, support_arm = resolve_task_roles(host.task)
        if payload["arm"] != worker_arm or payload["operation"] not in OPERATIONS:
            raise ValueError("Loaded plug path must use the frozen worker and static support arms")
        from .grasp_episode import _state, _live
        records, states = host.grasp_states, {side: host.grasps.active(side) for side in ("left", "right")}
        for side, state in states.items():
            if state is not None:
                states[side] = _state(state)
                _live(states[side], host.clock())
        worker, peer = states[worker_arm], states[support_arm]
        if worker is None or peer is None or peer["status"] != "retained_static":
            raise ValueError("Current retained worker grasp and independent static support episode required")
        allowed_next(worker, payload["operation"], payload["source_object_id"], payload["target_object_id"],
                     task_roles=host.task)
        result = {}
        for side, role in ((worker_arm, "worker"), (support_arm, "peer")):
            state, record = states[side], records.get(side)
            if (record is None or record.get("status") != state["status"]
                    or state["identity"]["owner"] != host.owner or host.clock() >= state["retention_expires_at"]):
                raise GraspBindingError("Loaded scope does not match current owned, unexpired grasp")
            probe = state["probe_ref"]
            expected = {"identity": state["identity"], "probe_event_id": probe["event_id"],
                        "trace_sha256": probe["trace_sha256"], "requested_width_m": probe["requested_width_m"],
                        "original_anchor": state["original_anchor"], "retention_contract": state["retention_contract"]}
            if any(record.get(k) != v for k, v in expected.items()) or record.get("local_anchor", record["original_anchor"]) != body_anchor(state):
                raise GraspBindingError("Loaded scope cannot replace the adapter's probe, target or body anchor")
            result[role] = {"identity": copy.deepcopy(state["identity"]), "revision": state["revision"],
                "probe_event_id": probe["event_id"], "probe_trace_sha256": probe["trace_sha256"],
                "requested_width_m": probe["requested_width_m"], "original_anchor": copy.deepcopy(state["original_anchor"]),
                "local_anchor": copy.deepcopy(body_anchor(state))}
        return {"schema": "piper_rgb_loaded_episode_v1", "event_id": event_id,
            "operation": payload["operation"], **role_fields(host.task), **result,
            "object_scene": {"source_id": payload["source_object_id"], "target_id": payload["target_object_id"],
                "observation_id": scene["observation_id"], "description": payload["loaded_observation"]}}

    def begin(self, event_id, payload, context):
        worker_arm, support_arm = resolve_task_roles(self.host.task)
        if (payload["arm"] != worker_arm
                or resolve_task_roles(context["loaded_context"]) != (worker_arm, support_arm)):
            raise GraspBindingError("Loaded task roles changed before claim")
        states = {side: self.host.grasps.active(side) for side in ("right", "left")}
        for side, role in ((worker_arm, "worker"), (support_arm, "peer")):
            current, expected = states[side], context["loaded_context"][role]
            if (current is None or current["revision"] != expected["revision"]
                    or current["identity"] != expected["identity"]
                    or current["original_anchor"] != expected["original_anchor"]
                    or body_anchor(current) != expected["local_anchor"]
                    or current["probe_ref"]["event_id"] != expected["probe_event_id"]
                    or current["probe_ref"]["trace_sha256"] != expected["probe_trace_sha256"]
                    or current["probe_ref"]["requested_width_m"] != expected["requested_width_m"]):
                raise GraspBindingError("Loaded worker or peer revision/binding changed before claim")
        state = states[worker_arm]
        scene = self.host.grasps.scene(state, payload["observation_id"])
        from .joint_path import encode_joint_target
        return self.host.grasps.append(state, "loaded_begin_"+event_id, "begin_loaded", {
            "action_event_id": event_id, "operation": payload["operation"],
            "source_id": payload["source_object_id"], "target_id": payload["target_object_id"],
            **role_fields(self.host.task),
            "target_raw": encode_joint_target(payload["target"])[0], "scene": scene,
            "context_sha256": digest(context["loaded_context"])})

    def finish(self, event_id, payload, receipt):
        host = self.host
        worker_arm, support_arm = resolve_task_roles(host.task)
        state = host.grasps.active(worker_arm)
        stored = host.ledger.event(event_id)
        if (stored is None or stored["status"] != "complete" or not stored["success"]
                or stored["payload"] != payload or stored["receipt"] != receipt):
            raise GraspBindingError("Loaded completion must resolve the exact completed dispatch")
        report = receipt.get("loaded_receipt")
        local = host.device.grasp_states.get(worker_arm)
        plan = receipt.get("joint_path_plan") or {}
        if (payload["arm"] != worker_arm or state is None
                or resolve_task_roles(plan.get("loaded_context") or {}) != (worker_arm, support_arm)
                or type(report) is not dict or local is None or local.get("status") != "loaded_pending_visual"
                or report.get("action_event_id") != event_id or report.get("operation") != payload["operation"]
                or report.get("plan_sha256") != plan.get("plan_sha256")
                or report.get("original_anchor") != state["original_anchor"]
                or report.get("probe_event_id") != state["probe_ref"]["event_id"]
                or report.get("probe_trace_sha256") != state["probe_ref"]["trace_sha256"]
                or report.get("local_anchor") != local.get("local_anchor")
                or any(local.get("loaded_pending", {}).get(k) != report.get(k) for k in
                       ("action_event_id", "operation", "finished_at", "plan_sha256", "local_anchor"))):
            raise GraspBindingError("Loaded completion differs from the adapter's retained pending response")
        evidence = {k: copy.deepcopy(report[k]) for k in
                    ("action_event_id", "operation", "finished_at", "plan_sha256", "local_anchor")}
        updated = host.grasps.append(state, "loaded_finish_"+event_id, "finish_loaded", evidence)
        host.journal.append("pair_loaded_pending_visual", event_id=event_id, episode=updated)
        return updated

    def confirm(self, event_id, action_event_id, observation_id, description, response,
                object_relation, support_relation, task_relation):
        host = self.host
        worker_arm, support_arm = resolve_task_roles(host.task)
        if type(description) is not str or not 1 <= len(description.strip()) <= 4000 or "\x00" in description:
            raise ValueError("Describe object, source/target and fixed peer in the new RGB")
        request = {"event_id": event_id, "action_event_id": action_event_id,
            **role_fields(host.task),
            "observation_id": observation_id, "description": description, "response": response,
            "object_relation": object_relation, "support_relation": support_relation, "task_relation": task_relation}
        key = digest(request)
        if event_id in self.requests:
            old = self.requests[event_id]
            if old["digest"] != key or old["result"] is None:
                raise ValueError("Changed or incomplete loaded confirmation cannot be replayed")
            return {**copy.deepcopy(old["result"]), "replayed": True}
        state = host.grasps.active(worker_arm)
        if (state is None or state["status"] != "loaded_pending_visual"
                or state["loaded"]["pending"]["action_event_id"] != action_event_id
                or resolve_task_roles(state["loaded"]) != (worker_arm, support_arm)):
            raise ValueError("Exact pending loaded event required")
        scene = host.grasps.scene(state, observation_id)
        last = state["loaded"]["history"][-1]
        if any(f["host_received_at"] <= last["finished_at"] for f in scene["frames"]):
            raise ValueError("Every response image must follow this completed segment")
        # Semantic response cannot replace live machine checks; the model never
        # supplies a measured pose, force, stop or hardware permission boolean.
        self.requests[event_id] = {"digest": key, "result": None}
        try:
            with host.device_lock:
                host._sample(host.device.observe())
                host._guard()
                host._saved_preparation_scene(observation_id, worker_arm)
                raw = {**request, "identity": state["identity"], "scene": scene,
                    "source_id": state["loaded"]["source_id"], "target_id": state["loaded"]["target_id"],
                    "source": "rgb_semantic_observation", "independent_visual_verification": False}
                sha = host.grasps._artifact(raw)
                evidence = {"action_event_id": action_event_id, "scene": scene,
                    "response": response, "object_relation": object_relation,
                    "support_relation": support_relation, "task_relation": task_relation, "artifact_sha256": sha}
                updated = host.grasps.append(state, event_id, "confirm_loaded", evidence)
                host._guard()
                host._saved_preparation_scene(observation_id, worker_arm)
                result = host.device.confirm_loaded_response(worker_arm, identity=state["identity"],
                    action_event_id=action_event_id, plan_sha256=last["plan_sha256"])
                expected = {k: last[k] for k in ("action_event_id", "operation", "finished_at", "plan_sha256", "local_anchor")}
                local = host.device.grasp_states.get(worker_arm)
                if (result.get("ok") is not True or type(result.get("hardware_commands_sent")) is not int
                        or result["hardware_commands_sent"] != 0 or result.get("confirmed_loaded_receipt") != expected
                        or local is None or local.get("status") != "retained_local" or local.get("loaded_pending") is not None
                        or local.get("local_anchor") != updated["loaded"]["local_anchor"]):
                    raise GraspBindingError("Loaded confirmation must finalize exact local pending state with zero TX")
                host._sample(result["sample"])
                host._guard()
                final = {"ok": True, "event_id": event_id, "action_event_id": action_event_id,
                    "episode": updated, "hardware_commands_sent": 0, "object_task_success": None,
                    "physical_stop_verified": None, "contact_force_verified": False,
                    "visual_response": copy.deepcopy(raw), "replayed": False}
                host.journal.append("pair_loaded_response", result=final)
                host._guard()
                host._saved_preparation_scene(observation_id, worker_arm)
                host._check_guard_context()
                self.requests[event_id]["result"] = copy.deepcopy(final)
                return final
        except BaseException as exc:
            host._fault("Loaded response unknown, adverse or inconsistent: "+str(exc))
            raise
