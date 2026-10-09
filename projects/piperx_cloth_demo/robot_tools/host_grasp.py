"""Resolve pair-owned grasp evidence; no caller-supplied hardware assertions.

Only the adapter measures traces/contracts. The caller supplies a semantic RGB
observation, explicitly recorded as model testimony, never as a force sensor
or independent physical qualification. This module does not dispatch targets.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import uuid

from .grasp_store import GraspStore
from .grasp_episode import is_resolved_release


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


class GraspBindingError(ValueError):
    """The live adapter and durable grasp history disagree; latch the pair."""


class HostGrasps:
    def __init__(self, host):
        self.host = host
        self.store = GraspStore(host.ledger)
        self.requests = {}
        self.release_tokens = {}

    def states(self):
        return self.store.read()

    def active(self, arm):
        rows = [s for s in self.states() if s["identity"]["arm"] == arm
                and s["identity"]["owner"] == self.host.owner and not is_resolved_release(s)]
        if len(rows) > 1:
            raise RuntimeError("Multiple unresolved owned grasp episodes")
        return rows[0] if rows else None

    def prepare_probe(self, arm, object_id):
        state = self.active(arm)
        if state is not None:
            if state["status"] != "empty" or (object_id is not None and object_id != state["identity"]["object_id"]):
                raise ValueError("Existing grasp identity/state cannot be replaced")
            return state
        if object_id is None:
            return None  # Existing observation-only API remains available.
        return self.store.create(self.host.owner, episode_id="grasp_" + uuid.uuid4().hex,
                                 arm=arm, object_id=object_id, epoch=self.host.owner)

    @staticmethod
    def bind_measurement(raw, state, probe_event_id):
        result = copy.deepcopy(raw)
        for key, value in (("identity", state["identity"]), ("probe_event_id", probe_event_id)):
            if key in result and result[key] != value:
                raise ValueError("Adapter measurement changed " + key)
            result[key] = copy.deepcopy(value)
        return result

    def append(self, state, event_id, kind, evidence):
        return self.store.append(self.host.owner, state["identity"]["episode_id"],
            {"event_id": event_id, "kind": kind, "identity": state["identity"], "evidence": evidence},
            expected_revision=state["revision"])

    def record_probe(self, event_id, payload, receipt):
        state = self.active(payload["arm"])
        if state is None:
            return None
        if receipt["contact_observation"]["outcome"] != "settled_contact_candidate":
            return state  # Arrived jaw is not a contact candidate.
        stored = self.host.ledger.event(event_id)
        if (stored is None or stored["status"] != "complete" or not stored["success"]
                or stored["payload"] != payload or stored["receipt"] != receipt):
            raise ValueError("Candidate must resolve the exact completed physical ledger event")
        probe = copy.deepcopy(receipt["candidate_probe"])
        bindings = {"identity": state["identity"], "event_id": event_id,
                    "observation_id": payload["observation_id"]}
        for key, value in bindings.items():
            if key in probe and probe[key] != value:
                raise ValueError("Adapter candidate changed " + key)
            probe[key] = copy.deepcopy(value)
        if probe["requested_width_m"] != payload["target"]:
            raise ValueError("Candidate does not refer to the sent jaw target")
        measurement = self.bind_measurement(receipt["candidate_measurement"], state, event_id)
        return self.append(state, "candidate_" + uuid.uuid4().hex, "record_candidate",
                           {"probe": probe, "measurement": measurement})

    def _artifact(self, data, suffix="json"):
        raw = (json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
               if suffix == "json" else data)
        sha = hashlib.sha256(raw).hexdigest()
        directory = self.host.directory / "grasp_evidence"
        directory.mkdir(exist_ok=True)
        path = directory / (sha + "." + suffix)
        try:
            with path.open("xb") as stream:
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
        except FileExistsError:
            if path.read_bytes() != raw:
                raise ValueError("Evidence artifact changed")
        fd = os.open(str(directory), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        return sha

    def record_observed_candidate(self, event_id, payload, receipt):
        """Bind a new zero-TX observation without rewriting the earlier failed send."""
        state = self.active(payload["arm"])
        stored = self.host.ledger.event(event_id)
        if (state is None or state["status"] != "empty" or stored is None
                or stored["status"] != "complete" or not stored["success"]
                or stored["payload"] != payload or stored["receipt"] != receipt
                or receipt.get("candidate_basis") != "existing_target_observation"):
            raise ValueError("Observed candidate requires this owner's exact completed zero-TX event")
        request = payload["request"]
        candidate = copy.deepcopy(receipt["current_contact_candidate"])
        if candidate["existing_target_ref"]["event_id"] != payload["source_event_id"]:
            raise ValueError("Observed candidate cannot replace the audited existing target")
        for key, value in (("identity", state["identity"]), ("event_id", event_id),
                           ("observation_id", request["observation_id"]),
                           ("requested_width_m", candidate["existing_target_ref"]["requested_width_m"])):
            if key in candidate and candidate[key] != value:
                raise ValueError("Observed candidate binding changed " + key)
            candidate[key] = copy.deepcopy(value)
        measurement = self.bind_measurement(receipt["candidate_measurement"], state, event_id)
        scene = self.scene(state, request["observation_id"])
        visual = {"identity": state["identity"], "evidence_id": "visual_"+uuid.uuid4().hex,
                  "observation_id": scene["observation_id"], "source": "rgb_and_user_contact_observation",
                  "producer_ref": "outer_codex_rgb_and_user_contact_report", "object_relation": request["contact_relation"],
                  "support_relation": request["support_relation"],
                  "bilateral_contact_source": copy.deepcopy(receipt["audited_existing_contact"]["bilateral_contact_source"])}
        visual["artifact_sha256"] = self._artifact({**visual, "scene": scene,
            "visual_description": request["visual_description"],
            "observation_proposal_sha256": payload["observation_proposal_sha256"],
            "independent_visual_verification": False})
        return self.append(state, "observed_candidate_"+uuid.uuid4().hex, "record_observed_candidate",
            {"candidate": candidate, "measurement": measurement, "scene": scene, "visual": visual})

    def scene(self, state, observation_id):
        latest = self.host.latest
        if latest is None or latest["observation_id"] != observation_id:
            raise ValueError("Current host scene required for grasp transition")
        saved = latest.get("saved_rgb_evidence")
        if not saved or set(saved) != {"front", "left_hand", "right_hand"}:
            raise ValueError("Grasp evidence requires saved RGB resolved by the recorder service")
        frames = []
        for camera, source in saved.items():
            raw = Path(source["rgb_path"]).read_bytes()
            if hashlib.sha256(raw).hexdigest() != source["artifact_sha256"]:
                raise ValueError("RGB artifact changed after shared-scene observation")
            frames.append({"camera_id": camera, "capture_id": latest["capture_id"],
                "frame_number": source["frame_number"], "host_received_at": source["host_received_at"],
                "artifact_sha256": self._artifact(raw, "png")})
        return {"identity": state["identity"], "observation_id": observation_id,
                "captured_at": latest["rgb_received_at"], "frames": frames}

    def retain(self, event_id, episode_id, observation_id, visual_description,
               object_relation, support_relation):
        if not isinstance(visual_description, str) or not 1 <= len(visual_description.strip()) <= 4000:
            raise ValueError("Describe the object/finger and original-support relationships in current RGB")
        if object_relation != "between_fingers" or support_relation != "original_support_present":
            raise ValueError("Static retention requires the observed original support and object between fingers")
        request = {"event_id": event_id, "episode_id": episode_id, "observation_id": observation_id,
                   "visual_description": visual_description, "object_relation": object_relation,
                   "support_relation": support_relation}
        key = digest(request)
        if event_id in self.requests:
            prior = self.requests[event_id]
            if prior["digest"] != key:
                raise ValueError("Existing retention event has different input")
            if prior["result"] is None:
                raise RuntimeError("Previous retention attempt incomplete; no automatic replay")
            return {**copy.deepcopy(prior["result"]), "replayed": True}
        host = self.host
        state = self.store.read(episode_id)
        if (state is None or state["identity"]["owner"] != host.owner
                or state["status"] not in ("contact_candidate", "retained_static")):
            raise ValueError("This owner needs a recorded candidate or retained static episode")
        scene = self.scene(state, observation_id)
        visual_record = {**request, "identity": state["identity"], "scene": scene,
                         "source": "rgb_semantic_observation", "producer_ref": "outer_codex_rgb_report",
                         "independent_visual_verification": False}
        visual = {"identity": state["identity"], "evidence_id": "visual_" + uuid.uuid4().hex,
                  "observation_id": observation_id, "source": "rgb_semantic_observation",
                  "producer_ref": "outer_codex_rgb_report", "artifact_sha256": self._artifact(visual_record),
                  "object_relation": object_relation, "support_relation": support_relation}
        self.requests[event_id] = {"digest": key, "result": None}
        try:
            with host.device_lock:
                host._guard()
                receipt = host.device.retain_grasp(state["identity"]["arm"], identity=state["identity"],
                    probe_event_id=state["probe_ref"]["event_id"],
                    probe_trace_sha256=state["probe_ref"]["trace_sha256"], deadline_at=state["deadline_at"])
                if (receipt.get("ok") is not True or receipt.get("completion_mode") != "retain_static"
                        or receipt.get("hardware_commands_sent") != 0 or receipt.get("target_calls_sent") != 0
                        or receipt.get("physical_stop_verified") is not None):
                    raise ValueError("Retention needs a zero-TX adapter-owned observation contract")
                host._sample(receipt["sample"])
                host._guard()
                updated = self.append(state, event_id,
                    "retain_static" if state["status"] == "contact_candidate" else "renew_static",
                    {"scene": scene, "visual": visual, "measurement": receipt["measurement"],
                     "retention_contract": receipt["retention_contract"]})
                result = {"ok": True, "event_id": event_id, "episode": updated, "replayed": False,
                          "hardware_commands_sent": 0, "physical_stop_verified": None,
                          "loaded_contact_available": False, "object_task_success": None}
                host.journal.append("pair_grasp_retained", request=request, receipt=result)
                self.requests[event_id]["result"] = copy.deepcopy(result)
                return result
        except BaseException as exc:
            host._fault("Static retention incomplete: " + str(exc))
            raise

    def _release_scene(self, state, observation_id):
        self.host._guard()
        self.host._saved_preparation_scene(observation_id, state["identity"]["arm"])
        scene = self.scene(state, observation_id)
        self.host._guard()
        self.host._saved_preparation_scene(observation_id, state["identity"]["arm"])
        self.host._check_guard_context()
        return scene

    def _release_visual(self, state, scene, description, relation, *, separated):
        if (type(description) is not str or not 1 <= len(description.strip()) <= 4000
                or relation != "independent_support_present"):
            raise ValueError("Describe this object's current independent support in the saved RGB")
        visual = {"identity": state["identity"], "evidence_id": "visual_" + uuid.uuid4().hex,
                  "observation_id": scene["observation_id"], "source": "rgb_semantic_observation",
                  "producer_ref": "outer_codex_rgb_report",
                  "object_relation": "object_clear_of_fingers" if separated else "separation_not_assessed",
                  "support_relation": relation}
        visual["artifact_sha256"] = self._artifact({**visual, "scene": scene,
            "visual_description": description, "independent_visual_verification": False})
        return visual

    def prepare_release(self, event_id, arm, target, observation_id,
                        support_observation=None, support_relation=None):
        state = self.active(arm)
        if state is None or state["status"] == "empty":
            return None
        scene = self._release_scene(state, observation_id)
        visual = self._release_visual(state, scene, support_observation, support_relation, separated=False)
        try:
            receipt = self.host.device.observe_grasp(arm, identity=state["identity"],
                                                    probe_event_id=state["probe_ref"]["event_id"])
            if receipt.get("ok") is not True or receipt.get("hardware_commands_sent") != 0:
                raise ValueError("Release preparation requires fresh zero-TX grasp feedback")
            self.host._sample(receipt["sample"])
            self.host._guard()
            self.host._saved_preparation_scene(observation_id, arm)
            return self.append(state, "release_begin_" + uuid.uuid4().hex, "begin_release",
                {"action_event_id": event_id, "target_width_m": target,
                 "scene": scene, "measurement": receipt["measurement"], "support_visual": visual})
        except BaseException as exc:
            self.host._fault("Release preparation feedback/state failed: " + str(exc))
            raise

    def finish_release(self, event_id, payload, receipt):
        state = self.active(payload["arm"])
        if state is None or state["status"] == "empty":
            return None
        if state["status"] != "release_pending":
            raise ValueError("Release receipt has no matching pending grasp transition")
        stored = self.host.ledger.event(event_id)
        if (stored is None or stored["status"] != "complete" or not stored["success"]
                or stored["payload"] != payload or stored["receipt"] != receipt):
            raise ValueError("Release must resolve the exact completed physical ledger event")
        measurement = self.bind_measurement(receipt["release_measurement"], state, state["probe_ref"]["event_id"])
        return self.append(state, "release_finish_" + uuid.uuid4().hex, "finish_release",
            {"action_event_id": event_id, "measurement": measurement,
             **{k: receipt[k] for k in ("actual_opening_increase_m", "arrival_confirmed",
                                       "target_calls_sent", "passive_arm_commands_sent")}})

    def confirm_release(self, event_id, episode_id, observation_id, visual_description,
                        object_relation, support_relation):
        """Record post-opening RGB and feedback, then clear the exact local episode."""
        if object_relation != "object_clear_of_fingers":
            raise ValueError("Current RGB must show the object clear of this gripper's fingers")
        request = {"operation": "confirm_release", "event_id": event_id, "episode_id": episode_id,
                   "observation_id": observation_id, "visual_description": visual_description,
                   "object_relation": object_relation, "support_relation": support_relation}
        key = digest(request)
        if event_id in self.requests:
            prior = self.requests[event_id]
            if prior["digest"] != key:
                raise ValueError("Existing grasp event has different input")
            if prior["result"] is None:
                raise RuntimeError("Prior release confirmation incomplete; no automatic replay")
            return {**copy.deepcopy(prior["result"]), "replayed": True}
        host = self.host
        state = self.store.read(episode_id)
        if (state is None or state["identity"]["owner"] != host.owner
                or state["status"] != "release_opened"):
            raise ValueError("This owner needs an opened, still-unresolved grasp episode")
        arm = state["identity"]["arm"]
        scene = self._release_scene(state, observation_id)
        visual = self._release_visual(state, scene, visual_description, support_relation, separated=True)
        opening = state["release_opening"]
        self.requests[event_id] = {"digest": key, "result": None}
        try:
            with host.device_lock:
                host._guard()
                self.check_retained_preparation(arm, "gripper", "release_retreat", host.grasp_states)
                host._saved_preparation_scene(observation_id, arm)
                receipt = host.device.observe_release(arm, identity=state["identity"],
                    probe_event_id=state["probe_ref"]["event_id"],
                    release_trace_sha256=opening["trace_sha256"])
                receipt = json.loads(json.dumps(receipt, allow_nan=False))
                if (receipt.get("ok") is not True or receipt.get("completion_mode") != "observe_release"
                        or type(receipt.get("hardware_commands_sent")) is not int
                        or receipt["hardware_commands_sent"] != 0
                        or type(receipt.get("target_calls_sent")) is not int or receipt["target_calls_sent"] != 0
                        or receipt.get("physical_stop_verified") is not None):
                    raise ValueError("Release confirmation needs fresh zero-TX adapter feedback")
                host._sample(receipt["sample"])
                host._guard()
                host._saved_preparation_scene(observation_id, arm)
                updated = self.append(state, event_id, "confirm_release",
                    {"action_event_id": opening["action_event_id"], "scene": scene,
                     "visual": visual, "measurement": receipt["measurement"]})
                if not is_resolved_release(updated):
                    raise ValueError("Durable release confirmation did not establish separation evidence")
                host._guard()
                host._saved_preparation_scene(observation_id, arm)
                cleanup = host.device.finalize_release(arm, identity=state["identity"],
                    probe_event_id=state["probe_ref"]["event_id"],
                    release_trace_sha256=opening["trace_sha256"],
                    confirmation_trace_sha256=receipt["measurement"]["trace_sha256"])
                cleanup = json.loads(json.dumps(cleanup, allow_nan=False))
                if (cleanup.get("ok") is not True or cleanup.get("completion_mode") != "finalize_release"
                        or type(cleanup.get("hardware_commands_sent")) is not int
                        or cleanup["hardware_commands_sent"] != 0
                        or type(cleanup.get("target_calls_sent")) is not int or cleanup["target_calls_sent"] != 0
                        or cleanup.get("release_trace_sha256") != opening["trace_sha256"]
                        or cleanup.get("confirmation_trace_sha256") != receipt["measurement"]["trace_sha256"]
                        or cleanup.get("physical_stop_verified") is not None
                        or host.grasp_states.get(arm) is not None):
                    raise ValueError("Exact local opening could not be finalized without sending")
                host._sample(cleanup["sample"])
                host._guard()
                host._saved_preparation_scene(observation_id, arm)
                result = {"ok": True, "event_id": event_id, "episode": updated, "replayed": False,
                          "hardware_commands_sent": 0, "physical_stop_verified": None,
                          "object_task_success": None, "loaded_contact_available": False,
                          "independent_visual_verification": False,
                          "adapter_observation": receipt, "adapter_finalization": cleanup}
                host.journal.append("pair_release_confirmed", request=request, receipt=result)
                host._guard()
                host._saved_preparation_scene(observation_id, arm)
                host._check_guard_context()
                self.requests[event_id]["result"] = copy.deepcopy(result)
                self.release_tokens[arm] = {"episode_id": episode_id,
                    "opening": copy.deepcopy(updated["release_opening"]),
                    "confirmation": copy.deepcopy(updated["release_confirmation"])}
                return result
        except BaseException as exc:
            host._fault("Release confirmation incomplete: " + str(exc))
            raise

    def forget_release(self, arm):
        self.release_tokens.pop(arm, None)

    def require_released_worker(self, arm, device_states, observation_id, description, sample):
        if device_states.get(arm) is not None or self.active(arm) is not None:
            raise ValueError("Release retreat requires a confirmed empty local gripper")
        token = self.release_tokens.get(arm)
        if token is None:
            raise ValueError("Release retreat needs a current confirmation not superseded by another jaw command")
        latest = self.store.read(token["episode_id"])
        if (latest is None or latest["identity"]["owner"] != self.host.owner or not is_resolved_release(latest)
                or token["opening"] != latest["release_opening"]
                or token["confirmation"] != latest["release_confirmation"]):
            raise GraspBindingError("Current release token no longer matches its durable confirmation")
        scene = self._release_scene(latest, observation_id)
        if (type(description) is not str or not 1 <= len(description.strip()) <= 4000
                or scene["captured_at"] <= latest["release_confirmation"]["confirmed_at"]):
            raise ValueError("Retreat requires new post-confirmation RGB describing the empty gripper clear of the object")
        width = sample["arms"][arm]["gripper"]["width_m"]
        opening = latest["release_opening"]
        if (abs(width-opening["observed_width_m"]) > .0005
                or abs(width-opening["target_width_m"]) > .002):
            raise GraspBindingError("Jaw changed since the confirmed opening")
        event = latest["release_opening"]["action_event_id"]
        stored = self.host.ledger.event(event)
        if (stored is None or stored["status"] != "complete" or not stored["success"]
                or stored["owner"] != self.host.owner):
            raise GraspBindingError("Confirmed release lacks its completed opening event")
        evidence = {"identity": latest["identity"], "scene": scene,
                    "opening": opening, "confirmation": latest["release_confirmation"],
                    "visual_description": description, "source": "rgb_semantic_observation",
                    "object_relation": "empty_gripper_clear_of_object",
                    "independent_visual_verification": False}
        reference = self._artifact(evidence)
        self.host._guard()
        self.host._saved_preparation_scene(observation_id, arm)
        self.host._check_guard_context()
        return {"episode_id": token["episode_id"], "artifact_sha256": reference}

    def check_retained_preparation(self, arm, kind, operation, device_states):
        for side in ("left", "right"):
            record = device_states.get(side)
            state = self.active(side)
            if state is not None and state["status"] != "empty":
                if record is None or state["status"] != record.get("status"):
                    raise GraspBindingError("Durable unresolved grasp is missing or changed in the adapter")
                probe = state["probe_ref"]
                if (record.get("trace_sha256") != probe["trace_sha256"]
                        or record.get("requested_width_m") != probe["requested_width_m"]
                        or record.get("original_anchor") != state["original_anchor"]
                        or record.get("identity") not in (None, state["identity"])
                        or record.get("probe_event_id") not in (None, probe["event_id"])):
                    raise GraspBindingError("Adapter candidate does not match this exact durable grasp episode")
                if state.get("loaded"):
                    from .loaded_episode import body_anchor
                    if record.get("local_anchor") != body_anchor(state):
                        raise GraspBindingError("Loaded local anchor differs from its completed device receipt")
                    if state["status"] in ("proof_pending", "loaded_pending_visual"):
                        raise ValueError("A loaded segment still needs its new visual response")
                    if state["status"] == "retained_local" and (side != arm or kind != "gripper" or operation != "release_retreat"):
                        raise ValueError("Retained loaded worker needs the dedicated segment route or supported same-jaw release")
                if state["status"] == "release_opened":
                    opening, local = state["release_opening"], record.get("release_opening")
                    if (not isinstance(local, dict) or any(local.get(key) != opening[key] for key in
                            ("trace_sha256", "finished_at", "observed_width_m", "target_width_m"))):
                        raise GraspBindingError("Adapter opening differs from the durable release episode")
                    if side != arm or kind != "gripper" or operation != "release_retreat":
                        raise ValueError("Opened grasp permits only same-jaw opening or release confirmation")
            if record is None or record.get("status") != "retained_static":
                continue
            if (state is None or state["status"] != "retained_static"
                    or self.host.clock() >= state["retention_expires_at"]):
                raise GraspBindingError("Adapter retention lacks a current durable host episode")
            if (record.get("identity") != state["identity"]
                    or record.get("probe_event_id") != state["probe_ref"]["event_id"]
                    or record.get("trace_sha256") != state["probe_ref"]["trace_sha256"]
                    or record.get("requested_width_m") != state["probe_ref"]["requested_width_m"]
                    or record.get("original_anchor") != state["original_anchor"]
                    or record.get("retention_contract") != state["retention_contract"]):
                raise GraspBindingError("Adapter retention does not match this exact durable grasp episode")
            if side == arm:
                if kind != "gripper" or operation != "release_retreat":
                    raise ValueError("Retained arm may only explicitly release its jaw")
            elif not ((kind in ("move", "joint") and operation in ("approach", "align", "release_retreat"))
                      or (kind == "gripper" and operation in ("grip_supported", "release_retreat"))):
                raise ValueError("Static retention permits only unloaded peer preparation")
