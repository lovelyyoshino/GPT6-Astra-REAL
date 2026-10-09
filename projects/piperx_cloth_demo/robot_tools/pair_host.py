"""Persistent, attended one-arm-at-a-time host; no physical stop or grasp claim.

The device supplies measured stationary receipts. SQLite claims precede the
only dispatch attempt, including when a process dies before recording a result.
Contact support is a device capability, never a caller-provided boolean.
"""
from __future__ import annotations

import copy
from .feedback_tolerance import PROFILE_KEY, task_policy, joints_within
from .task_roles import PROFILE_KEY as TASK_ROLES_KEY, resolve_task_roles, role_fields
import hashlib
import json
import math
import re
import threading
import time
import uuid
from pathlib import Path

from .execution import ExclusiveExecution, Journal
from .fault_feedback import diagnostic_value
from .host_grasp import GraspBindingError, HostGrasps
from .grasp_episode import is_resolved_release
from .pair_ledger import (PairLedger, platform_fault, activated_execution_budget,
                         initial_rx_round_requires_preparation)

RGB_MAX_AGE_S = 30.0
CONTACT_OPERATIONS = frozenset(("grip_supported", "grip_test", "extract_segment", "insert_segment",
                                "rotate_segment", "push_segment", "wipe_segment", "sweep_segment"))
OPERATIONS = CONTACT_OPERATIONS | frozenset(("approach", "align", "transport", "release_retreat",
                                            "return_reference", "observer_reposition"))


class PairHostError(RuntimeError):
    pass


class _PairClosing(PairHostError):
    """Expected guard refusal when a clean close interrupts an idle RX read."""


class _RefreshRGBRequired(PairHostError):
    def __init__(self, receipt):
        super().__init__(receipt["reason"])
        self.receipt = receipt


def _identifier(value, name):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,96}", value):
        raise ValueError(name + " must be a short alphanumeric identifier")
    return value


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


class PairHost:
    def __init__(self, runs, profile, run_id, task, max_steps=128, max_duration_s=900,
                 *, device_factory=None, clock=time.time, background=True, joint_sources_provider=None,
                 connection_mode="ready"):
        if type(connection_mode) is not str or connection_mode not in ("ready", "prepare"):
            raise ValueError("connection_mode must be ready or prepare")
        from .pair_task_enrollment import preparation_only
        if preparation_only(Path(runs) / "pair_sessions.sqlite", run_id) and connection_mode != "prepare":
            raise ValueError("This audited task enrollment requires a fresh preparation connection")
        if initial_rx_round_requires_preparation(Path(runs) / "pair_sessions.sqlite", run_id) and connection_mode != "prepare":
            raise ValueError("Initial RX recovery requires a fresh preparation connection")
        if type(max_steps) is not int or not 1 <= max_steps <= 1000:
            raise ValueError("Pair max_steps must be a bounded positive integer (expanded cap 1000)")
        if (type(max_duration_s) not in (int, float) or not 0 < max_duration_s <= 10800
                or not math.isfinite(max_duration_s)):
            raise ValueError("Pair max_duration_s must be finite, positive and at most 10800")
        if (max_steps > 128 or max_duration_s > 900) and not activated_execution_budget(
                Path(runs) / "pair_sessions.sqlite", run_id,
                max_steps=max_steps, max_duration_s=max_duration_s):
            raise ValueError("Expanded pair budget requires its exact explicitly activated execution epoch")
        self.runs, self.profile = Path(runs), copy.deepcopy(profile)
        self.run_id, self.task = _identifier(run_id, "run_id"), copy.deepcopy(task)
        if (not isinstance(task, dict) or not isinstance(task.get("task_id"), str)
                or not task["task_id"] or task.get("roles") != {"left": "task", "right": "task"}):
            raise ValueError("A frozen task_id and two task-arm roles are required")
        resolve_task_roles(self.task)
        if role_fields(self.task):
            self.profile[TASK_ROLES_KEY] = role_fields(self.task)
        else:
            self.profile.pop(TASK_ROLES_KEY, None)
        clearance = task.get("site_context", {}).get("workspace_clearance", {})
        if (not isinstance(clearance, dict) or clearance.get("source") != "user"
                or not isinstance(clearance.get("statement"), str) or not clearance["statement"].strip()):
            raise ValueError("Record this task's actual user workspace-clearance statement")
        if device_factory is None:
            from .pair_device import GuardedPairDevice
            device_factory = GuardedPairDevice
        self.feedback_policy = task_policy(self.task)
        self.profile[PROFILE_KEY] = copy.deepcopy(self.feedback_policy)
        self.factory, self.clock, self.background = device_factory, clock, background
        self.connection_mode = connection_mode
        self.task_ready, self.readiness = False, None
        self.joint_sources_diagnostic = None
        # Internal reviewed source reader, never a public tool argument. Its
        # presence cannot replace missing current limits, geometry or cache.
        self.joint_sources_provider = joint_sources_provider
        self.owner = "host_" + uuid.uuid4().hex
        self.directory = self.runs / ("pair_" + run_id)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.journal = Journal(self.directory)
        sources = ("pair_host.py", "pair_device.py", "pair_ledger.py", "single_supervised_actions.py",
                   "supervised_actions.py", "linear_hold.py", "takeover.py", "home_arm.py",
                   "single_gripper_prepare.py", "arms.py", "execution.py", "service.py", "server.py",
                   "contact_receipt.py", "supported_contact_measurement.py", "fault_feedback.py", "motion_effect.py",
                   "host_grasp.py", "grasp_store.py", "grasp_episode.py", "retention_receipt.py",
                   "host_joint.py", "pair_joint_adapter.py", "joint_path.py", "joint_geometry.py",
                   "hold_transaction.py", "model_compatibility.py", "pair_preparation.py", "pair_limits.py", "joint_sources.py",
                   "joint_initialization.py", "pair_initialization.py", "joint_ingress.py",
                   "joint_model_bounds.py", "site_geometry_records.py", "rgb_supervision.py",
                   "tracking_observation.py", "host_loaded.py", "loaded_episode.py", "pair_restart.py", "pair_round.py",
                   "pair_continuation.py", "pair_task_enrollment.py", "reboot_startup.py", "arm_power_cycle.py", "feedback_tolerance.py",
                   "preparation_continuation.py", "coherent_feedback.py", "feedback_continuation.py",
                   "initialization_continuation.py", "endpoint_continuation.py", "task_roles.py", "plug_recipe.py",
                   "controller_limits.py", "configuration_recovery.py", "round_rgb_continuation.py", "manual_gripper_continuation.py",
                   "supported_gripper_recovery.py", "host_recovery.py", "startup_reset.py", "gripper_zero.py")
        code = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in sources}
        contract = {"task": self.task, "arms": self.profile["arms"], "cameras": self.profile["cameras"],
                    "sdk_commit_audited": self.profile.get("sdk_commit_audited"), "code": code}
        self.ledger = PairLedger(self.runs / "pair_sessions.sqlite", run_id, contract,
                                 max_steps=max_steps, max_duration_s=max_duration_s, clock=clock)
        self.grasps = HostGrasps(self)
        from .host_loaded import HostLoaded
        self.loaded = HostLoaded(self)
        self.device, self.lease = None, None
        self.opened = False
        self.fault_event, self.quit_event = threading.Event(), threading.Event()
        self.state_lock, self.device_lock, self.guard_lock = threading.RLock(), threading.RLock(), threading.RLock()
        self.close_lock = threading.Lock()
        self._fault_record_lock = threading.Lock()
        self._fault_record_attempted = False
        self._fault_reason = self._fault_record_error = None
        self._failure_receipt_persistence_error = None
        self._feedback_lock = threading.Lock()
        self._fault_feedback = None
        self._fault_feedback_journal_error = None
        self._close_report = None
        self.worker, self.monitor = None, None
        self.actions, self.latest, self.frame_numbers = {}, None, {}
        self.sequence = 0
        self.active_event_id = None
        self.active_joint_bridge = None
        self.dispatch_rgb_deadline = None
        self.rgb_not_before = None
        status = self.ledger.status()
        self.deadline = status["deadline_s"]
        self.last_clock = self.clock()

    def _fault(self, reason):
        self.fault_event.set()  # A frame guard sees this before waiting on disk.
        bridge = self.active_joint_bridge
        if bridge is not None:
            bridge.invalidate(str(reason))  # A later independent fault also cancels the exceptional hold.
        if not self._fault_record_lock.acquire(blocking=False):
            return
        try:
            if self._fault_record_attempted:
                return
            self._fault_record_attempted = True
            self._fault_reason = str(reason)[:4096]
            try:
                self.ledger.fault(self.owner, self._fault_reason)
            except Exception as exc:
                # The owned/pending record still blocks a subsequent owner.
                # Fault diagnostics must not retry disk writes every 50 ms.
                self._fault_record_error = type(exc).__name__ + ": " + str(exc)
        finally:
            self._fault_record_lock.release()

    def _finish_failed_receipt(self, event_id, receipt):
        """Use the success path's JSON boundary without hiding persistence loss.

        SDK IntEnum/finite scalar values may serialize as ordinary JSON even
        though the ledger deliberately accepts only exact built-in types.
        Nonfinite/unserializable evidence remains rejected and the original
        receipt is retained in memory; never scrub it or retry a claimed step.
        """
        stage = "normalize"
        try:
            receipt = json.loads(json.dumps(receipt, allow_nan=False))
            stage = "ledger_finish"
            self.ledger.finish(self.owner, event_id, receipt, success=False)
        except Exception as exc:
            self._failure_receipt_persistence_error = {
                "event_id": event_id, "stage": stage,
                "type": type(exc).__name__, "detail": str(exc)[:4096]}
        return receipt

    def _guard(self):
        with self.guard_lock:
            return self._guard_locked()

    def _guard_locked(self):
        self._check_guard_context()
        if platform_fault(self.runs / "pair_sessions.sqlite", run_id=self.run_id) is not None:
            self.fault_event.set()
            raise PairHostError("Persistent pair fault blocks all dispatch")
        # The read can block behind storage/another process. Its earlier time
        # and cancellation snapshot cannot authorize a frame after it returns.
        self._check_guard_context()

    def _check_guard_context(self):
        if self.fault_event.is_set():
            raise PairHostError("Pair latched or closing; no further target frames")
        if self.quit_event.is_set():
            raise _PairClosing("Pair closing; no further target frames")
        now = self.clock()
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0:
            self._fault("Invalid clock during pair feedback or frame dispatch")
            raise PairHostError("Pair clock must be finite")
        if now < self.last_clock or now >= self.deadline:
            self._fault("Clock rollback or frozen session time budget exhausted")
            raise PairHostError("Pair time budget invalid")
        self.last_clock = now
        if self.active_event_id is not None and self.dispatch_rgb_deadline is not None and now > self.dispatch_rgb_deadline:
            self._fault("RGB shared scene expired before the next guarded operation")
            raise PairHostError("Pair RGB scene is stale")
        if self.fault_event.is_set():
            raise PairHostError("Pair latched or closing; no further target frames")
        if self.quit_event.is_set():
            raise _PairClosing("Pair closing; no further target frames")

    @property
    def capabilities(self):
        return copy.deepcopy(getattr(self.device, "capabilities", {}))

    @property
    def unresolved_gripper_probe(self):
        return copy.deepcopy(getattr(self.device, "unresolved_gripper_probe", None))

    @property
    def grasp_states(self):
        states = getattr(self.device, "grasp_states", None)
        if states is not None:
            return copy.deepcopy(states)
        # Compatibility for observation-only adapters. They cannot retain.
        candidate = self.unresolved_gripper_probe
        return {side: ({**candidate, "status": "contact_candidate"}
                       if candidate and candidate.get("arm") == side else None)
                for side in ("left", "right")}

    def open(self):
        with self.state_lock:
            return self._open_locked()

    def _open_locked(self):
        if self.quit_event.is_set():
            raise PairHostError("Closed host cannot reconnect; resume with a new owner and the same run")
        if self.opened:
            return self.status()
        self.lease = ExclusiveExecution(self.runs)
        self.lease.__enter__()
        claimed = False
        try:
            from .pair_task_enrollment import preparation_only
            if preparation_only(self.runs / "pair_sessions.sqlite", self.run_id) and self.connection_mode != "prepare":
                raise PairHostError("This audited task enrollment requires a fresh preparation connection")
            self.ledger.claim(self.owner)
            claimed = True
            self.device = self.factory(self.profile, lambda event, data: self.journal.append(event, **data), self._guard)
            self._guard()
            connect = (self.device.open if self.connection_mode == "ready"
                       else getattr(self.device, "connect_for_preparation", None))
            if not callable(connect):
                raise PairHostError("Device has no same-connection preparation entry")
            self._sample(connect())
            self._guard()
            self._advance_rgb_floor()
            self.opened = True
            if self.background:
                self.monitor = threading.Thread(target=self._monitor, name="piper-pair-monitor", daemon=True)
                self.monitor.start()
            return self.status()
        except BaseException as exc:
            if claimed:
                self._fault("Pair open failed: " + str(exc))
            if self.device is not None:
                try:
                    self.device.close()
                except Exception:
                    pass
            self.lease.__exit__(None, None, None)
            self.lease = None
            raise

    def _sample(self, sample):
        if (not isinstance(sample, dict) or set(sample.get("arms", {})) != {"left", "right"}
                or sample.get("stationary_observed") is not True
                or sample.get("baseline_duration_s", 0) < 3
                or sample.get("feedback_advances", 0) < 20):
            raise PairHostError("Device has no independent three-second dual-arm stationary receipt")
        now, stamps = self.clock(), []
        for state in sample["arms"].values():
            values = state.get("fragment_timestamps_s", {})
            if not values:
                raise PairHostError("Missing independent feedback timestamps")
            for stamp in values.values():
                if type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= now-stamp <= 0.1:
                    raise PairHostError("Device receipt feedback is stale or invalid")
                stamps.append(stamp)
        if max(stamps)-min(stamps) > 0.1:
            raise PairHostError("Device receipt feedback skew exceeds 100 ms")
        ready = sample.get("task_ready", True if self.connection_mode == "ready" else None)
        if type(ready) is not bool:
            raise PairHostError("Preparation adapter must report its actual task readiness")
        self.task_ready = ready
        self.readiness = copy.deepcopy(sample.get("readiness"))
        return copy.deepcopy(sample)

    def poll(self):
        if (not self.opened or self.quit_event.is_set()
                or (self.active_event_id is not None and not self.fault_event.is_set())):
            return self.status()
        if not self.device_lock.acquire(blocking=False):
            return self.status()
        try:
            # Lifetime/active flags may have changed before this lock was won.
            # Never take state_lock (including status()) while holding it.
            if self.opened and not self.quit_event.is_set():
                if self.fault_event.is_set():
                    self._poll_fault_feedback()
                elif self.active_event_id is None:
                    self._guard()
                    self._sample(self.device.observe())
        except _PairClosing:
            # close() has already blocked new work. A read interrupted by its
            # own guard is not a device fault; other errors still latch below.
            pass
        except Exception as exc:
            self._fault("Idle monitor: " + str(exc))
        finally:
            self.device_lock.release()
        return self.status()

    def _poll_fault_feedback(self):
        """Called with device_lock only; cannot renew a scene or call a guard."""
        try:
            feedback = self.device.observe_fault_feedback()
        except Exception as exc:
            feedback = {"status": "read_error", "observed_at_s": diagnostic_value(self.clock()),
                        "error": type(exc).__name__ + ": " + str(exc),
                        "hardware_commands_sent": 0, "motion_permitted": False,
                        "physical_stop_verified": None}
        feedback = {**feedback, "action_event_pending": self.active_event_id}
        with self._feedback_lock:
            self._fault_feedback = copy.deepcopy(feedback)
        try:
            self.journal.append("pair_fault_feedback", feedback=feedback)
        except Exception as exc:
            # Preserve the fresh in-memory reading even if recording fails.
            # This path is already latched; it never writes another fault.
            self._fault_feedback_journal_error = type(exc).__name__ + ": " + str(exc)

    def _monitor(self):
        while not self.quit_event.wait(0.05):
            self.poll()

    def _advance_rgb_floor(self):
        stamp = self.clock()
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or stamp < 0 or stamp < self.last_clock):
            raise PairHostError("Fresh RGB boundary needs a finite advancing clock")
        self.rgb_not_before = stamp

    def _rgb(self, rgb):
        if not isinstance(rgb, dict) or not isinstance(rgb.get("capture_id"), str):
            raise PairHostError("Current RGB capture metadata required")
        expected = {"front": self.profile["cameras"]["front"],
                    "left_hand": self.profile["cameras"]["left_wrist"],
                    "right_hand": self.profile["cameras"]["right_wrist"]}
        if set(rgb.get("cameras", {})) != set(expected):
            raise PairHostError("All three current RGB camera identities required")
        now, stamps, numbers = self.clock(), [], {}
        for name, serial in expected.items():
            camera = rgb["cameras"][name]
            stamp, number = camera.get("host_received_at"), camera.get("frame_number")
            if (camera.get("serial") != serial or type(number) is not int or number <= self.frame_numbers.get(name, -1)
                    or type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or self.rgb_not_before is None or stamp < self.rgb_not_before
                    or not 0 <= now-stamp <= RGB_MAX_AGE_S
                    or camera.get("depth_enabled", False) is not False):
                raise PairHostError("RGB identity, increasing frame or freshness check failed: " + name)
            stamps.append(stamp)
            numbers[name] = number
        if max(stamps)-min(stamps) > 0.15:
            raise PairHostError("RGB cross-view skew exceeds 150 ms")
        return numbers, min(stamps)

    def observe(self, rgb, *, saved_rgb_evidence=None):
        with self.state_lock:
            if not self.opened or self.active_event_id is not None:
                raise PairHostError("Open idle pair host required for a new shared scene")
            self._guard()
            numbers, stamp = self._rgb(rgb)
            try:
                with self.device_lock:
                    sample = self._sample(self.device.observe())
            except Exception as exc:
                self._fault("Shared-scene feedback failed: " + str(exc))
                raise
            self._guard()
            self.sequence += 1
            scene_id = self.owner + "_scene_" + str(self.sequence)
            receipts = {}
            for side in ("left", "right"):
                receipts[side] = {"receipt_id": "hold_" + uuid.uuid4().hex, "owner": self.owner,
                    "arm": side, "observation_id": scene_id, "sequence": self.sequence,
                    "feedback_digest": _digest(sample["arms"][side]),
                    "retained_target_stationary": True, "contact_support_verified": False,
                    "physical_stop_verified": None}
            self.latest = {"observation_id": scene_id, "capture_id": rgb["capture_id"],
                           "rgb_received_at": stamp, "issued_at": self.clock(),
                           "sample": sample, "peer_receipts": receipts,
                           "saved_rgb_evidence": copy.deepcopy(saved_rgb_evidence)}
            self.frame_numbers = numbers
            self.journal.append("pair_shared_scene", observation=copy.deepcopy(self.latest))
            if callable(getattr(self.joint_sources_provider, "diagnose", None)):
                self.latest["joint_sources_diagnostic"] = self._joint_sources_diagnose()
                self._guard()
            return copy.deepcopy(self.latest)

    def read_state(self):
        if self.fault_event.is_set():
            with self._feedback_lock:
                previous_sequence = (self._fault_feedback or {}).get("sequence")
            status = self.poll()  # Nonblocking with respect to a live action.
            feedback = status["fault_feedback"]
            sequence = (feedback or {}).get("sequence")
            new_read = (type(sequence) is int and
                        (previous_sequence is None or sequence > previous_sequence))
            read_state = status["fault_feedback_read_state"]
            if read_state == "observed" and not new_read:
                read_state = "cached_no_new_read"
            return {"ok": status["open"] and read_state == "observed" and new_read,
                    "state": {"status": "diagnostic", "arms": (feedback or {}).get("arms", {})},
                    "fault_feedback": feedback, "fault_latched": True,
                    "fault_feedback_read_state": read_state, "open": status["open"],
                    "cached": feedback is not None and not new_read,
                    "hardware_commands_sent": 0, "motion_permitted": False,
                    "physical_stop_verified": None, "owner": self.owner}
        with self.state_lock:
            if not self.opened or self.active_event_id is not None:
                raise PairHostError("Pair action active; use pair_status for its receipt")
            try:
                with self.device_lock:
                    sample = self._sample(self.device.observe())
            except Exception as exc:
                self._fault("State read failed: " + str(exc))
                raise
            return {"ok": True, "state": {"status": "complete", "arms": sample["arms"]},
                    "hardware_commands_sent": 0, "motion_permitted": False, "owner": self.owner,
                    "task_ready": self.task_ready, "readiness": copy.deepcopy(self.readiness)}

    def _joint_bindings(self):
        """Facts from this adapter plus its frozen profile, never scene input."""
        if not callable(getattr(self.device, "joint_binding", None)):
            raise PairHostError("Device has no current joint connection bindings")
        bindings = {}
        for side in ("left", "right"):
            current, config = self.device.joint_binding(side), self.profile["arms"][side]
            bindings[side] = {key: current[key] for key in ("connection_id", "model", "firmware_profile")}
            bindings[side].update({key: config[key] for key in ("channel", "usb_interface")})
        return bindings

    def _joint_sources_diagnose(self, arm="right"):
        """Explicit source inspection only; status/poll never hash source files."""
        with self.state_lock:
            started = self.clock()
            diagnose = getattr(self.joint_sources_provider, "diagnose", None)
            if not callable(diagnose) or self.latest is None:
                result = {"ready": False, "gaps": ["source_provider_or_current_scene_missing"]}
            else:
                try:
                    scene = copy.deepcopy(self.latest)
                    scene["joint_source_bindings"] = self._joint_bindings()
                    result = diagnose(scene, arm)
                except Exception as exc:
                    result = {"ready": False, "gaps": [str(exc)]}
            if type(result) is not dict:
                result = {"ready": False, "gaps": ["source_provider_returned_invalid_diagnostic"]}
            result = {**result, "inspection_started_at": diagnostic_value(started),
                      "inspection_finished_at": diagnostic_value(self.clock())}
            self.joint_sources_diagnostic = copy.deepcopy(result)
            return result

    @staticmethod
    def _target(kind, target):
        if kind == "joint":
            from .arms import _six_finite
            from .joint_path import encode_joint_target
            result = _six_finite(target)
            encode_joint_target(result)
            return result
        if kind == "move":
            from .linear_hold import _pose_frames
            from .arms import _six_finite
            result = _six_finite(target)
            _pose_frames(result)
            return result
        if kind == "gripper" and type(target) in (int, float) and math.isfinite(target) and 0 <= target <= 0.055:
            return float(target)
        raise PairHostError("Expected a bounded move pose, six joint radians or 0..55 mm gripper width")

    def _joint_context(self, event_id, payload, scene):
        """Resolve non-proposal facts before claiming the physical attempt."""
        from .joint_path import SCHEMA, COARSE_PROFILE_LIMITS, evidence_sha256, plan_joint_path, encode_joint_target
        loaded = payload.get("loaded_observation") is not None
        coarse = payload.get("motion_profile") == "coarse_approach"
        if payload["operation"] not in ("approach", "align", "release_retreat", "extract_segment", "transport", "insert_segment"):
            raise PairHostError("Joint transport covers unloaded approach/align and confirmed-release retreat")
        if not callable(self.joint_sources_provider):
            raise PairHostError("Joint scene source adapter unavailable: official model, controller limits "
                                "and current attachment/corridor evidence must be host-resolved; "
                                "initialize the first target on this same host when its separate contract applies")
        if not callable(getattr(self.device, "execute_joint", None)):
            raise PairHostError("Device has no persistent joint transport")
        arm = payload["arm"]
        visual = payload.get("admission_mode") == "rgb_supervised"
        if visual:
            scene = copy.deepcopy(self._saved_preparation_scene(payload["observation_id"], arm))
        binding = self.device.joint_binding(arm)
        if binding["cached_target"] is None:
            raise PairHostError("Joint bootstrap required: no complete same-connection MOVE_J cache history; "
                                "use the explicit unloaded initialize_joint_target entry when applicable")
        identity = {"run_id": self.run_id, "owner": self.owner, "epoch": self.owner,
                    "worker_id": event_id, "arm": arm,
                    **{key: binding[key] for key in ("connection_id", "model", "firmware_profile")}}
        resolved_scene = copy.deepcopy(scene)
        resolved_scene["joint_source_bindings"] = self._joint_bindings()
        resolver = (getattr(self.joint_sources_provider, "rgb_joint_basis", None)
                    if visual else self.joint_sources_provider)
        if not callable(resolver):
            raise PairHostError("Source provider has no explicit ordinary RGB joint basis")
        sources = resolver(resolved_scene, arm)
        if visual:
            refresh = self._rgb_dispatch_window(scene, event_id, "after_source_resolution", payload.get("motion_profile"))
            if refresh is not None:
                raise _RefreshRGBRequired(refresh)
        required_sources = {"model_catalog", "urdf_source", "controller_limits"}
        if not visual:
            required_sources.add("geometry")
        if type(sources) is not dict or set(sources) != required_sources:
            raise PairHostError("Joint source adapter returned the wrong sources for the explicit admission mode")
        sources = copy.deepcopy(sources)
        origin = self.device.observe_joint(identity)
        if visual:
            from .rgb_supervision import JOINT_PATH_SCHEMA, LOADED_JOINT_PATH_SCHEMA, COARSE_JOINT_PATH_SCHEMA
            evidence = {"identity": copy.deepcopy(identity), "operation": payload["operation"],
                "target_raw": encode_joint_target(payload["target"])[0],
                "observation_id": scene["observation_id"], "capture_id": scene["capture_id"],
                "saved_rgb_evidence": copy.deepcopy(scene["saved_rgb_evidence"]),
                "rgb_received_at": scene["rgb_received_at"],
                "corridor_observation": payload["corridor_observation"],
                "workspace_clearance_statement": self.task["site_context"]["workspace_clearance"]["statement"]}
            if loaded:
                loaded_context = self.loaded.context(event_id, payload, scene)
                evidence.update(loaded_observation=payload["loaded_observation"], loaded_context_sha256=evidence_sha256(loaded_context))
            else:
                evidence["unloaded_observation"] = payload["unloaded_observation"]
            if coarse:
                evidence["far_from_target_observation"] = payload["far_from_target_observation"]
            geometry_schema = (COARSE_JOINT_PATH_SCHEMA if coarse else
                               LOADED_JOINT_PATH_SCHEMA if loaded else JOINT_PATH_SCHEMA)
            sources["geometry"] = {"schema": geometry_schema, "origin_sample_id": origin["sample_id"],
                "evidence": evidence, "source": {"ref": "pair_event:"+event_id+":rgb_supervised_joint",
                                               "sha256": evidence_sha256(evidence)}}
        context = {"schema": SCHEMA, "identity": identity, **copy.deepcopy(sources),
                   "origin": origin, "origin_sha256": evidence_sha256(origin), "current": copy.deepcopy(origin),
                   "cached_target": copy.deepcopy(binding["cached_target"]),
                   "budget": {"max_translation_m": COARSE_PROFILE_LIMITS["process_translation_m"] if coarse else .020,
                              "max_rotation_rad": COARSE_PROFILE_LIMITS["process_rotation_rad"] if coarse else .05}, "unloaded_evidence": None}
        context["geometry"]["origin_sample_id"] = origin["sample_id"]
        if loaded:
            context["loaded_context"] = loaded_context
        if visual and not loaded:
            context["unloaded_evidence"] = {"origin_sample_id": origin["sample_id"],
                                           "source": copy.deepcopy(sources["geometry"]["source"])}
        if self.feedback_policy is not None:
            context["feedback_observation"] = copy.deepcopy(self.feedback_policy)
        initialization_sources = getattr(self.device, "joint_initialization_sources", None)
        if callable(initialization_sources):
            context["initialization_sources"] = initialization_sources()
        plan_joint_path(context, payload["target"], now=self.clock())
        if visual:
            self._guard()
            current_scene = self._saved_preparation_scene(payload["observation_id"], arm)
            if (current_scene["saved_rgb_evidence"] != scene["saved_rgb_evidence"]
                    or self.device.joint_binding(arm) != binding
                    or self._joint_bindings() != resolved_scene["joint_source_bindings"]):
                raise PairHostError("RGB joint sources or live target history changed before claim")
            self._check_guard_context()
        return context

    def _preparation_replay(self, event_id, request):
        key = _digest(request)
        if event_id in self.actions:
            if self.actions[event_id]["digest"] != key:
                raise PairHostError("Existing event id has a different request")
            return {**self.status(event_id), "replayed": True}
        stored = self.ledger.event(event_id)
        if stored is None:
            return None
        if _digest(stored["payload"].get("request")) != key:
            raise PairHostError("Existing durable event has a different request")
        if stored["status"] != "complete":
            self._fault("Uncertain prior preparation/query dispatch cannot be retried")
            raise PairHostError("Prior preparation/query event is pending; it cannot be resent")
        self.actions[event_id] = {"digest": key,
            "status": "completed" if stored["success"] else "fault", "receipt": stored["receipt"]}
        return {**self.status(event_id), "replayed": True}

    @staticmethod
    def _known_preparation_required(receipt):
        return (receipt.get("status") == "preparation_required" and receipt.get("ok") is False
                and type(receipt.get("hardware_commands_sent")) is int
                and receipt["hardware_commands_sent"] == 0 and receipt.get("fault_latched") is False
                and type(receipt.get("requirements")) is list and bool(receipt["requirements"]))

    def _preparation_idle(self, *, recovery_observation=False):
        if not self.opened or self.active_event_id is not None:
            raise PairHostError("Open idle pair host required for preparation or queries")
        self._guard()
        if not recovery_observation:
            from .host_recovery import SupportedRecovery
            SupportedRecovery(self).require_resolved()
        if any(self.grasp_states.values()) or any(
                state["status"] != "empty" and not is_resolved_release(state) for state in self.grasps.states()):
            raise PairHostError("Preparation/query cannot run with an unresolved or retained grasp")

    def _rgb_dispatch_window(self, scene, event_id, stage, motion_profile=None):
        """Reject an impossible RGB window before claiming any physical attempt.

        Reserve the baseline plus the finite-motion/IO and final-stability
        minimum. This does not promise completion or reserve the full maximum
        observation timeout; the original live deadlines remain authoritative.
        No scene timestamp, task budget, event or grasp state is changed.
        """
        from .pair_joint_adapter import rgb_joint_execution_window_budget
        self._check_guard_context()
        deadline = scene["rgb_received_at"] + RGB_MAX_AGE_S
        remaining = deadline - self.last_clock
        budget = rgb_joint_execution_window_budget(motion_profile)
        required = budget["preclaim_required_s"]
        if remaining > required:
            return None
        return {"status": "refresh_required", "reason": "insufficient_rgb_execution_window",
            "event_id": event_id, "observation_id": scene["observation_id"], "stage": stage,
            "rgb_deadline": deadline, "remaining_rgb_window_s": remaining,
            "required_rgb_window_s": required, "window_is_completion_guarantee": False,
            "execution_window_budget": budget,
            "hardware_commands_sent": 0, "event_claimed": False, "steps_consumed": 0,
            "fault_latched": False, "automatic_retry": False}

    def _saved_preparation_scene(self, observation_id, arm):
        scene = self.latest
        if scene is None or scene["observation_id"] != observation_id:
            raise PairHostError("Preparation requires the current host-issued RGB scene")
        saved = scene.get("saved_rgb_evidence")
        if type(saved) is not dict or set(saved) != {"front", "left_hand", "right_hand"}:
            raise PairHostError("Empty-jaw observation requires all three saved RGB artifacts")
        stamps = []
        for view, source in saved.items():
            path = Path(source["rgb_path"])
            if not path.is_file() or path.stat().st_size > 32*1024*1024:
                raise PairHostError("Preparation RGB artifact is missing or oversized")
            raw = path.read_bytes()
            if (len(raw) > 32*1024*1024 or hashlib.sha256(raw).hexdigest() != source["artifact_sha256"]
                    or source.get("frame_number") != self.frame_numbers[view]):
                raise PairHostError("Preparation RGB artifact/frame changed after observation")
            stamp = source.get("host_received_at")
            if type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise PairHostError("Preparation RGB timestamp must be finite")
            stamps.append(stamp)
        peer = "right" if arm == "left" else "left"
        receipt = scene["peer_receipts"][peer]
        if (min(stamps) != scene["rgb_received_at"] or max(stamps)-min(stamps) > .15
                or receipt["owner"] != self.owner or receipt["observation_id"] != observation_id
                or not 0 <= self.clock()-scene["rgb_received_at"] <= RGB_MAX_AGE_S):
            raise PairHostError("Preparation requires current saved RGB and an independent peer receipt")
        return scene

    def _start_preparation(self, event_id, request, payload, rgb_deadline=None):
        self._guard()
        # Native SDK feedback contains IntEnum values. Persist and execute the
        # same JSON value (integers), without changing its canonical hash.
        payload = json.loads(json.dumps(payload, allow_nan=False))
        try:
            claim = self.ledger.begin(self.owner, event_id, payload)
        except BaseException as exc:
            self._fault("Preparation claim failed or uncertain: " + str(exc))
            raise
        if claim["replayed"]:
            self.actions[event_id] = {"digest": _digest(request), "status": "completed", "receipt": claim["receipt"]}
            return {**self.status(event_id), "replayed": True}
        self.actions[event_id] = {"digest": _digest(request), "status": "pending", "receipt": None}
        self.active_event_id, self.dispatch_rgb_deadline = event_id, rgb_deadline
        try:
            self.worker = threading.Thread(target=self._execute_preparation, args=(event_id, payload),
                                           name="piper-pair-preparation", daemon=True)
            self.worker.start()
        except BaseException as exc:
            self._fault("Claimed preparation worker did not start: " + str(exc))
            raise
        return {"status": "pending", "event_id": event_id, "replayed": False}

    def prepare_gripper(self, event_id, observation_id, arm, empty_jaw_observation):
        event_id = _identifier(event_id, "event_id")
        if arm not in ("left", "right"):
            raise PairHostError("Explicit left/right arm required for jaw preparation")
        if (type(empty_jaw_observation) is not str
                or not 1 <= len(empty_jaw_observation.strip()) <= 4000):
            raise ValueError("Describe the current RGB evidence of the selected empty jaw and clearance")
        request = {"operation": "prepare_gripper", "observation_id": observation_id,
                   "arm": arm, "empty_jaw_observation": empty_jaw_observation}
        with self.state_lock:
            replay = self._preparation_replay(event_id, request)
            if replay is not None:
                return replay
            self._preparation_idle()
            if not callable(getattr(self.device, "prepare_gripper", None)):
                raise PairHostError("Device has no same-connection jaw preparation")
            scene = self._saved_preparation_scene(observation_id, arm)
            try:
                with self.device_lock:
                    self._sample(self.device.observe())
            except Exception as exc:
                self._fault("Preparation peer feedback failed: " + str(exc))
                raise
            peer = "right" if arm == "left" else "left"
            payload = {"kind": "preparation", "request": request,
                       "scene": {key: copy.deepcopy(scene[key]) for key in
                                 ("observation_id", "capture_id", "rgb_received_at", "saved_rgb_evidence")},
                       "peer_receipt": copy.deepcopy(scene["peer_receipts"][peer]),
                       "empty_jaw_evidence_kind": "model_RGB_semantic_observation_not_sensor_verification"}
            return self._start_preparation(event_id, request, payload,
                                           scene["rgb_received_at"] + RGB_MAX_AGE_S)

    def publish_geometry(self, observation_id, record_set_id):
        """Import actual site records; no measurement, device send or readiness change."""
        if (type(record_set_id) is not str or not 1 <= len(record_set_id) <= 96
                or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
                       for c in record_set_id)):
            raise ValueError("Expected a local geometry record-set identifier")
        with self.state_lock:
            self._preparation_idle()
            publish = getattr(self.joint_sources_provider, "publish_geometry", None)
            if not callable(publish):
                raise PairHostError("No host geometry record publisher is installed")
            scene = copy.deepcopy(self._saved_preparation_scene(observation_id, "left"))
            scene["joint_source_bindings"] = self._joint_bindings()
            # state_lock keeps a new proposal/scene/close from interleaving.
            # Local-file import needs neither device lock nor observe/send.
            result = publish(scene, record_set_id)
            self._guard()
            self._saved_preparation_scene(observation_id, "left")
            if scene["joint_source_bindings"] != self._joint_bindings():
                raise PairHostError("Geometry publication outlasted its device connections")
            if (type(result) is not dict or type(result.get("hardware_commands_sent")) is not int
                    or result["hardware_commands_sent"] != 0
                    or result.get("dispatch_authorized") is not False):
                raise PairHostError("Geometry publisher returned an invalid non-dispatch receipt")
            self.journal.append("pair_geometry_published", observation_id=observation_id,
                                record_set_id=record_set_id, publication=copy.deepcopy(result))
            diagnostic = self._joint_sources_diagnose()
            self._guard()
            self._saved_preparation_scene(observation_id, "left")
            if scene["joint_source_bindings"] != self._joint_bindings():
                raise PairHostError("Geometry inspection outlasted its device connections")
            return {**copy.deepcopy(result), "observation_id": observation_id,
                    "record_set_id": record_set_id, "task_ready": self.task_ready,
                    "joint_sources_diagnostic": diagnostic}

    def inspect_joint_limits(self, event_id):
        event_id = _identifier(event_id, "event_id")
        request = {"operation": "inspect_joint_limits"}
        with self.state_lock:
            replay = self._preparation_replay(event_id, request)
            if replay is not None:
                return replay
            self._preparation_idle()
            if not callable(getattr(self.device, "inspect_joint_limits", None)):
                raise PairHostError("Device has no same-connection joint limit query implementation")
            bindings = self._joint_bindings()
            return self._start_preparation(event_id, request,
                {"kind": "query", "request": request, "bindings": bindings})

    def initialize_joint_target(self, event_id, observation_id, arm, unloaded_observation,
                                admission_mode="metric_geometry", corridor_observation=None):
        """Explicit unloaded first target, using the existing supervised startup scope.

        This is not an ordinary path or a stop. The host resolves all numerical
        sources and freezes the automatically selected seed/boundary target.
        Explicit RGB supervision is an alternative first-target contract, not
        a fabricated geometry source or a fallback for ordinary joint motion.
        The public request supplies current RGB semantics, never target/cache.
        """
        event_id = _identifier(event_id, "event_id")
        if arm not in ("left", "right"):
            raise PairHostError("Explicit left/right arm required for initialization")
        if (type(unloaded_observation) is not str
                or not 1 <= len(unloaded_observation.strip()) <= 4000):
            raise ValueError("Describe both currently empty jaws and absence of object contact in the RGB scene")
        if admission_mode not in ("metric_geometry", "rgb_supervised"):
            raise ValueError("Explicit metric_geometry or rgb_supervised initialization required")
        visual = admission_mode == "rgb_supervised"
        if visual:
            if (type(corridor_observation) is not str
                    or not 1 <= len(corridor_observation.strip()) <= 4000):
                raise ValueError("Describe this unloaded initialization corridor, including both arms, attachments, cables and table, from current RGB")
        elif corridor_observation is not None:
            raise ValueError("A visual corridor description requires explicit rgb_supervised initialization")
        request = {"operation": "initialize_joint_target", "observation_id": observation_id,
                   "arm": arm, "unloaded_observation": unloaded_observation}
        if visual:
            request.update(admission_mode=admission_mode, corridor_observation=corridor_observation)
        with self.state_lock:
            replay = self._preparation_replay(event_id, request)
            if replay is not None:
                return replay
            self._preparation_idle()
            if (not callable(getattr(self.device, "initialize_joint_target", None))
                    or not callable(getattr(self.device, "observe_initialization", None))):
                raise PairHostError("Device has no same-connection joint initialization")
            binding = self.device.joint_binding(arm)
            bindings = self._joint_bindings()
            existing = binding["cached_target"]
            if existing is not None:
                # No new target, geometry, image or unloaded evidence is needed
                # to report an already established local cache. The worker
                # still checks live anchors and records a zero-TX result.
                return self._start_preparation(event_id, request,
                    {"kind": "initialization", "request": request, "bindings": bindings,
                     "context": None, "existing_cached_target": copy.deepcopy(existing),
                     "expected_target_raw": copy.deepcopy(existing["target_raw"])})
            scene = self._saved_preparation_scene(observation_id, arm)
            if visual:
                refresh = self._rgb_dispatch_window(scene, event_id, "before_source_resolution")
                if refresh is not None:
                    return refresh
            if not callable(self.joint_sources_provider):
                raise PairHostError("Initialization requires host-resolved official model, current limits and geometry")
            resolved_scene = copy.deepcopy(scene)
            resolved_scene["joint_source_bindings"] = bindings
            resolver = (getattr(self.joint_sources_provider, "initialization_basis", None)
                        if visual else self.joint_sources_provider)
            if not callable(resolver):
                raise PairHostError("Source provider has no explicit RGB-supervised initialization basis")
            sources = resolver(resolved_scene, arm)
            if visual:
                refresh = self._rgb_dispatch_window(scene, event_id, "after_source_resolution")
                if refresh is not None:
                    return refresh
            required_sources = {"model_catalog", "urdf_source", "controller_limits"}
            if not visual:
                required_sources.add("geometry")
            if (type(sources) is not dict or set(sources) !=
                    required_sources):
                raise PairHostError("Initialization source adapter returned the wrong sources for the explicit admission mode")
            identity = {"run_id": self.run_id, "owner": self.owner, "epoch": self.owner,
                        "worker_id": event_id, "arm": arm,
                        **{key: binding[key] for key in ("connection_id", "model", "firmware_profile")}}
            with self.device_lock:
                try:
                    origin = self.device.observe_initialization(identity)
                except Exception as exc:
                    self._fault("Initialization original feedback failed: " + str(exc))
                    raise
            from .joint_initialization import SCHEMA, plan_joint_initialization
            from .joint_path import evidence_sha256
            semantic_evidence = {"observation_id": observation_id,
                "saved_rgb_evidence": copy.deepcopy(scene["saved_rgb_evidence"]),
                "description": unloaded_observation,
                "kind": "model_RGB_semantic_observation_not_sensor_verification"}
            if visual:
                visual_evidence = {"identity": copy.deepcopy(identity),
                    "observation_id": observation_id, "capture_id": scene["capture_id"],
                    "saved_rgb_evidence": copy.deepcopy(scene["saved_rgb_evidence"]),
                    "rgb_received_at": scene["rgb_received_at"],
                    "unloaded_observation": unloaded_observation,
                    "corridor_observation": corridor_observation,
                    "workspace_clearance_statement": self.task["site_context"]["workspace_clearance"]["statement"]}
                sources["geometry"] = {"schema": "piper_rgb_supervised_initialization_v1",
                    "origin_sample_id": origin["sample_id"], "evidence": visual_evidence,
                    "source": {"ref": "pair_event:"+event_id+":rgb_supervised_initialization",
                               "sha256": evidence_sha256(visual_evidence)}}
            context = {"schema": SCHEMA, "identity": identity, **copy.deepcopy(sources),
                       "origin": origin, "current": copy.deepcopy(origin),
                       "origin_sha256": evidence_sha256(origin),
                       "unloaded_evidence": {"origin_sample_id": origin["sample_id"],
                           "source": {"ref": "pair_event:"+event_id+":unloaded_observation",
                                      "sha256": evidence_sha256(semantic_evidence)}}}
            context["geometry"]["origin_sample_id"] = origin["sample_id"]
            if self.feedback_policy is not None:
                context["feedback_observation"] = copy.deepcopy(self.feedback_policy)
            plan = plan_joint_initialization(context, now=self.clock())
            peer = "right" if arm == "left" else "left"
            payload = {"kind": "initialization", "request": request, "bindings": bindings,
                       "context": context, "existing_cached_target": None,
                       "expected_target_raw": copy.deepcopy(plan["target_raw"]),
                       "unloaded_evidence": semantic_evidence,
                       "peer_receipt": copy.deepcopy(scene["peer_receipts"][peer])}
            # Include source/image IO and pure planning in the original scene
            # window. A caller's text is testimony, never independent proof.
            self._guard()
            if visual:
                current_scene = self._saved_preparation_scene(observation_id, arm)
                if current_scene["saved_rgb_evidence"] != scene["saved_rgb_evidence"]:
                    raise PairHostError("RGB supervision source changed before initialization claim")
                self._check_guard_context()
                refresh = self._rgb_dispatch_window(scene, event_id, "before_claim")
                if refresh is not None:
                    return refresh
            return self._start_preparation(event_id, request, payload,
                                           scene["rgb_received_at"]+RGB_MAX_AGE_S)

    def _finish_initialization(self, event_id, payload):
        """Worker-only receipt checks; physical transmission is in the adapter."""
        arm = payload["request"]["arm"]
        if self._joint_bindings() != payload["bindings"]:
            raise PairHostError("Device binding changed before first-target initialization")
        result = self.device.initialize_joint_target(arm, context=payload["context"],
            event_id=event_id, deadline_at=self.deadline)
        receipt = json.loads(json.dumps(result, allow_nan=False))
        if type(receipt) is not dict:
            return {"ok": False, "invalid_initialization_receipt": receipt}, False
        # A failed/partial result is returned intact to the caller's fault path.
        # Do not throw here and lose its per-frame diagnostic receipt.
        count = 0 if payload["existing_cached_target"] is not None else 4
        expected_status = "already_initialized" if count == 0 else "joint_target_initialized"
        valid = (receipt.get("ok") is True and receipt.get("status") == expected_status
                 and receipt.get("cache_established") is True
                 and type(receipt.get("hardware_commands_sent")) is int
                 and receipt["hardware_commands_sent"] == count
                 and receipt.get("passive_arm_commands_sent") == 0
                 and receipt.get("gripper_commands_sent") == 0
                 and receipt.get("accepted") is None and receipt.get("physical_stop_verified") is None)
        cached = self.device.joint_binding(arm)["cached_target"]
        valid = (valid and type(cached) is dict and receipt.get("cached_target") == cached
                 and cached.get("target_raw") == payload["expected_target_raw"]
                 and self._joint_bindings() == payload["bindings"])
        if count == 0:
            valid = valid and cached == payload["existing_cached_target"]
        else:
            valid = (valid and cached.get("event_id") == event_id
                     and cached.get("identity") == payload["context"]["identity"]
                     and cached.get("frame_receipts") == receipt.get("frame_receipts")
                     and type(receipt.get("initialization_plan")) is dict
                     and receipt["initialization_plan"].get("target_raw") == payload["expected_target_raw"])
            if payload["request"].get("admission_mode") == "rgb_supervised":
                valid = (valid and receipt["initialization_plan"].get("geometry") == payload["context"]["geometry"]
                         and receipt["initialization_plan"].get("metric_clearance_checked") is False
                         and receipt["initialization_plan"].get("absolute_workspace_checked") is False)
            if valid:
                from .hold_transaction import joint_hold_frames
                expected = joint_hold_frames(payload["expected_target_raw"])
                frames = cached.get("frame_receipts")
                valid = type(frames) is list and len(frames) == 4
                previous = 0.
                for item, frame in zip(frames if valid else [], expected):
                    at = item.get("returned_at") if type(item) is dict else None
                    if (type(item) is not dict or item.get("frame") != frame
                            or item.get("outcome") != "returned"
                            or type(at) not in (int, float) or not math.isfinite(at)
                            or not 0 < at <= self.clock() or at < previous):
                        valid = False
                        break
                    previous = at
        return receipt, bool(valid)

    def _execute_preparation(self, event_id, payload):
        receipt = None
        operation = payload["request"]["operation"]
        try:
            with self.device_lock:
                self._guard()
                self.journal.append("pair_preparation_claimed", event_id=event_id, payload=payload)
                self._guard()
                if operation == "prepare_gripper":
                    arm = payload["request"]["arm"]
                    receipt = self.device.prepare_gripper(arm)
                    receipt = json.loads(json.dumps(receipt, allow_nan=False))
                    count = receipt.get("hardware_commands_sent")
                    known_missing = self._known_preparation_required(receipt)
                    if not known_missing:
                        if (receipt.get("ok") is not True or type(count) is not int or count not in (0, 1)
                                or receipt.get("grasp_verified") is not False
                                or receipt.get("physical_stop_verified") is not None
                                or receipt.get("accepted") is not None):
                            raise PairHostError("Jaw preparation did not establish a definite bounded result")
                        if count == 1 and (receipt.get("passive_arm_commands_sent") != 0
                                or receipt.get("arm_target_commands_sent") != 0
                                or receipt.get("mode_commands_sent") != 0
                                or receipt.get("enable_commands_sent") != 0
                                or receipt.get("gripper_enable_commands_sent") != 1):
                            raise PairHostError("Jaw preparation exceeded its single-gripper scope")
                    sample = self._sample(self.device.observe())
                    if not known_missing:
                        state = sample["arms"][arm]
                        if (state["arm_status"]["ctrl_mode"] != 1
                                or state["gripper"]["foc_status"].get("driver_enable_status") is not True
                                or not all(state["drivers"][str(i)]["foc_status"].get("driver_enable_status") is True
                                           for i in range(1, 7))):
                            raise PairHostError("Preparation result lacks fresh selected-jaw enabled feedback")
                    receipt["sample"] = sample
                elif operation == "initialize_joint_target":
                    receipt, valid = self._finish_initialization(event_id, payload)
                    if not valid:
                        raise PairHostError("First-target initialization incomplete, uncertain or inconsistent")
                    receipt["sample"] = self._sample(self.device.observe())
                    self.joint_sources_diagnostic = None
                elif operation == "inspect_joint_limits":
                    receipt = json.loads(json.dumps(self.device.inspect_joint_limits(), allow_nan=False))
                    if not self._known_preparation_required(receipt):
                        if (receipt.get("ok") is not True
                                or any(type(receipt.get(key)) is not int or receipt[key] != expected
                                       for key, expected in (("hardware_commands_sent", 12),
                                                             ("joint_limit_queries_sent", 12),
                                                             ("actuator_commands_sent", 0)))):
                            raise PairHostError("Controller limit query incomplete or uncertain")
                        if self._joint_bindings() != payload["bindings"]:
                            raise PairHostError("Device binding changed during controller limit queries")
                        from .joint_sources import publish_controller_limits
                        capture = {**receipt, "run_id": self.run_id, "owner": self.owner,
                                   "bindings": payload["bindings"]}
                        publication = publish_controller_limits(self.runs, self.profile, self.run_id,
                            self.owner, payload["bindings"], capture, clock=self.clock)
                        receipt["source_publication"] = publication
                        self.joint_sources_diagnostic = None
                    receipt["sample"] = self._sample(self.device.observe())
                elif operation in ("supported_recovery_open", "supported_recovery_confirm"):
                    from .host_recovery import SupportedRecovery
                    receipt = SupportedRecovery(self).execute(operation, payload)
                    if receipt.get("ok") is not True:
                        raise PairHostError("Supported jaw recovery failed; no automatic retry")
                elif operation == "supported_contact_observe":
                    from .host_recovery import SupportedRecovery
                    receipt = SupportedRecovery(self).execute_contact_observation(payload)
                    if receipt.get("ok") is not True:
                        raise PairHostError("Current supported contact observation failed; no automatic retry")
                else:
                    raise PairHostError("Unknown claimed preparation operation")
                self._guard()
                receipt = json.loads(json.dumps({**receipt, "pair_owner": self.owner, "event_id": event_id,
                    "execution_mode": operation, "object_task_success": None,
                    "physical_stop_verified": None}, allow_nan=False))
                self.ledger.finish(self.owner, event_id, receipt, success=True)
                if operation == "supported_contact_observe":
                    self.grasps.record_observed_candidate(event_id, payload, receipt)
        except BaseException as exc:
            self._fault("Claimed preparation/query failed or uncertain: " + str(exc))
            receipt = {"ok": False, "event_id": event_id, "error": str(exc),
                       "device_receipt": receipt, "physical_stop_verified": None, "automatic_retry": False}
            receipt = self._finish_failed_receipt(event_id, receipt)
        finally:
            with self.state_lock:
                try:
                    self._advance_rgb_floor()
                except Exception as exc:
                    self.rgb_not_before = None
                    self._fault("Cannot establish post-preparation RGB boundary: " + str(exc))
                self.actions[event_id].update(status="fault" if self.fault_event.is_set() else "completed", receipt=receipt)
                self.active_event_id, self.dispatch_rgb_deadline, self.latest = None, None, None

    def promote_ready(self):
        with self.state_lock:
            self._preparation_idle(recovery_observation=True)
            if not callable(getattr(self.device, "promote_ready", None)):
                raise PairHostError("Device has no same-connection readiness transition")
            try:
                with self.device_lock:
                    result = self.device.promote_ready()
                    if self._known_preparation_required(result):
                        self.readiness = copy.deepcopy(result.get("readiness"))
                        self._guard()
                        return {**result, "owner": self.owner, "run_id": self.run_id}
                    self._sample(result)
                    if not self.task_ready:
                        raise PairHostError("Device did not establish original task readiness")
                self._guard()
                self.journal.append("pair_ready_observed", task_ready=True, hardware_commands_sent=0)
                return {**self.status(), "hardware_commands_sent": 0}
            except BaseException as exc:
                self._fault("Readiness transition failed: " + str(exc))
                raise

    def submit(self, event_id, observation_id, peer_receipt_id, arm, kind, target, operation="approach",
               *, grasp_object_id=None, release_support_observation=None, release_support_relation=None,
               release_retreat_observation=None, admission_mode=None,
               unloaded_observation=None, corridor_observation=None, loaded_observation=None,
               source_object_id=None, target_object_id=None, motion_profile=None,
               far_from_target_observation=None, probe_support_observation=None,
               probe_support_relation=None):
        from .host_recovery import SupportedRecovery
        recovery = SupportedRecovery(self).admit_probe(arm, kind, operation, grasp_object_id,
            probe_support_observation, probe_support_relation)
        event_id = _identifier(event_id, "event_id")
        if arm not in ("left", "right") or operation not in OPERATIONS:
            raise PairHostError("Explicit arm and supported operation required")
        if self.feedback_policy is not None and kind == "move":
            raise PairHostError("Task observation profile requires RGB-supervised joint motion")
        target = self._target(kind, target)
        payload = {"observation_id": observation_id, "peer_receipt_id": peer_receipt_id,
                   "arm": arm, "kind": kind, "target": target, "operation": operation}
        if probe_support_observation is not None or probe_support_relation is not None:
            if (kind != 'gripper' or operation != 'grip_supported'
                    or type(probe_support_observation) is not str
                    or not 1 <= len(probe_support_observation.strip()) <= 4000
                    or '\x00' in probe_support_observation
                    or probe_support_relation != 'independent_support_present'):
                raise PairHostError('Support description applies only to a supported jaw probe')
            payload.update(probe_support_observation=probe_support_observation,
                           probe_support_relation=probe_support_relation)
        if recovery is not None:
            payload['reacquisition_proposal_sha256'] = recovery['proposal']['proposal_sha256']
        loaded = any(v is not None for v in (loaded_observation, source_object_id, target_object_id))
        coarse = motion_profile is not None or far_from_target_observation is not None
        if coarse:
            if (motion_profile != "coarse_approach" or kind != "joint" or operation != "approach"
                    or admission_mode != "rgb_supervised" or loaded
                    or any(v is not None for v in (grasp_object_id, release_support_observation,
                        release_support_relation, release_retreat_observation))):
                raise PairHostError("coarse_approach is only an explicit unloaded RGB joint approach")
            if (type(far_from_target_observation) is not str
                    or not 1 <= len(far_from_target_observation.strip()) <= 4000
                    or "\x00" in far_from_target_observation):
                raise PairHostError("Coarse approach requires a current far-from-target observation")
            payload.update(motion_profile=motion_profile,
                           far_from_target_observation=far_from_target_observation)
        if kind == "joint" and operation in ("extract_segment", "transport", "insert_segment") and not loaded:
            raise PairHostError("Loaded joint operations require their explicit current RGB grasp branch")
        if loaded:
            from .loaded_episode import OPERATIONS as LOADED_OPERATIONS
            if (arm != resolve_task_roles(self.task)[0] or kind != "joint" or operation not in LOADED_OPERATIONS
                    or admission_mode != "rgb_supervised" or unloaded_observation is not None):
                raise PairHostError("Explicit RGB loaded frozen-worker joint extract/transport/insert required")
            for description in (loaded_observation, corridor_observation):
                if type(description) is not str or not 1 <= len(description.strip()) <= 4000 or "\x00" in description:
                    raise PairHostError("Describe current retained plug, fixed strip and whole-arm corridor")
            source_object_id, target_object_id = (_identifier(value, name) for value, name in
                ((source_object_id, "source_object_id"), (target_object_id, "target_object_id")))
            if source_object_id == target_object_id:
                raise PairHostError("Freeze distinct source and target socket identities")
            payload.update(admission_mode=admission_mode, loaded_observation=loaded_observation,
                           corridor_observation=corridor_observation, source_object_id=source_object_id,
                           target_object_id=target_object_id)
        elif admission_mode is not None or unloaded_observation is not None or corridor_observation is not None:
            if kind != "joint" or operation not in ("approach", "align", "release_retreat"):
                raise PairHostError("Explicit spatial admission is only supported for ordinary joint approach/align")
            if admission_mode not in ("metric_geometry", "rgb_supervised"):
                raise PairHostError("Select an explicit metric_geometry or rgb_supervised joint admission mode")
            if admission_mode == "rgb_supervised":
                if operation == "release_retreat" and unloaded_observation is None:
                    unloaded_observation = release_retreat_observation
                for description in (unloaded_observation, corridor_observation):
                    if (type(description) is not str or not 1 <= len(description.strip()) <= 4000
                            or "\x00" in description):
                        raise PairHostError("RGB joint motion requires current selected-arm unloaded and whole-arm corridor descriptions")
                payload.update(admission_mode=admission_mode, unloaded_observation=unloaded_observation,
                               corridor_observation=corridor_observation)
            elif unloaded_observation is not None or corridor_observation is not None:
                raise PairHostError("RGB descriptions require the explicit rgb_supervised branch")
        if grasp_object_id is not None:
            if operation != "grip_supported" or kind != "gripper":
                raise PairHostError("Object identity is only used to start a supported jaw probe episode")
            payload["grasp_object_id"] = _identifier(grasp_object_id, "grasp_object_id")
        if release_support_observation is not None or release_support_relation is not None:
            if (operation != "release_retreat" or kind != "gripper"
                    or type(release_support_observation) is not str
                    or not 1 <= len(release_support_observation.strip()) <= 4000
                    or release_support_relation != "independent_support_present"):
                raise PairHostError("Opening support testimony is only for an explicit supported release")
            payload.update(release_support_observation=release_support_observation,
                           release_support_relation=release_support_relation)
        if release_retreat_observation is not None:
            if (kind != "joint" or operation != "release_retreat"
                    or type(release_retreat_observation) is not str
                    or not 1 <= len(release_retreat_observation.strip()) <= 4000):
                raise PairHostError("Describe current empty-gripper separation for a joint release retreat")
            payload["release_retreat_observation"] = release_retreat_observation
        digest = _digest(payload)
        with self.state_lock:
            if event_id in self.actions:
                previous = self.actions[event_id]
                if previous["digest"] != digest:
                    raise PairHostError("Existing event id has a different request")
                return {**self.status(event_id), "replayed": True}
            stored = self.ledger.event(event_id)
            if stored is not None:
                if _digest(stored["payload"]) != digest:
                    raise PairHostError("Existing durable event has a different request")
                if stored["status"] != "complete":
                    self._fault("Uncertain prior dispatch cannot be retried")
                    raise PairHostError("Prior dispatch is pending; reconciliation cannot resend it")
                self.actions[event_id] = {"digest": digest,
                    "status": "completed" if stored["success"] else "fault", "receipt": stored["receipt"]}
                return {**self.status(event_id), "replayed": True}
            if not self.opened or self.active_event_id is not None:
                raise PairHostError("Pair host must be open and have no pending action")
            self._guard()
            self.loaded.block_pending()
            if not self.task_ready:
                raise PairHostError("Pair preparation incomplete; task readiness required before dispatch claim")
            peer = "right" if arm == "left" else "left"
            scene = self.latest
            visual_joint = kind == "joint" and payload.get("admission_mode") == "rgb_supervised"
            if (scene is None or scene["observation_id"] != observation_id
                    or scene["peer_receipts"][peer]["receipt_id"] != peer_receipt_id
                    or scene["peer_receipts"][peer]["owner"] != self.owner
                    or self.clock() < scene["rgb_received_at"]
                    or (not visual_joint and self.clock()-scene["rgb_received_at"] > RGB_MAX_AGE_S)):
                raise PairHostError("Current host-issued shared scene and independent peer receipt required")
            if visual_joint:
                refresh = self._rgb_dispatch_window(scene, event_id, "before_source_resolution", payload.get("motion_profile"))
                if refresh is not None:
                    return refresh
            pending_contact = self.unresolved_gripper_probe
            grasp_states = self.grasp_states
            if coarse and (pending_contact is not None or any(grasp_states.values())
                    or any(not is_resolved_release(state) for state in self.grasps.states())):
                raise PairHostError("Coarse approach requires both arms free of active grasp episodes or contact")
            try:
                if not loaded:
                    self.grasps.check_retained_preparation(arm, kind, operation, grasp_states)
            except GraspBindingError as exc:
                self._fault("Grasp state binding lost: " + str(exc))
                raise
            selected_grasp = grasp_states.get(arm)
            if operation == "release_retreat" and kind in ("move", "joint"):
                if kind != "joint":
                    raise PairHostError("Confirmed release retreat uses the bounded joint path")
            execution_mode = "position"
            if selected_grasp is not None and operation == "release_retreat" and kind == "gripper":
                if not callable(getattr(self.device, "release_gripper_probe", None)):
                    raise PairHostError("Current adapter cannot explicitly release this grasp target")
                execution_mode = "contact_probe_release"
            elif pending_contact is not None:
                if (operation != "release_retreat" or kind != "gripper"
                        or pending_contact.get("arm") != arm
                        or not callable(getattr(self.device, "release_gripper_probe", None))):
                    raise PairHostError("Contact observation unresolved; only an explicit bounded same-jaw opening is available")
                execution_mode = "contact_probe_release"
            elif operation == "grip_supported" and kind == "gripper":
                if (self.capabilities.get("gripper_contact_observation") is not True
                        or not callable(getattr(self.device, "execute_gripper_probe", None))):
                    raise PairHostError("Current device has no bounded gripper contact observation capability")
                execution_mode = "contact_probe"
            elif not loaded and operation in CONTACT_OPERATIONS and (
                    self.capabilities.get("contact_step_supported") is not True
                    or self.capabilities.get("contact_support_verified") is not True):
                raise PairHostError("Current device has no qualified contact step/support capability")
            try:
                with self.device_lock:
                    sample = self._sample(self.device.observe())  # Revalidate peer before claiming any attempt.
            except Exception as exc:
                self._fault("Pre-dispatch peer feedback failed: " + str(exc))
                raise
            if operation == "release_retreat" and kind == "joint":
                try:
                    self.grasps.require_released_worker(arm, grasp_states, observation_id,
                                                       release_retreat_observation, sample)
                except GraspBindingError as exc:
                    self._fault("Release retreat binding lost: " + str(exc))
                    raise
            if execution_mode != "position":
                from .contact_receipt import probe_closure_within_bound
                current_width = sample["arms"][arm]["gripper"]["width_m"]
                before, after = (current_width, target) if execution_mode == "contact_probe" else (target, current_width)
                if not probe_closure_within_bound(before, after):
                    raise PairHostError("Contact probe closes, or explicitly releases by opening, at most 5 mm")
            release_prepared = False
            joint_context = None
            if kind == "joint":
                try:
                    with self.device_lock:
                        joint_context = self._joint_context(event_id, payload, scene)
                except _RefreshRGBRequired as exc:
                    return exc.receipt
                execution_mode = "joint"
            if visual_joint or execution_mode in ("contact_probe", "contact_probe_release"):
                # Contact also needs baseline/response time. Insufficient RGB
                # reserve is a refresh request before any episode or claim.
                refresh = self._rgb_dispatch_window(scene, event_id, "before_claim", payload.get("motion_profile"))
                if refresh is not None:
                    return refresh
            if execution_mode == "contact_probe":
                self.grasps.prepare_probe(arm, grasp_object_id)
            elif execution_mode == "contact_probe_release":
                with self.device_lock:
                    release_prepared = self.grasps.prepare_release(event_id, arm, target, observation_id,
                        release_support_observation, release_support_relation) is not None
            if loaded:
                self.loaded.begin(event_id, payload, joint_context)
            try:
                claim = self.ledger.begin(self.owner, event_id, payload)
            except BaseException as exc:
                if release_prepared or loaded:
                    self._fault("Prepared release could not claim its one physical attempt: " + str(exc))
                raise
            if claim["replayed"]:
                self.actions[event_id] = {"digest": digest, "status": "completed", "receipt": claim["receipt"]}
                return {**self.status(event_id), "replayed": True}
            if kind == "gripper":
                self.grasps.forget_release(arm)
            self.actions[event_id] = {"digest": digest, "status": "pending", "receipt": None}
            self.active_event_id = event_id
            self.dispatch_rgb_deadline = scene["rgb_received_at"] + RGB_MAX_AGE_S
            try:
                if kind == "joint" and payload.get("admission_mode") != "rgb_supervised":
                    from .host_joint import HostJointBridge
                    self.active_joint_bridge = HostJointBridge(self, event_id)
                else:
                    # Visual motion has its own finite observed corridor. It
                    # does not inherit the metric cancellation/hold contract.
                    self.active_joint_bridge = None
                self.worker = threading.Thread(target=self._execute, args=(event_id, payload, execution_mode, joint_context),
                                               name="piper-pair-action", daemon=True)
                self.worker.start()
            except BaseException as exc:
                self._fault("Claimed action worker did not start: " + str(exc))
                raise
            return {"status": "pending", "event_id": event_id, "replayed": False}

    def _execute(self, event_id, payload, execution_mode="position", joint_context=None):
        receipt = None
        try:
            with self.device_lock:
                self._guard()
                if self.clock()-self.latest["rgb_received_at"] > RGB_MAX_AGE_S:
                    raise PairHostError("RGB scene expired after durable dispatch claim")
                self.journal.append("pair_dispatch_claimed", event_id=event_id, payload=payload)
                self._guard()
                if execution_mode == "joint":
                    receipt = self.device.execute_joint(payload["arm"], payload["target"],
                        context=joint_context, event_id=event_id, deadline_at=self.deadline,
                        operation=payload["operation"], hold_bridge=self.active_joint_bridge)
                elif execution_mode == "contact_probe":
                    from .host_recovery import SupportedRecovery
                    receipt = SupportedRecovery(self).execute_probe(payload)
                elif execution_mode == "contact_probe_release":
                    receipt = self.device.release_gripper_probe(payload["arm"], payload["target"])
                else:
                    kwargs = ({"operation": payload["operation"]}
                              if any(s and s.get("status") == "retained_static" for s in self.grasp_states.values()) else {})
                    receipt = self.device.execute(payload["arm"], payload["kind"], payload["target"], **kwargs)
                # Vendor snapshots preserve IntEnum status values. Freeze the
                # JSON wire representation before strict durable validation;
                # do not turn valid SDK feedback into a post-send ledger fault.
                # Nonfinite or otherwise nonserializable values still fail.
                receipt = json.loads(json.dumps(receipt, allow_nan=False))
                if (receipt.get("ok") is not True or receipt.get("observed_stable") is not True
                        or receipt.get("controller_at_target") is not True
                        or receipt.get("feedback_all_after_send") is not True
                        or receipt.get("target_calls_sent") != 1
                        or receipt.get("passive_arm_commands_sent") != 0):
                    raise PairHostError("Stable feedback alone is not an arrived single dispatch")
                error = receipt.get("pose_error", {})
                if execution_mode == "joint":
                    after = receipt.get("after", {}).get(payload["arm"], {}).get("joints_rad")
                    encoded = (receipt.get("joint_path_plan") or {}).get("encoded_target_joints_rad")
                    if (type(after) is not list or type(encoded) is not list or len(after) != 6 or len(encoded) != 6
                            or any(type(v) not in (int, float) or not math.isfinite(v) for v in after+encoded)
                            or not joints_within(self.feedback_policy, payload["arm"], after, encoded)):
                        raise PairHostError("Missing fresh joint arrival within the encoded-target tolerance")
                    values = (0., 0.)  # Cartesian arrival is not a joint receipt requirement.
                else:
                    values = (error.get("position_m"), error.get("rotation_rad"))
                if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in values):
                    raise PairHostError("Missing finite pose arrival errors")
                if values[0] > 0.005 or values[1] > 0.05:
                    raise PairHostError("Observed pose is not at requested target")
                if payload["kind"] == "gripper":
                    width_error = receipt.get("width_error_m")
                    candidate = False
                    if execution_mode == "contact_probe":
                        observation = receipt.get("contact_observation", {})
                        outcome = observation.get("outcome")
                        candidate = outcome == "settled_contact_candidate"
                        if (outcome not in ("target_arrived", "settled_contact_candidate")
                                or observation.get("completion") != "observation_only"
                                or receipt.get("completion_mode") != "contact_probe"
                                or receipt.get("arrival_confirmed") is not (not candidate)
                                or receipt.get("grasp_verified") is not False
                                or receipt.get("accepted") is not None
                                or receipt.get("physical_stop_verified") is not None):
                            raise PairHostError("Contact observation cannot claim acknowledgement, grasp or physical stop")
                        pending_contact = self.unresolved_gripper_probe
                        if candidate and (not pending_contact or pending_contact.get("arm") != payload["arm"]):
                            raise PairHostError("Contact candidate must retain its unresolved jaw target")
                        if not candidate and pending_contact is not None:
                            raise PairHostError("Arrived contact probe cannot hide an unresolved target")
                    elif execution_mode == "contact_probe_release":
                        opening = receipt.get("actual_opening_increase_m")
                        episode = self.grasps.active(payload["arm"])
                        tracked = episode is not None and episode["status"] != "empty"
                        local = self.grasp_states.get(payload["arm"])
                        if (receipt.get("completion_mode") != "contact_probe_release"
                                or receipt.get("arrival_confirmed") is not True
                                or type(opening) not in (int, float) or not math.isfinite(opening)
                                or opening <= 0.0005
                                or (tracked and (local is None or local.get("status") != "release_opened"))
                                or (not tracked and local is not None)):
                            raise PairHostError("Probe opening must establish fresh arrival and preserve tracked release evidence")
                    if (type(width_error) not in (int, float) or not math.isfinite(width_error)
                            or width_error < 0 or (candidate and width_error <= 0.002)
                            or (not candidate and width_error > 0.002)):
                        raise PairHostError("Observed jaw width is not at requested target")
                self._guard()
                receipt = {**receipt, "pair_owner": self.owner, "event_id": event_id,
                           "observation_id": payload["observation_id"], "object_task_success": None,
                           "execution_mode": execution_mode,
                           "physical_stop_verified": None}
                self.ledger.finish(self.owner, event_id, receipt, success=True)
                if execution_mode == "contact_probe":
                    self.grasps.record_probe(event_id, payload, receipt)
                elif execution_mode == "contact_probe_release":
                    self.grasps.finish_release(event_id, payload, receipt)
                elif payload.get("loaded_observation") is not None:
                    self.loaded.finish(event_id, payload, receipt)
        except BaseException as exc:
            self._fault("Claimed dispatch failed or uncertain: " + str(exc))
            receipt = {"ok": False, "event_id": event_id, "error": str(exc),
                       "device_receipt": receipt, "physical_stop_verified": None,
                       "automatic_retry": False}
            receipt = self._finish_failed_receipt(event_id, receipt)
        finally:
            with self.state_lock:
                try:
                    self._advance_rgb_floor()
                except Exception as exc:
                    self.rgb_not_before = None
                    self._fault("Cannot establish post-action RGB boundary: " + str(exc))
                self.actions[event_id].update(status="fault" if self.fault_event.is_set() else "completed", receipt=receipt)
                self.active_event_id = None
                self.active_joint_bridge = None
                self.dispatch_rgb_deadline = None
                self.latest = None  # A new image and new peer receipt are needed for the next target.

    def confirm_loaded_response(self, event_id, action_event_id, observation_id, visual_description,
                                response, object_relation, support_relation, task_relation):
        _identifier(event_id, "event_id")
        _identifier(action_event_id, "action_event_id")
        with self.state_lock:
            if not self.opened or self.active_event_id is not None or not self.task_ready:
                raise PairHostError("Open idle task-ready host required for loaded response")
            self._guard()
            return self.loaded.confirm(event_id, action_event_id, observation_id, visual_description,
                                       response, object_relation, support_relation, task_relation)

    def retain_grasp(self, event_id, episode_id, observation_id, visual_description,
                     object_relation, support_relation):
        _identifier(event_id, "event_id")
        _identifier(episode_id, "episode_id")
        with self.state_lock:
            if not self.opened or self.active_event_id is not None:
                raise PairHostError("Open idle host required for static retention")
            self._guard()
            if not self.task_ready:
                raise PairHostError("Pair preparation incomplete; task readiness required for retention")
            from .host_recovery import SupportedRecovery
            SupportedRecovery(self).check_retention(episode_id)
            return self.grasps.retain(event_id, episode_id, observation_id, visual_description,
                                      object_relation, support_relation)

    def confirm_release(self, event_id, episode_id, observation_id, visual_description,
                        object_relation, support_relation):
        _identifier(event_id, "event_id")
        _identifier(episode_id, "episode_id")
        with self.state_lock:
            if not self.opened or self.active_event_id is not None:
                raise PairHostError("Open idle host required for release confirmation")
            self._guard()
            if not self.task_ready:
                raise PairHostError("Task readiness required for release confirmation")
            return self.grasps.confirm_release(event_id, episode_id, observation_id, visual_description,
                                               object_relation, support_relation)

    def status(self, event_id=None):
        with self.state_lock:
            if event_id is not None:
                action = self.actions.get(event_id)
                if action is None:
                    stored = self.ledger.event(event_id)
                    if stored is None:
                        raise PairHostError("Unknown event")
                    return {"event_id": event_id, "status": "pending" if stored["status"] == "pending"
                            else ("completed" if stored["success"] else "fault"), "receipt": stored["receipt"],
                            "recorded_owner": stored["owner"]}
                persistence_error = self._failure_receipt_persistence_error
                return {"event_id": event_id, "status": action["status"], "receipt": copy.deepcopy(action["receipt"]),
                        "failure_receipt_persistence_error": copy.deepcopy(persistence_error)
                            if persistence_error is not None and persistence_error["event_id"] == event_id else None}
            try:
                # Diagnostic polling must not append clock faults or renew
                # ledger time after dispatch has already been latched.
                state = self.ledger.peek_status() if self.fault_event.is_set() else self.ledger.status()
                if state["fault_latched"]:
                    self.fault_event.set()
            except Exception as exc:
                self._fault("Ledger status unavailable: " + str(exc))
                state = {"status": "unavailable", "fault_latched": True,
                         "error": type(exc).__name__ + ": " + str(exc)}
            with self._feedback_lock:
                feedback = copy.deepcopy(self._fault_feedback)
            try:
                grasp_episodes, grasp_error = self.grasps.states(), None
            except Exception as exc:
                # Diagnostics remain readable even if the grasp table cannot
                # be read. Missing history never becomes an empty-arm claim.
                grasp_episodes, grasp_error = None, type(exc).__name__ + ": " + str(exc)
            return {"status": "fault" if self.fault_event.is_set() else state["status"],
                    "fault_latched": self.fault_event.is_set() or state["fault_latched"],
                    "fault": state.get("fault") or state.get("global_fault"), "ledger": state,
                    "run_id": self.run_id, "owner": self.owner, "open": self.opened,
                    "connection_mode": self.connection_mode, "task_ready": self.task_ready,
                    "readiness": copy.deepcopy(self.readiness),
                    "joint_sources_diagnostic": copy.deepcopy(self.joint_sources_diagnostic),
                    "active_event_id": self.active_event_id, "capabilities": self.capabilities,
                    "unresolved_gripper_probe": self.unresolved_gripper_probe,
                    "grasp_states": self.grasp_states,
                    "grasp_episodes": grasp_episodes, "grasp_read_error": grasp_error,
                    "fault_reason": self._fault_reason, "fault_record_error": self._fault_record_error,
                    "failure_receipt_persistence_error": copy.deepcopy(self._failure_receipt_persistence_error),
                    "fault_feedback": feedback,
                    "fault_feedback_read_state": ("closed" if not self.opened or self.quit_event.is_set()
                        else "deferred_active_action" if self.active_event_id is not None and
                            (feedback is None or feedback.get("action_event_pending") != self.active_event_id)
                        else feedback.get("status", "unknown") if feedback is not None else "not_observed"),
                    "fault_feedback_journal_error": self._fault_feedback_journal_error,
                    "physical_stop_verified": None, "task_success": None}

    def wait(self, event_id, timeout=5):
        worker = self.worker
        if worker is not None:
            worker.join(timeout)
        return self.status(event_id)

    def cancel(self, reason="User cancelled further dispatch", *, allow_hold=True):
        bridge = self.active_joint_bridge
        if allow_hold and bridge is not None and self.active_event_id == bridge.event_id:
            bridge.cancel(reason)
        else:
            self._fault(reason)
        return {**self.status(), "software_cancelled": True, "physical_stop_verified": None}

    def close(self):
        # Do not hold state_lock while joining the monitor: its final status
        # read needs that lock. Serialize the full cleanup/release separately.
        with self.close_lock:
            if self._close_report is not None:
                return copy.deepcopy(self._close_report)
            return self._close_locked()

    def _close_locked(self):
        with self.state_lock:
            if self.active_event_id is not None:
                raise PairHostError("Action pending; cancel and wait for its receipt before resource cleanup")
            self.quit_event.set()
        if self.monitor is not None:
            self.monitor.join(1)
            if self.monitor.is_alive():
                self._fault("Monitor did not finish; retaining device lease")
                raise PairHostError("Monitor still running; cleanup and lease release deferred")
        cleanup = {"physical_stop_verified": None}
        try:
            if self.device is not None:
                try:
                    from .host_recovery import SupportedRecovery
                    SupportedRecovery(self).require_resolved()
                    if any(s["status"] != "empty" and not is_resolved_release(s) for s in self.grasps.states()):
                        self._fault("Closing with durable unreleased grasp; target state remains unresolved")
                except Exception as exc:
                    self._fault("Cannot establish clean grasp release during cleanup: " + str(exc))
                if self.unresolved_gripper_probe is not None:
                    self._fault("Closing with unresolved contact observation; residual jaw target not cancelled")
                with self.device_lock:
                    cleanup = self.device.close()
                if cleanup.get("requires_fault_latch") is True:
                    self._fault("Device cleanup retains an unresolved contact target")
                if (cleanup.get("guard_violations") or any(v.get("status") == "cleanup_failed"
                        for v in cleanup.get("arms", {}).values())):
                    raise PairHostError("Device cleanup failed or attempted an unexpected transmission")
            if self.opened and not self.fault_event.is_set():
                self.ledger.release(self.owner)
        except BaseException as exc:
            self._fault("Pair cleanup failed: " + str(exc))
            cleanup = {"device_cleanup": cleanup, "error": str(exc)}
            raise
        finally:
            self.opened = False
            if self.lease is not None:
                self.lease.__exit__(None, None, None)
                self.lease = None
            self._close_report = {"status": "closed", "cleanup": copy.deepcopy(cleanup),
                                  "physical_stop_verified": None, "fault_latched": self.fault_event.is_set()}
        return copy.deepcopy(self._close_report)
