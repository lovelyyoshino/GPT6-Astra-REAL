"""Same-connection MOVE_J transport and explicit-cancel target replacement.

Only GuardedPairDevice calls this module while holding its operation lock. A
plan never supplies dispatch permission: host-resolved provenance, the current
SDK connection's own cache history, the normal task guard and exact bus tickets
are all required. Fault replacement has its own durable bridge and transaction;
it never clears the original fault or authorizes a later task action.
"""
import copy
from .task_roles import PROFILE_KEY as TASK_ROLES_KEY, resolve_task_roles
from .feedback_tolerance import validate_policy, joints_within
import math
import threading
import time
import uuid

from . import arms
from .hold_transaction import JointHoldTransaction, joint_hold_frames, RAD_PER_RAW
from .joint_path import (plan_joint_path, validate_joint_path_sample, encode_joint_target,
                         evidence_sha256, _identity)
from .rgb_supervision import (JOINT_PATH_SCHEMA as RGB_JOINT_PATH_SCHEMA,
                              COARSE_JOINT_PATH_SCHEMA, LOADED_JOINT_PATH_SCHEMA,
                              LOADED_OPERATIONS, validate_loaded_context)
from .retention_receipt import anchor_deviation, measured_anchor, grasp_body_anchor
from .single_supervised_actions import BOUNDS
from .takeover import SIDES
from .tracking_observation import TrackingObservation


RGB_JOINT_SEND_SCHEMA = "piper_rgb_supervised_joint_send_v1"


def rgb_joint_execution_window_budget(motion_profile=None):
    """Minimum admission reserve for the supervised finite joint operation.

    Completed field segments took about eight seconds including the baseline.
    Reserve 3 s baseline + 6 s movement/dispatch/IO + 3 s final stability.
    The 20 s observation timeout is a maximum wait, not promised execution
    time. Reserving it all would leave only 6 s of the unchanged 30 s RGB age
    for the model's three-view supervision and RPC, defeating this workflow.
    These minima are not completion guarantees: the original RGB/task/50 ms
    feedback guards and observation timeout still reject any actual overrun.
    """
    baseline = BOUNDS["stable_s"]
    if motion_profile == "coarse_approach":
        # The 20-degree coarse envelope outgrew the small-step empirical
        # reserve. Keep the original RGB deadline; reserve the complete
        # bounded arrival/stability observation plus one second of dispatch IO.
        observation = BOUNDS["timeout_s"]
        return {"baseline_s": baseline,
                "postsend_observation_timeout_s": observation,
                "final_stable_window_s": BOUNDS["stable_s"],
                "dispatch_and_io_reserve_s": 1.0,
                "preclaim_required_s": baseline + observation + 1.0,
                "postsend_required_s": observation + 1.0,
                "minimum_admission_only": True,
                "covers_full_observation_timeout": True,
                "motion_profile": motion_profile}
    motion_and_io = 6.0
    final_stable = BOUNDS["stable_s"]
    return {"baseline_s": baseline,
            "postsend_observation_timeout_s": BOUNDS["timeout_s"],
            "final_stable_window_s": final_stable,
            "motion_dispatch_and_io_reserve_s": motion_and_io,
            "preclaim_required_s": baseline + motion_and_io + final_stable,
            "postsend_required_s": motion_and_io + final_stable,
            "minimum_admission_only": True,
            "covers_full_observation_timeout": False}


def checked_identity(device, identity):
    identity = _identity(identity)
    binding = device.joint_binding(identity["arm"])
    for field in ("connection_id", "model", "firmware_profile"):
        if identity[field] != binding[field]:
            raise RuntimeError("Joint identity differs from current SDK connection: " + field)
    return identity


def pair_sample(identity, states, *, now):
    return {"sample_id": "joint_rx_" + uuid.uuid4().hex,
            "identity": copy.deepcopy(identity), "captured_at": now,
            "arms": copy.deepcopy(states)}


def _identifier(value, field):
    if (type(value) is not str or not 0 < len(value) <= 128 or value != value.strip()
            or any(ord(c) < 32 or ord(c) == 127 for c in value)):
        raise ValueError("Host-resolved " + field + " required")


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _loaded_binding(device, context, event_id, operation, identity):
    """Compare typed host evidence to this connection's actual grasp records."""
    geometry = context.get("geometry")
    loaded = context.get("loaded_context")
    if type(geometry) is not dict or geometry.get("schema") != LOADED_JOINT_PATH_SCHEMA:
        if loaded is not None:
            raise ValueError("Loaded grasp evidence requires its dedicated RGB path")
        return None
    validate_loaded_context(loaded, identity, geometry)
    worker_arm, support_arm = resolve_task_roles(device._action.profile.get(TASK_ROLES_KEY, {}))
    if (loaded["event_id"] != event_id or loaded["operation"] != operation
            or identity["worker_id"] != event_id or identity["arm"] != worker_arm
            or resolve_task_roles(loaded) != (worker_arm, support_arm)):
        raise ValueError("Loaded event, selected worker and operation must match exactly")
    for role, side in (("worker", worker_arm), ("peer", support_arm)):
        expected, record = loaded[role], device._action.grasps[side]
        statuses = ("retained_static", "retained_local") if role == "worker" else ("retained_static",)
        if record is None or record["status"] not in statuses:
            raise RuntimeError("Loaded motion requires the live retained worker and static peer")
        for field, actual in (("identity", record["identity"]), ("probe_event_id", record["probe_event_id"]),
                ("probe_trace_sha256", record["trace_sha256"]), ("requested_width_m", record["requested_width_m"]),
                ("original_anchor", record["original_anchor"]), ("local_anchor", grasp_body_anchor(record))):
            if expected[field] != actual:
                raise RuntimeError("Loaded live grasp binding differs: " + role + "." + field)
    record = device._action.grasps[worker_arm]
    for previous in record.get("loaded_history", []):
        if previous["action_event_id"] == event_id:
            raise RuntimeError("A completed loaded event cannot be replayed by the adapter")
        if any(previous["object_scene"][key] != loaded["object_scene"][key] for key in ("source_id", "target_id")):
            raise RuntimeError("Loaded source and target object identities cannot change between segments")
    return copy.deepcopy(loaded)


class _TaskGuardFault(RuntimeError):
    """Marks only host dispatch-guard failures, never a telemetry failure."""


class _JointExecutor:
    def __init__(self, device, context, event_id, deadline_at, bridge):
        self.device, self.action = device, device._action
        self.context = copy.deepcopy(context)
        if validate_policy(context.get("feedback_observation")) != self.action.feedback_policy:
            raise RuntimeError("Observation policy differs from frozen device task")
        self.identity = checked_identity(device, context["identity"])
        self.event_id, self.deadline_at, self.bridge = event_id, deadline_at, bridge
        self.thread = threading.get_ident()
        self.plan = self.model = self.transaction = None
        self.phase = "normal"
        self.frames = []
        self.original_event = None
        self.original_recorded = False
        self.hold_event_id = None
        self.last_stamps = None
        self.last_now = time.time()
        self.last_sample = None
        self.last_sample_monotonic = None
        self.hold_frames_returned = []
        self.hold_failure_receipt = None
        self.mode_repeat_wait_s = 0.
        self.hold_prepare_sample = None
        self.completed_dispatch = None
        self.tracking = TrackingObservation("Joint")
        self.loaded_context = _loaded_binding(device, context, event_id,
            context.get("geometry", {}).get("evidence", {}).get("operation"), self.identity)

    def loaded_body_delegated(self, side):
        return (self.loaded_context is not None and self.phase == "normal"
                and side == resolve_task_roles(self.loaded_context)[0] == self.identity["arm"]
                and bool(self.frames) and self.plan is not None
                and self.plan.get("loaded_context") == self.loaded_context)

    def _time(self):
        now = time.time()
        if not _finite(now) or now < self.last_now:
            raise RuntimeError("Joint transport clock regressed or became invalid")
        self.last_now = now
        if now >= self.deadline_at:
            raise RuntimeError("Joint transport reached the original task deadline")
        if (self.plan is not None and self.plan["spatial_admission_mode"] == "rgb_supervised"
                and now > self.plan["visual_rgb_deadline"]):
            raise RuntimeError("visual_rgb_expired: original joint RGB deadline reached")
        return now

    def encoder_guard(self):
        if threading.get_ident() != self.thread:
            raise RuntimeError("Joint transport worker changed")
        self._time()
        if self.action.violations:
            raise RuntimeError("Persistent CAN ownership violation")
        if self.phase == "normal":
            if self.loaded_context is not None:
                if _loaded_binding(self.device, self.context, self.event_id,
                        self.loaded_context["operation"], self.identity) != self.loaded_context:
                    raise RuntimeError("Loaded grasp context changed during this event")
            if self.original_event is None and self.context.get("initialization_sources") != self.device.joint_initialization_sources():
                # Missing optional evidence remains compatible with callers
                # whose connection never established an initialization source.
                if ("initialization_sources" in self.context
                        or any(self.device.joint_initialization_sources().values())):
                    raise RuntimeError("Joint initialization source changed since host admission")
            try:
                self.action.guard()
            except BaseException as exc:
                raise _TaskGuardFault(str(exc)) from exc
            self._time()  # Include a slow durable host guard.
        elif (self.phase != "hold" or self.transaction is None
              or self.device._fault is None or self.original_event is None):
            raise RuntimeError("Dedicated fault-hold transaction unavailable")
        else:
            self.bridge.check_active()  # Pure local sticky abort flag, no SQLite.

    def read(self, *, journal=True):
        if self.phase == "hold":
            # The exceptional hold transaction keeps its existing reader.
            states = (self.action.read() if journal else
                      {s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES})
            self.action._checked_after_guard(states)
            return self.last_sample
        # A durable guard can block on storage. It must finish BEFORE taking
        # the observation that will be checked against the 50 ms age limit.
        began = time.monotonic()
        self.encoder_guard()
        guarded = time.monotonic()
        states = {s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES}
        captured = time.monotonic()
        captured_at = time.time()
        try:
            self.action._checked_after_guard(states)  # No rejected sample is retried.
            # Include the pair's later retained-body/jaw checks as well as the
            # pure validator in the same strict 50 ms observation age limit.
            self.final_fresh(self.last_sample)
            self.last_sample_monotonic = captured
        except BaseException as exc:
            rejected = pair_sample(self.identity, states, now=captured_at)
            self.tracking.record_failure(exc, rejected, self.tracking.last_tracking)
            self.action.report["rejected_joint_feedback"] = copy.deepcopy(rejected)
            self.action.report["after"] = copy.deepcopy(states)
            self.action.report["samples"] += 1
            try:
                self.action.emit("joint_feedback_rejected", {"sample": rejected, "error": str(exc)})
            except BaseException as journal_error:
                self.action.report["rejected_feedback_journal_error"] = str(journal_error)
            raise
        validated = time.monotonic()
        sample = self.last_sample
        stages = {"guard_s": guarded-began, "snapshot_s": captured-guarded,
                  "validation_s": validated-captured, "journal_s": 0.}
        if journal:
            self.action.report["after"] = copy.deepcopy(states)
            self.action.report["samples"] += 1
            before_journal = time.monotonic()
            self.action.emit("feedback", states)  # Preserve raw durable evidence.
            stages["journal_s"] = time.monotonic()-before_journal
            # A validated observation remains dated at capture, not at this
            # later return. A cancel/deadline during logging must still fail.
        timing = self.action.report.setdefault("joint_observation_timing", {"last": {}, "maximum_s": {}})
        timing["last"] = {"sample_id": sample["sample_id"], "captured_at": sample["captured_at"], **stages}
        for key, duration in stages.items():
            timing["maximum_s"][key] = max(timing["maximum_s"].get(key, 0.), duration)
        if journal:
            self.encoder_guard()
        return sample

    def prepare(self):
        """Original joint baseline spans/advances, using already validated RX.

        Unlike the generic single-action reader, logging cannot occur between
        snapshot capture and its first validation. No new observation replaces
        a rejected one; the extra final read only follows accepted samples.
        """
        action = self.action
        sample = self.read()
        state = sample["arms"]
        action.anchor = copy.deepcopy(state)
        action.expected_modes = {s: state[s]["arm_status"]["mode_feedback"] for s in SIDES}
        action.report["before"] = copy.deepcopy(state)
        action.baseline_window = action.new_window(state)
        start, start_monotonic = sample["captured_at"], self.last_sample_monotonic
        previous, advances = state, 0
        while (sample["captured_at"]-start < BOUNDS["stable_s"]
               or self.last_sample_monotonic-start_monotonic < BOUNDS["stable_s"]):
            time.sleep(BOUNDS["poll_s"])
            sample = self.read()
            state = sample["arms"]
            if not action.extend_window(action.baseline_window, state):
                raise RuntimeError("Single-action baseline exceeded stable feedback spans")
            if action.advanced(previous, state):
                previous, advances = state, advances+1
        if advances < BOUNDS["minimum_feedback_advances"]:
            raise RuntimeError("Baseline lacks 20 independently advancing complete samples in three seconds")
        # Logging/guard IO may have elapsed after the last accepted sample.
        # Recheck a new sample against the SAME window before leaving baseline.
        sample = self.read(journal=False)
        state = sample["arms"]
        if not action.extend_window(action.baseline_window, state):
            raise RuntimeError("Joint baseline changed during feedback recording")
        action.report.update(baseline_duration_s=self.last_sample_monotonic-start_monotonic,
                            baseline_spans=action.window_spans(action.baseline_window),
                            baseline_feedback_advances=advances)
        action.record_outcome(state, False, target_kind="joint")
        self.final_fresh(sample)

    def target_observed(self, states):
        arm = self.identity["arm"]
        return (self.action.all_after(states, self.action.sent_at)
            and states[arm]["arm_status"]["motion_status"] == 0
            and joints_within(self.action.feedback_policy, arm, states[arm]["joints_rad"],
                             self.plan["encoded_target_joints_rad"]))

    def check_states(self, states):
        if self.tracking.first_failure is not None:
            raise RuntimeError("Joint observation failure remains latched")
        now = self._time()
        validation_started = time.monotonic()
        sample_monotonic = validation_started
        stamps_at_entry = [v for state in states.values()
                           for v in state.get("fragment_timestamps_s", {}).values() if _finite(v)]
        diagnostic = {"entry_at": now, "oldest_fragment_age_at_entry_s":
            max((now-v for v in stamps_at_entry), default=None), "checked_at": None,
            "oldest_fragment_age_at_exit_s": None, "validation_elapsed_s": None,
            "static_geometry_policy": (
                "pure_immutable_numeric_box_memoized_at_admission"
                if self.plan.get("motion_profile") == "coarse_approach"
                else "coarse_box_memoization_not_applicable"),
            "live_feedback_age_limit_s": .05}
        self.action.report["joint_validation_timing"] = diagnostic
        sample = pair_sample(self.identity, states, now=now)
        settling = (self.phase == "normal" and self.action.active
            and self.completed_dispatch is not None
            and self.plan["spatial_admission_mode"] == "rgb_supervised"
            and self.plan["tracking_policy"]["mode"] == "bounded_postsend_settling")
        phase = "settling" if settling else "active" if self.action.active else "pre_dispatch"
        tracking = self.tracking.track(self.plan, sample)
        try:
            validation = validate_joint_path_sample(self.plan, sample, now=now, phase=phase)
            stamps = {(s, key): value for s in SIDES
                      for key, value in states[s]["fragment_timestamps_s"].items()}
            if self.last_stamps is not None and any(stamp < self.last_stamps[key]
                                                    for key, stamp in stamps.items()):
                raise RuntimeError("Joint feedback fragment regressed")
            # Include pure FK/hash work in the final freshness and original
            # deadline checks. Neither widened RX band waives a hard guard.
            checked_at = self._time()
            diagnostic.update(checked_at=checked_at,
                oldest_fragment_age_at_exit_s=max(checked_at-stamp for stamp in stamps.values()),
                validation_elapsed_s=time.monotonic()-validation_started)
            if any(not 0 <= checked_at-stamp <= .05 for stamp in stamps.values()):
                raise RuntimeError("Joint feedback exceeded 50 ms including validation")
            if settling:
                self.account_tracking_time(sample, tracking)
                validation["tracking"].update(cumulative_time_checked=True,
                    cumulative_outside_nominal_band_s=self.tracking.cumulative_s)
        except BaseException as exc:
            self.tracking.record_failure(exc, sample, tracking)
            raise
        self.last_stamps, self.last_sample = stamps, sample
        self.last_sample_monotonic = sample_monotonic
        self.action.report["joint_path_observation"] = validation
        return states

    def account_tracking_time(self, sample, tracking):
        # A helper start() call is not proof of sending. This runner owns and
        # independently verifies the actual completed attempt on every sample.
        proof, at = self.completed_dispatch, sample["captured_at"]
        if (self.phase != "normal" or proof is None
                or type(proof["comm_calls"]) is not int or proof["comm_calls"] != 4
                or type(proof["bus_calls"]) is not int or proof["bus_calls"] != 4
                or len(self.frames) != 4 or self.action.sent_at != proof["sent_at"]
                or self.action.ticket is not None or self.tracking.previous_at is None
                or not proof["sent_at"] <= self.tracking.previous_at <= at):
            raise RuntimeError("Settling requires this event's complete four-frame dispatch")
        self.tracking.account(sample, tracking,
            maximum_s=self.plan["tracking_policy"]["max_cumulative_outside_band_s"])

    def final_fresh(self, sample):
        now = self._time()
        if any(not 0 <= now-stamp <= .05 for side in SIDES
               for stamp in sample["arms"][side]["fragment_timestamps_s"].values()):
            raise RuntimeError("Joint final feedback exceeded 50 ms")

    def require_postsend_rgb_window(self, stage):
        if self.phase != "normal" or self.plan["spatial_admission_mode"] != "rgb_supervised":
            return
        now = self._time()
        remaining = self.plan["visual_rgb_deadline"] - now
        budget = rgb_joint_execution_window_budget(self.plan.get("motion_profile"))
        required = budget["postsend_required_s"]
        self.action.report["rgb_dispatch_window"] = {
            "stage": stage, "checked_at": now, "rgb_deadline": self.plan["visual_rgb_deadline"],
            "remaining_rgb_window_s": remaining, "required_postsend_window_s": required,
            "execution_window_budget": budget,
            "sufficient": remaining > required, "window_is_completion_guarantee": False,
            "returned_frames": len(self.frames), "automatic_retry": False,
            "claimed_event_or_budget_released": False}
        if remaining <= required:
            raise RuntimeError("insufficient_rgb_postsend_window: original RGB deadline cannot fit the minimum movement/IO and final-stability reserve; no first frame sent, claimed event remains faulted")

    def send_frame(self, side, original, frame, *args, **kwargs):
        self.encoder_guard()
        action, ticket = self.action, self.action.ticket
        if (ticket is None or side != self.identity["arm"] or side != ticket["side"]
                or ticket["thread"] != self.thread):
            action._deny(side, "Joint TX outside same-worker selected-arm ticket")
        index = ticket["bus_calls"]
        expected = self.plan["frames"] if self.phase == "normal" else self.transaction.report()["expected_frames"]
        if index >= len(expected):
            action._deny(side, "Extra joint transport frame")
        if self.phase == "hold":
            # Commit pending BEFORE taking the final fresh sample. If this
            # callback fails or the check fails, no frame is attempted/retried.
            self.device._joint_cache[self.identity["arm"]] = None
            self.device._joint_initializations[self.identity["arm"]] = None
            self.bridge.record_frame_begin(self.hold_event_id, index, copy.deepcopy(expected[index]))
        try:
            sample = self.read(journal=False)
            if self.phase == "hold":
                if index == 0:
                    # A fresh cache read need not contain new CAN fragments.
                    # The helper requires independent advancement after its
                    # prepared sample; wait finitely, preserving its anchor.
                    until = time.monotonic()+.05
                    while not self.action.advanced(self.hold_prepare_sample["arms"], sample["arms"]):
                        deviation = anchor_deviation(
                            measured_anchor(self.hold_prepare_sample["arms"][side]), sample["arms"][side])
                        if (deviation["joint_rad"] > .003 or deviation["position_m"] > .0005
                                or deviation["rotation_rad"] > .003 or deviation["jaw_m"] > .0005):
                            raise RuntimeError("Hold preparation anchor changed while awaiting new fragments")
                        if time.monotonic() >= until:
                            raise RuntimeError("Hold lacks independently advancing post-prepare feedback")
                        time.sleep(BOUNDS["poll_s"])
                        sample = self.read(journal=False)
                self.transaction.before_frame(expected[index], sample,
                    current_identity=self.identity, now=self._time())
            self.final_fresh(sample)
            if self.phase == "hold":
                self.bridge.check_active()
                self.final_fresh(sample)
            elif index == 0:
                # Check again after guard/storage/FK work, immediately before
                # the first actual bus send. Already claimed work is not erased.
                self.require_postsend_rgb_window("before_first_bus_frame")
            # Original wrappers still enforce actual frame bytes/flags, DLC,
            # comm/bus count, selected-arm/thread identity and per-attempt cap.
            result = original(frame, *args, **kwargs)
        except BaseException as exc:
            if self.phase == "hold":
                try:
                    self.transaction.record_frame_return(outcome="exception",
                        current_identity=self.identity, now=time.time())
                except BaseException:
                    pass
                self.bridge.record_frame_return(self.hold_event_id, index, "exception", error=str(exc))
            raise
        at = time.time()
        receipt = {"frame": copy.deepcopy(expected[index]), "outcome": "returned", "returned_at": at}
        if self.phase == "hold":
            self.hold_frames_returned.append(receipt)
            self.transaction.record_frame_return(outcome="returned", current_identity=self.identity, now=at)
            self.bridge.record_frame_return(self.hold_event_id, index, "returned")
        else:
            self.frames.append(receipt)
        return result

    def dispatch(self, target, frames):
        action, robot = self.action, self.action.robots[self.identity["arm"]]
        self.encoder_guard()
        for name, wanted in (("ctrl_mode", 1), ("mit_mode", 0), ("residence_time", 0), ("installation_pos", 0)):
            value = getattr(robot._msg_mode, name, None)
            if not isinstance(value, int) or isinstance(value, bool) or value != wanted:
                raise RuntimeError("Unexpected SDK mode cache " + name)
        # These two SDK operations mutate memory only; the sole SDK command
        # below still has to match its exact four standard CAN frames.
        robot.set_auto_set_motion_mode_enabled(True)
        robot._msg_mode.move_spd_rate_ctrl = 1
        ticket = {"side": self.identity["arm"], "thread": self.thread, "kind": "action",
                  "frames": [(f["arbitration_id"], bytes.fromhex(f["data_hex"])) for f in frames],
                  "comm_calls": 0, "bus_calls": 0, "error": None}
        with action.lock:
            action.active, action.dispatched, action.ticket = True, True, ticket
            try:
                robot.move_j(list(target))
                if ticket["error"] is not None:
                    raise RuntimeError("Joint CAN send failed") from ticket["error"]
                returned = self.frames if self.phase == "normal" else self.hold_frames_returned
                if (type(ticket["comm_calls"]) is not int or ticket["comm_calls"] != 4
                        or type(ticket["bus_calls"]) is not int or ticket["bus_calls"] != 4
                        or len(returned) != 4):
                    raise RuntimeError("Incomplete joint target; no retry or completion frames")
            finally:
                action.ticket = None
        action.sent_at = time.time()
        if self.phase == "normal":
            self.completed_dispatch = {"comm_calls": ticket["comm_calls"], "bus_calls": ticket["bus_calls"],
                                       "sent_at": action.sent_at}
            self.tracking.start(action.sent_at)
        action.report["target_calls_sent"] = 1

    def original(self):
        plan, arm = self.plan, self.identity["arm"]
        if len(self.frames) != 4:
            raise RuntimeError("Original MOVE_J incomplete; hold is unavailable")
        event = {"event_id": self.event_id, "identity": copy.deepcopy(self.identity),
                 "worker_thread_id": self.thread, "send_state": "all_frames_returned",
                 "target_raw": plan["target_raw"][:], "reference": copy.deepcopy(plan["origin"]),
                 "frame_receipts": copy.deepcopy(self.frames),
                 "limits": {"joint_limits_raw": copy.deepcopy(plan["effective_joint_limits_raw"]),
                    "max_translation_m": plan["budget"]["max_translation_m"],
                    "max_rotation_rad": plan["budget"]["max_rotation_rad"]},
                 "deadline_at": self.deadline_at, "fault": None}
        if plan["spatial_admission_mode"] == "rgb_supervised":
            # This is a returned-send fact, not the metric hold contract. RGB
            # semantics remain explicit; no workspace or attachment is inferred.
            if self.bridge is not None:
                raise RuntimeError("RGB joint motion cannot bind a cancellation hold bridge")
            event.update(schema=RGB_JOINT_SEND_SCHEMA,
                operation=plan["geometry"]["evidence"]["operation"],
                motion_profile=plan.get("motion_profile", "ordinary"),
                spatial_admission_mode="rgb_supervised", hold_supported=False,
                hold_policy="latch_only", plan_sha256=plan["plan_sha256"],
                rgb_admission=copy.deepcopy(plan["geometry"]),
                model_source=copy.deepcopy(self.model.source),
                accepted=None, physical_stop_verified=None)
        else:
            event["limits"].update(
                workspace_min_m=plan["geometry"]["workspace_min_m"][:],
                workspace_max_m=plan["geometry"]["workspace_max_m"][:])
            event["geometry_source"] = copy.deepcopy(self.model.source)
        self.original_event = event
        if self.bridge is not None:
            self.bridge.record_original(copy.deepcopy(event))
            self.original_recorded = True
        self.device._joint_cache[arm] = {key: copy.deepcopy(event[key]) for key in
                                         ("event_id", "identity", "target_raw", "frame_receipts")}
        self.device._joint_initializations[arm] = None

    def cancel_request(self):
        if self.bridge is None:
            return None
        return self.bridge.cancellation_request()

    def wait_mode_repeat_interval(self):
        """Honor the SDK's duplicate-mode filter without changing or probing it.

        Reading filter bookkeeping is memory-only. During this finite wait the
        ordinary fault stays set, original fixed anchors remain in force, and
        no encoder/ticket is available. Fake SDK fixtures without such a filter
        have no delay. No missing mode frame is ever completed after an attempt.
        """
        ctx = getattr(self.action.robots[self.identity["arm"]], "_ctx", None)
        if ctx is None:
            return
        lock = getattr(ctx, "_tx_repeat_lock", None)
        if lock is None:
            raise RuntimeError("SDK duplicate-mode timing is unavailable")
        with lock:
            interval = ctx._tx_repeat_min_interval.get(0x151, 0.)
            previous = ctx._tx_last_stamp.get(0x151)
            payload = ctx._tx_last_payload.get(0x151)
        if not _finite(interval) or not 0 <= interval <= .1:
            raise RuntimeError("Unreviewed SDK duplicate-mode interval")
        if interval == 0 or payload != bytes.fromhex(self.plan["frames"][0]["data_hex"]):
            return
        if not _finite(previous):
            raise RuntimeError("SDK duplicate-mode timestamp is unknown")
        began = time.monotonic()
        if previous > began:
            raise RuntimeError("SDK duplicate-mode clock regressed")
        until = previous + interval
        previous_now = began
        while time.monotonic() < until:
            observed_now = time.monotonic()
            if not _finite(observed_now) or observed_now < previous_now:
                raise RuntimeError("SDK mode wait monotonic clock regressed")
            previous_now = observed_now
            self.read(journal=False)
            time.sleep(max(0., min(BOUNDS["poll_s"], until-time.monotonic())))
        self.mode_repeat_wait_s = time.monotonic()-began

    def hold(self, request):
        if self.plan["spatial_admission_mode"] == "rgb_supervised" or self.bridge is None:
            raise RuntimeError("RGB joint motion has latch-only cancellation; no hold is available")
        if self.original_event is None or len(self.frames) != 4:
            raise RuntimeError("Original incomplete; explicit cancellation cannot send hold")
        if type(request) is not dict or set(request) != {"hold_event_id", "reason"}:
            raise RuntimeError("Explicit cancellation bridge returned invalid request")
        _identifier(request["hold_event_id"], "hold event id")
        _identifier(request["reason"], "cancellation reason")
        self.device._fault = "Explicit cancellation: " + request["reason"]
        self.wait_mode_repeat_interval()
        # Do not call the normal guard once it is fault-latched. The bridge's
        # durable claim alone verifies this is the permitted cancellation kind.
        states = {s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES}
        sample = pair_sample(self.identity, states, now=self._time())
        raw, _ = encode_joint_target(states[self.identity["arm"]]["joints_rad"])
        payload = {"operation": "joint_hold_current", "identity": copy.deepcopy(self.identity),
                   "target_raw": raw, "expected_frames": joint_hold_frames(raw),
                   "sample_ref": {"sample_id": sample["sample_id"], "captured_at": sample["captured_at"],
                                  "sha256": evidence_sha256(sample)}}
        self.hold_event_id = request["hold_event_id"]
        claim = self.bridge.claim_hold(self.event_id, self.hold_event_id, payload)
        try:
            self.transaction = JointHoldTransaction(claim["original_event"], claim["claim"],
                                                    model_geometry=self.model)
        except BaseException as exc:
            self.hold_failure_receipt = {"identity": copy.deepcopy(self.identity),
                "hold_event_id": self.hold_event_id, "original_event_id": self.event_id,
                "status": "fault", "frames_complete": False, "hold_observed": False,
                "original_fault": copy.deepcopy(claim.get("original_event", {}).get("fault")),
                "hold_fault": {"detail": str(exc)}, "frame_attempts": [],
                "physical_stop_verified": None, "original_target_cancelled": None}
            self.bridge.finish_hold(self.hold_event_id, copy.deepcopy(self.hold_failure_receipt))
            raise
        self.phase = "hold"
        try:
            # SQLite may have blocked. Prepare against a NEW real snapshot.
            sample = self.read(journal=False)
            envelope = {"sample_id": sample["sample_id"], "model": self.identity["model"],
                        "source_ref": self.context["geometry"]["source"]["ref"],
                        "source_sha256": self.context["geometry"]["source"]["sha256"],
                        "geometry_mode": "model_joint_geometry_v1",
                        "attachment_radius_m": self.context["geometry"]["attachment_radius_m"][self.identity["arm"]],
                        "link_body_allowance_m": .06}
            self.transaction.prepare(sample, envelope, current_identity=self.identity, now=self._time(),
                                     frozen_target_raw=raw)
            self.hold_prepare_sample = copy.deepcopy(sample)
            held = self.transaction.report()
            if held["target_raw"] != raw:
                raise RuntimeError("Hold target changed after its durable reservation")
            # A new bounded attempt on the SAME SDK connection. Original
            # counters/receipts remain separately archived and fault-latched.
            self.action.reset_action(self.identity["arm"], "move", [v*RAD_PER_RAW for v in raw])
            self.action.active = True
            self.dispatch([v*RAD_PER_RAW for v in raw], held["expected_frames"])
            deadline = time.monotonic() + BOUNDS["timeout_s"]
            while time.monotonic() < deadline:
                time.sleep(BOUNDS["poll_s"])
                self.encoder_guard()
                sample = self.read(journal=False)
                self.transaction.observe(sample, now=self._time())
                report = self.transaction.report()
                if report["status"] == "fault":
                    raise RuntimeError("Hold observation fault: " + repr(report["hold_fault"]))
                if report["hold_observed"]:
                    break
            else:
                raise RuntimeError("Hold observation timed out")
        except BaseException as exc:
            try:
                self.transaction.invalidate(str(exc), now=time.time())
            except BaseException:
                pass
            raise
        finally:
            report = self.transaction.report()
            report["adapter_integration"] = "same_connection_guarded_MOVE_J"
            self.bridge.finish_hold(self.hold_event_id, report)
        return report


def execute_joint(device, arm, target, *, context, event_id, deadline_at,
                  operation, recovery_mode, hold_bridge):
    if arm not in SIDES or operation not in ("approach", "align", "recover", "release_retreat", *LOADED_OPERATIONS):
        raise ValueError("Explicit supported joint operation required")
    if recovery_mode is not None:
        raise ValueError("Joint recovery transport gap: boundary-origin and disabled-jaw hold contract is not integrated")
    geometry = context.get("geometry")
    loaded = type(geometry) is dict and geometry.get("schema") == LOADED_JOINT_PATH_SCHEMA
    coarse = type(geometry) is dict and geometry.get("schema") == COARSE_JOINT_PATH_SCHEMA
    visual = type(geometry) is dict and geometry.get("schema") in (
        RGB_JOINT_PATH_SCHEMA, COARSE_JOINT_PATH_SCHEMA, LOADED_JOINT_PATH_SCHEMA)
    if visual:
        if hold_bridge is not None:
            raise ValueError("RGB joint motion cannot bind a cancellation hold bridge")
        operations = LOADED_OPERATIONS if loaded else ("approach",) if coarse else ("approach", "align", "release_retreat")
        if (operation not in operations
                or type(geometry.get("evidence")) is not dict
                or geometry["evidence"].get("operation") != operation):
            raise ValueError("RGB joint operation must match its dedicated current evidence")
    elif operation in LOADED_OPERATIONS:
        raise ValueError("Loaded operations require dedicated RGB episode evidence")
    _identifier(event_id, "original event id")
    if not _finite(deadline_at) or deadline_at <= time.time():
        raise ValueError("Original finite task deadline required")
    if hold_bridge is not None and any(not callable(getattr(hold_bridge, name, None)) for name in (
            "record_original", "cancellation_request", "claim_hold", "record_frame_begin",
            "record_frame_return", "finish_hold", "check_active")):
        raise ValueError("Complete durable cancellation bridge required before joint dispatch")
    target = arms._six_finite(target)
    encode_joint_target(target)
    device._usable()
    action = device._action
    action.require_no_loaded_pending_tx()
    if coarse and (action.unresolved_probe is not None or any(action.grasps.values())):
        raise RuntimeError("Coarse approach requires both live grippers empty of grasp episodes")
    if not loaded and (action.unresolved_probe is not None or action.grasps[arm] is not None):
        raise RuntimeError("A candidate or retained selected arm cannot execute joint motion")
    if not loaded and any(action.grasps.values()) and operation not in ("approach", "align", "release_retreat"):
        raise RuntimeError("Peer static retention permits only unloaded approach/align/release_retreat")
    runner = _JointExecutor(device, context, event_id, deadline_at, hold_bridge)
    if loaded and runner.loaded_context["operation"] != operation:
        raise ValueError("Loaded operation differs from the bound episode")
    if runner.identity["arm"] != arm:
        raise ValueError("Selected arm and joint context differ")
    origin = context["origin"]
    if device._joint_observations.get(origin.get("sample_id")) != origin:
        raise RuntimeError("Joint origin was not issued by this live device")
    if device._joint_cache[arm] is None:
        raise RuntimeError("Joint bootstrap gap: this SDK connection has no known complete MOVE_J target cache")
    if context.get("cached_target") != device._joint_cache[arm]:
        raise RuntimeError("Joint cached target differs from this device's returned frame history")
    live_sources = device.joint_initialization_sources()
    if context.get("initialization_sources", dict.fromkeys(SIDES)) != live_sources:
        raise RuntimeError("Joint initialization source differs from this device's live cache history")
    original_report, hold_report = None, None
    action.reset_action(arm, "move", target)
    try:
        from .joint_geometry import OfficialJointModel
        runner.model = OfficialJointModel(context["model_catalog"], runner.identity["model"])
        current = pair_sample(runner.identity,
            {s: arms.snapshot(action.robots[s], action.grippers[s]) for s in SIDES}, now=time.time())
        runner.context["current"] = current
        runner.plan = plan_joint_path(runner.context, target, now=time.time(), recovery_mode=recovery_mode)
        action.joint_executor = runner
        # Planning can occupy the Python worker while the two independent SDK
        # receiver threads wait to decode queued frames. Yield once BEFORE the
        # first baseline snapshot, just as between ordinary baseline samples.
        # This is not a retry of rejected telemetry: every subsequently read
        # timestamp still faces the original 50 ms limit and all task guards.
        runner.encoder_guard()
        rx_yield_began = time.monotonic()
        time.sleep(BOUNDS["poll_s"])
        action.report["initial_rx_schedule"] = {
            "requested_yield_s": BOUNDS["poll_s"],
            "elapsed_s": time.monotonic()-rx_yield_began,
            "before_first_baseline_sample": True,
            "rejected_samples_retried": 0,
            "timestamps_renewed": False}
        runner.prepare()
        action.emit("pair_joint_intent", {"event_id": event_id, "plan": runner.plan})
        runner.encoder_guard()
        dispatch_sample = runner.read(journal=False)
        if not action.extend_window(action.baseline_window, dispatch_sample["arms"]):
            raise RuntimeError("Joint baseline changed before dispatch")
        action.report["dispatch_feedback"] = copy.deepcopy(dispatch_sample["arms"])
        action.dispatch_anchor = copy.deepcopy(dispatch_sample["arms"][arm])
        runner.require_postsend_rgb_window("after_baseline_before_dispatch")
        runner.dispatch(target, runner.plan["frames"])
        original_report = copy.deepcopy(action.report)
        runner.original()
        began, began_monotonic, previous, advances, window = None, None, None, 0, None
        timeout = time.monotonic() + BOUNDS["timeout_s"]
        while time.monotonic() < timeout:
            request = runner.cancel_request()
            if request is not None:
                hold_report = runner.hold(request)
                raise RuntimeError("Original action cancelled; same-mode hold observation does not clear its fault")
            runner.encoder_guard()
            sample = runner.read()
            states = sample["arms"]
            arrived = runner.target_observed(states)
            if arrived:
                if began is None or not action.extend_window(window, states):
                    began, began_monotonic = sample["captured_at"], runner.last_sample_monotonic
                    previous, advances = states, 0
                    window = action.new_window(states)
                elif action.advanced(previous, states):
                    previous, advances = states, advances+1
                if (sample["captured_at"]-began >= BOUNDS["stable_s"]
                        and runner.last_sample_monotonic-began_monotonic >= BOUNDS["stable_s"]
                        and advances >= BOUNDS["minimum_feedback_advances"]):
                    # The preceding validated observation was durably logged.
                    # Completion additionally needs an actual new observation
                    # after that IO, preserving the original stable window.
                    sample = runner.read(journal=False)
                    states = sample["arms"]
                    if not action.extend_window(window, states) or not runner.target_observed(states):
                        began, began_monotonic, previous, advances, window = None, None, None, 0, None
                        time.sleep(BOUNDS["poll_s"])
                        continue
                    runner.final_fresh(sample)
                    if loaded:
                        # Durable/raw diagnostic I/O precedes a last real read;
                        # no timestamp replacement can hide drift while writing.
                        action.journal("pair_loaded_segment_observed", {
                            "event_id": event_id, "operation": operation,
                            "plan_sha256": runner.plan["plan_sha256"],
                            "loaded_context": copy.deepcopy(runner.loaded_context),
                            "sample": copy.deepcopy(sample), "feedback_advances": advances,
                            "observed_stable_duration_s": runner.last_sample_monotonic-began_monotonic})
                        runner.encoder_guard()
                        sample = runner.read(journal=False)
                        states = sample["arms"]
                        if (not action.extend_window(window, states)
                                or not action.all_after(states, action.sent_at)
                                or states[arm]["arm_status"]["motion_status"] != 0
                                or not joints_within(action.feedback_policy, arm, states[arm]["joints_rad"],
                                    runner.plan["encoded_target_joints_rad"])):
                            raise RuntimeError("Loaded completion changed during receipt recording")
                        runner.final_fresh(sample)
                        record = action.grasps[arm]
                        local = measured_anchor(states[arm])
                        pending = {"action_event_id": event_id, "operation": operation,
                            "finished_at": sample["captured_at"], "plan_sha256": runner.plan["plan_sha256"],
                            "local_anchor": copy.deepcopy(local)}
                        receipt = {**copy.deepcopy(pending), "original_anchor": copy.deepcopy(record["original_anchor"]),
                            "probe_event_id": record["probe_event_id"], "probe_trace_sha256": record["trace_sha256"]}
                        record.setdefault("loaded_history", []).append({**copy.deepcopy(receipt),
                            "object_scene": copy.deepcopy(runner.loaded_context["object_scene"])})
                        record.update(status="loaded_pending_visual", local_anchor=copy.deepcopy(local),
                                      loaded_pending=copy.deepcopy(pending))
                        action.report["loaded_receipt"] = receipt
                    for key in ("pose_m_rad", "joints_rad"):
                        action.idle_anchor[arm][key] = copy.deepcopy(states[arm][key])
                    action.active = False
                    action.anchor = copy.deepcopy(action.idle_anchor)
                    device._baseline_duration, device._advances = runner.last_sample_monotonic-began_monotonic, advances
                    action.record_outcome(states, True, target_kind="joint")
                    action.report.update(ok=True, status="joint_target_arrived_and_stationary_observed",
                        arrival_confirmed=True, controller_at_target=True, observed_stable=True,
                        observed_stable_duration_s=device._baseline_duration,
                        observed_feedback_advances=advances, feedback_all_after_send=True,
                        sample=device._sample(states), after=copy.deepcopy(states))
                    break
            else:
                began, began_monotonic, previous, advances, window = None, None, None, 0, None
            time.sleep(BOUNDS["poll_s"])
        else:
            raise RuntimeError("Joint arrival observation timed out; no retry")
    except BaseException as exc:
        if (isinstance(exc, _TaskGuardFault) and runner.original_recorded
                and runner.transaction is None and runner.phase == "normal"):
            # Cancellation may arrive between the nonblocking poll and guard.
            # Only a host-guard exception gets this second dedicated poll;
            # telemetry/geometry/transport failures NEVER become hold permits.
            try:
                request = runner.cancel_request()
                if request is not None:
                    hold_report = runner.hold(request)
            except BaseException as hold_exc:
                action.report.setdefault("errors", []).append({
                    "type": type(hold_exc).__name__, "detail": str(hold_exc)})
        runner.tracking.record_failure(exc)
        device._fault = device._fault or str(exc)
        action.report.update(ok=False, status="pair_device_fault", arrival_confirmed=False, sample=None)
        action.report.setdefault("errors", []).append({"type": type(exc).__name__, "detail": str(exc)})
        if runner.original_event is None and action.counts[arm]["attempted_frames"]:
            device._joint_cache[arm] = None
            device._joint_initializations[arm] = None
    finally:
        action.ticket = None
        action.joint_executor = None
    result = copy.deepcopy(action.report)
    if runner.transaction is not None and hold_report is None:
        hold_report = runner.transaction.report()
    elif hold_report is None:
        hold_report = runner.hold_failure_receipt
    result.update(arm=arm, kind="joint", target_joints_rad=target,
        joint_path_plan=copy.deepcopy(runner.plan), original_event=copy.deepcopy(runner.original_event),
        original_action_report=original_report, hold_receipt=hold_report,
        accepted=None, capabilities=device.capabilities, physical_stop_verified=None,
        physical_stop_supported=False, grasp_verified=False,
        explicit_cancel_hold_bridge_bound=hold_bridge is not None,
        spatial_admission_mode=(runner.plan or {}).get("spatial_admission_mode"),
        motion_profile=(runner.plan or {}).get("motion_profile"),
        hold_policy=(runner.plan or {}).get("hold_policy"),
        hold_supported=(runner.plan or {}).get("hold_supported"),
        tracking_observation=copy.deepcopy(runner.tracking.report),
        transmission_counts=copy.deepcopy(action.counts), session_transmission_counts=action.totals(),
        hardware_commands_sent=len(runner.frames)+len(runner.hold_frames_returned),
        passive_arm_commands_sent=action.counts[action.passive_arm]["sent_frames"],
        sdk_mode_repeat_wait_s=runner.mode_repeat_wait_s,
        guard_violations=copy.deepcopy(action.violations), connected=not device._closed)
    if loaded:
        result.update(loaded_observation_only=True, loaded_response_pending_visual=result.get("ok") is True,
                      loaded_receipt=copy.deepcopy(action.report.get("loaded_receipt")),
                      grasp_states=device.grasp_states,
                      contact_support_verified=False, object_progress_measurement=None)
    return result
