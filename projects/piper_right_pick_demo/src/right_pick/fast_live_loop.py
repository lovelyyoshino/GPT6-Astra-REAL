"""Finite ROS/RGB phase runner with an independent, fail-closed admission gate.

This module does not commission the robot. The exact ROS adapter must verify
its physical safety prerequisites before cameras, model calls, or dispatch.
Every dispatch still passes the adapter's own guard. No stop, reset, disable,
driver shutdown, SDK call, target-coordinate estimator, or calibration here.
"""
import copy
import math
import time

from .fast_policy import (compact_controller_state, completion_phase, parse_response,
                          requires_explanation, validate_decision,
                          phase_translation_fraction, phase_rotation_fraction)
from .fast_recording import FastRunMetrics
from .fast_pipeline import (SINGLE_ARM_PIPELINE_ID, compact_pipeline, pipeline_contract,
                            validate_preflight, validate_fresh_observation,
                            validate_task_evidence)
from .fast_safety import FastSafetyError, rotation_distance


_PHYSICAL_ACTIONS = frozenset(("move_eef", "move_eef_chunk", "gripper"))
_FAILURES = frozenset(("no_progress", "target_lost", "grasp_failed"))


def verify_preflight_contract(result):
    """Require qualified timeout behaviour, without claiming an instant hold.

    Readiness is not action authorization: only the adapter's subsequent
    guard.validate(decision, measured_state) can authorize a particular goal.
    """
    if (not isinstance(result, dict) or result.get("physical_execution_ready") is not True
            or result.get("timeout_policy_verified") is not True
            or result.get("hold_verified") is not False or result.get("blockers") != []
            or not isinstance(result.get("qualification"), dict) or not result["qualification"]):
        raise FastSafetyError("ROS physical qualification is absent or incomplete")
    validate_preflight(result)
    return result


def verify_decision_robot_stability(before, current):
    """Endpoint feedback check, not visual scene validation or motion tracking.

    Use the same conservative change bounds as the adapter's before-send
    check. This grants no permission to dispatch and supplies no target pose.
    """
    initial, final = before["robot_state"], current["robot_state"]
    a, b = initial["pose_m_rad"], final["pose_m_rad"]
    drift = dict(max_xyz_delta_m=max(abs(x-y) for x, y in zip(a[:3], b[:3])),
                 orientation_delta_rad=rotation_distance(a, b),
                 max_joint_delta_rad=max(abs(x-y) for x, y in zip(initial["joints_rad"], final["joints_rad"])),
                 gripper_delta_m=abs(before["gripper_state"]["opening_m"]-current["gripper_state"]["opening_m"]),
                 sampled_before_model=initial["sampled_at"], sampled_after_model=final["sampled_at"])
    if (drift["max_xyz_delta_m"] > .0005 or drift["orientation_delta_rad"] > .003
            or drift["max_joint_delta_rad"] > .003 or drift["gripper_delta_m"] > .0005):
        raise FastSafetyError("Robot state changed while model used the earlier RGB snapshot")
    return drift


class FastLiveClosedLoop:
    """One new RGB packet and exactly one model action per bounded step.

    Offline tests patch the concrete adapters; there is intentionally no public
    fake-robot, skip-preflight, or qualification-boolean escape hatch.
    """
    def __init__(self, *, model, robot, cameras, recorder, limits, options=None,
                 observation_contract=None):
        from .fast_ros import ROSRightArm
        from .fast_observation import SubprocessRGBCameras
        if type(robot) is not ROSRightArm or type(cameras) is not SubprocessRGBCameras:
            raise FastSafetyError("Live execution requires the concrete ROS and RGB adapters")
        if not isinstance(limits, dict) or not limits:
            raise FastSafetyError("Explicit physical limits required; no replay defaults")
        if limits != robot.config.get("physical_limits"):
            raise FastSafetyError("Model action budget must use the ROS adapter's physical limits")
        if recorder.mode != "physical":
            raise FastSafetyError("Live execution requires a distinct physical run recorder")
        self.model, self.robot, self.cameras = model, robot, cameras
        self.recorder, self.limits = recorder, copy.deepcopy(limits)
        # A live run must have one visible stop contract.  In particular,
        # model timeout * max_calls must not silently become a multi-hour wait.
        self.options = dict(max_steps=32, max_model_calls=24,
                            max_retries=2, max_phase_steps=4,
                            max_recoveries=1, max_observe_unknown=2,
                            max_elapsed_s=900)
        self.options.update(options or {})
        allowed = {"max_steps", "max_model_calls", "max_retries", "max_phase_steps",
                   "max_recoveries", "max_observe_unknown", "max_elapsed_s"}
        for key, value in self.options.items():
            if key not in allowed or type(value) is not int or value <= 0:
                raise ValueError("Unknown/nonpositive integer controller option: " + key)
        self.phase, self.retry_count = "INIT", 0
        self.previous_action = self.previous_result = None
        self.memory = ""
        self.execution_evidence, self.visual_evidence = {}, []
        self.consecutive_observe_unknown = 0
        self._last_observation_time = None
        self._observation_receipt = None
        self.observation_contract = dict(max_age_s=.8, max_skew_s=.15)
        self.observation_contract.update(observation_contract or {})
        if set(self.observation_contract) != {"max_age_s", "max_skew_s"}:
            raise ValueError("Unknown observation contract field")
        self._used = False
        self._decision_calls = 0

    def _preflight(self):
        # The exact adapter, not this controller, owns qualification checking.
        # Absence of the independently implemented verifier is a hard failure.
        preflight = getattr(self.robot, "preflight", None)
        if not callable(preflight):
            raise FastSafetyError("ROS physical preflight has not been implemented")
        result = verify_preflight_contract(preflight())
        self.recorder.event("live_preflight", result)
        self.recorder.event("atomic_preflight_single", validate_preflight(result))
        return result

    def _budget(self):
        limits = self.limits
        return dict(
            max_translation_m=limits["max_translation_step_m"] * phase_translation_fraction(self.phase),
            max_rotation_rad=limits["max_rotation_step_rad"] * phase_rotation_fraction(self.phase),
            max_speed_percent=limits["max_speed_percent"],
            # Current ROS adapter accepts a single endpoint. Advertising a
            # multi-waypoint budget would induce unexecutable proposals.
            max_waypoints=1, gripper_min_m=limits["gripper_min_m"],
            gripper_max_m=limits["gripper_max_m"],
            max_effort_parameter_nm=limits["max_effort_parameter_nm"],
            allow_waypoint_chunks=False, required_effort_parameter_nm=.2)

    def _fresh_observation(self, observation, capture_started):
        cameras = observation.get("cameras", {}) if isinstance(observation, dict) else {}
        if set(cameras) != {"front", "left_hand", "right_hand"}:
            raise FastSafetyError("All three current RGB views must be recorded")
        stamps = [view.get("timestamp") for view in cameras.values()]
        if any(type(stamp) not in (int, float) or not math.isfinite(stamp) for stamp in stamps):
            raise FastSafetyError("RGB views need finite capture timestamps")
        if (min(stamps) < capture_started or max(stamps) > time.time()
                or self._last_observation_time is not None and min(stamps) <= self._last_observation_time):
            raise FastSafetyError("RGB snapshot is historical, repeated, or not from this capture")
        self._last_observation_time = max(stamps)
        self._observation_receipt = validate_fresh_observation(
            observation, previous_observation=self._observation_receipt,
            require_identity=True, **self.observation_contract)
        return self._observation_receipt

    @staticmethod
    def _arrival(decision, result):
        flag = "stability_confirmed" if decision.action == "gripper" else "arrival_confirmed"
        if (not isinstance(result, dict) or result.get("status") != "command_observed_stable"
                or result.get(flag) is not True):
            raise FastSafetyError("ROS result lacks independently confirmed mechanical arrival")
        receipt = result.get("receipt", {})
        if (not isinstance(receipt, dict) or receipt.get("event") != "command_observed_stable"
                or type(result.get("command_sequence")) is not int
                or result["command_sequence"] <= 0
                or receipt.get("sequence") != result["command_sequence"]):
            raise FastSafetyError("ROS arrival has no matching driver transaction receipt")

    def _visual_report(self, decision, row):
        evidence = decision.arguments.get("evidence")
        result = dict(status="observed")
        if evidence == "no_progress":
            result["visual_progress"] = False
        elif evidence == "target_lost":
            result["target_visible"] = False
        elif evidence == "grasp_failed":
            result["grasp_verified"] = False
        if evidence in _FAILURES:
            self.retry_count += 1
            self.memory = "Model reported: " + evidence
        # Only explicit model reports from this step's fresh RGB are visual
        # evidence. Motor arrival and jaw width are never visual task evidence.
        stage = None
        if decision.action == "advance" and evidence == "phase_complete":
            stage = {"VERIFY_GRASP": "grasp", "LIFT": "lift", "ALIGN_HOLDER": "transport"}.get(self.phase)
        elif (self.phase == "VERIFY_SUCCESS" and decision.action == "observe"
              and evidence == "phase_complete" and self.execution_evidence.get("release_open_completed")):
            stage = "release"
            self.execution_evidence["release_visually_confirmed"] = True
            self.memory = "Release visually reported; retreat once, then inspect new RGB for final success."
        elif self.phase == "VERIFY_SUCCESS" and decision.action == "advance" and evidence == "success":
            if not self.execution_evidence.get("release_visually_confirmed"):
                raise FastSafetyError("Fresh post-release visual report is required before final success")
            stage = "stable"
        if stage:
            self.visual_evidence.append(dict(stage=stage, confirmed=True, source="model_visual_report",
                reference="steps.jsonl:step_id=%d;observation_and_model_response_in_events.jsonl" % row["step_id"],
                at=time.time(), reported_by="model", phase=self.phase,
                evidence_grade="model_visual_report",
                observation_id=self._observation_receipt["observation_id"]))
        return result

    def _mechanical_report(self, decision, result):
        # Compact model input deliberately omits adapter's grasp_verified=False
        # placeholder: absence of visual verification is not failed grasp proof.
        self.previous_result = {"status": "command_observed_stable"}
        if decision.action == "gripper":
            flag = {"GRASP": "grasp_close_completed", "RELEASE": "release_open_completed"}.get(self.phase)
        else:
            flag = {"LIFT": "lift_completed", "INSERT": "insertion_move_completed",
                    "VERIFY_SUCCESS": "retreat_after_release_completed"}.get(self.phase)
        if flag:
            self.execution_evidence[flag] = True

    def run(self):
        if self._used:
            raise FastSafetyError("A live run cannot be resumed or implicitly retried")
        self._used = True
        metrics = FastRunMetrics(self.recorder)
        deadline = metrics.started + self.options["max_elapsed_s"]
        phase_steps = recoveries = 0
        reason, admitted, successful = "preflight_blocked", False, False
        try:
            self._preflight()
            admitted = True
            self.recorder.event("pipeline_contract", pipeline_contract())
            self.recorder.event("model_visual_latency_contract", {
                "model_visual_observation_refreshed_after_response": False,
                "assumption": "Scene objects, people and camera poses remain unchanged during the decision wait; this is an operating assumption, not a detector result.",
                "fresh_robot_feedback_scope": "Robot health/pose only; it cannot prove that people, the pen or the holder have not moved.",
                "old_rgb_timestamp_relabelled": False,
                "subsequent_closed_loop_observation": "All three cameras acquire new RGB before each next model decision.",
            })
            if getattr(self.model, "source", None) == "scripted":
                raise FastSafetyError("Scripted decisions cannot control the live robot")
            if hasattr(self.model, "prepare"):
                self.recorder.event("decision_backend_prepared", self.model.prepare())
            reason = "step_budget_exhausted"
            for step_id in range(1, self.options["max_steps"] + 1):
                budgets = ((self._decision_calls >= self.options["max_model_calls"], "model_call_budget_exhausted"),
                           (time.monotonic() >= deadline, "wall_time_budget_exhausted"),
                           (self.retry_count >= self.options["max_retries"], "retry_budget_exhausted"),
                           (recoveries > self.options["max_recoveries"], "recovery_budget_exhausted"))
                expired = next((label for condition, label in budgets if condition), None)
                if expired:
                    reason = expired
                    break
                started, before = time.monotonic(), self.phase
                row = dict(step_id=step_id, phase=before, timestamp=time.time(),
                    pipeline_id=SINGLE_ARM_PIPELINE_ID, pipeline_stage="decide_one",
                    pipeline=compact_pipeline(), pipeline_trace=["observe_scene"],
                    model_request_start=None, model_response_end=None, agent_decide_s=0.0,
                    image_capture_s=0.0, image_encode_s=0.0, robot_execute_s=0.0, robot_wait_s=0.0,
                    total_step_s=0.0, input_tokens=None, output_tokens=None, reasoning_output_tokens=None,
                    selected_camera_views=[], action=None, action_arguments=None, confidence=None,
                    previous_result=copy.deepcopy(self.previous_result), retry_count=self.retry_count,
                    phase_transition=None, decision_requested=False, action_dispatched=False,
                    dispatch_attempted=False, command_receipt_confirmed=False, result=None,
                    model_rgb_oldest_timestamp=None, model_rgb_age_at_response_s=None,
                    model_rgb_age_at_dispatch_s=None, model_visual_observation_refreshed=False,
                    robot_stability_across_decision=None,
                    consecutive_observe_unknown_before=self.consecutive_observe_unknown,
                    decision_source=getattr(self.model, "source", "responses_api"))
                fatal = False
                try:
                    tick, capture_started = time.monotonic(), time.time()
                    try:
                        observation = self.cameras.capture()
                        row["observation_receipt"] = self._fresh_observation(observation, capture_started)
                        row["pipeline_trace"].append("check_fresh_observation")
                        row["model_rgb_oldest_timestamp"] = min(view["timestamp"] for view in observation["cameras"].values())
                    finally:
                        row["image_capture_s"] = time.monotonic() - tick
                    measured = self.robot.observe()
                    state = compact_controller_state(dict(phase=self.phase,
                        robot_state=measured["robot_state"], gripper_state=measured["gripper_state"],
                        action_budget=self._budget(), previous_action=self.previous_action,
                        previous_result=self.previous_result, retry_count=self.retry_count, memory=self.memory))
                    self.recorder.event("controller_observation", observation, step_id=step_id)
                    self.recorder.event("controller_state", state, step_id=step_id)
                    if time.monotonic() >= deadline:
                        reason, fatal = "wall_time_budget_exhausted", True
                        raise TimeoutError(reason)
                    self.model.last_metrics = {}
                    row["decision_requested"] = True
                    try:
                        self._decision_calls += 1
                        raw = self.model.decide(state, observation)
                    finally:
                        row["model_rgb_age_at_response_s"] = time.time() - row["model_rgb_oldest_timestamp"]
                        for key, value in getattr(self.model, "last_metrics", {}).items():
                            if key in row or key in ("reasoning_effort", "requested_model", "actual_model", "request_id"):
                                # Never let backend metrics overwrite controller-owned evidence.
                                if key in ("model_request_start", "model_response_end", "agent_decide_s", "image_encode_s",
                                           "input_tokens", "output_tokens", "reasoning_output_tokens", "selected_camera_views",
                                           "reasoning_effort", "requested_model", "actual_model", "request_id"):
                                    row[key] = value
                    self.recorder.event("controller_response", raw, step_id=step_id)
                    decision = parse_response(raw, require_explanation=requires_explanation(state))
                    row["pipeline_trace"].append("decide_one")
                    validate_decision(decision, self.phase, limits=self.limits,
                        controller_state=dict(state, execution_evidence=self.execution_evidence))
                    row.update(action=decision.action, action_arguments=decision.arguments, confidence=decision.confidence)
                    if time.monotonic() >= deadline:
                        reason, fatal = "response_arrived_after_deadline", True
                        raise TimeoutError(reason)
                    self.previous_action = decision.to_dict()
                    if decision.action == "pause":
                        self.previous_result = row["result"] = {"status": "paused"}
                        reason, fatal = "model_requested_pause", True
                        self.robot.latch_failure(reason)
                    elif decision.confidence < .65:
                        self.consecutive_observe_unknown = 0
                        self.previous_result = {"status": "rejected", "error_code": "low_confidence"}
                        self.retry_count += 1
                        self.memory = "Low confidence; reconsider using all three current RGB views."
                    else:
                        repeated_observe = (decision.action == "observe"
                                            and decision.arguments.get("evidence") == "unknown")
                        self.consecutive_observe_unknown = self.consecutive_observe_unknown + 1 if repeated_observe else 0
                        if decision.action in _PHYSICAL_ACTIONS:
                            try:
                                current = self.robot.observe()
                                row["robot_stability_across_decision"] = verify_decision_robot_stability(measured, current)
                                # Fresh robot health and independent safety validation
                                # remain mandatory despite a successful preflight.
                                self.robot.validate(decision, current)
                                row["pipeline_trace"].append("admit_action")
                            except BaseException:
                                reason, fatal = "safety_validation_failed_latched", True
                                self.robot.latch_failure(reason)
                                raise
                            if time.monotonic() >= deadline:
                                reason, fatal = "dispatch_deadline_exceeded", True
                                raise TimeoutError(reason)
                            row["model_rgb_age_at_dispatch_s"] = time.time() - row["model_rgb_oldest_timestamp"]
                            row["dispatch_attempted"] = True
                            row["pipeline_trace"].append("dispatch_once")
                            tick = time.monotonic()
                            try:
                                result = self.robot.execute(decision)
                                self._arrival(decision, result)
                            except BaseException:
                                fatal, reason = True, "execution_failed_latched"
                                self.robot.latch_failure(reason)
                                raise
                            finally:
                                row["robot_execute_s"] = time.monotonic() - tick
                            wait = result.get("robot_wait_s")
                            if type(wait) not in (int, float) or not math.isfinite(wait) or wait < 0:
                                raise FastSafetyError("ROS adapter did not supply valid wait timing")
                            row["robot_wait_s"] = wait
                            row["robot_execute_s"] = max(0.0, row["robot_execute_s"] - wait)
                            row.update(result=result, action_dispatched=True, command_receipt_confirmed=True)
                            row["pipeline_trace"].append("read_receipt")
                            self._mechanical_report(decision, result)
                            row["visual_verification"] = "pending_new_observation"
                        else:
                            # advance and observe never enter the ROS executor.
                            self.previous_result = row["result"] = self._visual_report(decision, row)
                            row["pipeline_trace"].append("verify_visual")
                            row["visual_verification"] = "model_report_from_current_observation"
                        self.phase = completion_phase(decision)
                        # Require explicit fresh visual evidence after lift and
                        # final holder alignment before committing the transition.
                        if decision.action in ("move_eef", "move_eef_chunk") and before in ("LIFT", "ALIGN_HOLDER"):
                            self.phase = before
                            self.memory = "Action arrived. Inspect new RGB; advance with phase_complete only if this phase is visually complete."
                        if before == "RELEASE" and self.phase == "VERIFY_SUCCESS":
                            self.memory = "Inspect new RGB and report observe phase_complete if released; then retreat and reobserve before success."
                        if self.phase != before:
                            self.consecutive_observe_unknown = 0
                            phase_steps = 0
                            if self.phase in ("INIT", "GRASP"):
                                self.execution_evidence.clear()
                                self.visual_evidence.clear()
                            if self.phase != "RECOVERY":
                                self.retry_count = 0
                                if before != "RELEASE":
                                    self.memory = ""
                        else:
                            phase_steps += 1
                        # This is an action-event counter, not a judgement of
                        # image change, target visibility, or task progress.
                        if self.consecutive_observe_unknown >= 1:
                            self.memory = ("observe(unknown) did not change the scene. Choose one bounded "
                                           "supported action or pause with the missing fact; do not repeat the "
                                           "same observation. A move_eef proposal is not measured object coordinates.")
                        if self.consecutive_observe_unknown >= 3:
                            self.retry_count = max(1, self.retry_count)
                        if self.consecutive_observe_unknown >= self.options["max_observe_unknown"]:
                            reason, fatal = "observe_unknown_budget_exhausted", True
                            self.robot.latch_failure(reason)
                    if not fatal and phase_steps >= self.options["max_phase_steps"]:
                        self.phase, phase_steps = "RECOVERY", 0
                        self.retry_count += 1
                        self.memory = "Phase step budget reached; report the current visual problem or pause."
                except KeyboardInterrupt:
                    reason, fatal = "operator_interrupted", True
                    self.robot.latch_failure(reason)
                    row["error"] = reason
                except Exception as exc:
                    row["error"] = type(exc).__name__
                    self.recorder.event("controller_error", {"step_id": step_id, "type": type(exc).__name__, "message": str(exc)[:1000]})
                    self.previous_result = {"status": "rejected", "error_code": type(exc).__name__}
                    # Any execution attempt is never automatically retried, even
                    # when post-execution logging/verification fails.
                    if row["dispatch_attempted"]:
                        fatal, reason = True, "execution_failed_latched"
                        self.robot.latch_failure(reason)
                    elif not fatal:
                        self.phase = "RECOVERY"
                        self.retry_count += 1
                        self.memory = "Step rejected before dispatch; inspect new RGB. No automatic action retry."
                finally:
                    row["phase_transition"] = {"from": before, "to": self.phase}
                    row["retry_count_after"] = self.retry_count
                    row["consecutive_observe_unknown_after"] = self.consecutive_observe_unknown
                    row["total_step_s"] = time.monotonic() - started
                    if self.phase == "RECOVERY" and before != "RECOVERY":
                        recoveries += 1
                    metrics.step(row)
                if fatal:
                    break
                if self.phase == "DONE":
                    self.recorder.event("atomic_verify_task_evidence", validate_task_evidence(self.visual_evidence))
                    reason, successful = "model_reported_visual_completion", True
                    break
        except (Exception, KeyboardInterrupt) as exc:
            reason = "preflight_blocked" if not admitted else "live_initialization_failed"
            self.recorder.event("live_run_blocked", {"type": type(exc).__name__, "message": str(exc)[:1000]})
        finally:
            for name, adapter in (("cameras", self.cameras), ("robot", self.robot)):
                try:
                    adapter.close()  # The adapter must never close the shared driver.
                except Exception as exc:
                    reason, successful = "live_cleanup_failed", False
                    self.recorder.event("live_cleanup_error", {"adapter": name, "type": type(exc).__name__})
        uncertain = bool(getattr(self.robot, "target_uncertain", False)) or any(
            row["dispatch_attempted"] and not row["command_receipt_confirmed"] for row in metrics.steps)
        self.recorder.event("atomic_verify_return", {
            "return_verified": False, "reason": "current_pen_runner_has_no_return_motion_stage",
            "task_result_is_separate_from_return_result": True})
        return metrics.finish(termination_reason=reason, phase=self.phase, nonphysical=False,
            physical_loop=True, task_evidence=self.visual_evidence,
            model_reported_success=successful, target_uncertain=uncertain)
