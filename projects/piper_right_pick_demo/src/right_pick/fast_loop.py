"""Bounded phase controller; vision judgements come only from the model.

The supplied executable adapter in this release is nonphysical. A config flag
cannot turn it into a physical adapter or commission timeout/hold behaviour.
"""
import copy
import time

from .fast_recording import FastRunMetrics
from .fast_pipeline import SINGLE_ARM_PIPELINE_ID, compact_pipeline, pipeline_contract


class FastClosedLoop:
    def __init__(self, *, model, robot, cameras, recorder, limits, options=None):
        # Fail before opening a camera/model/robot. No duck-typed live fallback.
        from .fast_safety import require_nonphysical_runtime
        require_nonphysical_runtime(robot, cameras)
        self.model, self.robot, self.cameras = model, robot, cameras
        self.recorder, self.limits = recorder, limits
        self.options = dict(max_steps=32, max_model_calls=24,
                            max_retries=2, max_phase_steps=4,
                            max_recoveries=1, max_observe_unknown=2,
                            max_elapsed_s=900)
        self.options.update(options or {})
        for key, value in self.options.items():
            if key not in ("max_steps", "max_model_calls", "max_retries", "max_phase_steps",
                           "max_recoveries", "max_observe_unknown", "max_elapsed_s"):
                raise ValueError("Unknown controller option: " + key)
            if type(value) is not int or value <= 0:
                raise ValueError(key + " must be a positive integer")
        self.phase = "INIT"
        self.retry_count = 0
        self.previous_action = None
        self.previous_result = None
        self.memory = ""
        self.execution_evidence = {}

    def run(self):
        from .fast_policy import (parse_response, validate_decision, completion_phase,
                                  compact_controller_state, requires_explanation,
                                  phase_translation_fraction, phase_rotation_fraction)
        metrics = FastRunMetrics(self.recorder)
        self.recorder.event("pipeline_contract", pipeline_contract())
        deadline = metrics.started + self.options["max_elapsed_s"]
        phase_steps, recoveries, decision_calls = 0, 0, 0
        consecutive_observe_unknown = 0
        reason = "step_budget_exhausted"
        try:
            for step_id in range(1, self.options["max_steps"] + 1):
                if decision_calls >= self.options["max_model_calls"]:
                    reason = "model_call_budget_exhausted"
                    break
                if time.monotonic() >= deadline:
                    reason = "wall_time_budget_exhausted"
                    break
                if self.retry_count >= self.options["max_retries"]:
                    reason = "retry_budget_exhausted"
                    break
                if recoveries > self.options["max_recoveries"]:
                    reason = "recovery_budget_exhausted"
                    break
                started = time.monotonic()
                before = self.phase
                row = dict(step_id=step_id, phase=before, timestamp=time.time(),
                           pipeline_id=SINGLE_ARM_PIPELINE_ID, pipeline_stage="decide_one",
                           pipeline=compact_pipeline(), pipeline_trace=["observe_scene"],
                           model_request_start=None, model_response_end=None,
                           agent_decide_s=0.0, image_capture_s=0.0, image_encode_s=0.0,
                           robot_execute_s=0.0, robot_wait_s=0.0, total_step_s=0.0,
                           input_tokens=None, output_tokens=None, reasoning_output_tokens=None,
                           selected_camera_views=[], action=None, action_arguments=None,
                           previous_result=copy.deepcopy(self.previous_result), confidence=None,
                           phase_transition=None, retry_count=self.retry_count,
                           decision_source=getattr(self.model, "source", "responses_api"),
                           decision_requested=False, action_dispatched=False, result=None,
                           consecutive_observe_unknown_before=consecutive_observe_unknown)
                fatal = False
                try:
                    tick = time.monotonic()
                    try:
                        observation = self.cameras.capture()
                    finally:
                        row["image_capture_s"] = time.monotonic() - tick
                    measured = self.robot.observe()
                    budget = {
                        "max_translation_m": min(self.limits["max_translation_step_m"], self.limits["max_chunk_translation_m"]) * phase_translation_fraction(self.phase),
                        "max_rotation_rad": min(self.limits["max_rotation_step_rad"], self.limits["max_chunk_rotation_rad"]) * phase_rotation_fraction(self.phase),
                        "max_speed_percent": self.limits["max_speed_percent"],
                        "max_waypoints": min(3, self.limits["max_waypoints"]),
                        "gripper_min_m": self.limits["gripper_min_m"],
                        "gripper_max_m": self.limits["gripper_max_m"],
                        "max_effort_parameter_nm": self.limits["max_effort_parameter_nm"]}
                    state = compact_controller_state({
                        "phase": self.phase, "robot_state": measured["robot_state"],
                        "gripper_state": measured["gripper_state"],
                        "action_budget": budget,
                        "previous_action": self.previous_action,
                        "previous_result": self.previous_result,
                        "retry_count": self.retry_count, "memory": self.memory})
                    self.recorder.event("controller_observation", observation)
                    self.recorder.event("controller_state", state)
                    # Always new RGB before the next decision, including the
                    # post-close/post-lift/post-insert/post-release checkpoints.
                    self.model.last_metrics = {}
                    try:
                        row["decision_requested"] = True
                        decision_calls += 1
                        raw = self.model.decide(state, observation)
                    finally:
                        data = getattr(self.model, "last_metrics", {})
                        for key in ("model_request_start", "model_response_end", "agent_decide_s",
                                    "image_encode_s", "selected_camera_views", "reasoning_effort",
                                    "input_tokens", "output_tokens", "reasoning_output_tokens"):
                            if key in data:
                                row[key] = data[key]
                    self.recorder.event("controller_response", raw)
                    decision = parse_response(raw, require_explanation=requires_explanation(state))
                    row["pipeline_trace"].append("decide_one")
                    validate_decision(decision, self.phase, limits=self.limits,
                                      controller_state=dict(state, execution_evidence=self.execution_evidence))
                    row.update(action=decision.action, action_arguments=decision.arguments,
                               confidence=decision.confidence)
                    # Late responses cannot start another action after the run
                    # deadline, even when a transport timeout is still pending.
                    if time.monotonic() >= deadline:
                        reason = "response_arrived_after_deadline"
                        fatal = True
                        raise RuntimeError(reason)
                    self.previous_action = decision.to_dict()
                    if decision.action == "pause":
                        self.previous_result = {"status": "paused"}
                        row["result"] = self.previous_result
                        self.robot.latch_failure("model_requested_pause")
                        reason, fatal = "model_requested_pause", True
                    elif decision.confidence < 0.65:
                        consecutive_observe_unknown = 0
                        self.previous_result = {"status": "rejected", "error_code": "low_confidence"}
                        self.retry_count += 1
                        self.memory = "Last proposal was low confidence; reconsider with all views."
                    else:
                        repeated_observe = (decision.action == "observe"
                                            and decision.arguments.get("evidence") == "unknown")
                        consecutive_observe_unknown = (consecutive_observe_unknown + 1
                                                       if repeated_observe else 0)
                        # Re-read robot state immediately before dispatch. No
                        # response is retried after a partial/uncertain send.
                        current = self.robot.observe()
                        self.robot.validate(decision, current)
                        row["pipeline_trace"].append("admit_action")
                        tick = time.monotonic()
                        try:
                            row["pipeline_trace"].append("dispatch_once")
                            result = self.robot.execute(decision)
                        except BaseException:
                            self.robot.latch_failure("execution_failed_or_interrupted")
                            fatal = True
                            reason = "execution_failed_latched"
                            raise
                        finally:
                            row["robot_execute_s"] = time.monotonic() - tick
                        row["robot_wait_s"] = result.get("robot_wait_s", 0.0)
                        # execute() must return dispatch-only elapsed time if
                        # its implementation includes a separately timed wait.
                        row["robot_execute_s"] = max(0.0, row["robot_execute_s"] - row["robot_wait_s"])
                        row["action_dispatched"] = decision.action in ("move_eef", "move_eef_chunk", "gripper")
                        row["result"] = result
                        row["pipeline_trace"].append("read_receipt")
                        self.previous_result = result
                        if result.get("status") != "completed":
                            self.robot.latch_failure("non_completed_execution")
                            fatal = True
                            reason = "execution_not_completed_latched"
                        else:
                            if decision.action == "gripper":
                                if before == "GRASP":
                                    self.execution_evidence["grasp_close_completed"] = True
                                elif before == "RELEASE":
                                    self.execution_evidence["release_open_completed"] = True
                            if decision.action in ("move_eef", "move_eef_chunk"):
                                flag = {"LIFT": "lift_completed", "INSERT": "insertion_move_completed",
                                        "VERIFY_SUCCESS": "retreat_after_release_completed"}.get(before)
                                if flag:
                                    self.execution_evidence[flag] = True
                            evidence = decision.arguments.get("evidence")
                            if evidence in ("no_progress", "target_lost", "grasp_failed"):
                                self.retry_count += 1
                                self.memory = "Model reported: " + evidence
                            self.phase = completion_phase(decision)
                            if decision.action == "pause":
                                reason, fatal = "model_requested_pause", True
                            if self.phase != before:
                                phase_steps = 0
                                consecutive_observe_unknown = 0
                                if self.phase in ("INIT", "GRASP"):
                                    self.execution_evidence.clear()
                                elif self.phase == "INSERT":
                                    for key in ("insertion_move_completed", "release_open_completed", "retreat_after_release_completed"):
                                        self.execution_evidence.pop(key, None)
                                if self.phase != "RECOVERY":
                                    self.retry_count = 0
                                self.memory = ""
                            else:
                                phase_steps += 1
                            row["visual_verification"] = "historical_replay_only"
                            if not row["action_dispatched"]:
                                row["pipeline_trace"].append("verify_visual")
                            if consecutive_observe_unknown >= 1:
                                self.memory = ("observe(unknown) did not change the state. Choose one bounded action "
                                               "supported by the current RGB, or pause with the specific missing fact; "
                                               "do not repeat the same observation.")
                            if consecutive_observe_unknown >= 3:
                                self.retry_count = max(1, self.retry_count)
                            if consecutive_observe_unknown >= self.options["max_observe_unknown"]:
                                self.robot.latch_failure("observe_unknown_budget_exhausted")
                                reason, fatal = "observe_unknown_budget_exhausted", True
                            if phase_steps >= self.options["max_phase_steps"]:
                                self.phase = "RECOVERY"
                                self.retry_count += 1
                                self.memory = "Phase step budget reached; visual progress has not been established."
                                phase_steps = 0
                except KeyboardInterrupt:
                    reason, fatal = "operator_interrupted", True
                    self.robot.latch_failure(reason)
                    row["error"] = reason
                except Exception as exc:
                    # No raw network bodies, secrets or arbitrary exception
                    # content enters the next model request.
                    row["error"] = type(exc).__name__
                    self.recorder.event("controller_error", {"type": type(exc).__name__, "message": str(exc)[:1000]})
                    self.previous_result = {"status": "rejected", "error_code": type(exc).__name__}
                    if not fatal:
                        self.phase = "RECOVERY"
                        self.retry_count += 1
                        self.memory = "Previous step was rejected; no automatic action retry."
                finally:
                    row["phase_transition"] = {"from": before, "to": self.phase}
                    row["retry_count_after"] = self.retry_count
                    row["consecutive_observe_unknown_after"] = consecutive_observe_unknown
                    row["total_step_s"] = time.monotonic() - started
                    if self.phase == "RECOVERY" and before != "RECOVERY":
                        recoveries += 1
                    metrics.step(row)
                if fatal:
                    break
                if self.phase == "DONE":
                    reason = "nonphysical_replay_completed"
                    break
        finally:
            # For this nonphysical adapter, closing cannot command the robot.
            # A future live adapter needs a separately qualified lifecycle.
            self.cameras.close()
            self.robot.close()
        return metrics.finish(termination_reason=reason, phase=self.phase, nonphysical=True,
                              replay_completed=self.phase == "DONE")
