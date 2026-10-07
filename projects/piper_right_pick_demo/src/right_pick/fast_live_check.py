"""One real RGB/ROS -> real model -> validated proposal integration test.

No execution method is called here, even for an otherwise valid proposal.
This test cannot establish a grasp, a hold qualification, or task success.
"""
import time

from .fast_policy import compact_controller_state, completion_phase, parse_response, requires_explanation
from .fast_recording import FastRunMetrics


def run_live_check(*, model, robot, cameras, recorder, phase="INIT"):
    from .fast_live_policy import assess_live_proposal
    if getattr(robot, "nonphysical", None) is not False or getattr(cameras, "nonphysical", None) is not False:
        raise ValueError("live-check requires actual sensor adapters; no mock fallback")
    if getattr(model, "source", "responses_api") == "scripted":
        raise ValueError("live-check requires a real decision backend")
    metrics = FastRunMetrics(recorder)
    started = time.monotonic()
    row = dict(step_id=1, phase=phase, timestamp=time.time(),
               model_request_start=None, model_response_end=None, agent_decide_s=0.0,
               image_capture_s=0.0, image_encode_s=0.0, robot_execute_s=0.0,
               robot_wait_s=0.0, total_step_s=0.0, input_tokens=None,
               output_tokens=None, reasoning_output_tokens=None, selected_camera_views=[],
               action=None, action_arguments=None, confidence=None, previous_result=None,
               retry_count=0, phase_transition={"from": phase, "to": phase},
               action_dispatched=False, decision_requested=False,
               decision_source=getattr(model, "source", "responses_api"),
               execution_scope="live_proposal_only")
    passed, reason = False, "live_check_failed"
    try:
        # Version/catalog preparation makes no model request and should finish
        # before collecting the short-lived current RGB/feedback packet.
        if hasattr(model, "prepare"):
            recorder.event("decision_backend_prepared", model.prepare())
        # Start and verify the read-only feedback connection before camera
        # startup, then take new feedback after the complete RGB snapshot.
        robot.observe()
        tick = time.monotonic()
        try:
            observation = cameras.capture()
        finally:
            row["image_capture_s"] = time.monotonic() - tick
        measured = robot.observe()
        recorder.event("live_observation", observation)
        recorder.event("live_robot_feedback", measured)
        state = compact_controller_state({"phase": phase, "robot_state": measured["robot_state"],
                                          "gripper_state": measured["gripper_state"],
                                          "previous_action": None, "previous_result": None,
                                          "retry_count": 0, "memory": ""})
        recorder._write_json("live_controller_state.json", state)
        row["decision_requested"] = True
        model.last_metrics = {}
        try:
            proposal = model.decide(state, observation)
        finally:
            data = getattr(model, "last_metrics", {})
            for key in ("model_request_start", "model_response_end", "agent_decide_s", "image_encode_s",
                        "selected_camera_views", "reasoning_effort", "input_tokens", "output_tokens",
                        "reasoning_output_tokens", "requested_model", "actual_model", "request_id"):
                if key in data:
                    row[key] = data[key]
        recorder._write_json("live_model_proposal.json", proposal)
        decision = parse_response(proposal, require_explanation=requires_explanation(state))
        row.update(action=decision.action, action_arguments=decision.arguments, confidence=decision.confidence)
        # A new feedback observation is always acquired after model latency.
        # It is logged separately; it does not refresh or relabel the old RGB.
        post_model = robot.observe()
        recorder.event("post_model_robot_feedback", post_model)
        # The compact packet is the model's frozen input. Raw post-response
        # feedback is used only by the local command-envelope checker.
        assessment_state = dict(state)
        assessment_state.update(raw_telemetry=post_model["raw_telemetry"],
                                provenance=post_model.get("provenance", {}),
                                robot_state=post_model["robot_state"],
                                gripper_state=post_model["gripper_state"])
        assessment = assess_live_proposal(proposal, assessment_state, robot)
        assessment["post_model_feedback_acquired"] = True
        assessment["model_visual_observation_refreshed"] = False
        assessment["action_dispatched"] = False
        row["proposal_assessment"] = assessment
        row["phase_transition"]["proposed_to"] = completion_phase(decision)
        recorder._write_json("live_proposal_assessment.json", assessment)
        command_ok = (decision.action not in ("move_eef", "move_eef_chunk", "gripper")
                      or assessment.get("command_encoding_valid") is True)
        passed = bool(assessment.get("schema_valid") and assessment.get("phase_valid") and command_ok)
        reason = "live_sensor_model_integration_passed" if passed else "live_model_proposal_rejected"
    except Exception as exc:
        row["error"] = type(exc).__name__
        recorder.event("live_check_error", {"type": type(exc).__name__, "message": str(exc)[:1000]})
        reason = "live_check_" + type(exc).__name__
    finally:
        for name, adapter in (("cameras", cameras), ("robot", robot)):
            try:
                adapter.close()
            except Exception as exc:
                passed = False
                reason = "live_check_cleanup_failed"
                recorder.event("live_check_cleanup_error", {"adapter": name, "type": type(exc).__name__})
        row["total_step_s"] = time.monotonic() - started
        metrics.step(row)
    return metrics.finish(termination_reason=reason, phase=phase, nonphysical=False,
                          live_check=True, live_check_passed=passed)
