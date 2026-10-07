"""Offline controller contracts; every camera/ROS operation is patched in-process.

These tests do not commission a physical robot or exercise an actual ROS node.
"""
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from right_pick.fast_live_loop import FastLiveClosedLoop, verify_preflight_contract
from right_pick.fast_observation import SubprocessRGBCameras
from right_pick.fast_policy import Decision, reasoning_effort, requires_explanation, select_camera_views
from right_pick.fast_replay import demonstration_decisions, mock_limits
from right_pick.fast_ros import ROSRightArm
from right_pick.fast_safety import FastSafetyError
from right_pick.recording import Recorder


def advance(phase, target):
    return dict(phase=phase, action="advance", arguments=dict(next_phase=target, evidence="phase_complete"), confidence=.9)


def observe(phase, evidence="unknown"):
    return dict(phase=phase, action="observe", arguments=dict(evidence=evidence), confidence=.9)


def complete_decisions():
    result = []
    for decision in demonstration_decisions():
        if decision["action"] == "gripper":
            decision["arguments"]["effort_parameter_nm"] = .2
        result.append(decision)
        if decision["phase"] == "LIFT" and decision["action"] == "move_eef":
            result.append(advance("LIFT", "APPROACH_HOLDER"))
        elif decision["phase"] == "ALIGN_HOLDER" and decision["action"] == "move_eef":
            result.append(advance("ALIGN_HOLDER", "INSERT"))
        elif decision["phase"] == "RELEASE":
            result.append(observe("VERIFY_SUCCESS", "phase_complete"))
    return result


class FakeDecisionBackend:
    source = "offline_test_backend"

    def __init__(self, decisions):
        self.decisions = list(decisions)
        self.calls = []
        self.last_metrics = {}

    def decide(self, state, observation):
        self.calls.append((copy.deepcopy(state), copy.deepcopy(observation)))
        self.last_metrics = dict(agent_decide_s=0., image_encode_s=0., selected_camera_views=["front", "right_hand"])
        if not self.decisions:
            raise ValueError("offline test decisions exhausted")
        return copy.deepcopy(self.decisions.pop(0))


class LiveLoopContracts(unittest.TestCase):
    def build(self, directory, decisions=None, options=None, admitted=True):
        limits = mock_limits()
        recorder = Recorder(Path(directory) / "runs", {}, "offline patched contract test", mode="physical")
        model = FakeDecisionBackend(complete_decisions() if decisions is None else decisions)
        robot = ROSRightArm(dict(ros={"command_log": "/offline/no-driver-log"}, physical_limits=limits), proposal_only=False)
        cameras = SubprocessRGBCameras(dict(backend="realsense", camera_python=sys.executable,
            cameras={name: {} for name in ("front", "left_hand", "right_hand")}), recorder.run_dir / "observations")
        state = {"robot_state": dict(pose_m_rad=[.25, 0, .25, 0, 0, 0], joints_rad=[0]*6,
            enabled=True, moving=False, binding_verified=True, arm_status=0, err_code=0),
            "gripper_state": {"opening_m": .04}, "raw_telemetry": {}, "provenance": {}}
        def measured():
            state["robot_state"]["sampled_at"] = time.time()
            return copy.deepcopy(state)
        frame_counter = [0]
        def captured():
            frame_counter[0] += 1
            stamp = time.time()
            return {"capture_id": "offline-" + str(frame_counter[0]),
                    "cameras": {name: dict(timestamp=stamp, host_received_at_s=stamp,
                        serial="offline-" + name, frame_number=frame_counter[0], rgb_path="/offline/" + name + ".png")
                    for name in ("front", "left_hand", "right_hand")}}
        counter = [0]
        def executed(decision):
            counter[0] += 1
            if decision.action == "gripper":
                state["gripper_state"]["opening_m"] = decision.arguments["opening_m"]
            else:
                state["robot_state"]["pose_m_rad"] = decision.arguments["pose_m_rad"]
            return dict(status="command_observed_stable", arrival_confirmed=decision.action != "gripper",
                stability_confirmed=True, command_sequence=counter[0], robot_wait_s=0.,
                jaw_target_reached=False, grasp_verified=False, task_success=False,
                receipt={"event": "command_observed_stable", "sequence": counter[0]})
        cameras.capture, cameras.close = Mock(side_effect=captured), Mock()
        robot.observe, robot.validate = Mock(side_effect=measured), Mock()
        robot.execute, robot.close = Mock(side_effect=executed), Mock()
        if admitted:
            # Test-only monkeypatch of the real adapter verifier. No production
            # fixture/boolean can produce or consume this fictional receipt.
            robot.preflight = Mock(return_value=dict(physical_execution_ready=True, physical_motion_authorized=False,
                timeout_policy_verified=True, hold_verified=False,
                blockers=[], qualification={"scope": "offline_test_patch_never_site_evidence"}))
        else:
            robot.preflight = Mock(side_effect=FastSafetyError("current session not qualified"))
        loop = FastLiveClosedLoop(model=model, robot=robot, cameras=cameras, recorder=recorder,
                                  limits=limits, options=options)
        return loop, recorder, model, robot, cameras

    def rows(self, recorder):
        path = recorder.run_dir / "steps.jsonl"
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def test_closed_preflight_prevents_camera_model_and_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, cameras = self.build(directory, admitted=False)
            report = loop.run()
            self.assertEqual(report["termination_reason"], "preflight_blocked")
            self.assertEqual(report["step_count"], 0)
            self.assertIsNone(report["task_success"])
            self.assertEqual(report["control_commands_sent"], 0)
            self.assertFalse(model.calls)
            cameras.capture.assert_not_called()
            robot.execute.assert_not_called()
            self.assertTrue(robot.close.called and cameras.close.called)

    def test_plain_authorization_boolean_cannot_admit(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, cameras = self.build(directory)
            robot.preflight.return_value = {"physical_motion_authorized": True}
            self.assertEqual(loop.run()["termination_reason"], "preflight_blocked")
            self.assertFalse(model.calls)
            cameras.capture.assert_not_called()
            robot.execute.assert_not_called()

    def test_preflight_means_timeout_policy_ready_not_instantaneous_hold_or_action_authority(self):
        valid = dict(physical_execution_ready=True, physical_motion_authorized=False,
                     timeout_policy_verified=True, hold_verified=False, blockers=[],
                     qualification={"qualification_scope": "offline_fixture_bounded_goal_completion_only"})
        self.assertEqual(verify_preflight_contract(valid), valid)
        for changes in ({"hold_verified": True}, {"timeout_policy_verified": False},
                        {"physical_execution_ready": False}, {"qualification": {}}):
            with self.assertRaises(FastSafetyError):
                verify_preflight_contract(dict(valid, **changes))

    def test_full_contract_host_actions_fresh_views_and_recorded_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, cameras = self.build(directory)
            report = loop.run()
            rows = self.rows(recorder)
            self.assertEqual(report["final_phase"], "DONE", rows)
            self.assertTrue(report["model_reported_task_success"])
            self.assertIsNone(report["task_success"])
            self.assertFalse(report["task_evidence_order_valid"])
            self.assertEqual(len(rows), len(complete_decisions()))
            self.assertEqual(cameras.capture.call_count, len(rows))
            self.assertEqual(report["action_count"], robot.execute.call_count)
            self.assertEqual(report["control_commands_sent"], robot.execute.call_count)
            self.assertTrue(all(c.args[0].action not in ("observe", "advance") for c in robot.execute.call_args_list))
            self.assertEqual(robot.validate.call_count, robot.execute.call_count)
            grasp_follow = next(state for state, _ in model.calls if state["phase"] == "VERIFY_GRASP")
            self.assertEqual(grasp_follow["previous_result"], {"status": "command_observed_stable"})
            self.assertEqual([e["stage"] for e in loop.visual_evidence], ["grasp", "lift", "transport", "release", "stable"])
            self.assertTrue(all(e["source"] == "model_visual_report" for e in loop.visual_evidence))
            dispatched = [row for row in rows if row["action_dispatched"]]
            self.assertTrue(dispatched)
            self.assertTrue(all("verify_visual" not in row["pipeline_trace"] for row in dispatched))
            self.assertTrue(all(row["visual_verification"] == "pending_new_observation" for row in dispatched))
            verified = [row for row in rows if "verify_visual" in row["pipeline_trace"]]
            self.assertTrue(verified)
            self.assertTrue(all(not row["action_dispatched"] for row in verified))
            required = {"step_id", "phase", "timestamp", "model_request_start", "model_response_end",
                        "agent_decide_s", "image_capture_s", "image_encode_s", "robot_execute_s", "robot_wait_s",
                        "total_step_s", "input_tokens", "output_tokens", "reasoning_output_tokens", "selected_camera_views",
                        "action", "action_arguments", "previous_result", "confidence", "phase_transition", "retry_count"}
            self.assertTrue(all(required <= row.keys() for row in rows))
            self.assertTrue(any(row["phase"] == "LIFT" and row["phase_transition"]["to"] == "LIFT" for row in rows))
            self.assertTrue(all(row["model_visual_observation_refreshed"] is False for row in rows))
            self.assertTrue(all(row["model_rgb_age_at_response_s"] >= 0 for row in rows))
            self.assertTrue(all(row["model_rgb_age_at_dispatch_s"] >= row["model_rgb_age_at_response_s"]
                                for row in rows if row["dispatch_attempted"]))
            self.assertTrue(all(row["robot_stability_across_decision"] is not None
                                for row in rows if row["dispatch_attempted"]))

    def test_gripper_stability_does_not_require_empty_jaw_target(self):
        d = Decision("GRASP", "gripper", {"opening_m": 0., "effort_parameter_nm": .2}, .9)
        result = dict(status="command_observed_stable", stability_confirmed=True, arrival_confirmed=False,
                      jaw_target_reached=False, command_sequence=4,
                      receipt={"event": "command_observed_stable", "sequence": 4})
        FastLiveClosedLoop._arrival(d, result)
        result["stability_confirmed"] = False
        with self.assertRaises(FastSafetyError):
            FastLiveClosedLoop._arrival(d, result)

    def test_arrival_cannot_be_inferred_from_completed_or_wrong_receipt(self):
        d = Decision("LIFT", "move_eef", {}, .9)
        for result in ({"status": "completed"},
                       {"status": "command_observed_stable", "arrival_confirmed": True},
                       {"status": "command_observed_stable", "arrival_confirmed": True, "command_sequence": 1,
                        "receipt": {"event": "command_observed_stable", "sequence": 2}}):
            with self.assertRaises(FastSafetyError):
                FastLiveClosedLoop._arrival(d, result)

    def test_timeout_latches_stops_and_retains_uncertain_command_count(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, cameras = self.build(directory)
            robot.execute.side_effect = TimeoutError("firmware target outcome uncertain")
            report = loop.run()
            self.assertEqual(report["termination_reason"], "execution_failed_latched")
            self.assertEqual(robot.execute.call_count, 1)
            self.assertEqual(len(model.calls), 2)
            self.assertIsNone(report["control_commands_sent"])
            self.assertTrue(report["target_uncertain"])
            self.assertIsNotNone(robot.failure)
            self.assertEqual(loop.phase, "APPROACH_PEN")
            self.assertTrue(robot.close.called and cameras.close.called)

    def test_missing_mechanical_evidence_is_not_retried(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory)
            robot.execute.side_effect = None
            robot.execute.return_value = {"status": "completed", "robot_wait_s": 0.}
            report = loop.run()
            self.assertEqual(report["termination_reason"], "execution_failed_latched")
            self.assertEqual(robot.execute.call_count, 1)
            self.assertEqual(len(model.calls), 2)
            self.assertNotIn("lift_completed", loop.execution_evidence)

    def test_independent_guard_rejection_latches_before_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory)
            robot.validate.side_effect = FastSafetyError("current joint limit rejected")
            report = loop.run()
            self.assertEqual(report["termination_reason"], "safety_validation_failed_latched")
            self.assertEqual(len(model.calls), 2)
            self.assertEqual(report["control_commands_sent"], 0)
            self.assertFalse(report["target_uncertain"])
            self.assertIsNotNone(robot.failure)
            robot.execute.assert_not_called()

    def test_state_change_while_waiting_for_model_stops_without_another_model_call(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, _ = self.build(directory)
            original_observe = robot.observe.side_effect
            calls = [0]
            def moved():
                state = original_observe()
                calls[0] += 1
                if calls[0] == 3:  # INIT read, APPROACH model read, pre-dispatch read.
                    state["robot_state"]["pose_m_rad"][0] += .001
                return state
            robot.observe.side_effect = moved
            report = loop.run()
            self.assertEqual(report["termination_reason"], "safety_validation_failed_latched")
            self.assertEqual(len(model.calls), 2)
            self.assertEqual(report["control_commands_sent"], 0)
            self.assertTrue(all(row["model_visual_observation_refreshed"] is False for row in self.rows(recorder)))
            robot.execute.assert_not_called()

    def test_response_after_deadline_cannot_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory, options={"max_elapsed_s": 1})
            original_clock, offset = time.monotonic, [0.]
            decide = model.decide
            def late(state, observation):
                response = decide(state, observation)
                offset[0] = 10.
                return response
            model.decide = late
            with patch("right_pick.fast_live_loop.time.monotonic", side_effect=lambda: original_clock()+offset[0]):
                report = loop.run()
            self.assertEqual(report["termination_reason"], "response_arrived_after_deadline")
            robot.execute.assert_not_called()

    def test_observation_exhausting_deadline_skips_model_call(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, cameras = self.build(directory, options={"max_elapsed_s": 1})
            original_clock, offset = time.monotonic, [0.]
            capture = cameras.capture.side_effect
            def slow_capture():
                result = capture()
                offset[0] = 10.
                return result
            cameras.capture.side_effect = slow_capture
            with patch("right_pick.fast_live_loop.time.monotonic", side_effect=lambda: original_clock()+offset[0]):
                report = loop.run()
            self.assertEqual(report["termination_reason"], "wall_time_budget_exhausted")
            self.assertEqual(len(model.calls), 0)
            robot.execute.assert_not_called()

    def test_low_confidence_has_finite_step_budget(self):
        decision = observe("INIT")
        decision["confidence"] = .2
        decisions = [decision]
        for _ in range(10):
            following = copy.deepcopy(decision)
            following["explanation"] = "Current scene remains uncertain."
            decisions.append(following)
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory, decisions)
            report = loop.run()
            self.assertEqual(report["termination_reason"], "retry_budget_exhausted")
            self.assertEqual(len(model.calls), 2)
            robot.execute.assert_not_called()

    def test_unknown_observe_budget_can_be_explicitly_extended(self):
        decisions = [observe("INIT") for _ in range(3)]
        decisions += [dict(observe("INIT"), explanation="The static RGB views leave approach direction unresolved.") for _ in range(3)]
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, _ = self.build(directory, decisions,
                options={"max_phase_steps": 36, "max_steps": 120, "max_observe_unknown": 4})
            report = loop.run()
            self.assertEqual(report["termination_reason"], "observe_unknown_budget_exhausted")
            self.assertEqual(len(model.calls), 4)
            self.assertEqual(report["control_commands_sent"], 0)
            before, after = model.calls[2][0], model.calls[3][0]
            self.assertEqual(before["retry_count"], 0)
            self.assertEqual(after["retry_count"], 1)
            self.assertTrue(requires_explanation(after))
            self.assertEqual(reasoning_effort(after), "high")
            self.assertEqual(set(select_camera_views(after)), {"front", "left_hand", "right_hand"})
            self.assertLessEqual(len(after["memory"]), 240)
            self.assertNotIn("visual_progress", after["previous_result"])
            self.assertNotIn("target_visible", after["previous_result"])
            self.assertEqual([r["consecutive_observe_unknown_after"] for r in self.rows(recorder)], list(range(1, 5)))
            robot.execute.assert_not_called()

    def test_default_second_unknown_observe_ends_without_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, _ = self.build(directory,
                [observe("INIT"), observe("INIT")])
            report = loop.run()
            self.assertEqual(report["termination_reason"], "observe_unknown_budget_exhausted")
            self.assertEqual(len(model.calls), 2)
            self.assertEqual(model.calls[1][0]["retry_count"], 0)
            self.assertIn("missing fact", model.calls[1][0]["memory"])
            self.assertEqual([r["consecutive_observe_unknown_after"] for r in self.rows(recorder)], [1, 2])
            robot.execute.assert_not_called()

    def test_unknown_observe_streak_clears_on_explicit_phase_advance(self):
        decisions = [observe("INIT") for _ in range(3)]
        decisions += [dict(advance("INIT", "APPROACH_PEN"), explanation="The pen and holder are visible; inspect approach next."),
                      observe("APPROACH_PEN")]
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, _ = self.build(directory, decisions,
                options={"max_steps": 5, "max_observe_unknown": 4})
            report = loop.run()
            self.assertEqual(report["termination_reason"], "step_budget_exhausted")
            self.assertEqual(model.calls[4][0]["retry_count"], 0)
            self.assertEqual(model.calls[4][0]["memory"], "")
            self.assertEqual(self.rows(recorder)[-1]["consecutive_observe_unknown_after"], 1)
            robot.execute.assert_not_called()

    def test_pause_never_reaches_ros_executor(self):
        decision = dict(phase="INIT", action="pause", arguments={"evidence": "unknown"}, confidence=.1)
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory, [decision])
            self.assertEqual(loop.run()["termination_reason"], "model_requested_pause")
            self.assertEqual(len(model.calls), 1)
            robot.execute.assert_not_called()
            robot.validate.assert_not_called()

    def test_bad_schema_has_finite_retry_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory, [{"bad": "schema"}] * 10)
            report = loop.run()
            self.assertEqual(report["termination_reason"], "retry_budget_exhausted")
            self.assertEqual(len(model.calls), 2)
            robot.execute.assert_not_called()

    def test_one_schema_correction_allows_new_observation_and_normal_progress(self):
        recover = dict(advance("RECOVERY", "INIT"),
                       explanation="New RGB is usable; previous invalid JSON sent no command.")
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, _ = self.build(directory,
                [{"bad": "schema"}, recover] + complete_decisions())
            report = loop.run()
            self.assertEqual(report["final_phase"], "DONE", self.rows(recorder))
            self.assertTrue(report["model_reported_task_success"])
            self.assertEqual(report["recovery_count"], 1)
            self.assertEqual(len(model.calls), len(complete_decisions()) + 2)
            self.assertGreater(robot.execute.call_count, 0)

    def test_second_recovery_episode_stops_before_another_model_or_action(self):
        recover = dict(advance("RECOVERY", "INIT"), explanation="Fresh scene permits evaluation.")
        failure = dict(phase="INIT", action="advance", confidence=.9,
                       arguments={"next_phase": "RECOVERY", "evidence": "no_progress"})
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, _ = self.build(directory, [{"bad": "schema"}, recover, failure])
            report = loop.run()
            self.assertEqual(report["termination_reason"], "recovery_budget_exhausted")
            self.assertEqual(len(model.calls), 3)
            robot.execute.assert_not_called()

    def test_cached_camera_observation_cannot_drive_next_action(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, model, robot, cameras = self.build(directory)
            cached = {}
            capture = cameras.capture.side_effect
            def repeated():
                if not cached:
                    cached.update(capture())
                return copy.deepcopy(cached)
            cameras.capture.side_effect = repeated
            report = loop.run()
            self.assertEqual(report["termination_reason"], "retry_budget_exhausted")
            self.assertEqual(len(model.calls), 1)
            robot.execute.assert_not_called()

    def test_run_cannot_implicitly_resume(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, _, _, _, _ = self.build(directory, admitted=False)
            loop.run()
            with self.assertRaises(FastSafetyError):
                loop.run()

    def test_no_duck_typed_physical_boolean_adapter(self):
        with tempfile.TemporaryDirectory() as directory:
            loop, recorder, model, robot, cameras = self.build(directory)
            with self.assertRaises(FastSafetyError):
                FastLiveClosedLoop(model=model, robot=Mock(nonphysical=False), cameras=cameras,
                                   recorder=recorder, limits=mock_limits())


if __name__ == "__main__":
    unittest.main()
