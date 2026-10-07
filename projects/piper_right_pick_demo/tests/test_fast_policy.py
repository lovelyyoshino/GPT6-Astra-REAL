"""Offline protocol tests: no model, camera, robot, ROS, or CAN access."""
from copy import deepcopy
import json
import math
import unittest

from right_pick.fast_policy import (
    FastPolicyError, PHASES, compact_controller_state, completion_phase,
    parse_response, phase_spec, phase_translation_fraction, phase_rotation_fraction,
    reasoning_effort, requires_explanation,
    response_schema, select_camera_views, validate_decision,
)


def decision(phase="INIT", action="observe", arguments=None, confidence=0.8):
    return {"phase": phase, "action": action,
            "arguments": {"evidence": "unknown"} if arguments is None else arguments,
            "confidence": confidence}


def advance(phase, target, evidence="phase_complete"):
    return decision(phase, "advance", {"next_phase": target, "evidence": evidence})


def move(phase="APPROACH_PEN", target=None, chunk=False):
    args = {"speed_percent": 1, "next_phase": target}
    args["waypoints" if chunk else "pose_m_rad"] = ([[0.2, 0, 0.3, 0, 1, 0]] if chunk else [0.2, 0, 0.3, 0, 1, 0])
    return decision(phase, "move_eef_chunk" if chunk else "move_eef", args)


def state(phase="INIT", **extra):
    result = {"phase": phase, "robot_state": {"pose_m_rad": [0.2, 0, 0.3, 0, 1, 0],
              "joints_rad": [0] * 6, "arm_status": 0, "err_code": 0,
              "enabled": True, "moving": False, "binding_verified": True, "sampled_at": 100},
              "gripper_state": {"opening_m": 0.035, "effort": 0.1}}
    result.update(extra)
    return result


class FastPolicyTests(unittest.TestCase):
    # These are explicitly nonphysical test placeholders, never execution limits.
    semantic_context = {"test_only_not_execution_authorization": True}

    def check(self, value, current=None, measured=None):
        return validate_decision(value, current or value["phase"], limits=self.semantic_context,
                                 controller_state=measured or state(value["phase"]))

    def test_exact_four_fields_and_exception_explanation_contract(self):
        raw = decision()
        self.assertEqual(parse_response(json.dumps(raw)).to_dict(), raw)
        for bad in (dict(raw, reason="extra"), dict(raw, explanation="not requested")):
            with self.assertRaises(FastPolicyError):
                parse_response(bad)
        with self.assertRaises(FastPolicyError):
            parse_response(raw, require_explanation=True)
        exceptional = dict(raw, explanation="Target not visible in the current image.")
        self.assertEqual(parse_response(exceptional, True).explanation, exceptional["explanation"])
        with self.assertRaises(FastPolicyError):
            parse_response(dict(raw, explanation="x" * 241), True)
        self.assertEqual(set(response_schema()["required"]), {"phase", "action", "arguments", "confidence"})
        self.assertIn("explanation", response_schema(True)["required"])

    def test_duplicate_keys_extra_commands_and_nonfinite_numbers_rejected(self):
        with self.assertRaisesRegex(FastPolicyError, "duplicate"):
            parse_response('{"phase":"INIT","phase":"DONE","action":"observe","arguments":{"evidence":"unknown"},"confidence":1}')
        for bad in (dict(decision(), confidence=True), dict(decision(), confidence=math.nan),
                    dict(decision(), confidence=2), dict(move(), arguments={"pose_m_rad": [0]*6, "speed_percent": 1, "next_phase": None, "gripper": 0.01}),
                    dict(decision(), action="enable"), dict(decision(), action="quick_stop"),
                    dict(decision(), actions=[move(), decision()])):
            with self.subTest(bad=bad), self.assertRaises(FastPolicyError):
                parse_response(bad)
        bad = move()
        bad["arguments"]["pose_m_rad"][2] = math.inf
        with self.assertRaises(FastPolicyError):
            parse_response(bad)

    def test_action_and_argument_union_still_checked_semantically(self):
        with self.assertRaises(FastPolicyError):
            parse_response(decision("INIT", "gripper", {"evidence": "unknown"}))
        with self.assertRaises(FastPolicyError):
            parse_response(decision("INIT", "move_eef", {"opening_m": 0.03, "effort_parameter_nm": 0.2}))

    def test_no_missing_limits_or_measured_state_for_motion(self):
        for limits, measured in ((None, state()), ({}, state()), (self.semantic_context, None)):
            with self.subTest(limits=limits), self.assertRaises(FastPolicyError):
                validate_decision(move(), "APPROACH_PEN", limits=limits, controller_state=measured)

    def test_current_phase_binding_and_no_phase_jump(self):
        with self.assertRaises(FastPolicyError):
            self.check(move(), current="INSERT")
        with self.assertRaises(FastPolicyError):
            self.check(advance("INIT", "DONE", "success"))
        self.check(advance("INIT", "APPROACH_PEN"))
        for phase, target in (("GRASP", "VERIFY_GRASP"), ("RELEASE", "VERIFY_SUCCESS")):
            with self.assertRaises(FastPolicyError):
                self.check(advance(phase, target))

    def test_waypoint_chunk_is_finite_and_cannot_cross_contact_phase(self):
        accepted = self.check(move(target="ALIGN_PEN", chunk=True))
        self.assertEqual(completion_phase(accepted), "ALIGN_PEN")
        bad = move(chunk=True)
        bad["arguments"]["waypoints"] *= 4
        with self.assertRaises(FastPolicyError):
            parse_response(bad)
        for phase in ("PREGRASP", "INSERT", "GRASP", "VERIFY_SUCCESS", "LIFT"):
            with self.subTest(phase=phase), self.assertRaises(FastPolicyError):
                self.check(move(phase, chunk=True))
        with self.assertRaises(FastPolicyError):
            self.check(move(target="GRASP", chunk=True))

    def test_motion_cannot_skip_visual_verification(self):
        for phase, target in (("INSERT", "RELEASE"), ("VERIFY_GRASP", "LIFT"),
                              ("VERIFY_SUCCESS", "DONE")):
            with self.subTest(phase=phase), self.assertRaises(FastPolicyError):
                self.check(move(phase, target))
        self.check(move("VERIFY_SUCCESS"))  # A retreat itself is permitted.

    def test_mechanical_gripper_completion_does_not_claim_visual_success(self):
        close = decision("GRASP", "gripper", {"opening_m": 0.006, "effort_parameter_nm": 0.2})
        self.assertEqual(completion_phase(self.check(close)), "VERIFY_GRASP")
        release_state = state("RELEASE")
        release_state["gripper_state"]["opening_m"] = 0.009
        release = decision("RELEASE", "gripper", {"opening_m": 0.035, "effort_parameter_nm": 0.2})
        self.assertEqual(completion_phase(self.check(release, measured=release_state)), "VERIFY_SUCCESS")
        for bad in (decision("GRASP", "gripper", {"opening_m": 0.05, "effort_parameter_nm": 0.2}),
                    decision("RELEASE", "gripper", {"opening_m": 0.006, "effort_parameter_nm": 0.2}),
                    decision("PREGRASP", "gripper", {"opening_m": 0.006, "effort_parameter_nm": 0.2}),
                    decision("INIT", "gripper", {"opening_m": 0.006, "effort_parameter_nm": 0.2})):
            with self.assertRaises(FastPolicyError):
                self.check(bad)
        self.assertNotIn("grasp_verified", compact_controller_state(state("VERIFY_GRASP")))

    def test_visual_transitions_require_runner_owned_mechanical_evidence(self):
        cases = (("VERIFY_GRASP", "LIFT", "phase_complete", ("grasp_close_completed",)),
                 ("LIFT", "APPROACH_HOLDER", "phase_complete", ("lift_completed",)),
                 ("INSERT", "RELEASE", "phase_complete", ("insertion_move_completed",)),
                 ("VERIFY_SUCCESS", "DONE", "success", ("release_open_completed", "retreat_after_release_completed")))
        for phase, target, evidence, flags in cases:
            value = advance(phase, target, evidence)
            with self.subTest(phase=phase):
                with self.assertRaises(FastPolicyError):
                    self.check(value)
                measured = state(phase, execution_evidence={flag: True for flag in flags})
                self.check(value, measured=measured)
                self.assertNotIn("execution_evidence", compact_controller_state(measured))
                measured["execution_evidence"][flags[0]] = 1
                with self.assertRaises(FastPolicyError):
                    self.check(value, measured=measured)

    def test_explicit_failure_recovery_is_not_success(self):
        self.check(advance("INSERT", "RECOVERY", "target_lost"))
        self.check(advance("RECOVERY", "INIT"))
        for value in (advance("INSERT", "RECOVERY"), advance("DONE", "RECOVERY", "grasp_failed"),
                      advance("RECOVERY", "RELEASE")):
            with self.assertRaises(FastPolicyError):
                self.check(value)

    def test_camera_and_effort_policy_preserves_critical_views(self):
        for phase in ("ALIGN_PEN", "GRASP", "INSERT", "VERIFY_GRASP", "VERIFY_SUCCESS"):
            self.assertEqual(select_camera_views(state(phase)), ("front", "right_hand"))
        for phase in ("GRASP", "INSERT", "RELEASE", "VERIFY_GRASP", "VERIFY_SUCCESS"):
            self.assertEqual(reasoning_effort(state(phase)), "high")
        for result in ({"visual_progress": "no_progress"}, {"target_visible": False},
                       {"grasp_verified": False}, {"status": "timeout"}):
            exceptional = state("ALIGN_PEN", previous_result=result)
            self.assertTrue(requires_explanation(exceptional))
            self.assertEqual(select_camera_views(exceptional), ("front", "left_hand", "right_hand"))
        self.assertEqual(reasoning_effort(state(retry_count=2)), "xhigh")
        self.assertFalse(requires_explanation(state()))
        self.assertEqual(set(PHASES), {phase_spec(phase)["phase"] for phase in PHASES})

    def test_phase_caps_tighten_both_rotation_and_translation_without_absolute_defaults(self):
        expected = {"APPROACH_PEN": 1, "APPROACH_HOLDER": 1,
                    "ALIGN_PEN": .5, "ALIGN_HOLDER": .5, "PREGRASP": .15,
                    "INSERT": .1, "LIFT": .5, "VERIFY_GRASP": .15,
                    "VERIFY_SUCCESS": .5, "RECOVERY": .15,
                    "INIT": 0, "GRASP": 0, "RELEASE": 0, "DONE": 0}
        for phase, fraction in expected.items():
            with self.subTest(phase=phase):
                spec = phase_spec(phase)
                self.assertEqual(phase_translation_fraction(phase), fraction)
                self.assertEqual(phase_rotation_fraction(phase), fraction)
                self.assertEqual(spec["translation_fraction"], fraction)
                self.assertEqual(spec["rotation_fraction"], fraction)
                self.assertTrue(0 <= fraction <= 1)
                self.assertEqual(spec["chunk_allowed"], phase in ("APPROACH_PEN", "APPROACH_HOLDER"))
                self.assertIn("without contact", spec["chunk_policy"])
        for function in (phase_translation_fraction, phase_rotation_fraction):
            with self.assertRaises(FastPolicyError):
                function("UNKNOWN")

    def test_compact_state_removes_cv_depth_calibration_history_and_oracles(self):
        raw = state(memory="The previous proposed move was completed.")
        raw.update(red_candidates=[{"secret": "CV_LEAK"}], depth_m_path="DEPTH_LEAK", intrinsics="INTRINSICS_LEAK",
                   calibration="EXTRINSIC_LEAK", history=["HISTORY_LEAK"], task_success="ORACLE_LEAK",
                   execution_evidence={"release_open_completed": True})
        raw["robot_state"].update(ik_solution="IK_LEAK", T_flange_camera="TRANSFORM_LEAK")
        raw["gripper_state"]["classification"] = "GRASP_ORACLE_LEAK"
        raw["previous_action"] = dict(move(), detector="ACTION_LEAK")
        raw["previous_result"] = {"status": "arrived", "grasp_verified": None, "oracle": "RESULT_LEAK"}
        compact = compact_controller_state(raw)
        self.assertNotIn("LEAK", json.dumps(compact))
        self.assertNotIn("execution_evidence", compact)
        self.assertEqual(compact["previous_result"], {"status": "arrived", "grasp_verified": None})
        compact["robot_state"]["pose_m_rad"][0] = 999
        self.assertEqual(raw["robot_state"]["pose_m_rad"][0], 0.2)

    def test_action_budget_is_exact_guardrail_data_not_target_geometry(self):
        budget = {"max_translation_m": .003, "max_rotation_rad": .01,
                  "max_speed_percent": 5, "max_waypoints": 1,
                  "gripper_min_m": 0, "gripper_max_m": .07, "max_effort_parameter_nm": .2}
        projected = compact_controller_state(state("INSERT", action_budget=budget))
        self.assertEqual(projected["action_budget"], budget)
        self.assertIsNot(projected["action_budget"], budget)
        zero_motion = dict(budget, max_translation_m=0, max_rotation_rad=0)
        self.assertEqual(compact_controller_state(state("GRASP", action_budget=zero_motion))["action_budget"], zero_motion)
        for change in ({"target_xyz": [1, 2, 3]}, {"workspace_min_m": [0, 0, 0]},
                       {"max_translation_m": math.nan}, {"max_rotation_rad": math.inf},
                       {"max_translation_m": -1}, {"max_rotation_rad": -1},
                       {"max_speed_percent": True}, {"max_speed_percent": 101},
                       {"max_waypoints": 4}, {"max_waypoints": 0},
                       {"max_effort_parameter_nm": 0}, {"gripper_min_m": -.1},
                       {"gripper_min_m": .07}):
            with self.subTest(change=change), self.assertRaises(FastPolicyError):
                compact_controller_state(state("INSERT", action_budget=dict(budget, **change)))
        for invalid in (None, {}, {k: v for k, v in budget.items() if k != "gripper_min_m"}):
            with self.assertRaises(FastPolicyError):
                compact_controller_state(state("INSERT", action_budget=invalid))

    def test_bad_compact_values_are_not_coerced_and_embedded_action_extras_fail(self):
        for bad in (state(memory="x" * 241), state(retry_count=True),
                    state(previous_result={"grasp_verified": "true"}),
                    state(previous_result={"error_code": "arbitrary prose with spaces"})):
            with self.assertRaises(FastPolicyError):
                compact_controller_state(bad)
        bad = state()
        bad["robot_state"]["enabled"] = 1
        with self.assertRaises(FastPolicyError):
            compact_controller_state(bad)
        bad = state(previous_action=move())
        bad["previous_action"]["arguments"]["depth"] = 123
        with self.assertRaises(FastPolicyError):
            compact_controller_state(bad)


if __name__ == "__main__":
    unittest.main()
