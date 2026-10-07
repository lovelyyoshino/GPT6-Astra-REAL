"""Live proposal assessment tests use only pure fake envelope builders."""
from copy import deepcopy
import unittest

from right_pick.fast_live_policy import assess_live_proposal
from right_pick.fast_policy import FastPolicyError, validate_decision, validate_phase_decision


def state(phase="APPROACH_PEN", **extra):
    value = {"phase": phase, "robot_state": {"pose_m_rad": [.2, 0, .3, 0, 1, 0],
             "joints_rad": [0] * 6, "sampled_at": 1.0, "enabled": True,
             "binding_verified": True, "moving": False, "arm_status": 0, "err_code": 0},
             "gripper_state": {"opening_m": .035, "effort": 0.0}}
    value.update(extra)
    return value


def decision(phase="APPROACH_PEN", action="move_eef", arguments=None):
    return {"phase": phase, "action": action, "confidence": .8,
            "arguments": {"pose_m_rad": [.201, 0, .3, 0, 1, 0], "speed_percent": 1,
                          "next_phase": None} if arguments is None else arguments}


class PureRobot:
    def __init__(self, envelope=None):
        self.envelope = {"topic": "/test/pos_cmd", "nonphysical": False} if envelope is None else envelope
        self.prepared = []

    def prepare_command(self, value, measured):
        self.prepared.append((value.to_dict(), deepcopy(measured)))
        return deepcopy(self.envelope)

    def execute(self, *args, **kwargs):
        raise AssertionError("assessment must never execute")

    def observe(self):
        raise AssertionError("assessment must use supplied state, not contact devices")

    def validate(self, *args, **kwargs):
        raise AssertionError("assessment must not pass through a mock/physical execution guard")


class FastLivePolicyTests(unittest.TestCase):
    def test_valid_motion_proposal_needs_no_fake_limits_and_remains_blocked(self):
        raw, measured, robot = decision(), state(), PureRobot()
        before = deepcopy(measured)
        report = assess_live_proposal(raw, measured, robot)
        self.assertTrue(report["schema_valid"])
        self.assertTrue(report["phase_valid"])
        self.assertTrue(report["command_encoding_valid"])
        self.assertFalse(report["numeric_limits_verified"])
        self.assertFalse(report["physical_execution_allowed"])
        self.assertFalse(report["action_dispatched"])
        self.assertEqual(report["control_commands_sent"], 0)
        self.assertIsNone(report["task_success"])
        self.assertEqual(len(robot.prepared), 1)
        self.assertEqual(measured, before)

    def test_no_motion_advance_is_not_execution_or_task_completion(self):
        robot = PureRobot()
        raw = decision("INIT", "advance", {"next_phase": "APPROACH_PEN", "evidence": "phase_complete"})
        report = assess_live_proposal(raw, state("INIT"), robot)
        self.assertTrue(report["schema_valid"] and report["phase_valid"])
        self.assertEqual(report["numeric_validation_status"], "not_applicable_no_motion")
        self.assertFalse(report["physical_execution_allowed"])
        self.assertFalse(report["phase_transition_applied"])
        self.assertIsNone(report["command_proposal"])
        self.assertIsNone(report["command_encoding_valid"])
        self.assertIsNone(report["task_success"])
        self.assertEqual(robot.prepared, [])

    def test_schema_and_phase_failures_are_separate_and_prevent_envelope(self):
        robot = PureRobot()
        malformed = assess_live_proposal(dict(decision(), enable=True), state(), robot)
        self.assertFalse(malformed["schema_valid"])
        wrong_phase = assess_live_proposal(decision("INSERT"), state(), robot)
        self.assertTrue(wrong_phase["schema_valid"])
        self.assertFalse(wrong_phase["phase_valid"])
        self.assertEqual(robot.prepared, [])

    def test_exception_state_still_requires_exception_schema(self):
        robot = PureRobot()
        measured = state("RECOVERY", retry_count=1)
        raw = decision("RECOVERY", "pause", {"evidence": "target_lost"})
        self.assertFalse(assess_live_proposal(raw, measured, robot)["schema_valid"])
        report = assess_live_proposal(dict(raw, explanation="Current RGB does not resolve the target."), measured, robot)
        self.assertTrue(report["schema_valid"] and report["phase_valid"])
        self.assertEqual(robot.prepared, [])

    def test_mock_or_unproven_limits_cannot_claim_numeric_verification(self):
        candidates = ({"numeric_limits_verified": True},
                      {"numeric_limits_verified": True, "limits_source": "mock", "nonphysical": True},
                      {"numeric_limits_verified": True, "limits_source": "explicit_physical", "nonphysical": True},
                      {"numeric_limits_verified": 1, "limits_source": "explicit_physical", "nonphysical": False})
        for envelope in candidates:
            with self.subTest(envelope=envelope):
                report = assess_live_proposal(decision(), state(), PureRobot(envelope))
                self.assertFalse(report["numeric_limits_verified"])
                self.assertFalse(report["physical_execution_allowed"])
        real_limits = {"numeric_limits_verified": True, "limits_source": "explicit_physical", "nonphysical": False}
        report = assess_live_proposal(decision(), state(), PureRobot(real_limits))
        self.assertTrue(report["numeric_limits_verified"])
        self.assertFalse(report["physical_execution_allowed"])

    def test_phase_only_helper_preserves_execution_context_gate(self):
        self.assertEqual(validate_phase_decision(decision(), "APPROACH_PEN", controller_state=state()).action, "move_eef")
        with self.assertRaisesRegex(FastPolicyError, "explicit motion limits"):
            validate_decision(decision(), "APPROACH_PEN", controller_state=state())
        raw = decision("INSERT", "advance", {"next_phase": "RELEASE", "evidence": "phase_complete"})
        with self.assertRaisesRegex(FastPolicyError, "mechanical evidence"):
            validate_phase_decision(raw, "INSERT", controller_state=state("INSERT"))

    def test_gripper_direction_is_semantic_without_limit_substitution(self):
        raw = decision("GRASP", "gripper", {"opening_m": .006, "effort_parameter_nm": .2})
        self.assertTrue(assess_live_proposal(raw, state("GRASP"), PureRobot())["phase_valid"])
        raw["arguments"]["opening_m"] = .05
        self.assertFalse(assess_live_proposal(raw, state("GRASP"), PureRobot())["phase_valid"])

    def test_preparation_error_is_reported_without_execute_or_message_leak(self):
        class FailingRobot(PureRobot):
            def prepare_command(self, value, measured):
                raise RuntimeError("not for log: potentially secret adapter details")
        report = assess_live_proposal(decision(), state(), FailingRobot())
        self.assertTrue(report["schema_valid"] and report["phase_valid"])
        self.assertFalse(report["numeric_limits_verified"])
        self.assertIn("command_preparation_failed:RuntimeError", report["blockers"])
        self.assertFalse(report["command_encoding_valid"])
        self.assertNotIn("potentially secret", repr(report))


if __name__ == "__main__":
    unittest.main()
