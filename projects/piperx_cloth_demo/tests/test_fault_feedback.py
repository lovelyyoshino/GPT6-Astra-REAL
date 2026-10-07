"""Pure feedback diagnosis; socket creation is forbidden throughout."""
import copy
import json
import unittest
from unittest.mock import patch

from robot_tools.fault_feedback import FaultFeedback
from test_execution import healthy_arm


class FaultFeedbackTests(unittest.TestCase):
    def setUp(self):
        guard = patch("socket.socket", side_effect=AssertionError("No physical sockets"))
        guard.start()
        self.addCleanup(guard.stop)
        self.now = 1800000000.0
        self.states = {side: healthy_arm(self.now) for side in ("left", "right")}
        self.tracker = FaultFeedback()

    def fragment(self, result, name="joint_12", side="left"):
        return result["diagnostics"][side]["fragments"][name]

    def test_advancement_is_independent_per_fragment_and_never_a_hold(self):
        original = copy.deepcopy(self.states)
        first = self.tracker.capture(self.states, self.now)
        self.assertEqual(self.fragment(first)["progress"], "first_observation")
        self.assertEqual(self.states, original)
        self.states["left"]["fragment_timestamps_s"]["joint_12"] += .02
        second = self.tracker.capture(self.states, self.now + .02)
        self.assertEqual(self.fragment(second)["progress"], "advanced")
        self.assertEqual(self.fragment(second, "joint_34")["progress"], "repeated")
        self.assertIsNone(second["stationary_observed"])
        self.assertIsNone(second["physical_stop_verified"])
        self.assertFalse(second["motion_permitted"])
        self.assertEqual(second["hardware_commands_sent"], 0)

    def test_missing_fragment_never_resets_its_prior_high_watermark(self):
        self.tracker.seed(self.states, self.now)
        del self.states["left"]["fragment_timestamps_s"]["joint_12"]
        missing = self.tracker.capture(self.states, self.now + .01)
        self.assertEqual(self.fragment(missing)["progress"], "missing")
        self.states["left"]["fragment_timestamps_s"]["joint_12"] = self.now - .02
        regressed = self.tracker.capture(self.states, self.now + .02)
        self.assertEqual(self.fragment(regressed)["progress"], "regressed")
        self.assertFalse(self.fragment(regressed)["fresh"])
        self.assertEqual(regressed["arms"]["left"]["fragment_timestamps_s"]["joint_12"], self.now - .02)

    def test_stale_future_invalid_clock_and_nan_remain_explicit(self):
        self.tracker.seed(self.states, self.now)
        self.states["left"]["fragment_timestamps_s"]["joint_12"] = self.now - .2
        self.states["left"]["fragment_timestamps_s"]["joint_34"] = self.now + .1
        self.states["left"]["fragment_timestamps_s"]["joint_56"] = float("nan")
        report = self.tracker.capture(self.states, self.now)
        self.assertIn("stale", self.fragment(report)["issues"])
        self.assertIn("future_timestamp", self.fragment(report, "joint_34")["issues"])
        self.assertEqual(self.fragment(report, "joint_56")["timestamp_s"], {"invalid_numeric": "nan"})
        self.assertFalse(report["within_fragment_skew"])
        invalid = self.tracker.capture(self.states, float("nan"))
        self.assertFalse(invalid["observation_clock_valid"])
        self.assertFalse(self.fragment(invalid)["fresh"])
        json.dumps(invalid, allow_nan=False)

    def test_observation_clock_rollback_does_not_become_a_new_time_origin(self):
        self.tracker.seed(self.states, self.now)
        for offset in (-.02, -.01):
            report = self.tracker.capture(self.states, self.now + offset)
            self.assertTrue(report["observation_clock_regressed"])
            self.assertIn("observation_clock_regressed", self.fragment(report)["issues"])

    def test_disabled_and_faulted_feedback_is_retained(self):
        self.states["left"]["drivers"]["1"]["foc_status"]["driver_enable_status"] = False
        self.states["left"]["drivers"]["1"]["foc_status"]["collision_status"] = True
        self.states["left"]["gripper"]["foc_status"]["driver_enable_status"] = False
        self.states["left"]["arm_status"]["arm_status"] = 1
        report = self.tracker.capture(self.states, self.now)
        self.assertEqual(report["arms"], self.states)
        self.assertFalse(report["diagnostics"]["left"]["health"]["healthy"])
        self.assertTrue(report["diagnostics"]["right"]["health"]["healthy"])

    def test_one_failed_reader_does_not_erase_peer_feedback(self):
        self.states["left"] = None
        report = self.tracker.capture(self.states, self.now, {"left": "decoder failure"})
        self.assertEqual(report["status"], "partial")
        self.assertEqual(report["read_errors"], {"left": "decoder failure"})
        self.assertIsNone(report["arms"]["left"])
        self.assertEqual(report["arms"]["right"], self.states["right"])
        self.assertEqual(self.fragment(report)["progress"], "missing")

    def test_no_readable_arm_is_explicit_failure_with_no_stop_verdict(self):
        result = self.tracker.capture({"left": None, "right": None}, self.now,
                                      {"left": "decoder", "right": "decoder"})
        self.assertEqual(result["status"], "read_error")
        self.assertIsNone(result["physical_stop_verified"])
        self.assertFalse(result["within_fragment_skew"])


if __name__ == "__main__":
    unittest.main()
