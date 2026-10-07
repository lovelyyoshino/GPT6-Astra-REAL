"""Prevent ordinary pose-arrival tolerances from masquerading as microsteps."""
import math
import unittest

from robot_tools.motion_effect import measure_motion_effect


class MotionEffectTests(unittest.TestCase):
    def test_two_mm_target_without_motion_is_not_progress_inside_five_mm_arrival(self):
        before = [0.0] * 6
        result = measure_motion_effect(before, [0.002, 0, 0, 0, 0, 0], before)
        self.assertLess(result["target_error"]["position_m"], 0.005)
        self.assertEqual(result["translation"]["response"], "no_discriminable_response")
        self.assertIsNone(result["object_progress_measurement"])
        self.assertIsNone(result["task_success"])

    def test_measured_robot_response_does_not_claim_grasp_or_visual_progress(self):
        result = measure_motion_effect([0]*6, [0, 0, .004, 0, 0, 0], [0, 0, .003, 0, 0, 0])
        self.assertEqual(result["translation"]["response"], "requested_direction_observed")
        self.assertAlmostEqual(result["translation"]["along_requested_direction"], .003)
        for key in ("load_response", "visual_object_effect", "object_progress_measurement", "task_success"):
            self.assertIsNone(result[key])

    def test_wrong_axis_and_reverse_motion_are_not_forward_response(self):
        for actual, expected in (([0, .003, 0, 0, 0, 0], "transverse_response_observed"),
                                 ([-.003, 0, 0, 0, 0, 0], "opposite_direction_observed")):
            with self.subTest(expected=expected):
                result = measure_motion_effect([0]*6, [.004, 0, 0, 0, 0, 0], actual)
                self.assertEqual(result["translation"]["response"], expected)

    def test_sub_resolution_request_is_never_claimed_as_effective_step(self):
        result = measure_motion_effect([0]*6, [.0002, 0, 0, 0, 0, 0], [.0002, 0, 0, 0, 0, 0])
        self.assertEqual(result["translation"]["response"], "below_observation_resolution")

    def test_forward_component_cannot_hide_larger_lateral_motion_inside_arrival_tolerance(self):
        result = measure_motion_effect([0]*6, [.002, 0, 0, 0, 0, 0], [.0011, .003, 0, 0, 0, 0])
        self.assertLess(result["target_error"]["position_m"], .005)
        self.assertGreater(result["translation"]["along_requested_direction"], .001)
        self.assertEqual(result["translation"]["response"], "transverse_response_observed")
        self.assertTrue(result["translation"]["transverse_exceeds_comparison_band"])

    def test_angle_wrap_is_shortest_rotation(self):
        before = [0, 0, 0, 0, 0, math.pi-.01]
        after = [0, 0, 0, 0, 0, -math.pi+.01]
        result = measure_motion_effect(before, after, after)
        self.assertAlmostEqual(result["rotation"]["measured_norm"], .02)
        self.assertEqual(result["rotation"]["response"], "requested_direction_observed")
        self.assertAlmostEqual(result["target_error"]["rotation_rad"], 0)

    def test_unrequested_orientation_change_is_not_rotation_success(self):
        result = measure_motion_effect([0]*6, [.004, 0, 0, 0, 0, 0], [.004, 0, 0, .02, 0, 0])
        self.assertEqual(result["rotation"]["response"], "unrequested_response")

    def test_equivalent_euler_angles_at_singularity_are_not_motion(self):
        before = [0, 0, 0, 0, math.pi/2, 0]
        after = [0, 0, 0, math.pi/4, math.pi/2, math.pi/4]
        result = measure_motion_effect(before, after, after)
        self.assertLess(result["rotation"]["measured_norm"], 1e-12)
        self.assertEqual(result["rotation"]["response"], "below_observation_resolution")

    def test_overflowing_difference_is_rejected(self):
        with self.assertRaises(ValueError):
            measure_motion_effect([-1e308, 0, 0, 0, 0, 0], [1e308, 0, 0, 0, 0, 0], [0]*6)

    def test_invalid_input_never_produces_response(self):
        for value in ([0]*5, [float("nan")]*6, [True]*6, "000000"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                measure_motion_effect([0]*6, value, [0]*6)


if __name__ == "__main__":
    unittest.main()
