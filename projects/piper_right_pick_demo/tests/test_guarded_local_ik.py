"""Offline manufacturer-FK regression; all sockets and subprocesses forbidden."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location("guarded_local_ik_test", ROOT / "scripts/guarded_local_ik.py")
planner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(planner)


class GuardedLocalIKTests(unittest.TestCase):
    def setUp(self):
        for target in ("socket.socket", "subprocess.Popen"):
            guard = patch(target, side_effect=AssertionError("No hardware/network/process allowed"))
            guard.start()
            self.addCleanup(guard.stop)
        # Old Python's platform.system() shells out to uname during python-can
        # import. Supply host identity without permitting that subprocess.
        with patch("platform.system", return_value="Linux"):
            self.fk = planner.support.manufacturer_fk()
        self.raw = [20000, 45000, -65000, 10000, 25000, 5000]
        self.q = np.asarray(self.raw)*planner.support.RAD_PER_RAW
        self.pose = self.fk(self.q.tolist())
        self.limits = {"max_translation_step_m": .03, "max_rotation_step_rad": .05,
                       "max_state_age_s": .1, "workspace_min_m": [-.6, -.6, .05],
                       "workspace_max_m": [.6, .6, .65], "joint_limits_rad": [
                           [lo*planner.support.RAD_PER_RAW, hi*planner.support.RAD_PER_RAW]
                           for lo, hi in planner.support.JOINT_LIMITS_RAW]}

    def solve(self, xyz=(0., 0., 0.), rotation=(0., 0., 0.), **kwargs):
        return planner.solve_local_delta(self.raw, self.pose, list(xyz), list(rotation),
                                          kwargs.pop("limits", self.limits), **kwargs)

    def check_success(self, result):
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["fresh_feedback_verified"])
        self.assertFalse(result["collision_verified"])
        self.assertLessEqual(result["endpoint_position_error_m"], .0005)
        self.assertLessEqual(result["endpoint_rotation_error_rad"], .005)
        self.assertLessEqual(max(abs(a-b) for a, b in zip(self.raw, result["target_raw"])), 2000)
        self.assertEqual(result["path_check"]["target_raw"], result["target_raw"])
        self.assertFalse(result["path_check"]["collision_verified"])

    def test_each_base_translation_axis_uses_single_local_branch_and_real_fk(self):
        for axis in range(3):
            delta = [0., 0., 0.]
            delta[axis] = .0001
            with self.subTest(axis=axis), patch.object(planner, "least_squares", wraps=planner.least_squares) as solve:
                result = self.solve(xyz=delta)
            self.check_success(result)
            self.assertEqual(solve.call_count, 1)
            self.assertEqual(result["solver_calls"], 1)
            self.assertGreaterEqual(result["translation_progress_fraction"], .5)
            self.assertTrue(np.array_equal(solve.call_args.args[1], self.q))
            self.assertAlmostEqual(result["requested_pose_m_rad"][axis]-self.pose[axis], .0001)
            lower, upper = solve.call_args.kwargs["bounds"]
            self.assertTrue(np.all(lower >= self.q-math.radians(2)-1e-12))
            self.assertTrue(np.all(upper <= self.q+math.radians(2)+1e-12))

    def test_so3_delta_is_base_frame_left_composition_each_axis(self):
        start = Rotation.from_euler("xyz", self.pose[3:])
        for axis in range(3):
            delta = [0., 0., 0.]
            delta[axis] = .001
            with self.subTest(axis=axis):
                result = self.solve(rotation=delta)
                self.check_success(result)
                expected = Rotation.from_rotvec(delta)*start
                received = Rotation.from_euler("xyz", result["requested_pose_m_rad"][3:])
                self.assertLess(np.linalg.norm((expected.inv()*received).as_rotvec()), 1e-12)
                self.assertGreaterEqual(result["rotation_progress_fraction"], .5)

    def test_strict_nominal_current_bounds_and_narrower_site_bounds(self):
        for axis, pair in enumerate(planner.support.JOINT_LIMITS_RAW):
            for bad in (pair[0]-1, pair[1]+1):
                raw = list(self.raw)
                raw[axis] = bad
                result = planner.solve_local_delta(raw, self.pose, [0]*3, [0]*3, self.limits)
                self.assertFalse(result["ok"])
                self.assertEqual(result["solver_calls"], 0)
                self.assertIn("strict manufacturer", result["error"])
        limits = copy.deepcopy(self.limits)
        limits["joint_limits_rad"][0][1] = float(self.q[0]-.001)
        result = self.solve(limits=limits)
        self.assertFalse(result["ok"])
        self.assertIn("outside site", result["error"])

    def test_small_cap_unreachable_request_fails_without_retry_or_smaller_goal(self):
        with patch.object(planner, "least_squares", wraps=planner.least_squares) as solve:
            result = self.solve(xyz=[0, 0, .003], max_joint_step_deg=.001)
        self.assertFalse(result["ok"])
        self.assertEqual(solve.call_count, 1)
        self.assertEqual(result["delta_xyz_m"], [0., 0., .003])
        self.assertEqual(result["stage"], "quantized_endpoint")
        self.assertNotIn("target_raw", result)

    def test_existing_complete_joint_box_rejection_is_not_relaxed(self):
        result = self.solve(xyz=[.001, 0, 0])
        self.assertFalse(result["ok"])
        self.assertEqual(result["stage"], "original_path_check")
        self.assertIn("Complete joint box", result["error"])
        self.assertLessEqual(result["endpoint_position_error_m"], .0005)
        self.assertNotIn("target_raw", result)

    def test_nonzero_small_request_cannot_pass_as_unchanged_joint_or_endpoint(self):
        fake = types.SimpleNamespace(success=True, x=self.q.copy(), nfev=1)
        for xyz, rotation in (([.0001, 0, 0], [0]*3), ([0]*3, [.001, 0, 0])):
            with self.subTest(xyz=xyz, rotation=rotation), patch.object(planner, "least_squares", return_value=fake):
                result = self.solve(xyz=xyz, rotation=rotation)
            self.assertFalse(result["ok"])
            self.assertIn("insufficient physical", result["error"])
            self.assertNotIn("target_raw", result)

    def test_zero_delta_never_solves_or_corrects_feedback_with_motion(self):
        with patch.object(planner, "least_squares", side_effect=AssertionError("No optimization for zero delta")):
            result = self.solve()
        self.check_success(result)
        self.assertEqual(result["status"], "zero_delta_no_command")
        self.assertFalse(result["should_send"])
        self.assertEqual(result["target_raw"], self.raw)
        self.assertEqual(result["solver_calls"], 0)
        self.pose[0] += .001
        result = self.solve()
        self.assertFalse(result["ok"])
        self.assertIn("residual", result["error"])

    def test_invalid_inputs_limits_and_large_requests_fail_before_solver(self):
        for name, larger in (("max_translation_step_m", .030001), ("max_rotation_step_rad", .050001),
                             ("max_state_age_s", .100001)):
            limits = dict(self.limits, **{name: larger})
            result = self.solve(limits=limits)
            self.assertFalse(result["ok"])
            self.assertEqual(result["solver_calls"], 0)
        for kwargs in ({"max_joint_step_deg": 2.001}, {"max_joint_step_deg": True},
                       {"xyz": [float("nan"), 0, 0]}, {"xyz": [.031, 0, 0]},
                       {"rotation": [0, 0, .051]}):
            with self.subTest(kwargs=kwargs):
                result = self.solve(**kwargs)
                self.assertFalse(result["ok"])
                self.assertEqual(result["solver_calls"], 0)
        raw = [True]+self.raw[1:]
        self.assertFalse(planner.solve_local_delta(raw, self.pose, [0]*3, [0]*3, self.limits)["ok"])

    def test_quantized_output_rechecked_even_when_solver_reports_success(self):
        for offset in (.001, .04):
            target = self.q.copy()
            target[0] += offset
            fake = types.SimpleNamespace(success=True, x=target, nfev=1)
            with patch.object(planner, "least_squares", return_value=fake):
                result = self.solve(xyz=[0, 0, .003])
            self.assertFalse(result["ok"])
            self.assertEqual(result["stage"], "quantized_endpoint")
            self.assertNotIn("target_raw", result)

    def test_initial_fk_mismatch_is_a_failure_without_fitting_away_bad_state(self):
        self.pose[0] += .0021
        result = self.solve(xyz=[.0003, 0, 0])
        self.assertFalse(result["ok"])
        self.assertEqual(result["stage"], "initial_fk")
        self.assertEqual(result["solver_calls"], 0)

    def test_orientation_euler_wrap_does_not_change_physical_request(self):
        normal = self.solve(rotation=[0, 0, .001])
        self.pose[3] += 2*math.pi
        wrapped = self.solve(rotation=[0, 0, .001])
        self.check_success(normal)
        self.check_success(wrapped)
        self.assertEqual(normal["target_raw"], wrapped["target_raw"])

    def test_exact_nominal_zero_boundary_is_valid_and_results_are_strict_json(self):
        raw = [0]*6
        pose = self.fk([0.]*6)
        result = planner.solve_local_delta(raw, pose, [0.]*3, [0.]*3, self.limits)
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["target_raw"], raw)
        self.assertEqual(result["status"], "zero_delta_no_command")
        json.dumps(result, allow_nan=False)
        json.dumps(self.solve(xyz=[.001, 0, 0]), allow_nan=False)


if __name__ == "__main__":
    unittest.main()
