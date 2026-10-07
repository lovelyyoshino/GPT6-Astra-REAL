"""Whole offline grasp checks; no socket or actuator access."""
import importlib.util
from pathlib import Path
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation
from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics

PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'pick_trajectory.py'
SPEC = importlib.util.spec_from_file_location('pick_trajectory_test_module', PATH)
planner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(planner)
Q = [36562, 281, -329, -1784, 21222, -9858]
POSE = [43201, 30764, 181596, -145000, 70132, -107442]


def geometry():
    pose = np.asarray(C_PiperForwardKinematics(1).CalFK(np.deg2rad(np.asarray(Q)/1000).tolist())[-1])
    tcp = pose[:3] + Rotation.from_euler('xyz', pose[3:], degrees=True).apply([0, 0, 135.8])
    return dict(tcp_base_mm=tcp.tolist(), cube_grasp_base_mm=[387.586, 276.111, -20.045],
                table_base_z_mm=-40.045, uncertainty=dict(depth_mm=3, finger_feature_mm=12),
                place_candidates=[dict(tcp_base_mm=[438.140, 214.109, -20.045], empty_patch_radius_mm=25)])


class WholePickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with mock.patch('socket.socket', side_effect=AssertionError('hardware access')):
            cls.plan = planner.plan_pick(Q, POSE, geometry())

    def test_complete_capture_grip_place_order_and_finite_budget(self):
        steps = self.plan['stages']
        events = [(s['kind'], s['label']) for s in steps if s['kind'] != 'move']
        self.assertEqual(events, [('capture', 'pregrasp'), ('gripper', 'close'),
                                 ('capture', 'lifted'), ('gripper', 'release'), ('capture', 'placed')])
        self.assertEqual([s['width_raw'] for s in steps if s['kind'] == 'gripper'], [0, 55000])
        self.assertTrue(all(s['effort_raw'] == 300 for s in steps if s['kind'] == 'gripper'))
        self.assertLessEqual(sum(s['kind'] == 'move' for s in steps), 120)
        self.assertGreaterEqual(self.plan['assessment']['lift_mm'], 40)
        self.assertGreaterEqual(self.plan['assessment']['carry_mm'], 50)

    def test_every_target_nominal_and_each_segment_has_arrival_error_margin(self):
        q0, p0 = np.asarray(Q)/1000., np.asarray(POSE)/1000.
        fk = C_PiperForwardKinematics(1)
        for stage in self.plan['stages']:
            if stage['kind'] != 'move':
                continue
            q, p = np.asarray(stage['joints_raw'])/1000., np.asarray(stage['pose_raw'])/1000.
            self.assertTrue(np.all(q >= planner.LIMITS_DEG[:, 0]))
            self.assertTrue(np.all(q <= planner.LIMITS_DEG[:, 1]))
            self.assertLessEqual(float(np.max(np.abs(q-q0))), 7.501)
            self.assertLessEqual(float(np.linalg.norm(p[:3]-p0[:3])), 14.01)
            self.assertLessEqual(planner._angle(planner._rotation(p), planner._rotation(p0)), 4.51)
            computed = np.asarray(fk.CalFK(np.deg2rad(q).tolist())[-1])
            self.assertLess(float(np.linalg.norm(computed[:3]-p[:3])), .01)
            self.assertLess(planner._angle(planner._rotation(computed), planner._rotation(p)), .01)
            q0, p0 = q, p

    def test_clearance_includes_uncertainty_and_observed_tracking_not_just_endpoints(self):
        self.assertEqual(self.plan['limits']['tracking_allowance_deg'], .5)
        self.assertEqual(self.plan['limits']['uncertainty_reserve_mm'], 15)
        self.assertGreater(self.plan['assessment']['checked_states'], 1000)
        self.assertGreaterEqual(self.plan['assessment']['minimum_sampled_clearance_mm'], 10)
        self.assertTrue(self.plan['assessment']['may_miss_above_cube'])
        self.assertGreater(self.plan['assessment']['grasp_tcp_table_height_mm'], 30)
        for stage in self.plan['stages']:
            if stage['kind'] == 'move':
                self.assertGreaterEqual(stage['clearance_mm'], 10)
                actual = planner.clearance_for_joints(stage['joints_raw'], geometry())
                self.assertGreaterEqual(actual['minimum_clearance_mm'], 10)

    def test_wrong_pose_or_scene_reference_rejects_before_a_plan_is_returned(self):
        wrong = list(POSE); wrong[0] += 2000
        with self.assertRaisesRegex(planner.PlanRejected, 'FK disagree'):
            planner.plan_pick(Q, wrong, geometry())
        bad = geometry(); bad['tcp_base_mm'][0] += 10
        with self.assertRaisesRegex(planner.PlanRejected, 'this robot pose'):
            planner.plan_pick(Q, POSE, bad)

    def test_table_contact_and_uncertainty_fail_clearance_check(self):
        bad = geometry(); bad['table_base_z_mm'] = 180
        self.assertLess(planner.clearance_for_joints(Q, bad)['minimum_clearance_mm'], 0)
        bad['uncertainty']['finger_feature_mm'] = 40
        with self.assertRaisesRegex(planner.PlanRejected, 'uncertainty'):
            planner.clearance_for_joints(Q, bad)

    def test_missing_placement_and_invalid_inputs_cannot_form_partial_action_plan(self):
        bad = geometry(); bad['place_candidates'] = []
        with self.assertRaisesRegex(planner.PlanRejected, 'placement'):
            planner.plan_pick(Q, POSE, bad)
        for q in (Q[:5], [True]+Q[1:], [1.5]+Q[1:]):
            with self.assertRaisesRegex(planner.PlanRejected, 'six integer'):
                planner.plan_pick(q, POSE, geometry())


if __name__ == '__main__':
    unittest.main()
