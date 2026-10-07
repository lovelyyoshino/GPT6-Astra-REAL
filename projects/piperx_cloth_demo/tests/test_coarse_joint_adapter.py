"""Offline coarse-profile transport checks. Every CAN/device is a test double."""
import math
import unittest

import test_pair_joint_adapter as fixture
from robot_tools import joint_path
from robot_tools.rgb_supervision import COARSE_JOINT_PATH_SCHEMA


class CoarseJointAdapterTests(fixture.JointFixture):
    response = fixture.RGBJointSettlingTests.response
    assert_four_frames = fixture.RGBJointSettlingTests.assert_four_frames
    assert_latched_once = fixture.RGBJointAdapterTests.assert_latched_once

    def coarse(self):
        context, _ = self.make_context()
        target = context['origin']['arms']['right']['joints_rad'][:]
        target[1] += math.radians(3)
        target[2] -= math.radians(3)
        context, target = fixture.visual_joint_context(context, target)
        geometry = context['geometry']
        geometry['schema'] = COARSE_JOINT_PATH_SCHEMA
        geometry['evidence']['far_from_target_observation'] = (
            'Synthetic RGB: both empty arms are distant from all task contacts; '
            'the complete arm and attached camera corridor is visible and clear.')
        geometry['source']['sha256'] = joint_path.evidence_sha256(geometry['evidence'])
        context['budget'] = {'max_translation_m': .035, 'max_rotation_rad': .08}
        return context, target

    def test_three_degree_centimeter_candidate_uses_four_frames_and_original_arrival(self):
        context, target = self.coarse()
        result = self.execute(context, target)
        self.assertTrue(result['ok'], result.get('errors'))
        self.assert_four_frames(result)
        plan = result['joint_path_plan']
        self.assertEqual(result['motion_profile'], 'coarse_approach')
        self.assertEqual(result['original_event']['motion_profile'], 'coarse_approach')
        self.assertGreater(plan['model_endpoint_displacement_m'], .01)
        self.assertLessEqual(plan['model_endpoint_displacement_m'], .02)
        self.assertAlmostEqual(plan['tracking_policy']['max_origin_excursion_rad'], math.radians(3)+.003)
        self.assertEqual(plan['tracking_policy']['transient_band_rad'], .025)
        self.assertGreaterEqual(result['observed_stable_duration_s'], 3.)
        self.assertGreaterEqual(result['observed_feedback_advances'], 20)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(result['after']['right']['joints_rad'],
            plan['encoded_target_joints_rad'])), .003)
        self.assertFalse(plan['hold_supported'])
        self.assertFalse(result['grasp_verified'])

    def test_same_three_degree_target_still_rejected_by_ordinary_profile(self):
        context, target = self.coarse()
        context, target = fixture.visual_joint_context(context, target)
        context['budget'] = {'max_translation_m': .020, 'max_rotation_rad': .05}
        result = self.execute(context, target)
        self.assert_latched_once(result, [])
        self.assertEqual(result['errors'][-1]['type'], 'JointPathError')

    def test_coarse_is_not_an_alignment_or_contact_adapter(self):
        context, target = self.coarse()
        for operation in ('align', 'release_retreat', 'extract_segment', 'insert_segment'):
            with self.subTest(operation=operation):
                with self.assertRaisesRegex(ValueError, 'must match'):
                    self.execute(context, target, operation=operation)
                self.assertEqual(self.ids(), [])
                self.assertEqual(self.ids('left'), [])

    def test_retained_peer_blocks_coarse_even_when_worker_empty(self):
        context, target = self.coarse()
        self.device._action.grasps['left'] = {'status': 'retained_static'}
        try:
            with self.assertRaisesRegex(RuntimeError, 'both live grippers empty'):
                self.execute(context, target)
            self.assertEqual(self.ids(), [])
        finally:
            self.device._action.grasps['left'] = None

    def test_coarse_cannot_bind_a_hold_bridge(self):
        context, target = self.coarse()
        with self.assertRaisesRegex(ValueError, 'cannot bind'):
            self.execute(context, target, fixture.Bridge(self, cancel=True))
        self.assertEqual(self.ids(), [])

    def test_partial_coarse_target_latches_without_retry_or_cache(self):
        context, target = self.coarse()
        self.robots['right'].partial = True
        result = self.execute(context, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156])
        self.assertIsNone(self.device.joint_binding('right')['cached_target'])
        self.assertIsNone(result['original_event'])
        self.assertEqual(result['tracking_observation']['cumulative_outside_nominal_band_s'], 0.)
        with self.assertRaises(RuntimeError):
            self.execute(context, target)
        self.assertEqual(len(self.ids()), 3)

    def test_short_transient_keeps_original_one_second_and_final_stability(self):
        context, target = self.coarse()
        self.response(lambda dt, state: state['joints_rad'].__setitem__(4,
            state['joints_rad'][4]+(.008 if dt < .4 else 0)))
        result = self.execute(context, target)
        self.assertTrue(result['ok'], result.get('errors'))
        self.assert_four_frames(result)
        self.assertGreater(result['tracking_observation']['cumulative_outside_nominal_band_s'], .3)
        self.assertLess(result['tracking_observation']['cumulative_outside_nominal_band_s'], 1.)
        self.assertGreaterEqual(result['observed_stable_duration_s'], 3.)

    def test_persistent_deviation_does_not_inherit_a_larger_time_budget(self):
        context, target = self.coarse()
        self.response(lambda dt, state: state['joints_rad'].__setitem__(4, state['joints_rad'][4]+.008))
        result = self.execute(context, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156, 0x157])
        self.assertIn('cumulative', result['errors'][-1]['detail'])
        self.assertGreater(result['tracking_observation']['cumulative_outside_nominal_band_s'], 1.)

    def test_raw_pose_guard_still_independent_from_model(self):
        context, target = self.coarse()
        self.response(lambda dt, state: state['pose_m_rad'].__setitem__(0, state['pose_m_rad'][0]+.0351))
        result = self.execute(context, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(result['tracking_observation']['first_failure']['code'], 'controller_relative_pose_envelope')

    def test_stale_feedback_stops_even_inside_coarse_target(self):
        context, target = self.coarse()
        self.response(lambda dt, state: state['fragment_timestamps_s'].update(joint_56=self.clock.time()-.051))
        result = self.execute(context, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(result['tracking_observation']['first_failure']['code'], 'stale_feedback')

    def test_cancel_after_full_send_latches_without_replacement_hold(self):
        context, target = self.coarse()
        def guard():
            if len(self.ids()) == 4:
                self.guard_error = 'explicit client cancellation'
        self.guard_hook = guard
        result = self.execute(context, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(result['original_event']['motion_profile'], 'coarse_approach')
        self.assertIsNone(result['hold_receipt'])


if __name__ == '__main__':
    unittest.main()
