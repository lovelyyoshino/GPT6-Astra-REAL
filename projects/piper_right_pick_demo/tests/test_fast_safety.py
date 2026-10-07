"""Pure numerical/lifecycle checks; no hardware, sockets or model requests."""
import copy
import math
import unittest

from right_pick.fast_replay import MockRobot, mock_limits
from right_pick.fast_observation import HistoricalRGBSource
from right_pick.fast_safety import (FastSafetyError, FastSafetyGuard, MockMotionGuard,
                                   physical_blockers, require_nonphysical_runtime,
                                   rotation_distance)


def state():
    return dict(pose_m_rad=[.25, 0, .25, 0, 0, 0], joints_rad=[0.]*6,
                sampled_at=100., enabled=True, moving=False, binding_verified=True,
                arm_status=0, err_code=0, opening_m=.04)


def move(pose=None, phase='APPROACH_PEN', speed=3):
    return dict(phase=phase, action='move_eef', confidence=.9,
                arguments=dict(pose_m_rad=pose or [.26, 0, .25, 0, 0, 0],
                               speed_percent=speed, next_phase=None))


def chunk(points, phase='APPROACH_PEN'):
    return dict(phase=phase, action='move_eef_chunk', confidence=.9,
                arguments=dict(waypoints=points, speed_percent=3, next_phase=None))


class FastSafetyTests(unittest.TestCase):
    def setUp(self):
        self.limits = mock_limits()
        self.guard = MockMotionGuard(self.limits, nonphysical=True, clock=lambda: 100.)
        self.state = state()

    def test_physical_flags_cannot_qualify_live_dispatch(self):
        config = dict(allow_motion=True, hold_verified=True, physical_dispatch_commissioned=True,
                      bindings_verified=True, limits=self.limits)
        self.assertTrue(any('hold_unqualified' in x for x in physical_blockers(config)))
        codes = {reason.split(':', 1)[0] for reason in physical_blockers(config)}
        self.assertIn('live_adapter_uncommissioned', codes)
        self.assertIn('physical_binding_scope_unqualified', codes)
        self.assertIn('physical_limits_unqualified', codes)
        self.assertNotIn('live_adapter_missing', codes)
        self.assertNotIn('live_bindings_unverified', codes)
        with self.assertRaisesRegex(FastSafetyError, 'Physical execution unavailable'):
            FastSafetyGuard(config).validate(move(), self.state)

    def test_exact_nonphysical_runtime_only(self):
        robot = MockRobot()
        # Construction only, no file is accessed or capture requested.
        cameras = HistoricalRGBSource(['unused-observation.json'], 'unused-output')
        require_nonphysical_runtime(robot, cameras)
        class Pretender:
            nonphysical = True
        class Subclass(MockRobot):
            pass
        for bad_robot, bad_camera in ((Pretender(), cameras), (Subclass(), cameras), (robot, Pretender())):
            with self.assertRaises(FastSafetyError):
                require_nonphysical_runtime(bad_robot, bad_camera)
        robot.nonphysical = False
        with self.assertRaises(FastSafetyError):
            require_nonphysical_runtime(robot, cameras)

    def test_limits_and_nonphysical_opt_in_are_explicit(self):
        with self.assertRaises(TypeError):
            MockMotionGuard(self.limits)
        for opt in (False, 1, 'true', None):
            with self.assertRaises(FastSafetyError):
                MockMotionGuard(self.limits, nonphysical=opt)
        for limits in (None, {}, [], dict(self.limits, hold_verified=True)):
            with self.assertRaises(FastSafetyError):
                MockMotionGuard(limits, nonphysical=True)

    def test_invalid_numeric_limits_rejected(self):
        for key, value in (('max_state_age_s', None), ('max_speed_percent', True),
                           ('max_waypoints', 0), ('max_rotation_step_rad', float('nan')),
                           ('joint_limits_rad', [[0, 0]] * 6), ('workspace_max_m', [-1, -1, -1])):
            limits = dict(self.limits, **{key: value})
            with self.subTest(key=key), self.assertRaises(FastSafetyError):
                MockMotionGuard(limits, nonphysical=True)

    def test_valid_mock_action_never_claims_physical_permission(self):
        result = self.guard.validate(move(), self.state)
        self.assertTrue(result['nonphysical'])
        self.assertFalse(result['physical_motion_authorized'])
        self.assertFalse(result['hold_verified'])
        self.assertFalse(result['path_or_ik_verified'])
        self.assertAlmostEqual(result['cumulative_translation_m'], .01)

    def test_nested_runner_state_is_accepted(self):
        flat = self.state.copy()
        flat.pop('opening_m')
        wrapped = {'robot_state': flat, 'gripper_state': {'opening_m': .04}, 'nonphysical': True}
        self.guard.validate(move(), wrapped)

    def test_feedback_freshness_and_health_required_before_any_ticket(self):
        for key, value in (('sampled_at', 99.), ('sampled_at', 100.001), ('enabled', False),
                           ('binding_verified', 1), ('moving', True), ('arm_status', 4),
                           ('err_code', False), ('opening_m', None), ('joints_rad', [4.]*6)):
            bad = dict(self.state, **{key: value})
            with self.subTest(key=key, value=value), self.assertRaises(FastSafetyError):
                self.guard.begin(move(), bad)
        self.assertEqual(self.guard.actions_started, 0)

    def test_workspace_checks_current_and_each_target(self):
        bad = copy.deepcopy(self.state)
        bad['pose_m_rad'][2] = .01
        with self.assertRaisesRegex(FastSafetyError, 'workspace'):
            self.guard.validate(move(), bad)
        with self.assertRaisesRegex(FastSafetyError, 'workspace'):
            self.guard.validate(move([.25, 0, .7, 0, 0, 0]), self.state)

    def test_speed_bounds_use_percent_not_invented_physical_velocity(self):
        self.guard.validate(move(speed=20), self.state)
        with self.assertRaisesRegex(FastSafetyError, 'speed_percent'):
            self.guard.validate(move(speed=21), self.state)

    def test_step_and_cumulative_translation_both_enforced(self):
        with self.assertRaisesRegex(FastSafetyError, 'Translation step'):
            self.guard.validate(move([.281, 0, .25, 0, 0, 0]), self.state)
        # End equals start; out-and-back path exceeds the whole-action budget.
        points = [[.27, 0, .25, 0, 0, 0], [.25, 0, .25, 0, 0, 0]]
        with self.assertRaisesRegex(FastSafetyError, 'Cumulative chunk'):
            self.guard.validate(chunk(points), self.state)

    def test_later_invalid_waypoint_rejects_whole_action_before_begin(self):
        points = [[.26, 0, .25, 0, 0, 0], [.8, 0, .25, 0, 0, 0]]
        with self.assertRaises(FastSafetyError):
            self.guard.begin(chunk(points), self.state)
        self.assertEqual(self.guard.actions_started, 0)

    def test_chunk_cannot_multiply_single_step_limit(self):
        limits = dict(self.limits, max_chunk_translation_m=.3)
        guard = MockMotionGuard(limits, nonphysical=True, clock=lambda: 100.)
        points = [[.27, 0, .25, 0, 0, 0], [.29, 0, .25, 0, 0, 0]]
        with self.assertRaisesRegex(FastSafetyError, 'Cumulative chunk'):
            guard.validate(chunk(points), self.state)

    def test_waypoints_have_hard_cap_three_even_with_large_configuration(self):
        limits = dict(self.limits, max_waypoints=10)
        guard = MockMotionGuard(limits, nonphysical=True, clock=lambda: 100.)
        with self.assertRaisesRegex(FastSafetyError, 'Too many|waypoints'):
            guard.validate(chunk([self.state['pose_m_rad']]*4), self.state)

    def test_rotation_uses_so3_and_cumulative_path(self):
        wrapped = [0., 0., 0., 0., 0., 2*math.pi]
        self.assertAlmostEqual(rotation_distance([0.]*6, wrapped), 0., places=12)
        composed = [.25, 0., .25, .08, .08, 0.]
        with self.assertRaisesRegex(FastSafetyError, 'Rotation step'):
            self.guard.validate(move(composed), self.state)
        points = [[.25, 0, .25, 0, 0, .08], [.25, 0, .25, 0, 0, 0]]
        with self.assertRaisesRegex(FastSafetyError, 'Cumulative chunk'):
            self.guard.validate(chunk(points), self.state)

    def test_phase_limits_reduce_both_translation_and_rotation(self):
        for phase, fraction in (('ALIGN_PEN', .5), ('PREGRASP', .15), ('INSERT', .1),
                                ('VERIFY_GRASP', .15), ('LIFT', .5), ('RECOVERY', .15)):
            with self.subTest(phase=phase):
                allowed = [.25+.03*fraction*.99, 0, .25, 0, 0, .1*fraction*.99]
                self.guard.validate(move(allowed, phase), self.state)
                too_far = allowed.copy()
                too_far[0] = .25+.03*fraction*1.01
                with self.assertRaisesRegex(FastSafetyError, 'Translation step'):
                    self.guard.validate(move(too_far, phase), self.state)
                too_rotated = allowed.copy()
                too_rotated[5] = .1*fraction*1.01
                with self.assertRaisesRegex(FastSafetyError, 'Rotation step'):
                    self.guard.validate(move(too_rotated, phase), self.state)

    def test_gripper_mapping_is_explicit_and_not_force_calibration(self):
        decision = dict(phase='GRASP', action='gripper', confidence=.9,
                        arguments=dict(opening_m=.005, effort_parameter_nm=.2))
        self.guard.validate(decision, self.state)
        for key, value in (('opening_m', .08), ('effort_parameter_nm', 1.01)):
            bad = copy.deepcopy(decision)
            bad['arguments'][key] = value
            with self.assertRaises(FastSafetyError):
                self.guard.validate(bad, self.state)

    def test_ticket_serializes_and_cannot_be_finished_twice(self):
        ticket = self.guard.begin(move(), self.state)
        with self.assertRaisesRegex(FastSafetyError, 'already active'):
            self.guard.begin(move(), self.state)
        result = self.guard.finish(ticket, self.state)
        self.assertFalse(result['physical_arrival_verified'])
        with self.assertRaisesRegex(FastSafetyError, 'already consumed'):
            self.guard.finish(ticket, self.state)
        with self.assertRaisesRegex(FastSafetyError, 'Failure latched'):
            self.guard.begin(move(), self.state)

    def test_send_failure_latch_is_permanent_and_consumes_ticket(self):
        ticket = self.guard.begin(move(), self.state)
        self.guard.latch_failure('partial_send')
        self.guard.latch_failure('timeout')
        self.assertEqual(self.guard.failure, 'partial_send')
        for call in (lambda: self.guard.finish(ticket, self.state),
                     lambda: self.guard.validate(move(), self.state),
                     lambda: self.guard.begin(move(), self.state)):
            with self.assertRaisesRegex(FastSafetyError, 'Failure latched'):
                call()
        self.assertEqual(self.guard.actions_started, 1)

    def test_post_action_feedback_failure_latches(self):
        ticket = self.guard.begin(move(), self.state)
        bad = dict(self.state, joints_rad=[4.]*6)
        with self.assertRaises(FastSafetyError):
            self.guard.finish(ticket, bad)
        with self.assertRaisesRegex(FastSafetyError, 'Failure latched'):
            self.guard.begin(move(), self.state)


if __name__ == '__main__':
    unittest.main()
