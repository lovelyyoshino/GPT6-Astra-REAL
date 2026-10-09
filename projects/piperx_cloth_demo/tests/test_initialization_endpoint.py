"""Reserve accepted J4 variation by choosing a bounded target, never filtering RX."""
import copy
import math
import unittest
from unittest.mock import patch

from robot_tools import joint_initialization as ji
from robot_tools.joint_path import evidence_sha256
from robot_tools.model_compatibility import fk_matrix, matrix_error
import test_joint_initialization as fixture
import test_feedback_tolerance as native

# Original zero-TX event right-init-after-schedule-repair, 2026-10-08.
RIGHT = [0.7027568233155168, -0.02879793265790644, 0.04584979944989104,
         0., 0.40257764526501205, -0.1100953692158023]
LEFT = [-0.18503980729643882, 2.1569651560771925, -1.5019954476812802,
        -0.13857914260834978, 0.9365611299126771, 0.009197885158010117]


def context():
    ctx = fixture.visual_context()
    ctx['feedback_observation'] = native.POLICY
    for label in ('origin','current'):
        for side, joints in (('left',LEFT),('right',RIGHT)):
            ctx[label]['arms'][side]['joints_rad'] = joints[:]
    fixture.commit_origin(ctx)
    return ctx


class EndpointSelectionTests(unittest.TestCase):
    def setUp(self):
        guard=patch('socket.socket',side_effect=AssertionError('Offline only'))
        guard.start();self.addCleanup(guard.stop)

    def test_recorded_j4_failure_fits_a_different_target_without_changing_limits(self):
        ctx=context();original=copy.deepcopy(ctx)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertEqual(ctx,original)
        nearest=RIGHT[:];nearest[1:3]=[0.,0.]
        moved=RIGHT[:];moved[3]=math.radians(.4)
        old_distance=matrix_error(fk_matrix(plan['model']['mdh'],moved),fk_matrix(plan['model']['mdh'],nearest))['position_error_m']
        self.assertGreater(old_distance,.015)
        self.assertLessEqual(plan['target_selection']['chosen_endpoint_upper_m'],.015)
        self.assertAlmostEqual(plan['encoded_target_joints_rad'][1],.01,places=5)
        self.assertEqual(plan['encoded_target_joints_rad'][2],0.)
        self.assertEqual(plan['startup_limits']['target_displacement_m'],.015)
        for degrees in (-.499,0.,.4,.499):
            sample=fixture.fresh_sample(ctx)
            sample['arms']['right']['joints_rad'][3]=math.radians(degrees)
            raw=copy.deepcopy(sample)
            ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase='pre_dispatch')
            self.assertEqual(sample,raw)
        sample['arms']['right']['joints_rad'][3]=math.radians(.501)
        with self.assertRaises(ji.JointInitializationError):
            ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase='pre_dispatch')

    def test_genuine_endpoint_excess_is_still_rejected(self):
        ctx=context();plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fixture.fresh_sample(ctx)
        sample['arms']['right']['joints_rad'][1]-=.0029
        sample['arms']['right']['joints_rad'][2]+=.0029
        sample['arms']['right']['joints_rad'][3]=math.radians(.49)
        with self.assertRaisesRegex(ji.JointInitializationError,'model_endpoint_displacement'):
            ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase='pre_dispatch')

    def test_no_extra_recovery_excursion_or_metric_fallback(self):
        ctx=context()
        for label in ('origin','current'):
            ctx[label]['arms']['right']['joints_rad'][1:3]=[-.099,.099]
        fixture.commit_origin(ctx)
        with self.assertRaisesRegex(ji.JointInitializationError,'initialization_endpoint_reserve_unavailable'):
            ji.plan_joint_initialization(ctx,now=100.01)
        ctx=context();ctx.pop('feedback_observation')
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertNotIn('target_selection',plan)
        self.assertEqual(plan['target_raw'][1:3],[0,0])
        ctx=context();ctx['geometry']=fixture.context()['geometry']
        with self.assertRaisesRegex(ji.JointInitializationError,'feedback_policy_requires_rgb'):
            ji.plan_joint_initialization(ctx,now=100.01)


class NativeEndpointSelectionTests(unittest.TestCase):
    # Real host/source provider/native encoder; transport and images are fake.
    def test_once_only_bounded_startup_then_source_bound_inward_motion(self):
        f=native.NativeFeedbackToleranceTests('runTest');f.setUp();self.addCleanup(f.doCleanups)
        f.joints={'left':LEFT[:],'right':RIGHT[:]}
        f.prepared()
        source=f.device._joint_initializations['right']
        self.assertEqual(source['target_selection']['mode'],'j2_interior_for_endpoint_reserve_v1')
        self.assertEqual(f.ids('right')[:4],[0x151,0x155,0x156,0x157])
        original_source=copy.deepcopy(source)
        source['target_selection']['j2_interior_margin_rad']=.02
        bad=f.step('right','forged-selection')
        with self.assertRaises(ji.JointInitializationError) as rejected:
            f.service.call('robot_pair_submit_once',bad)
        self.assertEqual(rejected.exception.code,'initialization_source_target_selection')
        self.assertEqual(len(f.ids('right')),5)
        self.assertIsNone(f.host.ledger.event('forged-selection'))
        f.device._joint_initializations['right']=original_source
        before=f.host.ledger.peek_status()['steps'];request=f.step('right','endpoint-inward')
        result=f.execute(request)
        self.assertEqual(result['status'],'completed',result.get('receipt'))
        self.assertEqual(f.host.ledger.peek_status()['steps'],before+1)
        self.assertEqual(len(f.ids('right')),9)  # init four, jaw one, joint four.
