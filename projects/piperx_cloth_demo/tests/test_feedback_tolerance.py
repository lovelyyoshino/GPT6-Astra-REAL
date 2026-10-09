"""Task-bound raw feedback policy; offline only, including native SDK/FakeCAN."""
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from robot_tools import feedback_tolerance as ft, joint_path as jp, joint_initialization as ji
from robot_tools import contact_receipt, retention_receipt, grasp_episode
import test_joint_path as path_fixture
import test_joint_initialization as init_fixture
import test_contact_receipt as contact_fixture
import test_grasp_episode as grasp_fixture
import test_host_rgb_joint as native
import test_pair_task_enrollment as enrollment

POLICY = {"profile": ft.PROFILE, "source": "user", "statement": "本次右臂 J4 小幅反馈波动可以允许。"}
JITTER = math.radians(.481)
POSE_FIXTURE = json.loads((Path(__file__).parent/'fixtures/right_j4_pose_observation.json').read_text())
POSE_KEYS = ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')


def jitter_pose(angle):
    # Recorded low/high controller poses, scaled only inside synthetic tests.
    lo,hi=POSE_FIXTURE['low'],POSE_FIXTURE['high']
    span=math.radians((hi['joints_raw']['joint_4']-lo['joints_raw']['joint_4'])/1000)
    return [(lo['end_pose_raw'][k]+angle/span*(hi['end_pose_raw'][k]-lo['end_pose_raw'][k]))
            *(1e-6 if i<3 else math.pi/180000) for i,k in enumerate(POSE_KEYS)]


class FeedbackToleranceTests(unittest.TestCase):
    def setUp(self):
        guard = patch('socket.socket', side_effect=AssertionError('Offline tests only'))
        guard.start(); self.addCleanup(guard.stop)

    def test_explicit_task_bound_authorization_and_axis_limits(self):
        self.assertEqual(ft.task_policy({'task_id':'plug_transfer_left','site_context':{'feedback_observation':POLICY}}),POLICY)
        for task in ('different_task', '', None):
            with self.assertRaises(ValueError):
                ft.task_policy({'task_id':task,'site_context':{'feedback_observation':POLICY}})
        for policy in ({**POLICY,'profile':'anything'}, {**POLICY,'source':'model'}, {**POLICY,'statement':''}):
            with self.assertRaises(ValueError): ft.validate_policy(policy)
        origin = [0.]*6
        for side in ('left','right'):
            for axis in range(6):
                after=origin[:];after[axis]=JITTER
                self.assertEqual(ft.joints_within(POLICY,side,origin,after),side=='right' and axis==3)
                self.assertFalse(ft.joints_within(None,side,origin,after))
        self.assertFalse(ft.joints_within(POLICY,'right',origin,[0,0,0,math.radians(.501),0,0]))

    def test_initialization_and_path_preserve_fixed_anchor_raw_data_and_pose_limits(self):
        for arm in ('left','right'):
            ctx=init_fixture.visual_context(arm);ctx['feedback_observation']=POLICY
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            observed=init_fixture.fresh_sample(ctx)
            observed['arms']['right']['joints_rad'][3]+=JITTER
            original=copy.deepcopy(observed)
            result=ji.validate_joint_initialization_sample(plan,observed,now=100.02,phase='pre_dispatch')
            self.assertTrue(result['within_initialization_envelope']);self.assertEqual(original,observed)
            # Small consecutive deltas do not reset the original anchor.
            observed['arms']['right']['joints_rad'][3]+=math.radians(.04)
            with self.assertRaises(ji.JointInitializationError):
                ji.validate_joint_initialization_sample(plan,observed,now=100.02,phase='pre_dispatch')
        for arm in ('left','right'):
            ctx,q=path_fixture.visual_joint_context(path_fixture.context(arm=arm))
            ctx['feedback_observation']=POLICY
            plan=jp.plan_joint_path(ctx,q,now=100.01)
            observed=copy.deepcopy(ctx['current']);observed['arms']['right']['joints_rad'][3]+=JITTER
            jp.validate_joint_path_sample(plan,observed,now=100.01,phase='pre_dispatch')
            observed['arms']['right']['pose_m_rad'][0]+=.001
            with self.assertRaises(jp.JointPathError):
                jp.validate_joint_path_sample(plan,observed,now=100.01,phase='pre_dispatch')
        ctx,q=path_fixture.visual_joint_context();ctx['feedback_observation']=POLICY
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        self.assertEqual(plan['budget'],ctx['budget'])
        self.assertEqual(plan['tracking_policy']['max_cumulative_outside_band_s'],1.)
        plan['feedback_observation']=None
        with self.assertRaises(jp.JointPathError):jp.validate_joint_path_sample(plan,ctx['current'],now=100.01)

    def test_coarse_geometry_includes_j4_band_without_enlarging_motion_budget(self):
        ctx,q=path_fixture.coarse_joint_context();ctx['feedback_observation']=POLICY
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        sample=copy.deepcopy(ctx['current']);sample['arms']['right']['joints_rad'][3]+=JITTER
        jp.validate_joint_path_sample(plan,sample,now=100.01,phase='pre_dispatch')
        self.assertEqual(plan['budget'],ctx['budget'])
        self.assertEqual(plan['tracking_policy']['settle_tolerances_rad'],ft.joint_tolerances(POLICY,'right'))
        self.assertFalse(plan['hold_supported'])

    def test_device_rejects_changed_or_injected_context_policy_before_any_io(self):
        from types import SimpleNamespace
        from robot_tools.pair_joint_adapter import _JointExecutor
        from robot_tools.pair_initialization import _Initializer
        device=SimpleNamespace(_action=SimpleNamespace(feedback_policy=POLICY))
        # The mismatch is checked even before identity/device state is accessed.
        with self.assertRaisesRegex(RuntimeError,'frozen device task'):
            _JointExecutor(device,{},'event',100.,None)
        with self.assertRaisesRegex(RuntimeError,'frozen device task'):
            _Initializer(device,{'identity':{}},'event',100.)

    def test_strict_default_and_unsupported_metric_mode(self):
        ctx=init_fixture.visual_context();plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=init_fixture.fresh_sample(ctx);sample['arms']['right']['joints_rad'][3]+=JITTER
        with self.assertRaises(ji.JointInitializationError):
            ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase='pre_dispatch')
        ctx=init_fixture.context();ctx['feedback_observation']=POLICY
        with self.assertRaisesRegex(ji.JointInitializationError,'feedback_policy_requires_rgb'):
            ji.plan_joint_initialization(ctx,now=100.01)

    def test_recorded_pose_swing_matches_bound_and_rejects_other_arm_or_excess(self):
        samples=[contact_fixture.sample(100+i*.05) for i in range(61)]
        for i,sample in enumerate(samples):
            row=POSE_FIXTURE['low' if i%2==0 else 'high']
            sample['arms']['right']['joints_rad'][3]=math.radians(row['joints_raw']['joint_4']/1000)
            sample['arms']['right']['pose_m_rad']=[row['end_pose_raw'][k]*(1e-6 if n<3 else math.pi/180000)
                                                   for n,k in enumerate(POSE_KEYS)]
        original=copy.deepcopy(samples)
        with self.assertRaises(ValueError):contact_receipt._stable(samples)
        spans=contact_receipt._stable(samples,feedback_policy=POLICY)
        self.assertGreater(spans['right']['rotation_rad'],.003)
        self.assertLess(spans['right']['rotation_rad'],ft.RIGHT_J4_RAD)
        self.assertEqual(samples,original)
        for side,angle in (('left',.2),('right',.501)):
            changed=copy.deepcopy(samples)
            for i,sample in enumerate(changed):sample['arms'][side]['pose_m_rad']=[.1,.1,.1,math.radians(angle)*(i%2),0,0]
            with self.assertRaises(ValueError):contact_receipt._stable(changed,feedback_policy=POLICY)
        changed=copy.deepcopy(samples)
        changed[30]['arms']['right']['pose_m_rad'][0]+=.001
        with self.assertRaises(ValueError):contact_receipt._stable(changed,feedback_policy=POLICY)

    def test_contact_and_retention_use_same_policy_and_unmodified_raw_trace(self):
        args=dict(arm='right',requested_width_m=.025,sent_at=103.02,
            baseline_samples=[contact_fixture.sample(100+i*.05) for i in range(61)],
            post_samples=[contact_fixture.sample(103.07+i*.05,.0285) for i in range(81)])
        for trace in (args['baseline_samples'],args['post_samples']):
            for i,sample in enumerate(trace):sample['arms']['right']['joints_rad'][3]+=JITTER*(i%2)
        original=copy.deepcopy(args)
        self.assertEqual(contact_receipt.classify_gripper_probe(**args)['outcome'],'unconfirmed')
        result=contact_receipt.classify_gripper_probe(**args,feedback_policy=POLICY)
        self.assertEqual(result['outcome'],'settled_contact_candidate',result)
        self.assertEqual(args,original)
        samples=args['post_samples']
        anchor=retention_receipt.measured_anchor(samples[0]['arms']['right'])
        summary=retention_receipt.summarize_retention_trace(arm='right',identity=None,probe_event_id=None,
            trace_id='trace',trace_sha256='a'*64,original_anchor=anchor,samples=samples,
            now=samples[-1]['observed_at_s'],feedback_policy=POLICY)
        self.assertEqual(summary['joint_spans_rad'][3],JITTER)
        self.assertEqual(summary['spans']['joint_rad'],JITTER)
        stale=copy.deepcopy(args);stale['post_samples'][10]['arms']['right']['fragment_timestamps_s']['joints']=1.
        self.assertEqual(contact_receipt.classify_gripper_probe(**stale,feedback_policy=POLICY)['outcome'],'unconfirmed')
        for sample in samples:sample['arms']['right']['joints_rad'][3]+=math.radians(.6)
        with self.assertRaisesRegex(ValueError,'original candidate anchor'):
            retention_receipt.summarize_retention_trace(arm='right',identity=None,probe_event_id=None,
                trace_id='trace',trace_sha256='a'*64,original_anchor=anchor,samples=samples,
                now=samples[-1]['observed_at_s'],feedback_policy=POLICY)

    def test_grasp_episode_requires_axis_evidence_and_matching_authorization(self):
        state=grasp_fixture.episode('right');state['feedback_observation']=POLICY
        data=grasp_fixture.measurement(state)
        data.update(feedback_observation=POLICY,joint_spans_rad=[0,0,0,JITTER,0,0],
                    joint_anchor_deviation_rad=[0,0,0,JITTER,0,0])
        data['spans']['joint_rad']=data['anchor_deviation']['joint_rad']=JITTER
        data['observed']['joints_rad'][3]=JITTER
        grasp_episode._measurement(state,data,104.)
        for mutation in ('wrong_axis','no_policy','missing_vector'):
            changed=copy.deepcopy(data)
            if mutation=='wrong_axis':changed['joint_spans_rad']=[JITTER,0,0,0,0,0]
            elif mutation=='no_policy':changed.pop('feedback_observation')
            else:changed.pop('joint_anchor_deviation_rad')
            with self.assertRaises(grasp_episode.GraspEpisodeError):grasp_episode._measurement(state,changed,104.)


class NativeFeedbackToleranceTests(unittest.TestCase):
    # Reuse fixture methods, not its test cases. Real encoders, fake CAN transport.
    open=native.HostRGBJointTests.open
    ids=native.HostRGBJointTests.ids
    check_async_errors=native.HostRGBJointTests.check_async_errors
    snapshot=native.HostRGBJointTests.snapshot
    start=native.HostRGBJointTests.start
    observe=native.HostRGBJointTests.observe
    request=native.HostRGBJointTests.request
    initialize=native.HostRGBJointTests.initialize
    prepared=native.HostRGBJointTests.prepared
    step=native.HostRGBJointTests.step
    execute=native.HostRGBJointTests.execute

    def setUp(self):
        native.HostRGBJointTests.setUp(self)
        task=native.fixture.native.TASK
        patched=patch.dict(task,site_context={**task['site_context'],'feedback_observation':POLICY})
        patched.start();self.addCleanup(patched.stop)
        self.counter=0
        self.jitter=JITTER
        def mutate(side,state):
            if side=='right':
                self.counter+=1
                angle=self.jitter*(self.counter%2)
                state['joints_rad'][3]+=angle
                state['pose_m_rad']=jitter_pose(angle)
        self.feedback_mutator=mutate

    def test_prepare_initialize_both_jaws_ingress_then_move_both_arms(self):
        self.prepared()
        self.assertEqual(self.host.feedback_policy,POLICY)
        self.assertEqual(self.device._action.feedback_policy,POLICY)
        for side in ('left','right'):
            request=self.step(side,'jitter-'+side)
            before=len(self.ids(side));peer='right' if side=='left' else 'left';peer_before=self.ids(peer)
            result=self.execute(request)
            self.assertEqual(result['status'],'completed',result.get('receipt'))
            self.assertEqual(result['receipt']['joint_path_plan']['feedback_observation'],POLICY)
            self.assertEqual(self.ids(side)[before:],[0x151,0x155,0x156,0x157])
            self.assertEqual(self.ids(peer),peer_before)
        self.assertEqual(self.host.ledger.peek_status()['steps'],6)
        before=len(self.sent)
        origin=self.device._action.idle_anchor['right']['joints_rad'][3]
        def out_of_bound(side,state):
            if side=='right':state['joints_rad'][3]=origin+math.radians(.501)
        self.feedback_mutator=out_of_bound
        with self.assertRaisesRegex(RuntimeError,'stationary|anchor|envelope'):self.device.observe()
        self.assertEqual(len(self.sent),before)

    def _service_open_query_initialize_and_move(self, policy):
        # Exercise the real service constructor and the host's publication path.
        # Only hardware limit acquisition is replaced by explicit synthetic raw
        # replies. Do not publish with the reader's profile: that hid the bug.
        from types import SimpleNamespace
        from robot_tools.pair_host import PairHost
        from robot_tools.joint_sources import JointSourcesProvider
        fixture = native.fixture
        original_profile = copy.deepcopy(self.service.profile)
        self.service.persistent = True
        self.jaws = dict.fromkeys(self.channels, False)
        if policy is None:
            self.feedback_mutator = None
        args = dict(run_id='first-target-integration', task_id='plug_transfer_left',
                    workspace_clearance_statement='Synthetic current clear workspace',
                    connection_mode='prepare')
        if policy:
            args.update(feedback_observation_profile=policy['profile'],
                        feedback_observation_statement=policy['statement'])
        def host_factory(*values, **kwargs):
            return PairHost(*values, clock=self.clock.time, background=False, **kwargs)
        def provider_factory(*values, **kwargs):
            return JointSourcesProvider(*values, clock=self.clock.time, **kwargs)
        with patch('robot_tools.pair_host.PairHost', side_effect=host_factory), \
                patch('robot_tools.joint_sources.JointSourcesProvider', side_effect=provider_factory):
            self.service.call('robot_pair_open', args)
        self.host = self.service.pair_host
        self.addCleanup(self.host.close)
        self.device = self.host.device
        self.sources = self.host.joint_sources_provider
        self.assertEqual(self.service.profile, original_profile)
        for robot in self.device._action.robots.values():
            self.assertEqual(hasattr(robot,'_pair_coherent_feedback'),policy is not None)
        capture = fixture.source_fixture.JointSourcesTests.make_capture(
            SimpleNamespace(bindings=self.host._joint_bindings()))
        capture.update(ok=True, hardware_commands_sent=12)
        offset = self.clock.time() - capture['began_at']
        time_keys = {'began_at', 'ended_at', 'request_started_unix_s', 'finished_unix_s',
                     'sent_at', 'returned_at', 'timestamp', 'received_unix_s'}
        def shift(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    if key in time_keys:
                        value[key] = item + offset
                    else:
                        shift(item)
            elif isinstance(value, list):
                for item in value:
                    shift(item)
        shift(capture)
        self.clock.sleep(4.01)
        with patch.object(self.device, 'inspect_joint_limits', return_value=capture):
            self.assertEqual(self.service.call('robot_pair_inspect_joint_limits',
                {'event_id': 'service-limits'})['status'], 'pending')
            result = self.host.wait('service-limits', 10)
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.publication = result['receipt']['source_publication']
        owner, deadline = self.host.owner, self.host.deadline
        for side in ('left', 'right'):
            result = self.initialize(self.request(side, 'service-init-' + side))
            self.assertEqual(result['status'], 'completed', result.get('receipt'))
            self.assertEqual(self.ids(side), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.sources.profile, self.host.profile)
        for side in ('left', 'right'):
            scene = self.observe()
            event = 'service-jaw-' + side
            self.service.call('robot_pair_prepare_gripper', dict(event_id=event,
                observation_id=scene['observation_id'], arm=side,
                empty_jaw_observation=fixture.UNLOADED))
            self.assertEqual(self.host.wait(event, 10)['status'], 'completed')
        self.assertTrue(self.service.call('robot_pair_promote_ready', {})['task_ready'])
        for side in ('left', 'right'):
            result = self.execute(self.step(side, 'service-step-' + side))
            self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))
        self.assertEqual(self.host.ledger.peek_status()['steps'], 7)
        self.assertEqual((len(self.created), len(self.buses)), (2, 2))
        self.assertEqual(self.service.profile, original_profile)

    def test_service_open_bounded_policy_reuses_published_limits_through_motion(self):
        self._service_open_query_initialize_and_move(POLICY)

    def test_service_open_strict_default_reuses_published_limits_through_motion(self):
        self._service_open_query_initialize_and_move(None)

    def test_partial_send_is_latched_without_retry(self):
        self.prepared()
        self.fail_id=0x156
        result=self.execute(self.step('right','partial-jitter'))
        self.assertEqual(result['status'],'fault')
        self.assertTrue(self.host.fault_event.is_set())
        before=len(self.sent)
        with self.assertRaises(Exception):self.device.observe()
        self.assertEqual(before,len(self.sent))


class EnrollmentFeedbackToleranceTests(unittest.TestCase):
    def test_new_contract_allows_only_authorized_right_j4_and_preserves_history(self):
        f=enrollment.TaskEnrollmentTests('runTest');f.setUp();self.addCleanup(f.doCleanups)
        raw=json.loads(f.passive['right'].read_text())
        for i,row in enumerate(raw['pose_trace']):
            row['joints_raw']['joint_4']+=481*(i%2)
            row['end_pose_raw']={k:round(v/(1e-6 if n<3 else math.pi/180000))
                                 for n,(k,v) in enumerate(zip(POSE_KEYS,jitter_pose(JITTER*(i%2))))}
        f.passive['right'].write_text(json.dumps(raw))
        before=f.rows()
        with self.assertRaises(enrollment.PairLedgerError):f.prepare()
        task=json.loads(f.task_file.read_text());task['task']['site_context']['feedback_observation']=POLICY
        f.task_file.write_text(json.dumps(task))
        proposal=f.prepare()
        self.assertEqual(before,f.rows())
        self.assertEqual(proposal['reviewed_contract']['task']['site_context']['feedback_observation'],POLICY)
        for i,row in enumerate(raw['pose_trace']):row['joints_raw']['joint_3']+=600*(i%2)
        f.passive['right'].write_text(json.dumps(raw))
        with self.assertRaises(enrollment.PairLedgerError):f.prepare()


import test_host_loaded_joint as loaded_native


class NativeLoadedFeedbackToleranceTests(unittest.TestCase):
    def setUp(self):
        self.contacts=set()
        NativeFeedbackToleranceTests.setUp(self)

    open=NativeFeedbackToleranceTests.open
    ids=NativeFeedbackToleranceTests.ids
    check_async_errors=NativeFeedbackToleranceTests.check_async_errors
    start=NativeFeedbackToleranceTests.start
    observe=NativeFeedbackToleranceTests.observe
    request=NativeFeedbackToleranceTests.request
    initialize=NativeFeedbackToleranceTests.initialize
    prepared=NativeFeedbackToleranceTests.prepared
    step=NativeFeedbackToleranceTests.step
    execute=NativeFeedbackToleranceTests.execute
    snapshot=loaded_native.HostLoadedJointTests.snapshot
    frame_record=loaded_native.HostLoadedJointTests.frame_record
    jaw=loaded_native.HostLoadedJointTests.jaw
    retained_pair=loaded_native.HostLoadedJointTests.retained_pair
    loaded_request=loaded_native.HostLoadedJointTests.loaded_request
    confirm_loaded=loaded_native.HostLoadedJointTests.confirm_loaded
    release_confirm=loaded_native.HostLoadedJointTests.release_confirm

    def test_native_probe_retain_extract_insert_release_with_j4_fluctuation(self):
        self.contacts=set()
        loaded_native.HostLoadedJointTests.test_native_complete_chain_pending_response_and_local_anchor_release(self)


import test_pair_service as service_fixture


class ServiceFeedbackToleranceTests(unittest.TestCase):
    def test_schema_freezes_named_policy_and_requires_the_original_statement(self):
        f=service_fixture.PairServiceTests('runTest');f.setUp();self.addCleanup(f.doCleanups)
        f.service.persistent=True
        args={**service_fixture.OPEN,'feedback_observation_profile':ft.PROFILE,
              'feedback_observation_statement':POLICY['statement']}
        with patch('robot_tools.pair_host.PairHost',return_value=f.host) as constructor:
            f.service.call('robot_pair_open',args)
            self.assertEqual(constructor.call_args.args[3]['site_context']['feedback_observation'],POLICY)
        for wrong in ({'feedback_observation_statement':None},{'task_id':'other'},
                      {'feedback_observation_profile':'all_joints'}, {'feedback_observation_rad':1.}):
            f.service.pair_host=None
            with patch('robot_tools.pair_host.PairHost') as constructor:
                with self.assertRaises(ValueError):f.service.call('robot_pair_open',{**args,**wrong})
                constructor.assert_not_called()
