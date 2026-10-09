"""Real service/host/native SDK flow on fake CAN; enrollment tested separately."""
import copy
from unittest.mock import patch
from robot_tools import host_recovery
from robot_tools.retention_receipt import measured_anchor, digest
from test_host_grasp_integration import HostGraspFixture


class HostSupportedRecoveryTests(HostGraspFixture):
    def setUp(self):
        super().setUp()
        states=copy.deepcopy(self.host.device._previous)
        self.source=dict(before=states,
            candidate_measurement=dict(anchor=measured_anchor(states['left']),
                                       observed=dict(width_m=states['left']['gripper']['width_m'])),
            candidate_probe=dict(requested_width_m=.046,sent_at=self.clock.time()-10,trace_sha256='a'*64))
        self.proposal=dict(proposal_sha256='b'*64,
            snapshot=dict(event=dict(event_id='old-failed-probe'),run={},scope=dict(owner='old-owner')))
        self.stack.enter_context(patch.object(host_recovery, 'recovery_source',
                                              return_value=({'arm':'left'},self.source)))
        self.stack.enter_context(patch.object(host_recovery.SupportedRecovery,'state',lambda _:self.recovery_state()))

    def recovery_state(self):
        opening=self.host.ledger.event('recovery-open')
        confirmation=self.host.ledger.event('recovery-confirm')
        phase='opening_required' if opening is None else 'confirmation_required'
        events=[]
        if opening:
            if opening['status']!='complete' or not opening['success']:phase='unresolved'
            token=self.host.device._supported_recovery_opening
            if token is not None:opening={**opening,'finished_at':token['finished_at']}
            events=[opening]
        if confirmation:
            phase='resolved' if confirmation['status']=='complete' and confirmation['success'] else 'unresolved'
        return dict(phase=phase,events=events,proposal=self.proposal)

    def opening(self):
        scene=self.observe()
        req=dict(event_id='recovery-open',observation_id=scene['observation_id'],width_m=.054,
                 visual_description='Synthetic independently supported object, no body motion.',
                 support_relation='independent_support_present')
        self.service.call('robot_pair_recover_supported_gripper',req)
        result=self.host.wait('recovery-open',10)
        self.assertEqual(result['status'],'completed',result.get('receipt'))
        return req,result

    def test_service_open_confirm_replay_and_preparation_gate(self):
        with self.assertRaisesRegex(RuntimeError,'recovery must finish'):
            self.host.inspect_joint_limits('premature-query')
        req,opened=self.opening()
        self.assertEqual(opened['receipt']['hardware_commands_sent'],1)
        self.assertEqual(self.robots['right'].sent,[])
        self.assertEqual(len(self.robots['left'].sent),1)
        replay=self.service.call('robot_pair_recover_supported_gripper',req)
        self.assertTrue(replay['replayed'])
        with self.assertRaisesRegex(RuntimeError,'recovery must finish'):
            self.host.inspect_joint_limits('still-premature')
        scene=self.observe()
        req=dict(event_id='recovery-confirm',observation_id=scene['observation_id'],
                 visual_description='Synthetic object supported clear of the fingers.',
                 support_relation='independent_support_present',object_relation='object_clear_of_fingers')
        self.service.call('robot_pair_confirm_recovery_release',req)
        result=self.host.wait('recovery-confirm',10)
        self.assertEqual(result['status'],'completed',result.get('receipt'))
        self.assertEqual(result['receipt']['hardware_commands_sent'],0)
        self.assertEqual(self.host.ledger.status()['steps'],2)
        host_recovery.SupportedRecovery(self.host).require_resolved()
        self.assertEqual(len(self.robots['left'].sent),1)
        self.assertEqual(self.host.grasps.states(),[])

    def test_missing_support_or_separation_cannot_claim(self):
        scene=self.observe()
        with self.assertRaises(ValueError):
            self.service.call('robot_pair_recover_supported_gripper',dict(event_id='recovery-open',
                observation_id=scene['observation_id'],width_m=.054,visual_description='Synthetic unsupported',
                support_relation='unknown'))
        self.assertEqual(self.host.ledger.status()['steps'],0)
        self.opening()
        scene=self.observe()
        with self.assertRaises(ValueError):
            self.service.call('robot_pair_confirm_recovery_release',dict(event_id='recovery-confirm',
                observation_id=scene['observation_id'],visual_description='Synthetic still touching',
                support_relation='independent_support_present',object_relation='between_fingers'))
        self.assertEqual(len(self.robots['left'].sent),1)

    def test_partial_opening_failure_blocks_confirmation(self):
        self.robots['left'].fail_id=0x159
        scene=self.observe()
        self.service.call('robot_pair_recover_supported_gripper',dict(event_id='recovery-open',
            observation_id=scene['observation_id'],width_m=.054,visual_description='Synthetic independent support',
            support_relation='independent_support_present'))
        result=self.host.wait('recovery-open',10)
        self.assertEqual(result['status'],'fault')
        self.assertFalse(result['receipt']['device_receipt']['ok'])
        self.assertTrue(self.host.status()['fault_latched'])
        self.assertEqual(self.robots['right'].sent,[])

    def test_audited_continuation_cannot_replay_prior_width_or_claim_step(self):
        # A width that is a legal current increment can still be an old target.
        self.source['audited_opening_continuation'] = {'prior_opening_target_m': .054}
        scene = self.observe()
        with self.assertRaisesRegex(ValueError, 'must exceed the previous'):
            self.service.call('robot_pair_recover_supported_gripper', dict(
                event_id='old-opening-replay', observation_id=scene['observation_id'],
                width_m=.054, visual_description='Synthetic object independently supported.',
                support_relation='independent_support_present'))
        self.assertEqual(self.host.ledger.status()['steps'], 0)
        self.assertEqual(self.robots['left'].sent, [])
        self.assertEqual(self.robots['right'].sent, [])

    def test_audited_continuation_opens_then_observes_release_without_body_tx(self):
        original = self.source['candidate_measurement']['anchor']
        original['width_m'] = .046
        self.source['candidate_measurement']['observed']['width_m'] = .046
        self.source['candidate_probe']['completed_at'] = self.clock.time() - 5
        frozen = copy.deepcopy(self.source)
        self.source['audited_opening_continuation'] = dict(
            schema='piper_supported_opening_continuation_v1', arm='left',
            source_receipt_sha256=digest(frozen), prior_opening_event_id='synthetic-failed-opening',
            prior_opening_receipt_sha256='c'*64, prior_opening_target_m=.051,
            prior_opening_finished_at=self.clock.time()-3, audit_proposal_sha256='d'*64,
            residual_jaw_anchor=dict(width_m=.05, observed_at=self.clock.time()-1,
                                     source=dict(path='/synthetic/passive.json', sha256='e'*64)))
        _, opened = self.opening()
        self.assertEqual(opened['receipt']['hardware_commands_sent'], 1)
        self.assertEqual(original, frozen['candidate_measurement']['anchor'])
        self.assertEqual([frame.arbitration_id for frame in self.robots['left'].sent], [0x159])
        scene = self.observe()
        self.service.call('robot_pair_confirm_recovery_release', dict(
            event_id='recovery-confirm', observation_id=scene['observation_id'],
            visual_description='Synthetic object visibly separate and independently supported.',
            support_relation='independent_support_present', object_relation='object_clear_of_fingers'))
        result = self.host.wait('recovery-confirm', 10)
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(result['receipt']['hardware_commands_sent'], 0)
        self.assertEqual(len(self.robots['left'].sent), 1)
        self.assertEqual(self.robots['right'].sent, [])
