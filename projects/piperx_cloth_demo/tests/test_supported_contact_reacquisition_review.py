"""Independent supported left-jaw reacquisition checks; never physical I/O.

Host tests retain the actual service, ledger, grasp store, device adapter and
native encoder on FakeCAN. Only the separately audited enrollment source and
its phase reader are synthetic, not a real-scene qualification.
"""
import copy
from unittest.mock import patch

from robot_tools import host_recovery
from robot_tools import supported_gripper_recovery as recovery
from robot_tools.retention_receipt import measured_anchor, digest
from test_host_grasp_integration import HostGraspFixture


class SupportedContactReacquisitionHostReviewTests(HostGraspFixture):
    def setUp(self):
        super().setUp()
        states = copy.deepcopy(self.host.device._previous)
        anchor = measured_anchor(states['left'])
        anchor['width_m'] = .046
        self.source = dict(before=states,
            candidate_measurement=dict(anchor=anchor, observed=dict(width_m=.046)),
            candidate_probe=dict(requested_width_m=.044, sent_at=self.clock.time()-40,
                                 completed_at=self.clock.time()-30, trace_sha256='a'*64))
        self.source['audited_supported_reacquisition'] = dict(
            schema='piper_supported_reacquisition_v1', arm='left',
            source_receipt_sha256=digest(self.source), opening_event_id='old-complete-opening',
            opening_receipt_sha256='b'*64, opening_finished_at=self.clock.time()-4,
            audit_proposal_sha256='c'*64,
            opening_jaw_anchor=dict(width_m=.05, observed_at=self.clock.time()-5,
                                    source=dict(path='/synthetic/opening.json', sha256='d'*64)))
        self.proposal = dict(route='audited_contact_reacquisition', proposal_sha256='c'*64,
            snapshot=dict(event=dict(event_id='old-complete-opening')),
            reacquisition=copy.deepcopy(self.source['audited_supported_reacquisition']),
            evidence=dict(passive=dict(left=dict(jaw_width_m=.05))), budget=dict(steps=0))
        self.old_payload = dict(arm='left', grasp_object_id='charger')
        self.stack.enter_context(patch.object(recovery, 'reacquisition_source',
                                               return_value=(self.old_payload, self.source), create=True))
        self.stack.enter_context(patch.object(host_recovery.SupportedRecovery, 'state',
                                               lambda _: self.recovery_state()))
        self.probes = []
        self.contact_hook = self.hook

    def recovery_state(self):
        events = [self.host.ledger.event(event_id) for event_id in self.probes]
        events = [event for event in events if event is not None]
        phase = 'reacquire_required'
        for event in events:
            if event['status'] != 'complete' or not event['success']:
                phase = 'unresolved'
                break
            if event['receipt'].get('contact_observation', {}).get('outcome') == 'settled_contact_candidate':
                phase = 'contact_candidate'
                break
        if len(events) >= 3 and phase == 'reacquire_required':
            phase = 'exhausted'
        return dict(phase=phase, proposal=self.proposal, events=events, probe_count=len(events))

    def request(self, event_id='reacquire-1', *, target=.0455, **changes):
        scene = self.observe()
        result = dict(event_id=event_id, observation_id=scene['observation_id'],
            peer_receipt_id=scene['peer_receipts']['right']['receipt_id'], arm='left',
            kind='gripper', operation='grip_supported', width_m=target,
            grasp_object_id='charger',
            probe_support_observation='Synthetic left fingertip still touches the charger; original socket independently supports it.',
            probe_support_relation='independent_support_present')
        result.update(changes)
        return result

    def probe(self, event_id='reacquire-1', *, target=.0455, contact=.048):
        if contact is not None:
            self.robots['left'].accept = False
            self.contacts['left'] = contact
            prior_frames = len(self.robots['left'].sent)
            self.hook = lambda robot, state: (
                self.contact_hook(robot, state)
                if robot.side != 'left' or len(robot.sent) > prior_frames else None)
        request = self.request(event_id, target=target)
        self.probes.append(event_id)
        self.service.call('robot_pair_submit_once', request)
        result = self.host.wait(event_id, 10)
        return request, result

    def assert_only_left_jaw_frames(self, count):
        self.assertEqual([frame.arbitration_id for frame in self.robots['left'].sent], [0x159]*count)
        self.assertEqual(self.robots['right'].sent, [])

    def test_touching_supported_object_can_form_new_candidate_then_retain_without_tx(self):
        frozen = copy.deepcopy(self.source)
        request, result = self.probe()
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(result['receipt']['contact_observation']['outcome'], 'settled_contact_candidate')
        episode = self.host.grasps.active('left')
        self.assertEqual(episode['status'], 'contact_candidate')
        self.assertEqual(episode['identity']['object_id'], 'charger')
        self.assertEqual(episode['probe_ref']['event_id'], request['event_id'])
        self.assert_only_left_jaw_frames(1)
        retained, _ = self.retain('left')
        self.assertEqual(retained['episode']['status'], 'retained_static')
        self.assertEqual(retained['hardware_commands_sent'], 0)
        self.assertFalse(retained['loaded_contact_available'])
        self.assert_only_left_jaw_frames(1)
        self.assertEqual(self.source, frozen)

    def test_right_other_object_or_missing_support_is_rejected_before_claim(self):
        for changes in (dict(arm='right'), dict(grasp_object_id='other-object'),
                        dict(probe_support_relation='unknown'), dict(probe_support_observation='')):
            with self.subTest(changes=changes):
                request = self.request('wrong-binding', **changes)
                with self.assertRaises((ValueError, RuntimeError)):
                    self.service.call('robot_pair_submit_once', request)
                self.assertIsNone(self.host.ledger.event('wrong-binding'))
        self.assert_only_left_jaw_frames(0)

    def test_candidate_and_retention_never_unlock_ordinary_or_loaded_motion(self):
        _, result = self.probe()
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.retain('left')
        with self.assertRaises(RuntimeError):
            self.host.inspect_joint_limits('forbidden-query')
        for side, operation in (('left', 'approach'), ('right', 'approach'),
                                ('left', 'extract_segment'), ('right', 'grip_supported')):
            scene = self.observe()
            kind = 'gripper' if operation == 'grip_supported' else 'move'
            request = dict(event_id='forbidden-'+side+'-'+operation,
                observation_id=scene['observation_id'],
                peer_receipt_id=scene['peer_receipts']['right' if side == 'left' else 'left']['receipt_id'],
                arm=side, kind=kind, operation=operation)
            request['width_m' if kind == 'gripper' else 'target_pose_m_rad'] = (
                .0455 if kind == 'gripper' else self.robots[side].motion.origin[:])
            with self.subTest(side=side, operation=operation), self.assertRaises((ValueError, RuntimeError)):
                self.service.call('robot_pair_submit_once', request)
        self.assert_only_left_jaw_frames(1)

    def test_failed_or_unknown_frame_never_creates_a_retainable_candidate(self):
        self.robots['left'].fail_id = 0x159
        _, result = self.probe()
        self.assertEqual(result['status'], 'fault')
        self.assertFalse(result['receipt']['ok'])
        self.assertTrue(self.host.status()['fault_latched'])
        state = self.host.grasps.active('left')
        self.assertTrue(state is None or state['status'] == 'empty')
        before = len(self.robots['left'].sent)
        with self.assertRaises((ValueError, RuntimeError)):
            self.service.call('robot_pair_submit_once', self.request('not-a-retry'))
        self.assertEqual(len(self.robots['left'].sent), before)
        self.assertEqual(self.robots['right'].sent, [])

    def test_target_arrival_without_contact_does_not_grant_retention(self):
        _, result = self.probe(contact=None)
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(result['receipt']['contact_observation']['outcome'], 'target_arrived')
        state = self.host.grasps.active('left')
        self.assertEqual(state['status'], 'empty')
        scene = self.observe()
        with self.assertRaises((ValueError, RuntimeError)):
            self.host.retain_grasp('premature-retain', state['identity']['episode_id'], scene['observation_id'],
                'Synthetic reached width without contact.', 'between_fingers', 'original_support_present')
        self.assert_only_left_jaw_frames(1)

    def test_new_smaller_probe_after_arrival_can_create_candidate_without_body_motion(self):
        _, first = self.probe(contact=None)
        self.assertEqual(first['status'], 'completed', first.get('receipt'))
        _, second = self.probe('reacquire-2', target=.041, contact=.0435)
        self.assertEqual(second['status'], 'completed', second.get('receipt'))
        self.assertEqual(second['receipt']['contact_observation']['outcome'], 'settled_contact_candidate')
        self.assertEqual(self.host.grasps.active('left')['probe_ref']['event_id'], 'reacquire-2')
        self.assert_only_left_jaw_frames(2)
        for side in ('left', 'right'):
            self.assertEqual(self.host.device._previous[side]['joints_rad'], self.source['before'][side]['joints_rad'])
            self.assertEqual(self.host.device._previous[side]['pose_m_rad'], self.source['before'][side]['pose_m_rad'])

    def test_three_arrivals_without_contact_end_the_probe_allowance(self):
        for index, target in enumerate((.0455, .042, .0385), 1):
            _, result = self.probe('reacquire-'+str(index), target=target, contact=None)
            self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(self.recovery_state()['phase'], 'exhausted')
        with self.assertRaises((ValueError, RuntimeError)):
            self.service.call('robot_pair_submit_once', self.request('fourth-probe', target=.035))
        self.assertIsNone(self.host.ledger.event('fourth-probe'))
        self.assert_only_left_jaw_frames(3)

    def test_late_rgb_requests_refresh_before_claim_or_episode_then_fresh_probe_runs(self):
        request = self.request('late-reacquisition')
        before_steps = self.host.ledger.status()['steps']
        self.clock.sleep(27.)
        result = self.service.call('robot_pair_submit_once', request)
        self.assertEqual(result['status'], 'refresh_required')
        self.assertEqual(result['stage'], 'before_claim')
        self.assertFalse(result['event_claimed'])
        self.assertEqual(result['steps_consumed'], 0)
        self.assertEqual(result['hardware_commands_sent'], 0)
        self.assertFalse(result['fault_latched'])
        self.assertAlmostEqual(result['remaining_rgb_window_s'], 3., places=5)
        self.assertIsNone(self.host.ledger.event(request['event_id']))
        self.assertIsNone(self.host.grasps.active('left'))
        self.assertIsNone(self.host.active_event_id)
        self.assertFalse(self.host.status()['fault_latched'])
        self.assertEqual(self.host.ledger.status()['steps'], before_steps)
        self.assert_only_left_jaw_frames(0)
        _, fresh = self.probe('fresh-after-refresh')
        self.assertEqual(fresh['status'], 'completed', fresh.get('receipt'))
        self.assertEqual(fresh['receipt']['contact_observation']['outcome'], 'settled_contact_candidate')
        self.assert_only_left_jaw_frames(1)


class ContactReleaseWindowReviewTests(HostGraspFixture):
    def test_late_release_refresh_preserves_candidate_and_residual_target(self):
        self.candidate('left')
        episode = copy.deepcopy(self.host.grasps.active('left'))
        residual = copy.deepcopy(self.host.unresolved_gripper_probe)
        device_grasps = copy.deepcopy(self.host.grasp_states)
        before_steps = self.host.ledger.status()['steps']
        frames = {side:len(robot.sent) for side,robot in self.robots.items()}
        scene = self.observe()
        self.clock.sleep(27.)
        result = self.service.call('robot_pair_submit_once', dict(
            event_id='late-supported-release', observation_id=scene['observation_id'],
            peer_receipt_id=scene['peer_receipts']['right']['receipt_id'],
            arm='left', kind='gripper', operation='release_retreat', width_m=.05,
            release_support_observation='Synthetic original support remains present.',
            release_support_relation='independent_support_present'))
        self.assertEqual(result['status'], 'refresh_required')
        self.assertEqual(result['stage'], 'before_claim')
        self.assertFalse(result['event_claimed'])
        self.assertEqual(result['hardware_commands_sent'], 0)
        self.assertFalse(result['fault_latched'])
        self.assertIsNone(self.host.ledger.event('late-supported-release'))
        self.assertEqual(self.host.grasps.active('left'), episode)
        self.assertEqual(self.host.unresolved_gripper_probe, residual)
        self.assertEqual(self.host.grasp_states, device_grasps)
        self.assertEqual(self.host.ledger.status()['steps'], before_steps)
        self.assertFalse(self.host.status()['fault_latched'])
        self.assertEqual({side:len(robot.sent) for side,robot in self.robots.items()}, frames)
