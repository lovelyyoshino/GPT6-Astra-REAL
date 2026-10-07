"""ROS transport is wholly fake; no ROS imports, node, service or hardware I/O."""
import ast
import copy
import importlib
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

from right_pick.fast_policy import parse_response
from right_pick.fast_ros import (ROSRightArm, FastROSError, PINNED, VENDOR_SHA256,
                                FEEDBACK_IDS, check_telemetry, _events, _ROSTransport, _check_command_publishers)
from right_pick.fast_safety import FastSafetyError


def telemetry(now=100., seq=100):
    return dict(source='sdk_receive_raw_frames', can_interface='can1', driver_sha256=VENDOR_SHA256,
                sdk_version='0.6.2', stamps=[now-.01]*14, stamp=now-.005,
                sequence=seq, source_sequence=seq, raw_q=[0]*6, q=[0.]*6,
                pose=[.25, 0., .25, 0., 0., 0.], opening_m=.035, gripper_torque_sdk_units=50,
                ctrl_mode=1, arm_status=0, mode=0, teach_status=0, motion_status=0, fault=0,
                driver_codes=[64]*6, jaw_code=64, enabled=[True]*6,
                active_command=False, driver_accepts_commands=True, failure=None)


def move(x=.26, phase='APPROACH_PEN', speed=3):
    return dict(phase=phase, action='move_eef', confidence=.9,
                arguments=dict(pose_m_rad=[x, 0., .25, 0., 0., 0.], speed_percent=speed, next_phase=None))


def jaw(width=.006):
    return dict(phase='GRASP', action='gripper', confidence=.9,
                arguments=dict(opening_m=width, effort_parameter_nm=.2))


def config():
    return dict(ros=dict(PINNED, command_log='/unused/current_driver.log'),
                physical_limits=dict(workspace_min_m=[.1, -.3, .1], workspace_max_m=[.4, .3, .5],
                    joint_limits_rad=[[-3., 3.]]*6, max_state_age_s=.1,
                    gripper_min_m=0., gripper_max_m=.055, max_effort_parameter_nm=.2,
                    max_translation_step_m=.03, max_rotation_step_rad=.1, max_speed_percent=5))


class FakeTransport:
    """Feedback changes here are state assignment, not simulation or dynamics."""
    def __init__(self):
        self.now = 100.
        self.raw = telemetry()
        self.history = [dict(source='ros_resume_entry', event='read_only_adoption_complete'),
                        dict(event='command_intent', sequence=1,
                             frames=[dict(id=0x159, data_hex='000088b800c80100')])]
        self.calls = []
        self.mode = 'normal'
        self.speed = 3
        self.closed = False
        self.last_envelope = None
        self.receives = 0

    def identity(self):
        self.calls.append('identity')
        return dict(binding_verified=True, source_verified=True, speed_percent=self.speed,
                    adapter_sha256=PINNED['source_sha256'], vendor_sha256=VENDOR_SHA256)

    def events(self):
        self.calls.append('events')
        return copy.deepcopy(self.history)

    def receive(self, timeout_s):
        self.calls.append('receive')
        if self.mode == 'receive_timeout':
            raise TimeoutError('fake missing feedback')
        self.receives += 1
        if self.last_envelope or (self.receives > 1 and self.mode != 'repeated'):
            self.now += 1 if self.last_envelope else .02
            self.raw['stamps'] = [self.now-.01]*14
            self.raw['stamp'] = self.now-.005
            self.raw['sequence'] += 10
            self.raw['source_sequence'] = self.raw['sequence']
            if self.mode == 'no_receipt':
                self.now += 130
                self.raw['stamps'] = [self.now-.01]*14
                self.raw['stamp'] = self.now-.005
            if self.mode == 'post_fault' and self.last_envelope:
                self.raw['arm_status'] = 4
            if self.mode == 'post_site_joint' and self.last_envelope:
                self.raw['raw_q'][0] = 1000
                self.raw['q'][0] = math.pi/180
        return copy.deepcopy(self.raw)

    def _dispatch_once(self, envelope, before_send):
        before_send()
        self.calls.append('dispatch')
        self.last_envelope = copy.deepcopy(envelope)
        if self.mode == 'send_failure':
            raise OSError('fake one send failed; might have been accepted')
        if self.mode == 'no_receipt':
            return
        seq = envelope['expected_command_sequence']
        intent = dict(event='command_intent', sequence=seq, kind=envelope['kind'],
                      frames=copy.deepcopy(envelope['expected_frames']), speed_percent=envelope['speed_percent'],
                      unix_s=self.now, before=copy.deepcopy(self.raw))
        sent = dict(event='command_sent_unconfirmed', sequence=seq, unix_s=self.now+.1,
                    attempted_frames=len(envelope['expected_frames']), socket_send_returns=len(envelope['expected_frames']))
        after = copy.deepcopy(self.raw)
        after['stamps'] = [self.now+.2]*14
        after['sequence'] += 5
        if envelope['kind'] == 'pose':
            self.raw['pose'] = copy.deepcopy(envelope['encoded_target_m_rad'])
        # Closing on an object may not reach requested opening. Test does NOT infer grasp.
        self.raw['opening_m'] = .009 if envelope['kind'] == 'gripper' else envelope['jaw_target_m']
        stable = dict(event='command_observed_stable', sequence=seq, kind=envelope['kind'],
                      unix_s=self.now+.3, after=after)
        if self.mode == 'wrong_sequence':
            intent['sequence'] += 1
        elif self.mode == 'wrong_frame':
            intent['frames'][0]['data_hex'] = 'ff'*8
        elif self.mode == 'partial':
            sent['socket_send_returns'] -= 1
        elif self.mode == 'old_after':
            after['stamps'][0] = self.now-.5
        self.history.extend([intent, sent, stable])

    def close(self):
        self.calls.append('close')
        self.closed = True


class FastROSTests(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.cfg = config()
        self.arm = self.new_arm()

    def new_arm(self, proposal_only=True, cfg=None):
        return ROSRightArm(cfg or self.cfg, proposal_only=proposal_only, transport=self.transport,
                           clock=lambda: self.transport.now, monotonic=lambda: self.transport.now)

    def test_constructor_is_zero_io_and_no_ROS_imports(self):
        self.assertEqual(self.transport.calls, [])
        source = Path(importlib.import_module('right_pick.fast_ros').__file__).read_text()
        tree = ast.parse(source)
        for item in tree.body:
            if isinstance(item, (ast.Import, ast.ImportFrom)):
                names = [n.name for n in item.names] if isinstance(item, ast.Import) else [item.module]
                self.assertFalse(any(n.startswith(('rospy', 'piper_sdk', 'rosgraph', 'piper_msgs')) for n in names))
        with patch.object(_ROSTransport, '_connect', side_effect=AssertionError('unexpected connection')):
            ROSRightArm(self.cfg)

    def test_config_routes_and_hash_cannot_be_repointed(self):
        for key, value in [('can_interface', 'can0'), ('source_sha256', '0'*64), ('pose_topic', '/left/pos_cmd')]:
            cfg = config()
            cfg['ros'][key] = value
            with self.subTest(key=key), self.assertRaises(FastROSError):
                ROSRightArm(cfg)
        with self.assertRaisesRegex(FastROSError, 'command_log'):
            ROSRightArm({})
        self.assertEqual(self.transport.calls, [])

    def test_observe_maps_actual_feedback_without_motion_claim(self):
        result = self.arm.observe()
        self.assertFalse(result['nonphysical'])
        self.assertTrue(result['robot_state']['enabled'])
        self.assertFalse(result['robot_state']['moving'])
        self.assertEqual(result['robot_state']['sampled_at'], 99.99)
        self.assertNotIn('effort', result['gripper_state'])
        self.assertEqual(result['raw_telemetry']['gripper_torque_sdk_units'], 50)
        self.assertEqual(result['provenance']['feedback_ids'], list(FEEDBACK_IDS))
        self.assertEqual(result['provenance']['command_sequence'], 1)
        self.assertFalse(result['provenance']['physical_motion_authorized'])
        self.assertNotIn('dispatch', self.transport.calls)

    def test_raw14_age_skew_future_and_publication_rejected(self):
        variants = []
        for change in (lambda r:r.update(stamps=r['stamps'][:-1]),
                       lambda r:r['stamps'].__setitem__(0, 99.89),
                       lambda r:r['stamps'].__setitem__(0, 100.001),
                       lambda r:r.update(stamp=99.98), lambda r:r.update(stamp=100.001),
                       lambda r:r.update(source_sequence=99), lambda r:r.update(sequence=True)):
            raw = telemetry(); change(raw); variants.append(raw)
        for raw in variants:
            with self.subTest(raw=raw), self.assertRaises(FastROSError):
                check_telemetry(raw, 100.)
        raw = telemetry(); raw['stamps'][0] = 99.900001
        check_telemetry(raw, 100.)

    def test_all_health_flags_and_source_are_required(self):
        for key, value in [('ctrl_mode', 0), ('arm_status', 4), ('fault', 1), ('teach_status', 1),
                           ('motion_status', 1), ('jaw_code', 0), ('driver_codes', [64]*5+[0]),
                           ('enabled', [True]*5+[1]), ('failure', 'latched'), ('active_command', True),
                           ('driver_accepts_commands', False), ('can_interface', 'can0'),
                           ('driver_sha256', 'x'), ('sdk_version', '0.0'), ('source', 'fake'),
                           ('opening_m', float('nan')), ('gripper_torque_sdk_units', None)]:
            raw = telemetry(); raw[key] = value
            with self.subTest(key=key), self.assertRaises(FastROSError):
                check_telemetry(raw, 100.)

    def test_joint_raw_limits_conversion_and_pose_encoding_checked(self):
        for key, value in [('raw_q', [0, -1, 0, 0, 0, 0]), ('raw_q', [0.]*6),
                           ('q', [.001]*6), ('pose', [1.1, 0, .25, 0, 0, 0]),
                           ('opening_m', .071)]:
            raw = telemetry(); raw[key] = value
            with self.subTest(key=key), self.assertRaises(FastROSError):
                check_telemetry(raw, 100.)

    def test_observe_rejects_repeated_and_regressing_raw_feedback(self):
        self.arm.observe()
        self.transport.mode = 'repeated'
        with self.assertRaisesRegex(FastROSError, 'regressed/repeated'):
            self.arm.observe()

    def test_prepare_is_no_io_and_exact_seven_CAN_frame_proposal(self):
        state = self.arm.observe()
        calls = list(self.transport.calls)
        envelope = self.arm.prepare_command(move(.26049), state)
        self.assertEqual(self.transport.calls, calls)
        self.assertEqual(envelope['message']['gripper'], .035)
        self.assertEqual(envelope['message']['mode1'], 0)
        self.assertEqual(envelope['encoded_target_m_rad'][0], .260)
        self.assertEqual([f['id'] for f in envelope['expected_frames']], [0x150, 0x151, 0x152, 0x153, 0x154, 0x159, 0x151])
        self.assertEqual(envelope['expected_frames'][5]['data_hex'], '000088b800c80100')
        self.assertFalse(envelope['numeric_limits_verified'])
        self.assertFalse(envelope['physical_motion_authorized'])

    def test_P_keeps_commanded_contact_width_instead_of_measured_width(self):
        self.transport.history[-1]['frames'][0]['data_hex'] = '0000177000c80100'
        self.transport.raw['opening_m'] = .009
        state = self.arm.observe()
        env = self.arm.prepare_command(move(), state)
        self.assertEqual(env['message']['gripper'], .006)
        self.assertEqual(env['expected_frames'][5]['data_hex'], '0000177000c80100')
        state['provenance']['jaw_command_target_m'] = None
        with self.assertRaisesRegex(FastROSError, 'commanded jaw target'):
            self.arm.prepare_command(move(), state)

    def test_prepare_rotation_rounding_matches_vendor_operation_order(self):
        state = self.arm.observe()
        proposal = move(); proposal['arguments']['pose_m_rad'][3] = .021546089248333334
        envelope = self.arm.prepare_command(proposal, state)
        self.assertAlmostEqual(envelope['encoded_target_m_rad'][3], 1234*math.pi/180000)

    def test_old_model_state_may_encode_but_cannot_authorize(self):
        state = self.arm.observe()
        self.transport.now += 5
        envelope = self.arm.prepare_command(move(), state)
        self.assertTrue(envelope['source_state_stale_at_preparation'])
        self.assertTrue(envelope['requires_fresh_revalidation'])
        with self.assertRaisesRegex(FastROSError, 'age/skew'):
            self.arm.validate(move(), state)

    def test_gripper_exact_fields_effort_and_one_frame(self):
        state = self.arm.observe()
        env = self.arm.prepare_command(jaw(), state)
        self.assertEqual(env['request'], dict(gripper_angle=.006, gripper_effort=.2, gripper_code=1, set_zero=0))
        self.assertEqual(env['expected_frames'], [dict(id=0x159, data_hex='0000177000c80100')])
        bad = jaw(); bad['arguments']['effort_parameter_nm'] = .3
        with self.assertRaises(FastROSError): self.arm.prepare_command(bad, state)
        with self.assertRaises(FastROSError): self.arm.prepare_command(jaw(.056), state)

    def test_chunks_and_phase_forbidden_requests_have_no_route(self):
        state = self.arm.observe()
        proposal = move(phase='INSERT')
        proposal['action'] = 'move_eef_chunk'
        proposal['arguments'] = dict(waypoints=[[.26, 0, .25, 0, 0, 0]], speed_percent=3, next_phase=None)
        with self.assertRaises(FastROSError): self.arm.prepare_command(proposal, state)
        proposal['phase'] = 'APPROACH_PEN'
        with self.assertRaisesRegex(FastROSError, 'no chunks'): self.arm.prepare_command(proposal, state)

    def test_shadow_never_dispatches_or_reports_completion(self):
        result = self.arm.execute(move())
        self.assertEqual(result['status'], 'not_dispatched')
        self.assertTrue(result['shadow'])
        self.assertFalse(result['completed'])
        self.assertNotIn('dispatch', self.transport.calls)

    def test_real_mode_config_booleans_never_unlock_formal_gate(self):
        cfg = dict(config(), allow_motion=True, hold_verified=True, physical_dispatch_commissioned=True)
        arm = self.new_arm(proposal_only=False, cfg=cfg)
        with self.assertRaisesRegex(FastSafetyError, 'Physical execution unavailable'):
            arm.execute(move())
        self.assertNotIn('dispatch', self.transport.calls)
        self.assertIsNone(arm.failure)  # A zero-send policy refusal is not an uncertain firmware goal.

    def test_formal_gate_before_dispatch_even_if_transport_would_send(self):
        arm = self.new_arm(proposal_only=False)
        with self.assertRaises(FastSafetyError): arm.execute(jaw())
        self.assertNotIn('dispatch', self.transport.calls)
        self.assertFalse(arm.authorization_status()['physical_motion_authorized'])

    def fake_qualified(self):
        # Test-only patch; injected fake transport has no ROS API. Not qualification evidence.
        return patch('right_pick.fast_ros.FastSafetyGuard.validate', return_value=None)

    def test_mock_transport_covers_guarded_pose_receipt_and_new_arrival(self):
        arm = self.new_arm(proposal_only=False)
        with self.fake_qualified(): result = arm.execute(move())
        self.assertEqual(self.transport.calls.count('dispatch'), 1)
        self.assertEqual(result['status'], 'command_observed_stable')
        self.assertTrue(result['arm_target_reached'])
        self.assertFalse(result['task_success'])
        self.assertFalse(result['hold_verified'])

    def test_mock_gripper_stability_does_not_claim_width_or_grasp(self):
        arm = self.new_arm(proposal_only=False)
        with self.fake_qualified(): result = arm.execute(jaw())
        self.assertFalse(result['jaw_target_reached'])
        self.assertFalse(result['grasp_verified'])
        self.assertFalse(result['completed'])
        self.assertEqual(self.transport.calls.count('dispatch'), 1)

    def test_unknown_numeric_limits_still_refuse_after_fake_gate(self):
        cfg = config(); cfg.pop('physical_limits')
        arm = self.new_arm(proposal_only=False, cfg=cfg)
        with self.fake_qualified(), self.assertRaisesRegex(FastROSError, 'physical_limits'):
            arm.execute(move())
        self.assertNotIn('dispatch', self.transport.calls)

    def test_numeric_phase_cap_encoded_target_and_live_speed_are_checked(self):
        for target, phase, cfg, speed in [(.254, 'INSERT', config(), 3), (.29, 'APPROACH_PEN', config(), 3),
                                        (.26, 'APPROACH_PEN', config(), 1)]:
            self.transport = FakeTransport(); self.transport.speed = speed
            arm = self.new_arm(proposal_only=False, cfg=cfg)
            with self.subTest(phase=phase, speed=speed), self.fake_qualified(), self.assertRaises(FastROSError):
                arm.execute(move(target, phase=phase))
            self.assertNotIn('dispatch', self.transport.calls)
        cfg = config(); cfg['physical_limits']['workspace_max_m'][0] = .2606
        self.transport = FakeTransport(); arm = self.new_arm(proposal_only=False, cfg=cfg)
        with self.fake_qualified(), self.assertRaisesRegex(FastROSError, 'workspace'):
            arm.execute(move(.26055))  # Encodes to .261, beyond explicit workspace.

    def test_partial_failed_timeout_and_receipt_mismatch_latch_no_retry(self):
        for mode in ('send_failure', 'wrong_sequence', 'wrong_frame', 'partial', 'old_after', 'no_receipt', 'post_fault', 'post_site_joint'):
            self.transport = FakeTransport(); self.transport.mode = mode
            cfg = config()
            if mode == 'post_site_joint': cfg['physical_limits']['joint_limits_rad'][0] = [-.01, .01]
            arm = self.new_arm(proposal_only=False, cfg=cfg)
            with self.subTest(mode=mode), self.fake_qualified():
                with self.assertRaises((FastROSError, OSError, TimeoutError)): arm.execute(move())
                self.assertIsNotNone(arm.failure)
                self.assertTrue(arm.target_uncertain)
                with self.assertRaises(FastROSError): arm.execute(move())
                self.assertEqual(self.transport.calls.count('dispatch'), 1)
                self.assertNotIn('stop', self.transport.calls)
                self.assertNotIn('disable', self.transport.calls)
                self.assertNotIn('reset', self.transport.calls)

    def test_close_during_action_is_not_cancellation(self):
        self.arm._action_lock.acquire()
        with self.assertRaisesRegex(FastROSError, 'not cancelled'): self.arm.close()
        self.arm._action_lock.release()
        self.assertTrue(self.arm.target_uncertain)
        self.assertNotIn('close', self.transport.calls)
        self.arm.close()
        self.assertEqual(self.transport.calls, ['close'])

    def test_failure_latch_is_irreversible_and_close_only_releases_client(self):
        self.arm.latch_failure('first'); self.arm.latch_failure('second')
        self.assertEqual(self.arm.failure, 'first')
        with self.assertRaises(FastROSError): self.arm.observe()
        self.arm.close()
        self.assertEqual(self.transport.calls, ['close'])

    def test_graph_allows_only_this_adapters_registered_pose_publisher(self):
        own = '/right_pick_passive_observer_123'
        pose = PINNED['pose_topic']
        graph = [(PINNED['telemetry_topic'], [PINNED['driver_node']]), (pose, [own])]
        self.assertEqual(_check_command_publishers(graph, PINNED, own, True), {pose: [own]})
        for pubs, owned in ((graph, False), ([(pose, [own, '/other'])], True),
                            ([('/piper/right/joint_cmd', [own])], True),
                            ([('/piper/right/enable_flag', ['/other'])], False)):
            with self.subTest(pubs=pubs, owned=owned), self.assertRaises(FastROSError):
                _check_command_publishers(pubs, PINNED, own, owned)

    def test_private_ROS_topic_route_publishes_once_after_last_moment_check(self):
        state = self.arm.observe()
        env = self.arm.prepare_command(move(), state)
        transport = _ROSTransport(dict(PINNED))
        calls = []
        class Publisher:
            def get_num_connections(self): return 1
            def publish(self, msg): calls.append(('publish', msg))
        pub = Publisher()
        transport.rospy = types.SimpleNamespace(Publisher=lambda *a, **k: pub)
        module = types.ModuleType('piper_msgs.msg')
        module.PosCmd = lambda **fields: fields
        with patch.dict('sys.modules', {'piper_msgs.msg': module}):
            transport._dispatch_once(env, lambda: calls.append(('recheck', None)))
        self.assertEqual([c[0] for c in calls], ['recheck', 'publish'])
        self.assertEqual(calls[1][1], env['message'])
        self.assertEqual(transport.publishers, [pub])

    def test_private_ROS_service_is_one_call_and_checks_before_request(self):
        state = self.arm.observe()
        env = self.arm.prepare_command(jaw(), state)
        transport = _ROSTransport(dict(PINNED))
        calls = []
        def service(**fields):
            calls.append(('request', fields))
            return types.SimpleNamespace(status=True)
        transport.rospy = types.SimpleNamespace(
            wait_for_service=lambda *a, **k: calls.append(('discovery', None)),
            ServiceProxy=lambda *a, **k: service)
        module = types.ModuleType('piper_msgs.srv'); module.Gripper = object()
        with patch.dict('sys.modules', {'piper_msgs.srv': module}):
            transport._dispatch_once(env, lambda: calls.append(('recheck', None)))
        self.assertEqual([c[0] for c in calls], ['discovery', 'recheck', 'request'])
        self.assertEqual(calls[-1][1], env['request'])

    def test_last_moment_check_failure_prevents_private_publish(self):
        env = self.arm.prepare_command(move(), self.arm.observe())
        transport = _ROSTransport(dict(PINNED)); sent = []
        pub = types.SimpleNamespace(get_num_connections=lambda: 1, publish=lambda x: sent.append(x))
        transport.rospy = types.SimpleNamespace(Publisher=lambda *a, **k: pub)
        module = types.ModuleType('piper_msgs.msg'); module.PosCmd = lambda **fields: fields
        def reject(): raise FastROSError('stale after subscriber setup')
        with patch.dict('sys.modules', {'piper_msgs.msg': module}), self.assertRaises(FastROSError):
            transport._dispatch_once(env, reject)
        self.assertEqual(sent, [])

    def test_log_parser_keeps_only_exact_driver_JSON_and_requires_adoption(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'log'
            path.write_text('[INFO] hello\n{"source":"ros_resume_entry","event":"read_only_adoption_complete"}\n{"incomplete"')
            self.assertEqual(len(_events(path)), 1)
            path.write_text('{"source":"other","event":"read_only_adoption_complete"}\n')
            with self.assertRaises(FastROSError): _events(path)


if __name__ == '__main__':
    unittest.main()
