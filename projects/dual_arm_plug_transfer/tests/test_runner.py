"""Offline lifecycle tests: fake service/cameras, real metadata admission code."""
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runner


class FakeService:
    def __init__(self):
        self.calls = []
        self.shutdowns = 0
        self.answer = None
        self.error = None

    def call(self, tool, arguments):
        self.calls.append((tool, copy.deepcopy(arguments)))
        if self.error:
            raise self.error
        if self.answer is not None:
            return copy.deepcopy(self.answer)
        if tool == 'robot_pair_submit_once':
            return {'status': 'pending', 'event_id': arguments['event_id'], 'replayed': False}
        if tool == 'robot_pair_close':
            return {'status': 'closed', 'cleanup': {}, 'fault_latched': False}
        return {'status': 'owned', 'run_id': 'same-run', 'owner': 'same-owner',
                'open': True, 'ledger': {}, 'fault_latched': False}

    def shutdown(self):
        self.shutdowns += 1


class FakeRig:
    def __init__(self):
        self.calls = self.closed = 0
        self.close_errors = []

    def capture(self, directory):
        self.calls += 1
        folder = Path(directory) / ('capture_' + str(self.calls))
        folder.mkdir(parents=True)
        now = time.time()
        result = {'capture_id': folder.name, 'cameras': {}, 'metadata_path': str(folder / 'observation.json')}
        for view, key in runner.VIEWS.items():
            image = folder / (view + '.png')
            image.write_bytes(b'fake offline RGB bytes; not a physical scene')
            result['cameras'][view] = {'serial': runner.CAMERAS[key], 'rgb_path': str(image),
                                      'frame_number': self.calls, 'host_received_at': now, 'depth_enabled': False}
        Path(result['metadata_path']).write_text(json.dumps(result))
        return result

    def close(self):
        self.closed += 1


class RunnerTests(unittest.TestCase):
    def setUp(self):
        # Offline fixtures belong to this checkout, even when it is a temporary
        # publication tree. Do not alter the live runner's canonical binding.
        bundle = runner.PROJECT.parents[1]
        for name, value in {
            'BUNDLE': bundle,
            'CANONICAL': bundle / 'projects/piperx_cloth_demo',
            'LEDGER': bundle / 'projects/piperx_cloth_demo/runs/pair_sessions.sqlite',
            'RECIPE': bundle / 'tasks/plug_transfer_left.json',
        }.items():
            patcher = patch.object(runner, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        # Files must be within the bundle for the real service metadata gate.
        self.temp = tempfile.TemporaryDirectory(prefix='offline-test-', dir=runner.PROJECT)
        self.addCleanup(self.temp.cleanup)
        self.backend, self.rig = FakeService(), FakeRig()
        self.make_service, self.make_rig = Mock(return_value=self.backend), Mock(return_value=self.rig)
        self.session = runner.Session(Path(self.temp.name) / 'session', context='offline test',
                                      make_service=self.make_service, make_rig=self.make_rig)
        self.addCleanup(self.session.close)

    def request(self, identifier, op, arguments=None):
        return self.session.request({'id': identifier, 'op': op, 'arguments': arguments or {}})

    def test_construction_and_note_do_not_construct_devices_or_send(self):
        self.make_service.assert_not_called()
        self.make_rig.assert_not_called()
        self.request('note-1', 'note', {'text': 'No object result has been observed.'})
        self.make_service.assert_not_called()
        self.make_rig.assert_not_called()
        task = json.loads((self.session.directory / 'task.json').read_text())
        self.assertFalse(task['task_completed'])
        self.assertEqual(task['canonical_ledger'], str(runner.LEDGER))

    def test_exact_forwarding_one_retained_service_and_no_automatic_preparation(self):
        opening = {'run_id': 'already-authorized', 'task_id': 'plug_transfer_left',
                   'workspace_clearance_statement': 'actual statement', 'connection_mode': 'prepare'}
        self.request('open', 'robot_pair_open', opening)
        arguments = {'event_id': 'chosen-event', 'observation_id': 'current-scene',
                     'peer_receipt_id': 'current-peer', 'arm': 'right', 'kind': 'joint',
                     'operation': 'align', 'target_joints_rad': [0.1] * 6,
                     'admission_mode': 'rgb_supervised', 'unloaded_observation': 'current description',
                     'corridor_observation': 'current corridor'}
        original = copy.deepcopy(arguments)
        self.request('segment', 'robot_pair_submit_once', arguments)
        self.request('status', 'robot_pair_status', {'event_id': 'chosen-event'})
        self.make_service.assert_called_once_with()
        self.assertEqual(self.backend.calls, [('robot_pair_open', opening),
                         ('robot_pair_submit_once', original), ('robot_pair_status', {'event_id': 'chosen-event'})])
        self.assertEqual(arguments, original)
        self.make_rig.assert_not_called()

    def test_fault_does_not_retry_clear_or_change_owner(self):
        self.request('open', 'robot_pair_open', {'run_id': 'same-run', 'task_id': 'plug_transfer_left'})
        self.backend.answer = {'status': 'fault', 'receipt': {'ok': False, 'fault_latched': True}}
        self.request('step', 'robot_pair_submit_once', {'event_id': 'uncertain'})
        with self.assertRaisesRegex(ValueError, 'Fault'):
            self.request('repeat', 'robot_pair_submit_once', {'event_id': 'replacement'})
        with self.assertRaisesRegex(ValueError, 'already attempted'):
            self.request('reopen', 'robot_pair_open', {'run_id': 'other-run'})
        self.backend.answer = None
        self.request('status', 'robot_pair_status')
        self.request('close', 'robot_pair_close')
        with self.assertRaises(ValueError):
            self.request('reopen-after-close', 'robot_pair_open', {'run_id': 'other-run'})
        self.assertEqual([n for n, _ in self.backend.calls],
                         ['robot_pair_open', 'robot_pair_submit_once', 'robot_pair_status', 'robot_pair_close'])
        self.assertTrue(self.session.fault_observed)
        self.make_service.assert_called_once_with()

    def test_backend_exception_is_logged_without_retry_and_status_remains_available(self):
        self.backend.error = RuntimeError('CAN outcome uncertain')
        result = self.request('open', 'robot_pair_open', {'run_id': 'same-run', 'task_id': 'plug_transfer_left'})
        self.assertIn('CAN outcome uncertain', result['error'])
        self.assertFalse(result['automatic_retry'])
        self.backend.error = None
        self.request('status', 'robot_pair_status')
        self.assertEqual(len(self.backend.calls), 2)
        rows = [json.loads(line) for line in (self.session.directory / 'session.jsonl').read_text().splitlines()]
        self.assertEqual([row['kind'] for row in rows[1:3]], ['request', 'request_error'])

    def test_allowlist_duplicates_and_malformed_json_never_reach_devices(self):
        for op in ('robot_startup_arms', 'robot_home_arm', 'robot_clear_fault', 'robot_read_state'):
            with self.assertRaises(ValueError):
                self.request('forbidden', op)
        with self.assertRaises(ValueError):
            runner.parse('{"op":"note","op":"robot_pair_open"}')
        with self.assertRaises(ValueError):
            runner.parse('{"value":NaN}')
        self.request('note', 'note', {'text': 'one'})
        with self.assertRaises(ValueError):
            self.request('note', 'note', {'text': 'two'})
        self.make_service.assert_not_called()
        self.make_rig.assert_not_called()

    def test_existing_contact_observation_forwarding_and_response_handling(self):
        self.request('open', 'robot_pair_open', {'run_id': 'same-run', 'task_id': 'plug_transfer_left'})
        arguments = {'event_id': 'contact-observation', 'observation_id': 'fresh-scene',
                     'object_id': 'white_charger', 'visual_description': 'Fresh scene and onsite contact source.',
                     'contact_relation': 'bilateral_finger_contact', 'support_relation': 'independent_support_present'}
        tool = 'robot_pair_observe_supported_contact'
        pending = {'status': 'pending', 'event_id': 'contact-observation', 'receipt': None}
        self.backend.answer = pending
        self.request('observe-contact', tool, arguments)
        self.assertEqual(self.backend.calls[-1], (tool, arguments))
        self.assertFalse(self.session.fault_observed)
        for result in (pending, {'status': 'completed', 'event_id': 'contact-observation', 'receipt': {'ok': True}},
                       {'status': 'refresh_required', 'hardware_commands_sent': 0, 'event_claimed': False,
                        'steps_consumed': 0, 'fault_latched': False}):
            self.assertFalse(runner.Session.failed(result, tool))
        for result in ({'status': 'completed', 'event_id': 'contact-observation', 'receipt': {'ok': False}},
                       {'status': 'invented', 'ok': True}, {}):
            self.assertTrue(runner.Session.failed(result, tool))

    def test_wrong_task_cannot_construct_the_canonical_service(self):
        for value in (None, 'unrelated-task'):
            with self.assertRaisesRegex(ValueError, 'task_id'):
                self.request('wrong-task', 'robot_pair_open', {'run_id': 'same-run', 'task_id': value})
        self.make_service.assert_not_called()
        self.assertFalse(self.session.open_attempted)

    def test_unknown_results_latch_but_known_preparation_and_refresh_do_not(self):
        self.assertTrue(runner.Session.failed({}))
        for value in ({}, {'status': 'invented'}, {'ok': True, 'status': 'invented'}):
            self.assertTrue(runner.Session.failed(value, 'robot_pair_submit_once'))
        self.assertFalse(runner.Session.failed(
            {'ok': False, 'status': 'preparation_required', 'fault_latched': False,
             'hardware_commands_sent': 0, 'requirements': ['jaw'], 'readiness': {}},
            'robot_pair_promote_ready'))
        self.assertFalse(runner.Session.failed(
            {'status': 'refresh_required', 'hardware_commands_sent': 0, 'event_claimed': False,
             'steps_consumed': 0, 'fault_latched': False}, 'robot_pair_submit_once'))
        self.backend.answer = {}
        self.request('open', 'robot_pair_open', {'run_id': 'same-run', 'task_id': 'plug_transfer_left'})
        self.assertTrue(self.session.fault_observed)
        with self.assertRaisesRegex(ValueError, 'Fault'):
            self.request('step', 'robot_pair_submit_once', {'event_id': 'no-bypass'})
        self.assertEqual(len(self.backend.calls), 1)

    def test_completed_missing_preparation_is_known_but_nested_faults_are_not(self):
        missing = {'ok': False, 'status': 'preparation_required', 'hardware_commands_sent': 0,
                   'fault_latched': False, 'requirements': ['jaw'], 'readiness': {},
                   'execution_mode': 'prepare_gripper'}
        for mode in ('prepare_gripper', 'inspect_joint_limits'):
            missing['execution_mode'] = mode
            result = {'status': 'completed', 'event_id': 'e', 'receipt': copy.deepcopy(missing)}
            self.assertFalse(runner.Session.failed(result, 'robot_pair_status'))
            self.assertFalse(runner.Session.failed(result, 'robot_pair_' + mode))
            other = 'inspect_joint_limits' if mode == 'prepare_gripper' else 'prepare_gripper'
            self.assertTrue(runner.Session.failed(result, 'robot_pair_' + other))
            self.assertTrue(runner.Session.failed(result, 'robot_pair_submit_once'))
        for negative in ({'ok': False}, {'error': 'uncertain'}, {'fault_latched': True},
                         {'isError': True}, {'errors': ['unknown']}, {'guard_violations': ['unexpected']},
                         {'ok': True, 'status': 'pair_device_fault'}, {'status': 'fault'}):
            result = {'status': 'completed', 'event_id': 'e',
                      'receipt': {'ok': True, 'device_receipt': negative}}
            self.assertTrue(runner.Session.failed(result, 'robot_pair_status'), negative)
        missing['device_receipt'] = {'ok': False, 'error': 'not a known shortfall'}
        self.assertTrue(runner.Session.failed({'status': 'completed', 'event_id': 'e', 'receipt': missing},
                                             'robot_pair_status'))

    def test_explicit_camera_retained_and_metadata_accepted_by_canonical_observe(self):
        first = self.request('rgb-1', 'capture')['result']
        second = self.request('rgb-2', 'capture')['result']
        self.make_rig.assert_called_once_with()
        self.assertEqual(self.rig.calls, 2)
        self.make_service.assert_not_called()  # Capture never implicitly observes or opens the pair.
        sys.path.insert(0, str(runner.CANONICAL))
        from robot_tools.service import ToolService
        from robot_tools.pair_host import PairHost
        receiver = types.SimpleNamespace(profile={'cameras': runner.CAMERAS}, clock=time.time,
                                         frame_numbers={}, rgb_not_before=0)
        observed = []
        def observe(rgb, *, saved_rgb_evidence):
            numbers, stamp = PairHost._rgb(receiver, rgb)
            receiver.frame_numbers = numbers
            observed.append((rgb, saved_rgb_evidence))
            return {'status': 'offline_metadata_checked', 'rgb_received_at': stamp}
        service = ToolService(runner.CANONICAL)
        service.pair_host = types.SimpleNamespace(observe=observe)
        with patch('socket.socket', side_effect=AssertionError('No real sockets in offline tests')):
            for result in (first, second):
                checked = service.pair_observe(result['rgb_observation_path'])
                self.assertEqual(checked['status'], 'offline_metadata_checked')
        self.assertEqual(receiver.frame_numbers, {'front': 2, 'left_hand': 2, 'right_hand': 2})
        self.assertTrue(all(len(item['artifact_sha256']) == 64 for item in observed[-1][1].values()))

    def test_eof_uses_canonical_shutdown_once_and_logs_unknown_stop(self):
        source = io.StringIO(runner.encode({'id': 'open', 'op': 'robot_pair_open',
                                           'arguments': {'run_id': 'same-run', 'task_id': 'plug_transfer_left'}}) + '\n')
        output = io.StringIO()
        runner.serve(self.session, source, output)
        self.assertEqual(self.backend.shutdowns, 1)
        self.assertEqual(len(self.backend.calls), 1)
        self.assertEqual(json.loads(output.getvalue().splitlines()[0])['automatic_tool_calls'], 0)
        last = json.loads((self.session.directory / 'session.jsonl').read_text().splitlines()[-1])
        self.assertEqual(last['kind'], 'session_ended')
        self.assertIsNone(last['physical_stop_verified'])
        self.assertIsNone(last['task_completed'])
        self.session.close()
        self.assertEqual(self.backend.shutdowns, 1)


if __name__ == '__main__':
    unittest.main()
