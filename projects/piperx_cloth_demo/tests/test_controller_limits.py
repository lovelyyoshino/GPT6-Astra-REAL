"""No physical sockets: fixed manufacturer configuration and failure boundaries."""
import copy
import errno
import json
import math
import sqlite3
import time
import unittest
from pathlib import Path
import tempfile
from unittest.mock import patch, Mock

from robot_tools import controller_limits as maintenance, pair_device, pair_limits, pair_preparation, linear_hold
from robot_tools import single_supervised_actions, supervised_actions
from test_pair_limits import QueryRobot
from test_single_gripper_prepare import SingleGripperFixture
from test_joint_limits import reply_bytes
import test_sdk_joint_limit_semantics as sdk_fixture


class ConfigRobot(QueryRobot):
    def __init__(self, side, clock):
        super().__init__(side, clock)
        self.limits = {j: [-1800, 1800, 300] for j in range(1, 7)}
        self.limits[4], self.limits[5] = [-1000, 1000, 300], [-700, 700, 300]
        self.raw = lambda j: reply_bytes(j, *self.limits[j])
        self.accept_config = True
        self.config_error = None
        self.after_config = None
        self.emit_ack = True
        self.ack_payload = b'\x74' + bytes(7)
        self._MSG_MotorAngleLimitMaxSpdSet = lambda j, hi, lo, speed: self.can.Message(
            arbitration_id=0x474, is_extended_id=False,
            data=bytes([j]) + hi.to_bytes(2, 'big', signed=True) + lo.to_bytes(2, 'big', signed=True)
            + speed.to_bytes(2, 'big') + b'\0')

    def _send_msg(self, frame):
        self.comm.send(self.frame_transform(frame))
        if self.duplicate and frame.arbitration_id == 0x474:
            self.comm.send(frame)

    def _bus_send(self, frame):
        if frame.arbitration_id != 0x474:
            return super()._bus_send(frame)
        if self.config_error:
            raise self.config_error
        if self.send_error:
            raise self.send_error
        self.sent.append(copy.deepcopy(frame))
        if self.accept_config:
            j = frame.data[0]
            self.limits[j][:2] = [int.from_bytes(frame.data[3:5], 'big', signed=True),
                                  int.from_bytes(frame.data[1:3], 'big', signed=True)]
        if self.after_config:
            self.after_config(self, frame)
        if self.emit_ack:
            self.callback(self.can.Message(arbitration_id=0x476, is_extended_id=False,
                timestamp=self.clock.time(), data=self.ack_payload))


class ControllerMaintenanceTests(SingleGripperFixture):
    def setUp(self):
        super().setUp()
        for module in (maintenance, pair_device, pair_limits, pair_preparation, linear_hold,
                       single_supervised_actions, supervised_actions):
            self.stack.enter_context(patch.object(module, 'time', self.clock))
        self.profile['sdk_commit_audited'] = maintenance.SDK_COMMIT
        self.robots = {s: ConfigRobot(s, self.clock) for s in ('left', 'right')}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)), lambda: None)
        self.addCleanup(self.device.close)
        self.hook = lambda robot, state: state.update(joints_rad=[0, .2, -.2, 0, math.radians(73.5), 0])

    def open(self):
        return self.device.connect_for_preparation()

    def writes(self):
        return [(s, f) for s, r in self.robots.items() for f in r.sent if f.arbitration_id == 0x474]

    def run_maintenance(self):
        self.open()
        return maintenance.reconcile(self.device)

    def test_exact_manufacturer_writes_readbacks_preserve_speed_and_no_task_grant(self):
        result = self.run_maintenance()
        self.assertTrue(result['ok'], result)
        self.assertEqual(len(self.writes()), 4)
        self.assertEqual([bytes(f.data).hex() for _, f in self.writes()],
                         ['04037afc867fff00', '05037afc867fff00'] * 2)
        self.assertTrue(all(f.arbitration_id in (0x472, 0x474) for r in self.robots.values() for f in r.sent))
        self.assertEqual(sum(len(r.sent) for r in self.robots.values()), 32)
        self.assertFalse(self.device._task_ready)
        for side in ('left', 'right'):
            for key in ('joints_rad', 'pose_m_rad'):
                self.assertEqual(result['before_arms'][side][key], result['after_arms'][side][key])
            self.assertEqual(result['before_arms'][side]['gripper']['width_m'], result['after_arms'][side]['gripper']['width_m'])
        for side in ('left', 'right'):
            for j in range(1, 7):
                self.assertEqual(result['joint_limits'][side][str(j)]['raw_max_joint_spd'], 300)

    def test_already_correct_limits_are_only_read_not_rewritten(self):
        for robot in self.robots.values():
            for j in (4, 5):
                robot.limits[j][:2] = [-890, 890]
        result = self.run_maintenance()
        self.assertTrue(result['ok'], result)
        self.assertEqual(self.writes(), [])
        self.assertEqual(sum(len(r.sent) for r in self.robots.values()), 24)

    def test_unexpected_installed_limit_blocks_all_writes(self):
        self.robots['right'].limits[5][:2] = [-600, 600]
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(self.writes(), [])

    def test_refused_firmware_write_is_not_repeated_or_rolled_back(self):
        self.robots['left'].accept_config = False
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(len(self.writes()), 1)
        self.assertIn('did not confirm', str(result['errors']))

    def test_unexpected_speed_change_stops_remaining_writes(self):
        self.robots['left'].after_config = lambda r, f: r.limits[f.data[0]].__setitem__(2, 301)
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(len(self.writes()), 1)

    def test_wrong_joint_or_velocity_payload_cannot_reach_bus(self):
        self.open()
        def wrong(frame):
            if frame.arbitration_id == 0x474:
                frame.data[5:7] = b'\x01\x2c'
            return frame
        self.robots['left'].frame_transform = wrong
        result = maintenance.reconcile(self.device)
        self.assertFalse(result['ok'])
        self.assertEqual(self.writes(), [])

    def test_duplicate_config_is_blocked_no_second_joint(self):
        self.open()
        self.robots['left'].after_config = lambda r, f: setattr(r, 'duplicate', True)
        result = maintenance.reconcile(self.device)
        self.assertFalse(result['ok'])
        self.assertEqual(len(self.writes()), 1)

    def test_peer_drift_after_write_stops_before_next_write(self):
        self.open()
        def after(robot, frame):
            self.hook = lambda r, s: s.update(joints_rad=[0, .2, -.2, .02 if r.side == 'right' else 0, math.radians(73.5), 0])
        self.robots['left'].after_config = after
        result = maintenance.reconcile(self.device)
        self.assertFalse(result['ok'])
        self.assertEqual(len(self.writes()), 1)

    def test_missing_final_readback_stops_without_retry(self):
        self.robots['left'].after_config = lambda r, f: setattr(r, 'no_reply_joint', f.data[0])
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(len(self.writes()), 1)

    def test_configuration_send_exception_remains_uncertain_no_retry(self):
        self.robots['left'].config_error = OSError(errno.ENOBUFS, 'send buffer full')
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(self.writes(), [])
        self.assertEqual(len(result['writes']), 1)
        self.assertEqual(result['writes'][0]['outcome'], 'uncertain')

    def test_configuration_intent_journal_failure_prevents_write(self):
        self.open()
        original = self.device._action.journal
        def journal(event, data):
            if event == 'manufacturer_limit_write_intent':
                raise OSError('durable storage unavailable')
            return original(event, data)
        self.device._action.journal = journal
        result = maintenance.reconcile(self.device)
        self.assertFalse(result['ok'])
        self.assertEqual(self.writes(), [])

    def test_wrong_model_refuses_before_any_query(self):
        self.open()
        self.device._action.profile['arms']['left']['model'] = 'piper'
        result = maintenance.reconcile(self.device)
        self.assertFalse(result['ok'])
        self.assertEqual(sum(len(r.sent) for r in self.robots.values()), 0)

    def test_unknown_nominal_90_degree_pose_refuses_zero_tx(self):
        self.hook = lambda r, s: s.update(joints_rad=[0, .2, -.2, 0, math.radians(90), 0])
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(sum(len(r.sent) for r in self.robots.values()), 0)

    def test_source_hash_mutation_refuses(self):
        self.open()
        with patch.object(maintenance, 'KNOWN_OFFICIAL_CONSTANTS', {maintenance.SDK_COMMIT: '0' * 64}):
            result = maintenance.reconcile(self.device)
        self.assertFalse(result['ok'])
        self.assertEqual(self.writes(), [])

    def test_missing_configuration_ack_never_queries_or_retries_after_write(self):
        self.robots['left'].emit_ack = False
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertIn('ACK', str(result['errors']))
        self.assertEqual(len(self.writes()), 1)
        self.assertEqual([f.arbitration_id for f in self.robots['left'].sent], [0x472]*6 + [0x474])

    def test_wrong_ack_cannot_trigger_followup_query(self):
        self.robots['left'].ack_payload = b'\x75' + bytes(7)
        result = self.run_maintenance()
        self.assertFalse(result['ok'])
        self.assertEqual(len(self.robots['left'].sent), 7)

    def test_busy_controller_waits_rx_only_then_checks_original_freshness(self):
        original_hook = self.hook
        def busy(robot, frame):
            stamp, end = self.clock.time(), self.clock.time() + .2
            def hook(r, state):
                original_hook(r, state)
                if r.side == robot.side and self.clock.time() < end:
                    state['fragment_timestamps_s'] = {k: stamp-.01 for k in state['fragment_timestamps_s']}
            self.hook = hook
        self.robots['left'].after_config = busy
        result = self.run_maintenance()
        self.assertTrue(result['ok'], result)
        self.assertEqual(len(self.writes()), 4)
        row = result['writes'][0]
        self.assertGreaterEqual(row['ack_wait']['finished_at']-row['returned_at'], .2)


class ManufacturerConfigurationEncodingTests(unittest.TestCase):
    setUp = sdk_fixture.SDKJointLimitSemanticsTests.setUp
    def test_real_piper_x_manufacturer_message_encodes_no_speed_change_or_target(self):
        for j in (4, 5):
            self.robot._send_msg(self.robot._MSG_MotorAngleLimitMaxSpdSet(j, 890, -890, 0x7fff))
        self.assertEqual([f.arbitration_id for f in self.frames], [0x474, 0x474])
        self.assertEqual([bytes(f.data).hex() for f in self.frames],
                         ['04037afc867fff00', '05037afc867fff00'])
        self.assertIsNone(self.robot.get_arm_status())


class MaintenanceAccountingTests(unittest.TestCase):
    def setUp(self):
        from test_backend import PROFILE
        from robot_tools.pair_ledger import PairLedger
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'runs').mkdir()
        (self.root / 'configs').mkdir()
        self.profile = copy.deepcopy(PROFILE)
        self.profile['sdk_commit_audited'] = maintenance.SDK_COMMIT
        self.profile['cameras'] = {'front': 'test-front', 'left_wrist': 'test-left', 'right_wrist': 'test-right'}
        (self.root / 'configs/robot.json').write_text(json.dumps(self.profile))
        self.contract = {'task': {'task_id': 'plug_transfer_left', 'roles': {'left': 'task', 'right': 'task'}},
                         'arms': self.profile['arms'], 'cameras': self.profile['cameras'],
                         'sdk_commit_audited': maintenance.SDK_COMMIT, 'code': {}}
        self.ledger = PairLedger(self.root / 'runs/pair_sessions.sqlite', 'task', self.contract)
        with sqlite3.connect(self.ledger.path) as db:
            db.execute('CREATE TABLE pair_grasp_episodes(run_id TEXT, state_json TEXT)')
        self.request = {'event_id': 'manufacturer-limits', 'authorization': '按厂家Piper X参数调整控制器限位',
                        'passive_paths': {}, 'rgb_observation': 'image', 'visual_observation': 'two empty jaws'}
        self.evidence = {'rgb': {'images': {s: {'host_received_at': time.time()} for s in ('front', 'left_hand', 'right_hand')}}}
        self.device = Mock()
        self.device.close.return_value = {'requires_fault_latch': False, 'physical_stop_verified': None}
        self.factory = Mock(side_effect=self.make_device)
        for target, kwargs in (
            ('socket.socket', {'side_effect': AssertionError('Physical sockets forbidden')}),
            ('robot_tools.controller_limits.check_processes', {}),
            ('robot_tools.controller_limits._lock_roots', {'return_value': [self.root]}),
            ('robot_tools.controller_limits._observations', {'return_value': self.evidence}),
            ('robot_tools.controller_limits.GuardedPairDevice', {'new': self.factory})):
            p = patch(target, **kwargs); p.start(); self.addCleanup(p.stop)

    def make_device(self, profile, journal, guard):
        # The write reservation must already be durable before opening CAN.
        event = self.ledger.event(self.request['event_id'])
        self.assertEqual(event['status'], 'pending')
        self.assertEqual(event['payload']['kind'], 'manufacturer_configuration')
        self.assertIn('controller_limits.py', event['payload']['maintenance_code_sha256'])
        guard()
        return self.device

    def run_with(self, receipt):
        with patch.object(maintenance, 'reconcile', return_value=receipt):
            return maintenance.execute(self.root, self.request)

    def test_claimed_before_device_counts_existing_budget_and_cleanly_detaches(self):
        before = self.ledger.peek_status()
        result = self.run_with({'ok': True})
        self.assertTrue(result['ok'])
        after = self.ledger.peek_status()
        self.assertEqual(after['steps'], before['steps'] + 1)
        self.assertEqual(after['started_at'], before['started_at'])
        self.assertEqual(after['contract'], before['contract'])
        self.assertEqual(after['status'], 'detached')
        self.device.close.assert_called_once()

    def test_failed_configuration_latches_canonical_task_and_blocks_new_id(self):
        result = self.run_with({'ok': False, 'error': 'configuration readback differs'})
        self.assertFalse(result['ok'])
        self.assertTrue(self.ledger.peek_status()['fault_latched'])
        self.request['event_id'] = 'second'
        with self.assertRaisesRegex(RuntimeError, 'cleanly detached'):
            self.run_with({'ok': True})
        self.factory.assert_called_once()

    def test_completed_id_cannot_send_again(self):
        self.run_with({'ok': True})
        with self.assertRaisesRegex(RuntimeError, 'already exists'):
            self.run_with({'ok': True})
        self.factory.assert_called_once()

    def test_rgb_expiration_prevents_connection_and_latches_no_retry(self):
        for row in self.evidence['rgb']['images'].values():
            row['host_received_at'] -= 31
        result = self.run_with({'ok': True})
        self.assertFalse(result['ok'])
        self.device.connect_for_preparation.assert_not_called()
        self.assertTrue(self.ledger.peek_status()['fault_latched'])

    def test_existing_owner_blocks_without_opening_or_erasing_history(self):
        self.ledger.claim('existing-owner')
        with self.assertRaisesRegex(RuntimeError, 'cleanly detached'):
            self.run_with({'ok': True})
        self.factory.assert_not_called()
        self.assertEqual(self.ledger.peek_status()['owner'], 'existing-owner')

    def test_unresolved_grasp_blocks_configuration(self):
        with sqlite3.connect(self.ledger.path) as db:
            db.execute('INSERT INTO pair_grasp_episodes VALUES(?,?)', ('task', json.dumps({'status': 'grasped'})))
        with self.assertRaisesRegex(RuntimeError, 'Unresolved'):
            self.run_with({'ok': True})
        self.factory.assert_not_called()

    def test_cleanup_failure_latches_even_after_good_readback(self):
        self.device.close.return_value = {'requires_fault_latch': True}
        self.assertFalse(self.run_with({'ok': True})['ok'])
        self.assertTrue(self.ledger.peek_status()['fault_latched'])

    def test_realistic_sdk_enum_receipt_is_identical_in_file_and_ledger(self):
        from enum import IntEnum
        class Mode(IntEnum):
            CAN = 1
        receipt = {'ok': False, 'before_arms': {'left': {'ctrl_mode': Mode.CAN}},
                   'errors': [{'detail': 'feedback skew after configuration'}]}
        result = self.run_with(receipt)
        self.assertEqual(type(result['before_arms']['left']['ctrl_mode']), int)
        self.assertEqual(json.loads(Path(result['record_path']).read_text()), result)
        self.assertEqual(self.ledger.event(self.request['event_id'])['receipt'], result)
        self.assertTrue(self.ledger.peek_status()['fault_latched'])

    def diagnostic_failure(self):
        self.device.close.return_value = {'requires_fault_latch': False,
            'arms': {s: {'status': 'disconnected'} for s in ('left', 'right')},
            'unresolved_gripper_probe': None, 'grasp_states': {'left': None, 'right': None}}
        return self.run_with({'ok': False, 'schema': maintenance.SCHEMA,
            'arm_target_commands_sent': 0, 'gripper_commands_sent': 0, 'guard_violations': []})

    def test_diagnostic_only_queries_and_preserves_entire_ledger(self):
        self.diagnostic_failure()
        original = Path(self.ledger.path).read_bytes()
        def factory(profile, journal, guard):
            guard()
            return self.device
        self.factory.side_effect = factory
        self.device.inspect_joint_limits.return_value = {
            'ok': True, 'joint_limit_queries_sent': 12, 'actuator_commands_sent': 0}
        result = maintenance.inspect_after_configuration_fault(self.root, self.request['event_id'])
        self.assertTrue(result['ok'])
        self.assertTrue(result['canonical_ledger_unchanged'])
        self.assertFalse(result['fault_cleared'])
        self.device.inspect_joint_limits.assert_called_once_with()
        self.assertEqual(Path(self.ledger.path).read_bytes(), original)

    def test_diagnostic_rejects_mutated_saved_result_before_device(self):
        result = self.diagnostic_failure()
        Path(result['record_path']).write_text('{}')
        with self.assertRaisesRegex(RuntimeError, 'must agree'):
            maintenance.inspect_after_configuration_fault(self.root, self.request['event_id'])
        self.factory.assert_called_once()

    def test_diagnostic_rejects_unrelated_motion_receipt(self):
        result = self.run_with({'ok': False, 'schema': maintenance.SCHEMA,
            'arm_target_commands_sent': 1, 'gripper_commands_sent': 0, 'guard_violations': []})
        with self.assertRaisesRegex(RuntimeError, 'Unexpected motion'):
            maintenance.inspect_after_configuration_fault(self.root, self.request['event_id'])
        self.factory.assert_called_once()

    def test_diagnostic_refuses_incomplete_query_without_changing_original_fault(self):
        self.diagnostic_failure()
        before = self.ledger.peek_status()
        self.factory.side_effect = lambda profile, journal, guard: self.device
        self.device.inspect_joint_limits.return_value = {'ok': False, 'joint_limit_queries_sent': 1}
        result = maintenance.inspect_after_configuration_fault(self.root, self.request['event_id'])
        self.assertFalse(result['ok'])
        self.assertEqual(self.ledger.peek_status()['fault'], before['fault'])
