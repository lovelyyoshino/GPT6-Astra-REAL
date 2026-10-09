"""Independent recovery-scope checks on synthetic SQLite, never hardware.

The failed old opening stays in the ledger.  A separately enrolled successor
has its own two-event gate, while continuing the same run and step sequence.
Full enrollment/history auditing is exercised by the admission tests.
"""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import supported_gripper_recovery as recovery
from robot_tools.pair_ledger import PairLedgerError, _execution_scope
import test_supported_gripper_recovery as prior_recovery_tests
from test_execution import healthy_arm
from robot_tools.retention_receipt import measured_anchor


class OpeningContinuationGateReviewTests(unittest.TestCase):
    TABLE = 'pair_supported_gripper_opening_continuations'

    def setUp(self):
        socket_guard = patch('socket.socket', side_effect=AssertionError('Physical I/O forbidden'))
        socket_guard.start()
        self.addCleanup(socket_guard.stop)
        self.db = sqlite3.connect(':memory:')
        self.db.row_factory = sqlite3.Row
        self.addCleanup(self.db.close)
        self.db.executescript('''
            CREATE TABLE pair_rounds(ordinal INTEGER,run_id TEXT,record_json TEXT);
            INSERT INTO pair_rounds VALUES(1,'run','{"proposal":{}}');
            CREATE TABLE pair_supported_gripper_recoveries(
                ordinal INTEGER, run_id TEXT, owner TEXT, round_ordinal INTEGER,
                record_json TEXT, fault_id INTEGER);
            CREATE TABLE pair_supported_gripper_opening_continuations(
                ordinal INTEGER, run_id TEXT, owner TEXT, round_ordinal INTEGER,
                record_json TEXT, fault_id INTEGER);
            CREATE TABLE pair_events(
                run_id TEXT, event_id TEXT, step INTEGER, owner TEXT,
                payload_json TEXT, status TEXT, success INTEGER, receipt_json TEXT);
        ''')
        prior = dict(schema='piper_supported_gripper_recovery_v1', snapshot={},
                     budget=dict(steps=13))
        current = dict(schema=recovery.OPENING_SCHEMA,
                       route='audited_opening_continuation', snapshot={},
                       budget=dict(steps=14),
                       opening_continuation=dict(prior_opening_target_m=.037))
        self.db.execute('INSERT INTO pair_supported_gripper_recoveries VALUES(?,?,?,?,?,?)',
                        (1, 'run', 'failed-owner', 1, json.dumps(dict(proposal=prior)), 33))
        self.db.execute('INSERT INTO '+self.TABLE+' VALUES(?,?,?,?,?,?)',
                        (1, 'run', 'new-owner', 1, json.dumps(dict(proposal=current)), None))
        self.event(14, 'supported_recovery_open', owner='failed-owner', success=0,
                   receipt=dict(ok=False, automatic_retry=False))
        self.old_scope = dict(self.db.execute('SELECT * FROM pair_supported_gripper_recoveries').fetchone())
        self.old_event = dict(self.db.execute('SELECT * FROM pair_events').fetchone())

    def event(self, step, kind, *, owner='new-owner', success=1, status='complete', receipt=None):
        if receipt is None:
            receipt = dict(ok=True, physical_stop_verified=None,
                           hardware_commands_sent=int(kind == 'supported_recovery_open'))
        self.db.execute('INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?)',
                        ('run', 'event-'+str(step), step, owner, json.dumps(dict(kind=kind)),
                         status, success, None if status == 'pending' else json.dumps(receipt)))

    def allowed(self, kind, owner='new-owner', width_m=.040):
        return recovery.check_request(self.db, 'run', owner,
                                      dict(kind=kind, request=dict(width_m=width_m)))

    def assert_blocked(self, *kinds):
        for kind in kinds:
            with self.subTest(kind=kind), self.assertRaises(PairLedgerError):
                self.allowed(kind)

    def test_new_scope_starts_after_failed_event_without_rewriting_it(self):
        state = recovery.runtime(self.db, 'run', 'new-owner')
        self.assertEqual(state['phase'], 'opening_required')
        self.assertEqual(state['events'], [])
        self.allowed('supported_recovery_open')
        self.assert_blocked('query', 'initialization', 'joint', 'gripper',
                            'supported_recovery_confirm')
        self.assertEqual(dict(self.db.execute('SELECT * FROM pair_supported_gripper_recoveries').fetchone()), self.old_scope)
        self.assertEqual(dict(self.db.execute('SELECT * FROM pair_events').fetchone()), self.old_event)

    def test_execution_scope_selects_new_enrollment_and_rejects_foreign_run(self):
        name, key, ordinal, row = _execution_scope(self.db, 'run', writable=True)
        self.assertEqual((name, key, ordinal), (self.TABLE, 'ordinal', 1))
        self.assertEqual(row['owner'], 'new-owner')
        with self.assertRaises(PairLedgerError):
            _execution_scope(self.db, 'foreign-run', writable=True)

    def test_previous_owner_cannot_enter_new_recovery(self):
        with self.assertRaises(PairLedgerError):
            self.allowed('supported_recovery_open', owner='failed-owner')

    def test_previous_target_and_same_encoded_target_cannot_be_replayed(self):
        for width in (.03654, .037, .0370001, .03700049):
            with self.subTest(width=width), self.assertRaises(PairLedgerError):
                self.allowed('supported_recovery_open', width_m=width)
        self.allowed('supported_recovery_open', width_m=.040)

    def test_larger_request_cannot_reuse_rounded_up_previous_frame(self):
        record = json.loads(self.db.execute('SELECT record_json FROM '+self.TABLE).fetchone()[0])
        record['proposal']['opening_continuation']['prior_opening_target_m'] = .0370006
        self.db.execute('UPDATE '+self.TABLE+' SET record_json=?', (json.dumps(record),))
        with self.assertRaises(PairLedgerError):
            self.allowed('supported_recovery_open', width_m=.0370008)

    def test_opening_alone_never_unlocks_ordinary_preparation(self):
        self.event(15, 'supported_recovery_open')
        self.assertEqual(recovery.runtime(self.db, 'run', 'new-owner')['phase'],
                         'confirmation_required')
        self.allowed('supported_recovery_confirm')
        self.assert_blocked('query', 'initialization', 'joint', 'gripper',
                            'supported_recovery_open')
        self.event(16, 'supported_recovery_confirm')
        self.assertEqual(recovery.runtime(self.db, 'run', 'new-owner')['phase'], 'resolved')
        self.allowed('query')
        self.assert_blocked('supported_recovery_open', 'supported_recovery_confirm')

    def test_pending_or_failed_opening_never_retries_or_confirms(self):
        for status, success in (('pending', None), ('complete', 0)):
            with self.subTest(status=status):
                self.db.execute('DELETE FROM pair_events WHERE step>14')
                self.event(15, 'supported_recovery_open', status=status, success=success)
                self.assertEqual(recovery.runtime(self.db, 'run', 'new-owner')['phase'], 'unresolved')
                self.assert_blocked('supported_recovery_open', 'supported_recovery_confirm',
                                    'query', 'joint')

    def test_failed_confirmation_never_unlocks_or_reopens(self):
        self.event(15, 'supported_recovery_open')
        self.event(16, 'supported_recovery_confirm', success=0,
                   receipt=dict(ok=False, hardware_commands_sent=0))
        self.assertEqual(recovery.runtime(self.db, 'run', 'new-owner')['phase'], 'unresolved')
        self.assert_blocked('supported_recovery_open', 'supported_recovery_confirm',
                            'initialization', 'joint')

    def test_peer_owner_or_wrong_first_operation_is_rejected(self):
        for owner, kind in (('failed-owner', 'supported_recovery_open'),
                            ('new-owner', 'query'),
                            ('new-owner', 'supported_recovery_confirm')):
            with self.subTest(owner=owner, kind=kind):
                self.db.execute('DELETE FROM pair_events WHERE step>14')
                self.event(15, kind, owner=owner)
                with self.assertRaises(PairLedgerError):
                    recovery.runtime(self.db, 'run', 'new-owner')

    def test_confirmation_may_not_hide_an_extra_transmission(self):
        self.event(15, 'supported_recovery_open')
        self.event(16, 'supported_recovery_confirm', receipt=dict(
            ok=True, hardware_commands_sent=1, physical_stop_verified=None))
        with self.assertRaises(PairLedgerError):
            recovery.runtime(self.db, 'run', 'new-owner')

    def test_missing_successor_keeps_previous_failure_unresolved(self):
        self.db.execute('DELETE FROM '+self.TABLE)
        self.assertEqual(recovery.runtime(self.db, 'run', 'failed-owner')['phase'], 'unresolved')
        with self.assertRaises(PairLedgerError):
            self.allowed('supported_recovery_open', owner='failed-owner')


class OpeningContinuationSourceReviewTests(unittest.TestCase):
    def setUp(self):
        self.fixture = prior_recovery_tests.SupportedRecoveryTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        failed = self.fixture.f
        self.candidate_anchor = dict(joints_rad=[.1]*6, pose_m_rad=[.4, .1, .2, 0, 0, 0],
                                     width_m=.028)
        failed.receipt['device_receipt']['candidate_measurement']['anchor'] = self.candidate_anchor
        failed.refresh()
        original = dict(event=copy.deepcopy(failed.event), run=copy.deepcopy(failed.run),
                        scope=dict(owner='old-owner'))
        self.proposal = dict(route='audited_opening_continuation',
            proposal_sha256='b'*64,
            snapshot=dict(scope=dict(record_json=json.dumps(dict(proposal=dict(snapshot=original)))),
                event=dict(event_id='later-failed-opening',
                    receipt_json=json.dumps(dict(device_receipt=dict(before=dict(
                        right=dict(joints_rad=[.4]*6, pose_m_rad=[.6, .1, .2, 0, 0, 0]))))))),
            opening_continuation=dict(prior_opening_target_m=.033,
                residual_jaw_anchor=dict(width_m=.0326, observed_at=120.,
                    source=dict(path='/synthetic/passive.json', sha256='c'*64))))

    def test_source_envelope_preserves_original_candidate_body_and_failure(self):
        before = copy.deepcopy(self.proposal)
        payload, source = recovery.recovery_source(dict(proposal=self.proposal))
        self.assertEqual(payload['arm'], 'right')
        self.assertFalse(source['ok'])
        self.assertEqual(source['candidate_measurement']['anchor'], self.candidate_anchor)
        self.assertEqual(source['candidate_measurement']['observed']['width_m'], .028)
        self.assertEqual(source['audited_opening_continuation']['residual_jaw_anchor']['width_m'], .0326)
        self.assertEqual(source['audited_opening_continuation']['audit_proposal_sha256'], 'b'*64)
        self.assertEqual(self.proposal, before)

    def test_latest_receipt_cannot_hide_original_partial_send(self):
        source = json.loads(self.proposal['snapshot']['scope']['record_json'])
        event = source['proposal']['snapshot']['event']
        receipt = json.loads(event['receipt_json'])
        receipt['device_receipt']['transmission_counts']['right']['sent_frames'] = 0
        event['receipt_json'] = json.dumps(receipt)
        self.proposal['snapshot']['scope']['record_json'] = json.dumps(source)
        with self.assertRaises(PairLedgerError):
            recovery.recovery_source(dict(proposal=self.proposal))


class OpeningContinuationProvenanceReviewTests(unittest.TestCase):
    """Known frame completion must be supported by immutable source bytes."""

    def setUp(self):
        self.fixture = prior_recovery_tests.SupportedRecoveryTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        old = self.fixture.f
        body = healthy_arm(107.)
        body['gripper']['width_m'] = .028
        anchor = measured_anchor(body)
        old.receipt['device_receipt']['candidate_measurement']['anchor'] = anchor
        old.receipt['device_receipt']['candidate_probe'].update(sent_at=103.02, completed_at=107.)
        old.refresh()
        original = dict(proposal_sha256='a'*64,
            snapshot=dict(event=old.event, run=old.run, scope=dict(owner='old-owner')))
        self.scope = dict(run_id='run', owner='failed-owner', record_json=json.dumps(dict(proposal=original)),
                          contract_json=json.dumps(dict(task={})))
        self.payload = dict(kind='supported_recovery_open', arm='right', source_event_id=old.event['event_id'],
            recovery_proposal_sha256='a'*64, saved_rgb_evidence={},
            request=dict(operation='supported_recovery_open', observation_id='new-scene',
                         visual_description='Synthetic independent support.',
                         support_relation='independent_support_present', width_m=.032,
                         object_relation=None))
        after = {side: healthy_arm(109.99) for side in ('left', 'right')}
        after['right']['joints_rad'][3] = .0032
        after['right']['gripper']['width_m'] = .0318
        counts = {side: dict(attempted_frames=int(side == 'right'), sent_frames=int(side == 'right'),
                             blocked_frames=0) for side in ('left', 'right')}
        residual = dict(status='recovery_residual', arm='right', original_anchor=anchor,
            requested_width_m=.025, observed_width_m=.028, sent_at=103.02, trace_sha256='a'*64,
            identity=None, probe_event_id=None, failed_receipt_preserved=True,
            grasp_verified=False, target_may_remain_active=True)
        device = dict(ok=False, arrival_confirmed=False, requested_target=.032,
            completion_mode='contact_probe_release',
            errors=[dict(type='RuntimeError', detail='Grasp arm changed from its original candidate anchor: right')],
            transmission_counts=counts, session_transmission_counts=copy.deepcopy(counts),
            hardware_commands_sent=1, target_calls_sent=1, nominal_force_N=.2, guard_violations=[],
            enable_commands_sent=0, stop_commands_sent=0, retries=0, passive_arm_commands_sent=0,
            unresolved_gripper_probe=None, grasp_verified=False, physical_stop_verified=None,
            grasp_states=dict(left=None, right=residual), after=after,
            dispatch_feedback=dict(right=dict(gripper=dict(width_m=.028))))
        self.receipt = dict(ok=False, event_id='failed-opening', automatic_retry=False,
                            physical_stop_verified=None, device_receipt=device)
        self.event = dict(run_id='run', event_id='failed-opening', owner='failed-owner', step=10,
                          status='complete', success=0, began_at=109., finished_at=110.)
        self.run = dict(run_id='run', steps=10)
        self.refresh()
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def refresh(self):
        self.event.update(payload_json=json.dumps(self.payload), receipt_json=json.dumps(self.receipt))
        self.event['payload_digest'] = hashlib.sha256(self.event['payload_json'].encode()).hexdigest()

    def validate(self):
        self.refresh()
        return recovery.validate_failed_opening(self.event, self.run, self.scope)

    def history_sources(self):
        self.validate()
        device = self.receipt['device_receipt']
        cleanup = dict(session_transmission_counts=device['session_transmission_counts'],
            guard_violations=[], unresolved_gripper_probe=None, grasp_states=device['grasp_states'],
            requires_fault_latch=True, arms={side: dict(status='disconnected') for side in ('left','right')})
        session = [dict(sequence=1, at=108., kind='result', result=dict(owner='failed-owner', run_id='run')),
                   dict(sequence=2, at=110.1, kind='result',
                        result=dict(status='closed', fault_latched=True, cleanup=cleanup)),
                   dict(sequence=3, at=110.2, kind='session_ended', cleanup_errors=[])]
        rows = [dict(event='pair_preparation_claimed', unix_s=109.01, event_id='failed-opening', payload=self.payload),
                dict(event='single_supervised_action_intent', unix_s=109.02, arm='right', kind='gripper', target=.032,
                     frames=[dict(id=0x159, data_hex='00007d0000c80100')]),
                dict(event='single_supervised_action_sent_unconfirmed', unix_s=109.04,
                     finished_unix_s=109.039, kind='gripper')]
        self.session_path = self.root/'session.jsonl'
        self.session_path.write_text(''.join(json.dumps(row)+'\n' for row in session))
        self.journal_path = self.root/'events.jsonl'
        self.journal_path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        self.index = dict(source=str(self.journal_path),
                          source_sha256=hashlib.sha256(self.journal_path.read_bytes()).hexdigest(), rows=rows)
        self.index_path = self.root/'opening-index.json'
        self.index_path.write_text(json.dumps(self.index))
        return dict(event=self.event, run=self.run, scope=self.scope)

    def test_complete_opening_anchor_failure_remains_failure(self):
        payload, receipt = self.validate()
        self.assertEqual(payload['request']['width_m'], .032)
        self.assertFalse(receipt['ok'])
        self.assertFalse(receipt['arrival_confirmed'])
        self.assertFalse(receipt['grasp_verified'])

    def test_unknown_send_hardware_fault_or_reanchored_body_is_not_recoverable(self):
        original = copy.deepcopy(self.receipt)
        mutations = [
            lambda d: d['transmission_counts']['right'].update(sent_frames=0),
            lambda d: d.update(retries=1),
            lambda d: d['after']['left']['drivers']['1']['foc_status'].update(driver_overcurrent=True),
            lambda d: d['grasp_states']['right']['original_anchor']['joints_rad'].__setitem__(3, .0032),
            lambda d: d['after']['right']['joints_rad'].__setitem__(3, 0.),
        ]
        for index, mutation in enumerate(mutations):
            self.receipt = copy.deepcopy(original)
            mutation(self.receipt['device_receipt'])
            with self.subTest(index=index), self.assertRaises(PairLedgerError):
                self.validate()

    def test_normal_future_journal_append_preserves_frozen_old_frame_provenance(self):
        snapshot = self.history_sources()
        history = recovery.opening_history(self.session_path, self.index_path, snapshot)
        with self.journal_path.open('a') as stream:
            stream.write(json.dumps(dict(event='pair_preparation_claimed', unix_s=120., event_id='new'))+'\n')
        checked = recovery.opening_history(self.session_path, self.index_path, snapshot,
                                           source_prefix=history['source_prefix'])
        self.assertEqual(checked, history)

    def test_replaced_prefix_or_omitted_returned_frame_is_rejected(self):
        snapshot = self.history_sources()
        history = recovery.opening_history(self.session_path, self.index_path, snapshot)
        old = self.journal_path.read_bytes()
        self.journal_path.write_bytes(old.replace(b'00007d00', b'00007d01'))
        with self.assertRaises(PairLedgerError):
            recovery.opening_history(self.session_path, self.index_path, snapshot,
                                     source_prefix=history['source_prefix'])
        self.journal_path.write_bytes(old)
        self.index['rows'].pop()
        self.index_path.write_text(json.dumps(self.index))
        with self.assertRaises(PairLedgerError):
            recovery.opening_history(self.session_path, self.index_path, snapshot)

    def reconciled_history(self):
        snapshot = self.history_sources()
        session = [json.loads(line) for line in self.session_path.read_text().splitlines()]
        session.insert(1, dict(at=110.05, kind='result',
            result=dict(event_id=self.event['event_id'], status='fault', receipt=copy.deepcopy(self.receipt))))
        for number, row in enumerate(session, 1):
            row['sequence'] = number
        self.session_path.write_text(''.join(json.dumps(row)+'\n' for row in session))
        history = recovery.opening_history(self.session_path, self.index_path, snapshot)
        pending = {**self.event, 'status':'pending', 'receipt_json':None,
                   'finished_at':None, 'success':None}
        self.receipt['archival_reconciliation'] = dict(
            schema='piper_failed_opening_archival_v1', original_pending_event=pending,
            original_pending_sha256=recovery._sha(pending),
            original_failure_receipt_sha256=recovery._receipt_digest(self.receipt),
            original_fault=dict(id=33, owner='failed-owner', run_id='run', at=110.,
                reason='Claimed preparation/query failed or uncertain: Supported jaw recovery failed; no automatic retry'),
            recorded_at=500., hardware_commands_sent=0, old_failure_preserved=True,
            history=history, fault_result_sequence=2)
        self.event['finished_at'] = 500.
        self.refresh()
        return dict(event=self.event, run=self.run, scope=self.scope), history

    def test_late_bookkeeping_time_does_not_retimestamp_physical_feedback(self):
        snapshot, history = self.reconciled_history()
        self.validate()
        self.assertEqual(self.event['finished_at'], 500.)
        self.assertEqual(recovery._opening_failure_at(self.event), 110.)
        self.assertEqual(self.receipt['device_receipt']['after']['right']['timestamp'], 109.99)
        self.assertEqual(recovery.opening_history(self.session_path, self.index_path, snapshot), history)
        self.assertEqual(self.event['success'], 0)

    def test_rehashed_reconciliation_cannot_substitute_an_external_fault_receipt(self):
        snapshot, _ = self.reconciled_history()
        self.receipt['device_receipt']['after']['right']['gripper']['width_m'] = .0319
        self.receipt['archival_reconciliation']['original_failure_receipt_sha256'] = recovery._receipt_digest(
            {key:value for key,value in self.receipt.items() if key != 'archival_reconciliation'})
        self.refresh()
        with self.assertRaisesRegex(PairLedgerError, 'equal the original host fault result'):
            recovery.opening_history(self.session_path, self.index_path, snapshot)


if __name__ == '__main__':
    unittest.main()
