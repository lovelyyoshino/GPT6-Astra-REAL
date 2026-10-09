"""Independent contact-round checks, entirely offline.

Native host tests use the real service, ledger, encoder and FakeCAN. Their
archived enrollment source is synthetic; the production contact phase reader
consumes actual SQL action rows. Separate enrollment tests cover authorization
and the immutable parent lineage, not physical scene qualification.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import supported_gripper_recovery as recovery
from robot_tools.pair_ledger import (
    PairLedger, PairLedgerError, _execution_scope, activated_execution_budget,
)
import test_supported_contact_reacquisition_review as contact_review


class ContactZeroTxRoundHostReviewTests(
        contact_review.SupportedContactReacquisitionHostReviewTests):
    def setUp(self):
        super().setUp()
        self.proposal.update(max_probes=2, stage_attempt_limit=3,
                             consumed_probe_count=1)

    def recovery_state(self):
        with sqlite3.connect(str(self.host.runs/'pair_sessions.sqlite')) as db:
            db.row_factory = sqlite3.Row
            events = list(db.execute(
                'SELECT * FROM pair_events WHERE run_id=? ORDER BY step',
                (self.host.run_id,)))
            return recovery._contact_runtime(db, self.host.run_id,
                dict(owner=self.host.owner), self.proposal, events)

    # A renewed task budget does not renew the original three-probe phase.
    test_three_arrivals_without_contact_end_the_probe_allowance = None

    def test_only_two_remaining_arrivals_and_no_third_claim_or_frame(self):
        initial = self.recovery_state()
        self.assertEqual(initial['phase'], 'reacquire_required')
        self.assertIsNone(initial['prior_target_m'])
        for index, target in enumerate((.0455, .042), 1):
            _, result = self.probe('remaining-'+str(index), target=target,
                                   contact=None)
            self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(self.recovery_state()['phase'], 'exhausted')
        before = self.host.ledger.status()['steps']
        with self.assertRaises((ValueError, RuntimeError)):
            self.service.call('robot_pair_submit_once',
                self.request('forbidden-third', target=.0385))
        self.assertIsNone(self.host.ledger.event('forbidden-third'))
        self.assertEqual(self.host.ledger.status()['steps'], before)
        self.assert_only_left_jaw_frames(2)

    def test_query_initialization_and_empty_jaw_preparation_stay_blocked(self):
        scene = self.observe()
        calls = (
            lambda: self.host.inspect_joint_limits('no-query'),
            lambda: self.host.initialize_joint_target('no-initialize',
                scene['observation_id'], 'left', 'Synthetic requested empty-jaw admission',
                admission_mode='rgb_supervised', corridor_observation='Synthetic corridor'),
            lambda: self.host.prepare_gripper('no-prepare', scene['observation_id'],
                'left', 'Synthetic requested empty-jaw preparation'),
        )
        for call in calls:
            with self.subTest(call=call), self.assertRaises(RuntimeError):
                call()
        self.assertEqual(self.host.ledger.status()['steps'], 0)
        self.assert_only_left_jaw_frames(0)
        self.assertFalse(self.host.status()['fault_latched'])

    def test_new_source_uses_measured_opening_not_the_untransmitted_target(self):
        state = self.recovery_state()
        self.assertEqual(state['current_width_m'], .05)
        self.assertIsNone(state['prior_target_m'])
        original = self.source['candidate_measurement']['anchor']
        _, result = self.probe(target=.0455)
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(result['receipt']['reacquisition_original_anchor'], original)
        self.assertEqual(self.recovery_state()['phase'], 'contact_candidate')
        retained, _ = self.retain('left')
        self.assertEqual(retained['hardware_commands_sent'], 0)
        self.assertFalse(retained['loaded_contact_available'])
        self.assert_only_left_jaw_frames(1)


class ContactZeroTxRoundGateReviewTests(unittest.TestCase):
    """The new round must keep its contact gate after the old table retires.

This is a runtime-selection fixture, not a fabricated enrollment audit. Only
the archived source lookup is replaced; owner, round and request gates run.
"""
    def setUp(self):
        self.db = sqlite3.connect(':memory:')
        self.db.row_factory = sqlite3.Row
        self.addCleanup(self.db.close)
        self.db.executescript('''
            CREATE TABLE pair_rounds(ordinal INTEGER,run_id TEXT,parent_run_id TEXT,
                owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL,
                retired_owners_json TEXT,proposal_sha256 TEXT,authorization_sha256 TEXT,
                record_json TEXT);
            CREATE TABLE pair_supported_contact_reacquisitions(ordinal INTEGER,
                run_id TEXT,owner TEXT,round_ordinal INTEGER,record_json TEXT,fault_id INTEGER);
            CREATE TABLE pair_events(run_id TEXT,event_id TEXT,step INTEGER,owner TEXT,
                payload_json TEXT,status TEXT,success INTEGER,receipt_json TEXT);
            CREATE TABLE pair_faults(id INTEGER,run_id TEXT,owner TEXT,reason TEXT,at REAL);
            INSERT INTO pair_faults VALUES(36,'old-run','retired-owner','RGB shared scene expired before the next guarded operation',40);
            INSERT INTO pair_faults VALUES(37,'old-run','retired-owner','execution_receipt_failed',41);
        ''')
        route = dict(route='audited_contact_reacquisition', stage_attempt_limit=3,
                     consumed_probe_count=1, max_probes=2)
        self.proposal = dict(parent_kind='supported_contact_zero_tx_fault',
            new_run_id='new-run', proposal_sha256='b'*64, contact_route=route,
            snapshot=dict(contact_attempts=copy.deepcopy(route),
                contact_source=dict(snapshot={}, reacquisition={})),
            recovery_evidence=dict(passive=dict(left=dict(jaw_width_m=.03836))))
        self.db.execute('INSERT INTO pair_rounds VALUES(?,?,?,?,?,?,?,?,?,?,?)',
            (2, 'new-run', 'old-run', 'new-owner', 'new-run', None, 100.,
             json.dumps(['retired-owner']), 'b'*64, 'c'*64,
             json.dumps(dict(proposal=self.proposal))))
        self.db.execute('INSERT INTO pair_supported_contact_reacquisitions VALUES(?,?,?,?,?,?)',
            (1, 'old-run', 'retired-owner', 1, '{"synthetic_old_scope":true}', 36))
        self.db.execute('INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?)',
            ('old-run', 'zero-send-failed', 16, 'retired-owner', '{}', 'complete', 0,
             '{"ok":false,"hardware_commands_sent":0}'))
        self.parent_rows = self.old_rows()
        source_patch = patch.object(recovery, 'reacquisition_source', return_value=(
            dict(arm='left', grasp_object_id='white_charger'), {}))
        source_patch.start()
        self.addCleanup(source_patch.stop)

    def old_rows(self):
        return {name:[dict(row) for row in self.db.execute(
            'SELECT * FROM '+name+' WHERE run_id=?', ('old-run',))]
            for name in ('pair_events','pair_faults','pair_supported_contact_reacquisitions')}

    def payload(self, **changes):
        result = dict(arm='left', kind='gripper', operation='grip_supported',
            target=.034, grasp_object_id='white_charger',
            reacquisition_proposal_sha256='b'*64,
            probe_support_relation='independent_support_present',
            probe_support_observation='Synthetic original socket remains the support.')
        result.update(changes)
        return result

    def test_new_round_selects_restricted_route_and_preserves_failed_parent(self):
        name, _, ordinal, _ = _execution_scope(self.db, 'new-run', writable=True)
        self.assertEqual((name, ordinal), ('pair_rounds', 2))
        state = recovery.runtime(self.db, 'new-run', 'new-owner')
        self.assertEqual(state['phase'], 'reacquire_required')
        self.assertEqual(state['consumed_probe_count'], 1)
        self.assertEqual(state['proposal']['max_probes'], 2)
        self.assertEqual(state['current_width_m'], .03836)
        self.assertIsNone(state['prior_target_m'])
        recovery.check_request(self.db, 'new-run', 'new-owner', self.payload())
        self.assertEqual(self.old_rows(), self.parent_rows)


    def test_retired_owner_and_arbitrary_run_cannot_select_new_scope(self):
        for run in ('old-run', 'arbitrary-new-run'):
            with self.subTest(run=run), self.assertRaises(PairLedgerError):
                _execution_scope(self.db, run, writable=True)
        with self.assertRaises(PairLedgerError):
            recovery.check_request(self.db, 'new-run', 'retired-owner', self.payload())

    def test_only_same_left_object_support_and_proposal_are_admitted(self):
        for changes in (dict(kind='query'), dict(kind='initialization'), dict(kind='joint'),
                        dict(arm='right'), dict(operation='extract_segment'),
                        dict(grasp_object_id='other-object'),
                        dict(reacquisition_proposal_sha256='d'*64),
                        dict(probe_support_relation='unknown'),
                        dict(probe_support_observation=''), dict(target=.032)):
            with self.subTest(changes=changes), self.assertRaises(PairLedgerError):
                recovery.check_request(self.db, 'new-run', 'new-owner', self.payload(**changes))
        self.assertEqual(self.old_rows(), self.parent_rows)

    def test_phase_allowance_cannot_be_restored_by_normalizing_a_new_round(self):
        for changes in (dict(max_probes=3), dict(consumed_probe_count=0),
                        dict(stage_attempt_limit=4), dict(route='ordinary')):
            proposal = copy.deepcopy(self.proposal)
            proposal['contact_route'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(PairLedgerError):
                recovery.contact_round_proposal(proposal)
        proposal = copy.deepcopy(self.proposal)
        for key in ('contact_route',):
            proposal[key].update(consumed_probe_count=0, max_probes=3)
        proposal['snapshot']['contact_attempts'] = copy.deepcopy(proposal['contact_route'])
        with self.assertRaises(PairLedgerError):
            recovery.contact_round_proposal(proposal)

    def test_new_pending_or_failed_claim_is_never_automatically_retried(self):
        for status, success, receipt in (('pending', None, None),
                ('complete', 0, json.dumps(dict(ok=False, hardware_commands_sent=0)))):
            with self.subTest(status=status):
                self.db.execute("DELETE FROM pair_events WHERE run_id='new-run'")
                self.db.execute('INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?)',
                    ('new-run','new-attempt',1,'new-owner',json.dumps(self.payload()),
                     status,success,receipt))
                self.assertEqual(recovery.runtime(self.db,'new-run','new-owner')['phase'], 'unresolved')
                with self.assertRaises(PairLedgerError):
                    recovery.check_request(self.db,'new-run','new-owner',self.payload())
        self.assertEqual(self.old_rows(), self.parent_rows)

@unittest.skipUnless(os.environ.get('PIPER_RUN_ARCHIVED_CONTACT_TESTS') == '1',
                     'Explicit opt-in required for read-only archived ledger backup')
class ContactZeroTxArchivedRoundReviewTests(unittest.TestCase):
    """Full real-parent audit, synthetic new authority, only temporary DB writes."""
    def setUp(self):
        fixture_path = Path('artifacts/contact_zero_tx_restart_20261008/offline_fixture.py')
        spec = importlib.util.spec_from_file_location('contact_round_offline_fixture', fixture_path)
        fixture = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fixture)
        manager = fixture.copied_fixture()
        self.f = manager.__enter__()
        self.addCleanup(manager.__exit__, None, None, None)
        self.path = self.f['path']
        self.assertNotEqual(self.path.resolve(),
            Path('projects/piperx_cloth_demo/runs/pair_sessions.sqlite').resolve())
        self.before = self.rows()

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            return {name:[dict(r) for r in db.execute('SELECT * FROM '+name)]
                for name, in db.execute(
                    "SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'").fetchall()}

    def activate(self, authorization=None, clock=None):
        from robot_tools import pair_round
        return pair_round.activate_round(self.f['proposal'],
            self.f['authorization'] if authorization is None else authorization,
            project_root=self.f['root'], clock=clock or (lambda:self.f['now']))

    def test_full_archived_budget_history_and_prepare_host(self):
        from robot_tools.pair_host import PairHost
        from test_pair_host import FakePairDevice
        result = self.activate()
        after = self.rows()
        for name, rows in self.before.items():
            self.assertTrue(all(row in after[name] for row in rows), name)
        self.assertEqual(after['pair_faults'], self.before['pair_faults'])
        proposal = self.f['proposal']; run = proposal['new_run_id']
        self.assertEqual(proposal['contact_route']['consumed_probe_count'], 1)
        self.assertEqual(proposal['contact_route']['max_probes'], 2)
        self.assertEqual(proposal['parent_run']['steps'], 16)
        self.assertTrue(activated_execution_budget(self.path,run,max_steps=1000,max_duration_s=10800))
        self.assertFalse(activated_execution_budget(self.path,run,max_steps=999,max_duration_s=10800))
        ledger = PairLedger(self.path,run,result['new_contract'],max_steps=1000,
                            max_duration_s=10800,clock=lambda:self.f['now'])
        with self.assertRaises(PairLedgerError):
            ledger.claim(proposal['snapshot']['retired_owner'])
        devices = []
        now = [self.f['now']]
        clock = SimpleNamespace(time=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt))
        class PreparationDevice(FakePairDevice):
            def connect_for_preparation(inner):
                return {**inner.open(), 'task_ready':False, 'readiness':{'synthetic_fixture':True}}
        def factory(*args):
            device=PreparationDevice(*args,clock);devices.append(device);return device
        profile=json.loads((self.f['root']/'configs/robot.json').read_text())
        args=dict(device_factory=factory,clock=clock.time,background=False)
        with self.assertRaises(ValueError):
            PairHost(self.f['root']/'runs',profile,run,result['new_contract']['task'],
                1000,10800,connection_mode='ready',**args)
        self.assertEqual(devices, [])
        host=PairHost(self.f['root']/'runs',profile,run,result['new_contract']['task'],
            1000,10800,connection_mode='prepare',**args)
        self.addCleanup(host.close)
        opened=host.open()
        self.assertTrue(opened['open']);self.assertFalse(opened['task_ready'])
        self.assertEqual(host.deadline,proposal['deadline_s'])
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            state=recovery.runtime(db,run,host.owner)
            self.assertEqual(state['phase'],'reacquire_required')
            self.assertEqual(state['proposal']['max_probes'],2)
            self.assertIsNone(state['prior_target_m'])
        with self.assertRaises(RuntimeError):host.inspect_joint_limits('forbidden-query')
        self.assertEqual(devices[0].calls,[])
        self.assertEqual(devices[0].frame_attempts,0)
        self.assertEqual(host.ledger.status()['steps'],0)

    def test_old_authority_wrong_budget_and_expired_activation_leave_all_rows_unchanged(self):
        proposal=self.f['proposal']
        for changes in (
                dict(decision='authorize_repaired_continuation'),
                dict(received_at=proposal['authorization_not_before']-1),
                dict(new_budget={**proposal['new_budget'],'max_steps':999}),
                dict(proposal_sha256='0'*64)):
            authorization={**self.f['authorization'],**changes}
            with self.subTest(changes=changes),self.assertRaises(PairLedgerError):
                self.activate(authorization)
            self.assertEqual(self.rows(),self.before)
        with self.assertRaises(PairLedgerError):
            self.activate(clock=lambda:proposal['deadline_s'])
        self.assertEqual(self.rows(),self.before)

    def test_partial_send_or_nonempty_episode_cannot_borrow_archived_zero_tx_proof(self):
        from robot_tools import pair_round
        event=self.f['proposal']['snapshot']['zero_tx_event']
        receipt=json.loads(event['receipt_json'])
        for field in ('attempted_frames','sent_frames','blocked_frames'):
            changed=copy.deepcopy(receipt)
            changed['device_receipt']['transmission_counts']['left'][field]=1
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE run_id=? AND event_id=?',
                    (json.dumps(changed),event['run_id'],event['event_id']))
            with self.subTest(field=field),self.assertRaises(PairLedgerError):
                pair_round.prepare_round(self.path,event['run_id'],**self.f['args'])
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE run_id=? AND event_id=?',
                    (event['receipt_json'],event['run_id'],event['event_id']))
        episode=next(row for row in self.before['pair_grasp_episodes'] if row['owner']==event['owner'])
        original=json.loads(episode['state_json'])
        for changes in (dict(status='contact_candidate'),dict(dispatch_authorized=True),
                        dict(target_may_remain_active=True),
                        dict(identity={**original['identity'],'owner':'other-owner'})):
            state={**original,**changes}
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_grasp_episodes SET state_json=? WHERE episode_id=?',
                           (json.dumps(state),episode['episode_id']))
            with self.subTest(changes=changes),self.assertRaises(PairLedgerError):
                pair_round.prepare_round(self.path,event['run_id'],**self.f['args'])
