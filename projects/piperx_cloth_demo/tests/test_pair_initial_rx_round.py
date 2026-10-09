"""Offline archived-receipt mutation tests, never original ledger/device I/O."""
import copy
import gzip
import json
import os
from pathlib import Path
import shutil
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import pair_round as entry
from robot_tools.pair_ledger import PairLedger, PairLedgerError, platform_fault, activated_execution_budget


class InitialRXRoundTests(unittest.TestCase):
    def setUp(self):
        for target in ('socket.socket', 'robot_tools.arms._load_sdk', 'robot_tools.cameras.capture_cameras'):
            p = patch(target, side_effect=AssertionError('Offline only')); p.start(); self.addCleanup(p.stop)
        p = patch.object(entry, '_live_control_processes', return_value=[]); p.start(); self.addCleanup(p.stop)
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)/'piperx_cloth_demo'; (self.root/'runs').mkdir(parents=True)
        (self.root/'configs').mkdir(); self.path = self.root/'runs/pair_sessions.sqlite'
        with gzip.open(Path(__file__).parent/'fixtures/initial_rx_round_audit.json.gz', 'rt') as f:
            self.data = json.load(f)
        self.run = self.data['run_id']
        with sqlite3.connect(self.path) as db:
            for name, sql in self.data['schema'].items():
                db.execute(sql)
                for row in self.data['tables'][name]:
                    db.execute('INSERT INTO '+name+' ('+','.join(row)+') VALUES ('+','.join('?' for _ in row)+')', list(row.values()))
        old = json.loads(self.data['tables']['pair_endpoint_continuations'][0]['contract_json'])
        (self.root/'configs/robot.json').write_text(json.dumps({k:old[k] for k in ('arms','cameras','sdk_commit_audited')}))
        shutil.copytree(Path(entry.__file__).parent, self.root/'robot_tools', ignore=shutil.ignore_patterns('__pycache__'))
        manifest = {'schema':'piper_cli_shutdown_evidence_v1'}
        for name, data in self.data['close_docs'].items():
            p = self.root/(name+'.json')
            p.write_text(''.join(json.dumps(r)+'\n' for r in data) if name.endswith('_log') else json.dumps(data))
            manifest[name] = str(p)
        self.close = self.root/'close.json'; self.close.write_text(json.dumps(manifest))
        self.now = self.data['close_docs']['rejected_open_log'][-1]['at']+20
        self.started = self.now-1; self.passive = {}
        def shift(x, delta):
            if isinstance(x, dict): return {k:shift(v,delta) for k,v in x.items()}
            if isinstance(x, list): return [shift(v,delta) for v in x]
            return x+delta if type(x) in (int,float) and 1e9 < x < 2e9 else x
        for side, data in self.data['passive'].items():
            data = shift(data, self.now-1-data['finished_at_s'])
            p = self.root/(side+'.json'); p.write_text(json.dumps(data)); self.passive[side] = str(p)
        rgb = {'capture_id':'synthetic-current', 'cameras':{}}
        for view, key in (('front','front'),('left_hand','left_wrist'),('right_hand','right_wrist')):
            p = self.root/(view+'.png'); p.write_bytes(b'Synthetic test image, not physical evidence')
            rgb['cameras'][view] = dict(serial=old['cameras'][key],rgb_path=str(p),host_received_at=self.now-1,frame_number=1,depth_enabled=False)
        self.rgb = self.root/'rgb.json'; self.rgb.write_text(json.dumps(rgb))

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            return {n:sorted([dict(r) for r in db.execute('SELECT * FROM '+n)],key=lambda r:json.dumps(r,sort_keys=True))
                    for n, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'").fetchall()}

    def prepare(self):
        return entry.prepare_round(self.path,self.run,close_log=self.close,new_run_id='repaired-new-round',
            started_at=self.started,max_steps=1000,max_duration_s=10800,
            budget_start_policy='after_repair_before_online_execution',parent_kind='initial_rx_zero_tx_fault',
            recovery_evidence=dict(passive_paths=self.passive,rgb_observation=str(self.rgb),
                visual_observation='Synthetic test: empty jaws, table supports strip, plug still seated'),clock=lambda:self.now)

    def activate(self, p):
        auth = dict(source='user_message',message_id='test-user-new-budget',statement='After repair, a new 3h/1000 step round',
            received_at=self.started,decision='authorize_explicit_new_round',proposal_sha256=p['proposal_sha256'],
            new_budget=p['new_budget'],budget_start_policy=p['budget_start_policy'])
        return entry.activate_round(p,auth,project_root=self.root,clock=lambda:self.now)

    def audit(self, db=None):
        if db is None:
            with sqlite3.connect(self.path.as_uri()+'?mode=ro',uri=True) as db:
                db.row_factory = sqlite3.Row
                db.execute('PRAGMA query_only=ON')
                return self.audit(db)
        run = db.execute("SELECT * FROM pair_runs WHERE run_id='repaired-new-round'").fetchone()
        scope = db.execute("SELECT * FROM pair_rounds WHERE run_id='repaired-new-round'").fetchone()
        return entry.audit_unopened_enrollment(db,scope,run)

    def test_append_only_round_preserves_faults_and_blocks_old_owner(self):
        before = self.rows(); p = self.prepare(); self.assertEqual(before,self.rows())
        result = self.activate(p); after = self.rows()
        for name, rows in before.items():
            self.assertTrue(all(r in after[name] for r in rows), name)
        self.assertEqual(before['pair_faults'],after['pair_faults'])
        self.assertFalse(result['cache_or_limits_transferred']); self.assertEqual(result['hardware_commands_sent'],0)
        self.assertIsNone(result['physical_stop_verified'])
        self.assertIsNotNone(platform_fault(self.path))
        self.assertIsNone(platform_fault(self.path,run_id='repaired-new-round'))
        self.assertTrue(activated_execution_budget(self.path,'repaired-new-round',max_steps=1000,max_duration_s=10800))
        ledger = PairLedger(self.path,'repaired-new-round',result['new_contract'],max_steps=1000,max_duration_s=10800,clock=lambda:self.now)
        for owner in p['snapshot']['retired_owners']:
            with self.assertRaises(PairLedgerError): ledger.claim(owner)
        state = ledger.claim('new-offline-owner')
        self.assertEqual(state['deadline_s'],self.started+10800)
        self.assertEqual(state['execution_lineage']['cumulative_steps'],p['snapshot']['cumulative_prior_steps'])
        with self.assertRaises(PairLedgerError): self.activate(p)

    def test_partial_send_other_failure_or_missing_sample_refuses(self):
        mutations = [lambda d:d.update(hardware_commands_sent=1),
            lambda d:d['transmission_counts']['left'].update(attempted_frames=1),
            lambda d:d.update(original_event={'send_state':'unknown'}),
            lambda d:d['errors'][0].update(detail='hard limit'),
            lambda d:d['tracking_observation']['first_failure'].update(code='joint_limit'),
            lambda d:d['joint_validation_timing'].update(live_feedback_age_limit_s=.1),
            lambda d:d['rejected_joint_feedback']['arms']['right']['joints_rad'].__setitem__(0,2.)]
        original = next(e for e in self.data['tables']['pair_events'] if e['event_id']=='left-clear-plug-up-4')
        for mutate in mutations:
            receipt = json.loads(original['receipt_json']); mutate(receipt['device_receipt'])
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(receipt),original['event_id']))
            with self.subTest(mutate=mutate), self.assertRaises((PairLedgerError,ValueError)): self.prepare()
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(original['receipt_json'],original['event_id']))
        self.prepare()

    def test_historical_fault_change_and_pending_event_refuse(self):
        for sql in ("UPDATE pair_faults SET reason='erased' WHERE id=15",
                    "UPDATE pair_faults SET reason='different' WHERE id=23",
                    "UPDATE pair_events SET status='pending' WHERE event_id='left-clear-plug-up-4'"):
            with sqlite3.connect(self.path) as db:
                db.execute('BEGIN'); db.execute(sql)
                # Validate in the same uncommitted test transaction.
                db.row_factory = sqlite3.Row
                with self.subTest(sql=sql), self.assertRaises(PairLedgerError):
                    entry._snapshot(db,self.run,'initial_rx_zero_tx_fault')
                db.rollback()

    def test_cleanup_transmissions_or_missing_finally_refuse(self):
        p = self.root/'close_verification.json'; original = json.loads(p.read_text())
        for mutate in (lambda d:d['tx_packet_change'].update(can0=1),
                       lambda d:d['session_end'].update(cleanup_errors=['failed']),
                       lambda d:d.update(original_process_exited=False)):
            changed = copy.deepcopy(original); mutate(changed); p.write_text(json.dumps(changed))
            with self.subTest(mutate=mutate), self.assertRaises(PairLedgerError): self.prepare()
        p.write_text(json.dumps(original)); self.prepare()

    def test_live_host_changed_source_and_stale_evidence_refuse(self):
        p = self.prepare(); before = self.rows()
        with patch.object(entry,'_live_control_processes',return_value=['active']):
            with self.assertRaises(PairLedgerError): self.activate(p)
        source = self.root/'robot_tools/pair_joint_adapter.py'; original = source.read_bytes(); source.write_bytes(original+b'\n#changed')
        with self.assertRaises(PairLedgerError): self.activate(p)
        source.write_bytes(original); self.now += 31
        with self.assertRaises(PairLedgerError): self.activate(p)
        self.assertEqual(before,self.rows())

    def test_rpc_target_or_current_joint_excursion_refuses(self):
        session = self.root/'session_log.json'; original = session.read_text()
        rows = [json.loads(l) for l in original.splitlines()]
        next(r for r in rows if r.get('request',{}).get('op') == 'robot_pair_submit_once')['request']['arguments']['target_joints_rad'][0] += .01
        session.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        with self.assertRaises(PairLedgerError): self.prepare()
        session.write_text(original)
        path = Path(self.passive['right']); data = json.loads(path.read_text())
        data['pose_trace'][10]['joints_raw']['joint_4'] += 1000
        path.write_text(json.dumps(data))
        with self.assertRaises(PairLedgerError): self.prepare()

    def test_unrelated_control_change_and_missing_authorization_refuse(self):
        p = self.prepare(); before = self.rows()
        with self.assertRaises((PairLedgerError,KeyError)):
            entry.activate_round(p,{},project_root=self.root,clock=lambda:self.now)
        source = self.root/'robot_tools/joint_path.py'; source.write_bytes(source.read_bytes()+b'\n# unrelated')
        with self.assertRaises(PairLedgerError): self.prepare()
        self.assertEqual(before,self.rows())

    def test_rejected_sample_does_not_hide_another_device_or_limit_fault(self):
        event = next(e for e in self.data['tables']['pair_events'] if e['event_id']=='left-clear-plug-up-4')
        mutations = [lambda arms:arms['left']['arm_status'].update(err_code=1),
                     lambda arms:arms['right']['drivers']['1']['foc_status'].update(driver_overcurrent=True),
                     lambda arms:arms['left']['gripper']['foc_status'].update(driver_enable_status=False),
                     lambda arms:arms['right']['feedback_assembly'].update(error='malformed CAN group'),
                     lambda arms:arms['left']['joints_rad'].__setitem__(0,9.),
                     lambda arms:arms['left']['fragment_timestamps_s'].pop('gripper')]
        for mutate in mutations:
            receipt = json.loads(event['receipt_json']); device = receipt['device_receipt']
            arms = device['tracking_observation']['first_failure']['sample']['arms']; mutate(arms)
            device['rejected_joint_feedback']['arms'] = copy.deepcopy(arms)
            device['after'] = copy.deepcopy(arms)
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',
                           (json.dumps(receipt),event['event_id']))
            with self.subTest(mutate=mutate),self.assertRaises(PairLedgerError): self.prepare()

    def test_original_enrollment_is_rechecked_read_only_after_child_claim(self):
        proposal = self.prepare(); result = self.activate(proposal)
        before = self.rows(); audit = self.audit()
        self.assertEqual(before,self.rows())
        self.assertTrue(audit['historical_observations_only'])
        self.assertTrue(audit['fresh_host_admission_required'])
        self.assertEqual(audit['original_proposal_sha256'],proposal['proposal_sha256'])
        ledger = PairLedger(self.path,'repaired-new-round',result['new_contract'],max_steps=1000,
                            max_duration_s=10800,clock=lambda:self.now)
        ledger.claim('audited-child-owner')
        self.assertEqual(audit,self.audit())
        for sql in ("UPDATE pair_faults SET reason='changed-parent' WHERE id=15",
                    "UPDATE pair_runs SET max_steps=999 WHERE run_id='repaired-new-round'",
                    "UPDATE pair_events SET payload_digest='changed-parent' WHERE event_id='left-clear-plug-up-1'"):
            with sqlite3.connect(self.path) as db:
                db.row_factory = sqlite3.Row
                db.execute('BEGIN'); db.execute(sql)
                with self.subTest(sql=sql),self.assertRaises(PairLedgerError): self.audit(db)
                db.rollback()

    def test_historical_closure_paths_are_cwd_independent_and_hash_bound(self):
        manifest = json.loads(self.close.read_text())
        for key in manifest.keys()-{'schema'}:
            manifest[key] = Path(manifest[key]).name
        self.close.write_text(json.dumps(manifest))
        previous = Path.cwd()
        try:
            os.chdir(self.root)
            self.activate(self.prepare())
        finally:
            os.chdir(previous)
        self.assertTrue(self.audit()['historical_observations_only'])
        source = self.root/'session_log.json'
        source.write_bytes(source.read_bytes()+b'\n')
        with self.assertRaises(PairLedgerError): self.audit()

    def test_nonchild_revision_and_modified_original_authorization_refuse(self):
        self.activate(self.prepare())
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            db.execute('BEGIN')
            db.execute('CREATE TABLE pair_unopened_round_revisions(run_id TEXT PRIMARY KEY, record_json TEXT)')
            db.execute('INSERT INTO pair_unopened_round_revisions VALUES(?,?)',('different-run','{}'))
            with self.assertRaises(PairLedgerError): self.audit(db)
            db.rollback()
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            row = db.execute("SELECT * FROM pair_rounds WHERE run_id='repaired-new-round'").fetchone()
            record = json.loads(row['record_json'])
            record['authorization']['statement'] = ''
            db.execute('UPDATE pair_rounds SET authorization_sha256=?,record_json=? WHERE ordinal=?',
                       (entry._sha(record['authorization']),json.dumps(record),row['ordinal']))
            with self.assertRaises(PairLedgerError): self.audit(db)
