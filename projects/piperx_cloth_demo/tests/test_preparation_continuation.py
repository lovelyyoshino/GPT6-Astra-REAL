"""Offline same-budget continuation of the exact preparation-close race."""
import copy
import json
from pathlib import Path
import shutil
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import preparation_continuation as entry
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget, platform_fault
from robot_tools.pair_host import PairHost

FIXTURE = Path(__file__).parent/'fixtures/preparation_close_audit.json'


class PreparationContinuationTests(unittest.TestCase):
    def setUp(self):
        for target in ('socket.socket','robot_tools.arms._load_sdk','robot_tools.cameras.capture_cameras'):
            p=patch(target,side_effect=AssertionError('Offline audit only'));p.start();self.addCleanup(p.stop)
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)/'projects/piperx_cloth_demo';(self.root/'runs').mkdir(parents=True)
        (self.root/'configs').mkdir();self.path=self.root/'runs/pair_sessions.sqlite'
        self.data=json.loads(FIXTURE.read_text());self.run=self.data['run_id']
        with sqlite3.connect(self.path) as db:
            for name,sql in self.data['schema'].items():
                db.execute(sql)
                for row in self.data['tables'][name]:
                    db.execute('INSERT INTO '+name+' ('+','.join(row)+') VALUES ('+','.join('?' for _ in row)+')',list(row.values()))
        old=json.loads(next(r['contract_json'] for r in self.data['tables']['pair_runs'] if r['run_id']==self.run))
        self.profile={k:old[k] for k in ('arms','cameras','sdk_commit_audited')}
        (self.root/'configs/robot.json').write_text(json.dumps(self.profile))
        shutil.copytree(Path(entry.__file__).parent,self.root/'robot_tools',ignore=shutil.ignore_patterns('__pycache__'))
        self.session=self.root/'session.jsonl';self.journal=self.root/'events.jsonl'
        self.write_lines(self.session,self.data['session']);self.write_lines(self.journal,self.data['journal'])
        ended=self.data['session'][-1]['at'];self.now=ended+12.
        self.passive={}
        def shift(x,offset):
            if isinstance(x,dict):return {k:shift(v,offset) for k,v in x.items()}
            if isinstance(x,list):return [shift(v,offset) for v in x]
            if type(x) in (int,float) and 1e9<x<2e9:return x+offset
            return x
        for side,data in self.data['passive'].items():
            data=shift(data,self.now-1-data['finished_at_s'])
            p=self.root/(side+'.json');p.write_text(json.dumps(data));self.passive[side]=p
        rgb={'capture_id':'synthetic-current','cameras':{}}
        for view,key in (('front','front'),('left_hand','left_wrist'),('right_hand','right_wrist')):
            p=self.root/(view+'.png');p.write_bytes(b'\x89PNG\r\n\x1a\nSynthetic RGB evidence, not a real scene')
            rgb['cameras'][view]={'serial':old['cameras'][key],'rgb_path':str(p),'host_received_at':self.now-1,
                                   'frame_number':1,'depth_enabled':False}
        self.rgb=self.root/'rgb.json';self.rgb.write_text(json.dumps(rgb))
        for target,value in (('check_processes',None),('_lock_roots',[self.root])):
            p=patch.object(entry,target,return_value=value);p.start();self.addCleanup(p.stop)

    def write_lines(self,path,rows):path.write_text(''.join(json.dumps(r)+'\n' for r in rows))

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            return {name:sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=entry._json_sort)
                    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'").fetchall()}

    def prepare(self):
        return entry.prepare(self.path,self.run,session_log=self.session,journal_path=self.journal,
            passive_paths=self.passive,rgb_observation=self.rgb,
            visual_observation='Synthetic empty jaws; plug and strip independently supported; no contact load',clock=lambda:self.now)

    def activate(self,p):return entry.activate(p,project_root=self.root,clock=lambda:self.now)

    def test_activation_preserves_all_history_budget_and_rejects_old_owners_and_contract(self):
        before=self.rows();p=self.prepare();self.assertEqual(before,self.rows());r=self.activate(p)
        after=self.rows()
        for name,rows in before.items():self.assertEqual(rows,after[name],name)
        self.assertFalse(r['new_budget_allocated']);self.assertFalse(r['cache_or_limits_transferred'])
        self.assertEqual(r['hardware_commands_sent'],0)
        self.assertIsNotNone(platform_fault(self.path));self.assertIsNone(platform_fault(self.path,run_id=self.run))
        self.assertTrue(activated_execution_budget(self.path,self.run,max_steps=1000,max_duration_s=10800))
        ledger=PairLedger(self.path,self.run,r['new_contract'],max_steps=1000,max_duration_s=10800,clock=lambda:self.now)
        self.assertEqual(ledger.peek_status()['steps'],3)
        self.assertEqual(ledger.peek_status()['deadline_s'],p['budget']['deadline_s'])
        for owner in p['snapshot']['retired_owners']:
            with self.assertRaises(PairLedgerError):ledger.claim(owner)
        with self.assertRaises(ValueError):
            PairLedger(self.path,self.run,p['snapshot']['contract'],max_steps=1000,max_duration_s=10800,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):
            PairLedger(self.path,'other-run',r['new_contract'],clock=lambda:self.now)
        for mode in ('ready','prepare'):
            args=dict(device_factory=lambda *a:(_ for _ in ()).throw(AssertionError('No hardware')),
                      clock=lambda:self.now,background=False,connection_mode=mode)
            if mode=='ready':
                with self.assertRaisesRegex(ValueError,'preparation connection'):
                    PairHost(self.root/'runs',self.profile,self.run,r['new_contract']['task'],1000,10800,**args)
            else:
                host=PairHost(self.root/'runs',self.profile,self.run,r['new_contract']['task'],1000,10800,**args)
                self.assertIsNone(host.device)
        ledger.claim('new-test-owner')
        e=p['snapshot']['events'][-1]
        replay=ledger.begin('new-test-owner',e['event_id'],json.loads(e['payload_json']))
        self.assertTrue(replay['replayed']);self.assertEqual(ledger.peek_status()['steps'],3)
        with self.assertRaises(PairLedgerError):self.activate(p)

    def test_other_fault_or_pending_or_joint_history_refused(self):
        for mutation in ('fault','pending','kind','partial'):
            with self.subTest(mutation=mutation):
                self.setUp_variant(mutation)
                with self.assertRaises(PairLedgerError):self.prepare()
                self.restore_variant()

    def setUp_variant(self,mutation):
        self.saved=self.rows()
        with sqlite3.connect(self.path) as db:
            if mutation=='fault':db.execute('UPDATE pair_faults SET reason=? WHERE run_id=?',('actual feedback fault',self.run))
            elif mutation=='pending':db.execute("UPDATE pair_events SET status='pending' WHERE run_id=? AND step=3",(self.run,))
            elif mutation=='kind':
                row=self.saved['pair_events'][-1];p=json.loads(row['payload_json']);p['kind']='joint'
                db.execute('UPDATE pair_events SET payload_json=? WHERE run_id=? AND step=3',(json.dumps(p),self.run))
            else:
                row=next(r for r in self.saved['pair_events'] if r['step']==3);r=json.loads(row['receipt_json'])
                r['transmission_counts']['right']['sent_frames']=0
                db.execute('UPDATE pair_events SET receipt_json=? WHERE run_id=? AND step=3',(json.dumps(r),self.run))

    def restore_variant(self):
        with sqlite3.connect(self.path) as db:
            for name in ('pair_events','pair_faults'):
                db.execute('DELETE FROM '+name)
                for row in self.saved[name]:db.execute('INSERT INTO '+name+' ('+','.join(row)+') VALUES ('+','.join('?' for _ in row)+')',list(row.values()))

    def test_close_time_wire_or_unexpected_request_refused(self):
        for mutation in ('outside_close','wire','request'):
            rows=copy.deepcopy(self.data['session']);journal=copy.deepcopy(self.data['journal'])
            if mutation=='outside_close':
                row=next(r for r in rows if r['kind']=='request' and r['request']['op']=='robot_pair_close');row['at']+=.1
            elif mutation=='wire':next(r for r in journal if r['event']=='single_gripper_prepare_intent')['data_hex']='00'*8
            else:next(r for r in rows if r['kind']=='request' and r['request']['op']=='capture')['request']['op']='robot_pair_submit_once'
            self.write_lines(self.session,rows);self.write_lines(self.journal,journal)
            with self.subTest(mutation=mutation),self.assertRaises(PairLedgerError):self.prepare()

    def test_changed_code_evidence_and_deadline_do_not_write_scope(self):
        p=self.prepare();before=self.rows()
        source=self.root/'robot_tools/service.py';raw=source.read_bytes();source.write_bytes(raw+b'\n# changed\n')
        with self.assertRaises(PairLedgerError):self.activate(p)
        source.write_bytes(raw);self.assertEqual(before,self.rows())
        self.now+=31
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.rows())
        self.now=p['budget']['deadline_s']
        with self.assertRaises(PairLedgerError):self.prepare()
        self.assertEqual(before,self.rows())

    def test_current_overbound_or_stale_feedback_does_not_admit(self):
        p=self.passive['right'];base=json.loads(p.read_text())
        for mutation in ('overbound','stale','tx'):
            d=copy.deepcopy(base)
            if mutation=='overbound':d['pose_trace'][10]['joints_raw']['joint_4']+=1000
            elif mutation=='stale':d['pose_trace'][10]['field_received_at_s']['joint_4']-=1
            else:d['frames_sent_by_this_script']=1
            p.write_text(json.dumps(d))
            with self.subTest(mutation=mutation),self.assertRaises(PairLedgerError):self.prepare()

    def test_unrelated_controller_change_or_budget_extension_refused(self):
        p=self.root/'robot_tools/pair_device.py';raw=p.read_bytes();p.write_bytes(raw+b'\n# unrelated change\n')
        with self.assertRaisesRegex(PairLedgerError,'Unrelated controller'):self.prepare()
        p.write_bytes(raw)
        with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_runs SET max_duration=max_duration+10 WHERE run_id=?',(self.run,))
        with self.assertRaisesRegex(PairLedgerError,'budget authorization'):self.prepare()
