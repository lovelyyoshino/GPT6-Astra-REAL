"""Offline same-budget successor; no live devices or original ledger writes."""
import copy
import json
from pathlib import Path
import shutil
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import feedback_continuation as entry
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget, platform_fault
from robot_tools.pair_host import PairHost


class FeedbackContinuationTests(unittest.TestCase):
    def setUp(self):
        for target in ('socket.socket','robot_tools.arms._load_sdk','robot_tools.cameras.capture_cameras'):
            p=patch(target,side_effect=AssertionError('Offline only'));p.start();self.addCleanup(p.stop)
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup)
        self.root=Path(t.name)/'projects/piperx_cloth_demo';(self.root/'runs').mkdir(parents=True)
        (self.root/'configs').mkdir();self.path=self.root/'runs/pair_sessions.sqlite'
        self.data=json.loads((Path(__file__).parent/'fixtures/feedback_continuation_audit.json').read_text());self.run=self.data['run_id']
        with sqlite3.connect(self.path) as db:
            for name,sql in self.data['schema'].items():
                db.execute(sql)
                for row in self.data['tables'][name]:self.insert(db,name,row)
        old=json.loads(self.data['tables']['pair_preparation_continuations'][0]['contract_json'])
        self.profile={k:old[k] for k in ('arms','cameras','sdk_commit_audited')}
        (self.root/'configs/robot.json').write_text(json.dumps(self.profile))
        shutil.copytree(Path(entry.__file__).parent,self.root/'robot_tools',ignore=shutil.ignore_patterns('__pycache__'))
        self.session=self.root/'session.jsonl';self.journal=self.root/'events.jsonl'
        self.lines(self.session,self.data['session']);self.lines(self.journal,self.data['journal'])
        self.now=self.data['session'][-1]['at']+12.;self.passive={}
        def shift(x,offset):
            if isinstance(x,dict):return {k:shift(v,offset) for k,v in x.items()}
            if isinstance(x,list):return [shift(v,offset) for v in x]
            if type(x) in (int,float) and 1e9<x<2e9:return x+offset
            return x
        for side,data in self.data['passive'].items():
            data=shift(data,self.now-1-data['finished_at_s']);p=self.root/(side+'.json');p.write_text(json.dumps(data));self.passive[side]=p
        rgb={'capture_id':'synthetic-current','cameras':{}}
        for view,key in (('front','front'),('left_hand','left_wrist'),('right_hand','right_wrist')):
            p=self.root/(view+'.png');p.write_bytes(b'\x89PNG\r\n\x1a\nSynthetic fixture, not a scene')
            rgb['cameras'][view]=dict(serial=old['cameras'][key],rgb_path=str(p),host_received_at=self.now-1,frame_number=1,depth_enabled=False)
        self.rgb=self.root/'rgb.json';self.rgb.write_text(json.dumps(rgb))
        for target,value in (('check_processes',None),('_lock_roots',[self.root])):
            p=patch.object(entry,target,return_value=value);p.start();self.addCleanup(p.stop)

    def insert(self,db,name,row):db.execute('INSERT INTO '+name+' ('+','.join(row)+') VALUES ('+','.join('?' for _ in row)+')',list(row.values()))
    def lines(self,path,rows):path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            return {name:sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=entry._json_sort)
                    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'").fetchall()}
    def prepare(self):
        return entry.prepare(self.path,self.run,session_log=self.session,journal_path=self.journal,passive_paths=self.passive,
            rgb_observation=self.rgb,visual_observation='Synthetic empty jaws and independently supported objects; no contact load',clock=lambda:self.now)
    def activate(self,p):return entry.activate(p,project_root=self.root,clock=lambda:self.now)

    def test_preserves_history_budget_and_blocks_old_contract_owners_and_reactivation(self):
        before=self.rows();p=self.prepare();self.assertEqual(before,self.rows());r=self.activate(p);after=self.rows()
        for table,rows in before.items():self.assertEqual(rows,after[table],table)
        self.assertFalse(r['new_budget_allocated']);self.assertFalse(r['cache_or_limits_transferred']);self.assertEqual(r['hardware_commands_sent'],0)
        self.assertIsNotNone(platform_fault(self.path));self.assertIsNone(platform_fault(self.path,run_id=self.run))
        self.assertTrue(activated_execution_budget(self.path,self.run,max_steps=1000,max_duration_s=10800))
        ledger=PairLedger(self.path,self.run,r['new_contract'],max_steps=1000,max_duration_s=10800,clock=lambda:self.now)
        self.assertEqual(ledger.peek_status()['steps'],4);self.assertEqual(ledger.peek_status()['deadline_s'],p['budget']['deadline_s'])
        self.assertIn('feedback_continuation',ledger.peek_status())
        for owner in p['snapshot']['retired_owners']:
            with self.assertRaises(PairLedgerError):ledger.claim(owner)
        with self.assertRaises(ValueError):PairLedger(self.path,self.run,p['snapshot']['contract'],max_steps=1000,max_duration_s=10800,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):PairLedger(self.path,'wrong-run',r['new_contract'],clock=lambda:self.now)
        with self.assertRaisesRegex(ValueError,'preparation connection'):
            PairHost(self.root/'runs',self.profile,self.run,r['new_contract']['task'],1000,10800,connection_mode='ready',clock=lambda:self.now,background=False)
        ledger.claim('new-offline-owner');e=p['snapshot']['event']
        self.assertTrue(ledger.begin('new-offline-owner',e['event_id'],json.loads(e['payload_json']))['replayed'])
        self.assertEqual(ledger.peek_status()['steps'],4)
        with self.assertRaises(PairLedgerError):self.activate(p)

    def test_prior_history_change_and_other_fault_are_rejected(self):
        for query in ("UPDATE pair_faults SET reason='other failure' WHERE id=16",
                      "UPDATE pair_faults SET reason='old failure erased' WHERE id=15",
                      "UPDATE pair_events SET status='pending' WHERE step=4 AND run_id='"+self.run+"'",
                      "UPDATE pair_runs SET max_duration=max_duration+1 WHERE run_id='"+self.run+"'"):
            before=self.rows()
            with sqlite3.connect(self.path) as db:db.execute(query)
            with self.subTest(query=query),self.assertRaises(PairLedgerError):self.prepare()
            with sqlite3.connect(self.path) as db:
                for name in before:
                    db.execute('DELETE FROM '+name)
                    for row in before[name]:self.insert(db,name,row)

    def test_unrelated_control_edits_and_post_review_edits_rejected(self):
        p=self.prepare();before=self.rows();source=self.root/'robot_tools/joint_path.py';source.write_bytes(source.read_bytes()+b'\n# changed\n')
        with self.assertRaises(PairLedgerError):self.prepare()
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.rows())

    def test_real_complete_overbound_pose_cannot_use_mixed_frame_recovery(self):
        rows=copy.deepcopy(self.data['journal']);mixed=next(r for r in reversed(rows) if r['event']=='feedback')
        s=mixed['right']['fragment_timestamps_s'];s['end_pose_ryrz']=s['end_pose_zrx']+.0001
        self.lines(self.journal,rows)
        with self.assertRaisesRegex(PairLedgerError,'mixture'):self.prepare()

    def test_extra_targets_or_unaccounted_query_frames_rejected(self):
        rows=copy.deepcopy(self.data['session']);next(r for r in rows if r.get('request',{}).get('op')=='capture')['request']['op']='robot_pair_submit_once'
        self.lines(self.session,rows)
        with self.assertRaises(PairLedgerError):self.prepare()
        self.lines(self.session,self.data['session']);rows=copy.deepcopy(self.data['journal'])
        next(r for r in rows if r['event']=='pair_joint_limit_query_intent')['data_hex']='00'*8
        self.lines(self.journal,rows)
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_stale_current_evidence_and_expired_budget_do_not_activate(self):
        p=self.prepare();before=self.rows();self.now+=31
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.rows());self.now=p['budget']['deadline_s']
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_current_joint_excess_not_waived_by_historical_diagnosis(self):
        p=self.passive['right'];data=json.loads(p.read_text());data['pose_trace'][10]['joints_raw']['joint_4']+=1000;p.write_text(json.dumps(data))
        with self.assertRaises(PairLedgerError):self.prepare()
