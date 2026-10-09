"""Same-run RGB repair against synthetic ledgers/feedback; no hardware."""
import copy
import json
import sqlite3
import unittest
from unittest.mock import patch

from robot_tools import round_rgb_continuation as entry, pair_round
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
from robot_tools.pair_task_enrollment import preparation_only
import test_pair_tracking_round as fixtures


class RoundRGBContinuationTests(unittest.TestCase):
    def setUp(self):
        f=fixtures.TrackingRoundTests('runTest');f.setUp();self.addCleanup(f.doCleanups)
        self.f=f;self.root=f.root;self.path=f.path;self.now=7000.
        # This fixture models a frozen full manifest from before the repair.
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            row=db.execute("SELECT * FROM pair_rounds WHERE run_id='round-2'").fetchone()
            record=json.loads(row['record_json']);proposal=record['proposal']
            old=entry._current_contract(self.root,record['new_contract'],require_change=False)
            old['code']['pair_host.py']='0'*64;old['code']['pair_joint_adapter.py']='1'*64
            old['code'].pop('round_rgb_continuation.py')
            record['new_contract']=old;proposal['reviewed_contract']=copy.deepcopy(old)
            proposal['proposal_sha256']=entry._sha({k:v for k,v in proposal.items() if k!='proposal_sha256'})
            record['authorization']['proposal_sha256']=proposal['proposal_sha256']
            db.execute("UPDATE pair_runs SET contract_json=? WHERE run_id='round-2'",(json.dumps(old,sort_keys=True,separators=(',',':')),))
            db.execute('UPDATE pair_rounds SET proposal_sha256=?,authorization_sha256=?,record_json=? WHERE ordinal=?',
                (proposal['proposal_sha256'],entry._sha(record['authorization']),json.dumps(record),row['ordinal']))
        r=copy.deepcopy(f.receipt);d=r['device_receipt'];plan=d['joint_path_plan']
        plan['geometry']['evidence']['rgb_received_at']=4016.
        plan['geometry']['source']['sha256']=pair_round._sha(plan['geometry']['evidence'])
        plan['visual_rgb_deadline']=4046.
        plan['plan_sha256']=pair_round._sha({k:v for k,v in plan.items() if k!='plan_sha256'})
        d['original_event'].update(rgb_admission=copy.deepcopy(plan['geometry']),plan_sha256=plan['plan_sha256'])
        error={'type':'RuntimeError','detail':'visual_rgb_expired: original joint RGB deadline reached'}
        d['errors']=[error];d['tracking_observation']['first_failure'].update(**error,sample_role='latest_observation_before_failure')
        d['before']=copy.deepcopy(plan['origin']['arms'])
        f.write_failure(r,f.payload);self.receipt=r
        self.log=self.root/'complete-session.jsonl'
        closed=json.loads(f.close.read_text())
        rows=[dict(sequence=1,at=4020.,kind='result',result=dict(status='owned',open=True,owner=f.f.owner,run_id='round-2')),
              dict(sequence=2,at=4048.,kind='result',result=closed),
              dict(sequence=3,at=4049.,kind='session_ended',cleanup_errors=[])]
        self.log.write_text(''.join(json.dumps(x)+'\n' for x in rows))
        for path in [*f.passive.values(),f.rgb]:
            data=json.loads(path.read_text());f.f.f.f.shift(data,-1000.)
            if path in f.passive.values():
                side=next(s for s,p in f.passive.items() if p==path)
                pose=d['before'][side]['pose_m_rad']
                import math
                for sample in data['pose_trace']:
                    sample['end_pose_raw']={k:round(v/(1e-6 if i<3 else math.pi/180000))
                        for i,(k,v) in enumerate(zip(('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'),pose))}
                data['feedback']['PiperMsgGripperFeedBack']['fields']['grippers_angle']=round(d['before'][side]['gripper']['width_m']*1e6)
            path.write_text(json.dumps(data))
        for name in ('socket.socket','robot_tools.round_rgb_continuation.check_processes'):
            p=patch(name,**({'side_effect':AssertionError('Physical sockets forbidden')} if name=='socket.socket' else {'return_value':None}))
            p.start();self.addCleanup(p.stop)

    def prepare(self):
        return entry.prepare(self.path,'round-2',session_log=self.log,passive_paths=self.f.passive,rgb_observation=self.f.rgb,
            visual_observation='Synthetic unloaded arms; objects on table; unchanged peer and no grasp.',clock=lambda:self.now)

    def activate(self,p):
        return entry.activate(p,project_root=self.root,clock=lambda:self.now)

    def test_same_budget_same_run_old_rows_preserved_and_no_old_owner_or_replay(self):
        before=self.f.rows();p=self.prepare();self.assertEqual(before,self.f.rows())
        result=self.activate(p);after=self.f.rows()
        for name,rows in before.items():self.assertTrue(all(r in after[name] for r in rows),name)
        self.assertEqual(before['pair_runs'],after['pair_runs'])
        self.assertEqual(before['pair_faults'],after['pair_faults'])
        self.assertFalse(result['new_budget_allocated'])
        self.assertTrue(preparation_only(self.path,'round-2'))
        self.assertTrue(activated_execution_budget(self.path,'round-2',max_steps=500,max_duration_s=3600))
        ledger=PairLedger(self.path,'round-2',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):ledger.claim(self.f.f.owner)
        state=ledger.claim('new-preparation-owner');self.assertEqual(state['deadline_s'],7600.)
        self.assertEqual(state['steps'],5)
        ledger.begin('new-preparation-owner','new-query',{'kind':'query'});ledger.finish('new-preparation-owner','new-query',{'ok':True})
        self.assertTrue(activated_execution_budget(self.path,'round-2',max_steps=500,max_duration_s=3600))
        with self.assertRaises(PairLedgerError):self.activate(p)
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_old_failure_alteration_invalidates_budget_recognition(self):
        self.activate(self.prepare())
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET success=1 WHERE event_id='rgb-failed'")
        with self.assertRaises(PairLedgerError):activated_execution_budget(self.path,'round-2',max_steps=500,max_duration_s=3600)

    def test_unknown_partial_grasp_or_other_fault_refuses(self):
        original=copy.deepcopy(self.receipt)
        for change in (lambda d:d['original_event']['frame_receipts'].pop(),
                       lambda d:d['original_event']['frame_receipts'][0].update(outcome='unknown'),
                       lambda d:d['joint_path_plan'].update(loaded_observation_only=True),
                       lambda d:d['errors'][0].update(detail='driver fault')):
            r=copy.deepcopy(original);change(r['device_receipt']);self.f.write_failure(r)
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.prepare()

    def test_fresh_stationary_feedback_does_not_allow_new_pose_or_jaw_target(self):
        path=self.f.passive['right'];original=path.read_text();d=json.loads(original)
        for sample in d['pose_trace']:sample['joints_raw']['joint_1']+=1000
        path.write_text(json.dumps(d))
        with self.assertRaisesRegex(PairLedgerError,'fully returned target'):self.prepare()
        d=json.loads(original);d['feedback']['PiperMsgGripperFeedBack']['fields']['grippers_angle']+=1000
        path.write_text(json.dumps(d))
        with self.assertRaisesRegex(PairLedgerError,'jaw changed'):self.prepare()

    def test_expired_budget_stale_evidence_or_changed_sources_cannot_activate(self):
        p=self.prepare();self.now+=31
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.now=7600.
        with self.assertRaisesRegex(PairLedgerError,'Original budget'):self.prepare()
        self.now=7000.;source=self.root/'robot_tools/pair_joint_adapter.py';source.write_text(source.read_text()+'\n# changed after review\n')
        with self.assertRaises(PairLedgerError):self.activate(p)
