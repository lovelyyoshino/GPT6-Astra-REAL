"""Offline same-deadline repair enrollment; synthetic receipts, no hardware."""
import copy
import json
import sqlite3
import unittest
from robot_tools import pair_round
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
from test_pair_round import PairRoundTests

class FreshnessRoundTests(unittest.TestCase):
    def setUp(self):
        self.f=PairRoundTests('runTest'); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.root,self.path=self.f.root,self.f.path
        host=self.root/'robot_tools/pair_host.py'
        host.write_text(host.read_text().replace("'pair_round.py'", "'pair_round.py','joint_path.py'"))
        (self.root/'robot_tools/joint_path.py').write_text('# old geometry\n')
        activation=self.f.activate();self.now=4020.
        self.ledger=PairLedger(self.path,'round-2',activation['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        self.owner='freshness-owner';self.ledger.claim(self.owner)
        self.totals={s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in ('left','right')}
        # A successful target before the failed next target is retained.
        event=next(e for e in self.f.rows()['pair_events'] if e['event_id']=='successful-5')
        payload=json.loads(event['payload_json']);receipt=json.loads(event['receipt_json'])
        receipt['original_event']['event_id']='success-before-failure'
        for frame in receipt['original_event']['frame_receipts']:frame['returned_at']+=self.now-event['began_at']
        self.totals=copy.deepcopy(receipt['transmission_counts']);receipt['session_transmission_counts']=copy.deepcopy(self.totals)
        self.ledger.begin(self.owner,'success-before-failure',payload);self.now+=1
        self.ledger.finish(self.owner,'success-before-failure',receipt)
        base=next(e for e in self.f.rows()['pair_events'] if e['event_id']=='failed-zero')
        self.payload=json.loads(base['payload_json']);self.receipt=json.loads(base['receipt_json'])
        d=self.receipt['device_receipt'];plan=d['joint_path_plan'];ident=plan['identity']
        ident.update(run_id='round-2',owner=self.owner,epoch=self.owner,worker_id='failed-current')
        plan['geometry']['schema']='piper_rgb_supervised_coarse_approach_v1'
        plan['plan_sha256']=pair_round._sha({k:v for k,v in plan.items() if k!='plan_sha256'})
        d['tracking_observation']['first_failure']['sample'].update(identity=copy.deepcopy(ident),captured_at=4022.5)
        d['session_transmission_counts']=copy.deepcopy(self.totals);self.receipt['event_id']='failed-current'
        self.now=4022.;self.ledger.begin(self.owner,'failed-current',self.payload);self.now=4023.
        self.ledger.fault(self.owner,'Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch')
        self.ledger.finish(self.owner,'failed-current',self.receipt,success=False)
        self.close=self.root/'freshness-close.json';closed=json.loads(self.f.close.read_text())
        closed['fault_latched']=True;closed['cleanup']['session_transmission_counts']=self.totals
        self.close.write_text(json.dumps(closed))
        previous=self.f.f
        self.passive=previous.passive;self.rgb=previous.rgb
        for path in [*self.passive.values(),self.rgb]:
            data=json.loads(path.read_text());previous.shift(data,3790.);path.write_text(json.dumps(data))
        for name in ('joint_path.py','pair_joint_adapter.py'):(self.root/'robot_tools'/name).write_text('# repaired computation\n')
        self.now=4060.

    def prepare(self,**kwargs):
        args=dict(close_log=self.close,new_run_id='continued',started_at=4050.,max_steps=498,max_duration_s=3550.,
            budget_start_policy='preserve_parent_deadline',parent_kind='zero_tx_freshness_fault',clock=lambda:self.now,
            recovery_evidence=dict(passive_paths=self.passive,rgb_observation=self.rgb,visual_observation='Synthetic empty jaws and no contact.'))
        args.update(kwargs);return pair_round.prepare_round(self.path,'round-2',**args)

    def activate(self,p):
        auth=dict(source='user_message',message_id='repair-request',statement='Repair and continue this task',received_at=4040.,
            decision='authorize_repaired_continuation',proposal_sha256=p['proposal_sha256'],new_budget=p['new_budget'],
            budget_start_policy='preserve_parent_deadline')
        return pair_round.activate_round(p,auth,project_root=self.root,clock=lambda:self.now)

    def test_deadline_allowance_fault_history_and_retired_owner(self):
        before=self.f.rows();p=self.prepare();result=self.activate(p);after=self.f.rows()
        self.assertFalse(result['new_budget_allocated'])
        for name,rows in before.items():
            self.assertEqual(rows,[r for r in after[name] if not (name in ('pair_runs','pair_rounds') and r['run_id']=='continued')],name)
        new=PairLedger(self.path,'continued',result['new_contract'],max_steps=498,max_duration_s=3550.,clock=lambda:self.now)
        self.assertEqual(new.peek_status()['deadline_s'],7600.)
        self.assertTrue(activated_execution_budget(self.path,'continued',max_steps=498,max_duration_s=3550.))
        with self.assertRaises(PairLedgerError):new.claim(self.owner)
        with self.assertRaises(PairLedgerError):self.ledger.begin(self.owner,'replay',self.payload)

    def test_never_extend_deadline_steps_or_change_policy(self):
        for extra in ({'max_duration_s':3551.},{'max_steps':499},{'budget_start_policy':'after_repair_before_online_execution'}):
            with self.subTest(extra=extra),self.assertRaises(PairLedgerError):self.prepare(**extra)

    def test_partial_send_unrelated_error_or_fault_refuse(self):
        baseline=copy.deepcopy(self.receipt)
        for mutate in (lambda d:d['transmission_counts']['left'].update(attempted_frames=1),
                       lambda d:d.update(target_calls_sent=1),lambda d:d.update(original_event={}),
                       lambda d:d['errors'][0].update(detail='different'),lambda d:d['joint_path_plan'].update(loaded_context={})):
            value=copy.deepcopy(baseline);mutate(value['device_receipt'])
            with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(value),'failed-current'))
            with self.assertRaises(PairLedgerError):self.prepare()
        with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(baseline),'failed-current'))
        self.ledger.fault(self.owner,'additional fault')
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_changed_evidence_code_or_history_refuse(self):
        p=self.prepare();before=self.f.rows()
        path=self.root/'robot_tools/joint_path.py';original=path.read_text();path.write_text('# changed again')
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.f.rows());path.write_text(original)
        data=json.loads(self.rgb.read_text());data['cameras']['front']['host_received_at']=4000.;self.rgb.write_text(json.dumps(data))
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.f.rows())

if __name__=='__main__':unittest.main()
