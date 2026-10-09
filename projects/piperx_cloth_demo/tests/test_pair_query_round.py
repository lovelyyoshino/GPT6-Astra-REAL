"""Query-only timing restart: temporary DBs and files, no device calls."""
import copy
import json
import unittest

from robot_tools import pair_round
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
from test_pair_round import PairRoundTests


class QueryRoundTests(unittest.TestCase):
    def setUp(self):
        self.f = PairRoundTests('runTest'); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.root, self.path = self.f.root, self.f.path
        host = self.root/'robot_tools/pair_host.py'
        host.write_text(host.read_text().replace("'pair_round.py'", "'pair_round.py','pair_limits.py','joint_sources.py'"))
        for name in ('pair_limits.py','joint_sources.py'):
            (self.root/'robot_tools'/name).write_text('# prior receiver\n')
        initial = self.f.activate(); self.now = 4020.
        self.ledger = PairLedger(self.path,'round-2',initial['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        self.owner = 'query-owner'; self.ledger.claim(self.owner)
        bindings = json.loads(next(e['payload_json'] for e in self.f.rows()['pair_events']
                                  if e['event_id']=='query-healthy'))['bindings']
        payload = {'kind':'query','request':{'operation':'inspect_joint_limits'},'bindings':bindings}
        self.ledger.begin(self.owner,'query-failed',payload)
        frame = {'dlc':8,'payload_hex':'0105dcfa24012c00','timestamp':4020.03,'received_unix_s':4020.031}
        second = {**frame,'timestamp':4020.030096,'received_unix_s':4020.031096}
        counts = {s:{'attempted_frames':int(s=='left'),'sent_frames':int(s=='left'),'blocked_frames':0} for s in ('left','right')}
        error = {'type':'RuntimeError','detail':'Unexpected or duplicate joint-limit response'}
        self.device = {'schema':'piper_pair_controller_limits_capture_v1','operation':'inspect_joint_limits',
            'ok':False,'status':'joint_limits_capture_failed','fault_latched':True,
            'controller_limits_changed':False,'sdk_joint_limits_changed':False,
            'began_at':4020.01,'ended_at':4020.04,'transmission_counts':counts,'session_transmission_counts':counts,
            'errors':[error,error], 'guard_violations':[{'side':'left','detail':error['detail'],'arbitration_id':0x473}],
            'pre_or_post_window_limit_frames':{'left':0,'right':0},'controller_limits_rad':{'left':[],'right':[]},
            'joint_limits':{'right':{},'left':{'1':{'status':'unconfirmed','raw_response_hex':frame['payload_hex'],
                'response_evidence':{'active':False,'request_started_unix_s':4020.02,'finished_unix_s':4020.039,
                    'ignored_stale_frames':[],'response_frames':[frame],'rejected_frames':[second]}}}},
            'query_receipts':{'right':{},'left':{'1':{'arbitration_id':0x472,'data_hex':'0101000000000000',
                'outcome':'returned','sent_at':4020.02,'returned_at':4020.021}}}}
        self.device.update({k:0 for k in ('actuator_commands_sent','target_commands_sent','mode_commands_sent','enable_commands_sent','stop_commands_sent')})
        self.device.update({k:1 for k in ('hardware_commands_sent','joint_limit_queries_sent','joint_limit_queries_attempted')})
        self.receipt = {'ok':False,'automatic_retry':False,'event_id':'query-failed','device_receipt':self.device}
        self.now = 4020.05
        self.ledger.fault(self.owner,'Claimed preparation/query failed or uncertain: Controller limit query incomplete or uncertain')
        self.ledger.finish(self.owner,'query-failed',self.receipt,success=False)
        status = {'run_id':'round-2','owner':self.owner,'open':False,'active_event_id':None,
            'fault_latched':True,'fault_feedback_read_state':'closed','unresolved_gripper_probe':None,
            'grasp_states':{'left':None,'right':None},'ledger':self.ledger.peek_status()}
        documents = {'failed_query':{'event_id':'query-failed','status':'fault','receipt':self.receipt},
            'close_error':{'ok':False,'error':'PairHostError: Device cleanup failed or attempted an unexpected transmission'},
            'closed_status':status,'process_exit':{'schema':'piper_control_process_exit_v1','run_id':'round-2',
                'owner':self.owner,'exit_code':0,'observed_at':4021.,'source_ref':'Synthetic unit-test exit'}}
        self.manifest = {'schema':'piper_query_fault_exit_evidence_v1'}
        for key,value in documents.items():
            path = self.root/(key+'.json');path.write_text(json.dumps(value));self.manifest[key] = str(path)
        self.evidence = self.root/'exit-manifest.json'; self.evidence.write_text(json.dumps(self.manifest))
        for name in ('pair_limits.py','joint_sources.py'):
            (self.root/'robot_tools'/name).write_text('# repaired receiver\n')
        self.now = 4040.

    def prepare(self, **extra):
        args = dict(close_log=self.evidence,new_run_id='round-3',started_at=4030.,max_steps=499,max_duration_s=3600,
                    budget_start_policy='after_repair_before_online_execution',parent_kind='query_duplicate_fault',clock=lambda:self.now)
        args.update(extra)
        return pair_round.prepare_round(self.path,'round-2',**args)

    def authorize(self,p):
        return {'source':'user_message','message_id':'repair-clock-request','statement':'Restart timer after this repair',
                'received_at':4022.,'decision':'authorize_explicit_new_round','proposal_sha256':p['proposal_sha256'],
                'new_budget':p['new_budget'],'budget_start_policy':'after_repair_before_online_execution'}

    def test_restart_preserves_all_old_rows_and_unused_steps(self):
        before = self.f.rows(); p = self.prepare()
        result = pair_round.activate_round(p,self.authorize(p),project_root=self.root,clock=lambda:self.now)
        after = self.f.rows()
        for name,rows in before.items():
            retained = [r for r in after[name] if not (name=='pair_runs' and r['run_id']=='round-3')
                        and not (name=='pair_rounds' and r['run_id']=='round-3')]
            self.assertEqual(rows,retained,name)
        new = PairLedger(self.path,'round-3',result['new_contract'],max_steps=499,max_duration_s=3600,clock=lambda:self.now)
        self.assertEqual(new.peek_status()['deadline_s'],7630.)
        self.assertEqual(new.peek_status()['remaining_steps'],499)
        self.assertTrue(activated_execution_budget(self.path,'round-3',max_steps=499,max_duration_s=3600))
        with self.assertRaises(PairLedgerError):new.claim(self.owner)
        with self.assertRaises(PairLedgerError):self.ledger.begin(self.owner,'old-owner-replay',{})

    def test_budget_expansion_default_policy_and_stale_authorization_refuse(self):
        for extra in ({'max_steps':500},{'max_duration_s':3599},{'budget_start_policy':'include_repair_time'}):
            with self.assertRaises(PairLedgerError):self.prepare(**extra)
        p=self.prepare();a=self.authorize(p);a['received_at']=4019.
        with self.assertRaises(PairLedgerError):pair_round.activate_round(p,a,project_root=self.root,clock=lambda:self.now)

    def test_actuation_partial_conflict_and_extra_fault_refuse(self):
        original=copy.deepcopy(self.receipt)
        import sqlite3
        changes = (lambda d:d.update(actuator_commands_sent=1),
                   lambda d:d['query_receipts']['left']['1'].update(outcome='uncertain'),
                   lambda d:d['joint_limits']['left']['1']['response_evidence']['rejected_frames'][0].update(payload_hex='0105dcfa24012d00'))
        for change in changes:
            altered=copy.deepcopy(original);change(altered['device_receipt'])
            with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(altered),'query-failed'))
            with self.assertRaises(PairLedgerError):self.prepare()
        with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(original),'query-failed'))
        self.ledger.fault(self.owner,'different failure')
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_close_exit_and_source_changes_cannot_be_substituted(self):
        p=self.prepare();path=self.root/'process_exit.json';raw=path.read_text()
        obj=json.loads(raw);obj['exit_code']=1;path.write_text(json.dumps(obj))
        with self.assertRaises(PairLedgerError):self.prepare()
        path.write_text(raw)
        (self.root/'robot_tools/pair_limits.py').write_text('# changed after review\n')
        with self.assertRaises(PairLedgerError):pair_round.activate_round(p,self.authorize(p),project_root=self.root,clock=lambda:self.now)


if __name__ == '__main__': unittest.main()
