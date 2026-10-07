"""Administrative restart on synthetic SQLite history; sockets forbidden."""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import test_joint_sources as fixtures

from robot_tools.pair_ledger import (PairLedger, PairLedgerError, _hold_frames, platform_state, platform_fault,
                                    activated_execution_budget)
from robot_tools.pair_restart import prepare_restart, activate_restart


class RestartTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'runs').mkdir(); (self.root/'configs').mkdir(); (self.root/'robot_tools').mkdir()
        self.path = self.root/'runs/pair_sessions.sqlite'
        self.now = 80.
        self.contract = {'task':{'task_id':'plug','roles':{'left':'task','right':'task'},
            'site_context':{'workspace_clearance':{'source':'user','statement':'Synthetic task clearance statement'}}},
            'arms':{s:{'model':'piper_x','firmware':'default','channel':'can'+str(i),'usb_interface':'port-'+s}
                    for i,s in enumerate(('left','right'))},
            'cameras':{'front':'a'},'sdk_commit_audited':'a'*40,
            'code':{'pair_host.py':'a'*64,'pair_ledger.py':'b'*64}}
        (self.root/'configs/robot.json').write_text(json.dumps(self.contract))
        (self.root/'robot_tools/pair_host.py').write_text("sources = ('pair_host.py','pair_ledger.py','pair_restart.py')\n")
        for name in ('pair_ledger.py','pair_restart.py'):
            (self.root/'robot_tools'/name).write_text('# synthetic repaired code\n')
        self.ledger = PairLedger(self.path,'old',self.contract,max_steps=8,max_duration_s=30,clock=lambda:self.now)
        self.ledger.claim('owner')
        bindings={s:{'connection_id':'connection-'+s,'model':'piper_x','firmware_profile':'default',
                       'channel':v['channel'],'usb_interface':v['usb_interface']} for s,v in self.contract['arms'].items()}
        self.ledger.begin('owner','limits',{'kind':'query','request':{'operation':'inspect_joint_limits'},'bindings':bindings})
        capture=fixtures.JointSourcesTests.make_capture(SimpleNamespace(bindings=bindings))
        capture.update(ok=True,fault_latched=False,event_id='limits',pair_owner='owner',execution_mode='inspect_joint_limits',
            hardware_commands_sent=12,joint_limit_queries_attempted=12,target_commands_sent=0,mode_commands_sent=0,
            enable_commands_sent=0,stop_commands_sent=0,
            transmission_counts={s:{'attempted_frames':6,'sent_frames':6,'blocked_frames':0} for s in bindings},
            session_transmission_counts={s:{'attempted_frames':6,'sent_frames':6,'blocked_frames':0} for s in bindings})
        self.now=95.
        self.ledger.finish('owner','limits',capture)
        self.now = 101.
        self.ledger.begin('owner','init',{'kind':'initialization'})
        self.device = {'operation':'initialize_joint_target','cache_established':False,'ok':False,
            'errors':[{'type':'JointPathError','detail':'joint_tracking_envelope','code':'joint_tracking_envelope'}],
            'guard_violations':[],'automatic_retry':False,'arm':'right',
            'initialization_plan':{'identity':{'run_id':'old','owner':'owner','worker_id':'init'},
                'spatial_admission_mode':'rgb_supervised','target_raw':[100,0,0,200,300,0]},
            'frame_receipts':[{'frame':frame,'outcome':'returned','returned_at':101.1+i*.01}
                              for i,frame in enumerate(_hold_frames([100,0,0,200,300,0]))],
            'transmission_counts':{s:{'attempted_frames':4 if s=='right' else 0,
                                    'sent_frames':4 if s=='right' else 0,'blocked_frames':0} for s in ('left','right')},
            'session_transmission_counts':{s:{'attempted_frames':10 if s=='right' else 6,
                                            'sent_frames':10 if s=='right' else 6,'blocked_frames':0} for s in ('left','right')},
            'hardware_commands_sent':4,'gripper_commands_sent':0,'enable_commands_sent':0,'stop_commands_sent':0,'retries':0}
        self.now=102.
        self.ledger.fault('owner','Claimed preparation/query failed or uncertain: First-target initialization incomplete, uncertain or inconsistent')
        self.ledger.finish('owner','init',{'ok':False,'device_receipt':self.device},success=False)
        close = {'status':'closed','fault_latched':True,'physical_stop_verified':None,
            'cleanup':{'unresolved_gripper_probe':None,'grasp_states':{'left':None,'right':None},
                'requires_fault_latch':False,'guard_violations':[],
                'session_transmission_counts':self.device['session_transmission_counts'],
                'arms':{s:{'status':'disconnected','physically_stopped':None} for s in ('left','right')}}}
        self.close_log=self.root/'close.jsonl'
        self.close_log.write_text(json.dumps({'result':{'content':[{'type':'text','text':json.dumps(close)}]}})+'\n')
        self.now=120.
        sock=patch('socket.socket',side_effect=AssertionError('No hardware/network'))
        sock.start();self.addCleanup(sock.stop)
        proc=patch('robot_tools.pair_restart._live_control_processes',return_value=[])
        proc.start();self.addCleanup(proc.stop)

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            return {name:[dict(r) for r in db.execute('SELECT * FROM '+name)]
                    for name in ('pair_scope','pair_runs','pair_events','pair_faults')}

    def proposal(self):
        return prepare_restart(self.path,'old',close_log=self.close_log,new_run_id='new',
            max_steps=6,max_duration_s=900,clock=lambda:self.now)

    def authorize(self,p):
        return {'source':'user_message','message_id':'synthetic-user-confirmation','statement':'SYNTHETIC explicit new bounded attempt',
            'received_at':self.now,'decision':'authorize_audited_new_attempt',
            'proposal_sha256':p['proposal_sha256'],'new_budget':copy.deepcopy(p['new_budget'])}

    def activate(self,p=None,a=None):
        p=p or self.proposal()
        return activate_restart(p,a or self.authorize(p),project_root=self.root,clock=lambda:self.now)

    def test_prepare_is_read_only_and_retains_failure_and_deadline(self):
        before=self.rows();p=self.proposal()
        self.assertEqual(before,self.rows())
        self.assertEqual(p['parent_run']['started_at']+p['parent_run']['max_duration'],110.)
        self.assertFalse(p['dispatch_authorized']);self.assertIsNone(p['new_authorization'])

    def test_one_successor_preserves_old_scope_events_faults_and_budget(self):
        before=self.rows();result=self.activate();after=self.rows()
        for table in ('pair_scope','pair_events','pair_faults'):
            self.assertEqual(before[table],after[table])
        self.assertEqual(before['pair_runs'][0],after['pair_runs'][0])
        new=PairLedger(self.path,'new',result['new_contract'],max_steps=6,max_duration_s=900,clock=lambda:self.now)
        status=new.claim('new_owner')
        self.assertEqual(status['deadline_s'],1020.)
        self.assertEqual(status['execution_lineage']['cumulative_steps'],2)
        self.assertIsNone(platform_fault(self.path,run_id='new'))
        self.assertIsNotNone(platform_fault(self.path))
        self.assertIsNotNone(platform_fault(self.path,run_id='random'))
        new.begin('new_owner','step',{'kind':'query'})
        new.finish('new_owner','step',{'ok':True})
        self.assertEqual(new.status()['execution_lineage']['cumulative_steps'],3)
        new.release('new_owner')
        self.assertEqual(before['pair_scope'],self.rows()['pair_scope'])

    def test_grant_replay_old_owner_and_random_run_cannot_claim(self):
        p=self.proposal();a=self.authorize(p);result=self.activate(p,a);before=self.rows()
        for call in (lambda:self.activate(p,a),lambda:self.ledger.claim('owner'),
                     lambda:self.ledger.fault('owner','late call'),lambda:self.ledger.status(),
                     lambda:PairLedger(self.path,'random',self.contract,clock=lambda:self.now),
                     lambda:PairLedger(self.path,'old',self.contract,max_steps=8,max_duration_s=30,clock=lambda:self.now)):
            with self.assertRaises(PairLedgerError):call()
        self.assertEqual(before,self.rows())
        self.assertTrue(self.ledger.peek_status()['fault_latched'])

    def test_new_fault_stays_in_new_scope_without_touching_old_rows(self):
        before=self.rows();result=self.activate()
        new=PairLedger(self.path,'new',result['new_contract'],max_steps=6,max_duration_s=900,clock=lambda:self.now)
        new.claim('new_owner');new.fault('new_owner','new genuine fault')
        self.assertEqual(platform_fault(self.path,run_id='new')['reason'],'new genuine fault')
        self.assertEqual(before['pair_scope'],self.rows()['pair_scope'])
        with self.assertRaises(PairLedgerError):new.begin('new_owner','cannot_send',{})

    def test_old_or_mismatched_authorization_refuses_without_writes(self):
        p=self.proposal();a=self.authorize(p);before=self.rows()
        for change in ({'received_at':101.},{'received_at':121.},{'source':'goal_continuation'},
                       {'decision':'continue'},{'new_budget':{'max_steps':128,'max_duration_s':900}},
                       {'proposal_sha256':'0'*64},{'statement':''}):
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.activate(p,{**a,**change})
        self.assertEqual(before,self.rows())

    def test_code_or_history_changed_after_proposal_requires_new_review(self):
        p=self.proposal();a=self.authorize(p)
        (self.root/'robot_tools/pair_restart.py').write_text('# changed after proposal')
        before=self.rows()
        with self.assertRaises(PairLedgerError):self.activate(p,a)
        self.assertEqual(before,self.rows())
        p=self.proposal();a=self.authorize(p)
        self.ledger.fault('owner','additional unexpected fault')
        before=self.rows()
        with self.assertRaises(PairLedgerError):self.activate(p,a)
        self.assertEqual(before,self.rows())

    def test_partial_unknown_extra_frame_and_other_failure_refused(self):
        original=self.rows()['pair_events'][-1]['receipt_json']
        for change in (lambda d:d['frame_receipts'].pop(),
                       lambda d:d['frame_receipts'][2].update(outcome='unknown'),
                       lambda d:d.update(errors=[{'code':'hard_joint_limit'}]),
                       lambda d:d.update(gripper_commands_sent=1),
                       lambda d:d['transmission_counts']['right'].update(sent_frames=5)):
            v=json.loads(original);change(v['device_receipt'])
            with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(v),'init'))
            with self.assertRaises(PairLedgerError):self.proposal()

    def test_held_history_pending_send_or_extra_fault_refuses(self):
        with sqlite3.connect(self.path) as db:
            db.execute('CREATE TABLE pair_grasp_episodes(state_json TEXT)')
            db.execute('INSERT INTO pair_grasp_episodes VALUES(?)',('{"status":"retained_static"}',))
        with self.assertRaises(PairLedgerError):self.proposal()

    def test_live_control_owner_blocks_zero_write_activation(self):
        p=self.proposal();before=self.rows()
        with patch('robot_tools.pair_restart._live_control_processes',return_value=[{'pid':123}]),self.assertRaises(PairLedgerError):
            self.activate(p)
        self.assertEqual(before,self.rows())

    def test_budget_cannot_increase_total_steps_or_use_unexpired_parent(self):
        for kwargs in ({'max_steps':7},{'max_duration_s':901}):
            with self.assertRaises(PairLedgerError):
                prepare_restart(self.path,'old',close_log=self.close_log,new_run_id='new',
                    clock=lambda:self.now,**kwargs)
        self.now=105.
        with self.assertRaises(PairLedgerError):self.proposal()

    def test_rehashed_noncanonical_proposal_cannot_relax_budget(self):
        from robot_tools.pair_restart import _sha
        original=self.proposal();before=self.rows()
        for change in ({'max_steps':128,'max_duration_s':900}, {'max_steps':6,'max_duration_s':9000}):
            p=copy.deepcopy(original);p['new_budget']=change
            p['proposal_sha256']=_sha({k:v for k,v in p.items() if k!='proposal_sha256'})
            with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.rows())

    def test_sql_writes_rollback_on_late_clock_regression(self):
        p=self.proposal();a=self.authorize(p);before=self.rows()
        for times in ((120.,121.,119.),(120.,121.,122.,121.)):
            clock=iter(times)
            with self.assertRaises(PairLedgerError):
                activate_restart(p,a,project_root=self.root,clock=lambda:next(clock))
            self.assertEqual(before,self.rows())
            with sqlite3.connect(self.path) as db:
                self.assertIsNone(db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_execution_epochs'").fetchone())

    def test_query_raw_frame_and_extra_actuator_fields_cannot_hide_behind_counters(self):
        original=self.rows()['pair_events'][0]['receipt_json']
        for mutate in (lambda r:r['query_receipts']['left']['1'].update(arbitration_id=0x151),
                       lambda r:r.update(mode_commands_sent=1),
                       lambda r:r['joint_limits']['right']['6']['response_evidence'].update(response_frames=[])):
            receipt=json.loads(original);mutate(receipt)
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(receipt),'limits'))
            with self.assertRaises((PairLedgerError,ValueError)):self.proposal()

    def test_matching_close_and_init_cannot_hide_extra_lifetime_sends(self):
        row=self.rows()['pair_events'][-1];receipt=json.loads(row['receipt_json'])
        counts=receipt['device_receipt']['session_transmission_counts']['right']
        counts.update(sent_frames=11,attempted_frames=11)
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(receipt),'init'))
        rpc=json.loads(self.close_log.read_text());closed=json.loads(rpc['result']['content'][0]['text'])
        closed['cleanup']['session_transmission_counts']['right']=counts
        rpc['result']['content'][0]['text']=json.dumps(closed)
        self.close_log.write_text(json.dumps(rpc)+'\n')
        with self.assertRaises(PairLedgerError):self.proposal()

    def test_prior_query_owner_close_does_not_replace_final_fault_owner_close(self):
        final=self.close_log.read_text()
        earlier={'result':{'content':[{'type':'text','text':json.dumps({'status':'closed','fault_latched':False})}]}}
        self.close_log.write_text(json.dumps(earlier)+'\n'+final)
        self.assertTrue(self.proposal()['close']['receipt']['fault_latched'])
        self.close_log.write_text(final+json.dumps(earlier)+'\n')
        with self.assertRaises(PairLedgerError):self.proposal()

    def test_source_reader_ignores_unrelated_runtime_source_variables(self):
        p=self.root/'robot_tools/pair_host.py'
        p.write_text(p.read_text()+'\ndef unrelated():\n    sources = runtime_source_reader()\n')
        self.assertIn('pair_restart.py',self.proposal()['reviewed_contract']['code'])

    def explicit_proposal(self, **overrides):
        return prepare_restart(self.path, 'old', close_log=self.close_log, new_run_id='new',
            clock=lambda:self.now, **{'max_steps':500, 'max_duration_s':3600,
            'budget_mode':'explicit_user_budget_request', **overrides})

    def explicit_authorization(self, proposal, **overrides):
        return {**self.authorize(proposal), 'decision':'authorize_explicit_budget_request',
                'received_at':115., 'statement':'SYNTHETIC user asks to restart the budget at 60 minutes and 500 sends',
                **overrides}

    def test_explicit_new_budget_request_before_proposal_preserves_old_history(self):
        before = self.rows()
        proposal = self.explicit_proposal()
        self.assertEqual(self.rows(), before)
        self.assertEqual(proposal['cumulative_step_ceiling'], 502)
        self.assertEqual(proposal['authorization_not_before'], 102.)
        self.assertFalse(activated_execution_budget(self.path,'new',max_steps=500,max_duration_s=3600))
        result = self.activate(proposal, self.explicit_authorization(proposal))
        for name in ('pair_scope','pair_events','pair_faults'):
            self.assertEqual(self.rows()[name], before[name])
        self.assertEqual(self.rows()['pair_runs'][0], before['pair_runs'][0])
        self.assertTrue(activated_execution_budget(self.path,'new',max_steps=500,max_duration_s=3600))
        for run, steps, duration in (('old',500,3600),('other',500,3600),('new',499,3600),('new',500,3599)):
            self.assertFalse(activated_execution_budget(self.path,run,max_steps=steps,max_duration_s=duration))
        ledger = PairLedger(self.path,'new',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        status = ledger.claim('new-owner')
        self.assertEqual(status['deadline_s'],3720.)
        self.assertEqual(status['remaining_steps'],500)
        self.assertEqual(status['execution_lineage']['cumulative_step_ceiling'],502)
        self.assertEqual(status['execution_lineage']['cumulative_steps'],2)
        ledger.begin('new-owner','first',{'kind':'query'})
        ledger.finish('new-owner','first',{'ok':True})
        self.assertEqual(ledger.status()['execution_lineage']['cumulative_steps'],3)
        self.assertEqual(ledger.status()['execution_lineage']['cumulative_step_ceiling'],502)
        with self.assertRaises(PairLedgerError):
            self.activate(proposal,self.explicit_authorization(proposal))
        self.assertEqual(self.rows()['pair_runs'][0],before['pair_runs'][0])

    def test_expanded_budget_requires_actual_postfault_user_request_matching_exact_hash_and_budget(self):
        proposal=self.explicit_proposal(); authorization=self.explicit_authorization(proposal); before=self.rows()
        for changes in ({'decision':'authorize_audited_new_attempt'}, {'source':'goal_continuation'},
                {'decision':'continue'}, {'received_at':101.9}, {'received_at':102.}, {'received_at':121.},
                {'new_budget':{'max_steps':499,'max_duration_s':3600}},
                {'new_budget':{'max_steps':500.,'max_duration_s':3600}}, {'proposal_sha256':'0'*64}):
            with self.subTest(changes=changes),self.assertRaises(PairLedgerError):
                self.activate(proposal,{**authorization,**changes})
            self.assertEqual(self.rows(),before)
        # A new decision cannot relabel a proposal made under the old policy.
        legacy=self.proposal()
        with self.assertRaises(PairLedgerError):
            self.activate(legacy,self.explicit_authorization(legacy))
        self.assertEqual(self.rows(),before)
        # An authorization may precede the proposal; activation itself may not
        # travel backward before the complete reviewed snapshot was prepared.
        with self.assertRaises(PairLedgerError):
            activate_restart(proposal,authorization,project_root=self.root,clock=lambda:116.)
        self.assertEqual(self.rows(),before)

    def test_explicit_policy_has_hard_caps_and_default_policy_is_unchanged(self):
        before=self.rows()
        for overrides in ({'max_steps':501},{'max_duration_s':3601},{'max_steps':True},
                {'max_duration_s':float('inf')},{'budget_mode':'automatic'},
                {'budget_mode':'preserve_parent_ceiling'}):
            with self.subTest(overrides=overrides),self.assertRaises((PairLedgerError,ValueError)):
                self.explicit_proposal(**overrides)
        self.assertEqual(self.rows(),before)
        missing=self.root/'does-not-exist.sqlite'
        self.assertFalse(activated_execution_budget(missing,'new',max_steps=500,max_duration_s=3600))
        self.assertFalse(missing.exists())

    def test_actual_host_accepts_only_the_exact_activated_expanded_contract(self):
        from robot_tools import pair_host
        # Freeze the real host's complete source set in the synthetic project;
        # this is not a repair/activation of any real run or device connection.
        for source in Path(pair_host.__file__).parent.glob('*.py'):
            (self.root/'robot_tools'/source.name).write_bytes(source.read_bytes())
        proposal=self.explicit_proposal()
        self.activate(proposal,self.explicit_authorization(proposal))
        before=self.rows()
        host=pair_host.PairHost(self.root/'runs',self.contract,'new',self.contract['task'],500,3600,
            device_factory=lambda *args:self.fail('No device construction'),clock=lambda:self.now,background=False)
        self.addCleanup(host.close)
        status=host.status()['ledger']
        self.assertEqual((status['max_steps'],status['max_duration_s']),(500,3600))
        self.assertEqual(status['execution_lineage']['cumulative_step_ceiling'],502)
        self.assertEqual(self.rows()['pair_runs'][0],before['pair_runs'][0])
        for run,steps,duration in (('random',500,3600),('new',499,3600),('new',500,3599)):
            with self.subTest(run=run,steps=steps,duration=duration),self.assertRaises(ValueError):
                pair_host.PairHost(self.root/'runs',self.contract,run,self.contract['task'],steps,duration,
                    device_factory=lambda *args:self.fail('No device construction'),clock=lambda:self.now,background=False)


if __name__=='__main__':
    unittest.main()
