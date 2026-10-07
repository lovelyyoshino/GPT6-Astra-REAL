"""Explicit healthy new-round administration; temporary SQLite, no sockets."""
import copy
import hashlib
import json
import math
import sqlite3
import unittest
from unittest.mock import patch

from robot_tools import pair_round
from robot_tools.pair_ledger import PairLedger, PairLedgerError, _hold_frames, activated_execution_budget, platform_fault
import test_pair_continuation as fixtures


class PairRoundTests(unittest.TestCase):
    def setUp(self):
        f=fixtures.ZeroTXContinuationTests('runTest');f.setUp();self.addCleanup(f.doCleanups)
        self.f=f;self.root,self.path=f.root,f.path
        self.now=260.;activation=f.activate()
        self.old=PairLedger(self.path,'new',activation['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        self.old.claim('healthy-owner')
        rows=f.rows();old={e['event_id']:e for e in rows['pair_events']}
        self.totals={s:{'attempted_frames':0,'sent_frames':0,'blocked_frames':0} for s in ('left','right')}
        p=json.loads(old['query-second']['payload_json']);r=json.loads(old['query-second']['receipt_json'])
        for s in ('left','right'):p['bindings'][s]['connection_id']='healthy-'+s
        fixtures.ZeroTXContinuationTests.shift(r,100.);r.update(pair_owner='healthy-owner',event_id='query-healthy')
        self.now=280.;self.old.begin('healthy-owner','query-healthy',p);self.now=292.
        self.totals={s:{'attempted_frames':6,'sent_frames':6,'blocked_frames':0} for s in ('left','right')}
        r['session_transmission_counts']=copy.deepcopy(self.totals);self.old.finish('healthy-owner','query-healthy',r)
        self.raw=[100,0,0,200,300,0]
        for i,s in enumerate(('left','right')):
            p=json.loads(old['init-second-'+s]['payload_json']);r=json.loads(old['init-second-'+s]['receipt_json'])
            self.now=300.+i*5;fixtures.ZeroTXContinuationTests.shift(r,self.now-old['init-second-'+s]['began_at'])
            r['initialization_plan']['identity'].update(owner='healthy-owner',epoch='healthy-owner',worker_id='init-healthy-'+s,connection_id='healthy-'+s)
            self.old.begin('healthy-owner','init-healthy-'+s,p);self.now+=1
            for k in ('attempted_frames','sent_frames'):self.totals[s][k]+=4
            r['session_transmission_counts']=copy.deepcopy(self.totals);self.old.finish('healthy-owner','init-healthy-'+s,r)
        for i,kind in enumerate(('joint','joint','gripper','gripper','joint','joint')):
            self.now=310.+i*5;side='left' if i%2==0 else 'right';event='successful-'+str(i)
            count=4 if kind=='joint' else 1
            p={'kind':kind,'arm':side,'operation':'approach','target':[math.radians(v/1000) for v in self.raw] if kind=='joint' else .045}
            self.old.begin('healthy-owner',event,p)
            counts={s:{'attempted_frames':count if s==side else 0,'sent_frames':count if s==side else 0,'blocked_frames':0} for s in self.totals}
            for k in ('attempted_frames','sent_frames'):self.totals[side][k]+=count
            r={'ok':True,'guard_violations':[],'hardware_commands_sent':count,'enable_commands_sent':0,
               'stop_commands_sent':0,'transmission_counts':counts,'session_transmission_counts':copy.deepcopy(self.totals),
               'arrival_confirmed':True,'target_calls_sent':1,'feedback_all_after_send':True}
            if kind=='joint':r['original_event']={'event_id':event,'send_state':'all_frames_returned','target_raw':self.raw,
                'frame_receipts':[{'frame':frame,'outcome':'returned','returned_at':self.now+.1+j*.01} for j,frame in enumerate(_hold_frames(self.raw))]}
            self.now+=1;self.old.finish('healthy-owner',event,r)
        self.now=345.;self.old.release('healthy-owner');self.assertEqual(self.old.peek_status()['steps'],17)
        self.close=self.root/'healthy-close.json'
        self.close.write_text(json.dumps({'status':'closed','fault_latched':False,'cleanup':{
            'requires_fault_latch':False,'unresolved_gripper_probe':None,'grasp_states':{'left':None,'right':None},
            'guard_violations':[],'session_transmission_counts':self.totals,
            'arms':{s:{'status':'disconnected'} for s in self.totals}}},indent=2))
        self.started=4000.;self.now=4010.
        (self.root/'robot_tools/pair_round.py').write_text('# synthetic new management code\n')
        (self.root/'robot_tools/pair_host.py').write_text("sources=('pair_host.py','pair_ledger.py','pair_restart.py','pair_continuation.py','pair_joint_adapter.py','pair_round.py')\n")
        for target in ('socket.socket','robot_tools.pair_round._live_control_processes'):
            q=patch(target,**({'side_effect':AssertionError('No socket')} if target=='socket.socket' else {'return_value':[]}))
            q.start();self.addCleanup(q.stop)

    def rows(self):return self.f.rows()

    def prepare(self,**kwargs):
        args=dict(close_log=self.close,new_run_id='round-2',started_at=self.started,max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        args.update(kwargs)
        return pair_round.prepare_round(self.path,'new',**args)

    def authorize(self,p):
        return {'source':'user_message','message_id':'new-round-request','statement':'Begin another 60 minute round, at most 500 commands',
            'received_at':self.started,'decision':'authorize_explicit_new_round','proposal_sha256':p['proposal_sha256'],
            'new_budget':copy.deepcopy(p['new_budget'])}

    def activate(self,p=None,a=None,clock=None):
        p=p or self.prepare();return pair_round.activate_round(p,a or self.authorize(p),project_root=self.root,clock=clock or (lambda:self.now))

    def change_receipt(self,fn):
        with sqlite3.connect(self.path) as db:
            raw=db.execute("SELECT receipt_json FROM pair_events WHERE event_id='successful-5'").fetchone()[0]
            r=json.loads(raw);fn(r)
            db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='successful-5'",(json.dumps(r),))

    def test_preserves_old_history_fixed_start_budget_and_blocks_old_instances(self):
        before=self.rows();p=self.prepare();self.assertEqual(before,self.rows());result=self.activate(p)
        after=self.rows()
        for t,rows in before.items():
            self.assertEqual(rows,[r for r in after[t] if t!='pair_runs' or r['run_id']!='round-2'],t)
        new=PairLedger(self.path,'round-2',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        self.assertTrue(activated_execution_budget(self.path,'round-2',max_steps=500,max_duration_s=3600))
        self.assertFalse(activated_execution_budget(self.path,'new',max_steps=500,max_duration_s=3600))
        for owner in p['snapshot']['retired_owners']:
            with self.assertRaises(PairLedgerError):new.claim(owner)
        status=new.claim('new-round-owner')
        self.assertEqual((status['steps'],status['remaining_steps'],status['started_at'],status['deadline_s']),(0,500,4000.,7600.))
        self.assertEqual(status['remaining_s'],3590.)
        self.assertEqual(status['execution_lineage']['cumulative_steps'],19)
        self.assertEqual(status['execution_lineage']['cumulative_step_ceiling'],519)
        current=self.rows()
        for call in (lambda:self.old.claim('healthy-owner'),lambda:self.old.begin('healthy-owner','old-replay',{}),
                     lambda:self.old.finish('healthy-owner','successful-5',{}),lambda:self.old.fault('healthy-owner','late'),
                     lambda:PairLedger(self.path,'random',result['new_contract'],clock=lambda:self.now)):
            with self.assertRaises(PairLedgerError):call()
            self.assertEqual(current,self.rows())
        self.assertIsNone(platform_fault(self.path,run_id='round-2'));self.assertIsNotNone(platform_fault(self.path))
        self.assertFalse(result['cache_or_limits_transferred']);self.assertEqual(result['hardware_commands_sent'],0)

    def test_same_authorization_cannot_move_start_or_double_activate(self):
        p=self.prepare();a=self.authorize(p)
        for mutate in (lambda x:x.update(received_at=3999.),lambda x:x.update(decision='authorize_explicit_budget_request'),
                       lambda x:x['new_budget'].update(max_steps=499),lambda x:x['new_budget'].update(started_at=4010.)):
            changed=copy.deepcopy(a);mutate(changed);before=self.rows()
            with self.assertRaises(PairLedgerError):self.activate(p,changed)
            self.assertEqual(before,self.rows())
        self.activate(p,a);before=self.rows()
        with self.assertRaises(PairLedgerError):self.activate(p,a)
        with self.assertRaises(PairLedgerError):self.prepare(new_run_id='another')
        self.assertEqual(before,self.rows())

    def test_invalid_window_and_caps_refuse_read_only(self):
        before=self.rows()
        for kw in ({'max_steps':501},{'max_steps':True},{'max_duration_s':3601},{'started_at':3700.},
                   {'started_at':4011.},{'started_at':float('nan')},{'new_run_id':'new'}, {'clock':lambda:7600.}):
            with self.subTest(kw=kw),self.assertRaises((PairLedgerError,ValueError)):self.prepare(**kw)
            self.assertEqual(before,self.rows())

    def test_unknown_or_partial_receipts_refuse(self):
        for mutate in (lambda r:r['transmission_counts']['left'].update(attempted_frames=1),
                       lambda r:r['transmission_counts']['right'].update(blocked_frames=1),
                       lambda r:r['transmission_counts']['left'].update(blocked_frames=False),
                       lambda r:r['original_event']['frame_receipts'][1].update(outcome='unknown'),
                       lambda r:r.update(arrival_confirmed=False),lambda r:r.update(guard_violations=['new error'])):
            with self.subTest(mutate=mutate):
                before=self.rows();self.change_receipt(mutate)
                with self.assertRaises(PairLedgerError):self.prepare()
                with sqlite3.connect(self.path) as db:
                    old=next(e for e in before['pair_events'] if e['event_id']=='successful-5')
                    db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='successful-5'",(old['receipt_json'],))

    def test_pending_active_fault_and_close_mismatch_refuse(self):
        for assignment in ("owner='someone',active_run_id='new'","fault_id=1"):
            with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_predispatch_continuations SET '+assignment)
            with self.assertRaises(PairLedgerError):self.prepare()
            with sqlite3.connect(self.path) as db:db.execute('UPDATE pair_predispatch_continuations SET owner=NULL,active_run_id=NULL,fault_id=NULL')
        data=json.loads(self.close.read_text());data['cleanup']['session_transmission_counts']['left']['sent_frames']-=1
        self.close.write_text(json.dumps(data))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_history_or_code_change_after_proposal_and_live_process_refuse(self):
        p=self.prepare();code=self.root/'robot_tools/pair_round.py';text=code.read_text();code.write_text(text+'# changed\n')
        with self.assertRaises(PairLedgerError):self.activate(p)
        code.write_text(text)
        with patch('robot_tools.pair_round._live_control_processes',return_value=['live']):
            with self.assertRaises(PairLedgerError):self.activate(p)
        with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_faults SET reason=reason||'changed' WHERE id=1")
        with self.assertRaises(PairLedgerError):self.activate(p)

    def test_clock_after_io_is_recorded_and_expiry_rolls_back_all_writes(self):
        p=self.prepare();before=self.rows()
        for times in ((4009.,),(4010.,7600.),(4010.,4011.,7600.),(4010.,4009.)):
            it=iter(times)
            with self.subTest(times=times),self.assertRaises(PairLedgerError):self.activate(p,clock=lambda:next(it))
            self.assertEqual(before,self.rows())
        it=iter((4010.,4011.,4100.));result=self.activate(p,clock=lambda:next(it))
        self.assertEqual(result['activated_at'],4100.)
        self.assertEqual(self.rows()['pair_rounds'][0]['last_time'],4100.)
        earlier=PairLedger(self.path,'round-2',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:4099.)
        self.assertTrue(earlier.peek_status()['fault_latched'])

    def test_explicit_after_repair_policy_uses_the_frozen_later_start_only(self):
        p=self.prepare(budget_start_policy='after_repair_before_online_execution')
        a=self.authorize(p);a.update(received_at=3900.,budget_start_policy='after_repair_before_online_execution')
        for changed in ({k:v for k,v in a.items() if k!='budget_start_policy'},
                        {**a,'budget_start_policy':'include_repair_time'}, {**a,'received_at':4001.}):
            with self.assertRaises(PairLedgerError):self.activate(p,changed)
        self.now=4020.;result=self.activate(p,a)
        new=PairLedger(self.path,'round-2',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        state=new.peek_status()
        self.assertEqual((state['started_at'],state['deadline_s'],state['remaining_s']),(4000.,7600.,3580.))
        self.assertTrue(activated_execution_budget(self.path,'round-2',max_steps=500,max_duration_s=3600))

    def test_raw_query_counters_and_event_window_cannot_be_relabelled(self):
        with sqlite3.connect(self.path) as db:
            original=db.execute("SELECT receipt_json FROM pair_events WHERE event_id='query-healthy'").fetchone()[0]
        for change in ({'mode_commands_sent':1},{'target_commands_sent':1},{'joint_limit_queries_attempted':13},
                       {'began_at':279.}):
            r=json.loads(original);r.update(change)
            with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='query-healthy'",(json.dumps(r),))
            with self.subTest(change=change),self.assertRaises((PairLedgerError,ValueError)):self.prepare()
        with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='query-healthy'",(original,))

    def test_missing_old_event_cannot_be_hidden_by_current_clean_suffix(self):
        with sqlite3.connect(self.path) as db:db.execute("DELETE FROM pair_events WHERE run_id='new' AND event_id='query'")
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_later_clean_round_can_use_identical_reviewed_code(self):
        result=self.activate();new=PairLedger(self.path,'round-2',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        new.claim('round-two-owner')
        original=next(e for e in self.rows()['pair_events'] if e['event_id']=='query-healthy')
        p=json.loads(original['payload_json']);r=json.loads(original['receipt_json'])
        fixtures.ZeroTXContinuationTests.shift(r,self.now+1-r['began_at'])
        r.update(event_id='limits-round-two',pair_owner='round-two-owner')
        counts={s:{'attempted_frames':6,'sent_frames':6,'blocked_frames':0} for s in ('left','right')}
        r['session_transmission_counts']=copy.deepcopy(counts)
        new.begin('round-two-owner','limits-round-two',p);self.now+=20;new.finish('round-two-owner','limits-round-two',r)
        new.release('round-two-owner')
        close=json.loads(self.close.read_text());close['cleanup']['session_transmission_counts']=counts;self.close.write_text(json.dumps(close))
        self.started=8000.;self.now=8010.
        p=pair_round.prepare_round(self.path,'round-2',close_log=self.close,new_run_id='round-3',started_at=self.started,clock=lambda:self.now)
        self.assertEqual(p['reviewed_contract'],result['new_contract'])
        before=self.rows();next_round=self.activate(p,self.authorize(p));after=self.rows()
        for name,rows in before.items():
            self.assertEqual(rows,after[name][:len(rows)],name)
        self.assertEqual(next_round['proposal']['new_run_id'],'round-3')
        self.assertTrue(activated_execution_budget(self.path,'round-3',max_steps=500,max_duration_s=3600))


if __name__=='__main__':unittest.main()
