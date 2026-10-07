"""Administrative same-budget continuation; synthetic SQLite and files only."""
import copy
import json
import math
from pathlib import Path
import sqlite3
import unittest
from unittest.mock import patch

from robot_tools.pair_ledger import PairLedger, PairLedgerError, _hold_frames, activated_execution_budget, platform_fault
from robot_tools.pair_continuation import prepare_continuation, activate_continuation
import test_pair_restart as restart_fixtures


class ContinuationTests(unittest.TestCase):
    def setUp(self):
        fixture = restart_fixtures.RestartTests('runTest'); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        self.f = fixture; self.root,self.path = fixture.root,fixture.path
        fixture.contract['cameras'].update(left_wrist='b',right_wrist='c')
        (self.root/'configs/robot.json').write_text(json.dumps(fixture.contract))
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_runs SET contract_json=? WHERE run_id=?',
                       (json.dumps(fixture.contract,sort_keys=True,separators=(',',':')),'old'))
        proposal=fixture.explicit_proposal()
        activation=fixture.activate(proposal,fixture.explicit_authorization(proposal))
        self.now=121.
        self.ledger=PairLedger(self.path,'new',activation['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        self.ledger.claim('retired')
        oldquery=fixture.rows()['pair_events'][0]
        payload=json.loads(oldquery['payload_json']); receipt=json.loads(oldquery['receipt_json'])
        def move_times(value):
            if isinstance(value,dict):
                for k,v in list(value.items()):
                    if k in ('began_at','ended_at','request_started_unix_s','finished_unix_s','timestamp','received_unix_s','sent_at','returned_at'):
                        value[k]=v+32.
                    else:move_times(v)
            elif isinstance(value,list):
                for v in value:move_times(v)
        move_times(receipt)
        receipt.update(event_id='query',pair_owner='retired')
        self.ledger.begin('retired','query',payload);self.now=127.;self.ledger.finish('retired','query',receipt)
        self.total={s:dict(attempted_frames=6,sent_frames=6,blocked_frames=0) for s in ('left','right')}
        raw=[100,0,0,200,300,0]
        for index,side in enumerate(('left','right')):
            self.now=128.+index*4
            ident=dict(run_id='new',owner='retired',epoch='retired',worker_id='init-'+side,arm=side,
                       connection_id='old-'+side,model='piper_x',firmware_profile='default')
            self.ledger.begin('retired','init-'+side,{'kind':'initialization','expected_target_raw':raw})
            counts={s:dict(attempted_frames=4 if s==side else 0,sent_frames=4 if s==side else 0,blocked_frames=0) for s in self.total}
            self.total[side]['attempted_frames']+=4;self.total[side]['sent_frames']+=4
            self.now+=1
            self.ledger.finish('retired','init-'+side,{'ok':True,'status':'joint_target_initialized','arm':side,
                'operation':'initialize_joint_target','cache_established':True,'errors':[],'guard_violations':[],
                'initialization_plan':{'identity':ident,'target_raw':raw},
                'frame_receipts':[{'frame':f,'outcome':'returned','returned_at':self.now-.5+i*.01} for i,f in enumerate(_hold_frames(raw))],
                'hardware_commands_sent':4,'gripper_commands_sent':0,'enable_commands_sent':0,'stop_commands_sent':0,'retries':0,
                'transmission_counts':counts,'session_transmission_counts':copy.deepcopy(self.total)})
        self.now=140.
        ident.update(arm='left',worker_id='failed',connection_id='old-left')
        target=[math.radians(v/1000) for v in raw]
        self.ledger.begin('retired','failed',{'kind':'joint','arm':'left','operation':'approach','target':target})
        geometry={'schema':'piper_rgb_supervised_joint_path_v1','evidence':{'operation':'approach'}}
        original={'schema':'piper_rgb_supervised_joint_send_v1','event_id':'failed','identity':ident,
            'operation':'approach','spatial_admission_mode':'rgb_supervised','hold_supported':False,
            'hold_policy':'latch_only','fault':None,'send_state':'all_frames_returned','target_raw':raw,
            'plan_sha256':'a'*64,'rgb_admission':geometry,
            'frame_receipts':[{'frame':f,'outcome':'returned','returned_at':140.1+i*.01} for i,f in enumerate(_hold_frames(raw))]}
        self.total['left']['attempted_frames']+=4;self.total['left']['sent_frames']+=4
        self.device={'ok':False,'errors':[{'type':'RuntimeError','detail':'visual_rgb_expired: original joint RGB deadline reached'}],
            'original_event':original,'joint_path_plan':{'identity':ident,'geometry':geometry,'loaded_context':None,'plan_sha256':'a'*64},
            'guard_violations':[],'hold_receipt':None,'automatic_retry':False,'enable_commands_sent':0,'stop_commands_sent':0,
            'retries':0,'passive_arm_commands_sent':0,'hardware_commands_sent':4,
            'transmission_counts':{'left':dict(attempted_frames=4,sent_frames=4,blocked_frames=0),
                                   'right':dict(attempted_frames=0,sent_frames=0,blocked_frames=0)},
            'session_transmission_counts':copy.deepcopy(self.total)}
        self.now=141.
        self.ledger.fault('retired','Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch')
        self.ledger.finish('retired','failed',{'ok':False,'device_receipt':self.device},success=False)
        close={'status':'closed','fault_latched':True,'cleanup':{'unresolved_gripper_probe':None,'grasp_states':{'left':None,'right':None},
            'requires_fault_latch':False,'guard_violations':[],'session_transmission_counts':self.total,
            'arms':{s:{'status':'disconnected'} for s in self.total}}}
        self.close=self.root/'current-close.jsonl';self.close.write_text(json.dumps({'result':{'content':[{'type':'text','text':json.dumps(close)}]}})+'\n')
        self.passive={}
        for side in self.total:
            self.passive[side]=self.root/(side+'.json')
            self.passive[side].write_text(json.dumps(self.passive_record(side)))
        self.rgb=self.root/'observation.json'; cameras={}
        for view,serial in [('front','a'),('left_hand','b'),('right_hand','c')]:
            image=self.root/(view+'.png');image.write_bytes(b'synthetic PNG fixture')
            cameras[view]={'serial':serial,'host_received_at':154.,'frame_number':10,'depth_enabled':False,'rgb_path':str(image)}
        self.rgb.write_text(json.dumps({'cameras':cameras}))
        self.now=160.
        (self.root/'robot_tools/pair_host.py').write_text("sources = ('pair_host.py','pair_ledger.py','pair_restart.py','pair_continuation.py')\n")
        (self.root/'robot_tools/pair_continuation.py').write_text('# synthetic repaired continuation\n')
        self.proc=patch('robot_tools.pair_continuation._live_control_processes',return_value=[])
        self.proc.start();self.addCleanup(self.proc.stop)

    def passive_record(self, side):
        trace=[];joints={'joint_'+str(i):0 for i in range(1,7)}
        pose={k:0 for k in ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')}
        for i in range(65):
            at=150+i*.05
            trace.append({'received_at_s':at,'joints_raw':joints,'end_pose_raw':pose,
                          'field_received_at_s':{k:at for k in (*joints,*pose)}})
        feedback={}
        for name in ['PiperMsgGripperFeedBack']+['PiperMsgLowSpdFeed_'+str(i) for i in range(1,7)]:
            keys={'voltage_too_low','motor_overheating','driver_overcurrent','driver_overheating','driver_error_status','driver_enable_status'}
            keys|={'sensor_status','homing_status'} if name=='PiperMsgGripperFeedBack' else {'collision_status','stall_status'}
            feedback[name]={'received_at_s':153.2,'fields':{'foc_status':{k:k=='driver_enable_status' for k in keys},'grippers_angle':10000}}
        return {'mode':'passive_receive_only','channel':self.f.contract['arms'][side]['channel'],
            'frames_sent_by_this_script':0,'malformed_frames':0,'complete_feedback_received':True,'missing_feedback_types':[],
            'started_at_s':150.,'finished_at_s':153.2,'pose_trace':trace,'feedback':feedback,
            'raw_frame_latest':{'0x2A1':{'payload_hex':'0100010000000000','received_at_s':153.2}}}

    def prepare(self):
        return prepare_continuation(self.path,'new',close_log=self.close,passive_paths=self.passive,
            rgb_observation=self.rgb,visual_observation='Synthetic image interpretation: both jaws empty and no object contact',clock=lambda:self.now)

    def activate(self, proposal=None, clock=None):
        return activate_continuation(proposal or self.prepare(),project_root=self.root,clock=clock or (lambda:self.now))

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            return {name:[dict(r) for r in db.execute('SELECT * FROM '+name)] for name in
                    ('pair_scope','pair_execution_epochs','pair_runs','pair_events','pair_faults')}

    def test_same_run_budget_and_old_records_unchanged_new_owner_and_contract_only(self):
        before=self.rows();proposal=self.prepare();self.assertEqual(self.rows(),before)
        result=self.activate(proposal);self.assertEqual(self.rows(),before)
        self.assertEqual(result['hardware_commands_sent'],0)
        self.assertFalse(result['new_budget_allocated'])
        self.assertTrue(activated_execution_budget(self.path,'new',max_steps=500,max_duration_s=3600))
        with self.assertRaises(ValueError):
            PairLedger(self.path,'new',json.loads(before['pair_runs'][-1]['contract_json']),max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):self.ledger.fault('retired','late stale instance')
        self.assertTrue(self.ledger.peek_status()['fault_latched'])
        new=PairLedger(self.path,'new',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):new.claim('retired')
        status=new.claim('fresh-owner')
        self.assertEqual((status['steps'],status['remaining_steps'],status['deadline_s']),(4,496,3720.))
        self.assertEqual(status['execution_lineage']['cumulative_step_ceiling'],502)
        self.assertEqual(status['contract'],result['new_contract'])
        replay=new.begin('fresh-owner','failed',json.loads(before['pair_events'][-1]['payload_json']))
        self.assertTrue(replay['replayed']);self.assertFalse(replay['receipt']['ok'])
        self.assertEqual(new.status()['steps'],4)
        new.begin('fresh-owner','new-event',{'kind':'query'});new.finish('fresh-owner','new-event',{'ok':True})
        self.assertEqual(new.status()['steps'],5)
        self.assertEqual(new.status()['deadline_s'],3720.)
        self.assertIsNone(platform_fault(self.path,run_id='new'))
        with self.assertRaises(PairLedgerError):self.activate(proposal)
        self.assertEqual(before['pair_execution_epochs'],self.rows()['pair_execution_epochs'])
        self.assertEqual(before['pair_faults'],self.rows()['pair_faults'])

    def corrupt_receipt(self, event, change):
        with sqlite3.connect(self.path) as db:
            text=db.execute('SELECT receipt_json FROM pair_events WHERE run_id=? AND event_id=?',('new',event)).fetchone()[0]
            data=json.loads(text);change(data)
            db.execute('UPDATE pair_events SET receipt_json=? WHERE run_id=? AND event_id=?',(json.dumps(data),'new',event))

    def test_failed_partial_frame_or_extra_error_refuses(self):
        self.corrupt_receipt('failed',lambda r:r['device_receipt']['original_event']['frame_receipts'][2].update(outcome='unknown'))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_historical_query_mode_frame_cannot_hide_in_success_counters(self):
        self.corrupt_receipt('query',lambda r:r.update(mode_commands_sent=1))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_historical_initialization_unknown_frame_cannot_hide_in_success(self):
        self.corrupt_receipt('init-left',lambda r:r['frame_receipts'][1].update(outcome='unknown'))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_nonzero_passive_tx_or_body_movement_refuses(self):
        original=self.passive['left'].read_text()
        for mutation in (lambda d:d.update(frames_sent_by_this_script=1),
                lambda d:d['pose_trace'][30]['end_pose_raw'].update(X_axis=400,Y_axis=400)):
            value=json.loads(original);mutation(value);self.passive['left'].write_text(json.dumps(value))
            with self.assertRaises(PairLedgerError):self.prepare()

    def test_profile_rgb_identity_or_pre_failure_image_refuses(self):
        original=self.rgb.read_text()
        for changes in ({'serial':'another-camera'},{'host_received_at':140.}):
            data=json.loads(original);data['cameras']['front'].update(changes);self.rgb.write_text(json.dumps(data))
            with self.assertRaises(PairLedgerError):self.prepare()

    def test_deadline_clock_source_change_and_second_continuation_refuse_without_old_writes(self):
        p=self.prepare();before=self.rows()
        for times in ((159.,),(160.,161.,160.),(160.,161.,3720.),(160.,161.,162.,161.)):
            ticks=iter(times)
            with self.subTest(times=times),self.assertRaises(PairLedgerError):self.activate(p,lambda:next(ticks))
            self.assertEqual(self.rows(),before)
        (self.root/'robot_tools/pair_continuation.py').write_text('# changed after proposal')
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(self.rows(),before)

    def test_new_fault_remains_sticky_and_cannot_open_another_scope(self):
        result=self.activate()
        new=PairLedger(self.path,'new',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        new.claim('fresh-owner');new.fault('fresh-owner','different fault')
        with self.assertRaises(PairLedgerError):new.begin('fresh-owner','never',{})
        with self.assertRaises(PairLedgerError):self.prepare()


class ZeroTXContinuationTests(unittest.TestCase):
    def setUp(self):
        from robot_tools.pair_continuation import _sha
        b=ContinuationTests('runTest');b.setUp();self.addCleanup(b.doCleanups);self.base=b
        self.root,self.path=b.root,b.path
        self.adapter=self.root/'robot_tools/pair_joint_adapter.py';self.adapter.write_text('# old synthetic adapter\n')
        host=self.root/'robot_tools/pair_host.py'
        host.write_text("sources = ('pair_host.py','pair_ledger.py','pair_restart.py','pair_continuation.py','pair_joint_adapter.py')\n")
        first=b.activate();self.first_ledger=b.ledger;self.now=180.
        self.ledger=PairLedger(self.path,'new',first['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        self.ledger.claim('second-owner')
        oldevents={e['event_id']:e for e in b.rows()['pair_events'] if e['run_id']=='new'}
        self.total={s:dict(attempted_frames=6,sent_frames=6,blocked_frames=0) for s in ('left','right')}
        self.query_payload=json.loads(oldevents['query']['payload_json'])
        for side,row in self.query_payload['bindings'].items():row['connection_id']='second-'+side
        query=json.loads(oldevents['query']['receipt_json']);self.shift(query,64.)
        query.update(event_id='query-second',pair_owner='second-owner')
        self.ledger.begin('second-owner','query-second',self.query_payload);self.now=191.
        self.ledger.finish('second-owner','query-second',query)
        raw=[100,0,0,200,300,0]
        for i,side in enumerate(('left','right')):
            name='init-second-'+side;self.now=195.+i*5
            payload=json.loads(oldevents['init-'+side]['payload_json'])
            receipt=json.loads(oldevents['init-'+side]['receipt_json'])
            self.shift(receipt,self.now-oldevents['init-'+side]['began_at'])
            receipt['initialization_plan']['identity'].update(owner='second-owner',epoch='second-owner',worker_id=name,connection_id='second-'+side)
            self.total[side]['attempted_frames']+=4;self.total[side]['sent_frames']+=4
            receipt['session_transmission_counts']=copy.deepcopy(self.total)
            self.ledger.begin('second-owner',name,payload);self.now+=1
            self.ledger.finish('second-owner',name,receipt)
        self.now=205.;self.target=[math.radians(v/1000) for v in raw]
        self.payload={'kind':'joint','arm':'right','operation':'approach','target':self.target}
        self.ledger.begin('second-owner','failed-zero',self.payload)
        ident=dict(run_id='new',owner='second-owner',epoch='second-owner',worker_id='failed-zero',arm='right',connection_id='second-right',model='piper_x',firmware_profile='default')
        plan={'identity':ident,'loaded_context':None,'loaded_observation_only':False,'recovery_mode':None,
            'spatial_admission_mode':'rgb_supervised','geometry':{'schema':'piper_rgb_supervised_joint_path_v1','evidence':{'operation':'approach'}},
            'hold_supported':False,'hold_policy':'latch_only','requested_target_joints_rad':self.target,'target_raw':raw,'frames':_hold_frames(raw)}
        plan['plan_sha256']=_sha(plan)
        error='Joint feedback exceeded 50 ms including validation'
        zero={s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in self.total}
        self.device={'ok':False,'status':'pair_device_fault','kind':'joint','errors':[{'type':'RuntimeError','detail':error}],
            'original_event':None,'original_action_report':None,'hold_receipt':None,'automatic_retry':False,
            'guard_violations':[],'enable_commands_sent':0,'stop_commands_sent':0,'retries':0,
            'passive_arm_commands_sent':0,'hardware_commands_sent':0,'target_commands_sent':0,'target_calls_sent':0,
            'motion_gate_unlocked':False,'joint_limits_changed':False,'spatial_admission_mode':'rgb_supervised',
            'hold_supported':False,'hold_policy':'latch_only','explicit_cancel_hold_bridge_bound':False,
            'joint_path_plan':plan,'target_joints_rad':self.target,'transmission_counts':zero,'session_transmission_counts':copy.deepcopy(self.total),
            'tracking_observation':{'first_failure':{'type':'RuntimeError','detail':error,'sample_role':'rejected_observation',
                'sample':{'identity':ident,'captured_at':205.5}}}}
        self.now=206.;self.ledger.fault('second-owner','Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch')
        self.ledger.finish('second-owner','failed-zero',{'ok':False,'event_id':'failed-zero','automatic_retry':False,'error':'Stable feedback alone is not an arrived single dispatch','device_receipt':self.device},success=False)
        close={'status':'closed','fault_latched':True,'cleanup':{'unresolved_gripper_probe':None,'grasp_states':{'left':None,'right':None},
            'requires_fault_latch':False,'guard_violations':[],'session_transmission_counts':self.total,
            'arms':{s:{'status':'disconnected'} for s in self.total}}}
        self.close=self.root/'second-close.jsonl';self.close.write_text(json.dumps({'result':{'content':[{'type':'text','text':json.dumps(close)}]}})+'\n')
        self.passive=b.passive;self.rgb=b.rgb
        for p in self.passive.values():
            d=json.loads(p.read_text());self.shift(d,100.);p.write_text(json.dumps(d))
        d=json.loads(self.rgb.read_text());self.shift(d,100.);self.rgb.write_text(json.dumps(d))
        self.now=260.;self.adapter.write_text('# synthetic repaired scheduling\n')

    @staticmethod
    def shift(value, delta):
        if isinstance(value,dict):
            for k,v in value.items():
                if type(v) in (int,float) and k in ('began_at','ended_at','request_started_unix_s','finished_unix_s','timestamp',
                    'received_unix_s','sent_at','returned_at','started_at_s','finished_at_s','received_at_s','host_received_at'):
                    value[k]=v+delta
                elif k=='field_received_at_s':value[k]={k:t+delta for k,t in v.items()}
                else:ZeroTXContinuationTests.shift(v,delta)
        elif isinstance(value,list):
            for v in value:ZeroTXContinuationTests.shift(v,delta)

    def prepare(self):
        from robot_tools.pair_continuation import prepare_zero_tx_continuation
        return prepare_zero_tx_continuation(self.path,'new',close_log=self.close,passive_paths=self.passive,rgb_observation=self.rgb,
            visual_observation='Synthetic current RGB: both jaws empty, no contact, plug still in source socket.',clock=lambda:self.now)

    def activate(self, proposal=None, clock=None):
        from robot_tools.pair_continuation import activate_zero_tx_continuation
        return activate_zero_tx_continuation(proposal or self.prepare(),project_root=self.root,clock=clock or (lambda:self.now))

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            return {r[0]:[dict(x) for x in db.execute('SELECT * FROM '+r[0])] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%' ORDER BY name")}

    def change_device(self, change):
        self.base.corrupt_receipt('failed-zero',lambda r:change(r['device_receipt']))

    def test_success_preserves_every_old_row_budget_fault_and_replay(self):
        before=self.rows();p=self.prepare();self.assertEqual(self.rows(),before)
        result=self.activate(p);after=self.rows()
        for k,v in before.items():self.assertEqual(after[k],v,k)
        self.assertEqual(result['hardware_commands_sent'],0);self.assertFalse(result['new_budget_allocated'])
        new=PairLedger(self.path,'new',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now)
        for oldowner in p['retired_owners']:
            with self.assertRaises(PairLedgerError):new.claim(oldowner)
        status=new.claim('third-owner');self.assertEqual((status['steps'],status['remaining_steps'],status['deadline_s']),(8,492,3720.))
        self.assertEqual(status['contract'],result['new_contract'])
        for old in (self.ledger,self.first_ledger):
            with self.assertRaises(PairLedgerError):old.status()
            with self.assertRaises(PairLedgerError):old.fault('second-owner','stale instance')
            self.assertTrue(old.peek_status()['fault_latched'])
        replay=new.begin('third-owner','failed-zero',self.payload);self.assertTrue(replay['replayed']);self.assertFalse(replay['receipt']['ok'])
        self.assertEqual(new.status()['steps'],8)
        new.begin('third-owner','genuinely-new',{'kind':'query'});new.finish('third-owner','genuinely-new',{'ok':True})
        self.assertEqual(new.status()['steps'],9);self.assertEqual(new.status()['deadline_s'],3720.)
        with self.assertRaises(PairLedgerError):self.prepare()
        with self.assertRaises(PairLedgerError):self.activate(p)
        with self.assertRaises(PairLedgerError):self.base.prepare()

    def test_zero_counter_bool_attempt_block_original_and_sdk_call_refused(self):
        baseline=json.dumps(self.device)
        mutations=[lambda d:d['transmission_counts']['right'].update(attempted_frames=1),
            lambda d:d['transmission_counts']['right'].update(blocked_frames=1),
            lambda d:d['transmission_counts']['right'].update(sent_frames=False),
            lambda d:d.update(target_calls_sent=1),lambda d:d.update(hardware_commands_sent=False),
            lambda d:d.update(original_event={}),lambda d:d.update(original_action_report={}),
            lambda d:d['session_transmission_counts']['right'].update(blocked_frames=False)]
        for change in mutations:
            d=json.loads(baseline);change(d);self.base.corrupt_receipt('failed-zero',lambda r:r.update(device_receipt=d))
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.prepare()

    def test_only_exact_unloaded_rgb_error_and_plan_integrity(self):
        baseline=json.dumps(self.device)
        changes=[lambda d:d['errors'].append({'type':'RuntimeError','detail':'another fault'}),
            lambda d:d['errors'][0].update(detail='Feedback stale'),
            lambda d:d['joint_path_plan'].update(loaded_context={}),
            lambda d:d['joint_path_plan'].update(spatial_admission_mode='metric'),
            lambda d:d['joint_path_plan']['identity'].update(connection_id='forged'),
            lambda d:d['tracking_observation']['first_failure'].update(sample_role='replaced_observation')]
        for change in changes:
            d=json.loads(baseline);change(d);self.base.corrupt_receipt('failed-zero',lambda r:r.update(device_receipt=d))
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.prepare()

    def test_prior_audited_history_cannot_be_rewritten(self):
        self.base.corrupt_receipt('query',lambda r:r.update(mode_commands_sent=1))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_current_preparation_raw_history_cannot_be_rewritten(self):
        self.base.corrupt_receipt('init-second-right',lambda r:r['frame_receipts'][1].update(outcome='unknown'))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_no_adapter_repair_no_proposal(self):
        self.adapter.write_text('# old synthetic adapter\n')
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_pending_and_additional_fault_refused(self):
        with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_events SET status='pending' WHERE event_id='failed-zero'")
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_proposal_cas_source_clock_and_deadline_preserve_old_rows(self):
        p=self.prepare();before=self.rows()
        for times in ((259.,),(260.,261.,260.),(260.,261.,3720.),(260.,261.,262.,261.)):
            ticks=iter(times)
            with self.subTest(times=times),self.assertRaises(PairLedgerError):self.activate(p,lambda:next(ticks))
            self.assertEqual(self.rows(),before)
        self.adapter.write_text('# changed again after proposal')
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(self.rows(),before)

    def test_old_event_payload_conflict_cannot_dispatch_and_new_run_cannot_bypass(self):
        result=self.activate();new=PairLedger(self.path,'new',result['new_contract'],max_steps=500,max_duration_s=3600,clock=lambda:self.now);new.claim('third-owner')
        with self.assertRaises(PairLedgerError):new.begin('third-owner','failed-zero',{**self.payload,'operation':'align'})
        self.assertTrue(new.peek_status()['fault_latched'])
        with self.assertRaises(PairLedgerError):PairLedger(self.path,'other',result['new_contract'],clock=lambda:self.now)


    def test_rehashed_connection_or_model_must_match_actual_query_and_initialization(self):
        from robot_tools.pair_continuation import _sha
        baseline=json.dumps(self.device)
        for key,value in [('connection_id','not-the-queried-connection'),('model','piper'),('firmware_profile','different')]:
            d=json.loads(baseline);plan=d['joint_path_plan'];plan['identity'][key]=value
            d['tracking_observation']['first_failure']['sample']['identity'][key]=value
            plan['plan_sha256']=_sha({k:v for k,v in plan.items() if k!='plan_sha256'})
            self.base.corrupt_receipt('failed-zero',lambda r:r.update(device_receipt=d))
            with self.subTest(key=key),self.assertRaises(PairLedgerError):self.prepare()
        self.base.corrupt_receipt('failed-zero',lambda r:r.update(device_receipt=json.loads(baseline)))
        self.base.corrupt_receipt('init-second-right',lambda r:r['initialization_plan']['identity'].update(connection_id='different'))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_additional_fault_is_outside_the_narrow_branch(self):
        self.ledger.fault('second-owner','another independent failure')
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_close_lifetime_mismatch_and_old_observation_refuse(self):
        original=self.close.read_text();d=json.loads(original);close=json.loads(d['result']['content'][0]['text'])
        close['cleanup']['session_transmission_counts']['right']['sent_frames']=9
        d['result']['content'][0]['text']=json.dumps(close);self.close.write_text(json.dumps(d)+'\n')
        with self.assertRaises(PairLedgerError):self.prepare()
        self.close.write_text(original)
        d=json.loads(self.rgb.read_text());d['cameras']['front']['host_received_at']=205.
        self.rgb.write_text(json.dumps(d))
        with self.assertRaises(PairLedgerError):self.prepare()


    def test_live_process_or_new_fault_prevents_activation_without_writes(self):
        p=self.prepare();before=self.rows()
        with patch('robot_tools.pair_continuation._live_control_processes',return_value=[{'pid':123,'command':'synthetic controller'}]):
            with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(self.rows(),before)
        # A new fault after the read-only proposal must invalidate its snapshot.
        self.ledger.fault('second-owner','independent fault after proposal')
        changed=self.rows()
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(self.rows(),changed)



if __name__=='__main__':unittest.main()
