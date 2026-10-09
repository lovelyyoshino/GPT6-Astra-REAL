"""New-task preparation admission against complete temporary real ledger shapes."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import pair_task_enrollment as entry
from robot_tools.pair_host import PairHost
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget, platform_fault
from robot_tools.reboot_startup import SCOPE_TABLES, CONFIRMATION
from test_pair_rgb_round import RGBExpiryRoundTests
from test_execution import healthy_arm


class TaskEnrollmentTests(unittest.TestCase):
    def setUp(self):
        fixture = RGBExpiryRoundTests('runTest'); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        self.f = fixture
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)/'bundle/projects/piperx_cloth_demo'
        self.root.parent.mkdir(parents=True)
        shutil.copytree(fixture.root,self.root)
        self.path = self.root/'runs/pair_sessions.sqlite'; self.now = 8000.
        self.close = self.root/fixture.close.name
        self.passive,self.rgb = fixture.passive,fixture.rgb
        for path in [*self.passive.values(),self.rgb]:
            data = json.loads(path.read_text()); fixture.f.f.shift(data,3940.)
            if path in self.passive.values():
                data['raw_frame_latest']['0x2A1']['payload_hex'] = '0100000000000000'
                data['feedback']['PiperMsgGripperFeedBack']['fields']['foc_status']['driver_enable_status'] = False
                data.update(frame_origin_counts={'local':0,'nonlocal':1000},command_feedback={},
                    frame_id_counts={'0x2A1':100,'0x2A2':100,'0x2A5':100})
            path.write_text(json.dumps(data))
        actual = Path(entry.__file__).parent
        for source in actual.glob('*.py'):
            (self.root/'robot_tools'/source.name).write_bytes(source.read_bytes())
        recipe = self.root.parent.parent/'tasks/plug_transfer_left.json'; recipe.parent.mkdir()
        recipe.write_bytes((actual.parents[2]/'tasks/plug_transfer_left.json').read_bytes())
        self.task = {'task_id':'plug_transfer_left','roles':{'left':'task','right':'task'},
            'site_context':{'workspace_clearance':{'source':'user','statement':'Synthetic current clearance'}}}
        self.task_file = self.root/'new-task.json'
        self.task_file.write_text(json.dumps({'schema':'piper_supervised_plug_task_v1','task':self.task,
            'recipe_path':str(recipe),'recipe_sha256':hashlib.sha256(recipe.read_bytes()).hexdigest(),
            'on_site_supervision':{'source':'user','statement':'Synthetic on-site supervisor'}}))
        self.boot = {'boot_id':'5376e3e5-15c8-46ad-b5de-1f6124b09b80','started_at':7700}
        self.startup_dir = self.root/'runs/reboot_startup_fixture'; self.startup_dir.mkdir()
        self.make_startup()
        for target, value in (('boot_identity',self.boot),('check_processes',None),('_lock_roots',[self.root])):
            p = patch.object(entry,target,return_value=value); p.start(); self.addCleanup(p.stop)

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            return {name:[dict(r) for r in db.execute('SELECT * FROM '+name)] for name, in
                    db.execute("SELECT name FROM sqlite_master WHERE name LIKE 'pair_%'").fetchall()}

    def make_startup(self):
        rows = self.rows(); scopes=[]; times=[]
        for table in SCOPE_TABLES:
            for value in rows.get(table,[]):
                times.append(value['last_time'])
                scopes.append({'table':table,'ordinal':value.get('ordinal',1),
                    'run_id':value.get('run_id',value.get('active_run_id')),'owner':value['owner'],'fault_id':value['fault_id'],
                    'sha256':hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()})
        for table,field in (('pair_events','began_at'),('pair_events','finished_at'),('pair_faults','at'),('pair_runs','started_at')):
            times += [r[field] for r in rows[table] if r[field] is not None]
        enrollment={'route':'reboot_startup','boot':self.boot,'ledgers':[{'database':str(self.path),
            'scopes':scopes,'last_task_activity_at':max(times)}], 'physical_stop_verified':None,'task_motion_authorized':False}
        request={'run_id':self.startup_dir.name,'arm':'both','operator_statement':CONFIRMATION,'enrollment':enrollment}
        self.result={'run_id':self.startup_dir.name,'record_path':str(self.startup_dir/'result.json'),
            'reboot_startup':enrollment,'ok':False,'status':'aborted_after_dispatch','operation':'startup_arms',
            'errors':[{'type':'RuntimeError','detail':entry.STARTUP_ERROR}],'guard_violations':[],
            'transmission_counts':{s:dict(attempted_frames=2,sent_frames=2,blocked_frames=0) for s in ('left','right')},
            'transmission_counts_by_kind':{s:{k:dict(attempted_frames=1,sent_frames=1) for k in ('mode','enable')}
                for s in ('left','right')},'old_task_faults_preserved':True,'task_motion_authorized':False,
            'motion_gate_unlocked':False,'grasp_verified':False,'hardware_commands_sent':4,'enable_commands_sent':2,
            'target_commands_sent':0,'gripper_target_commands_sent':0,'stop_commands_sent':0,'retries':0,
            'last_enable_feedback':{s:{'driver_enabled':[True]*6,'gripper_enabled':False} for s in ('left','right')},
            'cleanup':{'arms':{s:{'status':'disconnected','physically_stopped':None} for s in ('left','right')}}}
        self.write_result(self.result)
        (self.startup_dir/'request.json').write_text(json.dumps(request))
        counts={s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in ('left','right')}
        self.journal=[{'event':'operation_started','unix_s':7799.},
            {'event':'connected_passively','unix_s':7799.5,'transmission_counts':copy.deepcopy(counts)}]
        for side,kind in (('left','mode'),('right','mode'),('left','enable'),('right','enable')):
            at=7799.+len(self.journal)*.5
            self.journal.append({'event':kind+'_request_intent','side':side,'unix_s':at,
                'arbitration_id':0x151 if kind=='mode' else 0x471,
                'data_hex':'0100010000000000' if kind=='mode' else '0702000000000000',
                'mode_feedback':0,'speed_percent':1,'sdk_api':'enable(255)','cached_target_activation_possible':True})
            counts[side]['attempted_frames']+=1;counts[side]['sent_frames']+=1
            self.journal.append({'event':kind+'_frame_sent_unconfirmed','side':side,'unix_s':at+.5,
                'sent_at':at+.4,'sdk_cached_return':False,'transmission_counts':copy.deepcopy(counts)})
            if kind=='mode' or side=='left':
                self.journal.append({'event':'can_control_observed' if kind=='mode' else 'joints_enabled_observed',
                    'side':side,'unix_s':at+.9})
        modes={s:0 for s in ('left','right')};enabled={s:False for s in modes};feedback=[]
        def observe(at, spike=False):
            row={'event':'feedback','unix_s':at}
            for side in modes:
                state=healthy_arm(at-.01);state['arm_status'].update(ctrl_mode=modes[side],mode_feedback=0,teach_status=0)
                for item in state['drivers'].values():item['foc_status']['driver_enable_status']=enabled[side]
                state['gripper']['foc_status']['driver_enable_status']=False
                if spike and side=='right':state['joints_rad'][3]+=.005602506898901798
                row[side]=state
            feedback.append(row)
        observe(7799.75)
        for event in self.journal:
            if event['event']=='mode_frame_sent_unconfirmed':modes[event['side']]=1;observe(event['unix_s']+.1)
            if event['event']=='enable_frame_sent_unconfirmed':enabled[event['side']]=True;observe(event['unix_s']+.1,event['side']=='right')
        observe(7809.)
        self.result.update(samples=len(feedback),before={s:feedback[0][s] for s in modes},
            after={s:feedback[-1][s] for s in modes},
            drift={s:dict(joint_rad=.005602506898901798 if s=='right' else 0.,position_m=0.,gripper_m=0.) for s in modes})
        # Compute with the same raw floating-point samples, avoiding rounded summaries.
        self.result['drift']['right']['joint_rad']=max(abs(r['right']['joints_rad'][3]-feedback[0]['right']['joints_rad'][3]) for r in feedback)
        self.write_result(self.result)
        self.journal=sorted(self.journal+feedback,key=lambda r:r['unix_s'])
        self.write_journal(self.journal)
        with sqlite3.connect(self.path) as db:
            db.execute('CREATE TABLE pair_reboot_startups(boot_id TEXT,arm TEXT,run_id TEXT,status TEXT,'
                       'record_path TEXT,started_at REAL,finished_at REAL,PRIMARY KEY(boot_id,arm))')
            for side in ('left','right'):
                db.execute('INSERT INTO pair_reboot_startups VALUES(?,?,?,?,?,?,?)',(self.boot['boot_id'],side,
                    self.startup_dir.name,'failed',str(self.startup_dir/'result.json'),7799.9,7810.))

    def write_result(self,value): (self.startup_dir/'result.json').write_text(json.dumps(value))
    def write_journal(self,value): (self.startup_dir/'events.jsonl').write_text('\n'.join(json.dumps(r) for r in value)+'\n')

    def prepare(self,**kwargs):
        values=dict(close_log=self.close,task_file=self.task_file,new_run_id='new-supervised-task',started_at=7990.,
            passive_paths=self.passive,rgb_observation=self.rgb,visual_observation='Synthetic empty jaws, no contact; independent support.',
            clock=lambda:self.now)
        values.update(kwargs); return entry.prepare_task(self.path,'round-2',**values)

    def authorize(self,p):
        return dict(source='user_message',message_id='new-task-request',statement='Begin the new supervised plug task.',
            received_at=7980.,decision='authorize_explicit_new_round',proposal_sha256=p['proposal_sha256'],
            new_budget=copy.deepcopy(p['new_budget']),budget_start_policy=p['budget_start_policy'])

    def activate(self,p,authorization=None):
        return entry.activate_task(p,authorization or self.authorize(p),project_root=self.root,clock=lambda:self.now)

    def test_new_preparation_scope_preserves_all_history_and_uses_real_host_contract(self):
        before=self.rows(); p=self.prepare(); self.assertEqual(before,self.rows()); result=self.activate(p)
        after=self.rows()
        for table,rows in before.items():
            self.assertEqual(rows,[r for r in after[table] if not (table in ('pair_runs','pair_rounds')
                and r['run_id']=='new-supervised-task')],table)
        self.assertTrue(activated_execution_budget(self.path,'new-supervised-task',max_steps=1000,max_duration_s=10800))
        self.assertEqual(result['new_contract']['task'],self.task)
        self.assertFalse(result['cache_or_limits_transferred']);self.assertEqual(result['hardware_commands_sent'],0)
        self.assertIsNotNone(platform_fault(self.path));self.assertIsNone(platform_fault(self.path,run_id='new-supervised-task'))
        profile=json.loads((self.root/'configs/robot.json').read_text())
        for mode in ('ready','prepare'):
            args=dict(device_factory=lambda *a: (_ for _ in ()).throw(AssertionError('No device construction')),
                clock=lambda:self.now,background=False,connection_mode=mode)
            if mode=='ready':
                with self.assertRaisesRegex(ValueError,'preparation connection'):
                    PairHost(self.root/'runs',profile,'new-supervised-task',self.task,1000,10800,**args)
            else:
                host=PairHost(self.root/'runs',profile,'new-supervised-task',self.task,1000,10800,**args)
                self.assertIsNone(host.device)
                self.assertEqual(host.ledger.peek_status()['steps'],0)
        ledger=PairLedger(self.path,'new-supervised-task',result['new_contract'],max_steps=1000,max_duration_s=10800,
            clock=lambda:self.now)
        for owner in p['snapshot']['retired_owners']:
            with self.assertRaises(PairLedgerError):ledger.claim(owner)
        for run in ('round-2','arbitrary'):
            with self.assertRaises(PairLedgerError):PairLedger(self.path,run,result['new_contract'],clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):self.activate(p)

    def test_original_continuation_still_refuses_p_mode_and_disabled_jaws(self):
        from robot_tools.pair_continuation import _observations
        p=self.prepare()
        with self.assertRaisesRegex(PairLedgerError,'CAN/J'):
            _observations(self.passive,self.rgb,'Synthetic empty jaws.',p['reviewed_contract'],7810.,self.now)

    def test_p_j_l_and_known_jaw_flags_supported_only_with_stationary_trace(self):
        path=self.passive['right'];base=json.loads(path.read_text())
        for mode in (0,1,2):
            for enabled in (False,True):
                data=copy.deepcopy(base);data['raw_frame_latest']['0x2A1']['payload_hex']='0100%02x0000000000'%mode
                data['feedback']['PiperMsgGripperFeedBack']['fields']['foc_status']['driver_enable_status']=enabled
                path.write_text(json.dumps(data));p=self.prepare()
                self.assertEqual(p['recovery_evidence']['passive']['right']['jaw_enabled'],enabled)

    def test_real_scale_j4_jump_and_interior_pose_excursions_refuse_without_db_writes(self):
        path=self.passive['right'];base=json.loads(path.read_text());digest=hashlib.sha256(self.path.read_bytes()).hexdigest()
        for group,key,change in (('joints_raw','joint_4',351),('end_pose_raw','X_axis',501)):
            data=copy.deepcopy(base);data['pose_trace'][10][group][key]+=change;path.write_text(json.dumps(data))
            with self.subTest(key=key),self.assertRaisesRegex(PairLedgerError,'not stationary'):self.prepare()
            self.assertEqual(digest,hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_preparation_rotation_uses_so3_across_wrap_and_gimbal_coordinates(self):
        path=self.passive['right'];base=json.loads(path.read_text())
        for gimbal in (False,True):
            data=copy.deepcopy(base)
            for i,row in enumerate(data['pose_trace']):
                angle=179999 if i%2 else -179999
                row['end_pose_raw'].update(RX_axis=angle if gimbal else 0,
                    RY_axis=90000 if gimbal else 0,RZ_axis=angle)
            path.write_text(json.dumps(data));p=self.prepare()
            self.assertLess(p['recovery_evidence']['passive']['right']['rotation_diameter_rad'],.003)
        data=copy.deepcopy(base);data['pose_trace'][10]['end_pose_raw']['RX_axis']+=180
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(PairLedgerError,'whole-position/rotation'):self.prepare()

    def test_status_enable_health_and_stale_fields_refuse(self):
        path=self.passive['left'];base=json.loads(path.read_text())
        mutate=[lambda d:d['raw_frame_latest']['0x2A1'].update(payload_hex='0000000000000000'),
            lambda d:d['raw_frame_latest']['0x2A1'].update(payload_hex='0100030000000000'),
            lambda d:d['raw_frame_latest']['0x2A1'].update(payload_hex='0100000001000000'),
            lambda d:d['feedback']['PiperMsgLowSpdFeed_1']['fields']['foc_status'].update(driver_enable_status=False),
            lambda d:d['feedback']['PiperMsgGripperFeedBack']['fields']['foc_status'].update(driver_enable_status=None),
            lambda d:d['feedback']['PiperMsgLowSpdFeed_4']['fields']['foc_status'].update(stall_status=True),
            lambda d:d['pose_trace'][10]['field_received_at_s'].update(joint_4=7000.),
            lambda d:d['frame_origin_counts'].update(local=1),
            lambda d:d['frame_id_counts'].update({'0x151':1}),
            lambda d:d['command_feedback'].update({'PiperMsgMotionCtrl_2':{'ctrl_mode':1}})]
        for fn in mutate:
            data=copy.deepcopy(base);fn(data);path.write_text(json.dumps(data))
            with self.subTest(fn=fn),self.assertRaises(PairLedgerError):self.prepare()

    def test_each_startup_wire_fact_and_cumulative_return_is_required(self):
        mutations=[lambda r:r[2].update(data_hex='0101010000000000'),lambda r:r[5].update(side='left'),
            lambda r:r[8].update(arbitration_id=0x159),lambda r:r[12].update(sent_at=7000.),
            lambda r:r[6]['transmission_counts']['right'].update(sent_frames=0),lambda r:r.pop(9),
            lambda r:r.append(copy.deepcopy(r[-1]))]
        before=self.rows()
        for fn in mutations:
            rows=copy.deepcopy(self.journal);events=[r for r in rows if r['event']!='feedback'];fn(events)
            self.write_journal(sorted(events+[r for r in rows if r['event']=='feedback'],key=lambda r:r['unix_s']))
            with self.subTest(fn=fn),self.assertRaises(PairLedgerError):self.prepare()
            self.assertEqual(before,self.rows())

    def test_failed_startup_result_cannot_hide_other_errors_or_unknown_counts(self):
        mutations=[lambda r:r.update(ok=True),lambda r:r['errors'].append({'type':'RuntimeError','detail':'other'}),
            lambda r:r.update(guard_violations=['blocked frame']),lambda r:r.update(target_commands_sent=1),
            lambda r:r['transmission_counts']['right'].update(attempted_frames=3),
            lambda r:r['cleanup']['arms']['right'].update(status='failed'),
            lambda r:r['last_enable_feedback']['right']['driver_enabled'].__setitem__(3,False),
            lambda r:r['reboot_startup']['boot'].update(boot_id='other')]
        for fn in mutations:
            data=copy.deepcopy(self.result);fn(data);self.write_result(data)
            with self.subTest(fn=fn),self.assertRaises(PairLedgerError):self.prepare()

    def test_original_startup_feedback_and_drift_cannot_be_removed_or_relabelled(self):
        without=[r for r in self.journal if r['event']!='feedback'];self.write_journal(without)
        with self.assertRaisesRegex(PairLedgerError,'feedback journal'):self.prepare()
        changed=copy.deepcopy(self.journal)
        feedback=next(r for r in changed if r['event']=='feedback')
        feedback['right']['drivers']['4']['foc_status']['stall_status']=True
        self.write_journal(changed)
        with self.assertRaises(PairLedgerError):self.prepare()
        self.write_journal(self.journal);changed=copy.deepcopy(self.result);changed['drift']['right']['joint_rad']=0.
        self.write_result(changed)
        with self.assertRaisesRegex(PairLedgerError,'drift/terminal'):self.prepare()
        changed=copy.deepcopy(self.result);changed['samples']+=1;self.write_result(changed)
        warmup={'event':'feedback','unix_s':7799.6,'left':{'status':'partial'},
            'right':{'status':'partial','drivers':{'4':{'foc_status':{'stall_status':True}}}}}
        self.write_journal(sorted(self.journal+[warmup],key=lambda r:r['unix_s']))
        with self.assertRaisesRegex(PairLedgerError,'Known warmup fault'):self.prepare()

    def test_startup_claims_pending_one_sided_foreign_or_wrong_path_refuse(self):
        original=self.rows()['pair_reboot_startups']
        for sql in ("DELETE FROM pair_reboot_startups WHERE arm='right'",
                    "UPDATE pair_reboot_startups SET status='pending' WHERE arm='left'",
                    "UPDATE pair_reboot_startups SET run_id='other' WHERE arm='right'",
                    "UPDATE pair_reboot_startups SET record_path='/tmp/foreign' WHERE arm='right'"):
            with sqlite3.connect(self.path) as db:db.execute(sql)
            with self.subTest(sql=sql),self.assertRaises(PairLedgerError):self.prepare()
            with sqlite3.connect(self.path) as db:
                db.execute('DELETE FROM pair_reboot_startups')
                for r in original:db.execute('INSERT INTO pair_reboot_startups VALUES(?,?,?,?,?,?,?)',tuple(r.values()))

    def test_evidence_must_follow_startup_and_stay_fresh(self):
        for path in (self.passive['left'],self.rgb):
            raw=path.read_text();data=json.loads(raw);self.f.f.f.shift(data,-300.)
            path.write_text(json.dumps(data))
            with self.assertRaises(PairLedgerError):self.prepare()
            path.write_text(raw)
        self.now+=31.
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_exact_new_task_authorization_and_caps_are_required(self):
        p=self.prepare();before=self.rows()
        for change in (dict(received_at=7800.),dict(decision='authorize_repaired_continuation'),
                       dict(new_budget={**p['new_budget'],'max_steps':999}),dict(proposal_sha256='0'*64)):
            auth=self.authorize(p);auth.update(change)
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.activate(p,auth)
            self.assertEqual(before,self.rows())
        for change in (dict(max_steps=1001),dict(max_duration_s=10801),dict(started_at=7600.)):
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.prepare(**change)

    def test_activation_rechecks_boot_processes_code_images_and_history(self):
        p=self.prepare();before=self.rows()
        with patch.object(entry,'boot_identity',return_value={**self.boot,'boot_id':'other'}):
            with self.assertRaises(PairLedgerError):self.activate(p)
        with patch.object(entry,'check_processes',side_effect=RuntimeError('Live controller')):
            with self.assertRaises(RuntimeError):self.activate(p)
        for path in (self.root/'robot_tools/pair_task_enrollment.py',self.task_file,self.close,
                     Path(p['recovery_evidence']['rgb']['images']['front']['path'])):
            raw=path.read_bytes();path.write_bytes(raw+b' ')
            with self.subTest(path=path),self.assertRaises(PairLedgerError):self.activate(p)
            path.write_bytes(raw)
        self.assertEqual(before,self.rows())
        with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_faults SET reason=reason||'changed' WHERE id=1")
        with self.assertRaises(PairLedgerError):self.activate(p)

    def test_commit_time_evidence_change_and_age_expiry_roll_back_new_rows(self):
        p=self.prepare();before=self.rows()
        for path in (self.passive['right'],self.rgb,self.close,self.startup_dir/'events.jsonl',
                     Path(p['recovery_evidence']['rgb']['images']['front']['path'])):
            raw=path.read_bytes();calls=[]
            def clock():
                calls.append(None)
                if len(calls)==2:path.write_bytes(raw+b' ')
                return self.now
            with self.subTest(path=path),self.assertRaises(PairLedgerError):
                entry.activate_task(p,self.authorize(p),project_root=self.root,clock=clock)
            path.write_bytes(raw);self.assertEqual(before,self.rows())
        ticks=iter((8000.,8031.))
        with self.assertRaisesRegex(PairLedgerError,'Fresh post-startup'):
            entry.activate_task(p,self.authorize(p),project_root=self.root,clock=lambda:next(ticks))
        self.assertEqual(before,self.rows())

    def test_locked_open_rechecks_prepare_restriction_before_device_construction(self):
        self.activate(self.prepare());profile=json.loads((self.root/'configs/robot.json').read_text());created=[]
        with patch.object(entry,'preparation_only',side_effect=(False,True)):
            host=PairHost(self.root/'runs',profile,'new-supervised-task',self.task,1000,10800,
                device_factory=lambda *a:created.append(True),clock=lambda:self.now,background=False,connection_mode='ready')
            with self.assertRaisesRegex(RuntimeError,'preparation connection'):host.open()
        self.assertEqual(created,[]);self.assertIsNone(host.ledger.peek_status()['owner'])

    def test_pending_contact_and_bad_original_send_still_refuse(self):
        baseline=self.rows()
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET status='pending' WHERE event_id='rgb-failed'")
        with self.assertRaises(PairLedgerError):self.prepare()
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET status='complete' WHERE event_id='rgb-failed'")
            receipt=json.loads(next(r['receipt_json'] for r in baseline['pair_events'] if r['event_id']=='rgb-failed'))
            receipt['device_receipt']['original_event']['frame_receipts'][0]['outcome']='unknown'
            db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='rgb-failed'",(json.dumps(receipt),))
        with self.assertRaises(PairLedgerError):self.prepare()
        with sqlite3.connect(self.path) as db:
            original=next(r['receipt_json'] for r in baseline['pair_events'] if r['event_id']=='rgb-failed')
            db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='rgb-failed'",(original,))
            db.execute('CREATE TABLE IF NOT EXISTS pair_grasp_episodes(run_id TEXT,state_json TEXT)')
            db.execute('INSERT INTO pair_grasp_episodes(run_id,state_json) VALUES(?,?)',('round-2',json.dumps({'status':'retained'})))
        with self.assertRaisesRegex(PairLedgerError,'Unresolved grasp'):self.prepare()


if __name__ == '__main__': unittest.main()
