"""Offline legacy-coordinate compatibility; fake sockets/CAN only."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import tempfile
import time
from unittest.mock import patch

from robot_tools import arms, legacy_controller as legacy, maintenance_scope as scope
from robot_tools.service import ToolService
from test_single_arm_no_zero import history
from test_single_supervised_actions import SingleActionFixture


class LegacyControllerTests(SingleActionFixture):
    def setUp(self):
        super().setUp()
        self.make_robots(selected='left')
        for cfg in self.profile['arms'].values(): cfg['model']='piper_x'
        self.profile['physical_model_confirmation']={'left':'piper_x','right':'piper_x'}
        self.stack.enter_context(patch.object(legacy,'boot_identity',lambda:dict(boot_id='boot')))
        self.stack.enter_context(patch.object(scope,'boot_identity',lambda:dict(boot_id='boot')))
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)
        self.dbpath,_=history(self.root,self.profile)
        self.old='single_supervised_move_model_refused'
        directory=self.root/'runs'/self.old;directory.mkdir()
        self.receipt=directory/'result.json'
        scope.claim(self.root/'runs',self.old,'left','single_supervised_move',self.receipt)
        self.rejected=dict(run_id=self.old,operation='single_supervised_move',selected_arm='left',
            ok=False,status='refused_before_send',errors=[dict(type='RuntimeError',
                detail='Manufacturer FK does not agree with current flange feedback')],guard_violations=[],
            transmission_counts={s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in ('left','right')},
            cleanup={'arms':{s:dict(status='disconnected',physically_stopped=None) for s in ('left','right')}},
            hardware_commands_sent=0,target_calls_sent=0,target_commands_sent=0,
            enable_commands_sent=0,stop_commands_sent=0,retries=0)
        self.receipt.write_text(json.dumps(self.rejected))
        self.receipt.with_name('events.jsonl').write_text(json.dumps(dict(event='single_supervised_action_intent'))+'\n')
        scope.finish(self.root/'runs',self.old,self.rejected,self.receipt)
        self.grant=dict(version=legacy.VERSION,arm='left',source='user',statement='那就按照旧的piper来控制',
            boot_id='boot',refused_run_id=self.old,refused_sha256=hashlib.sha256(self.receipt.read_bytes()).hexdigest(),
            expires_at_unix_s=2000000000.)
        self.profile[legacy.KEY]=self.grant
        # Real vendor FK for a fixed legal synthetic posture; fake dynamics are
        # deliberately not used as physical model or extraction evidence.
        self.pose=arms.vendor_fk('piper',self.joints['left'],self.profile['sdk_path'])['pose_m_rad']
        self.robots['left'].motion.origin=self.pose[:]
        self.target=self.pose[:];self.target[2]+=.001
        motion=self.robots['left'].motion
        def pose():
            if motion.started is None:return motion.origin[:],0
            length=math.dist(motion.target[:3],motion.origin[:3])
            fraction=min(1.,(self.clock.elapsed-motion.started)*motion.speed/length)
            return [a+(b-a)*fraction for a,b in zip(motion.origin,motion.target)],int(fraction<1.)
        motion.pose=pose

    def service(self):
        (self.root/'configs/robot.json').write_text(json.dumps(self.profile))
        return ToolService(self.root)

    def invoke(self):
        return self.service().call('robot_single_arm_move_once',dict(arm='left',target_pose_m_rad=self.target))

    def test_selected_limit_diagnostic_preserves_existing_fault_and_action_rows(self):
        from robot_tools import joint_limits
        with sqlite3.connect(self.dbpath) as db:
            faults=list(db.execute('SELECT * FROM pair_faults'))
            actions=list(db.execute('SELECT * FROM '+scope.TABLE))
        with patch.object(joint_limits,'inspect_joint_limits',return_value={'ok':True}) as query:
            r=self.service().call('robot_inspect_joint_limits',{'arm':'left'})
            self.assertTrue(r['ok']);self.assertEqual(query.call_args.kwargs,{'arm':'left'})
        with sqlite3.connect(self.dbpath) as db:
            self.assertEqual(faults,list(db.execute('SELECT * FROM pair_faults')))
            self.assertEqual(actions,list(db.execute('SELECT * FROM '+scope.TABLE)))
        with self.assertRaises(RuntimeError):self.service().call('robot_inspect_joint_limits',{})

    def test_selected_limit_diagnostic_does_not_compete_with_owner(self):
        from robot_tools import joint_limits
        with sqlite3.connect(self.dbpath) as db:db.execute("UPDATE pair_scope SET owner='live' WHERE id=1")
        with patch.object(joint_limits,'inspect_joint_limits') as query:
            with self.assertRaises(RuntimeError):self.service().call('robot_inspect_joint_limits',{'arm':'left'})
            query.assert_not_called()

    def wrist_review_fixture(self):
        from test_joint_limits import reply_bytes
        self.five_mm_step()
        self.joints['left'][:]=[math.radians(x) for x in [-5.048,136.754,-108.955,-1.917,70.519,-5.558]]
        self.pose=arms.vendor_fk('piper',self.joints['left'],self.profile['sdk_path'])['pose_m_rad']
        self.robots['left'].motion.origin=self.pose[:]
        self.target=self.pose[:];self.target[2]+=.005
        rid='single_supervised_move_wrist_failed';qid='joint_limits_current_left'
        fp=self.root/'runs'/rid/'result.json';fp.parent.mkdir()
        scope.claim(self.root/'runs',rid,'left','single_supervised_move',fp,self.grant)
        width=self.robots['left'].width
        v=dict(joint_index=5,observed_rad=math.radians(70.2),minimum_rad=-1.22173,maximum_rad=1.22173)
        f=copy.deepcopy(self.rejected);f.update(run_id=rid,status='aborted_after_dispatch',
            legacy_controller_compatibility=copy.deepcopy(self.grant),
            errors=[dict(type='RuntimeError',detail='Selected joint feedback exceeds action-specific boundary allowance: '+repr([v]))],
            requested_target=self.pose[:],before={'left':{'gripper':{'width_m':width}}},
            after={'left':{'timestamp':200.}},hardware_commands_sent=4,target_calls_sent=1,
            target_commands_sent=3,arm_target_commands_sent=3,mode_commands_sent=1,gripper_target_commands_sent=0)
        f['transmission_counts']['left']=dict(attempted_frames=4,sent_frames=4,blocked_frames=0)
        fp.write_text(json.dumps(f));fp.with_name('events.jsonl').write_text(json.dumps(dict(event='single_supervised_action_sent_unconfirmed',unix_s=199.))+'\n')
        fp.with_name('request.json').write_text(json.dumps({'arms':self.profile['arms']}));scope.finish(self.root/'runs',rid,f,fp)
        lp=self.root/'runs'/qid/'result.json';lp.parent.mkdir()
        rows={}
        for j in range(1,7):
            lo,hi=(-890,890) if j in (4,5) else (-1800,1800)
            rows[str(j)]={'status':'confirmed','raw_response_hex':reply_bytes(j,lo,hi).hex(),
                          'raw_min_angle_tenth_deg':lo,'raw_max_angle_tenth_deg':hi}
        limits=dict(run_id=qid,ok=True,operation='inspect_joint_limits',selected_arm='left',query_sides=['left'],
            errors=[],guard_violations=[],transmission_counts={s:dict(attempted_frames=6 if s=='left' else 0,
                sent_frames=6 if s=='left' else 0,blocked_frames=0) for s in ('left','right')},
            joint_limit_queries_sent=6,controller_limits_changed=False,sdk_joint_limits_changed=False,
            actuator_commands_sent=0,mode_commands_sent=0,target_commands_sent=0,enable_commands_sent=0,stop_commands_sent=0,retries=0,
            joint_limits={'left':rows,'right':{}},cleanup=copy.deepcopy(f['cleanup']),
            after={'left':{'pose_m_rad':self.pose[:],'gripper':{'width_m':width},'arm_status':{'motion_status':0}}},
            drift={'left':{'joint_rad':0.,'position_m':0.,'gripper_m':0.}})
        lp.write_text(json.dumps(limits));lp.with_name('request.json').write_text(json.dumps(dict(arms=self.profile['arms'],arguments={'arm':'left'},started_unix_s=300.)))
        self.grant['reviewed_wrist_limit_failure']=dict(run_id=rid,sha256=hashlib.sha256(fp.read_bytes()).hexdigest(),
            limits_run_id=qid,limits_sha256=hashlib.sha256(lp.read_bytes()).hexdigest(),source='user',statement='J5按厂家89度，继续上提')
        return fp,lp

    def test_reviewed_current_wrist_readback_uses_x_limit_and_preserves_fault(self):
        fp,lp=self.wrist_review_fixture();before=fp.read_bytes()
        r=self.invoke();self.assertTrue(r['ok'],r)
        self.assert_frame_ids(r,[0x151,0x152,0x153,0x154])
        self.assertIn('89',r['joint_limits_source'])
        self.assertIn('pre_send_endpoint_prediction',r)
        self.assertEqual(fp.read_bytes(),before)
        with sqlite3.connect(self.dbpath) as db:
            self.assertEqual(db.execute('SELECT status FROM '+scope.TABLE+' WHERE run_id=?',(fp.parent.name,)).fetchone()[0],'failed')

    def test_wrist_review_rejects_wrong_readback_and_partial_old_send(self):
        fp,lp=self.wrist_review_fixture()
        from robot_tools.legacy_wrist_review import review
        with sqlite3.connect(self.dbpath) as db:
            db.row_factory=sqlite3.Row;row=dict(db.execute('SELECT * FROM '+scope.TABLE+' WHERE run_id=?',(fp.parent.name,)).fetchone())
        original=json.loads(lp.read_text());bad=copy.deepcopy(original)
        bad['joint_limits']['left']['5']['raw_max_angle_tenth_deg']=700
        lp.write_text(json.dumps(bad));self.grant['reviewed_wrist_limit_failure']['limits_sha256']=hashlib.sha256(lp.read_bytes()).hexdigest()
        with self.assertRaises(RuntimeError):review(self.root/'runs',row,self.grant)
        lp.write_text(json.dumps(original));self.grant['reviewed_wrist_limit_failure']['limits_sha256']=hashlib.sha256(lp.read_bytes()).hexdigest()
        f=json.loads(fp.read_text());f['transmission_counts']['left']['sent_frames']=3
        fp.write_text(json.dumps(f));digest=hashlib.sha256(fp.read_bytes()).hexdigest();row['result_sha256']=digest;self.grant['reviewed_wrist_limit_failure']['sha256']=digest
        with self.assertRaisesRegex(RuntimeError,'partial'):review(self.root/'runs',row,self.grant)

    def test_wrist_review_cannot_cover_new_pending_action(self):
        self.wrist_review_fixture()
        scope.claim(self.root/'runs','single_supervised_move_other','left','single_supervised_move',
                    self.root/'runs/single_supervised_move_other/result.json',self.grant)
        with self.assertRaises(RuntimeError):self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_endpoint_prediction_blocks_target_over_89_before_send(self):
        self.wrist_review_fixture()
        from robot_tools.legacy_joint_prediction import predict
        from scipy.spatial.transform import Rotation
        from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET
        target=self.pose[:];target[2]+=.1
        with self.assertRaisesRegex(RuntimeError,'before send'):
            predict(self.profile,self.joints['left'],target,ROBOT_JOINT_LIMIT_PRESET['piper_x'],
                    self.grant,lambda a,b:(Rotation.from_euler('xyz',a[3:]).inv()*Rotation.from_euler('xyz',b[3:])).magnitude())

    def test_slow_prediction_refreshes_feedback_before_single_send(self):
        self.wrist_review_fixture()
        from robot_tools import legacy_joint_prediction as prediction
        original=prediction.predict
        def slow(*args):
            result=original(*args);self.clock.sleep(.8);return result
        with patch.object(prediction,'predict',side_effect=slow):r=self.invoke()
        self.assertTrue(r['ok'],r)
        self.assertTrue(r['feedback_refreshed_after_endpoint_prediction'])
        self.assertLessEqual(r['last_checked_feedback_age_s'],.1)
        self.assertAlmostEqual(r['endpoint_prediction_duration_s'],.8)
        self.assert_frame_ids(r,[0x151,0x152,0x153,0x154])

    def test_prediction_over_one_second_is_zero_tx(self):
        self.wrist_review_fixture()
        from robot_tools import legacy_joint_prediction as prediction
        original=prediction.predict
        def slow(*args):
            result=original(*args);self.clock.sleep(1.01);return result
        with patch.object(prediction,'predict',side_effect=slow):r=self.invoke()
        self.assertFalse(r['ok']);self.assertEqual(r['hardware_commands_sent'],0)
        self.assertIn('1 s',r['errors'][0]['detail'])

    def test_drift_during_prediction_is_zero_tx(self):
        self.wrist_review_fixture()
        from robot_tools import legacy_joint_prediction as prediction
        original=prediction.predict
        def drift(*args):
            result=original(*args);self.clock.sleep(.25)
            self.joints['left'][0]+=.01
            return result
        with patch.object(prediction,'predict',side_effect=drift):r=self.invoke()
        self.assertFalse(r['ok']);self.assertEqual(r['hardware_commands_sent'],0)

    def prediction_age_fixture(self):
        self.wrist_review_fixture()
        from robot_tools.single_supervised_actions import _SingleSupervisedAction
        original=_SingleSupervisedAction.check_freshness
        raised=[]
        def old_check(device,state):
            if not raised and 'pre_send_endpoint_prediction' in device.report:
                raised.append(True)
                raise RuntimeError('Single action requires receive age/skew within 100 ms including processing; age=0.252007 skew=0.024324')
            return original(device,state)
        with patch.object(_SingleSupervisedAction,'check_freshness',old_check):r=self.invoke()
        self.assertFalse(r['ok']);self.assertEqual(r['hardware_commands_sent'],0)
        path=Path(r['record_path'])
        self.grant['reviewed_prediction_age_refusal']={'run_id':r['run_id'],'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        return path

    def test_exact_prediction_age_review_preserves_failure_and_count(self):
        path=self.prediction_age_fixture();old=path.read_bytes()
        before=scope.inspect(self.root/'runs','left',self.grant)['task_budget_receipt']['used_steps']
        q=copy.deepcopy(self.joints);target=self.target[:]
        self.make_robots(selected='left');self.joints=q;self.target=target
        motion=self.robots['left'].motion;motion.origin=self.pose[:]
        def pose():
            if motion.started is None:return motion.origin[:],0
            length=math.dist(motion.target[:3],motion.origin[:3])
            fraction=min(1.,(self.clock.elapsed-motion.started)*motion.speed/length)
            return [a+(b-a)*fraction for a,b in zip(motion.origin,motion.target)],int(fraction<1.)
        motion.pose=pose
        r=self.invoke();self.assertTrue(r['ok'],r)
        self.assertEqual(path.read_bytes(),old)
        self.assertEqual(scope.inspect(self.root/'runs','left',self.grant)['task_budget_receipt']['used_steps'],before+1)
        with sqlite3.connect(self.dbpath) as db:
            self.assertEqual(db.execute('SELECT status FROM '+scope.TABLE+' WHERE run_id=?',(path.parent.name,)).fetchone()[0],'failed')

    def test_prediction_age_review_rejects_any_attempted_send(self):
        path=self.prediction_age_fixture();r=json.loads(path.read_text())
        r['transmission_counts']['left']['attempted_frames']=1
        path.write_text(json.dumps(r));digest=hashlib.sha256(path.read_bytes()).hexdigest()
        self.grant['reviewed_prediction_age_refusal']['sha256']=digest
        with sqlite3.connect(self.dbpath) as db:
            db.execute('UPDATE '+scope.TABLE+' SET result_sha256=? WHERE run_id=?',(digest,path.parent.name))
        with self.assertRaisesRegex(RuntimeError,'zero-TX'):
            scope.inspect(self.root/'runs','left',self.grant)

    def live_reference_fixture(self):
        path=self.prediction_age_fixture();result=json.loads(path.read_text())
        del self.grant['reviewed_prediction_age_refusal']
        result['errors']=[{'type':'RuntimeError','detail':'Legacy +Z target exceeds selected step or existing 0.5 mm / 0.003 rad start-reference bands'}]
        result['requested_target']=result['before']['left']['pose_m_rad'][:]
        result['requested_target'][2]+=.005;result['requested_target'][4]+=.0045
        path.write_text(json.dumps(result));digest=hashlib.sha256(path.read_bytes()).hexdigest()
        self.grant['reviewed_live_start_refusal']={'run_id':path.parent.name,'sha256':digest}
        with sqlite3.connect(self.dbpath) as db:
            db.execute('UPDATE '+scope.TABLE+' SET result_sha256=? WHERE run_id=?',(digest,path.parent.name))
        return path

    def test_exact_live_reference_refusal_audit_preserves_original(self):
        path=self.live_reference_fixture();raw=path.read_bytes()
        scope.inspect(self.root/'runs','left',self.grant)
        self.assertEqual(raw,path.read_bytes());self.assertEqual(self.robots['left'].sent,[])

    def test_live_reference_refusal_cannot_cover_large_target_or_send(self):
        path=self.live_reference_fixture();original=json.loads(path.read_text())
        for mutation in ('large_rotation','sent'):
            result=copy.deepcopy(original)
            if mutation=='large_rotation':result['requested_target'][4]+=.02
            else:result['transmission_counts']['left']['attempted_frames']=1
            path.write_text(json.dumps(result));digest=hashlib.sha256(path.read_bytes()).hexdigest()
            self.grant['reviewed_live_start_refusal']['sha256']=digest
            with sqlite3.connect(self.dbpath) as db:
                db.execute('UPDATE '+scope.TABLE+' SET result_sha256=? WHERE run_id=?',(digest,path.parent.name))
            with self.subTest(mutation=mutation),self.assertRaises(RuntimeError):
                scope.inspect(self.root/'runs','left',self.grant)

    def continuation_window_fixture(self):
        self.wrist_review_fixture();self.original_task_budget()
        old_deadline=self.grant['expires_at_unix_s']
        self.stack.enter_context(patch.object(legacy.time,'time',lambda:old_deadline+10))
        self.grant['continuation_window']=dict(schema='legacy_explicit_continuation_window_v1',
            source='user',statement='明确同意修复后重新给三小时，原计数保留',
            starts_at_unix_s=old_deadline+5,expires_at_unix_s=old_deadline+10805,
            reviewed_failure_run_id=self.grant['reviewed_wrist_limit_failure']['run_id'])

    def test_explicit_later_window_keeps_original_deadline_and_all_steps(self):
        self.continuation_window_fixture()
        old_deadline=self.grant['expires_at_unix_s'];old_budget=copy.deepcopy(self.grant['task_budget'])
        before=scope.inspect(self.root/'runs','left',self.grant)['task_budget_receipt']
        r=self.invoke();self.assertTrue(r['ok'],r)
        after=scope.inspect(self.root/'runs','left',self.grant)['task_budget_receipt']
        self.assertEqual(after['used_steps'],before['used_steps']+1)
        self.assertEqual(after['original_expires_at_unix_s'],old_deadline)
        self.assertEqual(after['original_task_budget'],old_budget)
        self.grant['continuation_window']['starts_at_unix_s']+=1
        self.grant['continuation_window']['expires_at_unix_s']+=1
        with self.assertRaisesRegex(RuntimeError,'frozen continuation window'):
            scope.inspect(self.root/'runs','left',self.grant)

    def test_no_automatic_time_extension_and_invalid_windows_send_nothing(self):
        self.continuation_window_fixture();valid=copy.deepcopy(self.grant['continuation_window'])
        for change in ({'source':'model'}, {'statement':''}, {'reviewed_failure_run_id':'another'},
                       {'expires_at_unix_s':valid['expires_at_unix_s']+1},
                       {'starts_at_unix_s':valid['starts_at_unix_s']+20}, {'reset_steps':True}):
            self.grant['continuation_window']={**valid,**change}
            with self.subTest(change=change), self.assertRaises(RuntimeError):self.invoke()
            self.assertEqual(self.robots['left'].sent,[])
        del self.grant['continuation_window']
        with self.assertRaises(RuntimeError):self.invoke()

    def larger_step(self):
        self.grant['step_profile'] = dict(name=legacy.LARGER_STEP, source='user',
            statement='更多一些至少2mm以上一次', boot_id=self.grant['boot_id'],
            expires_at_unix_s=self.grant['expires_at_unix_s'])
        self.target = self.pose[:]
        self.target[2] += .0025

    def five_mm_step(self):
        self.larger_step()
        self.grant['step_profile'].update(name=legacy.FIVE_MM_STEP, statement='每次至少5mm')
        self.target[2] = self.pose[2] + .005

    def original_task_budget(self):
        # Preserve the parent deadline; synthetic clock puts this fixture
        # inside its original three-hour window without changing any receipts.
        started = self.grant['expires_at_unix_s'] - 10800
        self.stack.enter_context(patch.object(legacy.time, 'time', lambda:started+1000))
        with sqlite3.connect(self.dbpath) as db:
            db.execute('UPDATE '+scope.TABLE+' SET started_at=?', (started+1,))
        self.grant['task_budget'] = dict(name=legacy.TASK_BUDGET, source='user',
            statement='原任务3小时1000步', started_at_unix_s=started,
            max_duration_s=10800, max_steps=1000)

    def record_complete(self, name):
        path = self.root/'runs'/name/'result.json';path.parent.mkdir()
        scope.claim(self.root/'runs',name,'left','single_supervised_move',path,self.grant)
        result = dict(ok=True, legacy_controller_compatibility=copy.deepcopy(self.grant))
        path.write_text(json.dumps(result));scope.finish(self.root/'runs',name,result,path)

    def test_five_mm_single_dispatch_and_existing_physical_guards(self):
        self.five_mm_step()
        r = self.invoke()
        self.assertTrue(r['ok'],r)
        self.assert_frame_ids(r,[0x151,0x152,0x153,0x154])
        self.assertEqual(r['legacy_step_limits']['nominal_step_m'],.005)
        self.assertEqual(r['legacy_step_limits']['joint_change_rad'],.025)
        self.assertEqual(r['legacy_step_limits']['physical_lateral_m'],.001)
        self.assertEqual(r['enable_commands_sent'],0)
        self.assertEqual(r['retries'],0)

    def test_five_mm_never_falls_back_to_subfive_target(self):
        self.five_mm_step()
        for dz in (.00451,.005,.00549):
            legacy.check_up_target([0.]*6,[0.,0.,dz,0.,0.,0.],lambda a,b:0.,self.grant)
        for dz in (.001,.0025,.00449,.00551,-.005):
            with self.subTest(dz=dz), self.assertRaises(RuntimeError):
                legacy.check_up_target([0.]*6,[0.,0.,dz,0.,0.,0.],lambda a,b:0.,self.grant)

    def test_five_mm_model_motion_remains_bounded(self):
        self.five_mm_step();q=self.joints['left'];moved=q[:];moved[1]+=.014
        def observed(delta,rotation):
            with patch.object(arms,'vendor_fk',side_effect=[{'pose_m_rad':[0.]*6},
                    {'pose_m_rad':delta+[0.,0.,0.]}]):
                return legacy.model_delta(self.profile,q,moved,lambda a,b:rotation,self.grant)
        observed([.0002,-.00021,.00652],.0186)
        for delta,rotation in (([0.,0.,.00801],0.),([.00101,0.,.006],0.),
                              ([0.,0.,-.00051],0.),([0.,0.,.006],.02401)):
            with self.subTest(delta=delta,rotation=rotation), self.assertRaises(RuntimeError):
                observed(delta,rotation)

    def test_original_budget_carries_all_prior_calls_beyond_twenty(self):
        # Original zero-TX failure, old 1mm actions, then new task budget.
        for i in range(legacy.MAX_STEPS):self.record_complete('single_supervised_move_old_%d'%i)
        self.original_task_budget();self.five_mm_step()
        before=scope.inspect(self.root/'runs','left',self.grant)['task_budget_receipt']
        self.assertEqual(before['used_steps'],21)
        self.assertEqual(before['max_steps'],1000)
        r=self.invoke();self.assertTrue(r['ok'],r)
        self.assertEqual(scope.inspect(self.root/'runs','left',self.grant)['task_budget_receipt']['used_steps'],22)
        del self.grant['task_budget']
        with self.assertRaisesRegex(RuntimeError,'cannot be changed or removed'):
            scope.inspect(self.root/'runs','left',self.grant)

    def test_original_budget_rejects_later_clock_more_steps_or_arbitrary_limits(self):
        self.original_task_budget();valid=copy.deepcopy(self.grant['task_budget'])
        for change in ({'started_at_unix_s':valid['started_at_unix_s']+1},
                       {'max_steps':1001},{'max_steps':True},{'max_duration_s':10801},
                       {'source':'model'},{'used_steps':0}):
            with self.subTest(change=change):
                self.grant['task_budget']={**valid,**change}
                with self.assertRaises(RuntimeError):self.invoke()
                self.assertEqual(self.robots['left'].sent,[])

    def test_original_budget_enforces_thousand_including_reviewed_failure(self):
        self.original_task_budget()
        # Shared immutable synthetic receipt suffices for counting; real
        # claim/finish and all hash checks remain exercised above.
        path=self.root/'runs/budget_receipt.json'
        path.write_text(json.dumps(dict(ok=True,legacy_controller_compatibility=self.grant)))
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        with sqlite3.connect(self.dbpath) as db:
            source=db.execute('SELECT source_sha256 FROM '+scope.TABLE).fetchone()[0]
            db.executemany('INSERT INTO '+scope.TABLE+' VALUES(?,?,?,?,?,?,?)',[
                ('single_supervised_move_count_%d'%i,'left',source,'complete',str(path),digest,time.time())
                for i in range(999)])
        with self.assertRaisesRegex(RuntimeError,'step budget exhausted'):self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_original_budget_does_not_cover_new_failed_or_pending_action(self):
        self.original_task_budget();self.five_mm_step()
        scope.claim(self.root/'runs','single_supervised_move_unknown','left','single_supervised_move',
                    self.root/'runs/single_supervised_move_unknown/result.json',self.grant)
        with self.assertRaises(RuntimeError):self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_larger_step_requires_explicit_profile(self):
        self.target[2] = self.pose[2] + .0025
        self.assert_no_tx(self.invoke())

    def test_explicit_larger_step_dispatches_once_with_right_tx_blocked(self):
        self.larger_step()
        r = self.invoke()
        self.assertTrue(r['ok'], r)
        self.assert_frame_ids(r, [0x151, 0x152, 0x153, 0x154])
        self.assertEqual(r['legacy_step_limits']['nominal_step_m'], .0025)
        self.assertEqual(r['physical_model'], 'piper_x')
        self.assertEqual(r['gripper_target_commands_sent'], 0)
        self.assertEqual(r['enable_commands_sent'], 0)
        self.assertEqual(r['retries'], 0)
        self.assertFalse(r['grasp_verified'])

    def test_invalid_larger_authorization_never_opens_devices(self):
        self.larger_step()
        valid = copy.deepcopy(self.grant['step_profile'])
        for change in ({'name':'arbitrary'}, {'source':'model'}, {'statement':''},
                       {'boot_id':'other'}, {'expires_at_unix_s':1},
                       {'expires_at_unix_s':2000000001.}, {'max_step_m':1.}):
            with self.subTest(change=change):
                self.grant['step_profile'] = {**valid, **change}
                with self.assertRaises(RuntimeError): self.invoke()
                self.assertEqual(self.robots['left'].sent, [])

    def test_larger_step_keeps_axis_orientation_and_reference_bands(self):
        self.larger_step()
        legacy.check_up_target([0.]*6, [0.,0.,.0025,0.,0.,0.], lambda a,b:0., self.grant)
        for target, rotation in (([0.,0.,.00301,0.,0.,0.],0.),
                                 ([.00051,0.,.0025,0.,0.,0.],0.),
                                 ([0.,0.,-.001,0.,0.,0.],0.),
                                 ([0.,0.,.0025,0.,0.,0.],.00301)):
            with self.subTest(target=target, rotation=rotation):
                with self.assertRaises(RuntimeError):
                    legacy.check_up_target([0.]*6, target, lambda a,b:rotation, self.grant)

    def test_larger_model_envelope_remains_finite_and_default_unchanged(self):
        self.larger_step()
        q = self.joints['left']; moved = q[:]; moved[1] += .005
        def observed(delta, rotation, grant, joints=moved):
            poses = [{'pose_m_rad':[0.]*6}, {'pose_m_rad':delta+[0.,0.,0.]}]
            with patch.object(arms, 'vendor_fk', side_effect=poses):
                return legacy.model_delta(self.profile, q, joints, lambda a,b:rotation, grant)
        observed([0.,0.,.00325], .0092, self.grant)
        with self.assertRaises(RuntimeError): observed([0.,0.,.00325], .0092, None)
        excessive = moved[:]; excessive[1] = q[1]+.0251
        for delta, rotation, joints in (([0.,0.,.00451],0.,moved),
                ([.00101,0.,.003],0.,moved), ([0.,0.,-.00051],0.,moved),
                ([0.,0.,.003],.0121,moved), ([0.,0.,.003],0.,excessive)):
            with self.subTest(delta=delta, rotation=rotation):
                with self.assertRaises(RuntimeError): observed(delta,rotation,self.grant,joints)

    def test_larger_profile_counts_prior_default_actions_toward_same_budget(self):
        original = copy.deepcopy(self.grant)
        for i in range(legacy.MAX_STEPS):
            if i == 10: self.larger_step()
            name = 'single_supervised_move_budget_%02d' % i
            path = self.root/'runs'/name/'result.json';path.parent.mkdir()
            scope.claim(self.root/'runs', name, 'left', 'single_supervised_move', path, self.grant)
            result = dict(ok=True, legacy_controller_compatibility=copy.deepcopy(self.grant))
            path.write_text(json.dumps(result));scope.finish(self.root/'runs',name,result,path)
        with self.assertRaisesRegex(RuntimeError, 'step budget exhausted'): self.invoke()
        self.assertEqual(self.robots['left'].sent, [])
        self.assertTrue(legacy.same_authorization(original, self.grant))
        altered = copy.deepcopy(self.grant);altered['expires_at_unix_s'] += 1
        self.assertFalse(legacy.same_authorization(original, altered))

    def test_failed_larger_action_stays_latched_when_selecting_default(self):
        self.larger_step();self.target[2] = self.pose[2]+.01
        self.assert_no_tx(self.invoke())
        del self.grant['step_profile'];self.target[2] = self.pose[2]+.001
        with self.assertRaises(RuntimeError): self.invoke()

    def test_review_to_existing_fake_can_preserves_failed_rows_and_physical_model(self):
        with sqlite3.connect(self.dbpath) as db: before=list(db.execute('SELECT * FROM pair_faults'))
        old=self.receipt.read_bytes()
        r=self.invoke()
        self.assertTrue(r['ok'],r)
        self.assert_frame_ids(r,[0x151,0x152,0x153,0x154])
        self.assertEqual(r['physical_model'],'piper_x')
        self.assertEqual(r['controller_coordinate_model'],'piper')
        self.assertEqual(old,self.receipt.read_bytes())
        with sqlite3.connect(self.dbpath) as db:
            self.assertEqual(before,list(db.execute('SELECT * FROM pair_faults')))
            self.assertEqual(db.execute('SELECT status FROM '+scope.TABLE+' WHERE run_id=?',(self.old,)).fetchone()[0],'failed')
        self.assertEqual(scope.inspect(self.root/'runs','left',self.grant)['excluded_arm'],'right')
        self.assertTrue(all(c['model']=='piper_x' for c in self.profile['arms'].values()))

    def test_missing_explicit_grant_never_continues_failed_action(self):
        del self.profile[legacy.KEY]
        with self.assertRaises(RuntimeError): self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_wrong_hash_never_reinterprets_failure(self):
        self.grant['refused_sha256']='0'*64
        with self.assertRaises(RuntimeError): self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_expired_authorization_no_device(self):
        self.grant['expires_at_unix_s']=1
        with self.assertRaises(RuntimeError): self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_right_motion_not_authorized(self):
        with self.assertRaises(RuntimeError):
            self.service().call('robot_single_arm_move_once',dict(arm='right',target_pose_m_rad=self.target))
        self.assertEqual(self.robots['right'].sent,[])

    def test_physical_profile_cannot_be_replaced_by_piper(self):
        self.profile['arms']['left']['model']='piper'
        with self.assertRaises(RuntimeError): self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_ten_mm_target_refused_before_send_and_latched(self):
        self.target[2]=self.pose[2]+.01
        r=self.invoke();self.assert_no_tx(r)
        self.target[2]=self.pose[2]+.001
        with self.assertRaises(RuntimeError): self.invoke()

    def test_downward_or_lateral_targets_are_not_legacy_trials(self):
        for target in ([0,0,-.001,0,0,0],[.001,0,.001,0,0,0]):
            with self.assertRaises(RuntimeError):legacy.check_up_target([0.]*6,target,lambda a,b:0.)

    def test_bounded_changed_start_does_not_require_exact_coordinates(self):
        legacy.check_up_target([.000137,-.000012,.000163,0.,0.,0.],
                               [0.,0.,.001,0.,0.,0.],lambda a,b:.00072)

    def test_start_reference_band_does_not_allow_large_rotation(self):
        with self.assertRaises(RuntimeError):
            legacy.check_up_target([0.]*6,[0.,0.,.001,0.,0.,0.],lambda a,b:.0031)

    def test_x_model_envelope_not_ordinary_piper_envelope(self):
        q=self.joints['left']; changed=q[:];changed[0]+=.024
        from robot_tools.single_gripper_prepare import _SingleGripperPrepare
        def distance(a,b):
            qa=_SingleGripperPrepare._manufacturer_quaternion(*a[3:])
            qb=_SingleGripperPrepare._manufacturer_quaternion(*b[3:])
            return 2*math.acos(min(1.,abs(sum(x*y for x,y in zip(qa,qb)))))
        with self.assertRaisesRegex(RuntimeError,'Physical Piper X'):
            legacy.model_delta(self.profile,q,changed,distance)

    def test_partial_or_unknown_original_failure_never_continues(self):
        self.rejected['transmission_counts']['left']['attempted_frames']=1
        self.receipt.write_text(json.dumps(self.rejected))
        digest=hashlib.sha256(self.receipt.read_bytes()).hexdigest()
        self.grant['refused_sha256']=digest
        with sqlite3.connect(self.dbpath) as db:
            db.execute('UPDATE '+scope.TABLE+' SET result_sha256=? WHERE run_id=?',(digest,self.old))
        with self.assertRaisesRegex(RuntimeError,'zero-TX'): self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_limit_intersection_retains_old_controller_limit(self):
        self.joints['left'][4]=math.radians(80)
        self.assert_no_tx(self.invoke())

    def test_new_pending_action_is_not_covered_by_old_review(self):
        scope.claim(self.root/'runs','single_supervised_move_new','left','single_supervised_move',
                    self.root/'runs/single_supervised_move_new/result.json',self.grant)
        with self.assertRaises(RuntimeError):self.invoke()
        self.assertEqual(self.robots['left'].sent,[])

    def test_exact_zero_tx_start_reference_review_keeps_original_failures(self):
        name='single_supervised_move_reference_refused'
        path=self.root/'runs'/name/'result.json';path.parent.mkdir()
        scope.claim(self.root/'runs',name,'left','single_supervised_move',path,self.grant)
        r=copy.deepcopy(self.rejected);r.update(run_id=name,
            legacy_controller_compatibility=copy.deepcopy(self.grant),
            before={'left':{'pose_m_rad':[.000137,-.000012,.000163,0.,0.,0.]}},
            requested_target=[0.,0.,.001,0.,0.,0.],
            errors=[dict(type='RuntimeError',detail='Legacy compatibility permits only <=1 mm +Z with unchanged controller orientation')])
        path.write_text(json.dumps(r));path.with_name('events.jsonl').write_text('')
        scope.finish(self.root/'runs',name,r,path)
        with self.assertRaises(RuntimeError):scope.inspect(self.root/'runs','left',self.grant)
        self.grant['reviewed_start_reference_refusal']={'run_id':name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        self.assertEqual(scope.inspect(self.root/'runs','left',self.grant)['selected_arm'],'left')
        with sqlite3.connect(self.dbpath) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM "+scope.TABLE+" WHERE status='failed'").fetchone()[0],2)
