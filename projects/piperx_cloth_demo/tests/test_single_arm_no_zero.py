"""Offline peer maintenance isolation and one-command jaw enable/clamp."""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import maintenance_scope as scope
from robot_tools.pair_ledger import PairLedger
from robot_tools.service import ToolService
from robot_tools.single_supervised_actions import _SingleSupervisedAction
from robot_tools.feedback_tolerance import PROFILE_KEY, validate_policy
from test_backend import PROFILE
from test_single_supervised_actions import SingleActionFixture


def history(root, profile):
    (root/'configs').mkdir();(root/'runs').mkdir()
    (root/'configs/robot.json').write_text(json.dumps(profile))
    path=root/'runs/pair_sessions.sqlite'
    PairLedger(path,'seed',{'task':'schema'})
    directory=root/'runs/calibration_test';directory.mkdir()
    counts={s:dict(attempted_frames=int(s=='right'),sent_frames=int(s=='right'),blocked_frames=0)
            for s in ('left','right')}
    result=dict(run_id='calibration_test',arm='right',operation='calibrate_empty_gripper_zero',
        ok=False,status='aborted_after_dispatch',transmission_counts=counts,hardware_commands_sent=1,
        guard_violations=[],target_commands_sent=0,enable_commands_sent=0,stop_commands_sent=0,retries=0,
        errors=[dict(type='RuntimeError',detail='Manufacturer zero ACK missing; no retry')],
        cleanup={'arms':{s:dict(status='disconnected') for s in ('left','right')}},
        **{p:{'right':{'gripper':{'foc_status':{'driver_enable_status':False}}}} for p in ('before','after')})
    receipt=directory/'result.json';receipt.write_text(json.dumps(result))
    (directory/'request.json').write_text(json.dumps(dict(run_id='calibration_test',arm='right',boot_id='boot')))
    events=[dict(event='gripper_zero_intent',arm='right',unix_s=1.,arbitration_id=0x159,data_hex='00000000000000ae'),
            dict(event='gripper_zero_returned',arm='right',unix_s=2.,transmission_counts=counts)]
    (directory/'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events)+'\n')
    with sqlite3.connect(path) as db:
        db.execute('DELETE FROM pair_runs')
        db.execute("INSERT INTO pair_faults(run_id,owner,reason,at) VALUES('calibration_test',NULL,'gripper_zero_in_progress',1)")
        fid=db.execute('SELECT MAX(id) FROM pair_faults').fetchone()[0]
        db.execute("INSERT INTO pair_faults(run_id,owner,reason,at) VALUES('calibration_test',NULL,'gripper_zero_calibration_failed_or_uncertain',2)")
        db.execute('UPDATE pair_scope SET fault_id=?,active_run_id=NULL,owner=NULL WHERE id=1',(fid,))
        db.execute('CREATE TABLE pair_gripper_zero_calibrations (boot_id TEXT, arm TEXT, run_id TEXT, status TEXT, '
                   'record_path TEXT, started_at REAL, result_sha256 TEXT, fault_id INTEGER)')
        db.execute('INSERT INTO pair_gripper_zero_calibrations VALUES(?,?,?,?,?,?,?,?)',
                   ('boot','right','calibration_test','failed',str(receipt),1.,hashlib.sha256(receipt.read_bytes()).hexdigest(),fid))
    return path,receipt


class ScopeTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)
        self.path,self.receipt=history(self.root,PROFILE)
        self.patch=patch.object(scope,'boot_identity',lambda:dict(boot_id='boot'))
        self.patch.start();self.addCleanup(self.patch.stop)

    def inspect(self, arm='left'):
        return scope.inspect(self.root/'runs',arm)

    def update_result(self, change):
        result=json.loads(self.receipt.read_text());change(result);self.receipt.write_text(json.dumps(result))
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_gripper_zero_calibrations SET result_sha256=?',
                       (hashlib.sha256(self.receipt.read_bytes()).hexdigest(),))

    def test_peer_only_and_original_fault_unchanged(self):
        before=self.path.read_bytes()
        self.assertFalse(self.inspect()['calibration_required'])
        self.assertFalse(self.inspect()['calibration_resolved'])
        self.assertEqual(before,self.path.read_bytes())
        with self.assertRaises(RuntimeError):self.inspect('right')

    def test_pending_calibration_refused(self):
        with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_gripper_zero_calibrations SET status='pending'")
        with self.assertRaises(RuntimeError):self.inspect()

    def test_unknown_partial_send_refused(self):
        self.update_result(lambda r:r['transmission_counts']['right'].update(sent_frames=0))
        with self.assertRaises(RuntimeError):self.inspect()

    def test_extra_arm_command_refused(self):
        self.update_result(lambda r:r.update(target_commands_sent=1))
        with self.assertRaises(RuntimeError):self.inspect()

    def test_other_failure_refused(self):
        self.update_result(lambda r:r.update(errors=[dict(type='RuntimeError',detail='left body drift')]))
        with self.assertRaises(RuntimeError):self.inspect()

    def test_new_fault_refused(self):
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_faults(run_id,owner,reason,at) VALUES('other',NULL,'driver_failure',3)")
        with self.assertRaises(RuntimeError):self.inspect()

    def test_owner_refused(self):
        with sqlite3.connect(self.path) as db:db.execute("UPDATE pair_scope SET owner='live'")
        with self.assertRaises(RuntimeError):self.inspect()

    def test_foreign_boot_refused(self):
        with patch.object(scope,'boot_identity',lambda:dict(boot_id='other')):
            with self.assertRaises(RuntimeError):self.inspect()

    def test_changed_journal_payload_refused(self):
        p=self.receipt.with_name('events.jsonl')
        p.write_text(p.read_text().replace('00000000000000ae','0000000000000100'))
        with self.assertRaises(RuntimeError):self.inspect()

    def test_crashed_claim_cannot_retry(self):
        scope.claim(self.root/'runs','action','left','single_supervised_gripper',self.root/'runs/action/result.json')
        with self.assertRaisesRegex(RuntimeError,'failed or pending'):self.inspect()

    def test_dual_executor_never_gets_exception(self):
        with self.assertRaises(RuntimeError):scope.claim(self.root/'runs','action','left','supervised_gripper',self.receipt)

    def test_service_other_arm_and_dual_action_refuse_before_device(self):
        service=ToolService(self.root)
        with patch('robot_tools.arms._load_sdk',side_effect=AssertionError('device must not be constructed')):
            for name,arm in [('robot_single_arm_gripper_once','right'),('robot_gripper_once','left')]:
                with self.assertRaises(RuntimeError):
                    service.call(name,dict(arm=arm,width_m=.029,nominal_force_N=.2))


class SingleJawTests(SingleActionFixture):
    def enable_on_command(self):
        self.make_robots(selected='left')
        r=self.robots['left'];r.gripper_enabled=False
        self.robots['right'].width=-.00133
        original=r.comm.send_bus.send
        def send(frame,*a,**kw):
            result=original(frame,*a,**kw)
            r.gripper_enabled=True
            return result
        r.comm.send_bus.send=send

    def test_disabled_left_clamps_and_enables_in_one_nonzeroing_frame(self):
        self.enable_on_command()
        result=self.run_tool(grip=True)
        self.assertTrue(result['ok'],result)
        self.assertEqual(len(self.robots['left'].sent),1)
        self.assertEqual(self.robots['right'].sent,[])
        self.assertEqual(bytes(self.robots['left'].sent[0].data)[6:],bytes((1,0)))
        self.assertTrue(result['gripper_enable_requested'])
        self.assertTrue(result['gripper_enable_observed'])
        self.assertFalse(result['grasp_verified'])
        self.assertFalse(result['calibration_required'])
        self.assertEqual(result['after']['right']['gripper']['width_m'],-.00133)

    def test_disabled_left_cannot_lift_before_enable(self):
        self.enable_on_command()
        self.assert_no_tx(self.run_tool())

    def test_missing_enable_feedback_fails_once_without_retry(self):
        self.make_robots(selected='left');self.robots['left'].gripper_enabled=False
        result=self.run_tool(grip=True)
        self.assertFalse(result['ok']);self.assertEqual(result['hardware_commands_sent'],1)
        self.assertFalse(result['gripper_enable_observed'])

    def test_enable_regression_still_fails(self):
        self.enable_on_command()
        def hook(robot,state):
            if robot.side=='left' and self.clock.elapsed>3.2:
                state['gripper']['foc_status']['driver_enable_status']=False
        self.hook=hook
        result=self.run_tool(grip=True)
        self.assertFalse(result['ok']);self.assertEqual(result['hardware_commands_sent'],1)

    def test_negative_selected_aperture_still_refused(self):
        self.make_robots(selected='left');self.robots['left'].width=-.00133
        self.assert_no_tx(self.run_tool(grip=True))

    def test_passive_disabled_jaw_drift_still_refused(self):
        self.make_robots(selected='left')
        def hook(robot,state):
            if robot.side=='right':state['gripper']['width_m']=-.00133+self.clock.elapsed*.003
        self.hook=hook
        self.assert_no_tx(self.run_tool(grip=True))

    def test_maintenance_peer_must_stay_disabled(self):
        self.make_robots(selected='left');self.robots['right'].gripper_enabled=True
        self.profile['single_arm_maintenance_scope']=dict(selected_arm='left',excluded_arm='right')
        self.assert_no_tx(self.run_tool(grip=True))

    def test_existing_j4_policy_applies_to_window_without_widening_other_axes(self):
        self.profile[PROFILE_KEY]=validate_policy(dict(profile='right_j4_bounded_v1',source='user',statement='本次允许右J4约0.4度波动'))
        action=_SingleSupervisedAction(self.profile,lambda *a:None,'left','gripper',.02)
        states={s:self.snapshot(r,r.gripper) for s,r in self.robots.items()}
        window=action.new_window(states);window['right']['qhigh'][3]+=.007
        window['right']['rotation_span_rad']=.007
        self.assertTrue(action.window_stable(window))
        window['right']['qhigh'][2]+=.0031
        self.assertFalse(action.window_stable(window))

    def test_real_service_claim_to_fake_can_preserves_original_fault(self):
        self.enable_on_command()
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);root=Path(tmp.name)
        dbpath,_=history(root,self.profile)
        with sqlite3.connect(dbpath) as db:before=list(db.execute('SELECT * FROM pair_faults'))
        with patch.object(scope,'boot_identity',lambda:dict(boot_id='boot')):
            result=ToolService(root).call('robot_single_arm_gripper_once',dict(arm='left',width_m=.02,nominal_force_N=.2))
            self.assertTrue(result['ok'],result)
            self.assertEqual(scope.inspect(root/'runs','left')['excluded_arm'],'right')
        with sqlite3.connect(dbpath) as db:
            self.assertEqual(before,list(db.execute('SELECT * FROM pair_faults')))
            self.assertIsNotNone(db.execute('SELECT fault_id FROM pair_scope').fetchone()[0])
            self.assertEqual(db.execute('SELECT status FROM '+scope.TABLE).fetchone()[0],'complete')
        self.assertEqual(self.robots['right'].sent,[])

    def test_new_failed_left_action_is_durable_and_blocks_repeat(self):
        self.make_robots(selected='left');self.robots['left'].gripper_enabled=False
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);root=Path(tmp.name)
        history(root,self.profile)
        service=ToolService(root)
        with patch.object(scope,'boot_identity',lambda:dict(boot_id='boot')):
            args=dict(arm='left',width_m=.02,nominal_force_N=.2)
            self.assertFalse(service.call('robot_single_arm_gripper_once',args)['ok'])
            with self.assertRaisesRegex(RuntimeError,'failed or pending'):
                service.call('robot_single_arm_gripper_once',args)
        self.assertEqual(len(self.robots['left'].sent),1)
