"""Generic complete-send recovery: synthetic ledger, feedback and RGB only."""
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import joint_path as jp, pair_round as entry
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
from robot_tools.plug_recipe import render_plug_recipe
import test_pair_rgb_round as round_fixtures
from test_joint_path import context, sample, visual_joint_context, RAW


class TrackingRoundTests(unittest.TestCase):
    def setUp(self):
        f = round_fixtures.RGBExpiryRoundTests('runTest'); f.setUp(); self.addCleanup(f.doCleanups)
        self.f = f
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)/'bundle/projects/piperx_cloth_demo'
        self.root.parent.mkdir(parents=True); shutil.copytree(f.root,self.root)
        self.path = self.root/'runs/pair_sessions.sqlite'; self.now = 8000.
        self.close = self.root/f.close.name
        for source in Path(entry.__file__).parent.glob('*.py'):
            (self.root/'robot_tools'/source.name).write_bytes(source.read_bytes())
        self.task = {'task_id':'plug_transfer_left','roles':{'left':'task','right':'task'},
                     'site_context':{'workspace_clearance':{'source':'user','statement':'Synthetic current clearance'}}}
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            scope = db.execute("SELECT * FROM pair_rounds WHERE run_id='round-2'").fetchone()
            record = json.loads(scope['record_json']); proposal = record['proposal']
            record['new_contract']['task'] = copy.deepcopy(self.task)
            proposal['reviewed_contract'] = copy.deepcopy(record['new_contract'])
            proposal['proposal_sha256'] = entry._sha({k:v for k,v in proposal.items() if k!='proposal_sha256'})
            record['authorization']['proposal_sha256'] = proposal['proposal_sha256']
            db.execute("UPDATE pair_runs SET contract_json=? WHERE run_id='round-2'",
                       (json.dumps(record['new_contract'],sort_keys=True,separators=(',',':')),))
            db.execute('UPDATE pair_rounds SET proposal_sha256=?,authorization_sha256=?,record_json=? WHERE ordinal=?',
                       (proposal['proposal_sha256'],entry._sha(record['authorization']),json.dumps(record),scope['ordinal']))
        self.receipt,self.payload = self.tracking_failure()
        self.write_failure(self.receipt,self.payload)
        recipe = self.root.parent.parent/'tasks/plug_transfer_left.json'; recipe.parent.mkdir()
        recipe.write_bytes((Path(entry.__file__).parents[3]/'tasks/plug_transfer_left.json').read_bytes())
        rendered = self.root/'left-worker-recipe.json'
        rendered.write_text(json.dumps(render_plug_recipe(json.loads(recipe.read_text()),worker_arm='left',support_arm='right')))
        self.task_file = self.root/'new-task.json'
        self.task_file.write_text(json.dumps({'schema':'piper_supervised_plug_task_v1',
            'task':{**self.task,'worker_arm':'left','support_arm':'right'},
            'recipe_path':str(recipe),'recipe_sha256':hashlib.sha256(recipe.read_bytes()).hexdigest(),
            'rendered_recipe_path':str(rendered),'rendered_recipe_sha256':hashlib.sha256(rendered.read_bytes()).hexdigest(),
            'on_site_supervision':{'source':'user','statement':'Synthetic supervision'}}))
        self.passive,self.rgb = f.passive,f.rgb
        for path in [*self.passive.values(),self.rgb]:
            data = json.loads(path.read_text()); f.f.f.shift(data,3940.)
            if path in self.passive.values():
                data.update(frame_origin_counts={'local':0,'nonlocal':1000},command_feedback={},
                            frame_id_counts={'0x2A1':100,'0x2A2':100,'0x2A5':100})
                side = next(s for s,p in self.passive.items() if p==path)
                raw = self.receipt['device_receipt']['joint_path_plan']['target_raw'] if side=='left' else RAW
                for frame in data['pose_trace']:
                    frame['joints_raw'] = {'joint_'+str(i+1):q for i,q in enumerate(raw)}
            path.write_text(json.dumps(data))

    def tracking_failure(self):
        receipt = copy.deepcopy(self.f.receipt); r = receipt['device_receipt']
        old_plan = r['joint_path_plan']; identity = copy.deepcopy(old_plan['identity'])
        ctx = context(arm='left'); ctx['identity'] = identity
        ctx['origin'] = sample(4042.,identity=identity); ctx['current'] = sample(4042.01,identity=identity)
        ctx['origin_sha256'] = jp.evidence_sha256(ctx['origin'])
        ctx['cached_target']['identity'] = identity
        for frame in ctx['cached_target']['frame_receipts']: frame['returned_at'] += 3942.
        target = ctx['current']['arms']['left']['joints_rad'][:]; target[5] += .001
        ctx,target = visual_joint_context(ctx,target)
        ctx['geometry']['origin_sample_id'] = ctx['origin']['sample_id']
        plan = jp.plan_joint_path(ctx,target,now=4042.01)
        observed = sample(4046.,identity=identity)
        observed['arms']['left']['joints_rad'][5] = ctx['origin']['arms']['left']['joints_rad'][5]+.0265
        r.update(joint_path_plan=plan,target_joints_rad=target,
                 errors=[{'type':'JointPathError','detail':'joint_tracking_envelope'}],
                 rejected_joint_feedback=copy.deepcopy(observed),after=copy.deepcopy(observed['arms']))
        r['tracking_observation']['first_failure'] = dict(type='JointPathError',detail='joint_tracking_envelope',
            code='joint_tracking_envelope',sample_role='rejected_observation',sample=copy.deepcopy(observed))
        r['original_action_report']['requested_target'] = target
        r['original_event'].update(rgb_admission=plan['geometry'],target_raw=plan['target_raw'],plan_sha256=plan['plan_sha256'],
            frame_receipts=[dict(frame=frame,outcome='returned',returned_at=4042.1+i*.01)
                            for i,frame in enumerate(plan['frames'])])
        payload = {**self.f.payload,'target':target,'observation_id':ctx['geometry']['evidence']['observation_id'],
                   'unloaded_observation':ctx['geometry']['evidence']['unloaded_observation']}
        return receipt,payload

    def write_failure(self,receipt,payload=None):
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='rgb-failed'",(json.dumps(receipt),))
            if payload is not None:
                text = json.dumps(payload,sort_keys=True,separators=(',',':'))
                db.execute("UPDATE pair_events SET payload_json=?,payload_digest=? WHERE event_id='rgb-failed'",
                           (text,hashlib.sha256(text.encode()).hexdigest()))

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            return {n:[dict(r) for r in db.execute('SELECT * FROM '+n)] for n, in
                    db.execute("SELECT name FROM sqlite_master WHERE name LIKE 'pair_%'").fetchall()}

    def prepare(self,**kwargs):
        args = dict(close_log=self.close,new_run_id='tracking-new-round',started_at=7990.,max_steps=500,max_duration_s=3600.,
            budget_start_policy='after_repair_before_online_execution',parent_kind='completed_unloaded_joint_fault',
            task_file=self.task_file,clock=lambda:self.now,recovery_evidence=dict(passive_paths=self.passive,
                rgb_observation=self.rgb,visual_observation='Synthetic: empty jaws, table support, no contact.'))
        args.update(kwargs); return entry.prepare_round(self.path,'round-2',**args)

    def activate(self,proposal,**auth_changes):
        authorization = self.f.authorize(proposal); authorization.update(auth_changes)
        return entry.activate_round(proposal,authorization,project_root=self.root,clock=lambda:self.now)

    def test_full_enrollment_recognition_keeps_old_fault_and_new_roles(self):
        before = self.rows(); proposal = self.prepare(); self.assertEqual(before,self.rows())
        result = self.activate(proposal); after = self.rows()
        for name,rows in before.items(): self.assertTrue(all(row in after[name] for row in rows),name)
        self.assertEqual(after['pair_faults'],before['pair_faults'])
        self.assertEqual(result['required_connection_mode'],'prepare')
        self.assertEqual(result['new_contract']['task']['worker_arm'],'left')
        self.assertFalse(result['cache_or_limits_transferred']); self.assertEqual(result['hardware_commands_sent'],0)
        self.assertIsNone(result['physical_stop_verified'])
        self.assertTrue(activated_execution_budget(self.path,'tracking-new-round',max_steps=500,max_duration_s=3600.))
        ledger = PairLedger(self.path,'tracking-new-round',result['new_contract'],max_steps=500,
                            max_duration_s=3600,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError): ledger.claim(self.f.owner)
        current = ledger.claim('fresh-preparation-owner')
        self.assertEqual(current['deadline_s'],11590.)
        self.assertTrue(activated_execution_budget(self.path,'tracking-new-round',max_steps=500,max_duration_s=3600.))
        with self.assertRaises(PairLedgerError): self.activate(proposal)

    def test_real_pair_host_constructor_and_prepare_open_with_zero_device_sends(self):
        from robot_tools.pair_host import PairHost
        from test_pair_host import FakePairDevice
        result=self.activate(self.prepare()); before=self.rows();devices=[]
        clock=SimpleNamespace(time=lambda:self.now,sleep=lambda dt:setattr(self,'now',self.now+dt))
        class PreparationDevice(FakePairDevice):
            def connect_for_preparation(inner):
                return {**inner.open(),'task_ready':False,'readiness':{'synthetic_fixture':True}}
        def factory(*args):
            device=PreparationDevice(*args,clock);devices.append(device);return device
        profile=json.loads((self.root/'configs/robot.json').read_text());task=result['new_contract']['task']
        arguments=dict(device_factory=factory,clock=clock.time,background=False)
        with self.assertRaises(ValueError):
            PairHost(self.root/'runs',profile,'tracking-new-round',task,500,3600,connection_mode='ready',**arguments)
        self.assertEqual(devices,[])
        host=PairHost(self.root/'runs',profile,'tracking-new-round',task,500,3600,connection_mode='prepare',**arguments)
        self.addCleanup(host.close)
        self.assertEqual(host.ledger.status()['contract'],result['new_contract'])
        opened=host.open();self.assertEqual(opened['status'],'owned');self.assertFalse(opened['task_ready'])
        self.assertEqual(host.profile['_pair_task_roles'],{'worker_arm':'left','support_arm':'right'})
        self.assertEqual(devices[0].calls,[]);self.assertEqual(devices[0].frame_attempts,0)
        self.assertEqual(host.ledger.status()['steps'],0);self.assertEqual(host.deadline,11590.)
        self.assertIsNone(host.close()['physical_stop_verified'])
        self.assertEqual(before['pair_events'],self.rows()['pair_events'])
        self.assertEqual(before['pair_faults'],self.rows()['pair_faults'])

    def test_partial_unknown_loaded_and_other_failures_refuse(self):
        mutations = [lambda d:d['original_event']['frame_receipts'].pop(),
            lambda d:d['original_event']['frame_receipts'][0].update(outcome='unknown'),
            lambda d:d['transmission_counts']['right'].update(attempted_frames=1),
            lambda d:d.update(target_calls_sent=2),lambda d:d['joint_path_plan'].update(loaded_context={}),
            lambda d:d.update(hold_receipt={}),lambda d:d['errors'].append({'type':'RuntimeError','detail':'bus-off'}),
            lambda d:d['tracking_observation']['first_failure'].update(code='feedback_joint_limit')]
        for mutation in mutations:
            receipt = copy.deepcopy(self.receipt); mutation(receipt['device_receipt']); self.write_failure(receipt)
            with self.subTest(mutation=mutation),self.assertRaises((PairLedgerError,jp.JointPathError)): self.prepare()

    def test_no_device_fault_absolute_limit_or_missing_timestamp_can_be_hidden(self):
        mutations = [lambda arms:arms['left']['arm_status'].update(err_code=1),
            lambda arms:arms['right']['drivers']['2']['foc_status'].update(stall_status=True),
            lambda arms:arms['left']['joints_rad'].__setitem__(0,4.),
            lambda arms:arms['right']['fragment_timestamps_s'].pop('driver_state_1'),
            lambda arms:arms['right']['joints_rad'].__setitem__(0,.5)]
        for mutation in mutations:
            receipt = copy.deepcopy(self.receipt); d=receipt['device_receipt']
            sample=d['tracking_observation']['first_failure']['sample']; mutation(sample['arms'])
            d['rejected_joint_feedback']=copy.deepcopy(sample);d['after']=copy.deepcopy(sample['arms'])
            self.write_failure(receipt)
            with self.subTest(mutation=mutation),self.assertRaises((PairLedgerError,jp.JointPathError,KeyError)): self.prepare()

    def test_independent_feedback_must_be_new_stationary_healthy_and_near_target(self):
        path=self.passive['left']; original=path.read_text()
        mutations=[lambda d:d['pose_trace'][2]['joints_raw'].update(joint_6=14000),
            lambda d:d['pose_trace'][2]['joints_raw'].update(joint_1=180000),
            lambda d:d.update(frames_sent_by_this_script=1),
            lambda d:d['feedback']['PiperMsgLowSpdFeed_1']['fields']['foc_status'].update(driver_overcurrent=True),
            lambda d:d.update(finished_at_s=4047.)]
        for mutation in mutations:
            data=json.loads(original);mutation(data);path.write_text(json.dumps(data))
            with self.subTest(mutation=mutation),self.assertRaises(PairLedgerError): self.prepare()
        path.write_text(original);self.prepare()
        data=json.loads(original)
        for sample in data['pose_trace']:sample['joints_raw']['joint_6']=14000
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(PairLedgerError,'not settled near'):self.prepare()

    def test_actual_new_budget_and_role_recipe_required(self):
        for kwargs in ({'started_at':7599.},{'budget_start_policy':'preserve_parent_deadline'}, {'task_file':None}):
            with self.subTest(kwargs=kwargs),self.assertRaises(PairLedgerError):self.prepare(**kwargs)
        proposal=self.prepare()
        for mutation in ({'received_at':7599.},{'new_budget':{**proposal['new_budget'],'max_steps':501}}):
            with self.subTest(mutation=mutation),self.assertRaises(PairLedgerError):self.activate(proposal,**mutation)
        with self.assertRaises((KeyError,PairLedgerError)):
            entry.activate_round(proposal,{},project_root=self.root,clock=lambda:self.now)
        envelope=json.loads(self.task_file.read_text());path=Path(envelope['rendered_recipe_path'])
        body=json.loads(path.read_text());body['steps'][3]['arm']='left';path.write_text(json.dumps(body))
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_current_host_stale_evidence_changed_parent_or_task_prevent_activation(self):
        proposal=self.prepare();before=self.rows()
        with patch.object(entry,'_live_control_processes',return_value=['live']):
            with self.assertRaises(PairLedgerError):self.activate(proposal)
        self.now+=31
        with self.assertRaises(PairLedgerError):self.activate(proposal)
        self.assertEqual(before,self.rows())

    def test_parent_mutation_is_rejected_by_activation_and_budget_reader(self):
        proposal=self.prepare()
        with sqlite3.connect(self.path) as db:
            original=db.execute("SELECT reason FROM pair_faults ORDER BY id LIMIT 1").fetchone()[0]
            db.execute("UPDATE pair_faults SET reason='changed old fault' WHERE id=(SELECT MIN(id) FROM pair_faults)")
        with self.assertRaises(PairLedgerError):self.activate(proposal)
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_faults SET reason=? WHERE id=(SELECT MIN(id) FROM pair_faults)',(original,))
        self.activate(proposal)
        receipt=copy.deepcopy(self.receipt);receipt['device_receipt']['errors'][0]['detail']='bus-off'
        self.write_failure(receipt)
        with self.assertRaises(PairLedgerError):
            activated_execution_budget(self.path,'tracking-new-round',max_steps=500,max_duration_s=3600.)


if __name__ == '__main__': unittest.main()
