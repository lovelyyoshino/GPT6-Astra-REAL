"""Explicit post-send RGB-expiry new rounds: synthetic SQLite, no hardware."""
import copy
import json
import math
import sqlite3
import unittest
from unittest.mock import patch

from robot_tools import pair_round
from robot_tools.pair_continuation import _observations
from robot_tools.joint_path import evidence_sha256
from robot_tools.pair_ledger import PairLedger, PairLedgerError, _hold_frames, activated_execution_budget
import test_pair_round as fixtures


class RGBExpiryRoundTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.PairRoundTests('runTest'); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.root, self.path = self.f.root, self.f.path
        activation = self.f.activate(); self.now = 4020.
        self.ledger = PairLedger(self.path,'round-2',activation['new_contract'],max_steps=500,
                                max_duration_s=3600,clock=lambda:self.now)
        self.owner = 'rgb-owner'; self.ledger.claim(self.owner)
        old = {e['event_id']:e for e in self.f.rows()['pair_events']}
        self.totals = {s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in ('left','right')}
        for index, source in enumerate(('query-healthy','init-healthy-left','init-healthy-right')):
            e = old[source]; p = json.loads(e['payload_json']); r = json.loads(e['receipt_json'])
            self.now = (4020.,4033.,4035.)[index]
            self.f.f.shift(r,self.now-e['began_at']); event = 'rgb-'+source
            if index == 0:
                self.bindings = p['bindings']; r.update(event_id=event,pair_owner=self.owner)
            else:
                r['initialization_plan']['identity'].update(run_id='round-2',owner=self.owner,
                    epoch=self.owner,worker_id=event)
            self.totals = copy.deepcopy(r['session_transmission_counts'])
            self.ledger.begin(self.owner,event,p); self.now += 12. if index == 0 else 1.
            self.ledger.finish(self.owner,event,r)
        self.raw = [100,0,0,200,300,0]; self.target = [math.radians(v/1000) for v in self.raw]
        self.now = 4040.; p,r = self.action('rgb-success',failed=False)
        self.ledger.begin(self.owner,'rgb-success',p); self.now = 4041.; self.ledger.finish(self.owner,'rgb-success',r)
        self.now = 4042.; self.payload,self.receipt = self.action('rgb-failed',failed=True)
        self.ledger.begin(self.owner,'rgb-failed',self.payload); self.now = 4047.
        self.ledger.fault(self.owner,'Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch')
        self.ledger.finish(self.owner,'rgb-failed',self.receipt,success=False)
        self.close = self.root/'rgb-close.json'; closed = json.loads(self.f.close.read_text())
        closed['fault_latched'] = True; closed['cleanup']['session_transmission_counts'] = self.totals
        self.close.write_text(json.dumps(closed))
        self.passive, self.rgb = self.f.f.passive, self.f.f.rgb
        delta = 4050.-min(json.loads(path.read_text())['started_at_s'] for path in self.passive.values())
        for path in [*self.passive.values(), self.rgb]:
            data = json.loads(path.read_text()); self.f.f.shift(data,delta); path.write_text(json.dumps(data))
        for name in ('pair_host.py','pair_joint_adapter.py'):
            path = self.root/'robot_tools'/name; path.write_text(path.read_text()+'\n# repaired RGB reserve\n')
        self.now = 8000.

    def action(self,event,failed):
        arm = 'left'; identity = dict(run_id='round-2',owner=self.owner,epoch=self.owner,
            worker_id=event,arm=arm,**{k:self.bindings[arm][k] for k in ('connection_id','model','firmware_profile')})
        payload = dict(kind='joint',arm=arm,operation='approach',admission_mode='rgb_supervised',
            target=self.target,observation_id='rgb-scene-'+event,unloaded_observation='双爪空载，未接触。')
        evidence = dict(identity=identity,operation='approach',observation_id=payload['observation_id'],
            target_raw=self.raw,unloaded_observation=payload['unloaded_observation'],rgb_received_at=4016.)
        geometry = dict(schema='piper_rgb_supervised_coarse_approach_v1',evidence=evidence,
                        source=dict(ref='synthetic_rgb',sha256=evidence_sha256(evidence)))
        plan = dict(identity=identity,loaded_context=None,loaded_observation_only=False,recovery_mode=None,
            hold_supported=False,hold_policy='latch_only',spatial_admission_mode='rgb_supervised',geometry=geometry,
            requested_target_joints_rad=self.target,target_raw=self.raw,frames=_hold_frames(self.raw),visual_rgb_deadline=4046.)
        plan['plan_sha256'] = evidence_sha256(plan)
        original = dict(schema='piper_rgb_supervised_joint_send_v1',send_state='all_frames_returned',fault=None,
            event_id=event,identity=identity,operation='approach',rgb_admission=geometry,target_raw=self.raw,
            plan_sha256=plan['plan_sha256'],spatial_admission_mode='rgb_supervised',hold_policy='latch_only',
            hold_supported=False,deadline_at=7600.,frame_receipts=[dict(frame=f,outcome='returned',returned_at=self.now+.1+i*.01)
                for i,f in enumerate(_hold_frames(self.raw))])
        counts = {s:dict(attempted_frames=4 if s==arm else 0,sent_frames=4 if s==arm else 0,blocked_frames=0)
                  for s in self.totals}
        for key in ('attempted_frames','sent_frames'):self.totals[arm][key] += 4
        error = dict(type='RuntimeError',detail='visual_rgb_expired: original joint RGB deadline reached')
        r = dict(ok=not failed,kind='joint',status='pair_device_fault' if failed else 'completed',
            automatic_retry=False,errors=[error] if failed else [],guard_violations=[],hold_receipt=None,
            hold_supported=False,hold_policy='latch_only',explicit_cancel_hold_bridge_bound=False,
            motion_gate_unlocked=False,joint_limits_changed=False,arrival_confirmed=not failed,
            hardware_commands_sent=4,target_calls_sent=1,target_commands_sent=0,enable_commands_sent=0,
            stop_commands_sent=0,retries=0,passive_arm_commands_sent=0,transmission_counts=counts,
            session_transmission_counts=copy.deepcopy(self.totals),joint_path_plan=plan,original_event=original,
            spatial_admission_mode='rgb_supervised',target_joints_rad=self.target,
            original_action_report=dict(errors=[],automatic_retry=False,target_calls_sent=1,
                requested_target=self.target,target_commands_sent=0,enable_commands_sent=0,
                stop_commands_sent=0,retries=0,passive_arm_commands_sent=0),
            tracking_observation=dict(first_failure=dict(**error,sample_role='latest_observation_before_failure',
                sample=dict(identity=identity,captured_at=4045.9))))
        if failed:r = dict(ok=False,automatic_retry=False,event_id=event,
            error='Stable feedback alone is not an arrived single dispatch',device_receipt=r)
        return payload,r

    def prepare(self,**kwargs):
        args = dict(close_log=self.close,new_run_id='rgb-new-round',started_at=7990.,max_steps=500,max_duration_s=3600.,
            budget_start_policy='after_repair_before_online_execution',parent_kind='postsend_rgb_expiry_fault',
            clock=lambda:self.now,recovery_evidence=dict(passive_paths=self.passive,rgb_observation=self.rgb,
                visual_observation='Synthetic empty jaws, no contact, object independently supported.'))
        args.update(kwargs); return pair_round.prepare_round(self.path,'round-2',**args)

    def authorize(self,p):
        return dict(source='user_message',message_id='new-rgb-goal',statement='Fix and start a new timed task round',
            received_at=7980.,decision='authorize_explicit_new_round',proposal_sha256=p['proposal_sha256'],
            new_budget=p['new_budget'],budget_start_policy=p['budget_start_policy'])

    def activate(self,p,authorization=None):
        return pair_round.activate_round(p,authorization or self.authorize(p),project_root=self.root,clock=lambda:self.now)

    def write_receipt(self,value,event='rgb-failed'):
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_events SET receipt_json=? WHERE event_id=?',(json.dumps(value),event))

    def test_explicit_new_budget_preserves_history_faults_and_retires_owners(self):
        before = self.f.rows(); p = self.prepare(); result = self.activate(p); after = self.f.rows()
        for name,rows in before.items():
            self.assertEqual(rows,[r for r in after[name] if not (name in ('pair_runs','pair_rounds')
                and r['run_id']=='rgb-new-round')],name)
        self.assertTrue(result['new_budget_allocated']); self.assertEqual(result['hardware_commands_sent'],0)
        self.assertFalse(result['cache_or_limits_transferred']); self.assertIsNone(result['physical_stop_verified'])
        self.assertTrue(activated_execution_budget(self.path,'rgb-new-round',max_steps=500,max_duration_s=3600.))
        new = PairLedger(self.path,'rgb-new-round',result['new_contract'],max_steps=500,max_duration_s=3600.,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):new.claim(self.owner)
        status = new.claim('new-rgb-owner')
        self.assertEqual(status['deadline_s'],11590.); self.assertEqual(status['steps'],0)
        self.assertEqual(status['execution_lineage']['cumulative_steps'],p['snapshot']['cumulative_prior_steps'])
        with self.assertRaises(PairLedgerError):self.ledger.begin(self.owner,'replay',self.payload)
        with self.assertRaises(PairLedgerError):self.activate(p)

    def test_explicit_three_hour_thousand_step_budget_and_upper_bounds(self):
        for limits in (dict(max_steps=1001),dict(max_duration_s=10800.1)):
            with self.subTest(limits=limits),self.assertRaises(PairLedgerError):self.prepare(**limits)
        p = self.prepare(max_steps=1000,max_duration_s=10800.); self.activate(p)
        self.assertTrue(activated_execution_budget(self.path,'rgb-new-round',max_steps=1000,max_duration_s=10800.))

    def test_no_automatic_extension_or_old_authorization(self):
        for extra in (dict(budget_start_policy='preserve_parent_deadline'),dict(budget_start_policy='include_repair_time'),
                      dict(started_at=7599.)):
            with self.subTest(extra=extra),self.assertRaises(PairLedgerError):self.prepare(**extra)
        p = self.prepare()
        for change in (dict(received_at=7599.),dict(decision='authorize_repaired_continuation'),
                       dict(new_budget={**p['new_budget'],'max_steps':501})):
            auth = self.authorize(p); auth.update(change)
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.activate(p,auth)

    def test_partial_wrong_unknown_or_extra_sends_and_other_faults_refuse(self):
        mutations = [lambda d:d['original_event']['frame_receipts'][2].update(outcome='unknown'),
            lambda d:d['original_event']['frame_receipts'].pop(),
            lambda d:d['original_event'].update(send_state='partial'),
            lambda d:d['transmission_counts']['right'].update(attempted_frames=1,sent_frames=1),
            lambda d:d['session_transmission_counts']['left'].update(attempted_frames=19,sent_frames=19),
            lambda d:d.update(target_calls_sent=2),lambda d:d.update(hold_receipt={}),
            lambda d:d.update(arrival_confirmed=True),lambda d:d['errors'].append(dict(type='RuntimeError',detail='other')),
            lambda d:d['original_action_report']['errors'].append(dict(type='RuntimeError',detail='other')),
            lambda d:d['joint_path_plan'].update(loaded_context={})]
        for mutate in mutations:
            receipt = copy.deepcopy(self.receipt); mutate(receipt['device_receipt']); self.write_receipt(receipt)
            with self.subTest(mutate=mutate),self.assertRaises(PairLedgerError):self.prepare()
        self.write_receipt(self.receipt); self.ledger.fault(self.owner,'unrelated')
        with self.assertRaises(PairLedgerError):self.prepare()

    def test_identity_plan_target_digest_and_chronology_refuse(self):
        mutations = [lambda d:d['original_event'].update(event_id='foreign'),
            lambda d:d['original_event']['identity'].update(connection_id='other'),
            lambda d:d['joint_path_plan'].update(plan_sha256='0'*64),
            lambda d:d['original_event']['target_raw'].__setitem__(0,200),
            lambda d:d['original_event']['frame_receipts'][3].update(returned_at=4046.1),
            lambda d:d['joint_path_plan'].update(visual_rgb_deadline=9000.),
            lambda d:d['tracking_observation']['first_failure']['sample'].update(captured_at=4040.)]
        for mutate in mutations:
            receipt = copy.deepcopy(self.receipt); mutate(receipt['device_receipt']); self.write_receipt(receipt)
            with self.subTest(mutate=mutate),self.assertRaises(PairLedgerError):self.prepare()

    def test_changed_recovery_code_close_history_or_live_owner_refuse(self):
        p = self.prepare(); before = self.f.rows()
        with patch('robot_tools.pair_round._live_control_processes',return_value=['active']):
            with self.assertRaises(PairLedgerError):self.activate(p)
        path = self.root/'robot_tools/pair_host.py'; old = path.read_text(); path.write_text(old+'# changed\n')
        with self.assertRaises(PairLedgerError):self.activate(p)
        path.write_text(old); raw = self.close.read_text(); closed = json.loads(raw); closed['fault_latched']=False
        self.close.write_text(json.dumps(closed))
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.close.write_text(raw); data = json.loads(self.rgb.read_text()); data['cameras']['front']['host_received_at']=4040.
        self.rgb.write_text(json.dumps(data))
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.assertEqual(before,self.f.rows())

    def observation(self,metric='euler_span_bound'):
        contract = json.loads(next(row['contract_json'] for row in self.f.rows()['pair_runs'] if row['run_id']=='round-2'))
        return _observations(passive_paths=self.passive,rgb_observation=self.rgb,
            visual_observation='Synthetic empty jaws, no contact.',contract=contract,
            failed_at=4047.,now=self.now,orientation_metric=metric)

    def change_trace_pose(self,poses):
        path = self.passive['left']; data = json.loads(path.read_text())
        for row in data['pose_trace']:
            row['end_pose_raw'].update(RX_axis=0,RY_axis=0,RZ_axis=0)
        for index,angles in poses:
            data['pose_trace'][index]['end_pose_raw'].update(zip(('RX_axis','RY_axis','RZ_axis'),angles))
        path.write_text(json.dumps(data))

    def test_so3_explicit_only_preserves_default_and_audits_both_metrics(self):
        original = self.observation()
        self.assertNotIn('orientation_observation',original['passive']['left'])
        self.change_trace_pose([(10,(61,61,61))])
        with self.assertRaises(PairLedgerError):self.observation()
        result = self.observation('so3_diameter')
        angles = result['passive']['left']['orientation_observation']
        self.assertGreater(angles['euler_span_sum_bound_rad'],.003)
        self.assertLess(angles['so3_diameter_rad'],.003)
        self.assertEqual(angles['method'],'pairwise_so3_diameter')
        self.assertEqual(angles['overall_limit_rad'],.003)
        # The new branch selects SO(3) itself; its caller cannot override it.
        p = self.prepare(); self.activate(p)
        self.assertEqual(p['recovery_evidence']['passive']['left']['orientation_observation'],angles)

    def test_so3_covers_interior_pairwise_diameter_not_only_endpoints(self):
        self.change_trace_pose([(8,(-85,-85,-85)),(12,(85,85,85))])
        # Endpoints coincide and each interior sample is within .003 rad of
        # them; the two interior samples are farther apart. Every Euler-axis
        # span also remains < .003, so the pairwise angle must reject this.
        with self.assertRaisesRegex(PairLedgerError,'whole-position/rotation'):
            self.observation('so3_diameter')

    def test_so3_retains_axis_joint_translation_and_wrap_constraints(self):
        for angles in ((180,0,0),(179999,0,0)):
            self.change_trace_pose([(10,angles)])
            with self.subTest(angles=angles),self.assertRaisesRegex(PairLedgerError,'not stationary'):
                self.observation('so3_diameter')
        self.change_trace_pose([(8,(179999,0,0)),(12,(-179999,0,0))])
        with self.assertRaisesRegex(PairLedgerError,'not stationary'):self.observation('so3_diameter')
        self.change_trace_pose([])
        path = self.passive['left']; base = json.loads(path.read_text())
        for group,key,delta in (('joints_raw','joint_5',180),('end_pose_raw','X_axis',501)):
            data = copy.deepcopy(base); data['pose_trace'][10][group][key] += delta
            path.write_text(json.dumps(data))
            with self.subTest(key=key),self.assertRaisesRegex(PairLedgerError,'not stationary'):
                self.observation('so3_diameter')
        path.write_text(json.dumps(base))
        with self.assertRaises(PairLedgerError):self.observation('unknown')


if __name__ == '__main__':unittest.main()
