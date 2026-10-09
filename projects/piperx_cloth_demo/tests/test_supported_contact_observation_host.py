"""Typed existing-target observations through the native host/device on FakeCAN.

Archived enrollment and onsite contact are synthetic fixtures. No hardware,
canonical ledger writes, historical success reclassification or force claim.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import host_recovery, supported_gripper_recovery as recovery
from robot_tools.grasp_episode import GraspEpisodeError, apply_event
from robot_tools.retention_receipt import digest, measured_anchor
from test_host_grasp_integration import HostGraspFixture
import test_grasp_episode as pure


class SupportedContactObservationHostTests(HostGraspFixture):
    def setUp(self):
        super().setUp()
        before=copy.deepcopy(self.host.device._previous)
        anchor=measured_anchor(before['left']);anchor['width_m']=.046
        self.source=dict(before=before,candidate_measurement=dict(anchor=anchor),
            candidate_probe=dict(completed_at=self.clock.time()-40))
        self.source['audited_existing_contact']=dict(
            schema='piper_existing_supported_contact_admission_v1',arm='left',
            source_receipt_sha256=digest(self.source),failed_event_id='old-failed-probe',
            failed_receipt_sha256='a'*64,prior_sent_target_m=.0455,
            failed_sent_at=self.clock.time()-20,failed_finished_at=self.clock.time()-15,
            audit_proposal_sha256='b'*64,consumed_probe_count=3,remaining_probe_count=0,
            residual_jaw_anchor=dict(width_m=.05,observed_at=self.clock.time()-1,
                source=dict(path='/synthetic/residual.json',sha256='c'*64)),
            bilateral_contact_source=dict(path='/synthetic/user-contact.json',sha256='d'*64))
        self.old=dict(arm='left',grasp_object_id='charger')
        self.proposal=dict(schema=recovery.OBSERVATION_SCHEMA,
            route='audited_supported_contact_observation',proposal_sha256='b'*64,
            snapshot=dict(event=dict(event_id='old-failed-probe')),budget=dict(steps=0),
            max_observations=1,max_probes=0,consumed_probe_count=3,stage_attempt_limit=3,
            existing_contact={k:copy.deepcopy(v) for k,v in self.source['audited_existing_contact'].items()
                              if k!='audit_proposal_sha256'})
        self.events=[]
        self.stack.enter_context(patch.object(recovery,'current_contact_source',
            return_value=(self.old,self.source),create=True))
        self.stack.enter_context(patch.object(host_recovery.SupportedRecovery,'state',lambda _:self.current()))

    def current(self):
        with sqlite3.connect(self.host.runs/'pair_sessions.sqlite') as db:
            db.row_factory=sqlite3.Row
            events=list(db.execute('SELECT * FROM pair_events WHERE run_id=? ORDER BY step',(self.host.run_id,)))
            return recovery._existing_contact_runtime(dict(owner=self.host.owner),self.proposal,events)

    def request(self,event_id='observe-existing',**changes):
        scene=self.observe()
        request=dict(event_id=event_id,observation_id=scene['observation_id'],object_id='charger',
            visual_description='Synthetic current bilateral finger contact, charger independently supported by socket.',
            contact_relation='bilateral_finger_contact',support_relation='independent_support_present')
        request.update(changes);return request

    def observed(self,event_id='observe-existing'):
        request=self.request(event_id);self.events.append(event_id)
        self.service.call('robot_pair_observe_supported_contact',request)
        result=self.host.wait(event_id,10)
        self.assertEqual(result['status'],'completed',result.get('receipt'))
        return request,result

    def zero(self):
        for robot in self.robots.values():self.assertEqual(robot.sent,[])

    def test_typed_current_candidate_and_static_retention_send_nothing(self):
        frozen=copy.deepcopy(self.source)
        request,result=self.observed()
        receipt=result['receipt'];state=self.host.grasps.active('left')
        self.assertEqual(receipt['hardware_commands_sent'],0)
        self.assertNotIn('candidate_probe',receipt)
        self.assertEqual(state['status'],'contact_candidate')
        self.assertEqual(state['candidate_basis'],'existing_target_observation')
        self.assertEqual(state['visual_evidence']['source'],'rgb_and_user_contact_observation')
        self.assertEqual(state['visual_evidence']['bilateral_contact_source'],
                         self.source['audited_existing_contact']['bilateral_contact_source'])
        self.assertEqual(state['probe_ref']['event_id'],request['event_id'])
        self.assertEqual(state['probe_ref']['existing_target_ref']['event_id'],'old-failed-probe')
        self.assertEqual(state['residual_target'],dict(event_id='old-failed-probe',requested_width_m=.0455))
        self.assertNotIn('conservative_closure_displacement_m',state['probe_ref'])
        self.assertNotIn('sent_at',state['probe_ref'])
        self.assertNotIn('outcome',state['probe_ref'])
        self.assertEqual(state['original_anchor']['joints_rad'],self.source['candidate_measurement']['anchor']['joints_rad'])
        retained,_=self.retain('left')
        self.assertEqual(retained['episode']['status'],'retained_static')
        self.assertEqual(retained['hardware_commands_sent'],0)
        self.assertFalse(retained['loaded_contact_available'])
        self.assertEqual(self.source,frozen)
        self.zero()

    def test_completed_event_replay_does_not_observe_again_or_consume_step(self):
        request,_=self.observed();revision=self.host.grasps.active('left')['revision']
        result=self.service.call('robot_pair_observe_supported_contact',request)
        self.assertTrue(result['replayed']);self.assertEqual(self.host.ledger.status()['steps'],1)
        self.assertEqual(self.host.grasps.active('left')['revision'],revision)
        with self.assertRaises(RuntimeError):
            self.service.call('robot_pair_observe_supported_contact',self.request('new-observation'))
        self.zero()

    def test_wrong_object_semantics_or_missing_scope_cannot_claim(self):
        for changes in (dict(object_id='other'),dict(contact_relation='unknown'),
                        dict(support_relation='unknown'),dict(visual_description='')):
            with self.subTest(changes=changes),self.assertRaises((ValueError,RuntimeError)):
                self.service.call('robot_pair_observe_supported_contact',self.request(**changes))
        self.proposal['route']='audited_contact_reacquisition'
        with self.assertRaises(RuntimeError):
            self.service.call('robot_pair_observe_supported_contact',self.request())
        self.assertEqual(self.host.ledger.status()['steps'],0);self.zero()

    def test_stale_rgb_refresh_is_before_claim_and_episode(self):
        request=self.request();self.clock.sleep(27)
        result=self.service.call('robot_pair_observe_supported_contact',request)
        self.assertEqual(result['status'],'refresh_required')
        self.assertIsNone(self.host.grasps.active('left'))
        self.assertIsNone(self.host.ledger.event(request['event_id']))
        self.assertFalse(self.host.status()['fault_latched'])
        self.observed('fresh-observation');self.zero()

    def test_observation_and_retention_do_not_unlock_any_target_or_query(self):
        self.observed();self.retain('left')
        with self.assertRaises(RuntimeError):self.host.inspect_joint_limits('query')
        for arm,kind,op,target in (('left','gripper','grip_supported',.045),
                ('right','gripper','grip_supported',.045),('left','joint','extract_segment',[0.]*6)):
            scene=self.observe()
            request=dict(event_id='forbidden-'+arm+'-'+kind,observation_id=scene['observation_id'],
                peer_receipt_id=scene['peer_receipts']['right' if arm=='left' else 'left']['receipt_id'],
                arm=arm,kind=kind,operation=op)
            request['width_m' if kind=='gripper' else 'target_joints_rad']=target
            with self.subTest(arm=arm,kind=kind),self.assertRaises((ValueError,RuntimeError)):
                self.service.call('robot_pair_submit_once',request)
        self.assertEqual(self.host.ledger.status()['steps'],1);self.zero()

    def test_no_persistent_target_gap_faults_and_never_creates_candidate(self):
        self.source['audited_existing_contact']['prior_sent_target_m']=.05
        request=self.request();self.events.append(request['event_id'])
        self.service.call('robot_pair_observe_supported_contact',request)
        result=self.host.wait(request['event_id'],10)
        self.assertEqual(result['status'],'fault')
        self.assertEqual(self.host.grasps.active('left')['status'],'empty')
        with self.assertRaises((ValueError,RuntimeError)):
            self.service.call('robot_pair_observe_supported_contact',self.request('retry'))
        self.zero()

    def test_switched_user_contact_source_is_not_relabelled_as_rgb_evidence(self):
        original=self.host.device.observe_supported_contact
        def substituted(*args,**kwargs):
            result=original(*args,**kwargs)
            result['audited_existing_contact']['bilateral_contact_source']['sha256']='f'*64
            return result
        with patch.object(self.host.device,'observe_supported_contact',side_effect=substituted):
            request=self.request();self.events.append(request['event_id'])
            self.service.call('robot_pair_observe_supported_contact',request)
            result=self.host.wait(request['event_id'],10)
        self.assertEqual(result['status'],'fault')
        self.assertEqual(self.host.grasps.active('left')['status'],'empty')
        self.zero()


class ExistingContactEpisodeTests(unittest.TestCase):
    def evidence(self):
        state=pure.episode();measurement=pure.measurement(state)
        candidate=dict(schema='piper_existing_supported_contact_candidate_v1',basis='existing_target_observation',
            identity=state['identity'],event_id='probe-left',observation_id='scene-1',requested_width_m=.035,
            existing_target_ref=dict(event_id='old-failed',receipt_sha256='a'*64,sent_at=90.,finished_at=91.,requested_width_m=.035),
            trace_sha256=measurement['trace_sha256'],started_at=101.,completed_at=104.,observed_width_m=.039,
            minimum_target_gap_m=.004,completion='observation_only',target_may_remain_active=True)
        scene=pure.scene(state,at=100.5,frame=1)
        visual=pure.release_visual(state,scene);visual['object_relation']='bilateral_finger_contact'
        visual.update(source='rgb_and_user_contact_observation',
                      bilateral_contact_source=dict(path='/synthetic/user-contact.json',sha256='d'*64))
        return state,pure.event(state,'new-candidate','record_observed_candidate',
            dict(candidate=candidate,measurement=measurement,scene=scene,visual=visual))

    def test_new_observation_without_resolved_closure_is_distinct_and_retainable(self):
        state,event=self.evidence();result=apply_event(state,event,now=104.)
        self.assertEqual(result['status'],'contact_candidate')
        self.assertEqual(result['candidate_basis'],'existing_target_observation')
        retained=apply_event(result,pure.retention_event(result),now=108.)
        self.assertEqual(retained['status'],'retained_static')
        old=pure.candidate_event(state);old['evidence']['probe']['conservative_closure_displacement_m']=.00014
        with self.assertRaises(GraspEpisodeError):apply_event(state,old,now=104.)

    def test_new_basis_cannot_forge_send_closure_time_gap_or_rgb(self):
        state,event=self.evidence()
        variants=[]
        for key,value in (('sent_at',100.5),('conservative_closure_displacement_m',.001),
                ('basis','settled_contact_candidate'),('started_at',90.),('minimum_target_gap_m',.001)):
            changed=copy.deepcopy(event);changed['evidence']['candidate'][key]=value;variants.append(changed)
        changed=copy.deepcopy(event);changed['evidence']['measurement']['trace_sha256']='b'*64;variants.append(changed)
        changed=copy.deepcopy(event);changed['evidence']['visual']['object_relation']='between_fingers';variants.append(changed)
        changed=copy.deepcopy(event);changed['evidence']['scene']['captured_at']=90.;variants.append(changed)
        changed=copy.deepcopy(event);del changed['evidence']['visual']['bilateral_contact_source'];variants.append(changed)
        changed=copy.deepcopy(event);changed['evidence']['visual']['source']='rgb_semantic_observation';variants.append(changed)
        for changed in variants:
            with self.subTest(evidence=changed),self.assertRaises(GraspEpisodeError):apply_event(state,changed,now=104.)
        self.assertEqual(state['status'],'empty')


@unittest.skipUnless(os.environ.get('PIPER_RUN_ARCHIVED_CONTACT_TESTS')=='1',
                     'Explicit opt-in for archived read-only ledger backup')
class ExistingContactArchivedHostTests(unittest.TestCase):
    def test_actual_enrollment_prepare_open_and_native_source_schema(self):
        from robot_tools.pair_host import PairHost
        from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
        from robot_tools.supported_contact_measurement import validate_source
        from test_pair_host import FakePairDevice
        location=Path('artifacts/existing_supported_contact_20261009/offline_fixture.py')
        spec=importlib.util.spec_from_file_location('existing_contact_archived_fixture',location)
        fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
        with fixture.copied_fixture() as f:
            self.assertNotEqual(f['path'].resolve(),Path('projects/piperx_cloth_demo/runs/pair_sessions.sqlite').resolve())
            p=f['proposal'];b=p['budget']
            activated=recovery.activate_existing_contact_observation(p,project_root=f['root'],clock=lambda:f['now'])
            self.assertFalse(activated['new_budget_allocated'])
            self.assertTrue(activated_execution_budget(f['path'],f['run'],max_steps=b['max_steps'],max_duration_s=b['max_duration_s']))
            ledger=PairLedger(f['path'],f['run'],activated['new_contract'],max_steps=b['max_steps'],
                             max_duration_s=b['max_duration_s'],clock=lambda:f['now'])
            with self.assertRaises(PairLedgerError):ledger.claim(p['snapshot']['scope']['owner'])
            ticks=[f['now']];clock=SimpleNamespace(time=lambda:ticks[0],sleep=lambda dt:ticks.__setitem__(0,ticks[0]+dt))
            devices=[]
            class Prepared(FakePairDevice):
                def connect_for_preparation(inner):
                    return {**inner.open(),'task_ready':False,'readiness':{'synthetic_fixture':True}}
            def factory(*args):
                device=Prepared(*args,clock);devices.append(device);return device
            profile=json.loads((f['root']/'configs/robot.json').read_text())
            args=dict(device_factory=factory,clock=clock.time,background=False)
            with self.assertRaises(ValueError):
                PairHost(f['root']/'runs',profile,f['run'],activated['new_contract']['task'],b['max_steps'],b['max_duration_s'],
                         connection_mode='ready',**args)
            self.assertEqual(devices,[])
            host=PairHost(f['root']/'runs',profile,f['run'],activated['new_contract']['task'],b['max_steps'],b['max_duration_s'],
                          connection_mode='prepare',**args)
            try:
                opened=host.open();self.assertTrue(opened['open']);self.assertFalse(opened['task_ready'])
                self.assertEqual(host.deadline,b['deadline_s']);self.assertEqual(host.ledger.status()['steps'],2)
                with sqlite3.connect(f['path']) as db:
                    db.row_factory=sqlite3.Row;state=recovery.runtime(db,f['run'],host.owner)
                self.assertEqual(state['phase'],'contact_observation_required')
                self.assertEqual(state['proposal']['max_probes'],0)
                old,source=recovery.current_contact_source(state)
                admission,anchors=validate_source(source,'left',clock.time())
                self.assertEqual(admission['prior_sent_target_m'],.0295)
                self.assertEqual(admission['residual_jaw_anchor']['width_m'],.03325)
                self.assertEqual(admission['bilateral_contact_source'],p['bilateral_contact']['source'])
                self.assertEqual(anchors['left']['joints_rad'],source['candidate_measurement']['anchor']['joints_rad'])
                with self.assertRaises(RuntimeError):host.inspect_joint_limits('forbidden')
                self.assertEqual(devices[0].calls,[]);self.assertEqual(devices[0].frame_attempts,0)
            finally:host.close()
