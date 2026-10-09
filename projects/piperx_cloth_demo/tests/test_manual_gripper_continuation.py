"""Synthetic failure classification and authorization checks, zero physical I/O."""
import copy
import hashlib
import json
import unittest
from unittest.mock import patch
from robot_tools import manual_gripper_continuation as entry
from robot_tools.contact_receipt import classify_gripper_probe
from robot_tools.pair_ledger import PairLedgerError
from test_contact_receipt import sample


class ManualGripperContinuationTests(unittest.TestCase):
    def setUp(self):
        p=patch('socket.socket',side_effect=AssertionError('Hardware forbidden'));p.start();self.addCleanup(p.stop)
        self.payload=dict(arm='right',kind='gripper',operation='grip_supported',target=.025,grasp_object_id='supported_object',observation_id='scene',peer_receipt_id='peer')
        c=classify_gripper_probe(arm='right',requested_width_m=.025,sent_at=103.02,
            baseline_samples=[sample(100+i*.05) for i in range(61)],
            post_samples=[sample(103.07+i*.05,.030,force=.2) for i in range(81)])
        d=dict(transmission_counts={s:dict(attempted_frames=int(s=='right'),sent_frames=int(s=='right'),blocked_frames=0) for s in ('left','right')},
            hardware_commands_sent=1,target_calls_sent=1,nominal_force_N=.2,enable_commands_sent=0,stop_commands_sent=0,retries=0,passive_arm_commands_sent=0,
            guard_violations=[],ok=False,errors=[dict(type='RuntimeError',detail='Probe response unconfirmed: '+str([entry.REASON]))],
            contact_observation=c,unresolved_gripper_probe=None,grasp_states=dict.fromkeys(('left','right')))
        self.receipt=dict(device_receipt=d,event_id='probe',ok=False,automatic_retry=False)
        self.event=dict(event_id='probe',run_id='run',owner='old-owner',step=9,status='complete',success=0)
        self.run=dict(run_id='run',steps=9)
        self.refresh()

    def refresh(self):
        self.event.update(payload_json=json.dumps(self.payload),receipt_json=json.dumps(self.receipt))
        self.event['payload_digest']=hashlib.sha256(self.event['payload_json'].encode()).hexdigest()

    def validate(self):
        self.refresh();return entry.validate_failed_probe(self.event,self.run,'old-owner')

    def test_complete_unconfirmed_probe_is_history_only_not_grasp(self):
        p,d=self.validate();self.assertEqual(d['contact_observation']['outcome'],'unconfirmed')
        self.assertFalse(d['contact_observation']['grasp_verified']);self.assertEqual(p['target'],.025)

    def test_partial_unknown_or_duplicate_send_refuses(self):
        original=copy.deepcopy(self.receipt)
        for k,v in [('hardware_commands_sent',0),('hardware_commands_sent',2),('target_calls_sent',2),('retries',1)]:
            self.receipt=copy.deepcopy(original);self.receipt['device_receipt'][k]=v
            with self.subTest(k=k,v=v),self.assertRaises(PairLedgerError):self.validate()
        self.receipt=copy.deepcopy(original);self.receipt['device_receipt']['transmission_counts']['right']['sent_frames']=0
        with self.assertRaises(PairLedgerError):self.validate()

    def test_other_fault_force_or_retained_contact_refuses(self):
        original=copy.deepcopy(self.receipt)
        for k,v in [('nominal_force_N',1.),('errors',[{'type':'RuntimeError','detail':'driver overcurrent'}]),('unresolved_gripper_probe',{'arm':'right'}),('grasp_states',{'right':{'status':'retained_static'},'left':None})]:
            self.receipt=copy.deepcopy(original);self.receipt['device_receipt'][k]=v
            with self.subTest(k=k),self.assertRaises(PairLedgerError):self.validate()

    def test_arrival_or_contact_success_cannot_be_relabelled_failure(self):
        for outcome in ('target_arrived','settled_contact_candidate'):
            self.receipt['device_receipt']['contact_observation']['outcome']=outcome
            with self.subTest(outcome=outcome),self.assertRaises(PairLedgerError):self.validate()

    def test_changed_original_payload_digest_refuses(self):
        self.event['payload_digest']='0'*64
        with self.assertRaises(PairLedgerError):entry.validate_failed_probe(self.event,self.run,'old-owner')

    def test_explicit_new_instruction_after_closed_session_required(self):
        a=dict(source='user_message',message_id='new-user-message',statement='I repositioned it; try tightening again.',recorded_at=110.,decision='authorize_repositioned_gripper_attempt')
        h=dict(ended_at=109.)
        self.assertEqual(entry.authorization(a,{},h,111.),a)
        for k,v in [('source','model'),('decision','automatic_retry'),('recorded_at',108.),('recorded_at',112.),('statement','')]:
            bad={**a,k:v}
            with self.subTest(k=k,v=v),self.assertRaises(PairLedgerError):entry.authorization(bad,{},h,111.)

    def test_old_owner_other_run_wrong_phase_refuse(self):
        for k,v in [('operation','release_retreat'),('kind','joint'),('arm','other')]:
            original=copy.deepcopy(self.payload);self.payload[k]=v
            with self.subTest(k=k),self.assertRaises(PairLedgerError):self.validate()
            self.payload=original
        self.refresh()
        with self.assertRaises(PairLedgerError):entry.validate_failed_probe(self.event,self.run,'new-owner')

if __name__=='__main__':unittest.main()
