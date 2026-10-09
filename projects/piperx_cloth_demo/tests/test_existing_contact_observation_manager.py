"""Fast isolated zero-TX manager gates; archived lineage is tested separately on copies."""
import copy,json,unittest
from unittest.mock import patch
from robot_tools import supported_gripper_recovery as recovery
from robot_tools.pair_ledger import PairLedgerError

class ExistingContactManagerTests(unittest.TestCase):
    def setUp(self):
        self.anchor=dict(joints_rad=[0.]*6,pose_m_rad=[0.]*6,width_m=.03325)
        self.admission=dict(failed_event_id='old-failed',failed_receipt_sha256='a'*64,
            failed_sent_at=1.,failed_finished_at=2.,prior_sent_target_m=.0295)
        self.proposal=dict(schema=recovery.OBSERVATION_SCHEMA,route=recovery.OBSERVATION_ROUTE,
            max_observations=1,max_probes=0,consumed_probe_count=3,stage_attempt_limit=3,
            proposal_sha256='b'*64,budget=dict(steps=2),snapshot=dict(event=dict(event_id='old-failed')),existing_contact=self.admission)
        source=dict(candidate_measurement=dict(anchor=self.anchor),audited_existing_contact={**self.admission,'audit_proposal_sha256':'b'*64})
        self.source=patch.object(recovery,'current_contact_source',return_value=(dict(grasp_object_id='white_charger'),source))
        self.source.start();self.addCleanup(self.source.stop)
        self.payload=dict(arm='left',kind='supported_contact_observe',source_event_id='old-failed',observation_proposal_sha256='b'*64,
            request=dict(operation='supported_contact_observe',observation_id='new-rgb',object_id='white_charger',visual_description='Synthetic new bilateral contact under independent support.',
                contact_relation='bilateral_finger_contact',support_relation='independent_support_present'))
        measurement=dict(trace_sha256='c'*64,anchor=copy.deepcopy(self.anchor),observed=dict(width_m=.03325),started_at=10.,ended_at=13.,health='healthy',mode='stationary',sample_count=31,feedback_advances=30)
        candidate=dict(schema='piper_existing_supported_contact_candidate_v1',basis='existing_target_observation',
            existing_target_ref=dict(event_id='old-failed',receipt_sha256='a'*64,sent_at=1.,finished_at=2.,requested_width_m=.0295),trace_sha256='c'*64,started_at=10.,completed_at=13.,observed_width_m=.03325,minimum_target_gap_m=.00375,completion='observation_only',target_may_remain_active=True)
        zero={side:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for side in ('left','right')}
        self.receipt=dict(ok=True,status='observed_supported_contact_candidate',completion_mode='supported_contact_observe',candidate_basis='existing_target_observation',nominal_force_N=None,
            audited_existing_contact=source['audited_existing_contact'],loaded=False,hardware_commands_sent=0,target_calls_sent=0,transmission_counts=zero,session_transmission_counts=zero,
            errors=[],guard_violations=[],physical_stop_verified=None,grasp_verified=False,enable_commands_sent=0,stop_commands_sent=0,retries=0,passive_arm_commands_sent=0,
            current_contact_candidate=candidate,candidate_measurement=measurement)
        self.event=dict(owner='new-owner',step=3,payload_json=json.dumps(self.payload),status='complete',success=1,began_at=10.,finished_at=13.,receipt_json=json.dumps(self.receipt))
    def state(self,events):return recovery._existing_contact_runtime(dict(owner='new-owner'),self.proposal,events)
    def test_observation_then_candidate_without_restoring_probe_allowance(self):
        self.assertEqual(self.state([])['phase'],'contact_observation_required')
        current=self.state([self.event]);self.assertEqual(current['phase'],'contact_candidate');self.assertEqual(current['consumed_probe_count'],3)
        for key,value in (('max_observations',2),('max_probes',1),('consumed_probe_count',2)):
            old=self.proposal[key];self.proposal[key]=value
            with self.assertRaises(PairLedgerError):self.state([])
            self.proposal[key]=old
    def test_unknown_failed_or_second_observation_never_retry(self):
        for changes in (dict(status='pending',success=None),dict(success=0)):
            self.assertEqual(self.state([{**self.event,**changes}])['phase'],'unresolved')
        with self.assertRaises(PairLedgerError):self.state([self.event,self.event])
    def test_targets_peer_and_wrong_contact_source_rejected(self):
        for changes in (dict(kind='gripper'),dict(kind='joint'),dict(kind='query'),dict(kind='initialization'),dict(arm='right'),dict(source_event_id='other'),dict(observation_proposal_sha256='0'*64)):
            with self.subTest(changes=changes),self.assertRaises(PairLedgerError):recovery._check_observation_payload({**self.payload,**changes},self.proposal)
    def test_transferred_old_candidate_new_body_send_or_unobserved_trace_rejected(self):
        variants=[]
        for changes in (dict(hardware_commands_sent=1),dict(candidate_probe={'old':True}),dict(nominal_force_N=.2),dict(hardware_commands_sent=False)):
            variants.append({**self.receipt,**changes})
        altered=copy.deepcopy(self.receipt);altered['candidate_measurement']['anchor']['joints_rad'][3]=.01;variants.append(altered)
        altered=copy.deepcopy(self.receipt);altered['current_contact_candidate']['existing_target_ref']['event_id']='new-observation';variants.append(altered)
        altered=copy.deepcopy(self.receipt);altered['candidate_measurement']['ended_at']=12.;variants.append(altered)
        for receipt in variants:
            with self.subTest(receipt=receipt),self.assertRaises(PairLedgerError):self.state([{**self.event,'receipt_json':json.dumps(receipt)}])
