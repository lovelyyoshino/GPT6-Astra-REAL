#!/usr/bin/env python3
"""Same guarded task with contiguous stability windows and exact zero-TX refusal semantics."""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_recorded_probe_entry as predecessor
import sliding_stability as stability
profile=predecessor.profile;guard=predecessor.guard;motion=predecessor.motion
support=predecessor.support;require=predecessor.require;sha=predecessor.sha;SessionFile=predecessor.SessionFile
CONFIG_SHA=predecessor.CONFIG_SHA;PROFILE_SHA=predecessor.PROFILE_SHA
STABILITY_SHA='fa4ff24d686294bb34a387d651cd9640def0d528e5c494a369a5050d229c1f2e'
PREDECESSOR_SHA='21e773be0b837ef675a74721389da76121b85248c89cbf803e6398e0b7477f65'
PARENT_SHA='ee24acae5987bbd47e0598b690fd2b81442aa523d95acb1acef36fba4eb82473'
RESULT_SHA='2c4d49fe5285701c697b01120e5535a559727208b23941d38e0a3cec740eead5'
LAST_SUCCESS_SHA='021b070308e5f75d4ed130d6bae16c196dd1b65a6d006370eab9889d23d84994'
DIAGNOSIS_SHA='7e5bd5274dc2dce8be1b2e52f10128d4c8991f6d4e09d43c72e389f2dc24e67f'
PARENT_TOKEN='985191c3284343a390a8d9b74e0bb67b'
PARENT_RUN=ROOT/'runs/cola_on_cup_probe_recorded_20261006_212800'
RECEIPT_SHA={1:'74a9d6bf56f63a800cdae43e7b975bbb6d548fb5b136ef87215f476297f72550',
             9:'9d7f62878ab2a7990544137fe9c2a6164e1f86ef0da7c07122ad0511944810f4'}


def child_name(boot):return 'guarded_sliding_readiness_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(sliding_readiness_reviewed=True,zero_tx_parent_session_sha256=PARENT_SHA,
        zero_tx_result_sha256=RESULT_SHA,baseline_diagnosis_sha256=DIAGNOSIS_SHA,
        original_thresholds_unchanged=True,no_replay_of_rejected_goal=True)
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Exact zero-TX readiness review required')
    result_path=PARENT_RUN/'runtime/action_000010_result.json';diagnosis_path=PARENT_RUN/'seq10_baseline_diagnosis.json'
    success_path=PARENT_RUN/'runtime/action_000009_result.json'
    require(sha(result_path)==RESULT_SHA and sha(diagnosis_path)==DIAGNOSIS_SHA
        and sha(success_path)==LAST_SUCCESS_SHA,'Parent evidence changed')
    state=json.loads(result_path.read_text());diagnosis=json.loads(diagnosis_path.read_text());last=json.loads(success_path.read_text())
    require(parent['status']==state and parent['identity']['source_sha256']==PREDECESSOR_SHA
        and parent['identity']['config_sha256']==CONFIG_SHA and parent['identity']['adoption_token']==PARENT_TOKEN
        and parent['generation']==15 and parent['regrasp_stage']=='task','Wrong parent identity/stage')
    require(state['sequence']==10 and state['phase']=='failed'and not state['active']
        and state['failure']==parent['failure']['error']=='Baseline stability not established'
        and parent['failure']['accepted_target_may_continue']is False and not parent['stop_latched']
        and state['receipts']==[] and state['command_sent_unix_s']is None
        and state['initial_transaction_started']is False
        and parent['pending']==dict(sequence=10,kind='preflight',attempted_frames=0)
        and parent.get('raw_feedback_fault')is None and state.get('raw_feedback_fault')is None,
        'Only the exact preflight zero-TX refusal is admissible')
    receipts={}
    with (PARENT_RUN/'runtime/feedback.jsonl').open('rb')as stream:
        for line in stream:
            value=json.loads(line)
            if value.get('sequence')==10:
                require(value.get('event')not in ('intent','send_receipt'),'Sequence10 contains a dispatch intent/receipt')
            seq=value.get('sequence')
            if value.get('event')=='send_receipt'and seq in RECEIPT_SHA:
                require(seq not in receipts and hashlib.sha256(line).hexdigest()==RECEIPT_SHA[seq],'Earlier receipt changed')
                receipts[seq]=value
    require(set(receipts)=={1,9} and all(r['kind']=='initial'and r['attempted_frames']==r['socket_send_returns']==4
        for r in receipts.values())and last['phase']=='completed'and last['sequence']==9
        and last['stable_window']['duration_s']>=3 and parent['held_raw']==receipts[9]['target_raw'],
        'Previous qualified/successful transactions incomplete')
    reviewed_probe=parent['probe_reviews']
    require(len(reviewed_probe)==1 and reviewed_probe[0]['sequence']==1,'Original recorded qualification absent')
    probe=reviewed_probe[0];probe_path=PARENT_RUN/'runtime/action_000001_result.json'
    review_path=PARENT_RUN/'runtime/reviews/action_000001_review.json'
    require(sha(probe_path)==probe['result_sha256']and sha(review_path)==probe['review_sha256'],'Qualification evidence changed')
    predecessor.previous.review_material(review_path,'wrist_review',1,14,probe['result_sha256'],PARENT_TOKEN,
        receipts[1],json.loads(probe_path.read_text()))
    require(diagnosis['sequence']==10 and diagnosis['result_sha256']==RESULT_SHA
        and diagnosis['receipts']==[]and diagnosis['full_raw_reviewed_for_interval']is True
        and diagnosis['raw_transport_clean']is True and diagnosis['limits_unchanged']is True
        and diagnosis['raw_transport_violations']==diagnosis['raw_health_violations']==diagnosis['raw_nominal_violations']==[]
        and diagnosis['sliding_all_raw_3s_pass_count']>0,'No unchanged-limit raw basis for readiness correction')
    for item in diagnosis['raw_source_windows']:
        p=Path(item['path']);length=item['last_byte_exclusive']-item['first_byte']
        require(p.is_absolute()and 0<length<=50000000,'Invalid diagnosis raw window')
        with p.open('rb')as stream:stream.seek(item['first_byte']);data=stream.read(length)
        require(len(data)==length and hashlib.sha256(data).hexdigest()==item['sha256'],'Diagnosis raw bytes changed')
    item=reviewed['readiness_visual_review'];path=Path(item['path'])
    require(path.is_absolute()and sha(path)==item['sha256'],'Current visual review changed')
    view=json.loads(path.read_text())
    require(view.get('reviewed')is True and view.get('table_supported')is True
        and view.get('no_visible_hook_or_tension')is True and view.get('held_load_verified')is False
        and view.get('user_authorization')and view.get('images'),'Current supported/contact-possible review required')
    for image in view['images']:
        p=Path(image['path']);require(p.is_absolute()and sha(p)==image['sha256'],'Current RGB changed')
    return copy.deepcopy(state['latest_state'])


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    path=root/predecessor.child_name(boot)
    with SessionFile(path):
        original=path.read_bytes();require(hashlib.sha256(original).hexdigest()==PARENT_SHA,'Zero-TX parent changed')
        parent=json.loads(original);endpoint=reviewed_evidence(reviewed,parent)
        def prior_evidence(material,older_parent):
            predecessor.reviewed_evidence(material,older_parent)
            return endpoint
        implementation=profile.private_function(predecessor.reserve.__wrapped__,child_name=child_name,reviewed_evidence=prior_evidence)
        try:
            with contextmanager(implementation)(boot,reviewed,session_root=root)as(_,__,store,session):
                session.update(generation=16,regrasp_stage='task',first_segment_verified=True,
                    probe_reviews=copy.deepcopy(parent['probe_reviews']),probe_attempted=True,
                    prior_preflight_refusal=copy.deepcopy(parent['failure']),zero_tx_parent_result_sha256=RESULT_SHA,
                    preflight_refusals=[],initial_scope='readiness_correction_no_goal_replay',
                    no_automatic_repeat_if_gap_fault_or_partial=True)
                store.save(session);yield parent,endpoint,store,session
        finally:require(path.read_bytes()==original,'Zero-TX failed parent changed')


class ReadinessTask(predecessor.RecordedTask):
    wait_arrival=profile.private_function(guard.GuardedTask.wait_arrival,StableWindow=stability.StableWindow)
    _gripper=profile.private_function(guard.GuardedTask.gripper,StableWindow=stability.StableWindow)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.identity.update(parent_sequence=10,parent_generation=15,parent_adoption_token=PARENT_TOKEN,
            readiness_predecessor_sha256=PREDECESSOR_SHA,sliding_stability_sha256=sha(stability.__file__),
            initial_scope='same_limits_contiguous_readiness_no_replay',contact_possible=True,initial_no_contact_claim=False,
            baseline_diagnosis_sha256=DIAGNOSIS_SHA)

    def baseline(self):
        deadline,last,window=self.clock.monotonic()+10.,None,stability.StableWindow()
        while self.clock.monotonic()<deadline:
            s=self.read();self.record('feedback',stage='baseline',state=s)
            if last is None or self.advanced(last,s):
                done=window.add(s,self.clock.monotonic());last=s
                if done:
                    if self.handoff_pending:
                        self.session['scope_anchor_raw']=list(s['raw_q']);self.handoff_pending=False
                    return s
            self.clock.sleep(.05)
        raise stability.BaselineNotReady('Baseline stability not established')

    def gripper(self,request):
        require(self.session['regrasp_stage']=='task'and self.session['first_segment_verified'],
            'Prior qualification required before jaw')
        return self._gripper(request)

    def fail(self,error):
        # Only this typed read-only timeout, before any transaction starts, is a
        # readiness refusal. RX/health/path/slip/IO/partial errors retain hard fail.
        with self.state_lock:
            with self.node.piper.rx_lock:fault=copy.deepcopy(self.node.piper.rx_latch.first_fault)
            sequence=self.state['sequence']
            joint_preflight=(self.state['kind']=='joint'and self.state['phase']=='preflight'
                and not self.state['initial_transaction_started']
                and self.session['pending']==dict(sequence=sequence,kind='preflight',attempted_frames=0))
            jaw_preflight=(self.state['kind']=='gripper'and self.state['phase']=='gripper_preflight'
                and self.session['pending']==dict(sequence=sequence,kind='gripper',
                    target_m=self.state.get('jaw_target_m'),attempted_frames=0))
            soft=(type(error)is stability.BaselineNotReady and self.session['stage']=='task'
                and self.session['regrasp_stage']=='task'and (joint_preflight or jaw_preflight)
                and self.state['active']and self.state['command_sent_unix_s']is None
                and self.state['receipts']==[]and self.node.piper.ticket is None
                and not self.session['failure']and not self.session['stop_latched']and not self.state['hold_requested']
                and not self.node.failed and not self.node.piper.broken and fault is None)
            if not soft:return super().fail(error)
            result=dict(phase='rejected_preflight',kind=self.state['kind'],sequence=self.state['sequence'],error=str(error),
                actuator_frames=0,actuator_transaction_started=False,accepted_target_may_continue=False,
                target_reached=False,task_success=False,new_explicit_request_required=True)
            self.session.setdefault('preflight_refusals',[]).append(copy.deepcopy(result))
            self.session['pending']=None
            self.state.update(phase='rejected_preflight',active=False,result=result,result_sha256=None,
                last_refusal=dict(error=str(error),unix_s=self.clock.time(),actuator_frames=0))
            path=self.output/('action_%06d_preflight_refusal.json'%self.state['sequence'])
            try:
                require(not path.exists(),'Refusal evidence already exists')
                SessionFile(path).save(result)
                SessionFile(self.output/('action_%06d_result.json'%self.state['sequence'])).save(result)
                self.save()
                self.record('rejected_preflight',durable=True,**result)
            except Exception as persistence_error:
                super().fail(persistence_error)
                raise

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(initial_scope='readiness_correction_no_goal_replay',old_seq10_stays_failed=True,
                thresholds_unchanged=True,sliding_stability_sha256=sha(stability.__file__),contact_possible=True)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


def main(argv=None):
    require(sha(predecessor.__file__)==PREDECESSOR_SHA and sha(stability.__file__)==STABILITY_SHA,'Frozen predecessor/stability source changed')
    # Execute the predecessor's complete frozen-dependency checks and main body
    # with only this task, reservation and identity bound in a private namespace.
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_readiness.json').read_bytes()==(directory/'reviewed_handoff.json').read_bytes(),
        'Readiness review alias mismatch')
    require(sha(predecessor.predecessor.__file__)==predecessor.PREDECESSOR_SHA
        and sha(predecessor.wide.__file__)==predecessor.predecessor.WIDE_SHA
        and sha(profile.__file__)==PROFILE_SHA and sha(predecessor.previous.__file__)==predecessor.wide.PREVIOUS_SHA
        and sha(predecessor.previous.release.__file__)==predecessor.previous.RELEASE_SOURCE_SHA
        and sha(motion.__file__)==predecessor.previous.MOTION_SOURCE_SHA
        and sha(profile.original.__file__)==motion.PROFILE_SHA and sha(motion.frozen.__file__)==motion.RX_SHA
        and sha(motion.frozen.overlay.__file__)==motion.RAW_OVERLAY_SHA,'Frozen readiness dependency changed')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=predecessor.previous.release.ReleaseParser))
    return profile.private_function(motion.frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=ReadinessTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)(argv)


if __name__=='__main__':main()
