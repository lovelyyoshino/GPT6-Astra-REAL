#!/usr/bin/env python3
"""Exact zero-execution continuation; invalid J5 requests stay rejected without target clipping."""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_post_contact_entry as predecessor
profile=predecessor.profile;guard=predecessor.guard;motion=predecessor.motion
support=predecessor.support;require=predecessor.require;sha=predecessor.sha;SessionFile=predecessor.SessionFile
CONFIG_SHA=predecessor.CONFIG_SHA;PROFILE_SHA=predecessor.PROFILE_SHA
PREDECESSOR_SHA='c776fd1111fc039db8f2a31cdfb3fcd25b227ddaf83b814f9e717af86bd224cb'
PARENT_SHA='86afa66ccf1f915d46d5b065a880dbc103320c7eee0f1dd44c20ea497e6bd7a5'
RESULT_SHA='2070ca85a3efeaf66e26f843fa84642f176f3c036d37dbbeff832d10e9d1aed0'
LAST_SUCCESS_SHA='182cb401ab507aff4f0a1b52beff0ad36145c92f9a14f9cacfe99c1d806d26a5'
RAW_REVIEW_SHA='ae19b922b52f0b903e6714d9fbee1e2395ffad59d6af903a945ac656a2afaa6f'
PARENT_TOKEN='7dcfda0a69e1410a916f40cd15c0e1ef'
PARENT_RUN=ROOT/'runs/cola_on_cup_post_contact_20261006_233500'
LAST_TARGET=[53018,103860,-48249,0,-39839,0]
REFUSED_TARGET=[53018,103860,-48249,0,-39689,0]
REFUSAL='J5 command increment exceeds200millidegrees'


class PreDispatchJ5Refusal(RuntimeError):
    """Pure request-decode rejection; never evidence of a completed physical action."""


def decode_require(condition,message):
    if not condition and message==REFUSAL:raise PreDispatchJ5Refusal(message)
    return require(condition,message)


# Only this exact pure decoder assertion is typed. Every send_once, RX,
# path, arrival, slip and I/O assertion retains its frozen exception behavior.
scoped_target=profile.private_function(predecessor.scoped_target,require=decode_require)


def child_name(boot):return 'zero_tx_continuation_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(zero_tx_continuation_reviewed=True,zero_tx_continuation_parent_sha256=PARENT_SHA,
        zero_tx_continuation_result_sha256=RESULT_SHA,zero_tx_continuation_raw_review_sha256=RAW_REVIEW_SHA,
        original_thresholds_unchanged=True,no_old_goal_replay=True,new_explicit_target_required=True)
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Exact zero-execution continuation review required')
    result_path=PARENT_RUN/'runtime/action_000093_result.json';last_path=PARENT_RUN/'runtime/action_000092_result.json'
    raw_path=PARENT_RUN/'seq91_92_completed_seq93_presend_review.json'
    require(sha(result_path)==RESULT_SHA and sha(last_path)==LAST_SUCCESS_SHA and sha(raw_path)==RAW_REVIEW_SHA,'Zero-execution evidence changed')
    state=json.loads(result_path.read_text());last=json.loads(last_path.read_text());audit=json.loads(raw_path.read_text())
    require(parent['status']==state and parent['identity']['source_sha256']==PREDECESSOR_SHA
        and parent['identity']['config_sha256']==CONFIG_SHA and parent['identity']['adoption_token']==PARENT_TOKEN
        and parent['generation']==19 and parent['contact_stage']==parent['regrasp_stage']==parent['stage']=='task',
        'Wrong zero-execution parent identity/scope')
    require(state['sequence']==93 and state['phase']=='failed'and not state['active']
        and state['failure']==parent['failure']['error']==REFUSAL
        and parent['failure']['accepted_target_may_continue']is False
        and state['receipts']==[] and state['command_sent_unix_s']is None
        and state['initial_transaction_started']is False and state['hold_requested']is False
        and parent['pending']==dict(sequence=93,kind='preflight',attempted_frames=0)
        and parent['stop_latched']is False and state['stop_latched']is False
        and parent.get('raw_feedback_fault')is None and state.get('raw_feedback_fault')is None
        and parent['held_raw']==LAST_TARGET and state['target_raw']==REFUSED_TARGET
        and parent['attempted_frames']==368,'Only exact zero-transaction seq93 is admissible')
    rows=[]
    with (PARENT_RUN/'runtime/feedback.jsonl').open('rb')as stream:
        for line in stream:
            value=json.loads(line)
            if value.get('sequence')==93:rows.append(value['event'])
    require(rows==['action_admitted','failed'],'Seq93 has unaccounted activity')
    evidence=audit['seq93'];zero=evidence['zero_transaction_evidence'];raw=evidence['raw_review_using_retained_seq92_box']
    require(evidence['original_result_sha256']==RESULT_SHA and evidence['failure']==REFUSAL
        and zero['session_snapshot_sha256']==PARENT_SHA and zero['receipts']==[]
        and zero['initial_transaction_started']is False and zero['command_sent_unix_s']is None
        and zero['pending']==parent['pending']and evidence['retained_previous_goal_raw']==LAST_TARGET
        and evidence['rechecked_j5_command_difference_mdeg']==228 and evidence['original_j5_increment_cap_mdeg']==200,
        'Raw diagnosis does not bind the exact zero-execution refusal')
    require(raw['guard_checks_clean']is True and raw['transport_clean']is True and raw['full_raw_reviewed']is True
        and all(raw[k]==[]for k in ('nominal_violations','joint_tracking_violations','jaw_guard_violations','health_violations','transport_violations'))
        and raw['post_command_jaw_range_m']==[.05747,.05747],'Retained target raw evidence is not clean')
    predecessor.verify_windows(evidence['source_windows'])
    previous=next(a for a in audit['completed_actions']if a['sequence']==92)
    receipt=previous['receipt']
    require(last['sequence']==92 and last['generation']==19 and last['phase']=='completed'
        and last['stable_window']['duration_s']>=3 and receipt['kind']=='initial'
        and receipt['attempted_frames']==receipt['socket_send_returns']==4 and receipt['target_raw']==LAST_TARGET,
        'Previous successful target is unresolved')
    predecessor.clean_raw(previous,92,19,PARENT_TOKEN,LAST_SUCCESS_SHA,receipt,last)
    # Retain, rather than replace, the completed clearance qualification.
    reviews=parent['post_contact_reviews'];require(len(reviews)==1,'Unexpected prior clearance reviews')
    q=reviews[0];review_path=PARENT_RUN/'runtime/reviews/action_000001_review.json'
    qualification_path=PARENT_RUN/'runtime/action_000001_result.json'
    require(q['sequence']==1 and q['generation']==18 and sha(review_path)==q['review_sha256']
        and sha(qualification_path)==q['result_sha256'],'Prior clearance qualification changed')
    material=json.loads(review_path.read_text());qraw=json.loads(Path(material['raw_review']['path']).read_text())
    predecessor.review_material(review_path,1,18,q['result_sha256'],PARENT_TOKEN,qraw['receipt'],json.loads(qualification_path.read_text()))
    require(material['next_stage']=='task','Parent did not have task permission')
    item=reviewed['zero_tx_visual_review'];path=Path(item['path'])
    require(path.is_absolute()and sha(path)==item['sha256'],'Current continuation RGB review changed')
    view=json.loads(path.read_text())
    require(view.get('reviewed')is True and view.get('seq93_zero_execution_reviewed')is True
        and view.get('new_explicit_target_required')is True and view.get('no_old_goal_replay')is True
        and view.get('loaded_can')is True and view.get('no_visible_human_contact')is True
        and view.get('held_load_context')and view.get('images')and view.get('user_authorization'),'Current held-load continuation review required')
    for image in view['images']:
        p=Path(image['path']);require(p.is_absolute()and sha(p)==image['sha256'],'Continuation RGB changed')
    return copy.deepcopy(last['after'])


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    path=root/predecessor.child_name(boot)
    with SessionFile(path):
        original=path.read_bytes();require(hashlib.sha256(original).hexdigest()==PARENT_SHA,'Failed zero-TX parent changed')
        parent=json.loads(original);endpoint=reviewed_evidence(reviewed,parent)
        def prior_evidence(material,older_parent):
            predecessor.reviewed_evidence(material,older_parent)
            return endpoint
        implementation=profile.private_function(predecessor.reserve.__wrapped__,child_name=child_name,reviewed_evidence=prior_evidence)
        try:
            with contextmanager(implementation)(boot,reviewed,session_root=root)as(_,__,store,session):
                session.update(generation=20,stage='task',regrasp_stage='task',contact_stage='task',first_segment_verified=True,
                    stop_latched=False,held_raw=list(LAST_TARGET),scope_anchor_raw=list(LAST_TARGET),
                    post_contact_reviews=copy.deepcopy(parent['post_contact_reviews']),lifting_completed=parent['lifting_completed'],
                    ordinary_task_authorized=True,jaw_authorized=True,prior_zero_tx_failure=copy.deepcopy(parent['failure']),
                    prior_zero_tx_result_sha256=RESULT_SHA,zero_tx_refusals=[],initial_scope='same_task_typed_request_refusal',
                    no_automatic_goal_replay=True)
                store.save(session);yield parent,endpoint,store,session
        finally:require(path.read_bytes()==original,'Failed zero-TX parent bytes changed')


class ZeroTXContinuationTask(predecessor.PostContactTask):
    execute=profile.private_function(guard.GuardedTask.execute,support=support,decode=scoped_target)

    def __init__(self,*a,**k):
        super().__init__(*a,**k);self.zero_tx_handoff_pending=True
        self.identity.update(zero_tx_predecessor_sha256=PREDECESSOR_SHA,zero_tx_raw_review_sha256=RAW_REVIEW_SHA,
            parent_sequence=93,parent_generation=19,parent_adoption_token=PARENT_TOKEN,
            initial_scope='same_task_typed_request_refusal',no_automatic_goal_replay=True,
            j5_request_increment_max_mdeg=200)

    def baseline(self):
        sample=super().baseline()
        if self.zero_tx_handoff_pending:
            require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(sample['raw_q'],LAST_TARGET))<=.003,
                'Fresh zero-TX handoff differs from retained successful target')
            require(sample['jaw_code']==64 and abs(sample['opening_m']-.05747)<=.0005,'Held jaw changed before continuation')
            self.zero_tx_handoff_pending=False
        return sample

    def fail(self,error):
        if type(error)is not PreDispatchJ5Refusal:return super().fail(error)
        with self.state_lock:
            with self.node.piper.rx_lock:fault=copy.deepcopy(self.node.piper.rx_latch.first_fault)
            sequence=self.state['sequence']
            soft=(self.session['stage']==self.session['regrasp_stage']==self.session['contact_stage']=='task'
                and self.state['kind']=='joint'and self.state['phase']=='preflight'and self.state['active']
                and self.session['pending']==dict(sequence=sequence,kind='preflight',attempted_frames=0)
                and not self.state['initial_transaction_started']and self.state['command_sent_unix_s']is None
                and self.state['receipts']==[]and self.node.piper.ticket is None and not self.state['hold_requested']
                and not self.session['failure']and not self.session['stop_latched']
                and not self.node.failed and not self.node.piper.broken and fault is None)
            if not soft:return motion.frozen.RXGuardedTask.fail(self,error)
            result=dict(phase='rejected_preflight',kind='joint',sequence=sequence,error=str(error),
                refusal_type='J5_request_increment_exceeds_200mdeg',actuator_frames=0,
                actuator_transaction_started=False,accepted_target_may_continue=False,target_reached=False,
                task_success=False,new_explicit_request_required=True,target_not_clipped=True,
                requested_raw=copy.deepcopy(self.state['requested_raw']),first_state=copy.deepcopy(self.state['before']),
                latest_state=copy.deepcopy(self.state['latest_state']))
            self.session.setdefault('zero_tx_refusals',[]).append(copy.deepcopy(result))
            self.session['pending']=None
            self.state.update(phase='rejected_preflight',active=False,result=result,result_sha256=None,
                last_refusal=dict(error=str(error),unix_s=self.clock.time(),actuator_frames=0))
            path=self.output/('action_%06d_preflight_refusal.json'%sequence)
            try:
                require(not path.exists(),'Refusal evidence already exists')
                SessionFile(path).save(result);SessionFile(self.output/('action_%06d_result.json'%sequence)).save(result)
                self.save();self.record('rejected_preflight',durable=True,**result)
            except Exception as persistence_error:
                motion.frozen.RXGuardedTask.fail(self,persistence_error)
                raise

    def record(self,event,**fields):
        if event=='ready':
            fields.update(initial_scope='same_task_typed_request_refusal',old_seq93_stays_failed=True,
                no_automatic_goal_replay=True,all_numeric_thresholds_unchanged=True,loaded_can=True)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


def main(argv=None):
    require(sha(predecessor.__file__)==PREDECESSOR_SHA,'Frozen post-contact source changed')
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_zero_tx.json').read_bytes()==(directory/'reviewed_post_contact.json').read_bytes(),
        'Zero-TX review alias mismatch')
    return profile.private_function(predecessor.main,__file__=__file__,__doc__=__doc__,PostContactTask=ZeroTXContinuationTask,
        reserve=reserve,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)(argv)


if __name__=='__main__':main()
