#!/usr/bin/env python3
"""One reviewed loaded-contact retreat to the prior goal; never release or resume a task."""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_readiness_entry as predecessor
profile=predecessor.profile;guard=predecessor.guard;motion=predecessor.motion
support=predecessor.support;require=predecessor.require;sha=predecessor.sha;SessionFile=predecessor.SessionFile
CONFIG_SHA=predecessor.CONFIG_SHA;PROFILE_SHA=predecessor.PROFILE_SHA
PREDECESSOR_SHA='0a4f3edefeb8799c816b37d6fce6d0b79ae6e4fb4c0b8b39b9ece8463e00f46d'
PARENT_SHA='a378294bc16d7554944984bae851622f21f7ec1afa0b5cdba1fb960c336be235'
RESULT_SHA='3e92d39274eff5e4d6972ba12f575e00817d4903560ebd454f37e1832fc2eb80'
LAST_SUCCESS_SHA='464afd29acd022c1c5b20bd97b6e36deffbbe6be1ed860cdc50991373ee4e7ea'
LAST_RECEIPT_SHA='5bc4c74b7dfb8891ada35d83ccbeef38adc2511f6dc58f893368ffad79e745f0'
LAST_RAW_REVIEW_SHA='e395b3eecd29b02519c9699b099bd67fe8bb3dd93320deeaea81c93176d9f9c8'
RAW_REVIEW_SHA='344ed25a09346030fc8a8e354b75666b49c7f8bbecc5e475d46b8172ace4c69e'
VISUAL_SHA='3f3da507ec7e215c8e7ef2a22d803059be46a937652832ea9cc9d588ae3478da'
PARENT_TOKEN='7df0f39b16824684951888356510f7d3'
PARENT_RUN=ROOT/'runs/cola_on_cup_sliding_20261006_215500'
TARGET=[53018,109860,-48249,0,-52889,0]
FAILED_TARGET=[53018,110160,-48249,0,-52889,0]
FAILURE='Raw feedback violation latched: Outside original joint box'


def child_name(boot):return 'guarded_contact_retreat_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(contact_retreat_reviewed=True,loaded_contact_parent_sha256=PARENT_SHA,
        loaded_contact_result_sha256=RESULT_SHA,loaded_contact_raw_review_sha256=RAW_REVIEW_SHA,
        loaded_contact_target_raw=TARGET,no_jaw_authorized=True,no_ordinary_task_authorized=True,
        original_contact_failure_preserved=True,original_thresholds_unchanged=True)
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Exact loaded-contact retreat review required')
    result_path=PARENT_RUN/'runtime/action_000060_result.json'
    prior_path=PARENT_RUN/'runtime/action_000059_result.json'
    raw_path=PARENT_RUN/'seq60_failure_raw_review.json'
    require(sha(result_path)==RESULT_SHA and sha(prior_path)==LAST_SUCCESS_SHA and sha(raw_path)==RAW_REVIEW_SHA,
        'Contact evidence changed')
    state=json.loads(result_path.read_text());prior=json.loads(prior_path.read_text());audit=json.loads(raw_path.read_text())
    require(parent['status']==state and parent['identity']['source_sha256']==PREDECESSOR_SHA
        and parent['identity']['config_sha256']==CONFIG_SHA and parent['identity']['adoption_token']==PARENT_TOKEN
        and parent['generation']==16 and parent['regrasp_stage']=='task','Wrong loaded-contact parent')
    receipts=state['receipts']
    require(state['sequence']==60 and state['phase']=='failed'and not state['active']
        and parent['failure']['error']==state['failure']==FAILURE
        and parent['failure']['accepted_target_may_continue']is True
        and len(receipts)==1 and receipts[0]['kind']=='initial'
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4
        and receipts[0]['target_raw']==FAILED_TARGET
        and parent['pending']==dict(sequence=60,kind='initial',target_raw=FAILED_TARGET,attempted_frames=4),
        'Only the exact complete failed contact transaction is admissible')
    fault=parent['raw_feedback_fault']
    require(fault==state['raw_feedback_fault']==audit['original_raw_feedback_fault']
        and fault['axis']==5 and fault['raw']==-52532 and fault['context']['sequence']==60
        and audit['source_result_sha256']==RESULT_SHA and audit['failure_preserved']is True
        and audit['review_pass']is False and audit['receipt']==receipts[0], 'Original RX violation must remain failed')
    found=[]
    with (PARENT_RUN/'runtime/feedback.jsonl').open('rb')as stream:
        for line in stream:
            row=json.loads(line)
            if row.get('sequence')==59 and row.get('event')=='send_receipt':
                require(hashlib.sha256(line).hexdigest()==LAST_RECEIPT_SHA,'Previous goal receipt changed')
                found.append(row)
    require(len(found)==1 and found[0]['target_raw']==TARGET and found[0]['kind']=='initial'
        and found[0]['attempted_frames']==found[0]['socket_send_returns']==4
        and prior['phase']=='completed'and prior['sequence']==59 and prior['stable_window']['duration_s']>=3,
        'Previous completed goal not established')
    prior_raw_path=PARENT_RUN/'seq37_to_59_raw_review.json'
    require(sha(prior_raw_path)==LAST_RAW_REVIEW_SHA,'Previous goal independent review changed')
    previous_raw=json.loads(prior_raw_path.read_text())['actions'][-1]
    require(previous_raw['sequence']==59 and previous_raw['target_raw']==TARGET
        and previous_raw['completed_result_sha256']==LAST_SUCCESS_SHA
        and previous_raw['guard_checks_clean']is True and previous_raw['full_raw_reviewed']is True
        and previous_raw['transport_clean']is True,'Previous goal raw review not clean')
    for item in audit['source_windows']+previous_raw['source_windows']:
        path=Path(item['path']);length=item['last_byte_exclusive']-item['first_byte']
        require(path.is_absolute()and 0<length<=50000000,'Invalid contact raw byte window')
        with path.open('rb')as stream:stream.seek(item['first_byte']);content=stream.read(length)
        require(len(content)==length and hashlib.sha256(content).hexdigest()==item['sha256'],'Contact raw evidence changed')
    tail=audit['latest_ten_seconds'];checks=tail['raw_guard_checks'];summary=tail['summary']
    require(tail['window_end_unix_s']-tail['window_begin_unix_s']>=3
        and checks['guard_checks_clean']is True and checks['transport_clean']is True
        and checks['full_raw_reviewed']is True and summary['all_samples_within_original_arrival_rad_003']is True
        and all(a==b for a,b in summary['joint_ranges_raw']) and summary['xyz_bbox_span_m']<=.0005
        and summary['orientation_diameter_rad']<=.003 and summary['jaw_range_raw']==[57470,57470]
        and summary['status_payloads']==['0100010000000000']
        and all(v==[64]for v in summary['motor_codes'].values()),'Settled healthy loaded endpoint absent')
    visual=reviewed['loaded_contact_visual_review'];path=Path(visual['path'])
    require(path.is_absolute()and sha(path)==visual['sha256']==VISUAL_SHA,'Human/visual contact review changed')
    view=json.loads(path.read_text())
    require(view.get('reviewed')is True and view.get('can_held_by_gripper')is True
        and view.get('can_supported_by_cup')is False and view.get('contact_possible')is True
        and view.get('user_support_confirmation')=='仅碰到杯沿／仍悬空'
        and view.get('no_visible_human_contact')is True and view.get('proposed_target_raw')==TARGET
        and view.get('source_failed_sequence')==60 and view.get('source_generation')==16
        and view.get('source_adoption_token')==PARENT_TOKEN,'Loaded rim-only recovery scope not confirmed')
    images=reviewed['loaded_contact_images']
    require(len(images)==2 and {i['path']for i in images}==set(view['images'].values()),'Current RGB evidence missing')
    for item in images:
        p=Path(item['path']);require(p.is_absolute()and sha(p)==item['sha256'],'Current loaded-contact RGB changed')
    raw=summary['final_raw_q']
    require(all(lo<=v<=hi for v,(lo,hi)in zip(raw,support.JOINT_LIMITS_RAW)),'Settled endpoint outside nominal limits')
    # A historical stationary summary, not a forged fresh snapshot. Do not copy
    # the failure-time raw frames/stamps while replacing their joint values.
    return dict(raw_q=list(raw),q=[v*support.RAD_PER_RAW for v in raw],
        pose=[v[0]for v in summary['pose_component_ranges']],opening_m=.05747,jaw_code=64,
        evidence_scope='historical_stationary_raw_summary_requires_fresh_adoption',
        raw_review_sha256=RAW_REVIEW_SHA)


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    path=root/predecessor.child_name(boot)
    with SessionFile(path):
        original=path.read_bytes();require(hashlib.sha256(original).hexdigest()==PARENT_SHA,'Contact failed parent changed')
        parent=json.loads(original);endpoint=reviewed_evidence(reviewed,parent)
        def prior_evidence(material,older_parent):
            predecessor.reviewed_evidence(material,older_parent)
            return endpoint
        implementation=profile.private_function(predecessor.reserve.__wrapped__,child_name=child_name,reviewed_evidence=prior_evidence)
        try:
            with contextmanager(implementation)(boot,reviewed,session_root=root)as(_,__,store,session):
                session.update(generation=17,regrasp_stage='task',first_segment_verified=True,
                    contact_retreat_attempted=False,contact_retreat_completed=False,
                    loaded_contact_failure=copy.deepcopy(parent['failure']),loaded_contact_raw_fault=copy.deepcopy(parent['raw_feedback_fault']),
                    loaded_contact_parent_result_sha256=RESULT_SHA,loaded_contact_target_raw=list(TARGET),
                    initial_scope='one_loaded_contact_retreat',can_held_by_gripper=True,
                    ordinary_task_authorized=False,jaw_authorized=False)
                store.save(session);yield parent,endpoint,store,session
        finally:require(path.read_bytes()==original,'Contact parent bytes changed')


class ContactRetreatTask(predecessor.ReadinessTask):
    def __init__(self,*a,**k):
        super().__init__(*a,**k)
        for key in ('empty_gripper_operator_confirmed','table_supported_contact_confirmed'):
            self.identity.pop(key,None)
        self.identity.update(parent_sequence=60,parent_generation=16,parent_adoption_token=PARENT_TOKEN,
            contact_predecessor_sha256=PREDECESSOR_SHA,contact_raw_review_sha256=RAW_REVIEW_SHA,
            initial_scope='one_loaded_contact_retreat',purpose='lift_held_can_away_from_rim_contact',
            can_held_by_gripper=True,can_supported_by_cup=False,contact_possible=True,
            table_supported_object='transparent cup only',ordinary_task_authorized=False,jaw_authorized=False,
            loaded_contact_target_raw=list(TARGET))

    def baseline(self):
        sample=super().baseline()
        require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(sample['raw_q'],FAILED_TARGET))<=.003,
            'Current loaded endpoint differs from the reviewed failed goal')
        require(sample['jaw_code']==64 and abs(sample['opening_m']-.05747)<=.0005,
            'Held-load jaw differs from reviewed57.47mm baseline')
        return sample

    def execute(self,message):
        # This pure shape/encoding check does not consume a request for a different target.
        raw=guard.decode(message,dict(raw_q=[0]*6),dict(stage='task'))
        require(raw==TARGET,'Only the exact prior seq59 goal is permitted')
        with self.state_lock:
            require(not self.session.get('contact_retreat_attempted')and not self.session['failure']
                and not self.session['stop_latched']and self.node.adopted,'Contact retreat opportunity consumed/unavailable')
            try:
                self.session['contact_retreat_attempted']=True;self.save()
            except Exception as error:self.fail(error);raise
        try:return super().execute(message)
        except Exception as error:self.fail(error);raise

    def send_once(self,raw,measured,origin,target,kind,*,moving):
        require(kind=='initial'and raw==TARGET and self.session.get('contact_retreat_attempted')
            and not self.session.get('contact_retreat_completed'),'Only one exact initial retreat transaction is allowed')
        return super().send_once(raw,measured,origin,target,kind,moving=moving)

    def fail(self,error):
        # Even a typed zero-TX readiness failure consumes this recovery opportunity.
        return motion.frozen.RXGuardedTask.fail(self,error)

    def request_hold(self,*a,**k):return False,dict(error='Fixed retreat only; no replacement/hold target',hold_confirmed=False)
    def send_hold(self,*a,**k):raise RuntimeError('Contact retreat cannot dispatch a hold replacement')
    def gripper(self,*a,**k):raise RuntimeError('Held load: all jaw commands forbidden')
    def resume(self,*a,**k):raise RuntimeError('Contact retreat never authorizes ordinary continuation')
    def review_probe(self,*a,**k):raise RuntimeError('No promotion endpoint in contact retreat scope')
    def review_opening(self,*a,**k):raise RuntimeError('No opening review in contact retreat scope')

    def finish(self,phase,after,window,**extra):
        require(phase=='completed'and self.state['kind']=='joint','Only normal completion of the fixed retreat is valid')
        with self.state_lock:
            self.node.adopted=False
            self.session['stop_latched']=True;self.state['stop_latched']=True
            result=super().finish(phase,after,window,contact_retreat_only=True,jaw_command_not_sent=True,
                held_load_assumed_from_review=True,post_retreat_grasp_verified=False,
                release_verified=False,task_success=False,review_required=True,ordinary_task_authorized=False,**extra)
            self.session.update(contact_retreat_completed=True,regrasp_stage='contact_retreat_review')
            self.state.update(regrasp_stage='contact_retreat_review',probe_review_service=None,resume_service=None)
            self.save();return result

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(initial_scope='one_loaded_contact_retreat',can_held_by_gripper=True,
                contact_possible=True,can_supported_by_cup=False,only_joint_target_raw=TARGET,
                jaw_authorized=False,ordinary_task_authorized=False,original_seq60_stays_failed=True)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


class LoadedParser(argparse.ArgumentParser):
    def add_argument(self,*names,**kwargs):
        if names==('--empty-gripper-confirmed',):names=('--loaded-rim-contact-confirmed',)
        return super().add_argument(*names,**kwargs)


def main(argv=None):
    require(sha(predecessor.__file__)==PREDECESSOR_SHA,'Frozen readiness source changed')
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_contact_retreat.json').read_bytes()==(directory/'reviewed_handoff.json').read_bytes(),
        'Contact review alias mismatch')
    # Preserve the full dependency verification of the immediate predecessor,
    # privately substitute only its review alias and final entry/task bindings.
    source=predecessor.predecessor
    require(sha(source.__file__)==predecessor.PREDECESSOR_SHA
        and sha(predecessor.stability.__file__)==predecessor.STABILITY_SHA
        and sha(source.predecessor.__file__)==source.PREDECESSOR_SHA
        and sha(source.wide.__file__)==source.predecessor.WIDE_SHA
        and sha(profile.__file__)==PROFILE_SHA and sha(source.previous.__file__)==source.wide.PREVIOUS_SHA
        and sha(source.previous.release.__file__)==source.previous.RELEASE_SOURCE_SHA
        and sha(motion.__file__)==source.previous.MOTION_SOURCE_SHA
        and sha(profile.original.__file__)==motion.PROFILE_SHA and sha(motion.frozen.__file__)==motion.RX_SHA
        and sha(motion.frozen.overlay.__file__)==motion.RAW_OVERLAY_SHA,'Frozen contact dependency changed')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=LoadedParser))
    return profile.private_function(motion.frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=ContactRetreatTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)(argv)


if __name__=='__main__':main()
