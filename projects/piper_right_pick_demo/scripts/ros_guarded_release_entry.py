#!/usr/bin/env python3
"""Reviewed seq82 table-supported contact release; exactly one55mm jaw action.

All joint commands remain forbidden, including after release. No automatic
recovery, physical stop, power change, retry or old-session mutation.
"""
import argparse
from contextlib import ExitStack,contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_j5_entry as previous
profile=previous.profile;frozen=previous.frozen;guard=previous.guard;support=previous.support
require=previous.require;sha=previous.sha;SessionFile=previous.SessionFile
PREVIOUS_SHA='b9ebb31e3ab791e3844237b1d679cd66e7ad5b540f900bb982bcc9cabb0b2273'
CONFIG_SHA=previous.CONFIG_SHA
PARENT_SHA='9ef8ea2edf424abd38ce6e706f9916dc2ad17a832e6b35068c246f7d3cc8dd89'
RESULT_SHA='5e1d08629365f61fe1cdd516f14df38aa81208e296c1bbacc8cd6e3f1799bd24'
RAW_REVIEW_SHA='8cfa78b6132dd3b888269e362915472e56286af115814a436b695f06dccfecce'
PARENT_TOKEN='cbdc63a7224f43438051eb2afb717ccb'
PARENT_RUN=ROOT/'runs/cola_on_cup_j5margin_20261006_185000'
PARENT_TARGET=[33100,105650,-31850,0,-52600,0]


def child_name(boot):return 'guarded_contact_release_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(reviewed=True,release_only=True,table_supported_contact_confirmed=True,
        joint_motion_authorized=False,parent_session_sha256=PARENT_SHA,parent_result_sha256=RESULT_SHA,
        parent_sequence=82,parent_adoption_token=PARENT_TOKEN,raw_review_sha256=RAW_REVIEW_SHA,
        additional_j5_tracking_violation_acknowledged=True,release_target_m=.055)
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Explicit release-only and additional J5 review required')
    require(isinstance(reviewed.get('user_authorization'),str)and reviewed['user_authorization'],'User authorization absent')
    images=reviewed.get('visual_evidence')
    require(isinstance(images,list)and images,'Saved table-support visual evidence required')
    for row in images:
        path=Path(row['path']);require(path.is_absolute()and path.is_file()and sha(path)==row['sha256'],'Visual evidence changed')
    result_path=PARENT_RUN/'runtime/action_000082_result.json'
    raw_path=PARENT_RUN/'seq82_raw_jaw_and_tracking_review.json'
    require(sha(result_path)==RESULT_SHA and sha(raw_path)==RAW_REVIEW_SHA,'Reviewed evidence changed')
    result=json.loads(result_path.read_text());review=json.loads(raw_path.read_text())
    identity=parent['identity'];state=parent['status']
    require(identity['source_sha256']==PREVIOUS_SHA and identity['config_sha256']==CONFIG_SHA
        and identity['adoption_token']==PARENT_TOKEN and parent['generation']==7,'Parent identity mismatch')
    require(state==result and state['sequence']==82 and state['phase']=='failed'and not state['active']
        and parent['failure']['error']==state['failure']=='Raw feedback violation latched: Jaw changed during joint command',
        'Unexpected parent failure')
    require(parent['raw_feedback_fault']==state['raw_feedback_fault']==review['original_failure']
        and review['jaw_before_m']==.05012 and review['jaw_at_fault_m']==.04949
        and review['jaw_limit_m']==.0005,'Wrong jaw failure evidence')
    receipts=state['receipts']
    require(len(receipts)==1 and receipts[0]==review['initial_socket_receipt']
        and receipts[0]['kind']=='initial'and receipts[0]['target_raw']==PARENT_TARGET
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4
        and parent['pending']==dict(sequence=82,kind='initial',target_raw=PARENT_TARGET,attempted_frames=4),
        'Partial or different parent transaction')
    violations=review['all_joint_tracking_violations']
    require(len(violations)==7 and all(v['axis']==5 and v['after_original_fault']for v in violations)
        and max(v['excess_mdeg']for v in violations)==320 and not review['nominal_violations'],
        'Additional J5 fault must remain explicitly unresolved')
    for window in review['source_windows']:
        path=Path(window['path']);require(path.parent.parent==PARENT_RUN,'Unexpected raw source')
        length=window['last_byte_exclusive']-window['first_byte']
        require(0<length<20000000,'Invalid raw evidence window')
        with path.open('rb')as stream:stream.seek(window['first_byte']);raw=stream.read(length)
        digest=hashlib.sha256();count=0
        for line in raw.splitlines(keepends=True):
            if json.loads(line).get('event')=='frame':digest.update(line);count+=1
        require(digest.hexdigest()==window['selected_complete_lines_sha256']and count==window['selected_frames'],
            'Independent recorded frame evidence changed')
    steady=review['steady_after_send_plus3s'];final=review['final'];q=final['q_raw']
    require(review['status_payloads']==['0100010000000000']and review['jaw_code_values']==[64]
        and all(v==[64]for v in review['motor_codes'].values())and len(review['motor_codes'])==6
        and steady['last']-steady['first']>=3 and steady['samples']>=20
        and max((hi-lo)*support.RAD_PER_RAW for lo,hi in steady['joint_ranges_raw'])<=.003
        and steady['max_error_to_original_goal_rad']<=.003 and steady['opening_range_m']==[.0483,.0483],
        'Independent settled endpoint not healthy/within original arrival limits')
    require(all(lo<=v<=hi for v,(lo,hi)in zip(q,support.JOINT_LIMITS_RAW))and final['opening_m']==.0483,
        'Independent endpoint violates nominal/jaw scope')
    return dict(raw_q=q,q=[v*support.RAD_PER_RAW for v in q],pose=final['pose'],opening_m=.0483,jaw_code=64)


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    require(boot==guard.predecessor.PARENT_BOOT,'Different boot requires separate review')
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    interior=previous.predecessor
    names=[boot+'.json',guard.predecessor.CHILD_NAME,'guarded_task_'+boot+'.json',
        interior.predecessor.CHILD_NAME,interior.CHILD_NAME,frozen.child_name(boot),previous.child_name(boot)]
    hashes=[guard.predecessor.PARENT_SHA,guard.SUCCESS_SESSION_SHA,interior.predecessor.PARENT_SHA,
        interior.PARENT_SHA,frozen.PARENT_SHA,previous.PARENT_SHA,PARENT_SHA]
    with ExitStack()as stack:
        originals=[]
        for name,digest in zip(names,hashes):
            path=root/name;require(path.is_file(),'Immutable parent absent')
            stack.enter_context(SessionFile(path));raw=path.read_bytes()
            require(hashlib.sha256(raw).hexdigest()==digest,'Immutable parent changed');originals.append(raw)
        parent=json.loads(originals[-1]);endpoint=reviewed_evidence(reviewed,parent)
        store=stack.enter_context(SessionFile(root/child_name(boot)))
        require(store.load()is None,'Release child already exists; no restart/output bypass')
        session=dict(stage='task',generation=8,generations=[],pending=None,failure=None,stop_latched=False,
            held_raw=None,commissioning_attempted=True,first_segment_verified=False,release_attempted=False,
            release_only=True,joint_motion_authorized=False,release_completed=False,
            parent_session_sha256=PARENT_SHA,parent_failure_chain_preserved=True,
            prior_failure=copy.deepcopy(parent['failure']),raw_review_sha256=RAW_REVIEW_SHA,
            additional_j5_tracking_violation_unresolved=True,feedback_excursion_physical_cause_resolved=False)
        store.save(session)
        try:yield parent,endpoint,store,session
        finally:require(all((root/name).read_bytes()==raw for name,raw in zip(names,originals)),'Old parent bytes changed')


class ReleaseTask(previous.J5Task):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.identity.pop('empty_gripper_operator_confirmed',None)
        self.identity.update(parent_sequence=82,parent_adoption_token=PARENT_TOKEN,
            j5_runtime_sha256=PREVIOUS_SHA,table_supported_contact_confirmed=True,
            release_only=True,joint_motion_authorized=False,additional_j5_tracking_violation_unresolved=True,
            reviewed_release_sha256=self.identity.get('reviewed_handoff_sha256'),
            table_support_basis='Reviewed saved RGB; not an empty-gripper assertion')
        self.state.update(release_only=True,joint_motion_authorized=False)

    def baseline(self):
        current=guard.v1.Interruptible.baseline(self)
        if self.handoff_pending:
            require(max(abs(q-t)*support.RAD_PER_RAW for q,t in zip(current['raw_q'],PARENT_TARGET))<=.003,
                    'Fresh handoff differs from original target arrival bounds')
            self.handoff_pending=False
        return current

    def execute(self,*a,**k):raise RuntimeError('Release-only: all joint motion remains unreviewed')
    def send_once(self,*a,**k):raise RuntimeError('Release-only: no joint transaction')
    def send_hold(self,*a,**k):raise RuntimeError('Release-only: no joint hold transaction')
    def resume(self,*a,**k):raise RuntimeError('Release-only: no resume or joint promotion')

    def gripper(self,request):
        require(type(request.gripper_angle)in(int,float)and request.gripper_angle==.055
            and type(request.gripper_effort)in(int,float)and request.gripper_effort==.2
            and type(request.gripper_code)is int and request.gripper_code==1
            and type(request.set_zero)is int and request.set_zero==0,'Only55mm effort.2 release is permitted')
        with self.state_lock:
            require(not self.session['release_attempted'],'Release opportunity already consumed; no retry')
            require(not self.session['failure']and not self.session['pending'],'Unresolved release session')
            self.session['release_attempted']=True;self.save()
        try:return guard.GuardedTask.gripper(self,request)
        except Exception as error:
            if not self.session['failure']:self.fail(error)
            raise

    def finish(self,phase,after,window,**extra):
        require(phase=='completed'and self.state['kind']=='gripper'and self.session['release_attempted'],
                'Only release completion is possible')
        require(abs(after['opening_m']-.055)<=.0015,'Release opening not reached; no retry')
        with self.state_lock:
            self.node.adopted=False
            self.session['stop_latched']=True;self.state['stop_latched']=True
            result=super().finish(phase,after,window,release_only=True,joint_motion_authorized=False,
                additional_j5_tracking_violation_unresolved=True,visual_release_review_required=True,**extra)
            self.session['release_completed']=True;self.save()
            return result

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(release_only=True,table_supported_contact_confirmed=True,joint_motion_authorized=False,
                          additional_j5_tracking_violation_unresolved=True,only_jaw_target_m=.055)
        return guard.v1.Interruptible.record(self,event,**fields)


class ReleaseParser(argparse.ArgumentParser):
    def add_argument(self,*names,**kwargs):
        if names==('--empty-gripper-confirmed',):names=('--table-supported-contact-confirmed',)
        return super().add_argument(*names,**kwargs)


def main(argv=None):
    require(sha(previous.__file__)==PREVIOUS_SHA and sha(frozen.__file__)==previous.RX_SHA
        and sha(frozen.overlay.__file__)==previous.RAW_OVERLAY_SHA and sha(profile.__file__)==previous.PROFILE_SHA,
        'Frozen dependency changed')
    check=argparse.ArgumentParser(add_help=False);check.add_argument('--entry-config',required=True)
    args,_=check.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    review=directory/'reviewed_release.json';alias=directory/'reviewed_handoff.json'
    require(review.read_bytes()==alias.read_bytes(),'Release review compatibility alias must be identical')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=ReleaseParser))
    launch=profile.private_function(frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=ReleaseTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=previous.PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)
    return launch(argv)


if __name__=='__main__':main()
