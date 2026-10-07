#!/usr/bin/env python3
"""Bounded local regrasp scope with two separately reviewed recovery probes.

Startup binds the real failed release and independently settled endpoint.
Nothing here schedules a probe, retries a command or clears a parent.
"""
import argparse
from contextlib import ExitStack,contextmanager
import copy
import hashlib
import types
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_j5_entry as motion
import ros_guarded_release_entry as release
profile=motion.profile;guard=motion.guard;support=motion.support
require=motion.require;sha=motion.sha;SessionFile=motion.SessionFile
RELEASE_SOURCE_SHA='c6592256a7c7619fda8172431b5b0653a62b7064d834806cfd1da9dc1dafcef3'
MOTION_SOURCE_SHA='b9ebb31e3ab791e3844237b1d679cd66e7ad5b540f900bb982bcc9cabb0b2273'
RELEASE_SESSION_SHA='0bf8bdacdd96f7211794d5bf1b0962001a53b97759570fafd5b2c9fc02feb85e'
RELEASE_RESULT_SHA='66ea14be1661ff18a9ba8c821ba0fd4e2952891c93526fcebf28fe4a0d2efc1a'
RELEASE_TOKEN='3e49b255e26d489f832802636699b450'
RAW_REVIEW_SHA='ae44917f012e177dbcb8cd8924a8de0bc37823afd5e602433ae742bdc13f6d95'
CONFIG_SHA=motion.CONFIG_SHA
PARENT_RUN=ROOT/'runs/cola_on_cup_contact_release_20261006_195000'
MAX_RETREAT_SEGMENTS=8
DELTAS={'retreat_ready':[0,-350,350,0,0,0],'wrist_ready':[0,0,0,0,-150,0]}


def scoped_target(message,before,session):
    raw=guard.decode(message,before,session)
    stage=session['regrasp_stage']
    if stage in DELTAS:
        if stage=='retreat_ready':
            require(session.get('retreat_completed',0)<MAX_RETREAT_SEGMENTS,'Reviewed retreat budget exhausted')
        anchor=session['scope_anchor_raw']
        require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(anchor,before['raw_q']))<=.003,
                'Probe anchor drift')
        require(raw==[a+d for a,d in zip(anchor,DELTAS[stage])], 'Only the current fixed relative probe is permitted')
        require(not session.get('probe_attempted'),'Probe opportunity already consumed')
    else:require(stage=='task','Independent probe review required before any next action')
    require(abs(raw[4]-before['raw_q'][4])<=200,'J5 command increment exceeds200millidegrees')
    return raw


def probe_progress(stage,before,after):
    axes=(1,2)if stage=='retreat_ready'else(4,)
    delta=DELTAS[stage]
    progress={str(i+1):(after['raw_q'][i]-before['raw_q'][i])*(1 if delta[i]>0 else -1)/1000 for i in axes}
    threshold=.15 if stage=='retreat_ready'else .10
    return dict(progress_deg=progress,minimum_actual_progress_deg=threshold,
                sufficient=all(v>=threshold for v in progress.values()))


def review_material(path,stage,sequence,generation,digest,token,receipt,result):
    data=json.loads(Path(path).read_text())
    expected=dict(reviewed=True,stage=stage,sequence=sequence,generation=generation,
        adoption_token=token,completed_result_sha256=digest,prior_failures_preserved=True,
        additional_j5_violation_not_reclassified=True)
    require(all(data.get(k)==v for k,v in expected.items()),'Wrong or incomplete probe review')
    next_stage=data.get('next_stage')
    if stage=='retreat_review':
        require(next_stage in ('retreat_ready','wrist_ready'),'Explicit retreat continuation or wrist review decision required')
        if next_stage=='retreat_ready':
            require(data.get('table_supported')is True and data.get('no_visible_hook_or_tension')is True,
                    'Continued separation requires reviewed table support without visible hook/tension')
        else:require(data.get('object_contact_absent')is True,'Wrist probe requires reviewed separation')
    else:
        require(stage=='wrist_review'and next_stage=='task'and data.get('object_contact_absent')is True,
                'Task admission requires clean wrist probe and reviewed separation')
    require(isinstance(data.get('review_authorization'),str)and data['review_authorization'],'Explicit review authorization required')
    raw=data['raw_review'];images=data['images']
    require(images and isinstance(images,list),'New post-probe RGB evidence required')
    for item in [raw]+images:
        p=Path(item['path']);require(p.is_absolute()and p.is_file()and sha(p)==item['sha256'],'Probe evidence changed')
    evidence=json.loads(Path(raw['path']).read_text())
    require(evidence.get('sequence')==sequence and evidence.get('adoption_token')==token
        and evidence.get('completed_result_sha256')==digest
        and evidence.get('full_raw_reviewed')is True and evidence.get('transport_clean')is True
        and evidence.get('nominal_violations')==[] and evidence.get('joint_tracking_violations')==[]
        and evidence.get('jaw_guard_violations')==[] and evidence.get('health_violations')==[]
        and evidence['first_kernel_unix_s']<=receipt['started_unix_s']
        and evidence['last_kernel_unix_s']>=min(result['after']['stamps']),
        'Independent raw review does not cover a clean probe')
    sources=evidence.get('source_windows')
    require(isinstance(sources,list)and sources,'Raw review must bind source byte windows')
    import hashlib
    for item in sources:
        p=Path(item['path']);length=item['last_byte_exclusive']-item['first_byte']
        require(p.is_absolute()and 0<length<=50000000,'Invalid source byte window')
        with p.open('rb')as stream:stream.seek(item['first_byte']);content=stream.read(length)
        require(len(content)==length and hashlib.sha256(content).hexdigest()==item['sha256'],'Raw probe source changed')
    return data


class RegraspTask(motion.J5Task):
    execute=profile.private_function(guard.GuardedTask.execute,support=support,decode=scoped_target)

    def __init__(self,*a,**k):
        super().__init__(*a,**k)
        self.identity.pop('empty_gripper_operator_confirmed',None)
        self.identity.update(regrasp_scope=True,j5_command_increment_max_mdeg=200,
            parent_sequence=1,parent_generation=8,parent_adoption_token=RELEASE_TOKEN,
            release_source_sha256=RELEASE_SOURCE_SHA,j5_runtime_sha256=MOTION_SOURCE_SHA,
            table_supported_contact_confirmed=True,contact_possible=True,initial_no_contact_claim=False,
            purpose='contact_separation_retreat_then_independent_probe_reviews',
            prior_jaw_and_j5_faults_preserved=True)
        self.state.update(regrasp_stage=self.session['regrasp_stage'],probe_review_service=None)

    def baseline(self):
        current=guard.v1.Interruptible.baseline(self)
        if self.handoff_pending:
            self.session['scope_anchor_raw']=list(current['raw_q']);self.handoff_pending=False
        return current

    def send_once(self,raw,measured,origin,target,kind,*,moving):
        if kind=='initial':
            stage=self.session['regrasp_stage']
            require(stage in DELTAS or stage=='task','Probe review still pending')
            require(abs(raw[4]-measured['raw_q'][4])<=200,'J5 send increment exceeds200millidegrees')
            if stage in DELTAS:
                require(not self.session.get('probe_attempted'),'Probe already attempted')
                require(raw==[a+d for a,d in zip(self.session['scope_anchor_raw'],DELTAS[stage])], 'Probe target changed')
                self.session['probe_attempted']=True;self.save()
        return super().send_once(raw,measured,origin,target,kind,moving=moving)

    def finish(self,phase,after,window,**extra):
        stage=self.session['regrasp_stage']
        if stage in DELTAS:
            with self.state_lock:
                self.node.adopted=False
                evidence=probe_progress(stage,self.state['before'],after)
                if stage=='retreat_ready'and phase=='completed':
                    self.session['retreat_completed']=self.session.get('retreat_completed',0)+1
                result=super().finish(phase,after,window,regrasp_probe_stage=stage,
                    probe_motion_evidence=evidence,probe_review_required=True,**extra)
                self.session['regrasp_stage']=stage.replace('_ready','_review')
                self.state['regrasp_stage']=self.session['regrasp_stage']
                if phase=='completed'and evidence['sufficient']:
                    seq=self.state['sequence'];generation=self.session['generation'];digest=self.state['result_sha256']
                    name=self.namespace+'/review_probe/seq_%d_gen_%d_%s_%s'%(seq,generation,self.token,digest)
                    self.state['probe_review_service']=name
                    self.retained_services.append(self.service_factory(name,lambda:self.review_probe(seq,generation,digest)))
                self.save();return result
        return super().finish(phase,after,window,**extra)

    def review_probe(self,sequence,generation,digest):
        require(self.node.action_lock.acquire(False),'Action still active')
        admitted=False
        try:
            with self.state_lock:
                stage=self.session['regrasp_stage']
                require(stage in ('retreat_review','wrist_review')and self.state['phase']=='completed'
                    and not self.state['active']and not self.session['failure']and not self.session['pending']
                    and not self.session['stop_latched']and not self.node.failed and not self.node.piper.broken,
                    'Only a completed clean probe may be reviewed')
                require(sequence==self.state['sequence']and generation==self.session['generation']
                    and digest==self.state['result_sha256'],'Retired probe review')
                result=self.state['result'];receipt=self.state['receipts']
                require(result['probe_motion_evidence']['sufficient']and len(receipt)==1
                    and receipt[0]['kind']=='initial'and receipt[0]['attempted_frames']==receipt[0]['socket_send_returns']==4,
                    'Probe physical progress or complete receipt missing')
                require(sha(self.output/('action_%06d_result.json'%sequence))==digest,'Probe result changed')
                path=self.output/'reviews'/('action_%06d_review.json'%sequence)
                material=review_material(path,stage,sequence,generation,digest,self.token,receipt[0],result)
                next_stage=material['next_stage']
                if next_stage=='retreat_ready':
                    require(self.session.get('retreat_completed',0)<MAX_RETREAT_SEGMENTS,'Reviewed retreat budget exhausted')
                self.node.active=True;admitted=True
            self.ownership_check();fresh=self.baseline();guard.slip(result['after'],fresh)
            require(fresh['jaw_code']==result['after']['jaw_code']and abs(fresh['opening_m']-result['after']['opening_m'])<=.0005,'Jaw changed since probe')
            self.node.piper.healthy(fresh);require(not self.node.rospy_probe.is_shutdown(),'Shutdown during review')
            with self.state_lock:
                require(not self.session['failure']and not self.node.piper.broken,'Fault while reviewing')
                self.node.piper.rx_latch.assert_clean()
                self.session.setdefault('probe_reviews',[]).append(dict(stage=stage,sequence=sequence,
                    result_sha256=digest,review_sha256=sha(path),review=material,fresh_state=fresh))
                self.session.update(regrasp_stage=next_stage,generation=generation+1,scope_anchor_raw=list(fresh['raw_q']),
                    probe_attempted=False,held_raw=list(fresh['raw_q']))
                self.original=None;self.bound_hold=None
                self.state.update(regrasp_stage=next_stage,generation=generation+1,phase='idle',probe_review_service=None,
                    result=None,result_sha256=None,receipts=[],before=None,target_raw=None,requested_raw=None)
                self.save();self.node.adopted=True
            return True,self.status()
        except Exception as error:
            if admitted:self.fail(error)
            raise
        finally:self.node.active=False;self.node.action_lock.release()

    def resume(self,*a,**k):
        require(self.session['regrasp_stage']=='task','Probe sequence cannot use hold resume')
        return super().resume(*a,**k)

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(contact_possible=True,purpose='contact_separation_retreat',
                regrasp_stage=self.session['regrasp_stage'],retreat_delta_raw=DELTAS['retreat_ready'],
                prior_release_action_remains_failed=True,post_probe_reviews_required=True)
        return guard.v1.Interruptible.record(self,event,**fields)

    def gripper(self,request):
        require(self.session['regrasp_stage']=='task','Jaw forbidden until both independent probe reviews')
        return super().gripper(request)



def child_name(boot):return 'guarded_regrasp_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(reviewed=True,contact_separation_retreat_only=True,table_supported_contact_confirmed=True,
        initial_no_contact_claim=False,post_probe_reviews_required=True,parent_session_sha256=RELEASE_SESSION_SHA,
        parent_result_sha256=RELEASE_RESULT_SHA,parent_sequence=1,parent_generation=8,parent_adoption_token=RELEASE_TOKEN,
        raw_review_sha256=RAW_REVIEW_SHA,prior_jaw_and_joint_failures_preserved=True,
        retreat_delta_raw=DELTAS['retreat_ready'],held_jaw_m=.05439)
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Explicit failed-release contact-separation review required')
    require(isinstance(reviewed.get('user_authorization'),str)and reviewed['user_authorization'],'User authorization absent')
    images=reviewed.get('visual_evidence');require(isinstance(images,list)and images,'Table-support RGB review required')
    for image in images:
        path=Path(image['path']);require(path.is_absolute()and sha(path)==image['sha256'],'Visual evidence changed')
    result_path=PARENT_RUN/'runtime/action_000001_result.json'
    raw_path=PARENT_RUN/'release_failure_independent_review.json'
    require(sha(result_path)==RELEASE_RESULT_SHA and sha(raw_path)==RAW_REVIEW_SHA,'Release evidence changed')
    result=json.loads(result_path.read_text());rawreview=json.loads(raw_path.read_text());state=parent['status']
    identity=parent['identity']
    require(identity['source_sha256']==RELEASE_SOURCE_SHA and identity['config_sha256']==CONFIG_SHA
        and identity['adoption_token']==RELEASE_TOKEN and parent['generation']==8,'Release parent identity mismatch')
    require(parent['release_attempted']is True and parent['release_completed']is False
        and state==result and state['phase']=='failed'and not state['active']
        and parent['failure']['error']==state['failure']=='Raw feedback violation latched: Outside original joint box',
        'Real failed release must remain failed')
    require(parent['raw_feedback_fault']==rawreview['first_fault']and rawreview['first_fault']['axis']==5
        and rawreview['first_fault']['tracking_tolerance_rad']==.003,'Wrong release fault')
    receipts=state['receipts']
    require(len(receipts)==1 and receipts[0]['kind']=='gripper'and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==1
        and receipts[0]['jaw_target_m']==.055 and receipts[0]['frames']==[dict(id=0x159,data_hex='0000d6d800c80100')]
        and parent['pending']==dict(sequence=1,kind='gripper',target_m=.055,attempted_frames=1),
        'Partial/different release transaction')
    for key,value in rawreview['initial_receipt'].items():require(receipts[0][key]==value,'Release receipt mismatch')
    require(rawreview['result_sha256']==RELEASE_RESULT_SHA and rawreview['health']['nominal_violation_count']==0,
        'Release raw review identity or nominal health changed')
    for window in rawreview['raw_windows']:
        path=Path(window['path']);length=window['last_byte_exclusive']-window['first_byte']
        require(path.is_absolute()and 0<length<50000000,'Invalid release raw window')
        with path.open('rb')as stream:stream.seek(window['first_byte']);data=stream.read(length)
        digest=hashlib.sha256()
        for line in data.splitlines(keepends=True):
            if json.loads(line).get('event')=='frame':digest.update(line)
        require(digest.hexdigest()==window['selected_lines_sha256'],'Release raw source changed')
    tail=rawreview['later_raw007_observation'];health=tail['health'];last=tail['last'];q=last['raw_q']
    require(tail['last']['timestamp']-tail['first']['timestamp']>=3
        and tail['joint_ranges_raw']==[[v,v]for v in q]and tail['jaw_width_range_raw']==[54390,54390]
        and health['status_payloads']==['0100010000000000']and health['health_violation_count']==0
        and health['nominal_violation_count']==0 and health['max_per_id_gap_s']<=.1
        and len(health['motor_codes'])==6 and all(v==[64]for v in health['motor_codes'].values()),
        'Independent release tail not healthy stationary')
    require(all(lo<=v<=hi for v,(lo,hi)in zip(q,support.JOINT_LIMITS_RAW)),'Tail outside nominal bounds')
    return dict(raw_q=q,q=[v*support.RAD_PER_RAW for v in q],pose=last['pose'],opening_m=.05439,jaw_code=64)


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    require(boot==guard.predecessor.PARENT_BOOT,'Different boot requires separate review')
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    interior=motion.predecessor;rx=motion.frozen
    names=[boot+'.json',guard.predecessor.CHILD_NAME,'guarded_task_'+boot+'.json',interior.predecessor.CHILD_NAME,
        interior.CHILD_NAME,rx.child_name(boot),motion.child_name(boot),release.child_name(boot)]
    hashes=[guard.predecessor.PARENT_SHA,guard.SUCCESS_SESSION_SHA,interior.predecessor.PARENT_SHA,
        interior.PARENT_SHA,rx.PARENT_SHA,motion.PARENT_SHA,release.PARENT_SHA,RELEASE_SESSION_SHA]
    with ExitStack()as stack:
        originals=[]
        for name,digest in zip(names,hashes):
            path=root/name;require(path.is_file(),'Immutable parent absent')
            stack.enter_context(SessionFile(path));data=path.read_bytes()
            require(hashlib.sha256(data).hexdigest()==digest,'Immutable parent changed');originals.append(data)
        parent=json.loads(originals[-1]);endpoint=reviewed_evidence(reviewed,parent)
        store=stack.enter_context(SessionFile(root/child_name(boot)))
        require(store.load()is None,'Regrasp child already exists; no restart/output bypass')
        session=dict(stage='task',regrasp_stage='retreat_ready',generation=9,generations=[],pending=None,failure=None,
            stop_latched=False,held_raw=None,scope_anchor_raw=None,commissioning_attempted=True,
            first_segment_verified=False,probe_attempted=False,probe_reviews=[],retreat_completed=0,parent_failure_chain_preserved=True,
            prior_release_failure=copy.deepcopy(parent['failure']),raw_review_sha256=RAW_REVIEW_SHA,
            initial_contact_possible=True,feedback_excursion_physical_cause_resolved=False)
        store.save(session)
        try:yield parent,endpoint,store,session
        finally:require(all((root/name).read_bytes()==raw for name,raw in zip(names,originals)),'Old parent bytes changed')


def main(argv=None):
    require(sha(release.__file__)==RELEASE_SOURCE_SHA and sha(motion.__file__)==MOTION_SOURCE_SHA
        and sha(motion.frozen.__file__)==motion.RX_SHA and sha(motion.frozen.overlay.__file__)==motion.RAW_OVERLAY_SHA
        and sha(profile.__file__)==motion.PROFILE_SHA,'Frozen dependency changed')
    check=argparse.ArgumentParser(add_help=False);check.add_argument('--entry-config',required=True)
    args,_=check.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_regrasp.json').read_bytes()==(directory/'reviewed_handoff.json').read_bytes(),
            'Regrasp review compatibility alias must be identical')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=release.ReleaseParser))
    launch=profile.private_function(motion.frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=RegraspTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=motion.PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=RELEASE_SESSION_SHA,RESULT_SHA=RELEASE_RESULT_SHA,PARENT_TOKEN=RELEASE_TOKEN)
    return launch(argv)


if __name__=='__main__':main()
