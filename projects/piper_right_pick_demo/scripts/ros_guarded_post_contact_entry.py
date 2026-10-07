#!/usr/bin/env python3
"""Reviewed loaded-contact clearance lifts, then explicit evidence-bound task continuation."""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_contact_retreat_entry as predecessor
readiness=predecessor.predecessor
profile=predecessor.profile;guard=predecessor.guard;motion=predecessor.motion
support=predecessor.support;require=predecessor.require;sha=predecessor.sha;SessionFile=predecessor.SessionFile
CONFIG_SHA=predecessor.CONFIG_SHA;PROFILE_SHA=predecessor.PROFILE_SHA
PREDECESSOR_SHA='cfab5fcf874a116705ccdb9d2549385ab69f6a0a6a244bfd0851f5dd68c009d7'
PARENT_SHA='cafb578c2cb61f64a2d104ef7226d6ecf66c5a48a461494752e72781f547e882'
RESULT_SHA='bf7431e006edb22079586f964e6b52939d723e3835b964d2565ca1a303f3b64f'
RAW_REVIEW_SHA='f03b5ee8dfe6eeffbb3816f25ff2501f7b5b1a009c61db6647165acad192cd10'
VISUAL_SHA='2f764eeb76308962816a69dcefa7bd00f657890a9110d68ab0be631750e9fa7c'
PARENT_TOKEN='d1c4047cee2a43beadd593c7509ea252'
PARENT_RUN=ROOT/'runs/cola_on_cup_contact_retreat_20261006_225500'
TARGET=[53018,109860,-48249,0,-52889,0]
MAX_LIFTS=3


def child_name(boot):return 'guarded_post_contact_'+boot+'.json'


def verify_windows(items):
    require(isinstance(items,list)and items,'Full raw source windows required')
    for item in items:
        path=Path(item['path']);length=item['last_byte_exclusive']-item['first_byte']
        require(path.is_absolute()and 0<length<=50000000,'Invalid raw source byte window')
        with path.open('rb')as stream:stream.seek(item['first_byte']);data=stream.read(length)
        require(len(data)==length and hashlib.sha256(data).hexdigest()==item['sha256'],'Raw source changed')


def clean_raw(audit,sequence,generation,token,digest,receipt,result):
    require(audit.get('sequence')==sequence and audit.get('generation')==generation
        and audit.get('adoption_token')==token and audit.get('completed_result_sha256')==digest
        and audit.get('full_raw_reviewed')is True and audit.get('guard_checks_clean')is True
        and audit.get('transport_clean')is True
        and all(audit.get(key)==[]for key in ('nominal_violations','joint_tracking_violations','jaw_guard_violations','health_violations','transport_violations'))
        and audit.get('all_14_result_feedback_frames_matched_exactly')is True
        and audit['first_kernel_unix_s']<=receipt['started_unix_s']
        and audit['last_kernel_unix_s']>=max(result['after']['stamps'])
        and audit['target_raw']==receipt['target_raw'],'Independent full raw does not prove this clean finite segment')
    verify_windows(audit['source_windows'])


def reviewed_evidence(reviewed,parent):
    expected=dict(post_contact_reviewed=True,post_contact_parent_sha256=PARENT_SHA,
        post_contact_result_sha256=RESULT_SHA,post_contact_raw_review_sha256=RAW_REVIEW_SHA,
        prior_contact_failure_preserved=True,original_thresholds_unchanged=True,max_lifts=3,max_j2_decrease_mdeg=1300)
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Exact completed-retreat review required')
    path=PARENT_RUN/'runtime/action_000001_result.json';raw_path=PARENT_RUN/'seq1_contact_retreat_raw_review.json'
    require(sha(path)==RESULT_SHA and sha(raw_path)==RAW_REVIEW_SHA,'Completed retreat evidence changed')
    result=json.loads(path.read_text());audit=json.loads(raw_path.read_text());state=parent['status']
    require(parent['identity']['source_sha256']==PREDECESSOR_SHA and parent['identity']['config_sha256']==CONFIG_SHA
        and parent['identity']['adoption_token']==PARENT_TOKEN and parent['generation']==17
        and parent['contact_retreat_attempted']is True and parent['contact_retreat_completed']is True
        and parent['failure']is None and parent['pending']is None and parent['stop_latched']is True
        and parent.get('raw_feedback_fault')is None and state['phase']=='completed'and not state['active']
        and state['sequence']==1 and state['result']==result and state['result_sha256']==RESULT_SHA
        and parent['held_raw']==TARGET,'Only the exact clean completed retreat is admissible')
    receipts=state['receipts']
    require(len(receipts)==1 and receipts[0]['kind']=='initial'and receipts[0]['target_raw']==TARGET
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4
        and result['phase']=='completed'and result['stable_window']['duration_s']>=3
        and result['ordinary_task_authorized']is False,'Incomplete previous retreat')
    clean_raw(audit,1,17,PARENT_TOKEN,RESULT_SHA,receipts[0],result)
    item=reviewed['post_contact_visual_review'];p=Path(item['path'])
    require(p.is_absolute()and sha(p)==item['sha256']==VISUAL_SHA,'Loaded post-contact review changed')
    visual=json.loads(p.read_text())
    require(visual.get('reviewed')is True and visual.get('loaded_can')is True
        and visual.get('can_supported_by_cup')is False and visual.get('contact_possible')is True
        and visual.get('jaw_remains_closed')is True and visual.get('lifting_only_until_review')is True
        and visual.get('max_lifts')==3 and visual.get('max_j2_decrease_mdeg')==1300
        and visual.get('frozen_other_targets')==TARGET and visual.get('user_authorization'),'Wrong post-contact scope')
    require(visual.get('images'),'Current RGB absent')
    for image in visual['images']:
        p=Path(image['path']);require(p.is_absolute()and sha(p)==image['sha256'],'Post-contact RGB changed')
    return copy.deepcopy(result['after'])


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    path=root/predecessor.child_name(boot)
    with SessionFile(path):
        original=path.read_bytes();require(hashlib.sha256(original).hexdigest()==PARENT_SHA,'Completed retreat parent changed')
        parent=json.loads(original);endpoint=reviewed_evidence(reviewed,parent)
        def prior_evidence(material,older_parent):
            predecessor.reviewed_evidence(material,older_parent)
            return endpoint
        implementation=profile.private_function(predecessor.reserve.__wrapped__,child_name=child_name,reviewed_evidence=prior_evidence)
        try:
            with contextmanager(implementation)(boot,reviewed,session_root=root)as(_,__,store,session):
                session.update(generation=18,regrasp_stage='task',contact_stage='lifting',stage='task',
                    stop_latched=False,first_segment_verified=True,lifting_completed=0,lifting_attempted=False,
                    scope_anchor_raw=list(TARGET),held_raw=list(TARGET),post_contact_reviews=[],
                    original_contact_failure=copy.deepcopy(parent['loaded_contact_failure']),
                    original_contact_raw_fault=copy.deepcopy(parent['loaded_contact_raw_fault']),
                    clean_contact_retreat_result_sha256=RESULT_SHA,initial_scope='reviewed_loaded_clearance',
                    can_held_by_gripper=True,can_supported_by_cup=False,ordinary_task_authorized=False,jaw_authorized=False)
                store.save(session);yield parent,endpoint,store,session
        finally:require(path.read_bytes()==original,'Completed retreat parent bytes changed')


def check_lift(raw,before,session):
    anchor=session['scope_anchor_raw']
    require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(anchor,before['raw_q']))<=.003,'Lift anchor drift')
    require(all(raw[i]==TARGET[i]for i in (0,2,3,4,5)),'Five non-J2 targets must remain at the reviewed nominal values')
    require(0<anchor[1]-raw[1]<=1300 and 0<before['raw_q'][1]-raw[1]<=1300,'Only J2 decrease up to1300millidegrees is allowed')


def scoped_target(message,before,session):
    raw=guard.decode(message,before,session)
    stage=session['contact_stage']
    if stage=='lifting':
        require(session['lifting_completed']<MAX_LIFTS and not session['lifting_attempted'],'Lift budget/opportunity exhausted')
        check_lift(raw,before,session)
    else:require(stage=='task','New RGB and complete raw review required')
    require(abs(raw[4]-before['raw_q'][4])<=200,'J5 command increment exceeds200millidegrees')
    return raw


def review_material(path,sequence,generation,digest,token,receipt,result):
    data=json.loads(Path(path).read_text())
    expected=dict(reviewed=True,stage='post_contact_lift',sequence=sequence,generation=generation,
        adoption_token=token,completed_result_sha256=digest,prior_failures_preserved=True,grasp_retained=True)
    require(all(data.get(k)==v for k,v in expected.items()),'Wrong or incomplete loaded-lift review')
    require(data.get('next_stage')in ('lifting','task')and data.get('review_authorization'),'Explicit next-stage review required')
    if data['next_stage']=='task':
        require(data.get('clearance_visible')is True and data.get('object_contact_absent')is True,'Task requires visible clearance and retained grasp')
    require(data.get('images'),'New post-lift RGB required')
    for image in data['images']:
        p=Path(image['path']);require(p.is_absolute()and sha(p)==image['sha256']
            and image['captured_at_unix_s']>=max(result['after']['stamps']),'RGB changed or predates completed lift')
    item=data['raw_review'];p=Path(item['path']);require(p.is_absolute()and sha(p)==item['sha256'],'Raw review changed')
    clean_raw(json.loads(p.read_text()),sequence,generation,token,digest,receipt,result)
    return data


class PostContactTask(readiness.ReadinessTask):
    execute=profile.private_function(guard.GuardedTask.execute,support=support,decode=scoped_target)

    def __init__(self,*a,**k):
        super().__init__(*a,**k)
        for key in ('empty_gripper_operator_confirmed','table_supported_contact_confirmed'):
            self.identity.pop(key,None)
        self.identity.update(post_contact_predecessor_sha256=PREDECESSOR_SHA,post_contact_raw_review_sha256=RAW_REVIEW_SHA,
            parent_sequence=1,parent_generation=17,parent_adoption_token=PARENT_TOKEN,
            initial_scope='reviewed_loaded_clearance',can_held_by_gripper=True,can_supported_by_cup=False,
            contact_possible=True,table_supported_object='transparent cup only',max_lifts=3,max_j2_decrease_mdeg=1300,
            frozen_other_targets=list(TARGET),ordinary_task_requires_clearance_review=True)
        self.state.update(contact_stage=self.session['contact_stage'],contact_review_service=None)

    def baseline(self):
        # Parent startup may initialize scope_anchor from feedback. The nominal
        # five held targets must never drift with that feedback or later reviews.
        anchor=list(self.session['scope_anchor_raw'])
        sample=super().baseline()
        if self.session['contact_stage']!='task':
            self.session['scope_anchor_raw']=anchor
            require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(sample['raw_q'],anchor))<=.003,'Loaded lift endpoint differs from reviewed nominal target')
            require(sample['jaw_code']==64 and abs(sample['opening_m']-.05747)<=.0005,'Held-load jaw differs from reviewed57.47mm baseline')
        return sample

    def send_once(self,raw,measured,origin,target,kind,*,moving):
        if self.session['contact_stage']!='task':
            require(kind=='initial','No replacement target during clearance lift')
            message=type('Message',(),dict(position=[v*support.RAD_PER_RAW for v in raw],velocity=[0]*6+[1],effort=[]))()
            require(scoped_target(message,measured,self.session)==raw,'Lift target changed before send')
            with self.state_lock:self.session['lifting_attempted']=True;self.save()
        return super().send_once(raw,measured,origin,target,kind,moving=moving)

    def fail(self,error):
        if self.session['contact_stage']!='task':
            return motion.frozen.RXGuardedTask.fail(self,error)
        return super().fail(error)

    def monitor(self,s,origin,target):
        super().monitor(s,origin,target)
        # The frozen sender calls monitor with its final, strictly advanced
        # fresh snapshot after durable intent and immediately before ticketing.
        if (self.session['contact_stage']=='lifting'and self.state['phase']=='sending'
            and not self.state['initial_transaction_started']):
            check_lift(self.state['target_raw'],s,self.session)

    def request_hold(self,*a,**k):
        if self.session['contact_stage']!='task':return False,dict(error='Finite clearance lift only; no replacement target',hold_confirmed=False)
        return super().request_hold(*a,**k)
    def send_hold(self,*a,**k):
        require(self.session['contact_stage']=='task','No hold replacement during finite clearance lift')
        return super().send_hold(*a,**k)
    def gripper(self,*a,**k):
        require(self.session['contact_stage']=='task','Held load: jaw forbidden until reviewed visible clearance')
        return super().gripper(*a,**k)
    def resume(self,*a,**k):
        require(self.session['contact_stage']=='task','Clearance lift requires its evidence-bound review')
        return super().resume(*a,**k)
    def review_probe(self,*a,**k):raise RuntimeError('Use the post-contact review endpoint')
    def review_opening(self,*a,**k):raise RuntimeError('No opening review in this scope')

    def finish(self,phase,after,window,**extra):
        if self.session['contact_stage']=='task':return super().finish(phase,after,window,**extra)
        require(phase=='completed'and self.session['contact_stage']=='lifting','Only finite lift completion can be reviewed')
        with self.state_lock:
            self.node.adopted=False
            result=super().finish(phase,after,window,post_contact_lift=True,jaw_command_not_sent=True,
                post_lift_grasp_verified=False,clearance_verified=False,ordinary_task_authorized=False,**extra)
            self.session['lifting_completed']+=1
            self.session['scope_anchor_raw']=list(self.state['target_raw'])
            self.session['contact_stage']='lift_review';self.state['contact_stage']='lift_review'
            seq=self.state['sequence'];generation=self.session['generation'];digest=self.state['result_sha256']
            name=self.namespace+'/review_contact/seq_%d_gen_%d_%s_%s'%(seq,generation,self.token,digest)
            self.state['contact_review_service']=name
            self.retained_services.append(self.service_factory(name,lambda:self.review_contact(seq,generation,digest)))
            self.save();return result

    def review_contact(self,sequence,generation,digest):
        require(self.node.action_lock.acquire(False),'Action still active');admitted=False
        try:
            with self.state_lock:
                require(self.session['contact_stage']=='lift_review'and self.state['phase']=='completed'
                    and not self.state['active']and not self.session['failure']and not self.session['pending']
                    and not self.session['stop_latched']and not self.node.failed and not self.node.piper.broken,'Only a clean completed lift may be reviewed')
                require(sequence==self.state['sequence']and generation==self.session['generation']and digest==self.state['result_sha256'],'Retired contact review')
                result=self.state['result'];receipts=self.state['receipts']
                require(len(receipts)==1 and receipts[0]['kind']=='initial'
                    and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4,'Complete lift receipt missing')
                require(sha(self.output/('action_%06d_result.json'%sequence))==digest,'Lift result changed')
                path=self.output/'reviews'/('action_%06d_review.json'%sequence)
                material=review_material(path,sequence,generation,digest,self.token,receipts[0],result)
                next_stage=material['next_stage']
                require(next_stage!='lifting'or self.session['lifting_completed']<MAX_LIFTS,'Three-lift ceiling reached')
                self.node.active=True;admitted=True
            self.ownership_check();fresh=self.baseline();guard.slip(result['after'],fresh)
            require(fresh['jaw_code']==result['after']['jaw_code']and abs(fresh['opening_m']-result['after']['opening_m'])<=.0005,'Jaw changed since lift')
            self.node.piper.healthy(fresh);require(not self.node.rospy_probe.is_shutdown(),'Shutdown during review')
            with self.state_lock:
                require(not self.session['failure']and not self.node.piper.broken,'Fault while reviewing')
                self.node.piper.rx_latch.assert_clean()
                self.session['post_contact_reviews'].append(dict(sequence=sequence,generation=generation,result_sha256=digest,
                    review_sha256=sha(path),review=material,fresh_state=fresh))
                self.session.update(contact_stage=next_stage,generation=generation+1,lifting_attempted=False,
                    ordinary_task_authorized=next_stage=='task',jaw_authorized=next_stage=='task')
                self.original=None;self.bound_hold=None
                self.state.update(contact_stage=next_stage,generation=generation+1,phase='idle',contact_review_service=None,
                    result=None,result_sha256=None,receipts=[],before=None,target_raw=None,requested_raw=None)
                self.save();self.node.adopted=True
            return True,self.status()
        except Exception as error:
            if admitted:self.fail(error)
            raise
        finally:self.node.active=False;self.node.action_lock.release()

    def record(self,event,**fields):
        if event=='ready':
            fields.update(initial_scope='reviewed_loaded_clearance',contact_stage=self.session['contact_stage'],
                loaded_can=True,can_supported_by_cup=False,contact_possible=True,original_seq60_stays_failed=True,
                max_lifts=3,max_j2_decrease_mdeg=1300,ordinary_task_authorized=False,jaw_authorized=False)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


def main(argv=None):
    require(sha(predecessor.__file__)==PREDECESSOR_SHA,'Frozen retreat source changed')
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_post_contact.json').read_bytes()==(directory/'reviewed_contact_retreat.json').read_bytes(),
        'Post-contact compatibility alias mismatch')
    return profile.private_function(predecessor.main,__file__=__file__,__doc__=__doc__,ContactRetreatTask=PostContactTask,
        reserve=reserve,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)(argv)


if __name__=='__main__':main()
