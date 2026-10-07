#!/usr/bin/env python3
"""Reviewed70mm software scope: first65mm, then explicit raw/RGB review."""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_regrasp_entry as previous
import gripper70_profile as profile
motion=previous.motion;guard=previous.guard;support=profile.support_view()
require=previous.require;sha=previous.sha;SessionFile=previous.SessionFile
PREVIOUS_SHA='f3027c1976a907b5f82e626b70de32e5547ac918a6eb4081e777d24b5c6d6737'
CONFIG_SHA='ce435a54f59285ee6d7f5479582f5503fdbf3264f8b70d98cd6d8229e3a4e632'
RANGE_QUERY_SHA='785dbe5b36d1a67cff354217a337c761084f22c269b964e43ef97695adc764ea'
OPENINGS={'opening65_ready':.065,'opening70_ready':.070}


def child_name(boot):return 'guarded_wide70_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    require(reviewed.get('wide_opening_reviewed')is True
        and reviewed.get('configured_gripper_max_mm')==70 and reviewed.get('initial_opening_m')==.065,
        'Explicit70mm scope and initial65mm review required')
    item=reviewed['range_query'];path=Path(item['path'])
    require(path.is_absolute()and sha(path)==item['sha256']==RANGE_QUERY_SHA,'Range query evidence changed')
    data=json.loads(path.read_text())
    require(data['channel']=='can1'and data['query_id']=='0x477'
        and data['query_payload_hex']=='0400000003000000'and data['reply_id']=='0x47e'
        and data['reply_payload_hex']=='6446050000000000'and data['max_range_config_mm']==70
        and data['parameters_changed']is False and data['actuator_commands_sent']==0,
        'Not the reviewed read-only configured range reply')
    # The inherited retreat-only field describes the earlier separation review,
    # not this entry's initial action. Its current scope is explicitly first65mm.
    return previous.reviewed_evidence(reviewed,parent)


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    implementation=profile.private_function(previous.reserve.__wrapped__,
        child_name=child_name,reviewed_evidence=reviewed_evidence)
    with contextmanager(implementation)(boot,reviewed,session_root=session_root)as(parent,endpoint,store,session):
        session.update(regrasp_stage='opening65_ready',wide_open_attempted=False,
            wide_opening_reviews=[],configured_gripper_max_mm=70,initial_opening_m=.065,
            initial_scope='reviewed65mm_opening_not_retreat',range_query_sha256=RANGE_QUERY_SHA)
        store.save(session)
        yield parent,endpoint,store,session


def opening_review_material(path,stage,sequence,generation,digest,token,receipt,result):
    data=json.loads(Path(path).read_text())
    expected=dict(reviewed=True,stage=stage,sequence=sequence,generation=generation,
        adoption_token=token,completed_result_sha256=digest,prior_failures_preserved=True,
        additional_j5_violation_not_reclassified=True,actual_opening_reviewed=True,
        table_supported=True,no_visible_hook_or_tension=True)
    require(all(data.get(k)==v for k,v in expected.items()),'Wrong or incomplete wide-opening review')
    allowed=('opening70_ready','retreat_ready')if stage=='opening65_review'else('retreat_ready',)
    require(data.get('next_stage')in allowed,'Opening review cannot skip separation/probe qualification')
    require(isinstance(data.get('review_authorization'),str)and data['review_authorization'],'Explicit review authorization required')
    raw=data['raw_review'];images=data['images']
    require(isinstance(images,list)and images,'New post-opening RGB required')
    for item in [raw]+images:
        p=Path(item['path']);require(p.is_absolute()and sha(p)==item['sha256'],'Opening evidence changed')
    evidence=json.loads(Path(raw['path']).read_text())
    require(evidence.get('sequence')==sequence and evidence.get('adoption_token')==token
        and evidence.get('completed_result_sha256')==digest and evidence.get('full_raw_reviewed')is True
        and evidence.get('transport_clean')is True and evidence.get('nominal_violations')==[]
        and evidence.get('joint_tracking_violations')==[] and evidence.get('jaw_guard_violations')==[]
        and evidence.get('health_violations')==[] and evidence['first_kernel_unix_s']<=receipt['started_unix_s']
        and evidence['last_kernel_unix_s']>=max(result['after']['stamps']),
        'Independent raw review does not cover a clean opening')
    sources=evidence.get('source_windows');require(isinstance(sources,list)and sources,'Bound raw windows required')
    for item in sources:
        p=Path(item['path']);length=item['last_byte_exclusive']-item['first_byte']
        require(p.is_absolute()and 0<length<=50000000,'Invalid raw window')
        with p.open('rb')as stream:stream.seek(item['first_byte']);content=stream.read(length)
        require(len(content)==length and hashlib.sha256(content).hexdigest()==item['sha256'],'Raw opening source changed')
    return data


class WideTask(previous.RegraspTask):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        require(self.limits['gripper_max_m']==.070,'Wide runtime requires70mm config')
        self.identity.update(wide_profile_sha256=sha(profile.__file__),regrasp_source_sha256=PREVIOUS_SHA,
            tracking_profile_sha256=sha(profile.__file__),joint_tracking_source_sha256=motion.PROFILE_SHA,
            range_query_sha256=RANGE_QUERY_SHA,configured_gripper_max_mm=70,
            initial_scope='only65mm_opening_then_review',initial_opening_m=.065,
            legacy_retreat_only_field_scope='inherited parent separation review, not current opening permission')
        self.state['opening_review_service']=None

    def execute(self,*args,**kwargs):
        require(self.session['regrasp_stage']not in OPENINGS
            and not self.session['regrasp_stage'].startswith('opening'),'Opening review required before any joint action')
        return super().execute(*args,**kwargs)

    def send_once(self,*args,**kwargs):
        require(not self.session['regrasp_stage'].startswith('opening'),'No joint transaction during opening scope')
        return super().send_once(*args,**kwargs)

    def gripper(self,request):
        stage=self.session['regrasp_stage']
        if stage not in OPENINGS:return super().gripper(request)
        require(type(request.gripper_angle)in(int,float)and request.gripper_angle==OPENINGS[stage]
            and type(request.gripper_effort)in(int,float)and request.gripper_effort==.2
            and type(request.gripper_code)is int and request.gripper_code==1
            and type(request.set_zero)is int and request.set_zero==0,'Only the reviewed fixed opening is permitted')
        with self.state_lock:
            require(not self.session['wide_open_attempted']and not self.session['failure']and not self.session['pending'],
                    'Wide opening opportunity consumed or unresolved')
            self.session['wide_open_attempted']=True;self.save()
        try:return guard.GuardedTask.gripper(self,request)
        except Exception as error:
            if not self.session['failure']:self.fail(error)
            raise

    def finish(self,phase,after,window,**extra):
        stage=self.session['regrasp_stage']
        if stage not in OPENINGS:return super().finish(phase,after,window,**extra)
        require(phase=='completed'and self.state['kind']=='gripper','Only fixed opening completion permitted')
        require(abs(after['opening_m']-OPENINGS[stage])<=.0015,'Requested opening not reached; no automatic retry')
        with self.state_lock:
            self.node.adopted=False
            result=motion.J5Task.finish(self,phase,after,window,wide_opening_stage=stage,
                configured_range_not_mechanical_measurement=True,visual_opening_review_required=True,**extra)
            self.session['regrasp_stage']=stage.replace('_ready','_review')
            self.state['regrasp_stage']=self.session['regrasp_stage']
            seq=self.state['sequence'];gen=self.session['generation'];digest=self.state['result_sha256']
            name=self.namespace+'/review_opening/seq_%d_gen_%d_%s_%s'%(seq,gen,self.token,digest)
            self.state['opening_review_service']=name
            self.retained_services.append(self.service_factory(name,lambda:self.review_opening(seq,gen,digest)))
            self.save();return result

    def review_opening(self,sequence,generation,digest):
        require(self.node.action_lock.acquire(False),'Action still active')
        admitted=False
        try:
            with self.state_lock:
                stage=self.session['regrasp_stage']
                require(stage in ('opening65_review','opening70_review')and self.state['phase']=='completed'
                    and not self.state['active']and not self.session['failure']and not self.session['pending']
                    and not self.session['stop_latched']and not self.node.failed and not self.node.piper.broken,
                    'Only completed clean opening can be reviewed')
                require(sequence==self.state['sequence']and generation==self.session['generation']
                    and digest==self.state['result_sha256'],'Retired opening review')
                result=self.state['result'];receipts=self.state['receipts'];target=OPENINGS[stage.replace('_review','_ready')]
                require(result['jaw_target_reached']is True and abs(result['after']['opening_m']-target)<=.0015
                    and len(receipts)==1 and receipts[0]['kind']=='gripper'
                    and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==1
                    and receipts[0]['jaw_target_m']==target,'Complete matching opening receipt required')
                require(sha(self.output/('action_%06d_result.json'%sequence))==digest,'Opening result changed')
                path=self.output/'reviews'/('action_%06d_review.json'%sequence)
                material=opening_review_material(path,stage,sequence,generation,digest,self.token,receipts[0],result)
                self.node.active=True;admitted=True
            self.ownership_check();fresh=self.baseline();guard.slip(result['after'],fresh)
            require(fresh['jaw_code']==result['after']['jaw_code']and abs(fresh['opening_m']-result['after']['opening_m'])<=.0005,
                    'Jaw changed since reviewed opening')
            self.node.piper.healthy(fresh);require(not self.node.rospy_probe.is_shutdown(),'Shutdown during review')
            with self.state_lock:
                require(not self.session['failure']and not self.node.piper.broken,'Fault while reviewing')
                self.node.piper.rx_latch.assert_clean();next_stage=material['next_stage']
                self.session['wide_opening_reviews'].append(dict(stage=stage,sequence=sequence,result_sha256=digest,
                    review_sha256=sha(path),review=material,fresh_state=fresh))
                self.session.update(regrasp_stage=next_stage,generation=generation+1,
                    wide_open_attempted=False,scope_anchor_raw=list(fresh['raw_q']),held_raw=list(fresh['raw_q']))
                self.original=None;self.bound_hold=None
                self.state.update(regrasp_stage=next_stage,generation=generation+1,phase='idle',opening_review_service=None,
                    result=None,result_sha256=None,receipts=[],before=None,target_raw=None,requested_raw=None)
                self.save();self.node.adopted=True
            return True,self.status()
        except Exception as error:
            if admitted:self.fail(error)
            raise
        finally:self.node.active=False;self.node.action_lock.release()

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(initial_scope='only65mm_opening_then_independent_review',configured_gripper_max_mm=70,
                prior_release_action_remains_failed=True,contact_possible=True)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


def main(argv=None):
    require(sha(previous.__file__)==PREVIOUS_SHA and sha(previous.release.__file__)==previous.RELEASE_SOURCE_SHA
        and sha(motion.__file__)==previous.MOTION_SOURCE_SHA and sha(profile.original.__file__)==motion.PROFILE_SHA
        and sha(motion.frozen.__file__)==motion.RX_SHA and sha(motion.frozen.overlay.__file__)==motion.RAW_OVERLAY_SHA,
        'Frozen wide-scope dependency changed')
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);config=Path(args.entry_config).resolve();directory=config.parent
    require(sha(config)==CONFIG_SHA,'Reviewed wide config changed')
    candidate=json.loads(config.read_text());narrow=copy.deepcopy(candidate);narrow['physical_limits']['gripper_max_m']=.055
    old=previous.PARENT_RUN/'entry_config.json'
    require(sha(old)==previous.CONFIG_SHA and narrow==json.loads(old.read_text()),'Only gripper maximum may change')
    require((directory/'reviewed_wide.json').read_bytes()==(directory/'reviewed_handoff.json').read_bytes(),
            'Wide review compatibility alias must be identical')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=previous.release.ReleaseParser))
    launch=profile.private_function(motion.frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=WideTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=sha(profile.__file__),
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=previous.RELEASE_SESSION_SHA,RESULT_SHA=previous.RELEASE_RESULT_SHA,
        PARENT_TOKEN=previous.RELEASE_TOKEN)
    return launch(argv)


if __name__=='__main__':main()
