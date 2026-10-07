#!/usr/bin/env python3
"""One new current-anchor wrist qualification after a completed but unrecorded probe.

The old result stays completed with independent_raw_review_complete=False.
No old failure is cleared and no missing feedback is reconstructed or passed.
"""
import argparse
from contextlib import ExitStack, contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_relief_entry as predecessor
wide=predecessor.wide;previous=predecessor.previous;motion=predecessor.motion
profile=predecessor.profile;guard=predecessor.guard;support=predecessor.support
require=predecessor.require;sha=predecessor.sha;SessionFile=predecessor.SessionFile
CONFIG_SHA=predecessor.CONFIG_SHA;PROFILE_SHA=predecessor.PROFILE_SHA
PREDECESSOR_SHA='d6d7f4bc04952a808893051ad6d6267779ab5709007c83aa91faef7ee1193690'
PARENT_SHA='e3ebe631033dd3d003faf5a2c82fd57bdc2ed6a6c06c27d492a420843e9bd56a'
RESULT_SHA='79f8fc716837f155a226a8f015d5d697bfe95fa0960225eba8343b5dc2b76061'
GAP_SHA='d9768b0e9425dc90aa3444c87c0da1ea87c76bcea55eab3244a75d97703f1b0b'
INTENT_LINE_SHA='ad3bab4bf9522f4d04bca55ec98a9bd6c216b4a3b7d5bc8d34fa673a114fe17a'
PARENT_TOKEN='90e4d7816c4a4fd2a50871c2726ad5fe'
PARENT_RUN=ROOT/'runs/cola_on_cup_relief67_20261006_205300'
RANGE_QUERY_SHA=wide.RANGE_QUERY_SHA
OPENINGS={}  # This entry has no opening qualification stage.


def child_name(boot):return 'guarded_recorded_probe_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(recorded_probe_reviewed=True,completed_probe_session_sha256=PARENT_SHA,
        completed_probe_result_sha256=RESULT_SHA,missing_raw_diagnosis_sha256=GAP_SHA,
        prior_seq3_independent_raw_review_complete=False,one_new_probe_only=True,
        no_automatic_repeat_if_gap_fault_or_partial=True,new_probe_delta_raw=[0,0,0,0,-150,0])
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Specific new recorded-probe review required')
    result_path=PARENT_RUN/'runtime/action_000003_result.json';gap_path=PARENT_RUN/'seq3_independent_raw_gap.json'
    require(sha(result_path)==RESULT_SHA and sha(gap_path)==GAP_SHA,'Completed probe or gap diagnosis changed')
    result=json.loads(result_path.read_text());gap=json.loads(gap_path.read_text());state=parent['status'];identity=parent['identity']
    require(identity['source_sha256']==PREDECESSOR_SHA and identity['config_sha256']==CONFIG_SHA
        and identity['adoption_token']==PARENT_TOKEN and parent['generation']==13
        and parent['regrasp_stage']=='wrist_review'and parent['probe_attempted']is True
        and not parent['failure']and not parent['pending']and not parent['stop_latched']
        and state['sequence']==3 and state['phase']=='completed'and not state['active']and not state['failure']
        and state['result']==result and state['result_sha256']==RESULT_SHA,
        'Only the exact completed clean wrist parent is admissible')
    require(parent.get('raw_feedback_fault')is None and state.get('raw_feedback_fault')is None
        and result['after'].get('raw_feedback_fault')is None,'Any parent RX fault forbids requalification')
    receipts=state['receipts'];target=[33118,105285,-31513,0,-52747,0]
    require(len(receipts)==1 and receipts[0]['kind']=='initial'
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4
        and receipts[0]['target_raw']==target
        and result['phase']=='completed'and result['sequence']==3 and result['generation']==13
        and result['regrasp_probe_stage']=='wrist_ready'and result['probe_motion_evidence']['sufficient']is True,
        'Partial, failed or different previous action is inadmissible')
    # The joint receipt records counts/target, not frame payloads. Preserve that
    # distinction: bind its original intent line, without inventing a bus receipt.
    intents=[]
    with (PARENT_RUN/'runtime/feedback.jsonl').open('rb')as stream:
        for line in stream:
            if hashlib.sha256(line).hexdigest()==INTENT_LINE_SHA:intents.append(json.loads(line))
    require(len(intents)==1,'Original joint intent absent or changed')
    intent=intents[0]
    require(intent['event']=='intent'and intent['sequence']==3 and intent['kind']=='initial'
        and intent['target_raw']==target and intent['unix_s']<=receipts[0]['started_unix_s']
        and intent['frames']==[dict(id=i,data_hex=b.hex())for i,b in support.frames_for(target)],
        'Original intended frames mismatch; not an independent bus-delivery receipt')
    require(gap['reviewed']is True and gap['complete_independent_raw_missing']is True
        and gap['full_raw_reviewed']is False and gap['action_sequence']==3
        and gap['action_result_sha256']==RESULT_SHA and gap['runtime_action_completed']is True
        and gap['runtime_raw_feedback_fault']is None
        and gap['raw013_last_feedback']<receipts[0]['started_unix_s']
        and gap['raw014_first_started']>max(result['after']['stamps']),
        'Previous independent evidence gap must remain explicitly incomplete')
    item=reviewed['recorded_probe_visual_review'];path=Path(item['path'])
    require(path.is_absolute()and sha(path)==item['sha256'],'Current visual review changed')
    view=json.loads(path.read_text())
    require(all(view.get(k)is True for k in ('reviewed','object_contact_absent','table_supported',
        'no_visible_hook_or_tension','one_new_probe_only','no_automatic_repeat_if_gap_fault_or_partial'))
        and view.get('held_load_verified')is False and view.get('prior_seq3_independent_raw_review_complete')is False
        and view.get('user_authorization')and view.get('images'),'Current separated empty-gripper review required')
    for image in view['images']:
        p=Path(image['path']);require(p.is_absolute()and sha(p)==image['sha256'],'Current RGB changed')
    endpoint=copy.deepcopy(result['after'])
    require(all(lo<=q<=hi for q,(lo,hi)in zip(endpoint['raw_q'],support.JOINT_LIMITS_RAW))
        and endpoint['arm_status']==endpoint['motion_status']==endpoint['fault']==0
        and endpoint['driver_codes']==[64]*6 and endpoint['jaw_code']==64,'Completed endpoint not healthy nominal')
    return endpoint


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    with ExitStack()as stack:
        originals=[]
        for name,digest in ((wide.child_name(boot),predecessor.PARENT_SHA),(predecessor.child_name(boot),PARENT_SHA)):
            path=root/name;require(path.is_file(),'Reviewed parent absent');stack.enter_context(SessionFile(path))
            data=path.read_bytes();require(hashlib.sha256(data).hexdigest()==digest,'Reviewed parent changed')
            originals.append((path,data))
        parent=json.loads(originals[-1][1]);endpoint=reviewed_evidence(reviewed,parent)
        # Keep the former unmet-width and all earlier safety-failure evidence/bytes.
        predecessor.reviewed_evidence(reviewed,json.loads(originals[0][1]))
        def legacy_evidence(material,older_parent):
            wide.reviewed_evidence(material,older_parent)
            return endpoint
        implementation=profile.private_function(previous.reserve.__wrapped__,child_name=child_name,reviewed_evidence=legacy_evidence)
        try:
            with contextmanager(implementation)(boot,reviewed,session_root=root)as(_,__,store,session):
                session.update(generation=14,regrasp_stage='wrist_ready',probe_attempted=False,
                    retreat_completed=parent.get('retreat_completed',0),configured_gripper_max_mm=70,
                    prior_seq3_independent_raw_review_complete=False,
                    prior_seq3_result_remains_completed_but_evidence_incomplete=True,
                    prior_completed_probe_result_sha256=RESULT_SHA,missing_raw_diagnosis_sha256=GAP_SHA,
                    parent_completed_probe_session_sha256=PARENT_SHA,one_new_probe_only=True,
                    no_automatic_repeat_if_gap_fault_or_partial=True,
                    initial_scope='one_new_current_anchor_wrist_probe_then_full_raw_rgb_review')
                store.save(session);yield parent,endpoint,store,session
        finally:require(all(path.read_bytes()==data for path,data in originals),'Previous parent bytes changed')


class RecordedTask(wide.WideTask):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.identity.pop('initial_opening_m',None)
        self.identity.update(parent_sequence=3,parent_generation=13,parent_adoption_token=PARENT_TOKEN,
            recorded_probe_predecessor_sha256=PREDECESSOR_SHA,missing_raw_diagnosis_sha256=GAP_SHA,
            prior_seq3_independent_raw_review_complete=False,initial_no_contact_claim=True,
            contact_possible=False,initial_scope='one_new_current_anchor_wrist_probe_then_full_raw_rgb_review',
            purpose='new_qualification_not_reconstruction_of_missing_recording')

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(regrasp_stage=self.session['regrasp_stage'],new_probe_delta_raw=[0,0,0,0,-150,0],
                prior_seq3_independent_raw_review_complete=False,post_probe_full_raw_rgb_review_required=True,
                recording_liveness_does_not_guarantee_future_coverage=True)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


def main(argv=None):
    require(sha(predecessor.__file__)==PREDECESSOR_SHA and sha(wide.__file__)==predecessor.WIDE_SHA
        and sha(profile.__file__)==PROFILE_SHA and sha(previous.__file__)==wide.PREVIOUS_SHA
        and sha(previous.release.__file__)==previous.RELEASE_SOURCE_SHA and sha(motion.__file__)==previous.MOTION_SOURCE_SHA
        and sha(profile.original.__file__)==motion.PROFILE_SHA and sha(motion.frozen.__file__)==motion.RX_SHA
        and sha(motion.frozen.overlay.__file__)==motion.RAW_OVERLAY_SHA,'Frozen recorded-probe dependency changed')
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_recorded_probe.json').read_bytes()==(directory/'reviewed_handoff.json').read_bytes(),
        'Recorded-probe review alias mismatch')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=previous.release.ReleaseParser))
    return profile.private_function(motion.frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=RecordedTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)(argv)


if __name__=='__main__':main()
