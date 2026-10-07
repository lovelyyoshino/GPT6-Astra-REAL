#!/usr/bin/env python3
"""One reviewed67mm lower opening request after the specific unmet70mm goal."""
import argparse
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_wide_entry as wide
previous=wide.previous;motion=wide.motion;guard=wide.guard;profile=wide.profile;support=wide.support
require=wide.require;sha=wide.sha;SessionFile=wide.SessionFile
CONFIG_SHA=wide.CONFIG_SHA
WIDE_SHA='879b398a8dca273ee60e0de4a0e87f9acd731f0da60d6ebe8fbca698679011ca'
PROFILE_SHA='fb13fa4969f222e320ce2a98589847cdef4e1fd27f805fe9476f2d0022fb0a05'
PARENT_SHA='96fdce8cae2ccb202a7c4e8766a520a4d09369c38390c3375d2cd66fabd93c22'
RESULT_SHA='69ab8d438d1d10640c510de576500f71ec078dc1f626ec524407a0bccf8e32b8'
RAW_SHA='3ac557fe3691d666feafcbff9a086bc5d9cb5792d4b83c4de16958a0d25d68aa'
PARENT_TOKEN='98c7c2b1e8bf49899b1fbe332f55554f'
PARENT_RUN=ROOT/'runs/cola_on_cup_wide70_20261006_202700'
OPENINGS={'opening67_ready':.067}
RANGE_QUERY_SHA=wide.RANGE_QUERY_SHA


def child_name(boot):return 'guarded_relief67_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    require(reviewed.get('relief67_reviewed')is True and reviewed.get('relief_target_m')==.067
        and reviewed.get('failed_wide_session_sha256')==PARENT_SHA
        and reviewed.get('failed_wide_result_sha256')==RESULT_SHA
        and reviewed.get('failed_wide_raw_review_sha256')==RAW_SHA,'Specific67mm lower-target review required')
    result_path=PARENT_RUN/'runtime/action_000002_result.json';raw_path=PARENT_RUN/'seq2_failed70_raw_review.json'
    require(sha(result_path)==RESULT_SHA and sha(raw_path)==RAW_SHA,'Unmet70mm evidence changed')
    state=json.loads(result_path.read_text());raw=json.loads(raw_path.read_text());identity=parent['identity']
    require(identity['source_sha256']==WIDE_SHA and identity['config_sha256']==CONFIG_SHA
        and identity['adoption_token']==PARENT_TOKEN and parent['generation']==10
        and parent['status']==state and state['sequence']==2 and state['phase']=='failed'
        and not state['active']and parent['failure']['error']==state['failure']==
        'Requested opening not reached; no automatic retry','Only this unmet-width failure is admissible')
    require(parent.get('raw_feedback_fault')is None and state.get('raw_feedback_fault')is None
        and state['latest_state'].get('raw_feedback_fault')is None,'Any RX safety failure forbids this relief')
    receipts=state['receipts']
    require(len(receipts)==1 and receipts[0]==raw['receipt']and receipts[0]['kind']=='gripper'
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==1
        and receipts[0]['jaw_target_m']==.07 and receipts[0]['frames']==[dict(id=345,data_hex='0001117000c80100')]
        and parent['pending']==dict(sequence=2,kind='gripper',target_m=.07,attempted_frames=1),'Incomplete/different parent transaction')
    require(raw['completed_result_sha256']==RESULT_SHA and raw['adoption_token']==PARENT_TOKEN
        and raw['result_phase']=='failed'and raw['review_pass']is False and raw['full_raw_reviewed']is True
        and raw['transport_clean']is True and raw['all_14_result_feedback_frames_matched_exactly']is True
        and all(raw[k]==[]for k in ('nominal_violations','joint_tracking_violations','jaw_guard_violations',
            'health_violations','transport_violations')),'Raw safety fault cannot become a width-reachability recovery')
    for item in raw['source_windows']:
        p=Path(item['path']);length=item['last_byte_exclusive']-item['first_byte']
        require(p.is_absolute()and 0<length<50000000,'Invalid raw window')
        with p.open('rb')as stream:stream.seek(item['first_byte']);data=stream.read(length)
        require(len(data)==length and hashlib.sha256(data).hexdigest()==item['sha256'],'Raw source changed')
    endpoint=copy.deepcopy(state['latest_state']);tail=raw['later_tail'];q=endpoint['raw_q']
    require(tail['last_kernel_unix_s']-tail['first_kernel_unix_s']>=3
        and tail['joint_ranges_raw']==[[v,v]for v in q]and tail['jaw_range_raw']==[67060,67060]
        and endpoint['opening_m']==.06706 and raw['max_joint_deviation_mdeg']==[0]*6
        and raw['max_observed_translation_m']==raw['max_observed_rotation_rad']==0,
        'Independent stationary67.06mm endpoint required')
    visual=reviewed['root_visual_review'];vp=Path(visual['path'])
    require(vp.is_absolute()and sha(vp)==visual['sha256'],'Visual review changed')
    view=json.loads(vp.read_text())
    require(view.get('reviewed')is True and view.get('table_supported')is True
        and view.get('no_visible_hook_or_tension')is True and view.get('held_load_verified')is False,
        'Reviewed table-supported contact scope required')
    require(view.get('user_authorization')and view.get('images'),'Visual authorization/images absent')
    for item in view['images']:
        p=Path(item['path']);require(p.is_absolute()and sha(p)==item['sha256'],'Current RGB changed')
    return endpoint


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    path=root/wide.child_name(boot)
    with SessionFile(path):
        original=path.read_bytes();require(hashlib.sha256(original).hexdigest()==PARENT_SHA,'Unmet-width parent changed')
        parent=json.loads(original);endpoint=reviewed_evidence(reviewed,parent)
        def legacy_evidence(material,older_parent):
            wide.reviewed_evidence(material,older_parent)  # Preserve all prior safety-failure review/bindings.
            return endpoint
        implementation=profile.private_function(previous.reserve.__wrapped__,child_name=child_name,reviewed_evidence=legacy_evidence)
        try:
            with contextmanager(implementation)(boot,reviewed,session_root=root)as(_,__,store,session):
                session.update(generation=11,regrasp_stage='opening67_ready',wide_open_attempted=False,
                    wide_opening_reviews=[],configured_gripper_max_mm=70,initial_opening_m=.067,
                    prior_unmet_width_failure=copy.deepcopy(parent['failure']),parent_wide_session_sha256=PARENT_SHA,
                    actual_target_changed_from_m=.07,physical_cause_unknown=True,first_segment_verified=False)
                store.save(session);yield parent,endpoint,store,session
        finally:require(path.read_bytes()==original,'Unmet-width parent bytes changed')


def relief_function(function):
    result=profile.private_function(function,OPENINGS=OPENINGS)
    old=('opening65_review','opening70_review')
    if old in function.__code__.co_consts:
        result.__code__=function.__code__.replace(co_consts=tuple(('opening67_review',)if x==old else x
            for x in function.__code__.co_consts))
    return result


class ReliefTask(wide.WideTask):
    gripper=relief_function(wide.WideTask.gripper)
    finish=relief_function(wide.WideTask.finish)
    review_opening=relief_function(wide.WideTask.review_opening)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.identity.update(parent_sequence=2,parent_generation=10,parent_adoption_token=PARENT_TOKEN,
            wide_runtime_sha256=WIDE_SHA,initial_scope='only67mm_lower_opening_then_review',initial_opening_m=.067,
            failed70_result_sha256=RESULT_SHA,failed70_raw_review_sha256=RAW_SHA,
            actual_target_changed_from_m=.07,physical_cause_unknown=True)

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None);fields.pop('first_segment_max_joint_deg',None)
            fields.update(initial_scope='only67mm_lower_opening_then_review',prior70_action_remains_failed=True,
                force_unchanged=True,physical_cause_unknown=True)
            return guard.v1.Interruptible.record(self,event,**fields)
        return super().record(event,**fields)


def main(argv=None):
    require(sha(wide.__file__)==WIDE_SHA and sha(profile.__file__)==PROFILE_SHA
        and sha(previous.__file__)==wide.PREVIOUS_SHA and sha(previous.release.__file__)==previous.RELEASE_SOURCE_SHA
        and sha(motion.__file__)==previous.MOTION_SOURCE_SHA
        and sha(profile.original.__file__)==motion.PROFILE_SHA and sha(motion.frozen.__file__)==motion.RX_SHA
        and sha(motion.frozen.overlay.__file__)==motion.RAW_OVERLAY_SHA,'Frozen relief dependency changed')
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--entry-config',required=True)
    args,_=parser.parse_known_args(argv);directory=Path(args.entry_config).resolve().parent
    require((directory/'reviewed_relief.json').read_bytes()==(directory/'reviewed_handoff.json').read_bytes(),'Relief review alias mismatch')
    facade=types.SimpleNamespace(**dict(vars(argparse),ArgumentParser=previous.release.ReleaseParser))
    return profile.private_function(motion.frozen.main,__file__=__file__,__doc__=__doc__,argparse=facade,
        RXGuardedTask=ReliefTask,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)(argv)


if __name__=='__main__':main()
