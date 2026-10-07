#!/usr/bin/env python3
"""Reviewed seq62 handoff with one shared J5-only joint tracking profile."""
from contextlib import ExitStack,contextmanager
import copy
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import j5_tracking_profile as profile
frozen=profile.frozen;guard=frozen.guard;support=profile.support_view()
require=frozen.require;sha=frozen.sha;SessionFile=frozen.SessionFile
predecessor=frozen.predecessor
PREDECESSOR_SHA=frozen.PREDECESSOR_SHA
RX_SHA='7a73c848b02832c1c7f4a4bd9febcb35e3afd90be984065c76a6c0604c4ffcd6'
RAW_OVERLAY_SHA=frozen.OVERLAY_SHA
PROFILE_SHA='9b05d8a2b63901d878f4fdd1e75fbfcc950925dedaddfeac2c4e5b3012fc776f'
CONFIG_SHA='f9b686aa081e6c9233541a32ec98f4a3f8451666e17486b6a1818d56d1ad32e9'
PARENT_SHA='8171c711d7737fa1caebc7666d6784b538e826ed044d7bb8595c24cfb3e24de8'
RESULT_SHA='d9c077af1f8326b9f80eaffe93ef3798944d041735e5b278761356314ddaada6'
RAW_REVIEW_SHA='2eef5ebc88e1efcf16abe6e73a54134a5ed8e18cee94a308e8a80d3de118d471'
PARENT_TOKEN='f40cd4455bd146f6b84e2488572c07b0'
PARENT_RUN=ROOT/'runs/cola_on_cup_rxguard_20261006_180445'
PARENT_TARGET=[36800,64100,-7400,0,-35500,0]


def child_name(boot):return 'guarded_j5margin_'+boot+'.json'


def reviewed_evidence(reviewed,parent):
    expected=dict(reviewed=True,parent_session_sha256=PARENT_SHA,parent_result_sha256=RESULT_SHA,
        parent_sequence=62,parent_adoption_token=PARENT_TOKEN,raw_review_sha256=RAW_REVIEW_SHA,
        first_segment_max_joint_deg=.5,joint_tracking_margin_rad=list(profile.MARGINS))
    require(all(reviewed.get(k)==v for k,v in expected.items()),'Explicit seq62/profile review required')
    require(isinstance(reviewed.get('user_authorization'),str)and reviewed['user_authorization'],'User authorization absent')
    result_path=PARENT_RUN/'runtime/action_000062_result.json'
    raw_path=PARENT_RUN/'seq62_raw_overshoot_fk_review.json'
    require(reviewed.get('raw_review_path')==str(raw_path)and sha(raw_path)==RAW_REVIEW_SHA
        and sha(result_path)==RESULT_SHA,'Reviewed evidence changed')
    result=json.loads(result_path.read_text());review=json.loads(raw_path.read_text())
    identity=parent['identity'];state=parent['status']
    require(identity['source_sha256']==RX_SHA and identity['config_sha256']==frozen.CONFIG_SHA
        and identity['adoption_token']==PARENT_TOKEN and parent['generation']==6,'Parent identity mismatch')
    require(state==result and state['sequence']==62 and state['phase']=='failed'and not state['active']
        and parent['failure']['error']==state['failure']and parent['failure']['error']==
        'Raw feedback violation latched: Outside original joint box','Unexpected parent failure')
    fault=parent['raw_feedback_fault']
    require(fault==state['raw_feedback_fault']==review['raw_fault']and fault['axis']==5
        and fault['raw']==-35673 and fault['tracking_tolerance_rad']==.003
        and fault['context']['sequence']==62 and fault['context']['target_raw']==PARENT_TARGET,'Wrong raw failure')
    receipts=state['receipts'];pending=parent['pending']
    require(len(receipts)==1 and receipts[0]==review['initial_socket_receipt']
        and receipts[0]['kind']=='initial'and receipts[0]['target_raw']==PARENT_TARGET
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4
        and pending==dict(sequence=62,kind='initial',target_raw=PARENT_TARGET,attempted_frames=4),
        'Partial or different parent transaction')
    require(review['result_sha256']==RESULT_SHA and review['result_path']==str(result_path)
        and review['status_payloads']==['0100010000000000']
        and set(review['motor_codes'])=={hex(i)for i in range(0x261,0x267)}
        and all(c==[64]for c in review['motor_codes'].values()),'Independent review unhealthy')
    source=Path(review['source_path'])
    require(source==PARENT_RUN/'raw_can_003/frames.jsonl','Wrong independent raw stream')
    length=review['source_last_byte_exclusive']-review['source_first_byte']
    require(0<length<20000000,'Invalid raw review byte window')
    with source.open('rb')as stream:
        stream.seek(review['source_first_byte']);data=stream.read(length)
    require(len(data)==length and hashlib.sha256(data).hexdigest()==review['source_window_sha256'],
        'Independent raw evidence changed')
    final=review['final'];stable=review['steady_from_fault_plus3s'];raw=final['q_raw']
    require(stable['last']-stable['first']>=3 and stable['joint_ranges_raw']==[[q,q]for q in raw],
        'Independent endpoint not stationary')
    require(all(lo<=q<=hi for q,(lo,hi)in zip(raw,support.JOINT_LIMITS_RAW))
        and max(abs(q-t)*support.RAD_PER_RAW for q,t in zip(raw,PARENT_TARGET))<=.003,
        'Independent endpoint outside original nominal/arrival bounds')
    return dict(raw_q=list(raw),q=[q*support.RAD_PER_RAW for q in raw],pose=final['pose'],
                opening_m=state['before']['opening_m'],jaw_code=state['before']['jaw_code'])


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    require(boot==guard.predecessor.PARENT_BOOT,'Different boot requires separate review')
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    names=[boot+'.json',guard.predecessor.CHILD_NAME,'guarded_task_'+boot+'.json',
        predecessor.predecessor.CHILD_NAME,predecessor.CHILD_NAME,frozen.child_name(boot)]
    hashes=[guard.predecessor.PARENT_SHA,guard.SUCCESS_SESSION_SHA,predecessor.predecessor.PARENT_SHA,
        predecessor.PARENT_SHA,frozen.PARENT_SHA,PARENT_SHA]
    with ExitStack()as stack:
        originals=[]
        for name,digest in zip(names,hashes):
            path=root/name;require(path.is_file(),'Immutable parent absent')
            stack.enter_context(SessionFile(path));raw=path.read_bytes()
            require(hashlib.sha256(raw).hexdigest()==digest,'Immutable parent changed');originals.append(raw)
        parent=json.loads(originals[-1]);endpoint=reviewed_evidence(reviewed,parent)
        store=stack.enter_context(SessionFile(root/child_name(boot)))
        require(store.load()is None,'J5 profile child already exists; no restart/output bypass')
        session=dict(stage='task',generation=parent['generation']+1,generations=[],pending=None,failure=None,
            stop_latched=False,held_raw=None,commissioning_attempted=True,first_segment_verified=False,
            parent_session_sha256=PARENT_SHA,parent_failure_chain_preserved=True,
            prior_failure=copy.deepcopy(parent['failure']),raw_review_sha256=RAW_REVIEW_SHA,
            joint_tracking_margin_rad=list(profile.MARGINS),feedback_excursion_physical_cause_resolved=False)
        store.save(session)
        try:yield parent,endpoint,store,session
        finally:require(all((root/name).read_bytes()==raw for name,raw in zip(names,originals)),
                        'Old parent bytes changed')


class J5Task(frozen.RXGuardedTask):
    # Only these reviewed functions resolve path_check via the private facade.
    # Their original globals and every frozen module remain untouched.
    execute=profile.private_function(guard.GuardedTask.execute,support=support)
    send_hold=profile.private_function(guard.GuardedTask.send_hold,support=support)
    _send_once=profile.private_function(guard.v1.Interruptible.send_once,support=support)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        require(tuple(self.limits.get('joint_tracking_margin_rad',()))==profile.MARGINS,'Runtime/profile mismatch')
        self.identity.update(parent_sequence=62,parent_adoption_token=PARENT_TOKEN,
            rx_runtime_sha256=RX_SHA,raw_overlay_sha256=RAW_OVERLAY_SHA,
            tracking_profile_sha256=sha(profile.__file__),joint_tracking_margin_rad=list(profile.MARGINS))
        self.handoff_pending=True

    def baseline(self):
        current=super().baseline()
        if self.handoff_pending:
            require(max(abs(q-t)*support.RAD_PER_RAW for q,t in zip(current['raw_q'],PARENT_TARGET))<=.003,
                    'Fresh handoff differs from original target arrival bounds')
            self.handoff_pending=False
        return current

    def monitor(self,s,origin,target):
        profile.monitor(s,origin,target,self.limits)
        if self.original is not None:profile.monitor(s,self.original[0],self.original[1],self.limits)

    def send_once(self,raw,measured,origin,target,kind,*,moving):
        if not self.session['first_segment_verified']and kind=='initial':
            require(max(abs(a-b)for a,b in zip(raw,measured['raw_q']))<=500,'First profile segment ceiling0.5degree')
        self.node.piper.rx_latch.assert_clean()
        return self._send_once(raw,measured,origin,target,kind,moving=moving)

    def record(self,event,**fields):
        if event=='ready':
            fields.pop('retrospective_monitor_failure_preserved',None)
            fields.update(prior_j5_tracking_failure_preserved=True,first_segment_max_joint_deg=.5,
                          joint_tracking_margin_rad=list(profile.MARGINS))
        return super().record(event,**fields)


def main(argv=None):
    require(sha(frozen.__file__)==RX_SHA and sha(frozen.overlay.__file__)==RAW_OVERLAY_SHA
            and sha(profile.__file__)==PROFILE_SHA,'Frozen RX/profile changed')
    # Reuse the complete reviewed startup/exit path with private bindings. Its
    # base module is freshly loaded; no live process or imported frozen global
    # is changed. Constructor corrects the predecessor-specific identity fields.
    launch=profile.private_function(frozen.main,__file__=__file__,__doc__=__doc__,
        RXGuardedTask=J5Task,reserve=reserve,support=support,overlay=profile,OVERLAY_SHA=PROFILE_SHA,
        CONFIG_SHA=CONFIG_SHA,PARENT_SHA=PARENT_SHA,RESULT_SHA=RESULT_SHA,PARENT_TOKEN=PARENT_TOKEN)
    return launch(argv)


if __name__=='__main__':main()
