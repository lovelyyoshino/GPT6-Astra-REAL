#!/usr/bin/env python3
"""One explicitly reviewed child trial of the frozen v2 J2 hold core.

The original failed session is locked and read, never cleared or overwritten.
This entry is bound to one particular parent failure and one user authorization.
"""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"scripts"))
import ros_interruptible_joint_entry_v2 as core

v1=core.v1;support=core.support;require=core.require;SessionFile=v1.SessionFile
V2_SHA="1ad178ab26b5724ffeb70adc48be734ae1e61c5075dd797b4417409090d20121"
PARENT_BOOT="9d436862-a252-46e5-ac49-0a650ec2a603"
PARENT_TOKEN="1f5a51ad367b46f9a9ded9aa7063d4d9"
PARENT_SHA="cfa3c2b827cabd2e0ed0afb47f13263bda294ef4b741e79adcb950a0cd025140"
PARENT_RUN=ROOT/"runs/stop_integration_20261006_152651"
PARENT_SUMMARY_SHA="8ca92727ac4ac9ff56edcefae1a22861e973b72f26cc9cffc041711594a6f5de"
PARENT_CAN_SHA="5717b156d761d93065d9cfc2486abba8918410d80ba3d9ecda37f7c94d778008"
AUTHORIZATION="请直接开始测试，我很需要尽快开始试验"
CHILD_NAME="trial_after_presend_slip_"+PARENT_SHA+".json"


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_parent(parent,digest,boot,reviewed,summary,can_review):
    require(digest==PARENT_SHA and boot==PARENT_BOOT,"Parent hash/boot is not the specifically reviewed failure")
    require(reviewed.get("parent_failure_sha256")==PARENT_SHA
            and reviewed.get("parent_run")==str(PARENT_RUN)
            and reviewed.get("parent_adoption_token")==PARENT_TOKEN
            and reviewed.get("parent_sequence")==1
            and reviewed.get("user_authorization")==AUTHORIZATION,"Specific child authorization record required")
    identity=parent["identity"];status=parent["status"];pending=parent["pending"]
    require(identity["boot_id"]==boot and identity["adoption_token"]==PARENT_TOKEN
            and identity["source_sha256"]==core.V1_SHA,"Parent identity mismatch")
    require(parent["failure"]["error"]==status["failure"]=="Pre-send slip"
            and status["sequence"]==1 and status["adoption_token"]==PARENT_TOKEN
            and status["phase"]=="failed"and status["active"]is False
            and parent["stop_latched"]is True and status["hold_confirmed"]is False,
            "Parent must retain its failed hold latch")
    receipts=status["receipts"]
    require(len(receipts)==1 and receipts[0]["kind"]=="initial"
            and receipts[0]["attempted_frames"]==receipts[0]["socket_send_returns"]==4
            and parent["attempted_frames"]==4 and pending["kind"]=="hold"
            and pending["sequence"]==1 and pending["attempted_frames"]==0,
            "Parent was not exactly initial4/4 then hold0TX")
    require(summary["failure"]=="Pre-send slip"and summary["hold_frames_attempted"]==0
            and summary["initial_frames_attempted"]==summary["initial_socket_send_returns"]==4
            and summary["failure_latch_preserved"]is True,"Parent review mismatch")
    after=summary["after_observation"]
    require(after["samples"]>=20 and after["fault"]==0 and after["driver_codes"]==[64]*6
            and after["active"]is False and after["driver_failure"]=="Pre-send slip",
            "Missing reviewed healthy completion evidence")
    require(can_review["transport_errors"]==0 and can_review["all_driver_codes_64"]is True
            and can_review["all_jaw_code_64"]is True and can_review["last_50s"]["samples"]>=2000
            and can_review["last_50s"]["j2_constant"]is True
            and abs(can_review["final_j2_error_deg"])<=.003*180/3.141592653589793
            and can_review["z_below_original_precommand"]is False,"Independent parent arrival review missing")


@contextmanager
def reserve_trial(boot,reviewed,summary,can_review,*,session_root=None):
    root=Path(session_root)if session_root is not None else v1.SESSION_ROOT
    parent_path=root/(boot+".json")
    require(parent_path.is_file(),"Original failed session does not exist")
    with SessionFile(parent_path)as parent_store:
        original=parent_path.read_bytes();parent=json.loads(original)
        check_parent(parent,hashlib.sha256(original).hexdigest(),boot,reviewed,summary,can_review)
        with SessionFile(root/CHILD_NAME)as child_store:
            require(child_store.load()is None,"This authorized child trial already exists; no retry/new-output bypass")
            child=dict(parent_failure_sha256=PARENT_SHA,parent_session_path=str(parent_path),
                       pending=None,failure=None,stop_latched=False,held_raw=None,
                       trial_attempted=False,trial_closed=False,user_authorization=AUTHORIZATION)
            child_store.save(child)
            try:
                yield parent,child_store,child
            finally:
                require(parent_path.read_bytes()==original,"Parent failed session changed during trial")


class OneShotTrial(core.InterruptibleCandidate):
    def finish(self,phase,after,window,**extra):
        # Keep status readers blocked until inherited provenance is relabelled.
        # The immutable core still reports its original offline-candidate origin.
        with self.state_lock:
            result=super().finish(phase,after,window,physical_trial_wrapper=True,
                                 candidate_offline_only_field_scope="Inherited core-origin marker; execution is this separately authorized physical trial",
                                 **extra)
            result.update(candidate_offline_only=False,core_origin_offline_candidate=True,
                          physical_trial_wrapper=True,one_shot_trial=True)
            self.state["result"]=result;self.save()
            SessionFile(self.output/("action_%06d_result.json"%self.state["sequence"])).save(result)
            self.record("physical_trial_result",durable=True,**result)
            return result

    def execute(self,message):
        with self.state_lock:
            require(not self.session["trial_attempted"]and not self.session["trial_closed"],
                    "Only one admitted-or-refused command opportunity in this child trial")
            self.session["trial_attempted"]=True
            self.save()
        try:return super().execute(message)
        finally:
            with self.state_lock:
                self.node.adopted=False  # Software only; never disable motors.
                self.session["trial_closed"]=True
                self.state["trial_closed"]=True
                self.save()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entry-config",required=True)
    parser.add_argument("--entry-output-dir",required=True)
    parser.add_argument("--empty-gripper-confirmed",action="store_true",required=True)
    args,remaining=parser.parse_known_args(argv)
    require(remaining and not remaining[0].startswith("-"),"Pinned vendor script required")
    require(sha(core.__file__)==V2_SHA and sha(v1.__file__)==core.V1_SHA,"Frozen v2/v1 changed")
    require(sha(support.__file__)==v1.SUPPORT_SHA and sha(v1.EVIDENCE_PATH)==v1.EVIDENCE_SHA,"Frozen support/evidence changed")
    summary_path=PARENT_RUN/"verification_summary.json";can_path=PARENT_RUN/"independent_can_review.json"
    require(sha(summary_path)==PARENT_SUMMARY_SHA and sha(can_path)==PARENT_CAN_SHA,"Parent review evidence changed")
    summary=json.loads(summary_path.read_text());can_review=json.loads(can_path.read_text())
    config_path=Path(args.entry_config).resolve();review_path=config_path.parent/"reviewed_parent.json"
    reviewed=json.loads(review_path.read_text());limits=support.checked_probe_limits(json.loads(config_path.read_text()))
    boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    base=support.load_base();token=uuid.uuid4().hex;instances=[]
    with reserve_trial(boot,reviewed,summary,can_review)as(parent,store,session):
        output=Path(args.entry_output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
        original_factory=base.node_class
        def factory(vendor,rospy):
            Parent=original_factory(vendor,rospy)
            class TrialNode(Parent):
                def __init__(self):
                    self.rospy_probe=rospy;self.executor=None
                    super().__init__()  # Same guarded 0TX adoption and three-second baseline.
                    import rosgraph
                    from std_srvs.srv import Trigger,TriggerResponse
                    def ownership():
                        base.binding();speed=rospy.get_param("~speed_percent")
                        require(type(speed)is int and speed==1,"Only live1percent allowed")
                        pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                        require(len(dict(pubs).get(rospy.resolve_name("joint_ctrl_single"),[]))<=1,"Competing joint publishers")
                        for topic in ("pos_cmd","enable_flag"):
                            require(not dict(pubs).get(rospy.resolve_name(topic)),"Unsupported publisher present")
                    def services(name,call):
                        def handler(_):
                            accepted,data=call()
                            return TriggerResponse(success=accepted,message=json.dumps(data,allow_nan=False))
                        return rospy.Service(name,Trigger,handler)
                    identity=dict(boot_id=boot,pid=os.getpid(),adoption_token=token,adopted_unix_s=time.time(),
                                  source_sha256=sha(__file__),entry_sha256=sha(__file__),v2_sha256=V2_SHA,v1_sha256=core.V1_SHA,
                                  base_sha256=support.BASE_SHA,support_sha256=v1.SUPPORT_SHA,
                                  vendor_sha256=base.DRIVER_SHA256,config_sha256=sha(config_path),
                                  parent_failure_sha256=PARENT_SHA,parent_run=str(PARENT_RUN),
                                  parent_adoption_token=PARENT_TOKEN,parent_sequence=1,
                                  reviewed_parent_sha256=sha(review_path),parent_summary_sha256=PARENT_SUMMARY_SHA,
                                  parent_can_review_sha256=PARENT_CAN_SHA,parent_session_path=session["parent_session_path"],
                                  child_session_path=str(store.path),can_interface=base.CAN_NAME,usb_interface=base.USB_INTERFACE,
                                  user_authorization=AUTHORIZATION,empty_gripper_operator_confirmed=True)
                    ownership();session["identity"]=identity
                    executor=OneShotTrial(self,base,limits,support.manufacturer_fk(),store,session,output,identity,ownership,
                                         service_factory=services,namespace=rospy.get_name(),token=token)
                    instances.append(executor)
                    initial=executor.read()
                    expected=[x*3.141592653589793/180 for x in summary["after_observation"]["q_deg"]]
                    require(max(abs(a-b)for a,b in zip(initial["q"],expected))<=.003
                            and abs(initial["opening_m"]-parent["status"]["before"]["opening_m"])<=.0005,
                            "Current fresh pose/jaw no longer matches specifically reviewed parent endpoint")
                    session["held_raw"]=list(initial["raw_q"]);self.executor=executor;executor.save()
                    self.hold_status_service=services("~hold_status",lambda:(True,executor.status()))
                    executor.record("ready",durable=True,identity=identity,actuator_frames=0,sdk_init_queries=0,
                                    one_shot_child_trial=True,parent_failure_preserved=True)
                def joint_callback(self,message):
                    require(self.executor is not None,"Adoption incomplete")
                    return self.executor.execute(message)
                def denied_trial(self,*a,**k):
                    raise RuntimeError("Child trial only allows one unloaded1percent J2 toward-zero command")
                pos_callback=denied_trial
                handle_gripper_service=denied_trial
            return TrialNode
        base.node_class=factory;sys.argv=[str(Path(__file__))]+remaining
        try:base.main()
        finally:
            for instance in instances:instance.feedback.close()


if __name__=="__main__":main()
