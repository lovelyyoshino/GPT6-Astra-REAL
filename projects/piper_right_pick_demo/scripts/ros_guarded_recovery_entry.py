#!/usr/bin/env python3
"""Reviewed seq52 completion handoff, then one separated shoulder pilot.

The failed parent and frozen GuardedTask remain unchanged. A new session cannot
restart or approve itself. No hardware access occurs at import.
"""
import argparse
from contextlib import ExitStack,contextmanager
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"scripts"))
import ros_guarded_task_entry as guard
require=guard.require;support=guard.support;SessionFile=guard.SessionFile
GUARD_SHA="3ba50485539a379af82a681e33084feb8d058c5cc5180849be38182590dbd70a"
PARENT_SHA="8e08acf11ab585bd4b5cd2cda5599df8376e2e36155dd109e30acc7d57cf0c2f"
RESULT_SHA="00c9a2fb943fb5da9d223d142a7cb29881650d35cec0363876ae9efd3df39efb"
CAN_SHA="b762d4393642d02df94de3db89f89c9c81b3215b92afafe5197f1e21ae672d76"
CONFIG_SHA="4771a64546cd6ba513b125b8717fae10169b06870a22b4662a14207bbb219863"
PARENT_TOKEN="62e5812f4ca24dcc8098c14d13be1cc8"
PARENT_RUN=ROOT/"runs/cola_on_cup_resume_20261006_160005"
FIXED_TARGET=[67,33732,-26396,0,7144,-3539]
CHILD_NAME="guarded_recovery_"+PARENT_SHA+".json"
sha=guard.sha


def check_evidence(parent,result,trace,reviewed):
    expected=dict(parent_session_sha256=PARENT_SHA,parent_action_result_sha256=RESULT_SHA,
        independent_can_sha256=CAN_SHA,parent_sequence=52,parent_adoption_token=PARENT_TOKEN,
        user_authorization=guard.AUTHORIZATION)
    require(all(reviewed.get(k)==v for k,v in expected.items()),"Explicit reviewed seq52 recovery required")
    identity=parent["identity"];s=parent["status"]
    require(identity["source_sha256"]==GUARD_SHA and identity["config_sha256"]==CONFIG_SHA
            and identity["adoption_token"]==PARENT_TOKEN and s==result,"Parent identity/result mismatch")
    require(parent["failure"]["error"]==s["failure"]=="Outside original probe joint box"
            and s["sequence"]==52 and s["phase"]=="failed"and s["active"]is False
            and s["hold_requested"]is False and s["hold_confirmed"]is False,"Different failure scope")
    receipts=s["receipts"];pending=parent["pending"]
    require(len(receipts)==1 and receipts[0]["kind"]=="initial"
            and receipts[0]["attempted_frames"]==receipts[0]["socket_send_returns"]==4
            and receipts[0]["target_raw"]==s["target_raw"]==FIXED_TARGET
            and pending==dict(sequence=52,kind="initial",target_raw=FIXED_TARGET,attempted_frames=4),
            "Expected one complete initial transaction and retained pending record")
    require(trace["frames_sent_by_this_script"]==0 and trace["trace_transport_clean"]is True
            and trace["kernel_timestamp_enabled"]is True and trace["socket_dropped_total"]==0
            and not trace["control_frames"]and not trace["bad_frames"]and not trace["missing_feedback_ids"]
            and not trace["timestamp_backwards"]and not trace["sample_gaps"],"Independent trace incomplete")
    rows=trace["samples"];require(len(rows)>=100 and rows[-1]["sampled_at"]-rows[0]["sampled_at"]>=3.,"Insufficient independent stability")
    old=None;last=None;feedback_ids=set(support.load_base().FEEDBACK_IDS)
    for row in rows:
        frames={f["id"]:f for f in row["frames"]}
        require(set(frames)==feedback_ids,"Missing raw fragment")
        data={i:bytes.fromhex(f["data_hex"])for i,f in frames.items()}
        stamps=[f["timestamp"]for f in frames.values()]
        require(all(f["timestamp_basis"]=="kernel_socket_SO_TIMESTAMPNS_unix"for f in frames.values())
                and 0<=row["sampled_at"]-min(stamps)<=.1 and max(stamps)<=row["sampled_at"]
                and min(stamps)>receipts[0]["finished_unix_s"],"Independent state is not fresh post-send feedback")
        q=sum((list(struct.unpack(">ii",data[i]))for i in range(0x2a5,0x2a8)),[])
        pose=sum((list(struct.unpack(">ii",data[i]))for i in range(0x2a2,0x2a5)),[])
        jaw=struct.unpack(">i",data[0x2a8][:4])[0]
        require(data[0x2a1]==bytes.fromhex("0100010000000000")and all(data[i][5]==64 for i in range(0x261,0x267))
                and data[0x2a8][6]==64,"Independent feedback not healthy idle enabled J")
        require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(q,FIXED_TARGET))<=.003,"Original target has not arrived")
        last=dict(raw_q=q,q=[v*support.RAD_PER_RAW for v in q],pose=[v/1e6 for v in pose[:3]]+[v*support.RAD_PER_RAW for v in pose[3:]],
                  opening_m=jaw/1e6,jaw_code=64)
        signature=(q,pose,jaw)
        require(old is None or old==signature,"Independent endpoint is not constant")
        old=signature
    return last


@contextmanager
def reserve(boot,reviewed,result,trace,*,session_root=None):
    require(boot==guard.predecessor.PARENT_BOOT,"Recovery boot mismatch")
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    names=[boot+".json",guard.predecessor.CHILD_NAME,"guarded_task_"+boot+".json"]
    hashes=[guard.predecessor.PARENT_SHA,guard.SUCCESS_SESSION_SHA,PARENT_SHA]
    with ExitStack()as stack:
        originals=[]
        for name,digest in zip(names,hashes):
            path=root/name;require(path.is_file(),"Missing immutable parent")
            stack.enter_context(SessionFile(path));raw=path.read_bytes()
            require(hashlib.sha256(raw).hexdigest()==digest,"Parent changed")
            originals.append(raw)
        parent=json.loads(originals[-1]);endpoint=check_evidence(parent,result,trace,reviewed)
        store=stack.enter_context(SessionFile(root/CHILD_NAME))
        require(store.load()is None,"Reviewed recovery already exists; no second attempt/output bypass")
        session=dict(pending=None,failure=None,stop_latched=False,held_raw=None,stage="task",generation=parent["generation"]+1,
            generations=[],commissioning_attempted=True,parent_session_sha256=PARENT_SHA,
            parent_failure_preserved=True,parent_target_completion_independently_observed=True,
            recovery_stage="pilot",pilot_attempted=False,fixed_target_raw=list(FIXED_TARGET))
        store.save(session)
        try:yield parent,endpoint,store,session
        finally:require(all((root/name).read_bytes()==raw for name,raw in zip(names,originals)),"Old session modified")


def pilot_target(message,before,anchor):
    raw=[round(v*(1000*180/math.pi))for v in message.position]
    require(len(raw)==6,"Six target positions required")
    require(all(raw[i]==FIXED_TARGET[i]for i in (0,3,4,5)),"Pilot must keep four original target axes fixed")
    require(anchor[1]>0 and anchor[2]<0,"Pilot requires interior shoulder/elbow state")
    scale=raw[1]/anchor[1]
    require(0<=scale<1 and abs(raw[2]-round(anchor[2]*scale))<=1,"Pilot requires common J2/J3 toward-zero ratio")
    require(200<=before["raw_q"][1]-raw[1]<=800 and 200<=raw[2]-before["raw_q"][2]<=800,
            "Pilot shoulders must approach zero by0.2..0.8degree")
    return raw


class RecoveryTask(guard.GuardedTask):
    def __init__(self,*a,**k):
        super().__init__(*a,**k);self.state.update(recovery_stage=self.session["recovery_stage"],recovery_review_service=None)

    def execute(self,message):
        if self.session["recovery_stage"]=="approved":return super().execute(message)
        with self.state_lock:
            require(self.session["recovery_stage"]=="pilot"and not self.session["pilot_attempted"],"Recovery pilot opportunity consumed")
            self.session["pilot_attempted"]=True;self.save()
        try:
            pilot_target(message,self.read(),self.session["pilot_anchor_raw"])
            return super().execute(message)
        except Exception as error:
            if not self.session["failure"]:self.fail(error)
            raise
        finally:
            with self.state_lock:
                if self.session["recovery_stage"]!="approved":self.node.adopted=False

    def send_once(self,raw,measured,origin,original_target,kind,*,moving):
        if self.session["recovery_stage"]=="pilot"and kind=="initial":
            # Revalidate against the three-second baseline, not just admission.
            from types import SimpleNamespace
            pilot_target(SimpleNamespace(position=[v*support.RAD_PER_RAW for v in raw]),
                         measured,self.session["pilot_anchor_raw"])
        return super().send_once(raw,measured,origin,original_target,kind,moving=moving)

    def finish(self,phase,after,window,**extra):
        pilot=self.session["recovery_stage"]=="pilot"
        with self.state_lock:
            if pilot:self.node.adopted=False
            if pilot and phase=="completed":
                before=self.state["before"]
                progress=[(before["raw_q"][1]-after["raw_q"][1])/1000,
                          (after["raw_q"][2]-before["raw_q"][2])/1000]
                require(all(v>=.2 for v in progress),"Pilot did not demonstrate two-axis actual progress")
                extra["observed_pilot_progress_deg"]=progress
            result=super().finish(phase,after,window,recovery_parent_sequence=52,parent_failure_preserved=True,**extra)
            if pilot:
                self.session["recovery_stage"]="awaiting_review"if phase=="completed"else"locked"
                self.state.update(recovery_stage=self.session["recovery_stage"],resume_service=None)
                if phase=="completed":
                    seq=self.state["sequence"];gen=self.session["generation"];digest=self.state["result_sha256"]
                    service=self.namespace+"/review_recovery/seq_%d_gen_%d_%s_%s"%(seq,gen,self.token,digest)
                    self.state["recovery_review_service"]=service
                    self.retained_services.append(self.service_factory(service,lambda:self.review_recovery(seq,gen,digest)))
                self.save()
        return result

    def resume(self,*a):
        require(self.session["recovery_stage"]=="approved","Pilot cannot use ordinary hold resume")
        return super().resume(*a)

    def gripper(self,request):
        require(self.session["recovery_stage"]=="approved","Jaw forbidden before recovery review")
        return super().gripper(request)

    def review_recovery(self,seq,generation,digest):
        require(self.node.action_lock.acquire(False),"Action still active")
        admitted=False
        try:
            with self.state_lock:
                require(self.session["recovery_stage"]=="awaiting_review"and self.state["phase"]=="completed"
                    and self.state["sequence"]==seq and self.session["generation"]==generation
                    and self.state["result_sha256"]==digest,"Wrong or retired pilot review")
                require(not self.state["active"]and not self.session["failure"]and not self.session["pending"]
                    and not self.session["stop_latched"]and not self.node.failed and not self.node.piper.broken,"Recovery unresolved")
                require(sha(self.output/("action_%06d_result.json"%seq))==digest,"Pilot result changed")
                receipts=self.state["receipts"]
                require(len(receipts)==1 and receipts[0]["kind"]=="initial"
                    and receipts[0]["attempted_frames"]==receipts[0]["socket_send_returns"]==4,"Pilot transaction not complete")
                result=self.state["result"];require(result["target_reached"]is True,"Pilot has not arrived")
                self.node.active=True;admitted=True
            self.ownership_check();fresh=self.baseline();guard.slip(result["after"],fresh)
            require(abs(fresh["opening_m"]-result["after"]["opening_m"])<=.0005 and fresh["jaw_code"]==64,"Pilot jaw changed")
            self.node.piper.healthy(fresh);require(not self.node.rospy_probe.is_shutdown(),"Shutdown during review")
            with self.state_lock:
                self.session["generations"].append(dict(generation=generation,result_sha256=digest,
                    kind="reviewed_separated_shoulder_pilot",result=copy.deepcopy(result),fresh_state=fresh,parent_failure_preserved=True))
                self.session.update(generation=generation+1,recovery_stage="approved",held_raw=list(fresh["raw_q"]))
                self.state.update(generation=generation+1,recovery_stage="approved",phase="idle",active=False,
                    hold_service=None,recovery_review_service=None,resume_service=None,result=None,result_sha256=None,
                    receipts=[],before=None,target_raw=None,requested_raw=None)
                self.original=None;self.bound_hold=None;self.save();self.node.adopted=True
            self.record("recovery_pilot_reviewed",durable=True,old_generation=generation,new_generation=generation+1,
                        result_sha256=digest,parent_failure_preserved=True,actuator_frames=0)
            return True,self.status()
        except Exception as error:
            if admitted:self.fail(error)
            raise
        finally:self.node.active=False;self.node.action_lock.release()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entry-config",required=True);parser.add_argument("--entry-output-dir",required=True)
    parser.add_argument("--empty-gripper-confirmed",action="store_true",required=True)
    args,remaining=parser.parse_known_args(argv)
    require(remaining and not remaining[0].startswith("-"),"Pinned vendor script required")
    require(sha(guard.__file__)==GUARD_SHA and sha(guard.v1.__file__)==guard.predecessor.core.V1_SHA
        and sha(support.__file__)==guard.v1.SUPPORT_SHA and sha(guard.predecessor.__file__)==guard.TRIAL_SHA
        and sha(guard.predecessor.core.__file__)==guard.predecessor.V2_SHA,"Frozen executor changed")
    result_path=PARENT_RUN/"runtime/action_000052_result.json";trace_path=PARENT_RUN/"home051_post_failure_can.json"
    require(sha(result_path)==RESULT_SHA and sha(trace_path)==CAN_SHA,"Reviewed evidence changed")
    result=json.loads(result_path.read_text());trace=json.loads(trace_path.read_text())
    config=Path(args.entry_config).resolve();require(sha(config)==CONFIG_SHA,"Recovery limits/config must remain unchanged")
    limits=support.checked_probe_limits(json.loads(config.read_text()));review_path=config.parent/"reviewed_recovery.json"
    reviewed=json.loads(review_path.read_text());boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    base=support.load_base();token=uuid.uuid4().hex;instances=[]
    with reserve(boot,reviewed,result,trace)as(parent,endpoint,store,session):
        output=Path(args.entry_output_dir).resolve();output.mkdir(parents=True,exist_ok=False);original_factory=base.node_class
        def factory(vendor,rospy):
            Parent=original_factory(vendor,rospy)
            class RecoveryNode(Parent):
                def __init__(self):
                    self.rospy_probe=rospy;self.executor=None;super().__init__()
                    import rosgraph
                    from std_srvs.srv import Trigger,TriggerResponse
                    def ownership():
                        base.binding();speed=rospy.get_param("~speed_percent")
                        require(type(speed)is int and speed==1,"Only1percent allowed")
                        pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                        require(len(dict(pubs).get(rospy.resolve_name("joint_ctrl_single"),[]))<=1,"Competing joint publishers")
                        for topic in ("pos_cmd","enable_flag"):
                            require(not dict(pubs).get(rospy.resolve_name(topic)),"Unsupported publisher")
                    def services(name,call):
                        def handler(_):
                            try:ok,data=call()
                            except Exception as error:return TriggerResponse(success=False,message=json.dumps(dict(error=str(error))))
                            return TriggerResponse(success=ok,message=json.dumps(data,allow_nan=False))
                        return rospy.Service(name,Trigger,handler)
                    identity=dict(boot_id=boot,pid=os.getpid(),adoption_token=token,adopted_unix_s=time.time(),
                        source_sha256=sha(__file__),guard_sha256=GUARD_SHA,base_sha256=support.BASE_SHA,
                        support_sha256=guard.v1.SUPPORT_SHA,vendor_sha256=base.DRIVER_SHA256,config_sha256=sha(config),
                        reviewed_recovery_sha256=sha(review_path),parent_session_sha256=PARENT_SHA,
                        parent_action_result_sha256=RESULT_SHA,independent_can_sha256=CAN_SHA,parent_sequence=52,
                        parent_adoption_token=PARENT_TOKEN,task_session_path=str(store.path),
                        can_interface=base.CAN_NAME,usb_interface=base.USB_INTERFACE,user_authorization=guard.AUTHORIZATION,
                        empty_gripper_operator_confirmed=True)
                    ownership();session["identity"]=identity
                    executor=RecoveryTask(self,base,limits,support.manufacturer_fk(),store,session,output,identity,ownership,
                                         service_factory=services,namespace=rospy.get_name(),token=token)
                    instances.append(executor);initial=executor.read();guard.slip(endpoint,initial)
                    require(max(abs(q-r*support.RAD_PER_RAW)for q,r in zip(initial["q"],FIXED_TARGET))<=.003
                        and abs(initial["opening_m"]-endpoint["opening_m"])<=.0005,"Current state not at reviewed original goal")
                    session["held_raw"]=list(initial["raw_q"]);session["pilot_anchor_raw"]=list(initial["raw_q"])
                    self.executor=executor;executor.save()
                    self.hold_status_service=services("~hold_status",lambda:(True,executor.status()))
                    executor.record("ready",durable=True,identity=identity,actuator_frames=0,sdk_init_queries=0,
                                    recovery_stage="pilot",parent_failure_preserved=True)
                def joint_callback(self,msg):
                    require(self.executor is not None,"Adoption incomplete");return self.executor.execute(msg)
                def handle_gripper_service(self,req):
                    require(self.executor is not None and self.gripper_exist is True,"Existing adopted jaw required")
                    self.executor.gripper(req);reply=vendor.GripperResponse();reply.code=15900;reply.status=True;return reply
                def pos_callback(self,*a,**k):raise RuntimeError("P/L remain forbidden")
            return RecoveryNode
        base.node_class=factory;sys.argv=[str(Path(__file__))]+remaining
        try:base.main()
        finally:
            for instance in instances:instance.feedback.close()


if __name__=="__main__":main()
