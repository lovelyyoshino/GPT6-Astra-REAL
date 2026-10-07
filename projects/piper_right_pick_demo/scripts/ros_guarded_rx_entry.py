#!/usr/bin/env python3
"""Reviewed idle handoff to the unchanged guarded J worker plus a raw RX latch."""
import argparse
from contextlib import ExitStack,contextmanager
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"scripts"))
import ros_guarded_interior_entry as predecessor
import guarded_rx_latch as overlay
guard=predecessor.guard;support=guard.support;require=guard.require;sha=guard.sha
SessionFile=guard.SessionFile
PREDECESSOR_SHA="36f4c2faae2e08bb5aa716c2c3c4f1e0860aa64f6b017f4da668e6f14abc4b61"
OVERLAY_SHA="7dd60a42f6bc73f4dfeb0ca62a10707f2bab75cc35ab4a7b391b204c866aa1ec"
PARENT_SHA="9ad647f223b9be4c5c5a7aacd9a0507c441b93af570ee58bd9cf4b10101d472e"
RESULT_SHA="1eeaf1c338528c6b2516403a1cf6de897f46158bb7c6c41fd03645e56b6eb2a7"
AUDIT_SHA="4c83b9e8c7464a79bfa02b6955e1f7fa84d4e2773d14fa610e52a72ab890d7a9"
CONTEXT_SHA="1a481cbc752cce624c4c2a82a7cf6d026ee45d259d547c1cdf1943647c4e07a8"
CONFIG_SHA=predecessor.CONFIG_SHA
PARENT_TOKEN="651db950ae394049895ab9dcbf680c68"
PARENT_RUN=ROOT/"runs/cola_on_cup_nearzero_20261006_173237"


def child_name(boot):return "guarded_rx_"+boot+".json"


def reviewed_evidence(reviewed,parent):
    expected=dict(parent_session_sha256=PARENT_SHA,parent_result_sha256=RESULT_SHA,
        parent_source_sha256=PREDECESSOR_SHA,parent_sequence=38,parent_generation=5,
        parent_adoption_token=PARENT_TOKEN,audit_sha256=AUDIT_SHA,context_sha256=CONTEXT_SHA,
        retrospective_monitor_failure=True,allow_limit_relaxation=False,home_replay_allowed=False,
        reviewed=True,current_completed_state_reviewed=True,first_segment_max_joint_deg=1)
    require(all(reviewed.get(k)==v for k,v in expected.items()),"Explicit unchanged-limit completed-state review required")
    require(isinstance(reviewed.get("user_authorization"),str)and reviewed["user_authorization"],"User task authorization required")
    result_path=PARENT_RUN/"runtime/action_000038_result.json"
    audit_path=PARENT_RUN/"raw_handoff_prep_nearzero_audit.json"
    context_path=PARENT_RUN/"raw_joint_pose_status_context.json"
    for path,key,digest in((result_path,"parent_result_path",RESULT_SHA),(audit_path,"audit_path",AUDIT_SHA),
                          (context_path,"context_path",CONTEXT_SHA)):
        require(str(path)==reviewed[key]and sha(path)==digest,"Reviewed evidence changed")
    result=json.loads(result_path.read_text());state=parent["status"];identity=parent["identity"]
    require(identity["source_sha256"]==PREDECESSOR_SHA and identity["config_sha256"]==CONFIG_SHA
        and identity["adoption_token"]==PARENT_TOKEN and parent["generation"]==5,"Parent identity mismatch")
    require(state["sequence"]==38 and state["phase"]=="completed"and not state["active"]
        and not state["failure"]and not parent["failure"]and not parent["pending"]
        and not parent["stop_latched"]and state["result"]==result,"Parent finite goal unresolved")
    require(result["phase"]=="completed"and result["target_reached"]is True
        and result["after"]["raw_q"]==reviewed["completed_actual_raw"],"Completed endpoint mismatch")
    receipts=state["receipts"]
    require(len(receipts)==1 and receipts[0]["kind"]=="initial"
        and receipts[0]["attempted_frames"]==receipts[0]["socket_send_returns"]==4,"Incomplete parent transaction")
    return result["after"]


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    require(boot==guard.predecessor.PARENT_BOOT,"Different boot requires separate review")
    root=Path(session_root)if session_root is not None else guard.v1.SESSION_ROOT
    names=[boot+".json",guard.predecessor.CHILD_NAME,"guarded_task_"+boot+".json",
           predecessor.predecessor.CHILD_NAME,predecessor.CHILD_NAME]
    hashes=[guard.predecessor.PARENT_SHA,guard.SUCCESS_SESSION_SHA,predecessor.predecessor.PARENT_SHA,
            predecessor.PARENT_SHA,PARENT_SHA]
    require(reviewed["parent_session_path"]==str(guard.v1.SESSION_ROOT/predecessor.CHILD_NAME),"Wrong parent session path")
    with ExitStack()as stack:
        originals=[]
        for name,digest in zip(names,hashes):
            path=root/name;require(path.is_file(),"Immutable parent absent")
            stack.enter_context(SessionFile(path));raw=path.read_bytes()
            require(hashlib.sha256(raw).hexdigest()==digest,"Immutable parent changed")
            originals.append(raw)
        parent=json.loads(originals[-1]);endpoint=reviewed_evidence(reviewed,parent)
        store=stack.enter_context(SessionFile(root/child_name(boot)))
        require(store.load()is None,"RX-guard session already exists; no restart/output bypass")
        session=dict(stage="task",generation=parent["generation"]+1,generations=[],pending=None,failure=None,
            stop_latched=False,held_raw=None,commissioning_attempted=True,first_segment_verified=False,
            parent_session_sha256=PARENT_SHA,parent_failure_chain_preserved=True,
            retrospective_monitor_failure=True,retrospective_audit_sha256=AUDIT_SHA,
            feedback_excursion_physical_cause_resolved=False)
        store.save(session)
        try:yield parent,endpoint,store,session
        finally:require(all((root/name).read_bytes()==raw for name,raw in zip(names,originals)),"Old parent bytes changed")


class RXGuardedTask(guard.GuardedTask):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.node.piper.rx_context_provider=self.tx_context

    def tx_context(self):
        # Called only by this worker's guarded socket send, with rx_lock held.
        # Do not take state/action/io locks here: those precede rx_lock elsewhere.
        require(self.node.piper.ticket is not None,"No worker ticket for RX context")
        state=self.state;before=state["before"]
        require(before is not None and state["active"],"No active action for RX context")
        kind=state["kind"]
        raw=state["target_raw"]if kind=="joint"else before["raw_q"]
        hold=None
        if kind=="joint"and self.bound_hold is not None:
            hold=dict(origin_raw=self.bound_hold["plan_sample"]["raw_q"],target_raw=self.bound_hold["target_raw"])
        return dict(kind=kind,sequence=state["sequence"],generation=self.session["generation"],
            token=self.token,origin_raw=before["raw_q"],target_raw=raw,jaw_m=before["opening_m"],
            jaw_code=before["jaw_code"],hold_box=hold)

    def synchronize_fault(self):
        with self.node.piper.rx_lock:fault=copy.deepcopy(self.node.piper.rx_latch.first_fault)
        if fault is not None:
            with self.state_lock:
                if not self.session["failure"]:self.fail(RuntimeError("Raw feedback violation latched: "+fault["reason"]))
        return fault

    def fail(self,error):
        with self.state_lock:
            with self.node.piper.rx_lock:fault=copy.deepcopy(self.node.piper.rx_latch.first_fault)
            if fault is not None:
                self.session["raw_feedback_fault"]=fault;self.state["raw_feedback_fault"]=fault
            self.node.adopted=False
            if not self.session["failure"]:return super().fail(error)
            # An idle-monitor callback may persist the fault before the worker's
            # send-finally has appended a partial receipt. Preserve the first
            # fault, but durably include the final transaction counts as well.
            if any(r["attempted_frames"]for r in self.state["receipts"]):
                self.session["failure"]["accepted_target_may_continue"]=True
            self.save()
            SessionFile(self.output/("action_%06d_result.json"%self.state["sequence"])).save(self.state)
            self.record("failure_receipt_finalized",durable=True,sequence=self.state["sequence"],
                        receipts=self.state["receipts"],first_failure=self.session["failure"])

    def status(self):
        self.synchronize_fault()
        return super().status()

    def send_once(self,raw,measured,origin,target,kind,*,moving):
        if not self.session["first_segment_verified"]and kind=="initial":
            require(max(abs(a-b)for a,b in zip(raw,measured["raw_q"]))<=1000,"First guarded segment ceiling1degree")
        self.node.piper.rx_latch.assert_clean()
        return super().send_once(raw,measured,origin,target,kind,moving=moving)

    def finish(self,phase,after,window,**extra):
        self.node.piper.rx_latch.assert_clean()
        result=super().finish(phase,after,window,raw_feedback_guard=True,**extra)
        self.node.piper.rx_latch.assert_clean()
        if phase=="completed"and self.state["kind"]=="joint":
            self.session["first_segment_verified"]=True;self.save()
        return result

    def gripper(self,request):
        require(self.session["first_segment_verified"],"First reviewed small J segment required before jaw action")
        return super().gripper(request)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entry-config",required=True);parser.add_argument("--entry-output-dir",required=True)
    parser.add_argument("--empty-gripper-confirmed",action="store_true",required=True)
    args,remaining=parser.parse_known_args(argv)
    require(remaining and not remaining[0].startswith("-"),"Pinned vendor driver required")
    require(sha(overlay.__file__)==OVERLAY_SHA and sha(predecessor.__file__)==PREDECESSOR_SHA
        and sha(predecessor.predecessor.__file__)==predecessor.PREDECESSOR_SHA
        and sha(guard.__file__)==predecessor.GUARD_SHA
        and sha(support.__file__)==guard.v1.SUPPORT_SHA and sha(guard.v1.__file__)==guard.predecessor.core.V1_SHA
        and sha(guard.predecessor.__file__)==guard.TRIAL_SHA
        and sha(guard.predecessor.core.__file__)==guard.predecessor.V2_SHA,"Frozen dependency changed")
    config=Path(args.entry_config).resolve();require(sha(config)==CONFIG_SHA,"Unchanged config required")
    limits=support.checked_probe_limits(json.loads(config.read_text()))
    review_path=config.parent/"reviewed_handoff.json";reviewed=json.loads(review_path.read_text())
    boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip();token=uuid.uuid4().hex
    base=support.load_base();instances=[];timers=[]
    original_sdk=base.sdk_class;base.sdk_class=lambda original:overlay.sdk_overlay(original_sdk(original),limits)
    with reserve(boot,reviewed)as(parent,endpoint,store,session):
        output=Path(args.entry_output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
        factory=base.node_class
        def node_factory(vendor,rospy):
            Parent=factory(vendor,rospy)
            class RXNode(Parent):
                def __init__(self):
                    self.rospy_probe=rospy;self.executor=None
                    try:super().__init__()
                    except Exception as error:
                        piper=getattr(self,"piper",None);latch=getattr(piper,"rx_latch",None)
                        fault=copy.deepcopy(latch.first_fault)if latch is not None else None
                        session["failure"]=dict(error=str(error),unix_s=time.time(),startup=True,
                            accepted_target_may_continue=False,raw_feedback_fault=fault)
                        store.save(session)
                        SessionFile(output/"startup_failure.json").save(session["failure"])
                        raise
                    import rosgraph
                    from std_srvs.srv import Trigger,TriggerResponse
                    def ownership():
                        base.binding();speed=rospy.get_param("~speed_percent")
                        require(type(speed)is int and speed==1,"Only1percent allowed")
                        pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                        require(len(dict(pubs).get(rospy.resolve_name("joint_ctrl_single"),[]))<=1,"Competing joint publisher")
                        for name in("pos_cmd","enable_flag"):
                            require(not dict(pubs).get(rospy.resolve_name(name)),"Unsupported publisher")
                    def services(name,call):
                        def handler(_):
                            try:ok,data=call()
                            except Exception as error:return TriggerResponse(success=False,message=json.dumps(dict(error=str(error))))
                            return TriggerResponse(success=ok,message=json.dumps(data,allow_nan=False))
                        return rospy.Service(name,Trigger,handler)
                    identity=dict(boot_id=boot,pid=os.getpid(),adoption_token=token,adopted_unix_s=time.time(),
                        source_sha256=sha(__file__),rx_overlay_sha256=sha(overlay.__file__),predecessor_sha256=PREDECESSOR_SHA,
                        guard_sha256=predecessor.GUARD_SHA,base_sha256=support.BASE_SHA,support_sha256=guard.v1.SUPPORT_SHA,
                        vendor_sha256=base.DRIVER_SHA256,config_sha256=sha(config),reviewed_handoff_sha256=sha(review_path),
                        parent_session_sha256=PARENT_SHA,parent_result_sha256=RESULT_SHA,parent_sequence=38,
                        parent_adoption_token=PARENT_TOKEN,task_session_path=str(store.path),
                        can_interface=base.CAN_NAME,usb_interface=base.USB_INTERFACE,
                        user_authorization=reviewed["user_authorization"],empty_gripper_operator_confirmed=True)
                    ownership();session["identity"]=identity
                    executor=RXGuardedTask(self,base,limits,support.manufacturer_fk(),store,session,output,identity,ownership,
                        service_factory=services,namespace=rospy.get_name(),token=token)
                    instances.append(executor)
                    initial=executor.baseline();guard.slip(endpoint,initial)
                    require(abs(initial["opening_m"]-endpoint["opening_m"])<=.0005,"Jaw differs from reviewed endpoint")
                    session["held_raw"]=list(initial["raw_q"]);self.executor=executor;executor.save()
                    self.hold_status_service=services("~hold_status",lambda:(True,executor.status()))
                    timers.append(rospy.Timer(rospy.Duration(.05),lambda _:executor.synchronize_fault()))
                    executor.record("ready",durable=True,identity=identity,actuator_frames=0,sdk_init_queries=0,
                        retrospective_monitor_failure_preserved=True,first_segment_max_joint_deg=1)
                def joint_callback(self,message):
                    require(self.executor is not None,"Adoption incomplete");return self.executor.execute(message)
                def handle_gripper_service(self,request):
                    require(self.executor is not None and self.gripper_exist is True,"Existing adopted jaw required")
                    self.executor.gripper(request);reply=vendor.GripperResponse();reply.code=15900;reply.status=True;return reply
                def pos_callback(self,*args,**kwargs):raise RuntimeError("P/L remain forbidden")
            return RXNode
        base.node_class=node_factory;sys.argv=[str(Path(__file__))]+remaining
        try:base.main()
        finally:
            for timer in timers:timer.shutdown()
            for instance in instances:
                instance.synchronize_fault();instance.feedback.close()


if __name__=="__main__":main()
