#!/usr/bin/env python3
"""ROS J2-only, unloaded, one-percent execution with sequence-bound hold requests.

No import-time hardware I/O. Normal P/gripper/enable/reset/stop remain denied.
An external request is consumed by the original action worker, never a second
CAN sender. Hold locks further motion; there is deliberately no resume service.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"scripts"))
import ros_joint_stop_probe_entry as support
from ros_home_step import SessionFile, StableWindow

SUPPORT_SHA="6dcb316417f66081f8105cdd82b03540dacfd9974c54a355a8bcde9aca490176"
EVIDENCE_PATH=ROOT/"runs/stop_validation_20261006_150503/probe_runtime/dynamic_result.json"
EVIDENCE_SHA="28fc3d180f378ef770f5c9fcbec7a5a809c407ef89a241b8a0a6492d4b772fb7"
SESSION_ROOT=ROOT/"runs/ros_interruptible_joint_sessions"
require=support.require


class CancelBeforeDispatch(RuntimeError):
    pass


def decode_target(message,before,held_raw):
    require(len(message.position)==6,"Exactly six joint positions required")
    require(not message.effort,"Effort commands are unsupported")
    require(len(message.velocity)==7 and all(v==0 for v in message.velocity[:6])
            and type(message.velocity[6]) in (int,float) and message.velocity[6]==1,
            "Only explicit velocity=[0,0,0,0,0,0,1] is supported")
    values=list(message.position)
    require(all(type(v)in(int,float)and math.isfinite(v)for v in values),"Finite joint positions required")
    raw=[round(v*(1000*180/math.pi))for v in values]
    support.frames_for(raw)
    for i in (0,2,3,4,5):
        require(abs(raw[i]-before["raw_q"][i])*support.RAD_PER_RAW<=.003
                and abs(raw[i]-held_raw[i])*support.RAD_PER_RAW<=.003
                and abs(before["raw_q"][i]-held_raw[i])*support.RAD_PER_RAW<=.003,
                "Only J2 may change; other requested axes must match held targets within feedback tolerance")
    require(0<=raw[1]<before["raw_q"][1] and 0<before["raw_q"][1]-raw[1]<=1000,
            "Only positive J2 toward existing zero by at most1degree is supported")
    effective=list(held_raw);effective[1]=raw[1]
    support.frames_for(effective)
    return raw,effective


class Interruptible(support.Probe):
    def __init__(self,*args,service_factory,namespace,token,**kwargs):
        self.state_lock=threading.RLock();self.io_lock=threading.RLock()
        self.service_factory=service_factory;self.namespace=namespace;self.token=token
        self.retained_services=[]
        self.motion_history=[]
        self.state=dict(schema_version=1,adoption_token=token,sequence=0,active=False,phase="idle",
                        hold_service=None,hold_requested=False,hold_confirmed=False,stop_latched=False,
                        failure=None,command_sent_unix_s=None,before=None,target_raw=None,requested_raw=None,
                        latest_state=None,receipts=[],result=None,last_refusal=None)
        super().__init__(*args,**kwargs)

    def status(self):
        with self.state_lock:
            return copy.deepcopy(self.state)

    def save(self):
        with self.state_lock,self.io_lock:
            self.session["status"]=copy.deepcopy(self.state)
            self.store.save(self.session)
            SessionFile(self.output/"state.json").save(self.state)

    def record(self,event,*,durable=False,**values):
        row=dict(event=event,unix_s=self.clock.time(),monotonic_s=self.clock.monotonic(),**values)
        with self.io_lock:
            self.feedback.write(json.dumps(row,allow_nan=False)+"\n")
            if durable:self.feedback.flush();os.fsync(self.feedback.fileno())
        if event!="feedback":self.base.emit("interruptible_joint_"+event,**values)
        return row

    def change(self,**values):
        with self.state_lock:
            self.state.update(values);self.save()

    def read(self,*,moving=False):
        s=super().read(moving=moving)
        with self.state_lock:self.state["latest_state"]=copy.deepcopy(s)
        return s

    def requested(self):
        with self.state_lock:return self.state["hold_requested"]

    def request_hold(self,sequence,token):
        with self.state_lock:
            accepted=(token==self.token and sequence==self.state["sequence"] and self.state["active"]
                      and not self.state["failure"] and not self.state["hold_requested"]
                      and self.state["phase"] not in ("completed","hold_confirmed","already_completed",
                                                       "cancelled_before_dispatch","failed"))
            if accepted:
                self.state.update(hold_requested=True,stop_latched=True)
                self.state["hold_requested_unix_s"]=self.clock.time()
                self.session["stop_latched"]=True
                self.save()
                self.record("hold_request_accepted",durable=True,sequence=sequence,adoption_token=token)
            response=copy.deepcopy(self.state)
            response.update(accepted=bool(accepted),confirmed=self.state["hold_confirmed"])
            if not accepted:response["reason"]="Wrong/retired action, duplicate request, inactive or failed"
            return accepted,response

    def baseline(self):
        deadline,last,window=self.clock.monotonic()+10.,None,StableWindow()
        while self.clock.monotonic()<deadline:
            s=self.read()
            self.record("feedback",stage="baseline",state=s)
            if last is None or self.advanced(last,s):
                done=window.add(s,self.clock.monotonic());last=s
                if done:return s
            self.clock.sleep(.05)
        raise RuntimeError("Baseline stability not established")

    def send_once(self,raw,measured,origin,original_target,kind,*,moving):
        frames=support.frames_for(raw)
        path=support.path_check(measured,raw,self.limits,self.fk)
        require(self.node.piper.ticket is None,"Prior transaction unresolved")
        self.ownership_check()
        with self.state_lock:
            if kind=="initial"and self.state["hold_requested"]:raise CancelBeforeDispatch()
            self.state["phase"]="sending"if kind=="initial"else"hold_sending"
            self.session["pending"]=dict(sequence=self.state["sequence"],kind=kind,target_raw=list(raw),attempted_frames=0)
            self.save()
        self.record("intent",durable=True,sequence=self.state["sequence"],kind=kind,path=path,
                    requested_raw=self.state["requested_raw"],target_raw=raw,measured=measured,
                    frames=[dict(id=k,data_hex=v.hex())for k,v in frames])
        deadline=self.clock.monotonic()+.1
        fresh=self.read(moving=moving)
        while not self.advanced(measured,fresh)and self.clock.monotonic()<deadline:
            self.clock.sleep(.005);fresh=self.read(moving=moving)
        require(self.advanced(measured,fresh),"New complete feedback required before send")
        require(max(abs(a-b)for a,b in zip(fresh["q"],measured["q"]))<=.003
                and math.dist(fresh["pose"][:3],measured["pose"][:3])<=.0005
                and support.rotation_distance(fresh["pose"],measured["pose"])<=.003,"Pre-send slip")
        self.monitor(fresh,origin,original_target)
        self.node.piper.healthy(measured,allow_moving=moving)
        self.node.piper.healthy(fresh,allow_moving=moving)
        require(self.node.adopted and not self.node.rospy_probe.is_shutdown(),"Shutdown before send")
        ticket=dict(thread=threading.get_ident(),expected=list(frames),speed=1,in_comm=False,attempted=0,sent=0)
        with self.state_lock:
            if kind=="initial"and self.state["hold_requested"]:raise CancelBeforeDispatch()
            # Once this marker is committed, the worker completes the initial
            # transaction even if a request arrives between its CAN fragments.
            self.state["initial_transaction_started"]=True
        started=self.clock.time();self.node.piper.ticket=ticket
        try:
            self.node.piper.MotionCtrl_2(1,1,1,0,0,0)
            self.node.piper.JointCtrl(*raw)
            require(not self.node.piper.broken and not ticket["expected"]
                    and ticket["attempted"]==ticket["sent"]==4,"Partial or failed transaction; no hold retry")
        finally:
            self.node.piper.ticket=None
            receipt=dict(kind=kind,started_unix_s=started,finished_unix_s=self.clock.time(),
                         attempted_frames=ticket["attempted"],socket_send_returns=ticket["sent"],
                         target_raw=list(raw),dispatch_sample=fresh)
            with self.state_lock:
                self.state["receipts"].append(receipt)
                self.session["pending"]["attempted_frames"]=ticket["attempted"]
                self.session["attempted_frames"]=self.session.get("attempted_frames",0)+ticket["attempted"]
                self.save()
            self.record("send_receipt",durable=True,sequence=self.state["sequence"],**receipt)
        if kind=="initial":self.change(command_sent_unix_s=receipt["finished_unix_s"],phase="moving")
        else:self.change(phase="holding")
        return receipt

    def wait_arrival(self,before,target,after,*,allow_request,deadline):
        last,window=None,StableWindow()
        while self.clock.monotonic()<deadline:
            s=self.read(moving=True);self.monitor(s,before,target)
            self.record("feedback",stage="arrival",state=s)
            self.motion_history=[r for r in self.motion_history if s["stamps"][4]-r["stamps"][4]<=.2]
            if not self.motion_history or self.advanced(self.motion_history[-1],s):
                self.motion_history.append(copy.deepcopy(s))
            if allow_request and self.requested():return "request",s,None
            ready=(min(s["stamps"])>after and s["motion_status"]==0
                   and max(abs(a-b)for a,b in zip(s["q"],target))<=.003)
            if not ready:window,last=StableWindow(),None
            elif last is None or self.advanced(last,s):
                done=window.add(s,self.clock.monotonic());last=s
                if done:
                    self.node.piper.healthy(s)
                    return "arrived",s,dict(duration_s=window.samples[-1][0]-window.samples[0][0],
                                            new_feedback_groups=len(window.samples))
            self.clock.sleep(.01 if allow_request else .05)
        raise RuntimeError("Arrival/hold observation timeout; no automatic stop or retry")

    def finish(self,phase,after,window,**extra):
        with self.state_lock:
            # Retirement and completion are atomic with hold-request admission.
            require(not(phase=="completed"and self.state["hold_requested"]),"Hold request raced completion")
            result=dict(phase=phase,sequence=self.state["sequence"],after=after,stable_window=window,
                        general_stop_qualified=False,crash_safe_hold_verified=False,**extra)
            self.state.update(phase=phase,active=False,hold_confirmed=phase=="hold_confirmed",result=result)
            if self.session.get("stop_latched"):
                # Revoke software command adoption, not motor enable. The
                # inherited generic telemetry must also say accepts=false.
                self.node.adopted=False
            self.session["pending"]=None
            self.session["held_raw"]=list(self.state["target_raw"])if phase=="completed"else list(after["raw_q"])
            self.save()
            SessionFile(self.output/("action_%06d_result.json"%self.state["sequence"])).save(result)
        self.record("result",durable=True,**result)
        return result

    def fail(self,error):
        with self.state_lock:
            self.node.failed=str(error)
            self.session["failure"]=dict(error=str(error),unix_s=self.clock.time(),
                                          accepted_target_may_continue=any(r["attempted_frames"]for r in self.state["receipts"]))
            self.state.update(active=False,phase="failed",failure=str(error),result=None)
            self.save()
            SessionFile(self.output/("action_%06d_result.json"%self.state["sequence"])).save(self.state)
        self.record("failed",durable=True,sequence=self.state["sequence"],error=str(error),physical_recovery_sent=False)

    def execute(self,message):
        require(self.node.action_lock.acquire(False),"Another command is active")
        admitted=False
        try:
            with self.state_lock:
                require(self.node.adopted and not self.node.failed and not self.node.piper.broken,
                        "Driver unavailable")
                require(not self.session.get("failure")and not self.session.get("pending")
                        and not self.session.get("stop_latched"),"Failed/unresolved/held session cannot continue")
            first=self.read()
            requested,effective=decode_target(message,first,self.session["held_raw"])
            support.path_check(first,effective,self.limits,self.fk)
            self.ownership_check()
            with self.state_lock:
                self.node.command_sequence+=1
                seq=self.node.command_sequence
                service=self.namespace+"/hold_current/seq_%d_%s"%(seq,self.token)
                self.state.update(sequence=seq,active=True,phase="preflight",hold_service=service,
                                  hold_requested=False,hold_confirmed=False,command_sent_unix_s=None,
                                  before=first,target_raw=effective,requested_raw=requested,receipts=[],result=None,
                                  initial_transaction_started=False,last_refusal=None)
                self.node.active=True
                self.motion_history=[]
                self.session["pending"]=dict(sequence=seq,kind="preflight",attempted_frames=0)
                admitted=True;self.save()
            self.retained_services.append(self.service_factory(service,lambda: self.request_hold(seq,self.token)))
            self.record("action_admitted",durable=True,sequence=seq,hold_service=service,
                        requested_raw=requested,target_raw=effective,
                        other_axes_scope="Requested within feedback tolerance; frozen held targets unchanged")
            before=self.baseline()
            requested,effective=decode_target(message,before,self.session["held_raw"])
            plan=support.path_check(before,effective,self.limits,self.fk)
            self.change(before=before,target_raw=effective,requested_raw=requested)
            if self.requested():raise CancelBeforeDispatch()
            receipt=self.send_once(effective,before,before,plan["target"],"initial",moving=False)
            deadline=self.clock.monotonic()+120.
            disposition,current,window=self.wait_arrival(before,plan["target"],receipt["finished_unix_s"],
                                                         allow_request=True,deadline=deadline)
            if disposition=="arrived":
                with self.state_lock:
                    if not self.state["hold_requested"]:
                        return self.finish("completed",current,window,target_reached=True)
                # A request admitted at the last stable sample is handled below.
            near=(current["motion_status"]==0 and max(abs(a-b)for a,b in zip(current["q"],plan["target"]))<=.003)
            if near:
                _,after,window=self.wait_arrival(before,plan["target"],receipt["finished_unix_s"],
                                                 allow_request=False,deadline=self.clock.monotonic()+20.)
                return self.finish("already_completed",after,window,target_reached=True,hold_sent=False,
                                   interruption_demonstrated=False)
            hold_raw=list(effective);hold_raw[1]=current["raw_q"][1]
            recent_motion=support.Probe.moving_reference(self.motion_history,current)is not None
            progress=(before["raw_q"][1]-current["raw_q"][1])/1000
            remaining=(current["raw_q"][1]-effective[1])/1000
            require(min(before["raw_q"][1],effective[1])<=hold_raw[1]<=max(before["raw_q"][1],effective[1]),
                    "Measured J2 outside original goal interval; no blind hold")
            hold=support.path_check(current,hold_raw,self.limits,self.fk)
            held_receipt=self.send_once(hold_raw,current,before,plan["target"],"hold",moving=True)
            _,after,window=self.wait_arrival(before,hold["target"],held_receipt["finished_unix_s"],
                                             allow_request=False,deadline=self.clock.monotonic()+20.)
            separation=(after["raw_q"][1]-effective[1])/1000
            return self.finish("hold_confirmed",after,window,hold_sent=True,target_reached=False,
                               original_target_raw=effective,hold_target_raw=hold_raw,
                               progress_before_hold_deg=progress,remaining_before_hold_deg=remaining,
                               recent_measured_motion_before_hold=recent_motion,
                               final_distance_before_old_target_deg=separation,
                               interruption_demonstrated=recent_motion and progress>=.25 and remaining>=.35 and separation>=.2,
                               scope="Local healthy unloaded1percent J2 replacement; near-goal hold is not proof of interruption")
        except CancelBeforeDispatch:
            try:
                # No accepted target exists. Still verify fresh stationary feedback.
                after=self.baseline()
                window=dict(duration_s=3.,scope="At least three seconds freshly observed; no initial transmission")
                return self.finish("cancelled_before_dispatch",after,window,hold_sent=False,target_reached=False,
                                   interruption_demonstrated=False)
            except Exception as error:
                self.fail(error);raise
        except Exception as error:
            if admitted:
                self.fail(error)
            else:
                self.change(last_refusal=dict(error=str(error),unix_s=self.clock.time()))
                self.record("refused_before_admission",error=str(error))
            raise
        finally:
            self.node.piper.ticket=None
            self.node.active=False
            self.node.action_lock.release()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entry-config",required=True)
    parser.add_argument("--entry-output-dir",required=True)
    parser.add_argument("--empty-gripper-confirmed",required=True,action="store_true")
    args,remaining=parser.parse_known_args(argv)
    require(remaining and not remaining[0].startswith("-"),"Pinned vendor script required")
    require(hashlib.sha256(Path(support.__file__).read_bytes()).hexdigest()==SUPPORT_SHA,"Frozen support changed")
    require(hashlib.sha256(EVIDENCE_PATH.read_bytes()).hexdigest()==EVIDENCE_SHA,"Local probe evidence changed")
    require(json.loads(EVIDENCE_PATH.read_text())["success"]is True,"Required local probe has not passed")
    base=support.load_base();limits=support.checked_probe_limits(json.loads(Path(args.entry_config).read_text()))
    output=Path(args.entry_output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
    boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip();token=uuid.uuid4().hex
    with SessionFile(SESSION_ROOT/(boot+".json"))as store:
        require(store.load()is None,"Existing entry session must be manually reviewed; no restart bypass")
        session=dict(pending=None,failure=None,stop_latched=False,held_raw=None)
        store.save(session);original_factory=base.node_class;instances=[]
        def node_factory(vendor,rospy):
            Parent=original_factory(vendor,rospy)
            class Node(Parent):
                def __init__(self):
                    self.rospy_probe=rospy;self.executor=None
                    super().__init__()
                    import rosgraph
                    from std_srvs.srv import Trigger,TriggerResponse
                    def ownership():
                        base.binding()
                        speed=rospy.get_param("~speed_percent")
                        require(type(speed)is int and speed==1,"Only live speed1 accepted; no parameter writes")
                        pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                        require(len(dict(pubs).get(rospy.resolve_name("joint_ctrl_single"),[]))<=1,"Competing joint publishers")
                        for topic in ("pos_cmd","enable_flag"):
                            require(not dict(pubs).get(rospy.resolve_name(topic)),"Unsupported command publisher present")
                    def services(name,call):
                        def handle(_):
                            accepted,data=call()
                            return TriggerResponse(success=accepted,message=json.dumps(data,allow_nan=False))
                        return rospy.Service(name,Trigger,handle)
                    identity=dict(boot_id=boot,pid=os.getpid(),adoption_token=token,adopted_unix_s=time.time(),
                                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                  base_sha256=support.BASE_SHA,support_sha256=SUPPORT_SHA,
                                  local_evidence_sha256=EVIDENCE_SHA,vendor_sha256=base.DRIVER_SHA256,
                                  config_sha256=hashlib.sha256(Path(args.entry_config).read_bytes()).hexdigest(),
                                  can_interface=base.CAN_NAME,usb_interface=base.USB_INTERFACE,
                                  empty_gripper_operator_confirmed=True)
                    ownership();session["identity"]=identity
                    control=Interruptible(self,base,limits,support.manufacturer_fk(),store,session,output,identity,ownership,
                                          service_factory=services,namespace=rospy.get_name(),token=token)
                    initial=control.read();session["held_raw"]=list(initial["raw_q"])
                    self.executor=control;instances.append(control);control.save()
                    self.hold_status_service=services("~hold_status",lambda:(True,control.status()))
                    control.record("ready",durable=True,identity=identity,actuator_frames=0,sdk_init_queries=0)
                def joint_callback(self,msg):
                    require(self.executor is not None,"Read-only adoption incomplete")
                    return self.executor.execute(msg)
                def unsupported(self,*a,**k):
                    raise RuntimeError("Only unloaded1percent J2 toward-zero commands and bound hold are supported")
                pos_callback=unsupported
                handle_gripper_service=unsupported
            return Node
        base.node_class=node_factory;sys.argv=[str(Path(__file__))]+remaining
        try:base.main()
        finally:
            for instance in instances:instance.feedback.close()


if __name__=="__main__":main()
