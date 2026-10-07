#!/usr/bin/env python3
"""Single ROS J worker with fresh full-joint hold and reviewed generation resume.

One initial coupled commissioning action; no P/L/task algorithm. No imports
connect to hardware. Old sessions and frozen sources are never rewritten.
"""
import argparse
import copy
from contextlib import contextmanager, ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"scripts"))
import ros_interruptible_joint_trial_entry as predecessor

v1=predecessor.v1; support=v1.support; require=support.require
SessionFile=v1.SessionFile; StableWindow=v1.StableWindow
TRIAL_SHA="db42853ba93817b9225f0cd3b70429c037bd10d28417d2740b3940bc07918f80"
SUCCESS_SESSION_SHA="c68cda74e15f5535db9e05c452474f82aea44575318afea3f3b70a4d31325140"
SUCCESS_RESULT=ROOT/"runs/stop_integration_retest_20261006_154450/runtime/action_000001_result.json"
SUCCESS_RESULT_SHA="c3763f57ca2780f2285910e8c740c067930c246734a67f36c08ace8f19d8731b"
AUTHORIZATION="请将机械臂归零后开始执行任务可以么，直到任务完成前都不要结束对话"


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def decode(message,before,session):
    require(len(message.position)==6 and not message.effort,"Exactly six positions; no effort commands")
    require(len(message.velocity)==7 and list(message.velocity[:6])==[0]*6
            and type(message.velocity[6])in(int,float)and message.velocity[6]==1,"Explicit1percent required")
    require(all(type(v)in(int,float)and math.isfinite(v)for v in message.position),"Finite positions required")
    raw=[round(v*(1000*180/math.pi))for v in message.position]
    support.frames_for(raw)
    require(raw!=before["raw_q"],"No encoded movement")
    if session["stage"]=="commissioning":
        require(not session["commissioning_attempted"],"Commissioning opportunity already consumed")
        anchor=session["held_raw"]
        require(max(abs(a-b)*support.RAD_PER_RAW for a,b in zip(anchor,before["raw_q"]))<=.003,
                "Commissioning anchor drift")
        i=max(range(6),key=lambda k:abs(anchor[k]));require(anchor[i]!=0,"Already zero")
        scale=raw[i]/anchor[i]
        require(0<=scale<1 and all(abs(v-round(a*scale))<=1 for a,v in zip(anchor,raw)),
                "Commissioning requires a common toward-zero ratio")
        require(all(min(0,a)<=v<=max(0,a)for a,v in zip(anchor,raw)),"Commissioning must approach zero")
        require(max(abs(a-b)for a,b in zip(before["raw_q"],raw))<=1000,"Commissioning joint ceiling1degree")
        require(sum(abs(a-b)>=200 for a,b in zip(before["raw_q"],raw))>=2,"Coupled commissioning needs two moving axes")
    else:
        require(session["stage"]=="task","Unknown stage")
    return raw


def slip(a,b):
    require(max(abs(x-y)for x,y in zip(a["q"],b["q"]))<=.003
            and math.dist(a["pose"][:3],b["pose"][:3])<=.0005
            and support.rotation_distance(a["pose"],b["pose"])<=.003,"Pre-send slip from latest plan")


def motion_evidence(history,origin,target,current):
    rows=[]
    for i,(start,goal,now)in enumerate(zip(origin["raw_q"],target,current["raw_q"])):
        sign=1 if goal>start else -1
        fragment=4+i//2
        candidates=[s for s in history if .05<=current["stamps"][fragment]-s["stamps"][fragment]<=.15]
        reference=min(candidates,key=lambda s:abs(current["stamps"][fragment]-s["stamps"][fragment]-.1))if candidates else None
        trend=reference is not None and sign*(now-reference["raw_q"][i])*support.RAD_PER_RAW>=.0005
        rows.append(dict(axis=i+1,progress_deg=sign*(now-start)/1000,
                         remaining_deg=sign*(goal-now)/1000,recent_motion=bool(trend)))
    return rows


class GuardedTask(v1.Interruptible):
    def __init__(self,*a,**k):
        super().__init__(*a,**k)
        self.original=None;self.bound_hold=None
        self.state.update(generation=self.session["generation"],stage=self.session["stage"],
                          resume_service=None,result_sha256=None)

    def monitor(self,s,origin,target):
        super().monitor(s,origin,target)
        if self.original is not None:
            super().monitor(s,self.original[0],self.original[1])

    def send_hold(self,measured):
        origin,target=self.original
        require(self.node.piper.ticket is None and self.bound_hold is None,"Prior/duplicate transaction")
        self.ownership_check()
        with self.state_lock:
            require(self.state["active"]and self.state["hold_requested"]and not self.state["failure"],"No active hold request")
            self.session["pending"]=dict(kind="hold",sequence=self.state["sequence"],target_raw=None,
                                          attempted_frames=0,target_policy="All six latest measured joints")
            self.state["phase"]="hold_sending";self.save()
        self.record("hold_preparation",durable=True,sequence=self.state["sequence"],request_sample=measured)
        deadline=self.clock.monotonic()+.1;planned=self.read(moving=True)
        while not self.advanced(measured,planned)and self.clock.monotonic()<deadline:
            self.clock.sleep(.005);planned=self.read(moving=True)
        require(self.advanced(measured,planned),"New complete feedback required for hold")
        self.monitor(planned,origin,target)
        raw=list(planned["raw_q"]);path=support.path_check(planned,raw,self.limits,self.fk)
        frames=support.frames_for(raw)
        evidence=motion_evidence(self.motion_history,origin,self.state["target_raw"],planned)
        with self.state_lock:
            witness=self.read(moving=True)
            require(witness["sequence"]>=planned["sequence"]and all(a>=b for a,b in zip(witness["stamps"],planned["stamps"])),"Feedback regressed")
            slip(planned,witness);self.monitor(witness,origin,target);self.monitor(witness,planned,path["target"])
            self.node.piper.healthy(planned,allow_moving=True);self.node.piper.healthy(witness,allow_moving=True)
            require(self.node.adopted and not self.node.rospy_probe.is_shutdown(),"Shutdown before hold")
            ticket=dict(thread=threading.get_ident(),expected=list(frames),speed=1,in_comm=False,attempted=0,sent=0)
            started=self.clock.time();self.node.piper.ticket=ticket
            self.bound_hold=dict(target_raw=raw,target=path["target"],plan_sample=planned,dispatch_sample=witness,
                                 path=path,motion=evidence)
        try:
            self.node.piper.MotionCtrl_2(1,1,1,0,0,0);self.node.piper.JointCtrl(*raw)
            require(not self.node.piper.broken and not ticket["expected"]and ticket["attempted"]==ticket["sent"]==4,
                    "Partial hold transaction; no retry")
        finally:
            self.node.piper.ticket=None
            receipt=dict(kind="hold",started_unix_s=started,finished_unix_s=self.clock.time(),
                         attempted_frames=ticket["attempted"],socket_send_returns=ticket["sent"],
                         target_raw=raw,plan_sample=planned,dispatch_sample=witness,path=path,
                         frames=[dict(id=k,data_hex=v.hex())for k,v in frames])
            with self.state_lock:
                self.state["receipts"].append(receipt)
                self.session["pending"].update(target_raw=raw,attempted_frames=ticket["attempted"])
                self.session["attempted_frames"]=self.session.get("attempted_frames",0)+ticket["attempted"]
                self.save()
            self.record("send_receipt",durable=True,sequence=self.state["sequence"],**receipt)
        self.change(phase="holding")
        return receipt

    def wait_arrival(self,before,target,after,*,allow_request,deadline):
        last,window=None,StableWindow();samples=[]
        while self.clock.monotonic()<deadline:
            s=self.read(moving=True);self.monitor(s,before,target)
            self.record("feedback",stage="arrival"if allow_request else"hold",state=s)
            self.motion_history=[r for r in self.motion_history if min(s["stamps"])-min(r["stamps"])<=.2]
            if not self.motion_history or self.advanced(self.motion_history[-1],s):self.motion_history.append(copy.deepcopy(s))
            if min(s["stamps"])>after:samples.append(s)
            if allow_request and self.requested():return "request",s,None
            ready=(min(s["stamps"])>after and s["motion_status"]==0
                   and max(abs(a-b)for a,b in zip(s["q"],target))<=.003)
            if not ready:window,last=StableWindow(),None
            elif last is None or self.advanced(last,s):
                done=window.add(s,self.clock.monotonic());last=s
                if done:
                    self.node.piper.healthy(s)
                    return "arrived",s,dict(duration_s=window.samples[-1][0]-window.samples[0][0],
                        new_feedback_groups=len(window.samples),post_dispatch_samples=samples)
            self.clock.sleep(.01 if allow_request else .05)
        raise RuntimeError("Arrival/hold timeout; accepted goal not automatically cancelled")

    def finish(self,phase,after,window,**extra):
        with self.state_lock:
            window={k:v for k,v in window.items()if k!="post_dispatch_samples"}
            result=super().finish(phase,after,window,generation=self.session["generation"],stage=self.session["stage"],**extra)
            path=self.output/("action_%06d_result.json"%self.state["sequence"])
            digest=sha(path);self.state["result_sha256"]=digest
            eligible=(phase=="hold_confirmed"and (self.session["stage"]=="task"or result.get("coupled_interruption_demonstrated")is True))
            if eligible:
                seq=self.state["sequence"];generation=self.session["generation"]
                service=self.namespace+"/resume_after_review/seq_%d_gen_%d_%s_%s"%(seq,generation,self.token,digest)
                self.state["resume_service"]=service
                self.retained_services.append(self.service_factory(service,lambda:self.resume(seq,generation,digest)))
            # A commissioning run cannot silently become ordinary operation after natural arrival.
            if self.session["stage"]=="commissioning":self.node.adopted=False
            self.save()
        return result

    def resume(self,sequence,generation,digest):
        require(self.node.action_lock.acquire(False),"Action still active")
        admitted=False
        try:
            with self.state_lock:
                require(sequence==self.state["sequence"]and generation==self.session["generation"]
                        and digest==self.state["result_sha256"],"Retired resume request")
                require(not self.state["active"]and self.state["phase"]=="hold_confirmed"
                        and self.session["stop_latched"]and not self.session["failure"]and not self.session["pending"]
                        and not self.node.failed and not self.node.piper.broken,"Only successful resolved hold can resume")
                result_path=self.output/("action_%06d_result.json"%sequence)
                require(sha(result_path)==digest,"Reviewed result changed")
                result=self.state["result"]
                require(self.session["stage"]=="task"or result.get("coupled_interruption_demonstrated")is True,
                        "Coupled commissioning not demonstrated")
                require(len(self.state["receipts"])==2 and all(r["attempted_frames"]==r["socket_send_returns"]==4 for r in self.state["receipts"]),"Incomplete receipt")
                admitted=True;self.node.active=True
            self.ownership_check();fresh=self.baseline();slip(result["after"],fresh)
            require(fresh["jaw_code"]==result["after"]["jaw_code"]and abs(fresh["opening_m"]-result["after"]["opening_m"])<=.0005,"Jaw changed since reviewed hold")
            self.node.piper.healthy(fresh)
            require(not self.node.rospy_probe.is_shutdown(),"Shutdown during resume")
            with self.state_lock:
                old=dict(generation=generation,stop_latched=True,sequence=sequence,result_sha256=digest,
                         result=copy.deepcopy(result),reviewed_unix_s=self.clock.time(),fresh_state=fresh)
                self.session["generations"].append(old)
                self.session.update(generation=generation+1,stage="task",stop_latched=False,held_raw=list(fresh["raw_q"]))
                self.original=None;self.bound_hold=None
                self.state.update(generation=generation+1,stage="task",phase="idle",stop_latched=False,
                                  hold_requested=False,hold_confirmed=False,hold_service=None,resume_service=None,
                                  result=None,result_sha256=None,receipts=[],before=None,target_raw=None,requested_raw=None)
                self.save()  # Persist new software generation before granting command adoption.
                self.node.adopted=True
            self.record("reviewed_resume",durable=True,old_generation=generation,new_generation=generation+1,
                        reviewed_result_sha256=digest,actuator_frames=0)
            return True,self.status()
        except Exception as error:
            if admitted:self.fail(error)
            raise
        finally:self.node.active=False;self.node.action_lock.release()

    def gripper(self,request):
        """Explicit idle jaw request: stable feedback is not proof of grasp/release."""
        require(self.node.action_lock.acquire(False),"Another command is active")
        admitted=False
        try:
            require(self.session["stage"]=="task","Gripper forbidden before reviewed commissioning")
            require(self.node.adopted and not self.node.failed and not self.node.piper.broken,"Driver unavailable")
            require(not self.session["failure"]and not self.session["pending"]and not self.session["stop_latched"],"Unresolved/held session")
            width=request.gripper_angle;effort=request.gripper_effort
            require(type(width)in(int,float)and math.isfinite(width)
                    and self.limits["gripper_min_m"]<=width<=self.limits["gripper_max_m"],"Jaw target outside explicit0..55mm limits")
            require(type(effort)in(int,float)and math.isfinite(effort)and effort==.2
                    and type(request.gripper_code)is int and request.gripper_code==1
                    and type(request.set_zero)is int and request.set_zero==0,"Jaw requires effort.2/code1/setzero0")
            raw=round(width*1e6);target=raw/1e6
            require(self.limits["gripper_min_m"]<=target<=self.limits["gripper_max_m"],"Encoded jaw target outside limits")
            self.ownership_check();first=self.read()  # Already-enabled jaw required by frozen healthy().
            with self.state_lock:
                self.node.command_sequence+=1;seq=self.node.command_sequence
                self.state.update(sequence=seq,kind="gripper",active=True,phase="gripper_preflight",
                    hold_service=None,hold_requested=False,hold_confirmed=False,command_sent_unix_s=None,
                    before=first,target_raw=list(first["raw_q"]),requested_raw=None,jaw_target_m=target,
                    receipts=[],result=None,result_sha256=None,resume_service=None,last_refusal=None)
                self.node.active=True;self.original=None;self.bound_hold=None
                self.session["pending"]=dict(sequence=seq,kind="gripper",target_m=target,attempted_frames=0)
                admitted=True;self.save()
            before=self.baseline()
            self.change(before=before,target_raw=list(before["raw_q"]))
            frame=(0x159,struct.pack(">iHBB",raw,200,1,0))
            self.ownership_check()
            self.record("gripper_intent",durable=True,sequence=seq,before=before,jaw_target_m=target,
                        effort_parameter=.2,frame=dict(id=frame[0],data_hex=frame[1].hex()),
                        already_enabled_required=True,grasp_verified=False)
            deadline=self.clock.monotonic()+.1;fresh=self.read()
            while not self.advanced(before,fresh)and self.clock.monotonic()<deadline:
                self.clock.sleep(.005);fresh=self.read()
            require(self.advanced(before,fresh),"New complete feedback required before jaw send")
            slip(before,fresh)
            require(abs(before["opening_m"]-fresh["opening_m"])<=.0005,"Jaw drift before dispatch")
            self.node.piper.healthy(before);self.node.piper.healthy(fresh)
            require(self.node.adopted and not self.node.rospy_probe.is_shutdown()and self.node.piper.ticket is None,"Jaw dispatch unavailable")
            ticket=dict(thread=threading.get_ident(),expected=[frame],speed=1,in_comm=False,attempted=0,sent=0)
            started=self.clock.time();self.node.piper.ticket=ticket
            try:
                self.node.piper.GripperCtrl(raw,200,1,0)
                require(not self.node.piper.broken and not ticket["expected"]and ticket["attempted"]==ticket["sent"]==1,
                        "Incomplete jaw transaction; no retry")
            finally:
                self.node.piper.ticket=None
                receipt=dict(kind="gripper",started_unix_s=started,finished_unix_s=self.clock.time(),
                    attempted_frames=ticket["attempted"],socket_send_returns=ticket["sent"],jaw_target_m=target,
                    dispatch_sample=fresh,frames=[dict(id=frame[0],data_hex=frame[1].hex())])
                with self.state_lock:
                    self.state["receipts"].append(receipt)
                    self.session["pending"]["attempted_frames"]=ticket["attempted"]
                    self.session["attempted_frames"]=self.session.get("attempted_frames",0)+ticket["attempted"]
                    self.save()
                self.record("send_receipt",durable=True,sequence=seq,**receipt)
            self.change(phase="gripper_observing",command_sent_unix_s=receipt["finished_unix_s"])
            window,last=StableWindow(),None;deadline=self.clock.monotonic()+20.
            while self.clock.monotonic()<deadline:
                s=self.read()  # Arm must remain idle/healthy; jaw may approach contact.
                require(max(abs(a-b)for a,b in zip(s["q"],before["q"]))<=.003
                        and math.dist(s["pose"][:3],before["pose"][:3])<=.002
                        and support.rotation_distance(s["pose"],before["pose"])<=.003,"Arm drift during jaw action")
                require(s["jaw_code"]==before["jaw_code"],"Jaw enable/fault state changed")
                self.record("feedback",stage="gripper",state=s)
                if min(s["stamps"])>receipt["finished_unix_s"]and(last is None or self.advanced(last,s)):
                    done=window.add(s,self.clock.monotonic());last=s
                    if done:
                        self.node.piper.healthy(s)
                        error=s["opening_m"]-target
                        self.session["jaw_command_target_m"]=target
                        return self.finish("completed",s,dict(duration_s=window.samples[-1][0]-window.samples[0][0],
                            new_feedback_groups=len(window.samples)),kind="gripper",jaw_target_m=target,
                            jaw_width_error_m=error,jaw_target_reached=abs(error)<=.0015,
                            jaw_target_report_tolerance_m=.0015,grasp_verified=False,release_verified=False,
                            service_status_scope="request_sent_and_feedback_stable_not_grasp_or_release")
                self.clock.sleep(.05)
            raise RuntimeError("Jaw feedback stability timeout; no retry or automatic opening")
        except Exception as error:
            if admitted:self.fail(error)
            else:self.change(last_refusal=dict(error=str(error),unix_s=self.clock.time()))
            raise
        finally:self.node.piper.ticket=None;self.node.active=False;self.node.action_lock.release()

    def execute(self,message):
        require(self.node.action_lock.acquire(False),"Another command is active")
        admitted=False
        try:
            require(self.node.adopted and not self.node.failed and not self.node.piper.broken,"Driver unavailable")
            require(not self.session["failure"]and not self.session["pending"]and not self.session["stop_latched"],"Unresolved/held session")
            first=self.read();raw=decode(message,first,self.session);support.path_check(first,raw,self.limits,self.fk)
            self.ownership_check()
            with self.state_lock:
                self.node.command_sequence+=1;seq=self.node.command_sequence
                service=self.namespace+"/hold_current/seq_%d_%s"%(seq,self.token)
                self.state.update(sequence=seq,kind="joint",active=True,phase="preflight",hold_service=service,
                    hold_requested=False,hold_confirmed=False,command_sent_unix_s=None,before=first,
                    target_raw=raw,requested_raw=raw,receipts=[],result=None,result_sha256=None,resume_service=None,
                    initial_transaction_started=False,last_refusal=None)
                self.node.active=True;self.motion_history=[];self.original=None;self.bound_hold=None
                self.session["pending"]=dict(sequence=seq,kind="preflight",attempted_frames=0)
                admitted=True;self.save()
            self.retained_services.append(self.service_factory(service,lambda:self.request_hold(seq,self.token)))
            self.record("action_admitted",durable=True,sequence=seq,target_raw=raw,hold_service=service,stage=self.session["stage"])
            before=self.baseline();raw=decode(message,before,self.session)
            plan=support.path_check(before,raw,self.limits,self.fk);self.original=(before,plan["target"])
            if self.session["stage"]=="commissioning":self.session["commissioning_attempted"]=True
            self.change(before=before,target_raw=raw,requested_raw=raw)
            if self.requested():raise v1.CancelBeforeDispatch()
            receipt=self.send_once(raw,before,before,plan["target"],"initial",moving=False)
            disposition,current,window=self.wait_arrival(before,plan["target"],receipt["finished_unix_s"],allow_request=True,deadline=self.clock.monotonic()+120.)
            if disposition=="arrived":
                with self.state_lock:
                    if not self.state["hold_requested"]:return self.finish("completed",current,window,target_reached=True)
            if current["motion_status"]==0 and max(abs(a-b)for a,b in zip(current["q"],plan["target"]))<=.003:
                _,after,window=self.wait_arrival(before,plan["target"],receipt["finished_unix_s"],allow_request=False,deadline=self.clock.monotonic()+20.)
                return self.finish("already_completed",after,window,target_reached=True,hold_sent=False,interruption_demonstrated=False)
            held=self.send_hold(current);bound=self.bound_hold
            _,after,window=self.wait_arrival(bound["plan_sample"],bound["target"],held["finished_unix_s"],allow_request=False,deadline=self.clock.monotonic()+20.)
            axes=[]
            for row in bound["motion"]:
                i=row["axis"]-1;row=dict(row,final_separation_deg=abs(after["raw_q"][i]-raw[i])/1000)
                row["demonstrated"]=row["recent_motion"]and row["progress_deg"]>=.2 and row["remaining_deg"]>=.2 and row["final_separation_deg"]>=.2
                axes.append(row)
            samples=window["post_dispatch_samples"];dispatch=bound["dispatch_sample"]
            maxima=dict(joint_rad=[max([abs(s["q"][i]-dispatch["q"][i])for s in samples]or[0])for i in range(6)],
                        end_reference_m=max([math.dist(s["pose"][:3],dispatch["pose"][:3])for s in samples]or[0]),
                        scope="Observed discrete feedback samples, not a continuous physical stopping bound")
            return self.finish("hold_confirmed",after,window,hold_sent=True,target_reached=False,
                hold_target_raw=bound["target_raw"],hold_plan_sample=bound["plan_sample"],hold_dispatch_sample=dispatch,
                original_target_raw=raw,axes=axes,interruption_demonstrated=any(x["demonstrated"]for x in axes),
                coupled_interruption_demonstrated=sum(x["demonstrated"]for x in axes)>=2,observed_post_hold_motion=maxima)
        except v1.CancelBeforeDispatch:
            try:
                after=self.baseline()
                return self.finish("cancelled_before_dispatch",after,dict(duration_s=3.),hold_sent=False,interruption_demonstrated=False)
            except Exception as error:self.fail(error);raise
        except Exception as error:
            if admitted:self.fail(error)
            else:self.change(last_refusal=dict(error=str(error),unix_s=self.clock.time()))
            raise
        finally:self.node.piper.ticket=None;self.node.active=False;self.node.action_lock.release()


@contextmanager
def reserve(boot,reviewed,*,session_root=None):
    root=Path(session_root)if session_root is not None else v1.SESSION_ROOT
    require(boot==predecessor.PARENT_BOOT,"Different boot requires separate reviewed adoption")
    paths=[root/(boot+".json"),root/predecessor.CHILD_NAME]
    expected=[predecessor.PARENT_SHA,SUCCESS_SESSION_SHA]
    with ExitStack()as stack:
        originals=[]
        for path,digest in zip(paths,expected):
            require(path.is_file(),"Missing prior session")
            stack.enter_context(SessionFile(path));content=path.read_bytes();originals.append(content)
            require(hashlib.sha256(content).hexdigest()==digest,"Prior session hash changed")
        parent=json.loads(originals[1]);status=parent["status"]
        require(reviewed.get("parent_success_session_sha256")==SUCCESS_SESSION_SHA
                and reviewed.get("parent_success_result_sha256")==SUCCESS_RESULT_SHA
                and reviewed.get("user_authorization")==AUTHORIZATION,"Explicit successful-parent task review required")
        require(parent["trial_closed"]and parent["stop_latched"]and not parent["failure"]and parent["pending"]is None
                and status["phase"]=="hold_confirmed"and status["result"]["interruption_demonstrated"]is True,
                "Parent did not complete verified hold")
        store=stack.enter_context(SessionFile(root/("guarded_task_"+boot+".json")))
        require(store.load()is None,"Guarded task session already exists; no restart bypass")
        session=dict(parent_success_sha256=SUCCESS_SESSION_SHA,pending=None,failure=None,stop_latched=False,
                     held_raw=None,generation=0,generations=[],stage="commissioning",commissioning_attempted=False)
        store.save(session)
        try:yield parent,store,session
        finally:
            require(all(p.read_bytes()==b for p,b in zip(paths,originals)),"Previous session was modified")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entry-config",required=True);parser.add_argument("--entry-output-dir",required=True)
    parser.add_argument("--empty-gripper-confirmed",action="store_true",required=True)
    args,remaining=parser.parse_known_args(argv)
    require(remaining and not remaining[0].startswith("-"),"Pinned vendor script required")
    require(sha(predecessor.__file__)==TRIAL_SHA and sha(predecessor.core.__file__)==predecessor.V2_SHA
            and sha(v1.__file__)==predecessor.core.V1_SHA and sha(support.__file__)==v1.SUPPORT_SHA,"Frozen dependency changed")
    require(sha(SUCCESS_RESULT)==SUCCESS_RESULT_SHA,"Successful parent result changed")
    config_path=Path(args.entry_config).resolve();review_path=config_path.parent/"reviewed_parent.json"
    reviewed=json.loads(review_path.read_text());limits=support.checked_probe_limits(json.loads(config_path.read_text()))
    boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip();token=uuid.uuid4().hex
    base=support.load_base();instances=[]
    with reserve(boot,reviewed)as(parent,store,session):
        output=Path(args.entry_output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
        original_factory=base.node_class
        def factory(vendor,rospy):
            Parent=original_factory(vendor,rospy)
            class TaskNode(Parent):
                def __init__(self):
                    self.rospy_probe=rospy;self.executor=None;super().__init__()
                    import rosgraph
                    from std_srvs.srv import Trigger,TriggerResponse
                    def ownership():
                        base.binding();speed=rospy.get_param("~speed_percent")
                        require(type(speed)is int and speed==1,"Only live1percent allowed")
                        pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                        require(len(dict(pubs).get(rospy.resolve_name("joint_ctrl_single"),[]))<=1,"Competing joint publishers")
                        for topic in ("pos_cmd","enable_flag"):
                            require(not dict(pubs).get(rospy.resolve_name(topic)),"Unsupported publisher")
                    def services(name,call):
                        def handler(_):
                            try:accepted,data=call()
                            except Exception as error:return TriggerResponse(success=False,message=json.dumps(dict(error=str(error))))
                            return TriggerResponse(success=accepted,message=json.dumps(data,allow_nan=False))
                        return rospy.Service(name,Trigger,handler)
                    identity=dict(boot_id=boot,pid=os.getpid(),adoption_token=token,adopted_unix_s=time.time(),
                        source_sha256=sha(__file__),base_sha256=support.BASE_SHA,v1_sha256=predecessor.core.V1_SHA,
                        v2_sha256=predecessor.V2_SHA,trial_sha256=TRIAL_SHA,support_sha256=v1.SUPPORT_SHA,
                        vendor_sha256=base.DRIVER_SHA256,config_sha256=sha(config_path),reviewed_parent_sha256=sha(review_path),
                        parent_success_session_sha256=SUCCESS_SESSION_SHA,parent_success_result_sha256=SUCCESS_RESULT_SHA,
                        task_session_path=str(store.path),can_interface=base.CAN_NAME,usb_interface=base.USB_INTERFACE,
                        user_authorization=AUTHORIZATION,empty_gripper_operator_confirmed=True)
                    ownership();session["identity"]=identity
                    executor=GuardedTask(self,base,limits,support.manufacturer_fk(),store,session,output,identity,ownership,
                                        service_factory=services,namespace=rospy.get_name(),token=token)
                    instances.append(executor);initial=executor.read();previous=parent["status"]["result"]["after"]
                    slip(previous,initial)
                    require(initial["jaw_code"]==previous["jaw_code"]and abs(initial["opening_m"]-previous["opening_m"])<=.0005,"Parent jaw changed")
                    session["held_raw"]=list(initial["raw_q"]);self.executor=executor;executor.save()
                    self.hold_status_service=services("~hold_status",lambda:(True,executor.status()))
                    executor.record("ready",durable=True,identity=identity,actuator_frames=0,sdk_init_queries=0,stage="commissioning")
                def joint_callback(self,message):
                    require(self.executor is not None,"Adoption incomplete");return self.executor.execute(message)
                def handle_gripper_service(self,request):
                    require(self.executor is not None and self.gripper_exist is True,"Adopted existing gripper required")
                    self.executor.gripper(request)
                    response=vendor.GripperResponse();response.code=15900;response.status=True
                    return response
                def denied_task(self,*a,**k):raise RuntimeError("Only guarded1percent J and idle explicit jaw; P/L disabled")
                pos_callback=denied_task
            return TaskNode
        base.node_class=factory;sys.argv=[str(Path(__file__))]+remaining
        try:base.main()
        finally:
            for instance in instances:instance.feedback.close()


if __name__=="__main__":main()
