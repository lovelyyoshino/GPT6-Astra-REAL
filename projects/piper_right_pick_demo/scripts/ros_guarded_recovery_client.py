#!/usr/bin/env python3
"""One ROS command or explicit recovery review; never an automatic task loop."""
import json
import math
from pathlib import Path
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"scripts"))
import ros_guarded_task_client as previous
import ros_guarded_recovery_entry as recovery

CLIENT_SHA="951c1747fd7344f09640cc143b499a9955e7c711cf5391cd9a7af4f4a9461d1d"
frozen=previous.frozen;support=previous.support;require=previous.require


def shoulder_target(before,limits,fk,max_joint_deg=.8,anchor=None):
    require(math.isfinite(max_joint_deg)and 0<max_joint_deg<=.8,"Separated home ceiling must be within0..0.8degree")
    origin=list(before["raw_q"]if anchor is None else anchor)
    require(origin[1]>0 and origin[2]<0,"Shoulders already zero/noninterior; select later wrist goals explicitly")
    upper=min(1.,max_joint_deg*1000/max(abs(origin[1]),abs(origin[2])))
    def candidate(scale):
        raw=list(recovery.FIXED_TARGET)
        raw[1]=round(origin[1]*(1-scale));raw[2]=round(origin[2]*(1-scale))
        require(0<before["raw_q"][1]-raw[1]<=max_joint_deg*1000
                and 0<raw[2]-before["raw_q"][2]<=max_joint_deg*1000,"Shoulder step outside limit")
        return dict(target_raw=raw,scale=scale,path=support.path_check(before,raw,limits,fk),speed_percent=1,
                    strategy="J2/J3 ratio; other four original seq52 targets unchanged")
    low,high,best=0.,upper,None
    for i in range(24):
        scale=upper if i==0 else(low+high)/2
        try:plan=candidate(scale)
        except RuntimeError as error:
            if str(error)not in("Complete joint box exceeds original motion bounds","Complete joint box exceeds workspace",
                "End-reference segment exceeds original 30mm/.05rad bounds","Shoulder step outside limit"):
                raise
            high=scale;continue
        low,best=scale,plan
        if scale==upper or high-low<1e-7:break
    require(best is not None,"No valid separated shoulder segment")
    if best["scale"]<1.:best=candidate(best["scale"]*.95)
    return best


class RecoveryTransport(previous.TaskTransport):
    def __init__(self,config,runtime):
        import rospy,rosgraph,rosnode
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from xmlrpc.client import ServerProxy
        require(recovery.sha(previous.__file__)==CLIENT_SHA and recovery.sha(frozen.__file__)==previous.CLIENT_SHA,"Frozen client changed")
        self.rospy,self.message,self.trigger=rospy,JointState,Trigger
        self.config,self.runtime=Path(config).resolve(),Path(runtime).resolve()
        self.master=rosgraph.Master(rospy.get_name())
        boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.session_path=recovery.guard.v1.SESSION_ROOT/recovery.CHILD_NAME
        self.session=json.loads(self.session_path.read_text());self.identity=self.session["identity"]
        entry=Path(recovery.__file__).resolve()
        require(self.identity["boot_id"]==boot and recovery.sha(entry)==self.identity["source_sha256"]
                and self.identity["guard_sha256"]==recovery.GUARD_SHA
                and recovery.sha(recovery.guard.__file__)==recovery.GUARD_SHA,"Recovery source/boot changed")
        require(recovery.sha(self.config)==self.identity["config_sha256"]==recovery.CONFIG_SHA
                and self.identity["task_session_path"]==str(self.session_path),"Recovery config/session changed")
        parent=recovery.guard.v1.SESSION_ROOT/("guarded_task_"+boot+".json")
        require(recovery.sha(parent)==self.identity["parent_session_sha256"]==recovery.PARENT_SHA,"Parent failure changed")
        uri=rosnode.get_api_uri(self.master,frozen.NODE);require(uri,"Recovery driver absent")
        pid=ServerProxy(uri).getPid(rospy.get_name())[2];require(pid==self.identity["pid"],"Recovery PID mismatch")
        argv=(Path("/proc")/str(pid)/"cmdline").read_bytes().split(b"\0")
        for value in (str(entry),"--entry-config",str(self.config),"--entry-output-dir",str(self.runtime)):
            require(value.encode()in argv,"Recovery process arguments changed")
        pubs,subs,_=self.master.getSystemState()
        require(dict(subs).get(frozen.TOPIC)==[frozen.NODE],"Joint subscriber not exclusive")
        for topic in(frozen.TOPIC,"/piper/right/pos_cmd","/piper/right/enable_flag"):
            require(not dict(pubs).get(topic),"Competing publisher")
        require(dict(pubs).get("/piper/right/eval_telemetry")==[frozen.NODE],"Telemetry owner mismatch")
        require(rospy.get_param(frozen.NODE+"/speed_percent")==1,"Only1percent allowed")
        self.publisher=rospy.Publisher(frozen.TOPIC,JointState,queue_size=1,latch=False)
        deadline=time.monotonic()+3.
        while not self.publisher.get_num_connections()and time.monotonic()<deadline:time.sleep(.02)
        require(self.publisher.get_num_connections()==1,"Joint connection unavailable")
        previous.home_target=self.home_plan

    def home_plan(self,before,limits,fk,max_joint_deg=.8,anchor=None):
        anchor=self.session["pilot_anchor_raw"]if self.session["recovery_stage"]=="pilot"else None
        return shoulder_target(before,limits,fk,max_joint_deg,anchor)

    def resume(self,digest):
        state=self.status()
        if state.get("recovery_stage")=="approved":return super().resume(digest)
        require(state.get("recovery_stage")=="awaiting_review"and state["phase"]=="completed",
                "Separated pilot has not completed")
        path=self.runtime/("action_%06d_result.json"%state["sequence"])
        require(recovery.sha(path)==digest==state["result_sha256"],"Pilot review SHA mismatch")
        service=state["recovery_review_service"]
        expected=frozen.NODE+"/review_recovery/seq_%d_gen_%d_%s_%s"%(state["sequence"],state["generation"],state["adoption_token"],digest)
        require(service==expected,"Pilot review service mismatch")
        self.rospy.wait_for_service(service,timeout=1.);output=[]
        def invoke():
            try:output.append((True,self.rospy.ServiceProxy(service,self.trigger)()))
            except Exception as error:output.append((False,error))
        worker=threading.Thread(target=invoke,daemon=True);worker.start();worker.join(15.)
        require(output,"Review outcome unknown; no retry")
        ok,value=output[0]
        if not ok:raise value
        answer=json.loads(value.message);require(value.success,"Pilot review refused: "+str(answer))
        require(answer["adoption_token"]==state["adoption_token"]and answer["generation"]==state["generation"]+1
            and answer["recovery_stage"]=="approved"and answer["phase"]=="idle"and not answer["failure"],"Review not confirmed")
        return dict(operation="reviewed_recovery",reviewed_result_sha256=digest,actuator_frames=0,state=answer)


def main():
    # Reuse the frozen CLI's one-publication/cancellation/result loop verbatim.
    previous.TaskTransport=RecoveryTransport
    if "--max-joint-step-deg" not in sys.argv[1:]:
        sys.argv.extend(["--max-joint-step-deg","0.8"])
    return previous.main()


if __name__=="__main__":sys.exit(main())
