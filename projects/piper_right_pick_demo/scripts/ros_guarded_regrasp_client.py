#!/usr/bin/env python3
"""Single bounded regrasp command or explicit evidence-bound probe review."""
import argparse
import json
from pathlib import Path
import sys
import time
import threading
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"scripts"))
import ros_guarded_task_client as previous
import ros_guarded_regrasp_entry as recovery
frozen=previous.frozen;require=previous.require
CLIENT_SHA="951c1747fd7344f09640cc143b499a9955e7c711cf5391cd9a7af4f4a9461d1d"


class RegraspTransport(previous.TaskTransport):
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
        self.session_path=recovery.guard.v1.SESSION_ROOT/recovery.child_name(boot)
        self.session=json.loads(self.session_path.read_text());self.identity=self.session["identity"]
        entry=Path(recovery.__file__).resolve()
        require(self.identity["boot_id"]==boot and recovery.sha(entry)==self.identity["source_sha256"]
                and self.identity["guard_sha256"]==recovery.motion.predecessor.GUARD_SHA
                and recovery.sha(recovery.guard.__file__)==recovery.motion.predecessor.GUARD_SHA
                and self.identity["predecessor_sha256"]==recovery.sha(recovery.motion.predecessor.__file__)==recovery.motion.PREDECESSOR_SHA
                and self.identity["rx_overlay_sha256"]==recovery.sha(recovery.profile.__file__)==recovery.motion.PROFILE_SHA
                and self.identity["tracking_profile_sha256"]==recovery.motion.PROFILE_SHA
                and self.identity["rx_runtime_sha256"]==recovery.sha(recovery.motion.frozen.__file__)==recovery.motion.RX_SHA
                and self.identity["raw_overlay_sha256"]==recovery.sha(recovery.motion.frozen.overlay.__file__)==recovery.motion.RAW_OVERLAY_SHA
                and self.identity["release_source_sha256"]==recovery.sha(recovery.release.__file__)==recovery.RELEASE_SOURCE_SHA
                and self.identity["j5_runtime_sha256"]==recovery.sha(recovery.motion.__file__)==recovery.MOTION_SOURCE_SHA,"Recovery source/boot changed")
        require(recovery.sha(self.config)==self.identity["config_sha256"]==recovery.CONFIG_SHA
                and self.identity["task_session_path"]==str(self.session_path),"Recovery config/session changed")
        parent=recovery.guard.v1.SESSION_ROOT/recovery.release.child_name(boot)
        require(recovery.sha(parent)==self.identity["parent_session_sha256"]==recovery.RELEASE_SESSION_SHA,"Parent failure changed")
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
        

    def publish(self,raw):
        state=self.observe();session=json.loads(self.session_path.read_text())
        require(session['identity']['adoption_token']==self.identity['adoption_token'],'Session adoption changed')
        msg=types.SimpleNamespace(position=[v*recovery.support.RAD_PER_RAW for v in raw],velocity=[0]*6+[1],effort=[])
        require(recovery.scoped_target(msg,state,session)==raw,'Encoded target changed')
        return super().publish(raw)

    def review_probe(self,digest):
        state=self.status();path=self.runtime/('action_%06d_result.json'%state['sequence'])
        require(state.get('result_sha256')==digest and recovery.sha(path)==digest,'Explicit probe result digest mismatch')
        service=state.get('probe_review_service')
        expected=frozen.NODE+'/review_probe/seq_%d_gen_%d_%s_%s'%(state['sequence'],state['generation'],state['adoption_token'],digest)
        require(service==expected,'Review endpoint does not match this probe/result/generation')
        self.rospy.wait_for_service(service,timeout=1.)
        output=[]
        def invoke():
            try:output.append((True,self.rospy.ServiceProxy(service,self.trigger)()))
            except Exception as error:output.append((False,error))
        worker=threading.Thread(target=invoke,daemon=True);worker.start();worker.join(15.)
        require(output,'Probe review result unknown; no automatic retry')
        ok,value=output[0]
        if not ok:raise value
        data=json.loads(value.message);require(value.success,'Probe review refused: '+str(data))
        require(data['adoption_token']==state['adoption_token']and data['generation']==state['generation']+1
            and data['phase']=='idle'and not data['failure'],'Probe review did not establish next stage')
        return dict(operation='review',actuator_frames=0,state=data)


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--operation',choices=['joint','gripper','resume','status','review'],required=True)
    args,_=parser.parse_known_args()
    if args.operation!='review':
        run=recovery.profile.private_function(previous.main,TaskTransport=RegraspTransport,support=recovery.support)
        return run()
    parser.add_argument('--config',required=True);parser.add_argument('--runtime',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--reviewed-result-sha256',required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    import rospy
    rospy.init_node('guarded_probe_review_client',anonymous=True,disable_signals=True)
    io=RegraspTransport(args.config,args.runtime)
    result=io.review_probe(args.reviewed_result_sha256)
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(operation='review',actuator_frames=0,regrasp_stage=result['state']['regrasp_stage'])))
    return 0


if __name__=='__main__':sys.exit(main())
