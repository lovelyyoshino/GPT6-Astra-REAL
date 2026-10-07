#!/usr/bin/env python3
"""One explicit ROS joint/jaw action or interior-result review; no home replay."""
import argparse
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"scripts"))
import ros_guarded_recovery_client as recovery_client
import ros_guarded_interior_entry as recovery
previous=recovery_client.previous;frozen=previous.frozen;require=previous.require
CLIENT_SHA="951c1747fd7344f09640cc143b499a9955e7c711cf5391cd9a7af4f4a9461d1d"
RECOVERY_CLIENT_SHA="fcc01f2ad85d910bfdfaa224b79409ba5d27f130eabe5636b53229e826a25327"


class InteriorTransport(recovery_client.RecoveryTransport):
    def __init__(self,config,runtime):
        import rospy,rosgraph,rosnode
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from xmlrpc.client import ServerProxy
        require(recovery.sha(recovery_client.__file__)==RECOVERY_CLIENT_SHA and recovery.sha(previous.__file__)==CLIENT_SHA and recovery.sha(frozen.__file__)==previous.CLIENT_SHA,"Frozen client changed")
        self.rospy,self.message,self.trigger=rospy,JointState,Trigger
        self.config,self.runtime=Path(config).resolve(),Path(runtime).resolve()
        self.master=rosgraph.Master(rospy.get_name())
        boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.session_path=recovery.guard.v1.SESSION_ROOT/recovery.CHILD_NAME
        self.session=json.loads(self.session_path.read_text());self.identity=self.session["identity"]
        entry=Path(recovery.__file__).resolve()
        require(self.identity["boot_id"]==boot and recovery.sha(entry)==self.identity["source_sha256"]
                and self.identity["guard_sha256"]==recovery.GUARD_SHA
                and recovery.sha(recovery.guard.__file__)==recovery.GUARD_SHA
                and self.identity["predecessor_sha256"]==recovery.sha(recovery.predecessor.__file__)==recovery.PREDECESSOR_SHA,"Recovery source/boot changed")
        require(recovery.sha(self.config)==self.identity["config_sha256"]==recovery.CONFIG_SHA
                and self.identity["task_session_path"]==str(self.session_path),"Recovery config/session changed")
        parent=recovery.guard.v1.SESSION_ROOT/recovery.predecessor.CHILD_NAME
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
        


def main():
    # Preserve the reviewed client's single publication / bounded hold / result
    # loop; omit automatic zero planning because the accepted goal is interior.
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument("--operation",choices=["joint","gripper","resume","status"],required=True)
    parser.parse_known_args()
    previous.TaskTransport=InteriorTransport
    return previous.main()


if __name__=="__main__":sys.exit(main())
