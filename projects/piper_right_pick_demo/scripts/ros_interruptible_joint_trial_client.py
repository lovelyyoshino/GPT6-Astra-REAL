#!/usr/bin/env python3
"""Client for the explicitly reviewed one-shot post-slip maintenance trial.

Reuses the frozen single-publication, action-bound cancel/confirmation loop.
No CAN/SDK transport; the parent failure record is never changed.
"""
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import ros_interruptible_joint_client as frozen

PARENT_SHA='cfa3c2b827cabd2e0ed0afb47f13263bda294ef4b741e79adcb950a0cd025140'
CLIENT_SHA='57dff327668377d52380923f0813b4d8a195ae3cb0464a64cf51a8e753557bfa'
require=frozen.require


class TrialTransport(frozen.RosTransport):
    def __init__(self,config,runtime):
        import rospy
        import rosgraph
        import rosnode
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from xmlrpc.client import ServerProxy
        require(hashlib.sha256(Path(frozen.__file__).read_bytes()).hexdigest()==CLIENT_SHA,'Frozen client changed')
        self.rospy,self.message,self.trigger=rospy,JointState,Trigger
        self.config,self.runtime=Path(config),Path(runtime)
        self.master=rosgraph.Master(rospy.get_name())
        root=ROOT/'runs/ros_interruptible_joint_sessions'
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        require(hashlib.sha256((root/(boot+'.json')).read_bytes()).hexdigest()==PARENT_SHA,'Reviewed parent changed')
        child=root/('trial_after_presend_slip_'+PARENT_SHA+'.json')
        session=json.loads(child.read_text());self.identity=session['identity']
        entry=ROOT/'scripts/ros_interruptible_joint_trial_entry.py'
        require(hashlib.sha256(entry.read_bytes()).hexdigest()==self.identity['source_sha256'],'Trial entry changed')
        require(hashlib.sha256(self.config.read_bytes()).hexdigest()==self.identity['config_sha256'],'Trial config changed')
        uri=rosnode.get_api_uri(self.master,frozen.NODE)
        require(uri,'Trial driver absent')
        pid=ServerProxy(uri).getPid(rospy.get_name())[2]
        require(pid==self.identity['pid'],'Trial PID/adoption mismatch')
        argv=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
        for item in (str(entry),'--entry-config',str(self.config),'--entry-output-dir',str(self.runtime)):
            require(item.encode() in argv,'Trial process argument mismatch: '+item)
        pubs,subs,_=self.master.getSystemState()
        require(dict(subs).get(frozen.TOPIC)==[frozen.NODE],'Joint subscriber is not exclusive')
        for topic in (frozen.TOPIC,'/piper/right/pos_cmd','/piper/right/enable_flag'):
            require(not dict(pubs).get(topic),'Competing publisher')
        require(dict(pubs).get('/piper/right/eval_telemetry')==[frozen.NODE],'Telemetry owner mismatch')
        require(rospy.get_param(frozen.NODE+'/speed_percent')==1,'Trial speed must remain1')
        self.publisher=rospy.Publisher(frozen.TOPIC,JointState,queue_size=1,latch=False)
        deadline=time.monotonic()+3.
        while not self.publisher.get_num_connections()and time.monotonic()<deadline:time.sleep(.02)
        require(self.publisher.get_num_connections()==1,'Joint transport unavailable')


if __name__=='__main__':
    frozen.RosTransport=TrialTransport
    sys.exit(frozen.main())
