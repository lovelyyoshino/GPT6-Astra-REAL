#!/usr/bin/env python3
"""Single guarded ROS goal, pure bounded home proposal, or reviewed resume.

No task object localization or camera calibration. The model selects each next
goal; this client checks robot geometry and uses the frozen cancel loop.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'src'))
import ros_interruptible_joint_client as frozen
import ros_joint_stop_probe_entry as support
require=frozen.require
CLIENT_SHA='57dff327668377d52380923f0813b4d8a195ae3cb0464a64cf51a8e753557bfa'


def home_target(before,limits,fk,max_joint_deg=30.,anchor=None):
    """Pure proportional move toward existing zeros, limited by full joint box."""
    require(math.isfinite(max_joint_deg)and 0<max_joint_deg<=30.,'Joint step ceiling exceeds30degrees')
    origin=list(before['raw_q']if anchor is None else anchor)
    require(any(origin),'Already exact joint zero')
    upper=min(1.,max_joint_deg*1000/max(abs(v)for v in origin))
    def candidate(scale):
        raw=[round(v*(1-scale))for v in origin]
        require(raw!=before['raw_q'],'No encoded movement')
        require(max(abs(a-b)for a,b in zip(before['raw_q'],raw))<=max_joint_deg*1000,'Joint step exceeds ceiling')
        require(all(min(0,a)<=b<=max(0,a)for a,b in zip(origin,raw)),'Home target must approach zero')
        return dict(target_raw=raw,scale=scale,path=support.path_check(before,raw,limits,fk),speed_percent=1)
    low,best,high=0.,None,upper
    for i in range(24):
        scale=upper if i==0 else(low+high)/2
        try:plan=candidate(scale)
        except RuntimeError as error:
            if str(error)not in ('Complete joint box exceeds original motion bounds','Complete joint box exceeds workspace',
                                'End-reference segment exceeds original 30mm/.05rad bounds','End reference outside original workspace',
                                'Joint step exceeds ceiling','No encoded movement'):
                raise
            high=scale;continue
        low,best=scale,plan
        if scale==upper or high-low<1e-7:break
    require(best is not None,'No bounded common-scale home segment')
    if best['scale']<1.:best=candidate(best['scale']*.95)
    return best


class TaskTransport(frozen.RosTransport):
    def __init__(self,config,runtime):
        import rospy,rosgraph,rosnode
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from xmlrpc.client import ServerProxy
        require(hashlib.sha256(Path(frozen.__file__).read_bytes()).hexdigest()==CLIENT_SHA,'Frozen client changed')
        self.rospy,self.message,self.trigger=rospy,JointState,Trigger
        self.config,self.runtime=Path(config).resolve(),Path(runtime).resolve()
        self.master=rosgraph.Master(rospy.get_name())
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.session_path=ROOT/'runs/ros_interruptible_joint_sessions'/('guarded_task_'+boot+'.json')
        self.session=json.loads(self.session_path.read_text());self.identity=self.session['identity']
        entry=ROOT/'scripts/ros_guarded_task_entry.py'
        require(hashlib.sha256(entry.read_bytes()).hexdigest()==self.identity['source_sha256'],'Task entry source changed')
        require(hashlib.sha256(self.config.read_bytes()).hexdigest()==self.identity['config_sha256'],'Task config changed')
        require(self.identity['task_session_path']==str(self.session_path),'Task session binding mismatch')
        uri=rosnode.get_api_uri(self.master,frozen.NODE);require(uri,'Guarded driver absent')
        pid=ServerProxy(uri).getPid(rospy.get_name())[2]
        require(pid==self.identity['pid'],'Guarded driver PID mismatch')
        argv=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
        for item in (str(entry),'--entry-config',str(self.config),'--entry-output-dir',str(self.runtime)):
            require(item.encode()in argv,'Guarded driver process argument mismatch: '+item)
        pubs,subs,_=self.master.getSystemState()
        require(dict(subs).get(frozen.TOPIC)==[frozen.NODE],'Joint subscriber must be exclusive')
        for topic in (frozen.TOPIC,'/piper/right/pos_cmd','/piper/right/enable_flag'):
            require(not dict(pubs).get(topic),'Competing motion publisher')
        require(dict(pubs).get('/piper/right/eval_telemetry')==[frozen.NODE],'Telemetry source mismatch')
        require(rospy.get_param(frozen.NODE+'/speed_percent')==1,'Only1percent is admitted')
        self.publisher=rospy.Publisher(frozen.TOPIC,JointState,queue_size=1,latch=False)
        deadline=time.monotonic()+3
        while not self.publisher.get_num_connections()and time.monotonic()<deadline:time.sleep(.02)
        require(self.publisher.get_num_connections()==1,'Joint transport unavailable')

    def resume(self,digest):
        state=self.status()
        path=self.runtime/('action_%06d_result.json'%state['sequence'])
        require(state.get('result_sha256')==digest and hashlib.sha256(path.read_bytes()).hexdigest()==digest,
                'Explicit reviewed result digest mismatch')
        require(state['phase']=='hold_confirmed'and state.get('resume_service'),'No confirmed hold eligible for resume')
        service=state['resume_service']
        expected=frozen.NODE+'/resume_after_review/seq_%d_gen_%d_%s_%s'%(state['sequence'],state['generation'],state['adoption_token'],digest)
        require(service==expected,'Resume service does not match reviewed generation/result')
        self.rospy.wait_for_service(service,timeout=1.)
        output=[]
        def invoke():
            try:output.append((True,self.rospy.ServiceProxy(service,self.trigger)()))
            except Exception as error:output.append((False,error))
        worker=threading.Thread(target=invoke,daemon=True);worker.start();worker.join(15.)
        require(output,'Resume response unknown; no retry')
        ok,value=output[0]
        if not ok:raise value
        answer=json.loads(value.message)
        require(value.success,'Resume refused: '+str(answer))
        require(answer['adoption_token']==state['adoption_token']and answer['generation']==state['generation']+1
                and answer['phase']=='idle'and answer['stage']=='task'and not answer['stop_latched'],
                'Resume did not confirm new generation')
        return dict(operation='reviewed_resume',reviewed_result_sha256=digest,actuator_frames=0,
                    previous_generation=state['generation'],new_generation=answer['generation'],state=answer)

    def gripper(self,width_mm):
        require(math.isfinite(width_mm)and 0<=width_mm<=55.,'Jaw width must be0..55mm')
        from piper_msgs.srv import Gripper
        state=self.status()
        require(state['stage']=='task'and state['phase']in('idle','completed')and not state['active']
                and not state['stop_latched']and not state['failure'],'Jaw requires healthy idle task stage')
        self.observe()
        service='/piper/right/gripper_srv'
        _,_,services=self.master.getSystemState()
        require(dict(services).get(service)==[frozen.NODE],'Gripper service owner mismatch')
        self.rospy.wait_for_service(service,timeout=1.)
        output=[]
        def invoke():
            try:output.append((True,self.rospy.ServiceProxy(service,Gripper)(width_mm/1000.,.2,1,0)))
            except Exception as error:output.append((False,error))
        worker=threading.Thread(target=invoke,daemon=True);worker.start();worker.join(35.)
        require(output,'Jaw response unknown; no retry')
        ok,value=output[0]
        if not ok:raise value
        after=self.status()
        require(value.status and value.code==15900,'Jaw service refused or failed')
        require(after['adoption_token']==state['adoption_token']and after['generation']==state['generation']
                and after['sequence']==state['sequence']+1 and after['phase']=='completed'
                and not after['failure'],'Jaw did not confirm stable completion')
        return dict(operation='gripper',target_width_mm=width_mm,service_code=value.code,
                    feedback_stable=True,grasp_verified=False,driver_result=after['result'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True);parser.add_argument('--runtime',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--operation',choices=['home','joint','gripper','resume','status'],required=True)
    parser.add_argument('--max-joint-step-deg',type=float,default=30.)
    parser.add_argument('--target-raw',type=int,nargs=6)
    parser.add_argument('--width-mm',type=float)
    parser.add_argument('--reviewed-result-sha256')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--cancel-after-progress-deg',type=float)
    parser.add_argument('--motion-timeout-s',type=float,default=20.)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    import rospy
    rospy.init_node('guarded_task_client',anonymous=True,disable_signals=True)
    cancelled=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:cancelled.set());signal.signal(signal.SIGTERM,lambda *_:cancelled.set())
    io=TaskTransport(args.config,args.runtime);status=io.status()
    (out/'status_before.json').write_text(json.dumps(status,indent=2)+'\n')
    if args.operation=='status':result=dict(operation='status',state=status)
    elif args.operation=='resume':
        require(args.execute and args.reviewed_result_sha256 and not cancelled.is_set(),'Explicit result review and execute required')
        result=io.resume(args.reviewed_result_sha256)
    elif args.operation=='gripper':
        require(args.execute and args.width_mm is not None and not cancelled.is_set(),'Explicit gripper width and execute required')
        result=io.gripper(args.width_mm)
    else:
        before=io.observe();limits=support.checked_probe_limits(json.loads(Path(args.config).read_text()))
        fk=support.manufacturer_fk()
        if args.operation=='home':
            ceiling=min(args.max_joint_step_deg,1.)if status['stage']=='commissioning'else args.max_joint_step_deg
            anchor=io.session['held_raw']if status['stage']=='commissioning'else None
            plan=home_target(before,limits,fk,ceiling,anchor)
        else:
            require(args.target_raw is not None,'Six raw joint targets required')
            plan=dict(target_raw=args.target_raw,path=support.path_check(before,args.target_raw,limits,fk),speed_percent=1)
        (out/'before.json').write_text(json.dumps(before,indent=2)+'\n')
        (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        if not args.execute:result=dict(operation='proposal',published=False,plan=plan)
        else:
            with(out/'events.jsonl').open('x',buffering=1)as log:
                result=frozen.run_goal(io,plan['target_raw'],motion_timeout_s=args.motion_timeout_s,
                    cancel_after_progress_deg=args.cancel_after_progress_deg,cancelled=cancelled.is_set,
                    audit=lambda row:log.write(json.dumps(row,allow_nan=False)+'\n'))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items()if k not in('driver_result','state','plan')}))
    return 1 if result.get('error')else 0


if __name__=='__main__':sys.exit(main())
