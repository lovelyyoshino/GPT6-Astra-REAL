#!/usr/bin/env python3
"""One explicit ROS wide opening/action or result-bound review; never a retry."""
import argparse
import json
from pathlib import Path
import sys
import threading
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_regrasp_client as previous
import ros_guarded_wide_entry as entry
profile=entry.profile;require=entry.require;frozen=previous.frozen
PREVIOUS_CLIENT_SHA='175e95f0b5aca861901d188deb8111c0e2a252fb774219fb8a820f1f15fbc79c'


def identity_view():
    # Reuse all graph/PID/boot/parent checks while naming the actual new sources.
    motion=types.SimpleNamespace(**dict(vars(entry.motion),PROFILE_SHA=entry.sha(profile.__file__)))
    return types.SimpleNamespace(**dict(vars(entry.previous),__file__=entry.__file__,motion=motion,
        profile=profile,CONFIG_SHA=entry.CONFIG_SHA,child_name=entry.child_name,support=entry.support))


def wide_gripper_method():
    function=previous.previous.TaskTransport.gripper
    require(function.__code__.co_consts.count(55.)==1,'Expected frozen client width check absent')
    copy=profile.private_function(function)
    copy.__code__=function.__code__.replace(co_consts=tuple(70. if type(x)is float and x==55. else
        'Jaw width must be0..70mm'if x=='Jaw width must be0..55mm'else x for x in function.__code__.co_consts))
    return copy


class WideTransport(previous.RegraspTransport):
    def __init__(self,config,runtime):
        require(entry.sha(previous.__file__)==PREVIOUS_CLIENT_SHA,'Frozen regrasp client changed')
        require(entry.sha(profile.original.__file__)==entry.motion.PROFILE_SHA,'Frozen J5 tracking changed')
        profile.private_function(previous.RegraspTransport.__init__,recovery=identity_view())(self,config,runtime)
        require(self.identity['wide_profile_sha256']==entry.sha(profile.__file__)
            and self.identity['joint_tracking_source_sha256']==entry.motion.PROFILE_SHA
            and self.identity['range_query_sha256']==entry.RANGE_QUERY_SHA,'Wide identity evidence mismatch')

    _gripper=wide_gripper_method()

    def gripper(self,width_mm):
        state=self.status();stage=state['regrasp_stage']
        if stage in entry.OPENINGS:
            require(width_mm==entry.OPENINGS[stage]*1000,'Only this stage fixed opening is permitted')
        else:require(stage=='task','Opening/separation review still required')
        return self._gripper(width_mm)

    def review_probe(self,digest):
        state=self.status()
        if not state['regrasp_stage'].startswith('opening'):return super().review_probe(digest)
        path=self.runtime/('action_%06d_result.json'%state['sequence'])
        require(state.get('result_sha256')==digest and entry.sha(path)==digest,'Reviewed opening result changed')
        service=state.get('opening_review_service')
        expected=frozen.NODE+'/review_opening/seq_%d_gen_%d_%s_%s'%(state['sequence'],state['generation'],state['adoption_token'],digest)
        require(service==expected,'Opening review endpoint mismatch')
        self.rospy.wait_for_service(service,timeout=1.);output=[]
        def invoke():
            try:output.append((True,self.rospy.ServiceProxy(service,self.trigger)()))
            except Exception as error:output.append((False,error))
        worker=threading.Thread(target=invoke,daemon=True);worker.start();worker.join(15.)
        require(output,'Opening review result unknown; no automatic retry')
        ok,value=output[0]
        if not ok:raise value
        data=json.loads(value.message);require(value.success,'Opening review refused: '+str(data))
        require(data['adoption_token']==state['adoption_token']and data['generation']==state['generation']+1
            and data['phase']=='idle'and not data['failure'],'Opening review did not establish next stage')
        return dict(operation='review',actuator_frames=0,state=data)


def main():
    # The frozen CLI uses this transport for both its one-call review path and
    # ordinary single commands. No automatic home operation is admitted.
    run=profile.private_function(previous.main,RegraspTransport=WideTransport,
        recovery=types.SimpleNamespace(profile=profile,support=entry.support))
    return run()


if __name__=='__main__':sys.exit(main())
