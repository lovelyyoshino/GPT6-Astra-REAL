#!/usr/bin/env python3
"""One explicit guarded command or one evidence-bound post-contact review."""
import argparse
import json
from pathlib import Path
import sys
import threading
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_readiness_client as previous
import ros_guarded_post_contact_entry as entry
profile=entry.profile;require=entry.require;wide_client=previous.wide_client
CLIENT_SHA='e5e70c71f9b7a76b3917dd1667e58c87485c9f135fa147fa9ce28cc37a046d34'


def identity_view():
    old=entry.readiness.predecessor.previous
    motion=types.SimpleNamespace(**dict(vars(entry.motion),PROFILE_SHA=entry.PROFILE_SHA))
    release=types.SimpleNamespace(**dict(vars(old.release),child_name=entry.predecessor.child_name))
    return types.SimpleNamespace(**dict(vars(old),__file__=entry.__file__,motion=motion,
        profile=profile,release=release,RELEASE_SESSION_SHA=entry.PARENT_SHA,CONFIG_SHA=entry.CONFIG_SHA,
        child_name=entry.child_name,support=entry.support))


class PostContactTransport(previous.ReadinessTransport):
    def __init__(self,config,runtime):
        require(entry.sha(previous.__file__)==CLIENT_SHA
            and entry.sha(wide_client.__file__)==previous.WIDE_CLIENT_SHA
            and entry.sha(wide_client.previous.__file__)==wide_client.PREVIOUS_CLIENT_SHA,'Frozen client changed')
        profile.private_function(wide_client.previous.RegraspTransport.__init__,recovery=identity_view())(self,config,runtime)
        require(self.identity['post_contact_predecessor_sha256']==entry.sha(entry.predecessor.__file__)==entry.PREDECESSOR_SHA
            and self.identity['parent_result_sha256']==entry.RESULT_SHA
            and self.identity['post_contact_raw_review_sha256']==entry.RAW_REVIEW_SHA
            and self.identity['frozen_other_targets']==entry.TARGET,'Post-contact identity changed')

    def publish(self,raw):
        state=self.observe();session=json.loads(self.session_path.read_text())
        require(session['identity']['adoption_token']==self.identity['adoption_token'],'Session changed')
        msg=types.SimpleNamespace(position=[v*entry.support.RAD_PER_RAW for v in raw],velocity=[0]*6+[1],effort=[])
        require(entry.scoped_target(msg,state,session)==raw,'Encoded post-contact target changed')
        return super().publish(raw)

    def gripper(self,*a,**k):
        require(self.status()['contact_stage']=='task','No jaw before reviewed visible clearance')
        return super().gripper(*a,**k)
    def hold(self,*a,**k):
        require(self.status()['contact_stage']=='task','Finite clearance lift cannot accept replacement hold')
        return super().hold(*a,**k)
    def resume(self,*a,**k):
        require(self.status()['contact_stage']=='task','Use the completed-lift review endpoint')
        return super().resume(*a,**k)
    def review_probe(self,*a,**k):raise RuntimeError('Use review_contact')

    def review_contact(self,digest):
        state=self.status();path=self.runtime/('action_%06d_result.json'%state['sequence'])
        require(state.get('contact_stage')=='lift_review'and state.get('result_sha256')==digest
            and entry.sha(path)==digest,'Explicit completed-lift digest mismatch')
        service=state.get('contact_review_service')
        expected=previous.frozen.NODE+'/review_contact/seq_%d_gen_%d_%s_%s'%(state['sequence'],state['generation'],state['adoption_token'],digest)
        require(service==expected,'Review endpoint does not match lift/token/generation')
        self.rospy.wait_for_service(service,timeout=1.)
        output=[]
        def invoke():
            try:output.append((True,self.rospy.ServiceProxy(service,self.trigger)()))
            except Exception as error:output.append((False,error))
        worker=threading.Thread(target=invoke,daemon=True);worker.start();worker.join(15.)
        require(output,'Review result unknown; no automatic retry')
        ok,value=output[0]
        if not ok:raise value
        data=json.loads(value.message);require(value.success,'Contact review refused: '+str(data))
        require(data['adoption_token']==state['adoption_token']and data['generation']==state['generation']+1
            and data['phase']=='idle'and not data['failure']and data['contact_stage']in ('lifting','task'),
            'Review did not establish the bound next stage')
        return dict(operation='review',actuator_frames=0,state=data)


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--operation',choices=['joint','gripper','resume','status','review'],required=True)
    args,_=parser.parse_known_args()
    if args.operation!='review':
        facade=types.SimpleNamespace(**dict(vars(previous.frozen),run_goal=previous.run_goal))
        return profile.private_function(previous.task_client.main,TaskTransport=PostContactTransport,
            support=entry.support,frozen=facade)()
    parser.add_argument('--config',required=True);parser.add_argument('--runtime',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--reviewed-result-sha256',required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    import rospy
    rospy.init_node('post_contact_review_client',anonymous=True,disable_signals=True)
    result=PostContactTransport(args.config,args.runtime).review_contact(args.reviewed_result_sha256)
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(operation='review',actuator_frames=0,contact_stage=result['state']['contact_stage'])))
    return 0


if __name__=='__main__':sys.exit(main())
