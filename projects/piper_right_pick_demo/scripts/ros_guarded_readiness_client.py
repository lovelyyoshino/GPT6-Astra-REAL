#!/usr/bin/env python3
"""Single guarded command; zero-TX readiness refusal is terminal, never retried."""
import argparse
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_wide_client as wide_client
import ros_guarded_readiness_entry as entry
profile=entry.profile;require=entry.require
frozen=wide_client.frozen
task_client=wide_client.previous.previous
WIDE_CLIENT_SHA='62678dcbc491008c5eb584156fbed5483a980dbaf5f0dc3789b9783a516931d4'


def identity_view():
    motion=types.SimpleNamespace(**dict(vars(entry.motion),PROFILE_SHA=entry.PROFILE_SHA))
    old=entry.predecessor.previous
    release=types.SimpleNamespace(**dict(vars(old.release),child_name=entry.predecessor.child_name))
    return types.SimpleNamespace(**dict(vars(old),__file__=entry.__file__,motion=motion,
        profile=profile,release=release,RELEASE_SESSION_SHA=entry.PARENT_SHA,CONFIG_SHA=entry.CONFIG_SHA,
        child_name=entry.child_name,support=entry.support))


def readiness_gripper_method():
    function=profile.private_function(wide_client.WideTransport._gripper)
    old=('idle','completed')
    require(old in function.__code__.co_consts,'Frozen jaw readiness clause changed')
    function.__code__=function.__code__.replace(co_consts=tuple(old+('rejected_preflight',)if value==old else value
        for value in function.__code__.co_consts))
    return function


class ReadinessTransport(wide_client.WideTransport):
    _gripper=readiness_gripper_method()
    def __init__(self,config,runtime):
        require(entry.sha(wide_client.__file__)==WIDE_CLIENT_SHA
            and entry.sha(wide_client.previous.__file__)==wide_client.PREVIOUS_CLIENT_SHA,'Frozen client changed')
        profile.private_function(wide_client.previous.RegraspTransport.__init__,recovery=identity_view())(self,config,runtime)
        require(self.identity['readiness_predecessor_sha256']==entry.sha(entry.predecessor.__file__)==entry.PREDECESSOR_SHA
            and self.identity['sliding_stability_sha256']==entry.sha(entry.stability.__file__)==entry.STABILITY_SHA
            and self.identity['baseline_diagnosis_sha256']==entry.DIAGNOSIS_SHA
            and self.identity['parent_result_sha256']==entry.RESULT_SHA,'Readiness identity changed')

    def gripper(self,width_mm):
        before=self.status()
        try:return super().gripper(width_mm)
        except Exception:
            # A service error alone is ambiguous. Only the bound driver state
            # can prove this particular request ended before an actuator send.
            after=self.status();result=after.get('result')or{}
            if (after['adoption_token']==before['adoption_token']
                and after['generation']==before['generation']
                and after['sequence']==before['sequence']+1 and not after['active']
                and not after['failure']and after['phase']=='rejected_preflight'
                and after.get('kind')=='gripper'and result.get('kind')=='gripper'
                and result.get('phase')=='rejected_preflight'
                and after['receipts']==[]and result.get('actuator_frames')==0
                and result.get('actuator_transaction_started')is False):
                return dict(operation='gripper',rejected_preflight=True,actuator_frames=0,
                    execution_started=False,error=result['error'],new_explicit_request_required=True,
                    task_success=False,grasp_verified=False,driver_result=result)
            raise


def _loop():
    function=profile.private_function(frozen.run_goal,TERMINAL=frozen.TERMINAL|{'rejected_preflight'})
    old=('idle','completed')
    require(old in function.__code__.co_consts,'Frozen client readiness clause changed')
    function.__code__=function.__code__.replace(co_consts=tuple(old+('rejected_preflight',)if value==old else value
        for value in function.__code__.co_consts))
    return function


def run_goal(io,target_raw,**kwargs):
    result=_loop()(io,target_raw,**kwargs)
    if result.get('driver_phase')=='rejected_preflight':
        evidence=result.get('driver_result')or{}
        require(evidence.get('actuator_frames')==0 and evidence.get('actuator_transaction_started')is False
            and evidence.get('phase')=='rejected_preflight','Zero-TX refusal proof missing')
        result.update(actuator_frames=0,execution_started=False,rejected_preflight=True,
            target_reached=False,task_success=False,new_explicit_request_required=True,
            accepted_target_may_continue=False,error=evidence['error'])
    return result


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--operation',choices=['joint','gripper','resume','status'],required=True)
    parser.parse_known_args()
    facade=types.SimpleNamespace(**dict(vars(frozen),run_goal=run_goal))
    return profile.private_function(task_client.main,TaskTransport=ReadinessTransport,
        support=entry.support,frozen=facade)()


if __name__=='__main__':sys.exit(main())
