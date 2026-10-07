#!/usr/bin/env python3
"""One explicit task request; a zero-TX refusal terminates without retry or hold."""
import argparse
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_post_contact_client as previous
import ros_zero_tx_continuation_entry as entry
profile=entry.profile;require=entry.require;ready=previous.previous;wide_client=ready.wide_client
CLIENT_SHA='8509db871d478d3feb0d27a5ada1505c088ace66dd0e24327addcfc0f0863203'


def identity_view():
    old=entry.predecessor.readiness.predecessor.previous
    motion=types.SimpleNamespace(**dict(vars(entry.motion),PROFILE_SHA=entry.PROFILE_SHA))
    release=types.SimpleNamespace(**dict(vars(old.release),child_name=entry.predecessor.child_name))
    return types.SimpleNamespace(**dict(vars(old),__file__=entry.__file__,motion=motion,
        profile=profile,release=release,RELEASE_SESSION_SHA=entry.PARENT_SHA,CONFIG_SHA=entry.CONFIG_SHA,
        child_name=entry.child_name,support=entry.support))


class ZeroTXContinuationTransport(previous.PostContactTransport):
    def __init__(self,config,runtime):
        require(entry.sha(previous.__file__)==CLIENT_SHA and entry.sha(ready.__file__)==previous.CLIENT_SHA
            and entry.sha(wide_client.__file__)==ready.WIDE_CLIENT_SHA
            and entry.sha(wide_client.previous.__file__)==wide_client.PREVIOUS_CLIENT_SHA,'Frozen clients changed')
        profile.private_function(wide_client.previous.RegraspTransport.__init__,recovery=identity_view())(self,config,runtime)
        require(self.identity['zero_tx_predecessor_sha256']==entry.sha(entry.predecessor.__file__)==entry.PREDECESSOR_SHA
            and self.identity['parent_result_sha256']==entry.RESULT_SHA
            and self.identity['zero_tx_raw_review_sha256']==entry.RAW_REVIEW_SHA
            and self.identity['j5_request_increment_max_mdeg']==200,'Zero-execution continuation identity changed')

    def review_contact(self,*a,**k):raise RuntimeError('Existing task clearance retained; no new promotion scope')


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--operation',choices=['joint','gripper','resume','status'],required=True)
    parser.parse_known_args()
    # The frozen readiness client already treats rejected_preflight as a final
    # zero-execution result. It neither replays nor asks hold for that result.
    facade=types.SimpleNamespace(**dict(vars(ready.frozen),run_goal=ready.run_goal))
    return profile.private_function(ready.task_client.main,TaskTransport=ZeroTXContinuationTransport,
        support=entry.support,frozen=facade)()


if __name__=='__main__':sys.exit(main())
