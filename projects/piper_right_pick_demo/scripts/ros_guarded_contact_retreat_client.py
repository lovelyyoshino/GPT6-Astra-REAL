#!/usr/bin/env python3
"""One exact loaded-contact retreat or read-only status; no jaw, hold or promotion."""
import argparse
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_readiness_client as previous
import ros_guarded_contact_retreat_entry as entry
profile=entry.profile;require=entry.require;wide_client=previous.wide_client
CLIENT_SHA='e5e70c71f9b7a76b3917dd1667e58c87485c9f135fa147fa9ce28cc37a046d34'


def identity_view():
    old=entry.predecessor.predecessor.previous
    motion=types.SimpleNamespace(**dict(vars(entry.motion),PROFILE_SHA=entry.PROFILE_SHA))
    release=types.SimpleNamespace(**dict(vars(old.release),child_name=entry.predecessor.child_name))
    return types.SimpleNamespace(**dict(vars(old),__file__=entry.__file__,motion=motion,
        profile=profile,release=release,RELEASE_SESSION_SHA=entry.PARENT_SHA,CONFIG_SHA=entry.CONFIG_SHA,
        child_name=entry.child_name,support=entry.support))


class ContactRetreatTransport(wide_client.WideTransport):
    def __init__(self,config,runtime):
        require(entry.sha(previous.__file__)==CLIENT_SHA
            and entry.sha(wide_client.__file__)==previous.WIDE_CLIENT_SHA
            and entry.sha(wide_client.previous.__file__)==wide_client.PREVIOUS_CLIENT_SHA,'Frozen clients changed')
        profile.private_function(wide_client.previous.RegraspTransport.__init__,recovery=identity_view())(self,config,runtime)
        require(self.identity['contact_predecessor_sha256']==entry.sha(entry.predecessor.__file__)==entry.PREDECESSOR_SHA
            and self.identity['parent_result_sha256']==entry.RESULT_SHA
            and self.identity['contact_raw_review_sha256']==entry.RAW_REVIEW_SHA
            and self.identity['loaded_contact_target_raw']==entry.TARGET
            and self.identity['can_held_by_gripper']is True and self.identity['jaw_authorized']is False,
            'Loaded-contact recovery identity changed')

    def publish(self,raw):
        require(raw==entry.TARGET,'Only the exact seq59 retreat target is permitted')
        return super().publish(raw)
    def gripper(self,*a,**k):raise RuntimeError('Held load: all jaw commands forbidden')
    def hold(self,*a,**k):raise RuntimeError('No hold replacement in this fixed retreat; accepted goal may continue')
    def resume(self,*a,**k):raise RuntimeError('No task resumption in this scope')
    def review_probe(self,*a,**k):raise RuntimeError('No promotion in this scope')


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--operation',choices=['joint','status'],required=True)
    parser.add_argument('--target-raw',type=int,nargs=6)
    parser.add_argument('--cancel-after-progress-deg',type=float)
    args,_=parser.parse_known_args()
    require(args.cancel_after_progress_deg is None,'No automatic hold request in this fixed retreat scope')
    require(args.operation=='status'or args.target_raw==entry.TARGET,'Exact prior seq59 target required')
    return profile.private_function(previous.task_client.main,TaskTransport=ContactRetreatTransport,
        support=entry.support,frozen=previous.frozen)()


if __name__=='__main__':sys.exit(main())
