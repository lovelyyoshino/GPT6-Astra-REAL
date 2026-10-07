#!/usr/bin/env python3
"""One explicit current-anchor probe/action or full-evidence review; no retry."""
from pathlib import Path
import sys
import types
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import ros_guarded_wide_client as wide_client
import ros_guarded_recorded_probe_entry as entry
profile=entry.profile;require=entry.require
WIDE_CLIENT_SHA='62678dcbc491008c5eb584156fbed5483a980dbaf5f0dc3789b9783a516931d4'


def identity_view():
    motion=types.SimpleNamespace(**dict(vars(entry.motion),PROFILE_SHA=entry.PROFILE_SHA))
    release=types.SimpleNamespace(**dict(vars(entry.previous.release),child_name=entry.predecessor.child_name))
    return types.SimpleNamespace(**dict(vars(entry.previous),__file__=entry.__file__,motion=motion,
        profile=profile,release=release,RELEASE_SESSION_SHA=entry.PARENT_SHA,CONFIG_SHA=entry.CONFIG_SHA,
        child_name=entry.child_name,support=entry.support))


class RecordedTransport(wide_client.WideTransport):
    def __init__(self,config,runtime):
        require(entry.sha(wide_client.__file__)==WIDE_CLIENT_SHA
            and entry.sha(wide_client.previous.__file__)==wide_client.PREVIOUS_CLIENT_SHA,'Frozen client changed')
        profile.private_function(wide_client.previous.RegraspTransport.__init__,recovery=identity_view())(self,config,runtime)
        require(self.identity['recorded_probe_predecessor_sha256']==entry.sha(entry.predecessor.__file__)==entry.PREDECESSOR_SHA
            and self.identity['missing_raw_diagnosis_sha256']==entry.GAP_SHA
            and self.identity['prior_seq3_independent_raw_review_complete']is False
            and self.identity['parent_result_sha256']==entry.RESULT_SHA,'Recorded-probe identity changed')

    gripper=profile.private_function(wide_client.WideTransport.gripper,entry=entry)
    review_probe=profile.private_function(wide_client.WideTransport.review_probe,entry=entry)


def main():return profile.private_function(wide_client.main,WideTransport=RecordedTransport)()


if __name__=='__main__':sys.exit(main())
