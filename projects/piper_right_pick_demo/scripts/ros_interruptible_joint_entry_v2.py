#!/usr/bin/env python3
"""Offline-only candidate for late-binding the moving J2 hold target.

The frozen live v1, its evidence, and its persistent failure latch are unchanged.
There is deliberately NO deployable entry point in this candidate. Qualification
and deployment identity need a separate reviewed maintenance handoff.
"""
import copy
import hashlib
import math
from pathlib import Path
import sys
import threading

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"scripts"))
import ros_interruptible_joint_entry as v1

V1_SHA="61c7ad2cff4b072fb6e4c27f37b1a38b50250fb5fd6417d3457810b7ba427505"
SESSION_ROOT=v1.SESSION_ROOT  # No new namespace or alternative failure lock.
require=v1.require
support=v1.support


class InterruptibleCandidate(v1.Interruptible):
    """Offline testable core; production startup is intentionally unavailable."""
    def __init__(self,*args,**kwargs):
        require(hashlib.sha256(Path(v1.__file__).read_bytes()).hexdigest()==V1_SHA,"Frozen v1 changed")
        self.candidate_source_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.rebased_hold=None
        super().__init__(*args,**kwargs)

    def send_once(self,raw,measured,origin,original_target,kind,*,moving):
        if kind!="hold":
            self.rebased_hold=None
            return super().send_once(raw,measured,origin,original_target,kind,moving=moving)
        require(moving,"Moving hold path requires explicit moving state")
        require(self.node.piper.ticket is None,"Previous transaction unresolved")
        original_raw=[round(q/support.RAD_PER_RAW)for q in original_target]
        require(all(raw[i]==original_raw[i]for i in (0,2,3,4,5)),"Five original targets must remain unchanged")
        # All blocking identity/durability work precedes the final target sample.
        # The durable unresolved reservation authorizes no arbitrary target:
        # only current J2, the five frozen targets, and the original envelope.
        self.ownership_check()
        with self.state_lock:
            require(self.state["hold_requested"]and self.session["stop_latched"],"No bound hold request")
            require(not self.state["failure"]and not self.rebased_hold,"Hold is failed or already dispatched")
            self.state["phase"]="hold_sending"
            self.session["pending"]=dict(sequence=self.state["sequence"],kind="hold",attempted_frames=0,
                                          target_raw=None,target_policy="Current J2 sampled after durable reservation; five original targets",
                                          original_target_raw=original_raw)
            self.save()
        self.record("hold_preparation",durable=True,sequence=self.state["sequence"],
                    request_sample=measured,request_candidate_raw=list(raw),original_target_raw=original_raw,
                    encoded_target_not_yet_bound=True)
        deadline=self.clock.monotonic()+.1
        planned=self.read(moving=True)
        while not self.advanced(measured,planned)and self.clock.monotonic()<deadline:
            self.clock.sleep(.005);planned=self.read(moving=True)
        require(self.advanced(measured,planned),"New complete feedback required for hold binding")
        self.monitor(planned,origin,original_target)
        bound_raw=list(original_raw);bound_raw[1]=planned["raw_q"][1]
        require(original_raw[1]<=bound_raw[1]<=origin["raw_q"][1],"Latest measured J2 outside original interval")
        # Between planned and witness: only local pure validation and snapshots.
        # No ROS master call, journal, flush/fsync, or deliberate full-frame wait.
        path=support.path_check(planned,bound_raw,self.limits,self.fk)
        frames=support.frames_for(bound_raw)
        recent_motion=support.Probe.moving_reference(self.motion_history,planned)is not None
        with self.state_lock:
            require(self.state["active"]and self.state["hold_requested"]and not self.state["failure"],
                    "Bound action no longer active")
            witness=self.read(moving=True)
            require(witness["sequence"]>=planned["sequence"]and
                    all(a>=b for a,b in zip(witness["stamps"],planned["stamps"])),"Feedback regressed")
            require(max(abs(a-b)for a,b in zip(witness["q"],planned["q"]))<=.003
                    and math.dist(witness["pose"][:3],planned["pose"][:3])<=.0005
                    and support.rotation_distance(witness["pose"],planned["pose"])<=.003,
                    "Pre-send slip from latest hold plan")
            self.monitor(witness,origin,original_target)
            self.node.piper.healthy(planned,allow_moving=True)
            self.node.piper.healthy(witness,allow_moving=True)
            require(self.node.adopted and not self.node.rospy_probe.is_shutdown(),"Shutdown before hold send")
            ticket=dict(thread=threading.get_ident(),expected=list(frames),speed=1,in_comm=False,attempted=0,sent=0)
            started=self.clock.time()
            self.session["pending"]["target_raw"]=list(bound_raw)  # Persisted with receipt after critical section.
            self.node.piper.ticket=ticket
            self.rebased_hold=dict(target_raw=list(bound_raw),target=path["target"],plan_sample=planned,
                                   dispatch_sample=witness,request_sample=measured,path=path,
                                   recent_motion_at_plan=recent_motion,
                                   source_v1_sha256=V1_SHA,source_v2_sha256=self.candidate_source_sha)
        try:
            self.node.piper.MotionCtrl_2(1,1,1,0,0,0)
            self.node.piper.JointCtrl(*bound_raw)
            require(not self.node.piper.broken and not ticket["expected"]
                    and ticket["attempted"]==ticket["sent"]==4,"Partial or failed hold transaction; no retry")
        finally:
            self.node.piper.ticket=None
            receipt=dict(kind="hold",started_unix_s=started,finished_unix_s=self.clock.time(),
                         attempted_frames=ticket["attempted"],socket_send_returns=ticket["sent"],
                         target_raw=list(bound_raw),dispatch_sample=witness,plan_sample=planned,
                         request_candidate_raw=list(raw),path=path,
                         source_v1_sha256=V1_SHA,source_v2_sha256=self.candidate_source_sha,
                         frames=[dict(id=k,data_hex=data.hex())for k,data in frames])
            with self.state_lock:
                self.state["receipts"].append(receipt)
                self.session["pending"]["attempted_frames"]=ticket["attempted"]
                self.session["attempted_frames"]=self.session.get("attempted_frames",0)+ticket["attempted"]
                self.save()
            self.record("send_receipt",durable=True,sequence=self.state["sequence"],**receipt)
        self.rebased_hold["finished_unix_s"]=receipt["finished_unix_s"]
        self.change(phase="holding")
        return receipt

    def wait_arrival(self,before,target,after,*,allow_request,deadline):
        if not allow_request and self.rebased_hold is not None:
            require(after==self.rebased_hold["finished_unix_s"],"Hold receipt/monitor mismatch")
            target=self.rebased_hold["target"]
        return super().wait_arrival(before,target,after,allow_request=allow_request,deadline=deadline)

    def finish(self,phase,after,window,**extra):
        if phase=="hold_confirmed":
            require(self.rebased_hold is not None,"No actual hold receipt")
            bound=self.rebased_hold;sample=bound["plan_sample"]
            initial=self.state["before"];original=extra["original_target_raw"]
            progress=(initial["raw_q"][1]-sample["raw_q"][1])/1000
            remaining=(sample["raw_q"][1]-original[1])/1000
            separation=(after["raw_q"][1]-original[1])/1000
            recent=bound["recent_motion_at_plan"]
            extra.update(request_candidate_raw=extra.get("hold_target_raw"),hold_target_raw=bound["target_raw"],
                         hold_plan_sample=sample,hold_dispatch_sample=bound["dispatch_sample"],
                         progress_before_hold_deg=progress,remaining_before_hold_deg=remaining,
                         recent_measured_motion_before_hold=recent,final_distance_before_old_target_deg=separation,
                         interruption_demonstrated=recent and progress>=.25 and remaining>=.35 and separation>=.2,
                         hold_result_scope="held_near_original_goal"if separation<.2 else"held_before_original_goal",
                         source_v1_sha256=V1_SHA,source_v2_sha256=self.candidate_source_sha,
                         candidate_offline_only=True)
        return super().finish(phase,after,window,**extra)


def main():
    raise SystemExit("Offline candidate only: no ROS startup or physical execution; frozen v1 failure/session locks remain in force")


if __name__=="__main__":main()
