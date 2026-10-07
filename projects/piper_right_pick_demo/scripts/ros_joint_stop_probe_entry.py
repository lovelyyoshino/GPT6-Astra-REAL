#!/usr/bin/env python3
"""Restricted ROS experiment: one static J hold, then one in-motion J overwrite.

This is a driver entry, not a task client. Importing it performs no ROS/CAN I/O.
Only private Trigger services static_hold_probe/dynamic_stop_probe can transmit.
Neither success nor driver exit establishes a general stop or crash-safe hold.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from supervised_ros_home_segment import (manufacturer_fk, envelope,
                                        RAD_PER_RAW, JOINT_LIMITS_RAW, rotation_distance)
from ros_home_step import SessionFile, StableWindow

BASE_PATH = Path("/home/agilex/piperx_cloth_demo/robot_tools/ros_resume_entry.py")
BASE_SHA = "3cd2fcc87b5444a4e21419f33a4f38169b7ef2b7645fe329abba1aed70a992c8"
SESSION_ROOT = ROOT / "runs" / "ros_joint_stop_probe_sessions"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def load_base():
    require(hashlib.sha256(BASE_PATH.read_bytes()).hexdigest() == BASE_SHA, "Frozen resume entry changed")
    spec = importlib.util.spec_from_file_location("pinned_probe_resume", BASE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def checked_probe_limits(config):
    limits = copy.deepcopy(config["physical_limits"])
    require(type(limits.get("max_speed_percent")) is int and limits["max_speed_percent"] == 1,
            "Probe configuration must explicitly limit speed to1")
    def number(v):
        require(type(v) in (int,float) and math.isfinite(v), "Finite numeric bound required")
        return v
    for key, cap in (("max_translation_step_m",.03),("max_rotation_step_rad",.05),("max_state_age_s",.1)):
        require(0 < number(limits[key]) <= cap, "Original motion/age bounds cannot increase")
    require(len(limits["workspace_min_m"]) == len(limits["workspace_max_m"]) == 3, "Workspace dimensions")
    for lo,hi,a,b in zip(limits["workspace_min_m"],limits["workspace_max_m"],(-.6,-.6,.05),(.6,.6,.65)):
        require(a <= number(lo) < number(hi) <= b, "Original workspace cannot increase")
    require(len(limits["joint_limits_rad"]) == 6, "Six joint bounds required")
    for pair,nominal in zip(limits["joint_limits_rad"],JOINT_LIMITS_RAW):
        require(len(pair)==2 and nominal[0]*RAD_PER_RAW-1e-12 <= number(pair[0])
                < number(pair[1]) <= nominal[1]*RAD_PER_RAW+1e-12, "Original nominal bounds cannot increase")
    require(0 <= number(limits["gripper_min_m"]) < number(limits["gripper_max_m"]) <= .055, "Jaw bounds")
    return limits


def frames_for(raw):
    require(len(raw) == 6 and all(type(v) is int and lo <= v <= hi
            for v, (lo, hi) in zip(raw, JOINT_LIMITS_RAW)), "Nominal integer joint target required")
    return [(0x151, bytes((1, 1, 1, 0, 0, 0, 0, 0)))] + [
        (0x155+i, struct.pack(">ii", *raw[2*i:2*i+2])) for i in range(3)]


def path_check(before, raw_target, limits, fk):
    """Bound every independently progressing joint in the monitored .003 box."""
    frames_for(raw_target)
    target = [v*RAD_PER_RAW for v in raw_target]
    start_fk = fk(before["q"])
    pos_error = math.dist(start_fk[:3], before["pose"][:3])
    rot_error = rotation_distance(start_fk, before["pose"])
    require(pos_error <= .002 and rot_error <= .02, "FK/feedback mismatch")
    excursions = [abs(a-b)+.003 for a, b in zip(before["q"], target)]
    radii = fk.joint_radius_bounds_m
    require(len(radii) == 6 and all(math.isfinite(r) and r >= 0 for r in radii), "Invalid FK radii")
    position = pos_error + sum(r*d for r, d in zip(radii, excursions))
    rotation = rot_error + sum(excursions)
    require(position <= limits["max_translation_step_m"] and rotation <= limits["max_rotation_step_rad"],
            "Complete joint box exceeds original motion bounds")
    require(all(lo <= p-position and p+position <= hi for p, lo, hi in
                zip(before["pose"][:3], limits["workspace_min_m"], limits["workspace_max_m"])),
            "Complete joint box exceeds workspace")
    samples = []
    for i in range(21):
        q = [a+(b-a)*i/20 for a, b in zip(before["q"], target)]
        pose = fk(q)
        envelope(q, pose, before["pose"], limits)
        samples.append({"q": q, "pose": pose})
    return dict(target_raw=list(raw_target), target=target, position_bound_m=position,
                rotation_bound_rad=rotation, samples=samples,
                collision_verified=False, bound_conditional_on_monitored_joint_box=True)


def dynamic_goal(before):
    raw = list(before["raw_q"])
    require(raw[1] >= 600, "J2 needs at least 0.60deg toward-zero distance for a distinguishable probe")
    raw[1] -= min(1000, raw[1])
    return raw


class Probe:
    """Same worker sends start then overwrite; no parallel ticket or retry path."""
    def __init__(self, node, base, limits, fk, store, session, output, identity,
                 ownership_check, clock=time):
        self.node, self.base, self.limits, self.fk = node, base, limits, fk
        self.store, self.session, self.output = store, session, Path(output)
        self.identity, self.ownership_check, self.clock = identity, ownership_check, clock
        self.feedback = (self.output / "feedback.jsonl").open("a", buffering=1)

    def record(self, event, *, durable=False, **values):
        row = dict(event=event, unix_s=self.clock.time(), monotonic_s=self.clock.monotonic(), **values)
        self.feedback.write(json.dumps(row, allow_nan=False)+"\n")
        if durable:
            self.feedback.flush()
            os.fsync(self.feedback.fileno())
        if event != "feedback":
            self.base.emit("stop_probe_"+event, **values)
        return row

    def save(self):
        self.store.save(self.session)

    def read(self, *, moving=False):
        require(not self.node.rospy_probe.is_shutdown(), "ROS shutdown; accepted target not cancelled")
        with self.node.piper.rx_lock:
            s = self.node.piper.snapshot()
            s["raw_feedback"] = {hex(k): {"kernel_unix_s": v[0], "data_hex": v[1].hex()}
                                 for k, v in self.node.piper.received.items()}
        self.node.piper.healthy(s, allow_moving=moving)
        require(s["mode"] == 1, "Probe requires existing J mode; no mode transition experiment")
        require(self.clock.time()-min(s["stamps"]) <= self.limits["max_state_age_s"], "Configured age exceeded")
        require(self.limits["gripper_min_m"] <= s["opening_m"] <= self.limits["gripper_max_m"], "Jaw range")
        envelope(s["q"], s["pose"], s["pose"], self.limits)
        expected = self.fk(s["q"])
        require(math.dist(expected[:3], s["pose"][:3]) <= .002
                and rotation_distance(expected, s["pose"]) <= .02, "Live FK/feedback mismatch")
        self.node.piper.healthy(s, allow_moving=moving)  # Account for FK cost.
        return s

    def monitor(self, s, origin, target):
        envelope(s["q"], s["pose"], origin["pose"], self.limits)
        require(all(min(a,b)-.003 <= q <= max(a,b)+.003
                    for q,a,b in zip(s["q"], origin["q"], target)), "Outside original probe joint box")
        require(s["jaw_code"] == origin["jaw_code"] and abs(s["opening_m"]-origin["opening_m"]) <= .0005,
                "Jaw changed")

    @staticmethod
    def advanced(a, b):
        return b["sequence"] > a["sequence"] and all(x > y for x,y in zip(b["stamps"],a["stamps"]))

    def stable(self, origin=None, target=None, after=0., original_goal=None):
        deadline, last, window = self.clock.monotonic()+20., None, StableWindow()
        post_samples = []
        while self.clock.monotonic() < deadline:
            s = self.read(moving=after > 0)
            if origin is not None:
                self.monitor(s, origin, target)
            self.record("feedback", stage="hold" if after else "baseline", state=s)
            if after and min(s["stamps"]) > after:
                post_samples.append(copy.deepcopy(s))
            ready = s["motion_status"] == 0 and min(s["stamps"]) > after
            if target is not None:
                ready = ready and max(abs(a-b) for a,b in zip(s["q"],target)) <= .003
            if original_goal is not None:
                require(s["raw_q"][1]-original_goal[1] >= 200,
                        "Old goal approached within0.20deg; interruption not demonstrated")
            if not ready:
                window, last = StableWindow(), None
            elif last is None or self.advanced(last, s):
                done = window.add(s, self.clock.monotonic())
                last = s
                if done:
                    self.node.piper.healthy(s)
                    return s, {"duration_s": window.samples[-1][0]-window.samples[0][0],
                               "new_feedback_groups": len(window.samples),
                               "stable_window_first_kernel_unix_s": min(window.samples[0][1]["stamps"]),
                               "post_dispatch_samples": post_samples}
            self.clock.sleep(.05)
        raise RuntimeError("Three-second fresh hold not verified; no automatic recovery")

    @staticmethod
    def moving_reference(history, current):
        candidates = [s for s in history if .05 <= current["stamps"][4]-s["stamps"][4] <= .15]
        if not candidates:
            return None
        reference = min(candidates, key=lambda s: abs(current["stamps"][4]-s["stamps"][4]-.1))
        return reference if reference["raw_q"][1]-current["raw_q"][1] >= math.ceil(.0005/RAD_PER_RAW) else None

    def send(self, raw, measured, label, origin, original_target, *, dynamic=False, trend_reference=None):
        expected = frames_for(raw)
        path = path_check(measured, raw, self.limits, self.fk)
        require(self.node.piper.ticket is None, "Previous CAN ticket still active")
        self.ownership_check()
        self.session["pending"] = dict(label=label, raw=raw, prepared_unix_s=self.clock.time())
        self.save()  # Durable before any attempt.
        intent = self.record("intent", durable=True, label=label, path=path, measured=measured,
                             frames=[{"id": k,"data_hex": v.hex()} for k,v in expected])
        # Logging/FK/ownership may take time. Re-read all raw fragments immediately
        # before TX, refuse stale/slipping observations; never silently retarget.
        deadline = self.clock.monotonic()+.1
        fresh = self.read(moving=dynamic)
        while not self.advanced(measured, fresh) and self.clock.monotonic() < deadline:
            self.clock.sleep(.005)
            fresh = self.read(moving=dynamic)
        require(self.advanced(measured, fresh), "All fragments must advance before dispatch")
        require(max(abs(a-b) for a,b in zip(fresh["q"], measured["q"])) <= .003
                and math.dist(fresh["pose"][:3], measured["pose"][:3]) <= .0005
                and rotation_distance(fresh["pose"], measured["pose"]) <= .003,
                "Pre-send drift/slip exceeded")
        self.monitor(fresh, origin, original_target)
        if dynamic:
            require(origin["raw_q"][1]-fresh["raw_q"][1] >= 250
                    and fresh["raw_q"][1]-round(original_target[1]/RAD_PER_RAW) >= 350,
                    "Progress/remaining-distance trigger no longer valid")
            require(trend_reference is not None and self.moving_reference([trend_reference],fresh) is not None,
                    "No recent measured motion at overwrite dispatch")
        self.node.piper.healthy(measured, allow_moving=dynamic)
        self.node.piper.healthy(fresh, allow_moving=dynamic)
        require(self.node.adopted and not self.node.rospy_probe.is_shutdown(), "Shutdown before probe dispatch")
        ticket = dict(thread=threading.get_ident(), expected=list(expected), speed=1,
                      in_comm=False, attempted=0, sent=0)
        self.node.command_sequence += 1
        self.session["pending"]["sequence"] = self.node.command_sequence
        started = self.clock.time()
        self.node.piper.ticket = ticket
        try:
            self.node.piper.MotionCtrl_2(1,1,1,0,0,0)
            self.node.piper.JointCtrl(*raw)
            require(not self.node.piper.broken and not ticket["expected"]
                    and ticket["attempted"] == ticket["sent"] == 4, "Partial or failed transaction")
        finally:
            self.node.piper.ticket = None
            self.session["pending"].update(attempted_frames=ticket["attempted"], socket_send_returns=ticket["sent"])
            self.session["total_attempted_frames"] = self.session.get("total_attempted_frames",0)+ticket["attempted"]
            self.save()
            self.record("send_receipt", durable=True, label=label, sequence=self.node.command_sequence,
                        attempted_frames=ticket["attempted"], socket_send_returns=ticket["sent"],
                        started_unix_s=started, finished_unix_s=self.clock.time(),
                        before=fresh, requested_sample=measured, target_raw=raw)
        return self.clock.time(), fresh, intent

    def perform(self, kind):
        require(kind in ("static", "dynamic"), "Unknown probe")
        if not self.node.action_lock.acquire(False):
            return False, "Another probe is active; no concurrent request accepted"
        attempted = False
        try:
            require(self.node.adopted and not self.node.failed and not self.node.piper.broken,
                    "Probe driver not available")
            require(not self.session["failure"] and not self.session["pending"], "Latched or unresolved session")
            require(not self.session[kind+"_attempted"], "Each probe is allowed at most once")
            require(kind == "static" or self.session["static_passed"], "Static hold must pass first in this process")
            require(self.session["identity"] == self.identity, "Adoption identity changed")
            self.session[kind+"_attempted"] = True
            attempted = True
            self.save()
            self.node.active = True
            self.ownership_check()
            before, baseline_window = self.stable()
            if kind == "dynamic":
                prior = self.session["static_after"]
                require(max(abs(a-b) for a,b in zip(before["q"],prior["q"])) <= .003
                        and math.dist(before["pose"][:3],prior["pose"][:3]) <= .0005
                        and rotation_distance(before["pose"],prior["pose"]) <= .003
                        and abs(before["opening_m"]-prior["opening_m"]) <= .0005,
                        "State changed since static qualification")
            raw = list(before["raw_q"]) if kind == "static" else dynamic_goal(before)
            plan = path_check(before, raw, self.limits, self.fk)
            finished, _, _ = self.send(raw, before, kind+"_initial", before, plan["target"])
            overwrite = None
            if kind == "dynamic":
                deadline = self.clock.monotonic()+30.
                last = before
                history = []
                while self.clock.monotonic() < deadline:
                    current = self.read(moving=True)
                    self.monitor(current, before, plan["target"])
                    self.record("feedback", stage="moving", state=current)
                    if min(current["stamps"]) > finished and self.advanced(last,current):
                        last = current
                        progress = before["raw_q"][1]-current["raw_q"][1]
                        remaining = current["raw_q"][1]-raw[1]
                        reference = self.moving_reference(history,current)
                        history = [s for s in history if current["stamps"][4]-s["stamps"][4] <= .2]
                        history.append(copy.deepcopy(current))
                        if progress >= 250:
                            require(remaining >= 350, "Missed interruption window; no late overwrite")
                            if reference is None:
                                self.clock.sleep(.01)
                                continue
                            overwrite = copy.deepcopy(current)
                            hold_raw = list(raw)
                            hold_raw[1] = current["raw_q"][1]
                            require(all(min(a,b) <= v <= max(a,b) for v,a,b in
                                        zip(hold_raw,before["raw_q"],raw)), "Overwrite target outside original target box")
                            finished, trigger_fresh, _ = self.send(hold_raw, current, "dynamic_overwrite",
                                                                  before, plan["target"], dynamic=True,
                                                                  trend_reference=reference)
                            target = [v*RAD_PER_RAW for v in hold_raw]
                            break
                    self.clock.sleep(.01)
                require(overwrite is not None, "No timely measured motion; no retry or automatic stop")
            else:
                target = plan["target"]
            after, window = self.stable(before, target, finished, raw if kind == "dynamic" else None)
            self.ownership_check()
            post_samples = window.pop("post_dispatch_samples")
            baseline_window.pop("post_dispatch_samples")
            result = dict(kind=kind, success=True, before=before, after=after, baseline=baseline_window,
                          hold_window=window, initial_target_raw=raw, overwrite_sample=overwrite,
                          target_uncertain=False, general_stop_qualified=False, crash_safe_hold_verified=False,
                          scope="Only this unloaded 1-percent J probe and observed three-second interval")
            if overwrite is not None:
                result.update(overwrite_scope="J2 measured-position replacement; five other initial joint targets unchanged",
                              motion_metrics_scope="Observed maxima in nominal50ms feedback samples after all14 fragments advanced past send; not a continuous physical stopping-distance bound; initial transients may be missed",
                              send_finished_unix_s=finished,
                              stable_window_start_after_send_s=window["stable_window_first_kernel_unix_s"]-finished,
                              stability_confirmation_after_send_s=self.clock.time()-finished,
                              j2_net_after_overwrite_deg=(overwrite["raw_q"][1]-after["raw_q"][1])/1000,
                              end_reference_net_after_overwrite_m=math.dist(overwrite["pose"][:3],after["pose"][:3]),
                              final_distance_from_old_goal_deg=(after["raw_q"][1]-raw[1])/1000,
                              actual_dispatch_sample=trigger_fresh)
                for name, reference in (("requested_sample",overwrite),("dispatch_sample",trigger_fresh)):
                    result["max_motion_from_"+name] = dict(
                        joint_abs_rad=[max(abs(s["q"][i]-reference["q"][i]) for s in post_samples) for i in range(6)],
                        end_reference_m=max(math.dist(s["pose"][:3],reference["pose"][:3]) for s in post_samples),
                        rotation_rad=max(rotation_distance(s["pose"],reference["pose"]) for s in post_samples))
            self.record("verified", durable=True, result=result)
            self.session.update(pending=None)
            self.session[kind+"_passed"] = True
            self.session[kind+"_after"] = after
            self.save()
            SessionFile(self.output/(kind+"_result.json")).save(result)
            SessionFile(self.output/"result.json").save(result)
            return True, "Verified limited %s J probe; not general stopping qualification" % kind
        except Exception as error:
            if attempted:
                self.node.failed = str(error)
                self.session["failure"] = dict(error=str(error), unix_s=self.clock.time(),
                                               target_uncertain=self.session.get("total_attempted_frames",0) > 0,
                                               no_physical_recovery_sent=True)
                try:
                    self.save()
                finally:
                    self.record("failed", durable=True, failure=self.session["failure"])
                    result = dict(kind=kind,success=False,failure=self.session["failure"],general_stop_qualified=False)
                    SessionFile(self.output/(kind+"_result.json")).save(result)
                    SessionFile(self.output/"result.json").save(result)
            return False, str(error)
        finally:
            self.node.piper.ticket = None
            self.node.active = False
            self.node.action_lock.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-config", required=True)
    parser.add_argument("--probe-output-dir", required=True)
    parser.add_argument("--empty-gripper-confirmed", action="store_true", required=True,
                        help="Operator visual assertion, not inferred from jaw width")
    args, remaining = parser.parse_known_args(argv)
    require(remaining and not remaining[0].startswith("-"), "Pinned vendor script argument required")
    base = load_base()
    config = json.loads(Path(args.probe_config).read_text())
    limits = checked_probe_limits(config)
    output = Path(args.probe_output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    with SessionFile(SESSION_ROOT/(boot_id+".json")) as store:
        require(store.load() is None, "A probe session already exists for this boot; manual review required, no restart retry")
        session = dict(identity=None, pending=None, failure=None, static_attempted=False,
                       dynamic_attempted=False, static_passed=False, dynamic_passed=False)
        store.save(session)
        original_factory = base.node_class
        active_probe = []
        def restricted_factory(vendor, rospy):
            Parent = original_factory(vendor, rospy)
            class ProbeNode(Parent):
                def __init__(self):
                    self.rospy_probe = rospy
                    super().__init__()
                    import rosgraph
                    from std_srvs.srv import Trigger, TriggerResponse
                    def ownership():
                        base.binding()
                        require(rospy.get_param("~speed_percent") == 1
                                and type(rospy.get_param("~speed_percent")) is int, "Probe speed must remain integer1")
                        pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
                        for topic in ("joint_ctrl_single", "pos_cmd", "enable_flag"):
                            require(not dict(pubs).get(rospy.resolve_name(topic)), "Command publisher present")
                    ownership()
                    identity = dict(boot_id=boot_id, pid=os.getpid(), adopted_unix_s=time.time(),
                                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                    base_sha256=BASE_SHA, vendor_sha256=base.DRIVER_SHA256,
                                    config_sha256=hashlib.sha256(Path(args.probe_config).read_bytes()).hexdigest(),
                                    can_interface=base.CAN_NAME, usb_interface=base.USB_INTERFACE,
                                    empty_gripper_operator_confirmed=True)
                    session["identity"] = identity
                    store.save(session)
                    probe = Probe(self,base,limits,manufacturer_fk(),store,session,output,identity,ownership)
                    active_probe.append(probe)
                    def callback(kind):
                        def handle(_):
                            success, message = probe.perform(kind)
                            return TriggerResponse(success=success, message=message)
                        return handle
                    self.static_probe_service = rospy.Service("~static_hold_probe", Trigger, callback("static"))
                    self.dynamic_probe_service = rospy.Service("~dynamic_stop_probe", Trigger, callback("dynamic"))
                    probe.record("ready", durable=True, identity=identity, actuator_frames=0,
                                 services=["~static_hold_probe", "~dynamic_stop_probe"])

                def probe_only(self, *a, **k):
                    raise RuntimeError("Only the two bounded probe Trigger services are available")
                pos_callback = probe_only
                joint_callback = probe_only
                handle_gripper_service = probe_only
            return ProbeNode
        base.node_class = restricted_factory
        sys.argv = [str(Path(__file__))] + remaining
        try:
            base.main()
        finally:
            for probe in active_probe:
                probe.feedback.close()
            # Parent main performs only locked socket cleanup. No stop/disable.


if __name__ == "__main__":
    main()
