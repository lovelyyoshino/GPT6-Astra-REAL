#!/usr/bin/env python3
"""One supervised ROS segment toward existing joint zeros, never zero calibration.

Default is a fresh-state proposal, with no command publisher. --execute permits
one JointState only, at the already configured 50 percent. The 30 degree joint
ceiling does not replace the original 30 mm / .05 rad end-reference bounds.
No SDK control, parameter writes, driver lifecycle, retry or physical recovery.
The retained finite firmware target is NOT cancelled by client timeout/exit.
"""
import argparse
import copy
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from right_pick.fast_ros import (ROSRightArm, check_telemetry, _last_command_sequence,
                                JOINT_LIMITS_RAW, RAD_PER_RAW, rotation_distance)
from ros_home_step import SessionFile, StableWindow

SESSION_ROOT = ROOT / "runs" / "supervised_ros_home_sessions"
TOPIC = "/piper/right/joint_cmd"


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def finite(value):
    require(type(value) in (int, float) and math.isfinite(value), "Finite number required")
    return float(value)


def checked_limits(config, max_joint_step_deg):
    limit = config["physical_limits"]
    require(type(limit.get("max_speed_percent")) is int and limit["max_speed_percent"] == 50,
            "Independent home configuration must explicitly specify max_speed_percent=50")
    require(0 < finite(max_joint_step_deg) <= 30, "Joint ceiling must be within (0,30] degrees")
    for key, cap in (("max_translation_step_m", .03), ("max_rotation_step_rad", .05),
                     ("max_state_age_s", .1)):
        require(0 < finite(limit.get(key)) <= cap, "Original safety bound may not increase: " + key)
    low, high = limit["workspace_min_m"], limit["workspace_max_m"]
    require(len(low) == len(high) == 3, "Three explicit workspace bounds required")
    for lo, hi, outer_lo, outer_hi in zip(low, high, (-.6, -.6, .05), (.6, .6, .65)):
        require(outer_lo <= finite(lo) < finite(hi) <= outer_hi, "Workspace exceeds original site bounds")
    require(len(limit["joint_limits_rad"]) == 6, "Six explicit joint bounds required")
    for pair, nominal in zip(limit["joint_limits_rad"], JOINT_LIMITS_RAW):
        require(len(pair) == 2, "Joint bound pair required")
        lo, hi = map(finite, pair)
        require(nominal[0]*RAD_PER_RAW-1e-12 <= lo < hi <= nominal[1]*RAD_PER_RAW+1e-12,
                "Site joint bounds cannot exceed manufacturer nominal bounds")
    require(0 <= finite(limit["gripper_min_m"]) < finite(limit["gripper_max_m"]) <= .055,
            "Original jaw range must be retained")
    return copy.deepcopy(limit)


def manufacturer_fk():
    # Kinematics only. No C_PiperInterface instance or SDK communication.
    from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
    engine = C_PiperForwardKinematics(1)
    def calculate(q):
        value = engine.CalFK(q)[-1]
        return [v/1000 for v in value[:3]] + [math.radians(v) for v in value[3:]]
    # Modified-DH translations after joint i bound that joint's end-reference
    # lever arm in every posture. Its own d is along its rotation axis.
    calculate.joint_radius_bounds_m = [sum(math.hypot(engine._a[k], engine._d[k])
                                         for k in range(i+1, 6))/1000 for i in range(6)]
    return calculate


def envelope(q, pose, origin, limits):
    require(len(q) == len(pose) == 6 and all(math.isfinite(v) for v in q+pose), "Invalid FK/state vector")
    require(all(lo <= v <= hi for v, lo, hi in zip(pose[:3], limits["workspace_min_m"], limits["workspace_max_m"])),
            "End reference outside original workspace")
    require(all(lo <= v <= hi for v, (lo, hi) in zip(q, limits["joint_limits_rad"])), "Joint outside site bounds")
    require(math.dist(pose[:3], origin[:3]) <= limits["max_translation_step_m"]
            and rotation_distance(pose, origin) <= limits["max_rotation_step_rad"],
            "End-reference segment exceeds original 30mm/.05rad bounds")


def state_check(raw, limits, now, *, idle):
    check_telemetry(raw, now, require_idle=idle)
    require(now-min(raw["stamps"]) <= limits["max_state_age_s"], "Configured feedback age exceeded")
    require(limits["gripper_min_m"] <= raw["opening_m"] <= limits["gripper_max_m"], "Jaw outside site range")
    envelope(raw["q"], raw["pose"], raw["pose"], limits)


def new_feedback(previous, current, *, all_fragments=False):
    require(current["sequence"] > previous["sequence"], "Feedback sequence did not advance")
    require(all(a > b if all_fragments else a >= b for a, b in zip(current["stamps"], previous["stamps"])),
            "Feedback fragment timestamp regressed or failed to advance")


def no_drift(a, b):
    new_feedback(a, b)
    require(max(abs(x-y) for x, y in zip(a["q"], b["q"])) <= .003, "Pre-send joint drift")
    require(math.dist(a["pose"][:3], b["pose"][:3]) <= .0005
            and rotation_distance(a["pose"], b["pose"]) <= .003, "Pre-send end-reference drift")
    require(a["jaw_code"] == b["jaw_code"] and abs(a["opening_m"]-b["opening_m"]) <= .0005, "Pre-send jaw drift")


def candidate(before, scale, cap_deg, limits, fk):
    start = before["raw_q"]
    raw = [round(v*(1-scale)) for v in start]
    require(any(a != b for a, b in zip(raw, start)), "No encoded joint motion toward zero")
    require(all(a*b >= 0 and abs(b) <= abs(a) for a, b in zip(start, raw)), "Target must move only toward existing zero")
    require(all(lo <= v <= hi for v, (lo, hi) in zip(raw, JOINT_LIMITS_RAW)), "Non-nominal target")
    require(max(abs(a-b) for a, b in zip(start, raw)) <= cap_deg*1000+1e-8, "Encoded joint ceiling exceeded")
    target = [v*RAD_PER_RAW for v in raw]
    require([round(v*(1000*180/math.pi)) for v in target] == raw, "Vendor encoding round-trip mismatch")
    start_fk = fk(before["q"])
    position_error = math.dist(start_fk[:3], before["pose"][:3])
    orientation_error = rotation_distance(start_fk, before["pose"])
    require(position_error <= .002 and orientation_error <= .02, "Manufacturer FK/feedback mismatch")
    # The firmware's relative progress of different joints is not assumed.
    # Bound the whole monitored independent joint box, including its .003rad
    # tracking margin. Sum-of-axis-angle and lever-arm integral bounds cover
    # all states in that box, not just a synchronous FK line or its corners.
    radii = fk.joint_radius_bounds_m
    require(len(radii) == 6 and all(math.isfinite(r) and r >= 0 for r in radii), "Missing FK geometry radii")
    excursions = [abs(a-b)+.003 for a,b in zip(before["q"], target)]
    box_distance = position_error+sum(r*d for r,d in zip(radii, excursions))
    box_rotation = orientation_error+sum(excursions)
    require(box_distance <= limits["max_translation_step_m"] and box_rotation <= limits["max_rotation_step_rad"],
            "Conservative joint-box displacement/rotation bound exceeded")
    require(all(lo <= v-box_distance and v+box_distance <= hi for v,lo,hi in
                zip(before["pose"][:3],limits["workspace_min_m"],limits["workspace_max_m"])),
            "Conservative joint-box workspace bound exceeded")
    intervals = max(20, math.ceil(max(abs(a-b) for a, b in zip(start, raw))/50))
    samples = []
    for i in range(intervals+1):
        q = [a+(b-a)*i/intervals for a, b in zip(before["q"], target)]
        pose = fk(q)
        envelope(q, pose, before["pose"], limits)
        samples.append(dict(q=q, pose=pose))
    frames = [{"id": 0x151, "data_hex": bytes((1, 1, 50, 0, 0, 0, 0, 0)).hex()}]
    frames += [{"id": 0x155+i, "data_hex": struct.pack(">ii", *raw[2*i:2*i+2]).hex()} for i in range(3)]
    return dict(kind="J", scale=scale, target_raw=raw, target=target, expected_frames=frames,
                speed_percent=50, max_joint_step_deg=cap_deg,
                message={"position": target, "velocity": [0.]*6+[50.]},
                joint_box_bound={"position_m":box_distance,"rotation_rad":box_rotation,
                                 "axis_radius_bounds_m":radii,"tracking_margin_rad":.003,
                                 "scope":"End-reference bound conditional on staying inside monitored joint box; not body clearance or stopping qualification"},
                path_check={"samples": samples, "sample_max_joint_spacing_deg": .05,
                            "scope": "Sampled manufacturer FK end-reference envelope, not collision or dynamic path certification"})


def make_plan(before, cap_deg, limits, fk, *, scale_hint=None):
    require(any(before["raw_q"]), "Already exact six-zero feedback; no publication needed")
    upper = min(1., cap_deg*1000/max(abs(v) for v in before["raw_q"]))
    if scale_hint is not None:
        # Cheap final recomputation after obtaining NEW feedback. A changed
        # state that no longer fits is refused, not silently replanned on send.
        return candidate(before, min(upper, scale_hint), cap_deg, limits, fk)
    low, best, high = 0., None, upper
    for iteration in range(20):
        scale = upper if iteration == 0 else (low+high)/2
        try:
            plan = candidate(before, scale, cap_deg, limits, fk)
        except RuntimeError as error:
            if str(error) not in ("End-reference segment exceeds original 30mm/.05rad bounds",
                                   "End reference outside original workspace", "Encoded joint ceiling exceeded",
                                   "Conservative joint-box displacement/rotation bound exceeded",
                                   "Conservative joint-box workspace bound exceeded"):
                raise
            high = scale
            continue
        low, best = scale, plan
        if scale == upper or high-low < 1e-7:
            break
    require(best is not None, "No bounded nonzero common-scale segment found")
    # Reserve numerical/feedback headroom, unless the complete exact-zero goal fits.
    return best if best["scale"] == 1 else candidate(before, best["scale"]*.95, cap_deg, limits, fk)


def monitor(raw, before, plan, limits, fk):
    envelope(raw["q"], raw["pose"], before["pose"], limits)
    require(all(min(a, b)-.003 <= v <= max(a, b)+.003
                for v, a, b in zip(raw["q"], before["q"], plan["target"])), "Joint left initial-to-target box")
    require(raw["jaw_code"] == before["jaw_code"] and abs(raw["opening_m"]-before["opening_m"]) <= .0005,
            "Jaw changed during home segment")
    expected = fk(raw["q"])
    require(math.dist(expected[:3], raw["pose"][:3]) <= .002
            and rotation_distance(expected, raw["pose"]) <= .02, "Live FK/feedback mismatch")


def matching_receipt(events, sequence, plan, command_at):
    require(_last_command_sequence(events) <= sequence, "Unexpected additional command")
    require(not any(e.get("event") == "command_refused_or_failed" and e.get("unix_s",0) >= command_at
                    for e in events), "Driver reports command refusal/failure")
    rows = [e for e in events if e.get("sequence") == sequence]
    require(len({e["event"] for e in rows}) == len(rows), "Duplicate command receipt event")
    require(not any(e.get("event") == "command_failed" for e in rows), "Driver reports command failure")
    by = {e["event"]: e for e in rows}
    intent = by.get("command_intent")
    if intent is None:
        return None
    require(intent.get("kind") == "joint" and intent.get("speed_percent") == 50
            and intent.get("frames") == plan["expected_frames"] and intent["unix_s"] >= command_at,
            "Home command intent mismatch")
    sent = by.get("command_sent_unconfirmed")
    if sent is None:
        return None
    require(sent.get("attempted_frames") == sent.get("socket_send_returns") == 4
            and sent["unix_s"] >= intent["unix_s"], "Partial/mismatched home dispatch")
    stable = by.get("command_observed_stable")
    if stable is None:
        return None
    require(stable.get("kind") == "joint" and stable.get("arm_target_reached") is True
            and stable["unix_s"] >= sent["unix_s"], "Invalid stable arrival receipt")
    state = stable.get("after", {})
    require(len(state.get("stamps", [])) == 14 and len(state.get("q",[])) == 6 and min(state["stamps"]) > sent["unix_s"]
            and max(abs(a-b) for a, b in zip(state.get("q", []), plan["target"])) <= .003,
            "Receipt lacks fresh near-target joints")
    return stable


def session_identity(provenance):
    keys = ("driver_pid", "current_driver_adoption_unix_s", "adapter_sha256", "vendor_sha256",
            "can_interface", "usb_interface", "driver_node", "command_log")
    require(provenance.get("binding_verified") is True and provenance.get("source_verified") is True,
            "Pinned live identity required")
    identity = {key: provenance[key] for key in keys}
    require(type(identity["driver_pid"]) is int and identity["driver_pid"] > 0
            and finite(identity["current_driver_adoption_unix_s"]) > 0, "Invalid driver session identity")
    return identity


def graph_check(transport, arm, sequence, publisher, speed=50):
    pubs, subs, _ = transport.master.getSystemState()
    expected = [transport.rospy.get_name()] if publisher else []
    require(dict(pubs).get(TOPIC, []) == expected
            and not dict(pubs).get(arm.ros["pose_topic"])
            and not dict(pubs).get("/piper/right/enable_flag")
            and dict(subs).get(TOPIC) == [arm.ros["driver_node"]]
            and dict(pubs).get(arm.ros["telemetry_topic"]) == [arm.ros["driver_node"]],
            "Exclusive ROS command/subscriber ownership changed")
    live_speed = transport.master.getParam(arm.ros["speed_param"])
    require(type(live_speed) is int and live_speed == speed, "Live speed changed or differs from required value; never changed here")
    require(_last_command_sequence(transport.events()) == sequence, "Command sequence changed")


def run(config, max_joint_step_deg, execute, output, result, *, arm_factory=ROSRightArm,
        fk_factory=manufacturer_fk, session_root=SESSION_ROOT, clock=time, message_factory=None):
    limits = checked_limits(config, max_joint_step_deg)
    arm, publisher, session, store = arm_factory(config, proposal_only=True), None, None, None
    resources = ExitStack()
    try:
        initial = arm.observe()
        identity = session_identity(initial["provenance"])
        live_speed = initial["provenance"]["speed_percent"]
        result.update(planned_speed_percent=50, live_speed_percent=live_speed, speed_mismatch=live_speed != 50)
        require(not execute or live_speed == 50, "Execution requires live speed50; proposal may inspect current speed")
        # Filename does not depend on output/config/command sequence, so a new
        # output directory cannot bypass a failed or unresolved transaction.
        key = hashlib.sha256(json.dumps([identity["driver_pid"], identity["current_driver_adoption_unix_s"]]).encode()).hexdigest()[:24]
        store = resources.enter_context(SessionFile(Path(session_root)/(key+".json")))
        session = store.load()
        sequence = initial["provenance"]["command_sequence"]
        result["session_path"] = str(store.path)
        if session is None:
            session = dict(identity=identity, last_sequence=sequence, pending=None, failure=None, completed_segments=0)
        require(session["identity"] == identity and not session["failure"] and session["pending"] is None,
                "Home session identity changed, failed or unresolved; no retry")
        require(session["last_sequence"] == sequence, "Other actions occurred since this home session")
        result["first_segment_pilot_required"] = session["completed_segments"] == 0
        require(not execute or session["completed_segments"] > 0 or max_joint_step_deg <= 1,
                "First executed segment in this driver session must use joint ceiling <=1degree")
        transport, fk = arm._get_transport(), fk_factory()
        a = initial["raw_telemetry"]
        state_check(a, limits, clock.time(), idle=True)
        plan = make_plan(a, max_joint_step_deg, limits, fk)
        fresh_state = arm.observe()  # Revalidates hashes/binding/owner before any publisher.
        require(session_identity(fresh_state["provenance"]) == identity, "Live driver identity changed")
        fresh = fresh_state["raw_telemetry"]
        state_check(fresh, limits, clock.time(), idle=True); no_drift(a, fresh)
        plan = make_plan(fresh, max_joint_step_deg, limits, fk, scale_hint=plan["scale"])
        result.update(before=fresh, plan=plan, expected_command_sequence=sequence+1)
        graph_check(transport, arm, sequence, False, speed=live_speed)
        if not execute:
            state_check(fresh, limits, clock.time(), idle=True)
            result.update(status="proposal_only", not_dispatched=True, arrival_confirmed=False)
            return
        if message_factory is None:
            from sensor_msgs.msg import JointState
            message_factory = JointState
        publisher = transport.rospy.Publisher(TOPIC, message_factory, queue_size=1, latch=False)
        deadline = clock.monotonic()+3
        while publisher.get_num_connections() != 1:
            require(clock.monotonic() < deadline, "Exactly one subscriber required before send")
            clock.sleep(.01)
        graph_check(transport, arm, sequence, True)
        newest = transport.receive(1.)
        state_check(newest, limits, clock.time(), idle=True); no_drift(fresh, newest)
        plan = make_plan(newest, max_joint_step_deg, limits, fk, scale_hint=plan["scale"])
        before = newest
        result.update(before=before, plan=plan)
        (output/"before_plan.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
        graph_check(transport, arm, sequence, True)
        session["pending"] = dict(sequence=sequence+1, output=str(output), plan=plan, at=clock.time())
        store.save(session)  # Durable before the sole publication.
        state_check(before, limits, clock.time(), idle=True)
        result.update(publish_attempts=1, command_unix_s=clock.time())
        publisher.publish(message_factory(**plan["message"]))
        deadline, previous, window = clock.monotonic()+120, before, StableWindow()
        with (output/"trajectory.jsonl").open("x") as trace:
            while clock.monotonic() < deadline:
                raw = transport.receive(1.)
                trace.write(json.dumps(raw, allow_nan=False)+"\n"); trace.flush()
                result["after"] = raw
                state_check(raw, limits, clock.time(), idle=False); new_feedback(previous, raw)
                monitor(raw, before, plan, limits, fk); previous = raw
                events = transport.events()
                stable = matching_receipt(events, sequence+1, plan, result["command_unix_s"])
                require(transport.master.getParam(arm.ros["speed_param"]) == 50, "Speed changed during segment")
                ready = (stable is not None and min(raw["stamps"]) > max(stable["after"]["stamps"])
                         and raw["mode"] == 1 and raw["motion_status"] == 0
                         and not raw["active_command"] and raw["driver_accepts_commands"]
                         and max(abs(a-b) for a, b in zip(raw["q"], plan["target"])) <= .003)
                if not ready:
                    window = StableWindow(); continue
                if window.samples and not all(a > b for a,b in zip(raw["stamps"], window.samples[-1][1]["stamps"])):
                    continue  # Do not count a repeated fragment as a new stable sample.
                if not window.add(raw, clock.monotonic()):
                    continue
                graph_check(transport, arm, sequence+1, True)
                publisher.unregister(); publisher = None
                final = arm.observe()  # Full identity, source, binding and sole-owner recheck.
                require(session_identity(final["provenance"]) == identity
                        and final["provenance"]["command_sequence"] == sequence+1, "Final identity/sequence changed")
                end = final["raw_telemetry"]
                state_check(end, limits, clock.time(), idle=True); no_drift(raw, end)
                monitor(end, before, plan, limits, fk)
                require(max(abs(a-b) for a,b in zip(end["q"],plan["target"])) <= .003,
                        "Final fresh feedback no longer near target")
                require(sum(abs(v)for v in end["raw_q"]) < sum(abs(v)for v in before["raw_q"]),
                        "No measured progress toward existing zero")
                session.update(last_sequence=sequence+1, pending=None, completed_segments=session["completed_segments"]+1)
                store.save(session)
                result.update(status="supervised_home_segment_arrived", arrival_confirmed=True, after=end,
                              receipt=[e for e in events if e.get("sequence")==sequence+1],
                              stable_duration_s=window.samples[-1][0]-window.samples[0][0],
                              stable_feedback_groups=len(window.samples), measured_exact_zero=all(v==0 for v in end["raw_q"]),
                              measured_near_zero=max(abs(v)for v in end["q"]) <= .003)
                return
        raise TimeoutError("120s arrival/stability timeout; finite firmware target not cancelled")
    except BaseException as error:
        result.update(status="failed", error_type=type(error).__name__, error=str(error),
                      target_uncertain=result.get("publish_attempts",0)>0)
        if execute and session is not None and store is not None and not session.get("failure"):
            # The same session lock remains held through durable failure logging.
            session["failure"] = dict(reason=str(error), at=clock.time(), target_uncertain=result["target_uncertain"])
            store.save(session)
        raise
    finally:
        try:
            try:
                if publisher is not None: publisher.unregister()
            finally:
                arm.close()
        except BaseException as error:
            result.update(status="failed", cleanup_error=str(error),
                          target_uncertain=result.get("publish_attempts",0)>0 and not result.get("arrival_confirmed",False))
            if execute and session is not None and store is not None and not session.get("failure"):
                session["failure"] = dict(reason="Client cleanup failed: "+str(error), at=clock.time(),
                                          target_uncertain=result["target_uncertain"])
                store.save(session)
            raise
        finally:
            resources.close()


def offline_proposal(config, state_path, cap_deg, result):
    """Historical file/FK only; no ROS adapter construction or live freshness claim."""
    limits = checked_limits(config, cap_deg)
    data = state_path.read_bytes()
    state = json.loads(data)
    raw = state.get("raw_telemetry", state)
    state_check(raw, limits, raw["stamp"], idle=True)
    plan = make_plan(raw, cap_deg, limits, manufacturer_fk())
    result.update(status="historical_proposal_only", not_dispatched=True, historical_state=True,
                  live_freshness_verified=False, requires_fresh_live_replanning=True,
                  planned_speed_percent=50, source_state_file=str(state_path),
                  source_state_sha256=hashlib.sha256(data).hexdigest(), before=raw, plan=plan)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-joint-step-deg", type=float, default=30.)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--state-file", type=Path, help="Historical offline proposal only; never with --execute")
    args = parser.parse_args(argv)
    if args.state_file is not None and args.execute:
        parser.error("--state-file is proposal-only and cannot be combined with --execute")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    result = dict(status="starting", started_at=time.time(), publish_attempts=0, arrival_confirmed=False,
                  execute_requested=args.execute, task_success=False, general_high_speed_qualified=False,
                  hold_verified=False, collision_path_verified=False, automatic_retry=False,
                  physical_stop_sent=False, disable_sent=False, reset_sent=False, driver_retained=True)
    try:
        config = json.loads(args.config.read_text())
        result["home_authorization"] = config.get("home_authorization")
        if args.state_file is not None:
            offline_proposal(config, args.state_file, args.max_joint_step_deg, result)
        else:
            run(config, args.max_joint_step_deg, args.execute, args.output_dir, result)
        code = 0
    except (Exception, KeyboardInterrupt) as error:
        result.update(status="failed", error_type=type(error).__name__, error=str(error)); code = 2
    result.update(finished_at=time.time(), target_uncertain=result["publish_attempts"]>0 and not result["arrival_confirmed"])
    (args.output_dir/"result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"status":result["status"],"publish_attempts":result["publish_attempts"],"output":str(args.output_dir)}))
    return code


if __name__ == "__main__":
    sys.exit(main())
