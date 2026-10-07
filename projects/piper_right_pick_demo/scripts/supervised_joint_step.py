#!/usr/bin/env python3
"""One supervised 0.55..1 degree ROS joint step; not an autonomous task route.

No retry, SDK control, stop, disable, reset, or qualification/configuration write.
Timeout leaves the accepted finite goal with the running driver and records it.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from commission_ros_client_exit import (ROSRightArm, check_telemetry, prepare,
    check_receipt, _last_command_sequence, rotation_distance, six, RAD_PER_RAW)


def assert_no_drift(before, current):
    """Match the frozen ROS driver's existing two-sample stability bounds.

    These are feedback tolerances, not motion permissions. The separate
    check_telemetry calls retain the existing raw age/health requirements.
    """
    if max(abs(a-b) for a, b in zip(before["q"], current["q"])) > .003:
        raise ValueError("Joint drift during publisher connection")
    if (max(abs(a-b) for a, b in zip(before["pose"][:3], current["pose"][:3])) > .0005
            or rotation_distance(before["pose"], current["pose"]) > .003):
        raise ValueError("End pose drift during publisher connection")
    if abs(before["opening_m"]-current["opening_m"]) > .0005:
        raise ValueError("Gripper drift during publisher connection")
    if current["sequence"] <= before["sequence"] or any(a < b for a, b in zip(current["stamps"], before["stamps"])):
        raise ValueError("New coherent feedback required after publisher connection")


def make_plan(axis, delta_deg, raw):
    if type(axis) is not int or not 1 <= axis <= 6:
        raise ValueError("axis must be 1..6")
    if type(delta_deg) not in (int, float) or not math.isfinite(delta_deg) or not .55 <= abs(delta_deg) <= 1.:
        raise ValueError("delta_deg magnitude must be 0.55..1.0")
    target = six(raw["q"])
    target[axis-1] += math.radians(delta_deg)
    return prepare("J", target, raw)


def bounds(raw, origin, limits):
    pose, q = six(raw["pose"]), six(raw["q"])
    if limits.get("max_speed_percent") != 1:
        raise ValueError("Explicit supervised site speed limit must be 1 percent")
    translation, rotation = limits["max_translation_step_m"], limits["max_rotation_step_rad"]
    if not 0 < translation <= .03 or not 0 < rotation <= .05:
        raise ValueError("Original 30mm/0.05rad caps must be retained")
    low, high, joints = limits["workspace_min_m"], limits["workspace_max_m"], limits["joint_limits_rad"]
    if len(low) != 3 or len(high) != 3 or len(joints) != 6:
        raise ValueError("Explicit workspace and six joint bounds required")
    if any(not a < b or not a <= x <= b for x, a, b in zip(pose[:3], low, high)):
        raise ValueError("EEF outside explicit workspace")
    if any(not a < b or not a <= x <= b for x, (a, b) in zip(q, joints)):
        raise ValueError("Joint outside explicit site limits")
    if math.dist(pose[:3], origin["pose"][:3]) > translation or rotation_distance(pose, origin["pose"]) > rotation:
        raise ValueError("EEF step exceeds original translation/rotation caps")


def check_planned_path(plan, before, limits):
    # Pure manufacturer FK only: no IK, object geometry, camera calibration,
    # robot interface construction or SDK communication.
    from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
    fk = C_PiperForwardKinematics(1)
    def pose(q):
        value = fk.CalFK(q)[-1]
        return [v/1000. for v in value[:3]] + [math.radians(v) for v in value[3:]]
    start = pose(before["q"])
    if math.dist(start[:3], before["pose"][:3]) > .002 or rotation_distance(start, before["pose"]) > .02:
        raise ValueError("Manufacturer FK and measured end reference disagree")
    path = []
    for i in range(21):
        q = [a+(b-a)*i/20. for a, b in zip(before["q"], plan["encoded_target"])]
        sample = dict(q=q, pose=pose(q))
        bounds(sample, before, limits)
        path.append(sample)
    return {"scope": "joint interpolation FK envelope only; not collision clearance", "samples": path}


def arrived(events, expected, plan, command_at, current):
    if not check_receipt(events, expected, plan, command_at):
        return False
    rows = [e for e in events if e.get("sequence") == expected]
    intent = next(e for e in rows if e.get("event") == "command_intent")
    sent = next(e for e in rows if e.get("event") == "command_sent_unconfirmed")
    stable = next((e for e in rows if e.get("event") == "command_observed_stable"), None)
    if stable is None:
        return False
    after = stable.get("after", {})
    if (stable.get("kind") != "joint" or stable.get("arm_target_reached") is not True
            or not command_at <= intent.get("unix_s", 0) <= sent.get("unix_s", 0) <= stable.get("unix_s", 0)
            or len(after.get("stamps", [])) != 14 or min(after["stamps"]) <= sent["unix_s"]):
        raise RuntimeError("Stable receipt lacks matching post-send joint arrival evidence")
    return (min(current["stamps"]) > max(after["stamps"]) and current["mode"] == 1
            and current["motion_status"] == 0 and not current["active_command"] and current["driver_accepts_commands"]
            and max(abs(a-b) for a, b in zip(six(current["q"]), plan["encoded_target"])) <= .003
            and max(abs(a-b) for a, b in zip(six(after.get("q")), plan["encoded_target"])) <= .003)


def run(config, axis, delta_deg, output, result):
    arm, publisher = ROSRightArm(config, proposal_only=True), None
    try:
        before = arm.observe(timeout_s=3.)
        transport, limits = arm._get_transport(), config["physical_limits"]
        initial = before["raw_telemetry"]
        make_plan(axis, delta_deg, initial)
        bounds(initial, initial, limits)
        if before["provenance"]["speed_percent"] != 1:
            raise RuntimeError("Existing driver must be at 1 percent")
        from sensor_msgs.msg import JointState
        publisher = transport.rospy.Publisher("/piper/right/joint_cmd", JointState, queue_size=1, latch=False)
        deadline = time.monotonic()+3.
        while publisher.get_num_connections() != 1:
            if time.monotonic() >= deadline:
                raise TimeoutError("Exactly one driver subscriber required; not sent")
            time.sleep(.01)
        fresh = transport.receive(1.)
        check_telemetry(fresh, time.time(), require_idle=True)
        assert_no_drift(initial, fresh)
        events = transport.events()
        expected = before["provenance"]["command_sequence"]+1
        publishers = dict(transport.master.getSystemState()[0])
        if (publishers.get("/piper/right/joint_cmd") != [transport.rospy.get_name()]
                or publishers.get(arm.ros["pose_topic"]) or publishers.get("/piper/right/enable_flag")
                or _last_command_sequence(events)+1 != expected or transport.master.getParam(arm.ros["speed_param"]) != 1):
            raise RuntimeError("Publisher ownership, command sequence or speed changed")
        plan = make_plan(axis, delta_deg, fresh)  # Relative to NEW feedback, never old absolute values.
        result.update(before=fresh, plan=plan, path_check=check_planned_path(plan, fresh, limits), expected_command_sequence=expected)
        (output/"before_plan.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
        # FK import and first disk write may age feedback. Take another real
        # sample and recompute every target from it; never relabel old state.
        newer = transport.receive(1.)
        check_telemetry(newer, time.time(), require_idle=True)
        assert_no_drift(fresh, newer)
        fresh, plan = newer, make_plan(axis, delta_deg, newer)
        result.update(before=fresh, plan=plan, path_check=check_planned_path(plan, fresh, limits))
        (output/"before_plan.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
        if _last_command_sequence(transport.events())+1 != expected or transport.master.getParam(arm.ros["speed_param"]) != 1:
            raise RuntimeError("Command sequence or speed changed before the sole publication")
        with (output/"trajectory.jsonl").open("x", encoding="utf-8") as trace:
            trace.write(json.dumps(fresh, allow_nan=False)+"\n"); trace.flush()
            check_telemetry(fresh, time.time(), require_idle=True)
            result.update(command_unix_s=time.time(), publish_attempts=1)
            publisher.publish(JointState(**plan["message"]))  # Sole publication.
            deadline, previous = time.monotonic()+120., fresh
            while time.monotonic() < deadline:
                try:
                    current = transport.receive(min(1., max(.001, deadline-time.monotonic())))
                except TimeoutError:
                    trace.write(json.dumps({"event": "feedback_receive_timeout", "unix_s": time.time()})+"\n"); trace.flush()
                    continue
                trace.write(json.dumps(current, allow_nan=False)+"\n"); trace.flush()
                result["after"] = current
                check_telemetry(current, time.time(), require_idle=False)
                if current["sequence"] <= previous["sequence"] or any(a < b for a, b in zip(current["stamps"], previous["stamps"])):
                    raise RuntimeError("Raw feedback sequence/timestamps regressed")
                bounds(current, fresh, limits)
                if any(abs(a-b) > (.003 if i != axis-1 else math.radians(1.)+.003)
                       for i, (a, b) in enumerate(zip(current["q"], fresh["q"]))):
                    raise RuntimeError("Observed joints left single-axis step envelope")
                previous, events = current, transport.events()
                if arrived(events, expected, plan, result["command_unix_s"], current):
                    result.update(status="supervised_joint_step_arrived", arrival_confirmed=True,
                                  receipt=[e for e in events if e.get("sequence") == expected])
                    return
            raise TimeoutError("120-second arrival timeout; no retry, stop or disable sent")
    finally:
        try:
            if publisher is not None:
                publisher.unregister()
        finally:
            arm.close()  # Only disconnect this client; keep the shared driver running.


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, required=True); p.add_argument("--axis", type=int, required=True)
    p.add_argument("--delta-deg", type=float, required=True); p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    result = dict(status="starting", axis=args.axis, delta_deg=args.delta_deg, publish_attempts=0,
                  qualified=False, task_attempted=False, started_at=time.time(), arrival_confirmed=False,
                  scope="one explicitly supervised relative J step only; no P or autonomous qualification")
    try:
        run(json.loads(args.config.read_text()), args.axis, args.delta_deg, args.output_dir, result)
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="failed", error_type=type(exc).__name__, error=str(exc)); code = 2
    result.update(finished_at=time.time(), target_uncertain=result["publish_attempts"] > 0 and not result["arrival_confirmed"])
    (args.output_dir/"result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"status": result["status"], "output": str(args.output_dir), "publish_attempts": result["publish_attempts"]}))
    return code


if __name__ == "__main__":
    sys.exit(main())
