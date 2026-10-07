#!/usr/bin/env python3
"""One bounded, supervised ROS client-exit commissioning transaction.

Not a task executor or a qualification gate override. One J/P publication only;
no SDK/CAN connection, retries, stop, reset, disable or driver shutdown.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from right_pick.fast_ros import (ROSRightArm, check_telemetry, _last_command_sequence,
    _last_jaw_target, JOINT_LIMITS_RAW, RAD_PER_RAW)
from right_pick.fast_safety import rotation_distance


def six(values):
    if not isinstance(values, list) or len(values) != 6 or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError("Target must be exactly six finite numeric values")
    return list(map(float, values))


def prepare(kind, requested, before, jaw_target=None):
    """Pure bounded numeric preparation; encodings match the frozen ROS owner."""
    requested = six(requested)
    q, pose = six(before["q"]), six(before["pose"])
    frames = []
    def frame(identifier, data):
        return {"id": identifier, "data_hex": data.hex()}
    if kind == "J":
        deltas = [abs(a-b) for a, b in zip(requested, q)]
        axis = max(range(6), key=deltas.__getitem__)
        if not math.radians(.55) <= deltas[axis] <= math.radians(1.) + 1e-12:
            raise ValueError("J commissioning requires one 0.55..1 degree joint target")
        if any(deltas[i] > .001 for i in range(6) if i != axis):
            raise ValueError("J target changes more than one joint")
        # Preserve the other five actual joints after the connection wait.
        target = list(q)
        target[axis] = requested[axis]
        raw = [round(v / RAD_PER_RAW) for v in target]
        if any(not low <= value <= high for value, (low, high) in zip(raw, JOINT_LIMITS_RAW)):
            raise ValueError("J target exceeds manufacturer nominal limits")
        encoded = [value * RAD_PER_RAW for value in raw]
        if abs(encoded[axis]-q[axis]) > math.radians(1.) + RAD_PER_RAW / 2:
            raise ValueError("Encoded J target exceeds commissioning displacement")
        frames = [frame(0x151, bytes((1, 1, 1, 0, 0, 0, 0, 0)))]
        frames += [frame(0x155+i, struct.pack(">ii", *raw[2*i:2*i+2])) for i in range(3)]
        return dict(kind=kind, target=target, encoded_target=encoded, requested_target=requested,
                    axis=axis, units="radians", expected_frames=frames,
                    message={"position": target, "velocity": [0.]*6+[1.]})
    if kind != "P":
        raise ValueError("Only bounded J or P commissioning is permitted")
    if type(jaw_target) not in (int, float) or not math.isfinite(jaw_target) or not 0 <= jaw_target <= .055:
        raise ValueError("P requires an established commanded jaw target")
    factor = 180 / 3.1415926
    raw = [round(v*1000)*1000 for v in requested[:3]] + [round(v*1000*factor) for v in requested[3:]]
    encoded = [v/1e6 for v in raw[:3]] + [v*RAD_PER_RAW for v in raw[3:]]
    for target in (requested, encoded):
        distance = math.dist(target[:3], pose[:3])
        if not .0035 <= distance <= .006 + 1e-12:
            raise ValueError("P commissioning requires 3.5..6 mm translation")
        if target[2] < pose[2] - 1e-12:
            raise ValueError("P commissioning must not descend")
        if rotation_distance(target, pose) > .003:
            raise ValueError("P rotation exceeds .003 radians")
        if any(abs(v) > 1 for v in target[:3]) or any(abs(v) > 2*math.pi for v in target[3:]):
            raise ValueError("P target exceeds vendor encoding envelope")
    mode = frame(0x151, bytes((1, 0, 1, 0, 0, 0, 0, 0)))
    frames = [frame(0x150, bytes(8)), mode]
    frames += [frame(0x152+i, struct.pack(">ii", *raw[2*i:2*i+2])) for i in range(3)]
    frames += [frame(0x159, struct.pack(">iHBB", round(jaw_target*1e6), 200, 1, 0)), mode]
    return dict(kind=kind, requested_target=requested, target=requested, encoded_target=encoded,
                units="metres_radians", jaw_target_m=jaw_target, expected_frames=frames,
                message=dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), requested),
                             gripper=jaw_target, mode1=0, mode2=0))


def assert_no_drift(before, current):
    if max(abs(a-b) for a, b in zip(before["q"], current["q"])) > .001:
        raise ValueError("Joint drift during publisher connection")
    if math.dist(before["pose"][:3], current["pose"][:3]) > .0005 or rotation_distance(before["pose"], current["pose"]) > .001:
        raise ValueError("End pose drift during publisher connection")
    if current["sequence"] <= before["sequence"] or any(a < b for a, b in zip(current["stamps"], before["stamps"])):
        raise ValueError("New coherent feedback required after publisher connection")


def progress(plan, before, current, recent=()):
    if plan["kind"] == "J":
        axis = plan["axis"]
        direction = 1 if plan["encoded_target"][axis] > before["q"][axis] else -1
        moved = (current["q"][axis]-before["q"][axis]) * direction
        remaining = abs(plan["encoded_target"][axis]-current["q"][axis])
        ready = moved >= math.radians(.2) and remaining > math.radians(.3)
        trend_min = .0005
        source_indices = (4 + axis // 2,)
    else:
        direction = [a-b for a, b in zip(plan["encoded_target"][:3], before["pose"][:3])]
        length = math.sqrt(sum(v*v for v in direction))
        moved = sum((a-b)*v for a, b, v in zip(current["pose"][:3], before["pose"][:3], direction)) / length
        remaining = math.dist(plan["encoded_target"][:3], current["pose"][:3])
        ready = moved >= .0012 and remaining > .002
        trend_min = .0003
        source_indices = (1, 2, 3)
    trend = None
    for earlier in recent:
        dt = current["stamp"] - earlier["stamp"]
        if (not 0 < dt <= .1 or earlier["sequence"] >= current["sequence"]
                or any(earlier["stamps"][i] >= current["stamps"][i] for i in source_indices)):
            continue
        old_error = (abs(plan["encoded_target"][axis] - earlier["q"][axis])
                     if plan["kind"] == "J" else
                     math.dist(plan["encoded_target"][:3], earlier["pose"][:3]))
        advance = old_error - remaining
        if advance >= trend_min:
            trend = {"basis": "fresh_measured_target_error_reduction_not_motion_status_bit",
                     "earlier": earlier, "window_s": dt,
                     "target_error_reduction": advance, "minimum_reduction": trend_min}
            break
    # Actual J trace showed motion_status=0 throughout a genuine 1-degree move.
    # Retain that flag exactly; motion must be proved from two fresh raw states.
    correct_mode = current.get("mode") == (1 if plan["kind"] == "J" else 0)
    return {"progress": moved, "remaining": remaining, "motion_status": current["motion_status"],
            "measured_motion_evidence": trend,
            "exit_condition_met": ready and trend is not None and correct_mode}


def check_receipt(events, expected_sequence, plan, command_at):
    sequence = _last_command_sequence(events)
    if sequence > expected_sequence:
        raise RuntimeError("Unexpected additional command sequence; no retry")
    rows = [event for event in events if event.get("sequence") == expected_sequence]
    intent = next((event for event in rows if event.get("event") == "command_intent"), None)
    sent = next((event for event in rows if event.get("event") == "command_sent_unconfirmed"), None)
    if intent is None:
        return False
    if (intent.get("kind") != {"J": "joint", "P": "pose"}[plan["kind"]]
            or intent.get("frames") != plan["expected_frames"]
            or intent.get("speed_percent") != 1 or intent.get("unix_s", 0) < command_at):
        raise RuntimeError("Single commissioning command receipt mismatch")
    if sent is None:
        return False
    count = len(plan["expected_frames"])
    if sent.get("attempted_frames") != count or sent.get("socket_send_returns") != count:
        raise RuntimeError("Partial command dispatch; no retry")
    return True


def run(config, kind, target, result):
    arm = ROSRightArm(config, proposal_only=True)
    publisher = None
    try:
        before = arm.observe(timeout_s=3.)  # Includes one live transport identity check.
        transport = arm._get_transport()
        rospy = transport.rospy
        if before["provenance"]["speed_percent"] != 1:
            raise RuntimeError("Commissioning requires existing driver speed_percent=1")
        initial = before["raw_telemetry"]
        prepare(kind, target, initial, before["provenance"]["jaw_command_target_m"])
        if kind == "J":
            from sensor_msgs.msg import JointState
            message_type, topic = JointState, "/piper/right/joint_cmd"
        else:
            from piper_msgs.msg import PosCmd
            message_type, topic = PosCmd, arm.ros["pose_topic"]
        publisher = rospy.Publisher(topic, message_type, queue_size=1, latch=False)
        deadline = time.monotonic() + 3.
        while publisher.get_num_connections() != 1:
            if time.monotonic() >= deadline:
                raise TimeoutError("Expected exactly one driver subscriber; nothing published")
            time.sleep(.01)
        fresh = transport.receive(1.)
        check_telemetry(fresh, time.time(), require_idle=True)
        assert_no_drift(initial, fresh)
        events = transport.events()
        if _last_command_sequence(events) != before["provenance"]["command_sequence"]:
            raise RuntimeError("Command history changed while connecting publisher")
        if transport.master.getParam(arm.ros["speed_param"]) != 1:
            raise RuntimeError("Commissioning speed changed before publication")
        plan = prepare(kind, target, fresh, _last_jaw_target(events))
        expected = before["provenance"]["command_sequence"] + 1
        result.update(before=fresh, initial_observation=before, plan=plan,
                      expected_command_sequence=expected, topic=topic, speed_percent=1)
        check_telemetry(fresh, time.time(), require_idle=True)
        result["command_unix_s"] = time.time()
        result["publish_attempts"] = 1
        publisher.publish(message_type(**plan["message"]))  # THE SOLE PUBLICATION.
        deadline = time.monotonic() + 10.
        previous = fresh
        recent = [fresh]
        while time.monotonic() < deadline:
            try:
                current = transport.receive(min(1., max(.001, deadline-time.monotonic())))
            except TimeoutError:
                if time.monotonic() >= deadline:
                    break  # Deadline expiry is not a claim of a telemetry failure.
                raise
            check_telemetry(current, time.time(), require_idle=False)
            if current["sequence"] <= previous["sequence"] or any(a < b for a, b in zip(current["stamps"], previous["stamps"])):
                raise RuntimeError("Feedback sequence or timestamps failed to advance")
            previous = current
            received = check_receipt(transport.events(), expected, plan, result["command_unix_s"])
            observed = progress(plan, fresh, current, recent)
            result.update(final=current, progress_at_exit=observed, matching_send_receipt=received)
            if received and observed["exit_condition_met"]:
                result["status"] = "exiting_client_during_bounded_motion"
                return
            recent = [state for state in recent if current["stamp"] - state["stamp"] <= .1]
            recent.append(current)
        raise TimeoutError("No commissioning exit condition within 10 seconds; no retry/stop/disable")
    finally:
        result["disconnect_started_unix_s"] = time.time()
        errors = []
        if publisher is not None:
            try:
                publisher.unregister()
            except Exception as exc:
                errors.append(str(exc))
        try:
            arm.close()
        except Exception as exc:
            errors.append(str(exc))
        result["client_exit_unix_s"] = time.time()
        result["disconnect_errors"] = errors
        if errors:
            raise RuntimeError("ROS client cleanup failed: " + repr(errors))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", required=True, choices=("J", "P"))
    parser.add_argument("--target-file", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    target = six(json.loads(args.target_file.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = {"status": "starting", "kind": args.kind, "started_at": time.time(),
              "publish_attempts": 0, "qualified": False, "task_attempted": False,
              "scope": "single supervised ROS client-exit commissioning only"}
    with args.output.open("x", encoding="utf-8") as stream:
        try:
            run(config, args.kind, target, result)
            code = 0
        except Exception as exc:
            result.update(status="failed", error_type=type(exc).__name__, error=str(exc))
            code = 2
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "output": str(args.output.resolve()),
                      "publish_attempts": result["publish_attempts"]}), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
