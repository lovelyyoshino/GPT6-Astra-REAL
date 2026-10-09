#!/usr/bin/env python3
"""Receive-only Piper CAN snapshot. Run in the host terminal, never with sudo.

Does not bring interfaces up, instantiate C_PiperInterface, enable motors, query
firmware, or send any CAN frames. Uses the installed vendor 0.6.2 decoder only.
The operating CAN interface must already be up. No arm identity is inferred.
"""
import argparse
import enum
import importlib.metadata
import json
import math
import socket
import struct
import sys
import time


def plain(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, enum.Enum):
        return value.name
    if hasattr(value, "__dict__"):
        result = {key: plain(item) for key, item in vars(value).items()
                  if not key.startswith("_")}
        for key in ("err_code", "foc_status", "status_code"):
            if hasattr(value, key):
                result[key] = plain(getattr(value, key))
        return result
    return str(value)


RECEIVE_GROUPS = (("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"),
                  tuple("joint_%d" % n for n in range(1, 7)))
GROUP_SPAN_S = .002


def complete_receive_groups(times):
    """Timestamp-only grouping, never select samples by their measured values."""
    for group in RECEIVE_GROUPS:
        stamps = [times.get(name) for name in group]
        if not all(type(t) in (int, float) and math.isfinite(t) for t in stamps):
            return False
        if any(stamps[i] != stamps[i+1] for i in (0, 2, 4)):
            return False
        if not stamps[0] < stamps[2] < stamps[4] or stamps[4]-stamps[0] > GROUP_SPAN_S:
            return False
    return True


def collect(channel, seconds, include_pose_trace=False, coherent_pose_trace=False):
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not .2 <= seconds <= 15:
        raise ValueError("seconds must be within [0.2, 15]")
    version = importlib.metadata.version("piper_sdk")
    if version != "0.6.2":
        raise RuntimeError("Only inspected piper_sdk 0.6.2 is supported; found " + version)
    # These are data objects and the official decoder, never an SDK CAN interface.
    from can import Message
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
    from piper_sdk.piper_msgs.msg_v2 import PiperMessage

    parser = C_PiperParserV2()
    parser_module_path = getattr(importlib.import_module(
        "piper_sdk.protocol.protocol_v2.piper_protocol_v2"), "__file__", None)
    messages_module_path = getattr(importlib.import_module(
        "piper_sdk.piper_msgs.msg_v2"), "__file__", None)
    mappings = {
        "PiperMsgStatusFeedback": "arm_status_msgs",
        "PiperMsgGripperFeedBack": "gripper_feedback",
    }
    mappings.update({"PiperMsgEndPoseFeedback_%d" % i: "arm_end_pose" for i in range(1, 4)})
    mappings.update({"PiperMsgJointFeedBack_" + pair: "arm_joint_feedback" for pair in ("12", "34", "56")})
    mappings.update({"PiperMsgLowSpdFeed_%d" % i: "arm_low_spd_feedback_%d" % i for i in range(1, 7)})
    command_fields = {"PiperMsgJointCtrl_" + pair:
                      ("arm_joint_ctrl", tuple("joint_" + n for n in pair), "0.001 degree")
                      for pair in ("12", "34", "56")}
    command_fields["PiperMsgGripperCtrl"] = (
        "arm_gripper_ctrl", ("grippers_angle", "grippers_effort", "status_code", "set_zero"),
        {"grippers_angle": "0.001 mm", "grippers_effort": "0.001 N m",
         "status_code": "vendor enum", "set_zero": "vendor enum"})
    pose_fields = {"PiperMsgJointFeedBack_" + pair: tuple("joint_" + n for n in pair)
                   for pair in ("12", "34", "56")}
    pose_fields.update({"PiperMsgEndPoseFeedback_%d" % index: fields
                       for index, fields in ((1, ("X_axis", "Y_axis")),
                                             (2, ("Z_axis", "RX_axis")),
                                             (3, ("RY_axis", "RZ_axis")))})
    joint_names = tuple("joint_%d" % n for n in range(1, 7))
    end_names = ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")
    latest_pose = {}
    field_wall_times = {}
    field_monotonic_times = {}
    last_trace_at = None
    output = {"mode": "passive_receive_only", "channel": channel,
              "right_arm_identity_verified": False, "sdk_version": version,
              "sdk_parser_module_path": parser_module_path,
              "sdk_messages_module_path": messages_module_path,
              "started_at_s": time.time(), "duration_s": seconds,
              "frames_received": 0, "frames_sent_by_this_script": 0,
              "malformed_frames": 0, "frame_id_counts": {}, "raw_frame_latest": {}, "feedback": {},
              "command_feedback": {}, "frame_origin_counts": {"local": 0, "nonlocal": 0},
              "origin_basis": "SocketCAN recvmsg MSG_DONTROUTE means local socket loopback; otherwise nonlocal bus. This does not establish the sending device identity.",
              "motion_ready": False, "tcp_verified": False,
              "timestamp_basis": "local_receipt_time_not_hardware_clock",
              "source": "https://github.com/agilexrobotics/piper_sdk"}
    if include_pose_trace:
        output.update({"pose_trace": [], "pose_trace_max_rate_hz": 100,
                       "pose_trace_units": {"joints_raw": "0.001 degree",
                           "end_pose_raw": "position: 0.001 mm; Euler angles: 0.001 degree; vendor J6 reference"},
                       "pose_trace_note": "Latest actually received fields; not a simultaneous device measurement. Use per-field receipt times and frame_skew_s to assess alignment."})
        if coherent_pose_trace:
            output.update(pose_trace_assembly={"schema":"piper_complete_receive_groups_v1",
                "max_group_receive_span_s":GROUP_SPAN_S,"complete_groups_only":True,
                "firmware_cycle_ids_available":False,"whole_snapshot_atomic":False},
                pose_trace_incomplete_updates=0)
    # Linux struct can_frame is transport framing, not a handwritten Piper protocol.
    frame_format = struct.Struct("=IB3x8s")
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
        receiver.bind((channel,))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            receiver.settimeout(min(0.1, max(.001, deadline - time.monotonic())))
            try:
                raw, ancillary, flags, address = receiver.recvmsg(frame_format.size)
            except socket.timeout:
                continue
            received_monotonic = time.monotonic()
            received = time.time()
            if received_monotonic >= deadline:
                break
            if len(raw) != frame_format.size or flags & socket.MSG_TRUNC:
                output["malformed_frames"] += 1
                continue
            can_id, dlc, payload = frame_format.unpack(raw)
            output["frames_received"] += 1
            origin = "local" if flags & socket.MSG_DONTROUTE else "nonlocal"
            output["frame_origin_counts"][origin] += 1
            id_hex = "0x%03X" % (can_id & socket.CAN_EFF_MASK)
            output["frame_id_counts"][id_hex] = output["frame_id_counts"].get(id_hex, 0) + 1
            if can_id & (socket.CAN_ERR_FLAG | socket.CAN_RTR_FLAG | socket.CAN_EFF_FLAG) or dlc != 8:
                continue
            # Keep the actual bytes even for vendor messages we do not aggregate.
            # Administrative packets alone are not joint feedback or motion readiness.
            output["raw_frame_latest"][id_hex] = {
                "payload_hex": payload.hex(), "origin": origin,
                "received_at_s": received, "received_monotonic_s": received_monotonic,
            }
            frame = Message(arbitration_id=can_id & socket.CAN_SFF_MASK,
                            data=payload, timestamp=received, is_extended_id=False)
            decoded = PiperMessage()
            if not parser.DecodeMessage(frame, decoded):
                continue
            message_type = decoded.type_.name
            if message_type in command_fields:
                attribute, names, units = command_fields[message_type]
                decoded_fields = plain(getattr(decoded, attribute))
                previous = output["command_feedback"].get(message_type, {})
                origins = dict(previous.get("origin_counts", {"local": 0, "nonlocal": 0}))
                origins[origin] += 1
                by_origin = dict(previous.get("latest_by_origin", {}))
                previous_source = by_origin.get(origin, {})
                current_fields = {name: decoded_fields[name] for name in names}
                item = {
                    "received_at_s": received,
                    "received_monotonic_s": received_monotonic,
                    "origin": origin, "recvmsg_flags": flags,
                    "fields": current_fields,
                    "field_min": {name: min(value, previous_source.get("field_min", {}).get(name, value))
                                  for name, value in current_fields.items()},
                    "field_max": {name: max(value, previous_source.get("field_max", {}).get(name, value))
                                  for name, value in current_fields.items()},
                    "units": units, "observed_count": origins[origin],
                }
                by_origin[origin] = item
                output["command_feedback"][message_type] = dict(
                    item, observed_count=sum(origins.values()), origin_counts=origins,
                    latest_by_origin=by_origin)
            if message_type in mappings:
                output["feedback"][message_type] = {
                    "received_at_s": received,
                    "received_monotonic_s": received_monotonic,
                    "origin": origin,
                    "fields": plain(getattr(decoded, mappings[message_type])),
                    "vendor_text": str(getattr(decoded, mappings[message_type])),
                }
                if include_pose_trace and message_type in pose_fields:
                    # Each freshly constructed vendor object has zeros for fields
                    # carried by other CAN frames. Never treat those as observed.
                    fields = output["feedback"][message_type]["fields"]
                    for name in pose_fields[message_type]:
                        latest_pose[name] = fields[name]
                        field_wall_times[name] = received
                        field_monotonic_times[name] = received_monotonic
                    if coherent_pose_trace and not (complete_receive_groups(field_wall_times)
                                                     and complete_receive_groups(field_monotonic_times)):
                        output['pose_trace_incomplete_updates'] += 1
                        continue
                    if (len(latest_pose) == 12 and
                            (last_trace_at is None or received_monotonic - last_trace_at >= .01)):
                        output["pose_trace"].append({
                            "received_at_s": received,
                            "received_monotonic_s": received_monotonic,
                            "joints_raw": {name: latest_pose[name] for name in joint_names},
                            "end_pose_raw": {name: latest_pose[name] for name in end_names},
                            "field_received_at_s": dict(field_wall_times),
                            "field_received_monotonic_s": dict(field_monotonic_times),
                            "frame_skew_s": max(field_monotonic_times.values()) - min(field_monotonic_times.values()),
                        })
                        last_trace_at = received_monotonic
    output["finished_at_s"] = time.time()
    output["finished_monotonic_s"] = time.monotonic()
    for item in output["command_feedback"].values():
        item["age_s_at_finish"] = output["finished_monotonic_s"] - item["received_monotonic_s"]
        for source_item in item["latest_by_origin"].values():
            source_item["age_s_at_finish"] = output["finished_monotonic_s"] - source_item["received_monotonic_s"]
    output["missing_feedback_types"] = sorted(set(mappings) - set(output["feedback"]))
    # Each split feedback frame is reported separately; untouched zero defaults
    # in another pair of fields are not measurements. Mark its valid fields.
    for pair in ("12", "34", "56"):
        item = output["feedback"].get("PiperMsgJointFeedBack_" + pair)
        if item:
            item["fields"] = {"joint_" + n: item["fields"]["joint_" + n] for n in pair}
            item["units"] = "0.001 degree"
            item.pop("vendor_text", None)
    for index, fields in ((1, ("X_axis", "Y_axis")), (2, ("Z_axis", "RX_axis")),
                          (3, ("RY_axis", "RZ_axis"))):
        item = output["feedback"].get("PiperMsgEndPoseFeedback_%d" % index)
        if item:
            item["fields"] = {name: item["fields"][name] for name in fields}
            item["units"] = "position: 0.001 mm; Euler angles: 0.001 degree; vendor J6 reference"
            item.pop("vendor_text", None)
    for item in output["feedback"].values():
        item["age_s_at_finish"] = output["finished_monotonic_s"] - item["received_monotonic_s"]
    output["complete_feedback_received"] = not output["missing_feedback_types"]
    output["note"] = "Diagnostic only. Data may include other publishers or controllers. No TCP, calibration, workspace, firmware or right-arm binding is validated."
    return output


def main():
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument("--channel", default="can2")
    argument_parser.add_argument("--seconds", default=3.0, type=float)
    argument_parser.add_argument("--pose-trace", action="store_true",
                                 help="record complete joint/end feedback at up to 100 Hz, with per-field receipt times")
    argument_parser.add_argument("--coherent-pose-trace", action="store_true",
                                 help="record only ordered complete pose/joint receive groups; implies --pose-trace")
    args = argument_parser.parse_args()
    if not math.isfinite(args.seconds) or not .2 <= args.seconds <= 15:
        argument_parser.error("--seconds must be within [0.2, 15]")
    if not args.channel or len(args.channel) > 15 or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in args.channel):
        argument_parser.error("invalid CAN interface name")
    try:
        kwargs = {"include_pose_trace":args.pose_trace or args.coherent_pose_trace}
        if args.coherent_pose_trace:
            kwargs["coherent_pose_trace"] = True
        output = collect(args.channel, args.seconds, **kwargs)
        print(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False))
        return 0 if output["frames_received"] else 2
    except Exception as exc:
        print(json.dumps({"mode": "passive_receive_only", "channel": args.channel,
            "status": "unavailable", "error": str(exc),
            "error_type": type(exc).__name__, "frames_sent_by_this_script": 0,
            "interface_changed": False, "motion_ready": False},
            indent=2, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    sys.exit(main())
