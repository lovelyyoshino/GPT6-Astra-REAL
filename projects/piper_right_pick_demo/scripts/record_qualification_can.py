#!/usr/bin/env python3
"""Bounded receive-only right-arm SocketCAN trace; never transmits a frame.

Raw bytes and real per-frame receipt times only. No SDK, robot decoder, motion,
interface setup or geometric inference. This trace alone is not qualification.
"""
import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

CHANNEL = "can1"
USB_INTERFACE = "1-6.3:1.0"
FEEDBACK_IDS = frozenset(range(0x2A1, 0x2A9)) | frozenset(range(0x261, 0x267))
CONTROL_IDS = frozenset(range(0x150, 0x160)) | frozenset(range(0x470, 0x500))
CAN_FRAME = struct.Struct("=IB3x8s")
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)  # Linux SO_TIMESTAMPNS_OLD
SO_RXQ_OVFL = getattr(socket, "SO_RXQ_OVFL", 40)
TIMESPEC = struct.Struct("@ll")


def verify_binding(sys_root=Path("/sys/class/net")):
    device = (Path(sys_root) / CHANNEL / "device").resolve(strict=True)
    if USB_INTERFACE not in device.parts:
        raise RuntimeError("Right-arm USB binding mismatch: " + str(device))
    interface = Path(sys_root) / CHANNEL
    if int((interface / "type").read_text().strip()) != 280:
        raise RuntimeError("can1 is not a SocketCAN interface")
    if not int((interface / "flags").read_text().strip(), 0) & 1:
        raise RuntimeError("can1 is DOWN; this passive tool never changes interfaces")
    return {"channel": CHANNEL, "expected_usb_interface": USB_INTERFACE,
            "resolved_device": str(device), "binding_verified": True}


def parse_received(raw, ancillary, flags, host_received_at):
    """Decode Linux transport framing only; preserve the eight payload bytes."""
    if len(raw) != CAN_FRAME.size or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
        raise ValueError("truncated/malformed SocketCAN message or ancillary data")
    can_id, dlc, payload = CAN_FRAME.unpack(raw)
    if can_id & (socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
        raise ValueError("nonstandard/RTR/error CAN frame")
    if dlc != 8 or can_id > socket.CAN_SFF_MASK:
        raise ValueError("CAN frame must be standard identifier and eight bytes")
    stamp, basis, dropped = host_received_at, "host_recvmsg_return_unix", None
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
            if len(data) < TIMESPEC.size:
                raise ValueError("short kernel timestamp")
            seconds, nanos = TIMESPEC.unpack(data[:TIMESPEC.size])
            if seconds <= 0 or not 0 <= nanos < 1000000000:
                raise ValueError("invalid kernel timestamp")
            stamp = seconds + nanos / 1e9
            basis = "kernel_socket_SO_TIMESTAMPNS_unix"
        elif level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL:
            if len(data) != 4:
                raise ValueError("invalid SocketCAN overflow counter")
            dropped = struct.unpack("=I", data)[0]
    if not math.isfinite(stamp) or stamp > host_received_at:
        raise ValueError("nonfinite/future frame receipt time")
    result = {"id": can_id, "timestamp": stamp, "data_hex": payload.hex(),
              "timestamp_basis": basis, "host_received_at": host_received_at,
              "origin": "local_socket_loopback" if flags & socket.MSG_DONTROUTE else "bus_or_nonlocal"}
    if dropped is not None:
        result["socket_dropped_total"] = dropped
    return result


def complete_sample(latest, sampled_at, max_age_s=0.1):
    if set(latest) != FEEDBACK_IDS:
        return None, "missing_feedback"
    times = [latest[identifier]["timestamp"] for identifier in FEEDBACK_IDS]
    if min(times) > sampled_at or max(times) > sampled_at:
        return None, "future_feedback"
    if sampled_at - min(times) > max_age_s:
        return None, "stale_feedback"
    if max(times) - min(times) > max_age_s:
        return None, "feedback_skew"
    # New dicts avoid later mutation changing the historical sample.
    return {"sampled_at": sampled_at,
            "frames": [dict(latest[identifier]) for identifier in sorted(FEEDBACK_IDS)]}, None


def collect(seconds, ready=None, socket_factory=None, clock=time, binding_check=verify_binding):
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 2 <= seconds <= 120:
        raise ValueError("seconds must be within [2, 120]")
    binding = binding_check()
    socket_factory = socket_factory or socket.socket
    report = {"mode": "passive_right_can_qualification_recording", "binding": binding,
              "frames_sent_by_this_script": 0, "sdk_used": False,
              "interface_changed": False, "duration_requested_s": seconds,
              "sample_rate_limit_hz": 50, "max_frame_age_s": .1, "max_frame_skew_s": .1,
              "feedback_ids": sorted(FEEDBACK_IDS), "samples": [], "control_frames": [],
              "frame_id_counts": {}, "bad_frames": [], "sample_gaps": [],
              "skipped_samples": {}, "timestamp_backwards": [],
              "socket_dropped_total": 0, "timestamp_sources": [],
              "origin_note": "local loopback does not identify which process sent a frame",
              "control_id_note": "Raw command/administrative identifier ranges, not decoded commands",
              "qualified": False}
    latest, timestamp_sources = {}, set()
    with socket_factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
        try:
            receiver.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            report["kernel_timestamp_enabled"] = True
        except OSError as exc:
            report.update(kernel_timestamp_enabled=False, timestamp_fallback_reason=str(exc))
        try:
            receiver.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
            report["overflow_ancillary_enabled"] = True
        except OSError as exc:
            report.update(overflow_ancillary_enabled=False, overflow_monitor_note=str(exc))
        receiver.bind((CHANNEL,))
        start = clock.monotonic()
        deadline, next_tick = start + seconds, start + .02
        report["started_at"] = clock.time()
        if ready:
            ready({"event": "ready", "channel": CHANNEL, "started_at": report["started_at"],
                   "duration_s": seconds, "frames_sent_by_this_script": 0,
                   "binding_verified": True})
        while clock.monotonic() < deadline:
            now = clock.monotonic()
            receiver.settimeout(max(.0001, min(next_tick, deadline) - now))
            try:
                raw, ancillary, flags, _ = receiver.recvmsg(CAN_FRAME.size, socket.CMSG_SPACE(TIMESPEC.size) + socket.CMSG_SPACE(4))
                received = clock.time()
                try:
                    frame = parse_received(raw, ancillary, flags, received)
                except ValueError as exc:
                    report["bad_frames"].append({"timestamp": received, "reason": str(exc),
                                                 "transport_hex": raw.hex()})
                else:
                    identifier = frame["id"]
                    key = "0x%03X" % identifier
                    report["frame_id_counts"][key] = report["frame_id_counts"].get(key, 0) + 1
                    timestamp_sources.add(frame["timestamp_basis"])
                    report["socket_dropped_total"] = max(report["socket_dropped_total"], frame.get("socket_dropped_total", 0))
                    if identifier in FEEDBACK_IDS:
                        if identifier in latest and frame["timestamp"] < latest[identifier]["timestamp"]:
                            report["timestamp_backwards"].append(dict(frame))
                        else:
                            latest[identifier] = frame
                    elif identifier in CONTROL_IDS:
                        report["control_frames"].append(frame)
            except socket.timeout:
                pass
            now = clock.monotonic()
            if now >= next_tick:
                sample, why = complete_sample(latest, clock.time())
                if sample is not None:
                    if report["samples"]:
                        gap = sample["sampled_at"] - report["samples"][-1]["sampled_at"]
                        if gap > .1 or gap <= 0:
                            report["sample_gaps"].append({"sampled_at": sample["sampled_at"], "gap_s": gap})
                    report["samples"].append(sample)
                else:
                    report["skipped_samples"][why] = report["skipped_samples"].get(why, 0) + 1
                # No catch-up samples: each sample has its actual acquisition time.
                next_tick += max(1, int((now - next_tick) / .02) + 1) * .02
    report["finished_at"] = clock.time()
    report["timestamp_sources"] = sorted(timestamp_sources)
    report["missing_feedback_ids"] = sorted(FEEDBACK_IDS - set(latest))
    report["sample_count"] = len(report["samples"])
    report["status"] = "recorded" if report["samples"] else "no_complete_samples"
    report["trace_transport_clean"] = bool(report["samples"]) and not any(
        (report["bad_frames"], report["sample_gaps"], report["timestamp_backwards"], report["socket_dropped_total"]))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 120:
        parser.error("--seconds must be within [2, 120]")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation protects earlier qualification traces from overwrite.
    with args.output.open("x", encoding="utf-8") as output:
        try:
            result = collect(args.seconds, ready=lambda event: print(json.dumps(event), flush=True))
            code = 0 if result["trace_transport_clean"] else 2
        except Exception as exc:
            result = {"status": "failed", "error_type": type(exc).__name__, "error": str(exc),
                      "frames_sent_by_this_script": 0, "interface_changed": False, "qualified": False}
            code = 2
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps({"event": "closed", "status": result["status"],
                      "sample_count": result.get("sample_count", 0), "output": str(args.output.resolve())}), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
