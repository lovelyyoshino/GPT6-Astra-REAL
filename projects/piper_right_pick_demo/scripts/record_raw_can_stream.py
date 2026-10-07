#!/usr/bin/env python3
"""Stream every right-arm feedback frame without SDK, ROS or CAN transmission.

Fixed can1/USB binding. The 14 feedback IDs and observed control IDs are written
without subsampling; other identifiers are counted. Fresh-ready is a transport
condition, not a robot-health or motion authorization. An absent control frame
does not prove zero TX when a sender has local loopback disabled.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_qualification_can as transport


def _kernel_ns(ancillary):
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == transport.SO_TIMESTAMPNS:
            seconds, nanos = transport.TIMESPEC.unpack(data[:transport.TIMESPEC.size])
            return str(seconds * 1000000000 + nanos)
    return None


def collect_stream(seconds, emit, *, ready=None, stop_reason=lambda: None,
                   socket_factory=None, clock=time,
                   binding_check=transport.verify_binding):
    """Receive only. ``emit`` consumes one row; no frame history is retained.

    Fake socket/clock injection is provided for offline tests. A three-second
    startup budget bounds waiting for the first complete fresh feedback set.
    The caller owns output flushing and handles incomplete/crashed recordings.
    """
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 2 <= seconds <= 600:
        raise ValueError("seconds must be within [2, 600]")
    report = dict(schema_version=1, mode="receive_only_raw_can_stream", channel=transport.CHANNEL,
                  frames_sent_by_this_script=0, sdk_used=False, interface_changed=False,
                  duration_requested_s=seconds, max_frame_age_s=.1, max_frame_skew_s=.1,
                  feedback_ids=sorted(transport.FEEDBACK_IDS), received_count=0,
                  feedback_frame_count=0, control_frame_count=0, other_frame_count=0,
                  frame_id_counts={}, bad_frame_count=0, timestamp_backwards_count=0,
                  kernel_timestamp_missing_count=0, socket_dropped_total=0,
                  overflow_counter_regressions=0, feedback_gap_count=0,
                  stale_window_count=0, max_feedback_gap_s=0., max_frame_age_observed_s=0.,
                  fresh_ready=False, ready_at=None, first_frame_kernel_unix_s=None,
                  last_frame_kernel_unix_s=None, first_feedback_kernel_unix_s=None,
                  last_feedback_kernel_unix_s=None, kernel_timestamp_enabled=False,
                  overflow_ancillary_enabled=False, robot_health_evaluated=False,
                  qualified=False, trace_transport_clean=False)
    latest, sources, last_stamp, overflow, stale_since = {}, set(), None, 0, None
    start = clock.monotonic()
    report["started_at"] = clock.time()
    reason, fatal = None, None

    def anomaly(event, **details):
        emit(dict(event=event, observed_unix_s=clock.time(), **details))

    def no_transport_errors():
        return not any(report[k] for k in (
            "bad_frame_count", "timestamp_backwards_count", "kernel_timestamp_missing_count",
            "socket_dropped_total", "overflow_counter_regressions", "feedback_gap_count",
            "stale_window_count"))

    try:
        report["binding"] = binding_check()
        socket_factory = socket_factory or socket.socket
        with socket_factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
            # No silent userspace timestamp fallback: ready requires both options.
            receiver.setsockopt(socket.SOL_SOCKET, transport.SO_TIMESTAMPNS, 1)
            report["kernel_timestamp_enabled"] = True
            receiver.setsockopt(socket.SOL_SOCKET, transport.SO_RXQ_OVFL, 1)
            report["overflow_ancillary_enabled"] = True
            receiver.bind((transport.CHANNEL,))
            start = clock.monotonic()
            report["started_at"] = clock.time()
            deadline, next_check = start + seconds, start + .02
            emit(dict(event="recording_started", **report))
            while clock.monotonic() < deadline:
                requested = stop_reason()
                if requested:
                    reason = requested if isinstance(requested, str) else "operator_requested"
                    break
                now = clock.monotonic()
                receiver.settimeout(max(.0001, min(next_check, deadline) - now))
                try:
                    raw, ancillary, flags, _ = receiver.recvmsg(
                        transport.CAN_FRAME.size,
                        socket.CMSG_SPACE(transport.TIMESPEC.size) + socket.CMSG_SPACE(4))
                    received = clock.time()
                    report["received_count"] += 1
                    try:
                        frame = transport.parse_received(raw, ancillary, flags, received)
                    except ValueError as exc:
                        report["bad_frame_count"] += 1
                        anomaly("bad_frame", received_index=report["received_count"],
                                reason=str(exc), transport_hex=raw.hex(), msg_flags=flags,
                                ancillary=[dict(level=a, kind=b, data_hex=c.hex()) for a, b, c in ancillary])
                    else:
                        ident, stamp = frame["id"], frame["timestamp"]
                        key = "0x%03X" % ident
                        report["frame_id_counts"][key] = report["frame_id_counts"].get(key, 0) + 1
                        sources.add(frame["timestamp_basis"])
                        exact_ns = _kernel_ns(ancillary)
                        frame["kernel_timestamp_ns"] = exact_ns
                        frame["msg_flags"] = flags
                        if exact_ns is None:
                            report["kernel_timestamp_missing_count"] += 1
                            anomaly("kernel_timestamp_missing", received_index=report["received_count"], id=ident)
                        else:
                            if report["first_frame_kernel_unix_s"] is None:
                                report["first_frame_kernel_unix_s"] = stamp
                            report["last_frame_kernel_unix_s"] = stamp
                        if last_stamp is not None and stamp < last_stamp:
                            report["timestamp_backwards_count"] += 1
                            anomaly("timestamp_backwards", id=ident, previous=last_stamp, current=stamp)
                        last_stamp = stamp
                        dropped = frame.get("socket_dropped_total")
                        if dropped is not None:
                            if dropped < overflow:
                                report["overflow_counter_regressions"] += 1
                                anomaly("overflow_counter_regressed", previous=overflow, current=dropped)
                            if dropped != overflow:
                                anomaly("socket_overflow", previous=overflow, current=dropped)
                            overflow = dropped
                            report["socket_dropped_total"] = max(report["socket_dropped_total"], dropped)
                        if ident in transport.FEEDBACK_IDS:
                            report["feedback_frame_count"] += 1
                            if exact_ns is not None:
                                if report["first_feedback_kernel_unix_s"] is None:
                                    report["first_feedback_kernel_unix_s"] = stamp
                                report["last_feedback_kernel_unix_s"] = stamp
                            if ident in latest:
                                gap = stamp - latest[ident]["timestamp"]
                                report["max_feedback_gap_s"] = max(report["max_feedback_gap_s"], gap)
                                if gap > .1:
                                    report["feedback_gap_count"] += 1
                                    anomaly("feedback_gap", id=ident, gap_s=gap,
                                            previous=latest[ident]["timestamp"], current=stamp)
                            if ident not in latest or stamp >= latest[ident]["timestamp"]:
                                latest[ident] = frame
                            emit(dict(event="frame", role="feedback", received_index=report["received_count"], **frame))
                        elif ident in transport.CONTROL_IDS:
                            report["control_frame_count"] += 1
                            emit(dict(event="frame", role="observed_control", received_index=report["received_count"], **frame))
                        else:
                            report["other_frame_count"] += 1
                except socket.timeout:
                    pass
                now = clock.monotonic()
                if now >= next_check:
                    sample, why = transport.complete_sample(latest, clock.time())
                    if latest:
                        age = clock.time() - min(v["timestamp"] for v in latest.values())
                        report["max_frame_age_observed_s"] = max(report["max_frame_age_observed_s"], age)
                    if not report["fresh_ready"]:
                        if sample is not None and no_transport_errors():
                            report.update(fresh_ready=True, ready_at=clock.time())
                            event = dict(event="fresh_ready", started_at=report["started_at"],
                                         ready_at=report["ready_at"], channel=transport.CHANNEL,
                                         complete_feedback_id_count=14, max_frame_age_s=.1,
                                         frames_sent_by_this_script=0, robot_health_evaluated=False)
                            emit(event)
                            if ready:
                                ready(event)
                        elif now - start >= 3.:
                            reason = "fresh_ready_timeout"
                            anomaly(reason, missing_feedback_ids=sorted(transport.FEEDBACK_IDS - set(latest)),
                                    last_sample_problem=why)
                            break
                    elif sample is None:
                        if stale_since is None:
                            stale_since = clock.time()
                            report["stale_window_count"] += 1
                            anomaly("feedback_unavailable", reason=why)
                    elif stale_since is not None:
                        anomaly("feedback_recovered", unavailable_since=stale_since)
                        stale_since = None
                    next_check += max(1, int((now-next_check)/.02)+1) * .02
            if reason is None:
                reason = "duration_elapsed"
    except Exception as exc:
        fatal = dict(error_type=type(exc).__name__, error=str(exc))
        reason = "recording_error"
        anomaly("recording_error", **fatal)
    if report["fresh_ready"] and transport.complete_sample(latest, clock.time())[0] is None:
        if stale_since is None:
            stale_since = clock.time()
            report["stale_window_count"] += 1
            anomaly("feedback_unavailable_at_close")
    report.update(finished_at=clock.time(), elapsed_s=clock.monotonic()-start,
                  close_reason=reason, timestamp_sources=sorted(sources),
                  missing_feedback_ids=sorted(transport.FEEDBACK_IDS-set(latest)))
    if fatal:
        report["error"] = fatal
    report["trace_transport_clean"] = bool(report["fresh_ready"] and no_transport_errors()
                                             and fatal is None and stale_since is None)
    report["status"] = "recorded" if report["trace_transport_clean"] else "incomplete_or_transport_error"
    report["recording_scope"] = "Every received feedback/control frame; other identifiers counted only. No sample interpolation."
    report["control_frame_limit"] = "Missing control frames do not prove zero TX; sender local loopback may be disabled."
    return report


class JsonlSink:
    def __init__(self, file):
        self.file, self.digest, self.rows = file, hashlib.sha256(), 0
        self.last_flush = time.monotonic()

    def __call__(self, row):
        encoded = (json.dumps(row, separators=(",", ":"), allow_nan=False)+"\n").encode()
        self.file.write(encoded)
        self.digest.update(encoded)
        self.rows += 1
        if row["event"] != "frame" or time.monotonic()-self.last_flush >= .5:
            self.file.flush()
            self.last_flush = time.monotonic()
        if row["event"] in ("fresh_ready", "recording_closed"):
            os.fsync(self.file.fileno())


def watch_stdin(stream, stopped, reason):
    """Optional daemon input reader; a close line or EOF asks the receiver to end."""
    try:
        for line in stream:
            if line.strip().lower() == "close":
                reason.append("stdin_close")
                stopped.set()
                return
        reason.append("stdin_eof")
        stopped.set()
    except Exception:
        reason.append("stdin_error")
        stopped.set()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=600.)
    parser.add_argument("--stdin-close", action="store_true", help="Stop on a close line or stdin EOF")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 600:
        parser.error("--seconds must be within [2, 600]")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    stopped, reasons = threading.Event(), []
    def stop_signal(number, _):
        reasons.append("signal_%d" % number)
        stopped.set()
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    if args.stdin_close:
        threading.Thread(target=watch_stdin, args=(sys.stdin, stopped, reasons), daemon=True).start()
    with (args.output_dir/"frames.jsonl").open("xb") as output:
        sink = JsonlSink(output)
        report = collect_stream(args.seconds, sink,
            ready=lambda row: print(json.dumps(dict(row, output_dir=str(args.output_dir.resolve()))), flush=True),
            stop_reason=lambda: (reasons[0] if reasons else "operator_requested") if stopped.is_set() else None)
        sink(dict(event="recording_closed", **report))
        report.update(jsonl_rows=sink.rows, jsonl_sha256=sink.digest.hexdigest(),
                      frames_file=str((args.output_dir/"frames.jsonl").resolve()))
    with (args.output_dir/"summary.json").open("x") as summary:
        json.dump(report, summary, indent=2, allow_nan=False)
        summary.write("\n")
        summary.flush()
        os.fsync(summary.fileno())
    print(json.dumps(dict(event="closed", status=report["status"], close_reason=report["close_reason"],
                          trace_transport_clean=report["trace_transport_clean"],
                          feedback_frame_count=report["feedback_frame_count"],
                          output_dir=str(args.output_dir.resolve()))), flush=True)
    return 0 if report["trace_transport_clean"] else 2


if __name__ == "__main__":
    sys.exit(main())
