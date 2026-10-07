"""Receive-only streaming transport tests; real sockets/devices are forbidden."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import socket
import struct
import tempfile
import threading
import unittest
from unittest.mock import patch

import test_qualification_can_recording as fixtures

SPEC = importlib.util.spec_from_file_location(
    "raw_can_stream_test", Path(__file__).parents[1]/"scripts/record_raw_can_stream.py")
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


class AlteredSocket(fixtures.FakeSocket):
    def __init__(self, clock, mode):
        super().__init__(clock)
        self.mode = mode

    def recvmsg(self, size, ancillary_size):
        raw, ancillary, flags, address = super().recvmsg(size, ancillary_size)
        if self.mode == "missing_timestamp":
            ancillary = []
        if self.count == 50:
            if self.mode == "bad":
                raw = b"malformed"
            elif self.mode == "overflow":
                ancillary.append((socket.SOL_SOCKET, recorder.transport.SO_RXQ_OVFL, struct.pack("=I", 7)))
            elif self.mode == "gap":
                self.clock.value += .2
            elif self.mode == "backwards":
                ancillary = [(socket.SOL_SOCKET, recorder.transport.SO_TIMESTAMPNS,
                              recorder.transport.TIMESPEC.pack(1799999999, 0))]
            elif self.mode == "receive_error":
                raise OSError("synthetic receive failure")
        return raw, ancillary, flags, address


class RawCANStreamTests(unittest.TestCase):
    def setUp(self):
        blocker = patch.object(socket, "socket", side_effect=AssertionError("No real CAN/network"))
        blocker.start(); self.addCleanup(blocker.stop)

    def collect(self, receiver=None, seconds=2, stop=None):
        clock = receiver.clock if receiver else fixtures.FakeClock()
        receiver = receiver or fixtures.FakeSocket(clock)
        events, ready = [], []
        def on_ready(event):
            ready.append((receiver.count, event))
        result = recorder.collect_stream(seconds, events.append, ready=on_ready,
            stop_reason=stop or (lambda: None), socket_factory=lambda *args: receiver,
            clock=clock, binding_check=lambda: {"binding_verified": True})
        return result, events, ready, receiver

    def test_all_feedback_frames_are_streamed_without_subsampling_or_any_sender(self):
        result, events, ready, receiver = self.collect()
        frames = [e for e in events if e["event"] == "frame"]
        self.assertTrue(result["trace_transport_clean"])
        self.assertEqual(len(frames), receiver.count)
        self.assertEqual(result["received_count"], receiver.count)
        self.assertEqual(result["feedback_frame_count"]+result["control_frame_count"], receiver.count)
        self.assertGreater(result["feedback_frame_count"], 1800)
        self.assertNotIn("samples", result)
        self.assertNotIn("frames", result)
        self.assertEqual(len(ready), 1)
        self.assertGreaterEqual(ready[0][0], 14)
        self.assertFalse(ready[0][1]["robot_health_evaluated"])
        for frame in frames:
            self.assertEqual(frame["data_hex"], bytes([frame["received_index"] % 256]*8).hex())
            self.assertEqual(frame["timestamp_basis"], "kernel_socket_SO_TIMESTAMPNS_unix")
            self.assertTrue(frame["kernel_timestamp_ns"].isdigit())
            self.assertGreaterEqual(frame["host_received_at"], frame["timestamp"])
        self.assertTrue(receiver.closed)
        self.assertEqual(receiver.address, ("can1",))
        self.assertEqual(result["frames_sent_by_this_script"], 0)
        self.assertFalse(result["sdk_used"])

    def test_raw_kernel_nanoseconds_remain_exact_beyond_float_precision(self):
        ancillary = [(socket.SOL_SOCKET, recorder.transport.SO_TIMESTAMPNS,
                      recorder.transport.TIMESPEC.pack(1800000000, 123456789))]
        self.assertEqual(recorder._kernel_ns(ancillary), "1800000000123456789")

    def test_missing_timestamps_cannot_advertise_fresh_ready_or_clean(self):
        result, _, ready, _ = self.collect(AlteredSocket(fixtures.FakeClock(), "missing_timestamp"))
        self.assertFalse(result["fresh_ready"])
        self.assertFalse(result["trace_transport_clean"])
        self.assertGreater(result["kernel_timestamp_missing_count"], 0)
        self.assertEqual(ready, [])

    def test_bad_frames_overflow_time_regression_and_feedback_gaps_are_preserved(self):
        cases = (("bad", "bad_frame_count", "bad_frame"),
                 ("overflow", "socket_dropped_total", "socket_overflow"),
                 ("backwards", "timestamp_backwards_count", "timestamp_backwards"),
                 ("gap", "feedback_gap_count", "feedback_gap"))
        for mode, counter, event in cases:
            with self.subTest(mode=mode):
                result, events, _, receiver = self.collect(AlteredSocket(fixtures.FakeClock(), mode))
                self.assertFalse(result["trace_transport_clean"])
                self.assertGreater(result[counter], 0)
                self.assertTrue(any(e["event"] == event for e in events))
                self.assertTrue(receiver.closed)
                if mode == "bad":
                    row = next(e for e in events if e["event"] == event)
                    self.assertEqual(row["transport_hex"], b"malformed".hex())
                    self.assertIn("ancillary", row)

    def test_socket_options_must_work_and_receive_failure_is_not_retried(self):
        clock = fixtures.FakeClock(); receiver = fixtures.FakeSocket(clock)
        receiver.setsockopt = lambda *args: (_ for _ in ()).throw(OSError("no kernel option"))
        result, _, ready, _ = self.collect(receiver)
        self.assertFalse(result["trace_transport_clean"])
        self.assertEqual(result["received_count"], 0)
        self.assertEqual(ready, [])
        result, events, _, receiver = self.collect(AlteredSocket(fixtures.FakeClock(), "receive_error"))
        self.assertEqual(result["close_reason"], "recording_error")
        self.assertEqual(receiver.count, 50)
        self.assertEqual(sum(e["event"] == "recording_error" for e in events), 1)

    def test_ten_minute_bound_and_early_close_are_receive_only(self):
        clock = fixtures.FakeClock()
        result, _, ready, _ = self.collect(fixtures.FakeSocket(clock), seconds=600,
                                         stop=lambda: "stdin_close" if clock.value >= .2 else None)
        self.assertTrue(result["trace_transport_clean"])
        self.assertEqual(result["close_reason"], "stdin_close")
        self.assertLess(result["elapsed_s"], .25)
        self.assertEqual(len(ready), 1)
        for bad in (True, 1.99, 600.01, float("inf")):
            with self.subTest(seconds=bad), self.assertRaises(ValueError):
                recorder.collect_stream(bad, lambda row: None)

    def test_stdin_close_and_eof_are_explicit_stop_reasons(self):
        for contents, expected in (("ignored\nclose\nnot_read\n", "stdin_close"), ("", "stdin_eof")):
            stopped, reasons = threading.Event(), []
            recorder.watch_stdin(io.StringIO(contents), stopped, reasons)
            self.assertTrue(stopped.is_set())
            self.assertEqual(reasons, [expected])

    def test_cli_writes_exclusive_jsonl_hash_and_summary_with_fake_transport(self):
        real_collect = recorder.collect_stream
        clock = fixtures.FakeClock(); receiver = fixtures.FakeSocket(clock)
        def offline_collect(seconds, sink, **kwargs):
            return real_collect(seconds, sink, **kwargs, socket_factory=lambda *a: receiver,
                                clock=clock, binding_check=lambda: {"binding_verified": True})
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/"raw"
            with patch.object(recorder, "collect_stream", side_effect=offline_collect), \
                    patch.object(recorder.signal, "signal"), patch("sys.stdout", io.StringIO()):
                self.assertEqual(recorder.main(["--output-dir", str(output), "--seconds", "2"]), 0)
                with self.assertRaises(FileExistsError):
                    recorder.main(["--output-dir", str(output), "--seconds", "2"])
            content = (output/"frames.jsonl").read_bytes()
            summary = json.loads((output/"summary.json").read_text())
            self.assertEqual(summary["jsonl_sha256"], hashlib.sha256(content).hexdigest())
            rows = [json.loads(line) for line in content.splitlines()]
            self.assertEqual(len(rows), summary["jsonl_rows"])
            self.assertEqual(rows[-1]["event"], "recording_closed")
            self.assertEqual(sum(r["event"] == "fresh_ready" for r in rows), 1)


if __name__ == "__main__":
    unittest.main()
