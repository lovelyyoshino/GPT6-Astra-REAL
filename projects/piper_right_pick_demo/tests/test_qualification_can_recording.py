"""Passive qualification trace tests: fake sockets/clocks, never real CAN."""
import importlib.util
from pathlib import Path
import socket
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

SPEC = importlib.util.spec_from_file_location("qualification_recorder", Path(__file__).parents[1] / "scripts/record_qualification_can.py")
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


class FakeClock:
    value = 0.

    def time(self):
        return 1800000000. + self.value

    def monotonic(self):
        return self.value


class FakeSocket:
    def __init__(self, clock, identifiers=None):
        self.clock = clock
        self.identifiers = identifiers or sorted(recorder.FEEDBACK_IDS) + [0x151]
        self.count = 0
        self.options = []
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def setsockopt(self, *args):
        self.options.append(args)

    def bind(self, address):
        self.address = address

    def settimeout(self, timeout):
        self.timeout = timeout

    def recvmsg(self, size, ancillary_size):
        self.clock.value += min(.001, self.timeout)
        identifier = self.identifiers[self.count % len(self.identifiers)]
        self.count += 1
        stamp = self.clock.time() - .00001
        seconds = int(stamp)
        nanos = int((stamp - seconds) * 1e9)
        ancillary = [(socket.SOL_SOCKET, recorder.SO_TIMESTAMPNS, recorder.TIMESPEC.pack(seconds, nanos))]
        return recorder.CAN_FRAME.pack(identifier, 8, bytes([self.count % 256]) * 8), ancillary, 0, ("can1",)

    def send(self, *args):
        raise AssertionError("No sender is permitted")

    sendto = send
    sendmsg = send


class QualificationCANTests(unittest.TestCase):
    def test_transport_parser_preserves_bytes_and_kernel_time(self):
        payload = bytes.fromhex("0100020000000000")
        raw = recorder.CAN_FRAME.pack(0x2A1, 8, payload)
        frame = recorder.parse_received(raw, [(socket.SOL_SOCKET, recorder.SO_TIMESTAMPNS,
                    recorder.TIMESPEC.pack(100, 250000000))], socket.MSG_DONTROUTE, 100.3)
        self.assertEqual(frame["data_hex"], payload.hex())
        self.assertEqual(frame["timestamp"], 100.25)
        self.assertEqual(frame["timestamp_basis"], "kernel_socket_SO_TIMESTAMPNS_unix")
        self.assertEqual(frame["origin"], "local_socket_loopback")
        fallback = recorder.parse_received(raw, [], 0, 123.5)
        self.assertEqual(fallback["timestamp"], 123.5)
        self.assertEqual(fallback["timestamp_basis"], "host_recvmsg_return_unix")

    def test_bad_transport_and_future_kernel_times_are_rejected(self):
        for raw, ancillary, flags in (
            (b"short", [], 0),
            (recorder.CAN_FRAME.pack(0x2A1, 7, bytes(8)), [], 0),
            (recorder.CAN_FRAME.pack(0x2A1 | socket.CAN_ERR_FLAG, 8, bytes(8)), [], 0),
            (recorder.CAN_FRAME.pack(0x2A1, 8, bytes(8)), [], socket.MSG_CTRUNC),
            (recorder.CAN_FRAME.pack(0x2A1, 8, bytes(8)), [(socket.SOL_SOCKET, recorder.SO_TIMESTAMPNS, recorder.TIMESPEC.pack(200, 0))], 0),
        ):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                recorder.parse_received(raw, ancillary, flags, 100.)

    def test_never_fills_missing_or_stale_frames(self):
        latest = {identifier: {"id": identifier, "timestamp": 100., "data_hex": "00" * 8}
                  for identifier in recorder.FEEDBACK_IDS}
        self.assertEqual(len(recorder.complete_sample(latest, 100.05)[0]["frames"]), 14)
        self.assertEqual(recorder.complete_sample(latest, 100.2), (None, "stale_feedback"))
        latest.pop(0x2A8)
        self.assertEqual(recorder.complete_sample(latest, 100.05), (None, "missing_feedback"))

    def test_fixed_usb_can_binding_and_down_interface_rejection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            interface = root / "can1"
            interface.mkdir()
            target = root / "usb" / "1-6.3:1.0"
            target.mkdir(parents=True)
            (interface / "device").symlink_to(target)
            (interface / "type").write_text("280")
            (interface / "flags").write_text("0x1")
            self.assertTrue(recorder.verify_binding(root)["binding_verified"])
            (interface / "flags").write_text("0x0")
            with self.assertRaisesRegex(RuntimeError, "DOWN"):
                recorder.verify_binding(root)
            (interface / "device").unlink()
            (interface / "device").symlink_to(root)
            with self.assertRaisesRegex(RuntimeError, "binding mismatch"):
                recorder.verify_binding(root)

    def test_two_seconds_real_timestamp_samples_without_any_sender(self):
        clock = FakeClock()
        receiver = FakeSocket(clock)
        ready = Mock(side_effect=lambda event: self.assertEqual(receiver.count, 0))
        with patch.object(recorder.socket, "socket", side_effect=AssertionError("Actual SocketCAN forbidden")):
            result = recorder.collect(2, ready=ready, socket_factory=lambda *args: receiver,
                                      clock=clock, binding_check=lambda: {"binding_verified": True})
        ready.assert_called_once()
        self.assertEqual(receiver.address, ("can1",))
        self.assertTrue(receiver.closed)
        self.assertTrue(result["trace_transport_clean"])
        self.assertGreaterEqual(result["sample_count"], 99)
        self.assertLessEqual(result["sample_count"], 101)
        self.assertGreater(len(result["control_frames"]), 10)
        self.assertEqual(result["frames_sent_by_this_script"], 0)
        self.assertFalse(result["qualified"])
        for sample in result["samples"]:
            self.assertEqual({frame["id"] for frame in sample["frames"]}, recorder.FEEDBACK_IDS)
            self.assertEqual(len({frame["timestamp"] for frame in sample["frames"]}), 14)
            self.assertTrue(all(0 <= sample["sampled_at"] - frame["timestamp"] <= .1 for frame in sample["frames"]))

    def test_missing_stream_returns_no_complete_samples_and_duration_is_bounded(self):
        clock = FakeClock()
        receiver = FakeSocket(clock, identifiers=[0x2A1])
        result = recorder.collect(2, socket_factory=lambda *args: receiver, clock=clock,
                                  binding_check=lambda: {"binding_verified": True})
        self.assertEqual(result["samples"], [])
        self.assertEqual(result["status"], "no_complete_samples")
        self.assertEqual(len(result["missing_feedback_ids"]), 13)
        self.assertFalse(result["trace_transport_clean"])
        for invalid in (True, 1.9, 121, float("nan")):
            with self.assertRaises(ValueError):
                recorder.collect(invalid)


if __name__ == "__main__":
    unittest.main()
