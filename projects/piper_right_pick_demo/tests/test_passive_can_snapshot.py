"""Transport/aggregation tests; no hardware or SDK CAN interface is initialized."""
import importlib.util
import io
import json
from pathlib import Path
import socket
import struct
import sys
import types
import unittest
from unittest import mock


_PATH = Path(__file__).resolve().parents[1] / "scripts" / "passive_can_snapshot.py"
_SPEC = importlib.util.spec_from_file_location("passive_can_snapshot_test_module", _PATH)
snapshot = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(snapshot)

_POSE_TYPES = {
    0x2A5: ("PiperMsgJointFeedBack_12", "arm_joint_feedback", ("joint_1", "joint_2")),
    0x2A6: ("PiperMsgJointFeedBack_34", "arm_joint_feedback", ("joint_3", "joint_4")),
    0x2A7: ("PiperMsgJointFeedBack_56", "arm_joint_feedback", ("joint_5", "joint_6")),
    0x2A2: ("PiperMsgEndPoseFeedback_1", "arm_end_pose", ("X_axis", "Y_axis")),
    0x2A3: ("PiperMsgEndPoseFeedback_2", "arm_end_pose", ("Z_axis", "RX_axis")),
    0x2A4: ("PiperMsgEndPoseFeedback_3", "arm_end_pose", ("RY_axis", "RZ_axis")),
}
_COMMAND_TYPES = {
    0x155: ("PiperMsgJointCtrl_12", "arm_joint_ctrl", ("joint_1", "joint_2")),
    0x156: ("PiperMsgJointCtrl_34", "arm_joint_ctrl", ("joint_3", "joint_4")),
    0x157: ("PiperMsgJointCtrl_56", "arm_joint_ctrl", ("joint_5", "joint_6")),
}
_ALL_NAMES = tuple("joint_%d" % n for n in range(1, 7)) + (
    "X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")


def can_frame(can_id, first=0, second=0, dlc=8):
    return struct.pack("=IB3x8s", can_id, dlc, struct.pack(">ii", first, second))


def complete_events():
    return [(index * .001, can_frame(can_id, index * 10 + 1, index * 10 + 2))
            for index, can_id in enumerate(_POSE_TYPES, 1)]


class FakeClock:
    def __init__(self):
        self.elapsed = 0.0
        self.wall_offset = 0.0

    def monotonic(self):
        return 1000.0 + self.elapsed

    def time(self):
        return 1700000000.0 + self.elapsed + self.wall_offset


class ReceiveOnlySocket:
    """Deliberately has no send/sendto/sendmsg method."""
    def __init__(self, clock, events):
        self.clock = clock
        self.events = list(events)
        self.bound_to = None
        self.closed = False
        self.timeout = .1

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def bind(self, address):
        self.bound_to = address

    def settimeout(self, seconds):
        self.timeout = seconds

    def recvmsg(self, size):
        if self.events and self.events[0][0] <= self.clock.elapsed + self.timeout:
            event = self.events.pop(0)
            self.clock.elapsed = event[0]
            if len(event) > 2:
                self.clock.wall_offset = event[2]
            flags = event[3] if len(event) > 3 else 0
            return event[1], [], flags, self.bound_to
        self.clock.elapsed += self.timeout
        raise socket.timeout()


class FakeVendorParser:
    """Models a decoder returning fresh objects with untouched zero defaults."""
    def DecodeMessage(self, frame, decoded):
        if frame.arbitration_id == 0x159:
            decoded.type_ = types.SimpleNamespace(name="PiperMsgGripperCtrl")
            decoded.arm_gripper_ctrl = types.SimpleNamespace(**dict(zip(
                ("grippers_angle", "grippers_effort", "status_code", "set_zero"),
                struct.unpack(">ihBB", frame.data))))
            return True
        spec = _POSE_TYPES.get(frame.arbitration_id) or _COMMAND_TYPES.get(frame.arbitration_id)
        if spec is None:
            return False
        message_type, attr, names = spec
        decoded.type_ = types.SimpleNamespace(name=message_type)
        fields = types.SimpleNamespace(**{name: 0 for name in _ALL_NAMES})
        for name, value in zip(names, struct.unpack(">ii", frame.data)):
            setattr(fields, name, value)
        setattr(decoded, attr, fields)
        return True


class PassiveCanSnapshotTests(unittest.TestCase):
    def run_capture(self, events, include_trace=True, seconds=.2):
        clock = FakeClock()
        receiver = ReceiveOnlySocket(clock, events)
        factory = mock.Mock(return_value=receiver)
        fake_socket = types.SimpleNamespace(**{
            name: getattr(socket, name) for name in (
                "AF_CAN", "SOCK_RAW", "CAN_RAW", "CAN_EFF_MASK", "CAN_SFF_MASK",
                "CAN_ERR_FLAG", "CAN_RTR_FLAG", "CAN_EFF_FLAG", "timeout",
                "MSG_DONTROUTE", "MSG_CONFIRM", "MSG_TRUNC")})
        fake_socket.socket = factory
        # Only the decoder and plain message containers exist. Any accidental
        # attempt to import an SDK control interface fails this test.
        fake_modules = {
            "can": types.SimpleNamespace(Message=lambda **kw: types.SimpleNamespace(**kw)),
            "piper_sdk.protocol.protocol_v2.piper_protocol_v2": types.SimpleNamespace(
                C_PiperParserV2=FakeVendorParser),
            "piper_sdk.piper_msgs.msg_v2": types.SimpleNamespace(PiperMessage=types.SimpleNamespace),
        }
        with mock.patch.dict(sys.modules, fake_modules), \
                mock.patch.object(snapshot.importlib.metadata, "version", return_value="0.6.2"), \
                mock.patch.object(snapshot, "socket", fake_socket), \
                mock.patch.object(snapshot, "time", clock):
            if include_trace is None:
                result = snapshot.collect("can2", seconds)
            else:
                result = snapshot.collect("can2", seconds, include_pose_trace=include_trace)
        factory.assert_called_once_with(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.assertEqual(receiver.bound_to, ("can2",))
        self.assertTrue(receiver.closed)
        self.assertFalse(hasattr(receiver, "send"))
        self.assertEqual(result["frames_sent_by_this_script"], 0)
        return result

    def test_receive_only_complete_fields_and_real_pair_timestamps(self):
        output = self.run_capture(complete_events())
        self.assertEqual(output["frames_received"], 6)
        self.assertEqual(len(output["pose_trace"]), 1)
        sample = output["pose_trace"][0]
        self.assertEqual(sample["joints_raw"], {
            "joint_1": 11, "joint_2": 12, "joint_3": 21,
            "joint_4": 22, "joint_5": 31, "joint_6": 32})
        self.assertEqual(sample["end_pose_raw"], {
            "X_axis": 41, "Y_axis": 42, "Z_axis": 51,
            "RX_axis": 52, "RY_axis": 61, "RZ_axis": 62})
        self.assertEqual(set(sample["field_received_at_s"]), set(_ALL_NAMES))
        self.assertEqual(set(sample["field_received_monotonic_s"]), set(_ALL_NAMES))
        self.assertAlmostEqual(sample["received_monotonic_s"], 1000.006)
        self.assertAlmostEqual(sample["frame_skew_s"], .005)
        self.assertEqual(sample["received_at_s"], 1700000000.006)
        for index, (_, _, names) in enumerate(_POSE_TYPES.values(), 1):
            for name in names:
                self.assertAlmostEqual(sample["field_received_monotonic_s"][name], 1000 + index * .001)
                self.assertEqual(sample["field_received_at_s"][name], 1700000000 + index * .001)

    def test_missing_pair_never_uses_vendor_default_zeros(self):
        events = complete_events()[:-1]
        events.extend((.01 + i * .01, can_frame(0x2A5, i, -i)) for i in range(8))
        result = self.run_capture(events)
        self.assertEqual(result["pose_trace"], [])
        self.assertIn("PiperMsgEndPoseFeedback_3", result["missing_feedback_types"])
        self.assertEqual(set(result["feedback"]["PiperMsgJointFeedBack_12"]["fields"]),
                         {"joint_1", "joint_2"})

    def test_throttled_sample_copies_latest_real_data_and_exposes_stale_fields(self):
        events = complete_events() + [
            (.007, can_frame(0x2A5, 111, 112)),
            (.008, can_frame(0x2A6, 211, 212)),
            (.020, can_frame(0x2A5, 311, 312)),
        ]
        trace = self.run_capture(events)["pose_trace"]
        self.assertEqual(len(trace), 2)
        self.assertEqual(trace[0]["joints_raw"]["joint_1"], 11)
        self.assertEqual(trace[1]["joints_raw"]["joint_1"], 311)
        self.assertEqual(trace[1]["joints_raw"]["joint_3"], 211)
        self.assertEqual(trace[1]["joints_raw"]["joint_5"], 31)
        self.assertAlmostEqual(trace[1]["field_received_monotonic_s"]["joint_3"], 1000.008)
        self.assertAlmostEqual(trace[1]["frame_skew_s"], .017)

    def test_dense_frames_cannot_exceed_100hz_and_clock_jump_does_not_hide_skew(self):
        events = complete_events()
        events += [(.006 + index * .0005, can_frame(0x2A5, index, -index), -25.0)
                   for index in range(1, 300)]
        trace = self.run_capture(events)["pose_trace"]
        self.assertGreater(len(trace), 10)
        self.assertLessEqual(len(trace), 16)
        for previous, current in zip(trace, trace[1:]):
            self.assertGreaterEqual(current["received_monotonic_s"] - previous["received_monotonic_s"], .01)
        self.assertLess(trace[1]["received_at_s"], trace[0]["received_at_s"])
        self.assertGreater(trace[-1]["frame_skew_s"], .1)
        self.assertEqual(trace[-1]["field_received_at_s"]["Z_axis"], 1700000000.005)

    def test_malformed_remote_error_and_extended_frames_do_not_fill_fields(self):
        events = [(0.001, b"bad"),
                  (.002, can_frame(0x2A5 | socket.CAN_RTR_FLAG)),
                  (.003, can_frame(0x2A5 | socket.CAN_ERR_FLAG)),
                  (.004, can_frame(0x2A5 | socket.CAN_EFF_FLAG)),
                  (.005, can_frame(0x2A5, dlc=4))]
        output = self.run_capture(events)
        self.assertEqual(output["pose_trace"], [])
        self.assertEqual(output["feedback"], {})
        self.assertEqual(output["malformed_frames"], 1)

    def test_default_call_stays_snapshot_only(self):
        result = self.run_capture(complete_events(), include_trace=None)
        self.assertNotIn("pose_trace", result)
        self.assertEqual(result["feedback"]["PiperMsgJointFeedBack_12"]["fields"],
                         {"joint_1": 11, "joint_2": 12})

    def test_collect_duration_is_bounded_before_hardware_or_imports(self):
        for invalid in (0, .199, 15.001, float("inf"), float("nan"), True, "3"):
            with self.subTest(invalid=invalid), \
                    mock.patch.object(snapshot.socket, "socket") as factory, \
                    self.assertRaises(ValueError):
                try:
                    snapshot.collect("can2", invalid, include_pose_trace=True)
                finally:
                    factory.assert_not_called()

    def test_packets_at_or_after_deadline_are_not_sampled(self):
        result = self.run_capture(complete_events() + [(.2, can_frame(0x2A5, 999, 999))])
        self.assertEqual(result["frames_received"], 6)
        self.assertEqual(len(result["pose_trace"]), 1)

    def test_cli_enables_trace_without_starting_hardware_in_test(self):
        result = {"frames_received": 6, "pose_trace": []}
        output = io.StringIO()
        with mock.patch.object(sys, "argv", [str(_PATH), "--channel", "can2", "--seconds", "3", "--pose-trace"]), \
                mock.patch.object(snapshot, "collect", return_value=result) as collect, \
                mock.patch.object(sys, "stdout", output):
            self.assertEqual(snapshot.main(), 0)
        collect.assert_called_once_with("can2", 3.0, include_pose_trace=True)
        self.assertEqual(json.loads(output.getvalue()), result)

    def test_commands_preserve_nonlocal_latest_when_local_loopback_arrives(self):
        events = [(.010, can_frame(0x155, 111, -112)),
                  (.015, can_frame(0x155, 211, -212), 0, socket.MSG_DONTROUTE),
                  (.020, can_frame(0x155, 311, -312)),
                  (.030, can_frame(0x155, 999, 999), 0, socket.MSG_DONTROUTE | socket.MSG_CONFIRM)]
        output = self.run_capture(events)
        item = output["command_feedback"]["PiperMsgJointCtrl_12"]
        self.assertEqual(item["origin"], "local")
        self.assertEqual(item["observed_count"], 4)
        self.assertEqual(item["origin_counts"], {"local": 2, "nonlocal": 2})
        self.assertEqual(output["frame_origin_counts"], {"local": 2, "nonlocal": 2})
        external = item["latest_by_origin"]["nonlocal"]
        self.assertEqual(external["origin"], "nonlocal")
        self.assertEqual(external["observed_count"], 2)
        self.assertEqual(external["fields"], {"joint_1": 311, "joint_2": -312})
        self.assertEqual(external["received_at_s"], 1700000000.020)
        self.assertAlmostEqual(external["received_monotonic_s"], 1000.020)
        self.assertAlmostEqual(external["age_s_at_finish"], output["finished_monotonic_s"] - 1000.020)
        self.assertEqual(item["latest_by_origin"]["local"]["fields"], {"joint_1": 999, "joint_2": 999})
        self.assertEqual(output["feedback"], {})
        self.assertEqual(output["pose_trace"], [])

    def test_command_ranges_accumulate_per_origin_without_local_contamination(self):
        events = [(.010, can_frame(0x155, 100, 200)),
                  (.020, can_frame(0x155, -999, -888), 0, socket.MSG_DONTROUTE),
                  (.030, can_frame(0x155, 90, 220)),
                  (.040, can_frame(0x155, 999, 888), 0, socket.MSG_DONTROUTE),
                  (.050, can_frame(0x155, 105, 190))]
        item = self.run_capture(events)["command_feedback"]["PiperMsgJointCtrl_12"]
        external = item["latest_by_origin"]["nonlocal"]
        local = item["latest_by_origin"]["local"]
        self.assertEqual(external["field_min"], {"joint_1": 90, "joint_2": 190})
        self.assertEqual(external["field_max"], {"joint_1": 105, "joint_2": 220})
        self.assertEqual(external["fields"], {"joint_1": 105, "joint_2": 190})
        self.assertEqual(external["observed_count"], 3)
        self.assertEqual(local["field_min"], {"joint_1": -999, "joint_2": -888})
        self.assertEqual(local["field_max"], {"joint_1": 999, "joint_2": 888})
        self.assertEqual(local["observed_count"], 2)

    def test_joint_control_valid_pairs_and_gripper_units_are_separate_from_feedback(self):
        events = [(.001 * n, can_frame(can_id, n * 1000, -n * 1000))
                  for n, can_id in enumerate(_COMMAND_TYPES, 1)]
        gripper_payload = struct.pack(">ihBB", 30000, 750, 1, 0)
        events.append((.010, struct.pack("=IB3x8s", 0x159, 8, gripper_payload)))
        output = self.run_capture(events)
        commands = output["command_feedback"]
        self.assertEqual(len(commands), 4)
        for n, (_, (name, attr, fields)) in enumerate(_COMMAND_TYPES.items(), 1):
            self.assertEqual(commands[name]["fields"], dict(zip(fields, (n * 1000, -n * 1000))))
            self.assertEqual(commands[name]["units"], "0.001 degree")
        gripper = commands["PiperMsgGripperCtrl"]
        self.assertEqual(gripper["fields"], {"grippers_angle": 30000, "grippers_effort": 750,
                                              "status_code": 1, "set_zero": 0})
        self.assertEqual(gripper["units"]["grippers_angle"], "0.001 mm")
        self.assertEqual(output["feedback"], {})
        self.assertEqual(output["pose_trace"], [])
        self.assertIsNone(output["sdk_parser_module_path"])
        self.assertIsNone(output["sdk_messages_module_path"])

    def test_command_age_uses_monotonic_clock_despite_wall_clock_change(self):
        output = self.run_capture([(.010, can_frame(0x155, 1, 2)),
                                   (.030, can_frame(0x156, 3, 4), -30.0)])
        item = output["command_feedback"]["PiperMsgJointCtrl_12"]
        self.assertGreater(item["age_s_at_finish"], .18)
        self.assertLess(item["age_s_at_finish"], .20)
        self.assertLess(output["finished_at_s"], item["received_at_s"])

    def test_actual_feedback_records_origin_and_monotonic_age(self):
        output = self.run_capture([(.010, can_frame(0x2A5, 1, 2)),
                                   (.030, can_frame(0x2A6, 3, 4), -30.0, socket.MSG_DONTROUTE)])
        external = output["feedback"]["PiperMsgJointFeedBack_12"]
        local = output["feedback"]["PiperMsgJointFeedBack_34"]
        self.assertEqual(external["origin"], "nonlocal")
        self.assertEqual(local["origin"], "local")
        self.assertGreater(external["age_s_at_finish"], .18)
        self.assertLess(external["age_s_at_finish"], .20)
        self.assertLess(output["finished_at_s"], external["received_at_s"])

    def test_truncated_recvmsg_does_not_decode_even_if_buffer_has_frame_size(self):
        output = self.run_capture([(.010, can_frame(0x155, 1, 2), 0, socket.MSG_TRUNC)])
        self.assertEqual(output["command_feedback"], {})
        self.assertEqual(output["malformed_frames"], 1)


if __name__ == "__main__":
    unittest.main()
