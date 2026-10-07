"""Firmware query tests: real sockets prohibited, including SDK integration."""
import copy
import errno
import time
import unittest
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from robot_tools import arms, commissioning, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeRobot, TakeoverFixture


def firmware_bytes(version=b"S-V1.6-5"):
    raw = bytearray(88)
    for start, value in ((0, b"H-V1.2-1"), (16, b"10"), (32, b"ARM_MC"),
                         (60, version), (68, b"250409"), (76, b"15")):
        raw[start:start + len(value)] = value
    return bytes(raw)


def firmware_fields(raw):
    return {name: raw[start:end].decode("utf-8") for name, (start, end) in {
        "hardware_version": (0, 8), "motor_ratio_and_batch": (16, 18),
        "node_type": (32, 38), "software_version": (60, 68),
        "production_date": (68, 74), "node_number": (76, 78)}.items()}


class FirmwareRobot(FakeRobot):
    def __init__(self, side, clock):
        super().__init__(side)
        self.ctrl_mode, self.clock = 1, clock
        self.raw = firmware_bytes()
        self.reply_count = 11
        self.reply_timestamp_offset = 0
        self.parsed_override = None
        self.query_frame = None
        self.get_firmware_calls = []
        self._parser = NS(firmware_info=NS(msg=NS(clear=Mock())))
        self.comm.get_callback = lambda: self.callback

    def get_firmware(self, *, timeout, min_interval):
        self.get_firmware_calls.append((timeout, min_interval))
        frame = self.query_frame or self.can.Message(arbitration_id=0x4AF, is_extended_id=False, data=b"\x01")
        self._send_msg(frame)
        if self.duplicate:
            self._send_msg(frame)
        for i in range(self.reply_count):
            data = self.raw[i * 8:(i + 1) * 8] if i < 11 else b"extra123"
            self.callback(self.can.Message(arbitration_id=0x4AF, is_extended_id=False,
                                           timestamp=self.clock.time() + self.reply_timestamp_offset,
                                           data=data))
        self._parser.firmware_info.msg.clear()  # Real SDK clears the completed response cache.
        if self.reply_count < 11:
            return None
        return self.parsed_override or firmware_fields(self.raw)


class FirmwareTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots = {side: FirmwareRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.sdk.__file__ = PROFILE["sdk_path"] + "/pyAgxArm/__init__.py"
        from pyAgxArm import resolve_firmware_profile
        self.sdk.resolve_firmware_profile = resolve_firmware_profile
        self.stack.enter_context(patch.object(commissioning, "time", self.clock))

    def snapshot(self, robot, gripper):
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        state = healthy_arm(self.clock.time())
        state["arm_status"].update(ctrl_mode=robot.ctrl_mode, teach_status=0, mode_feedback=0)
        state["gripper"]["foc_status"]["driver_enable_status"] = False
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None):
        return commissioning.inspect_firmware(PROFILE, journal or (lambda event, data: self.events.append((event, data))))

    def test_two_exact_one_byte_queries_and_fresh_raw_survives_sdk_clear(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["firmware_queries_sent"], 2)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["mode_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertFalse(result["sdk_profile_changed"])
        for side, robot in self.robots.items():
            self.assertEqual(robot.get_firmware_calls, [(2.0, 0.0)])
            self.assertEqual(len(robot.sent), 1)
            self.assertEqual(robot.sent[0].arbitration_id, 0x4AF)
            self.assertEqual(robot.sent[0].dlc, 1)
            self.assertEqual(bytes(robot.sent[0].data), b"\x01")
            self.assertEqual(result["firmware"][side]["raw_response_hex"], firmware_bytes().hex())
            self.assertEqual(result["firmware"][side]["manufacturer_result"]["software_version"], "S-V1.6-5")
            self.assertEqual(result["firmware"][side]["suggested_sdk_profile"], "default")
            self.assertEqual(len(result["firmware"][side]["response_evidence"]["response_frames"]), 11)
            self.assertGreaterEqual(robot._parser.firmware_info.msg.clear.call_count, 2)

    def test_no_incomplete_reply_can_be_success_and_no_second_query(self):
        self.robots["left"].reply_count = 10
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["firmware_queries_sent"], 1)
        self.assertEqual(len(bytes.fromhex(result["firmware"]["left"]["raw_response_hex"])), 80)
        self.assertEqual(self.robots["right"].sent, [])

    def test_pre_window_frames_are_not_used_as_current_response(self):
        import can
        def connect():
            self.robots["left"].callback(can.Message(arbitration_id=0x4AF, is_extended_id=False,
                                                     timestamp=self.clock.time(), data=b"old_data"))
        self.robots["left"].connect = connect
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["pre_or_post_window_firmware_frames"]["left"], 1)
        self.assertNotIn("old_data", result["firmware"]["left"]["raw_response_text"])

    def test_old_receive_timestamps_cannot_confirm_firmware(self):
        self.robots["left"].reply_timestamp_offset = -1
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        left = result["firmware"]["left"]
        self.assertEqual(left["raw_response_hex"], "")
        self.assertEqual(len(left["response_evidence"]["ignored_stale_frames"]), 11)
        self.assertEqual(self.robots["right"].sent, [])

    def test_parsed_stale_version_cannot_override_actual_fresh_reply(self):
        self.robots["left"].parsed_override = firmware_fields(firmware_bytes(b"S-V1.8-8"))
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("does not match", result["errors"][0]["detail"])
        self.assertEqual(self.robots["right"].sent, [])

    def test_extra_reply_frame_aborts_without_silently_truncating(self):
        self.robots["left"].reply_count = 12
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertTrue(result["guard_violations"])
        self.assertEqual(len(result["firmware"]["left"]["response_evidence"]["rejected_frames"]), 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_duplicate_sdk_query_refused(self):
        self.robots["left"].duplicate = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["firmware_queries_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_sdk_swallowed_send_exception_is_not_query_success(self):
        self.robots["left"].send_error = OSError(errno.ENOBUFS, "query queue full")
        self.robots["left"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["firmware_queries_sent"], 0)
        self.assertEqual(result["transmission_counts_by_kind"]["left"]["firmware"]["attempted_frames"], 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_padded_old_sdk_query_frame_rejected(self):
        import can
        self.robots["left"].query_frame = can.Message(arbitration_id=0x4AF, is_extended_id=False,
                                                      data=b"\x01" + bytes(7))
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_actuator_frame_rejected(self):
        import can
        self.robots["left"].query_frame = can.Message(arbitration_id=0x151, is_extended_id=False,
                                                      data=bytes((1, 0, 1, 0, 0, 0, 0, 0)))
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_gripper_enable_change_after_query_blocks_next_query(self):
        def hook(robot, state):
            if self.robots["left"].sent and robot.side == "left":
                state["gripper"]["foc_status"]["driver_enable_status"] = True
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["firmware_queries_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_joint_drift_allows_only_queries_and_reports_not_stationary(self):
        def hook(robot, state):
            if self.robots["left"].sent:
                state["joints_rad"][0] += 0.004
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["firmware_queries_sent"], 2)
        self.assertFalse(result["stationary"])
        self.assertAlmostEqual(result["drift"]["left"]["joint_rad"], 0.004)
        self.assertIn("joint_rad", result["drift_exceeds_stationary_limits"]["left"])
        self.assertEqual(result["mode_commands_sent"], 0)
        self.assertEqual(result["actuator_commands_sent"], 0)
        for robot in self.robots.values():
            self.assertEqual([frame.arbitration_id for frame in robot.sent], [0x4AF])

    def test_initial_disabled_joint_or_wrong_mode_refused(self):
        self.robots["left"].ctrl_mode = 0
        self.assert_no_tx(self.run_tool())


class RealSDKFirmwareTests(unittest.TestCase):
    def test_real_sdk_query_encoder_parser_and_raw_capture_without_receive_deadlock(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, robots = Clock(), [], {}
        class FakeBus:
            def __init__(self, channel):
                self.channel, self.pending = channel, deque()
            def recv(self, timeout=None):
                time.sleep(0.001)
                return self.pending.popleft() if self.pending else None
            def send(self, frame, timeout=None):
                sent.append((self.channel, frame))
                raw = firmware_bytes(b"S-V1.8-8" if self.channel == "can0" else b"S-V1.6-5")
                for start in range(0, 88, 8):
                    self.pending.append(can.Message(arbitration_id=0x4AF, is_extended_id=False,
                                                    timestamp=clock.time(), data=raw[start:start + 8]))
            def shutdown(self):
                pass
        original_factory = sdk.AgxArmFactory.create_arm
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"
        def create_robot(config):
            robot = original_factory(config)
            robot.get_fps = lambda: 100.0  # Only unrelated arm traffic's FPS is mocked.
            robots[config["comm"]["can"]["channel"]] = robot
            return robot
        def snapshot(robot, gripper):
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=1, teach_status=0, mode_feedback=0, motion_status=0, arm_status=0, err_code=0))
            state["gripper"]["foc_status"]["driver_enable_status"] = False
            return state
        with patch("socket.socket", side_effect=AssertionError("Real sockets prohibited")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), patch.object(commissioning, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = commissioning.inspect_firmware(profile, lambda event, data: None)
        self.assertTrue(result["ok"], result)
        self.assertEqual(len(sent), 2)
        for channel, frame in sent:
            self.assertEqual(frame.arbitration_id, 0x4AF)
            self.assertEqual(frame.dlc, 1)
            self.assertEqual(bytes(frame.data), b"\x01")
            self.assertFalse(frame.is_extended_id)
        self.assertEqual(result["firmware"]["left"]["suggested_sdk_profile"], "v188")
        self.assertEqual(result["firmware"]["right"]["suggested_sdk_profile"], "default")
        self.assertEqual(result["firmware"]["right"]["raw_response_hex"], firmware_bytes().hex())
        self.assertEqual(robots["can0"]._parser.firmware_info.msg.data_seg, bytearray())


if __name__ == "__main__":
    unittest.main()
