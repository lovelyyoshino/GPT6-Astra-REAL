"""Joint limit diagnostics: no real sockets, including manufacturer SDK tests."""
import copy
import errno
import math
import time
import unittest
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from robot_tools import arms, commissioning, joint_limits, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeRobot, TakeoverFixture


def reply_bytes(joint, minimum=-1800, maximum=1800, speed=150):
    return (bytes((joint,)) + maximum.to_bytes(2, "big", signed=True)
            + minimum.to_bytes(2, "big", signed=True) + speed.to_bytes(2, "big") + b"\0")


def parsed_reply(raw, timestamp):
    return NS(timestamp=timestamp, msg=NS(joint_index=raw[0],
              max_angle_limit=int.from_bytes(raw[1:3], "big", signed=True) * 0.1 * math.pi / 180,
              min_angle_limit=int.from_bytes(raw[3:5], "big", signed=True) * 0.1 * math.pi / 180,
              max_joint_spd=int.from_bytes(raw[5:7], "big") * 0.01))


class LimitsRobot(FakeRobot):
    def __init__(self, side, clock):
        super().__init__(side)
        self.ctrl_mode, self.clock = 1, clock
        self.calls = []
        self.no_reply_joint = None
        self.reply_transform = lambda frame: frame
        self.parsed_transform = lambda parsed: parsed
        self.extra_reply = False
        self.raw = reply_bytes
        self._parser = NS(motor_angle_limit_max_spd=NS(msg=NS(joints=[NS(clear=Mock()) for _ in range(6)])))
        self.comm.get_callback = lambda: self.callback

    def get_joint_angle_vel_limits(self, *, joint_index, timeout, min_interval):
        self.calls.append((joint_index, timeout, min_interval))
        frame = self.can.Message(arbitration_id=0x472, is_extended_id=False,
                                 data=bytes((joint_index, 1, 0, 0, 0, 0, 0, 0)))
        self._send_msg(self.frame_transform(frame))
        if self.duplicate:
            self._send_msg(frame)
        if self.no_reply_joint == joint_index:
            return None
        raw = self.raw(joint_index)
        timestamp = self.clock.time()
        reply = self.reply_transform(self.can.Message(arbitration_id=0x473, is_extended_id=False,
                                                       timestamp=timestamp, data=raw))
        self.callback(reply)
        if self.extra_reply:
            self.callback(reply)
        self._parser.motor_angle_limit_max_spd.msg.joints[joint_index - 1].clear()
        return self.parsed_transform(parsed_reply(raw, timestamp))


class JointLimitsTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots = {side: LimitsRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.sdk.__file__ = PROFILE["sdk_path"] + "/pyAgxArm/__init__.py"
        self.stack.enter_context(patch.object(joint_limits, "time", self.clock))

    def snapshot(self, robot, gripper):
        state = super().snapshot(robot, gripper)
        state["gripper"]["foc_status"]["driver_enable_status"] = False
        return state

    def run_tool(self, journal=None):
        return joint_limits.inspect_joint_limits(
            PROFILE, journal or (lambda event, data: self.events.append((event, data))))

    def test_twelve_fixed_queries_and_signed_limit_evidence_without_motion(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["joint_limit_queries_sent"], 12)
        self.assertEqual(result["hardware_commands_sent"], 12)
        for key in ("mode_commands_sent", "target_commands_sent", "enable_commands_sent", "stop_commands_sent"):
            self.assertEqual(result[key], 0)
        for key in ("motion_gate_unlocked", "controller_limits_changed", "sdk_joint_limits_changed",
                    "limits_are_motion_permission"):
            self.assertFalse(result[key])
        for side, robot in self.robots.items():
            self.assertEqual(robot.calls, [(i, 1.0, 0.0) for i in range(1, 7)])
            self.assertEqual(len(robot.sent), 6)
            for joint, frame in enumerate(robot.sent, 1):
                self.assertEqual(frame.arbitration_id, 0x472)
                self.assertEqual(frame.dlc, 8)
                self.assertEqual(bytes(frame.data), bytes((joint, 1, 0, 0, 0, 0, 0, 0)))
                entry = result["joint_limits"][side][str(joint)]
                self.assertEqual(entry["status"], "confirmed")
                self.assertEqual(entry["raw_min_angle_tenth_deg"], -1800)
                self.assertAlmostEqual(entry["manufacturer_result"]["min_angle_limit"], -math.pi)
                self.assertEqual(entry["raw_response_hex"], reply_bytes(joint).hex())
                self.assertEqual(robot._parser.motor_angle_limit_max_spd.msg.joints[joint - 1].clear.call_count, 2)
                self.assertFalse(entry["motion_permission"])

    def test_first_missing_response_ends_remaining_queries_without_retry(self):
        self.robots["left"].no_reply_joint = 4
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["joint_limit_queries_sent"], 4)
        self.assertEqual(len(self.robots["left"].calls), 4)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertEqual(result["joint_limits"]["left"]["4"]["raw_response_hex"], "")

    def test_pre_window_reply_is_not_passed_to_parser(self):
        import can
        self.robots["left"].connect = lambda: self.robots["left"].callback(can.Message(
            arbitration_id=0x473, is_extended_id=False, timestamp=self.clock.time(), data=reply_bytes(1)))
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["pre_or_post_window_limit_frames"]["left"], 1)

    def test_stale_or_future_response_never_confirms(self):
        for offset in (-1, 1):
            with self.subTest(offset=offset):
                self.setUp_fresh_robots()
                def transform(frame):
                    frame.timestamp += offset
                    return frame
                self.robots["left"].reply_transform = transform
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(result["joint_limit_queries_sent"], 1)
                evidence = result["joint_limits"]["left"]["1"]["response_evidence"]
                self.assertEqual(len(evidence["ignored_stale_frames"]), 1)
                self.assertEqual(evidence["response_frames"], [])

    def setUp_fresh_robots(self):
        self.robots = {side: LimitsRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def test_wrong_joint_or_duplicate_or_malformed_response_aborts(self):
        for condition in ("index", "duplicate", "short", "extended", "not_rx"):
            with self.subTest(condition=condition):
                self.setUp_fresh_robots()
                def transform(frame):
                    if condition == "index":
                        frame.data[0] = 2
                    elif condition == "short":
                        frame.data = frame.data[:-1]
                        frame.dlc = 7
                    elif condition == "extended":
                        frame.is_extended_id = True
                    elif condition == "not_rx":
                        frame.is_rx = False
                    return frame
                self.robots["left"].reply_transform = transform
                self.robots["left"].extra_reply = condition == "duplicate"
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertTrue(result["guard_violations"])
                self.assertEqual(result["joint_limit_queries_sent"], 1)
                self.assertEqual(self.robots["right"].sent, [])

    def test_mismatched_manufacturer_parse_or_timestamp_cannot_confirm(self):
        for field, value in (("min_angle_limit", 0.0), ("joint_index", 2),
                             ("max_joint_spd", 0.15), ("max_joint_spd", float("nan")),
                             ("timestamp", 0)):
            with self.subTest(field=field):
                self.setUp_fresh_robots()
                def transform(parsed):
                    setattr(parsed if field == "timestamp" else parsed.msg, field, value)
                    return parsed
                self.robots["left"].parsed_transform = transform
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(result["joint_limit_queries_sent"], 1)
                self.assertIn("do not match", result["errors"][0]["detail"])

    def test_inverted_limits_refused_but_observed_outside_limits_is_diagnostic(self):
        self.robots["left"].raw = lambda joint: reply_bytes(joint, minimum=100, maximum=-100)
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.setUp_fresh_robots()
        self.robots["left"].raw = lambda joint: reply_bytes(joint, minimum=100, maximum=200)
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["joint_limits"]["left"]["1"]["observed_joint_within_reported_limits"])
        self.assertFalse(result["motion_gate_unlocked"])

    def test_bad_query_or_actuator_write_or_duplicate_is_guarded(self):
        for condition in ("set_limit", "set_mode", "wrong_search", "wrong_joint", "duplicate"):
            with self.subTest(condition=condition):
                self.setUp_fresh_robots()
                def transform(frame):
                    if condition == "set_limit":
                        frame.arbitration_id = 0x474
                    elif condition == "set_mode":
                        frame.arbitration_id = 0x151
                    elif condition == "wrong_search":
                        frame.data[1] = 2
                    elif condition == "wrong_joint":
                        frame.data[0] = 2
                    return frame
                self.robots["left"].frame_transform = transform
                self.robots["left"].duplicate = condition == "duplicate"
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(result["joint_limit_queries_sent"], int(condition == "duplicate"))
                self.assertEqual(self.robots["right"].sent, [])

    def test_sdk_swallowed_send_error_does_not_retry(self):
        self.robots["left"].send_error = OSError(errno.ENOBUFS, "queue full")
        self.robots["left"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["joint_limit_queries_sent"], 0)
        self.assertEqual(len(self.robots["left"].calls), 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_drift_reports_without_rejecting_query_or_authorizing_motion(self):
        def hook(robot, state):
            if self.robots["left"].sent:
                state["joints_rad"][3] += 0.006
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["stationary"])
        self.assertAlmostEqual(result["drift"]["left"]["joint_rad"], 0.006)
        self.assertEqual(result["joint_limit_queries_sent"], 12)
        self.assertFalse(result["motion_gate_unlocked"])

    def test_post_query_fault_stops_without_remaining_queries(self):
        def hook(robot, state):
            if self.robots["left"].sent:
                state["drivers"]["1"]["foc_status"]["driver_error_status"] = True
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["joint_limit_queries_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_journal_intent_failure_prevents_send(self):
        def journal(event, data):
            if event == "joint_limits_query_intent":
                raise OSError("disk full")
        self.assert_no_tx(self.run_tool(journal))


class RealSDKJointLimitsTests(unittest.TestCase):
    def test_real_sdk_encoder_parser_signed_limits_and_cleared_cache(self):
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
                self.pending.append(can.Message(arbitration_id=0x473, is_extended_id=False,
                    timestamp=clock.time(), data=reply_bytes(frame.data[0], minimum=-215, maximum=1925)))
            def shutdown(self):
                pass
        original_factory = sdk.AgxArmFactory.create_arm
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"
        def create_robot(config):
            robot = original_factory(config)
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
             patch.object(takeover, "time", clock), patch.object(joint_limits, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = joint_limits.inspect_joint_limits(profile, lambda event, data: None)
        self.assertTrue(result["ok"], result)
        self.assertEqual(len(sent), 12)
        self.assertEqual([(channel, frame.data[0]) for channel, frame in sent],
                         [(channel, i) for channel in ("can0", "can2") for i in range(1, 7)])
        for channel, frame in sent:
            self.assertEqual(frame.arbitration_id, 0x472)
            self.assertEqual(frame.dlc, 8)
            self.assertEqual(bytes(frame.data)[1:], b"\1" + bytes(6))
            self.assertFalse(frame.is_extended_id)
        for side, channel in (("left", "can0"), ("right", "can2")):
            for joint in range(1, 7):
                entry = result["joint_limits"][side][str(joint)]
                self.assertAlmostEqual(entry["manufacturer_result"]["min_angle_limit"], math.radians(-21.5))
                self.assertAlmostEqual(entry["manufacturer_result"]["max_angle_limit"], math.radians(192.5))
                self.assertAlmostEqual(entry["manufacturer_result"]["max_joint_spd"], 1.5)
                self.assertIsNone(robots[channel]._parser.motor_angle_limit_max_spd.msg.joints[joint - 1].min_angle_limit)


if __name__ == "__main__":
    unittest.main()
