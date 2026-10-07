"""Observed-width jaw preparation tests; real sockets are always forbidden."""
import copy
import errno
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, gripper_prepare, takeover
from robot_tools.backend import PyAgxBackend
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeRobot, TakeoverFixture


class FakeGripper:
    def __init__(self, robot):
        self.robot = robot

    def _send_msg(self, frame):
        self.robot._send_msg(frame)

    def move_gripper_m(self, *, value, force):
        frame = self.robot.can.Message(
            arbitration_id=0x159, is_extended_id=False,
            data=round(value * 1e6).to_bytes(4, "big", signed=True)
                 + round(force * 1e3).to_bytes(2, "big") + bytes((1, 0)))
        self._send_msg(self.robot.frame_transform(frame))
        if self.robot.duplicate:
            self._send_msg(frame)


class GripperRobot(FakeRobot):
    def __init__(self, side):
        super().__init__(side)
        self.ctrl_mode = 1
        self.driver_enabled = [True] * 6
        self.gripper_enabled = False
        self.width = 0.055 if side == "left" else 0.030
        self.gripper = FakeGripper(self)

    def _bus_send(self, frame):
        if self.send_error:
            raise self.send_error
        self.sent.append(frame)
        if self.accept:
            self.gripper_enabled = True


class GripperPrepareTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        self.robots = {side: GripperRobot(side) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def snapshot(self, robot, gripper):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        state = healthy_arm(self.clock.time())
        state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
            ctrl_mode=robot.ctrl_mode, mode_feedback=robot.mode_feedback,
            teach_status=0, motion_status=0, arm_status=0, err_code=0))
        for i, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = enabled
        state["gripper"].update(width_m=robot.width)
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None):
        return gripper_prepare.prepare_grippers(
            self.profile, journal or (lambda event, data: self.events.append((event, data))))

    def site_tolerance(self):
        self.profile["gripper_prepare_stationary_joint_tolerance_rad"] = {
            "right": [0.003, 0.003, 0.003, 0.008, 0.003, 0.003]}

    def test_site_right_j4_feedback_tolerance_preserves_actual_changes(self):
        self.site_tolerance()
        def hook(robot, state):
            if robot is self.robots["right"] and self.snapshot_count > 4:
                state["joints_rad"][3] += 0.006
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(result["observed_joint_changes_rad"]["right"][3], 0.006)
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertEqual(result["arm_target_commands_sent"], 0)

    def test_right_j5_cannot_use_j4_tolerance(self):
        self.site_tolerance()
        def hook(robot, state):
            if robot is self.robots["right"] and self.snapshot_count > 4:
                state["joints_rad"][4] += 0.006
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("J5", result["errors"][0]["detail"])

    def test_left_j4_cannot_use_right_j4_tolerance(self):
        self.site_tolerance()
        def hook(robot, state):
            if robot is self.robots["left"] and self.snapshot_count > 4:
                state["joints_rad"][3] += 0.006
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_stationary_tolerance_cannot_exceed_hard_ceiling(self):
        self.site_tolerance()
        self.profile["gripper_prepare_stationary_joint_tolerance_rad"]["right"][3] = 0.009
        with self.assertRaisesRegex(ValueError, "tolerances"):
            self.run_tool()
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_exact_one_159_per_disabled_jaw_no_arm_or_stop_frames(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["gripper_target_commands_sent"], 2)
        self.assertEqual(result["target_commands_sent"], 2)
        self.assertEqual(result["arm_target_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["gripper_enable_commands_sent"], 2)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertFalse(result["grasp_verified"])
        self.assertFalse(result["fold_ready"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertTrue(result["hold_not_validated"])
        self.assertTrue(PyAgxBackend(PROFILE).commissioning_errors())
        self.assertGreaterEqual(self.clock.elapsed, 5)
        for side, width_raw in (("left", 55000), ("right", 30000)):
            self.assertEqual(len(self.robots[side].sent), 1)
            frame = self.robots[side].sent[0]
            self.assertEqual(frame.arbitration_id, 0x159)
            self.assertEqual(bytes(frame.data), width_raw.to_bytes(4, "big") + bytes((0, 200, 1, 0)))
            self.assertFalse(frame.is_extended_id)
            self.assertFalse(frame.is_fd)
            self.assertIsNone(result["cleanup"]["arms"][side]["physically_stopped"])

    def test_already_enabled_is_passive_noop(self):
        for robot in self.robots.values():
            robot.gripper_enabled = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(result["gripper_enabled"], {"left": True, "right": True})
        self.assertTrue(all(v["status"] == "already_enabled_observed" for v in result["arms"].values()))

    def test_one_enabled_jaw_does_not_receive_new_target(self):
        self.robots["left"].gripper_enabled = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["left"].sent, [])

    def test_disabled_width_outside_range_or_nonfinite_refused(self):
        for width in (0.0, 0.004999, 0.070001, float("nan"), float("inf"), False):
            with self.subTest(width=width):
                self.robots["left"].width = width
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                self.assert_no_tx(self.run_tool())
                for robot in self.robots.values():
                    robot.disconnect.reset_mock()

    def test_first_width_is_retained_and_half_mm_presend_drift_refused(self):
        def journal(event, data):
            if event == "gripper_prepare_intent":
                self.robots["left"].width += 0.0006
        result = self.run_tool(journal)
        self.assert_no_tx(result)
        self.assertIn("0.5 mm", result["errors"][0]["detail"])
        self.assertEqual(result["targets_raw"]["left"], 55000)

    def test_disabled_joint_or_wrong_ctrl_or_fault_blocks_all_tx(self):
        for condition in ("joint_disabled", "ctrl_mode", "joint_fault", "jaw_fault", "jaw_unknown"):
            with self.subTest(condition=condition):
                def hook(robot, state):
                    if condition == "joint_disabled":
                        state["drivers"]["6"]["foc_status"]["driver_enable_status"] = False
                    elif condition == "ctrl_mode":
                        state["arm_status"]["ctrl_mode"] = 2
                    elif condition == "joint_fault":
                        state["drivers"]["1"]["foc_status"]["collision_status"] = True
                    elif condition == "jaw_fault":
                        state["gripper"]["foc_status"]["driver_error_status"] = True
                    else:
                        state["gripper"]["foc_status"]["driver_enable_status"] = 0
                self.hook = hook
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                self.assert_no_tx(self.run_tool())
                for robot in self.robots.values():
                    robot.disconnect.reset_mock()

    def test_swallowed_bus_exception_aborts_before_second_jaw(self):
        self.robots["left"].send_error = OSError(errno.ENOBUFS, "queue full")
        self.robots["left"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["transmission_counts"]["left"]["attempted_frames"], 1)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIn("CAN send failed", result["errors"][0]["detail"])

    def test_no_enable_feedback_times_out_without_retry_or_second_jaw(self):
        self.robots["left"].accept = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(len(self.robots["left"].sent), 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIn("did not confirm enabled", result["errors"][0]["detail"])

    def test_requested_jaw_enable_cannot_regress_after_seen_true(self):
        def hook(robot, state):
            if robot.side == "left" and robot.sent and self.clock.elapsed > 1.2:
                state["gripper"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("regressed", result["errors"][0]["detail"])
        self.assertEqual(self.robots["right"].sent, [])

    def test_other_jaw_cannot_enable_during_first_request(self):
        def hook(robot, state):
            if robot.side == "right" and self.robots["left"].sent:
                state["gripper"]["foc_status"]["driver_enable_status"] = True
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("outside its authorized", result["errors"][0]["detail"])
        self.assertEqual(self.robots["right"].sent, [])

    def test_enabled_other_jaw_cannot_drop_enable(self):
        self.robots["right"].gripper_enabled = True
        def hook(robot, state):
            if robot.side == "right" and self.robots["left"].sent:
                state["gripper"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("became disabled", result["errors"][0]["detail"])

    def test_postsend_width_must_be_within_one_mm_even_inside_anchor_limit(self):
        def hook(robot, state):
            if robot.side == "left" and robot.sent:
                state["gripper"]["width_m"] += 0.0012
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("within 1 mm", result["errors"][0]["detail"])
        self.assertEqual(self.robots["right"].sent, [])

    def test_postsend_joint_pose_jaw_and_mode_drift_each_blocks_second_jaw(self):
        for condition in ("joint", "pose", "jaw", "mode", "joint_enable"):
            with self.subTest(condition=condition):
                self.robots = {side: GripperRobot(side) for side in takeover.SIDES}
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                def hook(robot, state):
                    if robot.side == "right" and self.robots["left"].sent:
                        if condition == "joint":
                            state["joints_rad"][2] += 0.0031
                        elif condition == "pose":
                            state["pose_m_rad"][2] += 0.0021
                        elif condition == "jaw":
                            state["gripper"]["width_m"] += 0.0021
                        elif condition == "mode":
                            state["arm_status"]["mode_feedback"] = 1
                        else:
                            state["drivers"]["2"]["foc_status"]["driver_enable_status"] = False
                self.hook = hook
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(len(self.robots["left"].sent), 1)
                self.assertEqual(self.robots["right"].sent, [])

    def test_cumulative_drift_not_reanchored_between_grippers(self):
        def hook(robot, state):
            state["joints_rad"][0] = 0.002 * sum(bool(r.sent) for r in self.robots.values())
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertGreater(result["drift"]["left"]["joint_rad"], 0.003)

    def test_bad_id_width_force_enable_zeroing_and_fd_bytes_are_blocked(self):
        for field in ("id", "width", "force", "enable", "zero", "fd"):
            with self.subTest(field=field):
                self.robots = {side: GripperRobot(side) for side in takeover.SIDES}
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                def bad(frame):
                    if field == "id":
                        frame.arbitration_id = 0x471
                    elif field == "fd":
                        frame.is_fd = True
                    else:
                        frame.data[{"width": 3, "force": 5, "enable": 6, "zero": 7}[field]] ^= 1
                    return frame
                self.robots["left"].frame_transform = bad
                result = self.run_tool()
                self.assert_no_tx(result)
                self.assertEqual(result["transmission_counts"]["left"]["blocked_frames"], 1)

    def test_duplicate_sdk_request_cannot_reach_bus_twice(self):
        self.robots["left"].duplicate = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])

    def test_stale_or_nonadvancing_gripper_feedback_cannot_confirm(self):
        first_stamp = self.clock.time()
        def hook(robot, state):
            state["fragment_timestamps_s"]["gripper"] = first_stamp
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_fresh_precommand_feedback_cannot_confirm_after_transmission(self):
        before_send = None
        def journal(event, data):
            nonlocal before_send
            if event == "gripper_prepare_intent":
                before_send = self.clock.time()
        def hook(robot, state):
            if robot.side == "left" and robot.sent:
                state["fragment_timestamps_s"]["gripper"] = before_send
        self.hook = hook
        result = self.run_tool(journal)
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIn("fragment_skew", result["errors"][0]["detail"])

    def test_delayed_enable_feedback_is_allowed_once_without_retry(self):
        self.robots["left"].accept = False
        def hook(robot, state):
            if robot.side == "left" and robot.sent and self.clock.elapsed > 1.4:
                state["gripper"]["foc_status"]["driver_enable_status"] = True
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)

    def test_initialization_cannot_emit_gripper_or_arm_frames(self):
        def connect():
            self.robots["left"].gripper.move_gripper_m(value=0.055, force=0.2)
        self.robots["left"].connect = connect
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("Initialization TX", result["errors"][0]["detail"])

    def test_journal_intent_failure_prevents_any_send(self):
        def journal(event, data):
            if event == "gripper_prepare_intent":
                raise OSError("disk full")
        result = self.run_tool(journal)
        self.assert_no_tx(result)


class RealSDKGripperPrepareTests(unittest.TestCase):
    def test_manufacturer_effector_encoder_and_enums_with_mocked_can_only(self):
        self.run_vendor_case(send_error=False)

    def test_manufacturer_sdk_send_error_is_not_hidden_or_retried(self):
        self.run_vendor_case(send_error=True)

    def run_vendor_case(self, *, send_error):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, robots = Clock(), [], {}

        class FakeBus:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(0.002)
                return None
            def send(self, frame, timeout=None):
                if send_error:
                    raise OSError(errno.ENOBUFS, "mock queue full")
                sent.append((self.channel, frame))
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
            channel = next(channel for channel, value in robots.items() if value is robot)
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=1, teach_status=0, mode_feedback=0,
                motion_status=0, arm_status=0, err_code=0))
            state["gripper"]["width_m"] = 0.055321 if channel == "can0" else 0.023456
            state["gripper"]["foc_status"]["driver_enable_status"] = any(ch == channel for ch, _ in sent)
            return state

        with patch("socket.socket", side_effect=AssertionError("No real socket")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = gripper_prepare.prepare_grippers(profile, lambda event, data: None)
        if send_error:
            self.assertFalse(result["ok"], result)
            self.assertEqual(sent, [])
            self.assertEqual(result["transmission_counts"]["left"]["attempted_frames"], 1)
            self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 0)
            self.assertEqual(result["hardware_commands_sent"], 0)
            self.assertIn("CAN send failed", result["errors"][0]["detail"])
            return
        self.assertTrue(result["ok"], result)
        self.assertEqual(len(sent), 2)
        for (channel, frame), expected_channel, width_raw in zip(sent, ("can0", "can2"), (55321, 23456)):
            self.assertEqual(channel, expected_channel)
            self.assertEqual(frame.arbitration_id, 0x159)
            self.assertEqual(bytes(frame.data), width_raw.to_bytes(4, "big") + bytes((0, 200, 1, 0)))
            self.assertEqual(frame.dlc, 8)
            self.assertFalse(frame.is_extended_id)
            self.assertFalse(frame.is_fd)


if __name__ == "__main__":
    unittest.main()
